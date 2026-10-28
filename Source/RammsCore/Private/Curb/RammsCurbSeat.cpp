// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbSeat.h"

#include "Ramms5BarKinematics.h"

namespace RammsCurb
{
	namespace
	{
		FVector2D KneeA(const FRamms5BarLinkageSpec& L, double AngleA)
		{
			const double Theta = L.ZeroDirA + L.AngleSignA * AngleA;
			return L.PivotA + FVector2D(L.ProximalLengthA * FMath::Cos(Theta), L.ProximalLengthA * FMath::Sin(Theta));
		}
	} // namespace

	FSeatPose SeatForward(const FRammsCurbSeat& Seat, FVector2D AnglesAB)
	{
		FSeatPose		Out;
		bool			bValid = false;
		const FVector2D Q = URamms5BarKinematics::ComputeEndpoint(Seat.Linkage, AnglesAB, bValid);
		const FVector2D P = KneeA(Seat.Linkage, AnglesAB.X);
		const FVector2D U = (Q - P).GetSafeNormal();
		Out.bValid = bValid && !U.IsNearlyZero();
		Out.LinkAngle = FMath::Atan2(U.Y, U.X);
		Out.Pivot = Q + U * Seat.PivotBeyondEndpointCm;
		return Out;
	}

	bool SeatSolve(const FRammsCurbSeat& Seat, FVector2D Target, FVector2D Hint, FVector2D& OutAngles, FSeatPose* OutPose)
	{
		// Newton on the forward kinematics from the hint: the pivot is a smooth
		// function of the two motor angles on one assembly branch, and the
		// branch the hint is on is the mechanism's own.
		constexpr double H = 1e-4;
		FVector2D		 Q = Hint;
		FSeatPose		 Pose;
		bool			 bConverged = false;
		for (int32 Iter = 0; Iter < 40; ++Iter)
		{
			Pose = SeatForward(Seat, Q);
			if (!Pose.bValid)
			{
				break;
			}
			const FVector2D F = Pose.Pivot - Target;
			if (F.Size() < 1e-5)
			{
				bConverged = true;
				break;
			}
			const FVector2D Fa = SeatForward(Seat, Q + FVector2D(H, 0.0)).Pivot - Pose.Pivot;
			const FVector2D Fb = SeatForward(Seat, Q + FVector2D(0.0, H)).Pivot - Pose.Pivot;
			// J = [Fa Fb] / H; solve J d = -F.
			const double J00 = Fa.X / H, J10 = Fa.Y / H, J01 = Fb.X / H, J11 = Fb.Y / H;
			const double Det = J00 * J11 - J01 * J10;
			if (FMath::Abs(Det) < 1e-9)
			{
				break;
			}
			FVector2D D(-(J11 * F.X - J01 * F.Y) / Det, -(-J10 * F.X + J00 * F.Y) / Det);
			// No step over half a radian: a far target is walked to, not jumped
			// at, so the branch is kept.
			const double Most = 0.5;
			if (D.Size() > Most)
			{
				D *= Most / D.Size();
			}
			Q += D;
		}
		OutAngles = Q;
		if (OutPose)
		{
			*OutPose = Pose;
		}
		if (!bConverged || !Pose.bValid)
		{
			return false;
		}
		// The endpoint the 5-bar solver would give for this pivot's endpoint:
		// the same angles, and within the knee ranges.
		bool			bReachable = false;
		const FVector2D Endpoint = Pose.Pivot - (Pose.Pivot - KneeA(Seat.Linkage, Q.X)).GetSafeNormal() * Seat.PivotBeyondEndpointCm;
		const FVector2D Angles = URamms5BarKinematics::SolveIK(Seat.Linkage, Endpoint, bReachable);
		if (!bReachable)
		{
			return false;
		}
		return FMath::Abs(FMath::FindDeltaAngleRadians(Angles.X, Q.X)) < 1e-3 && FMath::Abs(FMath::FindDeltaAngleRadians(Angles.Y, Q.Y)) < 1e-3;
	}

	double SeatPlatformPitch(const FRammsCurbSeat& Seat, double LinkAngle, double LinkAngle0, double TiltAngle)
	{
		return FMath::FindDeltaAngleRadians(LinkAngle0, LinkAngle) + (Seat.TiltMotor.IsNone() ? 0.0 : Seat.TiltAngleSign * TiltAngle);
	}

	double SeatTiltFor(const FRammsCurbSeat& Seat, double LinkAngle, double LinkAngle0, double PitchRad)
	{
		const double Sign = Seat.TiltAngleSign >= 0.0f ? 1.0 : -1.0;
		return (PitchRad - FMath::FindDeltaAngleRadians(LinkAngle0, LinkAngle)) / Sign;
	}

	FVector2D SeatReachX(const FRammsCurbSeat& Seat, double Z, FVector2D Hint, FVector2D RangeA, FVector2D RangeB, bool& bValid,
		double StepCm, double SpanCm)
	{
		auto Within = [](double V, const FVector2D& R) { return R.X >= R.Y || (V >= R.X - 1e-3 && V <= R.Y + 1e-3); }; // a solve lands within 1e-5 cm: a hair of angle
		auto Reaches = [&](double X, FVector2D& InOutHint) {
			FVector2D Angles;
			if (!SeatSolve(Seat, FVector2D(X, Z), InOutHint, Angles) || !Within(Angles.X, RangeA) || !Within(Angles.Y, RangeB))
			{
				return false;
			}
			InOutHint = Angles;
			return true;
		};
		const FSeatPose At0 = SeatForward(Seat, Hint);
		FVector2D		H0 = Hint;
		bValid = At0.bValid && Reaches(At0.Pivot.X, H0);
		if (!bValid)
		{
			return FVector2D(1.0, 0.0);
		}
		FVector2D Out(At0.Pivot.X, At0.Pivot.X);
		FVector2D H = H0;
		for (double X = At0.Pivot.X + StepCm; X <= At0.Pivot.X + SpanCm && Reaches(X, H); X += StepCm)
		{
			Out.Y = X;
		}
		H = H0;
		for (double X = At0.Pivot.X - StepCm; X >= At0.Pivot.X - SpanCm && Reaches(X, H); X -= StepCm)
		{
			Out.X = X;
		}
		return Out;
	}
} // namespace RammsCurb
