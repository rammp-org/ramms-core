// Copyright Epic Games, Inc. All Rights Reserved.

#include "Ramms5BarKinematics.h"

namespace
{
	// AngleSignA/B are documented as ±1. Anything else (0, NaN, a stray 0.5)
	// would divide by zero in the IK or silently rescale angles, so both public
	// entry points validate and normalise them before touching the maths.
	bool NormalizeSign(double In, double& Out)
	{
		if (!FMath::IsFinite(In) || FMath::Abs(In) < KINDA_SMALL_NUMBER)
		{
			return false;
		}
		Out = In > 0.0 ? 1.0 : -1.0;
		return true;
	}

	bool NormalizeSigns(const FRamms5BarLinkageSpec& Spec, double& SignA, double& SignB)
	{
		const bool bOk = NormalizeSign(Spec.AngleSignA, SignA) & NormalizeSign(Spec.AngleSignB, SignB);
		if (!bOk)
		{
			static bool bWarned = false;
			if (!bWarned)
			{
				bWarned = true;
				UE_LOG(LogTemp, Error, TEXT("Ramms5BarKinematics: linkage (motors '%s' / '%s') has AngleSignA/B = %g / %g; they must be +1 or -1. Targets are reported unreachable until the table is fixed."),
					*Spec.ProximalMotorA.ToString(), *Spec.ProximalMotorB.ToString(), Spec.AngleSignA, Spec.AngleSignB);
			}
		}
		return bOk;
	}
} // namespace

FVector2D URamms5BarKinematics::KneePosition(
	FVector2D Pivot, double ProximalLen, double ZeroDir, double Sign, double JointAngle)
{
	const double LinkAngle = ZeroDir + Sign * JointAngle;
	return Pivot + FVector2D(ProximalLen * FMath::Cos(LinkAngle), ProximalLen * FMath::Sin(LinkAngle));
}

double URamms5BarKinematics::SolveArm(
	FVector2D Pivot, double ProximalLen, double DistalLen,
	double ZeroDir, double Sign, bool bElbowUp, FVector2D Target, bool& bReachable)
{
	const FVector2D ToTarget = Target - Pivot;
	const double	Dist = ToTarget.Size();
	const double	Base = FMath::Atan2(ToTarget.Y, ToTarget.X);

	bReachable = true;

	// Degenerate: target on the pivot — cannot define a direction.
	if (Dist < KINDA_SMALL_NUMBER)
	{
		bReachable = false;
		return (0.0 - ZeroDir) / Sign;
	}

	// Interior angle at the pivot between (pivot->target) and the proximal link.
	double CosInterior = (ProximalLen * ProximalLen + Dist * Dist - DistalLen * DistalLen)
		/ (2.0 * ProximalLen * Dist);
	if (CosInterior < -1.0 || CosInterior > 1.0)
	{
		// Out of reach (too far or too close): clamp so the arm points as close
		// as it can — straight at the target (far) or folded back (near).
		bReachable = false;
		CosInterior = FMath::Clamp(CosInterior, -1.0, 1.0);
	}
	const double Interior = FMath::Acos(CosInterior);

	const double LinkAngle = Base + (bElbowUp ? Interior : -Interior);
	// Joint angles are reported unwrapped near zero by the physics backends, so
	// return the principal value rather than an aliased multiple-of-2π branch.
	return FMath::UnwindRadians((LinkAngle - ZeroDir) / Sign);
}

FVector2D URamms5BarKinematics::SolveIK(const FRamms5BarLinkageSpec& Spec, FVector2D TargetXZ, bool& bReachable)
{
	double SignA = 1.0, SignB = 1.0;
	if (!NormalizeSigns(Spec, SignA, SignB))
	{
		bReachable = false;
		return FVector2D::ZeroVector;
	}
	bool			bReachA = false;
	bool			bReachB = false;
	const double	AngleA = SolveArm(Spec.PivotA, Spec.ProximalLengthA, Spec.DistalLengthA,
		Spec.ZeroDirA, SignA, Spec.bElbowUpA, TargetXZ, bReachA);
	const double	AngleB = SolveArm(Spec.PivotB, Spec.ProximalLengthB, Spec.DistalLengthB,
		Spec.ZeroDirB, SignB, Spec.bElbowUpB, TargetXZ, bReachB);
	const FVector2D Angles(AngleA, AngleB);
	bReachable = bReachA && bReachB;
	if (bReachable)
	{
		// Each arm's annulus test is necessary, not sufficient: the pair can
		// belong to the mirrored assembly (the other side of the knee-to-knee
		// line) or need a knee past straight. Only a pose the closed loop can
		// hold, that actually lands on the target, is reachable.
		bool			bValid = false;
		const FVector2D Endpoint = ComputeEndpoint(Spec, Angles, bValid);
		const double	Tolerance = 1e-4 * (Spec.ProximalLengthA + Spec.DistalLengthA + Spec.ProximalLengthB + Spec.DistalLengthB);
		bReachable = bValid && (Endpoint - TargetXZ).Size() <= Tolerance;
	}
	return Angles;
}

FVector2D URamms5BarKinematics::ComputeEndpoint(const FRamms5BarLinkageSpec& Spec, FVector2D JointAnglesAB, bool& bValid)
{
	double SignA = 1.0, SignB = 1.0;
	if (!NormalizeSigns(Spec, SignA, SignB))
	{
		bValid = false;
		return Spec.PivotA;
	}
	const FVector2D KneeA = KneePosition(Spec.PivotA, Spec.ProximalLengthA, Spec.ZeroDirA, SignA, JointAnglesAB.X);
	const FVector2D KneeB = KneePosition(Spec.PivotB, Spec.ProximalLengthB, Spec.ZeroDirB, SignB, JointAnglesAB.Y);

	// Endpoint = intersection of circle(KneeA, DistalA) and circle(KneeB, DistalB).
	const FVector2D Delta = KneeB - KneeA;
	const double	CenterDist = Delta.Size();
	const double	Ra = Spec.DistalLengthA;
	const double	Rb = Spec.DistalLengthB;

	bValid = true;
	if (CenterDist < KINDA_SMALL_NUMBER || CenterDist > Ra + Rb || CenterDist < FMath::Abs(Ra - Rb))
	{
		// Distal links can't meet: return the midpoint of the two circles' closest
		// points, which lie on the knee-to-knee line. T is the parameter along
		// Delta (0 = KneeA, 1 = KneeB).
		bValid = false;
		if (CenterDist < KINDA_SMALL_NUMBER)
		{
			return KneeA;
		}
		double T;
		if (CenterDist > Ra + Rb)
		{
			// Disjoint: A's point at Ra towards B, B's point at Rb towards A.
			T = (Ra + CenterDist - Rb) / (2.0 * CenterDist);
		}
		else if (Ra >= Rb)
		{
			// B's circle inside A's: both points beyond KneeB, away from A.
			T = (Ra + CenterDist + Rb) / (2.0 * CenterDist);
		}
		else
		{
			// A's circle inside B's: both points behind KneeA, away from B.
			T = (CenterDist - Ra - Rb) / (2.0 * CenterDist);
		}
		return KneeA + Delta * T;
	}

	const double	A = (Ra * Ra - Rb * Rb + CenterDist * CenterDist) / (2.0 * CenterDist);
	const double	H = FMath::Sqrt(FMath::Max(0.0, Ra * Ra - A * A));
	const FVector2D Mid = KneeA + Delta * (A / CenterDist);
	// Perpendicular to the knee-to-knee line.
	const FVector2D Perp = FVector2D(-Delta.Y, Delta.X) / CenterDist;

	// The two intersections are mirror images across the knee-to-knee line, and
	// the assembled mechanism stays on one side of it (crossing needs the distal
	// links collinear). Pick by fixed chirality — the sign of
	// cross(KneeB - KneeA, Endpoint - KneeA) — rather than by height, which
	// would flip the answer when the knees tilt far enough.
	const FVector2D Endpoint = Spec.bFlipEndpointSide ? (Mid - Perp * H) : (Mid + Perp * H);

	// Feasibility: each knee bends one way only (the passive knee joints have a
	// hard stop at straight), so a hip pair whose geometric solution needs a
	// knee bent past straight is not a pose the closed loop can hold. That
	// bend direction is exactly the IK elbow branch: elbow-up ⇔
	// cross(proximal dir, distal dir) < 0.
	auto KneeBendMatches = [&Endpoint](FVector2D Pivot, FVector2D Knee, bool bElbowUp) {
		const FVector2D Prox = Knee - Pivot;
		const FVector2D Dist = Endpoint - Knee;
		const double	Cross = Prox.X * Dist.Y - Prox.Y * Dist.X;
		// A straight knee (the reach boundary IK treats as reachable) is valid
		// for either bend direction; tolerance scales with the link lengths
		// since the cross product is an area.
		const double StraightTolerance = 1e-6 * Prox.Size() * Dist.Size();
		if (FMath::Abs(Cross) <= StraightTolerance)
		{
			return true;
		}
		return (Cross < 0.0) == bElbowUp;
	};
	if (!KneeBendMatches(Spec.PivotA, KneeA, Spec.bElbowUpA) || !KneeBendMatches(Spec.PivotB, KneeB, Spec.bElbowUpB))
	{
		bValid = false;
	}
	return Endpoint;
}
