// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbRobotModel.h"

#include "Algo/Reverse.h"
#include "Curb/RammsCurbTerrainFit.h"
#include "Engine/DataTable.h"
#include "Ramms4BarKinematics.h"
#include "Ramms5BarKinematics.h"
#include "Ramms5BarLinkageController.h"

namespace RammsCurb
{
	// --- 4-bar table ---------------------------------------------------------------

	bool FFourBarTable::Build(const FRamms4BarLegSpec& Spec, int32 Samples)
	{
		Z.Reset();
		X.Reset();
		Crank.Reset();
		Samples = FMath::Max(2, Samples);
		TArray<double> Zs, Xs, Cs;
		for (int32 s = 0; s <= Samples; ++s)
		{
			const double	Angle = FMath::Lerp(static_cast<double>(Spec.CrankRange.X), static_cast<double>(Spec.CrankRange.Y), double(s) / Samples);
			bool			bValid = false;
			const FVector2D W = URamms4BarKinematics::ComputeWheelCenter(Spec, static_cast<float>(Angle), bValid);
			if (!bValid)
			{
				return false;
			}
			Zs.Add(W.Y);
			Xs.Add(W.X);
			Cs.Add(Angle);
		}
		// One way only: rising or falling with the crank, never back.
		const double Trend = Zs.Last() >= Zs[0] ? 1.0 : -1.0;
		for (int32 s = 1; s < Zs.Num(); ++s)
		{
			if ((Zs[s] - Zs[s - 1]) * Trend < -1e-6)
			{
				return false;
			}
		}
		if (Trend < 0.0)
		{
			Algo::Reverse(Zs);
			Algo::Reverse(Xs);
			Algo::Reverse(Cs);
		}
		Z = MoveTemp(Zs);
		X = MoveTemp(Xs);
		Crank = MoveTemp(Cs);
		return true;
	}

	FVector2D FFourBarTable::HeightRange() const
	{
		return IsValid() ? FVector2D(Z[0], Z.Last()) : FVector2D(1.0, 0.0);
	}

	double FFourBarTable::Interpolate(const TArray<double>& Values, double Height) const
	{
		if (!IsValid())
		{
			return 0.0;
		}
		if (Height <= Z[0])
		{
			return Values[0];
		}
		if (Height >= Z.Last())
		{
			return Values.Last();
		}
		// The first sample above Height; heights that repeat (a flat stretch)
		// interpolate over the one span that rises.
		int32 Lo = 0, Hi = Z.Num() - 1;
		while (Hi - Lo > 1)
		{
			const int32 Mid = (Lo + Hi) / 2;
			(Z[Mid] <= Height ? Lo : Hi) = Mid;
		}
		const double Span = Z[Hi] - Z[Lo];
		const double T = Span > 1e-12 ? (Height - Z[Lo]) / Span : 0.0;
		return FMath::Lerp(Values[Lo], Values[Hi], T);
	}

	double FFourBarTable::XAt(double Height) const
	{
		return Interpolate(X, Height);
	}

	double FFourBarTable::CrankAt(double Height) const
	{
		return Interpolate(Crank, Height);
	}

	// --- 5-bar grid ----------------------------------------------------------------

	void FFiveBarGrid::Build(TFunctionRef<bool(double X, double Z)> Reachable, const FVector2D& XRange, const FVector2D& ZRange,
		double InXStep, double InZStep)
	{
		XStep = FMath::Max(InXStep, 1e-3);
		ZStep = FMath::Max(InZStep, 1e-3);
		Origin = FVector2D(XRange.X, ZRange.X);
		NumX = XRange.Y >= XRange.X ? FMath::FloorToInt((XRange.Y - XRange.X) / XStep + 1e-6) + 1 : 0;
		NumZ = ZRange.Y >= ZRange.X ? FMath::FloorToInt((ZRange.Y - ZRange.X) / ZStep + 1e-6) + 1 : 0;
		Cells.Init(false, NumX * NumZ);
		for (int32 J = 0; J < NumZ; ++J)
		{
			for (int32 I = 0; I < NumX; ++I)
			{
				Cells[J * NumX + I] = Reachable(Origin.X + I * XStep, Origin.Y + J * ZStep);
			}
		}
	}

	bool FFiveBarGrid::Contains(double X, double Z) const
	{
		if (!IsValid())
		{
			return false;
		}
		const int32 I = FMath::RoundToInt((X - Origin.X) / XStep);
		const int32 J = FMath::RoundToInt((Z - Origin.Y) / ZStep);
		return I >= 0 && I < NumX && J >= 0 && J < NumZ && Cell(I, J);
	}

	FVector2D FFiveBarGrid::HeightRangeAt(double X) const
	{
		FVector2D	Out(1.0, 0.0);
		const int32 I = IsValid() ? FMath::RoundToInt((X - Origin.X) / XStep) : -1;
		if (I < 0 || I >= NumX)
		{
			return Out;
		}
		bool bAny = false;
		for (int32 J = 0; J < NumZ; ++J)
		{
			if (Cell(I, J))
			{
				const double Z = Origin.Y + J * ZStep;
				Out.X = bAny ? FMath::Min(Out.X, Z) : Z;
				Out.Y = bAny ? FMath::Max(Out.Y, Z) : Z;
				bAny = true;
			}
		}
		return Out;
	}

	FVector2D FFiveBarGrid::XRangeAt(double Z) const
	{
		FVector2D	Out(1.0, 0.0);
		const int32 J = IsValid() ? FMath::RoundToInt((Z - Origin.Y) / ZStep) : -1;
		if (J < 0 || J >= NumZ)
		{
			return Out;
		}
		bool bAny = false;
		for (int32 I = 0; I < NumX; ++I)
		{
			if (Cell(I, J))
			{
				const double X = Origin.X + I * XStep;
				Out.X = bAny ? FMath::Min(Out.X, X) : X;
				Out.Y = bAny ? FMath::Max(Out.Y, X) : X;
				bAny = true;
			}
		}
		return Out;
	}

	int32 FFiveBarGrid::CountReachable() const
	{
		int32 Count = 0;
		for (TConstSetBitIterator<> It(Cells); It; ++It)
		{
			++Count;
		}
		return Count;
	}

	// --- the robot -------------------------------------------------------------------

	FVector FRobotModel::CenterOfMassAt(TConstArrayView<FVector2D> WheelXZ) const
	{
		FVector C = CenterOfMass;
		if (MassKg <= 0.0)
		{
			return C;
		}
		for (int32 i = 0; i < Legs.Num() && i < WheelXZ.Num(); ++i)
		{
			const double K = Legs[i].MovingMassKg / MassKg;
			C.X += K * (WheelXZ[i].X - Legs[i].ZeroPose.X);
			C.Z += K * (WheelXZ[i].Y - Legs[i].ZeroPose.Y);
		}
		return C;
	}

	void FRobotModel::BellyPoints(TArray<FVector2D>& Out, int32 Along, int32 Across) const
	{
		Out.Reset();
		Along = FMath::Max(2, Along);
		Across = FMath::Max(2, Across);
		for (int32 b = 0; b < Across; ++b)
		{
			const double Y = FMath::Lerp(-BellyHalfExtent.Y, BellyHalfExtent.Y, double(b) / (Across - 1));
			for (int32 a = 0; a < Along; ++a)
			{
				Out.Add(FVector2D(FMath::Lerp(-BellyHalfExtent.X, BellyHalfExtent.X, double(a) / (Along - 1)), Y));
			}
		}
	}

	FVector TunedCenterOfMass(const FRammsCurbTuning& Tuning)
	{
		FVector C = Tuning.CenterOfMass;
		if (C.X == 0.0)
		{
			C.X = Tuning.CenterOfMassX;
		}
		return C;
	}

	FVector2D BellyHalfExtentOf(TConstArrayView<FRammsCurbLeg> Legs, const FRammsCurbTuning& Tuning)
	{
		FVector2D Half = Tuning.ChassisHalfExtent;
		if (Half.X <= 0.0 || Half.Y <= 0.0)
		{
			double X = 0.0, Y = 0.0;
			for (const FRammsCurbLeg& Leg : Legs)
			{
				if (Leg.Kind == ERammsCurbLegKind::FourBar && Leg.Axle != ERammsCurbAxle::Center)
				{
					X = FMath::Max(X, FMath::Abs(static_cast<double>(Leg.FourBar.CrankPivot.X)));
					Y = FMath::Max(Y, FMath::Abs(static_cast<double>(Leg.Wheel.Position.Y)));
				}
			}
			Half = FVector2D(Half.X > 0.0 ? Half.X : X, Half.Y > 0.0 ? Half.Y : Y);
		}
		return Half;
	}

	bool FiveBarZeroPose(const URamms5BarLinkageController& FiveBar, FVector2D& Out)
	{
		// The spec the controller resolves at BeginPlay: its table's row when it
		// names one that is there, else its inline one.
		FRamms5BarLinkageSpec Spec = FiveBar.Linkage;
		if (FiveBar.KinematicTable && !FiveBar.LinkageRow.IsNone())
		{
			static const FString Context(TEXT("RammsCurbRobotModel"));
			if (const FRamms5BarLinkageSpec* Row = FiveBar.KinematicTable->FindRow<FRamms5BarLinkageSpec>(FiveBar.LinkageRow, Context, false))
			{
				Spec = *Row;
			}
		}
		bool			bValid = false;
		const FVector2D Local = URamms5BarKinematics::ComputeEndpoint(Spec, FVector2D::ZeroVector, bValid);
		// The controller's robot frame mirrors fore/aft on a linkage whose
		// TranslationSign is negative (URamms5BarLinkageController::ToRobot).
		Out = FVector2D(Local.X * (Spec.TranslationSign < 0.0f ? -1.0 : 1.0), Local.Y);
		return bValid;
	}

	bool BuildRobotModel(TConstArrayView<FRammsCurbLeg> Legs, const FRammsCurbTuning& Tuning,
		TConstArrayView<FVector2D> FiveBarZeroPoses, FRobotModel& Out, FString& OutReason)
	{
		Out = FRobotModel();
		Out.CenterOfMass = TunedCenterOfMass(Tuning);
		Out.MassKg = Tuning.RobotMassKg;
		Out.BellyHalfExtent = BellyHalfExtentOf(Legs, Tuning);
		Out.BellyUndersideCm = Tuning.ChassisUnderside;
		if (!FMath::IsFinite(Tuning.RobotMassKg) || Tuning.RobotMassKg < 0.0f || !FMath::IsFinite(Out.CenterOfMass.X)
			|| !FMath::IsFinite(Out.CenterOfMass.Y) || !FMath::IsFinite(Out.CenterOfMass.Z))
		{
			OutReason = TEXT("the tuning's RobotMassKg or CenterOfMass is not a number, or the mass is negative");
			return false;
		}
		double Moving = 0.0;
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			const FRammsCurbLeg& Leg = Legs[i];
			FModelLeg&			 M = Out.Legs.AddDefaulted_GetRef();
			M.Kind = Leg.Kind;
			M.Axle = Leg.Axle;
			M.Y = Leg.Wheel.Position.Y;
			M.RadiusCm = Leg.WheelRadiusCm;
			M.ContactWidthCm = ContactWidth(Leg.TyreWidthCm, Leg.ContactWidthCm);
			M.MovingMassKg = Leg.MovingMassKg;
			const double Theta = FMath::DegreesToRadians(static_cast<double>(Leg.Wheel.RollDirectionDeg));
			M.RollDirection = FVector2D(FMath::Cos(Theta), FMath::Sin(Theta));
			if (!FMath::IsFinite(Leg.MovingMassKg) || Leg.MovingMassKg < 0.0f)
			{
				OutReason = FString::Printf(TEXT("leg '%s': its MovingMassKg is negative or not a number"), *Leg.Name.ToString());
				return false;
			}
			Moving += Leg.MovingMassKg;
			if (Leg.Kind == ERammsCurbLegKind::FourBar)
			{
				if (!M.FourBar.Build(Leg.FourBar))
				{
					OutReason = FString::Printf(TEXT("leg '%s': its 4-bar does not close across its crank range, or its wheel height "
													 "is not monotonic in the crank"),
						*Leg.Name.ToString());
					return false;
				}
				bool			bValid = false;
				const FVector2D Zero = URamms4BarKinematics::ComputeWheelCenter(Leg.FourBar, 0.0f, bValid);
				if (!bValid && Leg.MovingMassKg > 0.0f && Tuning.RobotMassKg > 0.0f)
				{
					OutReason = FString::Printf(TEXT("leg '%s': its 4-bar does not close at crank 0, the zero pose its MovingMassKg is "
													 "reckoned from"),
						*Leg.Name.ToString());
					return false;
				}
				// A leg that carries no mass needs no zero pose.
				M.ZeroPose = bValid ? Zero : FVector2D::ZeroVector;
			}
			else
			{
				M.ZeroPose = FiveBarZeroPoses.IsValidIndex(i) ? FiveBarZeroPoses[i] : FVector2D::ZeroVector;
			}
		}
		if (Tuning.RobotMassKg > 0.0f && Moving >= Tuning.RobotMassKg)
		{
			OutReason = FString::Printf(TEXT("the legs' moving masses add up to %.1f kg, as much as the whole robot (RobotMassKg %.1f)"),
				Moving, Tuning.RobotMassKg);
			return false;
		}
		return true;
	}

	// --- stability -------------------------------------------------------------------

	namespace
	{
		double Cross(const FVector2D& O, const FVector2D& A, const FVector2D& B)
		{
			return (A.X - O.X) * (B.Y - O.Y) - (A.Y - O.Y) * (B.X - O.X);
		}

		/** P's distance from the segment A-B. */
		double SegmentDistance(const FVector2D& A, const FVector2D& B, const FVector2D& P)
		{
			const FVector2D E = B - A;
			const double	L2 = E.SizeSquared();
			const double	T = L2 > 1e-18 ? FMath::Clamp(FVector2D::DotProduct(P - A, E) / L2, 0.0, 1.0) : 0.0;
			return (P - (A + E * T)).Size();
		}
	} // namespace

	double HullMargin(TConstArrayView<FVector2D> Points, const FVector2D& P)
	{
		// Andrew's monotone chain, counter-clockwise, collinear points dropped.
		TArray<FVector2D, TInlineAllocator<16>> Sorted(Points.GetData(), Points.Num());
		Sorted.Sort([](const FVector2D& A, const FVector2D& B) { return A.X < B.X || (A.X == B.X && A.Y < B.Y); });
		TArray<FVector2D, TInlineAllocator<16>> Unique;
		for (const FVector2D& Q : Sorted)
		{
			if (Unique.Num() == 0 || (Q - Unique.Last()).SizeSquared() > 1e-12)
			{
				Unique.Add(Q);
			}
		}
		if (Unique.Num() == 0)
		{
			return TNumericLimits<double>::Lowest();
		}
		TArray<FVector2D, TInlineAllocator<16>> Hull;
		for (int32 Pass = 0; Pass < 2; ++Pass)
		{
			const int32 Start = Hull.Num();
			for (int32 k = 0; k < Unique.Num(); ++k)
			{
				const FVector2D& Q = Unique[Pass == 0 ? k : Unique.Num() - 1 - k];
				while (Hull.Num() >= Start + 2 && Cross(Hull[Hull.Num() - 2], Hull.Last(), Q) <= 1e-12)
				{
					Hull.Pop();
				}
				Hull.Add(Q);
			}
			Hull.Pop(); // each chain's last point starts the other
		}
		if (Hull.Num() < 3)
		{
			// A segment or a point: no area to stand inside.
			if (Unique.Num() == 1)
			{
				return -(P - Unique[0]).Size();
			}
			return -SegmentDistance(Unique[0], Unique.Last(), P);
		}
		double Margin = TNumericLimits<double>::Max();
		for (int32 k = 0; k < Hull.Num(); ++k)
		{
			const FVector2D& A = Hull[k];
			const FVector2D& B = Hull[(k + 1) % Hull.Num()];
			const FVector2D	 E = B - A;
			const double	 Length = E.Size();
			if (Length < 1e-9)
			{
				continue;
			}
			// The left normal points inside a counter-clockwise hull.
			const FVector2D N(-E.Y / Length, E.X / Length);
			Margin = FMath::Min(Margin, FVector2D::DotProduct(P - A, N));
		}
		return Margin;
	}

	double ZmpShiftCm(double HeightCm, double AccelCmS2)
	{
		return HeightCm * AccelCmS2 / GravityCmS2;
	}

	double DynamicMargin(TConstArrayView<FVector2D> Points, const FVector2D& ComXY, double HeightCm, const FVector2D& Heading,
		double Acceleration, double Deceleration)
	{
		const FVector2D H = Heading.GetSafeNormal();
		const double	SpeedingUp = HullMargin(Points, ComXY - H * ZmpShiftCm(HeightCm, Acceleration));
		const double	Braking = HullMargin(Points, ComXY + H * ZmpShiftCm(HeightCm, Deceleration));
		return FMath::Min(SpeedingUp, Braking);
	}

	FVector WheelContact(const FVector& Centre, const FVector& Axle, const FVector2D& Gradient, double RadiusCm, double ContactWidthCm)
	{
		// The plane's upward normal, and the axle's lean against it.
		const FVector N = FVector(-Gradient.X, -Gradient.Y, 1.0).GetSafeNormal();
		const FVector A = Axle.GetSafeNormal();
		const double  Lean = FVector::DotProduct(N, A);
		// The disc's point lowest against the plane: down the normal's part in
		// the wheel's own plane. An axle square to the plane (the wheel lying
		// flat) has none: its middle.
		const FVector InPlane = N - A * Lean;
		const double  Length = InPlane.Size();
		const FVector Down = Length > 1e-9 ? InPlane / Length : FVector::ZeroVector;
		// The rim nearer the plane: against the normal along the axle.
		const double Ramp = FMath::Sin(FMath::DegreesToRadians(RimContactRampDeg));
		const double Along = -0.5 * FMath::Max(0.0, ContactWidthCm) * FMath::Clamp(Lean / Ramp, -1.0, 1.0);
		return Centre - Down * RadiusCm + A * Along;
	}

	namespace
	{
		/** A wheel's axle in its chassis' frame: level, square to its roll
		 *  direction. */
		FVector LevelAxle(const FModelLeg& Leg)
		{
			const FVector2D F = Leg.RollDirection.GetSafeNormal();
			return FVector(-F.Y, F.X, 0.0);
		}
	} // namespace

	FPoseMargin PoseMargin(const FRobotModel& Model, TConstArrayView<FPoseLeg> Pose, double Acceleration, double Deceleration)
	{
		FPoseMargin							   Out;
		TArray<FVector2D, TInlineAllocator<8>> XZ;
		TArray<FVector2D, TInlineAllocator<8>> Contacts;
		double								   Ground = 0.0;
		for (int32 i = 0; i < Model.Legs.Num(); ++i)
		{
			const FPoseLeg* Leg = Pose.IsValidIndex(i) ? &Pose[i] : nullptr;
			XZ.Add(Leg ? Leg->XZ : Model.Legs[i].ZeroPose);
			if (!Leg || !Leg->bSupport)
			{
				continue;
			}
			const FModelLeg& M = Model.Legs[i];
			const FVector	 C = WheelContact(FVector(Leg->XZ.X, M.Y, Leg->XZ.Y), LevelAxle(M), Leg->Gradient, M.RadiusCm, M.ContactWidthCm);
			Contacts.Add(FVector2D(C.X, C.Y));
			Ground += C.Z;
		}
		Out.CenterOfMass = Model.CenterOfMassAt(XZ);
		Out.Support = Contacts.Num();
		if (Contacts.Num() == 0)
		{
			Out.StaticCm = Out.DynamicCm = TNumericLimits<double>::Lowest();
			return Out;
		}
		Out.HeightCm = Out.CenterOfMass.Z - Ground / Contacts.Num();
		const FVector2D C(Out.CenterOfMass.X, Out.CenterOfMass.Y);
		Out.StaticCm = HullMargin(Contacts, C);
		Out.DynamicCm = DynamicMargin(Contacts, C, Out.HeightCm, FVector2D(1.0, 0.0), Acceleration, Deceleration);
		return Out;
	}

	FChassisFrame ChassisFrameFrom(const FVector& Forward, double PitchRad, double RollRad, const FVector& PivotMeanWorld,
		const FVector& PivotMeanLocal)
	{
		FChassisFrame F;
		const FVector H = FVector(Forward.X, Forward.Y, 0.0).GetSafeNormal();
		const FVector Right(-H.Y, H.X, 0.0);
		const FVector Up = FVector::UpVector;
		F.X = H * FMath::Cos(PitchRad) + Up * FMath::Sin(PitchRad);
		const FVector Y0 = Right * FMath::Cos(RollRad) + Up * FMath::Sin(RollRad);
		F.Y = (Y0 - F.X * FVector::DotProduct(Y0, F.X)).GetSafeNormal();
		F.Z = FVector::CrossProduct(F.X, F.Y);
		F.Origin = PivotMeanWorld - (F.X * PivotMeanLocal.X + F.Y * PivotMeanLocal.Y + F.Z * PivotMeanLocal.Z);
		return F;
	}

	FPoseMargin MeasuredMargin(const FRobotModel& Model, const FChassisFrame& Frame, TConstArrayView<FVector2D> LegXZ,
		TConstArrayView<FMeasuredWheel> Wheels, const FVector2D& Heading, double Acceleration, double Deceleration)
	{
		FPoseMargin Out;
		Out.CenterOfMass = Frame.ToWorld(Model.CenterOfMassAt(LegXZ));
		TArray<FVector2D, TInlineAllocator<8>> Contacts;
		double								   Ground = 0.0;
		for (int32 i = 0; i < Wheels.Num() && i < Model.Legs.Num(); ++i)
		{
			const FMeasuredWheel& W = Wheels[i];
			if (!W.bSupport)
			{
				continue;
			}
			// The axle as the chassis carries it: tilted with it.
			const FModelLeg& M = Model.Legs[i];
			const FVector	 Local = LevelAxle(M);
			const FVector	 Axle = Frame.X * Local.X + Frame.Y * Local.Y;
			const FVector	 C = WheelContact(W.Centre, Axle, W.Gradient, M.RadiusCm, M.ContactWidthCm);
			Contacts.Add(FVector2D(C.X, C.Y));
			Ground += C.Z;
		}
		Out.Support = Contacts.Num();
		if (Contacts.Num() == 0)
		{
			Out.StaticCm = Out.DynamicCm = TNumericLimits<double>::Lowest();
			return Out;
		}
		Out.HeightCm = Out.CenterOfMass.Z - Ground / Contacts.Num();
		const FVector2D C(Out.CenterOfMass.X, Out.CenterOfMass.Y);
		Out.StaticCm = HullMargin(Contacts, C);
		Out.DynamicCm = DynamicMargin(Contacts, C, Out.HeightCm, Heading, Acceleration, Deceleration);
		return Out;
	}
} // namespace RammsCurb
