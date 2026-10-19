// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbLevel.h"

#include "Curb/RammsCurbSpeedLaw.h"
#include "Curb/RammsCurbTerrainFit.h"
#include "Ramms4BarKinematics.h"

namespace RammsCurb
{
	float Deadband(float Error, float Band)
	{
		Band = FMath::Max(0.0f, Band);
		return Error > Band ? Error - Band : (Error < -Band ? Error + Band : 0.0f);
	}

	FVector2D FAttitudeLoop::Tick(const FVector2D& ErrorDeg, float Dt, bool bIntegrate, float Kp, float Ki, float DeadbandDeg,
		float MaxDeg)
	{
		MaxDeg = FMath::Max(0.0f, MaxDeg);
		const FVector2D E(Deadband(static_cast<float>(ErrorDeg.X), DeadbandDeg), Deadband(static_cast<float>(ErrorDeg.Y), DeadbandDeg));
		if (bIntegrate && Dt > 0.0f)
		{
			Integral += E * (Ki * Dt);
			Integral.X = FMath::Clamp(Integral.X, -MaxDeg, MaxDeg);
			Integral.Y = FMath::Clamp(Integral.Y, -MaxDeg, MaxDeg);
		}
		Correction = Integral + E * Kp;
		Correction.X = FMath::Clamp(Correction.X, -MaxDeg, MaxDeg);
		Correction.Y = FMath::Clamp(Correction.Y, -MaxDeg, MaxDeg);
		return Correction;
	}

	void AttitudeOffsets(TConstArrayView<FVector2D> LegXY, TConstArrayView<bool> Support, const FVector2D& CorrectionDeg,
		TArrayView<float> Out)
	{
		const double TanPitch = FMath::Tan(FMath::DegreesToRadians(CorrectionDeg.X));
		const double TanRoll = FMath::Tan(FMath::DegreesToRadians(CorrectionDeg.Y));
		double		 Sum = 0.0;
		int32		 Count = 0;
		for (int32 i = 0; i < Out.Num(); ++i)
		{
			const bool bSupport = Support.IsValidIndex(i) && Support[i] && LegXY.IsValidIndex(i);
			Out[i] = bSupport ? static_cast<float>(LegXY[i].X * TanPitch + LegXY[i].Y * TanRoll) : 0.0f;
			if (bSupport)
			{
				Sum += Out[i];
				++Count;
			}
		}
		const float Mean = Count > 0 ? static_cast<float>(Sum / Count) : 0.0f;
		for (int32 i = 0; i < Out.Num(); ++i)
		{
			if (Support.IsValidIndex(i) && Support[i] && LegXY.IsValidIndex(i))
			{
				Out[i] -= Mean;
			}
		}
	}

	void FPlaneBias::Tick(TConstArrayView<float> ResidualsCm, float Dt)
	{
		if (ResidualsCm.Num() < MinWheelsForBias || Dt <= 0.0f)
		{
			return;
		}
		float Lowest = TNumericLimits<float>::Max();
		for (const float R : ResidualsCm)
		{
			Lowest = FMath::Min(Lowest, FMath::Clamp(R, -PlaneBiasSampleCm, PlaneBiasSampleCm));
		}
		Bias += (Lowest - Bias) * FMath::Clamp(Dt / PlaneBiasSeconds, 0.0f, 1.0f);
		Bias = FMath::Clamp(Bias, -MaxPlaneBiasCm, MaxPlaneBiasCm);
	}

	void DriveRoles(TConstArrayView<FDriveWheel> Wheels, TArrayView<bool> OutDriven)
	{
		// Per side: is there a Main wheel, and is one of them not planted.
		bool bHasMain[2] = { false, false };
		bool bMainDown[2] = { false, false };
		for (const FDriveWheel& W : Wheels)
		{
			if (W.Role == ERammsCurbWheelDrive::Main)
			{
				const int32 Side = W.bPlusY ? 0 : 1;
				bHasMain[Side] = true;
				bMainDown[Side] = bMainDown[Side] || !W.bPlanted;
			}
		}
		for (int32 i = 0; i < Wheels.Num() && i < OutDriven.Num(); ++i)
		{
			const FDriveWheel& W = Wheels[i];
			const int32		   Side = W.bPlusY ? 0 : 1;
			switch (W.Role)
			{
				case ERammsCurbWheelDrive::Main:
					OutDriven[i] = W.bPlanted;
					break;
				case ERammsCurbWheelDrive::Fallback:
					OutDriven[i] = !bHasMain[Side] || bMainDown[Side];
					break;
				case ERammsCurbWheelDrive::Passive:
					OutDriven[i] = false;
					break;
				default: // Assist
					OutDriven[i] = true;
					break;
			}
		}
	}

	float SwingTiltRaise(float X, float Y, float PitchRad, float RollRad)
	{
		return FMath::Max(0.0f, -(X * FMath::Sin(PitchRad) + Y * FMath::Sin(RollRad)));
	}

	double SurfaceHeight(const FRammsCurbTerrain& Terrain, bool bFar, const FVector2D& XY)
	{
		return (bFar ? Terrain.Far : Terrain.Near).HeightAt(XY);
	}

	double TerrainOffsets(const FRammsCurbTerrain& Terrain, TConstArrayView<FTerrainLeg> Legs, double GroundZ,
		double StepHeightCm, const FVector2D& BiasCm, TArrayView<double> Out)
	{
		// Each wheel's ground over the ground the plan is reckoned on, less the
		// level the level-ground plan puts it on; their mean is the reference.
		double Sum = 0.0;
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			const FTerrainLeg& L = Legs[i];
			const double	   Above = SurfaceHeight(Terrain, L.bFar, L.XY) + (L.bFar ? BiasCm.Y : BiasCm.X) - GroundZ
				- (L.bFar ? StepHeightCm : 0.0);
			if (Out.IsValidIndex(i))
			{
				Out[i] = Above;
			}
			Sum += Above;
		}
		const double Reference = Legs.Num() > 0 ? Sum / Legs.Num() : 0.0;
		for (int32 i = 0; i < Legs.Num() && i < Out.Num(); ++i)
		{
			const FTerrainLeg& L = Legs[i];
			const FVector2D	   Gradient = (L.bFar ? Terrain.Far : Terrain.Near).Gradient;
			const double	   Rho = WheelCentreHeight(L.RadiusCm, L.ContactWidthCm, Gradient, L.RollDirection);
			Out[i] += Rho - L.RadiusCm - Reference;
		}
		return Reference;
	}

	double PathClearanceHeight(const FRammsCurbTerrain& Terrain, TConstArrayView<FVector2D> Path, const FVector2D& RollDirection,
		double RadiusCm, double TyreWidthCm, double DilationCm)
	{
		double Best = -TNumericLimits<double>::Max();
		for (const FVector2D& P : Path)
		{
			Best = FMath::Max(Best, ClearanceHeight(Terrain, P, RollDirection, RadiusCm, TyreWidthCm, DilationCm));
		}
		return Best;
	}

	bool MeasureFourBarCrank(const FRamms4BarLegSpec& Spec, float Crank, float ToleranceRad, FVector2D& OutWheel,
		float& OutCrank, bool& bOutClamped)
	{
		bOutClamped = false;
		OutCrank = Crank;
		bool bValid = false;
		OutWheel = URamms4BarKinematics::ComputeWheelCenter(Spec, Crank, bValid);
		if (bValid)
		{
			return true;
		}
		// A servo pressed a hair past its stop -- by the ground, or by the mode
		// before -- reads just outside the range the spec closes over. Within
		// the tolerance it is at that end; further out, the leg is somewhere
		// the spec does not describe.
		const float Lo = FMath::Min(Spec.CrankRange.X, Spec.CrankRange.Y);
		const float Hi = FMath::Max(Spec.CrankRange.X, Spec.CrankRange.Y);
		const float Tolerance = FMath::Max(0.0f, ToleranceRad);
		float		End = 0.0f;
		if (Crank < Lo && Crank >= Lo - Tolerance)
		{
			End = Lo;
		}
		else if (Crank > Hi && Crank <= Hi + Tolerance)
		{
			End = Hi;
		}
		else
		{
			return false;
		}
		OutWheel = URamms4BarKinematics::ComputeWheelCenter(Spec, End, bValid);
		if (!bValid)
		{
			OutWheel = FVector2D::ZeroVector;
			return false;
		}
		OutCrank = End;
		bOutClamped = true;
		return true;
	}

	bool CorridorPath(TConstArrayView<FVector2D> Bands, TConstArrayView<double> Wish, double Rate, TArray<float>& OutPath,
		int32& OutStuck)
	{
		const int32 N = Bands.Num();
		OutStuck = INDEX_NONE;
		OutPath.Reset();
		Rate = FMath::Max(0.0, Rate);
		// Forward: what is reachable having kept inside every band before;
		// back: of that, what can still keep inside every band after.
		TArray<FVector2D> Tube(Bands.GetData(), N);
		for (int32 k = 0; k < N && OutStuck == INDEX_NONE; ++k)
		{
			if (k > 0)
			{
				Tube[k].X = FMath::Max(Tube[k].X, Tube[k - 1].X - Rate);
				Tube[k].Y = FMath::Min(Tube[k].Y, Tube[k - 1].Y + Rate);
			}
			OutStuck = Tube[k].X > Tube[k].Y ? k : INDEX_NONE;
		}
		for (int32 k = N - 2; k >= 0 && OutStuck == INDEX_NONE; --k)
		{
			Tube[k].X = FMath::Max(Tube[k].X, Tube[k + 1].X - Rate);
			Tube[k].Y = FMath::Min(Tube[k].Y, Tube[k + 1].Y + Rate);
			OutStuck = Tube[k].X > Tube[k].Y ? k : INDEX_NONE;
		}
		if (OutStuck != INDEX_NONE || N == 0)
		{
			return false;
		}
		// Every height in the tube has a next one within Rate in the next
		// tube, so the pass forward never runs dry.
		OutPath.SetNum(N);
		for (int32 k = 0; k < N; ++k)
		{
			double Lo = Tube[k].X, Hi = Tube[k].Y;
			if (k > 0)
			{
				Lo = FMath::Max(Lo, OutPath[k - 1] - Rate);
				Hi = FMath::Min(Hi, OutPath[k - 1] + Rate);
			}
			const double Want = Wish.IsValidIndex(k) && !FMath::IsNaN(Wish[k]) ? Wish[k] : (k > 0 ? OutPath[k - 1] : 0.5 * (Lo + Hi));
			OutPath[k] = static_cast<float>(FMath::Clamp(Want, Lo, FMath::Max(Lo, Hi)));
		}
		return true;
	}

	float LevelInStartCm(float Seconds, float SpeedCmS, float ApproachDeg)
	{
		const double Cos = FMath::Max(0.1, FMath::Cos(FMath::DegreesToRadians(static_cast<double>(ApproachDeg))));
		return static_cast<float>(LevelBeforeEdgeCm / Cos) + LevelInMarginCm
			+ FMath::Max(0.0f, SpeedCmS) * (FMath::Max(0.0f, Seconds) + LevelInSpareSeconds + DriveLagSeconds);
	}

	FVector2D FLevelState::TickAttitude(const FVector2D& MeasuredDeg, const FVector2D& PlannedDeg, float Dt, bool bIntegrate,
		const FRammsCurbTuning& Tuning)
	{
		if (!bFilterSeeded)
		{
			FilteredAttitudeDeg = MeasuredDeg;
			bFilterSeeded = true;
		}
		else
		{
			FilteredAttitudeDeg += (MeasuredDeg - FilteredAttitudeDeg) * FMath::Clamp(Dt / AttitudeFilterSeconds, 0.0f, 1.0f);
		}
		if (!bAttitudeActive)
		{
			return FVector2D::ZeroVector;
		}
		return Attitude.Tick(FilteredAttitudeDeg - PlannedDeg, Dt, bIntegrate, Tuning.AttitudeProportionalGain,
			Tuning.AttitudeIntegralGain, Tuning.AttitudeDeadbandDeg, Tuning.MaxAttitudeCorrectionDeg);
	}

	bool FLevelState::TickLevelIn(bool bUnderWay, bool bCaughtUp)
	{
		if (bLevelled || !bUnderWay || !bCaughtUp)
		{
			return false;
		}
		// The legs are where the terrain wants them: from here what is left of
		// the tilt is the loop's to trim.
		bAttitudeActive = true;
		if (FMath::Abs(FilteredAttitudeDeg.X) > LevelInToleranceDeg || FMath::Abs(FilteredAttitudeDeg.Y) > LevelInToleranceDeg)
		{
			return false;
		}
		bLevelled = true;
		return true;
	}

	float LateLevelSpeed(float DistanceCm, float ApproachDeg, float GateCm, float Seconds, float Deceleration, float SpeedCmS,
		float LagSeconds)
	{
		const double Cos = FMath::Max(0.1, FMath::Cos(FMath::DegreesToRadians(static_cast<double>(ApproachDeg))));
		// The room to the line, less what a chassis running a lag behind its
		// command still covers once the command stops.
		const float Room = LagBrakingDistance(DistanceCm - static_cast<float>(LevelBeforeEdgeCm / Cos) - LevelCapMarginCm, SpeedCmS,
			LagSeconds);
		if (Room <= 0.0f)
		{
			return 0.0f;
		}
		// At speed v the line comes in Room / v and the level-in is done in
		// Gate / v + Seconds: so v <= (Room - Gate) / Seconds.
		const float OnTime = FMath::Max(0.0f, Room - FMath::Max(0.0f, GateCm)) / FMath::Max(Seconds, MinDeadlineSeconds);
		return FMath::Min(OnTime, BrakingSpeed(Room, Deceleration));
	}
} // namespace RammsCurb
