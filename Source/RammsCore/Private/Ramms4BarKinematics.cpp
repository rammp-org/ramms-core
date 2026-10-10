// Copyright Epic Games, Inc. All Rights Reserved.

#include "Ramms4BarKinematics.h"

namespace
{
	/** Rotate an x-z vector about the chassis' +Y axis. Right-handed: a
	 *  positive angle carries +X towards -Z, as a MuJoCo hinge on (0 1 0) does. */
	FVector2D RotateXZ(double Angle, const FVector2D& V)
	{
		const double C = FMath::Cos(Angle);
		const double S = FMath::Sin(Angle);
		return FVector2D(V.X * C + V.Y * S, -V.X * S + V.Y * C);
	}

	/** Angle of an x-z vector, measured the same way RotateXZ turns. */
	double AngleOf(const FVector2D& V)
	{
		return FMath::Atan2(-V.Y, V.X);
	}

	/** Which way round the rocker-pin-coupler triangle turns: the assembly
	 *  mode. Moving the leg never changes it short of a singular pose. */
	double Winding(const FVector2D& RockerPivot, const FVector2D& Pin, const FVector2D& CouplerPin)
	{
		return FVector2D::CrossProduct(Pin - RockerPivot, CouplerPin - Pin);
	}

	/**
	 * Every crank angle has up to two loop closures, one per assembly mode.
	 * The one the leg is built in is the one the zero pose has, so take the
	 * closure that winds the same way.
	 *
	 * Not the closure inside the rocker's joint range: near the ends of the
	 * crank's travel both are inside it -- at crank -0.5 on the lift-drive the
	 * crossed closure puts a wheel that is 13 cm up 7 cm down, and choosing by
	 * range sent the rear cranks the wrong way when asked to plant.
	 */
	bool SolveRocker(const FRamms4BarLegSpec& Spec, double Crank, double& OutRocker)
	{
		const FVector2D CouplerPin = Spec.CrankPivot + RotateXZ(Spec.CrankAngleSign * Crank, Spec.CrankArm);
		const FVector2D ToPin = CouplerPin - Spec.RockerPivot;
		const double	R1 = Spec.RockerPin.Size();
		const double	R2 = Spec.CouplerLength;
		const double	D = ToPin.Size();
		if (D < UE_KINDA_SMALL_NUMBER || D > R1 + R2 || D < FMath::Abs(R1 - R2))
		{
			return false;
		}
		const double	A = (R1 * R1 - R2 * R2 + D * D) / (2.0 * D);
		const double	H = FMath::Sqrt(FMath::Max(R1 * R1 - A * A, 0.0));
		const FVector2D Ex = ToPin / D;
		const FVector2D Mid = Spec.RockerPivot + Ex * A;

		const double ZeroAngle = AngleOf(Spec.RockerPin);
		const double Mode = FMath::Sign(
			Winding(Spec.RockerPivot, Spec.RockerPivot + Spec.RockerPin, Spec.CrankPivot + Spec.CrankArm));
		for (const double Side : { 1.0, -1.0 })
		{
			const FVector2D Pin(Mid.X - Side * H * Ex.Y, Mid.Y + Side * H * Ex.X);
			if (FMath::Sign(Winding(Spec.RockerPivot, Pin, CouplerPin)) != Mode)
			{
				continue;
			}
			const double Rocker =
				FMath::UnwindRadians((AngleOf(Pin - Spec.RockerPivot) - ZeroAngle) * Spec.RockerAngleSign);
			if (Rocker < Spec.RockerRange.X - 1e-3 || Rocker > Spec.RockerRange.Y + 1e-3)
			{
				return false;
			}
			OutRocker = Rocker;
			return true;
		}
		return false;
	}
} // namespace

FVector2D URamms4BarKinematics::ComputeWheelCenter(const FRamms4BarLegSpec& Spec, float CrankAngle, bool& bValid)
{
	double Rocker = 0.0;
	bValid = SolveRocker(Spec, CrankAngle, Rocker);
	if (!bValid)
	{
		return FVector2D::ZeroVector;
	}
	return Spec.RockerPivot + RotateXZ(Spec.RockerAngleSign * Rocker, Spec.WheelOnRocker) + Spec.WheelOffset;
}

float URamms4BarKinematics::SolveCrankForHeight(const FRamms4BarLegSpec& Spec, float Z, bool& bReachable, float& OutWheelX)
{
	bReachable = false;
	OutWheelX = 0.0f;

	double Lo = Spec.CrankRange.X;
	double Hi = Spec.CrankRange.Y;
	bool   bLoValid = false;
	bool   bHiValid = false;
	double ZLo = ComputeWheelCenter(Spec, Lo, bLoValid).Y;
	double ZHi = ComputeWheelCenter(Spec, Hi, bHiValid).Y;
	if (!bLoValid || !bHiValid)
	{
		return static_cast<float>(Lo);
	}

	// Height is monotonic in the crank across the authored range, so this is a
	// bracketed root find. Which end is higher depends on how the leg is hung.
	if (Z < FMath::Min(ZLo, ZHi) || Z > FMath::Max(ZLo, ZHi))
	{
		const bool	 bNearLo = FMath::Abs(Z - ZLo) < FMath::Abs(Z - ZHi);
		const double End = bNearLo ? Lo : Hi;
		bool		 bValid = false;
		OutWheelX = static_cast<float>(ComputeWheelCenter(Spec, End, bValid).X);
		return static_cast<float>(End);
	}
	for (int32 i = 0; i < 48; ++i)
	{
		const double Mid = 0.5 * (Lo + Hi);
		bool		 bValid = false;
		const double ZMid = ComputeWheelCenter(Spec, Mid, bValid).Y;
		if (!bValid)
		{
			return static_cast<float>(Mid);
		}
		if ((ZMid - Z) * (ZLo - Z) > 0.0)
		{
			Lo = Mid;
			ZLo = ZMid;
		}
		else
		{
			Hi = Mid;
		}
	}
	const double Crank = 0.5 * (Lo + Hi);
	bool		 bValid = false;
	OutWheelX = static_cast<float>(ComputeWheelCenter(Spec, Crank, bValid).X);
	bReachable = bValid;
	return static_cast<float>(Crank);
}

FVector2D URamms4BarKinematics::GetHeightRange(const FRamms4BarLegSpec& Spec)
{
	double		Min = TNumericLimits<double>::Max();
	double		Max = TNumericLimits<double>::Lowest();
	const int32 Samples = 64;
	for (int32 i = 0; i <= Samples; ++i)
	{
		const double Crank = FMath::Lerp(Spec.CrankRange.X, Spec.CrankRange.Y, double(i) / Samples);
		bool		 bValid = false;
		const double Z = ComputeWheelCenter(Spec, Crank, bValid).Y;
		if (bValid)
		{
			Min = FMath::Min(Min, Z);
			Max = FMath::Max(Max, Z);
		}
	}
	return Min <= Max ? FVector2D(Min, Max) : FVector2D::ZeroVector;
}
