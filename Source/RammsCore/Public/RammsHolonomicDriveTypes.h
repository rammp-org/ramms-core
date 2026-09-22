// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataTable.h"
#include "RammsHolonomicDriveTypes.generated.h"

/**
 * One omni wheel: where it sits on the chassis and which way it rolls.
 *
 * The roll direction is the heading the wheel drives the contact point along,
 * which is perpendicular to its spin axis in the ground plane. For a spin axis
 * (ax, ay, 0) that is (ay, -ax) -- worth stating because reading the MJCF gives
 * the axis, not the direction.
 *
 * Positions are chassis-relative and must be authored: in the MJCF each wheel
 * body nests inside a swing arm, so its pos attribute is relative to that
 * parent (centimetres from a linkage, not from the chassis) and cannot be used
 * here.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsOmniWheelSpec
{
	GENERATED_BODY()

	/** Motor on the robot base that spins this wheel. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic")
	FName MotorId;

	/** Chassis-relative position, cm. X forward, Y left. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic")
	FVector2D Position = FVector2D::ZeroVector;

	/** Heading the wheel rolls along, degrees CCW from chassis forward. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic")
	float RollDirectionDeg = 0.0f;

	/** Flip when the motor's positive command spins the wheel backwards. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic")
	bool bInvert = false;
};

/** A whole base: its wheels and the scales that turn a stick into wheel speed. */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsHolonomicDriveSpec : public FTableRowBase
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic")
	TArray<FRammsOmniWheelSpec> Wheels;

	/**
	 * Rolling radius, cm: the height of the wheel's axle above the ground it
	 * rests on, which for a simple wheel is its collision radius.
	 *
	 * SolveWheelRates divides every commanded ground speed by this, so a wrong
	 * value scales every wheel rate and nothing anywhere can detect it -- the
	 * velocity loop tracks the wrong target perfectly and the robot simply
	 * moves at the wrong speed. This default is a placeholder; the lift-drive
	 * ran on it at 7.5 against a real 10.7 and asked its wheels for 43% more
	 * than the body twist needed. Measure it per robot.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic", meta = (ClampMin = "0.1"))
	float WheelRadiusCm = 7.5f;

	/** Ground speed at full stick deflection, cm/s. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic", meta = (ClampMin = "0.0"))
	float MaxLinearSpeedCmPerSecond = 60.0f;

	/** Yaw rate at full stick deflection, deg/s. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic", meta = (ClampMin = "0.0"))
	float MaxYawRateDegPerSecond = 90.0f;
};
