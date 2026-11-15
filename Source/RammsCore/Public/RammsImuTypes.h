// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsImuTypes.generated.h"

/** Where inertial readings come from. */
UENUM(BlueprintType)
enum class ERammsImuSource : uint8
{
	/** The physics engine that simulates the body the sensor rides on: MuJoCo for a URLab body (or anything
	 *  attached below one), Chaos for a primitive or skeletal bone that simulates physics; else Kinematic. */
	Auto,
	/** Chaos rigid-body state, sampled every Chaos step. */
	Chaos,
	/** MuJoCo's body state (mjData cvel / cacc), sampled every MuJoCo step. */
	MuJoCo,
	/** Differencing the sensor's transform at the game rate. */
	Kinematic,
};

/**
 * One inertial sample of a point on a body, everything in world space and Unreal units, AT THE POINT (the lever
 * arm from the body's origin or centre of mass already applied): v_p = v_o + w x r, a_p = a_o + alpha x r + w x (w x r).
 *
 * Angular rates follow Unreal's quaternions: a point at r from the axis moves at AngularVelocity x r
 * (FVector::CrossProduct), with the angle in degrees. Express a vector in the sensor's frame with
 * Orientation.UnrotateVector (RammsImuMath::ToSensorFrame).
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsImuSample
{
	GENERATED_BODY()

	/** A pose has been sampled (velocities may still be zero on the very first Kinematic sample). */
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	bool bValid = false;

	/** ProperAcceleration and AngularAcceleration hold a measurement (not yet on a first sample). */
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	bool bHasAcceleration = false;

	/** The engine that produced the sample (never Auto). */
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	ERammsImuSource Source = ERammsImuSource::Kinematic;

	/** The time of the state, on the producing engine's clock (s): MuJoCo's mjData::time, the Chaos solver's time
	 *  at the end of the step, or the world's game time for Kinematic. */
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	double SimTimeSeconds = 0.0;

	/** The point (cm, world). */
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	FVector Location = FVector::ZeroVector;

	/** The sensor frame's orientation (world). */
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	FQuat Orientation = FQuat::Identity;

	/** The point's velocity (cm/s, world). */
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	FVector LinearVelocity = FVector::ZeroVector;

	/** The body's angular velocity (deg/s, world). */
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	FVector AngularVelocity = FVector::ZeroVector;

	/** What an accelerometer at the point reads, in world axes (cm/s^2): its acceleration minus gravity, so +|g| up
	 *  at rest and about zero in free fall. */
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	FVector ProperAcceleration = FVector::ZeroVector;

	/** The body's angular acceleration (deg/s^2, world). */
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	FVector AngularAcceleration = FVector::ZeroVector;

	/** The gravity the producing engine applies (cm/s^2, world): the point's coordinate acceleration is
	 *  ProperAcceleration + Gravity. */
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	FVector Gravity = FVector::ZeroVector;
};
