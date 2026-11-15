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

	/** The time of the state (pose and velocities), on the producing engine's clock (s): the Chaos solver's time at
	 *  the end of the step, MuJoCo's mjData::time at the start of the step (the state its forward pass computed), or
	 *  the world's game time for Kinematic. */
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
	 *  at rest and about zero in free fall. The acceleration the engine's integrator applied over a step: for Chaos
	 *  the velocity change of the step that ENDS at this sample's state, for MuJoCo (by default; see
	 *  ramms.Imu.MuJoCoAcceleration) that of the step that STARTS at it, the state its forward pass computed. So the
	 *  two engines' accelerations are a step apart in phase; differencing LinearVelocity across samples gives one
	 *  convention for both. */
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	FVector ProperAcceleration = FVector::ZeroVector;

	/** The body's angular acceleration (deg/s^2, world). */
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	FVector AngularAcceleration = FVector::ZeroVector;

	/** The gravity the producing engine applies to the body (cm/s^2, world; zero for a body it applies none to):
	 *  the point's coordinate acceleration is ProperAcceleration + Gravity. */
	UPROPERTY(BlueprintReadOnly, Category = "IMU")
	FVector Gravity = FVector::ZeroVector;
};

/**
 * The last Capacity samples of one tracked point, one per physics step, numbered by a sequence that only grows (the
 * first sample pushed is 0). SetCapacity allocates (game thread); Push never does, so a physics thread can keep it.
 * Not thread-safe: its owner locks. Providers push complete samples only (bHasAcceleration): a history row always
 * carries a measured acceleration, never a stand-in zero.
 */
struct RAMMSCORE_API FRammsImuSampleRing
{
	/** Keep the last Capacity samples (0: keep none). Drops what it held; the sequence goes on from where it was. */
	void SetCapacity(int32 Capacity);

	int32 GetCapacity() const { return Buffer.Num(); }

	/** Add a sample (nothing without capacity). A sample stamped with the newest one's time replaces it, so a step
	 *  an engine takes in several passes leaves its final state. */
	void Push(const FRammsImuSample& Sample);

	/** The sequence number the next sample gets. */
	int64 GetNextSeq() const { return Next; }

	/** The oldest sequence number kept (GetNextSeq when the ring is empty). */
	int64 GetOldestSeq() const { return Next - Count; }

	/** Append up to Max samples from sequence Since on (clamped to the kept range), oldest first; returns the
	 *  sequence of the first one appended (where the read started). */
	int64 Read(int64 Since, int32 Max, TArray<FRammsImuSample>& Out) const;

private:
	TArray<FRammsImuSample> Buffer;
	int64					Next = 0;
	int32					Count = 0;
};
