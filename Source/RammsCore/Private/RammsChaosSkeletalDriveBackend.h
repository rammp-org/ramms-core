// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsDriveBackend.h"
#include "UObject/WeakObjectPtr.h"

class USkeletalMeshComponent;
struct FBodyInstance;

/**
 * The default drive backend: applies torque to, and reads state from, wheel
 * bones of a Chaos-simulated skeletal mesh. This is the exact behavior the
 * differential-drive controller had before the backend seam was introduced —
 * the wheel-state read, motor-curve + slip/traction torque application, and
 * brake damping, all against `USkeletalMeshComponent` `FBodyInstance`s.
 *
 * Configuration (bone names, wheel radius, motor and slip/traction parameters,
 * debug flags) is read from the owning controller passed to Initialize.
 */
class FRammsChaosSkeletalDriveBackend final : public IRammsDriveBackend
{
public:
	virtual bool Initialize(URammsDifferentialDriveController& Controller) override;
	virtual void ReadWheelState(ERammsDriveWheel Wheel, FWheelState& OutState) override;
	virtual void ApplyWheelTorque(ERammsDriveWheel Wheel, float RequestedTorque,
		const FMotorParameters& MotorParams, FWheelState& WheelState) override;
	virtual void ApplyBrake(ERammsDriveWheel Wheel, FWheelState& WheelState) override;

private:
	/** The bone name the owning controller assigns to this wheel. */
	FName BoneNameFor(ERammsDriveWheel Wheel) const;

	/** Body instance for a wheel bone, or null when unavailable. */
	FBodyInstance* BodyInstanceFor(ERammsDriveWheel Wheel) const;

	/** Owning controller — source of all configuration. Set in Initialize. */
	TWeakObjectPtr<URammsDifferentialDriveController> Controller;

	/** The resolved skeletal mesh carrying the wheel bones. */
	TWeakObjectPtr<USkeletalMeshComponent> SkeletalMesh;
};
