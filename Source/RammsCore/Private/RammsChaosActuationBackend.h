// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsActuationBackend.h"
#include "UObject/WeakObjectPtr.h"

class USkeletalMeshComponent;
struct FConstraintInstance;

/**
 * IRammsActuationBackend backed by a Chaos skeletal physics asset — the native
 * UE path, so it lives in RammsCore (no third-party dependency) and the base
 * component instantiates it directly rather than through the cross-plugin
 * registry.
 *
 * A motor's Chaos name (FRammsMotorSpec::ChaosName, falling back to Id) is:
 *
 *  - for a **Torque** motor, a skeletal bone with a simulated body: SetCommand
 *    applies torque about the bone's local spin axis (Y), the established
 *    wheel convention;
 *  - for a **Position** motor, a physics-asset **constraint**: SetCommand sets
 *    the constraint's drive target. The driven degree of freedom is inferred
 *    from the constraint's first non-locked axis — a linear axis means a
 *    linear actuator commanded/read in cm along it, otherwise an angular one
 *    (twist = X, swing2 = Y, swing1 = Z) commanded/read in radians. Drive
 *    stiffness / damping / force limit come from the base component's
 *    Chaos settings. If the owner also carries a UMebotControllerComponent
 *    that lists the same constraint, that entry is disabled on first command
 *    so the two don't fight over the drive target.
 *
 * Velocity motors have no Chaos counterpart here and are ignored with a
 * one-time warning.
 *
 * Units caveat (torque): this reproduces the pre-existing differential-drive
 * Chaos path exactly — the command is scaled ×100 and applied with
 * `bAccelChange = true`, so Chaos treats it as an inertia-independent angular
 * acceleration change, not a physical N·m torque. Kept on purpose so the tuned
 * Chaos chair drives identically through the base component; on MuJoCo the
 * same command IS a torque in N·m, so motor parameters tuned on Chaos need
 * retuning there.
 */
class FRammsChaosActuationBackend final : public IRammsActuationBackend
{
public:
	virtual bool  Initialize(URammsRobotBaseComponent& Base) override;
	virtual void  SetCommand(FName MotorId, float Value) override;
	virtual float GetValue(FName MotorId) const override;
	virtual float GetVelocity(FName MotorId) const override;
	virtual void  ReleaseMotor(FName MotorId) override;
	virtual bool  GetMotorTransform(FName MotorId, FTransform& OutWorld) const override;

private:
	/** A Position motor resolved to a physics-asset constraint. */
	struct FConstraintMotor
	{
		FName ConstraintName;
		bool  bLinear = false;
		/** Constraint-frame axis: 0 = X (linear X / twist), 1 = Y (linear Y /
		 *  swing2), 2 = Z (linear Z / swing1). */
		int32 Axis = 0;
		FName ChildBone;
		FName ParentBone;
		/** Child bone's reference-pose location in the parent bone's space:
		 *  the linear travel's zero. */
		FVector RestOffset = FVector::ZeroVector;
		bool	bTakenOver = false;
	};

	/** The skeletal bone / constraint name a motor maps to (ChaosName, else Id). */
	FName BoneFor(FName MotorId) const;

	/** Is this a Position motor (constraint-driven) rather than a bone torque? */
	bool IsConstraintMotor(FName MotorId) const;

	/** Resolve (and cache) a Position motor's constraint and driven axis. Null
	 *  (with a one-time warning) when the constraint is missing or fully locked. */
	FConstraintInstance* ResolveConstraintMotor(FName MotorId, FConstraintMotor*& OutInfo) const;

	/** Relative transform of the constraint's child bone in its parent bone's space. */
	bool GetChildInParent(const FConstraintMotor& Info, FTransform& OutChildInParent) const;

	/** Hand the constraint over from a UMebotControllerComponent on the owner, once. */
	void TakeOverFromMebotController(FConstraintMotor& Info);

	TWeakObjectPtr<class URammsRobotBaseComponent> BaseComp;
	TWeakObjectPtr<USkeletalMeshComponent>		   Mesh;

	/** Resolved constraint motors by Id (lazily; constraints exist only once
	 *  the mesh's physics state does). */
	mutable TMap<FName, FConstraintMotor> ConstraintMotors;

	/** Motors already warned about (unsupported type, missing constraint...). */
	mutable TSet<FName> WarnedUnsupported;
};
