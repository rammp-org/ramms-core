// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsActuationBackend.h"
#include "UObject/WeakObjectPtr.h"

class USkeletalMeshComponent;

/**
 * IRammsActuationBackend backed by a Chaos skeletal physics asset — the native
 * UE path, so it lives in RammsCore (no third-party dependency) and the base
 * component instantiates it directly rather than through the cross-plugin
 * registry.
 *
 * A motor's Chaos name (FRammsMotorSpec::ChaosName, falling back to Id) is a
 * skeletal **bone** with a simulated body. Only **Torque** motors are supported:
 * SetCommand applies torque (N·m) about the bone's local spin axis, matching the
 * established wheel convention (local Y). Position/Velocity motors have no Chaos
 * counterpart here (those are MuJoCo <position>/<velocity> actuators) and are
 * ignored with a one-time warning — a Chaos robot drives its wheels by torque.
 */
class FRammsChaosActuationBackend final : public IRammsActuationBackend
{
public:
	virtual bool  Initialize(URammsRobotBaseComponent& Base) override;
	virtual void  SetCommand(FName MotorId, float Value) override;
	virtual float GetValue(FName MotorId) const override;
	virtual float GetVelocity(FName MotorId) const override;
	virtual bool  GetMotorTransform(FName MotorId, FTransform& OutWorld) const override;

private:
	/** The skeletal bone name a motor maps to (ChaosName, else Id). */
	FName BoneFor(FName MotorId) const;

	TWeakObjectPtr<class URammsRobotBaseComponent> BaseComp;
	TWeakObjectPtr<USkeletalMeshComponent>		   Mesh;

	/** Motors already warned about for lacking Chaos support (Position/Velocity). */
	mutable TSet<FName> WarnedUnsupported;
};
