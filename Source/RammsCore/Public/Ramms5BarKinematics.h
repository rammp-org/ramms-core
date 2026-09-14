// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Ramms5BarLinkageSpec.h"
#include "Ramms5BarKinematics.generated.h"

/**
 * Pure planar kinematics for a parallel 5-bar linkage described by an
 * FRamms5BarLinkageSpec. Stateless and physics-free — it turns a desired
 * endpoint into the two proximal joint angles a controller commands, and the
 * two joint angles back into the endpoint. All positions are in the linkage's
 * local x-z plane (cm), angles in radians (see the spec for the frame).
 */
UCLASS()
class RAMMSCORE_API URamms5BarKinematics : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Inverse kinematics: the two proximal joint angles (FVector2D(A, B), rad)
	 * that place the shared endpoint at TargetXZ. bReachable is false when either
	 * arm can't reach (the returned angles then clamp that arm as close as it
	 * gets) or when the pair is not a pose the assembled mechanism can hold at
	 * the target (checked with ComputeEndpoint: mirrored assembly, knee past
	 * straight); callers should gate commanding on it.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|5-Bar")
	static FVector2D SolveIK(const FRamms5BarLinkageSpec& Spec, FVector2D TargetXZ, bool& bReachable);

	/**
	 * Forward kinematics: the shared endpoint (x-z, cm) for a pair of proximal
	 * joint angles (FVector2D(A, B), rad). bValid is false when the closed loop
	 * can't hold those angles — either the distal links can't meet (the point
	 * returned is then the closest-approach midpoint) or a passive knee would
	 * have to bend past straight (the geometric intersection is returned, but
	 * it is not a pose of the assembled mechanism). Live joint readings always
	 * produce valid poses; the flag matters for hypothetical angle pairs.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|5-Bar")
	static FVector2D ComputeEndpoint(const FRamms5BarLinkageSpec& Spec, FVector2D JointAnglesAB, bool& bValid);

private:
	/** 2-link IK for one arm: joint angle (rad) reaching Target from Pivot.
	 *  bReachable false when out of the annulus (angle clamps to closest). */
	static double SolveArm(FVector2D Pivot, double ProximalLen, double DistalLen,
		double ZeroDir, double Sign, bool bElbowUp, FVector2D Target, bool& bReachable);

	/** Knee position for one arm at a given joint angle. */
	static FVector2D KneePosition(FVector2D Pivot, double ProximalLen, double ZeroDir, double Sign, double JointAngle);
};
