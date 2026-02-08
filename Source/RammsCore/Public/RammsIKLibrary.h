#pragma once

#include "Kismet/BlueprintFunctionLibrary.h"
#include "RammsIKLibrary.generated.h"

USTRUCT(BlueprintType)
struct FIKSolveResult
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly)
	bool bSuccess = false;

	UPROPERTY(BlueprintReadOnly)
	TArray<float> JointAngles; // degrees

	UPROPERTY(BlueprintReadOnly)
	float PositionError = 0.0f; // cm

	UPROPERTY(BlueprintReadOnly)
	float RotationError = 0.0f; // degrees

	UPROPERTY(BlueprintReadOnly)
	int32 IterationsUsed = 0;
};

UCLASS()
class URammsIKLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, Category="RAMMS|IK")
	static FTransform ComputeForwardKinematics(
		const FTransform& BaseTransform,
		const TArray<float>& JointAnglesDeg,
		const TArray<FTransform>& JointLocalTransforms,
		const TArray<FVector>& JointAxesLocal,
		const FTransform& EndEffectorOffset,
		bool bEnableDebugLogging);

	/**
	 * Proper iterative DLS IK using an FK chain model.
	 *
	 * Conventions (match this!):
	 * - JointLocalTransforms[i] transforms ParentJointFrame -> Joint_i_Frame at q_i = 0 (full rotation+translation).
	 * - JointAxesLocal[i] is joint axis expressed in Joint_i_Frame.
	 * - FK step: T = JointLocalTransforms[i] * T; then rotate about axis in that joint frame.
	 *
	 * TaskSpaceMask6 is [X,Y,Z, Roll,Pitch,Yaw] where angular part masks the rotation-vector components
	 * (world-axis axis-angle error), which matches "position + yaw" well in UE (yaw ~ Z).
	 *
	 * JointWeights: higher => prefer NOT to move that joint (1.0 normal, 10.0 moves ~10x less).
	 * If empty => all 1.0.
	 */
	UFUNCTION(BlueprintCallable, Category="RAMMS|IK")
	static FIKSolveResult SolveIK_FKChain(
		const FTransform& BaseTransform,
		const TArray<float>& CurrentAnglesDeg,
		const TArray<FTransform>& JointLocalTransforms,
		const TArray<FVector>& JointAxesLocal,
		const FTransform& EndEffectorOffset,
		const FTransform& TargetEndEffectorWorld,
		const TArray<FVector2D>& JointLimitsDeg,
		const TArray<bool>& TaskSpaceMask6,
		const TArray<float>& JointWeights,
		bool bEnableNullSpaceOptimization,
		float NullSpaceGain,
		const TArray<float>& NullSpaceBiasDeg,
		float Damping,
		float StepClipDeg,
		int32 MaxIterations,
		float PositionToleranceCm,
		float RotationToleranceDeg);
};
