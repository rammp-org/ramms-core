// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"

// Include Eigen (comes with Unreal Engine)
THIRD_PARTY_INCLUDES_START
#include <Eigen/Dense>
THIRD_PARTY_INCLUDES_END

#include "RammsIKLibrary.generated.h"

/**
 * Joint information for IK calculations
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FIKJoint
{
	GENERATED_BODY()

	/** Joint position in world space */
	UPROPERTY(BlueprintReadWrite, Category = "IK")
	FVector Position = FVector::ZeroVector;

	/** Joint rotation in world space */
	UPROPERTY(BlueprintReadWrite, Category = "IK")
	FRotator Rotation = FRotator::ZeroRotator;

	/** Minimum angle limit (degrees) */
	UPROPERTY(BlueprintReadWrite, Category = "IK")
	float MinLimit = -180.0f;

	/** Maximum angle limit (degrees) */
	UPROPERTY(BlueprintReadWrite, Category = "IK")
	float MaxLimit = 180.0f;
};

/**
 * IK solver result
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FIKSolveResult
{
	GENERATED_BODY()

	/** Whether IK solved successfully */
	UPROPERTY(BlueprintReadOnly, Category = "IK")
	bool bSuccess = false;

	/** Final joint angles (degrees) */
	UPROPERTY(BlueprintReadOnly, Category = "IK")
	TArray<float> JointAngles;

	/** Number of iterations used */
	UPROPERTY(BlueprintReadOnly, Category = "IK")
	int32 IterationsUsed = 0;

	/** Final position error (cm) */
	UPROPERTY(BlueprintReadOnly, Category = "IK")
	float PositionError = 0.0f;

	/** Final rotation error (degrees) */
	UPROPERTY(BlueprintReadOnly, Category = "IK")
	float RotationError = 0.0f;
};

/**
 * Library of inverse kinematics functions for robotic arms
 */
UCLASS()
class RAMMSCORE_API URammsIKLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Solve inverse kinematics using FABRIK (Forward And Backward Reaching Inverse Kinematics)
	 * Fast, robust iterative IK solver for serial chains
	 * 
	 * @param JointPositions - Current joint positions in world space
	 * @param JointAngles - Current joint angles (degrees), will be modified
	 * @param TargetTransform - Desired end effector transform in world space
	 * @param JointLimits - Min/max angle limits for each joint (degrees)
	 * @param MaxIterations - Maximum number of iterations
	 * @param PositionTolerance - Position convergence tolerance (cm)
	 * @param RotationTolerance - Rotation convergence tolerance (degrees)
	 * @param StepSize - Step size for damping (0-1, lower = smoother)
	 * @return IK solve result with success status and joint angles
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Inverse Kinematics")
	static FIKSolveResult SolveIK_FABRIK(
		const TArray<FVector>& JointPositions,
		TArray<float>& JointAngles,
		const FTransform& TargetTransform,
		const TArray<FVector2D>& JointLimits,
		int32 MaxIterations = 10,
		float PositionTolerance = 1.0f,
		float RotationTolerance = 5.0f,
		float StepSize = 0.3f);

	/**
	 * Compute forward kinematics from base transform, joint angles, and local joint transforms
	 * 
	 * @param BaseTransform - World transform of the robot base
	 * @param JointAngles - Joint angles in DEGREES (will be converted internally)
	 * @param JointLocalTransforms - Local transform of each joint relative to parent (at zero angle)
	 * @param JointAxes - Rotation axis for each joint in local space
	 * @param EndEffectorOffset - Local transform from last joint to end effector
	 * @param bDebugLog - Enable detailed debug logging
	 * @return End effector world transform
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Inverse Kinematics")
	static FTransform ComputeForwardKinematics(
		const FTransform& BaseTransform,
		const TArray<float>& JointAngles,
		const TArray<FTransform>& JointLocalTransforms,
		const TArray<FVector>& JointAxes,
		const FTransform& EndEffectorOffset,
		bool bDebugLog = false);

	/**
	 * Solve inverse kinematics using Damped Least Squares (DLS) method with Eigen
	 * High-quality IK solver with numerical Jacobian, joint limits, task-space masking, and null-space optimization
	 * 
	 * @param JointAngles - Current joint angles (degrees), will be modified
	 * @param BaseTransform - World transform of robot base
	 * @param JointLocalTransforms - Local transforms of each joint relative to parent (at zero angle)
	 * @param JointAxes - Rotation axis for each joint in local space
	 * @param EndEffectorOffset - Local transform from last joint to end effector
	 * @param TargetTransform - Desired end effector transform in world space
	 * @param JointLimits - Min/max angle limits for each joint (degrees)
	 * @param TaskSpaceMask - Which DOFs to control [X,Y,Z,Roll,Pitch,Yaw], true=control
	 * @param JointWeights - Weight matrix for joints (higher = penalize movement, empty = equal)
	 * @param DampingFactor - DLS damping factor (0.01-1.0, higher = more stable but slower)
	 * @param StepClip - Maximum joint velocity step per iteration (radians)
	 * @param MaxIterations - Number of IK iterations per solve
	 * @param PositionTolerance - Position convergence tolerance (cm)
	 * @param RotationTolerance - Rotation convergence tolerance (degrees)
	 * @param NullSpaceBias - Optional null-space joint angle bias for redundancy resolution (empty = disabled)
	 * @param NullSpaceGain - Gain for null-space projection (0-1, 0=disabled)
	 * @return IK solve result with success status and joint angles
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Inverse Kinematics")
	static FIKSolveResult SolveIK_DLS(
		TArray<float>& JointAngles,
		const FTransform& BaseTransform,
		const TArray<FTransform>& JointLocalTransforms,
		const TArray<FVector>& JointAxes,
		const FTransform& EndEffectorOffset,
		const FTransform& TargetTransform,
		const TArray<FVector2D>& JointLimits,
		const TArray<bool>& TaskSpaceMask,
		const TArray<float>& JointWeights,
		float DampingFactor,
		float StepClip,
		int32 MaxIterations,
		float PositionTolerance,
		float RotationTolerance,
		const TArray<float>& NullSpaceBias,
		float NullSpaceGain);

	/**
	 * Solve inverse kinematics using Jacobian transpose method (legacy)
	 * @deprecated Use SolveIK_DLS instead for better convergence
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Inverse Kinematics")
	static FIKSolveResult SolveIK_JacobianTranspose(
		TArray<float>& JointAngles,
		const TArray<FTransform>& JointTransforms,
		const TArray<FVector>& JointAxes,
		const FTransform& EndEffectorTransform,
		const FTransform& TargetTransform,
		const TArray<FVector2D>& JointLimits,
		int32 MaxIterations = 10,
		float PositionTolerance = 1.0f,
		float RotationTolerance = 5.0f,
		float StepSize = 0.1f);

	/**
	 * Clamp joint angle to limits
	 * @param Angle - Angle to clamp (degrees)
	 * @param Min - Minimum limit (degrees)
	 * @param Max - Maximum limit (degrees)
	 * @return Clamped angle
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Inverse Kinematics")
	static float ClampJointAngle(float Angle, float Min, float Max);

	/**
	 * Calculate distance error between two transforms
	 * @param Transform1 - First transform
	 * @param Transform2 - Second transform
	 * @param OutPositionError - Position error (cm)
	 * @param OutRotationError - Rotation error (degrees)
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Inverse Kinematics")
	static void CalculateTransformError(
		const FTransform& Transform1,
		const FTransform& Transform2,
		float& OutPositionError,
		float& OutRotationError);

	/**
	 * Normalize angle to [-180, 180] range
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Inverse Kinematics")
	static float NormalizeAngle(float Angle);
};
