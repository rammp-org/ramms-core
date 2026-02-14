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

/** Single joint in a FABRIK chain with 1-DOF revolute constraint */
USTRUCT(BlueprintType)
struct FFABRIKJoint
{
	GENERATED_BODY()

	/** Joint position in world space (updated during solve) */
	UPROPERTY(BlueprintReadWrite)
	FVector Position = FVector::ZeroVector;

	/** Parent joint index (-1 for base/root) */
	UPROPERTY(BlueprintReadWrite)
	int32 ParentIndex = -1;

	/** Rotation axis in parent's local frame (1-DOF revolute) */
	UPROPERTY(BlueprintReadWrite)
	FVector AxisDirection = FVector::XAxisVector;

	/** Minimum angle limit in degrees */
	UPROPERTY(BlueprintReadWrite)
	float MinAngleLimit = -180.0f;

	/** Maximum angle limit in degrees */
	UPROPERTY(BlueprintReadWrite)
	float MaxAngleLimit = 180.0f;

	/** Current joint angle in degrees */
	UPROPERTY(BlueprintReadWrite)
	float CurrentAngle = 0.0f;

	/** Link length to next joint (cm) */
	UPROPERTY(BlueprintReadWrite)
	float LinkLength = 0.0f;

	FFABRIKJoint()
		: Position(FVector::ZeroVector)
		, ParentIndex(-1)
		, AxisDirection(FVector::XAxisVector)
		, MinAngleLimit(-180.0f)
		, MaxAngleLimit(180.0f)
		, CurrentAngle(0.0f)
		, LinkLength(0.0f)
	{}
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

	/**
	 * FABRIK (Forward And Backward Reaching Inverse Kinematics) solver with hard joint limits.
	 * 
	 * Solves IK for a serial chain of 1-DOF revolute joints with constraint limits.
	 * Uses iterative position-based approach with hard clamping to joint limits.
	 * 
	 * @param BaseTransform World-space transform of the robot base
	 * @param CurrentAnglesDeg Current joint angles in degrees
	 * @param JointLocalTransforms Local transforms from parent to each joint (at q=0)
	 * @param JointAxesLocal Rotation axis for each joint in joint's local frame
	 * @param JointLimitsDeg Min/max angle limits for each joint (X=min, Y=max)
	 * @param EndEffectorOffset Transform from last joint to end-effector
	 * @param TargetEndEffectorWorld Target end-effector transform in world space
	 * @param TaskSpaceMask6 Which DOFs to solve [X,Y,Z,Roll,Pitch,Yaw]
	 * @param MaxIterations Maximum FABRIK iterations
	 * @param PositionToleranceCm Position convergence tolerance in cm
	 * @param RotationToleranceDeg Rotation convergence tolerance in degrees
	 * @param AngleGain Multiplier for per-joint angle updates (FABRIK aggressiveness)
	 * @param MaxAngleStepDeg Maximum per-joint step (deg) for stability near target
	 * @param LimitEscapeDeg Max step (deg) to move off joint limits if stuck
	 * @param OrientationIterations Number of orientation refinement iterations
	 * @param OrientationGain Gain multiplier for orientation refinement
	 * @return Solve result with joint angles and convergence info
	 */
	UFUNCTION(BlueprintCallable, Category="RAMMS|IK|FABRIK")
	static FIKSolveResult SolveIK_FABRIK(
		const FTransform& BaseTransform,
		const TArray<float>& CurrentAnglesDeg,
		const TArray<FTransform>& JointLocalTransforms,
		const TArray<FVector>& JointAxesLocal,
		const TArray<FVector2D>& JointLimitsDeg,
		const FTransform& EndEffectorOffset,
		const FTransform& TargetEndEffectorWorld,
		const TArray<bool>& TaskSpaceMask6,
		int32 MaxIterations,
		float PositionToleranceCm,
		float RotationToleranceDeg,
		float AngleGain,
		float MaxAngleStepDeg,
		float LimitEscapeDeg,
		int32 OrientationIterations,
		float OrientationGain);

	/**
	 * CCD (Cyclic Coordinate Descent) solver with joint axis constraints.
	 */
	UFUNCTION(BlueprintCallable, Category="RAMMS|IK|CCD")
	static FIKSolveResult SolveIK_CCD(
		const FTransform& BaseTransform,
		const TArray<float>& CurrentAnglesDeg,
		const TArray<FTransform>& JointLocalTransforms,
		const TArray<FVector>& JointAxesLocal,
		const TArray<FVector2D>& JointLimitsDeg,
		const FTransform& EndEffectorOffset,
		const FTransform& TargetEndEffectorWorld,
		const TArray<bool>& TaskSpaceMask6,
		int32 MaxIterations,
		float PositionToleranceCm,
		float RotationToleranceDeg,
		float PositionGain,
		float OrientationGain,
		float MaxAngleStepDeg);

	/**
	 * UE built-in FABRIK solver (AnimationCore) with joint angle reconstruction.
	 */
	UFUNCTION(BlueprintCallable, Category="RAMMS|IK|UE")
	static FIKSolveResult SolveIK_UEFabrik(
		const FTransform& BaseTransform,
		const TArray<float>& CurrentAnglesDeg,
		const TArray<FTransform>& JointLocalTransforms,
		const TArray<FVector>& JointAxesLocal,
		const TArray<FVector2D>& JointLimitsDeg,
		const FTransform& EndEffectorOffset,
		const FTransform& TargetEndEffectorWorld,
		const TArray<bool>& TaskSpaceMask6,
		int32 MaxIterations,
		float PositionToleranceCm,
		float RotationToleranceDeg,
		float LimitEscapeDeg,
		bool bAxesInParentFrame);
};
