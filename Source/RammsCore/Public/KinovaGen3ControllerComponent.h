// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "KinovaGen3ControllerComponent.generated.h"

class USkeletalMeshComponent;

/**
 * Control mode for arm joints
 */
UENUM(BlueprintType)
enum class EJointControlMode : uint8
{
	PositionControl UMETA(DisplayName = "Position Control"),
	VelocityControl UMETA(DisplayName = "Velocity Control"),
	TorqueControl UMETA(DisplayName = "Torque Control")
};

/**
 * Control mode for the arm (joint-level vs end-effector level)
 */
UENUM(BlueprintType)
enum class EArmControlMode : uint8
{
	JointControl UMETA(DisplayName = "Joint Control (Direct)"),
	EndEffectorControl UMETA(DisplayName = "End Effector Control (IK)")
};

/**
 * Which angular axis of the constraint is being controlled
 */
UENUM(BlueprintType)
enum class EConstraintAxis : uint8
{
	Swing1 UMETA(DisplayName = "Swing1 (secondary rotation)"),
	Swing2 UMETA(DisplayName = "Swing2 (tertiary rotation)"),
	Twist UMETA(DisplayName = "Twist (primary rotation)")
};

/**
 * Configuration for a single revolute joint
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRevoluteJointConfig
{
	GENERATED_BODY()

	/** Bone name for this joint */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint")
	FName BoneName = NAME_None;

	/** Physics constraint name (if different from bone name) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint")
	FName ConstraintName = NAME_None;

	/** Which constraint axis is controlled (auto-detected or manual) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint")
	EConstraintAxis ControlledAxis = EConstraintAxis::Twist;

	/** Invert rotation axis direction for IK (if axis points backwards) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint")
	bool bInvertAxisForIK = false;

	/** Angle offset (degrees) - calibration between constraint zero and reference skeleton pose */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint")
	float AngleOffset = 0.0f;
	
	/** Computed Frame1 rotation offset (degrees) - automatic offset from constraint Frame1 orientation */
	float ComputedFrameOffset = 0.0f;

	/** Target angle in degrees */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint")
	float TargetAngle = 0.0f;

	/** Current angle in degrees (read from constraint, runtime) */
	UPROPERTY(BlueprintReadOnly, Category = "Joint")
	float CurrentAngle = 0.0f;
	
	/** Smoothed angle for speed limiting (interpolates toward TargetAngle) */
	float SmoothedAngle = 0.0f;

	/** Maximum angular velocity (degrees/sec) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint", meta = (ClampMin = "0.1"))
	float MaxAngularSpeed = 45.0f;

	/** Speed multiplier (0-1) for dynamic speed adjustment */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float SpeedMultiplier = 1.0f;

	/** Maximum torque (N⋅m) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint", meta = (ClampMin = "0.1"))
	float MaxTorque = 39.0f; // Kinova Gen3 typical max torque

	/** Angular position drive strength (spring constant) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint|PD Control", meta = (ClampMin = "0.0"))
	float PositionStrength = 5000000.0f;

	/** Angular position drive damping */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint|PD Control", meta = (ClampMin = "0.0"))
	float PositionDamping = 0.0f;

	/** Minimum angle limit (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint|Limits")
	float MinAngleLimit = -180.0f;

	/** Maximum angle limit (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint|Limits")
	float MaxAngleLimit = 180.0f;

	/** Enable software limits */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint|Limits")
	bool bEnableSoftwareLimits = true;

	FRevoluteJointConfig()
		: BoneName(NAME_None)
		, ConstraintName(NAME_None)
		, TargetAngle(0.0f)
		, CurrentAngle(0.0f)
		, MaxAngularSpeed(45.0f)
		, SpeedMultiplier(1.0f)
		, MaxTorque(39.0f)
		, PositionStrength(5000000.0f)
		, PositionDamping(0.0f)
		, MinAngleLimit(-180.0f)
		, MaxAngleLimit(180.0f)
		, bEnableSoftwareLimits(true)
	{}
};

/**
 * End-effector state information
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FEndEffectorState
{
	GENERATED_BODY()

	/** End-effector position in world space */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "End Effector")
	FVector Position = FVector::ZeroVector;

	/** End-effector rotation in world space */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "End Effector")
	FRotator Rotation = FRotator::ZeroRotator;

	/** Linear velocity (cm/s) */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "End Effector")
	FVector LinearVelocity = FVector::ZeroVector;

	/** Angular velocity (degrees/s) */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "End Effector")
	FVector AngularVelocity = FVector::ZeroVector;

	FEndEffectorState()
		: Position(FVector::ZeroVector)
		, Rotation(FRotator::ZeroRotator)
		, LinearVelocity(FVector::ZeroVector)
		, AngularVelocity(FVector::ZeroVector)
	{}
};

/**
 * Controller component for Kinova Gen3 robotic arm
 * Manages joint control, applies forces, tracks forward kinematics
 */
UCLASS(ClassGroup=(Ramms), meta=(BlueprintSpawnableComponent))
class RAMMSCORE_API UKinovaGen3ControllerComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UKinovaGen3ControllerComponent();

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// ========== Configuration ==========

	/** Name of the skeletal mesh component containing the arm bones (leave empty to auto-find first skeletal mesh) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Configuration")
	FName SkeletalMeshComponentName = NAME_None;

	/** When enabled, automatically populates joints when SkeletalMeshComponentName changes */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Configuration")
	bool bAutoPopulateOnSkeletalMeshChange = true;

	/** Name of the end-effector bone (typically the wrist or tool flange) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Configuration")
	FName EndEffectorBoneName = FName("end_effector");

	/** Arm control mode (joint control vs end effector IK control) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control")
	EArmControlMode ArmControlMode = EArmControlMode::JointControl;

	/** Control mode for all joints */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control")
	EJointControlMode ControlMode = EJointControlMode::PositionControl;

	/** Optional target actor to follow for IK (if set, overrides TargetEndEffectorTransform) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector")
	AActor* TargetActor = nullptr;

	/** Target end effector transform (world space) - used when ArmControlMode is EndEffectorControl and TargetActor is null */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector")
	FTransform TargetEndEffectorTransform;

	/** IK convergence tolerance (cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector", meta = (ClampMin = "0.01"))
	float IKPositionTolerance = 1.0f;

	/** IK convergence tolerance (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector", meta = (ClampMin = "0.1"))
	float IKRotationTolerance = 5.0f;

	/** IK iterations per frame (DLS performs multiple iterations internally) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector", meta = (ClampMin = "1", ClampMax = "500"))
	int32 MaxIKIterations = 300;

	/** DLS damping factor (0.01-1.0, higher = more stable but slower convergence) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector", meta = (ClampMin = "0.01", ClampMax = "1.0"))
	float IKDampingFactor = 0.1f;

	/** Maximum joint velocity step per iteration (radians, prevents large jumps) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector", meta = (ClampMin = "0.01", ClampMax = "1.0"))
	float IKStepClip = 0.2f;

	/** Task-space mask: control which DOFs [X, Y, Z, Roll, Pitch, Yaw] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector")
	TArray<bool> TaskSpaceMask = {true, true, true, false, false, true}; // Position + Yaw only by default

	/** Joint weights for DLS (empty = equal weights, higher = prefer not to move this joint) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector")
	TArray<float> JointWeights;

	/** Enable null-space optimization for redundant manipulators */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector")
	bool bEnableNullSpaceOptimization = false;

	/** Null-space gain (0-1, strength of bias toward preferred joint configuration) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bEnableNullSpaceOptimization"))
	float NullSpaceGain = 0.1f;

	/** Preferred joint angles for null-space optimization (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector", meta = (EditCondition = "bEnableNullSpaceOptimization"))
	TArray<float> NullSpaceBias;

	/** Use FABRIK solver instead of DLS (for testing/comparison) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|FABRIK")
	bool bUseFABRIK = true;

	/** Maximum FABRIK iterations per solve */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|FABRIK", meta = (ClampMin = "1", ClampMax = "100", EditCondition = "bUseFABRIK"))
	int32 FABRIKMaxIterations = 15;

	/** Position convergence tolerance for FABRIK (cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|FABRIK", meta = (ClampMin = "0.01", EditCondition = "bUseFABRIK"))
	float FABRIKPositionTolerance = 1.0f;

	/** Joint configurations (5-7 joints for Kinova Gen3) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Joints")
	TArray<FRevoluteJointConfig> Joints;

	// ========== State ==========

	/** Current end-effector state */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Arm|State")
	FEndEffectorState EndEffectorState;
	
	/** Last IK solve result (for debugging) */
	float LastIKPositionError = 0.0f;
	float LastIKRotationError = 0.0f;
	int32 LastIKIterations = 0;
	bool bLastIKSuccess = false;

	// ========== Blueprint API ==========

	/**
	 * Set target angle for a specific joint by index
	 * @param JointIndex - Index of the joint in the Joints array
	 * @param TargetAngle - Target angle in degrees
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void SetJointTarget(int32 JointIndex, float TargetAngle);

	/**
	 * Set target angles for all joints
	 * @param TargetAngles - Array of target angles in degrees (must match joint count)
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void SetAllJointTargets(const TArray<float>& TargetAngles);

	/**
	 * Set speed multiplier for a specific joint
	 * @param JointIndex - Index of the joint in the Joints array
	 * @param SpeedMultiplier - Speed multiplier (0-1)
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void SetJointSpeedMultiplier(int32 JointIndex, float SpeedMultiplier);

	/**
	 * Set max angular speed for a specific joint
	 * @param JointIndex - Index of the joint in the Joints array
	 * @param MaxSpeed - Maximum angular speed in degrees/sec
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void SetJointMaxSpeed(int32 JointIndex, float MaxSpeed);

	/**
	 * Get current joint angle
	 * @param JointIndex - Index of the joint in the Joints array
	 * @return Current angle in degrees
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Kinova Gen3")
	float GetJointAngle(int32 JointIndex) const;

	/**
	 * Get all current joint angles
	 * @return Array of current angles in degrees
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Kinova Gen3")
	TArray<float> GetAllJointAngles() const;

	/**
	 * Get end-effector state
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Kinova Gen3")
	FEndEffectorState GetEndEffectorState() const { return EndEffectorState; }

	/**
	 * Set target end effector transform (world space) for IK control
	 * @param TargetTransform - Desired end effector transform in world space
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void SetEndEffectorTarget(const FTransform& TargetTransform);

	/**
	 * Set target end effector position (world space) for IK control, preserving current orientation
	 * @param TargetPosition - Desired end effector position in world space
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void SetEndEffectorTargetPosition(const FVector& TargetPosition);

	/**
	 * Set target end effector rotation (world space) for IK control, preserving current position
	 * @param TargetRotation - Desired end effector rotation in world space
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void SetEndEffectorTargetRotation(const FRotator& TargetRotation);

	/**
	 * Set target end effector transform relative to the arm base (skeletal mesh component)
	 * @param RelativeTransform - Target transform relative to arm base
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void SetEndEffectorTargetRelativeToBase(const FTransform& RelativeTransform);

	/**
	 * Set target end effector position relative to the arm base (skeletal mesh component)
	 * @param RelativePosition - Target position relative to arm base
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void SetEndEffectorTargetPositionRelativeToBase(const FVector& RelativePosition);

	/**
	 * Set target end effector transform relative to the owner actor
	 * @param RelativeTransform - Target transform relative to owner actor
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void SetEndEffectorTargetRelativeToActor(const FTransform& RelativeTransform);

	/**
	 * Set target end effector position relative to the owner actor
	 * @param RelativePosition - Target position relative to owner actor
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void SetEndEffectorTargetPositionRelativeToActor(const FVector& RelativePosition);

	/**
	 * Move end effector by offset from current target position
	 * @param Offset - Offset in world space
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void MoveEndEffectorTargetBy(const FVector& Offset);

	/**
	 * Move end effector by offset in its current orientation frame
	 * @param LocalOffset - Offset in end effector's local space (forward, right, up)
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void MoveEndEffectorTargetByLocal(const FVector& LocalOffset);

	/**
	 * Set control mode for all joints
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void SetControlMode(EJointControlMode Mode) { ControlMode = Mode; }

	/**
	 * Auto-populate joints from all physics constraints on the skeletal mesh (clears existing joints)
	 * Use this in the editor to automatically configure the Joints array
	 */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Ramms|Kinova Gen3")
	void AutoPopulateJointsFromConstraints();

	/**
	 * Auto-populate joints from all physics constraints on the skeletal mesh
	 * @param bOverwriteExisting - If true, clears existing joints first
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void AutoPopulateJoints(bool bOverwriteExisting = true);

	/**
	 * Auto-detect which constraint axis is free/limited (not locked) for a joint
	 * @param JointIndex - Index of the joint to configure
	 * @return True if a free/limited axis was found
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	bool AutoDetectJointAxis(int32 JointIndex);

	/**
	 * Auto-detect axes for all joints
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void AutoDetectAllJointAxes();

	/**
	 * Initialize/reinitialize constraint drives for all joints
	 * Call this after changing joint configuration or control parameters
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void ReinitializeConstraints();
	
	/**
	 * Calibrate angle offsets - captures current constraint angles as offsets
	 * Call this when the arm is in the reference skeleton pose
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Kinova Gen3")
	void CalibrateAngleOffsets();

	/**
	 * Validate FK accuracy by comparing FK prediction to actual bone positions
	 * Returns maximum error in cm across all joints
	 */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Ramms|Kinova Gen3|Debug")
	float ValidateForwardKinematics();

	/**
	 * Auto-calibrate angle offsets to make FK match actual bone positions
	 * Computes what offsets are needed to make FK accurate
	 */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Ramms|Kinova Gen3|Debug")
	void AutoCalibrateFK();

	/**
	 * Print detailed FK diagnostic information
	 */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Ramms|Kinova Gen3|Debug")
	void PrintFKDiagnostics();

	/**
	 * Validate FABRIK solution against physics asset constraints
	 * Returns maximum constraint violation in degrees
	 */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Ramms|Kinova Gen3|Debug")
	float ValidateFABRIKConstraints();

	// ========== Debug ==========

	/** Enable debug logging */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Debug")
	bool bEnableDebugLogging = false;

	/** Enable on-screen debug display */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Debug")
	bool bEnableDebugDisplay = false;

	/** Show joint coordinate frames */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Debug")
	bool bShowJointFrames = false;

private:
	// Cached skeletal mesh component reference
	UPROPERTY(Transient)
	USkeletalMeshComponent* SkeletalMeshComponent = nullptr;
	
	// Cached IK data (computed once per frame in UpdateInverseKinematics)
	FTransform CachedEndEffectorOffset = FTransform::Identity;
	TArray<FTransform> CachedJointLocalTransforms;
	TArray<FVector> CachedJointAxesLocal;

	/** Update joint positions using position control */
	void UpdatePositionControl(float DeltaTime);

	/** Update joint using velocity control */
	void UpdateVelocityControl(float DeltaTime);

	/** Update joint using torque control */
	void UpdateTorqueControl(float DeltaTime);

	/** Update using inverse kinematics to reach end effector target */
	void UpdateInverseKinematics(float DeltaTime);
	
	/** Compute empirical rotation axis by perturbing the constraint and observing bone movement */
	FVector ComputeEmpiricalRotationAxis(int32 JointIndex, const FTransform& ParentWorldTransform);

	/** Initialize constraint drives for all joints (one-time setup) */
	void InitializeJointConstraints();

	/** Cache joint rotation axes for FK/IK without modifying constraint drives */
	void CacheJointAxes();

	/** Apply settings to a revolute joint */
	void ApplyJointSettings(FRevoluteJointConfig& Joint, float DeltaTime);

	/** Update end-effector state from forward kinematics */
	void UpdateEndEffectorState();

	/** Calculate forward kinematics for end effector given joint angles */
	FTransform CalculateForwardKinematics(const TArray<float>& JointAngles) const;

	/** Clamp angle to joint limits if enabled */
	float ClampToLimits(const FRevoluteJointConfig& Joint, float Angle) const;

	/** Get the current angle from a constraint for a specific axis */
	float GetConstraintAngle(FConstraintInstance* Constraint, EConstraintAxis Axis) const;

	/** Set the target angle on a constraint for a specific axis */
	void SetConstraintAngle(FConstraintInstance* Constraint, EConstraintAxis Axis, float AngleDegrees) const;

	/** Check if a constraint axis is free or limited (not locked) */
	bool IsAxisFreeOrLimited(FConstraintInstance* Constraint, EConstraintAxis Axis) const;

	/** Get the angular range (in degrees) for a constraint axis */
	float GetConstraintAxisRange(FConstraintInstance* Constraint, EConstraintAxis Axis) const;

	/** Detect if an axis should be inverted for IK based on constraint frame orientation */
	bool ShouldInvertAxisForIK(FConstraintInstance* Constraint, FName BoneName, EConstraintAxis Axis) const;

	/** Debug visualization */
	void DebugDraw();
};
