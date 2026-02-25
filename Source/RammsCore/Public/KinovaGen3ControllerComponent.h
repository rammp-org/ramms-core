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
	TorqueControl	UMETA(DisplayName = "Torque Control")
};

/**
 * Control mode for the arm (joint-level vs end-effector level)
 */
UENUM(BlueprintType)
enum class EArmControlMode : uint8
{
	JointControl	   UMETA(DisplayName = "Joint Control (Direct)"),
	EndEffectorControl UMETA(DisplayName = "End Effector Control (IK)")
};

/**
 * IK solver selection
 */
UENUM(BlueprintType)
enum class EIKSolverType : uint8
{
	DLS	   UMETA(DisplayName = "Damped Least Squares (DLS)"),
	FABRIK UMETA(DisplayName = "FABRIK"),
	CCD	   UMETA(DisplayName = "CCD")
};

/**
 * Which angular axis of the constraint is being controlled
 */
UENUM(BlueprintType)
enum class EConstraintAxis : uint8
{
	Swing1 UMETA(DisplayName = "Swing1 (secondary rotation)"),
	Swing2 UMETA(DisplayName = "Swing2 (tertiary rotation)"),
	Twist  UMETA(DisplayName = "Twist (primary rotation)")
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

	/** Tracks whether SmoothedAngle has been initialized from the current joint angle */
	bool bSmoothedAngleInitialized = false;

	/** Maximum angular velocity (degrees/sec) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint", meta = (ClampMin = "0.1"))
	float MaxAngularSpeed = 90.0f;

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
		, MaxAngularSpeed(90.0f)
		, SpeedMultiplier(1.0f)
		, MaxTorque(39.0f)
		, PositionStrength(5000000.0f)
		, PositionDamping(0.0f)
		, MinAngleLimit(-180.0f)
		, MaxAngleLimit(180.0f)
		, bEnableSoftwareLimits(true)
	{
	}
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
	{
	}
};

/**
 * Controller component for Kinova Gen3 robotic arm
 * Manages joint control, applies forces, tracks forward kinematics
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
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
	FName SkeletalMeshComponentName = FName("ArmSkMesh");

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
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Target")
	AActor* TargetActor = nullptr;

	/** Target end effector transform (world space) - used when ArmControlMode is EndEffectorControl and TargetActor is null */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Target")
	FTransform TargetEndEffectorTransform;

	/** IK solver technique */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver")
	EIKSolverType IKSolverType = EIKSolverType::DLS;

	/** IK convergence tolerance (cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|Common", meta = (ClampMin = "0.01"))
	float IKPositionTolerance = 1.0f;

	/** IK convergence tolerance (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|Common", meta = (ClampMin = "0.1"))
	float IKRotationTolerance = 5.0f;

	/** Task-space mask: control which DOFs [X, Y, Z, Roll, Pitch, Yaw] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|Common")
	TArray<bool> TaskSpaceMask = { true, true, true, true, true, true };

	/** Threshold for detecting target position changes (cm). Smaller = more sensitive. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|Target Change", meta = (ClampMin = "0.0"))
	float IKTargetChangePosThreshold = 0.01f;

	/** Threshold for detecting target rotation changes (degrees). Smaller = more sensitive. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|Target Change", meta = (ClampMin = "0.0"))
	float IKTargetChangeRotThreshold = 0.1f;

	/** IK iterations per frame (DLS performs multiple iterations internally) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|DLS", meta = (ClampMin = "1", ClampMax = "500", EditCondition = "IKSolverType == EIKSolverType::DLS"))
	int32 MaxIKIterations = 300;

	/** DLS damping factor (0.01-1.0, higher = more stable but slower convergence) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|DLS", meta = (ClampMin = "0.01", ClampMax = "1.0", EditCondition = "IKSolverType == EIKSolverType::DLS"))
	float IKDampingFactor = 0.1f;

	/** Maximum joint velocity step per iteration (radians, prevents large jumps) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|DLS", meta = (ClampMin = "0.01", ClampMax = "1.0", EditCondition = "IKSolverType == EIKSolverType::DLS"))
	float IKStepClip = 0.2f;

	/** Joint weights for DLS (empty = equal weights, higher = prefer not to move this joint) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|DLS", meta = (EditCondition = "IKSolverType == EIKSolverType::DLS"))
	TArray<float> JointWeights;

	/** Enable null-space optimization for redundant manipulators */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|DLS", meta = (EditCondition = "IKSolverType == EIKSolverType::DLS"))
	bool bEnableNullSpaceOptimization = false;

	/** Null-space gain (0-1, strength of bias toward preferred joint configuration) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|DLS", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "IKSolverType == EIKSolverType::DLS && bEnableNullSpaceOptimization"))
	float NullSpaceGain = 0.1f;

	/** Preferred joint angles for null-space optimization (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|DLS", meta = (EditCondition = "IKSolverType == EIKSolverType::DLS && bEnableNullSpaceOptimization"))
	TArray<float> NullSpaceBias;

	/** Maximum FABRIK iterations per solve */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|FABRIK", meta = (ClampMin = "1", ClampMax = "500", EditCondition = "IKSolverType == EIKSolverType::FABRIK"))
	int32 FABRIKMaxIterations = 200;

	/** Position convergence tolerance for FABRIK (cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|FABRIK", meta = (ClampMin = "0.01", EditCondition = "IKSolverType == EIKSolverType::FABRIK"))
	float FABRIKPositionTolerance = 1.0f;

	/** Angle gain multiplier for per-joint scalar DLS (1.0 = optimal step, >1 more aggressive) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|FABRIK", meta = (ClampMin = "0.1", ClampMax = "5.0", EditCondition = "IKSolverType == EIKSolverType::FABRIK"))
	float FABRIKAngleGain = 1.0f;

	/** Maximum angle step per joint per iteration (degrees) - like DLS StepClip */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|FABRIK", meta = (ClampMin = "0.1", ClampMax = "45.0", EditCondition = "IKSolverType == EIKSolverType::FABRIK"))
	float FABRIKMaxAngleStepDeg = 12.0f;

	/** Maximum "escape" step (deg) to move off joint limits if stuck */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|FABRIK", meta = (ClampMin = "0.0", ClampMax = "20.0", EditCondition = "IKSolverType == EIKSolverType::FABRIK"))
	float FABRIKLimitEscapeDeg = 2.0f;

	/** Orientation refinement iterations after position solve */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|FABRIK", meta = (ClampMin = "0", ClampMax = "50", EditCondition = "IKSolverType == EIKSolverType::FABRIK"))
	int32 FABRIKOrientationIterations = 10;

	/** Orientation refinement gain (scaled relative to position gain) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|FABRIK", meta = (ClampMin = "0.1", ClampMax = "2.0", EditCondition = "IKSolverType == EIKSolverType::FABRIK"))
	float FABRIKOrientationGain = 0.5f;

	/** Maximum CCD iterations per solve */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|CCD", meta = (ClampMin = "1", ClampMax = "500", EditCondition = "IKSolverType == EIKSolverType::CCD"))
	int32 CCDMaxIterations = 100;

	/** CCD position gain (scales the exact hinge-plane angle; 1.0 = full correction) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|CCD", meta = (ClampMin = "0.1", ClampMax = "5.0", EditCondition = "IKSolverType == EIKSolverType::CCD"))
	float CCDPositionGain = 1.0f;

	/** CCD orientation gain (blended with position; 0 = position only) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|CCD", meta = (ClampMin = "0.0", ClampMax = "5.0", EditCondition = "IKSolverType == EIKSolverType::CCD"))
	float CCDOrientationGain = 0.3f;

	/** Maximum CCD angle step per joint per iteration (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Control|End Effector|Solver|CCD", meta = (ClampMin = "0.1", ClampMax = "45.0", EditCondition = "IKSolverType == EIKSolverType::CCD"))
	float CCDMaxAngleStepDeg = 12.0f;

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
	bool  bLastIKSuccess = false;

	/** Actual skeletal mesh error (from previous frame) vs IK solver prediction */
	float LastActualPosError = 0.0f;
	float LastActualRotError = 0.0f;
	bool  bLastIKSolverSuccess = false; // What the IK solver reported (before actual check)

	/** FK model validation (for debugging FK mismatch) */
	float	LastFKvsActualError = 0.0f;
	FVector LastFKEndEffectorPos = FVector::ZeroVector;
	FVector LastActualEndEffectorPos = FVector::ZeroVector;

	/** Cached target transform to detect changes and avoid unnecessary IK solving */
	FTransform	  LastIKTargetTransform = FTransform::Identity;
	FVector		  LastSolveBaseLocation = FVector::ZeroVector;
	bool		  bIKTargetInitialized = false;
	bool		  bIKTargetSatisfied = false;
	EIKSolverType LastIKSolverType = EIKSolverType::DLS;
	bool		  bIKSolverTypeInitialized = false;

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
	FEndEffectorState GetEndEffectorState() const
	{
		return EndEffectorState;
	}

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
	void SetControlMode(EJointControlMode Mode)
	{
		ControlMode = Mode;
	}

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
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Ramms|Kinova Gen3")
	void AutoPopulateJoints(bool bOverwriteExisting = true);

	/** Build joints from a skeletal mesh's PhysicsAsset constraints (editor-safe) */
	bool BuildJointsFromPhysicsAsset(USkeletalMesh* SkeletalMeshAsset, TArray<FRevoluteJointConfig>& OutJoints) const;

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

	/** If true, keep per-joint TargetAngle values set in editor when PIE begins */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Joint Control")
	bool bPreserveJointTargetsOnBeginPlay = true;

	/** If true (and bPreserveJointTargetsOnBeginPlay is true), snap constraints
	 *  directly to their per-joint TargetAngle on BeginPlay so the arm starts
	 *  in the desired configuration without having to drive there over time. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Arm|Joint Control")
	bool bSnapToTargetsOnBeginPlay = true;

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
	FTransform		   CachedEndEffectorOffset = FTransform::Identity;
	TArray<FTransform> CachedJointLocalTransforms;
	TArray<FVector>	   CachedJointAxesLocal;

	// FK local transforms are calibrated from runtime physics body positions
	// to eliminate small offsets between skeleton reference pose and constraint solver
	bool  bFKLocalTransformsCalibrated = false;
	int32 FKCalibrationTickCounter = 0;

	/** Smoothing factor for per-tick FK calibration (0-1). Lower = smoother but slower tracking.
	 *  0.05 is very smooth (~1s convergence at 60fps), 0.2 is responsive (~0.2s). */
	UPROPERTY(EditAnywhere, Category = "IK", meta = (ClampMin = "0.01", ClampMax = "1.0"))
	float CalibrationSmoothingAlpha = 0.1f;

	/** Update joint positions using position control */
	void UpdatePositionControl(float DeltaTime);

	/** Update joint using velocity control */
	void UpdateVelocityControl(float DeltaTime);

	/** Update joint using torque control */
	void UpdateTorqueControl(float DeltaTime);

	/** Update using inverse kinematics to reach end effector target */
	void UpdateInverseKinematics(float DeltaTime);

	/** Initialize constraint drives for all joints (one-time setup) */
	void InitializeJointConstraints();

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
