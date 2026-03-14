// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsSkeletalPoseComponent.generated.h"

class UPoseableMeshComponent;
class URammsJointPoseAsset;

/**
 * Type of kinematic joint: revolute (rotation) or prismatic (translation).
 */
UENUM(BlueprintType)
enum class EKinematicJointType : uint8
{
	/** Rotation around an axis (degrees). Used for arm joints, caster swing arms, etc. */
	Revolute UMETA(DisplayName = "Revolute (Rotation)"),

	/** Translation along an axis (centimeters). Used for linear actuators, lifts, etc. */
	Prismatic UMETA(DisplayName = "Prismatic (Translation)")
};

/**
 * Configuration for a single kinematic joint on a poseable skeletal mesh.
 * Maps a logical joint name to a skeleton bone and defines how to drive it.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FKinematicJointConfig
{
	GENERATED_BODY()

	/**
	 * Name of the UPoseableMeshComponent this joint targets.
	 * If empty, uses the first (or only) poseable mesh on the actor.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint")
	FName MeshComponentName = NAME_None;

	/** Skeleton bone name to control */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint")
	FName BoneName = NAME_None;

	/** Logical joint name for external APIs (e.g. "Shoulder", "Elbow"). Defaults to BoneName if empty. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint")
	FName JointName = NAME_None;

	/** Whether this is a rotational or translational joint */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint")
	EKinematicJointType JointType = EKinematicJointType::Revolute;

	/** Local axis to rotate/translate around */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint")
	TEnumAsByte<EAxis::Type> Axis = EAxis::X;

	/** Invert the direction of rotation/translation */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint")
	bool bInvertDirection = false;

	/** Calibration offset added to the target value before applying (degrees or cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint")
	float Offset = 0.0f;

	/** Minimum allowed value (degrees or cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint|Limits")
	float MinValue = -180.0f;

	/** Maximum allowed value (degrees or cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint|Limits")
	float MaxValue = 180.0f;

	/** Whether to enforce software limits */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint|Limits")
	bool bEnforceLimits = true;

	/** Current target value (degrees or cm). Set via API, applied each tick. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Joint|Runtime")
	float TargetValue = 0.0f;

	/** Currently applied value after interpolation (degrees or cm) */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Joint|Runtime")
	float CurrentValue = 0.0f;

	/** Get the effective joint name (JointName if set, otherwise BoneName) */
	FName GetEffectiveName() const { return JointName.IsNone() ? BoneName : JointName; }
};

/**
 * Component for kinematically posing UPoseableMeshComponents using joint angles/positions.
 *
 * Designed for UI visualization of robots — no physics simulation, just direct bone
 * manipulation. Supports both revolute (rotation) and prismatic (translation) joints,
 * making it suitable for robotic arms (Kinova Gen3), mobile bases (Mebot), and
 * other articulated mechanisms.
 *
 * Automatically discovers ALL UPoseableMeshComponents on the owning actor and
 * manages joints across all of them from a single flat Joints array. Each joint's
 * MeshComponentName field determines which mesh it targets (empty = first mesh).
 *
 * External systems (Blueprints, Python Remote Control) set joint targets, and this
 * component applies the corresponding bone transforms each tick.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsSkeletalPoseComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsSkeletalPoseComponent();

	// ── Joint Configuration ─────────────────────────────────────────

	/** Ordered array of joint configurations. Each maps a bone on a specific mesh to a controllable joint. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Skeletal Pose|Configuration")
	TArray<FKinematicJointConfig> Joints;

	// ── Interpolation ───────────────────────────────────────────────

	/** If true, joint values interpolate smoothly toward targets. If false, targets apply instantly. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Skeletal Pose|Interpolation")
	bool bEnableInterpolation = false;

	/**
	 * Maximum speed for interpolation (degrees/sec for revolute, cm/sec for prismatic).
	 * Only used when bEnableInterpolation is true. Set to 0 for instant snap.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Skeletal Pose|Interpolation",
		meta = (ClampMin = "0.0", EditCondition = "bEnableInterpolation"))
	float InterpolationSpeed = 180.0f;

	// ── Set Joint Targets ───────────────────────────────────────────

	/** Set a single joint target by index (degrees for revolute, cm for prismatic). */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Skeletal Pose")
	void SetJointTarget(int32 JointIndex, float Value);

	/** Set a single joint target by logical joint name. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Skeletal Pose")
	void SetJointTargetByName(FName JointName, float Value);

	/**
	 * Set all joint targets at once (array must match Joints array length).
	 *
	 * Uses TArray<double> instead of TArray<float> to work around a
	 * UE Remote Control API deserialization bug where TArray<float>
	 * values get doubled with a zero prepended.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Skeletal Pose")
	void SetAllJointTargets(const TArray<double>& Values);

	/** Set joint targets from a pose asset by index. Returns false if index is invalid. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Skeletal Pose")
	bool SetJointTargetsFromPoseIndex(URammsJointPoseAsset* PoseAsset, int32 PoseIndex = 0);

	/** Set joint targets from a pose asset by name. Returns false if name not found. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Skeletal Pose")
	bool SetJointTargetsFromPoseName(URammsJointPoseAsset* PoseAsset, const FString& PoseName);

	/** Immediately snap all joints to their targets (bypass interpolation). */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Skeletal Pose")
	void SnapToTargets();

	// ── Read Joint State ────────────────────────────────────────────

	/** Get the current applied value for a joint by index. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Skeletal Pose")
	float GetJointValue(int32 JointIndex) const;

	/** Get the current applied value for a joint by name. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Skeletal Pose")
	float GetJointValueByName(FName JointName) const;

	/** Get all current joint values. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Skeletal Pose")
	TArray<float> GetAllJointValues() const;

	/** Get the target value for a joint by index. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Skeletal Pose")
	float GetJointTarget(int32 JointIndex) const;

	/** Get all current joint targets. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Skeletal Pose")
	TArray<float> GetAllJointTargets() const;

	/** Get number of configured joints. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Skeletal Pose")
	int32 GetNumJoints() const { return Joints.Num(); }

	// ── Mesh Discovery ──────────────────────────────────────────────

	/** Get the names of all discovered UPoseableMeshComponents on this actor. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Skeletal Pose")
	TArray<FName> GetPoseableMeshNames() const;

	/** Get the number of discovered poseable meshes. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Skeletal Pose")
	int32 GetNumPoseableMeshes() const { return PoseableMeshes.Num(); }

	/**
	 * Re-discover all UPoseableMeshComponents on the owning actor.
	 * Call this if meshes are added/removed at runtime.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Skeletal Pose")
	void RefreshPoseableMeshes();

	// ── Joint Discovery ─────────────────────────────────────────────

	/**
	 * Auto-populate the Joints array from ALL discovered poseable meshes.
	 * Creates one revolute joint per non-root bone on each mesh, tagged with
	 * the mesh component name. Clears any existing configuration.
	 */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Ramms|Skeletal Pose")
	void AutoPopulateFromSkeleton();

	// ── Pose Asset ──────────────────────────────────────────────────

	/**
	 * Capture the current joint values into a pose asset with the given name.
	 * Creates a new pose entry (or updates existing one with same name).
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Skeletal Pose")
	void CaptureCurrentPose(URammsJointPoseAsset* PoseAsset, const FString& PoseName);

	// ── Debug ───────────────────────────────────────────────────────

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Skeletal Pose|Debug")
	bool bDebugLog = false;

protected:
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

private:
	/** All discovered poseable meshes, keyed by component name */
	UPROPERTY()
	TMap<FName, TObjectPtr<UPoseableMeshComponent>> PoseableMeshes;

	/** Name of the first mesh found (used as default when joint's MeshComponentName is empty) */
	FName DefaultMeshName;

	/** Joint name → index lookup (built on BeginPlay and when joints change) */
	TMap<FName, int32> JointNameMap;

	/** Discover and cache all UPoseableMeshComponents on the actor */
	void CacheAllPoseableMeshes();

	/** Rebuild the JointName→Index map */
	void RebuildNameMap();

	/** Resolve the UPoseableMeshComponent for a given joint */
	UPoseableMeshComponent* ResolveMeshForJoint(const FKinematicJointConfig& Joint) const;

	/** Apply a single joint's current value to its target mesh */
	void ApplyJointToBone(const FKinematicJointConfig& Joint) const;

	/** Clamp a value to a joint's limits if enforcement is enabled */
	static float ClampToLimits(const FKinematicJointConfig& Joint, float Value);

	/** Find a joint index by name. Returns INDEX_NONE if not found. */
	int32 FindJointIndex(FName JointName) const;
};
