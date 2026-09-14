// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "MebotControllerComponent.generated.h"

class USkeletalMeshComponent;
struct FConstraintInstance;

UENUM(BlueprintType)
enum class EMotorAxis : uint8
{
	X UMETA(DisplayName = "X Axis"),
	Y UMETA(DisplayName = "Y Axis"),
	Z UMETA(DisplayName = "Z Axis")
};

USTRUCT(BlueprintType)
struct FAngularMotorConfig
{
	GENERATED_BODY()

	// Name of the constraint component to control
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	FName ConstraintName;

	// Registry Id of this motor on the owner's RammsRobotBaseComponent. Empty =
	// the registry motor whose ChaosName is ConstraintName (resolved at play).
	// An explicit Id that is not in the registry drives the constraint directly
	// (with a warning) rather than falling back to the constraint-name lookup.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	FName MotorId;

	// Which angular axis this motor controls
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	EMotorAxis ControlAxis;

	// Whether this motor is enabled
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	bool bEnabled;

	// Invert the rotation direction (multiply angle by -1)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	bool bInvertDirection;

	// Target angle in degrees
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	float TargetAngle;

	// Current angle (for smooth interpolation)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Motor")
	float CurrentAngle;

	// Maximum angular speed in degrees per second
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor|Speed", meta = (ClampMin = "0.0"))
	float MaxSpeed;

	// Current movement speed (for dynamic control)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor|Speed", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float SpeedMultiplier;

	// Motor strength (spring constant)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor", meta = (ClampMin = "0.0"))
	float MotorStrength;

	// Motor damping
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor", meta = (ClampMin = "0.0"))
	float MotorDamping;

	// Max drive force/torque the motor can apply (UE drive units). 0 = unlimited.
	// For grasping, a finite cap makes the squeeze compliant: the finger pushes toward its
	// target with at most this force instead of integrating unbounded torque against a blocked
	// object (which rams the fingers through the object and violates the finger joint constraints).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor", meta = (ClampMin = "0.0"))
	float MaxForce;

	// Cached constraint reference
	FConstraintInstance* CachedConstraint;

	// The registry Id this motor is routed through at runtime (resolved from
	// MotorId / ConstraintName by ResolveRobotBase; NAME_None = drive the
	// constraint directly). The authored MotorId is never overwritten.
	FName ResolvedMotorId;

	// A settings/target change not yet pushed to the drive. The drive holds its
	// last target, so it is only re-commanded while moving or after a change —
	// never every tick, which would override anyone else commanding the same
	// motor through the robot base.
	bool bPendingApply = true;

	FAngularMotorConfig()
		: ConstraintName(NAME_None)
		, ControlAxis(EMotorAxis::Z)
		, bEnabled(true)
		, bInvertDirection(false)
		, TargetAngle(0.0f)
		, CurrentAngle(0.0f)
		, MaxSpeed(45.0f)
		, SpeedMultiplier(1.0f)
		, MotorStrength(100000.0f)
		, MotorDamping(10000.0f)
		, MaxForce(0.0f)
		, CachedConstraint(nullptr)
	{
	}
};

USTRUCT(BlueprintType)
struct FLinearMotorConfig
{
	GENERATED_BODY()

	// Name of the constraint component to control
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	FName ConstraintName;

	// Registry Id of this motor on the owner's RammsRobotBaseComponent. Empty =
	// the registry motor whose ChaosName is ConstraintName (resolved at play).
	// An explicit Id that is not in the registry drives the constraint directly
	// (with a warning) rather than falling back to the constraint-name lookup.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	FName MotorId;

	// Which linear axis this motor controls
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	EMotorAxis ControlAxis;

	// Whether this motor is enabled
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	bool bEnabled;

	// Target position in cm
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	float TargetPosition;

	// Current position (for smooth interpolation)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Motor")
	float CurrentPosition;

	// Maximum linear speed in cm per second
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor|Speed", meta = (ClampMin = "0.0"))
	float MaxSpeed;

	// Current movement speed (for dynamic control)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor|Speed", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float SpeedMultiplier;

	// Motor strength (spring constant)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor", meta = (ClampMin = "0.0"))
	float MotorStrength;

	// Motor damping
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor", meta = (ClampMin = "0.0"))
	float MotorDamping;

	// Cached constraint reference
	FConstraintInstance* CachedConstraint;

	// See FAngularMotorConfig::ResolvedMotorId.
	FName ResolvedMotorId;

	// See FAngularMotorConfig::bPendingApply.
	bool bPendingApply = true;

	FLinearMotorConfig()
		: ConstraintName(NAME_None)
		, ControlAxis(EMotorAxis::Z)
		, bEnabled(true)
		, TargetPosition(0.0f)
		, CurrentPosition(0.0f)
		, MaxSpeed(50.0f)
		, SpeedMultiplier(1.0f)
		, MotorStrength(100000.0f)
		, MotorDamping(10000.0f)
		, CachedConstraint(nullptr)
	{
	}
};

class URammsRobotBaseComponent;

/**
 * Rate-limited position targets for the chair's lift / linkage actuators
 * (drive-motor elevators, drive-plate translators, caster arms), addressed
 * by their physics-asset constraint name (angles in degrees, travel in cm).
 *
 * When the owner carries a RammsRobotBaseComponent with a backend, each
 * motor is routed through it by registry Id (bUseRobotBase): the base's
 * backend drives the constraint on Chaos, or the matching position actuator
 * on MuJoCo, and reads come back the same way. Without a base the component
 * drives the constraints directly, as it always has.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API UMebotControllerComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UMebotControllerComponent();

	// Route motors through the owner's RammsRobotBaseComponent (by MotorId, or
	// the registry motor whose ChaosName is the constraint) when it has a
	// backend; otherwise drive the constraints directly.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mebot Controller")
	bool bUseRobotBase = true;

	// True while motors are being routed through the robot base component.
	UFUNCTION(BlueprintPure, Category = "Mebot Controller")
	bool IsUsingRobotBase() const;

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// Set target angle for a specific angular motor by name
	UFUNCTION(BlueprintCallable, Category = "Mebot Controller")
	void SetAngularMotorTarget(FName MotorName, float TargetAngle);

	// Set target position for a specific linear motor by name
	UFUNCTION(BlueprintCallable, Category = "Mebot Controller")
	void SetLinearMotorTarget(FName MotorName, float TargetPosition);

	// Set max speed for a specific angular motor (degrees per second)
	UFUNCTION(BlueprintCallable, Category = "Mebot Controller")
	void SetAngularMotorMaxSpeed(FName MotorName, float MaxSpeed);

	// Set max speed for a specific linear motor (cm per second)
	UFUNCTION(BlueprintCallable, Category = "Mebot Controller")
	void SetLinearMotorMaxSpeed(FName MotorName, float MaxSpeed);

	// Set speed multiplier for a specific angular motor (0.0 to 1.0)
	UFUNCTION(BlueprintCallable, Category = "Mebot Controller")
	void SetAngularMotorSpeedMultiplier(FName MotorName, float SpeedMultiplier);

	// Set speed multiplier for a specific linear motor (0.0 to 1.0)
	UFUNCTION(BlueprintCallable, Category = "Mebot Controller")
	void SetLinearMotorSpeedMultiplier(FName MotorName, float SpeedMultiplier);

	// Enable or disable a specific angular motor by name
	UFUNCTION(BlueprintCallable, Category = "Mebot Controller")
	void SetAngularMotorEnabled(FName MotorName, bool bEnabled);

	// Enable or disable a specific linear motor by name
	UFUNCTION(BlueprintCallable, Category = "Mebot Controller")
	void SetLinearMotorEnabled(FName MotorName, bool bEnabled);

	// Get current angle for a specific angular motor
	UFUNCTION(BlueprintPure, Category = "Mebot Controller")
	float GetAngularMotorCurrentAngle(FName MotorName) const;

	// Get current position for a specific linear motor
	UFUNCTION(BlueprintPure, Category = "Mebot Controller")
	float GetLinearMotorCurrentPosition(FName MotorName) const;

	// Reinitialize all motor constraints (useful if constraints are added at runtime)
	UFUNCTION(BlueprintCallable, Category = "Mebot Controller")
	void ReinitializeMotors();

	// Get debug info string for a motor
	UFUNCTION(BlueprintPure, Category = "Mebot Controller|Debug")
	FString GetMotorDebugInfo() const;

	// Get all angular motor configurations
	UFUNCTION(BlueprintPure, Category = "Mebot Controller")
	TArray<FAngularMotorConfig> GetAngularMotors() const { return AngularMotors; }

	// Get all linear motor configurations
	UFUNCTION(BlueprintPure, Category = "Mebot Controller")
	TArray<FLinearMotorConfig> GetLinearMotors() const { return LinearMotors; }

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// Angular motor configurations
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mebot Controller|Motors")
	TArray<FAngularMotorConfig> AngularMotors;

	// Linear motor configurations
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mebot Controller|Motors")
	TArray<FLinearMotorConfig> LinearMotors;

	// If true, automatically find and cache constraint components at BeginPlay
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mebot Controller")
	bool bAutoFindConstraints;

	// If true, automatically find the skeletal mesh component on the owner
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mebot Controller")
	bool bAutoFindSkeletalMesh;

	// Enable debug logging
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mebot Controller|Debug")
	bool bEnableDebugLog;

	// Show debug visualization of motor positions
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mebot Controller|Debug")
	bool bShowMotorGizmos;

	// Size of the motor gizmos
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mebot Controller|Debug", meta = (ClampMin = "1.0", ClampMax = "200.0"))
	float GizmoSize;

private:
	// Cached skeletal mesh component
	UPROPERTY()
	USkeletalMeshComponent* CachedSkeletalMesh;

	// The owner's robot base, when routing through it
	UPROPERTY(Transient)
	TObjectPtr<URammsRobotBaseComponent> RobotBase;

	// Resolve the base (once) and each motor's registry Id
	void ResolveRobotBase();

	// Registry Id for a motor entry (MotorId, else the motor whose ChaosName is
	// the constraint); NAME_None when the base doesn't know it
	FName BaseMotorId(FName ConstraintName, FName ExplicitMotorId) const;

	// Find and cache skeletal mesh and constraints
	void FindConstraints();

	// Re-resolve every motor's CachedConstraint from the mesh. The pointers reference the
	// mesh's constraint array, which is freed and rebuilt whenever its physics state is
	// recreated (component re-registration from details-panel edits, LOD/mesh changes) —
	// a once-at-BeginPlay cache dangles after that and crashes on the next constraint call.
	// Quiet counterpart of FindConstraints (no per-frame warning spam).
	void RefreshConstraintCache();

	// Get skeletal mesh from owner
	USkeletalMeshComponent* GetOwnerSkeletalMesh();

	// Update all angular motors
	void UpdateAngularMotors(float DeltaTime);

	// Update all linear motors
	void UpdateLinearMotors(float DeltaTime);

	// Apply angular motor settings to constraint
	void ApplyAngularMotorSettings(FAngularMotorConfig& Motor);

	// Apply linear motor settings to constraint
	void ApplyLinearMotorSettings(FLinearMotorConfig& Motor);

	// Draw debug visualization
	void DrawDebugVisualization();
};
