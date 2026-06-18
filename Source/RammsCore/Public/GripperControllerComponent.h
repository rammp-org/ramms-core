// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "MebotControllerComponent.h"
#include "GripperControllerComponent.generated.h"

class USkeletalMeshComponent;
struct FConstraintInstance;

UENUM(BlueprintType)
enum class EGripperState : uint8
{
	Open	UMETA(DisplayName = "Open"),
	Closed	UMETA(DisplayName = "Closed"),
	Opening UMETA(DisplayName = "Opening"),
	Closing UMETA(DisplayName = "Closing")
};

// Event delegates
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnGripperOpened);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnGripperClosed);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnGripperStateChanged, EGripperState, NewState);

/**
 * Controller component for a two-finger gripper with physics constraints
 * Each finger has an angular motor that can be controlled to open/close
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API UGripperControllerComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UGripperControllerComponent();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// Open the gripper
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Gripper Controller")
	void Open();

	// Close the gripper
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Gripper Controller")
	void Close();

	// Toggle between open and closed
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Gripper Controller")
	void Toggle();

	// Set the gripper to a specific state
	UFUNCTION(BlueprintCallable, Category = "Gripper Controller")
	void SetGripperState(EGripperState NewState);

	// Get the current state of the gripper
	UFUNCTION(BlueprintPure, Category = "Gripper Controller")
	EGripperState GetGripperState() const { return CurrentState; }

	// Check if gripper is fully open
	UFUNCTION(BlueprintPure, Category = "Gripper Controller")
	bool IsOpen() const { return CurrentState == EGripperState::Open; }

	// Check if gripper is fully closed
	UFUNCTION(BlueprintPure, Category = "Gripper Controller")
	bool IsClosed() const { return CurrentState == EGripperState::Closed; }

	// Set target angles directly for both fingers
	UFUNCTION(BlueprintCallable, Category = "Gripper Controller")
	void SetFingerAngles(float Finger1Angle, float Finger2Angle);

	// Get current finger angles
	UFUNCTION(BlueprintPure, Category = "Gripper Controller")
	void GetFingerAngles(float& OutFinger1Angle, float& OutFinger2Angle) const;

	// Set max speed for both finger motors
	UFUNCTION(BlueprintCallable, Category = "Gripper Controller")
	void SetMotorMaxSpeed(float MaxSpeed);

	// Set speed multiplier for both finger motors (0.0 to 1.0)
	UFUNCTION(BlueprintCallable, Category = "Gripper Controller")
	void SetMotorSpeedMultiplier(float SpeedMultiplier);

	// Grip-force cap (Chaos angular-drive force limit). Safe to call at runtime.
	UFUNCTION(BlueprintCallable, Category = "Gripper Controller")
	void SetMotorForceLimit(float NewLimit) { MotorForceLimit = FMath::Max(0.0f, NewLimit); }

	UFUNCTION(BlueprintPure, Category = "Gripper Controller")
	float GetMotorForceLimit() const { return MotorForceLimit; }

	// Events
	UPROPERTY(BlueprintAssignable, Category = "Gripper Controller|Events")
	FOnGripperOpened OnGripperOpened;

	UPROPERTY(BlueprintAssignable, Category = "Gripper Controller|Events")
	FOnGripperClosed OnGripperClosed;

	UPROPERTY(BlueprintAssignable, Category = "Gripper Controller|Events")
	FOnGripperStateChanged OnGripperStateChanged;

protected:
	virtual void BeginPlay() override;

	// Motor configuration for finger 1
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Motors")
	FAngularMotorConfig Finger1Motor;

	// Motor configuration for finger 2
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Motors")
	FAngularMotorConfig Finger2Motor;

	// Force cap on the KNUCKLE (close) drives only — this is the grip/squeeze force.
	// 0 = unlimited (the original behavior: unbounded force that crushed/ejected items).
	// Default is high (≈ unlimited = firm, fully controlled fingers); TUNE THIS DOWN to make
	// the grip gentler. Plain component float -> safe to set at runtime (the per-finger
	// struct must NOT be round-tripped through set_editor_property; that clobbers its raw
	// CachedConstraint pointer and crashes).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Motors", meta = (ClampMin = "0.0"))
	float MotorForceLimit = 500000.0f;

	// Force cap on the fingertip MIMIC drive (keeps the pad parallel). This is pose-keeping,
	// NOT grip force, so it must stay strong — 0 = unlimited. Capping it low makes the
	// fingertip go limp ("seaweed"); leave at 0 unless you have a specific reason.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Motors", meta = (ClampMin = "0.0"))
	float MimicForceLimit = 0.0f;

	// Target angle when gripper is fully open (degrees)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Configuration")
	float OpenAngle;

	// Target angle when gripper is fully closed (degrees)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Configuration")
	float ClosedAngle;

	// Tolerance for considering gripper fully open/closed (degrees)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Configuration")
	float AngleTolerance;

	// Skeletal mesh component containing the gripper (optional, auto-found if not set)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Configuration")
	FName GripperMeshName;

	// If true, automatically find the skeletal mesh component on the owner
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Configuration")
	bool bAutoFindSkeletalMesh;

	// Enable debug logging
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Debug")
	bool bEnableDebugLog;

	// --- Parallel-jaw pad mimic: drive a distal finger joint so the pad stays parallel
	//     as the knuckle closes (replicates the real parallelogram linkage) ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Pad Mimic")
	bool bEnablePadMimic = true;

	// distal joint (child bone name) on each finger that orients the pad
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Pad Mimic")
	FName MimicConstraint1 = FName("end_r");
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Pad Mimic")
	FName MimicConstraint2 = FName("end_l");

	// hinge axis of the mimic joint
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Pad Mimic")
	EMotorAxis MimicAxis = EMotorAxis::Z;

	// pad joint angle = Multiplier * knuckle angle + Offset (deg). -1 keeps the pad parallel.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Pad Mimic")
	float MimicMultiplier = -1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Pad Mimic")
	float MimicOffset = 0.0f;

	// per-finger direction flip (mirror the left finger if needed)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Pad Mimic")
	bool bMimic1Invert = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Pad Mimic")
	bool bMimic2Invert = false;

private:
	// Current state of the gripper
	UPROPERTY()
	EGripperState CurrentState;

	// Previous state (for detecting transitions)
	UPROPERTY()
	EGripperState PreviousState;

	// Cached skeletal mesh component
	UPROPERTY()
	USkeletalMeshComponent* CachedGripperMesh;

	// cached mimic joints + driver
	FConstraintInstance* MimicConstraint1Cached = nullptr;
	FConstraintInstance* MimicConstraint2Cached = nullptr;
	void DriveMimicJoint(FConstraintInstance* Constraint, float AngleDeg, bool bInvert);

	// Find and cache skeletal mesh and constraints
	void FindConstraints();

	// Get skeletal mesh from owner
	USkeletalMeshComponent* GetOwnerSkeletalMesh();

	// Update motor states
	void UpdateMotors(float DeltaTime);

	// Apply motor settings to constraint
	void ApplyMotorSettings(FAngularMotorConfig& Motor);

	// Update gripper state based on current angles
	void UpdateGripperState();

	// Fire appropriate events when state changes
	void HandleStateChange();
};
