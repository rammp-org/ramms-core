// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "MebotControllerComponent.h"
#include "GripperControllerComponent.generated.h"

class USkeletalMeshComponent;

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

	// Set drive params for both finger motors. MaxForce 0 = unlimited; a finite value caps the
	// squeeze force so the grasp is compliant (won't ram through the object or break the joints).
	// Useful for live tuning of grip strength.
	UFUNCTION(BlueprintCallable, Category = "Gripper Controller")
	void SetFingerDriveParams(float Strength, float Damping, float MaxForce);

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

	// Stop driving a finger closed once it stalls against an object, instead of commanding it
	// all the way to ClosedAngle (which slowly drives the pads through the object). When a finger
	// can't keep up with the commanded angle, the command is held a small lead past the physical
	// angle so the drive keeps a bounded squeeze (pair with a finite Motor MaxForce).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Grasp")
	bool bStallAwareClosing = true;

	// How far (degrees) the commanded angle may lead the physical finger angle before it's
	// treated as stalled and clamped. Larger = firmer squeeze; smaller = gentler.
	// Only applied while closing — opening is always free so the gripper can release.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Grasp", meta = (ClampMin = "0.0"))
	float StallLeadDegrees = 4.0f;

	// Finger speed (deg/s) used when opening/releasing. Opening doesn't contact anything, so it
	// runs fast and at full drive force regardless of the (gentle) closing speed / force cap.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Grasp", meta = (ClampMin = "0.0"))
	float OpenSpeed = 90.0f;

	// Keep the two fingers synchronized while closing: neither finger's commanded angle may lead
	// the SLOWER finger's physical angle by more than FingerSyncLeadDegrees. This keeps contact
	// centered so the first pad to touch can't shove the object toward the other pad.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Grasp")
	bool bSyncFingersWhileClosing = true;

	// Max lead (degrees) one finger's command may run ahead of the slower finger while closing.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Grasp", meta = (ClampMin = "0.0"))
	float FingerSyncLeadDegrees = 3.0f;

	// Closing drive force cap used to hold the grasp. 0 = use each finger's own MaxForce. A single
	// explicit knob for grip firmness once the fingers stall on the object.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Grasp", meta = (ClampMin = "0.0"))
	float GripHoldForce = 0.0f;

	// Gentle final-approach speed (deg/s) used within FinalApproachBandDegrees of ClosedAngle so the
	// pads seat softly on the object instead of slamming. 0 = disabled (use full closing speed).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Grasp", meta = (ClampMin = "0.0"))
	float FinalApproachSpeed = 0.0f;

	// Angular band (degrees) before ClosedAngle within which FinalApproachSpeed applies.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Grasp", meta = (ClampMin = "0.0"))
	float FinalApproachBandDegrees = 10.0f;

	// Enable debug logging
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gripper Controller|Debug")
	bool bEnableDebugLog;

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

	// Find and cache skeletal mesh and constraints
	void FindConstraints();

	// Re-resolve the cached FConstraintInstance pointers from the mesh. They point into the
	// mesh's constraint array, which is freed and rebuilt whenever its physics state is
	// recreated (component re-registration from details-panel edits, LOD/mesh changes) —
	// a once-at-BeginPlay cache dangles after that and crashes on the next constraint call.
	void RefreshConstraintCache();

	// Get skeletal mesh from owner
	USkeletalMeshComponent* GetOwnerSkeletalMesh();

	// Update motor states
	void UpdateMotors(float DeltaTime);

	// Ramp one finger toward its target (stall-aware when closing) and push to the constraint
	void AdvanceMotor(FAngularMotorConfig& Motor, float DeltaTime, bool bApplySyncClamp = false, float SyncMaxCommand = 0.0f);

	// Apply motor settings to constraint. ForceLimitOverride < 0 uses Motor.MaxForce;
	// >= 0 overrides it (e.g. 0 = unlimited force when opening).
	void ApplyMotorSettings(FAngularMotorConfig& Motor, float ForceLimitOverride = -1.0f);

	// Update gripper state based on current angles
	void UpdateGripperState();

	// Fire appropriate events when state changes
	void HandleStateChange();
};
