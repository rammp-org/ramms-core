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

	UFUNCTION(BlueprintPure, Category = "Gripper Controller")
	FAngularMotorConfig GetFinger1MotorConfig() const { return Finger1Motor; }

	UFUNCTION(BlueprintPure, Category = "Gripper Controller")
	FAngularMotorConfig GetFinger2MotorConfig() const { return Finger2Motor; }

	UFUNCTION(BlueprintPure, Category = "Gripper Controller")
	FName GetGripperSkeletalMeshComponentName() const { return GripperMeshName; }

	UFUNCTION(BlueprintPure, Category = "Gripper Controller")
	float GetOpenAngle() const { return OpenAngle; }

	UFUNCTION(BlueprintPure, Category = "Gripper Controller")
	float GetClosedAngle() const { return ClosedAngle; }

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

	FConstraintInstance* ResolveConstraint(FAngularMotorConfig& Motor);

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
