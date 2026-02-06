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

	// Cached constraint reference
	FConstraintInstance* CachedConstraint;

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
		, CachedConstraint(nullptr)
	{}
};

USTRUCT(BlueprintType)
struct FLinearMotorConfig
{
	GENERATED_BODY()

	// Name of the constraint component to control
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	FName ConstraintName;

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
	{}
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class RAMMSCORE_API UMebotControllerComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	UMebotControllerComponent();

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

	// Find and cache skeletal mesh and constraints
	void FindConstraints();

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
