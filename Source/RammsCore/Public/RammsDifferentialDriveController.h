// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsDifferentialDriveTypes.h"
#include "RammsDifferentialDriveController.generated.h"

class UPrimitiveComponent;

/**
 * Differential drive controller component for powered wheelchair simulation
 * Manages motor control, applies forces to wheels, tracks odometry
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsDifferentialDriveController : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsDifferentialDriveController();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// ========== Drive Configuration ==========

	/** Control mode selection */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Control")
	EDriveControlMode ControlMode = EDriveControlMode::TorqueControl;

	/** Name of the skeletal mesh component containing the wheel bones (leave empty to auto-find first skeletal mesh) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Wheels")
	FName SkeletalMeshComponentName = NAME_None;

	/** Left wheel bone name */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Wheels")
	FName LeftWheelBoneName = FName("left_motor");

	/** Right wheel bone name */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Wheels")
	FName RightWheelBoneName = FName("right_motor");

	/** Wheel radius in cm */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Wheels", meta = (ClampMin = "1.0"))
	float WheelRadius = 18.0f;

	/** Distance between left and right wheels in cm */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Wheels", meta = (ClampMin = "10.0"))
	float TrackWidth = 60.0f;

	// ========== Motor Configuration ==========

	/** Motor parameters for left wheel */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Motor")
	FMotorParameters LeftMotorParams;

	/** Motor parameters for right wheel */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Motor")
	FMotorParameters RightMotorParams;

	// ========== Torque Control Settings ==========

	/** Torque multiplier for torque control mode */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Torque Control", meta = (ClampMin = "0.0"))
	float TorqueMultiplier = 1.0f;

	/** Resistive torque coefficient - opposes wheel motion (simulates back-EMF and friction) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Torque Control", meta = (ClampMin = "0.0"))
	float ResistiveTorqueCoefficient = 0.5f;

	/** Maximum turning speed (angular velocity) in degrees per second */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Torque Control", meta = (ClampMin = "0.0"))
	float MaxTurningSpeed = 90.0f;

	// ========== Velocity Control Settings ==========

	/** Maximum target velocity in cm/s */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Velocity Control", meta = (ClampMin = "0.0"))
	float MaxVelocity = 200.0f;

	/** PID Proportional gain */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Velocity Control|PID", meta = (ClampMin = "0.0"))
	float PID_Kp = 10.0f;

	/** PID Integral gain */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Velocity Control|PID", meta = (ClampMin = "0.0"))
	float PID_Ki = 0.5f;

	/** PID Derivative gain */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Velocity Control|PID", meta = (ClampMin = "0.0"))
	float PID_Kd = 1.0f;

	/** Maximum integral windup value */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Velocity Control|PID", meta = (ClampMin = "0.0"))
	float PID_IntegralMax = 100.0f;

	// ========== Braking ==========

	/** Enable automatic braking when input is at rest */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Braking")
	bool bEnableAutoBraking = false;

	/** Input magnitude threshold below which brakes engage */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Braking", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BrakingThreshold = 0.01f;

	/** Brake torque to apply when braking (N*m) - acts as damping coefficient */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Braking", meta = (ClampMin = "0.0"))
	float BrakeTorque = 2.0f;

	/** Velocity threshold below which brakes engage (cm/s) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Braking", meta = (ClampMin = "0.0"))
	float BrakingVelocityThreshold = 5.0f;

	// ========== Slip/Traction ==========

	/** Enable slip/traction modeling */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction")
	bool bEnableSlipModeling = false;

	/** Base traction coefficient (multiplied by surface friction) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction", meta = (ClampMin = "0.1", ClampMax = "2.0"))
	float TractionCoefficient = 1.0f;

	/** Use physical material friction from ground surface */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction")
	bool bUsePhysicalMaterialFriction = true;

	/** Enable load-dependent traction (wheels with more weight get more grip) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction")
	bool bEnableLoadDependentTraction = true;

	/** Mass of the vehicle (kg) - used for load calculations */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction", meta = (ClampMin = "1.0"))
	float VehicleMass = 100.0f;

	/** Slip ratio where peak traction occurs (typically 0.1-0.15) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction", meta = (ClampMin = "0.01", ClampMax = "0.3"))
	float PeakSlipRatio = 0.15f;

	/** Enable lateral slip resistance (reduces torque when sliding sideways) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction")
	bool bEnableLateralSlipResistance = true;

	/** Lateral velocity threshold before slip resistance kicks in (cm/s) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction", meta = (ClampMin = "0.0"))
	float LateralSlipThreshold = 10.0f;

	// ========== Input ==========

	/** Current drive input (2D joystick style) */
	UPROPERTY(BlueprintReadOnly, Category = "Drive|Input")
	FVector2D DriveInput = FVector2D::ZeroVector;

	/** Input dead zone */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Input", meta = (ClampMin = "0.0", ClampMax = "0.5"))
	float InputDeadZone = 0.05f;

	// ========== State ==========

	/** Left wheel state */
	UPROPERTY(BlueprintReadOnly, Category = "Drive|State")
	FWheelState LeftWheelState;

	/** Right wheel state */
	UPROPERTY(BlueprintReadOnly, Category = "Drive|State")
	FWheelState RightWheelState;

	/** Current odometry data */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drive|State")
	FOdometryData Odometry;

	/** Is braking currently engaged */
	UPROPERTY(BlueprintReadOnly, Category = "Drive|State")
	bool bIsBraking = false;

	/** Velocity threshold for odometry updates (cm/s) - movements below this are ignored to prevent drift */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Odometry", meta = (ClampMin = "0.0"))
	float OdometryVelocityThreshold = 0.5f;

	// ========== Blueprint API ==========

	/**
	 * Set drive input from 2D joystick
	 * @param Input - Joystick input (X = turn, Y = forward/back), -1 to 1
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	void SetDriveInput(FVector2D Input);

	/**
	 * Get current odometry data
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Differential Drive")
	FOdometryData GetOdometry() const { return Odometry; }

	/**
	 * Reset odometry to given position and orientation
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	void ResetOdometry(FVector Position = FVector::ZeroVector, FRotator Orientation = FRotator::ZeroRotator);

	/**
	 * Get left wheel state
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Differential Drive")
	FWheelState GetLeftWheelState() const { return LeftWheelState; }

	/**
	 * Get right wheel state
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Differential Drive")
	FWheelState GetRightWheelState() const { return RightWheelState; }

	/**
	 * Set control mode
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	void SetControlMode(EDriveControlMode Mode) { ControlMode = Mode; }

	/**
	 * Enable or disable slip modeling
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	void SetSlipModelingEnabled(bool bEnabled) { bEnableSlipModeling = bEnabled; }

	// ========== Debug ==========

	/** Enable debug logging */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Debug")
	bool bEnableDebugLogging = false;

	/** Enable on-screen debug display */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Debug")
	bool bEnableDebugDisplay = false;

private:
	// Cached skeletal mesh component reference
	UPROPERTY(Transient)
	USkeletalMeshComponent* SkeletalMeshComponent = nullptr;

	// PID state for velocity control
	float LeftIntegralError = 0.0f;
	float LeftPreviousError = 0.0f;
	float RightIntegralError = 0.0f;
	float RightPreviousError = 0.0f;

	// Previous wheel rotations for odometry
	float PreviousLeftRotation = 0.0f;
	float PreviousRightRotation = 0.0f;

	/** Get body instance for a specific bone */
	FBodyInstance* GetBoneBodyInstance(FName BoneName);

	/** Update wheel state from physics */
	void UpdateWheelState(FName BoneName, FWheelState& OutState);

	/** Update controller in torque control mode */
	void UpdateTorqueControl(float DeltaTime);

	/** Update controller in velocity control mode */
	void UpdateVelocityControl(float DeltaTime);

	/** Calculate PID control output */
	float CalculatePID(float Error, float& IntegralError, float& PreviousError, float DeltaTime);

	/** Apply torque to wheel with motor and slip modeling */
	void ApplyWheelTorque(FName BoneName, float RequestedTorque, const FMotorParameters& MotorParams, FWheelState& WheelState);

	/** Update odometry from wheel movements */
	void UpdateOdometry(float DeltaTime);

	/** Check if braking should be applied */
	bool ShouldApplyBrakes() const;

	/** Apply brake torque to both wheels */
	void ApplyBrakes();

	/** Debug log current state */
	void DebugLogState();
};
