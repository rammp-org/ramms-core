// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RammsDifferentialDriveTypes.h"
#include "RammsDifferentialDriveLibrary.generated.h"

/**
 * Blueprint function library for differential drive calculations
 * Provides static utility functions for wheelchair drive kinematics
 */
UCLASS()
class RAMMSCORE_API URammsDifferentialDriveLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Convert 2D joystick input to differential drive commands
	 * @param JoystickInput - 2D input vector (X = turn, Y = forward/back), normalized -1 to 1
	 * @param MaxValue - Maximum output value (torque or velocity depending on control mode)
	 * @param DeadZone - Input values below this threshold are treated as zero
	 * @return Left and right wheel commands
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	static FDifferentialDriveCommand JoystickToDifferentialDrive(
		FVector2D JoystickInput,
		float MaxValue = 1.0f,
		float DeadZone = 0.05f);

	/**
	 * Calculate wheel velocities from linear and angular velocities
	 * @param LinearVelocity - Forward velocity in cm/s
	 * @param AngularVelocity - Turning rate in degrees/s
	 * @param TrackWidth - Distance between left and right wheels in cm
	 * @param WheelRadius - Radius of wheels in cm
	 * @return Left and right wheel angular velocities in rad/s
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	static FDifferentialDriveCommand CalculateWheelVelocities(
		float LinearVelocity,
		float AngularVelocity,
		float TrackWidth,
		float WheelRadius);

	/**
	 * Calculate wheelchair velocity from wheel velocities (inverse kinematics)
	 * @param LeftWheelVelocity - Left wheel angular velocity in rad/s
	 * @param RightWheelVelocity - Right wheel angular velocity in rad/s
	 * @param TrackWidth - Distance between left and right wheels in cm
	 * @param WheelRadius - Radius of wheels in cm
	 * @param OutLinearVelocity - Calculated linear velocity in cm/s
	 * @param OutAngularVelocity - Calculated angular velocity in degrees/s
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	static void CalculateChassisVelocity(
		float LeftWheelVelocity,
		float RightWheelVelocity,
		float TrackWidth,
		float WheelRadius,
		float& OutLinearVelocity,
		float& OutAngularVelocity);

	/**
	 * Evaluate motor torque based on current RPM and motor parameters
	 * @param CurrentRPM - Current motor speed in RPM
	 * @param MotorParams - Motor characteristic parameters
	 * @param RequestedTorque - Desired torque (before motor limits)
	 * @return Available torque considering motor speed and characteristics
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	static float EvaluateMotorTorque(
		float CurrentRPM,
		const FMotorParameters& MotorParams,
		float RequestedTorque);

	/**
	 * Convert angular velocity (rad/s) to RPM
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Differential Drive")
	static float RadPerSecToRPM(float RadPerSec);

	/**
	 * Convert RPM to angular velocity (rad/s)
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Differential Drive")
	static float RPMToRadPerSec(float RPM);

	/**
	 * Update odometry based on wheel movements
	 * @param CurrentOdometry - Current odometry state
	 * @param LeftWheelDelta - Left wheel rotation change in radians
	 * @param RightWheelDelta - Right wheel rotation change in radians
	 * @param TrackWidth - Distance between wheels in cm
	 * @param WheelRadius - Wheel radius in cm
	 * @param DeltaTime - Time elapsed in seconds
	 * @return Updated odometry data
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	static FOdometryData UpdateOdometry(
		const FOdometryData& CurrentOdometry,
		float LeftWheelDelta,
		float RightWheelDelta,
		float TrackWidth,
		float WheelRadius,
		float DeltaTime);

	/**
	 * Reset odometry to given position and orientation
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	static FOdometryData ResetOdometry(
		FVector Position = FVector::ZeroVector,
		FRotator Orientation = FRotator::ZeroRotator);

	/**
	 * Calculate slip ratio between desired and actual wheel velocity
	 * @param DesiredVelocity - Target wheel velocity
	 * @param ActualVelocity - Measured wheel velocity
	 * @return Slip ratio (0 = no slip, 1 = full slip)
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Differential Drive")
	static float CalculateSlipRatio(float DesiredVelocity, float ActualVelocity);

	/**
	 * Apply dead zone to input value
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Differential Drive")
	static float ApplyDeadZone(float Value, float DeadZone);

	/**
	 * Apply dead zone to 2D input vector
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Differential Drive")
	static FVector2D ApplyDeadZone2D(FVector2D Value, float DeadZone);

	/**
	 * Calculate traction multiplier from slip ratio using realistic tire curve
	 * @param SlipRatio - Current slip ratio (0-1)
	 * @param PeakSlipRatio - Slip ratio where peak grip occurs (typically 0.1-0.15)
	 * @return Traction multiplier (0-1, peaks at PeakSlipRatio)
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Differential Drive")
	static float GetTractionMultiplierFromSlip(float SlipRatio, float PeakSlipRatio = 0.15f);

	/**
	 * Calculate available grip force based on wheel load and surface friction
	 * @param WheelLoad - Normal force on wheel (Newtons)
	 * @param SurfaceFriction - Friction coefficient from physical material
	 * @param TractionCoefficient - Base traction multiplier
	 * @return Maximum grip force (Newtons)
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Differential Drive")
	static float CalculateAvailableGrip(float WheelLoad, float SurfaceFriction, float TractionCoefficient);
};
