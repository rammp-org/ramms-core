// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "RammsIMUSensorComponent.generated.h"

/**
 * Output data from the IMU sensor, produced each measurement cycle.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FIMUSensorData
{
	GENERATED_BODY()

	/** Linear acceleration in sensor-local frame (cm/s²). Includes gravity if bIncludeGravity is true. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU")
	FVector LinearAcceleration = FVector::ZeroVector;

	/** Angular velocity in sensor-local frame (degrees/s) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU")
	FVector AngularVelocity = FVector::ZeroVector;

	/** Orientation as quaternion (world-space) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU")
	FQuat Orientation = FQuat::Identity;

	/** Timestamp of the measurement (seconds since game start) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU")
	float Timestamp = 0.0f;
};

/** Fired each time a new IMU measurement is produced */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnIMUDataUpdated, const FIMUSensorData&, Data);

/**
 * Simulated Inertial Measurement Unit (IMU) sensor.
 * Measures linear acceleration (accelerometer), angular velocity (gyroscope),
 * and orientation. Supports configurable noise, bias, and update rate.
 * Inherits from USceneComponent so it can be attached to any bone, socket,
 * or other component. Uses the component's own world transform as the sensor frame.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsIMUSensorComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	URammsIMUSensorComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// ========== Configuration ==========

	/** Whether the accelerometer output includes gravitational acceleration */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Configuration")
	bool bIncludeGravity = true;

	/** Update rate in Hz (0 = every tick) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Configuration", meta = (ClampMin = "0.0"))
	float UpdateRateHz = 100.0f;

	// ========== Noise ==========

	/** Accelerometer Gaussian noise standard deviation (cm/s² per axis) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Noise", meta = (ClampMin = "0.0"))
	float AccelNoiseStdDev = 0.0f;

	/** Gyroscope Gaussian noise standard deviation (degrees/s per axis) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Noise", meta = (ClampMin = "0.0"))
	float GyroNoiseStdDev = 0.0f;

	/** Orientation Gaussian noise standard deviation (degrees per axis) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Noise", meta = (ClampMin = "0.0"))
	float OrientationNoiseStdDev = 0.0f;

	// ========== Bias ==========

	/** Constant accelerometer bias in sensor-local frame (cm/s²) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Bias")
	FVector AccelBias = FVector::ZeroVector;

	/** Constant gyroscope bias in sensor-local frame (degrees/s) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Bias")
	FVector GyroBias = FVector::ZeroVector;

	// ========== Debug ==========

	/** Draw debug axes at sensor location */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Debug")
	bool bEnableDebugDisplay = false;

	/** Log sensor output to console */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Debug")
	bool bEnableDebugLogging = false;

	// ========== State ==========

	/** Most recent sensor reading */
	UPROPERTY(BlueprintReadOnly, Category = "IMU|State")
	FIMUSensorData CurrentData;

	// ========== Events ==========

	/** Fires each measurement cycle with the latest data */
	UPROPERTY(BlueprintAssignable, Category = "IMU|Events")
	FOnIMUDataUpdated OnIMUDataUpdated;

	// ========== Blueprint API ==========

	/** Get the latest IMU reading */
	UFUNCTION(BlueprintPure, Category = "Ramms|Sensors|IMU")
	FIMUSensorData GetIMUData() const { return CurrentData; }

private:
	/** Generate a Gaussian random value with given standard deviation */
	float GaussianNoise(float StdDev) const;

	/** Add Gaussian noise to a vector */
	FVector AddVectorNoise(const FVector& Value, float StdDev) const;

	// Internal state for velocity/acceleration computation
	FVector	   PreviousVelocity = FVector::ZeroVector;
	FVector	   PreviousAngularVelocity = FVector::ZeroVector;
	FTransform PreviousTransform = FTransform::Identity;
	FQuat	   PreviousRotation = FQuat::Identity;
	bool	   bPreviousStateValid = false;

	// Update rate tracking
	float TimeSinceLastUpdate = 0.0f;
};
