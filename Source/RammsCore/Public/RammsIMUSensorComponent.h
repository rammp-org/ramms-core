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

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// ========== Configuration ==========

	/** Whether the accelerometer output includes gravitational acceleration */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Configuration")
	bool bIncludeGravity = true;

	/** Update rate in Hz (0 = every tick) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Configuration", meta = (ClampMin = "0.0"))
	float UpdateRateHz = 100.0f;

	/**
	 * Use physics velocity directly from the nearest primitive component when available.
	 * Avoids double-differentiating position (which amplifies physics jitter).
	 * Falls back to position differentiation if no physics primitive is found.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Configuration")
	bool bUsePhysicsVelocity = true;

	// ========== Filtering ==========

	/**
	 * Exponential moving average (EMA) smoothing factor for acceleration (0–1).
	 * 0 = no smoothing (raw output), 1 = maximum smoothing (output never changes).
	 * Models the bandwidth limit of real IMU accelerometers.
	 * Typical values: 0.5–0.9 for heavy smoothing, 0.0–0.3 for light smoothing.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Filtering", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float AccelSmoothingFactor = 0.0f;

	/**
	 * Exponential moving average (EMA) smoothing factor for gyroscope (0–1).
	 * Same semantics as AccelSmoothingFactor.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Filtering", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float GyroSmoothingFactor = 0.0f;

	/**
	 * Acceleration dead-band threshold (cm/s²). If the magnitude of the
	 * acceleration (excluding gravity) is below this value, it is zeroed out.
	 * Models the quiescent behavior of real accelerometers.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Filtering", meta = (ClampMin = "0.0"))
	float AccelDeadBand = 0.0f;

	/**
	 * Gyroscope dead-band threshold (degrees/s). If the magnitude of the
	 * angular velocity is below this value, it is zeroed out.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Filtering", meta = (ClampMin = "0.0"))
	float GyroDeadBand = 0.0f;

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

protected:
	virtual void BeginPlay() override;

private:
	// Internal state for velocity/acceleration computation
	FVector	   PreviousVelocity = FVector::ZeroVector;
	FTransform PreviousTransform = FTransform::Identity;
	FQuat	   PreviousRotation = FQuat::Identity;
	bool	   bPreviousStateValid = false;

	// EMA filter state (sensor-local frame, pre-noise)
	FVector SmoothedAccel = FVector::ZeroVector;
	FVector SmoothedGyro = FVector::ZeroVector;
	bool	bSmoothedStateValid = false;

	// Cached physics primitive (resolved in BeginPlay)
	UPROPERTY()
	TWeakObjectPtr<UPrimitiveComponent> CachedPhysicsPrimitive;

	// Update rate tracking
	float TimeSinceLastUpdate = 0.0f;
};
