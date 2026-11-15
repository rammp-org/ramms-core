// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "RammsImuTypes.h"
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

	/** Where the measurement came from (never Auto): Chaos or MuJoCo, sampled at the physics rate, or Kinematic,
	 *  differenced from the sensor's transform at the game rate. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU")
	ERammsImuSource Source = ERammsImuSource::Kinematic;

	/** Chaos / MuJoCo: the engine's time (s, its own clock) of the state the measurement ends at; it stops
	 *  advancing while that engine is paused. 0 for Kinematic. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU")
	double SimTimeSeconds = 0.0;
};

/** Fired each time a new IMU measurement is produced */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnIMUDataUpdated, const FIMUSensorData&, Data);

/**
 * Simulated Inertial Measurement Unit (IMU) sensor.
 * Measures linear acceleration (accelerometer), angular velocity (gyroscope),
 * and orientation. Supports configurable noise, bias, and update rate.
 * Inherits from USceneComponent so it can be attached to any bone, socket,
 * or other component. Uses the component's own world transform as the sensor frame.
 *
 * Source picks the data: by default (Auto) the physics engine that simulates
 * the body the sensor rides on, sampled every physics step at the sensor's
 * point (URammsPhysicsImuSubsystem): MuJoCo for a URLab body, Chaos for a
 * simulating primitive or skeletal bone. Elsewhere, and until that engine
 * has a sample, the transform is differenced at the game rate (Kinematic).
 *
 * With an engine's samples a measurement covers the whole interval since the
 * last one, as a real IMU's filtered output does: the accelerometer reads the
 * point's velocity change over the interval divided by the engine time it
 * spans (minus gravity), the gyroscope the mean angular velocity (the turn
 * over the interval divided by its time), both in the sensor frame halfway
 * through the turn. Not one physics step's sample. No new engine step since
 * the last measurement repeats it, as does a moment (up to a second) in which
 * the engine has no sample (its body handed a new physics proxy, its scene
 * recompiled). The filters, noise, bias and gravity options then apply as
 * before.
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

	/**
	 * Where the readings come from. Auto: the engine that simulates the body this sensor is attached to (MuJoCo
	 * for a URLab body or anything below one, Chaos for a simulating primitive or bone), at the physics rate; else
	 * Kinematic. Chaos / MuJoCo ask for that engine (Auto when it does not simulate the body). Kinematic: the
	 * transform differenced at the game rate (with bUsePhysicsVelocity, the nearest simulating primitive's
	 * velocity). Auto with bUsePhysicsVelocity off stays Kinematic, as before.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "IMU|Configuration")
	ERammsImuSource Source = ERammsImuSource::Auto;

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

	/** The source the latest reading actually came from (Chaos, MuJoCo or Kinematic). */
	UFUNCTION(BlueprintPure, Category = "Ramms|Sensors|IMU")
	ERammsImuSource GetActiveSource() const { return CurrentData.Source; }

	/** The source this sensor asks URammsPhysicsImuSubsystem for: Source, except that Auto with
	 *  bUsePhysicsVelocity off is Kinematic. */
	ERammsImuSource GetRequestedSource() const;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	/** The physics engine's latest sample of this sensor's frame, tracking it first if need be. False when Source
	 *  resolves to Kinematic or the engine has no sample yet; bOutPending: an engine serves the point but has no
	 *  sample just now (a moment without one repeats the last reading rather than switching to differencing). */
	bool SamplePhysics(FRammsImuSample& OutSample, bool& bOutPending);

	/** The point URammsPhysicsImuSubsystem tracks for this sensor (0: none), and the source it was asked for. */
	int32			PhysicsHandle = 0;
	ERammsImuSource TrackedSource = ERammsImuSource::Auto;

	/** When to ask the subsystem to track this sensor again after it refused (game time, s). */
	double NextTrackAttempt = 0.0;

	/** The engine sample the last measurement ended at (the next one's interval starts there), its point's
	 *  handle, and what that measurement read (repeated while the engine has no new step). */
	FRammsImuSample PreviousPhysicsSample;
	int32			PreviousPhysicsHandle = 0;
	bool			bPreviousPhysicsValid = false;
	FVector			LastIntervalAccel = FVector::ZeroVector;		   // coordinate acceleration, world (cm/s^2)
	FVector			LastIntervalAngularVelocity = FVector::ZeroVector; // world (deg/s)
	FQuat			LastIntervalFrame = FQuat::Identity;

	/** Whether the last measurement used an engine's samples: a change of path starts the differencing over. */
	bool bLastMeasurementPhysics = false;

	/** Game time of the last measurement that had a fresh engine sample (s): how long a gap is bridged. */
	double LastPhysicsSampleGameTime = 0.0;

	/** Game time of the last measurement (s): the interval the transform differencing divides by. */
	double LastMeasurementTime = 0.0;

	/** The interval the previous velocity was differenced over (s; 0: none). */
	double PreviousElapsed = 0.0;

	// Internal state for velocity/acceleration computation
	FVector	   PreviousVelocity = FVector::ZeroVector;
	bool	   bPreviousVelocityValid = false;
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
