// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "RammsSonarSensorComponent.generated.h"

/**
 * Output data from a single sonar/radar measurement.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FSonarSensorData
{
	GENERATED_BODY()

	/** Measured distance to nearest obstacle (cm). -1 if nothing detected within range. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar")
	float Distance = -1.0f;

	/** Whether an obstacle was detected */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar")
	bool bHit = false;

	/** World-space location of the closest hit point */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar")
	FVector HitLocation = FVector::ZeroVector;

	/** Surface normal at the hit point */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar")
	FVector HitNormal = FVector::ZeroVector;

	/** The actor that was hit (may be null) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar")
	AActor* HitActor = nullptr;

	/** Timestamp of the measurement (seconds since game start) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar")
	float Timestamp = 0.0f;
};

/** Fired each time a new sonar measurement is produced */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnSonarDataUpdated, const FSonarSensorData&, Data);

/**
 * Simulated Sonar / RADAR distance sensor.
 * Uses a cone-shaped beam (approximated with multiple ray casts) to detect
 * the nearest obstacle within range. Inherits from USceneComponent so it
 * has a transform — the sensor fires along its local +X axis.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsSonarSensorComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	URammsSonarSensorComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// ========== Configuration ==========

	/** Maximum detection range (cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Configuration", meta = (ClampMin = "1.0"))
	float MaxRange = 400.0f;

	/** Minimum detection range (cm) — objects closer than this are not detected */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Configuration", meta = (ClampMin = "0.0"))
	float MinRange = 2.0f;

	/** Half-angle of the sonar cone (degrees). Total beam width = 2× this value. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Configuration", meta = (ClampMin = "0.0", ClampMax = "90.0"))
	float BeamHalfAngle = 15.0f;

	/** Number of rays used to approximate the cone beam. 1 = single ray (no cone). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Configuration", meta = (ClampMin = "1", ClampMax = "64"))
	int32 NumRays = 7;

	/** Update rate in Hz (0 = every tick) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Configuration", meta = (ClampMin = "0.0"))
	float UpdateRateHz = 40.0f;

	/** Collision channel to trace against */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Configuration")
	TEnumAsByte<ECollisionChannel> TraceChannel = ECC_Visibility;

	/** Whether to ignore the owning actor in traces */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Configuration")
	bool bIgnoreOwner = true;

	// ========== Noise ==========

	/** Distance measurement Gaussian noise standard deviation (cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Noise", meta = (ClampMin = "0.0"))
	float DistanceNoiseStdDev = 0.0f;

	// ========== Debug ==========

	/** Draw debug rays and hit points */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Debug")
	bool bEnableDebugDisplay = false;

	/** Log sensor output to console */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Debug")
	bool bEnableDebugLogging = false;

	// ========== State ==========

	/** Most recent sensor reading */
	UPROPERTY(BlueprintReadOnly, Category = "Sonar|State")
	FSonarSensorData CurrentData;

	// ========== Events ==========

	/** Fires each measurement cycle with the latest data */
	UPROPERTY(BlueprintAssignable, Category = "Sonar|Events")
	FOnSonarDataUpdated OnSonarDataUpdated;

	// ========== Blueprint API ==========

	/** Get the latest sonar reading */
	UFUNCTION(BlueprintPure, Category = "Ramms|Sensors|Sonar")
	FSonarSensorData GetSonarData() const { return CurrentData; }

	/** Perform a single measurement right now (ignoring update rate) */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensors|Sonar")
	FSonarSensorData MeasureNow();

private:
	/** Perform the multi-ray cone trace and return the result */
	FSonarSensorData PerformMeasurement() const;

	float TimeSinceLastUpdate = 0.0f;
};
