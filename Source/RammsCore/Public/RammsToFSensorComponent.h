// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "RammsToFSensorComponent.generated.h"

/** Operating mode for the ToF sensor */
UENUM(BlueprintType)
enum class EToFSensorMode : uint8
{
	SinglePoint UMETA(DisplayName = "Single Point"),
	Grid		UMETA(DisplayName = "Grid (NxM)")
};

/**
 * Output data from a Time-of-Flight sensor measurement.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FToFSensorData
{
	GENERATED_BODY()

	/** Distance readings in cm. Single element for SinglePoint mode, NxM for Grid mode.
	 *  Grid is row-major (index = row * NumColumns + col). -1 means no detection. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF")
	TArray<float> Distances;

	/** Number of grid rows (1 for SinglePoint mode) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF")
	int32 NumRows = 1;

	/** Number of grid columns (1 for SinglePoint mode) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF")
	int32 NumColumns = 1;

	/** Whether any zone detected an obstacle */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF")
	bool bAnyHit = false;

	/** Minimum distance across all zones (cm). -1 if nothing detected. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF")
	float MinDistance = -1.0f;

	/** Timestamp of the measurement (seconds since game start) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF")
	float Timestamp = 0.0f;
};

/** Fired each time a new ToF measurement is produced */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnToFDataUpdated, const FToFSensorData&, Data);

/**
 * Simulated Time-of-Flight (ToF) distance sensor.
 * Supports single-point mode (like VL53L0X) and grid mode (like VL53L5CX with 8×8 zones).
 * Fires along the component's local +X axis. Grid zones are distributed across the
 * configured field of view.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsToFSensorComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	URammsToFSensorComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// ========== Configuration ==========

	/** Sensor operating mode */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF|Configuration")
	EToFSensorMode SensorMode = EToFSensorMode::SinglePoint;

	/** Maximum detection range (cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF|Configuration", meta = (ClampMin = "1.0"))
	float MaxRange = 400.0f;

	/** Minimum detection range (cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF|Configuration", meta = (ClampMin = "0.0"))
	float MinRange = 1.0f;

	/** Horizontal field of view (degrees) for grid mode */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF|Configuration|Grid", meta = (ClampMin = "1.0", ClampMax = "120.0", EditCondition = "SensorMode == EToFSensorMode::Grid"))
	float HorizontalFOV = 45.0f;

	/** Vertical field of view (degrees) for grid mode */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF|Configuration|Grid", meta = (ClampMin = "1.0", ClampMax = "120.0", EditCondition = "SensorMode == EToFSensorMode::Grid"))
	float VerticalFOV = 45.0f;

	/** Number of grid rows (Y zones) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF|Configuration|Grid", meta = (ClampMin = "1", ClampMax = "64", EditCondition = "SensorMode == EToFSensorMode::Grid"))
	int32 GridRows = 8;

	/** Number of grid columns (X zones) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF|Configuration|Grid", meta = (ClampMin = "1", ClampMax = "64", EditCondition = "SensorMode == EToFSensorMode::Grid"))
	int32 GridColumns = 8;

	/** Update rate in Hz (0 = every tick) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF|Configuration", meta = (ClampMin = "0.0"))
	float UpdateRateHz = 15.0f;

	/** Collision channel to trace against */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF|Configuration")
	TEnumAsByte<ECollisionChannel> TraceChannel = ECC_Visibility;

	/** Whether to ignore the owning actor in traces */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF|Configuration")
	bool bIgnoreOwner = true;

	// ========== Noise ==========

	/** Distance measurement Gaussian noise standard deviation (cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF|Noise", meta = (ClampMin = "0.0"))
	float DistanceNoiseStdDev = 0.0f;

	// ========== Debug ==========

	/** Draw debug rays and hit points */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF|Debug")
	bool bEnableDebugDisplay = false;

	/** Log sensor output to console */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ToF|Debug")
	bool bEnableDebugLogging = false;

	// ========== State ==========

	/** Most recent sensor reading */
	UPROPERTY(BlueprintReadOnly, Category = "ToF|State")
	FToFSensorData CurrentData;

	// ========== Events ==========

	/** Fires each measurement cycle with the latest data */
	UPROPERTY(BlueprintAssignable, Category = "ToF|Events")
	FOnToFDataUpdated OnToFDataUpdated;

	// ========== Blueprint API ==========

	/** Get the latest ToF reading */
	UFUNCTION(BlueprintPure, Category = "Ramms|Sensors|ToF")
	FToFSensorData GetToFData() const { return CurrentData; }

	/** Get distance at a specific grid cell (row, col). Returns -1 if out of range or no hit. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Sensors|ToF")
	float GetDistanceAt(int32 Row, int32 Column) const;

	/** Perform a single measurement right now (ignoring update rate) */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensors|ToF")
	FToFSensorData MeasureNow();

private:
	/** Perform the measurement and return the result */
	FToFSensorData PerformMeasurement() const;

	/** Trace a single ray and return distance (-1 if no hit) */
	float TraceRay(const FVector& Start, const FVector& Direction, const FCollisionQueryParams& QueryParams) const;

	float TimeSinceLastUpdate = 0.0f;
};
