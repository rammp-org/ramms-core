// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "RammsSensorRayTracer.h"
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

	/** Surface normal at the hit point.
	 *  CPU path: true geometric normal from the collision system.
	 *  GPU path: approximate normal derived from the ray/triangle front-face
	 *  direction — not a true mesh normal. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar")
	FVector HitNormal = FVector::ZeroVector;

	/** The actor that was hit (may be null).
	 *  Note: Not available when using GPU ray tracing path. */
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
 *
 * When bUseGPURayTracing is enabled and the system supports it, rays are traced on the
 * GPU via a compute shader with inline ray tracing (TraceRayInline). This path uses
 * RT cores when available and falls back to software ray tracing otherwise.
 * If GPU tracing is unavailable, the component automatically falls back to CPU LineTrace.
 *
 * Note: The GPU path cannot resolve HitActor (always null). If you need the hit actor,
 * disable GPU ray tracing or use the OnSonarDataUpdated delegate and perform a
 * supplementary CPU trace.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsSonarSensorComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	URammsSonarSensorComponent();

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

	/** Collision channel to trace against.
	 *  CPU path only — the GPU ray tracing path traces against the full TLAS
	 *  and does not filter by collision channel. Greyed out when GPU tracing
	 *  is enabled; set bUseGPURayTracing=false to use channel filtering. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Configuration",
		meta = (EditCondition = "!bUseGPURayTracing"))
	TEnumAsByte<ECollisionChannel> TraceChannel = ECC_Visibility;

	/** Whether to ignore the owning actor in traces.
	 *  CPU path only — the GPU ray tracing path cannot exclude specific actors.
	 *  Greyed out when GPU tracing is enabled. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Configuration",
		meta = (EditCondition = "!bUseGPURayTracing"))
	bool bIgnoreOwner = true;

	/** Use GPU ray tracing (compute shader with TraceRayInline) when available.
	 *  Falls back to CPU LineTrace if the system does not support ray tracing.
	 *  Note: The GPU path does not honor TraceChannel or bIgnoreOwner — it traces
	 *  against the full scene TLAS. Use CPU fallback if collision filtering is required.
	 *  GPU path also cannot resolve HitActor (always null). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Configuration")
	bool bUseGPURayTracing = true;

	// ========== Noise ==========

	/** Distance measurement Gaussian noise standard deviation (cm) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Noise", meta = (ClampMin = "0.0"))
	float DistanceNoiseStdDev = 0.0f;

	// ========== Debug ==========

	/** Draw debug rays and hit points at runtime */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Debug")
	bool bEnableDebugDisplay = false;

	/** Log sensor output to console */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Debug")
	bool bEnableDebugLogging = false;

	// ========== Shape Visualization ==========

	/** Draw the sensor cone shape during gameplay */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Visualization")
	bool bDrawShapeInGame = false;

	/** Draw the sensor cone shape in the editor viewport */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Visualization")
	bool bDrawShapeInEditor = true;

	/** Color of the sensor cone outline */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Visualization",
		meta = (EditCondition = "bDrawShapeInGame || bDrawShapeInEditor"))
	FColor ShapeColor = FColor(255, 165, 0); // Orange

	/** Thickness of cone outline lines */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Visualization",
		meta = (EditCondition = "bDrawShapeInGame || bDrawShapeInEditor", ClampMin = "0.1"))
	float ShapeLineThickness = 0.5f;

	/** Draw filled translucent planes on the cone surface */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Visualization",
		meta = (EditCondition = "bDrawShapeInGame || bDrawShapeInEditor"))
	bool bDrawShapePlanes = true;

	/** Color (with alpha) for the filled cone surface */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sonar|Visualization",
		meta = (EditCondition = "bDrawShapeInGame || bDrawShapeInEditor"))
	FLinearColor ShapePlaneColor = FLinearColor(1.0f, 0.65f, 0.0f, 0.05f);

	// ========== State ==========

	/** Most recent sensor reading */
	UPROPERTY(BlueprintReadOnly, Category = "Sonar|State")
	FSonarSensorData CurrentData;

	/** Whether the GPU path is currently active */
	UPROPERTY(BlueprintReadOnly, Category = "Sonar|State")
	bool bGPUPathActive = false;

	// ========== Events ==========

	/** Fires each measurement cycle with the latest data */
	UPROPERTY(BlueprintAssignable, Category = "Sonar|Events")
	FOnSonarDataUpdated OnSonarDataUpdated;

	// ========== Blueprint API ==========

	/** Get the latest sonar reading */
	UFUNCTION(BlueprintPure, Category = "Ramms|Sensors|Sonar")
	FSonarSensorData GetSonarData() const { return CurrentData; }

	/** Perform a single measurement right now (ignoring update rate).
	 *  Note: Always uses CPU path for immediate results. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensors|Sonar")
	FSonarSensorData MeasureNow();

protected:
	virtual void BeginPlay() override;

private:
	// ---- Shape visualization ----
	void DrawSensorShape();

	// ---- CPU path (original implementation) ----
	FSonarSensorData PerformCPUMeasurement() const;

	// ---- GPU path ----
	TArray<FSensorRayInput> BuildRayInputs() const;
	void					SubmitGPUMeasurement();
	bool					HarvestGPUResults();

	FRammsSensorTraceRequest PendingGPURequest;
	TArray<FSensorRayInput>	 PendingGPURays;
	bool					 bGPUAvailable = false;

	float TimeSinceLastUpdate = 0.0f;
};
