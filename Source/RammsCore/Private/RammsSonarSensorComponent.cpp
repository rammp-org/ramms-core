// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsSonarSensorComponent.h"
#include "RammsSensorUtils.h"
#include "RammsSensorRayTracer.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

URammsSonarSensorComponent::URammsSonarSensorComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
#if WITH_EDITORONLY_DATA
	bTickInEditor = true;
#endif
}

void URammsSonarSensorComponent::BeginPlay()
{
	Super::BeginPlay();
	TimeSinceLastUpdate = 0.0f;
	bGPUAvailable = FRammsSensorRayTracer::IsAvailable();
	bGPUPathActive = bUseGPURayTracing && bGPUAvailable;

	if (bUseGPURayTracing && !bGPUAvailable)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Sonar] GPU ray tracing requested but not available — falling back to CPU LineTrace"));
	}
	else if (bGPUPathActive)
	{
		UE_LOG(LogTemp, Log, TEXT("[Sonar] GPU ray tracing enabled (compute shader with inline RT)"));
	}
}

void URammsSonarSensorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Shape visualization (runs every tick, independent of measurement rate)
	{
		bool bShouldDraw = false;
		if (bDrawShapeInGame && GetWorld() && GetWorld()->IsGameWorld())
			bShouldDraw = true;
#if WITH_EDITOR
		if (bDrawShapeInEditor && GIsEditor && GetWorld() && !GetWorld()->IsGameWorld())
			bShouldDraw = true;
#endif
		if (bShouldDraw)
			DrawSensorShape();
	}

	// Sensing pipeline only runs in game worlds (not in editor viewport)
	if (!GetWorld() || !GetWorld()->IsGameWorld())
		return;

	bGPUPathActive = bUseGPURayTracing && FRammsSensorRayTracer::IsAvailable();
	if (PendingGPURequest.bPending)
	{
		if (HarvestGPUResults())
		{
			// Results harvested and broadcast
		}
		else if ((GFrameCounter - PendingGPURequest.SubmitFrame) > FRammsSensorRayTracer::MaxReadbackWaitFrames)
		{
			UE_LOG(LogTemp, Warning, TEXT("[Sonar] GPU readback timed out after %d frames"), FRammsSensorRayTracer::MaxReadbackWaitFrames);
			PendingGPURequest.Reset();
		}
		else
		{
			return;
		}
	}

	// Rate limiting
	if (UpdateRateHz > 0.0f)
	{
		TimeSinceLastUpdate += DeltaTime;
		const float UpdateInterval = 1.0f / UpdateRateHz;
		if (TimeSinceLastUpdate < UpdateInterval)
			return;
		TimeSinceLastUpdate = FMath::Fmod(TimeSinceLastUpdate, UpdateInterval);
	}

	if (bGPUPathActive)
	{
		SubmitGPUMeasurement();
	}
	else
	{
		CurrentData = PerformCPUMeasurement();
		OnSonarDataUpdated.Broadcast(CurrentData);

		if (bEnableDebugLogging && GFrameCounter % 60 == 0)
		{
			UE_LOG(LogTemp, Log, TEXT("[Sonar] Hit=%d Distance=%.1f cm (CPU)"),
				CurrentData.bHit ? 1 : 0, CurrentData.Distance);
		}
	}
}

FSonarSensorData URammsSonarSensorComponent::MeasureNow()
{
	CurrentData = PerformCPUMeasurement();
	OnSonarDataUpdated.Broadcast(CurrentData);
	return CurrentData;
}

// ============================================================================
// GPU Path
// ============================================================================

TArray<FSensorRayInput> URammsSonarSensorComponent::BuildRayInputs() const
{
	TArray<FSensorRayInput> Rays;

	const FTransform SensorTransform = GetComponentTransform();
	const FVector	 Origin = SensorTransform.GetLocation();
	const FVector	 ForwardDir = SensorTransform.GetUnitAxis(EAxis::X);

	const int32 RayCount = FMath::Max(1, NumRays);
	Rays.Reserve(RayCount);

	for (int32 i = 0; i < RayCount; ++i)
	{
		FVector RayDir;
		if (RayCount == 1 || BeamHalfAngle <= 0.0f)
		{
			RayDir = ForwardDir;
		}
		else
		{
			// Vogel/golden-angle spiral distribution (same as CPU path)
			static constexpr float GoldenAngle = 2.399963f;
			float				   t = (float)i / (float)(RayCount - 1);
			float				   ConeAngle = BeamHalfAngle * FMath::Sqrt(t);
			float				   AzimuthAngle = (float)i * GoldenAngle;

			float	ConeRad = FMath::DegreesToRadians(ConeAngle);
			FVector LocalDir(
				FMath::Cos(ConeRad),
				FMath::Sin(ConeRad) * FMath::Cos(AzimuthAngle),
				FMath::Sin(ConeRad) * FMath::Sin(AzimuthAngle));

			RayDir = SensorTransform.TransformVectorNoScale(LocalDir);
			RayDir.Normalize();
		}

		FSensorRayInput Ray;
		Ray.Origin = FVector3f(Origin);
		Ray.Direction = FVector3f(RayDir);
		Ray.MinDistance = MinRange;
		Ray.MaxDistance = MaxRange;
		Rays.Add(Ray);
	}

	return Rays;
}

void URammsSonarSensorComponent::SubmitGPUMeasurement()
{
	TArray<FSensorRayInput> Rays = BuildRayInputs();
	if (Rays.Num() == 0)
		return;

	PendingGPURays = Rays;

	UWorld* World = GetWorld();
	PendingGPURequest = FRammsSensorRayTracer::SubmitTraces(Rays, World ? World->Scene : nullptr);
}

bool URammsSonarSensorComponent::HarvestGPUResults()
{
	if (!FRammsSensorRayTracer::IsRequestReady(PendingGPURequest))
		return false;

	TArray<FSensorRayOutput> Results = FRammsSensorRayTracer::HarvestResults(PendingGPURequest);

	UWorld* World = GetWorld();

	FSonarSensorData Data;
	Data.Timestamp = World ? World->GetTimeSeconds() : 0.0f;
	Data.Distance = -1.0f;
	Data.bHit = false;
	Data.HitActor = nullptr; // GPU path cannot resolve actors

	float ClosestDist = MaxRange + 1.0f;

	// Use the cached world-space rays from submit time for correct hit locations
	// even if the sensor has moved since submission (1–2 frame latency)
	const TArray<FSensorRayInput>& Rays = PendingGPURays;

	for (int32 i = 0; i < Results.Num(); ++i)
	{
		if (Results[i].bHit)
		{
			float Dist = Results[i].HitDistance;
			if (Dist >= MinRange && Dist < ClosestDist)
			{
				ClosestDist = Dist;
				Data.bHit = true;

				FVector RayOrigin = FVector(Rays[i].Origin);
				FVector RayDir = FVector(Rays[i].Direction);
				Data.HitLocation = RayOrigin + RayDir * Dist;
				Data.HitNormal = FVector(Results[i].HitNormal);
			}
		}
	}

	if (Data.bHit)
	{
		Data.Distance = ClosestDist;

		// Apply noise (same model as CPU path)
		if (DistanceNoiseStdDev > 0.0f)
		{
			Data.Distance += RammsSensorUtils::GaussianNoise(DistanceNoiseStdDev);
			if (Data.Distance < MinRange || Data.Distance > MaxRange)
			{
				Data.bHit = false;
				Data.Distance = -1.0f;
				Data.HitLocation = FVector::ZeroVector;
				Data.HitNormal = FVector::ZeroVector;
			}
		}
	}

	CurrentData = Data;
	OnSonarDataUpdated.Broadcast(CurrentData);

	if (bEnableDebugLogging && GFrameCounter % 60 == 0)
	{
		UE_LOG(LogTemp, Log, TEXT("[Sonar] Hit=%d Distance=%.1f cm (GPU)"),
			CurrentData.bHit ? 1 : 0, CurrentData.Distance);
	}

	// Debug visualization — draw rays and hit points using world-space ray data
	if (bEnableDebugDisplay && World)
	{
		for (int32 i = 0; i < Results.Num() && i < PendingGPURays.Num(); ++i)
		{
			const FSensorRayInput& Ray = PendingGPURays[i];
			const FVector		   RayOrigin(Ray.Origin);
			const FVector		   RayDir(Ray.Direction);

			if (Results[i].bHit && Results[i].HitDistance >= MinRange)
			{
				FVector HitPoint = RayOrigin + RayDir * Results[i].HitDistance;
				DrawDebugLine(World, RayOrigin, HitPoint, FColor::Green, false, -1.0f, 0, 0.5f);
				DrawDebugPoint(World, HitPoint, 4.0f, FColor::Red, false, -1.0f);
			}
			else
			{
				FVector EndPoint = RayOrigin + RayDir * Ray.MaxDistance;
				DrawDebugLine(World, RayOrigin, EndPoint, FColor::Yellow, false, -1.0f, 0, 0.3f);
			}
		}
	}
	PendingGPURays.Reset();

	return true;
}

// ============================================================================
// Shape Visualization
// ============================================================================

void URammsSonarSensorComponent::DrawSensorShape()
{
	UWorld* World = GetWorld();
	if (!World)
		return;

	FTransform SensorTransform = GetComponentTransform();
	SensorTransform.SetScale3D(FVector::OneVector);
	const FVector Origin = SensorTransform.GetLocation();

	const float NearDist = FMath::Max(1.0f, MinRange);
	const float FarDist = FMath::Max(NearDist + 1.0f, MaxRange);
	const float HalfAngleRad = FMath::DegreesToRadians(FMath::Clamp(BeamHalfAngle, 0.1f, 89.0f));

	// Number of segments for the cone circles
	constexpr int32 NumSegments = 24;
	const float		AngleStep = 2.0f * PI / NumSegments;

	// Compute circle points at near and far distances in local space
	// Cone radius at distance D = D * tan(halfAngle)
	const float NearRadius = NearDist * FMath::Tan(HalfAngleRad);
	const float FarRadius = FarDist * FMath::Tan(HalfAngleRad);

	TArray<FVector> NearCircle;
	TArray<FVector> FarCircle;
	NearCircle.SetNum(NumSegments);
	FarCircle.SetNum(NumSegments);

	for (int32 i = 0; i < NumSegments; ++i)
	{
		const float Angle = AngleStep * i;
		const float CosA = FMath::Cos(Angle);
		const float SinA = FMath::Sin(Angle);

		// Local space: +X is forward, Y/Z are the lateral plane
		FVector LocalNear(NearDist, NearRadius * CosA, NearRadius * SinA);
		FVector LocalFar(FarDist, FarRadius * CosA, FarRadius * SinA);

		NearCircle[i] = SensorTransform.TransformPosition(LocalNear);
		FarCircle[i] = SensorTransform.TransformPosition(LocalFar);
	}

	// Draw near and far circles
	for (int32 i = 0; i < NumSegments; ++i)
	{
		int32 Next = (i + 1) % NumSegments;
		DrawDebugLine(World, NearCircle[i], NearCircle[Next], ShapeColor, false, -1.0f, 0, ShapeLineThickness);
		DrawDebugLine(World, FarCircle[i], FarCircle[Next], ShapeColor, false, -1.0f, 0, ShapeLineThickness);
	}

	// Draw 4 edge lines from origin through near circle to far circle (at 0°, 90°, 180°, 270°)
	const int32 EdgeIndices[4] = { 0, NumSegments / 4, NumSegments / 2, 3 * NumSegments / 4 };
	for (int32 Idx : EdgeIndices)
	{
		DrawDebugLine(World, Origin, FarCircle[Idx], ShapeColor, false, -1.0f, 0, ShapeLineThickness);
		DrawDebugLine(World, NearCircle[Idx], FarCircle[Idx], ShapeColor, false, -1.0f, 0, ShapeLineThickness);
	}

	// Center axis line
	const FVector CenterFar = SensorTransform.TransformPosition(FVector(FarDist, 0.0f, 0.0f));
	DrawDebugLine(World, Origin, CenterFar, ShapeColor, false, -1.0f, 0, ShapeLineThickness * 0.5f);

	// Filled cone surface
	if (bDrawShapePlanes)
	{
		const FColor PlaneCol = ShapePlaneColor.ToFColor(true);

		// Far cap (filled disc)
		{
			TArray<FVector> Verts;
			TArray<int32>	Indices;
			Verts.Reserve(NumSegments + 1);
			Indices.Reserve(NumSegments * 3);

			Verts.Add(CenterFar);
			for (int32 i = 0; i < NumSegments; ++i)
				Verts.Add(FarCircle[i]);

			for (int32 i = 0; i < NumSegments; ++i)
			{
				int32 Next = (i + 1) % NumSegments;
				Indices.Add(0);
				Indices.Add(i + 1);
				Indices.Add(Next + 1);
			}
			DrawDebugMesh(World, Verts, Indices, PlaneCol, false, -1.0f, 0);
		}

		// Cone side surface (triangle fan from origin to far circle)
		{
			TArray<FVector> Verts;
			TArray<int32>	Indices;
			Verts.Reserve(NumSegments + 1);
			Indices.Reserve(NumSegments * 3);

			Verts.Add(Origin);
			for (int32 i = 0; i < NumSegments; ++i)
				Verts.Add(FarCircle[i]);

			for (int32 i = 0; i < NumSegments; ++i)
			{
				int32 Next = (i + 1) % NumSegments;
				Indices.Add(0);
				Indices.Add(i + 1);
				Indices.Add(Next + 1);
			}
			DrawDebugMesh(World, Verts, Indices, PlaneCol, false, -1.0f, 0);
		}
	}

	// Origin crosshair
	DrawDebugCrosshairs(World, Origin, SensorTransform.Rotator(), 5.0f, ShapeColor, false, -1.0f, 0);
}

// ============================================================================
// CPU Path (original implementation, preserved as-is)
// ============================================================================

FSonarSensorData URammsSonarSensorComponent::PerformCPUMeasurement() const
{
	UWorld* World = GetWorld();
	if (!World)
		return FSonarSensorData();

	const FTransform SensorTransform = GetComponentTransform();
	const FVector	 Origin = SensorTransform.GetLocation();
	const FVector	 ForwardDir = SensorTransform.GetUnitAxis(EAxis::X);

	FCollisionQueryParams QueryParams;
	QueryParams.bTraceComplex = false;
	if (bIgnoreOwner && GetOwner())
	{
		QueryParams.AddIgnoredActor(GetOwner());
	}

	FSonarSensorData Result;
	Result.Timestamp = World->GetTimeSeconds();
	Result.Distance = -1.0f;
	Result.bHit = false;

	float ClosestDist = MaxRange + 1.0f;

	// Generate ray directions within the cone
	const int32 RayCount = FMath::Max(1, NumRays);
	for (int32 i = 0; i < RayCount; ++i)
	{
		FVector RayDir;
		if (RayCount == 1 || BeamHalfAngle <= 0.0f)
		{
			// Single ray — straight ahead
			RayDir = ForwardDir;
		}
		else
		{
			// Distribute rays in a cone pattern using a Vogel/golden-angle spiral
			static constexpr float GoldenAngle = 2.399963f;				 // radians (PI * (3 - sqrt(5)))
			float				   t = (float)i / (float)(RayCount - 1); // 0 to 1
			float				   ConeAngle = BeamHalfAngle * FMath::Sqrt(t);
			float				   AzimuthAngle = (float)i * GoldenAngle;

			// Build direction in sensor-local space then transform to world
			float	ConeRad = FMath::DegreesToRadians(ConeAngle);
			FVector LocalDir(
				FMath::Cos(ConeRad),
				FMath::Sin(ConeRad) * FMath::Cos(AzimuthAngle),
				FMath::Sin(ConeRad) * FMath::Sin(AzimuthAngle));

			RayDir = SensorTransform.TransformVectorNoScale(LocalDir);
			RayDir.Normalize();
		}

		FVector TraceEnd = Origin + RayDir * MaxRange;

		FHitResult HitResult;
		if (World->LineTraceSingleByChannel(HitResult, Origin, TraceEnd, TraceChannel, QueryParams))
		{
			float Dist = HitResult.Distance;
			if (Dist >= MinRange && Dist < ClosestDist)
			{
				ClosestDist = Dist;
				Result.bHit = true;
				Result.HitLocation = HitResult.ImpactPoint;
				Result.HitNormal = HitResult.ImpactNormal;
				Result.HitActor = HitResult.GetActor();
			}
		}

		// Debug draw
		if (bEnableDebugDisplay)
		{
			if (HitResult.bBlockingHit && HitResult.Distance >= MinRange)
			{
				DrawDebugLine(World, Origin, HitResult.ImpactPoint, FColor::Green, false, -1.0f, 0, 0.5f);
				DrawDebugPoint(World, HitResult.ImpactPoint, 4.0f, FColor::Red, false, -1.0f);
			}
			else
			{
				DrawDebugLine(World, Origin, Origin + RayDir * MaxRange, FColor::Yellow, false, -1.0f, 0, 0.3f);
			}
		}
	}

	if (Result.bHit)
	{
		Result.Distance = ClosestDist;

		// Add noise and clamp to configured range
		if (DistanceNoiseStdDev > 0.0f)
		{
			Result.Distance += RammsSensorUtils::GaussianNoise(DistanceNoiseStdDev);
			if (Result.Distance < MinRange || Result.Distance > MaxRange)
			{
				// Noise pushed outside valid range — treat as no detection
				Result.bHit = false;
				Result.Distance = -1.0f;
				Result.HitLocation = FVector::ZeroVector;
				Result.HitNormal = FVector::ZeroVector;
				Result.HitActor = nullptr;
			}
		}
	}

	return Result;
}
