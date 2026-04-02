// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsToFSensorComponent.h"
#include "RammsSensorUtils.h"
#include "RammsSensorRayTracer.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

URammsToFSensorComponent::URammsToFSensorComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
#if WITH_EDITORONLY_DATA
	bTickInEditor = true;
#endif
}

void URammsToFSensorComponent::BeginPlay()
{
	Super::BeginPlay();
	TimeSinceLastUpdate = 0.0f;
	bGPUAvailable = FRammsSensorRayTracer::IsAvailable();
	bGPUPathActive = bUseGPURayTracing && bGPUAvailable;

	if (bUseGPURayTracing && !bGPUAvailable)
	{
		UE_LOG(LogTemp, Warning, TEXT("[ToF] GPU ray tracing requested but not available — falling back to CPU LineTrace"));
	}
	else if (bGPUPathActive)
	{
		UE_LOG(LogTemp, Log, TEXT("[ToF] GPU ray tracing enabled (compute shader with inline RT)"));
	}
}

void URammsToFSensorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
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

	// Re-check GPU availability in case CVar changed at runtime
	bGPUPathActive = bUseGPURayTracing && FRammsSensorRayTracer::IsAvailable();

	// If a GPU request is pending, try to harvest results
	if (PendingGPURequest.bPending)
	{
		if (HarvestGPUResults())
		{
			// Results harvested — data was already broadcast in HarvestGPUResults
		}
		else if ((GFrameCounter - PendingGPURequest.SubmitFrame) > FRammsSensorRayTracer::MaxReadbackWaitFrames)
		{
			// Timeout — discard pending request
			UE_LOG(LogTemp, Warning, TEXT("[ToF] GPU readback timed out after %d frames"), FRammsSensorRayTracer::MaxReadbackWaitFrames);
			PendingGPURequest.Reset();
		}
		else
		{
			// Still waiting — skip this tick
			return;
		}
	}

	// Rate limiting — keep only the remainder to avoid bursty catch-up after hitches
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
		// GPU path — submit rays asynchronously; results arrive in a future tick
		SubmitGPUMeasurement();
	}
	else
	{
		// CPU path — synchronous measurement (original implementation)
		CurrentData = PerformCPUMeasurement();
		OnToFDataUpdated.Broadcast(CurrentData);

		if (bEnableDebugLogging && GFrameCounter % 60 == 0)
		{
			if (SensorMode == EToFSensorMode::SinglePoint)
			{
				UE_LOG(LogTemp, Log, TEXT("[ToF] Mode=Single Hit=%d Distance=%.1f cm (CPU)"),
					CurrentData.bAnyHit ? 1 : 0, CurrentData.MinDistance);
			}
			else
			{
				int32 HitCount = 0;
				for (float D : CurrentData.Distances)
				{
					if (D >= 0.0f)
						HitCount++;
				}
				UE_LOG(LogTemp, Log, TEXT("[ToF] Mode=Grid(%dx%d) Hits=%d/%d MinDist=%.1f cm (CPU)"),
					CurrentData.NumRows, CurrentData.NumColumns,
					HitCount, CurrentData.Distances.Num(), CurrentData.MinDistance);
			}
		}
	}
}

float URammsToFSensorComponent::GetDistanceAt(int32 Row, int32 Column) const
{
	if (SensorMode == EToFSensorMode::SinglePoint)
	{
		return (CurrentData.Distances.Num() > 0) ? CurrentData.Distances[0] : -1.0f;
	}

	const int32 Index = Row * CurrentData.NumColumns + Column;
	if (CurrentData.Distances.IsValidIndex(Index))
	{
		return CurrentData.Distances[Index];
	}
	return -1.0f;
}

FToFSensorData URammsToFSensorComponent::MeasureNow()
{
	// MeasureNow always uses CPU for immediate synchronous results
	CurrentData = PerformCPUMeasurement();
	OnToFDataUpdated.Broadcast(CurrentData);
	return CurrentData;
}

// ============================================================================
// GPU Path
// ============================================================================

TArray<FSensorRayInput> URammsToFSensorComponent::BuildRayInputs() const
{
	TArray<FSensorRayInput> Rays;

	const FTransform SensorTransform = GetComponentTransform();
	const FVector	 Origin = SensorTransform.GetLocation();

	if (SensorMode == EToFSensorMode::SinglePoint)
	{
		FSensorRayInput Ray;
		Ray.Origin = FVector3f(Origin);
		Ray.Direction = FVector3f(SensorTransform.GetUnitAxis(EAxis::X));
		Ray.MinDistance = MinRange;
		Ray.MaxDistance = MaxRange;
		Rays.Add(Ray);
	}
	else
	{
		const int32 Rows = FMath::Max(1, GridRows);
		const int32 Cols = FMath::Max(1, GridColumns);
		Rays.Reserve(Rows * Cols);

		const float HalfH = HorizontalFOV * 0.5f;
		const float HalfV = VerticalFOV * 0.5f;

		for (int32 Row = 0; Row < Rows; ++Row)
		{
			float VAngle = (Rows > 1)
				? FMath::Lerp(HalfV, -HalfV, (float)Row / (float)(Rows - 1))
				: 0.0f;

			for (int32 Col = 0; Col < Cols; ++Col)
			{
				float HAngle = (Cols > 1)
					? FMath::Lerp(-HalfH, HalfH, (float)Col / (float)(Cols - 1))
					: 0.0f;

				float	VRad = FMath::DegreesToRadians(VAngle);
				float	HRad = FMath::DegreesToRadians(HAngle);
				FVector LocalDir(
					FMath::Cos(VRad) * FMath::Cos(HRad),
					FMath::Cos(VRad) * FMath::Sin(HRad),
					FMath::Sin(VRad));
				LocalDir.Normalize();

				FVector WorldDir = SensorTransform.TransformVectorNoScale(LocalDir);
				WorldDir.Normalize();

				FSensorRayInput Ray;
				Ray.Origin = FVector3f(Origin);
				Ray.Direction = FVector3f(WorldDir);
				Ray.MinDistance = MinRange;
				Ray.MaxDistance = MaxRange;
				Rays.Add(Ray);
			}
		}
	}

	return Rays;
}

void URammsToFSensorComponent::SubmitGPUMeasurement()
{
	TArray<FSensorRayInput> Rays = BuildRayInputs();
	if (Rays.Num() == 0)
		return;

	// Save a copy of world-space rays for debug drawing when results arrive
	PendingGPURays = Rays;

	UWorld* World = GetWorld();
	PendingGPURequest = FRammsSensorRayTracer::SubmitTraces(Rays, World ? World->Scene : nullptr);
}

bool URammsToFSensorComponent::HarvestGPUResults()
{
	if (!FRammsSensorRayTracer::IsRequestReady(PendingGPURequest))
		return false;

	TArray<FSensorRayOutput> Results = FRammsSensorRayTracer::HarvestResults(PendingGPURequest);

	UWorld* World = GetWorld();

	FToFSensorData Data;
	Data.Timestamp = World ? World->GetTimeSeconds() : 0.0f;
	Data.MinDistance = -1.0f;
	Data.bAnyHit = false;

	if (SensorMode == EToFSensorMode::SinglePoint)
	{
		Data.NumRows = 1;
		Data.NumColumns = 1;
		Data.Distances.SetNum(1);

		if (Results.Num() > 0 && Results[0].bHit)
		{
			float Dist = Results[0].HitDistance;
			if (DistanceNoiseStdDev > 0.0f)
			{
				Dist += RammsSensorUtils::GaussianNoise(DistanceNoiseStdDev);
				if (Dist < MinRange || Dist > MaxRange)
				{
					Data.Distances[0] = -1.0f;
				}
				else
				{
					Data.Distances[0] = Dist;
					Data.bAnyHit = true;
					Data.MinDistance = Dist;
				}
			}
			else
			{
				Data.Distances[0] = Dist;
				Data.bAnyHit = true;
				Data.MinDistance = Dist;
			}
		}
		else
		{
			Data.Distances[0] = -1.0f;
		}
	}
	else
	{
		const int32 Rows = FMath::Max(1, GridRows);
		const int32 Cols = FMath::Max(1, GridColumns);
		Data.NumRows = Rows;
		Data.NumColumns = Cols;
		Data.Distances.SetNum(Rows * Cols);

		for (int32 i = 0; i < Results.Num() && i < Rows * Cols; ++i)
		{
			if (Results[i].bHit)
			{
				float Dist = Results[i].HitDistance;
				if (DistanceNoiseStdDev > 0.0f)
				{
					Dist += RammsSensorUtils::GaussianNoise(DistanceNoiseStdDev);
					if (Dist < MinRange || Dist > MaxRange)
					{
						Data.Distances[i] = -1.0f;
						continue;
					}
				}
				Data.Distances[i] = Dist;
				Data.bAnyHit = true;
				if (Data.MinDistance < 0.0f || Dist < Data.MinDistance)
				{
					Data.MinDistance = Dist;
				}
			}
			else
			{
				Data.Distances[i] = -1.0f;
			}
		}
	}

	CurrentData = Data;
	OnToFDataUpdated.Broadcast(CurrentData);

	if (bEnableDebugLogging && GFrameCounter % 60 == 0)
	{
		if (SensorMode == EToFSensorMode::SinglePoint)
		{
			UE_LOG(LogTemp, Log, TEXT("[ToF] Mode=Single Hit=%d Distance=%.1f cm (GPU)"),
				CurrentData.bAnyHit ? 1 : 0, CurrentData.MinDistance);
		}
		else
		{
			int32 HitCount = 0;
			for (float D : CurrentData.Distances)
			{
				if (D >= 0.0f)
					HitCount++;
			}
			UE_LOG(LogTemp, Log, TEXT("[ToF] Mode=Grid(%dx%d) Hits=%d/%d MinDist=%.1f cm (GPU)"),
				CurrentData.NumRows, CurrentData.NumColumns,
				HitCount, CurrentData.Distances.Num(), CurrentData.MinDistance);
		}
	}

	// Debug visualization — draw rays and hit points using world-space ray data
	if (bEnableDebugDisplay && World)
	{
		const FVector Origin(GetComponentLocation());
		for (int32 i = 0; i < Results.Num() && i < PendingGPURays.Num(); ++i)
		{
			const FSensorRayInput& Ray = PendingGPURays[i];
			const FVector		   RayOrigin(Ray.Origin);
			const FVector		   RayDir(Ray.Direction);

			if (Results[i].bHit && Results[i].HitDistance > 0.0f)
			{
				FVector HitPoint = RayOrigin + RayDir * Results[i].HitDistance;
				DrawDebugLine(World, RayOrigin, HitPoint, FColor::Green, false, -1.0f, 0, 0.3f);
				DrawDebugPoint(World, HitPoint, 3.0f, FColor::Red, false, -1.0f);
			}
			else
			{
				FVector EndPoint = RayOrigin + RayDir * Ray.MaxDistance;
				DrawDebugLine(World, RayOrigin, EndPoint, FColor(64, 64, 0), false, -1.0f, 0, 0.2f);
			}
		}
	}
	PendingGPURays.Reset();

	return true;
}

// ============================================================================
// Shape Visualization
// ============================================================================

void URammsToFSensorComponent::DrawSensorShape()
{
	UWorld* World = GetWorld();
	if (!World)
		return;

	FTransform SensorTransform = GetComponentTransform();
	SensorTransform.SetScale3D(FVector::OneVector);
	const FVector Origin = SensorTransform.GetLocation();

	const float NearDist = FMath::Max(1.0f, MinRange);
	const float FarDist = FMath::Max(NearDist + 1.0f, MaxRange);

	if (SensorMode == EToFSensorMode::SinglePoint)
	{
		// Single line along +X axis
		const FVector EndPoint = SensorTransform.TransformPosition(FVector(FarDist, 0.0f, 0.0f));
		const FVector NearPoint = SensorTransform.TransformPosition(FVector(NearDist, 0.0f, 0.0f));
		DrawDebugLine(World, Origin, EndPoint, ShapeColor, false, -1.0f, 0, ShapeLineThickness);

		// Small crosshair at the near distance
		const float	  CrossSize = FMath::Max(1.0f, FarDist * 0.01f);
		const FVector CrossY = SensorTransform.TransformVector(FVector(0.0f, CrossSize, 0.0f));
		const FVector CrossZ = SensorTransform.TransformVector(FVector(0.0f, 0.0f, CrossSize));
		DrawDebugLine(World, NearPoint - CrossY, NearPoint + CrossY, ShapeColor, false, -1.0f, 0, ShapeLineThickness);
		DrawDebugLine(World, NearPoint - CrossZ, NearPoint + CrossZ, ShapeColor, false, -1.0f, 0, ShapeLineThickness);
	}
	else
	{
		// Grid mode — draw a frustum defined by HorizontalFOV × VerticalFOV
		const float HalfH = FMath::DegreesToRadians(HorizontalFOV * 0.5f);
		const float HalfV = FMath::DegreesToRadians(VerticalFOV * 0.5f);

		// Frustum corner directions in local space (sensor fires along +X)
		// Tangent of half-angle gives the offset at unit distance along X
		const float TanH = FMath::Tan(HalfH);
		const float TanV = FMath::Tan(HalfV);

		// Local-space corner offsets at unit distance along X
		// Index: 0=BotLeft, 1=BotRight, 2=TopRight, 3=TopLeft
		const FVector LocalDirs[4] = {
			FVector(1.0f, -TanH, -TanV),
			FVector(1.0f, TanH, -TanV),
			FVector(1.0f, TanH, TanV),
			FVector(1.0f, -TanH, TanV),
		};

		FVector NearWorld[4];
		FVector FarWorld[4];
		for (int32 i = 0; i < 4; ++i)
		{
			FVector Dir = LocalDirs[i].GetSafeNormal();
			NearWorld[i] = SensorTransform.TransformPosition(Dir * NearDist);
			FarWorld[i] = SensorTransform.TransformPosition(Dir * FarDist);
		}

		// 4 edge rays from origin to far corners
		for (int32 i = 0; i < 4; ++i)
		{
			DrawDebugLine(World, Origin, FarWorld[i], ShapeColor, false, -1.0f, 0, ShapeLineThickness);
		}

		// Near and far rectangles + connecting edges
		for (int32 i = 0; i < 4; ++i)
		{
			int32 Next = (i + 1) % 4;
			DrawDebugLine(World, FarWorld[i], FarWorld[Next], ShapeColor, false, -1.0f, 0, ShapeLineThickness);
			DrawDebugLine(World, NearWorld[i], NearWorld[Next], ShapeColor, false, -1.0f, 0, ShapeLineThickness);
			DrawDebugLine(World, NearWorld[i], FarWorld[i], ShapeColor, false, -1.0f, 0, ShapeLineThickness);
		}

		// Center line (sensor axis)
		const FVector CenterFar = SensorTransform.TransformPosition(FVector(FarDist, 0.0f, 0.0f));
		DrawDebugLine(World, Origin, CenterFar, ShapeColor, false, -1.0f, 0, ShapeLineThickness * 0.5f);

		// Filled planes
		if (bDrawShapePlanes)
		{
			const FColor PlaneCol = ShapePlaneColor.ToFColor(true);

			auto DrawQuad = [&](const FVector& A, const FVector& B, const FVector& C, const FVector& D) {
				TArray<FVector> Verts;
				Verts.Reserve(4);
				Verts.Add(A);
				Verts.Add(B);
				Verts.Add(C);
				Verts.Add(D);
				TArray<int32> Indices;
				Indices.Reserve(6);
				Indices.Add(0);
				Indices.Add(1);
				Indices.Add(2);
				Indices.Add(0);
				Indices.Add(2);
				Indices.Add(3);
				DrawDebugMesh(World, Verts, Indices, PlaneCol, false, -1.0f, 0);
			};

			// Near and far planes
			DrawQuad(NearWorld[0], NearWorld[1], NearWorld[2], NearWorld[3]);
			DrawQuad(FarWorld[0], FarWorld[1], FarWorld[2], FarWorld[3]);

			// Side planes
			DrawQuad(NearWorld[0], NearWorld[3], FarWorld[3], FarWorld[0]); // Left
			DrawQuad(NearWorld[1], NearWorld[2], FarWorld[2], FarWorld[1]); // Right
			DrawQuad(NearWorld[0], NearWorld[1], FarWorld[1], FarWorld[0]); // Bottom
			DrawQuad(NearWorld[3], NearWorld[2], FarWorld[2], FarWorld[3]); // Top
		}
	}

	// Origin crosshair
	DrawDebugCrosshairs(World, Origin, SensorTransform.Rotator(), 5.0f, ShapeColor, false, -1.0f, 0);
}

// ============================================================================
// CPU Path (original implementation, preserved as-is)
// ============================================================================

FToFSensorData URammsToFSensorComponent::PerformCPUMeasurement() const
{
	UWorld* World = GetWorld();
	if (!World)
		return FToFSensorData();

	const FTransform SensorTransform = GetComponentTransform();
	const FVector	 Origin = SensorTransform.GetLocation();

	FCollisionQueryParams QueryParams;
	QueryParams.bTraceComplex = false;
	if (bIgnoreOwner && GetOwner())
	{
		QueryParams.AddIgnoredActor(GetOwner());
	}

	FToFSensorData Result;
	Result.Timestamp = World->GetTimeSeconds();
	Result.MinDistance = -1.0f;
	Result.bAnyHit = false;

	if (SensorMode == EToFSensorMode::SinglePoint)
	{
		// Single ray along +X
		Result.NumRows = 1;
		Result.NumColumns = 1;
		Result.Distances.SetNum(1);

		FVector Dir = SensorTransform.GetUnitAxis(EAxis::X);
		float	Dist = TraceRay(Origin, Dir, QueryParams);
		Result.Distances[0] = Dist;

		if (Dist >= 0.0f)
		{
			Result.bAnyHit = true;
			Result.MinDistance = Dist;
		}

		if (bEnableDebugDisplay)
		{
			FVector End = Origin + Dir * MaxRange;
			if (Dist >= 0.0f)
			{
				DrawDebugLine(World, Origin, Origin + Dir * Dist, FColor::Green, false, -1.0f, 0, 0.5f);
				DrawDebugPoint(World, Origin + Dir * Dist, 4.0f, FColor::Red, false, -1.0f);
			}
			else
			{
				DrawDebugLine(World, Origin, End, FColor::Yellow, false, -1.0f, 0, 0.3f);
			}
		}
	}
	else // Grid mode
	{
		const int32 Rows = FMath::Max(1, GridRows);
		const int32 Cols = FMath::Max(1, GridColumns);
		Result.NumRows = Rows;
		Result.NumColumns = Cols;
		Result.Distances.SetNum(Rows * Cols);

		const float HalfH = HorizontalFOV * 0.5f;
		const float HalfV = VerticalFOV * 0.5f;

		for (int32 Row = 0; Row < Rows; ++Row)
		{
			// Map row to vertical angle: top = +HalfV, bottom = -HalfV
			float VAngle = (Rows > 1)
				? FMath::Lerp(HalfV, -HalfV, (float)Row / (float)(Rows - 1))
				: 0.0f;

			for (int32 Col = 0; Col < Cols; ++Col)
			{
				// Map column to horizontal angle: col 0 = left (-HalfH / -Y), col max = right (+HalfH / +Y)
				float HAngle = (Cols > 1)
					? FMath::Lerp(-HalfH, HalfH, (float)Col / (float)(Cols - 1))
					: 0.0f;

				// Build ray direction in sensor-local space (+X forward, +Y right, +Z up)
				float	VRad = FMath::DegreesToRadians(VAngle);
				float	HRad = FMath::DegreesToRadians(HAngle);
				FVector LocalDir(
					FMath::Cos(VRad) * FMath::Cos(HRad),
					FMath::Cos(VRad) * FMath::Sin(HRad),
					FMath::Sin(VRad));
				LocalDir.Normalize();

				FVector WorldDir = SensorTransform.TransformVectorNoScale(LocalDir);
				WorldDir.Normalize();

				int32 Index = Row * Cols + Col;
				float Dist = TraceRay(Origin, WorldDir, QueryParams);
				Result.Distances[Index] = Dist;

				if (Dist >= 0.0f)
				{
					Result.bAnyHit = true;
					if (Result.MinDistance < 0.0f || Dist < Result.MinDistance)
					{
						Result.MinDistance = Dist;
					}
				}

				if (bEnableDebugDisplay)
				{
					FVector End = Origin + WorldDir * MaxRange;
					if (Dist >= 0.0f)
					{
						DrawDebugLine(World, Origin, Origin + WorldDir * Dist, FColor::Green, false, -1.0f, 0, 0.3f);
						DrawDebugPoint(World, Origin + WorldDir * Dist, 3.0f, FColor::Red, false, -1.0f);
					}
					else
					{
						DrawDebugLine(World, Origin, End, FColor(64, 64, 0), false, -1.0f, 0, 0.2f);
					}
				}
			}
		}
	}

	return Result;
}

float URammsToFSensorComponent::TraceRay(const FVector& Start, const FVector& Direction, const FCollisionQueryParams& QueryParams) const
{
	UWorld* World = GetWorld();
	if (!World)
		return -1.0f;

	FVector	   End = Start + Direction * MaxRange;
	FHitResult HitResult;

	if (World->LineTraceSingleByChannel(HitResult, Start, End, TraceChannel, QueryParams))
	{
		float Dist = HitResult.Distance;
		if (Dist >= MinRange && Dist <= MaxRange)
		{
			if (DistanceNoiseStdDev > 0.0f)
			{
				Dist += RammsSensorUtils::GaussianNoise(DistanceNoiseStdDev);
				if (Dist < MinRange || Dist > MaxRange)
				{
					return -1.0f;
				}
			}
			return Dist;
		}
	}

	return -1.0f;
}
