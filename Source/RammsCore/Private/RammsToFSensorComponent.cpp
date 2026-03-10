// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsToFSensorComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

URammsToFSensorComponent::URammsToFSensorComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void URammsToFSensorComponent::BeginPlay()
{
	Super::BeginPlay();
	TimeSinceLastUpdate = 0.0f;
}

void URammsToFSensorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Rate limiting
	if (UpdateRateHz > 0.0f)
	{
		TimeSinceLastUpdate += DeltaTime;
		const float UpdateInterval = 1.0f / UpdateRateHz;
		if (TimeSinceLastUpdate < UpdateInterval)
			return;
		TimeSinceLastUpdate -= UpdateInterval;
	}

	CurrentData = PerformMeasurement();
	OnToFDataUpdated.Broadcast(CurrentData);

	if (bEnableDebugLogging && GFrameCounter % 60 == 0)
	{
		if (SensorMode == EToFSensorMode::SinglePoint)
		{
			UE_LOG(LogTemp, Log, TEXT("[ToF] Mode=Single Hit=%d Distance=%.1f cm"),
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
			UE_LOG(LogTemp, Log, TEXT("[ToF] Mode=Grid(%dx%d) Hits=%d/%d MinDist=%.1f cm"),
				CurrentData.NumRows, CurrentData.NumColumns,
				HitCount, CurrentData.Distances.Num(), CurrentData.MinDistance);
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
	CurrentData = PerformMeasurement();
	OnToFDataUpdated.Broadcast(CurrentData);
	return CurrentData;
}

FToFSensorData URammsToFSensorComponent::PerformMeasurement() const
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
				// Map column to horizontal angle: left = +HalfH, right = -HalfH
				float HAngle = (Cols > 1)
					? FMath::Lerp(HalfH, -HalfH, (float)Col / (float)(Cols - 1))
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
				Dist += GaussianNoise(DistanceNoiseStdDev);
				Dist = FMath::Max(0.0f, Dist);
			}
			return Dist;
		}
	}

	return -1.0f;
}

float URammsToFSensorComponent::GaussianNoise(float StdDev) const
{
	if (StdDev <= 0.0f)
		return 0.0f;
	float U1 = FMath::FRand();
	float U2 = FMath::FRand();
	if (U1 < SMALL_NUMBER)
		U1 = SMALL_NUMBER;
	return StdDev * FMath::Sqrt(-2.0f * FMath::Loge(U1)) * FMath::Cos(2.0f * PI * U2);
}
