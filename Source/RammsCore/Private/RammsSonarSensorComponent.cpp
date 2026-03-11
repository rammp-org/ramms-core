// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsSonarSensorComponent.h"
#include "RammsSensorUtils.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

URammsSonarSensorComponent::URammsSonarSensorComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void URammsSonarSensorComponent::BeginPlay()
{
	Super::BeginPlay();
	TimeSinceLastUpdate = 0.0f;
}

void URammsSonarSensorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
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
	OnSonarDataUpdated.Broadcast(CurrentData);

	if (bEnableDebugLogging && GFrameCounter % 60 == 0)
	{
		UE_LOG(LogTemp, Log, TEXT("[Sonar] Hit=%d Distance=%.1f cm"),
			CurrentData.bHit ? 1 : 0, CurrentData.Distance);
	}
}

FSonarSensorData URammsSonarSensorComponent::MeasureNow()
{
	CurrentData = PerformMeasurement();
	OnSonarDataUpdated.Broadcast(CurrentData);
	return CurrentData;
}

FSonarSensorData URammsSonarSensorComponent::PerformMeasurement() const
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
			// Distribute rays in a cone pattern
			// Use a spiral pattern for even distribution
			float t = (float)i / (float)(RayCount - 1);			  // 0 to 1
			float ConeAngle = BeamHalfAngle * FMath::Sqrt(t);	  // Sqrt for area-uniform distribution
			float AzimuthAngle = t * 2.4f * PI * (float)RayCount; // Golden angle spiral

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
