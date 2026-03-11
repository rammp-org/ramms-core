// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsIMUSensorComponent.h"
#include "RammsSensorUtils.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

URammsIMUSensorComponent::URammsIMUSensorComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void URammsIMUSensorComponent::BeginPlay()
{
	Super::BeginPlay();

	PreviousTransform = GetComponentTransform();
	PreviousRotation = PreviousTransform.GetRotation();
	PreviousVelocity = FVector::ZeroVector;
	bPreviousStateValid = false;
	TimeSinceLastUpdate = 0.0f;
}

void URammsIMUSensorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (DeltaTime <= 0.0f)
		return;

	// Rate limiting
	if (UpdateRateHz > 0.0f)
	{
		TimeSinceLastUpdate += DeltaTime;
		const float UpdateInterval = 1.0f / UpdateRateHz;
		if (TimeSinceLastUpdate < UpdateInterval)
			return;
		TimeSinceLastUpdate -= UpdateInterval;
	}

	const FTransform CurrentTransform = GetComponentTransform();
	const FQuat		 CurrentRotation = CurrentTransform.GetRotation();
	const FVector	 CurrentLocation = CurrentTransform.GetLocation();
	const float		 GameTime = GetWorld()->GetTimeSeconds();

	// Derive velocity from position delta (works for any attachment — bone, socket, etc.)
	FVector CurrentVelocity = FVector::ZeroVector;
	if (bPreviousStateValid)
	{
		CurrentVelocity = (CurrentLocation - PreviousTransform.GetLocation()) / DeltaTime;
	}

	FIMUSensorData Data;
	Data.Timestamp = GameTime;

	// --- Accelerometer ---
	// Linear acceleration = dV/dt in world space, then transform to sensor-local
	FVector WorldAccel = FVector::ZeroVector;
	if (bPreviousStateValid)
	{
		WorldAccel = (CurrentVelocity - PreviousVelocity) / DeltaTime;
	}

	if (bIncludeGravity)
	{
		// Add gravity (UE default is -Z at 980 cm/s²)
		float GravityZ = GetWorld()->GetGravityZ();	 // Negative value
		WorldAccel -= FVector(0.0f, 0.0f, GravityZ); // Subtract because sensor "feels" upward when stationary
	}

	// Transform to sensor-local frame
	const FQuat InvRotation = CurrentRotation.Inverse();
	Data.LinearAcceleration = InvRotation.RotateVector(WorldAccel);
	Data.LinearAcceleration += AccelBias;
	Data.LinearAcceleration = RammsSensorUtils::AddVectorNoise(Data.LinearAcceleration, AccelNoiseStdDev);

	// --- Gyroscope ---
	// Angular velocity from rotation delta
	if (bPreviousStateValid)
	{
		// Compute rotation delta: Q_delta = Q_prev^-1 * Q_current
		FQuat DeltaQ = PreviousRotation.Inverse() * CurrentRotation;
		DeltaQ.Normalize();

		// Convert to axis-angle
		FVector Axis;
		float	AngleRad;
		DeltaQ.ToAxisAndAngle(Axis, AngleRad);

		// Wrap angle to [-PI, PI]
		if (AngleRad > PI)
			AngleRad -= 2.0f * PI;

		// Angular velocity in world space (degrees/s)
		FVector WorldAngVel = Axis * FMath::RadiansToDegrees(AngleRad) / DeltaTime;

		// Transform to sensor-local frame
		Data.AngularVelocity = InvRotation.RotateVector(WorldAngVel);
	}
	Data.AngularVelocity += GyroBias;
	Data.AngularVelocity = RammsSensorUtils::AddVectorNoise(Data.AngularVelocity, GyroNoiseStdDev);

	// --- Orientation ---
	Data.Orientation = CurrentRotation;
	if (OrientationNoiseStdDev > 0.0f)
	{
		// Apply small random rotation noise
		FVector NoiseAxis(RammsSensorUtils::GaussianNoise(1.0f), RammsSensorUtils::GaussianNoise(1.0f), RammsSensorUtils::GaussianNoise(1.0f));
		if (!NoiseAxis.IsNearlyZero())
		{
			NoiseAxis.Normalize();
			float NoiseAngle = RammsSensorUtils::GaussianNoise(OrientationNoiseStdDev);
			FQuat NoiseQ(NoiseAxis, FMath::DegreesToRadians(NoiseAngle));
			Data.Orientation = Data.Orientation * NoiseQ;
			Data.Orientation.Normalize();
		}
	}

	// Store state
	CurrentData = Data;
	PreviousVelocity = CurrentVelocity;
	PreviousTransform = CurrentTransform;
	PreviousRotation = CurrentRotation;
	bPreviousStateValid = true;

	// Broadcast
	OnIMUDataUpdated.Broadcast(CurrentData);

	// Debug
	if (bEnableDebugDisplay)
	{
		const FVector Loc = CurrentTransform.GetLocation();
		DrawDebugCoordinateSystem(GetWorld(), Loc, CurrentRotation.Rotator(), 15.0f, false, -1.0f, 0, 1.0f);

		// Draw acceleration vector (scaled down for visibility)
		FVector WorldAccelVis = CurrentRotation.RotateVector(Data.LinearAcceleration) * 0.05f;
		DrawDebugDirectionalArrow(GetWorld(), Loc, Loc + WorldAccelVis, 3.0f, FColor::Red, false, -1.0f, 0, 1.5f);
	}

	if (bEnableDebugLogging && GFrameCounter % 60 == 0)
	{
		UE_LOG(LogTemp, Log, TEXT("[IMU] Accel=(%.1f, %.1f, %.1f) Gyro=(%.1f, %.1f, %.1f) Orient=(%.3f, %.3f, %.3f, %.3f)"),
			Data.LinearAcceleration.X, Data.LinearAcceleration.Y, Data.LinearAcceleration.Z,
			Data.AngularVelocity.X, Data.AngularVelocity.Y, Data.AngularVelocity.Z,
			Data.Orientation.X, Data.Orientation.Y, Data.Orientation.Z, Data.Orientation.W);
	}
}
