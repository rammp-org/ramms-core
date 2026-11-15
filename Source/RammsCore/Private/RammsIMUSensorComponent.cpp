// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsIMUSensorComponent.h"
#include "RammsPhysicsImuSubsystem.h"
#include "RammsSensorUtils.h"
#include "Components/PrimitiveComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

URammsIMUSensorComponent::URammsIMUSensorComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void URammsIMUSensorComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (PhysicsHandle != 0)
	{
		if (URammsPhysicsImuSubsystem* Imu = URammsPhysicsImuSubsystem::Get(this))
		{
			Imu->Untrack(PhysicsHandle);
		}
		PhysicsHandle = 0;
	}
	Super::EndPlay(EndPlayReason);
}

bool URammsIMUSensorComponent::SamplePhysics(FRammsImuSample& OutSample)
{
	// bUsePhysicsVelocity off has always meant differencing the transform; under Auto it still does.
	const ERammsImuSource	   Wanted = Source == ERammsImuSource::Auto && !bUsePhysicsVelocity ? ERammsImuSource::Kinematic : Source;
	URammsPhysicsImuSubsystem* Imu = URammsPhysicsImuSubsystem::Get(this);
	if (!Imu)
	{
		return false;
	}
	if (PhysicsHandle != 0 && (TrackedSource != Wanted || !Imu->IsTracked(PhysicsHandle)))
	{
		Imu->Untrack(PhysicsHandle);
		PhysicsHandle = 0;
	}
	if (Wanted == ERammsImuSource::Kinematic)
	{
		return false; // this component's own differencing, below
	}
	if (PhysicsHandle == 0)
	{
		// Tracked from the first measurement rather than BeginPlay: by then every actor has begun play, so a URLab
		// scene or a Chaos articulation this sensor rides on exists. One that comes up later is picked up by the
		// subsystem, which retries a Kinematic fallback every second.
		PhysicsHandle = Imu->TrackPose(this, NAME_None, FTransform::Identity, Wanted);
		TrackedSource = Wanted;
	}
	return PhysicsHandle != 0 && Imu->GetTrackedSource(PhysicsHandle) != ERammsImuSource::Kinematic
		&& Imu->GetSample(PhysicsHandle, OutSample) && OutSample.bValid;
}

void URammsIMUSensorComponent::BeginPlay()
{
	Super::BeginPlay();

	PreviousTransform = GetComponentTransform();
	PreviousRotation = PreviousTransform.GetRotation();
	PreviousVelocity = FVector::ZeroVector;
	bPreviousStateValid = false;
	bSmoothedStateValid = false;
	SmoothedAccel = FVector::ZeroVector;
	SmoothedGyro = FVector::ZeroVector;
	TimeSinceLastUpdate = 0.0f;

	// Walk up the attachment hierarchy to find the nearest physics-simulating primitive
	CachedPhysicsPrimitive = nullptr;
	if (bUsePhysicsVelocity)
	{
		for (USceneComponent* Comp = GetAttachParent(); Comp; Comp = Comp->GetAttachParent())
		{
			UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(Comp);
			if (Prim && Prim->IsSimulatingPhysics())
			{
				CachedPhysicsPrimitive = Prim;
				break;
			}
		}
		// Also check the owner's root component
		if (!CachedPhysicsPrimitive.IsValid() && GetOwner())
		{
			UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent());
			if (RootPrim && RootPrim->IsSimulatingPhysics())
			{
				CachedPhysicsPrimitive = RootPrim;
			}
		}
	}
}

void URammsIMUSensorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (DeltaTime <= 0.0f)
		return;

	UWorld* World = GetWorld();
	if (!World)
		return;

	// Rate limiting — keep only the remainder to avoid bursty catch-up after hitches
	if (UpdateRateHz > 0.0f)
	{
		TimeSinceLastUpdate += DeltaTime;
		const float UpdateInterval = 1.0f / UpdateRateHz;
		if (TimeSinceLastUpdate < UpdateInterval)
			return;
		TimeSinceLastUpdate = FMath::Fmod(TimeSinceLastUpdate, UpdateInterval);
	}

	const FTransform CurrentTransform = GetComponentTransform();
	const FQuat		 CurrentRotation = CurrentTransform.GetRotation();
	const FVector	 CurrentLocation = CurrentTransform.GetLocation();
	const float		 GameTime = World->GetTimeSeconds();

	// --- The physics engine's sample of this sensor's frame, when one simulates the body it rides on ---
	// (Chaos / MuJoCo, at the physics rate, at this sensor's point; see Source)
	FRammsImuSample PhysicsSample;
	const bool		bPhysicsSample = SamplePhysics(PhysicsSample);

	// --- Derive world-space velocity ---
	FVector CurrentVelocity = FVector::ZeroVector;
	bool	bHasPhysicsVelocity = false;

	if (bPhysicsSample)
	{
		// The point's own velocity from the engine (the lever arm from the body's origin applied)
		CurrentVelocity = PhysicsSample.LinearVelocity;
	}
	else if (bUsePhysicsVelocity && CachedPhysicsPrimitive.IsValid())
	{
		// Use physics-engine velocity (single differentiation inside the solver, much smoother)
		CurrentVelocity = CachedPhysicsPrimitive->GetPhysicsLinearVelocity();
		bHasPhysicsVelocity = true;
	}
	else if (bPreviousStateValid)
	{
		// Fallback: finite-difference from position
		CurrentVelocity = (CurrentLocation - PreviousTransform.GetLocation()) / DeltaTime;
	}

	FIMUSensorData Data;
	Data.Timestamp = GameTime;
	Data.Source = bPhysicsSample ? PhysicsSample.Source : ERammsImuSource::Kinematic;

	// --- Accelerometer ---
	FVector WorldAccel = FVector::ZeroVector;
	FVector WorldGravity(0.0, 0.0, World->GetGravityZ());
	if (bPhysicsSample)
	{
		// The engine's acceleration of the point over its last step (coordinate acceleration: what an accelerometer
		// reads plus the gravity that engine applies); zero until it has two steps
		WorldGravity = PhysicsSample.Gravity;
		if (PhysicsSample.bHasAcceleration)
		{
			WorldAccel = PhysicsSample.ProperAcceleration + PhysicsSample.Gravity;
		}
	}
	else if (bHasPhysicsVelocity && bPreviousStateValid)
	{
		// Single differentiation of physics velocity
		WorldAccel = (CurrentVelocity - PreviousVelocity) / DeltaTime;
	}
	else if (!bHasPhysicsVelocity && bPreviousStateValid)
	{
		// Double differentiation fallback
		WorldAccel = (CurrentVelocity - PreviousVelocity) / DeltaTime;
	}

	// The sensor frame: the physics sample's (the state at the step it was taken), else the component's
	const FQuat SensorRotation = bPhysicsSample ? PhysicsSample.Orientation : CurrentRotation;

	// Apply dead-band before adding gravity (so the threshold is against dynamic acceleration only)
	const FQuat InvRotation = SensorRotation.Inverse();
	FVector		LocalDynamicAccel = InvRotation.RotateVector(WorldAccel);
	if (AccelDeadBand > 0.0f && LocalDynamicAccel.Size() < AccelDeadBand)
	{
		LocalDynamicAccel = FVector::ZeroVector;
	}

	// Add gravity in sensor-local frame
	if (bIncludeGravity)
	{
		FVector WorldGravityAccel = -WorldGravity; // sensor "feels" upward when stationary
		LocalDynamicAccel += InvRotation.RotateVector(WorldGravityAccel);
	}

	// EMA smoothing
	if (AccelSmoothingFactor > 0.0f && bSmoothedStateValid)
	{
		LocalDynamicAccel = FMath::Lerp(LocalDynamicAccel, SmoothedAccel, AccelSmoothingFactor);
	}
	SmoothedAccel = LocalDynamicAccel;

	Data.LinearAcceleration = LocalDynamicAccel;
	Data.LinearAcceleration += AccelBias;
	Data.LinearAcceleration = RammsSensorUtils::AddVectorNoise(Data.LinearAcceleration, AccelNoiseStdDev);

	// --- Gyroscope ---
	FVector LocalAngVel = FVector::ZeroVector;
	if (bPhysicsSample)
	{
		// The body's angular velocity from the engine (degrees/s in world space)
		LocalAngVel = InvRotation.RotateVector(PhysicsSample.AngularVelocity);
	}
	else if (bUsePhysicsVelocity && CachedPhysicsPrimitive.IsValid())
	{
		// Get angular velocity directly from physics (degrees/s in world space)
		FVector WorldAngVel = CachedPhysicsPrimitive->GetPhysicsAngularVelocityInDegrees();
		LocalAngVel = InvRotation.RotateVector(WorldAngVel);
	}
	else if (bPreviousStateValid)
	{
		// Derive from rotation delta
		FQuat DeltaQ = PreviousRotation.Inverse() * CurrentRotation;
		DeltaQ.Normalize();

		FVector Axis;
		float	AngleRad;
		DeltaQ.ToAxisAndAngle(Axis, AngleRad);

		if (AngleRad > PI)
			AngleRad -= 2.0f * PI;

		FVector WorldAngVel = Axis * FMath::RadiansToDegrees(AngleRad) / DeltaTime;
		LocalAngVel = InvRotation.RotateVector(WorldAngVel);
	}

	// Gyro dead-band
	if (GyroDeadBand > 0.0f && LocalAngVel.Size() < GyroDeadBand)
	{
		LocalAngVel = FVector::ZeroVector;
	}

	// EMA smoothing
	if (GyroSmoothingFactor > 0.0f && bSmoothedStateValid)
	{
		LocalAngVel = FMath::Lerp(LocalAngVel, SmoothedGyro, GyroSmoothingFactor);
	}
	SmoothedGyro = LocalAngVel;

	Data.AngularVelocity = LocalAngVel;
	Data.AngularVelocity += GyroBias;
	Data.AngularVelocity = RammsSensorUtils::AddVectorNoise(Data.AngularVelocity, GyroNoiseStdDev);

	// --- Orientation ---
	Data.Orientation = SensorRotation;
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
	bSmoothedStateValid = true;

	// Broadcast
	OnIMUDataUpdated.Broadcast(CurrentData);

	// Debug
	if (bEnableDebugDisplay)
	{
		const FVector Loc = CurrentTransform.GetLocation();
		DrawDebugCoordinateSystem(World, Loc, CurrentRotation.Rotator(), 15.0f, false, -1.0f, 0, 1.0f);

		// Draw acceleration vector (scaled down for visibility)
		FVector WorldAccelVis = SensorRotation.RotateVector(Data.LinearAcceleration) * 0.05f;
		DrawDebugDirectionalArrow(World, Loc, Loc + WorldAccelVis, 3.0f, FColor::Red, false, -1.0f, 0, 1.5f);
	}

	if (bEnableDebugLogging && GFrameCounter % 60 == 0)
	{
		UE_LOG(LogTemp, Log, TEXT("[IMU] Accel=(%.1f, %.1f, %.1f) Gyro=(%.1f, %.1f, %.1f) Orient=(%.3f, %.3f, %.3f, %.3f)"),
			Data.LinearAcceleration.X, Data.LinearAcceleration.Y, Data.LinearAcceleration.Z,
			Data.AngularVelocity.X, Data.AngularVelocity.Y, Data.AngularVelocity.Z,
			Data.Orientation.X, Data.Orientation.Y, Data.Orientation.Z, Data.Orientation.W);
	}
}
