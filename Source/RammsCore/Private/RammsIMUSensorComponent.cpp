// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsIMUSensorComponent.h"
#include "RammsImuMath.h"
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

ERammsImuSource URammsIMUSensorComponent::GetRequestedSource() const
{
	// bUsePhysicsVelocity off has always meant differencing the transform; under Auto it still does.
	return Source == ERammsImuSource::Auto && !bUsePhysicsVelocity ? ERammsImuSource::Kinematic : Source;
}

bool URammsIMUSensorComponent::SamplePhysics(FRammsImuSample& OutSample)
{
	const ERammsImuSource	   Wanted = GetRequestedSource();
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
		// scene or a Chaos articulation this sensor rides on exists. One that comes up later (or a body that changes
		// engine) is picked up by the subsystem, which resolves every point again each second.
		const UWorld* World = GetWorld();
		const double  Now = World ? World->GetTimeSeconds() : 0.0;
		if (Now < NextTrackAttempt)
		{
			return false;
		}
		PhysicsHandle = Imu->TrackPose(this, NAME_None, FTransform::Identity, Wanted);
		TrackedSource = Wanted;
		if (PhysicsHandle == 0)
		{
			NextTrackAttempt = Now + 1.0; // the world tracks as many points as it may: ask again later
			return false;
		}
	}
	return Imu->GetTrackedSource(PhysicsHandle) != ERammsImuSource::Kinematic && Imu->GetSample(PhysicsHandle, OutSample) && OutSample.bValid;
}

void URammsIMUSensorComponent::BeginPlay()
{
	Super::BeginPlay();

	PreviousTransform = GetComponentTransform();
	PreviousRotation = PreviousTransform.GetRotation();
	PreviousVelocity = FVector::ZeroVector;
	bPreviousStateValid = false;
	bPreviousVelocityValid = false;
	bPreviousPhysicsValid = false;
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
	const double	 GameTime = World->GetTimeSeconds();
	// The time since the last measurement: several ticks when UpdateRateHz is below the frame rate.
	const double Elapsed = bPreviousStateValid && GameTime > LastMeasurementTime ? GameTime - LastMeasurementTime : static_cast<double>(DeltaTime);

	// --- The physics engine's sample of this sensor's frame, when one simulates the body it rides on ---
	// (Chaos / MuJoCo, at the physics rate, at this sensor's point; see Source)
	FRammsImuSample PhysicsSample;
	const bool		bPhysicsSample = SamplePhysics(PhysicsSample);
	if (bPhysicsSample != bLastMeasurementPhysics)
	{
		// A change of path: neither differences against the other's state (an engine's point velocity against a
		// centre-of-mass or transform-differenced one, or the other way round).
		bPreviousStateValid = false;
		bPreviousVelocityValid = false;
		bPreviousPhysicsValid = false;
	}
	bLastMeasurementPhysics = bPhysicsSample;

	FIMUSensorData Data;
	Data.Timestamp = static_cast<float>(GameTime);
	Data.Source = bPhysicsSample ? PhysicsSample.Source : ERammsImuSource::Kinematic;
	Data.SimTimeSeconds = bPhysicsSample ? PhysicsSample.SimTimeSeconds : 0.0;

	// World-space motion over the measurement: the coordinate acceleration (gravity not in it), the angular velocity
	// (deg/s), and the sensor frame they are read in.
	FVector CurrentVelocity = FVector::ZeroVector;
	bool	bVelocityKnown = false;
	FVector WorldAccel = FVector::ZeroVector;
	FVector WorldGravity(0.0, 0.0, World->GetGravityZ());
	FVector WorldAngVel = FVector::ZeroVector;
	FQuat	FrameRotation = CurrentRotation;

	if (bPhysicsSample)
	{
		// Over the interval since the last measurement, on the engine's clock: the point's velocity change and the
		// body's turn. The engine's samples carry the lever arm, so this is the sensor point's own motion.
		WorldGravity = PhysicsSample.Gravity;
		const bool bSameTrack =
			bPreviousPhysicsValid && PreviousPhysicsHandle == PhysicsHandle && PreviousPhysicsSample.Source == PhysicsSample.Source;
		const double Span = PhysicsSample.SimTimeSeconds - PreviousPhysicsSample.SimTimeSeconds;
		if (bSameTrack && Span > 0.0)
		{
			WorldAccel = (PhysicsSample.LinearVelocity - PreviousPhysicsSample.LinearVelocity) / Span;
			WorldAngVel = RammsImuMath::RadiansToDegrees(
				RammsImuMath::AngularVelocityBetween(PreviousPhysicsSample.Orientation, PhysicsSample.Orientation, Span));
			FrameRotation = FQuat::Slerp(PreviousPhysicsSample.Orientation, PhysicsSample.Orientation, 0.5);
		}
		else if (bSameTrack && Span == 0.0)
		{
			// No new engine step since the last measurement (the engine is paused, or slower than UpdateRateHz): the
			// same reading again.
			WorldAccel = LastIntervalAccel;
			WorldAngVel = LastIntervalAngularVelocity;
			FrameRotation = LastIntervalFrame;
		}
		else
		{
			// The first sample of this point (or the engine's clock went back): that step's own values.
			WorldAccel = PhysicsSample.bHasAcceleration ? PhysicsSample.ProperAcceleration + PhysicsSample.Gravity : FVector::ZeroVector;
			WorldAngVel = PhysicsSample.AngularVelocity;
			FrameRotation = PhysicsSample.Orientation;
		}
		LastIntervalAccel = WorldAccel;
		LastIntervalAngularVelocity = WorldAngVel;
		LastIntervalFrame = FrameRotation;
		PreviousPhysicsSample = PhysicsSample;
		PreviousPhysicsHandle = PhysicsHandle;
		bPreviousPhysicsValid = true;
		CurrentVelocity = PhysicsSample.LinearVelocity;
		bVelocityKnown = true;
	}
	else
	{
		// --- Kinematic: this component's own measurement, at the game rate ---
		const bool bPrimitiveVelocity = bUsePhysicsVelocity && CachedPhysicsPrimitive.IsValid();
		if (bPrimitiveVelocity)
		{
			// The nearest simulating primitive's velocity (its centre of mass): a single differentiation
			CurrentVelocity = CachedPhysicsPrimitive->GetPhysicsLinearVelocity();
			bVelocityKnown = true;
			if (bPreviousVelocityValid)
			{
				WorldAccel = (CurrentVelocity - PreviousVelocity) / Elapsed;
			}
		}
		else if (bPreviousStateValid)
		{
			// Finite differences of the position: each velocity belongs to the middle of its interval, so two of them
			// are half of each interval apart
			CurrentVelocity = (CurrentLocation - PreviousTransform.GetLocation()) / Elapsed;
			bVelocityKnown = true;
			if (bPreviousVelocityValid)
			{
				const double Spacing = PreviousElapsed > 0.0 ? 0.5 * (Elapsed + PreviousElapsed) : Elapsed;
				WorldAccel = (CurrentVelocity - PreviousVelocity) / Spacing;
			}
		}

		if (bPrimitiveVelocity)
		{
			// Angular velocity directly from physics (degrees/s in world space)
			WorldAngVel = CachedPhysicsPrimitive->GetPhysicsAngularVelocityInDegrees();
		}
		else if (bPreviousStateValid)
		{
			// The world-space turn from the previous rotation to this one (Current = Turn * Previous)
			WorldAngVel = RammsImuMath::RadiansToDegrees(RammsImuMath::AngularVelocityBetween(PreviousRotation, CurrentRotation, Elapsed));
		}
	}

	// Apply dead-band before adding gravity (so the threshold is against dynamic acceleration only)
	const FQuat InvRotation = FrameRotation.Inverse();
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
	FVector LocalAngVel = InvRotation.RotateVector(WorldAngVel);

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
	// The sensor frame: the physics sample's (the state at the step it was taken), else the component's
	const FQuat SensorRotation = bPhysicsSample ? PhysicsSample.Orientation : CurrentRotation;
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
	bPreviousVelocityValid = bVelocityKnown;
	PreviousElapsed = bPreviousStateValid ? Elapsed : 0.0;
	PreviousTransform = CurrentTransform;
	PreviousRotation = CurrentRotation;
	LastMeasurementTime = GameTime;
	bPreviousStateValid = true;
	bSmoothedStateValid = true;

	// Broadcast
	OnIMUDataUpdated.Broadcast(CurrentData);

	// Debug
	if (bEnableDebugDisplay)
	{
		const FVector Loc = CurrentTransform.GetLocation();
		DrawDebugCoordinateSystem(World, Loc, SensorRotation.Rotator(), 15.0f, false, -1.0f, 0, 1.0f);

		// Draw acceleration vector (scaled down for visibility), from the frame it was read in
		FVector WorldAccelVis = FrameRotation.RotateVector(Data.LinearAcceleration) * 0.05f;
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
