// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsImuProvider.h"
#include "RammsImuTypes.h"
#include "Subsystems/WorldSubsystem.h"
#include "RammsPhysicsImuSubsystem.generated.h"

class AActor;
class USceneComponent;

/**
 * Game-rate transform differencing of one sensor frame: the Kinematic source. Velocities from the first difference
 * of the pose, accelerations from the second; the first sample has a pose only, the second velocities too.
 */
struct RAMMSCORE_API FRammsImuDifferencer
{
	/** Take the frame Sensor (cm, world, unscaled) at Time (s). A second call at the same time returns the last sample. */
	const FRammsImuSample& Step(double Time, const FTransform& Sensor, const FVector& Gravity);

	void Reset() { *this = FRammsImuDifferencer(); }

	const FRammsImuSample& GetLatest() const { return Latest; }

private:
	bool			bHasPose = false;
	bool			bHasVelocity = false;
	double			Time = 0.0;
	FVector			Location = FVector::ZeroVector;
	FQuat			Rotation = FQuat::Identity;
	FVector			Velocity = FVector::ZeroVector;
	FVector			AngularVelocityRad = FVector::ZeroVector;
	FRammsImuSample Latest;
};

/**
 * Inertial data for any point on any body, from the physics engine that simulates it, sampled at the physics rate:
 * the IMU component's source, and the sensing bridge's for bodies that carry no IMU component.
 *
 * Track a point (a component, optionally a bone, and a pose in its frame) and read its latest sample. Auto takes the
 * engine that simulates the body the point rides on, walking up its attach chain: MuJoCo for a URLab body (or
 * anything attached below one; a URLab robot that Chaos simulates resolves to Chaos), Chaos for a primitive or a
 * skeletal bone that simulates physics, the nearest of those winning; else Kinematic (game-rate transform
 * differencing). Asking for an engine by name that does not simulate the body falls back to Auto. A point that fell
 * back to Kinematic is resolved again every second, so a robot whose physics comes up later (MuJoCo compiles, a
 * Chaos articulation builds) gets its engine's data then.
 *
 * Engines plug in through RammsImuProviders (RammsCore's Chaos provider, RammsMujocoSupport's MuJoCo one).
 */
UCLASS()
class RAMMSCORE_API URammsPhysicsImuSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	/** The subsystem of WorldContext's world, or null. */
	static URammsPhysicsImuSubsystem* Get(const UObject* WorldContext);

	/** Track the point LocalOffset (cm) in Component's frame (or its bone / socket Bone's). A handle > 0, or 0 when
	 *  Component is null. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensors|IMU")
	int32 Track(USceneComponent* Component, FVector LocalOffset, ERammsImuSource Preferred = ERammsImuSource::Auto, FName Bone = NAME_None);

	/** Track the sensor frame LocalPose in (Component, Bone)'s frame. IdleTimeoutSeconds > 0: dropped once not read
	 *  (GetSample) for that long. A handle > 0, or 0 when Component is null. */
	int32 TrackPose(USceneComponent* Component, FName Bone, const FTransform& LocalPose, ERammsImuSource Preferred,
		float IdleTimeoutSeconds = 0.0f);

	/** The point's latest sample (world space, at the point). False before its engine has sampled it. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensors|IMU")
	bool GetSample(int32 Handle, FRammsImuSample& OutSample);

	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensors|IMU")
	void Untrack(int32 Handle);

	UFUNCTION(BlueprintPure, Category = "Ramms|Sensors|IMU")
	bool IsTracked(int32 Handle) const;

	/** The engine serving a tracked point (Kinematic when untracked). */
	UFUNCTION(BlueprintPure, Category = "Ramms|Sensors|IMU")
	ERammsImuSource GetTrackedSource(int32 Handle) const;

	/** The body a tracked point rides on, for reports (a MuJoCo body, bone or component name). */
	FString GetTrackedBodyName(int32 Handle) const;

	/** The body and engine Preferred picks for a sensor frame, without tracking it (see the class comment). Always
	 *  finds one: Kinematic on Component itself at worst. */
	bool ResolveBody(USceneComponent& Component, FName Bone, const FTransform& LocalPose, ERammsImuSource Preferred,
		FRammsImuBodyRef& OutRef);

	/** The simulated bodies of Actor (and of actors attached to it), main body first: MuJoCo's, then Chaos'. */
	void ListBodies(AActor& Actor, TArray<FRammsImuBodyRef>& OutBodies);

	/** "auto", "chaos", "mujoco", "kinematic". */
	static FString SourceName(ERammsImuSource Source);

	/** The inverse of SourceName (case-insensitive). False for anything else. */
	static bool ParseSource(const FString& Text, ERammsImuSource& OutSource);

	// UTickableWorldSubsystem
	virtual void	Deinitialize() override;
	virtual void	Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool	IsTickable() const override;
	virtual bool	IsTickableInEditor() const override { return true; }

private:
	struct FTrackedPoint
	{
		TWeakObjectPtr<USceneComponent> Component;
		FName							Bone;
		FTransform						LocalPose = FTransform::Identity;
		ERammsImuSource					Preferred = ERammsImuSource::Auto;
		FRammsImuBodyRef				Ref;
		int32							ProviderHandle = INDEX_NONE;
		float							IdleTimeoutSeconds = 0.0f;
		double							LastRead = 0.0;
		double							NextResolve = 0.0;
	};

	/** The provider for an engine, made on first use (null when no engine registered one). */
	IRammsImuProvider* GetProvider(ERammsImuSource Source);

	/** Resolve the point and track it with the provider that serves it (Kinematic when that one refuses). */
	void Attach(FTrackedPoint& Point);
	void Detach(FTrackedPoint& Point);

	double Now() const;

	TMap<int32, FTrackedPoint>							 Points;
	TMap<ERammsImuSource, TSharedPtr<IRammsImuProvider>> Providers;
	int32												 NextHandle = 1;
};
