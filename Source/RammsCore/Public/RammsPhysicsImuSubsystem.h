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
 * of the pose, accelerations from the second (over the spacing of the two velocities, so an uneven frame rate does
 * not bias them); the first sample has a pose only, the second velocities too.
 */
struct RAMMSCORE_API FRammsImuDifferencer
{
	/** Take the frame Sensor (cm, world, unscaled) at InTime (s). A second call at the same time returns the last sample. */
	const FRammsImuSample& Step(double InTime, const FTransform& Sensor, const FVector& Gravity);

	void Reset() { *this = FRammsImuDifferencer(); }

	const FRammsImuSample& GetLatest() const { return Latest; }

private:
	bool			bHasPose = false;
	bool			bHasVelocity = false;
	double			Time = 0.0;
	double			LastDt = 0.0; // the interval the last velocity was taken over
	FVector			Location = FVector::ZeroVector;
	FQuat			Rotation = FQuat::Identity;
	FVector			Velocity = FVector::ZeroVector;
	FVector			AngularVelocityRad = FVector::ZeroVector;
	FRammsImuSample Latest;
};

/** What URammsPhysicsImuSubsystem::ReadHistory returns: a tracked point's kept physics-step samples. */
struct RAMMSCORE_API FRammsImuHistoryRead
{
	/** Oldest first; world space, at the point (see FRammsImuSample). */
	TArray<FRammsImuSample> Samples;

	/** The sequence number of Samples[0] (when there is one). */
	int64 FirstSeq = 0;

	/** The sequence number the next sample will get: pass it back as Since. */
	int64 Next = 0;

	/** Samples lost between Since and the oldest still kept (the ring holds HistoryCapacity). */
	int64 Dropped = 0;
};

/**
 * Inertial data for any point on any body, from the physics engine that simulates it, sampled at the physics rate:
 * the IMU component's source, and the sensing bridge's for bodies that carry no IMU component.
 *
 * Track a point (a component, optionally a bone, and a pose in its frame) and read its latest sample, or (after
 * EnableHistory) every physics step's since a cursor. Auto takes the engine that simulates the body the point rides
 * on, walking up its attach chain: MuJoCo for a URLab body (or anything attached below one; a URLab robot that Chaos
 * simulates resolves to Chaos), Chaos for a primitive or a skeletal bone that simulates physics, the nearest of those
 * winning; else Kinematic (game-rate transform differencing). Asking for an engine by name that does not simulate the
 * body falls back to Auto.
 *
 * Every point is resolved again every second, and at once when its provider reports its body gone: a robot whose
 * physics comes up later (MuJoCo compiles, a Chaos articulation builds), is handed to another engine, or loses the
 * body it rode on moves to whatever serves it now. Its sample sequence goes on across such a move.
 *
 * At most MaxTrackedPoints points per world (IMU components and sensing-bridge reads together), of which the sensing
 * bridge (Remote Control) may hold MaxBridgePoints (TrackBridgePose): a client that leaves points behind cannot take
 * the robots' own IMU components' points. A refusal is logged once, not on every retry, until a point is tracked again.
 *
 * History (EnableHistory / ReadHistory) keeps complete samples only: a Kinematic point's first two frames (a pose, then
 * a velocity) and an engine's first step on a body carry no acceleration and are left out of it.
 *
 * Engines plug in through RammsImuProviders (RammsCore's Chaos provider, RammsMujocoSupport's MuJoCo one).
 */
UCLASS()
class RAMMSCORE_API URammsPhysicsImuSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	/** Points one world tracks at most. */
	static constexpr int32 MaxTrackedPoints = 64;

	/** Of those, the points the sensing bridge (Remote Control readers and IMU streams) may hold at once. */
	static constexpr int32 MaxBridgePoints = 32;

	/** Samples a point with history keeps: over 2 s of MuJoCo's 500 Hz. */
	static constexpr int32 HistoryCapacity = 1024;

	/** The subsystem of WorldContext's world, or null. */
	static URammsPhysicsImuSubsystem* Get(const UObject* WorldContext);

	/** Track the point LocalOffset (cm) in Component's frame (or its bone / socket Bone's). A handle > 0, or 0 when
	 *  Component is null or the world tracks MaxTrackedPoints already. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensors|IMU")
	int32 Track(USceneComponent* Component, FVector LocalOffset, ERammsImuSource Preferred = ERammsImuSource::Auto, FName Bone = NAME_None);

	/** Track the sensor frame LocalPose in (Component, Bone)'s frame. IdleTimeoutSeconds > 0: dropped once not read
	 *  (GetSample / ReadHistory) for that long. A handle > 0, or 0 when Component is null or the world tracks
	 *  MaxTrackedPoints already. */
	int32 TrackPose(USceneComponent* Component, FName Bone, const FTransform& LocalPose, ERammsImuSource Preferred,
		float IdleTimeoutSeconds = 0.0f);

	/** TrackPose for the sensing bridge: the point also counts against MaxBridgePoints. A handle > 0, or 0 and why in
	 *  OutError. */
	int32 TrackBridgePose(USceneComponent* Component, FName Bone, const FTransform& LocalPose, ERammsImuSource Preferred,
		float IdleTimeoutSeconds, FString& OutError);

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

	/** The component carrying the body a tracked point rides on (a primitive, a skeletal mesh, a URLab body, or the
	 *  tracked component itself for Kinematic); null when untracked. */
	USceneComponent* GetTrackedBody(int32 Handle) const;

	/** How many points this world tracks. */
	int32 GetTrackedCount() const { return Points.Num(); }

	/** How many of them the sensing bridge holds (TrackBridgePose). */
	int32 GetBridgePointCount() const;

	/** From now on keep the point's samples, one per physics step (HistoryCapacity of them). Allocates once. */
	void EnableHistory(int32 Handle);

	/**
	 * The point's kept samples from sequence number Since on, at most Max, oldest first. Since < 0 starts now: no
	 * samples, only the cursor. A cursor ahead of the point's (one from a point tracked again since) also starts now.
	 * Enables history if it was off. False for an unknown handle.
	 */
	bool ReadHistory(int32 Handle, int64 Since, int32 Max, FRammsImuHistoryRead& Out);

	/** The body and engine Preferred picks for a sensor frame, without tracking it (see the class comment). Always
	 *  finds one: Kinematic on Component itself at worst. */
	bool ResolveBody(USceneComponent& Component, FName Bone, const FTransform& LocalPose, ERammsImuSource Preferred,
		FRammsImuBodyRef& OutRef);

	/** The simulated bodies of Actor (and of actors attached to it), main body first: MuJoCo's, then Chaos' (a
	 *  URLab body that Chaos simulates is listed once, by its URLab name). */
	void ListBodies(AActor& Actor, TArray<FRammsImuBodyRef>& OutBodies);

	/** Drop points whose component is gone or that sat idle, and move points whose body changed engine or is gone
	 *  (what Tick does; bForceResolve resolves every point again now rather than on its one-second schedule). */
	void RefreshPoints(bool bForceResolve);

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
		double							NextInvalidRetry = 0.0;
		/** Keep a ring of every step's sample (EnableHistory), on whichever provider serves the point. */
		bool bHistory = false;
		/** The point's sequence numbers are the provider's plus this: they go on across a change of provider. */
		int64 SeqBase = 0;
		/** Tracked for the sensing bridge (TrackBridgePose): counted against MaxBridgePoints. */
		bool bBridge = false;
	};

	/** TrackPose / TrackBridgePose. */
	int32 TrackPoint(USceneComponent* Component, FName Bone, const FTransform& LocalPose, ERammsImuSource Preferred, float IdleTimeoutSeconds,
		bool bBridge, FString& OutError);

	/** The provider for an engine, made on first use (null when no engine registered one). */
	IRammsImuProvider* GetProvider(ERammsImuSource Source);

	/** Resolve the point and track it with the provider that serves it (Kinematic when that one refuses). */
	void Attach(FTrackedPoint& Point);

	/** Move a tracked point to Ref's provider, if that provider takes it; the old one keeps it otherwise (unless
	 *  bForce: then the point falls back to Kinematic). */
	void Reattach(FTrackedPoint& Point, const FRammsImuBodyRef& Ref, bool bForce);

	/** Track Ref with its provider (history on when the point keeps it). INDEX_NONE when the provider refuses. */
	int32 TrackWith(const FTrackedPoint& Point, const FRammsImuBodyRef& Ref);

	void Detach(FTrackedPoint& Point);

	double Now() const;

	TMap<int32, FTrackedPoint>							 Points;
	TMap<ERammsImuSource, TSharedPtr<IRammsImuProvider>> Providers;
	int32												 NextHandle = 1;
	/** A refusal has been logged since the last point was tracked (the world's limit, the bridge's): log the next one
	 *  only after a track succeeds again. */
	bool bLoggedWorldFull = false;
	bool bLoggedBridgeFull = false;
};
