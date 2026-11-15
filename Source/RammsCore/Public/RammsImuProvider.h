// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsImuTypes.h"
#include "UObject/WeakObjectPtr.h"

class AActor;
class USceneComponent;
class UWorld;

/** Where a tracked point lives: the body an engine simulates that carries it, and the point's frame in the body's. */
struct RAMMSCORE_API FRammsImuBodyRef
{
	/** The engine that serves the point (a provider may hand a point to another: a URLab body that Chaos
	 *  simulates resolves to Chaos). */
	ERammsImuSource Source = ERammsImuSource::Kinematic;

	/** The body: a primitive (Chaos), a skeletal mesh with Bone (Chaos), a URLab body component (MuJoCo), or any
	 *  scene component (Kinematic). */
	TWeakObjectPtr<USceneComponent> Body;

	/** The bone of a skeletal mesh Body (Chaos), else None. */
	FName Bone;

	/** The sensor frame in the body's frame (cm, unscaled): the physics body's own frame for Chaos (its particle's
	 *  actor frame), the URLab body component's for MuJoCo, Body (and Bone)'s for Kinematic. */
	FTransform PointInBody = FTransform::Identity;

	/** The body's name for reports: a MuJoCo body name, a bone name, or a component name. */
	FString BodyName;

	/** Attach hops from the tracked component up to Body (0: the component itself). Auto takes the nearest. */
	int32 Depth = 0;
};

/**
 * A physics engine's IMU data source, per world: it tracks points on the bodies it simulates and samples them every
 * physics step (on its own thread), keeping the latest sample of each and, when asked, a ring of every step's.
 * RammsCore owns this interface and a Chaos provider; RammsMujocoSupport registers a MuJoCo one, so RammsCore depends
 * on no engine plugin.
 *
 * All calls are on the game thread. URammsPhysicsImuSubsystem is the usual client.
 */
class RAMMSCORE_API IRammsImuProvider
{
public:
	virtual ~IRammsImuProvider() = default;

	/** The engine this provider serves. */
	virtual ERammsImuSource GetSource() const = 0;

	/**
	 * Find the body this engine simulates that carries the sensor frame LocalPose x (Component, Bone), walking up
	 * Component's attach chain (Component included). bExplicit: the caller asked for this engine by name, so a body
	 * it only moves kinematically will do. False when this engine simulates none of them.
	 */
	virtual bool Resolve(USceneComponent& Component, FName Bone, const FTransform& LocalPose, bool bExplicit,
		FRammsImuBodyRef& OutRef) const = 0;

	/** Start sampling a point Resolve found (Ref.Source is this provider's). A provider handle, or INDEX_NONE. */
	virtual int32 Track(const FRammsImuBodyRef& Ref) = 0;

	/** The latest physics-rate sample of a tracked point. False before the first physics step has sampled it (or
	 *  when its body is gone). */
	virtual bool GetSample(int32 Handle, FRammsImuSample& OutSample) = 0;

	virtual void Untrack(int32 Handle) = 0;

	/** False once the body a tracked point was resolved on can no longer serve it (destroyed, re-welded into
	 *  another body, handed to another engine): the subsystem resolves the point again. */
	virtual bool IsTrackValid(int32 Handle) const { return true; }

	/** Keep the last Capacity samples of a tracked point, one per physics step (0: stop). Allocates; the physics
	 *  thread then fills the ring without allocating. */
	virtual void SetHistoryCapacity(int32 Handle, int32 Capacity) {}

	/**
	 * Append up to Max kept samples of a tracked point from the provider's sequence number Since on (clamped to what
	 * is kept), oldest first. OutFirst: the sequence of the first one appended; [OutOldest, OutNext): what is kept.
	 * False when the point keeps no history.
	 */
	virtual bool ReadHistory(int32 Handle, int64 Since, int32 Max, TArray<FRammsImuSample>& OutSamples, int64& OutFirst,
		int64& OutOldest, int64& OutNext)
	{
		return false;
	}

	/** The bodies of Actor (and of actors attached to it) this engine simulates, main body first. */
	virtual void ListBodies(AActor& Actor, TArray<FRammsImuBodyRef>& OutBodies) const {}
};

/** Creates a provider for a world (or null when the engine has nothing in it). */
DECLARE_DELEGATE_RetVal_OneParam(TSharedPtr<IRammsImuProvider>, FRammsImuProviderFactory, UWorld&);

/** Cross-plugin registration point for IMU providers, one per engine (RammsCore registers Chaos at startup,
 *  RammsMujocoSupport MuJoCo). */
namespace RammsImuProviders
{
	RAMMSCORE_API void Register(ERammsImuSource Source, FRammsImuProviderFactory Factory);
	RAMMSCORE_API void Unregister(ERammsImuSource Source);
	RAMMSCORE_API bool IsRegistered(ERammsImuSource Source);

	/** A new provider for World, or null when none is registered for Source. */
	RAMMSCORE_API TSharedPtr<IRammsImuProvider> Create(ERammsImuSource Source, UWorld& World);

	/** The world transform, scale removed, of Component, or of its bone / socket Bone. */
	RAMMSCORE_API FTransform ComponentFrame(const USceneComponent& Component, FName Bone);
} // namespace RammsImuProviders
