// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "RammsImuProvider.h"
#include "UObject/WeakObjectPtr.h"

struct FBodyInstance;
class FRammsChaosImuCallback;
class UPrimitiveComponent;
class UWorld;

namespace Chaos
{
	class FSingleParticlePhysicsProxy;
}

/**
 * IMU data from Chaos: the rigid bodies a world's solver simulates (a primitive's body, a skeletal mesh's bone
 * bodies), sampled after every solver step on the physics thread by one sim callback per world.
 *
 * Each step reads the body's velocity at its centre of mass and its angular velocity from the solver, and the
 * accelerations as the first difference of those over the step (the solver's own dt); the lever arm from the
 * centre of mass to the point gives the point's. Gravity is the one the solver applies to that body (its gravity
 * group's, none when its gravity is off; the world's for a kinematic body). Only tracked bodies are read; each keeps
 * its latest sample and, when asked, a ring of every step's, under a lock the game thread takes to read them.
 *
 * Bodies are held by their physics proxy. A proxy the game thread releases (a destroyed component, recreated
 * physics state) is dropped on the physics thread before Chaos deletes it (the callback watches particle
 * unregistration, and forgets the body's samples), and the game thread hands over the body's new proxy on its next
 * read (GetSample or ReadHistory). A tracked body that is gone, or that is welded into another body, is no longer
 * valid (IsTrackValid): the subsystem resolves the point again. The history keeps complete samples only (a body's
 * first step, or its first on a new proxy, has no acceleration yet and is left out).
 */
class FRammsChaosImuProvider final : public IRammsImuProvider
{
public:
	explicit FRammsChaosImuProvider(UWorld& InWorld);
	virtual ~FRammsChaosImuProvider() override;

	virtual ERammsImuSource GetSource() const override { return ERammsImuSource::Chaos; }
	virtual bool			Resolve(USceneComponent& Component, FName Bone, const FTransform& LocalPose, bool bExplicit,
				   FRammsImuBodyRef& OutRef) const override;
	virtual int32			Track(const FRammsImuBodyRef& Ref) override;
	virtual bool			GetSample(int32 Handle, FRammsImuSample& OutSample) override;
	virtual void			Untrack(int32 Handle) override;
	virtual bool			IsTrackValid(int32 Handle) const override;
	virtual void			SetHistoryCapacity(int32 Handle, int32 Capacity) override;
	virtual bool			ReadHistory(int32 Handle, int64 Since, int32 Max, TArray<FRammsImuSample>& OutSamples, int64& OutFirst,
				   int64& OutOldest, int64& OutNext) override;
	virtual void			ListBodies(AActor& Actor, TArray<FRammsImuBodyRef>& OutBodies) const override;

	/** Prim's body instance (a welded primitive's weld parent's) or, for a skeletal mesh, the body of BoneOrSocket's
	 *  bone or of its nearest ancestor that has one (None: the root body). */
	static FBodyInstance* FindBody(UPrimitiveComponent& Prim, FName BoneOrSocket);

	/** One physics step's state of a body. */
	struct FStepState
	{
		bool	bValid = false;
		double	Time = 0.0;						   // solver time at the end of the step (s)
		FVector ComVelocity = FVector::ZeroVector; // cm/s, world
		FVector AngularVelocityRad = FVector::ZeroVector;
	};

	/** The sample of a point at Lever (point - centre of mass, world) from two consecutive steps' states: velocities
	 *  from Current, accelerations from the difference (none when Previous is not an earlier step). */
	static FRammsImuSample MakeSample(const FStepState& Previous, const FStepState& Current, const FVector& PointLocation,
		const FQuat& PointOrientation, const FVector& Lever, const FVector& Gravity);

	/** One tracked point, as the physics thread samples it. */
	struct FPoint
	{
		// Written on the game thread.
		Chaos::FSingleParticlePhysicsProxy* Proxy = nullptr; // also nulled by the physics thread on unregistration
		FTransform							PointInBody = FTransform::Identity;
		uint32								Generation = 0; // bumped whenever the body changes: the history starts over

		// Written on the physics thread (History sized on the game thread).
		uint32				SeenGeneration = MAX_uint32;
		FStepState			Previous;
		FStepState			Current;
		FRammsImuSample		Latest;
		FRammsImuSampleRing History;
	};

	/** The points, shared by the provider and the physics-thread callback. */
	struct FTable
	{
		FCriticalSection	Lock;
		TMap<int32, FPoint> Points;
	};

	/** Forget the latest sample and the step history a point's accelerations difference against (its body is gone
	 *  or changed); its ring keeps what it holds. */
	static void ForgetBodyState(FPoint& Point);

private:
	/** The callback on the world's solver, registered on the first track. */
	bool EnsureCallback();

	/** Give the physics thread the proxy the point's body has now (none once its physics state is gone, a new one once
	 *  it is recreated, none once it is welded into another body). Every read does this first. */
	void HandOverProxy(int32 Handle);

	struct FTracked
	{
		TWeakObjectPtr<UPrimitiveComponent> Body;
		FName								Bone;
	};

	TWeakObjectPtr<UWorld>					World;
	TSharedPtr<FTable, ESPMode::ThreadSafe> Table;
	TMap<int32, FTracked>					Tracked;
	int32									NextHandle = 1;
	FRammsChaosImuCallback*					Callback = nullptr;
	/** The solver Callback is registered with (compared, never dereferenced once the world is gone). */
	void* CallbackSolver = nullptr;
};

/** The factory RammsCore registers for ERammsImuSource::Chaos. */
TSharedPtr<IRammsImuProvider> MakeRammsChaosImuProvider(UWorld& World);
