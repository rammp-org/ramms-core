// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsChaosImuProvider.h"

#include "Chaos/ParticleHandle.h"
#include "Chaos/SimCallbackObject.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "PBDRigidsSolver.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "RammsImuMath.h"

/**
 * Samples the tracked bodies after every solver step (PostSolve: the step's velocities are final; the positions are
 * the step's new ones, P / Q, which the solver copies to X / R right after). PostSolve runs once per evolution step:
 * when the solver splits one step in several, each overwrites the last and the step's end time stamps the final one.
 * Presimulate is requested only because the solver calls it on every callback that asks for a simulation hook.
 */
class FRammsChaosImuCallback : public Chaos::TSimCallbackObject<Chaos::FSimCallbackNoInput, Chaos::FSimCallbackNoOutput, Chaos::ESimCallbackOptions::Presimulate | Chaos::ESimCallbackOptions::PostSolve | Chaos::ESimCallbackOptions::ParticleUnregister>
{
public:
	explicit FRammsChaosImuCallback(TSharedPtr<FRammsChaosImuProvider::FTable, ESPMode::ThreadSafe> InTable)
		: Table(MoveTemp(InTable))
	{
	}

	virtual FName GetFNameForStatId() const override
	{
		static const FName StatName(TEXT("RammsChaosImu"));
		return StatName;
	}

	virtual void OnPreSimulate_Internal() override {}

	virtual void OnPostSolve_Internal() override
	{
		using FPoint = FRammsChaosImuProvider::FPoint;
		using FStepState = FRammsChaosImuProvider::FStepState;

		const double Now = GetSimTime_Internal() + GetDeltaTime_Internal();
		FScopeLock	 Guard(&Table->Lock);
		for (TPair<int32, FPoint>& Pair : Table->Points)
		{
			FPoint& Point = Pair.Value;
			if (Point.SeenGeneration != Point.Generation)
			{
				Point.SeenGeneration = Point.Generation;
				Point.Previous = FStepState();
				Point.Current = FStepState();
				Point.Latest = FRammsImuSample();
			}
			// Null until the physics thread has registered the particle.
			Chaos::FGeometryParticleHandle* Handle = Point.Proxy ? Point.Proxy->GetHandle_LowLevel() : nullptr;
			if (Handle == nullptr)
			{
				continue;
			}
			const Chaos::FGenericParticleHandle Particle(Handle);
			const FTransform					BodyWorld(FQuat(Particle->GetQ()), FVector(Particle->GetP()));
			const FTransform					Sensor = Point.PointInBody * BodyWorld;

			FStepState State;
			State.bValid = true;
			State.Time = Now;
			State.ComVelocity = FVector(Particle->GetV());
			State.AngularVelocityRad = FVector(Particle->GetW());
			if (Point.Current.bValid && Now > Point.Current.Time)
			{
				Point.Previous = Point.Current;
			}
			else if (Point.Current.bValid && Now < Point.Current.Time)
			{
				Point.Previous = FStepState(); // the clock went back (a rewind): start over
			}
			Point.Current = State;
			Point.Latest = FRammsChaosImuProvider::MakeSample(Point.Previous, Point.Current, Sensor.GetLocation(), Sensor.GetRotation(),
				Sensor.GetLocation() - FVector(Particle->PCom()), Point.Gravity);
		}
	}

	virtual void OnParticleUnregistered_Internal(TArray<TTuple<Chaos::FUniqueIdx, Chaos::FSingleParticlePhysicsProxy*>>& UnregisteredProxies) override
	{
		// Called before Chaos deletes these proxies: no point may read one after this.
		FScopeLock Guard(&Table->Lock);
		for (const TTuple<Chaos::FUniqueIdx, Chaos::FSingleParticlePhysicsProxy*>& Entry : UnregisteredProxies)
		{
			for (TPair<int32, FRammsChaosImuProvider::FPoint>& Pair : Table->Points)
			{
				if (Pair.Value.Proxy == Entry.Get<1>())
				{
					Pair.Value.Proxy = nullptr;
				}
			}
		}
	}

private:
	TSharedPtr<FRammsChaosImuProvider::FTable, ESPMode::ThreadSafe> Table;
};

// ---------------------------------------------------------------------------

TSharedPtr<IRammsImuProvider> MakeRammsChaosImuProvider(UWorld& World)
{
	return MakeShared<FRammsChaosImuProvider>(World);
}

FRammsChaosImuProvider::FRammsChaosImuProvider(UWorld& InWorld)
	: World(&InWorld)
	, Table(MakeShared<FTable, ESPMode::ThreadSafe>())
{
}

FRammsChaosImuProvider::~FRammsChaosImuProvider()
{
	{
		FScopeLock Guard(&Table->Lock);
		Table->Points.Reset();
	}
	if (Callback)
	{
		// Freed on the physics thread once the solver has dropped it; it keeps the table alive until then.
		UWorld*				   OwnerWorld = World.Get();
		auto*				   Scene = OwnerWorld ? OwnerWorld->GetPhysicsScene() : nullptr;
		Chaos::FPhysicsSolver* Solver = Scene ? Scene->GetSolver() : nullptr;
		if (Solver && Solver == CallbackSolver)
		{
			Solver->UnregisterAndFreeSimCallbackObject_External(Callback);
		}
		Callback = nullptr;
		CallbackSolver = nullptr;
	}
}

bool FRammsChaosImuProvider::EnsureCallback()
{
	UWorld*				   OwnerWorld = World.Get();
	auto*				   Scene = OwnerWorld ? OwnerWorld->GetPhysicsScene() : nullptr;
	Chaos::FPhysicsSolver* Solver = Scene ? Scene->GetSolver() : nullptr;
	if (!Solver)
	{
		return false;
	}
	if (Callback && CallbackSolver == Solver)
	{
		return true;
	}
	Callback = Solver->CreateAndRegisterSimCallbackObject_External<FRammsChaosImuCallback>(Table);
	CallbackSolver = Solver;
	return Callback != nullptr;
}

FBodyInstance* FRammsChaosImuProvider::FindBody(UPrimitiveComponent& Prim, FName BoneOrSocket)
{
	if (USkeletalMeshComponent* Skel = Cast<USkeletalMeshComponent>(&Prim))
	{
		const FName Bone = BoneOrSocket.IsNone() ? NAME_None : Skel->GetSocketBoneName(BoneOrSocket);
		if (Bone.IsNone())
		{
			return Skel->GetBodyInstance(NAME_None); // the root body
		}
		// A bone without a body of its own (a finger, a socket's bone) rides on its nearest ancestor that has one.
		for (FName Candidate = Bone; !Candidate.IsNone(); Candidate = Skel->GetParentBone(Candidate))
		{
			if (FBodyInstance* Body = Skel->GetBodyInstance(Candidate))
			{
				return Body;
			}
		}
		return nullptr;
	}
	return Prim.GetBodyInstance(NAME_None, /*bGetWelded=*/true);
}

bool FRammsChaosImuProvider::Resolve(USceneComponent& Component, FName Bone, const FTransform& LocalPose, bool bExplicit,
	FRammsImuBodyRef& OutRef) const
{
	const FTransform SensorWorld = LocalPose * RammsImuProviders::ComponentFrame(Component, Bone);

	// Up the attach chain: the first primitive whose body (for a skeletal mesh, the body of the bone the chain is
	// attached at) Chaos simulates. Asked for by name, a body Chaos only moves kinematically will do.
	int32 Depth = 0;
	FName AttachedAt = Bone;
	for (USceneComponent* Node = &Component; Node; AttachedAt = Node->GetAttachSocketName(), Node = Node->GetAttachParent(), ++Depth)
	{
		UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(Node);
		FBodyInstance*		 Body = Prim ? FindBody(*Prim, AttachedAt) : nullptr;
		if (!Body || !Body->IsValidBodyInstance())
		{
			continue;
		}
		if (!Body->IsInstanceSimulatingPhysics() && !(bExplicit && Body->GetPhysicsActor() != nullptr))
		{
			continue;
		}
		UPrimitiveComponent* Owner = Body->OwnerComponent.Get();
		if (!Owner)
		{
			continue;
		}
		const USkeletalMeshComponent* Skel = Cast<USkeletalMeshComponent>(Owner);
		OutRef.Source = ERammsImuSource::Chaos;
		OutRef.Body = Owner;
		OutRef.Bone = Skel ? Skel->GetBoneName(Body->InstanceBoneIndex) : FName(NAME_None);
		// The body's own frame: its particle's actor frame, which a sample composes PointInBody with.
		FTransform BodyWorld = Body->GetUnrealWorldTransform();
		BodyWorld.SetScale3D(FVector::OneVector);
		OutRef.PointInBody = SensorWorld.GetRelativeTransform(BodyWorld);
		OutRef.BodyName = OutRef.Bone.IsNone() ? Owner->GetName() : OutRef.Bone.ToString();
		OutRef.Depth = Depth;
		return true;
	}
	return false;
}

int32 FRammsChaosImuProvider::Track(const FRammsImuBodyRef& Ref)
{
	UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(Ref.Body.Get());
	if (!Prim || Ref.Source != ERammsImuSource::Chaos || !EnsureCallback())
	{
		return INDEX_NONE;
	}
	const int32 Handle = NextHandle++;
	FTracked&	Entry = Tracked.Add(Handle);
	Entry.Body = Prim;
	Entry.Bone = Ref.Bone;
	{
		FScopeLock Guard(&Table->Lock);
		FPoint&	   Point = Table->Points.Add(Handle);
		Point.PointInBody = Ref.PointInBody;
	}
	// Hands the body's proxy and the world's gravity over.
	FRammsImuSample Unused;
	GetSample(Handle, Unused);
	return Handle;
}

bool FRammsChaosImuProvider::GetSample(int32 Handle, FRammsImuSample& OutSample)
{
	const FTracked* Entry = Tracked.Find(Handle);
	if (!Entry)
	{
		return false;
	}
	// The body's proxy as the game thread holds it now: null once its physics state is gone, a new one once it is
	// recreated.
	Chaos::FSingleParticlePhysicsProxy* Proxy = nullptr;
	if (UPrimitiveComponent* Prim = Entry->Body.Get())
	{
		if (FBodyInstance* Body = FindBody(*Prim, Entry->Bone); Body && Body->IsValidBodyInstance())
		{
			Proxy = Body->GetPhysicsActor();
		}
	}
	const UWorld* OwnerWorld = World.Get();
	const FVector Gravity(0.0, 0.0, OwnerWorld ? OwnerWorld->GetGravityZ() : 0.0);

	FScopeLock Guard(&Table->Lock);
	FPoint*	   Point = Table->Points.Find(Handle);
	if (!Point)
	{
		return false;
	}
	if (Point->Proxy != Proxy)
	{
		Point->Proxy = Proxy;
		++Point->Generation;
	}
	Point->Gravity = Gravity;
	if (Point->SeenGeneration != Point->Generation || !Point->Latest.bValid)
	{
		return false;
	}
	OutSample = Point->Latest;
	return true;
}

void FRammsChaosImuProvider::Untrack(int32 Handle)
{
	Tracked.Remove(Handle);
	FScopeLock Guard(&Table->Lock);
	Table->Points.Remove(Handle);
}

void FRammsChaosImuProvider::ListBodies(AActor& Actor, TArray<FRammsImuBodyRef>& OutBodies) const
{
	TArray<AActor*> Actors;
	Actors.Add(&Actor);
	TArray<AActor*> Attached;
	Actor.GetAttachedActors(Attached, /*bResetArray=*/true, /*bRecursivelyIncludeAttachedActors=*/true);
	Actors.Append(Attached);

	for (AActor* Each : Actors)
	{
		if (!IsValid(Each))
		{
			continue;
		}
		const USceneComponent*						Root = Each->GetRootComponent();
		TInlineComponentArray<UPrimitiveComponent*> Prims(Each);
		for (UPrimitiveComponent* Prim : Prims)
		{
			if (!IsValid(Prim))
			{
				continue;
			}
			TArray<FRammsImuBodyRef> Found;
			if (USkeletalMeshComponent* Skel = Cast<USkeletalMeshComponent>(Prim))
			{
				for (const FBodyInstance* Body : Skel->Bodies)
				{
					if (Body && Body->IsInstanceSimulatingPhysics())
					{
						FRammsImuBodyRef& Ref = Found.AddDefaulted_GetRef();
						Ref.Source = ERammsImuSource::Chaos;
						Ref.Body = Skel;
						Ref.Bone = Skel->GetBoneName(Body->InstanceBoneIndex);
						Ref.BodyName = Ref.Bone.ToString();
					}
				}
			}
			else if (const FBodyInstance* Body = Prim->GetBodyInstance(NAME_None, /*bGetWelded=*/false);
					 Body && Body->IsInstanceSimulatingPhysics())
			{
				FRammsImuBodyRef& Ref = Found.AddDefaulted_GetRef();
				Ref.Source = ERammsImuSource::Chaos;
				Ref.Body = Prim;
				Ref.BodyName = Prim->GetName();
			}
			// The main body (the root of the actor asked about) first.
			if (Each == &Actor && Prim == Root)
			{
				OutBodies.Insert(Found, 0);
			}
			else
			{
				OutBodies.Append(Found);
			}
		}
	}
}

FRammsImuSample FRammsChaosImuProvider::MakeSample(const FStepState& Previous, const FStepState& Current, const FVector& PointLocation,
	const FQuat& PointOrientation, const FVector& Lever, const FVector& Gravity)
{
	FRammsImuSample Sample;
	Sample.bValid = Current.bValid;
	Sample.Source = ERammsImuSource::Chaos;
	Sample.SimTimeSeconds = Current.Time;
	Sample.Location = PointLocation;
	Sample.Orientation = PointOrientation;
	Sample.Gravity = Gravity;
	Sample.LinearVelocity = RammsImuMath::PointVelocity(Current.ComVelocity, Current.AngularVelocityRad, Lever);
	Sample.AngularVelocity = RammsImuMath::RadiansToDegrees(Current.AngularVelocityRad);
	const double Dt = Current.Time - Previous.Time;
	if (Previous.bValid && Current.bValid && Dt > 0.0)
	{
		const FVector ComAcceleration = RammsImuMath::Rate(Previous.ComVelocity, Current.ComVelocity, Dt);
		const FVector AngularAcceleration = RammsImuMath::Rate(Previous.AngularVelocityRad, Current.AngularVelocityRad, Dt);
		const FVector PointAcceleration = RammsImuMath::PointAcceleration(ComAcceleration, AngularAcceleration, Current.AngularVelocityRad, Lever);
		Sample.ProperAcceleration = RammsImuMath::ProperAcceleration(PointAcceleration, Gravity);
		Sample.AngularAcceleration = RammsImuMath::RadiansToDegrees(AngularAcceleration);
		Sample.bHasAcceleration = true;
	}
	return Sample;
}
