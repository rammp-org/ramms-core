// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsPhysicsImuSubsystem.h"

#include "Components/SceneComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "RammsImuMath.h"

namespace RammsPhysicsImuImpl
{
	/** How often a point that fell back to Kinematic asks the engines again (s). */
	constexpr double ResolveRetrySeconds = 1.0;

	/** The engines Auto asks, in the order a tie goes. */
	const ERammsImuSource PhysicsEngines[] = { ERammsImuSource::MuJoCo, ERammsImuSource::Chaos };

	/** The Kinematic source: each point's frame differenced at the game rate, stepped by the subsystem's tick. */
	class FKinematicProvider final : public IRammsImuProvider
	{
	public:
		explicit FKinematicProvider(UWorld& InWorld)
			: OwnerWorld(&InWorld)
		{
		}

		virtual ERammsImuSource GetSource() const override { return ERammsImuSource::Kinematic; }

		virtual bool Resolve(USceneComponent& Component, FName Bone, const FTransform& LocalPose, bool /*bExplicit*/,
			FRammsImuBodyRef& OutRef) const override
		{
			OutRef.Source = ERammsImuSource::Kinematic;
			OutRef.Body = &Component;
			OutRef.Bone = Bone;
			OutRef.PointInBody = LocalPose;
			OutRef.BodyName = Bone.IsNone() ? Component.GetName() : Bone.ToString();
			OutRef.Depth = 0;
			return true;
		}

		virtual int32 Track(const FRammsImuBodyRef& Ref) override
		{
			USceneComponent* Body = Ref.Body.Get();
			if (!Body)
			{
				return INDEX_NONE;
			}
			const int32 Handle = NextHandle++;
			FPoint&		Point = Points.Add(Handle);
			Point.Body = Body;
			Point.Bone = Ref.Bone;
			Point.PointInBody = Ref.PointInBody;
			StepPoint(Point); // a pose at once
			return Handle;
		}

		virtual bool GetSample(int32 Handle, FRammsImuSample& OutSample) override
		{
			const FPoint* Point = Points.Find(Handle);
			if (!Point || !Point->Differencer.GetLatest().bValid)
			{
				return false;
			}
			OutSample = Point->Differencer.GetLatest();
			return true;
		}

		virtual void Untrack(int32 Handle) override { Points.Remove(Handle); }

		void Step()
		{
			for (TPair<int32, FPoint>& Pair : Points)
			{
				StepPoint(Pair.Value);
			}
		}

	private:
		struct FPoint
		{
			TWeakObjectPtr<USceneComponent> Body;
			FName							Bone;
			FTransform						PointInBody = FTransform::Identity;
			FRammsImuDifferencer			Differencer;
		};

		void StepPoint(FPoint& Point) const
		{
			const USceneComponent* Body = Point.Body.Get();
			const UWorld*		   InWorld = OwnerWorld.Get();
			if (!Body || !InWorld)
			{
				return;
			}
			Point.Differencer.Step(InWorld->GetTimeSeconds(), Point.PointInBody * RammsImuProviders::ComponentFrame(*Body, Point.Bone),
				FVector(0.0, 0.0, InWorld->GetGravityZ()));
		}

		TWeakObjectPtr<UWorld> OwnerWorld;
		TMap<int32, FPoint>	   Points;
		int32				   NextHandle = 1;
	};
} // namespace RammsPhysicsImuImpl

// ---------------------------------------------------------------------------

const FRammsImuSample& FRammsImuDifferencer::Step(double InTime, const FTransform& Sensor, const FVector& Gravity)
{
	if (bHasPose && InTime <= Time)
	{
		return Latest; // the same frame again
	}
	const FVector NewLocation = Sensor.GetLocation();
	const FQuat	  NewRotation = Sensor.GetRotation();

	FRammsImuSample Sample;
	Sample.bValid = true;
	Sample.Source = ERammsImuSource::Kinematic;
	Sample.SimTimeSeconds = InTime;
	Sample.Location = NewLocation;
	Sample.Orientation = NewRotation;
	Sample.Gravity = Gravity;
	if (bHasPose)
	{
		const double  Dt = InTime - Time;
		const FVector NewVelocity = RammsImuMath::Rate(Location, NewLocation, Dt);
		const FVector NewAngularVelocity = RammsImuMath::AngularVelocityBetween(Rotation, NewRotation, Dt);
		Sample.LinearVelocity = NewVelocity;
		Sample.AngularVelocity = RammsImuMath::RadiansToDegrees(NewAngularVelocity);
		if (bHasVelocity)
		{
			Sample.ProperAcceleration = RammsImuMath::ProperAcceleration(RammsImuMath::Rate(Velocity, NewVelocity, Dt), Gravity);
			Sample.AngularAcceleration = RammsImuMath::RadiansToDegrees(RammsImuMath::Rate(AngularVelocityRad, NewAngularVelocity, Dt));
			Sample.bHasAcceleration = true;
		}
		Velocity = NewVelocity;
		AngularVelocityRad = NewAngularVelocity;
		bHasVelocity = true;
	}
	Location = NewLocation;
	Rotation = NewRotation;
	Time = InTime;
	bHasPose = true;
	Latest = Sample;
	return Latest;
}

// ---------------------------------------------------------------------------

URammsPhysicsImuSubsystem* URammsPhysicsImuSubsystem::Get(const UObject* WorldContext)
{
	UWorld* ContextWorld = WorldContext ? WorldContext->GetWorld() : nullptr;
	return ContextWorld ? ContextWorld->GetSubsystem<URammsPhysicsImuSubsystem>() : nullptr;
}

double URammsPhysicsImuSubsystem::Now() const
{
	// Wall time: an idle point expires and a fallback retries while the game is paused too.
	return FPlatformTime::Seconds();
}

int32 URammsPhysicsImuSubsystem::Track(USceneComponent* Component, FVector LocalOffset, ERammsImuSource Preferred, FName Bone)
{
	return TrackPose(Component, Bone, FTransform(LocalOffset), Preferred);
}

int32 URammsPhysicsImuSubsystem::TrackPose(USceneComponent* Component, FName Bone, const FTransform& LocalPose, ERammsImuSource Preferred,
	float IdleTimeoutSeconds)
{
	if (!IsValid(Component))
	{
		return 0;
	}
	FTrackedPoint Point;
	Point.Component = Component;
	Point.Bone = Bone;
	Point.LocalPose = LocalPose;
	Point.LocalPose.SetScale3D(FVector::OneVector);
	Point.Preferred = Preferred;
	Point.IdleTimeoutSeconds = IdleTimeoutSeconds;
	Point.LastRead = Now();
	Attach(Point);
	const int32 Handle = NextHandle++;
	Points.Add(Handle, MoveTemp(Point));
	return Handle;
}

bool URammsPhysicsImuSubsystem::GetSample(int32 Handle, FRammsImuSample& OutSample)
{
	FTrackedPoint* Point = Points.Find(Handle);
	if (!Point)
	{
		return false;
	}
	Point->LastRead = Now();
	const TSharedPtr<IRammsImuProvider>* Provider = Providers.Find(Point->Ref.Source);
	return Provider && Provider->IsValid() && Point->ProviderHandle != INDEX_NONE && (*Provider)->GetSample(Point->ProviderHandle, OutSample);
}

void URammsPhysicsImuSubsystem::Untrack(int32 Handle)
{
	if (FTrackedPoint* Point = Points.Find(Handle))
	{
		Detach(*Point);
		Points.Remove(Handle);
	}
}

bool URammsPhysicsImuSubsystem::IsTracked(int32 Handle) const
{
	return Points.Contains(Handle);
}

ERammsImuSource URammsPhysicsImuSubsystem::GetTrackedSource(int32 Handle) const
{
	const FTrackedPoint* Point = Points.Find(Handle);
	return Point && Point->ProviderHandle != INDEX_NONE ? Point->Ref.Source : ERammsImuSource::Kinematic;
}

FString URammsPhysicsImuSubsystem::GetTrackedBodyName(int32 Handle) const
{
	const FTrackedPoint* Point = Points.Find(Handle);
	return Point ? Point->Ref.BodyName : FString();
}

IRammsImuProvider* URammsPhysicsImuSubsystem::GetProvider(ERammsImuSource Source)
{
	if (Source == ERammsImuSource::Auto)
	{
		return nullptr;
	}
	if (const TSharedPtr<IRammsImuProvider>* Found = Providers.Find(Source))
	{
		return Found->Get();
	}
	UWorld* OwnerWorld = GetWorld();
	if (!OwnerWorld)
	{
		return nullptr;
	}
	TSharedPtr<IRammsImuProvider> Made;
	if (Source == ERammsImuSource::Kinematic)
	{
		Made = MakeShared<RammsPhysicsImuImpl::FKinematicProvider>(*OwnerWorld);
	}
	else
	{
		Made = RammsImuProviders::Create(Source, *OwnerWorld);
	}
	if (!Made.IsValid())
	{
		return nullptr; // no engine registered one (RammsMujocoSupport not loaded)
	}
	Providers.Add(Source, Made);
	return Made.Get();
}

bool URammsPhysicsImuSubsystem::ResolveBody(USceneComponent& Component, FName Bone, const FTransform& LocalPose, ERammsImuSource Preferred,
	FRammsImuBodyRef& OutRef)
{
	// By name: that engine, if it simulates the body (it may hand the point to another: a URLab robot on Chaos).
	if (Preferred == ERammsImuSource::Chaos || Preferred == ERammsImuSource::MuJoCo)
	{
		if (const IRammsImuProvider* Provider = GetProvider(Preferred); Provider && Provider->Resolve(Component, Bone, LocalPose, true, OutRef))
		{
			return true;
		}
	}
	// Auto: every engine, the nearest body up the attach chain winning.
	if (Preferred != ERammsImuSource::Kinematic)
	{
		bool			 bFound = false;
		FRammsImuBodyRef Best;
		for (const ERammsImuSource Engine : RammsPhysicsImuImpl::PhysicsEngines)
		{
			const IRammsImuProvider* Provider = GetProvider(Engine);
			FRammsImuBodyRef		 Candidate;
			if (Provider && Provider->Resolve(Component, Bone, LocalPose, false, Candidate) && (!bFound || Candidate.Depth < Best.Depth))
			{
				Best = Candidate;
				bFound = true;
			}
		}
		if (bFound)
		{
			OutRef = Best;
			return true;
		}
	}
	const IRammsImuProvider* Kinematic = GetProvider(ERammsImuSource::Kinematic);
	return Kinematic && Kinematic->Resolve(Component, Bone, LocalPose, false, OutRef);
}

void URammsPhysicsImuSubsystem::ListBodies(AActor& Actor, TArray<FRammsImuBodyRef>& OutBodies)
{
	for (const ERammsImuSource Engine : RammsPhysicsImuImpl::PhysicsEngines)
	{
		if (const IRammsImuProvider* Provider = GetProvider(Engine))
		{
			Provider->ListBodies(Actor, OutBodies);
		}
	}
}

void URammsPhysicsImuSubsystem::Attach(FTrackedPoint& Point)
{
	Point.ProviderHandle = INDEX_NONE;
	Point.NextResolve = Now() + RammsPhysicsImuImpl::ResolveRetrySeconds;
	USceneComponent* Component = Point.Component.Get();
	if (!Component)
	{
		return;
	}
	FRammsImuBodyRef Ref;
	ResolveBody(*Component, Point.Bone, Point.LocalPose, Point.Preferred, Ref);
	IRammsImuProvider* Provider = GetProvider(Ref.Source);
	int32			   Handle = Provider ? Provider->Track(Ref) : INDEX_NONE;
	if (Handle == INDEX_NONE && Ref.Source != ERammsImuSource::Kinematic)
	{
		// The engine could not take it (no physics scene): differencing it is.
		Provider = GetProvider(ERammsImuSource::Kinematic);
		if (Provider && Provider->Resolve(*Component, Point.Bone, Point.LocalPose, false, Ref))
		{
			Handle = Provider->Track(Ref);
		}
	}
	Point.Ref = Ref;
	Point.ProviderHandle = Handle;
}

void URammsPhysicsImuSubsystem::Detach(FTrackedPoint& Point)
{
	if (Point.ProviderHandle != INDEX_NONE)
	{
		if (const TSharedPtr<IRammsImuProvider>* Provider = Providers.Find(Point.Ref.Source); Provider && Provider->IsValid())
		{
			(*Provider)->Untrack(Point.ProviderHandle);
		}
	}
	Point.ProviderHandle = INDEX_NONE;
}

void URammsPhysicsImuSubsystem::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	const double WallNow = Now();
	for (auto It = Points.CreateIterator(); It; ++It)
	{
		FTrackedPoint& Point = It.Value();
		const bool	   bIdle = Point.IdleTimeoutSeconds > 0.0f && WallNow - Point.LastRead > Point.IdleTimeoutSeconds;
		if (bIdle || !Point.Component.IsValid())
		{
			Detach(Point);
			It.RemoveCurrent();
			continue;
		}
		// A point that wanted an engine but got Kinematic asks again now and then: MuJoCo compiles after BeginPlay, a
		// Chaos articulation builds on its first tick.
		const bool bFellBack = Point.Preferred != ERammsImuSource::Kinematic
			&& (Point.Ref.Source == ERammsImuSource::Kinematic || Point.ProviderHandle == INDEX_NONE);
		if (bFellBack && WallNow >= Point.NextResolve)
		{
			FRammsImuBodyRef Ref;
			if (ResolveBody(*Point.Component.Get(), Point.Bone, Point.LocalPose, Point.Preferred, Ref) && Ref.Source != ERammsImuSource::Kinematic)
			{
				Detach(Point);
				Attach(Point);
			}
			else
			{
				Point.NextResolve = WallNow + RammsPhysicsImuImpl::ResolveRetrySeconds;
			}
		}
	}

	if (const TSharedPtr<IRammsImuProvider>* Kinematic = Providers.Find(ERammsImuSource::Kinematic); Kinematic && Kinematic->IsValid())
	{
		static_cast<RammsPhysicsImuImpl::FKinematicProvider*>(Kinematic->Get())->Step();
	}
}

TStatId URammsPhysicsImuSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(URammsPhysicsImuSubsystem, STATGROUP_Tickables);
}

bool URammsPhysicsImuSubsystem::IsTickable() const
{
	return Points.Num() > 0;
}

void URammsPhysicsImuSubsystem::Deinitialize()
{
	for (TPair<int32, FTrackedPoint>& Pair : Points)
	{
		Detach(Pair.Value);
	}
	Points.Reset();
	// A Chaos provider unregisters its solver callback here, while the world's physics scene still exists.
	Providers.Reset();
	Super::Deinitialize();
}

FString URammsPhysicsImuSubsystem::SourceName(ERammsImuSource Source)
{
	switch (Source)
	{
		case ERammsImuSource::Auto:
			return TEXT("auto");
		case ERammsImuSource::Chaos:
			return TEXT("chaos");
		case ERammsImuSource::MuJoCo:
			return TEXT("mujoco");
		case ERammsImuSource::Kinematic:
		default:
			return TEXT("kinematic");
	}
}

bool URammsPhysicsImuSubsystem::ParseSource(const FString& Text, ERammsImuSource& OutSource)
{
	const FString		  Trimmed = Text.TrimStartAndEnd();
	const ERammsImuSource All[] = { ERammsImuSource::Auto, ERammsImuSource::Chaos, ERammsImuSource::MuJoCo, ERammsImuSource::Kinematic };
	for (const ERammsImuSource Candidate : All)
	{
		if (Trimmed.Equals(SourceName(Candidate), ESearchCase::IgnoreCase))
		{
			OutSource = Candidate;
			return true;
		}
	}
	return false;
}
