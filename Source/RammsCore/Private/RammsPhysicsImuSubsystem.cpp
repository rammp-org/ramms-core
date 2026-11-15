// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsPhysicsImuSubsystem.h"

#include "Components/SceneComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "RammsImuMath.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsPhysicsImuSubsystem, Log, All);

namespace RammsPhysicsImuImpl
{
	/** How often every point asks the engines again which one serves it (s). */
	constexpr double ResolveRetrySeconds = 1.0;

	/** The engines Auto asks, in the order a tie goes. */
	const ERammsImuSource PhysicsEngines[] = { ERammsImuSource::MuJoCo, ERammsImuSource::Chaos };

	/** Two resolutions name the same body of the same engine. */
	bool SameBody(const FRammsImuBodyRef& A, const FRammsImuBodyRef& B)
	{
		return A.Source == B.Source && A.Body == B.Body && A.Bone == B.Bone;
	}

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

		virtual bool IsTrackValid(int32 Handle) const override
		{
			const FPoint* Point = Points.Find(Handle);
			return Point && Point->Body.IsValid();
		}

		virtual void SetHistoryCapacity(int32 Handle, int32 Capacity) override
		{
			if (FPoint* Point = Points.Find(Handle))
			{
				Point->History.SetCapacity(Capacity);
			}
		}

		virtual bool ReadHistory(int32 Handle, int64 Since, int32 Max, TArray<FRammsImuSample>& OutSamples, int64& OutFirst,
			int64& OutOldest, int64& OutNext) override
		{
			const FPoint* Point = Points.Find(Handle);
			if (!Point || Point->History.GetCapacity() == 0)
			{
				return false;
			}
			OutFirst = Point->History.Read(Since, Max, OutSamples);
			OutOldest = Point->History.GetOldestSeq();
			OutNext = Point->History.GetNextSeq();
			return true;
		}

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
			FRammsImuSampleRing				History;
		};

		void StepPoint(FPoint& Point) const
		{
			const USceneComponent* Body = Point.Body.Get();
			const UWorld*		   InWorld = OwnerWorld.Get();
			if (!Body || !InWorld)
			{
				return;
			}
			const FRammsImuSample& Sample = Point.Differencer.Step(InWorld->GetTimeSeconds(),
				Point.PointInBody * RammsImuProviders::ComponentFrame(*Body, Point.Bone), FVector(0.0, 0.0, InWorld->GetGravityZ()));
			if (Sample.bHasAcceleration)
			{
				// Complete samples only (the first two frames give a pose, then a velocity); the same frame again
				// replaces itself.
				Point.History.Push(Sample);
			}
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
			// Each velocity belongs to the middle of its interval: the two are half of each interval apart.
			const double Spacing = 0.5 * (Dt + LastDt);
			Sample.ProperAcceleration = RammsImuMath::ProperAcceleration(RammsImuMath::Rate(Velocity, NewVelocity, Spacing), Gravity);
			Sample.AngularAcceleration = RammsImuMath::RadiansToDegrees(RammsImuMath::Rate(AngularVelocityRad, NewAngularVelocity, Spacing));
			Sample.bHasAcceleration = true;
		}
		Velocity = NewVelocity;
		AngularVelocityRad = NewAngularVelocity;
		LastDt = Dt;
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
	// Wall time: an idle point expires and a point is resolved again while the game is paused too.
	return FPlatformTime::Seconds();
}

int32 URammsPhysicsImuSubsystem::Track(USceneComponent* Component, FVector LocalOffset, ERammsImuSource Preferred, FName Bone)
{
	return TrackPose(Component, Bone, FTransform(LocalOffset), Preferred);
}

int32 URammsPhysicsImuSubsystem::TrackPose(USceneComponent* Component, FName Bone, const FTransform& LocalPose, ERammsImuSource Preferred,
	float IdleTimeoutSeconds)
{
	FString Unused;
	return TrackPoint(Component, Bone, LocalPose, Preferred, IdleTimeoutSeconds, /*bBridge=*/false, Unused);
}

int32 URammsPhysicsImuSubsystem::TrackBridgePose(USceneComponent* Component, FName Bone, const FTransform& LocalPose, ERammsImuSource Preferred,
	float IdleTimeoutSeconds, FString& OutError)
{
	return TrackPoint(Component, Bone, LocalPose, Preferred, IdleTimeoutSeconds, /*bBridge=*/true, OutError);
}

int32 URammsPhysicsImuSubsystem::GetBridgePointCount() const
{
	int32 Count = 0;
	for (const TPair<int32, FTrackedPoint>& Pair : Points)
	{
		Count += Pair.Value.bBridge ? 1 : 0;
	}
	return Count;
}

int32 URammsPhysicsImuSubsystem::TrackPoint(USceneComponent* Component, FName Bone, const FTransform& LocalPose, ERammsImuSource Preferred,
	float IdleTimeoutSeconds, bool bBridge, FString& OutError)
{
	if (!IsValid(Component))
	{
		OutError = TEXT("no component to track");
		return 0;
	}
	// A refused point is logged once: the IMU component asks again every second and a Remote Control client may poll
	// many times a second, so later refusals stay quiet until a point is tracked again.
	const auto Refuse = [this, Component, &OutError](bool& bLogged, const FString& Why) {
		OutError = Why;
		if (!bLogged)
		{
			bLogged = true;
			UE_LOG(LogRammsPhysicsImuSubsystem, Warning, TEXT("%s: not tracking an IMU point on %s: %s (further refusals are not logged until a point is tracked again)"),
				*GetNameSafe(GetWorld()), *Component->GetPathName(), *Why);
		}
		return 0;
	};
	if (Points.Num() >= MaxTrackedPoints)
	{
		return Refuse(bLoggedWorldFull, FString::Printf(TEXT("the world tracks %d IMU points already, the most it may"), MaxTrackedPoints));
	}
	if (bBridge && GetBridgePointCount() >= MaxBridgePoints)
	{
		return Refuse(bLoggedBridgeFull,
			FString::Printf(TEXT("Remote Control holds %d IMU points already, the most it may (points nobody reads are dropped after a "
								 "while; stop the IMU streams no longer needed)"),
				MaxBridgePoints));
	}
	bLoggedWorldFull = false;
	if (bBridge)
	{
		bLoggedBridgeFull = false;
	}
	FTrackedPoint Point;
	Point.bBridge = bBridge;
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

USceneComponent* URammsPhysicsImuSubsystem::GetTrackedBody(int32 Handle) const
{
	const FTrackedPoint* Point = Points.Find(Handle);
	return Point ? Point->Ref.Body.Get() : nullptr;
}

void URammsPhysicsImuSubsystem::EnableHistory(int32 Handle)
{
	FTrackedPoint* Point = Points.Find(Handle);
	if (!Point || Point->bHistory)
	{
		return;
	}
	Point->bHistory = true;
	if (Point->ProviderHandle != INDEX_NONE)
	{
		if (IRammsImuProvider* Provider = GetProvider(Point->Ref.Source))
		{
			Provider->SetHistoryCapacity(Point->ProviderHandle, HistoryCapacity);
		}
	}
}

bool URammsPhysicsImuSubsystem::ReadHistory(int32 Handle, int64 Since, int32 Max, FRammsImuHistoryRead& Out)
{
	Out = FRammsImuHistoryRead();
	FTrackedPoint* Point = Points.Find(Handle);
	if (!Point)
	{
		return false;
	}
	Point->LastRead = Now();
	EnableHistory(Handle);

	const TSharedPtr<IRammsImuProvider>* Provider = Providers.Find(Point->Ref.Source);
	int64								 First = 0;
	int64								 Oldest = 0;
	int64								 ProviderNext = 0;
	// A point between providers (none takes it just now) keeps its cursor.
	Out.Next = FMath::Max(Since, Point->SeqBase);
	if (!Provider || !Provider->IsValid() || Point->ProviderHandle == INDEX_NONE)
	{
		return true;
	}
	if (Since < 0)
	{
		// Start now: no samples, only where the next one will be.
		TArray<FRammsImuSample> None;
		if ((*Provider)->ReadHistory(Point->ProviderHandle, 0, 0, None, First, Oldest, ProviderNext))
		{
			Out.Next = Point->SeqBase + ProviderNext;
		}
		return true;
	}
	const int64 LocalSince = Since - Point->SeqBase; // below the provider's range: what was before is gone
	if (!(*Provider)->ReadHistory(Point->ProviderHandle, LocalSince, FMath::Clamp(Max, 0, HistoryCapacity), Out.Samples, First, Oldest,
			ProviderNext))
	{
		return true;
	}
	if (LocalSince > ProviderNext)
	{
		// A cursor ahead of this point (one from a point tracked again since): start now.
		Out.Samples.Reset();
		Out.Next = Point->SeqBase + ProviderNext;
		return true;
	}
	Out.FirstSeq = Point->SeqBase + First;
	Out.Next = Out.FirstSeq + Out.Samples.Num();
	Out.Dropped = FMath::Max<int64>(Out.FirstSeq - Since, 0);
	return true;
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
	TArray<FRammsImuBodyRef> MuJoCoBodies;
	if (const IRammsImuProvider* MuJoCo = GetProvider(ERammsImuSource::MuJoCo))
	{
		MuJoCo->ListBodies(Actor, MuJoCoBodies);
	}
	// A URLab body that Chaos simulates is listed by its URLab name; the Chaos body that carries it is not listed
	// again under its own.
	TSet<const USceneComponent*> CarriedByChaos;
	for (const FRammsImuBodyRef& Listed : MuJoCoBodies)
	{
		FRammsImuBodyRef Carrier;
		if (USceneComponent* Body = Listed.Body.Get(); Body && Listed.Source == ERammsImuSource::Chaos
			&& ResolveBody(*Body, NAME_None, FTransform::Identity, ERammsImuSource::Auto, Carrier) && Carrier.Source == ERammsImuSource::Chaos)
		{
			CarriedByChaos.Add(Carrier.Body.Get());
		}
	}
	TArray<FRammsImuBodyRef> ChaosBodies;
	if (const IRammsImuProvider* Chaos = GetProvider(ERammsImuSource::Chaos))
	{
		Chaos->ListBodies(Actor, ChaosBodies);
	}
	ChaosBodies.RemoveAll([&CarriedByChaos](const FRammsImuBodyRef& Listed) { return Listed.Bone.IsNone() && CarriedByChaos.Contains(Listed.Body.Get()); });

	// The actor's own root body leads (the Chaos provider lists it first when it simulates).
	const bool bChaosRootFirst = ChaosBodies.Num() > 0 && ChaosBodies[0].Body.Get() == Actor.GetRootComponent();
	if (bChaosRootFirst)
	{
		OutBodies.Add(ChaosBodies[0]);
		ChaosBodies.RemoveAt(0);
	}
	OutBodies.Append(MuJoCoBodies);
	OutBodies.Append(ChaosBodies);
}

int32 URammsPhysicsImuSubsystem::TrackWith(const FTrackedPoint& Point, const FRammsImuBodyRef& Ref)
{
	IRammsImuProvider* Provider = GetProvider(Ref.Source);
	const int32		   Handle = Provider ? Provider->Track(Ref) : INDEX_NONE;
	if (Handle != INDEX_NONE && Point.bHistory)
	{
		Provider->SetHistoryCapacity(Handle, HistoryCapacity);
	}
	return Handle;
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
	Reattach(Point, Ref, /*bForce=*/true);
}

void URammsPhysicsImuSubsystem::Reattach(FTrackedPoint& Point, const FRammsImuBodyRef& Ref, bool bForce)
{
	FRammsImuBodyRef Taken = Ref;
	int32			 Handle = TrackWith(Point, Taken);
	if (Handle == INDEX_NONE)
	{
		if (!bForce && Point.ProviderHandle != INDEX_NONE)
		{
			return; // that engine cannot take it (yet): the provider serving it keeps it
		}
		// The engine could not take it (no physics scene): differencing it is. A point already differenced on the
		// same frame keeps its differencer.
		USceneComponent*		 Component = Point.Component.Get();
		const IRammsImuProvider* Kinematic = GetProvider(ERammsImuSource::Kinematic);
		if (Ref.Source == ERammsImuSource::Kinematic || !Component || !Kinematic
			|| !Kinematic->Resolve(*Component, Point.Bone, Point.LocalPose, false, Taken))
		{
			Detach(Point);
			Point.Ref = Ref;
			return;
		}
		if (Point.ProviderHandle != INDEX_NONE && RammsPhysicsImuImpl::SameBody(Point.Ref, Taken))
		{
			return;
		}
		Handle = TrackWith(Point, Taken);
	}
	Detach(Point);
	Point.Ref = Taken;
	Point.ProviderHandle = Handle;
}

void URammsPhysicsImuSubsystem::Detach(FTrackedPoint& Point)
{
	if (Point.ProviderHandle != INDEX_NONE)
	{
		if (const TSharedPtr<IRammsImuProvider>* Provider = Providers.Find(Point.Ref.Source); Provider && Provider->IsValid())
		{
			// The point's sequence goes on where this provider's ends.
			TArray<FRammsImuSample> None;
			int64					First = 0;
			int64					Oldest = 0;
			int64					ProviderNext = 0;
			if (Point.bHistory && (*Provider)->ReadHistory(Point.ProviderHandle, 0, 0, None, First, Oldest, ProviderNext))
			{
				Point.SeqBase += ProviderNext;
			}
			(*Provider)->Untrack(Point.ProviderHandle);
		}
	}
	Point.ProviderHandle = INDEX_NONE;
}

void URammsPhysicsImuSubsystem::RefreshPoints(bool bForceResolve)
{
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
		USceneComponent* Component = Point.Component.Get();

		// The body it rode on is gone or no longer the one it was resolved on: resolve it again at once.
		const TSharedPtr<IRammsImuProvider>* Provider = Providers.Find(Point.Ref.Source);
		const bool							 bLost = Point.ProviderHandle == INDEX_NONE || !Provider || !Provider->IsValid() || !(*Provider)->IsTrackValid(Point.ProviderHandle);
		if (bLost && (bForceResolve || WallNow >= Point.NextInvalidRetry))
		{
			Point.NextInvalidRetry = WallNow + RammsPhysicsImuImpl::ResolveRetrySeconds;
			Point.NextResolve = WallNow + RammsPhysicsImuImpl::ResolveRetrySeconds;
			FRammsImuBodyRef Ref;
			ResolveBody(*Component, Point.Bone, Point.LocalPose, Point.Preferred, Ref);
			Reattach(Point, Ref, /*bForce=*/true);
			continue;
		}

		// Now and then: does another engine (or another body) serve it now? MuJoCo compiles after BeginPlay, a Chaos
		// articulation builds on its first tick, a robot is switched to Chaos, a body is welded into another.
		if (Point.Preferred == ERammsImuSource::Kinematic || (!bForceResolve && WallNow < Point.NextResolve))
		{
			continue;
		}
		Point.NextResolve = WallNow + RammsPhysicsImuImpl::ResolveRetrySeconds;
		FRammsImuBodyRef Ref;
		ResolveBody(*Component, Point.Bone, Point.LocalPose, Point.Preferred, Ref);
		if (!RammsPhysicsImuImpl::SameBody(Ref, Point.Ref))
		{
			Reattach(Point, Ref, /*bForce=*/false);
		}
	}
}

void URammsPhysicsImuSubsystem::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	RefreshPoints(/*bForceResolve=*/false);

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
