// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRobotControlSurfaceComponent.h"
#include "Components/ChildActorComponent.h"
#include "RammsControlContributor.h"
#include "RammsRobotBaseComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "JsonObjectConverter.h"
#include "TimerManager.h"
#include "RammsControlSurfaceRegistry.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsControlSurface, Log, All);

URammsRobotControlSurfaceComponent::URammsRobotControlSurfaceComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URammsRobotControlSurfaceComponent::BeginPlay()
{
	Super::BeginPlay();
	if (AActor* Owner = GetOwner())
	{
		Base = Owner->FindComponentByClass<URammsRobotBaseComponent>();
		if (Base)
		{
			Base->OnMotorRegistryLoaded.AddUniqueDynamic(this, &URammsRobotControlSurfaceComponent::OnRegistryLoaded);
		}
	}
	// Sibling contributors resolve their own dependencies in their BeginPlay,
	// in no guaranteed order relative to ours: build once everyone has run.
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().SetTimerForNextTick(this, &URammsRobotControlSurfaceComponent::RebuildControlSurface);
		if (URammsControlSurfaceRegistry* Registry = World->GetSubsystem<URammsControlSurfaceRegistry>())
		{
			Registry->RegisterControlSurface(this);
		}
	}
}

TArray<UObject*> URammsRobotControlSurfaceComponent::FindControlSurfaces(const UObject* WorldContextObject)
{
	if (const UWorld* World = WorldContextObject ? WorldContextObject->GetWorld() : nullptr)
	{
		if (URammsControlSurfaceRegistry* Registry = World->GetSubsystem<URammsControlSurfaceRegistry>())
		{
			return Registry->GetAllControlSurfaces();
		}
	}
	return {};
}

void URammsRobotControlSurfaceComponent::RequireCentre(FName Id)
{
	if (!Id.IsNone())
	{
		++CentreRequests.FindOrAdd(Id, 0);
	}
}

int32 URammsRobotControlSurfaceComponent::GetCentreRequests(FName Id) const
{
	const int32* Count = CentreRequests.Find(Id);
	return Count ? *Count : 0;
}

void URammsRobotControlSurfaceComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopDeadmanTicker();
	if (UWorld* World = GetWorld())
	{
		if (URammsControlSurfaceRegistry* Registry = World->GetSubsystem<URammsControlSurfaceRegistry>())
		{
			Registry->UnregisterControlSurface(this);
		}
	}
	Super::EndPlay(EndPlayReason);
}

void URammsRobotControlSurfaceComponent::OnRegistryLoaded(URammsRobotBaseComponent* /*InBase*/)
{
	if (bBuilt)
	{
		RebuildControlSurface();
	}
}

void URammsRobotControlSurfaceComponent::EnsureBuilt() const
{
	if (!bBuilt)
	{
		const_cast<URammsRobotControlSurfaceComponent*>(this)->RebuildControlSurface();
	}
}

URammsRobotControlSurfaceComponent* URammsRobotControlSurfaceComponent::FindGoverningSurface(
	const UActorComponent* Component)
{
	const AActor* Actor = Component ? Component->GetOwner() : nullptr;
	// Bounded as the gather is, and for the same reason: this walks spawned
	// actors, so a chain that loops back has to stop somewhere.
	for (int32 Depth = 0; Actor != nullptr && Depth <= MaxChildActorGatherDepth; ++Depth)
	{
		if (URammsRobotControlSurfaceComponent* Surface =
				Actor->FindComponentByClass<URammsRobotControlSurfaceComponent>())
		{
			// Only if that surface would actually gather from this far down.
			// Answering with one that does not include the caller is worse than
			// answering with nothing: its commands would reach a surface whose
			// Routes never mention them, so they would quietly do nothing -- or,
			// where an id collides, act on a different actor's control.
			if (Depth == 0)
			{
				return Surface;
			}
			const int32 Reach = Surface->bGatherFromChildActors
				? FMath::Clamp(Surface->ChildActorGatherDepth, 0, MaxChildActorGatherDepth)
				: 0;
			return Depth <= Reach ? Surface : nullptr;
		}
		Actor = Actor->GetParentActor();
	}
	return nullptr;
}

void URammsRobotControlSurfaceComponent::GatherContributorComponents(AActor* Actor, int32 Depth,
	TSet<AActor*>& Visited, TArray<UActorComponent*>& OutComponents) const
{
	if (!Actor || Visited.Contains(Actor))
	{
		return;
	}
	Visited.Add(Actor);

	// GetComponents empties what it is given, so this cannot gather straight
	// into the caller's array.
	TArray<UActorComponent*> Own;
	Actor->GetComponents(Own);
	OutComponents.Append(Own);

	// Clamped, not trusted: the depth is a serialized BlueprintReadWrite field
	// and its ClampMax reaches only the details panel.
	const int32 MaxDepth = FMath::Clamp(ChildActorGatherDepth, 0, MaxChildActorGatherDepth);
	if (!bGatherFromChildActors || Depth >= MaxDepth)
	{
		return;
	}

	for (UActorComponent* C : Own)
	{
		UChildActorComponent* AsChild = Cast<UChildActorComponent>(C);
		AActor*				  Child = AsChild ? AsChild->GetChildActor() : nullptr;
		if (!Child)
		{
			// Null before the child actor is created. The surface is built a
			// tick after BeginPlay precisely so composition has settled, and
			// RebuildControlSurface can be called again if one appears later.
			continue;
		}
		if (Child->FindComponentByClass<URammsRobotControlSurfaceComponent>())
		{
			// It publishes itself. Taking its controls as well would put each
			// of them on two surfaces, where a release through one would leave
			// the other still holding.
			continue;
		}
		GatherContributorComponents(Child, Depth + 1, Visited, OutComponents);
	}
}

void URammsRobotControlSurfaceComponent::RebuildControlSurface()
{
	// Who owned each control before this rebuild. A drive mode switch keeps
	// the Ids -- both modes offer drive.forward -- while changing the
	// contributor behind them, and both modes come up at zero. Carrying the
	// old hold and target across would have the surface report the previous
	// mode's command and refuse a lower-priority source over a robot that is
	// not moving.
	TMap<FName, TWeakObjectPtr<UObject>> PreviousOwners;
	PreviousOwners.Reserve(Routes.Num());
	for (const TPair<FName, FRoute>& Pair : Routes)
	{
		PreviousOwners.Add(Pair.Key, Pair.Value.Contributor);
	}

	Surface = FRammsControlSurface();
	Routes.Reset();
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}
	Surface.RobotName = RobotDisplayName.IsEmpty() ? FText::FromString(Owner->GetActorNameOrLabel()) : RobotDisplayName;

	// Every contributor on the actor, by interface — nothing wired by name —
	// and, unless turned off, on the actors it carries as child actors.
	TArray<UActorComponent*> Components;
	TSet<AActor*>			 Visited;
	GatherContributorComponents(Owner, 0, Visited, Components);
	TArray<TPair<IRammsControlContributor*, UObject*>> Contributors;
	for (UActorComponent* C : Components)
	{
		if (C && C != this && C->GetClass()->ImplementsInterface(URammsControlContributor::StaticClass()))
		{
			if (IRammsControlContributor* I = Cast<IRammsControlContributor>(C))
			{
				Contributors.Emplace(I, C);
			}
		}
	}
	Contributors.Sort([](const TPair<IRammsControlContributor*, UObject*>& A, const TPair<IRammsControlContributor*, UObject*>& B) {
		return A.Key->GetControlOrder() < B.Key->GetControlOrder();
	});

	TSet<FName> Claimed;
	for (const TPair<IRammsControlContributor*, UObject*>& Pair : Contributors)
	{
		FRammsControlSurface Part;
		Pair.Key->DescribeControls(Part);
		for (const FRammsControlAxis& Axis : Part.Axes)
		{
			if (Routes.Contains(Axis.Id))
			{
				// Naming the actor, not just the component. Ids already carry
				// the contributor's component name, so a collision needs two
				// contributors named alike -- which, now that carried actors
				// are gathered too, most likely means two copies of the same
				// child actor rather than a mistake on this one. Without the
				// actor in the message the two are indistinguishable.
				const AActor* From = Pair.Value->GetTypedOuter<AActor>();
				UE_LOG(LogRammsControlSurface, Warning,
					TEXT("[ControlSurface] '%s': control '%s' contributed twice (second from '%s' on '%s') — first wins. ")
						TEXT("Control ids must be unique across everything this surface gathers, including carried actors. ")
							TEXT("Some contributors derive ids from their component name and can be renamed; others publish fixed ")
								TEXT("ids (a differential drive always publishes drive.forward), and those need the carried actor to ")
									TEXT("have its own control surface so it publishes separately."),
					*Owner->GetName(), *Axis.Id.ToString(), *Pair.Value->GetName(),
					From ? *From->GetName() : TEXT("?"));
				continue;
			}
			FRammsControlAxis Normalized = Axis;
			if (Normalized.IsAction())
			{
				// Actions have no value to read back, whatever the contributor left set.
				Normalized.bReadback = false;
			}
			else if (Normalized.IsText())
			{
				// Words to read, never a command, whatever the contributor left set.
				Normalized.bReadOnly = true;
			}
			Surface.Add(Normalized);
			FRoute Route;
			Route.Contributor = Pair.Value;
			Routes.Add(Axis.Id, Route);
		}
		TArray<FName> Ids;
		Pair.Key->GetClaimedMotorIds(Ids);
		Claimed.Append(Ids);
	}

	// Raw axes for whatever the registry has that nobody drives.
	if (bExposeUnclaimedMotors && Base)
	{
		int32 Order = 0;
		for (const FRammsMotorSpec& Spec : Base->GetMotorSpecs())
		{
			if (Claimed.Contains(Spec.Id))
			{
				continue;
			}
			FRammsControlAxis Axis;
			Axis.Id = *FString::Printf(TEXT("motor.%s"), *Spec.Id.ToString());
			Axis.Group = UnclaimedMotorGroup;
			Axis.DisplayName = FText::FromName(Spec.Id);
			Axis.Order = Order++;
			// The registry's ControlRange is the command clamp; an unset one
			// (min >= max) means "defer to the backend" and stays unbounded here
			// too — SetAxis clamps to Axis.Range, so substituting limits would
			// refuse commands the backend accepts. Panels pick a display range
			// for an unbounded axis themselves.
			Axis.Range = Spec.ControlRange;
			switch (Spec.Type)
			{
				case ERammsActuatorType::Position:
					Axis.Kind = ERammsControlKind::Position;
					Axis.Units = ERammsControlUnits::Radians;
					break;
				case ERammsActuatorType::Velocity:
					Axis.Kind = ERammsControlKind::Velocity;
					Axis.Units = ERammsControlUnits::RadiansPerSecond;
					break;
				default:
					Axis.Kind = ERammsControlKind::Continuous;
					Axis.Units = ERammsControlUnits::None; // backend units (N·m on MuJoCo)
					break;
			}
			Surface.Add(Axis);
			FRoute Route;
			Route.MotorId = Spec.Id;
			Routes.Add(Axis.Id, Route);
		}
	}

	// Anything held by (or targeted through) a control that no longer exists,
	// or that has changed hands, is forgotten.
	auto IsStale = [this, &PreviousOwners](FName Id) {
		const FRoute* Route = Routes.Find(Id);
		if (!Route)
		{
			return true;
		}
		const TWeakObjectPtr<UObject>* Before = PreviousOwners.Find(Id);
		return Before != nullptr && Before->Get() != Route->Contributor.Get();
	};
	for (auto It = Holds.CreateIterator(); It; ++It)
	{
		if (IsStale(It.Key()))
		{
			It.RemoveCurrent();
		}
	}
	for (auto It = Targets.CreateIterator(); It; ++It)
	{
		if (IsStale(It.Key()))
		{
			It.RemoveCurrent();
		}
	}
	bBuilt = true;
	++Version;
	UE_LOG(LogRammsControlSurface, Log, TEXT("[ControlSurface] '%s': %d controls in %d groups from %d contributors (%d raw motors); version %d."),
		*Owner->GetName(), Surface.Axes.Num(), Surface.Groups.Num(), Contributors.Num(),
		Surface.Axes.FilterByPredicate([this](const FRammsControlAxis& A) { return A.Group == UnclaimedMotorGroup; }).Num(), Version);
}

// --- arbitration ------------------------------------------------------------

int32 URammsRobotControlSurfaceComponent::Priority(ERammsControlSource Source)
{
	switch (Source)
	{
		case ERammsControlSource::Autonomy:
			return 3;
		case ERammsControlSource::Remote:
			return 2;
		case ERammsControlSource::Keyboard:
		case ERammsControlSource::Gamepad:
		case ERammsControlSource::Touch:
			return 1;
		default:
			return 0;
	}
}

bool URammsRobotControlSurfaceComponent::MayDrive(FName Id, ERammsControlSource Source) const
{
	const FHold* H = Holds.Find(Id);
	if (!H || H->Source == Source)
	{
		return true;
	}
	// A higher-priority holder keeps the control for ExternalHoldSeconds
	// after its last command; equal or lower priority yields at once.
	if (Priority(H->Source) > Priority(Source) && ClockNow() - H->Time < GetExternalHoldSecondsClamped())
	{
		return false;
	}
	return true;
}

void URammsRobotControlSurfaceComponent::Hold(FName Id, ERammsControlSource Source, double Time)
{
	FHold& H = Holds.FindOrAdd(Id);
	H.Source = Source;
	H.Time = Time;
}

float URammsRobotControlSurfaceComponent::GetExternalHoldSecondsClamped() const
{
	// Clamped where used: the field is Blueprint-writable (and so writable over
	// Remote Control's property route), and ClampMin reaches only the details
	// panel. A NaN clamps to the top.
	return FMath::Clamp(ExternalHoldSeconds, MinHoldSeconds, MaxHoldSeconds);
}

float URammsRobotControlSurfaceComponent::GetDeadmanSecondsClamped() const
{
	return FMath::Clamp(DeadmanSeconds, MinHoldSeconds, MaxHoldSeconds);
}

double URammsRobotControlSurfaceComponent::ClockNow() const
{
	return ClockOverride ? ClockOverride() : FPlatformTime::Seconds();
}

void URammsRobotControlSurfaceComponent::SetClockForTesting(TFunction<double()> Clock)
{
	ClockOverride = MoveTemp(Clock);
}

bool URammsRobotControlSurfaceComponent::GetHoldTime(FName Id, double& OutTime) const
{
	const FHold* H = Holds.Find(Id);
	OutTime = H ? H->Time : 0.0;
	return H != nullptr;
}

// --- the drive deadman ----------------------------------------------------------

bool URammsRobotControlSurfaceComponent::IsDeadmanSource(ERammsControlSource Source)
{
	return Source == ERammsControlSource::Remote || Source == ERammsControlSource::Autonomy;
}

bool URammsRobotControlSurfaceComponent::IsDeadmanKind(ERammsControlKind Kind)
{
	return Kind == ERammsControlKind::Continuous || Kind == ERammsControlKind::Velocity;
}

void URammsRobotControlSurfaceComponent::NoteHeard(ERammsControlSource Source)
{
	if (Source == ERammsControlSource::Remote)
	{
		HeardRemote = ClockNow();
	}
	else if (Source == ERammsControlSource::Autonomy)
	{
		HeardAutonomy = ClockNow();
	}
}

double URammsRobotControlSurfaceComponent::LastHeard(ERammsControlSource Source) const
{
	return Source == ERammsControlSource::Remote  ? HeardRemote
		: Source == ERammsControlSource::Autonomy ? HeardAutonomy
												  : 0.0;
}

void URammsRobotControlSurfaceComponent::UpdateDeadman(FName Id, ERammsControlKind Kind, ERammsControlSource Source, ECommandOrigin Origin)
{
	if (Origin != ECommandOrigin::Robot && IsDeadmanKind(Kind) && IsDeadmanSource(Source))
	{
		DeadmanWatch.Add(Id);
		StartDeadmanTicker();
	}
	else
	{
		// Taken over by a source whose commands stand, by the robot itself, or a
		// control whose commands stand.
		DeadmanWatch.Remove(Id);
	}
}

void URammsRobotControlSurfaceComponent::StartDeadmanTicker()
{
	if (DeadmanTicker.IsValid())
	{
		return;
	}
	// Its first frame is counted from the command that started it, so a first
	// run a frame late (or after a hitch) is given that frame's grace.
	LastDeadmanCheck = ClockNow();
	PreviousDeadmanGap = 0.0;
	// The core ticker rather than this component's tick: it runs while the
	// world is paused, as the real-time holds it times do, and it needs no
	// tick function on a component Blueprints already carry without one.
	TWeakObjectPtr<URammsRobotControlSurfaceComponent> WeakThis(this);
	DeadmanTicker = FTSTicker::GetCoreTicker().AddTicker(TEXT("RammsControlSurfaceDeadman"), 0.0f,
		[WeakThis](float) { return StepDeadmanTicker(WeakThis); });
}

void URammsRobotControlSurfaceComponent::StopDeadmanTicker()
{
	if (DeadmanTicker.IsValid())
	{
		// Safe from inside the entry's own callback (the core ticker only marks
		// an entry that is running on this thread).
		FTSTicker::RemoveTicker(DeadmanTicker);
	}
	DeadmanTicker.Reset();
}

bool URammsRobotControlSurfaceComponent::StepDeadmanTicker(const TWeakObjectPtr<URammsRobotControlSurfaceComponent>& WeakSurface)
{
	URammsRobotControlSurfaceComponent* Self = WeakSurface.Get();
	if (Self == nullptr)
	{
		return false; // gone (destroyed or collected): the entry goes with it
	}
	if (Self->TickDeadman(Self->ClockNow()))
	{
		return true;
	}
	// Nothing left to watch: stop until the next deadman command. Removing the
	// entry here, not just returning false, so a test running this by hand
	// leaves no second entry behind once a new command starts one.
	Self->StopDeadmanTicker();
	return false;
}

bool URammsRobotControlSurfaceComponent::TickDeadman(double Now)
{
	// The grace: the last DeadmanGraceFrames frames. A source is heard at frame
	// granularity (a Remote Control request is answered in the core ticker, a
	// RammsAccess intent in the world's tick), and within a frame the deadman
	// may look before or after it is heard, so a source is only silent once it
	// has missed more than a frame or two past the window.
	static_assert(DeadmanGraceFrames == 2, "the grace below sums the last two frames");
	const double Gap = FMath::Max(0.0, Now - LastDeadmanCheck);
	const double Grace = Gap + PreviousDeadmanGap;
	PreviousDeadmanGap = Gap;
	LastDeadmanCheck = Now;
	ExpireQuietHolds(Now, Grace);
	return DeadmanWatch.Num() > 0;
}

int32 URammsRobotControlSurfaceComponent::ExpireQuietHolds(double Now, double Grace)
{
	const double Window = static_cast<double>(GetDeadmanSecondsClamped()) + FMath::Max(0.0, Grace);

	// Find the quiet ones first and spring them back after: a release can
	// rebuild the surface (the drive-mode selector's contributors do), which
	// changes Holds and Surface under an iterator.
	struct FQuiet
	{
		FName				Id;
		ERammsControlSource Source;
		ERammsControlKind	Kind;
	};
	TArray<FQuiet, TInlineAllocator<8>> Quiet;
	double								Longest = 0.0;
	for (auto It = DeadmanWatch.CreateIterator(); It; ++It)
	{
		const FName				 Id = *It;
		const FHold*			 H = Holds.Find(Id);
		const FRammsControlAxis* Axis = Surface.Find(Id);
		if (H == nullptr || !IsDeadmanSource(H->Source) || Axis == nullptr || !IsDeadmanKind(Axis->Kind))
		{
			// Released, taken over by a source whose commands stand, dropped by a
			// rebuild (it changed hands), or no longer a kind that lapses.
			It.RemoveCurrent();
			continue;
		}
		// Per source: anything heard from it since keeps its commands.
		const double Quietness = Now - FMath::Max(H->Time, LastHeard(H->Source));
		if (Quietness >= Window)
		{
			Longest = FMath::Max(Longest, Quietness);
			Quiet.Add({ Id, H->Source, Axis->Kind });
			It.RemoveCurrent();
		}
	}
	if (Quiet.Num() == 0)
	{
		return 0;
	}

	int32			SprungBack = 0;
	TArray<FString> Sprung;
	TArray<FString> Refused;
	for (const FQuiet& Entry : Quiet)
	{
		// A spring-back before this one may have rebuilt the surface: a control
		// that is gone, or has changed hands, is forgotten, not sprung back.
		const FHold*			 H = Holds.Find(Entry.Id);
		const FRammsControlAxis* Axis = Surface.Find(Entry.Id);
		if (H == nullptr || H->Source != Entry.Source || Axis == nullptr || Axis->Kind != Entry.Kind)
		{
			continue;
		}
		// Continuous: as the source itself letting go would (the holder may
		// always release). Velocity: stopped -- commanded its default -- rather
		// than let go of. A refusal leaves the hold to lapse as any hold does;
		// not retried.
		const bool	  bBack = Entry.Kind == ERammsControlKind::Velocity ? StopHeldVelocity(Entry.Id) : ReleaseHeld(Entry.Id, Entry.Source);
		const FString Name = FString::Printf(TEXT("%s (%s)"), *Entry.Id.ToString(),
			Entry.Source == ERammsControlSource::Remote ? TEXT("Remote") : TEXT("Autonomy"));
		if (bBack)
		{
			++SprungBack;
			Sprung.Add(Name);
		}
		else
		{
			Refused.Add(Name);
		}
	}

	// At Log (refusals at Warning) at most every DeadmanLogRepeatSeconds per
	// surface; in between, Verbose: a client that keeps dropping in and out,
	// or the robot's own holds lapsing every crossing, is not news each time.
	const bool bReport = LastDeadmanReport < 0.0 || Now - LastDeadmanReport >= DeadmanLogRepeatSeconds;
	if (bReport && (Sprung.Num() > 0 || Refused.Num() > 0))
	{
		LastDeadmanReport = Now;
	}
	const FString OwnerName = GetNameSafe(GetOwner());
	if (Sprung.Num() > 0)
	{
		const FString List = FString::Join(Sprung, TEXT(", "));
		if (bReport)
		{
			UE_LOG(LogRammsControlSurface, Log, TEXT("[ControlSurface] '%s': deadman: nothing heard for %.2f s (window %.2f s): %s sprang back."),
				*OwnerName, Longest, Window, *List);
		}
		else
		{
			UE_LOG(LogRammsControlSurface, Verbose, TEXT("[ControlSurface] '%s': deadman: nothing heard for %.2f s (window %.2f s): %s sprang back."),
				*OwnerName, Longest, Window, *List);
		}
	}
	if (Refused.Num() > 0)
	{
		const FString List = FString::Join(Refused, TEXT(", "));
		if (bReport)
		{
			UE_LOG(LogRammsControlSurface, Warning,
				TEXT("[ControlSurface] '%s': deadman: could not spring back %s (refused by what drives it); the last command may still be acting."),
				*OwnerName, *List);
		}
		else
		{
			UE_LOG(LogRammsControlSurface, Verbose,
				TEXT("[ControlSurface] '%s': deadman: could not spring back %s (refused by what drives it); the last command may still be acting."),
				*OwnerName, *List);
		}
	}
	return SprungBack;
}

bool URammsRobotControlSurfaceComponent::StopHeldVelocity(FName Id)
{
	const FRammsControlAxis* Found = Surface.Find(Id);
	const FRoute*			 Route = Routes.Find(Id);
	if (!Found || !Route)
	{
		return false;
	}
	// By value: ApplyControl may rebuild the surface (see ApplyAxis).
	const float Default = Found->DefaultValue;
	const FName MotorId = Route->MotorId;
	bool		bStopped = false;
	if (IRammsControlContributor* C = ContributorFor(Id))
	{
		bStopped = C->ApplyControl(Id, Default);
	}
	else if (Base && Base->HasBackend() && !MotorId.IsNone())
	{
		Base->SetMotorCommand(MotorId, Default);
		bStopped = true;
	}
	if (bStopped)
	{
		Holds.Remove(Id);
		DeadmanWatch.Remove(Id);
		Targets.Add(Id, Default); // what it is commanded now
	}
	return bStopped;
}

IRammsControlContributor* URammsRobotControlSurfaceComponent::ContributorFor(FName Id) const
{
	const FRoute* Route = Routes.Find(Id);
	return (Route && Route->Contributor.IsValid()) ? Cast<IRammsControlContributor>(Route->Contributor.Get()) : nullptr;
}

// --- sink ---------------------------------------------------------------------

TArray<UActorComponent*> URammsRobotControlSurfaceComponent::GetContributorComponents() const
{
	EnsureBuilt();
	TArray<UActorComponent*> Out;
	for (const TPair<FName, FRoute>& Pair : Routes)
	{
		if (UActorComponent* Comp = Cast<UActorComponent>(Pair.Value.Contributor.Get()))
		{
			Out.AddUnique(Comp);
		}
	}
	return Out;
}

bool URammsRobotControlSurfaceComponent::SetAxis_Implementation(FName Id, float Value, ERammsControlSource Source)
{
	return ApplyAxis(Id, Value, Source, ECommandOrigin::Client, 0.0);
}

bool URammsRobotControlSurfaceComponent::SetControlInternal(FName Id, float Value, ERammsControlSource Source)
{
	return ApplyAxis(Id, Value, Source, ECommandOrigin::Robot, 0.0);
}

bool URammsRobotControlSurfaceComponent::ReapplyHeldControl(FName Id, float Value, ERammsControlSource Source, double HoldTime)
{
	return ApplyAxis(Id, Value, Source, ECommandOrigin::Carried, HoldTime);
}

bool URammsRobotControlSurfaceComponent::ApplyAxis(FName Id, float Value, ERammsControlSource Source, ECommandOrigin Origin, double HoldTime)
{
	EnsureBuilt();
	const FRammsControlAxis* Found = Surface.Find(Id);
	const FRoute*			 Route = Routes.Find(Id);
	if (!Found || !Route || Found->IsAction() || Found->bReadOnly || !MayDrive(Id, Source))
	{
		return false;
	}
	// By value, and the route resolved now: a contributor may rebuild the
	// surface from inside ApplyControl -- the drive-mode selector does exactly
	// that -- which reallocates Surface.Axes and rehashes Routes under us.
	const FRammsControlAxis Axis = *Found;
	const FName				MotorId = Route->MotorId;
	const float				Clamped = Axis.Clamp(Value);
	bool					bApplied = false;
	if (IRammsControlContributor* C = ContributorFor(Id))
	{
		bApplied = C->ApplyControl(Id, Clamped);
	}
	else if (Base && Base->HasBackend() && !MotorId.IsNone())
	{
		// Without a backend SetMotorCommand is a silent no-op: not applied.
		Base->SetMotorCommand(MotorId, Clamped);
		bApplied = true;
	}
	if (bApplied)
	{
		// A carried command keeps the time it was given (the source has said
		// nothing since); anything else is held from now.
		Hold(Id, Source, Origin == ECommandOrigin::Carried ? HoldTime : ClockNow());
		Targets.Add(Id, Clamped);
		UpdateDeadman(Id, Axis.Kind, Source, Origin);
		if (Origin == ECommandOrigin::Client)
		{
			NoteHeard(Source);
		}
	}
	return bApplied;
}

bool URammsRobotControlSurfaceComponent::TriggerAction_Implementation(FName Id, ERammsControlSource Source)
{
	const bool bTriggered = TriggerAxis(Id, Source);
	if (bTriggered)
	{
		NoteHeard(Source);
	}
	return bTriggered;
}

bool URammsRobotControlSurfaceComponent::TriggerControlInternal(FName Id, ERammsControlSource Source)
{
	return TriggerAxis(Id, Source);
}

bool URammsRobotControlSurfaceComponent::TriggerAxis(FName Id, ERammsControlSource Source)
{
	EnsureBuilt();
	const FRammsControlAxis* Axis = Surface.Find(Id);
	// An Action fires; an Enum steps to its next choice where its owner offers
	// that -- the drive-mode selector's, for a controller's mode button.
	const bool bTriggerable = Axis && (Axis->IsAction() || (Axis->Kind == ERammsControlKind::Enum && !Axis->bReadOnly));
	if (!bTriggerable || !MayDrive(Id, Source))
	{
		return false;
	}
	IRammsControlContributor* C = ContributorFor(Id);
	return C && C->TriggerControl(Id);
}

bool URammsRobotControlSurfaceComponent::ReleaseAxis_Implementation(FName Id, ERammsControlSource Source)
{
	const bool bReleased = ReleaseHeld(Id, Source);
	if (bReleased)
	{
		NoteHeard(Source);
	}
	return bReleased;
}

bool URammsRobotControlSurfaceComponent::ReleaseHeld(FName Id, ERammsControlSource Source)
{
	EnsureBuilt();
	const FRammsControlAxis* Found = Surface.Find(Id);
	const FRoute*			 Route = Routes.Find(Id);
	if (!Found || !Route || Found->IsAction())
	{
		return false;
	}
	// By value: ReleaseControl may rebuild the surface, and everything below
	// reads the axis again afterwards. See ApplyAxis.
	const FRammsControlAxis Axis = *Found;
	const FName				MotorId = Route->MotorId;
	// Only the holder (or nobody) releases; a lower-priority source can't
	// release what an autonomy client is driving.
	if (const FHold* H = Holds.Find(Id))
	{
		if (H->Source != Source && !MayDrive(Id, Source))
		{
			return false;
		}
	}
	bool bReleased = false;
	if (IRammsControlContributor* C = ContributorFor(Id))
	{
		bReleased = C->ReleaseControl(Id);
		if (!bReleased && Axis.Kind == ERammsControlKind::Continuous)
		{
			bReleased = C->ApplyControl(Id, Axis.DefaultValue); // spring back
		}
	}
	else if (Base && Base->HasBackend() && !MotorId.IsNone())
	{
		if (Axis.Kind == ERammsControlKind::Continuous)
		{
			Base->SetMotorCommand(MotorId, Axis.DefaultValue); // spring back
			bReleased = true;
		}
		else
		{
			bReleased = Base->ReleaseMotor(MotorId);
		}
	}
	if (bReleased)
	{
		Holds.Remove(Id);
		DeadmanWatch.Remove(Id);

		// Some controls cannot be let go of one at a time. A 5-bar's height and
		// fore/aft are two axes over one pair of motors, so releasing either
		// releases the endpoint itself -- and whoever held the other axis would
		// otherwise keep its ownership entry over a target nobody is holding.
		// Ask the contributor: any of its axes that now reports no target has
		// been released too.
		if (IRammsControlContributor* C = ContributorFor(Id))
		{
			for (const FRammsControlAxis& Other : Surface.Axes)
			{
				// Position and Velocity only. A false from ReadTarget is
				// authoritative for those -- the contributor is saying it holds
				// no target -- but it is also the default implementation, so a
				// Continuous contributor that never overrode it would look like
				// it had released everything. Releasing drive.forward would
				// then drop the holds on strafe and turn while both are still
				// being commanded.
				const bool bServo = Other.Kind == ERammsControlKind::Position
					|| Other.Kind == ERammsControlKind::Velocity;
				if (Other.Id == Id || !bServo || ContributorFor(Other.Id) != C)
				{
					continue;
				}
				float Unused = 0.0f;
				if (!C->ReadTarget(Other.Id, Unused))
				{
					Holds.Remove(Other.Id);
					Targets.Remove(Other.Id);
				}
			}
		}

		if (Axis.Kind == ERammsControlKind::Continuous)
		{
			Targets.Add(Id, Axis.DefaultValue); // sprung back: that is the target now
		}
		else
		{
			Targets.Remove(Id); // no longer holding a target
		}
	}
	return bReleased;
}

float URammsRobotControlSurfaceComponent::GetAxisValue_Implementation(FName Id) const
{
	EnsureBuilt();
	const FRammsControlAxis* Axis = Surface.Find(Id);
	const FRoute*			 Route = Routes.Find(Id);
	if (!Axis || !Route)
	{
		return 0.0f;
	}
	float Value = Axis->DefaultValue;
	if (const IRammsControlContributor* C = ContributorFor(Id))
	{
		C->ReadControl(Id, Value);
	}
	else if (Base && !Route->MotorId.IsNone())
	{
		Value = Base->GetMotorValue(Route->MotorId);
	}
	return Value;
}

ERammsControlSource URammsRobotControlSurfaceComponent::GetAxisOwner_Implementation(FName Id) const
{
	const FHold* H = Holds.Find(Id);
	return H ? H->Source : ERammsControlSource::Script;
}

// --- provider + twins --------------------------------------------------------

FRammsControlSurface URammsRobotControlSurfaceComponent::GetControlSurface_Implementation() const
{
	EnsureBuilt();
	return Surface;
}

int32 URammsRobotControlSurfaceComponent::GetControlSurfaceVersion_Implementation() const
{
	EnsureBuilt();
	return Version;
}

FRammsControlSurface URammsRobotControlSurfaceComponent::DescribeControlSurface() const
{
	return GetControlSurface_Implementation();
}

bool URammsRobotControlSurfaceComponent::SetControl(FName Id, float Value, ERammsControlSource Source)
{
	return SetAxis_Implementation(Id, Value, Source);
}

bool URammsRobotControlSurfaceComponent::TriggerControl(FName Id, ERammsControlSource Source)
{
	return TriggerAction_Implementation(Id, Source);
}

bool URammsRobotControlSurfaceComponent::ReleaseControl(FName Id, ERammsControlSource Source)
{
	return ReleaseAxis_Implementation(Id, Source);
}

float URammsRobotControlSurfaceComponent::GetControlValue(FName Id) const
{
	return GetAxisValue_Implementation(Id);
}

FString URammsRobotControlSurfaceComponent::GetControlSurfaceJson() const
{
	EnsureBuilt();
	FString Out;
	FJsonObjectConverter::UStructToJsonObjectString(Surface, Out);
	return Out;
}

bool URammsRobotControlSurfaceComponent::GetAxisTarget_Implementation(FName Id, float& OutTarget) const
{
	EnsureBuilt(); // may be the first surface call a panel makes: routes must exist for the lookup
	const FRammsControlAxis* Axis = Surface.Find(Id);
	// The owning contributor knows the real target, including one set by a
	// direct call on the controller that never passed through the surface.
	if (IRammsControlContributor* C = ContributorFor(Id))
	{
		if (C->ReadTarget(Id, OutTarget))
		{
			return true;
		}
		// For a servo axis the contributor's "no target" is the answer: it has
		// been released (or disabled behind our back), and our own record of
		// the last command would report a target nothing is holding. Rate
		// axes and contributors that don't track targets fall through.
		if (Axis && (Axis->Kind == ERammsControlKind::Position || Axis->Kind == ERammsControlKind::Velocity))
		{
			OutTarget = 0.0f;
			return false;
		}
	}
	if (const float* Target = Targets.Find(Id))
	{
		OutTarget = *Target;
		return true;
	}
	OutTarget = 0.0f;
	return false;
}

bool URammsRobotControlSurfaceComponent::GetControlTarget(FName Id, float& OutTarget) const
{
	return GetAxisTarget_Implementation(Id, OutTarget);
}

bool URammsRobotControlSurfaceComponent::GetAxisText_Implementation(FName Id, FText& OutText) const
{
	EnsureBuilt();
	const FRammsControlAxis*		Axis = Surface.Find(Id);
	const IRammsControlContributor* C = Axis && Axis->IsText() ? ContributorFor(Id) : nullptr;
	if (C && C->ReadControlText(Id, OutText))
	{
		return true;
	}
	OutText = FText::GetEmpty();
	return false;
}

FString URammsRobotControlSurfaceComponent::GetControlText(FName Id) const
{
	FText Text;
	return GetAxisText_Implementation(Id, Text) ? Text.ToString() : FString();
}
