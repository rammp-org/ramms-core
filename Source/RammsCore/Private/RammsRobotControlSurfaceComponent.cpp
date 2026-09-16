// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRobotControlSurfaceComponent.h"
#include "RammsControlContributor.h"
#include "RammsRobotBaseComponent.h"
#include "GameFramework/Actor.h"
#include "JsonObjectConverter.h"
#include "TimerManager.h"
#include "RammsUISubsystem.h"

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
		if (URammsUISubsystem* UI = World->GetSubsystem<URammsUISubsystem>())
		{
			UI->RegisterControlSurface(this);
		}
	}
}

TArray<UObject*> URammsRobotControlSurfaceComponent::FindControlSurfaces(const UObject* WorldContextObject)
{
	if (const UWorld* World = WorldContextObject ? WorldContextObject->GetWorld() : nullptr)
	{
		if (URammsUISubsystem* UI = World->GetSubsystem<URammsUISubsystem>())
		{
			return UI->GetAllControlSurfaces();
		}
	}
	return {};
}

void URammsRobotControlSurfaceComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UWorld* World = GetWorld())
	{
		if (URammsUISubsystem* UI = World->GetSubsystem<URammsUISubsystem>())
		{
			UI->UnregisterControlSurface(this);
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

void URammsRobotControlSurfaceComponent::RebuildControlSurface()
{
	Surface = FRammsControlSurface();
	Routes.Reset();
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}
	Surface.RobotName = RobotDisplayName.IsEmpty() ? FText::FromString(Owner->GetActorNameOrLabel()) : RobotDisplayName;

	// Every contributor on the actor, by interface — nothing wired by name.
	TArray<UActorComponent*> Components;
	Owner->GetComponents(Components);
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
				UE_LOG(LogTemp, Warning, TEXT("[ControlSurface] '%s': control '%s' contributed twice (second from %s) — first wins."),
					*Owner->GetName(), *Axis.Id.ToString(), *Pair.Value->GetName());
				continue;
			}
			FRammsControlAxis Normalized = Axis;
			if (Normalized.IsAction())
			{
				// Actions have no value to read back, whatever the contributor left set.
				Normalized.bReadback = false;
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

	// Anything held by (or targeted through) a control that no longer exists is forgotten.
	for (auto It = Holds.CreateIterator(); It; ++It)
	{
		if (!Routes.Contains(It.Key()))
		{
			It.RemoveCurrent();
		}
	}
	for (auto It = Targets.CreateIterator(); It; ++It)
	{
		if (!Routes.Contains(It.Key()))
		{
			It.RemoveCurrent();
		}
	}
	bBuilt = true;
	++Version;
	UE_LOG(LogTemp, Log, TEXT("[ControlSurface] '%s': %d controls in %d groups from %d contributors (%d raw motors); version %d."),
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
	if (Priority(H->Source) > Priority(Source) && FPlatformTime::Seconds() - H->Time < ExternalHoldSeconds)
	{
		return false;
	}
	return true;
}

void URammsRobotControlSurfaceComponent::Hold(FName Id, ERammsControlSource Source)
{
	FHold& H = Holds.FindOrAdd(Id);
	H.Source = Source;
	H.Time = FPlatformTime::Seconds();
}

IRammsControlContributor* URammsRobotControlSurfaceComponent::ContributorFor(FName Id) const
{
	const FRoute* Route = Routes.Find(Id);
	return (Route && Route->Contributor.IsValid()) ? Cast<IRammsControlContributor>(Route->Contributor.Get()) : nullptr;
}

// --- sink ---------------------------------------------------------------------

bool URammsRobotControlSurfaceComponent::SetAxis_Implementation(FName Id, float Value, ERammsControlSource Source)
{
	EnsureBuilt();
	const FRammsControlAxis* Axis = Surface.Find(Id);
	const FRoute*			 Route = Routes.Find(Id);
	if (!Axis || !Route || Axis->IsAction() || Axis->bReadOnly || !MayDrive(Id, Source))
	{
		return false;
	}
	const float Clamped = Axis->Clamp(Value);
	bool		bApplied = false;
	if (IRammsControlContributor* C = ContributorFor(Id))
	{
		bApplied = C->ApplyControl(Id, Clamped);
	}
	else if (Base && Base->HasBackend() && !Route->MotorId.IsNone())
	{
		// Without a backend SetMotorCommand is a silent no-op: not applied.
		Base->SetMotorCommand(Route->MotorId, Clamped);
		bApplied = true;
	}
	if (bApplied)
	{
		Hold(Id, Source);
		Targets.Add(Id, Clamped);
	}
	return bApplied;
}

bool URammsRobotControlSurfaceComponent::TriggerAction_Implementation(FName Id, ERammsControlSource Source)
{
	EnsureBuilt();
	const FRammsControlAxis* Axis = Surface.Find(Id);
	if (!Axis || !Axis->IsAction() || !MayDrive(Id, Source))
	{
		return false;
	}
	IRammsControlContributor* C = ContributorFor(Id);
	return C && C->TriggerControl(Id);
}

bool URammsRobotControlSurfaceComponent::ReleaseAxis_Implementation(FName Id, ERammsControlSource Source)
{
	EnsureBuilt();
	const FRammsControlAxis* Axis = Surface.Find(Id);
	const FRoute*			 Route = Routes.Find(Id);
	if (!Axis || !Route || Axis->IsAction())
	{
		return false;
	}
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
		if (!bReleased && Axis->Kind == ERammsControlKind::Continuous)
		{
			bReleased = C->ApplyControl(Id, Axis->DefaultValue); // spring back
		}
	}
	else if (Base && Base->HasBackend() && !Route->MotorId.IsNone())
	{
		if (Axis->Kind == ERammsControlKind::Continuous)
		{
			Base->SetMotorCommand(Route->MotorId, Axis->DefaultValue); // spring back
			bReleased = true;
		}
		else
		{
			bReleased = Base->ReleaseMotor(Route->MotorId);
		}
	}
	if (bReleased)
	{
		Holds.Remove(Id);
		if (Axis->Kind == ERammsControlKind::Continuous)
		{
			Targets.Add(Id, Axis->DefaultValue); // sprung back: that is the target now
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
