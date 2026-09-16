// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsControlInputComponent.h"
#include "RammsControlSink.h"
#include "RammsControlSurfaceProvider.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "Engine/LocalPlayer.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"

URammsControlInputComponent::URammsControlInputComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void URammsControlInputComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Unbind();
	Super::EndPlay(EndPlayReason);
}

UObject* URammsControlInputComponent::ResolveSink()
{
	if (Sink)
	{
		return Sink;
	}
	if (const AActor* Owner = GetOwner())
	{
		TArray<UActorComponent*> Components;
		Owner->GetComponents(Components);
		for (UActorComponent* C : Components)
		{
			if (C && C->GetClass()->ImplementsInterface(URammsControlSink::StaticClass()))
			{
				Sink = C;
				break;
			}
		}
	}
	return Sink;
}

bool URammsControlInputComponent::SurfaceAxis(FName Id, FRammsControlAxis& Out) const
{
	if (const FRammsControlAxis* Axis = CachedSurface.Find(Id))
	{
		Out = *Axis;
		return true;
	}
	return false;
}

void URammsControlInputComponent::ResolveSlots()
{
	// Expand every binding's control Ids against the sink's current surface
	// (wildcards included); redone when the surface version changes.
	if (!Sink || !Sink->GetClass()->ImplementsInterface(URammsControlSurfaceProvider::StaticClass()))
	{
		return;
	}
	const int32 Version = IRammsControlSurfaceProvider::Execute_GetControlSurfaceVersion(Sink);
	if (Version == SurfaceVersionSeen && Slots.Num() > 0)
	{
		return;
	}
	SurfaceVersionSeen = Version;
	CachedSurface = IRammsControlSurfaceProvider::Execute_GetControlSurface(Sink);

	auto Expand = [this](FName Pattern) {
		TArray<FName> Out;
		if (Pattern.IsNone())
		{
			return Out;
		}
		const FString P = Pattern.ToString();
		if (!P.Contains(TEXT("*")) && !P.Contains(TEXT("?")))
		{
			Out.Add(Pattern);
			return Out;
		}
		for (const FRammsControlAxis& Axis : CachedSurface.Axes)
		{
			if (Axis.Id.ToString().MatchesWildcard(P))
			{
				Out.Add(Axis.Id);
			}
		}
		return Out;
	};

	// Keep the slot list stable (handles index into it); only refresh targets.
	if (Slots.Num() == 0)
	{
		for (const URammsControlInputMap* Map : InputMaps)
		{
			if (!Map)
			{
				continue;
			}
			for (int32 i = 0; i < Map->Bindings.Num(); ++i)
			{
				FSlot Slot;
				Slot.Map = Map;
				Slot.Index = i;
				Slots.Add(Slot);
			}
		}
	}
	for (FSlot& Slot : Slots)
	{
		const URammsControlInputMap* Map = Slot.Map.Get();
		if (!Map || !Map->Bindings.IsValidIndex(Slot.Index))
		{
			continue;
		}
		const FRammsControlInputBinding& B = Map->Bindings[Slot.Index];
		Slot.Ids[0] = Expand(B.ControlId);
		Slot.Ids[1] = Expand(B.ControlIdY);
		Slot.Ids[2] = Expand(B.ControlIdZ);
	}
}

void URammsControlInputComponent::TryBind()
{
	APawn*			   Pawn = Cast<APawn>(GetOwner());
	APlayerController* PC = Pawn ? Cast<APlayerController>(Pawn->GetController()) : nullptr;
	if (!PC || !PC->IsLocalController())
	{
		if (BoundInputComponent.IsValid() || BoundPC.IsValid())
		{
			Unbind();
		}
		return;
	}
	UEnhancedInputComponent* EIC = Cast<UEnhancedInputComponent>(Pawn->InputComponent.Get());
	if (!EIC)
	{
		return;
	}
	if (BoundInputComponent.Get() == EIC && BoundPC.Get() == PC)
	{
		return;
	}
	Unbind();

	if (!ResolveSink())
	{
		return; // no surface on this pawn (yet)
	}
	ResolveSlots();

	ULocalPlayer* LP = PC->GetLocalPlayer();
	BoundSubsystem = LP ? ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(LP) : nullptr;
	if (BoundSubsystem.IsValid())
	{
		for (const URammsControlInputMap* Map : InputMaps)
		{
			if (Map && Map->MappingContext && !BoundSubsystem->HasMappingContext(Map->MappingContext))
			{
				BoundSubsystem->AddMappingContext(Map->MappingContext, Map->Priority);
				AddedContexts.Add(Map->MappingContext);
			}
		}
	}

	for (int32 s = 0; s < Slots.Num(); ++s)
	{
		const URammsControlInputMap* Map = Slots[s].Map.Get();
		if (!Map || !Map->Bindings.IsValidIndex(Slots[s].Index))
		{
			continue;
		}
		const FRammsControlInputBinding& B = Map->Bindings[Slots[s].Index];
		if (!B.Action)
		{
			continue;
		}
		switch (B.Mode)
		{
			case ERammsControlInputMode::Action:
				BindingHandles.Add(EIC->BindActionInstanceLambda(B.Action, ETriggerEvent::Started, [this, s](const FInputActionInstance& I) { OnStarted(s, I); }).GetHandle());
				break;
			case ERammsControlInputMode::IncrementRate:
				BindingHandles.Add(EIC->BindActionInstanceLambda(B.Action, ETriggerEvent::Started, [this, s](const FInputActionInstance& I) { OnStarted(s, I); }).GetHandle());
				BindingHandles.Add(EIC->BindActionInstanceLambda(B.Action, ETriggerEvent::Triggered, [this, s](const FInputActionInstance& I) { OnTriggered(s, I); }).GetHandle());
				break;
			case ERammsControlInputMode::Axis:
			default:
				BindingHandles.Add(EIC->BindActionInstanceLambda(B.Action, ETriggerEvent::Triggered, [this, s](const FInputActionInstance& I) { OnTriggered(s, I); }).GetHandle());
				BindingHandles.Add(EIC->BindActionInstanceLambda(B.Action, ETriggerEvent::Completed, [this, s](const FInputActionInstance& I) { OnCompleted(s, I); }).GetHandle());
				break;
		}
	}

	BoundInputComponent = EIC;
	BoundPC = PC;
	if (bLogCommands)
	{
		UE_LOG(LogTemp, Log, TEXT("[ControlInput] '%s' bound %d action bindings for %s"), *GetOwner()->GetName(), BindingHandles.Num(), *PC->GetName());
	}
}

void URammsControlInputComponent::Unbind()
{
	if (UEnhancedInputComponent* EIC = BoundInputComponent.Get())
	{
		for (uint32 Handle : BindingHandles)
		{
			EIC->RemoveBindingByHandle(Handle);
		}
	}
	BindingHandles.Reset();
	if (UEnhancedInputLocalPlayerSubsystem* Subsystem = BoundSubsystem.Get())
	{
		// Only the contexts this component added: one already present at bind
		// time belongs to someone else (a controller, another pawn component).
		for (const TWeakObjectPtr<const UInputMappingContext>& Context : AddedContexts)
		{
			if (const UInputMappingContext* IMC = Context.Get())
			{
				Subsystem->RemoveMappingContext(IMC);
			}
		}
	}
	AddedContexts.Reset();
	// Let go of everything this driver was holding.
	if (Sink)
	{
		for (const FName& Id : HeldAxes)
		{
			IRammsControlSink::Execute_ReleaseAxis(Sink, Id, Source);
		}
	}
	HeldAxes.Reset();
	IncrementTargets.Reset();
	Injections.Reset();
	BoundInputComponent = nullptr;
	BoundPC = nullptr;
	BoundSubsystem = nullptr;
}

void URammsControlInputComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	TryBind();
	if (!BoundInputComponent.IsValid())
	{
		return;
	}
	ResolveSlots(); // picks up surface rebuilds (new contributors, registry loads)

	// Held injections: re-feed each action every tick until it expires.
	if (Injections.Num() > 0)
	{
		const double Now = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
		if (UEnhancedInputLocalPlayerSubsystem* Subsystem = BoundSubsystem.Get())
		{
			for (const FInjection& Inj : Injections)
			{
				if (const UInputAction* Action = Inj.Action.Get())
				{
					Subsystem->InjectInputForAction(Action, FInputActionValue(Inj.Value), {}, {});
				}
			}
		}
		Injections.RemoveAll([Now](const FInjection& Inj) { return !Inj.Action.IsValid() || Now >= Inj.Until; });
	}
}

void URammsControlInputComponent::InjectAction(const UInputAction* Action, FVector Value, float HoldSeconds)
{
	if (!Action)
	{
		return;
	}
	const double Now = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
	FInjection*	 Existing = Injections.FindByPredicate([Action](const FInjection& I) { return I.Action.Get() == Action; });
	FInjection&	 Inj = Existing ? *Existing : Injections.AddDefaulted_GetRef();
	Inj.Action = Action;
	Inj.Value = Value;
	Inj.Until = Now + FMath::Max(0.0f, HoldSeconds);
	// Inject right away as well, so a one-shot lands this frame if we're
	// called before input processing.
	if (UEnhancedInputLocalPlayerSubsystem* Subsystem = BoundSubsystem.Get())
	{
		Subsystem->InjectInputForAction(Action, FInputActionValue(Value), {}, {});
	}
}

bool URammsControlInputComponent::InjectActionByName(FName ActionName, FVector Value, float HoldSeconds)
{
	for (const URammsControlInputMap* Map : InputMaps)
	{
		if (!Map)
		{
			continue;
		}
		for (const FRammsControlInputBinding& B : Map->Bindings)
		{
			if (B.Action && B.Action->GetFName() == ActionName)
			{
				InjectAction(B.Action, Value, HoldSeconds);
				return true;
			}
		}
	}
	return false;
}

TArray<FName> URammsControlInputComponent::GetBoundActionNames() const
{
	TArray<FName> Out;
	if (!BoundInputComponent.IsValid())
	{
		return Out;
	}
	for (const FSlot& Slot : Slots)
	{
		const URammsControlInputMap* Map = Slot.Map.Get();
		if (Map && Map->Bindings.IsValidIndex(Slot.Index) && Map->Bindings[Slot.Index].Action)
		{
			Out.AddUnique(Map->Bindings[Slot.Index].Action->GetFName());
		}
	}
	return Out;
}

// --- action handlers -------------------------------------------------------------

void URammsControlInputComponent::OnStarted(int32 SlotIndex, const FInputActionInstance& Instance)
{
	if (!Sink || !Slots.IsValidIndex(SlotIndex))
	{
		return;
	}
	const FSlot&				 Slot = Slots[SlotIndex];
	const URammsControlInputMap* Map = Slot.Map.Get();
	if (!Map || !Map->Bindings.IsValidIndex(Slot.Index))
	{
		return;
	}
	const FRammsControlInputBinding& B = Map->Bindings[Slot.Index];
	if (B.Mode == ERammsControlInputMode::Action)
	{
		for (const FName& Id : Slot.Ids[0])
		{
			const bool bOk = IRammsControlSink::Execute_TriggerAction(Sink, Id, Source);
			if (bLogCommands)
			{
				UE_LOG(LogTemp, Log, TEXT("[ControlInput] trigger %s -> %s"), *Id.ToString(), bOk ? TEXT("ok") : TEXT("refused"));
			}
		}
	}
	else if (B.Mode == ERammsControlInputMode::IncrementRate)
	{
		// Start moving from wherever the control is now.
		for (int32 a = 0; a < 3; ++a)
		{
			for (const FName& Id : Slot.Ids[a])
			{
				IncrementTargets.Add(Id, IRammsControlSink::Execute_GetAxisValue(Sink, Id));
			}
		}
	}
}

void URammsControlInputComponent::OnTriggered(int32 SlotIndex, const FInputActionInstance& Instance)
{
	if (!Sink || !Slots.IsValidIndex(SlotIndex))
	{
		return;
	}
	const FSlot&				 Slot = Slots[SlotIndex];
	const URammsControlInputMap* Map = Slot.Map.Get();
	if (!Map || !Map->Bindings.IsValidIndex(Slot.Index))
	{
		return;
	}
	const FRammsControlInputBinding& B = Map->Bindings[Slot.Index];
	const FVector					 Value = Instance.GetValue().Get<FVector>();
	const float						 Dt = GetWorld() ? GetWorld()->GetDeltaSeconds() : 0.0f;

	for (int32 a = 0; a < 3; ++a)
	{
		const float V = Component(Value, a) * B.Scale;
		for (const FName& Id : Slot.Ids[a])
		{
			if (B.Mode == ERammsControlInputMode::IncrementRate)
			{
				float* Target = IncrementTargets.Find(Id);
				if (!Target)
				{
					Target = &IncrementTargets.Add(Id, IRammsControlSink::Execute_GetAxisValue(Sink, Id));
				}
				*Target += V * B.RatePerSecond * Dt;
				FRammsControlAxis Axis;
				if (SurfaceAxis(Id, Axis))
				{
					*Target = Axis.Clamp(*Target);
				}
				IRammsControlSink::Execute_SetAxis(Sink, Id, *Target, Source);
				// A held Position target is not "held" in the spring-back sense:
				// letting go leaves the motor where it is.
			}
			else // Axis
			{
				const bool bOk = IRammsControlSink::Execute_SetAxis(Sink, Id, V, Source);
				if (bOk)
				{
					HeldAxes.Add(Id);
				}
				if (bLogCommands)
				{
					UE_LOG(LogTemp, Verbose, TEXT("[ControlInput] %s = %.3f -> %s"), *Id.ToString(), V, bOk ? TEXT("ok") : TEXT("refused"));
				}
			}
		}
	}
}

void URammsControlInputComponent::OnCompleted(int32 SlotIndex, const FInputActionInstance& Instance)
{
	if (!Sink || !Slots.IsValidIndex(SlotIndex))
	{
		return;
	}
	const FSlot& Slot = Slots[SlotIndex];
	for (int32 a = 0; a < 3; ++a)
	{
		for (const FName& Id : Slot.Ids[a])
		{
			IRammsControlSink::Execute_ReleaseAxis(Sink, Id, Source);
			HeldAxes.Remove(Id);
			if (bLogCommands)
			{
				UE_LOG(LogTemp, Verbose, TEXT("[ControlInput] release %s"), *Id.ToString());
			}
		}
	}
}
