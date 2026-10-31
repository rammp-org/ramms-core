// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsInputTestLibrary.h"

#include "Engine/World.h"
#include "GameFramework/PlayerController.h"

#if WITH_EDITOR

	#include "InputKeyEventArgs.h"

TMap<TWeakObjectPtr<APlayerController>, TSet<FKey>> URammsInputTestLibrary::InjectedKeys;
TMap<TWeakObjectPtr<APlayerController>, TSet<FKey>> URammsInputTestLibrary::InjectedAxes;

bool URammsInputTestLibrary::SetKeyDown(APlayerController* PlayerController, FName KeyName, bool bDown)
{
	const FKey Key(KeyName);
	if (!PlayerController || !Key.IsValid())
	{
		return false;
	}

	// The default device, as hardware input would arrive on.
	const FInputDeviceId Device = IPlatformInputDeviceMapper::Get().GetDefaultInputDevice();
	FInputKeyEventArgs	 Args(
		/*Viewport=*/nullptr,
		Device,
		Key,
		bDown ? EInputEvent::IE_Pressed : EInputEvent::IE_Released,
		FPlatformTime::Cycles64());

	const bool bHandled = PlayerController->InputKey(Args);

	if (bDown)
	{
		InjectedKeys.FindOrAdd(PlayerController).Add(Key);
	}
	else if (TSet<FKey>* Held = InjectedKeys.Find(PlayerController))
	{
		Held->Remove(Key);
		if (Held->IsEmpty())
		{
			InjectedKeys.Remove(PlayerController);
		}
	}
	return bHandled;
}

bool URammsInputTestLibrary::IsKeyDown(APlayerController* PlayerController, FName KeyName)
{
	const FKey Key(KeyName);
	return PlayerController && Key.IsValid() && PlayerController->IsInputKeyDown(Key);
}

bool URammsInputTestLibrary::SetAxisKey(APlayerController* PlayerController, FName KeyName, float Value)
{
	const FKey Key(KeyName);
	if (!PlayerController || !Key.IsValid() || !Key.IsAxis1D())
	{
		return false;
	}
	// One sample, as a pad's driver sends one per change: the key's state
	// holds until the next (UPlayerInput keeps an axis with no samples).
	const UWorld*	   World = PlayerController->GetWorld();
	FInputKeyEventArgs Args(
		/*Viewport=*/nullptr,
		IPlatformInputDeviceMapper::Get().GetDefaultInputDevice(),
		Key,
		Value,
		World ? World->GetDeltaSeconds() : 0.0f,
		/*NumSamples=*/1,
		FPlatformTime::Cycles64());
	PlayerController->InputKey(Args);

	if (Value != 0.0f)
	{
		InjectedAxes.FindOrAdd(PlayerController).Add(Key);
	}
	else if (TSet<FKey>* Moved = InjectedAxes.Find(PlayerController))
	{
		Moved->Remove(Key);
		if (Moved->IsEmpty())
		{
			InjectedAxes.Remove(PlayerController);
		}
	}
	return true;
}

void URammsInputTestLibrary::ReleaseAllInjectedKeys(APlayerController* PlayerController)
{
	// Drop entries whose controller has gone; nothing can release those.
	for (TMap<TWeakObjectPtr<APlayerController>, TSet<FKey>>* Injected : { &InjectedKeys, &InjectedAxes })
	{
		for (auto It = Injected->CreateIterator(); It; ++It)
		{
			if (!It.Key().IsValid())
			{
				It.RemoveCurrent();
			}
		}
	}

	if (const TSet<FKey>* Moved = InjectedAxes.Find(PlayerController))
	{
		// Copy: SetAxisKey mutates the set as it centres.
		const TSet<FKey> Axes = *Moved;
		for (const FKey& Key : Axes)
		{
			SetAxisKey(PlayerController, Key.GetFName(), 0.0f);
		}
		InjectedAxes.Remove(PlayerController);
	}

	const TSet<FKey>* Found = InjectedKeys.Find(PlayerController);
	if (!Found)
	{
		return;
	}
	// Copy: SetKeyDown mutates the set as it releases.
	const TSet<FKey> Held = *Found;
	for (const FKey& Key : Held)
	{
		SetKeyDown(PlayerController, Key.GetFName(), false);
	}
	InjectedKeys.Remove(PlayerController);
}

#endif // WITH_EDITOR
