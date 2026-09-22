// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsInputTestLibrary.h"

#include "GameFramework/PlayerController.h"

#if WITH_EDITOR

	#include "InputKeyEventArgs.h"

TMap<TWeakObjectPtr<APlayerController>, TSet<FKey>> URammsInputTestLibrary::InjectedKeys;

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

void URammsInputTestLibrary::ReleaseAllInjectedKeys(APlayerController* PlayerController)
{
	// Drop entries whose controller has gone; nothing can release those.
	for (auto It = InjectedKeys.CreateIterator(); It; ++It)
	{
		if (!It.Key().IsValid())
		{
			It.RemoveCurrent();
		}
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
