// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsInputTestLibrary.h"

#include "GameFramework/PlayerController.h"

#if WITH_EDITOR

	#include "InputKeyEventArgs.h"

TSet<FKey> URammsInputTestLibrary::InjectedKeys;

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
		InjectedKeys.Add(Key);
	}
	else
	{
		InjectedKeys.Remove(Key);
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
	// Copy: SetKeyDown mutates the set as it releases.
	const TSet<FKey> Held = InjectedKeys;
	for (const FKey& Key : Held)
	{
		SetKeyDown(PlayerController, Key.GetFName(), false);
	}
	InjectedKeys.Reset();
}

#endif // WITH_EDITOR
