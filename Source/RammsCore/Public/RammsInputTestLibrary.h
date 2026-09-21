// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "InputCoreTypes.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RammsInputTestLibrary.generated.h"

class APlayerController;

/**
 * Inject key state so a test can drive input paths that read the keyboard.
 *
 * URammsKeyboardTeleopComponent POLLS: every tick it asks
 * APlayerController::IsInputKeyDown. So a single injected press is enough --
 * the key reads as held on every later frame until a release is injected, and
 * a test does not have to fake a repeat per frame.
 *
 * Exists because none of this is otherwise reachable from a test: InputKey is
 * not BlueprintCallable, and Python exposes only the read side
 * (is_input_key_down and friends). With this, the PIE harness can hold W for a
 * second and assert the robot responded -- covering teleop's own tick path
 * rather than just the control-surface calls underneath it.
 *
 * Editor-only. This is test scaffolding, not shipping surface.
 */
UCLASS()
class RAMMSCORE_API URammsInputTestLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
#if WITH_EDITOR
	/**
	 * Press or release Key on PlayerController as though the hardware had.
	 *
	 * @param PlayerController  The controller to deliver to; typically the PIE one.
	 * @param KeyName           The key's name, e.g. "W", "E", "SpaceBar".
	 * @param bDown             True presses, false releases.
	 * @return False when the controller is null or the name is not a key.
	 *
	 * Takes a name rather than an FKey because FKey cannot be constructed from
	 * Python -- unreal.Key exposes no KeyName -- and a name is what a remote
	 * caller has anyway.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Input Test",
		meta = (ToolTip = "Inject a key press or release for tests. Editor only."))
	static bool SetKeyDown(APlayerController* PlayerController, FName KeyName, bool bDown);

	/** True while the injected (or real) key reads as held -- what teleop polls. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Input Test")
	static bool IsKeyDown(APlayerController* PlayerController, FName KeyName);

	/** Release every key this library pressed and is still holding.
	 *  A test that fails partway would otherwise leave a key stuck down, and
	 *  the next test inherits it. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Input Test")
	static void ReleaseAllInjectedKeys(APlayerController* PlayerController);

private:
	/** Keys pressed through SetKeyDown and not yet released. */
	static TSet<FKey> InjectedKeys;
#endif
};
