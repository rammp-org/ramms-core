// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RammsBlueprintCleanupLibrary.generated.h"

class UBlueprint;

/**
 * Editor-only helpers for migrating robot Blueprints onto the control
 * surface: strip the direct input wiring (key events, input-action value
 * reads, per-tick drive writes) that the Enhanced Input map and the surface
 * replace. Callable from editor Python / utility widgets.
 */
UCLASS()
class RAMMSCOREEDITOR_API URammsBlueprintCleanupLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** Remove from the event graph every InputKey / InputAction / InputAxis /
	 *  EnhancedInputAction / GetInputActionValue node, plus every call to a
	 *  function named in FunctionsToRemove (e.g. "SetDriveInput" for the
	 *  legacy per-tick joystick write). Nodes that were only fed by the
	 *  removed ones are left in place (RemoveUnusedNodes cleans them).
	 *  Returns the number of nodes removed; compiles the Blueprint. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Blueprint Cleanup")
	static int32 RemoveLegacyInputNodes(UBlueprint* Blueprint, const TArray<FName>& FunctionsToRemove);

	/** Names of the input-related nodes currently in the event graph (audit). */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Blueprint Cleanup")
	static TArray<FString> ListInputNodes(UBlueprint* Blueprint);
};
