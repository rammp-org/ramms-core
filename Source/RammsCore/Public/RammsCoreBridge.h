// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RammsCoreBridge.generated.h"

/**
 * Static function library for Remote Control API integration (core/non-UI).
 *
 * Provides actor and component discovery functions callable via:
 *   PUT /remote/object/call on the CDO path:
 *   /Script/RammsCore.Default__RammsCoreBridge
 *
 * These use GEngine->GetWorldContexts() and TActorIterator to find runtime
 * objects without needing a world context, making them accessible from
 * the Remote Control HTTP API and Python scripts.
 */
UCLASS()
class RAMMSCORE_API URammsCoreBridge : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	// ── Actor Discovery ───────────────────────────────────────────

	/**
	 * Get object paths for all actors in the current play world (PIE or game).
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote")
	static TArray<FString> GetAllActorPaths();

	/**
	 * Get object paths for actors matching a class name substring.
	 * e.g. "StaticMeshActor", "KinovaGen3", "CameraActor"
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote")
	static TArray<FString> FindActors(const FString& ClassNameFilter);

	// ── Component Discovery ───────────────────────────────────────

	/**
	 * Find actors that have a component matching a class name substring.
	 * Returns an array of "ActorPath|ComponentName:ComponentClassName" strings.
	 *
	 * @param ComponentClassFilter  Class name substring filter for components
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote")
	static TArray<FString> FindActorsByComponent(const FString& ComponentClassFilter);

	/**
	 * Get component names and class names for a given actor.
	 * Returns an array of "ComponentName:ClassName" strings.
	 *
	 * @param ActorPath  Full object path of the actor
	 * @param ClassNameFilter  Substring filter matched against both the component
	 *                         instance name and the class name (empty string = all components)
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote")
	static TArray<FString> FindComponents(const FString& ActorPath, const FString& ClassNameFilter = TEXT(""));

	/**
	 * Get the full object path for a named component on an actor.
	 * Returns empty string if not found.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote")
	static FString GetComponentPath(const FString& ActorPath, const FString& ComponentName);

	// ── World Utility ─────────────────────────────────────────────

	/**
	 * Get the current play world (PIE or Game), falling back to Editor world.
	 * Useful for other code that needs the same world-finding logic.
	 */
	static UWorld* GetPlayWorld();
};
