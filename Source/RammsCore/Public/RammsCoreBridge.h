// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RammsCoreBridge.generated.h"

class URammsSkeletalPoseComponent;

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
	 * Find actors that have a component whose class name or instance name
	 * contains the given substring.
	 * Returns an array of "ActorPath|ComponentName:ComponentClassName" strings.
	 *
	 * @param ComponentFilter  Substring matched against both the component
	 *                         class name and the component instance name
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote")
	static TArray<FString> FindActorsByComponent(const FString& ComponentFilter);

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

	// ── Skeletal Pose Helpers ─────────────────────────────────────

	/**
	 * Find actors that have a URammsSkeletalPoseComponent.
	 * Returns "ActorPath|ComponentName" for each match.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote")
	static TArray<FString> FindSkeletalPoseActors();

	/**
	 * Set joint targets on the first URammsSkeletalPoseComponent found on the
	 * specified actor.  Returns false if actor or component not found.
	 *
	 * Uses TArray<double> to work around UE Remote Control TArray<float> bug.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote")
	static bool SetSkeletalPoseJointTargets(const FString& ActorPath, const TArray<double>& JointValues);

	/**
	 * Get current joint values from the first URammsSkeletalPoseComponent
	 * on the specified actor.  Returns empty array if not found.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote")
	static TArray<float> GetSkeletalPoseJointValues(const FString& ActorPath);

	// ── World Utility ─────────────────────────────────────────────

	/**
	 * Get the current play world (PIE or Game), falling back to Editor world.
	 * Useful for other code that needs the same world-finding logic.
	 */
	static UWorld* GetPlayWorld();
};
