// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsCoreBridge.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"

// ── World Utility ──────────────────────────────────────────────────

UWorld* URammsCoreBridge::GetPlayWorld()
{
	if (!GEngine)
		return nullptr;

	// Try play worlds first (PIE or standalone game)
	for (const FWorldContext& Context : GEngine->GetWorldContexts())
	{
		if (Context.WorldType == EWorldType::PIE || Context.WorldType == EWorldType::Game)
		{
			return Context.World();
		}
	}
	// Fall back to editor world
	for (const FWorldContext& Context : GEngine->GetWorldContexts())
	{
		if (Context.WorldType == EWorldType::Editor)
		{
			return Context.World();
		}
	}
	return nullptr;
}

// ── Actor Discovery ────────────────────────────────────────────────

TArray<FString> URammsCoreBridge::GetAllActorPaths()
{
	TArray<FString> Paths;

	UWorld* World = GetPlayWorld();
	if (World)
	{
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			AActor* Actor = *It;
			if (IsValid(Actor))
			{
				Paths.Add(Actor->GetPathName());
			}
		}
	}

	return Paths;
}

TArray<FString> URammsCoreBridge::FindActors(const FString& ClassNameFilter)
{
	TArray<FString> Paths;

	UWorld* World = GetPlayWorld();
	if (World)
	{
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			AActor* Actor = *It;
			if (IsValid(Actor))
			{
				FString ClassName = Actor->GetClass()->GetName();
				if (ClassNameFilter.IsEmpty() || ClassName.Contains(ClassNameFilter))
				{
					Paths.Add(Actor->GetPathName());
				}
			}
		}
	}

	return Paths;
}

// ── Component Discovery ────────────────────────────────────────────

TArray<FString> URammsCoreBridge::FindActorsByComponent(const FString& ComponentFilter)
{
	TArray<FString> Results;

	UWorld* World = GetPlayWorld();
	if (!World || ComponentFilter.IsEmpty())
		return Results;

	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (!IsValid(Actor))
			continue;

		TInlineComponentArray<UActorComponent*> Components;
		Actor->GetComponents(Components);

		for (UActorComponent* Comp : Components)
		{
			if (!IsValid(Comp))
				continue;

			FString ClassName = Comp->GetClass()->GetName();
			FString CompName = Comp->GetName();
			if (ClassName.Contains(ComponentFilter) || CompName.Contains(ComponentFilter))
			{
				// Format: "ActorPath|ComponentName:ComponentClassName"
				Results.Add(FString::Printf(TEXT("%s|%s:%s"),
					*Actor->GetPathName(), *Comp->GetName(), *ClassName));
				break; // One match per actor is enough
			}
		}
	}

	return Results;
}

TArray<FString> URammsCoreBridge::FindComponents(const FString& ActorPath, const FString& ClassNameFilter)
{
	TArray<FString> Results;

	UObject* Obj = StaticFindObject(AActor::StaticClass(), nullptr, *ActorPath);
	AActor* Actor = Cast<AActor>(Obj);
	if (!Actor)
	{
		// Try FindObject with ANY_PACKAGE
		Actor = FindObject<AActor>(nullptr, *ActorPath);
	}
	if (!Actor)
		return Results;

	TInlineComponentArray<UActorComponent*> Components;
	Actor->GetComponents(Components);

	for (UActorComponent* Comp : Components)
	{
		if (!IsValid(Comp))
			continue;

		FString ClassName = Comp->GetClass()->GetName();
		FString CompName = Comp->GetName();
		if (!ClassNameFilter.IsEmpty()
			&& !ClassName.Contains(ClassNameFilter)
			&& !CompName.Contains(ClassNameFilter))
			continue;

		// Format: "ComponentName:ClassName"
		Results.Add(FString::Printf(TEXT("%s:%s"), *Comp->GetName(), *ClassName));
	}

	return Results;
}

FString URammsCoreBridge::GetComponentPath(const FString& ActorPath, const FString& ComponentName)
{
	UObject* Obj = StaticFindObject(AActor::StaticClass(), nullptr, *ActorPath);
	AActor* Actor = Cast<AActor>(Obj);
	if (!Actor)
	{
		Actor = FindObject<AActor>(nullptr, *ActorPath);
	}
	if (!Actor)
		return FString();

	TInlineComponentArray<UActorComponent*> Components;
	Actor->GetComponents(Components);

	for (UActorComponent* Comp : Components)
	{
		if (IsValid(Comp) && Comp->GetName() == ComponentName)
		{
			return Comp->GetPathName();
		}
	}

	return FString();
}
