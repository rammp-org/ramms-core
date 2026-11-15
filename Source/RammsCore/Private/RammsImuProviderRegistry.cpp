// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsImuProvider.h"

#include "Components/SceneComponent.h"
#include "Engine/World.h"

namespace RammsImuProviderRegistry
{
	/** Game thread only: registration happens at module startup and shutdown, creation from world subsystems. */
	TMap<ERammsImuSource, FRammsImuProviderFactory>& Factories()
	{
		static TMap<ERammsImuSource, FRammsImuProviderFactory> Registered;
		return Registered;
	}
} // namespace RammsImuProviderRegistry

void RammsImuProviders::Register(ERammsImuSource Source, FRammsImuProviderFactory Factory)
{
	check(IsInGameThread());
	RammsImuProviderRegistry::Factories().Add(Source, MoveTemp(Factory));
}

void RammsImuProviders::Unregister(ERammsImuSource Source)
{
	check(IsInGameThread());
	RammsImuProviderRegistry::Factories().Remove(Source);
}

bool RammsImuProviders::IsRegistered(ERammsImuSource Source)
{
	const FRammsImuProviderFactory* Factory = RammsImuProviderRegistry::Factories().Find(Source);
	return Factory && Factory->IsBound();
}

TSharedPtr<IRammsImuProvider> RammsImuProviders::Create(ERammsImuSource Source, UWorld& World)
{
	const FRammsImuProviderFactory* Factory = RammsImuProviderRegistry::Factories().Find(Source);
	return Factory && Factory->IsBound() ? Factory->Execute(World) : nullptr;
}

FTransform RammsImuProviders::ComponentFrame(const USceneComponent& Component, FName Bone)
{
	// GetSocketTransform resolves a bone or socket of a skinned mesh (or any socket a component declares) and falls
	// back to the component's own transform.
	FTransform Frame = Bone.IsNone() ? Component.GetComponentTransform() : Component.GetSocketTransform(Bone, RTS_World);
	Frame.SetScale3D(FVector::OneVector);
	return Frame;
}
