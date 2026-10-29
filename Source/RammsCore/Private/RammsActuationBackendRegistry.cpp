// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsActuationBackendRegistry.h"

namespace RammsActuationBackends
{
	// Process-wide registered factory (game-thread access: module startup +
	// actor BeginPlay).
	static FRammsActuationBackendFactory GMujocoFactory;
	static FRammsActuationBackendFactory GChaosArticulationFactory;

	void RegisterMujocoFactory(FRammsActuationBackendFactory Factory)
	{
		GMujocoFactory = MoveTemp(Factory);
	}

	void UnregisterMujocoFactory()
	{
		GMujocoFactory.Unbind();
	}

	IRammsActuationBackend* CreateMujocoBackend(URammsRobotBaseComponent& Base)
	{
		return GMujocoFactory.IsBound() ? GMujocoFactory.Execute(Base) : nullptr;
	}

	void RegisterChaosArticulationFactory(FRammsActuationBackendFactory Factory)
	{
		GChaosArticulationFactory = MoveTemp(Factory);
	}

	void UnregisterChaosArticulationFactory()
	{
		GChaosArticulationFactory.Unbind();
	}

	IRammsActuationBackend* CreateChaosArticulationBackend(URammsRobotBaseComponent& Base)
	{
		return GChaosArticulationFactory.IsBound() ? GChaosArticulationFactory.Execute(Base) : nullptr;
	}
} // namespace RammsActuationBackends
