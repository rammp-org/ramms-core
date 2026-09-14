// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsActuationBackendRegistry.h"

namespace RammsActuationBackends
{
	// Process-wide registered factory (game-thread access: module startup +
	// actor BeginPlay).
	static FRammsActuationBackendFactory GMujocoFactory;

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
} // namespace RammsActuationBackends
