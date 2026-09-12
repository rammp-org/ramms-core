// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsDriveBackendRegistry.h"

namespace RammsDriveBackends
{
	// Process-wide registered factory. Set by RammsMujocoSupport at module
	// startup; read by the controller in BeginPlay. Single-threaded (game
	// thread) access, matching module startup and actor BeginPlay.
	static FRammsDriveBackendFactory GMujocoFactory;

	void RegisterMujocoFactory(FRammsDriveBackendFactory Factory)
	{
		GMujocoFactory = MoveTemp(Factory);
	}

	void UnregisterMujocoFactory()
	{
		GMujocoFactory.Unbind();
	}

	IRammsDriveBackend* CreateMujocoBackend(URammsDifferentialDriveController& Controller)
	{
		return GMujocoFactory.IsBound() ? GMujocoFactory.Execute(Controller) : nullptr;
	}
} // namespace RammsDriveBackends
