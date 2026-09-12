// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class IRammsDriveBackend;
class URammsDifferentialDriveController;

/**
 * Factory that creates a drive backend for a controller. Returns a
 * heap-allocated backend (ownership passes to the caller — the controller frees
 * it), or nullptr if it cannot provide one. Whether the backend can actually
 * drive is decided later by IRammsDriveBackend::Initialize; a factory only
 * needs to construct.
 */
DECLARE_DELEGATE_RetVal_OneParam(IRammsDriveBackend*, FRammsDriveBackendFactory, URammsDifferentialDriveController&);

/**
 * Cross-plugin registration point for alternate drive backends.
 *
 * RammsCore owns the differential-drive controller and the IRammsDriveBackend
 * interface, but must not depend on any specific physics engine. A plugin that
 * can drive bases in another engine — RammsMujocoSupport for URLab/MuJoCo —
 * registers its factory here at module startup; the controller asks for one in
 * BeginPlay and falls back to the built-in Chaos backend if none is registered
 * or the returned backend fails to initialize.
 */
namespace RammsDriveBackends
{
	/** Register the MuJoCo drive-backend factory (RammsMujocoSupport startup). */
	RAMMSCORE_API void RegisterMujocoFactory(FRammsDriveBackendFactory Factory);

	/** Clear the registered MuJoCo factory (RammsMujocoSupport shutdown). */
	RAMMSCORE_API void UnregisterMujocoFactory();

	/** Create a MuJoCo backend for the controller, or nullptr when none is
	 *  registered. The caller owns the result and must call Initialize. */
	RAMMSCORE_API IRammsDriveBackend* CreateMujocoBackend(URammsDifferentialDriveController& Controller);
} // namespace RammsDriveBackends
