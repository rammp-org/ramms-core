// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class IRammsActuationBackend;
class URammsRobotBaseComponent;

/**
 * Factory that creates an actuation backend for a robot base component, or
 * nullptr if it cannot. Ownership passes to the caller (the base component
 * frees it). Construct-only; whether it can actually drive is decided by
 * IRammsActuationBackend::Initialize.
 */
DECLARE_DELEGATE_RetVal_OneParam(IRammsActuationBackend*, FRammsActuationBackendFactory, URammsRobotBaseComponent&);

/**
 * Cross-plugin registration point for engine-specific actuation backends.
 * RammsCore owns the robot base component and the IRammsActuationBackend
 * interface but must not depend on any physics engine; RammsMujocoSupport
 * registers a URLab/MuJoCo backend here at module startup.
 */
namespace RammsActuationBackends
{
	/** Register the MuJoCo actuation-backend factory (RammsMujocoSupport startup). */
	RAMMSCORE_API void RegisterMujocoFactory(FRammsActuationBackendFactory Factory);

	/** Clear the registered MuJoCo factory (RammsMujocoSupport shutdown). */
	RAMMSCORE_API void UnregisterMujocoFactory();

	/** Create a MuJoCo backend for the base component, or nullptr when none is
	 *  registered. The caller owns the result and must call Initialize. */
	RAMMSCORE_API IRammsActuationBackend* CreateMujocoBackend(URammsRobotBaseComponent& Base);
} // namespace RammsActuationBackends
