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
 * registers here at module startup:
 *  - a MuJoCo backend: the robot's URLab articulation, on whichever engine
 *    simulates it;
 *  - a Chaos articulation backend: the same articulation, switched to Chaos
 *    (Backend = Chaos on the base component).
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

	/** Register the factory for a robot described by MJCF / USD whose
	 *  articulation Chaos simulates (RammsMujocoSupport startup). Its
	 *  Initialize returns false for a robot that has no such articulation, and
	 *  the base component then falls back to the skeletal Chaos backend. */
	RAMMSCORE_API void RegisterChaosArticulationFactory(FRammsActuationBackendFactory Factory);

	/** Clear the registered Chaos articulation factory (RammsMujocoSupport shutdown). */
	RAMMSCORE_API void UnregisterChaosArticulationFactory();

	/** Create a Chaos articulation backend for the base component, or nullptr
	 *  when none is registered. The caller owns the result and must call
	 *  Initialize. */
	RAMMSCORE_API IRammsActuationBackend* CreateChaosArticulationBackend(URammsRobotBaseComponent& Base);
} // namespace RammsActuationBackends
