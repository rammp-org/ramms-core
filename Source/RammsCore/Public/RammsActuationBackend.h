// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class URammsRobotBaseComponent;

/**
 * The physics I/O of a robot's motors, abstracted away from the engine that
 * simulates them. `URammsRobotBaseComponent` owns one of these (Chaos skeletal
 * by default, MuJoCo/URLab supplied by RammsMujocoSupport) and routes every
 * controller's per-motor command/read/query through it — so RammsCore depends
 * on no specific physics engine, and controllers never touch one.
 *
 * Motors are addressed by their registry Id (see FRammsMotorSpec); the backend
 * resolves the Id to its engine handle (and honours the per-backend name
 * override) via the base component passed to Initialize.
 */
class RAMMSCORE_API IRammsActuationBackend
{
public:
	virtual ~IRammsActuationBackend() = default;

	/** Resolve the simulated robot (skeletal mesh / articulation) and any
	 *  one-time setup. Returns false when it cannot drive this robot, in which
	 *  case the base component leaves this backend unused. */
	virtual bool Initialize(URammsRobotBaseComponent& Base) = 0;

	/** Command a motor. Interpretation follows the motor's ERammsActuatorType
	 *  (torque / target position / target velocity). */
	virtual void SetCommand(FName MotorId, float Value) = 0;

	/** The motor's current scalar value — joint position/angle for a Position
	 *  motor, otherwise the driven joint's coordinate. 0 when unavailable. */
	virtual float GetValue(FName MotorId) const = 0;

	/** The motor's current joint velocity (rad/s or cm/s). 0 when unavailable. */
	virtual float GetVelocity(FName MotorId) const = 0;

	/** Stop actively driving a motor: a Position/Velocity servo lets go of its
	 *  joint (a Chaos constraint drive is disabled; a MuJoCo actuator has no
	 *  off switch and keeps its last ctrl). Torque motors have nothing to
	 *  release. Default: no-op. */
	virtual void ReleaseMotor(FName MotorId) {}

	/** World transform of the motor's joint/body, for the base component's
	 *  geometry queries (e.g. drive-motor separation). Returns false when the
	 *  motor or its transform can't be resolved. */
	virtual bool GetMotorTransform(FName MotorId, FTransform& OutWorld) const = 0;
};
