// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsMotorSpec.h"

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
	 *  joint (a Chaos constraint drive is disabled; a MuJoCo position actuator
	 *  is parked at the joint's current position). Torque motors have nothing
	 *  to release. Returns false when this backend cannot release the motor —
	 *  the caller must then treat it as still driven. Default: unsupported. */
	virtual bool ReleaseMotor(FName MotorId) { return false; }

	/** World transform of the motor's joint/body, for the base component's
	 *  geometry queries (e.g. drive-motor separation). Returns false when the
	 *  motor or its transform can't be resolved. */
	virtual bool GetMotorTransform(FName MotorId, FTransform& OutWorld) const = 0;

	/** Seconds of simulated time the physics has advanced, or negative when
	 *  this backend does not keep its own clock (it then runs on the game's).
	 *  A simulator that cannot keep up with real time falls behind the game
	 *  clock, so anything timing motion against physics must use this. */
	virtual double GetSimulationTime() const { return -1.0; }

	/**
	 * Hold a torque motor's joint at TargetRate (the joint's own sense: rad/s,
	 * or cm/s for a slide joint) with a PI loop the backend runs at the physics
	 * rate, its output clamped to [TorqueMin, TorqueMax] (no clamp when
	 * min >= max). The base calls this every tick a speed is commanded, with
	 * the latest target. False means the backend cannot, and the base closes
	 * the loop itself once per game tick.
	 *
	 * The physics rate is the point. A real hub-motor driver closes its speed
	 * loop at kilohertz; once per game frame, a loop stiff enough to push the
	 * robot rings on a light wheel -- an unloaded omni hub at a 20 ms frame
	 * sees Kp * dt / inertia near 10, five times past where a discrete loop
	 * goes unstable -- and the wheel chatters instead of driving.
	 */
	virtual bool SetVelocityDrive(FName MotorId, float TargetRate, const FRammsVelocityGains& Gains,
		float TorqueMin, float TorqueMax)
	{
		return false;
	}

	/** Stop a loop SetVelocityDrive started. Its last output is not held. */
	virtual void ClearVelocityDrive(FName MotorId) {}
};
