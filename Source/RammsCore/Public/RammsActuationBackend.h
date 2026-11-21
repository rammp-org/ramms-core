// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsMotorSpec.h"

class URammsRobotBaseComponent;

/**
 * What a motor's actuator put out over a trailing window of simulated time, in
 * its joint's own sense (N*m; N on a slide): IRammsActuationBackend::
 * GetMotorEffort. On a real robot, the drive's current times its torque
 * constant and gear.
 */
struct FRammsMotorEffort
{
	/** The time-weighted mean over the window, the sample of largest
	 *  magnitude (signed), and the last physics step's. */
	double MeanNm = 0.0;
	double PeakNm = 0.0;
	double LastNm = 0.0;
	/** The limits the last step ran under -- the actuator's own range within
	 *  any torque limit (SetMotorTorqueLimit) -- and the actuator's own
	 *  (authored) range; +-the largest double where there is none. */
	double LowerNm = 0.0;
	double UpperNm = 0.0;
	double RatedLowerNm = 0.0;
	double RatedUpperNm = 0.0;
	/** The simulated time the window covered (s), and of it how long the
	 *  force was at its lower and upper limits (within 0.5%) and at its own
	 *  rating on the side it pushed. */
	double Seconds = 0.0;
	double AtLowerSeconds = 0.0;
	double AtUpperSeconds = 0.0;
	double AtRatedSeconds = 0.0;
	/** The simulation time (s) of the last step recorded; -1 for none. */
	double LatestTime = -1.0;
};

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

	/**
	 * Whether a motor's joint is simulated as a joint: GetValue reports its
	 * coordinate -- for a continuous wheel its angle, unwrapped -- and, for a
	 * Torque motor, SetCommand applies a physical torque (N.m) about it. A
	 * controller that closes its own loop on a wheel's angle, with gains from
	 * the robot's mass (a drive mode holding the robot still), needs both.
	 * Default true, as GetValue and SetCommand are documented above; the Chaos
	 * backend's skeletal wheels are not (GetValue reads 0, and their command
	 * is the legacy x100 acceleration change).
	 */
	virtual bool SimulatesMotorJoint(FName MotorId) const { return true; }

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

	/** Stop a loop SetVelocityDrive started, integral and all; a later
	 *  SetVelocityDrive starts a new one from zero. Like the base's own loop
	 *  (ClearMotorVelocityCommand), this leaves whatever the loop last wrote
	 *  until the motor is commanded again. */
	virtual void ClearVelocityDrive(FName MotorId) {}

	/**
	 * Ease a position actuator's command to Target (the joint's own sense)
	 * every physics step, no faster than MaxVelocity and MaxAcceleration
	 * (either 0: no limit of that kind), each new target reached along a line
	 * over the time since the one before (RammsMotion::FFollower). The base
	 * calls this with every command to a profiled motor. StartFrom, when
	 * finite, is where a profile this backend is not already running starts,
	 * at rest. False means the backend cannot, and the base steps the profile
	 * itself once per game tick.
	 *
	 * The physics rate is the point, as for SetVelocityDrive: stepped once per
	 * frame, the setpoint a position servo chases moves in a staircase at the
	 * frame rate -- at 16 Hz a crank's jumps a quarter radian at a time -- and
	 * whatever commands through it moves differently at every frame rate.
	 */
	virtual bool SetPositionProfile(FName MotorId, float Target, float MaxVelocity, float MaxAcceleration, float StartFrom)
	{
		return false;
	}

	/** Stop a profile SetPositionProfile started; the actuator keeps its last
	 *  setpoint until it is commanded again. */
	virtual void ClearPositionProfile(FName MotorId) {}

	/**
	 * Engage or release a Torque motor's parking brake: its joint held where it
	 * stands, as a power chair's electromagnetic brakes hold its wheels while the
	 * joystick is at centre. For backends whose wheels a drive cannot hold with
	 * torque (SimulatesMotorJoint false). Engaging an engaged brake keeps it where
	 * it was set. A backend whose brake can slip holds up to the motor's
	 * FRammsMotorSpec::BrakeTorque. False: this backend has no brake for the
	 * motor.
	 */
	virtual bool SetMotorBrake(FName MotorId, bool bEngaged) { return false; }

	/** Whether SetMotorBrake would engage a brake on this motor now. Default
	 *  false; a backend that brakes a motor whose joint it simulates
	 *  (SimulatesMotorJoint) says so here, for a drive mode holding the robot
	 *  still to close the brakes once its own hold has brought it to rest. */
	virtual bool HasMotorBrake(FName MotorId) const { return false; }

	/**
	 * What the motor's actuator put out over the last WindowSeconds of
	 * simulated time (FRammsMotorEffort), averaged on the physics clock: a
	 * single step's force scatters far wider than what the leg carries. The
	 * first call on a motor starts recording it and may return false, as may a
	 * motor with nothing recorded yet. False: this backend cannot say.
	 */
	virtual bool GetMotorEffort(FName MotorId, double WindowSeconds, FRammsMotorEffort& Out) const { return false; }

	/**
	 * Hold a motor's output within [MinTorque, MaxTorque] (its joint's sense,
	 * N*m; N on a slide), applied every physics step inside the actuator's own
	 * range -- a drive's per-direction torque limit. MinTorque >= MaxTorque
	 * clears it; a side at +-the largest float keeps the actuator's own limit
	 * there. False: this backend cannot limit this motor.
	 */
	virtual bool SetMotorTorqueLimit(FName MotorId, float MinTorque, float MaxTorque) { return false; }
};
