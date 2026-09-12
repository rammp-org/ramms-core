// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

struct FWheelState;
struct FMotorParameters;
class URammsDifferentialDriveController;

/** Which wheel of the differential pair a backend call refers to. */
enum class ERammsDriveWheel : uint8
{
	Left,
	Right
};

/**
 * The physics I/O of a differential-drive base, abstracted away from the engine
 * that simulates it.
 *
 * The controller (URammsDifferentialDriveController) owns all the
 * backend-neutral behavior — input, the SetDriveInput/SetExternalDriveInput
 * arbitration, the torque/velocity control law, PID, braking decision, and
 * odometry integration — and delegates only the three operations that actually
 * touch a physics body to a backend: read a wheel's live state, apply the
 * control law's requested torque, and apply braking. That lets the same
 * controller drive the Chaos skeletal-wheel rig today
 * (FRammsChaosSkeletalDriveBackend) and a URLab/MuJoCo articulation later,
 * with its whole Blueprint / Remote-Control API surface unchanged.
 *
 * A backend reads configuration (wheel bone/actuator names, radius, motor and
 * slip/traction parameters) from the controller handed to Initialize; it does
 * not duplicate that state.
 */
class RAMMSCORE_API IRammsDriveBackend
{
public:
	virtual ~IRammsDriveBackend() = default;

	/**
	 * Resolve the simulated base (skeletal mesh / articulation + actuators) and
	 * do one-time setup (e.g. max angular velocity). Returns false when the base
	 * cannot be driven, in which case the controller's per-tick calls are no-ops.
	 */
	virtual bool Initialize(URammsDifferentialDriveController& Controller) = 0;

	/**
	 * Read the wheel's live state — angular/linear/lateral velocity, and (when the
	 * controller's slip/traction options are on) slip ratio, suspension load and
	 * surface friction. Leaves OutState untouched when the wheel isn't simulating.
	 */
	virtual void ReadWheelState(ERammsDriveWheel Wheel, FWheelState& OutState) = 0;

	/**
	 * Apply the control law's requested torque (N·m) to the wheel, through the
	 * motor curve and — for Chaos — the slip/traction model. WheelState carries
	 * the state read this tick and receives the resolved AppliedTorque.
	 */
	virtual void ApplyWheelTorque(ERammsDriveWheel Wheel, float RequestedTorque,
		const FMotorParameters& MotorParams, FWheelState& WheelState) = 0;

	/** Apply braking damping to the wheel (proportional to its angular velocity). */
	virtual void ApplyBrake(ERammsDriveWheel Wheel, FWheelState& WheelState) = 0;
};
