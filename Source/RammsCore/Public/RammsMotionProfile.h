// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * A setpoint eased toward a target no faster than an actuator moves: its rate
 * capped at MaxRate and changing no faster than MaxAccel (a trapezoid in the
 * rate, with the stop ahead of the target planned so it arrives at rest), or,
 * with no acceleration limit, a pure rate limit. What a real position servo
 * does with a step command, done here so a simulated servo with the gains to
 * follow anything moves no faster than the actuator it stands for.
 */
namespace RammsMotion
{
	/**
	 * Step Setpoint (and its Rate, per second) toward Target over Dt seconds,
	 * rate within MaxRate and acceleration within MaxAccel (either 0: no
	 * limit of that kind). With neither limit the setpoint jumps to the
	 * target. Returns the new setpoint.
	 */
	RAMMSCORE_API double StepToward(double& Setpoint, double& Rate, double Target, double MaxRate, double MaxAccel, double Dt);
} // namespace RammsMotion
