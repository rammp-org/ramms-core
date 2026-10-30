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

	/**
	 * StepToward for a target moving at TargetRate (per second), Target being
	 * where it is at the start of the step: the setpoint closes on it under
	 * the same limits and then moves with it, rather than stopping where the
	 * target was. A target faster than MaxRate is chased at MaxRate. One that
	 * stops short of a setpoint still moving with it is passed by the
	 * setpoint's stopping distance, and come back to, at the acceleration
	 * limit (StepToward would stop on it at once); otherwise, with TargetRate
	 * 0, the same as StepToward.
	 */
	RAMMSCORE_API double StepTowardMoving(double& Setpoint, double& Rate, double Target, double TargetRate, double MaxRate,
		double MaxAccel, double Dt);

	/** The longest gap (s) between two commands that FFollower still draws a
	 *  line across, and the most its delay grows to: a command after a
	 *  longer silence is a new target, not the next point of a motion. */
	constexpr double MaxInterpolationGapSeconds = 0.15;

	/** FFollower's delay, in command intervals: more than one, so a frame
	 *  that comes late still finds the line drawn to its command. */
	constexpr double FollowDelayIntervals = 1.3;

	/** How fast FFollower's estimate of the command interval follows each new
	 *  one (the weight of the newest): slowly, as every change of the delay
	 *  moves the target in time. */
	constexpr double IntervalSmoothing = 0.03;

	/**
	 * A position command that arrives once per game frame, followed by a
	 * setpoint stepped every physics step. Each command is a point, stamped
	 * with the time it was first seen; the target runs along the line through
	 * those points FollowDelayIntervals command intervals behind (an estimate
	 * of the interval, smoothed), and the setpoint follows the target. The
	 * line's slopes come from the stamps, so frames of uneven length still
	 * make a smooth motion, where easing to each command as it came made a
	 * staircase at the frame rate -- a jitter buffer, the way a servo drive
	 * takes a slower position stream. The setpoint passes a command only
	 * where the commands stop dead while it moves, by its stopping distance
	 * at MaxAccel. The first command, and one after a silence over
	 * MaxInterpolationGapSeconds, start a new line; a jump faster than
	 * MaxRate is chased at it.
	 */
	struct RAMMSCORE_API FFollower
	{
		double Setpoint = 0.0;
		double Rate = 0.0;

		/** Hold Value, at rest, with no command. */
		void Reset(double Value);

		/** A command, first seen at time Now (s). */
		void Command(double NewTarget, double Now);

		/** One step from Now to Now + Dt; returns the setpoint. With no
		 *  command yet, the setpoint holds. */
		double Step(double Now, double Dt, double MaxRate, double MaxAccel);

		/** Where the target stands at time Now, and (OutRate) how fast it moves. */
		double TargetAt(double Now, double* OutRate = nullptr) const;

		/** How far (s) the target runs behind the commands. */
		double Delay() const;

	private:
		struct FPoint
		{
			double Time = 0.0;
			double Value = 0.0;
		};
		/** The commands the target has yet to pass, and the one it is leaving. */
		TArray<FPoint, TInlineAllocator<16>> Points;
		/** The command interval, smoothed (s); 0 before the second command. */
		double Interval = 0.0;
	};
} // namespace RammsMotion
