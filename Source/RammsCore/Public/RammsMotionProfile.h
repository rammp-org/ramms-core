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

	/** FFollower's delay, in command intervals, on top of its jitter margin
	 *  (FCommandInterval::Delay): the line to a command is drawn by the time
	 *  the target gets there when the commands come on time. (1.3 intervals
	 *  before 2026-11-20: 0.08 s of delay at 16 Hz on every command.) A command
	 *  that comes late finds the target already at the last one's, and it
	 *  holds there until the command comes: the motion stutters on uneven
	 *  frames unless a jitter margin is given (FollowJitterMargin). */
	constexpr double FollowDelayIntervals = 1.0;

	/** How many of the commands' measured jitters (FCommandInterval::Jitter)
	 *  FFollower's delay runs behind them on top of FollowDelayIntervals:
	 *  none by default -- the jitter estimate follows the late frames, and in
	 *  a capped 16 Hz editor two of them came to 0.4 interval, more than the
	 *  1.3's whole margin; 2 for a frame rate that is uneven enough to stutter
	 *  (Ramms.Motor.FollowJitterMargin). */
	constexpr double FollowJitterMargin = 0.0;

	/** How fast FFollower's estimate of the command interval follows each new
	 *  one (the weight of the newest): slowly, as every change of the delay
	 *  moves the target in time. */
	constexpr double IntervalSmoothing = 0.03;

	/** How fast the jitter estimate follows a gap further off the interval
	 *  than it has (a late frame: soon) and one nearer (slowly). */
	constexpr double JitterRiseSmoothing = 0.1;
	constexpr double JitterFallSmoothing = 0.02;

	/** How fast (s of delay per s) FFollower's delay may grow and shrink
	 *  toward its commands' (FCommandInterval::Delay): the target slows by as
	 *  much while it grows and speeds up by as much while it shrinks. Stepped
	 *  at each command instead, the target jumped back a little every frame
	 *  and the setpoint's rate dipped to close the gap. */
	constexpr double DelayRiseRate = 0.1;
	constexpr double DelayFallRate = 0.005;

	/**
	 * A command stream's interval and jitter (s), both smoothed: the delay a
	 * follower of the stream needs (Delay) so that a command that comes late
	 * still finds the line drawn to it. FFollower keeps one of its commands;
	 * the curb controller and its executor keep one of their frames, to know
	 * the delay their commands run behind.
	 */
	struct RAMMSCORE_API FCommandInterval
	{
		/** The interval (s); 0 before the first gap. */
		double Interval = 0.0;
		/** The gaps' mean distance from the interval (s). */
		double Jitter = 0.0;

		/** One gap (s) between two commands. A gap over
		 *  MaxInterpolationGapSeconds is a silence, not an interval: left out.
		 *  The first gap starts the jitter at FirstJitterShare of itself, so the
		 *  delay starts where it once stood (1.3 intervals) and comes down as
		 *  the frames show themselves even. */
		void Add(double Gap);

		/** The delay (s): Ramms.Motor.FollowDelayIntervals intervals and
		 *  Ramms.Motor.FollowJitterMargin jitters, at most
		 *  MaxInterpolationGapSeconds; 0 before the first gap. */
		double Delay() const;

		/** The delay as a follower runs it (FFollower::Delay): Delay, slewed
		 *  toward as time goes on (DelayRiseRate / DelayFallRate) and taken
		 *  outright the first time there is one. Advance moves it on by Dt
		 *  (s); a stream's other readers (the curb's controller and executor)
		 *  advance theirs with their frames, so the delay they plan with is the
		 *  one the follower runs, not the estimate it slews toward. */
		void   Advance(double Dt);
		double SlewedDelay() const { return bSlewed ? Slewed : Delay(); }
		/** Start the slew from Delay (s): a reader that begins again (each
		 *  crossing's executor) takes up where a longer-lived one stands. */
		void Seed(double Delay)
		{
			Slewed = Delay;
			bSlewed = Delay > 0.0;
		}

		static constexpr double FirstJitterShare = 0.15;

	private:
		double Slewed = 0.0;
		bool   bSlewed = false;
	};

	/**
	 * A position command that arrives once per game frame, followed by a
	 * setpoint stepped every physics step. Each command is a point, stamped
	 * with the time it was first seen; the target runs along the line through
	 * those points its delay behind (FCommandInterval::Delay: FollowDelayIntervals
	 * of the commands' interval and FollowJitterMargin of their jitter, both
	 * measured and smoothed), and the setpoint follows the target. The
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

		/** How far (s) the target runs behind the commands: its commands'
		 *  delay (FCommandInterval::Delay), slewed (DelayRiseRate /
		 *  DelayFallRate) as Step goes. */
		double Delay() const;

	private:
		struct FPoint
		{
			double Time = 0.0;
			double Value = 0.0;
		};
		/** The commands the target has yet to pass, and the one it is leaving. */
		TArray<FPoint, TInlineAllocator<16>> Points;
		/** The commands' interval and jitter (smoothed; none before the second
		 *  command), and the delay the target runs behind, slewed toward theirs
		 *  (FCommandInterval::SlewedDelay). */
		FCommandInterval Cadence;
	};
} // namespace RammsMotion
