// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Curb/RammsCurbPlanner.h"
#include "Curb/RammsCurbSchedule.h"

/**
 * A kinematic dry run of a per-wheel plan, as plain C++
 * for the automation tests: the robot follows the schedule open loop, with no
 * physics. Progress integrates the planned speed under Acceleration and
 * Deceleration, braking onto each planned stop and resting there while the
 * centre wheels and the chassis re-pose (within CenterShiftSpeed and
 * BaseHeightSpeed) until the legs settle; every leg's command is a trapezoid
 * at LegSpeed and LegAcceleration towards where the plan wants it -- a
 * support leg on the plane under its wheel a servo lag ahead, a lifted one
 * over its clearance profile (to the most the window asks, then down as the
 * terrain under it falls away, then onto the far surface), each 5-bar
 * walking to the planned fore/aft -- with the base at the planned height and
 * attitude (pitch and roll: a tilted plan's legs reach for their surfaces
 * from the tilted base), and the leg follows its command with the servo lag.
 * Not reflected, not exported.
 *
 * What it checks every tick, from the end of the level-in on: a lifted wheel
 * inside its window stays above the terrain (its rim over the analytic
 * planes); no wheel the plan has on the ground is inside its keep-off window;
 * no set off the ground is one a six-wheeled robot cannot stand without (a
 * same-side corner and centre, both fronts and a centre, a centre and both
 * rears); the robot stands at least a margin inside the wheels on the ground
 * as they lag, its base at the planned attitude; the base the support legs'
 * lag would give stays within a tolerance of the plan's attitude; no 5-bar
 * is commanded outside its reach; and the speed never falls under the floor
 * once the robot has risen from rest, nor stops, but at a planned stop.
 */
namespace RammsCurb
{
	struct FDryRunOptions
	{
		/** The tick (s). */
		double Dt = 1.0 / 120.0;
		/** The legs lag their commands by ServoLagSeconds times this. */
		double LagScale = 1.0;
		/** Give up after this long (s). */
		double MaxSeconds = 60.0;
	};

	struct FDryRunResult
	{
		/** It reached the plan's end. */
		bool   bCompleted = false;
		double Seconds = 0.0;
		/** The least commanded speed once risen from rest (cm/s), and the stops
		 *  (2 cm/s or less for 0.2 s or more) after first motion -- both away
		 *  from the planned stops (the slowing into one and the rise out of it);
		 *  the planned stops made, and how long the robot rested at them (s). */
		double MinSpeedCmS = 0.0;
		int32  Stops = 0;
		int32  PlannedStops = 0;
		double PlannedDwellSeconds = 0.0;
		/** The most the base was inclined as planned (deg; its total tilt,
		 *  InclinationDeg). */
		double MaxPlannedTiltDeg = 0.0;
		/** The least gap between a lifted wheel's rim and the terrain inside its
		 *  window (cm). */
		double MinSwingClearanceCm = TNumericLimits<double>::Max();
		/** The least dynamic margin of the lagging robot on its planted wheels (cm). */
		double MinMarginCm = TNumericLimits<double>::Max();
		/** The largest pitch or roll (deg) the support legs' lag would give the
		 *  base, against the plan's. */
		double MaxAttitudeErrorDeg = 0.0;
		/** Ticks with a wheel on the ground inside its keep-off window, a
		 *  forbidden set off the ground, a 5-bar commanded outside its reach. */
		int32 KeepOffViolations = 0;
		int32 ForbiddenSets = 0;
		int32 KneeViolations = 0;
		/** The first few of them, said. */
		TArray<FString> Notes;
	};

	/** Follow Plan (made from In) to its end. */
	FDryRunResult DryRun(const FPlannerInput& In, const FCurbSchedule& Plan, const FDryRunOptions& Options = FDryRunOptions());
} // namespace RammsCurb
