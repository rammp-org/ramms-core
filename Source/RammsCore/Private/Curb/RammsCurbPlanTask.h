// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Curb/RammsCurbPlanner.h"
#include "Curb/RammsCurbSchedule.h"
#include "Tasks/Task.h"

/**
 * A per-wheel plan made off the game thread. A plan takes tens to a few
 * hundred milliseconds (the plan lines say how long), too long for a tick, so
 * a re-plan -- the terrain refined on the approach, the robot slowed or
 * nudged -- runs as a UE::Tasks task on a copy of its input; the game thread
 * polls for it and swaps the schedule in when it is done. Not reflected, not
 * exported.
 */
namespace RammsCurb
{
	class FPlanTask
	{
	public:
		FPlanTask() = default;
		~FPlanTask();

		FPlanTask(const FPlanTask&) = delete;
		FPlanTask& operator=(const FPlanTask&) = delete;

		/** Plan a copy of In in the background. A plan already under way is
		 *  left to finish and its result dropped: only the latest counts. */
		void Launch(const FPlannerInput& In);

		/** Launched and not yet taken (running or done). */
		bool IsPending() const { return bPending; }

		/** Done and not yet taken. */
		bool IsReady() const;

		/** The finished schedule (moved out); false while it is still running or
		 *  nothing is pending. */
		bool Take(FCurbSchedule& Out);

		/** Block until the pending plan is done (tests; shutdown). */
		void Wait() const;

		/** Seconds from launch to now (or to when it was taken). */
		double PendingSeconds() const;

		/** How long Launch itself held the caller (s): the copy and the hand-off. */
		double LaunchSeconds() const { return LastLaunchSeconds; }

	private:
		UE::Tasks::TTask<FCurbSchedule> Task;
		bool							bPending = false;
		double							LaunchedAt = 0.0;
		double							TakenAt = 0.0;
		double							LastLaunchSeconds = 0.0;
	};
} // namespace RammsCurb
