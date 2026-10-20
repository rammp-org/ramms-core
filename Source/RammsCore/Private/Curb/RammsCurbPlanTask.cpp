// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbPlanTask.h"

#include "HAL/PlatformTime.h"

namespace RammsCurb
{
	FPlanTask::~FPlanTask()
	{
		// The task owns its input; nothing here outlives it, but a plan under way
		// at shutdown is waited for rather than left running against a module
		// that is going away.
		Wait();
	}

	void FPlanTask::Launch(const FPlannerInput& In)
	{
		const double Started = FPlatformTime::Seconds();
		// The planner keeps references into its input (the robot model's tables):
		// the task owns the copy for as long as it runs.
		TSharedRef<const FPlannerInput, ESPMode::ThreadSafe> Copy = MakeShared<const FPlannerInput, ESPMode::ThreadSafe>(In);
		Task = UE::Tasks::Launch(
			UE_SOURCE_LOCATION, [Copy]() { return PlanCrossing(*Copy); }, UE::Tasks::ETaskPriority::BackgroundHigh);
		bPending = true;
		LaunchedAt = Started;
		TakenAt = 0.0;
		LastLaunchSeconds = FPlatformTime::Seconds() - Started;
	}

	bool FPlanTask::IsReady() const
	{
		return bPending && Task.IsValid() && Task.IsCompleted();
	}

	bool FPlanTask::Take(FCurbSchedule& Out)
	{
		if (!IsReady())
		{
			return false;
		}
		Out = MoveTemp(Task.GetResult());
		Task = UE::Tasks::TTask<FCurbSchedule>();
		bPending = false;
		TakenAt = FPlatformTime::Seconds();
		return true;
	}

	void FPlanTask::Wait() const
	{
		if (Task.IsValid())
		{
			Task.Wait();
		}
	}

	double FPlanTask::PendingSeconds() const
	{
		if (LaunchedAt <= 0.0)
		{
			return 0.0;
		}
		return (bPending ? FPlatformTime::Seconds() : TakenAt) - LaunchedAt;
	}
} // namespace RammsCurb
