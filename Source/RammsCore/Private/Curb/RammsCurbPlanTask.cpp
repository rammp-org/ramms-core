// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbPlanTask.h"

#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"

namespace
{
	TAutoConsoleVariable<int32> CVarCurbPlanInStep(TEXT("Ramms.Curb.PlanInStep"), 0,
		TEXT("1: a plan is waited for where it is asked for (the game thread held until it is done), so it is taken on the same "
			 "tick in every run: for runs that must repeat exactly, with the physics locked to the frames "
			 "(urlab.LockedFrameSeconds). 0 (the default): planned in the background and taken on the tick that finds it done, "
			 "which turns on the machine's load."),
		ECVF_Default);
} // namespace

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
		if (CVarCurbPlanInStep.GetValueOnAnyThread() != 0)
		{
			// (Ramms.Curb.PlanInStep: done before the caller goes on, so the tick
			// that takes it is the same in every run.)
			Task.Wait();
		}
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
