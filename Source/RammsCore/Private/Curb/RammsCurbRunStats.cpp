// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbRunStats.h"

namespace RammsCurb
{
	// --- FMotionWindow -----------------------------------------------------------

	void FMotionWindow::Add(double Time, const FVector2D& Position, const FVector2D& InForward)
	{
		if (Samples.Num() > 0)
		{
			if (Time < Samples.Last().Time)
			{
				// The clock went back: a reset, and the robot is somewhere else.
				Samples.Reset();
			}
			else if (Time <= Samples.Last().Time)
			{
				Samples.Last().Position = Position;
				Forward = InForward;
				return;
			}
		}
		Samples.Add({ Time, Position });
		Forward = InForward;
		// Keep the newest sample that is at least the window old, and nothing
		// older: the speed then always spans the whole window, or a little more.
		while (Samples.Num() > 2 && Time - Samples[1].Time >= SpeedWindowSeconds)
		{
			Samples.RemoveAt(0);
		}
	}

	void FMotionWindow::Reset()
	{
		Samples.Reset();
	}

	bool FMotionWindow::IsFull() const
	{
		return Samples.Num() >= 2 && Samples.Last().Time - Samples[0].Time >= SpeedWindowSeconds - 1e-4;
	}

	float FMotionWindow::GetSpeed() const
	{
		if (Samples.Num() < 2)
		{
			return 0.0f;
		}
		const double Span = Samples.Last().Time - Samples[0].Time;
		if (Span <= 0.0)
		{
			return 0.0f;
		}
		return static_cast<float>(FVector2D::DotProduct(Samples.Last().Position - Samples[0].Position, Forward) / Span);
	}

	// --- FRunStatsTracker ----------------------------------------------------------

	void FRunStatsTracker::Begin(float PlannedMinSpeed, float PlannedMaxTiltDeg, float PlannedCrossingSeconds,
		TOptional<float> PlannedMinMarginCm, float CruiseFloorSpeed)
	{
		*this = FRunStatsTracker();
		Stats.PlannedMinSpeed = PlannedMinSpeed;
		Stats.PlannedMaxTiltDeg = PlannedMaxTiltDeg;
		Stats.PlannedCrossingSeconds = PlannedCrossingSeconds;
		Stats.MinPlannedMarginCm = PlannedMinMarginCm.Get(0.0f);
		bPlannedMargin = PlannedMinMarginCm.IsSet();
		CruiseFloor = CruiseFloorSpeed;
	}

	void FRunStatsTracker::Tick(const FRunTick& In)
	{
		if (bEnded)
		{
			return;
		}
		if (In.bLegsMoved && LegsMovedAt < 0.0)
		{
			LegsMovedAt = In.Elapsed;
		}
		if (LegsMovedAt >= 0.0)
		{
			Stats.CrossingSeconds = static_cast<float>(In.Elapsed - LegsMovedAt);
		}

		TickSpeed(In);

		Stats.MaxAbsPitchDeg = FMath::Max(Stats.MaxAbsPitchDeg, FMath::Abs(In.PitchDeg));
		Stats.MaxAbsRollDeg = FMath::Max(Stats.MaxAbsRollDeg, FMath::Abs(In.RollDeg));
		Stats.MaxInclinationDeg = FMath::Max(Stats.MaxInclinationDeg, static_cast<float>(InclinationDeg(In.PitchDeg, In.RollDeg)));
		if (In.bAttitudePlanned)
		{
			const float Error = FMath::Max(FMath::Abs(In.PitchDeg - In.PlannedPitchDeg), FMath::Abs(In.RollDeg - In.PlannedRollDeg));
			Stats.MaxAttitudeErrorDeg = FMath::Max(Stats.MaxAttitudeErrorDeg, Error);
		}
		Stats.MaxLegErrorCm = FMath::Max(Stats.MaxLegErrorCm, In.LegErrorCm);
		if (In.SwingClearanceCm.IsSet())
		{
			Stats.MinSwingClearanceCm = bSwingSeen ? FMath::Min(Stats.MinSwingClearanceCm, In.SwingClearanceCm.GetValue())
												   : In.SwingClearanceCm.GetValue();
			bSwingSeen = true;
		}
		if (In.MeasuredMarginCm.IsSet())
		{
			Stats.MinMeasuredMarginCm = bMarginSeen ? FMath::Min(Stats.MinMeasuredMarginCm, In.MeasuredMarginCm.GetValue())
													: In.MeasuredMarginCm.GetValue();
			bMarginSeen = true;
		}
		Stats.LateralDriftCm = In.LateralDriftCm;
		Stats.YawChangeDeg = In.YawChangeDeg;
	}

	void FRunStatsTracker::TickSpeed(const FRunTick& In)
	{
		if (!bUnderWay)
		{
			if (!In.bMeasuredSpeedValid || In.MeasuredSpeed < FirstMotionSpeedCmS)
			{
				return;
			}
			bUnderWay = true;
			MotionMinCommanded = In.CommandedSpeed;
			MotionMinMeasured = In.MeasuredSpeed;
			RisePeak = In.MeasuredSpeed;
		}

		MotionMinCommanded = FMath::Min(MotionMinCommanded, In.CommandedSpeed);
		if (In.bMeasuredSpeedValid)
		{
			MotionMinMeasured = FMath::Min(MotionMinMeasured, In.MeasuredSpeed);
			if (!bRisen)
			{
				// A standing start passes through every speed below the floor on
				// its way up; none of them is the robot slowing. The rise is over
				// once it reaches the floor (at once, for a robot already rolling)
				// or turns back down short of it.
				if (In.MeasuredSpeed >= CruiseFloor || In.MeasuredSpeed <= RisePeak - RiseDropCmS)
				{
					bRisen = true;
					RisenMinCommanded = In.CommandedSpeed;
					RisenMinMeasured = In.MeasuredSpeed;
				}
				RisePeak = FMath::Max(RisePeak, In.MeasuredSpeed);
			}
			if (In.MeasuredSpeed < StopSpeedCmS)
			{
				Stats.SecondsBelowStopSpeed += In.DeltaSeconds;
				if (BelowStopSince < 0.0)
				{
					BelowStopSince = In.Elapsed;
				}
				// Timed from the first tick below to this one, so a single slow
				// tick is never a stop however long it was.
				if (!bStopCounted && In.Elapsed - BelowStopSince >= MinStopSeconds - 1e-6)
				{
					++Stats.Stops;
					bStopCounted = true;
				}
			}
			else
			{
				BelowStopSince = -1.0;
				bStopCounted = false;
			}
		}
		if (bRisen)
		{
			RisenMinCommanded = FMath::Min(RisenMinCommanded, In.CommandedSpeed);
			if (In.bMeasuredSpeedValid)
			{
				RisenMinMeasured = FMath::Min(RisenMinMeasured, In.MeasuredSpeed);
			}
			Stats.MinCommandedSpeed = RisenMinCommanded;
			Stats.MinMeasuredSpeed = RisenMinMeasured;
		}
		else
		{
			Stats.MinCommandedSpeed = MotionMinCommanded;
			Stats.MinMeasuredSpeed = MotionMinMeasured;
		}
	}

	void FRunStatsTracker::SetPlan(float PlannedMinSpeed, float PlannedMaxTiltDeg, float PlannedCrossingSeconds,
		TOptional<float> PlannedMinMarginCm)
	{
		Stats.PlannedMinSpeed = PlannedMinSpeed;
		Stats.PlannedMaxTiltDeg = PlannedMaxTiltDeg;
		Stats.PlannedCrossingSeconds = PlannedCrossingSeconds;
		Stats.MinPlannedMarginCm = PlannedMinMarginCm.Get(0.0f);
		bPlannedMargin = PlannedMinMarginCm.IsSet();
	}

	void FRunStatsTracker::AddPlannedStop()
	{
		if (!bEnded)
		{
			++Stats.PlannedStops;
		}
	}

	void FRunStatsTracker::AddHold()
	{
		if (!bEnded)
		{
			++Stats.Holds;
		}
	}

	void FRunStatsTracker::AddReplan()
	{
		if (!bEnded)
		{
			++Stats.Replans;
		}
	}

	void FRunStatsTracker::AddReplan(float PlannedMinSpeed, TOptional<float> PlannedMinMarginCm)
	{
		if (bEnded)
		{
			return;
		}
		++Stats.Replans;
		Stats.PlannedMinSpeed = FMath::Min(Stats.PlannedMinSpeed, PlannedMinSpeed);
		if (PlannedMinMarginCm.IsSet())
		{
			Stats.MinPlannedMarginCm = bPlannedMargin ? FMath::Min(Stats.MinPlannedMarginCm, PlannedMinMarginCm.GetValue())
													  : PlannedMinMarginCm.GetValue();
			bPlannedMargin = true;
		}
	}

	void FRunStatsTracker::End(double Elapsed, float PitchDeg, float RollDeg)
	{
		if (bEnded)
		{
			return;
		}
		bEnded = true;
		Stats.FinalPitchDeg = PitchDeg;
		Stats.FinalRollDeg = RollDeg;
		// A crossing stopped by a tilt ends on a tick that was not counted:
		// the tilt that stopped it is still the largest, moving or not.
		Stats.MaxAbsPitchDeg = FMath::Max(Stats.MaxAbsPitchDeg, FMath::Abs(PitchDeg));
		Stats.MaxAbsRollDeg = FMath::Max(Stats.MaxAbsRollDeg, FMath::Abs(RollDeg));
		Stats.MaxInclinationDeg = FMath::Max(Stats.MaxInclinationDeg, static_cast<float>(InclinationDeg(PitchDeg, RollDeg)));
		if (LegsMovedAt >= 0.0)
		{
			Stats.CrossingSeconds = static_cast<float>(Elapsed - LegsMovedAt);
		}
	}
} // namespace RammsCurb
