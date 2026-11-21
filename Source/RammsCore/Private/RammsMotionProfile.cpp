// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMotionProfile.h"
#include "HAL/IConsoleManager.h"

namespace RammsMotion
{
	/** StepToward, and with bSoftArrival a setpoint that reaches the target
	 *  still moving faster than it can stop in the step passes it and comes
	 *  back, rather than stopping on it at once. */
	static double StepTo(double& Setpoint, double& Rate, double Target, double MaxRate, double MaxAccel, double Dt, bool bSoftArrival)
	{
		if (!(Dt > 0.0) || !FMath::IsFinite(Target))
		{
			return Setpoint;
		}
		const double D = Target - Setpoint;
		if (MaxRate <= 0.0 && MaxAccel <= 0.0)
		{
			Setpoint = Target;
			Rate = 0.0;
			return Setpoint;
		}
		if (MaxAccel <= 0.0)
		{
			// A pure rate limit: the whole of the way when it fits in the step.
			const double Most = MaxRate * Dt;
			const double Step = FMath::Clamp(D, -Most, Most);
			Rate = Step / Dt;
			Setpoint += Step;
			return Setpoint;
		}
		// The fastest rate this step may end at and still stop at the target:
		// the setpoint moves by the mean of the old and new rates (midpoint), and
		// what is left after that must be the new rate's stopping distance,
		// v^2 / 2a -- the positive root of v^2 / 2a + v dt / 2 = Dist - Rate dt / 2.
		// Planned this way the rate reaches 0 as the setpoint reaches the
		// target (a plan from the distance alone lands a step late, still
		// moving, and the stop then breaks the acceleration limit).
		const double Sign = D >= 0.0 ? 1.0 : -1.0;
		const double Dist = FMath::Abs(D);
		const double Left = Dist - 0.5 * Rate * Sign * Dt;
		const double Half = 0.5 * MaxAccel * Dt;
		double		 Want = -Half + FMath::Sqrt(Half * Half + 2.0 * MaxAccel * FMath::Max(0.0, Left));
		if (MaxRate > 0.0)
		{
			Want = FMath::Min(Want, MaxRate);
		}
		Want *= Sign;
		const double DRate = MaxAccel * Dt;
		const double NewRate = FMath::Clamp(Want, Rate - DRate, Rate + DRate);
		double		 Step = 0.5 * (Rate + NewRate) * Dt;
		// Planned from rest it arrives all but stopped; arriving faster (a
		// target that stopped short of it, moving), a soft arrival passes it at
		// the acceleration limit instead.
		const bool bLands = !bSoftArrival || FMath::Abs(Rate) <= DRate;
		Rate = NewRate;
		if (bLands && ((Sign > 0.0 && Step >= D) || (Sign < 0.0 && Step <= D)))
		{
			Step = D; // the last step lands on the target
			Rate = 0.0;
		}
		Setpoint += Step;
		if (FMath::Abs(Target - Setpoint) < 1e-12)
		{
			Setpoint = Target;
			Rate = 0.0;
		}
		return Setpoint;
	}

	double StepToward(double& Setpoint, double& Rate, double Target, double MaxRate, double MaxAccel, double Dt)
	{
		return StepTo(Setpoint, Rate, Target, MaxRate, MaxAccel, Dt, /*bSoftArrival=*/false);
	}

	double StepTowardMoving(double& Setpoint, double& Rate, double Target, double TargetRate, double MaxRate, double MaxAccel,
		double Dt)
	{
		if (TargetRate == 0.0)
		{
			return StepTo(Setpoint, Rate, Target, MaxRate, MaxAccel, Dt, /*bSoftArrival=*/true);
		}
		if (!(Dt > 0.0) || !FMath::IsFinite(Target) || !FMath::IsFinite(TargetRate))
		{
			return Setpoint;
		}
		const double Ahead = Target + TargetRate * Dt;
		if (MaxRate > 0.0 && FMath::Abs(TargetRate) >= MaxRate)
		{
			return StepTo(Setpoint, Rate, Ahead, MaxRate, MaxAccel, Dt, /*bSoftArrival=*/true);
		}
		if (MaxRate <= 0.0 && MaxAccel <= 0.0)
		{
			Setpoint = Ahead;
			Rate = TargetRate;
			return Setpoint;
		}
		// In the target's frame the target stands still: close the gap as to a
		// fixed one, then move with it. The target's own rate leaves the rest of
		// MaxRate for closing; its acceleration is nil within a step.
		double Gap = Setpoint - Target;
		double Closing = Rate - TargetRate;
		StepTo(Gap, Closing, 0.0, MaxRate > 0.0 ? MaxRate - FMath::Abs(TargetRate) : 0.0, MaxAccel, Dt, /*bSoftArrival=*/true);
		Setpoint = Ahead + Gap;
		Rate = TargetRate + Closing;
		return Setpoint;
	}

	void FFollower::Reset(double Value)
	{
		Setpoint = Value;
		Rate = 0.0;
		Points.Reset();
		Cadence = FCommandInterval();
		DelayNow = 0.0;
		bDelayNow = false;
	}

	static TAutoConsoleVariable<float> CVarFollowDelayIntervals(TEXT("Ramms.Motor.FollowDelayIntervals"), static_cast<float>(FollowDelayIntervals),
		TEXT("How far (in command intervals) a physics-rate position profile runs behind its commands (RammsMotion::FFollower), "
			 "on top of its jitter margin (Ramms.Motor.FollowJitterMargin). More: smoother through uneven frames, slower to "
			 "react. 0: eased toward the latest command at once."),
		ECVF_Default);
	static TAutoConsoleVariable<float> CVarFollowJitterMargin(TEXT("Ramms.Motor.FollowJitterMargin"), static_cast<float>(FollowJitterMargin),
		TEXT("How many of the commands' measured jitters (their gaps' mean distance from the interval) a physics-rate position "
			 "profile runs behind its commands on top of Ramms.Motor.FollowDelayIntervals: even frames add next to nothing, "
			 "uneven ones as much as the late ones need."),
		ECVF_Default);

	void FCommandInterval::Add(double Gap)
	{
		if (!(Gap > 0.0) || Gap > MaxInterpolationGapSeconds)
		{
			return;
		}
		if (Interval <= 0.0)
		{
			Interval = Gap;
			Jitter = FirstJitterShare * Gap;
			return;
		}
		const double Off = FMath::Abs(Gap - Interval);
		Jitter += (Off > Jitter ? JitterRiseSmoothing : JitterFallSmoothing) * (Off - Jitter);
		Interval += IntervalSmoothing * (Gap - Interval);
	}

	double FCommandInterval::Delay() const
	{
		if (Interval <= 0.0)
		{
			return 0.0;
		}
		const double Intervals = FMath::Max(0.0f, CVarFollowDelayIntervals.GetValueOnAnyThread());
		const double Margin = FMath::Max(0.0f, CVarFollowJitterMargin.GetValueOnAnyThread());
		return FMath::Min(Intervals * Interval + Margin * Jitter, MaxInterpolationGapSeconds);
	}

	double FFollower::Delay() const
	{
		return bDelayNow ? DelayNow : Cadence.Delay();
	}

	double FFollower::TargetAt(double Now, double* OutRate) const
	{
		if (OutRate)
		{
			*OutRate = 0.0;
		}
		if (Points.Num() == 0)
		{
			return Setpoint;
		}
		const double At = Now - Delay();
		if (At <= Points[0].Time)
		{
			return Points[0].Value;
		}
		for (int32 i = 1; i < Points.Num(); ++i)
		{
			const FPoint& A = Points[i - 1];
			const FPoint& B = Points[i];
			if (At <= B.Time)
			{
				const double Slope = (B.Value - A.Value) / (B.Time - A.Time);
				if (OutRate)
				{
					*OutRate = Slope;
				}
				return A.Value + Slope * (At - A.Time);
			}
		}
		return Points.Last().Value;
	}

	void FFollower::Command(double NewTarget, double Now)
	{
		if (Points.Num() > 0)
		{
			const double Gap = Now - Points.Last().Time;
			if (Gap <= 0.0)
			{
				Points.Last().Value = NewTarget; // the same instant: the later command wins
				return;
			}
			if (Gap > MaxInterpolationGapSeconds)
			{
				// After a silence: a new line, from where the target stands, to
				// the command; not one drawn back across the silence.
				const double Here = TargetAt(Now);
				Points.Reset();
				Points.Add({ Now - 1e-9, Here });
			}
			else
			{
				Cadence.Add(Gap);
			}
		}
		// The points the target has passed go, but for the one it is leaving
		// and a delay's worth more (the delay grows as the interval does);
		// within the inline storage, so the physics thread never allocates.
		const double At = Now - 2.0 * Delay();
		while (Points.Num() > 1 && Points[1].Time <= At)
		{
			Points.RemoveAt(0, EAllowShrinking::No);
		}
		if (Points.Num() >= 16)
		{
			Points.RemoveAt(0, Points.Num() - 15, EAllowShrinking::No);
		}
		Points.Add({ Now, NewTarget });
	}

	double FFollower::Step(double Now, double Dt, double MaxRate, double MaxAccel)
	{
		if (Points.Num() == 0)
		{
			return Setpoint;
		}
		// The delay slewed toward the commands' (the first one taken outright).
		const double Want = Cadence.Delay();
		if (!bDelayNow)
		{
			if (Want > 0.0)
			{
				DelayNow = Want;
				bDelayNow = true;
			}
		}
		else if (Dt > 0.0)
		{
			DelayNow += FMath::Clamp(Want - DelayNow, -DelayFallRate * Dt, DelayRiseRate * Dt);
		}
		double		 TargetRate = 0.0;
		const double Target = TargetAt(Now, &TargetRate);
		return StepTowardMoving(Setpoint, Rate, Target, TargetRate, MaxRate, MaxAccel, Dt);
	}
} // namespace RammsMotion
