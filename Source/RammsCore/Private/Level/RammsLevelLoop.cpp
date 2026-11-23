// Copyright Epic Games, Inc. All Rights Reserved.

#include "Level/RammsLevelLoop.h"

namespace RammsLevel
{
	namespace
	{
		/** The share of the gap a low-pass closes in Dt: all of it with no
		 *  time constant. (The curb's form.) */
		double LoopFilterShare(double Dt, double Seconds)
		{
			return Seconds > 0.0 ? FMath::Clamp(Dt / Seconds, 0.0, 1.0) : 1.0;
		}

		/** A timer has run its time. The hair is for a time counted up in
		 *  ticks: ten ticks of 0.1 s do not quite add up to 1. */
		bool LoopTimerHasRun(double Seconds, double Needed)
		{
			return Seconds >= Needed - 1.0e-9;
		}

		/** A straight line from (X0, Y0) to (X1, Y1), held at its ends; the
		 *  step at X0 when the ends are not in order. */
		double LoopRamp(double X, double X0, double Y0, double X1, double Y1)
		{
			if (X <= X0)
			{
				return Y0;
			}
			if (X >= X1 || X1 <= X0)
			{
				return Y1;
			}
			return Y0 + (Y1 - Y0) * (X - X0) / (X1 - X0);
		}

		/** The speed scale's floor. A floor of 0 would let the assist stop
		 *  the rider, which it must never do, so a tuning of 0 or less still
		 *  leaves a crawl. */
		double LoopSpeedFloor(const FTuning& Tuning)
		{
			return FMath::Clamp(Tuning.MinSpeedScale, 0.01, 1.0);
		}
	} // namespace

	FVector2D FAttitudeFilter::Tick(const FVector2D& MeasuredDeg, double Dt, double Seconds)
	{
		if (!bSeeded)
		{
			// The first reading is taken whole, whatever Dt is: a filter that
			// started from zero would report a level base that is not.
			Value = MeasuredDeg;
			bSeeded = true;
			return Value;
		}
		if (Dt <= 0.0)
		{
			return Value;
		}
		Value += (MeasuredDeg - Value) * LoopFilterShare(Dt, Seconds);
		return Value;
	}

	FVector2D FTerrainEstimate::Tick(const FVector2D& AttitudeDeg, const FVector2D& MeasuredCorrectionDeg, double Dt, double Seconds)
	{
		// The base reads A with the achieved correction taken out of it, so
		// the ground is A with that correction put back.
		return Filter.Tick(AttitudeDeg + MeasuredCorrectionDeg, Dt, Seconds);
	}

	/**
	 * WantedTiltGoal with the reach share given rather than found: the part of
	 * the rule that is this file's own, so the rule can be checked at any
	 * share without building a robot whose legs give exactly that share.
	 */
	FVector2D WantedTiltGoalForShare(const FVector2D& GroundDeg, double Share, const FTuning& Tuning)
	{
		const FVector2D Whole = GroundDeg - Tuning.LevelReferenceDeg;
		const FVector2D Got = ClampAxes(Whole, Tuning.MaxCorrectionDeg) * FMath::Clamp(Share, 0.0, 1.0);
		return Tuning.LevelReferenceDeg + ClampTotal(Whole - Got, Tuning.MaxTiltDeg);
	}

	FVector2D WantedTiltGoal(const FVector2D& GroundDeg, TConstArrayView<FLegReach> Legs, TConstArrayView<FClearancePoint> Points,
		const FTuning& Tuning)
	{
		// The limits are pulled in by the soft zone, so that resting on the
		// goal the correction sits short of the margin.
		const FVector2D Whole = GroundDeg - Tuning.LevelReferenceDeg;
		const double	Share = AllocateFraction(Legs, Points, Whole, Tuning, Tuning.SoftZoneCm);
		return WantedTiltGoalForShare(GroundDeg, Share, Tuning);
	}

	FVector2D FTiltTarget::Tick(const FVector2D& GoalDeg, double Dt, double MaxRateDegPerSec, double MaxAccelDegPerSec2)
	{
		if (Dt <= 0.0)
		{
			return Wanted;
		}
		const bool bRateLimit = MaxRateDegPerSec > 0.0;
		const bool bAccelLimit = MaxAccelDegPerSec2 > 0.0;
		if (!bRateLimit && !bAccelLimit)
		{
			Wanted = GoalDeg;
			Rate = FVector2D::ZeroVector;
			return Wanted;
		}

		const FVector2D ToGoal = GoalDeg - Wanted;
		const double	Distance = ToGoal.Size();
		const double	MaxRateChange = bAccelLimit ? MaxAccelDegPerSec2 * Dt : 0.0;

		// There, and slow enough to stop within this tick: stop on the goal
		// exactly, so a goal that stands still is reached and not crept up on.
		if (Distance <= 1.0e-9 && (!bAccelLimit || Rate.Size() <= MaxRateChange))
		{
			Wanted = GoalDeg;
			Rate = FVector2D::ZeroVector;
			return Wanted;
		}

		// The most speed that can still stop on the goal in whole ticks. The
		// continuous sqrt(2 a d) would be a tick too late, and overshoot.
		double Speed = 0.0;
		if (Distance > 0.0)
		{
			Speed = bAccelLimit ? FMath::Sqrt(FMath::Square(MaxRateChange) + 2.0 * MaxAccelDegPerSec2 * Distance) - MaxRateChange : Distance / Dt;
			if (bRateLimit)
			{
				Speed = FMath::Min(Speed, MaxRateDegPerSec);
			}
		}
		const FVector2D WantedRate = Distance > 0.0 ? ToGoal * (Speed / Distance) : FVector2D::ZeroVector;

		if (bAccelLimit)
		{
			FVector2D	 Change = WantedRate - Rate;
			const double ChangeSize = Change.Size();
			if (ChangeSize > MaxRateChange)
			{
				Change *= MaxRateChange / ChangeSize;
			}
			Rate += Change;
		}
		else
		{
			Rate = WantedRate;
		}
		if (bRateLimit)
		{
			const double RateSize = Rate.Size();
			if (RateSize > MaxRateDegPerSec)
			{
				Rate *= MaxRateDegPerSec / RateSize;
			}
		}
		Wanted += Rate * Dt;
		return Wanted;
	}

	FVector2D FLevelLoop::Tick(const FVector2D& AttitudeDeg, const FVector2D& WantedDeg, const FVector2D& GroundDeg, double Dt, bool bIntegrate,
		bool bSaturated, const FTuning& Tuning)
	{
		if (Dt <= 0.0)
		{
			return Correction;
		}
		const FVector2D Error(Deadband(AttitudeDeg.X - WantedDeg.X, Tuning.DeadbandDeg), Deadband(AttitudeDeg.Y - WantedDeg.Y, Tuning.DeadbandDeg));
		if (bIntegrate)
		{
			const FVector2D Step = Error * (Tuning.IntegralGain * Dt);
			// Against a limit the integral may only come back: a step with the
			// sign of the last correction would wind it further into a plane
			// the legs cannot give, to be unwound later as an overshoot. The
			// legs' reach holds both axes; an axis' own limit holds that axis
			// (a capped roll must not stop the pitch being trimmed).
			const bool bHeldX = bSaturated || FMath::Abs(Correction.X) >= FMath::Max(0.0, Tuning.MaxCorrectionDeg.X);
			const bool bHeldY = bSaturated || FMath::Abs(Correction.Y) >= FMath::Max(0.0, Tuning.MaxCorrectionDeg.Y);
			if (!bHeldX || Step.X * Correction.X < 0.0)
			{
				Integral.X += Step.X;
			}
			if (!bHeldY || Step.Y * Correction.Y < 0.0)
			{
				Integral.Y += Step.Y;
			}
		}
		const double IntegralMax = FMath::Max(0.0, Tuning.IntegralMaxDeg);
		Integral.X = FMath::Clamp(Integral.X, -IntegralMax, IntegralMax);
		Integral.Y = FMath::Clamp(Integral.Y, -IntegralMax, IntegralMax);

		Others = (GroundDeg - WantedDeg) * Tuning.FeedForwardGain + Error * Tuning.ProportionalGain;
		Correction = Others + Integral;
		return Correction;
	}

	void FLevelLoop::BackCalculate(const FVector2D& AskedDeg, const FVector2D& AchievedDeg, const FTuning& Tuning)
	{
		// Towards zero by what was cut, and no further: beyond its own size
		// the cut was of something the integral did not ask for.
		const auto GiveUp = [](double Held, double Cut) {
			if (Held > 0.0)
			{
				return FMath::Clamp(Held - Cut, 0.0, Held);
			}
			if (Held < 0.0)
			{
				return FMath::Clamp(Held - Cut, Held, 0.0);
			}
			return 0.0;
		};
		const double IntegralMax = FMath::Max(0.0, Tuning.IntegralMaxDeg);
		Integral.X = FMath::Clamp(GiveUp(Integral.X, AskedDeg.X - AchievedDeg.X), -IntegralMax, IntegralMax);
		Integral.Y = FMath::Clamp(GiveUp(Integral.Y, AskedDeg.Y - AchievedDeg.Y), -IntegralMax, IntegralMax);
		Correction = Others + Integral;
	}

	void FLevelLoop::StartFrom(const FVector2D& CorrectionDeg, const FVector2D& WantedDeg, const FVector2D& GroundDeg, const FTuning& Tuning)
	{
		// The feed-forward will ask for its own share of the correction on the
		// next tick, so the integral holds only the rest. Holding all of it
		// would apply the plane twice at a feed-forward gain of 1.
		const double IntegralMax = FMath::Max(0.0, Tuning.IntegralMaxDeg);
		Others = (GroundDeg - WantedDeg) * Tuning.FeedForwardGain;
		Integral = CorrectionDeg - Others;
		Integral.X = FMath::Clamp(Integral.X, -IntegralMax, IntegralMax);
		Integral.Y = FMath::Clamp(Integral.Y, -IntegralMax, IntegralMax);
		Correction = Others + Integral;
	}

	FVector2D SlewCorrection(TConstArrayView<FLegReach> Legs, const FVector2D& FromDeg, const FVector2D& ToDeg, double Dt,
		double MaxRateDegPerSec, const FTuning& Tuning, double& OutRateUse, bool& bOutLimited)
	{
		OutRateUse = 0.0;
		bOutLimited = false;
		if (Dt <= 0.0)
		{
			return FromDeg;
		}

		// 1. The plane's own rate.
		FVector2D Step = ToDeg - FromDeg;
		if (MaxRateDegPerSec > 0.0)
		{
			const double MaxStep = MaxRateDegPerSec * Dt;
			const double StepSize = Step.Size();
			if (StepSize > MaxStep)
			{
				Step *= MaxStep / StepSize;
				bOutLimited = true;
			}
		}
		if (Step.IsZero() || Legs.Num() == 0)
		{
			return FromDeg + Step;
		}

		FLegValues Before;
		FLegValues After;
		RigidOffsets(Legs, FromDeg, Before);
		RigidOffsets(Legs, FromDeg + Step, After);
		if (Before.Num() != Legs.Num() || After.Num() != Legs.Num())
		{
			return FromDeg + Step;
		}

		// 2. The busiest leg's share of its own rate. Measured before the cut,
		// so the speed cap sees how much faster the ground asks than the legs
		// can give. Every cut scales the whole step: the plane stays rigid.
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			if (Legs[i].MaxRateCmS > 0.0)
			{
				OutRateUse = FMath::Max(OutRateUse, FMath::Abs(After[i] - Before[i]) / (Dt * Legs[i].MaxRateCmS));
			}
		}
		// (A fraction of 0 or less would stop the plane for good; it is read
		// as "no leg-rate cut".)
		if (Tuning.LegRateFraction > 0.0 && OutRateUse > Tuning.LegRateFraction)
		{
			Step *= Tuning.LegRateFraction / OutRateUse;
			bOutLimited = true;
			RigidOffsets(Legs, FromDeg + Step, After);
			if (After.Num() != Legs.Num())
			{
				return FromDeg + Step;
			}
		}

		// 3. The soft zone: a support leg nearing a limit slows the plane down
		// to a tenth at the limit itself, so it is met gently. Only motion
		// towards the limit is slowed; coming away is free. Not reported as a
		// rate limit (the header says why).
		if (Tuning.SoftZoneCm > 0.0)
		{
			double Least = 1.0;
			for (int32 i = 0; i < Legs.Num(); ++i)
			{
				const FLegReach& Leg = Legs[i];
				if (!Leg.bSupport)
				{
					continue;
				}
				const double Move = After[i] - Before[i];
				double		 Room = 0.0;
				if (Move > 1.0e-9)
				{
					Room = FMath::Max(0.0, Leg.MaxZ - Tuning.LegRangeMarginCm - Leg.BaseZ) - Before[i];
				}
				else if (Move < -1.0e-9)
				{
					Room = Before[i] - FMath::Min(0.0, Leg.MinZ + Tuning.LegRangeMarginCm - Leg.BaseZ);
				}
				else
				{
					continue;
				}
				Room = FMath::Max(0.0, Room);
				if (Room < Tuning.SoftZoneCm)
				{
					Least = FMath::Min(Least, SoftZoneFloor + (1.0 - SoftZoneFloor) * Room / Tuning.SoftZoneCm);
				}
			}
			if (Least < 1.0)
			{
				Step *= Least;
			}
		}
		return FromDeg + Step;
	}

	double FSeatTrim::Tick(double BasePitchDeg, bool bRun, double Dt, const FTuning& Tuning)
	{
		if (!bSeeded)
		{
			Filtered = BasePitchDeg;
			bSeeded = true;
		}
		if (Dt <= 0.0)
		{
			return Trim;
		}
		// The filter runs whether or not the trim does, so the trim starts
		// from the pitch as it is, not as it was when it last ran.
		Filtered += (BasePitchDeg - Filtered) * LoopFilterShare(Dt, Tuning.SeatFilterSeconds);

		// Nose up at the base is answered nose down at the seat.
		const double MaxTrim = FMath::Max(0.0, Tuning.SeatTrimMaxDeg);
		const double Target = bRun ? FMath::Clamp(-Deadband(Filtered, SeatTrimDeadbandDeg), -MaxTrim, MaxTrim) : 0.0;
		const double MaxStep = FMath::Max(0.0, Tuning.SeatTrimRateDegPerSec) * Dt;
		Trim += FMath::Clamp(Target - Trim, -MaxStep, MaxStep);
		return Trim;
	}

	double FSpeedScale::Tick(double ErrorDeg, double MinMarginCm, bool bMoving, double RateUse, bool bPastReach, double Dt, const FTuning& Tuning)
	{
		if (Dt <= 0.0)
		{
			return Scale;
		}
		const double Floor = LoopSpeedFloor(Tuning);

		// Error: the base is off what it should be, so go slower until the
		// legs have caught up.
		double Target = bPastReach ? Floor : LoopRamp(ErrorDeg, Tuning.SpeedCapErrorDeg, 1.0, Tuning.SpeedCapErrorFullDeg, Floor);

		// Reach: little travel left, and the ground still changing. Standing
		// or driving steadily at the limit is no reason to stay slow.
		SinceMoving = bMoving ? 0.0 : SinceMoving + Dt;
		if (SinceMoving <= SpeedCapMovingHoldSeconds)
		{
			Target = FMath::Min(Target, LoopRamp(MinMarginCm, SpeedCapMarginFloorCm, Floor, SpeedCapMarginFullCm, 1.0));
		}

		// Rate: the ground asks the legs for more than their rate, so the
		// drive slows by as much.
		if (Tuning.LegRateFraction > 0.0 && RateUse > Tuning.LegRateFraction)
		{
			Target = FMath::Min(Target, FMath::Max(Floor, Tuning.LegRateFraction / RateUse));
		}

		if (Target < Scale)
		{
			Scale = FMath::Max(Target, Scale - SpeedScaleFallPerSec * Dt);
		}
		else
		{
			Scale = FMath::Min(Target, Scale + FMath::Max(0.0, Tuning.SpeedScaleRisePerSec) * Dt);
		}
		return Scale;
	}

	double FSpeedScale::Relax(double Dt, const FTuning& Tuning)
	{
		if (Dt <= 0.0)
		{
			return Scale;
		}
		SinceMoving += Dt;
		Scale = FMath::Min(1.0, Scale + FMath::Max(0.0, Tuning.SpeedScaleRisePerSec) * Dt);
		return Scale;
	}

	bool FQuiet::Tick(double SpeedCmS, double ErrorDeg, bool bStick, bool bMayRest, double Dt, const FTuning& Tuning)
	{
		if (Dt <= 0.0)
		{
			return bQuiet;
		}
		const bool bStill = FMath::Abs(SpeedCmS) < QuietSpeedCmS && !bStick && bMayRest;
		if (!bStill || ErrorDeg >= Tuning.WakeErrorDeg)
		{
			bQuiet = false;
			SettledSeconds = 0.0;
		}
		else if (ErrorDeg < Tuning.SettleErrorDeg)
		{
			SettledSeconds += Dt;
			if (LoopTimerHasRun(SettledSeconds, Tuning.SettleSeconds))
			{
				bQuiet = true;
			}
		}
		else
		{
			// Between the two errors: not settled enough to count towards a
			// quiet, not disturbed enough to end one. Without the gap a base
			// resting on the settle error would start and stop writing.
			SettledSeconds = 0.0;
		}
		return bQuiet;
	}

	bool FDivergence::Tick(double ErrorDeg, const FVector2D& AttitudeDeg, const FVector2D& CorrectionDeg, bool bCounts, double Dt, const FTuning& Tuning)
	{
		if (bDiverged || Dt <= 0.0)
		{
			return bDiverged;
		}
		// How far the base went the way its correction went, this tick. Kept
		// up on every tick, so that an episode's first tick has a last one.
		double WithLegs = 0.0;
		if (bHasLast)
		{
			const FVector2D LegsMoved = CorrectionDeg - LastCorrectionDeg;
			const double	Size = LegsMoved.Size();
			if (Size > 1.0e-9)
			{
				WithLegs = FVector2D::DotProduct(AttitudeDeg - LastAttitudeDeg, LegsMoved) / Size;
			}
		}
		LastAttitudeDeg = AttitudeDeg;
		LastCorrectionDeg = CorrectionDeg;
		bHasLast = true;

		if (!bCounts || ErrorDeg <= DivergenceErrorDeg)
		{
			Seconds = 0.0;
			Worst = 0.0;
			WithLegsDeg = 0.0;
			return false;
		}
		if (ErrorDeg <= Worst - DivergenceReliefDeg)
		{
			// The error is coming back: start the count again from here, in
			// case it turns round. The evidence stays: an error can come back
			// because the wanted attitude gave way to it, not the base.
			Seconds = 0.0;
			Worst = 0.0;
		}
		Worst = FMath::Max(Worst, ErrorDeg);
		Seconds += Dt;
		WithLegsDeg += WithLegs;
		if (LoopTimerHasRun(Seconds, Tuning.FaultSeconds) && WithLegsDeg >= DivergenceWithLegsDeg)
		{
			bDiverged = true;
		}
		return bDiverged;
	}
} // namespace RammsLevel
