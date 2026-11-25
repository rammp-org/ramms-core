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

		/** How much of a leg's move from offset From to offset To (cm) counts
		 *  against its rate when the correction plane is slewed: all of it for
		 *  a leg the robot stands on; for a lifted leg, only what takes it up
		 *  (positive retracts) between the two ends held inside its travel. */
		double LoopCountedMove(const FLegReach& Leg, double From, double To)
		{
			if (Leg.bSupport)
			{
				return FMath::Abs(To - From);
			}
			const double Low = FMath::Min(0.0, Leg.MinZ - Leg.BaseZ);
			const double High = FMath::Max(0.0, Leg.MaxZ - Leg.BaseZ);
			return FMath::Max(0.0, FMath::Clamp(To, Low, High) - FMath::Clamp(From, Low, High));
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

	FVector2D FGroundGate::Tick(const FVector2D& RawGroundDeg, double SpeedCmS, double TurnDegPerCm, double ExplainedDeg, double Dt, double YawRateDegPerSec)
	{
		if (!bSeeded)
		{
			Seed(RawGroundDeg);
		}
		if (Dt <= 0.0)
		{
			return Value;
		}
		// How hard the base is being braked or driven off: from one tick's
		// speed to the next. (None on the first: there is no tick before it.)
		const double Speed = FMath::Abs(SpeedCmS);
		const double Accel = bHasSpeed ? FMath::Abs(Speed - LastSpeedCmS) / Dt : 0.0;
		LastSpeedCmS = Speed;
		bHasSpeed = true;
		const double Start = FMath::Max(0.0, Tuning.BodyAccelStartCmS2);
		const double Full = FMath::Max(Start, Tuning.BodyAccelFullCmS2);
		const double Believed = Accel <= Start ? 1.0 : (Accel >= Full ? 0.0 : (Full - Accel) / (Full - Start));
		// Down at once, up again only as the body settles.
		const double TrustBefore = Trust;
		Trust = Believed < Trust ? Believed : FMath::Min(Believed, Trust + (Tuning.BodySettleSeconds > 0.0 ? Dt / Tuning.BodySettleSeconds : 1.0));
		// A speed is the distance between two readings over the time between
		// them: it says how the base moved BETWEEN them, and a braking that
		// began late in that time hardly shows in it. So the tick before the
		// one that shows the braking has already read some of the body's
		// pitch, and followed it as ground: as much of that last step as the
		// trust now lost says was not the ground's is taken back.
		if (Trust < TrustBefore && TrustBefore > 0.0)
		{
			Value -= LastStep * ((TrustBefore - Trust) / TrustBefore);
		}

		if (Tuning.BreakDeg <= 0.0)
		{
			Allowance = 0.0;
			Value = RawGroundDeg;
			LastStep = FVector2D::ZeroVector;
			return Value;
		}
		// (While the base is being braked or driven off hard its attitude says
		// nothing of the ground, slowly or otherwise: the settling allowance
		// is given up with the travel's. What the baselines did is known.)
		// (Turning on a slope: the slope as it is believed, swung by the angle
		// turned. A turn on level ground swings nothing.)
		const double YawRadPerSec = FMath::IsFinite(YawRateDegPerSec) ? FMath::DegreesToRadians(FMath::Abs(YawRateDegPerSec)) : 0.0;
		Allowance = Trust * (FMath::Max(0.0, Tuning.SettleDegPerSec) + FMath::Max(0.0, TurnDegPerCm) * Speed + Value.Size() * YawRadPerSec) + FMath::Max(0.0, ExplainedDeg) / Dt;
		FVector2D	 Step = RawGroundDeg - Value;
		const double Size = Step.Size();
		const double Most = Allowance * Dt;
		if (Size > Most)
		{
			Step *= Most / Size;
		}
		Value += Step;
		// (What the baselines turned the stance by is known, whatever the
		// base is doing: that part of a step is never taken back.)
		const double Explained = FMath::Max(0.0, ExplainedDeg);
		const double Taken = Step.Size();
		LastStep = Taken > Explained ? Step * ((Taken - Explained) / Taken) : FVector2D::ZeroVector;
		return Value;
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
		return WantedTiltGoalWithLimit(GroundDeg, Legs, Points, Tuning).GoalDeg;
	}

	FTiltGoal WantedTiltGoalWithLimit(const FVector2D& GroundDeg, TConstArrayView<FLegReach> Legs, TConstArrayView<FClearancePoint> Points,
		const FTuning& Tuning)
	{
		const FVector2D Whole = GroundDeg - Tuning.LevelReferenceDeg;
		const FReach	Reach = AllocateReach(Legs, Points, Whole, Tuning, Tuning.SoftZoneCm);
		FTiltGoal		Out;
		Out.GoalDeg = WantedTiltGoalForShare(GroundDeg, Reach.Share, Tuning);
		// What the legs leave of it, and why: their own limits first; with
		// those out of the way, an axis held at the most it may correct.
		const FVector2D Clamped = ClampAxes(Whole, Tuning.MaxCorrectionDeg);
		Out.ShortDeg = (Whole - Clamped * FMath::Clamp(Reach.Share, 0.0, 1.0)).Size();
		if (Reach.Share < 1.0)
		{
			Out.Limit = Reach.Limit;
		}
		else if (!(Whole - Clamped).IsNearlyZero(1.0e-9))
		{
			Out.Limit = EReachLimit::AxisLimit;
		}
		return Out;
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

	FVector2D FTiltTarget::Brake(double Dt, double MaxAccelDegPerSec2)
	{
		if (Dt <= 0.0)
		{
			return Wanted;
		}
		const double Speed = Rate.Size();
		const double Slower = MaxAccelDegPerSec2 > 0.0 ? Speed - MaxAccelDegPerSec2 * Dt : 0.0;
		if (Slower <= 0.0 || Speed <= 0.0)
		{
			Rate = FVector2D::ZeroVector;
			return Wanted;
		}
		Rate *= Slower / Speed;
		Wanted += Rate * Dt;
		return Wanted;
	}

	FVector2D FLevelLoop::Tick(const FVector2D& AttitudeDeg, const FVector2D& WantedDeg, const FVector2D& GroundDeg, double Dt, bool bIntegrate,
		bool bSaturated, const FTuning& Tuning, double TrimDeadbandDeg)
	{
		if (Dt <= 0.0)
		{
			return Correction;
		}
		const FVector2D Error(Deadband(AttitudeDeg.X - WantedDeg.X, Tuning.DeadbandDeg), Deadband(AttitudeDeg.Y - WantedDeg.Y, Tuning.DeadbandDeg));
		if (bIntegrate)
		{
			// The integral's own band, where one is given: never wider than
			// the loop's.
			const double	TrimBand = TrimDeadbandDeg >= 0.0 ? FMath::Min(TrimDeadbandDeg, FMath::Max(0.0, Tuning.DeadbandDeg)) : Tuning.DeadbandDeg;
			const FVector2D Trimmed(Deadband(AttitudeDeg.X - WantedDeg.X, TrimBand), Deadband(AttitudeDeg.Y - WantedDeg.Y, TrimBand));
			const FVector2D Step = Trimmed * (Tuning.IntegralGain * Dt);
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
		double MaxRateDegPerSec, const FTuning& Tuning, double& OutRateUse, bool& bOutLimited, TConstArrayView<double> AboveCm)
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
		// so that what is reported says how much faster the loop asks than
		// the legs can give. Every cut scales the whole step: the plane stays
		// rigid.
		// A lifted leg counts only for what brings it UP, within its travel:
		// let down, or resting on its stop, it is no reason to hold the legs
		// that carry the robot back (the header says why).
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			if (Legs[i].MaxRateCmS > 0.0)
			{
				// (A lifted leg that stands above where the plane has it is
				// brought up only from where it stands.)
				const double Above = !Legs[i].bSupport && AboveCm.IsValidIndex(i) ? FMath::Max(0.0, AboveCm[i]) : 0.0;
				OutRateUse = FMath::Max(OutRateUse, LoopCountedMove(Legs[i], Before[i] + Above, After[i]) / (Dt * Legs[i].MaxRateCmS));
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

	double FSeatTrim::Tick(double ShareDeg, double AnswerDeg, bool bCover, double LowDeg, double HighDeg, double Dt, const FTuning& Tuning)
	{
		if (Dt <= 0.0)
		{
			return Trim;
		}
		// A base nose up of where it is wanted is answered nose down at the seat.
		const double MaxTrim = FMath::Max(0.0, Tuning.SeatTrimMaxDeg);
		const double Low = FMath::Max(-MaxTrim, FMath::Min(0.0, LowDeg));
		const double High = FMath::Min(MaxTrim, FMath::Max(0.0, HighDeg));
		const double CoverDeg = bCover ? -Deadband(AnswerDeg, SeatTrimDeadbandDeg) : 0.0;
		const double Target = FMath::Clamp(ShareDeg + CoverDeg, Low, High);
		const double MaxStep = FMath::Max(0.0, Tuning.SeatTrimRateDegPerSec) * Dt;
		bBehind = FMath::Abs(Target - Trim) > MaxStep + 1.0e-12;
		Trim += FMath::Clamp(Target - Trim, -MaxStep, MaxStep);
		return Trim;
	}

	double PlaneRateDegPerSec(TConstArrayView<FLegReach> Legs, const FVector2D& DirDeg, const FTuning& Tuning)
	{
		const double Size = DirDeg.Size();
		if (Size <= 1.0e-12)
		{
			return 0.0;
		}
		// Each leg's cm for a degree of plane along the direction.
		FLegValues PerDegree;
		RigidOffsets(Legs, DirDeg / Size, PerDegree);
		bool bAnyStands = false;
		for (const FLegReach& Leg : Legs)
		{
			bAnyStands = bAnyStands || Leg.bSupport;
		}
		// The busiest counting leg's seconds per degree at its whole rate.
		double SecondsPerDegree = 0.0;
		for (int32 Index = 0; Index < Legs.Num() && Index < PerDegree.Num(); ++Index)
		{
			if ((Legs[Index].bSupport || !bAnyStands) && Legs[Index].MaxRateCmS > 0.0)
			{
				SecondsPerDegree = FMath::Max(SecondsPerDegree, FMath::Abs(PerDegree[Index]) / Legs[Index].MaxRateCmS);
			}
		}
		double Rate = Tuning.LegRateFraction > 0.0 && SecondsPerDegree > 0.0 ? Tuning.LegRateFraction / SecondsPerDegree : 0.0;
		if (Tuning.MaxCorrectionRateDegPerSec > 0.0)
		{
			Rate = Rate > 0.0 ? FMath::Min(Rate, Tuning.MaxCorrectionRateDegPerSec) : Tuning.MaxCorrectionRateDegPerSec;
		}
		return Rate;
	}

	double FSpeedScale::Tick(const FSpeedCapInputs& In, double Dt, const FTuning& Tuning)
	{
		if (Dt <= 0.0)
		{
			return Out;
		}
		const double Floor = LoopSpeedFloor(Tuning);

		// The scale as the drive has had time to answer it, and from that the
		// speed the rider's stick is asking for: the speed there is, over the
		// share of it the drive is being let have.
		Driven += (Out - Driven) * LoopFilterShare(Dt, Cap.DriveLagSeconds);
		const double Speed = FMath::IsFinite(In.SpeedCmS) ? FMath::Abs(In.SpeedCmS) : 0.0;
		const bool	 bAskKnown = FMath::IsFinite(In.AskedSpeedCmS) && In.AskedSpeedCmS >= 0.0;
		if (bAskKnown)
		{
			// The drive says what its stick asks.
			FreeSpeed = In.AskedSpeedCmS >= Cap.MinSpeedCmS ? In.AskedSpeedCmS : 0.0;
			// A stick that has just asked for another speed: the drive will
			// overshoot it as it arrives, and that is not a robot running away.
			const double StickAccel = bHasFreeSpeed ? FMath::Abs(In.AskedSpeedCmS - LastFreeSpeedCmS) / Dt : 0.0;
			LastFreeSpeedCmS = In.AskedSpeedCmS;
			bHasFreeSpeed = true;
			SinceStickSeconds = StickAccel > FMath::Max(0.0, Cap.UnsteadyAskCmS2) ? 0.0 : SinceStickSeconds + Dt;
		}
		else if (Speed >= Cap.MinSpeedCmS)
		{
			const double Asked = Speed / FMath::Max(Driven, Floor);
			// (The first of it whole: a robot already under way when the cap
			// starts is not let run on while a filter finds its speed.)
			FreeSpeed = FreeSpeed > 0.0 ? FreeSpeed + (Asked - FreeSpeed) * LoopFilterShare(Dt, Cap.FreeSpeedFilterSeconds) : Asked;
		}
		else
		{
			FreeSpeed = 0.0;
		}

		// How steadily the drive is being asked -- by its stick, and by this
		// scale as the last tick left it -- and how steadily the robot is
		// moving (a drive still bringing it up to what was asked a while ago
		// is pitching it too). Down at once, up again only once both have
		// been steady for a while.
		const double AskedNow = Out * FreeSpeed;
		const double AskAccel = bHasAsked ? FMath::Abs(AskedNow - LastAskedCmS) / Dt : 0.0;
		const double SpeedAccel = bHasAsked ? FMath::Abs(Speed - LastSpeedCmS) / Dt : 0.0;
		LastAskedCmS = AskedNow;
		LastSpeedCmS = Speed;
		bHasAsked = true;
		const double Believed = FMath::Min(LoopRamp(AskAccel, FMath::Max(0.0, Cap.SteadyAskCmS2), 1.0, Cap.UnsteadyAskCmS2, 0.0),
			LoopRamp(SpeedAccel, FMath::Max(0.0, Cap.SteadySpeedCmS2), 1.0, Cap.UnsteadySpeedCmS2, 0.0));
		Steady = Believed < Steady ? Believed : FMath::Min(Believed, Steady + (Cap.SteadySettleSeconds > 0.0 ? Dt / Cap.SteadySettleSeconds : 1.0));

		// How fast the ground is turning under the robot, and how fast it may.
		// Less what the chassis' own turning about the vertical swings the
		// slope by in its frame: no break in the ground, and nothing that
		// going slower helps. Taken up only as far as the drive is being
		// asked steadily (the pitch of a drive changing speed is in the
		// estimate too); what was last read stands meanwhile.
		FVector2D	 Rate = In.bGroundRateKnown && FMath::IsFinite(In.GroundRateDegPerSec.X) && FMath::IsFinite(In.GroundRateDegPerSec.Y) ? In.GroundRateDegPerSec : FVector2D::ZeroVector;
		const double RateSize = Rate.Size();
		const double YawRadPerSec = FMath::IsFinite(In.YawRateDegPerSec) ? FMath::DegreesToRadians(FMath::Abs(In.YawRateDegPerSec)) : 0.0;
		const double Swung = (FMath::IsFinite(In.SlopeDeg) ? FMath::Max(0.0, In.SlopeDeg) : 0.0) * YawRadPerSec;
		Rate = RateSize > Swung ? Rate * ((RateSize - Swung) / RateSize) : FVector2D::ZeroVector;
		Turn += (Rate - Turn) * (LoopFilterShare(Dt, Cap.TurnFilterSeconds) * (In.bGroundRateKnown ? Steady : 1.0));
		Follow = FMath::Max(0.0, Cap.FollowMaxDegPerSec);
		if (In.LegsDegPerSec > 0.0)
		{
			Follow = FMath::Min(Follow, FMath::Max(0.0, Cap.FollowShare) * In.LegsDegPerSec);
		}

		// The error, as the backstop; past reach, the floor outright.
		double Target = In.bPastReach ? Floor : LoopRamp(In.ErrorDeg, Tuning.SpeedCapErrorDeg, 1.0, Tuning.SpeedCapErrorFullDeg, Floor);

		// The ground turning now faster than the legs can follow: the speed is
		// what turns it, so the scale that would turn it at their rate.
		// (No threshold: at just their rate the scale holds where it is, and
		// under it the term is over 1 and binds nothing.)
		const double Turning = Turn.Size();
		if (Follow > 0.0 && Turning > 1.0e-9)
		{
			Target = FMath::Min(Target, Driven * Follow / Turning);
		}

		// The first sign of a break: the same turning where it shows first,
		// kept as a turn per cm travelled until the break is past the legs.
		{
			FVector2D	 Seen = In.bSeenRateKnown && FMath::IsFinite(In.SeenRateDegPerSec.X) && FMath::IsFinite(In.SeenRateDegPerSec.Y) ? In.SeenRateDegPerSec : FVector2D::ZeroVector;
			const double SeenSize = FMath::Max(0.0, Seen.Size() - Swung);
			// (Read as it comes for a moment after the first sign; otherwise
			// only as far as the drive is being asked steadily.)
			const bool bOpen = HeldTurnPerCm > 0.0 && SinceHeldSeconds < Cap.EarlyOpenSeconds;
			Early += (SeenSize - Early) * (LoopFilterShare(Dt, Cap.EarlyFilterSeconds) * (In.bSeenRateKnown && !bOpen ? Steady : 1.0));
			SinceHeldCm += Speed * Dt;
			SinceHeldSeconds += Dt;
			if (Cap.EarlyHeldSpans > 0.0 && Follow > 0.0 && Speed >= Cap.MinSpeedCmS && Early > FMath::Max(0.0, Cap.EarlyStartShare) * Follow)
			{
				if (HeldTurnPerCm <= 0.0 && Steady >= 1.0 - 1.0e-9)
				{
					// The first sign.
					HeldTurnPerCm = Early / Speed;
					SinceHeldCm = 0.0;
					SinceHeldSeconds = 0.0;
				}
				else if (bOpen)
				{
					HeldTurnPerCm = FMath::Max(HeldTurnPerCm, Early / Speed);
				}
			}
			if (In.MaxTurnDegPerCm > 0.0)
			{
				HeldTurnPerCm = FMath::Min(HeldTurnPerCm, In.MaxTurnDegPerCm);
			}
			if (Cap.EarlyHeldSpans <= 0.0 || In.SpanCm <= 0.0 || SinceHeldCm > Cap.EarlyHeldSpans * In.SpanCm)
			{
				HeldTurnPerCm = 0.0;
			}
			if (HeldTurnPerCm > 0.0 && Follow > 0.0 && FreeSpeed > 0.0)
			{
				Target = FMath::Min(Target, FMath::Max(FMath::Clamp(Cap.EarlyMinScale, 0.0, 1.0), Follow / (HeldTurnPerCm * FreeSpeed)));
			}
		}
		// A speed to keep under (a three-point robot still carrying roll).
		if (In.HoldSpeedCmS > 0.0 && FreeSpeed > In.HoldSpeedCmS)
		{
			Target = FMath::Min(Target, In.HoldSpeedCmS / FreeSpeed);
		}

		// The slope the robot is levelled on will end, and the ground then
		// comes back by all of it inside one span's travel: no faster than
		// that can be followed.
		// (The slope as it was when it began to end, until the robot is well
		// past where it did: half way through a break the slope left is half,
		// and so is the way left to lose it in.)
		const double Slope = FMath::IsFinite(In.SlopeDeg) ? FMath::Max(0.0, In.SlopeDeg) : 0.0;
		SinceSteepestCm += Speed * Dt;
		if (Slope >= Steepest || Speed < Cap.MinSpeedCmS || In.SpanCm <= 0.0 || SinceSteepestCm > Cap.SlopeHeldSpans * In.SpanCm)
		{
			Steepest = Slope;
			SinceSteepestCm = 0.0;
		}
		ExitSpeed = 0.0;
		if (Cap.bSlopeEnd && Follow > 0.0 && Steepest > 0.0 && In.SpanCm > 0.0)
		{
			ExitSpeed = Follow * In.SpanCm / Steepest;
			if (FreeSpeed > ExitSpeed)
			{
				Target = FMath::Min(Target, ExitSpeed / FreeSpeed);
			}
		}
		Target = FMath::Clamp(Target, Floor, 1.0);
		// It slows, it never stops: the drive is a speed loop, and asked for
		// much less than the speed there is it brakes through what was asked.
		// Never asked for more than a little under the speed the robot has,
		// it is led down, and arrives.
		if (FreeSpeed > 0.0 && Cap.LeadCmS > 0.0)
		{
			Target = FMath::Max(Target, FMath::Min(1.0, (Speed * (1.0 - FMath::Clamp(Cap.LeadShare, 0.0, 1.0)) - Cap.LeadCmS) / FreeSpeed));
		}

		if (Target < Scale)
		{
			// Down no faster than brakes the robot as hard as the ground's
			// estimate still believes the base's attitude through; and the
			// last of the way gently.
			double Fall = FMath::Max(0.0, Cap.FallPerSec);
			if (Cap.BrakeCmS2 > 0.0 && FreeSpeed > 0.0)
			{
				Fall = FMath::Min(Fall, Cap.BrakeCmS2 / FreeSpeed);
			}
			if (Cap.FallSeconds > 0.0)
			{
				Fall = FMath::Min(Fall, (Scale - Target) / Cap.FallSeconds);
			}
			Scale = FMath::Max(Target, Scale - Fall * Dt);
		}
		else
		{
			Scale = FMath::Min(Target, Scale + FMath::Max(0.0, Tuning.SpeedScaleRisePerSec) * Dt);
		}
		// A speed to keep under that the robot has not yet reached is kept
		// under at once: that brakes nothing, it only does not let the drive
		// run up past it while the scale comes down at its own pace.
		if (In.HoldSpeedCmS > 0.0 && FreeSpeed > In.HoldSpeedCmS && Speed <= 1.1 * In.HoldSpeedCmS)
		{
			Scale = FMath::Min(Scale, FMath::Max(Floor, In.HoldSpeedCmS / FreeSpeed));
		}
		// A robot running away past what its stick asks (gravity, down the far
		// side of a crest, with the drive's loop still holding the climb): the
		// further past, the less is asked, at once and for as long as it is
		// past and no longer. That is the drive's own loop made stiffer while
		// it is being outrun; it fades as the speed comes back, so it brakes
		// the robot towards what was asked and never through it.
		Out = Scale;
		// (Only where the drive says what is asked: a speed taken for the
		// speed asked cannot be over it.)
		// (And only past a band, once the stick's own ask has stood a while: a
		// start's overshoot and a speed that wanders are not a run-away.)
		const double Band = FMath::Max(FMath::Max(0.0, Cap.OverspeedBandCmS), FMath::Max(0.0, Cap.OverspeedBandShare) * FreeSpeed);
		if (bAskKnown && FreeSpeed > 0.0 && Cap.OverspeedGain > 0.0 && Speed > FreeSpeed + Band && LoopTimerHasRun(SinceStickSeconds, Cap.OverspeedSettleSeconds))
		{
			Out = FMath::Max(Floor, Scale - Cap.OverspeedGain * (Speed - FreeSpeed - Band) / FreeSpeed);
		}
		return Out;
	}

	double FSpeedScale::Relax(double Dt, const FTuning& Tuning)
	{
		if (Dt <= 0.0)
		{
			return Out;
		}
		// Nothing is being levelled: what the ground was doing is forgotten.
		Driven += (Out - Driven) * LoopFilterShare(Dt, Cap.DriveLagSeconds);
		Turn = FVector2D::ZeroVector;
		ExitSpeed = 0.0;
		Steepest = 0.0;
		SinceSteepestCm = 0.0;
		bHasAsked = false;
		Steady = 1.0;
		Early = 0.0;
		HeldTurnPerCm = 0.0;
		SinceHeldCm = 0.0;
		SinceHeldSeconds = 0.0;
		bHasFreeSpeed = false;
		SinceStickSeconds = 1.0e6;
		Scale = FMath::Min(1.0, Scale + FMath::Max(0.0, Tuning.SpeedScaleRisePerSec) * Dt);
		Out = Scale;
		return Out;
	}

	FVector2D FCarryLimit::Tick(const FCarryInputs& In, double Dt, const FTuning& LawTuning)
	{
		const FVector2D Full(FMath::Max(0.0, In.FullDeg.X), FMath::Max(0.0, In.FullDeg.Y));
		if (Dt <= 0.0 && bSeeded)
		{
			return Limit.Get();
		}
		// The speed the rules go by: up at once, down only as the robot slows.
		const double Now = FMath::IsFinite(In.SpeedCmS) ? FMath::Abs(In.SpeedCmS) : 0.0;
		Speed = bHasSpeed ? Speed + (Now - Speed) * LoopFilterShare(Dt, Tuning.SpeedFilterSeconds) : Now;
		bHasSpeed = true;
		const double V = FMath::Max(Now, Speed);

		FVector2D Target = Full;
		// No more than the legs can take back out while a break passes under
		// them.
		if (Tuning.bCrest && V >= QuietSpeedCmS && In.SpanCm > 0.0)
		{
			// How long the break is under the legs once they have begun to
			// answer it: the robot slowed for it as the speed cap will.
			const double Lag = FMath::Max(0.0, Tuning.CrestLagSeconds);
			const double Brake = FMath::Max(0.0, Tuning.CrestBrakeCmS2);
			const double Floor = FMath::Max(QuietSpeedCmS, FMath::Min(V, LoopSpeedFloor(LawTuning) * V));
			const double Left = FMath::Max(0.0, In.SpanCm - V * Lag);
			double		 Seconds = 0.0;
			if (Brake <= 0.0)
			{
				Seconds = Left / V;
			}
			else if ((V * V - Floor * Floor) / (2.0 * Brake) >= Left)
			{
				Seconds = (V - FMath::Sqrt(FMath::Max(0.0, V * V - 2.0 * Brake * Left))) / Brake;
			}
			else
			{
				Seconds = (V - Floor) / Brake + (Left - (V * V - Floor * Floor) / (2.0 * Brake)) / Floor;
			}
			Seconds *= FMath::Clamp(Tuning.CrestShare, 0.0, 1.0);
			if (In.LegsDegPerSec.X > 0.0)
			{
				Target.X = FMath::Min(Target.X, In.LegsDegPerSec.X * Seconds);
			}
			if (In.LegsDegPerSec.Y > 0.0)
			{
				Target.Y = FMath::Min(Target.Y, In.LegsDegPerSec.Y * Seconds);
			}
		}
		// A roll correction that stands the robot on three points: at a crawl
		// only, and only for a roll worth correcting.
		bThreePoint = In.bThreePoint && Tuning.bThreePoint;
		if (bThreePoint)
		{
			double Needed = FMath::IsFinite(In.NeededDeg.Y) ? FMath::Abs(In.NeededDeg.Y) : 0.0;
			// The ground's roll is read true only while no roll is carried: with
			// some carried, the rigid pairs of a three-point robot still bear
			// on both wheels for the first of it and hold the body back, the
			// legs' plane reads as more roll than the body has made, and the
			// rest is taken for ground (a roll "needed" that grows with the
			// correction, and never comes back under the let-go). So the
			// ground's roll as it last read with none carried is kept, and for
			// as long as the robot has not moved off that spot the roll needed
			// is counted from it. (Engaged or not: a roll let go by the one
			// reckoning and taken up again by the other would do so every
			// other tick.)
			const double Ground = In.GroundRollDeg;
			const bool	 bGround = In.bGroundRollKnown && FMath::IsFinite(Ground);
			if (bGround && FMath::IsFinite(In.CarriedDeg.Y) && FMath::Abs(In.CarriedDeg.Y) <= Tuning.RollClearDeg)
			{
				FreeGroundRoll = Ground;
				bHasFreeGround = true;
				MovedCm = 0.0;
			}
			else
			{
				MovedCm += (Now >= QuietSpeedCmS ? Now : 0.0) * FMath::Max(0.0, Dt);
			}
			if (bHasFreeGround && MovedCm <= Tuning.RollSpotCm && bGround && FMath::IsFinite(In.NeededDeg.Y))
			{
				// (The reference's roll is the ground's as read now less what
				// reads as needed now.)
				Needed = FMath::Abs(FreeGroundRoll - (Ground - In.NeededDeg.Y));
			}
			bRollEngaged = bRollEngaged ? Needed >= FMath::Min(Tuning.RollLetGoDeg, Tuning.RollEngageDeg) : Needed >= Tuning.RollEngageDeg;
			// All of the roll or none of it, by the speed the robot is going
			// at: none from NoneCmS, all again from FullCmS down, and between
			// the two whichever it was. Never a part of it for good: a robot
			// stopped with part of its roll out swings further than one with
			// all of it carried, and turns again as the rest comes back.
			// (The speed it is going at is the speed its stick asks, where the
			// drive can say: held to a walk while its roll is eased out, it
			// is still a robot going fast. The speed there is counts only
			// past a start's overshoot of what was asked.)
			const bool	 bAsked = FMath::IsFinite(In.AskedSpeedCmS) && In.AskedSpeedCmS >= 0.0;
			const double Going = bAsked ? FMath::Max(In.AskedSpeedCmS, V / (1.0 + FMath::Max(0.0, Tuning.ThreePointOvershoot))) : V;
			if (Going >= FMath::Max(Tuning.ThreePointNoneCmS, Tuning.ThreePointFullCmS) && Going > Tuning.ThreePointFullCmS)
			{
				bRollOut = true;
			}
			else if (Going <= Tuning.ThreePointFullCmS)
			{
				bRollOut = false;
			}
			Target.Y = bRollEngaged && !bRollOut ? Target.Y : 0.0;
		}
		else
		{
			bRollEngaged = true;
			bRollOut = false;
		}
		if (!bSeeded)
		{
			// (From what is carried already, where that is more: walked down.)
			const FVector2D Carried(FMath::IsFinite(In.CarriedDeg.X) ? FMath::Min(Full.X, FMath::Abs(In.CarriedDeg.X)) : 0.0, FMath::IsFinite(In.CarriedDeg.Y) ? FMath::Min(Full.Y, FMath::Abs(In.CarriedDeg.Y)) : 0.0);
			Limit.Reset(FVector2D(FMath::Max(Target.X, Carried.X), FMath::Max(Target.Y, Carried.Y)));
			bSeeded = true;
			bWasRollEngaged = bRollEngaged;
			return Limit.Get();
		}
		// Not back up while the base is being braked: it is on all its wheels,
		// and stays so until it moves steadily again.
		if (!In.bMayRise)
		{
			Target.X = FMath::Min(Target.X, FMath::Max(0.0, Limit.Get().X));
			Target.Y = FMath::Min(Target.Y, FMath::Max(0.0, Limit.Get().Y));
		}
		// Nor does the roll of a three-point robot come back while it is still
		// coming DOWN to what its stick asks, however gently: only once it is
		// at the speed asked. (All the way down: a robot being stopped from a
		// speed at which part of the roll was out gets none of it back until
		// it stands. Roll that came back as the speed fell through the band
		// came back in the middle of the braking it was taken out for.)
		// Coming down is not the same as being over the ask. A robot that has
		// been AT the speed asked and is then a little over it (the ground
		// tipping away under it, a drive that wanders) is not being slowed:
		// it stays "there" until its stick asks less than it did when the
		// robot was last there, or it is faster than a start's overshoot of
		// what is asked (the speed the rule itself goes by, above). Taken for
		// coming down, a robot a few cm/s over its ask on the ticks a cross
		// fall came on under it was given no roll limit at all for as long as
		// it was over, the fall left in the base meanwhile.
		if (bThreePoint && FMath::IsFinite(In.AskedSpeedCmS) && In.AskedSpeedCmS >= 0.0)
		{
			if (V <= In.AskedSpeedCmS + QuietSpeedCmS)
			{
				bAtAsked = true;
				AtAskedCmS = In.AskedSpeedCmS;
			}
			else if (In.AskedSpeedCmS < AtAskedCmS - QuietSpeedCmS || V > In.AskedSpeedCmS * (1.0 + FMath::Max(0.0, Tuning.ThreePointOvershoot)) + QuietSpeedCmS)
			{
				bAtAsked = false;
			}
			if (!bAtAsked)
			{
				Target.Y = FMath::Min(Target.Y, FMath::Max(0.0, Limit.Get().Y));
			}
		}
		else
		{
			bAtAsked = false;
		}
		// A roll that has just come to be worth correcting (out of the dead
		// band) is corrected from here on with all the roll limit there is: at
		// once, not walked up from nothing. The limit only bounds the
		// correction, and where it is walked up under a cross fall that comes
		// on as the robot drives, it is the limit and not the legs that the
		// correction then follows, a second or more behind the ground. (The
		// rider is still turned within the tilt policy: where the base is
		// wanted walks there from where the dead band left it.) Not where
		// the limit is held just above: a robot being slowed, or braked.
		if (bThreePoint && bRollEngaged && !bWasRollEngaged && Target.Y > Limit.Get().Y)
		{
			Limit.Reset(FVector2D(Limit.Get().X, Target.Y));
		}
		bWasRollEngaged = bRollEngaged;
		Limit.Tick(Target, Dt, LawTuning.TiltRateDegPerSec, LawTuning.TiltAccelDegPerSec2);
		return FVector2D(FMath::Clamp(Limit.Get().X, 0.0, Full.X), FMath::Clamp(Limit.Get().Y, 0.0, Full.Y));
	}

	double FCarryLimit::HoldSpeedCmS(double RollCorrectionDeg, double FullRollDeg) const
	{
		const double Roll = FMath::Abs(RollCorrectionDeg);
		if (!bThreePoint || !bRollOut || !Tuning.bHoldSpeed || !(FullRollDeg > 0.0) || !(Roll > Tuning.RollClearDeg) || !(Tuning.ThreePointFullCmS > 0.0))
		{
			return 0.0;
		}
		return Tuning.ThreePointFullCmS;
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
	double FMotionSense::Tick(const FVector& PositionCm, const FVector2D& Heading, double SampleSeconds)
	{
		Moved = 0.0;
		bBackwards = false;
		if (PositionCm.ContainsNaN() || !FMath::IsFinite(SampleSeconds))
		{
			return 0.0;
		}
		const double Since = bHas ? SampleSeconds - LastSeconds : 0.0;
		if (bHas && Since == 0.0)
		{
			// The same pose again: it says nothing the last one did not.
			return 0.0;
		}
		if (!bHas || Since < 0.0)
		{
			// The first reading, or a clock that was reset: begin again here.
			bBackwards = bHas;
			SpeedCmS = 0.0;
			YawRateDegPerSec = 0.0;
		}
		else
		{
			Moved = FVector::Dist(PositionCm, LastPosition);
			SpeedCmS = Moved / Since;
			// How fast the heading turned: on a slope that swings the ground
			// from pitch into roll under a robot that has gone nowhere.
			const bool bHeadings = !Heading.IsNearlyZero() && !LastHeading.IsNearlyZero();
			YawRateDegPerSec = bHeadings ? FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(FVector2D::DotProduct(Heading, LastHeading), -1.0, 1.0))) / Since : 0.0;
		}
		LastPosition = PositionCm;
		LastHeading = Heading;
		LastSeconds = SampleSeconds;
		bHas = true;
		return FMath::Max(0.0, Since);
	}
} // namespace RammsLevel
