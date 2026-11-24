// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Level/RammsLevelAllocate.h"
#include "Level/RammsLevelTypes.h"

/**
 * The level assist's control pieces, each a small class or function with its
 * own state, fed made-up numbers by the tests and strung together by
 * FLaw::Step (RammsLevelLaw.h), which alone decides when each runs. Plain
 * C++. Units and signs as RammsLevelTypes.h: degrees, cm, s; X pitch (nose
 * up), Y roll (+Y up).
 *
 * The variables, as the law names them:
 *   A  the measured attitude, low-passed (FAttitudeFilter)
 *   S  the ground as far as it can have turned (FGroundGate): A with the
 *      joints' correction put back, less what only the body can have done
 *   G  the terrain estimate: the attitude the base would have standing on
 *      its baseline stance (FTerrainEstimate), from S
 *   W  the wanted attitude: every deliberate change of the rider's attitude
 *      goes through it, on the tilt policy's rate and acceleration
 *      (WantedTiltGoal, FTiltTarget)
 *   C  the correction plane the legs apply (FLevelLoop, SlewCorrection)
 *   e  the error A - W
 *
 * Every Tick with Dt <= 0 changes nothing and returns what it last returned
 * (a filter not yet seeded still takes its reading).
 */
namespace RammsLevel
{
	/** Error less the band either side of zero; 0 inside it. (RammsCurb::
	 *  Deadband, in double.) */
	inline double Deadband(double Error, double Band)
	{
		Band = FMath::Max(0.0, Band);
		return Error > Band ? Error - Band : (Error < -Band ? Error + Band : 0.0);
	}

	/**
	 * A first-order low-pass of an attitude, as the curb's: the first reading
	 * is taken whole; after that the value moves towards each reading by the
	 * share clamp(Dt / Seconds, 0, 1) of the gap (all of it for Seconds <= 0).
	 */
	class FAttitudeFilter
	{
	public:
		/** Forget: the next reading is taken whole. */
		void Reset() { *this = FAttitudeFilter(); }
		/** Take Deg now, whole. */
		void Seed(const FVector2D& Deg)
		{
			Value = Deg;
			bSeeded = true;
		}
		/** One reading (deg); returns the filtered value. */
		FVector2D Tick(const FVector2D& MeasuredDeg, double Dt, double Seconds);

		FVector2D Get() const { return Value; }
		bool	  IsSeeded() const { return bSeeded; }

	private:
		FVector2D Value = FVector2D::ZeroVector;
		bool	  bSeeded = false;
	};

	/**
	 * The terrain estimate G = low-pass(A + c_meas, TerrainFilterSeconds): A
	 * the filtered attitude, c_meas the correction the joints of the legs it
	 * stands on have achieved (MeasuredCorrection). The base reads A with
	 * c_meas applied, so without it it would read A + c_meas -- whatever the
	 * correction is, and however far the joints are behind their commands:
	 * the same ground gives the same G.
	 * The filter is FAttitudeFilter's.
	 */
	class FTerrainEstimate
	{
	public:
		void Reset() { Filter.Reset(); }
		void Seed(const FVector2D& GroundDeg) { Filter.Seed(GroundDeg); }
		/** One tick; returns G (deg). */
		FVector2D Tick(const FVector2D& AttitudeDeg, const FVector2D& MeasuredCorrectionDeg, double Dt, double Seconds);

		FVector2D Get() const { return Filter.Get(); }
		bool	  IsSeeded() const { return Filter.IsSeeded(); }

	private:
		FAttitudeFilter Filter;
	};

	/**
	 * The ground as far as it can have turned.
	 *
	 * The attitude, with the correction the joints have made put back (A +
	 * c_meas, the terrain estimate's reading), is the ground's -- while the
	 * robot rests quietly on it. A base being braked, driven off, knocked, or
	 * rocking onto a leg just set down reads something else for a few tenths
	 * of a second, and legs sent after that reading arrive as the base comes
	 * back by itself: the knock is answered by a swing the other way. The
	 * ground cannot do that. Its attitude changes only as the robot moves
	 * over it, by no more than a grade break turns it over the standing legs'
	 * span; so the reading is followed only that fast:
	 *
	 *   allowance (deg/s) = Trust * (Tuning.SettleDegPerSec
	 *                                + TurnDegPerCm * |SpeedCmS|)
	 *                     + ExplainedDeg / Dt
	 *   Value moves towards RawGroundDeg by at most allowance * Dt (Euclidean
	 *   over pitch and roll)
	 *
	 * TurnDegPerCm is what the ground may turn per cm travelled (the law
	 * gives Tuning.BreakDeg / StandingSpanCm). ExplainedDeg is how far the
	 * legs' own baselines can have turned the stance this tick
	 * (BaselineTurnDeg): a stance being ramped moves the reading with the ground
	 * standing still, and by exactly that. Trust (0..1) is how far the
	 * attitude is believed to be the ground's at all: with the base's speed
	 * changing by a (cm/s^2, from one tick's SpeedCmS to the next) it is at
	 * most 1 for |a| <= BodyAccelStartCmS2, 0 for |a| >= BodyAccelFullCmS2
	 * and on a straight line between, taken at once when that is less than
	 * it was; and it rises again by no more than Dt / BodySettleSeconds a
	 * tick. A base being braked that hard pitches by its own inertia, and
	 * goes on rocking for a moment after.
	 *
	 * What the gate leaves out is not lost: a reading that stays is caught up
	 * at the allowance. With Tuning.BreakDeg <= 0 there is no gate: Value is
	 * RawGroundDeg. The first reading (and Seed) is taken whole.
	 */
	class FGroundGate
	{
	public:
		/** Its own tuning; a test or the dry run may change it. */
		FGroundTuning Tuning;

		/** Forget (the tuning stays): the next reading is taken whole. */
		void Reset()
		{
			const FGroundTuning Kept = Tuning;
			*this = FGroundGate();
			Tuning = Kept;
		}
		/** Take GroundDeg now, whole; the travel is believed again from here. */
		void Seed(const FVector2D& GroundDeg)
		{
			Value = GroundDeg;
			bSeeded = true;
			Trust = 1.0;
			bHasSpeed = false;
		}
		/** One tick; returns the ground as far as it can have turned (deg). */
		FVector2D Tick(const FVector2D& RawGroundDeg, double SpeedCmS, double TurnDegPerCm, double ExplainedDeg, double Dt);

		FVector2D Get() const { return Value; }
		bool	  IsSeeded() const { return bSeeded; }
		/** How far the travel is believed (0..1), and the last tick's
		 *  allowance (deg/s). */
		double GetTrust() const { return Trust; }
		double GetAllowanceDegPerSec() const { return Allowance; }

	private:
		FVector2D Value = FVector2D::ZeroVector;
		bool	  bSeeded = false;
		double	  Trust = 1.0;
		double	  LastSpeedCmS = 0.0;
		bool	  bHasSpeed = false;
		double	  Allowance = 0.0;
	};

	/**
	 * Where the wanted attitude should go while levelling (deg): level where
	 * the legs can hold it, and beyond that the part of the ground's tilt they
	 * cannot take out, along the ground's own direction, never more than
	 * MaxTiltDeg.
	 *
	 *   T    = GroundDeg - Tuning.LevelReferenceDeg     (the correction that
	 *          would hold the reference on this ground)
	 *   s0   = AllocateFraction(Legs, Points, T, Tuning, Tuning.SoftZoneCm)
	 *   Got  = s0 * ClampAxes(T, Tuning.MaxCorrectionDeg)
	 *   goal = LevelReferenceDeg + ClampTotal(T - Got, Tuning.MaxTiltDeg)
	 *
	 * With no per-axis clamp binding this is the plan's (1 - s0) T. Taking
	 * the per-axis clamp into Got is a choice made here: a robot whose roll
	 * correction is capped then plans to keep the roll it cannot remove,
	 * rather than wanting a level it cannot reach.
	 *
	 * It depends only on G and the legs' reach, never on W, C or A, so it
	 * cannot chatter against the loop. The limits are pulled in by the soft
	 * zone so that, resting on the goal, the correction sits short of the
	 * margin and Allocate's share stays 1. Legs must have been through
	 * SupportMask.
	 */
	FVector2D WantedTiltGoal(const FVector2D& GroundDeg, TConstArrayView<FLegReach> Legs, TConstArrayView<FClearancePoint> Points,
		const FTuning& Tuning);

	/** WantedTiltGoal, with how much the legs must leave and what makes them. */
	struct FTiltGoal
	{
		/** WantedTiltGoal's goal (deg). */
		FVector2D GoalDeg = FVector2D::ZeroVector;
		/** How much of the correction that would hold the reference the legs
		 *  cannot give (deg, Euclidean): |T - Got|, before MaxTiltDeg caps the
		 *  goal. 0 inside reach. */
		double ShortDeg = 0.0;
		/** What stops them: the allocation's limit where its share is under 1
		 *  (AllocateReach, the limits pulled in by the soft zone), else
		 *  AxisLimit where the per-axis clamp cut T, else None. */
		EReachLimit Limit = EReachLimit::None;
	};

	FTiltGoal WantedTiltGoalWithLimit(const FVector2D& GroundDeg, TConstArrayView<FLegReach> Legs, TConstArrayView<FClearancePoint> Points,
		const FTuning& Tuning);

	/** WantedTiltGoal's rule with the reach share s0 given (0..1) rather than
	 *  found from the legs: the tests check it at any share with this. */
	FVector2D WantedTiltGoalForShare(const FVector2D& GroundDeg, double Share, const FTuning& Tuning);

	/**
	 * The wanted attitude W, moved to its goal within a rate (deg/s) and an
	 * acceleration (deg/s^2), both as Euclidean lengths over pitch and roll
	 * together, arriving at rest without passing a goal that stands still.
	 *
	 * One Tick, with D = Goal - W, d = |D|, a = MaxAccel, v = MaxRate:
	 *  - the speed wanted along D is the most that can still stop on the goal
	 *    in whole ticks: min(v, sqrt(a^2 Dt^2 + 2 a d) - a Dt); the velocity
	 *    wanted is that along D (zero for d = 0);
	 *  - the velocity moves towards the wanted one by at most a Dt (as a
	 *    vector), then is shortened to length v if longer;
	 *  - W += velocity Dt;
	 *  - once d <= 1e-9 and the velocity is at most a Dt long, W is the goal
	 *    exactly and the velocity zero.
	 * With MaxAccel <= 0 the velocity is the wanted one at once (a pure rate
	 * limit, min(v, d / Dt)); with MaxRate <= 0 too, W jumps to the goal.
	 * MaxRate <= 0 with MaxAccel > 0 is an acceleration limit alone. The d of
	 * the arrival test is the distance at the start of the tick.
	 *
	 * So the rate never exceeds MaxRate, the velocity never changes by more
	 * than MaxAccel Dt in a tick, and from rest to a goal that stays put W
	 * travels the straight line and stops on it. A goal that moves is chased
	 * under the same limits.
	 */
	class FTiltTarget
	{
	public:
		/** W := Deg, at rest. */
		void Reset(const FVector2D& Deg)
		{
			Wanted = Deg;
			Rate = FVector2D::ZeroVector;
		}
		/** One tick; returns W (deg). */
		FVector2D Tick(const FVector2D& GoalDeg, double Dt, double MaxRateDegPerSec, double MaxAccelDegPerSec2);
		/**
		 * One tick of coming to rest where it can, whatever the goal: the
		 * velocity is shortened by at most MaxAccel Dt (to zero at once for
		 * MaxAccel <= 0) and W moves on by it. At rest it stays at rest. For
		 * a walk that must wait (a stance being changed under the base): it
		 * stops as gently as it would at its goal, not dead.
		 */
		FVector2D Brake(double Dt, double MaxAccelDegPerSec2);

		FVector2D Get() const { return Wanted; }
		/** W's velocity (deg/s). */
		FVector2D GetRate() const { return Rate; }
		/** Within TiltArrivedDeg of GoalDeg and not moving. */
		bool HasArrived(const FVector2D& GoalDeg) const { return (GoalDeg - Wanted).Size() < TiltArrivedDeg && Rate.IsNearlyZero(1e-6); }
		/**
		 * As good as there: within TiltArrivedDeg of GoalDeg and slower than
		 * TiltArrivedRateDegPerSec. For a goal that is a live estimate (the
		 * ground's attitude, or a goal worked out from it): a real base's
		 * attitude moves in its last digits for ever, W follows the goal with
		 * it, and "not moving" never comes true.
		 */
		bool IsNear(const FVector2D& GoalDeg) const { return (GoalDeg - Wanted).Size() < TiltArrivedDeg && Rate.Size() < TiltArrivedRateDegPerSec; }

	private:
		FVector2D Wanted = FVector2D::ZeroVector;
		FVector2D Rate = FVector2D::ZeroVector;
	};

	/**
	 * The loop: the curb's structure, terrain feed-forward with a PI trim.
	 *
	 *   e     = (Deadband(A.X - W.X, DeadbandDeg), Deadband(A.Y - W.Y, ...))
	 *   e_I   = e, or with a TrimDeadbandDeg of 0 or more the same error less
	 *           that band (the lesser of it and DeadbandDeg) instead
	 *   I    += IntegralGain e_I Dt, per axis, where allowed (below), then
	 *           clamped to +-IntegralMaxDeg per axis
	 *   C_raw = FeedForwardGain (G - W) + I + ProportionalGain e
	 *
	 * The deadband keeps the legs from hunting after the last tenth of a
	 * degree while the ground moves under them. It also means that nothing
	 * acts inside it: an integral wound up by a transient (the lag of a walk,
	 * a knock) unwinds only until the error is back at the band's edge, and
	 * the base is left resting there. So where the law knows the base is
	 * standing still and about to rest it gives the integral a narrower band
	 * (TrimDeadbandDeg), and the rest is on the wanted attitude. The
	 * proportional term keeps the whole deadband either way.
	 *
	 * A nose-up error gives a positive pitch correction, which retracts the
	 * front legs. C_raw is NOT clamped here: the law clamps per axis, slews
	 * and allocates it.
	 *
	 * The integral moves on an axis only with bIntegrate and Dt > 0, and,
	 * against a limit, only when its step is of the opposite sign to the last
	 * C_raw on that axis: it never pushes further into a limit, and unwinds at
	 * once. An axis is against a limit with bSaturated (the last allocation
	 * could not give all of C: both axes), or when the last C_raw on that axis
	 * was at or past Tuning.MaxCorrectionDeg (that axis alone: the other goes
	 * on trimming). The clamp is applied on every tick with Dt > 0,
	 * integrating or not.
	 */
	class FLevelLoop
	{
	public:
		void Reset() { *this = FLevelLoop(); }

		/** One tick; returns C_raw (deg). TrimDeadbandDeg under 0: the
		 *  integral's band is the loop's. */
		FVector2D Tick(const FVector2D& AttitudeDeg, const FVector2D& WantedDeg, const FVector2D& GroundDeg, double Dt, bool bIntegrate,
			bool bSaturated, const FTuning& Tuning, double TrimDeadbandDeg = -1.0);

		/**
		 * A limit gave AchievedDeg where AskedDeg was asked (the allocation's
		 * share of the slewed plane, at the legs' reach). The
		 * integral gives up its own part of what was cut, per axis: it moves
		 * by (AchievedDeg - AskedDeg) but only TOWARDS zero and never past
		 * it. C_raw becomes the last Tick's other two terms plus the new
		 * integral.
		 *
		 * (Back-calculation, of the integral's own contribution only. What
		 * the feed-forward asks beyond a limit is not the integral's to
		 * cancel: an integral driven negative to make up for it would, when
		 * the limit lets go, hold the correction short by that much until it
		 * had integrated back. The plan words this "I is back-calculated to
		 * the achieved s * C"; taken literally that stores the feed-forward's
		 * excess and the slew's lag in the integral.)
		 */
		void BackCalculate(const FVector2D& AskedDeg, const FVector2D& AchievedDeg, const FTuning& Tuning);

		/**
		 * Start from a correction that is already applied, with no error:
		 * I := clamp(CorrectionDeg - FeedForwardGain (GroundDeg - WantedDeg),
		 * +-IntegralMaxDeg), and C_raw := the feed-forward term plus that I. A
		 * Tick straight after, with A = W, returns CorrectionDeg (to within
		 * the clamp): how an adopted plane and a resume go on without a step.
		 */
		void StartFrom(const FVector2D& CorrectionDeg, const FVector2D& WantedDeg, const FVector2D& GroundDeg, const FTuning& Tuning);

		FVector2D GetIntegral() const { return Integral; }
		/** The last C_raw (deg). */
		FVector2D GetCorrection() const { return Correction; }

	private:
		FVector2D Integral = FVector2D::ZeroVector;
		FVector2D Correction = FVector2D::ZeroVector;
		/** The last Tick's Kff (G - W) + Kp e, for BackCalculate. */
		FVector2D Others = FVector2D::ZeroVector;
	};

	/**
	 * One tick of the correction's slew: the plane moves from FromDeg towards
	 * ToDeg (both after the per-axis clamps), as a whole, no faster than the
	 * actuators allow. Returns the new plane. In order:
	 *
	 *  1. Step = ToDeg - FromDeg, shortened to MaxRateDegPerSec Dt (Euclidean;
	 *     no limit for MaxRateDegPerSec <= 0).
	 *  2. Each leg's move is RigidOffsets(From + Step) - RigidOffsets(From).
	 *     OutRateUse = the largest |move| / (Dt MaxRateCmS) over every leg
	 *     that counts and has MaxRateCmS > 0: the share of its rate the plane
	 *     asks of the busiest leg, measured here, before the next cut, and so
	 *     able to pass 1. Above Tuning.LegRateFraction the Step is scaled by
	 *     LegRateFraction / OutRateUse.
	 *     A leg the robot stands on counts for all of its move. A LIFTED leg
	 *     (bSupport false) counts only for the part of its move that takes it
	 *     UP, and that it can make: both ends of the move held inside its
	 *     travel (Allocate's own clamp) first. Where a lifted leg is changes
	 *     nothing about the base's attitude; it is carried with the plane to
	 *     keep its clearance, and only a leg late in coming UP loses any. One
	 *     that is being let down, or that rests on its stop and is not moving
	 *     at all, must not hold the whole plane back: carried far ahead of the
	 *     legs that stand, it would be the busiest leg by far, and the
	 *     correction would unwind at a fraction of what those legs can do. (A
	 *     lifted leg let down faster than it can go is told so all the same,
	 *     at its own rate: FLaw::Step's g2.)
	 *  3. Soft zone: for each support leg moving towards a limit of the
	 *     constraint set (Lo_i or Hi_i of RammsLevelAllocate.h, ExtraMarginCm
	 *     0) with less than Tuning.SoftZoneCm to go from where From puts it
	 *     (d, 0 or more), f_i = SoftZoneFloor + (1 - SoftZoneFloor) d /
	 *     SoftZoneCm. The Step is scaled by the least f_i.
	 *
	 * The plane stays rigid: every cut scales the whole Step. bOutLimited
	 * says a RATE cut it (1 or 2): the legs are going as fast as they may,
	 * and the law holds its integral meanwhile. The soft zone's taper is not
	 * reported: it slows the last of every approach to a limit for many
	 * ticks, and an integral held all that while could not trim a leg that
	 * rests at the zone's edge (where the wanted tilt is planned to leave
	 * it). With Dt <= 0: FromDeg, use 0, not limited.
	 * Tuning.LegRateFraction <= 0 means no leg-rate cut (the formula would
	 * stop the plane for good) and SoftZoneCm <= 0 no taper. A leg is moving
	 * towards a limit when its move, after the leg-rate cut, is over 1e-9 cm.
	 * Legs must have been through SupportMask.
	 */
	FVector2D SlewCorrection(TConstArrayView<FLegReach> Legs, const FVector2D& FromDeg, const FVector2D& ToDeg, double Dt,
		double MaxRateDegPerSec, const FTuning& Tuning, double& OutRateUse, bool& bOutLimited);

	/**
	 * How fast (deg/s) the legs the robot stands on can turn the correction
	 * plane along DirDeg (a direction over pitch and roll; its length does not
	 * matter): the plane's own rate, Tuning.MaxCorrectionRateDegPerSec, and no
	 * faster than keeps the busiest of those legs within Tuning.LegRateFraction
	 * of its MaxRateCmS (RigidOffsets of a degree along DirDeg gives each
	 * leg's cm per degree). The legs that count are the support legs (every
	 * leg when none stands): a lifted leg holds the plane back only while it
	 * is being brought up (SlewCorrection), which is not what the ground can
	 * be followed at in general. 0: nothing limits it (no plane rate, and no
	 * counting leg with a rate, or no LegRateFraction), or DirDeg is zero.
	 * Legs must have been through SupportMask. What the speed cap takes for
	 * the rate the ground may turn at.
	 */
	double PlaneRateDegPerSec(TConstArrayView<FLegReach> Legs, const FVector2D& DirDeg, const FTuning& Tuning);

	/**
	 * The seat's trim (deg; positive pitches the seat nose up against the
	 * chassis): what the level assist adds to the seat's own tilt, on a robot
	 * whose seat tilts. It has two parts.
	 *
	 * The SHARE is the part of the rider's wanted pitch that is held in the
	 * seat rather than in the base: the pitch the rider prefers, and the tilt
	 * a slope past the legs' reach leaves the base with, taken back out. It
	 * is a deliberate change of the rider's attitude, so the law moves it
	 * within the tilt policy (FLaw::Step) and gives it here.
	 *
	 * The COVER is of the base's pitch error while the legs catch the ground
	 * up: AnswerDeg, how far nose up of where it is wanted the base will be,
	 * by the ground's doing, when the seat's command lands (the law works it
	 * out: FLaw::Step, step 4). The seat takes it out, and the legs, bringing
	 * the base to where it is wanted, take the cover away again by
	 * themselves. Roll has no seat axis.
	 *
	 *   cover  = bCover ? -Deadband(AnswerDeg, SeatTrimDeadbandDeg) : 0
	 *   target = clamp(ShareDeg + cover, max(LowDeg, -SeatTrimMaxDeg),
	 *                                    min(HighDeg, SeatTrimMaxDeg))
	 *   trim moves to target by at most SeatTrimRateDegPerSec Dt
	 *
	 * LowDeg (0 or less) and HighDeg (0 or more) are the seat's own room about
	 * where the rider has it. A rate of 0 holds the trim where it is. Turning
	 * the trim into a seat joint command is the plant's.
	 */
	class FSeatTrim
	{
	public:
		/** The cover's own tuning (the law reads it; a test or the dry run
		 *  may change it). */
		FSeatCoverTuning Cover;

		/** Forget (the tuning stays). */
		void Reset()
		{
			const FSeatCoverTuning Kept = Cover;
			*this = FSeatTrim();
			Cover = Kept;
		}
		/** The trim has become part of the seat's own command (a fold): it is
		 *  zero now, with no motion. */
		void Zero()
		{
			Trim = 0.0;
			bBehind = false;
		}
		/** One tick; returns the trim (deg). */
		double Tick(double ShareDeg, double AnswerDeg, bool bCover, double LowDeg, double HighDeg, double Dt, const FTuning& Tuning);
		double Get() const { return Trim; }
		/** The last tick's rate held the trim short of its target. */
		bool IsBehind() const { return bBehind; }

	private:
		double Trim = 0.0;
		bool   bBehind = false;
	};

	/** What the drive-speed cap is told each tick. */
	struct FSpeedCapInputs
	{
		/** The size of A - W (deg, Euclidean, no deadband). */
		double ErrorDeg = 0.0;
		/** The slope the robot stands on (deg): the terrain estimate's
		 *  inclination. What the legs will have to turn the correction by
		 *  when that slope ends. */
		double SlopeDeg = 0.0;
		/** How fast the terrain estimate moved over this tick (deg/s, pitch
		 *  and roll), and whether that is the ground's doing: not on a tick
		 *  that reseeded the estimate, nor while the legs' own baselines move
		 *  (a stance being changed turns it with the ground standing still). */
		FVector2D GroundRateDegPerSec = FVector2D::ZeroVector;
		bool	  bGroundRateKnown = false;
		/** How far apart the legs the robot stands on are (cm; StandingSpanCm):
		 *  a break in the ground has passed under the robot once it has moved
		 *  that far. */
		double SpanCm = 0.0;
		/** How fast the legs can turn the correction (deg/s;
		 *  PlaneRateDegPerSec); 0: nothing limits them. */
		double LegsDegPerSec = 0.0;
		/** The chassis' speed over the ground as measured (cm/s). */
		double SpeedCmS = 0.0;
		/** The legs are stopped with the base beyond the tilt policy. */
		bool bPastReach = false;
	};

	/**
	 * The drive-speed cap: a scale on the rider's stick, 1 down to
	 * Tuning.MinSpeedScale and never 0.
	 *
	 * WHAT IT SLOWS FOR. The legs take a turn of the ground out by turning the
	 * correction, at no more than their own rate; what the ground turns
	 * faster than that, the rider gets. The ground turns under the robot only
	 * as the robot moves over it -- by its speed times the ground's curvature
	 * -- so the drive is slowed until the ground turns at a rate the legs can
	 * follow:
	 *
	 *   Follow = min(Cap.FollowMaxDegPerSec, Cap.FollowShare * LegsDegPerSec)
	 *            (the first alone with no leg rate known)
	 *
	 * The scale is the least of four terms, each from 1 down to the floor:
	 *  - THE TURN NOW: with Turn the low-pass (Cap.TurnFilterSeconds) of
	 *    GroundRateDegPerSec (towards zero on a tick it is not known):
	 *    Driven * Follow / |Turn|. Driven is the scale as the drive has had
	 *    time to answer it (a first-order lag, Cap.DriveLagSeconds): the
	 *    ground turns in proportion to the speed, so that is the scale at
	 *    which it would turn at Follow. (Over 1, and so nothing, while the
	 *    ground turns slower than that at the speed there is.)
	 *  - THE SLOPE'S END: the robot is levelled on a slope of SlopeDeg, and
	 *    where that slope ends the ground comes back to level within one
	 *    span's travel: it will turn SlopeDeg / SpanCm for every cm. That is
	 *    the one turn of the ground that can be known before it comes, so the
	 *    speed it can be followed at, ExitSpeed = Follow * SpanCm / SlopeDeg,
	 *    is kept to from the start: ExitSpeed / FreeSpeed. (The slope counts
	 *    as steep as it has been until the robot has moved
	 *    Cap.SlopeHeldSpans spans on from there: half way through its end the
	 *    slope left is half, and so is the way left to lose it in, so the
	 *    drive is not let go into the break.) FreeSpeed is the
	 *    speed the rider's stick is asking for: the low-pass
	 *    (Cap.FreeSpeedFilterSeconds) of |SpeedCmS| / Driven, taken only while
	 *    the robot moves (Cap.MinSpeedCmS) -- the speed as measured, so a
	 *    robot that is not getting the speed its stick asks (a drive at its
	 *    torque limit on a steep ramp) is not slowed for a speed it does not
	 *    have. No such term standing still, on level ground, or with no span.
	 *  - THE ERROR, as a backstop for what those two do not see: 1 at
	 *    ErrorDeg <= SpeedCapErrorDeg, the floor at SpeedCapErrorFullDeg or
	 *    more, a straight line between.
	 *  - PAST REACH: the floor outright with bPastReach.
	 * Standing at a limit is no reason to be slow, and nothing here looks at
	 * how much reach is left: a steady slope at the very end of the legs'
	 * travel is driven at its ExitSpeed like any other.
	 *
	 * The scale falls towards that at Cap.FallPerSec -- and no faster than
	 * brakes the robot at Cap.BrakeCmS2 (Cap.BrakeCmS2 / FreeSpeed per s) --
	 * and rises at Tuning.SpeedScaleRisePerSec. It starts at 1. The floor is
	 * clamp(MinSpeedScale, 0.01, 1): a tuning of 0 still leaves a crawl.
	 *
	 * WHAT IT CANNOT DO. It reacts: a break the robot meets at speed from
	 * level ground (nothing levelled yet, so nothing known to end) has turned
	 * the ground for a tick or two before anything here moves, and the drive
	 * takes as long again to slow. On a robot standing on two axles the first
	 * sign of a break is the base's own pitch, when the leading axle is on it;
	 * no leg meets it sooner. Slowing early for those takes looking ahead.
	 */
	class FSpeedScale
	{
	public:
		/** Its own tuning; a test or the dry run may change it. */
		FSpeedCapTuning Cap;

		/** Forget (the tuning stays): full speed. */
		void Reset()
		{
			const FSpeedCapTuning Kept = Cap;
			*this = FSpeedScale();
			Cap = Kept;
		}
		/** One tick; returns the scale. */
		double Tick(const FSpeedCapInputs& In, double Dt, const FTuning& Tuning);
		/** One tick with nothing to cap for: the scale rises towards 1. */
		double Relax(double Dt, const FTuning& Tuning);
		double Get() const { return Scale; }
		/** The speed the rider's stick is taken to ask for (cm/s), the
		 *  ground's turning rate as filtered (deg/s), and the last tick's
		 *  Follow (deg/s) and ExitSpeed (cm/s; 0: no such term). */
		double	  GetFreeSpeedCmS() const { return FreeSpeed; }
		FVector2D GetTurnDegPerSec() const { return Turn; }
		double	  GetFollowDegPerSec() const { return Follow; }
		double	  GetExitSpeedCmS() const { return ExitSpeed; }

	private:
		double	  Scale = 1.0;
		double	  Driven = 1.0;
		double	  FreeSpeed = 0.0;
		FVector2D Turn = FVector2D::ZeroVector;
		double	  Follow = 0.0;
		double	  ExitSpeed = 0.0;
		/** The steepest the slope has been of late (deg), and how far the
		 *  robot has moved since it was (cm). */
		double Steepest = 0.0;
		double SinceSteepestCm = 0.0;
	};

	/**
	 * At rest: once the chassis has been slower than QuietSpeedCmS with the
	 * error under SettleErrorDeg, the stick still and bMayRest, for
	 * SettleSeconds without a break, it is quiet -- the law then freezes the
	 * integral and writes nothing, so the position servos simply hold. It
	 * wakes (and the count starts again) when the error reaches WakeErrorDeg,
	 * the stick moves, the chassis moves at QuietSpeedCmS or more, or bMayRest
	 * falls. Between SettleErrorDeg and WakeErrorDeg a count under way is
	 * broken but a quiet already reached is kept. (This timer and
	 * FDivergence's count as run a hair, 1e-9 s, early: ten ticks of 0.1 s do
	 * not quite add up to 1.)
	 */
	class FQuiet
	{
	public:
		void Reset() { *this = FQuiet(); }
		/**
		 * One tick; returns whether it is quiet. ErrorDeg is the size of A - W
		 * (deg, Euclidean, no deadband); bMayRest is the law's leave (the
		 * wanted tilt has arrived at its goal, and no leg's baseline is being
		 * moved under it).
		 */
		bool Tick(double SpeedCmS, double ErrorDeg, bool bStick, bool bMayRest, double Dt, const FTuning& Tuning);
		bool IsQuiet() const { return bQuiet; }

	private:
		double SettledSeconds = 0.0;
		bool   bQuiet = false;
	};

	/**
	 * The mark of a sign or geometry error: the error is large, does not
	 * come back, and the base has been moving WITH the legs' correction
	 * where a correction should move it the other way.
	 *
	 * An EPISODE runs while bCounts and ErrorDeg > DivergenceErrorDeg. In it:
	 *  - a timer runs, and the worst error since it started is kept. Both
	 *    start again when the error has come back DivergenceReliefDeg from
	 *    the worst (a kick being taken out; the count then starts again from
	 *    that tick, its error the new worst);
	 *  - the evidence is summed: each tick, how far the attitude moved along
	 *    the direction the correction moved in (deg, signed; nothing on a
	 *    tick the correction stood still). A base wired the right way moves
	 *    AGAINST its correction, so the sum goes negative; one wired
	 *    backwards moves with it, degree for degree. The correction meant
	 *    is the one the joints have made (MeasuredCorrection): joints move
	 *    after the ground that made the law move them, so a ground that
	 *    steps does not coincide with its own answer.
	 * The episode ends, and timer, worst and evidence are forgotten, when
	 * bCounts is false or the error is at or under DivergenceErrorDeg.
	 * Diverged, and it stays so until Reset, once the timer has reached
	 * Tuning.FaultSeconds with the evidence at DivergenceWithLegsDeg or more.
	 *
	 * Why the evidence, and not the plan's "growing ... while unsaturated and
	 * not rate-limited": with the terrain feed-forward a base wired backwards
	 * runs away at the correction's full rate and is on a limit within a
	 * second, so it is rate-limited all the way and then stands still. Read
	 * as a steady state it is exactly a slope past a limit, which is
	 * nobody's fault (a correction held at its per-axis limit on a steep
	 * cross fall leaves a large steady error for good). Only how it got
	 * there tells the two apart. The law counts the ticks whose allocation
	 * gave the whole plane (Fraction 1), rate-limited or not.
	 *
	 * What it cannot tell apart from a fault: ground that turns faster than
	 * the legs can follow for a degree or more while the error is already
	 * over DivergenceErrorDeg, ending past the tilt policy on a limit that
	 * is not the legs' reach. That takes a slope well past FaultTiltDeg.
	 */
	class FDivergence
	{
	public:
		void Reset() { *this = FDivergence(); }
		/** One tick; returns whether it has diverged. ErrorDeg is the size of
		 *  A - W (deg, Euclidean, no deadband); AttitudeDeg is A and
		 *  CorrectionDeg the correction the joints have made (deg). */
		bool Tick(double ErrorDeg, const FVector2D& AttitudeDeg, const FVector2D& CorrectionDeg, bool bCounts, double Dt, const FTuning& Tuning);
		bool HasDiverged() const { return bDiverged; }

	private:
		double	  Seconds = 0.0;
		double	  Worst = 0.0;
		double	  WithLegsDeg = 0.0;
		FVector2D LastAttitudeDeg = FVector2D::ZeroVector;
		FVector2D LastCorrectionDeg = FVector2D::ZeroVector;
		bool	  bHasLast = false;
		bool	  bDiverged = false;
	};
} // namespace RammsLevel
