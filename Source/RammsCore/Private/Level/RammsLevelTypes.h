// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Curb/RammsCurbRunStats.h"

/**
 * The level assist's shared types, as plain C++ fed made-up numbers so each
 * piece can be tested on its own. Not reflected, not exported.
 *
 * The level assist keeps a robot's base level while it drives over sloping
 * ground. It never moves a leg's own command (the BASELINE, which the drive
 * modes, the curb controllers and the rider go on writing): it adds one rigid
 * plane of height OFFSETS over all the levelled legs. Everything in
 * Private/Level works in that height space; turning a leg's height change
 * into joint angles is a later layer's.
 *
 * CONVENTIONS, the same in every file of Private/Level
 *  - Lengths are cm, times s, angles DEGREES (never radians) in every
 *    interface. Everything is double.
 *  - An attitude, a tilt, a correction or a plane is an FVector2D: X pitch,
 *    positive nose up; Y roll, positive when the robot's +Y side is up. (The
 *    curb controller's convention, RammsCurb::FAttitudeLoop.)
 *  - The chassis frame: x forward, y towards +Y, z up, cm.
 *  - A leg's height is its wheel centre's z in the chassis frame. A height
 *    change dz is positive when the leg RETRACTS (the wheel comes up towards
 *    the chassis, so that end of the chassis comes down).
 *  - A correction plane C gives each leg dz = x tan(C.X) + y tan(C.Y) (+ a
 *    common height term): a positive pitch correction retracts the front legs
 *    and the nose drops; a positive roll correction retracts the +Y legs and
 *    the +Y side drops (RammsCurb::AttitudeOffsets' sense). So a base that
 *    reads attitude A on some stance reads about A - C once C is applied.
 *  - The size of an attitude is its inclination (Inclination: the angle its
 *    plane makes with the horizontal, RammsCurb::InclinationDeg). The size of
 *    a RATE of attitude, and the length used to cap a vector of angles
 *    (ClampTotal), is the plain Euclidean norm of (pitch, roll) in degrees:
 *    the two agree to a hundredth of a degree at the angles in use.
 *  - Dt <= 0 is "no time passes": every Tick / Step below must then leave its
 *    filters, integrals and timers where they are and return what it last
 *    returned (seeding from the reading where it was not yet seeded).
 */
namespace RammsLevel
{
	/** One value per levelled leg, in the legs' order. */
	using FLegValues = TArray<double, TInlineAllocator<8>>;

	// ---- constants the plan fixes that are not tuning ----

	/** The law never steps further than this in one call (s): Dt is clamped. */
	constexpr double MaxStepSeconds = 0.1;
	/** The loop's integral runs only on at least this many support legs. */
	constexpr int32 MinSupportForIntegral = 3;
	/** Within the soft zone of a travel limit, motion towards the limit is
	 *  slowed to this share at the limit itself. */
	constexpr double SoftZoneFloor = 0.1;
	/** Allocate's search for the largest share of a plane that fits. */
	constexpr int32 AllocateBisections = 12;
	/** A leg displaced less than this (cm) from its baseline is not counted
	 *  in TravelMargin; and what TravelMargin returns when none is. */
	constexpr double TravelMarginMinMoveCm = 0.01;
	constexpr double NoTravelMarginCm = 1.0e6;
	/** The seat trim ignores a base pitch smaller than this (deg). */
	constexpr double SeatTrimDeadbandDeg = 0.2;
	/** The speed cap's reach term: full speed with this much travel left
	 *  (cm), the floor with this little. */
	constexpr double SpeedCapMarginFullCm = 3.0;
	constexpr double SpeedCapMarginFloorCm = 0.5;
	/** ... counted only while the terrain estimate, the correction or the
	 *  wanted tilt moves faster than this (deg/s), and this long (s) after. */
	constexpr double SpeedCapMovingDegPerSec = 0.5;
	constexpr double SpeedCapMovingHoldSeconds = 1.0;
	/** The speed scale falls no faster than this (per s). */
	constexpr double SpeedScaleFallPerSec = 2.0;
	/** At rest: the chassis slower than this (cm/s). */
	constexpr double QuietSpeedCmS = 2.0;
	/** The wanted tilt has arrived: within this (deg) of its goal, and still. */
	constexpr double TiltArrivedDeg = 0.05;
	/** ... or, of a goal that is itself a live estimate, as good as arrived:
	 *  within TiltArrivedDeg and slower than this (deg/s). (At the tilt
	 *  policy's 4 deg/s^2 that is what stops within the 0.05 deg.) */
	constexpr double TiltArrivedRateDegPerSec = 0.5;
	/** A release is done once the correction is under this (deg, Euclidean),
	 *  the ride-height shift under this (cm) and the seat trim under this
	 *  (deg): the offsets are then set exactly to zero. */
	constexpr double ReleaseDoneDeg = 0.1;
	constexpr double ReleaseDoneCm = 0.1;
	/** A closed-loop release that has not brought the wanted tilt to the
	 *  ground's in this long (s) finishes open loop. */
	constexpr double ReleaseMaxSeconds = 6.0;
	/** Divergence: the error over this (deg) and not having come back this
	 *  far (deg) from its worst, for FTuning::FaultSeconds, the base having
	 *  moved at least this far (deg) WITH the legs' correction meanwhile. */
	constexpr double DivergenceErrorDeg = 3.0;
	constexpr double DivergenceReliefDeg = 0.5;
	constexpr double DivergenceWithLegsDeg = 1.0;
	/** The loop asks for more than a per-axis limit (MaxCorrectionDeg) by
	 *  more than this (deg): the correction is then held at that limit, which
	 *  is reported as being at reach. (The slack keeps a correction that
	 *  merely rests on its limit from reading so.) */
	constexpr double AxisLimitSlackDeg = 0.05;
	/** The offsets' common height counts as having moved only from this much
	 *  (cm): the last bit of the arithmetic of a mean that is zero is not a
	 *  change of ride height. */
	constexpr double RideStepSlackCm = 1.0e-9;
	/** ... and when it does step (the legs that stand have changed), it picks
	 *  the ride height's rate up, and loses it, over this long (s). */
	constexpr double RideHeightRampSeconds = 0.25;

	/**
	 * The level assist's tuning. A plain mirror of the reflected
	 * FRammsLevelTuning a later milestone adds (Public/RammsLevelTypes.h):
	 * the same member names and defaults, so keep them in step.
	 */
	struct FTuning
	{
		/** The attitude "level" means (deg). */
		FVector2D LevelReferenceDeg = FVector2D::ZeroVector;

		/** The loop: correction = Kff (G - W) + I + Kp e (FLevelLoop). Ki per s;
		 *  the integral within +-IntegralMaxDeg per axis; e less DeadbandDeg. */
		double FeedForwardGain = 1.0;
		double ProportionalGain = 0.3;
		double IntegralGain = 2.0;
		double IntegralMaxDeg = 3.0;
		double DeadbandDeg = 0.15;
		/** Low-pass time constants (s) of the measured attitude and of the
		 *  terrain estimate. */
		double AttitudeFilterSeconds = 0.05;
		double TerrainFilterSeconds = 0.15;

		/** The most correction per axis (deg; X pitch, Y roll), either way. */
		FVector2D MaxCorrectionDeg = FVector2D(15.0, 15.0);
		/** The correction moves no faster than this (deg/s, Euclidean), and so
		 *  that no leg is asked for more than LegRateFraction of its rate. */
		double MaxCorrectionRateDegPerSec = 10.0;
		double LegRateFraction = 0.8;

		/** The tilt policy: what the rider is deliberately tilted by is at most
		 *  MaxTiltDeg (off the reference) and changes within this rate and
		 *  acceleration (deg/s, deg/s^2; Euclidean, both axes together). */
		double MaxTiltDeg = 5.0;
		double TiltRateDegPerSec = 3.0;
		double TiltAccelDegPerSec2 = 4.0;

		/** A support leg is kept this far (cm) inside its travel. */
		double LegRangeMarginCm = 0.5;
		/** Within this (cm) of that, motion towards the limit tapers; and the
		 *  wanted tilt is planned with the limits pulled in by as much. */
		double SoftZoneCm = 2.0;
		/** A leg joins the support set when its wheel bottom is within In (cm)
		 *  of the lowest baseline bottom, and leaves it beyond Out. */
		double SupportBandInCm = 1.0;
		double SupportBandOutCm = 2.0;
		/** A lifted leg that cannot follow the plane may lose clearance down to
		 *  this (cm) before it limits the correction. */
		double MinLiftedClearanceCm = 2.5;
		/** What the plant asks of its belly points (cm): it fills
		 *  FClearancePoint::MinCm with this. Allocate reads MinCm only. */
		double MinBellyClearanceCm = 3.0;

		/** How far (cm) the ride height may rise (Up) or sink (Down) from the
		 *  stance's to make a plane fit, and how fast it moves (cm/s): that
		 *  shift, and the offsets' common height when the legs that stand
		 *  change (0 or less: that one is then left to the legs' own rate). */
		double RideHeightUpCm = 0.0;
		double RideHeightDownCm = 0.0;
		double RideHeightRateCmPerSec = 3.0;

		/** The drive-speed cap (FSpeedScale): its floor, the error (deg) where
		 *  it starts to bind and where it is at the floor, how fast it lets go
		 *  (per s). */
		double MinSpeedScale = 0.25;
		double SpeedCapErrorDeg = 1.0;
		double SpeedCapErrorFullDeg = 3.0;
		double SpeedScaleRisePerSec = 0.5;

		/** At rest (FQuiet): settled when the error has been under
		 *  SettleErrorDeg for SettleSeconds; awake again at WakeErrorDeg. */
		double SettleErrorDeg = 0.3;
		double SettleSeconds = 1.0;
		double WakeErrorDeg = 0.5;

		/** Faults: the tilt (deg off the reference) reported as an error; how
		 *  long a divergence must last (s); how long the attitude may be
		 *  missing before it is reported (s). */
		double FaultTiltDeg = 12.0;
		double FaultSeconds = 1.5;
		double MeasurementLossSeconds = 0.5;

		/** The seat trim (FSeatTrim): whether it runs, its limit (deg), rate
		 *  (deg/s) and the low-pass of the pitch it answers (s). */
		bool   bSeatCoversTransients = true;
		double SeatTrimMaxDeg = 8.0;
		double SeatTrimRateDegPerSec = 10.0;
		double SeatFilterSeconds = 0.1;
	};

	/**
	 * One levelled leg as the law sees it this tick. Everything but bSupport,
	 * BaselineClearanceCm and CarryShare is an input the plant measures; those
	 * three are filled by SupportMask.
	 *
	 * A ganged pair of wheels on one actuator is ONE leg, at the middle of
	 * the pair (y = 0 for a pair across the robot), with bRoll false.
	 */
	struct FLegReach
	{
		/** Its wheel centre in the chassis frame (cm; x forward, y to +Y). */
		FVector2D XY = FVector2D::ZeroVector;
		/** Its wheel centre's z (cm, chassis frame) at its BASELINE command:
		 *  where the leg would be with no offset. */
		double BaseZ = 0.0;
		/** The same z as its joints measure it now. */
		double MeasuredZ = 0.0;
		/** Its travel at the baseline's fore/aft position: the lowest (most
		 *  extended, MinZ) and highest (most retracted, MaxZ) z it reaches. */
		double MinZ = 0.0;
		double MaxZ = 0.0;
		/** How fast its z can move (cm/s); 0 or less: not limited. */
		double MaxRateCmS = 0.0;
		/** Its wheel's radius (cm): its wheel bottom is at BaseZ - RadiusCm. */
		double RadiusCm = 0.0;
		/** Roll reaches it. False for a leg a roll correction must not move (a
		 *  ganged pair): it then takes x tan(pitch) only. */
		bool bRoll = true;
		/** On the ground at its baseline, as SupportMask finds. */
		bool bSupport = true;
		/** How far (cm, 0 or more) its wheel bottom is above the lowest
		 *  baseline wheel bottom, as SupportMask finds: for a lifted leg, the
		 *  clearance the stance gives it. */
		double BaselineClearanceCm = 0.0;
		/** How far it counts as CARRYING the robot (0..1), as SupportMask
		 *  finds: 1 with its wheel bottom level with the lowest baseline
		 *  bottom, less in a straight line as its stance lifts it, 0 from the
		 *  inner support band up. What MeasuredCorrection weighs each leg by
		 *  (squared). Unlike bSupport it has no memory and no step: a leg
		 *  being lifted off the ground, or set down, fades out of and into
		 *  the measured correction. A plant that gives the support set itself
		 *  need not give this (the law then takes 1 for a leg that stands and
		 *  0 for one that does not). */
		double CarryShare = 1.0;
	};

	/** A point of the robot's underside that must stay clear of the ground. */
	struct FClearancePoint
	{
		/** Where (chassis frame, cm). */
		FVector2D XY = FVector2D::ZeroVector;
		/** The underside's z there (cm, chassis frame). */
		double UndersideZ = 0.0;
		/** The clearance (cm) it must keep over the ground. */
		double MinCm = 0.0;
	};

	/** What Allocate gives: every leg's offset for one correction plane. */
	struct FAllocation
	{
		/** Each leg's height offset (cm, positive retracts), in the legs' order. */
		FLegValues Dz;
		/** The share s of the wanted plane (0..1) that fits; 1: all of it. */
		double Fraction = 1.0;
		/** The plane applied (deg): s times the wanted one after the per-axis
		 *  clamps. */
		FVector2D AppliedDeg = FVector2D::ZeroVector;
		/** The ride-height shift used (cm): positive RAISES the chassis (every
		 *  leg extended by it, so it enters each dz with a minus sign); 0 is
		 *  the stance's own ride height. */
		double HeightCm = 0.0;
		/** With Fraction < 1, what stopped it: the leg (index into the legs)
		 *  or the clearance point (index into the points) whose limit is
		 *  crossed first beyond s; INDEX_NONE where it is not that kind. Both
		 *  INDEX_NONE at Fraction 1. */
		int32 LimitingLeg = INDEX_NONE;
		int32 LimitingPoint = INDEX_NONE;
		/** TravelMargin of Dz (cm). */
		double MinMarginCm = NoTravelMarginCm;
		/** The least share (0..1) of travel left to a support leg, from its
		 *  baseline to its margin-ed limit in the direction it was moved: 1
		 *  with no leg moved, 0 with one on its limit. What level.reach shows. */
		double ReachLeft = 1.0;
		/** The largest leg-rate use of the tick (SlewCorrection's). Allocate
		 *  leaves it 0; the law fills it. */
		double RateUse = 0.0;
	};

	/** The arbiter's states. Offsets are non-zero only in Active, Holding,
	 *  Releasing and a Faulted that is still ramping out. */
	enum class EState : uint8
	{
		/** Switched off: no offsets, the motors not claimed. */
		Off,
		/** Levelling. */
		Active,
		/** The attitude is missing: the offsets are frozen as they were. */
		Holding,
		/** Giving the base back to its stance, within the tilt policy. */
		Releasing,
		/** A curb controller has the legs: the offsets were folded into the
		 *  baselines and are zero. */
		Yielded,
		/** Switched on, with zero offsets, waiting for a mode it may level in
		 *  and a valid attitude. */
		Suspended,
		/** A divergence was caught: the correction is ramped out open loop,
		 *  then it stays off until switched off and on again. */
		Faulted
	};

	/** What a tick of the arbiter asks to be done, once. The law applies each
	 *  to its own state (FLaw::Step says how); the shell around it does the
	 *  part that touches motors. */
	enum class EAction : uint8
	{
		None,
		/** Begin levelling from nothing: correction, integral and ride shift
		 *  zero, the wanted tilt set to the measured attitude. */
		StartFromAttitude,
		/** The offsets become part of the baselines (the shell rebases; the
		 *  law's correction goes to zero with no motion). */
		Fold,
		/** A plane has been taken back out of the baselines (the shell has
		 *  rebased): the law's correction becomes that plane, the wanted tilt
		 *  the measured attitude. */
		Adopt,
		/** Go on from the correction as it stands: the wanted tilt set to the
		 *  measured attitude, the filters seeded afresh. */
		ColdResume,
		/** Start giving the base back. */
		BeginRelease,
		/** Hold the offsets as they are. */
		Freeze,
		/** The offsets are set exactly to zero now. */
		Clear
	};

	/** The active drive mode, as far as the level assist cares. */
	enum class EMode : uint8
	{
		/** One it may level in. */
		Allowed,
		/** One it must let go of gently (low level; a mode not enabled):
		 *  release within the policy, then wait. */
		Release,
		/** One that takes the legs (the curb mode, or any mode while a taker
		 *  that was yielded to still holds them): no offsets at all. */
		Taken
	};

	/** What the status line says; the text is made by a later layer. */
	enum class EStatus : uint8
	{
		Off,
		/** Active, inside reach. */
		Levelling,
		/** Active and at rest: nothing is being written. */
		Settled,
		/** Active, the correction scaled back at a travel or clearance limit,
		 *  or held at a per-axis limit the loop asks to pass. */
		AtReach,
		/** ... and the base more than MaxTiltDeg off the reference: "slope
		 *  past levelling reach". A warning; the drive is slowed, not stopped. */
		PastReach,
		/** The base more than FaultTiltDeg off the reference. An error;
		 *  levelling goes on. */
		TiltFault,
		/** Holding, the attitude missing for less than MeasurementLossSeconds. */
		Holding,
		/** ... for longer. A warning. */
		AttitudeLost,
		Releasing,
		Yielded,
		/** Suspended: waiting for a mode or an attitude. */
		Waiting,
		/** Faulted. An error. */
		Diverged
	};

	/** An attitude's size (deg): its plane's angle with the horizontal. */
	inline double Inclination(const FVector2D& Deg)
	{
		return RammsCurb::InclinationDeg(Deg.X, Deg.Y);
	}

	/** Deg with each axis within +-MaxDeg's (negative limits count as 0). */
	inline FVector2D ClampAxes(const FVector2D& Deg, const FVector2D& MaxDeg)
	{
		const double Mx = FMath::Max(0.0, MaxDeg.X);
		const double My = FMath::Max(0.0, MaxDeg.Y);
		return FVector2D(FMath::Clamp(Deg.X, -Mx, Mx), FMath::Clamp(Deg.Y, -My, My));
	}

	/** Deg shortened, keeping its direction, to a Euclidean length of at most
	 *  MaxDeg (0 for MaxDeg <= 0). */
	inline FVector2D ClampTotal(const FVector2D& Deg, double MaxDeg)
	{
		const double Size = Deg.Size();
		if (MaxDeg <= 0.0)
		{
			return FVector2D::ZeroVector;
		}
		return Size > MaxDeg ? Deg * (MaxDeg / Size) : Deg;
	}

	/**
	 * How much a point at XY (chassis frame, cm) rises under the plane Deg,
	 * before any height term: x tan(pitch) + y tan(roll), the roll part left
	 * out with bRoll false. For a leg this is its dz (positive retracts).
	 */
	inline double PlaneRise(const FVector2D& XY, bool bRoll, const FVector2D& Deg)
	{
		const double Pitch = XY.X * FMath::Tan(FMath::DegreesToRadians(Deg.X));
		return bRoll ? Pitch + XY.Y * FMath::Tan(FMath::DegreesToRadians(Deg.Y)) : Pitch;
	}
} // namespace RammsLevel
