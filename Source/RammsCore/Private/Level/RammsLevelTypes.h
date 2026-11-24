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
	 *  (deg): the offsets are then set exactly to zero. As good as nothing:
	 *  a release's own ramp arrives at zero within the tilt policy, and what
	 *  it leaves to that last step must be no step at all (a thousandth of a
	 *  degree is a hundredth of a millimetre at a leg; a tenth of a degree
	 *  was a millimetre, and a raised leg's crank dropped by it at once). */
	constexpr double ReleaseDoneDeg = 0.001;
	constexpr double ReleaseDoneCm = 0.001;
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
	/** The correction that would hold the reference cannot all be given: said
	 *  ("at reach") once what the legs must leave is over On (deg), and until
	 *  it is back under Off. The gap keeps a slope at the very end of the
	 *  reach from being reported one tick and not the next. */
	constexpr double AtReachOnDeg = 0.15;
	constexpr double AtReachOffDeg = 0.05;
	/** A correction counts towards the per-axis limit's share of the reach
	 *  from this much (deg). */
	constexpr double ReachMinCorrectionDeg = 0.01;
	/** A leg's baseline has MOVED once it is this far (cm) from where it was
	 *  last found still: a stance being ramped, a height being jogged. Under
	 *  it is the last digits of a joint read back. */
	constexpr double BaselineStillCm = 0.02;
	/** The standing legs span at least this (cm) for the ground's turning
	 *  rate (FGroundGate): a robot standing on a line of legs has no span,
	 *  and none would mean the ground could turn at any rate. */
	constexpr double GroundSpanMinCm = 10.0;
	/** Standing still before a rest, the integral trims the error down to
	 *  this (deg) rather than to the loop's deadband (FLevelLoop). What a
	 *  walk or a knock wound into the integral otherwise leaves the base
	 *  resting anywhere inside the deadband, most often on its edge. */
	constexpr double RestTrimDeadbandDeg = 0.05;
	/** PlanAdoption's rounds: the offsets a plane leaves decide which legs
	 *  stand, which decides the offsets. Who stands does not move with a
	 *  height every leg shares, so the second round already agrees with the
	 *  first; the others are for a plane cut at the legs' reach. */
	constexpr int32 AdoptPasses = 4;

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
	 * How fast the GROUND's attitude can change under a robot (FGroundGate).
	 * Not part of FTuning: nothing here is a robot's to author (the one figure
	 * a robot does bring, how far apart its standing legs are, is read off the
	 * legs), and FTuning mirrors the reflected tuning member for member.
	 *
	 * The ground turns under the robot only as the robot moves over it: a
	 * break of B deg has passed under legs L cm apart once the robot has moved
	 * L, so the ground the robot stands on turns B / L deg per cm travelled.
	 * What changes the attitude faster than that is the BODY's doing (braking,
	 * a start, a knock, a leg set down), and is left to pass.
	 */
	struct FGroundTuning
	{
		/** The sharpest grade break (deg) the ground's attitude is followed
		 *  through whole: with the standing legs L cm apart it may turn
		 *  BreakDeg / L deg for every cm the robot moves. 0 or less: no bound
		 *  at all (the attitude as read is taken for the ground's, at once). */
		double BreakDeg = 12.0;
		/** ... and this much (deg/s) besides, moving or not: joints settling
		 *  under the load, a leg model a little off while the correction
		 *  moves, a stance rocking onto a leg that is lifted or set down. (A
		 *  knock of a degree and a half that is gone in a third of a second
		 *  is followed by under half a degree at this.) */
		double SettleDegPerSec = 1.25;
		/** A base being braked or driven off harder than Start (cm/s^2) pitches
		 *  on its tyres and legs by its own inertia, and its attitude then
		 *  says nothing of the ground: from there both allowances above are
		 *  given up in a straight line, all of them at Full, and they come
		 *  back over BodySettleSeconds (s) once the base moves steadily again.
		 *  (About 0.15 and 0.3 of gravity: a drive slowed by the speed cap, or
		 *  meeting a grade break, stays under the first; letting go of the
		 *  stick at speed is past the second.) */
		double BodyAccelStartCmS2 = 150.0;
		double BodyAccelFullCmS2 = 300.0;
		double BodySettleSeconds = 0.5;
	};

	/**
	 * The drive-speed cap's own figures (FSpeedScale). Not part of FTuning, as
	 * FGroundTuning is not: FTuning mirrors the reflected tuning member for
	 * member, and what a robot does bring to the cap (how fast its legs turn
	 * the correction, how far apart they stand) is read off the legs.
	 *
	 * The cap is about one thing: how fast the GROUND turns under the robot,
	 * against how fast the legs can turn the correction to take it out. The
	 * ground turns only as the robot moves over it -- its turning rate is the
	 * robot's speed times the ground's curvature -- so there is a speed the
	 * legs can follow, and the drive is slowed to it.
	 */
	struct FSpeedCapTuning
	{
		/** The ground may turn under the robot at this share of the rate the
		 *  legs can turn the correction at: the rest is theirs to take out
		 *  what the law's own delay let through. */
		double FollowShare = 0.8;
		/** ... and no faster than this (deg/s) however quick the legs are: a
		 *  turn of the ground is answered a tick or two after it is read
		 *  (the terrain estimate's own filter, a command interval), what it
		 *  turns by in that time the rider gets, and when the turn ends the
		 *  legs take that back out in a few tenths of a second. At 6 deg/s
		 *  the base is left about a degree behind, and closing that is still
		 *  inside the rider's 4 deg/s; at 8 it was not (the dry run's slope
		 *  ends, RammsLevelDryRunTests). */
		double FollowMaxDegPerSec = 6.0;
		/** A slope that has begun to end counts as steep as it was until the
		 *  robot has moved this many spans on: the break is not past the legs
		 *  until then, and the drive is not let go into it. */
		double SlopeHeldSpans = 1.5;
		/** The ground's turning rate is the terrain estimate's, low-passed
		 *  over this long (s): one tick's change of an estimate is mostly its
		 *  sensor's noise. */
		double TurnFilterSeconds = 0.2;
		/** How long the drive is taken to need to answer the scale (s), and
		 *  the low-pass (s) on the speed the rider's stick is asking for
		 *  (the speed measured over the scale the drive has answered). */
		double DriveLagSeconds = 0.15;
		double FreeSpeedFilterSeconds = 0.3;
		/** The scale falls no faster than this (per s) ... */
		double FallPerSec = SpeedScaleFallPerSec;
		/** ... and no faster than brakes the robot this hard (cm/s^2; 0 or
		 *  less: no such limit): FGroundTuning::BodyAccelStartCmS2, the
		 *  braking from which the law stops taking the base's pitch for the
		 *  ground's. A cap that braked harder would blind the very estimate
		 *  it slows the robot for. */
		double BrakeCmS2 = 150.0;
		/** Slower than this (cm/s) the robot is not being driven anywhere:
		 *  the speed its stick asks for is not worked out from it. */
		double MinSpeedCmS = 2.0;
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
		/** Something commands it: BaseZ is a baseline. False for a leg that was
		 *  let go (driven by hand for a moment): its BaseZ is then only where
		 *  its joint is, and its coming and going there is not a stance being
		 *  moved. */
		bool bHasBaseline = true;
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

	/** What stops a correction, in the order the status names them. */
	enum class EReachLimit : uint8
	{
		None,
		/** A leg the robot stands on is at the end of its travel. */
		LegTravel,
		/** A wheel the stance carries off the ground would come too near it. */
		LiftedClearance,
		/** The underside would come too near the ground. */
		Underside,
		/** The most correction allowed on an axis (MaxCorrectionDeg). */
		AxisLimit
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
		/** ... and what kind of limit that is (None at Fraction 1, or when only
		 *  the ride window is at fault). */
		EReachLimit Limit = EReachLimit::None;
		/** The least room (cm) left to whatever is nearest stopping the plane:
		 *  TravelMargin of Dz, a lifted leg's clearance above the least it may
		 *  keep, an underside point's above the least it must keep, and a
		 *  per-axis limit's room turned into cm at the standing leg furthest
		 *  from the standing legs' middle. NoTravelMarginCm when nothing is
		 *  used at all. */
		double MinMarginCm = NoTravelMarginCm;
		/** The least share (0..1) left of whatever limits the plane, each
		 *  measured from none of it used to all of it: a support leg's travel
		 *  from its baseline to its margin-ed limit in the direction it was
		 *  moved; a lifted leg's rise to the most it may be asked; an underside
		 *  point's ground rise to the most it may take; each axis' correction
		 *  to MaxCorrectionDeg. 1 with nothing used, 0 with one on its limit.
		 *  What level.reach shows; NearestLimit is whose share it is. */
		double		ReachLeft = 1.0;
		EReachLimit NearestLimit = EReachLimit::None;
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
		/** Active, and the correction that would hold the reference cannot all
		 *  be given: the legs' travel, a clearance or a per-axis limit stops
		 *  it (FOutputs::ReachLimit says which), and the base is left tilted
		 *  by the rest. */
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
