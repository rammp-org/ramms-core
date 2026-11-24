// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Level/RammsLevelLaw.h"
#include "Level/RammsLevelTypes.h"
#include "Templates/Function.h"

/**
 * A kinematic dry run of the level assist, as plain C++ for the automation
 * tests: a made-up robot is driven in a straight line over a ground given as
 * a height function, the real law (FLaw::Step) running once per frame against
 * a plant stepped every millisecond. No map, no physics engine, no robot's
 * own numbers. Not reflected, not exported.
 *
 * THE PLANT, per physics step (FOptions::PhysicsStepSeconds):
 *  - Drive: the speed follows SpeedCmS times the law's speed scale with a
 *    first-order lag (DriveLagSeconds); the chassis origin moves that far
 *    along its heading. The heading (YawDeg) and the track never change.
 *  - Body, optional: a base being braked or driven off pitches on its tyres
 *    and legs by its own inertia. The body's pitch follows
 *    PitchPerAccelDeg times the drive's acceleration (cm/s^2, positive
 *    speeding up: nose up; braking: nose down) with a first-order lag
 *    (BodyLagSeconds), and is added to the solved attitude as a kick is:
 *    whatever the legs do. It is the body's, not the ground's.
 *  - Each leg: its command is baseline + offset (cm, a wheel-centre height in
 *    the law's terms), given once per frame to a RammsMotion::FFollower,
 *    which is stepped here within the leg's rate and acceleration; the
 *    joint follows the follower's setpoint with a first-order lag
 *    (ServoLagSeconds), plus the sag below, and stops dead at MinZ and MaxZ.
 *    That joint height q is what the law is given as MeasuredZ.
 *  - The wheel's TRUE height is StanceZ + Gain (q - StanceZ): Gain 1 is a
 *    leg the law models exactly; another value is a leg whose real travel is
 *    that multiple of what the law believes (the law still reads q).
 *  - Sag, optional: a touching leg's joint settles retracted by SagCm times
 *    its share of the load times the number of legs (so SagCm is the sag
 *    with the load shared evenly), following with the servo lag. A leg's
 *    share is the sum over its touching wheels of the least-norm loads that
 *    balance the centre of mass (clipped at 0 and renormalised).
 *  - The seat, on a robot that has one (FRobot::bSeat): the law's trim is
 *    given once per frame, as a leg's command is, to a follower stepped here
 *    within SeatMaxRateDegPerSec, and the seat's tilt follows that with a
 *    first-order lag (SeatLagSeconds). The RIDER's attitude is the chassis'
 *    true attitude with the seat's tilt added to its pitch. A take-over's
 *    fold puts the trim into the seat's own command, which the taker then
 *    walks back to zero at the tilt policy's rate (as a crossing brings the
 *    rider to their own setting).
 *  - Pose, every PoseStepSeconds: SolveRest over the wheels' true bottoms.
 *    Floating wheels, three-point stances and rocking come out of that; the
 *    attitude is NOT a fit through the wheels. A kick (below) is added to
 *    the solved attitude.
 * Per frame (FOptions::FrameSeconds): the sensor reads the true attitude
 * (plus Bias, plus noise uniform in +-NoiseDeg per axis from a FRandomStream
 * seeded with Seed, a frame late with bDelayOneFrame); the legs are measured;
 * FLaw::Step runs with Dt = FrameSeconds; its Dz is commanded when bWrite or
 * when a baseline changed (a baseline write always pushes baseline + the
 * offset last written); its speed scale is taken. An event takes effect on
 * the first frame at or after its time. A stalled leg's setpoint waits with
 * it, so on release the leg catches up within its own rate.
 *
 * Signs as RammsLevelTypes.h. A point p of the chassis frame is at
 *   world = Origin + Rz(Yaw) Ry(-Pitch) Rx(Roll) p
 * (right-handed rotations about z, y, x): the forward axis rises by sin Pitch
 * and the +Y axis by cos Pitch sin Roll, so pitch is positive nose up and
 * roll positive with the +Y side up. Unreal's FRotator rolls the other way:
 * do not build this with it.
 */
namespace RammsLevel::DryRun
{
	/** A ground: its height (cm) at world (X, Y) cm. Must be continuous. */
	using FGround = TFunction<double(double, double)>;
	using FGroundRef = TFunctionRef<double(double, double)>;

	// ---- made-up grounds ----

	/** A plane through the origin rising tan(PitchDeg) along +X and
	 *  tan(RollDeg) along +Y: a robot heading +X on it reads that pitch and
	 *  that roll (to first order). Plane(0, 0) is flat. */
	FGround Plane(double PitchDeg, double RollDeg);
	/** Straight lines between Knots (X, height; X rising) along +X, level
	 *  before the first and after the last, the same at every Y, with a cross
	 *  fall added that rises tan(CrossDeg) along +Y. Ramps up and down, crests
	 *  and sags are profiles. */
	FGround Profile(TConstArrayView<FVector2D> Knots, double CrossDeg = 0.0);
	/** A ridge along the X axis falling tan(Deg) to either side. */
	FGround Crown(double Deg);
	/** Flat up to a fold line through (X0, 0) turned FoldYawDeg off square to
	 *  +X, then a plane rising at Deg straight away from that line: wheels
	 *  cross it one at a time, and pitch and roll come on together. */
	FGround Fold(double Deg, double FoldYawDeg, double X0);
	/** A cross fall that changes sides: rising tan(Deg) along +Y up to X0,
	 *  falling as much from X0 + LengthCm on, and twisting in a straight line
	 *  (of the cross fall's angle) between the two. The same height along the X
	 *  axis everywhere: a robot driven along it is rolled from one side to the
	 *  other, wheel pair by wheel pair. */
	FGround Warp(double Deg, double X0, double LengthCm);

	// ---- the made-up robot ----

	/** One leg: one actuator carrying one wheel, or a pair across the robot. */
	struct FRobotLeg
	{
		/** The wheel centre (chassis xy, cm); for a pair, the middle of it. */
		FVector2D XY = FVector2D::ZeroVector;
		/** A ganged pair: its two wheels are at XY -+ (0, PairHalfTrackCm) and
		 *  move as one. 0: a single wheel at XY. */
		double PairHalfTrackCm = 0.0;
		double RadiusCm = 9.0;
		/** The wheel centre's z in the chassis frame in the stance the run
		 *  starts in: the first baseline. */
		double StanceZ = -30.0;
		/** Its travel (z, cm) and how fast it moves (cm/s, cm/s^2; 0: no limit). */
		double MinZ = -42.0;
		double MaxZ = -18.0;
		double MaxRateCmS = 12.0;
		double MaxAccelCmS2 = 60.0;
		/** Roll reaches it (FLegReach::bRoll). */
		bool bRoll = true;
		/** True travel per unit of the travel the law believes (see the plant). */
		double Gain = 1.0;
	};

	struct FRobot
	{
		TArray<FRobotLeg> Legs;
		/** The centre of mass (chassis frame, cm). */
		FVector CentreOfMass = FVector::ZeroVector;
		/** Underside points to keep clear, as the law is given them. */
		TArray<FClearancePoint> Points;
		/** It has a seat tilt (FInputs::bSeat). */
		bool bSeat = false;
	};

	/**
	 * The made-up six-wheeled robot of the curb tests: single wheels at
	 * (45, -+25), (0, -+27), (-45, -+25), in that order (front -Y, front +Y,
	 * centre -Y, centre +Y, rear -Y, rear +Y), every leg as FRobotLeg's
	 * defaults (9 cm wheels, stance z -30, travel -42..-18, 12 cm/s,
	 * 60 cm/s^2, roll on every leg). Centre of mass (-8, 0, 10): a little
	 * aft of the centre axle, as a rigid six-wheeler balanced exactly on its
	 * middle wheels tips from one end to the other at every crest. Four
	 * underside points at (+-40, +-20), z -24, keeping 3 cm. A seat.
	 * On level ground it stands with its origin 39 cm up.
	 */
	FRobot MakeSixWheelRobot();

	/**
	 * The made-up robot of the other kind: two independent drive legs and
	 * two ganged pairs. In this order:
	 *   0, 1  drive, single wheels at (0, -28) and (0, 28): radius 15, stance
	 *         z -15, travel -23..-7, 6 cm/s, 40 cm/s^2, roll reaches them;
	 *   2     rear pair at (-40, 0), half track 20: radius 6, stance z -24,
	 *         travel -32..-16, 6 cm/s, 40 cm/s^2, no roll;
	 *   3     front pair at (45, 0), half track 22: radius 6, stance z -20
	 *         (carried 4 cm clear of the ground), travel -30..-14, 6 cm/s,
	 *         40 cm/s^2, no roll.
	 * Centre of mass (-12, 0, 5). Underside points at (30, +-20) and
	 * (-30, +-20), z -22, keeping 3 cm. No seat. On level ground it stands on
	 * the drive wheels and the rear pair, its origin 30 cm up.
	 */
	FRobot MakeTwoDrivePairsRobot();

	// ---- the rest solve ----

	/** Where a robot comes to rest on a ground. */
	struct FRestPose
	{
		/** The chassis origin's height (cm, world) and the chassis' attitude
		 *  (deg; X pitch nose up, Y roll +Y up). */
		double	  Z = 0.0;
		FVector2D AttitudeDeg = FVector2D::ZeroVector;
		/** Each bottom's height over the ground (cm, 0 or more) and whether it
		 *  touches (within RestTouchCm), in the bottoms' order. */
		TArray<double, TInlineAllocator<8>> GapCm;
		TArray<bool, TInlineAllocator<8>>	bTouching;
		int32								Touching = 0;
		/** A rest was found: the bottoms that touch hold the centre of mass.
		 *  False when the robot would tip over (the pose is then the last
		 *  iterate) or the solve did not settle. */
		bool bStable = false;
	};

	/** A bottom this close to the ground (cm) touches it. */
	constexpr double RestTouchCm = 0.01;

	/**
	 * The pose a rigid robot rests in: the lowest its centre of mass can be
	 * with no wheel bottom under the ground -- the origin's height, pitch and
	 * roll that minimise the centre of mass's world height subject to every
	 * gap being 0 or more -- found from the warm start (the nearest such
	 * rest). The chassis origin's XY and the yaw are given and stay.
	 *
	 * Bottoms are the points (chassis frame, cm) where each wheel meets level
	 * ground: its centre less its radius in z. A bottom's gap is its world z
	 * less Ground at its world xy. (Treating the contact as that fixed point
	 * leaves the wheel's roundness out: a few hundredths of a centimetre at
	 * the slopes in use.) CentreOfMass is in the chassis frame.
	 *
	 * Out.bStable needs at least three touching bottoms that are not in line
	 * and whose polygon, seen from above, holds the centre of mass (within
	 * about 0.01 cm). A balance on two bottoms is not a rest here. The pose
	 * given back always has its lowest bottom on the ground and no bottom
	 * under it, stable or not.
	 * Warm, when given, is where the search starts (the last pose: the same
	 * rest is then kept from step to step, which is what makes a rocking
	 * stance repeat); without it, level at a height that clears the ground.
	 * Deterministic: the same arguments give the same answer bit for bit.
	 * Returns Out.bStable.
	 */
	bool SolveRest(TConstArrayView<FVector> Bottoms, FGroundRef Ground, const FVector2D& ChassisXY, double YawDeg,
		const FVector& CentreOfMass, const FRestPose* Warm, FRestPose& Out);

	// ---- the run ----

	/** What can be done to a run at a given time. */
	enum class EEvent : uint8
	{
		/** The switch on / off. */
		Enable,
		Disable,
		/** The commanded speed becomes Value (cm/s); 0 parks. */
		SetSpeed,
		/** The chassis is knocked: PlaneDeg (deg) is added to its true attitude
		 *  at once and dies away with time constant Seconds. */
		AttitudeKick,
		/** Leg stops following its command for Seconds (its joint holds). */
		StallLeg,
		/** Leg's Gain becomes Value (10 to 20 percent off 1 for a model error). */
		LegGain,
		/** For Seconds the law is told the support set Mask (bit i: leg i
		 *  stands; clearances 0) in place of working it out. */
		WrongSupport,
		/** A stance change as a drive mode's ramp makes it: every leg's
		 *  baseline (Leg = INDEX_NONE) or one leg's moves by Value (cm,
		 *  positive retracts) in a straight line over Seconds. Baseline writes
		 *  only; the offsets are the law's. */
		StanceRamp,
		/** A take-over: the law is told bYield with Mode Taken, and on its
		 *  Fold the offsets are folded into the baselines (baseline :=
		 *  baseline + offset, the offsets zero); the law writes nothing for
		 *  Seconds (a frame at least), while the taker may move the baselines
		 *  to a pose of its own (StanceRamp events in that time). Then the
		 *  hand-back, to the stance the run started in (each leg's StanceZ):
		 *  PlanAdoption takes the plane by which the pose differs from that
		 *  stance back out of the baselines, and the law is told bResume with
		 *  that plane and the offsets it left -- or, with bCold, told bResume
		 *  with no plane, the baselines left as the taker had them. With
		 *  RampSeconds over 0 a drive mode's stance ramp follows at once:
		 *  every leg's baseline goes in a straight line to its StanceZ over
		 *  that long, the law told the stance is changing meanwhile. Any Fold
		 *  the law asks for (an unexpected take-over too) is carried out the
		 *  same way. */
		FoldAdopt,
		/** For Seconds the attitude cannot be read. */
		AttitudeLoss,
		/** For Seconds the active mode is one to release for (EMode::Release). */
		ModeRelease,
		/** What "level" means becomes PlaneDeg (deg; it replaces
		 *  Tuning.LevelReferenceDeg from here on): the rider changing the tilt
		 *  they prefer. "Off level" in the result is measured against the
		 *  reference in force on each frame. */
		SetReference,
		/** For Seconds the law is told a drive mode's stance is being changed
		 *  (FInputs::bStanceChanging): a mode switch under way, whose stages
		 *  are StanceRamp events in that time. */
		StanceChanging,
		/** The rider's preferred tilt becomes PlaneDeg (deg; pitch, roll):
		 *  FOptions::PreferenceDeg from here on. */
		SetPreference
	};

	struct FDryEvent
	{
		double	  AtSeconds = 0.0;
		EEvent	  Kind = EEvent::Enable;
		int32	  Leg = INDEX_NONE;
		double	  Value = 0.0;
		double	  Seconds = 0.0;
		FVector2D PlaneDeg = FVector2D::ZeroVector;
		uint32	  Mask = 0;
		/** FoldAdopt: the hand-back gives no plane (levelling starts again
		 *  from nothing on the pose the taker left); and how long the stance
		 *  ramp that follows the hand-back takes (s; 0: none). */
		bool   bCold = false;
		double RampSeconds = 0.0;
	};

	struct FOptions
	{
		FTuning Tuning;
		/** The law's tick (s): 1/16, 1/30, 1/60. */
		double FrameSeconds = 1.0 / 16.0;
		/** The plant's step (s), and how often the pose is solved (s). */
		double PhysicsStepSeconds = 0.001;
		double PoseStepSeconds = 0.002;
		/** The chassis starts with its origin over here, heading YawDeg (deg,
		 *  anticlockwise from +X seen from above), at rest on the ground. */
		FVector2D StartXY = FVector2D::ZeroVector;
		double	  YawDeg = 0.0;
		/** The speed commanded from the start (cm/s) and the drive's lag (s). */
		double SpeedCmS = 60.0;
		double DriveLagSeconds = 0.15;
		/** The body's pitch under the drive's acceleration (deg per cm/s^2,
		 *  nose up speeding up; 0: none) and its lag (s). */
		double PitchPerAccelDeg = 0.0;
		double BodyLagSeconds = 0.1;
		/** How fast the law takes the ground to turn (FGroundGate's tuning). */
		FGroundTuning Ground;
		/** The joints' lag behind their setpoints (s) and their sag (cm, with
		 *  the load shared evenly; 0: none). */
		double ServoLagSeconds = 0.03;
		double SagCm = 0.0;
		/** The sensor: a fixed bias (deg), noise (deg, uniform either side,
		 *  per axis and frame), a frame of delay, and the noise's seed. */
		FVector2D BiasDeg = FVector2D::ZeroVector;
		double	  NoiseDeg = 0.0;
		bool	  bDelayOneFrame = false;
		int32	  Seed = 1;
		/** The seat's tilt, on a robot that has one: its servo's lag behind
		 *  its setpoint (s), how fast it turns (deg/s; 0: no limit), and how
		 *  far either way it can be trimmed from where the rider has it (deg). */
		double SeatLagSeconds = 0.03;
		double SeatMaxRateDegPerSec = 20.0;
		double SeatRoomDeg = 1.0e6;
		/** What the seat's cover goes by (FSeatTrim::Cover). */
		FSeatCoverTuning SeatCover;
		/** The rider's preferred tilt (deg; pitch, roll), over
		 *  Tuning.LevelReferenceDeg. Held as the level assist's component
		 *  holds it (SplitPreference): on a robot with a seat, with the
		 *  tuning's seat trim on, the pitch in the seat and the roll in the
		 *  base's reference; on any other, both in the base's reference. */
		FVector2D PreferenceDeg = FVector2D::ZeroVector;
		/** The switch is on from the first frame. */
		bool bEnabledAtStart = true;
		/** The run ends when the origin has travelled DistanceCm (0: no such
		 *  end) or after MaxSeconds, whichever is first. */
		double DistanceCm = 0.0;
		double MaxSeconds = 20.0;
		/** The steady figures are taken over this long at the end (s). */
		double SteadySeconds = 1.5;
		/** A wheel of a support leg counts as floating from this gap (cm). */
		double FloatGapCm = 0.2;
		/** The base counts as off level, for the lag, beyond this (deg). */
		double LagTiltDeg = 0.5;
		/** What happens on the way, in any order. */
		TArray<FDryEvent> Events;
		/** Keep every frame in FResult::Trace. */
		bool bKeepTrace = false;
	};

	/** One frame, when the trace is kept. */
	struct FSample
	{
		double Seconds = 0.0;
		/** The origin's travel (cm) and speed (cm/s). */
		double	  TravelCm = 0.0;
		double	  SpeedCmS = 0.0;
		FVector2D TrueAttitudeDeg = FVector2D::ZeroVector;
		/** The rider's true attitude (the chassis' with the seat's tilt added
		 *  to its pitch), the attitude the rider asked for (the reference with
		 *  the whole preference), and the seat's tilt (deg, nose up against
		 *  the chassis). */
		FVector2D RiderDeg = FVector2D::ZeroVector;
		FVector2D RiderReferenceDeg = FVector2D::ZeroVector;
		double	  SeatDeg = 0.0;
		/** The law's outputs of the frame. */
		FOutputs Out;
		/** Each leg's baseline and command (baseline + offset) (cm). */
		FLegValues BaselineZ;
		FLegValues CommandZ;
		/** Wheels of support legs floating; bottoms touching. */
		int32 Floating = 0;
		int32 Touching = 0;
	};

	/**
	 * What a run measured. "True" is the plant's attitude, not the sensor's;
	 * "off level" is Inclination(true - the base's reference): the tuning's
	 * LevelReferenceDeg with the part of the preference the base holds. The
	 * RIDER's figures are of the chassis' true attitude with the seat's tilt
	 * added to its pitch, against the reference with the whole preference; on
	 * a robot with no seat they are the base's. Rates and accelerations of
	 * attitudes are Euclidean over pitch and roll.
	 */
	struct FResult
	{
		/** It ran to its end with every rest solve stable. */
		bool   bCompleted = false;
		double Seconds = 0.0;
		double TravelCm = 0.0;
		/** Frames whose pose was not a stable rest. */
		int32 UnstableFrames = 0;

		/** Off level: the mean over the last SteadySeconds, the most over the
		 *  whole run, the last frame's; and the true attitude's peak-to-peak
		 *  over the last SteadySeconds (the larger axis): a limit cycle shows
		 *  here. */
		double SteadyResidualDeg = 0.0;
		double PeakTiltDeg = 0.0;
		double FinalTiltDeg = 0.0;
		double SteadyTiltRangeDeg = 0.0;
		/** The lag: the longest unbroken time (s) the base was more than
		 *  Options.LagTiltDeg off level, frame by frame. Through a grade break
		 *  that is how long the legs took to catch the ground up; on a course
		 *  that starts on a slope it is the walk to level. */
		double LongestOffLevelSeconds = 0.0;
		/** The RIDER's wanted attitude (FOutputs::RiderWantedDeg: W itself on
		 *  a robot with no seat): its largest rate (deg/s) and acceleration
		 *  (deg/s^2), frame to frame, over frames where the law was Active or
		 *  Releasing on both sides, not ramping out open loop (it is then the
		 *  measured attitude), and no action reset it between them. */
		double MaxWantedRateDegPerSec = 0.0;
		double MaxWantedAccelDegPerSec2 = 0.0;
		/** The true attitude: its largest rate over any 0.3 s (deg/s), and
		 *  the largest change of that rate between windows 0.3 s apart, per
		 *  second (deg/s^2). Kicks are in it. */
		double MaxTrueRateDegPerSec = 0.0;
		double MaxTrueAccelDegPerSec2 = 0.0;
		/** The rider: off what they asked for, the mean over the last
		 *  SteadySeconds, the most over the whole run and the last frame's
		 *  (deg); their attitude's largest rate over any 0.3 s (deg/s); and
		 *  the most the seat was tilted (deg). */
		double SteadyRiderResidualDeg = 0.0;
		double PeakRiderTiltDeg = 0.0;
		double FinalRiderTiltDeg = 0.0;
		double MaxRiderRateDegPerSec = 0.0;
		double MaxSeatDeg = 0.0;

		/** The largest change of any leg's command (baseline + offset, cm)
		 *  from one frame to the next; the same over the frames of a fold and
		 *  of an adopt and the one after each (0 with no FoldAdopt); and the
		 *  largest peak-to-peak of any leg's command over the last
		 *  SteadySeconds. */
		double MaxLegStepCm = 0.0;
		double MaxEventLegStepCm = 0.0;
		double SteadyLegRangeCm = 0.0;
		/** Frames in the last SteadySeconds on which the law wrote. */
		int32 SteadyWrites = 0;

		/** The least speed scale, the least speed once first over 2 cm/s
		 *  while a speed was commanded (cm/s), and the mean speed. */
		double MinSpeedScale = 1.0;
		double MinSpeedCmS = 0.0;
		double MeanSpeedCmS = 0.0;
		/** How long (s) any one wheel of a support leg floated at most (its
		 *  longest unbroken spell), and how long in all at least one did. */
		double LongestFloatSeconds = 0.0;
		double FloatingSeconds = 0.0;
		/** How long (s) the allocation was short of the plane (Fraction < 1),
		 *  and the least ReachLeft. */
		double SecondsAtReach = 0.0;
		double MinReachLeft = 1.0;
		/** The largest correction applied (deg, Euclidean) and offset (cm). */
		double MaxCorrectionDeg = 0.0;
		double MaxOffsetCm = 0.0;

		/** How it ended, and whether it was ever Faulted. */
		EState	FinalState = EState::Off;
		EStatus FinalStatus = EStatus::Off;
		bool	bEverFaulted = false;
		/** The largest offset left at the end (cm): exactly 0 after a release. */
		double FinalMaxOffsetCm = 0.0;

		TArray<FSample> Trace;
	};

	/** Run Robot over Ground. Deterministic: the same arguments give the same
	 *  result bit for bit. */
	FResult Run(const FRobot& Robot, FGroundRef Ground, const FOptions& Options);
} // namespace RammsLevel::DryRun
