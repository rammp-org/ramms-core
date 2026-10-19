// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Curb/RammsCurbLevel.h"
#include "Curb/RammsCurbPlanner.h"
#include "Curb/RammsCurbRobotModel.h"
#include "Curb/RammsCurbSchedule.h"
#include "Curb/RammsCurbSpeedLaw.h"
#include "RammsCurbNegotiationTypes.h"
#include "Templates/UniquePtr.h"

/**
 * The per-wheel gait's executor, as plain C++: a measurement in, leg and
 * drive commands out, every tick -- so that it can be dry-run on a made-up
 * robot (RammsCurb::ExecDryRun) as well as driven by the controller. Not
 * reflected, not exported.
 *
 * It runs a plan (RammsCurb::PlanCrossing) in the chassis travel S the robot
 * has measured along the plan's heading, each wheel on its own state machine:
 *
 *   Planted  on the surface it started on, or has landed on; carrying.
 *   Lifting  its lift has begun: when the plan has it begin (S reaches its
 *            SLiftStart), or sooner when the robot runs ahead of the plan and
 *            its window is no more than its lift's lead away -- and only when
 *            the wheels it must not be off the ground before are planted
 *            (MustBeDownMask), the set off the ground with it is one the plan
 *            has (and never a forbidden one), the robot as measured now, with
 *            that set removed, stands at least MinLiveStabilityMarginCm inside
 *            its wheels braking and speeding up, the centre wheels are within
 *            PermitToleranceCm of their planned fore/aft, and the base has been
 *            levelled in. Otherwise its lift is blocked, and says why. (The
 *            chassis height is logged against the plan, not a permit; the
 *            margin is the one measured now, not one predicted over the time
 *            the wheel will be up.) A lift still within AbortWithinCm of the
 *            ground when that margin falls under the floor is aborted: the
 *            wheel is Planted again.
 *   Clear    measured over the most its clearance profile asks over its
 *            window, less ClearToleranceCm: from here no braking cap holds it.
 *   Planting past its window (its measured distance past the edge), going
 *            down onto the far surface.
 *   Loading  touched down -- its bottom within TouchCm of the surface, or its
 *            leg stopped above where the servo lag model has it -- and taking
 *            load for LoadSeconds (the attitude loop's integral held). Its
 *            command joins the carrying legs' with their lead over their servo
 *            lag (the chassis moving), so it does not trail them.
 *   Held     off the ground in the stance the robot drove up in: carried on
 *            to its window if the plan holds it up from the start, else set
 *            down during the level-in, before the other legs move -- and loaded
 *            where it touched down, in that stance, not in the plan's pose.
 *
 * Legs are commanded a servo lag ahead along the plan, from the robot as it
 * is measured: a support leg puts its wheel's contact on the plane under its
 * own wheel (the near one until it has landed, the far one after), where the
 * wheel will be a servo lag from now, the chassis at the plan's height and
 * attitude, turned by the attitude loop; a lifted leg carries its wheel at
 * least as high as its clearance profile asks over its window, and as high as
 * it asks 0.3 s ahead, in the chassis' planned attitude or its measured one,
 * whichever needs more; each 5-bar walks to the plan's fore/aft. Every leg's
 * command is a trapezoid at LegSpeed and LegAcceleration towards its target.
 *
 * The speed: the plan's a little ahead (its own braking and speeding up are
 * in it), capped by each lift's deadline -- the speed at which the wheel
 * reaches its window just as the work in front of its lift is done (the
 * wheels its lift waits for to land and load, the centre wheels to walk into
 * place, the lift itself) -- and by a late level-in, a planned stop, and the
 * centre wheels or chassis lagging the plan; ramped at Acceleration and
 * Deceleration. Then the hard braking caps, the only terms that reach zero:
 * climbing, no wheel that is not yet clear goes into its window; descending,
 * no loaded wheel on the high side does -- each braked allowing for the
 * chassis running a drive lag behind its command, and braked at up to twice
 * Deceleration only when the command is more than 2 cm/s over its cap.
 *
 * A lift still on the ground is loaded: descending, it may come no nearer the
 * drop edge than the edge itself while its leg rises as the servo lag model
 * has it (unloading), and no nearer than its window once it has stalled there.
 *
 * The level-in takes the support legs from the drive stance to their level
 * targets along an S-curve in time whose rate and acceleration keep every leg
 * within LegSpeed / LegAcceleration and the base's turn within
 * TiltRateDegPerSec / TiltAccelDegPerSec2; the attitude loop and the level
 * check start once it is complete.
 *
 * A robot that stands behind the plan's origin (it rolled back after the plan
 * was made) reads the plan's speed from the origin on: it drives up to the
 * plan rather than resting at its first sample.
 *
 * A landing leg at its reach waits on the chassis, which the plan brings down
 * with travel; when the robot is held back meanwhile (a lift waiting on that
 * landing), the chassis comes down to the leg at once instead, within the
 * plan's corridor -- a wait that would otherwise never end. Conversely, a
 * wheel the plan has off the ground that still carries (its lift waiting on
 * a late landing) holds the chassis no higher than its leg reaches on the
 * ground: the plan's height there is for it up.
 *
 * Phases: Approach (the drive stance, legs held), Prepare (the level-in),
 * Cross (wheel events), Hold (braked to wait: a blocked lift, a wheel's
 * deadline or braking cap, the centre wheels lagging, the far side
 * unconfirmed -- not the plan's own speed, a planned stop or the level-in),
 * Settle (every wheel on the far side, rolling on until the rear tyres are
 * FinishClearance past the edge and the legs are where the plan ends them),
 * Done.
 */
namespace RammsCurb
{
	/** A hold lasting longer than this (s) faults the crossing. */
	constexpr double HoldTimeoutSeconds = 5.0;
	/** The lift's permits: the centre wheels within this (cm) of their planned
	 *  fore/aft. */
	constexpr double PermitToleranceCm = 1.5;
	/** A lift whose wheel is still within this (cm) of the ground is aborted
	 *  (the wheel Planted again) when the measured margin with it up falls
	 *  AbortHysteresisCm under MinLiveStabilityMarginCm. */
	constexpr double AbortWithinCm = 1.0;
	constexpr double AbortHysteresisCm = 0.25;
	/** Lifting is Clear this far (cm) under the most the profile asks. */
	constexpr double ClearToleranceCm = 0.3;
	/** A wheel's bottom this near (cm) its surface has touched down; its leg
	 *  stopped above where the servo lag model has it (or the model at its
	 *  target) counts only with its bottom within TouchdownToleranceCm. */
	constexpr double TouchCm = 0.3;
	constexpr double TouchdownToleranceCm = 1.5;
	/** A leg stopped this much (cm) above where the servo lag model has it,
	 *  going down, has met the ground. */
	constexpr double StallCm = 0.4;
	/** A leg going down slower than this (cm/s, measured, filtered over
	 *  StallFilterSeconds) has stopped: one merely slower than the lag model
	 *  is still on its way. */
	constexpr double StallSpeedCmS = 1.0;
	constexpr double StallFilterSeconds = 0.05;
	/** How far ahead (cm of travel) ReachGate looks for the plan's chassis to
	 *  come down to a leg at its reach. */
	constexpr double ReachGateCm = 40.0;
	/** A chassis brought down to a landing leg at its reach goes this much
	 *  (cm) further, so that the leg can press its wheel down. */
	constexpr double ReachDropExtraCm = 0.3;
	/** Once the leg has landed the chassis is let go back to the plan's height
	 *  no faster than this (cm/s). */
	constexpr double ReachReleaseCmS = 2.0;
	/** A swing leg looks this far ahead (s) along its clearance profile. */
	constexpr double SwingAheadSeconds = 0.3;
	/** The plan's speed is taken this far ahead (s, at the speed measured): the
	 *  chassis' lag behind its command. */
	constexpr double SpeedAheadSeconds = 0.15;
	/** A lift waiting on a landing keeps this much time in hand (s). The
	 *  planner's gap caps keep none (a lift begins as the wheel it waits on has
	 *  taken load), so a larger margin here held the robot under the plan's
	 *  speed at every hand-off; the lift's permit, not this, keeps the order. */
	constexpr double ExecGapMarginSeconds = 0.03;
	/** A rising wheel's deadline asks it to be no further than this (cm) under
	 *  its clearance profile (which keeps SwingClearance over the dilated lip):
	 *  more than the planner's own 0.25, so that a rise a little slower than
	 *  the planner timed it (the chassis sagging under its command) does not
	 *  hold the robot under the plan's speed; the hard braking cap, not this,
	 *  keeps the wheel off the curb. */
	constexpr double ExecProfileToleranceCm = 0.75;
	/** A rising wheel's hard cap stops it this far (cm) short of where its rim
	 *  would meet the terrain, which is marched for every this far (cm). */
	constexpr double TouchMarginCm = 0.5;
	constexpr double TouchStepCm = 0.5;
	/** A rising leg further than this (cm) from where the servo lag model has
	 *  it is not taken to rise as the model does (a lifted leg carries nothing,
	 *  so it follows the model closely): its cap is its window, as on the
	 *  ground. */
	constexpr double TrackingErrorCm = 0.5;
	/** The chassis this far (cm) off its commanded height is logged. */
	constexpr double ChassisOffCm = 1.5;
	/** A late level-in brakes the robot to rest this far (cm) short of the
	 *  first lift's start. */
	constexpr double LevelInStopShortCm = 1.0;

	/** The time it takes a loaded wheel to come off the ground once its lift
	 *  begins (s): a descent's deadline. A lift still on the ground after
	 *  UnloadStallFactor times this has stalled there, whatever the lag model
	 *  says. */
	constexpr double UnloadSeconds = 0.15;
	constexpr double UnloadStallFactor = 3.0;
	/** A landing wheel is set down this far (cm) into its surface (no further
	 *  than its preload), just to touch; Loading presses the rest of its
	 *  preload in over the load time. */
	constexpr double PlantProbeCm = 0.3;
	/** At touchdown a landing leg's command is moved on by the lead the
	 *  carrying legs' commands have over their servo lag models (the chassis'
	 *  motion through the lag), at most this far (cm) either way. */
	constexpr double TouchdownLeadMaxCm = 4.0;
	/** Emergency braking (twice Deceleration) only above a braking cap by more
	 *  than this (cm/s). */
	constexpr double EmergencyOverCmS = 2.0;
	/** The commanded speed under this (cm/s) is a hold (when no planned stop
	 *  asks for it), and over this plus 1 the hold is over. */
	constexpr double HoldSpeedCmS = 2.0;
	/** The chassis this much faster (cm/s) than its command -- the command as
	 *  the chassis can follow it (RunawayLagSeconds: the drive's lag and half
	 *  the speed's measuring window) -- or rolling back faster than
	 *  RollBackCmS, for this long (s), runs away: nothing on the ground drives
	 *  it (on a slope, with its driven wheels lifted). */
	constexpr double RunawayCmS = 15.0;
	constexpr double RollBackCmS = 5.0;
	constexpr double RunawaySeconds = 0.4;
	constexpr double RunawayLagSeconds = 0.3;
	/** A leg's command within this (cm) of its target has caught up. */
	constexpr double ExecCaughtUpCm = 0.05;
	/** v_track: the centre wheels or the chassis this far (cm) off the plan
	 *  begin to slow the robot, and it is at its least LagSlowCm further. */
	constexpr double LagFreeCm = 1.0;
	constexpr double LagSlowCm = 4.0;
	/** The least speed (cm/s) v_track slows to, as a fraction of the plan's. */
	constexpr double LagSlowFraction = 0.25;

	/** Where the executor is: the controller's phase. */
	enum class EExecPhase : uint8
	{
		/** Driving up in the drive stance, the legs held. */
		Approach,
		/** The level-in. */
		Prepare,
		Cross,
		/** Braked and waiting: a lift is blocked. */
		Hold,
		Settle,
		Done,
	};

	/** The term of the speed law that set the target speed. */
	enum class EExecTerm : uint8
	{
		/** The plan's own speed: its rise from rest, its braking into a planned
		 *  stop. */
		Plan,
		/** A wheel's deadline (FExecDiag::DeadlineLeg). */
		Deadline,
		/** The level-in the plan has the robot make (or a late one). */
		LevelIn,
		/** The centre wheels lagging their walk. */
		Track,
		/** Braking onto, or resting at, a planned stop. */
		PlannedStop,
		/** The cap from outside the plan (the commit point while the far side is
		 *  unconfirmed). */
		External,
	};

	const TCHAR* ExecPhaseName(EExecPhase Phase);
	const TCHAR* WheelStateName(ERammsCurbWheelState State);
	const TCHAR* ExecTermName(EExecTerm Term);

	/** One leg, as measured this tick. */
	struct FExecLeg
	{
		/** The wheel centre (world, cm). */
		FVector WheelWorld = FVector::ZeroVector;
		/** The wheel centre in the chassis frame (x-z, cm): a 4-bar's from its
		 *  crank, a 5-bar's from its hips. */
		FVector2D XZ = FVector2D::ZeroVector;
		/** Where the servo lag model has the leg's height (cm, chassis frame),
		 *  under the commands sent. */
		double LagZ = 0.0;
		bool   bValid = false;
	};

	/** The robot, as measured this tick. */
	struct FExecMeasure
	{
		/** Simulated seconds since the last tick. */
		double Dt = 0.0;
		/** The chassis: its origin over the ground (world xy), its origin's world
		 *  height, its horizontal heading (unit), its frame, its attitude (deg:
		 *  pitch nose up, roll +Y side up). */
		FVector2D	  OriginXY = FVector2D::ZeroVector;
		double		  OriginZ = 0.0;
		FVector2D	  Forward = FVector2D(1.0, 0.0);
		FChassisFrame Frame;
		FVector2D	  AttitudeDeg = FVector2D::ZeroVector;
		/** The chassis' speed along its heading (cm/s), and whether that spans
		 *  its whole window. */
		double			 SpeedCmS = 0.0;
		bool			 bSpeedValid = false;
		TArray<FExecLeg> Legs;
	};

	/** What to command this tick. */
	struct FExecCommand
	{
		/** Each leg's wheel-centre height (cm, chassis frame) and a 5-bar's
		 *  fore/aft (cm; a 4-bar's is its table's at that height). */
		TArray<double> LegZ;
		TArray<double> LegX;
		/** The drive speed (cm/s). */
		double DriveSpeed = 0.0;
	};

	/** One wheel's crossing, live. */
	struct FExecWheel
	{
		ERammsCurbWheelState State = ERammsCurbWheelState::Planted;
		/** The surface it stands on or is headed for: 0 near, 1 far. */
		int32 Plane = 0;
		/** Off the ground in the drive stance, set down in the level-in (rather
		 *  than held up to its window). */
		bool bLower = false;
		/** Measured this tick: its centre's distance past the edge (cm), and its
		 *  bottom over the surface it follows (cm). */
		double PastEdgeCm = 0.0;
		double BottomCm = 0.0;
		/** Its centre's distance past the edge once set down on the far plane
		 *  (cm): a corner's fore/aft moves with its leg's height, as the plan's
		 *  landing position has it. */
		double LandedPastCm = 0.0;
		/** Its leg's command: height (and its rate, the trapezoid) and, for a
		 *  5-bar, fore/aft (cm). */
		double CmdZ = 0.0;
		double CmdVz = 0.0;
		double CmdX = 0.0;
		/** Where the leg's command is headed this tick (cm, chassis frame). */
		double TargetZ = 0.0;
		/** The attitude loop's share (cm), while it supports; and the tilt of the
		 *  plane that best fits the support legs' preloads, taken out (cm): the
		 *  preloads share the load out among the legs, but do not tilt the base. */
		double AttitudeDz = 0.0;
		double PreloadTiltDz = 0.0;
		/** The most its clearance profile asks over its window (world, cm) along
		 *  the plan, and where (S) its drop to the landing begins. */
		double ZTop = 0.0;
		double SDrop = 0.0;
		/** Its lift is blocked, and why (logged once per reason). */
		bool	bBlocked = false;
		FString BlockedWhy;
		/** When its state last changed (s since Begin), and how long it has
		 *  been Loading (s). */
		double Since = 0.0;
		double Loading = 0.0;
		/** Its leg's command when the level-in's second stage began (cm), and
		 *  where it was sent at its last touchdown (cm). */
		double StanceZ = 0.0;
		double TouchZ = 0.0;

		/** Its leg's measured height last tick, and its rate (cm/s, filtered). */
		double MeasZ = 0.0;
		double MeasVz = 0.0;
		bool   bMeasZ = false;
		/** Its target last tick lay beyond its leg's reach (clamped to it), and by
		 *  how much (cm): a leg at its reach neither meets its target nor presses
		 *  its wheel down -- going down a step, it waits on the chassis to come
		 *  down to it. */
		bool   bReachClamped = false;
		double ReachDeficitCm = 0.0;
	};

	/** Why the speed is what it is, as of the last tick (cm/s; the largest
	 *  double for a cap that did not apply), and the wheel each cap is for. */
	struct FExecDiag
	{
		double Plan = 0.0;
		double Deadline = 0.0;
		int32  DeadlineLeg = INDEX_NONE;
		double Braking = 0.0;
		int32  BrakingLeg = INDEX_NONE;
		double LateLevel = 0.0;
		double Track = 0.0;
		double Target = 0.0;
		/** The term that set Target. */
		EExecTerm TargetBy = EExecTerm::Plan;
		double	  Command = 0.0;
		/** The chassis off its commanded height (cm, the steady offset taken out). */
		double ChassisOffCm = 0.0;
		/** The margin the plan has here, and the one measured (cm); the centre
		 *  wheels' fore/aft, planned and measured (cm; -Y side, +Y side). */
		double	  PlanMarginCm = 0.0;
		double	  LiveMarginCm = 0.0;
		FVector2D PlanX = FVector2D::ZeroVector;
		FVector2D LiveX = FVector2D::ZeroVector;
	};

	/** What the executor counted, for the run stats and the log. */
	struct FExecCounts
	{
		int32 Holds = 0;
		int32 PlannedStops = 0;
		/** Ticks on which a forbidden set was off the ground, or a loaded wheel
		 *  was inside its window (both never planned). */
		int32 ForbiddenTicks = 0;
		int32 WindowTicks = 0;
		/** The lifts that began on the plan's position and those that began
		 *  early on the time left; the lifts that were blocked at some point. */
		int32 LiftsOnPosition = 0;
		int32 LiftsOnTime = 0;
		int32 LiftsBlocked = 0;
		/** Lifts aborted while still near the ground (the margin fell). */
		int32 LiftsAborted = 0;
		/** Ticks a hard braking cap bound (the command at it); the legs (bit per
		 *  leg) whose deadline set the speed on some tick. */
		int32  BrakingTicks = 0;
		uint32 DeadlineLegs = 0;
	};

	class FExecutor
	{
	public:
		FExecutor();
		~FExecutor();

		/**
		 * Start running Plan, made from In, from the robot as Measure finds it.
		 * Each wheel off the ground there (In.Legs[i].bOffGround) is Held, the
		 * rest Planted on the near surface. False, with why, for a plan that is
		 * not valid or does not fit the input, or that inclines the base more
		 * than the input's MaxPlannedTiltDeg anywhere (its total tilt).
		 */
		bool Begin(const FPlannerInput& In, const FCurbSchedule& Plan, const FExecMeasure& Measure, FString& OutWhy);

		/** A new plan for the crossing under way -- allowed only before any wheel
		 *  has begun to cross (a re-plan on the approach), and held to
		 *  MaxPlannedTiltDeg as Begin holds one; the wheels keep their states
		 *  and the legs their commands. */
		bool Adopt(const FPlannerInput& In, const FCurbSchedule& Plan, FString& OutWhy);

		/** The crossing is over (done, faulted or aborted): IsRunning is false
		 *  until the next Begin. What it counted stays readable. */
		void End() { bRunning = false; }

		/** One tick: the wheels' states, the legs' commands, the speed. The
		 *  level state's attitude loop and plane biases are run here. */
		void Tick(const FExecMeasure& Measure, FLevelState& Level, FExecCommand& Out);

		/** The command a 5-bar actually took (the linkage refused part of a
		 *  move): the tracker carries on from there. */
		void SetCommanded(int32 Leg, double X, double Z);

		/** A further cap on the speed (cm/s) from outside the plan -- the commit
		 *  point while the far side is unconfirmed; the largest double for none. */
		void SetExternalCap(double CmS) { ExternalCap = CmS; }

		bool				 IsRunning() const { return bRunning; }
		EExecPhase			 GetPhase() const { return Phase; }
		const FString&		 GetStatus() const { return Status; }
		double				 GetS() const { return S; }
		double				 GetElapsed() const { return Elapsed; }
		const FExecCounts&	 GetCounts() const { return Counts; }
		const FExecDiag&	 GetDiag() const { return Diag; }
		const FCurbSchedule& GetPlan() const { return Plan; }
		const FPlannerInput& GetInput() const { return In; }
		/** The planned attitude where the robot is (deg; pitch, roll). */
		FVector2D GetPlannedAttitude() const { return PlannedAttitude; }
		/** The chassis origin's commanded world height (cm). */
		double					  GetBaseCommand() const { return BCmd; }
		const TArray<FExecWheel>& GetWheels() const { return Wheels; }
		/** The axle of the latest wheel event. */
		ERammsCurbAxle GetLatestAxle() const { return LatestAxle; }
		/** On the ground and carrying: Planted or Loading. */
		bool IsSupport(int32 Leg) const;
		/** The lines logged since the last call (moved out). */
		void TakeLog(TArray<FString>& Out);
		/** The level-in has begun (Prepare or later). */
		bool HasLevelInBegun() const { return bLevelInBegun; }
		/** Some wheel has left the ground for its crossing (or a held one has
		 *  been carried on): a re-plan is no longer possible. */
		bool HasCrossingBegun() const;

	private:
		/** The plan's values between its samples at travel SAt: chassis height,
		 *  attitude, centre fore/aft by side. Past a planned stop the robot leaves
		 *  from where it re-posed to. */
		struct FPlanAt
		{
			double B = 0.0;
			double Pitch = 0.0;
			double Roll = 0.0;
			double X[2] = { 0.0, 0.0 };
			double V = 0.0;
		};
		FPlanAt Between(double SAt) const;
		/** The plan's speed at travel SAt (cm/s), read no nearer its origin (or
		 *  the planned stop the robot last rested at, SResume) than its first
		 *  sample past it: a robot at or behind the origin of a plan made at rest
		 *  takes the speed the plan rises from rest at, not sample 0's rest. */
		double PlanSpeedAt(double SAt) const;
		/** A lifting wheel still on the ground whose leg no longer rises as the
		 *  servo lag model has it (or that has been on the ground for
		 *  UnloadStallFactor times UnloadSeconds): stalled there, loaded. */
		bool IsLiftStalledOnGround(int32 Leg, const FExecMeasure& Measure) const;
		/** Set up what each wheel's crossing needs from the plan (ZTop, SDrop). */
		void Prepare();
		void SetState(int32 Leg, ERammsCurbWheelState State, const FString& Why);
		void Log(const FString& Line);
		/** The leg's support target (chassis z, cm): its wheel's contact on its
		 *  plane where the wheel will be a servo lag from now. */
		double SupportTarget(int32 Leg, const FExecMeasure& Measure, const FPlanAt& Ahead, double Ahead_m, double Preload,
			const FVector2D& Bias) const;
		/** The leg's swing target (chassis z, cm): at least WorldZ in the planned
		 *  attitude and in the measured one. */
		double SwingTarget(int32 Leg, const FExecMeasure& Measure, const FPlanAt& Ahead, double WorldZ) const;
		/** The highest a lifted wheel must be carried (world, cm) now: ZTop until
		 *  its drop, then the profile 0.3 s ahead and the far surface's support. */
		double SwingWorldZ(int32 Leg, const FExecMeasure& Measure, double V) const;
		/** Clamped to the leg's reach at fore/aft X (cm, chassis frame). */
		double ClampToReach(int32 Leg, double X, double Z) const;
		/** Seconds (at the speeds the robot will run: the lesser of its own and
		 *  the plan's, sample by sample) until it has gone from where it is to
		 *  travel SAt. */
		double SecondsTo(double SAt) const;
		/** Whether leg Leg may lift now, and if not why. */
		bool LiftPermit(int32 Leg, const FExecMeasure& Measure, const FLevelState& Level, FString& OutWhy) const;
		/** The measured margin with Without (bit per leg) off the ground too. */
		double MarginWithout(const FExecMeasure& Measure, uint32 Without) const;
		/** The deadline cap (cm/s) of each lift still ahead; the largest double
		 *  with none. Names the wheel that binds. */
		double DeadlineCap(const FExecMeasure& Measure, int32& OutLeg) const;
		/** A planted centre wheel's walk before its window (cm along the
		 *  heading; positive aft, which gives it that much more room than
		 *  RoomCm, the room it has where it stands): the plan's fore/aft for it
		 *  where it must be clear less its own now -- aft, only as far as it can
		 *  walk (CenterShiftSpeed) while the chassis covers RoomCm at the speed
		 *  it has, and none while the centre wheels hold their fore/aft for a
		 *  late landing. 0 for any other wheel. */
		double WalkCredit(int32 Leg, const FExecMeasure& Measure, double RoomCm) const;
		/** The highest the chassis origin may be commanded (world, cm) for
		 *  support leg Leg to still reach its wheel's ground with its preload,
		 *  LegRangeMargin kept, in the attitude Ahead; the largest double when
		 *  its reach is unknown. */
		double SupportCeiling(int32 Leg, const FExecMeasure& Measure, const FPlanAt& Ahead, double AheadCm, const FVector2D& Bias) const;
		/** What is left of wheel Leg's landing: the distance (cm, along the
		 *  heading) until it may be set down, then the seconds to set it down
		 *  and load it. */
		void LandingWork(int32 Leg, const FExecMeasure& Measure, double& OutGate, double& OutSeconds) const;
		/** A landing wheel whose leg is at its reach: how far (cm of travel)
		 *  until the plan's chassis has come down by what its target lies beyond
		 *  that reach; 0 when the plan does not bring it down that far within
		 *  ReachGateCm. */
		double ReachGate(int32 Leg) const;
		/** How far (cm) the chassis may come down below the plan's height here,
		 *  every leg still reaching (the plan's corridor, LegRangeMargin kept);
		 *  and the most a landing leg is short of its target at its reach (cm,
		 *  as of the last tick). */
		double ReachRoom() const;
		double ReachDrop() const;
		/** How far (cm, along the heading) a rising wheel may go before its rim,
		 *  at the height it is now, meets the terrain (not dilated); the largest
		 *  double when it clears everything to its landing. */
		double TouchDistance(int32 Leg, const FExecMeasure& Measure) const;
		/** The hard braking caps (cm/s): each wheel that may not go into its
		 *  window (on the ground), or meet the terrain (rising). */
		double BrakingCap(const FExecMeasure& Measure, int32& OutLeg) const;
		/** A rising wheel's deadline (cm/s): the fastest the chassis may go for
		 *  its leg, rising as its command and the servo lag take it (on the chassis
		 *  as the plan raises it), to stay over its clearance profile -- the
		 *  plan's, less ProfileToleranceCm -- all the way to where it may land;
		 *  the largest double when it does at the speed commanded. */
		double LiftDeadline(int32 Leg, const FExecMeasure& Measure) const;
		/** A rising wheel's centre (world, cm) once the chassis has gone Ahead cm
		 *  on at the speed it has: its command rising towards where it is headed
		 *  (a trapezoid, as high as its leg reaches on the chassis as the plan
		 *  raises it), the leg a servo lag behind. Steps on from the state in
		 *  Sim (start with Sim.bStarted false). */
		struct FRise
		{
			bool   bStarted = false;
			double Time = 0.0;
			double C = 0.0;
			double Cv = 0.0;
			double H = 0.0;
		};
		double PredictRise(int32 Leg, const FExecMeasure& Measure, double Ahead, FRise& Sim) const;
		/** A landing wheel's target (chassis z, cm): its contact PressCm into the
		 *  far surface from the chassis as measured (moved on a servo lag along
		 *  the plan's height) -- it carries nothing, so the height the chassis
		 *  rides at on the others' preloads does not move it. */
		double PlantingTarget(int32 Leg, const FExecMeasure& Measure, double AheadCm, const FVector2D& Bias, double PressCm) const;

		FPlannerInput		   In;
		TUniquePtr<FPlanModel> M;
		FCurbSchedule		   Plan;
		TArray<FExecWheel>	   Wheels;
		FExecCounts			   Counts;
		FExecDiag			   Diag;
		TArray<FString>		   Pending;
		bool				   bRunning = false;
		EExecPhase			   Phase = EExecPhase::Approach;
		FString				   Status;
		double				   Elapsed = 0.0;
		/** Travel along the plan's heading from its origin (cm), and the speed
		 *  the robot is taken to have (cm/s). */
		double S = 0.0;
		double VHat = 0.0;
		double VCmd = 0.0;
		/** The commanded chassis origin height (world, cm); that command through
		 *  the servo lag model; and how far the chassis stood off it when the
		 *  level-in was done (cm: the preloads and the servos' sag). */
		double	  BCmd = 0.0;
		double	  BCmdLag = 0.0;
		double	  BOffset = 0.0;
		FVector2D PlannedAttitude = FVector2D::ZeroVector;
		bool	  bLevelInBegun = false;
		/** The level-in's first stage (the held corners set down) is over; and
		 *  when its second began (s since Begin): the support legs go from the
		 *  stance to their level targets along an S-curve in time (SCurveShare of
		 *  the whole move, at RampRate / RampAccel of it per second), over
		 *  RampSpan, so that no leg moves faster than LegSpeed / LegAcceleration
		 *  and neither the base's pitch nor its roll turns faster than
		 *  TiltRateDegPerSec / TiltAccelDegPerSec2. */
		bool   bLoweredDown = false;
		double LevelRampFrom = -1.0;
		double RampRate = 1.0;
		double RampAccel = 1.0;
		double RampSpan = 0.0;
		/** Where the crossing is done (S, cm): every wheel loaded on the far side,
		 *  the rear tyres FinishClearance past the edge, the centre wheels home. */
		double SEnd = 0.0;
		/** The next planned stop (sample), and whether the robot is resting at it
		 *  (and since when). */
		int32  NextStop = INDEX_NONE;
		bool   bDwelling = false;
		double DwellFor = 0.0;
		/** The plan's speed is read no nearer than this (S, cm): its origin, or
		 *  the last planned stop the robot rested at -- the plan's sample there is
		 *  at rest, and the robot leaves from it at the speed the plan rises at. */
		double SResume = 0.0;
		/** A hold under way. */
		bool bHolding = false;
		/** The centre wheels holding their fore/aft for a wheel landing late. */
		bool bCentresHeld = false;
		/** The chassis held under the plan's height for a wheel the plan has off
		 *  the ground that still carries, at its leg's reach (SupportCeiling). */
		bool bSupportCeiling = false;
		/** The chassis brought down below the plan to a landing leg at its reach. */
		bool   bReachLowering = false;
		double ReachLowerCm = 0.0;
		double ExternalCap = TNumericLimits<double>::Max();
		/** What last held the robot under the plan's speed (logged when it
		 *  changes), and when. */
		FString		   SlowBy;
		double		   SlowAt = -1.0;
		ERammsCurbAxle LatestAxle = ERammsCurbAxle::Front;
		/** The 5-bar legs by side (-Y [0], +Y [1]). */
		int32 Centre[2] = { INDEX_NONE, INDEX_NONE };
	};

	/** What the closed-loop dry run of the executor found. */
	struct FExecDryRunResult
	{
		bool   bDone = false;
		double Seconds = 0.0;
		/** The least commanded speed once risen from rest, and the stops (2 cm/s
		 *  or less for 0.2 s) after first motion. */
		double		MinSpeedCmS = 0.0;
		int32		Stops = 0;
		FExecCounts Counts;
		/** The least gap between a lifted wheel's rim and the analytic terrain
		 *  inside its window (cm); ticks with a wheel on the ground (as
		 *  simulated) inside its window, or into the terrain. */
		double MinSwingClearanceCm = TNumericLimits<double>::Max();
		int32  KeepOffViolations = 0;
		int32  IntoTerrain = 0;
		/** Lifts begun before a wheel they wait on (MustBeDownMask) had taken
		 *  load. */
		int32 EarlyLifts = 0;
		/** The least measured dynamic margin over the run (cm). */
		double MinMarginCm = TNumericLimits<double>::Max();
		/** The largest base tilt (deg) the support legs' errors give. */
		double MaxTiltDeg = 0.0;
		/** The fastest the base turned (deg/s, either axis, over 0.3 s) while
		 *  levelling in (Prepare). */
		double MaxLevelInTurnDegS = 0.0;
		/** Per wheel: the furthest past the edge (cm) it was while it carried
		 *  load on the near surface (the lowest -- largest double for never). */
		TArray<double> MaxNearCarryPastEdgeCm;
		/** Where each wheel ended (its distance past the edge, cm), and the
		 *  chassis' speed then (cm/s). */
		TArray<double> FinalPastEdgeCm;
		double		   FinalSpeedCmS = 0.0;
		/** The lines worth reading (blocked lifts, holds), and every line the
		 *  executor logged. */
		TArray<FString> Notes;
		TArray<FString> Log;
		/** One line every TraceEvery ticks (FExecDryRunOptions): time, travel,
		 *  speeds and caps, each wheel's state, distance past the edge and leg
		 *  error, the base's attitude. */
		TArray<FString> Trace;
		EExecPhase		EndPhase = EExecPhase::Approach;
	};

	/** How the dry run disturbs the robot. */
	struct FExecDryRunOptions
	{
		double Dt = 1.0 / 120.0;
		/** The legs lag their commands by ServoLagSeconds times this. */
		double LagScale = 1.0;
		/** The chassis follows its drive command with this lag (s). */
		double DriveLag = 0.15;
		/** Leg StallLeg's lift stops this far up (cm) from where it began. */
		int32  StallLeg = INDEX_NONE;
		double StallAtCm = 0.0;
		/** The legs (bit per leg) whose plant -- going down onto the far surface --
		 *  lags SlowPlantScale times as much as the rest. */
		uint32 SlowPlantLegs = 0;
		double SlowPlantScale = 1.0;
		/** A trace line every this many ticks; 0 for none. */
		int32  TraceEvery = 0;
		double MaxSeconds = 60.0;
		/** The robot begins this far (cm) along the plan's heading from where the
		 *  plan was made (negative: behind it, as a robot that rolled back). */
		double StartTravelCm = 0.0;
	};

	/**
	 * The executor closed round a kinematic robot: legs following their
	 * commands with the servo lag, the chassis following the drive command
	 * with DriveLag, its height and attitude from the support legs (a rigid
	 * base fitted to them by least squares), lifted wheels kept out of the
	 * analytic terrain. Checks every tick that no wheel on the ground is in its
	 * window (a support wheel, or one lifting that still carries load), lifted
	 * wheels clear the terrain inside it, and counts what the executor counts.
	 */
	FExecDryRunResult ExecDryRun(const FPlannerInput& In, const FCurbSchedule& Plan, const FExecDryRunOptions& Options = FExecDryRunOptions());
} // namespace RammsCurb
