// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsDriveMotion.h"

class URammsRobotBaseComponent;

/**
 * Changing drive mode on the move, as plain C++ so that each piece can be
 * tested on made-up numbers. Not reflected, not exported.
 *
 * URammsDriveModeSelector measures how the chassis moves (FChassisMotionWindow)
 * and hands the switch itself to FModeSwitcher, which talks to the robot only
 * through ISwitchHost: it asks the incoming mode what of the motion it can
 * carry, has the outgoing mode bring the rest to zero and waits for the
 * measurement to show it (FHandOverGate), ramps the stance (FStanceChannel) and
 * hands over once the incoming mode's wheels are down. The modes carry the
 * motion they are handed across: the holonomic one from it to its operator's
 * command on a rate-limited twist (RampMotion), the differential one as a pull
 * towards its wheel speeds (DifferentialWheelSpeeds) while its operator holds
 * the stick (FCarry), faded out into its own command (CarryBlend).
 */
namespace RammsDrive
{
	/** 0 at or below 0, 1 at or above 1, and an S between: 3x^2 - 2x^3, whose
	 *  slope is 0 at both ends and 1.5 at its steepest. */
	double SmoothStep(double X);

	/** A chassis moving no faster than this (cm/s, forward and sideways each)
	 *  and turning no faster than AtRestYawDegS is at rest: a switch then
	 *  hands over at once with nothing to carry. */
	constexpr double AtRestSpeedCmS = 2.0;
	constexpr double AtRestYawDegS = 3.0;
	bool			 IsAtRest(const FRammsDriveMotion& Motion);

	/**
	 * The chassis' motion from its pose over a short window: the velocity of
	 * its centre and its rate of turn, each the least-squares slope through
	 * every pose in the window, the velocity along and across the heading at
	 * the window's mean time. Poses
	 * come once a tick; a pose older than the window is dropped, and one whose
	 * time goes back (a reset) starts the window again.
	 */
	class FChassisMotionWindow
	{
	public:
		explicit FChassisMotionWindow(double InWindowSeconds = 0.15, double InMinSpanSeconds = 0.04);

		/** A pose: the chassis centre (world cm) and its forward direction (any
		 *  length but 0), at Time (s). */
		void Add(double Time, const FVector2D& Centre, const FVector2D& Forward);

		void Reset();

		/** True once the poses held span at least the least span. */
		bool IsValid() const;

		/** The motion over the window; zero when not valid. */
		FRammsDriveMotion GetMotion() const;

		/** The time of the newest pose, or a negative number with none. */
		double GetLatestTime() const;

	private:
		struct FPose
		{
			double	  Time = 0.0;
			FVector2D Centre = FVector2D::ZeroVector;
			/** Heading (rad), unwrapped against the pose before it. */
			double Yaw = 0.0;
		};
		TArray<FPose> Poses;
		double		  WindowSeconds;
		double		  MinSpanSeconds;
	};

	/** What the selector holds a hand-over to. */
	struct FHandOverLimits
	{
		/** The sideways speed (cm/s) the incoming mode will take over. */
		double MaxSidewaysCmS = 2.0;
		/** The yaw rate (deg/s) past what the incoming mode can turn it will
		 *  take over. */
		double MaxYawExcessDegS = 3.0;
		/** How long (s) the measurement must stay within both. */
		double SustainSeconds = 0.1;
		/** How long (s) the outgoing mode has to get there before the switch is
		 *  refused. */
		double TimeoutSeconds = 3.0;
	};

	/** How far a measured motion is from what a mode can carry: X the sideways
	 *  speed (cm/s), Y the yaw rate (deg/s), each as a magnitude. */
	FVector2D MotionExcess(const FRammsDriveMotion& Measured, const FRammsDriveMotion& Carriable);

	/** True when the excess is past either limit: the outgoing mode has to take
	 *  it off before the incoming one can take the robot over. */
	bool NeedsHandOver(const FRammsDriveMotion& Measured, const FRammsDriveMotion& Carriable, const FHandOverLimits& Limits);

	/**
	 * True when the incoming mode cannot carry the measured motion (past the
	 * limits) and the outgoing one can: a motion the outgoing mode drives, which
	 * it can take off first. A motion neither carries -- a slide on a slope under
	 * a mode that cannot move sideways either -- is not the outgoing mode's
	 * doing, and asking it to take it off would only wait out the timeout.
	 */
	bool NeedsHandOverBetween(const FRammsDriveMotion& Measured, const FRammsDriveMotion& OutgoingCarriable,
		const FRammsDriveMotion& IncomingCarriable, const FHandOverLimits& Limits);

	enum class EHandOverGate : uint8
	{
		/** Not yet within the limits long enough. */
		Waiting,
		/** Within them for SustainSeconds: hand over. */
		Ready,
		/** TimeoutSeconds went by first: refuse. */
		TimedOut,
	};

	/** The wait for the outgoing mode to take off what the incoming one cannot
	 *  carry, tick by tick on measured motion. A tick with no measurement
	 *  (bMeasured false: the window restarting after a reset, say) is not
	 *  within the limits: the gate never opens on no data. */
	struct FHandOverGate
	{
		double Elapsed = 0.0;
		double Within = 0.0;

		void		  Reset();
		EHandOverGate Step(const FRammsDriveMotion& Measured, const FRammsDriveMotion& Carriable, const FHandOverLimits& Limits,
			double DeltaSeconds, bool bMeasured = true);
	};

	/** One actuator a stance change moves: a linkage axis (cm) or a motor
	 *  (its own units, radians for a hinge), from where it is to where the
	 *  stance wants it. */
	struct FStanceChannel
	{
		FName  Id;
		bool   bMotor = false;
		double From = 0.0;
		double To = 0.0;
		/** A linkage endpoint's height (the main wheel's, up positive), rather
		 *  than its fore/aft position. */
		bool bHeight = false;
	};

	/**
	 * A stance change in two stages, wheels down before wheels up: where the
	 * linkage endpoints (the main wheels) go down -- their heights falling, on
	 * the whole -- the linkage axes first and the motors (the corner legs)
	 * after; where they go up, the motors first and the linkage axes after.
	 * With no linkage height moving, all of it in the first stage.
	 */
	void SplitStanceStages(const TArray<FStanceChannel>& Channels, TArray<FStanceChannel>& OutFirst, TArray<FStanceChannel>& OutSecond);

	/**
	 * How long a stance change takes: at least MinSeconds, and long enough
	 * that on the S-curve (SmoothStep) no linkage axis moves faster than
	 * MaxLinkageSpeed (cm/s) and no motor faster than MaxMotorRate (units/s)
	 * at the curve's steepest. A limit at or below 0 does not apply; 0 when
	 * none does or nothing moves -- the stance is then taken at once, a jump.
	 */
	double StanceRampSeconds(const TArray<FStanceChannel>& Channels, double MinSeconds, double MaxLinkageSpeed, double MaxMotorRate);

	/** Where a channel is Elapsed seconds into a change of Duration seconds:
	 *  every channel on the same S-curve, so they all arrive together. */
	double StanceRampValue(const FStanceChannel& Channel, double Elapsed, double Duration);

	/**
	 * From moved towards To by at most LinearAccel * DeltaSeconds of
	 * velocity (forward and right together, along the straight way between
	 * them, cm/s^2) and YawAccel * DeltaSeconds of yaw rate (deg/s^2). A
	 * limit at or below 0 does not apply.
	 */
	FRammsDriveMotion RampMotion(const FRammsDriveMotion& From, const FRammsDriveMotion& To, double LinearAccel, double YawAccel,
		double DeltaSeconds);

	/** True when A and B are within Tolerance of each other in every term
	 *  (cm/s, and deg/s for the yaw rate). */
	bool MotionsEqual(const FRammsDriveMotion& A, const FRammsDriveMotion& B, double Tolerance = 1e-3);

	/** What a differential drive can carry of a motion: its forward speed, no
	 *  sideways speed, its yaw rate within MaxYawDegS (at or below 0: any). */
	FRammsDriveMotion DifferentialCarriable(const FRammsDriveMotion& Motion, double MaxYawDegS);

	/** A differential drive's left (X) and right (Y) wheel ground speeds
	 *  (cm/s) for a motion, wheels TrackWidthCm apart: the outer wheel of a
	 *  right turn is the left one. */
	FVector2D DifferentialWheelSpeeds(const FRammsDriveMotion& Motion, double TrackWidthCm);

	/** The weight of the operator's command, 0..1, SinceHoldEnded seconds into
	 *  a carried motion's blend of BlendSeconds (at or below 0: at once). */
	double CarryBlend(double SinceHoldEnded, double BlendSeconds);

	/** A holonomic drive's normalised command (X forward, Y right, Z yaw,
	 *  each -1..1) for a motion, at full stick MaxLinearCmS and MaxYawDegS. */
	FVector HolonomicCommand(const FRammsDriveMotion& Motion, double MaxLinearCmS, double MaxYawDegS);

	/** The motion a holonomic drive's normalised command asks for. */
	FRammsDriveMotion HolonomicMotion(const FVector& Command, double MaxLinearCmS, double MaxYawDegS);

	/**
	 * A motion handed to a mode as it takes over, and how far through
	 * carrying it the mode is: held while its operator keeps commanding what
	 * they were at the hand-over, for the hold handed with it, then given up to
	 * the operator's command over a blend; the hold ends early when they stop
	 * (the caller says which: bOperatorCommanding).
	 */
	struct FCarry
	{
		bool			  bActive = false;
		bool			  bHolding = false;
		FRammsDriveMotion Motion;
		double			  HoldLeft = 0.0;
		/** Seconds since the hold ended (the blend's clock). */
		double SinceHold = 0.0;

		void Begin(const FRammsDriveMotion& InMotion, double HoldSeconds, bool bOperatorCommanding);
		/** Advance by DeltaSeconds; the hold ends early once the operator stops
		 *  commanding motion. */
		void Step(double DeltaSeconds, bool bOperatorCommanding);
		void End();
	};

	// --- holding still ------------------------------------------------------------
	//
	// With no command from its operator a drive mode holds the robot where it
	// comes to rest, on a slope too, rather than letting its wheels roll free:
	// each wheel it drives is held at the angle it stopped at. A wheel under a
	// speed loop (the robot base's) is asked a rate back towards that angle
	// (FWheelHold); a torque-driven one is given the torque itself, by a stiff
	// loop on its angle whose target first brakes the wheel to rest
	// (FTorqueHold).

	/** How long (simulated seconds) after a drive mode's first tick it holds
	 *  nothing: the robot is being put down as it spawns and the selector
	 *  jumps its legs into the initial stance -- a leg carrying its
	 *  wheel fore/aft -- and a wheel held at its angle meanwhile would drag
	 *  the chassis back (and, one side before the other, turn it) instead of
	 *  rolling with its leg. */
	constexpr double SpawnSettleSeconds = 1.0;

	/** The development switch ramms.Drive.HoldWhenIdle (1 by default): 0 turns
	 *  every drive mode's hold off, for A/B runs. */
	bool HoldWhenIdleEnabled();

	/** The development switch ramms.Drive.HoldBrakes (1 by default): 0 holds
	 *  with torque alone where the backend has brakes too, for A/B runs. */
	bool HoldBrakesEnabled();

	/** A drive mode holding the robot still closes its wheels' brakes once the
	 *  robot moves slower than this (cm/s), and opens them again for its own
	 *  hold to take the robot back if they let it move faster than
	 *  BrakeSlipCmS: the differential drive's HoldBrakeSpeedCmS and
	 *  HoldBrakeSlipCmS. */
	constexpr double BrakeAtRestCmS = 2.0;
	constexpr double BrakeSlipCmS = 4.0;

	/**
	 * Close the parking brake (URammsRobotBaseComponent::SetMotorBrake) of each
	 * of Motors that has one, unless ramms.Drive.HoldBrakes is 0, each added
	 * to InOutBraked; its motor applies nothing beneath it. A braked wheel is
	 * held by its brake alone: it must not be commanded a speed as well (the
	 * backend would open the brake for it). True if InOutBraked holds any.
	 */
	bool EngageBrakes(URammsRobotBaseComponent& Base, TConstArrayView<FName> Motors, TArray<FName>& InOutBraked);

	/** Open every brake in InOutBraked, and empty it. */
	void ReleaseBrakes(URammsRobotBaseComponent& Base, TArray<FName>& InOutBraked);

	/**
	 * A drive mode's clock from one tick to the next: OutDt, the seconds since
	 * the last tick (0 on the first); InOutFirst, the first tick's time (from
	 * which SpawnSettleSeconds runs); InOutLast, this tick's. Returns true when
	 * the clock went back -- the simulation was reset (the physics' clock
	 * restarted, the robot back at its start pose, every wheel's angle with
	 * it): nothing held before may stand, and the first second without a hold
	 * starts again from T, as at the spawn.
	 */
	bool AdvanceHoldClock(double T, double& InOutFirst, double& InOutLast, double& OutDt);

	/**
	 * A wheel's angle for holding it still: the backend's reading while it
	 * follows the wheel's rate, else the rate integrated. A reading that does
	 * not move while the rate turns the wheel through CheckRad (a backend that
	 * reports no angle for a continuous wheel reads 0 throughout) is not an
	 * angle: a hold on it would brake the wheel towards a target the reading
	 * never reaches and drive it on instead. Decided once, the first time the
	 * rate has turned the wheel through CheckRad.
	 */
	struct FWheelAngle
	{
		/** The rate's travel (rad) over which the reading is checked, and the
		 *  least share of it the reading must have moved. */
		static constexpr double CheckRad = 0.2;
		static constexpr double FollowShare = 0.25;

		bool   bStarted = false;
		bool   bDecided = false;
		bool   bReadingFollows = true;
		double Integrated = 0.0;
		double ReadingAtStart = 0.0;
		double Offset = 0.0;
		double Value = 0.0;

		/** The angle now (rad), from the reading and the rate (rad/s) over
		 *  DeltaSeconds since the last update. */
		double Update(double Reading, double Rate, double DeltaSeconds);
		/** Start again (the simulation reset). */
		void Reset() { *this = FWheelAngle(); }
	};

	/** The torque (N.m) on each of Wheels drive wheels of RadiusCm that holds
	 *  a robot of MassKg on ground rising GradeDeg ahead -- positive drives it
	 *  forward, uphill; a holding loop begun with it as its integral takes the
	 *  slope's pull at once rather than finding it as the robot sags back. */
	double GroundHoldTorque(double MassKg, double GradeDeg, double RadiusCm, int32 Wheels);

	/** A wheel under a speed loop held at an angle: the speed loop's own
	 *  integral carries the load (a slope's pull), this brings the wheel back
	 *  to where it was latched. */
	struct FWheelHold
	{
		/** The rate asked per rad off the angle held (1/s), and the most asked
		 *  either way (rad/s). */
		static constexpr double Gain = 4.0;
		static constexpr double MaxRate = 1.0;

		bool   bHeld = false;
		double Anchor = 0.0;

		/** Hold the wheel at Angle (rad), its angle now. */
		void Latch(double Angle)
		{
			bHeld = true;
			Anchor = Angle;
		}
		void Release() { bHeld = false; }
		/** The rate (rad/s) to ask the wheel's speed loop for, at Angle: back
		 *  towards the angle held, Gain per rad, at most MaxRate; 0 when not
		 *  held. */
		double Rate(double Angle) const;
	};

	/** How a torque-driven wheel is held: a loop on its angle, stiff enough to
	 *  stop a robot on a slope within a centimetre or two and settle in about
	 *  a second, braking it to rest first at no more than Decel. */
	struct FTorqueHoldGains
	{
		/** N.m per rad, N.m per rad/s, N.m per rad-second. */
		double Kp = 0.0;
		double Kd = 0.0;
		double Ki = 0.0;
		/** The most torque (N.m) either way. */
		double MaxTorque = 0.0;
		/** How fast (rad/s^2) the target brakes from the wheel's rate to rest. */
		double Decel = 0.0;
		/** The wheel's share of the robot (kg m^2): the target's braking is fed
		 *  forward as this times its deceleration. */
		double Inertia = 0.0;
	};

	/** Gains for a wheel carrying InertiaKgM2 of the robot (its share of the
	 *  mass times the radius squared) with a motor of MaxTorqueNm: a loop of
	 *  about 6 rad/s, damped 0.8, its integral over half a second; the target
	 *  braking at DecelCmS2 over a wheel of RadiusCm. */
	FTorqueHoldGains TorqueHoldGains(double InertiaKgM2, double MaxTorqueNm, double RadiusCm, double DecelCmS2);

	/**
	 * A torque-driven wheel held still: Begin from its angle and rate now;
	 * the target then brakes to rest at the gains' Decel -- twice that while
	 * the wheel runs on ahead of it, or as fast as the wheel slows if that is
	 * faster (CatchUpRadS) -- and stays there, and
	 * Step gives the torque that holds the wheel on it (proportional, rate and
	 * integral, the integral frozen while the torque is at its limit, and the
	 * target's braking fed forward through the wheel's inertia). The
	 * torque Step gave last is kept, for a release that fades it out rather
	 * than dropping it.
	 */
	struct FTorqueHold
	{
		/** A wheel the loop's own torque has driven this much (rad/s) off its
		 *  target -- off it the way the last torque pushed -- FreeSpinTicks
		 *  ticks running is not one the loop holds: it carries none of the
		 *  robot (in the air), against whose share of the robot the gains were
		 *  set, and the loop, closed once a tick, spins it up (tens of rad/s
		 *  within a second). The hold lets go of it (no torque) until it turns
		 *  slower than RearmRadS, then holds it again where it is. A loaded
		 *  wheel stays well inside it -- one the slope runs off against the
		 *  torque is held on, and one tick's reading of a jolted wheel (over
		 *  15 rad/s, seen once) is not enough to let go of it. */
		static constexpr double FreeSpinRadS = 8.0;
		static constexpr int32	FreeSpinTicks = 3;
		static constexpr double RearmRadS = 0.5;
		/** While braking, the most (rad/s) the target may run ahead of a wheel
		 *  slowing faster than it: beyond, the target is brought back to the
		 *  wheel (its angle, and its rate plus this). */
		static constexpr double CatchUpRadS = 0.5;
		/** While braking, a wheel running more than CatchUpRadS ahead of its
		 *  target (a slope pulling it on) has the target brake this many times
		 *  as hard as the gains' Decel. */
		static constexpr double PulledOnDecelScale = 2.0;
		/** A hold begun eased (bEaseToRest) has its target's braking eased
		 *  off as it comes to rest: once its rate is within this long (s) of
		 *  rest at its deceleration, it slows in proportion to what is left
		 *  (a first-order approach), never below BrakeTaperFloor of the
		 *  deceleration. A target stopped in one step leaves a loop closed
		 *  once a game tick to carry the wheel past and back (the tests'
		 *  made-up wheel, on a 62.5 ms tick: about 2 cm/s of roll-back after
		 *  a stop from speed). */
		static constexpr double BrakeTaperSeconds = 0.3;
		static constexpr double BrakeTaperFloor = 0.15;

		bool bActive = false;
		/** Let go of: turning far off its target (FreeSpinRadS), and the ticks
		 *  running it has been so. */
		bool   bFree = false;
		int32  OffTicks = 0;
		double Target = 0.0;
		double TargetRate = 0.0;
		double Integral = 0.0;
		double LastTorque = 0.0;
		/** For the next Step only: the braking decided for the robot rather
		 *  than from this wheel alone (StepTorqueHoldPair) -- the target not
		 *  caught up to this wheel, and braking DecelScale times the gains'
		 *  Decel. */
		bool   bPairDecided = false;
		double DecelScale = 1.0;
		/** Ease the target to rest (BrakeTaperSeconds) -- except in a braking
		 *  a slope has pulled the wheel on ahead of its target in, where easing
		 *  off would only let it run further (bPulledThisBraking: held for the
		 *  rest of the braking, so a wheel read noisily either side of its
		 *  target does not ease the braking off and on). */
		bool bEaseToRest = false;
		bool bPulledThisBraking = false;

		/** Hold from Angle (rad), braking from Rate (rad/s); Integral (N.m) is
		 *  what the loop starts holding with -- a slope's pull already known
		 *  (GroundHoldTorque), or what a hold just ended held -- 0 to find it;
		 *  bEase: the target eased to rest (bEaseToRest). */
		void Begin(double Angle, double Rate, double InitialIntegral = 0.0, bool bEase = false);
		void End() { bActive = false; }
		/** The torque (N.m) this tick, with the wheel at Angle (rad) turning
		 *  at Rate (rad/s), DeltaSeconds since the last; 0 when not active. */
		double Step(double Angle, double Rate, double DeltaSeconds, const FTorqueHoldGains& Gains);
		/** At rest on its target: braking done. */
		bool IsBraked() const { return bActive && TargetRate == 0.0; }
	};

	/**
	 * Two torque-held wheels of one robot, one each side (a differential
	 * drive): if their errors (target less angle, rad, each wheel in its own
	 * motor's sense, both turning the same way when the robot drives
	 * straight) differ by more than MaxDifferenceRad, move their targets
	 * towards each other's -- half the excess each, so what they hold the
	 * robot at along its way is unchanged -- until they differ by that much:
	 * the hold then never turns the robot back by more than that, whatever
	 * the wheels did since it began (one slipped while braking, a leg dragged
	 * one). Nothing unless both are held and neither is let go of.
	 */
	void BoundTurnError(FTorqueHold& Left, double LeftAngle, FTorqueHold& Right, double RightAngle, double MaxDifferenceRad);

	/**
	 * One tick of two torque-held wheels of one robot, one each side, braking
	 * as one: whether the robot slows faster than its targets (their targets
	 * then follow it, CatchUpRadS) or runs on ahead of them (a slope pulling
	 * it on: PulledOnDecelScale) is decided on the two wheels' mean rate
	 * against their targets' mean, and both targets are moved alike -- each
	 * wheel deciding for itself on its own reading (up to a rad/s off on a
	 * loaded wheel for a tick) would brake the two sides differently and turn
	 * the robot. Then BoundTurnError, then each wheel's Step; the torques
	 * (N.m) out.
	 */
	void StepTorqueHoldPair(FTorqueHold& Left, double LeftAngle, double LeftRate, FTorqueHold& Right, double RightAngle, double RightRate,
		double DeltaSeconds, const FTorqueHoldGains& LeftGains, const FTorqueHoldGains& RightGains, double MaxTurnDifferenceRad,
		double& OutLeftTorque, double& OutRightTorque);

	/**
	 * The least time (s) a stance change on the selector's S-curve
	 * (SmoothStep, whose slope is 1.5 at its steepest and whose second
	 * derivative is 6 at its ends) must take so that a turn of the base of
	 * TurnDeg goes no faster than RateDegS nor changes its rate faster than
	 * AccelDegS2; 0 for no turn or no limits.
	 */
	double StanceTurnSeconds(double TurnDeg, double RateDegS, double AccelDegS2);

	// --- the switch itself -------------------------------------------------------

	/** The chassis' measured motion, or none (the window not yet full, or a
	 *  robot whose chassis cannot be located). */
	struct FMotionSample
	{
		bool			  bValid = false;
		FRammsDriveMotion Motion;

		FMotionSample() = default;
		FMotionSample(bool bInValid, const FRammsDriveMotion& InMotion)
			: bValid(bInValid)
			, Motion(InMotion)
		{
		}
		/** Measured, and not at rest. */
		bool IsMoving() const { return bValid && !IsAtRest(Motion); }
	};

	/** Where a switch is. */
	enum class ESwitchPhase : uint8
	{
		/** No switch: nothing to do. */
		Idle,
		/** The outgoing mode is taking off motion the incoming one cannot
		 *  carry; waiting for the measurement to show it gone. */
		Constraining,
		/** The stance's first stage (the incoming mode's wheels going down),
		 *  the outgoing mode still driving. */
		OutgoingStage,
		/** The active mode in charge; the rest of the stance change. */
		IncomingStage,
	};

	/** The switch's limits and rates (the selector's tuning). */
	struct FSwitchConfig
	{
		FHandOverLimits Limits;
		/** The least time of each stance stage (s), and the fastest a linkage
		 *  axis (cm/s) and a motor (units/s) may move at the S-curve's
		 *  steepest; all 0: the stance at once. */
		double MinStageSeconds = 1.0;
		double MaxLinkageSpeed = 15.0;
		double MaxMotorRate = 1.5;
	};

	/**
	 * What FModeSwitcher needs of the robot, modes named by index: the
	 * selector's drive modes, or a made-up robot in a test.
	 */
	class ISwitchHost
	{
	public:
		virtual ~ISwitchHost() = default;

		virtual bool IsValidMode(int32 Mode) const = 0;
		virtual bool IsModeActive(int32 Mode) const = 0;
		/** Whether the mode has a drive stance of its own. */
		virtual bool			  HasStance(int32 Mode) const = 0;
		virtual FRammsDriveMotion GetCarriableMotion(int32 Mode, const FRammsDriveMotion& Motion) const = 0;
		/** IRammsDriveMode::BeginHandOver on Mode, bounded by Other; false when
		 *  it cannot. */
		virtual bool BeginHandOver(int32 Mode, int32 Other) = 0;
		virtual void EndHandOver(int32 Mode) = 0;
		/** IRammsDriveMode::HoldForStance. */
		virtual void HoldForStance(int32 Mode, bool bHold) = 0;
		/** IRammsDriveMode::HoldWheelsWhileInactive, on a mode just stood down
		 *  whose wheels still carry the robot. The default does nothing. */
		virtual void HoldOutgoingWheels(int32 Mode, bool bHold) {}
		/** Stand every other mode down and activate Mode; with Seed, hand it
		 *  that motion to carry (HoldSeconds the stance change left). Returns a
		 *  line on what it carried, for the status. */
		virtual FString ActivateMode(int32 Mode, const FRammsDriveMotion* Seed, double HoldSeconds) = 0;
		/** The actuators Mode's stance moves, each from where it is; false when
		 *  the mode has no stance (or stances are not applied). */
		virtual bool GetStanceChannels(int32 Mode, TArray<FStanceChannel>& OutChannels) = 0;
		/** Command a stage's channels Elapsed seconds into its Duration. */
		virtual void CommandStance(const TArray<FStanceChannel>& Channels, double Elapsed, double Duration) = 0;
		/** Command Mode's stance outright (a ramp's end, or a jump). */
		virtual void	ApplyStance(int32 Mode) = 0;
		virtual FString ModeName(int32 Mode) const = 0;
		/** A line for the log. */
		virtual void Report(const FString& Line, bool bWarning) {}
	};

	/**
	 * A drive-mode switch, step by step (URammsDriveModeSelector's, as plain
	 * C++). Request asks for one; Tick advances it.
	 *
	 *  - On the move, when the incoming mode cannot carry what the outgoing one
	 *    drives (NeedsHandOverBetween), or when the stance has a first stage to
	 *    put the incoming mode's wheels down: the outgoing mode stays in charge
	 *    under BeginHandOver(Incoming). Constraining waits (FHandOverGate) for
	 *    the measured motion to come within what the incoming mode carries,
	 *    refusing at the timeout; then the first stage ramps, pausing while the
	 *    motion is past the limits again and refusing once it has paused for
	 *    the timeout; at its end the incoming mode is activated, handed the
	 *    measured motion, and held to what the outgoing one carries
	 *    (BeginHandOver(Outgoing)) through the second stage -- the outgoing
	 *    mode's wheels are still coming up.
	 *  - Otherwise -- at rest, not measurable, or an outgoing mode that cannot
	 *    keep driving -- the incoming mode is active at once (handed the motion
	 *    if moving), its stance planned against what activating it made
	 *    available, held to what the outgoing mode carries through the whole
	 *    change, and, while a two-stage change's first stage puts its own
	 *    wheels down, held still (HoldForStance) -- the outgoing mode, stood
	 *    down, keeping its wheels held where they are meanwhile
	 *    (HoldOutgoingWheels): they still carry the robot, which on a slope
	 *    would otherwise roll.
	 *  - A refusal or a cancel brings the stance back to the active mode's from
	 *    wherever the change had it, the mode held to what the abandoned one
	 *    carries until it is.
	 */
	class FModeSwitcher
	{
	public:
		/** A mode in place without a switch (the initial one). */
		void Reset(int32 ActiveMode);

		/** Switch to Wanted (an index). False only for no mode. */
		bool Request(ISwitchHost& Host, int32 Wanted, const FMotionSample& Measured, const FSwitchConfig& Config);
		void Tick(ISwitchHost& Host, double DeltaSeconds, const FMotionSample& Measured, const FSwitchConfig& Config);

		ESwitchPhase GetPhase() const { return Phase; }
		int32		 GetActive() const { return Active; }
		int32		 GetPending() const { return Pending; }
		/** The active mode is under a BeginHandOver of this switch's. */
		bool IsActiveHeld() const { return bActiveHeld; }
		/** The active mode is held still for its wheels (HoldForStance). */
		bool IsAwaitingWheels() const { return bAwaiting; }
		/** The mode stood down whose wheels are held meanwhile
		 *  (HoldOutgoingWheels), or INDEX_NONE. */
		int32 GetHeldOutgoing() const { return HeldOutgoing; }
		/** The stage being ramped (2: none). */
		int32		   GetStage() const { return Stage; }
		double		   GetStageSeconds(int32 S) const { return (S == 0 || S == 1) ? StageSeconds[S] : 0.0; }
		const FString& GetStatus() const { return Status; }

	private:
		bool PlanStance(ISwitchHost& Host, int32 Mode, bool bTwoStages, const FSwitchConfig& Config);
		/** Ramp the current stage on; true once the last is done. */
		bool StepStance(ISwitchHost& Host, double DeltaSeconds);
		void HandOver(ISwitchHost& Host, const FMotionSample& Measured);
		void Finish(ISwitchHost& Host);
		void Cancel(ISwitchHost& Host, const FString& Why, bool bRestore, const FSwitchConfig& Config);
		/** End this switch's hold and wait on the active mode, if any, and the
		 *  hold on the outgoing mode's wheels. */
		void ReleaseActive(ISwitchHost& Host);
		/** End the hold on the outgoing mode's wheels, if any. */
		void ReleaseOutgoing(ISwitchHost& Host);
		void ResetStages();

		ESwitchPhase  Phase = ESwitchPhase::Idle;
		int32		  Active = INDEX_NONE;
		int32		  Pending = INDEX_NONE;
		int32		  StanceMode = INDEX_NONE;
		bool		  bActiveHeld = false;
		bool		  bAwaiting = false;
		int32		  HeldOutgoing = INDEX_NONE;
		FHandOverGate Gate;
		/** How long the first stage has waited for the motion (s). */
		double				   Paused = 0.0;
		double				   SinceAsked = 0.0;
		TArray<FStanceChannel> Stages[2];
		double				   StageSeconds[2] = { 0.0, 0.0 };
		int32				   Stage = 2;
		double				   Elapsed = 0.0;
		FString				   Status;
	};
} // namespace RammsDrive
