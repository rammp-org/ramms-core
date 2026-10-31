// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Curb/RammsCurbStick.h"
#include "RammsCurbNegotiationTypes.h"

/**
 * The curb controller's speed law and the leg and wheel rates it drives with,
 * as plain C++ so that each can be tested on made-up numbers. Not reflected,
 * not exported.
 *
 * Speed is a ramp towards the drive speed, capped by a deadline -- the speed
 * at which the next wheel to be lifted reaches the point it must be clear by
 * just as the legs finish the work in front of it -- and clamped by the hard
 * braking caps, the only terms that ever ask for a standstill. Nothing in it
 * is a step factor: every cap moves continuously with the distance and the
 * leg work left.
 */
namespace RammsCurb
{
	/** A deadline keeps this much distance in hand (cm). */
	constexpr float DeadlineMarginCm = 1.0f;
	/** Time added to every deadline for what the leg model misses (s). */
	constexpr float DeadlineMarginSeconds = 0.1f;
	/** The least time a deadline divides by (s): leg work all but done. */
	constexpr float MinDeadlineSeconds = 0.05f;

	/** How far behind its drive command the chassis runs (s), taken as first
	 *  order: the wheels' speed loops, and the chassis riding on its wheels.
	 *  A stop the robot must come to rest at -- the commit point, the line a
	 *  late level-in holds it short of -- is braked towards this much of the
	 *  wheels' speed short of it (LagBrakingDistance). A chassis that lags
	 *  more comes to rest a little past the stop, by the difference times its
	 *  speed when the command reaches 0; one that lags less, short of it, and
	 *  creeps on. */
	constexpr float DriveLagSeconds = 0.15f;

	/** The stall check: a commanded speed above this (cm/s) ... */
	constexpr float StallCommandCmS = 5.0f;
	/** ... with the chassis measured below this (cm/s). */
	constexpr float StallMeasuredCmS = 1.0f;

	/** How long the carrier fit remembers (s): a tick that long ago weighs
	 *  1/e of the latest. */
	constexpr float CarrierFitMemorySeconds = 1.0f;
	/** The carrier fit's ridge ((cm/s)^2 s): the leg motion a gain needs
	 *  behind it before the fit trusts it, rather than 0. */
	constexpr float CarrierFitRidge = 4.0f;

	/** Braking-limited speed with Distance (cm) left to a hard stop. */
	float BrakingSpeed(float DistanceCm, float Deceleration);

	/**
	 * The distance (cm) to brake the command towards, for a stop DistanceCm
	 * ahead that the chassis must come to rest at, when the chassis follows
	 * its drive command as a first-order lag of LagSeconds: DistanceCm less
	 * the LagSeconds * SpeedCmS (its speed now, never taken below 0) it runs
	 * on past wherever the command stops. Braked on this at Deceleration, the
	 * distance shrinks at the command's own speed, so the command falls at
	 * Deceleration and the chassis comes to rest on the stop. Braked on the
	 * raw distance, the chassis -- running ahead of its command -- uses up the
	 * braking curve faster than the command slows: the command collapses to 0
	 * over the last few cm, and the chassis still runs LagSeconds of its
	 * speed past the stop.
	 */
	float LagBrakingDistance(float DistanceCm, float SpeedCmS, float LagSeconds);

	/**
	 * The drive speed's cap (cm/s) while a plan made on the move is pending,
	 * the robot rolling on in its drive stance: braked, allowing for the
	 * chassis' lag (LagBrakingDistance, DriveLagSeconds), short of the line
	 * its level-in must be done by -- LevelInCm (along the heading) short of
	 * the nearest wheel's window, ToWindowCm ahead, where a level-in begun
	 * standing still ends -- and so of every window. A long plan would
	 * otherwise roll the robot on into the LEVEL window in its stance; braked
	 * to rest here, it levels in standing once the plan comes.
	 */
	float AwaitingPlanCap(float ToWindowCm, float LevelInCm, float SpeedCmS, float Deceleration);

	/**
	 * The drive command one tick on while a plan made on the move is pending:
	 * from Current, slowed (at Deceleration, never faster) towards the speed
	 * that would cover RoomCm -- from here to the line the level-in must be
	 * done by -- in LevelInSeconds, the level-in's time from the stance the
	 * robot rolls in (AwaitingLevelInSeconds). AwaitingPlanCap alone keeps the
	 * speed until the line's braking curve, so a plan that comes late finds
	 * the robot at the line with no room left to level in on the move, and it
	 * stops to level in; slowed so from the ask on, it keeps the level-in's
	 * room in hand at a speed that does it -- slower, not stopped. Never
	 * faster than Current.
	 */
	float AwaitingLevelInStep(float Current, float RoomCm, float LevelInSeconds, float Deceleration, float Dt);

	/**
	 * The level-in's time (s) from a drive stance inclined TurnDeg from level:
	 * the base turned to level on an S-curve within RateDegS and AccelDegS2,
	 * the legs' servo lag, a spare 0.3 s (as the planner times it), and
	 * CornersDownSeconds first where the stance holds corners up to set down.
	 */
	float AwaitingLevelInSeconds(float TurnDeg, float RateDegS, float AccelDegS2, float LagSeconds, float CornersDownSeconds);

	/** Holding the robot still in a crossing: the chassis' drive speed back
	 *  towards where it was held from (OffCm along the heading past it), at
	 *  RestHoldGainPerSecond, at most RestHoldMaxCmS either way -- a creep:
	 *  a robot knocked off its place (a corner set down on a slope) is brought
	 *  back slowly, not driven back at speed. */
	constexpr double RestHoldGainPerSecond = 6.0;
	constexpr double RestHoldMaxCmS = 3.0;

	/**
	 * The drive speed (cm/s) that holds the chassis still: RestHoldGainPerSecond
	 * of how far (cm, along the heading) it has moved from where it was held,
	 * back, at most RestHoldMaxCmS. A speed of 0 alone leaves only the wheels'
	 * speed loops: on a slope the robot creeps until they wind up, and as the
	 * legs move the base (setting corners down, levelling in) its chassis moves
	 * over the wheels; held so, the wheels roll to keep it where it was.
	 */
	double RestHoldSpeed(double OffCm);
	/** The level-in's first stage, the corners the stance holds up set down
	 *  (s): what AwaitingLevelInSeconds adds for them. */
	constexpr float LevelInCornersDownSeconds = 0.6f;

	/**
	 * The deadline speed: held for Seconds (at least MinDeadlineSeconds), it
	 * leaves the chassis able to brake at Deceleration to a stop within
	 * RoomCm -- v * T + v^2 / (2 * Deceleration) = RoomCm. So the work ends
	 * with the chassis on, not past, the braking curve of the hard stop it was
	 * racing, and that cap never has to act; with no time left it is that
	 * braking curve. 0 only with no room.
	 */
	float DeadlineSpeed(float RoomCm, float Seconds, float Deceleration);

	/**
	 * The ramp: Current moved towards Target by at most Acceleration * Dt
	 * when speeding up and Deceleration * Dt when slowing, never below 0.
	 */
	float RampSpeed(float Current, float Target, float Acceleration, float Deceleration, float Dt);

	/**
	 * One tick of the drive speed (cm/s), as the controller runs it from its
	 * tuning: Current ramped towards Tuning.DriveSpeed * StepFraction, no
	 * faster than DeadlineCmS, at Tuning.Acceleration going up and
	 * Tuning.Deceleration coming down; then clamped by the braking cap of
	 * each hard stop BrakingDistancesCm ahead, shaped at Tuning.Deceleration.
	 * The braking caps are the only terms that reach 0.
	 */
	float NextDriveSpeed(const FRammsCurbTuning& Tuning, float Current, float StepFraction, float DeadlineCmS,
		TConstArrayView<float> BrakingDistancesCm, float Dt);

	/** One wheel as a crossing finds it when it begins. */
	struct FEntryWheel
	{
		/** A main drive wheel (FRammsCurbLeg::Drive), on the ground. */
		bool bMain = false;
		bool bOnGround = false;
		/** Its joint rate (rad/s) in the sense that rolls it forward. */
		float JointRate = 0.0f;
		float RadiusCm = 0.0f;
		/** The forward (chassis heading) part of its rolling direction. */
		float RollForward = 1.0f;
	};

	/**
	 * The speed a crossing starts at (cm/s): the mean speed along the heading
	 * of the main drive wheels on the ground (joint rate * radius over the
	 * forward part of the rolling direction, wheels rolling mostly across the
	 * heading left out); with none, the chassis' measured speed when that is
	 * valid; else 0. Never below 0. A crossing asked for on the move carries on
	 * at the speed the robot has, rather than from a standstill.
	 */
	float EntrySpeed(TConstArrayView<FEntryWheel> Wheels, bool bChassisSpeedValid, float ChassisSpeedCmS);

	/**
	 * The turn rate (rad/s) of the link a driven wheel hangs from, over a tick
	 * of Dt: the wheel's world turn about its axle over the tick (WorldTurn,
	 * rad) less its joint's, each in the sense that rolls the wheel forward.
	 *
	 * The joint's turn must be the one the backend's wheel speed loop holds.
	 * Where the backend reports the joint's angle, it is that angle's change
	 * from LastAngle to Angle (unwound): exact. An angle of exactly 0 on both
	 * ticks is a backend that reports none (URammsRobotBaseComponent::
	 * GetMotorValue's "0 if unknown"), and then it is the reported joint rate
	 * averaged over the tick (LastRate, Rate). So on a backend whose wheel
	 * rate is the wheel's own spin in the world -- whose speed loop already
	 * turns the wheel over the ground at the commanded rate, whatever its link
	 * does -- the link's turn comes out near 0, as it should, rather than as
	 * the wheel's whole spin. 0 for a tick of no length.
	 */
	float CarrierRate(double WorldTurn, float LastAngle, float Angle, float LastRate, float Rate, float Dt);

	/**
	 * Seconds until a leg move is done: its command, RemainingCm short of its
	 * goal, arrives at RateCmS, and the leg -- a first-order follower with
	 * time constant LagSeconds, ErrorCm off its command now -- comes within
	 * ToleranceCm of it. A command still moving trails it by RateCmS *
	 * LagSeconds (or by ErrorCm, when the leg lags more than the model).
	 */
	float LegMoveSeconds(float RemainingCm, float RateCmS, float ErrorCm, float LagSeconds, float ToleranceCm);

	/**
	 * Seconds until a leg at MeasuredCm, following a command at CommandCm that
	 * ramps at RateCmS (first order, LagSeconds), reaches TargetCm, going up.
	 * 0 when it is there already. The command is taken to go on past the
	 * target, as a lift's does.
	 */
	float LegReachSeconds(float MeasuredCm, float CommandCm, float RateCmS, float TargetCm, float LagSeconds);

	/**
	 * The work before a lift, as the deadline sees it: time-gated stretches
	 * (leg moves that end a step) and distance gates (a step that ends when the
	 * chassis has driven far enough), in order.
	 */
	class FDeadlineChain
	{
	public:
		/** Leg work that takes Seconds, whatever the speed. */
		void AddTime(float Seconds);

		/** A step that ends once the chassis has driven DistanceCm from where it
		 *  is now. */
		void AddGate(float DistanceCm);

		/**
		 * The fastest speed now (cm/s) at which the chassis, RoomCm short of
		 * the hard stop the lifting wheel must be clear by, gets there no
		 * sooner than the work is done: the DeadlineSpeed of the room over all
		 * the time-gated work, and behind each gate the DeadlineSpeed of what
		 * is left of the room over the work after it, braked back to now at
		 * Deceleration. DeadlineMarginCm and DeadlineMarginSeconds are kept in
		 * hand; 0 only once the room is used up.
		 */
		float Speed(float RoomCm, float Deceleration) const;

		float TotalSeconds() const { return Total; }

	private:
		struct FGate
		{
			float DistanceCm = 0.0f;
			/** The time-gated work after this gate. */
			float SecondsAfter = 0.0f;
		};
		TArray<FGate> Gates;
		float		  Total = 0.0f;
	};

	/**
	 * Watches for a stall: the speed commanded above StallCommandCmS while the
	 * chassis measures below StallMeasuredCmS.
	 */
	class FStallWatch
	{
	public:
		void Reset() { Seconds = 0.0f; }

		/** One tick of Dt: true once the stall has lasted LimitSeconds (and
		 *  LimitSeconds > 0). A measurement that is not valid yet neither adds
		 *  to the stall nor ends it. */
		bool Tick(float CommandedCmS, float MeasuredCmS, bool bMeasuredValid, float Dt, float LimitSeconds);

		/** How long the current stall has lasted (s). */
		float GetSeconds() const { return Seconds; }

	private:
		float Seconds = 0.0f;
	};

	/** What FRestWatch::Tick found. */
	enum class ERest : uint8
	{
		/** Not held at the stop, or held there and still moving. */
		Moving,
		/** Held at the stop, and measured at rest. */
		Rested,
		/** Held at the stop for the time allowed without coming to rest. */
		TimedOut,
	};

	/**
	 * Watches a robot held at a stop it has braked to -- within reach of the
	 * stop, its braking cap bringing the command to 0 -- for coming to rest
	 * there. What a decision taken at the stop waits for: the chassis runs a
	 * lag behind its command, so a command at 0 does not yet mean a robot at
	 * rest.
	 */
	class FRestWatch
	{
	public:
		void Reset() { Seconds = 0.0f; }

		/** One tick of Dt: Rested once held (bHeld) with the measured speed,
		 *  either way, under StoppedCmS and the measurement valid; TimedOut
		 *  once held LimitSeconds (> 0) without that; else Moving. Not held
		 *  starts the count over. */
		ERest Tick(bool bHeld, float MeasuredCmS, bool bMeasuredValid, float Dt, float StoppedCmS, float LimitSeconds);

		/** How long the robot has been held at the stop (s). */
		float GetSeconds() const { return Seconds; }

	private:
		float Seconds = 0.0f;
	};

	/**
	 * The turn rate (rad/s) of the link a wheel hangs from, as a linear
	 * function of its leg's fore/aft and vertical rates (cm/s), fitted by least
	 * squares to what was measured, forgetting over CarrierFitMemorySeconds.
	 * A 5-bar's distal link turns as the leg walks its wheel fore/aft and as it
	 * raises or lowers it, by amounts that change only slowly with the pose.
	 */
	class FCarrierFit
	{
	public:
		void Reset() { *this = FCarrierFit(); }

		/** One tick of Dt: the leg's measured rates and the carrier's. */
		void Add(float XRate, float ZRate, float CarrierRate, float Dt);

		/** The carrier rate the fit puts on these leg rates; 0 for any rate the
		 *  leg has not yet moved at. */
		float Predict(float XRate, float ZRate) const;

		/** The fitted gains (rad/cm): per fore/aft and per vertical cm. */
		FVector2D GetGains() const;

	private:
		double Sxx = 0.0, Sxz = 0.0, Szz = 0.0, Sxc = 0.0, Szc = 0.0;
	};

	/**
	 * One driven wheel's feed-forward: how fast its leg carries it fore/aft,
	 * and how fast the link it hangs from turns it, over the tick to come.
	 *
	 * Each is what was measured over the tick just gone, plus the change the
	 * servo lag model predicts from that tick to the next under the commands
	 * just sent -- for the carrier, through FCarrierFit. Measured alone, each
	 * would be a tick or more late, and a 5-bar starting or ending a walk turns
	 * its wheel's link faster than the wheel's speed loop can absorb a late
	 * command: the chassis surges at the start and stalls at the end.
	 */
	class FWheelFeed
	{
	public:
		/** Forget everything: the next Measure only records where the leg is. */
		void Reset() { *this = FWheelFeed(); }

		/**
		 * One tick of Dt, measured at its end: the wheel centre (chassis x and
		 * z, cm), the turn rate (rad/s) of the link it hangs from over the tick,
		 * and the rates (cm/s) the servo lag model gave the leg over that tick.
		 */
		void Measure(float Dt, const FVector2D& LegXZ, float CarrierRate, const FVector2D& ModelRate);

		/** The rates (cm/s) the servo lag model gives the leg over the coming
		 *  tick, under the commands just sent. */
		void Predict(const FVector2D& ModelRateNext);

		/** The leg's fore/aft rate (cm/s) over the coming tick. */
		float GetLegXRate() const { return MeasuredRate.X + Change.X; }

		/** The carrier's turn rate (rad/s) over the coming tick. */
		float GetCarrierRate() const { return MeasuredCarrier + Fit.Predict(Change.X, Change.Y); }

		const FCarrierFit& GetFit() const { return Fit; }

	private:
		FCarrierFit Fit;
		FVector2D	LastXZ = FVector2D::ZeroVector;
		bool		bHasLast = false;
		FVector2D	MeasuredRate = FVector2D::ZeroVector;
		float		MeasuredCarrier = 0.0f;
		FVector2D	ModelRateLast = FVector2D::ZeroVector;
		FVector2D	Change = FVector2D::ZeroVector;
	};

	/** What the controller keeps between ticks to drive by: the stall watch,
	 *  the watch on a stop at the commit point, a feed for each leg's wheel, in
	 *  Legs order, and the rider's stick. */
	struct FDriveState
	{
		FStallWatch		   Stall;
		FRestWatch		   CommitRest;
		TArray<FWheelFeed> Feeds;
		FStickGate		   Stick;
	};

	/** A first-order follower (time constant Lag) of Command, Dt on from
	 *  Value: the servo lag model. */
	float LagStep(float Value, float Command, float Dt, float Lag);

	/** The speed trim's integral gain (1/s) and its bound (cm/s); it runs only
	 *  while the command is steady: within SpeedTrimSteadyCmS of itself as the
	 *  chassis follows it. */
	constexpr double SpeedTrimGain = 1.0;
	constexpr double SpeedTrimMaxCmS = 45.0;
	constexpr double SpeedTrimSteadyCmS = 1.0;
	/** The trim runs only on a grade steeper than this (deg, along the
	 *  heading); on less it fades (SpeedTrimFadeSeconds) -- a shortfall on the
	 *  level is something holding a wheel back (a leg moving, a tyre
	 *  scrubbing), which more drive would only fight, slewing the robot. */
	constexpr double SpeedTrimGradeDeg = 2.0;
	constexpr double SpeedTrimFadeSeconds = 1.0;
	/** On level ground (development, ramms.Curb.SpeedTrimLevel) the trim is
	 *  bounded to this either way (cm/s). */
	constexpr double SpeedTrimLevelMaxCmS = 6.0;
	/** The most of the measured speed's shortfall (cm/s, either way) the trim
	 *  integrates on one tick: a large one is a transient the chassis' own lag
	 *  is still taking out (coming out of a roll back, say), not the steady
	 *  error a grade leaves, and integrated whole it winds the trim up past
	 *  what the grade needs. */
	constexpr double SpeedTrimMaxErrorCmS = 8.0;

	/**
	 * One tick of the speed trim (cm/s): the integral, at SpeedTrimGain, of
	 * how far the chassis' measured speed falls short of its command as the
	 * chassis can follow it (CommandFollowedCmS: the command through the
	 * drive's lag and the speed measurement's window; at most
	 * SpeedTrimMaxErrorCmS of it a tick), bounded by SpeedTrimMaxCmS. Added to
	 * what the carrying driven wheels are asked, it makes the chassis go at its
	 * command where the wheels' own speed loops (proportional: they carry a
	 * load only with a speed error) would not: on a grade, a robot asked to
	 * stop rolls on at the speed error its weight along the slope needs.
	 * Unchanged while the measurement is not valid, and while the command
	 * (CommandCmS) is more than SpeedTrimSteadyCmS from itself as followed --
	 * braking or speeding up, the chassis' own lag (which no model has
	 * exactly) is most of the difference, and an integral of it overshoots
	 * the speed at the end of the ramp. The follower must start from the
	 * chassis' signed measured speed (a robot rolling back is not at rest).
	 */
	double SpeedTrimStep(double Trim, double CommandCmS, double CommandFollowedCmS, double MeasuredCmS, bool bMeasuredValid, bool bOnGrade,
		double Dt);

	/**
	 * The speed trim kept under a hard braking cap (cm/s): once the chassis
	 * goes faster than the cap (MeasuredCmS over CapCmS, the speed it can
	 * still brake short of a window from), the trim forward is cut to the room
	 * between the drive command (CommandCmS) and the cap -- the trim carries a
	 * grade's load at the speed asked, it never takes the chassis past where
	 * it can stop. Under the cap, or backward (down a grade, or a robot
	 * rolling back), it is left alone: cut whenever the cap merely binds the
	 * command, it would slow a robot on a grade under the command the cap
	 * allows.
	 */
	double SpeedTrimUnderCap(double Trim, double MeasuredCmS, double CommandCmS, double CapCmS);

	/**
	 * The drive command as the measured speed can follow it, one tick on: the
	 * command through the chassis' lag behind it (DriveLagSeconds), then
	 * through the speed measurement's own (DriveLagSeconds: half its 0.3 s
	 * window) -- two first-order stages, A and B, kept by the caller. Returns
	 * B: what SpeedTrimStep compares the measured speed with.
	 */
	double SpeedFollowStep(double& A, double& B, double CommandCmS, double Dt);

	/**
	 * The heading correction (rad, positive turning towards the robot's +Y)
	 * for a chassis OffsetCm to the +Y side of the line it follows: back
	 * towards the line, GainDegPerCm per cm, at most MaxDeg either way.
	 */
	double CrossTrackSteer(double OffsetCm, double GainDegPerCm, double MaxDeg);

	/** The sideways speed command's gain on the offset (1/s), its integral's
	 *  gain (1/s^2) and the most either asks (cm/s). */
	constexpr double LateralGain = 2.0;
	constexpr double LateralTrimGain = 2.0;
	constexpr double LateralMaxCmS = 30.0;

	/**
	 * The sideways speed (cm/s, towards the robot's +Y) to ask of the wheels
	 * that can roll the chassis sideways (omni wheels whose hubs are turned off
	 * the heading) for a chassis OffsetCm to the +Y side of its line: back
	 * towards it, LateralGain per cm, plus an integral of the offset (Trim,
	 * cm/s, kept by the caller) that carries a steady sideways push -- the
	 * robot's weight on a cross slope, with nothing else holding it. The
	 * integral runs only while such wheels drive (bAuthority); both bounded by
	 * LateralMaxCmS.
	 */
	double LateralSpeed(double& Trim, double OffsetCm, bool bAuthority, double Dt);

	/**
	 * The angle (rad) a body turned about Axis (a unit vector, world) from one
	 * world rotation to the next: positive by the right-hand rule in the
	 * quaternion algebra, which is the sense a wheel rolls forward in about
	 * Up x its rolling direction.
	 */
	double TurnAbout(const FQuat& From, const FQuat& To, const FVector& Axis);
} // namespace RammsCurb
