// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsDriveMotion.h"

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
	 *    wheels down, held still (HoldForStance).
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
		/** End this switch's hold and wait on the active mode, if any. */
		void ReleaseActive(ISwitchHost& Host);
		void ResetStages();

		ESwitchPhase  Phase = ESwitchPhase::Idle;
		int32		  Active = INDEX_NONE;
		int32		  Pending = INDEX_NONE;
		int32		  StanceMode = INDEX_NONE;
		bool		  bActiveHeld = false;
		bool		  bAwaiting = false;
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
