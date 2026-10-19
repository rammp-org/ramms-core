// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Misc/Optional.h"
#include "RammsCurbNegotiationTypes.h"

/**
 * The curb controller's telemetry: the chassis speed it measures and the run
 * statistics it keeps (FRammsCurbRunStats), as plain C++ fed one tick at a
 * time, so that both can be tested on made-up motion. Not reflected, not
 * exported.
 */
namespace RammsCurb
{
	/** Measured speeds below this count as stopped (cm/s). */
	constexpr float StopSpeedCmS = 2.0f;
	/** A spell below StopSpeedCmS lasting at least this long is a stop (s). */
	constexpr float MinStopSeconds = 0.2f;
	/** The chassis speed is measured over this window (s): shorter ones read
	 *  the pivots' jitter as speed. */
	constexpr float SpeedWindowSeconds = 0.3f;
	/** The robot is under way once its measured speed reaches this (cm/s):
	 *  stops are counted from here. */
	constexpr float FirstMotionSpeedCmS = 5.0f;
	/** The least speed worth calling rolling, as a fraction of the drive speed:
	 *  the speed minima start once the measured speed has risen this far from
	 *  rest, so that a standing start's own acceleration is not its slowest
	 *  moment. */
	constexpr float CruiseFloorFraction = 0.25f;
	/** A rise from rest that falls back this far below the most it reached
	 *  (cm/s) has ended without reaching the floor: the robot is slowing, and
	 *  the speed minima count from there. More than the speed window's
	 *  frame-to-frame jitter at a steady speed. */
	constexpr float RiseDropCmS = 2.0f;

	/**
	 * The base's inclination from level (deg) at a pitch and a roll (deg): the
	 * angle its plane makes with the horizontal, atan(sqrt(tan^2 pitch +
	 * tan^2 roll)) -- its total tilt, whichever way it leans. Pitch and roll
	 * are the elevations of its forward and +Y axes as ChassisFrameFrom builds
	 * them (the plane through both rises tan pitch along the heading and
	 * tan roll across it). What MaxPlannedTiltDeg and MaxTiltDeg bound.
	 */
	inline double InclinationDeg(double PitchDeg, double RollDeg)
	{
		const double Tp = FMath::Tan(FMath::DegreesToRadians(PitchDeg));
		const double Tr = FMath::Tan(FMath::DegreesToRadians(RollDeg));
		return FMath::RadiansToDegrees(FMath::Atan(FMath::Sqrt(Tp * Tp + Tr * Tr)));
	}

	/** Whether a measured attitude is past the safety limit MaxTiltDeg: its
	 *  inclination over it, whichever way it leans (not pitch or roll each). */
	inline bool OverMaxTilt(double PitchDeg, double RollDeg, double MaxTiltDeg)
	{
		return InclinationDeg(PitchDeg, RollDeg) > MaxTiltDeg;
	}

	/**
	 * The chassis' ground speed along its heading, over the last
	 * SpeedWindowSeconds of samples.
	 */
	class FMotionWindow
	{
	public:
		/** Add a sample: a time (s) and the chassis' horizontal position (cm)
		 *  and unit heading then. A time before the last one starts the window
		 *  again (the clock was reset); one equal to it replaces the last
		 *  sample (no time has passed). */
		void Add(double Time, const FVector2D& Position, const FVector2D& Forward);

		void Reset();

		bool HasSample() const { return Samples.Num() > 0; }

		/** True once the samples span the whole window. */
		bool IsFull() const;

		/** Speed (cm/s) along the latest heading, from the oldest sample held
		 *  to the newest; 0 with fewer than two. */
		float GetSpeed() const;

		/** The newest sample's position and heading. */
		FVector2D GetPosition() const { return HasSample() ? Samples.Last().Position : FVector2D::ZeroVector; }
		FVector2D GetForward() const { return Forward; }

	private:
		struct FSample
		{
			double	  Time = 0.0;
			FVector2D Position = FVector2D::ZeroVector;
		};
		/** Oldest first: the newest sample at least the window old, then every
		 *  one after it. */
		TArray<FSample> Samples;
		FVector2D		Forward = FVector2D(1.0, 0.0);
	};

	/** What the controller measured and commanded on one of its ticks. */
	struct FRunTick
	{
		/** Simulated seconds since the last tick, and since the crossing began. */
		float  DeltaSeconds = 0.0f;
		double Elapsed = 0.0;

		float CommandedSpeed = 0.0f;
		float MeasuredSpeed = 0.0f;
		/** MeasuredSpeed spans the whole window; a shorter one is not used. */
		bool bMeasuredSpeedValid = false;

		float PitchDeg = 0.0f;
		float RollDeg = 0.0f;
		/** The gait means to hold an attitude this tick, and which. */
		bool  bAttitudePlanned = false;
		float PlannedPitchDeg = 0.0f;
		float PlannedRollDeg = 0.0f;

		/** Some leg has been commanded away from where it began. */
		bool bLegsMoved = false;
		/** The largest leg error against the servo lag model. */
		float LegErrorCm = 0.0f;

		/** Measured this tick, when the gait has them to measure. */
		TOptional<float> SwingClearanceCm;
		TOptional<float> MeasuredMarginCm;

		/** The chassis against where the crossing began. */
		float LateralDriftCm = 0.0f;
		float YawChangeDeg = 0.0f;
	};

	/**
	 * Accumulates FRammsCurbRunStats over a crossing.
	 *
	 * Attitude, leg error, margins, clearance, drift and the clocks count on
	 * every tick from Begin: a robot can tilt, lag or stand on a poor margin
	 * before it moves. The speed and stop figures wait for motion: stops and
	 * the time below StopSpeedCmS count once the robot is under way
	 * (FirstMotionSpeedCmS), and the speed minima once it has also finished
	 * rising from rest -- its measured speed has reached the cruise floor, or
	 * fallen RiseDropCmS back from the most it reached. A robot that never
	 * finishes rising has its minima from first motion.
	 */
	class FRunStatsTracker
	{
	public:
		/** A crossing begins: clear everything, and keep what the plan said.
		 *  CruiseFloorSpeed is the speed (cm/s) that ends the rise from rest. */
		void Begin(float PlannedMinSpeed, float PlannedMaxTiltDeg, float PlannedCrossingSeconds,
			TOptional<float> PlannedMinMarginCm, float CruiseFloorSpeed);

		/** One controller tick, every one from Begin to the one that ends the
		 *  crossing. After End nothing counts. */
		void Tick(const FRunTick& In);

		/** The plan's figures, for a plan that came after Begin (one made off the
		 *  game thread for a crossing begun on the move). */
		void SetPlan(float PlannedMinSpeed, float PlannedMaxTiltDeg, float PlannedCrossingSeconds, TOptional<float> PlannedMinMarginCm);

		/** The gait stopped, held or re-planned on purpose. */
		void AddPlannedStop();
		void AddHold();
		void AddReplan();

		/** A re-plan, whose plan's figures join those of the plans before it:
		 *  the planned least speed and margin are the least over every plan
		 *  the crossing ran on. */
		void AddReplan(float PlannedMinSpeed, TOptional<float> PlannedMinMarginCm);

		/** The crossing ended -- Done, or a fault or abort -- at Elapsed, in
		 *  this attitude, which counts towards the largest tilt (a tilt fault
		 *  ends on a tick Tick never saw). */
		void End(double Elapsed, float PitchDeg, float RollDeg);

		const FRammsCurbRunStats& Get() const { return Stats; }
		bool					  IsUnderWay() const { return bUnderWay; }
		/** The rise from rest is over: the speed minima count from here. */
		bool HasRisen() const { return bRisen; }
		bool HasEnded() const { return bEnded; }

	private:
		/** The speed and stop figures for one tick. */
		void TickSpeed(const FRunTick& In);

		FRammsCurbRunStats Stats;
		bool			   bUnderWay = false;
		bool			   bRisen = false;
		bool			   bEnded = false;
		float			   CruiseFloor = 0.0f;
		/** The most the measured speed reached while rising from rest. */
		float RisePeak = 0.0f;
		/** The least speeds from first motion, reported until the rise ends
		 *  (both start on a tick with a valid measured speed). */
		float MotionMinCommanded = 0.0f;
		float MotionMinMeasured = 0.0f;
		/** The least speeds from the end of the rise. */
		float RisenMinCommanded = 0.0f;
		float RisenMinMeasured = 0.0f;
		/** When a leg was first commanded to move (s since Begin), or < 0. */
		double LegsMovedAt = -1.0;
		/** When the current spell below StopSpeedCmS began, or < 0. */
		double BelowStopSince = -1.0;
		bool   bStopCounted = false;
		bool   bSwingSeen = false;
		bool   bMarginSeen = false;
		/** Some plan the crossing ran on planned a margin. */
		bool bPlannedMargin = false;
	};

	/** What the controller keeps between ticks for its telemetry. */
	struct FRunTelemetry
	{
		FMotionWindow	 Motion;
		FRunStatsTracker Stats;
		/** Where the chassis was, and which way it faced, when the crossing
		 *  began. */
		FVector2D StartPosition = FVector2D::ZeroVector;
		FVector2D StartForward = FVector2D(1.0, 0.0);
	};
} // namespace RammsCurb
