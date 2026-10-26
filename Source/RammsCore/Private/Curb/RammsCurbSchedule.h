// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsCurbTerrain.h"

/**
 * The per-wheel gait's plan of a crossing (RammsCurb::PlanCrossing), as plain
 * data: what each wheel does and where, and the robot's planned state every
 * sample (1 cm) of chassis travel. Not reflected, not exported.
 *
 * Progress is the chassis origin's travel S (cm) along the heading the
 * crossing holds, from where the plan was made. Each wheel crosses the edge
 * on its own: it lifts off before its keep-off window (the stretch of its
 * distance past the edge where a loaded wheel must not be), is clear of the
 * terrain over it, may land once past it, touches down and takes load. Every
 * sample says which wheels are off the ground then, where the chassis is
 * (its origin's world height and its attitude), where the centre wheels
 * stand fore/aft, how fast it drives, and how far inside the wheels on the
 * ground the robot stands.
 */
namespace RammsCurb
{
	/** How a crossing was planned: level all the way; tilted where the legs
	 *  cannot span the terrain level; level with a stop where it cannot keep
	 *  rolling; or not at all. */
	enum class EPlanClass : uint8
	{
		Level,
		Tilt,
		PlannedStop,
		Refused,
	};

	/** The planner's stage a plan came from, in the order the planner tries
	 *  them, the best last: refused, level with planned stops (or a tilt and
	 *  stops), tilted, level once slowed where it failed at its own speeds,
	 *  level at its own speeds. Of two plans of one crossing the later stage
	 *  ranks first (RammsCurb::IsBetterPlan). */
	enum class EPlanStage : uint8
	{
		Refused,
		Stops,
		Tilt,
		LevelSlowed,
		Level,
	};

	/** One wheel's crossing. Distances are its centre's distance past the
	 *  edge (cm, along the edge normal); S the chassis travel (cm). */
	struct FWheelPlan
	{
		/** Its leg, in the controller's Legs order. */
		int32 Leg = INDEX_NONE;
		/** Its keep-off window: no loaded wheel between DUp and DDown. Climbing
		 *  [-FaceMargin - Eps, LandingOverlap + Lambda], descending
		 *  [-EdgeMargin - Lambda, DropClearance + Eps]: Lambda = (w_t / 2)
		 *  |a . n| and Eps = R |f . n| + Lambda from its own rolling direction f
		 *  and axle a against the edge normal n. */
		double DUp = 0.0;
		double DDown = 0.0;
		double Eps = 0.0;
		double Lambda = 0.0;
		/** The surface it starts on and the one it lands on: 0 near, 1 far. */
		int32 FromPlane = 0;
		int32 ToPlane = 1;
		/** How long before reaching its window its lift must begin (s): the leg's
		 *  rise (trapezoid at LegSpeed and LegAcceleration, then its servo lag)
		 *  kept over the clearance profile; how long it takes to set down once
		 *  past it, and to take load (s). */
		double TLift = 0.0;
		double TPlant = 0.0;
		double TLoad = 0.0;
		/** Its lift eased (LiftEaseSeconds): begun TEase s earlier than it need
		 *  be, where the robot stands with it up, its leg rising at LiftSpeed
		 *  (cm/s; 0: the tuning's LegSpeed) so that it is clear by its window all
		 *  the same. SLiftStart and TLift include the easing; TLiftMin is the
		 *  lead with the leg rising as fast as it can. */
		double TEase = 0.0;
		double LiftSpeed = 0.0;
		double TLiftMin = 0.0;
		/** Where (S, cm) it begins to lift, reaches its window (must be clear),
		 *  leaves it (may land), touches down, and has taken load. Off the ground
		 *  from SLiftStart to SLoaded. */
		double SLiftStart = 0.0;
		double SClear = 0.0;
		double SLand = 0.0;
		double SDown = 0.0;
		double SLoaded = 0.0;
		/** The wheels whose landing its lift waits on: lifting it before they
		 *  have taken load would leave a set off the ground the robot does not
		 *  stand on (bit per leg). */
		uint32 MustBeDownMask = 0;
		/** Held off the ground from the start (the stance the robot drove up in)
		 *  rather than lifted. */
		bool bHeldFromStart = false;
	};

	/** The plan at one sample of chassis travel. */
	struct FScheduleSample
	{
		double S = 0.0;
		/** Planned drive speed (cm/s). */
		double V = 0.0;
		/** The chassis origin's world height (cm). */
		double B = 0.0;
		/** The base's planned attitude (deg): pitch nose up, roll +Y side up. */
		double PitchDeg = 0.0;
		double RollDeg = 0.0;
		/** Each centre 5-bar's fore/aft (cm, chassis frame), in Legs order of the
		 *  5-bar legs. */
		double X[2] = { 0.0, 0.0 };
		/** Off the ground (bit per leg). */
		uint32 AirborneMask = 0;
		/** The dynamic stability margin (cm) at the planned pose. */
		double Margin = 0.0;
		/** The chassis heights (world, cm) every leg, the belly and the margin
		 *  allow here. */
		double BLo = 0.0;
		double BHi = 0.0;
		/** Within the level-in (the robot still in its drive stance or on its
		 *  way from it): not checked as a planned pose. */
		bool bApproach = false;
		/** A planned stop: the robot is brought to rest here, and rests while
		 *  the centre wheels and the chassis move to the next sample's fore/aft
		 *  and height (within CenterShiftSpeed and BaseHeightSpeed) -- for
		 *  DwellSeconds. */
		bool   bStop = false;
		double DwellSeconds = 0.0;
	};

	/** A whole crossing's plan. */
	struct FCurbSchedule
	{
		EPlanClass Class = EPlanClass::Refused;
		/** Where the plan was made from: the chassis origin (world xy) at S = 0
		 *  and the heading held. */
		FVector2D		  Origin = FVector2D::ZeroVector;
		FVector2D		  Heading = FVector2D(1.0, 0.0);
		FRammsCurbTerrain Terrain;
		/** One per leg, in Legs order. */
		TArray<FWheelPlan> Wheels;
		/** Every SampleCm of S from 0. */
		TArray<FScheduleSample> Samples;
		double					SampleCm = 1.0;
		/** The least planned speed once the robot has risen from rest (cm/s),
		 *  planned stops left out. */
		double VFloor = 0.0;
		/** The largest planned inclination of the base from level (deg; its total
		 *  tilt, RammsCurb::InclinationDeg), and the largest planned pitch and
		 *  roll (each signed as planned). */
		double	  PlannedMaxTiltDeg = 0.0;
		FVector2D PlannedTiltDeg = FVector2D::ZeroVector;
		/** The stretch of S the planned tilt is held over (cm; empty when none). */
		FVector2D TiltRangeCm = FVector2D::ZeroVector;
		int32	  PlannedStops = 0;
		/** The least dynamic margin over the planned poses (cm). */
		double MinMargin = 0.0;
		/** The narrowest stretch of chassis heights the plan passes through (cm). */
		double NarrowestCorridorCm = 0.0;
		/** From the first wheel's lift to the last wheel's load, as planned (s). */
		double PlannedCrossingSeconds = 0.0;
		/** The drive stance's stability margin when planning (cm): braking and
		 *  speeding up from where it stands (the dynamic margin). Under
		 *  MinStabilityMargin the robot levels in standing still before it
		 *  moves (bLevelInAtRest, LevelFromS 0), or at once as it drives off or
		 *  rolls on (bLevelInAtOnce), or at once holding the speed it rolls at
		 *  until it is level (bHoldSpeedThroughLevelIn). */
		double ApproachMarginCm = 0.0;
		bool   bLevelInAtRest = false;
		bool   bLevelInAtOnce = false;
		bool   bHoldSpeedThroughLevelIn = false;
		/** The level-in's least margin by a straight move from the drive stance
		 *  to the first level pose (cm; reported). */
		double LevelInMarginCm = 0.0;
		/** Where the level-in ends (S, cm): the robot planned level from there;
		 *  and how long it takes (s). */
		double LevelFromS = 0.0;
		double LevelInSeconds = 0.0;
		/** Where the level-in begins (S, cm): LevelInSeconds before LevelFromS at
		 *  the planned speed, or 0 where it begins as the crossing does (a
		 *  stance under MinStabilityMargin, or a near surface sloping more than
		 *  EarlyLevelInSlopeDeg). And the latest it may end (S, cm): 30 cm (across
		 *  the edge) short of the first wheel's window. */
		double LevelInFromS = 0.0;
		double LevelInLatestS = 0.0;
		/** One line per event in S order, then a summary; GetScheduleLines. */
		TArray<FString> Lines;
		/** What it planned, or why it refused. */
		FString Summary;
		/** How long the plan took (ms, wall time). */
		double PlanMs = 0.0;
		/** The planner's stage it came from (Refused for a refusal), and its work
		 *  (instants of the robot model evaluated: the same for the same input,
		 *  unlike its time). */
		EPlanStage Stage = EPlanStage::Refused;
		int64	   PlanWork = 0;

		bool IsValid() const { return Class != EPlanClass::Refused; }

		/** The sample at S, clamped to the plan. */
		const FScheduleSample* SampleAt(double S) const
		{
			if (Samples.Num() == 0)
			{
				return nullptr;
			}
			const int32 I = FMath::Clamp(FMath::RoundToInt(S / SampleCm), 0, Samples.Num() - 1);
			return &Samples[I];
		}
	};

	/** "level", "tilt", "planned stop" or "refused". */
	inline const TCHAR* PlanClassName(EPlanClass Class)
	{
		switch (Class)
		{
			case EPlanClass::Level:
				return TEXT("level");
			case EPlanClass::Tilt:
				return TEXT("tilt");
			case EPlanClass::PlannedStop:
				return TEXT("planned stop");
			default:
				return TEXT("refused");
		}
	}
} // namespace RammsCurb
