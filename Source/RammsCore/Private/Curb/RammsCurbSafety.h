// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsCurbTerrain.h"

/**
 * The per-wheel crossing's runtime checks, as plain C++ fed made-up numbers
 * so that each can be tested on its own: what the executor (RammsCurb::
 * FExecutor) watches every tick once a crossing runs, and the thresholds it
 * holds them to. Not reflected, not exported.
 *
 *  - the terrain monitor: traces ahead of and under every wheel against the
 *    planes the crossing was planned on (CheckTerrain);
 *  - the base against its planned attitude, as an inclination
 *    (AttitudeErrorDeg);
 *  - a lifted wheel's rim against the terrain the traces find under it
 *    (RimGap);
 *  - and the time a condition has held (FPersist), which every "for so long"
 *    check counts with.
 *
 * Layering: speed caps first (slow down), holds second (brake and wait),
 * faults third (stop and hold), and refusals before starting.
 */
namespace RammsCurb
{
	/** What the traces and the plane fits are good to (cm): a difference
	 *  within this of a limit is not past it. */
	constexpr double TerrainToleranceCm = 0.1;
	/** A probe within this (cm, across the edge) of the edge line is on
	 *  neither surface: the face, the lip, a face that leans. */
	constexpr double TerrainFaceBandCm = 4.0;
	/** A probe further than this (cm) off its surface, for this long (s), is
	 *  terrain not as detected (with TerrainMismatchCm). */
	constexpr double TerrainMismatchSeconds = 0.05;
	/** A surface the traces find this far (cm) off its plane, before any wheel
	 *  has begun to cross, is re-fitted and the crossing re-planned on it. */
	constexpr double ReplanTerrainCm = 1.0;
	/** A surface the traces find at least this far (cm) off its plane is taken
	 *  up by its bias while too few wheels stand on it to say. */
	constexpr double TerrainSeenCm = 0.5;
	/** The probes along the heading from each wheel's centre (cm): behind it
	 *  (a descent's lip), under it, and ahead -- the furthest far enough that a
	 *  robot rolling at a crossing's speeds can stop short of a wheel's window
	 *  once the probe has seen past the face band. */
	constexpr double ProbeAheadCm[] = { -5.0, 0.0, 5.0, 10.0, 20.0, 30.0 };
	constexpr int32	 NumProbes = UE_ARRAY_COUNT(ProbeAheadCm);

	/** Half AttitudeFaultDeg off the planned attitude for this long (s) slows
	 *  the crossing to AttitudeSlowShare of the plan's speed; the whole of it
	 *  for AttitudeFaultSeconds faults. */
	constexpr double AttitudeSlowSeconds = 0.1;
	constexpr double AttitudeSlowShare = 0.5;
	constexpr double AttitudeFaultSeconds = 0.25;

	/** The measured margin under MinLiveStabilityMarginCm holds the crossing
	 *  (braked at this share of Deceleration); back over it by
	 *  MarginReleaseCm, it goes on. Under MarginFaultCm for MarginFaultSeconds
	 *  it faults. */
	constexpr double MarginHoldDecelShare = 0.5;
	constexpr double MarginReleaseCm = 0.25;
	constexpr double MarginFaultCm = 0.5;
	constexpr double MarginFaultSeconds = 0.1;

	/** A lifted wheel's rim this near (cm) the terrain the traces find under
	 *  it, inside its keep-off window, stops the robot (a speed cap of 0);
	 *  still there after SwingFaultSeconds, it faults. */
	constexpr double SwingFaultCm = 0.5;
	constexpr double SwingFaultSeconds = 0.5;

	/** A landing wheel stopped above its surface by more than
	 *  LandingToleranceCm for this long (s) faults. */
	constexpr double TouchdownFaultSeconds = 0.1;

	/** A landing wheel pressing on the ground (FPressWatch): over
	 *  PressWindowSeconds its leg went down at least PressLegCm (chassis frame)
	 *  while the wheel itself moved no more than PressWheelCm (world). */
	constexpr double PressWindowSeconds = 0.3;
	constexpr double PressLegCm = 0.4;
	constexpr double PressWheelCm = 0.15;

	/** A leg this far (cm) from where its servo lag model has it, for this
	 *  long (s), faults. */
	constexpr double LegErrorFaultCm = 3.0;
	constexpr double LegErrorFaultSeconds = 1.0;

	/** An action -- a climb's lift up to clear, a landing's plant down to the
	 *  surface, the level-in -- taking longer than ActionTimeoutFactor times
	 *  what the plan gave it, plus ActionTimeoutExtraSeconds, faults; one
	 *  taking LateActionSeconds longer than planned is logged late. */
	constexpr double ActionTimeoutFactor = 3.0;
	constexpr double ActionTimeoutExtraSeconds = 1.0;
	constexpr double LateActionSeconds = 0.3;

	/** Re-plan triggers before any wheel has begun to cross: the heading
	 *  turned this far (deg) from the plan's, or a wheel's window this far (cm,
	 *  of chassis travel) from where the plan has it. After the crossing has
	 *  begun a re-plan is not possible: the heading turned that far faults,
	 *  the window is logged (every trigger is on measured distances). */
	constexpr double ReplanYawDeg = 4.0;
	constexpr double ReplanWindowCm = 3.0;
	/** Re-plan triggers fire at most once in this much travel (cm). */
	constexpr double ReplanEveryCm = 10.0;

	/**
	 * How long (s) a condition has held: Tick adds Dt while it holds and
	 * starts again from 0 when it does not; true once it has held Need.
	 */
	struct FPersist
	{
		double Seconds = 0.0;

		bool Tick(bool bHolds, double Dt, double Need)
		{
			Seconds = bHolds ? Seconds + FMath::Max(0.0, Dt) : 0.0;
			return bHolds && Seconds >= Need;
		}
		void Reset() { Seconds = 0.0; }
	};

	/**
	 * A landing wheel on the ground with its leg still going down: the leg
	 * lifting the robot rather than lowering the wheel. A leg that meets the
	 * ground at speed stalls, which the touchdown sees; one creeping down (at
	 * its reach, the chassis coming down to it) can lift the robot off its
	 * other wheels instead, slowly enough never to stall. Tick takes the
	 * wheel's height (world) and its leg's (chassis frame, cm), both measured;
	 * true at the end of a PressWindowSeconds window over which the leg went
	 * down PressLegCm or more and the wheel moved PressWheelCm or less. A
	 * window in which the wheel moves more starts again from there.
	 */
	struct FPressWatch
	{
		double Seconds = -1.0;
		double WheelZ0 = 0.0;
		double LegZ0 = 0.0;

		bool Tick(double WheelZ, double LegZ, double Dt)
		{
			if (Seconds < 0.0 || FMath::Abs(WheelZ - WheelZ0) > PressWheelCm)
			{
				Seconds = 0.0;
				WheelZ0 = WheelZ;
				LegZ0 = LegZ;
				return false;
			}
			Seconds += FMath::Max(0.0, Dt);
			if (Seconds < PressWindowSeconds)
			{
				return false;
			}
			const bool bPressing = LegZ0 - LegZ >= PressLegCm;
			Seconds = 0.0;
			WheelZ0 = WheelZ;
			LegZ0 = LegZ;
			return bPressing;
		}
		void Reset() { Seconds = -1.0; }
	};

	/** What the live margin's watch says: carry on, hold (no wheel lifts, the
	 *  robot braked to rest), or fault. */
	enum class EMarginWatch : uint8
	{
		Ok,
		Hold,
		Fault,
	};

	/**
	 * The measured stability margin, watched: under MinLiveCm it holds, until
	 * it is MarginReleaseCm back over it; under MarginFaultCm for
	 * MarginFaultSeconds it faults.
	 */
	struct FMarginWatch
	{
		bool	 bHold = false;
		FPersist Low;

		EMarginWatch Tick(double MarginCm, double MinLiveCm, double Dt)
		{
			if (Low.Tick(MarginCm < MarginFaultCm, Dt, MarginFaultSeconds))
			{
				return EMarginWatch::Fault;
			}
			if (!bHold && MarginCm < MinLiveCm)
			{
				bHold = true;
			}
			else if (bHold && MarginCm > MinLiveCm + MarginReleaseCm)
			{
				bHold = false;
			}
			return bHold ? EMarginWatch::Hold : EMarginWatch::Ok;
		}
		void Reset() { *this = FMarginWatch(); }
	};

	/**
	 * How far (deg) the base's attitude is from a planned one, as an
	 * inclination: the angle between the two planes the chassis would lie in
	 * (pitch, roll each the elevation of its forward and +Y axes, as
	 * InclinationDeg takes them) -- the measured inclination from level when
	 * the plan is level.
	 */
	double AttitudeErrorDeg(const FVector2D& MeasuredDeg, const FVector2D& PlannedDeg);

	/** One trace of the terrain the crossing makes: leg Leg's, AheadCm along
	 *  the heading from its wheel's centre, at XY (world); the height it hit
	 *  (world), or none (bHit false: nothing within the traced span). */
	struct FTerrainProbe
	{
		int32	  Leg = INDEX_NONE;
		double	  AheadCm = 0.0;
		FVector2D XY = FVector2D::ZeroVector;
		double	  Z = 0.0;
		bool	  bHit = false;
		/** The span traced (world z): from above to below. */
		double AboveZ = 0.0;
		double BelowZ = 0.0;
	};

	/** What the probes made of the terrain this tick. */
	struct FTerrainCheck
	{
		/** The probe furthest off the surface it lies on (cm; positive above
		 *  it), its wheel and how far ahead of it, and the surface (0 near, 1
		 *  far); WorstLeg INDEX_NONE with no probe on either surface. A probe
		 *  that hit nothing where its surface lies inside the traced span is a
		 *  drop: off by the span below its surface. */
		double WorstCm = 0.0;
		int32  WorstLeg = INDEX_NONE;
		double WorstAheadCm = 0.0;
		int32  WorstPlane = 0;
		/** Per surface (0 near, 1 far): the probes on it, and their median
		 *  distance off it (cm). */
		int32  Probes[2] = { 0, 0 };
		double MedianCm[2] = { 0.0, 0.0 };
	};

	/**
	 * The terrain monitor's look: each probe on the surface it lies over --
	 * the near plane short of the edge, the far one past it, none within
	 * TerrainFaceBandCm of the edge line or past the far surface's depth
	 * (FarDepthCm less the band: a second edge) -- against that plane plus its
	 * bias (BiasCm: X near, Y far).
	 */
	FTerrainCheck CheckTerrain(const FRammsCurbTerrain& Terrain, TConstArrayView<FTerrainProbe> Probes, const FVector2D& BiasCm);

	/**
	 * A lifted wheel's rim over the terrain the probes found under it (cm;
	 * negative into it): its centre at CentreWorld, radius RadiusCm, rolling
	 * along RollDirection (world xy, unit) on a tread TyreWidthCm wide; each
	 * probe of its own (Leg) within the tread and a radius along its rolling
	 * direction counts, the rim sqrt(R^2 - u^2) under the centre there. The
	 * largest double when no probe lies under it.
	 */
	double RimGap(int32 Leg, const FVector& CentreWorld, double RadiusCm, const FVector2D& RollDirection, double TyreWidthCm,
		TConstArrayView<FTerrainProbe> Probes);
} // namespace RammsCurb
