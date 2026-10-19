// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Ramms4BarLegSpec.h"
#include "RammsCurbNegotiationTypes.h"
#include "RammsCurbTerrain.h"

/**
 * The curb controller's level base, as plain C++ fed made-up numbers so each
 * piece can be tested on its own. Not reflected, not exported.
 *
 * Every support leg takes its height from the plane under its own wheel --
 * the near surface before its axle crosses, the far one after -- so the
 * chassis stays level whatever the two surfaces do. The gait plans its
 * chassis height over a reference that follows the terrain under the wheels
 * (TerrainOffsets): on level ground that reference is the ground the crossing
 * began on, and every leg's height is what a level-ground plan gives it; on a
 * slope each leg differs from that by how far its own wheel's ground is above
 * or below the others'. A gentle attitude loop (FAttitudeLoop) trims what the
 * legs' servos and the drive wheels' extra press leave, as a pure rotation of
 * the support legs about their middle; a per-surface bias (FPlaneBias) takes
 * up how far the planted wheels sit off the planes the detection found.
 */
namespace RammsCurb
{
	/** The base is level within this (deg): the level-in is done, and the
	 *  first lift may begin. */
	constexpr float LevelInToleranceDeg = 1.0f;
	/** The base must be level before any wheel comes this close to its edge
	 *  (cm, across the edge). */
	constexpr float LevelBeforeEdgeCm = 30.0f;
	/** The level-in is planned to be done this far (cm) short of that ... */
	constexpr float LevelInMarginCm = 5.0f;
	/** ... allowing this long (s) beyond the time it is predicted to take. */
	constexpr float LevelInSpareSeconds = 0.3f;
	/** A level-in that is late is held this far (cm) short of the line. */
	constexpr float LevelCapMarginCm = 1.0f;
	/** The longest a level-in is taken to need when planning where the legs
	 *  start to follow the terrain (s): the plan checks the legs reach from
	 *  that far out, so the level-in may start anywhere after it. */
	constexpr float LevelInPlanSeconds = 1.5f;

	/** Descending, a lifted wheel is carried clear of the terrain over its
	 *  own footprint and this far (cm) beyond it: the high side it is leaving
	 *  may rise towards the lip, but nothing ahead rises sharply. */
	constexpr float DescentSwingLookCm = 12.0f;

	/** The plane bias follows the planted wheels with this time constant (s)
	 *  and stays within this (cm). */
	constexpr float PlaneBiasSeconds = 0.5f;
	constexpr float MaxPlaneBiasCm = 1.5f;
	/** A planted wheel's bottom is counted towards its plane's bias only this
	 *  far (cm) off it either way. */
	constexpr float PlaneBiasSampleCm = 1.5f;
	/** A surface's bias moves only while at least this many wheels stand on
	 *  it: two corner wheels alone may both be carrying nothing, floating a
	 *  few millimetres up, and a bias that followed them would lift their
	 *  legs' targets and float them further. */
	constexpr int32 MinWheelsForBias = 3;

	/** The attitude loop reads the measured attitude low-passed over this (s). */
	constexpr float AttitudeFilterSeconds = 0.05f;
	/** The attitude loop's integral runs only on at least this many planted
	 *  wheels. */
	constexpr int32 MinPlantedForIntegral = 3;

	/** A 4-bar crank read at most this far (rad) past an end of its range is
	 *  taken to be at that end: a servo pressed a hair past its stop. */
	constexpr float CrankReadingToleranceRad = 0.02f;

	/** Error less the band either side of zero; 0 inside it. */
	float Deadband(float Error, float Band);

	/**
	 * The attitude loop: a PI on the base's attitude error, as a correction
	 * (deg) the support legs apply as a pure rotation (AttitudeOffsets).
	 * Error is measured less planned (deg; X pitch, positive nose up; Y roll,
	 * positive +Y up): a nose-up error gives a positive pitch correction, which
	 * retracts the front legs, and the nose drops.
	 *
	 * correction = clamp(integral, +-Max) + Kp e, the integral moving by
	 * Ki e dt only while bIntegrate, e the error less the deadband; the whole
	 * clamped to +-Max too. No height integral: the chassis height is the
	 * plan's.
	 */
	class FAttitudeLoop
	{
	public:
		void Reset() { *this = FAttitudeLoop(); }

		/** One tick of Dt; returns the correction (deg). */
		FVector2D Tick(const FVector2D& ErrorDeg, float Dt, bool bIntegrate, float Kp, float Ki, float DeadbandDeg,
			float MaxDeg);

		FVector2D GetCorrection() const { return Correction; }
		FVector2D GetIntegral() const { return Integral; }

	private:
		FVector2D Integral = FVector2D::ZeroVector;
		FVector2D Correction = FVector2D::ZeroVector;
	};

	/**
	 * Each leg's height change (cm, chassis frame, up positive) for an
	 * attitude correction (deg; X pitch, Y roll): x tan(pitch) + y tan(roll)
	 * at its wheel (LegXY, chassis cm: x forward, y towards +Y), less the mean
	 * of that over the support legs -- a pure rotation about their middle,
	 * which leaves the chassis' height where it was. 0 for a leg not in
	 * Support.
	 */
	void AttitudeOffsets(TConstArrayView<FVector2D> LegXY, TConstArrayView<bool> Support, const FVector2D& CorrectionDeg,
		TArrayView<float> Out);

	/**
	 * How far a surface sits from the plane the detection found, as its
	 * planted wheels see it: the lowest of their bottoms' heights off it,
	 * each taken within PlaneBiasSampleCm, followed with time constant
	 * PlaneBiasSeconds and kept within MaxPlaneBiasCm. Held while fewer than
	 * MinWheelsForBias wheels stand on it. The lowest, not a middle: six stiff legs share
	 * the load by millimetres, and a wheel carrying little floats a few
	 * millimetres over its surface -- never under it.
	 */
	class FPlaneBias
	{
	public:
		void  Reset() { Bias = 0.0f; }
		void  Tick(TConstArrayView<float> ResidualsCm, float Dt);
		float Get() const { return Bias; }

	private:
		float Bias = 0.0f;
	};

	/** One wheel, for its drive role. */
	struct FDriveWheel
	{
		ERammsCurbWheelDrive Role = ERammsCurbWheelDrive::Fallback;
		/** Which side of the robot (FRammsCurbLeg::Wheel.Position.Y > 0). */
		bool bPlusY = false;
		/** On the ground, carrying: where its step wants it, and in contact. */
		bool bPlanted = false;
	};

	/**
	 * Which wheels are driven, per side: a Main wheel while it is planted; a
	 * Fallback wheel while a Main wheel on its own side is not (or its side
	 * has none); an Assist wheel throughout (on the ground it drives, lifted
	 * it is spun at ground speed, so it lands without a jolt); a Passive one
	 * never.
	 */
	void DriveRoles(TConstArrayView<FDriveWheel> Wheels, TArrayView<bool> OutDriven);

	/**
	 * How much higher (cm, never negative) a lifted wheel at chassis (X, Y)
	 * must be carried, on a chassis pitched PitchRad (nose up) and rolled
	 * RollRad (+Y up), to stay as high over the ground as it would on a level
	 * one: the tilt lowers it by x sin(pitch) + y sin(roll) where that is
	 * negative. A tilt that raises it is not taken off.
	 */
	float SwingTiltRaise(float X, float Y, float PitchRad, float RollRad);

	/** One wheel, for the terrain the legs follow. */
	struct FTerrainLeg
	{
		/** Its centre (world xy), where its height is taken. */
		FVector2D XY = FVector2D::ZeroVector;
		/** On the far surface, or headed for it; else on the near one. */
		bool   bFar = false;
		double RadiusCm = 10.0;
		double ContactWidthCm = 6.0;
		/** The direction it rolls in (world xy). */
		FVector2D RollDirection = FVector2D(1.0, 0.0);
	};

	/**
	 * The terrain under the wheels, in the plan's level-ground terms. Each
	 * wheel's level is 0 on the near surface and StepHeightCm on the far one;
	 * the reference D is the mean over the wheels of how far their surface
	 * (plus its bias: BiasCm X near, Y far) lies above GroundZ plus their
	 * level. A level-ground plan's chassis height H is then GroundZ + H + D in
	 * the world. Out gets each wheel's offset: how much higher (cm) its leg's
	 * wheel centre sits in the chassis frame than the level-ground plan puts
	 * it -- its surface's height over the reference, plus rho less its radius
	 * (WheelCentreHeight). On level ground at GroundZ every offset is 0 and so
	 * is D. Returns D.
	 */
	double TerrainOffsets(const FRammsCurbTerrain& Terrain, TConstArrayView<FTerrainLeg> Legs, double GroundZ,
		double StepHeightCm, const FVector2D& BiasCm, TArrayView<double> Out);

	/** The surface a wheel follows: the near or far plane at XY. */
	double SurfaceHeight(const FRammsCurbTerrain& Terrain, bool bFar, const FVector2D& XY);

	/**
	 * The lowest a lifted wheel's centre may be carried along Path (world xy)
	 * without its rim touching the terrain: the most of ClearanceHeight over
	 * the path. Add the swing clearance for the height to carry it at.
	 */
	double PathClearanceHeight(const FRammsCurbTerrain& Terrain, TConstArrayView<FVector2D> Path, const FVector2D& RollDirection,
		double RadiusCm, double TyreWidthCm, double DilationCm);

	/**
	 * A 4-bar leg's wheel centre (chassis x-z, cm) from its crank reading.
	 * A reading up to ToleranceRad past an end of CrankRange, where the
	 * linkage does not close, is taken at that end (bOutClamped). False when
	 * it does not close at the reading and the reading is not within the
	 * tolerance of an end. OutCrank is the angle used.
	 */
	bool MeasureFourBarCrank(const FRamms4BarLegSpec& Spec, float Crank, float ToleranceRad, FVector2D& OutWheel,
		float& OutCrank, bool& bOutClamped);

	/**
	 * A path for the chassis' height through a corridor sampled evenly along
	 * its travel: Bands (cm; least, most at each sample), moving no more than
	 * Rate (cm) from one sample to the next. Forward, then backward interval
	 * propagation finds the heights at each sample from which every band
	 * before and after can be kept; then a pass forward keeps as near Wish
	 * (cm, at each sample; NaN: hold the height before) as that lets it.
	 * False when no such path exists, OutStuck the sample where the
	 * propagation ran dry.
	 */
	bool CorridorPath(TConstArrayView<FVector2D> Bands, TConstArrayView<double> Wish, double Rate, TArray<float>& OutPath,
		int32& OutStuck);

	/** What the controller keeps between ticks for the level base. */
	struct FLevelState
	{
		FAttitudeLoop Attitude;
		/** Each surface's bias: the near one, the far one. */
		FPlaneBias NearBias;
		FPlaneBias FarBias;
		/** The measured attitude (deg; X pitch, Y roll), low-passed over
		 *  AttitudeFilterSeconds, for the attitude loop. */
		FVector2D FilteredAttitudeDeg = FVector2D::ZeroVector;
		bool	  bFilterSeeded = false;
		/** The attitude loop runs: from the first tick of the level-in on
		 *  which every support leg's command had caught up with where the
		 *  terrain wants it. Before, the base holds the stance the robot drove
		 *  up in, or is on its way from it, and a correction would only chase
		 *  that. */
		bool bAttitudeActive = false;
		/** The level-in is done: the base has been brought level from the
		 *  stance the robot drove up in, and the first lift may begin. */
		bool bLevelled = false;

		void	  Reset() { *this = FLevelState(); }
		FVector2D GetBias() const { return FVector2D(NearBias.Get(), FarBias.Get()); }

		/**
		 * One tick of the attitude loop: MeasuredDeg (pitch, roll) low-passed
		 * over AttitudeFilterSeconds (seeded on the first tick), then, while
		 * the loop runs (bAttitudeActive), the correction (deg) of
		 * FAttitudeLoop with the tuning's gains, deadband and clamp, against a
		 * planned attitude PlannedDeg, its integral moving only while
		 * bIntegrate. 0 while the loop does not run.
		 */
		FVector2D TickAttitude(const FVector2D& MeasuredDeg, const FVector2D& PlannedDeg, float Dt, bool bIntegrate,
			const FRammsCurbTuning& Tuning);

		/**
		 * One tick of the level-in, once it is under way (bUnderWay: the
		 * first Prepare has begun). The attitude loop starts on the first tick
		 * on which every support leg's command has caught up with its target
		 * (bCaughtUp): what the legs alone leave -- servo sag, the drive
		 * wheels' preload rocking the robot, error in the planes -- is the
		 * loop's to trim, and the level-in could not finish without it. The
		 * level-in is done once, with the loop running and the commands caught
		 * up, the filtered attitude is within LevelInToleranceDeg. True on the
		 * tick it is done.
		 */
		bool TickLevelIn(bool bUnderWay, bool bCaughtUp);
	};

	/**
	 * Where the level-in starts: how far (cm, along the heading) the lead
	 * wheel is still short of its edge -- the line LevelBeforeEdgeCm short of
	 * the edge across it, at ApproachDeg, plus LevelInMarginCm, plus the
	 * distance covered at SpeedCmS over the level-in's Seconds,
	 * LevelInSpareSeconds and DriveLagSeconds (the chassis running on behind
	 * its command, which LateLevelSpeed allows for: a level-in done on time
	 * then never has it brake).
	 */
	float LevelInStartCm(float Seconds, float SpeedCmS, float ApproachDeg);

	/**
	 * The speed cap on a level-in that is late (cm/s). The lead wheel is
	 * DistanceCm short of its edge along the heading; the line it must not
	 * pass before the base is level is LevelBeforeEdgeCm from the edge
	 * (across it, at ApproachDeg), less LevelCapMarginCm. The level-in starts
	 * GateCm further on (0 once it has started) and takes Seconds from there.
	 * The cap is the speed that reaches the line no sooner than the level-in
	 * is done, and never more than the braking curve to the line at
	 * Deceleration: on time it does not bind; late, the robot slows; past the
	 * line it is 0, and a robot not yet level goes no nearer the edge.
	 *
	 * The chassis runs LagSeconds behind its drive command (first order), at
	 * SpeedCmS now: the room to the line is taken less what it covers past
	 * wherever its command stops (LagBrakingDistance), as at the commit
	 * point, so that braked on it the chassis comes to rest at the line, not
	 * LagSeconds of its speed past it.
	 */
	float LateLevelSpeed(float DistanceCm, float ApproachDeg, float GateCm, float Seconds, float Deceleration, float SpeedCmS,
		float LagSeconds);
} // namespace RammsCurb
