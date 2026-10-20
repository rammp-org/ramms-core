// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsCurbNegotiationTypes.h"
#include "RammsCurbTerrain.h"
#include "Templates/Function.h"

/**
 * The curb controller's terrain model: finding the two surfaces and the edge
 * of a curb from line traces (FitTerrain), and the contact geometry of a
 * wheel on them -- how high a wheel centre stands over a sloped surface, and
 * how high a lifted wheel must be carried to clear the edge -- as plain C++
 * fed a surface function, so each can be tested on made-up terrain. Not
 * reflected, not exported.
 *
 * The detection scans lanes parallel to the robot's heading, fits a plane to
 * the ground it stands on, finds where each lane's height jumps off that
 * plane, fits one straight edge through those points, and fits a second plane
 * to what lies beyond it. Slopes are no steps: every height is taken against
 * the plane it should be on, never against a level.
 */
namespace RammsCurb
{
	// --- the scan ------------------------------------------------------------------

	/** Spacing of the samples along each lane (cm). */
	constexpr float DetectSampleCm = 2.0f;
	/** The outer lanes run this far beyond the outermost tyres, and every
	 *  lane starts this far behind the rearmost wheel (cm). */
	constexpr float LaneMarginCm = 10.0f;
	/** The step is checked along the edge this far beyond the stretch the
	 *  robot's footprint crosses (cm). */
	constexpr float FootprintBandMarginCm = 10.0f;

	// --- planes ----------------------------------------------------------------------

	/** Random three-point planes tried before a surface's least-squares fit. */
	constexpr int32 PlaneRansacIterations = 60;
	/** A sample within this of a candidate plane counts for it (cm). */
	constexpr float PlaneRansacBandCm = 0.5f;
	/** At least this share of a surface's samples must lie within
	 *  MaxPlaneResidualCm of its plane. */
	constexpr float MinPlaneInlierFraction = 0.9f;
	/** The far plane needs at least this many samples. */
	constexpr int32 MinFarPlaneSamples = 12;

	// --- edges -----------------------------------------------------------------------

	/** Halvings of the sample spacing that place an edge (2 cm / 2^12 is
	 *  under 0.5 mm). */
	constexpr int32 EdgeBisections = 12;
	/** A jump must persist over this many samples after it, keeping its sign
	 *  and EdgePersistFraction of its size; a shorter one is a spike. */
	constexpr int32 EdgePersistSamples = 3;
	constexpr float EdgePersistFraction = 0.75f;
	/** The near plane takes each lane's samples up to this far short of its
	 *  edge, the far plane from this far past it (cm). */
	constexpr float NearShortOfEdgeCm = 4.0f;
	constexpr float FarPastEdgeCm = 4.0f;
	/** The face between the two surfaces may be at most this wide across the
	 *  edge (cm); wider, and it is a ramp. A sample within FaceOnPlaneCm of a
	 *  surface's plane is on that surface. */
	constexpr float MaxFaceWidthCm = 3.0f;
	constexpr float FaceOnPlaneCm = 0.5f;
	/** The edge line needs this many lanes' edges, the lanes along both
	 *  outermost tyre tracks among them; it may drop one lane that does not
	 *  fit. */
	constexpr int32 MinEdgeLanes = 5;

	// --- the far side and the wheels --------------------------------------------------

	/** The least far depth (cm) a crossing may start on before it has seen
	 *  all it needs; the approach must see the rest. */
	constexpr float MinPlanFarDepthCm = 60.0f;
	/** Kept beyond the depth the robot itself needs (cm). */
	constexpr float FarDepthMarginCm = 10.0f;
	/** A wheel whose bottom is within this of the near surface stands on it,
	 *  and none may be further below it (cm). */
	constexpr float WheelOnPlaneCm = 1.0f;
	/** The robot must stand on the near surface on at least this many wheels. */
	constexpr int32 MinWheelsOnPlane = 3;

	// --- looking again on the approach ---------------------------------------------

	/** The approach looks again every this much travel (cm) ... */
	constexpr float RedetectEveryCm = 10.0f;
	/** ... and throws away a result that moved anything more than this
	 *  (cm, deg) from what it had: a glitch, not a refinement. */
	constexpr float RedetectRejectCm = 2.0f;
	constexpr float RedetectRejectDeg = 2.0f;
	/** The weight a new result gets when blended into what it had. */
	constexpr float RedetectBlend = 0.5f;
	/** The plan is rebuilt when the terrain moves this far (cm, deg) from the
	 *  one it was built on. */
	constexpr float ReplanCm = 0.5f;
	constexpr float ReplanDeg = 0.5f;

	/** The per-axle gait lifts both wheels of an axle at once, which works
	 *  only this close to square (deg). */
	constexpr float PerAxleMaxApproachDeg = 10.0f;

	/** One wheel, as the detection sees it. */
	struct FTerrainWheel
	{
		FName Name;
		/** Wheel centre, world (cm). */
		FVector Centre = FVector::ZeroVector;
		float	RadiusCm = 10.0f;
		float	TyreWidthCm = 6.0f;
		float	ContactWidthCm = 6.0f;
		/** On the front axle: the edge distance is measured from these. */
		bool bFront = false;
	};

	/**
	 * How deep the far side must be for the robot to end a crossing wholly on
	 * it: its wheel centres' extent across the edge -- Wheelbase cos(a) +
	 * Track |sin(a)| at approach angle a -- plus Extra (the rear tyre, the
	 * clearance the crossing ends with past the edge, and a margin).
	 */
	struct FFarDepthNeed
	{
		/** From the rearmost a rear wheel reaches to the farthest a front one
		 *  does, fore/aft (cm). */
		float WheelbaseCm = 0.0f;
		/** Across the outermost wheel centres (cm). */
		float TrackCm = 0.0f;
		float ExtraCm = 0.0f;

		bool  IsSet() const { return WheelbaseCm > 0.0f || ExtraCm > 0.0f; }
		float At(float ApproachDeg) const;
	};

	/** What the detection works from. */
	struct FTerrainFitInput
	{
		/** The chassis' horizontal centre and unit heading, world. */
		FVector2D Origin = FVector2D::ZeroVector;
		FVector2D Heading = FVector2D(1.0, 0.0);
		/** Every wheel, measured. */
		TArray<FTerrainWheel> Wheels;

		float DetectRangeCm = 350.0f;
		float MinStepHeightCm = 2.0f;
		float MaxStepHeightCm = 22.0f;
		float MaxApproachDeg = 35.0f;
		float MaxSurfaceSlopeDeg = 8.0f;
		float MaxEdgeResidualCm = 1.5f;
		float MaxPlaneRmsCm = 0.5f;
		float MaxPlaneResidualCm = 1.5f;
		float FarFitDepthCm = 120.0f;

		/** Unset: the far depth is measured but not checked. */
		FFarDepthNeed FarDepthNeed;

		/** Check that the robot stands on the near surface, every wheel short
		 *  of the edge: before a crossing. A look again on the approach, with
		 *  legs already moving, does not. */
		bool bCheckWheels = true;

		/** The detection limits from the controller's tuning. */
		void SetTuning(const FRammsCurbTuning& Tuning);
	};

	/** One scan lane, for logs and tests. */
	struct FTerrainLane
	{
		/** Across the heading from the origin (cm, +Y). */
		float OffsetCm = 0.0f;
		/** Its first edge was found, at EdgeAtCm along the heading from the
		 *  origin; and it was used for the edge line. */
		bool  bEdge = false;
		bool  bUsed = false;
		float EdgeAtCm = 0.0f;
		/** Its far side ends at a second edge, FarDepthCm past the edge line;
		 *  else it was seen that deep. */
		bool  bSecondEdge = false;
		float FarDepthCm = 0.0f;
	};

	struct FTerrainFitReport
	{
		TArray<FTerrainLane> Lanes;
		/** Calls of the surface function. */
		int32 Traces = 0;
		/** The lane left out of the edge line, or INDEX_NONE. */
		int32 DroppedLane = INDEX_NONE;
		/** Along the heading, from the front wheel nearest the edge to it (cm). */
		float EdgeDistanceCm = 0.0f;
		/** The far depth the robot needs at this approach angle (cm), 0 when
		 *  not checked. */
		float NeededFarDepthCm = 0.0f;
		/** Half the scan's width to the outermost tyre edges (cm). */
		float TrackHalfWidthCm = 0.0f;
	};

	/**
	 * Find the curb ahead: two planes and a straight edge between them.
	 * SurfaceAt(p, z) gives the height of the topmost surface over the world
	 * point p, false where there is none. False, with why, when there is no
	 * step this detection can describe: none in range, a slope or a ramp
	 * rather than a face, an edge that is not straight or too far off square,
	 * a step too small or too large somewhere along the robot's footprint or
	 * changing sign, a surface that is not a plane or too steep, a far side
	 * too shallow, or a robot not standing on the near side (bCheckWheels).
	 * True with a far depth below the need is a crossing that may start but
	 * must confirm the rest on the way (FarDepthVerdict). OutWhy then
	 * describes what it found.
	 */
	bool FitTerrain(const FTerrainFitInput& In, TFunctionRef<bool(const FVector2D&, double&)> SurfaceAt,
		FRammsCurbTerrain& OutTerrain, FString& OutWhy, FTerrainFitReport* OutReport = nullptr);

	/** Every wheel on the near surface or above it -- none more than
	 *  WheelOnPlaneCm below, at least MinWheelsOnPlane on it -- and short of
	 *  the edge. False, naming the wheel, when the robot stands across a step
	 *  (or in a hole). */
	bool CheckWheelsOnNear(const FRammsCurbTerrain& Terrain, TConstArrayView<FTerrainWheel> Wheels,
		const FVector2D& Heading, FString& OutWhy);

	/** What a terrain's far depth allows: enough for the crossing; a start
	 *  that has to confirm the rest on the approach; too shallow (a second
	 *  edge seen short of the need); or too little in view to start. */
	enum class EFarDepth : uint8
	{
		Enough,
		Pending,
		TooShallow,
		OutOfView,
	};
	EFarDepth FarDepthVerdict(const FRammsCurbTerrain& Terrain, float NeededCm, FString* OutWhy = nullptr);

	// --- contact geometry -------------------------------------------------------------

	/** A wheel's flat contact width (cm) from FRammsCurbLeg's two widths: its
	 *  ContactWidthCm where that is set (> 0), else the tyre's whole width. */
	float ContactWidth(float TyreWidthCm, float ContactWidthCm);

	/**
	 * rho: how high a wheel's centre stands over the plane point straight
	 * below it, the wheel resting on a plane of Gradient and rolling along
	 * RollDirection (world xy): R sqrt(1 + (g.f)^2) + (ContactWidth / 2) |g.a|,
	 * a the axle. Exact for a cylinder of that width with a level axle.
	 */
	double WheelCentreHeight(double RadiusCm, double ContactWidthCm, const FVector2D& Gradient, const FVector2D& RollDirection);

	/**
	 * The lowest a lifted wheel's centre may be at Centre (world xy) without
	 * its rim touching the terrain: the most, over its footprint (2R along
	 * RollDirection by TyreWidthCm across), of the surface plus the rim's
	 * height over its bottom there, sqrt(R^2 - u^2) at u along the rolling
	 * direction. The higher side is taken DilationCm further towards the
	 * lower one (a climb's curb top over its face, a descent's top past its
	 * lip), so a wheel clearing this also keeps DilationCm from the corner
	 * sideways. Sampled at 9 points along and 3 across, and exactly where each
	 * line of samples crosses the dilated corner. Add the swing clearance to
	 * get the height to carry it at.
	 */
	double ClearanceHeight(const FRammsCurbTerrain& Terrain, const FVector2D& Centre, const FVector2D& RollDirection,
		double RadiusCm, double TyreWidthCm, double DilationCm);

	// --- the legacy profile, and the approach's second looks -------------------------

	/** A profile's legacy fields from a terrain: the step at the centreline,
	 *  the front wheel nearest the edge's distance to it along the heading,
	 *  the skew of a front track at the approach angle; the terrain itself. */
	void FillLegacyProfile(const FRammsCurbTerrain& Terrain, TConstArrayView<FTerrainWheel> Wheels, const FVector2D& Origin,
		const FVector2D& Heading, FRammsCurbProfile& OutProfile);

	/** The terrain a profile without one stands for: level ground at GroundZ
	 *  and StepHeightCm above it beyond an edge through EdgePoint square to
	 *  Heading. Its far depth is unknown (0, not observed). */
	FRammsCurbTerrain LevelTerrain(double GroundZ, double StepHeightCm, const FVector2D& EdgePoint, const FVector2D& Heading);

	/** How far one terrain is from another, at the first one's edge point. */
	struct FTerrainChange
	{
		/** The second's edge line from the first's edge point (cm), and the
		 *  angle between their normals (deg). */
		double EdgeCm = 0.0;
		double EdgeDeg = 0.0;
		/** Each surface's height there (cm) and the angle between its
		 *  gradients (deg). */
		double NearCm = 0.0;
		double NearDeg = 0.0;
		double FarCm = 0.0;
		double FarDeg = 0.0;

		double	MaxCm() const { return FMath::Max3(EdgeCm, NearCm, FarCm); }
		double	MaxDeg() const { return FMath::Max3(EdgeDeg, NearDeg, FarDeg); }
		FString Describe() const;
	};
	FTerrainChange CompareTerrain(const FRammsCurbTerrain& From, const FRammsCurbTerrain& To);

	/**
	 * From moved Weight of the way to To: the edge, both planes and the step;
	 * the approach angle against Heading. The far depth is the better
	 * knowledge of the two: a second edge seen by either (the nearer, when
	 * both saw one), else the deeper view.
	 */
	FRammsCurbTerrain BlendTerrain(const FRammsCurbTerrain& From, const FRammsCurbTerrain& To, double Weight,
		const FVector2D& Heading);

	/** The approach angle (deg) of an edge normal against a heading. */
	double ApproachAngleDeg(const FVector2D& Heading, const FVector2D& EdgeNormal);
} // namespace RammsCurb
