// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Curb/RammsCurbRobotModel.h"
#include "Curb/RammsCurbRunStats.h"
#include "Curb/RammsCurbSchedule.h"
#include "RammsCurbNegotiationTypes.h"
#include "RammsCurbTerrain.h"

/**
 * The per-wheel gait's planner, as plain C++ fed a robot model, a terrain and
 * a start, so that it can be tested and dry-run on made-up robots. Not
 * reflected, not exported.
 *
 * PlanCrossing plans the whole crossing in chassis travel S before anything
 * moves:
 *
 *  - the drive stance the robot stands in, and its margin braking and
 *    speeding up: with MinStabilityMargin it levels in on the way; with
 *    less, standing still, it levels in first (if the stance has
 *    MinStabilityMargin standing), else with MinLiveStabilityMarginCm it
 *    levels in at once, else rolling it levels in at once holding its speed
 *    (if the stance has that floor at a steady speed and is level in time),
 *    else it refuses;
 *  - each wheel's keep-off window from its own rolling direction and axle,
 *    and its clearance profile -- its rim over the terrain with the higher
 *    side dilated towards the lower -- which with the leg's trapezoid move
 *    (no faster than its crank's rate limit allows) and servo lag says how
 *    long before its window its lift must begin, and how long after it it is
 *    down and loaded;
 *  - the per-instant model: which wheels are off the ground, each support
 *    leg on the plane under its own wheel, each lifted one over its profile,
 *    the belly over the terrain under the plate, the chassis heights all of
 *    them allow (the corridor), and the dynamic stability margin (the
 *    robot model's centre of mass and zero-moment point against the support
 *    wheels' contacts), which must hold over MinCorridorCm of chassis
 *    heights. Which wheels conflict -- may not be off the ground together --
 *    is never tabled: a set that leaves no pose with the margin is what
 *    forbids it; and whatever the margin, never a same-side corner and
 *    centre, both fronts and a centre, or a centre and both rears;
 *  - centre wheels that walk: no walk first, then walking knots (forward
 *    while the front corners are up, aft for the rears, carried while
 *    lifted), then a search over each wheel's path on a grid of its 5-bar's
 *    reach (the fallback: both alike, then one with the other held), and
 *    where that still fails a search over both wheels' pairs about each
 *    stretch that fails, kept only where it makes the plan level; always
 *    within CenterShiftSpeed and never walking a planted wheel aft faster
 *    than 0.7 of the travel, but at a planned stop, where the robot rests
 *    while they re-pose;
 *  - the speed envelope: where a wheel's lift lead or landing tail puts a set
 *    off the ground the robot does not stand on, the speed over that stretch
 *    is capped until it does not (the gap cap), then passes forward and back
 *    at Acceleration and Deceleration, from the speed the robot enters at; a
 *    cap under the floor (MinSpeedFraction x DriveSpeed) fails level, and the
 *    robot never comes to rest but at a planned stop;
 *  - the chassis height: a path through the corridor, no faster than
 *    BaseHeightSpeed at the planned speed, as near its middle as it may;
 *  - the fallback chain, stretch by stretch where level fails at or above
 *    the floor: the least tilt -- the least inclination of the base from
 *    level (InclinationDeg) up to MaxPlannedTiltDeg, which bounds the
 *    inclination, not pitch and roll each -- each stretch's least by two
 *    picks (ETiltPick: rings of inclination TiltGridDeg apart, the middle of
 *    a ring's widest arc that keeps a pose; squares of the larger axis, the
 *    first that does); where neither verifies at the plan's speeds, the
 *    stretches it failed about made larger; every try by the inclination its
 *    plan comes to, least first, every candidate's least and at most
 *    Ramms.Curb.MaxTiltPlans larger ones (SearchTilts)
 *    -- held over the stretch that needs it and eased in, between stretches
 *    and out on S-curves along the straight way between two tilts, the
 *    base's turn (pitch and roll together) within TiltRateDegPerSec and
 *    TiltAccelDegPerSec2 in time
 *    (slowing over an easing that has too little
 *    time; an easing that would not fit even at the floor runs into the
 *    stretches either side, or one of them is left out); then, if allowed,
 *    a planned stop where no tilt is enough (the least tilt elsewhere), or
 *    level with planned stops, the robot resting at each while the centre
 *    wheels and the chassis re-pose; then a refusal naming where, which
 *    wheels and why -- each fallback on the level search's few best
 *    candidates;
 *  - verification every SampleCm: every leg's reach (LegRangeMargin inside
 *    the 4-bar tables and the knee-limited 5-bar grids), lifted wheels over
 *    their profiles, the belly, the margin, the corridor, the speed floor,
 *    no stop but a planned one, braking from the entry speed within
 *    Deceleration, every pose of a planned stop's re-pose, the base's
 *    inclination within MaxPlannedTiltDeg at every sample (the easings
 *    between two tilts included), and the rate limits on the chassis
 *    height, the centre wheels and the tilt (the base's turn rate and its
 *    change, pitch and roll together); the level-in's least
 *    margin by a straight move from the
 *    stance, reported;
 *  - one plan line per event.
 */
namespace RammsCurb
{
	/** The most legs a plan handles (its masks are bits). */
	constexpr int32 MaxPlanLegs = 8;

	/** A wheel takes load over this long once down (s). */
	constexpr double LoadSeconds = 0.1;
	/** A leg move is taken to settle this long after its lag (s). */
	constexpr double LegSettleSeconds = 0.05;
	/** A planted centre wheel walks aft at most this much per cm of travel:
	 *  its ground speed stays at least 0.3 of the chassis'. */
	constexpr double MaxAftWalkPerCm = 0.7;
	/** The planned tilt is searched on rings of inclination this far apart
	 *  (deg), each ring's tilts about this far apart along it. */
	constexpr double TiltGridDeg = 0.25;
	/** The tilt search orders its tries by inclination to this resolution
	 *  (deg): tilts within it are alike (a square's 3.748 deg and a ring's
	 *  3.75), and of those the ring's goes first; a tilt a tenth of a degree
	 *  less inclined goes before. */
	constexpr double TiltOrderDeg = 0.05;
	/** A corner held off the ground in the drive stance stays up when its own
	 *  lift would begin within this much travel (cm) of the level-in's end. */
	constexpr double HeldCornerLeadCm = 40.0;
	/** On a near surface sloping more than this (deg) the level-in begins as
	 *  soon as the crossing does: the drive stance is least stable there. */
	constexpr double EarlyLevelInSlopeDeg = 2.0;
	/** From a wheel's touchdown until this long (s) after it has taken load,
	 *  the planned chassis height moves at only this share of BaseHeightSpeed. */
	constexpr double LandingSettleSeconds = 0.3;
	constexpr double LandingHeightShare = 0.25;
	/** The least servo lag (s) a touchdown's approach is shaped by. */
	constexpr double MinTouchdownLagSeconds = 0.02;

	/** How fast a wheel being set down may be sent down with its command
	 *  HeightCm over where it meets the ground (cm/s): TouchdownSpeed plus
	 *  that height over a servo lag, so it slows over its last few
	 *  centimetres; no limit with TouchdownSpeed 0. The executor's landings
	 *  and the planner's timing of them both use it. */
	inline double TouchdownSpeedCap(double HeightCm, const FRammsCurbTuning& T)
	{
		if (T.TouchdownSpeed <= 0.0f)
		{
			return TNumericLimits<double>::Max();
		}
		return T.TouchdownSpeed + FMath::Max(0.0, HeightCm) / FMath::Max(MinTouchdownLagSeconds, static_cast<double>(T.ServoLagSeconds));
	}

	/** The hard braking caps' deceleration (cm/s^2): BrakingDeceleration, else
	 *  Deceleration. */
	inline double BrakingDecel(const FRammsCurbTuning& T)
	{
		return FMath::Max(1.0, static_cast<double>(T.BrakingDeceleration > 0.0f ? T.BrakingDeceleration : T.Deceleration));
	}

	/** How long before reaching its window (s) a wheel's lift must begin, at
	 *  speed V, for the robot never to come within its braking cap's distance
	 *  of that window with the wheel still down: the drive's lag and the
	 *  braking at BrakingDecel. */
	inline double BrakeLeadSeconds(double V, double LagSeconds, const FRammsCurbTuning& T)
	{
		return LagSeconds + FMath::Max(0.0, V) / (2.0 * BrakingDecel(T));
	}

	/** How long a wheel takes to come down DownCm to the ground at Speed (s),
	 *  slowed over its last centimetres by TouchdownSpeedCap (its speed's
	 *  changes not counted). */
	inline double DescentSeconds(double DownCm, double Speed, const FRammsCurbTuning& T)
	{
		DownCm = FMath::Max(0.0, DownCm);
		Speed = FMath::Max(1.0, Speed);
		if (T.TouchdownSpeed <= 0.0f || T.TouchdownSpeed >= Speed)
		{
			return DownCm / Speed;
		}
		// Within Zone of the ground the cap is under Speed; there the command
		// closes on the ground exponentially, at TouchdownSpeed at the last.
		const double Tau = FMath::Max(MinTouchdownLagSeconds, static_cast<double>(T.ServoLagSeconds));
		const double Vt = T.TouchdownSpeed;
		const double Zone = (Speed - Vt) * Tau;
		const double Above = FMath::Max(0.0, DownCm - Zone);
		return Above / Speed + Tau * FMath::Loge((Vt + FMath::Min(DownCm, Zone) / Tau) / Vt);
	}

	/** What the planner knows of one leg beyond the robot model. */
	struct FPlannerLeg
	{
		FName Name;
		/** The tyre's whole width (cm): a lifted wheel's footprint. */
		double TyreWidthCm = 6.0;
		/** How far below its surface a support wheel is sent (cm):
		 *  PlantPreload, and MainWheelPreload more on a Main wheel. */
		double PreloadCm = 0.3;
		/** Where it is now (chassis x-z, cm), and whether it is off the ground
		 *  (the stance the robot drove up in). */
		FVector2D Measured = FVector2D::ZeroVector;
		bool	  bOffGround = false;
		/** The fastest its crank turns (rad/s); 0 sets no limit. A 4-bar's lift
		 *  and landing take at least the crank's turn over its table at this
		 *  rate; a 5-bar's hips are not limited by it. */
		double MaxCrankRateRadS = 0.0;
		/** A 5-bar's reach (knee-limited). */
		FFiveBarGrid Grid;
	};

	/** The 5-bar legs' reach grids, kept by the controller between plans: one
	 *  per leg (invalid for a 4-bar), built from each leg's controller (Keys)
	 *  once. */
	struct FFiveBarGridCache
	{
		TArray<const void*>	 Keys;
		TArray<FFiveBarGrid> Grids;
		/** How long building them took (ms). */
		double BuildMs = 0.0;
	};

	/** Everything a plan is made from. The plan copies nothing it does not
	 *  need, but the input must outlive the call. */
	struct FPlannerInput
	{
		FRobotModel			Model;
		TArray<FPlannerLeg> Legs;
		FRammsCurbTerrain	Terrain;
		/** The chassis origin (world xy) and its height (world z), and the heading
		 *  the crossing holds (unit, world xy). */
		FVector2D Origin = FVector2D::ZeroVector;
		double	  OriginZ = 0.0;
		FVector2D Heading = FVector2D(1.0, 0.0);
		/** The base's attitude now (deg): pitch nose up, roll +Y side up. */
		FVector2D AttitudeDeg = FVector2D::ZeroVector;
		/** The chassis' speed now (cm/s): where the plan's speed starts. */
		double EntrySpeedCmS = 0.0;
		/** The robot moving as it asks (rolling on, or back: EntrySpeedCmS is
		 *  never under 0): the robot does not wait at rest for the plan
		 *  (PlanCrossing's work budget). An entry speed over StopSpeedCmS is
		 *  taken as moving too. */
		bool			 bOnTheMove = false;
		FRammsCurbTuning Tuning;
		/** Which centre-wheel paths the level search tries (for tests): no walk,
		 *  the walking knots, the grid search. */
		bool bTryNoWalk = true;
		bool bTryKnots = true;
		bool bTrySearch = true;
	};

	/** Seconds to move DistanceCm at up to SpeedCmS with AccelCmS2 at either
	 *  end: a trapezoid (a triangle when it is short). */
	double TrapezoidSeconds(double DistanceCm, double SpeedCmS, double AccelCmS2);

	/** How far (cm) a trapezoid move of DistanceCm has gone after Seconds. */
	double TrapezoidAt(double Seconds, double DistanceCm, double SpeedCmS, double AccelCmS2);

	/**
	 * A tilt eased from one value to another: its rate never above Rate, its
	 * rate changing no faster than Accel -- an S-curve in the tilt, its rate a
	 * trapezoid (bounded rate and acceleration; the jerk is not bounded
	 * separately). The seconds it takes, and the share of the change done
	 * after T seconds (0..1).
	 */
	double SCurveSeconds(double Delta, double Rate, double Accel);
	double SCurveShare(double T, double Delta, double Rate, double Accel);

	/** One stretch of a planned tilt: held at Tilt (deg; X pitch, Y roll) from
	 *  sample First to sample Last. */
	struct FTiltHold
	{
		int32	  First = 0;
		int32	  Last = 0;
		FVector2D Tilt = FVector2D::ZeroVector;
	};

	/** A transition between two tilt holds (or from or back to level) that does
	 *  not fit between samples First and Last at the speeds planned: it needs
	 *  NeedSeconds, and fits at no more than CapCmS over that stretch. Hold is
	 *  the hold it leads out of (-1: from level, before the first). */
	struct FTiltGap
	{
		int32  First = 0;
		int32  Last = 0;
		int32  Hold = -1;
		double NeedSeconds = 0.0;
		double CapCmS = 0.0;
	};

	/**
	 * The planned tilt over a plan's samples: each hold's tilt over its
	 * samples; between two holds back to level and out again where the time
	 * between allows both easings, else straight from one to the other
	 * (easing out of the larger early, into the larger late, or centred where
	 * the two lean apart); level before the first and after the last. Every
	 * easing is one S-curve (SCurveShare) in time along the straight way
	 * between its two tilts, pitch and roll together, SampleSeconds[k] being
	 * the time at sample k, timed by the whole turn: the base turns (pitch and
	 * roll together) no faster than Rate nor its turn rate changes faster than
	 * Accel, and the way between two tilts never inclines the base more than
	 * the more inclined of them. A
	 * transition with too little time is named in OutTight (with the speed
	 * over its stretch that would give it the time). Holds are taken in
	 * sample order; overlapping ones are a transition with no time. True when
	 * every transition fits.
	 *
	 * A transition between two holds with too little time is not hurried: it
	 * runs into the holds, over the limits never -- centred on the gap
	 * (TightModes[h] 0, the transition into hold h; the default), or ending as
	 * the later hold begins (1: into the earlier), or beginning as the earlier
	 * ends (2: into the later). Whether the holds' samples stand so is for the
	 * plan's verification.
	 */
	bool TiltProfile(TConstArrayView<double> SampleSeconds, TConstArrayView<double> SampleS, TConstArrayView<FTiltHold> Holds, double Rate,
		double Accel, TArray<FVector2D>& OutTilt, TArray<FTiltGap>* OutTight = nullptr, TConstArrayView<uint8> TightModes = {});

	/** The tilts (pitch, roll; deg) of inclination T (InclinationDeg), about
	 *  Grid (deg) apart along that ring: 4 x ceil(pi/2 x T / Grid) of them,
	 *  the four along an axis first, then by how far they lean from the
	 *  nearest axis, the diagonals last. Every one is the same total tilt;
	 *  the planner searches these rings, T on a TiltGridDeg grid, least
	 *  first. */
	TArray<FVector2D> TiltCandidates(double T, double Grid);

	/** The tilt (pitch, roll; deg) leaning the way Direction leans (its
	 *  plane's gradient's direction) with inclination InclinationDeg; zero
	 *  for a level Direction. */
	FVector2D TiltAtInclination(const FVector2D& Direction, double InclinationDeg);

	/** Of TiltCandidates(T, Grid) -- the ring of inclination T -- the one for
	 *  which Stands holds that lies furthest round the ring from one for which
	 *  it does not: the middle of the widest arc that stands, the tilt of
	 *  that inclination furthest from failing (ties, and a ring that stands
	 *  all round: the first in TiltCandidates' order). False when none
	 *  stands. */
	bool LeastInclinationAt(double T, double Grid, TFunctionRef<bool(const FVector2D&)> Stands, FVector2D& Out);

	/** The tilts (pitch, roll; deg) whose larger axis is T: the square of that
	 *  size, the four along an axis first, then leaning further from them in
	 *  Grid steps of the smaller axis, the corners last. */
	TArray<FVector2D> SquareCandidates(double T, double Grid);

	/**
	 * How a stretch's tilt of one size is picked. Ring: of the tilts inclining
	 * the base that much (TiltCandidates), the middle of the widest arc that
	 * keeps a pose (LeastInclinationAt). Square: of the tilts whose larger
	 * axis is that much (SquareCandidates), the first that keeps a pose. The
	 * two find different tilts of about the same inclination -- the square
	 * keeps one axis at a grid value and the other along an axis where it
	 * can, the ring leans furthest from failing -- and a plan that does not
	 * verify with one often does with the other (an easing that turns one
	 * axis only, a corridor at speed), so the search tries both, by the
	 * inclination they come to.
	 */
	enum class ETiltPick : uint8
	{
		Ring,
		Square
	};

	/** A tilt's size as Pick measures it (deg): its inclination (Ring), its
	 *  larger axis (Square). */
	double PickSize(ETiltPick Pick, const FVector2D& Tilt);

	/** Tilt made Size large as Pick measures it, leaning the same way. */
	FVector2D ScaledTilt(ETiltPick Pick, const FVector2D& Tilt, double Size);

	/** Pick's tilt of size Size (deg) for which Stands holds and that inclines
	 *  the base no more than MaxTilt; false when there is none. */
	bool PickTiltAt(ETiltPick Pick, double Size, double Grid, double MaxTilt, TFunctionRef<bool(const FVector2D&)> Stands, FVector2D& Out);

	/** Pick's least: sizes Grid apart, least first, the first with a tilt
	 *  (PickTiltAt) within MaxTilt; false when none. */
	bool PickLeastTilt(ETiltPick Pick, double Grid, double MaxTilt, TFunctionRef<bool(const FVector2D&)> Stands, FVector2D& Out);

	/** One candidate of the tilt search: a candidate plan's (Base: which; two
	 *  picks on one plan share it) stretches and the least tilt of each, by
	 *  one pick. */
	struct FTiltSearchCandidate
	{
		int32			  Base = 0;
		ETiltPick		  Pick = ETiltPick::Ring;
		TArray<FVector2D> Tilts;
	};

	/** One tilted plan the search makes: candidate Candidate's least tilts
	 *  (More 0), or with the stretches Larger -- those its least tilts' plan
	 *  failed about -- More (deg, as its pick measures them) larger: by its
	 *  pick (Way 0) or each least tilt's own direction made larger (Way 1).
	 *  Size: the most it inclines the base (deg). */
	struct FTiltTry
	{
		int32			  Candidate = 0;
		double			  More = 0.0;
		int32			  Way = 0;
		double			  Size = 0.0;
		TArray<FVector2D> Tilts;
		TArray<int32>	  Larger;
	};

	/** A tilt's inclination Size (deg) to a resolution: rounded up to the
	 *  Step it lies in. Inclinations within one step are alike to the search
	 *  (TiltOrderDeg: a square's tilt 0.002 deg less inclined than a ring's is
	 *  not a lesser tilt). */
	double TiltStep(double Size, double Step);

	/** Sorts tries as the search makes them: by their inclination to Step
	 *  (TiltStep; the search's TiltOrderDeg), least first; of one step a least
	 *  tilt before one made larger, then by candidate (the candidate plans in
	 *  their order, the ring before the square on each), step and way. */
	void SortTiltTries(TArray<FTiltTry>& Tries, double Step);

	/** What the tilt search came to. */
	struct FTiltSearchResult
	{
		bool	 bFound = false;
		FTiltTry Planned;
		/** The plans made (each a TiltPlan: tens to hundreds of ms), and of
		 *  them those with stretches made larger. */
		int32 Plans = 0;
		int32 LargerPlans = 0;
		/** The least inclination (deg) of a try not made because it would
		 *  incline the base more than MaxTilt; -1 when none was cut so. */
		double CutDeg = -1.0;
		/** Larger tries left unplanned at MaxPlans. */
		bool bCapped = false;
	};

	/**
	 * The tilt fallback's search, the decided chain's "least tilt": the least
	 * inclination that verifies. Every candidate's least tilts are tried, then
	 * -- once a candidate's least tilts' plan has failed -- the stretches it
	 * failed about made larger by each of Steps (deg, as the candidate's pick
	 * measures them), by its pick and by each least tilt's direction; all by
	 * the inclination they come to (to TiltOrderDeg), least first
	 * (SortTiltTries), a tilt set
	 * already planned never again, none inclining the base over MaxTilt; every
	 * candidate's least tilts, and at most MaxPlans larger tries. Stands(c, r, Tilt): whether stretch r of candidate
	 * c keeps a pose at Tilt (at the floor speed). Plan(Try, OutFailing):
	 * plans it, true when it verifies; else OutFailing, the stretches its
	 * failure is about. The first that verifies is taken.
	 */
	FTiltSearchResult SearchTilts(TConstArrayView<FTiltSearchCandidate> Candidates, double MaxTilt, double Grid, TConstArrayView<double> Steps,
		int32 MaxPlans, TFunctionRef<bool(int32, int32, const FVector2D&)> Stands, TFunctionRef<bool(const FTiltTry&, TArray<int32>&)> Plan);

	/** A wheel's keep-off window from its rolling direction
	 *  (world xy, unit) against the edge normal: climbing
	 *  [-FaceMargin - Eps, LandingOverlap + Lambda], descending
	 *  [-EdgeMargin - Lambda, DropClearance + Eps]. */
	void KeepOffWindow(double RadiusCm, double TyreWidthCm, const FVector2D& RollDirection, const FVector2D& EdgeNormal,
		bool bClimb, const FRammsCurbTuning& Tuning, double& OutUp, double& OutDown, double& OutEps, double& OutLambda);

	/** Whether the legs off the ground (bit per leg) hold a same-side corner
	 *  and centre, both fronts and a centre, or a centre and both rears: the
	 *  sets the gait never lifts together, whatever the margin says. */
	bool IsForbiddenSet(const FRobotModel& Model, uint32 Airborne);

	/** One instant as the planner asks about it. */
	/** FInstantQuery::SeatX left to the pose. */
	constexpr double FreeSeatX = -1.0e9;
	inline bool		 IsSeatFree(double SeatX)
	{
		return !(SeatX > -1.0e8);
	}

	struct FInstantQuery
	{
		/** Chassis travel (cm). */
		double S = 0.0;
		/** The base's attitude (deg). */
		double PitchDeg = 0.0;
		double RollDeg = 0.0;
		/** Off the ground; and of those on it, on the far plane (bit per leg). */
		uint32 Airborne = 0;
		uint32 OnFar = 0;
		/** Of those off the ground, the landing ones from their planned
		 *  touchdown until they have taken load: each must reach its far
		 *  surface (a leg at its reach touches down only once the chassis has
		 *  come down to it), but carries nothing yet (bit per leg). */
		uint32 Touching = 0;
		/** Each 5-bar's fore/aft (cm, chassis frame), by leg. */
		double X[MaxPlanLegs] = {};
		/** The seat's pivot fore/aft (cm, chassis frame) the instant is held
		 *  with; FreeSeatX (the default): wherever the seat keeps the pose
		 *  best (FPlanModel::PoseAt chooses, and reports it). */
		double SeatX = FreeSeatX;
	};

	/** What an instant allows: the chassis heights (world, cm) at which every
	 *  leg reaches -- support legs their surface, lifted legs their clearance
	 *  -- and the belly clears; and the pose at one of them. */
	struct FInstantResult
	{
		double Lo = 0.0;
		double Hi = 0.0;
		/** The legs that bound it from below and above (-1 the belly, -2
		 *  nothing). */
		int32 LoLeg = -2;
		int32 HiLeg = -2;
		/** The pose at B. */
		double B = 0.0;
		bool   bReach = false;
		double Margin = 0.0;
		double StaticMargin = 0.0;
		int32  Support = 0;
		double LegX[MaxPlanLegs] = {};
		double LegZ[MaxPlanLegs] = {};
		/** The seat's pivot fore/aft and the platform's pitch against the
		 *  chassis (rad) the pose was taken with (the zero pose, level, on a
		 *  robot without a seat). */
		double SeatX = 0.0;
		double SeatPitchRad = 0.0;

		double Width() const { return Hi - Lo; }
	};

	/**
	 * The per-instant model: the robot over the terrain at chassis travel S
	 * along the heading, its origin at world height B, at an attitude. Every
	 * definition is the controller's own: support legs on the
	 * plane under their own wheel through the wheel's contact (a cylinder of
	 * its contact width, its axle tilted with the chassis) less their preload;
	 * lifted legs SwingClearance over their clearance profile; the belly
	 * ChassisClearance over the terrain under the whole plate; the dynamic
	 * margin of the robot model's centre of mass against the support
	 * contacts.
	 */
	class FPlanModel
	{
	public:
		explicit FPlanModel(const FPlannerInput& In);

		int32 NumLegs() const { return Legs.Num(); }
		bool  IsCentre(int32 Leg) const { return Legs[Leg].bFiveBar; }
		bool  IsClimb() const { return bClimb; }
		/** The chassis origin (world xy) at travel S. */
		FVector2D OriginAt(double S) const { return Origin + Heading * S; }
		/** A leg's wheel centre's distance past the edge (cm) at travel S with
		 *  its wheel at fore/aft X (chassis frame), the chassis level. */
		double DistancePast(int32 Leg, double S, double X) const;
		/** The travel at which that distance is D. */
		double TravelAt(int32 Leg, double D, double X) const;
		/** The heading's share along the edge normal (cos of the approach). */
		double NormalShare() const { return CosApproach; }
		/** The world height a lifted wheel's centre needs at XY (world):
		 *  SwingClearance over its clearance profile. */
		double ClearanceZ(int32 Leg, const FVector2D& XY) const;
		/** A support wheel's centre world height at XY (world), the chassis level,
		 *  on the near or far plane, preload included. */
		double SupportZ(int32 Leg, const FVector2D& XY, bool bFar) const;
		/** A wheel's world xy at travel S, fore/aft X, chassis level. */
		FVector2D WheelXY(int32 Leg, double S, double X) const;
		/** The world height of leg Leg's wheel centre (cm) when its contact
		 *  touches the near or far plane, its centre over XY (world), the chassis
		 *  at an attitude (deg) tilting its axle: the plane under the contact
		 *  less the contact's offset from the centre. No preload. */
		double ContactCentreZ(int32 Leg, const FVector2D& XY, bool bFar, double PitchDeg = 0.0, double RollDeg = 0.0) const;
		/** A corner's fore/aft at chassis height Z (its 4-bar table). */
		double CornerX(int32 Leg, double Z) const;
		/** A corner's crank angle (rad) at chassis height Z (its 4-bar table);
		 *  0 for a 5-bar. */
		double CornerCrank(int32 Leg, double Z) const;
		/** The chassis z (cm) at which leg Leg, at fore/aft X, puts its wheel on
		 *  its surface (near or far plane, through its contact, preload
		 *  included) with the chassis origin at world height B and the base at
		 *  an attitude (deg), at travel S. */
		double LegZOnSurface(int32 Leg, double S, double X, double B, double PitchDeg, double RollDeg, bool bFar) const;
		/** The chassis z (cm) at which leg Leg, at fore/aft X, holds its wheel
		 *  centre at world height WorldZ, likewise. */
		double LegZForWorldZ(int32 Leg, double X, double WorldZ, double B, double PitchDeg, double RollDeg) const;
		/** Leg Leg's wheel centre in the world at chassis x-z (X, Z), likewise. */
		FVector WheelCentre(int32 Leg, double S, double X, double Z, double B, double PitchDeg, double RollDeg) const;
		/** The stability margin (cm) of a pose: every leg's wheel at LegXZ
		 *  (chassis x-z, one per leg), those in Support (bit per leg) on the
		 *  ground -- on the far plane those in OnFar -- with the chassis origin at
		 *  world height B and the base at an attitude (deg), at travel S. Braking
		 *  and speeding up (bDynamic: the dynamic margin), or standing still. */
		double PoseMarginAt(TConstArrayView<FVector2D> LegXZ, uint32 Support, uint32 OnFar, double S, double B, double PitchDeg, double RollDeg,
			bool bDynamic) const;
		/** A leg's reach (chassis z, cm, least and most) at fore/aft X, without
		 *  the margin: a 4-bar's table, a 5-bar's grid column. */
		FVector2D Reach(int32 Leg, double X) const;

		/** The corridor of chassis heights at the instant, the legs' fore/aft
		 *  taken at BGuess, and the pose at its middle (or at BGuess when
		 *  bAtGuess). */
		FInstantResult Evaluate(const FInstantQuery& Q, double BGuess, bool bAtGuess = false) const;
		/** The pose at chassis height B: every leg's height and fore/aft,
		 *  whether all reach, the margins. */
		void PoseAt(const FInstantQuery& Q, double B, FInstantResult& InOut) const;

		const FRammsCurbTuning&	 GetTuning() const { return Tuning; }
		const FRammsCurbTerrain& GetTerrain() const { return Terrain; }
		FVector2D				 GetHeading() const { return Heading; }
		FVector2D				 RollDirection(int32 Leg) const { return Legs[Leg].RollWorld; }
		double					 Radius(int32 Leg) const { return Legs[Leg].Radius; }
		double					 TyreWidth(int32 Leg) const { return Legs[Leg].Tyre; }
		double					 Preload(int32 Leg) const { return Legs[Leg].Preload; }
		double					 LegY(int32 Leg) const { return Legs[Leg].Y; }
		int32					 Side(int32 Leg) const { return Legs[Leg].Y > 0.0 ? 1 : 0; }
		ERammsCurbAxle			 Axle(int32 Leg) const { return Legs[Leg].Axle; }

	private:
		struct FAxes
		{
			FVector X = FVector::ForwardVector;
			FVector Y = FVector::RightVector;
			FVector Z = FVector::UpVector;
		};
		FAxes	Axes(double PitchDeg, double RollDeg) const;
		FVector ContactOffset(int32 Leg, const FAxes& A, bool bFar) const;
		double	Belly(const FAxes& A, const FVector2D& O) const;
		/** The margin of a centre of mass (chassis frame) against Contacts
		 *  (world xy, Ground their heights' sum): the dynamic margin, or with
		 *  fewer than three the hull's. */
		double MarginFor(const FVector& Com, TConstArrayView<FVector2D> Contacts, double Ground, const FAxes& A, const FVector& O3) const;
		/** Where the seat is for an instant: its pivot at SeatX (or, free,
		 *  where the pose keeps its margin best, nearest the zero pose among
		 *  equals) at the seat's crossing height, and the platform pitched
		 *  against the base's PitchDeg to keep the rider level, within its
		 *  limit. The zero pose, level, on a robot without a seat. */
		/** Touching: of Support, the legs landing that carry nothing yet (they
		 *  are not across for the seat's home). */
		void SeatAt(double PitchDeg, double SeatX, uint32 Support, uint32 OnFar, uint32 Touching, TConstArrayView<FVector2D> LegXZ,
			TConstArrayView<FVector2D> Contacts, double Ground, const FAxes& A, const FVector& O3, FVector2D& OutPivot, double& OutPitchRad) const;

		struct FLeg
		{
			bool		   bFiveBar = false;
			ERammsCurbAxle Axle = ERammsCurbAxle::Front;
			double		   Y = 0.0;
			double		   Radius = 10.0;
			double		   Contact = 6.0;
			double		   Tyre = 6.0;
			double		   Preload = 0.3;
			FVector2D	   RollLocal = FVector2D(1.0, 0.0);
			FVector2D	   RollWorld = FVector2D(1.0, 0.0);
			/** A 4-bar's table and height range; a 5-bar's grid column ranges, by
			 *  column, from GridX0 every GridDx. */
			const FFourBarTable* Table = nullptr;
			FVector2D			 TableRange = FVector2D(1.0, 0.0);
			TArray<FVector2D>	 Columns;
			double				 GridX0 = 0.0;
			double				 GridDx = 1.0;
			/** The level chassis' contact offsets on the near and far planes. */
			FVector LevelOffset[2];
		};
		const FRobotModel& Model;
		FRammsCurbTerrain  Terrain;
		FRammsCurbTuning   Tuning;
		TArray<FLeg>	   Legs;
		FVector2D		   Origin = FVector2D::ZeroVector;
		FVector2D		   Heading = FVector2D(1.0, 0.0);
		FVector2D		   Right = FVector2D(0.0, 1.0);
		double			   CosApproach = 1.0;
		bool			   bClimb = true;
		double			   Dilation = 3.0;
		FAxes			   Level;
	};

	/**
	 * Plan the crossing from where the robot is: the first plan that keeps the
	 * base level at or above the floor speed; else, stretch by stretch where
	 * level fails, the least inclination up to MaxPlannedTiltDeg everywhere; else the
	 * least tilt where it is enough and a planned stop elsewhere, or level with
	 * planned stops (Tuning.bAllowPlannedStop; at a stop the robot rests while
	 * the centre wheels and the chassis re-pose); else a refusal (Class
	 * Refused, Summary saying where, which wheels and why; a tilt refusal names
	 * MaxPlannedTiltDeg). Its samples lie on the terrain (AnchorPlanOrigin;
	 * Ramms.Curb.AnchorGrid): the crossing is planned on two grids, side by
	 * side on two threads -- the edge on a sample and half a sample past one
	 * -- and the better plan kept (IsBetterPlan), so the schedule's Origin may
	 * be up to a sample behind In.Origin, the robot that far into its plan as
	 * it begins. A grid stops early once the other's finished plan outranks
	 * anything its own stages left can give, and, on the move (In.bOnTheMove,
	 * or an entry speed over StopSpeedCmS), once its work (instants evaluated) is over
	 * OverGridBudget of the other's finished valid plan -- and a plan whose
	 * work is so over is not kept even where it finished: which grid stopped
	 * when depends on the threads' timing, which plan is kept does not.
	 * Pure: the same input gives the same plan (its lines may say how far the
	 * other grid got).
	 */
	FCurbSchedule PlanCrossing(const FPlannerInput& In);

	/** Whether a grid whose work so far is Mine is over its budget against the
	 *  other grid's finished plan of work Theirs (PlanCrossing, on the move):
	 *  Mine over Ramms.Curb.GridBudget's share (at least 1) of Theirs and
	 *  Ramms.Curb.GridBudgetWork more; never with the share 0. At most one of
	 *  two grids is ever over against the other. */
	bool OverGridBudget(int64 Mine, int64 Theirs);

	/** The plan's origin moved back along In.Heading (under SampleCm, at the
	 *  near surface's height) so that the travel from it to the edge is a whole
	 *  number of samples and PhaseShare of one: the samples then lie at the
	 *  same places on the terrain from every start. False (In unchanged) with
	 *  no terrain, the heading not towards the edge, the origin past it, or
	 *  already so. */
	bool AnchorPlanOrigin(FPlannerInput& In, double SampleCm, double PhaseShare);

	/** In with Plan's origin (and the near surface's height there): a plan's
	 *  samples are laid on the terrain, its origin up to a sample behind the
	 *  input's (PlanCrossing). A model that places the robot at the plan's
	 *  travel S (FPlanModel) is built from this. */
	void AtPlanOrigin(FPlannerInput& In, const FCurbSchedule& Plan);

	/** Of two plans of the same crossing, whether A is the better: the later
	 *  planner stage (EPlanStage: level at its own speeds, level slowed, tilt,
	 *  planned stops, refused -- the class, with a level plan that had to be
	 *  slowed ranked under one that did not), then the less inclined, the
	 *  fewer planned stops, the higher floor speed. A plan that does not say
	 *  its stage is ranked by its class. */
	bool IsBetterPlan(const FCurbSchedule& A, const FCurbSchedule& B);

	/**
	 * A planner input to a file and back (development: a plan made in the
	 * game replayed off it). The robot model, legs and their reach grids are
	 * written as they are, the tuning and the terrain by property name, so a
	 * file outlives fields added to those. False, with why, on a file that
	 * cannot be written or read, or is not one of these.
	 */
	bool SavePlannerInput(const FPlannerInput& In, const FString& Path, FString& OutWhy);
	bool LoadPlannerInput(const FString& Path, FPlannerInput& Out, FString& OutWhy);

	/** Development (not in shipping builds): where the console variable
	 *  Ramms.Curb.PlanDumpPath names a file, write In there and clear it;
	 *  OutNote says so (or why not). */
	void DumpPlannerInputIfAsked(const FPlannerInput& In, FString& OutNote);
} // namespace RammsCurb
