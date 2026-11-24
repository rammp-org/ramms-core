// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Level/RammsLevelTypes.h"

/**
 * The level assist's geometry: which legs stand, the plane through a set of
 * leg heights, and how much of a wanted correction plane the legs' travel and
 * the robot's clearances allow. Plain C++, no state: every function is a pure
 * function of its arguments. Units and signs as RammsLevelTypes.h.
 *
 * THE CONSTRAINT SET, shared by Allocate and AllocateFraction. For a share s
 * (0..1) of the wanted plane and a ride-height shift r (cm, positive raises
 * the chassis):
 *
 *   P       = s * ClampAxes(CorrectionDeg, Tuning.MaxCorrectionDeg)
 *   rise_i  = PlaneRise(Legs[i].XY, Legs[i].bRoll, P)
 *   m       = the mean of rise_i over the support legs (over every leg when
 *             none is a support leg; 0 with no legs)
 *   want_i  = rise_i - m - r
 *
 * so that with r = 0 the support legs' mean offset is zero and the stance's
 * ride height is kept. With M = Tuning.LegRangeMarginCm + ExtraMarginCm:
 *
 *  - a SUPPORT leg must have Lo_i <= want_i <= Hi_i, where
 *    Lo_i = min(0, MinZ + M - BaseZ) and Hi_i = max(0, MaxZ - M - BaseZ).
 *    The min / max with 0 mean a leg whose baseline is already inside its
 *    margin is not a fault: it may stay, but not go further that way.
 *  - a LIFTED leg (bSupport false) follows the plane as far as its travel
 *    goes and stops there without limiting anything, until what it falls
 *    short by would bring its clearance under MinLiftedClearanceCm:
 *    want_i <= max(0, MaxZ - BaseZ) + max(0, BaselineClearanceCm -
 *    MinLiftedClearanceCm). No lower limit (a lifted leg that cannot extend
 *    as far as the plane asks only ends up higher).
 *  - a CLEARANCE POINT k: the ground under it, in the chassis frame, is taken
 *    to be the plane FitPlane puts through the support legs' baseline wheel
 *    bottoms (BaseZ - RadiusCm; every leg's roll counted), g0_k its height at
 *    the point. The correction raises that ground at the point by
 *    PlaneRise(XY_k, true, P) - m - r, which must be at most
 *    max(0, UndersideZ_k - g0_k - MinCm_k). No constraint with no support leg.
 *  - -Tuning.RideHeightDownCm <= r <= Tuning.RideHeightUpCm.
 *
 * A share s FITS when some r satisfies all of these, to within 1e-9 cm (a
 * plane that puts a leg exactly on its margin fits, whatever the last bit of
 * the arithmetic says). s = 0 with r = 0 always fits. The largest share is 1
 * when 1 fits, else the lower end after AllocateBisections halvings of [0, 1]
 * (lower end fits, upper does not), so it is always a share that fits.
 *
 * Negative RideHeightUpCm, RideHeightDownCm and MaxRideStepCm count as 0.
 * The ground under the clearance points comes from FitPlane's own rules, so
 * with one or two support legs it is level across the direction they do not
 * span.
 */
namespace RammsLevel
{
	/**
	 * Which legs stand: fills bSupport, BaselineClearanceCm and CarryShare of
	 * every leg. A leg's wheel bottom is BaseZ - RadiusCm; Low is the lowest of
	 * them. BaselineClearanceCm = bottom - Low. A leg that was a support leg (Was[i])
	 * stays one while that is at most Tuning.SupportBandOutCm; one that was
	 * not becomes one at Tuning.SupportBandInCm or less. With Was empty (or
	 * not the legs' length) there is no memory and every leg is judged by the
	 * In band. Only baseline heights count: an offset never changes who stands.
	 * (A leg that stood stays within the larger of the two bands, so a tuning
	 * with Out under In cannot drop a leg inside the In band.)
	 *
	 * CarryShare = clamp(1 - BaselineClearanceCm / SupportBandInCm, 0, 1)
	 * (with no band: 1 on the lowest bottom, 0 above it). It takes no memory,
	 * so it never steps, and it is gone by the INNER band: a leg joins the
	 * support set, and leaves it, with a share of 0. The set is kept wide so
	 * that who stands does not flap; the share is narrow because a leg drawn
	 * up off the ground is in the air from the first of its lift.
	 */
	void SupportMask(TArrayView<FLegReach> Legs, TConstArrayView<bool> Was, const FTuning& Tuning);

	/**
	 * A leg whose wheel is still ABOVE where it is told to be carries less:
	 * each leg's CarryShare (as SupportMask left it) less L / SupportBandInCm,
	 * not under 0. With late_i = max(0, MeasuredZ - (BaseZ + Dz[i])) (cm; Dz
	 * the offsets the legs were last told, 0 for a leg past its end), L is
	 * late_i less the least late_j among the legs whose share is over 0: what
	 * they are all late by (a sag under the load, a stance being lowered) is
	 * no leg's lateness.
	 *
	 * A leg being set down has not reached the ground when its baseline has:
	 * until its joint has come down too it stands on nothing, and where it
	 * is says nothing of the base. (A wheel BELOW where it is told, a leg
	 * late in lifting, is left alone: it is on the ground for as long as it
	 * is late, and its baseline's share is already going.) With no band a
	 * late leg's share is 0.
	 */
	void DiscountLateLegs(TArrayView<FLegReach> Legs, TConstArrayView<double> Dz, const FTuning& Tuning);

	/**
	 * The least-squares plane through Values (cm) at XY (chassis cm):
	 * value = x tan(pitch) + y' tan(roll) + height, y' being y for a point
	 * roll reaches and 0 for one it does not. Only points with Use[i] count
	 * (Use empty: all); Roll[i] says whether roll reaches point i (Roll empty:
	 * all). OutDeg is (pitch, roll) in degrees, OutHeightCm the plane's value
	 * at the chassis origin.
	 *
	 * The inverse of Allocate's rule: fed XY and PlaneRise(XY, bRoll, C) + h
	 * for three or more points that are not in line, it returns C and h.
	 * Anything that is not a plane (a common height aside, a warp, one leg
	 * lifted) is left out as the residual.
	 *
	 * Where the points do not span a direction (fewer than three, all in
	 * line, or no point roll reaches) the slope across the unspanned direction
	 * is 0: the minimum-norm solution, taken about the points' centroid, a
	 * direction counting as unspanned when its spread is under 1e-6 of the
	 * larger one or under 1e-9 cm^2. One point gives a level plane at its
	 * value. False, with zeros, only when no point counts. A Use shorter than
	 * the points leaves the points past its end out; a Roll shorter than the
	 * points takes those past its end as reached by roll.
	 */
	bool FitPlane(TConstArrayView<FVector2D> XY, TConstArrayView<double> Values, TConstArrayView<bool> Use,
		TConstArrayView<bool> Roll, FVector2D& OutDeg, double& OutHeightCm);

	/**
	 * The correction the joints have actually achieved (deg): the plane
	 * through MeasuredZ - BaseZ of the legs that CARRY the robot, each weighed
	 * by the square of its CarryShare, each leg's bRoll honoured, the height dropped. Servo
	 * sag, rate limits and travel stops of the legs the robot stands on are in
	 * it. A leg carried clear of the ground is not: where it is changes nothing
	 * about the base's attitude, and one that rests on its retract stop, or
	 * lags a stance ramp, would bend a plane fitted through every leg (the
	 * terrain estimate then reads short of the ground). Legs must have been
	 * through SupportMask, and DiscountLateLegs where the offsets are known
	 * (or carry a CarryShare from whoever gave the set).
	 *
	 * It is FitPlane's least-squares plane with those weights (the square,
	 * because a leg with leverage over the rest keeps most of its say down to
	 * a small weight: squared, a leg being lifted gives it up more evenly), and
	 * FitPlane's rule for a direction the points do not span, but stricter: a
	 * direction counts as spanned by the carrying legs when its weighted
	 * spread is at least 1e-2 of the larger one (the legs a tenth as wide as
	 * they are long) and 1e-9 cm^2. Along a direction they do not span (one
	 * carrying leg, two, several all but in line) the slope is that of
	 * FitPlane's plane through EVERY leg, unweighted: what the legs were all
	 * told, as far as they have followed it. With no share over 0 it is that
	 * plane outright. Zero with no legs.
	 */
	FVector2D MeasuredCorrection(TConstArrayView<FLegReach> Legs);

	/**
	 * Each leg's offset for the whole plane Deg with nothing limiting it:
	 * want_i of the constraint set at s = 1, r = 0, WITHOUT the per-axis
	 * clamp (the caller's plane is taken as given). Out is set to the legs'
	 * length. What the correction slew measures leg rates and positions with.
	 */
	void RigidOffsets(TConstArrayView<FLegReach> Legs, const FVector2D& Deg, FLegValues& Out);

	/**
	 * The largest share (0..1) of CorrectionDeg that fits the constraint set,
	 * with every support leg's limits pulled in by a further ExtraMarginCm.
	 * The ride shift is free within its allowance (no slew). Legs must have
	 * been through SupportMask.
	 */
	double AllocateFraction(TConstArrayView<FLegReach> Legs, TConstArrayView<FClearancePoint> Points, const FVector2D& CorrectionDeg,
		const FTuning& Tuning, double ExtraMarginCm);

	/**
	 * Every leg's offset for the wanted plane CorrectionDeg. Legs must have
	 * been through SupportMask.
	 *
	 * The constraint set with ExtraMarginCm 0 and one more limit on r: it may
	 * not be further than MaxRideStepCm (0 or more) from PrevRideCm, the shift
	 * of the tick before. Fraction is the largest share that fits; HeightCm
	 * the r that fits it nearest 0 (so a shift that is no longer needed goes
	 * back at the slew rate). Should not even s = 0 fit (the slew window does
	 * not yet reach an r the limits allow), Fraction is 0 and HeightCm the end
	 * of the slew window, cut to the ride allowance, nearest the r the limits
	 * want: the rate limit wins and the hard clamp below keeps the legs in
	 * their travel.
	 *
	 * Dz[i] = clamp(want_i, min(0, MinZ - BaseZ), max(0, MaxZ - BaseZ)) at
	 * that s and r, for every leg: a rigid plane over all of them, but for a
	 * lifted leg stopped at its travel (and the last-resort clamp). AppliedDeg
	 * = Fraction * ClampAxes(CorrectionDeg, MaxCorrectionDeg).
	 *
	 * With Fraction < 1, LimitingLeg / LimitingPoint name the constraint with
	 * the largest violation at the last share the search found not to fit
	 * (at the r nearest to fitting: midway between the two limits that clash,
	 * kept inside the ride window; an upper limit wins an exact tie, and both
	 * stay INDEX_NONE when only the window is at fault); a leg at Fraction < 1
	 * then sits on its margin to within the search's resolution, and HeightCm,
	 * where the slew window is what binds, within about 0.001 cm of the
	 * window's end. MinMarginCm = TravelMargin(Legs,
	 * Dz). ReachLeft = the least, over support legs with |Dz| over
	 * TravelMarginMinMoveCm, of 1 - Dz / Hi_i (Dz > 0) or 1 - Dz / Lo_i
	 * (Dz < 0), within 0..1, 0 where that limit is 0; 1 with no such leg.
	 * RateUse is left 0.
	 */
	FAllocation Allocate(TConstArrayView<FLegReach> Legs, TConstArrayView<FClearancePoint> Points, const FVector2D& CorrectionDeg,
		const FTuning& Tuning, double PrevRideCm, double MaxRideStepCm);

	/**
	 * The least travel (cm, 0 or more) a support leg has left, to its true
	 * limit (no margin), in the direction its offset moved it: MaxZ - (BaseZ +
	 * Dz) for Dz > 0, (BaseZ + Dz) - MinZ for Dz < 0. A leg moved less than
	 * TravelMarginMinMoveCm is not counted: a leg whose stance rests it on a
	 * stop, and is not asked further, is not "at reach". NoTravelMarginCm when
	 * no leg counts.
	 */
	double TravelMargin(TConstArrayView<FLegReach> Legs, TConstArrayView<double> Dz);
} // namespace RammsLevel
