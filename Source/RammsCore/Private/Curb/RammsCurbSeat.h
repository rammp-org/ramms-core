// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsCurbNegotiationTypes.h"

/**
 * The seat mechanism's kinematics (FRammsCurbSeat): a planar 5-bar whose
 * shared endpoint carries the platform's pivot a little further along arm
 * A's distal link, and a tilt motor at the pivot. Pure functions of the spec,
 * in the chassis' x-z plane (cm, x forward, z up), angles in radians as the
 * motors read them (the spec's AngleSign maps them to link angles).
 */
namespace RammsCurb
{
	/** Where the mechanism puts the platform's pivot for a pair of 5-bar
	 *  motor angles, and the absolute angle of arm A's distal link (the
	 *  platform turns with it, less the tilt motor). */
	struct FSeatPose
	{
		FVector2D Pivot = FVector2D::ZeroVector;
		double	  LinkAngle = 0.0;
		/** The 5-bar closes at these angles (URamms5BarKinematics::ComputeEndpoint). */
		bool bValid = false;
	};

	/** Forward kinematics: the pose at 5-bar motor angles (A, B). */
	FSeatPose SeatForward(const FRammsCurbSeat& Seat, FVector2D AnglesAB);

	/**
	 * Inverse kinematics: the 5-bar motor angles (A, B) that put the pivot
	 * at Target, found from Hint (the angles now) by Newton's method on the
	 * forward kinematics, so the solution stays on the mechanism's own
	 * assembly branch. False when it does not converge, closes nowhere, or
	 * a knee leaves the spec's DistalRangeA/B (URamms5BarKinematics::SolveIK
	 * at the endpoint found); OutAngles then holds the last try.
	 */
	bool SeatSolve(const FRammsCurbSeat& Seat, FVector2D Target, FVector2D Hint, FVector2D& OutAngles, FSeatPose* OutPose = nullptr);

	/** The platform's pitch relative to the chassis (rad, nose up positive)
	 *  with arm A's distal link at LinkAngle, against LinkAngle0 (its angle at
	 *  the zero pose, where the platform is level) and the tilt motor at
	 *  TiltAngle (0 without a tilt motor). */
	double SeatPlatformPitch(const FRammsCurbSeat& Seat, double LinkAngle, double LinkAngle0, double TiltAngle);

	/** The tilt motor angle that pitches the platform PitchRad relative to
	 *  the chassis with the link at LinkAngle (the inverse of SeatPlatformPitch). */
	double SeatTiltFor(const FRammsCurbSeat& Seat, double LinkAngle, double LinkAngle0, double PitchRad);

	/**
	 * The fore/aft the pivot can be held at while at height Z (cm, chassis
	 * frame): the least and most x the mechanism reaches there, scanned at
	 * StepCm from the pivot's zero-pose x (Hint the angles there), within the
	 * motors' ranges when given (robot sense; min >= max: unbounded). bValid
	 * false when it does not reach Z at the zero-pose x at all.
	 */
	FVector2D SeatReachX(const FRammsCurbSeat& Seat, double Z, FVector2D Hint, FVector2D RangeA, FVector2D RangeB, bool& bValid,
		double StepCm = 1.0, double SpanCm = 80.0);
} // namespace RammsCurb
