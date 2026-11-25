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

	/** The tilt motor's command with TrimRad taken back off it: a platform
	 *  pitch (rad, nose up) that was added to the command on top of the
	 *  rider's own setting (a level assist's seat trim, folded into the
	 *  command as a crossing took the seat). The platform's pitch at what this
	 *  returns is its pitch at TiltCommand less TrimRad, whatever the link's
	 *  angle. */
	double SeatTiltLessTrim(const FRammsCurbSeat& Seat, double TiltCommand, double TrimRad);

	/** The pitch in the world the platform is held at (rad), a step on from
	 *  RefRad toward the rider's own OwnRad: by MaxStepRad at most, and never
	 *  past it. A crossing that begins on a slope takes the seat over at the
	 *  slope's pitch; the rider is level (at their own setting) once the base
	 *  is, not held at the slope's for the crossing. */
	double SeatPitchRefStep(double RefRad, double OwnRad, double MaxStepRad);

	/** The rider's own platform pitch (rad, against the base) as a crossing
	 *  takes the seat over. CommandedRad is the pitch the seat's commands
	 *  stand for now. A crossing before this one that held KeptOwnRad (bKept)
	 *  left the seat's motors commanded to KeptCommands, its tilt making up for
	 *  the base's pitch at its last tick; where they are commanded the same
	 *  still (NowCommands, the same motors in the same order), nobody has set
	 *  the seat since and the rider's setting is the one kept, not the pitch
	 *  read back. Commanded otherwise since (a drive stance, the seat's own
	 *  controls): what is commanded now is the setting. */
	double SeatOwnPitchAtTakeOver(bool bKept, double KeptOwnRad, TConstArrayView<float> KeptCommands, TConstArrayView<float> NowCommands,
		double CommandedRad);

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
