// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Ramms4BarLegSpec.h"

/**
 * How the robot's weight shares out among the wheels carrying it, and what a
 * leg's motors put out to carry its share: plain C++, so each piece can be
 * tested on its own. Not reflected, not exported.
 *
 * Signs: a leg's Jacobian is how far (cm) its wheel centre moves per radian of
 * a motor, in the chassis x-z frame (x forward, z up), with the motor's angle
 * in the robot's sense (the sense its commands and readings take). A motor
 * holding its wheel against a ground force F (on the wheel, z up) puts out
 * tau = -J^T F / 100 (N*m from N and cm/rad): carrying N newtons the wheel's
 * motor pushes with -Jz * N / 100.
 */
namespace RammsCurb
{
	/** How far (cm, x and z) a 4-bar's wheel centre moves per radian of its
	 *  crank at CrankRad: a central difference over +-0.005 rad, one-sided
	 *  within that of an end of its range. False where the loop does not close
	 *  on either side. */
	bool FourBarWheelRate(const FRamms4BarLegSpec& Spec, double CrankRad, FVector2D& OutCmPerRad);

	/**
	 * The torque (N*m) one leg motor needs to press its wheel onto the ground
	 * with ForceCapN and drive it along with TractionRatio of that:
	 * (|Jz| F + (|Jx| + Reaction) Ratio F) / 100, from its Jacobian column
	 * (Jx, Jz; cm/rad) and, for a leg whose motor turns as its wheel drives, the
	 * reaction of that drive (ReactionCmPerRad: the wheel's radius times how far
	 * the motor turns per cm the wheel rolls). OutPressSign is the sense (+1 or
	 * -1) the motor turns in to press its wheel down. False where the motor has
	 * less vertical lever than MinJzCmPerRad: it does not carry the wheel, and
	 * is left alone.
	 */
	bool PressTorqueCap(double JxCmPerRad, double JzCmPerRad, double ForceCapN, double TractionRatio, double ReactionCmPerRad,
		double MinJzCmPerRad, double& OutCapNm, int32& OutPressSign);

	/**
	 * The share of WeightN each of Contacts (horizontal, cm) carries with the
	 * centre of mass over Com: the statics with every wheel's leg as stiff as
	 * the others' (the least-squares shares: W/n plus a plane through the
	 * contacts, centred on them), a negative share dropped (its wheel carrying
	 * nothing) and the rest solved again. Contacts on one line share along it
	 * (the centre of mass must lie within 1 cm of the line). False with fewer
	 * than two contacts or with the centre of mass outside them (no share
	 * pattern carries it without pulling a wheel down).
	 */
	bool MinNormLoadShares(TConstArrayView<FVector2D> Contacts, const FVector2D& Com, double WeightN, TArray<double>& OutShares);
} // namespace RammsCurb
