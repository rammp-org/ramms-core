// Copyright Epic Games, Inc. All Rights Reserved.

#include "Ramms5BarKinematics.h"

FVector2D URamms5BarKinematics::KneePosition(
	FVector2D Pivot, double ProximalLen, double ZeroDir, double Sign, double JointAngle)
{
	const double LinkAngle = ZeroDir + Sign * JointAngle;
	return Pivot + FVector2D(ProximalLen * FMath::Cos(LinkAngle), ProximalLen * FMath::Sin(LinkAngle));
}

double URamms5BarKinematics::SolveArm(
	FVector2D Pivot, double ProximalLen, double DistalLen,
	double ZeroDir, double Sign, bool bElbowUp, FVector2D Target, bool& bReachable)
{
	const FVector2D ToTarget = Target - Pivot;
	const double	Dist = ToTarget.Size();
	const double	Base = FMath::Atan2(ToTarget.Y, ToTarget.X);

	bReachable = true;

	// Degenerate: target on the pivot — cannot define a direction.
	if (Dist < KINDA_SMALL_NUMBER)
	{
		bReachable = false;
		return (0.0 - ZeroDir) / Sign;
	}

	// Interior angle at the pivot between (pivot->target) and the proximal link.
	double CosInterior = (ProximalLen * ProximalLen + Dist * Dist - DistalLen * DistalLen)
		/ (2.0 * ProximalLen * Dist);
	if (CosInterior < -1.0 || CosInterior > 1.0)
	{
		// Out of reach (too far or too close): clamp so the arm points as close
		// as it can — straight at the target (far) or folded back (near).
		bReachable = false;
		CosInterior = FMath::Clamp(CosInterior, -1.0, 1.0);
	}
	const double Interior = FMath::Acos(CosInterior);

	const double LinkAngle = Base + (bElbowUp ? Interior : -Interior);
	// Joint angles are reported unwrapped near zero by the physics backends, so
	// return the principal value rather than an aliased multiple-of-2π branch.
	return FMath::UnwindRadians((LinkAngle - ZeroDir) / Sign);
}

FVector2D URamms5BarKinematics::SolveIK(const FRamms5BarLinkageSpec& Spec, FVector2D TargetXZ, bool& bReachable)
{
	bool		 bReachA = false;
	bool		 bReachB = false;
	const double AngleA = SolveArm(Spec.PivotA, Spec.ProximalLengthA, Spec.DistalLengthA,
		Spec.ZeroDirA, Spec.AngleSignA, Spec.bElbowUpA, TargetXZ, bReachA);
	const double AngleB = SolveArm(Spec.PivotB, Spec.ProximalLengthB, Spec.DistalLengthB,
		Spec.ZeroDirB, Spec.AngleSignB, Spec.bElbowUpB, TargetXZ, bReachB);
	bReachable = bReachA && bReachB;
	return FVector2D(AngleA, AngleB);
}

FVector2D URamms5BarKinematics::ComputeEndpoint(const FRamms5BarLinkageSpec& Spec, FVector2D JointAnglesAB, bool& bValid)
{
	const FVector2D KneeA = KneePosition(Spec.PivotA, Spec.ProximalLengthA, Spec.ZeroDirA, Spec.AngleSignA, JointAnglesAB.X);
	const FVector2D KneeB = KneePosition(Spec.PivotB, Spec.ProximalLengthB, Spec.ZeroDirB, Spec.AngleSignB, JointAnglesAB.Y);

	// Endpoint = intersection of circle(KneeA, DistalA) and circle(KneeB, DistalB).
	const FVector2D Delta = KneeB - KneeA;
	const double	CenterDist = Delta.Size();
	const double	Ra = Spec.DistalLengthA;
	const double	Rb = Spec.DistalLengthB;

	bValid = true;
	if (CenterDist < KINDA_SMALL_NUMBER || CenterDist > Ra + Rb || CenterDist < FMath::Abs(Ra - Rb))
	{
		// Distal links can't meet: return the midpoint along the knee-to-knee
		// line at the proportional radius (closest consistent approximation).
		bValid = false;
		if (CenterDist < KINDA_SMALL_NUMBER)
		{
			return KneeA;
		}
		const double AClosest = (Ra * Ra - Rb * Rb + CenterDist * CenterDist) / (2.0 * CenterDist);
		return KneeA + Delta * (AClosest / CenterDist);
	}

	const double	A = (Ra * Ra - Rb * Rb + CenterDist * CenterDist) / (2.0 * CenterDist);
	const double	H = FMath::Sqrt(FMath::Max(0.0, Ra * Ra - A * A));
	const FVector2D Mid = KneeA + Delta * (A / CenterDist);
	// Perpendicular to the knee-to-knee line.
	const FVector2D Perp = FVector2D(-Delta.Y, Delta.X) / CenterDist;

	// The two intersections are mirror images across the knee-to-knee line, and
	// the assembled mechanism stays on one side of it (crossing needs the distal
	// links collinear). Pick by fixed chirality — the sign of
	// cross(KneeB - KneeA, Endpoint - KneeA) — rather than by height, which
	// would flip the answer when the knees tilt far enough.
	const FVector2D Endpoint = Spec.bFlipEndpointSide ? (Mid - Perp * H) : (Mid + Perp * H);

	// Feasibility: each knee bends one way only (the passive knee joints have a
	// hard stop at straight), so a hip pair whose geometric solution needs a
	// knee bent past straight is not a pose the closed loop can hold. That
	// bend direction is exactly the IK elbow branch: elbow-up ⇔
	// cross(proximal dir, distal dir) < 0.
	auto KneeBendMatches = [&Endpoint](FVector2D Pivot, FVector2D Knee, bool bElbowUp) {
		const FVector2D Prox = Knee - Pivot;
		const FVector2D Dist = Endpoint - Knee;
		const double	Cross = Prox.X * Dist.Y - Prox.Y * Dist.X;
		// A straight knee (the reach boundary IK treats as reachable) is valid
		// for either bend direction; tolerance scales with the link lengths
		// since the cross product is an area.
		const double StraightTolerance = 1e-6 * Prox.Size() * Dist.Size();
		if (FMath::Abs(Cross) <= StraightTolerance)
		{
			return true;
		}
		return (Cross < 0.0) == bElbowUp;
	};
	if (!KneeBendMatches(Spec.PivotA, KneeA, Spec.bElbowUpA) || !KneeBendMatches(Spec.PivotB, KneeB, Spec.bElbowUpB))
	{
		bValid = false;
	}
	return Endpoint;
}
