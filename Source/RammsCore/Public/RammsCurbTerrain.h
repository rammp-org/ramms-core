// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsCurbTerrain.generated.h"

/**
 * One surface of a curb crossing, as a plane over the horizontal: its height
 * at a point p is Origin.Z + Gradient . (p - Origin), the gradient being
 * (dz/dx, dz/dy). World frame, cm.
 *
 * See FRammsCurbTerrain, which holds two of them.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsCurbPlane
{
	GENERATED_BODY()

	/** A point on the plane: the middle of the samples it was fitted to. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FVector Origin = FVector::ZeroVector;

	/** dz/dx and dz/dy (cm per cm): the plane's slope, world frame. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FVector2D Gradient = FVector2D::ZeroVector;

	/** RMS distance (cm), vertically, of the samples the plane was fitted
	 *  to from it -- those within the fit's residual limit; see
	 *  FRammsCurbTuning::MaxPlaneResidualCm. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	float FitRmsCm = 0.0f;

	/** The largest of those distances (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	float FitMaxCm = 0.0f;

	/** How many samples the fit kept. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	int32 Samples = 0;

	/** Height of the plane over the point P (world xy, cm). */
	double HeightAt(const FVector2D& P) const
	{
		return Origin.Z + Gradient.X * (P.X - Origin.X) + Gradient.Y * (P.Y - Origin.Y);
	}

	/** The plane's steepest slope (deg). */
	double SlopeDeg() const { return FMath::RadiansToDegrees(FMath::Atan(Gradient.Size())); }
};

/**
 * The ground a curb crossing goes over: a near surface the robot stands on,
 * a far one beyond a straight edge, each a plane, and the edge between them
 * at any angle to the robot's heading. World frame, cm, X forward, Y right,
 * Z up. Found by URammsCurbNegotiationController::DetectTerrainAhead.
 *
 * A point p is d(p) = EdgeNormal . (p - EdgePoint) past the edge (negative
 * short of it) and u(p) = EdgeDirection . (p - EdgePoint) along it. The
 * surface under p is Near's where d < 0 and Far's beyond: the curb face
 * between them is taken to be vertical. The step at u along the edge is
 * Far - Near there, which changes linearly along the edge when the two planes
 * slope differently across it.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsCurbTerrain
{
	GENERATED_BODY()

	/** False until a detection (or a plan made without one) has filled it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	bool bValid = false;

	/** Where the robot's centreline, as it was found, crosses the edge. Its
	 *  Z is the near surface there. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FVector EdgePoint = FVector::ZeroVector;

	/** Unit vector along the edge (world xy), EdgeNormal turned 90 deg
	 *  towards +Y. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FVector2D EdgeDirection = FVector2D(0.0, 1.0);

	/** Unit vector across the edge (world xy), from the near side to the
	 *  far: the way the robot crosses it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FVector2D EdgeNormal = FVector2D(1.0, 0.0);

	/** The angle (deg) from the robot's heading, as it was found, to
	 *  EdgeNormal: 0 square on; positive when the robot's +Y wheels meet the
	 *  edge first. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	float ApproachAngleDeg = 0.0f;

	/** How far (cm) the edge points the scan found lie from the straight
	 *  line fitted through them, at most. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	float EdgeFitResidualCm = 0.0f;

	/** The surface on the robot's side of the edge. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FRammsCurbPlane Near;

	/** The surface beyond the edge. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FRammsCurbPlane Far;

	/** Far less Near at EdgePoint (cm): positive for a curb to climb,
	 *  negative for one to descend. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	float StepHeightCm = 0.0f;

	/** The least and greatest step (cm, signed as StepHeightCm) along the
	 *  stretch of edge the robot's footprint crosses. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FVector2D StepHeightRangeCm = FVector2D::ZeroVector;

	/** How deep (cm, along EdgeNormal) the far surface was seen to go: to a
	 *  second edge where one was found (bFarDepthObserved), else to the end
	 *  of what the scan could see, so at least this deep. The least over the
	 *  scan's lanes. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	float FarDepthCm = 0.0f;

	/** FarDepthCm ends at a second edge that was seen, not at the end of the
	 *  scan. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	bool bFarDepthObserved = false;

	/** d(p): how far the point P (world xy) is past the edge (cm). */
	double DistancePastEdge(const FVector2D& P) const
	{
		return FVector2D::DotProduct(EdgeNormal, P - FVector2D(EdgePoint.X, EdgePoint.Y));
	}

	/** u(p): how far along the edge the point P is from EdgePoint (cm). */
	double AlongEdge(const FVector2D& P) const
	{
		return FVector2D::DotProduct(EdgeDirection, P - FVector2D(EdgePoint.X, EdgePoint.Y));
	}

	/** Height of the surface under P (world xy, cm). */
	double SurfaceAt(const FVector2D& P) const { return DistancePastEdge(P) < 0.0 ? Near.HeightAt(P) : Far.HeightAt(P); }

	/** The step (cm) at U along the edge from EdgePoint. */
	double StepAt(double U) const
	{
		const FVector2D P = FVector2D(EdgePoint.X, EdgePoint.Y) + EdgeDirection * U;
		return Far.HeightAt(P) - Near.HeightAt(P);
	}
};
