// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbSafety.h"

namespace RammsCurb
{
	namespace
	{
		/** The upward normal of the plane a chassis at this pitch and roll
		 *  (deg) lies in: it rises tan(pitch) along its heading and tan(roll)
		 *  across it (the frame InclinationDeg measures). */
		FVector AttitudeNormal(const FVector2D& Deg)
		{
			const double Tp = FMath::Tan(FMath::DegreesToRadians(Deg.X));
			const double Tr = FMath::Tan(FMath::DegreesToRadians(Deg.Y));
			return FVector(-Tp, -Tr, 1.0).GetSafeNormal();
		}
	} // namespace

	double AttitudeErrorDeg(const FVector2D& MeasuredDeg, const FVector2D& PlannedDeg)
	{
		const FVector A = AttitudeNormal(MeasuredDeg);
		const FVector B = AttitudeNormal(PlannedDeg);
		return FMath::RadiansToDegrees(FMath::Atan2(FVector::CrossProduct(A, B).Size(), FVector::DotProduct(A, B)));
	}

	FTerrainCheck CheckTerrain(const FRammsCurbTerrain& Terrain, TConstArrayView<FTerrainProbe> Probes, const FVector2D& BiasCm)
	{
		FTerrainCheck						 Out;
		TArray<double, TInlineAllocator<32>> Off[2];
		const double						 FarEnd = Terrain.FarDepthCm > 0.0f ? Terrain.FarDepthCm - TerrainFaceBandCm : TNumericLimits<double>::Max();
		for (const FTerrainProbe& P : Probes)
		{
			const double D = Terrain.DistancePastEdge(P.XY);
			if (FMath::Abs(D) < TerrainFaceBandCm || D > FarEnd)
			{
				continue;
			}
			const int32	 Plane = D >= 0.0 ? 1 : 0;
			const double Surface = (Plane == 1 ? Terrain.Far : Terrain.Near).HeightAt(P.XY) + (Plane == 1 ? BiasCm.Y : BiasCm.X);
			double		 E = 0.0;
			if (P.bHit)
			{
				E = P.Z - Surface;
			}
			else if (Surface <= P.AboveZ && Surface >= P.BelowZ)
			{
				// Nothing under it as far down as the trace went, where its surface
				// should have been: a drop at least that deep.
				E = P.BelowZ - Surface;
			}
			else
			{
				continue;
			}
			Off[Plane].Add(E);
			if (Out.WorstLeg == INDEX_NONE || FMath::Abs(E) > FMath::Abs(Out.WorstCm))
			{
				Out.WorstCm = E;
				Out.WorstLeg = P.Leg;
				Out.WorstAheadCm = P.AheadCm;
				Out.WorstPlane = Plane;
			}
		}
		for (int32 Plane = 0; Plane < 2; ++Plane)
		{
			Out.Probes[Plane] = Off[Plane].Num();
			if (Off[Plane].Num() > 0)
			{
				Off[Plane].Sort();
				const int32 N = Off[Plane].Num();
				Out.MedianCm[Plane] = N % 2 == 1 ? Off[Plane][N / 2] : 0.5 * (Off[Plane][N / 2 - 1] + Off[Plane][N / 2]);
			}
		}
		return Out;
	}

	double RimGap(int32 Leg, const FVector& CentreWorld, double RadiusCm, const FVector2D& RollDirection, double TyreWidthCm,
		TConstArrayView<FTerrainProbe> Probes)
	{
		const FVector2D F = RollDirection.GetSafeNormal();
		const FVector2D A(-F.Y, F.X);
		const FVector2D C(CentreWorld.X, CentreWorld.Y);
		double			Gap = TNumericLimits<double>::Max();
		for (const FTerrainProbe& P : Probes)
		{
			if (P.Leg != Leg || !P.bHit)
			{
				continue;
			}
			const double U = FVector2D::DotProduct(P.XY - C, F);
			const double V = FVector2D::DotProduct(P.XY - C, A);
			if (FMath::Abs(U) >= RadiusCm || FMath::Abs(V) > 0.5 * TyreWidthCm + 1e-6)
			{
				continue;
			}
			const double Rim = CentreWorld.Z - FMath::Sqrt(FMath::Max(0.0, RadiusCm * RadiusCm - U * U));
			Gap = FMath::Min(Gap, Rim - P.Z);
		}
		return Gap;
	}
} // namespace RammsCurb
