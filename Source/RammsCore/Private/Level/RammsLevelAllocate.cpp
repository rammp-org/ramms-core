// Copyright Epic Games, Inc. All Rights Reserved.

#include "Level/RammsLevelAllocate.h"

namespace RammsLevel
{
	// Named, not anonymous: in a unity build the other Level files share this
	// translation unit, and their helpers may well be called the same.
	namespace AllocateImpl
	{
		/** Two limits that meet to within this (cm) count as meeting: a plane
		 *  that puts a leg exactly on its margin must fit, whatever the last
		 *  bit of the arithmetic says. */
		constexpr double FitSlackCm = 1.0e-9;
		/** "No limit" for a ride shift (cm). */
		constexpr double UnboundedCm = 1.0e30;

		/** FitPlane's rank tolerance: a direction is spanned when its spread
		 *  is at least this share of the larger one, and this much (cm^2). */
		constexpr double SpreadRatioMin = 1.0e-6;
		constexpr double SpreadMinCm2 = 1.0e-9;

		/** The mean rise of the plane over the support legs (over every leg
		 *  when none stands; 0 with no legs): what is taken off every leg so
		 *  the stance keeps its ride height. */
		double MeanRise(TConstArrayView<FLegReach> Legs, const FVector2D& PlaneDeg)
		{
			double SupportSum = 0.0;
			double AllSum = 0.0;
			int32  SupportCount = 0;
			for (const FLegReach& Leg : Legs)
			{
				const double Rise = PlaneRise(Leg.XY, Leg.bRoll, PlaneDeg);
				AllSum += Rise;
				if (Leg.bSupport)
				{
					SupportSum += Rise;
					++SupportCount;
				}
			}
			if (SupportCount > 0)
			{
				return SupportSum / SupportCount;
			}
			return Legs.Num() > 0 ? AllSum / Legs.Num() : 0.0;
		}

		/** A support leg's limits on its offset (cm), its travel pulled in by
		 *  MarginCm. Widened to hold 0: a baseline already inside the margin
		 *  may stay where it is, it only may not go further that way. */
		double SupportLow(const FLegReach& Leg, double MarginCm)
		{
			return FMath::Min(0.0, Leg.MinZ + MarginCm - Leg.BaseZ);
		}

		double SupportHigh(const FLegReach& Leg, double MarginCm)
		{
			return FMath::Max(0.0, Leg.MaxZ - MarginCm - Leg.BaseZ);
		}

		/** One search's fixed part: the legs, the points, the whole plane
		 *  (after the per-axis clamps) and how far each point's ground may rise. */
		struct FProblem
		{
			TConstArrayView<FLegReach>		 Legs;
			TConstArrayView<FClearancePoint> Points;
			FVector2D						 FullDeg = FVector2D::ZeroVector;
			double							 MarginCm = 0.0;
			double							 MinLiftedClearanceCm = 0.0;
			/** Per point: the most its ground may rise (cm, 0 or more). Empty
			 *  with no support leg: there is then no ground to keep clear of. */
			FLegValues PointCap;
		};

		/** The ride shifts r one share of the plane allows, [Lo, Hi], and
		 *  which constraint sets each end. Every constraint is "want <= U"
		 *  (r >= a - U, an end of Lo) or "want >= L" (r <= a - L, an end of Hi). */
		struct FRideSpan
		{
			double Lo = -UnboundedCm;
			double Hi = UnboundedCm;
			int32  LoLeg = INDEX_NONE;
			int32  LoPoint = INDEX_NONE;
			int32  HiLeg = INDEX_NONE;
		};

		FProblem MakeProblem(TConstArrayView<FLegReach> Legs, TConstArrayView<FClearancePoint> Points, const FVector2D& CorrectionDeg,
			const FTuning& Tuning, double ExtraMarginCm)
		{
			FProblem Problem;
			Problem.Legs = Legs;
			Problem.Points = Points;
			Problem.FullDeg = ClampAxes(CorrectionDeg, Tuning.MaxCorrectionDeg);
			Problem.MarginCm = Tuning.LegRangeMarginCm + ExtraMarginCm;
			Problem.MinLiftedClearanceCm = Tuning.MinLiftedClearanceCm;

			// The ground under the belly, in the chassis frame: the plane
			// through the standing wheels' bottoms at their baselines. Every
			// leg's roll counts here: this is where the wheels are, not how a
			// correction reaches them.
			TArray<FVector2D, TInlineAllocator<8>> BottomXY;
			FLegValues							   Bottoms;
			TArray<bool, TInlineAllocator<8>>	   Stands;
			for (const FLegReach& Leg : Legs)
			{
				BottomXY.Add(Leg.XY);
				Bottoms.Add(Leg.BaseZ - Leg.RadiusCm);
				Stands.Add(Leg.bSupport);
			}
			FVector2D GroundDeg = FVector2D::ZeroVector;
			double	  GroundHeightCm = 0.0;
			if (Points.Num() > 0 && FitPlane(BottomXY, Bottoms, Stands, TConstArrayView<bool>(), GroundDeg, GroundHeightCm))
			{
				for (const FClearancePoint& Point : Points)
				{
					const double GroundZ = GroundHeightCm + PlaneRise(Point.XY, true, GroundDeg);
					Problem.PointCap.Add(FMath::Max(0.0, Point.UndersideZ - GroundZ - Point.MinCm));
				}
			}
			return Problem;
		}

		FRideSpan SpanAt(const FProblem& Problem, double Share)
		{
			const FVector2D PlaneDeg = Problem.FullDeg * Share;
			const double	Mean = MeanRise(Problem.Legs, PlaneDeg);

			FRideSpan Span;
			for (int32 LegIndex = 0; LegIndex < Problem.Legs.Num(); ++LegIndex)
			{
				const FLegReach& Leg = Problem.Legs[LegIndex];
				const double	 Rise = PlaneRise(Leg.XY, Leg.bRoll, PlaneDeg) - Mean;

				double High = 0.0;
				if (Leg.bSupport)
				{
					High = SupportHigh(Leg, Problem.MarginCm);
					const double NeedHi = Rise - SupportLow(Leg, Problem.MarginCm);
					if (NeedHi < Span.Hi)
					{
						Span.Hi = NeedHi;
						Span.HiLeg = LegIndex;
					}
				}
				else
				{
					// A lifted leg stops at its travel and says nothing, until
					// what it falls short by has used up the clearance its
					// stance gave it. Asked to extend too far it only ends up
					// higher, so it has no lower limit.
					High = FMath::Max(0.0, Leg.MaxZ - Leg.BaseZ) + FMath::Max(0.0, Leg.BaselineClearanceCm - Problem.MinLiftedClearanceCm);
				}
				const double NeedLo = Rise - High;
				if (NeedLo > Span.Lo)
				{
					Span.Lo = NeedLo;
					Span.LoLeg = LegIndex;
					Span.LoPoint = INDEX_NONE;
				}
			}
			for (int32 PointIndex = 0; PointIndex < Problem.PointCap.Num(); ++PointIndex)
			{
				const double NeedLo = PlaneRise(Problem.Points[PointIndex].XY, true, PlaneDeg) - Mean - Problem.PointCap[PointIndex];
				if (NeedLo > Span.Lo)
				{
					Span.Lo = NeedLo;
					Span.LoLeg = INDEX_NONE;
					Span.LoPoint = PointIndex;
				}
			}
			return Span;
		}

		/** Whether some r in [WindowLo, WindowHi] is in the span. */
		bool Fits(const FRideSpan& Span, double WindowLo, double WindowHi)
		{
			return FMath::Max(Span.Lo, WindowLo) <= FMath::Min(Span.Hi, WindowHi) + FitSlackCm;
		}

		/**
		 * The largest share that fits with r in the window. OutUnfitShare is
		 * the last share the search found not to fit: -1 when the whole plane
		 * fits, 0 when not even none of it does (the return is then 0 all the
		 * same; only a window that does not hold 0 can do that).
		 */
		double LargestShare(const FProblem& Problem, double WindowLo, double WindowHi, double& OutUnfitShare)
		{
			if (Fits(SpanAt(Problem, 1.0), WindowLo, WindowHi))
			{
				OutUnfitShare = -1.0;
				return 1.0;
			}
			if (!Fits(SpanAt(Problem, 0.0), WindowLo, WindowHi))
			{
				OutUnfitShare = 0.0;
				return 0.0;
			}
			double Low = 0.0;
			double High = 1.0;
			for (int32 Pass = 0; Pass < AllocateBisections; ++Pass)
			{
				const double Mid = 0.5 * (Low + High);
				if (Fits(SpanAt(Problem, Mid), WindowLo, WindowHi))
				{
					Low = Mid;
				}
				else
				{
					High = Mid;
				}
			}
			OutUnfitShare = High;
			return Low;
		}
	} // namespace AllocateImpl

	void SupportMask(TArrayView<FLegReach> Legs, TConstArrayView<bool> Was, const FTuning& Tuning)
	{
		if (Legs.Num() == 0)
		{
			return;
		}
		double Lowest = Legs[0].BaseZ - Legs[0].RadiusCm;
		for (const FLegReach& Leg : Legs)
		{
			Lowest = FMath::Min(Lowest, Leg.BaseZ - Leg.RadiusCm);
		}
		// Two bands, so a leg hovering at the edge of one does not flip in
		// and out of the support set tick by tick.
		const bool	 bMemory = Was.Num() == Legs.Num();
		const double StayCm = FMath::Max(Tuning.SupportBandInCm, Tuning.SupportBandOutCm);
		for (int32 LegIndex = 0; LegIndex < Legs.Num(); ++LegIndex)
		{
			FLegReach& Leg = Legs[LegIndex];
			Leg.BaselineClearanceCm = FMath::Max(0.0, Leg.BaseZ - Leg.RadiusCm - Lowest);
			const bool bWas = bMemory && Was[LegIndex];
			Leg.bSupport = Leg.BaselineClearanceCm <= (bWas ? StayCm : Tuning.SupportBandInCm);
		}
	}

	bool FitPlane(TConstArrayView<FVector2D> XY, TConstArrayView<double> Values, TConstArrayView<bool> Use, TConstArrayView<bool> Roll,
		FVector2D& OutDeg, double& OutHeightCm)
	{
		OutDeg = FVector2D::ZeroVector;
		OutHeightCm = 0.0;

		const int32 Num = FMath::Min(XY.Num(), Values.Num());
		const auto	Counts = [&Use](int32 Index) { return Use.Num() == 0 || (Use.IsValidIndex(Index) && Use[Index]); };
		// A point roll does not reach takes x tan(pitch) only, so it sits at
		// y' = 0 as far as the fit goes.
		const auto Across = [&XY, &Roll](int32 Index) { return (Roll.Num() == 0 || !Roll.IsValidIndex(Index) || Roll[Index]) ? XY[Index].Y : 0.0; };

		int32  Count = 0;
		double MeanX = 0.0;
		double MeanY = 0.0;
		double MeanValue = 0.0;
		for (int32 Index = 0; Index < Num; ++Index)
		{
			if (Counts(Index))
			{
				MeanX += XY[Index].X;
				MeanY += Across(Index);
				MeanValue += Values[Index];
				++Count;
			}
		}
		if (Count == 0)
		{
			return false;
		}
		MeanX /= Count;
		MeanY /= Count;
		MeanValue /= Count;

		// About the centroid the height drops out, and what is left is a
		// 2 x 2 problem: Spread * (slope x, slope y) = Moment.
		double Sxx = 0.0;
		double Sxy = 0.0;
		double Syy = 0.0;
		double Sxv = 0.0;
		double Syv = 0.0;
		for (int32 Index = 0; Index < Num; ++Index)
		{
			if (Counts(Index))
			{
				const double Dx = XY[Index].X - MeanX;
				const double Dy = Across(Index) - MeanY;
				const double Dv = Values[Index] - MeanValue;
				Sxx += Dx * Dx;
				Sxy += Dx * Dy;
				Syy += Dy * Dy;
				Sxv += Dx * Dv;
				Syv += Dy * Dv;
			}
		}

		// The spread's two principal directions. A direction the points do
		// not span carries no information: dividing by its (near) zero spread
		// would turn rounding into slope, so it gets none -- the pseudo-inverse.
		const double Angle = 0.5 * FMath::Atan2(2.0 * Sxy, Sxx - Syy);
		const double Cos = FMath::Cos(Angle);
		const double Sin = FMath::Sin(Angle);
		const double SpreadMajor = Sxx * Cos * Cos + 2.0 * Sxy * Cos * Sin + Syy * Sin * Sin;
		const double SpreadMinor = Sxx * Sin * Sin - 2.0 * Sxy * Cos * Sin + Syy * Cos * Cos;
		const double Larger = FMath::Max(SpreadMajor, SpreadMinor);
		const auto	 Spanned = [Larger](double Spread) { return Spread >= AllocateImpl::SpreadMinCm2 && Spread >= AllocateImpl::SpreadRatioMin * Larger; };

		double SlopeX = 0.0;
		double SlopeY = 0.0;
		if (Spanned(SpreadMajor))
		{
			const double Along = (Sxv * Cos + Syv * Sin) / SpreadMajor;
			SlopeX += Along * Cos;
			SlopeY += Along * Sin;
		}
		if (Spanned(SpreadMinor))
		{
			const double Along = (-Sxv * Sin + Syv * Cos) / SpreadMinor;
			SlopeX += -Along * Sin;
			SlopeY += Along * Cos;
		}

		OutDeg = FVector2D(FMath::RadiansToDegrees(FMath::Atan(SlopeX)), FMath::RadiansToDegrees(FMath::Atan(SlopeY)));
		OutHeightCm = MeanValue - SlopeX * MeanX - SlopeY * MeanY;
		return true;
	}

	FVector2D MeasuredCorrection(TConstArrayView<FLegReach> Legs)
	{
		TArray<FVector2D, TInlineAllocator<8>> XY;
		FLegValues							   Moved;
		TArray<bool, TInlineAllocator<8>>	   Roll;
		for (const FLegReach& Leg : Legs)
		{
			XY.Add(Leg.XY);
			Moved.Add(Leg.MeasuredZ - Leg.BaseZ);
			Roll.Add(Leg.bRoll);
		}
		FVector2D Deg = FVector2D::ZeroVector;
		double	  HeightCm = 0.0;
		FitPlane(XY, Moved, TConstArrayView<bool>(), Roll, Deg, HeightCm);
		return Deg;
	}

	void RigidOffsets(TConstArrayView<FLegReach> Legs, const FVector2D& Deg, FLegValues& Out)
	{
		const double Mean = AllocateImpl::MeanRise(Legs, Deg);
		Out.Reset();
		for (const FLegReach& Leg : Legs)
		{
			Out.Add(PlaneRise(Leg.XY, Leg.bRoll, Deg) - Mean);
		}
	}

	double AllocateFraction(TConstArrayView<FLegReach> Legs, TConstArrayView<FClearancePoint> Points, const FVector2D& CorrectionDeg,
		const FTuning& Tuning, double ExtraMarginCm)
	{
		const AllocateImpl::FProblem Problem = AllocateImpl::MakeProblem(Legs, Points, CorrectionDeg, Tuning, ExtraMarginCm);
		double						 UnfitShare = -1.0;
		return AllocateImpl::LargestShare(Problem, -FMath::Max(0.0, Tuning.RideHeightDownCm), FMath::Max(0.0, Tuning.RideHeightUpCm), UnfitShare);
	}

	FAllocation Allocate(TConstArrayView<FLegReach> Legs, TConstArrayView<FClearancePoint> Points, const FVector2D& CorrectionDeg,
		const FTuning& Tuning, double PrevRideCm, double MaxRideStepCm)
	{
		const AllocateImpl::FProblem Problem = AllocateImpl::MakeProblem(Legs, Points, CorrectionDeg, Tuning, 0.0);

		// Where the ride shift may be this tick: inside its allowance, and no
		// further from last tick's than the slew lets it go.
		const double RideLo = -FMath::Max(0.0, Tuning.RideHeightDownCm);
		const double RideHi = FMath::Max(0.0, Tuning.RideHeightUpCm);
		const double StepCm = FMath::Max(0.0, MaxRideStepCm);
		const double WindowLo = FMath::Max(RideLo, PrevRideCm - StepCm);
		const double WindowHi = FMath::Min(RideHi, PrevRideCm + StepCm);

		FAllocation Out;
		double		UnfitShare = -1.0;
		Out.Fraction = AllocateImpl::LargestShare(Problem, WindowLo, WindowHi, UnfitShare);
		Out.AppliedDeg = Problem.FullDeg * Out.Fraction;

		const AllocateImpl::FRideSpan Span = AllocateImpl::SpanAt(Problem, Out.Fraction);
		if (UnfitShare == 0.0)
		{
			// Not even the bare stance fits: the shift is still on its way
			// back from where an earlier plane left it. The rate limit wins;
			// go as far towards what the limits want as it allows.
			const double WantedCm = FMath::Clamp(0.0, Span.Lo, FMath::Max(Span.Lo, Span.Hi));
			Out.HeightCm = FMath::Clamp(FMath::Clamp(WantedCm, PrevRideCm - StepCm, PrevRideCm + StepCm), RideLo, RideHi);
		}
		else
		{
			// The shift that fits nearest none at all: one no longer needed
			// goes back by itself, at the slew rate.
			const double FitLo = FMath::Max(Span.Lo, WindowLo);
			const double FitHi = FMath::Max(FitLo, FMath::Min(Span.Hi, WindowHi));
			Out.HeightCm = FMath::Clamp(0.0, FitLo, FitHi);
		}

		const double Mean = AllocateImpl::MeanRise(Legs, Out.AppliedDeg);
		Out.Dz.Reserve(Legs.Num());
		for (const FLegReach& Leg : Legs)
		{
			const double WantCm = PlaneRise(Leg.XY, Leg.bRoll, Out.AppliedDeg) - Mean - Out.HeightCm;
			// The last resort, and a lifted leg's quiet stop: never past the
			// travel, never pulled back from where the baseline already is.
			Out.Dz.Add(FMath::Clamp(WantCm, FMath::Min(0.0, Leg.MinZ - Leg.BaseZ), FMath::Max(0.0, Leg.MaxZ - Leg.BaseZ)));
		}

		if (UnfitShare >= 0.0)
		{
			// What stopped it: at the share that did not fit, with the shift
			// that comes nearest to fitting (every limit moves one for one
			// with the shift, so that is midway between the two that clash,
			// kept inside the window), the limit crossed furthest.
			const AllocateImpl::FRideSpan Bad = AllocateImpl::SpanAt(Problem, UnfitShare);
			double						  NearestCm = 0.5 * (Bad.Lo + Bad.Hi);
			if (Bad.Hi >= AllocateImpl::UnboundedCm)
			{
				NearestCm = Bad.Lo;
			}
			else if (Bad.Lo <= -AllocateImpl::UnboundedCm)
			{
				NearestCm = Bad.Hi;
			}
			NearestCm = FMath::Clamp(NearestCm, WindowLo, FMath::Max(WindowLo, WindowHi));
			const double OverHigh = Bad.Lo - NearestCm;
			const double UnderLow = NearestCm - Bad.Hi;
			if (OverHigh >= UnderLow && OverHigh > 0.0)
			{
				Out.LimitingLeg = Bad.LoLeg;
				Out.LimitingPoint = Bad.LoPoint;
			}
			else if (UnderLow > 0.0)
			{
				Out.LimitingLeg = Bad.HiLeg;
			}
		}

		Out.MinMarginCm = TravelMargin(Legs, Out.Dz);
		for (int32 LegIndex = 0; LegIndex < Legs.Num(); ++LegIndex)
		{
			const FLegReach& Leg = Legs[LegIndex];
			const double	 MovedCm = Out.Dz[LegIndex];
			if (!Leg.bSupport || FMath::Abs(MovedCm) <= TravelMarginMinMoveCm)
			{
				continue;
			}
			const double LimitCm = MovedCm > 0.0 ? AllocateImpl::SupportHigh(Leg, Problem.MarginCm) : AllocateImpl::SupportLow(Leg, Problem.MarginCm);
			const double Left = FMath::Abs(LimitCm) > 0.0 ? FMath::Clamp(1.0 - MovedCm / LimitCm, 0.0, 1.0) : 0.0;
			Out.ReachLeft = FMath::Min(Out.ReachLeft, Left);
		}
		return Out;
	}

	double TravelMargin(TConstArrayView<FLegReach> Legs, TConstArrayView<double> Dz)
	{
		double		Least = NoTravelMarginCm;
		const int32 Num = FMath::Min(Legs.Num(), Dz.Num());
		for (int32 LegIndex = 0; LegIndex < Num; ++LegIndex)
		{
			const FLegReach& Leg = Legs[LegIndex];
			const double	 MovedCm = Dz[LegIndex];
			if (!Leg.bSupport || FMath::Abs(MovedCm) <= TravelMarginMinMoveCm)
			{
				continue;
			}
			const double AtZ = Leg.BaseZ + MovedCm;
			const double LeftCm = MovedCm > 0.0 ? Leg.MaxZ - AtZ : AtZ - Leg.MinZ;
			Least = FMath::Min(Least, FMath::Max(0.0, LeftCm));
		}
		return Least;
	}
} // namespace RammsLevel
