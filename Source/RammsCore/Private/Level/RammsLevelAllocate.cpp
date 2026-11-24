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
		/** MeasuredCorrection's own, stricter, ratio for the legs that stand:
		 *  they must be at least a tenth as wide as they are long (a hundredth
		 *  in spread) for the plane across them to be theirs. Narrower, and a
		 *  millimetre of sag on one of them would read as degrees. */
		constexpr double CarrySpreadRatioMin = 1.0e-2;

		/** A plane fit as FitPlane makes it, with what MeasuredCorrection
		 *  needs besides: each point counted by a weight (0 or more; FitPlane's
		 *  Use is 1 or 0), the slopes as tangents, and the two principal
		 *  directions of the points' spread with whether each is spanned. */
		struct FPlaneFit
		{
			/** A point counted (some weight was over 0). */
			bool bAny = false;
			/** The slopes along x and y' (tangents), and the height at the origin. */
			FVector2D Slope = FVector2D::ZeroVector;
			double	  HeightCm = 0.0;
			/** The spread's principal directions (unit vectors in x, y') and
			 *  whether the points span each. An unspanned one has no slope. */
			FVector2D Major = FVector2D(1.0, 0.0);
			FVector2D Minor = FVector2D(0.0, 1.0);
			bool	  bMajor = false;
			bool	  bMinor = false;
		};

		FPlaneFit Fit(TConstArrayView<FVector2D> XY, TConstArrayView<double> Values, TConstArrayView<double> Weights, TConstArrayView<bool> Roll, double RatioMin)
		{
			FPlaneFit Out;

			const int32 Num = FMath::Min(XY.Num(), Values.Num());
			// (No weights: every point counts whole. Weights shorter than the
			// points leave the points past their end out.)
			const auto Weight = [&Weights](int32 Index) { return Weights.Num() == 0 ? 1.0 : (Weights.IsValidIndex(Index) ? FMath::Max(0.0, Weights[Index]) : 0.0); };
			// A point roll does not reach takes x tan(pitch) only, so it sits at
			// y' = 0 as far as the fit goes.
			const auto Across = [&XY, &Roll](int32 Index) { return (Roll.Num() == 0 || !Roll.IsValidIndex(Index) || Roll[Index]) ? XY[Index].Y : 0.0; };

			double Count = 0.0;
			double MeanX = 0.0;
			double MeanY = 0.0;
			double MeanValue = 0.0;
			for (int32 Index = 0; Index < Num; ++Index)
			{
				const double W = Weight(Index);
				MeanX += W * XY[Index].X;
				MeanY += W * Across(Index);
				MeanValue += W * Values[Index];
				Count += W;
			}
			if (!(Count > 0.0))
			{
				return Out;
			}
			Out.bAny = true;
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
				const double W = Weight(Index);
				const double Dx = XY[Index].X - MeanX;
				const double Dy = Across(Index) - MeanY;
				const double Dv = Values[Index] - MeanValue;
				Sxx += W * Dx * Dx;
				Sxy += W * Dx * Dy;
				Syy += W * Dy * Dy;
				Sxv += W * Dx * Dv;
				Syv += W * Dy * Dv;
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
			const auto	 Spanned = [Larger, RatioMin](double Spread) { return Spread >= SpreadMinCm2 && Spread >= RatioMin * Larger; };

			Out.Major = FVector2D(Cos, Sin);
			Out.Minor = FVector2D(-Sin, Cos);
			Out.bMajor = Spanned(SpreadMajor);
			Out.bMinor = Spanned(SpreadMinor);
			if (Out.bMajor)
			{
				Out.Slope += Out.Major * ((Sxv * Cos + Syv * Sin) / SpreadMajor);
			}
			if (Out.bMinor)
			{
				Out.Slope += Out.Minor * ((-Sxv * Sin + Syv * Cos) / SpreadMinor);
			}
			Out.HeightCm = MeanValue - Out.Slope.X * MeanX - Out.Slope.Y * MeanY;
			return Out;
		}

		FVector2D SlopeDeg(const FVector2D& Slope)
		{
			return FVector2D(FMath::RadiansToDegrees(FMath::Atan(Slope.X)), FMath::RadiansToDegrees(FMath::Atan(Slope.Y)));
		}

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

		/** The most a lifted leg may be asked to rise: its travel, and what of
		 *  its stance's clearance it may lose (the constraint set's U_i). */
		double LiftedHigh(const FLegReach& Leg, double MinLiftedClearanceCm)
		{
			return FMath::Max(0.0, Leg.MaxZ - Leg.BaseZ) + FMath::Max(0.0, Leg.BaselineClearanceCm - MinLiftedClearanceCm);
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
					High = LiftedHigh(Leg, Problem.MinLiftedClearanceCm);
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

		/**
		 * What stopped a search: at UnfitShare, the last share found not to
		 * fit, with the shift that comes nearest to fitting (every limit moves
		 * one for one with the shift, so that is midway between the two that
		 * clash, kept inside the window), the limit crossed furthest. Both
		 * INDEX_NONE when only the window is at fault.
		 */
		void LimitAt(const FProblem& Problem, double UnfitShare, double WindowLo, double WindowHi, int32& OutLeg, int32& OutPoint)
		{
			OutLeg = INDEX_NONE;
			OutPoint = INDEX_NONE;
			const FRideSpan Bad = SpanAt(Problem, UnfitShare);
			double			NearestCm = 0.5 * (Bad.Lo + Bad.Hi);
			if (Bad.Hi >= UnboundedCm)
			{
				NearestCm = Bad.Lo;
			}
			else if (Bad.Lo <= -UnboundedCm)
			{
				NearestCm = Bad.Hi;
			}
			NearestCm = FMath::Clamp(NearestCm, WindowLo, FMath::Max(WindowLo, WindowHi));
			const double OverHigh = Bad.Lo - NearestCm;
			const double UnderLow = NearestCm - Bad.Hi;
			if (OverHigh >= UnderLow && OverHigh > 0.0)
			{
				OutLeg = Bad.LoLeg;
				OutPoint = Bad.LoPoint;
			}
			else if (UnderLow > 0.0)
			{
				OutLeg = Bad.HiLeg;
			}
		}

		EReachLimit KindOf(TConstArrayView<FLegReach> Legs, int32 Leg, int32 Point)
		{
			if (Point != INDEX_NONE)
			{
				return EReachLimit::Underside;
			}
			if (Legs.IsValidIndex(Leg))
			{
				return Legs[Leg].bSupport ? EReachLimit::LegTravel : EReachLimit::LiftedClearance;
			}
			return EReachLimit::None;
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
			// No memory in this one, so that it never steps; and gone by the
			// inner band, the narrower: a leg drawn up off the ground is in
			// the air from the first of its lift, whatever the set still says.
			const double CarryCm = FMath::Min(Tuning.SupportBandInCm, StayCm);
			Leg.CarryShare = CarryCm > 0.0 ? FMath::Clamp(1.0 - Leg.BaselineClearanceCm / CarryCm, 0.0, 1.0) : (Leg.BaselineClearanceCm <= 0.0 ? 1.0 : 0.0);
		}
	}

	void DiscountLateLegs(TArrayView<FLegReach> Legs, TConstArrayView<double> Dz, const FTuning& Tuning)
	{
		const double CarryCm = FMath::Min(Tuning.SupportBandInCm, FMath::Max(Tuning.SupportBandInCm, Tuning.SupportBandOutCm));
		// How far each wheel is above where it is told to be, and the least
		// of that among the legs that carry: what they are all late by (a
		// sag under the load, a stance being lowered) is nobody's lateness.
		FLegValues Late;
		double	   Least = 0.0;
		bool	   bAny = false;
		for (int32 LegIndex = 0; LegIndex < Legs.Num(); ++LegIndex)
		{
			const FLegReach& Leg = Legs[LegIndex];
			Late.Add(FMath::Max(0.0, Leg.MeasuredZ - (Leg.BaseZ + (Dz.IsValidIndex(LegIndex) ? Dz[LegIndex] : 0.0))));
			if (Leg.CarryShare > 0.0)
			{
				Least = bAny ? FMath::Min(Least, Late[LegIndex]) : Late[LegIndex];
				bAny = true;
			}
		}
		for (int32 LegIndex = 0; LegIndex < Legs.Num(); ++LegIndex)
		{
			FLegReach&	 Leg = Legs[LegIndex];
			const double LateCm = Late[LegIndex] - Least;
			if (LateCm > 0.0)
			{
				Leg.CarryShare = CarryCm > 0.0 ? FMath::Clamp(Leg.CarryShare - LateCm / CarryCm, 0.0, 1.0) : 0.0;
			}
		}
	}

	bool FitPlane(TConstArrayView<FVector2D> XY, TConstArrayView<double> Values, TConstArrayView<bool> Use, TConstArrayView<bool> Roll,
		FVector2D& OutDeg, double& OutHeightCm)
	{
		// (A Use shorter than the points leaves the points past its end out.)
		FLegValues Weights;
		if (Use.Num() > 0)
		{
			for (int32 Index = 0; Index < XY.Num(); ++Index)
			{
				Weights.Add(Use.IsValidIndex(Index) && Use[Index] ? 1.0 : 0.0);
			}
		}
		const AllocateImpl::FPlaneFit Fit = AllocateImpl::Fit(XY, Values, Weights, Roll, AllocateImpl::SpreadRatioMin);
		OutDeg = AllocateImpl::SlopeDeg(Fit.Slope);
		OutHeightCm = Fit.HeightCm;
		return Fit.bAny;
	}

	FVector2D MeasuredCorrection(TConstArrayView<FLegReach> Legs)
	{
		TArray<FVector2D, TInlineAllocator<8>> XY;
		FLegValues							   Moved;
		TArray<bool, TInlineAllocator<8>>	   Roll;
		FLegValues							   Carries;
		for (const FLegReach& Leg : Legs)
		{
			XY.Add(Leg.XY);
			Moved.Add(Leg.MeasuredZ - Leg.BaseZ);
			Roll.Add(Leg.bRoll);
			// (The square: a leg with leverage over the others keeps most of
			// its say in a least-squares plane down to a small weight, and
			// would lose it all in the last of its lift. Squared, it gives
			// it up more evenly on the way.)
			Carries.Add(FMath::Square(FMath::Clamp(Leg.CarryShare, 0.0, 1.0)));
		}
		// The attitude the joints give the base is set by the legs it stands
		// on. A leg carried clear of the ground changes nothing by where it
		// is: one resting on its retract stop, or still on its way, is not
		// part of the plane the base was moved by.
		const AllocateImpl::FPlaneFit Carried = AllocateImpl::Fit(XY, Moved, Carries, Roll, AllocateImpl::CarrySpreadRatioMin);
		FVector2D					  Slope = Carried.Slope;
		if (!Carried.bAny || !Carried.bMajor || !Carried.bMinor)
		{
			// Where the standing legs say nothing (none stands; across a line
			// of them), the plane through every leg stands in: what the legs
			// were all told, as far as they have followed it.
			const AllocateImpl::FPlaneFit All = AllocateImpl::Fit(XY, Moved, TConstArrayView<double>(), Roll, AllocateImpl::SpreadRatioMin);
			if (!Carried.bAny)
			{
				Slope = All.Slope;
			}
			else
			{
				if (!Carried.bMajor)
				{
					Slope += Carried.Major * FVector2D::DotProduct(All.Slope, Carried.Major);
				}
				if (!Carried.bMinor)
				{
					Slope += Carried.Minor * FVector2D::DotProduct(All.Slope, Carried.Minor);
				}
			}
		}
		return AllocateImpl::SlopeDeg(Slope);
	}

	FVector2D BaselinePlaneMove(TConstArrayView<FLegReach> Legs, TConstArrayView<double> PrevBaseZ)
	{
		if (PrevBaseZ.Num() != Legs.Num())
		{
			return FVector2D::ZeroVector;
		}
		// The plane the joints would be measured to have made had they gone
		// from the old baselines to the new: the same legs, the same weights.
		TArray<FLegReach, TInlineAllocator<8>> Moved;
		Moved.Append(Legs.GetData(), Legs.Num());
		for (int32 Index = 0; Index < Moved.Num(); ++Index)
		{
			Moved[Index].MeasuredZ = Legs[Index].BaseZ;
			Moved[Index].BaseZ = PrevBaseZ[Index];
		}
		return MeasuredCorrection(Moved);
	}

	double BaselineTurnDeg(TConstArrayView<FLegReach> Legs, TConstArrayView<double> PrevBaseZ)
	{
		if (PrevBaseZ.Num() != Legs.Num())
		{
			return 0.0;
		}
		const double Carried = BaselinePlaneMove(Legs, PrevBaseZ).Size();
		// ... and through the whole support set, each leg alike: one drawn up
		// inside the support band may be the one the robot now sits on.
		TArray<FVector2D, TInlineAllocator<8>> XY;
		FLegValues							   Moved;
		TArray<bool, TInlineAllocator<8>>	   Stands;
		TArray<bool, TInlineAllocator<8>>	   Roll;
		for (int32 Index = 0; Index < Legs.Num(); ++Index)
		{
			XY.Add(Legs[Index].XY);
			Moved.Add(Legs[Index].BaseZ - PrevBaseZ[Index]);
			Stands.Add(Legs[Index].bSupport);
			Roll.Add(Legs[Index].bRoll);
		}
		FVector2D	 Deg = FVector2D::ZeroVector;
		double		 HeightCm = 0.0;
		const double Standing = FitPlane(XY, Moved, Stands, Roll, Deg, HeightCm) ? Deg.Size() : 0.0;
		return FMath::Max(Carried, Standing);
	}

	double StandingSpanCm(TConstArrayView<FLegReach> Legs)
	{
		bool bAnyStands = false;
		for (const FLegReach& Leg : Legs)
		{
			bAnyStands = bAnyStands || Leg.bSupport;
		}
		FVector2D Low = FVector2D::ZeroVector;
		FVector2D High = FVector2D::ZeroVector;
		bool	  bAny = false;
		for (const FLegReach& Leg : Legs)
		{
			if (bAnyStands && !Leg.bSupport)
			{
				continue;
			}
			Low = bAny ? FVector2D(FMath::Min(Low.X, Leg.XY.X), FMath::Min(Low.Y, Leg.XY.Y)) : Leg.XY;
			High = bAny ? FVector2D(FMath::Max(High.X, Leg.XY.X), FMath::Max(High.Y, Leg.XY.Y)) : Leg.XY;
			bAny = true;
		}
		const double Along = High.X - Low.X;
		const double Across = High.Y - Low.Y;
		double		 Span = 0.0;
		if (Along >= GroundSpanMinCm)
		{
			Span = Along;
		}
		if (Across >= GroundSpanMinCm)
		{
			Span = Span > 0.0 ? FMath::Min(Span, Across) : Across;
		}
		return Span > 0.0 ? Span : GroundSpanMinCm;
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
		return AllocateReach(Legs, Points, CorrectionDeg, Tuning, ExtraMarginCm).Share;
	}

	FReach AllocateReach(TConstArrayView<FLegReach> Legs, TConstArrayView<FClearancePoint> Points, const FVector2D& CorrectionDeg, const FTuning& Tuning,
		double ExtraMarginCm)
	{
		const AllocateImpl::FProblem Problem = AllocateImpl::MakeProblem(Legs, Points, CorrectionDeg, Tuning, ExtraMarginCm);
		const double				 WindowLo = -FMath::Max(0.0, Tuning.RideHeightDownCm);
		const double				 WindowHi = FMath::Max(0.0, Tuning.RideHeightUpCm);
		double						 UnfitShare = -1.0;
		FReach						 Out;
		Out.Share = AllocateImpl::LargestShare(Problem, WindowLo, WindowHi, UnfitShare);
		if (UnfitShare >= 0.0)
		{
			int32 Leg = INDEX_NONE;
			int32 Point = INDEX_NONE;
			AllocateImpl::LimitAt(Problem, UnfitShare, WindowLo, WindowHi, Leg, Point);
			Out.Limit = AllocateImpl::KindOf(Legs, Leg, Point);
		}
		return Out;
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
			AllocateImpl::LimitAt(Problem, UnfitShare, WindowLo, WindowHi, Out.LimitingLeg, Out.LimitingPoint);
			Out.Limit = AllocateImpl::KindOf(Legs, Out.LimitingLeg, Out.LimitingPoint);
		}

		// How near the plane applied is to whatever would stop it: a leg's
		// travel, a lifted wheel's clearance, the underside's, an axis' limit.
		// The first of them on a tie.
		const auto Take = [&Out](double Used, double Most, double RoomCm, EReachLimit Kind) {
			const double Left = Most > 0.0 ? FMath::Clamp(1.0 - Used / Most, 0.0, 1.0) : 0.0;
			if (Left < Out.ReachLeft)
			{
				Out.ReachLeft = Left;
				Out.NearestLimit = Kind;
			}
			Out.MinMarginCm = FMath::Min(Out.MinMarginCm, FMath::Max(0.0, RoomCm));
		};
		Out.MinMarginCm = TravelMargin(Legs, Out.Dz);
		for (int32 LegIndex = 0; LegIndex < Legs.Num(); ++LegIndex)
		{
			const FLegReach& Leg = Legs[LegIndex];
			const double	 MovedCm = Out.Dz[LegIndex];
			if (!Leg.bSupport || FMath::Abs(MovedCm) <= TravelMarginMinMoveCm)
			{
				continue;
			}
			// Of the travel the wanted tilt is PLANNED with (the soft zone kept
			// back): resting at the end of the planned reach reads 0.
			const double PlannedCm = Problem.MarginCm + FMath::Max(0.0, Tuning.SoftZoneCm);
			const double LimitCm = MovedCm > 0.0 ? AllocateImpl::SupportHigh(Leg, PlannedCm) : AllocateImpl::SupportLow(Leg, PlannedCm);
			// (The room is TravelMargin's, taken above: to the true limit.)
			Take(FMath::Abs(MovedCm), FMath::Abs(LimitCm), NoTravelMarginCm, EReachLimit::LegTravel);
		}
		for (const FLegReach& Leg : Legs)
		{
			// What the plane asks of a leg carried off the ground, whatever its
			// travel then lets it do: the part it cannot follow comes off its
			// clearance.
			const double WantCm = PlaneRise(Leg.XY, Leg.bRoll, Out.AppliedDeg) - Mean - Out.HeightCm;
			if (Leg.bSupport || WantCm <= TravelMarginMinMoveCm)
			{
				continue;
			}
			const double MostCm = AllocateImpl::LiftedHigh(Leg, Problem.MinLiftedClearanceCm);
			Take(WantCm, MostCm, MostCm - WantCm, EReachLimit::LiftedClearance);
		}
		for (int32 PointIndex = 0; PointIndex < Problem.PointCap.Num(); ++PointIndex)
		{
			const double RiseCm = PlaneRise(Points[PointIndex].XY, true, Out.AppliedDeg) - Mean - Out.HeightCm;
			if (RiseCm <= TravelMarginMinMoveCm)
			{
				continue;
			}
			Take(RiseCm, Problem.PointCap[PointIndex], Problem.PointCap[PointIndex] - RiseCm, EReachLimit::Underside);
		}
		{
			// An axis' own limit, its room as a height at the standing leg
			// furthest from the standing legs' middle along that axis.
			bool bAnyStands = false;
			for (const FLegReach& Leg : Legs)
			{
				bAnyStands = bAnyStands || Leg.bSupport;
			}
			FVector2D Middle = FVector2D::ZeroVector;
			int32	  Count = 0;
			for (const FLegReach& Leg : Legs)
			{
				if (!bAnyStands || Leg.bSupport)
				{
					Middle += Leg.XY;
					++Count;
				}
			}
			Middle = Count > 0 ? Middle / Count : Middle;
			FVector2D Arm = FVector2D::ZeroVector;
			for (const FLegReach& Leg : Legs)
			{
				if (!bAnyStands || Leg.bSupport)
				{
					Arm.X = FMath::Max(Arm.X, FMath::Abs(Leg.XY.X - Middle.X));
					Arm.Y = FMath::Max(Arm.Y, Leg.bRoll ? FMath::Abs(Leg.XY.Y - Middle.Y) : 0.0);
				}
			}
			const FVector2D MostDeg(FMath::Max(0.0, Tuning.MaxCorrectionDeg.X), FMath::Max(0.0, Tuning.MaxCorrectionDeg.Y));
			for (int32 Axis = 0; Axis < 2; ++Axis)
			{
				const double UsedDeg = FMath::Abs(Out.AppliedDeg[Axis]);
				if (UsedDeg <= ReachMinCorrectionDeg)
				{
					continue;
				}
				// (Legs with no extent along the axis are moved by none of it:
				// there is then no height to give the room as.)
				const double RoomDeg = FMath::Max(0.0, MostDeg[Axis] - UsedDeg);
				const double RoomCm = Arm[Axis] > 0.0 ? Arm[Axis] * FMath::Tan(FMath::DegreesToRadians(RoomDeg)) : NoTravelMarginCm;
				Take(UsedDeg, MostDeg[Axis], RoomCm, EReachLimit::AxisLimit);
			}
		}
		return Out;
	}

	bool PlanAdoption(TConstArrayView<FLegReach> Legs, TConstArrayView<double> StanceZ, TConstArrayView<FClearancePoint> Points, const FTuning& Tuning,
		FAdoption& Out)
	{
		using FLegs = TArray<FLegReach, TInlineAllocator<8>>;
		Out = FAdoption();
		const int32 Num = Legs.Num();
		if (Num == 0 || StanceZ.Num() != Num)
		{
			return false;
		}

		// Who stands in the stance the legs are going to: the base's attitude
		// there is theirs, so the plane is fitted over them and no other. (A
		// wheel the stance carries off the ground may be anywhere in the pose.)
		FLegs Stance;
		Stance.Append(Legs.GetData(), Num);
		for (int32 Index = 0; Index < Num; ++Index)
		{
			Stance[Index].BaseZ = StanceZ[Index];
		}
		SupportMask(Stance, TConstArrayView<bool>(), Tuning);
		TArray<FVector2D, TInlineAllocator<8>> XY;
		FLegValues							   Carried;
		TArray<bool, TInlineAllocator<8>>	   Use;
		TArray<bool, TInlineAllocator<8>>	   Roll;
		bool								   bAnyStands = false;
		for (int32 Index = 0; Index < Num; ++Index)
		{
			XY.Add(Legs[Index].XY);
			Carried.Add(Legs[Index].BaseZ - StanceZ[Index]);
			Use.Add(Stance[Index].bSupport);
			Roll.Add(Legs[Index].bRoll);
			bAnyStands = bAnyStands || Stance[Index].bSupport;
		}
		FVector2D PlaneDeg = FVector2D::ZeroVector;
		double	  HeightCm = 0.0;
		if (!bAnyStands || !FitPlane(XY, Carried, Use, Roll, PlaneDeg, HeightCm) || !FMath::IsFinite(PlaneDeg.X) || !FMath::IsFinite(PlaneDeg.Y))
		{
			return false;
		}
		PlaneDeg = ClampAxes(PlaneDeg, Tuning.MaxCorrectionDeg);

		// The offsets are the law's own for that plane, on the baselines they
		// leave: which legs stand there (and so the height the offsets are
		// taken about) is found from those baselines, which the offsets set.
		// Who stands does not depend on a height all the legs share, so a
		// round or two settles it; a plane cut at the legs' reach takes more.
		// The first round takes the plane about the legs that stand in the
		// stance, as it will be once the ramp is done: a leg the stance
		// carries off the ground then starts from a baseline that keeps it
		// off the ground, and is not taken for one that limits the plane.
		FLegs	   Trial;
		FLegValues Dz;
		Trial.Append(Legs.GetData(), Num);
		const double StanceMean = AllocateImpl::MeanRise(Stance, PlaneDeg);
		for (int32 Index = 0; Index < Num; ++Index)
		{
			Dz.Add(PlaneRise(Legs[Index].XY, Legs[Index].bRoll, PlaneDeg) - StanceMean);
		}
		const auto Place = [&]() {
			for (int32 Index = 0; Index < Num; ++Index)
			{
				// Never a baseline past the leg's travel: no joint makes it.
				const FLegReach& Leg = Legs[Index];
				Trial[Index].BaseZ = FMath::Clamp(Leg.BaseZ - Dz[Index], FMath::Min(Leg.MinZ, Leg.MaxZ), FMath::Max(Leg.MinZ, Leg.MaxZ));
			}
		};
		for (int32 Pass = 0; Pass < AdoptPasses; ++Pass)
		{
			Place();
			SupportMask(Trial, TConstArrayView<bool>(), Tuning);
			const FAllocation Alloc = Allocate(Trial, Points, PlaneDeg, Tuning, 0.0, 0.0);
			if (Alloc.Dz.Num() != Num)
			{
				return false;
			}
			if (Alloc.Fraction < 1.0)
			{
				PlaneDeg = Alloc.AppliedDeg;
			}
			Dz = Alloc.Dz;
		}
		Place();
		Out.PlaneDeg = PlaneDeg;
		for (int32 Index = 0; Index < Num; ++Index)
		{
			Out.BaseZ.Add(Trial[Index].BaseZ);
			Out.Dz.Add(Legs[Index].BaseZ - Trial[Index].BaseZ);
		}
		return true;
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
