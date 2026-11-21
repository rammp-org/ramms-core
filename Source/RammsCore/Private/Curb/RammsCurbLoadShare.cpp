// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbLoadShare.h"
#include "Ramms4BarKinematics.h"

namespace RammsCurb
{
	bool FourBarWheelRate(const FRamms4BarLegSpec& Spec, double CrankRad, FVector2D& OutCmPerRad)
	{
		constexpr double Step = 0.005;
		const double	 Lo = FMath::Min(Spec.CrankRange.X, Spec.CrankRange.Y);
		const double	 Hi = FMath::Max(Spec.CrankRange.X, Spec.CrankRange.Y);
		const double	 At = FMath::Clamp(CrankRad, Lo, Hi);
		const double	 Back = FMath::Max(Lo, At - Step);
		const double	 On = FMath::Min(Hi, At + Step);
		if (On - Back < 1e-6)
		{
			return false;
		}
		bool			bBack = false, bOn = false;
		const FVector2D PBack = URamms4BarKinematics::ComputeWheelCenter(Spec, static_cast<float>(Back), bBack);
		const FVector2D POn = URamms4BarKinematics::ComputeWheelCenter(Spec, static_cast<float>(On), bOn);
		if (bBack && bOn)
		{
			OutCmPerRad = (POn - PBack) / (On - Back);
			return true;
		}
		// One side alone, from the pose itself.
		bool			bHere = false;
		const FVector2D PHere = URamms4BarKinematics::ComputeWheelCenter(Spec, static_cast<float>(At), bHere);
		if (bHere && bOn && On - At > 1e-6)
		{
			OutCmPerRad = (POn - PHere) / (On - At);
			return true;
		}
		if (bHere && bBack && At - Back > 1e-6)
		{
			OutCmPerRad = (PHere - PBack) / (At - Back);
			return true;
		}
		return false;
	}

	bool PressTorqueCap(double JxCmPerRad, double JzCmPerRad, double ForceCapN, double TractionRatio, double ReactionCmPerRad,
		double MinJzCmPerRad, double& OutCapNm, int32& OutPressSign)
	{
		OutCapNm = 0.0;
		OutPressSign = 0;
		if (!(FMath::Abs(JzCmPerRad) >= FMath::Max(MinJzCmPerRad, 1e-6)) || !(ForceCapN > 0.0))
		{
			return false;
		}
		const double F = ForceCapN;
		OutCapNm = (FMath::Abs(JzCmPerRad) * F + (FMath::Abs(JxCmPerRad) + FMath::Max(0.0, ReactionCmPerRad)) * FMath::Max(0.0, TractionRatio) * F) / 100.0;
		// Pressing down pushes the ground with -z: the motor turns against Jz.
		OutPressSign = JzCmPerRad < 0.0 ? 1 : -1;
		return true;
	}

	namespace
	{
		/** The least-squares shares of WeightN among the contacts in Active
		 *  (indices into P), written to Out at those indices. False with the
		 *  centre of mass off a line of contacts. */
		bool SolveShares(TConstArrayView<FVector2D> P, const TArray<int32, TInlineAllocator<8>>& Active, const FVector2D& Com, double WeightN,
			TArray<double>& Out)
		{
			const int32 n = Active.Num();
			FVector2D	Centroid = FVector2D::ZeroVector;
			for (const int32 i : Active)
			{
				Centroid += P[i];
			}
			Centroid /= n;
			double Suu = 0.0, Svv = 0.0, Suv = 0.0;
			for (const int32 i : Active)
			{
				const FVector2D d = P[i] - Centroid;
				Suu += d.X * d.X;
				Svv += d.Y * d.Y;
				Suv += d.X * d.Y;
			}
			const FVector2D Off = Com - Centroid;
			const double	Trace = Suu + Svv;
			const double	Det = Suu * Svv - Suv * Suv;
			if (Trace < 1e-9)
			{
				// Every contact at one point: it carries the weight if the centre of
				// mass is over it.
				if (Off.Size() > 1.0)
				{
					return false;
				}
				for (const int32 i : Active)
				{
					Out[i] = WeightN / n;
				}
				return true;
			}
			if (Det > 1e-6 * Trace * Trace)
			{
				const double b = WeightN * (Svv * Off.X - Suv * Off.Y) / Det;
				const double c = WeightN * (Suu * Off.Y - Suv * Off.X) / Det;
				for (const int32 i : Active)
				{
					const FVector2D d = P[i] - Centroid;
					Out[i] = WeightN / n + b * d.X + c * d.Y;
				}
				return true;
			}
			// On one line: along its principal axis.
			const double	Angle = 0.5 * FMath::Atan2(2.0 * Suv, Suu - Svv);
			const FVector2D Axis(FMath::Cos(Angle), FMath::Sin(Angle));
			const double	Across = FMath::Abs(Off.X * -Axis.Y + Off.Y * Axis.X);
			if (Across > 1.0)
			{
				return false;
			}
			double Sss = 0.0;
			for (const int32 i : Active)
			{
				const double s = FVector2D::DotProduct(P[i] - Centroid, Axis);
				Sss += s * s;
			}
			const double b = Sss > 1e-9 ? WeightN * FVector2D::DotProduct(Off, Axis) / Sss : 0.0;
			for (const int32 i : Active)
			{
				Out[i] = WeightN / n + b * FVector2D::DotProduct(P[i] - Centroid, Axis);
			}
			return true;
		}
	} // namespace

	bool MinNormLoadShares(TConstArrayView<FVector2D> Contacts, const FVector2D& Com, double WeightN, TArray<double>& OutShares)
	{
		const int32 N = Contacts.Num();
		OutShares.Init(0.0, N);
		if (N < 2 || !(WeightN > 0.0))
		{
			return false;
		}
		TArray<int32, TInlineAllocator<8>> Active;
		for (int32 i = 0; i < N; ++i)
		{
			Active.Add(i);
		}
		for (int32 Round = 0; Round <= N; ++Round)
		{
			if (Active.Num() < 2 || !SolveShares(Contacts, Active, Com, WeightN, OutShares))
			{
				OutShares.Init(0.0, N);
				return false;
			}
			int32  Worst = INDEX_NONE;
			double Least = -1e-6 * WeightN;
			for (int32 k = 0; k < Active.Num(); ++k)
			{
				if (OutShares[Active[k]] < Least)
				{
					Least = OutShares[Active[k]];
					Worst = k;
				}
			}
			if (Worst == INDEX_NONE)
			{
				for (double& Share : OutShares)
				{
					Share = FMath::Max(0.0, Share);
				}
				return true;
			}
			OutShares[Active[Worst]] = 0.0;
			Active.RemoveAt(Worst);
		}
		OutShares.Init(0.0, N);
		return false;
	}
} // namespace RammsCurb
