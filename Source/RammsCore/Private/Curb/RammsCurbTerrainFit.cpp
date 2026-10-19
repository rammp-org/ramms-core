// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbTerrainFit.h"

#include "Math/RandomStream.h"

namespace RammsCurb
{
	namespace
	{
		FVector2D XY(const FVector& V)
		{
			return FVector2D(V.X, V.Y);
		}

		double Residual(const FRammsCurbPlane& Plane, const FVector& P)
		{
			return P.Z - Plane.HeightAt(XY(P));
		}

		/** Least squares over the points Use marks, about their mean. False
		 *  when they fix no plane: fewer than three, or all on (nearly) one
		 *  line -- a single scan lane, say. */
		bool LeastSquaresPlane(TConstArrayView<FVector> Points, TConstArrayView<bool> Use, FRammsCurbPlane& Out)
		{
			double N = 0.0, Cx = 0.0, Cy = 0.0, Cz = 0.0;
			for (int32 i = 0; i < Points.Num(); ++i)
			{
				if (Use[i])
				{
					N += 1.0;
					Cx += Points[i].X;
					Cy += Points[i].Y;
					Cz += Points[i].Z;
				}
			}
			if (N < 3.0)
			{
				return false;
			}
			Cx /= N;
			Cy /= N;
			Cz /= N;
			double Sxx = 0.0, Sxy = 0.0, Syy = 0.0, Sxz = 0.0, Syz = 0.0;
			for (int32 i = 0; i < Points.Num(); ++i)
			{
				if (!Use[i])
				{
					continue;
				}
				const double Dx = Points[i].X - Cx;
				const double Dy = Points[i].Y - Cy;
				const double Dz = Points[i].Z - Cz;
				Sxx += Dx * Dx;
				Sxy += Dx * Dy;
				Syy += Dy * Dy;
				Sxz += Dx * Dz;
				Syz += Dy * Dz;
			}
			const double Det = Sxx * Syy - Sxy * Sxy;
			// The spread across the points' main direction: under a millimetre
			// RMS and they lie on one line, which any plane through it fits.
			const double Tr = Sxx + Syy;
			const double Minor = 0.5 * (Tr - FMath::Sqrt(FMath::Max(0.0, Tr * Tr - 4.0 * Det)));
			if (Minor / N < 0.01 || Det <= 0.0)
			{
				return false;
			}
			Out = FRammsCurbPlane();
			Out.Origin = FVector(Cx, Cy, Cz);
			Out.Gradient = FVector2D((Sxz * Syy - Syz * Sxy) / Det, (Syz * Sxx - Sxz * Sxy) / Det);
			return true;
		}

		/** The plane through three points; false when they are (nearly) in a
		 *  line seen from above. */
		bool PlaneThrough(const FVector& A, const FVector& B, const FVector& C, FRammsCurbPlane& Out)
		{
			const FVector U = B - A;
			const FVector V = C - A;
			const double  Det = U.X * V.Y - V.X * U.Y; // twice the triangle's area, cm^2
			if (FMath::Abs(Det) < 1.0)
			{
				return false;
			}
			Out = FRammsCurbPlane();
			Out.Origin = A;
			Out.Gradient = FVector2D((U.Z * V.Y - V.Z * U.Y) / Det, (U.X * V.Z - V.X * U.Z) / Det);
			return true;
		}

		/**
		 * The plane most of Points lie on: the best (most samples within
		 * PlaneRansacBandCm) of the least-squares plane of them all and of
		 * PlaneRansacIterations random three-point planes, refitted by least
		 * squares to the samples within the band of it. Its fit figures count
		 * the samples within MaxResidualCm; OutOff the rest. The random
		 * stream is seeded the same every time, so the same samples always
		 * give the same plane.
		 */
		bool RobustPlane(TConstArrayView<FVector> Points, double MaxResidualCm, FRammsCurbPlane& Out, int32& OutOff)
		{
			const int32 N = Points.Num();
			OutOff = N;
			if (N < 3)
			{
				return false;
			}
			TArray<bool> Use;
			Use.Init(true, N);
			TArray<bool> Best;
			int32		 BestCount = -1;
			auto		 Consider = [&](const FRammsCurbPlane& P) {
				int32 Count = 0;
				for (const FVector& V : Points)
				{
					Count += FMath::Abs(Residual(P, V)) <= PlaneRansacBandCm ? 1 : 0;
				}
				if (Count > BestCount)
				{
					BestCount = Count;
					Best.SetNum(N);
					for (int32 i = 0; i < N; ++i)
					{
						Best[i] = FMath::Abs(Residual(P, Points[i])) <= PlaneRansacBandCm;
					}
				}
			};
			FRammsCurbPlane Candidate;
			if (LeastSquaresPlane(Points, Use, Candidate))
			{
				Consider(Candidate);
			}
			FRandomStream Rng(0x5eed);
			for (int32 It = 0; It < PlaneRansacIterations; ++It)
			{
				const int32 A = Rng.RandRange(0, N - 1);
				const int32 B = Rng.RandRange(0, N - 1);
				const int32 C = Rng.RandRange(0, N - 1);
				if (A != B && B != C && A != C && PlaneThrough(Points[A], Points[B], Points[C], Candidate))
				{
					Consider(Candidate);
				}
			}
			FRammsCurbPlane Plane;
			if (BestCount < 3 || !LeastSquaresPlane(Points, Best, Plane))
			{
				return false;
			}
			for (int32 Pass = 0; Pass < 2; ++Pass)
			{
				for (int32 i = 0; i < N; ++i)
				{
					Use[i] = FMath::Abs(Residual(Plane, Points[i])) <= PlaneRansacBandCm;
				}
				FRammsCurbPlane Refit;
				if (!LeastSquaresPlane(Points, Use, Refit))
				{
					break;
				}
				Plane = Refit;
			}
			double Sum2 = 0.0, Max = 0.0;
			int32  Kept = 0;
			for (const FVector& V : Points)
			{
				const double R = FMath::Abs(Residual(Plane, V));
				if (R <= MaxResidualCm)
				{
					Sum2 += R * R;
					Max = FMath::Max(Max, R);
					++Kept;
				}
			}
			Plane.FitRmsCm = Kept > 0 ? static_cast<float>(FMath::Sqrt(Sum2 / Kept)) : 0.0f;
			Plane.FitMaxCm = static_cast<float>(Max);
			Plane.Samples = Kept;
			OutOff = N - Kept;
			Out = Plane;
			return Kept >= 3;
		}

		/** A fitted surface within the plane limits, or why not. */
		bool CheckPlane(const TCHAR* What, const FRammsCurbPlane& Plane, int32 Off, const FTerrainFitInput& In, FString& OutWhy)
		{
			const int32 N = Plane.Samples + Off;
			if (N > 0 && Plane.Samples < MinPlaneInlierFraction * N)
			{
				OutWhy = FString::Printf(TEXT("the %s surface is not a plane: %d of its %d samples lie more than %.1f cm "
											  "(MaxPlaneResidualCm) off the best fit"),
					What, Off, N, In.MaxPlaneResidualCm);
				return false;
			}
			if (Plane.FitRmsCm > In.MaxPlaneRmsCm)
			{
				OutWhy = FString::Printf(TEXT("the %s surface is not a plane: its samples lie %.2f cm RMS off the best "
											  "fit, more than MaxPlaneRmsCm %.2f"),
					What, Plane.FitRmsCm, In.MaxPlaneRmsCm);
				return false;
			}
			return true;
		}

		/** The surface function, counted. */
		struct FTracer
		{
			TFunctionRef<bool(const FVector2D&, double&)> Fn;
			int32										  Count = 0;

			bool operator()(const FVector2D& P, double& Z)
			{
				++Count;
				return Fn(P, Z);
			}
		};

		/** The scan's frame: lanes run along Heading at an offset towards Side
		 *  (+Y), positions along a lane measured from Origin. */
		struct FFrame
		{
			FVector2D Origin;
			FVector2D Heading;
			FVector2D Side;

			FVector2D At(double Offset, double T) const { return Origin + Heading * T + Side * Offset; }
		};

		/** One scan lane: its samples and what was found along it. */
		struct FLane
		{
			double		   Offset = 0.0;
			TArray<double> T;
			TArray<double> Z;
			TArray<bool>   Hit;
			/** The first sample at or past the rearmost wheel: edges are looked
			 *  for from here; the ones behind only help fit the near plane. */
			int32 ScanFrom = 0;

			/** The first edge: between samples Before and After, placed at
			 *  EdgeT; a drop out of view when bDrop. Jump is the height change
			 *  against the near plane. */
			bool   bEdge = false;
			bool   bDrop = false;
			int32  Before = INDEX_NONE;
			int32  After = INDEX_NONE;
			double EdgeT = 0.0;
			double Jump = 0.0;

			bool bUsed = false;

			/** The far side: to a second edge at EndT (bSecond), else seen to
			 *  EndT. */
			bool   bSecond = false;
			double EndT = 0.0;
		};

		/** Samples K+1 .. K+EdgePersistSamples missing too (as far as the scan
		 *  goes): the surface falls out of view at K, rather than one trace
		 *  missing. */
		bool MissingPersists(const FLane& L, int32 K)
		{
			for (int32 m = 1; m <= EdgePersistSamples; ++m)
			{
				const int32 Idx = K + m;
				if (Idx >= L.T.Num())
				{
					break;
				}
				if (L.Hit[Idx])
				{
					return false;
				}
			}
			return true;
		}

		/**
		 * A jump of Jump at sample K, against a reference that read RefR
		 * before it, that persists: the next EdgePersistSamples samples keep
		 * its sign and EdgePersistFraction of its size against the reference
		 * (Dev gives a sample's deviation). Missing samples continue a drop.
		 * Too near the end of the scan to tell is not an edge.
		 */
		template <typename DevFn>
		bool JumpPersists(const FLane& L, int32 K, double RefR, double Jump, DevFn Dev)
		{
			for (int32 m = 1; m <= EdgePersistSamples; ++m)
			{
				const int32 Idx = K + m;
				if (Idx >= L.T.Num())
				{
					return false;
				}
				if (!L.Hit[Idx])
				{
					if (Jump < 0.0)
					{
						continue;
					}
					return false;
				}
				const double D = Dev(Idx) - RefR;
				if (D * Jump <= 0.0 || FMath::Abs(D) < EdgePersistFraction * FMath::Abs(Jump))
				{
					return false;
				}
			}
			return true;
		}

		/** The level (by Dev) at sample K: the median of K and the next two
		 *  samples with a hit going Step (+1 on, -1 back); fewer where the scan
		 *  ends. A sample partway down a face is outvoted. */
		template <typename DevFn>
		double LevelAround(const FLane& L, int32 K, int32 Step, DevFn Dev)
		{
			double V[3] = { 0.0, 0.0, 0.0 };
			int32  N = 0;
			for (int32 k = K; k >= 0 && k < L.T.Num() && N < 3; k += Step)
			{
				if (L.Hit[k])
				{
					V[N++] = Dev(k);
				}
			}
			if (N < 3)
			{
				return N == 0 ? 0.0 : (N == 1 ? V[0] : 0.5 * (V[0] + V[1]));
			}
			return FMath::Max(FMath::Min(V[0], V[1]), FMath::Min(FMath::Max(V[0], V[1]), V[2]));
		}

		/** The first sample from K on (up to EdgePersistSamples further) whose
		 *  level (by Dev) is nearer FarLevel than NearLevel: past the halfway
		 *  point of the jump. */
		template <typename DevFn>
		int32 PastHalfway(const FLane& L, int32 K, double NearLevel, double FarLevel, DevFn Dev)
		{
			int32 k = K;
			for (int32 Extra = 0; Extra < EdgePersistSamples && k + 1 < L.T.Num(); ++Extra)
			{
				if (!L.Hit[k] || FMath::Abs(Dev(k) - FarLevel) < FMath::Abs(Dev(k) - NearLevel))
				{
					break;
				}
				++k;
			}
			return k;
		}

		/** Each lane's first edge against the near plane: the first jump of
		 *  more than MinStepHeight from the rearmost wheel on that persists,
		 *  or the surface falling out of view, bisected. */
		void ScanFirstEdge(FLane& L, const FRammsCurbPlane& Plane, const FTerrainFitInput& In, const FFrame& F, FTracer& Trace)
		{
			L.bEdge = L.bDrop = false;
			L.Before = L.After = INDEX_NONE;
			auto  R = [&](int32 k) { return L.Z[k] - Plane.HeightAt(F.At(L.Offset, L.T[k])); };
			int32 Prev = INDEX_NONE;
			for (int32 k = L.ScanFrom; k < L.T.Num(); ++k)
			{
				if (!L.Hit[k])
				{
					if (Prev != INDEX_NONE && MissingPersists(L, k))
					{
						L.bEdge = L.bDrop = true;
						L.Before = Prev;
						L.After = k;
						break;
					}
					continue;
				}
				if (Prev == INDEX_NONE)
				{
					Prev = k;
					continue;
				}
				const double Jump = R(k) - R(Prev);
				if (FMath::Abs(Jump) > In.MinStepHeightCm)
				{
					if (JumpPersists(L, k, R(Prev), Jump, R))
					{
						L.bEdge = true;
						L.Before = Prev;
						L.After = k;
						L.Jump = Jump;
						break;
					}
					continue; // a spike
				}
				Prev = k;
			}
			if (!L.bEdge)
			{
				return;
			}
			// The edge is where the surface is halfway between the levels either
			// side of the jump, each the median of three samples: a face that
			// leans into view puts the sample just before (or after) it partway
			// down, by however the lane's samples happen to fall, and halving
			// from that sample would move the edge with it.
			const double NearLevel = LevelAround(L, L.Before, -1, R);
			const double FarLevel = L.bDrop ? 0.0 : LevelAround(L, L.After, +1, R);
			// The sample after the jump may be less than halfway up a face
			// that leans into view: then the halfway point is past it.
			const int32 HiK = L.bDrop ? L.After : PastHalfway(L, L.After, NearLevel, FarLevel, R);
			double		Lo = L.T[HiK > L.After ? HiK - 1 : L.Before];
			double		Hi = L.T[HiK];
			for (int32 i = 0; i < EdgeBisections; ++i)
			{
				const double	Mid = 0.5 * (Lo + Hi);
				const FVector2D P = F.At(L.Offset, Mid);
				double			Z = 0.0;
				bool			bChanged = !Trace(P, Z);
				if (!bChanged)
				{
					const double Rm = Z - Plane.HeightAt(P);
					bChanged = L.bDrop ? FMath::Abs(Rm - NearLevel) > In.MinStepHeightCm
									   : FMath::Abs(Rm - FarLevel) < FMath::Abs(Rm - NearLevel);
				}
				(bChanged ? Hi : Lo) = Mid;
			}
			L.EdgeT = 0.5 * (Lo + Hi);
		}

		/**
		 * A lane's far side, past its own edge: a running straight-line fit of
		 * its far samples from FarPastEdgeCm on, against which the next sample
		 * is judged -- never the near plane, which the far side is a step off.
		 * A jump of more than MinStepHeight that persists, or the surface
		 * falling out of view, is a second edge (bisected); else the lane is
		 * seen to its last sample.
		 */
		void ScanFarSide(FLane& L, const FTerrainFitInput& In, const FFrame& F, FTracer& Trace)
		{
			L.bSecond = false;
			L.EndT = L.EdgeT;
			double N = 0.0, St = 0.0, Sz = 0.0, Stt = 0.0, Stz = 0.0;
			auto   Add = [&](int32 k) {
				  N += 1.0;
				  St += L.T[k];
				  Sz += L.Z[k];
				  Stt += L.T[k] * L.T[k];
				  Stz += L.T[k] * L.Z[k];
			};
			auto Predict = [&](double T) {
				if (N < 2.0)
				{
					return N > 0.0 ? Sz / N : 0.0;
				}
				const double Det = N * Stt - St * St;
				if (Det <= 1e-9)
				{
					return Sz / N;
				}
				const double B = (N * Stz - St * Sz) / Det;
				return (Sz - B * St) / N + B * T;
			};
			auto  Dev = [&](int32 k) { return L.Z[k] - Predict(L.T[k]); };
			int32 Last = INDEX_NONE;
			int32 Before = INDEX_NONE, After = INDEX_NONE;
			bool  bDrop = false;
			for (int32 k = 0; k < L.T.Num(); ++k)
			{
				if (L.T[k] < L.EdgeT + FarPastEdgeCm)
				{
					continue;
				}
				if (!L.Hit[k])
				{
					if (MissingPersists(L, k))
					{
						if (Last == INDEX_NONE)
						{
							return; // nothing in view past the edge
						}
						Before = Last;
						After = k;
						bDrop = true;
						break;
					}
					continue;
				}
				if (N == 0.0)
				{
					Add(k);
					Last = k;
					continue;
				}
				const double D = Dev(k);
				if (FMath::Abs(D) > In.MinStepHeightCm)
				{
					if (JumpPersists(L, k, 0.0, D, Dev))
					{
						Before = Last;
						After = k;
						break;
					}
					continue; // a spike
				}
				Add(k);
				Last = k;
			}
			if (Before == INDEX_NONE)
			{
				L.EndT = Last != INDEX_NONE ? L.T[Last] : L.EdgeT;
				return;
			}
			// Halfway between the far side's line and the level past the jump
			// (as for the first edge).
			const double Beyond = bDrop ? 0.0 : LevelAround(L, After, +1, Dev);
			const int32	 HiK = bDrop ? After : PastHalfway(L, After, 0.0, Beyond, Dev);
			double		 Lo = L.T[HiK > After ? HiK - 1 : Before];
			double		 Hi = L.T[HiK];
			for (int32 i = 0; i < EdgeBisections; ++i)
			{
				const double Mid = 0.5 * (Lo + Hi);
				double		 Z = 0.0;
				bool		 bChanged = !Trace(F.At(L.Offset, Mid), Z);
				if (!bChanged)
				{
					const double Dm = Z - Predict(Mid);
					bChanged = bDrop ? FMath::Abs(Dm) > In.MinStepHeightCm : FMath::Abs(Dm - Beyond) < FMath::Abs(Dm);
				}
				(bChanged ? Hi : Lo) = Mid;
			}
			L.bSecond = true;
			L.EndT = 0.5 * (Lo + Hi);
		}

		/** A straight line through points by total least squares: their mean,
		 *  main direction, a normal, and the farthest point's distance from it. */
		struct FLine
		{
			FVector2D Centre = FVector2D::ZeroVector;
			FVector2D Direction = FVector2D(1.0, 0.0);
			FVector2D Normal = FVector2D(0.0, 1.0);
			double	  Residual = 0.0;
		};

		bool FitLine(TConstArrayView<FVector2D> Points, FLine& Out)
		{
			if (Points.Num() < 2)
			{
				return false;
			}
			FVector2D C = FVector2D::ZeroVector;
			for (const FVector2D& P : Points)
			{
				C += P;
			}
			C /= Points.Num();
			double Sxx = 0.0, Sxy = 0.0, Syy = 0.0;
			for (const FVector2D& P : Points)
			{
				const FVector2D D = P - C;
				Sxx += D.X * D.X;
				Sxy += D.X * D.Y;
				Syy += D.Y * D.Y;
			}
			const double Theta = 0.5 * FMath::Atan2(2.0 * Sxy, Sxx - Syy);
			Out.Centre = C;
			Out.Direction = FVector2D(FMath::Cos(Theta), FMath::Sin(Theta));
			Out.Normal = FVector2D(-Out.Direction.Y, Out.Direction.X);
			Out.Residual = 0.0;
			for (const FVector2D& P : Points)
			{
				Out.Residual = FMath::Max(Out.Residual, FMath::Abs(FVector2D::DotProduct(Out.Normal, P - C)));
			}
			return true;
		}

		const TCHAR* LaneName(int32 Index, int32 Count)
		{
			// Lanes run from the -Y side to the +Y side; the outer two are past
			// the tyres, the next two along the outermost tyre edges.
			static const TCHAR* Names[] = { TEXT("-Y outer"), TEXT("-Y track"), TEXT("-Y middle"), TEXT("centre"),
				TEXT("+Y middle"), TEXT("+Y track"), TEXT("+Y outer") };
			return Count == UE_ARRAY_COUNT(Names) && Index >= 0 && Index < Count ? Names[Index] : TEXT("?");
		}

		/** Wheels on the plane: none more than WheelOnPlaneCm below it, and at
		 *  least MinWheelsOnPlane on it. */
		bool CheckWheelsOnPlane(const FRammsCurbPlane& Plane, TConstArrayView<FTerrainWheel> Wheels, const FVector2D& Heading,
			FString& OutWhy)
		{
			int32 On = 0;
			for (const FTerrainWheel& W : Wheels)
			{
				const double Rho = WheelCentreHeight(W.RadiusCm, W.ContactWidthCm, Plane.Gradient, Heading);
				const double Gap = W.Centre.Z - Rho - Plane.HeightAt(XY(W.Centre));
				if (Gap < -WheelOnPlaneCm)
				{
					OutWhy = FString::Printf(TEXT("the '%s' wheel stands %.1f cm below the surface the robot is on: it is "
												  "standing across a step (or in a hole); drive clear of it first"),
						*W.Name.ToString(), -Gap);
					return false;
				}
				On += Gap <= WheelOnPlaneCm ? 1 : 0;
			}
			if (On < MinWheelsOnPlane)
			{
				OutWhy = FString::Printf(TEXT("only %d of the robot's wheels stand on the surface under it (it needs %d)"), On,
					MinWheelsOnPlane);
				return false;
			}
			return true;
		}

		bool CheckWheelsShortOfEdge(const FRammsCurbTerrain& T, TConstArrayView<FTerrainWheel> Wheels, FString& OutWhy)
		{
			for (const FTerrainWheel& W : Wheels)
			{
				const double D = T.DistancePastEdge(XY(W.Centre));
				if (D >= 0.0)
				{
					OutWhy = FString::Printf(TEXT("the '%s' wheel is already %.1f cm past the edge: the robot is standing "
												  "across a step; drive clear of it first"),
						*W.Name.ToString(), D);
					return false;
				}
			}
			return true;
		}
	} // namespace

	// --- the scan --------------------------------------------------------------------

	float FFarDepthNeed::At(float ApproachDeg) const
	{
		const double A = FMath::DegreesToRadians(static_cast<double>(ApproachDeg));
		return static_cast<float>(WheelbaseCm * FMath::Cos(A) + TrackCm * FMath::Abs(FMath::Sin(A)) + ExtraCm);
	}

	void FTerrainFitInput::SetTuning(const FRammsCurbTuning& Tuning)
	{
		DetectRangeCm = Tuning.DetectRange;
		MinStepHeightCm = Tuning.MinStepHeight;
		MaxStepHeightCm = Tuning.MaxStepHeight;
		MaxApproachDeg = Tuning.MaxApproachDeg;
		MaxSurfaceSlopeDeg = Tuning.MaxSurfaceSlopeDeg;
		MaxEdgeResidualCm = Tuning.MaxEdgeResidualCm;
		MaxPlaneRmsCm = Tuning.MaxPlaneRmsCm;
		MaxPlaneResidualCm = Tuning.MaxPlaneResidualCm;
		FarFitDepthCm = Tuning.FarFitDepthCm;
	}

	double ApproachAngleDeg(const FVector2D& Heading, const FVector2D& EdgeNormal)
	{
		return FMath::RadiansToDegrees(
			FMath::Atan2(Heading.X * EdgeNormal.Y - Heading.Y * EdgeNormal.X, FVector2D::DotProduct(Heading, EdgeNormal)));
	}

	namespace
	{
		bool FitTerrainCore(const FTerrainFitInput& In, FTracer& Trace, FRammsCurbTerrain& OutTerrain, FString& OutWhy,
			FTerrainFitReport& Rep);
	} // namespace

	bool FitTerrain(const FTerrainFitInput& In, TFunctionRef<bool(const FVector2D&, double&)> SurfaceAt,
		FRammsCurbTerrain& OutTerrain, FString& OutWhy, FTerrainFitReport* OutReport)
	{
		OutTerrain = FRammsCurbTerrain();
		FTracer			   Trace{ SurfaceAt };
		FTerrainFitReport  Scratch;
		FTerrainFitReport& Rep = OutReport ? *OutReport : Scratch;
		Rep = FTerrainFitReport();
		const bool bFound = FitTerrainCore(In, Trace, OutTerrain, OutWhy, Rep);
		Rep.Traces = Trace.Count;
		if (!bFound)
		{
			OutTerrain = FRammsCurbTerrain();
		}
		return bFound;
	}

	namespace
	{
		bool FitTerrainCore(const FTerrainFitInput& In, FTracer& Trace, FRammsCurbTerrain& OutTerrain, FString& OutWhy,
			FTerrainFitReport& Rep)
		{
			const FVector2D H = In.Heading.GetSafeNormal();
			if (H.IsNearlyZero() || In.Wheels.Num() == 0)
			{
				OutWhy = TEXT("no heading or no wheels to look from");
				return false;
			}
			const FFrame F{ In.Origin, H, FVector2D(-H.Y, H.X) };

			// Where the wheels are in the scan's frame: the lanes reach from behind
			// the rearmost to DetectRange past the frontmost, and to LaneMarginCm
			// beyond the outermost tyre edges.
			double XRear = TNumericLimits<double>::Max(), XFront = -TNumericLimits<double>::Max(), HalfWidth = 0.0;
			for (const FTerrainWheel& W : In.Wheels)
			{
				const FVector2D D = XY(W.Centre) - In.Origin;
				const double	X = FVector2D::DotProduct(D, F.Heading);
				XRear = FMath::Min(XRear, X);
				XFront = FMath::Max(XFront, X);
				HalfWidth = FMath::Max(HalfWidth, FMath::Abs(FVector2D::DotProduct(D, F.Side)) + 0.5 * W.TyreWidthCm);
			}
			Rep.TrackHalfWidthCm = static_cast<float>(HalfWidth);
			const double Offsets[] = { -(HalfWidth + LaneMarginCm), -HalfWidth, -0.5 * HalfWidth, 0.0, 0.5 * HalfWidth,
				HalfWidth, HalfWidth + LaneMarginCm };
			const int32	 TrackLanes[] = { 1, 5 };
			const int32	 NumLanes = UE_ARRAY_COUNT(Offsets);

			TArray<FLane> Lanes;
			Lanes.SetNum(NumLanes);
			const double T0 = XRear - LaneMarginCm;
			const double T1 = XFront + In.DetectRangeCm;
			for (int32 l = 0; l < NumLanes; ++l)
			{
				FLane& L = Lanes[l];
				L.Offset = Offsets[l];
				for (double T = T0; T <= T1 + 1e-6; T += DetectSampleCm)
				{
					double	   Z = 0.0;
					const bool bHit = Trace(F.At(L.Offset, T), Z);
					L.T.Add(T);
					L.Z.Add(bHit ? Z : 0.0);
					L.Hit.Add(bHit);
				}
				L.ScanFrom = 0;
				while (L.ScanFrom < L.T.Num() && L.T[L.ScanFrom] < XRear)
				{
					++L.ScanFrom;
				}
			}

			// The ground under the robot: under each wheel, a 3 x 3 grid over the
			// footprint, and the lanes as far as the front wheels.
			TArray<FVector> Under;
			for (const FTerrainWheel& W : In.Wheels)
			{
				double Z = 0.0;
				if (Trace(XY(W.Centre), Z))
				{
					Under.Add(FVector(W.Centre.X, W.Centre.Y, Z));
				}
			}
			for (const double X : { XRear, 0.5 * (XRear + XFront), XFront })
			{
				for (const double Y : { -HalfWidth, 0.0, HalfWidth })
				{
					const FVector2D P = F.At(Y, X);
					double			Z = 0.0;
					if (Trace(P, Z))
					{
						Under.Add(FVector(P.X, P.Y, Z));
					}
				}
			}
			TArray<FVector> Points = Under;
			for (const FLane& L : Lanes)
			{
				for (int32 k = 0; k < L.T.Num() && L.T[k] <= XFront; ++k)
				{
					if (L.Hit[k])
					{
						const FVector2D P = F.At(L.Offset, L.T[k]);
						Points.Add(FVector(P.X, P.Y, L.Z[k]));
					}
				}
			}
			FRammsCurbPlane Near;
			int32			Off = 0;
			if (!RobustPlane(Points, In.MaxPlaneResidualCm, Near, Off))
			{
				OutWhy = TEXT("cannot see the ground under the robot");
				return false;
			}

			// Each lane's first edge against it; then the near plane again, from
			// every lane up to NearShortOfEdgeCm short of its edge (a lane with
			// none, until it leaves the plane), and the edges again against that.
			for (FLane& L : Lanes)
			{
				ScanFirstEdge(L, Near, In, F, Trace);
			}
			Points = Under;
			for (const FLane& L : Lanes)
			{
				for (int32 k = 0; k < L.T.Num(); ++k)
				{
					if (!L.Hit[k])
					{
						continue;
					}
					const FVector2D P = F.At(L.Offset, L.T[k]);
					const double	R = L.Z[k] - Near.HeightAt(P);
					if (L.bEdge ? L.T[k] > L.EdgeT - NearShortOfEdgeCm : (k >= L.ScanFrom && FMath::Abs(R) > In.MaxPlaneResidualCm))
					{
						break;
					}
					if (k < L.ScanFrom && FMath::Abs(R) > In.MaxPlaneResidualCm)
					{
						continue; // behind the robot, and on something else
					}
					Points.Add(FVector(P.X, P.Y, L.Z[k]));
				}
			}
			if (!RobustPlane(Points, In.MaxPlaneResidualCm, Near, Off))
			{
				OutWhy = TEXT("cannot fit a plane to the ground under and ahead of the robot");
				return false;
			}
			if (!CheckPlane(TEXT("near"), Near, Off, In, OutWhy))
			{
				return false;
			}
			if (In.bCheckWheels && !CheckWheelsOnPlane(Near, In.Wheels, F.Heading, OutWhy))
			{
				return false;
			}
			for (FLane& L : Lanes)
			{
				ScanFirstEdge(L, Near, In, F, Trace);
			}

			// No step, yet the ground leaves the plane: it bends -- a ramp, a
			// crowned road -- and the robot has nothing it can cross as a step.
			for (int32 l = 0; l < NumLanes; ++l)
			{
				const FLane& L = Lanes[l];
				const int32	 Last = L.bEdge ? L.Before : L.T.Num() - 1;
				for (int32 k = L.ScanFrom; k <= Last; ++k)
				{
					if (!L.Hit[k])
					{
						continue;
					}
					const double R = L.Z[k] - Near.HeightAt(F.At(L.Offset, L.T[k]));
					if (FMath::Abs(R) > In.MinStepHeightCm)
					{
						OutWhy = FString::Printf(TEXT("the near surface changes slope ahead (a ramp, or a crowned road?): %+.1f "
													  "cm off its plane %.0f cm ahead of the front wheels on the %s lane, with "
													  "no step"),
							R, L.T[k] - XFront, LaneName(l, NumLanes));
						return false;
					}
				}
			}

			Rep.Lanes.SetNum(NumLanes);
			TArray<int32> Have;
			for (int32 l = 0; l < NumLanes; ++l)
			{
				Rep.Lanes[l].OffsetCm = static_cast<float>(Lanes[l].Offset);
				Rep.Lanes[l].bEdge = Lanes[l].bEdge;
				Rep.Lanes[l].EdgeAtCm = static_cast<float>(Lanes[l].EdgeT);
				if (Lanes[l].bEdge)
				{
					Have.Add(l);
				}
			}
			if (Have.Num() == 0)
			{
				OutWhy = FString::Printf(TEXT("no step within %.0f cm ahead"), In.DetectRangeCm);
				return false;
			}
			for (const int32 Track : TrackLanes)
			{
				if (!Lanes[Track].bEdge)
				{
					OutWhy = FString::Printf(TEXT("no step along the %s side's outermost tyre track: the step there is under "
												  "MinStepHeight %.1f cm, or out of view"),
						Lanes[Track].Offset > 0.0 ? TEXT("+Y") : TEXT("-Y"), In.MinStepHeightCm);
					return false;
				}
			}
			if (Have.Num() < MinEdgeLanes)
			{
				OutWhy = FString::Printf(TEXT("a step on only %d of the %d scan lanes; the edge needs %d"), Have.Num(), NumLanes,
					MinEdgeLanes);
				return false;
			}

			// One straight edge through the lanes' edge points; one lane that
			// does not fit may go, so long as both tyre tracks keep theirs.
			auto EdgePoints = [&](TConstArrayView<int32> Use) {
				TArray<FVector2D> P;
				for (const int32 l : Use)
				{
					P.Add(F.At(Lanes[l].Offset, Lanes[l].EdgeT));
				}
				return P;
			};
			FLine		  Line;
			TArray<int32> Used = Have;
			FitLine(EdgePoints(Used), Line);
			if (Line.Residual > In.MaxEdgeResidualCm)
			{
				FLine BestLine;
				int32 BestDrop = INDEX_NONE;
				if (Have.Num() - 1 >= MinEdgeLanes)
				{
					for (const int32 Drop : Have)
					{
						if (Drop == TrackLanes[0] || Drop == TrackLanes[1])
						{
							continue;
						}
						TArray<int32> Rest = Have;
						Rest.Remove(Drop);
						FLine Trial;
						if (FitLine(EdgePoints(Rest), Trial) && (BestDrop == INDEX_NONE || Trial.Residual < BestLine.Residual))
						{
							BestLine = Trial;
							BestDrop = Drop;
						}
					}
				}
				if (BestDrop == INDEX_NONE || BestLine.Residual > In.MaxEdgeResidualCm)
				{
					OutWhy = FString::Printf(TEXT("edge residual %.1f cm > %.1f (MaxEdgeResidualCm): the edge is not straight"),
						BestDrop == INDEX_NONE ? Line.Residual : BestLine.Residual, In.MaxEdgeResidualCm);
					return false;
				}
				Line = BestLine;
				Used.Remove(BestDrop);
				Rep.DroppedLane = BestDrop;
			}
			for (const int32 l : Used)
			{
				Lanes[l].bUsed = true;
				Rep.Lanes[l].bUsed = true;
			}
			FVector2D N = Line.Normal;
			if (FVector2D::DotProduct(N, F.Heading) < 0.0)
			{
				N = -N;
			}
			const double Alpha = ApproachAngleDeg(F.Heading, N);
			if (FMath::Abs(Alpha) > In.MaxApproachDeg)
			{
				OutWhy = FString::Printf(TEXT("the edge is %.1f deg off square to the approach, more than MaxApproachDeg %.1f"),
					Alpha, In.MaxApproachDeg);
				return false;
			}
			const double	   NH = FVector2D::DotProduct(N, F.Heading);
			const FVector2D	   E0 = In.Origin + F.Heading * (FVector2D::DotProduct(N, Line.Centre - In.Origin) / NH);
			FRammsCurbTerrain& T = OutTerrain;
			T.EdgeNormal = N;
			T.EdgeDirection = FVector2D(-N.Y, N.X);
			T.EdgePoint = FVector(E0.X, E0.Y, Near.HeightAt(E0));
			T.ApproachAngleDeg = static_cast<float>(Alpha);
			T.EdgeFitResidualCm = static_cast<float>(Line.Residual);
			T.Near = Near;
			if (In.bCheckWheels && !CheckWheelsShortOfEdge(T, In.Wheels, OutWhy))
			{
				OutTerrain = FRammsCurbTerrain();
				return false;
			}
			int32 Drops = 0;
			for (const int32 l : Used)
			{
				Drops += Lanes[l].bDrop ? 1 : 0;
			}
			if (Drops * 2 >= Used.Num())
			{
				OutWhy = FString::Printf(TEXT("an edge %.0f cm ahead with nothing beyond it: a drop deeper than the robot can see"),
					-T.DistancePastEdge(FVector2D(In.Origin + F.Heading * XFront)) / NH);
				OutTerrain = FRammsCurbTerrain();
				return false;
			}

			// A face, not a ramp: from the last point on the near plane to the
			// first where the far side has settled -- past the face, and rising
			// no more over the next sample spacing than the steepest surface
			// accepted -- is at most MaxFaceWidthCm across the edge. Judged on
			// the lanes' own samples, before the far side is scanned: a steep
			// ramp would have that scan start on the ramp.
			const double FlatRise = DetectSampleCm * FMath::Tan(FMath::DegreesToRadians(In.MaxSurfaceSlopeDeg)) + FaceOnPlaneCm;
			for (const int32 l : Used)
			{
				const FLane& L = Lanes[l];
				if (L.bDrop)
				{
					continue;
				}
				const double Step = 0.5;
				const double Reach = 2.0 * MaxFaceWidthCm;
				double		 NearEnd = L.EdgeT - Reach - Step;
				double		 FarStart = L.EdgeT + Reach + Step;
				for (double s = 0.0; s <= Reach; s += Step)
				{
					const FVector2D P = F.At(L.Offset, L.EdgeT - s);
					double			Z = 0.0;
					if (Trace(P, Z) && FMath::Abs(Z - Near.HeightAt(P)) <= FaceOnPlaneCm)
					{
						NearEnd = L.EdgeT - s;
						break;
					}
				}
				for (double s = 0.0; s <= Reach; s += Step)
				{
					const FVector2D P0 = F.At(L.Offset, L.EdgeT + s);
					const FVector2D P1 = F.At(L.Offset, L.EdgeT + s + DetectSampleCm);
					double			Z0 = 0.0, Z1 = 0.0;
					if (Trace(P0, Z0) && Trace(P1, Z1) && FMath::Abs(Z0 - Near.HeightAt(P0)) > 0.5 * FMath::Abs(L.Jump)
						&& FMath::Abs(Z1 - Z0) <= FlatRise)
					{
						FarStart = L.EdgeT + s;
						break;
					}
				}
				const double Width = (FarStart - NearEnd) * NH;
				if (Width > MaxFaceWidthCm)
				{
					OutWhy = FString::Printf(TEXT("a ramp, not a step: the ground takes %s%.1f cm across the edge to go from the "
												  "near surface to the far one on the %s lane (a face is at most %.1f)"),
						(NearEnd < L.EdgeT - Reach || FarStart > L.EdgeT + Reach) ? TEXT("more than ") : TEXT(""), Width,
						LaneName(l, NumLanes), MaxFaceWidthCm);
					OutTerrain = FRammsCurbTerrain();
					return false;
				}
			}

			// The far side, lane by lane: how deep it goes, and the samples to fit
			// it from, FarPastEdgeCm to FarFitDepthCm past the edge and short of
			// any second edge.
			const double Needed = In.FarDepthNeed.IsSet() ? In.FarDepthNeed.At(static_cast<float>(Alpha)) : 0.0;
			Rep.NeededFarDepthCm = static_cast<float>(Needed);
			double			ViewDepth = TNumericLimits<double>::Max();
			double			SeenDepth = TNumericLimits<double>::Max(); // to a second edge actually seen
			bool			bViewObserved = false;
			TArray<FVector> FarPoints;
			for (const int32 l : Used)
			{
				FLane& L = Lanes[l];
				ScanFarSide(L, In, F, Trace);
				const double Depth = T.DistancePastEdge(F.At(L.Offset, L.EndT));
				Rep.Lanes[l].bSecondEdge = L.bSecond;
				Rep.Lanes[l].FarDepthCm = static_cast<float>(Depth);
				if (Depth < ViewDepth)
				{
					ViewDepth = Depth;
					bViewObserved = L.bSecond;
				}
				if (L.bSecond)
				{
					SeenDepth = FMath::Min(SeenDepth, Depth);
				}
				const double Upper = FMath::Min(static_cast<double>(In.FarFitDepthCm), L.bSecond ? Depth - FarPastEdgeCm : Depth);
				for (int32 k = 0; k < L.T.Num(); ++k)
				{
					const FVector2D P = F.At(L.Offset, L.T[k]);
					const double	D = T.DistancePastEdge(P);
					if (L.Hit[k] && D >= FarPastEdgeCm && D <= Upper && (!L.bSecond || L.T[k] < L.EndT))
					{
						FarPoints.Add(FVector(P.X, P.Y, L.Z[k]));
					}
				}
			}
			T.FarDepthCm = static_cast<float>(FMath::Max(0.0, ViewDepth));
			T.bFarDepthObserved = bViewObserved;
			if (Needed > 0.0 && SeenDepth < Needed)
			{
				double Rise = 0.0;
				for (const int32 l : Used)
				{
					Rise += Lanes[l].Jump;
				}
				OutWhy = FString::Printf(TEXT("the %s beyond the edge is %.0f cm deep; the robot needs %.0f"),
					Rise >= 0.0 ? TEXT("sidewalk") : TEXT("ground"), SeenDepth, Needed);
				return false;
			}
			if (Needed > 0.0 && T.FarDepthCm < Needed && T.FarDepthCm < MinPlanFarDepthCm)
			{
				OutWhy = FString::Printf(TEXT("only %.0f cm of the far side is in view (DetectRange %.0f); the robot needs %.0f "
											  "to start and %.0f to cross"),
					T.FarDepthCm, In.DetectRangeCm, MinPlanFarDepthCm, Needed);
				OutTerrain = FRammsCurbTerrain();
				return false;
			}
			FRammsCurbPlane Far;
			if (FarPoints.Num() < MinFarPlaneSamples || !RobustPlane(FarPoints, In.MaxPlaneResidualCm, Far, Off))
			{
				OutWhy = FString::Printf(TEXT("too little of the far side in view to fit its surface (%d samples)"), FarPoints.Num());
				OutTerrain = FRammsCurbTerrain();
				return false;
			}
			if (!CheckPlane(TEXT("far"), Far, Off, In, OutWhy))
			{
				OutTerrain = FRammsCurbTerrain();
				return false;
			}
			T.Far = Far;

			// The step along the stretch of edge the robot's footprint crosses:
			// one sign, and within the limits at both ends.
			const double Band = HalfWidth / FMath::Max(0.1, FMath::Cos(FMath::DegreesToRadians(Alpha))) + FootprintBandMarginCm;
			const double S0 = T.StepAt(0.0);
			const double SA = T.StepAt(-Band);
			const double SB = T.StepAt(Band);
			const double SLo = FMath::Min(SA, SB);
			const double SHi = FMath::Max(SA, SB);
			T.StepHeightCm = static_cast<float>(S0);
			T.StepHeightRangeCm = FVector2D(SLo, SHi);
			if (SLo * SHi <= 0.0)
			{
				OutWhy = FString::Printf(TEXT("the step changes sign along the edge: %+.1f to %+.1f cm across the robot's footprint"),
					SA, SB);
				OutTerrain = FRammsCurbTerrain();
				return false;
			}
			const double Least = FMath::Min(FMath::Abs(SA), FMath::Abs(SB));
			const double Most = FMath::Max(FMath::Abs(SA), FMath::Abs(SB));
			if (Least < In.MinStepHeightCm)
			{
				OutWhy = FString::Printf(TEXT("the step is %+.1f to %+.1f cm across the robot's footprint: under MinStepHeight "
											  "%.1f cm at one end"),
					SLo, SHi, In.MinStepHeightCm);
				OutTerrain = FRammsCurbTerrain();
				return false;
			}
			if (Most > In.MaxStepHeightCm)
			{
				OutWhy = FString::Printf(TEXT("the step is %+.1f to %+.1f cm across the robot's footprint: over MaxStepHeight "
											  "%.1f cm"),
					SLo, SHi, In.MaxStepHeightCm);
				OutTerrain = FRammsCurbTerrain();
				return false;
			}
			for (const FRammsCurbPlane* Plane : { &T.Near, &T.Far })
			{
				if (Plane->SlopeDeg() > In.MaxSurfaceSlopeDeg)
				{
					OutWhy = FString::Printf(TEXT("the %s surface slopes %.1f deg, more than MaxSurfaceSlopeDeg %.1f"),
						Plane == &T.Near ? TEXT("near") : TEXT("far"), Plane->SlopeDeg(), In.MaxSurfaceSlopeDeg);
					OutTerrain = FRammsCurbTerrain();
					return false;
				}
			}

			// Done. How far the edge is from the front wheel nearest it.
			double Nearest = TNumericLimits<double>::Max();
			bool   bAnyFront = false;
			for (const FTerrainWheel& W : In.Wheels)
			{
				bAnyFront |= W.bFront;
			}
			for (const FTerrainWheel& W : In.Wheels)
			{
				if (W.bFront || !bAnyFront)
				{
					Nearest = FMath::Min(Nearest, -T.DistancePastEdge(XY(W.Centre)) / NH);
				}
			}
			Rep.EdgeDistanceCm = static_cast<float>(Nearest);
			T.bValid = true;
			OutWhy = FString::Printf(TEXT("%+.1f cm step (%+.1f..%+.1f across the footprint) %.1f cm ahead, %+.1f deg off square "
										  "(edge residual %.2f cm over %d lanes); near slope %.1f deg, far slope %.1f deg; far "
										  "side %.0f cm deep%s%s"),
				S0, SLo, SHi, Nearest, Alpha, Line.Residual, Used.Num(), T.Near.SlopeDeg(), T.Far.SlopeDeg(), T.FarDepthCm,
				T.bFarDepthObserved ? TEXT(" (to a second edge)") : TEXT(" (to the end of view)"),
				Needed > 0.0 ? *FString::Printf(TEXT(", the robot needs %.0f"), Needed) : TEXT(""));
			return true;
		}
	} // namespace

	bool CheckWheelsOnNear(const FRammsCurbTerrain& Terrain, TConstArrayView<FTerrainWheel> Wheels, const FVector2D& Heading,
		FString& OutWhy)
	{
		return CheckWheelsOnPlane(Terrain.Near, Wheels, Heading.GetSafeNormal(), OutWhy)
			&& CheckWheelsShortOfEdge(Terrain, Wheels, OutWhy);
	}

	EFarDepth FarDepthVerdict(const FRammsCurbTerrain& Terrain, float NeededCm, FString* OutWhy)
	{
		const TCHAR* FarSide = Terrain.StepHeightCm >= 0.0f ? TEXT("sidewalk") : TEXT("ground");
		if (Terrain.bFarDepthObserved && Terrain.FarDepthCm < NeededCm)
		{
			if (OutWhy)
			{
				*OutWhy = FString::Printf(TEXT("the %s beyond the edge is %.0f cm deep; the robot needs %.0f"), FarSide,
					Terrain.FarDepthCm, NeededCm);
			}
			return EFarDepth::TooShallow;
		}
		if (Terrain.FarDepthCm >= NeededCm)
		{
			return EFarDepth::Enough;
		}
		if (Terrain.FarDepthCm < MinPlanFarDepthCm)
		{
			if (OutWhy)
			{
				*OutWhy = FString::Printf(TEXT("only %.0f cm of the far side is in view; the robot needs %.0f to start and %.0f "
											   "to cross"),
					Terrain.FarDepthCm, MinPlanFarDepthCm, NeededCm);
			}
			return EFarDepth::OutOfView;
		}
		if (OutWhy)
		{
			*OutWhy = FString::Printf(TEXT("the %s beyond the edge is seen %.0f cm deep so far; the robot needs %.0f"), FarSide,
				Terrain.FarDepthCm, NeededCm);
		}
		return EFarDepth::Pending;
	}

	// --- contact geometry -------------------------------------------------------------

	float ContactWidth(float TyreWidthCm, float ContactWidthCm)
	{
		return ContactWidthCm > 0.0f ? ContactWidthCm : FMath::Max(0.0f, TyreWidthCm);
	}

	double WheelCentreHeight(double RadiusCm, double ContactWidthCm, const FVector2D& Gradient, const FVector2D& RollDirection)
	{
		const FVector2D Fw = RollDirection.GetSafeNormal();
		const FVector2D Axle(-Fw.Y, Fw.X);
		const double	Gf = FVector2D::DotProduct(Gradient, Fw);
		const double	Ga = FVector2D::DotProduct(Gradient, Axle);
		return RadiusCm * FMath::Sqrt(1.0 + Gf * Gf) + 0.5 * ContactWidthCm * FMath::Abs(Ga);
	}

	double ClearanceHeight(const FRammsCurbTerrain& Terrain, const FVector2D& Centre, const FVector2D& RollDirection,
		double RadiusCm, double TyreWidthCm, double DilationCm)
	{
		const FVector2D		   Fw = RollDirection.GetSafeNormal();
		const FVector2D		   Axle(-Fw.Y, Fw.X);
		const bool			   bClimb = Terrain.StepHeightCm >= 0.0f;
		const FRammsCurbPlane& High = bClimb ? Terrain.Far : Terrain.Near;
		const FRammsCurbPlane& Low = bClimb ? Terrain.Near : Terrain.Far;
		// The higher side reaches DilationCm into the lower: a climb's curb top
		// out over its face, a descent's top out past its lip.
		const double Boundary = bClimb ? -DilationCm : DilationCm;
		auto		 IsHigh = [&](const FVector2D& P) {
			const double D = Terrain.DistancePastEdge(P);
			return bClimb ? D >= Boundary : D < Boundary;
		};
		const double R = RadiusCm;
		const double NF = FVector2D::DotProduct(Terrain.EdgeNormal, Fw);
		double		 Best = -TNumericLimits<double>::Max();
		const int32	 Across = TyreWidthCm > 0.0 ? 3 : 1;
		for (int32 j = 0; j < Across; ++j)
		{
			const double	V = Across == 1 ? 0.0 : (j - 1) * 0.5 * TyreWidthCm;
			const FVector2D Line = Centre + Axle * V;
			for (int32 k = 0; k <= 8; ++k)
			{
				const double	U = -R + 2.0 * R * k / 8.0;
				const FVector2D P = Line + Fw * U;
				const double	Surface = IsHigh(P) ? High.HeightAt(P) : Low.HeightAt(P);
				Best = FMath::Max(Best, Surface + FMath::Sqrt(FMath::Max(0.0, R * R - U * U)));
			}
			// Where this line crosses the (dilated) corner: the rim is highest
			// over the higher side there, nearest the wheel's middle.
			if (FMath::Abs(NF) > 1e-9)
			{
				const double U = (Boundary - Terrain.DistancePastEdge(Line)) / NF;
				if (FMath::Abs(U) <= R)
				{
					const FVector2D P = Line + Fw * U;
					Best = FMath::Max(Best, FMath::Max(High.HeightAt(P), Low.HeightAt(P)) + FMath::Sqrt(FMath::Max(0.0, R * R - U * U)));
				}
			}
		}
		return Best;
	}

	// --- the legacy profile, and the approach's second looks -------------------------

	void FillLegacyProfile(const FRammsCurbTerrain& Terrain, TConstArrayView<FTerrainWheel> Wheels, const FVector2D& Origin,
		const FVector2D& Heading, FRammsCurbProfile& OutProfile)
	{
		const FVector2D H = Heading.GetSafeNormal();
		const FVector2D Side(-H.Y, H.X);
		const double	NH = FMath::Max(1e-6, FVector2D::DotProduct(Terrain.EdgeNormal, H));
		bool			bAnyFront = false;
		for (const FTerrainWheel& W : Wheels)
		{
			bAnyFront |= W.bFront;
		}
		double Nearest = TNumericLimits<double>::Max();
		double YLo = TNumericLimits<double>::Max(), YHi = -TNumericLimits<double>::Max();
		for (const FTerrainWheel& W : Wheels)
		{
			if (!W.bFront && bAnyFront)
			{
				continue;
			}
			Nearest = FMath::Min(Nearest, -Terrain.DistancePastEdge(XY(W.Centre)) / NH);
			const double Y = FVector2D::DotProduct(XY(W.Centre) - Origin, Side);
			YLo = FMath::Min(YLo, Y);
			YHi = FMath::Max(YHi, Y);
		}
		OutProfile.StepHeightCm = Terrain.StepHeightCm;
		OutProfile.EdgeDistanceCm = Nearest < TNumericLimits<double>::Max() ? static_cast<float>(Nearest) : 0.0f;
		OutProfile.EdgeSkewCm = YHi > YLo
			? static_cast<float>((YHi - YLo) * FMath::Abs(FMath::Tan(FMath::DegreesToRadians(Terrain.ApproachAngleDeg))))
			: 0.0f;
		OutProfile.ApproachAngleDeg = Terrain.ApproachAngleDeg;
		OutProfile.Terrain = Terrain;
	}

	FRammsCurbTerrain LevelTerrain(double GroundZ, double StepHeightCm, const FVector2D& EdgePoint, const FVector2D& Heading)
	{
		FRammsCurbTerrain T;
		const FVector2D	  H = Heading.GetSafeNormal();
		T.bValid = true;
		T.EdgeNormal = H;
		T.EdgeDirection = FVector2D(-H.Y, H.X);
		T.EdgePoint = FVector(EdgePoint.X, EdgePoint.Y, GroundZ);
		T.Near.Origin = T.EdgePoint;
		T.Far.Origin = FVector(EdgePoint.X, EdgePoint.Y, GroundZ + StepHeightCm);
		T.StepHeightCm = static_cast<float>(StepHeightCm);
		T.StepHeightRangeCm = FVector2D(StepHeightCm, StepHeightCm);
		return T;
	}

	FString FTerrainChange::Describe() const
	{
		return FString::Printf(TEXT("edge %.2f cm / %.2f deg, near %.2f cm / %.2f deg, far %.2f cm / %.2f deg"), EdgeCm, EdgeDeg,
			NearCm, NearDeg, FarCm, FarDeg);
	}

	FTerrainChange CompareTerrain(const FRammsCurbTerrain& From, const FRammsCurbTerrain& To)
	{
		auto GradientDeg = [](const FVector2D& A, const FVector2D& B) {
			return FMath::RadiansToDegrees(FMath::Atan((A - B).Size()));
		};
		const FVector2D P = XY(From.EdgePoint);
		FTerrainChange	C;
		C.EdgeCm = FMath::Abs(To.DistancePastEdge(P));
		C.EdgeDeg = FMath::RadiansToDegrees(
			FMath::Acos(FMath::Clamp(FVector2D::DotProduct(From.EdgeNormal.GetSafeNormal(), To.EdgeNormal.GetSafeNormal()), -1.0, 1.0)));
		C.NearCm = FMath::Abs(To.Near.HeightAt(P) - From.Near.HeightAt(P));
		C.NearDeg = GradientDeg(To.Near.Gradient, From.Near.Gradient);
		C.FarCm = FMath::Abs(To.Far.HeightAt(P) - From.Far.HeightAt(P));
		C.FarDeg = GradientDeg(To.Far.Gradient, From.Far.Gradient);
		return C;
	}

	FRammsCurbTerrain BlendTerrain(const FRammsCurbTerrain& From, const FRammsCurbTerrain& To, double Weight,
		const FVector2D& Heading)
	{
		const double	  W = FMath::Clamp(Weight, 0.0, 1.0);
		FRammsCurbTerrain T = To; // the newest fit's figures
		auto			  Blend = [W](const FRammsCurbPlane& A, const FRammsCurbPlane& B) {
			 FRammsCurbPlane P = B;
			 const FVector2D O = XY(A.Origin);
			 P.Origin = FVector(O.X, O.Y, FMath::Lerp(A.HeightAt(O), B.HeightAt(O), W));
			 P.Gradient = FMath::Lerp(A.Gradient, B.Gradient, W);
			 return P;
		};
		T.Near = Blend(From.Near, To.Near);
		T.Far = Blend(From.Far, To.Far);
		FVector2D N = FMath::Lerp(From.EdgeNormal, To.EdgeNormal, W).GetSafeNormal();
		if (N.IsNearlyZero())
		{
			N = To.EdgeNormal;
		}
		T.EdgeNormal = N;
		T.EdgeDirection = FVector2D(-N.Y, N.X);
		const FVector2D E = FMath::Lerp(XY(From.EdgePoint), XY(To.EdgePoint), W);
		T.EdgePoint = FVector(E.X, E.Y, T.Near.HeightAt(E));
		T.ApproachAngleDeg = static_cast<float>(ApproachAngleDeg(Heading.GetSafeNormal(), N));
		T.StepHeightCm = static_cast<float>(T.StepAt(0.0));
		T.StepHeightRangeCm = FMath::Lerp(From.StepHeightRangeCm, To.StepHeightRangeCm, W);
		// Depth: a second edge seen beats any view; the nearer of two seen;
		// the deeper of two views.
		if (From.bFarDepthObserved && To.bFarDepthObserved)
		{
			T.FarDepthCm = FMath::Min(From.FarDepthCm, To.FarDepthCm);
		}
		else if (From.bFarDepthObserved || To.bFarDepthObserved)
		{
			T.FarDepthCm = From.bFarDepthObserved ? From.FarDepthCm : To.FarDepthCm;
			T.bFarDepthObserved = true;
		}
		else
		{
			T.FarDepthCm = FMath::Max(From.FarDepthCm, To.FarDepthCm);
		}
		T.bValid = From.bValid && To.bValid;
		return T;
	}
} // namespace RammsCurb
