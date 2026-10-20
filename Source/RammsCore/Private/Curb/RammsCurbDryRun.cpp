// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbDryRun.h"

#include "Curb/RammsCurbTerrainFit.h"

namespace RammsCurb
{
	namespace
	{
		/** A trapezoid tracker: the command C moving at Cv toward Target. */
		void Track(double& C, double& Cv, double Target, double Speed, double Accel, double Dt)
		{
			const double Gap = Target - C;
			const double Want = FMath::Sign(Gap) * FMath::Min(Speed, FMath::Sqrt(2.0 * Accel * FMath::Abs(Gap)));
			Cv += FMath::Clamp(Want - Cv, -Accel * Dt, Accel * Dt);
			C += Cv * Dt;
			if ((Target - C) * Gap < 0.0 && FMath::Abs(Cv) <= Accel * Dt * 1.01)
			{
				C = Target;
				Cv = 0.0;
			}
		}

		/** What the dry run keeps of one leg. */
		struct FDryLeg
		{
			double C = 0.0;	 // commanded height (chassis z)
			double Cv = 0.0; // its rate
			double H = 0.0;	 // the leg's height, a lag behind
			double X = 0.0;	 // a 5-bar's fore/aft, a lag behind its command
			double CX = 0.0; // a 5-bar's commanded fore/aft
			/** The most its clearance profile asks over its window (world), and
			 *  where it is highest before the landing. */
			double ZTop = 0.0;
			double SDrop = 0.0;
		};

		/** The plan between its samples: the chassis height, the centre wheels'
		 *  fore/aft and the base's attitude. */
		struct FPlanAt
		{
			double B = 0.0;
			double X[2] = { 0.0, 0.0 };
			double Pitch = 0.0;
			double Roll = 0.0;
		};
	} // namespace

	FDryRunResult DryRun(const FPlannerInput& In, const FCurbSchedule& Plan, const FDryRunOptions& Options)
	{
		FDryRunResult Out;
		if (!Plan.IsValid() || Plan.Samples.Num() < 2 || Plan.Wheels.Num() != In.Legs.Num())
		{
			Out.Notes.Add(TEXT("no plan to run"));
			return Out;
		}
		FPlannerInput AtPlan = In;
		AtPlanOrigin(AtPlan, Plan);
		const FPlanModel		M(AtPlan);
		const FRammsCurbTuning& T = In.Tuning;
		const int32				N = M.NumLegs();
		const double			Dt = FMath::Max(1e-4, Options.Dt);
		const double			Tau = FMath::Max(1e-3, T.ServoLagSeconds * Options.LagScale);
		const double			LegSpeed = FMath::Max(1.0, static_cast<double>(T.LegSpeed));
		const double			LegAccel = FMath::Max(1.0, static_cast<double>(T.LegAcceleration));
		const double			CSS = FMath::Max(0.1, static_cast<double>(T.CenterShiftSpeed));
		const double			BH = FMath::Max(0.1, static_cast<double>(T.BaseHeightSpeed));
		const double			Floor = FMath::Clamp(static_cast<double>(T.MinSpeedFraction), 0.0, 1.0) * T.DriveSpeed;
		const double			SEnd = Plan.Samples.Last().S;
		const double			Ds = Plan.SampleCm;
		const int32				Last = Plan.Samples.Num() - 1;
		auto					At = [&](double S) -> const FScheduleSample& { return *Plan.SampleAt(S); };
		auto					Between = [&](double S) {
			   const double			  U = FMath::Clamp(S / Plan.SampleCm, 0.0, static_cast<double>(Last));
			   const int32			  K = FMath::Min(Last - 1, FMath::FloorToInt(U));
			   const FScheduleSample& P0 = Plan.Samples[K];
			   const FScheduleSample& P1 = Plan.Samples[K + 1];
			   // Past a planned stop the robot leaves from where it re-posed to.
			   const double A = P0.bStop ? 1.0 : U - K;
			   FPlanAt		Out;
			   Out.B = FMath::Lerp(P0.B, P1.B, A);
			   Out.X[0] = FMath::Lerp(P0.X[0], P1.X[0], A);
			   Out.X[1] = FMath::Lerp(P0.X[1], P1.X[1], A);
			   Out.Pitch = FMath::Lerp(P0.PitchDeg, P1.PitchDeg, A);
			   Out.Roll = FMath::Lerp(P0.RollDeg, P1.RollDeg, A);
			   return Out;
		};
		auto CentreSide = [&](int32 i) { return M.Side(i); };
		auto Note = [&](const FString& S) {
			if (Out.Notes.Num() < 8)
			{
				Out.Notes.Add(S);
			}
		};
		// The planned stops, and the runs under the floor about them (the slowing
		// into a stop and the rise out of it), where the speed may be low -- and
		// a few samples more, for the run's own following a tick behind.
		constexpr int32 StopPad = 3;
		TArray<int32>	Stops;
		TArray<uint8>	AtStop;
		AtStop.Init(0, Plan.Samples.Num());
		for (int32 k = 0; k <= Last; ++k)
		{
			if (!Plan.Samples[k].bStop)
			{
				continue;
			}
			Stops.Add(k);
			int32 j = k;
			for (; j >= 0 && (j == k || Plan.Samples[j].V < Floor - 1e-6); --j)
			{
				AtStop[j] = 1;
			}
			for (int32 p = 0; p < StopPad && j - p >= 0; ++p)
			{
				AtStop[j - p] = 1;
			}
			for (j = k; j <= Last && (j == k || Plan.Samples[j].V < Floor - 1e-6); ++j)
			{
				AtStop[j] = 1;
			}
			for (int32 p = 0; p < StopPad && j + p <= Last; ++p)
			{
				AtStop[j + p] = 1;
			}
		}

		// Each leg's lifted carry: the most its profile asks over the window,
		// and where its drop begins (as the planner times it).
		TArray<FDryLeg> Legs;
		Legs.SetNum(N);
		for (int32 i = 0; i < N; ++i)
		{
			const FWheelPlan& W = Plan.Wheels[i];
			FDryLeg&		  L = Legs[i];
			L.H = L.C = In.Legs[i].Measured.Y;
			L.X = L.CX = In.Legs[i].Measured.X;
			auto XAt = [&](double S, double Z) { return M.IsCentre(i) ? At(S).X[CentreSide(i)] : M.CornerX(i, Z - At(S).B); };
			// Carried at ZTop (world), a corner's wheel sits where its 4-bar puts it
			// at that height: the most the profile asks there, found again from
			// where the last guess put the wheel (from where it stands, first).
			L.ZTop = -TNumericLimits<double>::Max();
			for (int32 Pass = 0; Pass < 4; ++Pass)
			{
				double Top = -TNumericLimits<double>::Max();
				for (double S = W.SClear; S <= W.SLand + 1e-6; S += Ds)
				{
					const double ZAt = Pass == 0 ? In.Legs[i].Measured.Y + At(S).B : L.ZTop;
					Top = FMath::Max(Top, M.ClearanceZ(i, M.WheelXY(i, S, XAt(S, ZAt))));
				}
				L.ZTop = Pass == 0 ? Top : FMath::Max(L.ZTop, Top);
			}
			L.SDrop = W.SLand;
			double ZMax = -TNumericLimits<double>::Max();
			for (double S = W.SLand; S >= W.SLand - 2.5 * M.Radius(i); S -= Ds)
			{
				const double Z = M.ClearanceZ(i, M.WheelXY(i, S, XAt(S, L.ZTop)));
				if (Z > ZMax + 1e-6)
				{
					ZMax = Z;
					L.SDrop = S;
				}
			}
		}

		double S = 0.0, V = FMath::Max(0.0, In.EntrySpeedCmS), Time = 0.0;
		bool   bMoved = V >= 5.0, bRisen = V >= Floor, bRising = !bRisen;
		double Peak = V, StopFor = 0.0;
		// A planned stop under way: its sample, the chassis height at rest
		// moving to where the robot leaves from, and how long it has rested.
		int32  NextStop = 0;
		int32  Dwelling = INDEX_NONE;
		double DwellB = 0.0, DwellFor = 0.0;
		Out.MinSpeedCmS = TNumericLimits<double>::Max();
		const double Acc = FMath::Max(1.0, static_cast<double>(T.Acceleration));
		const double Dec = FMath::Max(1.0, static_cast<double>(T.Deceleration));
		while (S < SEnd && Time < Options.MaxSeconds)
		{
			FPlanAt Ahead, Now;
			if (Dwelling != INDEX_NONE)
			{
				// At rest at the stop while the centre wheels and the chassis move to
				// the next sample's (within CenterShiftSpeed and BaseHeightSpeed);
				// on once they are there and the legs have settled.
				const FScheduleSample& To = Plan.Samples[FMath::Min(Last, Dwelling + 1)];
				DwellB += FMath::Clamp(To.B - DwellB, -BH * Dt, BH * Dt);
				Now.B = DwellB;
				Now.X[0] = To.X[0];
				Now.X[1] = To.X[1];
				Now.Pitch = To.PitchDeg;
				Now.Roll = To.RollDeg;
				Ahead = Now;
				DwellFor += Dt;
				bool bThere = FMath::Abs(DwellB - To.B) < 1e-6;
				for (int32 i = 0; i < N; ++i)
				{
					bThere = bThere && FMath::Abs(Legs[i].C - Legs[i].H) < 0.05 && (!M.IsCentre(i) || FMath::Abs(Legs[i].X - Legs[i].CX) < 0.05);
				}
				if (bThere || DwellFor > 10.0)
				{
					if (!bThere)
					{
						Note(FString::Printf(TEXT("%.1f cm: the planned stop's re-pose did not settle in 10 s"), S));
					}
					Out.PlannedDwellSeconds += DwellFor;
					Dwelling = INDEX_NONE;
					++NextStop;
				}
				Time += Dt;
			}
			else
			{
				// The speed: the plan's a sample ahead (from rest, the first
				// sample's), within the ramps; braked onto a planned stop.
				double Want = At(S + Ds).V;
				if (Stops.IsValidIndex(NextStop))
				{
					const double SS = Plan.Samples[Stops[NextStop]].S;
					Want = FMath::Min(Want, FMath::Sqrt(2.0 * Dec * FMath::Max(0.0, SS - S)));
				}
				V = FMath::Max(0.0, V + FMath::Clamp(Want - V, -Dec * Dt, Acc * Dt));
				S += V * Dt;
				if (Stops.IsValidIndex(NextStop))
				{
					const double SS = Plan.Samples[Stops[NextStop]].S;
					if (S >= SS - 0.05 && V <= Dec * Dt * 2.0)
					{
						// At the stop.
						S = SS;
						V = 0.0;
						Dwelling = Stops[NextStop];
						DwellB = Plan.Samples[Dwelling].B;
						DwellFor = 0.0;
						++Out.PlannedStops;
					}
				}
				Time += Dt;
				// Legs are commanded a servo lag ahead: a leg that lags its command
				// by that much is where the plan wants it now. The plan's chassis
				// height, attitude and centre-wheel fore/aft between its samples,
				// linearly.
				Ahead = Between(S + V * T.ServoLagSeconds);
				Now = Between(S);
			}
			const FScheduleSample& Here = At(S);
			const double		   B = Now.B;
			const double		   SAhead = S + V * T.ServoLagSeconds;
			// Legs.
			uint32 Planted = 0;
			for (int32 i = 0; i < N; ++i)
			{
				const FWheelPlan& W = Plan.Wheels[i];
				FDryLeg&		  L = Legs[i];
				const bool		  bAir = W.bHeldFromStart ? S < W.SLand : (S >= W.SLiftStart && S < W.SDown);
				const bool		  bFar = S >= W.SDown;
				if (M.IsCentre(i))
				{
					L.CX = Ahead.X[CentreSide(i)];
					L.X += FMath::Clamp(L.CX - L.X, -CSS * Dt, CSS * Dt);
				}
				const double X = M.IsCentre(i) ? L.X : M.CornerX(i, L.H);
				double		 Target = 0.0;
				if (!bAir)
				{
					Target = M.LegZOnSurface(i, SAhead, X, Ahead.B, Ahead.Pitch, Ahead.Roll, bFar);
					Planted |= 1u << i;
				}
				else if (S < L.SDrop || (W.bHeldFromStart && S < W.SClear))
				{
					Target = M.LegZForWorldZ(i, X, L.ZTop, Ahead.B, Ahead.Pitch, Ahead.Roll);
				}
				else if (S < W.SLand)
				{
					double Need = M.SupportZ(i, M.WheelXY(i, SAhead, X), true);
					for (double Sa = S; Sa <= FMath::Min(W.SLand, S + V * 0.3); Sa += Ds)
					{
						Need = FMath::Max(Need, M.ClearanceZ(i, M.WheelXY(i, Sa, X)));
					}
					Target = M.LegZForWorldZ(i, X, Need, Ahead.B, Ahead.Pitch, Ahead.Roll);
				}
				else
				{
					Target = M.LegZOnSurface(i, SAhead, X, Ahead.B, Ahead.Pitch, Ahead.Roll, true);
				}
				Track(L.C, L.Cv, Target, LegSpeed, LegAccel, Dt);
				L.H += (L.C - L.H) * FMath::Min(1.0, Dt / Tau);
				if (M.IsCentre(i))
				{
					const FVector2D Reach = M.Reach(i, L.CX);
					if (Reach.X > Reach.Y || L.C < Reach.X - 1e-6 || L.C > Reach.Y + 1e-6)
					{
						++Out.KneeViolations;
						Note(FString::Printf(TEXT("%.1f cm: leg %d commanded to (%.1f, %.1f), outside its reach %.1f..%.1f"), S, i, L.CX, L.C,
							Reach.X, Reach.Y));
					}
				}
			}
			if (Here.bApproach)
			{
				continue;
			}
			// The checks.
			TArray<FVector2D, TInlineAllocator<MaxPlanLegs>> XZ;
			uint32											 Support = 0, OnFar = 0;
			double											 Sx = 0, Sy = 0, Sxx = 0, Syy = 0, Sxy = 0, Sr = 0, Sxr = 0, Syr = 0;
			FString											 Residuals;
			int32											 NumSupport = 0;
			for (int32 i = 0; i < N; ++i)
			{
				const FWheelPlan& W = Plan.Wheels[i];
				const FDryLeg&	  L = Legs[i];
				const double	  X = M.IsCentre(i) ? L.X : M.CornerX(i, L.H);
				const double	  D = M.DistancePast(i, S, X);
				// Carrying the robot: on the ground and loaded, as the plan has it
				// (a wheel just set down takes LoadSeconds to take its share).
				const bool bOnGround = ((Planted >> i) & 1u) && !((Here.AirborneMask >> i) & 1u);
				const bool bFar = S >= W.SDown;
				XZ.Add(FVector2D(X, L.H));
				if (bOnGround)
				{
					Support |= 1u << i;
					OnFar |= (bFar ? 1u : 0u) << i;
					if (D > W.DUp + 1e-3 && D < W.DDown - 1e-3)
					{
						++Out.KeepOffViolations;
						Note(FString::Printf(TEXT("%.1f cm: leg %d on the ground %.1f cm past the edge, inside %.1f..%.1f"), S, i, D, W.DUp,
							W.DDown));
					}
					// The base the legs' lag would give: each support leg off the
					// height its plane wants with the base as planned.
					const double Need = M.LegZOnSurface(i, S, X, B, Now.Pitch, Now.Roll, bFar);
					const double R = L.H - Need;
					Residuals += FString::Printf(TEXT(" %d:%+.2f"), i, R);
					const double Y = M.LegY(i);
					Sx += X;
					Sy += Y;
					Sxx += X * X;
					Syy += Y * Y;
					Sxy += X * Y;
					Sr += R;
					Sxr += X * R;
					Syr += Y * R;
					++NumSupport;
				}
				else if (D >= W.DUp && D <= W.DDown)
				{
					const FVector C = M.WheelCentre(i, S, X, L.H, B, Now.Pitch, Now.Roll);
					const double  Rim = M.ClearanceZ(i, FVector2D(C.X, C.Y)) - T.SwingClearance;
					if (C.Z - Rim < 0.0)
					{
						Note(FString::Printf(TEXT("%.1f cm: leg %d's wheel %.2f cm into the terrain, %.1f cm past the edge (window %.1f..%.1f; lift at "
												  "%.1f, clear by %.1f)"),
							S, i, Rim - C.Z, D, W.DUp, W.DDown, W.SLiftStart, W.SClear));
					}
					Out.MinSwingClearanceCm = FMath::Min(Out.MinSwingClearanceCm, C.Z - Rim);
				}
			}
			if (IsForbiddenSet(In.Model, Here.AirborneMask))
			{
				++Out.ForbiddenSets;
				Note(FString::Printf(TEXT("%.1f cm: a forbidden set is off the ground (mask %x)"), S, Here.AirborneMask));
			}
			// The lagging robot's margin, its base at the planned height and
			// attitude; standing still at a stop.
			const double Margin = M.PoseMarginAt(XZ, Support, OnFar, S, B, Now.Pitch, Now.Roll, Dwelling == INDEX_NONE);
			if (Margin < 0.0)
			{
				Note(FString::Printf(TEXT("%.1f cm: the robot stands %.2f cm outside its support (mask %x, the plan's off the ground %x)"), S, -Margin,
					Support, Here.AirborneMask));
			}
			Out.MinMarginCm = FMath::Min(Out.MinMarginCm, Margin);
			if (NumSupport >= 3)
			{
				// R = a + b x + c y, least squares: the tilt the support legs' errors
				// would put on a base standing on them, against the plan's.
				const double n = NumSupport;
				const double A11 = n, A12 = Sx, A13 = Sy, A22 = Sxx, A23 = Sxy, A33 = Syy;
				const double Det = A11 * (A22 * A33 - A23 * A23) - A12 * (A12 * A33 - A23 * A13) + A13 * (A12 * A23 - A22 * A13);
				if (FMath::Abs(Det) > 1e-9)
				{
					const double DetB = A11 * (Sxr * A33 - A23 * Syr) - Sr * (A12 * A33 - A23 * A13) + A13 * (A12 * Syr - Sxr * A13);
					const double DetC = A11 * (A22 * Syr - Sxr * A23) - A12 * (A12 * Syr - Sxr * A13) + Sr * (A12 * A23 - A22 * A13);
					const double Bx = DetB / Det, Cy = DetC / Det;
					const double Error = FMath::Max(FMath::Abs(FMath::RadiansToDegrees(FMath::Atan(Bx))), FMath::Abs(FMath::RadiansToDegrees(FMath::Atan(Cy))));
					if (Error > 0.5 && Error > Out.MaxAttitudeErrorDeg)
					{
						Note(FString::Printf(TEXT("%.1f cm: the support legs' lag tilts the base %.2f deg off the plan (residuals by leg, cm:%s)"), S,
							Error, *Residuals));
					}
					Out.MaxAttitudeErrorDeg = FMath::Max(Out.MaxAttitudeErrorDeg, Error);
				}
			}
			Out.MaxPlannedTiltDeg = FMath::Max(Out.MaxPlannedTiltDeg, InclinationDeg(Now.Pitch, Now.Roll));
			// The speed.
			bMoved = bMoved || V >= 5.0;
			if (bRising)
			{
				Peak = FMath::Max(Peak, V);
				if (V >= Floor || V < Peak - 2.0)
				{
					bRising = false;
					bRisen = true;
				}
			}
			const bool bAtStop = Dwelling != INDEX_NONE || AtStop[FMath::Clamp(FMath::RoundToInt(S / Ds), 0, Last)];
			if (bRisen && !bAtStop)
			{
				Out.MinSpeedCmS = FMath::Min(Out.MinSpeedCmS, V);
			}
			if (bMoved && !bAtStop)
			{
				StopFor = V <= 2.0 ? StopFor + Dt : 0.0;
				if (StopFor >= 0.2 && StopFor - Dt < 0.2)
				{
					++Out.Stops;
				}
			}
		}
		Out.bCompleted = S >= SEnd;
		Out.Seconds = Time;
		if (Out.MinSpeedCmS == TNumericLimits<double>::Max())
		{
			Out.MinSpeedCmS = 0.0;
		}
		return Out;
	}
} // namespace RammsCurb
