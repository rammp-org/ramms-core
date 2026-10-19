// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbExecutor.h"

#include "Curb/RammsCurbRunStats.h"
#include "Curb/RammsCurbTerrainFit.h"

namespace RammsCurb
{
	namespace
	{
		/** A trapezoid tracker: the command C moving at Cv toward Target, at most
		 *  Speed, its speed changing by at most Accel. */
		void TrackTrapezoid(double& C, double& Cv, double Target, double Speed, double Accel, double Dt)
		{
			const double Gap = Target - C;
			const double Want = FMath::Sign(Gap) * FMath::Min(Speed, FMath::Sqrt(2.0 * Accel * FMath::Abs(Gap)));
			Cv += FMath::Clamp(Want - Cv, -Accel * Dt, Accel * Dt);
			C += Cv * Dt;
			if ((Target - C) * Gap <= 0.0 && FMath::Abs(Cv) <= Accel * Dt * 1.01)
			{
				C = Target;
				Cv = 0.0;
			}
		}

		double StepTowards(double Value, double Target, double MaxStep)
		{
			return Value + FMath::Clamp(Target - Value, -MaxStep, MaxStep);
		}

		/** Whether every attitude Plan calls for inclines the base no more than
		 *  the tuning's MaxPlannedTiltDeg (its total tilt, not pitch and roll
		 *  each); the planner verified it, and a plan from elsewhere (an
		 *  older one, a file) is held to it too. False, with why, where one
		 *  does not. */
		bool PlannedAttitudeWithinLimit(const FCurbSchedule& Plan, const FRammsCurbTuning& Tuning, FString& OutWhy)
		{
			const double Limit = FMath::Max(0.0, static_cast<double>(Tuning.MaxPlannedTiltDeg));
			for (const FScheduleSample& Sm : Plan.Samples)
			{
				const double Incl = InclinationDeg(Sm.PitchDeg, Sm.RollDeg);
				if (Incl > Limit + 1e-3)
				{
					OutWhy = FString::Printf(TEXT("the plan inclines the base %.2f deg at %.0f cm (%+.2f pitch, %+.2f roll), over MaxPlannedTiltDeg %.2f"),
						Incl, Sm.S, Sm.PitchDeg, Sm.RollDeg, Limit);
					return false;
				}
			}
			return true;
		}

		bool IsAirborne(ERammsCurbWheelState State)
		{
			return State == ERammsCurbWheelState::Lifting || State == ERammsCurbWheelState::Clear || State == ERammsCurbWheelState::Planting
				|| State == ERammsCurbWheelState::Held;
		}
	} // namespace

	const TCHAR* ExecPhaseName(EExecPhase Phase)
	{
		switch (Phase)
		{
			case EExecPhase::Approach:
				return TEXT("Approach");
			case EExecPhase::Prepare:
				return TEXT("Prepare");
			case EExecPhase::Cross:
				return TEXT("Cross");
			case EExecPhase::Hold:
				return TEXT("Hold");
			case EExecPhase::Settle:
				return TEXT("Settle");
			case EExecPhase::Halted:
				return TEXT("Halted");
			default:
				return TEXT("Done");
		}
	}

	const TCHAR* WheelStateName(ERammsCurbWheelState State)
	{
		switch (State)
		{
			case ERammsCurbWheelState::Planted:
				return TEXT("planted");
			case ERammsCurbWheelState::Lifting:
				return TEXT("lifting");
			case ERammsCurbWheelState::Clear:
				return TEXT("clear");
			case ERammsCurbWheelState::Planting:
				return TEXT("planting");
			case ERammsCurbWheelState::Loading:
				return TEXT("loading");
			default:
				return TEXT("held");
		}
	}

	const TCHAR* ExecTermName(EExecTerm Term)
	{
		switch (Term)
		{
			case EExecTerm::Plan:
				return TEXT("the plan's own speed");
			case EExecTerm::Deadline:
				return TEXT("a wheel's deadline");
			case EExecTerm::LevelIn:
				return TEXT("the level-in");
			case EExecTerm::Track:
				return TEXT("the centre wheels lagging their walk");
			case EExecTerm::PlannedStop:
				return TEXT("a planned stop");
			case EExecTerm::Attitude:
				return TEXT("the base off its planned attitude");
			case EExecTerm::Margin:
				return TEXT("the robot measured too near the edge of its wheels (MinLiveStabilityMarginCm)");
			case EExecTerm::Swing:
				return TEXT("a clear wheel's rim too near the terrain under it");
			default:
				return TEXT("the far side unconfirmed (the commit point)");
		}
	}

	FExecutor::FExecutor() = default;
	FExecutor::~FExecutor() = default;

	bool FExecutor::IsSupport(int32 Leg) const
	{
		return Wheels.IsValidIndex(Leg)
			&& (Wheels[Leg].State == ERammsCurbWheelState::Planted || Wheels[Leg].State == ERammsCurbWheelState::Loading);
	}

	bool FExecutor::HasCrossingBegun() const
	{
		for (int32 i = 0; i < Wheels.Num(); ++i)
		{
			const ERammsCurbWheelState St = Wheels[i].State;
			if (St == ERammsCurbWheelState::Lifting || St == ERammsCurbWheelState::Clear || Wheels[i].Plane == 1
				|| (St == ERammsCurbWheelState::Planting && !Wheels[i].bLower))
			{
				return true;
			}
		}
		return false;
	}

	void FExecutor::TakeLog(TArray<FString>& Out)
	{
		Out.Append(MoveTemp(Pending));
		Pending.Reset();
	}

	void FExecutor::Log(const FString& Line)
	{
		Pending.Add(FString::Printf(TEXT("s %6.1f cm  v %5.1f  %s"), S, VCmd, *Line));
	}

	bool FExecutor::Begin(const FPlannerInput& InInput, const FCurbSchedule& InPlan, const FExecMeasure& Measure, FString& OutWhy)
	{
		bRunning = false;
		if (!InPlan.IsValid() || InPlan.Samples.Num() < 2 || InPlan.Wheels.Num() != InInput.Legs.Num()
			|| InInput.Model.Legs.Num() != InInput.Legs.Num() || Measure.Legs.Num() != InInput.Legs.Num())
		{
			OutWhy = TEXT("no plan to run, or one that does not fit the robot");
			return false;
		}
		if (!PlannedAttitudeWithinLimit(InPlan, InInput.Tuning, OutWhy))
		{
			return false;
		}
		M.Reset();
		In = InInput;
		M = MakeUnique<FPlanModel>(In);
		Plan = InPlan;
		Counts = FExecCounts();
		Pending.Reset();
		Centre[0] = Centre[1] = INDEX_NONE;
		const int32 N = In.Legs.Num();
		for (int32 i = 0; i < N; ++i)
		{
			if (M->IsCentre(i))
			{
				Centre[M->Side(i)] = i;
			}
		}
		Wheels.Reset();
		Wheels.SetNum(N);
		for (int32 i = 0; i < N; ++i)
		{
			FExecWheel& W = Wheels[i];
			W.State = In.Legs[i].bOffGround ? ERammsCurbWheelState::Held : ERammsCurbWheelState::Planted;
			W.Plane = 0;
			W.bLower = In.Legs[i].bOffGround && !Plan.Wheels[i].bHeldFromStart;
			W.CmdZ = W.TargetZ = Measure.Legs[i].XZ.Y;
			W.CmdVz = 0.0;
			W.CmdX = Measure.Legs[i].XZ.X;
		}
		Phase = EExecPhase::Approach;
		Status = TEXT("Approach");
		Elapsed = 0.0;
		S = FVector2D::DotProduct(Measure.OriginXY - Plan.Origin, Plan.Heading);
		VCmd = FMath::Max(0.0, In.EntrySpeedCmS);
		VHat = VCmd;
		BCmd = BCmdLag = Measure.OriginZ;
		BOffset = 0.0;
		bLevelInBegun = false;
		bLoweredDown = false;
		LevelRampFrom = -1.0;
		RampRate = RampAccel = 1.0;
		RampSpan = 0.0;
		bDwelling = false;
		DwellFor = 0.0;
		SResume = 0.0;
		bHolding = false;
		bCentresHeld = false;
		bReachLowering = false;
		bSupportCeiling = false;
		ReachLowerCm = 0.0;
		SlowBy.Reset();
		SlowAt = -1.0;
		LatestAxle = ERammsCurbAxle::Front;
		ExternalCap = TNumericLimits<double>::Max();
		// The runtime checks start afresh.
		bFault = false;
		FaultWhy.Reset();
		bHalted = false;
		HaltDecel = 0.0;
		bHaltSetUp = false;
		bStopped = false;
		bLegsHome = false;
		AtRest.Reset();
		TerrainOff.Reset();
		PlantedHigh[0].Reset();
		PlantedHigh[1].Reset();
		AttitudeOff.Reset();
		AttitudeSlow.Reset();
		bAttitudeSlow = false;
		MarginWatch.Reset();
		bMarginHold = false;
		LiveMarginCm = 0.0;
		bSwingStop = false;
		LevelInFrom = -1.0;
		bReplanAsk = false;
		Replan = FReplanAsk();
		ReplanAskedAt = -TNumericLimits<double>::Max();
		SwingRaise = 0.0;
		Diag = FExecDiag();
		Prepare();
		bRunning = true;
		Log(FString::Printf(TEXT("per-wheel crossing begins: %s; the level-in from %.1f cm to %.1f (by %.1f at the latest), done at %.1f cm"),
			PlanClassName(Plan.Class), Plan.LevelInFromS, Plan.LevelFromS, Plan.LevelInLatestS, SEnd));
		return true;
	}

	bool FExecutor::Adopt(const FPlannerInput& InInput, const FCurbSchedule& InPlan, FString& OutWhy)
	{
		if (!bRunning)
		{
			OutWhy = TEXT("no crossing under way");
			return false;
		}
		if (HasCrossingBegun())
		{
			OutWhy = TEXT("a wheel has begun to cross: the plan in force is kept");
			return false;
		}
		if (!InPlan.IsValid() || InPlan.Samples.Num() < 2 || InPlan.Wheels.Num() != Wheels.Num() || InInput.Legs.Num() != Wheels.Num())
		{
			OutWhy = TEXT("the new plan does not fit the robot");
			return false;
		}
		if (!PlannedAttitudeWithinLimit(InPlan, InInput.Tuning, OutWhy))
		{
			return false;
		}
		// Where the robot is along the new plan: from where it was along the old
		// one (until the next tick measures it).
		const FVector2D Was = Plan.Origin + Plan.Heading * S;
		M.Reset();
		In = InInput;
		M = MakeUnique<FPlanModel>(In);
		Plan = InPlan;
		S = FVector2D::DotProduct(Was - Plan.Origin, Plan.Heading);
		SResume = 0.0;
		for (int32 i = 0; i < Wheels.Num(); ++i)
		{
			FExecWheel& W = Wheels[i];
			// A wheel held up whose new plan sets it down (or the other way) keeps
			// what the level-in already did with it.
			if (W.State == ERammsCurbWheelState::Held && !bLevelInBegun)
			{
				W.bLower = !Plan.Wheels[i].bHeldFromStart;
			}
			W.bBlocked = false;
			W.BlockedWhy.Reset();
			W.bDriftLogged = false;
		}
		bReplanAsk = false;
		Prepare();
		Log(FString::Printf(TEXT("re-planned: %s; done at %.1f cm"), PlanClassName(Plan.Class), SEnd));
		return true;
	}

	void FExecutor::Halt(const FString& Why)
	{
		if (!bRunning || bHalted)
		{
			return;
		}
		bHalted = true;
		bHaltSetUp = false;
		bStopped = false;
		bLegsHome = false;
		AtRest.Reset();
		// Braked at Deceleration -- twice that where a hard braking cap was
		// already under the command (an imminent collision).
		const double Dec = FMath::Max(1.0, static_cast<double>(In.Tuning.Deceleration));
		HaltDecel = VCmd > Diag.Braking + EmergencyOverCmS ? 2.0 * Dec : Dec;
		Phase = EExecPhase::Halted;
		Status = FString::Printf(TEXT("Halted: %s"), *Why);
		Log(FString::Printf(TEXT("halted (%s): braking from %.1f cm/s at %.0f cm/s^2, every leg holding its target"), *Why, VCmd, HaltDecel));
	}

	void FExecutor::Fault(const FString& Why)
	{
		if (!bRunning || bHalted)
		{
			return;
		}
		bFault = true;
		FaultWhy = Why;
		Log(FString::Printf(TEXT("fault: %s"), *Why));
		Halt(Why);
	}

	bool FExecutor::TakeReplanAsk(FReplanAsk& Out)
	{
		if (!bReplanAsk)
		{
			return false;
		}
		bReplanAsk = false;
		Out = Replan;
		return true;
	}

	void FExecutor::AskReplan(const FString& Why, const FVector2D& RefitCm, bool bHeading)
	{
		if (bHalted || bReplanAsk || HasCrossingBegun() || S - ReplanAskedAt < ReplanEveryCm)
		{
			return;
		}
		bReplanAsk = true;
		Replan.Why = Why;
		Replan.RefitCm = RefitCm;
		Replan.bHeading = bHeading;
		ReplanAskedAt = S;
		++Counts.ReplanAsks;
		Log(FString::Printf(TEXT("re-plan asked: %s"), *Why));
	}

	double FExecutor::SwingRaiseCm(const FLevelState& Level) const
	{
		// The higher side -- a climb's far surface, a descent's near one -- found
		// above its plane: a lifted wheel goes over it that much higher.
		const FVector2D Bias = Level.GetBias();
		return FMath::Max(0.0, M && M->IsClimb() ? Bias.Y : Bias.X);
	}

	void FExecutor::MeasureWheels(const FExecMeasure& Measure, const FVector2D& Bias, double Dt)
	{
		const FRammsCurbTerrain& Terrain = M->GetTerrain();
		for (int32 i = 0; i < Wheels.Num(); ++i)
		{
			FExecWheel&		W = Wheels[i];
			const FExecLeg& L = Measure.Legs[i];
			const FVector2D XY(L.WheelWorld.X, L.WheelWorld.Y);
			W.PastEdgeCm = Terrain.DistancePastEdge(XY);
			if (W.bMeasZ && Dt > 1e-6)
			{
				W.MeasVz += ((L.XZ.Y - W.MeasZ) / Dt - W.MeasVz) * FMath::Min(1.0, Dt / StallFilterSeconds);
			}
			W.MeasZ = L.XZ.Y;
			W.bMeasZ = true;
			W.LandedPastCm = W.PastEdgeCm;
			if (!M->IsCentre(i))
			{
				const double Shift = M->CornerX(i, M->SupportZ(i, XY, true) - Measure.OriginZ) - L.XZ.X;
				W.LandedPastCm = Terrain.DistancePastEdge(XY + Measure.Forward * Shift);
			}
			const bool bFar = IsAirborne(W.State) && W.State != ERammsCurbWheelState::Planting ? W.PastEdgeCm >= 0.0 : W.Plane == 1;
			W.RawBottomCm = L.WheelWorld.Z - M->ContactCentreZ(i, XY, bFar, Measure.AttitudeDeg.X, Measure.AttitudeDeg.Y);
			W.BottomCm = W.RawBottomCm - (bFar ? Bias.Y : Bias.X);
		}
	}

	double PredictedLiftMargin(const FPlanModel& M, const FCurbSchedule& Plan, int32 Leg, double S, TConstArrayView<double> UpUntilS, double& OutAt)
	{
		// From S until the wheel has taken load again, at the plan's poses: it
		// up, and every late wheel -- up where the plan has it down by now --
		// still up until UpUntilS (one on time lands as the plan has it). Only
		// where those sets are not the plan's own -- there its verification
		// holds (a lift on the plan's position, no wheel late, checks nothing).
		OutAt = S;
		if (!Plan.Wheels.IsValidIndex(Leg) || Plan.Samples.Num() == 0)
		{
			return TNumericLimits<double>::Max();
		}
		const FWheelPlan& P = Plan.Wheels[Leg];
		const double	  Ds = Plan.SampleCm;
		const int32		  Last = Plan.Samples.Num() - 1;
		const int32		  K0 = FMath::Clamp(FMath::CeilToInt(S / Ds), 0, Last);
		const int32		  K1 = FMath::Clamp(FMath::CeilToInt(P.SLoaded / Ds), K0, Last);
		const int32		  NW = Plan.Wheels.Num();
		// The centre 5-bars, by side.
		int32 Centre[2] = { INDEX_NONE, INDEX_NONE };
		for (int32 j = 0; j < NW && j < M.NumLegs(); ++j)
		{
			if (M.IsCentre(j))
			{
				Centre[M.Side(j)] = j;
			}
		}
		double Least = TNumericLimits<double>::Max();
		for (int32 k = K0; k <= K1; k += 2)
		{
			const FScheduleSample& Sm = Plan.Samples[k];
			if (Sm.S >= P.SLoaded)
			{
				// It has taken load here: down, as the plan has it (the plan may
				// lift the next wheel on this very sample).
				break;
			}
			if (Sm.bApproach)
			{
				continue;
			}
			uint32 Air = Sm.AirborneMask | (1u << Leg);
			for (int32 j = 0; j < NW && j < UpUntilS.Num(); ++j)
			{
				Air |= j != Leg && Sm.S < UpUntilS[j] ? 1u << j : 0u;
			}
			if (Air == Sm.AirborneMask)
			{
				continue;
			}
			FInstantQuery Q;
			Q.S = Sm.S;
			Q.PitchDeg = Sm.PitchDeg;
			Q.RollDeg = Sm.RollDeg;
			Q.Airborne = Air;
			for (int32 j = 0; j < NW; ++j)
			{
				const FWheelPlan& Pj = Plan.Wheels[j];
				Q.OnFar |= !((Air >> j) & 1u) && Sm.S >= Pj.SLoaded ? 1u << j : 0u;
				Q.Touching |= ((Sm.AirborneMask >> j) & 1u) && Sm.S + 0.5 * Ds >= Pj.SDown ? 1u << j : 0u;
			}
			for (int32 Side = 0; Side < 2; ++Side)
			{
				if (Centre[Side] != INDEX_NONE)
				{
					Q.X[Centre[Side]] = Sm.X[Side];
				}
			}
			const FInstantResult R = M.Evaluate(Q, Sm.B, /*bAtGuess=*/true);
			const double		 Margin = R.Support >= 3 ? R.Margin : -1e9;
			if (Margin < Least)
			{
				Least = Margin;
				OutAt = Sm.S;
			}
		}
		return Least;
	}

	double FExecutor::PredictedMargin(int32 Leg, const FExecMeasure& Measure, double& OutAt) const
	{
		// Each wheel up now that the plan has down by now: up until it has been
		// set down and loaded -- the travel to where it may be (LandingWork's
		// gate), then its work at the speed the robot has, LateWorkFactor times
		// as long (it is late: slower than planned).
		TArray<double, TInlineAllocator<MaxPlanLegs>> UpUntil;
		const double								  V = FMath::Max(FMath::Max(VHat, VCmd), 1.0);
		for (int32 j = 0; j < Wheels.Num(); ++j)
		{
			double Until = -TNumericLimits<double>::Max();
			if (j != Leg && !IsSupport(j) && S >= Plan.Wheels[j].SLoaded)
			{
				double Gate = 0.0, Work = 0.0;
				LandingWork(j, Measure, Gate, Work);
				Until = S + Gate + V * Work * LateWorkFactor;
			}
			UpUntil.Add(Until);
		}
		return PredictedLiftMargin(*M, Plan, Leg, S, UpUntil, OutAt);
	}

	void FExecutor::SetCommanded(int32 Leg, double X, double Z)
	{
		if (Wheels.IsValidIndex(Leg))
		{
			Wheels[Leg].CmdX = X;
			if (!FMath::IsNearlyEqual(Wheels[Leg].CmdZ, Z, 1e-4))
			{
				Wheels[Leg].CmdZ = Z;
				Wheels[Leg].CmdVz = 0.0;
			}
		}
	}

	FExecutor::FPlanAt FExecutor::Between(double SAt) const
	{
		FPlanAt				   Out;
		const int32			   Last = Plan.Samples.Num() - 1;
		const double		   U = FMath::Clamp(SAt / Plan.SampleCm, 0.0, static_cast<double>(Last));
		const int32			   K = FMath::Min(Last - 1, FMath::FloorToInt(U));
		const FScheduleSample& P0 = Plan.Samples[K];
		const FScheduleSample& P1 = Plan.Samples[K + 1];
		const double		   A = P0.bStop ? 1.0 : U - K;
		Out.B = FMath::Lerp(P0.B, P1.B, A);
		Out.Pitch = FMath::Lerp(P0.PitchDeg, P1.PitchDeg, A);
		Out.Roll = FMath::Lerp(P0.RollDeg, P1.RollDeg, A);
		Out.X[0] = FMath::Lerp(P0.X[0], P1.X[0], A);
		Out.X[1] = FMath::Lerp(P0.X[1], P1.X[1], A);
		Out.V = FMath::Lerp(P0.V, P1.V, U - K);
		return Out;
	}

	double FExecutor::PlanSpeedAt(double SAt) const
	{
		return Between(FMath::Max(SAt, SResume + Plan.SampleCm)).V;
	}

	bool FExecutor::IsLiftStalledOnGround(int32 Leg, const FExecMeasure& Measure) const
	{
		const FExecWheel& W = Wheels[Leg];
		const FExecLeg&	  L = Measure.Legs[Leg];
		if (W.State != ERammsCurbWheelState::Lifting || W.BottomCm > TouchCm)
		{
			return false;
		}
		// Unloading, its leg is pressed up by the ground until the lag model has it
		// above where the ground holds it; still there once the model is
		// TrackingErrorCm above it, and not rising, it has stalled.
		const bool bBehindModel = L.LagZ - L.XZ.Y > TrackingErrorCm && W.MeasVz <= StallSpeedCmS;
		return bBehindModel || Elapsed - W.Since > UnloadStallFactor * UnloadSeconds;
	}

	void FExecutor::Prepare()
	{
		const int32	 N = Wheels.Num();
		const double Ds = Plan.SampleCm;
		// Each lifted wheel's carry, as the planner timed it: the most its
		// clearance profile asks over its window -- a corner at the fore/aft its
		// 4-bar puts it at that height -- and where its drop begins.
		for (int32 i = 0; i < N; ++i)
		{
			const FWheelPlan& P = Plan.Wheels[i];
			FExecWheel&		  W = Wheels[i];
			auto			  XAt = [&](double At, double WorldZ) {
				 const FPlanAt A = Between(At);
				 return M->IsCentre(i) ? A.X[M->Side(i)] : M->CornerX(i, WorldZ - A.B);
			};
			W.ZTop = -TNumericLimits<double>::Max();
			for (int32 Pass = 0; Pass < 4; ++Pass)
			{
				double Top = -TNumericLimits<double>::Max();
				for (double At = P.SClear; At <= P.SLand + 1e-6; At += Ds)
				{
					const double ZAt = Pass == 0 ? In.Legs[i].Measured.Y + Between(At).B : W.ZTop;
					Top = FMath::Max(Top, M->ClearanceZ(i, M->WheelXY(i, At, XAt(At, ZAt))));
				}
				W.ZTop = Pass == 0 ? Top : FMath::Max(W.ZTop, Top);
			}
			W.SDrop = P.SLand;
			double ZMax = -TNumericLimits<double>::Max();
			for (double At = P.SLand; At >= P.SLand - 2.5 * M->Radius(i); At -= Ds)
			{
				const double Z = M->ClearanceZ(i, M->WheelXY(i, At, XAt(At, W.ZTop)));
				if (Z > ZMax + 1e-6)
				{
					ZMax = Z;
					W.SDrop = At;
				}
			}
		}
		// Done: past the last load, the centre wheels home (where they began),
		// every wheel's trailing tyre FinishClearance past the edge.
		double LastLoaded = 0.0;
		for (const FWheelPlan& P : Plan.Wheels)
		{
			LastLoaded = FMath::Max(LastLoaded, P.SLoaded);
		}
		const FRammsCurbTuning& T = In.Tuning;
		SEnd = Plan.Samples.Last().S;
		for (int32 k = FMath::Clamp(FMath::CeilToInt(LastLoaded / Ds), 0, Plan.Samples.Num() - 1); k < Plan.Samples.Num(); ++k)
		{
			const FScheduleSample& Sm = Plan.Samples[k];
			bool				   bHome = true;
			for (int32 Side = 0; Side < 2 && bHome; ++Side)
			{
				if (Centre[Side] != INDEX_NONE)
				{
					bHome = FMath::Abs(Sm.X[Side] - In.Legs[Centre[Side]].Measured.X) <= 0.25;
				}
			}
			for (int32 i = 0; i < N && bHome; ++i)
			{
				double X = M->IsCentre(i) ? Sm.X[M->Side(i)] : In.Legs[i].Measured.X;
				if (!M->IsCentre(i))
				{
					for (int32 Pass = 0; Pass < 2; ++Pass)
					{
						X = M->CornerX(i, M->SupportZ(i, M->WheelXY(i, Sm.S, X), true) - Sm.B);
					}
				}
				bHome = M->DistancePast(i, Sm.S, X) - M->Radius(i) >= T.FinishClearance;
			}
			if (bHome)
			{
				SEnd = Sm.S;
				break;
			}
		}
		NextStop = INDEX_NONE;
		for (int32 k = 0; k < Plan.Samples.Num(); ++k)
		{
			if (Plan.Samples[k].bStop && Plan.Samples[k].S > S - 1.0)
			{
				NextStop = k;
				break;
			}
		}
	}

	double FExecutor::ReachRoom() const
	{
		// The chassis may come down below the plan's height as far as the plan's
		// corridor here lets every leg still reach (BLo, LegRangeMargin kept).
		// (The plan has no corridor before its level-in ends.)
		const FScheduleSample* Sm = Plan.SampleAt(S);
		if (!Sm || bDwelling || S < Plan.LevelFromS)
		{
			return 0.0;
		}
		return FMath::Max(0.0, Between(S).B - (Sm->BLo + In.Tuning.LegRangeMargin));
	}

	double FExecutor::ReachDrop() const
	{
		double Drop = 0.0;
		for (const FExecWheel& W : Wheels)
		{
			if ((W.State == ERammsCurbWheelState::Planting || W.State == ERammsCurbWheelState::Loading) && W.bReachClamped)
			{
				Drop = FMath::Max(Drop, W.ReachDeficitCm);
			}
		}
		return Drop;
	}

	double FExecutor::ReachGate(int32 Leg) const
	{
		// What the chassis cannot come down by here (ReachRoom) comes with travel.
		const double Need = Wheels[Leg].ReachDeficitCm - ReachRoom();
		if (Need <= 0.05)
		{
			return 0.0;
		}
		const double From = Between(S).B;
		for (double D = 0.0; D <= ReachGateCm; D += 1.0)
		{
			if (Between(S + D).B <= From - Need + 0.05)
			{
				return D;
			}
		}
		return 0.0;
	}

	double FExecutor::SecondsTo(double SAt) const
	{
		const double Ds = Plan.SampleCm;
		double		 Seconds = 0.0;
		for (double At = S; At < SAt - 1e-6; At += Ds)
		{
			const double Step = FMath::Min(Ds, SAt - At);
			Seconds += Step / FMath::Max(0.5, FMath::Min(FMath::Max(VHat, VCmd), PlanSpeedAt(At + 0.5 * Step)));
		}
		return Seconds;
	}

	void FExecutor::SetState(int32 Leg, ERammsCurbWheelState NewState, const FString& Why)
	{
		FExecWheel& W = Wheels[Leg];
		if (W.State == NewState)
		{
			return;
		}
		Log(FString::Printf(TEXT("%-12s %s -> %s, %.1f cm past the edge (window %.1f..%.1f)%s%s"), *In.Legs[Leg].Name.ToString(),
			WheelStateName(W.State), WheelStateName(NewState), W.PastEdgeCm, Plan.Wheels[Leg].DUp, Plan.Wheels[Leg].DDown,
			Why.IsEmpty() ? TEXT("") : TEXT(": "), *Why));
		W.State = NewState;
		W.Since = Elapsed;
		W.Loading = 0.0;
		W.PlantingFor = 0.0;
		W.SwingLow.Reset();
		W.LandHigh.Reset();
		W.NoGround.Reset();
		W.Press.Reset();
		LatestAxle = M->Axle(Leg);
	}

	double FExecutor::ClampToReach(int32 Leg, double X, double Z) const
	{
		const double	m = In.Tuning.LegRangeMargin;
		const FVector2D R = M->Reach(Leg, X);
		if (R.X > R.Y)
		{
			return Z;
		}
		const double Lo = R.X + m, Hi = R.Y - m;
		return Lo <= Hi ? FMath::Clamp(Z, Lo, Hi) : 0.5 * (R.X + R.Y);
	}

	double FExecutor::SupportTarget(int32 Leg, const FExecMeasure& Measure, const FPlanAt& Ahead, double AheadCm, double Preload,
		const FVector2D& Bias) const
	{
		const FExecWheel& W = Wheels[Leg];
		const FExecLeg&	  L = Measure.Legs[Leg];
		const bool		  bFar = W.Plane == 1;
		// Where the wheel will be a servo lag from now.
		const FVector2D XY = FVector2D(L.WheelWorld.X, L.WheelWorld.Y) + Measure.Forward * AheadCm;
		const double	WorldZ = M->ContactCentreZ(Leg, XY, bFar, Ahead.Pitch, Ahead.Roll) + (bFar ? Bias.Y : Bias.X) - Preload;
		return M->LegZForWorldZ(Leg, L.XZ.X, WorldZ, BCmd, Ahead.Pitch, Ahead.Roll);
	}

	double FExecutor::SwingTarget(int32 Leg, const FExecMeasure& Measure, const FPlanAt& Ahead, double WorldZ) const
	{
		const FExecLeg& L = Measure.Legs[Leg];
		const double	X = L.XZ.X;
		// In the planned attitude at the commanded height, and in the attitude
		// and at the height measured: whichever asks more (a tilt that lowers
		// the wheel raises it; one that raises it is not taken off).
		const double		 Planned = M->LegZForWorldZ(Leg, X, WorldZ, BCmd, Ahead.Pitch, Ahead.Roll);
		const FChassisFrame& F = Measure.Frame;
		const double		 Measured = F.Z.Z > 0.1 ? (WorldZ - Measure.OriginZ - F.X.Z * X - F.Y.Z * M->LegY(Leg)) / F.Z.Z : Planned;
		return FMath::Max(Planned, Measured);
	}

	double FExecutor::SwingWorldZ(int32 Leg, const FExecMeasure& Measure, double V) const
	{
		const FWheelPlan& P = Plan.Wheels[Leg];
		const FExecWheel& W = Wheels[Leg];
		const FExecLeg&	  L = Measure.Legs[Leg];
		const FVector2D	  XY(L.WheelWorld.X, L.WheelWorld.Y);
		const double	  Ahead = FMath::Max(0.0, V) * SwingAheadSeconds;
		const int32		  Steps = 6;
		double			  Z = -TNumericLimits<double>::Max();
		if (S < W.SDrop || (P.bHeldFromStart && S < P.SClear))
		{
			// Up to the most its window asks, as the plan carries it -- the higher
			// side found above its plane, that much higher.
			Z = W.ZTop + SwingRaise;
		}
		else
		{
			// Coming down as the terrain under it falls away, never under the far
			// surface's support (a climb's, raised as the far surface was found).
			Z = M->ContactCentreZ(Leg, XY, true) - M->Preload(Leg) + (M->IsClimb() ? SwingRaise : 0.0);
		}
		// And whatever its profile asks 0.3 s ahead, from where it is now, short of
		// where it may land.
		for (int32 k = 0; k <= Steps; ++k)
		{
			const FVector2D At = XY + Measure.Forward * (Ahead * k / Steps);
			if (k > 0 && M->GetTerrain().DistancePastEdge(At) > P.DDown)
			{
				break;
			}
			Z = FMath::Max(Z, M->ClearanceZ(Leg, At) + SwingRaise);
		}
		return Z;
	}

	double FExecutor::MarginWithout(const FExecMeasure& Measure, uint32 Without, uint32 With) const
	{
		const int32											  N = Wheels.Num();
		TArray<FVector2D, TInlineAllocator<MaxPlanLegs>>	  XZ;
		TArray<FMeasuredWheel, TInlineAllocator<MaxPlanLegs>> Ws;
		for (int32 i = 0; i < N; ++i)
		{
			XZ.Add(Measure.Legs[i].XZ);
			FMeasuredWheel& W = Ws.AddDefaulted_GetRef();
			W.Centre = Measure.Legs[i].WheelWorld;
			W.bSupport = ((With >> i) & 1u) || (IsSupport(i) && !((Without >> i) & 1u));
			const bool bFar = (With >> i) & 1u ? Wheels[i].PastEdgeCm >= 0.0 : Wheels[i].Plane == 1;
			W.Gradient = (bFar ? M->GetTerrain().Far : M->GetTerrain().Near).Gradient;
		}
		const FPoseMargin R = MeasuredMargin(In.Model, Measure.Frame, XZ, Ws, Measure.Forward, In.Tuning.Acceleration, In.Tuning.Deceleration);
		return R.Support >= 3 ? R.DynamicCm : -1e9;
	}

	bool FExecutor::LiftPermit(int32 Leg, const FExecMeasure& Measure, const FLevelState& Level, FString& OutWhy) const
	{
		const FWheelPlan&		P = Plan.Wheels[Leg];
		const FRammsCurbTuning& T = In.Tuning;
		if (!Level.bLevelled)
		{
			OutWhy = TEXT("the level-in is not done");
			return false;
		}
		if (bMarginHold)
		{
			OutWhy = FString::Printf(TEXT("the robot is measured %.2f cm inside its wheels, under MinLiveStabilityMarginCm %.1f: holding"), LiveMarginCm,
				T.MinLiveStabilityMarginCm);
			return false;
		}
		for (int32 j = 0; j < Wheels.Num(); ++j)
		{
			if (((P.MustBeDownMask >> j) & 1u) && Wheels[j].State != ERammsCurbWheelState::Planted)
			{
				OutWhy = FString::Printf(TEXT("waits for '%s' to land (it is %s)"), *In.Legs[j].Name.ToString(), WheelStateName(Wheels[j].State));
				return false;
			}
		}
		uint32 Set = 1u << Leg;
		for (int32 j = 0; j < Wheels.Num(); ++j)
		{
			Set |= IsSupport(j) ? 0u : (1u << j);
		}
		if (IsForbiddenSet(In.Model, Set))
		{
			OutWhy = FString::Printf(TEXT("it would leave a set the gait never lifts together off the ground (mask %x)"), Set);
			return false;
		}
		// The plan has these off the ground together somewhere between here and
		// the wheel's window.
		const double Ds = Plan.SampleCm;
		const int32	 K0 = FMath::Clamp(FMath::FloorToInt(S / Ds) - 2, 0, Plan.Samples.Num() - 1);
		const int32	 K1 = FMath::Clamp(FMath::CeilToInt(P.SClear / Ds) + 2, K0, Plan.Samples.Num() - 1);
		bool		 bPlanned = false;
		for (int32 k = K0; k <= K1 && !bPlanned; ++k)
		{
			bPlanned = (Set & ~Plan.Samples[k].AirborneMask) == 0;
		}
		if (!bPlanned)
		{
			OutWhy = FString::Printf(TEXT("the plan never has these off the ground together here (mask %x)"), Set);
			return false;
		}
		const FPlanAt Here = Between(S);
		for (int32 Side = 0; Side < 2; ++Side)
		{
			const int32 C = Centre[Side];
			if (C != INDEX_NONE && C != Leg && FMath::Abs(Measure.Legs[C].XZ.X - Here.X[Side]) > PermitToleranceCm)
			{
				OutWhy = FString::Printf(TEXT("'%s' is %.1f cm off its planned fore/aft %.1f"), *In.Legs[C].Name.ToString(),
					Measure.Legs[C].XZ.X - Here.X[Side], Here.X[Side]);
				return false;
			}
		}
		const double Margin = MarginWithout(Measure, Set);
		if (Margin < T.MinLiveStabilityMarginCm)
		{
			OutWhy = FString::Printf(TEXT("with it up the robot would stand %.2f cm inside its wheels (MinLiveStabilityMarginCm %.1f)"), Margin,
				T.MinLiveStabilityMarginCm);
			return false;
		}
		// The chassis where it is sent (its steady offset, the preloads' and the
		// servos' sag, taken out): the lift's reach and clearance were planned
		// at that height.
		const double Off = Measure.OriginZ - BCmdLag - BOffset;
		if (FMath::Abs(Off) > PermitToleranceCm)
		{
			OutWhy = FString::Printf(TEXT("the chassis is %+.2f cm off its commanded height (more than %.1f)"), Off, PermitToleranceCm);
			return false;
		}
		// Over the time it will be up, at the plan's poses, with it up and every
		// wheel up now that the plan has down by then still up.
		double		 At = S;
		const double Predicted = PredictedMargin(Leg, Measure, At);
		if (Predicted < T.MinStabilityMargin)
		{
			OutWhy = FString::Printf(TEXT("with it up, and the wheels landing late still up, the plan's pose at %.0f cm would stand %.2f cm "
										  "inside its wheels (MinStabilityMargin %.1f)"),
				At, Predicted, T.MinStabilityMargin);
			return false;
		}
		return true;
	}

	double FExecutor::LiftDeadline(int32 Leg, const FExecMeasure& Measure) const
	{
		const FExecWheel&		W = Wheels[Leg];
		const FExecLeg&			L = Measure.Legs[Leg];
		const FWheelPlan&		P = Plan.Wheels[Leg];
		const FRammsCurbTuning& T = In.Tuning;
		const double			Cos = FMath::Max(0.1, M->NormalShare());
		const double			Span = FMath::Max(0.0, (P.DDown - W.PastEdgeCm) / Cos);
		if (!bLevelInBegun)
		{
			// Held in the drive stance: the level-in raises it (and the late level-in
			// cap times that).
			return TNumericLimits<double>::Max();
		}
		const FVector2D XY(L.WheelWorld.X, L.WheelWorld.Y);
		const double	Frame = L.WheelWorld.Z - L.XZ.Y - Measure.OriginZ;
		const double	Off = Measure.OriginZ - BCmd;
		// The leg height (chassis frame) the profile asks at each point ahead from
		// its window on (short of it the wheel may still be rising from the ground,
		// as the plan has it), the chassis as the plan has it there; no height
		// where it asks none.
		constexpr double					  StepCm = 1.0;
		TArray<double, TInlineAllocator<128>> Need;
		for (double X = 0.0; X <= Span + 1e-6 && Need.Num() < 128; X += StepCm)
		{
			if (W.PastEdgeCm + X * Cos < P.DUp)
			{
				Need.Add(-TNumericLimits<double>::Max());
				continue;
			}
			const double World = M->ClearanceZ(Leg, XY + Plan.Heading * X) + SwingRaise - ExecProfileToleranceCm;
			Need.Add(World - (Between(S + X).B + Off) - Frame);
		}
		// The leg's height in time: its command a trapezoid towards its target,
		// the leg following with the servo lag.
		constexpr double							   StepS = 0.01;
		constexpr int32								   MaxSteps = 400;
		const double								   Tau = FMath::Max(1e-3, static_cast<double>(T.ServoLagSeconds));
		TArray<double, TInlineAllocator<MaxSteps + 1>> H;
		{
			double C = W.CmdZ, Cv = W.CmdVz, Hh = L.XZ.Y;
			H.Add(Hh);
			for (int32 k = 0; k < MaxSteps; ++k)
			{
				TrackTrapezoid(C, Cv, W.TargetZ, T.LegSpeed, T.LegAcceleration, StepS);
				Hh += (C - Hh) * FMath::Min(1.0, StepS / Tau);
				H.Add(Hh);
			}
		}
		// What its leg cannot reach at any speed on the chassis as it is now comes
		// with the chassis rising along the plan -- with distance, not time --
		// and slowing would not bring it: the deadline asks only for what the leg
		// can reach, in time.
		for (double& N : Need)
		{
			N = FMath::Min(N, H.Last() - 0.01);
		}
		auto Clears = [&](double V) {
			for (int32 j = 0; j < Need.Num(); ++j)
			{
				const double Time = j * StepCm / FMath::Max(V, 1e-3);
				const double U = Time / StepS;
				const int32	 K = FMath::FloorToInt(U);
				const double Height = K >= MaxSteps ? H.Last() : FMath::Lerp(H[K], H[K + 1], U - K);
				if (Height < Need[j])
				{
					return false;
				}
			}
			return true;
		};
		const double Most = FMath::Max(FMath::Max(VCmd, VHat), 1.0);
		if (Clears(Most))
		{
			return TNumericLimits<double>::Max();
		}
		double Lo = 0.0, Hi = Most;
		for (int32 n = 0; n < 16; ++n)
		{
			const double Mid = 0.5 * (Lo + Hi);
			(Clears(Mid) ? Lo : Hi) = Mid;
		}
		return Lo;
	}

	double FExecutor::PredictRise(int32 Leg, const FExecMeasure& Measure, double Ahead, FRise& Sim) const
	{
		const FExecWheel&		W = Wheels[Leg];
		const FExecLeg&			L = Measure.Legs[Leg];
		const FRammsCurbTuning& T = In.Tuning;
		const double			Tau = FMath::Max(1e-3, static_cast<double>(T.ServoLagSeconds));
		const double			V = FMath::Max(1.0, VHat);
		const double			Frame = L.WheelWorld.Z - L.XZ.Y - Measure.OriginZ;
		const double			Off = Measure.OriginZ - BCmd;
		if (!Sim.bStarted)
		{
			Sim.bStarted = true;
			Sim.Time = 0.0;
			Sim.C = W.CmdZ;
			Sim.Cv = W.CmdVz;
			Sim.H = L.XZ.Y;
		}
		constexpr double StepS = 0.01;
		for (; Sim.Time + StepS <= Ahead / V; Sim.Time += StepS)
		{
			// Headed for the height its target asks in the world, on the chassis as
			// the plan has it then, no higher than its leg reaches.
			const double B = Between(S + V * Sim.Time).B + Off;
			const double Target = ClampToReach(Leg, L.XZ.X, W.TargetZ + (Measure.OriginZ - B));
			TrackTrapezoid(Sim.C, Sim.Cv, Target, T.LegSpeed, T.LegAcceleration, StepS);
			Sim.H += (Sim.C - Sim.H) * FMath::Min(1.0, StepS / Tau);
		}
		return Between(S + Ahead).B + Off + Sim.H + Frame;
	}

	double FExecutor::PlantingTarget(int32 Leg, const FExecMeasure& Measure, double AheadCm, const FVector2D& Bias, double PressCm) const
	{
		const FExecWheel&	 W = Wheels[Leg];
		const FExecLeg&		 L = Measure.Legs[Leg];
		const bool			 bFar = W.Plane == 1;
		const FVector2D		 XY = FVector2D(L.WheelWorld.X, L.WheelWorld.Y) + Measure.Forward * AheadCm;
		const double		 WorldZ = M->ContactCentreZ(Leg, XY, bFar, Measure.AttitudeDeg.X, Measure.AttitudeDeg.Y) + (bFar ? Bias.Y : Bias.X) - PressCm;
		const FChassisFrame& F = Measure.Frame;
		// From the chassis as it will be a servo lag on, as the leg will: where
		// it is, moved by what the lag model has still to follow of the plan's
		// height (a chassis coming down would otherwise have the leg set down too
		// long, and lift the base as it lands).
		const double Origin = Measure.OriginZ + (BCmd - BCmdLag);
		return F.Z.Z > 0.1 ? (WorldZ - Origin - F.X.Z * L.XZ.X - F.Y.Z * M->LegY(Leg)) / F.Z.Z : W.CmdZ;
	}

	void FExecutor::LandingWork(int32 Leg, const FExecMeasure& Measure, double& OutGate, double& OutSeconds) const
	{
		const FWheelPlan& P = Plan.Wheels[Leg];
		const FExecWheel& W = Wheels[Leg];
		const double	  Cos = FMath::Max(0.1, M->NormalShare());
		OutGate = 0.0;
		OutSeconds = 0.0;
		switch (W.State)
		{
			case ERammsCurbWheelState::Planted:
				if (W.Plane == 1)
				{
					return;
				}
				// Still to cross: its landing is a distance away, then its plant.
				OutGate = FMath::Max(0.0, (P.DDown - W.LandedPastCm) / Cos);
				OutSeconds = P.TPlant + P.TLoad;
				return;
			case ERammsCurbWheelState::Loading:
				// A leg at its reach takes no load until the chassis comes down to it.
				OutGate = W.bReachClamped ? ReachGate(Leg) : 0.0;
				OutSeconds = FMath::Max(0.0, P.TLoad - W.Loading);
				return;
			case ERammsCurbWheelState::Planting:
			{
				// Coming down: what is left of its plant (the lag model's), then its
				// load; a leg at its reach, once the plan's chassis has come down to
				// it.
				const FRammsCurbTuning& T = In.Tuning;
				const double			Down = FMath::Max(0.0, Measure.Legs[Leg].XZ.Y - W.TargetZ);
				OutGate = W.bReachClamped ? ReachGate(Leg) : 0.0;
				OutSeconds = FMath::Min(P.TPlant, Down / FMath::Max(1.0f, T.LegSpeed) + T.ServoLagSeconds) + P.TLoad;
				return;
			}
			default: // Lifting, Clear, Held: it lands once past its window
				OutGate = FMath::Max(0.0, (P.DDown - W.LandedPastCm) / Cos);
				OutSeconds = P.TPlant + P.TLoad;
				return;
		}
	}

	double FExecutor::DeadlineCap(const FExecMeasure& Measure, int32& OutLeg) const
	{
		const FRammsCurbTuning& T = In.Tuning;
		const double			Cos = FMath::Max(0.1, M->NormalShare());
		const bool				bClimb = M->IsClimb();
		const double			Dec = FMath::Max(1.0, static_cast<double>(T.Deceleration));
		double					Cap = TNumericLimits<double>::Max();
		OutLeg = INDEX_NONE;
		auto Offer = [&](double V, int32 Leg) {
			if (V < Cap)
			{
				Cap = FMath::Max(0.0, V);
				OutLeg = Leg;
			}
		};
		for (int32 i = 0; i < Wheels.Num(); ++i)
		{
			const FWheelPlan& P = Plan.Wheels[i];
			const FExecWheel& W = Wheels[i];
			// How far (along the heading) the wheel is from its window: where it
			// must be clear by. Past it, nothing to cap.
			const double Phys = (P.DUp - W.PastEdgeCm) / Cos;
			if (Phys <= 0.0)
			{
				continue;
			}
			// For a planted centre wheel, after the walk the plan has it make
			// before then, as far as it can make it (WalkCredit): walked aft before
			// its lift, it has that much more room (taken where it stands, forward
			// for the front corners' crossing, the deadline would hold the robot
			// under the plan's speed at every such hand-off); walked forward, less
			// -- down to none, which caps the speed to a stop rather than dropping
			// the wheel's caps while it is still short of its window.
			double Room = Phys;
			if (M->IsCentre(i) && W.State == ERammsCurbWheelState::Planted)
			{
				Room = FMath::Max(0.0, Phys + WalkCredit(i, Measure, Phys));
			}
			if (W.State == ERammsCurbWheelState::Lifting || (W.State == ERammsCurbWheelState::Held && !W.bLower))
			{
				// Rising (or carried up from the drive stance): clear by its window,
				// at the rate its leg rises -- on a descent it is off the ground
				// already, which is all its window asks.
				if (!bClimb)
				{
					continue;
				}
				Offer(LiftDeadline(i, Measure), i);
				continue;
			}
			if (W.State != ERammsCurbWheelState::Planted || W.Plane != 0 || P.bHeldFromStart)
			{
				continue;
			}
			// Its lift still to begin. Waiting for wheels to land first (its
			// MustBeDownMask): each such wheel's landing is a distance away (it is
			// set down past its own window), then its plant and load, then this
			// lift -- the gap the plan timed (its gap caps), braked down to from
			// now. Blocked otherwise: the lift from now. Neither: its own trigger
			// starts it in time.
			const double Lift = (bClimb ? P.TLift : UnloadSeconds) + ExecGapMarginSeconds;
			for (int32 j = 0; j < Wheels.Num(); ++j)
			{
				if (!((P.MustBeDownMask >> j) & 1u) || Wheels[j].State == ERammsCurbWheelState::Planted)
				{
					continue;
				}
				double Gate = 0.0, Work = 0.0;
				LandingWork(j, Measure, Gate, Work);
				const double After = (Room - Gate) / FMath::Max(Work + Lift, static_cast<double>(MinDeadlineSeconds));
				Offer(After <= 0.0 ? 0.0 : FMath::Sqrt(After * After + 2.0 * Dec * Gate), i);
			}
			if (W.bBlocked)
			{
				Offer(Room / (Lift + T.ServoLagSeconds), i);
			}
		}
		return Cap;
	}

	double FExecutor::WalkCredit(int32 Leg, const FExecMeasure& Measure, double RoomCm) const
	{
		if (!M->IsCentre(Leg) || !Measure.Legs.IsValidIndex(Leg))
		{
			return 0.0;
		}
		double Walk = Measure.Legs[Leg].XZ.X - Between(Plan.Wheels[Leg].SClear).X[M->Side(Leg)];
		if (Walk > 0.0)
		{
			// Aft: room it has only once it has walked there -- not while the centre
			// wheels hold their fore/aft (a landing late), and no further than it
			// can walk while the chassis covers the room it has now.
			if (bCentresHeld)
			{
				return 0.0;
			}
			const double CSS = FMath::Max(0.1, static_cast<double>(In.Tuning.CenterShiftSpeed));
			const double V = FMath::Max(1.0, FMath::Max(VHat, VCmd));
			Walk = FMath::Min(Walk, CSS * FMath::Max(0.0, RoomCm) / V);
		}
		return Walk;
	}

	double FExecutor::SupportCeiling(int32 Leg, const FExecMeasure& Measure, const FPlanAt& Ahead, double AheadCm, const FVector2D& Bias) const
	{
		const FExecLeg& L = Measure.Legs[Leg];
		const FVector2D R = M->Reach(Leg, L.XZ.X);
		if (R.X > R.Y)
		{
			return TNumericLimits<double>::Max();
		}
		// The leg's support target is linear in the chassis height (its slope the
		// attitude's): at BCmd, and the chassis height at which it comes down to
		// the bottom of its reach, LegRangeMargin kept.
		const FExecWheel& W = Wheels[Leg];
		const bool		  bFar = W.Plane == 1;
		const FVector2D	  XY = FVector2D(L.WheelWorld.X, L.WheelWorld.Y) + Measure.Forward * AheadCm;
		const double	  WorldZ = M->ContactCentreZ(Leg, XY, bFar, Ahead.Pitch, Ahead.Roll) + (bFar ? Bias.Y : Bias.X) - M->Preload(Leg);
		const double	  Z0 = M->LegZForWorldZ(Leg, L.XZ.X, WorldZ, BCmd, Ahead.Pitch, Ahead.Roll);
		const double	  Z1 = M->LegZForWorldZ(Leg, L.XZ.X, WorldZ, BCmd + 1.0, Ahead.Pitch, Ahead.Roll);
		const double	  PerCm = Z0 - Z1; // the leg comes down this much (cm) per cm the chassis rises
		if (PerCm <= 1e-6)
		{
			return TNumericLimits<double>::Max();
		}
		return BCmd + (Z0 - (R.X + In.Tuning.LegRangeMargin)) / PerCm;
	}

	double FExecutor::TouchDistance(int32 Leg, const FExecMeasure& Measure) const
	{
		// Along the heading from where the wheel is, the first point at which the
		// terrain under its footprint -- not dilated, no clearance -- comes up to
		// its rim: the rim rising as its leg does, its command going on to where
		// it is headed and the leg following it with the servo lag -- while the
		// leg follows its lag model; at the height it is now when it does not
		// (stalled, or held down).
		const FExecLeg&			L = Measure.Legs[Leg];
		const FExecWheel&		W = Wheels[Leg];
		const FRammsCurbTuning& T = In.Tuning;
		const FVector2D			XY(L.WheelWorld.X, L.WheelWorld.Y);
		const FWheelPlan&		P = Plan.Wheels[Leg];
		const double			Cos = FMath::Max(0.1, M->NormalShare());
		const double			Span = FMath::Max(0.0, (P.DDown - W.PastEdgeCm) / Cos);
		FRise					Sim;
		for (double Ahead = 0.0; Ahead <= Span + 1e-6; Ahead += TouchStepCm)
		{
			const FVector2D At = XY + Plan.Heading * Ahead;
			// Only where its footprint reaches the edge: on its own side, the surface
			// it rises from (into which a preloaded wheel is pressed) is no
			// obstacle.
			if (M->GetTerrain().DistancePastEdge(At) + P.Eps < 0.0)
			{
				continue;
			}
			const double Z = PredictRise(Leg, Measure, Ahead, Sim);
			if (ClearanceHeight(M->GetTerrain(), At, M->RollDirection(Leg), M->Radius(Leg), M->TyreWidth(Leg), 0.0) + SwingRaise > Z)
			{
				return Ahead;
			}
		}
		return TNumericLimits<double>::Max();
	}

	double FExecutor::BrakingCap(const FExecMeasure& Measure, int32& OutLeg) const
	{
		const FRammsCurbTuning& T = In.Tuning;
		const double			Cos = FMath::Max(0.1, M->NormalShare());
		const bool				bClimb = M->IsClimb();
		double					Cap = TNumericLimits<double>::Max();
		OutLeg = INDEX_NONE;
		for (int32 i = 0; i < Wheels.Num(); ++i)
		{
			const FWheelPlan& P = Plan.Wheels[i];
			const FExecWheel& W = Wheels[i];
			if (W.PastEdgeCm >= P.DDown)
			{
				continue;
			}
			double Distance = TNumericLimits<double>::Max();
			if (bClimb)
			{
				if (W.State == ERammsCurbWheelState::Lifting || (W.State == ERammsCurbWheelState::Held && !W.bLower))
				{
					// Rising as its lag model has it: on as far as its rim, rising so,
					// clears the curb. Not following its model (stalled, held down):
					// no nearer the face than FaceMargin, as on the ground.
					const FExecLeg& L = Measure.Legs[i];
					// Still rising (if slower than the lag model has it), or with the
					// model: its touch distance, predicted from where it is. Stalled
					// short of its command: its window, as on the ground.
					const bool bRising = W.MeasVz > StallSpeedCmS || W.CmdZ - L.XZ.Y <= TrackingErrorCm;
					if (FMath::Abs(L.XZ.Y - L.LagZ) <= TrackingErrorCm || bRising)
					{
						const double Touch = TouchDistance(i, Measure);
						if (Touch < TNumericLimits<double>::Max())
						{
							Distance = Touch - TouchMarginCm;
						}
					}
					else
					{
						Distance = (P.DUp - W.PastEdgeCm) / Cos;
					}
				}
				else if (W.Plane == 0 && W.State != ERammsCurbWheelState::Clear)
				{
					// On the ground short of the face (or being set down there): no
					// nearer than FaceMargin.
					Distance = (P.DUp - W.PastEdgeCm) / Cos;
				}
			}
			else if ((W.State == ERammsCurbWheelState::Planted || W.State == ERammsCurbWheelState::Loading) && W.Plane == 0)
			{
				// Loaded on the high side: it may not roll off the lip.
				Distance = (P.DUp - W.PastEdgeCm) / Cos;
			}
			else if (W.State == ERammsCurbWheelState::Lifting && W.Plane == 0 && W.BottomCm <= TouchCm)
			{
				// Lifting, but still on the ground on the high side: loaded until it
				// is off. While its leg rises as the servo lag model has it
				// (unloading), no nearer the lip than the drop edge itself -- the
				// window's EdgeMargin is the unloading's room; stalled there, no
				// nearer than its window, as when planted.
				Distance = ((IsLiftStalledOnGround(i, Measure) ? P.DUp : -P.Lambda) - W.PastEdgeCm) / Cos;
			}
			if (Distance == TNumericLimits<double>::Max())
			{
				continue;
			}
			const double V = BrakingSpeed(LagBrakingDistance(static_cast<float>(Distance), static_cast<float>(VHat), DriveLagSeconds),
				T.Deceleration);
			if (V < Cap)
			{
				Cap = V;
				OutLeg = i;
			}
		}
		return Cap;
	}

	void FExecutor::Tick(const FExecMeasure& Measure, FLevelState& Level, FExecCommand& Out)
	{
		const int32 N = Wheels.Num();
		Out.LegZ.SetNum(N);
		Out.LegX.SetNum(N);
		if (!bRunning || Measure.Legs.Num() != N)
		{
			for (int32 i = 0; i < N; ++i)
			{
				Out.LegZ[i] = Wheels[i].CmdZ;
				Out.LegX[i] = Wheels[i].CmdX;
			}
			Out.DriveSpeed = 0.0;
			return;
		}
		const FRammsCurbTuning&	 T = In.Tuning;
		const FRammsCurbTerrain& Terrain = M->GetTerrain();
		const double			 Dt = FMath::Max(0.0, Measure.Dt);
		const double			 LegSpeed = FMath::Max(1.0, static_cast<double>(T.LegSpeed));
		const double			 LegAccel = FMath::Max(1.0, static_cast<double>(T.LegAcceleration));
		const double			 CSS = FMath::Max(0.1, static_cast<double>(T.CenterShiftSpeed));
		const double			 BH = FMath::Max(0.1, static_cast<double>(T.BaseHeightSpeed));
		const double			 Cos = FMath::Max(0.1, M->NormalShare());
		Elapsed += Dt;
		if (bHalted)
		{
			TickHalted(Measure, Level, Out, Dt, /*bWheelsMeasured=*/false);
			return;
		}

		// Where the robot is along the plan, and how fast it goes.
		S = FVector2D::DotProduct(Measure.OriginXY - Plan.Origin, Plan.Heading);
		VHat = Measure.bSpeedValid ? FMath::Max(0.0, Measure.SpeedCmS) : VCmd;
		const double V = FMath::Max(VHat, VCmd);

		// Each wheel against the edge and the surface it follows; the biases.
		// A surface's planted wheels' lowest bottom over its plane, whatever it
		// is (the bias takes those near it), says how far above it they all
		// stand.
		const FVector2D Bias = Level.GetBias();
		MeasureWheels(Measure, Bias, Dt);
		TArray<float, TInlineAllocator<8>> NearRes, FarRes;
		double							   LowestRaw[2] = { TNumericLimits<double>::Max(), TNumericLimits<double>::Max() };
		int32							   Standing[2] = { 0, 0 };
		for (int32 i = 0; i < N; ++i)
		{
			const FExecWheel& W = Wheels[i];
			if (W.State != ERammsCurbWheelState::Planted)
			{
				continue;
			}
			if (FMath::Abs(W.RawBottomCm) <= PlaneBiasSampleCm)
			{
				(W.Plane == 1 ? FarRes : NearRes).Add(static_cast<float>(W.RawBottomCm));
			}
			LowestRaw[W.Plane] = FMath::Min(LowestRaw[W.Plane], W.RawBottomCm);
			++Standing[W.Plane];
		}
		Level.NearBias.Tick(NearRes, static_cast<float>(Dt));
		Level.FarBias.Tick(FarRes, static_cast<float>(Dt));

		// The terrain monitor: the probes ahead of and under each wheel against
		// the planes. Off by more than TerrainMismatchCm: the terrain is not as
		// detected. A surface they find off its plane, where too few wheels
		// stand on it to say, is taken up by its bias; before any wheel has begun
		// to cross, one off by more than ReplanTerrainCm is re-fitted and
		// re-planned.
		{
			// (A landing wheel's own probes are the landing check's: it is set
			// down where they look. The probes behind a wheel are swing
			// clearance's, over a lip it has left: the terrain behind the robot
			// -- a curb it has just climbed -- is none of the crossing's.)
			TArray<FTerrainProbe, TInlineAllocator<48>> Probes;
			for (const FTerrainProbe& Pr : Measure.Probes)
			{
				const ERammsCurbWheelState St = Wheels.IsValidIndex(Pr.Leg) ? Wheels[Pr.Leg].State : ERammsCurbWheelState::Planted;
				if (Pr.AheadCm >= 0.0 && St != ERammsCurbWheelState::Planting && St != ERammsCurbWheelState::Loading)
				{
					Probes.Add(Pr);
				}
			}
			// Once a wheel has begun to cross no re-plan is possible: a surface
			// further off than a plane's bias takes up cannot be crossed on this
			// plan either.
			const bool			bBegun = HasCrossingBegun();
			const double		Allowed = bBegun ? FMath::Min(static_cast<double>(T.TerrainMismatchCm), static_cast<double>(MaxPlaneBiasCm))
												 : static_cast<double>(T.TerrainMismatchCm);
			const double		Limit = Allowed + TerrainToleranceCm;
			const FTerrainCheck TC = CheckTerrain(Terrain, Probes, FVector2D::ZeroVector);
			if (TC.WorstLeg != INDEX_NONE && FMath::Abs(TC.WorstCm) > FMath::Abs(Counts.TerrainOffCm))
			{
				Counts.TerrainOffCm = TC.WorstCm;
				Counts.TerrainOffPlane = TC.WorstPlane;
				Counts.TerrainOffAheadCm = TC.WorstAheadCm;
			}
			if (TerrainOff.Tick(TC.WorstLeg != INDEX_NONE && FMath::Abs(TC.WorstCm) > Limit, Dt, TerrainMismatchSeconds))
			{
				const int32	  Leg = TC.WorstLeg;
				const bool	  bBias = Allowed < T.TerrainMismatchCm;
				const FString Beyond = bBias ? FString::Printf(TEXT("more than the %.1f cm a plane's bias takes up, with the crossing begun (too late to re-plan)"), Allowed)
											 : FString::Printf(TEXT("more than TerrainMismatchCm %.1f"), Allowed);
				Fault(FString::Printf(TEXT("terrain not as detected: the %s surface %.1f cm %s than detected %.0f cm %s '%s' (%s), %s"),
					TC.WorstPlane == 1 ? TEXT("far") : TEXT("near"), FMath::Abs(TC.WorstCm), TC.WorstCm > 0.0 ? TEXT("higher") : TEXT("lower"),
					FMath::Abs(TC.WorstAheadCm), TC.WorstAheadCm >= 0.0 ? TEXT("ahead of") : TEXT("behind"), *In.Legs[Leg].Name.ToString(),
					WheelStateName(Wheels[Leg].State), *Beyond));
				TickHalted(Measure, Level, Out, Dt, /*bWheelsMeasured=*/true);
				return;
			}
			FVector2D Refit = FVector2D::ZeroVector;
			for (int32 Plane = 0; Plane < 2; ++Plane)
			{
				const double Off = TC.MedianCm[Plane];
				if (TC.Probes[Plane] >= 2 && FMath::Abs(Off) >= TerrainSeenCm && Standing[Plane] < MinWheelsForBias)
				{
					(Plane == 1 ? Level.FarBias : Level.NearBias).Follow(static_cast<float>(Off), static_cast<float>(Dt));
				}
				if (TC.Probes[Plane] >= 3 && FMath::Abs(Off) > ReplanTerrainCm)
				{
					(Plane == 1 ? Refit.Y : Refit.X) = Off;
				}
				// Every wheel planted on it standing above it: it is higher than
				// detected.
				if (PlantedHigh[Plane].Tick(Standing[Plane] >= MinWheelsForBias && LowestRaw[Plane] > Limit, Dt, AttitudeFaultSeconds))
				{
					Fault(FString::Printf(TEXT("terrain not as detected: the %d wheels planted on the %s surface all stand at least %.1f cm above "
											   "it, more than TerrainMismatchCm %.1f"),
						Standing[Plane], Plane == 1 ? TEXT("far") : TEXT("near"), LowestRaw[Plane], T.TerrainMismatchCm));
					TickHalted(Measure, Level, Out, Dt, /*bWheelsMeasured=*/true);
					return;
				}
			}
			if (!Refit.IsZero() && !HasCrossingBegun())
			{
				AskReplan(FString::Printf(TEXT("the terrain traced ahead: the near surface %+.1f cm, the far %+.1f cm off its plane"), Refit.X,
							  Refit.Y),
					Refit);
			}
		}
		SwingRaise = SwingRaiseCm(Level);

		// The level-in begins where the plan has it (at once on a slope, or for a
		// stance that needs it).
		if (!bLevelInBegun && (S >= Plan.LevelInFromS - 1e-6 || Plan.LevelInFromS <= 1e-6))
		{
			bLevelInBegun = true;
			LevelInFrom = Elapsed;
			BCmd = Measure.OriginZ;
			Phase = EExecPhase::Prepare;
			Log(FString::Printf(TEXT("levelling in from the drive stance (pitch %+.2f, roll %+.2f deg)%s"), Measure.AttitudeDeg.X,
				Measure.AttitudeDeg.Y, Plan.bLevelInAtRest ? TEXT(", standing still") : TEXT("")));
		}

		// The plan a servo lag ahead, and here.
		const double  AheadCm = VHat * T.ServoLagSeconds;
		const FPlanAt Ahead = Between(S + AheadCm);
		const FPlanAt Here = Between(S);
		PlannedAttitude = FVector2D(Here.Pitch, Here.Roll);

		// The base against its planned attitude, once it has been levelled in:
		// off by more than AttitudeFaultDeg for AttitudeFaultSeconds, a fault (a
		// slip, a wheel off an edge, a landing on something); off by half that,
		// filtered, for AttitudeSlowSeconds, half the plan's speed.
		if (Level.bLevelled)
		{
			const double Err = AttitudeErrorDeg(Measure.AttitudeDeg, PlannedAttitude);
			Counts.AttitudeOffDeg = FMath::Max(Counts.AttitudeOffDeg, Err);
			if (AttitudeOff.Tick(Err > T.AttitudeFaultDeg, Dt, AttitudeFaultSeconds))
			{
				Fault(FString::Printf(TEXT("the base %.1f deg off its planned attitude for %.2f s (pitch %+.1f, roll %+.1f deg; planned %+.1f / %+.1f), "
										   "more than AttitudeFaultDeg %.1f"),
					Err, AttitudeOff.Seconds, Measure.AttitudeDeg.X, Measure.AttitudeDeg.Y, PlannedAttitude.X, PlannedAttitude.Y, T.AttitudeFaultDeg));
				TickHalted(Measure, Level, Out, Dt, /*bWheelsMeasured=*/true);
				return;
			}
			const double Filtered = AttitudeErrorDeg(Level.FilteredAttitudeDeg, PlannedAttitude);
			const double Slow = 0.5 * T.AttitudeFaultDeg;
			if (!bAttitudeSlow && AttitudeSlow.Tick(Filtered > Slow, Dt, AttitudeSlowSeconds))
			{
				bAttitudeSlow = true;
				Log(FString::Printf(TEXT("the base %.1f deg off its planned attitude (half AttitudeFaultDeg): half the plan's speed"), Filtered));
			}
			else if (bAttitudeSlow && Filtered < Slow - 0.25)
			{
				bAttitudeSlow = false;
				AttitudeSlow.Reset();
				Log(FString::Printf(TEXT("the base back within %.1f deg of its planned attitude"), Slow - 0.25));
			}
		}

		// The measured margin, once levelled in: under MinLiveStabilityMarginCm the
		// robot holds (no wheel lifts, braked at half Deceleration) until it is
		// MarginReleaseCm back over it; under MarginFaultCm, a fault.
		LiveMarginCm = MarginWithout(Measure, 0);
		if (Level.bLevelled)
		{
			const bool bWasHolding = bMarginHold;
			if (MarginWatch.Tick(LiveMarginCm, T.MinLiveStabilityMarginCm, Dt) == EMarginWatch::Fault)
			{
				Fault(FString::Printf(TEXT("the robot measured %.2f cm inside its wheels for %.2f s, under %.1f: near tipping"), LiveMarginCm,
					MarginWatch.Low.Seconds, MarginFaultCm));
				TickHalted(Measure, Level, Out, Dt, /*bWheelsMeasured=*/true);
				return;
			}
			bMarginHold = MarginWatch.bHold;
			if (bMarginHold && !bWasHolding)
			{
				++Counts.MarginHolds;
				Log(FString::Printf(TEXT("the robot measured %.2f cm inside its wheels, under MinLiveStabilityMarginCm %.1f: no wheel lifts, braking"),
					LiveMarginCm, T.MinLiveStabilityMarginCm));
			}
			else if (!bMarginHold && bWasHolding)
			{
				Log(FString::Printf(TEXT("the robot measured %.2f cm inside its wheels again: going on"), LiveMarginCm));
			}
		}

		// Each leg against its servo lag model: off by LegErrorFaultCm for
		// LegErrorFaultSeconds, a fault naming it (a leg stalled, slipped, held).
		for (int32 i = 0; i < N; ++i)
		{
			const FExecLeg& L = Measure.Legs[i];
			const double	Err = L.bLagX ? FMath::Sqrt(FMath::Square(L.XZ.Y - L.LagZ) + FMath::Square(L.XZ.X - L.LagX)) : FMath::Abs(L.XZ.Y - L.LagZ);
			if (Wheels[i].LegError.Tick(L.bValid && Err > LegErrorFaultCm, Dt, LegErrorFaultSeconds))
			{
				Fault(FString::Printf(TEXT("'%s' (%s) %.1f cm off where its servo lag model has it for %.1f s, more than %.0f cm"),
					*In.Legs[i].Name.ToString(), WheelStateName(Wheels[i].State), Err, Wheels[i].LegError.Seconds, LegErrorFaultCm));
				TickHalted(Measure, Level, Out, Dt, /*bWheelsMeasured=*/true);
				return;
			}
		}

		// The level-in's watchdog: ActionTimeoutFactor times the time the plan
		// gives it, and the ramp's own once it is known, plus
		// ActionTimeoutExtraSeconds.
		if (bLevelInBegun && !Level.bLevelled && LevelInFrom >= 0.0)
		{
			const double Planned = FMath::Max(Plan.LevelInSeconds,
				LevelRampFrom >= 0.0 ? LevelRampFrom - LevelInFrom + RampSpan + T.ServoLagSeconds + LevelInSpareSeconds : 0.0);
			const double Took = Elapsed - LevelInFrom;
			if (Took > ActionTimeoutFactor * Planned + ActionTimeoutExtraSeconds)
			{
				Fault(FString::Printf(TEXT("the level-in took %.1f s, more than %.0f x its planned %.2f s + %.0f (pitch %+.2f, roll %+.2f deg%s)"), Took,
					ActionTimeoutFactor, Planned, ActionTimeoutExtraSeconds, Level.FilteredAttitudeDeg.X, Level.FilteredAttitudeDeg.Y,
					Level.bAttitudeActive ? TEXT(", the attitude loop running") : TEXT(", the legs not yet on the terrain")));
				TickHalted(Measure, Level, Out, Dt, /*bWheelsMeasured=*/true);
				return;
			}
		}

		// Re-plan triggers: the heading turned from the plan's, a wheel's window
		// moved from where the plan has it (both from where the plan was made,
		// as measured). Before any wheel has begun to cross, a re-plan; after,
		// none is possible -- the heading that far off faults, a window is
		// logged (every trigger works on measured distances).
		{
			const double Yaw = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(FVector2D::DotProduct(Measure.Forward, Plan.Heading), -1.0, 1.0)));
			if (Yaw > ReplanYawDeg)
			{
				if (!HasCrossingBegun())
				{
					AskReplan(FString::Printf(TEXT("the heading turned %.1f deg from the plan's"), Yaw), FVector2D::ZeroVector, /*bHeading=*/true);
				}
				else
				{
					Fault(FString::Printf(TEXT("the heading turned %.1f deg from the plan's after a wheel began to cross (more than %.0f): the plan "
											   "no longer fits"),
						Yaw, ReplanYawDeg));
					TickHalted(Measure, Level, Out, Dt, /*bWheelsMeasured=*/true);
					return;
				}
			}
			for (int32 i = 0; i < N; ++i)
			{
				const FWheelPlan& P = Plan.Wheels[i];
				FExecWheel&		  W = Wheels[i];
				if (W.State != ERammsCurbWheelState::Planted || W.Plane != 0 || P.bHeldFromStart || S >= P.SLiftStart)
				{
					continue;
				}
				const double Drift = (W.PastEdgeCm - M->DistancePast(i, S, Measure.Legs[i].XZ.X)) / Cos;
				if (FMath::Abs(Drift) <= ReplanWindowCm)
				{
					continue;
				}
				const FString Why = FString::Printf(TEXT("'%s' %.1f cm %s its window than the plan has it"), *In.Legs[i].Name.ToString(), FMath::Abs(Drift),
					Drift > 0.0 ? TEXT("nearer") : TEXT("further from"));
				if (!HasCrossingBegun())
				{
					AskReplan(Why, FVector2D::ZeroVector);
				}
				else if (!W.bDriftLogged)
				{
					W.bDriftLogged = true;
					Log(FString::Printf(TEXT("%s (no re-plan once a wheel has begun to cross: its lift goes on its measured distance)"), *Why));
				}
			}
		}

		// The chassis height: from where it stands to the plan's, then the plan's
		// a servo lag ahead; at a planned stop, to where it leaves from.
		if (bLevelInBegun)
		{
			double To = bDwelling && Plan.Samples.IsValidIndex(NextStop + 1) ? Plan.Samples[NextStop + 1].B : Ahead.B;
			// A landing leg at its reach (set down a step, the chassis still a
			// little high) waits on the chassis, which the plan brings down with
			// travel. When the robot is held back meanwhile -- a lift that waits on
			// this landing has its deadline down at a standstill, a wait that would
			// never end -- the chassis comes down to the leg now instead, as far as
			// the plan's corridor here lets every other leg reach. Held down (the
			// most it has asked) while the wheel lands; then let go no faster than
			// the plan's own height comes down to it (or ReachReleaseCmS), never
			// back up in a step.
			bool bLanding = false;
			for (const FExecWheel& W : Wheels)
			{
				bLanding = bLanding || W.State == ERammsCurbWheelState::Planting || W.State == ERammsCurbWheelState::Loading;
			}
			const double Room = ReachRoom();
			const bool	 bHeldBack = VCmd <= HoldSpeedCmS + 1.0 && (bHolding || Diag.TargetBy == EExecTerm::Deadline);
			if (!bLanding)
			{
				ReachLowerCm = FMath::Max(0.0, FMath::Min(ReachLowerCm - ReachReleaseCmS * Dt, Between(S).B - BCmd));
			}
			else if (ReachDrop() > 0.0 && bHeldBack)
			{
				ReachLowerCm = FMath::Max(ReachLowerCm, ReachDrop() + ReachDropExtraCm);
			}
			const double Drop = FMath::Min(ReachLowerCm, Room);
			if (Drop > 0.0)
			{
				To = FMath::Min(To, Between(S).B - Drop);
				if (!bReachLowering)
				{
					bReachLowering = true;
					Log(FString::Printf(TEXT("a landing leg at its reach: the chassis comes down %.2f cm below the plan to it"), Drop));
				}
			}
			else
			{
				bReachLowering = false;
			}
			// A wheel the plan has off the ground here that still carries (its lift
			// late, or waiting on another wheel's landing): the plan's height is
			// for that wheel up, and may be more than its leg reaches on the
			// ground -- the leg would bottom out, and the base tip over that
			// corner. The chassis no higher than it reaches, until it lifts.
			if (const FScheduleSample* Sm = Plan.SampleAt(S))
			{
				double Ceiling = TNumericLimits<double>::Max();
				int32  By = INDEX_NONE;
				for (int32 i = 0; i < N; ++i)
				{
					if (((Sm->AirborneMask >> i) & 1u) && IsSupport(i))
					{
						const double C = SupportCeiling(i, Measure, Ahead, AheadCm, Bias);
						if (C < Ceiling)
						{
							Ceiling = C;
							By = i;
						}
					}
				}
				// (Logged once it holds the chassis a little: 0.25 cm.)
				if (Ceiling < To)
				{
					if (!bSupportCeiling && To - Ceiling > 0.25)
					{
						bSupportCeiling = true;
						Log(FString::Printf(TEXT("the chassis held %.2f cm under the plan's height: '%s' still carries, at its reach"), Between(S).B - Ceiling,
							*In.Legs[By].Name.ToString()));
					}
					To = Ceiling;
				}
				else if (bSupportCeiling)
				{
					bSupportCeiling = false;
					Log(TEXT("the chassis follows the plan's height again"));
				}
			}
			BCmd = StepTowards(BCmd, To, BH * Dt * (bDwelling ? 1.0 : 1.5));
			if (!bLoweredDown)
			{
				// The level-in's first stage: the corners the stance holds up are
				// set down while every other leg holds the stance, so the chassis
				// is where it stands, not where the plan will have it -- a corner
				// set down to reach the plan's chassis height (above the stance's)
				// would jack its side up and could lift the main wheels off the
				// ground, and on a slope the robot would slide on its free-rolling
				// wheels.
				BCmd = Measure.OriginZ;
			}
		}
		else
		{
			BCmd = Measure.OriginZ;
		}
		BCmdLag = bLevelInBegun ? LagStep(static_cast<float>(BCmdLag), static_cast<float>(BCmd), static_cast<float>(Dt), T.ServoLagSeconds) : BCmd;

		// The attitude loop (once the level-in has the legs on the terrain), its
		// integral held with fewer than three wheels carrying or one loading.
		int32 NumPlanted = 0;
		bool  bAnyLoading = false;
		for (int32 i = 0; i < N; ++i)
		{
			NumPlanted += Wheels[i].State == ERammsCurbWheelState::Planted ? 1 : 0;
			bAnyLoading = bAnyLoading || Wheels[i].State == ERammsCurbWheelState::Loading;
		}
		const FVector2D Correction = Level.TickAttitude(Measure.AttitudeDeg, PlannedAttitude, static_cast<float>(Dt),
			NumPlanted >= MinPlantedForIntegral && !bAnyLoading, T);
		{
			TArray<FVector2D, TInlineAllocator<8>> XY;
			TArray<bool, TInlineAllocator<8>>	   Support;
			TArray<float, TInlineAllocator<8>>	   Dz;
			for (int32 i = 0; i < N; ++i)
			{
				XY.Add(FVector2D(Measure.Legs[i].XZ.X, M->LegY(i)));
				Support.Add(IsSupport(i));
			}
			Dz.Init(0.0f, N);
			AttitudeOffsets(XY, Support, Correction, Dz);
			for (int32 i = 0; i < N; ++i)
			{
				Wheels[i].AttitudeDz = Dz[i];
			}
			// The support legs are pressed into their surfaces by different amounts
			// (MainWheelPreload on the main wheels): standing on them, the base would
			// tilt by the plane that best fits those presses -- afresh whenever the
			// set on the ground changes unevenly. That plane's tilt is taken out of
			// their targets (its mean stays: every leg pressed by it), so the presses
			// share the load out among the legs without tilting the base, and the
			// attitude loop has only the rest to trim.
			double Sx = 0, Sy = 0, Sxx = 0, Syy = 0, Sxy = 0, Sp = 0, Sxp = 0, Syp = 0;
			int32  n = 0;
			for (int32 i = 0; i < N; ++i)
			{
				Wheels[i].PreloadTiltDz = 0.0;
				if (!Support[i])
				{
					continue;
				}
				const double X = XY[i].X, Y = XY[i].Y, P = M->Preload(i);
				Sx += X;
				Sy += Y;
				Sxx += X * X;
				Syy += Y * Y;
				Sxy += X * Y;
				Sp += P;
				Sxp += X * P;
				Syp += Y * P;
				++n;
			}
			if (n >= 3)
			{
				const double A11 = n, A12 = Sx, A13 = Sy, A22 = Sxx, A23 = Sxy, A33 = Syy;
				const double Det = A11 * (A22 * A33 - A23 * A23) - A12 * (A12 * A33 - A23 * A13) + A13 * (A12 * A23 - A22 * A13);
				if (FMath::Abs(Det) > 1e-6)
				{
					const double Bx = (A11 * (Sxp * A33 - A23 * Syp) - Sp * (A12 * A33 - A23 * A13) + A13 * (A12 * Syp - Sxp * A13)) / Det;
					const double Cy = (A11 * (A22 * Syp - Sxp * A23) - A12 * (A12 * Syp - Sxp * A13) + Sp * (A12 * A23 - A22 * A13)) / Det;
					const double Mean = (Bx * Sx + Cy * Sy) / n;
					for (int32 i = 0; i < N; ++i)
					{
						if (Support[i])
						{
							Wheels[i].PreloadTiltDz = Bx * XY[i].X + Cy * XY[i].Y - Mean;
						}
					}
				}
			}
		}

		// The wheels' states.
		for (int32 i = 0; i < N; ++i)
		{
			const FWheelPlan& P = Plan.Wheels[i];
			FExecWheel&		  W = Wheels[i];
			const FExecLeg&	  L = Measure.Legs[i];
			switch (W.State)
			{
				case ERammsCurbWheelState::Held:
					if (W.bLower)
					{
						if (bLevelInBegun)
						{
							W.Plane = 0;
							SetState(i, ERammsCurbWheelState::Planting, TEXT("set down for the level-in"));
						}
					}
					else if (bLevelInBegun && L.WheelWorld.Z >= W.ZTop + SwingRaise - ClearToleranceCm)
					{
						SetState(i, ERammsCurbWheelState::Clear, TEXT("carried up from the drive stance"));
					}
					break;
				case ERammsCurbWheelState::Planted:
				{
					if (W.Plane != 0 || P.bHeldFromStart)
					{
						break;
					}
					// On the plan's position; or sooner when the robot runs ahead of the
					// plan, by the time left to its window at the speeds it will run.
					// (A centre wheel after the walk the plan has it make first, as far
					// as it can make it, as the deadline takes it.)
					double ToWindow = (P.DUp - W.PastEdgeCm) / Cos;
					ToWindow += WalkCredit(i, Measure, ToWindow);
					const bool bOnPosition = S >= P.SLiftStart - 1e-6;
					const bool bOnTime = Level.bLevelled && VHat > 1.0 && SecondsTo(S + ToWindow) <= P.TLift;
					if (!bOnPosition && !bOnTime)
					{
						break;
					}
					FString Why;
					if (LiftPermit(i, Measure, Level, Why))
					{
						if (W.bBlocked)
						{
							++Counts.LiftsBlocked;
						}
						W.bBlocked = false;
						W.BlockedWhy.Reset();
						++(bOnPosition ? Counts.LiftsOnPosition : Counts.LiftsOnTime);
						SetState(i, ERammsCurbWheelState::Lifting,
							bOnPosition ? FString::Printf(TEXT("on the plan's position (%.1f)"), P.SLiftStart)
										: FString::Printf(TEXT("ahead of the plan's position (%.1f): %.2f s to its window"), P.SLiftStart,
											  SecondsTo(S + ToWindow)));
					}
					else
					{
						if (!W.bBlocked || W.BlockedWhy != Why)
						{
							Log(FString::Printf(TEXT("%-12s lift blocked: %s"), *In.Legs[i].Name.ToString(), *Why));
						}
						W.bBlocked = true;
						W.BlockedWhy = Why;
					}
					break;
				}
				case ERammsCurbWheelState::Lifting:
					if (L.WheelWorld.Z >= W.ZTop + SwingRaise - ClearToleranceCm)
					{
						// Up and clear: a climb's lift that took LateActionSeconds longer
						// than planned is logged late.
						const double Took = Elapsed - W.Since;
						const bool	 bLate = M->IsClimb() && Took > P.TLift + LateActionSeconds;
						Counts.LateActions += bLate ? 1 : 0;
						SetState(i, ERammsCurbWheelState::Clear, bLate ? FString::Printf(TEXT("late: %.2f s against the %.2f s planned"), Took, P.TLift) : FString());
					}
					else if (W.PastEdgeCm >= P.DDown)
					{
						W.Plane = 1;
						SetState(i, ERammsCurbWheelState::Planting, TEXT("past its window before it was clear"));
					}
					else if (W.BottomCm <= AbortWithinCm)
					{
						// Still near the ground: the robot measured with it (and every
						// other wheel off the ground) up must still stand; if not -- or
						// holding on the measured margin -- it goes back down, and its
						// lift waits for its permit again.
						const double Margin = MarginWithout(Measure, 0);
						if (Margin < T.MinLiveStabilityMarginCm - AbortHysteresisCm || bMarginHold)
						{
							++Counts.LiftsAborted;
							W.bBlocked = true;
							W.BlockedWhy = FString::Printf(TEXT("with it up the robot stood %.2f cm inside its wheels (MinLiveStabilityMarginCm %.1f)"),
								Margin, T.MinLiveStabilityMarginCm);
							SetState(i, ERammsCurbWheelState::Planted, FString::Printf(TEXT("lift aborted %.2f cm off the ground: %s"), W.BottomCm, *W.BlockedWhy));
						}
					}
					else if (M->IsClimb() && Elapsed - W.Since > ActionTimeoutFactor * P.TLift + ActionTimeoutExtraSeconds)
					{
						// A climb's lift is the leg's rise to clear the curb top: it
						// never got there.
						Fault(FString::Printf(TEXT("'%s' (lifting) took %.1f s and did not lift clear, more than %.0f x its planned %.2f s + %.0f (%.1f cm "
												   "under the height that clears its window)"),
							*In.Legs[i].Name.ToString(), Elapsed - W.Since, ActionTimeoutFactor, P.TLift, ActionTimeoutExtraSeconds,
							W.ZTop + SwingRaise - L.WheelWorld.Z));
						TickHalted(Measure, Level, Out, Dt, /*bWheelsMeasured=*/true);
						return;
					}
					break;
				case ERammsCurbWheelState::Clear:
					// Past its window where it will be set down (the plan's landing).
					if (W.LandedPastCm >= P.DDown && S >= P.SLand - 1.0)
					{
						W.Plane = 1;
						SetState(i, ERammsCurbWheelState::Planting, FString());
					}
					break;
				case ERammsCurbWheelState::Planting:
				{
					// The trace under it: the ground there against where it is set
					// down to (Lower, cm: positive lower, negative higher).
					double Lower = 0.0;
					bool   bNone = false;
					bool   bSeen = false;
					for (const FTerrainProbe& Pr : Measure.Probes)
					{
						if (Pr.Leg != i || FMath::Abs(Pr.AheadCm) > 1e-3)
						{
							continue;
						}
						const double Expect = (W.Plane == 1 ? Terrain.Far : Terrain.Near).HeightAt(Pr.XY) + (W.Plane == 1 ? Bias.Y : Bias.X);
						if (Pr.bHit || (Expect <= Pr.AboveZ && Expect >= Pr.BelowZ))
						{
							bSeen = true;
							bNone = !Pr.bHit;
							Lower = Expect - (Pr.bHit ? Pr.Z : Pr.BelowZ);
						}
					}
					// Touchdown: its bottom on the surface -- as planned, or as the
					// trace under it finds it, within LandingToleranceCm above the
					// plan (a surface its bias has not taken up yet) -- or its leg
					// stopped above where the servo lag model has it, or stopped with
					// the model there. A leg still going down, if slower than the
					// model, is not down. Stopped (or pressing) within
					// LandingToleranceCm of its surface it is down; further above, the
					// landing check below faults: the two bands meet, so no landing
					// stopped short is left pressing on until its timeout.
					const bool bNear = W.BottomCm <= T.LandingToleranceCm + TerrainToleranceCm;
					const bool bOnTraced = bSeen && !bNone && Lower >= -(T.LandingToleranceCm + TerrainToleranceCm) && W.BottomCm + Lower <= TouchCm;
					const bool bTouch = W.BottomCm <= TouchCm || (bNear && bOnTraced);
					const bool bLegStopped = W.MeasVz > -StallSpeedCmS;
					// A leg stopped at its reach has not touched down: it waits on the
					// chassis.
					const bool bStall = bNear && bLegStopped && L.XZ.Y - L.LagZ >= StallCm && W.CmdZ < L.XZ.Y;
					const bool bModel = bNear && bLegStopped && !W.bReachClamped && L.LagZ <= W.TargetZ + 0.05 && W.Since + 0.5 < Elapsed;
					// Or pressing on the ground: its leg still going down, the wheel
					// not -- the leg lifting the robot instead (FPressWatch).
					const bool bPressing = W.Press.Tick(L.WheelWorld.Z, L.XZ.Y, Dt);
					const bool bPress = bNear && bPressing;
					// The landing check. Stopped by something above where it is to
					// land by more than LandingToleranceCm -- the lip, a thing lying
					// there -- (for TouchdownFaultSeconds), or pressing on it there:
					// a fault, naming it.
					const bool bAbove = W.BottomCm > T.LandingToleranceCm + TerrainToleranceCm;
					const bool bHeldUp = bLegStopped && L.XZ.Y - L.LagZ >= StallCm && W.CmdZ < L.XZ.Y && bAbove;
					if (W.LandHigh.Tick(bHeldUp, Dt, TouchdownFaultSeconds) || (bPressing && bAbove))
					{
						W.bLandFault = true;
						Fault(FString::Printf(TEXT("'%s' (planting) set down %.1f cm above where it was to land %.1f cm past the edge (on the lip, or "
												   "something on its surface), more than LandingToleranceCm %.1f"),
							*In.Legs[i].Name.ToString(), W.BottomCm, W.PastEdgeCm, T.LandingToleranceCm));
						TickHalted(Measure, Level, Out, Dt, /*bWheelsMeasured=*/true);
						return;
					}
					// Planting too long (its time at its reach, waiting on the chassis,
					// left out): ActionTimeoutFactor times its planned plant plus
					// ActionTimeoutExtraSeconds.
					W.PlantingFor += W.bReachClamped ? 0.0 : Dt;
					if (W.PlantingFor > ActionTimeoutFactor * P.TPlant + ActionTimeoutExtraSeconds)
					{
						W.bLandFault = true;
						Fault(FString::Printf(TEXT("'%s' (planting) took %.1f s and did not touch down, more than %.0f x its planned %.2f s + %.0f (its "
												   "bottom %+.1f cm over where it was to land)"),
							*In.Legs[i].Name.ToString(), W.PlantingFor, ActionTimeoutFactor, P.TPlant, ActionTimeoutExtraSeconds, W.BottomCm));
						TickHalted(Measure, Level, Out, Dt, /*bWheelsMeasured=*/true);
						return;
					}
					// No ground where it is to land: the trace under it finds the
					// surface more than LandingToleranceCm lower than it is set down
					// to (or nothing at all) -- before it touches, for
					// TouchdownFaultSeconds. Lifted clear again, and a fault.
					{
						if (W.NoGround.Tick(bSeen && Lower > T.LandingToleranceCm + TerrainToleranceCm, Dt, TouchdownFaultSeconds))
						{
							W.bRelift = true;
							Fault(FString::Printf(TEXT("'%s' (planting) found no ground where it was to land %.1f cm past the edge: the surface under it "
													   "%s%.1f cm lower than planned, more than LandingToleranceCm %.1f; lifted clear again"),
								*In.Legs[i].Name.ToString(), W.PastEdgeCm, bNone ? TEXT("at least ") : TEXT(""), Lower, T.LandingToleranceCm));
							TickHalted(Measure, Level, Out, Dt, /*bWheelsMeasured=*/true);
							return;
						}
					}
					if (bTouch || bStall || bModel || bPress)
					{
						// Down late by more than LateActionSeconds: logged.
						const double Took = Elapsed - W.Since;
						const bool	 bLate = W.Plane == 1 && Took > P.TPlant + LateActionSeconds;
						Counts.LateActions += bLate ? 1 : 0;
						// Down: its command back to where the leg stopped (it can go no
						// further), and from there to its load -- moved on as the legs
						// carrying the robot are: each runs its command ahead of where it
						// is by the servo lag of the chassis' motion (the chassis coming
						// down after a descent's landing, rising on a climb), and a leg
						// that joined them with no such lead would trail them by it, then
						// catch up only after its command had: the base would tip over its
						// corner.
						double Lead = 0.0, Rate = 0.0;
						int32  Carrying = 0;
						for (int32 j = 0; j < N; ++j)
						{
							if (j != i && Wheels[j].State == ERammsCurbWheelState::Planted)
							{
								Lead += Wheels[j].CmdZ - Measure.Legs[j].LagZ;
								Rate += Wheels[j].CmdVz;
								++Carrying;
							}
						}
						Lead = Carrying > 0 ? FMath::Clamp(Lead / Carrying, -TouchdownLeadMaxCm, TouchdownLeadMaxCm) : 0.0;
						W.CmdZ = FMath::Max(W.CmdZ, L.XZ.Y - FMath::Min(M->Preload(i), PlantProbeCm)) + Lead;
						W.CmdVz = Carrying > 0 ? Rate / Carrying : 0.0;
						W.TouchZ = W.CmdZ;
						SetState(i, ERammsCurbWheelState::Loading,
							FString::Printf(TEXT("touched down (%s), its bottom %+.2f cm%s"),
								bTouch		 ? (W.BottomCm <= TouchCm ? TEXT("on the surface") : TEXT("on the surface traced under it"))
									: bStall ? TEXT("its leg stopped")
									: bPress ? TEXT("pressing on the ground")
											 : TEXT("the lag model there"),
								W.BottomCm,
								bLate ? *FString::Printf(TEXT("; late: %.2f s against the %.2f s planned"), Took, P.TPlant) : TEXT("")));
					}
					break;
				}
				case ERammsCurbWheelState::Loading:
					// Loaded once it has pressed down for LoadSeconds: not while its leg
					// is at its reach, unable to.
					W.Loading += W.bReachClamped ? 0.0 : Dt;
					if (W.Loading >= LoadSeconds)
					{
						const FString Check = FMath::Abs(W.BottomCm) > T.SwingClearance * 0.6
							? FString::Printf(TEXT("landed %+.2f cm off the plane"), W.BottomCm)
							: FString();
						SetState(i, ERammsCurbWheelState::Planted, Check);
					}
					break;
				default:
					break;
			}
		}

		// The level-in's first stage: the corners the drive stance holds up and the
		// plan has down set down first, before any other leg moves.
		if (bLevelInBegun && !bLoweredDown)
		{
			bLoweredDown = true;
			for (int32 i = 0; i < N; ++i)
			{
				bLoweredDown = bLoweredDown && !(Wheels[i].bLower && Wheels[i].State != ERammsCurbWheelState::Planted);
			}
		}
		if (bLoweredDown && LevelRampFrom < 0.0)
		{
			// The level-in's second stage: every support leg from where the stance
			// has it to its level target, together, along one S-curve in time --
			// its rate and the rate's change bounded so that no leg moves faster
			// than LegSpeed / LegAcceleration and the base turns from the
			// stance's attitude to the plan's (pitch and roll together, along
			// the straight way between them) no faster than TiltRateDegPerSec /
			// TiltAccelDegPerSec2.
			LevelRampFrom = Elapsed;
			double Move = 0.0;
			for (int32 i = 0; i < N; ++i)
			{
				FExecWheel& W = Wheels[i];
				W.StanceZ = W.CmdZ;
				if (IsSupport(i))
				{
					const double X = M->IsCentre(i) ? W.CmdX : Measure.Legs[i].XZ.X;
					const double To = ClampToReach(i, X, SupportTarget(i, Measure, Ahead, AheadCm, M->Preload(i), Bias) + W.PreloadTiltDz);
					Move = FMath::Max(Move, FMath::Abs(To - W.CmdZ));
				}
			}
			const FVector2D Off = Measure.AttitudeDeg - PlannedAttitude;
			const double	Turn = Off.Size();
			RampRate = FMath::Min(T.TiltRateDegPerSec / FMath::Max(Turn, 1e-3), LegSpeed / FMath::Max(Move, 1e-3));
			RampAccel = FMath::Min(T.TiltAccelDegPerSec2 / FMath::Max(Turn, 1e-3), LegAccel / FMath::Max(Move, 1e-3));
			RampRate = FMath::Max(RampRate, 1e-3);
			RampAccel = FMath::Max(RampAccel, 1e-3);
			RampSpan = SCurveSeconds(1.0, RampRate, RampAccel);
			Log(FString::Printf(TEXT("levelling in: the legs to the terrain over %.2f s (the base %.2f deg off the plan's attitude, the legs "
									 "%.2f cm at most off their targets)"),
				RampSpan, Turn, Move));
		}
		// The level-in's ramp: the share of the move done (0 to 1); done once
		// the ramp is complete and the base is level.
		double Ramp = 1.0, RampLeft = 0.0;
		bool   bRampDone = false;
		if (LevelRampFrom >= 0.0)
		{
			const double Into = Elapsed - LevelRampFrom;
			bRampDone = Into >= RampSpan;
			if (!Level.bLevelled)
			{
				Ramp = bRampDone ? 1.0 : SCurveShare(Into, 1.0, RampRate, RampAccel);
				RampLeft = FMath::Max(0.0, RampSpan - Into);
			}
		}
		else if (bLevelInBegun)
		{
			RampLeft = FMath::Max(0.05, Plan.LevelInSeconds - static_cast<double>(T.ServoLagSeconds) - 0.3);
		}

		// A wheel the plan has down by now still off the ground (landing late):
		// the centre wheels hold their fore/aft until it is down -- the walk the
		// plan makes from there on is for the robot standing on it. They then lag
		// the plan, which slows the robot and holds the lifts that need them.
		int32 LateLeg = INDEX_NONE;
		for (int32 j = 0; j < N && LateLeg == INDEX_NONE; ++j)
		{
			LateLeg = IsAirborne(Wheels[j].State) && S > Plan.Wheels[j].SLoaded ? j : INDEX_NONE;
		}
		if ((LateLeg != INDEX_NONE) != bCentresHeld)
		{
			bCentresHeld = LateLeg != INDEX_NONE;
			Log(bCentresHeld ? FString::Printf(TEXT("the centre wheels hold their fore/aft: '%s' is not down where the plan has it (%.1f)"),
								   *In.Legs[LateLeg].Name.ToString(), Plan.Wheels[LateLeg].SLoaded)
							 : FString(TEXT("the centre wheels walk on")));
		}

		// The legs' targets and commands.
		bool bCaughtUp = true;
		for (int32 i = 0; i < N; ++i)
		{
			FExecWheel&		W = Wheels[i];
			const FExecLeg& L = Measure.Legs[i];
			const bool		bCentre = M->IsCentre(i);
			if (bCentre && bLevelInBegun && bLoweredDown)
			{
				// A 5-bar walks to the plan's fore/aft a servo lag ahead; at a planned
				// stop, to where the robot leaves from.
				const int32	 Side = M->Side(i);
				const double To = bDwelling && Plan.Samples.IsValidIndex(NextStop + 1) ? Plan.Samples[NextStop + 1].X[Side] : Ahead.X[Side];
				if (!bCentresHeld)
				{
					W.CmdX = StepTowards(W.CmdX, To, CSS * 1.25 * Dt);
				}
			}
			if (!bLevelInBegun)
			{
				// The drive stance: hold.
				W.TargetZ = W.CmdZ;
				continue;
			}
			const bool bStage1 = !bLoweredDown && !W.bLower;
			// A corner set down in the first stage takes its load where it touched
			// down -- the rest of its preload pressed in from there, then held, as
			// every other leg holds the stance -- not in the plan's level pose,
			// which would have it reach for a chassis that is not there yet.
			const bool	 bStage1Corner = !bLoweredDown && W.bLower;
			const double Pressed = FMath::Min(M->Preload(i), PlantProbeCm);
			double		 Target = W.CmdZ;
			switch (W.State)
			{
				case ERammsCurbWheelState::Planted:
					if (bStage1Corner)
					{
						Target = W.TouchZ - (M->Preload(i) - Pressed);
					}
					else if (!bStage1)
					{
						Target = SupportTarget(i, Measure, Ahead, AheadCm, M->Preload(i), Bias) + W.AttitudeDz + W.PreloadTiltDz;
						if (Ramp < 1.0)
						{
							Target = FMath::Lerp(W.StanceZ, Target, Ramp);
						}
					}
					break;
				case ERammsCurbWheelState::Loading:
					if (bStage1Corner)
					{
						Target = W.TouchZ - (M->Preload(i) - Pressed) * FMath::Clamp(W.Loading / LoadSeconds, 0.0, 1.0);
					}
					else if (!bStage1)
					{
						// Pressing its preload in over the load time.
						const double Probe = FMath::Min(M->Preload(i), PlantProbeCm);
						const double Press = FMath::Lerp(Probe, M->Preload(i), FMath::Clamp(W.Loading / LoadSeconds, 0.0, 1.0));
						Target = SupportTarget(i, Measure, Ahead, AheadCm, Press, Bias) + W.AttitudeDz + W.PreloadTiltDz;
						if (Ramp < 1.0)
						{
							Target = FMath::Lerp(W.StanceZ, Target, Ramp);
						}
					}
					break;
				case ERammsCurbWheelState::Planting:
					Target = PlantingTarget(i, Measure, AheadCm, Bias, FMath::Min(M->Preload(i), PlantProbeCm));
					break;
				default: // Lifting, Clear, Held up
					if (!bStage1)
					{
						Target = SwingTarget(i, Measure, Ahead, SwingWorldZ(i, Measure, V));
					}
					break;
			}
			const double X = bCentre ? W.CmdX : L.XZ.X;
			const double Wanted = Target;
			Target = ClampToReach(i, X, Target);
			W.ReachDeficitCm = FMath::Max(0.0, Target - Wanted);
			W.bReachClamped = W.ReachDeficitCm > 0.05;
			W.TargetZ = Target;
			TrackTrapezoid(W.CmdZ, W.CmdVz, Target, LegSpeed, LegAccel, Dt);
			W.CmdZ = ClampToReach(i, X, W.CmdZ);
			if (IsSupport(i) && FMath::Abs(W.CmdZ - Target) > ExecCaughtUpCm)
			{
				bCaughtUp = false;
			}
		}
		for (int32 i = 0; i < N; ++i)
		{
			Out.LegZ[i] = Wheels[i].CmdZ;
			Out.LegX[i] = Wheels[i].CmdX;
		}

		// The level-in is done once the ramp is complete and the support legs are
		// on the terrain, and the attitude loop (on from there) has the base
		// within LevelInToleranceDeg of the plan's.
		if (bLevelInBegun && !Level.bLevelled && bLoweredDown && bRampDone && bCaughtUp)
		{
			if (!Level.bAttitudeActive)
			{
				Level.bAttitudeActive = true;
				Log(FString::Printf(TEXT("levelling in: legs on the terrain, pitch %+.2f, roll %+.2f deg; the attitude loop takes the rest"),
					Level.FilteredAttitudeDeg.X, Level.FilteredAttitudeDeg.Y));
			}
			const FVector2D Err = Level.FilteredAttitudeDeg - PlannedAttitude;
			if (FMath::Abs(Err.X) <= LevelInToleranceDeg && FMath::Abs(Err.Y) <= LevelInToleranceDeg)
			{
				Level.bLevelled = true;
				BOffset = Measure.OriginZ - BCmdLag;
				// How long it took against what the plan gave it (late by more than
				// LateActionSeconds: counted).
				const double Took = LevelInFrom >= 0.0 ? Elapsed - LevelInFrom : 0.0;
				const double Planned = FMath::Max(Plan.LevelInSeconds, LevelRampFrom >= 0.0 && LevelInFrom >= 0.0 ? LevelRampFrom - LevelInFrom + RampSpan + T.ServoLagSeconds + LevelInSpareSeconds : 0.0);
				const bool	 bLate = Took > Planned + LateActionSeconds;
				Counts.LateActions += bLate ? 1 : 0;
				Log(FString::Printf(TEXT("level: pitch %+.2f, roll %+.2f deg (planned %+.2f / %+.2f); the chassis %+.2f cm off its command; in %.2f s "
										 "(planned %.2f%s)"),
					Level.FilteredAttitudeDeg.X, Level.FilteredAttitudeDeg.Y, PlannedAttitude.X, PlannedAttitude.Y, BOffset, Took, Planned,
					bLate ? TEXT(", late") : TEXT("")));
			}
		}

		// Swing clearance: a clear wheel's rim, inside its window, nearer than
		// SwingFaultCm to the terrain the probes find under it stops the robot;
		// still there after SwingFaultSeconds, a fault.
		bSwingStop = false;
		int32  SwingLeg = INDEX_NONE;
		double SwingGap = 0.0;
		for (int32 i = 0; i < N; ++i)
		{
			FExecWheel&		  W = Wheels[i];
			const FWheelPlan& P = Plan.Wheels[i];
			if (W.State != ERammsCurbWheelState::Clear || W.PastEdgeCm <= P.DUp || W.PastEdgeCm >= P.DDown)
			{
				W.SwingLow.Reset();
				continue;
			}
			const double Gap = RimGap(i, Measure.Legs[i].WheelWorld, M->Radius(i), M->RollDirection(i), M->TyreWidth(i), Measure.Probes);
			const bool	 bLow = Gap < SwingFaultCm;
			if (W.SwingLow.Tick(bLow, Dt, SwingFaultSeconds))
			{
				Fault(FString::Printf(TEXT("'%s' (clear) %.2f cm over the terrain under it, %.1f cm past the edge inside its window, for %.1f s (under "
										   "%.1f)"),
					*In.Legs[i].Name.ToString(), Gap, W.PastEdgeCm, W.SwingLow.Seconds, SwingFaultCm));
				TickHalted(Measure, Level, Out, Dt, /*bWheelsMeasured=*/true);
				return;
			}
			if (bLow && (!bSwingStop || Gap < SwingGap))
			{
				bSwingStop = true;
				SwingLeg = i;
				SwingGap = Gap;
			}
		}

		// Phases.
		bool bAllFar = true;
		for (int32 i = 0; i < N; ++i)
		{
			bAllFar = bAllFar && Wheels[i].State == ERammsCurbWheelState::Planted && Wheels[i].Plane == 1;
		}
		if (bAllFar && Phase != EExecPhase::Settle && Phase != EExecPhase::Done)
		{
			Phase = EExecPhase::Settle;
			Log(TEXT("every wheel on the far side: settling"));
		}
		else if (!bAllFar && Level.bLevelled && Phase == EExecPhase::Prepare)
		{
			Phase = EExecPhase::Cross;
		}

		// A planned stop: braked onto it, resting there while the centre wheels and
		// the chassis re-pose, then on.
		double StopCap = TNumericLimits<double>::Max();
		if (Plan.Samples.IsValidIndex(NextStop))
		{
			const double SStop = Plan.Samples[NextStop].S;
			if (bDwelling)
			{
				DwellFor += Dt;
				const FScheduleSample& To = Plan.Samples[FMath::Min(NextStop + 1, Plan.Samples.Num() - 1)];
				bool				   bThere = FMath::Abs(BCmd - To.B) < 0.05;
				for (int32 Side = 0; Side < 2 && bThere; ++Side)
				{
					const int32 C = Centre[Side];
					bThere = C == INDEX_NONE
						|| (FMath::Abs(Wheels[C].CmdX - To.X[Side]) < 0.05 && FMath::Abs(Measure.Legs[C].XZ.X - To.X[Side]) < T.SettleTolerance);
				}
				StopCap = 0.0;
				if (bThere || DwellFor > 10.0)
				{
					Log(FString::Printf(TEXT("planned stop over after %.2f s%s"), DwellFor, bThere ? TEXT("") : TEXT(" (the re-pose did not settle)")));
					bDwelling = false;
					SResume = FMath::Max(SResume, SStop);
					const int32 Was = NextStop;
					NextStop = INDEX_NONE;
					for (int32 k = Was + 1; k < Plan.Samples.Num(); ++k)
					{
						if (Plan.Samples[k].bStop && Plan.Samples[k].S > SStop)
						{
							NextStop = k;
							break;
						}
					}
					StopCap = TNumericLimits<double>::Max();
				}
			}
			else
			{
				StopCap = BrakingSpeed(LagBrakingDistance(static_cast<float>(SStop - S), static_cast<float>(VHat), DriveLagSeconds),
					T.Deceleration);
				if (S >= SStop - 1.0 && VHat < HoldSpeedCmS)
				{
					bDwelling = true;
					DwellFor = 0.0;
					++Counts.PlannedStops;
					Log(FString::Printf(TEXT("planned stop at %.1f cm: re-posing (%.2f s planned)"), SStop, Plan.Samples[NextStop].DwellSeconds));
				}
			}
		}

		// The speed.
		double	  VTarget = TNumericLimits<double>::Max();
		EExecTerm By = EExecTerm::Plan;
		{
			// The plan's, a little ahead: its least over the next stretch -- read
			// from the plan's origin on when the robot stands behind it (it rolled
			// back after the plan was made): the speed the plan rises from rest at
			// there, so that the robot drives up to the plan rather than resting at
			// its first sample.
			// Likewise after a planned stop, from the stop on. Short of a planned
			// stop, no nearer it than its last sample before it: braking onto the
			// stop itself is the stop's cap (lag-aware), which brings the robot to
			// rest on it -- the stop's own sample, at rest, read a little ahead,
			// would leave it short of the stop, where it never begins to rest.
			const double From = FMath::Max(S, SResume);
			const double Look = 1.0 + V * SpeedAheadSeconds;
			double		 LookFrom = From + 1.0, LookTo = From + Look;
			if (Plan.Samples.IsValidIndex(NextStop) && !bDwelling)
			{
				const double Before = Plan.Samples[NextStop].S - Plan.SampleCm;
				LookFrom = FMath::Min(LookFrom, Before);
				LookTo = FMath::Min(LookTo, Before);
			}
			for (double At = LookFrom; At <= LookTo + 1e-6; At += Plan.SampleCm)
			{
				VTarget = FMath::Min(VTarget, Between(At).V);
			}
			VTarget = FMath::Min(VTarget, Between(LookTo).V);
		}
		Diag = FExecDiag();
		Diag.Plan = VTarget;
		auto Limit = [&VTarget, &By](double Cap, EExecTerm Term) {
			if (Cap < VTarget)
			{
				VTarget = Cap;
				By = Term;
			}
		};
		int32		 DeadlineLeg = INDEX_NONE;
		const double Deadline = DeadlineCap(Measure, DeadlineLeg);
		Limit(Deadline, EExecTerm::Deadline);
		Diag.Deadline = Deadline;
		Diag.DeadlineLeg = DeadlineLeg;
		Diag.LateLevel = TNumericLimits<double>::Max();
		Diag.Track = TNumericLimits<double>::Max();
		// A level-in that is late: no nearer than where it must be done by, and no
		// sooner than it can be.
		if (!Level.bLevelled)
		{ // On time for the line the plan has it done by (30 cm short of the first
			// window); braked, allowing for the chassis' lag, short of where the
			// first lift begins (no lift begins before the base is level).
			const double Room = Plan.LevelInLatestS - LevelCapMarginCm - S;
			double		 FirstLift = TNumericLimits<double>::Max();
			for (const FWheelPlan& P : Plan.Wheels)
			{
				FirstLift = P.bHeldFromStart ? FirstLift : FMath::Min(FirstLift, P.SLiftStart);
			}
			const double StopRoom = LagBrakingDistance(static_cast<float>(FMath::Min(FirstLift, Plan.Samples.Last().S) - LevelInStopShortCm - S),
				static_cast<float>(VHat), DriveLagSeconds);
			double		 Left = Plan.LevelInSeconds;
			if (bLevelInBegun)
			{
				Left = RampLeft + T.ServoLagSeconds;
				for (int32 i = 0; i < N; ++i)
				{
					if (IsSupport(i) || Wheels[i].bLower)
					{
						Left = FMath::Max(Left, static_cast<double>(LegMoveSeconds(static_cast<float>(Wheels[i].TargetZ - Wheels[i].CmdZ), static_cast<float>(LegSpeed), static_cast<float>(Measure.Legs[i].XZ.Y - Wheels[i].CmdZ), T.ServoLagSeconds, 0.5f * T.SettleTolerance)));
					}
				}
			}
			const double Gate = bLevelInBegun ? 0.0 : FMath::Max(0.0, Plan.LevelInFromS - S);
			const double OnTime = Room <= 0.0 ? 0.0 : FMath::Max(0.0, Room - Gate) / FMath::Max(Left, static_cast<double>(MinDeadlineSeconds));
			Diag.LateLevel = FMath::Min(OnTime, static_cast<double>(BrakingSpeed(static_cast<float>(StopRoom), T.Deceleration)));
			Limit(Diag.LateLevel, EExecTerm::LevelIn);
			if (Plan.bLevelInAtRest)
			{
				VTarget = 0.0;
				By = EExecTerm::LevelIn;
			}
			else if (Plan.bHoldSpeedThroughLevelIn)
			{
				VTarget = VCmd;
				By = EExecTerm::LevelIn;
			}
		}
		// The plan's speed where the robot is (no nearer its origin than its first
		// sample past it).
		const double PlanHereV = PlanSpeedAt(S);
		// The centre wheels lagging the plan slow the robot, continuously.
		if (bLevelInBegun && bLoweredDown)
		{
			double Lag = 0.0;
			for (int32 Side = 0; Side < 2; ++Side)
			{
				if (Centre[Side] != INDEX_NONE)
				{
					Lag = FMath::Max(Lag, FMath::Abs(Measure.Legs[Centre[Side]].XZ.X - Here.X[Side]));
				}
			}
			if (Lag > LagFreeCm)
			{
				Diag.Track = PlanHereV * FMath::Clamp(1.0 - (Lag - LagFreeCm) / LagSlowCm, LagSlowFraction, 1.0);
				Limit(Diag.Track, EExecTerm::Track);
			}
		}
		Limit(StopCap, EExecTerm::PlannedStop);
		Limit(ExternalCap, EExecTerm::External);
		// The runtime checks' caps: the base off its planned attitude, half the
		// plan's speed; the measured margin under the floor, or a clear wheel's
		// rim at the terrain, a stop (braked, for the margin, at half
		// Deceleration: braking shifts the robot's weight forward).
		if (bAttitudeSlow)
		{
			Limit(AttitudeSlowShare * PlanHereV, EExecTerm::Attitude);
			++Counts.AttitudeSlowTicks;
		}
		if (bMarginHold)
		{
			Limit(0.0, EExecTerm::Margin);
		}
		if (bSwingStop)
		{
			Limit(0.0, EExecTerm::Swing);
		}
		// The hard braking caps: a limit on top of the ramp. Within
		// EmergencyOverCmS of one the command is clamped to it; further over (a
		// cap that has fallen fast: a stall, a wheel late), it brakes at twice
		// Deceleration down to it.
		int32		 BrakingLeg = INDEX_NONE;
		const double Braking = BrakingCap(Measure, BrakingLeg);
		const double Over = VCmd - Braking;
		const float	 DecelNow = Over > EmergencyOverCmS ? 2.0f * T.Deceleration
														: (By == EExecTerm::Margin ? static_cast<float>(MarginHoldDecelShare) * T.Deceleration : T.Deceleration);
		VCmd = RampSpeed(static_cast<float>(VCmd), static_cast<float>(FMath::Max(0.0, FMath::Min(VTarget, Braking))), T.Acceleration, DecelNow,
			static_cast<float>(Dt));
		if (VCmd > Braking && VCmd - Braking <= EmergencyOverCmS)
		{
			VCmd = Braking;
		}
		if (Over > 0.0)
		{
			++Counts.BrakingTicks;
		}
		VCmd = FMath::Max(0.0, VCmd);
		Out.DriveSpeed = VCmd;
		Diag.Braking = Braking;
		Diag.BrakingLeg = BrakingLeg;
		Diag.Target = VTarget;
		Diag.TargetBy = By;
		if (Wheels.IsValidIndex(DeadlineLeg) && By == EExecTerm::Deadline && Deadline < Diag.Plan - 0.05)
		{
			Counts.DeadlineLegs |= 1u << DeadlineLeg;
		}
		Diag.Command = VCmd;
		// What holds the robot under the plan's speed (or to a stop): the braking
		// cap where it binds, else the term that set the target.
		auto LegNamed = [&](int32 L) {
			return Wheels.IsValidIndex(L) ? FString::Printf(TEXT("'%s' (%s%s%s)"), *In.Legs[L].Name.ToString(), WheelStateName(Wheels[L].State),
												Wheels[L].bBlocked ? TEXT(", ") : TEXT(""), *Wheels[L].BlockedWhy)
										  : FString(TEXT("?"));
		};
		const bool bBrakingBinds = Braking <= FMath::Min(VTarget, VCmd + 0.5);
		auto	   Binding = [&]() {
			  if (bBrakingBinds)
			  {
				  return TEXT("the braking cap for ") + LegNamed(BrakingLeg);
			  }
			  if (By == EExecTerm::Deadline)
			  {
				  return TEXT("the deadline of ") + LegNamed(DeadlineLeg);
			  }
			  return FString(ExecTermName(By));
		};
		// Under the plan's speed by more than 2 cm/s: what holds it there, logged
		// when that changes (at most every 0.25 s).
		if (Phase == EExecPhase::Cross && VCmd < PlanHereV - 2.0 && Elapsed - SlowAt >= 0.25)
		{
			const FString Why = bBrakingBinds || By != EExecTerm::Plan ? Binding() : FString(TEXT("the ramp"));
			if (Why != SlowBy)
			{
				SlowBy = Why;
				SlowAt = Elapsed;
				Log(FString::Printf(TEXT("under the plan's %.1f cm/s: held by %s"), PlanHereV, *Why));
			}
		}
		Diag.ChassisOffCm = Level.bLevelled ? Measure.OriginZ - BCmdLag - BOffset : 0.0;
		{
			const FScheduleSample* Sm = Plan.SampleAt(S);
			Diag.PlanMarginCm = Sm ? Sm->Margin : 0.0;
			Diag.LiveMarginCm = LiveMarginCm;
			Diag.PlanX = FVector2D(Here.X[0], Here.X[1]);
			Diag.LiveX = FVector2D(Centre[0] != INDEX_NONE ? Measure.Legs[Centre[0]].XZ.X : 0.0, Centre[1] != INDEX_NONE ? Measure.Legs[Centre[1]].XZ.X : 0.0);
		}

		// A hold: the robot braked to a stop to wait -- the command down at the
		// hold speed, and what brought it there a wait the plan does not make: a
		// hard braking cap, a wheel's deadline (a blocked or late lift), the
		// centre wheels lagging, the far side unconfirmed. Not the plan's own
		// speed (its rise from rest, its braking into a planned stop), a planned
		// stop, or the level-in the plan has it make. Over once no such wait
		// holds the target or the braking caps under HoldSpeedCmS + 1.
		const bool bWaitTerm = By == EExecTerm::Deadline || By == EExecTerm::Track || By == EExecTerm::External || By == EExecTerm::Margin
			|| By == EExecTerm::Swing;
		auto Waits = [&](double Under) { return Braking < Under || (VTarget < Under && bWaitTerm); };
		if (!bHolding && VCmd <= HoldSpeedCmS && Waits(HoldSpeedCmS) && !bDwelling && bLevelInBegun && Phase != EExecPhase::Settle)
		{
			bHolding = true;
			++Counts.Holds;
			Log(FString::Printf(TEXT("hold: braked to wait for %s"), *Binding()));
			Phase = EExecPhase::Hold;
		}
		else if (bHolding && !Waits(HoldSpeedCmS + 1.0))
		{
			bHolding = false;
			Log(FString::Printf(TEXT("hold over (now %s)"), *Binding()));
			Phase = Level.bLevelled ? EExecPhase::Cross : EExecPhase::Prepare;
		}

		// The invariants, counted (never planned).
		{
			uint32 Air = 0;
			for (int32 i = 0; i < N; ++i)
			{
				Air |= IsSupport(i) ? 0u : (1u << i);
				const FWheelPlan& P = Plan.Wheels[i];
				// Loaded: carrying, or lifting but stalled on the ground.
				const bool bLoaded = IsSupport(i) || IsLiftStalledOnGround(i, Measure);
				if (bLoaded && Wheels[i].Plane == 0 && Wheels[i].PastEdgeCm > P.DUp + 0.5 && Wheels[i].PastEdgeCm < P.DDown - 0.5)
				{
					++Counts.WindowTicks;
				}
			}
			if (IsForbiddenSet(In.Model, Air))
			{
				++Counts.ForbiddenTicks;
			}
		}

		// Done: past where the plan ends the crossing, every wheel on the far side
		// and every leg where it was sent.
		if (Phase == EExecPhase::Settle && S >= SEnd)
		{
			// A support leg pressed into its surface stops short of its command by
			// up to its preload.
			bool bSettled = true;
			for (int32 i = 0; i < N && bSettled; ++i)
			{
				bSettled = FMath::Abs(Measure.Legs[i].XZ.Y - Wheels[i].CmdZ) <= T.SettleTolerance + (IsSupport(i) ? M->Preload(i) : 0.0)
					&& (!M->IsCentre(i) || FMath::Abs(Measure.Legs[i].XZ.X - Wheels[i].CmdX) <= T.SettleTolerance);
			}
			if (bSettled)
			{
				Phase = EExecPhase::Done;
				Log(TEXT("done"));
			}
		}

		Status = ExecPhaseName(Phase);
		if (Phase == EExecPhase::Hold)
		{
			if (bMarginHold)
			{
				Status += FString::Printf(TEXT(": the robot measured %.2f cm inside its wheels (MinLiveStabilityMarginCm %.1f)"), LiveMarginCm,
					T.MinLiveStabilityMarginCm);
			}
			else if (bSwingStop && Wheels.IsValidIndex(SwingLeg))
			{
				Status += FString::Printf(TEXT(": '%s' (clear) %.2f cm over the terrain inside its window"), *In.Legs[SwingLeg].Name.ToString(), SwingGap);
			}
			else
			{
				for (int32 i = 0; i < N; ++i)
				{
					if (Wheels[i].bBlocked)
					{
						Status += FString::Printf(TEXT(": '%s' %s"), *In.Legs[i].Name.ToString(), *Wheels[i].BlockedWhy);
						break;
					}
				}
			}
		}
	}

	void FExecutor::TickHalted(const FExecMeasure& Measure, FLevelState& Level, FExecCommand& Out, double Dt, bool bWheelsMeasured)
	{
		const int32				N = Wheels.Num();
		const FRammsCurbTuning& T = In.Tuning;
		const double			LegSpeed = FMath::Max(1.0, static_cast<double>(T.LegSpeed));
		const double			LegAccel = FMath::Max(1.0, static_cast<double>(T.LegAcceleration));
		S = FVector2D::DotProduct(Measure.OriginXY - Plan.Origin, Plan.Heading);
		VHat = Measure.bSpeedValid ? FMath::Max(0.0, Measure.SpeedCmS) : VCmd;
		const FVector2D Bias = Level.GetBias();
		if (!bWheelsMeasured)
		{
			MeasureWheels(Measure, Bias, Dt);
		}
		Phase = EExecPhase::Halted;
		if (!bHaltSetUp)
		{
			// Once: where the legs go while the robot stops. Every leg keeps the
			// target it had; a wheel that found no ground goes back up clear; with
			// bPlantOnHold, every wheel off the ground over a surface -- not inside
			// its window -- is set down on it, where the robot then stands.
			bHaltSetUp = true;
			const FPlanAt Here = Between(S);
			for (int32 i = 0; i < N; ++i)
			{
				FExecWheel&		W = Wheels[i];
				const FExecLeg& L = Measure.Legs[i];
				const double	X = M->IsCentre(i) ? W.CmdX : L.XZ.X;
				// A landing is set down just to touch; one held where it met the
				// ground presses no further.
				W.HaltPressCm = FMath::Min(M->Preload(i), PlantProbeCm);
				if (W.bRelift)
				{
					W.TargetZ = ClampToReach(i, X, SwingTarget(i, Measure, Here, W.ZTop + SwingRaise));
					SetState(i, ERammsCurbWheelState::Clear, TEXT("no ground where it was to land: lifted clear again"));
				}
				else if (W.State == ERammsCurbWheelState::Loading)
				{
					// Down, taking its load: it presses its preload in from where its
					// leg stands, no further -- its target was reckoned on the plane
					// and its bias, and a surface higher than they have it would have
					// the leg jack the robot up on it.
					W.TargetZ = FMath::Max(W.TargetZ, ClampToReach(i, X, L.XZ.Y - M->Preload(i)));
				}
			}
			if (T.bPlantOnHold)
			{
				uint32 Over = 0;
				for (int32 i = 0; i < N; ++i)
				{
					const FExecWheel& W = Wheels[i];
					const FWheelPlan& P = Plan.Wheels[i];
					const bool		  bUp = W.State == ERammsCurbWheelState::Lifting || W.State == ERammsCurbWheelState::Clear
						|| (W.State == ERammsCurbWheelState::Held && !W.bLower);
					Over |= bUp && !W.bRelift && (W.PastEdgeCm < P.DUp || W.PastEdgeCm > P.DDown) ? 1u << i : 0u;
				}
				if (Over != 0)
				{
					const double Margin = MarginWithout(Measure, 0, Over);
					if (Margin >= T.MinLiveStabilityMarginCm)
					{
						for (int32 i = 0; i < N; ++i)
						{
							if ((Over >> i) & 1u)
							{
								FExecWheel& W = Wheels[i];
								W.Plane = W.PastEdgeCm > Plan.Wheels[i].DDown ? 1 : 0;
								const double X = M->IsCentre(i) ? W.CmdX : Measure.Legs[i].XZ.X;
								W.TargetZ = ClampToReach(i, X, PlantingTarget(i, Measure, 0.0, Bias, M->Preload(i)));
								// Set down to stand on: its whole preload in.
								W.HaltPressCm = M->Preload(i);
								SetState(i, ERammsCurbWheelState::Planting,
									FString::Printf(TEXT("set down on hold over the %s surface (the robot then %.2f cm inside its wheels)"),
										W.Plane == 1 ? TEXT("far") : TEXT("near"), Margin));
							}
						}
					}
					else
					{
						Log(FString::Printf(TEXT("not set down on hold: with them down the robot would stand %.2f cm inside its wheels "
												 "(MinLiveStabilityMarginCm %.1f)"),
							Margin, T.MinLiveStabilityMarginCm));
					}
				}
			}
		}
		for (int32 i = 0; i < N; ++i)
		{
			FExecWheel&		W = Wheels[i];
			const FExecLeg& L = Measure.Legs[i];
			const double	X = M->IsCentre(i) ? W.CmdX : L.XZ.X;
			// A wheel being set down keeps going to its target only until it meets
			// the ground: on its surface; its leg stopped above where the servo
			// lag model has it, whatever the height (the lip, something lying
			// there) -- but not in the first two servo lags after it was turned
			// back down from a lift, while it still lags its model; or pressing
			// the robot up (FPressWatch; ticked only on the halt's own ticks, not
			// the one the main loop has already watched). Or its landing faulted,
			// which is where it met something. From there it is held HaltPressCm
			// in: its target, reckoned on the plane and its bias, may lie well
			// under a surface higher than they have it, and a leg sent on down to
			// it would jack its corner of the robot up. Until then its target
			// follows the surface under it.
			if (W.State == ERammsCurbWheelState::Planting && !W.bRelift && !W.bHaltHeld)
			{
				const bool bLegStopped = W.MeasVz > -StallSpeedCmS;
				const bool bTurning = Elapsed - W.Since < 2.0 * T.ServoLagSeconds;
				const bool bStalled = !bTurning && bLegStopped && L.XZ.Y - L.LagZ >= StallCm && W.CmdZ < L.XZ.Y;
				const bool bPressing = !bWheelsMeasured && W.Press.Tick(L.WheelWorld.Z, L.XZ.Y, Dt);
				if (W.bLandFault || W.BottomCm <= TouchCm || bStalled || bPressing)
				{
					W.bHaltHeld = true;
					W.HaltHeldAtZ = L.XZ.Y;
					W.CmdZ = W.TargetZ = ClampToReach(i, X, FMath::Max(W.CmdZ, L.XZ.Y - W.HaltPressCm));
					W.CmdVz = 0.0;
					Log(FString::Printf(TEXT("%-12s held where it met the ground (%s), its bottom %+.2f cm, %.1f cm in"), *In.Legs[i].Name.ToString(),
						W.bLandFault				? TEXT("its landing faulted")
							: W.BottomCm <= TouchCm ? TEXT("on its surface")
							: bPressing				? TEXT("pressing on the ground")
													: TEXT("its leg stopped"),
						W.BottomCm, W.HaltPressCm));
				}
				else
				{
					// On its way down: to the surface under it as the chassis now
					// stands (it may settle as it brakes), HaltPressCm in -- as the
					// landing's target follows it while crossing.
					W.TargetZ = ClampToReach(i, X, PlantingTarget(i, Measure, 0.0, Bias, W.HaltPressCm));
				}
			}
			TrackTrapezoid(W.CmdZ, W.CmdVz, W.TargetZ, LegSpeed, LegAccel, Dt);
			W.CmdZ = ClampToReach(i, X, W.CmdZ);
			Out.LegZ[i] = W.CmdZ;
			Out.LegX[i] = W.CmdX;
		}
		// Every leg on its target: its command there, and the leg where it was
		// sent -- or short of it by up to its preload on the ground, or stopped
		// short (a leg stalled against something gets no further). A wheel still
		// being set down is there only once its leg has stopped (at its target
		// in the air; one that meets the ground on the way is held there first,
		// above).
		bLegsHome = true;
		for (int32 i = 0; i < N && bLegsHome; ++i)
		{
			const FExecWheel& W = Wheels[i];
			const FExecLeg&	  L = Measure.Legs[i];
			const bool		  bStill = FMath::Abs(W.MeasVz) < StallSpeedCmS;
			const bool		  bLanding = W.State == ERammsCurbWheelState::Planting && !W.bHaltHeld;
			const double	  Short = IsSupport(i) || W.bHaltHeld ? M->Preload(i) : 0.0;
			const bool		  bThere = bLanding ? bStill && FMath::Abs(L.XZ.Y - W.CmdZ) <= T.SettleTolerance
												: FMath::Abs(L.XZ.Y - W.CmdZ) <= T.SettleTolerance + Short || bStill;
			bLegsHome = FMath::Abs(W.CmdZ - W.TargetZ) <= ExecCaughtUpCm && bThere
				&& (!M->IsCentre(i) || FMath::Abs(L.XZ.X - W.CmdX) <= T.SettleTolerance);
		}
		// Braked to rest at HaltDecel, never over the hard braking caps (twice
		// Deceleration down to one the command is over by more than
		// EmergencyOverCmS) -- and, halted (the terrain perhaps not as planned),
		// no wheel on the near side short of its window goes into it, lifted or
		// not: each braked short of its window.
		int32		 BrakingLeg = INDEX_NONE;
		double		 Braking = BrakingCap(Measure, BrakingLeg);
		const double Cos = FMath::Max(0.1, M->NormalShare());
		for (int32 i = 0; i < N; ++i)
		{
			const FExecWheel& W = Wheels[i];
			const FWheelPlan& P = Plan.Wheels[i];
			if (W.Plane != 0 || W.PastEdgeCm >= P.DUp)
			{
				continue;
			}
			const double Cap = BrakingSpeed(LagBrakingDistance(static_cast<float>((P.DUp - W.PastEdgeCm) / Cos), static_cast<float>(VHat), DriveLagSeconds),
				T.Deceleration);
			if (Cap < Braking)
			{
				Braking = Cap;
				BrakingLeg = i;
			}
		}
		// (Over a cap, at twice Deceleration -- not clamped to it: a halt never
		// brakes harder than that.)
		const double Decel = VCmd > Braking ? FMath::Max(HaltDecel, 2.0 * T.Deceleration) : HaltDecel;
		VCmd = FMath::Max(0.0, VCmd - Decel * Dt);
		Out.DriveSpeed = VCmd;
		Diag.Braking = Braking;
		Diag.BrakingLeg = BrakingLeg;
		Diag.Target = 0.0;
		Diag.Command = VCmd;
		LiveMarginCm = MarginWithout(Measure, 0);
		Diag.LiveMarginCm = LiveMarginCm;
		// Stopped once at rest with every leg on its target: a halt that ended
		// as the chassis came to rest would leave a leg still on its way -- a
		// wheel being set down on hold in the air -- where it then was.
		const bool bAtRest = VCmd <= 1e-6 && (!Measure.bSpeedValid || FMath::Abs(Measure.SpeedCmS) < StopSpeedCmS);
		if (!bStopped && AtRest.Tick(bAtRest && bLegsHome, Dt, MinStopSeconds))
		{
			bStopped = true;
			Log(FString::Printf(TEXT("halted at rest (%.1f cm/s measured), every leg on its target"), Measure.bSpeedValid ? Measure.SpeedCmS : 0.0));
		}
	}

	ERunaway RunawayStep(double& FollowedCmS, double& Seconds, double CommandCmS, double MeasuredCmS, bool bAheadArmed, bool bBackArmed, double Dt)
	{
		const double Step = FMath::Max(0.0, Dt);
		FollowedCmS += (CommandCmS - FollowedCmS) * FMath::Clamp(Step / RunawayLagSeconds, 0.0, 1.0);
		const bool bAhead = bAheadArmed && MeasuredCmS - FollowedCmS > RunawayCmS;
		const bool bBack = bBackArmed && MeasuredCmS < -RollBackCmS;
		Seconds = bAhead || bBack ? Seconds + Step : 0.0;
		if (Seconds > RunawaySeconds)
		{
			return bBack ? ERunaway::Back : ERunaway::Ahead;
		}
		return ERunaway::None;
	}

	// --- the closed-loop dry run ------------------------------------------------------------

	FExecDryRunResult ExecDryRun(const FPlannerInput& In, const FCurbSchedule& Plan, const FExecDryRunOptions& Options)
	{
		FExecDryRunResult Out;
		if (!Plan.IsValid() || Plan.Wheels.Num() != In.Legs.Num())
		{
			Out.Notes.Add(TEXT("no plan to run"));
			return Out;
		}
		// The world as it is: the plan's terrain, each surface offset as asked,
		// and a ridge on it. The robot stands and rolls on this one (MT); the
		// executor plans and acts on the plan's.
		FPlannerInput InTrue = In;
		InTrue.Terrain.Near.Origin.Z += Options.TrueOffsetCm.X;
		InTrue.Terrain.Far.Origin.Z += Options.TrueOffsetCm.Y;
		InTrue.Terrain.EdgePoint.Z += Options.TrueOffsetCm.X;
		InTrue.Terrain.StepHeightCm += static_cast<float>(Options.TrueOffsetCm.Y - Options.TrueOffsetCm.X);
		const FRammsCurbTerrain& TrueT = InTrue.Terrain;
		const FPlanModel		 M(InTrue);
		const FRammsCurbTuning&	 T = In.Tuning;
		const int32				 N = In.Legs.Num();
		const double			 Dt = FMath::Max(1e-4, Options.Dt);
		const double			 Tau = FMath::Max(1e-3, T.ServoLagSeconds * Options.LagScale);
		const double			 Floor = FMath::Clamp(static_cast<double>(T.MinSpeedFraction), 0.0, 1.0) * T.DriveSpeed;
		auto					 Note = [&](const FString& S) {
			if (Out.Notes.Num() < 12)
			{
				Out.Notes.Add(S);
			}
		};
		// The ridge across the far surface: its height where it is, and how much
		// it lifts a wheel whose centre is over XY (its rim meeting the ridge's
		// near or far side, or its top under the wheel's bottom).
		const bool bRidge = Options.RidgeHeightCm != 0.0 && Options.RidgeWidthCm > 0.0;
		auto	   RidgeAt = [&](const FVector2D& XY) {
			  const double D = TrueT.DistancePastEdge(XY);
			  return bRidge && D >= Options.RidgePastEdgeCm && D <= Options.RidgePastEdgeCm + Options.RidgeWidthCm ? Options.RidgeHeightCm : 0.0;
		};
		auto RidgeLift = [&](int32 i, const FVector2D& XY) {
			if (!bRidge)
			{
				return 0.0;
			}
			const double R = M.Radius(i);
			const double C = FMath::Max(0.1, FMath::Abs(FVector2D::DotProduct(M.RollDirection(i), TrueT.EdgeNormal)));
			const double D = TrueT.DistancePastEdge(XY);
			const double ULo = (Options.RidgePastEdgeCm - D) / C;
			const double UHi = (Options.RidgePastEdgeCm + Options.RidgeWidthCm - D) / C;
			const double U = FMath::Clamp(0.0, FMath::Min(ULo, UHi), FMath::Max(ULo, UHi));
			if (FMath::Abs(U) >= R)
			{
				return 0.0;
			}
			return FMath::Max(0.0, Options.RidgeHeightCm - (R - FMath::Sqrt(R * R - U * U)));
		};
		auto SurfaceTrue = [&](const FVector2D& XY) {
			return (TrueT.DistancePastEdge(XY) >= 0.0 ? TrueT.Far : TrueT.Near).HeightAt(XY) + RidgeAt(XY);
		};

		// The robot: each leg's height and a 5-bar's fore/aft as they follow their
		// commands, the chassis' travel, speed, height and attitude.
		TArray<FVector2D> Leg; // x, z (chassis frame)
		TArray<double>	  LagZ;
		TArray<double>	  Shown; // the leg height measured: held up by the ground
		Leg.SetNum(N);
		LagZ.SetNum(N);
		Shown.SetNum(N);
		for (int32 i = 0; i < N; ++i)
		{
			Leg[i] = In.Legs[i].Measured;
			LagZ[i] = Leg[i].Y;
			Shown[i] = Leg[i].Y;
		}
		double			Travel = Options.StartTravelCm, Speed = FMath::Max(0.0, In.EntrySpeedCmS), Time = 0.0;
		double			OriginZ = In.OriginZ + Options.TrueOffsetCm.X;
		FVector2D		Attitude = In.AttitudeDeg;
		const FVector2D H = In.Heading.GetSafeNormal();
		auto			FrameOf = [&](double Z, const FVector2D& Att) {
			   return ChassisFrameFrom(FVector(H.X, H.Y, 0.0), FMath::DegreesToRadians(Att.X), FMath::DegreesToRadians(Att.Y),
						   FVector(In.Origin.X + H.X * Travel, In.Origin.Y + H.Y * Travel, Z), FVector::ZeroVector);
		};
		auto WheelAt = [&](const FChassisFrame& F, int32 i, double Z) {
			const double X = M.IsCentre(i) ? Leg[i].X : M.CornerX(i, Z);
			return F.ToWorld(FVector(X, M.LegY(i), Z));
		};
		auto		 WheelWorld = [&](const FChassisFrame& F, int32 i) { return WheelAt(F, i, Leg[i].Y); };
		FExecutor	 E;
		FLevelState	 Level;
		FExecMeasure Meas;
		auto		 MeasureNow = [&]() {
			Meas.OriginXY = In.Origin + H * Travel;
			Meas.OriginZ = OriginZ;
			Meas.Forward = H;
			Meas.Frame = FrameOf(OriginZ, Attitude);
			// A kick: the attitude measured off from what the base does.
			Meas.AttitudeDeg = Attitude + (Options.KickAtSeconds >= 0.0 && Time >= Options.KickAtSeconds ? Options.KickDeg : FVector2D::ZeroVector);
			Meas.SpeedCmS = Speed;
			Meas.bSpeedValid = true;
			Meas.Legs.SetNum(N);
			Meas.Probes.Reset();
			for (int32 i = 0; i < N; ++i)
			{
				FExecLeg& L = Meas.Legs[i];
				L.WheelWorld = WheelAt(Meas.Frame, i, Shown[i]);
				L.XZ = FVector2D(M.IsCentre(i) ? Leg[i].X : M.CornerX(i, Shown[i]), Shown[i]);
				L.LagZ = LagZ[i];
				L.bValid = true;
				// The terrain as it is, traced along the heading from the wheel's
				// centre.
				for (int32 k = 0; Options.bProbes && k < NumProbes; ++k)
				{
					if (!Options.bProbeAhead && FMath::Abs(ProbeAheadCm[k]) > 1e-6)
					{
						continue;
					}
					FTerrainProbe& P = Meas.Probes.AddDefaulted_GetRef();
					P.Leg = i;
					P.AheadCm = ProbeAheadCm[k];
					P.XY = FVector2D(L.WheelWorld.X, L.WheelWorld.Y) + H * ProbeAheadCm[k];
					P.Z = SurfaceTrue(P.XY);
					P.bHit = true;
					P.AboveZ = L.WheelWorld.Z + 40.0;
					P.BelowZ = L.WheelWorld.Z - 60.0;
				}
			}
		};
		MeasureNow();
		FString Why;
		if (!E.Begin(In, Plan, Meas, Why))
		{
			Out.Notes.Add(Why);
			return Out;
		}
		bool   bMoved = Speed >= 5.0, bRisen = Speed >= Floor, bRising = !bRisen;
		double Peak = Speed, StopFor = 0.0;
		Out.MinSpeedCmS = TNumericLimits<double>::Max();
		TArray<double> StallFrom;
		Out.MaxNearCarryPastEdgeCm.Init(-TNumericLimits<double>::Max(), N);
		// The base's attitude over the last 0.3 s (time, pitch, roll) while levelling in.
		TArray<FVector> TurnHistory;
		uint32			Carry = 0;
		StallFrom.Init(TNumericLimits<double>::Max(), N);
		FExecCommand									  Cmd;
		TArray<ERammsCurbWheelState, TInlineAllocator<8>> Was;
		for (int32 i = 0; i < N; ++i)
		{
			Was.Add(ERammsCurbWheelState::Planted);
		}
		double HaltedFrom = -1.0, LastCmd = Speed, StoppedFor = 0.0;
		// A re-plan made, waiting ReplanDelaySeconds to be adopted (as one made off
		// the game thread does); after its adoption, until when its re-fitted
		// surfaces' biases are watched against where those truly lie.
		bool				  bPending = false;
		double				  PendingAt = 0.0, WatchBiasUntil = -1.0;
		FPlannerInput		  PendingIn;
		FCurbSchedule		  PendingPlan;
		FExecutor::FReplanAsk PendingAsk;
		FVector2D			  WatchRefit = FVector2D::ZeroVector;
		while (Time < Options.MaxSeconds)
		{
			Meas.Dt = Dt;
			if (((Options.HaltAtSeconds >= 0.0 && Time >= Options.HaltAtSeconds) || E.GetS() >= Options.HaltAtTravelCm) && !E.IsHalted())
			{
				E.Halt(TEXT("aborted (dry run)"));
			}
			// Halted and stopped, the controller ends the executor (the wheels
			// braked): from there the legs hold their last commands.
			if (E.IsHalted() && E.IsStopped())
			{
				Cmd.DriveSpeed = 0.0;
			}
			else
			{
				E.Tick(Meas, Level, Cmd);
			}
			Time += Dt;
			if (E.IsHalted() && HaltedFrom < 0.0)
			{
				HaltedFrom = Time;
				Out.bHalted = true;
				Out.bFaulted = E.HasFault();
				Out.FaultWhy = E.GetFaultWhy();
				Out.FaultAt = Time;
				if (Out.bFaulted)
				{
					Note(FString::Printf(TEXT("%.2f s: fault: %s"), Time, *Out.FaultWhy));
				}
			}
			if (HaltedFrom >= 0.0)
			{
				Out.HaltBrakingCmS2 = FMath::Max(Out.HaltBrakingCmS2, (LastCmd - Cmd.DriveSpeed) / Dt);
			}
			LastCmd = Cmd.DriveSpeed;
			// A re-plan the executor asks for before the crossing has begun, made
			// as the controller makes it: from the robot as it stands, on the
			// plan's terrain re-fitted by what the traces found.
			FExecutor::FReplanAsk Ask;
			if (Options.bReplan && !bPending && E.TakeReplanAsk(Ask))
			{
				FPlannerInput InNow = E.GetInput();
				InNow.Terrain.Near.Origin.Z += Ask.RefitCm.X;
				InNow.Terrain.Far.Origin.Z += Ask.RefitCm.Y;
				InNow.Terrain.EdgePoint.Z += Ask.RefitCm.X;
				InNow.Terrain.StepHeightCm += static_cast<float>(Ask.RefitCm.Y - Ask.RefitCm.X);
				InNow.Origin = In.Origin + H * Travel;
				InNow.OriginZ = OriginZ;
				InNow.AttitudeDeg = Attitude;
				InNow.EntrySpeedCmS = Speed;
				for (int32 i = 0; i < N; ++i)
				{
					InNow.Legs[i].Measured = Meas.Legs[i].XZ;
					InNow.Legs[i].bOffGround = E.GetWheels()[i].State == ERammsCurbWheelState::Held;
				}
				PendingPlan = PlanCrossing(InNow);
				PendingIn = MoveTemp(InNow);
				PendingAsk = Ask;
				PendingAt = Time;
				bPending = true;
			}
			if (bPending && Time >= PendingAt + Options.ReplanDelaySeconds - 1e-9)
			{
				bPending = false;
				FString Refused;
				if (PendingPlan.IsValid() && E.Adopt(PendingIn, PendingPlan, Refused))
				{
					Level.Refitted(PendingAsk.RefitCm);
					++Out.Replans;
					Out.ReplanWhy.Add(PendingAsk.Why);
					WatchRefit = PendingAsk.RefitCm;
					WatchBiasUntil = Time + 0.5;
				}
				else
				{
					Note(FString::Printf(TEXT("%.2f s: a re-plan (%s) not adopted: %s"), Time, *PendingAsk.Why,
						PendingPlan.IsValid() ? *Refused : *PendingPlan.Summary));
				}
			}
			// The re-fitted surfaces' biases, from the re-plan's adoption on: each
			// against where its surface truly lies off its new plane.
			if (Time <= WatchBiasUntil)
			{
				const FRammsCurbTerrain& Now = E.GetInput().Terrain;
				if (!FMath::IsNearlyZero(WatchRefit.X, 1e-3))
				{
					Out.ReplanBiasErrorCm = FMath::Max(Out.ReplanBiasErrorCm, FMath::Abs(Level.NearBias.Get() - (TrueT.Near.Origin.Z - Now.Near.Origin.Z)));
				}
				if (!FMath::IsNearlyZero(WatchRefit.Y, 1e-3))
				{
					Out.ReplanBiasErrorCm = FMath::Max(Out.ReplanBiasErrorCm, FMath::Abs(Level.FarBias.Get() - (TrueT.Far.Origin.Z - Now.Far.Origin.Z)));
				}
			}
			for (int32 i = 0; i < N; ++i)
			{
				const ERammsCurbWheelState St = E.GetWheels()[i].State;
				if (St == ERammsCurbWheelState::Lifting && Was[i] == ERammsCurbWheelState::Planted)
				{
					for (int32 j = 0; j < N; ++j)
					{
						if (((Plan.Wheels[i].MustBeDownMask >> j) & 1u) && E.GetWheels()[j].State != ERammsCurbWheelState::Planted)
						{
							++Out.EarlyLifts;
							Note(FString::Printf(TEXT("%.2f s: '%s' lifted with '%s' %s"), Time, *In.Legs[i].Name.ToString(), *In.Legs[j].Name.ToString(),
								WheelStateName(E.GetWheels()[j].State)));
						}
					}
				}
				Was[i] = St;
			}
			// The legs follow, a lag behind; the controller's lag model the
			// nominal lag.
			for (int32 i = 0; i < N; ++i)
			{
				const ERammsCurbWheelState St = E.GetWheels()[i].State;
				const bool				   bSlow = ((Options.SlowPlantLegs >> i) & 1u) && E.GetWheels()[i].Plane == 1
					&& (St == ERammsCurbWheelState::Planting || St == ERammsCurbWheelState::Loading);
				const double LegTau = bSlow ? Tau * FMath::Max(1.0, Options.SlowPlantScale) : Tau;
				double		 Z = Leg[i].Y + (Cmd.LegZ[i] - Leg[i].Y) * FMath::Min(1.0, Dt / LegTau);
				if (i == Options.StallLeg)
				{
					if (StallFrom[i] == TNumericLimits<double>::Max() && E.GetWheels()[i].State == ERammsCurbWheelState::Lifting)
					{
						StallFrom[i] = Leg[i].Y;
					}
					if (StallFrom[i] < TNumericLimits<double>::Max())
					{
						Z = FMath::Min(Z, StallFrom[i] + Options.StallAtCm);
					}
				}
				Leg[i].Y = Z;
				if (M.IsCentre(i))
				{
					Leg[i].X += (Cmd.LegX[i] - Leg[i].X) * FMath::Min(1.0, Dt / Tau);
				}
				LagZ[i] += (Cmd.LegZ[i] - LagZ[i]) * FMath::Min(1.0, Dt / FMath::Max(1e-3, static_cast<double>(T.ServoLagSeconds)));
			}
			Speed += (Cmd.DriveSpeed - Speed) * FMath::Min(1.0, Dt / FMath::Max(1e-3, Options.DriveLag));
			Travel += Speed * Dt;
			// The chassis on its support legs: a rigid base fitted by least squares
			// to where each support leg puts it (B + x tan(pitch) + y tan(roll) =
			// the contact's centre height less the leg's height), on the terrain as
			// it is (a ridge under a wheel holds it up).
			{
				// Every wheel the executor has carrying, and any other whose bottom is
				// on (or in) its surface -- a wheel being lifted still carries until it
				// leaves the ground, one being set down from when it touches.
				const FChassisFrame F0 = FrameOf(OriginZ, Attitude);
				auto				NeedAt = [&](int32 i, const FVector& W, const FVector2D& Att) {
					   const FVector2D XY(W.X, W.Y);
					   const bool	   bFar = E.IsSupport(i) ? E.GetWheels()[i].Plane == 1 : TrueT.DistancePastEdge(XY) >= 0.0;
					   return M.ContactCentreZ(i, XY, bFar, Att.X, Att.Y) + RidgeLift(i, XY);
				};
				Carry = 0;
				for (int32 i = 0; i < N; ++i)
				{
					const FVector W = WheelWorld(F0, i);
					Carry |= (E.IsSupport(i) || W.Z <= NeedAt(i, W, Attitude) + 1e-3 ? 1u : 0u) << i;
				}
				double Sx = 0, Sy = 0, Sxx = 0, Syy = 0, Sxy = 0, Sr = 0, Sxr = 0, Syr = 0;
				int32  n = 0;
				double SagCm = 0.0;
				for (int32 i = 0; i < N; ++i)
				{
					if (!((Carry >> i) & 1u))
					{
						continue;
					}
					SagCm += M.Preload(i);
					const FVector W = WheelWorld(F0, i);
					const double  Need = NeedAt(i, W, Attitude);
					const double  X = M.IsCentre(i) ? Leg[i].X : M.CornerX(i, Leg[i].Y);
					const double  Y = M.LegY(i);
					const double  R = Need - Leg[i].Y;
					Sx += X;
					Sy += Y;
					Sxx += X * X;
					Syy += Y * Y;
					Sxy += X * Y;
					Sr += R;
					Sxr += X * R;
					Syr += Y * R;
					++n;
				}
				SagCm = n > 0 ? SagCm / n : 0.0;
				if (n >= 3)
				{
					const double A11 = n, A12 = Sx, A13 = Sy, A22 = Sxx, A23 = Sxy, A33 = Syy;
					const double Det = A11 * (A22 * A33 - A23 * A23) - A12 * (A12 * A33 - A23 * A13) + A13 * (A12 * A23 - A22 * A13);
					if (FMath::Abs(Det) > 1e-9)
					{
						const double DetA = Sr * (A22 * A33 - A23 * A23) - A12 * (Sxr * A33 - A23 * Syr) + A13 * (Sxr * A23 - A22 * Syr);
						const double DetB = A11 * (Sxr * A33 - A23 * Syr) - Sr * (A12 * A33 - A23 * A13) + A13 * (A12 * Syr - Sxr * A13);
						const double DetC = A11 * (A22 * Syr - Sxr * A23) - A12 * (A12 * Syr - Sxr * A13) + Sr * (A12 * A23 - A22 * A13);
						// R = B + x sin(pitch) + y sin(roll): the frame's X and Y rise
						// with pitch (nose up) and roll (+Y up). The legs' servos sag
						// under the robot's weight, about as much as their mean preload
						// (a rigid fit alone would have the chassis ride that high).
						OriginZ = DetA / Det - SagCm;
						Attitude.X = FMath::RadiansToDegrees(FMath::Asin(FMath::Clamp(DetB / Det, -1.0, 1.0)));
						Attitude.Y = FMath::RadiansToDegrees(FMath::Asin(FMath::Clamp(DetC / Det, -1.0, 1.0)));
					}
				}
				// A carrying leg its servo presses into its surface is held on it: the
				// height measured is shorter than the servo's, its wheel on the
				// surface (as a real leg's encoder would read).
				const FChassisFrame F1 = FrameOf(OriginZ, Attitude);
				for (int32 i = 0; i < N; ++i)
				{
					Shown[i] = Leg[i].Y;
					if (!((Carry >> i) & 1u) || F1.Z.Z <= 0.1)
					{
						continue;
					}
					const FVector W = WheelWorld(F1, i);
					const double  Need = NeedAt(i, W, Attitude);
					if (W.Z < Need)
					{
						Shown[i] = Leg[i].Y + (Need - W.Z) / F1.Z.Z;
					}
				}
			}
			const double Incl = InclinationDeg(Attitude.X, Attitude.Y);
			Out.MaxInclinationDeg = FMath::Max(Out.MaxInclinationDeg, Incl);
			if (HaltedFrom >= 0.0)
			{
				Out.HaltedInclinationDeg = FMath::Max(Out.HaltedInclinationDeg, Incl);
			}
			// The base against the plan once levelled in (from the drive stance it
			// turns to the plan's attitude: MaxLevelInTurnDegS).
			if (E.GetPhase() != EExecPhase::Approach && E.GetPhase() != EExecPhase::Prepare && E.GetPhase() != EExecPhase::Halted)
			{
				Out.MaxTiltDeg = FMath::Max(Out.MaxTiltDeg, FMath::Max(FMath::Abs(Attitude.X - E.GetPlannedAttitude().X), FMath::Abs(Attitude.Y - E.GetPlannedAttitude().Y)));
			}
			// How fast the base turns while levelling in, over 0.3 s.
			if (E.GetPhase() == EExecPhase::Prepare)
			{
				TurnHistory.Add(FVector(Time, Attitude.X, Attitude.Y));
				while (TurnHistory.Num() > 1 && Time - TurnHistory[0].X > 0.3 + 1e-9)
				{
					TurnHistory.RemoveAt(0);
				}
				const double Span = Time - TurnHistory[0].X;
				if (Span >= 0.3 - 0.5 * Dt)
				{
					const double Turn = FVector2D(Attitude.X - TurnHistory[0].Y, Attitude.Y - TurnHistory[0].Z).Size();
					Out.MaxLevelInTurnDegS = FMath::Max(Out.MaxLevelInTurnDegS, Turn / Span);
				}
			}
			else
			{
				TurnHistory.Reset();
			}
			MeasureNow();
			// The checks, against the terrain as it is.
			for (int32 i = 0; i < N; ++i)
			{
				const FWheelPlan& P = Plan.Wheels[i];
				const FVector	  W = Meas.Legs[i].WheelWorld;
				const double	  D = TrueT.DistancePastEdge(FVector2D(W.X, W.Y));
				// On the ground on the near side: carrying, as the executor has it, or a
				// wheel it is lifting that still carries load.
				const bool bLoaded = E.GetWheels()[i].Plane == 0
					&& (E.IsSupport(i) || (E.GetWheels()[i].State == ERammsCurbWheelState::Lifting && ((Carry >> i) & 1u)));
				if (bLoaded)
				{
					Out.MaxNearCarryPastEdgeCm[i] = FMath::Max(Out.MaxNearCarryPastEdgeCm[i], D);
				}
				if (bLoaded && D > P.DUp + 1e-3 && D < P.DDown - 1e-3)
				{
					++Out.KeepOffViolations;
					Note(FString::Printf(TEXT("%.2f s: '%s' on the ground %.1f cm past the edge, inside %.1f..%.1f"), Time, *In.Legs[i].Name.ToString(), D,
						P.DUp, P.DDown));
				}
				if (!E.IsSupport(i) && D >= P.DUp && D <= P.DDown)
				{
					// Its rim against the terrain (no dilation, no clearance).
					const double Rim = FMath::Max(ClearanceHeight(TrueT, FVector2D(W.X, W.Y), M.RollDirection(i), M.Radius(i), M.TyreWidth(i), 0.0),
						D > 0.0 && bRidge ? M.ContactCentreZ(i, FVector2D(W.X, W.Y), true) + RidgeLift(i, FVector2D(W.X, W.Y)) : -TNumericLimits<double>::Max());
					const double Gap = W.Z - Rim;
					Out.MinSwingClearanceCm = FMath::Min(Out.MinSwingClearanceCm, Gap);
					if (Gap < 0.0)
					{
						++Out.IntoTerrain;
						Note(FString::Printf(TEXT("%.2f s: '%s' %.2f cm into the terrain, %.1f cm past the edge"), Time, *In.Legs[i].Name.ToString(), -Gap, D));
					}
				}
			}
			{
				TArray<FVector2D, TInlineAllocator<MaxPlanLegs>>	  XZ;
				TArray<FMeasuredWheel, TInlineAllocator<MaxPlanLegs>> Ws;
				for (int32 i = 0; i < N; ++i)
				{
					XZ.Add(Meas.Legs[i].XZ);
					FMeasuredWheel& W = Ws.AddDefaulted_GetRef();
					W.Centre = Meas.Legs[i].WheelWorld;
					W.bSupport = E.IsSupport(i);
					W.Gradient = (E.GetWheels()[i].Plane == 1 ? TrueT.Far : TrueT.Near).Gradient;
				}
				const FPoseMargin R = MeasuredMargin(In.Model, Meas.Frame, XZ, Ws, H, T.Acceleration, T.Deceleration);
				if (R.Support >= 3)
				{
					Out.MinMarginCm = FMath::Min(Out.MinMarginCm, R.DynamicCm);
				}
			}
			bMoved = bMoved || Speed >= 5.0;
			if (bRising)
			{
				Peak = FMath::Max(Peak, Speed);
				if (Speed >= Floor || Speed < Peak - 2.0)
				{
					bRising = false;
					bRisen = true;
				}
			}
			const bool bStopAhead = E.GetCounts().PlannedStops > 0 || Plan.PlannedStops > 0;
			if (bRisen && !bStopAhead && HaltedFrom < 0.0)
			{
				Out.MinSpeedCmS = FMath::Min(Out.MinSpeedCmS, Cmd.DriveSpeed);
			}
			if (bMoved && !bStopAhead && HaltedFrom < 0.0)
			{
				StopFor = Speed <= 2.0 ? StopFor + Dt : 0.0;
				if (StopFor >= 0.2 && StopFor - Dt < 0.2)
				{
					++Out.Stops;
				}
			}
			if (Options.TraceEvery > 0 && (Out.Trace.Num() == 0 || FMath::RoundToInt(Time / Dt) % Options.TraceEvery == 0))
			{
				const FExecDiag& Dg = E.GetDiag();
				auto			 Cap = [](double V) { return V >= 1e6 ? FString(TEXT("-")) : FString::Printf(TEXT("%.1f"), V); };
				FString			 Line = FString::Printf(TEXT("t %5.2f s %6.1f v %5.1f cmd %5.1f [plan %s dead %s(%d) brake %s(%d) late %s track %s] %-7s att %+.2f/%+.2f off %+.2f m %.2f/%.2f x %+.1f/%+.1f (%+.1f/%+.1f) |"),
							 Time, E.GetS(), Speed, Cmd.DriveSpeed, *Cap(Dg.Plan), *Cap(Dg.Deadline), Dg.DeadlineLeg, *Cap(Dg.Braking), Dg.BrakingLeg,
							 *Cap(Dg.LateLevel), *Cap(Dg.Track), ExecPhaseName(E.GetPhase()), Attitude.X, Attitude.Y, Dg.ChassisOffCm, Dg.PlanMarginCm,
							 Dg.LiveMarginCm, Dg.LiveX.X, Dg.LiveX.Y, Dg.PlanX.X, Dg.PlanX.Y);
				for (int32 i = 0; i < N; ++i)
				{
					const FExecWheel&  W = E.GetWheels()[i];
					static const TCHAR Letters[] = TEXT("PLCDOH"); // planted, lifting, clear, down (planting), loading, held
					const int32		   St = static_cast<int32>(W.State);
					Line += FString::Printf(TEXT(" %c%+.1f/%+.1f/%+.1f"), St >= 0 && St < 6 ? Letters[St] : TEXT('?'), W.PastEdgeCm, W.BottomCm, Leg[i].Y - W.CmdZ);
				}
				Out.Trace.Add(Line);
			}
			if (E.GetPhase() == EExecPhase::Done)
			{
				Out.bDone = true;
				break;
			}
			if (HaltedFrom >= 0.0)
			{
				StoppedFor = E.IsStopped() ? StoppedFor + Dt : 0.0;
				if (StoppedFor >= Options.StoppedRunSeconds || Time - HaltedFrom > Options.HaltRunSeconds)
				{
					Out.bStopped = E.IsStopped();
					break;
				}
			}
		}
		Out.EndPhase = E.GetPhase();
		Out.Counts = E.GetCounts();
		Out.Seconds = Time;
		Out.FinalSpeedCmS = Speed;
		for (int32 i = 0; i < N; ++i)
		{
			const FVector	W = Meas.Legs[i].WheelWorld;
			const FVector2D XY(W.X, W.Y);
			Out.FinalPastEdgeCm.Add(TrueT.DistancePastEdge(XY));
			Out.FinalBottomCm.Add(W.Z - (M.ContactCentreZ(i, XY, TrueT.DistancePastEdge(XY) >= 0.0, Attitude.X, Attitude.Y) + RidgeLift(i, XY)));
			const FExecWheel& EW = E.GetWheels()[i];
			Out.HaltHeldPressCm.Add(EW.bHaltHeld ? EW.HaltHeldAtZ - EW.CmdZ : -1.0);
		}
		if (Out.MinSpeedCmS == TNumericLimits<double>::Max())
		{
			Out.MinSpeedCmS = 0.0;
		}
		TArray<FString> Lines;
		E.TakeLog(Lines);
		for (const FString& L : Lines)
		{
			if (L.Contains(TEXT("blocked")) || L.Contains(TEXT("hold")) || L.Contains(TEXT("before it was clear")) || L.Contains(TEXT("fault"))
				|| L.Contains(TEXT("re-plan")))
			{
				Note(L);
			}
		}
		Out.Log = MoveTemp(Lines);
		return Out;
	}
} // namespace RammsCurb
