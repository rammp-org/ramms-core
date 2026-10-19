// Tests for the curb controller's per-wheel planner (Private/Curb/RammsCurbPlanner),
// its kinematic dry run (Private/Curb/RammsCurbDryRun) and its executor
// (Private/Curb/RammsCurbExecutor), closed round a kinematic robot.
//
// Every number is made up -- the made-up six-wheeled robot of
// the terrain, level and stability tests: corner wheels on parallelogram 4-bars
// 45 cm fore and aft of the middle (at mid height) and +-25 cm across, a pair of
// centre wheels on 5-bars +-27 cm across whose reach is a made-up dome, 9 cm
// wheels, a made-up 100 kg with 6 / 7 kg moving with each corner / centre wheel
// -- on analytic planes: no robot's own.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Curb/RammsCurbDryRun.h"
#include "Curb/RammsCurbExecutor.h"
#include "Curb/RammsCurbPlanTask.h"
#include "Curb/RammsCurbPlanner.h"
#include "Curb/RammsCurbRobotModel.h"
#include "Curb/RammsCurbSchedule.h"
#include "Curb/RammsCurbTerrainFit.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	/** Leg order: front -Y, front +Y, centre -Y, centre +Y, rear -Y, rear +Y. */
	constexpr int32	 PlanFL = 0, PlanFR = 1, PlanCL = 2, PlanCR = 3, PlanRL = 4, PlanRR = 5;
	const TCHAR*	 PlanLegName[6] = { TEXT("front_left"), TEXT("front_right"), TEXT("left_center"), TEXT("right_center"),
			TEXT("rear_left"), TEXT("rear_right") };
	constexpr double PlanCornerY = 25.0;
	constexpr double PlanCentreY = 27.0;
	constexpr double PlanRadius = 9.0;
	/** The chassis origin's height over flat ground the robot stands at. */
	constexpr double PlanStandCm = 14.0;
	/** The near edge this far ahead of the chassis origin, along +X. */
	constexpr double PlanEdgeCm = 120.0;

	/** A made-up parallelogram 4-bar whose wheel swings on a 21 cm arm about
	 *  a rocker pivot 24 cm fore (Sign +1) or aft (-1) of the middle: the wheel
	 *  45 cm out at mid height, -15 .. +12 cm high over +-0.7 rad of crank. */
	FRamms4BarLegSpec PlanCornerLeg(double Sign)
	{
		FRamms4BarLegSpec Spec;
		Spec.CrankMotor = TEXT("test_crank");
		Spec.CrankPivot = FVector2D(24.0 * Sign, 6.0);
		Spec.CrankArm = FVector2D(15.0 * Sign, 0.0);
		Spec.CrankAngleSign = static_cast<float>(Sign);
		Spec.CouplerLength = 6.0f;
		Spec.RockerPivot = FVector2D(24.0 * Sign, 0.0);
		Spec.RockerPin = FVector2D(15.0 * Sign, 0.0);
		Spec.RockerAngleSign = static_cast<float>(Sign);
		Spec.WheelOnRocker = FVector2D(21.0 * Sign, -2.0);
		Spec.WheelOffset = FVector2D::ZeroVector;
		Spec.CrankRange = FVector2D(-0.7, 0.7);
		Spec.RockerRange = FVector2D(-1.2, 1.2);
		return Spec;
	}

	/** The made-up centre 5-bar's reach: a dome, 14 cm high and 15 cm low at
	 *  its middle, closing in by x^2 / 80 either side. */
	bool PlanCentreReach(double X, double Z)
	{
		return FMath::Abs(X) <= 26.0 && Z <= 14.0 - X * X / 80.0 && Z >= -15.0 + X * X / 80.0;
	}

	/** The robot, standing level on flat ground PlanStandCm under its chassis
	 *  origin at the world origin, heading +X, every wheel down. */
	RammsCurb::FPlannerInput PlanRobot()
	{
		RammsCurb::FPlannerInput In;
		RammsCurb::FRobotModel&	 M = In.Model;
		M.CenterOfMass = FVector(0.0, 0.0, 12.0);
		M.MassKg = 100.0;
		M.BellyHalfExtent = FVector2D(36.0, 17.0);
		M.BellyUndersideCm = 2.5;
		In.Tuning = FRammsCurbTuning();
		for (int32 i = 0; i < 6; ++i)
		{
			const bool			  bCentre = i == PlanCL || i == PlanCR;
			const double		  Y = (i % 2 == 0 ? -1.0 : 1.0) * (bCentre ? PlanCentreY : PlanCornerY);
			RammsCurb::FModelLeg& L = M.Legs.AddDefaulted_GetRef();
			L.Kind = bCentre ? ERammsCurbLegKind::FiveBar : ERammsCurbLegKind::FourBar;
			L.Axle = i < 2 ? ERammsCurbAxle::Front : (i < 4 ? ERammsCurbAxle::Center : ERammsCurbAxle::Rear);
			L.Y = Y;
			L.RadiusCm = PlanRadius;
			L.ContactWidthCm = 6.0;
			L.MovingMassKg = bCentre ? 7.0 : 6.0;
			L.RollDirection = FVector2D(1.0, 0.0);
			RammsCurb::FPlannerLeg& P = In.Legs.AddDefaulted_GetRef();
			P.Name = PlanLegName[i];
			P.TyreWidthCm = 6.0;
			P.PreloadCm = In.Tuning.PlantPreload + (bCentre ? In.Tuning.MainWheelPreload : 0.0f);
			const double Z = PlanRadius - P.PreloadCm - PlanStandCm;
			if (bCentre)
			{
				L.ZeroPose = FVector2D(0.0, -3.0);
				P.Grid.Build([](double X, double Zc) { return PlanCentreReach(X, Zc); }, FVector2D(-30.0, 30.0), FVector2D(-20.0, 20.0));
				P.Measured = FVector2D(0.0, Z);
			}
			else
			{
				L.FourBar.Build(PlanCornerLeg(i < 2 ? 1.0 : -1.0));
				L.ZeroPose = FVector2D(L.FourBar.XAt(-2.0), -2.0);
				P.Measured = FVector2D(L.FourBar.XAt(Z), Z);
			}
		}
		In.Origin = FVector2D::ZeroVector;
		In.OriginZ = PlanStandCm;
		In.Heading = FVector2D(1.0, 0.0);
		In.EntrySpeedCmS = 0.0;
		return In;
	}

	/** Two planes and an edge: the near one through the world origin at height
	 *  0 sloping NearGrad, the far one Step above it at the edge, sloping
	 *  FarGrad; the edge PlanEdgeCm ahead on the robot's path, its normal
	 *  ApproachDeg off the heading (positive: the +Y wheels meet it first). */
	FRammsCurbTerrain PlanTerrain(double Step, double ApproachDeg, const FVector2D& NearGrad = FVector2D::ZeroVector,
		const FVector2D& FarGrad = FVector2D::ZeroVector)
	{
		FRammsCurbTerrain T;
		T.bValid = true;
		const double A = FMath::DegreesToRadians(ApproachDeg);
		T.EdgeNormal = FVector2D(FMath::Cos(A), FMath::Sin(A));
		T.EdgeDirection = FVector2D(-T.EdgeNormal.Y, T.EdgeNormal.X);
		T.ApproachAngleDeg = static_cast<float>(ApproachDeg);
		T.Near.Origin = FVector::ZeroVector;
		T.Near.Gradient = NearGrad;
		const FVector2D E(PlanEdgeCm, 0.0);
		T.EdgePoint = FVector(E.X, E.Y, T.Near.HeightAt(E));
		T.Far.Origin = FVector(E.X, E.Y, T.Near.HeightAt(E) + Step);
		T.Far.Gradient = FarGrad;
		T.StepHeightCm = static_cast<float>(Step);
		T.StepHeightRangeCm = FVector2D(Step, Step);
		T.FarDepthCm = 300.0f;
		return T;
	}

	/** The robot on Terrain, its legs standing on the near plane under them. */
	RammsCurb::FPlannerInput PlanOn(const FRammsCurbTerrain& Terrain)
	{
		RammsCurb::FPlannerInput In = PlanRobot();
		In.Terrain = Terrain;
		// Stand each leg on the plane under its wheel, the chassis level.
		for (int32 i = 0; i < 6; ++i)
		{
			RammsCurb::FPlannerLeg& P = In.Legs[i];
			const FVector2D			XY(P.Measured.X, In.Model.Legs[i].Y);
			const double			Z = Terrain.Near.HeightAt(XY) + PlanRadius - P.PreloadCm - PlanStandCm;
			P.Measured.Y = Z;
			if (In.Model.Legs[i].Kind == ERammsCurbLegKind::FourBar)
			{
				P.Measured.X = In.Model.Legs[i].FourBar.XAt(Z);
			}
		}
		In.OriginZ = PlanStandCm;
		return In;
	}

	/** A curb-like test terrain on the made-up robot: the climb (bUp) onto a
	 *  sidewalk 15.24 cm (6 in) up, or the descent off one. Square or Oblique30
	 *  / ObliqueNeg30 (the edge 30 deg off square, the +Y / -Y wheels first) on
	 *  flat ground; else square on with one 6 deg grade: RoadFalls (the road
	 *  falling towards the curb, a gutter), RoadRises (rising towards it),
	 *  StreetGrade (along the curb, under every surface), SidewalkRises (the
	 *  sidewalk rising away from the road), SidewalkTwisted (the sidewalk's own
	 *  grade along the curb). */
	FRammsCurbTerrain PlanScenario(const FString& Id, bool bUp)
	{
		const double	A = Id == TEXT("ObliqueNeg30") ? -30.0 : (Id == TEXT("Oblique30") ? 30.0 : 0.0);
		const FVector2D Nrm(FMath::Cos(FMath::DegreesToRadians(A)), FMath::Sin(FMath::DegreesToRadians(A)));
		const FVector2D Edge(-Nrm.Y, Nrm.X);
		const double	Six = FMath::Tan(FMath::DegreesToRadians(6.0));
		FVector2D		NearG = FVector2D::ZeroVector, FarG = FVector2D::ZeroVector;
		if (Id == TEXT("RoadFalls"))
		{
			// The roads fall towards the curb on either side.
			(bUp ? NearG : FarG) = Nrm * (bUp ? -Six : Six);
		}
		else if (Id == TEXT("RoadRises"))
		{
			// The roads rise towards the curb on either side.
			(bUp ? NearG : FarG) = Nrm * (bUp ? Six : -Six);
		}
		else if (Id == TEXT("StreetGrade"))
		{
			// A grade along the curb, under every surface.
			NearG = FarG = Edge * Six;
		}
		else if (Id == TEXT("SidewalkRises"))
		{
			// The sidewalk rising away from the road.
			(bUp ? FarG : NearG) = Nrm * Six;
		}
		else if (Id == TEXT("SidewalkTwisted"))
		{
			// The sidewalk's own grade along the curb.
			(bUp ? FarG : NearG) = Edge * Six;
		}
		return PlanTerrain(bUp ? 15.24 : -15.24, A, NearG, FarG);
	}

	/** A plan's query at a sample: its sets off the ground and on the far side,
	 *  the centre wheels' fore/aft, its attitude. */
	RammsCurb::FInstantQuery PlanQueryAt(const RammsCurb::FCurbSchedule& S, const RammsCurb::FScheduleSample& Sm)
	{
		RammsCurb::FInstantQuery Q;
		Q.S = Sm.S;
		Q.PitchDeg = Sm.PitchDeg;
		Q.RollDeg = Sm.RollDeg;
		Q.Airborne = Sm.AirborneMask;
		for (int32 i = 0; i < S.Wheels.Num(); ++i)
		{
			if (!((Sm.AirborneMask >> i) & 1u) && Sm.S >= S.Wheels[i].SLoaded)
			{
				Q.OnFar |= 1u << i;
			}
			// Off the ground from its touchdown until loaded (a sample stands for
			// half a sample either side): touching down.
			if (((Sm.AirborneMask >> i) & 1u) && Sm.S + 0.5 * S.SampleCm >= S.Wheels[i].SDown)
			{
				Q.Touching |= 1u << i;
			}
		}
		Q.X[PlanCL] = Sm.X[0];
		Q.X[PlanCR] = Sm.X[1];
		return Q;
	}

	/** Every sample from the level-in on, as the planner says it verified it:
	 *  re-evaluated with the per-instant model at the planned height. */
	void PlanCheckSamples(FAutomationTestBase& Test, const RammsCurb::FPlannerInput& In, const RammsCurb::FCurbSchedule& S,
		const TCHAR* What)
	{
		const RammsCurb::FPlanModel M(In);
		const FRammsCurbTuning&		T = In.Tuning;
		int32						Bad = 0;
		for (const RammsCurb::FScheduleSample& Sm : S.Samples)
		{
			if (Sm.bApproach)
			{
				continue;
			}
			const RammsCurb::FInstantResult R = M.Evaluate(PlanQueryAt(S, Sm), Sm.B, true);
			FString							Why;
			if (!R.bReach)
			{
				Why += TEXT(" a leg cannot reach;");
			}
			if (R.Margin < T.MinStabilityMargin - 1e-6)
			{
				Why += FString::Printf(TEXT(" margin %.3f;"), R.Margin);
			}
			if (!FMath::IsNearlyEqual(R.Margin, Sm.Margin, 1e-6))
			{
				Why += FString::Printf(TEXT(" margin %.4f against the planned %.4f;"), R.Margin, Sm.Margin);
			}
			if (Sm.B < R.Lo - 1e-6 || Sm.B > R.Hi + 1e-6)
			{
				Why += FString::Printf(TEXT(" chassis %.2f outside %.2f..%.2f;"), Sm.B, R.Lo, R.Hi);
			}
			if (Sm.BHi - Sm.BLo < T.MinCorridorCm - 1e-6)
			{
				Why += FString::Printf(TEXT(" corridor %.2f;"), Sm.BHi - Sm.BLo);
			}
			for (int32 i = 0; i < S.Wheels.Num(); ++i)
			{
				const RammsCurb::FWheelPlan& W = S.Wheels[i];
				const double				 D = M.DistancePast(i, Sm.S, R.LegX[i]);
				if (!((Sm.AirborneMask >> i) & 1u) && D > W.DUp + 1e-6 && D < W.DDown - 1e-6)
				{
					Why += FString::Printf(TEXT(" %s on the ground in its window (%.1f);"), PlanLegName[i], D);
				}
			}
			if (RammsCurb::IsForbiddenSet(In.Model, Sm.AirborneMask))
			{
				Why += FString::Printf(TEXT(" forbidden set %x;"), Sm.AirborneMask);
			}
			if (!Why.IsEmpty() && Bad++ < 5)
			{
				Test.AddError(FString::Printf(TEXT("%s at %.0f cm:%s"), What, Sm.S, *Why));
			}
		}
		Test.TestEqual(FString::Printf(TEXT("%s: samples that fail re-evaluation"), What), Bad, 0);
	}

	/** The rate limits, between samples in time at the planned speed (each
	 *  sample-to-sample step taken at its mean speed, as the planner times it;
	 *  the walks, as the planner bounds them, at the faster end's). The tilt
	 *  everywhere, to the limits themselves (its S-curves are exact in time);
	 *  the chassis and the walks from the level-in on, but over a planned
	 *  stop's dwell (a re-pose at rest, through which the tilt holds). */
	void PlanCheckRates(FAutomationTestBase& Test, const RammsCurb::FPlannerInput& In, const RammsCurb::FCurbSchedule& S,
		const TCHAR* What)
	{
		const FRammsCurbTuning& T = In.Tuning;
		const double			Floor = T.MinSpeedFraction * T.DriveSpeed;
		int32					Bad = 0;
		FVector2D				LastRate = FVector2D::ZeroVector;
		double					LastDt = 0.0;
		for (int32 k = 1; k < S.Samples.Num(); ++k)
		{
			const RammsCurb::FScheduleSample& A = S.Samples[k - 1];
			const RammsCurb::FScheduleSample& B = S.Samples[k];
			const double					  V = FMath::Max(0.5, FMath::Max(A.V, B.V));
			const double					  Dt = S.SampleCm / V;
			const double					  DtMean = S.SampleCm / FMath::Max(0.5, 0.5 * (A.V + B.V));
			FString							  Why;
			// Pitch and roll each.
			const FVector2D Rate((B.PitchDeg - A.PitchDeg) / DtMean, (B.RollDeg - A.RollDeg) / DtMean);
			if (Rate.GetAbsMax() > T.TiltRateDegPerSec * (1.0 + 1e-6) + 1e-6)
			{
				Why += FString::Printf(TEXT(" tilt rate %+.3f / %+.3f deg/s;"), Rate.X, Rate.Y);
			}
			// The rate's change between steps, over the time between their
			// middles.
			const double Accel = LastDt > 0.0 ? ((Rate - LastRate) / (0.5 * (DtMean + LastDt))).GetAbsMax() : 0.0;
			if (Accel > T.TiltAccelDegPerSec2 * (1.0 + 1e-6) + 1e-6)
			{
				Why += FString::Printf(TEXT(" tilt acceleration %.3f deg/s^2;"), Accel);
			}
			if ((A.bStop || B.bStop) && Rate.GetAbsMax() > 1e-9)
			{
				Why += TEXT(" tilt changing through a planned stop;");
			}
			LastRate = Rate;
			LastDt = DtMean;
			if (!A.bApproach && !A.bStop)
			{
				if (FMath::Abs(B.B - A.B) > T.BaseHeightSpeed * Dt + 1e-6)
				{
					Why += FString::Printf(TEXT(" chassis %.2f cm/s;"), FMath::Abs(B.B - A.B) / Dt);
				}
				const bool bNearRest = FMath::Min(A.V, B.V) < 0.5 * Floor && S.PlannedStops > 0;
				for (int32 Side = 0; Side < 2; ++Side)
				{
					const int32	 Leg = Side == 0 ? PlanCL : PlanCR;
					const double Dx = B.X[Side] - A.X[Side];
					const bool	 bAir = ((A.AirborneMask >> Leg) & 1u) && ((B.AirborneMask >> Leg) & 1u);
					const double AftMost = bAir || bNearRest ? T.CenterShiftSpeed * Dt : FMath::Min(T.CenterShiftSpeed * Dt, RammsCurb::MaxAftWalkPerCm * S.SampleCm);
					if (Dx > T.CenterShiftSpeed * Dt + 1e-6 || -Dx > AftMost + 1e-6)
					{
						Why += FString::Printf(TEXT(" %s walks %.2f cm in %.3f s;"), PlanLegName[Leg], Dx, Dt);
					}
				}
			}
			if (!Why.IsEmpty() && Bad++ < 5)
			{
				Test.AddError(FString::Printf(TEXT("%s at %.0f cm:%s"), What, B.S, *Why));
			}
		}
		Test.TestEqual(FString::Printf(TEXT("%s: samples over a rate limit"), What), Bad, 0);
	}

	/** Each wheel's crossing in order: lift, clear, land, down, loaded. */
	void PlanCheckWheels(FAutomationTestBase& Test, const RammsCurb::FCurbSchedule& S, const TCHAR* What)
	{
		for (int32 i = 0; i < S.Wheels.Num(); ++i)
		{
			const RammsCurb::FWheelPlan& W = S.Wheels[i];
			Test.TestTrue(FString::Printf(TEXT("%s: %s lifts (%.1f), is clear (%.1f), lands (%.1f), is down (%.1f) and loaded (%.1f) in order"),
							  What, PlanLegName[i], W.SLiftStart, W.SClear, W.SLand, W.SDown, W.SLoaded),
				W.SLiftStart <= W.SClear && W.SClear < W.SLand && W.SLand <= W.SDown && W.SDown <= W.SLoaded);
		}
	}

	FString PlanDump(const RammsCurb::FCurbSchedule& S)
	{
		FString Out = FString::Printf(TEXT("class %s: %s\n"), RammsCurb::PlanClassName(S.Class), *S.Summary);
		for (const FString& L : S.Lines)
		{
			Out += L + TEXT("\n");
		}
		return Out;
	}

	/** The plan, with its lines as test info. */
	RammsCurb::FCurbSchedule PlanAndLog(FAutomationTestBase& Test, const RammsCurb::FPlannerInput& In, const TCHAR* What)
	{
		RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
		Test.AddInfo(FString::Printf(TEXT("%s: %s"), What, *PlanDump(S)));
		return S;
	}

	/** The highest centre-wheel fore/aft (each side) while both front corners
	 *  are off the ground; -inf where they never are together. */
	FVector2D PlanCentresWithFrontsUp(const RammsCurb::FCurbSchedule& S)
	{
		FVector2D Most(-TNumericLimits<double>::Max(), -TNumericLimits<double>::Max());
		for (const RammsCurb::FScheduleSample& Sm : S.Samples)
		{
			if (!Sm.bApproach && ((Sm.AirborneMask >> PlanFL) & 1u) && ((Sm.AirborneMask >> PlanFR) & 1u))
			{
				Most.X = FMath::Max(Most.X, Sm.X[0]);
				Most.Y = FMath::Max(Most.Y, Sm.X[1]);
			}
		}
		return Most;
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerFlatSquareUp,
	"Ramms.Curb.Planner.FlatSquareUp",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerFlatSquareUp::RunTest(const FString& Parameters)
{
	// The made-up legs are what the tests take them for.
	RammsCurb::FFourBarTable Front, Rear;
	TestTrue(TEXT("front 4-bar builds"), Front.Build(PlanCornerLeg(1.0)));
	TestTrue(TEXT("rear 4-bar builds"), Rear.Build(PlanCornerLeg(-1.0)));
	TestTrue(TEXT("front reaches -14..11"), Front.HeightRange().X < -14.0 && Front.HeightRange().Y > 11.0);
	TestTrue(TEXT("rear mirrors front"), FMath::IsNearlyEqual(Front.XAt(-2.0), -Rear.XAt(-2.0), 1e-6));

	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("Square up"));
	TestEqual(TEXT("planned level"), FString(RammsCurb::PlanClassName(S.Class)), FString(TEXT("level")));
	TestTrue(TEXT("at or above the floor"), S.VFloor >= In.Tuning.MinSpeedFraction * In.Tuning.DriveSpeed - 1e-6);
	TestTrue(TEXT("margin at least MinStabilityMargin"), S.MinMargin >= In.Tuning.MinStabilityMargin - 1e-6);
	TestTrue(TEXT("corridor at least MinCorridorCm"), S.NarrowestCorridorCm >= In.Tuning.MinCorridorCm - 1e-6);
	TestEqual(TEXT("no tilt"), S.PlannedMaxTiltDeg, 0.0);
	TestEqual(TEXT("no planned stop"), S.PlannedStops, 0);
	PlanCheckWheels(*this, S, TEXT("Square up"));
	// Front, then centre, then rear, both sides together (square on).
	TestTrue(TEXT("the front corners clear before the centre wheels, they before the rears"),
		S.Wheels[PlanFL].SClear < S.Wheels[PlanCL].SClear && S.Wheels[PlanCL].SClear < S.Wheels[PlanRL].SClear
			&& S.Wheels[PlanFR].SClear < S.Wheels[PlanCR].SClear && S.Wheels[PlanCR].SClear < S.Wheels[PlanRR].SClear);
	TestTrue(TEXT("square on, the two sides together"), FMath::Abs(S.Wheels[PlanFL].SClear - S.Wheels[PlanFR].SClear) <= 1.0);
	PlanCheckSamples(*this, In, S, TEXT("Square up"));
	PlanCheckRates(*this, In, S, TEXT("Square up"));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerFlatSquareDown,
	"Ramms.Curb.Planner.FlatSquareDown",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerFlatSquareDown::RunTest(const FString& Parameters)
{
	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), false));
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("Square down"));
	TestEqual(TEXT("planned level"), FString(RammsCurb::PlanClassName(S.Class)), FString(TEXT("level")));
	TestTrue(TEXT("at or above the floor"), S.VFloor >= In.Tuning.MinSpeedFraction * In.Tuning.DriveSpeed - 1e-6);
	PlanCheckWheels(*this, S, TEXT("Square down"));
	// Descending, a wheel lifts off the high side just short of the lip and
	// lands a radius and DropClearance past it: its window is the lip's.
	const RammsCurb::FWheelPlan& F = S.Wheels[PlanFL];
	TestNearlyEqual(TEXT("descending, a wheel's window opens EdgeMargin short of the lip"), F.DUp, -static_cast<double>(In.Tuning.EdgeMargin),
		1e-6);
	TestNearlyEqual(TEXT("and closes DropClearance plus its radius past it"), F.DDown, In.Tuning.DropClearance + PlanRadius, 1e-6);
	PlanCheckSamples(*this, In, S, TEXT("Square down"));
	PlanCheckRates(*this, In, S, TEXT("Square down"));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerAngle30,
	"Ramms.Curb.Planner.Angle30",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerAngle30::RunTest(const FString& Parameters)
{
	for (const bool bNeg : { false, true })
	{
		const TCHAR*				   Id = bNeg ? TEXT("ObliqueNeg30") : TEXT("Oblique30");
		const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(Id, true));
		const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, Id);
		TestEqual(FString::Printf(TEXT("%s: planned level"), Id), FString(RammsCurb::PlanClassName(S.Class)), FString(TEXT("level")));
		TestTrue(FString::Printf(TEXT("%s: at or above the floor"), Id), S.VFloor >= In.Tuning.MinSpeedFraction * In.Tuning.DriveSpeed - 1e-6);
		PlanCheckWheels(*this, S, Id);
		// The side whose wheels meet the edge first leads on every axle: +Y at
		// +30 deg, -Y at -30.
		const int32 Pairs[3][2] = { { PlanFL, PlanFR }, { PlanCL, PlanCR }, { PlanRL, PlanRR } };
		for (const auto& Pair : Pairs)
		{
			const int32 First = bNeg ? Pair[0] : Pair[1], Second = bNeg ? Pair[1] : Pair[0];
			TestTrue(FString::Printf(TEXT("%s: %s crosses before %s"), Id, PlanLegName[First], PlanLegName[Second]),
				S.Wheels[First].SClear < S.Wheels[Second].SClear - 10.0);
		}
		// Each wheel's window from its own axle: (w_t / 2) |sin 30| = 1.5 cm wider
		// on the far side, and R cos 30 + 1.5 on the near one.
		const RammsCurb::FWheelPlan& W = S.Wheels[PlanFR];
		TestNearlyEqual(FString::Printf(TEXT("%s: lambda"), Id), W.Lambda, 1.5, 1e-6);
		TestNearlyEqual(FString::Printf(TEXT("%s: eps"), Id), W.Eps, PlanRadius * FMath::Cos(FMath::DegreesToRadians(30.0)) + 1.5, 1e-6);
		PlanCheckSamples(*this, In, S, Id);
		PlanCheckRates(*this, In, S, Id);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerAngleNeg30,
	"Ramms.Curb.Planner.AngleNeg30",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerAngleNeg30::RunTest(const FString& Parameters)
{
	// A -30 deg plan is the +30 deg one mirrored: the same timing, sides
	// swapped.
	const RammsCurb::FCurbSchedule P = RammsCurb::PlanCrossing(PlanOn(PlanScenario(TEXT("Oblique30"), true)));
	const RammsCurb::FCurbSchedule N = RammsCurb::PlanCrossing(PlanOn(PlanScenario(TEXT("ObliqueNeg30"), true)));
	TestEqual(TEXT("both level"), FString(RammsCurb::PlanClassName(N.Class)), FString(RammsCurb::PlanClassName(P.Class)));
	const int32 Mirror[6] = { PlanFR, PlanFL, PlanCR, PlanCL, PlanRR, PlanRL };
	for (int32 i = 0; i < 6; ++i)
	{
		TestNearlyEqual(FString::Printf(TEXT("%s's window entry mirrors"), PlanLegName[i]), N.Wheels[i].SClear, P.Wheels[Mirror[i]].SClear,
			1.0);
		TestNearlyEqual(FString::Printf(TEXT("%s's landing mirrors"), PlanLegName[i]), N.Wheels[i].SLand, P.Wheels[Mirror[i]].SLand, 1.0);
	}
	TestNearlyEqual(TEXT("the same floor"), N.VFloor, P.VFloor, 1.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerNoForbiddenSets,
	"Ramms.Curb.Planner.NoForbiddenSets",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerNoForbiddenSets::RunTest(const FString& Parameters)
{
	// The forbidden sets are the model's, not a table: standing still on
	// flat ground, a same-side corner and centre, both fronts and a centre, a
	// centre and both rears leave the robot under the margin, every single
	// wheel and the centre pair do not ...
	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	const RammsCurb::FPlanModel	   M(In);
	auto						   Margin = [&](uint32 Up) {
		  RammsCurb::FInstantQuery Q;
		  Q.S = 0.0;
		  Q.Airborne = Up;
		  return M.Evaluate(Q, PlanStandCm).Margin;
	};
	const uint32 Forbidden[] = { (1u << PlanFL) | (1u << PlanCL), (1u << PlanFR) | (1u << PlanCR), (1u << PlanCL) | (1u << PlanRL),
		(1u << PlanCR) | (1u << PlanRR), (1u << PlanFL) | (1u << PlanFR) | (1u << PlanCL),
		(1u << PlanCL) | (1u << PlanRL) | (1u << PlanRR) };
	for (const uint32 Up : Forbidden)
	{
		TestTrue(FString::Printf(TEXT("set %x: the model forbids it (%.2f cm)"), Up, Margin(Up)), Margin(Up) < In.Tuning.MinStabilityMargin);
		TestTrue(FString::Printf(TEXT("set %x: so does IsForbiddenSet"), Up), RammsCurb::IsForbiddenSet(In.Model, Up));
	}
	for (int32 i = 0; i < 6; ++i)
	{
		TestTrue(FString::Printf(TEXT("%s alone is allowed (%.2f cm)"), PlanLegName[i], Margin(1u << i)),
			Margin(1u << i) >= In.Tuning.MinStabilityMargin);
	}
	TestTrue(TEXT("both centres are allowed"), Margin((1u << PlanCL) | (1u << PlanCR)) >= In.Tuning.MinStabilityMargin);
	// ... and no plan ever lifts one.
	for (const TCHAR* Id : { TEXT("Square"), TEXT("Oblique30"), TEXT("ObliqueNeg30") })
	{
		for (const bool bUp : { true, false })
		{
			const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(PlanOn(PlanScenario(Id, bUp)));
			int32						   Count = 0;
			for (const RammsCurb::FScheduleSample& Sm : S.Samples)
			{
				Count += !Sm.bApproach && RammsCurb::IsForbiddenSet(In.Model, Sm.AirborneMask);
			}
			TestEqual(FString::Printf(TEXT("%s %s: samples with a forbidden set up"), Id, bUp ? TEXT("up") : TEXT("down")), Count, 0);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerVerifyEveryCm,
	"Ramms.Curb.Planner.VerifyEveryCm",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerVerifyEveryCm::RunTest(const FString& Parameters)
{
	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Oblique30"), false));
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("Oblique30 down"));
	TestTrue(TEXT("planned"), S.IsValid());
	TestEqual(TEXT("one sample per cm"), S.SampleCm, 1.0);
	bool bSpaced = S.Samples.Num() > 100;
	for (int32 k = 0; k < S.Samples.Num(); ++k)
	{
		bSpaced = bSpaced && FMath::IsNearlyEqual(S.Samples[k].S, static_cast<double>(k), 1e-9);
	}
	TestTrue(TEXT("every sample 1 cm on from the last"), bSpaced);
	TestTrue(TEXT("past the last landing"), S.Samples.Last().S >= S.Wheels[PlanRL].SLoaded && S.Samples.Last().S >= S.Wheels[PlanRR].SLoaded);
	PlanCheckSamples(*this, In, S, TEXT("Oblique30 down"));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerFloorRespected,
	"Ramms.Curb.Planner.FloorRespected",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerFloorRespected::RunTest(const FString& Parameters)
{
	// A floor the crossing cannot keep, no tilt and no stop: refused, saying so.
	RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	In.Tuning.MinSpeedFraction = 0.95f;
	In.Tuning.MaxPlannedTiltDeg = 0.0f;
	In.Tuning.bAllowPlannedStop = false;
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("Square up, floor 95 %"));
	TestEqual(TEXT("refused"), FString(RammsCurb::PlanClassName(S.Class)), FString(TEXT("refused")));
	TestTrue(TEXT("naming the floor"), S.Summary.Contains(TEXT("floor")));
	// A slow crossing allowed: planned level, under that floor.
	In.Tuning.bAllowSlowCrossing = true;
	const RammsCurb::FCurbSchedule Slow = PlanAndLog(*this, In, TEXT("Square up, floor 95 %, slow allowed"));
	TestEqual(TEXT("planned level when allowed slow"), FString(RammsCurb::PlanClassName(Slow.Class)), FString(TEXT("level")));
	TestTrue(TEXT("and slower than that floor"), Slow.VFloor < 0.95 * In.Tuning.DriveSpeed);
	// Slow, never stopped: a cap that would bring it to rest is no plan (a stop
	// is planned, counted and said, or not made).
	bool bRisenSlow = false, bMoving = true;
	for (int32 k = 0; k + 1 < Slow.Samples.Num(); ++k)
	{
		bRisenSlow = bRisenSlow || Slow.Samples[k + 1].V <= Slow.Samples[k].V;
		bMoving = bMoving && (!bRisenSlow || Slow.Samples[k].bStop || Slow.Samples[k].V > RammsCurb::StopSpeedCmS);
	}
	TestTrue(FString::Printf(TEXT("never at rest but at a planned stop (floor %.2f cm/s, %d planned)"), Slow.VFloor, Slow.PlannedStops), bMoving);
	// The default floor holds every planned speed once risen.
	const RammsCurb::FPlannerInput D = PlanOn(PlanScenario(TEXT("Square"), true));
	const RammsCurb::FCurbSchedule P = RammsCurb::PlanCrossing(D);
	const double				   Floor = D.Tuning.MinSpeedFraction * D.Tuning.DriveSpeed;
	bool						   bRisen = false, bOk = true;
	for (int32 k = 0; k + 1 < P.Samples.Num(); ++k)
	{
		bRisen = bRisen || P.Samples[k + 1].V <= P.Samples[k].V;
		bOk = bOk && (!bRisen || P.Samples[k].V >= Floor - 1e-6);
	}
	TestTrue(TEXT("never under the floor once risen"), bOk);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerWalkingWidensGap,
	"Ramms.Curb.Planner.WalkingWidensGap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerWalkingWidensGap::RunTest(const FString& Parameters)
{
	// Climbing square on, both front corners are up together: the robot stands
	// on its centre and rear wheels, the centre of mass over the centre pair --
	// only the centre wheels walking forward keeps it inside them.
	RammsCurb::FPlannerInput NoWalk = PlanOn(PlanScenario(TEXT("Square"), true));
	NoWalk.bTryKnots = NoWalk.bTrySearch = false;
	const RammsCurb::FCurbSchedule A = PlanAndLog(*this, NoWalk, TEXT("Square up, the centre wheels not walking"));
	TestNotEqual(TEXT("not level without walking"), FString(RammsCurb::PlanClassName(A.Class)), FString(TEXT("level")));
	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	const RammsCurb::FCurbSchedule B = RammsCurb::PlanCrossing(In);
	TestEqual(TEXT("level walking"), FString(RammsCurb::PlanClassName(B.Class)), FString(TEXT("level")));
	const FVector2D Fwd = PlanCentresWithFrontsUp(B);
	TestTrue(FString::Printf(TEXT("the centre wheels forward with the front corners up (%.1f / %.1f cm)"), Fwd.X, Fwd.Y),
		Fwd.X >= 4.0 && Fwd.Y >= 4.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerDPFallbackFindsPlan,
	"Ramms.Curb.Planner.DPFallbackFindsPlan",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerDPFallbackFindsPlan::RunTest(const FString& Parameters)
{
	// With neither standing still nor the knots, the search over the centre
	// wheels' paths alone finds the level crossing.
	RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	In.bTryNoWalk = In.bTryKnots = false;
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("Square up, the search alone"));
	TestEqual(TEXT("level"), FString(RammsCurb::PlanClassName(S.Class)), FString(TEXT("level")));
	bool bSearch = false;
	for (const FString& L : S.Lines)
	{
		bSearch = bSearch || L.StartsWith(TEXT("centre wheels: search"));
	}
	TestTrue(TEXT("found by the search"), bSearch);
	const FVector2D Fwd = PlanCentresWithFrontsUp(S);
	TestTrue(FString::Printf(TEXT("forward with the front corners up (%.1f / %.1f cm)"), Fwd.X, Fwd.Y), Fwd.X >= 4.0 && Fwd.Y >= 4.0);
	PlanCheckSamples(*this, In, S, TEXT("the search's plan"));
	PlanCheckRates(*this, In, S, TEXT("the search's plan"));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerSpanTooLargeRefuses,
	"Ramms.Curb.Planner.SpanTooLargeRefuses",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerSpanTooLargeRefuses::RunTest(const FString& Parameters)
{
	// A step the legs cannot span at any attitude: refused, naming the leg or
	// the belly that binds and where.
	RammsCurb::FPlannerInput	   In = PlanOn(PlanTerrain(30.0, 0.0));
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("a 30 cm step"));
	TestEqual(TEXT("refused"), FString(RammsCurb::PlanClassName(S.Class)), FString(TEXT("refused")));
	TestTrue(TEXT("naming what binds"),
		S.Summary.Contains(TEXT("leg '")) || S.Summary.Contains(TEXT("belly")) || S.Summary.Contains(TEXT("inside its wheels")));
	TestTrue(TEXT("and where"), S.Summary.Contains(TEXT(" cm on (")));
	TestTrue(TEXT("and the tilt it would need"), S.Summary.Contains(TEXT("MaxPlannedTiltDeg")));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerLeastTiltMinimal,
	"Ramms.Curb.Planner.LeastTiltMinimal",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerLeastTiltMinimal::RunTest(const FString& Parameters)
{
	// Roads rising 6 deg into the curb: the front corner must swing high over
	// the edge while the rear corners stand on road far below -- no level
	// pose. The planner tilts the least that keeps one, held only over the
	// stretch that needs it, and comes back to level.
	RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("RoadRises"), true));
	In.Tuning.MaxPlannedTiltDeg = 6.0f;
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("RoadRises up"));
	TestEqual(TEXT("tilted"), FString(RammsCurb::PlanClassName(S.Class)), FString(TEXT("tilt")));
	if (S.Class != RammsCurb::EPlanClass::Tilt)
	{
		return true;
	}
	const double t = S.PlannedMaxTiltDeg;
	TestTrue(TEXT("within MaxPlannedTiltDeg"), t <= In.Tuning.MaxPlannedTiltDeg + 1e-6);
	{
		// A ring's (its inclination on the grid) or a square's (its larger axis
		// on the grid).
		FVector2D Most = FVector2D::ZeroVector;
		for (const RammsCurb::FScheduleSample& Sm : S.Samples)
		{
			Most = RammsCurb::InclinationDeg(Sm.PitchDeg, Sm.RollDeg) > RammsCurb::InclinationDeg(Most.X, Most.Y) ? FVector2D(Sm.PitchDeg, Sm.RollDeg) : Most;
		}
		auto OnGrid = [](double V) { return FMath::IsNearlyEqual(V / RammsCurb::TiltGridDeg, FMath::RoundToDouble(V / RammsCurb::TiltGridDeg), 1e-6); };
		TestTrue(FString::Printf(TEXT("its inclination (%.4f) or its larger axis (%+.4f/%+.4f) on the 0.25 deg grid"), t, Most.X, Most.Y),
			OnGrid(t) || OnGrid(FMath::Max(FMath::Abs(Most.X), FMath::Abs(Most.Y))));
	}
	TestEqual(TEXT("level again at the end"), S.Samples.Last().PitchDeg + FMath::Abs(S.Samples.Last().RollDeg), 0.0);
	TestEqual(TEXT("level at the start"), S.Samples[0].PitchDeg + FMath::Abs(S.Samples[0].RollDeg), 0.0);
	// Least: a quarter degree less inclination, in no direction on its ring,
	// keeps a pose at every held sample.
	const RammsCurb::FPlanModel M(In);
	const double				Less = t - RammsCurb::TiltGridDeg;
	bool						bAnyHolds = false;
	{
		for (const FVector2D D : RammsCurb::TiltCandidates(Less, RammsCurb::TiltGridDeg))
		{
			if (bAnyHolds)
			{
				break;
			}
			bool bAll = true;
			for (const RammsCurb::FScheduleSample& Sm : S.Samples)
			{
				if (Sm.bApproach || RammsCurb::InclinationDeg(Sm.PitchDeg, Sm.RollDeg) < t - 1e-9)
				{
					continue;
				}
				RammsCurb::FInstantQuery Q = PlanQueryAt(S, Sm);
				Q.PitchDeg = D.X;
				Q.RollDeg = D.Y;
				const RammsCurb::FInstantResult R = M.Evaluate(Q, Sm.B);
				bool							bHere = R.Width() >= In.Tuning.MinCorridorCm && R.bReach && R.Margin >= In.Tuning.MinStabilityMargin;
				for (int32 p = 0; p <= 12 && !bHere && R.Width() >= In.Tuning.MinCorridorCm; ++p)
				{
					RammsCurb::FInstantResult Rp = R;
					M.PoseAt(Q, R.Lo + 0.25 + (FMath::Min(R.Hi, R.Lo + 60.0) - R.Lo - 0.5) * p / 12.0, Rp);
					bHere = Rp.bReach && Rp.Margin >= In.Tuning.MinStabilityMargin;
				}
				bAll = bAll && bHere;
				if (!bAll)
				{
					break;
				}
			}
			bAnyHolds = bAnyHolds || bAll;
		}
	}
	TestFalse(FString::Printf(TEXT("%.2f deg less than the planned %.2f keeps no pose over the held stretch"), RammsCurb::TiltGridDeg, t),
		bAnyHolds);
	PlanCheckSamples(*this, In, S, TEXT("RoadRises up"));
	PlanCheckRates(*this, In, S, TEXT("RoadRises up"));
	// At the default limit it is not allowed that much: refused, naming it.
	RammsCurb::FPlannerInput Tight = In;
	Tight.Tuning.MaxPlannedTiltDeg = static_cast<float>(FMath::Max(0.25, t - 0.5));
	Tight.Tuning.bAllowPlannedStop = false;
	const RammsCurb::FCurbSchedule R = RammsCurb::PlanCrossing(Tight);
	TestEqual(TEXT("over the limit: refused"), FString(RammsCurb::PlanClassName(R.Class)), FString(TEXT("refused")));
	TestTrue(TEXT("naming MaxPlannedTiltDeg"), R.Summary.Contains(TEXT("MaxPlannedTiltDeg")));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerInclinationBound,
	"Ramms.Curb.Planner.InclinationBound",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerInclinationBound::RunTest(const FString& Parameters)
{
	// The inclination is the angle the base's plane makes with the horizontal:
	// the chassis frame's up axis against the vertical, whichever way it leans.
	for (const FVector2D A : { FVector2D(3.0, 0.0), FVector2D(0.0, -2.5), FVector2D(3.5, 3.5), FVector2D(-4.0, 1.5) })
	{
		const RammsCurb::FChassisFrame F = RammsCurb::ChassisFrameFrom(FVector(0.6, 0.8, 0.0), FMath::DegreesToRadians(A.X), FMath::DegreesToRadians(A.Y),
			FVector::ZeroVector, FVector::ZeroVector);
		const double				   Up = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(F.Z.Z, -1.0, 1.0)));
		TestTrue(FString::Printf(TEXT("pitch %+.1f, roll %+.1f: inclined %.4f deg, the frame's up axis %.4f off the vertical"), A.X, A.Y,
					 RammsCurb::InclinationDeg(A.X, A.Y), Up),
			FMath::IsNearlyEqual(RammsCurb::InclinationDeg(A.X, A.Y), Up, 1e-6));
	}
	TestTrue(TEXT("3.5 deg of pitch and of roll is 4.94 deg of inclination"), FMath::IsNearlyEqual(RammsCurb::InclinationDeg(3.5, 3.5), 4.943, 1e-3));
	// The safety limit MaxTiltDeg is an inclination too: 9 deg of pitch and of
	// roll is past 12 (12.7 deg), 11.9 of pitch alone is not.
	TestTrue(TEXT("9 / 9 is past a 12 deg MaxTiltDeg"), RammsCurb::OverMaxTilt(9.0, 9.0, 12.0));
	TestFalse(TEXT("11.9 of pitch alone is not"), RammsCurb::OverMaxTilt(11.9, 0.0, 12.0));
	TestTrue(TEXT("12.1 of roll alone is"), RammsCurb::OverMaxTilt(0.0, -12.1, 12.0));

	// Roads rising 6 deg into a curb met 30 deg off square: the least tilt that
	// spans them leans the base along the edge's normal -- in pitch and roll at
	// once, so its inclination is more than either.
	const double			 Six = FMath::Tan(FMath::DegreesToRadians(6.0));
	const FVector2D			 Nrm(FMath::Cos(FMath::DegreesToRadians(30.0)), FMath::Sin(FMath::DegreesToRadians(30.0)));
	const FRammsCurbTerrain	 Terrain = PlanTerrain(15.24, 30.0, Nrm * Six, FVector2D::ZeroVector);
	RammsCurb::FPlannerInput In = PlanOn(Terrain);
	In.Tuning.MaxPlannedTiltDeg = 8.0f;
	In.Tuning.bAllowPlannedStop = false;
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("roads rising into an oblique curb"));
	if (!TestEqual(TEXT("tilted"), FString(RammsCurb::PlanClassName(S.Class)), FString(TEXT("tilt"))))
	{
		return true;
	}
	double MaxIncl = 0.0, MaxAxis = 0.0, InclAt = 0.0;
	for (const RammsCurb::FScheduleSample& Sm : S.Samples)
	{
		const double Incl = RammsCurb::InclinationDeg(Sm.PitchDeg, Sm.RollDeg);
		MaxAxis = FMath::Max(MaxAxis, FMath::Max(FMath::Abs(Sm.PitchDeg), FMath::Abs(Sm.RollDeg)));
		if (Incl > MaxIncl)
		{
			MaxIncl = Incl;
			InclAt = Sm.S;
		}
	}
	TestNearlyEqual(TEXT("the plan's largest tilt is its largest inclination"), S.PlannedMaxTiltDeg, MaxIncl, 1e-9);
	TestTrue(FString::Printf(TEXT("inclined %.2f deg at most (at %.0f cm), within MaxPlannedTiltDeg"), MaxIncl, InclAt),
		MaxIncl <= In.Tuning.MaxPlannedTiltDeg + 1e-6);
	TestTrue(FString::Printf(TEXT("leaning in pitch and roll at once: inclined %.2f deg, its larger axis %.2f"), MaxIncl, MaxAxis), MaxIncl > MaxAxis + 0.1);
	PlanCheckSamples(*this, In, S, TEXT("oblique rising roads"));
	PlanCheckRates(*this, In, S, TEXT("oblique rising roads"));

	// The limit bounds the inclination, not each axis: under a limit the
	// plan's larger axis keeps within but its inclination does not, there is
	// no plan -- no tilt inclining the base less keeps a pose -- and the
	// refusal names MaxPlannedTiltDeg (and the inclination it would need).
	RammsCurb::FPlannerInput Tight = In;
	Tight.Tuning.MaxPlannedTiltDeg = static_cast<float>(0.5 * (MaxAxis + MaxIncl));
	TestTrue(FString::Printf(TEXT("a limit of %.2f deg would hold the plan's pitch and roll each (%.2f at most), not its inclination (%.2f)"),
				 Tight.Tuning.MaxPlannedTiltDeg, MaxAxis, MaxIncl),
		MaxAxis <= Tight.Tuning.MaxPlannedTiltDeg && Tight.Tuning.MaxPlannedTiltDeg < MaxIncl);
	const RammsCurb::FCurbSchedule R = PlanAndLog(*this, Tight, TEXT("the same under a limit its inclination exceeds"));
	TestEqual(TEXT("refused"), FString(RammsCurb::PlanClassName(R.Class)), FString(TEXT("refused")));
	TestTrue(TEXT("naming MaxPlannedTiltDeg"), R.Summary.Contains(TEXT("MaxPlannedTiltDeg")));

	// The executor holds a plan to the same bound: the tilted plan, run on the
	// tighter limit, does not begin; on its own, it runs inclined as planned.
	const RammsCurb::FExecDryRunResult Over = RammsCurb::ExecDryRun(Tight, S);
	TestFalse(TEXT("the executor does not run a plan over the limit"), Over.bDone);
	TestTrue(FString::Printf(TEXT("and says so (%s)"), Over.Notes.Num() ? *Over.Notes[0] : TEXT("nothing")),
		Over.Notes.Num() > 0 && Over.Notes[0].Contains(TEXT("MaxPlannedTiltDeg")));
	const RammsCurb::FDryRunResult Run = RammsCurb::DryRun(In, S);
	TestNearlyEqual(TEXT("the dry run inclines the base as planned"), Run.MaxPlannedTiltDeg, S.PlannedMaxTiltDeg, 1e-6);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerPlannedStopWhenAllowed,
	"Ramms.Curb.Planner.PlannedStopWhenAllowed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerPlannedStopWhenAllowed::RunTest(const FString& Parameters)
{
	// A floor the crossing cannot keep and no tilt: with planned stops allowed,
	// level with a stop where it cannot roll; without, refused.
	RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	In.Tuning.MinSpeedFraction = 0.8f;
	In.Tuning.MaxPlannedTiltDeg = 0.0f;
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("Square up, floor 80 %"));
	TestEqual(TEXT("a planned stop"), FString(RammsCurb::PlanClassName(S.Class)), FString(TEXT("planned stop")));
	TestTrue(TEXT("at least one stop"), S.PlannedStops >= 1);
	bool bRest = false;
	for (const RammsCurb::FScheduleSample& Sm : S.Samples)
	{
		bRest = bRest || (Sm.bStop && Sm.V == 0.0);
	}
	TestTrue(TEXT("at rest at the stop"), bRest);
	TestEqual(TEXT("level"), S.PlannedMaxTiltDeg, 0.0);
	PlanCheckSamples(*this, In, S, TEXT("the stopping plan"));
	In.Tuning.bAllowPlannedStop = false;
	const RammsCurb::FCurbSchedule R = RammsCurb::PlanCrossing(In);
	TestEqual(TEXT("refused without"), FString(RammsCurb::PlanClassName(R.Class)), FString(TEXT("refused")));
	TestTrue(TEXT("saying stops are not allowed"), R.Summary.Contains(TEXT("bAllowPlannedStop")));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerRespectsRateLimits,
	"Ramms.Curb.Planner.RespectsRateLimits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerRespectsRateLimits::RunTest(const FString& Parameters)
{
	for (const TCHAR* Id : { TEXT("Square"), TEXT("Oblique30"), TEXT("RoadFalls"), TEXT("SidewalkRises") })
	{
		for (const bool bUp : { true, false })
		{
			const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(Id, bUp));
			const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
			const FString				   What = FString::Printf(TEXT("%s %s"), Id, bUp ? TEXT("up") : TEXT("down"));
			TestTrue(FString::Printf(TEXT("%s planned (%s)"), *What, *S.Summary), S.IsValid());
			PlanCheckRates(*this, In, S, *What);
		}
	}
	// A tilted plan's tilt eases in and out within TiltRateDegPerSec and
	// TiltAccelDegPerSec2.
	RammsCurb::FPlannerInput Tilt = PlanOn(PlanScenario(TEXT("RoadRises"), true));
	Tilt.Tuning.MaxPlannedTiltDeg = 6.0f;
	const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(Tilt);
	TestTrue(TEXT("RoadRises tilted"), S.Class == RammsCurb::EPlanClass::Tilt);
	PlanCheckRates(*this, Tilt, S, TEXT("RoadRises up"));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerAsyncMatchesSync,
	"Ramms.Curb.Planner.AsyncMatchesSync",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerAsyncMatchesSync::RunTest(const FString& Parameters)
{
	// A re-plan runs off the game thread on a copy of its input: the caller is
	// held only for the copy, the plan is the one the same input gives
	// synchronously, and a second launch drops the first's result.
	RammsCurb::FPlannerInput	   In = PlanOn(PlanScenario(TEXT("Oblique30"), true));
	const RammsCurb::FCurbSchedule Sync = RammsCurb::PlanCrossing(In);
	RammsCurb::FPlanTask		   Task;
	TestFalse(TEXT("nothing pending at first"), Task.IsPending());
	Task.Launch(In);
	// The input may change (or go) once launched.
	In.Terrain.StepHeightCm = 99.0f;
	In.Legs.Reset();
	TestTrue(TEXT("pending once launched"), Task.IsPending());
	TestTrue(FString::Printf(TEXT("the launch held the caller %.2f ms (under 5)"), 1000.0 * Task.LaunchSeconds()), Task.LaunchSeconds() < 0.005);
	Task.Wait();
	RammsCurb::FCurbSchedule Async;
	TestTrue(TEXT("taken once done"), Task.Take(Async));
	TestFalse(TEXT("nothing pending once taken"), Task.IsPending());
	TestEqual(TEXT("the same class"), static_cast<int32>(Async.Class), static_cast<int32>(Sync.Class));
	TestEqual(TEXT("the same floor"), Async.VFloor, Sync.VFloor);
	TestEqual(TEXT("as many samples"), Async.Samples.Num(), Sync.Samples.Num());
	bool bSame = Async.Samples.Num() == Sync.Samples.Num();
	for (int32 k = 0; bSame && k < Sync.Samples.Num(); ++k)
	{
		const RammsCurb::FScheduleSample& A = Async.Samples[k];
		const RammsCurb::FScheduleSample& S = Sync.Samples[k];
		bSame = A.V == S.V && A.B == S.B && A.X[0] == S.X[0] && A.X[1] == S.X[1] && A.AirborneMask == S.AirborneMask && A.PitchDeg == S.PitchDeg
			&& A.RollDeg == S.RollDeg;
	}
	TestTrue(TEXT("sample for sample the same plan"), bSame);
	AddInfo(FString::Printf(TEXT("planned in %.1f ms off the game thread, %.1f ms on it"), Async.PlanMs, Sync.PlanMs));
	// Only the latest counts.
	Task.Launch(PlanOn(PlanScenario(TEXT("Square"), true)));
	Task.Launch(PlanOn(PlanScenario(TEXT("Square"), false)));
	Task.Wait();
	RammsCurb::FCurbSchedule Latest;
	TestTrue(TEXT("the latest taken"), Task.Take(Latest));
	TestTrue(TEXT("and it is the descent's"), Latest.IsValid() && Latest.Wheels.Num() == 6 && Latest.Wheels[PlanFL].DUp < 0.0 && FMath::IsNearlyEqual(Latest.Wheels[PlanFL].DUp, -static_cast<double>(FRammsCurbTuning().EdgeMargin), 1e-6));
	return true;
}

// --- the dry runs ---------------------------------------------------------------

namespace
{
	/** A plan's dry run, and what it must keep: it completes; never under the
	 *  floor once risen, nor stopped, away from the planned stops it makes
	 *  (each of them); no wheel loaded inside its window, no forbidden set off
	 *  the ground, no 5-bar out of reach; lifted wheels clear; the lagging
	 *  robot stands; the base within 0.5 deg of the planned attitude -- and
	 *  tilted as much as the plan tilts it. */
	RammsCurb::FDryRunResult PlanDryRunOf(FAutomationTestBase& Test, const FString& What, const RammsCurb::FPlannerInput& In,
		const RammsCurb::FCurbSchedule& S, const RammsCurb::FDryRunOptions& Options = RammsCurb::FDryRunOptions())
	{
		const RammsCurb::FDryRunResult R = RammsCurb::DryRun(In, S, Options);
		for (const FString& N : R.Notes)
		{
			Test.AddInfo(FString::Printf(TEXT("%s: %s"), *What, *N));
		}
		const double Floor = In.Tuning.MinSpeedFraction * In.Tuning.DriveSpeed;
		Test.TestTrue(FString::Printf(TEXT("%s: completes (%.1f s)"), *What, R.Seconds), R.bCompleted);
		Test.TestTrue(FString::Printf(TEXT("%s: never under the floor once risen, but at a planned stop (%.2f cm/s)"), *What, R.MinSpeedCmS),
			R.MinSpeedCmS >= Floor - 1e-6);
		Test.TestEqual(FString::Printf(TEXT("%s: stops not planned"), *What), R.Stops, 0);
		Test.TestEqual(FString::Printf(TEXT("%s: planned stops made"), *What), R.PlannedStops, S.PlannedStops);
		Test.TestEqual(FString::Printf(TEXT("%s: wheels on the ground inside their windows"), *What), R.KeepOffViolations, 0);
		Test.TestEqual(FString::Printf(TEXT("%s: forbidden sets off the ground"), *What), R.ForbiddenSets, 0);
		Test.TestEqual(FString::Printf(TEXT("%s: 5-bars commanded out of reach"), *What), R.KneeViolations, 0);
		Test.TestTrue(FString::Printf(TEXT("%s: lifted wheels clear the terrain (least %.2f cm)"), *What, R.MinSwingClearanceCm),
			R.MinSwingClearanceCm >= 0.0);
		Test.TestTrue(FString::Printf(TEXT("%s: the base within 0.5 deg of the plan (%.2f)"), *What, R.MaxAttitudeErrorDeg),
			R.MaxAttitudeErrorDeg <= 0.5);
		Test.TestTrue(FString::Printf(TEXT("%s: the lagging robot stands (least margin %.2f cm)"), *What, R.MinMarginCm), R.MinMarginCm >= 0.0);
		Test.TestNearlyEqual(FString::Printf(TEXT("%s: the base tilted as planned"), *What), R.MaxPlannedTiltDeg, S.PlannedMaxTiltDeg, 1e-6);
		Test.AddInfo(FString::Printf(TEXT("%s: %.1f s, least %.1f cm/s, clearance %.2f cm, margin %.2f cm, attitude %.2f deg, tilt %.2f deg, %d "
										  "planned stops (%.2f s at rest)"),
			*What, R.Seconds, R.MinSpeedCmS, R.MinSwingClearanceCm, R.MinMarginCm, R.MaxAttitudeErrorDeg, R.MaxPlannedTiltDeg, R.PlannedStops,
			R.PlannedDwellSeconds));
		return R;
	}

	void PlanDryRun(FAutomationTestBase& Test, const TCHAR* Id, bool bUp)
	{
		const FString				   What = FString::Printf(TEXT("%s %s"), Id, bUp ? TEXT("up") : TEXT("down"));
		const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(Id, bUp));
		const RammsCurb::FCurbSchedule S = PlanAndLog(Test, In, *What);
		if (!Test.TestTrue(FString::Printf(TEXT("%s planned"), *What), S.IsValid()))
		{
			return;
		}
		PlanDryRunOf(Test, What, In, S);
	}
} // namespace

	#define RAMMS_CURB_DRY_RUN(Name, Id, bUp)                                                \
		IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbDryRun##Name, "Ramms.Curb.DryRun." #Name, \
			EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)        \
		bool FRammsCurbDryRun##Name::RunTest(const FString& Parameters)                      \
		{                                                                                    \
			PlanDryRun(*this, TEXT(Id), bUp);                                                \
			return true;                                                                     \
		}

RAMMS_CURB_DRY_RUN(SquareUp, "Square", true)
RAMMS_CURB_DRY_RUN(SquareDown, "Square", false)
RAMMS_CURB_DRY_RUN(Oblique30Up, "Oblique30", true)
RAMMS_CURB_DRY_RUN(Oblique30Down, "Oblique30", false)
RAMMS_CURB_DRY_RUN(ObliqueNeg30Up, "ObliqueNeg30", true)
RAMMS_CURB_DRY_RUN(ObliqueNeg30Down, "ObliqueNeg30", false)
RAMMS_CURB_DRY_RUN(RoadFallsUp, "RoadFalls", true)
RAMMS_CURB_DRY_RUN(RoadFallsDown, "RoadFalls", false)
RAMMS_CURB_DRY_RUN(StreetGradeDown, "StreetGrade", false)
RAMMS_CURB_DRY_RUN(SidewalkRisesUp, "SidewalkRises", true)
RAMMS_CURB_DRY_RUN(SidewalkRisesDown, "SidewalkRises", false)
RAMMS_CURB_DRY_RUN(SidewalkTwistedUp, "SidewalkTwisted", true)
RAMMS_CURB_DRY_RUN(SidewalkTwistedDown, "SidewalkTwisted", false)

	#undef RAMMS_CURB_DRY_RUN

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbDryRunRoadRisesTilted,
	"Ramms.Curb.DryRun.RoadRisesTilted",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbDryRunRoadRisesTilted::RunTest(const FString& Parameters)
{
	// A tilted plan dry-run with its tilt: the legs reach for their surfaces
	// from the tilted base, and the base the lagging support legs give stays
	// within 0.5 deg of the planned attitude.
	RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("RoadRises"), true));
	In.Tuning.MaxPlannedTiltDeg = 6.0f;
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("RoadRises up"));
	if (!TestEqual(TEXT("tilted"), FString(RammsCurb::PlanClassName(S.Class)), FString(TEXT("tilt"))))
	{
		return true;
	}
	PlanDryRunOf(*this, TEXT("RoadRises up"), In, S);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbDryRunPlannedStop,
	"Ramms.Curb.DryRun.PlannedStop",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbDryRunPlannedStop::RunTest(const FString& Parameters)
{
	// A plan with planned stops dry-run: the robot brakes onto each, rests while
	// the centre wheels and the chassis re-pose, and goes on -- no other stop,
	// nothing violated.
	RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	In.Tuning.MinSpeedFraction = 0.8f;
	In.Tuning.MaxPlannedTiltDeg = 0.0f;
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("Square up, floor 80 %"));
	if (!TestEqual(TEXT("a planned stop"), FString(RammsCurb::PlanClassName(S.Class)), FString(TEXT("planned stop"))))
	{
		return true;
	}
	double Dwell = 0.0;
	bool   bOnlyAtStops = true;
	for (const RammsCurb::FScheduleSample& Sm : S.Samples)
	{
		Dwell += Sm.bStop ? Sm.DwellSeconds : 0.0;
		bOnlyAtStops = bOnlyAtStops && (Sm.bStop || Sm.DwellSeconds == 0.0);
	}
	TestTrue(TEXT("a dwell only at a stop"), bOnlyAtStops);
	TestTrue(FString::Printf(TEXT("the stops rest a while (%.2f s)"), Dwell), Dwell > 0.0);
	PlanCheckRates(*this, In, S, TEXT("Square up, floor 80 %"));
	PlanDryRunOf(*this, TEXT("Square up, floor 80 %"), In, S);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerTiltProfileBlends,
	"Ramms.Curb.Planner.TiltProfileBlends",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerTiltProfileBlends::RunTest(const FString& Parameters)
{
	// Two stretches leaning the same way, closer than two easings: straight from
	// the larger to the smaller (no stop at the larger's level, no dip), pitch
	// and roll each within the rate and its change everywhere; level before
	// the first and after the last. Then too close for even that at the speed:
	// a gap named, and the speed that would let it.
	const double Rate = 2.0, Accel = 4.0;
	const int32	 N = 400;
	auto		 Run = [&](double Speed, TArray<FVector2D>& Out, TArray<RammsCurb::FTiltGap>& Tight, uint8 Mode = 0) {
		TArray<double> Seconds, S;
		for (int32 k = 0; k < N; ++k)
		{
			Seconds.Add(k / Speed);
			S.Add(k);
		}
		const RammsCurb::FTiltHold Holds[2] = { { 120, 165, FVector2D(3.5, -3.5) }, { 203, 260, FVector2D(1.25, -1.25) } };
		const uint8				   Modes[3] = { Mode, Mode, Mode };
		return RammsCurb::TiltProfile(Seconds, S, Holds, Rate, Accel, Out, &Tight, Modes);
	};
	// Pitch and roll each within the rate and its change at every sample, at
	// Speed; the largest rate of either axis and of the two together.
	auto Smooth = [&](const TArray<FVector2D>& T, double Speed, double* OutAxis = nullptr, double* OutTotal = nullptr) {
		const double D = 1.0 / Speed;
		bool		 bOk = true;
		double		 Axis = 0.0, Total = 0.0;
		for (int32 k = 1; k < T.Num(); ++k)
		{
			const FVector2D R = (T[k] - T[k - 1]) / D;
			Axis = FMath::Max(Axis, R.GetAbsMax());
			Total = FMath::Max(Total, R.Size());
			bOk = bOk && R.GetAbsMax() <= Rate * (1.0 + 1e-9) + 1e-9;
			if (k > 1)
			{
				bOk = bOk && ((T[k] - 2.0 * T[k - 1] + T[k - 2]) / (D * D)).GetAbsMax() <= Accel * (1.0 + 1e-9) + 1e-9;
			}
		}
		if (OutAxis)
		{
			*OutAxis = Axis;
		}
		if (OutTotal)
		{
			*OutTotal = Total;
		}
		return bOk;
	};
	TArray<FVector2D>			Tilt;
	TArray<RammsCurb::FTiltGap> Tight;
	TestTrue(TEXT("at 20 cm/s every easing fits"), Run(20.0, Tilt, Tight));
	TestEqual(TEXT("none too tight"), Tight.Num(), 0);
	int32 Bad = 0;
	for (int32 k = 0; k < N; ++k)
	{
		for (int32 a = 0; a < 2; ++a)
		{
			const double V = a == 0 ? Tilt[k].X : Tilt[k].Y;
			const double Sign = a == 0 ? 1.0 : -1.0;
			if (k >= 120 && k <= 165 && !FMath::IsNearlyEqual(V, 3.5 * Sign, 1e-12))
			{
				++Bad;
			}
			if (k >= 203 && k <= 260 && !FMath::IsNearlyEqual(V, 1.25 * Sign, 1e-12))
			{
				++Bad;
			}
			if (k > 165 && k < 203 && (FMath::Abs(V) < 1.25 - 1e-12 || FMath::Abs(V) > 3.5 + 1e-12))
			{
				++Bad;
			}
		}
	}
	TestEqual(TEXT("held exactly, and between the two levels between them"), Bad, 0);
	TestTrue(TEXT("within the rate and its change"), Smooth(Tilt, 20.0));
	TestTrue(TEXT("level at either end"), Tilt[0].IsNearlyZero(1e-12) && Tilt.Last().IsNearlyZero(1e-12));
	// Out of the larger early: already easing at the sample after it.
	TestTrue(FString::Printf(TEXT("easing out of the larger tilt as it ends (%.4f)"), Tilt[166].X), Tilt[166].X < 3.5 && Tilt[166].X > 3.4);
	// At 35 cm/s the 38 cm between them is 1.09 s, under the 1.63 s the change
	// takes.
	TestFalse(TEXT("at 35 cm/s it does not fit"), Run(35.0, Tilt, Tight));
	if (TestEqual(TEXT("one gap"), Tight.Num(), 1))
	{
		const double Need = RammsCurb::SCurveSeconds(2.25, Rate, Accel);
		TestEqual(TEXT("between the stretches"), Tight[0].First, 165);
		TestEqual(TEXT("to the next"), Tight[0].Last, 203);
		TestNearlyEqual(TEXT("needing the change's time"), Tight[0].NeedSeconds, Need, 1e-9);
		TestNearlyEqual(TEXT("and fitting at 38 cm over that"), Tight[0].CapCmS, 38.0 / Need, 1e-9);
	}
	// Not hurried but run into the stretches -- within the limits still:
	// centred on the gap (into both), ending as the later begins (into the
	// earlier only), beginning as the earlier ends (into the later only).
	TestTrue(TEXT("centred: within the limits"), Smooth(Tilt, 35.0));
	TestTrue(TEXT("centred: into both stretches"), Tilt[165].X < 3.5 - 1e-6 && Tilt[203].X > 1.25 + 1e-6);
	TArray<RammsCurb::FTiltGap> Ignored;
	Run(35.0, Tilt, Ignored, 1);
	TestTrue(TEXT("late: within the limits"), Smooth(Tilt, 35.0));
	TestTrue(TEXT("late: the later stretch whole, into the earlier"), FMath::IsNearlyEqual(Tilt[203].X, 1.25, 1e-12) && Tilt[165].X < 3.5 - 1e-6);
	Run(35.0, Tilt, Ignored, 2);
	TestTrue(TEXT("early: within the limits"), Smooth(Tilt, 35.0));
	TestTrue(TEXT("early: the earlier stretch whole, into the later"), FMath::IsNearlyEqual(Tilt[165].X, 3.5, 1e-12) && Tilt[203].X > 1.25 + 1e-6);

	// Between two tilts that turn pitch and roll different ways -- +3 / +1
	// then +1 / +3 -- the base goes along the straight way between them
	// (pitch plus roll stays 4), never leaning further than either; each axis
	// on its own S-curve, out of its larger early and into its larger late,
	// would dip towards +1 / +1 between them. The easing is timed by the axis
	// that changes most: a diagonal tilt eased in from level turns pitch and
	// roll each at the rate (the base, the two together, sqrt(2) times it).
	{
		TArray<double> Seconds, S;
		for (int32 k = 0; k < N; ++k)
		{
			Seconds.Add(k / 10.0);
			S.Add(k);
		}
		const RammsCurb::FTiltHold Holds[2] = { { 150, 180, FVector2D(3.0, 1.0) }, { 200, 230, FVector2D(1.0, 3.0) } };
		TArray<FVector2D>		   Way;
		RammsCurb::TiltProfile(Seconds, S, Holds, Rate, Accel, Way);
		bool   bStraight = true;
		double MostOff = 0.0;
		for (int32 k = 181; k < 200; ++k)
		{
			bStraight = bStraight && FMath::IsNearlyEqual(Way[k].X + Way[k].Y, 4.0, 1e-9);
			MostOff = FMath::Max(MostOff, RammsCurb::InclinationDeg(Way[k].X, Way[k].Y));
		}
		TestTrue(TEXT("between two tilts: along the straight way (pitch + roll 4)"), bStraight);
		TestTrue(FString::Printf(TEXT("between two tilts: inclined at most %.4f deg, either end %.4f"), MostOff, RammsCurb::InclinationDeg(3.0, 1.0)),
			MostOff <= RammsCurb::InclinationDeg(3.0, 1.0) + 1e-9);
		TestTrue(TEXT("between two tilts: within the rate and its change"), Smooth(Way, 10.0));
		const RammsCurb::FTiltHold Diag[1] = { { 150, 180, FVector2D(2.0, 2.0) } };
		TArray<FVector2D>		   In;
		RammsCurb::TiltProfile(Seconds, S, Diag, Rate, Accel, In);
		double Axis = 0.0, Total = 0.0;
		TestTrue(TEXT("diagonal: within the rate and its change"), Smooth(In, 10.0, &Axis, &Total));
		TestTrue(FString::Printf(TEXT("diagonal: pitch and roll each at the rate (%.4f), the two together at sqrt(2) times it (%.4f)"), Axis, Total),
			FMath::IsNearlyEqual(Axis, Rate, 0.01) && FMath::IsNearlyEqual(Total, Rate * UE_DOUBLE_SQRT_2, 0.02));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerTiltCandidatesOrder,
	"Ramms.Curb.Planner.TiltCandidatesOrder",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerTiltCandidatesOrder::RunTest(const FString& Parameters)
{
	// The tilts of one size are one inclination -- the base's total tilt, not
	// its larger axis -- on a ring about the grid apart along it: the four
	// along the axes first, then by how far they lean from the nearest axis,
	// the diagonals last.
	const TArray<FVector2D> C = RammsCurb::TiltCandidates(1.0, 0.25);
	// A quarter of the ring is pi/2 x 1.0 deg long: ceil(6.28) = 7 steps.
	TestEqual(TEXT("the ring about 0.25 deg apart"), C.Num(), 28);
	bool bOnRing = true, bUnique = true;
	for (int32 i = 0; i < C.Num(); ++i)
	{
		bOnRing = bOnRing && FMath::IsNearlyEqual(RammsCurb::InclinationDeg(C[i].X, C[i].Y), 1.0, 1e-9);
		for (int32 j = 0; j < i; ++j)
		{
			bUnique = bUnique && !C[i].Equals(C[j], 1e-9);
		}
	}
	TestTrue(TEXT("every one inclines the base 1.0 deg"), bOnRing);
	TestTrue(TEXT("no tilt twice"), bUnique);
	bool bAxesFirst = C.Num() >= 4;
	for (int32 i = 0; i < 4 && i < C.Num(); ++i)
	{
		bAxesFirst = bAxesFirst && FMath::Min(FMath::Abs(C[i].X), FMath::Abs(C[i].Y)) == 0.0 && FMath::Max(FMath::Abs(C[i].X), FMath::Abs(C[i].Y)) == 1.0;
	}
	TestTrue(TEXT("along the axes first, exactly"), bAxesFirst);
	// How far each leans from the nearest axis (deg of azimuth, in tan space).
	auto FromAxis = [](const FVector2D& D) {
		const double A = FMath::RadiansToDegrees(FMath::Atan2(FMath::Abs(FMath::Tan(FMath::DegreesToRadians(D.Y))), FMath::Abs(FMath::Tan(FMath::DegreesToRadians(D.X)))));
		return FMath::Min(A, 90.0 - A);
	};
	bool bOrdered = true;
	for (int32 i = 1; i < C.Num(); ++i)
	{
		bOrdered = bOrdered && FromAxis(C[i]) >= FromAxis(C[i - 1]) - 1e-9;
	}
	TestTrue(TEXT("never nearer an axis than the one before"), bOrdered);
	TestTrue(FString::Printf(TEXT("the last within half a step of a diagonal (%.2f deg from an axis)"), FromAxis(C.Last())),
		C.Num() > 0 && FromAxis(C.Last()) >= 45.0 - 0.5 * 90.0 / 7.0 - 1e-9);
	// A larger ring has more of them, still about the grid apart.
	TestEqual(TEXT("5 deg: 4 x ceil(31.4)"), RammsCurb::TiltCandidates(5.0, 0.25).Num(), 128);
	TestEqual(TEXT("nothing at 0"), RammsCurb::TiltCandidates(0.0, 0.25).Num(), 0);
	// A tilt made larger keeps the way it leans.
	const FVector2D Larger = RammsCurb::TiltAtInclination(FVector2D(1.0, -2.0), 4.0);
	TestTrue(FString::Printf(TEXT("made 4 deg: (%+.4f, %+.4f)"), Larger.X, Larger.Y),
		FMath::IsNearlyEqual(RammsCurb::InclinationDeg(Larger.X, Larger.Y), 4.0, 1e-9) && Larger.X > 0.0 && Larger.Y < 0.0
			&& FMath::IsNearlyEqual(FMath::Tan(FMath::DegreesToRadians(Larger.Y)) / FMath::Tan(FMath::DegreesToRadians(Larger.X)),
				FMath::Tan(FMath::DegreesToRadians(-2.0)) / FMath::Tan(FMath::DegreesToRadians(1.0)), 1e-9));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerTiltEscalation,
	"Ramms.Curb.Planner.TiltEscalation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerTiltEscalation::RunTest(const FString& Parameters)
{
	// The tilt search's order (SearchTilts): every try by the inclination it
	// comes to, least first; of one size a least tilt before one made larger;
	// a candidate's escalations only once its least tilt's plan has failed; a
	// tilt set planned once never again; none over the limit.
	using namespace RammsCurb;
	const double Steps[] = { 0.25, 0.5, 1.0 };
	auto		 AllStand = [](int32, int32, const FVector2D&) { return true; };
	auto		 Name = [](const FTiltTry& T) { return FString::Printf(TEXT("%d+%.2f"), T.Candidate, T.More); };
	auto		 Join = [](const TArray<FString>& A) { return FString::Join(A, TEXT(" ")); };
	// Two candidates, one stretch each, whose least tilts incline the base 2.0
	// and 2.25 deg. The first verifies only 0.5 larger, the second at its
	// least: the second's 2.25 is taken -- before the first's 2.0 made 0.25
	// larger, which comes to the same.
	{
		const TArray<FTiltSearchCandidate> C = { { 0, ETiltPick::Ring, { FVector2D(2.0, 0.0) } }, { 1, ETiltPick::Ring, { FVector2D(0.0, 2.25) } } };
		TArray<FString>					   Order;
		const FTiltSearchResult			   R = SearchTilts(C, 3.5, 0.25, Steps, 99, AllStand, [&](const FTiltTry& T, TArray<int32>& Failing) {
			   Order.Add(Name(T));
			   Failing = { 0 };
			   return T.Candidate == 1 ? T.More == 0.0 : T.More >= 0.5;
		   });
		TestTrue(TEXT("the second candidate's least tilt is taken"), R.bFound && R.Planned.Candidate == 1 && R.Planned.More == 0.0 && R.Planned.Size == 2.25);
		TestEqual(TEXT("planned after the first's least tilt only"), Join(Order), FString(TEXT("0+0.00 1+0.00")));
	}
	// A candidate whose least tilt is far larger comes after the others made
	// larger to less than it: 1.0 deg made 0.25, 0.5 and 1.0 larger is tried
	// before another candidate's 3.0; at a limit of 3.5 that one's 1.0 larger
	// is cut (named), and a tilt set two ways agree on is planned once.
	{
		const TArray<FTiltSearchCandidate> C = { { 0, ETiltPick::Ring, { FVector2D(1.0, 0.0) } }, { 1, ETiltPick::Ring, { FVector2D(3.0, 0.0) } } };
		TArray<FString>					   Order;
		const FTiltSearchResult			   R = SearchTilts(C, 3.5, 0.25, Steps, 99, AllStand, [&](const FTiltTry& T, TArray<int32>& Failing) {
			   Order.Add(Name(T));
			   Failing = { 0 };
			   return false;
		   });
		TestFalse(TEXT("nothing verifies"), R.bFound);
		TestEqual(TEXT("by the inclination they come to, each once"), Join(Order), FString(TEXT("0+0.00 0+0.25 0+0.50 0+1.00 1+0.00 1+0.25 1+0.50")));
		TestEqual(TEXT("seven plans"), R.Plans, 7);
		TestNearlyEqual(TEXT("the least tilt over the limit not tried: 4.0"), R.CutDeg, 4.0, 1e-9);
		TestFalse(TEXT("not cut short"), R.bCapped);
	}
	// A least tilt over the limit is not tried, and named.
	{
		const TArray<FTiltSearchCandidate> C = { { 0, ETiltPick::Ring, { FVector2D(0.0, -4.0) } } };
		int32							   Plans = 0;
		const FTiltSearchResult			   R = SearchTilts(C, 3.5, 0.25, Steps, 99, AllStand, [&](const FTiltTry&, TArray<int32>&) { return ++Plans < 0; });
		TestTrue(TEXT("over the limit: no plan, 4.0 named"), !R.bFound && Plans == 0 && FMath::IsNearlyEqual(R.CutDeg, 4.0, 1e-9));
	}
	// What a candidate's escalation comes to: its stretches' inclinations 1.5,
	// 2.75 and 0.25, the first failing. Made 1.0 larger it is still the 2.75 of
	// the second stretch -- tried at that size, before another candidate's
	// 3.0; made 1.5 larger it comes to 3.0, after that candidate's least tilt.
	{
		const double					   Big[] = { 1.0, 1.5 };
		const TArray<FTiltSearchCandidate> C = { { 0, ETiltPick::Ring, { FVector2D(1.5, 0.0), FVector2D(0.0, 2.75), FVector2D(0.25, 0.0) } },
			{ 1, ETiltPick::Ring, { FVector2D(3.0, 0.0) } } };
		TArray<FString>					   Order;
		TArray<double>					   Sizes;
		SearchTilts(C, 5.0, 0.25, Big, 99, AllStand, [&](const FTiltTry& T, TArray<int32>& Failing) {
			Order.Add(Name(T));
			Sizes.Add(T.Size);
			Failing = { 0 };
			return false;
		});
		TestEqual(TEXT("the least, its 2.75 made larger, the other's 3.0, then its 3.0, then the other's made larger"), Join(Order),
			FString(TEXT("0+0.00 0+1.00 1+0.00 0+1.50 1+1.00 1+1.50")));
		TestTrue(TEXT("at 2.75, 2.75, 3.0, 3.0, 4.0 and 4.5"), Sizes.Num() == 6 && FMath::IsNearlyEqual(Sizes[1], 2.75, 1e-9) && FMath::IsNearlyEqual(Sizes[3], 3.0, 1e-9) && FMath::IsNearlyEqual(Sizes[4], 4.0, 1e-9) && FMath::IsNearlyEqual(Sizes[5], 4.5, 1e-9));
	}
	// Two picks of one candidate plan, both verifying: the square's 3.55 deg
	// (+2.25 / +2.75) goes before the ring's 3.75, a fifth of a degree more;
	// a square's 3.748 (-3.0 / +2.25) is alike to the ring's 3.75, and the
	// ring's goes first.
	{
		const FVector2D					   Ring375 = TiltAtInclination(FVector2D(1.0, 1.7), 3.75);
		const TArray<FTiltSearchCandidate> C = { { 0, ETiltPick::Ring, { Ring375 } }, { 0, ETiltPick::Square, { FVector2D(2.25, 2.75) } } };
		const FTiltSearchResult			   R = SearchTilts(C, 5.0, 0.25, Steps, 99, AllStand, [&](const FTiltTry&, TArray<int32>&) { return true; });
		TestTrue(FString::Printf(TEXT("the square's 3.55 first (%.3f)"), R.Planned.Size), R.bFound && R.Planned.Candidate == 1);
		const TArray<FTiltSearchCandidate> D = { { 0, ETiltPick::Ring, { Ring375 } }, { 0, ETiltPick::Square, { FVector2D(-3.0, 2.25) } } };
		const FTiltSearchResult			   Q = SearchTilts(D, 5.0, 0.25, Steps, 99, AllStand, [&](const FTiltTry&, TArray<int32>&) { return true; });
		TestTrue(FString::Printf(TEXT("a square's 3.748 alike to the ring's 3.75: the ring's first (%.4f)"), Q.Planned.Size), Q.bFound && Q.Planned.Candidate == 0);
	}
	// At most MaxPlans larger tries, the least inclined first, and saying so;
	// every candidate's least tilt is planned regardless -- a far larger one
	// that verifies is found past the cap, never left for a planned stop.
	{
		const TArray<FTiltSearchCandidate> C = { { 0, ETiltPick::Ring, { FVector2D(1.0, 0.0) } }, { 1, ETiltPick::Ring, { FVector2D(3.0, 0.0) } } };
		TArray<FString>					   Order;
		const FTiltSearchResult			   R = SearchTilts(C, 3.5, 0.25, Steps, 2, AllStand, [&](const FTiltTry& T, TArray<int32>& Failing) {
			   Order.Add(Name(T));
			   Failing = { 0 };
			   return false;
		   });
		TestTrue(TEXT("two larger tries, then cut short"), !R.bFound && R.Plans == 4 && R.bCapped);
		TestEqual(TEXT("the two least inclined larger ones, and every least tilt"), Join(Order), FString(TEXT("0+0.00 0+0.25 0+0.50 1+0.00")));
		const FTiltSearchResult Late = SearchTilts(C, 3.5, 0.25, Steps, 2, AllStand, [&](const FTiltTry& T, TArray<int32>& Failing) {
			Failing = { 0 };
			return T.Candidate == 1 && T.More == 0.0;
		});
		TestTrue(TEXT("a least tilt past the cap that verifies is taken"), Late.bFound && Late.Planned.Candidate == 1 && Late.Planned.More == 0.0);
	}

	// A stretch made larger takes a tilt of its larger inclination that keeps
	// a pose, not its found tilt made larger: one that keeps a pose wherever
	// the roll is at least 3 deg, found at (+1.0, +2.57) on the ring of 2.75,
	// is (0, +3.0) at 3.0 -- not (+1.09, +2.80).
	{
		FVector2D  At = FVector2D::ZeroVector;
		const bool bFound = RammsCurb::LeastInclinationAt(3.0, 0.25, [](const FVector2D& D) { return D.Y >= 3.0 - 1e-9; }, At);
		TestTrue(FString::Printf(TEXT("the least inclination at 3.0 deg: (%+.2f, %+.2f)"), At.X, At.Y), bFound && At.Equals(FVector2D(0.0, 3.0), 1e-9));
		TestFalse(TEXT("none when nothing of that size keeps a pose"),
			RammsCurb::LeastInclinationAt(3.0, 0.25, [](const FVector2D&) { return false; }, At));
		// Of the ring's tilts that keep a pose, the one furthest from one that
		// does not: where pitch and roll must both be at least 1 deg, the arc
		// that stands runs from about 20 to 70 deg round the ring, and the
		// tilt taken is its middle, near the diagonal -- not the first of the
		// arc in the ring's order (about +2.75 / +1.2).
		const bool bArc = RammsCurb::LeastInclinationAt(3.0, 0.25, [](const FVector2D& D) { return D.X >= 1.0 && D.Y >= 1.0; }, At);
		TestTrue(FString::Printf(TEXT("the middle of the arc that stands: (%+.2f, %+.2f)"), At.X, At.Y),
			bArc && At.X >= 1.0 && At.Y >= 1.0 && FMath::Abs(At.X - At.Y) < 0.25 && FMath::IsNearlyEqual(RammsCurb::InclinationDeg(At.X, At.Y), 3.0, 1e-9));
		// A ring that stands all round: the first, along an axis.
		RammsCurb::LeastInclinationAt(3.0, 0.25, [](const FVector2D&) { return true; }, At);
		TestTrue(FString::Printf(TEXT("all round: the first (%+.2f, %+.2f)"), At.X, At.Y), At.Equals(FVector2D(3.0, 0.0), 1e-9));
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerTiltPicks,
	"Ramms.Curb.Planner.TiltPicks",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerTiltPicks::RunTest(const FString& Parameters)
{
	// The square of a larger axis: the four along the axes first, then
	// leaning further from them a grid step of the smaller axis at a time,
	// the corners last; every one that large on its larger axis.
	using namespace RammsCurb;
	const TArray<FVector2D> Sq = SquareCandidates(1.0, 0.25);
	TestEqual(TEXT("4 + 3 x 8 + 4 of them"), Sq.Num(), 32);
	bool bSize = true, bOrder = true;
	for (int32 i = 0; i < Sq.Num(); ++i)
	{
		bSize = bSize && PickSize(ETiltPick::Square, Sq[i]) == 1.0;
		bOrder = bOrder && (i == 0 || FMath::Min(FMath::Abs(Sq[i].X), FMath::Abs(Sq[i].Y)) >= FMath::Min(FMath::Abs(Sq[i - 1].X), FMath::Abs(Sq[i - 1].Y)));
	}
	TestTrue(TEXT("each 1.0 on its larger axis"), bSize);
	TestTrue(TEXT("never nearer an axis than the one before"), bOrder);
	TestTrue(TEXT("along the axes first"), Sq.Num() > 3 && Sq[0] == FVector2D(1.0, 0.0) && Sq[3] == FVector2D(0.0, -1.0));

	// Where a stretch keeps a pose only pitched 1.8 deg nose down or more and
	// rolled 1.5 or more: the square's least is -2.0 / +1.5 (its larger axis
	// 2.0, 2.4993 deg of inclination) -- a grid value of each axis -- and the
	// ring's least on the ring of 2.5, the middle of its arc that stands.
	auto	   Stands = [](const FVector2D& D) { return D.X <= -1.8 && D.Y >= 1.5; };
	FVector2D  Square, Ring;
	const bool bSquare = PickLeastTilt(ETiltPick::Square, 0.25, 5.0, Stands, Square);
	const bool bRing = PickLeastTilt(ETiltPick::Ring, 0.25, 5.0, Stands, Ring);
	TestTrue(FString::Printf(TEXT("the square's least: (%+.3f, %+.3f)"), Square.X, Square.Y), bSquare && Square == FVector2D(-2.0, 1.5));
	TestTrue(FString::Printf(TEXT("the ring's least: (%+.3f, %+.3f), %.3f deg"), Ring.X, Ring.Y, InclinationDeg(Ring.X, Ring.Y)),
		bRing && Stands(Ring) && FMath::IsNearlyEqual(InclinationDeg(Ring.X, Ring.Y), 2.5, 1e-9));
	TestNearlyEqual(TEXT("sizes: the square's its larger axis"), PickSize(ETiltPick::Square, Square), 2.0, 1e-12);
	TestNearlyEqual(TEXT("sizes: the ring's its inclination"), PickSize(ETiltPick::Ring, Ring), 2.5, 1e-9);
	// The square never takes a tilt inclining the base over the limit: at 2.4
	// its 2.0 has none (-2.0 / +1.5 inclines it 2.4993).
	FVector2D Out;
	TestFalse(TEXT("none within a 2.4 deg limit"), PickTiltAt(ETiltPick::Square, 2.0, 0.25, 2.4, Stands, Out));
	// Made larger, each keeps the way it leans: the square by its larger axis,
	// the ring by its inclination.
	TestTrue(TEXT("the square's made 2.5 on its larger axis"), ScaledTilt(ETiltPick::Square, Square, 2.5).Equals(FVector2D(-2.5, 1.875), 1e-12));
	const FVector2D R3 = ScaledTilt(ETiltPick::Ring, Ring, 3.0);
	TestTrue(TEXT("the ring's made 3.0 deg of inclination"), FMath::IsNearlyEqual(InclinationDeg(R3.X, R3.Y), 3.0, 1e-9) && R3.X < 0.0 && R3.Y > 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerTiltSearchSharedRoll,
	"Ramms.Curb.Planner.TiltSearchSharedRoll",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerTiltSearchSharedRoll::RunTest(const FString& Parameters)
{
	// Two stretches whose plan verifies only when they share their roll (the
	// easing between them turning the pitch alone): the first keeps a pose
	// rolled 1.5 deg or more and not pitched nose up, the second only pitched
	// 1.8 nose down or more and rolled 1.5 or more. Each stretch's least by
	// the ring -- (0, +1.5) and the middle of the second's arc on the ring of
	// 2.5, rolled about +1.59 -- do not share it, nor any of those made
	// larger; the square's -- (0, +1.5) and (-2.0, +1.5) -- do. The search
	// tries both picks, the least inclined first (by the 0.25 deg step the
	// inclination lies in; of one step the ring's first), and takes the
	// square's: 2.4993 deg, in the second plan.
	using namespace RammsCurb;
	const double Steps[] = { 0.25, 0.5, 1.0 };
	auto		 Stands = [](int32 Stretch, const FVector2D& D) {
		return Stretch == 0 ? D.Y >= 1.5 - 1e-12 && D.X <= 1e-12 : D.X <= -1.8 && D.Y >= 1.5;
	};
	auto Least = [&](ETiltPick Pick) {
		FTiltSearchCandidate C;
		C.Pick = Pick;
		for (int32 r = 0; r < 2; ++r)
		{
			FVector2D T = FVector2D::ZeroVector;
			TestTrue(TEXT("each stretch has a least tilt"), PickLeastTilt(Pick, 0.25, 3.0, [&](const FVector2D& D) { return Stands(r, D); }, T));
			C.Tilts.Add(T);
		}
		return C;
	};
	const FTiltSearchCandidate ByRing = Least(ETiltPick::Ring), BySquare = Least(ETiltPick::Square);
	auto					   Verifies = [&](const FTiltTry& T, TArray<int32>& Failing) {
		  Failing = { 0, 1 };
		  return Stands(0, T.Tilts[0]) && Stands(1, T.Tilts[1]) && FMath::Abs(T.Tilts[0].Y - T.Tilts[1].Y) < 1e-9;
	};
	auto							   StandsC = [&](int32, int32 r, const FVector2D& D) { return Stands(r, D); };
	const TArray<FTiltSearchCandidate> RingOnly = { ByRing };
	const FTiltSearchResult			   A = SearchTilts(RingOnly, 3.0, 0.25, Steps, 99, StandsC, Verifies);
	TestFalse(FString::Printf(TEXT("by the ring alone, no plan within 3 deg (%d tried)"), A.Plans), A.bFound);
	const TArray<FTiltSearchCandidate> Both = { ByRing, BySquare };
	const FTiltSearchResult			   B = SearchTilts(Both, 3.0, 0.25, Steps, 99, StandsC, Verifies);
	const FVector2D					   P0 = B.Planned.Tilts.Num() > 0 ? B.Planned.Tilts[0] : FVector2D::ZeroVector;
	const FVector2D					   P1 = B.Planned.Tilts.Num() > 1 ? B.Planned.Tilts[1] : FVector2D::ZeroVector;
	TestTrue(FString::Printf(TEXT("by both: the square's, sharing its roll (%+.2f/%+.2f, %+.2f/%+.2f)"), P0.X, P0.Y, P1.X, P1.Y),
		B.bFound && B.Planned.Candidate == 1 && B.Planned.More == 0.0);
	TestTrue(FString::Printf(TEXT("at %.4f deg, in the second plan (%d), after the ring's of the same step"), B.Planned.Size, B.Plans),
		FMath::IsNearlyEqual(B.Planned.Size, InclinationDeg(-2.0, 1.5), 1e-9) && B.Plans == 2);
	TestNearlyEqual(TEXT("2.4993 deg is alike to 2.5 (TiltOrderDeg)"), TiltStep(B.Planned.Size, TiltOrderDeg), 2.5, 1e-12);
	TestNearlyEqual(TEXT("2.5 is in its own step"), TiltStep(2.5, TiltOrderDeg), 2.5, 1e-12);
	TestTrue(TEXT("3.55 goes before 3.75"), TiltStep(3.55, TiltOrderDeg) < TiltStep(3.75, TiltOrderDeg));
	TestTrue(TEXT("2.66 goes before 2.75"), TiltStep(2.66, TiltOrderDeg) < TiltStep(2.75, TiltOrderDeg));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerRollingEntry,
	"Ramms.Curb.Planner.RollingEntry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerRollingEntry::RunTest(const FString& Parameters)
{
	// Rolling in at 35 cm/s with a long level-in (the centre wheels held 20 cm
	// up in the drive stance): the plan starts at the entry speed and asks no
	// harder braking than Deceleration -- the level-in's cap is reached on the
	// braking curve, not at once -- and its dry run, from 35 cm/s, keeps it.
	RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	for (const int32 Leg : { PlanCL, PlanCR })
	{
		In.Legs[Leg].Measured.Y += 20.0;
		In.Legs[Leg].bOffGround = true;
	}
	In.EntrySpeedCmS = 35.0;
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("Square up, rolling in"));
	if (!TestTrue(TEXT("planned"), S.IsValid()))
	{
		return true;
	}
	TestNearlyEqual(TEXT("from the entry speed"), S.Samples[0].V, 35.0, 1e-6);
	const double Dec = In.Tuning.Deceleration;
	bool		 bBrake = true;
	bool		 bCapped = false;
	for (const RammsCurb::FScheduleSample& Sm : S.Samples)
	{
		bBrake = bBrake && Sm.V >= FMath::Sqrt(FMath::Max(0.0, 35.0 * 35.0 - 2.0 * Dec * Sm.S)) - 0.01;
		bCapped = bCapped || (Sm.S <= S.LevelFromS && Sm.V < 35.0 - 1.0);
	}
	TestTrue(TEXT("never slower than braking at Deceleration from it"), bBrake);
	TestTrue(TEXT("and slowed for the level-in"), bCapped);
	// The level-in: as long as it takes, unless the floor holds it shorter.
	double Seconds = 0.0;
	for (int32 k = 1; k < S.Samples.Num() && k <= FMath::RoundToInt(S.LevelFromS / S.SampleCm); ++k)
	{
		Seconds += S.SampleCm / FMath::Max(0.5, 0.5 * (S.Samples[k].V + S.Samples[k - 1].V));
	}
	TestTrue(FString::Printf(TEXT("the level-in has its %.2f s (%.2f)"), S.LevelInSeconds, Seconds), Seconds >= S.LevelInSeconds - 0.02);
	PlanDryRunOf(*this, TEXT("Square up, rolling in"), In, S);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerCrankRateLengthensLift,
	"Ramms.Curb.Planner.CrankRateLengthensLift",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerCrankRateLengthensLift::RunTest(const FString& Parameters)
{
	// Corners whose cranks may turn only 0.6 rad/s (well under LegSpeed 30 on
	// the made-up 4-bar): their lifts begin earlier and their landings take
	// longer than with no limit.
	const RammsCurb::FPlannerInput Free = PlanOn(PlanScenario(TEXT("Square"), true));
	RammsCurb::FPlannerInput	   Slow = Free;
	for (const int32 Leg : { PlanFL, PlanFR, PlanRL, PlanRR })
	{
		Slow.Legs[Leg].MaxCrankRateRadS = 0.6;
	}
	// (Timed whether or not the slower legs still make a plan.)
	const RammsCurb::FCurbSchedule A = RammsCurb::PlanCrossing(Free);
	const RammsCurb::FCurbSchedule B = PlanAndLog(*this, Slow, TEXT("Square up, cranks at 0.6 rad/s"));
	if (!TestTrue(TEXT("both timed"), A.Wheels.Num() == 6 && B.Wheels.Num() == 6))
	{
		return true;
	}
	for (const int32 Leg : { PlanFL, PlanRL })
	{
		TestTrue(FString::Printf(TEXT("%s lifts longer (%.2f s against %.2f)"), PlanLegName[Leg], B.Wheels[Leg].TLift, A.Wheels[Leg].TLift),
			B.Wheels[Leg].TLift > A.Wheels[Leg].TLift + 0.05);
		TestTrue(FString::Printf(TEXT("%s lands longer (%.2f s against %.2f)"), PlanLegName[Leg], B.Wheels[Leg].TPlant, A.Wheels[Leg].TPlant),
			B.Wheels[Leg].TPlant > A.Wheels[Leg].TPlant + 0.01);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerDriveStanceChecked,
	"Ramms.Curb.Planner.DriveStanceChecked",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerDriveStanceChecked::RunTest(const FString& Parameters)
{
	// A drive stance with its rear corners up and its centre wheels a little
	// aft: it keeps more standing still than braking or speeding up. With
	// MinStabilityMargin between the two: standing still, the robot levels in
	// first, before it moves; rolling, at once as it rolls on (the stance over
	// MinLiveStabilityMarginCm); with that floor between the two too, at once
	// holding its speed; with it over both, refused, naming the stance. With
	// room to spare, none of these.
	RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	for (const int32 Leg : { PlanRL, PlanRR })
	{
		In.Legs[Leg].Measured.Y += 8.0;
		In.Legs[Leg].bOffGround = true;
	}
	for (const int32 Leg : { PlanCL, PlanCR })
	{
		In.Legs[Leg].Measured.X = -6.0;
	}
	const RammsCurb::FPlanModel M(In);
	TArray<FVector2D>			XZ;
	uint32						Support = 0;
	for (int32 i = 0; i < 6; ++i)
	{
		XZ.Add(In.Legs[i].Measured);
		Support |= (In.Legs[i].bOffGround ? 0u : 1u) << i;
	}
	const double Dynamic = M.PoseMarginAt(XZ, Support, 0, 0.0, In.OriginZ, 0.0, 0.0, true);
	const double Static = M.PoseMarginAt(XZ, Support, 0, 0.0, In.OriginZ, 0.0, 0.0, false);
	if (!TestTrue(FString::Printf(TEXT("the stance keeps more standing (%.2f cm) than braking or speeding up (%.2f)"), Static, Dynamic),
			Static > Dynamic + 0.2 && Dynamic > In.Tuning.MinLiveStabilityMarginCm))
	{
		return true;
	}
	In.Tuning.MinStabilityMargin = static_cast<float>(0.5 * (Static + Dynamic));
	const RammsCurb::FCurbSchedule Rest = PlanAndLog(*this, In, TEXT("from a marginal stance, standing"));
	TestTrue(TEXT("standing: levels in standing still"), Rest.bLevelInAtRest && Rest.LevelFromS == 0.0);
	TestFalse(TEXT("and is not refused for the stance"), Rest.Summary.Contains(TEXT("drive stance")));
	TestNearlyEqual(TEXT("its margin reported"), Rest.ApproachMarginCm, Dynamic, 1e-9);
	In.EntrySpeedCmS = 20.0;
	const RammsCurb::FCurbSchedule Rolling = PlanAndLog(*this, In, TEXT("from a marginal stance, rolling"));
	TestTrue(TEXT("rolling: levels in at once"), Rolling.bLevelInAtOnce && !Rolling.bLevelInAtRest && !Rolling.bHoldSpeedThroughLevelIn);
	TestFalse(TEXT("and is not refused for the stance"), Rolling.Summary.Contains(TEXT("drive stance")));
	In.Tuning.MinLiveStabilityMarginCm = static_cast<float>(0.5 * (Static + Dynamic));
	In.Tuning.MinStabilityMargin = static_cast<float>(Static + 0.1);
	const RammsCurb::FCurbSchedule Hold = PlanAndLog(*this, In, TEXT("from a marginal stance, rolling, steady only"));
	TestTrue(TEXT("rolling with the floor at a steady speed only: levels in at once, holding its speed"), Hold.bHoldSpeedThroughLevelIn);
	bool bHeld = true;
	for (const RammsCurb::FScheduleSample& Sm : Hold.Samples)
	{
		bHeld = bHeld && (Sm.S > Hold.LevelFromS + 1e-9 || FMath::IsNearlyEqual(Sm.V, 20.0, 0.01));
	}
	TestTrue(TEXT("its speed held through the level-in"), Hold.IsValid() ? bHeld : true);
	In.Tuning.MinLiveStabilityMarginCm = static_cast<float>(Static + 0.1);
	const RammsCurb::FCurbSchedule No = PlanAndLog(*this, In, TEXT("from a stance under the floor, rolling"));
	TestEqual(TEXT("under the floor, rolling: refused"), FString(RammsCurb::PlanClassName(No.Class)), FString(TEXT("refused")));
	TestTrue(TEXT("naming the stance and MinLiveStabilityMarginCm"), No.Summary.Contains(TEXT("drive stance")) && No.Summary.Contains(TEXT("MinLiveStabilityMarginCm")));
	const RammsCurb::FCurbSchedule Ample = RammsCurb::PlanCrossing(PlanOn(PlanScenario(TEXT("Square"), true)));
	TestFalse(TEXT("a stance with room to spare levels in on the way"), Ample.bLevelInAtRest || Ample.bLevelInAtOnce || Ample.bHoldSpeedThroughLevelIn);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPlannerPlannedStopRePoses,
	"Ramms.Curb.Planner.PlannedStopRePoses",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbPlannerPlannedStopRePoses::RunTest(const FString& Parameters)
{
	// Centre wheels that walk only 2 cm/s, no tilt: rolling, they cannot get
	// where the crossing needs them in time, so the robot stops, rests while
	// they re-pose -- further than they could walk at any speed the plan rolls
	// at -- and goes on. The rest lasts as long as the walk; the dry run makes
	// it.
	RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), false));
	In.Tuning.CenterShiftSpeed = 2.0f;
	In.Tuning.MaxPlannedTiltDeg = 0.0f;
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("Square down, slow centre wheels"));
	if (!TestEqual(TEXT("a planned stop"), FString(RammsCurb::PlanClassName(S.Class)), FString(TEXT("planned stop"))))
	{
		return true;
	}
	double Most = 0.0, Dwell = 0.0;
	for (int32 k = 0; k + 1 < S.Samples.Num(); ++k)
	{
		const RammsCurb::FScheduleSample& A = S.Samples[k];
		const RammsCurb::FScheduleSample& B = S.Samples[k + 1];
		if (!A.bStop)
		{
			continue;
		}
		const double Move = FMath::Max(FMath::Abs(B.X[0] - A.X[0]), FMath::Abs(B.X[1] - A.X[1]));
		Most = FMath::Max(Most, Move);
		Dwell += A.DwellSeconds;
		TestTrue(FString::Printf(TEXT("the rest at %.0f cm lasts the walk (%.2f s for %.1f cm)"), A.S, A.DwellSeconds, Move),
			A.DwellSeconds >= Move / In.Tuning.CenterShiftSpeed - 1e-9);
	}
	// Rolling 1 cm at the floor speed or more, a wheel walks under 0.3 cm.
	TestTrue(FString::Printf(TEXT("the centre wheels re-pose at rest (%.1f cm)"), Most), Most >= 5.0);
	PlanCheckSamples(*this, In, S, TEXT("Square down, slow centre wheels"));
	PlanCheckRates(*this, In, S, TEXT("Square down, slow centre wheels"));
	const RammsCurb::FDryRunResult R = PlanDryRunOf(*this, TEXT("Square down, slow centre wheels"), In, S);
	TestTrue(FString::Printf(TEXT("the dry run rests while they walk (%.1f s, the plan %.1f s)"), R.PlannedDwellSeconds, Dwell),
		R.PlannedDwellSeconds >= 0.9 * (Dwell - S.PlannedStops * (In.Tuning.ServoLagSeconds + RammsCurb::LegSettleSeconds)));
	return true;
}

// --- the executor (Private/Curb/RammsCurbExecutor), closed round a kinematic robot -----

namespace
{
	/** What every executor run keeps: done, never under the floor once risen,
	 *  never stopped, never held; no wheel on the ground in its window, no
	 *  lifted wheel into the terrain, no forbidden set off the ground; the robot
	 *  standing; the base within a degree of the plan. */
	RammsCurb::FExecDryRunResult ExecRunOf(FAutomationTestBase& Test, const FString& What, const RammsCurb::FPlannerInput& In,
		const RammsCurb::FCurbSchedule& S, const RammsCurb::FExecDryRunOptions& Options = RammsCurb::FExecDryRunOptions(),
		bool bExpectFluid = true)
	{
		RammsCurb::FExecDryRunOptions Traced = Options;
		Traced.TraceEvery = 6;
		const RammsCurb::FExecDryRunResult R = RammsCurb::ExecDryRun(In, S, Traced);
		for (const FString& N : R.Notes)
		{
			Test.AddInfo(FString::Printf(TEXT("%s: %s"), *What, *N));
		}
		const double Floor = In.Tuning.MinSpeedFraction * In.Tuning.DriveSpeed;
		Test.TestEqual(FString::Printf(TEXT("%s: wheels on the ground inside their windows"), *What), R.KeepOffViolations, 0);
		Test.TestEqual(FString::Printf(TEXT("%s: lifted wheels into the terrain"), *What), R.IntoTerrain, 0);
		Test.TestEqual(FString::Printf(TEXT("%s: ticks with a forbidden set off the ground"), *What), R.Counts.ForbiddenTicks, 0);
		Test.TestEqual(FString::Printf(TEXT("%s: ticks with a loaded wheel in its window"), *What), R.Counts.WindowTicks, 0);
		Test.TestEqual(FString::Printf(TEXT("%s: lifts before the wheels they wait on had landed"), *What), R.EarlyLifts, 0);
		Test.TestTrue(FString::Printf(TEXT("%s: the robot stands (least margin %.2f cm)"), *What, R.MinMarginCm),
			R.MinMarginCm >= In.Tuning.MinLiveStabilityMarginCm);
		if (bExpectFluid)
		{
			Test.TestTrue(FString::Printf(TEXT("%s: done (%.1f s, ended %s)"), *What, R.Seconds, RammsCurb::ExecPhaseName(R.EndPhase)), R.bDone);
			Test.TestTrue(FString::Printf(TEXT("%s: never under the floor once risen (%.2f cm/s)"), *What, R.MinSpeedCmS), R.MinSpeedCmS >= Floor - 1e-6);
			Test.TestEqual(FString::Printf(TEXT("%s: stops"), *What), R.Stops, 0);
			Test.TestEqual(FString::Printf(TEXT("%s: holds"), *What), R.Counts.Holds, 0);
			// The kinematic robot's base is a rigid fit to its support legs, which a
			// change of the set on the ground tilts at once (the legs then follow a
			// servo lag behind): within 2 deg, the band a crossing's base is held
			// to in simulation.
			Test.TestTrue(FString::Printf(TEXT("%s: the base within 2 deg of the plan (%.2f)"), *What, R.MaxTiltDeg), R.MaxTiltDeg <= 2.0);
		}
		if (Test.HasAnyErrors())
		{
			// What it did, for whoever reads the failure.
			for (const FString& L : R.Log)
			{
				Test.AddInfo(FString::Printf(TEXT("%s log: %s"), *What, *L));
			}
			for (const FString& L : R.Trace)
			{
				Test.AddInfo(FString::Printf(TEXT("%s trace: %s"), *What, *L));
			}
		}
		Test.AddInfo(FString::Printf(TEXT("%s: %.1f s, ended %s, least %.1f cm/s, %d stops, %d holds, lifts %d on position / %d on time / %d once "
										  "blocked, braking cap on %d ticks, clearance %.2f cm, margin %.2f cm, tilt %.2f deg"),
			*What, R.Seconds, RammsCurb::ExecPhaseName(R.EndPhase), R.MinSpeedCmS, R.Stops, R.Counts.Holds, R.Counts.LiftsOnPosition,
			R.Counts.LiftsOnTime, R.Counts.LiftsBlocked, R.Counts.BrakingTicks, R.MinSwingClearanceCm, R.MinMarginCm, R.MaxTiltDeg));
		return R;
	}

	void ExecRun(FAutomationTestBase& Test, const TCHAR* Id, bool bUp)
	{
		const FString				   What = FString::Printf(TEXT("%s %s"), Id, bUp ? TEXT("up") : TEXT("down"));
		const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(Id, bUp));
		const RammsCurb::FCurbSchedule S = PlanAndLog(Test, In, *What);
		if (!Test.TestTrue(FString::Printf(TEXT("%s planned"), *What), S.IsValid()))
		{
			return;
		}
		ExecRunOf(Test, What, In, S);
	}

	/** The robot in a drive stance: its front corners Up cm higher than on the
	 *  ground, off it, and its centre wheels 8 cm forward to carry it. */
	RammsCurb::FPlannerInput FrontsUp(RammsCurb::FPlannerInput In, double Up)
	{
		for (const int32 Leg : { PlanFL, PlanFR })
		{
			In.Legs[Leg].Measured.Y += Up;
			In.Legs[Leg].Measured.X = In.Model.Legs[Leg].FourBar.XAt(In.Legs[Leg].Measured.Y);
			In.Legs[Leg].bOffGround = true;
		}
		for (const int32 Leg : { PlanCL, PlanCR })
		{
			In.Legs[Leg].Measured.X = 8.0;
		}
		return In;
	}

	/** The robot in a tilted stance on flat ground: each leg's height moved so
	 *  that its wheel stays on the ground under a base pitched PitchDeg (nose
	 *  up) and rolled RollDeg (+Y up) about its origin. */
	RammsCurb::FPlannerInput TiltedStance(RammsCurb::FPlannerInput In, double PitchDeg, double RollDeg)
	{
		const double Tp = FMath::Tan(FMath::DegreesToRadians(PitchDeg));
		const double Tr = FMath::Tan(FMath::DegreesToRadians(RollDeg));
		for (int32 i = 0; i < 6; ++i)
		{
			RammsCurb::FPlannerLeg& P = In.Legs[i];
			const double			Z0 = P.Measured.Y;
			const double			Y = In.Model.Legs[i].Y;
			for (int32 Pass = 0; Pass < 3; ++Pass)
			{
				P.Measured.Y = Z0 - P.Measured.X * Tp - Y * Tr;
				if (In.Model.Legs[i].Kind == ERammsCurbLegKind::FourBar)
				{
					P.Measured.X = In.Model.Legs[i].FourBar.XAt(P.Measured.Y);
				}
			}
		}
		In.AttitudeDeg = FVector2D(PitchDeg, RollDeg);
		return In;
	}
} // namespace

	#define RAMMS_CURB_EXEC_RUN(Name, Id, bUp)                                                   \
		IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutor##Name, "Ramms.Curb.Executor." #Name, \
			EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)            \
		bool FRammsCurbExecutor##Name::RunTest(const FString& Parameters)                        \
		{                                                                                        \
			ExecRun(*this, TEXT(Id), bUp);                                                       \
			return true;                                                                         \
		}

RAMMS_CURB_EXEC_RUN(SquareUp, "Square", true)
RAMMS_CURB_EXEC_RUN(SquareDown, "Square", false)
RAMMS_CURB_EXEC_RUN(Oblique30Up, "Oblique30", true)
RAMMS_CURB_EXEC_RUN(Oblique30Down, "Oblique30", false)
RAMMS_CURB_EXEC_RUN(ObliqueNeg30Up, "ObliqueNeg30", true)
RAMMS_CURB_EXEC_RUN(ObliqueNeg30Down, "ObliqueNeg30", false)
RAMMS_CURB_EXEC_RUN(RoadFallsUp, "RoadFalls", true)
RAMMS_CURB_EXEC_RUN(SidewalkRisesDown, "SidewalkRises", false)

	#undef RAMMS_CURB_EXEC_RUN

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorLegLag50,
	"Ramms.Curb.Executor.LegLag50",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorLegLag50::RunTest(const FString& Parameters)
{
	// Legs half as slow again as the plan takes them to be: the lifts come
	// late, so the deadlines slow the robot -- it still crosses, nothing
	// violated, and no slower than it has to be (it may dip under the plan).
	for (const bool bUp : { true, false })
	{
		const FString				   What = FString::Printf(TEXT("Square %s, legs lagging 50 %% more"), bUp ? TEXT("up") : TEXT("down"));
		const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), bUp));
		const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
		RammsCurb::FExecDryRunOptions  Options;
		Options.LagScale = 1.5;
		const RammsCurb::FExecDryRunResult R = ExecRunOf(*this, What, In, S, Options, /*bExpectFluid=*/false);
		TestTrue(FString::Printf(TEXT("%s: done (%.1f s)"), *What, R.Seconds), R.bDone);
		TestEqual(FString::Printf(TEXT("%s: stops"), *What), R.Stops, 0);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorLegStall,
	"Ramms.Curb.Executor.LegStall",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorLegStall::RunTest(const FString& Parameters)
{
	// A front corner whose lift stops 3 cm up, short of clear, climbing: the
	// hard braking cap brings the robot to rest short of its window -- the
	// wheel never meets the face -- and the runtime checks fault, naming the
	// wheel (its leg off its servo lag model, or its lift overrunning); the
	// executor halts and the robot stays at rest there.
	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
	RammsCurb::FExecDryRunOptions  Options;
	Options.StallLeg = PlanFL;
	Options.StallAtCm = 3.0;
	Options.MaxSeconds = 20.0;
	const RammsCurb::FExecDryRunResult R = ExecRunOf(*this, TEXT("Square up, front_left stalling"), In, S, Options, /*bExpectFluid=*/false);
	TestFalse(TEXT("not done"), R.bDone);
	TestTrue(FString::Printf(TEXT("faulted, naming the wheel: %s"), *R.FaultWhy), R.bFaulted && R.FaultWhy.Contains(TEXT("front_left")));
	TestEqual(TEXT("halted"), FString(RammsCurb::ExecPhaseName(R.EndPhase)), FString(TEXT("Halted")));
	TestTrue(FString::Printf(TEXT("the braking cap bound (%d ticks)"), R.Counts.BrakingTicks), R.Counts.BrakingTicks > 0);
	TestTrue(FString::Printf(TEXT("at rest at its window, FaceMargin short of the face: %.2f cm past the edge, the window from %.2f"),
				 R.FinalPastEdgeCm.IsValidIndex(PlanFL) ? R.FinalPastEdgeCm[PlanFL] : 0.0, S.Wheels[PlanFL].DUp),
		R.FinalPastEdgeCm.IsValidIndex(PlanFL) && R.FinalPastEdgeCm[PlanFL] <= S.Wheels[PlanFL].DUp + 0.5 && R.FinalSpeedCmS < 0.5);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorRollingEntry,
	"Ramms.Curb.Executor.RollingEntry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorRollingEntry::RunTest(const FString& Parameters)
{
	// Begun on the move, faster than the drive speed: it carries on from there,
	// never stopping, never under the floor.
	for (const double Entry : { 20.0, 45.0 })
	{
		RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
		In.EntrySpeedCmS = Entry;
		const FString				   What = FString::Printf(TEXT("Square up, rolling in at %.0f cm/s"), Entry);
		const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, *What);
		if (TestTrue(FString::Printf(TEXT("%s: planned"), *What), S.IsValid()))
		{
			ExecRunOf(*this, What, In, S);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorHeldCorners,
	"Ramms.Curb.Executor.HeldCorners",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorHeldCorners::RunTest(const FString& Parameters)
{
	// A drive stance with the front corners up. Near the edge, whose lifts would
	// come soon after the level-in, they stay up and are carried across (Held,
	// then clear, then down past the window); far from it, they are set down
	// first in the level-in, before any other leg moves.
	{
		const RammsCurb::FPlannerInput In = FrontsUp(PlanOn(PlanScenario(TEXT("Square"), true)), 6.0);
		const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("Square up, front corners up"));
		if (TestTrue(TEXT("planned"), S.IsValid()) && TestTrue(TEXT("the front corners held from the start"), S.Wheels[PlanFL].bHeldFromStart && S.Wheels[PlanFR].bHeldFromStart))
		{
			const RammsCurb::FExecDryRunResult R = ExecRunOf(*this, TEXT("Square up, front corners up"), In, S);
			bool							   bCarried = false, bSetDown = false;
			for (const FString& L : R.Log)
			{
				bCarried = bCarried || (L.Contains(TEXT("front_left")) && L.Contains(TEXT("held -> clear")));
				bSetDown = bSetDown || L.Contains(TEXT("set down for the level-in"));
			}
			TestTrue(TEXT("carried up from the drive stance, clear"), bCarried);
			TestFalse(TEXT("not set down"), bSetDown);
		}
	}
	{
		// A road falling 6 deg towards the curb, its edge 60 cm further on: the
		// level-in begins as the crossing does, long before the front corners'
		// lifts.
		FRammsCurbTerrain T = PlanScenario(TEXT("RoadFalls"), true);
		const FVector2D	  Shift = T.EdgeNormal * 60.0;
		const FVector2D	  Edge(T.EdgePoint.X + Shift.X, T.EdgePoint.Y + Shift.Y);
		T.EdgePoint = FVector(Edge.X, Edge.Y, T.Near.HeightAt(Edge));
		T.Far.Origin = FVector(Edge.X, Edge.Y, T.Near.HeightAt(Edge) + T.StepHeightCm);
		const RammsCurb::FPlannerInput In = FrontsUp(PlanOn(T), 6.0);
		const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("RoadFalls up, front corners up"));
		if (TestTrue(TEXT("planned on the falling road"), S.IsValid()) && TestFalse(TEXT("the front corners not held"), S.Wheels[PlanFL].bHeldFromStart))
		{
			const RammsCurb::FExecDryRunResult R = ExecRunOf(*this, TEXT("RoadFalls up, front corners up"), In, S);
			int32							   SetDown = INDEX_NONE, Down = INDEX_NONE, OnTerrain = INDEX_NONE;
			for (int32 k = 0; k < R.Log.Num(); ++k)
			{
				const FString& L = R.Log[k];
				SetDown = SetDown == INDEX_NONE && L.Contains(TEXT("set down for the level-in")) ? k : SetDown;
				Down = Down == INDEX_NONE && L.Contains(TEXT("loading -> planted")) ? k : Down;
				OnTerrain = OnTerrain == INDEX_NONE && L.Contains(TEXT("legs on the terrain")) ? k : OnTerrain;
			}
			TestTrue(FString::Printf(TEXT("set down first (lines %d, %d), then the other legs on the terrain (%d)"), SetDown, Down, OnTerrain),
				SetDown != INDEX_NONE && Down > SetDown && OnTerrain > Down);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorMustBeDown,
	"Ramms.Curb.Executor.MustBeDown",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorMustBeDown::RunTest(const FString& Parameters)
{
	// Front legs that land late (a plant three times as slow as planned): the
	// centre wheels, which may not lift before the fronts have taken load, wait
	// -- their deadline slowing the robot -- and never lift with a front still
	// up (ExecRunOf counts any such lift); nothing violated, and the crossing
	// done.
	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
	if (!TestTrue(TEXT("the centre wheels wait for the fronts"), ((S.Wheels[PlanCL].MustBeDownMask | S.Wheels[PlanCR].MustBeDownMask) & ((1u << PlanFL) | (1u << PlanFR))) != 0))
	{
		return true;
	}
	RammsCurb::FExecDryRunOptions Options;
	Options.SlowPlantLegs = (1u << PlanFL) | (1u << PlanFR);
	Options.SlowPlantScale = 3.0;
	const RammsCurb::FExecDryRunResult R = ExecRunOf(*this, TEXT("Square up, fronts landing slowly"), In, S, Options, /*bExpectFluid=*/false);
	TestTrue(FString::Printf(TEXT("done (%.1f s)"), R.Seconds), R.bDone);
	bool bWaited = false;
	for (const FString& L : R.Log)
	{
		bWaited = bWaited || L.Contains(TEXT("waits for 'front_"));
	}
	const uint32 Centres = (1u << PlanCL) | (1u << PlanCR);
	TestTrue(FString::Printf(TEXT("a centre wheel's lift waited for a front, or its deadline slowed the robot for it (deadline legs %x)"), R.Counts.DeadlineLegs),
		bWaited || (R.Counts.DeadlineLegs & Centres) != 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorLateLandingHoldsCentres,
	"Ramms.Curb.Executor.LateLandingHoldsCentres",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorLateLandingHoldsCentres::RunTest(const FString& Parameters)
{
	// Front legs landing late enough that the centre wheels hold their
	// fore/aft for them (the walk the plan makes from there is for the robot
	// standing on the fronts): a centre wheel's deadline must not count the
	// aft walk it is not making as room, or the robot runs on towards its
	// window and only the hard braking cap stops it there (a hold): the
	// deadline slows it instead. (Counting that walk, the 5x slower landings
	// had the braking cap bind on 5 and 18 ticks and held the oblique
	// crossing.)
	for (const TCHAR* Id : { TEXT("Square"), TEXT("Oblique30") })
	{
		for (const double Scale : { 3.0, 5.0 })
		{
			const FString				   What = FString::Printf(TEXT("%s up, fronts landing %.0fx slower"), Id, Scale);
			const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(Id, true));
			const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
			if (!TestTrue(FString::Printf(TEXT("%s: planned"), *What), S.IsValid()))
			{
				continue;
			}
			RammsCurb::FExecDryRunOptions Options;
			Options.SlowPlantLegs = (1u << PlanFL) | (1u << PlanFR);
			Options.SlowPlantScale = Scale;
			const RammsCurb::FExecDryRunResult R = ExecRunOf(*this, What, In, S, Options, /*bExpectFluid=*/false);
			bool							   bHeld = false;
			for (const FString& L : R.Log)
			{
				bHeld = bHeld || L.Contains(TEXT("the centre wheels hold their fore/aft"));
			}
			AddInfo(FString::Printf(TEXT("%s: centres held %d, done %d, holds %d, braking cap on %d ticks, deadline legs %x"), *What, bHeld ? 1 : 0,
				R.bDone ? 1 : 0, R.Counts.Holds, R.Counts.BrakingTicks, R.Counts.DeadlineLegs));
			TestTrue(FString::Printf(TEXT("%s: done (%.1f s)"), *What, R.Seconds), R.bDone);
			TestTrue(FString::Printf(TEXT("%s: the centre wheels held their fore/aft for the late landing"), *What), bHeld);
			TestEqual(FString::Printf(TEXT("%s: holds"), *What), R.Counts.Holds, 0);
			TestTrue(FString::Printf(TEXT("%s: slowed by the deadline, the hard braking cap bound on %d ticks (2 at most)"), *What, R.Counts.BrakingTicks),
				R.Counts.BrakingTicks <= 2);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorBlockedLiftHoldsChassis,
	"Ramms.Curb.Executor.BlockedLiftHoldsChassis",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorBlockedLiftHoldsChassis::RunTest(const FString& Parameters)
{
	// A lift that waits on a late landing: its wheel still carries where the
	// plan has it up, and the plan's chassis height there is for it up. The
	// chassis is held no higher than that leg reaches on the ground, so the base
	// does not tip over its corner while the robot waits. Climbs: the plan
	// raises the chassis there for the rear corners' lifts. (Without the hold
	// the base tilted 1.28 deg on the square climb and 0.88 on the oblique
	// ones, against 1.07 and 0.59 with it.)
	struct FCase
	{
		const TCHAR* Id;
		double		 MaxTiltDeg;
	};
	for (const FCase& C : { FCase{ TEXT("Square"), 1.15 }, FCase{ TEXT("Oblique30"), 0.7 }, FCase{ TEXT("ObliqueNeg30"), 0.7 } })
	{
		const FString				   What = FString::Printf(TEXT("%s up, centres landing 5x slower"), C.Id);
		const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(C.Id, true));
		const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
		if (!TestTrue(FString::Printf(TEXT("%s: planned"), *What), S.IsValid()))
		{
			continue;
		}
		RammsCurb::FExecDryRunOptions Options;
		Options.SlowPlantLegs = (1u << PlanCL) | (1u << PlanCR);
		Options.SlowPlantScale = 5.0;
		const RammsCurb::FExecDryRunResult R = ExecRunOf(*this, What, In, S, Options, /*bExpectFluid=*/false);
		bool							   bCeiling = false;
		for (const FString& L : R.Log)
		{
			bCeiling = bCeiling || L.Contains(TEXT("under the plan's height: '"));
		}
		AddInfo(FString::Printf(TEXT("%s: chassis held %d, done %d, holds %d, tilt %.2f deg, lifts blocked %d"), *What, bCeiling ? 1 : 0, R.bDone ? 1 : 0,
			R.Counts.Holds, R.MaxTiltDeg, R.Counts.LiftsBlocked));
		TestTrue(FString::Printf(TEXT("%s: done (%.1f s)"), *What, R.Seconds), R.bDone);
		TestTrue(FString::Printf(TEXT("%s: the chassis held under the plan's height for the waiting lift"), *What), bCeiling);
		TestTrue(FString::Printf(TEXT("%s: the base within %.2f deg (%.2f)"), *What, C.MaxTiltDeg, R.MaxTiltDeg), R.MaxTiltDeg <= C.MaxTiltDeg);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorBehindOrigin,
	"Ramms.Curb.Executor.BehindOrigin",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorBehindOrigin::RunTest(const FString& Parameters)
{
	// Planned at rest, begun a little behind where it was planned (the robot
	// rolled back in the meantime): it drives up to the plan and crosses --
	// never resting at the plan's first sample, never held -- on flat ground
	// and on a road falling towards the curb, whose level-in begins at once.
	struct FCase
	{
		const TCHAR* Id;
		double		 Behind;
	};
	for (const FCase& C : { FCase{ TEXT("Square"), 3.0 }, FCase{ TEXT("Square"), 10.0 }, FCase{ TEXT("RoadFalls"), 8.0 } })
	{
		const FString				   What = FString::Printf(TEXT("%s up, begun %.0f cm behind its plan"), C.Id, C.Behind);
		const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(C.Id, true));
		const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, *What);
		if (!TestTrue(FString::Printf(TEXT("%s: planned"), *What), S.IsValid()))
		{
			continue;
		}
		TestTrue(FString::Printf(TEXT("%s: planned from rest"), *What), S.Samples[0].V <= 1e-6);
		RammsCurb::FExecDryRunOptions Options;
		Options.StartTravelCm = -C.Behind;
		const RammsCurb::FExecDryRunResult R = ExecRunOf(*this, What, In, S, Options);
		bool							   bHeld = false;
		for (const FString& L : R.Log)
		{
			bHeld = bHeld || L.Contains(TEXT("hold:"));
		}
		TestFalse(FString::Printf(TEXT("%s: no hold logged"), *What), bHeld);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorLevelInTurnRate,
	"Ramms.Curb.Executor.LevelInTurnRate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorLevelInTurnRate::RunTest(const FString& Parameters)
{
	// From a drive stance pitched 4 deg and rolled 2 deg: the level-in turns
	// the base no faster than TiltRateDegPerSec, all the way (the attitude loop
	// and the level check wait for the ramp to end), and the crossing is made.
	const RammsCurb::FPlannerInput In = TiltedStance(PlanOn(PlanScenario(TEXT("Square"), true)), 4.0, 2.0);
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("Square up, from a tilted stance"));
	if (!TestTrue(TEXT("planned"), S.IsValid()))
	{
		return true;
	}
	const double Need = RammsCurb::SCurveSeconds(1.0, In.Tuning.TiltRateDegPerSec / 4.0, In.Tuning.TiltAccelDegPerSec2 / 4.0);
	TestTrue(FString::Printf(TEXT("the plan gives the level-in the S-curve's time (%.2f s, the S-curve %.2f s)"), S.LevelInSeconds, Need),
		S.LevelInSeconds >= Need);
	const RammsCurb::FExecDryRunResult R = ExecRunOf(*this, TEXT("Square up, from a tilted stance"), In, S);
	TestTrue(FString::Printf(TEXT("the base turned at most %.2f deg/s levelling in (TiltRateDegPerSec %.1f)"), R.MaxLevelInTurnDegS,
				 In.Tuning.TiltRateDegPerSec),
		R.MaxLevelInTurnDegS > 0.5 && R.MaxLevelInTurnDegS <= In.Tuning.TiltRateDegPerSec * 1.05);
	int32 From = INDEX_NONE, OnTerrain = INDEX_NONE, Level = INDEX_NONE;
	for (int32 k = 0; k < R.Log.Num(); ++k)
	{
		From = From == INDEX_NONE && R.Log[k].Contains(TEXT("the legs to the terrain over")) ? k : From;
		OnTerrain = OnTerrain == INDEX_NONE && R.Log[k].Contains(TEXT("legs on the terrain")) ? k : OnTerrain;
		Level = Level == INDEX_NONE && R.Log[k].Contains(TEXT("level: pitch")) ? k : Level;
	}
	TestTrue(FString::Printf(TEXT("the ramp, then the legs on the terrain, then level (lines %d, %d, %d)"), From, OnTerrain, Level),
		From != INDEX_NONE && OnTerrain > From && Level >= OnTerrain);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorCornerSetDownInStance,
	"Ramms.Curb.Executor.CornerSetDownInStance",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorCornerSetDownInStance::RunTest(const FString& Parameters)
{
	// A drive stance rolled 4 deg and pitched 2 with its front corners up, far
	// enough from the edge that the level-in sets them down first. Each is set
	// down, and takes its load, where the stance has its wheel's ground -- not
	// at the plan's level pose, which would have it reach for a chassis that is
	// not there yet and jack the base round over the legs that hold the stance
	// -- so the base turns no faster than TiltRateDegPerSec all through the
	// level-in, its first stage included.
	FRammsCurbTerrain T = PlanScenario(TEXT("RoadFalls"), true);
	const FVector2D	  Shift = T.EdgeNormal * 150.0;
	const FVector2D	  Edge(T.EdgePoint.X + Shift.X, T.EdgePoint.Y + Shift.Y);
	T.EdgePoint = FVector(Edge.X, Edge.Y, T.Near.HeightAt(Edge));
	T.Far.Origin = FVector(Edge.X, Edge.Y, T.Near.HeightAt(Edge) + T.StepHeightCm);
	const RammsCurb::FPlannerInput In = FrontsUp(TiltedStance(PlanOn(T), 2.0, 4.0), 6.0);
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("RoadFalls up, a rolled stance, front corners up"));
	if (!TestTrue(TEXT("planned"), S.IsValid()) || !TestFalse(TEXT("the front corners set down in the level-in"), S.Wheels[PlanFL].bHeldFromStart))
	{
		return true;
	}
	const RammsCurb::FExecDryRunResult R = ExecRunOf(*this, TEXT("RoadFalls up, a rolled stance, front corners up"), In, S,
		RammsCurb::FExecDryRunOptions(), /*bExpectFluid=*/false);
	TestTrue(FString::Printf(TEXT("done (%.1f s)"), R.Seconds), R.bDone);
	TestTrue(FString::Printf(TEXT("the base turned at most %.2f deg/s levelling in, the corners' set-down included (TiltRateDegPerSec %.1f)"),
				 R.MaxLevelInTurnDegS, In.Tuning.TiltRateDegPerSec),
		R.MaxLevelInTurnDegS <= In.Tuning.TiltRateDegPerSec * 1.05);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorLegStallDown,
	"Ramms.Curb.Executor.LegStallDown",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorLegStallDown::RunTest(const FString& Parameters)
{
	// Descending, a front corner whose lift never leaves the ground: it is
	// loaded, so the hard braking cap brings the robot to rest before the
	// wheel rolls off the lip -- short of the drop edge, within its window's
	// EdgeMargin -- and the executor holds, naming the wheel.
	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), false));
	const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
	if (!TestTrue(TEXT("planned"), S.IsValid()))
	{
		return true;
	}
	RammsCurb::FExecDryRunOptions Options;
	Options.StallLeg = PlanFL;
	Options.StallAtCm = 0.0;
	Options.MaxSeconds = 20.0;
	Options.TraceEvery = 6;
	const RammsCurb::FExecDryRunResult R = RammsCurb::ExecDryRun(In, S, Options);
	const RammsCurb::FWheelPlan&	   W = S.Wheels[PlanFL];
	const double					   Most = R.MaxNearCarryPastEdgeCm.IsValidIndex(PlanFL) ? R.MaxNearCarryPastEdgeCm[PlanFL] : 1e9;
	// (Its leg rises no more than the descent's clearance asks, under
	// LegErrorFaultCm off its servo lag model: the executor holds, and the
	// controller's hold timeout faults it.)
	TestFalse(TEXT("not done"), R.bDone);
	TestEqual(TEXT("held"), FString(RammsCurb::ExecPhaseName(R.EndPhase)), FString(TEXT("Hold")));
	TestTrue(FString::Printf(TEXT("a hold counted (%d)"), R.Counts.Holds), R.Counts.Holds >= 1);
	TestTrue(FString::Printf(TEXT("the braking cap bound (%d ticks)"), R.Counts.BrakingTicks), R.Counts.BrakingTicks > 0);
	TestTrue(FString::Printf(TEXT("loaded, it never reached the drop edge: at most %.2f cm past the edge (the edge at %.2f, its window from %.2f)"),
				 Most, -W.Lambda, W.DUp),
		Most < -W.Lambda && Most > W.DUp - 30.0);
	TestTrue(FString::Printf(TEXT("at rest (%.2f cm/s)"), R.FinalSpeedCmS), R.FinalSpeedCmS < 0.5);
	TestEqual(TEXT("lifted wheels into the terrain"), R.IntoTerrain, 0);
	TestEqual(TEXT("ticks with a forbidden set off the ground"), R.Counts.ForbiddenTicks, 0);
	bool bNamed = false;
	for (const FString& L : R.Log)
	{
		bNamed = bNamed || (L.Contains(TEXT("hold")) && L.Contains(TEXT("front_left")));
	}
	TestTrue(TEXT("the hold names the wheel"), bNamed);
	AddInfo(FString::Printf(TEXT("%.1f s, ended %s, %d holds, braking cap on %d ticks, a loaded wheel in its window on %d ticks (executor) / %d "
								 "(dry run), front_left at most %.2f cm past the edge loaded"),
		R.Seconds, RammsCurb::ExecPhaseName(R.EndPhase), R.Counts.Holds, R.Counts.BrakingTicks, R.Counts.WindowTicks, R.KeepOffViolations, Most));
	if (HasAnyErrors())
	{
		for (const FString& L : R.Log)
		{
			AddInfo(FString::Printf(TEXT("log: %s"), *L));
		}
		for (const FString& L : R.Trace)
		{
			AddInfo(FString::Printf(TEXT("trace: %s"), *L));
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorPlannedStopNoHold,
	"Ramms.Curb.Executor.PlannedStopNoHold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorPlannedStopNoHold::RunTest(const FString& Parameters)
{
	// A plan with a planned stop (centre wheels that walk only 2 cm/s): the
	// robot brakes onto it, rests while they re-pose and goes on -- the stop
	// counted as a planned stop, never as a hold, braking into it included.
	RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), false));
	In.Tuning.CenterShiftSpeed = 2.0f;
	In.Tuning.MaxPlannedTiltDeg = 0.0f;
	const RammsCurb::FCurbSchedule S = PlanAndLog(*this, In, TEXT("Square down, slow centre wheels"));
	if (!TestEqual(TEXT("a planned stop"), FString(RammsCurb::PlanClassName(S.Class)), FString(TEXT("planned stop"))))
	{
		return true;
	}
	const RammsCurb::FExecDryRunResult R = ExecRunOf(*this, TEXT("Square down, a planned stop"), In, S, RammsCurb::FExecDryRunOptions(),
		/*bExpectFluid=*/false);
	TestTrue(FString::Printf(TEXT("done (%.1f s)"), R.Seconds), R.bDone);
	TestTrue(FString::Printf(TEXT("its planned stops made (%d of %d)"), R.Counts.PlannedStops, S.PlannedStops), R.Counts.PlannedStops >= S.PlannedStops);
	TestEqual(TEXT("holds"), R.Counts.Holds, 0);
	if (HasAnyErrors())
	{
		for (const FString& L : S.Lines)
		{
			AddInfo(FString::Printf(TEXT("plan: %s"), *L));
		}
		for (const FString& L : R.Log)
		{
			AddInfo(FString::Printf(TEXT("log: %s"), *L));
		}
		for (const FString& L : R.Trace)
		{
			AddInfo(FString::Printf(TEXT("trace: %s"), *L));
		}
	}
	return true;
}

namespace
{
	/** A disturbed run (the runtime checks): the executor round the kinematic
	 *  robot as Options disturb it, checked for what must hold whatever it
	 *  does -- no wheel on the ground in its window, no lifted one into the
	 *  terrain (as it is), no forbidden set, no lift before the wheels it
	 *  waits on -- with what it did as info, and its log on a failure. */
	RammsCurb::FExecDryRunResult ExecDisturbed(FAutomationTestBase& Test, const FString& What, const RammsCurb::FPlannerInput& In,
		const RammsCurb::FCurbSchedule& S, const RammsCurb::FExecDryRunOptions& Options)
	{
		const RammsCurb::FExecDryRunResult R = RammsCurb::ExecDryRun(In, S, Options);
		Test.TestEqual(FString::Printf(TEXT("%s: wheels on the ground inside their windows"), *What), R.KeepOffViolations, 0);
		Test.TestEqual(FString::Printf(TEXT("%s: lifted wheels into the terrain"), *What), R.IntoTerrain, 0);
		Test.TestEqual(FString::Printf(TEXT("%s: ticks with a forbidden set off the ground"), *What), R.Counts.ForbiddenTicks, 0);
		Test.TestEqual(FString::Printf(TEXT("%s: lifts before the wheels they wait on had landed"), *What), R.EarlyLifts, 0);
		Test.AddInfo(FString::Printf(TEXT("%s: %.1f s, ended %s, done %d, faulted %d (%s) at %.2f s, halted %d, at rest %d, braking at most %.0f cm/s^2, "
										  "inclined at most %.2f deg (%.2f once halted), %d re-plans, clearance %s cm"),
			*What, R.Seconds, RammsCurb::ExecPhaseName(R.EndPhase), R.bDone ? 1 : 0, R.bFaulted ? 1 : 0, *R.FaultWhy, R.FaultAt, R.bHalted ? 1 : 0,
			R.bStopped ? 1 : 0, R.HaltBrakingCmS2, R.MaxInclinationDeg, R.HaltedInclinationDeg, R.Replans,
			R.MinSwingClearanceCm < 1e6 ? *FString::Printf(TEXT("%.2f"), R.MinSwingClearanceCm) : TEXT("-")));
		if (Test.HasAnyErrors())
		{
			for (const FString& L : R.Log)
			{
				Test.AddInfo(FString::Printf(TEXT("%s log: %s"), *What, *L));
			}
		}
		return R;
	}

	/** A fault ended the run cleanly: named (Contains), the robot halted and
	 *  brought to rest, braked no harder than twice Deceleration, upright
	 *  (within MaxIncl of level once halted). */
	void ExpectCleanFault(FAutomationTestBase& Test, const FString& What, const RammsCurb::FExecDryRunResult& R, const TCHAR* Contains,
		const FRammsCurbTuning& T, double MaxIncl = 5.0)
	{
		Test.TestTrue(FString::Printf(TEXT("%s: faulted, '%s' (%s)"), *What, Contains, *R.FaultWhy), R.bFaulted && R.FaultWhy.Contains(Contains));
		Test.TestTrue(FString::Printf(TEXT("%s: halted and at rest (%.2f cm/s)"), *What, R.FinalSpeedCmS), R.bHalted && R.bStopped && R.FinalSpeedCmS < 0.5);
		Test.TestTrue(FString::Printf(TEXT("%s: braked at most twice Deceleration (%.0f cm/s^2)"), *What, R.HaltBrakingCmS2),
			R.HaltBrakingCmS2 <= 2.0 * T.Deceleration + 1.0);
		Test.TestTrue(FString::Printf(TEXT("%s: upright once halted (%.2f deg)"), *What, R.HaltedInclinationDeg), R.HaltedInclinationDeg < MaxIncl);
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorTerrainOff2cm,
	"Ramms.Curb.Executor.TerrainOff2cm",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorTerrainOff2cm::RunTest(const FString& Parameters)
{
	// The far surface off by what a plane's bias takes up (MaxPlaneBiasCm): the
	// traces ahead find it, its bias takes it up (lifted wheels carried that much
	// higher over a higher one), and the crossing is made, up and down -- no
	// fault. Off by 2 cm (TerrainMismatchCm), seen only once the front wheels
	// have begun to cross -- too late to re-plan -- more than a bias takes up: a
	// clean fault, naming it.
	for (const bool bUp : { true, false })
	{
		const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), bUp));
		const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
		if (!TestTrue(TEXT("planned"), S.IsValid()))
		{
			continue;
		}
		for (const double Off : { 1.4, -1.4, 2.0, -2.0 })
		{
			const FString				  What = FString::Printf(TEXT("Square %s, the far surface %+.1f cm"), bUp ? TEXT("up") : TEXT("down"), Off);
			RammsCurb::FExecDryRunOptions Options;
			Options.TrueOffsetCm = FVector2D(0.0, Off);
			const RammsCurb::FExecDryRunResult R = ExecDisturbed(*this, What, In, S, Options);
			if (FMath::Abs(Off) <= RammsCurb::MaxPlaneBiasCm)
			{
				TestTrue(FString::Printf(TEXT("%s: done, no fault (%s)"), *What, *R.FaultWhy), R.bDone && !R.bFaulted);
			}
			else
			{
				TestFalse(FString::Printf(TEXT("%s: not done"), *What), R.bDone);
				ExpectCleanFault(*this, What, R, TEXT("a plane's bias takes up"), In.Tuning);
			}
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorTerrainOff5cm,
	"Ramms.Curb.Executor.TerrainOff5cm",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorTerrainOff5cm::RunTest(const FString& Parameters)
{
	// The far surface 5 cm off: the traces ahead see it before any wheel
	// reaches it, and the crossing faults -- "terrain not as detected" -- the
	// robot braked to rest short of every window, upright; no wheel meets the
	// face or lands on what is not there.
	for (const bool bUp : { true, false })
	{
		const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), bUp));
		const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
		if (!TestTrue(TEXT("planned"), S.IsValid()))
		{
			continue;
		}
		for (const double Off : { 5.0, -5.0 })
		{
			const FString				  What = FString::Printf(TEXT("Square %s, the far surface %+.0f cm"), bUp ? TEXT("up") : TEXT("down"), Off);
			RammsCurb::FExecDryRunOptions Options;
			Options.TrueOffsetCm = FVector2D(0.0, Off);
			const RammsCurb::FExecDryRunResult R = ExecDisturbed(*this, What, In, S, Options);
			TestFalse(FString::Printf(TEXT("%s: not done"), *What), R.bDone);
			ExpectCleanFault(*this, What, R, TEXT("terrain not as detected"), In.Tuning);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorNearRefit,
	"Ramms.Curb.Executor.NearRefit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorNearRefit::RunTest(const FString& Parameters)
{
	// The surface the robot stands on 1.5 cm lower than planned (and so the
	// curb 1.5 cm taller): before any wheel has begun to cross, the traces find
	// it more than ReplanTerrainCm off, the near plane is re-fitted and the
	// crossing re-planned on it, and made.
	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
	if (!TestTrue(TEXT("planned"), S.IsValid()))
	{
		return true;
	}
	RammsCurb::FExecDryRunOptions Options;
	Options.TrueOffsetCm = FVector2D(-1.5, 0.0);
	const RammsCurb::FExecDryRunResult R = ExecDisturbed(*this, TEXT("Square up, the near surface 1.5 cm low"), In, S, Options);
	TestTrue(FString::Printf(TEXT("done, no fault (%s)"), *R.FaultWhy), R.bDone && !R.bFaulted);
	TestTrue(FString::Printf(TEXT("re-planned on the near surface re-fitted (%d: %s)"), R.Replans, R.ReplanWhy.Num() ? *R.ReplanWhy[0] : TEXT("")),
		R.Replans >= 1 && R.ReplanWhy.Num() > 0 && R.ReplanWhy[0].Contains(TEXT("near surface")));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorReplanBias,
	"Ramms.Curb.Executor.ReplanBias",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorReplanBias::RunTest(const FString& Parameters)
{
	// A runtime trigger's re-plan on a surface re-fitted to what the traces
	// found, adopted at once and after the time a plan made off the game thread
	// takes (0.15 s): the re-fitted surface's bias, from the adoption on, is
	// where that surface truly lies off its new plane (within a few mm) -- not
	// the old bias moved by the re-fit, which is right only where the old one
	// had caught up with the surface (one still catching up -- here through the
	// planted wheels, about 0.4 cm of 1.5 after 0.15 s -- would be left wrong
	// by the rest). The surface the robot stands on 1.5 cm low and 1.2 cm high,
	// seen from the start. (A far surface is seen by the traces only once the
	// front wheels are within 30 cm of the edge, after their lifts on this
	// robot: no re-plan, the bias takes it up -- TerrainOff2cm.)
	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
	if (!TestTrue(TEXT("planned"), S.IsValid()))
	{
		return true;
	}
	for (const double Off : { -1.5, 1.2 })
	{
		for (const double Delay : { 0.0, 0.15 })
		{
			const FString				  What = FString::Printf(TEXT("Square up, the near surface %+.1f cm, re-plans adopted %.2f s after the ask"), Off, Delay);
			RammsCurb::FExecDryRunOptions Options;
			Options.TrueOffsetCm = FVector2D(Off, 0.0);
			Options.ReplanDelaySeconds = Delay;
			const RammsCurb::FExecDryRunResult R = ExecDisturbed(*this, What, In, S, Options);
			TestTrue(FString::Printf(TEXT("%s: done, no fault (%s)"), *What, *R.FaultWhy), R.bDone && !R.bFaulted);
			TestTrue(FString::Printf(TEXT("%s: re-planned (%d)"), *What, R.Replans), R.Replans >= 1);
			TestTrue(FString::Printf(TEXT("%s: the re-fitted surface's bias where the surface is (off by at most %.2f cm)"), *What, R.ReplanBiasErrorCm),
				R.ReplanBiasErrorCm <= 0.25);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorTracedTouchdown,
	"Ramms.Curb.Executor.TracedTouchdown",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorTracedTouchdown::RunTest(const FString& Parameters)
{
	// The sidewalk 0.45 cm higher than planned -- under what a bias follows from
	// the traces ahead (here blind), so the first wheels land with none: each
	// touches down where the trace under it finds the surface, as it reaches
	// it, rather than pressing its leg on until it stalls or the plan's
	// surface is reached.
	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
	if (!TestTrue(TEXT("planned"), S.IsValid()))
	{
		return true;
	}
	RammsCurb::FExecDryRunOptions Options;
	Options.TrueOffsetCm = FVector2D(0.0, 0.45);
	Options.bProbeAhead = false;
	const RammsCurb::FExecDryRunResult R = ExecDisturbed(*this, TEXT("Square up, the sidewalk 0.45 cm high"), In, S, Options);
	TestTrue(FString::Printf(TEXT("done, no fault (%s)"), *R.FaultWhy), R.bDone && !R.bFaulted);
	int32 Traced = 0;
	for (const FString& L : R.Log)
	{
		Traced += L.Contains(TEXT("touched down (on the surface traced under it)")) ? 1 : 0;
	}
	TestTrue(FString::Printf(TEXT("wheels touched down on the surface traced under them: %d"), Traced), Traced >= 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorRidge,
	"Ramms.Curb.Executor.Ridge",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorRidge::RunTest(const FString& Parameters)
{
	// A ridge 2.5 cm high and 4 cm wide across the sidewalk, 25 cm past the
	// edge -- more than TerrainMismatchCm on a surface planned flat: the
	// crossing either makes it or faults naming it, never meeting it with a
	// wheel in the air or falling.
	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
	if (!TestTrue(TEXT("planned"), S.IsValid()))
	{
		return true;
	}
	RammsCurb::FExecDryRunOptions Options;
	Options.RidgeHeightCm = 2.5;
	Options.RidgePastEdgeCm = 25.0;
	Options.RidgeWidthCm = 4.0;
	const RammsCurb::FExecDryRunResult R = ExecDisturbed(*this, TEXT("Square up, a ridge past the edge"), In, S, Options);
	TestTrue(FString::Printf(TEXT("done, or faulted on the terrain (%s)"), *R.FaultWhy),
		R.bDone || (R.bFaulted && R.FaultWhy.Contains(TEXT("terrain not as detected"))));
	if (R.bFaulted)
	{
		ExpectCleanFault(*this, TEXT("a ridge"), R, TEXT("terrain not as detected"), In.Tuning);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorLandingChecks,
	"Ramms.Curb.Executor.LandingChecks",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorLandingChecks::RunTest(const FString& Parameters)
{
	// The landing check, with the traces ahead blind (the terrain monitor sees
	// nothing ahead to stop for): a front wheel set down on a sidewalk 2.2 cm
	// higher than planned is stopped by it more than LandingToleranceCm above
	// where it was to land -- a fault, naming it, and, halted, both front
	// legs held where they met the sidewalk, their commands no more than
	// PlantProbeCm under where their legs stood then (the faulting one at
	// once, the other once the halt sees it pressing) rather than sent on down
	// to where the plan has the surface (2.2 cm deeper: jacking the robot up
	// on them); one
	// coming down where a pit 3 cm deep begins just past its window finds no
	// ground there (the trace under it) -- lifted clear again, a fault, naming
	// it. With LandingToleranceCm raised to 2.5, a sidewalk 2.0 cm high is
	// within it: touched down, not left pressing on until its plant's timeout.
	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
	if (!TestTrue(TEXT("planned"), S.IsValid()))
	{
		return true;
	}
	{
		RammsCurb::FExecDryRunOptions Options;
		Options.bProbes = false;
		Options.TrueOffsetCm = FVector2D(0.0, 2.2);
		const FString					   What = TEXT("Square up, the sidewalk 2.2 cm high, no traces");
		const RammsCurb::FExecDryRunResult R = ExecDisturbed(*this, What, In, S, Options);
		ExpectCleanFault(*this, What, R, TEXT("above where it was to land"), In.Tuning);
		TestTrue(FString::Printf(TEXT("%s: a front wheel named"), *What), R.FaultWhy.Contains(TEXT("front_")));
		if (TestEqual(FString::Printf(TEXT("%s: a hold for every wheel"), *What), R.HaltHeldPressCm.Num(), In.Legs.Num()))
		{
			for (const int32 Leg : { PlanFL, PlanFR })
			{
				TestTrue(FString::Printf(TEXT("%s: '%s' held where it met the sidewalk, its command %.2f cm under its leg then (at most %.2f)"), *What,
							 *In.Legs[Leg].Name.ToString(), R.HaltHeldPressCm[Leg], RammsCurb::PlantProbeCm),
					R.HaltHeldPressCm[Leg] >= 0.0 && R.HaltHeldPressCm[Leg] <= RammsCurb::PlantProbeCm + 1e-3);
			}
		}
	}
	{
		RammsCurb::FPlannerInput Wide = In;
		Wide.Tuning.LandingToleranceCm = 2.5f;
		RammsCurb::FExecDryRunOptions Options;
		Options.bProbes = false;
		Options.TrueOffsetCm = FVector2D(0.0, 2.0);
		const FString					   What = TEXT("Square up, the sidewalk 2.0 cm high, no traces, LandingToleranceCm 2.5");
		const RammsCurb::FExecDryRunResult R = ExecDisturbed(*this, What, Wide, S, Options);
		int32							   Down = 0;
		for (const FString& L : R.Log)
		{
			Down += L.Contains(TEXT("front_")) && L.Contains(TEXT("touched down")) ? 1 : 0;
		}
		TestTrue(FString::Printf(TEXT("%s: the front wheels touched down (%d)"), *What, Down), Down >= 2);
		TestFalse(FString::Printf(TEXT("%s: no landing left to its timeout (%s)"), *What, *R.FaultWhy), R.FaultWhy.Contains(TEXT("did not touch down")));
	}
	{
		RammsCurb::FExecDryRunOptions Options;
		Options.bProbeAhead = false;
		Options.RidgeHeightCm = -3.0;
		Options.RidgePastEdgeCm = S.Wheels[PlanFL].DDown + 1.0;
		Options.RidgeWidthCm = 30.0;
		const FString					   What = TEXT("Square up, a pit where the fronts land, traces under the wheels only");
		const RammsCurb::FExecDryRunResult R = ExecDisturbed(*this, What, In, S, Options);
		ExpectCleanFault(*this, What, R, TEXT("found no ground where it was to land"), In.Tuning);
		bool bLifted = false;
		for (const FString& L : R.Log)
		{
			bLifted = bLifted || L.Contains(TEXT("lifted clear again"));
		}
		TestTrue(FString::Printf(TEXT("%s: the wheel lifted clear again"), *What), bLifted);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorAttitudeFault,
	"Ramms.Curb.Executor.AttitudeFault",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorAttitudeFault::RunTest(const FString& Parameters)
{
	// The base measured off its planned attitude half-way through the crossing
	// (a slip the model does not have), which the attitude loop -- at most
	// MaxAttitudeCorrectionDeg (3) -- cannot take out: 8 deg, still over
	// AttitudeFaultDeg after it -- a fault within half a second, the robot
	// braked to rest; 3.5 deg, over half of it until the loop has taken it
	// out -- the robot slowed to half the plan's speed meanwhile, the crossing
	// made.
	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
	if (!TestTrue(TEXT("planned"), S.IsValid()))
	{
		return true;
	}
	const RammsCurb::FExecDryRunResult Nominal = RammsCurb::ExecDryRun(In, S);
	if (!TestTrue(TEXT("the crossing undisturbed is made"), Nominal.bDone))
	{
		return true;
	}
	const double At = 0.5 * Nominal.Seconds;
	{
		RammsCurb::FExecDryRunOptions Options;
		Options.KickAtSeconds = At;
		Options.KickDeg = FVector2D(8.0, 0.0);
		const FString					   What = TEXT("Square up, 8 deg off its attitude");
		const RammsCurb::FExecDryRunResult R = ExecDisturbed(*this, What, In, S, Options);
		ExpectCleanFault(*this, What, R, TEXT("off its planned attitude"), In.Tuning, 10.0);
		TestTrue(FString::Printf(TEXT("%s: within half a second (%.2f s after)"), *What, R.FaultAt - At), R.FaultAt - At <= 0.5);
	}
	{
		RammsCurb::FExecDryRunOptions Options;
		Options.KickAtSeconds = At;
		Options.KickDeg = FVector2D(0.0, 3.5);
		const FString					   What = TEXT("Square up, 3.5 deg off its attitude");
		const RammsCurb::FExecDryRunResult R = ExecDisturbed(*this, What, In, S, Options);
		TestTrue(FString::Printf(TEXT("%s: done, no fault (%s)"), *What, *R.FaultWhy), R.bDone && !R.bFaulted);
		TestTrue(FString::Printf(TEXT("%s: slowed for it (%d ticks)"), *What, R.Counts.AttitudeSlowTicks), R.Counts.AttitudeSlowTicks > 0);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorHaltBrakes,
	"Ramms.Curb.Executor.HaltBrakes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorHaltBrakes::RunTest(const FString& Parameters)
{
	// Halted (an abort) at points through the crossing, up and down: the robot
	// braked to rest at Deceleration -- not snapped, and never harder than
	// twice it -- no wheel on the ground in its window, none into the terrain,
	// upright; not a fault.
	for (const bool bUp : { true, false })
	{
		const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Oblique30"), bUp));
		const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
		if (!TestTrue(TEXT("planned"), S.IsValid()))
		{
			continue;
		}
		const RammsCurb::FExecDryRunResult Nominal = RammsCurb::ExecDryRun(In, S);
		for (const double Share : { 0.3, 0.5, 0.7 })
		{
			const FString				  What = FString::Printf(TEXT("Oblique30 %s, halted %.0f %% through"), bUp ? TEXT("up") : TEXT("down"), 100.0 * Share);
			RammsCurb::FExecDryRunOptions Options;
			Options.HaltAtSeconds = Share * Nominal.Seconds;
			const RammsCurb::FExecDryRunResult R = ExecDisturbed(*this, What, In, S, Options);
			TestTrue(FString::Printf(TEXT("%s: halted, at rest, no fault"), *What), R.bHalted && R.bStopped && !R.bFaulted && R.FinalSpeedCmS < 0.5);
			TestTrue(FString::Printf(TEXT("%s: braked at most twice Deceleration (%.0f cm/s^2)"), *What, R.HaltBrakingCmS2),
				R.HaltBrakingCmS2 <= 2.0 * In.Tuning.Deceleration + 1.0);
			TestTrue(FString::Printf(TEXT("%s: upright (%.2f deg)"), *What, R.HaltedInclinationDeg), R.HaltedInclinationDeg < 5.0);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorPlantOnHold,
	"Ramms.Curb.Executor.PlantOnHold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorPlantOnHold::RunTest(const FString& Parameters)
{
	// Halted just after the front corners have begun to lift, short of their
	// windows: with bPlantOnHold they are set down again on the road (the robot
	// then standing on every wheel) -- on it when the halt is over, not left in
	// the air where they were as the robot came to rest; without it they stay
	// up where they are.
	RammsCurb::FPlannerInput	   In = PlanOn(PlanScenario(TEXT("Square"), true));
	const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
	if (!TestTrue(TEXT("planned"), S.IsValid()))
	{
		return true;
	}
	for (const bool bPlant : { true, false })
	{
		In.Tuning.bPlantOnHold = bPlant;
		const FString				  What = FString::Printf(TEXT("Square up, halted as the fronts lift, bPlantOnHold %d"), bPlant ? 1 : 0);
		RammsCurb::FExecDryRunOptions Options;
		Options.HaltAtTravelCm = FMath::Max(S.Wheels[PlanFL].SLiftStart, S.Wheels[PlanFR].SLiftStart) + 3.0;
		const RammsCurb::FExecDryRunResult R = ExecDisturbed(*this, What, In, S, Options);
		bool							   bSetDown = false;
		for (const FString& L : R.Log)
		{
			bSetDown = bSetDown || (L.Contains(TEXT("front_")) && L.Contains(TEXT("set down on hold over the near surface")));
		}
		TestTrue(FString::Printf(TEXT("%s: halted at rest"), *What), R.bHalted && R.bStopped && !R.bFaulted);
		TestEqual(FString::Printf(TEXT("%s: a front corner set down on hold"), *What), bSetDown, bPlant);
		if (R.FinalBottomCm.Num() == In.Legs.Num())
		{
			const double Highest = FMath::Max(R.FinalBottomCm[PlanFL], R.FinalBottomCm[PlanFR]);
			if (bPlant)
			{
				TestTrue(FString::Printf(TEXT("%s: the front corners on the road when the halt is over (bottoms %+.2f / %+.2f cm)"), *What,
							 R.FinalBottomCm[PlanFL], R.FinalBottomCm[PlanFR]),
					Highest <= RammsCurb::TouchCm);
			}
			else
			{
				TestTrue(FString::Printf(TEXT("%s: a front corner still up (bottoms %+.2f / %+.2f cm)"), *What, R.FinalBottomCm[PlanFL],
							 R.FinalBottomCm[PlanFR]),
					Highest > RammsCurb::TouchCm);
			}
		}
	}
	{
		// Standing in a drive stance with the front corners 6 cm up, halted at
		// once: the robot is at rest long before the corners are down. The halt
		// is over only once they are on the road where they were sent -- not as
		// the robot came to rest, which would leave them where their commands
		// then were, in the air. (The base settles 1.1 deg nose up on the first
		// tick; the corners' targets follow the road under them.)
		RammsCurb::FPlannerInput	   Up = FrontsUp(PlanOn(PlanScenario(TEXT("Square"), true)), 6.0);
		const RammsCurb::FCurbSchedule SU = RammsCurb::PlanCrossing(Up);
		if (TestTrue(TEXT("planned with the front corners up"), SU.IsValid()))
		{
			Up.Tuning.bPlantOnHold = true;
			const FString				  What = TEXT("Square up, the front corners 6 cm up, halted at once, bPlantOnHold 1");
			RammsCurb::FExecDryRunOptions Options;
			Options.HaltAtSeconds = 0.0;
			const RammsCurb::FExecDryRunResult R = ExecDisturbed(*this, What, Up, SU, Options);
			TestTrue(FString::Printf(TEXT("%s: halted at rest"), *What), R.bHalted && R.bStopped && !R.bFaulted);
			if (TestEqual(FString::Printf(TEXT("%s: a bottom for every wheel"), *What), R.FinalBottomCm.Num(), Up.Legs.Num()))
			{
				TestTrue(FString::Printf(TEXT("%s: the front corners on the road (bottoms %+.2f / %+.2f cm)"), *What, R.FinalBottomCm[PlanFL],
							 R.FinalBottomCm[PlanFR]),
					FMath::Max(R.FinalBottomCm[PlanFL], R.FinalBottomCm[PlanFR]) <= RammsCurb::TouchCm);
			}
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbExecutorPredictedLiftMargin,
	"Ramms.Curb.Executor.PredictedLiftMargin",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbExecutorPredictedLiftMargin::RunTest(const FString& Parameters)
{
	// The live lift permit's look ahead: a centre wheel lifting on the plan's
	// position, the wheels the plan has up then up, nothing late -- the plan's
	// own sets, nothing to check. With its own side's front corner still up
	// past where the plan has it down -- a same-side corner and centre off the
	// ground together -- the plan's poses over its lift stand under
	// MinStabilityMargin, and it may not lift.
	const RammsCurb::FPlannerInput In = PlanOn(PlanScenario(TEXT("Square"), true));
	const RammsCurb::FCurbSchedule S = RammsCurb::PlanCrossing(In);
	if (!TestTrue(TEXT("planned"), S.IsValid()))
	{
		return true;
	}
	const RammsCurb::FPlanModel	 M(In);
	const RammsCurb::FWheelPlan& C = S.Wheels[PlanCL];
	TestTrue(FString::Printf(TEXT("the left centre lifts after the left front has taken load (%.1f, %.1f)"), C.SLiftStart, S.Wheels[PlanFL].SLoaded),
		C.SLiftStart >= S.Wheels[PlanFL].SLoaded);
	double		   At = 0.0;
	const double   Never = -TNumericLimits<double>::Max();
	TArray<double> UpUntil;
	UpUntil.Init(Never, 6);
	const double None = RammsCurb::PredictedLiftMargin(M, S, PlanCL, C.SLiftStart, UpUntil, At);
	TestEqual(TEXT("on the plan's position, nothing late: nothing to check"), None, TNumericLimits<double>::Max());
	UpUntil[PlanFL] = C.SLiftStart + 20.0;
	const double Late = RammsCurb::PredictedLiftMargin(M, S, PlanCL, C.SLiftStart, UpUntil, At);
	TestTrue(FString::Printf(TEXT("the left front up 20 cm more: %.2f cm at %.0f cm, under MinStabilityMargin %.1f"), Late, At,
				 In.Tuning.MinStabilityMargin),
		Late < In.Tuning.MinStabilityMargin && At < C.SLiftStart + 20.0);
	UpUntil[PlanFL] = C.SLiftStart - 1.0;
	const double Down = RammsCurb::PredictedLiftMargin(M, S, PlanCL, C.SLiftStart, UpUntil, At);
	TestEqual(TEXT("the left front down before the lift begins: nothing to check"), Down, TNumericLimits<double>::Max());

	// A plan that lifts the left rear the moment the left centre has taken
	// load: from the left centre's own lift position (either sample parity),
	// nothing late, the sets are the plan's own up to its landing -- the
	// sample where it has taken load and the rear is up is not one with it up.
	RammsCurb::FCurbSchedule Tight = S;
	for (RammsCurb::FScheduleSample& Sm : Tight.Samples)
	{
		if (Sm.S >= C.SLoaded && Sm.S < C.SLoaded + 10.0)
		{
			Sm.AirborneMask |= 1u << PlanRL;
		}
	}
	UpUntil[PlanFL] = Never;
	for (const double Off : { 0.0, Tight.SampleCm })
	{
		const double Next = RammsCurb::PredictedLiftMargin(M, Tight, PlanCL, C.SLiftStart + Off, UpUntil, At);
		TestEqual(FString::Printf(TEXT("the left rear lifting as the left centre takes load (from %.1f): nothing to check"), C.SLiftStart + Off), Next,
			TNumericLimits<double>::Max());
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
