// Tests for the leg kinematics the curb controller plans with.
//
// URamms4BarKinematics and URamms5BarKinematics are pure functions of a spec,
// so they are checked against themselves: an inverse that does not land where
// the forward says it does, or a height range that is not the range the crank
// sweeps, plans a step the legs cannot take. The legs here are made up -- a
// parallelogram crank-rocker and a scaled-down copy of the 5-bar spec's
// defaults -- rather than any robot's.

#include "CoreMinimal.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/AutomationTest.h"

#include "Ramms4BarKinematics.h"
#include "Ramms5BarKinematics.h"
#include "Ramms5BarLinkageController.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	/** A parallelogram crank-rocker: crank and rocker are equal and parallel,
	 *  so the rocker turns with the crank and the wheel, 25 cm out on it, rises
	 *  and falls monotonically over +-0.6 rad. */
	FRamms4BarLegSpec MakeParallelogramLeg()
	{
		FRamms4BarLegSpec Spec;
		Spec.CrankMotor = TEXT("test_crank");
		Spec.CrankPivot = FVector2D(0.0, 0.0);
		Spec.CrankArm = FVector2D(15.0, 0.0);
		Spec.CrankAngleSign = 1.0f;
		Spec.CouplerLength = 6.0f;
		Spec.RockerPivot = FVector2D(0.0, -6.0);
		Spec.RockerPin = FVector2D(15.0, 0.0);
		Spec.RockerAngleSign = 1.0f;
		Spec.WheelOnRocker = FVector2D(25.0, -10.0);
		Spec.WheelOffset = FVector2D::ZeroVector;
		Spec.CrankRange = FVector2D(-0.6, 0.6);
		Spec.RockerRange = FVector2D(-1.0, 1.0);
		return Spec;
	}

	/** The 5-bar spec's defaults scaled by 0.8: the same assembly (angles,
	 *  signs, branches) on made-up lengths. */
	FRamms5BarLinkageSpec MakeFiveBar()
	{
		FRamms5BarLinkageSpec Spec;
		const float			  S = 0.8f;
		Spec.PivotA *= S;
		Spec.PivotB *= S;
		Spec.ProximalLengthA *= S;
		Spec.DistalLengthA *= S;
		Spec.ProximalLengthB *= S;
		Spec.DistalLengthB *= S;
		return Spec;
	}

	// A made-up 5-bar whose knee stops bind: 12 cm proximal links on pivots
	// 10 cm apart at z = 20, 18 cm distal links, mirror-symmetric about x = 0,
	// knee A ahead and knee B behind, the endpoint below them.
	constexpr double KneeTestPivotX = 5.0;
	constexpr double KneeTestPivotZ = 20.0;
	constexpr double KneeTestProximal = 12.0;
	constexpr double KneeTestDistal = 18.0;
	// Each knee may fold to 2.4 rad from straight and must keep 0.3 rad of bend.
	constexpr double KneeTestMostBend = 2.4;
	constexpr double KneeTestLeastBend = 0.3;

	/** The symmetric test linkage. With bKneeStops false it keeps the spec's
	 *  default knee ranges, which bound nothing. */
	FRamms5BarLinkageSpec MakeKneeLimitedFiveBar(bool bKneeStops)
	{
		FRamms5BarLinkageSpec Spec;
		Spec.ProximalMotorA = TEXT("test_hip_a");
		Spec.ProximalMotorB = TEXT("test_hip_b");
		Spec.PivotA = FVector2D(KneeTestPivotX, KneeTestPivotZ);
		Spec.PivotB = FVector2D(-KneeTestPivotX, KneeTestPivotZ);
		Spec.ProximalLengthA = Spec.ProximalLengthB = static_cast<float>(KneeTestProximal);
		Spec.DistalLengthA = Spec.DistalLengthB = static_cast<float>(KneeTestDistal);
		// Hinge A about +Y (joint angle turns the link clockwise in x-z),
		// hinge B about -Y, both links level at joint angle 0.
		Spec.ZeroDirA = 0.0f;
		Spec.AngleSignA = -1.0f;
		Spec.ZeroDirB = UE_PI;
		Spec.AngleSignB = 1.0f;
		Spec.bElbowUpA = true;
		Spec.bElbowUpB = false;
		Spec.bFlipEndpointSide = false;
		if (bKneeStops)
		{
			// Elbow-up bends negative, elbow-down positive (see the spec).
			Spec.DistalRangeA = FVector2D(-KneeTestMostBend, -KneeTestLeastBend);
			Spec.DistalRangeB = FVector2D(KneeTestLeastBend, KneeTestMostBend);
		}
		return Spec;
	}

	/** The analytic height at x = 0 where both knees of the test linkage bend
	 *  Bend rad from straight: by symmetry they bend alike there, and the
	 *  pivot-to-endpoint distance d obeys d^2 = l1^2 + l2^2 + 2 l1 l2 cos(Bend). */
	double KneeBendHeightAtZeroX(double Bend)
	{
		const double D2 = KneeTestProximal * KneeTestProximal + KneeTestDistal * KneeTestDistal
			+ 2.0 * KneeTestProximal * KneeTestDistal * FMath::Cos(Bend);
		return KneeTestPivotZ - FMath::Sqrt(D2 - KneeTestPivotX * KneeTestPivotX);
	}

	bool ReachableAt(const FRamms5BarLinkageSpec& Spec, double X, double Z)
	{
		bool bReachable = false;
		URamms5BarKinematics::SolveIK(Spec, FVector2D(X, Z), bReachable);
		return bReachable;
	}

	/** The edge of the reachable span along x = 0 between a height that
	 *  reaches and one that does not, by bisection. */
	double ReachEdgeAtZeroX(const FRamms5BarLinkageSpec& Spec, double Inside, double Outside)
	{
		for (int32 i = 0; i < 60; ++i)
		{
			const double Mid = 0.5 * (Inside + Outside);
			(ReachableAt(Spec, 0.0, Mid) ? Inside : Outside) = Mid;
		}
		return Inside;
	}

	/** Spec with each proximal link's zero direction turned so that joint
	 *  angles (0, 0) put the endpoint at TargetXZ. A linkage controller with
	 *  no robot base reads its live joint angles as zero, and its reachable
	 *  height scan starts from the live endpoint, so this is where that scan
	 *  starts. What is reachable is geometric, so nothing else changes. */
	FRamms5BarLinkageSpec ZeroedAt(const FRamms5BarLinkageSpec& Spec, FVector2D TargetXZ)
	{
		bool				  bReachable = false;
		const FVector2D		  Angles = URamms5BarKinematics::SolveIK(Spec, TargetXZ, bReachable);
		FRamms5BarLinkageSpec Out = Spec;
		Out.ZeroDirA = static_cast<float>(Spec.ZeroDirA + Spec.AngleSignA * Angles.X);
		Out.ZeroDirB = static_cast<float>(Spec.ZeroDirB + Spec.AngleSignB * Angles.Y);
		return Out;
	}

	/**
	 * A bare game world holding one actor with a 5-bar linkage controller
	 * whose inline spec is Spec, registered and begun play, so the controller
	 * resolves its spec the way it does on a robot. There is no robot base,
	 * so no motor ControlRange applies and the knee ranges are the only
	 * limit beyond the links themselves.
	 */
	struct FLinkageControllerFixture
	{
		UWorld*						 World = nullptr;
		AActor*						 Actor = nullptr;
		URamms5BarLinkageController* Controller = nullptr;

		FLinkageControllerFixture(const FRamms5BarLinkageSpec& Spec, FVector2D HeightScanLimits, FVector2D TranslationScanLimits)
		{
			World = UWorld::CreateWorld(EWorldType::Game, /*bInformEngineOfWorld=*/false);
			if (World == nullptr)
			{
				return;
			}
			GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
			Actor = World->SpawnActor<AActor>();
			if (Actor == nullptr)
			{
				return;
			}
			Controller = NewObject<URamms5BarLinkageController>(Actor, TEXT("KneeTestLinkage"));
			Controller->Linkage = Spec;
			Controller->HeightScanLimits = HeightScanLimits;
			Controller->TranslationScanLimits = TranslationScanLimits;
			Controller->RegisterComponent();
			Actor->DispatchBeginPlay();
		}

		~FLinkageControllerFixture()
		{
			if (World != nullptr)
			{
				if (Actor != nullptr)
				{
					Actor->Destroy();
				}
				GEngine->DestroyWorldContext(World);
				World->DestroyWorld(false);
			}
		}

		bool IsReady() const { return Controller != nullptr && Controller->HasBegunPlay(); }
	};

	bool ControllerSolves(const URamms5BarLinkageController& Controller, double X, double Z)
	{
		bool bReachable = false;
		Controller.SolveTarget(FVector2D(X, Z), bReachable);
		return bReachable;
	}
} // namespace

// ---------------------------------------------------------------------------
// 4-bar
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRamms4BarZeroPoseIsTheAssembly,
	"Ramms.Kinematics.FourBar.ZeroPoseIsTheAssembly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRamms4BarZeroPoseIsTheAssembly::RunTest(const FString& Parameters)
{
	// At crank 0 the loop closes in the pose the spec was measured in, rocker
	// at 0: the wheel is where the spec says it is.
	const FRamms4BarLegSpec Spec = MakeParallelogramLeg();
	bool					bValid = false;
	const FVector2D			Wheel = URamms4BarKinematics::ComputeWheelCenter(Spec, 0.0f, bValid);
	TestTrue(TEXT("the zero pose closes"), bValid);
	TestTrue(TEXT("the wheel sits at RockerPivot + WheelOnRocker"),
		Wheel.Equals(Spec.RockerPivot + Spec.WheelOnRocker + Spec.WheelOffset, 0.01));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRamms4BarHeightRoundTrips,
	"Ramms.Kinematics.FourBar.HeightRoundTrips",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRamms4BarHeightRoundTrips::RunTest(const FString& Parameters)
{
	// Every height inside the range solves to a crank angle whose wheel is at
	// that height, and at the fore/aft position the solve reports.
	const FRamms4BarLegSpec Spec = MakeParallelogramLeg();
	const FVector2D			Range = URamms4BarKinematics::GetHeightRange(Spec);
	TestTrue(TEXT("the range is not empty"), Range.Y > Range.X + 1.0);
	for (int32 i = 1; i < 20; ++i)
	{
		const float		Z = FMath::Lerp(Range.X, Range.Y, i / 20.0f);
		bool			bReachable = false;
		float			WheelX = 0.0f;
		const float		Crank = URamms4BarKinematics::SolveCrankForHeight(Spec, Z, bReachable, WheelX);
		bool			bValid = false;
		const FVector2D Wheel = URamms4BarKinematics::ComputeWheelCenter(Spec, Crank, bValid);
		if (!bReachable || !bValid || FMath::Abs(Wheel.Y - Z) > 0.01 || FMath::Abs(Wheel.X - WheelX) > 0.01)
		{
			AddError(FString::Printf(TEXT("z %.3f: crank %.4f reaches (%.3f, %.3f), reported x %.3f, reachable %d valid %d"),
				Z, Crank, Wheel.X, Wheel.Y, WheelX, bReachable, bValid));
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRamms4BarRangeIsTheSweep,
	"Ramms.Kinematics.FourBar.RangeIsTheSweep",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRamms4BarRangeIsTheSweep::RunTest(const FString& Parameters)
{
	// The height range is what the crank actually sweeps over CrankRange, not
	// what its two ends reach.
	const FRamms4BarLegSpec Spec = MakeParallelogramLeg();
	double					Lo = TNumericLimits<double>::Max(), Hi = TNumericLimits<double>::Lowest();
	for (int32 i = 0; i <= 600; ++i)
	{
		bool			bValid = false;
		const FVector2D Wheel =
			URamms4BarKinematics::ComputeWheelCenter(Spec, FMath::Lerp(Spec.CrankRange.X, Spec.CrankRange.Y, i / 600.0f), bValid);
		if (bValid)
		{
			Lo = FMath::Min(Lo, Wheel.Y);
			Hi = FMath::Max(Hi, Wheel.Y);
		}
	}
	const FVector2D Range = URamms4BarKinematics::GetHeightRange(Spec);
	TestNearlyEqual(TEXT("lowest height"), Range.X, Lo, 0.05);
	TestNearlyEqual(TEXT("highest height"), Range.Y, Hi, 0.05);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRamms4BarOutOfReachClampsToTheNearestEnd,
	"Ramms.Kinematics.FourBar.OutOfReachClampsToTheNearestEnd",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRamms4BarOutOfReachClampsToTheNearestEnd::RunTest(const FString& Parameters)
{
	// A height past either end is refused, and the angle returned is the end of
	// the crank's range that gets nearest -- what a planner holds the leg at.
	const FRamms4BarLegSpec Spec = MakeParallelogramLeg();
	const FVector2D			Range = URamms4BarKinematics::GetHeightRange(Spec);
	for (const float Z : { Range.X - 5.0f, Range.Y + 5.0f })
	{
		bool		bReachable = true;
		float		WheelX = 0.0f;
		const float Crank = URamms4BarKinematics::SolveCrankForHeight(Spec, Z, bReachable, WheelX);
		TestFalse(*FString::Printf(TEXT("z %.1f is out of reach"), Z), bReachable);
		bool			bValid = false;
		const FVector2D Wheel = URamms4BarKinematics::ComputeWheelCenter(Spec, Crank, bValid);
		const double	NearestEnd = Z < Range.X ? Range.X : Range.Y;
		TestNearlyEqual(*FString::Printf(TEXT("z %.1f holds the wheel at the nearest end"), Z), Wheel.Y, NearestEnd, 0.05);
	}
	return true;
}

// ---------------------------------------------------------------------------
// 5-bar
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRamms5BarEndpointRoundTrips,
	"Ramms.Kinematics.FiveBar.EndpointRoundTrips",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRamms5BarEndpointRoundTrips::RunTest(const FString& Parameters)
{
	// Joint angles around the assembly pose put the endpoint somewhere; asking
	// the inverse for that point gives the same angles back.
	const FRamms5BarLinkageSpec Spec = MakeFiveBar();
	int32						Checked = 0;
	for (int32 a = -3; a <= 3; ++a)
	{
		for (int32 b = -3; b <= 3; ++b)
		{
			const FVector2D Angles(a * 0.08, b * 0.08);
			bool			bValid = false;
			const FVector2D Endpoint = URamms5BarKinematics::ComputeEndpoint(Spec, Angles, bValid);
			if (!bValid)
			{
				continue;
			}
			++Checked;
			bool			bReachable = false;
			const FVector2D Back = URamms5BarKinematics::SolveIK(Spec, Endpoint, bReachable);
			if (!bReachable || !Back.Equals(Angles, 1e-3))
			{
				AddError(FString::Printf(TEXT("angles (%.3f, %.3f) -> endpoint (%.2f, %.2f) -> angles (%.4f, %.4f), reachable %d"),
					Angles.X, Angles.Y, Endpoint.X, Endpoint.Y, Back.X, Back.Y, bReachable));
			}
		}
	}
	TestTrue(TEXT("the assembly pose and most of its neighbourhood are valid"), Checked >= 25);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRamms5BarRefusesWhatItCannotReach,
	"Ramms.Kinematics.FiveBar.RefusesWhatItCannotReach",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRamms5BarRefusesWhatItCannotReach::RunTest(const FString& Parameters)
{
	// A target further from both pivots than the arms reach is refused.
	const FRamms5BarLinkageSpec Spec = MakeFiveBar();
	const double				Reach = FMath::Max(Spec.ProximalLengthA + Spec.DistalLengthA, Spec.ProximalLengthB + Spec.DistalLengthB);
	const FVector2D				Mid = 0.5 * (Spec.PivotA + Spec.PivotB);
	bool						bReachable = true;
	URamms5BarKinematics::SolveIK(Spec, Mid - FVector2D(0.0, Reach + 10.0), bReachable);
	TestFalse(TEXT("a target beyond both arms' reach"), bReachable);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRamms5BarKneeLimits,
	"Ramms.Kinematics.FiveBar.KneeLimits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRamms5BarKneeLimits::RunTest(const FString& Parameters)
{
	// With knee stops the endpoint reaches exactly as far up and down x = 0 as
	// the stops allow -- the analytic heights where both knees bend their
	// most and least -- and not as far as the free links could: past a stop
	// the IK refuses a target it would otherwise accept.
	const FRamms5BarLinkageSpec Limited = MakeKneeLimitedFiveBar(true);
	const FRamms5BarLinkageSpec Free = MakeKneeLimitedFiveBar(false);
	const double				Ceiling = KneeBendHeightAtZeroX(KneeTestMostBend);
	const double				Floor = KneeBendHeightAtZeroX(KneeTestLeastBend);

	// The bisection brackets: a height both specs reach, and one beyond any
	// reach (closer to the pivots than the folded links, or further than the
	// straight ones).
	const double Mid = 0.5 * (Ceiling + Floor);
	const double Above = KneeTestPivotZ - 0.5;
	const double Below = KneeTestPivotZ - (KneeTestProximal + KneeTestDistal) - 1.0;
	if (!TestTrue(TEXT("the knee-limited linkage reaches the middle of its span"), ReachableAt(Limited, 0.0, Mid))
		|| !TestFalse(TEXT("nothing reaches next to the pivots"), ReachableAt(Free, 0.0, Above))
		|| !TestFalse(TEXT("nothing reaches below the straight links"), ReachableAt(Free, 0.0, Below)))
	{
		return false;
	}

	TestNearlyEqual(TEXT("knee-limited ceiling at x = 0 (cm)"), ReachEdgeAtZeroX(Limited, Mid, Above), Ceiling, 1e-3);
	TestNearlyEqual(TEXT("knee-limited floor at x = 0 (cm)"), ReachEdgeAtZeroX(Limited, Mid, Below), Floor, 1e-3);

	// The stops are what binds: without them the same links reach well past
	// both heights (the free ceiling is where the knees fold flat, the free
	// floor where they straighten).
	const double FreeCeiling = ReachEdgeAtZeroX(Free, Mid, Above);
	const double FreeFloor = ReachEdgeAtZeroX(Free, Mid, Below);
	TestTrue(*FString::Printf(TEXT("the free ceiling %.3f is well above the knee-limited %.3f"), FreeCeiling, Ceiling),
		FreeCeiling > Ceiling + 5.0);
	TestTrue(*FString::Printf(TEXT("the free floor %.3f is below the knee-limited %.3f"), FreeFloor, Floor),
		FreeFloor < Floor - 0.2);

	// Just past a stop: refused with the stop, accepted without it, and the
	// angles are the same solution either way (refusing does not move them).
	for (const double Z : { Ceiling + 0.5, Floor - 0.1 })
	{
		bool			bWithStops = true;
		bool			bWithout = false;
		const FVector2D AnglesWith = URamms5BarKinematics::SolveIK(Limited, FVector2D(0.0, Z), bWithStops);
		const FVector2D AnglesWithout = URamms5BarKinematics::SolveIK(Free, FVector2D(0.0, Z), bWithout);
		TestFalse(*FString::Printf(TEXT("z %.2f is past a knee stop"), Z), bWithStops);
		TestTrue(*FString::Printf(TEXT("z %.2f is reachable with free knees"), Z), bWithout);
		TestTrue(*FString::Printf(TEXT("z %.2f: the refused solution is still the solution"), Z),
			AnglesWith.Equals(AnglesWithout, 1e-9));
	}
	TestTrue(TEXT("just inside the ceiling is reachable"), ReachableAt(Limited, 0.0, Ceiling - 0.05));
	TestTrue(TEXT("just inside the floor is reachable"), ReachableAt(Limited, 0.0, Floor + 0.05));

	// Off the centreline too, the stops only ever take targets away: anything
	// the knee-limited linkage reaches, the free one does.
	int32 Taken = 0;
	for (double Z = Below; Z <= Above; Z += 0.5)
	{
		for (double X = -20.0; X <= 20.0; X += 0.5)
		{
			const bool bLimited = ReachableAt(Limited, X, Z);
			const bool bFree = ReachableAt(Free, X, Z);
			if (bLimited && !bFree)
			{
				AddError(FString::Printf(TEXT("(%.1f, %.1f) reaches only with knee stops"), X, Z));
			}
			Taken += (bFree && !bLimited) ? 1 : 0;
		}
	}
	TestTrue(TEXT("the stops remove part of the region"), Taken > 0);

	// The range is read modulo 2 pi: the same stops written a turn round
	// bound the same knee.
	FRamms5BarLinkageSpec Turned = Limited;
	Turned.DistalRangeA += FVector2D(UE_DOUBLE_TWO_PI, UE_DOUBLE_TWO_PI);
	Turned.DistalRangeB -= FVector2D(UE_DOUBLE_TWO_PI, UE_DOUBLE_TWO_PI);
	TestNearlyEqual(TEXT("ceiling with the ranges a turn round"), ReachEdgeAtZeroX(Turned, Mid, Above), Ceiling, 1e-3);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRamms5BarKneeLimitsInTheController,
	"Ramms.Kinematics.FiveBar.KneeLimitsInTheController",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRamms5BarKneeLimitsInTheController::RunTest(const FString& Parameters)
{
	// The same stops seen through the linkage controller, on a spec it
	// resolved itself at BeginPlay: SolveTarget refuses past them, the height
	// range the control surface scans stops inside them, and the extent and
	// outline it advertises end on them.
	const double Ceiling = KneeBendHeightAtZeroX(KneeTestMostBend);
	const double Floor = KneeBendHeightAtZeroX(KneeTestLeastBend);
	const double Mid = 0.5 * (Ceiling + Floor);

	// Heights below the pivots only: this linkage also reaches a second lobe
	// above them, which a height extent would otherwise take in. Fore/aft, a
	// strip about x = 0, so the extent is the span at x = 0, where both stops
	// bind at once and the region is at its highest and lowest.
	const FVector2D			  HeightLimits(KneeTestPivotZ - (KneeTestProximal + KneeTestDistal) - 1.0, KneeTestPivotZ - 0.5);
	const FVector2D			  Strip(-0.5, 0.5);
	FLinkageControllerFixture Limited(ZeroedAt(MakeKneeLimitedFiveBar(true), FVector2D(0.0, Mid)), HeightLimits, Strip);
	FLinkageControllerFixture Free(ZeroedAt(MakeKneeLimitedFiveBar(false), FVector2D(0.0, Mid)), HeightLimits, Strip);
	if (!TestTrue(TEXT("the knee-limited controller began play"), Limited.IsReady())
		|| !TestTrue(TEXT("the free controller began play"), Free.IsReady()))
	{
		return false;
	}
	const URamms5BarLinkageController& L = *Limited.Controller;
	const URamms5BarLinkageController& F = *Free.Controller;

	// It is using the inline spec: at zero joint angles that spec, and not the
	// struct's defaults, puts the endpoint at (0, Mid).
	bool			bLive = false;
	const FVector2D Live = L.GetCurrentEndpointChecked(bLive);
	TestTrue(TEXT("the live endpoint closes"), bLive);
	TestTrue(*FString::Printf(TEXT("the live endpoint (%.4f, %.4f) is the inline spec's (0, %.4f)"), Live.X, Live.Y, Mid),
		Live.Equals(FVector2D(0.0, Mid), 1e-3));

	// SolveTarget: refused just past either stop and accepted just inside;
	// with free knees the same targets are accepted.
	TestFalse(TEXT("SolveTarget refuses 0.5 cm above the knee-limited ceiling"), ControllerSolves(L, 0.0, Ceiling + 0.5));
	TestFalse(TEXT("SolveTarget refuses 0.1 cm below the knee-limited floor"), ControllerSolves(L, 0.0, Floor - 0.1));
	TestTrue(TEXT("SolveTarget accepts 0.05 cm under the ceiling"), ControllerSolves(L, 0.0, Ceiling - 0.05));
	TestTrue(TEXT("SolveTarget accepts 0.05 cm over the floor"), ControllerSolves(L, 0.0, Floor + 0.05));
	TestTrue(TEXT("with free knees SolveTarget accepts 0.5 cm above that ceiling"), ControllerSolves(F, 0.0, Ceiling + 0.5));
	TestTrue(TEXT("with free knees SolveTarget accepts 0.1 cm below that floor"), ControllerSolves(F, 0.0, Floor - 0.1));

	// GetReachableHeightRange(0) walks out from the live height in
	// HeightScanStep steps, so each end lies within one step inside a stop.
	const double	Step = L.HeightScanStep;
	bool			bRange = false;
	const FVector2D Range = L.GetReachableHeightRange(0.0f, bRange);
	TestTrue(TEXT("the height range at x = 0 is valid"), bRange);
	TestTrue(*FString::Printf(TEXT("the height range's top %.4f is within a %.2f cm step under the ceiling %.4f"), Range.Y, Step, Ceiling),
		Range.Y <= Ceiling + 1e-3 && Range.Y > Ceiling - Step - 1e-3);
	TestTrue(*FString::Printf(TEXT("the height range's bottom %.4f is within a %.2f cm step over the floor %.4f"), Range.X, Step, Floor),
		Range.X >= Floor - 1e-3 && Range.X < Floor + Step + 1e-3);
	bool			bFreeRange = false;
	const FVector2D FreeRange = F.GetReachableHeightRange(0.0f, bFreeRange);
	TestTrue(*FString::Printf(TEXT("with free knees the height range's top %.4f is well above the ceiling"), FreeRange.Y),
		bFreeRange && FreeRange.Y > Ceiling + 5.0);

	// GetReachableExtent and the outline come from the measured region, whose
	// edges are bisected, so they land on the analytic heights themselves.
	bool			bExtent = false;
	const FVector2D Extent = L.GetReachableExtent(/*bHeight=*/true, bExtent);
	TestTrue(TEXT("the height extent is valid"), bExtent);
	TestNearlyEqual(TEXT("the advertised height extent's top is the knee-limited ceiling (cm)"), Extent.Y, Ceiling, 1e-3);
	TestNearlyEqual(TEXT("the advertised height extent's bottom is the knee-limited floor (cm)"), Extent.X, Floor, 1e-3);
	bool			bFreeExtent = false;
	const FVector2D FreeExtent = F.GetReachableExtent(/*bHeight=*/true, bFreeExtent);
	TestTrue(*FString::Printf(TEXT("with free knees the extent's top %.4f is well above the ceiling"), FreeExtent.Y),
		bFreeExtent && FreeExtent.Y > Ceiling + 5.0);

	bool					bOutline = false;
	const TArray<FVector2D> Outline = L.GetReachableOutline(bOutline);
	TestTrue(TEXT("the outline is a polygon"), bOutline && Outline.Num() >= 3);
	double OutlineLo = TNumericLimits<double>::Max();
	double OutlineHi = TNumericLimits<double>::Lowest();
	for (const FVector2D& Point : Outline)
	{
		OutlineLo = FMath::Min(OutlineLo, static_cast<double>(Point.Y));
		OutlineHi = FMath::Max(OutlineHi, static_cast<double>(Point.Y));
	}
	TestNearlyEqual(TEXT("the outline's top is the knee-limited ceiling (cm)"), OutlineHi, Ceiling, 1e-3);
	TestNearlyEqual(TEXT("the outline's bottom is the knee-limited floor (cm)"), OutlineLo, Floor, 1e-3);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
