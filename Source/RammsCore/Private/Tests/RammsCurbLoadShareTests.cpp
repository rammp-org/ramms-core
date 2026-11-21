// Tests for the curb controller's load sharing (Private/Curb/RammsCurbLoadShare):
// how a leg's motors carry its wheel, and how the weight shares out among the
// wheels carrying it. Every number is made up -- the parallelogram crank-rocker
// of the leg kinematics tests, round wheel positions and a round weight -- no
// robot's own.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Curb/RammsCurbLoadShare.h"
#include "Ramms4BarKinematics.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	/** The leg kinematics tests' parallelogram crank-rocker: the wheel 25 cm
	 *  out on a rocker that turns with the crank over +-0.6 rad. */
	FRamms4BarLegSpec MakeLoadShareLeg()
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

	double LoadShareSum(const TArray<double>& V)
	{
		double S = 0.0;
		for (const double X : V)
		{
			S += X;
		}
		return S;
	}

	/** The shares' moment about the origin over the weight: where they put
	 *  the centre of mass. */
	FVector2D LoadShareCentre(TConstArrayView<FVector2D> P, const TArray<double>& F)
	{
		FVector2D M = FVector2D::ZeroVector;
		double	  W = 0.0;
		for (int32 i = 0; i < P.Num(); ++i)
		{
			M += P[i] * F[i];
			W += F[i];
		}
		return W > 0.0 ? M / W : FVector2D::ZeroVector;
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbLoadShareFourBarRate,
	"Ramms.Curb.LoadShare.FourBarRate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbLoadShareFourBarRate::RunTest(const FString& Parameters)
{
	// The wheel's rate per radian of crank is the slope of the height (and the
	// fore/aft) the crank reaches: a finer difference of the height solve's
	// round trips, anywhere in the range and at its ends.
	const FRamms4BarLegSpec Spec = MakeLoadShareLeg();
	for (const double Crank : { -0.6, -0.598, -0.3, 0.0, 0.25, 0.598, 0.6 })
	{
		FVector2D Rate = FVector2D::ZeroVector;
		if (!RammsCurb::FourBarWheelRate(Spec, Crank, Rate))
		{
			AddError(FString::Printf(TEXT("crank %.3f: no rate"), Crank));
			continue;
		}
		// Heights a hair either side of the crank's (just inside its range: the
		// solve does not reach an end's height exactly), solved
		// back to their cranks.
		bool			bA = false, bB = false;
		const FVector2D PA = URamms4BarKinematics::ComputeWheelCenter(Spec, static_cast<float>(FMath::Max(-0.599, Crank - 0.002)), bA);
		const FVector2D PB = URamms4BarKinematics::ComputeWheelCenter(Spec, static_cast<float>(FMath::Min(0.599, Crank + 0.002)), bB);
		bool			bLo = false, bHi = false;
		float			XLo = 0.0f, XHi = 0.0f;
		const double	CLo = URamms4BarKinematics::SolveCrankForHeight(Spec, static_cast<float>(PA.Y), bLo, XLo);
		const double	CHi = URamms4BarKinematics::SolveCrankForHeight(Spec, static_cast<float>(PB.Y), bHi, XHi);
		if (!bA || !bB || !bLo || !bHi || FMath::Abs(CHi - CLo) < 1e-6)
		{
			AddError(FString::Printf(TEXT("crank %.3f: the height solve does not reach about it"), Crank));
			continue;
		}
		const double DzDc = (PB.Y - PA.Y) / (CHi - CLo);
		const double DxDc = (XHi - XLo) / (CHi - CLo);
		if (FMath::Abs(Rate.Y - DzDc) > 0.02 * FMath::Max(1.0, FMath::Abs(DzDc)) || FMath::Abs(Rate.X - DxDc) > 0.05 * FMath::Max(1.0, FMath::Abs(DzDc)))
		{
			AddError(FString::Printf(TEXT("crank %.3f: rate (%.3f, %.3f) cm/rad, the height solve's (%.3f, %.3f)"), Crank, Rate.X, Rate.Y, DxDc, DzDc));
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbLoadSharePressTorqueCap,
	"Ramms.Curb.LoadShare.PressTorqueCap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbLoadSharePressTorqueCap::RunTest(const FString& Parameters)
{
	// A made-up 5-bar's Jacobian (rows x, z; columns hips A, B, cm/rad)
	// [[-12, 0.5], [-6, -10]], capped at 800 N with 0.8 of it allowed fore/aft:
	// hip A (6 * 800 + 12 * 0.8 * 800) / 100 = 124.8 N*m, hip B
	// (10 * 800 + 0.5 * 0.8 * 800) / 100 = 83.2 N*m; both press down by turning
	// +, against their negative Jz.
	double Cap = 0.0;
	int32  Sign = 0;
	TestTrue(TEXT("hip A caps"), RammsCurb::PressTorqueCap(-12.0, -6.0, 800.0, 0.8, 0.0, 0.5, Cap, Sign));
	TestNearlyEqual(TEXT("hip A's cap"), Cap, 124.8, 1e-9);
	TestEqual(TEXT("hip A presses turning +"), Sign, 1);
	TestTrue(TEXT("hip B caps"), RammsCurb::PressTorqueCap(0.5, -10.0, 800.0, 0.8, 0.0, 0.5, Cap, Sign));
	TestNearlyEqual(TEXT("hip B's cap"), Cap, 83.2, 1e-9);
	TestEqual(TEXT("hip B presses turning +"), Sign, 1);

	// A positive Jz presses turning -; a drive reaction adds its lever.
	TestTrue(TEXT("a crank caps"), RammsCurb::PressTorqueCap(2.0, 10.0, 500.0, 0.8, 3.0, 0.5, Cap, Sign));
	TestNearlyEqual(TEXT("the crank's cap"), Cap, (10.0 * 500.0 + (2.0 + 3.0) * 0.8 * 500.0) / 100.0, 1e-9);
	TestEqual(TEXT("the crank presses turning -"), Sign, -1);

	// A motor with almost no vertical lever does not carry its wheel.
	TestFalse(TEXT("too little vertical lever"), RammsCurb::PressTorqueCap(-12.0, 0.3, 800.0, 0.8, 0.0, 0.5, Cap, Sign));
	TestFalse(TEXT("no force"), RammsCurb::PressTorqueCap(-12.0, -6.0, 0.0, 0.8, 0.0, 0.5, Cap, Sign));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbLoadShareShares,
	"Ramms.Curb.LoadShare.Shares",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbLoadShareShares::RunTest(const FString& Parameters)
{
	const double   W = 1000.0;
	TArray<double> F;

	// Six wheels symmetric about the centre of mass: W/6 each.
	{
		const TArray<FVector2D> P = { { 45, -25 }, { 45, 25 }, { 0, -27 }, { 0, 27 }, { -45, -25 }, { -45, 25 } };
		TestTrue(TEXT("six: solved"), RammsCurb::MinNormLoadShares(P, FVector2D::ZeroVector, W, F));
		for (int32 i = 0; i < P.Num(); ++i)
		{
			TestNearlyEqual(FString::Printf(TEXT("six: wheel %d"), i), F[i], W / 6.0, 1e-6);
		}
	}
	// Four wheels on two axles, the centre of mass 10 cm ahead of the middle:
	// the statics' fore/aft split, equal left and right.
	{
		const TArray<FVector2D> P = { { 45, -25 }, { 45, 25 }, { -45, -25 }, { -45, 25 } };
		TestTrue(TEXT("four: solved"), RammsCurb::MinNormLoadShares(P, FVector2D(10, 0), W, F));
		const double Front = W * (45.0 + 10.0) / 90.0;
		TestNearlyEqual(TEXT("four: front left"), F[0], Front / 2.0, 1e-6);
		TestNearlyEqual(TEXT("four: front right"), F[1], Front / 2.0, 1e-6);
		TestNearlyEqual(TEXT("four: rear left"), F[2], (W - Front) / 2.0, 1e-6);
		TestNearlyEqual(TEXT("four: rear right"), F[3], (W - Front) / 2.0, 1e-6);
	}
	// Three wheels: the statics exactly (the weight and its moments).
	{
		const TArray<FVector2D> P = { { 40, 0 }, { -20, -30 }, { -20, 30 } };
		const FVector2D			Com(5, 4);
		TestTrue(TEXT("three: solved"), RammsCurb::MinNormLoadShares(P, Com, W, F));
		TestNearlyEqual(TEXT("three: the weight"), LoadShareSum(F), W, 1e-6);
		const FVector2D At = LoadShareCentre(P, F);
		TestNearlyEqual(TEXT("three: centre x"), At.X, Com.X, 1e-6);
		TestNearlyEqual(TEXT("three: centre y"), At.Y, Com.Y, 1e-6);
	}
	// The centre of mass near an edge: the far wheel's share would be negative;
	// it carries nothing and the rest carry the weight about the centre of mass.
	{
		const TArray<FVector2D> P = { { 45, -25 }, { 45, 25 }, { -45, -25 }, { -45, 25 }, { -10, 0 } };
		const FVector2D			Com(40, 0);
		TestTrue(TEXT("edge: solved"), RammsCurb::MinNormLoadShares(P, Com, W, F));
		bool bNegative = false;
		for (const double X : F)
		{
			bNegative = bNegative || X < 0.0;
		}
		TestFalse(TEXT("edge: no negative share"), bNegative);
		TestNearlyEqual(TEXT("edge: the weight"), LoadShareSum(F), W, 1e-6);
		const FVector2D At = LoadShareCentre(P, F);
		TestNearlyEqual(TEXT("edge: centre x"), At.X, Com.X, 1e-6);
		TestNearlyEqual(TEXT("edge: centre y"), At.Y, Com.Y, 1e-6);
	}
	// Contacts on one line share along it.
	{
		const TArray<FVector2D> P = { { -30, 0 }, { 0, 0 }, { 30, 0 } };
		TestTrue(TEXT("line: solved"), RammsCurb::MinNormLoadShares(P, FVector2D(10, 0), W, F));
		TestNearlyEqual(TEXT("line: the weight"), LoadShareSum(F), W, 1e-6);
		TestNearlyEqual(TEXT("line: centre"), LoadShareCentre(P, F).X, 10.0, 1e-6);
		TestFalse(TEXT("line: the centre of mass off it"), RammsCurb::MinNormLoadShares(P, FVector2D(0, 5), W, F));
	}
	// The centre of mass outside the contacts: none carry it.
	{
		const TArray<FVector2D> P = { { 45, -25 }, { 45, 25 }, { -45, -25 }, { -45, 25 } };
		TestFalse(TEXT("outside: not solved"), RammsCurb::MinNormLoadShares(P, FVector2D(60, 0), W, F));
		TestFalse(TEXT("one contact: not solved"), RammsCurb::MinNormLoadShares(TArray<FVector2D>{ { 0, 0 } }, FVector2D::ZeroVector, W, F));
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
