// Tests for the curb controller's robot model (Private/Curb/RammsCurbRobotModel).
//
// The convex-hull margin against closed forms and brute force; the shift of
// the zero-moment point under braking and speeding up; the centre of mass the
// legs' moving masses carry; where a wheel touches a slope; which wheels a
// six-wheeled robot may lift at once (same-side corner and centre pairs tip it,
// a front pair needs the centre wheels forward); the margin as measured in the
// world against the one planned in the chassis frame; and the 4-bar table and
// 5-bar grid the model keeps. Every number is made up -- the made-up
// six-wheeled robot of the terrain and level tests (corners 90 cm apart
// fore/aft and 50 across, the middle pair 54 across) on 9 cm wheels, with a
// made-up 100 kg and made-up moving masses -- no robot's own.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Curb/RammsCurbRobotModel.h"
#include "Curb/RammsCurbTerrainFit.h"
#include "Ramms4BarKinematics.h"
#include "Ramms5BarKinematics.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	/** The made-up robot's wheels (chassis x, y), in leg order: front -Y,
	 *  front +Y, centre -Y, centre +Y, rear -Y, rear +Y. */
	const FVector2D	 StabTestWheelXY[6] = { { 45.0, -25.0 }, { 45.0, 25.0 }, { 0.0, -27.0 }, { 0.0, 27.0 }, { -45.0, -25.0 }, { -45.0, 25.0 } };
	constexpr int32	 StabFL = 0, StabFR = 1, StabCL = 2, StabCR = 3, StabRL = 4, StabRR = 5;
	const TCHAR*	 StabTestWheelName[6] = { TEXT("FL"), TEXT("FR"), TEXT("CL"), TEXT("CR"), TEXT("RL"), TEXT("RR") };
	constexpr double StabTestRadiusCm = 9.0;
	constexpr double StabTestMassKg = 100.0;
	constexpr double StabTestCornerMassKg = 6.0;
	constexpr double StabTestCentreMassKg = 7.0;
	/** The centre of mass at the zero pose: 12 cm over the chassis origin. */
	const FVector StabTestCenterOfMass(0.0, 0.0, 12.0);
	/** Its wheel centres at the zero pose sit at the chassis origin's height
	 *  plus this (cm): the robot stands with the origin on the ground. */
	constexpr double StabTestZeroZ = StabTestRadiusCm;
	constexpr double StabTestAccel = 60.0;

	/** The made-up robot as a model, its zero pose every wheel at its (x, y)
	 *  and StabTestZeroZ. */
	RammsCurb::FRobotModel StabTestRobot()
	{
		RammsCurb::FRobotModel M;
		M.CenterOfMass = StabTestCenterOfMass;
		M.MassKg = StabTestMassKg;
		for (int32 i = 0; i < 6; ++i)
		{
			RammsCurb::FModelLeg& L = M.Legs.AddDefaulted_GetRef();
			L.Kind = (i == StabCL || i == StabCR) ? ERammsCurbLegKind::FiveBar : ERammsCurbLegKind::FourBar;
			L.Axle = i < 2 ? ERammsCurbAxle::Front : (i < 4 ? ERammsCurbAxle::Center : ERammsCurbAxle::Rear);
			L.Y = StabTestWheelXY[i].Y;
			L.RadiusCm = StabTestRadiusCm;
			L.MovingMassKg = (i == StabCL || i == StabCR) ? StabTestCentreMassKg : StabTestCornerMassKg;
			L.ZeroPose = FVector2D(StabTestWheelXY[i].X, StabTestZeroZ);
		}
		return M;
	}

	/** Standing level on flat ground with the chassis origin Height over it,
	 *  every wheel down but those in Lifted, raised LiftCm; the centre wheels
	 *  at CentreX (cm fore/aft, -Y side first). */
	TArray<RammsCurb::FPoseLeg> StabStance(double Height, TConstArrayView<int32> Lifted, double LiftCm = 8.0,
		const FVector2D& CentreX = FVector2D::ZeroVector)
	{
		TArray<RammsCurb::FPoseLeg> Pose;
		for (int32 i = 0; i < 6; ++i)
		{
			RammsCurb::FPoseLeg& P = Pose.AddDefaulted_GetRef();
			const double		 X = i == StabCL ? CentreX.X : (i == StabCR ? CentreX.Y : StabTestWheelXY[i].X);
			const bool			 bUp = Lifted.Contains(i);
			P.XZ = FVector2D(X, StabTestRadiusCm - Height + (bUp ? LiftCm : 0.0));
			P.bSupport = !bUp;
		}
		return Pose;
	}

	/** The margin of a point inside a convex polygon, by brute force: its
	 *  least distance to the edges (the polygon given counter-clockwise). */
	double StabInsideDistance(TConstArrayView<FVector2D> Ccw, const FVector2D& P)
	{
		double Least = TNumericLimits<double>::Max();
		for (int32 k = 0; k < Ccw.Num(); ++k)
		{
			const FVector2D A = Ccw[k], B = Ccw[(k + 1) % Ccw.Num()];
			const FVector2D E = B - A;
			const double	T = FMath::Clamp(FVector2D::DotProduct(P - A, E) / E.SizeSquared(), 0.0, 1.0);
			Least = FMath::Min(Least, (P - (A + E * T)).Size());
		}
		return Least;
	}

	/** A parallelogram crank-rocker over +-0.6 rad (as the leg kinematics
	 *  tests'). */
	FRamms4BarLegSpec StabParallelogramLeg()
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

	// The knee-limited test 5-bar of the leg kinematics tests: 12 cm proximal
	// links on pivots 10 cm apart at z = 20, 18 cm distal links, symmetric
	// about x = 0, each knee bending 0.3 .. 2.4 rad from straight.
	constexpr double StabKneePivotX = 5.0;
	constexpr double StabKneePivotZ = 20.0;
	constexpr double StabKneeProximal = 12.0;
	constexpr double StabKneeDistal = 18.0;
	constexpr double StabKneeMostBend = 2.4;
	constexpr double StabKneeLeastBend = 0.3;

	FRamms5BarLinkageSpec StabKneeLimitedFiveBar()
	{
		FRamms5BarLinkageSpec Spec;
		Spec.ProximalMotorA = TEXT("test_hip_a");
		Spec.ProximalMotorB = TEXT("test_hip_b");
		Spec.PivotA = FVector2D(StabKneePivotX, StabKneePivotZ);
		Spec.PivotB = FVector2D(-StabKneePivotX, StabKneePivotZ);
		Spec.ProximalLengthA = Spec.ProximalLengthB = static_cast<float>(StabKneeProximal);
		Spec.DistalLengthA = Spec.DistalLengthB = static_cast<float>(StabKneeDistal);
		Spec.ZeroDirA = 0.0f;
		Spec.AngleSignA = -1.0f;
		Spec.ZeroDirB = UE_PI;
		Spec.AngleSignB = 1.0f;
		Spec.bElbowUpA = true;
		Spec.bElbowUpB = false;
		Spec.bFlipEndpointSide = false;
		Spec.DistalRangeA = FVector2D(-StabKneeMostBend, -StabKneeLeastBend);
		Spec.DistalRangeB = FVector2D(StabKneeLeastBend, StabKneeMostBend);
		return Spec;
	}

	/** Its height at x = 0 with both knees bent Bend rad from straight. */
	double StabKneeHeightAtZeroX(double Bend)
	{
		const double D2 = StabKneeProximal * StabKneeProximal + StabKneeDistal * StabKneeDistal + 2.0 * StabKneeProximal * StabKneeDistal * FMath::Cos(Bend);
		return StabKneePivotZ - FMath::Sqrt(D2 - StabKneePivotX * StabKneePivotX);
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStabilityHullMargin,
	"Ramms.Curb.Stability.HullMargin",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbStabilityHullMargin::RunTest(const FString& Parameters)
{
	using RammsCurb::HullMargin;
	// A 20 x 10 rectangle, its corners in no order, a point inside it and a
	// repeated corner that change nothing.
	const TArray<FVector2D> Box = { { 10.0, 5.0 }, { -10.0, -5.0 }, { 0.0, 1.0 }, { 10.0, -5.0 }, { -10.0, 5.0 }, { 10.0, 5.0 } };
	TestNearlyEqual(TEXT("centre of a 20 x 10 box: 5 cm in"), HullMargin(Box, FVector2D(0.0, 0.0)), 5.0, 1e-9);
	TestNearlyEqual(TEXT("2 cm from its end"), HullMargin(Box, FVector2D(8.0, 0.0)), 2.0, 1e-9);
	TestNearlyEqual(TEXT("2 cm outside its end: -2"), HullMargin(Box, FVector2D(12.0, 1.0)), -2.0, 1e-9);
	TestNearlyEqual(TEXT("on its edge: 0"), HullMargin(Box, FVector2D(-3.0, 5.0)), 0.0, 1e-9);

	// A triangle against brute force over a grid inside it.
	const TArray<FVector2D> Tri = { { -45.0, -25.0 }, { 45.0, 0.0 }, { -45.0, 25.0 } };
	int32					Checked = 0;
	for (double X = -44.0; X <= 44.0; X += 4.0)
	{
		for (double Y = -24.0; Y <= 24.0; Y += 4.0)
		{
			const double M = HullMargin(Tri, FVector2D(X, Y));
			if (M > 0.0)
			{
				++Checked;
				if (!FMath::IsNearlyEqual(M, StabInsideDistance(Tri, FVector2D(X, Y)), 1e-9))
				{
					AddError(FString::Printf(TEXT("(%.0f, %.0f): margin %.6f, distance to the edges %.6f"), X, Y, M,
						StabInsideDistance(Tri, FVector2D(X, Y))));
				}
			}
		}
	}
	TestTrue(TEXT("the grid had points inside the triangle"), Checked > 20);

	// No area: a segment or a point stands nothing up -- 0 at best, minus the
	// distance from it.
	const TArray<FVector2D> Line = { { -10.0, 0.0 }, { 0.0, 0.0 }, { 10.0, 0.0 } };
	TestNearlyEqual(TEXT("on a line of wheels: 0"), HullMargin(Line, FVector2D(3.0, 0.0)), 0.0, 1e-9);
	TestNearlyEqual(TEXT("beside a line of wheels: minus the distance"), HullMargin(Line, FVector2D(3.0, 4.0)), -4.0, 1e-9);
	TestNearlyEqual(TEXT("past its end"), HullMargin(Line, FVector2D(13.0, 4.0)), -5.0, 1e-9);
	const TArray<FVector2D> One = { { 1.0, 1.0 } };
	TestNearlyEqual(TEXT("one wheel"), HullMargin(One, FVector2D(4.0, 5.0)), -5.0, 1e-9);
	TestTrue(TEXT("no wheels at all"), HullMargin(TArray<FVector2D>(), FVector2D::ZeroVector) < -1e9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStabilityZmpShift,
	"Ramms.Curb.Stability.ZmpShift",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbStabilityZmpShift::RunTest(const FString& Parameters)
{
	// h a / g: 25 cm up at 60 cm/s^2 is 1.53 cm.
	TestNearlyEqual(TEXT("h 25 cm at 60 cm/s^2"), RammsCurb::ZmpShiftCm(25.0, 60.0), 25.0 * 60.0 / 980.665, 1e-9);
	TestNearlyEqual(TEXT("... 1.53 cm"), RammsCurb::ZmpShiftCm(25.0, 60.0), 1.53, 0.005);
	TestNearlyEqual(TEXT("standing still: none"), RammsCurb::ZmpShiftCm(25.0, 0.0), 0.0, 1e-12);

	// A box 20 long and 40 wide, the centre of mass 2 cm ahead of its middle,
	// 25 cm up, heading +X: braking moves the zero-moment point forward,
	// towards the front edge 8 cm away; speeding up moves it back, towards the
	// rear edge 12 cm away. The dynamic margin is the less of the two.
	const TArray<FVector2D> Wide = { { 10.0, 20.0 }, { -10.0, -20.0 }, { 10.0, -20.0 }, { -10.0, 20.0 } };
	const double			Shift = RammsCurb::ZmpShiftCm(25.0, 60.0);
	TestNearlyEqual(TEXT("braking binds: 8 - h a / g"),
		RammsCurb::DynamicMargin(Wide, FVector2D(2.0, 0.0), 25.0, FVector2D(1.0, 0.0), 60.0, 60.0), 8.0 - Shift, 1e-9);
	TestNearlyEqual(TEXT("braking harder binds harder"),
		RammsCurb::DynamicMargin(Wide, FVector2D(6.0, 0.0), 25.0, FVector2D(1.0, 0.0), 60.0, 120.0), 4.0 - RammsCurb::ZmpShiftCm(25.0, 120.0),
		1e-9);
	TestNearlyEqual(TEXT("speeding up binds when the centre of mass is near the rear"),
		RammsCurb::DynamicMargin(Wide, FVector2D(-8.0, 0.0), 25.0, FVector2D(1.0, 0.0), 60.0, 60.0), 2.0 - Shift, 1e-9);
	TestNearlyEqual(TEXT("standing still is the static margin"),
		RammsCurb::DynamicMargin(Wide, FVector2D(2.0, 0.0), 25.0, FVector2D(1.0, 0.0), 0.0, 0.0), 8.0, 1e-9);
	// A box 20 long and 10 wide, heading along its width: the sides bind.
	const TArray<FVector2D> Narrow = { { 10.0, 5.0 }, { -10.0, -5.0 }, { 10.0, -5.0 }, { -10.0, 5.0 } };
	TestNearlyEqual(TEXT("heading across: the sides bind"),
		RammsCurb::DynamicMargin(Narrow, FVector2D(0.0, 0.0), 25.0, FVector2D(0.0, 1.0), 60.0, 60.0), 5.0 - Shift, 1e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStabilitySameSideForbidden,
	"Ramms.Curb.Stability.SameSideForbidden",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbStabilitySameSideForbidden::RunTest(const FString& Parameters)
{
	// The made-up robot standing level, chassis origin 15 cm up, the centre
	// wheels under the middle, wheels lifted 8 cm. A corner and the centre
	// wheel on its own side leave the centre of mass on the diagonal between
	// the other corners (or past it); so do both fronts with a centre, or a
	// centre with both rears. Any one wheel, the two centres, the diagonal
	// pairs and the zig-zag triples leave plenty.
	const RammsCurb::FRobotModel Robot = StabTestRobot();
	const double				 Need = 3.0; // MinStabilityMargin
	struct FCase
	{
		TArray<int32> Lifted;
		bool		  bAllowed;
	};
	const TArray<FCase> Cases = {
		{ { StabFL, StabCL }, false },
		{ { StabFR, StabCR }, false },
		{ { StabCL, StabRL }, false },
		{ { StabCR, StabRR }, false },
		{ { StabFL, StabFR, StabCL }, false },
		{ { StabFL, StabFR, StabCR }, false },
		{ { StabCL, StabRL, StabRR }, false },
		{ { StabCR, StabRL, StabRR }, false },
		{ { StabFL, StabCL, StabRL }, false },
		{ { StabFL }, true },
		{ { StabFR }, true },
		{ { StabCL }, true },
		{ { StabCR }, true },
		{ { StabRL }, true },
		{ { StabRR }, true },
		{ { StabCL, StabCR }, true },
		{ { StabFL, StabCR }, true },
		{ { StabFR, StabCL }, true },
		{ { StabCL, StabRR }, true },
		{ { StabCR, StabRL }, true },
		{ { StabFL, StabRR }, true },
		{ { StabFR, StabRL }, true },
		{ { StabFL, StabCR, StabRL }, true },
		{ { StabFR, StabCL, StabRR }, true },
	};
	for (const FCase& C : Cases)
	{
		FString Name;
		for (const int32 W : C.Lifted)
		{
			Name += (Name.IsEmpty() ? TEXT("") : TEXT("+")) + FString(StabTestWheelName[W]);
		}
		const RammsCurb::FPoseMargin M = RammsCurb::PoseMargin(Robot, StabStance(15.0, C.Lifted), StabTestAccel, StabTestAccel);
		if (C.bAllowed)
		{
			TestTrue(*FString::Printf(TEXT("%s up: %.2f cm in, braking or speeding up included (%.0f needed)"), *Name, M.DynamicCm, Need),
				M.DynamicCm >= Need);
		}
		else
		{
			TestTrue(*FString::Printf(TEXT("%s up: forbidden, %.2f cm in standing still"), *Name, M.StaticCm), M.StaticCm <= 1.3);
		}
	}

	// The same-side pair's centre of mass is on the diagonal: within the
	// shift of the lifted wheels' own mass of the line.
	const RammsCurb::FPoseMargin Pair = RammsCurb::PoseMargin(Robot, StabStance(15.0, { StabFL, StabCL }), StabTestAccel, StabTestAccel);
	TestNearlyEqual(TEXT("FL+CL: the centre of mass on the diagonal from FR to RL"), Pair.StaticCm, 0.0, 0.05);
	// The zig-zag triple: the triangle FR, CL, RR, its slanted sides 17.7 cm
	// from the middle.
	const RammsCurb::FPoseMargin Zig = RammsCurb::PoseMargin(Robot, StabStance(15.0, { StabFL, StabCR, StabRL }), 0.0, 0.0);
	TestNearlyEqual(TEXT("FL+CR+RL: 17.7 cm in"), Zig.StaticCm, 27.0 * 45.0 / FMath::Sqrt(45.0 * 45.0 + 52.0 * 52.0), 0.05);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStabilityAxlePairNeedsShift,
	"Ramms.Curb.Stability.AxlePairNeedsShift",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbStabilityAxlePairNeedsShift::RunTest(const FString& Parameters)
{
	// Both front corners up: the robot stands on the centre wheels and the
	// rear corners, the front of its support the line between the centre
	// wheels. With them under the middle the centre of mass is on that line;
	// the margin needs them forward of it, by more than the shift braking
	// puts on the zero-moment point -- and walking them forward carries their
	// own mass, and the centre of mass, with them.
	const RammsCurb::FRobotModel Robot = StabTestRobot();
	const double				 Height = 15.0;
	const double				 Lift = 8.0;
	// The closed form: the centre wheels carry the centre of mass forward by
	// 2 m_c / M of their offset; every wheel stands Height - r below its zero
	// pose but for the lifted pair, Lift higher, so the centre of mass is
	// 12 + (2 m_corner (Lift - Height) + (2 m_corner + 2 m_c) (-Height)) / M
	// over the chassis origin, whose ground -- every contact -- is Height
	// below it. Braking moves the zero-moment point h a / g towards the line
	// between the centre wheels.
	auto Expected = [&](double Xc) {
		const double Cx = 2.0 * StabTestCentreMassKg / StabTestMassKg * Xc;
		const double Cz = StabTestCenterOfMass.Z
			+ (2.0 * StabTestCornerMassKg * (Lift - Height) + (2.0 * StabTestCornerMassKg + 2.0 * StabTestCentreMassKg) * -Height) / StabTestMassKg;
		return Xc - Cx - RammsCurb::ZmpShiftCm(Cz + Height, StabTestAccel);
	};
	double Previous = -TNumericLimits<double>::Max();
	for (const double Xc : { 0.0, 2.0, 4.0, 6.0, 8.0, 12.0, 16.0 })
	{
		const RammsCurb::FPoseMargin M = RammsCurb::PoseMargin(Robot, StabStance(Height, { StabFL, StabFR }, Lift, FVector2D(Xc, Xc)), StabTestAccel, StabTestAccel);
		TestNearlyEqual(*FString::Printf(TEXT("centre wheels %+.0f cm: the closed form"), Xc), M.DynamicCm, Expected(Xc), 1e-6);
		TestTrue(*FString::Printf(TEXT("centre wheels %+.0f cm: further forward is steadier"), Xc), M.DynamicCm > Previous);
		Previous = M.DynamicCm;
	}
	TestTrue(TEXT("under the middle: not enough"), RammsCurb::PoseMargin(Robot, StabStance(Height, { StabFL, StabFR }, Lift), StabTestAccel, StabTestAccel).DynamicCm < 0.0);
	TestTrue(TEXT("8 cm forward: enough"),
		RammsCurb::PoseMargin(Robot, StabStance(Height, { StabFL, StabFR }, Lift, FVector2D(8.0, 8.0)), StabTestAccel, StabTestAccel).DynamicCm >= 3.0);

	// The centre wheels need not stand alike: the front of the support is the
	// line between them, and what counts is where it crosses the centre of
	// mass's line of travel (here the middle) -- their mean, less a little for
	// the line's slant.
	const RammsCurb::FPoseMargin Even = RammsCurb::PoseMargin(Robot, StabStance(Height, { StabFL, StabFR }, Lift, FVector2D(8.0, 8.0)), 0.0, 0.0);
	const RammsCurb::FPoseMargin Uneven = RammsCurb::PoseMargin(Robot, StabStance(Height, { StabFL, StabFR }, Lift, FVector2D(12.0, 4.0)), 0.0, 0.0);
	TestNearlyEqual(TEXT("+12 / +4 stands as +8 / +8, but for the slant"), Uneven.StaticCm, Even.StaticCm * 54.0 / FMath::Sqrt(54.0 * 54.0 + 8.0 * 8.0),
		0.01);

	// The rear pair mirrors it: the centre wheels aft, and it is speeding up,
	// which moves the zero-moment point back, that binds.
	const RammsCurb::FPoseMargin Rear = RammsCurb::PoseMargin(Robot, StabStance(Height, { StabRL, StabRR }, Lift, FVector2D(-8.0, -8.0)), StabTestAccel, StabTestAccel);
	const RammsCurb::FPoseMargin Front = RammsCurb::PoseMargin(Robot, StabStance(Height, { StabFL, StabFR }, Lift, FVector2D(8.0, 8.0)), StabTestAccel, StabTestAccel);
	TestNearlyEqual(TEXT("the rear pair mirrors the front"), Rear.DynamicCm, Front.DynamicCm, 1e-6);
	TestNearlyEqual(TEXT("... and speeding up binds it"), Rear.DynamicCm,
		RammsCurb::DynamicMargin({ FVector2D(-8.0, -27.0), FVector2D(-8.0, 27.0), FVector2D(45.0, -25.0), FVector2D(45.0, 25.0) },
			FVector2D(Rear.CenterOfMass.X, Rear.CenterOfMass.Y), Rear.HeightCm, FVector2D(1.0, 0.0), StabTestAccel, 0.0),
		1e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStabilityCenterOfMass,
	"Ramms.Curb.Stability.CenterOfMass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbStabilityCenterOfMass::RunTest(const FString& Parameters)
{
	RammsCurb::FRobotModel Robot = StabTestRobot();
	TArray<FVector2D>	   XZ;
	for (const RammsCurb::FModelLeg& L : Robot.Legs)
	{
		XZ.Add(L.ZeroPose);
	}
	TestTrue(TEXT("at the zero pose, the centre of mass given"), Robot.CenterOfMassAt(XZ).Equals(StabTestCenterOfMass, 1e-12));
	// One corner up 10 cm and 2 cm forward: its 6 kg of 100 moves the centre
	// of mass 0.6 cm up and 0.12 cm forward; a centre wheel walked 8 cm, 7 kg
	// of it, 0.56 cm.
	XZ[StabFL] += FVector2D(2.0, 10.0);
	XZ[StabCR] += FVector2D(8.0, 0.0);
	const FVector C = Robot.CenterOfMassAt(XZ);
	TestNearlyEqual(TEXT("forward"), C.X, 0.12 + 0.56, 1e-12);
	TestNearlyEqual(TEXT("across: the legs move fore/aft and up only"), C.Y, 0.0, 1e-12);
	TestNearlyEqual(TEXT("up"), C.Z, 12.6, 1e-12);
	// No robot mass: the centre of mass stays put.
	Robot.MassKg = 0.0;
	TestTrue(TEXT("no mass given: fixed"), Robot.CenterOfMassAt(XZ).Equals(StabTestCenterOfMass, 1e-12));

	// The tuning's centre of mass: CenterOfMass, its X from the deprecated
	// CenterOfMassX while CenterOfMass.X is 0.
	FRammsCurbTuning Tuning;
	Tuning.CenterOfMassX = 1.5f;
	Tuning.CenterOfMass = FVector(0.0, -0.2, 11.0);
	TestTrue(TEXT("CenterOfMassX stands in for an unset X"), RammsCurb::TunedCenterOfMass(Tuning).Equals(FVector(1.5, -0.2, 11.0), 1e-6));
	Tuning.CenterOfMass.X = -0.4;
	TestTrue(TEXT("CenterOfMass.X wins once set"), RammsCurb::TunedCenterOfMass(Tuning).Equals(FVector(-0.4, -0.2, 11.0), 1e-6));

	// Built from legs: each 4-bar's zero pose its wheel at crank 0, and the
	// masses checked.
	TArray<FRammsCurbLeg> Legs;
	for (int32 i = 0; i < 6; ++i)
	{
		FRammsCurbLeg& Leg = Legs.AddDefaulted_GetRef();
		Leg.Name = FName(StabTestWheelName[i]);
		Leg.Kind = (i == StabCL || i == StabCR) ? ERammsCurbLegKind::FiveBar : ERammsCurbLegKind::FourBar;
		Leg.FourBar = StabParallelogramLeg();
		Leg.Wheel.Position = StabTestWheelXY[i];
		Leg.WheelRadiusCm = static_cast<float>(StabTestRadiusCm);
		Leg.MovingMassKg = 6.0f;
	}
	Tuning.RobotMassKg = 100.0f;
	TArray<FVector2D> Zeros;
	Zeros.Init(FVector2D(0.5, 7.0), 6);
	RammsCurb::FRobotModel Built;
	FString				   Why;
	TestTrue(TEXT("a sound robot builds"), RammsCurb::BuildRobotModel(Legs, Tuning, Zeros, Built, Why));
	bool			bValid = false;
	const FVector2D Crank0 = URamms4BarKinematics::ComputeWheelCenter(Legs[StabFL].FourBar, 0.0f, bValid);
	TestTrue(TEXT("a 4-bar's zero pose: its wheel at crank 0"), bValid && Built.Legs[StabFL].ZeroPose.Equals(Crank0, 1e-9));
	TestTrue(TEXT("a 5-bar's zero pose: as given"), Built.Legs[StabCL].ZeroPose.Equals(FVector2D(0.5, 7.0), 1e-9));
	TestTrue(TEXT("the tuned centre of mass"), Built.CenterOfMass.Equals(FVector(-0.4, -0.2, 11.0), 1e-6));
	TestNearlyEqual(TEXT("a tread touches across the tyre's width when no contact width is set"), Built.Legs[StabFL].ContactWidthCm,
		static_cast<double>(Legs[StabFL].TyreWidthCm), 1e-9);
	Legs[StabFL].ContactWidthCm = 2.5f;
	TestTrue(TEXT("... and across its contact width when one is"),
		RammsCurb::BuildRobotModel(Legs, Tuning, Zeros, Built, Why) && FMath::IsNearlyEqual(Built.Legs[StabFL].ContactWidthCm, 2.5, 1e-9));
	Legs[StabRR].MovingMassKg = -1.0f;
	TestFalse(TEXT("a negative moving mass refuses"), RammsCurb::BuildRobotModel(Legs, Tuning, Zeros, Built, Why));
	TestTrue(*FString::Printf(TEXT("... naming the leg: %s"), *Why), Why.Contains(TEXT("RR")));
	Legs[StabRR].MovingMassKg = 80.0f;
	TestFalse(TEXT("legs heavier than the robot refuse"), RammsCurb::BuildRobotModel(Legs, Tuning, Zeros, Built, Why));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStabilityContactPoint,
	"Ramms.Curb.Stability.ContactPoint",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

namespace
{
	/** The point of a cylinder (centre C, unit axle A, radius R, tread W wide)
	 *  lowest against the plane of gradient G, by brute force: every point of
	 *  its surface on a fine mesh, the least z - G . (x, y) (proportional to
	 *  its height over the plane along the normal). */
	FVector StabBruteContact(const FVector& C, const FVector& A, const FVector2D& G, double R, double W)
	{
		const FVector U = FVector::CrossProduct(A, FMath::Abs(A.Z) < 0.9 ? FVector::UpVector : FVector::ForwardVector).GetSafeNormal();
		const FVector V = FVector::CrossProduct(A, U);
		FVector		  Best = C;
		double		  Least = TNumericLimits<double>::Max();
		for (int32 k = 0; k <= 40; ++k)
		{
			const FVector Across = A * (W * (k / 40.0 - 0.5));
			for (double T = 0.0; T < 2.0 * UE_DOUBLE_PI; T += 2e-4)
			{
				const FVector P = C + Across + (U * FMath::Cos(T) + V * FMath::Sin(T)) * R;
				const double  Height = P.Z - G.X * P.X - G.Y * P.Y;
				if (Height < Least)
				{
					Least = Height;
					Best = P;
				}
			}
		}
		return Best;
	}
} // namespace

bool FRammsCurbStabilityContactPoint::RunTest(const FString& Parameters)
{
	using RammsCurb::WheelContact;
	const double  R = StabTestRadiusCm;
	const double  W = 6.0;
	const FVector C(10.0, -4.0, 30.0);
	const FVector Level(0.0, 1.0, 0.0); // the axle of a wheel rolling along +X
	auto		  Tan = [](double Deg) { return FMath::Tan(FMath::DegreesToRadians(Deg)); };

	// A slope s along the rolling direction, the axle level: the disc leans
	// R s / sqrt(1 + s^2) uphill of its centre's foot, R / sqrt(1 + s^2) below
	// its centre, and the axle lies along the plane, so the tread touches
	// across its middle.
	for (const double SlopeDeg : { -6.0, 3.0, 6.0, 10.0 })
	{
		const double  S = Tan(SlopeDeg);
		const FVector P = WheelContact(C, Level, FVector2D(S, 0.0), R, W);
		const FVector Brute = StabBruteContact(C, Level, FVector2D(S, 0.0), R, 0.0);
		TestNearlyEqual(*FString::Printf(TEXT("%.0f deg along: the contact, by brute force"), SlopeDeg), P.X, Brute.X, 2e-3);
		TestNearlyEqual(*FString::Printf(TEXT("%.0f deg along: closed form"), SlopeDeg), P.X - C.X, R * S / FMath::Sqrt(1.0 + S * S), 1e-9);
		TestTrue(*FString::Printf(TEXT("%.0f deg along: uphill"), SlopeDeg), (P.X - C.X) * S >= 0.0);
		TestNearlyEqual(*FString::Printf(TEXT("%.0f deg along: under the centre's height"), SlopeDeg), C.Z - P.Z, R / FMath::Sqrt(1.0 + S * S), 1e-9);
		TestNearlyEqual(*FString::Printf(TEXT("%.0f deg along: the tread's middle"), SlopeDeg), P.Y, C.Y, 1e-9);
	}

	// A slope across a level axle: a cylinder touches on its uphill rim, half
	// its tread from the middle, straight under its centre fore/aft -- the
	// wheel-centre height rho, R + (W / 2) |g . a|, over the plane under the centre.
	for (const double SlopeDeg : { -6.0, -2.0, 4.0, 6.0 })
	{
		const double  G = Tan(SlopeDeg);
		const FVector P = WheelContact(C, Level, FVector2D(0.0, G), R, W);
		const FVector Brute = StabBruteContact(C, Level, FVector2D(0.0, G), R, W);
		const FString At = FString::Printf(TEXT("%.0f deg across"), SlopeDeg);
		TestTrue(*(At + TEXT(": the contact, by brute force")), P.Equals(Brute, 2e-3));
		TestNearlyEqual(*(At + TEXT(": on the uphill rim")), P.Y - C.Y, 0.5 * W * FMath::Sign(G), 1e-9);
		TestNearlyEqual(*(At + TEXT(": not fore/aft")), P.X, C.X, 1e-9);
		// The centre over the plane through the contact, straight below it.
		const double Rho = (C.Z - P.Z) + G * (P.Y - C.Y);
		TestNearlyEqual(*(At + TEXT(": rho")), Rho, R + 0.5 * W * FMath::Abs(G), 1e-9);
		TestNearlyEqual(*(At + TEXT(": rho as the legs reckon it")), Rho,
			RammsCurb::WheelCentreHeight(R, W, FVector2D(0.0, G), FVector2D(1.0, 0.0)), 1e-9);
	}

	// Both at once (4 deg along, 4 deg across): brute force.
	{
		const FVector2D G(Tan(4.0), Tan(-4.0));
		TestTrue(TEXT("4 + 4 deg: the contact, by brute force"), WheelContact(C, Level, G, R, W).Equals(StabBruteContact(C, Level, G, R, W), 2e-3));
	}

	// The chassis rolled with a 6 deg cross slope, the axle along the plane:
	// the tread touches across its middle, R along the plane's normal from
	// the centre -- R sin(6 deg) uphill of the centre's foot.
	{
		const double  G = Tan(6.0);
		const FVector Axle = FVector(0.0, 1.0, G).GetSafeNormal();
		const FVector N = FVector(0.0, -G, 1.0).GetSafeNormal();
		const FVector P = WheelContact(C, Axle, FVector2D(0.0, G), R, W);
		TestTrue(TEXT("rolled with the slope: R along the normal"), P.Equals(C - N * R, 1e-9));
		TestNearlyEqual(TEXT("... R sin(6 deg) uphill"), P.Y - C.Y, R * FMath::Sin(FMath::DegreesToRadians(6.0)), 1e-9);
	}

	// On flat ground with the chassis rolled 2 deg (+Y up), the lower rim.
	{
		const double  Roll = FMath::DegreesToRadians(2.0);
		const FVector Axle(0.0, FMath::Cos(Roll), FMath::Sin(Roll));
		const FVector P = WheelContact(C, Axle, FVector2D::ZeroVector, R, W);
		TestTrue(TEXT("rolled 2 deg on flat ground: by brute force"), P.Equals(StabBruteContact(C, Axle, FVector2D::ZeroVector, R, W), 2e-3));
		TestNearlyEqual(TEXT("... on the lower rim"), FVector::DotProduct(P - C, Axle), -0.5 * W, 1e-9);
		TestTrue(TEXT("... either sense of the axle alike"), P.Equals(WheelContact(C, -Axle, FVector2D::ZeroVector, R, W), 1e-9));
	}

	// Within RimContactRampDeg of parallel, part way, in proportion: half a
	// degree across puts it halfway to the rim (sin 0.5 / sin 1 of the way).
	{
		const double  Half = Tan(0.5 * RammsCurb::RimContactRampDeg);
		const FVector P = WheelContact(C, Level, FVector2D(0.0, Half), R, W);
		const double  Expected = 0.5 * W * FMath::Sin(FMath::DegreesToRadians(0.5 * RammsCurb::RimContactRampDeg))
			/ FMath::Sin(FMath::DegreesToRadians(RammsCurb::RimContactRampDeg));
		TestNearlyEqual(TEXT("half the ramp across: halfway to the rim"), P.Y - C.Y, Expected, 1e-9);
		TestTrue(TEXT("flat: the middle, straight under the centre"), WheelContact(C, Level, FVector2D::ZeroVector, R, W).Equals(C - FVector(0.0, 0.0, R), 1e-12));
		TestTrue(TEXT("no tread: the disc"), WheelContact(C, Level, FVector2D(0.0, Tan(6.0)), R, 0.0).Equals(C - FVector(0.0, 0.0, R), 1e-12));
	}

	// The robot standing level, every wheel down, on a 6 deg cross slope
	// rising to +Y: each contact is on its uphill rim, so the downhill side of
	// the support polygon comes in by half the tread -- the centre wheels' 3 cm
	// there, against the robot's middle.
	{
		RammsCurb::FRobotModel Robot = StabTestRobot();
		for (int32 i = 0; i < 6; ++i)
		{
			Robot.Legs[i].ContactWidthCm = (i == StabCL || i == StabCR) ? 6.0 : 4.0;
		}
		TArray<RammsCurb::FPoseLeg>	 Pose = StabStance(15.0, TArray<int32>());
		const RammsCurb::FPoseMargin Flat = RammsCurb::PoseMargin(Robot, Pose, StabTestAccel, StabTestAccel);
		const double				 G = Tan(6.0);
		TArray<FVector2D>			 Expected;
		for (int32 i = 0; i < 6; ++i)
		{
			Pose[i].Gradient = FVector2D(0.0, G);
			Expected.Add(FVector2D(Pose[i].XZ.X, StabTestWheelXY[i].Y + 0.5 * Robot.Legs[i].ContactWidthCm));
		}
		const RammsCurb::FPoseMargin Sloped = RammsCurb::PoseMargin(Robot, Pose, StabTestAccel, StabTestAccel);
		const FVector2D				 Com(Sloped.CenterOfMass.X, Sloped.CenterOfMass.Y);
		TestNearlyEqual(TEXT("6 deg across: the uphill rims' polygon"), Sloped.StaticCm, RammsCurb::HullMargin(Expected, Com), 1e-9);
		TestTrue(*FString::Printf(TEXT("... the downhill side 3 cm in (flat %.3f, sloped %.3f)"), Flat.StaticCm, Sloped.StaticCm),
			FMath::IsNearlyEqual(Flat.StaticCm - Sloped.StaticCm, 3.0, 0.05));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStabilityMeasuredMatchesPlanned,
	"Ramms.Curb.Stability.MeasuredMatchesPlanned",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbStabilityMeasuredMatchesPlanned::RunTest(const FString& Parameters)
{
	// The robot standing level with its front corners up, yawed 30 deg and
	// moved: its margin measured in the world (each wheel's world centre, the
	// chassis frame from its attitude and pivots) is the one planned in the
	// chassis frame.
	const RammsCurb::FRobotModel	  Robot = StabTestRobot();
	const TArray<RammsCurb::FPoseLeg> Pose = StabStance(15.0, { StabFL, StabFR }, 8.0, FVector2D(9.0, 7.0));
	const RammsCurb::FPoseMargin	  Planned = RammsCurb::PoseMargin(Robot, Pose, StabTestAccel, StabTestAccel);

	const double  Yaw = FMath::DegreesToRadians(30.0);
	const FVector Forward(FMath::Cos(Yaw), FMath::Sin(Yaw), 0.0);
	const FVector Origin(120.0, -40.0, 15.0 - 200.0);
	// Pivots 40 cm fore/aft of the origin and 10 cm up, their mean 10 cm up.
	RammsCurb::FChassisFrame Frame = RammsCurb::ChassisFrameFrom(Forward, 0.0, 0.0, Origin + FVector(0.0, 0.0, 10.0), FVector(0.0, 0.0, 10.0));
	TestTrue(TEXT("level: the origin back where it was"), Frame.Origin.Equals(Origin, 1e-9));
	TArray<FVector2D>				  XZ;
	TArray<RammsCurb::FMeasuredWheel> Wheels;
	for (int32 i = 0; i < 6; ++i)
	{
		XZ.Add(Pose[i].XZ);
		RammsCurb::FMeasuredWheel& W = Wheels.AddDefaulted_GetRef();
		W.Centre = Frame.ToWorld(FVector(Pose[i].XZ.X, StabTestWheelXY[i].Y, Pose[i].XZ.Y));
		W.bSupport = Pose[i].bSupport;
	}
	const RammsCurb::FPoseMargin Measured = RammsCurb::MeasuredMargin(Robot, Frame, XZ, Wheels, FVector2D(Forward.X, Forward.Y), StabTestAccel, StabTestAccel);
	TestNearlyEqual(TEXT("level and yawed: the same margin standing"), Measured.StaticCm, Planned.StaticCm, 1e-6);
	TestNearlyEqual(TEXT("... and braking or speeding up"), Measured.DynamicCm, Planned.DynamicCm, 1e-6);
	TestNearlyEqual(TEXT("... the same height"), Measured.HeightCm, Planned.HeightCm, 1e-6);

	// Pitched 3 deg nose up about the origin, the same legs: the centre of
	// mass, CoM.z over the origin, swings back by CoM.z sin(3 deg) -- and so
	// does every wheel, by its own height. Against the front of the support
	// (the centre wheels, below the origin) the margin falls by the
	// difference.
	const double Pitch = FMath::DegreesToRadians(3.0);
	// The pivots 10 cm up the chassis' Z, which leans back as the nose rises.
	const FVector Up(-Forward.X * FMath::Sin(Pitch), -Forward.Y * FMath::Sin(Pitch), FMath::Cos(Pitch));
	Frame = RammsCurb::ChassisFrameFrom(Forward, Pitch, 0.0, Origin + Up * 10.0, FVector(0.0, 0.0, 10.0));
	TestTrue(TEXT("pitched: the origin where it was"), Frame.Origin.Equals(Origin, 1e-9));
	TestTrue(TEXT("pitched: X rises at the nose"), Frame.X.Z > 0.0 && FMath::IsNearlyEqual(Frame.X.Z, FMath::Sin(Pitch), 1e-12));
	for (int32 i = 0; i < 6; ++i)
	{
		Wheels[i].Centre = Frame.ToWorld(FVector(Pose[i].XZ.X, StabTestWheelXY[i].Y, Pose[i].XZ.Y));
	}
	const RammsCurb::FPoseMargin Pitched = RammsCurb::MeasuredMargin(Robot, Frame, XZ, Wheels, FVector2D(Forward.X, Forward.Y), 0.0, 0.0);
	// The front support line is where the centre wheels stand: their x, z.
	const double Front = 0.5 * (9.0 + 7.0);
	const double CentreZ = Pose[StabCL].XZ.Y;
	const double Expected = (Front * FMath::Cos(Pitch) - CentreZ * FMath::Sin(Pitch))
		- (Planned.CenterOfMass.X * FMath::Cos(Pitch) - Planned.CenterOfMass.Z * FMath::Sin(Pitch));
	TestNearlyEqual(TEXT("pitched 3 deg: the centre of mass and the wheels swing back by their heights"),
		Pitched.StaticCm, Expected * 54.0 / FMath::Sqrt(54.0 * 54.0 + 2.0 * 2.0), 1e-3);
	TestTrue(TEXT("... which the level margin does not see"), !FMath::IsNearlyEqual(Pitched.StaticCm, Planned.StaticCm, 0.1));

	// On a 6 deg slope rising to the chassis' -Y, yawed as above, every tread
	// 5 cm wide. Level, the margin measured is the one planned (each tread on
	// its uphill rim). Rolled 6 deg with the slope, every tread touches across
	// its middle, R along the plane's normal from its centre.
	{
		RammsCurb::FRobotModel Wide = StabTestRobot();
		for (RammsCurb::FModelLeg& L : Wide.Legs)
		{
			L.ContactWidthCm = 5.0;
		}
		const double				G = FMath::Tan(FMath::DegreesToRadians(6.0));
		const FVector2D				RightW(-Forward.Y, Forward.X); // the chassis' +Y in the world
		const FVector2D				Gw = RightW * -G;
		TArray<RammsCurb::FPoseLeg> Sloped = Pose;
		for (RammsCurb::FPoseLeg& P : Sloped)
		{
			P.Gradient = FVector2D(0.0, -G);
		}
		const RammsCurb::FPoseMargin PlannedSloped = RammsCurb::PoseMargin(Wide, Sloped, StabTestAccel, StabTestAccel);
		RammsCurb::FChassisFrame	 LevelFrame = RammsCurb::ChassisFrameFrom(Forward, 0.0, 0.0, Origin, FVector::ZeroVector);
		for (int32 i = 0; i < 6; ++i)
		{
			Wheels[i].Centre = LevelFrame.ToWorld(FVector(Pose[i].XZ.X, StabTestWheelXY[i].Y, Pose[i].XZ.Y));
			Wheels[i].Gradient = Gw;
		}
		const RammsCurb::FPoseMargin MeasuredSloped = RammsCurb::MeasuredMargin(Wide, LevelFrame, XZ, Wheels, FVector2D(Forward.X, Forward.Y),
			StabTestAccel, StabTestAccel);
		TestNearlyEqual(TEXT("level on a cross slope: the same margin standing"), MeasuredSloped.StaticCm, PlannedSloped.StaticCm, 1e-6);
		TestNearlyEqual(TEXT("... and braking or speeding up"), MeasuredSloped.DynamicCm, PlannedSloped.DynamicCm, 1e-6);
		TestNearlyEqual(TEXT("... the same height"), MeasuredSloped.HeightCm, PlannedSloped.HeightCm, 1e-6);

		// Rolled with the slope: +Y down.
		const RammsCurb::FChassisFrame Rolled = RammsCurb::ChassisFrameFrom(Forward, 0.0, -FMath::Atan(G), Origin, FVector::ZeroVector);
		const FVector				   N = FVector(-Gw.X, -Gw.Y, 1.0).GetSafeNormal();
		TArray<FVector2D>			   Contacts;
		for (int32 i = 0; i < 6; ++i)
		{
			Wheels[i].Centre = Rolled.ToWorld(FVector(Pose[i].XZ.X, StabTestWheelXY[i].Y, Pose[i].XZ.Y));
			if (Wheels[i].bSupport)
			{
				const FVector C = Wheels[i].Centre - N * StabTestRadiusCm;
				Contacts.Add(FVector2D(C.X, C.Y));
			}
		}
		const FVector				 Com = Rolled.ToWorld(Wide.CenterOfMassAt(XZ));
		const RammsCurb::FPoseMargin RolledMargin = RammsCurb::MeasuredMargin(Wide, Rolled, XZ, Wheels, FVector2D(Forward.X, Forward.Y), 0.0, 0.0);
		TestNearlyEqual(TEXT("rolled with the slope: each tread R along the normal"), RolledMargin.StaticCm,
			RammsCurb::HullMargin(Contacts, FVector2D(Com.X, Com.Y)), 1e-9);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbRobotModelFourBarTable,
	"Ramms.Curb.RobotModel.FourBarTable",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbRobotModelFourBarTable::RunTest(const FString& Parameters)
{
	const FRamms4BarLegSpec	 Spec = StabParallelogramLeg();
	RammsCurb::FFourBarTable Table;
	if (!TestTrue(TEXT("a sound leg tabulates"), Table.Build(Spec)))
	{
		return false;
	}
	const FVector2D Range = URamms4BarKinematics::GetHeightRange(Spec);
	TestNearlyEqual(TEXT("its lowest height, the crank sweep's"), Table.HeightRange().X, Range.X, 1e-3);
	TestNearlyEqual(TEXT("its highest"), Table.HeightRange().Y, Range.Y, 1e-3);
	// Against the crank solver, between the samples too.
	for (double Z = Range.X + 0.01; Z < Range.Y; Z += 0.37)
	{
		bool		bReachable = false;
		float		WheelX = 0.0f;
		const float Crank = URamms4BarKinematics::SolveCrankForHeight(Spec, static_cast<float>(Z), bReachable, WheelX);
		TestNearlyEqual(*FString::Printf(TEXT("z %.2f: fore/aft"), Z), Table.XAt(Z), static_cast<double>(WheelX), 0.01);
		TestNearlyEqual(*FString::Printf(TEXT("z %.2f: crank"), Z), Table.CrankAt(Z), static_cast<double>(Crank), 1e-3);
	}
	// Beyond the range, held at its ends.
	TestNearlyEqual(TEXT("below the range: the lowest wheel's x"), Table.XAt(Range.X - 5.0), Table.XAt(Range.X), 1e-12);
	TestNearlyEqual(TEXT("above it: the highest's"), Table.XAt(Range.Y + 5.0), Table.XAt(Range.Y), 1e-12);
	// A leg that does not close over its range has no table.
	FRamms4BarLegSpec Broken = Spec;
	Broken.CouplerLength = 40.0f;
	TestFalse(TEXT("a leg that does not close has no table"), Table.Build(Broken));
	TestFalse(TEXT("... and is left empty"), Table.IsValid());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbRobotModelFiveBarGrid,
	"Ramms.Curb.RobotModel.FiveBarGrid",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbRobotModelFiveBarGrid::RunTest(const FString& Parameters)
{
	// The knee-limited test 5-bar's reach, gridded from its own test (as the
	// controller grids a leg from its 5-bar controller's SolveTarget).
	const FRamms5BarLinkageSpec Spec = StabKneeLimitedFiveBar();
	auto						Reaches = [&Spec](double X, double Z) {
		   bool bReachable = false;
		   URamms5BarKinematics::SolveIK(Spec, FVector2D(X, Z), bReachable);
		   return bReachable;
	};
	RammsCurb::FFiveBarGrid Grid;
	Grid.Build(Reaches, FVector2D(-20.0, 20.0), FVector2D(-15.0, 20.0));
	if (!TestTrue(TEXT("built"), Grid.IsValid() && Grid.CountReachable() > 0))
	{
		return false;
	}
	// Every cell is the test's answer at its centre.
	int32 Wrong = 0;
	for (double Z = -15.0; Z <= 20.0; Z += 0.25)
	{
		for (double X = -20.0; X <= 20.0; X += 1.0)
		{
			Wrong += Grid.Contains(X, Z) != Reaches(X, Z) ? 1 : 0;
		}
	}
	TestEqual(TEXT("cells that disagree with the reach test"), Wrong, 0);
	TestFalse(TEXT("outside the grid: not reachable"), Grid.Contains(0.0, 30.0));
	// At x = 0 the column spans the knee-limited span, to a cell.
	const FVector2D Span = Grid.HeightRangeAt(0.0);
	const double	Ceiling = StabKneeHeightAtZeroX(StabKneeMostBend);
	const double	Floor = StabKneeHeightAtZeroX(StabKneeLeastBend);
	TestTrue(*FString::Printf(TEXT("its top %.2f within a cell under the knee-limited ceiling %.3f"), Span.Y, Ceiling),
		Span.Y <= Ceiling && Span.Y > Ceiling - RammsCurb::FiveBarGridZStepCm);
	TestTrue(*FString::Printf(TEXT("its bottom %.2f within a cell over the floor %.3f"), Span.X, Floor),
		Span.X >= Floor && Span.X < Floor + RammsCurb::FiveBarGridZStepCm);
	// The row through the middle of the span reaches either way of x = 0,
	// symmetrically.
	const FVector2D Row = Grid.XRangeAt(0.5 * (Ceiling + Floor));
	TestTrue(TEXT("the middle row reaches fore and aft"), Row.X < 0.0 && Row.Y > 0.0);
	TestNearlyEqual(TEXT("... as far each way, to a cell"), -Row.X, Row.Y, RammsCurb::FiveBarGridXStepCm);
	TestTrue(TEXT("a column outside the grid: none"), Grid.HeightRangeAt(40.0).X > Grid.HeightRangeAt(40.0).Y);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
