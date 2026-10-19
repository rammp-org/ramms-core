// Tests for the curb controller's level base (Private/Curb/RammsCurbLevel).
//
// The attitude loop's signs, deadband, freeze and clamps; the support legs'
// share of a correction; what the terrain does to each leg against the plan's
// level ground; the surfaces' bias; the per-side drive roles; the lift a
// tilted chassis gives a swinging wheel; where the level-in starts, the
// attitude loop finishing it, and the cap on one that runs late; a lifted
// wheel's clearance height over a path; the chassis' path through a
// corridor; and a 4-bar crank read a hair past its stop. Every number is
// made up -- the made-up six-wheeled robot of the terrain tests (corners
// 90 cm apart fore/aft and 50 across, the middle pair 54 across), on 9 cm
// wheels -- no robot's own.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Curb/RammsCurbLevel.h"
#include "Curb/RammsCurbSpeedLaw.h"
#include "Curb/RammsCurbTerrainFit.h"
#include "Ramms4BarKinematics.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	/** TestNearlyEqual on doubles, whatever the floats and doubles fed it. */
	void CheckNear(FAutomationTestBase& Test, const FString& What, double Actual, double Expected, double Tolerance)
	{
		Test.TestNearlyEqual(What, Actual, Expected, Tolerance);
	}

	constexpr float LevelKp = 0.3f;
	constexpr float LevelKi = 2.0f;
	constexpr float LevelBand = 0.15f;
	constexpr float LevelMaxDeg = 3.0f;

	/** The made-up robot's wheels (chassis x, y): front, centre, rear, each
	 *  side; their radius and contact width (cm). */
	const FVector2D	 WheelXY[6] = { { 45.0, -25.0 }, { 45.0, 25.0 }, { 0.0, -27.0 }, { 0.0, 27.0 }, { -45.0, -25.0 }, { -45.0, 25.0 } };
	constexpr double TestRadiusCm = 9.0;
	constexpr double TestContactCm = 5.0;

	/** A plane through Z at the origin with the given slope (deg) falling
	 *  away towards -Dir (world xy). */
	FRammsCurbPlane SlopedPlane(double Z, double SlopeDeg, const FVector2D& Dir)
	{
		FRammsCurbPlane P;
		P.Origin = FVector(0.0, 0.0, Z);
		P.Gradient = Dir.GetSafeNormal() * FMath::Tan(FMath::DegreesToRadians(SlopeDeg));
		return P;
	}

	/** Two parallel planes Step apart, an edge square to +X at X = EdgeX. */
	FRammsCurbTerrain TwoPlanes(const FRammsCurbPlane& Near, double Step, double EdgeX)
	{
		FRammsCurbTerrain T;
		T.bValid = true;
		T.EdgeNormal = FVector2D(1.0, 0.0);
		T.EdgeDirection = FVector2D(0.0, 1.0);
		T.EdgePoint = FVector(EdgeX, 0.0, Near.HeightAt(FVector2D(EdgeX, 0.0)));
		T.Near = Near;
		T.Far = Near;
		T.Far.Origin.Z += Step;
		T.StepHeightCm = static_cast<float>(Step);
		T.StepHeightRangeCm = FVector2D(Step, Step);
		return T;
	}

	/** The robot's wheels at chassis origin At (world xy), heading +X. */
	TArray<RammsCurb::FTerrainLeg> Robot(const FVector2D& At, int32 FarWheels = 0)
	{
		TArray<RammsCurb::FTerrainLeg> Out;
		for (int32 i = 0; i < 6; ++i)
		{
			RammsCurb::FTerrainLeg& L = Out.AddDefaulted_GetRef();
			L.XY = At + WheelXY[i];
			L.bFar = i < FarWheels;
			L.RadiusCm = TestRadiusCm;
			L.ContactWidthCm = TestContactCm;
			L.RollDirection = FVector2D(1.0, 0.0);
		}
		return Out;
	}

	/** A parallelogram crank-rocker whose rocker stops sit where the crank's
	 *  range ends: the crank turns the rocker one for one, so a crank read
	 *  past its range puts the rocker past its stop and the leg does not
	 *  close. */
	FRamms4BarLegSpec StoppedLeg()
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
		Spec.RockerRange = FVector2D(-0.6, 0.6);
		return Spec;
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbLevelAttitudeLoop,
	"Ramms.Curb.Level.AttitudeLoop",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbLevelAttitudeLoop::RunTest(const FString& Parameters)
{
	using RammsCurb::FAttitudeLoop;
	{
		// 1.15 deg nose up, 0.1 s: the error past the deadband is 1.0, the
		// integral 2 * 1.0 * 0.1 = 0.2, the proportional term 0.3.
		FAttitudeLoop	Loop;
		const FVector2D C = Loop.Tick(FVector2D(1.15, 0.0), 0.1f, true, LevelKp, LevelKi, LevelBand, LevelMaxDeg);
		CheckNear(*this, TEXT("nose up: a positive pitch correction, Ki e dt + Kp e"), C.X, 0.5, 1e-5);
		CheckNear(*this, TEXT("no roll error, no roll correction"), C.Y, 0.0, 1e-9);
		const FVector2D R = Loop.Tick(FVector2D(0.0, -1.15), 0.1f, true, LevelKp, LevelKi, LevelBand, LevelMaxDeg);
		CheckNear(*this, TEXT("the pitch integral is kept"), R.X, 0.2, 1e-5);
		CheckNear(*this, TEXT("+Y down: a negative roll correction"), R.Y, -0.5, 1e-5);
	}
	{
		// Within the deadband nothing moves, the integral included.
		FAttitudeLoop Loop;
		for (int32 i = 0; i < 100; ++i)
		{
			Loop.Tick(FVector2D(0.14, -0.14), 0.01f, true, LevelKp, LevelKi, LevelBand, LevelMaxDeg);
		}
		TestTrue(TEXT("errors inside the deadband are left alone"), Loop.GetCorrection().IsNearlyZero(1e-9));
		CheckNear(*this, TEXT("the deadband is taken off, not a threshold"), RammsCurb::Deadband(0.5f, 0.15f), 0.35f, 1e-6f);
		CheckNear(*this, TEXT("either way"), RammsCurb::Deadband(-0.5f, 0.15f), -0.35f, 1e-6f);
	}
	{
		// Frozen: the integral holds, the proportional term still acts.
		FAttitudeLoop Loop;
		Loop.Tick(FVector2D(1.15, 0.0), 0.1f, true, LevelKp, LevelKi, LevelBand, LevelMaxDeg);
		const FVector2D C = Loop.Tick(FVector2D(2.15, 0.0), 0.5f, false, LevelKp, LevelKi, LevelBand, LevelMaxDeg);
		CheckNear(*this, TEXT("frozen: the integral does not move"), Loop.GetIntegral().X, 0.2, 1e-5);
		CheckNear(*this, TEXT("frozen: the proportional term still acts"), C.X, 0.2 + 0.3 * 2.0, 1e-5);
	}
	{
		// Wound up: the integral and the whole correction stop at the limit.
		FAttitudeLoop Loop;
		for (int32 i = 0; i < 1000; ++i)
		{
			Loop.Tick(FVector2D(5.0, -5.0), 0.01f, true, LevelKp, LevelKi, LevelBand, LevelMaxDeg);
		}
		CheckNear(*this, TEXT("the integral is clamped to the limit"), Loop.GetIntegral().X, LevelMaxDeg, 1e-5);
		CheckNear(*this, TEXT("so is the correction"), Loop.GetCorrection().X, LevelMaxDeg, 1e-5);
		CheckNear(*this, TEXT("either way"), Loop.GetCorrection().Y, -LevelMaxDeg, 1e-5);
		// And it comes off at once when the error turns: no windup beyond it.
		const FVector2D C = Loop.Tick(FVector2D(-1.15, 0.0), 0.01f, true, LevelKp, LevelKi, LevelBand, LevelMaxDeg);
		TestTrue(TEXT("unwinding starts from the limit"), C.X < LevelMaxDeg - 0.29);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbLevelCorrection,
	"Ramms.Curb.Level.Correction",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbLevelCorrection::RunTest(const FString& Parameters)
{
	TArray<FVector2D> XY;
	for (const FVector2D& P : WheelXY)
	{
		XY.Add(P);
	}
	const double Tan1 = FMath::Tan(FMath::DegreesToRadians(1.0));
	{
		// Nose up corrected by 1 deg: the front legs retract (their wheels go
		// up the chassis), the rear ones extend, about the support's middle.
		const TArray<bool> All = { true, true, true, true, true, true };
		TArray<float>	   Dz;
		Dz.Init(0.0f, 6);
		RammsCurb::AttitudeOffsets(XY, All, FVector2D(1.0, 0.0), Dz);
		CheckNear(*this, TEXT("a front leg rises 45 tan(1 deg)"), Dz[0], 45.0 * Tan1, 1e-4);
		CheckNear(*this, TEXT("a rear leg falls as much"), Dz[5], -45.0 * Tan1, 1e-4);
		CheckNear(*this, TEXT("a centre leg stays"), Dz[2], 0.0, 1e-4);
		float Sum = 0.0f;
		for (const float V : Dz)
		{
			Sum += V;
		}
		CheckNear(*this, TEXT("a pure rotation: the support's mean height stays"), Sum, 0.0f, 1e-4f);
	}
	{
		// The front axle lifted: the rotation is about the centre and rear
		// wheels' middle, and the lifted legs are left alone.
		const TArray<bool> Support = { false, false, true, true, true, true };
		TArray<float>	   Dz;
		Dz.Init(0.0f, 6);
		RammsCurb::AttitudeOffsets(XY, Support, FVector2D(1.0, 0.0), Dz);
		CheckNear(*this, TEXT("a lifted leg is not turned"), Dz[1], 0.0, 1e-6);
		CheckNear(*this, TEXT("centre: 22.5 cm ahead of the support's middle"), Dz[3], 22.5 * Tan1, 1e-4);
		CheckNear(*this, TEXT("rear: 22.5 cm behind it"), Dz[4], -22.5 * Tan1, 1e-4);
	}
	{
		// +Y side up corrected: the +Y legs retract.
		const TArray<bool> All = { true, true, true, true, true, true };
		TArray<float>	   Dz;
		Dz.Init(0.0f, 6);
		RammsCurb::AttitudeOffsets(XY, All, FVector2D(0.0, 1.0), Dz);
		TestTrue(TEXT("+Y up: the +Y legs rise"), Dz[1] > 0.0f && Dz[3] > 0.0f && Dz[5] > 0.0f);
		TestTrue(TEXT("and the -Y legs fall"), Dz[0] < 0.0f && Dz[2] < 0.0f && Dz[4] < 0.0f);
		CheckNear(*this, TEXT("by y tan(roll)"), Dz[3], WheelXY[3].Y * Tan1, 1e-4);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbLevelTerrainOffsets,
	"Ramms.Curb.Level.TerrainOffsets",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbLevelTerrainOffsets::RunTest(const FString& Parameters)
{
	using RammsCurb::TerrainOffsets;
	const double GroundZ = 7.0;
	{
		// Level ground at GroundZ: the level-ground plan exactly.
		const FRammsCurbTerrain T = TwoPlanes(SlopedPlane(GroundZ, 0.0, FVector2D(1.0, 0.0)), 15.0, 200.0);
		TArray<double>			Out;
		Out.Init(1.0, 6);
		const double D = TerrainOffsets(T, Robot(FVector2D(0.0, 0.0), 2), GroundZ, 15.0, FVector2D::ZeroVector, Out);
		CheckNear(*this, TEXT("level ground: no reference shift"), D, 0.0, 1e-9);
		for (int32 i = 0; i < 6; ++i)
		{
			CheckNear(*this, *FString::Printf(TEXT("level ground: leg %d where the plan puts it"), i), Out[i], 0.0, 1e-9);
		}
	}
	// A 6 deg slope falling away 30 deg off the heading: each leg sits by its
	// own wheel's ground against the others', plus rho less its radius.
	const FVector2D			Fall(FMath::Cos(FMath::DegreesToRadians(30.0)), FMath::Sin(FMath::DegreesToRadians(30.0)));
	const FRammsCurbPlane	Near = SlopedPlane(10.0, 6.0, Fall);
	const FRammsCurbTerrain T = TwoPlanes(Near, 15.0, 200.0);
	TArray<double>			A, B, C, Biased;
	A.Init(0.0, 6);
	B.Init(0.0, 6);
	C.Init(0.0, 6);
	Biased.Init(0.0, 6);
	const double DA = TerrainOffsets(T, Robot(FVector2D(0.0, 0.0)), GroundZ, 15.0, FVector2D::ZeroVector, A);
	CheckNear(*this, TEXT("the reference is the mean of the wheels' ground over GroundZ"), DA, Near.HeightAt(FVector2D::ZeroVector) - GroundZ, 1e-9);
	const double Rho = RammsCurb::WheelCentreHeight(TestRadiusCm, TestContactCm, Near.Gradient, FVector2D(1.0, 0.0));
	TestTrue(TEXT("rho on the slope is more than the radius"), Rho > TestRadiusCm + 0.1);
	for (int32 i = 0; i < 6; ++i)
	{
		const double Expected = Near.HeightAt(WheelXY[i]) - Near.HeightAt(FVector2D::ZeroVector) + Rho - TestRadiusCm;
		CheckNear(*this, *FString::Printf(TEXT("leg %d: its ground against the others', plus rho - R"), i), A[i], Expected, 1e-9);
	}
	// The robot moved 37 cm on along the heading and 5 across: on one plane
	// every leg keeps its offset -- the reference moves with the ground.
	TerrainOffsets(T, Robot(FVector2D(37.0, 5.0)), GroundZ, 15.0, FVector2D::ZeroVector, B);
	// And with its front axle already on the far side, which here is the near
	// plane raised by exactly the step its level stands for: the same.
	TerrainOffsets(T, Robot(FVector2D(0.0, 0.0), 2), GroundZ, 15.0, FVector2D::ZeroVector, C);
	for (int32 i = 0; i < 6; ++i)
	{
		CheckNear(*this, *FString::Printf(TEXT("leg %d: unchanged as the robot moves over the plane"), i), B[i], A[i], 1e-9);
		CheckNear(*this, *FString::Printf(TEXT("leg %d: unchanged on a far side a step up"), i), C[i], A[i], 1e-9);
	}
	// A bias on the near surface raises every near wheel's ground: the
	// reference moves with it and the legs keep their places.
	const double DBiased = TerrainOffsets(T, Robot(FVector2D(0.0, 0.0)), GroundZ, 15.0, FVector2D(0.4, 0.0), Biased);
	CheckNear(*this, TEXT("a bias moves the reference"), DBiased, DA + 0.4, 1e-9);
	for (int32 i = 0; i < 6; ++i)
	{
		CheckNear(*this, *FString::Printf(TEXT("leg %d: a bias on every wheel's surface leaves it be"), i), Biased[i], A[i], 1e-9);
	}
	// A far side 3 cm higher than its level: the far wheels sit 3 cm higher
	// against the near ones, the reference taking a third of it (two wheels
	// of six).
	{
		FRammsCurbTerrain High = T;
		High.Far.Origin.Z += 3.0;
		TArray<double> Far2;
		Far2.Init(0.0, 6);
		const double D2 = TerrainOffsets(High, Robot(FVector2D(0.0, 0.0), 2), GroundZ, 15.0, FVector2D::ZeroVector, Far2);
		CheckNear(*this, TEXT("the reference takes the mean"), D2, DA + 1.0, 1e-9);
		CheckNear(*this, TEXT("a far wheel sits 2 cm over the plan against the mean"), Far2[0] - A[0], 2.0, 1e-9);
		CheckNear(*this, TEXT("a near wheel 1 cm under it"), Far2[4] - A[4], -1.0, 1e-9);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbLevelPlaneBias,
	"Ramms.Curb.Level.PlaneBias",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbLevelPlaneBias::RunTest(const FString& Parameters)
{
	using RammsCurb::FPlaneBias;
	{
		// Followed with a 0.5 s time constant: about 63 % of the way after it.
		FPlaneBias			Bias;
		const float			Dt = 0.001f;
		const TArray<float> Residuals = { 1.0f, 1.0f, 1.0f };
		for (int32 i = 0; i < 500; ++i)
		{
			Bias.Tick(Residuals, Dt);
		}
		CheckNear(*this, TEXT("one time constant: 1 - 1/e of the way"), Bias.Get(), 1.0f - FMath::Exp(-1.0f), 0.01f);
	}
	{
		// The lowest: wheels carrying little float over their surface, and do
		// not move it.
		FPlaneBias			Bias;
		const TArray<float> Residuals = { 0.6f, -0.2f, 0.6f, 1.4f };
		for (int32 i = 0; i < 5000; ++i)
		{
			Bias.Tick(Residuals, 0.001f);
		}
		CheckNear(*this, TEXT("the lowest planted wheel says where the surface is"), Bias.Get(), -0.2f, 1e-3f);
	}
	{
		// Clamped, and held on fewer than MinWheelsForBias wheels.
		FPlaneBias			Bias;
		const TArray<float> Far = { 9.0f, 9.0f, 9.0f };
		for (int32 i = 0; i < 5000; ++i)
		{
			Bias.Tick(Far, 0.001f);
		}
		CheckNear(*this, TEXT("never more than MaxPlaneBiasCm"), Bias.Get(), RammsCurb::MaxPlaneBiasCm, 1e-4f);
		// Two wheels alone may both be floating: they do not move it.
		const TArray<float> Two = { -1.0f, -1.0f };
		Bias.Tick(Two, 1.0f);
		CheckNear(*this, TEXT("two wheels alone do not move it"), Bias.Get(), RammsCurb::MaxPlaneBiasCm, 1e-4f);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbLevelDriveRoles,
	"Ramms.Curb.Level.DriveRoles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbLevelDriveRoles::RunTest(const FString& Parameters)
{
	using RammsCurb::FDriveWheel;
	using Role = ERammsCurbWheelDrive;
	auto Roles = [](TConstArrayView<FDriveWheel> Wheels) {
		TArray<bool> Out;
		Out.Init(false, Wheels.Num());
		RammsCurb::DriveRoles(Wheels, Out);
		return Out;
	};
	{
		// Corner fallbacks and centre mains, all planted: the mains drive.
		TArray<FDriveWheel> W = { { Role::Fallback, false, true }, { Role::Fallback, true, true }, { Role::Main, false, true },
			{ Role::Main, true, true }, { Role::Fallback, false, true }, { Role::Fallback, true, true } };
		TArray<bool>		D = Roles(W);
		TestTrue(TEXT("planted mains drive"), D[2] && D[3]);
		TestTrue(TEXT("their fallbacks roll free"), !D[0] && !D[1] && !D[4] && !D[5]);
		// The -Y main off the ground: that side's fallbacks take over, the
		// other side keeps its main.
		W[2].bPlanted = false;
		D = Roles(W);
		TestFalse(TEXT("a lifted main does not drive"), D[2]);
		TestTrue(TEXT("its side's fallbacks drive"), D[0] && D[4]);
		TestTrue(TEXT("the other side's main still drives"), D[3]);
		TestTrue(TEXT("and its fallbacks still roll free"), !D[1] && !D[5]);
	}
	{
		// Assist always; passive never; a side without a main hands its
		// fallbacks the drive.
		const TArray<FDriveWheel> W = { { Role::Assist, false, false }, { Role::Assist, true, true }, { Role::Main, false, true },
			{ Role::Fallback, true, true }, { Role::Passive, false, true }, { Role::Passive, true, false } };
		const TArray<bool>		  D = Roles(W);
		TestTrue(TEXT("an assist wheel drives, lifted (spun to ground speed) or not"), D[0] && D[1]);
		TestTrue(TEXT("a passive one never does"), !D[4] && !D[5]);
		TestTrue(TEXT("a fallback on a side with no main drives"), D[3]);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbLevelSwingTilt,
	"Ramms.Curb.Level.SwingTilt",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbLevelSwingTilt::RunTest(const FString& Parameters)
{
	using RammsCurb::SwingTiltRaise;
	const float Two = FMath::DegreesToRadians(2.0f);
	CheckNear(*this, TEXT("nose up: a rear wheel is raised by |x| sin(pitch)"), SwingTiltRaise(-45.0f, 0.0f, Two, 0.0f), 45.0f * FMath::Sin(Two), 1e-4f);
	TestEqual(TEXT("and a front one is not lowered"), SwingTiltRaise(45.0f, 0.0f, Two, 0.0f), 0.0f);
	CheckNear(*this, TEXT("+Y down: a +Y wheel is raised by y sin(roll)"), SwingTiltRaise(0.0f, 25.0f, 0.0f, -Two), 25.0f * FMath::Sin(Two), 1e-4f);
	TestEqual(TEXT("and a -Y one is not lowered"), SwingTiltRaise(0.0f, -25.0f, 0.0f, -Two), 0.0f);
	CheckNear(*this, TEXT("pitch and roll add"), SwingTiltRaise(-45.0f, 25.0f, Two, -Two), (45.0f + 25.0f) * FMath::Sin(Two), 1e-4f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbLevelLevelIn,
	"Ramms.Curb.Level.LevelIn",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbLevelLevelIn::RunTest(const FString& Parameters)
{
	using RammsCurb::LateLevelSpeed;
	using RammsCurb::LevelInStartCm;
	const float Line = RammsCurb::LevelBeforeEdgeCm;
	const float Lag = RammsCurb::DriveLagSeconds;
	CheckNear(*this, TEXT("square on: the line, the margin, the level-in's distance and the chassis' lag"),
		LevelInStartCm(0.5f, 35.0f, 0.0f), Line + RammsCurb::LevelInMarginCm + 35.0f * (0.5f + RammsCurb::LevelInSpareSeconds + Lag), 1e-4f);
	CheckNear(*this, TEXT("at 10 deg the line is further along the heading"), LevelInStartCm(0.5f, 35.0f, 10.0f) - LevelInStartCm(0.5f, 35.0f, 0.0f),
		Line / FMath::Cos(FMath::DegreesToRadians(10.0f)) - Line, 1e-3f);
	TestEqual(TEXT("past the line: no nearer"), LateLevelSpeed(Line + 0.5f, 0.0f, 0.0f, 0.3f, 60.0f, 0.0f, Lag), 0.0f);
	TestTrue(TEXT("late: slowed"), LateLevelSpeed(Line + 10.0f, 0.0f, 0.0f, 0.5f, 60.0f, 0.0f, Lag) < 20.0f);
	TestTrue(TEXT("still to start, the time after the gate counts"), LateLevelSpeed(100.0f, 0.0f, 60.0f, 0.5f, 60.0f, 0.0f, Lag) < 20.0f);
	TestTrue(TEXT("a chassis running on behind its command has less room"),
		LateLevelSpeed(Line + 20.0f, 0.0f, 0.0f, 0.05f, 60.0f, 30.0f, Lag) < LateLevelSpeed(Line + 20.0f, 0.0f, 0.0f, 0.05f, 60.0f, 0.0f, Lag));

	// The robot driven at 35 cm/s, the command ramped at 60 cm/s^2 under the
	// cap, from 80 cm out, its level-in starting where LevelInStartCm puts it
	// for a level-in planned at 0.6 s. The chassis follows the command as a
	// first-order lag (none, 0.1 s, and DriveLagSeconds), and the cap is fed
	// its speed. Taking 0.6 s it is never slowed. Taking 2.5 s it is slowed so
	// as to reach the line no sooner than it is level, and goes on at full
	// speed after. Never done -- its time left stuck at 0.05 s, always nearly
	// there -- it brakes to rest short of the line: the chassis, not just its
	// command.
	const float Planned = 0.6f;
	for (const float ChassisLag : { 0.0f, 0.1f, Lag })
	{
		for (const float Takes : { 0.6f, 2.5f, -1.0f })
		{
			const bool	  bNever = Takes < 0.0f;
			const float	  StartAt = LevelInStartCm(Planned, 35.0f, 0.0f);
			const float	  Dt = 1.0f / 120.0f;
			float		  Distance = 80.0f;
			float		  Command = 35.0f;
			float		  Chassis = 35.0f;
			float		  Least = Command;
			float		  Started = -1.0f;
			float		  T = 0.0f;
			bool		  bLevel = false;
			float		  Nearest = Distance;
			const FString Case = FString::Printf(TEXT("chassis lag %.2f s, level-in %s"), ChassisLag,
				bNever ? TEXT("never done") : *FString::Printf(TEXT("taking %.1f s"), Takes));
			for (int32 i = 0; i < 2400 && Distance > 0.0f; ++i, T += Dt)
			{
				if (Started < 0.0f && Distance <= StartAt)
				{
					Started = T;
				}
				bLevel = bLevel || (!bNever && Started >= 0.0f && T - Started >= Takes);
				const float Left = Started < 0.0f ? Planned : (bNever ? 0.05f : FMath::Max(0.0f, Takes - (T - Started)));
				const float Gate = Started < 0.0f ? Distance - StartAt : 0.0f;
				const float Cap = bLevel ? TNumericLimits<float>::Max() : LateLevelSpeed(Distance, 0.0f, Gate, Left, 60.0f, Chassis, Lag);
				Command = RammsCurb::RampSpeed(Command, FMath::Min(35.0f, Cap), 60.0f, 60.0f, Dt);
				Chassis = ChassisLag > 0.0f ? RammsCurb::LagStep(Chassis, Command, Dt, ChassisLag) : Command;
				Distance -= Chassis * Dt;
				if (!bLevel)
				{
					Least = FMath::Min(Least, Command);
					Nearest = FMath::Min(Nearest, Distance);
				}
			}
			if (Takes == Planned)
			{
				CheckNear(*this, Case + TEXT(": on time, never slowed"), Least, 35.0f, 1e-3f);
				TestTrue(Case + TEXT(": on time, across"), Distance <= 0.0f);
			}
			else if (!bNever)
			{
				TestTrue(Case + TEXT(": slowed while not level"), Least < 20.0f);
				TestTrue(Case + FString::Printf(TEXT(": no nearer the edge than the line before it is level (%.2f cm)"), Nearest),
					Nearest >= Line);
				TestTrue(Case + TEXT(": on again once level"), bLevel && Distance <= 0.0f);
			}
			else
			{
				TestTrue(Case + TEXT(": stopped"), Chassis < 0.5f && Command < 0.5f);
				TestTrue(Case + FString::Printf(TEXT(": the chassis short of the line (%.2f cm)"), Nearest), Nearest >= Line && Distance >= Line);
			}
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbLevelLevelInTrims,
	"Ramms.Curb.Level.LevelInTrims",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbLevelLevelInTrims::RunTest(const FString& Parameters)
{
	// The level-in, with what the controller does each tick: the attitude
	// loop (FLevelState::TickAttitude), then the level-in's latches
	// (TickLevelIn). The base comes out of the drive stance's pitch over
	// 0.3 s, as the support legs' commands catch up with the terrain; the
	// legs alone then leave it Leaves off level (servo sag, a drive wheel's
	// preload rocking it). The loop's correction turns the base back by what
	// it has applied, the legs taking it up a servo lag later.
	const FRammsCurbTuning Tuning;
	const float			   Dt = 1.0f / 120.0f;
	const float			   CatchUp = 0.3f;
	const FVector2D		   Stance(2.8, 0.0);
	for (const FVector2D& Leaves : { FVector2D(1.6, -1.3), FVector2D(0.5, 0.2), FVector2D(-2.5, 0.0) })
	{
		const FString		   Case = FString::Printf(TEXT("left %+.1f / %+.1f deg"), Leaves.X, Leaves.Y);
		RammsCurb::FLevelState State;
		FVector2D			   Applied = FVector2D::ZeroVector;
		FVector2D			   Measured = Stance;
		float				   LevelledAt = -1.0f;
		FVector2D			   AtLatch = FVector2D::ZeroVector;
		bool				   bActedEarly = false;
		for (float T = 0.0f; T < 4.0f; T += Dt)
		{
			const FVector2D Legs = T < CatchUp ? Stance + (Leaves - Stance) * (T / CatchUp) : Leaves;
			Measured = Legs - Applied;
			const FVector2D Correction = State.TickAttitude(Measured, FVector2D::ZeroVector, Dt, /*bIntegrate=*/true, Tuning);
			bActedEarly = bActedEarly || (T < CatchUp && !Correction.IsNearlyZero(1e-6));
			Applied.X = RammsCurb::LagStep(Applied.X, Correction.X, Dt, 0.1f);
			Applied.Y = RammsCurb::LagStep(Applied.Y, Correction.Y, Dt, 0.1f);
			if (State.TickLevelIn(/*bUnderWay=*/true, /*bCaughtUp=*/T >= CatchUp) && LevelledAt < 0.0f)
			{
				LevelledAt = T;
				AtLatch = State.FilteredAttitudeDeg;
			}
		}
		TestFalse(Case + TEXT(": no correction before the legs are on the terrain"), bActedEarly);
		TestTrue(Case + FString::Printf(TEXT(": levelled in (at %.2f s)"), LevelledAt), LevelledAt >= CatchUp && LevelledAt < CatchUp + 1.5f);
		TestTrue(Case + TEXT(": within 1 deg when it latched"), FMath::Abs(AtLatch.X) <= RammsCurb::LevelInToleranceDeg && FMath::Abs(AtLatch.Y) <= RammsCurb::LevelInToleranceDeg);
		TestTrue(Case + FString::Printf(TEXT(": trimmed to the deadband after (%+.2f / %+.2f deg)"), Measured.X, Measured.Y),
			FMath::Abs(Measured.X) <= Tuning.AttitudeDeadbandDeg + 0.05f && FMath::Abs(Measured.Y) <= Tuning.AttitudeDeadbandDeg + 0.05f);
	}
	{
		// Not under way (the approach): nothing latches, the loop stays off.
		RammsCurb::FLevelState State;
		State.TickAttitude(FVector2D(0.2, 0.0), FVector2D::ZeroVector, Dt, true, Tuning);
		TestFalse(TEXT("before the level-in: not done"), State.TickLevelIn(false, true));
		TestFalse(TEXT("before the level-in: the loop off"), State.bAttitudeActive);
		TestTrue(TEXT("the loop off: no correction"),
			State.TickAttitude(FVector2D(2.0, 0.0), FVector2D::ZeroVector, Dt, true, Tuning).IsNearlyZero(1e-9));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbLevelCorridorPath,
	"Ramms.Curb.Level.CorridorPath",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbLevelCorridorPath::RunTest(const FString& Parameters)
{
	using RammsCurb::CorridorPath;
	const double Rate = 1.0;
	{
		// The corridor's floor steps up 8 cm at sample 10; the chassis moves
		// 1 cm a sample at most, and would rather stay at 0. It must start
		// rising 8 samples early, and no sooner.
		TArray<FVector2D> Bands;
		TArray<double>	  Wish;
		for (int32 k = 0; k < 20; ++k)
		{
			Bands.Add(k < 10 ? FVector2D(0.0, 10.0) : FVector2D(8.0, 10.0));
			Wish.Add(0.0);
		}
		TArray<float> Path;
		int32		  Stuck = INDEX_NONE;
		TestTrue(TEXT("a floor that steps up within reach: a path"), CorridorPath(Bands, Wish, Rate, Path, Stuck));
		TestEqual(TEXT("a sample per band"), Path.Num(), Bands.Num());
		bool bInside = Path.Num() == Bands.Num(), bSlow = true;
		for (int32 k = 0; k < Path.Num(); ++k)
		{
			bInside = bInside && Path[k] >= Bands[k].X - 1e-4 && Path[k] <= Bands[k].Y + 1e-4;
			bSlow = bSlow && (k == 0 || FMath::Abs(Path[k] - Path[k - 1]) <= Rate + 1e-4);
		}
		TestTrue(TEXT("inside every band"), bInside);
		TestTrue(TEXT("never faster than the rate"), bSlow);
		if (Path.Num() == 20)
		{
			CheckNear(*this, TEXT("as low as it may be until it must rise"), Path[2], 0.0, 1e-4);
			CheckNear(*this, TEXT("rising in time: 3 cm five samples out"), Path[5], 3.0, 1e-4);
			CheckNear(*this, TEXT("on the new floor where it begins"), Path[10], 8.0, 1e-4);
		}
	}
	{
		// The same step, too steep: 8 cm in 5 samples at 1 cm a sample. No
		// path; where it ran dry is said.
		TArray<FVector2D> Bands;
		TArray<double>	  Wish;
		for (int32 k = 0; k < 10; ++k)
		{
			Bands.Add(k < 5 ? FVector2D(0.0, 2.0) : FVector2D(8.0, 10.0));
			Wish.Add(1.0);
		}
		TArray<float> Path;
		int32		  Stuck = INDEX_NONE;
		TestFalse(TEXT("a corridor that moves faster than the chassis: no path"), CorridorPath(Bands, Wish, Rate, Path, Stuck));
		TestTrue(TEXT("where it ran dry, at the step"), Stuck >= 4 && Stuck <= 5);
	}
	{
		// NaN wishes hold the height before; a wish out of reach is
		// approached at the rate.
		const TArray<FVector2D> Bands = { { 0.0, 10.0 }, { 0.0, 10.0 }, { 0.0, 10.0 }, { 0.0, 10.0 } };
		const TArray<double>	Wish = { 5.0, NAN, NAN, 2.0 };
		TArray<float>			Path;
		int32					Stuck = INDEX_NONE;
		TestTrue(TEXT("a wide corridor: a path"), CorridorPath(Bands, Wish, Rate, Path, Stuck));
		if (Path.Num() == 4)
		{
			CheckNear(*this, TEXT("no wish: it holds"), Path[2], 5.0, 1e-4);
			CheckNear(*this, TEXT("a wish too far: as far as the rate lets it"), Path[3], 4.0, 1e-4);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbLevelSwingHeight,
	"Ramms.Curb.Level.SwingHeight",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbLevelSwingHeight::RunTest(const FString& Parameters)
{
	// A wheel of radius 10 carried from 60 cm short of a 15 cm edge to 6 cm
	// past it: on level ground, the top plus the radius, as the level-ground
	// plan has it -- climbing or descending.
	TArray<FVector2D> Path;
	for (double X = 40.0; X <= 106.0; X += 2.0)
	{
		Path.Add(FVector2D(X, 0.0));
	}
	const FVector2D			Roll(1.0, 0.0);
	const FRammsCurbTerrain Climb = TwoPlanes(SlopedPlane(0.0, 0.0, Roll), 15.0, 100.0);
	CheckNear(*this, TEXT("a climb: the curb top plus the radius"), RammsCurb::PathClearanceHeight(Climb, Path, Roll, 10.0, 6.0, 3.0),
		25.0, 1e-6);
	FRammsCurbTerrain Descent = TwoPlanes(SlopedPlane(15.0, 0.0, Roll), -15.0, 100.0);
	CheckNear(*this, TEXT("a descent: the top it leaves plus the radius"), RammsCurb::PathClearanceHeight(Descent, Path, Roll, 10.0, 6.0, 2.0),
		25.0, 1e-6);
	// The top rising 6 deg away from the edge: the highest point of the
	// crossing is where it lands, about 6 cm past the edge.
	FRammsCurbTerrain Rising = Climb;
	Rising.Far = SlopedPlane(15.0 - 100.0 * FMath::Tan(FMath::DegreesToRadians(6.0)), 6.0, Roll);
	const double Top = RammsCurb::PathClearanceHeight(Rising, Path, Roll, 10.0, 6.0, 3.0);
	TestTrue(TEXT("a rising top: higher where the wheel lands"), Top > 15.0 + 6.0 * FMath::Tan(FMath::DegreesToRadians(6.0)) + 10.0 - 1e-3 && Top < 15.0 + 6.5 * FMath::Tan(FMath::DegreesToRadians(6.0)) + 10.2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbLevelCrankPastItsStop,
	"Ramms.Curb.Level.CrankPastItsStop",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbLevelCrankPastItsStop::RunTest(const FString& Parameters)
{
	const FRamms4BarLegSpec Spec = StoppedLeg();
	const float				Tol = RammsCurb::CrankReadingToleranceRad;
	FVector2D				Wheel;
	float					Used = 0.0f;
	bool					bClamped = true;
	{
		bool bValid = false;
		URamms4BarKinematics::ComputeWheelCenter(Spec, -0.6f - 0.5f * Tol, bValid);
		TestFalse(TEXT("the fixture: just past the range, the leg does not close"), bValid);
	}
	TestTrue(TEXT("inside the range: measured"), RammsCurb::MeasureFourBarCrank(Spec, 0.1f, Tol, Wheel, Used, bClamped));
	TestFalse(TEXT("inside the range: as read"), bClamped);
	TestEqual(TEXT("inside the range: the reading used"), Used, 0.1f);
	for (const float End : { -0.6f, 0.6f })
	{
		const float Reading = End + FMath::Sign(End) * 0.5f * Tol;
		TestTrue(*FString::Printf(TEXT("a hair past %+.1f: measured"), End), RammsCurb::MeasureFourBarCrank(Spec, Reading, Tol, Wheel, Used, bClamped));
		TestTrue(TEXT("a hair past: taken at the end"), bClamped && Used == End);
		bool			bValid = false;
		const FVector2D AtEnd = URamms4BarKinematics::ComputeWheelCenter(Spec, End, bValid);
		TestTrue(TEXT("a hair past: where the end puts the wheel"), bValid && Wheel.Equals(AtEnd, 1e-6));
		TestFalse(*FString::Printf(TEXT("well past %+.1f: not measured"), End),
			RammsCurb::MeasureFourBarCrank(Spec, End + FMath::Sign(End) * 2.0f * Tol, Tol, Wheel, Used, bClamped));
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
