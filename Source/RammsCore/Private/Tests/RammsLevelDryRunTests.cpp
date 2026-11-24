// Tests for the level assist's kinematic dry run (Private/Level/RammsLevelDryRun):
// the made-up grounds, the rest solve on its own, and then the real law
// (FLaw::Step) run once a frame against the dry run's plant -- over ramps,
// cross slopes, crests and sags, a crown, a grade with a cross fall, an
// oblique fold and a slope past the legs' reach, and through the things that
// go wrong: a knock, a stalled leg, legs that travel more or less than the
// law believes, a wrong support set, a stance change and a take-over in the
// middle of a slope, the switch thrown while parked, a noisy and biased
// sensor, joints that sag, and a base wired backwards; and the same grade
// breaks with the sensor a frame late, and at two and three times the pace.
//
// Two made-up robots, each at 16, 30 and 60 frames a second: the six-wheeled
// robot of the curb's tests (four corner legs and two centre legs, all
// independent), and one with two independent drive legs and two ganged pairs
// that roll does not reach. Every number is made up; no robot's own.
//
// Beside each test is what it must catch. Each run also logs one line of its
// figures (steady residual, peak, floating-wheel time ...), which together are
// the table of what to expect on a map.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Level/RammsLevelControls.h"
#include "Level/RammsLevelDryRun.h"
#include "Level/RammsLevelLaw.h"
#include "Level/RammsLevelTypes.h"
#include "RammsLevelAssistComponent.h"

#if WITH_DEV_AUTOMATION_TESTS

// Named, not anonymous: the other level tests share a unity file and may
// well call their own helpers the same.
namespace RammsLevelDryRunTests
{
	using namespace RammsLevel;
	using namespace RammsLevel::DryRun;

	/** With the sensor a frame late, the most the base may be off level
	 *  through a break of the whole grade (deg). */
	constexpr double LateSensorPeakBoundDeg = 1.1;
	/** At up to three times the usual pace, the most the base may be off
	 *  level through a grade break, as a share of the grade (doing nothing
	 *  would be all of it). */
	constexpr double SpeedPeakShare = 0.6;

	/** The law's ticks the cases are run at (s). */
	const double	   FrameSeconds[3] = { 1.0 / 16.0, 1.0 / 30.0, 1.0 / 60.0 };
	const TCHAR* const FrameNames[3] = { TEXT("16 fps"), TEXT("30 fps"), TEXT("60 fps") };

	/** One of the two made-up robots, with what the cases ask of it. */
	struct FKit
	{
		const TCHAR* Name = TEXT("");
		FRobot		 Robot;
		FTuning		 Tuning;
		/** The grade (deg) and the cross fall (deg) its courses are built at,
		 *  and the speed it is driven at (cm/s). */
		double GradeDeg = 0.0;
		double CrossDeg = 0.0;
		double SpeedCmS = 40.0;
		/** A stance change that tilts it: these legs' baselines move by
		 *  RampCm (positive retracts). Small enough that they still stand. */
		TArray<int32> RampLegs;
		double		  RampCm = 0.0;
		/** A support set that is not the true one (bit i: leg i stands). */
		uint32 WrongMask = 0;
		/** When its first leg meets the ramp of RampUp at SpeedCmS (s): the
		 *  moment to stall that leg so that the stall costs something. */
		double StallAtSeconds = 0.0;
		/** For the base wired backwards: the slope it is parked on (deg) and
		 *  a correction limit (deg) its legs can give in full, so that the
		 *  runaway stops at the limit and not at the legs' reach. */
		double BackwardsGroundDeg = 0.0;
		double BackwardsLimitDeg = 0.0;
	};

	/** The six-wheeled robot: four corner legs and two centre legs. */
	FKit SixWheel()
	{
		FKit Kit;
		Kit.Name = TEXT("six wheels");
		Kit.Robot = MakeSixWheelRobot();
		Kit.GradeDeg = 6.0;
		Kit.CrossDeg = 6.0;
		// The rear pair of legs: the robot then sits back on them, nose up.
		Kit.RampLegs.Add(4);
		Kit.RampLegs.Add(5);
		Kit.RampCm = 1.5;
		// "Only the front legs and one centre leg stand."
		Kit.WrongMask = 0x07;
		// Leg 0 is a front wheel, 45 cm ahead of the origin: on the ramp
		// (at 120 cm) 2.0 s in, at 40 cm/s after the drive's 0.15 s lag.
		Kit.StallAtSeconds = 2.2;
		Kit.BackwardsGroundDeg = 3.0;
		Kit.BackwardsLimitDeg = 8.0;
		return Kit;
	}

	/** Two drive legs and two ganged pairs; roll only on the drive legs, and
	 *  capped. Its front pair is carried 4 cm clear, so its grades are kept to
	 *  where that pair stays clear of a break. */
	FKit DrivePairs()
	{
		FKit Kit;
		Kit.Name = TEXT("drive pairs");
		Kit.Robot = MakeTwoDrivePairsRobot();
		Kit.Tuning.MaxCorrectionDeg = FVector2D(15.0, 6.0);
		Kit.GradeDeg = 3.0;
		Kit.CrossDeg = 5.0;
		Kit.RampLegs.Add(2);
		Kit.RampCm = 0.8;
		// "The front pair stands too."
		Kit.WrongMask = 0x0F;
		// Leg 0 is a drive wheel, under the origin: on the ramp 3.15 s in.
		// (The front pair is carried clear and asks nothing before that.)
		// 3.0 s is a whole frame at each of the three rates, so the leg is
		// caught at the same moment in all of them.
		Kit.StallAtSeconds = 3.0;
		Kit.BackwardsGroundDeg = -3.0;
		Kit.BackwardsLimitDeg = 7.0;
		return Kit;
	}

	TArray<FKit> Kits()
	{
		TArray<FKit> Out;
		Out.Add(SixWheel());
		Out.Add(DrivePairs());
		return Out;
	}

	FOptions MakeOptions(const FKit& Kit, int32 Rate)
	{
		FOptions Options;
		Options.Tuning = Kit.Tuning;
		Options.FrameSeconds = FrameSeconds[Rate];
		Options.SpeedCmS = Kit.SpeedCmS;
		Options.MaxSeconds = 40.0;
		return Options;
	}

	FDryEvent MakeEvent(double AtSeconds, EEvent Kind, double Seconds = 0.0, double Value = 0.0, int32 Leg = INDEX_NONE)
	{
		FDryEvent Event;
		Event.AtSeconds = AtSeconds;
		Event.Kind = Kind;
		Event.Seconds = Seconds;
		Event.Value = Value;
		Event.Leg = Leg;
		return Event;
	}

	/** How much a slope of Deg rises over LengthCm (cm). */
	double Rise(double LengthCm, double Deg)
	{
		return LengthCm * FMath::Tan(FMath::DegreesToRadians(Deg));
	}

	/** Flat, then a grade of Deg from X = 120 for LengthCm, then flat. */
	FGround RampUp(double Deg, double LengthCm)
	{
		const FVector2D Knots[2] = { { 120.0, 0.0 }, { 120.0 + LengthCm, Rise(LengthCm, Deg) } };
		return Profile(MakeArrayView(Knots));
	}

	/** Each wheel's contact point at the stance (chassis frame, cm). */
	TArray<FVector> StanceBottoms(const FRobot& Robot)
	{
		TArray<FVector> Out;
		for (const FRobotLeg& Leg : Robot.Legs)
		{
			if (Leg.PairHalfTrackCm > 0.0)
			{
				Out.Add(FVector(Leg.XY.X, Leg.XY.Y - Leg.PairHalfTrackCm, Leg.StanceZ - Leg.RadiusCm));
				Out.Add(FVector(Leg.XY.X, Leg.XY.Y + Leg.PairHalfTrackCm, Leg.StanceZ - Leg.RadiusCm));
			}
			else
			{
				Out.Add(FVector(Leg.XY.X, Leg.XY.Y, Leg.StanceZ - Leg.RadiusCm));
			}
		}
		return Out;
	}

	/** The most the base was off level between two times (deg), from a trace. */
	double PeakTiltBetween(const FResult& Result, const FTuning& Tuning, double FromSeconds, double ToSeconds)
	{
		double Most = 0.0;
		for (const FSample& Sample : Result.Trace)
		{
			if (Sample.Seconds >= FromSeconds && Sample.Seconds <= ToSeconds)
			{
				Most = FMath::Max(Most, Inclination(Sample.TrueAttitudeDeg - Tuning.LevelReferenceDeg));
			}
		}
		return Most;
	}

	/** One line of a run's figures, for the table. */
	void Report(FAutomationTestBase& Test, const FKit& Kit, const TCHAR* Case, int32 Rate, const FResult& Result)
	{
		Test.AddInfo(FString::Printf(TEXT("%s | %s | %s | steady %.2f deg | peak %.2f deg | lag %.2f s | wanted %.2f deg/s %.2f deg/s2 | true %.2f deg/s | floating %.2f s (longest %.2f) | at reach %.2f s | scale >= %.2f | mean %.1f cm/s | %.1f s"),
			Kit.Name, Case, FrameNames[Rate], Result.SteadyResidualDeg, Result.PeakTiltDeg, Result.LongestOffLevelSeconds, Result.MaxWantedRateDegPerSec, Result.MaxWantedAccelDegPerSec2,
			Result.MaxTrueRateDegPerSec, Result.FloatingSeconds, Result.LongestFloatSeconds, Result.SecondsAtReach, Result.MinSpeedScale, Result.MeanSpeedCmS,
			Result.Seconds));
	}

	FString Label(const FKit& Kit, int32 Rate, const TCHAR* What)
	{
		return FString::Printf(TEXT("%s, %s: %s"), Kit.Name, FrameNames[Rate], What);
	}

	/**
	 * What holds for every run that goes well: the robot always at a stable
	 * rest, no fault, the wanted attitude moved within the tilt policy, and
	 * the drive never stopped.
	 */
	void ExpectSound(FAutomationTestBase& Test, const FKit& Kit, int32 Rate, const FResult& Result)
	{
		Test.TestEqual(Label(Kit, Rate, TEXT("every pose a stable rest")), Result.UnstableFrames, 0);
		Test.TestFalse(Label(Kit, Rate, TEXT("no fault")), Result.bEverFaulted);
		Test.TestTrue(Label(Kit, Rate, TEXT("the wanted attitude within its rate")), Result.MaxWantedRateDegPerSec <= Kit.Tuning.TiltRateDegPerSec + 1.0e-3);
		Test.TestTrue(Label(Kit, Rate, TEXT("the wanted attitude within its acceleration")), Result.MaxWantedAccelDegPerSec2 <= Kit.Tuning.TiltAccelDegPerSec2 + 1.0e-3);
		Test.TestTrue(Label(Kit, Rate, TEXT("the speed scale never under its floor")), Result.MinSpeedScale >= Kit.Tuning.MinSpeedScale - 1.0e-9);
	}

	/**
	 * The three frame rates tell the same story: the largest of the three
	 * values is within 10 percent, or AbsoluteDeg, of the smallest (the plan's
	 * figure; the absolute part is for values so small that a tenth of them
	 * is under what one frame of a moving ground changes).
	 */
	void ExpectRatesAgree(FAutomationTestBase& Test, const FKit& Kit, const TCHAR* What, const double Values[3], double AbsoluteDeg)
	{
		const double Most = FMath::Max3(Values[0], Values[1], Values[2]);
		const double Least = FMath::Min3(Values[0], Values[1], Values[2]);
		Test.TestTrue(FString::Printf(TEXT("%s: %s agrees across frame rates (%.2f, %.2f, %.2f)"), Kit.Name, What, Values[0], Values[1], Values[2]),
			Most - Least <= FMath::Max(0.1 * Most, AbsoluteDeg));
	}
} // namespace RammsLevelDryRunTests

// Catches: a ground with a slope's sign turned, a profile read off the wrong
// knots, a fold line turned the wrong way.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunGrounds,
	"Ramms.Level.DryRun.Grounds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunGrounds::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FGround Slope = Plane(5.0, -3.0);
	TestNearlyEqual(TEXT("Plane: rises along +X by the pitch, falls along +Y by a negative roll"), Slope(100.0, 50.0), Rise(100.0, 5.0) - Rise(50.0, 3.0), 1.0e-9);
	TestNearlyEqual(TEXT("Plane: through the origin"), Slope(0.0, 0.0), 0.0, 1.0e-12);

	const FVector2D Knots[3] = { { 0.0, 0.0 }, { 100.0, 10.0 }, { 200.0, 10.0 } };
	const FGround	Line = Profile(MakeArrayView(Knots), 2.0);
	TestNearlyEqual(TEXT("Profile: level before the first knot"), Line(-50.0, 0.0), 0.0, 1.0e-12);
	TestNearlyEqual(TEXT("Profile: a straight line between knots"), Line(50.0, 0.0), 5.0, 1.0e-9);
	TestNearlyEqual(TEXT("Profile: the next stretch"), Line(150.0, 0.0), 10.0, 1.0e-9);
	TestNearlyEqual(TEXT("Profile: level after the last knot"), Line(300.0, 0.0), 10.0, 1.0e-12);
	TestNearlyEqual(TEXT("Profile: the cross fall rises along +Y"), Line(50.0, 10.0), 5.0 + Rise(10.0, 2.0), 1.0e-9);

	const FGround Ridge = Crown(2.0);
	TestNearlyEqual(TEXT("Crown: falls to +Y"), Ridge(70.0, 30.0), -Rise(30.0, 2.0), 1.0e-9);
	TestNearlyEqual(TEXT("Crown: and to -Y"), Ridge(-20.0, -30.0), -Rise(30.0, 2.0), 1.0e-9);
	TestNearlyEqual(TEXT("Crown: the ridge is the X axis"), Ridge(55.0, 0.0), 0.0, 1.0e-12);

	const FGround Folded = Fold(6.0, 30.0, 100.0);
	const double  Cos = FMath::Cos(FMath::DegreesToRadians(30.0));
	const double  Sin = FMath::Sin(FMath::DegreesToRadians(30.0));
	TestNearlyEqual(TEXT("Fold: flat before the line"), Folded(90.0, 0.0), 0.0, 1.0e-12);
	TestNearlyEqual(TEXT("Fold: zero along the line"), Folded(100.0 - 20.0 * Sin, 20.0 * Cos), 0.0, 1.0e-9);
	TestNearlyEqual(TEXT("Fold: rises straight away from the line"), Folded(100.0 + 50.0 * Cos, 50.0 * Sin), Rise(50.0, 6.0), 1.0e-9);
	TestTrue(TEXT("Fold: a wheel at +Y meets it before one at -Y"), Folded(100.0, 25.0) > 0.0 && Folded(100.0, -25.0) == 0.0);
	return true;
}

// Catches: a rest that does not put the wheels on the ground, a wheel left
// under it, and lifted wheels counted as touching.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunRestFlat,
	"Ramms.Level.DryRun.Rest.Flat",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunRestFlat::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FGround Flat = Plane(0.0, 0.0);
	{
		const FRobot		  Robot = MakeSixWheelRobot();
		const TArray<FVector> Bottoms = StanceBottoms(Robot);
		FRestPose			  Pose;
		TestTrue(TEXT("Six wheels: a rest is found"), SolveRest(Bottoms, Flat, FVector2D(30.0, -70.0), 25.0, Robot.CentreOfMass, nullptr, Pose));
		TestTrue(TEXT("Six wheels: stable"), Pose.bStable);
		TestNearlyEqual(TEXT("Six wheels: the origin 39 cm up"), Pose.Z, 39.0, 1.0e-6);
		TestNearlyEqual(TEXT("Six wheels: level (pitch)"), Pose.AttitudeDeg.X, 0.0, 1.0e-6);
		TestNearlyEqual(TEXT("Six wheels: level (roll)"), Pose.AttitudeDeg.Y, 0.0, 1.0e-6);
		TestEqual(TEXT("Six wheels: all six touch"), Pose.Touching, 6);
	}
	{
		const FRobot		  Robot = MakeTwoDrivePairsRobot();
		const TArray<FVector> Bottoms = StanceBottoms(Robot);
		FRestPose			  Pose;
		TestTrue(TEXT("Drive pairs: a rest is found"), SolveRest(Bottoms, Flat, FVector2D::ZeroVector, 0.0, Robot.CentreOfMass, nullptr, Pose));
		TestNearlyEqual(TEXT("Drive pairs: the origin 30 cm up"), Pose.Z, 30.0, 1.0e-6);
		TestNearlyEqual(TEXT("Drive pairs: level (pitch)"), Pose.AttitudeDeg.X, 0.0, 1.0e-6);
		TestNearlyEqual(TEXT("Drive pairs: level (roll)"), Pose.AttitudeDeg.Y, 0.0, 1.0e-6);
		TestEqual(TEXT("Drive pairs: the drive wheels and the rear pair touch"), Pose.Touching, 4);
		TestEqual(TEXT("Drive pairs: six bottoms in all"), Pose.GapCm.Num(), 6);
		if (Pose.GapCm.Num() == 6)
		{
			TestNearlyEqual(TEXT("Drive pairs: the front pair is carried 4 cm clear"), Pose.GapCm[4], 4.0, 1.0e-6);
			TestFalse(TEXT("... and does not touch"), Pose.bTouching[5]);
		}

		// With the weight out beyond the wheels there is no rest to find.
		FRestPose Tipped;
		TestFalse(TEXT("A centre of mass beyond the wheels: no rest"), SolveRest(Bottoms, Flat, FVector2D::ZeroVector, 0.0, FVector(80.0, 0.0, 5.0), nullptr, Tipped));
		TestFalse(TEXT("... and it says so"), Tipped.bStable);
	}
	return true;
}

// Catches: the pose built with a roll of the wrong hand (FRotator's), pitch
// and roll swapped when the robot is turned, and a slope's effect on where a
// tilted wheel meets it left out.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunRestTilted,
	"Ramms.Level.DryRun.Rest.TiltedPlaneGivesItsAttitude",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunRestTilted::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FRobot		  Robot = MakeSixWheelRobot();
	const TArray<FVector> Bottoms = StanceBottoms(Robot);
	const FGround		  Slope = Plane(5.0, -3.0);
	const double		  Tan5 = FMath::Tan(FMath::DegreesToRadians(5.0));
	const double		  Tan3 = FMath::Tan(FMath::DegreesToRadians(3.0));

	// Heading +X: the pitch is the plane's along X. The roll is the plane's
	// along Y seen from a chassis that is already pitched: a hair less.
	FRestPose Ahead;
	TestTrue(TEXT("Heading +X: a rest"), SolveRest(Bottoms, Slope, FVector2D(200.0, 40.0), 0.0, Robot.CentreOfMass, nullptr, Ahead));
	TestNearlyEqual(TEXT("Heading +X: nose up by the plane's pitch"), Ahead.AttitudeDeg.X, 5.0, 1.0e-4);
	TestNearlyEqual(TEXT("Heading +X: +Y side down by the plane's roll"), Ahead.AttitudeDeg.Y,
		-FMath::RadiansToDegrees(FMath::Atan(Tan3 * FMath::Cos(FMath::DegreesToRadians(5.0)))), 1.0e-4);
	TestEqual(TEXT("Heading +X: all six touch"), Ahead.Touching, 6);

	// Turned to head +Y: the plane falls ahead, and its rise along +X is now
	// on the robot's -Y side.
	FRestPose Across;
	TestTrue(TEXT("Heading +Y: a rest"), SolveRest(Bottoms, Slope, FVector2D(200.0, 40.0), 90.0, Robot.CentreOfMass, nullptr, Across));
	TestNearlyEqual(TEXT("Heading +Y: nose down by the plane's fall along Y"), Across.AttitudeDeg.X, -3.0, 1.0e-4);
	TestNearlyEqual(TEXT("Heading +Y: +Y side down by the plane's rise along X"), Across.AttitudeDeg.Y,
		-FMath::RadiansToDegrees(FMath::Atan(Tan5 * FMath::Cos(FMath::DegreesToRadians(3.0)))), 1.0e-4);
	TestEqual(TEXT("Heading +Y: all six touch"), Across.Touching, 6);

	// A warm start from somewhere else finds the same rest.
	FRestPose Warm = Across;
	Warm.AttitudeDeg = FVector2D(1.0, 2.0);
	TestTrue(TEXT("Warm start: a rest"), SolveRest(Bottoms, Slope, FVector2D(200.0, 40.0), 0.0, Robot.CentreOfMass, &Warm, Warm));
	TestNearlyEqual(TEXT("Warm start: the same pitch"), Warm.AttitudeDeg.X, Ahead.AttitudeDeg.X, 1.0e-5);
	TestNearlyEqual(TEXT("Warm start: the same roll"), Warm.AttitudeDeg.Y, Ahead.AttitudeDeg.Y, 1.0e-5);
	TestNearlyEqual(TEXT("Warm start: the same height"), Warm.Z, Ahead.Z, 1.0e-5);
	return true;
}

// Catches: the attitude taken as a fit through all the wheels (a short leg
// would then tilt the robot it does not even touch the ground with).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunRestShortLeg,
	"Ramms.Level.DryRun.Rest.ShortLegFloats",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunRestShortLeg::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FRobot	Robot = MakeSixWheelRobot();
	TArray<FVector> Bottoms = StanceBottoms(Robot);
	// The front -Y wheel drawn up 2 cm: the other five still hold the robot.
	Bottoms[0].Z += 2.0;
	FRestPose Pose;
	TestTrue(TEXT("A rest"), SolveRest(Bottoms, Plane(0.0, 0.0), FVector2D::ZeroVector, 0.0, Robot.CentreOfMass, nullptr, Pose));
	TestNearlyEqual(TEXT("Still level (pitch)"), Pose.AttitudeDeg.X, 0.0, 1.0e-6);
	TestNearlyEqual(TEXT("Still level (roll)"), Pose.AttitudeDeg.Y, 0.0, 1.0e-6);
	TestNearlyEqual(TEXT("Still 39 cm up"), Pose.Z, 39.0, 1.0e-6);
	TestEqual(TEXT("Five touch"), Pose.Touching, 5);
	TestFalse(TEXT("The short leg does not"), Pose.bTouching[0]);
	TestNearlyEqual(TEXT("It floats by the 2 cm"), Pose.GapCm[0], 2.0, 1.0e-6);
	return true;
}

// Catches: a rest that needs every wheel down, and the wrong one of two
// ganged wheels left bearing (the rest then would not be the lowest).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunRestThreePoint,
	"Ramms.Level.DryRun.Rest.ThreePointStance",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunRestThreePoint::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	// The drive legs set 4 cm apart in height, as a roll correction sets
	// them: -Y drawn up 2, +Y let down 2. The rear pair is one leg and
	// cannot follow, so one of its two wheels comes off the ground.
	const FRobot	Robot = MakeTwoDrivePairsRobot();
	TArray<FVector> Bottoms = StanceBottoms(Robot);
	Bottoms[0].Z += 2.0;
	Bottoms[1].Z -= 2.0;
	FRestPose Pose;
	TestTrue(TEXT("A rest on three wheels"), SolveRest(Bottoms, Plane(0.0, 0.0), FVector2D::ZeroVector, 0.0, Robot.CentreOfMass, nullptr, Pose));
	TestEqual(TEXT("Three touch"), Pose.Touching, 3);
	TestTrue(TEXT("Both drive wheels"), Pose.bTouching[0] && Pose.bTouching[1]);
	TestTrue(TEXT("The rear wheel on the low side"), Pose.bTouching[2]);
	TestFalse(TEXT("... and not the one on the high side"), Pose.bTouching[3]);

	// The drive wheels' bottoms are 56 cm apart and 4 cm apart in height.
	const double RollDeg = FMath::RadiansToDegrees(FMath::Atan(4.0 / 56.0));
	TestNearlyEqual(TEXT("The +Y side is up by the drive legs' difference"), Pose.AttitudeDeg.Y, RollDeg, 1.0e-3);
	// The rear wheel that bears is 20 cm out on the low side, so it holds the
	// rear up: the nose comes down. (Worked by hand from the three contacts.)
	const double SinR = FMath::Sin(FMath::DegreesToRadians(RollDeg));
	const double CosR = FMath::Cos(FMath::DegreesToRadians(RollDeg));
	const double PitchDeg = FMath::RadiansToDegrees(FMath::Atan(-((-28.0 * SinR - 28.0 * CosR) - (-20.0 * SinR - 30.0 * CosR)) / 40.0));
	TestNearlyEqual(TEXT("The nose comes down as the rear rides on one wheel"), Pose.AttitudeDeg.X, PitchDeg, 1.0e-3);
	TestTrue(TEXT("... by about two degrees"), PitchDeg < -1.9 && PitchDeg > -2.2);
	TestTrue(TEXT("The other rear wheel floats by its 40 cm of track at that roll"), Pose.GapCm[3] > 2.6 && Pose.GapCm[3] < 3.1);
	return true;
}

// Catches: a law that moves legs with nothing to level (hunting on flat
// ground), and a dry run that does not give the same answer twice.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunFlat,
	"Ramms.Level.DryRun.FlatStaysStillAndRepeats",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunFlat::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FGround Flat = Plane(0.0, 0.0);
	for (const FKit& Kit : Kits())
	{
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.DistanceCm = 300.0;
			const FResult Result = DryRun::Run(Kit.Robot, Flat, Options);
			Report(*this, Kit, TEXT("flat"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 300.0);
			TestTrue(Label(Kit, Rate, TEXT("level throughout")), Result.PeakTiltDeg <= 0.05);
			TestTrue(Label(Kit, Rate, TEXT("no leg moved")), Result.MaxOffsetCm <= 0.05);
			TestTrue(Label(Kit, Rate, TEXT("at full speed")), Result.MinSpeedScale >= 1.0 - 1.0e-9);
			TestEqual(Label(Kit, Rate, TEXT("levelling at the end")), static_cast<int32>(Result.FinalState), static_cast<int32>(EState::Active));

			if (Rate == 0)
			{
				// The same again, with noise so there is something to differ in.
				Options.NoiseDeg = 0.2;
				const FResult First = DryRun::Run(Kit.Robot, RampUp(Kit.GradeDeg, 200.0), Options);
				const FResult Second = DryRun::Run(Kit.Robot, RampUp(Kit.GradeDeg, 200.0), Options);
				TestTrue(Label(Kit, Rate, TEXT("the same run twice: the same time")), First.Seconds == Second.Seconds);
				TestTrue(Label(Kit, Rate, TEXT("... the same peak")), First.PeakTiltDeg == Second.PeakTiltDeg);
				TestTrue(Label(Kit, Rate, TEXT("... the same end")), First.FinalTiltDeg == Second.FinalTiltDeg && First.MaxOffsetCm == Second.MaxOffsetCm);
			}
		}
	}
	return true;
}

// Catches: a feed-forward of the wrong sign or left out (the base then
// follows the ramp), a terrain estimate too slow for a grade break, and a
// correction that is not taken out again at the top and the bottom.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunRampUpAndDown,
	"Ramms.Level.DryRun.RampUpAndDown",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunRampUpAndDown::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		// Flat, up for 2 m, a 2 m landing, down for 2 m, flat.
		const double	Top = Rise(200.0, Kit.GradeDeg);
		const FVector2D Knots[4] = { { 120.0, 0.0 }, { 320.0, Top }, { 520.0, Top }, { 720.0, 0.0 } };
		const FGround	Course = Profile(MakeArrayView(Knots));
		double			Peaks[3] = { 0.0, 0.0, 0.0 };
		double			Lags[3] = { 0.0, 0.0, 0.0 };
		double			Steadies[3] = { 0.0, 0.0, 0.0 };
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.DistanceCm = 900.0;
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("ramp up and down"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			Peaks[Rate] = Result.PeakTiltDeg;
			Lags[Rate] = Result.LongestOffLevelSeconds;
			Steadies[Rate] = Result.SteadyResidualDeg;
			TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 900.0);
			// Through four breaks of the whole grade. With the feed-forward
			// the base stays within a degree; without it (the integral alone)
			// it goes well over one, which is what this bound is for.
			TestTrue(Label(Kit, Rate, TEXT("the peak through the breaks")), Result.PeakTiltDeg <= 1.0);
			TestTrue(Label(Kit, Rate, TEXT("the legs did the levelling")), Result.MaxCorrectionDeg >= 0.8 * Kit.GradeDeg);
			TestTrue(Label(Kit, Rate, TEXT("level again on the flat")), Result.SteadyResidualDeg <= 0.5);
			TestTrue(Label(Kit, Rate, TEXT("never at the legs' reach")), Result.SecondsAtReach == 0.0);
			TestTrue(Label(Kit, Rate, TEXT("most of the speed kept")), Result.MeanSpeedCmS >= 0.6 * Kit.SpeedCmS);
		}
		ExpectRatesAgree(*this, Kit, TEXT("the peak"), Peaks, 0.05);
		// The lag is counted in whole frames: a slow frame of slack.
		ExpectRatesAgree(*this, Kit, TEXT("the lag"), Lags, FrameSeconds[0]);
		ExpectRatesAgree(*this, Kit, TEXT("the steady residual"), Steadies, 0.02);
	}
	return true;
}

// Catches: a loop that rings or runs away when the ground turns over quickly
// (a crest of twice the grade straight into a sag of twice the grade).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunCrestAndSag,
	"Ramms.Level.DryRun.CrestAndSag",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunCrestAndSag::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		// Half the grade up, straight over into half the grade down, straight
		// into half the grade up again, then a landing: breaks of the whole
		// grade at the crest and in the sag.
		const double	HalfDeg = 0.5 * Kit.GradeDeg;
		const double	Top = Rise(180.0, HalfDeg);
		const FVector2D Knots[4] = { { 120.0, 0.0 }, { 300.0, Top }, { 480.0, 0.0 }, { 660.0, Top } };
		const FGround	Course = Profile(MakeArrayView(Knots));
		double			Peaks[3] = { 0.0, 0.0, 0.0 };
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.DistanceCm = 860.0;
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("crest and sag"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			Peaks[Rate] = Result.PeakTiltDeg;
			TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 860.0);
			// The slopes are half the grade, so a base that did nothing
			// would never be further off than that (3 and 1.5 deg), and
			// one levelled by the integral alone is 1.3 deg off through a
			// break of the whole grade. With the feed-forward it is half
			// a degree or so: the bound has to be under all of those.
			TestTrue(Label(Kit, Rate, TEXT("the peak through the breaks")), Result.PeakTiltDeg <= 0.9);
			TestTrue(Label(Kit, Rate, TEXT("the legs did the levelling")), Result.MaxCorrectionDeg >= 0.8 * HalfDeg);
			TestTrue(Label(Kit, Rate, TEXT("level again on the landing")), Result.SteadyResidualDeg <= 0.5);
			TestTrue(Label(Kit, Rate, TEXT("no ringing at the end")), Result.SteadyTiltRangeDeg <= 0.3);
		}
		ExpectRatesAgree(*this, Kit, TEXT("the peak"), Peaks, 0.05);
	}
	return true;
}

// Catches: roll and pitch swapped or roll's sign turned, roll given to a
// ganged pair, and (the robot starts on the slope with the switch on) the
// base snapped level rather than walked there.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunCrossSlope,
	"Ramms.Level.DryRun.CrossSlope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunCrossSlope::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		const FGround Course = Plane(0.0, Kit.CrossDeg);
		double		  Steadies[3] = { 0.0, 0.0, 0.0 };
		double		  Lags[3] = { 0.0, 0.0, 0.0 };
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.MaxSeconds = 9.0;
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("cross slope"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			Steadies[Rate] = Result.SteadyResidualDeg;
			Lags[Rate] = Result.LongestOffLevelSeconds;
			TestTrue(Label(Kit, Rate, TEXT("it starts on the slope")), Result.PeakTiltDeg >= Kit.CrossDeg - 0.1);
			TestTrue(Label(Kit, Rate, TEXT("... and is never tilted further")), Result.PeakTiltDeg <= Kit.CrossDeg + 0.3);
			TestTrue(Label(Kit, Rate, TEXT("level at the end")), Result.SteadyResidualDeg <= 0.5);
			TestTrue(Label(Kit, Rate, TEXT("walked there, not snapped")), Result.MaxTrueRateDegPerSec <= 3.5);
			TestTrue(Label(Kit, Rate, TEXT("the roll came out of the legs")), Result.MaxCorrectionDeg >= 0.8 * Kit.CrossDeg);
		}
		// Where it comes to rest inside the deadband is a matter of the
		// frame it stopped on: the rates agree to the deadband, no closer.
		ExpectRatesAgree(*this, Kit, TEXT("the steady residual"), Steadies, Kit.Tuning.DeadbandDeg);
		ExpectRatesAgree(*this, Kit, TEXT("the walk to level"), Lags, FrameSeconds[0]);
	}
	return true;
}

// Catches: a law that only works with every wheel on one plane (the crown
// puts the two sides on different ones, and crossing it at an angle takes
// the wheels over one at a time).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunCrown,
	"Ramms.Level.DryRun.Crown",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunCrown::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const double  CrownDeg = 2.3;
	const FGround Course = Crown(CrownDeg);
	for (const FKit& Kit : Kits())
	{
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			// Straddling the ridge: next to nothing to level. (Wheels at
			// different tracks stand at different heights on the two falls,
			// which pitches the robot with the ganged pairs by half a degree.)
			FOptions Straddle = MakeOptions(Kit, Rate);
			Straddle.DistanceCm = 200.0;
			const FResult Astride = DryRun::Run(Kit.Robot, Course, Straddle);
			TestEqual(Label(Kit, Rate, TEXT("astride: every pose a stable rest")), Astride.UnstableFrames, 0);
			TestTrue(Label(Kit, Rate, TEXT("astride: all but level throughout")), Astride.PeakTiltDeg <= 0.6);
			TestTrue(Label(Kit, Rate, TEXT("astride: level at the end")), Astride.SteadyResidualDeg <= 0.3);
			TestTrue(Label(Kit, Rate, TEXT("astride: the legs hardly moved")), Astride.MaxOffsetCm <= 0.6);

			// From one side over to the other, 20 deg off the ridge.
			FOptions Options = MakeOptions(Kit, Rate);
			Options.StartXY = FVector2D(0.0, -100.0);
			Options.YawDeg = 20.0;
			Options.DistanceCm = 560.0;
			Options.bKeepTrace = true;
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("crown, crossed at 20 deg"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 560.0);
			// Once walked level on the first side, the crossing itself.
			TestTrue(Label(Kit, Rate, TEXT("the crossing")), PeakTiltBetween(Result, Kit.Tuning, 3.5, 100.0) <= 1.5);
			TestTrue(Label(Kit, Rate, TEXT("level on the far side")), Result.SteadyResidualDeg <= 0.5);
		}
	}
	return true;
}

// Catches: pitch and roll corrections that disturb each other (a grade that
// carries a cross fall needs both at once, and on the robot with ganged
// pairs the roll correction itself changes the pitch).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunCombined,
	"Ramms.Level.DryRun.GradeWithCrossFall",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunCombined::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		// The cross fall is there from the start; the grade comes on at 120.
		const double	CrossDeg = 0.5 * Kit.CrossDeg;
		const FVector2D Knots[2] = { { 120.0, 0.0 }, { 620.0, Rise(500.0, Kit.GradeDeg) } };
		const FGround	Course = Profile(MakeArrayView(Knots), CrossDeg);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.DistanceCm = 480.0;
			Options.bKeepTrace = true;
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("grade with cross fall"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 480.0);
			TestTrue(Label(Kit, Rate, TEXT("never tilted further than it started")), Result.PeakTiltDeg <= CrossDeg + 0.3);
			// It is walked level on the cross fall by 2.6 s; the grade
			// comes under the wheels from 2.0 s (six wheels) or 3.15 s on.
			// From there the peak is the grade break's, under what the
			// integral alone would leave (1.3 deg).
			const double BreakPeakDeg = PeakTiltBetween(Result, Kit.Tuning, 2.6, 100.0);
			AddInfo(FString::Printf(TEXT("%s | grade with cross fall | %s | through the grade break, once level on the cross fall: at most %.2f deg off level"), Kit.Name, FrameNames[Rate], BreakPeakDeg));
			TestTrue(Label(Kit, Rate, TEXT("the peak through the grade break")), BreakPeakDeg <= 0.7);
			TestTrue(Label(Kit, Rate, TEXT("level on the grade, both ways")), Result.SteadyResidualDeg <= 0.5);
			TestTrue(Label(Kit, Rate, TEXT("both corrections in use")), Result.MaxCorrectionDeg >= 0.8 * FMath::Sqrt(FMath::Square(Kit.GradeDeg) + FMath::Square(CrossDeg)));
		}
	}
	return true;
}

// Catches: a plane of offsets that is not rigid, or a terrain estimate that
// depends on which wheels bear (the fold line is crossed one wheel at a
// time, so the robot is twisted over it on three wheels at every moment).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunObliqueFold,
	"Ramms.Level.DryRun.ObliqueFold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunObliqueFold::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		const FGround Course = Fold(Kit.GradeDeg, 30.0, 150.0);
		double		  Peaks[3] = { 0.0, 0.0, 0.0 };
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.DistanceCm = 520.0;
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("oblique fold"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			Peaks[Rate] = Result.PeakTiltDeg;
			TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 520.0);
			// Half a degree with the feed-forward; over one without.
			TestTrue(Label(Kit, Rate, TEXT("the peak over the fold")), Result.PeakTiltDeg <= 0.8);
			TestTrue(Label(Kit, Rate, TEXT("level on the plane beyond")), Result.SteadyResidualDeg <= 0.5);
			TestTrue(Label(Kit, Rate, TEXT("the legs did the levelling")), Result.MaxCorrectionDeg >= 0.8 * Kit.GradeDeg);
		}
		ExpectRatesAgree(*this, Kit, TEXT("the peak"), Peaks, 0.05);
	}
	return true;
}

// Catches: a wanted attitude that stays level on a slope the legs cannot
// take out, a tilt past reach that is not bounded or does not come back to
// level, a slope past reach taken for a fault, and the drive stopped.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunPastReach,
	"Ramms.Level.DryRun.SlopePastReach",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunPastReach::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	// The six-wheeled robot with its legs cut to 6 cm either way, on 8 deg.
	// Planned with (0.5 margin and 2 soft zone off) that is 3.5 cm at the
	// corners' 45 cm: 4.45 deg of the 8, the base keeping the other 3.55.
	{
		FKit Kit = SixWheel();
		Kit.Name = TEXT("six wheels, short legs");
		for (FRobotLeg& Leg : Kit.Robot.Legs)
		{
			Leg.MinZ = Leg.StanceZ - 6.0;
			Leg.MaxZ = Leg.StanceZ + 6.0;
		}
		const double  SlopeDeg = 8.0;
		const double  KeptDeg = SlopeDeg - FMath::RadiansToDegrees(FMath::Atan(3.5 / 45.0));
		const FGround Course = RampUp(SlopeDeg, 300.0);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.DistanceCm = 680.0;
			Options.MaxSeconds = 60.0;
			Options.bKeepTrace = true;
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("slope past reach"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 680.0);
			// Every wheel is on the ramp from 165 to 375 cm of travel; settled
			// on it, the base holds the part the legs cannot take out.
			double Most = 0.0;
			double Least = 100.0;
			for (const FSample& Sample : Result.Trace)
			{
				if (Sample.TravelCm >= 310.0 && Sample.TravelCm <= 365.0)
				{
					Most = FMath::Max(Most, Inclination(Sample.TrueAttitudeDeg));
					Least = FMath::Min(Least, Inclination(Sample.TrueAttitudeDeg));
				}
			}
			TestTrue(Label(Kit, Rate, TEXT("on the ramp: the slope less the reach, to half a degree")), Most <= KeptDeg + 0.5 && Least >= KeptDeg - 0.5);
			TestTrue(Label(Kit, Rate, TEXT("never worse than the slope itself")), Result.PeakTiltDeg <= SlopeDeg);
			TestTrue(Label(Kit, Rate, TEXT("level again on the flat beyond")), Result.SteadyResidualDeg <= 0.5);
			TestEqual(Label(Kit, Rate, TEXT("levelling at the end")), static_cast<int32>(Result.FinalState), static_cast<int32>(EState::Active));
			TestTrue(Label(Kit, Rate, TEXT("no leg past its travel")), Result.MaxOffsetCm <= 6.0);
		}
	}

	// Far past reach: the same robot with 4 cm either way, on 14 deg. The
	// wanted tilt stops at the policy's 5 deg, and the 9 left are more than
	// the legs' 3.5 cm to the margin (4.45 deg): here the allocation really is
	// cut back, the legs sit on their margin, and the base keeps the rest.
	{
		FKit Kit = SixWheel();
		Kit.Name = TEXT("six wheels, shorter legs");
		for (FRobotLeg& Leg : Kit.Robot.Legs)
		{
			Leg.MinZ = Leg.StanceZ - 4.0;
			Leg.MaxZ = Leg.StanceZ + 4.0;
		}
		const double  SlopeDeg = 14.0;
		const double  KeptDeg = SlopeDeg - FMath::RadiansToDegrees(FMath::Atan(3.5 / 45.0));
		const FGround Course = RampUp(SlopeDeg, 300.0);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.DistanceCm = 680.0;
			Options.MaxSeconds = 120.0;
			Options.bKeepTrace = true;
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("slope far past reach"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 680.0);
			// On the ramp, and on the flat once the last wheel has left it (the
			// rear wheels do at 465 cm of travel).
			double Most = 0.0;
			double Least = 100.0;
			double MostBeyond = 0.0;
			double LevelAt = -1.0;
			double BeyondAt = -1.0;
			bool   bPastReachSaid = false;
			for (const FSample& Sample : Result.Trace)
			{
				const double Tilt = Inclination(Sample.TrueAttitudeDeg);
				if (Sample.TravelCm >= 310.0 && Sample.TravelCm <= 365.0)
				{
					Most = FMath::Max(Most, Tilt);
					Least = FMath::Min(Least, Tilt);
					bPastReachSaid = bPastReachSaid || Sample.Out.Status == EStatus::PastReach;
				}
				if (Sample.TravelCm >= 470.0)
				{
					BeyondAt = BeyondAt < 0.0 ? Sample.Seconds : BeyondAt;
					MostBeyond = FMath::Max(MostBeyond, Tilt);
					if (Tilt > 0.5)
					{
						LevelAt = -1.0;
					}
					else if (LevelAt < 0.0)
					{
						LevelAt = Sample.Seconds;
					}
				}
			}
			AddInfo(FString::Printf(TEXT("%s | slope far past reach | %s | on the flat beyond: at most %.2f deg off level, level to half a degree %.2f s after the last wheel left the ramp"), Kit.Name,
				FrameNames[Rate], MostBeyond, LevelAt >= 0.0 ? LevelAt - BeyondAt : -1.0));
			TestTrue(Label(Kit, Rate, TEXT("the allocation was cut back")), Result.SecondsAtReach > 1.0);
			TestTrue(Label(Kit, Rate, TEXT("... and it said so")), bPastReachSaid);
			TestTrue(Label(Kit, Rate, TEXT("on the ramp: the slope less the legs' hard reach, to half a degree")), Most <= KeptDeg + 0.5 && Least >= KeptDeg - 0.5);
			TestTrue(Label(Kit, Rate, TEXT("never worse than the slope itself")), Result.PeakTiltDeg <= SlopeDeg);
			TestTrue(Label(Kit, Rate, TEXT("no leg past its margin")), Result.MaxOffsetCm <= 3.5 + 0.01);
			TestTrue(Label(Kit, Rate, TEXT("level again on the flat beyond")), Result.SteadyResidualDeg <= 0.5);
			TestEqual(Label(Kit, Rate, TEXT("levelling at the end")), static_cast<int32>(Result.FinalState), static_cast<int32>(EState::Active));
		}
	}

	// The robot with ganged pairs has its roll capped at 6 deg: on 9 deg of
	// cross fall it plans to keep the other 3.
	//
	// Its front pair is carried 7 cm clear here, not its usual 4. With the
	// drive legs set 6 deg apart the rear pair rides on its uphill wheel, 2 cm
	// up, which the pitch correction takes out with a plane that also lets the
	// lifted front pair down by 2.7 cm; and that pair's own uphill wheel is
	// 2.3 cm nearer the ground at this roll. That is 5 cm of a 4 cm clearance:
	// at its usual height the front pair comes down, the robot ends up on it,
	// on one rear wheel and on ONE drive wheel, and 6 deg at the legs is then
	// no longer 6 deg at the base. The law has no term for a ganged pair's
	// half track (FLegReach does not carry one), so it cannot see this coming;
	// the last block below logs what happens. Here the pair is kept clear so
	// that the case tests the cap and nothing else.
	{
		FKit Kit = DrivePairs();
		Kit.Robot.Legs[3].StanceZ = -17.0;
		const double  SlopeDeg = 9.0;
		const double  KeptDeg = SlopeDeg - Kit.Tuning.MaxCorrectionDeg.Y;
		const FGround Course = Plane(0.0, SlopeDeg);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 20.0;
			Options.MaxSeconds = 10.0;
			Options.bKeepTrace = true;
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("cross fall past the roll cap"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			TestTrue(Label(Kit, Rate, TEXT("the base keeps the roll past the cap")), FMath::Abs(Result.SteadyResidualDeg - KeptDeg) <= 0.5);
			TestTrue(Label(Kit, Rate, TEXT("... and holds it steadily")), Result.SteadyTiltRangeDeg <= 0.3);
			TestTrue(Label(Kit, Rate, TEXT("never worse than the slope itself")), Result.PeakTiltDeg <= SlopeDeg + 0.3);
			if (Result.Trace.Num() > 0)
			{
				const FOutputs& Last = Result.Trace.Last().Out;
				TestNearlyEqual(Label(Kit, Rate, TEXT("the wanted roll is what the cap leaves")), Last.WantedDeg.Y, KeptDeg, 0.3);
				TestTrue(Label(Kit, Rate, TEXT("the roll correction sits on its cap")), Last.AppliedDeg.Y <= Kit.Tuning.MaxCorrectionDeg.Y + 1.0e-6 && Last.AppliedDeg.Y >= Kit.Tuning.MaxCorrectionDeg.Y - 0.5);
				// Both drive wheels and one rear wheel: the other rear wheel
				// is the only one of a standing leg off the ground.
				TestEqual(Label(Kit, Rate, TEXT("it stands on both drive wheels and one rear wheel")), Result.Trace.Last().Floating, 1);
				TestEqual(Label(Kit, Rate, TEXT("... and on nothing else")), Result.Trace.Last().Touching, 3);
			}
		}
	}

	// The same with the front pair at its usual 4 cm: logged, not judged (see
	// above). What it shows is a limit of the law as it stands.
	{
		const FKit	  Kit = DrivePairs();
		const FGround Course = Plane(0.0, 9.0);
		FOptions	  Options = MakeOptions(Kit, 0);
		Options.SpeedCmS = 20.0;
		Options.MaxSeconds = 10.0;
		Options.bKeepTrace = true;
		const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
		Report(*this, Kit, TEXT("cross fall past the roll cap, front pair 4 cm clear (it grounds)"), 0, Result);
		if (Result.Trace.Num() > 0)
		{
			const FSample& Last = Result.Trace.Last();
			AddInfo(FString::Printf(TEXT("drive pairs | front pair 4 cm clear on 9 deg of cross fall: wheels of standing legs off the ground at the end %d (1 is the rear pair's; 2 means a drive wheel too), base roll %.2f deg with %.2f deg at the legs"),
				Last.Floating, Last.TrueAttitudeDeg.Y, Last.Out.AppliedDeg.Y));
		}
	}
	return true;
}

// Catches: a loop that answers a knock with a larger one, or goes on
// rocking after it.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunKick,
	"Ramms.Level.DryRun.AttitudeKick",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunKick::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FGround Flat = Plane(0.0, 0.0);
	// See below: what may be left 0.6 s after the knock (deg): half the
	// knock.
	const double KickAnswerBoundDeg = 1.8;
	for (const FKit& Kit : Kits())
	{
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 20.0;
			Options.MaxSeconds = 8.0;
			Options.bKeepTrace = true;
			FDryEvent Kick = MakeEvent(2.0, EEvent::AttitudeKick, 0.3);
			Kick.PlaneDeg = FVector2D(3.0, -2.0);
			Options.Events.Add(Kick);
			const FResult Result = DryRun::Run(Kit.Robot, Flat, Options);
			Report(*this, Kit, TEXT("attitude kick"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			// The knock itself is added to the pose, whatever the legs do,
			// so the whole run's peak is the knock's. What the legs did
			// about it shows once it has died away: 0.6 s on it is down to
			// 0.49 deg (3.6 e^-2), and anything over that is the legs',
			// sent against a "slope" that has gone.
			const double AfterDeg = PeakTiltBetween(Result, Kit.Tuning, 2.6, 100.0);
			const double LaterDeg = PeakTiltBetween(Result, Kit.Tuning, 3.5, 100.0);
			AddInfo(FString::Printf(TEXT("%s | attitude kick | %s | from 0.6 s after the knock: at most %.2f deg off level (the knock's own share by then 0.49); from 1.5 s after: %.2f deg"), Kit.Name,
				FrameNames[Rate], AfterDeg, LaterDeg));
			TestTrue(Label(Kit, Rate, TEXT("the legs' answer, once the knock has gone, is smaller than the knock")), AfterDeg <= KickAnswerBoundDeg);
			TestTrue(Label(Kit, Rate, TEXT("... and is over a second and a half after it")), LaterDeg <= 0.5);
			// The knock is 3.6 deg and dies away by itself in a third of a
			// second: the legs' answer must not add to it.
			TestTrue(Label(Kit, Rate, TEXT("the knock was felt")), Result.PeakTiltDeg >= 3.0);
			TestTrue(Label(Kit, Rate, TEXT("... and not made worse")), Result.PeakTiltDeg <= 4.2);
			TestTrue(Label(Kit, Rate, TEXT("level after it")), Result.SteadyResidualDeg <= 0.3);
			TestTrue(Label(Kit, Rate, TEXT("no rocking after it")), Result.SteadyTiltRangeDeg <= 0.2);
		}
	}
	return true;
}

// Catches: a terrain estimate or a loop thrown by one leg that is not where
// it was told to be (it must go on levelling with the others, and come back).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunStalledLeg,
	"Ramms.Level.DryRun.StalledLeg",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunStalledLeg::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		const FGround Course = RampUp(Kit.GradeDeg, 400.0);
		double		  Peaks[3] = { 0.0, 0.0, 0.0 };
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.DistanceCm = 420.0;
			// The same ramp with nothing stalled, to measure the stall by.
			const FResult Clean = DryRun::Run(Kit.Robot, Course, Options);
			// The first leg holds for a second just as the ramp comes
			// under it (each robot's own moment: a leg stalled while
			// nothing is asked of it stalls nothing).
			Options.Events.Add(MakeEvent(Kit.StallAtSeconds, EEvent::StallLeg, 1.0, 0.0, 0));
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("stalled leg"), Rate, Result);
			Peaks[Rate] = Result.PeakTiltDeg;
			AddInfo(FString::Printf(TEXT("%s | stalled leg | %s | peak %.2f deg against %.2f deg with no leg stalled"), Kit.Name, FrameNames[Rate], Result.PeakTiltDeg, Clean.PeakTiltDeg));
			TestFalse(Label(Kit, Rate, TEXT("no fault")), Result.bEverFaulted);
			TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 420.0);
			TestTrue(Label(Kit, Rate, TEXT("the stall cost something")), Result.PeakTiltDeg >= Clean.PeakTiltDeg + 0.1);
			// One leg held for a second through a break of the whole grade
			// (3 or 6 deg, which is what doing nothing would leave).
			TestTrue(Label(Kit, Rate, TEXT("... but not much: the others went on levelling")), Result.PeakTiltDeg <= 1.1);
			TestTrue(Label(Kit, Rate, TEXT("level on the ramp once the leg is back")), Result.SteadyResidualDeg <= 0.5);
			TestTrue(Label(Kit, Rate, TEXT("the speed scale never under its floor")), Result.MinSpeedScale >= Kit.Tuning.MinSpeedScale - 1.0e-9);
		}
		ExpectRatesAgree(*this, Kit, TEXT("the peak"), Peaks, 0.05);
	}
	return true;
}

// Catches: a law that needs its model of the legs to be right (travel 15
// percent more or less than it believes must still come level, and without
// hunting).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunLegGain,
	"Ramms.Level.DryRun.LegGainError",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunLegGain::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const double Gains[2] = { 1.15, 0.85 };
	for (const FKit& Kit : Kits())
	{
		const FGround Course = RampUp(Kit.GradeDeg, 400.0);
		for (const double Gain : Gains)
		{
			double Peaks[3] = { 0.0, 0.0, 0.0 };
			for (int32 Rate = 0; Rate < 3; ++Rate)
			{
				FOptions Options = MakeOptions(Kit, Rate);
				Options.DistanceCm = 420.0;
				Options.Events.Add(MakeEvent(0.0, EEvent::LegGain, 0.0, Gain));
				const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
				Report(*this, Kit, Gain > 1.0 ? TEXT("legs travel 15 percent more") : TEXT("legs travel 15 percent less"), Rate, Result);
				ExpectSound(*this, Kit, Rate, Result);
				Peaks[Rate] = Result.PeakTiltDeg;
				TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 420.0);
				// 0.3 to 0.45 deg with the feed-forward; 1.3 without.
				TestTrue(Label(Kit, Rate, TEXT("the peak through the break")), Result.PeakTiltDeg <= 0.7);
				// Within the deadband on each axis.
				TestTrue(Label(Kit, Rate, TEXT("level on the ramp to the deadband")), Result.SteadyResidualDeg <= 0.3);
				TestTrue(Label(Kit, Rate, TEXT("no hunting")), Result.SteadyTiltRangeDeg <= 0.2);
			}
			ExpectRatesAgree(*this, Kit, TEXT("the peak"), Peaks, 0.05);
		}
	}
	return true;
}

// Catches: a terrain estimate that a wrong support set bends (the legs the
// law is told do not stand follow the plane all the same here, and a plane
// through any of the legs that do is the same plane: a wrong set must cost
// ride height at most, never level).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunWrongSupport,
	"Ramms.Level.DryRun.WrongSupportSet",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunWrongSupport::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		const FGround Course = RampUp(Kit.GradeDeg, 500.0);
		double		  Peaks[3] = { 0.0, 0.0, 0.0 };
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.DistanceCm = 520.0;
			Options.bKeepTrace = true;
			// Told wrong for three seconds, once it is on the ramp and level.
			FDryEvent Wrong = MakeEvent(7.0, EEvent::WrongSupport, 3.0);
			Wrong.Mask = Kit.WrongMask;
			Options.Events.Add(Wrong);
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("wrong support set"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 520.0);
			Peaks[Rate] = PeakTiltBetween(Result, Kit.Tuning, 6.5, 100.0);
			AddInfo(FString::Printf(TEXT("%s | wrong support set | %s | while told wrong and after: at most %.2f deg off level"), Kit.Name, FrameNames[Rate], Peaks[Rate]));
			// A support set told wrong moves the height term and nothing
			// else: the base stays as level as it was on the ramp.
			TestTrue(Label(Kit, Rate, TEXT("level while told wrong, and when told right again")), Peaks[Rate] <= 0.3);
			TestTrue(Label(Kit, Rate, TEXT("level at the end")), Result.SteadyResidualDeg <= 0.5);
		}
		ExpectRatesAgree(*this, Kit, TEXT("the peak while told wrong"), Peaks, 0.05);
	}
	return true;
}

// Catches: a level assist that fights a drive mode's stance change, or loses
// level while the baselines move under its offsets.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunStanceRamp,
	"Ramms.Level.DryRun.StanceRampMidSlope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunStanceRamp::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		const FGround Course = Plane(Kit.GradeDeg, 0.0);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 20.0;
			Options.MaxSeconds = 11.0;
			Options.bKeepTrace = true;
			for (const int32 Leg : Kit.RampLegs)
			{
				Options.Events.Add(MakeEvent(5.0, EEvent::StanceRamp, 1.5, Kit.RampCm, Leg));
			}
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("stance ramp mid-slope"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			TestTrue(Label(Kit, Rate, TEXT("level before it")), PeakTiltBetween(Result, Kit.Tuning, 4.0, 5.0) <= 0.5);
			// The stance change alone would tilt the base by one to two degrees.
			TestTrue(Label(Kit, Rate, TEXT("level through it")), PeakTiltBetween(Result, Kit.Tuning, 5.0, 100.0) <= 1.0);
			TestTrue(Label(Kit, Rate, TEXT("level after it")), Result.SteadyResidualDeg <= 0.5);
			if (Result.Trace.Num() > 0)
			{
				// The baseline went where the drive mode put it: the offsets
				// are the law's, the baselines are not.
				const FSample& Last = Result.Trace.Last();
				const int32	   Leg = Kit.RampLegs[0];
				TestNearlyEqual(Label(Kit, Rate, TEXT("the baseline is where the stance change left it")), Last.BaselineZ[Leg], Kit.Robot.Legs[Leg].StanceZ + Kit.RampCm, 1.0e-6);
			}
		}
	}
	return true;
}

// Catches: a fold or an adopt that moves a leg (the correction counted twice
// or not at all across a take-over), and levelling that does not go on after
// the hand-back.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunFoldAdopt,
	"Ramms.Level.DryRun.FoldAdoptMidSlope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunFoldAdopt::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		const FGround Course = Plane(Kit.GradeDeg, 0.5 * Kit.CrossDeg);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 20.0;
			Options.MaxSeconds = 10.0;
			Options.bKeepTrace = true;
			Options.Events.Add(MakeEvent(5.0, EEvent::FoldAdopt, 1.0));
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("fold and adopt mid-slope"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);

			bool bFolded = false;
			bool bAdopted = false;
			bool bWroteWhileTaken = false;
			for (const FSample& Sample : Result.Trace)
			{
				bFolded = bFolded || Sample.Out.Action == EAction::Fold;
				bAdopted = bAdopted || Sample.Out.Action == EAction::Adopt;
				bWroteWhileTaken = bWroteWhileTaken || (Sample.Out.State == EState::Yielded && Sample.Out.bWrite);
			}
			TestTrue(Label(Kit, Rate, TEXT("the offsets were folded")), bFolded);
			TestTrue(Label(Kit, Rate, TEXT("... and a plane adopted back")), bAdopted);
			TestFalse(Label(Kit, Rate, TEXT("nothing was written in between")), bWroteWhileTaken);
			TestTrue(Label(Kit, Rate, TEXT("there was a plane to fold")), Result.MaxCorrectionDeg >= 0.8 * Kit.GradeDeg);
			TestTrue(Label(Kit, Rate, TEXT("no leg stepped across the fold and the adopt")), Result.MaxEventLegStepCm <= 0.05);
			TestTrue(Label(Kit, Rate, TEXT("level across them")), PeakTiltBetween(Result, Kit.Tuning, 4.5, 100.0) <= 0.5);
			TestEqual(Label(Kit, Rate, TEXT("levelling at the end")), static_cast<int32>(Result.FinalState), static_cast<int32>(EState::Active));
		}
	}
	return true;
}

// Catches: a switch that bumps the base on or off, a parked robot whose
// motors are written for ever, and a release that leaves the robot anywhere
// but exactly on its stance.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunEnableDisable,
	"Ramms.Level.DryRun.EnableAndDisableParked",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunEnableDisable::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		const FGround Course = Plane(Kit.GradeDeg, 0.5 * Kit.CrossDeg);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 0.0;
			Options.bEnabledAtStart = false;
			Options.MaxSeconds = 22.0;
			Options.bKeepTrace = true;
			Options.Events.Add(MakeEvent(1.0, EEvent::Enable));
			Options.Events.Add(MakeEvent(9.0, EEvent::Disable));
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("enable and disable, parked"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);

			const bool	 bTraced = Result.Trace.Num() > 0;
			const double StartDeg = bTraced ? Inclination(Result.Trace[0].TrueAttitudeDeg) : 0.0;
			TestTrue(Label(Kit, Rate, TEXT("parked on the slope")), StartDeg >= 0.9 * Kit.GradeDeg);
			TestTrue(Label(Kit, Rate, TEXT("off until switched on")), bTraced && Result.Trace[0].Out.State == EState::Off);
			TestTrue(Label(Kit, Rate, TEXT("... and untouched")), PeakTiltBetween(Result, Kit.Tuning, 0.0, 0.9) <= StartDeg + 1.0e-6);
			TestTrue(Label(Kit, Rate, TEXT("level before it is switched off")), PeakTiltBetween(Result, Kit.Tuning, 7.5, 8.9) <= 0.5);
			int32 Writes = 0;
			for (const FSample& Sample : Result.Trace)
			{
				Writes += Sample.Seconds >= 7.5 && Sample.Seconds <= 8.9 && Sample.Out.bWrite ? 1 : 0;
			}
			TestEqual(Label(Kit, Rate, TEXT("parked and level: no command changes")), Writes, 0);
			// 3 deg/s by the policy, and a little for the legs catching up.
			TestTrue(Label(Kit, Rate, TEXT("on and off within the tilt policy")), Result.MaxTrueRateDegPerSec <= 3.5);
			TestTrue(Label(Kit, Rate, TEXT("never tilted further than the slope")), Result.PeakTiltDeg <= StartDeg + 0.3);

			TestEqual(Label(Kit, Rate, TEXT("off at the end")), static_cast<int32>(Result.FinalState), static_cast<int32>(EState::Off));
			TestTrue(Label(Kit, Rate, TEXT("every offset exactly zero")), Result.FinalMaxOffsetCm == 0.0);
			TestEqual(Label(Kit, Rate, TEXT("nothing written once off")), Result.SteadyWrites, 0);
			TestNearlyEqual(Label(Kit, Rate, TEXT("back on its stance, as it started")), Result.FinalTiltDeg, StartDeg, 0.02);
		}
	}
	return true;
}

// Catches: a loop that turns sensor noise into a limit cycle of the legs, or
// that cannot settle on a biased reading.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunNoise,
	"Ramms.Level.DryRun.NoiseAndBias",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunNoise::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		const FGround Course = Plane(Kit.GradeDeg, 0.0);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 20.0;
			Options.MaxSeconds = 10.0;
			Options.NoiseDeg = 0.2;
			Options.BiasDeg = FVector2D(0.5, -0.5);
			Options.bDelayOneFrame = true;
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("noise 0.2 deg, bias 0.5 deg, a frame late"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			// The base is levelled to the sensor, so it rests the bias off
			// (0.71 deg over the two axes), give or take the deadband.
			TestTrue(Label(Kit, Rate, TEXT("level to the sensor's bias")), Result.SteadyResidualDeg <= Inclination(Options.BiasDeg) + 0.4);
			TestTrue(Label(Kit, Rate, TEXT("no limit cycle in the attitude")), Result.SteadyTiltRangeDeg <= 0.5);
			TestTrue(Label(Kit, Rate, TEXT("... nor in the legs")), Result.SteadyLegRangeCm <= 0.6);
			TestTrue(Label(Kit, Rate, TEXT("the noise does not slow the drive to its floor")), Result.MinSpeedScale > 0.5);

			// One seed is one draw: the same again on four more.
			double MostRangeDeg = Result.SteadyTiltRangeDeg;
			double MostLegRangeCm = Result.SteadyLegRangeCm;
			for (int32 Seed = 2; Seed <= 5; ++Seed)
			{
				Options.Seed = Seed;
				const FResult Other = DryRun::Run(Kit.Robot, Course, Options);
				MostRangeDeg = FMath::Max(MostRangeDeg, Other.SteadyTiltRangeDeg);
				MostLegRangeCm = FMath::Max(MostLegRangeCm, Other.SteadyLegRangeCm);
				TestFalse(Label(Kit, Rate, TEXT("another seed: no fault")), Other.bEverFaulted);
			}
			AddInfo(FString::Printf(TEXT("%s | noise, five seeds | %s | tilt range at most %.2f deg (seed 1: %.2f) | leg range at most %.2f cm (seed 1: %.2f)"), Kit.Name, FrameNames[Rate], MostRangeDeg,
				Result.SteadyTiltRangeDeg, MostLegRangeCm, Result.SteadyLegRangeCm));
			// The worst draw is rougher than the first (the table says by how
			// much); a limit cycle, as the sweep's rows with a 0.1 s terrain
			// filter show one, is 1.2 deg and a centimetre or more. (The
			// robot with a raised pair reads 0.81 deg on its worst draw at
			// 16 fps, where it read 0.65 while the raised pair was fitted
			// into the achieved correction: that pair answers slowest, and
			// with the sensor a frame late its lateness happened to stand in
			// for the frame. The estimate is now exact for the legs that
			// carry, and the late frame shows in full.)
			TestTrue(Label(Kit, Rate, TEXT("no limit cycle on any of five seeds")), MostRangeDeg <= 0.9 && MostLegRangeCm <= 0.7);
		}
	}
	return true;
}

// Catches: a terrain estimate that takes the commanded correction for the
// achieved one (joints that give under load would then read as a slope, and
// the estimate would chase the correction).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunSag,
	"Ramms.Level.DryRun.JointSag",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunSag::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		const FGround Course = RampUp(Kit.GradeDeg, 400.0);
		double		  Peaks[3] = { 0.0, 0.0, 0.0 };
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.DistanceCm = 420.0;
			// Half a centimetre with the load shared evenly; more where the
			// weight is.
			Options.SagCm = 0.5;
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("joints sag under load"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			Peaks[Rate] = Result.PeakTiltDeg;
			TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 420.0);
			// Under half a degree with the feed-forward; 1.3 without.
			TestTrue(Label(Kit, Rate, TEXT("the peak through the break")), Result.PeakTiltDeg <= 0.8);
			TestTrue(Label(Kit, Rate, TEXT("level on the ramp")), Result.SteadyResidualDeg <= 0.5);
			TestTrue(Label(Kit, Rate, TEXT("no hunting")), Result.SteadyTiltRangeDeg <= 0.3);
		}
		ExpectRatesAgree(*this, Kit, TEXT("the peak"), Peaks, 0.05);
	}
	return true;
}

// Catches: a sign or geometry error left to run (legs that move the wrong
// way must be caught, ramped out with no snap, and stay off).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunBackwards,
	"Ramms.Level.DryRun.SignFlippedPlantFaults",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunBackwards::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (FKit Kit : Kits())
	{
		// The correction stops at a limit the legs can give in full: a
		// runaway that ran into the legs' reach first would read as a slope
		// past reach, which the divergence watch leaves alone.
		Kit.Tuning.MaxCorrectionDeg = FVector2D(Kit.BackwardsLimitDeg, Kit.BackwardsLimitDeg);
		const FGround Course = Plane(Kit.BackwardsGroundDeg, 0.0);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 0.0;
			Options.MaxSeconds = 20.0;
			// Every leg moves the other way from what it is told.
			Options.Events.Add(MakeEvent(0.0, EEvent::LegGain, 0.0, -1.0));
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("legs wired backwards"), Rate, Result);
			TestEqual(Label(Kit, Rate, TEXT("every pose a stable rest")), Result.UnstableFrames, 0);
			TestTrue(Label(Kit, Rate, TEXT("it ran away")), Result.PeakTiltDeg >= FMath::Abs(Kit.BackwardsGroundDeg) + 0.8 * Kit.BackwardsLimitDeg);
			TestTrue(Label(Kit, Rate, TEXT("... and was caught")), Result.bEverFaulted);
			TestEqual(Label(Kit, Rate, TEXT("faulted at the end")), static_cast<int32>(Result.FinalState), static_cast<int32>(EState::Faulted));
			TestEqual(Label(Kit, Rate, TEXT("and says so")), static_cast<int32>(Result.FinalStatus), static_cast<int32>(EStatus::Diverged));
			TestTrue(Label(Kit, Rate, TEXT("every offset exactly zero")), Result.FinalMaxOffsetCm == 0.0);
			TestNearlyEqual(Label(Kit, Rate, TEXT("back on its stance")), Result.FinalTiltDeg, FMath::Abs(Kit.BackwardsGroundDeg), 0.05);
			TestEqual(Label(Kit, Rate, TEXT("nothing written once latched")), Result.SteadyWrites, 0);
			TestTrue(Label(Kit, Rate, TEXT("the speed scale never under its floor")), Result.MinSpeedScale >= Kit.Tuning.MinSpeedScale - 1.0e-9);
		}
	}
	return true;
}

// The gain sweep that fixes the defaults: feed-forward on or off, the
// integral gain from 1 to 6, the terrain filter from 0.1 to 0.3 s, at the
// slowest and the fastest frame rate. Every row is logged, with two runs
// behind it: a grade break taken at speed (what a quick estimate and a
// strong integral are good for) and a slow drive on a steady grade with a
// noisy, biased sensor a frame late (what they cost). Catches: defaults that
// are not among the settings that level a ramp well, a feed-forward that
// does not earn its place, and a setting in the sweep that hunts.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunGainSweep,
	"Ramms.Level.DryRun.GainSweep",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunGainSweep::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const double FeedForwards[2] = { 1.0, 0.0 };
	const double IntegralGains[5] = { 1.0, 2.0, 3.0, 4.0, 6.0 };
	const double TerrainFilters[4] = { 0.1, 0.15, 0.2, 0.3 };
	const int32	 Rates[2] = { 0, 2 };
	for (const FKit& Base : Kits())
	{
		const FGround Break = RampUp(Base.GradeDeg, 400.0);
		const FGround Steady = Plane(Base.GradeDeg, 0.0);
		for (const int32 Rate : Rates)
		{
			const auto RunBreak = [&](const FKit& Kit) {
				FOptions Options = MakeOptions(Kit, Rate);
				Options.DistanceCm = 420.0;
				return DryRun::Run(Kit.Robot, Break, Options);
			};
			const auto RunNoise = [&](const FKit& Kit) {
				FOptions Options = MakeOptions(Kit, Rate);
				Options.SpeedCmS = 20.0;
				Options.MaxSeconds = 10.0;
				Options.NoiseDeg = 0.2;
				Options.BiasDeg = FVector2D(0.5, -0.5);
				Options.bDelayOneFrame = true;
				return DryRun::Run(Kit.Robot, Steady, Options);
			};

			// The defaults: the reference.
			const FResult Default = RunBreak(Base);
			const FResult DefaultNoise = RunNoise(Base);
			Report(*this, Base, TEXT("sweep: the defaults"), Rate, Default);

			double BestWithPeak = Default.PeakTiltDeg;
			double BestWithoutPeak = 1.0e6;
			double BestWithLegRange = DefaultNoise.SteadyLegRangeCm;
			for (const double FeedForward : FeedForwards)
			{
				for (const double IntegralGain : IntegralGains)
				{
					for (const double TerrainFilter : TerrainFilters)
					{
						FKit Kit = Base;
						Kit.Tuning.FeedForwardGain = FeedForward;
						Kit.Tuning.IntegralGain = IntegralGain;
						Kit.Tuning.TerrainFilterSeconds = TerrainFilter;
						if (FeedForward <= 0.0)
						{
							// The pure-PI fallback: the integral has to hold the
							// whole correction.
							Kit.Tuning.IntegralMaxDeg = 15.0;
						}
						const FResult Result = RunBreak(Kit);
						const FResult Noise = RunNoise(Kit);
						AddInfo(FString::Printf(TEXT("SWEEP | %s | %s | Kff %.0f | Ki %.0f | terrain %.2f s | break: peak %.2f deg | lag %.2f s | steady %.2f deg | range %.2f deg | mean %.1f cm/s | noise: tilt range %.2f deg | leg range %.2f cm | scale >= %.2f%s"),
							Kit.Name, FrameNames[Rate], FeedForward, IntegralGain, TerrainFilter, Result.PeakTiltDeg, Result.LongestOffLevelSeconds, Result.SteadyResidualDeg,
							Result.SteadyTiltRangeDeg, Result.MeanSpeedCmS, Noise.SteadyTiltRangeDeg, Noise.SteadyLegRangeCm, Noise.MinSpeedScale,
							(Result.bEverFaulted || Noise.bEverFaulted) ? TEXT(" | FAULTED") : TEXT("")));
						const FString Setting = FString::Printf(TEXT("%s, %s, Kff %.0f, Ki %.0f, terrain %.2f s"), Kit.Name, FrameNames[Rate], FeedForward, IntegralGain, TerrainFilter);
						if (FeedForward > 0.0)
						{
							BestWithPeak = FMath::Min(BestWithPeak, Result.PeakTiltDeg);
							BestWithLegRange = FMath::Min(BestWithLegRange, Noise.SteadyLegRangeCm);
							// With the feed-forward the law is not touchy on a clean
							// sensor: every setting in the sweep levels the ramp.
							// (On a noisy one it is, at 16 frames a second with a
							// terrain filter of 0.1 s: the rows say so, and only
							// the defaults are judged on it, below.)
							TestFalse(Setting + TEXT(": no fault"), Result.bEverFaulted || Noise.bEverFaulted);
							TestTrue(Setting + TEXT(": level on the ramp"), Result.SteadyResidualDeg <= 0.5);
							TestTrue(Setting + TEXT(": no hunting"), Result.SteadyTiltRangeDeg <= 0.3);
						}
						else
						{
							BestWithoutPeak = FMath::Min(BestWithoutPeak, Result.PeakTiltDeg);
						}
					}
				}
			}
			TestTrue(FString::Printf(TEXT("%s, %s: the defaults are within a quarter of a degree of the best peak in the sweep (%.2f against %.2f)"), Base.Name, FrameNames[Rate],
						 Default.PeakTiltDeg, BestWithPeak),
				Default.PeakTiltDeg <= BestWithPeak + 0.25);
			TestTrue(FString::Printf(TEXT("%s, %s: the feed-forward is no worse through the break than the best without it (%.2f against %.2f)"), Base.Name, FrameNames[Rate],
						 Default.PeakTiltDeg, BestWithoutPeak),
				Default.PeakTiltDeg <= BestWithoutPeak + 0.25);
			TestTrue(FString::Printf(TEXT("%s, %s: on a noisy sensor the defaults move the legs within a quarter of a centimetre of the quietest setting (%.2f against %.2f)"), Base.Name,
						 FrameNames[Rate], DefaultNoise.SteadyLegRangeCm, BestWithLegRange),
				DefaultNoise.SteadyLegRangeCm <= BestWithLegRange + 0.25);
		}

		// Below the slowest rate asked for: 10 frames a second, the noisy
		// sensor, the feed-forward on, Ki 2. Logged, not judged -- the low-pass
		// closes Dt / Seconds of the gap each frame, so a slower frame is a
		// quicker filter, and this is where the terrain filter's time runs out.
		const double SlowFilters[3] = { 0.15, 0.2, 0.3 };
		for (const double TerrainFilter : SlowFilters)
		{
			FKit Kit = Base;
			Kit.Tuning.TerrainFilterSeconds = TerrainFilter;
			FOptions Options = MakeOptions(Kit, 0);
			Options.FrameSeconds = 0.1;
			Options.SpeedCmS = 20.0;
			Options.MaxSeconds = 10.0;
			Options.NoiseDeg = 0.2;
			Options.BiasDeg = FVector2D(0.5, -0.5);
			Options.bDelayOneFrame = true;
			const FResult Noise = DryRun::Run(Kit.Robot, Steady, Options);
			AddInfo(FString::Printf(TEXT("SLOW | %s | 10 fps | Kff 1 | Ki 2 | terrain %.2f s | noise: tilt range %.2f deg | leg range %.2f cm | scale >= %.2f%s"), Kit.Name, TerrainFilter,
				Noise.SteadyTiltRangeDeg, Noise.SteadyLegRangeCm, Noise.MinSpeedScale, Noise.bEverFaulted ? TEXT(" | FAULTED") : TEXT("")));
		}
	}
	return true;
}

// Catches: break figures that hold only for a sensor with no latency. Every
// other course reads the attitude solved on the very millisecond the law
// runs (the exact source); here the reading is a frame old, as a sensor's
// would be, through the same four breaks as RampUpAndDown. The frame of
// delay is a frame of lag nothing cancels, so it costs more the slower the
// frame: the rates are NOT expected to agree here, and the figures logged
// are the ones to carry over to an attitude source that is not exact.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunLateSensor,
	"Ramms.Level.DryRun.GradeBreaksWithASensorAFrameLate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunLateSensor::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		const double	Top = Rise(200.0, Kit.GradeDeg);
		const FVector2D Knots[4] = { { 120.0, 0.0 }, { 320.0, Top }, { 520.0, Top }, { 720.0, 0.0 } };
		const FGround	Course = Profile(MakeArrayView(Knots));
		double			Late[3] = { 0.0, 0.0, 0.0 };
		double			Exact[3] = { 0.0, 0.0, 0.0 };
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.DistanceCm = 900.0;
			Exact[Rate] = DryRun::Run(Kit.Robot, Course, Options).PeakTiltDeg;
			Options.bDelayOneFrame = true;
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("ramp up and down, sensor a frame late"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			Late[Rate] = Result.PeakTiltDeg;
			TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 900.0);
			TestTrue(Label(Kit, Rate, TEXT("a late reading does not help")), Late[Rate] >= Exact[Rate] - 0.02);
			// Still well under what the integral alone leaves with an exact
			// sensor (1.3 deg): the feed-forward earns its place a frame late.
			TestTrue(Label(Kit, Rate, TEXT("the peak through the breaks")), Late[Rate] <= LateSensorPeakBoundDeg);
			TestTrue(Label(Kit, Rate, TEXT("level again on the flat")), Result.SteadyResidualDeg <= 0.5);
			TestTrue(Label(Kit, Rate, TEXT("no ringing at the end")), Result.SteadyTiltRangeDeg <= 0.3);
		}
		AddInfo(FString::Printf(TEXT("%s | sensor a frame late | peak through the breaks %.2f / %.2f / %.2f deg at 16 / 30 / 60 fps, against %.2f / %.2f / %.2f deg read exactly"), Kit.Name, Late[0], Late[1], Late[2],
			Exact[0], Exact[1], Exact[2]));
		TestTrue(FString::Printf(TEXT("%s: a frame of delay costs most at the slowest frame"), Kit.Name), Late[0] - Exact[0] >= Late[2] - Exact[2] - 0.02);
	}
	return true;
}

// Catches: figures that hold only at walking pace. Every other course is
// driven at 40 cm/s or less, where the legs never come near their rate
// through a break; here the same grade break is taken at 40, 80 and 120
// cm/s, to show where a law that only reacts runs out: the peak grows with
// the speed, the legs reach their rate, and the speed cap takes over.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunAtSpeed,
	"Ramms.Level.DryRun.GradeBreakAtSpeed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunAtSpeed::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const double	   Speeds[3] = { 40.0, 80.0, 120.0 };
	const TCHAR* const Cases[3] = { TEXT("grade break at 40 cm/s"), TEXT("grade break at 80 cm/s"), TEXT("grade break at 120 cm/s") };
	for (const FKit& Kit : Kits())
	{
		const FGround Course = RampUp(Kit.GradeDeg, 900.0);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			double Peaks[3] = { 0.0, 0.0, 0.0 };
			for (int32 SpeedIndex = 0; SpeedIndex < 3; ++SpeedIndex)
			{
				FOptions Options = MakeOptions(Kit, Rate);
				Options.SpeedCmS = Speeds[SpeedIndex];
				Options.DistanceCm = 800.0;
				Options.bKeepTrace = true;
				const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
				Report(*this, Kit, Cases[SpeedIndex], Rate, Result);
				ExpectSound(*this, Kit, Rate, Result);
				Peaks[SpeedIndex] = Result.PeakTiltDeg;

				double LimitedSeconds = 0.0;
				double MostUse = 0.0;
				for (const FSample& Sample : Result.Trace)
				{
					LimitedSeconds += Sample.Out.bRateLimited ? Options.FrameSeconds : 0.0;
					MostUse = FMath::Max(MostUse, Sample.Out.RateUse);
				}
				AddInfo(FString::Printf(TEXT("%s | %s | %s | legs rate-limited %.2f s, at most %.2f of their rate asked | least speed %.1f cm/s"), Kit.Name, Cases[SpeedIndex], FrameNames[Rate], LimitedSeconds,
					MostUse, Result.MinSpeedCmS));
				TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 800.0);
				TestTrue(Label(Kit, Rate, TEXT("level on the grade")), Result.SteadyResidualDeg <= 0.5);
				TestTrue(Label(Kit, Rate, TEXT("no hunting")), Result.SteadyTiltRangeDeg <= 0.3);
				// At 40 cm/s the base is a third of a degree behind; at three
				// times that, the robot with the quicker legs a degree (of 6)
				// and the one with the slower legs 1.65 (of 3), its legs at
				// their rate for half a second and the drive slowed for it.
				// That is where a law that only reacts runs out.
				TestTrue(Label(Kit, Rate, TEXT("the peak through the break")), Result.PeakTiltDeg <= SpeedPeakShare * Kit.GradeDeg);
				TestTrue(Label(Kit, Rate, TEXT("most of the speed kept")), Result.MeanSpeedCmS >= 0.6 * Speeds[SpeedIndex]);
			}
			TestTrue(Label(Kit, Rate, TEXT("the faster the break is taken, the further the base is left behind")), Peaks[1] > Peaks[0] && Peaks[2] > Peaks[1]);
		}
	}
	return true;
}

namespace RammsLevelDryRunTests
{
	/** The tilt each made-up robot's rider prefers in the preference cases
	 *  (deg; pitch, roll): a few degrees, both axes, inside what its legs can
	 *  give on top of its courses' slopes. */
	FVector2D PreferredTilt(const FKit& Kit)
	{
		return Kit.Robot.Legs.Num() == 6 ? FVector2D(2.0, -1.5) : FVector2D(1.5, 1.0);
	}

	/** The true attitude at a time (the last frame at or before it). */
	FVector2D TrueAttitudeAt(const FResult& Result, double Seconds)
	{
		FVector2D Out = FVector2D::ZeroVector;
		for (const FSample& Sample : Result.Trace)
		{
			if (Sample.Seconds > Seconds + 1.0e-6)
			{
				break;
			}
			Out = Sample.TrueAttitudeDeg;
		}
		return Out;
	}

	/** The level reference the law is given for a rider's preference, made as the component makes it (the reflected
	 *  tuning's own reference, the preference added within its bounds): the preference cases then run on what the
	 *  level.pref controls would give the law. */
	FVector2D ReferenceFor(const FVector2D& PreferenceDeg)
	{
		return MakeTuning(FRammsLevelTuning(), PreferenceDeg).LevelReferenceDeg;
	}

	const FSample* SampleAt(const FResult& Result, double Seconds)
	{
		const FSample* Out = nullptr;
		for (const FSample& Sample : Result.Trace)
		{
			if (Sample.Seconds > Seconds + 1.0e-6)
			{
				break;
			}
			Out = &Sample;
		}
		return Out;
	}
} // namespace RammsLevelDryRunTests

// Catches: a law that levels to the horizontal whatever the rider prefers
// (the reference left out of the wanted attitude's goal), a preference held
// on one kind of ground and lost on another, and a preference reached by a
// snap rather than on the tilt policy.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunPreferredTiltHeld,
	"Ramms.Level.DryRun.PreferredTiltHeld",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunPreferredTiltHeld::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		const FVector2D Preferred = PreferredTilt(Kit);
		const double	CrossDeg = 0.5 * Kit.CrossDeg;
		const FVector2D GradeKnots[2] = { { 120.0, 0.0 }, { 620.0, Rise(500.0, Kit.GradeDeg) } };
		struct FCourse
		{
			const TCHAR* Name;
			FGround		 Ground;
			/** The ground's own attitude where the run starts and where it
			 *  ends (deg). */
			FVector2D StartGroundDeg;
			FVector2D EndGroundDeg;
			double	  DistanceCm;
			double	  MaxSeconds;
		};
		const FCourse Courses[3] = {
			{ TEXT("preferred tilt, ramp"), RampUp(Kit.GradeDeg, 500.0), FVector2D::ZeroVector, FVector2D(Kit.GradeDeg, 0.0), 480.0, 40.0 },
			{ TEXT("preferred tilt, cross slope"), Plane(0.0, Kit.CrossDeg), FVector2D(0.0, Kit.CrossDeg), FVector2D(0.0, Kit.CrossDeg), 0.0, 9.0 },
			{ TEXT("preferred tilt, grade with cross fall"), Profile(MakeArrayView(GradeKnots), CrossDeg), FVector2D(0.0, CrossDeg), FVector2D(Kit.GradeDeg, CrossDeg), 480.0, 40.0 },
		};
		for (const FCourse& Course : Courses)
		{
			double Steadies[3] = { 0.0, 0.0, 0.0 };
			for (int32 Rate = 0; Rate < 3; ++Rate)
			{
				FOptions Options = MakeOptions(Kit, Rate);
				Options.Tuning.LevelReferenceDeg = ReferenceFor(Preferred);
				TestTrue(Label(Kit, Rate, TEXT("(the preference is inside its bounds: the law is given all of it)")), Options.Tuning.LevelReferenceDeg.Equals(Preferred, 1.0e-9));
				Options.DistanceCm = Course.DistanceCm;
				Options.MaxSeconds = Course.MaxSeconds;
				Options.bKeepTrace = true;
				const FResult Result = DryRun::Run(Kit.Robot, Course.Ground, Options);
				Report(*this, Kit, Course.Name, Rate, Result);
				ExpectSound(*this, Kit, Rate, Result);
				Steadies[Rate] = Result.SteadyResidualDeg;
				if (!TestTrue(Label(Kit, Rate, TEXT("a trace")), Result.Trace.Num() > 0))
				{
					continue;
				}
				const FVector2D End = Result.Trace.Last().TrueAttitudeDeg;
				// It starts as the ground has it and is walked to the
				// preference on the tilt policy: that far at its rate, the time
				// to get up to that rate, and a little for the legs. From there
				// on what is left is the grade breaks'.
				const double WalkedBy = FMath::Max(2.6,
					(Preferred - Course.StartGroundDeg).Size() / Kit.Tuning.TiltRateDegPerSec + Kit.Tuning.TiltRateDegPerSec / Kit.Tuning.TiltAccelDegPerSec2 + 0.4);
				const double BreakPeakDeg = PeakTiltBetween(Result, Options.Tuning, WalkedBy, 100.0);
				AddInfo(FString::Printf(TEXT("%s | %s | %s | prefers (%.1f, %.1f) deg on ground ending (%.1f, %.1f): ends at (%.2f, %.2f), at most %.2f deg off its preference from %.1f s on"),
					Kit.Name, Course.Name, FrameNames[Rate], Preferred.X, Preferred.Y, Course.EndGroundDeg.X, Course.EndGroundDeg.Y, End.X, End.Y, BreakPeakDeg, WalkedBy));
				// (off level here is off the preference: the reference is the
				// preference, RammsLevelDryRun.h.)
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: held at the preferred tilt"), Course.Name)), Result.SteadyResidualDeg <= 0.5);
				TestEqual(Label(Kit, Rate, *FString::Printf(TEXT("%s: the preferred pitch"), Course.Name)), End.X, Preferred.X, 0.25);
				TestEqual(Label(Kit, Rate, *FString::Printf(TEXT("%s: the preferred roll"), Course.Name)), End.Y, Preferred.Y, 0.25);
				// ... which is neither level nor the ground's own attitude.
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: not levelled to the horizontal"), Course.Name)), Inclination(End) >= Inclination(Preferred) - 0.4);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: the legs took the ground out"), Course.Name)), (End - Course.EndGroundDeg).Size() >= 1.0);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: the peak once at the preference"), Course.Name)), BreakPeakDeg <= 0.7);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: walked there, not snapped"), Course.Name)), Result.MaxTrueRateDegPerSec <= 3.5);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: inside the legs' reach"), Course.Name)), Result.SecondsAtReach == 0.0);
			}
			ExpectRatesAgree(*this, Kit, Course.Name, Steadies, Kit.Tuning.DeadbandDeg);
		}
	}
	return true;
}

// Catches: a change of the preferred tilt that steps the rider (the wanted
// attitude set to its new goal at once, or moved past the tilt policy's rate
// or acceleration), one that overshoots, and one that a robot at rest on a
// slope never answers (settled, it stopped looking at where level is).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunPreferenceChange,
	"Ramms.Level.DryRun.PreferenceChangeOnASlope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunPreferenceChange::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		// Parked on a grade with a cross fall. Level first; at 6 s the rider
		// asks for the preferred tilt, at 11 s for its opposite, at 17 s for
		// level again.
		const FGround	Course = Plane(Kit.GradeDeg, 0.5 * Kit.CrossDeg);
		const FVector2D Preferred = PreferredTilt(Kit);
		const double	ChangeAt[3] = { 6.0, 11.0, 17.0 };
		const FVector2D References[4] = { FVector2D::ZeroVector, Preferred, -Preferred, FVector2D::ZeroVector };
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 0.0;
			Options.MaxSeconds = 22.0;
			Options.bKeepTrace = true;
			for (int32 Change = 0; Change < 3; ++Change)
			{
				FDryEvent Event = MakeEvent(ChangeAt[Change], EEvent::SetReference);
				Event.PlaneDeg = ReferenceFor(References[Change + 1]);
				Options.Events.Add(Event);
			}
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("preference change, parked on a slope"), Rate, Result);
			// The wanted attitude within the tilt policy's rate and
			// acceleration over the whole run, the three changes included.
			ExpectSound(*this, Kit, Rate, Result);
			TestTrue(Label(Kit, Rate, TEXT("the rider moved, and no faster than the tilt policy allows")),
				Result.MaxTrueRateDegPerSec >= 1.0 && Result.MaxTrueRateDegPerSec <= 3.5);
			TestTrue(Label(Kit, Rate, TEXT("inside the legs' reach throughout")), Result.SecondsAtReach == 0.0);
			TestTrue(Label(Kit, Rate, TEXT("level again at the end")), Result.FinalTiltDeg <= 0.4);

			// At rest before the first change: settled, nothing written.
			const FSample* Before = SampleAt(Result, ChangeAt[0] - 0.1);
			if (TestNotNull(Label(Kit, Rate, TEXT("a frame before the first change")), Before))
			{
				TestTrue(Label(Kit, Rate, TEXT("settled level before the change")), Before->Out.bQuiet && Inclination(Before->TrueAttitudeDeg) <= 0.4);
			}
			for (int32 Change = 0; Change < 3; ++Change)
			{
				const FVector2D From = References[Change];
				const FVector2D To = References[Change + 1];
				const double	Until = Change < 2 ? ChangeAt[Change + 1] : Options.MaxSeconds;
				const double	Length = (To - From).Size();
				const FVector2D Along = (To - From) / Length;
				// The change wakes it, and it is at the new preference well
				// before the next one.
				const FSample* Woken = SampleAt(Result, ChangeAt[Change] + 0.5);
				const FSample* There = SampleAt(Result, Until - 0.1);
				if (!TestNotNull(Label(Kit, Rate, TEXT("frames after the change")), Woken) || !TestNotNull(Label(Kit, Rate, TEXT("frames before the next")), There))
				{
					continue;
				}
				TestFalse(Label(Kit, Rate, *FString::Printf(TEXT("change %d: it is answered (not left at rest)"), Change + 1)), Woken->Out.bQuiet);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("change %d: the wanted attitude arrives"), Change + 1)), There->Out.WantedDeg.Equals(To, 0.05));
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("change %d: the rider is held at the new preference"), Change + 1)), (There->TrueAttitudeDeg - To).Size() <= 0.4);
				// No overshoot, and no wandering off the straight line between
				// the two: frame by frame from the change to the next.
				double	  MostAlong = 0.0, MostWantedAlong = 0.0, MostAcross = 0.0, MostStepDeg = 0.0;
				FVector2D Last = TrueAttitudeAt(Result, ChangeAt[Change] - 1.0e-3);
				for (const FSample& Sample : Result.Trace)
				{
					if (Sample.Seconds < ChangeAt[Change] - 1.0e-6 || Sample.Seconds >= Until - 1.0e-6)
					{
						continue;
					}
					MostAlong = FMath::Max(MostAlong, FVector2D::DotProduct(Sample.TrueAttitudeDeg - From, Along));
					MostWantedAlong = FMath::Max(MostWantedAlong, FVector2D::DotProduct(Sample.Out.WantedDeg - From, Along));
					MostAcross = FMath::Max(MostAcross, FMath::Abs(FVector2D::CrossProduct(Sample.Out.WantedDeg - From, Along)));
					MostStepDeg = FMath::Max(MostStepDeg, (Sample.TrueAttitudeDeg - Last).Size());
					Last = Sample.TrueAttitudeDeg;
				}
				AddInfo(FString::Printf(TEXT("%s | preference change %d | %s | %.2f deg asked: rider at most %.2f deg along it, wanted %.3f; wanted off the line %.3f; largest step in a frame %.3f deg"),
					Kit.Name, Change + 1, FrameNames[Rate], Length, MostAlong, MostWantedAlong, MostAcross, MostStepDeg));
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("change %d: the wanted attitude never passes the preference"), Change + 1)), MostWantedAlong <= Length + 1.0e-6);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("change %d: the wanted attitude goes straight there"), Change + 1)), MostAcross <= 0.02);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("change %d: the rider does not overshoot"), Change + 1)), MostAlong <= Length + 0.1);
				// A frame's worth of the policy's rate, and the legs' lag.
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("change %d: no step in any one frame"), Change + 1)), MostStepDeg <= 3.5 * FrameSeconds[Rate] + 0.05);
			}
		}
	}
	return true;
}

// Catches: a change of the preferred tilt made on the move, as the ground
// changes under the robot, that is lost (the grade break's own correction
// taken for the answer), that steps the rider, or that takes the wanted
// attitude past the tilt policy while both are being answered at once.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunPreferenceChangeOnTheMove,
	"Ramms.Level.DryRun.PreferenceChangeOverAGradeBreak",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunPreferenceChangeOnTheMove::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		// Driving level ground onto a grade; the rider asks for the preferred
		// tilt as the first wheels meet it.
		const FVector2D Preferred = PreferredTilt(Kit);
		const FGround	Course = RampUp(Kit.GradeDeg, 500.0);
		double			Steadies[3] = { 0.0, 0.0, 0.0 };
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.DistanceCm = 480.0;
			Options.MaxSeconds = 40.0;
			Options.bKeepTrace = true;
			FDryEvent Event = MakeEvent(Kit.StallAtSeconds, EEvent::SetReference);
			Event.PlaneDeg = ReferenceFor(Preferred);
			Options.Events.Add(Event);
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("preference change over a grade break"), Rate, Result);
			// The wanted attitude within the tilt policy's rate and
			// acceleration throughout, the change and the break together.
			ExpectSound(*this, Kit, Rate, Result);
			Steadies[Rate] = Result.SteadyResidualDeg;
			if (!TestTrue(Label(Kit, Rate, TEXT("a trace")), Result.Trace.Num() > 0))
			{
				continue;
			}
			const FSample*	Before = SampleAt(Result, Kit.StallAtSeconds - 0.2);
			const FVector2D End = Result.Trace.Last().TrueAttitudeDeg;
			// The most the rider was off the reference in force from when the
			// walk to the preference is over.
			const double WalkedBy = Kit.StallAtSeconds + Preferred.Size() / Kit.Tuning.TiltRateDegPerSec + Kit.Tuning.TiltRateDegPerSec / Kit.Tuning.TiltAccelDegPerSec2 + 0.4;
			double		 MostOffDeg = 0.0;
			for (const FSample& Sample : Result.Trace)
			{
				if (Sample.Seconds >= WalkedBy)
				{
					MostOffDeg = FMath::Max(MostOffDeg, Inclination(Sample.TrueAttitudeDeg - Preferred));
				}
			}
			AddInfo(FString::Printf(TEXT("%s | preference change over a grade break | %s | asked (%.1f, %.1f) deg at %.1f s: ends at (%.2f, %.2f) on %.1f deg of grade, at most %.2f deg off it from %.1f s on"),
				Kit.Name, FrameNames[Rate], Preferred.X, Preferred.Y, Kit.StallAtSeconds, End.X, End.Y, Kit.GradeDeg, MostOffDeg, WalkedBy));
			if (TestNotNull(Label(Kit, Rate, TEXT("a frame before the change")), Before))
			{
				TestTrue(Label(Kit, Rate, TEXT("level before the change")), Inclination(Before->TrueAttitudeDeg) <= 0.4 && Before->Out.WantedDeg.Size() <= 0.05);
			}
			TestTrue(Label(Kit, Rate, TEXT("it got onto the grade")), Result.TravelCm >= 480.0);
			TestEqual(Label(Kit, Rate, TEXT("the preferred pitch, on the grade")), End.X, Preferred.X, 0.25);
			TestEqual(Label(Kit, Rate, TEXT("the preferred roll, on the grade")), End.Y, Preferred.Y, 0.25);
			TestTrue(Label(Kit, Rate, TEXT("the wanted attitude is the preference")), Result.Trace.Last().Out.WantedDeg.Equals(Preferred, 0.05));
			TestTrue(Label(Kit, Rate, TEXT("held at the preference")), Result.SteadyResidualDeg <= 0.5);
			TestTrue(Label(Kit, Rate, TEXT("the grade break costs no more with the change than without")), MostOffDeg <= 0.7);
			TestTrue(Label(Kit, Rate, TEXT("the rider moved, and no faster than the tilt policy allows")), Result.MaxTrueRateDegPerSec >= 1.0 && Result.MaxTrueRateDegPerSec <= 3.5);
			TestTrue(Label(Kit, Rate, TEXT("inside the legs' reach")), Result.SecondsAtReach == 0.0);
		}
		ExpectRatesAgree(*this, Kit, TEXT("preference change over a grade break"), Steadies, Kit.Tuning.DeadbandDeg);
	}
	return true;
}

// Catches: a preference chosen while the level assist was off that is
// snapped to when it is switched on (the wanted attitude must start where
// the base is and walk), or forgotten (levelled to the horizontal).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunPreferenceBeforeSwitchOn,
	"Ramms.Level.DryRun.PreferenceSetBeforeSwitchingOn",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunPreferenceBeforeSwitchOn::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		// Parked on a grade with a cross fall, off, the preference already
		// chosen; switched on at 2 s.
		const FVector2D GroundDeg(Kit.GradeDeg, 0.5 * Kit.CrossDeg);
		const FGround	Course = Plane(GroundDeg.X, GroundDeg.Y);
		const FVector2D Preferred = PreferredTilt(Kit);
		const double	OnAt = 2.0;
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.Tuning.LevelReferenceDeg = ReferenceFor(Preferred);
			Options.SpeedCmS = 0.0;
			Options.MaxSeconds = 12.0;
			Options.bEnabledAtStart = false;
			Options.bKeepTrace = true;
			Options.Events.Add(MakeEvent(OnAt, EEvent::Enable));
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("preference set before switching on"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			const FSample* Off = SampleAt(Result, OnAt - 0.2);
			if (!TestNotNull(Label(Kit, Rate, TEXT("a frame while off")), Off) || !TestTrue(Label(Kit, Rate, TEXT("a trace")), Result.Trace.Num() > 0))
			{
				continue;
			}
			const FVector2D Stood = Off->TrueAttitudeDeg;
			TestTrue(Label(Kit, Rate, TEXT("off: the base lies as the ground has it, nothing on the legs")),
				Off->Out.State == EState::Off && (Stood - GroundDeg).Size() <= 0.3 && Result.Trace[0].Out.Dz.Num() > 0);
			// The first frame it is levelling on: the wanted attitude is where
			// the base is, not where the rider prefers.
			const FSample* First = nullptr;
			for (const FSample& Sample : Result.Trace)
			{
				if (Sample.Out.State == EState::Active)
				{
					First = &Sample;
					break;
				}
			}
			const FVector2D End = Result.Trace.Last().TrueAttitudeDeg;
			AddInfo(FString::Printf(TEXT("%s | preference set before switching on | %s | stood at (%.2f, %.2f), prefers (%.1f, %.1f): first wanted (%.2f, %.2f), ends at (%.2f, %.2f); largest leg step in a frame %.3f cm"),
				Kit.Name, FrameNames[Rate], Stood.X, Stood.Y, Preferred.X, Preferred.Y, First ? First->Out.WantedDeg.X : 0.0, First ? First->Out.WantedDeg.Y : 0.0, End.X, End.Y, Result.MaxLegStepCm));
			if (TestNotNull(Label(Kit, Rate, TEXT("it came on")), First))
			{
				TestTrue(Label(Kit, Rate, TEXT("switched on: the wanted attitude starts where the base is")), (First->Out.WantedDeg - Stood).Size() <= 0.3);
				TestTrue(Label(Kit, Rate, TEXT("(which is not the preference)")), (Stood - Preferred).Size() >= 2.0);
			}
			TestTrue(Label(Kit, Rate, TEXT("walked to the preference, not snapped")), Result.MaxTrueRateDegPerSec >= 1.0 && Result.MaxTrueRateDegPerSec <= 3.5);
			// A frame of the policy's rate at the farthest leg, and a little.
			TestTrue(Label(Kit, Rate, TEXT("no leg stepped")), Result.MaxLegStepCm <= 0.6);
			TestEqual(Label(Kit, Rate, TEXT("the preferred pitch")), End.X, Preferred.X, 0.25);
			TestEqual(Label(Kit, Rate, TEXT("the preferred roll")), End.Y, Preferred.Y, 0.25);
			TestTrue(Label(Kit, Rate, TEXT("held at the preference")), Result.SteadyResidualDeg <= 0.4);
		}
	}
	return true;
}

// Catches: a preference that, with the ground's slope, asks more of the legs
// than they have, taken for a fault or answered by legs past their margin;
// the wanted tilt's allowance and the past-reach status measured from the
// horizontal rather than from what the rider asked for; and the base not
// brought as near the preference as the law plans to.
//
// What the rider gets, by the law's own rule (WantedTiltGoal): the part of
// the asked correction the legs' PLANNED reach (their travel less the margin
// and the soft zone) cannot give is kept as tilt off the preference, up to
// MaxTiltDeg; resting there is Levelling or Settled, not "at reach". Asked
// for more than that, the legs go to their margin, the base keeps the rest,
// and the status is "slope past levelling reach".
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunPreferencePastReach,
	"Ramms.Level.DryRun.PreferencePastTheLegsReach",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunPreferencePastReach::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	// The six-wheeled robot with its legs cut to 4 cm either way, parked nose
	// up. At the corners' 45 cm the legs plan on 1.5 cm (0.5 margin and 2
	// soft zone off): 1.91 deg; to the margin they have 3.5 cm: 4.45 deg.
	FKit Kit = SixWheel();
	Kit.Name = TEXT("six wheels, shorter legs");
	for (FRobotLeg& Leg : Kit.Robot.Legs)
	{
		Leg.MinZ = Leg.StanceZ - 4.0;
		Leg.MaxZ = Leg.StanceZ + 4.0;
	}
	const double PlannedDeg = FMath::RadiansToDegrees(FMath::Atan(1.5 / 45.0));
	const double HardDeg = FMath::RadiansToDegrees(FMath::Atan(3.5 / 45.0));
	struct FCase
	{
		const TCHAR* Name;
		double		 GroundDeg;
		double		 PreferredDeg;
		/** Where the base ends (deg, pitch), and whether that is past the
		 *  tilt policy's allowance off the preference. */
		double EndsAtDeg;
		bool   bPast;
	};
	const FCase Cases[2] = {
		// 3 deg nose up, 3 deg nose down asked: 6 asked of a planned 1.91.
		// The other 4.09 is kept, inside the 5 deg allowance.
		{ TEXT("preference a little past the legs' planned reach"), 3.0, -3.0, 3.0 - PlannedDeg, false },
		// 6 deg nose up, 4 deg nose down asked: 10 asked. The wanted tilt
		// stops 5 deg off the preference, which asks 5 of the legs' 4.45: the
		// legs sit on their margin and the base is 5.55 deg off the
		// preference. Off the horizontal it is only 1.55 deg, well inside the
		// allowance: measured from there nothing would be said.
		{ TEXT("preference far past the legs' reach"), 6.0, -4.0, 6.0 - HardDeg, true },
	};
	for (const FCase& Case : Cases)
	{
		const FVector2D Preferred(Case.PreferredDeg, 0.0);
		const double	ShortDeg = Case.EndsAtDeg - Case.PreferredDeg;
		const FGround	Course = Plane(Case.GroundDeg, 0.0);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.Tuning.LevelReferenceDeg = ReferenceFor(Preferred);
			Options.SpeedCmS = 0.0;
			Options.MaxSeconds = 14.0;
			Options.bKeepTrace = true;
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, Case.Name, Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			if (!TestTrue(Label(Kit, Rate, TEXT("a trace")), Result.Trace.Num() > 0))
			{
				continue;
			}
			const FSample& Last = Result.Trace.Last();
			AddInfo(FString::Printf(TEXT("%s | %s | %s | %.1f deg ground, prefers %.1f: ends at (%.2f, %.2f), %.2f deg off the preference; wanted (%.2f, %.2f); status %d; at reach %.2f s; largest offset %.2f cm"),
				Kit.Name, Case.Name, FrameNames[Rate], Case.GroundDeg, Case.PreferredDeg, Last.TrueAttitudeDeg.X, Last.TrueAttitudeDeg.Y, Result.SteadyResidualDeg, Last.Out.WantedDeg.X,
				Last.Out.WantedDeg.Y, static_cast<int32>(Last.Out.Status), Result.SecondsAtReach, Result.MaxOffsetCm));
			TestEqual(Label(Kit, Rate, *FString::Printf(TEXT("%s: where the base ends"), Case.Name)), Last.TrueAttitudeDeg.X, Case.EndsAtDeg, 0.3);
			TestEqual(Label(Kit, Rate, *FString::Printf(TEXT("%s: no roll comes of it"), Case.Name)), Last.TrueAttitudeDeg.Y, 0.0, 0.2);
			// (Off the horizontal it would read EndsAtDeg, 2.5 deg and more away.)
			TestEqual(Label(Kit, Rate, *FString::Printf(TEXT("%s: the shortfall is measured against the preference"), Case.Name)), Result.SteadyResidualDeg, ShortDeg, 0.3);
			TestEqual(Label(Kit, Rate, *FString::Printf(TEXT("%s: still levelling"), Case.Name)), static_cast<int32>(Result.FinalState), static_cast<int32>(EState::Active));
			if (Case.bPast)
			{
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: the wanted tilt stops at the policy's allowance off the preference"), Case.Name)),
					FMath::IsNearlyEqual(Last.Out.WantedDeg.X, Case.PreferredDeg + Kit.Tuning.MaxTiltDeg, 0.05));
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: the allocation was cut back"), Case.Name)), Result.SecondsAtReach > 1.0);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: no leg past its margin"), Case.Name)), Result.MaxOffsetCm <= 3.5 + 0.01);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: it says the slope is past reach"), Case.Name)), Last.Out.Status == EStatus::PastReach);
			}
			else
			{
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: the wanted tilt takes what the legs do not plan to"), Case.Name)),
					FMath::IsNearlyEqual(Last.Out.WantedDeg.X, Case.EndsAtDeg, 0.1));
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: the legs stay out of their soft zone"), Case.Name)), Result.MaxOffsetCm <= 1.5 + 0.15 && Result.SecondsAtReach == 0.0);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: resting there is not a warning"), Case.Name)),
					Last.Out.Status == EStatus::Settled || Last.Out.Status == EStatus::Levelling);
			}
		}
	}
	return true;
}

namespace RammsLevelDryRunTests
{
	/**
	 * The six-wheeled robot standing on its centre and rear legs, as a stance
	 * for driving on two axles does: the front pair carried 12 cm clear of the
	 * ground and 0.3 cm short of its retract stop. Its centre of mass is moved
	 * aft to sit well inside the four that stand.
	 */
	FKit RaisedFronts()
	{
		FKit Kit = SixWheel();
		Kit.Name = TEXT("raised fronts");
		for (int32 Index = 0; Index < 2; ++Index)
		{
			Kit.Robot.Legs[Index].StanceZ = -18.0;
			Kit.Robot.Legs[Index].MaxZ = -17.7;
		}
		Kit.Robot.CentreOfMass = FVector(-18.0, 0.0, 10.0);
		return Kit;
	}

	/**
	 * The six-wheeled robot with legs that can make a centimetre inside one
	 * frame (60 cm/s), and that do not all answer alike: the rear pair picks
	 * up speed a tenth as fast as the others.
	 */
	FKit QuickLegs()
	{
		FKit Kit = SixWheel();
		Kit.Name = TEXT("quick legs");
		for (int32 Index = 0; Index < Kit.Robot.Legs.Num(); ++Index)
		{
			Kit.Robot.Legs[Index].MaxRateCmS = 60.0;
			Kit.Robot.Legs[Index].MaxAccelCmS2 = Index >= 4 ? 60.0 : 600.0;
		}
		Kit.Robot.CentreOfMass = FVector(-18.0, 0.0, 10.0);
		return Kit;
	}

	/** When the law's state first was State at or after FromSeconds (s); -1 if never. */
	double FirstTimeIn(const FResult& Result, EState State, double FromSeconds)
	{
		for (const FSample& Sample : Result.Trace)
		{
			if (Sample.Seconds >= FromSeconds && Sample.Out.State == State)
			{
				return Sample.Seconds;
			}
		}
		return -1.0;
	}
} // namespace RammsLevelDryRunTests

// Catches: a ground estimate bent by legs that do not carry the robot. A
// raised leg asked to retract with the plane stops on its retract stop; fitted
// with the rest it makes the correction read short, the base settles short of
// level and never reads at rest, and a release walks to the wrong ground and
// waits out its time limit.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunRaisedLegsOnStop,
	"Ramms.Level.DryRun.RaisedLegsOnTheirStop",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunRaisedLegsOnStop::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FKit Kit = RaisedFronts();
	struct FCase
	{
		const TCHAR* Name;
		FVector2D	 GroundDeg;
	};
	// Nose up and the +Y side up are the ways that ask a raised front leg to
	// retract; nose down is the control, where both extend freely.
	const FCase Cases[4] = { { TEXT("nose up"), FVector2D(6.0, 0.0) }, { TEXT("cross"), FVector2D(0.0, 6.0) }, { TEXT("both"), FVector2D(4.0, -4.0) }, { TEXT("nose down"), FVector2D(-6.0, 0.0) } };
	for (const FCase& Case : Cases)
	{
		const FGround Course = Plane(Case.GroundDeg.X, Case.GroundDeg.Y);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 0.0;
			Options.bEnabledAtStart = false;
			Options.MaxSeconds = 24.0;
			Options.bKeepTrace = true;
			Options.Events.Add(MakeEvent(1.0, EEvent::Enable));
			Options.Events.Add(MakeEvent(12.0, EEvent::Disable));
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, Case.Name, Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			if (Result.Trace.Num() == 0)
			{
				AddError(TEXT("no trace"));
				continue;
			}
			const auto Say = [&](const TCHAR* What) { return FString::Printf(TEXT("%s, %s, %s: %s"), Kit.Name, Case.Name, FrameNames[Rate], What); };

			const FVector2D StartDeg = Result.Trace[0].TrueAttitudeDeg;
			TestTrue(Say(TEXT("parked on the slope, on four legs")), (StartDeg - Case.GroundDeg).Size() <= 0.3 && Result.Trace[0].Touching == 4);

			const FSample* Levelled = nullptr;
			bool		   bStopped = false;
			for (const FSample& Sample : Result.Trace)
			{
				if (Sample.Seconds <= 11.9)
				{
					Levelled = &Sample;
					// (At 0.3 cm from the stop; the command may ask past it.)
					bStopped = bStopped || Sample.CommandZ[0] > Kit.Robot.Legs[0].MaxZ - 1.0e-6 || Sample.CommandZ[1] > Kit.Robot.Legs[1].MaxZ - 1.0e-6;
				}
			}
			if (!Levelled)
			{
				continue;
			}
			if (Case.GroundDeg.X > 0.0 || Case.GroundDeg.Y != 0.0)
			{
				TestTrue(Say(TEXT("a raised leg was sent to its stop")), bStopped);
			}
			const double LevelDeg = PeakTiltBetween(Result, Kit.Tuning, 9.0, 11.9);
			AddInfo(FString::Printf(TEXT("%s | %s | %s | level to %.2f deg; ground read as (%.2f, %.2f) of (%.2f, %.2f); off %.2f s after the switch"), Kit.Name, Case.Name, FrameNames[Rate],
				LevelDeg, Levelled->Out.GroundDeg.X, Levelled->Out.GroundDeg.Y, StartDeg.X, StartDeg.Y, FirstTimeIn(Result, EState::Off, 12.0) - 12.0));
			TestTrue(Say(TEXT("level, to the deadband")), LevelDeg <= 0.3);
			TestEqual(Say(TEXT("... and at rest")), static_cast<int32>(Levelled->Out.Status), static_cast<int32>(EStatus::Settled));
			// What the release walks back to.
			TestTrue(Say(TEXT("the ground is read as it is")), (Levelled->Out.GroundDeg - StartDeg).Size() <= 0.25);

			// Switched off: the tilt policy's walk back (6 deg at 3 deg/s, and
			// 0.75 s to pick the rate up and lose it), not the 6 s time limit.
			const double OffAt = FirstTimeIn(Result, EState::Off, 12.0);
			TestTrue(Say(TEXT("switched off: back on the stance in the walk's own time")), OffAt >= 12.0 && OffAt - 12.0 <= 4.5);
			TestTrue(Say(TEXT("... within the tilt policy")), Result.MaxTrueRateDegPerSec <= 3.5);
			TestTrue(Say(TEXT("... every offset exactly zero")), Result.FinalMaxOffsetCm == 0.0);
			TestNearlyEqual(Say(TEXT("... as it started")), Result.FinalTiltDeg, Inclination(StartDeg), 0.02);
		}
	}
	return true;
}

// Catches: every standing leg's offset stepping in one frame when a drive
// mode's stance ramp lifts legs off the ground, or sets them down, under a
// levelled base (the mean the offsets are taken about jumps with the support
// set): on legs quick enough to make the step inside a frame, and not all
// alike, the base pitches with it.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunStanceRampLiftsLegs,
	"Ramms.Level.DryRun.StanceRampLiftsLegsWhileLevelling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunStanceRampLiftsLegs::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FKit	  Kit = QuickLegs();
	const FGround Course = Plane(-6.0, 0.0);
	for (int32 Rate = 0; Rate < 3; ++Rate)
	{
		FOptions Options = MakeOptions(Kit, Rate);
		Options.SpeedCmS = 0.0;
		Options.MaxSeconds = 20.0;
		Options.bKeepTrace = true;
		// A switch to a drive mode it levels in, 3 s in: levelling starts at
		// once, and 1.1 s later the mode's stance ramp draws the front pair
		// up 10 cm. They leave the support set with some 3 deg of the slope
		// taken out. Then the switch back, at 12 s: the release starts, and
		// the front pair is set down again under it.
		Options.Events.Add(MakeEvent(0.0, EEvent::ModeRelease, 3.0));
		Options.Events.Add(MakeEvent(4.1, EEvent::StanceRamp, 1.25, 10.0, 0));
		Options.Events.Add(MakeEvent(4.1, EEvent::StanceRamp, 1.25, 10.0, 1));
		Options.Events.Add(MakeEvent(12.0, EEvent::ModeRelease, 100.0));
		Options.Events.Add(MakeEvent(12.0, EEvent::StanceRamp, 1.25, -10.0, 0));
		Options.Events.Add(MakeEvent(12.0, EEvent::StanceRamp, 1.25, -10.0, 1));
		const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
		Report(*this, Kit, TEXT("stance ramp lifts the fronts"), Rate, Result);
		ExpectSound(*this, Kit, Rate, Result);

		// The legs the ramp does not move: how fast their commands went, and
		// the base with them, frame to frame.
		double MostLegRate[2] = { 0.0, 0.0 };
		double MostBaseRate[2] = { 0.0, 0.0 };
		double MostCommon[2] = { 0.0, 0.0 };
		for (int32 Index = 1; Index < Result.Trace.Num(); ++Index)
		{
			const FSample& Now = Result.Trace[Index];
			const FSample& Was = Result.Trace[Index - 1];
			const int32	   Part = Now.Seconds < 11.0 ? 0 : 1;
			double		   Common = 0.0;
			for (int32 Leg = 2; Leg < 6; ++Leg)
			{
				const double Move = Now.CommandZ[Leg] - Was.CommandZ[Leg];
				MostLegRate[Part] = FMath::Max(MostLegRate[Part], FMath::Abs(Move) / Options.FrameSeconds);
				Common += 0.25 * Move;
			}
			MostCommon[Part] = FMath::Max(MostCommon[Part], FMath::Abs(Common) / Options.FrameSeconds);
			MostBaseRate[Part] = FMath::Max(MostBaseRate[Part], (Now.TrueAttitudeDeg - Was.TrueAttitudeDeg).Size() / Options.FrameSeconds);
		}
		AddInfo(FString::Printf(TEXT("%s | stance ramp lifts the fronts | %s | lifting: legs %.1f cm/s, their mean %.1f cm/s, base %.1f deg/s a frame; setting down: %.1f, %.1f, %.1f; over 0.3 s %.2f deg/s"),
			Kit.Name, FrameNames[Rate], MostLegRate[0], MostCommon[0], MostBaseRate[0], MostLegRate[1], MostCommon[1], MostBaseRate[1], Result.MaxTrueRateDegPerSec));
		for (int32 Part = 0; Part < 2; ++Part)
		{
			const TCHAR* What = Part == 0 ? TEXT("lifting") : TEXT("setting down");
			// The ride height's 3 cm/s, and a hair.
			TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: the standing legs' common height moves at the ride height's rate (%.2f cm/s)"), What, MostCommon[Part])),
				MostCommon[Part] <= Kit.Tuning.RideHeightRateCmPerSec + 0.1);
			// ... and the plane's own part on top: 3 deg/s on a 45 cm arm.
			TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: no standing leg is stepped (%.2f cm/s)"), What, MostLegRate[Part])), MostLegRate[Part] <= 7.0);
			TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: the base is not jolted (%.2f deg/s in a frame)"), What, MostBaseRate[Part])), MostBaseRate[Part] <= 4.5);
		}
		TestTrue(Label(Kit, Rate, TEXT("within the tilt policy throughout")), Result.MaxTrueRateDegPerSec <= 3.5);
		TestTrue(Label(Kit, Rate, TEXT("level on four legs before the switch back")), PeakTiltBetween(Result, Kit.Tuning, 9.5, 11.9) <= 0.5);
		TestTrue(Label(Kit, Rate, TEXT("the fronts did leave the ground")), Result.Trace.Num() > 0 && Result.Trace.Last().Touching == 6 && Result.Trace[Result.Trace.Num() / 2].Touching == 4);
		TestEqual(Label(Kit, Rate, TEXT("waiting at the end")), static_cast<int32>(Result.FinalState), static_cast<int32>(EState::Suspended));
		TestTrue(Label(Kit, Rate, TEXT("... every offset exactly zero")), Result.FinalMaxOffsetCm == 0.0);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
