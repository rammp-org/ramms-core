// Tests for the level assist's kinematic dry run (Private/Level/RammsLevelDryRun):
// the made-up grounds, the rest solve on its own, and then the real law
// (FLaw::Step) run once a frame against the dry run's plant -- over ramps,
// cross slopes, crests and sags, a crown, a grade with a cross fall, an
// oblique fold and a slope past the legs' reach, and through the things that
// go wrong: a knock, a stalled leg, legs that travel more or less than the
// law believes, a wrong support set, a stance change and a take-over in the
// middle of a slope, the switch thrown while parked, a noisy and biased
// sensor, joints that sag, and a base wired backwards; and the same grade
// breaks with the sensor a frame late, and at two and three times the pace;
// a hard stop from speed and a knock, against the same with the switch off;
// a stance ramped under a base at rest; a slope past what the clearance
// of wheels carried off the ground allows; a taker's hand-back to a stance
// its pose is centimetres off; and a drive mode switched on a slope, into the
// mode that is levelled and out of it.
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
	constexpr double SpeedPeakShare = 0.65;

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
		/** How far apart the legs it stands on are (cm; the lesser of fore and
		 *  aft and across, as the law takes it): the speed cap keeps it to the
		 *  speed at which a grade's end turns the ground 6 deg/s under them. */
		double SpanCm = 0.0;
		/** How much correction it may carry at speed (FCarryLimit::Tuning). */
		FCarryTuning Carry;
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
		// 90 cm fore and aft, 54 across.
		Kit.SpanCm = 54.0;
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
		// Its courses show the law's own roll levelling, at a walk. That a robot
		// of this kind is to carry roll at a crawl only (it stands on three
		// points with any) is tried where that rule is the subject
		// (RollCarriedAtACrawlOnly); here the rule is off.
		Kit.Carry.bThreePoint = false;
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
		// The drive wheels to the rear pair, 40 cm; 56 across.
		Kit.SpanCm = 40.0;
		return Kit;
	}

	TArray<FKit> Kits()
	{
		TArray<FKit> Out;
		Out.Add(SixWheel());
		Out.Add(DrivePairs());
		return Out;
	}

	/**
	 * A kit for the cases whose subject is how the legs FOLLOW the ground at speed, or what the speed cap and the seat
	 * do for that: the crest rule (how much correction a robot may carry at the speed it has, FCarryTuning) is off, so
	 * that the base is levelled all the way up a slope at any speed, as those cases were written for. What the rule
	 * does, and costs, is CarriedAtSpeed's subject.
	 */
	FKit FollowingOnly(FKit Kit)
	{
		Kit.Carry.bCrest = false;
		return Kit;
	}

	FOptions MakeOptions(const FKit& Kit, int32 Rate)
	{
		FOptions Options;
		Options.Tuning = Kit.Tuning;
		Options.Carry = Kit.Carry;
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

	/** A base's swing between two times, from a trace: the true pitch's
	 *  peak-to-peak (deg) and the true attitude's largest rate over 0.3 s
	 *  (deg/s) of the frames in between. */
	struct FSwing
	{
		double RangeDeg = 0.0;
		double RateDegPerSec = 0.0;
	};

	FSwing SwingBetween(const FResult& Result, double FromSeconds, double ToSeconds, double TickSeconds)
	{
		FSwing		Out;
		double		Low = 0.0;
		double		High = 0.0;
		bool		bAny = false;
		const int32 Window = FMath::Max(1, FMath::RoundToInt32(0.3 / TickSeconds));
		for (int32 Index = 0; Index < Result.Trace.Num(); ++Index)
		{
			const FSample& Sample = Result.Trace[Index];
			if (Sample.Seconds < FromSeconds || Sample.Seconds > ToSeconds)
			{
				continue;
			}
			Low = bAny ? FMath::Min(Low, Sample.TrueAttitudeDeg.X) : Sample.TrueAttitudeDeg.X;
			High = bAny ? FMath::Max(High, Sample.TrueAttitudeDeg.X) : Sample.TrueAttitudeDeg.X;
			bAny = true;
			if (Index >= Window)
			{
				Out.RateDegPerSec = FMath::Max(Out.RateDegPerSec, (Sample.TrueAttitudeDeg - Result.Trace[Index - Window].TrueAttitudeDeg).Size() / (Window * TickSeconds));
			}
		}
		Out.RangeDeg = High - Low;
		return Out;
	}

	/** The law as it was before it asked how far the ground can have turned:
	 *  the attitude as read taken for the ground's, at once. */
	FGroundTuning NoGate()
	{
		FGroundTuning Tuning;
		Tuning.BreakDeg = 0.0;
		return Tuning;
	}

	/** What a change of speed does to the made-up robots' bodies (deg of
	 *  pitch per cm/s^2): 1.2 deg at a fifth of gravity. */
	constexpr double BodyPitchPerAccel = 0.006;
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
			// (Parked, the ground cannot have turned: the law takes the base's
			// going the wrong way for the body's own doing and follows it
			// only slowly, so the runaway is caught some degrees in, not at
			// the correction's limit.)
			TestTrue(Label(Kit, Rate, TEXT("it ran away")), Result.PeakTiltDeg >= FMath::Abs(Kit.BackwardsGroundDeg) + DivergenceErrorDeg);
			TestTrue(Label(Kit, Rate, TEXT("... not as far as the correction's limit")), Result.PeakTiltDeg <= FMath::Abs(Kit.BackwardsGroundDeg) + Kit.BackwardsLimitDeg);
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
	for (const FKit& Each : Kits())
	{
		const FKit	  Kit = FollowingOnly(Each);
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
				{
					// The ground that turns under a robot at speed is followed
					// as it was before the law asked how far the ground can
					// have turned: the same run with the reading taken for the
					// ground at once.
					FOptions Before = Options;
					Before.Ground = NoGate();
					const FResult Was = DryRun::Run(Kit.Robot, Course, Before);
					AddInfo(FString::Printf(TEXT("%s | %s | %s | peak %.2f deg, lag %.2f s; with the reading taken for the ground at once %.2f deg, %.2f s"), Kit.Name, Cases[SpeedIndex], FrameNames[Rate],
						Result.PeakTiltDeg, Result.LongestOffLevelSeconds, Was.PeakTiltDeg, Was.LongestOffLevelSeconds));
					TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: the break is followed as well as with no gate (%.2f against %.2f deg)"), Cases[SpeedIndex], Result.PeakTiltDeg, Was.PeakTiltDeg)),
						Result.PeakTiltDeg <= Was.PeakTiltDeg + 0.1);
					TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: ... and no later"), Cases[SpeedIndex])), Result.LongestOffLevelSeconds <= Was.LongestOffLevelSeconds + 0.07);
				}

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
				// and the one with the slower legs 1.8 (of 3), its legs at
				// their rate for half a second and the drive slowed for it.
				// That is where a law that only reacts runs out.
				TestTrue(Label(Kit, Rate, TEXT("the peak through the break")), Result.PeakTiltDeg <= SpeedPeakShare * Kit.GradeDeg);
				// (On the grade itself nothing is asked of the drive: a steady
				// slope the legs hold level costs the rider no speed.)
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("the speed kept (%.1f of %.1f cm/s)"), Result.MeanSpeedCmS, Speeds[SpeedIndex])), Result.MeanSpeedCmS >= 0.93 * Speeds[SpeedIndex]);
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
// MaxTiltDeg; resting there the base is held off the preference by the legs'
// reach, and the status says so ("at reach", with what stops the legs) for
// as long as it is. Asked for more than that, the legs go to their margin,
// the base keeps the rest, and the status is "slope past levelling reach".
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
				// Held off the preference by the legs' reach: said, steadily,
				// for the whole of the last two seconds, with what stops them.
				int32 NotSaid = 0;
				for (const FSample& Sample : Result.Trace)
				{
					NotSaid += Sample.Seconds >= Last.Seconds - 2.0 && !(Sample.Out.Status == EStatus::AtReach && Sample.Out.ReachLimit == EReachLimit::LegTravel) ? 1 : 0;
				}
				TestEqual(Label(Kit, Rate, *FString::Printf(TEXT("%s: resting there is said: at reach, at a leg's travel, on every frame"), Case.Name)), NotSaid, 0);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: level.reach reads none left"), Case.Name)), Last.Out.ReachLeft <= 0.1);
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

// Catches: the pitch of a body being braked taken for ground. Letting go of
// the stick at speed pitches the base nose down for a few tenths of a second
// whatever the legs do; legs sent after that reading arrive as the base
// comes back by itself, and the knock is answered by a swing the other way.
// Measured against the same stop with the switch off, and against the law
// with the reading taken for the ground at once (which is how it was).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunStopFromSpeed,
	"Ramms.Level.DryRun.StopFromSpeed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunStopFromSpeed::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		for (const double GradeDeg : { 0.0, Kit.GradeDeg })
		{
			const FGround Course = Plane(GradeDeg, 0.0);
			for (int32 Rate = 0; Rate < 3; ++Rate)
			{
				// 60 cm/s, the stick let go 5 s in: the drive is down to a
				// third in a tenth of a second (6 m/s^2 at first), and the
				// body pitches 1.3 deg with it.
				FOptions Options = MakeOptions(Kit, Rate);
				Options.SpeedCmS = 60.0;
				Options.DriveLagSeconds = 0.1;
				Options.PitchPerAccelDeg = BodyPitchPerAccel;
				Options.MaxSeconds = 9.0;
				Options.bKeepTrace = true;
				Options.Events.Add(MakeEvent(5.0, EEvent::SetSpeed, 0.0, 0.0));

				FOptions OffOptions = Options;
				OffOptions.bEnabledAtStart = false;
				FOptions BeforeOptions = Options;
				BeforeOptions.Ground = NoGate();
				const FResult Off = DryRun::Run(Kit.Robot, Course, OffOptions);
				const FResult Before = DryRun::Run(Kit.Robot, Course, BeforeOptions);
				const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
				const FSwing  OffSwing = SwingBetween(Off, 5.0, 8.0, Options.FrameSeconds);
				const FSwing  BeforeSwing = SwingBetween(Before, 5.0, 8.0, Options.FrameSeconds);
				const FSwing  Swing = SwingBetween(Result, 5.0, 8.0, Options.FrameSeconds);
				const FString Case = FString::Printf(TEXT("stop from 60 cm/s on %.0f deg"), GradeDeg);
				Report(*this, Kit, *Case, Rate, Result);
				AddInfo(FString::Printf(TEXT("%s | %s | %s | swing %.2f deg at %.1f deg/s; switched off %.2f at %.1f; with the reading taken for the ground at once %.2f at %.1f"), Kit.Name, *Case, FrameNames[Rate],
					Swing.RangeDeg, Swing.RateDegPerSec, OffSwing.RangeDeg, OffSwing.RateDegPerSec, BeforeSwing.RangeDeg, BeforeSwing.RateDegPerSec));
				ExpectSound(*this, Kit, Rate, Result);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: (the stop knocks the base over a degree, switched off)"), *Case)), OffSwing.RangeDeg >= 1.0);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: (taken for the ground, the knock is answered by a swing the other way: %.2f against %.2f deg)"), *Case, BeforeSwing.RangeDeg, OffSwing.RangeDeg)),
					BeforeSwing.RangeDeg >= 1.3 * OffSwing.RangeDeg);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: the knock is left to pass (%.2f against %.2f deg switched off)"), *Case, Swing.RangeDeg, OffSwing.RangeDeg)), Swing.RangeDeg <= 1.2 * OffSwing.RangeDeg);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: ... and the base moves no faster for the legs (%.1f against %.1f deg/s)"), *Case, Swing.RateDegPerSec, OffSwing.RateDegPerSec)),
					Swing.RateDegPerSec <= 1.2 * OffSwing.RateDegPerSec);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: level before the stop"), *Case)), PeakTiltBetween(Result, Kit.Tuning, 4.0, 5.0) <= 0.5);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: level after it"), *Case)), Result.SteadyResidualDeg <= 0.3);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: no rocking after it"), *Case)), Result.SteadyTiltRangeDeg <= 0.2);
			}
		}
	}
	return true;
}

// Catches: a knock on a robot that stands, or crawls, followed as ground
// would be. (On a robot at cruising speed a knock cannot be told from ground
// by how far it has travelled: the ground could have turned as fast. That
// one is reported, and held to being no worse than it was.)
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunKnock,
	"Ramms.Level.DryRun.KnockStandingAndOnTheMove",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunKnock::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FGround Flat = Plane(0.0, 0.0);
	// The most the swing may be of the same knock with the switch off:
	// standing, crawling, cruising (not asked of it there).
	const double Speeds[3] = { 0.0, 10.0, 40.0 };
	const double Allowed[3] = { 1.2, 1.25, 0.0 };
	for (const FKit& Kit : Kits())
	{
		for (int32 SpeedIndex = 0; SpeedIndex < 3; ++SpeedIndex)
		{
			for (int32 Rate = 0; Rate < 3; ++Rate)
			{
				// 1.5 deg nose down, gone in a fraction of a second.
				FOptions Options = MakeOptions(Kit, Rate);
				Options.SpeedCmS = Speeds[SpeedIndex];
				Options.MaxSeconds = 8.0;
				Options.bKeepTrace = true;
				FDryEvent Kick = MakeEvent(3.0, EEvent::AttitudeKick, 0.15);
				Kick.PlaneDeg = FVector2D(-1.5, 0.0);
				Options.Events.Add(Kick);

				FOptions OffOptions = Options;
				OffOptions.bEnabledAtStart = false;
				FOptions BeforeOptions = Options;
				BeforeOptions.Ground = NoGate();
				const FSwing  Off = SwingBetween(DryRun::Run(Kit.Robot, Flat, OffOptions), 3.0, 6.0, Options.FrameSeconds);
				const FSwing  Before = SwingBetween(DryRun::Run(Kit.Robot, Flat, BeforeOptions), 3.0, 6.0, Options.FrameSeconds);
				const FResult Result = DryRun::Run(Kit.Robot, Flat, Options);
				const FSwing  Swing = SwingBetween(Result, 3.0, 6.0, Options.FrameSeconds);
				const FString Case = FString::Printf(TEXT("a 1.5 deg knock at %.0f cm/s"), Speeds[SpeedIndex]);
				AddInfo(FString::Printf(TEXT("%s | %s | %s | swing %.2f deg at %.1f deg/s; switched off %.2f at %.1f; with the reading taken for the ground at once %.2f at %.1f"), Kit.Name, *Case, FrameNames[Rate],
					Swing.RangeDeg, Swing.RateDegPerSec, Off.RangeDeg, Off.RateDegPerSec, Before.RangeDeg, Before.RateDegPerSec));
				ExpectSound(*this, Kit, Rate, Result);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: level after it"), *Case)), Result.SteadyResidualDeg <= 0.3);
				if (Allowed[SpeedIndex] > 0.0)
				{
					TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: (taken for the ground, it is answered by a swing the other way: %.2f against %.2f deg)"), *Case, Before.RangeDeg, Off.RangeDeg)),
						Before.RangeDeg >= 1.25 * Off.RangeDeg);
					TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: left to pass (%.2f against %.2f deg switched off)"), *Case, Swing.RangeDeg, Off.RangeDeg)), Swing.RangeDeg <= Allowed[SpeedIndex] * Off.RangeDeg);
				}
				else
				{
					TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("%s: no worse than with no gate (%.2f against %.2f deg)"), *Case, Swing.RangeDeg, Before.RangeDeg)), Swing.RangeDeg <= Before.RangeDeg + 0.05);
				}
			}
		}
	}
	return true;
}

// Catches: a body that pitches under the drive's own changes of speed
// spoiling a grade break at speed (the speed cap slows the drive through the
// break, the body pitches with that, and a law that reads the pitch for
// ground slows the drive further for it).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunAtSpeedBody,
	"Ramms.Level.DryRun.GradeBreakAtSpeedWithABodyThatPitches",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunAtSpeedBody::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const double Speeds[3] = { 40.0, 80.0, 120.0 };
	for (const FKit& Each : Kits())
	{
		const FKit	  Kit = FollowingOnly(Each);
		const FGround Course = RampUp(Kit.GradeDeg, 900.0);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			for (const double Speed : Speeds)
			{
				FOptions Options = MakeOptions(Kit, Rate);
				Options.SpeedCmS = Speed;
				Options.DistanceCm = 800.0;
				Options.PitchPerAccelDeg = BodyPitchPerAccel;
				Options.bKeepTrace = true;
				FOptions BeforeOptions = Options;
				BeforeOptions.Ground = NoGate();
				const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
				const FResult Before = DryRun::Run(Kit.Robot, Course, BeforeOptions);
				// (From a second in: the start's own pitch is the body's, not the break's.)
				const double Peak = PeakTiltBetween(Result, Kit.Tuning, 1.0, 1000.0);
				const double Was = PeakTiltBetween(Before, Kit.Tuning, 1.0, 1000.0);
				AddInfo(FString::Printf(TEXT("%s | grade break at %.0f cm/s, the body pitching with the drive | %s | peak %.2f deg, least speed scale %.2f; with the reading taken for the ground at once %.2f deg, %.2f"), Kit.Name,
					Speed, FrameNames[Rate], Peak, Result.MinSpeedScale, Was, Before.MinSpeedScale));
				ExpectSound(*this, Kit, Rate, Result);
				TestTrue(Label(Kit, Rate, TEXT("it got there")), Result.TravelCm >= 800.0);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("at %.0f cm/s the break is followed as well as with no gate (%.2f against %.2f deg)"), Speed, Peak, Was)), Peak <= Was + 0.2);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("at %.0f cm/s the drive is slowed hardly more for it (%.2f against %.2f)"), Speed, Result.MinSpeedScale, Before.MinSpeedScale)),
					Result.MinSpeedScale >= Before.MinSpeedScale - 0.1);
				TestTrue(Label(Kit, Rate, TEXT("level on the grade")), Result.SteadyResidualDeg <= 0.5);
			}
		}
	}
	return true;
}

// Catches: "at rest" declared, or kept, while a drive mode's stance ramp is
// still moving the baselines: nothing is written at rest, so the base goes
// where the ramp carries it (under the wake error) and stays there reading
// "Level, at rest". And a quick stance change under a standing robot taken
// for a knock: what the baselines turn the stance by is the stance's, and is
// followed.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunStanceRampAtRest,
	"Ramms.Level.DryRun.StanceRampUnderABaseAtRest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunStanceRampAtRest::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FGround Flat = Plane(0.0, 0.0);
	// A part of each kit's stance change that tilts its base by 0.38 deg:
	// between the settle error and the wake error.
	const double	   SmallShare[2] = { 0.2, 0.33 };
	const TArray<FKit> AllKits = Kits();
	for (int32 KitIndex = 0; KitIndex < AllKits.Num(); ++KitIndex)
	{
		const FKit&	 Kit = AllKits[KitIndex];
		const double SmallCm = SmallShare[FMath::Min(KitIndex, 1)] * Kit.RampCm;
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			{
				// Parked, level and at rest; then the ramp, over three seconds.
				FOptions Options = MakeOptions(Kit, Rate);
				Options.SpeedCmS = 0.0;
				Options.MaxSeconds = 12.0;
				Options.bKeepTrace = true;
				for (const int32 Leg : Kit.RampLegs)
				{
					Options.Events.Add(MakeEvent(4.0, EEvent::StanceRamp, 3.0, SmallCm, Leg));
				}
				FOptions OffOptions = Options;
				OffOptions.bEnabledAtStart = false;
				const FResult Off = DryRun::Run(Kit.Robot, Flat, OffOptions);
				const FResult Result = DryRun::Run(Kit.Robot, Flat, Options);
				Report(*this, Kit, TEXT("stance ramp under a base at rest"), Rate, Result);
				ExpectSound(*this, Kit, Rate, Result);
				bool  bRestBefore = false;
				int32 RestDuring = 0;
				int32 WrittenDuring = 0;
				for (const FSample& Sample : Result.Trace)
				{
					bRestBefore = bRestBefore || (Sample.Seconds > 3.5 && Sample.Seconds < 3.95 && Sample.Out.bQuiet);
					// (From half a second in: a baseline has moved once it is
					// BaselineStillCm from where it stood, which this slow a
					// ramp takes a quarter of a second to do.)
					if (Sample.Seconds > 4.5 && Sample.Seconds < 6.95)
					{
						RestDuring += Sample.Out.bQuiet || Sample.Out.Status == EStatus::Settled ? 1 : 0;
						WrittenDuring += Sample.Out.bWrite ? 1 : 0;
					}
				}
				AddInfo(FString::Printf(TEXT("%s | stance ramp under a base at rest | %s | switched off the ramp leaves the base %.2f deg off level; on, %.2f deg; frames at rest while it ran %d, written %d"), Kit.Name,
					FrameNames[Rate], Off.FinalTiltDeg, Result.FinalTiltDeg, RestDuring, WrittenDuring));
				TestTrue(Label(Kit, Rate, TEXT("at rest before the ramp")), bRestBefore);
				TestTrue(Label(Kit, Rate, TEXT("(the ramp alone tilts the base: past the settle error, and by less than would wake it)")),
					Off.FinalTiltDeg > Kit.Tuning.SettleErrorDeg + 0.03 && Off.FinalTiltDeg < Kit.Tuning.WakeErrorDeg - 0.05);
				TestEqual(Label(Kit, Rate, TEXT("no frame at rest while the baselines move")), RestDuring, 0);
				TestTrue(Label(Kit, Rate, TEXT("... and the offsets are written through it")), WrittenDuring > 0);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("the base is not left where the ramp carried it (%.2f deg; the ramp alone %.2f)"), Result.FinalTiltDeg, Off.FinalTiltDeg)), Result.FinalTiltDeg <= 0.2);
				TestEqual(Label(Kit, Rate, TEXT("at rest again once the ramp is done")), static_cast<int32>(Result.FinalStatus), static_cast<int32>(EStatus::Settled));
				TestEqual(Label(Kit, Rate, TEXT("... with nothing written")), Result.SteadyWrites, 0);
			}
			{
				// The ramp already under way when the base would settle: no
				// rest until it is done and the settle time has passed.
				FOptions Options = MakeOptions(Kit, Rate);
				Options.SpeedCmS = 0.0;
				Options.MaxSeconds = 9.0;
				Options.bKeepTrace = true;
				for (const int32 Leg : Kit.RampLegs)
				{
					Options.Events.Add(MakeEvent(0.5, EEvent::StanceRamp, 4.0, SmallCm, Leg));
				}
				const FResult Result = DryRun::Run(Kit.Robot, Flat, Options);
				double		  FirstRest = -1.0;
				for (const FSample& Sample : Result.Trace)
				{
					if (Sample.Seconds > 0.8 && Sample.Out.bQuiet && FirstRest < 0.0)
					{
						FirstRest = Sample.Seconds;
					}
				}
				AddInfo(FString::Printf(TEXT("%s | stance ramp from 0.5 to 4.5 s | %s | first at rest %.2f s in; ends %.2f deg off level"), Kit.Name, FrameNames[Rate], FirstRest, Result.FinalTiltDeg));
				// (The settle time from the last time a baseline was found
				// moved, which may be up to a third of a second before the
				// ramp's end.)
				TestTrue(Label(Kit, Rate, TEXT("no rest is declared while the stance is still being ramped in")), FirstRest >= 4.5 + Kit.Tuning.SettleSeconds - 0.5);
				TestTrue(Label(Kit, Rate, TEXT("... and the base ends level")), Result.FinalTiltDeg <= 0.2);
			}
			{
				// A quick stance change, standing: the kit's whole ramp in half
				// a second turns the stance at two to four deg/s, far over what
				// a standing robot's attitude is otherwise followed at.
				FOptions Options = MakeOptions(Kit, Rate);
				Options.SpeedCmS = 0.0;
				Options.MaxSeconds = 9.0;
				Options.bKeepTrace = true;
				for (const int32 Leg : Kit.RampLegs)
				{
					Options.Events.Add(MakeEvent(4.0, EEvent::StanceRamp, 0.5, Kit.RampCm, Leg));
				}
				FOptions OffOptions = Options;
				OffOptions.bEnabledAtStart = false;
				FOptions BeforeOptions = Options;
				BeforeOptions.Ground = NoGate();
				const FResult Off = DryRun::Run(Kit.Robot, Flat, OffOptions);
				const FResult Before = DryRun::Run(Kit.Robot, Flat, BeforeOptions);
				const FResult Result = DryRun::Run(Kit.Robot, Flat, Options);
				const double  Peak = PeakTiltBetween(Result, Kit.Tuning, 4.0, 100.0);
				const double  PeakBefore = PeakTiltBetween(Before, Kit.Tuning, 4.0, 100.0);
				// How far the law's ground estimate is behind the stance's own
				// turn half a second after the ramp ended.
				double BehindDeg = 0.0;
				for (const FSample& Sample : Result.Trace)
				{
					if (Sample.Seconds >= 4.95 && Sample.Seconds <= 5.05)
					{
						BehindDeg = FMath::Max(BehindDeg, (Sample.Out.GroundDeg - Result.Trace.Last().Out.GroundDeg).Size());
					}
				}
				AddInfo(FString::Printf(TEXT("%s | quick stance change standing | %s | switched off it leaves the base %.2f deg off level; on, the peak through it %.2f deg (%.2f with the reading taken for the ground at once), and the ground estimate %.2f deg behind half a second after"),
					Kit.Name, FrameNames[Rate], Off.FinalTiltDeg, Peak, PeakBefore, BehindDeg));
				ExpectSound(*this, Kit, Rate, Result);
				TestTrue(Label(Kit, Rate, TEXT("(the stance change alone tilts the base a degree or more)")), Off.FinalTiltDeg >= 1.0);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("the stance's own turn is followed as it is made (the estimate %.2f deg behind half a second after)"), BehindDeg)), BehindDeg <= 0.25);
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("level through it (%.2f deg at most)"), Peak)), Peak <= 0.6 * Off.FinalTiltDeg);
				// (The six-wheeler sits back onto the pair that is drawn up, a
				// rock of its own on top of the stance's turn: that part is
				// followed at the settling allowance, a quarter of a degree
				// behind for a moment.)
				TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("... nearly as well as with no gate (%.2f against %.2f deg)"), Peak, PeakBefore)), Peak <= PeakBefore + 0.3);
				TestTrue(Label(Kit, Rate, TEXT("level after it")), Result.FinalTiltDeg <= 0.3);
			}
		}
	}
	return true;
}

// Catches: a correction stopped by the clearance of wheels carried off the
// ground (the ground ahead rising towards them as the base is levelled nose
// down) with the status reading "Levelling" and level.reach reading most of
// the legs' travel left; and the status dropped a moment after the limit is
// met, with the base still held off level.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunRaisedClearance,
	"Ramms.Level.DryRun.RaisedWheelsClearanceIsSaid",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunRaisedClearance::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	// The front pair carried 12 cm clear and 0.3 cm short of its stop may be
	// asked 0.3 + (12 - 2.5) = 9.8 cm; it is 67.5 cm ahead of the middle of
	// the four that stand: 8.26 deg. The standing legs' own travel would
	// give more than twice that.
	const FKit	 Kit = RaisedFronts();
	const double ClearanceReachDeg = FMath::RadiansToDegrees(FMath::Atan(9.8 / 67.5));
	for (int32 Rate = 0; Rate < 3; ++Rate)
	{
		{
			// Parked nose up on 10 deg: 1.74 deg past that.
			FOptions Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 0.0;
			Options.bEnabledAtStart = false;
			Options.MaxSeconds = 16.0;
			Options.bKeepTrace = true;
			Options.Events.Add(MakeEvent(1.0, EEvent::Enable));
			const FResult Result = DryRun::Run(Kit.Robot, Plane(10.0, 0.0), Options);
			Report(*this, Kit, TEXT("10 deg nose up, past the raised wheels' clearance"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			if (!TestTrue(Label(Kit, Rate, TEXT("a trace")), Result.Trace.Num() > 0))
			{
				continue;
			}
			const FSample& Last = Result.Trace.Last();
			int32		   NotSaid = 0;
			int32		   Counted = 0;
			double		   FirstSaid = -1.0;
			for (const FSample& Sample : Result.Trace)
			{
				const bool bSaid = (Sample.Out.Status == EStatus::AtReach || Sample.Out.Status == EStatus::PastReach) && Sample.Out.ReachLimit == EReachLimit::LiftedClearance;
				if (bSaid && FirstSaid < 0.0)
				{
					FirstSaid = Sample.Seconds;
				}
				if (Sample.Seconds >= 8.0)
				{
					++Counted;
					NotSaid += bSaid ? 0 : 1;
				}
			}
			AddInfo(FString::Printf(TEXT("%s | 10 deg nose up, parked | %s | ends at %.2f deg (the clearance gives %.2f of the 10); first said %.2f s in; status %d, limit %d, level.reach %.2f, room %.2f cm, at rest %d, touching %d"),
				Kit.Name, FrameNames[Rate], Last.TrueAttitudeDeg.X, ClearanceReachDeg, FirstSaid, static_cast<int32>(Last.Out.Status), static_cast<int32>(Last.Out.ReachLimit), Last.Out.ReachLeft,
				Last.Out.MinMarginCm, Last.Out.bQuiet ? 1 : 0, Last.Touching));
			TestEqual(Label(Kit, Rate, TEXT("the base is held where the raised wheels' clearance leaves it")), Last.TrueAttitudeDeg.X, 10.0 - ClearanceReachDeg, 0.35);
			TestTrue(Label(Kit, Rate, TEXT("(no standing leg is near the end of its 12 cm of travel)")), Result.MaxOffsetCm < 6.0);
			TestEqual(Label(Kit, Rate, TEXT("said on every frame of the last eight seconds: at reach, a raised wheel's clearance")), NotSaid, 0);
			TestTrue(Label(Kit, Rate, TEXT("(eight seconds of frames)")), Counted > 100);
			TestTrue(Label(Kit, Rate, TEXT("level.reach reads none left")), Last.Out.ReachLeft <= 0.05);
			TestTrue(Label(Kit, Rate, TEXT("the room the speed cap goes by is used up")), Last.Out.MinMarginCm <= 0.3);
			TestTrue(Label(Kit, Rate, TEXT("at rest there all the same")), Last.Out.bQuiet);
			TestEqual(Label(Kit, Rate, TEXT("the raised wheels are still off the ground")), Last.Touching, 4);
		}
		{
			// 6 deg: inside it. Level, at rest, and level.reach says how much
			// of the clearance is left (6 of 8.26: a quarter and a bit).
			FOptions Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 0.0;
			Options.MaxSeconds = 12.0;
			Options.bKeepTrace = true;
			const FResult Result = DryRun::Run(Kit.Robot, Plane(6.0, 0.0), Options);
			if (!TestTrue(Label(Kit, Rate, TEXT("a trace")), Result.Trace.Num() > 0))
			{
				continue;
			}
			const FSample& Last = Result.Trace.Last();
			AddInfo(FString::Printf(TEXT("%s | 6 deg nose up, parked | %s | ends at %.2f deg; status %d, level.reach %.2f"), Kit.Name, FrameNames[Rate], Last.TrueAttitudeDeg.X, static_cast<int32>(Last.Out.Status),
				Last.Out.ReachLeft));
			TestTrue(Label(Kit, Rate, TEXT("6 deg: level")), Result.FinalTiltDeg <= 0.3);
			TestEqual(Label(Kit, Rate, TEXT("6 deg: at rest, and nothing to say")), static_cast<int32>(Last.Out.Status), static_cast<int32>(EStatus::Settled));
			TestEqual(Label(Kit, Rate, TEXT("6 deg: level.reach is what is left of the raised wheels' clearance")), Last.Out.ReachLeft, 1.0 - FMath::Tan(FMath::DegreesToRadians(6.0)) * 67.5 / 9.8, 0.06);
		}
	}
	return true;
}

namespace RammsLevelDryRunTests
{
	/** How far the centre pair comes down, and the front pair goes up, when
	 *  the two-stance robot changes from its corner stance to its centre one
	 *  (cm, as baselines move: positive retracts). */
	constexpr double CentreDownCm = -4.3;
	constexpr double FrontUpCm = 12.0;

	/**
	 * The quick-legged six-wheeler in one of two made-up drive stances, as a
	 * robot that changes the wheels it stands on with its drive mode has
	 * them. On its corners: the centre pair carried 4 cm clear. On its
	 * centre and rear legs: the front pair carried 12 cm clear, 0.3 cm short
	 * of its retract stop, and the centre pair 0.3 cm below the corners' line
	 * (so that the robot sits back onto centre and rear the moment the centre
	 * pair is down, a rock of 0.4 deg that is the stance's own). The same
	 * robot either way: only where it starts differs.
	 */
	FKit TwoStances(bool bOnCentres)
	{
		FKit Kit = QuickLegs();
		Kit.Name = bOnCentres ? TEXT("on centre and rear") : TEXT("on its corners");
		for (int32 Index = 0; Index < 2; ++Index)
		{
			Kit.Robot.Legs[Index].MaxZ = -17.7;
			Kit.Robot.Legs[Index].StanceZ = bOnCentres ? -30.0 + FrontUpCm : -30.0;
			Kit.Robot.Legs[Index + 2].StanceZ = bOnCentres ? -30.3 : -30.3 - CentreDownCm;
		}
		return Kit;
	}

	/** The slopes the hand-over cases are parked on (deg; pitch, roll). */
	struct FParkedSlope
	{
		const TCHAR* Name;
		FVector2D	 Deg;
	};
	const FParkedSlope ParkedSlopes[4] = { { TEXT("nose up"), FVector2D(5.0, 0.0) }, { TEXT("nose down"), FVector2D(-5.0, 0.0) }, { TEXT("cross"), FVector2D(0.0, 5.0) },
		{ TEXT("nose down and across"), FVector2D(-4.0, -3.0) } };

	/** The largest change of any leg's command from one frame to the next
	 *  between two times (cm). */
	double MostLegStepBetween(const FResult& Result, double FromSeconds, double ToSeconds)
	{
		double Most = 0.0;
		for (int32 Index = 1; Index < Result.Trace.Num(); ++Index)
		{
			const FSample& Now = Result.Trace[Index];
			if (Now.Seconds < FromSeconds || Now.Seconds > ToSeconds)
			{
				continue;
			}
			for (int32 Leg = 0; Leg < Now.CommandZ.Num(); ++Leg)
			{
				Most = FMath::Max(Most, FMath::Abs(Now.CommandZ[Leg] - Result.Trace[Index - 1].CommandZ[Leg]));
			}
		}
		return Most;
	}
} // namespace RammsLevelDryRunTests

// Catches: a hand-back after a take-over that starts levelling again from
// nothing. The taker leaves the base level on the slope in a pose of its own
// (here 3 cm lower than the stance has it); the drive mode's stance ramp then
// turns the base to the ground's attitude, and levelling turns it back: the
// rider is taken a degree and more off level for seconds. With the pose's
// plane taken back out of the baselines the ramp is left a ride height to
// change, and the base stays level through it.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunHandBack,
	"Ramms.Level.DryRun.HandBackToAStance",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunHandBack::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FKit Kit = TwoStances(/*bOnCentres=*/true);
	for (const FParkedSlope& Slope : ParkedSlopes)
	{
		const FGround Course = Plane(Slope.Deg.X, Slope.Deg.Y);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			// Parked and level. At 6 s a take-over: for four seconds the taker
			// has the legs, and draws the four that stand 3 cm up (the base
			// 3 cm down). At 10 s it hands back, and the drive mode's stance
			// ramp takes two seconds to put the stance back.
			FOptions Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 0.0;
			Options.MaxSeconds = 20.0;
			Options.bKeepTrace = true;
			FDryEvent Taken = MakeEvent(6.0, EEvent::FoldAdopt, 4.0);
			Taken.RampSeconds = 2.0;
			Options.Events.Add(Taken);
			for (int32 Leg = 2; Leg < 6; ++Leg)
			{
				Options.Events.Add(MakeEvent(6.5, EEvent::StanceRamp, 1.5, 3.0, Leg));
			}
			FOptions ColdOptions = Options;
			ColdOptions.Events[0].bCold = true;
			const FResult Cold = DryRun::Run(Kit.Robot, Course, ColdOptions);
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, Slope.Name, Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			if (!TestTrue(Label(Kit, Rate, TEXT("traces")), Result.Trace.Num() > 0 && Cold.Trace.Num() > 0))
			{
				continue;
			}
			const auto Say = [&](const TCHAR* What) { return FString::Printf(TEXT("%s, %s, %s: %s"), Kit.Name, Slope.Name, FrameNames[Rate], What); };

			const double Before = PeakTiltBetween(Result, Kit.Tuning, 5.0, 9.9);
			const double Peak = PeakTiltBetween(Result, Kit.Tuning, 10.0, 20.0);
			const double ColdPeak = PeakTiltBetween(Cold, Kit.Tuning, 10.0, 20.0);
			const FSwing Swing = SwingBetween(Result, 10.0, 20.0, Options.FrameSeconds);
			const FSwing ColdSwing = SwingBetween(Cold, 10.0, 20.0, Options.FrameSeconds);
			const double Step = MostLegStepBetween(Result, 10.0 - 0.5 * Options.FrameSeconds, 10.0 + 1.5 * Options.FrameSeconds);
			// The plane the legs carried before the take-over, the one the
			// hand-back gave the law, and the pose it was handed.
			FVector2D CarriedDeg = FVector2D::ZeroVector;
			FVector2D AdoptedDeg = FVector2D::ZeroVector;
			bool	  bAdopted = false;
			double	  LowestCm = 0.0;
			for (const FSample& Sample : Result.Trace)
			{
				if (Sample.Seconds < 6.0 - 0.5 * Options.FrameSeconds)
				{
					CarriedDeg = Sample.Out.AppliedDeg;
				}
				if (Sample.Out.Action == EAction::Adopt)
				{
					AdoptedDeg = Sample.Out.AppliedDeg;
					bAdopted = true;
					LowestCm = Sample.CommandZ[4] - Kit.Robot.Legs[4].StanceZ + Sample.CommandZ[5] - Kit.Robot.Legs[5].StanceZ + Sample.CommandZ[2] - Kit.Robot.Legs[2].StanceZ + Sample.CommandZ[3]
						- Kit.Robot.Legs[3].StanceZ;
					LowestCm /= 4.0;
				}
			}
			AddInfo(FString::Printf(TEXT("%s | hand-back onto %s | %s | level to %.2f deg before; through the hand-back %.2f deg at most, %.2f deg/s (starting again from nothing: %.2f deg, %.2f deg/s); plane adopted (%.2f, %.2f); largest leg step at the hand-back %.3f cm"),
				Kit.Name, Slope.Name, FrameNames[Rate], Before, Peak, Swing.RateDegPerSec, ColdPeak, ColdSwing.RateDegPerSec, AdoptedDeg.X, AdoptedDeg.Y, Step));
			TestTrue(Say(TEXT("level before the hand-back, in the taker's pose")), Before <= 0.3);
			TestTrue(Say(TEXT("(the pose handed back has the standing legs centimetres off the stance)")), bAdopted && LowestCm >= 2.5);
			// (The slope's, and the stance's own lean over it: the centre pair
			// stands 0.3 cm below the rear pair's line.)
			TestTrue(Say(TEXT("the plane adopted is the one the legs carried before the take-over")), (AdoptedDeg - CarriedDeg).Size() <= 0.1 && (CarriedDeg - Slope.Deg).Size() <= 0.6);
			TestTrue(Say(*FString::Printf(TEXT("the base stays level through the hand-back and the stance ramp (%.2f deg)"), Peak)), Peak <= 0.5);
			TestTrue(Say(TEXT("... and is not turned faster than the tilt policy")), Swing.RateDegPerSec <= 3.5);
			// (The stance ramp begins on the same frame: 3 cm in 2 s is what a
			// standing leg's command may move by.)
			TestTrue(Say(*FString::Printf(TEXT("no leg is stepped by the hand-back (%.3f cm)"), Step)), Step <= 1.5 * Options.FrameSeconds + 0.02);
			// Starting again from nothing: a degree or more wherever the
			// stance ramp turns the nose up or the +Y side up (the raised pair
			// then meets its stop); the easy way it is a few tenths, and no
			// case to tell the two hand-backs apart by.
			if (Slope.Deg.X > 0.0 || Slope.Deg.Y > 0.0)
			{
				TestTrue(Say(*FString::Printf(TEXT("(starting again from nothing takes the base %.2f deg off level)"), ColdPeak)), ColdPeak >= 1.0);
			}
			// The stance ramp did its part: the baselines are the stance's again.
			const FSample& Last = Result.Trace.Last();
			double		   OffStance = 0.0;
			for (int32 Leg = 0; Leg < Kit.Robot.Legs.Num(); ++Leg)
			{
				OffStance = FMath::Max(OffStance, FMath::Abs(Last.BaselineZ[Leg] - Kit.Robot.Legs[Leg].StanceZ));
			}
			TestTrue(Say(TEXT("the stance ramp ends on the stance")), OffStance <= 1.0e-6);
			TestTrue(Say(TEXT("level and at rest at the end")), Result.FinalTiltDeg <= 0.3 && Result.FinalStatus == EStatus::Settled);
		}
	}
	return true;
}

// Catches: the wanted attitude walked on top of a drive mode's stance change.
// Switching, parked on a slope, into the mode it levels in: the centre pair
// comes down and the robot sits back onto it (the stance's own rock) while
// the levelling that began with the switch is turning the base the other
// way; the base stalls, and catches up faster than the tilt policy allows.
// And out of it: the base is given back to the ground while the stance
// changes under it. Either way the stance changes first, or the walk waits.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunModeSwitch,
	"Ramms.Level.DryRun.ModeSwitchOnASlope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunModeSwitch::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FParkedSlope& Slope : ParkedSlopes)
	{
		const FGround Course = Plane(Slope.Deg.X, Slope.Deg.Y);
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			const auto Say = [&](const TCHAR* Which, const TCHAR* What) { return FString::Printf(TEXT("%s, %s, %s: %s"), Which, Slope.Name, FrameNames[Rate], What); };
			{
				// Into the mode it levels in, 3 s in: the centre pair down in
				// the first second, the front pair up in the next.
				const FKit Kit = TwoStances(/*bOnCentres=*/false);
				FOptions   Options = MakeOptions(Kit, Rate);
				Options.SpeedCmS = 0.0;
				Options.MaxSeconds = 14.0;
				Options.bKeepTrace = true;
				Options.Events.Add(MakeEvent(0.0, EEvent::ModeRelease, 3.0));
				Options.Events.Add(MakeEvent(3.0, EEvent::StanceChanging, 2.0));
				for (int32 Side = 0; Side < 2; ++Side)
				{
					Options.Events.Add(MakeEvent(3.0, EEvent::StanceRamp, 1.0, CentreDownCm, 2 + Side));
					Options.Events.Add(MakeEvent(4.0, EEvent::StanceRamp, 1.0, FrontUpCm, Side));
				}
				FOptions OffOptions = Options;
				OffOptions.bEnabledAtStart = false;
				const FResult Off = DryRun::Run(Kit.Robot, Course, OffOptions);
				const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
				Report(*this, Kit, TEXT("switch into the levelled mode"), Rate, Result);
				ExpectSound(*this, Kit, Rate, Result);
				const FSwing Swing = SwingBetween(Result, 3.0, 14.0, Options.FrameSeconds);
				const FSwing OffSwing = SwingBetween(Off, 3.0, 14.0, Options.FrameSeconds);
				const double Started = FirstTimeIn(Result, EState::Active, 0.0);
				AddInfo(FString::Printf(TEXT("into the levelled mode | %s | %s | base turned at %.2f deg/s at most over 0.3 s (the stance change alone, switched off: %.2f); levelling began %.2f s in; ends %.2f deg off level"),
					Slope.Name, FrameNames[Rate], Swing.RateDegPerSec, OffSwing.RateDegPerSec, Started, Result.FinalTiltDeg));
				TestTrue(Say(TEXT("in"), TEXT("(the stance change rocks the base by itself)")), OffSwing.RateDegPerSec >= 0.5 && OffSwing.RateDegPerSec <= 2.5);
				TestTrue(Say(TEXT("in"), *FString::Printf(TEXT("the base is turned within the tilt policy (%.2f deg/s)"), Swing.RateDegPerSec)), Swing.RateDegPerSec <= 3.5);
				TestTrue(Say(TEXT("in"), TEXT("levelling begins once the stance has arrived, and no later")), Started >= 5.0 - 1.0e-6 && Started <= 5.0 + 2.0 * Options.FrameSeconds);
				TestTrue(Say(TEXT("in"), TEXT("level and at rest at the end")), Result.FinalTiltDeg <= 0.3 && Result.FinalStatus == EStatus::Settled);
				// Not left on the edge of the loop's deadband by what the walk
				// wound into the integral (0.10 to 0.15 deg at 16 fps).
				TestTrue(Say(TEXT("in"), *FString::Printf(TEXT("the rest is on level, not on the deadband's edge (%.3f deg)"), Result.FinalTiltDeg)), Result.FinalTiltDeg <= 0.08);
				TestEqual(Say(TEXT("in"), TEXT("on centre and rear")), Result.Trace.Num() > 0 ? Result.Trace.Last().Touching : 0, 4);
			}
			{
				// Out of it, 8 s in and level: the front pair down in the first
				// second, the centre pair up in the next.
				const FKit Kit = TwoStances(/*bOnCentres=*/true);
				FOptions   Options = MakeOptions(Kit, Rate);
				Options.SpeedCmS = 0.0;
				Options.MaxSeconds = 18.0;
				Options.bKeepTrace = true;
				Options.Events.Add(MakeEvent(8.0, EEvent::ModeRelease, 100.0));
				Options.Events.Add(MakeEvent(8.0, EEvent::StanceChanging, 2.0));
				for (int32 Side = 0; Side < 2; ++Side)
				{
					Options.Events.Add(MakeEvent(8.0, EEvent::StanceRamp, 1.0, -FrontUpCm, Side));
					Options.Events.Add(MakeEvent(9.0, EEvent::StanceRamp, 1.0, -CentreDownCm, 2 + Side));
				}
				FOptions OffOptions = Options;
				OffOptions.bEnabledAtStart = false;
				const FResult Off = DryRun::Run(Kit.Robot, Course, OffOptions);
				const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
				Report(*this, Kit, TEXT("switch out of the levelled mode"), Rate, Result);
				ExpectSound(*this, Kit, Rate, Result);
				const FSwing Swing = SwingBetween(Result, 8.0, 18.0, Options.FrameSeconds);
				const FSwing OffSwing = SwingBetween(Off, 8.0, 18.0, Options.FrameSeconds);
				const double During = PeakTiltBetween(Result, Kit.Tuning, 8.0, 10.0);
				AddInfo(FString::Printf(TEXT("out of the levelled mode | %s | %s | base turned at %.2f deg/s at most over 0.3 s (the stance change alone, switched off: %.2f); %.2f deg off level at most while the stance changed; given back %.2f s after the switch"),
					Slope.Name, FrameNames[Rate], Swing.RateDegPerSec, OffSwing.RateDegPerSec, During, FirstTimeIn(Result, EState::Suspended, 8.0) - 8.0));
				TestTrue(Say(TEXT("out"), TEXT("level before the switch")), PeakTiltBetween(Result, Kit.Tuning, 7.0, 7.9) <= 0.3);
				TestTrue(Say(TEXT("out"), *FString::Printf(TEXT("the base is turned within the tilt policy (%.2f deg/s)"), Swing.RateDegPerSec)), Swing.RateDegPerSec <= 3.5);
				TestTrue(Say(TEXT("out"), *FString::Printf(TEXT("the base is held level while the stance changes (%.2f deg)"), During)), During <= 0.6);
				TestEqual(Say(TEXT("out"), TEXT("waiting at the end")), static_cast<int32>(Result.FinalState), static_cast<int32>(EState::Suspended));
				TestTrue(Say(TEXT("out"), TEXT("... every offset exactly zero, on the ground's attitude")), Result.FinalMaxOffsetCm == 0.0 && FMath::Abs(Result.FinalTiltDeg - Off.FinalTiltDeg) <= 0.02);
				TestEqual(Say(TEXT("out"), TEXT("on its corners")), Result.Trace.Num() > 0 ? Result.Trace.Last().Touching : 0, 4);
			}
		}
	}
	return true;
}

namespace RammsLevelDryRunTests
{
	/** The rider's tilt-rate limit the courses are scored against (deg/s, the
	 *  base's attitude over 0.3 s). */
	constexpr double RiderRateLimitDegPerSec = 4.0;
	/** ... and what a slope's end taken at a robot's full speed, with nothing
	 *  slowed for it beforehand, may turn the rider at (deg/s, over 0.3 s). */
	constexpr double SlopeEndRateDegPerSec = 5.5;

	/**
	 * The six-wheeled robot on its centre and rear legs (RaisedFronts) as a
	 * robot with a top speed has it: the legs it stands on quick, so that the
	 * correction's own rate (Tuning.MaxCorrectionRateDegPerSec) is what binds
	 * them; the raised front pair slow to change height (near the top of its
	 * swing a leg's wheel hardly moves with its joint), and three times as far
	 * from the middle of the standing legs as they are.
	 */
	FKit QuickOnFour()
	{
		FKit Kit = RaisedFronts();
		Kit.Name = TEXT("quick on four");
		for (int32 Index = 0; Index < Kit.Robot.Legs.Num(); ++Index)
		{
			Kit.Robot.Legs[Index].MaxRateCmS = Index < 2 ? 10.0 : 60.0;
			Kit.Robot.Legs[Index].MaxAccelCmS2 = 600.0;
		}
		return Kit;
	}

	/** How far apart the legs that robot stands on are, fore and aft (cm). */
	constexpr double OnFourSpanCm = 45.0;

	/** The rider through a stretch of a course, from a trace. */
	struct FPassage
	{
		/** The most the base was off the wanted attitude (deg). */
		double PeakDeg = 0.0;
		/** The fastest the base was turned over 0.3 s (deg/s). */
		double RateDegPerSec = 0.0;
		/** The least speed scale, and the mean speed (cm/s). */
		double LeastScale = 1.0;
		double MeanSpeedCmS = 0.0;
		/** The most reach left and the least (FOutputs::ReachLeft). */
		double LeastReachLeft = 1.0;
		int32  Frames = 0;
	};

	/** The frames whose travel is from FromCm to ToCm. */
	FPassage PassageBetween(const FResult& Result, const FTuning& Tuning, double FromCm, double ToCm, double TickSeconds)
	{
		FPassage	Out;
		double		SpeedSum = 0.0;
		const int32 Window = FMath::Max(1, FMath::RoundToInt32(0.3 / TickSeconds));
		for (int32 Index = 0; Index < Result.Trace.Num(); ++Index)
		{
			const FSample& Sample = Result.Trace[Index];
			if (Sample.TravelCm < FromCm || Sample.TravelCm > ToCm)
			{
				continue;
			}
			Out.PeakDeg = FMath::Max(Out.PeakDeg, Inclination(Sample.TrueAttitudeDeg - Tuning.LevelReferenceDeg));
			if (Index >= Window)
			{
				Out.RateDegPerSec = FMath::Max(Out.RateDegPerSec, (Sample.TrueAttitudeDeg - Result.Trace[Index - Window].TrueAttitudeDeg).Size() / (Window * TickSeconds));
			}
			Out.LeastScale = FMath::Min(Out.LeastScale, Sample.Out.SpeedScale);
			Out.LeastReachLeft = FMath::Min(Out.LeastReachLeft, Sample.Out.ReachLeft);
			SpeedSum += Sample.SpeedCmS;
			++Out.Frames;
		}
		Out.MeanSpeedCmS = Out.Frames > 0 ? SpeedSum / Out.Frames : 0.0;
		return Out;
	}

	/** A place on a course where the ground turns under the robot. */
	struct FSeam
	{
		const TCHAR* Name;
		/** Where the leading standing wheels meet it (cm of travel), and how
		 *  long the turn itself is (cm; 0: a sharp break). */
		double AtCm;
		double LengthCm;
		/** The rider is to be kept inside the tilt-rate limit through it. */
		bool bWithinRate;
	};

	struct FReachCourse
	{
		const TCHAR*  Name;
		FGround		  Ground;
		TArray<FSeam> Seams;
		double		  DistanceCm;
		/** A stretch of steady slope (cm of travel, from and to; none: 0, 0). */
		FVector2D SteadyCm = FVector2D::ZeroVector;
	};

	TArray<FReachCourse> ReachCourses()
	{
		TArray<FReachCourse> Out;
		{
			// 6 deg up, 6 deg down: the ground turns 12 deg under the robot at the top.
			FReachCourse&	Course = Out.AddDefaulted_GetRef();
			const FVector2D Knots[3] = { { 150.0, 0.0 }, { 650.0, Rise(500.0, 6.0) }, { 1150.0, 0.0 } };
			Course.Name = TEXT("12 deg crest");
			Course.Ground = Profile(MakeArrayView(Knots));
			Course.Seams = { { TEXT("onto the grade"), 150.0, 0.0, false }, { TEXT("the crest"), 650.0, 0.0, false }, { TEXT("off the grade"), 1150.0, 0.0, true } };
			Course.DistanceCm = 1450.0;
		}
		{
			// 6 deg up to a landing.
			FReachCourse&	Course = Out.AddDefaulted_GetRef();
			const FVector2D Knots[2] = { { 150.0, 0.0 }, { 650.0, Rise(500.0, 6.0) } };
			Course.Name = TEXT("6 deg break");
			Course.Ground = Profile(MakeArrayView(Knots));
			Course.Seams = { { TEXT("onto the grade"), 150.0, 0.0, false }, { TEXT("onto the landing"), 650.0, 0.0, true } };
			Course.DistanceCm = 950.0;
		}
		{
			// A cross fall of 6 deg that changes sides within 40 cm.
			FReachCourse& Course = Out.AddDefaulted_GetRef();
			Course.Name = TEXT("warp");
			Course.Ground = Warp(6.0, 400.0, 200.0);
			Course.Seams = { { TEXT("the twist"), 400.0, 200.0, true } };
			Course.DistanceCm = 900.0;
		}
		{
			// 8 deg up to a landing: nose up, the raised wheels' clearance
			// gives this robot 8.26 deg.
			FReachCourse&	Course = Out.AddDefaulted_GetRef();
			const FVector2D Knots[2] = { { 150.0, 0.0 }, { 750.0, Rise(600.0, 8.0) } };
			Course.Name = TEXT("8 deg ramp at the end of reach");
			Course.Ground = Profile(MakeArrayView(Knots));
			Course.Seams = { { TEXT("onto the ramp"), 150.0, 0.0, false }, { TEXT("onto the landing"), 750.0, 0.0, true } };
			Course.DistanceCm = 1050.0;
			Course.SteadyCm = FVector2D(450.0, 740.0);
		}
		return Out;
	}
} // namespace RammsLevelDryRunTests

// Catches: a drive-speed cap that does not earn its keep, or that costs what
// it should not. With the cap applied to the drive (the dry run's plant takes
// the law's speed scale), with a body that pitches under its own changes of
// speed. At the robot's full speed: the rider through every seam no worse
// than with the switch off, a slope's end taken within a bound, and a steady
// slope -- at the very end of the legs' reach too -- driven at the speed the
// stick asks: it costs nothing. At twice that speed (no robot's own; kept for
// what it shows) the seams are reported and the same steady slope still costs
// nothing; nothing is promised of a crest there now that the drive is no
// longer slowed all the way up a slope for the slope's end.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunSpeedCap,
	"Ramms.Level.DryRun.SpeedCapThroughCrestBreakWarpAndRamp",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunSpeedCap::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FKit				   Kit = FollowingOnly(QuickOnFour());
	const TArray<FReachCourse> Courses = ReachCourses();
	const int32				   Rates[2] = { 0, 2 };
	for (const FReachCourse& Course : Courses)
	{
		for (const double Speed : { 60.0, 120.0 })
		{
			for (const int32 Rate : Rates)
			{
				// Levelled where it stands for five seconds, then driven.
				FOptions Options = MakeOptions(Kit, Rate);
				Options.SpeedCmS = 0.0;
				Options.DistanceCm = Course.DistanceCm;
				Options.MaxSeconds = 90.0;
				Options.PitchPerAccelDeg = BodyPitchPerAccel;
				Options.bKeepTrace = true;
				Options.Events.Add(MakeEvent(5.0, EEvent::SetSpeed, 0.0, Speed));
				FOptions OffOptions = Options;
				OffOptions.bEnabledAtStart = false;
				const FResult Off = DryRun::Run(Kit.Robot, Course.Ground, OffOptions);
				const FResult Result = DryRun::Run(Kit.Robot, Course.Ground, Options);
				const FString Case = FString::Printf(TEXT("%s at %.0f cm/s"), Course.Name, Speed);
				Report(*this, Kit, *Case, Rate, Result);
				ExpectSound(*this, Kit, Rate, Result);
				const auto Say = [&](const TCHAR* Where, const FString& What) { return FString::Printf(TEXT("%s, %s, %s: %s"), *Case, Where, FrameNames[Rate], *What); };
				TestTrue(Say(TEXT("the course"), TEXT("driven to its end")), Result.TravelCm >= Course.DistanceCm && Off.TravelCm >= Course.DistanceCm);

				// What it cost: the time from the stick to the end, against the
				// same switched off.
				const double Kept = Result.Seconds > 5.0 ? (Off.Seconds - 5.0) / (Result.Seconds - 5.0) : 0.0;
				AddInfo(FString::Printf(TEXT("%s | %s | speed kept %.2f (%.1f s against %.1f switched off), least speed scale %.2f, least speed %.1f cm/s"), *Case, FrameNames[Rate], Kept, Result.Seconds - 5.0,
					Off.Seconds - 5.0, Result.MinSpeedScale, Result.MinSpeedCmS));
				TestTrue(Say(TEXT("the course"), FString::Printf(TEXT("never slowed to a stop (%.1f cm/s at least)"), Result.MinSpeedCmS)), Result.MinSpeedCmS >= 0.2 * Speed * Kit.Tuning.MinSpeedScale);
				const bool bOwnSpeed = Speed <= 60.0;
				// The raised pair is let down behind the plane where it is slow,
				// never left low: it stays off the ground all the way.
				int32 MostTouching = 0;
				for (const FSample& Sample : Result.Trace)
				{
					MostTouching = FMath::Max(MostTouching, Sample.Touching);
				}
				TestEqual(Say(TEXT("the course"), TEXT("the raised wheels never touch the ground")), MostTouching, 4);

				for (const FSeam& Seam : Course.Seams)
				{
					// From just before the leading wheels meet it until the legs
					// have had a second and more to catch it up.
					const double   From = Seam.AtCm - 20.0;
					const double   To = Seam.AtCm + Seam.LengthCm + OnFourSpanCm + 100.0;
					const FPassage On = PassageBetween(Result, Kit.Tuning, From, To, Options.FrameSeconds);
					const FPassage Was = PassageBetween(Off, Kit.Tuning, From, To, Options.FrameSeconds);
					AddInfo(FString::Printf(TEXT("%s | %s | %s | rider %.2f deg at %.1f deg/s; switched off %.2f at %.1f; through it at %.1f cm/s (off %.1f), least speed scale %.2f"), *Case, Seam.Name, FrameNames[Rate],
						On.PeakDeg, On.RateDegPerSec, Was.PeakDeg, Was.RateDegPerSec, On.MeanSpeedCmS, Was.MeanSpeedCmS, On.LeastScale));
					if (!bOwnSpeed)
					{
						continue;
					}
					TestTrue(Say(Seam.Name, FString::Printf(TEXT("the rider no further off than switched off (%.2f against %.2f deg)"), On.PeakDeg, Was.PeakDeg)), On.PeakDeg <= Was.PeakDeg + 0.1);
					TestTrue(Say(Seam.Name, FString::Printf(TEXT("the rider turned no faster than switched off (%.1f against %.1f deg/s)"), On.RateDegPerSec, Was.RateDegPerSec)),
						On.RateDegPerSec <= Was.RateDegPerSec + 0.1);
					if (Seam.bWithinRate)
					{
						// A slope's end at full speed, nothing slowed for it beforehand: met as any turn of the ground.
						TestTrue(Say(Seam.Name, FString::Printf(TEXT("the rider turned gently through a slope's end (%.1f deg/s)"), On.RateDegPerSec)), On.RateDegPerSec <= SlopeEndRateDegPerSec);
					}
				}

				if (Course.SteadyCm.Y > Course.SteadyCm.X)
				{
					// A steady slope at the end of reach: the rider level, and
					// the drive not held on the cap's floor for standing at a limit.
					const FPassage Steady = PassageBetween(Result, Kit.Tuning, Course.SteadyCm.X, Course.SteadyCm.Y, Options.FrameSeconds);
					AddInfo(FString::Printf(TEXT("%s | the steady slope | %s | rider %.2f deg, reach left %.2f, speed scale %.2f at least, %.1f cm/s"), *Case, FrameNames[Rate], Steady.PeakDeg, Steady.LeastReachLeft,
						Steady.LeastScale, Steady.MeanSpeedCmS));
					TestTrue(Say(TEXT("the steady slope"), TEXT("at the end of the legs' reach")), Steady.LeastReachLeft <= 0.1);
					TestTrue(Say(TEXT("the steady slope"), FString::Printf(TEXT("the rider level (%.2f deg)"), Steady.PeakDeg)), Steady.PeakDeg <= 0.5);
					// Driven at the speed the stick asks: a steady slope costs
					// nothing, with the reach used up or not.
					TestTrue(Say(TEXT("the steady slope"), FString::Printf(TEXT("driven at the speed the stick asks (%.1f cm/s)"), Steady.MeanSpeedCmS)), Steady.MeanSpeedCmS >= 0.98 * Speed);
					TestTrue(Say(TEXT("the steady slope"), FString::Printf(TEXT("nothing asked of the drive (%.2f)"), Steady.LeastScale)), Steady.LeastScale >= 0.99);
				}
			}
		}
	}

	// Standing on a slope near the end of reach, the switch thrown: nothing
	// is turning under the robot, and the drive is not slowed.
	for (const int32 Rate : Rates)
	{
		FOptions Options = MakeOptions(Kit, Rate);
		Options.SpeedCmS = 0.0;
		Options.bEnabledAtStart = false;
		Options.MaxSeconds = 10.0;
		Options.Events.Add(MakeEvent(1.0, EEvent::Enable));
		const FResult Result = DryRun::Run(Kit.Robot, Plane(7.0, 0.0), Options);
		Report(*this, Kit, TEXT("switched on standing on 7 deg"), Rate, Result);
		ExpectSound(*this, Kit, Rate, Result);
		TestTrue(Label(Kit, Rate, TEXT("switched on standing on 7 deg: level")), Result.FinalTiltDeg <= 0.3);
		TestTrue(Label(Kit, Rate, TEXT("... near the end of reach")), Result.MinReachLeft <= 0.2);
		TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("... and the drive never slowed for it (%.2f)"), Result.MinSpeedScale)), Result.MinSpeedScale >= 0.99);
	}
	return true;
}

// Catches: a ride-height allowance that buys no reach (the wanted tilt planned
// without it, or the shift never asked for), one that is taken in a step, one
// that is kept once the slope no longer needs it, and one that sets the wheels
// it was meant to keep clear on the ground.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunRideHeight,
	"Ramms.Level.DryRun.RideHeightBuysReachPastTheRaisedWheels",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunRideHeight::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	// As RaisedWheelsClearanceIsSaid: nose up, the raised front pair may be
	// asked 9.8 cm, 67.5 cm ahead of the middle of the four that stand: 8.26
	// deg. Every cm the chassis is raised is a cm more for that pair: with
	// 3 cm allowed, 12.8 cm, 10.74 deg. On 10 deg the plane asks the pair
	// 11.9 cm, so 2.1 cm of the allowance is used.
	const double NeededCm = 67.5 * FMath::Tan(FMath::DegreesToRadians(10.0)) - 9.8;
	for (const int32 Rate : { 0, 2 })
	{
		FKit Kit = QuickOnFour();
		Kit.Tuning.RideHeightUpCm = 3.0;
		{
			// Parked on 10 deg, the switch thrown.
			FOptions Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 0.0;
			Options.bEnabledAtStart = false;
			Options.MaxSeconds = 16.0;
			Options.bKeepTrace = true;
			Options.Events.Add(MakeEvent(1.0, EEvent::Enable));
			FOptions NoneOptions = Options;
			NoneOptions.Tuning.RideHeightUpCm = 0.0;
			const FResult None = DryRun::Run(Kit.Robot, Plane(10.0, 0.0), NoneOptions);
			const FResult Result = DryRun::Run(Kit.Robot, Plane(10.0, 0.0), Options);
			Report(*this, Kit, TEXT("10 deg nose up, 3 cm of ride height allowed"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			if (!TestTrue(Label(Kit, Rate, TEXT("a trace")), Result.Trace.Num() > 1))
			{
				continue;
			}
			double MostRideStep = 0.0;
			int32  MostTouching = 0;
			for (int32 Index = 1; Index < Result.Trace.Num(); ++Index)
			{
				MostRideStep = FMath::Max(MostRideStep, FMath::Abs(Result.Trace[Index].Out.RideCm - Result.Trace[Index - 1].Out.RideCm));
				MostTouching = FMath::Max(MostTouching, Result.Trace[Index].Touching);
			}
			const FSample& Last = Result.Trace.Last();
			AddInfo(FString::Printf(TEXT("%s | 10 deg nose up, parked | %s | with no allowance %.2f deg off level; with 3 cm %.2f, the chassis raised %.2f cm (%.2f needed), %.3f cm a frame at most, status %d, level.reach %.2f"),
				Kit.Name, FrameNames[Rate], None.FinalTiltDeg, Result.FinalTiltDeg, Last.Out.RideCm, NeededCm, MostRideStep, static_cast<int32>(Last.Out.Status), Last.Out.ReachLeft));
			TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("(with no allowance the base is left %.2f deg off)"), None.FinalTiltDeg)), None.FinalTiltDeg >= 1.4);
			TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("with 3 cm allowed: level (%.2f deg)"), Result.FinalTiltDeg)), Result.FinalTiltDeg <= 0.3);
			TestEqual(Label(Kit, Rate, TEXT("... the chassis raised by what the raised wheels needed, and no more")), Last.Out.RideCm, NeededCm, 0.25);
			TestTrue(Label(Kit, Rate, TEXT("... at the ride height's rate, never in a step")), MostRideStep <= Kit.Tuning.RideHeightRateCmPerSec * Options.FrameSeconds + 1.0e-6);
			TestTrue(Label(Kit, Rate, TEXT("... inside reach: nothing to say, and at rest")), Last.Out.Status == EStatus::Settled);
			// (11.9 of the 12.8 cm the raised wheels may now be asked.)
			TestEqual(Label(Kit, Rate, TEXT("... level.reach reads the ride height still free")), Last.Out.ReachLeft, 1.0 - (NeededCm + 9.8) / 12.8, 0.02);
			TestEqual(Label(Kit, Rate, TEXT("... the raised wheels off the ground all the way")), MostTouching, 4);
			// The legs that stand were all let out by it: the base is higher.
			TestTrue(Label(Kit, Rate, TEXT("... the standing legs let out by it")), (Last.CommandZ[2] + Last.CommandZ[4]) * 0.5 - (-30.0) <= -NeededCm + 0.3);
		}
		{
			// Driven up a 10 deg ramp at a walk and onto its landing: level on
			// the ramp, and the ride height given back on the flat.
			const FVector2D Knots[2] = { { 150.0, 0.0 }, { 650.0, Rise(500.0, 10.0) } };
			const FGround	Course = Profile(MakeArrayView(Knots));
			FOptions		Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 30.0;
			Options.DistanceCm = 1000.0;
			Options.MaxSeconds = 90.0;
			Options.bKeepTrace = true;
			const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
			Report(*this, Kit, TEXT("a 10 deg ramp, 3 cm of ride height allowed"), Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			const FPassage OnRamp = PassageBetween(Result, Kit.Tuning, 400.0, 640.0, Options.FrameSeconds);
			double		   MostRide = 0.0;
			int32		   MostTouching = 0;
			for (const FSample& Sample : Result.Trace)
			{
				MostRide = FMath::Max(MostRide, Sample.Out.RideCm);
				MostTouching = FMath::Max(MostTouching, Sample.Touching);
			}
			AddInfo(FString::Printf(TEXT("%s | a 10 deg ramp at 30 cm/s | %s | on the ramp %.2f deg off at most, the chassis raised %.2f cm at most; at the end %.2f deg off, raised %.2f cm"), Kit.Name, FrameNames[Rate],
				OnRamp.PeakDeg, MostRide, Result.FinalTiltDeg, Result.Trace.Num() > 0 ? Result.Trace.Last().Out.RideCm : 0.0));
			TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("the ramp: level on it (%.2f deg)"), OnRamp.PeakDeg)), OnRamp.PeakDeg <= 0.5);
			TestEqual(Label(Kit, Rate, TEXT("the ramp: the chassis raised by what it needed")), MostRide, NeededCm, 0.4);
			TestTrue(Label(Kit, Rate, TEXT("the landing: level")), Result.SteadyResidualDeg <= 0.3);
			TestTrue(Label(Kit, Rate, TEXT("the landing: the ride height given back")), Result.Trace.Num() > 0 && FMath::Abs(Result.Trace.Last().Out.RideCm) <= 0.02);
			TestEqual(Label(Kit, Rate, TEXT("the raised wheels off the ground all the way")), MostTouching, 4);
		}
	}
	return true;
}

namespace RammsLevelDryRunTests
{
	/** How far the seat's tilt runs behind its command in the seat cases (s):
	 *  a quick servo, and one three times as slow. */
	const double SeatLags[2] = { 0.03, 0.1 };

	/** The rider between two times, from a trace: their pitch's peak-to-peak
	 *  (deg), their attitude's largest rate over 0.3 s (deg/s), the most they
	 *  were off what they asked for (deg), and how far the seat's tilt moved
	 *  (deg, peak-to-peak). */
	struct FRiderSwing
	{
		double RangeDeg = 0.0;
		double RateDegPerSec = 0.0;
		double PeakOffDeg = 0.0;
		double SeatRangeDeg = 0.0;
		int32  Frames = 0;
	};

	FRiderSwing RiderBetween(const FResult& Result, double FromSeconds, double ToSeconds, double TickSeconds)
	{
		FRiderSwing Out;
		double		Low = 0.0;
		double		High = 0.0;
		double		SeatLow = 0.0;
		double		SeatHigh = 0.0;
		const int32 Window = FMath::Max(1, FMath::RoundToInt32(0.3 / TickSeconds));
		for (int32 Index = 0; Index < Result.Trace.Num(); ++Index)
		{
			const FSample& Sample = Result.Trace[Index];
			if (Sample.Seconds < FromSeconds || Sample.Seconds > ToSeconds)
			{
				continue;
			}
			Low = Out.Frames > 0 ? FMath::Min(Low, Sample.RiderDeg.X) : Sample.RiderDeg.X;
			High = Out.Frames > 0 ? FMath::Max(High, Sample.RiderDeg.X) : Sample.RiderDeg.X;
			SeatLow = Out.Frames > 0 ? FMath::Min(SeatLow, Sample.SeatDeg) : Sample.SeatDeg;
			SeatHigh = Out.Frames > 0 ? FMath::Max(SeatHigh, Sample.SeatDeg) : Sample.SeatDeg;
			Out.PeakOffDeg = FMath::Max(Out.PeakOffDeg, Inclination(Sample.RiderDeg - Sample.RiderReferenceDeg));
			if (Index >= Window)
			{
				Out.RateDegPerSec = FMath::Max(Out.RateDegPerSec, (Sample.RiderDeg - Result.Trace[Index - Window].RiderDeg).Size() / (Window * TickSeconds));
			}
			++Out.Frames;
		}
		Out.RangeDeg = High - Low;
		Out.SeatRangeDeg = SeatHigh - SeatLow;
		return Out;
	}

	/** The rider through a stretch of a course (cm of travel), as RiderBetween. */
	FRiderSwing RiderOver(const FResult& Result, double FromCm, double ToCm, double TickSeconds)
	{
		double From = -1.0;
		double To = -1.0;
		for (const FSample& Sample : Result.Trace)
		{
			if (Sample.TravelCm >= FromCm && Sample.TravelCm <= ToCm)
			{
				From = From < 0.0 ? Sample.Seconds : From;
				To = Sample.Seconds;
			}
		}
		return From < 0.0 ? FRiderSwing() : RiderBetween(Result, From, To, TickSeconds);
	}

	/** The same run with the seat's trim switched off in the tuning: the
	 *  rider then rides the base, and a preference is the base's to hold. */
	FOptions WithoutSeat(const FOptions& Options)
	{
		FOptions Out = Options;
		Out.Tuning.bSeatCoversTransients = false;
		return Out;
	}
} // namespace RammsLevelDryRunTests

// Catches: a seat that answers what the BODY does. A base being braked, or
// knocked, pitches for a few tenths of a second and comes back by itself; a
// seat sent against that pitch arrives, a command interval and its own lag
// later, as the base is already on its way back, and the rider is swung the
// other way on top (the trap the legs fell into). The seat must leave both
// alone: the rider's swing with the seat no larger than the base's own. (A
// knock on a robot at cruising speed cannot be told from ground, by the seat
// any more than by the legs: there it is held to a twentieth more swing.)
// The same with the seat sent ahead by the ground's last turn
// (FSeatCoverTuning::GroundAhead), which is what that costs.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunSeatStopAndKnock,
	"Ramms.Level.DryRun.SeatLeavesAStopAndAKnockToPass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunSeatStopAndKnock::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : { SixWheel(), QuickOnFour() })
	{
		if (!TestTrue(FString::Printf(TEXT("%s: (the robot has a seat)"), Kit.Name), Kit.Robot.bSeat))
		{
			continue;
		}
		for (const double SeatLag : SeatLags)
		{
			for (const int32 Rate : { 0, 2 })
			{
				const auto Say = [&](const FString& Case, const FString& What) {
					return FString::Printf(TEXT("%s, seat lag %.2f s, %s, %s: %s"), Kit.Name, SeatLag, FrameNames[Rate], *Case, *What);
				};
				// The hard stop of StopFromSpeed, on the flat and on a grade.
				for (const double GradeDeg : { 0.0, Kit.GradeDeg })
				{
					FOptions Options = MakeOptions(Kit, Rate);
					Options.SpeedCmS = 60.0;
					Options.DriveLagSeconds = 0.1;
					Options.PitchPerAccelDeg = BodyPitchPerAccel;
					Options.SeatLagSeconds = SeatLag;
					Options.MaxSeconds = 9.0;
					Options.bKeepTrace = true;
					Options.Events.Add(MakeEvent(5.0, EEvent::SetSpeed, 0.0, 0.0));
					FOptions OffOptions = Options;
					OffOptions.bEnabledAtStart = false;
					const FResult	  Result = DryRun::Run(Kit.Robot, Plane(GradeDeg, 0.0), Options);
					const FRiderSwing With = RiderBetween(Result, 5.0, 8.0, Options.FrameSeconds);
					const FRiderSwing Without = RiderBetween(DryRun::Run(Kit.Robot, Plane(GradeDeg, 0.0), WithoutSeat(Options)), 5.0, 8.0, Options.FrameSeconds);
					const FRiderSwing Off = RiderBetween(DryRun::Run(Kit.Robot, Plane(GradeDeg, 0.0), OffOptions), 5.0, 8.0, Options.FrameSeconds);
					const FString	  Case = FString::Printf(TEXT("stop from 60 cm/s on %.0f deg"), GradeDeg);
					AddInfo(FString::Printf(TEXT("%s | seat lag %.2f s | %s | %s | rider swing %.2f deg at %.1f deg/s, the seat moving %.2f deg; with no seat trim %.2f at %.1f; switched off %.2f at %.1f"), Kit.Name, SeatLag,
						*Case, FrameNames[Rate], With.RangeDeg, With.RateDegPerSec, With.SeatRangeDeg, Without.RangeDeg, Without.RateDegPerSec, Off.RangeDeg, Off.RateDegPerSec));
					ExpectSound(*this, Kit, Rate, Result);
					TestTrue(Say(Case, TEXT("(the stop knocks the rider over a degree)")), Without.RangeDeg >= 1.0);
					TestTrue(Say(Case, FString::Printf(TEXT("the rider swung no further for the seat (%.2f against %.2f deg)"), With.RangeDeg, Without.RangeDeg)), With.RangeDeg <= Without.RangeDeg + 0.05);
					TestTrue(Say(Case, FString::Printf(TEXT("... nor faster (%.1f against %.1f deg/s)"), With.RateDegPerSec, Without.RateDegPerSec)), With.RateDegPerSec <= Without.RateDegPerSec + 0.3);
					TestTrue(Say(Case, FString::Printf(TEXT("the seat is not sent after the stop's pitch (%.2f deg)"), With.SeatRangeDeg)), With.SeatRangeDeg <= 0.15);
					TestTrue(Say(Case, TEXT("the rider level after it")), Result.SteadyRiderResidualDeg <= 0.3);

					// Sent ahead by the ground's last turn: a stop is still left
					// alone (the gate gives the reading up while the base brakes).
					FOptions AheadOptions = Options;
					AheadOptions.SeatCover.GroundAhead = 1.0;
					const FRiderSwing Ahead = RiderBetween(DryRun::Run(Kit.Robot, Plane(GradeDeg, 0.0), AheadOptions), 5.0, 8.0, Options.FrameSeconds);
					AddInfo(FString::Printf(TEXT("%s | seat lag %.2f s | %s, the seat sent ahead | %s | rider swing %.2f deg at %.1f deg/s, the seat moving %.2f deg"), Kit.Name, SeatLag, *Case, FrameNames[Rate], Ahead.RangeDeg,
						Ahead.RateDegPerSec, Ahead.SeatRangeDeg));
					TestTrue(Say(Case, FString::Printf(TEXT("sent ahead: the rider swung no further (%.2f against %.2f deg)"), Ahead.RangeDeg, Without.RangeDeg)), Ahead.RangeDeg <= Without.RangeDeg + 0.05);
				}
				// The knock of KnockStandingAndOnTheMove: standing, crawling,
				// cruising.
				for (const double Speed : { 0.0, 10.0, 40.0 })
				{
					FOptions Options = MakeOptions(Kit, Rate);
					Options.SpeedCmS = Speed;
					Options.SeatLagSeconds = SeatLag;
					Options.MaxSeconds = 8.0;
					Options.bKeepTrace = true;
					FDryEvent Kick = MakeEvent(3.0, EEvent::AttitudeKick, 0.15);
					Kick.PlaneDeg = FVector2D(-1.5, 0.0);
					Options.Events.Add(Kick);
					const FResult	  Result = DryRun::Run(Kit.Robot, Plane(0.0, 0.0), Options);
					const FRiderSwing With = RiderBetween(Result, 3.0, 6.0, Options.FrameSeconds);
					const FRiderSwing Without = RiderBetween(DryRun::Run(Kit.Robot, Plane(0.0, 0.0), WithoutSeat(Options)), 3.0, 6.0, Options.FrameSeconds);
					const FString	  Case = FString::Printf(TEXT("a 1.5 deg knock at %.0f cm/s"), Speed);
					AddInfo(FString::Printf(TEXT("%s | seat lag %.2f s | %s | %s | rider swing %.2f deg at %.1f deg/s, the seat moving %.2f deg; with no seat trim %.2f at %.1f"), Kit.Name, SeatLag, *Case, FrameNames[Rate],
						With.RangeDeg, With.RateDegPerSec, With.SeatRangeDeg, Without.RangeDeg, Without.RateDegPerSec));
					ExpectSound(*this, Kit, Rate, Result);
					TestTrue(Say(Case, FString::Printf(TEXT("... nor faster (%.1f against %.1f deg/s)"), With.RateDegPerSec, Without.RateDegPerSec)), With.RateDegPerSec <= Without.RateDegPerSec + 0.3);
					FOptions AheadOptions = Options;
					AheadOptions.SeatCover.GroundAhead = 1.0;
					const FRiderSwing Ahead = RiderBetween(DryRun::Run(Kit.Robot, Plane(0.0, 0.0), AheadOptions), 3.0, 6.0, Options.FrameSeconds);
					AddInfo(FString::Printf(TEXT("%s | seat lag %.2f s | %s, the seat sent ahead | %s | rider swing %.2f deg at %.1f deg/s, the seat moving %.2f deg"), Kit.Name, SeatLag, *Case, FrameNames[Rate], Ahead.RangeDeg,
						Ahead.RateDegPerSec, Ahead.SeatRangeDeg));
					if (Speed < 20.0)
					{
						TestTrue(Say(Case, FString::Printf(TEXT("the rider swung no further for the seat (%.2f against %.2f deg)"), With.RangeDeg, Without.RangeDeg)), With.RangeDeg <= Without.RangeDeg + 0.05);
						TestTrue(Say(Case, FString::Printf(TEXT("the seat is not sent after the knock (%.2f deg)"), With.SeatRangeDeg)), With.SeatRangeDeg <= 0.15);
					}
					else
					{
						TestTrue(Say(Case, FString::Printf(TEXT("the rider swung hardly further for the seat (%.2f against %.2f deg)"), With.RangeDeg, Without.RangeDeg)), With.RangeDeg <= 1.05 * Without.RangeDeg);
					}
					if (Speed > 0.0)
					{
						// What being sent ahead costs: a knock on the move reads
						// as the ground turning, and the seat goes after it.
						TestTrue(Say(Case, FString::Printf(TEXT("sent ahead: the rider swung at most an eighth further (%.2f against %.2f deg)"), Ahead.RangeDeg, Without.RangeDeg)), Ahead.RangeDeg <= 1.125 * Without.RangeDeg);
					}
					else
					{
						TestTrue(Say(Case, FString::Printf(TEXT("sent ahead: a knock standing is still left alone (%.2f against %.2f deg)"), Ahead.RangeDeg, Without.RangeDeg)), Ahead.RangeDeg <= Without.RangeDeg + 0.05);
					}
				}
			}
		}
	}
	return true;
}

// Catches: a seat that levels the rider at its own rate when the switch is
// thrown (every deliberate change of the rider's attitude is the tilt
// policy's: 3 deg/s, not the seat's 10), a seat that moves at all for a walk
// of the wanted attitude (the legs are told to follow it; the seat has no part
// in it), a seat left tilted after the switch is thrown back, and a seat sent
// after the sensor's noise on a robot that stands (the reading of a standing
// robot is let move no faster than ground can).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunSeatSwitch,
	"Ramms.Level.DryRun.SeatLeavesTheSwitchToTheTiltPolicy",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunSeatSwitch::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FKit Kit = SixWheel();
	for (const FVector2D& Preferred : { FVector2D::ZeroVector, FVector2D(3.0, -2.0), FVector2D(-3.0, 2.0) })
	{
		for (const int32 Rate : { 0, 2 })
		{
			// Parked nose up on the grade; switched on, and off again.
			FOptions Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 0.0;
			Options.bEnabledAtStart = false;
			Options.PreferenceDeg = Preferred;
			Options.MaxSeconds = 26.0;
			Options.bKeepTrace = true;
			Options.Events.Add(MakeEvent(1.0, EEvent::Enable));
			Options.Events.Add(MakeEvent(14.0, EEvent::Disable));
			const FResult Result = DryRun::Run(Kit.Robot, Plane(Kit.GradeDeg, 0.0), Options);
			const FString Case = FString::Printf(TEXT("parked on %.0f deg, prefers (%.0f, %.0f)"), Kit.GradeDeg, Preferred.X, Preferred.Y);
			Report(*this, Kit, *Case, Rate, Result);
			ExpectSound(*this, Kit, Rate, Result);
			const auto	   Say = [&](const TCHAR* What) { return FString::Printf(TEXT("%s, %s, %s: %s"), Kit.Name, *Case, FrameNames[Rate], What); };
			const FSample* Levelled = SampleAt(Result, 13.9);
			if (!TestTrue(Say(TEXT("a trace")), Levelled != nullptr))
			{
				continue;
			}
			const FRiderSwing On = RiderBetween(Result, 1.0, 13.9, Options.FrameSeconds);
			const FRiderSwing Back = RiderBetween(Result, 14.0, 26.0, Options.FrameSeconds);
			AddInfo(FString::Printf(TEXT("%s | %s | %s | switched on: rider turned at %.2f deg/s at most, ends (%.2f, %.2f) with the base at (%.2f, %.2f) and the seat at %.2f; switched off: %.2f deg/s, the seat left at %.4f"),
				Kit.Name, *Case, FrameNames[Rate], On.RateDegPerSec, Levelled->RiderDeg.X, Levelled->RiderDeg.Y, Levelled->TrueAttitudeDeg.X, Levelled->TrueAttitudeDeg.Y, Levelled->SeatDeg, Back.RateDegPerSec,
				Result.Trace.Last().SeatDeg));
			TestTrue(Say(TEXT("switched on: the rider turned within the tilt policy")), On.RateDegPerSec <= 3.5);
			TestTrue(Say(TEXT("... to what they asked for")), Inclination(Levelled->RiderDeg - Preferred) <= 0.3);
			// The pitch they prefer is the seat's; the base is level in pitch
			// and holds the roll.
			TestEqual(Say(TEXT("... the base's pitch level")), Levelled->TrueAttitudeDeg.X, 0.0, 0.3);
			TestEqual(Say(TEXT("... the base's roll the preferred one")), Levelled->TrueAttitudeDeg.Y, Preferred.Y, 0.3);
			TestEqual(Say(TEXT("... the seat holding the preferred pitch")), Levelled->SeatDeg, Preferred.X, 0.2);
			if (Preferred.IsZero())
			{
				// With no preference the seat has no part in a switch at all.
				TestTrue(Say(TEXT("... and the seat hardly moved on the way")), On.SeatRangeDeg <= 0.1 && Back.SeatRangeDeg <= 0.1);
			}
			TestTrue(Say(TEXT("switched off: the rider turned within the tilt policy")), Back.RateDegPerSec <= 3.5);
			TestEqual(Say(TEXT("... off")), static_cast<int32>(Result.FinalState), static_cast<int32>(EState::Off));
			TestTrue(Say(TEXT("... every offset exactly zero")), Result.FinalMaxOffsetCm == 0.0);
			TestTrue(Say(TEXT("... the seat's trim exactly zero")), Result.Trace.Last().Out.SeatTrimDeg == 0.0);
			TestTrue(Say(TEXT("... and the seat back where the rider has it")), FMath::Abs(Result.Trace.Last().SeatDeg) <= 1.0e-3);
		}
	}

	// A noisy, biased sensor on a grade: the seat is not sent after the noise.
	for (const int32 Rate : { 0, 2 })
	{
		FOptions Options = MakeOptions(Kit, Rate);
		Options.NoiseDeg = 0.2;
		Options.BiasDeg = FVector2D(0.5, -0.5);
		Options.SpeedCmS = 0.0;
		Options.MaxSeconds = 12.0;
		Options.bKeepTrace = true;
		const FResult	  Result = DryRun::Run(Kit.Robot, Plane(Kit.GradeDeg, 0.0), Options);
		const FRiderSwing Late = RiderBetween(Result, 8.0, 12.0, Options.FrameSeconds);
		AddInfo(FString::Printf(TEXT("%s | parked on a grade, 0.2 deg of noise | %s | the seat moving %.3f deg over the last 4 s"), Kit.Name, FrameNames[Rate], Late.SeatRangeDeg));
		ExpectSound(*this, Kit, Rate, Result);
		TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("noise: the seat does not chatter with the sensor (%.3f deg)"), Late.SeatRangeDeg)), Late.SeatRangeDeg <= 0.05);
	}

	// The same sensor on the move, on flat ground: there the reading passes for
	// ground (the ground could be turning that fast), the legs are sent after
	// it, and so is the seat. What that does to the rider, with the seat as it
	// is and with it sent ahead by the ground's last turn (which takes the
	// noise twice over): reported, and held to bounds.
	for (const int32 Rate : { 0, 2 })
	{
		FOptions Options = MakeOptions(Kit, Rate);
		Options.NoiseDeg = 0.2;
		Options.MaxSeconds = 12.0;
		Options.bKeepTrace = true;
		FOptions AheadOptions = Options;
		AheadOptions.SeatCover.GroundAhead = 1.0;
		const FResult	  Result = DryRun::Run(Kit.Robot, Plane(0.0, 0.0), Options);
		const FRiderSwing With = RiderBetween(Result, 6.0, 12.0, Options.FrameSeconds);
		const FRiderSwing Without = RiderBetween(DryRun::Run(Kit.Robot, Plane(0.0, 0.0), WithoutSeat(Options)), 6.0, 12.0, Options.FrameSeconds);
		const FRiderSwing Ahead = RiderBetween(DryRun::Run(Kit.Robot, Plane(0.0, 0.0), AheadOptions), 6.0, 12.0, Options.FrameSeconds);
		AddInfo(FString::Printf(TEXT("%s | flat at 40 cm/s, 0.2 deg of noise | %s | rider within %.2f deg at %.2f deg/s, the seat moving %.2f deg; with no seat trim %.2f at %.2f; the seat sent ahead %.2f at %.2f, moving %.2f"), Kit.Name,
			FrameNames[Rate], With.RangeDeg, With.RateDegPerSec, With.SeatRangeDeg, Without.RangeDeg, Without.RateDegPerSec, Ahead.RangeDeg, Ahead.RateDegPerSec, Ahead.SeatRangeDeg));
		ExpectSound(*this, Kit, Rate, Result);
		TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("noise on the move: the rider no worse off for the seat (%.2f against %.2f deg)"), With.RangeDeg, Without.RangeDeg)), With.RangeDeg <= Without.RangeDeg + 0.1);
		TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("noise on the move: the seat sent ahead moves more for it (%.2f against %.2f deg)"), Ahead.SeatRangeDeg, With.SeatRangeDeg)), Ahead.SeatRangeDeg >= With.SeatRangeDeg);
	}
	return true;
}

// Catches: the rider's fore/aft preference spent out of the legs' reach (the
// base tilted to it) on a robot whose seat could hold it; the lateral one put
// in the seat, which has no such axis; a change of the preference that steps
// the rider or moves them faster than the tilt policy; and, on a robot with
// no seat, the pitch preference lost.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunSeatPreference,
	"Ramms.Level.DryRun.SeatHoldsTheForeAftPreference",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunSeatPreference::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FKit Kit = SixWheel();
	// Leaning forward on an up-grade is what costs the legs most when the
	// base holds it: the grade and the preference add up.
	const FVector2D First(-3.0, 2.0);
	const FVector2D Second(2.0, -1.0);
	for (const int32 Rate : { 0, 2 })
	{
		FOptions Options = MakeOptions(Kit, Rate);
		Options.SpeedCmS = 0.0;
		Options.PreferenceDeg = First;
		Options.MaxSeconds = 24.0;
		Options.bKeepTrace = true;
		FDryEvent Change = MakeEvent(12.0, EEvent::SetPreference);
		Change.PlaneDeg = Second;
		Options.Events.Add(Change);
		FOptions PlainOptions = Options;
		PlainOptions.PreferenceDeg = FVector2D(0.0, First.Y);
		PlainOptions.Events.Reset();
		PlainOptions.MaxSeconds = 11.5;
		FOptions BaseOptions = WithoutSeat(Options);
		BaseOptions.Events.Reset();
		BaseOptions.MaxSeconds = 11.5;
		const FGround Course = Plane(Kit.GradeDeg, 0.0);
		const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
		const FResult Plain = DryRun::Run(Kit.Robot, Course, PlainOptions);
		const FResult InBase = DryRun::Run(Kit.Robot, Course, BaseOptions);
		Report(*this, Kit, TEXT("a preference held in the seat, then changed"), Rate, Result);
		ExpectSound(*this, Kit, Rate, Result);
		const FSample* Before = SampleAt(Result, 11.5);
		const FSample* After = Result.Trace.Num() > 0 ? &Result.Trace.Last() : nullptr;
		const FSample* PlainEnd = Plain.Trace.Num() > 0 ? &Plain.Trace.Last() : nullptr;
		const FSample* BaseEnd = InBase.Trace.Num() > 0 ? &InBase.Trace.Last() : nullptr;
		if (!TestTrue(Label(Kit, Rate, TEXT("traces")), Before && After && PlainEnd && BaseEnd))
		{
			continue;
		}
		const FRiderSwing Changing = RiderBetween(Result, 12.0, 24.0, Options.FrameSeconds);
		AddInfo(FString::Printf(TEXT("%s | prefers (%.0f, %.0f) on a %.0f deg grade | %s | rider (%.2f, %.2f), base (%.2f, %.2f), seat %.2f, reach left %.2f (%.2f with no fore/aft preference, %.2f with the base holding it); changed to (%.0f, %.0f): rider (%.2f, %.2f) at %.2f deg/s at most, base (%.2f, %.2f), seat %.2f"),
			Kit.Name, First.X, First.Y, Kit.GradeDeg, FrameNames[Rate], Before->RiderDeg.X, Before->RiderDeg.Y, Before->TrueAttitudeDeg.X, Before->TrueAttitudeDeg.Y, Before->SeatDeg, Before->Out.ReachLeft,
			PlainEnd->Out.ReachLeft, BaseEnd->Out.ReachLeft, Second.X, Second.Y, After->RiderDeg.X, After->RiderDeg.Y, Changing.RateDegPerSec, After->TrueAttitudeDeg.X, After->TrueAttitudeDeg.Y, After->SeatDeg));
		TestTrue(Label(Kit, Rate, TEXT("the rider at what they asked for")), Inclination(Before->RiderDeg - First) <= 0.3);
		TestEqual(Label(Kit, Rate, TEXT("the base level fore and aft")), Before->TrueAttitudeDeg.X, 0.0, 0.3);
		TestEqual(Label(Kit, Rate, TEXT("the base holding the lateral preference")), Before->TrueAttitudeDeg.Y, First.Y, 0.3);
		TestEqual(Label(Kit, Rate, TEXT("the seat holding the fore/aft preference")), Before->SeatDeg, First.X, 0.2);
		// What it is for: the legs' reach is not spent on it.
		TestEqual(Label(Kit, Rate, TEXT("the fore/aft preference costs the legs no reach")), Before->Out.ReachLeft, PlainEnd->Out.ReachLeft, 0.02);
		TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("(held by the base it would: reach left %.2f against %.2f)"), BaseEnd->Out.ReachLeft, Before->Out.ReachLeft)),
			BaseEnd->Out.ReachLeft <= Before->Out.ReachLeft - 0.1);
		TestTrue(Label(Kit, Rate, TEXT("(with the seat's trim off the base holds it, and the rider is where they asked all the same)")),
			FMath::Abs(BaseEnd->TrueAttitudeDeg.X - First.X) <= 0.3 && Inclination(BaseEnd->RiderDeg - First) <= 0.3 && BaseEnd->SeatDeg == 0.0);

		TestTrue(Label(Kit, Rate, TEXT("changed: the rider at the new preference")), Inclination(After->RiderDeg - Second) <= 0.3);
		TestEqual(Label(Kit, Rate, TEXT("... the base still level fore and aft")), After->TrueAttitudeDeg.X, 0.0, 0.3);
		TestEqual(Label(Kit, Rate, TEXT("... the seat at the new fore/aft preference")), After->SeatDeg, Second.X, 0.2);
		TestTrue(Label(Kit, Rate, *FString::Printf(TEXT("... moved within the tilt policy (%.2f deg/s)"), Changing.RateDegPerSec)), Changing.RateDegPerSec <= 3.5);
		// (5 deg of pitch and 3 of roll together: 5.8 deg at 3 deg/s.)
		TestTrue(Label(Kit, Rate, TEXT("... and it did move")), Changing.RateDegPerSec >= 2.0);
	}

	// A robot with no seat: both go into the base, as before.
	const FKit Pairs = DrivePairs();
	for (const int32 Rate : { 0, 2 })
	{
		const FVector2D Preferred = PreferredTilt(Pairs);
		FOptions		Options = MakeOptions(Pairs, Rate);
		Options.PreferenceDeg = Preferred;
		Options.SpeedCmS = 0.0;
		Options.MaxSeconds = 10.0;
		Options.bKeepTrace = true;
		const FResult Result = DryRun::Run(Pairs.Robot, Plane(Pairs.GradeDeg, 0.0), Options);
		ExpectSound(*this, Pairs, Rate, Result);
		if (TestTrue(Label(Pairs, Rate, TEXT("a trace")), Result.Trace.Num() > 0))
		{
			const FSample& End = Result.Trace.Last();
			TestTrue(Label(Pairs, Rate, TEXT("no seat: the base holds the whole preference")), (End.TrueAttitudeDeg - Preferred).Size() <= 0.3);
			TestTrue(Label(Pairs, Rate, TEXT("... and there is no trim")), Result.MaxSeatDeg == 0.0 && End.Out.SeatTrimDeg == 0.0);
		}
	}
	return true;
}

// Catches: a slope past the legs' reach left at the rider when the seat could
// take it out; the seat taking it out at its own rate rather than as the base
// is let go (the two must cancel at the rider); a trim past its limit or the
// seat's own range; and a seat left tilted when the slope is behind.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunSeatPastReach,
	"Ramms.Level.DryRun.SeatTakesOutWhatIsPastTheLegsReach",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunSeatPastReach::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	// As RideHeightBuysReachPastTheRaisedWheels: nose up, the raised front
	// pair's clearance gives this robot 8.26 deg, so 10 deg leaves the base
	// about 1.7 deg nose up.
	const FKit Kit = QuickOnFour();
	for (const double SeatLag : SeatLags)
	{
		for (const int32 Rate : { 0, 2 })
		{
			const auto Say = [&](const TCHAR* What) { return FString::Printf(TEXT("%s, seat lag %.2f s, %s: %s"), Kit.Name, SeatLag, FrameNames[Rate], What); };
			{
				// Parked on it, the switch thrown; then a take-over and its hand-back.
				FOptions Options = MakeOptions(Kit, Rate);
				Options.SpeedCmS = 0.0;
				Options.bEnabledAtStart = false;
				Options.SeatLagSeconds = SeatLag;
				Options.MaxSeconds = 30.0;
				Options.bKeepTrace = true;
				Options.Events.Add(MakeEvent(1.0, EEvent::Enable));
				Options.Events.Add(MakeEvent(14.0, EEvent::FoldAdopt, 3.0));
				const FResult  Result = DryRun::Run(Kit.Robot, Plane(10.0, 0.0), Options);
				const FSample* Held = SampleAt(Result, 13.9);
				Report(*this, Kit, TEXT("parked 10 deg nose up, past reach"), Rate, Result);
				ExpectSound(*this, Kit, Rate, Result);
				if (!TestTrue(Say(TEXT("a trace")), Held != nullptr))
				{
					continue;
				}
				const FRiderSwing Whole = RiderBetween(Result, 1.0, 30.0, Options.FrameSeconds);
				const FSample&	  End = Result.Trace.Last();
				AddInfo(FString::Printf(TEXT("%s | seat lag %.2f s | parked 10 deg nose up | %s | base left %.2f deg nose up, seat %.2f, rider %.2f off; through the switch, a take-over and its hand-back the rider turned at %.2f deg/s at most; at the end base %.2f, seat %.2f, rider %.2f off"),
					Kit.Name, SeatLag, FrameNames[Rate], Held->TrueAttitudeDeg.X, Held->SeatDeg, Inclination(Held->RiderDeg), Whole.RateDegPerSec, End.TrueAttitudeDeg.X, End.SeatDeg, Inclination(End.RiderDeg)));
				TestTrue(Say(TEXT("the legs leave the base over a degree nose up")), Held->TrueAttitudeDeg.X >= 1.4);
				TestTrue(Say(TEXT("the rider is level all the same")), Inclination(Held->RiderDeg) <= 0.3);
				TestEqual(Say(TEXT("the seat holds what the legs left")), Held->SeatDeg, -Held->TrueAttitudeDeg.X, 0.2);
				// (A seat three times as slow as the legs is a little out of step
				// with them while both are walked: held to the rider's own limit.)
				TestTrue(Say(TEXT("the rider turned within the tilt policy throughout")), Whole.RateDegPerSec <= (SeatLag <= 0.05 ? 3.5 : RiderRateLimitDegPerSec));
				// The take-over took the trim into the seat's own command (no
				// step), brought the rider to their own setting, and the
				// hand-back's levelling took the slope out of the seat again.
				TestEqual(Say(TEXT("a take-over was folded for, and handed back")), static_cast<int32>(Result.FinalState), static_cast<int32>(EState::Active));
				TestTrue(Say(TEXT("after the hand-back: the rider level again")), Inclination(End.RiderDeg) <= 0.3);
				TestEqual(Say(TEXT("... the seat holding what the legs leave")), End.SeatDeg, -End.TrueAttitudeDeg.X, 0.2);
			}
			{
				// Driven up a 10 deg ramp at a walk and onto its landing.
				const FVector2D Knots[2] = { { 150.0, 0.0 }, { 650.0, Rise(500.0, 10.0) } };
				FOptions		Options = MakeOptions(Kit, Rate);
				Options.SpeedCmS = 30.0;
				Options.SeatLagSeconds = SeatLag;
				Options.DistanceCm = 1000.0;
				Options.MaxSeconds = 90.0;
				Options.bKeepTrace = true;
				const FResult	  Result = DryRun::Run(Kit.Robot, Profile(MakeArrayView(Knots)), Options);
				const FResult	  NoSeat = DryRun::Run(Kit.Robot, Profile(MakeArrayView(Knots)), WithoutSeat(Options));
				const FRiderSwing OnRamp = RiderOver(Result, 420.0, 640.0, Options.FrameSeconds);
				const FRiderSwing Was = RiderOver(NoSeat, 420.0, 640.0, Options.FrameSeconds);
				const FRiderSwing Whole = RiderOver(Result, 0.0, 1000.0, Options.FrameSeconds);
				const FRiderSwing WholeWas = RiderOver(NoSeat, 0.0, 1000.0, Options.FrameSeconds);
				Report(*this, Kit, TEXT("a 10 deg ramp, past reach"), Rate, Result);
				ExpectSound(*this, Kit, Rate, Result);
				AddInfo(FString::Printf(TEXT("%s | seat lag %.2f s | a 10 deg ramp at 30 cm/s | %s | on the ramp the rider %.2f deg off at most (%.2f with no seat trim); over the course %.2f deg at %.2f deg/s (%.2f at %.2f); at the end seat %.3f, rider %.2f off"),
					Kit.Name, SeatLag, FrameNames[Rate], OnRamp.PeakOffDeg, Was.PeakOffDeg, Whole.PeakOffDeg, Whole.RateDegPerSec, WholeWas.PeakOffDeg, WholeWas.RateDegPerSec, Result.Trace.Num() > 0 ? Result.Trace.Last().SeatDeg : 0.0,
					Result.FinalRiderTiltDeg));
				TestTrue(Say(TEXT("(with no seat trim the ramp leaves the rider over a degree off)")), Was.PeakOffDeg >= 1.4);
				TestTrue(Say(TEXT("the ramp: the rider level on it")), OnRamp.PeakOffDeg <= 0.5);
				TestTrue(Say(TEXT("the course: the rider no further off than with no seat trim")), Whole.PeakOffDeg <= WholeWas.PeakOffDeg + 0.1);
				TestTrue(Say(TEXT("the course: the rider turned no faster than with no seat trim")), Whole.RateDegPerSec <= WholeWas.RateDegPerSec + 0.2);
				TestTrue(Say(TEXT("the landing: the rider level")), Result.SteadyRiderResidualDeg <= 0.3);
				// (All but its cover of what the legs' own deadband leaves.)
				TestTrue(Say(TEXT("the landing: the seat given back")), Result.Trace.Num() > 0 && FMath::Abs(Result.Trace.Last().SeatDeg) <= Kit.Tuning.DeadbandDeg);
			}
		}
	}

	// The trim's limits: the tuning's, and the seat's own range.
	for (const int32 Limit : { 0, 1 })
	{
		FOptions Options = MakeOptions(Kit, 0);
		Options.SpeedCmS = 0.0;
		Options.MaxSeconds = 12.0;
		Options.bKeepTrace = true;
		Options.Tuning.SeatTrimMaxDeg = Limit == 0 ? 1.0 : 8.0;
		Options.SeatRoomDeg = Limit == 0 ? 1.0e6 : 0.5;
		const double   Most = Limit == 0 ? 1.0 : 0.5;
		const FResult  Result = DryRun::Run(Kit.Robot, Plane(10.0, 0.0), Options);
		const TCHAR*   Name = Limit == 0 ? TEXT("a 1 deg trim limit") : TEXT("a seat with half a degree of range");
		const FSample* End = Result.Trace.Num() > 0 ? &Result.Trace.Last() : nullptr;
		ExpectSound(*this, Kit, 0, Result);
		if (TestTrue(Label(Kit, 0, Name), End != nullptr))
		{
			AddInfo(FString::Printf(TEXT("%s | parked 10 deg nose up, %s | base %.2f, seat %.2f, rider %.2f off"), Kit.Name, Name, End->TrueAttitudeDeg.X, End->SeatDeg, Inclination(End->RiderDeg)));
			TestTrue(Label(Kit, 0, *FString::Printf(TEXT("%s: the seat never past it"), Name)), Result.MaxSeatDeg <= Most + 1.0e-6);
			TestEqual(Label(Kit, 0, *FString::Printf(TEXT("%s: the seat on it"), Name)), End->SeatDeg, -Most, 0.02);
			TestEqual(Label(Kit, 0, *FString::Printf(TEXT("%s: the rider left with the rest"), Name)), End->RiderDeg.X, End->TrueAttitudeDeg.X - Most, 0.02);
		}
	}
	return true;
}

// Catches: a seat that does not earn its keep at a seam. Where the legs cannot
// turn the plane as fast as the ground turns under them the base is left
// behind its wanted attitude; that error is the seat's to take out while the
// legs catch up, and to give back as they do. And a seat that makes any seam
// of the speed cap's courses worse for the rider than leaving the seat alone,
// or that takes less off the seams when it is sent ahead by the ground's last
// turn than when it is not.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunSeatCover,
	"Ramms.Level.DryRun.SeatCoversLegsThatCannotKeepUp",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunSeatCover::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const TArray<FReachCourse> Courses = ReachCourses();
	// Legs that turn the plane at 4 deg/s, over the 12 deg crest: the drive is
	// held to the speed at which the slope's end turns the ground as fast as
	// they can follow, and the crest turns it twice as far.
	{
		FKit Slow = FollowingOnly(QuickOnFour());
		Slow.Name = TEXT("quick on four, a slow plane");
		Slow.Tuning.MaxCorrectionRateDegPerSec = 4.0;
		const FReachCourse& Crest = Courses[0];
		for (const double SeatLag : SeatLags)
		{
			for (const int32 Rate : { 0, 2 })
			{
				FOptions Options = MakeOptions(Slow, Rate);
				Options.SpeedCmS = 0.0;
				Options.SeatLagSeconds = SeatLag;
				Options.DistanceCm = Crest.DistanceCm;
				Options.MaxSeconds = 120.0;
				Options.PitchPerAccelDeg = BodyPitchPerAccel;
				Options.bKeepTrace = true;
				Options.Events.Add(MakeEvent(5.0, EEvent::SetSpeed, 0.0, 60.0));
				const FResult	  Result = DryRun::Run(Slow.Robot, Crest.Ground, Options);
				const FResult	  NoSeat = DryRun::Run(Slow.Robot, Crest.Ground, WithoutSeat(Options));
				const FRiderSwing With = RiderOver(Result, 630.0, 650.0 + OnFourSpanCm + 150.0, Options.FrameSeconds);
				const FRiderSwing Without = RiderOver(NoSeat, 630.0, 650.0 + OnFourSpanCm + 150.0, Options.FrameSeconds);
				Report(*this, Slow, TEXT("12 deg crest, the plane at 4 deg/s"), Rate, Result);
				ExpectSound(*this, Slow, Rate, Result);
				AddInfo(FString::Printf(TEXT("%s | seat lag %.2f s | the crest | %s | rider %.2f deg off at %.1f deg/s, the seat moving %.2f deg; with no seat trim %.2f at %.1f"), Slow.Name, SeatLag, FrameNames[Rate], With.PeakOffDeg,
					With.RateDegPerSec, With.SeatRangeDeg, Without.PeakOffDeg, Without.RateDegPerSec));
				const auto Say = [&](const FString& What) { return FString::Printf(TEXT("%s, seat lag %.2f s, %s: %s"), Slow.Name, SeatLag, FrameNames[Rate], *What); };
				TestTrue(Say(FString::Printf(TEXT("(the legs are left behind at the crest: %.2f deg with no seat trim)"), Without.PeakOffDeg)), Without.PeakOffDeg >= 1.2);
				TestTrue(Say(FString::Printf(TEXT("the seat takes out a third and more of what the legs owe (%.2f against %.2f deg)"), With.PeakOffDeg, Without.PeakOffDeg)), With.PeakOffDeg <= 0.7 * Without.PeakOffDeg);
				TestTrue(Say(FString::Printf(TEXT("... and the rider is turned no faster for it (%.1f against %.1f deg/s)"), With.RateDegPerSec, Without.RateDegPerSec)), With.RateDegPerSec <= Without.RateDegPerSec + 0.2);
				TestTrue(Say(TEXT("the seat given back once the legs have caught up")), Result.Trace.Num() > 0 && FMath::Abs(Result.Trace.Last().SeatDeg) <= Slow.Tuning.DeadbandDeg && Result.FinalRiderTiltDeg <= 0.2);
			}
		}
	}

	// The speed cap's courses as they are: no seam worse for the seat, quick
	// or slow. And what sending the seat ahead by the ground's last turn buys
	// there (FSeatCoverTuning::GroundAhead).
	const FKit Kit = FollowingOnly(QuickOnFour());
	double	   SumWithout = 0.0;
	double	   SumWith = 0.0;
	double	   SumAhead = 0.0;
	for (const FReachCourse& Course : Courses)
	{
		for (const double Speed : { 60.0, 120.0 })
		{
			for (const int32 Rate : { 0, 2 })
			{
				for (const double SeatLag : SeatLags)
				{
					FOptions Options = MakeOptions(Kit, Rate);
					Options.SpeedCmS = 0.0;
					Options.SeatLagSeconds = SeatLag;
					Options.DistanceCm = Course.DistanceCm;
					Options.MaxSeconds = 90.0;
					Options.PitchPerAccelDeg = BodyPitchPerAccel;
					Options.bKeepTrace = true;
					Options.Events.Add(MakeEvent(5.0, EEvent::SetSpeed, 0.0, Speed));
					FOptions AheadOptions = Options;
					AheadOptions.SeatCover.GroundAhead = 1.0;
					const FResult Result = DryRun::Run(Kit.Robot, Course.Ground, Options);
					const FResult NoSeat = DryRun::Run(Kit.Robot, Course.Ground, WithoutSeat(Options));
					const FResult Sent = DryRun::Run(Kit.Robot, Course.Ground, AheadOptions);
					ExpectSound(*this, Kit, Rate, Result);
					ExpectSound(*this, Kit, Rate, Sent);
					for (const FSeam& Seam : Course.Seams)
					{
						const double	  From = Seam.AtCm - 20.0;
						const double	  To = Seam.AtCm + Seam.LengthCm + OnFourSpanCm + 100.0;
						const FRiderSwing With = RiderOver(Result, From, To, Options.FrameSeconds);
						const FRiderSwing Without = RiderOver(NoSeat, From, To, Options.FrameSeconds);
						const FRiderSwing Ahead = RiderOver(Sent, From, To, Options.FrameSeconds);
						AddInfo(FString::Printf(TEXT("%s at %.0f cm/s | %s | seat lag %.2f s | %s | rider %.2f deg off at %.1f deg/s, the seat moving %.2f deg; with no seat trim %.2f at %.1f; the seat sent ahead %.2f at %.1f"), Course.Name,
							Speed, Seam.Name, SeatLag, FrameNames[Rate], With.PeakOffDeg, With.RateDegPerSec, With.SeatRangeDeg, Without.PeakOffDeg, Without.RateDegPerSec, Ahead.PeakOffDeg, Ahead.RateDegPerSec));
						const auto Say = [&](const FString& What) {
							return FString::Printf(TEXT("%s at %.0f cm/s, %s, seat lag %.2f s, %s: %s"), Course.Name, Speed, Seam.Name, SeatLag, FrameNames[Rate], *What);
						};
						// (A seat three times as slow as the legs comes a little
						// late to the end of a turn: a tenth more rate at most.)
						const double RateSlack = SeatLag <= 0.05 ? 0.3 : 0.3 + 0.1 * Without.RateDegPerSec;
						TestTrue(Say(FString::Printf(TEXT("the rider no further off for the seat (%.2f against %.2f deg)"), With.PeakOffDeg, Without.PeakOffDeg)), With.PeakOffDeg <= Without.PeakOffDeg + 0.1);
						TestTrue(Say(FString::Printf(TEXT("the rider turned no faster for the seat (%.1f against %.1f deg/s)"), With.RateDegPerSec, Without.RateDegPerSec)), With.RateDegPerSec <= Without.RateDegPerSec + RateSlack);
						TestTrue(Say(FString::Printf(TEXT("sent ahead: the rider no further off (%.2f against %.2f deg)"), Ahead.PeakOffDeg, Without.PeakOffDeg)), Ahead.PeakOffDeg <= Without.PeakOffDeg + 0.1);
						SumWithout += Without.PeakOffDeg;
						SumWith += With.PeakOffDeg;
						SumAhead += Ahead.PeakOffDeg;
					}
				}
			}
		}
	}
	AddInfo(FString::Printf(TEXT("every seam, both speeds, both rates, both seats: the rider's peaks add up to %.1f deg with no seat trim, %.1f with it, %.1f with the seat sent ahead"), SumWithout, SumWith, SumAhead));
	TestTrue(FString::Printf(TEXT("over all the seams the seat takes a tenth and more off the rider's peaks (%.1f against %.1f deg)"), SumWith, SumWithout), SumWith <= 0.9 * SumWithout);
	TestTrue(FString::Printf(TEXT("sent ahead it takes more (%.1f against %.1f deg)"), SumAhead, SumWith), SumAhead <= 0.9 * SumWith);
	return true;
}

// Catches: the law's seat trim not gone on from when a taker hands the legs back without ever having had the seat
// (FInputs::SeatTrimOnDeg): begun again from nothing, the seat is stepped by the whole trim at the hand-back and
// walked back; begun again on top of what is there, the rider gets it twice. (The taker here is the harness's own,
// as in every dry run: this is the law's bookkeeping. What the curb controller, the mode selector's stance ramp and
// the seat's controller do with the seat is tried on the component: Ramms.Level.Assist.SeatTrimKept... and
// ...CrossingTakesTheSeatWithItsTrim.)
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunSeatLeftAlone,
	"Ramms.Level.DryRun.SeatTrimGoneOnFromWhereTheTakerNeverTookTheSeat",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunSeatLeftAlone::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;
	// Parked 10 deg nose up, past the legs' reach: the seat holds what the legs leave. With a preference on top.
	const FKit Kit = QuickOnFour();
	for (const int32 Rate : { 0, 2 })
	{
		for (const bool bCold : { false, true })
		{
			const auto Say = [&](const TCHAR* What) { return FString::Printf(TEXT("%s, %s, %s: %s"), Kit.Name, FrameNames[Rate], bCold ? TEXT("a cold hand-back") : TEXT("a hand-back with its plane"), What); };
			FOptions   Options = MakeOptions(Kit, Rate);
			Options.SpeedCmS = 0.0;
			Options.bEnabledAtStart = false;
			Options.MaxSeconds = 26.0;
			Options.bKeepTrace = true;
			Options.PreferenceDeg = FVector2D(-2.0, 0.0);
			Options.Events.Add(MakeEvent(1.0, EEvent::Enable));
			FDryEvent TakeOver = MakeEvent(14.0, EEvent::FoldAdopt, 3.0);
			TakeOver.bLeaveSeat = true;
			TakeOver.bCold = bCold;
			Options.Events.Add(TakeOver);
			const FResult  Result = DryRun::Run(Kit.Robot, Plane(10.0, 0.0), Options);
			const FSample* Held = SampleAt(Result, 13.9);
			const FSample* Taken = SampleAt(Result, 16.0);
			if (!TestTrue(Say(TEXT("a trace")), Held != nullptr && Taken != nullptr && Result.Trace.Num() > 0))
			{
				continue;
			}
			const FSample& End = Result.Trace.Last();
			// (A cold hand-back gives the legs' plane up: the base is let go to the ground's attitude and levelled
			// again, and the seat's share follows the base through that within the policy. The seat itself must not
			// be stepped either way: the window is the hand-back's own few frames.)
			const FRiderSwing AtHandBack = RiderBetween(Result, 16.9, 17.0 + 4.0 * Options.FrameSeconds, Options.FrameSeconds);
			const FRiderSwing Through = RiderBetween(Result, 13.9, 26.0, Options.FrameSeconds);
			AddInfo(Say(*FString::Printf(TEXT("before the take-over the seat held %.2f deg, the rider %.2f; taken, the seat %.2f; over the hand-back's frames the seat moved %.3f deg; through it all the rider turned at %.2f deg/s at most; at the end the seat %.2f, the rider %.2f"),
				Held->SeatDeg, Held->RiderDeg.X, Taken->SeatDeg, AtHandBack.SeatRangeDeg, Through.RateDegPerSec, End.SeatDeg, End.RiderDeg.X)));
			TestTrue(Say(TEXT("(the seat carries a trim: the preference and what the legs leave)")), FMath::Abs(Held->SeatDeg) >= 0.2 && FMath::IsNearlyEqual(Held->RiderDeg.X, -2.0, 0.3));
			TestEqual(Say(TEXT("while the taker has the legs and not the seat: the seat stays as it was")), Taken->SeatDeg, Held->SeatDeg, 0.02);
			TestTrue(Say(TEXT("the hand-back does not step the seat")), AtHandBack.SeatRangeDeg <= 0.25);
			TestEqual(Say(TEXT("a take-over was folded for, and handed back")), static_cast<int32>(Result.FinalState), static_cast<int32>(EState::Active));
			TestEqual(Say(TEXT("at the end the rider has their preference, once")), End.RiderDeg.X, -2.0, 0.3);
			if (!bCold)
			{
				TestTrue(Say(TEXT("the rider is not turned by the take-over or its hand-back")), Through.RateDegPerSec <= 1.0);
			}
		}
	}
	return true;
}

// Catches: a fore/aft preference the seat has no room for dropped, where the legs could have held it (the seat's own
// room about where its rider has it; the trim's limit).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunPreferencePastTheSeat,
	"Ramms.Level.DryRun.PreferenceTheSeatHasNoRoomForIsTheLegs",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunPreferencePastTheSeat::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;
	const FKit Kit = QuickOnFour();
	for (const int32 Limit : { 0, 1 })
	{
		const TCHAR* const Name = Limit == 0 ? TEXT("a seat with 2 deg of room") : TEXT("a trim limit of 3 deg");
		FOptions		   Options = MakeOptions(Kit, 0);
		Options.SpeedCmS = 0.0;
		Options.MaxSeconds = 14.0;
		Options.bKeepTrace = true;
		Options.PreferenceDeg = FVector2D(5.0, 0.0);
		if (Limit == 0)
		{
			Options.SeatRoomDeg = 2.0;
		}
		else
		{
			Options.Tuning.SeatTrimMaxDeg = 3.0;
		}
		const double  Room = Limit == 0 ? 2.0 : 3.0;
		const FResult Result = DryRun::Run(Kit.Robot, Plane(0.0, 0.0), Options);
		if (!TestTrue(FString::Printf(TEXT("%s: a trace"), Name), Result.Trace.Num() > 0))
		{
			continue;
		}
		const FSample& End = Result.Trace.Last();
		AddInfo(FString::Printf(TEXT("%s | parked on the flat, 5 deg nose up preferred | the seat holds %.2f deg, the base %.2f; the rider %.2f"), Name, End.SeatDeg, End.TrueAttitudeDeg.X, End.RiderDeg.X));
		TestEqual(FString::Printf(TEXT("%s: the seat holds what it has room for"), Name), End.SeatDeg, Room, 0.2);
		TestEqual(FString::Printf(TEXT("%s: the base holds the rest"), Name), End.TrueAttitudeDeg.X, 5.0 - Room, 0.3);
		TestEqual(FString::Printf(TEXT("%s: the rider has the whole preference"), Name), End.RiderDeg.X, 5.0, 0.3);
	}
	return true;
}

namespace RammsLevelDryRunTests
{
	/** The frame rates the unlocked cases are run at (s), and their names. */
	const double	   UnlockedFrameSeconds[3] = { 1.0 / 16.0, 1.0 / 30.0, 1.0 / 50.0 };
	const TCHAR* const UnlockedFrameNames[3] = { TEXT("16 fps"), TEXT("30 fps"), TEXT("50 fps") };

} // namespace RammsLevelDryRunTests

// Catches: a law that is only right when every frame holds the same batch of physics. With the simulator stepping
// on its own thread (how the robot is really driven: FUnlocked) frames come uneven, the pose a tick reads is most of
// a frame old, and the clock the tick reads is neither the frame's time nor the pose's. Run on that clock, the
// chassis' speed (a distance between two poses over the time between two looks at the clock) swings by a quarter
// from frame to frame, the ground gate takes that for a base being braked and stops believing the attitude, and a
// crest is not levelled at all: as far off as with the switch off, or further. Run on the time the poses were
// taken at, the crest and the break are levelled as they are frame-locked, at 16, 30 and 50 frames a second.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunUnlockedCrest,
	"Ramms.Level.DryRun.UnlockedSteppingCrestAndBreak",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunUnlockedCrest::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FKit				   Kit = FollowingOnly(QuickOnFour());
	const TArray<FReachCourse> Courses = ReachCourses();
	// The 12 deg crest and the 6 deg break.
	for (int32 CourseIndex = 0; CourseIndex < 2 && CourseIndex < Courses.Num(); ++CourseIndex)
	{
		const FReachCourse& Course = Courses[CourseIndex];
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			// Levelled where it stands for five seconds, then driven at its full speed, the speed cap applied.
			FOptions Options = MakeOptions(Kit, 0);
			Options.FrameSeconds = UnlockedFrameSeconds[Rate];
			Options.SpeedCmS = 0.0;
			Options.DistanceCm = Course.DistanceCm;
			Options.MaxSeconds = 90.0;
			Options.PitchPerAccelDeg = BodyPitchPerAccel;
			Options.bKeepTrace = true;
			Options.Events.Add(MakeEvent(5.0, EEvent::SetSpeed, 0.0, 60.0));
			FOptions OffOptions = Options;
			OffOptions.bEnabledAtStart = false;
			FOptions UnlockedOptions = Options;
			UnlockedOptions.Unlocked.bOn = true;
			FOptions TickClockOptions = UnlockedOptions;
			TickClockOptions.Unlocked.bOnTickClock = true;
			const FResult Off = DryRun::Run(Kit.Robot, Course.Ground, OffOptions);
			const FResult Locked = DryRun::Run(Kit.Robot, Course.Ground, Options);
			const FResult Unlocked = DryRun::Run(Kit.Robot, Course.Ground, UnlockedOptions);
			const FResult TickClock = DryRun::Run(Kit.Robot, Course.Ground, TickClockOptions);
			const auto	  Say = [&](const TCHAR* Where, const FString& What) { return FString::Printf(TEXT("%s, %s, %s unlocked: %s"), Course.Name, Where, UnlockedFrameNames[Rate], *What); };
			ExpectSound(*this, Kit, 0, Unlocked);
			TestTrue(Say(TEXT("the course"), TEXT("driven to its end")), Unlocked.TravelCm >= Course.DistanceCm);
			TestTrue(Say(TEXT("the course"), FString::Printf(TEXT("never slowed to a stop (%.1f cm/s at least)"), Unlocked.MinSpeedCmS)), Unlocked.MinSpeedCmS >= 0.2 * 60.0 * Kit.Tuning.MinSpeedScale);
			double WorstUnlocked = 0.0;
			double WorstTickClock = 0.0;
			for (const FSeam& Seam : Course.Seams)
			{
				const double	  From = Seam.AtCm - 20.0;
				const double	  To = Seam.AtCm + Seam.LengthCm + OnFourSpanCm + 100.0;
				const FRiderSwing Was = RiderOver(Off, From, To, Options.FrameSeconds);
				const FRiderSwing Even = RiderOver(Locked, From, To, Options.FrameSeconds);
				const FRiderSwing Uneven = RiderOver(Unlocked, From, To, Options.FrameSeconds);
				const FRiderSwing Old = RiderOver(TickClock, From, To, Options.FrameSeconds);
				WorstUnlocked = FMath::Max(WorstUnlocked, Uneven.PeakOffDeg);
				WorstTickClock = FMath::Max(WorstTickClock, Old.PeakOffDeg);
				AddInfo(FString::Printf(TEXT("%s | %s | %s | rider, switched off %.2f deg at %.1f deg/s; frame-locked %.2f at %.1f; unlocked %.2f at %.1f; unlocked on the tick's clock %.2f at %.1f"), Course.Name, Seam.Name,
					UnlockedFrameNames[Rate], Was.PeakOffDeg, Was.RateDegPerSec, Even.PeakOffDeg, Even.RateDegPerSec, Uneven.PeakOffDeg, Uneven.RateDegPerSec, Old.PeakOffDeg, Old.RateDegPerSec));
				// The pose it steers by is most of a frame old: that much is lost, and no more.
				TestTrue(Say(Seam.Name, FString::Printf(TEXT("the rider as level as frame-locked, but for the pose's age (%.2f against %.2f deg)"), Uneven.PeakOffDeg, Even.PeakOffDeg)),
					Uneven.PeakOffDeg <= Even.PeakOffDeg + 0.7);
				// (Half a degree a second more than it was held to: frame-locked, the speed cap now slows at a break's
				// first sign, which a pose most of a frame old shows that much later.)
				TestTrue(Say(Seam.Name, FString::Printf(TEXT("the rider turned as gently (%.1f against %.1f deg/s)"), Uneven.RateDegPerSec, Even.RateDegPerSec)), Uneven.RateDegPerSec <= Even.RateDegPerSec + 2.5);
				TestTrue(Say(Seam.Name, FString::Printf(TEXT("the rider far better off than with the switch off (%.2f against %.2f deg)"), Uneven.PeakOffDeg, Was.PeakOffDeg)), Uneven.PeakOffDeg <= 0.4 * Was.PeakOffDeg);
			}
			// What the fault was, kept measured: the same frames and poses with the law on the tick's clock.
			TestTrue(Say(TEXT("the course"), FString::Printf(TEXT("(on the tick's clock the ground is not followed: the rider %.2f deg off against %.2f)"), WorstTickClock, WorstUnlocked)), WorstTickClock >= 2.5 * WorstUnlocked);
		}
	}
	return true;
}

// Catches: the same law unlocked losing what the ground gate is for. A base that is braked, or knocked, is still not
// taken for ground when the speed is measured from poses most of a frame old on their own uneven times: the stop
// from speed is left to pass as it is frame-locked (a speed between two poses shows a braking a pose late, which is
// why the gate takes its last step back when it stops believing), and so is a knock, but for the answer to what
// little of it is followed coming a frame later.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunUnlockedStopAndKnock,
	"Ramms.Level.DryRun.UnlockedSteppingStopAndKnock",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunUnlockedStopAndKnock::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	for (const FKit& Kit : Kits())
	{
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			// A stop from 60 cm/s on the grade, the body pitching 1.3 deg with it (StopFromSpeed's).
			{
				FOptions Options = MakeOptions(Kit, 0);
				Options.FrameSeconds = UnlockedFrameSeconds[Rate];
				Options.SpeedCmS = 60.0;
				Options.DriveLagSeconds = 0.1;
				Options.PitchPerAccelDeg = BodyPitchPerAccel;
				Options.MaxSeconds = 9.0;
				Options.bKeepTrace = true;
				Options.Unlocked.bOn = true;
				Options.Events.Add(MakeEvent(5.0, EEvent::SetSpeed, 0.0, 0.0));
				FOptions OffOptions = Options;
				OffOptions.bEnabledAtStart = false;
				FOptions LockedOptions = Options;
				LockedOptions.Unlocked.bOn = false;
				const FGround Course = Plane(Kit.GradeDeg, 0.0);
				const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
				const FSwing  Off = SwingBetween(DryRun::Run(Kit.Robot, Course, OffOptions), 5.0, 8.0, Options.FrameSeconds);
				const FSwing  Locked = SwingBetween(DryRun::Run(Kit.Robot, Course, LockedOptions), 5.0, 8.0, Options.FrameSeconds);
				const FSwing  Swing = SwingBetween(Result, 5.0, 8.0, Options.FrameSeconds);
				AddInfo(FString::Printf(TEXT("%s | stop from 60 cm/s on %.0f deg | %s unlocked | swing %.2f deg at %.1f deg/s; frame-locked %.2f at %.1f; switched off %.2f at %.1f"), Kit.Name, Kit.GradeDeg, UnlockedFrameNames[Rate],
					Swing.RangeDeg, Swing.RateDegPerSec, Locked.RangeDeg, Locked.RateDegPerSec, Off.RangeDeg, Off.RateDegPerSec));
				ExpectSound(*this, Kit, 0, Result);
				const FString Case = FString::Printf(TEXT("%s, %s unlocked, a stop from speed"), Kit.Name, UnlockedFrameNames[Rate]);
				TestTrue(FString::Printf(TEXT("%s: the knock is left to pass (%.2f against %.2f deg switched off)"), *Case, Swing.RangeDeg, Off.RangeDeg), Swing.RangeDeg <= 1.2 * Off.RangeDeg);
				TestTrue(FString::Printf(TEXT("%s: the base moves no faster for the legs (%.1f against %.1f deg/s)"), *Case, Swing.RateDegPerSec, Off.RateDegPerSec), Swing.RateDegPerSec <= 1.2 * Off.RateDegPerSec);
				TestTrue(FString::Printf(TEXT("%s: level before the stop"), *Case), PeakTiltBetween(Result, Kit.Tuning, 4.0, 5.0) <= 0.5);
				TestTrue(FString::Printf(TEXT("%s: level after it"), *Case), Result.SteadyResidualDeg <= 0.3);
				TestTrue(FString::Printf(TEXT("%s: no rocking after it"), *Case), Result.SteadyTiltRangeDeg <= 0.2);
			}
			// A 1.5 deg knock, standing and at a crawl (KnockStandingAndOnTheMove's).
			for (const double Speed : { 0.0, 10.0 })
			{
				FOptions Options = MakeOptions(Kit, 0);
				Options.FrameSeconds = UnlockedFrameSeconds[Rate];
				Options.SpeedCmS = Speed;
				Options.MaxSeconds = 8.0;
				Options.bKeepTrace = true;
				Options.Unlocked.bOn = true;
				FDryEvent Kick = MakeEvent(3.0, EEvent::AttitudeKick, 0.15);
				Kick.PlaneDeg = FVector2D(-1.5, 0.0);
				Options.Events.Add(Kick);
				FOptions OffOptions = Options;
				OffOptions.bEnabledAtStart = false;
				const FGround Flat = Plane(0.0, 0.0);
				const FResult Result = DryRun::Run(Kit.Robot, Flat, Options);
				const FSwing  Off = SwingBetween(DryRun::Run(Kit.Robot, Flat, OffOptions), 3.0, 6.0, Options.FrameSeconds);
				const FSwing  Swing = SwingBetween(Result, 3.0, 6.0, Options.FrameSeconds);
				AddInfo(FString::Printf(TEXT("%s | a 1.5 deg knock at %.0f cm/s | %s unlocked | swing %.2f deg at %.1f deg/s; switched off %.2f at %.1f"), Kit.Name, Speed, UnlockedFrameNames[Rate], Swing.RangeDeg, Swing.RateDegPerSec,
					Off.RangeDeg, Off.RateDegPerSec));
				ExpectSound(*this, Kit, 0, Result);
				const FString Case = FString::Printf(TEXT("%s, %s unlocked, a knock at %.0f cm/s"), Kit.Name, UnlockedFrameNames[Rate], Speed);
				TestTrue(FString::Printf(TEXT("%s: left to pass (%.2f against %.2f deg switched off)"), *Case, Swing.RangeDeg, Off.RangeDeg), Swing.RangeDeg <= 1.4 * Off.RangeDeg);
				TestTrue(FString::Printf(TEXT("%s: level after it"), *Case), Result.SteadyResidualDeg <= 0.3);
			}
		}
	}
	return true;
}

namespace RammsLevelDryRunTests
{
	/** The drive-speed cap as it was: slowing all the way along a slope for the slope's end, falling straight to
	 *  what it wanted however far under the robot's speed that was, taking the speed there is for the speed asked,
	 *  and answering whatever the estimate did while its own braking pitched the body. Kept to measure against. */
	FOptions WithTheCapAsItWas(const FOptions& Options)
	{
		FOptions Out = Options;
		Out.bDriveSaysWhatIsAsked = false;
		Out.SpeedCap.bSlopeEnd = true;
		Out.SpeedCap.LeadCmS = 0.0;
		Out.SpeedCap.LeadShare = 0.0;
		Out.SpeedCap.OverspeedGain = 0.0;
		Out.SpeedCap.FallSeconds = 0.0;
		Out.SpeedCap.SteadyAskCmS2 = 1.0e9;
		Out.SpeedCap.UnsteadyAskCmS2 = 2.0e9;
		Out.SpeedCap.SteadySpeedCmS2 = 1.0e9;
		Out.SpeedCap.UnsteadySpeedCmS2 = 2.0e9;
		return Out;
	}

	/** The least and the most speed (cm/s, as the robot went: forwards or back) over a stretch of travel. */
	FVector2D SpeedsOver(const FResult& Result, double FromCm, double ToCm)
	{
		FVector2D Out(1.0e9, 0.0);
		for (const FSample& Sample : Result.Trace)
		{
			if (Sample.TravelCm >= FromCm && Sample.TravelCm <= ToCm)
			{
				Out.X = FMath::Min(Out.X, FMath::Abs(Sample.SpeedCmS));
				Out.Y = FMath::Max(Out.Y, FMath::Abs(Sample.SpeedCmS));
			}
		}
		return Out;
	}
} // namespace RammsLevelDryRunTests

// Catches: a speed cap that is only right on a drive that does as it is told. A real drive is a speed loop on a
// robot gravity pulls (FDriveLoop): over a crest the robot runs away downhill by half its speed again while the
// loop still holds the climb, and asked for much less than the speed it has the loop brakes it through what was
// asked. On that drive, at half and at full stick, over a 12 deg crest and up an 8 deg ramp to a landing: the cap
// never brings the robot near a stand (as it was it all but stopped it at the crest); a steady slope costs nothing
// (as it was, a third of the pace all the way up); the run-away over the crest is held to a fifth over what the
// stick asks; the rider through every seam better off than with the switch off, and through the crest within a
// bound. And reversing downhill at full stick from a stand, the speed does not hunt (as it was it swung by half).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunCapOnALoop,
	"Ramms.Level.DryRun.SpeedCapOnADriveThatIsALoop",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunCapOnALoop::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;

	const FKit				   Kit = FollowingOnly(QuickOnFour());
	const TArray<FReachCourse> Courses = ReachCourses();
	if (!TestTrue(TEXT("the courses"), Courses.Num() >= 4))
	{
		return false;
	}
	// The rider through the crest: at half stick inside the tilt-rate limit; at full stick, where the robot still
	// runs a little away before the drive answers, within these (deg, deg/s).
	const double CrestPeakDeg[2] = { 1.5, 2.6 };
	const double CrestRateDegPerSec[2] = { RiderRateLimitDegPerSec, 7.5 };
	for (int32 StickIndex = 0; StickIndex < 2; ++StickIndex)
	{
		const double Stick = StickIndex == 0 ? 30.0 : 60.0;
		for (const int32 CourseIndex : { 0, 3 })
		{
			const bool			bCrest = CourseIndex == 0;
			const FReachCourse& Course = Courses[CourseIndex];
			// Levelled where it stands for five seconds, then driven.
			FOptions Options = MakeOptions(Kit, 0);
			Options.SpeedCmS = 0.0;
			Options.DistanceCm = Course.DistanceCm;
			Options.MaxSeconds = 90.0;
			Options.PitchPerAccelDeg = BodyPitchPerAccel;
			Options.Drive.bOn = true;
			Options.bKeepTrace = true;
			Options.Events.Add(MakeEvent(5.0, EEvent::SetSpeed, 0.0, Stick));
			FOptions OffOptions = Options;
			OffOptions.bEnabledAtStart = false;
			const FResult Off = DryRun::Run(Kit.Robot, Course.Ground, OffOptions);
			const FResult Was = DryRun::Run(Kit.Robot, Course.Ground, WithTheCapAsItWas(Options));
			const FResult Result = DryRun::Run(Kit.Robot, Course.Ground, Options);
			const FString Case = FString::Printf(TEXT("%s at %.0f cm/s on a drive that is a loop"), Course.Name, Stick);
			const auto	  Say = [&Case](const TCHAR* Where, const FString& What) { return FString::Printf(TEXT("%s, %s: %s"), *Case, Where, *What); };
			ExpectSound(*this, Kit, 0, Result);
			TestTrue(Say(TEXT("the course"), TEXT("driven to its end")), Result.TravelCm >= Course.DistanceCm && Off.TravelCm >= Course.DistanceCm);

			// From the first seam on (the start from a stand is the drive's own).
			const double	From = Course.Seams[0].AtCm - 20.0;
			const FVector2D Speeds = SpeedsOver(Result, From, Course.DistanceCm);
			const FVector2D WasSpeeds = SpeedsOver(Was, From, Course.DistanceCm);
			const FVector2D OffSpeeds = SpeedsOver(Off, From, Course.DistanceCm);
			AddInfo(FString::Printf(TEXT("%s | speed %.0f to %.0f cm/s; as the cap was %.0f to %.0f; switched off %.0f to %.0f | %.1f s, as it was %.1f, switched off %.1f"), *Case, Speeds.X, Speeds.Y, WasSpeeds.X, WasSpeeds.Y, OffSpeeds.X,
				OffSpeeds.Y, Result.Seconds - 5.0, Was.Seconds - 5.0, Off.Seconds - 5.0));
			TestTrue(Say(TEXT("the course"), FString::Printf(TEXT("the robot is never brought near a stand (%.1f cm/s at least)"), Speeds.X)), Speeds.X >= 0.35 * Stick);
			if (bCrest && StickIndex == 0)
			{
				TestTrue(Say(TEXT("the course"), FString::Printf(TEXT("(as the cap was, the robot was all but stopped past the crest: %.1f cm/s)"), WasSpeeds.X)), WasSpeeds.X <= 0.25 * Stick);
			}

			// The steady climb, clear of the breaks at either end of it:
			// nothing asked of the drive.
			const FVector2D Climb = bCrest ? FVector2D(350.0, 600.0) : FVector2D(500.0, 700.0);
			const FPassage	Steady = PassageBetween(Result, Kit.Tuning, Climb.X, Climb.Y, Options.FrameSeconds);
			const FPassage	WasSteady = PassageBetween(Was, Kit.Tuning, Climb.X, Climb.Y, Options.FrameSeconds);
			TestTrue(Say(TEXT("the steady climb"), FString::Printf(TEXT("costs nothing: %.1f cm/s, the scale %.2f at least"), Steady.MeanSpeedCmS, Steady.LeastScale)), Steady.MeanSpeedCmS >= 0.98 * Stick && Steady.LeastScale >= 0.99);
			TestTrue(Say(TEXT("the steady climb"), FString::Printf(TEXT("the rider level (%.2f deg)"), Steady.PeakDeg)), Steady.PeakDeg <= 0.6);
			if (StickIndex == 1)
			{
				TestTrue(Say(TEXT("the steady climb"), FString::Printf(TEXT("(as the cap was, it cost a good part of the pace: %.1f cm/s)"), WasSteady.MeanSpeedCmS)), WasSteady.MeanSpeedCmS <= 0.8 * Stick);
			}

			for (const FSeam& Seam : Course.Seams)
			{
				const double	  SeamFrom = Seam.AtCm - 20.0;
				const double	  SeamTo = Seam.AtCm + Seam.LengthCm + OnFourSpanCm + 100.0;
				const FRiderSwing On = RiderOver(Result, SeamFrom, SeamTo, Options.FrameSeconds);
				const FRiderSwing Before = RiderOver(Was, SeamFrom, SeamTo, Options.FrameSeconds);
				const FRiderSwing None = RiderOver(Off, SeamFrom, SeamTo, Options.FrameSeconds);
				AddInfo(FString::Printf(TEXT("%s | %s | rider %.2f deg at %.1f deg/s; as the cap was %.2f at %.1f; switched off %.2f at %.1f"), *Case, Seam.Name, On.PeakOffDeg, On.RateDegPerSec, Before.PeakOffDeg, Before.RateDegPerSec,
					None.PeakOffDeg, None.RateDegPerSec));
				TestTrue(Say(Seam.Name, FString::Printf(TEXT("the rider far better off than with the switch off (%.2f against %.2f deg)"), On.PeakOffDeg, None.PeakOffDeg)), On.PeakOffDeg <= 0.4 * None.PeakOffDeg);
				TestTrue(Say(Seam.Name, FString::Printf(TEXT("the rider turned more gently than with the switch off (%.1f against %.1f deg/s)"), On.RateDegPerSec, None.RateDegPerSec)), On.RateDegPerSec <= 0.8 * None.RateDegPerSec);
			}
			if (bCrest)
			{
				// Over the crest the robot runs away by gravity; the cap asks less of the drive by as much again as
				// it is past what the stick asks, and the run-away is held.
				const FVector2D	  Over = SpeedsOver(Result, 600.0, 900.0);
				const FVector2D	  OffOver = SpeedsOver(Off, 600.0, 900.0);
				const FRiderSwing Crest = RiderOver(Result, 630.0, 650.0 + OnFourSpanCm + 100.0, Options.FrameSeconds);
				TestTrue(Say(TEXT("the crest"), FString::Printf(TEXT("(switched off the robot runs away down the far side: %.0f cm/s)"), OffOver.Y)), OffOver.Y >= 1.4 * Stick);
				TestTrue(Say(TEXT("the crest"), FString::Printf(TEXT("the run-away is held (%.0f cm/s)"), Over.Y)), Over.Y <= 1.2 * Stick);
				TestTrue(Say(TEXT("the crest"), FString::Printf(TEXT("the rider within %.1f deg (%.2f)"), CrestPeakDeg[StickIndex], Crest.PeakOffDeg)), Crest.PeakOffDeg <= CrestPeakDeg[StickIndex]);
				TestTrue(Say(TEXT("the crest"), FString::Printf(TEXT("the rider turned within %.1f deg/s (%.1f)"), CrestRateDegPerSec[StickIndex], Crest.RateDegPerSec)), Crest.RateDegPerSec <= CrestRateDegPerSec[StickIndex]);
			}
		}
	}

	// Reverse, downhill, at full stick, from standing levelled on 6 deg. The drive's own start overshoots (it does
	// switched off as well); after that the speed is the stick's, and steady.
	{
		FOptions Options = MakeOptions(Kit, 0);
		Options.SpeedCmS = 0.0;
		Options.StartXY = FVector2D(2000.0, 0.0);
		Options.MaxSeconds = 13.0;
		Options.PitchPerAccelDeg = BodyPitchPerAccel;
		Options.Drive.bOn = true;
		Options.bKeepTrace = true;
		Options.Events.Add(MakeEvent(5.0, EEvent::SetSpeed, 0.0, -60.0));
		FOptions OffOptions = Options;
		OffOptions.bEnabledAtStart = false;
		const FGround Slope = Plane(6.0, 0.0);
		const FResult Off = DryRun::Run(Kit.Robot, Slope, OffOptions);
		const FResult Was = DryRun::Run(Kit.Robot, Slope, WithTheCapAsItWas(Options));
		const FResult Result = DryRun::Run(Kit.Robot, Slope, Options);
		// Least and most speed from a second after the stick, and from three seconds after it.
		const auto Range = [](const FResult& Run, double FromSeconds) {
			FVector2D Out(1.0e9, 0.0);
			for (const FSample& Sample : Run.Trace)
			{
				if (Sample.Seconds >= FromSeconds)
				{
					Out.X = FMath::Min(Out.X, FMath::Abs(Sample.SpeedCmS));
					Out.Y = FMath::Max(Out.Y, FMath::Abs(Sample.SpeedCmS));
				}
			}
			return Out;
		};
		const FVector2D Early = Range(Result, 6.0), Late = Range(Result, 8.0);
		const FVector2D WasEarly = Range(Was, 6.0), WasLate = Range(Was, 8.0);
		const FVector2D OffEarly = Range(Off, 6.0);
		AddInfo(FString::Printf(TEXT("reverse downhill at full stick | speed %.0f to %.0f cm/s from a second after the stick, %.0f to %.0f from three; as the cap was %.0f to %.0f, %.0f to %.0f; switched off %.0f to %.0f"), Early.X, Early.Y, Late.X,
			Late.Y, WasEarly.X, WasEarly.Y, WasLate.X, WasLate.Y, OffEarly.X, OffEarly.Y));
		ExpectSound(*this, Kit, 0, Result);
		TestTrue(FString::Printf(TEXT("reverse downhill: the speed does not hunt after the start (%.0f to %.0f cm/s)"), Early.X, Early.Y), Early.X >= 0.85 * 60.0);
		TestTrue(FString::Printf(TEXT("reverse downhill: ... no faster than the drive's own start, switched off (%.0f against %.0f cm/s)"), Early.Y, OffEarly.Y), Early.Y <= OffEarly.Y + 2.0);
		TestTrue(FString::Printf(TEXT("reverse downhill: steady at the stick's speed three seconds on (%.0f to %.0f cm/s)"), Late.X, Late.Y), Late.X >= 57.0 && Late.Y <= 63.0);
		TestTrue(FString::Printf(TEXT("reverse downhill: (as the cap was, the speed swung by half: down to %.0f cm/s)"), WasEarly.X), WasEarly.X <= 0.6 * 60.0);
		TestTrue(TEXT("reverse downhill: the rider level"), Result.SteadyRiderResidualDeg <= 0.3);
	}
	return true;
}

namespace RammsLevelDryRunTests
{
	/**
	 * A made-up robot of the drive-pairs kind with slow lifts and a top speed: two drive legs each on its own lift
	 * (roll reaches them), a rear pair on one slow lift 70 cm aft, the front pair carried 8 cm clear. Its lifts turn a
	 * pitch correction at under 6 deg/s; it is driven at 60 and at 120 cm/s. Roll is capped at 6 deg.
	 */
	FKit SlowLifts()
	{
		FKit Kit;
		Kit.Name = TEXT("slow lifts");
		const auto Leg = [](double X, double Y, double Half, double Radius, double Z, double Rate, bool bRoll) {
			FRobotLeg Out;
			Out.XY = FVector2D(X, Y);
			Out.PairHalfTrackCm = Half;
			Out.RadiusCm = Radius;
			Out.StanceZ = Z;
			Out.MinZ = Z - 12.0;
			Out.MaxZ = Z + 12.0;
			Out.MaxRateCmS = Rate;
			Out.MaxAccelCmS2 = 200.0;
			Out.bRoll = bRoll;
			return Out;
		};
		Kit.Robot.Legs.Add(Leg(0.0, -30.0, 0.0, 17.0, -15.0, 9.0, true));
		Kit.Robot.Legs.Add(Leg(0.0, 30.0, 0.0, 17.0, -15.0, 9.0, true));
		Kit.Robot.Legs.Add(Leg(-70.0, 0.0, 11.0, 7.5, -24.5, 6.0, false));
		Kit.Robot.Legs.Add(Leg(30.0, 0.0, 18.0, 7.5, -16.5, 10.0, false));
		Kit.Robot.CentreOfMass = FVector(-17.0, 0.0, 20.0);
		Kit.Tuning.MaxCorrectionDeg = FVector2D(10.0, 6.0);
		Kit.Tuning.MaxCorrectionRateDegPerSec = 8.0;
		Kit.SpeedCmS = 120.0;
		Kit.SpanCm = 60.0;
		return Kit;
	}

	/** Up Grade for 6 m from 300 cm, down again for 6 m: a crest of twice Grade at 900 cm. */
	FGround CrestOf(double GradeDeg)
	{
		const FVector2D Knots[3] = { { 300.0, 0.0 }, { 900.0, Rise(600.0, GradeDeg) }, { 1500.0, 0.0 } };
		return Profile(MakeArrayView(Knots));
	}

	/** Levelled where it stands for three seconds, then driven at Stick over a course 19 m long, on a drive that is a
	 *  speed loop and with a body that pitches as it is pushed. */
	FOptions DrivenOver(const FKit& Kit, double Stick)
	{
		FOptions Options = MakeOptions(Kit, 0);
		Options.SpeedCmS = 0.0;
		Options.DistanceCm = 1900.0;
		Options.MaxSeconds = 90.0;
		Options.PitchPerAccelDeg = BodyPitchPerAccel;
		Options.Drive.bOn = true;
		Options.bKeepTrace = true;
		Options.Events.Add(MakeEvent(3.0, EEvent::SetSpeed, 0.0, Stick));
		return Options;
	}
} // namespace RammsLevelDryRunTests

// Catches: a robot levelled on a slope it takes fast being thrown further over the crest than one that was never
// levelled. The correction it carries up the slope is wrong by twice the slope on the far side, and legs that cannot
// take it back out while the crest passes under them leave the rider with the difference (a robot on slow lifts at
// full stick over a 12 deg crest: half as far again as switched off). With the crest rule (FCarryTuning) the robot
// carries no more correction than its legs can take back out in that time, slowed as the speed cap will slow it: over
// every crest it is then thrown no further than switched off, at the price of a slope taken at full speed being left
// partly in the base; at half that speed, where the cap can slow the robot on the break, the rule asks nothing and
// the slope is held level as before. No look-ahead: the tilt RATE over a sharp crest at full stick stays far over the
// rider's limit, as it is switched off (reported, not bounded).
// (The rule is OFF as built, FCarryTuning::bCrest: it cannot tell a slope that ends in a crest from a ramp that ends on
// a landing, and what it leaves in the base on every fast slope cost more in the simulator than it saved at a crest.
// It is switched on here: this is what it does, kept to be measured against.)
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunCarriedAtSpeed,
	"Ramms.Level.DryRun.CarriedAtSpeed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunCarriedAtSpeed::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;
	for (int32 Which = 0; Which < 2; ++Which)
	{
		// The slow lifts at their full speed; the quick legs at twice theirs (where the rule first binds on them).
		const FKit	 Kit = Which == 0 ? SlowLifts() : QuickOnFour();
		const double Span = Which == 0 ? 70.0 : OnFourSpanCm;
		for (const double GradeDeg : { 2.5, 4.0, 6.0 })
		{
			const FGround Ground = CrestOf(GradeDeg);
			FOptions	  Options = DrivenOver(Kit, 120.0);
			Options.Carry.bCrest = true;
			FOptions OffOptions = Options;
			OffOptions.bEnabledAtStart = false;
			FOptions NoRule = Options;
			NoRule.Carry.bCrest = false;
			const FResult	  Result = DryRun::Run(Kit.Robot, Ground, Options);
			const FResult	  Without = DryRun::Run(Kit.Robot, Ground, NoRule);
			const FResult	  Off = DryRun::Run(Kit.Robot, Ground, OffOptions);
			const double	  From = 880.0, To = 900.0 + Span + 120.0;
			const FRiderSwing On = RiderOver(Result, From, To, Options.FrameSeconds);
			const FRiderSwing Was = RiderOver(Without, From, To, Options.FrameSeconds);
			const FRiderSwing None = RiderOver(Off, From, To, Options.FrameSeconds);
			const FPassage	  Climb = PassageBetween(Result, Kit.Tuning, 550.0, 850.0, Options.FrameSeconds);
			const FString	  Case = FString::Printf(TEXT("%s at 120 cm/s over a %.0f deg crest"), Kit.Name, 2.0 * GradeDeg);
			AddInfo(FString::Printf(TEXT("%s | rider %.2f deg at %.1f deg/s; without the rule %.2f at %.1f; switched off %.2f at %.1f | on the climb the base %.2f deg off at %.0f cm/s | %.1f s, without the rule %.1f, switched off %.1f"), *Case,
				On.PeakOffDeg, On.RateDegPerSec, Was.PeakOffDeg, Was.RateDegPerSec, None.PeakOffDeg, None.RateDegPerSec, Climb.PeakDeg, Climb.MeanSpeedCmS, Result.Seconds - 3.0, Without.Seconds - 3.0, Off.Seconds - 3.0));
			ExpectSound(*this, Kit, 0, Result);
			TestTrue(Case + TEXT(": driven to its end"), Result.TravelCm >= Options.DistanceCm && Off.TravelCm >= Options.DistanceCm);
			TestTrue(Case + FString::Printf(TEXT(": the rider thrown no further than switched off (%.2f against %.2f deg)"), On.PeakOffDeg, None.PeakOffDeg), On.PeakOffDeg <= None.PeakOffDeg + 0.1);
			TestTrue(Case + FString::Printf(TEXT(": ... nor turned faster (%.1f against %.1f deg/s)"), On.RateDegPerSec, None.RateDegPerSec), On.RateDegPerSec <= None.RateDegPerSec + 0.5);
			TestTrue(Case + FString::Printf(TEXT(": the slope is never left whole in the base (%.2f of %.1f deg)"), Climb.PeakDeg, GradeDeg), Climb.PeakDeg <= GradeDeg - 0.5);
			if (GradeDeg >= (Which == 0 ? 4.0 : 6.0))
			{
				TestTrue(Case + FString::Printf(TEXT(": (without the rule the rider is thrown further than switched off: %.2f against %.2f deg)"), Was.PeakOffDeg, None.PeakOffDeg), Was.PeakOffDeg >= None.PeakOffDeg + 0.3);
				TestTrue(Case + TEXT(": the correction is held back for the speed on the climb, and that is not being at reach"), Result.Trace.ContainsByPredicate([](const FSample& Sample) {
					return Sample.TravelCm > 600.0 && Sample.TravelCm < 850.0 && Sample.Out.bEased && Sample.Out.Status == EStatus::Levelling;
				}));
			}
			TestTrue(Case + FString::Printf(TEXT(": the pace kept (%.1f s against %.1f switched off)"), Result.Seconds - 3.0, Off.Seconds - 3.0), Result.Seconds - 3.0 <= 1.25 * (Off.Seconds - 3.0));
		}
	}
	// At half the slow lifts' speed the cap can slow the robot on the break: nothing is held back, the 6 deg slope is
	// level all the way up, and the crest is no worse than switched off all the same.
	{
		const FKit	   Kit = SlowLifts();
		const FGround  Ground = CrestOf(6.0);
		FOptions	   Options = DrivenOver(Kit, 60.0);
		Options.Carry.bCrest = true;
		FOptions OffOptions = Options;
		OffOptions.bEnabledAtStart = false;
		const FResult	  Result = DryRun::Run(Kit.Robot, Ground, Options);
		const FResult	  Off = DryRun::Run(Kit.Robot, Ground, OffOptions);
		const FPassage	  Climb = PassageBetween(Result, Kit.Tuning, 550.0, 850.0, Options.FrameSeconds);
		const FRiderSwing On = RiderOver(Result, 880.0, 1090.0, Options.FrameSeconds);
		const FRiderSwing None = RiderOver(Off, 880.0, 1090.0, Options.FrameSeconds);
		AddInfo(FString::Printf(TEXT("slow lifts at 60 cm/s over a 12 deg crest | on the climb the base %.2f deg off | rider over the crest %.2f deg at %.1f deg/s; switched off %.2f at %.1f"), Climb.PeakDeg, On.PeakOffDeg, On.RateDegPerSec,
			None.PeakOffDeg, None.RateDegPerSec));
		TestTrue(FString::Printf(TEXT("at 60 cm/s: the 6 deg slope held level (%.2f deg)"), Climb.PeakDeg), Climb.PeakDeg <= 0.4);
		TestTrue(TEXT("at 60 cm/s: nothing held back for the speed on the climb"), !Result.Trace.ContainsByPredicate([](const FSample& Sample) { return Sample.TravelCm > 600.0 && Sample.TravelCm < 850.0 && Sample.Out.bEased; }));
		TestTrue(FString::Printf(TEXT("at 60 cm/s: the crest no worse than switched off (%.2f against %.2f deg)"), On.PeakOffDeg, None.PeakOffDeg), On.PeakOffDeg <= None.PeakOffDeg + 0.1);
	}
	return true;
}

// Catches: a speed cap that hears of a grade break only through two low-passes (a third of a second, half a span at
// full speed), and one that answers things that are not ground. The first sign of a break is read where it shows
// first and the drive slowed from then: the rider is turned more gently onto a grade than with that term off, at no
// lower a speed than a first sign is trusted with. And a start from rest, whose drive overshoots what its stick asks
// as it arrives, costs nothing (it took the scale to two thirds for a third of a second), nor does a speed that
// wanders about what is asked at a steady pace.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunCapFirstSign,
	"Ramms.Level.DryRun.SpeedCapFirstSignAndStarts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunCapFirstSign::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;
	for (int32 Which = 0; Which < 2; ++Which)
	{
		// (The crest rule off: the base is levelled all the way, and what the cap does for a break is all there is.)
		const FKit	   Kit = FollowingOnly(Which == 0 ? SlowLifts() : QuickOnFour());
		const double   Span = Which == 0 ? 70.0 : OnFourSpanCm;
		const FGround  Ground = CrestOf(4.0);
		const FOptions Options = DrivenOver(Kit, 120.0);
		FOptions	   Late = Options;
		Late.SpeedCap.EarlyHeldSpans = 0.0;
		const FResult	  Result = DryRun::Run(Kit.Robot, Ground, Options);
		const FResult	  Before = DryRun::Run(Kit.Robot, Ground, Late);
		const FRiderSwing On = RiderOver(Result, 280.0, 300.0 + Span + 120.0, Options.FrameSeconds);
		const FRiderSwing Was = RiderOver(Before, 280.0, 300.0 + Span + 120.0, Options.FrameSeconds);
		const FVector2D	  Speeds = SpeedsOver(Result, 280.0, 300.0 + Span + 120.0);
		AddInfo(FString::Printf(TEXT("%s at 120 cm/s onto 4 deg | rider %.2f deg at %.1f deg/s; without the first sign %.2f at %.1f | least speed %.0f cm/s"), Kit.Name, On.PeakOffDeg, On.RateDegPerSec, Was.PeakOffDeg, Was.RateDegPerSec, Speeds.X));
		TestTrue(FString::Printf(TEXT("%s onto 4 deg: the rider turned more gently for the first sign (%.1f against %.1f deg/s)"), Kit.Name, On.RateDegPerSec, Was.RateDegPerSec), On.RateDegPerSec <= Was.RateDegPerSec - 0.3);
		TestTrue(FString::Printf(TEXT("%s onto 4 deg: ... and left no further off (%.2f against %.2f deg)"), Kit.Name, On.PeakOffDeg, Was.PeakOffDeg), On.PeakOffDeg <= Was.PeakOffDeg + 0.02);
		TestTrue(FString::Printf(TEXT("%s onto 4 deg: never brought near a stand (%.0f cm/s)"), Kit.Name, Speeds.X), Speeds.X >= 0.45 * 120.0);

		// Starts from rest on the flat, and the cruise after, with a little noise on the attitude.
		for (const double Stick : { 30.0, 60.0, 120.0 })
		{
			FOptions Start = MakeOptions(Kit, 0);
			Start.SpeedCmS = 0.0;
			Start.MaxSeconds = 12.0;
			Start.PitchPerAccelDeg = BodyPitchPerAccel;
			Start.Drive.bOn = true;
			Start.NoiseDeg = 0.02;
			Start.bKeepTrace = true;
			Start.Events.Add(MakeEvent(3.0, EEvent::SetSpeed, 0.0, Stick));
			FOptions AsItWas = Start;
			AsItWas.SpeedCap.OverspeedSettleSeconds = 0.0;
			AsItWas.SpeedCap.OverspeedBandShare = 0.0;
			AsItWas.SpeedCap.OverspeedBandCmS = 0.0;
			AsItWas.SpeedCap.OverspeedGain = 2.0;
			const FResult Run = DryRun::Run(Kit.Robot, Plane(0.0, 0.0), Start);
			const FResult Old = DryRun::Run(Kit.Robot, Plane(0.0, 0.0), AsItWas);
			AddInfo(FString::Printf(TEXT("%s, a start to %.0f cm/s | least scale %.3f; as the over-speed term was %.3f"), Kit.Name, Stick, Run.MinSpeedScale, Old.MinSpeedScale));
			TestTrue(FString::Printf(TEXT("%s, a start to %.0f cm/s and the cruise after: nothing asked of the drive (%.3f)"), Kit.Name, Stick, Run.MinSpeedScale), Run.MinSpeedScale >= 0.999);
			TestTrue(FString::Printf(TEXT("%s, a start to %.0f cm/s: (as it was, the start's overshoot took the scale to %.2f)"), Kit.Name, Stick, Old.MinSpeedScale), Old.MinSpeedScale <= 0.85);
		}
	}
	return true;
}

// Catches: roll carried at speed by a robot that a roll correction stands on three points. Both drive wheels and one
// wheel of the rear pair on the ground, the drive wheels unequally loaded: braked hard there, such a robot swings
// about its heavier wheel (a simulator's finding; this dry run has no yaw and cannot show the swing, only which
// wheels are down when). So on this kind of robot roll is levelled standing and at a crawl and eased out as the speed
// rises: driven off across a 5 deg fall the roll correction is walked out within the tilt policy while the drive is
// held to the speed the roll still carried allows, from 40 cm/s on the robot is on all four wheels, it is on all four
// through a hard stop and for as long as the base is settling after, and it levels again standing. Roll reaches every
// leg of a six-wheeler: it carries roll at any speed, as before.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunRollAtACrawl,
	"Ramms.Level.DryRun.RollCarriedAtACrawlOnly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunRollAtACrawl::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;
	const FKit	  Kit = SlowLifts();
	const FGround Fall = Plane(0.0, 5.0);
	for (const bool bRule : { true, false })
	{
		// Levelled standing for 5 s; full stick for 7 s; the stick let go; 6 s standing.
		FOptions Options = MakeOptions(Kit, 0);
		Options.Carry.bThreePoint = bRule;
		// (Roll carried up to 15 cm/s and out from 40: this case's own speeds, so that its figures do not move with the
		// rule's as built.)
		Options.Carry.ThreePointFullCmS = 15.0;
		Options.Carry.ThreePointNoneCmS = 40.0;
		Options.SpeedCmS = 0.0;
		Options.MaxSeconds = 18.0;
		Options.PitchPerAccelDeg = BodyPitchPerAccel;
		Options.Drive.bOn = true;
		Options.bKeepTrace = true;
		Options.Events.Add(MakeEvent(5.0, EEvent::SetSpeed, 0.0, 120.0));
		Options.Events.Add(MakeEvent(12.0, EEvent::SetSpeed, 0.0, 0.0));
		const FResult Result = DryRun::Run(Kit.Robot, Fall, Options);
		ExpectSound(*this, Kit, 0, Result);
		// Standing before the stick; while faster than 40 cm/s; from the stick let go until half a second after the
		// robot stands; at the end.
		double StandingRoll = 0.0, StandingOff = 0.0, FastRoll = 0.0, StopRoll = 0.0, WalkRate = 0.0, FastestWithRoll = 0.0, StoodAt = -1.0;
		int32  FastFloating = 0, StopFloating = 0;
		for (int32 Index = 1; Index < Result.Trace.Num(); ++Index)
		{
			const FSample& Sample = Result.Trace[Index];
			const double   Roll = FMath::Abs(Sample.Out.AppliedDeg.Y);
			if (Sample.Seconds > 4.0 && Sample.Seconds < 5.0)
			{
				StandingRoll = FMath::Max(StandingRoll, Roll);
				StandingOff = FMath::Max(StandingOff, Inclination(Sample.TrueAttitudeDeg));
			}
			if (Sample.Seconds > 5.0 && Sample.Seconds < 12.0)
			{
				WalkRate = FMath::Max(WalkRate, (Sample.Out.RiderWantedDeg - Result.Trace[Index - 1].Out.RiderWantedDeg).Size() / Options.FrameSeconds);
				if (Roll > 0.5)
				{
					FastestWithRoll = FMath::Max(FastestWithRoll, FMath::Abs(Sample.SpeedCmS));
				}
				if (FMath::Abs(Sample.SpeedCmS) > 45.0)
				{
					FastRoll = FMath::Max(FastRoll, Roll);
					FastFloating = FMath::Max(FastFloating, Sample.Floating);
				}
			}
			if (Sample.Seconds >= 12.0)
			{
				if (StoodAt < 0.0 && FMath::Abs(Sample.SpeedCmS) < 2.0)
				{
					StoodAt = Sample.Seconds;
				}
				if (StoodAt < 0.0 || Sample.Seconds <= StoodAt + 0.2)
				{
					StopRoll = FMath::Max(StopRoll, Roll);
					StopFloating = FMath::Max(StopFloating, Sample.Floating);
				}
			}
		}
		const TCHAR* Name = bRule ? TEXT("roll at a crawl only") : TEXT("(the rule off)");
		AddInfo(FString::Printf(TEXT("%s | standing: roll carried %.2f deg, %.2f deg off | moving over 45 cm/s: roll carried %.2f deg, %d wheels in the air; fastest with half a degree of roll carried %.0f cm/s; the wanted attitude walked at %.2f deg/s | through the stop (stood at %.2f s): roll carried %.2f deg, %d wheels in the air | at the end %.2f deg off, roll carried %.2f"),
			Name, StandingRoll, StandingOff, FastRoll, FastFloating, FastestWithRoll, WalkRate, StoodAt, StopRoll, StopFloating, Result.FinalTiltDeg, Result.Trace.IsEmpty() ? 0.0 : FMath::Abs(Result.Trace.Last().Out.AppliedDeg.Y)));
		TestTrue(FString::Printf(TEXT("%s: standing, the fall is levelled (%.2f deg off, %.2f carried)"), Name, StandingOff, StandingRoll), StandingOff <= 0.3 && StandingRoll >= 4.5);
		TestTrue(FString::Printf(TEXT("%s: standing again at the end, levelled again (%.2f deg off)"), Name, Result.FinalTiltDeg), Result.FinalTiltDeg <= 0.3);
		if (bRule)
		{
			TestTrue(FString::Printf(TEXT("moving faster than 45 cm/s no roll is carried (%.3f deg)"), FastRoll), FastRoll <= 0.05);
			TestEqual(TEXT("... and no wheel of a pair that stands is in the air"), FastFloating, 0);
			TestTrue(FString::Printf(TEXT("the drive is held while roll is still carried (%.0f cm/s at most with half a degree on)"), FastestWithRoll), FastestWithRoll <= 45.0);
			TestTrue(FString::Printf(TEXT("the roll is walked out within the tilt policy (%.2f deg/s)"), WalkRate), WalkRate <= Kit.Tuning.TiltRateDegPerSec + 0.05);
			TestTrue(FString::Printf(TEXT("through the stop next to no roll comes back before the robot stands (%.3f deg, from a crawl on)"), StopRoll), StopRoll <= 0.35);
			TestEqual(TEXT("... the robot on all its wheels throughout"), StopFloating, 0);
			TestTrue(TEXT("the robot reached its speed once the roll was out"), SpeedsOver(Result, 0.0, 1.0e6).Y >= 110.0);
			TestTrue(TEXT("the roll left in the base at speed is not called being at reach"), Result.Trace.ContainsByPredicate([](const FSample& Sample) { return Sample.Seconds > 10.0 && Sample.Seconds < 12.0 && Sample.Out.bEased && Sample.Out.Status == EStatus::Levelling; }));
		}
		else
		{
			TestTrue(FString::Printf(TEXT("(with the rule off the roll is carried at full speed: %.2f deg, a wheel in the air)"), FastRoll), FastRoll >= 4.5 && FastFloating >= 1);
		}
	}
	// A six-wheeler, every leg of which roll reaches: nothing eased, at its full speed across the fall.
	{
		const FKit Six = QuickOnFour();
		FOptions   Options = MakeOptions(Six, 0);
		Options.SpeedCmS = 60.0;
		Options.MaxSeconds = 10.0;
		Options.bKeepTrace = true;
		const FResult Result = DryRun::Run(Six.Robot, Fall, Options);
		TestTrue(FString::Printf(TEXT("quick on four at 60 cm/s across the fall: levelled (%.2f deg off)"), Result.SteadyResidualDeg), Result.SteadyResidualDeg <= 0.3);
		TestTrue(TEXT("... nothing held back for the speed"), !Result.Trace.ContainsByPredicate([](const FSample& Sample) { return Sample.Out.bEased; }));
	}
	return true;
}

// Catches: a release that walks to the ground's attitude as the JOINTS read it. Joints that stand off their commands
// under the load make a plane of their own with no offset on at all; taken for ground, the walk back goes past the
// stance by that plane and the open-loop ramp then brings the base back. The release goes to the stance as the
// joints will stand in it: read while there were no offsets, at the start.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunReleaseOnSaggingJoints,
	"Ramms.Level.DryRun.ReleaseEndsOnTheStanceOfSaggingJoints",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunReleaseOnSaggingJoints::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;
	for (const FKit& Kit : { SlowLifts(), DrivePairs(), SixWheel() })
	{
		// Switched on where it has been standing a while (its joints settled under the load), off again at 10 s.
		FOptions Options = MakeOptions(Kit, 0);
		Options.SpeedCmS = 0.0;
		Options.MaxSeconds = 18.0;
		Options.SagCm = 0.6;
		Options.bKeepTrace = true;
		Options.bEnabledAtStart = false;
		FOptions OffOptions = Options;
		Options.Events.Add(MakeEvent(2.0, EEvent::Enable));
		Options.Events.Add(MakeEvent(10.0, EEvent::Disable));
		const FGround Ground = Plane(3.0, 0.0);
		const FResult Result = DryRun::Run(Kit.Robot, Ground, Options);
		const FResult Off = DryRun::Run(Kit.Robot, Ground, OffOptions);
		if (!TestTrue(TEXT("the runs"), !Result.Trace.IsEmpty() && !Off.Trace.IsEmpty()))
		{
			return false;
		}
		const double Stance = Off.Trace.Last().TrueAttitudeDeg.X;
		double		 Past = 0.0, SagDeg = 0.0;
		for (const FSample& Sample : Result.Trace)
		{
			SagDeg = FMath::Max(SagDeg, FMath::Abs(Sample.Out.SagDeg.X));
			if (Sample.Seconds > 10.0)
			{
				Past = FMath::Max(Past, FMath::Abs(Sample.TrueAttitudeDeg.X) - FMath::Abs(Stance));
			}
		}
		AddInfo(FString::Printf(TEXT("%s | the stance stands at %.2f deg on 3 deg (its joints make %.2f deg of their own); the release went %.3f deg past it and ended at %.2f"), Kit.Name, Stance, SagDeg, Past, Result.Trace.Last().TrueAttitudeDeg.X));
		ExpectSound(*this, Kit, 0, Result);
		TestTrue(FString::Printf(TEXT("%s: levelled before the release"), Kit.Name), PeakTiltBetween(Result, Kit.Tuning, 8.0, 10.0) <= 0.3);
		TestTrue(FString::Printf(TEXT("%s: the release ends off, on the stance"), Kit.Name), Result.FinalState == EState::Off && FMath::Abs(Result.Trace.Last().TrueAttitudeDeg.X - Stance) <= 0.02 && Result.FinalMaxOffsetCm == 0.0);
		// (The drive-pairs robot's joints stand off their commands one way with no offsets on and the other way
		// levelled, the load having moved onto other legs: nothing is taken off its walk, and it goes as far past
		// as it did.)
		TestTrue(FString::Printf(TEXT("%s: ... without going past it (%.3f deg)"), Kit.Name, Past), Past <= (Kit.Robot.Legs.Num() == 4 && Kit.SpanCm == 40.0 ? 0.15 : 0.04));
	}
	return true;
}

// Catches: a jogged linkage read as ground by the law. The centre legs' height jogged up and down under a robot
// levelled on a 6 deg grade, standing on its centre and rear legs: with who stands judged from the ground the stance
// rests on (as a plant gives it by the robot's centre of mass) the base stays within a quarter of a degree and the
// rider with it; judged from the lowest wheels in the chassis' frame the centre legs read as lifted, the base goes two
// degrees off and the seat adds as much again at the rider.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunJoggedLegs,
	"Ramms.Level.DryRun.JoggedLegsAreNotGround",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunJoggedLegs::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;
	for (const double Cm : { 3.0, -3.0 })
	{
		double Rider[2] = { 0.0, 0.0 }, Base[2] = { 0.0, 0.0 };
		for (int32 Which = 0; Which < 2; ++Which)
		{
			FKit Kit = QuickOnFour();
			Kit.Robot.bStanceRests = Which == 0;
			FOptions Options = MakeOptions(Kit, 0);
			Options.SpeedCmS = 0.0;
			Options.MaxSeconds = 14.0;
			Options.bKeepTrace = true;
			for (const int32 Leg : { 2, 3 })
			{
				Options.Events.Add(MakeEvent(6.0, EEvent::StanceRamp, 1.5, Cm, Leg));
				Options.Events.Add(MakeEvent(7.5, EEvent::StanceRamp, 1.5, -Cm, Leg));
			}
			const FResult Result = DryRun::Run(Kit.Robot, Plane(6.0, 0.0), Options);
			for (const FSample& Sample : Result.Trace)
			{
				if (Sample.Seconds >= 5.9 && Sample.Seconds <= 11.0)
				{
					Rider[Which] = FMath::Max(Rider[Which], FMath::Abs(Sample.RiderDeg.X));
					Base[Which] = FMath::Max(Base[Which], FMath::Abs(Sample.TrueAttitudeDeg.X));
				}
			}
			if (Which == 0)
			{
				ExpectSound(*this, Kit, 0, Result);
				TestTrue(FString::Printf(TEXT("centre legs jogged %+.0f cm on 6 deg: level again after"), Cm), Result.FinalRiderTiltDeg <= 0.3);
			}
		}
		AddInfo(FString::Printf(TEXT("centre legs jogged %+.0f cm on 6 deg | the base %.2f deg off, the rider %.2f; with the lowest wheels taken for the ground: %.2f and %.2f"), Cm, Base[0], Rider[0], Base[1], Rider[1]));
		TestTrue(FString::Printf(TEXT("centre legs jogged %+.0f cm: the base stays level (%.2f deg)"), Cm, Base[0]), Base[0] <= 0.5);
		TestTrue(FString::Printf(TEXT("centre legs jogged %+.0f cm: the rider no further off than the base (%.2f against %.2f deg)"), Cm, Rider[0], Base[0]), Rider[0] <= Base[0] + 0.1);
		TestTrue(FString::Printf(TEXT("centre legs jogged %+.0f cm: (with the lowest wheels taken for the ground the rider goes %.2f deg off)"), Cm, Rider[1]), Rider[1] >= 2.0);
	}
	return true;
}

// Catches: a roll the robot has DECIDED not to carry held against the tilt policy's bound. Across a 5 deg fall with a
// roll preferred the other way, 7.5 deg of roll correction would hold the reference and the roll limit is 6: standing,
// the robot is at reach, 1.5 deg short. Driven off at a speed at which a three-point robot carries no roll, all 7.5
// deg is left in the base. Where the base is wanted was bounded to the policy's 5 deg off the reference, as it is for
// what the LEGS leave; the base then stood 2.5 deg from where it was wanted for as long as the fall lasted, an error
// no leg may close, and the speed cap slowed for it (a simulator's finding, on 6 deg with 8 wanted: a quarter of the
// stick across the whole cross fall, and the course timed out). What the speed leaves in the base is added whole to
// where the legs' reach has it wanted, and is not called being past reach.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunEasedRollIsNotPastReach,
	"Ramms.Level.DryRun.RollEasedForTheSpeedIsNotPastReach",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunEasedRollIsNotPastReach::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;
	const FKit	  Kit = SlowLifts();
	const FGround Fall = Plane(0.0, 5.0);
	// Which way the fall rolls the base, standing with the assist's reference level.
	FVector2D GroundDeg = FVector2D::ZeroVector;
	{
		FOptions Options = MakeOptions(Kit, 0);
		Options.SpeedCmS = 0.0;
		Options.MaxSeconds = 4.0;
		Options.bKeepTrace = true;
		const FResult Result = DryRun::Run(Kit.Robot, Fall, Options);
		if (!TestTrue(TEXT("(a standing run with a trace)"), !Result.Trace.IsEmpty()))
		{
			return false;
		}
		GroundDeg = Result.Trace.Last().Out.GroundDeg;
	}
	if (!TestTrue(FString::Printf(TEXT("(the fall reads as 5 deg of roll: %.2f)"), GroundDeg.Y), FMath::Abs(FMath::Abs(GroundDeg.Y) - 5.0) <= 0.3))
	{
		return false;
	}
	// A roll preferred 2.5 deg the other way: 7.5 deg of correction wanted, 6 to be had.
	FOptions Options = MakeOptions(Kit, 0);
	Options.Tuning.LevelReferenceDeg = FVector2D(0.0, -2.5 * FMath::Sign(GroundDeg.Y));
	Options.SpeedCmS = 0.0;
	Options.MaxSeconds = 17.0;
	Options.PitchPerAccelDeg = BodyPitchPerAccel;
	Options.Drive.bOn = true;
	Options.bKeepTrace = true;
	Options.Events.Add(MakeEvent(5.0, EEvent::SetSpeed, 0.0, 120.0));
	const FResult Result = DryRun::Run(Kit.Robot, Fall, Options);
	double		  StandingRoll = 0.0, CruiseScale = 1.0, CruiseSpeed = 0.0, CruiseRoll = 0.0, CruiseError = 0.0;
	bool		  bStandingAtReach = false, bCruisePastReach = false;
	for (const FSample& Sample : Result.Trace)
	{
		if (Sample.Seconds > 4.0 && Sample.Seconds < 5.0)
		{
			StandingRoll = FMath::Max(StandingRoll, FMath::Abs(Sample.Out.AppliedDeg.Y));
			bStandingAtReach = bStandingAtReach || Sample.Out.Status == EStatus::AtReach;
		}
		if (Sample.Seconds > 12.0)
		{
			CruiseScale = FMath::Min(CruiseScale, Sample.Out.SpeedScale);
			CruiseSpeed = FMath::Max(CruiseSpeed, FMath::Abs(Sample.SpeedCmS));
			CruiseRoll = FMath::Max(CruiseRoll, FMath::Abs(Sample.Out.AppliedDeg.Y));
			CruiseError = FMath::Max(CruiseError, Sample.Out.ErrorDeg);
			bCruisePastReach = bCruisePastReach || Sample.Out.Status == EStatus::PastReach;
		}
	}
	AddInfo(FString::Printf(TEXT("standing: roll carried %.2f deg, at reach %d | from 7 s after the stick: least scale %.3f, top speed %.0f cm/s, roll carried %.2f deg, past reach %d"),
		StandingRoll, bStandingAtReach ? 1 : 0, CruiseScale, CruiseSpeed, CruiseRoll, bCruisePastReach ? 1 : 0));
	TestTrue(FString::Printf(TEXT("standing: all of the roll limit is carried and the robot is at reach (%.2f deg)"), StandingRoll), StandingRoll >= 5.8 && bStandingAtReach);
	TestTrue(FString::Printf(TEXT("at speed the roll is out (%.2f deg carried)"), CruiseRoll), CruiseRoll <= 0.05);
	TestTrue(FString::Printf(TEXT("... the drive is not held for it (least scale %.3f)"), CruiseScale), CruiseScale >= 0.95);
	TestTrue(FString::Printf(TEXT("... the robot at its speed (%.0f cm/s)"), CruiseSpeed), CruiseSpeed >= 110.0);
	TestFalse(TEXT("... and that is not called being past reach"), bCruisePastReach);
	// (Where the base is wanted is where the roll left in it has it: a wanted attitude held to the tilt policy's bound
	// would stand 2.5 deg from the base for as long as the fall lasts, an error no leg may close.)
	TestTrue(FString::Printf(TEXT("... the base is where it is wanted (%.2f deg from it at most)"), CruiseError), CruiseError <= 0.5);
	return true;
}

// Catches: a drive slowed for a turn of the ground that no leg is answering. At a speed at which a three-point robot
// carries no roll, a cross fall that comes on under it turns the ground in roll faster than its lifts could follow;
// the speed cap, which slows the robot to the speed at which the legs can follow the ground, slowed it to a crawl for
// a roll it had decided not to follow at all (the simulator: full stick over a course whose cross fall comes and goes,
// down to a fifth of the stick for ten seconds). With the roll out, neither the ground's turning in roll nor the roll
// it leaves between the base and where it is wanted asks anything of the drive; nor is a fall of more than the tilt
// policy's bound, left in the base that way, counted as past reach on a tick the allocation is cut (the drive put on
// the scale's floor). At a walk the roll is carried onto the fall as before.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelDryRunNoCapForRollNotCarried,
	"Ramms.Level.DryRun.NoSpeedCapForARollThatIsNotCarried",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelDryRunNoCapForRollNotCarried::RunTest(const FString& Parameters)
{
	using namespace RammsLevelDryRunTests;
	const FKit Kit = SlowLifts();
	// Flat for 6 m; then a 6 deg cross fall comes on over 60 cm (faster than these lifts turn the roll at any speed
	// over a crawl) and stays.
	const FGround Course = [](double X, double Y) {
		const double Share = X < 600.0 ? 0.0 : X < 660.0 ? (X - 600.0) / 60.0
														 : 1.0;
		return Y * FMath::Tan(FMath::DegreesToRadians(6.0)) * Share;
	};
	double		 LeastScale[2] = { 1.0, 1.0 };
	double		 RollCarried[2] = { 0.0, 0.0 };
	const double Sticks[2] = { 120.0, 40.0 };
	for (int32 Which = 0; Which < 2; ++Which)
	{
		FOptions Options = MakeOptions(Kit, 0);
		Options.SpeedCmS = 0.0;
		Options.MaxSeconds = 40.0;
		Options.DistanceCm = 1200.0;
		Options.PitchPerAccelDeg = BodyPitchPerAccel;
		Options.Drive.bOn = true;
		Options.bKeepTrace = true;
		Options.Events.Add(MakeEvent(2.0, EEvent::SetSpeed, 0.0, Sticks[Which]));
		const FResult Result = DryRun::Run(Kit.Robot, Course, Options);
		TestTrue(FString::Printf(TEXT("at %.0f cm/s: driven to its end"), Sticks[Which]), Result.TravelCm >= Options.DistanceCm);
		for (const FSample& Sample : Result.Trace)
		{
			// From a span before the fall to two after it is whole.
			if (Sample.TravelCm > 540.0 && Sample.TravelCm < 800.0)
			{
				LeastScale[Which] = FMath::Min(LeastScale[Which], Sample.Out.SpeedScale);
				RollCarried[Which] = FMath::Max(RollCarried[Which], FMath::Abs(Sample.Out.AppliedDeg.Y));
			}
		}
	}
	AddInfo(FString::Printf(TEXT("a 6 deg cross fall coming on over 60 cm | at 120 cm/s: least scale %.3f, roll carried %.2f deg | at 40 cm/s: least scale %.3f, roll carried %.2f deg"), LeastScale[0], RollCarried[0], LeastScale[1], RollCarried[1]));
	TestTrue(FString::Printf(TEXT("at 120 cm/s no roll is carried onto the fall (%.2f deg)"), RollCarried[0]), RollCarried[0] <= 0.05);
	TestTrue(FString::Printf(TEXT("... and the drive is not slowed for the fall's coming on (least scale %.3f)"), LeastScale[0]), LeastScale[0] >= 0.95);
	TestTrue(FString::Printf(TEXT("at 40 cm/s the roll is carried onto the fall (%.2f deg)"), RollCarried[1]), RollCarried[1] >= 5.0);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
