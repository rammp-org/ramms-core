// The level assist's kinematic dry run (Private/Level/RammsLevelDryRun) on a robot of the RAMMP Gen1's SHAPE, at its
// own rates: two drive wheels that each move alone, a rear pair and a front pair that each move as one and that roll
// does not reach, a centre of mass between the drive wheels and the rear pair, and stances that do not stand level
// in the chassis' own frame. What the six-wheeled robot's dry runs settled is tried again here for this shape before
// anything is built on it: which legs carry (in the stance that carries the front pair clear, with every wheel
// down, and with every wheel down but the front pair raised), the three-point stance a roll correction leaves it on
// (one wheel of the rear pair off the ground), the slower rear lift, the frame's clearance, a take-over and its
// hand-back, and a simulator that steps on its own thread.
//
// The figures are the public robot's (mujoco/rammp_gen1 in the simulator's repository), rounded: where its wheels
// are, how far and how fast each lift moves them, where its frame is. Heights are from the ground it stands on with
// every lift at zero.
//
// Beside each test is what it must catch. Each run also logs one line of its figures.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Level/RammsLevelAllocate.h"
#include "Level/RammsLevelDryRun.h"
#include "Level/RammsLevelLaw.h"
#include "Level/RammsLevelTypes.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace RammsLevelGen1ShapeTests
{
	using namespace RammsLevel;
	using namespace RammsLevel::DryRun;

	/** The stances the robot is driven in. */
	enum class EStance : uint8
	{
		/** The front pair carried clear, the drive wheels let down for ground clearance: it stands on its drive wheels
		 *  and its rear pair, a degree and a half nose up. */
		Outdoor,
		/** Every lift at zero: every wheel down, the rear pair lowest. */
		AllDown,
		/** Every lift at zero but the front pair's, which is raised. */
		AllDownFrontUp
	};

	const TCHAR* StanceName(EStance Stance)
	{
		switch (Stance)
		{
			case EStance::Outdoor:
				return TEXT("outdoor stance");
			case EStance::AllDown:
				return TEXT("every wheel down");
			default:
				return TEXT("every wheel down, the front raised");
		}
	}

	/** Legs: 0 drive -Y, 1 drive +Y, 2 the rear pair, 3 the front pair. */
	constexpr int32 DriveMinusY = 0;
	constexpr int32 DrivePlusY = 1;
	constexpr int32 RearPair = 2;
	constexpr int32 FrontPair = 3;

	constexpr double DriveX = 48.6;
	constexpr double DriveHalfTrack = 30.4;
	constexpr double DriveRadius = 18.4;
	constexpr double RearX = -24.1;
	constexpr double RearHalfTrack = 10.9;
	constexpr double RearRadius = 10.1;
	constexpr double FrontX = 78.7;
	constexpr double FrontHalfTrack = 17.9;
	constexpr double FrontRadius = 7.7;
	/** With its lift at zero the rear pair's wheels are this far below the others' (cm). */
	constexpr double RearAtZero = -3.26;
	/** The frame's underside over the ground with every lift at zero (cm). */
	constexpr double FrameUnderside = 4.97;

	FRobot MakeRobot(EStance Stance)
	{
		// How far each support's wheels are from where its lift's zero has them (cm, up positive).
		const double Drive = Stance == EStance::Outdoor ? -4.97 : 0.0;
		const double Front = Stance == EStance::AllDown ? 0.0 : 5.03;
		FRobot		 Robot;
		for (const double Side : { -1.0, 1.0 })
		{
			FRobotLeg& Leg = Robot.Legs.AddDefaulted_GetRef();
			Leg.XY = FVector2D(DriveX, Side * DriveHalfTrack);
			Leg.RadiusCm = DriveRadius;
			Leg.StanceZ = DriveRadius + Drive;
			Leg.MinZ = DriveRadius - 22.1;
			Leg.MaxZ = DriveRadius + 22.9;
			// 18 deg/s on a 32 cm swing arm.
			Leg.MaxRateCmS = 10.0;
			Leg.MaxAccelCmS2 = 60.0;
		}
		{
			FRobotLeg& Leg = Robot.Legs.AddDefaulted_GetRef();
			Leg.XY = FVector2D(RearX, 0.0);
			Leg.PairHalfTrackCm = RearHalfTrack;
			Leg.RadiusCm = RearRadius;
			Leg.StanceZ = RearRadius + RearAtZero;
			Leg.MinZ = RearRadius - 14.9;
			Leg.MaxZ = RearRadius + 4.9;
			// 26 deg/s through a linkage that gives about a quarter of a centimetre a degree.
			Leg.MaxRateCmS = 6.0;
			Leg.MaxAccelCmS2 = 60.0;
			Leg.bRoll = false;
		}
		{
			FRobotLeg& Leg = Robot.Legs.AddDefaulted_GetRef();
			Leg.XY = FVector2D(FrontX, 0.0);
			Leg.PairHalfTrackCm = FrontHalfTrack;
			Leg.RadiusCm = FrontRadius;
			Leg.StanceZ = FrontRadius + Front;
			Leg.MinZ = FrontRadius - 15.2;
			Leg.MaxZ = FrontRadius + 14.4;
			// 30 deg/s on a 21 cm arm.
			Leg.MaxRateCmS = 11.0;
			Leg.MaxAccelCmS2 = 60.0;
			Leg.bRoll = false;
		}
		Robot.CentreOfMass = FVector(30.8, 0.0, 39.0);
		for (const double X : { -5.0, 59.6 })
		{
			for (const double Y : { -16.5, 16.5 })
			{
				FClearancePoint& Point = Robot.Points.AddDefaulted_GetRef();
				Point.XY = FVector2D(X, Y);
				Point.UndersideZ = FrameUnderside;
				Point.MinCm = 4.0;
			}
		}
		// Its stance does not stand level in its own frame: who stands is judged from the ground it rests on.
		Robot.bStanceRests = true;
		return Robot;
	}

	/** The tuning the public pawn is authored with. */
	FTuning MakeTuning()
	{
		FTuning Tuning;
		Tuning.MaxCorrectionDeg = FVector2D(10.0, 6.0);
		Tuning.MaxCorrectionRateDegPerSec = 8.0;
		Tuning.IntegralMaxDeg = 4.0;
		Tuning.MinBellyClearanceCm = 4.0;
		return Tuning;
	}

	const double	   FrameSeconds[3] = { 1.0 / 16.0, 1.0 / 30.0, 1.0 / 60.0 };
	const TCHAR* const FrameNames[3] = { TEXT("16 fps"), TEXT("30 fps"), TEXT("60 fps") };

	FOptions MakeOptions(int32 Rate)
	{
		FOptions Options;
		Options.Tuning = MakeTuning();
		Options.FrameSeconds = FrameSeconds[Rate];
		Options.SpeedCmS = 0.0;
		Options.MaxSeconds = 40.0;
		Options.bKeepTrace = true;
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

	double Rise(double LengthCm, double Deg)
	{
		return LengthCm * FMath::Tan(FMath::DegreesToRadians(Deg));
	}

	/** One line of a run's figures. */
	void Report(FAutomationTestBase& Test, const TCHAR* Case, int32 Rate, const FResult& Result)
	{
		Test.AddInfo(FString::Printf(TEXT("%s | %s | steady %.2f deg | peak %.2f deg | lag %.2f s | wanted %.2f deg/s %.2f deg/s2 | true %.2f deg/s | floating %.2f s | at reach %.2f s, reach left %.2f | correction %.2f deg | scale >= %.2f | least %.1f mean %.1f cm/s | %.1f s"),
			Case, FrameNames[Rate], Result.SteadyResidualDeg, Result.PeakTiltDeg, Result.LongestOffLevelSeconds, Result.MaxWantedRateDegPerSec, Result.MaxWantedAccelDegPerSec2, Result.MaxTrueRateDegPerSec,
			Result.FloatingSeconds, Result.SecondsAtReach, Result.MinReachLeft, Result.MaxCorrectionDeg, Result.MinSpeedScale, Result.MinSpeedCmS, Result.MeanSpeedCmS, Result.Seconds));
	}

	/** What holds for every run that goes well. */
	void ExpectSound(FAutomationTestBase& Test, const FString& Case, const FResult& Result, const FTuning& Tuning)
	{
		Test.TestEqual(Case + TEXT(": every pose a stable rest"), Result.UnstableFrames, 0);
		Test.TestFalse(Case + TEXT(": no fault"), Result.bEverFaulted);
		Test.TestTrue(Case + TEXT(": the wanted attitude within its rate"), Result.MaxWantedRateDegPerSec <= Tuning.TiltRateDegPerSec + 1.0e-3);
		Test.TestTrue(Case + TEXT(": the wanted attitude within its acceleration"), Result.MaxWantedAccelDegPerSec2 <= Tuning.TiltAccelDegPerSec2 + 1.0e-3);
		Test.TestTrue(Case + TEXT(": the speed scale never under its floor"), Result.MinSpeedScale >= Tuning.MinSpeedScale - 1.0e-9);
	}

	/** The most the base was off level (deg), and the most bottoms that touched, between two travels (cm). */
	struct FPassage
	{
		double PeakDeg = 0.0;
		double RateDegPerSec = 0.0;
		double LeastScale = 1.0;
		double MeanSpeedCmS = 0.0;
		double LeastSpeedCmS = 1.0e6;
		int32  MostTouching = 0;
		int32  LeastTouching = 100;
	};

	FPassage PassageBetween(const FResult& Result, const FTuning& Tuning, double FromCm, double ToCm, double TickSeconds)
	{
		FPassage	Out;
		int32		Count = 0;
		const int32 Window = FMath::Max(1, FMath::RoundToInt32(0.3 / TickSeconds));
		for (int32 Index = 0; Index < Result.Trace.Num(); ++Index)
		{
			const FSample& Sample = Result.Trace[Index];
			if (Sample.TravelCm < FromCm || Sample.TravelCm > ToCm)
			{
				continue;
			}
			Out.PeakDeg = FMath::Max(Out.PeakDeg, Inclination(Sample.TrueAttitudeDeg - Tuning.LevelReferenceDeg));
			Out.LeastScale = FMath::Min(Out.LeastScale, Sample.Out.SpeedScale);
			Out.MeanSpeedCmS += Sample.SpeedCmS;
			Out.LeastSpeedCmS = FMath::Min(Out.LeastSpeedCmS, Sample.SpeedCmS);
			Out.MostTouching = FMath::Max(Out.MostTouching, Sample.Touching);
			Out.LeastTouching = FMath::Min(Out.LeastTouching, Sample.Touching);
			++Count;
			if (Index >= Window)
			{
				Out.RateDegPerSec = FMath::Max(Out.RateDegPerSec, (Sample.TrueAttitudeDeg - Result.Trace[Index - Window].TrueAttitudeDeg).Size() / (Window * TickSeconds));
			}
		}
		Out.MeanSpeedCmS = Count > 0 ? Out.MeanSpeedCmS / Count : 0.0;
		return Out;
	}

	/** The same between two times (s). */
	double PeakBetween(const FResult& Result, const FTuning& Tuning, double FromSeconds, double ToSeconds)
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

	/** Each wheel's contact point at the stance (chassis frame, cm), the pairs' two wheels each. */
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
} // namespace RammsLevelGen1ShapeTests

// Catches: who stands judged from the lowest wheel in the chassis' own frame (this robot's stances do not stand level
// in it: the rear pair then reads as off the ground in the one, and as the only leg down in the other), so that the
// integral never runs and what the joints sag by under the load is left in the rider's attitude; and a stance that
// stands a degree and a half nose up on the flat left so with the switch on.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelGen1ShapeStands,
	"Ramms.Level.Gen1Shape.LevelOnTheLegsThatCarry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelGen1ShapeStands::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1ShapeTests;

	for (const EStance Stance : { EStance::Outdoor, EStance::AllDown, EStance::AllDownFrontUp })
	{
		const FRobot Robot = MakeRobot(Stance);
		// Where it rests on the flat, by the rest solve: which wheels touch, and how far off level its stance stands.
		FRestPose Rest;
		TestTrue(FString::Printf(TEXT("%s: a rest on the flat"), StanceName(Stance)), SolveRest(StanceBottoms(Robot), Plane(0.0, 0.0), FVector2D::ZeroVector, 0.0, Robot.CentreOfMass, nullptr, Rest));
		AddInfo(FString::Printf(TEXT("%s | on the flat it stands %.2f deg nose up; gaps (cm): drive %.2f %.2f, rear %.2f %.2f, front %.2f %.2f"), StanceName(Stance), Rest.AttitudeDeg.X, Rest.GapCm[0], Rest.GapCm[1],
			Rest.GapCm[2], Rest.GapCm[3], Rest.GapCm[4], Rest.GapCm[5]));

		// The law's own count of who stands, from the flat ground the stance rests on, is the rest solve's.
		{
			TArray<FLegReach, TInlineAllocator<8>> Legs;
			for (const FRobotLeg& Leg : Robot.Legs)
			{
				FLegReach& Reach = Legs.AddDefaulted_GetRef();
				Reach.XY = Leg.XY;
				Reach.BaseZ = Reach.MeasuredZ = Leg.StanceZ;
				Reach.MinZ = Leg.MinZ;
				Reach.MaxZ = Leg.MaxZ;
				Reach.RadiusCm = Leg.RadiusCm;
				Reach.bRoll = Leg.bRoll;
			}
			const double HalfTracks[4] = { 0.0, 0.0, RearHalfTrack, FrontHalfTrack };
			TestTrue(FString::Printf(TEXT("%s: the ground it rests on is found"), StanceName(Stance)), StanceGround(Legs, MakeArrayView(HalfTracks), FVector2D(Robot.CentreOfMass)));
			SupportMask(Legs, TConstArrayView<bool>(), MakeTuning());
			for (int32 Leg = 0; Leg < 4; ++Leg)
			{
				const double GapCm = Leg == DriveMinusY ? Rest.GapCm[0] : Leg == DrivePlusY ? Rest.GapCm[1]
					: Leg == RearPair														? Rest.GapCm[2]
																							: Rest.GapCm[4];
				TestEqual(FString::Printf(TEXT("%s: leg %d's clearance over the ground it rests on is the rest solve's gap"), StanceName(Stance), Leg), Legs[Leg].BaselineClearanceCm, GapCm, 0.02);
				TestEqual(FString::Printf(TEXT("%s: leg %d carries when it touches"), StanceName(Stance), Leg), Legs[Leg].CarryShare >= 0.99, GapCm <= 0.01);
			}
			int32 Standing = 0;
			for (const FLegReach& Leg : Legs)
			{
				Standing += Leg.bSupport ? 1 : 0;
			}
			TestTrue(FString::Printf(TEXT("%s: three legs or more stand, so the integral may run (%d)"), StanceName(Stance), Standing), Standing >= MinSupportForIntegral);
		}

		// The outdoor stance is the one it is driven in on slopes: its frame is 10 cm off the ground. The others
		// carry the frame at 5 cm, which is all but what must be kept: they are tried on what they can level, and
		// on a slope past it, where the base must be left the rest without hunting or a fault.
		const bool				bOutdoor = Stance == EStance::Outdoor;
		const TArray<FVector2D> Grounds = bOutdoor ? TArray<FVector2D>{ FVector2D(0.0, 0.0), FVector2D(4.0, 0.0), FVector2D(-4.0, 0.0), FVector2D(0.0, 4.0), FVector2D(3.0, -3.0) }
												   : TArray<FVector2D>{ FVector2D(0.0, 0.0), FVector2D(1.0, 0.0), FVector2D(-1.0, 0.0), FVector2D(0.0, 1.0), FVector2D(4.0, 0.0), FVector2D(-4.0, 0.0), FVector2D(0.0, 4.0) };
		for (const FVector2D& GroundDeg : Grounds)
		{
			for (int32 Rate = 0; Rate < 3; ++Rate)
			{
				// Parked, the switch thrown after a second; in the outdoor stance, with joints that give under the load.
				FOptions Options = MakeOptions(Rate);
				Options.bEnabledAtStart = false;
				Options.SagCm = bOutdoor ? 0.8 : 0.0;
				Options.MaxSeconds = 20.0;
				Options.Events.Add(MakeEvent(1.0, EEvent::Enable));
				FOptions OffOptions = Options;
				OffOptions.Events.Reset();
				const FResult Off = DryRun::Run(Robot, Plane(GroundDeg.X, GroundDeg.Y), OffOptions);
				const FResult Result = DryRun::Run(Robot, Plane(GroundDeg.X, GroundDeg.Y), Options);
				const FString Case = FString::Printf(TEXT("%s, parked on (%.0f, %.0f) deg, %s"), StanceName(Stance), GroundDeg.X, GroundDeg.Y, FrameNames[Rate]);
				Report(*this, *Case, Rate, Result);
				ExpectSound(*this, Case, Result, Options.Tuning);
				const bool bAtReach = Result.FinalStatus == EStatus::AtReach || Result.FinalStatus == EStatus::PastReach;
				TestTrue(Case + FString::Printf(TEXT(": still at the end (%.2f deg, legs %.3f cm)"), Result.SteadyTiltRangeDeg, Result.SteadyLegRangeCm), Result.SteadyTiltRangeDeg <= 0.05 && Result.SteadyLegRangeCm <= 0.05);
				TestTrue(Case + FString::Printf(TEXT(": brought there within the rider's tilt rate (%.2f deg/s)"), Result.MaxTrueRateDegPerSec), Result.MaxTrueRateDegPerSec <= 4.0);
				if (bOutdoor || !bAtReach)
				{
					TestTrue(Case + FString::Printf(TEXT(": level, what the joints sag by trimmed out (%.2f deg)"), Result.SteadyResidualDeg), Result.SteadyResidualDeg <= 0.2);
					TestEqual(Case + TEXT(": at rest at the end"), static_cast<int32>(Result.FinalStatus), static_cast<int32>(EStatus::Settled));
				}
				else
				{
					// Past what the low frame allows: said, and the base no further off than with the switch off.
					TestTrue(Case + FString::Printf(TEXT(": at reach, the base no further off than switched off (%.2f against %.2f deg)"), Result.FinalTiltDeg, Off.FinalTiltDeg), Result.FinalTiltDeg <= Off.FinalTiltDeg + 0.05);
					TestTrue(Case + TEXT(": and what stops it is named"), !Result.Trace.IsEmpty() && Result.Trace.Last().Out.ReachLimit != EReachLimit::None);
				}
			}
		}
	}
	return true;
}

// Catches: a roll correction given to a pair that roll does not reach; a base left tilted, or hunting, on the three
// points a roll correction puts it on (both drive wheels and one wheel of the rear pair, the other in the air); a
// drive wheel off the ground there; and a cross slope past the roll allowed taken for a fault.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelGen1ShapeThreePoints,
	"Ramms.Level.Gen1Shape.CrossSlopeOnThreePoints",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelGen1ShapeThreePoints::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1ShapeTests;

	const FRobot Robot = MakeRobot(EStance::Outdoor);
	for (int32 Rate = 0; Rate < 3; ++Rate)
	{
		// Driven from the flat onto a cross fall that comes on over a metre and a half, along it, and off again.
		for (const double CrossDeg : { 2.0, 5.0 })
		{
			FOptions Options = MakeOptions(Rate);
			Options.SpeedCmS = 60.0;
			Options.DistanceCm = 1100.0;
			// (A warp from a cross fall one way to the other is twice the roll: two of them make on and off.)
			const FGround On = Warp(CrossDeg / 2.0, 150.0, 150.0);
			const FGround Course = [On, CrossDeg](double X, double Y) {
				// Level, falling to -Y by CrossDeg from 300 to 700, level again from 850.
				const double Tan = FMath::Tan(FMath::DegreesToRadians(CrossDeg));
				const double Share = X < 150.0 ? 0.0 : X < 300.0 ? (X - 150.0) / 150.0
					: X < 700.0									 ? 1.0
					: X < 850.0									 ? 1.0 - (X - 700.0) / 150.0
																 : 0.0;
				return Y * Tan * Share;
			};
			const FResult Result = DryRun::Run(Robot, Course, Options);
			const FString Case = FString::Printf(TEXT("a %.0f deg cross fall at 60 cm/s, %s"), CrossDeg, FrameNames[Rate]);
			Report(*this, *Case, Rate, Result);
			ExpectSound(*this, Case, Result, Options.Tuning);
			TestTrue(Case + TEXT(": driven to its end"), Result.TravelCm >= Options.DistanceCm);
			const FPassage Along = PassageBetween(Result, Options.Tuning, 450.0, 650.0, Options.FrameSeconds);
			const FPassage Whole = PassageBetween(Result, Options.Tuning, 0.0, 1100.0, Options.FrameSeconds);
			AddInfo(FString::Printf(TEXT("%s | along the fall %.2f deg, %d to %d wheels down; over the whole course %.2f deg at %.1f deg/s"), *Case, Along.PeakDeg, Along.LeastTouching, Along.MostTouching, Whole.PeakDeg,
				Whole.RateDegPerSec));
			TestTrue(Case + FString::Printf(TEXT(": level along the fall (%.2f deg)"), Along.PeakDeg), Along.PeakDeg <= 0.3);
			// Three points: both drive wheels and the rear wheel on the high side.
			TestEqual(Case + TEXT(": on three wheels along the fall"), Along.MostTouching, 3);
			TestEqual(Case + TEXT(": never on fewer"), Along.LeastTouching, 3);
			TestTrue(Case + FString::Printf(TEXT(": the rider turned gently coming on and off it (%.1f deg/s, %.2f deg)"), Whole.RateDegPerSec, Whole.PeakDeg), Whole.RateDegPerSec <= 4.0 && Whole.PeakDeg <= 1.5);
			TestTrue(Case + FString::Printf(TEXT(": level again on the flat (%.2f deg)"), Result.FinalTiltDeg), Result.FinalTiltDeg <= 0.3);
		}

		// Parked across a fall past what roll is allowed (6 deg): held at the limit, the rest taken as tilt, no fault.
		FOptions Options = MakeOptions(Rate);
		Options.bEnabledAtStart = false;
		Options.MaxSeconds = 14.0;
		Options.Events.Add(MakeEvent(1.0, EEvent::Enable));
		const FResult Result = DryRun::Run(Robot, Plane(0.0, -9.0), Options);
		const FString Case = FString::Printf(TEXT("parked across 9 deg, %s"), FrameNames[Rate]);
		Report(*this, *Case, Rate, Result);
		ExpectSound(*this, Case, Result, Options.Tuning);
		TestTrue(Case + FString::Printf(TEXT(": the roll correction held at its limit (%.2f deg)"), Result.MaxCorrectionDeg), Result.MaxCorrectionDeg <= 6.0 + 0.1 && Result.MaxCorrectionDeg >= 5.8);
		TestTrue(Case + FString::Printf(TEXT(": left 3 deg off (%.2f)"), Result.FinalTiltDeg), Result.FinalTiltDeg >= 2.7 && Result.FinalTiltDeg <= 3.4);
		TestTrue(Case + TEXT(": and said so"), Result.FinalStatus == EStatus::AtReach && !Result.Trace.IsEmpty() && Result.Trace.Last().Out.ReachLimit == EReachLimit::AxisLimit);
	}
	return true;
}

// Catches: a 1:12 ramp met faster than this robot's lifts can follow with nothing asked of the drive (its rear lift
// is the slow one, and does two thirds of a pitch correction); the drive stopped for it; the raised front pair set
// on the ground at the ramp's foot; and a ramp's end at full speed left to turn the rider faster than switched off.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelGen1ShapeRamp,
	"Ramms.Level.Gen1Shape.RampAtAWalkAndAtFullSpeed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelGen1ShapeRamp::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1ShapeTests;

	const FRobot Robot = MakeRobot(EStance::Outdoor);
	// Flat, a 1:12 ramp up for 4 m, a 3 m landing, down for 4 m, flat.
	const double	GradeDeg = FMath::RadiansToDegrees(FMath::Atan(1.0 / 12.0));
	const double	Top = Rise(400.0, GradeDeg);
	const FVector2D Knots[4] = { { 200.0, 0.0 }, { 600.0, Top }, { 900.0, Top }, { 1300.0, 0.0 } };
	const FGround	Course = Profile(MakeArrayView(Knots));
	for (const double Speed : { 60.0, 120.0 })
	{
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			// Levelled where it stands for five seconds, then driven: a drive that is a speed loop on a robot gravity
			// pulls, and a body that pitches as it is pushed.
			FOptions Options = MakeOptions(Rate);
			Options.DistanceCm = 1600.0;
			Options.MaxSeconds = 90.0;
			Options.PitchPerAccelDeg = 0.006;
			Options.Drive.bOn = true;
			Options.Events.Add(MakeEvent(5.0, EEvent::SetSpeed, 0.0, Speed));
			FOptions OffOptions = Options;
			OffOptions.bEnabledAtStart = false;
			const FResult Off = DryRun::Run(Robot, Course, OffOptions);
			const FResult Result = DryRun::Run(Robot, Course, Options);
			const FString Case = FString::Printf(TEXT("a 1:12 ramp at %.0f cm/s, %s"), Speed, FrameNames[Rate]);
			Report(*this, *Case, Rate, Result);
			ExpectSound(*this, Case, Result, Options.Tuning);
			TestTrue(Case + TEXT(": driven to its end"), Result.TravelCm >= Options.DistanceCm && Off.TravelCm >= Options.DistanceCm);
			const double Kept = Result.Seconds > 5.0 ? (Off.Seconds - 5.0) / (Result.Seconds - 5.0) : 0.0;
			// (From the stick on: the first five seconds are the walk to level from the stance's own pitch.)
			const FPassage On = PassageBetween(Result, Options.Tuning, 50.0, 1600.0, Options.FrameSeconds);
			const FPassage Was = PassageBetween(Off, Options.Tuning, 50.0, 1600.0, Options.FrameSeconds);
			const FPassage Steady = PassageBetween(Result, Options.Tuning, 400.0, 550.0, Options.FrameSeconds);
			AddInfo(FString::Printf(TEXT("%s | rider %.2f deg at %.1f deg/s (switched off %.2f at %.1f); speed kept %.2f, least scale %.2f, least speed %.1f cm/s; on the ramp %.2f deg at %.1f cm/s; most wheels down %d"), *Case,
				On.PeakDeg, On.RateDegPerSec, Was.PeakDeg, Was.RateDegPerSec, Kept, Result.MinSpeedScale, On.LeastSpeedCmS, Steady.PeakDeg, Steady.MeanSpeedCmS, On.MostTouching));
			TestTrue(Case + FString::Printf(TEXT(": never slowed to a stop (%.1f cm/s)"), On.LeastSpeedCmS), On.LeastSpeedCmS >= Speed * Options.Tuning.MinSpeedScale);
			TestTrue(Case + FString::Printf(TEXT(": level on the ramp (%.2f deg)"), Steady.PeakDeg), Steady.PeakDeg <= 0.4);
			TestTrue(Case + FString::Printf(TEXT(": driven up it at the speed the stick asks (%.1f cm/s)"), Steady.MeanSpeedCmS), Steady.MeanSpeedCmS >= 0.95 * Speed);
			TestTrue(Case + FString::Printf(TEXT(": the rider turned no faster than switched off (%.1f against %.1f deg/s)"), On.RateDegPerSec, Was.RateDegPerSec), On.RateDegPerSec <= Was.RateDegPerSec + 0.1);
			TestTrue(Case + FString::Printf(TEXT(": level again at the end (%.2f deg)"), Result.FinalTiltDeg), Result.FinalTiltDeg <= 0.3);
			// The raised front pair stays off the ground all the way: the drive wheels and the rear pair's two.
			TestTrue(Case + FString::Printf(TEXT(": the raised front pair never set down (%d wheels down at most)"), On.MostTouching), On.MostTouching <= 4);
			if (Speed <= 60.0)
			{
				// At a walk the lifts follow the ramp's breaks: the rider well under the ramp's own 4.8 deg.
				TestTrue(Case + FString::Printf(TEXT(": the rider within 1.5 deg through the breaks (%.2f), turned within 4 deg/s (%.1f)"), On.PeakDeg, On.RateDegPerSec), On.PeakDeg <= 1.5 && On.RateDegPerSec <= 4.0);
				TestTrue(Case + FString::Printf(TEXT(": little of the pace lost (%.2f)"), Kept), Kept >= 0.9);
			}
			else
			{
				// At full speed the ramp's foot turns the ground faster than the rear lift can follow: the drive is
				// led down for the breaks, and what is left is far under switched off.
				TestTrue(Case + FString::Printf(TEXT(": the drive slowed for the breaks (scale %.2f)"), Result.MinSpeedScale), Result.MinSpeedScale <= 0.9);
				TestTrue(Case + FString::Printf(TEXT(": the rider within 2.5 deg (%.2f; switched off %.2f)"), On.PeakDeg, Was.PeakDeg), On.PeakDeg <= 2.5 && On.PeakDeg <= 0.5 * Was.PeakDeg);
				TestTrue(Case + FString::Printf(TEXT(": most of the pace kept (%.2f)"), Kept), Kept >= 0.6);
			}
		}
	}
	return true;
}

// Catches: a reach that is not the one the frame, the rear lift's travel and the raised front pair allow (the law
// then promises level where the frame would be on the ground, or gives up short); a slope past it taken for a fault
// or left with the rider turned faster than the tilt policy; and a ride-height allowance that buys nothing.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelGen1ShapeReach,
	"Ramms.Level.Gen1Shape.ReachNoseUpAndNoseDown",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelGen1ShapeReach::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1ShapeTests;

	const FRobot Robot = MakeRobot(EStance::Outdoor);
	// The steepest slope each way it is still held level on, parked, to half a degree: with no ride-height allowance,
	// and with the chassis allowed 3 cm up.
	for (const double RideUpCm : { 0.0, 3.0 })
	{
		for (const double Sign : { 1.0, -1.0 })
		{
			double		ReachDeg = 0.0;
			EReachLimit Limit = EReachLimit::None;
			for (double Deg = 4.0; Deg <= 14.0; Deg += 0.5)
			{
				FOptions Options = MakeOptions(0);
				Options.Tuning.RideHeightUpCm = RideUpCm;
				// (Past the public pawn's 10 deg per axis, to find what the robot's shape allows.)
				Options.Tuning.MaxCorrectionDeg.X = 15.0;
				Options.bEnabledAtStart = false;
				Options.MaxSeconds = 16.0;
				Options.Events.Add(MakeEvent(1.0, EEvent::Enable));
				const FResult Result = DryRun::Run(Robot, Plane(Sign * Deg, 0.0), Options);
				const FString Case = FString::Printf(TEXT("parked on %.1f deg, ride %.0f cm up"), Sign * Deg, RideUpCm);
				ExpectSound(*this, Case, Result, Options.Tuning);
				if (Result.FinalTiltDeg > 0.3)
				{
					Limit = Result.Trace.IsEmpty() ? EReachLimit::None : Result.Trace.Last().Out.ReachLimit;
					TestTrue(Case + FString::Printf(TEXT(": past reach the base is left the rest, and no more (%.2f deg)"), Result.FinalTiltDeg), Result.FinalTiltDeg <= Deg - ReachDeg + 0.3);
					TestTrue(Case + FString::Printf(TEXT(": and says it is at reach (status %d, %.2f deg off)"), static_cast<int32>(Result.FinalStatus), Result.FinalTiltDeg),
						Result.FinalStatus == EStatus::AtReach || Result.FinalStatus == EStatus::PastReach);
					break;
				}
				ReachDeg = Deg;
			}
			AddInfo(FString::Printf(TEXT("reach, %s, ride height %.0f cm up: %.1f deg (then: limit %d)"), Sign > 0.0 ? TEXT("nose up") : TEXT("nose down"), RideUpCm, ReachDeg, static_cast<int32>(Limit)));
			// Nose up it is the frame that comes down on the slope rising ahead; nose down, the rear lift at the top of
			// its short travel (its wheels come up only a third as far as they go down).
			TestEqual(FString::Printf(TEXT("what stops a %s correction, ride height %.0f cm up"), Sign > 0.0 ? TEXT("nose-up") : TEXT("nose-down"), RideUpCm), static_cast<int32>(Limit),
				static_cast<int32>(Sign > 0.0 ? EReachLimit::Underside : EReachLimit::LegTravel));
			if (Sign < 0.0 && RideUpCm > 0.0)
			{
				// The chassis let up 3 cm takes the rear lift off its stop: three degrees and more of nose-down reach.
				TestTrue(FString::Printf(TEXT("3 cm of ride height buys nose-down reach (%.1f deg)"), ReachDeg), ReachDeg >= 10.5);
			}
			// A 1:12 ramp both ways with room to spare, with or without the allowance.
			TestTrue(FString::Printf(TEXT("%s reach with %.0f cm of ride height covers a 1:12 ramp and a 6 deg one (%.1f deg)"), Sign > 0.0 ? TEXT("nose up") : TEXT("nose down"), RideUpCm, ReachDeg), ReachDeg >= 6.0);
		}
	}
	return true;
}

// Catches: a take-over (a curb crossing) that steps a lift at its fold or at its hand-back; a hand-back that leaves
// the levelling plane in the lifts' own commands (switching off would then not give the stance back); and the
// switch thrown on a slope moving the rider faster than the tilt policy.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelGen1ShapeTakeOver,
	"Ramms.Level.Gen1Shape.TakeOverHandBackAndSwitchOff",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelGen1ShapeTakeOver::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1ShapeTests;

	const FRobot Robot = MakeRobot(EStance::Outdoor);
	for (int32 Rate = 0; Rate < 3; ++Rate)
	{
		// Parked on a grade with a cross fall, levelled; taken over for two seconds by something that leaves the
		// lifts as it found them (a crossing puts every lift back); handed back; then switched off.
		FOptions Options = MakeOptions(Rate);
		Options.MaxSeconds = 24.0;
		FDryEvent Take = MakeEvent(8.0, EEvent::FoldAdopt, 2.0);
		Options.Events.Add(Take);
		Options.Events.Add(MakeEvent(14.0, EEvent::Disable));
		const FResult Result = DryRun::Run(Robot, Plane(3.0, -4.0), Options);
		const FString Case = FString::Printf(TEXT("a take-over parked on (3, -4) deg, %s"), FrameNames[Rate]);
		Report(*this, *Case, Rate, Result);
		ExpectSound(*this, Case, Result, Options.Tuning);
		TestTrue(Case + FString::Printf(TEXT(": no lift stepped at the fold or the hand-back (%.3f cm)"), Result.MaxEventLegStepCm), Result.MaxEventLegStepCm <= 0.05);
		const double Through = PeakBetween(Result, Options.Tuning, 7.5, 13.5);
		TestTrue(Case + FString::Printf(TEXT(": level before, through and after it (%.2f deg)"), Through), Through <= 0.3);
		// After the hand-back the plane is the law's again: the lifts' own commands are the stance's.
		for (const FSample& Sample : Result.Trace)
		{
			if (Sample.Seconds >= 11.0 && Sample.Seconds <= 13.5)
			{
				for (int32 Leg = 0; Leg < Robot.Legs.Num(); ++Leg)
				{
					if (FMath::Abs(Sample.BaselineZ[Leg] - Robot.Legs[Leg].StanceZ) > 0.05)
					{
						AddError(Case + FString::Printf(TEXT(": after the hand-back leg %d's own command is %.2f cm, not the stance's %.2f"), Leg, Sample.BaselineZ[Leg], Robot.Legs[Leg].StanceZ));
						return false;
					}
				}
			}
		}
		TestEqual(Case + TEXT(": off at the end"), static_cast<int32>(Result.FinalState), static_cast<int32>(EState::Off));
		TestTrue(Case + TEXT(": ended exactly on the stance"), Result.FinalMaxOffsetCm == 0.0);
		TestTrue(Case + FString::Printf(TEXT(": the rider given back to the ground within the tilt policy (%.2f deg/s)"), Result.MaxTrueRateDegPerSec), Result.MaxTrueRateDegPerSec <= 3.6);
	}
	return true;
}

// Catches: the law run on the tick's clock where the simulator steps on its own thread (the speed between two poses
// then swings from frame to frame, reads as braking, and the ground is not followed), on this robot's shape and
// rates.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelGen1ShapeUnlocked,
	"Ramms.Level.Gen1Shape.UnlockedStepping",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelGen1ShapeUnlocked::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1ShapeTests;

	const FRobot	Robot = MakeRobot(EStance::Outdoor);
	const double	GradeDeg = FMath::RadiansToDegrees(FMath::Atan(1.0 / 12.0));
	const double	Top = Rise(300.0, GradeDeg);
	const FVector2D Knots[3] = { { 200.0, 0.0 }, { 500.0, Top }, { 800.0, 0.0 } };
	const FGround	Course = Profile(MakeArrayView(Knots));
	const double	UnlockedFrames[3] = { 1.0 / 16.0, 1.0 / 30.0, 1.0 / 50.0 };
	const TCHAR*	UnlockedNames[3] = { TEXT("16 fps"), TEXT("30 fps"), TEXT("50 fps") };
	for (int32 Rate = 0; Rate < 3; ++Rate)
	{
		FOptions Options = MakeOptions(0);
		Options.FrameSeconds = UnlockedFrames[Rate];
		Options.DistanceCm = 1100.0;
		Options.MaxSeconds = 90.0;
		Options.PitchPerAccelDeg = 0.006;
		Options.Events.Add(MakeEvent(5.0, EEvent::SetSpeed, 0.0, 60.0));
		FOptions UnlockedOptions = Options;
		UnlockedOptions.Unlocked.bOn = true;
		FOptions OffOptions = Options;
		OffOptions.bEnabledAtStart = false;
		const FResult  Off = DryRun::Run(Robot, Course, OffOptions);
		const FResult  Locked = DryRun::Run(Robot, Course, Options);
		const FResult  Unlocked = DryRun::Run(Robot, Course, UnlockedOptions);
		const FString  Case = FString::Printf(TEXT("a 1:12 crest at 60 cm/s, %s unlocked"), UnlockedNames[Rate]);
		const FPassage Was = PassageBetween(Off, Options.Tuning, 50.0, 1100.0, Options.FrameSeconds);
		const FPassage Even = PassageBetween(Locked, Options.Tuning, 50.0, 1100.0, Options.FrameSeconds);
		const FPassage Uneven = PassageBetween(Unlocked, Options.Tuning, 50.0, 1100.0, Options.FrameSeconds);
		AddInfo(FString::Printf(TEXT("%s | rider, switched off %.2f deg at %.1f deg/s; frame-locked %.2f at %.1f; unlocked %.2f at %.1f"), *Case, Was.PeakDeg, Was.RateDegPerSec, Even.PeakDeg, Even.RateDegPerSec, Uneven.PeakDeg,
			Uneven.RateDegPerSec));
		ExpectSound(*this, Case, Unlocked, Options.Tuning);
		TestTrue(Case + TEXT(": driven to its end"), Unlocked.TravelCm >= Options.DistanceCm);
		TestTrue(Case + FString::Printf(TEXT(": as level as frame-locked, but for the pose's age (%.2f against %.2f deg)"), Uneven.PeakDeg, Even.PeakDeg), Uneven.PeakDeg <= Even.PeakDeg + 0.7);
		TestTrue(Case + FString::Printf(TEXT(": the rider turned as gently (%.1f against %.1f deg/s)"), Uneven.RateDegPerSec, Even.RateDegPerSec), Uneven.RateDegPerSec <= Even.RateDegPerSec + 2.0);
		TestTrue(Case + FString::Printf(TEXT(": far better off than switched off (%.2f against %.2f deg)"), Uneven.PeakDeg, Was.PeakDeg), Uneven.PeakDeg <= 0.5 * Was.PeakDeg);
	}
	return true;
}

// Catches: a rear pair whose wheels do not move as its table says (the table is of the lift with no load on it, and
// the pair hangs on a spring) left in the rider's attitude: the feed-forward believes a height the wheels did not go
// to, and only the integral takes the rest out -- which it may do only while the legs that carry count as standing.
// And that error turned into hunting, parked or on a ramp.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelGen1ShapeRearTable,
	"Ramms.Level.Gen1Shape.RearPairThatIsNotItsTable",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelGen1ShapeRearTable::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1ShapeTests;

	const double GradeDeg = FMath::RadiansToDegrees(FMath::Atan(1.0 / 12.0));
	const double Top = Rise(400.0, GradeDeg);
	for (const double Gain : { 0.8, 1.2 })
	{
		// The rear pair's wheels move a fifth less, or more, than the law is told they do.
		FRobot Robot = MakeRobot(EStance::Outdoor);
		Robot.Legs[RearPair].Gain = Gain;
		for (const FVector2D& GroundDeg : { FVector2D(4.0, 0.0), FVector2D(-4.0, 0.0), FVector2D(3.0, -3.0) })
		{
			for (int32 Rate = 0; Rate < 3; ++Rate)
			{
				FOptions Options = MakeOptions(Rate);
				Options.bEnabledAtStart = false;
				Options.SagCm = 0.8;
				Options.MaxSeconds = 20.0;
				Options.Events.Add(MakeEvent(1.0, EEvent::Enable));
				const FResult Result = DryRun::Run(Robot, Plane(GroundDeg.X, GroundDeg.Y), Options);
				const FString Case = FString::Printf(TEXT("rear travel x %.1f, parked on (%.0f, %.0f) deg, %s"), Gain, GroundDeg.X, GroundDeg.Y, FrameNames[Rate]);
				Report(*this, *Case, Rate, Result);
				ExpectSound(*this, Case, Result, Options.Tuning);
				TestTrue(Case + FString::Printf(TEXT(": level all the same (%.2f deg)"), Result.SteadyResidualDeg), Result.SteadyResidualDeg <= 0.2);
				TestTrue(Case + FString::Printf(TEXT(": still at the end (%.2f deg, legs %.3f cm)"), Result.SteadyTiltRangeDeg, Result.SteadyLegRangeCm), Result.SteadyTiltRangeDeg <= 0.05 && Result.SteadyLegRangeCm <= 0.05);
				TestEqual(Case + TEXT(": at rest at the end"), static_cast<int32>(Result.FinalStatus), static_cast<int32>(EStatus::Settled));
			}
		}
		// Up and down a 1:12 ramp at a walk: the breaks are met with a rear pair that answers wrongly, too.
		const FVector2D Knots[4] = { { 200.0, 0.0 }, { 600.0, Top }, { 900.0, Top }, { 1300.0, 0.0 } };
		const FGround	Course = Profile(MakeArrayView(Knots));
		for (int32 Rate = 0; Rate < 3; ++Rate)
		{
			FOptions Options = MakeOptions(Rate);
			Options.DistanceCm = 1600.0;
			Options.MaxSeconds = 90.0;
			Options.PitchPerAccelDeg = 0.006;
			Options.Drive.bOn = true;
			Options.Events.Add(MakeEvent(5.0, EEvent::SetSpeed, 0.0, 60.0));
			const FResult  Result = DryRun::Run(Robot, Course, Options);
			const FString  Case = FString::Printf(TEXT("rear travel x %.1f, a 1:12 ramp at 60 cm/s, %s"), Gain, FrameNames[Rate]);
			const FPassage On = PassageBetween(Result, Options.Tuning, 50.0, 1600.0, Options.FrameSeconds);
			const FPassage Steady = PassageBetween(Result, Options.Tuning, 450.0, 550.0, Options.FrameSeconds);
			Report(*this, *Case, Rate, Result);
			AddInfo(FString::Printf(TEXT("%s | rider %.2f deg at %.1f deg/s; on the ramp %.2f deg"), *Case, On.PeakDeg, On.RateDegPerSec, Steady.PeakDeg));
			ExpectSound(*this, Case, Result, Options.Tuning);
			TestTrue(Case + TEXT(": driven to its end"), Result.TravelCm >= Options.DistanceCm);
			TestTrue(Case + FString::Printf(TEXT(": the rider within 2 deg through the breaks (%.2f), turned within 4 deg/s (%.1f)"), On.PeakDeg, On.RateDegPerSec), On.PeakDeg <= 2.0 && On.RateDegPerSec <= 4.0);
			TestTrue(Case + FString::Printf(TEXT(": level on the ramp (%.2f deg)"), Steady.PeakDeg), Steady.PeakDeg <= 0.5);
			TestTrue(Case + FString::Printf(TEXT(": level again at the end (%.2f deg)"), Result.FinalTiltDeg), Result.FinalTiltDeg <= 0.3);
		}
	}
	return true;
}

// Catches: a lift moved by its rider while the robot is levelled (the lift controller ramps the lift's own command;
// the plane rides on top) tilting the rider, stepping a lift, or leaving the base off level -- the front pair let down
// and drawn up again, and both elevators let down to raise the chassis; and the tilt the rider prefers, fore and aft
// and to one side, not held in the chassis on a robot whose seat is fixed to it.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelGen1ShapeJogAndPreference,
	"Ramms.Level.Gen1Shape.LiftsJoggedAndATiltPreferred",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelGen1ShapeJogAndPreference::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1ShapeTests;

	const FRobot Robot = MakeRobot(EStance::Outdoor);
	for (int32 Rate = 0; Rate < 3; ++Rate)
	{
		{
			// Parked on a grade with a cross fall, levelled. The front pair is let down 4 cm in a second and a half
			// (it stays clear of the ground) and drawn up again; then both elevators let their wheels down 3 cm,
			// raising the chassis, at the elevators' own pace.
			FOptions Options = MakeOptions(Rate);
			Options.MaxSeconds = 26.0;
			Options.Events.Add(MakeEvent(8.0, EEvent::StanceRamp, 1.5, -4.0, FrontPair));
			Options.Events.Add(MakeEvent(11.0, EEvent::StanceRamp, 1.5, 4.0, FrontPair));
			Options.Events.Add(MakeEvent(14.0, EEvent::StanceRamp, 1.0, -3.0, DriveMinusY));
			Options.Events.Add(MakeEvent(14.0, EEvent::StanceRamp, 1.0, -3.0, DrivePlusY));
			const FResult Result = DryRun::Run(Robot, Plane(3.0, -3.0), Options);
			const FString Case = FString::Printf(TEXT("lifts jogged parked on (3, -3) deg, %s"), FrameNames[Rate]);
			Report(*this, *Case, Rate, Result);
			ExpectSound(*this, Case, Result, Options.Tuning);
			const double Front = PeakBetween(Result, Options.Tuning, 7.5, 13.5);
			const double Raised = PeakBetween(Result, Options.Tuning, 13.5, 19.0);
			AddInfo(FString::Printf(TEXT("%s | off level: %.2f deg while the front pair moved, %.2f deg while the chassis was raised; largest step of a lift's command %.3f cm"), *Case, Front, Raised, Result.MaxLegStepCm));
			// A lifted pair moved is nothing to the base: it carries nothing.
			TestTrue(Case + FString::Printf(TEXT(": the front pair let down and drawn up leaves the base level (%.2f deg)"), Front), Front <= 0.3);
			// The elevators raise the chassis at the drive wheels alone: the base is turned nose up as they go, and
			// the plane takes that back out behind them. What is left is what the legs' pace leaves.
			TestTrue(Case + FString::Printf(TEXT(": the chassis raised on its elevators stays within 1.5 deg (%.2f)"), Raised), Raised <= 1.5);
			TestTrue(Case + FString::Printf(TEXT(": no lift's command stepped by more than its pace allows (%.3f cm)"), Result.MaxLegStepCm), Result.MaxLegStepCm <= 11.0 * Options.FrameSeconds + 0.6);
			TestTrue(Case + FString::Printf(TEXT(": level at the end (%.2f deg), on the stance as jogged"), Result.FinalTiltDeg), Result.FinalTiltDeg <= 0.2);
			if (!Result.Trace.IsEmpty())
			{
				const FSample& Last = Result.Trace.Last();
				TestEqual(Case + TEXT(": the drive wheels' own command is 3 cm lower, the rider's doing"), Last.BaselineZ[DriveMinusY], Robot.Legs[DriveMinusY].StanceZ - 3.0, 0.02);
				TestEqual(Case + TEXT(": the front pair's own command is back where it was"), Last.BaselineZ[FrontPair], Robot.Legs[FrontPair].StanceZ, 0.02);
			}
			TestEqual(Case + TEXT(": at rest at the end"), static_cast<int32>(Result.FinalStatus), static_cast<int32>(EStatus::Settled));
		}
		{
			// The rider prefers to sit 2 deg back and 1.5 deg to one side: with no seat to tilt, both are the chassis'.
			FOptions Options = MakeOptions(Rate);
			Options.MaxSeconds = 22.0;
			FDryEvent Prefer = MakeEvent(8.0, EEvent::SetPreference);
			Prefer.PlaneDeg = FVector2D(2.0, -1.5);
			Options.Events.Add(Prefer);
			const FResult Result = DryRun::Run(Robot, Plane(3.0, -3.0), Options);
			const FString Case = FString::Printf(TEXT("a tilt of (2, -1.5) deg preferred, parked on (3, -3) deg, %s"), FrameNames[Rate]);
			Report(*this, *Case, Rate, Result);
			ExpectSound(*this, Case, Result, Options.Tuning);
			if (!Result.Trace.IsEmpty())
			{
				const FSample& Last = Result.Trace.Last();
				TestTrue(Case + FString::Printf(TEXT(": the chassis is at the preferred tilt (%.2f, %.2f deg)"), Last.TrueAttitudeDeg.X, Last.TrueAttitudeDeg.Y), Last.TrueAttitudeDeg.Equals(FVector2D(2.0, -1.5), 0.2));
				TestTrue(Case + TEXT(": the rider's attitude is the chassis' (no seat to tilt)"), Last.RiderDeg.Equals(Last.TrueAttitudeDeg, 1.0e-9) && Last.SeatDeg == 0.0);
			}
			TestTrue(Case + FString::Printf(TEXT(": the rider turned to it within the tilt policy (%.2f deg/s)"), Result.MaxRiderRateDegPerSec), Result.MaxRiderRateDegPerSec <= 3.6);
			TestTrue(Case + FString::Printf(TEXT(": the rider is where they asked to be (%.2f deg off)"), Result.FinalRiderTiltDeg), Result.FinalRiderTiltDeg <= 0.2);
		}
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
