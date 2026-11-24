// Tests for the level assist's whole per-tick law (Private/Level/RammsLevelLaw):
// FLaw::Step run once a frame round a toy base.
//
// Switching on with no bump and off onto the baseline exactly; a fold and an
// adopt that move nothing; the terrain estimate fed what the joints did, not
// what they were told; the hold while the attitude is missing; the rest at
// standstill; a base wired backwards caught and latched off; a slope past the
// legs' reach; a step in which no time passes; and the rules a review found
// unguarded or wrong (the seat trim, the integral's gates, each limit on the
// legs' rate, the level reference, a leg that is not a number, a release
// taken back, a correction held at its per-axis limit); a knock, and the pitch
// of a base being braked, left to pass where ground on the move is followed;
// no rest while a stance moves under the base; a release that ends without a
// step. Every number is made up -- the
// made-up six-wheeled robot of the curb's tests (corners 90 cm apart fore/aft
// and 50 across, the middle pair 54 across), on 9 cm wheels -- no robot's own.
//
// Beside each test is the one change to the law it must catch.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Level/RammsLevelAllocate.h"
#include "Level/RammsLevelLaw.h"
#include "Level/RammsLevelTypes.h"
#include "Templates/Function.h"

#include <limits>

#if WITH_DEV_AUTOMATION_TESTS

// Named, not anonymous: the other level tests share a unity file and may
// well call their own helpers the same.
namespace RammsLevelLawTests
{
	using namespace RammsLevel;

	/** The law's tick in these tests (s): a slow editor's. */
	constexpr double Frame = 1.0 / 16.0;

	/**
	 * A toy base. The ground under it has the attitude GroundDeg; the legs'
	 * joints follow their commands (baseline + offset) with a first-order lag,
	 * reaching JointShare of each offset; and the base reads the ground less
	 * the plane the joints have made -- plus it, with Sense -1, for a base
	 * wired backwards. It also does the shell's part of a fold and of an
	 * adopt.
	 */
	struct FToy
	{
		TArray<FLegReach> Legs;
		FLegValues		  Offset;
		FVector2D		  GroundDeg = FVector2D::ZeroVector;
		double			  Sense = 1.0;
		double			  JointShare = 1.0;
		double			  LagSeconds = 0.05;
		/** The attitude can be read; and reads as a number. */
		bool bReadable = true;
		bool bNumber = true;
		/** Added to the attitude as read (a spike, a sensor's error); the base
		 *  itself is not moved by it. */
		FVector2D ReadingOffsetDeg = FVector2D::ZeroVector;
		/** Underside points the law is given; none unless a test adds some. */
		TArray<FClearancePoint> Points;
		/** Spoils the legs or the points as the law is GIVEN them (the toy's
		 *  own stay good), for a plant that hands over something that is not
		 *  a number. */
		TFunction<void(TArray<FLegReach>&, TArray<FClearancePoint>&)> Spoil;
		TArray<FLegReach>											  GivenLegs;
		TArray<FClearancePoint>										  GivenPoints;

		FTuning Tuning;
		FState	State;
		/** The switch, the mode, the speed and the stick are the test's to
		 *  set; the legs and the attitude are filled in by Step. */
		FInputs In;

		/** TravelCm of travel either side of the stance on every leg. */
		explicit FToy(double TravelCm)
		{
			const FVector2D WheelXY[6] = { { 45.0, -25.0 }, { 45.0, 25.0 }, { 0.0, -27.0 }, { 0.0, 27.0 }, { -45.0, -25.0 }, { -45.0, 25.0 } };
			for (const FVector2D& Wheel : WheelXY)
			{
				FLegReach& Leg = Legs.AddDefaulted_GetRef();
				Leg.XY = Wheel;
				Leg.BaseZ = -30.0;
				Leg.MeasuredZ = Leg.BaseZ;
				Leg.MinZ = Leg.BaseZ - TravelCm;
				Leg.MaxZ = Leg.BaseZ + TravelCm;
				Leg.MaxRateCmS = 12.0;
				Leg.RadiusCm = 9.0;
				Offset.Add(0.0);
			}
			In.bEnable = true;
			// The toy's ground is changed under a base that stands where it
			// is, which no ground does: the law would (rightly) take a step of
			// it for a knock and follow it only slowly. These tests are of the
			// rules that come after, so the toy's law takes its reading for
			// the ground at once; the gate has its own tests, which switch it
			// on (GateOn).
			State.Ground.Tuning.BreakDeg = 0.0;
		}

		/** The law's own ground gate, as the robots run it. */
		void GateOn() { State.Ground.Tuning = FGroundTuning(); }

		FVector2D Attitude() const { return GroundDeg - MeasuredCorrection(Legs) * Sense; }

		/** Each leg's command: its baseline plus its offset (cm). */
		FLegValues Commands() const
		{
			FLegValues Out;
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				Out.Add(Legs[Index].BaseZ + Offset[Index]);
			}
			return Out;
		}

		/** One frame: the joints move, the law steps, the shell obeys. The
		 *  one-shot inputs (yield, resume, cold reset) are cleared after. */
		FOutputs Step(double Dt)
		{
			const double Share = LagSeconds > 0.0 ? FMath::Clamp(Dt / LagSeconds, 0.0, 1.0) : (Dt > 0.0 ? 1.0 : 0.0);
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				const double Target = Legs[Index].BaseZ + Offset[Index] * JointShare;
				Legs[Index].MeasuredZ += (Target - Legs[Index].MeasuredZ) * Share;
			}
			GivenLegs = Legs;
			GivenPoints = Points;
			if (Spoil)
			{
				Spoil(GivenLegs, GivenPoints);
			}
			In.Legs = GivenLegs;
			In.Points = GivenPoints;
			In.AttitudeDeg = bNumber ? Attitude() + ReadingOffsetDeg : FVector2D(std::numeric_limits<double>::quiet_NaN(), 0.0);
			In.bAttitudeValid = bReadable;
			const FOutputs Out = FLaw::Step(In, Dt, Tuning, State);
			if (Out.Action == EAction::Fold)
			{
				for (int32 Index = 0; Index < Legs.Num(); ++Index)
				{
					Legs[Index].BaseZ += Offset[Index];
					Offset[Index] = 0.0;
				}
			}
			if (Out.bWrite && Out.Dz.Num() == Legs.Num())
			{
				Offset = Out.Dz;
			}
			In.bYield = false;
			In.bResume = false;
			In.bResumeHasPlane = false;
			In.bColdReset = false;
			return Out;
		}

		/** Seconds of frames; returns the last frame's outputs. */
		FOutputs Run(double Seconds)
		{
			FOutputs	Out;
			const int32 Frames = FMath::Max(1, FMath::RoundToInt32(Seconds / Frame));
			for (int32 Index = 0; Index < Frames; ++Index)
			{
				Out = Step(Frame);
			}
			return Out;
		}

		/**
		 * The plant's part of a hand-back: the plane the baselines carry over
		 * the stance (every leg at StanceZ) is taken back out of them and
		 * becomes the offsets, and the law is told so on the next Step.
		 */
		void HandBack(double StanceZ)
		{
			TArray<FLegReach> Carried = Legs;
			for (FLegReach& Leg : Carried)
			{
				Leg.MeasuredZ = Leg.BaseZ;
				Leg.BaseZ = StanceZ;
			}
			const FVector2D PlaneDeg = MeasuredCorrection(Carried);
			FLegValues		Rebase;
			RigidOffsets(Carried, PlaneDeg, Rebase);
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				Legs[Index].BaseZ -= Rebase[Index];
				Offset[Index] = Rebase[Index];
			}
			In.bResume = true;
			In.bResumeHasPlane = true;
			In.ResumePlaneDeg = PlaneDeg;
			In.Mode = EMode::Allowed;
		}
	};

	/** The largest difference between two sets of leg values (cm). */
	double MaxDifference(const FLegValues& A, const FLegValues& B)
	{
		double		Most = A.Num() == B.Num() ? 0.0 : 1.0e6;
		const int32 Num = FMath::Min(A.Num(), B.Num());
		for (int32 Index = 0; Index < Num; ++Index)
		{
			Most = FMath::Max(Most, FMath::Abs(A[Index] - B[Index]));
		}
		return Most;
	}

	double MaxSize(const FLegValues& Values)
	{
		double Most = 0.0;
		for (const double Value : Values)
		{
			Most = FMath::Max(Most, FMath::Abs(Value));
		}
		return Most;
	}

	/**
	 * Watches a correction being ramped out open loop: its rate, and how fast
	 * that changes from one frame to the next, the ramp starting from rest.
	 */
	struct FRampWatch
	{
		FVector2D LastApplied = FVector2D::ZeroVector;
		FVector2D LastRate = FVector2D::ZeroVector;
		bool	  bHave = false;
		bool	  bWasOpen = false;
		double	  MostRate = 0.0;
		double	  MostAccel = 0.0;
		int32	  Frames = 0;

		void Feed(const FOutputs& Out, double Dt)
		{
			if (Out.bOpenLoop && bHave)
			{
				const FVector2D Rate = (Out.AppliedDeg - LastApplied) / Dt;
				if (!bWasOpen)
				{
					LastRate = FVector2D::ZeroVector;
				}
				MostRate = FMath::Max(MostRate, Rate.Size());
				MostAccel = FMath::Max(MostAccel, (Rate - LastRate).Size() / Dt);
				LastRate = Rate;
				++Frames;
			}
			bWasOpen = Out.bOpenLoop;
			LastApplied = Out.AppliedDeg;
			bHave = true;
		}
	};

	void ExpectState(FAutomationTestBase& Test, const TCHAR* What, const FOutputs& Out, EState State)
	{
		Test.TestEqual(FString::Printf(TEXT("%s: state"), What), static_cast<int32>(Out.State), static_cast<int32>(State));
	}

	void ExpectAction(FAutomationTestBase& Test, const TCHAR* What, const FOutputs& Out, EAction Action)
	{
		Test.TestEqual(FString::Printf(TEXT("%s: action"), What), static_cast<int32>(Out.Action), static_cast<int32>(Action));
	}

	void ExpectStatus(FAutomationTestBase& Test, const TCHAR* What, const FOutputs& Out, EStatus Status)
	{
		Test.TestEqual(FString::Printf(TEXT("%s: status"), What), static_cast<int32>(Out.Status), static_cast<int32>(Status));
	}
} // namespace RammsLevelLawTests

// Catches: the wanted attitude starting at level rather than at the attitude
// as found (the whole slope asked of the legs on the first frame), the wanted
// attitude moved on the very step that set it, or a start that keeps an old
// correction.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawEnableIsBumpless,
	"Ramms.Level.Law.EnableIsBumpless",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawEnableIsBumpless::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	FToy Toy(12.0);
	Toy.GroundDeg = FVector2D(6.0, -3.0);
	Toy.In.bEnable = false;

	FOutputs Out = Toy.Step(Frame);
	ExpectState(*this, TEXT("switched off"), Out, EState::Off);
	TestFalse(TEXT("Off: the motors are not claimed"), Out.bClaim);
	TestTrue(TEXT("Off: no offsets"), MaxSize(Out.Dz) == 0.0);
	Out = Toy.Step(Frame);
	TestFalse(TEXT("Off: nothing is written"), Out.bWrite);

	Toy.In.bEnable = true;
	Out = Toy.Step(Frame);
	ExpectState(*this, TEXT("switched on"), Out, EState::Active);
	ExpectAction(*this, TEXT("switched on"), Out, EAction::StartFromAttitude);
	TestTrue(TEXT("On: the motors are claimed"), Out.bClaim);
	TestEqual(TEXT("On: an offset for every leg"), Out.Dz.Num(), 6);
	TestTrue(TEXT("On: the first offsets are zero"), MaxSize(Out.Dz) <= 1.0e-9);
	TestNearlyEqual(TEXT("On: the wanted pitch is the pitch as found"), Out.WantedDeg.X, 6.0, 1.0e-9);
	TestNearlyEqual(TEXT("On: the wanted roll is the roll as found"), Out.WantedDeg.Y, -3.0, 1.0e-9);
	TestNearlyEqual(TEXT("On: the ground is read as it is"), Out.GroundDeg.X, 6.0, 1.0e-9);

	// From there the base is walked to level on the tilt policy: the wanted
	// attitude within its rate and acceleration, the legs with no step.
	FVector2D  LastWanted = Out.WantedDeg;
	FVector2D  LastRate = FVector2D::ZeroVector;
	FLegValues LastDz = Out.Dz;
	double	   MostRate = 0.0;
	double	   MostAccel = 0.0;
	double	   MostLegStep = 0.0;
	for (int32 Index = 0; Index < 96; ++Index)
	{
		Out = Toy.Step(Frame);
		const FVector2D Rate = (Out.WantedDeg - LastWanted) / Frame;
		MostRate = FMath::Max(MostRate, Rate.Size());
		MostAccel = FMath::Max(MostAccel, (Rate - LastRate).Size() / Frame);
		MostLegStep = FMath::Max(MostLegStep, MaxDifference(Out.Dz, LastDz));
		LastWanted = Out.WantedDeg;
		LastRate = Rate;
		LastDz = Out.Dz;
	}
	TestTrue(TEXT("The wanted attitude moves within its rate"), MostRate <= Toy.Tuning.TiltRateDegPerSec + 1.0e-6);
	TestTrue(TEXT("... and within its acceleration"), MostAccel <= Toy.Tuning.TiltAccelDegPerSec2 + 1.0e-6);
	// 3 deg/s at a 45 cm arm is 2.4 cm/s: 0.15 cm a frame. Twice that is a step.
	TestTrue(TEXT("No leg steps on the way"), MostLegStep <= 0.3);
	TestTrue(TEXT("The wanted attitude ends level"), Out.WantedDeg.Size() <= 1.0e-6);
	TestTrue(TEXT("The base ends level"), Inclination(Toy.Attitude()) <= 0.2);
	ExpectState(*this, TEXT("levelled"), Out, EState::Active);
	return true;
}

// Catches: a release that drops the offsets at once, one that ends before
// the correction is out or never ends (the integral's trim left in), a Clear
// that leaves a last offset on the motors, a release that skips the walk of
// the wanted attitude and ramps the correction straight out, and a ramp-out
// that starts or stops at full rate (the policy's acceleration not kept).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawDisableEndsOnBaseline,
	"Ramms.Level.Law.DisableEndsOnBaselineExactly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawDisableEndsOnBaseline::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	FToy Toy(12.0);
	Toy.GroundDeg = FVector2D(6.0, -3.0);
	// Joints that come up a little short leave the integral holding a trim:
	// the part of the correction no ground explains, which a release that
	// only walked the wanted attitude back would never take out.
	Toy.JointShare = 0.9;
	FOutputs Out = Toy.Run(8.0);
	TestTrue(TEXT("Levelled first"), Inclination(Toy.Attitude()) <= 0.25);
	TestTrue(TEXT("... with offsets on the legs"), MaxSize(Toy.Offset) > 3.0);

	const FVector2D WantedLevelled = Out.WantedDeg;
	Toy.In.bEnable = false;
	Out = Toy.Step(Frame);
	ExpectState(*this, TEXT("switched off"), Out, EState::Releasing);
	ExpectAction(*this, TEXT("switched off"), Out, EAction::BeginRelease);
	TestTrue(TEXT("Releasing: still claimed"), Out.bClaim);

	FVector2D LastAttitude = Toy.Attitude();
	FVector2D LastWanted = Out.WantedDeg;
	// The wanted attitude was at rest on level; the step just taken is the
	// first of its walk.
	FVector2D  LastWantedRate = (Out.WantedDeg - WantedLevelled) / Frame;
	FRampWatch Ramp;
	Ramp.Feed(Out, Frame);
	double MostRate = 0.0;
	double MostWantedRate = LastWantedRate.Size();
	double MostWantedAccel = LastWantedRate.Size() / Frame;
	bool   bCleared = false;
	bool   bClearWrote = false;
	bool   bOpenedEarly = Out.bOpenLoop;
	int32  ClosedFrames = 1;
	int32  Frames = 0;
	for (; Frames < 320 && Out.State != EState::Off; ++Frames)
	{
		Out = Toy.Step(Frame);
		MostRate = FMath::Max(MostRate, (Toy.Attitude() - LastAttitude).Size() / Frame);
		LastAttitude = Toy.Attitude();
		Ramp.Feed(Out, Frame);
		if (Out.State == EState::Releasing && !Out.bOpenLoop)
		{
			// The closed-loop part: the wanted attitude walked to the ground's.
			const FVector2D Rate = (Out.WantedDeg - LastWanted) / Frame;
			MostWantedRate = FMath::Max(MostWantedRate, Rate.Size());
			MostWantedAccel = FMath::Max(MostWantedAccel, (Rate - LastWantedRate).Size() / Frame);
			LastWantedRate = Rate;
			++ClosedFrames;
		}
		// 6.7 deg at 3 deg/s and 4 deg/s^2 is 2.98 s: for the first two
		// seconds the release has no business ramping anything out.
		bOpenedEarly = bOpenedEarly || (Out.bOpenLoop && Frames * Frame < 2.0);
		LastWanted = Out.WantedDeg;
		if (Out.Action == EAction::Clear)
		{
			bCleared = true;
			bClearWrote = Out.bWrite;
		}
	}
	ExpectState(*this, TEXT("released"), Out, EState::Off);
	TestFalse(TEXT("The release walks the wanted attitude first: not open loop while it does"), bOpenedEarly);
	TestTrue(TEXT("... for the 2.98 s the policy takes over 6.7 deg"), ClosedFrames * Frame >= 2.9 && ClosedFrames * Frame <= 3.4);
	TestTrue(TEXT("... within the policy's rate"), MostWantedRate <= Toy.Tuning.TiltRateDegPerSec + 1.0e-6);
	TestTrue(TEXT("... and its acceleration"), MostWantedAccel <= Toy.Tuning.TiltAccelDegPerSec2 + 1.0e-6);
	// What is left then (the integral's trim) is ramped out open loop, and
	// that ramp starts and stops within the policy's acceleration too.
	TestTrue(TEXT("The trim was ramped out open loop"), Ramp.Frames >= 2);
	TestTrue(TEXT("... within the policy's rate"), Ramp.MostRate <= Toy.Tuning.TiltRateDegPerSec + 1.0e-6);
	TestTrue(TEXT("... and its acceleration, from rest"), Ramp.MostAccel <= Toy.Tuning.TiltAccelDegPerSec2 + 1.0e-6);
	TestTrue(TEXT("The release took its time: 6.7 deg at 3 deg/s"), Frames * Frame > 2.0);
	TestTrue(TEXT("... and ended"), Frames * Frame < 12.0);
	// The policy's 3 deg/s, and a little for the joints catching up.
	TestTrue(TEXT("The base was given back within the tilt policy"), MostRate <= 3.6);
	TestTrue(TEXT("It ended with a Clear"), bCleared);
	TestTrue(TEXT("... which was written"), bClearWrote);
	TestFalse(TEXT("Off: the motors are let go"), Out.bClaim);
	for (int32 Index = 0; Index < Out.Dz.Num(); ++Index)
	{
		TestTrue(TEXT("Off: every offset is exactly zero"), Out.Dz[Index] == 0.0);
		TestTrue(TEXT("... on the motors too"), Toy.Offset[Index] == 0.0);
		TestTrue(TEXT("The baseline was never touched"), Toy.Legs[Index].BaseZ == -30.0);
	}
	Toy.Run(0.5);
	TestNearlyEqual(TEXT("The base is back on its stance (pitch)"), Toy.Attitude().X, 6.0, 1.0e-3);
	TestNearlyEqual(TEXT("... (roll)"), Toy.Attitude().Y, -3.0, 1.0e-3);
	return true;
}

// Catches: an adopt that puts the whole plane in the integral as well as in
// the feed-forward (twice the plane on the next frame), a fold that leaves
// the correction standing (the plane in the baselines and in the offsets),
// and either of them moving a leg.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawFoldThenAdopt,
	"Ramms.Level.Law.FoldThenAdoptIsMotionless",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawFoldThenAdopt::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// Once with whole frames, once with the yield and the resume carried out
	// in the middle of a frame (a step in which no time passes).
	const double EventDts[2] = { Frame, 0.0 };
	for (const double EventDt : EventDts)
	{
		const TCHAR* Which = EventDt > 0.0 ? TEXT("whole frames") : TEXT("mid-frame");
		FToy		 Toy(12.0);
		Toy.GroundDeg = FVector2D(5.0, 2.0);
		FOutputs Out = Toy.Run(6.0);
		TestTrue(FString::Printf(TEXT("%s: levelled first"), Which), Inclination(Toy.Attitude()) <= 0.2);
		const FLegValues Before = Toy.Commands();
		const FVector2D	 PlaneBefore = Out.AppliedDeg;
		TestTrue(FString::Printf(TEXT("%s: with a plane on the legs"), Which), PlaneBefore.Size() > 4.0);

		// The take-over.
		Toy.In.bYield = true;
		Toy.In.Mode = EMode::Taken;
		Out = Toy.Step(EventDt);
		ExpectState(*this, Which, Out, EState::Yielded);
		ExpectAction(*this, Which, Out, EAction::Fold);
		TestTrue(FString::Printf(TEXT("%s: folded, the offsets are zero"), Which), MaxSize(Out.Dz) == 0.0);
		TestTrue(FString::Printf(TEXT("%s: ... and the correction"), Which), Out.AppliedDeg.Size() == 0.0);
		TestTrue(FString::Printf(TEXT("%s: the fold moves no command"), Which), MaxDifference(Toy.Commands(), Before) == 0.0);
		for (int32 Index = 0; Index < 8; ++Index)
		{
			Out = Toy.Step(Frame);
			TestFalse(FString::Printf(TEXT("%s: nothing is written while the taker has the legs"), Which), Out.bWrite);
		}
		ExpectState(*this, Which, Out, EState::Yielded);
		TestTrue(FString::Printf(TEXT("%s: nothing moved meanwhile"), Which), MaxDifference(Toy.Commands(), Before) == 0.0);

		// The hand-back.
		Toy.HandBack(-30.0);
		Out = Toy.Step(EventDt);
		ExpectState(*this, Which, Out, EState::Active);
		ExpectAction(*this, Which, Out, EAction::Adopt);
		TestNearlyEqual(FString::Printf(TEXT("%s: the adopted plane is the one that was folded (pitch)"), Which), Out.AppliedDeg.X, PlaneBefore.X, 1.0e-6);
		TestNearlyEqual(FString::Printf(TEXT("%s: ... (roll)"), Which), Out.AppliedDeg.Y, PlaneBefore.Y, 1.0e-6);
		TestTrue(FString::Printf(TEXT("%s: the adopt's offsets are the ones on the motors"), Which), MaxDifference(Out.Dz, Toy.Offset) <= 1.0e-9);
		TestTrue(FString::Printf(TEXT("%s: the adopt moves no command"), Which), MaxDifference(Toy.Commands(), Before) <= 1.0e-9);

		// And on from there with no step: the plan allows 0.05 cm.
		double	   MostStep = 0.0;
		FLegValues Last = Toy.Commands();
		for (int32 Index = 0; Index < 48; ++Index)
		{
			Out = Toy.Step(Frame);
			MostStep = FMath::Max(MostStep, MaxDifference(Toy.Commands(), Last));
			Last = Toy.Commands();
		}
		TestTrue(FString::Printf(TEXT("%s: no leg steps after the adopt"), Which), MostStep <= 0.05);
		TestTrue(FString::Printf(TEXT("%s: the base is still level"), Which), Inclination(Toy.Attitude()) <= 0.2);
		ExpectState(*this, Which, Out, EState::Active);
	}
	return true;
}

// Catches: the terrain estimate fed the correction commanded rather than the
// one the joints measure (joints that fall short would then read as steeper
// ground, and the estimate would move with the correction).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawTerrainUsesJoints,
	"Ramms.Level.Law.TerrainUsesMeasuredJoints",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawTerrainUsesJoints::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	FToy Toy(12.0);
	Toy.GroundDeg = FVector2D(5.0, 0.0);
	// The joints reach four fifths of what they are told.
	Toy.JointShare = 0.8;
	// Driving, so the rest at standstill does not stop the integral early.
	Toy.In.SpeedCmS = 40.0;
	Toy.In.bStick = true;
	const FOutputs Out = Toy.Run(10.0);
	TestNearlyEqual(TEXT("The ground is read as 5 deg whatever the joints do"), Out.GroundDeg.X, 5.0, 0.02);
	// What the feed-forward cannot know, the integral trims: to the deadband.
	TestTrue(TEXT("The base is level to the deadband"), FMath::Abs(Toy.Attitude().X) <= Toy.Tuning.DeadbandDeg + 0.05);
	TestTrue(TEXT("... by commanding more than the ground's 5 deg"), Out.AppliedDeg.X > 5.5);
	return true;
}

// Catches: levelling on against a missing or not-a-number attitude, offsets
// dropped or moved while it is missing, the loss never reported, and a
// recovery that steps.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawHoldsWithoutAttitude,
	"Ramms.Level.Law.HoldsWhenAttitudeLost",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawHoldsWithoutAttitude::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	FToy Toy(12.0);
	Toy.GroundDeg = FVector2D(4.0, 3.0);
	Toy.In.SpeedCmS = 40.0;
	Toy.In.bStick = true;
	FOutputs		 Out = Toy.Run(6.0);
	const FLegValues Held = Out.Dz;
	TestTrue(TEXT("Levelled first"), Inclination(Toy.Attitude()) <= 0.2);

	Toy.bReadable = false;
	Out = Toy.Step(Frame);
	ExpectState(*this, TEXT("attitude lost"), Out, EState::Holding);
	ExpectAction(*this, TEXT("attitude lost"), Out, EAction::Freeze);
	ExpectStatus(*this, TEXT("attitude lost"), Out, EStatus::Holding);
	TestTrue(TEXT("Holding: the offsets are as they were"), MaxDifference(Out.Dz, Held) == 0.0);
	TestFalse(TEXT("Holding: nothing is written"), Out.bWrite);
	TestTrue(TEXT("Holding: still claimed"), Out.bClaim);
	Out = Toy.Run(1.0);
	ExpectStatus(*this, TEXT("lost for a second"), Out, EStatus::AttitudeLost);
	TestTrue(TEXT("Still held"), MaxDifference(Out.Dz, Held) == 0.0);

	Toy.bReadable = true;
	Out = Toy.Step(Frame);
	ExpectState(*this, TEXT("attitude back"), Out, EState::Active);
	ExpectAction(*this, TEXT("attitude back"), Out, EAction::ColdResume);
	TestTrue(TEXT("The recovery does not step"), MaxDifference(Out.Dz, Held) <= 1.0e-6);
	Out = Toy.Run(1.0);
	TestTrue(TEXT("... nor after"), MaxDifference(Out.Dz, Held) <= 0.05);

	// A reading that is not a number is no reading.
	Toy.bNumber = false;
	Out = Toy.Step(Frame);
	ExpectState(*this, TEXT("not a number"), Out, EState::Holding);
	// A new loss starts its own count: the second and more of the first one
	// is not held against it.
	ExpectStatus(*this, TEXT("not a number, a frame in"), Out, EStatus::Holding);
	TestTrue(TEXT("Not a number: the offsets stay numbers"), FMath::IsFinite(MaxSize(Out.Dz)));
	TestTrue(TEXT("... the ones held"), MaxDifference(Out.Dz, Held) <= 0.05);
	Toy.bNumber = true;
	Out = Toy.Run(1.0);
	ExpectState(*this, TEXT("a number again"), Out, EState::Active);
	TestTrue(TEXT("The base is level at the end"), Inclination(Toy.Attitude()) <= 0.2);
	return true;
}

// Catches: writing on at standstill (the servos never left to hold), a rest
// that begins while the base is still being walked to level, and a rest that
// does not end when the ground changes or the stick moves.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawQuietAtRest,
	"Ramms.Level.Law.QuietStopsWritingAndWakes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawQuietAtRest::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	FToy Toy(12.0);
	Toy.GroundDeg = FVector2D(6.0, 0.0);
	Toy.In.SpeedCmS = 0.0;
	Toy.In.bStick = false;

	// 6 deg takes 2.75 s on the policy: no rest before that.
	FOutputs Out = Toy.Run(2.0);
	TestFalse(TEXT("Not at rest while the base is being levelled"), Out.bQuiet);
	TestTrue(TEXT("... and writing"), Out.bWrite);

	Out = Toy.Run(4.0);
	TestTrue(TEXT("At rest once level and still"), Out.bQuiet);
	TestFalse(TEXT("At rest: nothing is written"), Out.bWrite);
	ExpectStatus(*this, TEXT("at rest"), Out, EStatus::Settled);
	ExpectState(*this, TEXT("at rest"), Out, EState::Active);
	const FLegValues Held = Out.Dz;
	TestTrue(TEXT("At rest: the offsets are still there"), MaxSize(Held) > 3.0);
	int32 Writes = 0;
	for (int32 Index = 0; Index < 32; ++Index)
	{
		Out = Toy.Step(Frame);
		Writes += Out.bWrite ? 1 : 0;
	}
	TestEqual(TEXT("At rest: no command changes"), Writes, 0);
	TestTrue(TEXT("At rest: the offsets hold"), MaxDifference(Out.Dz, Held) == 0.0);

	// The ground moves under it by a degree: more than the wake error.
	Toy.GroundDeg = FVector2D(7.0, 0.0);
	Out = Toy.Step(Frame);
	Out = Toy.Step(Frame);
	TestFalse(TEXT("A degree of error wakes it"), Out.bQuiet);
	TestTrue(TEXT("... and it writes again"), Out.bWrite);
	Out = Toy.Run(4.0);
	TestTrue(TEXT("Level again"), Inclination(Toy.Attitude()) <= 0.2);
	TestTrue(TEXT("... and at rest again"), Out.bQuiet);

	// The stick wakes it too, with no error at all.
	Toy.In.bStick = true;
	Out = Toy.Step(Frame);
	TestFalse(TEXT("The stick wakes it"), Out.bQuiet);
	TestTrue(TEXT("... and it writes"), Out.bWrite);
	return true;
}

// Catches: a divergence watch that is never consulted or never trips, a
// fault that drops the offsets at once rather than ramping them out, a latch
// that lets levelling start again by itself, and one that cannot be cleared
// by switching off and on.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawDivergenceLatches,
	"Ramms.Level.Law.DivergenceLatches",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawDivergenceLatches::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// Plenty of travel, so nothing but the sign is wrong: a correction that
	// stops at the legs' reach is a slope past reach, not a fault.
	{
		// The control: the same base wired the right way never faults.
		FToy Good(40.0);
		Good.GroundDeg = FVector2D(2.0, 0.0);
		Good.In.SpeedCmS = 40.0;
		Good.In.bStick = true;
		const FOutputs Out = Good.Run(12.0);
		ExpectState(*this, TEXT("wired right"), Out, EState::Active);
		TestTrue(TEXT("Wired right: level"), Inclination(Good.Attitude()) <= 0.2);
	}

	FToy Toy(40.0);
	Toy.GroundDeg = FVector2D(2.0, 0.0);
	Toy.Sense = -1.0;
	Toy.In.SpeedCmS = 40.0;
	Toy.In.bStick = true;

	FOutputs   Out;
	bool	   bFaulted = false;
	bool	   bRampedOpen = false;
	double	   MostTilt = 0.0;
	double	   MostStepAfterFault = 0.0;
	double	   LeastScale = 1.0;
	FLegValues Last = Toy.Commands();
	FRampWatch Ramp;
	int32	   Frames = 0;
	for (; Frames < 16 * 40 && !(Out.State == EState::Faulted && !Out.bClaim); ++Frames)
	{
		Out = Toy.Step(Frame);
		Ramp.Feed(Out, Frame);
		MostTilt = FMath::Max(MostTilt, Inclination(Toy.Attitude()));
		LeastScale = FMath::Min(LeastScale, Out.SpeedScale);
		if (bFaulted)
		{
			MostStepAfterFault = FMath::Max(MostStepAfterFault, MaxDifference(Toy.Commands(), Last));
			bRampedOpen = bRampedOpen || Out.bOpenLoop;
		}
		bFaulted = bFaulted || Out.State == EState::Faulted;
		Last = Toy.Commands();
	}
	TestTrue(TEXT("Wired backwards: it ran away"), MostTilt > 10.0);
	TestTrue(TEXT("... and was caught"), bFaulted);
	ExpectState(*this, TEXT("caught"), Out, EState::Faulted);
	ExpectStatus(*this, TEXT("caught"), Out, EStatus::Diverged);
	TestTrue(TEXT("The correction was ramped out open loop"), bRampedOpen);
	// 15 deg at the policy's 3 deg/s, started and stopped at its 4 deg/s^2.
	TestTrue(TEXT("... over the 5.75 s the policy takes"), Ramp.Frames * Frame >= 5.5 && Ramp.Frames * Frame <= 6.2);
	TestTrue(TEXT("... within the policy's rate"), Ramp.MostRate <= Toy.Tuning.TiltRateDegPerSec + 1.0e-6);
	TestTrue(TEXT("... and its acceleration, from rest"), Ramp.MostAccel <= Toy.Tuning.TiltAccelDegPerSec2 + 1.0e-6);
	// 3 deg/s at a 45 cm arm is 0.15 cm a frame: no leg was snapped back.
	TestTrue(TEXT("... with no leg snapped"), MostStepAfterFault <= 0.3);
	TestTrue(TEXT("The drive was slowed, never stopped"), LeastScale >= Toy.Tuning.MinSpeedScale - 1.0e-9 && LeastScale < 0.5);
	TestFalse(TEXT("Latched: the motors are let go"), Out.bClaim);
	TestTrue(TEXT("Latched: every offset is zero"), MaxSize(Out.Dz) == 0.0);
	TestTrue(TEXT("... on the motors too"), MaxSize(Toy.Offset) == 0.0);

	Out = Toy.Run(3.0);
	ExpectState(*this, TEXT("it stays latched"), Out, EState::Faulted);
	TestTrue(TEXT("... with no offsets"), MaxSize(Out.Dz) == 0.0);
	TestNearlyEqual(TEXT("The base is back on its stance"), Toy.Attitude().X, 2.0, 1.0e-3);

	// Off and on again is what clears it.
	Toy.In.bEnable = false;
	Out = Toy.Step(Frame);
	ExpectState(*this, TEXT("switched off"), Out, EState::Off);
	Toy.In.bEnable = true;
	Out = Toy.Step(Frame);
	ExpectState(*this, TEXT("switched on again"), Out, EState::Active);
	ExpectAction(*this, TEXT("switched on again"), Out, EAction::StartFromAttitude);
	return true;
}

// Catches: a wanted attitude that stays level on a slope the legs cannot
// take out (the error then never closes), a tilt past the policy's limit
// planned for, a slope past reach read as a divergence, a loop not told what
// the legs gave at the limit, and a speed scale that reaches zero.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawPastReach,
	"Ramms.Level.Law.PastReachIsBoundedAndReported",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawPastReach::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// 4 cm of travel either way. Planned with (0.5 margin and 2 soft zone
	// off): 1.5 cm at the 45 cm corners, 1.91 deg. Hard, to the margin:
	// 3.5 cm, 4.45 deg.
	const double PlannedReachDeg = FMath::RadiansToDegrees(FMath::Atan(1.5 / 45.0));
	const double HardReachDeg = FMath::RadiansToDegrees(FMath::Atan(3.5 / 45.0));

	FToy Toy(4.0);
	Toy.In.SpeedCmS = 40.0;
	Toy.In.bStick = true;

	// A slope a little past reach: the base keeps what the legs cannot take
	// out and the legs sit short of their margin, the whole correction asked
	// of them fitting. The base is held off level all the same, and that is
	// said for as long as it is so, with what stops the legs.
	Toy.GroundDeg = FVector2D(6.0, 0.0);
	FOutputs Out;
	int32	 NotSaid = 0;
	int32	 SaidPast = 0;
	for (int32 Index = 0; Index < 16 * 8; ++Index)
	{
		Out = Toy.Step(Frame);
		// (Switched on 6 deg off level, further than the tilt policy's 5:
		// the base is on its way in, the legs are not stopped, and "slope
		// past reach" is not yet anything to say.)
		SaidPast += Out.Status == EStatus::PastReach ? 1 : 0;
		NotSaid += Index >= 16 * 6 && !(Out.Status == EStatus::AtReach && Out.ReachLimit == EReachLimit::LegTravel) ? 1 : 0;
	}
	TestEqual(TEXT("6 deg: never said to be past reach, not even while the base is still walking in from 6 deg off"), SaidPast, 0);
	ExpectState(*this, TEXT("6 deg"), Out, EState::Active);
	ExpectStatus(*this, TEXT("6 deg"), Out, EStatus::AtReach);
	TestEqual(TEXT("6 deg: at reach, at a leg's travel, on every frame of two seconds resting there"), NotSaid, 0);
	TestTrue(TEXT("6 deg: level.reach reads none left"), Out.ReachLeft <= 0.05);
	TestNearlyEqual(TEXT("6 deg: the wanted tilt is the slope less the planned reach"), Out.WantedDeg.X, 6.0 - PlannedReachDeg, 0.05);
	TestNearlyEqual(TEXT("6 deg: the base rests on it"), Toy.Attitude().X, 6.0 - PlannedReachDeg, 0.25);
	TestTrue(TEXT("6 deg: the whole correction fits"), Out.Fraction >= 1.0);
	TestTrue(TEXT("6 deg: the speed comes back"), Out.SpeedScale > 0.9);

	// Far past reach: the planned tilt stops at the policy's 5 deg, the legs
	// stop at their margin, and the base takes the rest.
	Toy.GroundDeg = FVector2D(12.0, 0.0);
	double LeastScale = 1.0;
	bool   bFaulted = false;
	for (int32 Index = 0; Index < 16 * 10; ++Index)
	{
		Out = Toy.Step(Frame);
		LeastScale = FMath::Min(LeastScale, Out.SpeedScale);
		bFaulted = bFaulted || Out.State == EState::Faulted;
	}
	TestFalse(TEXT("12 deg: a slope past reach is not a divergence"), bFaulted);
	ExpectState(*this, TEXT("12 deg"), Out, EState::Active);
	ExpectStatus(*this, TEXT("12 deg"), Out, EStatus::PastReach);
	TestNearlyEqual(TEXT("12 deg: the wanted tilt stops at the policy's limit"), Out.WantedDeg.X, Toy.Tuning.MaxTiltDeg, 0.05);
	TestTrue(TEXT("12 deg: the correction is cut back"), Out.Fraction < 1.0);
	TestNearlyEqual(TEXT("12 deg: the legs give their hard reach"), Out.AppliedDeg.X, HardReachDeg, 0.1);
	TestNearlyEqual(TEXT("12 deg: the base takes the rest"), Toy.Attitude().X, 12.0 - HardReachDeg, 0.2);
	TestTrue(TEXT("12 deg: no leg past its margin"), MaxSize(Out.Dz) <= 3.5 + 0.01);
	// The feed-forward asks 12 - 5 = 7 deg and the error's share 0.3 (2.55 -
	// 0.15) more; the legs give 4.45. That excess is the feed-forward's, not
	// the integral's: an integral driven negative to cancel it (-3, its
	// limit) would hold the correction that far short when the slope eases.
	TestTrue(TEXT("12 deg: the integral is not made to cancel the feed-forward's excess"), FMath::Abs(Toy.State.Loop.GetIntegral().X) <= 0.05);
	TestTrue(TEXT("12 deg: the drive is slowed to the floor"), Out.SpeedScale <= Toy.Tuning.MinSpeedScale + 1.0e-6);
	TestTrue(TEXT("... and never stopped"), LeastScale >= Toy.Tuning.MinSpeedScale - 1.0e-9);

	// Steeper still: reported as a tilt fault, and levelling goes on.
	Toy.GroundDeg = FVector2D(18.0, 0.0);
	Out = Toy.Run(4.0);
	ExpectState(*this, TEXT("18 deg"), Out, EState::Active);
	ExpectStatus(*this, TEXT("18 deg"), Out, EStatus::TiltFault);

	// Back on a slope inside reach: level again, and the speed let go.
	Toy.GroundDeg = FVector2D(1.0, 0.0);
	Out = Toy.Run(10.0);
	ExpectState(*this, TEXT("1 deg"), Out, EState::Active);
	ExpectStatus(*this, TEXT("1 deg"), Out, EStatus::Levelling);
	TestTrue(TEXT("1 deg: level again"), Inclination(Toy.Attitude()) <= 0.2);
	TestTrue(TEXT("1 deg: the wanted tilt is level again"), Out.WantedDeg.Size() <= 1.0e-6);
	TestTrue(TEXT("1 deg: full speed again"), Out.SpeedScale > 0.99);

	// What the integral itself asked for IS given up at the limit. Joints
	// that reach four fifths of their commands leave it holding a trim; when
	// the slope then goes far past reach, that trim is part of what the legs
	// could not give. Left in (the integral only frozen), it would still be
	// there, pushing into the limit, when the slope eases. (Joints that reach
	// three fifths of their commands, so the trim is well over the deadband.)
	{
		FToy Short(4.0);
		Short.JointShare = 0.6;
		Short.In.SpeedCmS = 40.0;
		Short.In.bStick = true;
		Short.GroundDeg = FVector2D(6.0, 0.0);
		Short.Run(8.0);
		TestTrue(TEXT("Short joints: the integral holds a trim"), Short.State.Loop.GetIntegral().X > 0.5);
		Short.GroundDeg = FVector2D(12.0, 0.0);
		Out = Short.Run(8.0);
		TestTrue(TEXT("Short joints, 12 deg: the correction is cut back"), Out.Fraction < 1.0);
		TestTrue(TEXT("... and the integral has given its trim up"), FMath::Abs(Short.State.Loop.GetIntegral().X) <= 0.02);
	}
	return true;
}

// Catches: a step in which no time passes advancing a filter, the wanted
// attitude or the correction, or writing.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawZeroDt,
	"Ramms.Level.Law.ZeroDtAdvancesNothing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawZeroDt::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	FToy Toy(12.0);
	Toy.GroundDeg = FVector2D(6.0, -3.0);
	Toy.In.SpeedCmS = 40.0;
	Toy.In.bStick = true;
	// Part way through being walked to level: everything is moving.
	const FOutputs Before = Toy.Run(1.0);
	for (int32 Index = 0; Index < 3; ++Index)
	{
		const FOutputs Out = Toy.Step(0.0);
		ExpectState(*this, TEXT("no time"), Out, EState::Active);
		ExpectAction(*this, TEXT("no time"), Out, EAction::None);
		TestFalse(TEXT("No time: nothing is written"), Out.bWrite);
		TestTrue(TEXT("No time: the offsets hold"), MaxDifference(Out.Dz, Before.Dz) == 0.0);
		TestTrue(TEXT("No time: the wanted attitude holds"), (Out.WantedDeg - Before.WantedDeg).Size() == 0.0);
		TestTrue(TEXT("No time: the attitude filter holds"), (Out.AttitudeDeg - Before.AttitudeDeg).Size() == 0.0);
		TestTrue(TEXT("No time: the terrain estimate holds"), (Out.GroundDeg - Before.GroundDeg).Size() == 0.0);
		TestTrue(TEXT("No time: the correction holds"), (Out.AppliedDeg - Before.AppliedDeg).Size() == 0.0);
		TestTrue(TEXT("No time: the speed scale holds"), Out.SpeedScale == Before.SpeedScale);
	}
	// And time goes on from there as if they had not happened.
	const FOutputs After = Toy.Step(Frame);
	TestTrue(TEXT("Time again: it writes"), After.bWrite);
	TestTrue(TEXT("... and the wanted attitude moves on"), (After.WantedDeg - Before.WantedDeg).Size() > 0.01);
	return true;
}

// Catches: a hitch integrated whole (a one-second frame taken as a second of
// walking and winding), and a change in the number of legs carried on from
// (what was kept per leg then belongs to other legs).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawHousekeeping,
	"Ramms.Level.Law.LongStepIsClampedAndNewLegsStartCold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawHousekeeping::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// Two bases alike, part way through being walked to level. One then
	// sees a hitch of a whole second, the other the 0.1 s it is clamped to.
	FToy Hitch(12.0);
	FToy Clamped(12.0);
	for (FToy* Toy : { &Hitch, &Clamped })
	{
		Toy->GroundDeg = FVector2D(6.0, -3.0);
		Toy->In.SpeedCmS = 40.0;
		Toy->In.bStick = true;
		Toy->Run(1.0);
	}
	const FVector2D WantedBefore = Hitch.State.Tilt.Get();
	const FOutputs	Long = Hitch.Step(1.0);
	const FOutputs	Short = Clamped.Step(MaxStepSeconds);
	TestTrue(TEXT("A second's hitch moves the wanted attitude as 0.1 s does"), (Long.WantedDeg - Short.WantedDeg).Size() == 0.0);
	TestTrue(TEXT("... and the correction"), (Long.AppliedDeg - Short.AppliedDeg).Size() == 0.0);
	TestTrue(TEXT("... and the legs"), MaxDifference(Long.Dz, Short.Dz) == 0.0);
	TestTrue(TEXT("... which is not nothing"), (Long.WantedDeg - WantedBefore).Size() > 0.05);
	// A second at the rate it had reached would be well over a degree.
	TestTrue(TEXT("... nor a second's worth"), (Long.WantedDeg - WantedBefore).Size() <= Hitch.Tuning.TiltRateDegPerSec * MaxStepSeconds + 1.0e-6);

	// One leg fewer than last step: a cold start.
	TestTrue(TEXT("Offsets on the legs before the change"), MaxSize(Long.Dz) > 0.2);
	Hitch.Legs.Pop();
	Hitch.Offset.Pop();
	const FOutputs Fewer = Hitch.Step(Frame);
	ExpectState(*this, TEXT("one leg fewer"), Fewer, EState::Active);
	ExpectAction(*this, TEXT("one leg fewer"), Fewer, EAction::StartFromAttitude);
	TestEqual(TEXT("One leg fewer: an offset for each leg there is"), Fewer.Dz.Num(), 5);
	TestTrue(TEXT("... all zero"), MaxSize(Fewer.Dz) <= 1.0e-9);
	TestTrue(TEXT("... and no correction carried over"), Fewer.AppliedDeg.Size() <= 1.0e-9);
	return true;
}

// Catches: the seat trim's sign or the level reference left out of it, a trim
// that runs out while the attitude is missing (it holds with the offsets), a
// trim that ramps out after a fold (it went into the seat's own command with
// the offsets, at once), and one carried over an adopt.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawSeatTrim,
	"Ramms.Level.Law.SeatTrimThroughTheLaw",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawSeatTrim::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	const auto MakeToy = [](FToy& Toy) {
		Toy.GroundDeg = FVector2D(6.0, 0.0);
		Toy.In.bSeat = true;
		Toy.In.SpeedCmS = 40.0;
		Toy.In.bStick = true;
	};

	// Switched on nose up by 6 deg. The base is walked level at 3 deg/s; the
	// seat answers at once, nose DOWN, at its own 10 deg/s.
	{
		FToy Toy(12.0);
		MakeToy(Toy);
		FOutputs Out = Toy.Step(Frame);
		TestNearlyEqual(TEXT("The first frame: one frame of the trim's rate, nose down"), Out.SeatTrimDeg, -Toy.Tuning.SeatTrimRateDegPerSec * Frame, 1.0e-9);
		Out = Toy.Run(0.5 - Frame);
		// Half a second in the base has come back half a degree; the trim is
		// still on its rate, 5 of the 5.3 or so it is after.
		TestTrue(TEXT("Half a second in: about 5 deg nose down"), Out.SeatTrimDeg <= -4.6 && Out.SeatTrimDeg >= -5.2);
		const double BasePitchDeg = Toy.Attitude().X;
		TestTrue(TEXT("... no more than the base's pitch less the trim's deadband"), -Out.SeatTrimDeg <= BasePitchDeg - SeatTrimDeadbandDeg + 0.3);

		// The attitude is lost: the trim holds where it is, with the offsets.
		const double HeldDeg = Out.SeatTrimDeg;
		Toy.bReadable = false;
		for (int32 Index = 0; Index < 8; ++Index)
		{
			Out = Toy.Step(Frame);
			ExpectState(*this, TEXT("attitude lost"), Out, EState::Holding);
			TestTrue(TEXT("Holding: the trim holds"), Out.SeatTrimDeg == HeldDeg);
		}
		Toy.bReadable = true;

		// Levelled: the legs have taken the pitch out, so the trim has gone.
		Out = Toy.Run(8.0);
		TestTrue(TEXT("Level: the base"), Inclination(Toy.Attitude()) <= 0.2);
		TestTrue(TEXT("Level: no trim left"), FMath::Abs(Out.SeatTrimDeg) <= 0.05);
	}

	// A take-over while the trim is out: it goes into the seat's own command
	// with the offsets, so it is zero at once, not ramped; and the hand-back
	// starts it from zero again.
	{
		FToy Toy(12.0);
		MakeToy(Toy);
		FOutputs Out = Toy.Run(0.5);
		TestTrue(TEXT("Before the take-over: the trim is out"), Out.SeatTrimDeg <= -4.0);
		Toy.In.bYield = true;
		Toy.In.Mode = EMode::Taken;
		Out = Toy.Step(Frame);
		ExpectAction(*this, TEXT("take-over"), Out, EAction::Fold);
		TestTrue(TEXT("Folded: the trim is zero on that step"), Out.SeatTrimDeg == 0.0);
		Out = Toy.Run(0.5);
		TestTrue(TEXT("... and stays zero while the taker has the seat"), Out.SeatTrimDeg == 0.0);

		Toy.HandBack(-30.0);
		Out = Toy.Step(Frame);
		ExpectAction(*this, TEXT("hand-back"), Out, EAction::Adopt);
		// The base is still some 5 deg nose up, as the taker held it.
		TestTrue(TEXT("Adopted: the trim starts again from zero, a frame of its rate at most"),
			Out.SeatTrimDeg < 0.0 && Out.SeatTrimDeg >= -Toy.Tuning.SeatTrimRateDegPerSec * Frame - 1.0e-9);
	}

	// "Level" is the reference: a base standing on it wants no trim. And with
	// no seat, or the trim switched off, there is none.
	{
		FToy Toy(12.0);
		MakeToy(Toy);
		Toy.GroundDeg = FVector2D(2.0, 0.0);
		Toy.Tuning.LevelReferenceDeg = FVector2D(2.0, 0.0);
		double MostTrim = 0.0;
		for (int32 Index = 0; Index < 32; ++Index)
		{
			MostTrim = FMath::Max(MostTrim, FMath::Abs(Toy.Step(Frame).SeatTrimDeg));
		}
		TestTrue(TEXT("Standing on the reference: no trim"), MostTrim == 0.0);

		FToy NoSeat(12.0);
		MakeToy(NoSeat);
		NoSeat.In.bSeat = false;
		TestTrue(TEXT("No seat: no trim"), NoSeat.Run(1.0).SeatTrimDeg == 0.0);
		FToy Off(12.0);
		MakeToy(Off);
		Off.Tuning.bSeatCoversTransients = false;
		TestTrue(TEXT("Switched off in the tuning: no trim"), Off.Run(1.0).SeatTrimDeg == 0.0);
	}
	return true;
}

// Catches: the integral winding while the legs are already going as fast as
// they may (it would arrive with a wound-up integral to unwind as an
// overshoot), and winding on a stance of fewer than three standing legs.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawIntegralGates,
	"Ramms.Level.Law.IntegralHeldWhileRateLimitedOrOnTwoLegs",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawIntegralGates::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// Slow legs (3 cm/s: 3 deg/s of pitch at 0.8 of it) and a ground that
	// goes to 9 deg at once: three seconds of legs flat out, with degrees of
	// error all the while.
	{
		FToy Toy(12.0);
		for (FLegReach& Leg : Toy.Legs)
		{
			Leg.MaxRateCmS = 3.0;
		}
		Toy.In.SpeedCmS = 40.0;
		Toy.In.bStick = true;
		FOutputs Out = Toy.Run(1.0);
		Toy.GroundDeg = FVector2D(9.0, 0.0);
		FVector2D LastIntegral = Toy.State.Loop.GetIntegral();
		bool	  bWasLimited = false;
		int32	  HeldFrames = 0;
		int32	  MovedWhileHeld = 0;
		for (int32 Index = 0; Index < 16 * 3; ++Index)
		{
			Out = Toy.Step(Frame);
			const FVector2D Integral = Toy.State.Loop.GetIntegral();
			// This step's gate is the limit the step before reported.
			if (bWasLimited && Out.ErrorDeg > 1.0)
			{
				++HeldFrames;
				MovedWhileHeld += (Integral - LastIntegral).Size() > 0.0 ? 1 : 0;
			}
			LastIntegral = Integral;
			bWasLimited = Out.bRateLimited;
			TestTrue(TEXT("Slow legs: never at reach (this is about rate)"), Out.Fraction >= 1.0);
		}
		TestTrue(TEXT("Slow legs: rate-limited with a large error for two seconds or more"), HeldFrames >= 32);
		TestEqual(TEXT("... and the integral did not move on any such frame"), MovedWhileHeld, 0);
		Out = Toy.Run(6.0);
		TestTrue(TEXT("Slow legs: level in the end"), Inclination(Toy.Attitude()) <= 0.2);
	}

	// Two standing legs cannot hold an attitude, so there is nothing for an
	// integral to trim against: with joints that come up short the base is
	// left short. The same base on six legs is trimmed level.
	const auto Run = [](int32 StandingLegs, FVector2D& OutIntegral, double& OutErrorDeg) {
		FToy Toy(12.0);
		Toy.GroundDeg = FVector2D(4.0, 0.0);
		Toy.JointShare = 0.8;
		Toy.In.SpeedCmS = 40.0;
		Toy.In.bStick = true;
		Toy.In.bSupportGiven = true;
		for (int32 Index = 0; Index < Toy.Legs.Num(); ++Index)
		{
			// The two centre legs stand; the rest too, or carried 5 cm clear.
			const bool bStands = StandingLegs == 6 || Index == 2 || Index == 3;
			Toy.Legs[Index].bSupport = bStands;
			Toy.Legs[Index].BaselineClearanceCm = bStands ? 0.0 : 5.0;
		}
		const FOutputs Out = Toy.Run(5.0);
		OutIntegral = Toy.State.Loop.GetIntegral();
		OutErrorDeg = Out.ErrorDeg;
	};
	FVector2D Integral = FVector2D::ZeroVector;
	double	  ErrorDeg = 0.0;
	Run(2, Integral, ErrorDeg);
	TestTrue(TEXT("Two standing legs: the integral never moved"), Integral.Size() == 0.0);
	TestTrue(TEXT("... so the base is left short"), ErrorDeg > 0.4);
	Run(6, Integral, ErrorDeg);
	TestTrue(TEXT("Six standing legs: the integral holds the trim"), Integral.X > 0.5);
	TestTrue(TEXT("... and the base is level to the deadband"), ErrorDeg <= 0.2);
	return true;
}

// Catches: the speed cap's reach term not fed the legs' travel margin (the
// drive would run a leg into its limit at full speed on a steepening slope).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawSpeedNearLimit,
	"Ramms.Level.Law.SpeedDropsNearALimitWhileTheGroundChanges",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawSpeedNearLimit::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// 5 cm of travel either way: planned reach (0.5 margin and 2 soft zone
	// off) 2.5 cm at 45 cm, 3.18 deg. The ground steepens at 1 deg/s to 3.1
	// deg: inside reach, level all the way, the legs at a twelfth of their
	// rate -- nothing but the travel left (2.56 cm of 5 at the end, under
	// the 3 where the cap starts) asks the drive to slow.
	FToy Toy(5.0);
	Toy.In.SpeedCmS = 40.0;
	Toy.In.bStick = true;
	FOutputs Out = Toy.Run(1.0);
	double	 LeastScale = 1.0;
	double	 MostError = 0.0;
	double	 MostUse = 0.0;
	double	 LeastFraction = 1.0;
	bool	 bPast = false;
	while (Toy.GroundDeg.X < 3.1)
	{
		Toy.GroundDeg.X = FMath::Min(3.1, Toy.GroundDeg.X + 1.0 * Frame);
		Out = Toy.Step(Frame);
		LeastScale = FMath::Min(LeastScale, Out.SpeedScale);
		MostError = FMath::Max(MostError, Out.ErrorDeg);
		MostUse = FMath::Max(MostUse, Out.RateUse);
		LeastFraction = FMath::Min(LeastFraction, Out.Fraction);
		bPast = bPast || Out.Status == EStatus::PastReach || Out.Status == EStatus::AtReach;
	}
	for (int32 Index = 0; Index < 8; ++Index)
	{
		Out = Toy.Step(Frame);
		LeastScale = FMath::Min(LeastScale, Out.SpeedScale);
	}
	TestTrue(TEXT("The error never reached where its own term binds"), MostError < Toy.Tuning.SpeedCapErrorDeg);
	TestTrue(TEXT("... nor the legs' rate"), MostUse < Toy.Tuning.LegRateFraction);
	TestTrue(TEXT("... nor the reach"), LeastFraction >= 1.0 && !bPast);
	TestTrue(TEXT("The legs are within 3 cm of their travel"), Out.MinMarginCm < SpeedCapMarginFullCm && Out.MinMarginCm > 2.0);
	// 0.25 + 0.75 (2.56 - 0.5) / 2.5 = 0.87.
	TestTrue(FString::Printf(TEXT("The drive is slowed by the travel left, and by no more (%.3f)"), LeastScale), LeastScale <= 0.92 && LeastScale >= 0.82);
	// The ground stops changing: a second later the cap lets go.
	Out = Toy.Run(4.0);
	TestTrue(TEXT("A steady slope near the limit does not stay slow"), Out.SpeedScale > 0.99);
	return true;
}

// Catches: "level" taken as the horizontal where a reference is set -- in
// what the base is walked to, in what counts as past reach and as a tilt
// fault, and in the speed floor that goes with them.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawReference,
	"Ramms.Level.Law.LevelReferenceIsWhatLevelMeans",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawReference::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// Standing on the reference: nothing to do.
	{
		FToy Toy(12.0);
		Toy.Tuning.LevelReferenceDeg = FVector2D(2.0, -1.0);
		Toy.GroundDeg = FVector2D(2.0, -1.0);
		Toy.In.SpeedCmS = 40.0;
		Toy.In.bStick = true;
		const FOutputs Out = Toy.Run(3.0);
		ExpectStatus(*this, TEXT("on the reference"), Out, EStatus::Levelling);
		TestTrue(TEXT("On the reference: no offsets"), MaxSize(Out.Dz) <= 1.0e-6);
		TestTrue(TEXT("On the reference: the wanted attitude is the reference"), (Out.WantedDeg - Toy.Tuning.LevelReferenceDeg).Size() <= 1.0e-6);
	}
	// On flat ground the base is walked TO the reference.
	{
		FToy Toy(12.0);
		Toy.Tuning.LevelReferenceDeg = FVector2D(2.0, -1.0);
		Toy.In.SpeedCmS = 40.0;
		Toy.In.bStick = true;
		const FOutputs Out = Toy.Run(6.0);
		TestTrue(TEXT("Flat ground: the base ends on the reference"), (Toy.Attitude() - Toy.Tuning.LevelReferenceDeg).Size() <= 0.25);
		TestTrue(TEXT("... by a correction the other way"), Out.AppliedDeg.X < -1.5 && Out.AppliedDeg.Y > 0.5);
	}
	// 4 cm of travel (hard reach 4.45 deg), reference 4 deg nose DOWN, ground
	// 8 deg nose up: the legs leave the base at 3.55 deg. That is under the
	// 5 deg policy off the horizontal, but 7.55 off the reference: past reach.
	{
		FToy Toy(4.0);
		Toy.Tuning.LevelReferenceDeg = FVector2D(-4.0, 0.0);
		Toy.GroundDeg = FVector2D(8.0, 0.0);
		Toy.In.SpeedCmS = 40.0;
		Toy.In.bStick = true;
		const FOutputs Out = Toy.Run(10.0);
		TestTrue(TEXT("Reference nose down: the base is under 5 deg off the horizontal"), Out.TiltDeg < 4.0);
		ExpectStatus(*this, TEXT("reference nose down"), Out, EStatus::PastReach);
		TestNearlyEqual(TEXT("... the wanted attitude is the policy's 5 deg off the reference"), Out.WantedDeg.X, -4.0 + Toy.Tuning.MaxTiltDeg, 0.05);
		TestTrue(TEXT("... and the drive is at its floor"), Out.SpeedScale <= Toy.Tuning.MinSpeedScale + 1.0e-6);
	}
	// Reference 4 deg nose UP, ground 17: the base is left at 12.55 deg, over
	// the 12 deg fault off the horizontal but 8.55 off the reference: past
	// reach, and no tilt fault.
	{
		FToy Toy(4.0);
		Toy.Tuning.LevelReferenceDeg = FVector2D(4.0, 0.0);
		Toy.GroundDeg = FVector2D(17.0, 0.0);
		Toy.In.SpeedCmS = 40.0;
		Toy.In.bStick = true;
		const FOutputs Out = Toy.Run(10.0);
		TestTrue(TEXT("Reference nose up: the base is over 12 deg off the horizontal"), Out.TiltDeg > Toy.Tuning.FaultTiltDeg);
		ExpectStatus(*this, TEXT("reference nose up"), Out, EStatus::PastReach);
	}
	return true;
}

// Catches: the correction's own rate limit left out of the closed loop (only
// the legs' rates would then bound it, and legs with none would be snapped).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawCorrectionRate,
	"Ramms.Level.Law.CorrectionRateBinds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawCorrectionRate::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// Legs with no rate limit of their own and room to spare; the ground
	// goes to 8 deg at once.
	FToy Toy(40.0);
	for (FLegReach& Leg : Toy.Legs)
	{
		Leg.MaxRateCmS = 0.0;
	}
	Toy.In.SpeedCmS = 40.0;
	Toy.In.bStick = true;
	FOutputs Out = Toy.Run(1.0);
	Toy.GroundDeg = FVector2D(8.0, 0.0);
	FVector2D Last = Out.AppliedDeg;
	double	  MostStep = 0.0;
	int32	  LimitedFrames = 0;
	for (int32 Index = 0; Index < 32; ++Index)
	{
		Out = Toy.Step(Frame);
		MostStep = FMath::Max(MostStep, (Out.AppliedDeg - Last).Size());
		LimitedFrames += Out.bRateLimited ? 1 : 0;
		Last = Out.AppliedDeg;
	}
	const double MaxStep = Toy.Tuning.MaxCorrectionRateDegPerSec * Frame;
	TestTrue(TEXT("The correction never moves faster than its rate"), MostStep <= MaxStep + 1.0e-9);
	TestTrue(TEXT("... and did move at it"), MostStep >= MaxStep - 1.0e-9);
	// 8 deg at 10 deg/s: most of a second on the limit.
	TestTrue(TEXT("... for most of a second, and said so"), LimitedFrames >= 8);
	TestTrue(TEXT("The base is level two seconds on"), Inclination(Toy.Attitude()) <= 0.3);
	return true;
}

// Catches: the terrain estimate fed the attitude as read where the law's
// low-passed one is meant (one bad reading would go into the feed-forward at
// five times the weight).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawTerrainFiltered,
	"Ramms.Level.Law.TerrainEstimateUsesTheFilteredAttitude",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawTerrainFiltered::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// A tick short enough for the attitude filter to do something: 0.01 s is
	// a fifth of its 0.05 s and a fifteenth of the terrain's 0.15 s.
	const double Tick = 0.01;
	FToy		 Toy(12.0);
	Toy.In.SpeedCmS = 40.0;
	Toy.In.bStick = true;
	FOutputs Out;
	for (int32 Index = 0; Index < 100; ++Index)
	{
		Out = Toy.Step(Tick);
	}
	TestTrue(TEXT("Flat ground: everything at zero"), Out.GroundDeg.Size() == 0.0 && Out.AttitudeDeg.Size() == 0.0 && MaxSize(Out.Dz) == 0.0);

	// One reading 5 deg out. The legs have not moved, so the achieved
	// correction is nothing: A = 5 / 5, G = A / 15.
	Toy.ReadingOffsetDeg = FVector2D(5.0, 0.0);
	Out = Toy.Step(Tick);
	TestNearlyEqual(TEXT("The filtered attitude takes a fifth of the spike"), Out.AttitudeDeg.X, 1.0, 1.0e-9);
	TestNearlyEqual(TEXT("... and the terrain estimate a fifteenth of that"), Out.GroundDeg.X, 1.0 / 15.0, 1.0e-9);
	return true;
}

// Catches: a per-axis limit not seen as a limit -- a correction held on its
// roll cap by a steep cross fall taken for a divergence and latched off, not
// reported as past reach, the drive not slowed, the integral wound up
// against the cap.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawAxisLimit,
	"Ramms.Level.Law.AxisLimitIsReachNotADivergence",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawAxisLimit::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// Plenty of travel; roll capped at 6 deg.
	FToy Toy(40.0);
	Toy.Tuning.MaxCorrectionDeg = FVector2D(15.0, 6.0);
	Toy.In.SpeedCmS = 40.0;
	Toy.In.bStick = true;

	// 9 deg of cross fall: the cap takes 6 and the base is planned to keep 3,
	// inside the policy. That is levelling as far as it goes -- and the base
	// is 3 deg off level, so it is said: at reach, at the most correction
	// allowed.
	Toy.GroundDeg = FVector2D(0.0, 9.0);
	FOutputs Out = Toy.Run(8.0);
	ExpectState(*this, TEXT("9 deg"), Out, EState::Active);
	ExpectStatus(*this, TEXT("9 deg"), Out, EStatus::AtReach);
	TestTrue(TEXT("9 deg: what stops it is the per-axis limit"), Out.ReachLimit == EReachLimit::AxisLimit);
	TestTrue(TEXT("9 deg: level.reach reads none left"), Out.ReachLeft <= 0.01);
	TestNearlyEqual(TEXT("9 deg: the wanted roll is what the cap leaves"), Out.WantedDeg.Y, 3.0, 0.05);
	TestNearlyEqual(TEXT("9 deg: the roll correction rests on its cap"), Out.AppliedDeg.Y, 6.0, 0.05);
	TestTrue(TEXT("9 deg: full speed"), Out.SpeedScale > 0.99);

	// 15 deg: the cap takes 6, the policy lets the wanted roll go to 5, and
	// 4 deg of error stay for good -- large, steady, with every leg in reach.
	Toy.GroundDeg = FVector2D(0.0, 15.0);
	bool bFaulted = false;
	for (int32 Index = 0; Index < 16 * 12; ++Index)
	{
		Out = Toy.Step(Frame);
		bFaulted = bFaulted || Out.State == EState::Faulted;
	}
	TestFalse(TEXT("15 deg: a correction held on its cap is not a divergence"), bFaulted);
	ExpectState(*this, TEXT("15 deg"), Out, EState::Active);
	TestTrue(TEXT("15 deg: every leg has room (it is the cap that holds)"), Out.Fraction >= 1.0);
	TestTrue(TEXT("15 deg: the error is over the divergence watch's 3 deg"), Out.ErrorDeg > DivergenceErrorDeg + 0.5);
	ExpectStatus(*this, TEXT("15 deg"), Out, EStatus::PastReach);
	TestNearlyEqual(TEXT("15 deg: the wanted roll stops at the policy's limit"), Out.WantedDeg.Y, Toy.Tuning.MaxTiltDeg, 0.05);
	TestTrue(TEXT("15 deg: the roll correction is on its cap, not past it"), Out.AppliedDeg.Y <= 6.0 + 1.0e-9 && Out.AppliedDeg.Y >= 5.99);
	TestTrue(TEXT("15 deg: the drive is slowed to the floor"), Out.SpeedScale <= Toy.Tuning.MinSpeedScale + 1.0e-6);
	TestTrue(TEXT("15 deg: the roll integral is not wound up against the cap"), FMath::Abs(Toy.State.Loop.GetIntegral().Y) <= 0.05);

	// Back inside: level again, and at full speed in the end.
	Toy.GroundDeg = FVector2D(0.0, 3.0);
	Out = Toy.Run(10.0);
	ExpectStatus(*this, TEXT("3 deg"), Out, EStatus::Levelling);
	TestTrue(TEXT("3 deg: level again"), Inclination(Toy.Attitude()) <= 0.2);
	TestTrue(TEXT("3 deg: full speed again"), Out.SpeedScale > 0.99);
	return true;
}

// Catches: a leg or a clearance point that is not a number let through to
// the plane fit (the terrain estimate would then keep it for good, and the
// offsets of that step would be neither rate-limited nor a plane).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawNotANumberLeg,
	"Ramms.Level.Law.LegThatIsNotANumberHolds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawNotANumberLeg::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	const double	   Bad = std::numeric_limits<double>::quiet_NaN();
	const double	   Huge = std::numeric_limits<double>::infinity();
	const TCHAR* const Names[7] = { TEXT("a leg's x"), TEXT("a leg's y"), TEXT("a leg's travel"), TEXT("a leg's rate"), TEXT("a leg's radius"), TEXT("a leg's measured height"),
		TEXT("a point's height") };
	for (int32 Which = 0; Which < 7; ++Which)
	{
		FToy Toy(12.0);
		Toy.GroundDeg = FVector2D(4.0, 3.0);
		Toy.In.SpeedCmS = 40.0;
		Toy.In.bStick = true;
		FClearancePoint& Point = Toy.Points.AddDefaulted_GetRef();
		Point.XY = FVector2D(30.0, 0.0);
		Point.UndersideZ = -20.0;
		Point.MinCm = 3.0;
		FOutputs		 Out = Toy.Run(6.0);
		const FLegValues Held = Out.Dz;
		TestTrue(FString::Printf(TEXT("%s: levelled first"), Names[Which]), Inclination(Toy.Attitude()) <= 0.2 && MaxSize(Held) > 2.0);

		// One step with one number that is not one.
		Toy.Spoil = [Which, Bad, Huge](TArray<FLegReach>& Legs, TArray<FClearancePoint>& Points) {
			switch (Which)
			{
				case 0:
					Legs[2].XY.X = Bad;
					break;
				case 1:
					Legs[5].XY.Y = Huge;
					break;
				case 2:
					Legs[0].MaxZ = Bad;
					break;
				case 3:
					Legs[1].MaxRateCmS = Bad;
					break;
				case 4:
					Legs[3].RadiusCm = Bad;
					break;
				case 5:
					Legs[4].MeasuredZ = Bad;
					break;
				default:
					Points[0].UndersideZ = Bad;
					break;
			}
		};
		Out = Toy.Step(Frame);
		ExpectState(*this, Names[Which], Out, EState::Holding);
		TestTrue(FString::Printf(TEXT("%s: the offsets are the ones held, to the last bit"), Names[Which]), MaxDifference(Out.Dz, Held) == 0.0);
		TestFalse(FString::Printf(TEXT("%s: nothing is written"), Names[Which]), Out.bWrite);
		TestTrue(FString::Printf(TEXT("%s: the terrain estimate is still a number"), Names[Which]), FMath::IsFinite(Out.GroundDeg.X) && FMath::IsFinite(Out.GroundDeg.Y));

		// Good again: on from there with no step, and still the right ground.
		Toy.Spoil = nullptr;
		Out = Toy.Step(Frame);
		ExpectState(*this, Names[Which], Out, EState::Active);
		ExpectAction(*this, Names[Which], Out, EAction::ColdResume);
		TestTrue(FString::Printf(TEXT("%s: the recovery does not step"), Names[Which]), MaxDifference(Out.Dz, Held) <= 1.0e-6);
		Out = Toy.Run(2.0);
		TestTrue(FString::Printf(TEXT("%s: the ground is read as it is"), Names[Which]), (Out.GroundDeg - Toy.GroundDeg).Size() <= 0.05);
		TestTrue(FString::Printf(TEXT("%s: the base is level"), Names[Which]), Inclination(Toy.Attitude()) <= 0.2);
		TestTrue(FString::Printf(TEXT("%s: the offsets are numbers"), Names[Which]), FMath::IsFinite(MaxSize(Out.Dz)) && MaxDifference(Out.Dz, Held) <= 0.05);
	}
	return true;
}

// Catches: the soft zone's taper taken for a rate limit (the integral, held
// while rate-limited, could then hardly trim a leg that works inside the
// zone -- which is where a slope past the planned reach leaves it).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawSoftZoneIntegral,
	"Ramms.Level.Law.IntegralTrimsInsideTheSoftZone",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawSoftZoneIntegral::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// 4 cm of travel on 6 deg: the wanted tilt is planned to leave the corner
	// legs 1.5 cm out, on the edge of the soft zone (2 cm short of the 3.5
	// the margin allows). Joints that reach three quarters of their commands
	// need a third more command than that: every bit of the trim is made
	// inside the zone.
	FToy Toy(4.0);
	Toy.GroundDeg = FVector2D(6.0, 0.0);
	Toy.JointShare = 0.75;
	Toy.In.SpeedCmS = 40.0;
	Toy.In.bStick = true;

	FOutputs  Out;
	FVector2D LastIntegral = FVector2D::ZeroVector;
	int32	  InZoneWithError = 0;
	int32	  Trimmed = 0;
	for (int32 Index = 0; Index < 16 * 6; ++Index)
	{
		Out = Toy.Step(Frame);
		const FVector2D Integral = Toy.State.Loop.GetIntegral();
		const double	PitchError = FMath::Abs(Out.AttitudeDeg.X - Out.WantedDeg.X);
		// A leg more than 1.5 cm out has under 2.5 cm of its 4 left.
		if (Out.MinMarginCm < 2.5 && PitchError > Toy.Tuning.DeadbandDeg + 0.02 && Out.Fraction >= 1.0)
		{
			++InZoneWithError;
			Trimmed += Integral.X > LastIntegral.X ? 1 : 0;
		}
		LastIntegral = Integral;
	}
	TestTrue(TEXT("There was trimming to do inside the soft zone"), InZoneWithError >= 8);
	TestTrue(FString::Printf(TEXT("The integral moved on every such frame but the odd one (%d of %d)"), Trimmed, InZoneWithError), Trimmed >= InZoneWithError - 2);
	// (The base is held 4 deg off level: at reach, whatever fits.)
	ExpectStatus(*this, TEXT("at the end"), Out, EStatus::AtReach);
	TestTrue(TEXT("The base rests on the wanted tilt, to the deadband"), FMath::Abs(Toy.Attitude().X - Out.WantedDeg.X) <= Toy.Tuning.DeadbandDeg + 0.05);
	TestTrue(TEXT("... by a correction past the planned reach, inside the zone"), Out.AppliedDeg.X > 2.3 && Out.MinMarginCm < 2.5);
	return true;
}

// Catches: offsets that move faster than the legs may when it is not the
// plane that moves them -- the height term stepping as a leg leaves the
// support set, and the cut at reach falling in one step.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawLegRateLastWord,
	"Ramms.Level.Law.LegRateHoldsThroughASupportChangeAndACut",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawLegRateLastWord::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// 0.8 of 12 cm/s, a frame of it.
	const double MaxStepCm = 0.8 * 12.0 * Frame;

	// Levelled on 6 deg: the front legs 4.73 cm in, the rear 4.73 out. A
	// front leg's baseline is then drawn up 3 cm (a stance that lifts it):
	// it leaves the support set, the mean over the five that stand moves by
	// 4.73 / 5, and every leg's offset wants to step 0.95 cm with it.
	{
		FToy Toy(12.0);
		Toy.GroundDeg = FVector2D(6.0, 0.0);
		Toy.In.SpeedCmS = 40.0;
		Toy.In.bStick = true;
		FOutputs		 Out = Toy.Run(6.0);
		const FLegValues Before = Out.Dz;
		Toy.Legs[0].BaseZ += 3.0;
		Toy.Legs[0].MeasuredZ += 3.0;
		FLegValues Last = Before;
		double	   MostStep = 0.0;
		double	   LeastScale = 1.0;
		bool	   bSaid = false;
		for (int32 Index = 0; Index < 16; ++Index)
		{
			Out = Toy.Step(Frame);
			MostStep = FMath::Max(MostStep, MaxDifference(Out.Dz, Last));
			LeastScale = FMath::Min(LeastScale, Out.SpeedScale);
			bSaid = bSaid || (Index == 0 && Out.bRateLimited && Out.RateUse > 1.0);
			Last = Out.Dz;
		}
		TestTrue(TEXT("A leg left the support set: the height term did move"), FMath::Abs(Out.Dz[5] - Before[5]) > 0.9);
		TestTrue(FString::Printf(TEXT("... and no leg moved faster than 0.8 of its rate (%.3f cm a frame at most, %.3f allowed)"), MostStep, MaxStepCm), MostStep <= MaxStepCm + 1.0e-9);
		TestTrue(TEXT("... which the step said, with the rate the jump asked"), bSaid);
		// Offsets catching up with a height term are not the ground asking
		// more than the legs can give: nothing for the drive to slow for.
		TestTrue(TEXT("... and the drive was not slowed for it"), LeastScale >= 1.0 - 1.0e-9);
		TestTrue(TEXT("The base is still level"), Inclination(Toy.Attitude()) <= 0.3);
	}

	// Levelled on 6 deg again; then the reach is cut under the legs in one
	// step (the margin widened, as a stand-in for a limit that moves): the
	// share that fits falls to 2.5 of the 4.73 cm at once.
	{
		FToy Toy(12.0);
		Toy.GroundDeg = FVector2D(6.0, 0.0);
		Toy.In.SpeedCmS = 40.0;
		Toy.In.bStick = true;
		FOutputs Out = Toy.Run(6.0);
		Toy.Tuning.LegRangeMarginCm = 9.5;
		FLegValues Last = Out.Dz;
		double	   MostStep = 0.0;
		for (int32 Index = 0; Index < 16; ++Index)
		{
			Out = Toy.Step(Frame);
			MostStep = FMath::Max(MostStep, MaxDifference(Out.Dz, Last));
			Last = Out.Dz;
		}
		TestTrue(TEXT("The reach was cut: the legs came back to it"), Out.Fraction < 1.0 && MaxSize(Out.Dz) <= 2.5 + 0.01);
		TestTrue(FString::Printf(TEXT("... and no leg faster than 0.8 of its rate (%.3f cm a frame at most, %.3f allowed)"), MostStep, MaxStepCm), MostStep <= MaxStepCm + 1.0e-9);
	}
	return true;
}

// Catches: levelling taken up again part way through a release's open-loop
// ramp (the switch thrown back on; the attitude lost and found) with the
// loop as it was before the ramp -- its integral would ask again, at once,
// for what was just ramped out.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawReleaseTakenBack,
	"Ramms.Level.Law.ReleaseTakenBackGoesOnWithoutAStep",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawReleaseTakenBack::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// Joints that reach 0.7 of their commands: the integral holds 2.6 deg of
	// trim, which a release's walk leaves to be ramped out open loop.
	const auto MakeToy = [](FToy& Toy) {
		Toy.GroundDeg = FVector2D(6.0, 0.0);
		Toy.JointShare = 0.7;
		Toy.In.SpeedCmS = 40.0;
		Toy.In.bStick = true;
		Toy.Run(8.0);
	};
	// The most any leg's command moves in a frame over the next Frames.
	const auto MostStepOver = [](FToy& Toy, int32 Frames, FOutputs& Out) {
		double	   Most = 0.0;
		FLegValues Last = Toy.Commands();
		for (int32 Index = 0; Index < Frames; ++Index)
		{
			Out = Toy.Step(Frame);
			Most = FMath::Max(Most, MaxDifference(Toy.Commands(), Last));
			Last = Toy.Commands();
		}
		return Most;
	};

	// Switched off, and on again three frames into the open-loop ramp.
	{
		FToy Toy(12.0);
		MakeToy(Toy);
		TestTrue(TEXT("Levelled, with a trim in the integral"), Inclination(Toy.Attitude()) <= 0.25 && Toy.State.Loop.GetIntegral().X > 1.5);
		Toy.In.bEnable = false;
		FOutputs Out;
		int32	 OpenFrames = 0;
		for (int32 Index = 0; Index < 200 && OpenFrames < 3; ++Index)
		{
			Out = Toy.Step(Frame);
			OpenFrames += Out.bOpenLoop ? 1 : 0;
		}
		TestEqual(TEXT("The release reached its open-loop ramp"), OpenFrames, 3);
		ExpectState(*this, TEXT("ramping out"), Out, EState::Releasing);
		TestTrue(TEXT("... with correction still to go"), Out.AppliedDeg.X > 1.0);

		Toy.In.bEnable = true;
		const FVector2D AppliedBefore = Out.AppliedDeg;
		Out = Toy.Step(Frame);
		ExpectState(*this, TEXT("switched back on"), Out, EState::Active);
		ExpectAction(*this, TEXT("switched back on"), Out, EAction::None);
		TestFalse(TEXT("Switched back on: closed loop again"), Out.bOpenLoop);
		TestTrue(TEXT("... from the correction as it stood"), (Out.AppliedDeg - AppliedBefore).Size() <= 0.02);
		// The stale integral would ask for some 2 deg back at 10 deg/s: half
		// a centimetre a frame at the corners. (Four frames: after that the
		// walk back to level is itself up to a tenth of a centimetre a frame.)
		const double MostStep = MostStepOver(Toy, 4, Out);
		TestTrue(FString::Printf(TEXT("... and no leg steps (%.3f cm a frame at most)"), MostStep), MostStep <= 0.1);
		Out = Toy.Run(8.0);
		ExpectState(*this, TEXT("levelling again"), Out, EState::Active);
		TestTrue(TEXT("Level again"), Inclination(Toy.Attitude()) <= 0.25);
	}

	// Switched off; the attitude is lost for 12 frames of the release (the
	// correction ramped out open loop meanwhile) and then read again, while
	// the wanted attitude is still on its way: the walk goes on.
	{
		FToy Toy(12.0);
		MakeToy(Toy);
		Toy.In.bEnable = false;
		FOutputs Out = Toy.Step(Frame);
		Out = Toy.Step(Frame);
		TestFalse(TEXT("A release starts closed loop"), Out.bOpenLoop);
		Toy.bReadable = false;
		const FVector2D AppliedBefore = Out.AppliedDeg;
		for (int32 Index = 0; Index < 12; ++Index)
		{
			Out = Toy.Step(Frame);
			ExpectState(*this, TEXT("released blind"), Out, EState::Releasing);
			TestTrue(TEXT("Released blind: open loop"), Out.bOpenLoop);
		}
		TestTrue(TEXT("Released blind: the ramp took a degree out"), (Out.AppliedDeg - AppliedBefore).Size() > 0.9);
		Toy.bReadable = true;
		const double MostStep = MostStepOver(Toy, 4, Out);
		ExpectState(*this, TEXT("reading again"), Out, EState::Releasing);
		TestFalse(TEXT("Reading again: closed loop again"), Out.bOpenLoop);
		TestTrue(FString::Printf(TEXT("... and no leg steps (%.3f cm a frame at most)"), MostStep), MostStep <= 0.1);
		for (int32 Index = 0; Index < 320 && Out.State != EState::Off; ++Index)
		{
			Out = Toy.Step(Frame);
		}
		ExpectState(*this, TEXT("released in the end"), Out, EState::Off);
		TestTrue(TEXT("... onto the stance"), MaxSize(Toy.Offset) == 0.0);
	}
	return true;
}

// Catches: the offsets' common height stepping as fast as the legs can go
// when a leg leaves or joins the support set. On legs fast enough to make the
// whole step inside one frame the leg-rate limit never binds, every standing
// leg is told the step at once, and legs that do not answer alike pitch the
// base with it: the ride height has its own, gentler rate.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawRideHeightSlews,
	"Ramms.Level.Law.RideHeightSlewsThroughASupportChange",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawRideHeightSlews::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// Legs that move 60 cm/s: 0.8 of that is 3 cm a frame, more than any
	// height term here.
	FToy Toy(12.0);
	for (FLegReach& Leg : Toy.Legs)
	{
		Leg.MaxRateCmS = 60.0;
	}
	Toy.GroundDeg = FVector2D(6.0, 0.0);
	Toy.In.SpeedCmS = 40.0;
	Toy.In.bStick = true;
	FOutputs Out = Toy.Run(6.0);
	TestTrue(TEXT("Levelled on 6 deg"), Inclination(Toy.Attitude()) <= 0.3);

	// The ride height's rate, a frame of it; and a hair for the plane, which
	// stands still here. It is picked up over RideHeightRampSeconds: the
	// first frame's move is a frame's worth of that rise in speed.
	const double MaxStepCm = Toy.Tuning.RideHeightRateCmPerSec * Frame + 0.01;
	const double MaxFirstCm = Toy.Tuning.RideHeightRateCmPerSec * (Frame / RideHeightRampSeconds) * Frame + 0.01;
	// A front leg's baseline drawn up 3 cm leaves the support set, and let
	// down again joins it: either way the mean over the legs that stand
	// moves by 4.73 / 5 = 0.95 cm.
	for (int32 Pass = 0; Pass < 2; ++Pass)
	{
		const TCHAR*	 What = Pass == 0 ? TEXT("a leg leaves the support set") : TEXT("a leg joins the support set");
		const FLegValues Before = Out.Dz;
		const double	 Lift = Pass == 0 ? 3.0 : -3.0;
		Toy.Legs[0].BaseZ += Lift;
		Toy.Legs[0].MeasuredZ += Lift;
		FLegValues Last = Before;
		double	   MostStep = 0.0;
		double	   FirstStep = 0.0;
		double	   LastStep = 0.0;
		bool	   bSaid = false;
		for (int32 Index = 0; Index < 16; ++Index)
		{
			Out = Toy.Step(Frame);
			LastStep = MaxDifference(Out.Dz, Last);
			MostStep = FMath::Max(MostStep, LastStep);
			FirstStep = Index == 0 ? LastStep : FirstStep;
			bSaid = bSaid || (Index == 0 && Out.bRateLimited);
			Last = Out.Dz;
		}
		TestTrue(FString::Printf(TEXT("%s: the height term did move"), What), FMath::Abs(Out.Dz[5] - Before[5]) > 0.9);
		TestTrue(FString::Printf(TEXT("%s: ... at the ride height's rate, not the legs' (%.3f cm a frame at most, %.3f allowed)"), What, MostStep, MaxStepCm), MostStep <= MaxStepCm);
		TestTrue(FString::Printf(TEXT("%s: ... picking that rate up from rest (%.3f cm in the first frame, %.3f allowed)"), What, FirstStep, MaxFirstCm), FirstStep <= MaxFirstCm && FirstStep > 0.0);
		TestTrue(FString::Printf(TEXT("%s: ... and at rest again within the second (%.4f cm in the last frame)"), What, LastStep), LastStep <= 0.01 && !Out.bRateLimited);
		TestTrue(FString::Printf(TEXT("%s: ... which the step said"), What), bSaid);
		TestTrue(FString::Printf(TEXT("%s: the base is still level"), What), Inclination(Toy.Attitude()) <= 0.3);
		Out = Toy.Run(2.0);
	}

	// With no ride-height rate the legs' own rate is the only limit, as it
	// was: the step is made at once.
	{
		Toy.Tuning.RideHeightRateCmPerSec = 0.0;
		const FLegValues Before = Out.Dz;
		Toy.Legs[0].BaseZ += 3.0;
		Toy.Legs[0].MeasuredZ += 3.0;
		Out = Toy.Step(Frame);
		TestTrue(TEXT("No ride-height rate: the legs' rate alone limits the step"), FMath::Abs(Out.Dz[5] - Before[5]) > 0.9);
	}
	return true;
}

// Catches: a release that waits out its time limit whenever the integral
// holds a trim and the ground estimate is not perfectly still. The walk back
// ends when the wanted attitude has reached the ground's; "reached" asked of a
// live estimate as "exactly on it and not moving" never comes true on a real
// base, whose attitude always moves in its last digits.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawReleaseLiveGround,
	"Ramms.Level.Law.ReleaseArrivesOnAGroundEstimateThatIsNeverStill",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawReleaseLiveGround::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// Joints that reach 0.7 of their commands: the integral holds 2.6 deg of
	// trim. The attitude reads 0.003 deg high and low by turns, as a
	// simulator's does.
	FToy Toy(12.0);
	Toy.GroundDeg = FVector2D(6.0, 0.0);
	Toy.JointShare = 0.7;
	Toy.In.SpeedCmS = 40.0;
	Toy.In.bStick = true;
	Toy.Run(8.0);
	TestTrue(TEXT("Levelled, with a trim in the integral"), Inclination(Toy.Attitude()) <= 0.25 && Toy.State.Loop.GetIntegral().X > 1.5);

	Toy.In.bEnable = false;
	FOutputs   Out;
	double	   OpenAt = -1.0;
	double	   OffAt = -1.0;
	FRampWatch Watch;
	for (int32 Index = 0; Index < 160 && OffAt < 0.0; ++Index)
	{
		Toy.ReadingOffsetDeg = FVector2D((Index % 2 == 0) ? 0.003 : -0.003, (Index % 3 == 0) ? -0.002 : 0.002);
		Out = Toy.Step(Frame);
		Watch.Feed(Out, Frame);
		if (OpenAt < 0.0 && Out.bOpenLoop)
		{
			OpenAt = (Index + 1) * Frame;
		}
		if (Out.State == EState::Off)
		{
			OffAt = (Index + 1) * Frame;
		}
	}
	// 6 deg back at 3 deg/s and 4 deg/s^2 is 2.75 s; then the trim, 2.6 deg
	// of it, ramped out on the same policy in some 1.7 s.
	TestTrue(FString::Printf(TEXT("The walk back ended when the wanted attitude reached the ground's (%.2f s), not at the time limit"), OpenAt), OpenAt > 2.0 && OpenAt <= 3.5);
	TestTrue(FString::Printf(TEXT("... and the trim was ramped out after it (off %.2f s after the switch)"), OffAt), OffAt > OpenAt && OffAt <= 5.5);
	TestTrue(FString::Printf(TEXT("... within the tilt policy (%.2f deg/s, %.2f deg/s2)"), Watch.MostRate, Watch.MostAccel), Watch.MostRate <= 3.0 + 1.0e-6 && Watch.MostAccel <= 4.0 + 1.0e-6);
	TestTrue(TEXT("... onto the stance"), MaxSize(Toy.Offset) == 0.0);
	return true;
}

// Catches: a robot parked on a slope past its legs' reach that is written to
// for ever. There the wanted tilt's goal is worked out from the ground
// estimate, which on a real base never stands quite still; the rest must not
// wait for a goal that does.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawRestLiveGoal,
	"Ramms.Level.Law.RestsPastReachOnAGroundEstimateThatIsNeverStill",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawRestLiveGoal::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// 4 cm of travel: 6 deg is 4.09 deg past what is planned of the legs,
	// and the base keeps that. Parked; the attitude reads 0.003 deg high and
	// low by turns.
	FToy Toy(4.0);
	Toy.GroundDeg = FVector2D(6.0, 0.0);
	FOutputs Out;
	int32	 Writes = 0;
	bool	 bGoalMoved = false;
	double	 LastGround = 0.0;
	for (int32 Index = 0; Index < 16 * 9; ++Index)
	{
		Toy.ReadingOffsetDeg = FVector2D((Index % 2 == 0) ? 0.003 : -0.003, 0.0);
		Out = Toy.Step(Frame);
		if (Index >= 16 * 7)
		{
			Writes += Out.bWrite ? 1 : 0;
			bGoalMoved = bGoalMoved || (Index > 16 * 7 && Out.GroundDeg.X != LastGround);
			LastGround = Out.GroundDeg.X;
		}
	}
	ExpectState(*this, TEXT("parked past reach"), Out, EState::Active);
	TestTrue(TEXT("The base keeps what the legs cannot take out"), Out.WantedDeg.X > 3.5 && FMath::Abs(Toy.Attitude().X - Out.WantedDeg.X) <= 0.3);
	TestTrue(TEXT("The ground estimate does go on moving in its last digits"), bGoalMoved);
	TestTrue(TEXT("... and the robot rests all the same"), Out.bQuiet);
	// (At rest, and held off level by the legs' reach: the second is what
	// the rider is told.)
	ExpectStatus(*this, TEXT("parked past reach"), Out, EStatus::AtReach);
	TestTrue(TEXT("... at a leg's travel"), Out.ReachLimit == EReachLimit::LegTravel);
	TestEqual(TEXT("... with nothing written in its last two seconds"), Writes, 0);
	return true;
}

// Catches: the terrain estimate taking legs that do not carry the robot into
// the correction it believes was made -- found from the baselines, or told by
// the plant -- so that a raised leg's position reads as ground.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawTerrainCarryingLegs,
	"Ramms.Level.Law.TerrainIsReadOffTheCarryingLegs",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawTerrainCarryingLegs::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// The base reads 1 deg nose up; the four legs it stands on have made a
	// correction of (2, -1): the ground is (3, -1). The front pair is
	// somewhere else altogether.
	const FVector2D Made(2.0, -1.0);
	const FVector2D Reads(1.0, 0.0);
	for (int32 Pass = 0; Pass < 2; ++Pass)
	{
		const TCHAR* What = Pass == 0 ? TEXT("raised by its baseline") : TEXT("said not to stand");
		FToy		 Toy(12.0);
		for (int32 Index = 0; Index < Toy.Legs.Num(); ++Index)
		{
			FLegReach& Leg = Toy.Legs[Index];
			Leg.MeasuredZ = Leg.BaseZ + PlaneRise(Leg.XY, true, Made);
			if (Index < 2)
			{
				if (Pass == 0)
				{
					Leg.BaseZ += 6.0;
				}
				else
				{
					Leg.bSupport = false;
				}
				Leg.MeasuredZ = Leg.BaseZ + (Index == 0 ? 0.3 : -2.0);
			}
		}
		FInputs In;
		In.bEnable = true;
		In.Legs = Toy.Legs;
		In.bSupportGiven = Pass == 1;
		In.AttitudeDeg = Reads;
		In.bAttitudeValid = true;
		FState		   State;
		const FOutputs Out = FLaw::Step(In, Frame, Toy.Tuning, State);
		ExpectAction(*this, What, Out, EAction::StartFromAttitude);
		TestTrue(FString::Printf(TEXT("%s: the ground is the attitude and the standing legs' plane (%.3f, %.3f)"), What, Out.GroundDeg.X, Out.GroundDeg.Y),
			(Out.GroundDeg - (Reads + Made)).Size() <= 1.0e-6);
	}
	return true;
}

namespace RammsLevelLawTests
{
	/**
	 * How far the legs' offsets went from where they stood (cm, the most over
	 * the legs and the frames) through Frames frames in which the attitude
	 * READS KnockPitchDeg[i] off what the base is doing (0 past the end), with
	 * the base's speed SpeedsCmS[i] (the last one held; left alone when empty).
	 */
	double OffsetsMovedThrough(FToy& Toy, TConstArrayView<double> KnockPitchDeg, TConstArrayView<double> SpeedsCmS, int32 Frames)
	{
		const FLegValues Before = Toy.Offset;
		double			 Most = 0.0;
		for (int32 Index = 0; Index < Frames; ++Index)
		{
			Toy.ReadingOffsetDeg = FVector2D(KnockPitchDeg.IsValidIndex(Index) ? KnockPitchDeg[Index] : 0.0, 0.0);
			if (SpeedsCmS.Num() > 0)
			{
				Toy.In.SpeedCmS = SpeedsCmS[FMath::Min(Index, SpeedsCmS.Num() - 1)];
			}
			Toy.Step(Frame);
			Most = FMath::Max(Most, MaxDifference(Toy.Offset, Before));
		}
		Toy.ReadingOffsetDeg = FVector2D::ZeroVector;
		return Most;
	}
} // namespace RammsLevelLawTests

// Catches: a knock taken for ground (the legs sent after an attitude that
// comes back by itself, so the knock is answered by a swing the other way),
// and, the other way about, ground that turns under a robot on the move
// followed any differently for the gate.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawKnock,
	"Ramms.Level.Law.AKnockAtStandstillIsLeftToPass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawKnock::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// Level and at rest on 4 deg; then the base reads 1.5 deg nose down for a
	// quarter of a second, and is back.
	const double Knock[4] = { -1.5, -1.5, -1.5, -1.5 };
	double		 Moved[2] = { 0.0, 0.0 };
	for (int32 Pass = 0; Pass < 2; ++Pass)
	{
		FToy Toy(12.0);
		if (Pass == 1)
		{
			Toy.GateOn();
		}
		Toy.GroundDeg = FVector2D(4.0, 0.0);
		FOutputs Out = Toy.Run(8.0);
		ExpectStatus(*this, Pass == 0 ? TEXT("no gate, before the knock") : TEXT("before the knock"), Out, EStatus::Settled);
		Moved[Pass] = OffsetsMovedThrough(Toy, MakeArrayView(Knock), TConstArrayView<double>(), 32);
		Out = Toy.Run(3.0);
		TestTrue(Pass == 0 ? TEXT("no gate: level again after it") : TEXT("level again after it"), Inclination(Toy.Attitude()) <= 0.25);
	}
	AddInfo(FString::Printf(TEXT("a 1.5 deg knock of 0.25 s at standstill: the legs' offsets moved %.2f cm with the reading taken for the ground, %.2f cm as the law is"), Moved[0], Moved[1]));
	// (A degree at the corners' 45 cm is 0.79 cm.)
	TestTrue(TEXT("(taken for the ground, the knock is chased: most of a centimetre at a leg)"), Moved[0] >= 0.6);
	TestTrue(TEXT("left to pass: the legs hardly move"), Moved[1] <= 0.3);
	TestTrue(TEXT("... a third of the chase or less"), Moved[1] <= Moved[0] / 3.0);

	// Ground that really turns under a robot on the move: 3 deg in half a
	// second at 40 cm/s, well inside what a grade break does there. Followed
	// with the gate exactly as without: the same offsets on every frame.
	FToy With(12.0);
	FToy Without(12.0);
	With.GateOn();
	double Differ = 0.0;
	double Turned = 0.0;
	for (FToy* Toy : { &With, &Without })
	{
		Toy->GroundDeg = FVector2D(4.0, 0.0);
		Toy->In.SpeedCmS = 40.0;
		Toy->In.bStick = true;
		Toy->Run(6.0);
	}
	for (int32 Index = 0; Index < 48; ++Index)
	{
		const FVector2D Ground(4.0 + 6.0 * Frame * FMath::Min(Index + 1, 8), 0.0);
		With.GroundDeg = Ground;
		Without.GroundDeg = Ground;
		const FOutputs A = With.Step(Frame);
		const FOutputs B = Without.Step(Frame);
		Differ = FMath::Max(Differ, MaxDifference(A.Dz, B.Dz));
		Turned = FMath::Max(Turned, MaxSize(A.Dz));
	}
	TestTrue(TEXT("(the ground did turn, and the legs went with it)"), Turned > 4.0);
	TestTrue(FString::Printf(TEXT("ground turning under a robot on the move is followed as if there were no gate (%.3g cm apart at most)"), Differ), Differ <= 1.0e-9);
	return true;
}

// Catches: the pitch of a base being braked taken for the ground (a hard
// stop from speed chased as it is at standstill), the travel's allowance kept
// while it is braked, or the settling one.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawBraked,
	"Ramms.Level.Law.ABaseBeingBrakedIsNotReadForTheGround",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawBraked::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// Cruising level at 60 cm/s on 4 deg; the stick is let go, the speed
	// falls away in a quarter of a second and the base pitches nose down
	// with it and comes back. The first frame is still at speed: by travel
	// alone the ground could have turned as far as the base read.
	const double Speeds[12] = { 60.0, 32.0, 17.0, 9.0, 5.0, 2.6, 1.4, 0.7, 0.4, 0.2, 0.1, 0.0 };
	const double Knock[7] = { 0.0, -1.2, -1.27, -1.0, -0.6, -0.3, -0.1 };
	double		 Moved[2] = { 0.0, 0.0 };
	for (int32 Pass = 0; Pass < 2; ++Pass)
	{
		FToy Toy(12.0);
		if (Pass == 1)
		{
			Toy.GateOn();
		}
		Toy.GroundDeg = FVector2D(4.0, 0.0);
		Toy.In.SpeedCmS = 60.0;
		Toy.In.bStick = true;
		Toy.Run(6.0);
		TestTrue(TEXT("level at speed before the stop"), Inclination(Toy.Attitude()) <= 0.2);
		Toy.In.bStick = false;
		Moved[Pass] = OffsetsMovedThrough(Toy, MakeArrayView(Knock), MakeArrayView(Speeds), 32);
		Toy.Run(3.0);
		TestTrue(TEXT("level again after the stop"), Inclination(Toy.Attitude()) <= 0.25);
	}
	AddInfo(FString::Printf(TEXT("a stop from 60 cm/s that pitches the base 1.3 deg: the legs' offsets moved %.2f cm with the reading taken for the ground, %.2f cm as the law is"), Moved[0], Moved[1]));
	TestTrue(TEXT("(taken for the ground, the stop's pitch is chased)"), Moved[0] >= 0.5);
	TestTrue(TEXT("a base being braked is not read: the legs hardly move"), Moved[1] <= 0.2);
	TestTrue(TEXT("... a quarter of the chase or less"), Moved[1] <= Moved[0] / 4.0);
	return true;
}

// Catches: rest entered, or kept, while something moves the stance under the
// offsets (nothing is then written, and the base goes off level with the
// baselines while the status reads "at rest"); a leg that was only let go
// taken for a stance being moved.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawNoRestWhileMoving,
	"Ramms.Level.Law.NoRestWhileTheBaselinesMove",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawNoRestWhileMoving::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	FToy Toy(12.0);
	Toy.GateOn();
	Toy.GroundDeg = FVector2D(3.0, -2.0);
	FOutputs Out = Toy.Run(8.0);
	ExpectStatus(*this, TEXT("parked and level"), Out, EStatus::Settled);
	TestTrue(TEXT("parked and level: at rest, nothing written"), Out.bQuiet && !Out.bWrite);
	const auto MoveAll = [&Toy](double Cm) {
		for (FLegReach& Leg : Toy.Legs)
		{
			Leg.BaseZ += Cm;
			Leg.MinZ += Cm;
			Leg.MaxZ += Cm;
		}
	};
	// How long until it is at rest again, in frames (-1: not within three seconds).
	const auto FramesToRest = [&Toy]() {
		for (int32 Index = 1; Index <= 48; ++Index)
		{
			if (Toy.Step(Frame).bQuiet)
			{
				return Index;
			}
		}
		return -1;
	};

	// A ride height being ramped under it, 0.05 cm a frame. (In the toy that
	// turns nothing: the error stays far under the wake error, so it is the
	// baselines' moving that must end the rest.)
	int32  QuietFrames = 0;
	int32  NotWritten = 0;
	double MostError = 0.0;
	for (int32 Index = 0; Index < 16; ++Index)
	{
		MoveAll(0.05);
		Out = Toy.Step(Frame);
		QuietFrames += Out.bQuiet ? 1 : 0;
		NotWritten += Out.bWrite ? 0 : 1;
		MostError = FMath::Max(MostError, Out.ErrorDeg);
	}
	TestTrue(TEXT("(the error stayed far under the wake error)"), MostError < Toy.Tuning.WakeErrorDeg - 0.2);
	TestEqual(TEXT("a baseline moving under a resting base ends the rest: no frame at rest"), QuietFrames, 0);
	TestEqual(TEXT("... and every frame is written"), NotWritten, 0);
	int32 Frames = FramesToRest();
	TestTrue(FString::Printf(TEXT("still again: at rest after the settle time, not before (%d frames)"), Frames), Frames >= 15 && Frames <= 18);

	// Less than the last digits of a joint: no move.
	MoveAll(0.5 * BaselineStillCm);
	Out = Toy.Step(Frame);
	TestTrue(TEXT("a baseline moved by less than counts: still at rest"), Out.bQuiet);
	// ... but a creep adds up: 0.6 of that a frame for three seconds.
	QuietFrames = 0;
	for (int32 Index = 0; Index < 48; ++Index)
	{
		MoveAll(0.6 * BaselineStillCm);
		QuietFrames += Toy.Step(Frame).bQuiet ? 1 : 0;
	}
	TestTrue(FString::Printf(TEXT("a slow creep of the baselines: no rest once it has begun (%d frames at rest of 48)"), QuietFrames), QuietFrames <= 1);
	Frames = FramesToRest();
	TestTrue(TEXT("still again: at rest after the settle time"), Frames >= 15 && Frames <= 18);

	// The drive mode says its stance is being changed, before a baseline has moved.
	Toy.In.bStanceChanging = true;
	QuietFrames = 0;
	for (int32 Index = 0; Index < 24; ++Index)
	{
		QuietFrames += Toy.Step(Frame).bQuiet ? 1 : 0;
	}
	TestEqual(TEXT("a stance change under way: no rest, whether or not a baseline has moved yet"), QuietFrames, 0);
	Toy.In.bStanceChanging = false;
	Frames = FramesToRest();
	TestTrue(TEXT("the stance change done: at rest after the settle time"), Frames >= 15 && Frames <= 18);

	// A leg let go (nothing commands it): where its joint is, is not a
	// baseline, and its coming and going is not a stance being moved.
	Toy.Spoil = [](TArray<FLegReach>& Legs, TArray<FClearancePoint>&) {
		Legs[2].bHasBaseline = false;
		Legs[2].BaseZ += 0.3;
		Legs[2].MeasuredZ += 0.3;
	};
	QuietFrames = 0;
	for (int32 Index = 0; Index < 8; ++Index)
	{
		QuietFrames += Toy.Step(Frame).bQuiet ? 1 : 0;
	}
	TestEqual(TEXT("a leg let go: the rest goes on"), QuietFrames, 8);
	Toy.Spoil = nullptr;
	Out = Toy.Step(Frame);
	TestTrue(TEXT("taken back where it was: the rest goes on"), Out.bQuiet);
	// (The same move of a leg that IS commanded ends it.)
	Toy.Spoil = [](TArray<FLegReach>& Legs, TArray<FClearancePoint>&) {
		Legs[2].BaseZ += 0.3;
		Legs[2].MeasuredZ += 0.3;
	};
	Out = Toy.Step(Frame);
	TestFalse(TEXT("a commanded leg's baseline moved as far: the rest ends"), Out.bQuiet);
	return true;
}

// Catches: a release that ends by dropping what is left of the offsets in one
// frame (a tenth of a degree's worth: a step at every leg, largest on the
// crank of a leg carried off the ground), or one kept waiting for an exact
// zero that never comes.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawReleaseEnd,
	"Ramms.Level.Law.AReleaseEndsWithoutAStep",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawReleaseEnd::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	for (const double JointShare : { 1.0, 0.8 })
	{
		FToy Toy(12.0);
		Toy.GroundDeg = FVector2D(5.0, -3.0);
		// (Joints that fall short leave the integral a trim to give back.)
		Toy.JointShare = JointShare;
		Toy.Run(8.0);
		TestTrue(TEXT("levelled"), MaxSize(Toy.Offset) > 3.0);

		Toy.In.bEnable = false;
		FLegValues Last = Toy.Offset;
		double	   LastStep = 0.0;
		double	   StepBefore = 0.0;
		double	   MostStep = 0.0;
		int32	   Frames = 0;
		bool	   bOff = false;
		while (!bOff && Frames < 16 * 12)
		{
			const FOutputs Out = Toy.Step(Frame);
			const double   Step = MaxDifference(Toy.Offset, Last);
			Last = Toy.Offset;
			++Frames;
			bOff = Out.State == EState::Off;
			MostStep = FMath::Max(MostStep, Step);
			if (bOff)
			{
				LastStep = Step;
			}
			else
			{
				StepBefore = Step;
			}
		}
		AddInfo(FString::Printf(TEXT("release (joints reach %.1f of their commands): off after %.2f s; the last frame moved a leg %.4f cm, the one before %.4f cm; the most in a frame %.3f cm"), JointShare,
			Frames * Frame, LastStep, StepBefore, MostStep));
		if (!TestTrue(TEXT("the release ended"), bOff))
		{
			continue;
		}
		TestTrue(TEXT("every offset exactly zero"), MaxSize(Toy.Offset) == 0.0);
		// A thousandth of a degree at the corners' 45 cm is under 0.001 cm.
		TestTrue(TEXT("the last frame is no step: the ramp had brought the offsets in"), LastStep <= 0.002);
		// 3 deg/s at the corners is 0.15 cm a frame; the legs' own rate 0.6.
		TestTrue(TEXT("no frame steps a leg"), MostStep <= 0.25);
		// 5.8 deg at 3 deg/s, the rate picked up and lost, and the last of
		// the ramp: not kept waiting.
		TestTrue(TEXT("the release is not kept waiting for its end"), Frames * Frame <= 5.0);
	}
	return true;
}

// Catches: "at reach" said one tick and not the next on a slope at the very
// end of the reach (no gap between where it is said and where it is taken
// back), and what stops the legs not named until the correction has grown.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawAtReachSteady,
	"Ramms.Level.Law.AtReachIsSaidSteadilyAndFromTheFirstFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawAtReachSteady::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// 4 cm of travel: 1.5 cm planned at the corners' 45 cm, 1.909 deg.
	const double PlannedDeg = FMath::RadiansToDegrees(FMath::Atan(1.5 / 45.0));
	FToy		 Toy(4.0);
	Toy.In.SpeedCmS = 40.0;
	Toy.In.bStick = true;

	// 0.09 deg past the planned reach: under where it is said.
	Toy.GroundDeg = FVector2D(PlannedDeg + 0.09, 0.0);
	FOutputs Out = Toy.Run(4.0);
	ExpectStatus(*this, TEXT("0.09 deg past the planned reach"), Out, EStatus::Levelling);
	TestTrue(TEXT("0.09 deg past: no limit named"), Out.ReachLimit == EReachLimit::None);
	// 0.19 past: said.
	Toy.GroundDeg = FVector2D(PlannedDeg + 0.19, 0.0);
	Out = Toy.Run(4.0);
	ExpectStatus(*this, TEXT("0.19 deg past the planned reach"), Out, EStatus::AtReach);
	TestTrue(TEXT("0.19 deg past: a leg's travel"), Out.ReachLimit == EReachLimit::LegTravel);
	// Back to 0.09 past: between the two, and it stays said.
	Toy.GroundDeg = FVector2D(PlannedDeg + 0.09, 0.0);
	int32 NotSaid = 0;
	for (int32 Index = 0; Index < 16 * 4; ++Index)
	{
		NotSaid += Toy.Step(Frame).Status == EStatus::AtReach ? 0 : 1;
	}
	TestEqual(TEXT("back to 0.09 deg past: said on every frame still"), NotSaid, 0);
	// 0.02 past: taken back.
	Toy.GroundDeg = FVector2D(PlannedDeg + 0.02, 0.0);
	Out = Toy.Run(4.0);
	ExpectStatus(*this, TEXT("0.02 deg past the planned reach"), Out, EStatus::Levelling);

	// Switched on far past reach: what stops the legs is named on the very
	// first frame, with the correction still nothing.
	FToy Fresh(4.0);
	Fresh.GroundDeg = FVector2D(6.0, 0.0);
	Out = Fresh.Step(Frame);
	ExpectState(*this, TEXT("switched on past reach, first frame"), Out, EState::Active);
	ExpectStatus(*this, TEXT("switched on past reach, first frame"), Out, EStatus::AtReach);
	TestTrue(TEXT("switched on past reach, first frame: a leg's travel is named"), Out.ReachLimit == EReachLimit::LegTravel);
	TestTrue(TEXT("... with no offset given yet"), MaxSize(Out.Dz) == 0.0);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
