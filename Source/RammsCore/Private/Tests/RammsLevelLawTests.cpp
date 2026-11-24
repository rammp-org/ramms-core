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
// step; the wanted attitude kept waiting while a drive mode's stance changes;
// a hand-back's plane taken up with the offsets left on the legs; and a rest
// that is on level, not on the deadband's edge. Every number is made up -- the
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
			In.ResumeDz = TConstArrayView<double>();
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

// Catches: a seat that races the tilt policy when the switch is thrown (the
// rider is moved by the wanted attitude, the seat only covers what the base is
// behind it by); the cover's sign; a cover that runs out while the attitude is
// missing (it holds with the offsets); a trim that ramps out after a fold (it
// went into the seat's own command with the offsets, at once) or is carried
// over an adopt; the rider's fore/aft preference put into the base's wanted
// attitude, or moved faster than the tilt policy; what is past the legs' reach
// left at the rider, or taken out past the seat's room; a release that ends
// with the seat still trimmed; and any of it on a robot with no seat.
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
	// The fastest the rider's wanted attitude moved over some frames (deg/s).
	const auto RunWatching = [](FToy& Toy, double Seconds, double& InOutMostRate, double& InOutMostTrim) {
		FOutputs	Out;
		const int32 Frames = FMath::Max(1, FMath::RoundToInt32(Seconds / Frame));
		for (int32 Index = 0; Index < Frames; ++Index)
		{
			const FVector2D Was = Toy.State.Tilt.Get();
			Out = Toy.Step(Frame);
			// (Where it is walked: not on a step that set it afresh, nor while a
			// correction is ramped out open loop, when it is the attitude as read.)
			if (!Out.bOpenLoop && Out.Action == EAction::None && (Out.State == EState::Active || Out.State == EState::Releasing))
			{
				InOutMostRate = FMath::Max(InOutMostRate, (Out.RiderWantedDeg - Was).Size() / Frame);
			}
			InOutMostTrim = FMath::Max(InOutMostTrim, FMath::Abs(Out.SeatTrimDeg));
		}
		return Out;
	};

	// Switched on nose up by 6 deg. The rider is walked level at 3 deg/s with
	// the base; the seat has no part in it but to cover what the base is
	// behind its wanted attitude by, which on legs that keep up is next to
	// nothing.
	{
		FToy Toy(12.0);
		MakeToy(Toy);
		FOutputs Out = Toy.Step(Frame);
		TestTrue(TEXT("The first frame: the rider is wanted where they are"), (Out.RiderWantedDeg - FVector2D(6.0, 0.0)).Size() <= 1.0e-9 && (Out.WantedDeg - Out.RiderWantedDeg).Size() == 0.0);
		TestTrue(TEXT("... and the seat is not moved"), Out.SeatTrimDeg == 0.0 && Out.SeatShareDeg == 0.0);
		double MostRate = 0.0;
		double MostTrim = 0.0;
		Out = RunWatching(Toy, 8.0, MostRate, MostTrim);
		TestTrue(FString::Printf(TEXT("Walked level: the rider's wanted attitude within the tilt policy (%.3f deg/s)"), MostRate), MostRate <= Toy.Tuning.TiltRateDegPerSec + 1.0e-6 && MostRate >= 2.5);
		TestTrue(FString::Printf(TEXT("... the seat not moving on the way (%.3f deg)"), MostTrim), MostTrim <= 0.05);
		TestTrue(TEXT("Level: the base"), Inclination(Toy.Attitude()) <= 0.2);
		TestTrue(TEXT("Level: no trim left but the cover of the loop's own deadband"), FMath::Abs(Out.SeatTrimDeg) <= 0.15 && Out.SeatShareDeg == 0.0);
	}

	// Legs too slow for the ground: it steps 4 deg nose up under a levelled
	// base whose legs turn the plane at 1 deg/s. The seat answers nose DOWN,
	// at its own 10 deg/s, and gives the cover back as the legs catch up.
	{
		FToy Toy(12.0);
		MakeToy(Toy);
		Toy.GroundDeg = FVector2D::ZeroVector;
		Toy.Tuning.MaxCorrectionRateDegPerSec = 1.0;
		Toy.Run(2.0);
		Toy.GroundDeg = FVector2D(4.0, 0.0);
		FOutputs	 Out = Toy.Step(Frame);
		const double First = Out.SeatTrimDeg;
		TestTrue(TEXT("The ground steps: the seat sets off nose down, at its rate"), First < 0.0 && First >= -Toy.Tuning.SeatTrimRateDegPerSec * Frame - 1.0e-9);
		Out = Toy.Run(0.75);
		// Three quarters of a second in the legs have taken out under a degree.
		const double BaseOff = Toy.Attitude().X - Out.WantedDeg.X;
		TestTrue(FString::Printf(TEXT("... and covers what the legs owe (base %.2f deg off, trim %.2f)"), BaseOff, Out.SeatTrimDeg), BaseOff >= 2.5 && FMath::Abs(Out.SeatTrimDeg + BaseOff) <= 0.5);
		TestTrue(TEXT("... all of it cover: the seat holds none of the rider's wanted pitch"), Out.SeatShareDeg == 0.0 && Out.RiderWantedDeg.Equals(Out.WantedDeg, 0.0));

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
		// Read again: the base is wanted where it is and the rider where they
		// are, the seat's trim with them, so that nothing steps; what the
		// seat carries is the rider's to be walked from.
		Out = Toy.Step(Frame);
		ExpectAction(*this, TEXT("attitude back"), Out, EAction::ColdResume);
		TestTrue(TEXT("Read again: the base is wanted where it is"), (Out.WantedDeg - Toy.Attitude()).Size() <= 1.0e-9);
		TestTrue(TEXT("... the seat's trim as it was held"), FMath::Abs(Out.SeatTrimDeg - HeldDeg) <= 1.0e-9 && FMath::Abs(Out.SeatShareDeg - HeldDeg) <= 1.0e-9);
		TestTrue(TEXT("... and the rider wanted where they are, trim and all"), FMath::Abs(Out.RiderWantedDeg.X - (Toy.Attitude().X + HeldDeg)) <= 1.0e-9);

		// Levelled: the legs have taken the pitch out, so the cover has gone.
		Out = Toy.Run(10.0);
		TestTrue(TEXT("Level: the base"), Inclination(Toy.Attitude()) <= 0.2);
		TestTrue(TEXT("Level: no trim left but the cover of the loop's own deadband"), FMath::Abs(Out.SeatTrimDeg) <= 0.15);
	}

	// The rider's fore/aft preference is the seat's: the base's wanted attitude
	// stays on the reference. A take-over while it is out: it goes into the
	// seat's own command with the offsets, so it is zero at once, not ramped;
	// and the hand-back starts it from zero again, on the tilt policy.
	{
		FToy Toy(12.0);
		MakeToy(Toy);
		Toy.In.SeatPreferenceDeg = 3.0;
		double	 MostRate = 0.0;
		double	 MostTrim = 0.0;
		FOutputs Out = RunWatching(Toy, 8.0, MostRate, MostTrim);
		TestTrue(TEXT("A preference of 3 deg: the seat holds it"), FMath::Abs(Out.SeatShareDeg - 3.0) <= 1.0e-6 && FMath::Abs(Out.SeatTrimDeg - 3.0) <= 0.15);
		TestTrue(TEXT("... the rider wanted at it"), FMath::Abs(Out.RiderWantedDeg.X - 3.0) <= 1.0e-6);
		TestTrue(TEXT("... the base wanted level, and level"), FMath::Abs(Out.WantedDeg.X) <= 1.0e-6 && Inclination(Toy.Attitude()) <= 0.2);
		TestTrue(FString::Printf(TEXT("... reached within the tilt policy (%.3f deg/s)"), MostRate), MostRate <= Toy.Tuning.TiltRateDegPerSec + 1.0e-6);

		// Changed to 2 deg the other way: 5 deg of travel, at the policy's rate.
		Toy.In.SeatPreferenceDeg = -2.0;
		MostRate = 0.0;
		Out = RunWatching(Toy, 1.0, MostRate, MostTrim);
		TestTrue(TEXT("Changed: a second in, still on the way"), Out.RiderWantedDeg.X > -1.9 && Out.RiderWantedDeg.X < 2.5);
		Out = RunWatching(Toy, 3.0, MostRate, MostTrim);
		TestTrue(TEXT("... there"), FMath::Abs(Out.RiderWantedDeg.X + 2.0) <= 1.0e-6 && FMath::Abs(Out.SeatTrimDeg + 2.0) <= 0.15);
		TestTrue(FString::Printf(TEXT("... within the tilt policy (%.3f deg/s)"), MostRate), MostRate <= Toy.Tuning.TiltRateDegPerSec + 1.0e-6 && MostRate >= 2.5);
		TestTrue(TEXT("... the base's wanted attitude never moved"), FMath::Abs(Out.WantedDeg.X) <= 1.0e-6);

		Toy.In.bYield = true;
		Toy.In.Mode = EMode::Taken;
		Out = Toy.Step(Frame);
		ExpectAction(*this, TEXT("take-over"), Out, EAction::Fold);
		TestTrue(TEXT("Folded: the trim is zero on that step"), Out.SeatTrimDeg == 0.0 && Out.SeatShareDeg == 0.0);
		Out = Toy.Run(0.5);
		TestTrue(TEXT("... and stays zero while the taker has the seat"), Out.SeatTrimDeg == 0.0);

		Toy.HandBack(-30.0);
		Out = Toy.Step(Frame);
		ExpectAction(*this, TEXT("hand-back"), Out, EAction::Adopt);
		TestTrue(TEXT("Adopted: the trim starts again from zero"), Out.SeatTrimDeg == 0.0 && Out.SeatShareDeg == 0.0);
		Out = Toy.Step(Frame);
		TestTrue(TEXT("... and sets off for the preference no faster than the tilt policy"),
			Out.SeatShareDeg < 0.0 && Out.SeatShareDeg >= -Toy.Tuning.TiltAccelDegPerSec2 * Frame * Frame - 1.0e-9);
	}

	// A slope past the legs' reach: the base is wanted tilted by what they
	// cannot take out, and the seat takes that back out for the rider -- as
	// far as its own room goes.
	for (const double RoomDeg : { 1.0e6, 1.0 })
	{
		FToy Toy(5.0);
		MakeToy(Toy);
		Toy.In.SeatTrimLowDeg = -RoomDeg;
		const FOutputs Out = Toy.Run(10.0);
		const double   Excess = Out.WantedDeg.X;
		TestTrue(FString::Printf(TEXT("Past reach: the base is wanted nose up by what the legs must leave (%.2f deg)"), Excess), Excess >= 1.5 && Excess <= 5.0 + 1.0e-9);
		const double Taken = FMath::Min(Excess, RoomDeg);
		TestNearlyEqual(TEXT("... the seat holds it for the rider, within its room"), Out.SeatShareDeg, -Taken, 1.0e-6);
		TestNearlyEqual(TEXT("... the rider is wanted on the reference but for what the seat cannot take"), Out.RiderWantedDeg.X, Excess - Taken, 1.0e-6);
		TestTrue(TEXT("... and the trim is that share (the base is where it is wanted)"), FMath::Abs(Out.SeatTrimDeg - Out.SeatShareDeg) <= 0.2);
	}

	// Switched off with the seat holding a preference: the release walks the
	// rider back to the stance's attitude and the seat back to nothing, and
	// only then is it off.
	{
		FToy Toy(12.0);
		MakeToy(Toy);
		Toy.In.SeatPreferenceDeg = 3.0;
		Toy.Run(8.0);
		Toy.In.bEnable = false;
		double	 MostRate = 0.0;
		double	 MostTrim = 0.0;
		FOutputs Out = RunWatching(Toy, 0.5, MostRate, MostTrim);
		ExpectState(*this, TEXT("switched off, half a second in"), Out, EState::Releasing);
		TestTrue(TEXT("... the seat still on its way out"), Out.SeatTrimDeg > 0.5);
		Out = RunWatching(Toy, 12.0, MostRate, MostTrim);
		ExpectState(*this, TEXT("released"), Out, EState::Off);
		TestTrue(TEXT("Off: the seat's trim exactly zero"), Out.SeatTrimDeg == 0.0 && Out.SeatShareDeg == 0.0);
		TestTrue(FString::Printf(TEXT("... the rider walked back within the tilt policy (%.3f deg/s)"), MostRate), MostRate <= Toy.Tuning.TiltRateDegPerSec + 1.0e-6);
	}

	// Switched off with the attitude gone: the correction is ramped out open
	// loop, and what the seat holds goes out with it.
	{
		FToy Toy(12.0);
		MakeToy(Toy);
		Toy.In.SeatPreferenceDeg = 3.0;
		Toy.Run(8.0);
		Toy.bReadable = false;
		Toy.In.bEnable = false;
		FOutputs Out = Toy.Run(0.5);
		ExpectState(*this, TEXT("switched off blind, half a second in"), Out, EState::Releasing);
		TestTrue(TEXT("... ramping out open loop, the seat still on its way"), Out.bOpenLoop && Out.SeatTrimDeg > 0.5 && Out.SeatTrimDeg < 3.0);
		Out = Toy.Run(12.0);
		ExpectState(*this, TEXT("released blind"), Out, EState::Off);
		TestTrue(TEXT("Off: the seat's trim exactly zero"), Out.SeatTrimDeg == 0.0 && Out.SeatShareDeg == 0.0);
	}

	// With no seat, or the trim switched off, there is none of it: no trim,
	// no share, and the rider's wanted attitude is the base's.
	for (const int32 Which : { 0, 1 })
	{
		FToy Toy(5.0);
		MakeToy(Toy);
		Toy.In.SeatPreferenceDeg = 3.0;
		if (Which == 0)
		{
			Toy.In.bSeat = false;
		}
		else
		{
			Toy.Tuning.bSeatCoversTransients = false;
		}
		const TCHAR* Name = Which == 0 ? TEXT("No seat") : TEXT("Switched off in the tuning");
		bool		 bNone = true;
		for (int32 Index = 0; Index < 96; ++Index)
		{
			const FOutputs Out = Toy.Step(Frame);
			bNone = bNone && Out.SeatTrimDeg == 0.0 && Out.SeatShareDeg == 0.0 && (Out.RiderWantedDeg - Out.WantedDeg).Size() == 0.0;
		}
		TestTrue(FString::Printf(TEXT("%s: no trim, no share, the rider's wanted attitude the base's"), Name), bNone);
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

// Catches: a drive slowed for how little of the legs' travel is left (a robot
// on a steady ramp at the end of its reach would crawl for good); and the
// cap's own terms not fed from the law -- the terrain estimate's turning
// rate, and the slope with the standing legs' span and their rate.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawSpeedCap,
	"Ramms.Level.Law.SpeedCapGoesByTheGroundsTurnNotTheReachLeft",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawSpeedCap::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	{
		// 5 cm of travel either way: planned reach (0.5 margin and 2 soft zone
		// off) 2.5 cm at 45 cm, 3.18 deg. The ground steepens at 1 deg/s to
		// 3.1 deg: inside reach, level all the way, and at the end the legs
		// have 2.56 cm of their 5 left. That is no reason to slow.
		FToy Toy(5.0);
		Toy.In.SpeedCmS = 40.0;
		Toy.In.bStick = true;
		FOutputs Out = Toy.Run(1.0);
		double	 LeastScale = 1.0;
		double	 MostError = 0.0;
		while (Toy.GroundDeg.X < 3.1)
		{
			Toy.GroundDeg.X = FMath::Min(3.1, Toy.GroundDeg.X + 1.0 * Frame);
			Out = Toy.Step(Frame);
			LeastScale = FMath::Min(LeastScale, Out.SpeedScale);
			MostError = FMath::Max(MostError, Out.ErrorDeg);
		}
		Out = Toy.Run(4.0);
		LeastScale = FMath::Min(LeastScale, Out.SpeedScale);
		TestTrue(TEXT("Near a limit: level all the way"), MostError < 0.5 && Out.Fraction >= 1.0);
		TestTrue(TEXT("Near a limit: the legs are within 3 cm of their travel"), Out.MinMarginCm < 3.0 && Out.MinMarginCm > 2.0 && Out.ReachLeft < 0.1);
		TestTrue(FString::Printf(TEXT("Near a limit: the drive is not slowed for it (%.3f)"), LeastScale), LeastScale >= 1.0 - 1.0e-9);
	}
	{
		// Legs with travel to spare, and a ground that turns under the robot:
		// at 4.5 deg/s the legs follow (the ground may turn at 6), at 7.5 it
		// outruns what they are asked, with the base still inside a degree.
		for (const double TurnDegPerSec : { 4.5, 7.5 })
		{
			FToy Toy(20.0);
			Toy.In.SpeedCmS = 40.0;
			Toy.In.bStick = true;
			FOutputs Out = Toy.Run(1.0);
			double	 LeastScale = 1.0;
			double	 MostError = 0.0;
			for (int32 Index = 0; Index < 20; ++Index)
			{
				Toy.GroundDeg.X += TurnDegPerSec * Frame;
				Out = Toy.Step(Frame);
				LeastScale = FMath::Min(LeastScale, Out.SpeedScale);
				MostError = FMath::Max(MostError, Out.ErrorDeg);
			}
			TestTrue(FString::Printf(TEXT("The ground turning at %.1f deg/s: the error under where its own term binds (%.2f deg)"), TurnDegPerSec, MostError), MostError < Toy.Tuning.SpeedCapErrorDeg);
			TestNearlyEqual(FString::Printf(TEXT("The ground turning at %.1f deg/s: the cap has its rate"), TurnDegPerSec), Toy.State.SpeedScale.GetTurnDegPerSec().X, TurnDegPerSec, 0.3);
			TestNearlyEqual(FString::Printf(TEXT("The ground turning at %.1f deg/s: what the legs can follow"), TurnDegPerSec), Toy.State.SpeedScale.GetFollowDegPerSec(), 6.0, 1.0e-6);
			if (TurnDegPerSec < 6.0)
			{
				TestTrue(FString::Printf(TEXT("... which the legs can follow: full speed (%.3f)"), LeastScale), LeastScale >= 1.0 - 1.0e-9);
			}
			else
			{
				TestTrue(FString::Printf(TEXT("... which outruns the legs: the drive is slowed (%.3f)"), LeastScale), LeastScale < 0.9 && LeastScale >= Toy.Tuning.MinSpeedScale - 1.0e-9);
			}
		}
	}
	{
		// What turns the estimate without the ground turning is not the cap's
		// business: a stance being changed under the legs, and an estimate
		// seeded afresh. (The cap here takes one tick's turn whole, and the
		// legs to follow only half a degree a second, so that a single frame
		// of either would show; the robot stands, its stick held, so that no
		// slope's end is in it.)
		FToy Toy(20.0);
		Toy.State.SpeedScale.Cap.TurnFilterSeconds = 0.0;
		Toy.State.SpeedScale.Cap.FollowMaxDegPerSec = 0.5;
		Toy.In.SpeedCmS = 0.0;
		Toy.In.bStick = true;
		FOutputs Out = Toy.Run(2.0);
		TestTrue(TEXT("(level ground, standing steady: full speed)"), Out.SpeedScale >= 1.0 - 1.0e-9);
		// The rear pair's baselines drawn up a quarter of a centimetre a frame:
		// the stance turns nose up under the robot, 2.5 deg/s, the ground
		// standing still. (The toy's base reads what its ground says, so the
		// stance's turn is put there.)
		const FVector2D Before = Out.GroundDeg;
		double			LeastScale = 1.0;
		for (int32 Index = 0; Index < 6; ++Index)
		{
			for (int32 Leg = 4; Leg < 6; ++Leg)
			{
				Toy.Legs[Leg].BaseZ += 0.25;
				Toy.Legs[Leg].MeasuredZ += 0.25;
			}
			Toy.GroundDeg.X += FMath::RadiansToDegrees(FMath::Atan(0.25 / 90.0));
			Out = Toy.Step(Frame);
			LeastScale = FMath::Min(LeastScale, Out.SpeedScale);
		}
		TestTrue(TEXT("A stance being changed: the estimate turns with it"), (Out.GroundDeg - Before).Size() > 0.4);
		TestTrue(FString::Printf(TEXT("... and the drive is not slowed for it (%.3f)"), LeastScale), LeastScale >= 1.0 - 1.0e-9);
		// Put somewhere else, on ground 4 deg steeper across: the estimate is
		// seeded there in one frame, which is no turn of the ground.
		Toy.Run(3.0);
		Toy.GroundDeg.Y += 4.0;
		Toy.In.bColdReset = true;
		Out = Toy.Step(Frame);
		TestTrue(TEXT("Put somewhere else: the estimate is seeded there at once"), FMath::Abs(Out.GroundDeg.Y - Toy.GroundDeg.Y) < 0.5);
		TestTrue(FString::Printf(TEXT("... and the drive is not slowed for that (%.3f)"), Out.SpeedScale), Out.SpeedScale >= 1.0 - 1.0e-9);
	}
	{
		// A steady 9 deg under legs 54 cm apart (the lesser of the stance's two
		// extents) that turn the correction at 10 deg/s: where it ends the
		// ground turns 9 deg in 54 cm, which can be followed (at 6 deg/s) at
		// 36 cm/s. The toy's drive answers the scale as the cap takes a drive to.
		const auto Settle = [](FToy& Toy, double StickCmS) {
			double	 Driven = Toy.State.SpeedScale.Get();
			FOutputs Out;
			for (int32 Index = 0; Index < 160; ++Index)
			{
				Driven += (Toy.State.SpeedScale.Get() - Driven) * FMath::Min(1.0, Frame / Toy.State.SpeedScale.Cap.DriveLagSeconds);
				Toy.In.SpeedCmS = StickCmS * Driven;
				Out = Toy.Step(Frame);
			}
			return Out;
		};
		// (With the slope's-end term switched on: as the cap comes, a steady
		// slope costs nothing -- checked last.)
		const auto SlopeEndOn = [](FToy& Toy) {
			Toy.State.SpeedScale.Cap.bSlopeEnd = true;
			Toy.State.SpeedScale.Cap.LeadCmS = 0.0;
			Toy.State.SpeedScale.Cap.OverspeedGain = 0.0;
			Toy.State.SpeedScale.Cap.SteadyAskCmS2 = 1.0e9;
			Toy.State.SpeedScale.Cap.UnsteadyAskCmS2 = 2.0e9;
			Toy.State.SpeedScale.Cap.SteadySpeedCmS2 = 1.0e9;
			Toy.State.SpeedScale.Cap.UnsteadySpeedCmS2 = 2.0e9;
		};
		FToy Toy(20.0);
		SlopeEndOn(Toy);
		Toy.GroundDeg = FVector2D(9.0, 0.0);
		Toy.In.bStick = true;
		FOutputs Out = Settle(Toy, 30.0);
		TestTrue(TEXT("9 deg: level"), Out.ErrorDeg < 0.3 && Out.Fraction >= 1.0);
		TestNearlyEqual(TEXT("9 deg: the speed its end can be taken at"), Toy.State.SpeedScale.GetExitSpeedCmS(), 36.0, 0.5);
		TestTrue(TEXT("9 deg at 30 cm/s: full speed"), Out.SpeedScale >= 1.0 - 1.0e-9);
		Out = Settle(Toy, 72.0);
		TestNearlyEqual(TEXT("9 deg, the stick asking 72 cm/s: half"), Out.SpeedScale, 0.5, 0.02);
		// Standing on it: nothing turns, nothing will.
		Out = Settle(Toy, 0.0);
		TestTrue(TEXT("9 deg, standing: full speed"), Out.SpeedScale >= 1.0 - 1.0e-9);

		// Slow legs: 2 cm/s, of which the law asks 1.6, 45 cm from the middle
		// of the stance in pitch. The ground may turn at 0.8 of what that
		// turns the correction at, and a 4 deg grade's end is taken slower
		// for it.
		FToy Slow(20.0);
		SlopeEndOn(Slow);
		for (FLegReach& Leg : Slow.Legs)
		{
			Leg.MaxRateCmS = 2.0;
		}
		Slow.GroundDeg = FVector2D(4.0, 0.0);
		Slow.In.bStick = true;
		Settle(Slow, 10.0);
		const double LegsDegPerSec = 0.8 * 2.0 / (45.0 * FMath::Tan(FMath::DegreesToRadians(1.0)));
		TestNearlyEqual(TEXT("Slow legs: the ground may turn at 0.8 of what they turn the correction at"), Slow.State.SpeedScale.GetFollowDegPerSec(), 0.8 * LegsDegPerSec, 1.0e-3);
		TestNearlyEqual(TEXT("Slow legs: the speed a 4 deg grade's end can be taken at"), Slow.State.SpeedScale.GetExitSpeedCmS(), 0.8 * LegsDegPerSec * 54.0 / 4.0, 0.5);

		// As the cap comes: the same steady 9 deg, the stick asking 72 cm/s,
		// costs nothing.
		FToy Free(20.0);
		Free.GroundDeg = FVector2D(9.0, 0.0);
		Free.In.bStick = true;
		Free.In.AskedSpeedCmS = 72.0;
		Out = Settle(Free, 72.0);
		TestTrue(FString::Printf(TEXT("9 deg, levelled, the stick asking 72 cm/s: a steady slope costs nothing (%.3f)"), Out.SpeedScale), Out.ErrorDeg < 0.3 && Out.SpeedScale >= 1.0 - 1.0e-9);

		// What the stick asks reaches the cap: going half as fast again as
		// asked (run away downhill), the drive is asked for less at once; with
		// the drive not saying what is asked, the speed there is, is all the
		// cap has, and it is not taken for a run-away.
		Free.In.AskedSpeedCmS = 40.0;
		Free.In.SpeedCmS = 60.0;
		Out = Free.Step(Frame);
		TestNearlyEqual(TEXT("Half as fast again as its stick asks: asked for the floor"), Out.SpeedScale, Free.Tuning.MinSpeedScale, 1.0e-9);
		Free.In.AskedSpeedCmS = -1.0;
		Out = Free.Step(Frame);
		TestTrue(FString::Printf(TEXT("A drive that does not say what is asked: no run-away is read into its speed (%.3f)"), Out.SpeedScale), Out.SpeedScale >= 0.95);
	}
	return true;
}

// Catches: legs carried off the ground holding the whole plane to their pace
// while they are let down (the correction then unwinds at a fraction of what
// the legs that stand can do, and at a crest the base falls further forward
// than with no levelling at all); such a leg told to go faster than it can;
// its catching up taken for the legs being outrun (the integral held all the
// while); and a release that ends, and steps it, before it is back.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawLiftedLegs,
	"Ramms.Level.Law.LiftedLegsAreLetDownAtTheirOwnPace",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawLiftedLegs::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// On centre and rear legs, quick ones; the front pair carried 12 cm clear
	// with travel both ways, and slow: 2 cm/s, of which the law asks 1.6.
	FToy Toy(12.0);
	for (int32 Index = 0; Index < Toy.Legs.Num(); ++Index)
	{
		FLegReach& Leg = Toy.Legs[Index];
		Leg.MaxRateCmS = Index < 2 ? 2.0 : 60.0;
		if (Index < 2)
		{
			Leg.BaseZ = Leg.MeasuredZ = -18.0;
			Leg.MaxZ = -10.0;
			// (For the toy's own attitude: the law finds who stands itself.)
			Leg.bSupport = false;
			Leg.CarryShare = 0.0;
		}
	}
	const double OwnStepCm = Toy.Tuning.LegRateFraction * 2.0 * Frame;
	const double FrontArmCm = 67.5; // ahead of the middle of the four that stand
	const auto	 FrontAt = [FrontArmCm](double Deg) { return FrontArmCm * FMath::Tan(FMath::DegreesToRadians(Deg)); };

	// Nose up 6 deg: the front pair must come UP 7.1 cm to keep clear, and
	// while it does the plane waits for it (4.4 s at its pace).
	Toy.GroundDeg = FVector2D(6.0, 0.0);
	FOutputs Out = Toy.Run(2.0);
	TestTrue(TEXT("Brought up: the plane is held to the front pair's pace"), Out.AppliedDeg.X < 4.5 && Out.bRateLimited);
	Out = Toy.Run(8.0);
	TestTrue(TEXT("Nose up: level"), Inclination(Toy.Attitude()) <= 0.25);
	TestNearlyEqual(TEXT("Nose up: the front pair where the plane has it"), Out.Dz[0], FrontAt(Out.AppliedDeg.X), 0.05);

	// The ground turns to 6 deg nose down. The legs that stand unwind the
	// correction at the plane's own 10 deg/s; the front pair is let down
	// 14 cm behind them, at its own pace.
	Toy.GroundDeg = FVector2D(-6.0, 0.0);
	double MostFall = 0.0;
	double Last = Out.Dz[0];
	bool   bFreeWhileBehind = false;
	for (int32 Index = 0; Index < 32; ++Index)
	{
		Out = Toy.Step(Frame);
		MostFall = FMath::Max(MostFall, Last - Out.Dz[0]);
		Last = Out.Dz[0];
		if (Index == 23)
		{
			TestTrue(FString::Printf(TEXT("Let down: 1.5 s on the correction is unwound (%.2f deg)"), Out.AppliedDeg.X), Out.AppliedDeg.X <= -5.0);
			TestTrue(TEXT("... with the front pair still far behind"), Out.Dz[0] > FrontAt(Out.AppliedDeg.X) + 3.0);
		}
		// The plane has arrived and only the front pair is on its way: that is
		// not the legs being outrun.
		bFreeWhileBehind = bFreeWhileBehind || (Index >= 26 && !Out.bRateLimited && Out.Dz[0] > FrontAt(Out.AppliedDeg.X) + 1.0);
	}
	TestTrue(FString::Printf(TEXT("Let down: the front pair never asked faster than its own pace (%.3f of %.3f cm a frame)"), MostFall, OwnStepCm), MostFall <= OwnStepCm * 1.001);
	TestTrue(TEXT("... and at that pace"), MostFall >= OwnStepCm * 0.99);
	TestTrue(TEXT("Its catching up is not the legs being outrun"), bFreeWhileBehind);
	TestTrue(FString::Printf(TEXT("Let down: 2 s on the base is all but level (%.2f deg)"), Inclination(Toy.Attitude())), Inclination(Toy.Attitude()) <= 1.0);
	// (Standing still and level all the while: no rest is declared on a leg
	// that is still on its way.)
	Out = Toy.Run(10.0);
	TestNearlyEqual(TEXT("Nose down: the front pair has caught the plane up"), Out.Dz[0], FrontAt(Out.AppliedDeg.X), 0.05);
	TestTrue(TEXT("Nose down: level"), Inclination(Toy.Attitude()) <= 0.25);

	// Nose up again, then switched off: the correction goes out within the
	// tilt policy, faster than the front pair can be let down, and the
	// release waits for it rather than stepping it home.
	Toy.GroundDeg = FVector2D(6.0, 0.0);
	Out = Toy.Run(14.0);
	TestNearlyEqual(TEXT("Nose up again: the front pair where the plane has it"), Out.Dz[0], FrontAt(Out.AppliedDeg.X), 0.05);
	Toy.In.bEnable = false;
	MostFall = 0.0;
	Last = Out.Dz[0];
	double PlaneOutAt = -1.0;
	double OffAt = -1.0;
	for (int32 Index = 0; Index < 16 * 12 && OffAt < 0.0; ++Index)
	{
		Out = Toy.Step(Frame);
		MostFall = FMath::Max(MostFall, Last - Out.Dz[0]);
		Last = Out.Dz[0];
		if (PlaneOutAt < 0.0 && Out.AppliedDeg.Size() < 0.01)
		{
			PlaneOutAt = Index * Frame;
		}
		if (Out.State == EState::Off)
		{
			OffAt = Index * Frame;
		}
	}
	TestTrue(FString::Printf(TEXT("Switched off: the correction is out in the tilt policy's time (%.2f s)"), PlaneOutAt), PlaneOutAt > 0.0 && PlaneOutAt <= 3.5);
	TestTrue(FString::Printf(TEXT("... the front pair let down at its own pace all the way (%.3f cm a frame at most)"), MostFall), MostFall <= OwnStepCm * 1.001);
	TestTrue(FString::Printf(TEXT("... and off only once it is back (%.2f s)"), OffAt), OffAt >= PlaneOutAt + 0.5);
	TestTrue(TEXT("... every offset exactly zero"), MaxSize(Out.Dz) == 0.0);
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

	// Plenty of travel; roll capped at 6 deg. (At a pace the ends of these
	// slopes could be taken at: the speed cap has no business here but for
	// the slope past reach.)
	FToy Toy(40.0);
	Toy.Tuning.MaxCorrectionDeg = FVector2D(15.0, 6.0);
	Toy.In.SpeedCmS = 15.0;
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

// Catches: the wanted attitude walked while a drive mode's stance is being
// changed under the legs (the rider is then turned by the walk and by the
// stance's own rock at once); the walk stopped dead rather than within the
// tilt policy's acceleration; a release that runs its time out while it
// waits, and finishes open loop.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawWantedHeldForStance,
	"Ramms.Level.Law.WantedHeldWhileTheStanceChanges",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawWantedHeldForStance::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// The largest change of the wanted attitude's rate from one frame to the
	// next over Frames frames (deg/s^2), and where the rate ends (deg/s).
	struct FWalk
	{
		double MostAccel = 0.0;
		double MostRate = 0.0;
		double EndRate = 0.0;
	};
	const auto Watch = [](FToy& Toy, int32 Frames, FVector2D& InOutWanted, FVector2D& InOutRate) {
		FWalk Walk;
		for (int32 Index = 0; Index < Frames; ++Index)
		{
			const FOutputs	Out = Toy.Step(Frame);
			const FVector2D Rate = (Out.WantedDeg - InOutWanted) / Frame;
			Walk.MostAccel = FMath::Max(Walk.MostAccel, (Rate - InOutRate).Size() / Frame);
			Walk.MostRate = FMath::Max(Walk.MostRate, Rate.Size());
			InOutWanted = Out.WantedDeg;
			InOutRate = Rate;
		}
		Walk.EndRate = InOutRate.Size();
		return Walk;
	};

	{
		// Switched on on a slope, a second and a half into the walk to level:
		// the wanted attitude is moving at the policy's rate.
		FToy Toy(12.0);
		Toy.GroundDeg = FVector2D(6.0, 3.0);
		FOutputs  Out = Toy.Run(1.5 - Frame);
		FVector2D Wanted = Out.WantedDeg;
		FVector2D Rate = FVector2D::ZeroVector;
		Out = Toy.Step(Frame);
		Rate = (Out.WantedDeg - Wanted) / Frame;
		Wanted = Out.WantedDeg;
		TestTrue(TEXT("(walking to level at the policy's rate)"), Rate.Size() >= Toy.Tuning.TiltRateDegPerSec - 0.05 && Wanted.Size() > 2.0);

		// A drive mode's stance begins to change: the walk stops, within the
		// policy's acceleration, and waits.
		Toy.In.bStanceChanging = true;
		const FWalk Stopping = Watch(Toy, 16, Wanted, Rate);
		AddInfo(FString::Printf(TEXT("stance change during the walk: the wanted attitude's rate changed by %.2f deg/s^2 at most, ended at %.3f deg/s, held at (%.2f, %.2f)"), Stopping.MostAccel,
			Stopping.EndRate, Wanted.X, Wanted.Y));
		TestTrue(TEXT("the walk stops within the policy's acceleration"), Stopping.MostAccel <= Toy.Tuning.TiltAccelDegPerSec2 + 1.0e-6);
		TestEqual(TEXT("... and has stopped within a second"), Stopping.EndRate, 0.0, 1.0e-9);
		const FVector2D Held = Wanted;
		TestTrue(TEXT("(short of level: there is a walk left to wait with)"), Held.Size() > 0.5);
		const FWalk Waiting = Watch(Toy, 32, Wanted, Rate);
		TestTrue(TEXT("held for as long as the stance changes"), Waiting.MostRate <= 1.0e-9 && (Wanted - Held).Size() <= 1.0e-9);
		ExpectState(*this, TEXT("levelling still, on the attitude held"), Toy.Step(Frame), EState::Active);
		TestTrue(TEXT("... the base held there"), (Toy.Attitude() - Held).Size() <= 0.3);

		// The stance has arrived: the walk goes on, again within the policy.
		Toy.In.bStanceChanging = false;
		const FWalk Again = Watch(Toy, 64, Wanted, Rate);
		TestTrue(TEXT("the walk goes on once the stance has arrived"), Again.MostRate >= 1.0);
		TestTrue(TEXT("... within the policy's rate and acceleration"), Again.MostRate <= Toy.Tuning.TiltRateDegPerSec + 1.0e-6 && Again.MostAccel <= Toy.Tuning.TiltAccelDegPerSec2 + 1.0e-6);
		TestTrue(TEXT("... to level"), Wanted.Size() <= 0.05 && Inclination(Toy.Attitude()) <= 0.25);
	}
	{
		// Level on a slope, and the mode becomes one it does not level in, its
		// stance change beginning with it: the base is given back only once
		// the stance has arrived, however long that takes.
		FToy Toy(12.0);
		Toy.GroundDeg = FVector2D(-5.0, 2.0);
		FOutputs Out = Toy.Run(8.0);
		TestTrue(TEXT("(level on the slope)"), Inclination(Toy.Attitude()) <= 0.25);
		Toy.In.Mode = EMode::Release;
		Toy.In.bStanceChanging = true;
		FVector2D Wanted = Out.WantedDeg;
		FVector2D Rate = FVector2D::ZeroVector;
		// Longer than a release may run closed loop (ReleaseMaxSeconds).
		const FWalk Waiting = Watch(Toy, FMath::RoundToInt32((ReleaseMaxSeconds + 2.0) / Frame), Wanted, Rate);
		Out = Toy.Step(Frame);
		ExpectState(*this, TEXT("a release that waits for the stance"), Out, EState::Releasing);
		TestTrue(TEXT("the wanted attitude did not move while the stance changed"), Waiting.MostRate <= 1.0e-9);
		TestTrue(TEXT("the base stayed level meanwhile"), Inclination(Toy.Attitude()) <= 0.25);
		TestFalse(TEXT("the wait does not count against the release's time: still closed loop"), Out.bOpenLoop);
		Toy.In.bStanceChanging = false;
		Wanted = Out.WantedDeg;
		Rate = FVector2D::ZeroVector;
		const FWalk Back = Watch(Toy, 80, Wanted, Rate);
		TestTrue(TEXT("then it walks back to the ground's attitude"), Back.MostRate >= 1.0 && Back.MostRate <= Toy.Tuning.TiltRateDegPerSec + 1.0e-6);
		Out = Toy.Run(2.0);
		ExpectState(*this, TEXT("given back"), Out, EState::Suspended);
		TestTrue(TEXT("... on its stance"), (Toy.Attitude() - Toy.GroundDeg).Size() <= 1.0e-6 && MaxSize(Toy.Offset) == 0.0);
	}
	return true;
}

// Catches: a hand-back whose plane is taken up with the offsets the allocation
// would give rather than the ones the plant could leave on the legs: a leg it
// could not leave the plane on (a raised leg on its stop) is then stepped to
// the allocation's offset on the first frame after.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawAdoptKeepsOffsetsLeft,
	"Ramms.Level.Law.AdoptKeepsTheOffsetsLeft",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawAdoptKeepsOffsetsLeft::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	// The front pair carried 8 cm off the ground, 0.3 cm short of its stop.
	FToy Toy(12.0);
	for (int32 Index = 0; Index < 2; ++Index)
	{
		Toy.Legs[Index].BaseZ = -22.0;
		Toy.Legs[Index].MeasuredZ = -22.0;
		Toy.Legs[Index].MaxZ = -21.7;
		// (The toy's own base stands on the other four: where this pair is says nothing of its attitude.)
		Toy.Legs[Index].CarryShare = 0.0;
	}
	FLegValues StanceZ;
	for (const FLegReach& Leg : Toy.Legs)
	{
		StanceZ.Add(Leg.BaseZ);
	}
	Toy.GroundDeg = FVector2D(-5.0, 0.0);
	FOutputs Out = Toy.Run(8.0);
	TestTrue(TEXT("(level nose down, the raised pair let out with the plane)"), Inclination(Toy.Attitude()) <= 0.25 && Toy.Offset[0] < -3.0);

	// A take-over: folded, and the taker draws the raised pair back up onto
	// its stance height (the rest it leaves as they were, level).
	Toy.In.bYield = true;
	Toy.In.Mode = EMode::Taken;
	Out = Toy.Step(0.0);
	ExpectAction(*this, TEXT("the yield"), Out, EAction::Fold);
	for (int32 Index = 0; Index < 2; ++Index)
	{
		Toy.Legs[Index].BaseZ = -22.0;
	}
	Toy.Run(1.0);

	// The hand-back: the plane out of the baselines, but for the raised pair,
	// whose baselines cannot go past their stop.
	FAdoption Adoption;
	if (!TestTrue(TEXT("a plan"), PlanAdoption(Toy.Legs, StanceZ, TConstArrayView<FClearancePoint>(), Toy.Tuning, Adoption)))
	{
		return false;
	}
	for (int32 Index = 0; Index < Toy.Legs.Num(); ++Index)
	{
		Toy.Legs[Index].BaseZ = Adoption.BaseZ[Index];
		Toy.Offset[Index] = Adoption.Dz[Index];
	}
	TestTrue(TEXT("(the raised pair could keep only what its stop allows)"), Adoption.Dz[0] > -0.31 && Adoption.Dz[0] <= 0.0 && Adoption.Dz[4] > 1.0);
	Toy.In.bResume = true;
	Toy.In.bResumeHasPlane = true;
	Toy.In.ResumePlaneDeg = Adoption.PlaneDeg;
	Toy.In.ResumeDz = Adoption.Dz;
	Toy.In.Mode = EMode::Allowed;
	const FLegValues Left = Toy.Offset;
	Out = Toy.Step(0.0);
	ExpectAction(*this, TEXT("the hand-back"), Out, EAction::Adopt);
	ExpectState(*this, TEXT("the hand-back"), Out, EState::Active);
	TestTrue(TEXT("the offsets the law goes on from are the ones left on the legs"), MaxDifference(Out.Dz, Left) <= 1.0e-9);

	// The first frame after: the raised pair is on its way to the plane's
	// offset at the legs' rate, not there in one step; the legs that stand
	// have not moved.
	Out = Toy.Step(Frame);
	const double Most = Toy.Tuning.LegRateFraction * Toy.Legs[0].MaxRateCmS * Frame;
	AddInfo(FString::Printf(TEXT("hand-back with a raised pair on its stop: left with %.2f cm, the plane asks %.2f; first frame after it carries %.2f (the legs' rate allows %.2f a frame)"), Left[0],
		Toy.Legs[0].XY.X * FMath::Tan(FMath::DegreesToRadians(Adoption.PlaneDeg.X)), Out.Dz[0], Most));
	TestTrue(TEXT("the raised pair moves towards the plane"), Out.Dz[0] < Left[0] - 0.05);
	TestTrue(TEXT("... no faster than a leg may"), FMath::Abs(Out.Dz[0] - Left[0]) <= Most + 1.0e-6 && FMath::Abs(Out.Dz[1] - Left[1]) <= Most + 1.0e-6);
	for (int32 Index = 2; Index < 6; ++Index)
	{
		TestTrue(FString::Printf(TEXT("leg %d, which stands, is not moved by it"), Index), FMath::Abs(Out.Dz[Index] - Left[Index]) <= 0.02);
	}
	Out = Toy.Run(4.0);
	TestTrue(TEXT("in the end the raised pair carries the plane's offset"), Out.Dz[0] < -3.0);
	TestTrue(TEXT("... the base level throughout"), Inclination(Toy.Attitude()) <= 0.25);

	// The ground the law goes on from is read off the legs as the hand-back left them: who carries, and how far
	// each is behind its offset, by the offsets LEFT. (By the offsets from before the take-over, none, every leg
	// the plane has retracted would read as still on its way down and be left out of the plane.)
	{
		FToy Six(12.0);
		Six.GroundDeg = FVector2D(4.0, -2.0);
		Six.Run(8.0);
		Six.In.bYield = true;
		Six.In.Mode = EMode::Taken;
		Six.Step(0.0);
		Six.Run(0.5);
		FLegValues Stance;
		Stance.Init(-30.0, Six.Legs.Num());
		FAdoption Plan;
		if (!TestTrue(TEXT("six standing legs: a plan"), PlanAdoption(Six.Legs, Stance, TConstArrayView<FClearancePoint>(), Six.Tuning, Plan)))
		{
			return false;
		}
		// One standing leg left half a centimetre less than the plane (the pose is the same).
		Plan.BaseZ[1] += 0.5;
		Plan.Dz[1] -= 0.5;
		for (int32 Index = 0; Index < Six.Legs.Num(); ++Index)
		{
			Six.Legs[Index].BaseZ = Plan.BaseZ[Index];
			Six.Offset[Index] = Plan.Dz[Index];
		}
		TArray<FLegReach> AsLeft = Six.Legs;
		SupportMask(AsLeft, TConstArrayView<bool>(), Six.Tuning);
		DiscountLateLegs(AsLeft, Plan.Dz, Six.Tuning);
		const FVector2D Expected = Six.Attitude() + MeasuredCorrection(AsLeft);
		Six.In.bResume = true;
		Six.In.bResumeHasPlane = true;
		Six.In.ResumePlaneDeg = Plan.PlaneDeg;
		Six.In.ResumeDz = Plan.Dz;
		Six.In.Mode = EMode::Allowed;
		const FOutputs Adopted = Six.Step(0.0);
		ExpectAction(*this, TEXT("six standing legs: the hand-back"), Adopted, EAction::Adopt);
		AddInfo(FString::Printf(TEXT("hand-back with one standing leg left 0.5 cm short: ground read as (%.4f, %.4f); off the legs as left (%.4f, %.4f)"), Adopted.GroundDeg.X, Adopted.GroundDeg.Y, Expected.X,
			Expected.Y));
		TestTrue(TEXT("six standing legs: the ground is read off the legs as the hand-back left them"), (Adopted.GroundDeg - Expected).Size() <= 1.0e-9);
	}
	return true;
}

// Catches: a base left resting on the edge of the loop's deadband. Legs that
// give less than the law believes leave an error the integral trims only
// until it is back inside the deadband, where nothing acts: the rest is then
// declared 0.15 deg off, and stays. Standing still, the integral trims on to a
// narrower band before the rest; on the move it must not (the deadband is
// there to keep the legs from hunting over ground that moves under them).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLawRestOnLevel,
	"Ramms.Level.Law.RestIsOnLevelNotOnTheDeadbandEdge",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLawRestOnLevel::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLawTests;

	for (const FVector2D& GroundDeg : { FVector2D(6.0, 0.0), FVector2D(-4.0, 3.0) })
	{
		{
			// Parked, the legs giving 15 % less than the law believes.
			FToy Toy(12.0);
			Toy.JointShare = 0.85;
			Toy.GroundDeg = GroundDeg;
			const FOutputs	Out = Toy.Run(10.0);
			const FVector2D Off = Toy.Attitude();
			AddInfo(FString::Printf(TEXT("parked on (%.0f, %.0f) deg, legs giving 0.85: rests (%.3f, %.3f) deg off level; integral (%.2f, %.2f)"), GroundDeg.X, GroundDeg.Y, Off.X, Off.Y,
				Toy.State.Loop.GetIntegral().X, Toy.State.Loop.GetIntegral().Y));
			ExpectStatus(*this, TEXT("parked: at rest"), Out, EStatus::Settled);
			TestTrue(TEXT("(the integral had work to do)"), Toy.State.Loop.GetIntegral().Size() > 0.5);
			// (The trim has the settle time to work in, and the rest is declared
			// at its end whether or not the error is at the trim's band yet:
			// here it has come two thirds of the way from the deadband's edge.
			// A rest that waited for the band itself would never come on a base
			// whose attitude is not perfectly still.)
			TestTrue(FString::Printf(TEXT("parked: the rest is well inside the deadband, per axis nearer the rest trim's band than the edge (%.3f, %.3f)"), Off.X, Off.Y),
				FMath::Abs(Off.X) <= RestTrimDeadbandDeg + 0.045 && FMath::Abs(Off.Y) <= RestTrimDeadbandDeg + 0.045);
			int32 Writes = 0;
			for (int32 Index = 0; Index < 32; ++Index)
			{
				Writes += Toy.Step(Frame).bWrite ? 1 : 0;
			}
			TestEqual(TEXT("parked: at rest nothing is written"), Writes, 0);
		}
		{
			// The same, driving: the loop's own deadband, no narrower.
			FToy Toy(12.0);
			Toy.JointShare = 0.85;
			Toy.GroundDeg = GroundDeg;
			Toy.In.SpeedCmS = 30.0;
			Toy.In.bStick = true;
			Toy.Run(10.0);
			const FVector2D Off = Toy.Attitude();
			AddInfo(FString::Printf(TEXT("driving on (%.0f, %.0f) deg, legs giving 0.85: (%.3f, %.3f) deg off level"), GroundDeg.X, GroundDeg.Y, Off.X, Off.Y));
			TestTrue(TEXT("driving: inside the loop's deadband"), FMath::Abs(Off.X) <= Toy.Tuning.DeadbandDeg + 0.02 && FMath::Abs(Off.Y) <= Toy.Tuning.DeadbandDeg + 0.02);
			TestTrue(TEXT("driving: and not trimmed further (the deadband's edge on the axis with the slope)"), FMath::Abs(Off.X) >= Toy.Tuning.DeadbandDeg - 0.03);
		}
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
