// Tests for the RAMMP Gen1 curb crossing's arithmetic (Private/Curb/RammsRammpGen1Curb): the support height tables, the
// crossing height's band, finding a step along a line of ground samples, the rear-lift rule and the centre of
// mass the rule uses. Every number is made up, no robot's own: a support whose wheels rise 15 cm and fall 15 cm
// over +-45 deg, and one that rises only 5 cm (a sprung rear's short reach).

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Curb/RammsRammpGen1Curb.h"
#include "RammsRammpGen1CurbController.h"
#include "RammsControlTypes.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	/** A linear table: HeightAt(-45 deg) = Up, HeightAt(+45 deg) = -Down. */
	RammsRammpGen1Curb::FSupportTable RammpGen1CurbLinearTable(float UpCm, float DownCm)
	{
		RammsRammpGen1Curb::FSupportTable T;
		for (int32 i = 0; i <= 90; ++i)
		{
			const float A = FMath::DegreesToRadians(-45.0f + i);
			T.AngleRad.Add(A);
			T.HeightCm.Add(FMath::Lerp(UpCm, -DownCm, i / 90.0f));
		}
		return T;
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsRammpGen1CurbTableTest, "Ramms.Curb.RammpGen1.Table",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsRammpGen1CurbTableTest::RunTest(const FString& Parameters)
{
	const RammsRammpGen1Curb::FSupportTable T = RammpGen1CurbLinearTable(15.0f, 15.0f);
	TestTrue(TEXT("valid"), T.IsValid());
	TestNearlyEqual(TEXT("min"), T.MinHeight(), -15.0f, 1e-4f);
	TestNearlyEqual(TEXT("max"), T.MaxHeight(), 15.0f, 1e-4f);
	float A = 0.0f;
	TestTrue(TEXT("0 cm is in reach"), T.AngleFor(0.0f, A));
	TestNearlyEqual(TEXT("0 cm at 0 rad"), A, 0.0f, 1e-4f);
	TestTrue(TEXT("-10 cm is in reach"), T.AngleFor(-10.0f, A));
	TestNearlyEqual(TEXT("-10 cm at +30 deg"), FMath::RadiansToDegrees(A), 30.0f, 1e-2f);
	TestNearlyEqual(TEXT("and back"), T.HeightAt(A), -10.0f, 1e-3f);
	TestFalse(TEXT("16 cm is out of reach"), T.AngleFor(16.0f, A));
	TestNearlyEqual(TEXT("past the end clamps"), T.HeightAt(2.0f), -15.0f, 1e-4f);
	RammsRammpGen1Curb::FSupportTable Bad;
	Bad.AngleRad = { 0.0f };
	Bad.HeightCm = { 0.0f };
	TestFalse(TEXT("one entry is no table"), Bad.IsValid());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsRammpGen1CurbHeightTest, "Ramms.Curb.RammpGen1.CrossingHeight",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsRammpGen1CurbHeightTest::RunTest(const FString& Parameters)
{
	const RammsRammpGen1Curb::FSupportTable	 Long = RammpGen1CurbLinearTable(15.0f, 15.0f), Short = RammpGen1CurbLinearTable(5.0f, 14.0f);
	const RammsRammpGen1Curb::FSupportTable* Both[] = { &Long, &Short };
	float									 H = 0.0f;
	FString									 Why;
	// 12 cm, 3 cm clearance, a 5 cm belly with 2 cm margin: the frame needs 9, Long's lift 0 and Short's 10;
	// the lower level is in reach from 14 (Short): band 10..14, a quarter up = 11.
	TestTrue(TEXT("12 cm works"), RammsRammpGen1Curb::CrossingHeight(Both, 12.0f, 3.0f, 5.0f, 2.0f, H, Why));
	TestNearlyEqual(TEXT("a quarter up the band"), H, 11.0f, 1e-3f);
	TestTrue(TEXT("a step down is the same band"), RammsRammpGen1Curb::CrossingHeight(Both, -12.0f, 3.0f, 5.0f, 2.0f, H, Why));
	TestNearlyEqual(TEXT("down"), H, 11.0f, 1e-3f);
	// 17 cm: Short's lift needs 15, the reach stops at 14.
	TestFalse(TEXT("17 cm does not"), RammsRammpGen1Curb::CrossingHeight(Both, 17.0f, 3.0f, 5.0f, 2.0f, H, Why));
	TestTrue(TEXT("and says why"), Why.Contains(TEXT("no crossing height")));
	const RammsRammpGen1Curb::FSupportTable	 Empty;
	const RammsRammpGen1Curb::FSupportTable* WithEmpty[] = { &Long, &Empty };
	TestFalse(TEXT("a missing table refuses"), RammsRammpGen1Curb::CrossingHeight(WithEmpty, 10.0f, 3.0f, 5.0f, 2.0f, H, Why));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsRammpGen1CurbStepTest, "Ramms.Curb.RammpGen1.FindStep",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsRammpGen1CurbStepTest::RunTest(const FString& Parameters)
{
	// Ground at z 2 up to 73.4 cm along the line, then a 15 cm curb.
	const auto Up = [](float D, float& Z) {
		Z = D < 73.4f ? 2.0f : 17.0f;
		return true;
	};
	RammsRammpGen1Curb::FStep Step;
	FString					  Why;
	TestTrue(TEXT("finds the curb"), RammsRammpGen1Curb::FindStep(Up, 0.0f, 150.0f, 2.0f, 3.0f, Step, Why));
	TestNearlyEqual(TEXT("its edge to a millimetre"), Step.EdgeDistCm, 73.4f, 0.1f);
	TestNearlyEqual(TEXT("its height"), Step.HeightCm, 15.0f, 1e-3f);
	TestNearlyEqual(TEXT("the near level"), Step.NearZ, 2.0f, 1e-3f);

	const auto Down = [](float D, float& Z) {
		Z = D < 40.0f ? 15.0f : 0.0f;
		return true;
	};
	TestTrue(TEXT("finds a drop"), RammsRammpGen1Curb::FindStep(Down, 0.0f, 150.0f, 2.0f, 3.0f, Step, Why));
	TestNearlyEqual(TEXT("its height is negative"), Step.HeightCm, -15.0f, 1e-3f);

	const auto Bumpy = [](float D, float& Z) {
		Z = 0.5f * FMath::Sin(D * 0.3f); // a centimetre of ripple, no step
		return true;
	};
	TestFalse(TEXT("ripple is no step"), RammsRammpGen1Curb::FindStep(Bumpy, 0.0f, 150.0f, 2.0f, 3.0f, Step, Why));
	TestTrue(TEXT("and says so"), Why.Contains(TEXT("no step")));

	const auto Nothing = [](float, float&) { return false; };
	TestFalse(TEXT("no ground"), RammsRammpGen1Curb::FindStep(Nothing, 0.0f, 150.0f, 2.0f, 3.0f, Step, Why));

	// A 5% ramp climbs 7.5 cm over the scan, never steeply: no step.
	const auto Ramp = [](float D, float& Z) {
		Z = 0.05f * D;
		return true;
	};
	TestFalse(TEXT("a ramp is no step"), RammsRammpGen1Curb::FindStep(Ramp, 0.0f, 150.0f, 2.0f, 3.0f, Step, Why));

	// The ramp, then a 12 cm curb at 80.5 cm: its height is the face's, not the climb from the start.
	const auto RampCurb = [](float D, float& Z) {
		Z = 0.05f * D + (D >= 80.5f ? 12.0f : 0.0f);
		return true;
	};
	TestTrue(TEXT("a curb at the top of a ramp"), RammsRammpGen1Curb::FindStep(RampCurb, 0.0f, 150.0f, 2.0f, 3.0f, Step, Why));
	TestNearlyEqual(TEXT("at its edge"), Step.EdgeDistCm, 80.5f, 0.1f);
	TestNearlyEqual(TEXT("the face's height"), Step.HeightCm, 12.0f, 0.2f);
	TestNearlyEqual(TEXT("the ramp's level before it"), Step.NearZ, 4.0f, 0.2f);

	// No ground from 30 to 50 cm, then a platform: refused, not taken as a curb at 50.
	const auto Gap = [](float D, float& Z) {
		Z = D < 50.0f ? 0.0f : 10.0f;
		return D < 30.0f || D >= 50.0f;
	};
	TestFalse(TEXT("missing ground refuses"), RammsRammpGen1Curb::FindStep(Gap, 0.0f, 150.0f, 2.0f, 3.0f, Step, Why));
	TestTrue(TEXT("and says where"), Why.Contains(TEXT("no ground 30 cm")));

	// A 15 cm strip 4 cm wide, the road again past it: not a curb, up or down.
	const auto Strip = [](float D, float& Z) {
		Z = D >= 60.0f && D < 64.0f ? 15.0f : 0.0f;
		return true;
	};
	TestFalse(TEXT("a strip is not a curb"), RammsRammpGen1Curb::FindStep(Strip, 0.0f, 150.0f, 2.0f, 3.0f, Step, Why));
	TestTrue(TEXT("its far side is not level"), Why.Contains(TEXT("not a curb")));
	const auto Thin = [](float D, float& Z) {
		Z = D >= 59.0f && D < 61.0f ? 15.0f : 0.0f; // up and down between two samples' neighbours
		return true;
	};
	TestFalse(TEXT("a thin strip is not a curb"), RammsRammpGen1Curb::FindStep(Thin, 0.0f, 150.0f, 2.0f, 3.0f, Step, Why));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsRammpGen1CurbRuleTest, "Ramms.Curb.RammpGen1.RearLiftRule",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsRammpGen1CurbRuleTest::RunTest(const FString& Parameters)
{
	RammsRammpGen1Curb::FRearLiftCheck C;
	C.StepCm = 15.0f;
	for (int32 i = 0; i < 2; ++i)
	{
		C.PastEdgeCm[i] = 12.0f;
		C.bGroundFound[i] = true;
		C.GroundCm[i] = 0.1f;
	}
	C.GapCm[0] = 0.3f;
	C.GapCm[1] = -0.9f; // pressed in by the preload
	C.TranslatorCm[0] = C.TranslatorCm[1] = -30.0f;
	C.ComAheadOfAxleCm = 7.0f;
	FString Why;
	TestTrue(TEXT("down, aft, centre of mass ahead"), RammsRammpGen1Curb::RearLiftAllowed(C, Why));

	RammsRammpGen1Curb::FRearLiftCheck Short = C;
	Short.PastEdgeCm[1] = -2.0f;
	TestFalse(TEXT("one wheel short of the edge"), RammsRammpGen1Curb::RearLiftAllowed(Short, Why));
	TestTrue(TEXT("names it"), Why.Contains(TEXT("right drive wheel")));

	// Climbing, a wheel still on the road: 15 cm under the far level, its ground too.
	RammsRammpGen1Curb::FRearLiftCheck OnRoad = C;
	OnRoad.GroundCm[0] = -15.0f;
	OnRoad.GapCm[0] = -15.0f;
	TestFalse(TEXT("a wheel over the near level"), RammsRammpGen1Curb::RearLiftAllowed(OnRoad, Why));
	RammsRammpGen1Curb::FRearLiftCheck Below = C;
	Below.GapCm[0] = -15.0f;
	TestFalse(TEXT("a wheel below the far level"), RammsRammpGen1Curb::RearLiftAllowed(Below, Why));

	RammsRammpGen1Curb::FRearLiftCheck Up = C;
	Up.GapCm[1] = 2.0f;
	TestFalse(TEXT("a wheel not down"), RammsRammpGen1Curb::RearLiftAllowed(Up, Why));

	RammsRammpGen1Curb::FRearLiftCheck NoGround = C;
	NoGround.bGroundFound[0] = false;
	TestFalse(TEXT("no ground under a wheel"), RammsRammpGen1Curb::RearLiftAllowed(NoGround, Why));

	RammsRammpGen1Curb::FRearLiftCheck NotAft = C;
	NotAft.TranslatorCm[1] = 0.0f;
	TestFalse(TEXT("one translator not aft"), RammsRammpGen1Curb::RearLiftAllowed(NotAft, Why));
	TestTrue(TEXT("names it"), Why.Contains(TEXT("right")));

	RammsRammpGen1Curb::FRearLiftCheck Behind = C;
	Behind.ComAheadOfAxleCm = 1.0f;
	TestFalse(TEXT("centre of mass not far enough ahead"), RammsRammpGen1Curb::RearLiftAllowed(Behind, Why));

	// The drive assemblies carry the centre of mass with them: 100 kg at 30 cm, 15 kg each at the hubs.
	TestNearlyEqual(TEXT("hubs at 50"), RammsRammpGen1Curb::CentreOfMass(100.0f, 30.0f, 15.0f, 50.0f, 50.0f), (3000.0f + 1500.0f) / 130.0f, 1e-3f);
	TestNearlyEqual(TEXT("hubs aft at 20"), RammsRammpGen1Curb::CentreOfMass(100.0f, 30.0f, 15.0f, 20.0f, 20.0f), (3000.0f + 600.0f) / 130.0f, 1e-3f);
	TestNearlyEqual(TEXT("no mass"), RammsRammpGen1Curb::CentreOfMass(0.0f, 30.0f, 0.0f, 20.0f, 20.0f), 0.0f, 1e-6f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsRammpGen1CurbControlsTest, "Ramms.Curb.RammpGen1.Controls",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsRammpGen1CurbControlsTest::RunTest(const FString& Parameters)
{
	URammsRammpGen1CurbController* C = NewObject<URammsRammpGen1CurbController>(GetTransientPackage());
	FRammsControlSurface		   Surface;
	C->DescribeControls(Surface);
	TArray<FName> Ids;
	for (const FRammsControlAxis& A : Surface.Axes)
	{
		Ids.Add(A.Id);
	}
	TestTrue(TEXT("curb.negotiate"), Ids.Contains(FName("curb.negotiate")));
	TestTrue(TEXT("curb.abort"), Ids.Contains(FName("curb.abort")));
	TestTrue(TEXT("curb.phase"), Ids.Contains(FName("curb.phase")));
	float Value = -1.0f;
	TestTrue(TEXT("the phase reads"), C->ReadControl(FName("curb.phase"), Value));
	TestEqual(TEXT("idle"), Value, 0.0f);
	TestFalse(TEXT("the phase is not set"), C->ApplyControl(FName("curb.phase"), 3.0f));
	FString Why;
	TestFalse(TEXT("no robot: no crossing"), C->StartCrossing(Why));
	TestEqual(TEXT("drive mode"), C->GetDriveModeId(), FName("curb"));
	TArray<FName> Claimed;
	C->GetClaimedMotorIds(Claimed);
	TestEqual(TEXT("claims nothing while inactive"), Claimed.Num(), 0);

	// Curb assist's switch: on the surface, off at first; set, read back, flipped by a button.
	TestTrue(TEXT("curb.assist"), Ids.Contains(FName("curb.assist")));
	TestTrue(TEXT("assist: off at first"), C->ReadControl(FName("curb.assist"), Value) && Value == 0.0f);
	TestTrue(TEXT("assist: on"), C->ApplyControl(FName("curb.assist"), 1.0f) && C->Assist.bEnabled);
	TestTrue(TEXT("assist: the panel steps from it"), C->ReadTarget(FName("curb.assist"), Value) && Value == 1.0f);
	TestTrue(TEXT("assist: a button flips it off"), C->TriggerControl(FName("curb.assist")) && !C->Assist.bEnabled);
	// The stick: the drive controls are the curb mode's only while it has the robot.
	TestFalse(TEXT("inactive: no drive.forward"), Ids.Contains(FName("drive.forward")));
	TestFalse(TEXT("inactive: drive.forward not taken"), C->ApplyControl(FName("drive.forward"), 1.0f));
	TestTrue(TEXT("the stick is required by default"), C->Stick.bRequired);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
