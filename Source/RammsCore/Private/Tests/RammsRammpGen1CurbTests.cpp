// Tests for the RAMMP Gen1 curb crossing's arithmetic (Private/Curb/RammsRammpGen1Curb): the support height tables, the
// crossing height's band, finding a step along a line of ground samples, the rear-lift rule and the centre of
// mass the rule uses, the wheels on the ground and the stability margin over them; and the debug view's switches and
// HUD panel. Every number is made up, no robot's own: a support whose wheels rise 15 cm and fall 15 cm over +-45 deg,
// and one that rises only 5 cm (a sprung rear's short reach).

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "UObject/Package.h" // GetTransientPackage's UPackage, complete: NewObject's Outer

#include "Curb/RammsRammpGen1Curb.h"
#include "Curb/RammsRammpGen1CurbMeter.h"
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsRammpGen1CurbEdgeFrameTest, "Ramms.Curb.RammpGen1.EdgeFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsRammpGen1CurbEdgeFrameTest::RunTest(const FString& Parameters)
{
	const float EdgeHalfTrack = 30.0f;
	const auto	Check = [this, EdgeHalfTrack](const TCHAR* What, const FVector& Heading, float LeftCm, float RightCm) {
		 const RammsRammpGen1Curb::FEdgeFrame F = RammsRammpGen1Curb::EdgeFrame(Heading, LeftCm, RightCm, EdgeHalfTrack);
		 const FVector						  Fwd = Heading.GetSafeNormal2D();
		 const FVector						  Right(-Fwd.Y, Fwd.X, 0.0); // the heading turned 90 deg toward +Y
		 // Where the two lines meet the edge: the normal is square to the line between them, flat, and onto the far
		 // level (forward); it leans toward the side that meets it nearer; the angle is that lean.
		 const FVector OnLeft = -Right * EdgeHalfTrack + Fwd * LeftCm, OnRight = Right * EdgeHalfTrack + Fwd * RightCm;
		 TestTrue(FString::Printf(TEXT("%s: square to the edge"), What), FMath::Abs((OnRight - OnLeft).GetSafeNormal() | F.Normal) < 1.0e-4);
		 TestTrue(FString::Printf(TEXT("%s: flat, unit"), What), FMath::IsNearlyZero(F.Normal.Z) && FMath::IsNearlyEqual(F.Normal.Size(), 1.0, 1.0e-4));
		 TestTrue(FString::Printf(TEXT("%s: onto the far level"), What), (F.Normal | Fwd) > 0.0);
		 if (!FMath::IsNearlyEqual(LeftCm, RightCm))
		 {
			 TestTrue(FString::Printf(TEXT("%s: leans toward the side meeting it nearer"), What), ((F.Normal | Right) > 0.0) == (RightCm < LeftCm));
			 TestTrue(FString::Printf(TEXT("%s: positive when nearer on the right"), What), (F.OffSquareDeg > 0.0f) == (RightCm < LeftCm));
		 }
		 const double Lean = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(F.Normal | Fwd, -1.0, 1.0)));
		 TestTrue(FString::Printf(TEXT("%s: the angle is the lean (%.3f / %.3f deg)"), What, F.OffSquareDeg, Lean), FMath::IsNearlyEqual(FMath::Abs(F.OffSquareDeg), Lean, 1.0e-2));
	};
	Check(TEXT("square"), FVector(1, 0, 0), 50.0f, 50.0f);
	Check(TEXT("nearer on the right"), FVector(1, 0, 0), 60.0f, 40.0f);
	Check(TEXT("nearer on the left"), FVector(1, 0, 0), 40.0f, 60.0f);
	Check(TEXT("another heading"), FVector(0.6, -0.8, 0), 55.0f, 45.0f);
	Check(TEXT("a heading with Z"), FVector(-1, 0.2, 0.3), 45.0f, 55.0f);
	const RammsRammpGen1Curb::FEdgeFrame Square = RammsRammpGen1Curb::EdgeFrame(FVector(1, 0, 0), 50.0f, 50.0f, EdgeHalfTrack);
	TestTrue(TEXT("square: 0 deg, the heading"), FMath::IsNearlyZero(Square.OffSquareDeg) && Square.Normal.Equals(FVector(1, 0, 0), 1.0e-4));
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

	// The status: read-only words, and how much they need the rider; a refused start says why.
	const FRammsControlAxis* StatusAxis = Surface.Axes.FindByPredicate([](const FRammsControlAxis& A) { return A.Id == FName("curb.status"); });
	TestTrue(TEXT("curb.status: a read-only Text control"), StatusAxis && StatusAxis->Kind == ERammsControlKind::Text && StatusAxis->bReadOnly);
	FText Words;
	TestTrue(TEXT("status: idle, ready"), C->ReadControlText(FName("curb.status"), Words) && Words.ToString().StartsWith(TEXT("Ready")));
	TestTrue(TEXT("status: information"), C->ReadControl(FName("curb.status"), Value) && Value == static_cast<float>(ERammsControlTextSeverity::Info));
	TestFalse(TEXT("status: nothing to set"), C->ApplyControl(FName("curb.status"), 1.0f));
	TestFalse(TEXT("a start with no robot is refused"), C->TriggerControl(FName("curb.negotiate")));
	TestTrue(TEXT("status: why it was refused"), C->ReadControlText(FName("curb.status"), Words) && Words.ToString().StartsWith(TEXT("Not crossing: ")));
	TestTrue(TEXT("status: for the rider's attention"),
		C->ReadControl(FName("curb.status"), Value) && Value == static_cast<float>(ERammsControlTextSeverity::Attention));
	TestFalse(TEXT("status: no words for other controls"), C->ReadControlText(FName("curb.phase"), Words));
	TArray<FName> Claimed;
	C->GetClaimedMotorIds(Claimed);
	TestEqual(TEXT("claims nothing while inactive"), Claimed.Num(), 0);

	// Curb assist's switch: on the surface, off at first; set, read back, flipped by a button.
	TestTrue(TEXT("curb.assist"), Ids.Contains(FName("curb.assist")));
	TestTrue(TEXT("assist: off at first"), C->ReadControl(FName("curb.assist"), Value) && Value == 0.0f);
	TestTrue(TEXT("assist: on"), C->ApplyControl(FName("curb.assist"), 1.0f) && C->Assist.bEnabled);
	TestTrue(TEXT("assist: the panel steps from it"), C->ReadTarget(FName("curb.assist"), Value) && Value == 1.0f);
	TestTrue(TEXT("assist: a button flips it off"), C->TriggerControl(FName("curb.assist")) && !C->Assist.bEnabled);
	// The crossing's style: on the surface, a two-way switch; rolling at first; set, read back.
	const FRammsControlAxis* StyleAxis = Surface.Axes.FindByPredicate([](const FRammsControlAxis& A) { return A.Id == FName("curb.style"); });
	TestTrue(TEXT("curb.style"), StyleAxis && StyleAxis->Kind == ERammsControlKind::Enum && !StyleAxis->bReadOnly && StyleAxis->EnumLabels.Num() == 2);
	TestTrue(TEXT("style: rolling at first"), C->Style == ERammsRammpGen1CurbStyle::Blended && C->ReadControl(FName("curb.style"), Value) && Value == 1.0f);
	TestTrue(TEXT("style: stop and step"), C->ApplyControl(FName("curb.style"), 0.0f) && C->Style == ERammsRammpGen1CurbStyle::Stepped);
	TestTrue(TEXT("style: the panel steps from it"), C->ReadTarget(FName("curb.style"), Value) && Value == 0.0f);
	TestTrue(TEXT("style: rolling again"), C->ApplyControl(FName("curb.style"), 1.0f) && C->Style == ERammsRammpGen1CurbStyle::Blended);
	// The stick: the drive controls are the curb mode's only while it has the robot.
	TestFalse(TEXT("inactive: no drive.forward"), Ids.Contains(FName("drive.forward")));
	TestFalse(TEXT("inactive: drive.forward not taken"), C->ApplyControl(FName("drive.forward"), 1.0f));
	TestTrue(TEXT("the stick is required by default"), C->Stick.bRequired);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsRammpGen1CurbMeterTest, "Ramms.Curb.RammpGen1.Meter",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsRammpGen1CurbMeterTest::RunTest(const FString& Parameters)
{
	// Made-up motion, sampled at 100 Hz: at rest 1 s, speeding up at 50 cm/s^2 for 0.5 s (to 25 cm/s), rolling 1 s,
	// braking at 50 cm/s^2 for 0.5 s, at rest 1 s, then speeding up again the same way. The rider rides the chassis. The
	// wheels stand at the corners of a 100 cm square around the origin and the centre of mass 10 cm ahead of its
	// centre; the frame passes over the edge from 2 s to 3 s at 5, then 3, then 4 cm.
	RammsRammpGen1Curb::FCrossingMeter Meter;
	Meter.Begin(0.0);
	double		 X = 0.0, V = 0.0;
	const double Dt = 0.01;
	for (int32 i = 0; i <= 500; ++i)
	{
		const double					 T = i * Dt;
		const double					 A = ((T >= 1.0 && T < 1.5) || (T >= 4.0 && T < 4.5)) ? 50.0 : (T >= 2.5 && T < 3.0) ? -50.0
																															 : 0.0;
		RammsRammpGen1Curb::FMeterSample S;
		S.TimeS = T;
		S.ChassisPos = FVector(X, 0.0, 30.0);
		S.RiderPos = FVector(X, 0.0, 80.0);
		S.PitchDeg = (T >= 2.0 && T < 2.5) ? 2.0f : 0.0f;
		S.Contacts = { FVector2D(50, 50), FVector2D(50, -50), FVector2D(-50, 50), FVector2D(-50, -50) };
		S.ComXY = FVector2D(10.0, 0.0);
		S.bFrameOverEdge = T >= 2.0 && T < 3.0;
		S.FrameClearanceCm = T < 2.3 ? 5.0f : T < 2.6 ? 3.0f
													  : 4.0f;
		S.Phase = T < 2.3 ? ERammsRammpGen1CurbPhase::DriveUp : ERammsRammpGen1CurbPhase::DriveOver;
		Meter.Add(S);
		if (FMath::IsNearlyEqual(T, 1.2))
		{
			Meter.MarkFrontAtEdge(T);
		}
		if (FMath::IsNearlyEqual(T, 3.7))
		{
			Meter.MarkRearDown(T);
		}
		V += A * Dt;
		X += V * Dt;
	}
	Meter.End(5.0);
	const FRammsRammpGen1CurbStats& St = Meter.GetStats();
	TestEqual(TEXT("crossing seconds"), St.CrossingSeconds, 5.0f, 1.0e-3f);
	TestEqual(TEXT("front at the edge -> rear down"), St.EdgeToRearDownSeconds, 2.5f, 1.0e-3f);
	TestEqual(TEXT("front at the edge -> end"), St.EdgeToEndSeconds, 3.8f, 1.0e-3f);
	TestEqual(TEXT("one stop (the start's rest is before it first moved)"), St.Stops, 1);
	TestTrue(TEXT("stopped about 1 s"), St.SecondsStopped > 0.9f && St.SecondsStopped < 1.25f);
	TestTrue(FString::Printf(TEXT("peak fore/aft acceleration about 0.5 m/s^2 (%.2f)"), St.PeakRiderForeAftAccelMS2),
		St.PeakRiderForeAftAccelMS2 > 0.4f && St.PeakRiderForeAftAccelMS2 < 0.6f);
	TestTrue(TEXT("no vertical acceleration"), St.PeakRiderVerticalAccelMS2 < 0.01f);
	TestEqual(TEXT("pitch"), St.MaxAbsPitchDeg, 2.0f, 1.0e-4f);
	TestEqual(TEXT("margin: 40 cm to the square's front edge"), St.MinStabilityMarginCm, 40.0f, 1.0e-3f);
	TestTrue(TEXT("the frame measured"), St.bFrameMeasured);
	TestEqual(TEXT("the frame's least clearance"), St.MinFrameClearanceCm, 3.0f, 1.0e-4f);
	TestEqual(TEXT("...and when"), St.MinClearancePhase, ERammsRammpGen1CurbPhase::DriveOver);
	TestFalse(TEXT("off once ended"), Meter.IsOn());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsRammpGen1CurbSupportTest, "Ramms.Curb.RammpGen1.Support",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsRammpGen1CurbSupportTest::RunTest(const FString& Parameters)
{
	using RammsRammpGen1Curb::FWheelSample;
	// Four 10 cm wheels at the corners of a 100 x 60 cm rectangle on level ground at z 0 -- one just touching, one 0.5 cm
	// over it, one pressed 0.5 cm in, one touching -- a fifth 3 cm up, and a sixth over no ground at all.
	const auto Wheel = [](double X, double Y, double HubZ, bool bGround) {
		FWheelSample W;
		W.Hub = FVector(X, Y, HubZ);
		W.RadiusCm = 10.0f;
		W.bGroundFound = bGround;
		W.GroundZ = 0.0f;
		return W;
	};
	const FWheelSample Wheels[] = { Wheel(50.0, 30.0, 10.0, true), Wheel(50.0, -30.0, 10.5, true), Wheel(-50.0, 30.0, 9.5, true),
		Wheel(-50.0, -30.0, 10.0, true), Wheel(0.0, 0.0, 13.0, true), Wheel(80.0, 0.0, 10.0, false) };
	TestNearlyEqual(TEXT("a gap over the ground"), Wheels[1].GapCm(), 0.5f, 1e-4f);
	TestNearlyEqual(TEXT("pressed in: negative"), Wheels[2].GapCm(), -0.5f, 1e-4f);
	TestNearlyEqual(TEXT("the lifted wheel's"), Wheels[4].GapCm(), 3.0f, 1e-4f);
	TestFalse(TEXT("over no ground: not down"), Wheels[5].IsDown(0.8f));

	const TArray<FVector> Contacts = RammsRammpGen1Curb::GroundContacts(Wheels, 0.8f);
	TestEqual(TEXT("the four corners are down, within the tolerance"), Contacts.Num(), 4);
	TestTrue(TEXT("...each under its hub, on the ground"), Contacts.Num() == 4 && Contacts[1].Equals(FVector(50.0, -30.0, 0.0)));
	TestEqual(TEXT("a tighter tolerance leaves out the one 0.5 cm over"), RammsRammpGen1Curb::GroundContacts(Wheels, 0.3f).Num(), 3);

	TArray<FVector2D> Flat;
	for (const FVector& C : Contacts)
	{
		Flat.Add(FVector2D(C.X, C.Y));
	}
	TestNearlyEqual(TEXT("inside: to the nearest side, 30 cm"), RammsRammpGen1Curb::SupportMarginCm(Flat, FVector2D(10.0, 0.0)), 30.0f, 1e-3f);
	TestNearlyEqual(TEXT("outside, ahead: negative"), RammsRammpGen1Curb::SupportMarginCm(Flat, FVector2D(60.0, 0.0)), -10.0f, 1e-3f);
	const FVector2D Pair[] = { FVector2D(50.0, 30.0), FVector2D(50.0, -30.0) };
	TestNearlyEqual(TEXT("two wheels down: a line, no better than on it"), RammsRammpGen1Curb::SupportMarginCm(Pair, FVector2D(40.0, 0.0)), -10.0f, 1e-3f);
	TestEqual(TEXT("none down: the lowest margin"), RammsRammpGen1Curb::SupportMarginCm({}, FVector2D::ZeroVector), RammsRammpGen1Curb::NoSupportMarginCm);

	// The mass model in space: 100 kg 30 cm ahead and 50 up, 15 kg at each hub.
	const FVector Com = RammsRammpGen1Curb::CentreOfMassAt(100.0f, FVector(30.0, 0.0, 50.0), 15.0f, FVector(0.0, 25.0, 20.0), FVector(0.0, -25.0, 20.0));
	TestTrue(TEXT("the weighted mean"), Com.Equals(FVector(3000.0 / 130.0, 0.0, (5000.0 + 600.0) / 130.0), 1e-3));
	TestNearlyEqual(TEXT("along the heading, as the rule's"), static_cast<float>(Com.X), RammsRammpGen1Curb::CentreOfMass(100.0f, 30.0f, 15.0f, 0.0f, 0.0f), 1e-3f);
	TestTrue(TEXT("no mass: zero"), RammsRammpGen1Curb::CentreOfMassAt(0.0f, FVector(1.0), 0.0f, FVector(2.0), FVector(3.0)).IsZero());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsRammpGen1CurbViewTest, "Ramms.Curb.RammpGen1.View",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsRammpGen1CurbViewTest::RunTest(const FString& Parameters)
{
	URammsRammpGen1CurbController* C = NewObject<URammsRammpGen1CurbController>(GetTransientPackage());
	const FName					   Terrain("curb.view.terrain"), Robot("curb.view.robot"), Graphs("curb.view.graphs"), Labels("curb.view.labels");
	FRammsControlSurface		   Surface;
	C->DescribeControls(Surface);
	TArray<FRammsControlAxis> View;
	Surface.GetAxesInGroup(FName("Curb view"), View);
	TArray<FName> Ids;
	for (const FRammsControlAxis& A : View)
	{
		Ids.Add(A.Id);
		TestTrue(*FString::Printf(TEXT("%s: an on/off choice"), *A.Id.ToString()), A.Kind == ERammsControlKind::Enum && !A.bReadOnly && A.EnumLabels.Num() == 2);
		TestEqual(*FString::Printf(TEXT("%s: shown as it is set"), *A.Id.ToString()), A.DefaultValue, A.Id == Labels ? 1.0f : 0.0f);
	}
	TestEqual(TEXT("the layers Gen1 has: four switches in Gen2's group"), View.Num(), 4);
	TestTrue(TEXT("terrain, robot, graphs, labels"), Ids.Contains(Terrain) && Ids.Contains(Robot) && Ids.Contains(Graphs) && Ids.Contains(Labels));
	TestFalse(TEXT("no sensing switch"), Ids.Contains(FName("curb.view.sensing")));
	TestFalse(TEXT("no plan switch"), Ids.Contains(FName("curb.view.plan")));

	float Value = -1.0f;
	TestTrue(TEXT("graphs on"), C->ApplyControl(Graphs, 1.0f) && C->DebugView.bGraphs);
	TestTrue(TEXT("...read back"), C->ReadControl(Graphs, Value) && Value == 1.0f);
	TestTrue(TEXT("...the panel steps from it"), C->ReadTarget(Graphs, Value) && Value == 1.0f);
	TestFalse(TEXT("sensing is not Gen1's"), C->ApplyControl(FName("curb.view.sensing"), 2.0f));
	TestTrue(TEXT("...left off"), C->DebugView.Sensing == ERammsCurbSensingView::Off);
	TestFalse(TEXT("the plan is not Gen1's"), C->ApplyControl(FName("curb.view.plan"), 1.0f) || C->DebugView.bPlan);
	TestTrue(TEXT("the style is still its own switch"), C->ReadTarget(FName("curb.style"), Value) && Value == 1.0f);

	// The HUD panel: Gen2's, shown while it has something to show.
	TArray<FRammsHUDPanelDesc> Panels;
	C->GetHUDPanels(Panels);
	TestEqual(TEXT("one panel"), Panels.Num(), 1);
	if (Panels.Num() == 1)
	{
		const FRammsHUDPanelDesc& Panel = Panels[0];
		TestEqual(TEXT("the curb view, as Gen2's"), Panel.Id, FName("CurbView"));
		TestTrue(TEXT("with content and a test of when it is wanted"), static_cast<bool>(Panel.MakeContent) && static_cast<bool>(Panel.IsWanted));
		if (Panel.IsWanted)
		{
			TestTrue(TEXT("wanted with the graphs on"), Panel.IsWanted());
			C->ApplyControl(Graphs, 0.0f);
			TestFalse(TEXT("nothing to show: labels on, but no layer drawn to key"), Panel.IsWanted());
			C->ApplyControl(Robot, 1.0f);
			TestTrue(TEXT("the robot layer's colour key"), Panel.IsWanted());
			C->ApplyControl(Labels, 0.0f);
			TestFalse(TEXT("no labels, no key"), Panel.IsWanted());
		}
	}

	C->DebugView.bExposeControls = false;
	FRammsControlSurface Hidden;
	C->DescribeControls(Hidden);
	TArray<FRammsControlAxis> HiddenView;
	Hidden.GetAxesInGroup(FName("Curb view"), HiddenView);
	TestEqual(TEXT("not offered when asked not to"), HiddenView.Num(), 0);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
