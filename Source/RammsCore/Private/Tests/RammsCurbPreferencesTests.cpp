// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbPreferences.h"
#include "Misc/AutomationTest.h"
#include "RammsControlTypes.h"
#include "RammsCurbNegotiationTypes.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPreferencesMapTest, "Ramms.Curb.Preferences.Map",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsCurbPreferencesMapTest::RunTest(const FString& Parameters)
{
	FRammsCurbTuning T;
	T.DriveSpeed = 45.0f;
	T.Acceleration = 60.0f;
	T.Deceleration = 60.0f;
	T.TiltRateDegPerSec = 3.0f;
	T.TiltAccelDegPerSec2 = 4.0f;
	T.MaxPlannedTiltDeg = 5.0f;

	// The middle of every slider: the tuning as it is set, exactly.
	const FRammsCurbPreferences Mid;
	const FRammsCurbTuning		M = RammsCurb::ApplyPreferences(T, Mid);
	TestEqual(TEXT("mid: speed"), M.DriveSpeed, T.DriveSpeed);
	TestEqual(TEXT("mid: acceleration"), M.Acceleration, T.Acceleration);
	TestEqual(TEXT("mid: deceleration"), M.Deceleration, T.Deceleration);
	TestEqual(TEXT("mid: tilt rate"), M.TiltRateDegPerSec, T.TiltRateDegPerSec);
	TestEqual(TEXT("mid: tilt accel"), M.TiltAccelDegPerSec2, T.TiltAccelDegPerSec2);
	TestEqual(TEXT("mid: max planned tilt"), M.MaxPlannedTiltDeg, T.MaxPlannedTiltDeg);
	TestEqual(TEXT("mid: what the sliders leave alone"), M.MinStabilityMargin, T.MinStabilityMargin);

	// The ends.
	FRammsCurbPreferences P;
	P.Speed = 0.0f;
	TestTrue(TEXT("slowest: 0.6x"), FMath::IsNearlyEqual(RammsCurb::ApplyPreferences(T, P).DriveSpeed, 27.0f, 1e-3f));
	P.Speed = 1.0f;
	TestTrue(TEXT("fastest: 1.3x"), FMath::IsNearlyEqual(RammsCurb::ApplyPreferences(T, P).DriveSpeed, 58.5f, 1e-3f));
	P.Speed = 0.75f;
	TestTrue(TEXT("between: linear"), FMath::IsNearlyEqual(RammsCurb::ApplyPreferences(T, P).DriveSpeed, 45.0f * 1.15f, 1e-3f));
	P = FRammsCurbPreferences();
	P.Smoothness = 0.0f;
	const FRammsCurbTuning Brisk = RammsCurb::ApplyPreferences(T, P);
	TestTrue(TEXT("brisk: 1.5x accel, braking, tilt rates"), FMath::IsNearlyEqual(Brisk.Acceleration, 90.0f, 1e-3f) && FMath::IsNearlyEqual(Brisk.Deceleration, 90.0f, 1e-3f) && FMath::IsNearlyEqual(Brisk.TiltRateDegPerSec, 4.5f, 1e-3f) && FMath::IsNearlyEqual(Brisk.TiltAccelDegPerSec2, 6.0f, 1e-3f));
	P.Smoothness = 1.0f;
	const FRammsCurbTuning Gentle = RammsCurb::ApplyPreferences(T, P);
	TestTrue(TEXT("gentle: 0.6x"), FMath::IsNearlyEqual(Gentle.Acceleration, 36.0f, 1e-3f) && FMath::IsNearlyEqual(Gentle.TiltRateDegPerSec, 1.8f, 1e-3f));
	TestEqual(TEXT("smoothness leaves the pace"), Gentle.DriveSpeed, T.DriveSpeed);
	P = FRammsCurbPreferences();
	P.Tilt = 0.0f;
	TestEqual(TEXT("tilt 0: level only"), RammsCurb::ApplyPreferences(T, P).MaxPlannedTiltDeg, 0.0f);
	P.Tilt = 1.0f;
	TestTrue(TEXT("tilt 1: 1.6x"), FMath::IsNearlyEqual(RammsCurb::ApplyPreferences(T, P).MaxPlannedTiltDeg, 8.0f, 1e-3f));
	T.MaxPlannedTiltDeg = 8.0f;
	TestTrue(TEXT("tilt 1: no more than its ceiling"), FMath::IsNearlyEqual(RammsCurb::ApplyPreferences(T, P).MaxPlannedTiltDeg, 10.0f, 1e-3f));
	P.Tilt = 7.0f;
	TestTrue(TEXT("out of range: clamped"), FMath::IsNearlyEqual(RammsCurb::ApplyPreferences(T, P).MaxPlannedTiltDeg, 10.0f, 1e-3f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbPreferencesControlsTest, "Ramms.Curb.Preferences.Controls",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsCurbPreferencesControlsTest::RunTest(const FString& Parameters)
{
	FRammsCurbPreferences Prefs;
	FRammsControlSurface  Surface;
	RammsCurb::DescribePreferenceControls(Surface, Prefs, 5);
	const FName Ids[3] = { RammsCurb::PreferenceSpeedControlId(), RammsCurb::PreferenceSmoothnessControlId(), RammsCurb::PreferenceTiltControlId() };
	for (const FName Id : Ids)
	{
		const FRammsControlAxis* A = Surface.Find(Id);
		TestTrue(FString::Printf(TEXT("%s: a 0..1 slider in Curb, at rest 0.5"), *Id.ToString()),
			A && A->Kind == ERammsControlKind::Position && A->Group == FName("Curb") && A->Range == FVector2D(0.0, 1.0) && A->DefaultValue == 0.5f
				&& !A->bReadOnly);
	}
	TestTrue(TEXT("set speed"), RammsCurb::ApplyPreferenceControl(Prefs, Ids[0], 0.8f));
	TestEqual(TEXT("speed set"), Prefs.Speed, 0.8f);
	TestTrue(TEXT("set smoothness past the end: clamped"), RammsCurb::ApplyPreferenceControl(Prefs, Ids[1], 3.0f));
	TestEqual(TEXT("smoothness clamped"), Prefs.Smoothness, 1.0f);
	float Value = -1.0f;
	TestTrue(TEXT("read tilt"), RammsCurb::ReadPreferenceControl(Prefs, Ids[2], Value));
	TestEqual(TEXT("tilt as it was"), Value, 0.5f);
	TestFalse(TEXT("another Id: not one of them"), RammsCurb::ApplyPreferenceControl(Prefs, FName("curb.assist"), 1.0f));
	TestFalse(TEXT("another Id: no value"), RammsCurb::ReadPreferenceControl(Prefs, FName("curb.assist"), Value));
	return true;
}

#endif
