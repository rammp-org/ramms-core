// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbAssist.h"
#include "Misc/AutomationTest.h"
#include "RammsCurbNegotiationTypes.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbAssistWatchTest, "Ramms.Curb.Assist.Watch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsCurbAssistWatchTest::RunTest(const FString& Parameters)
{
	FRammsCurbAssist Assist;
	FRammsCurbStick	 Stick; // engage 0.25, release 0.15
	Assist.ScanIntervalSeconds = 0.1f;
	const double			AssistTick = 0.02;
	RammsCurb::FAssistWatch Watch;

	// Off: it never looks, however the stick is held.
	Assist.bEnabled = false;
	TestFalse(TEXT("off: no look"), Watch.Step(Assist, Stick, 1.0, AssistTick));

	// On: at rest, or forward under EngageFraction, no look; pushed forward, it looks at once, then every interval.
	Assist.bEnabled = true;
	TestFalse(TEXT("at rest: no look"), Watch.Step(Assist, Stick, 0.0, AssistTick));
	TestFalse(TEXT("forward 0.2: no look"), Watch.Step(Assist, Stick, 0.2, AssistTick));
	TestFalse(TEXT("pulled back: no look"), Watch.Step(Assist, Stick, -1.0, AssistTick));
	TestTrue(TEXT("forward: looks at once"), Watch.Step(Assist, Stick, 0.6, AssistTick));
	int32 Looks = 0;
	for (int32 i = 0; i < 50; ++i) // 1 s
	{
		Looks += Watch.Step(Assist, Stick, 0.6, AssistTick) ? 1 : 0;
	}
	TestTrue(FString::Printf(TEXT("about every 0.1 s (%d looks in 1 s)"), Looks), Looks >= 9 && Looks <= 11);

	// A crossing it began that finished: it looks again at once, the stick still forward.
	Watch.CrossingEnded(true);
	TestFalse(TEXT("finished: not waiting for a let-go"), Watch.IsWaitingForLetGo());
	TestTrue(TEXT("finished: looks again"), Watch.Step(Assist, Stick, 0.6, AssistTick));

	// One that did not finish: no look while the stick stays forward, nor while it is eased off but over the release
	// share; once let go, the next push forward looks.
	Watch.CrossingEnded(false);
	TestTrue(TEXT("unfinished: waits for a let-go"), Watch.IsWaitingForLetGo());
	for (int32 i = 0; i < 100; ++i)
	{
		if (Watch.Step(Assist, Stick, 1.0, AssistTick))
		{
			AddError(TEXT("looked with the stick held forward after an unfinished crossing"));
			break;
		}
	}
	TestFalse(TEXT("eased to 0.2: still waiting"), Watch.Step(Assist, Stick, 0.2, AssistTick));
	TestTrue(TEXT("eased to 0.2: still waiting (state)"), Watch.IsWaitingForLetGo());
	TestFalse(TEXT("let go: no look yet"), Watch.Step(Assist, Stick, 0.0, AssistTick));
	TestFalse(TEXT("let go: armed again"), Watch.IsWaitingForLetGo());
	TestTrue(TEXT("pushed again: looks"), Watch.Step(Assist, Stick, 0.6, AssistTick));

	// Switched off while waiting: switched on again, it is not waiting.
	Watch.CrossingEnded(false);
	Assist.bEnabled = false;
	Watch.Step(Assist, Stick, 1.0, AssistTick);
	Assist.bEnabled = true;
	TestTrue(TEXT("off and on: looks"), Watch.Step(Assist, Stick, 1.0, AssistTick));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbAssistControlTest, "Ramms.Curb.Assist.Control",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsCurbAssistControlTest::RunTest(const FString& Parameters)
{
	const FRammsControlAxis Off = RammsCurb::MakeAssistControl(false, 3);
	const FRammsControlAxis On = RammsCurb::MakeAssistControl(true, 3);
	TestEqual(TEXT("id"), Off.Id, FName("curb.assist"));
	TestEqual(TEXT("group"), Off.Group, FName("Curb"));
	TestTrue(TEXT("a writable enum (a button on it steps it)"), Off.Kind == ERammsControlKind::Enum && !Off.bReadOnly);
	TestEqual(TEXT("two choices"), Off.EnumLabels.Num(), 2);
	TestEqual(TEXT("off shows 0"), Off.DefaultValue, 0.0f);
	TestEqual(TEXT("on shows 1"), On.DefaultValue, 1.0f);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
