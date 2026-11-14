// Tests for the rider's stick in a curb crossing (Private/Curb/RammsCurbStick).
//
// The stick read each tick on made-up input: forward only once pushed far
// enough within the forward sector, then forward through a brief waver,
// let go, sideways and back read at once out of forward, and how long it has
// been held back. And what that does to a crossing at each stage: go, pause or
// cancel -- and, once the rear wheels can touch down on the far surface,
// finishing in place (once out of forward for the confirm), held to the end.
// And the hand-back at Done: never with the stick pulled back, drive.forward
// made to come back to centre where the stick is held at all backward, and
// the stick's X paired as the mode handed back to pairs it.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Curb/RammsCurbStick.h"
#include "RammsControlIds.h"
#include "RammsControlInputResolve.h"
#include "RammsCurbNegotiationTypes.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	using RammsCurb::EStickIntent;
	using RammsCurb::FStickGate;

	constexpr double StickDt = 1.0 / 60.0;

	/** The gate held at (Forward, Turn, Strafe) for Seconds; its last reading. */
	EStickIntent StickHold(FStickGate& Gate, const FRammsCurbStick& S, double Forward, double Turn, double Strafe, double Seconds)
	{
		EStickIntent Out = Gate.GetIntent();
		for (int32 i = FMath::RoundToInt(Seconds / StickDt); i > 0; --i)
		{
			Out = Gate.Step(S, Forward, Turn, Strafe, StickDt);
		}
		return Out;
	}

	bool StickIs(FAutomationTestBase& Test, const TCHAR* What, EStickIntent Got, EStickIntent Want)
	{
		return Test.TestEqual(FString::Printf(TEXT("%s: reads %s"), What, RammsCurb::StickIntentName(Got)), FString(RammsCurb::StickIntentName(Got)),
			FString(RammsCurb::StickIntentName(Want)));
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStickEngage,
	"Ramms.Curb.Stick.Engage",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbStickEngage::RunTest(const FString& Parameters)
{
	const FRammsCurbStick S;
	FStickGate			  Gate;
	StickIs(*this, TEXT("at rest"), StickHold(Gate, S, 0.0, 0.0, 0.0, 0.2), EStickIntent::LetGo);
	// Forward, but under EngageFraction: not enough to go.
	StickIs(*this, TEXT("forward 0.2"), StickHold(Gate, S, 0.2, 0.0, 0.0, 0.2), EStickIntent::LetGo);
	TestEqual(TEXT("no share asked under EngageFraction"), Gate.GetScale(), 0.0);
	// Past it: forward at once, at the share pushed.
	StickIs(*this, TEXT("forward 0.3"), Gate.Step(S, 0.3, 0.0, 0.0, StickDt), EStickIntent::Forward);
	TestEqual(TEXT("the share asked is how far the stick is pushed"), Gate.GetScale(), 0.3);
	StickIs(*this, TEXT("forward 1.0"), Gate.Step(S, 1.0, 0.0, 0.0, StickDt), EStickIntent::Forward);
	TestEqual(TEXT("full forward asks the plan's own speed"), Gate.GetScale(), 1.0);
	// Within the sector of straight ahead (30 deg): forward with a little turn
	// (about 22 deg round) or strafe.
	Gate.Reset();
	StickIs(*this, TEXT("forward with a little turn"), Gate.Step(S, 0.5, 0.2, 0.0, StickDt), EStickIntent::Forward);
	Gate.Reset();
	StickIs(*this, TEXT("forward with a little strafe"), Gate.Step(S, 0.5, 0.0, -0.2, StickDt), EStickIntent::Forward);
	// Too far round (about 39 deg): sideways, never forward.
	Gate.Reset();
	StickIs(*this, TEXT("forward and turning hard"), StickHold(Gate, S, 0.5, 0.4, 0.0, 0.5), EStickIntent::Sideways);
	// An EngageFraction under ReleaseFraction is taken as ReleaseFraction.
	FRammsCurbStick Low;
	Low.EngageFraction = 0.05f;
	Low.ReleaseFraction = 0.2f;
	Gate.Reset();
	StickIs(*this, TEXT("forward 0.1 against a release of 0.2"), StickHold(Gate, Low, 0.1, 0.0, 0.0, 0.2), EStickIntent::LetGo);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStickRelease,
	"Ramms.Curb.Stick.Release",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbStickRelease::RunTest(const FString& Parameters)
{
	const FRammsCurbStick S;
	FStickGate			  Gate;
	StickHold(Gate, S, 0.6, 0.0, 0.0, 0.1);
	// Eased back to between ReleaseFraction and EngageFraction: still forward,
	// at that share.
	StickIs(*this, TEXT("eased to 0.18"), StickHold(Gate, S, 0.18, 0.0, 0.0, 1.0), EStickIntent::Forward);
	TestEqual(TEXT("at the share it is held at"), Gate.GetScale(), 0.18);
	// A waver under ReleaseSeconds: forward throughout, at the share it had.
	StickIs(*this, TEXT("a 0.1 s waver to rest"), StickHold(Gate, S, 0.0, 0.0, 0.0, 0.1), EStickIntent::Forward);
	TestEqual(TEXT("the share kept through the waver"), Gate.GetScale(), 0.18);
	StickIs(*this, TEXT("back to 0.18 after the waver"), Gate.Step(S, 0.18, 0.0, 0.0, StickDt), EStickIntent::Forward);
	// Let go for longer: let go once ReleaseSeconds have passed.
	StickIs(*this, TEXT("let go 0.1 s"), StickHold(Gate, S, 0.0, 0.0, 0.0, 0.1), EStickIntent::Forward);
	StickIs(*this, TEXT("let go 0.2 s"), StickHold(Gate, S, 0.0, 0.0, 0.0, 0.1), EStickIntent::LetGo);
	TestEqual(TEXT("no share asked let go"), Gate.GetScale(), 0.0);
	// Forward, then swung round to the side: sideways once ReleaseSeconds have
	// passed.
	StickHold(Gate, S, 0.6, 0.0, 0.0, 0.1);
	StickIs(*this, TEXT("swung to the side 0.1 s"), StickHold(Gate, S, 0.2, 0.6, 0.0, 0.1), EStickIntent::Forward);
	StickIs(*this, TEXT("swung to the side 0.2 s"), StickHold(Gate, S, 0.2, 0.6, 0.0, 0.1), EStickIntent::Sideways);
	// Out of forward, what it reads changes at once.
	StickIs(*this, TEXT("to rest from sideways"), Gate.Step(S, 0.05, 0.05, 0.0, StickDt), EStickIntent::LetGo);
	StickIs(*this, TEXT("pulled back from rest"), Gate.Step(S, -0.6, 0.0, 0.0, StickDt), EStickIntent::Back);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStickBack,
	"Ramms.Curb.Stick.Back",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbStickBack::RunTest(const FString& Parameters)
{
	const FRammsCurbStick S;
	FStickGate			  Gate;
	// Within the sector of straight back: back, its time counted without a
	// break.
	StickIs(*this, TEXT("pulled back"), StickHold(Gate, S, -0.6, 0.0, 0.0, 0.5), EStickIntent::Back);
	TestTrue(FString::Printf(TEXT("held back 0.5 s (%.3f)"), Gate.GetBackSeconds()), FMath::IsNearlyEqual(Gate.GetBackSeconds(), 0.5, 0.02));
	StickIs(*this, TEXT("back and a little to the side"), StickHold(Gate, S, -0.6, 0.2, 0.0, 0.25), EStickIntent::Back);
	TestTrue(FString::Printf(TEXT("still counting (%.3f)"), Gate.GetBackSeconds()), FMath::IsNearlyEqual(Gate.GetBackSeconds(), 0.75, 0.02));
	// Back and too far to the side (about 39 deg off straight back): sideways,
	// and the count starts afresh.
	StickIs(*this, TEXT("back and well to the side"), Gate.Step(S, -0.5, 0.4, 0.0, StickDt), EStickIntent::Sideways);
	TestEqual(TEXT("the count broken"), Gate.GetBackSeconds(), 0.0);
	StickIs(*this, TEXT("pulled back again"), StickHold(Gate, S, -0.6, 0.0, 0.0, 0.2), EStickIntent::Back);
	TestTrue(FString::Printf(TEXT("counted afresh (%.3f)"), Gate.GetBackSeconds()), FMath::IsNearlyEqual(Gate.GetBackSeconds(), 0.2, 0.02));
	// Under ReleaseFraction it is at rest, whichever way it points.
	StickIs(*this, TEXT("a little back"), Gate.Step(S, -0.1, 0.0, 0.0, StickDt), EStickIntent::LetGo);
	TestEqual(TEXT("not counted"), Gate.GetBackSeconds(), 0.0);
	// Held forward, pulled back: back once ReleaseSeconds have passed, counted
	// from then.
	StickHold(Gate, S, 0.6, 0.0, 0.0, 0.1);
	StickIs(*this, TEXT("forward to back, 0.1 s"), StickHold(Gate, S, -0.6, 0.0, 0.0, 0.1), EStickIntent::Forward);
	StickIs(*this, TEXT("forward to back, 0.2 s"), StickHold(Gate, S, -0.6, 0.0, 0.0, 0.1), EStickIntent::Back);
	TestTrue(FString::Printf(TEXT("counted from there (%.3f)"), Gate.GetBackSeconds()), Gate.GetBackSeconds() < 0.1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStickSeconds,
	"Ramms.Curb.Stick.Seconds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbStickSeconds::RunTest(const FString& Parameters)
{
	// How long the stick has read what it reads: what a cancel waits on before
	// any leg has moved (ReleaseSeconds), so that a stick on its way forward
	// that passes off the axis does not cancel.
	const FRammsCurbStick S;
	FStickGate			  Gate;
	// From rest, one tick off the axis on the way forward: sideways, but only
	// for that tick.
	StickIs(*this, TEXT("on the way forward, off the axis"), Gate.Step(S, 0.10, 0.12, 0.0, StickDt), EStickIntent::Sideways);
	TestTrue(FString::Printf(TEXT("read so for one tick (%.3f s)"), Gate.GetSeconds()), Gate.GetSeconds() < S.ReleaseSeconds);
	StickIs(*this, TEXT("then forward"), Gate.Step(S, 0.6, 0.1, 0.0, StickDt), EStickIntent::Forward);
	TestTrue(FString::Printf(TEXT("forward afresh (%.3f s)"), Gate.GetSeconds()), FMath::IsNearlyEqual(Gate.GetSeconds(), StickDt));
	StickHold(Gate, S, 0.6, 0.1, 0.0, 0.5);
	TestTrue(FString::Printf(TEXT("forward for 0.5 s (%.3f)"), Gate.GetSeconds()), FMath::IsNearlyEqual(Gate.GetSeconds(), 0.5 + StickDt, 0.02));
	// Pushed to the side and held: counted from the tick it reads so.
	Gate.Reset();
	StickIs(*this, TEXT("pushed to the side"), StickHold(Gate, S, 0.0, 0.8, 0.0, 0.2), EStickIntent::Sideways);
	TestTrue(FString::Printf(TEXT("read so for 0.2 s (%.3f)"), Gate.GetSeconds()), FMath::IsNearlyEqual(Gate.GetSeconds(), 0.2, 0.02));
	TestEqual(TEXT("not back"), Gate.GetBackSeconds(), 0.0);
	// Swung round to the back: back, counted afresh.
	StickIs(*this, TEXT("swung back"), Gate.Step(S, -0.8, 0.0, 0.0, StickDt), EStickIntent::Back);
	TestTrue(FString::Printf(TEXT("back for a tick (%.3f)"), Gate.GetBackSeconds()), FMath::IsNearlyEqual(Gate.GetBackSeconds(), StickDt));
	return true;
}

namespace
{
	using RammsCurb::EStickAction;
	using RammsCurb::FStickDecision;
	using RammsCurb::FStickStage;

	const TCHAR* StickActionName(EStickAction Action)
	{
		switch (Action)
		{
			case EStickAction::Go:
				return TEXT("go");
			case EStickAction::Cancel:
				return TEXT("cancel");
			case EStickAction::FinishInPlace:
				return TEXT("finish in place");
			default:
				return TEXT("pause");
		}
	}

	/** The gate held at (Forward, Turn, Strafe) for Seconds, then what the stick does at
	 *  Stage; checked to be Want, its line to hold Says. */
	FStickDecision StickDecides(FAutomationTestBase& Test, const TCHAR* What, FStickGate& Gate, const FRammsCurbStick& S, double Forward, double Turn, double Strafe,
		double Seconds, const FStickStage& Stage, EStickAction Want, const TCHAR* Says = nullptr)
	{
		StickHold(Gate, S, Forward, Turn, Strafe, Seconds);
		const FStickDecision D = RammsCurb::DecideStick(S, Gate, Stage);
		Test.TestEqual(FString::Printf(TEXT("%s: %s (%s)"), What, StickActionName(D.Action), *D.Why), FString(StickActionName(D.Action)),
			FString(StickActionName(Want)));
		if (Says)
		{
			Test.TestTrue(FString::Printf(TEXT("%s: says '%s' (%s)"), What, Says, *D.Why), D.Why.Contains(Says));
		}
		return D;
	}

	FStickStage StickStageOf(bool bLegsMoved, bool bCrossing, bool bLevelled, bool bAtRest)
	{
		FStickStage Out;
		Out.bLegsMoved = bLegsMoved;
		Out.bCrossing = bCrossing;
		Out.bLevelled = bLevelled;
		Out.bAtRest = bAtRest;
		return Out;
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStickDecide,
	"Ramms.Curb.Stick.Decide",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbStickDecide::RunTest(const FString& Parameters)
{
	// What the stick does to a crossing at each stage: the controller acts on
	// exactly this (URammsCurbNegotiationController's StepStick).
	const FRammsCurbStick S;
	const FStickStage	  Rolling = StickStageOf(false, false, false, false);
	const FStickStage	  StandingStill = StickStageOf(false, false, false, true);
	const FStickStage	  Levelling = StickStageOf(true, false, false, true);
	const FStickStage	  LevelledMoving = StickStageOf(true, false, true, false);
	const FStickStage	  LevelledAtRest = StickStageOf(true, false, true, true);
	const FStickStage	  Crossing = StickStageOf(true, true, true, true);
	FStickGate			  Gate;

	// Held forward, at any stage: go, at the share pushed.
	for (const FStickStage& Stage : { Rolling, Levelling, LevelledAtRest, Crossing })
	{
		Gate.Reset();
		const FStickDecision D = StickDecides(*this, TEXT("held forward"), Gate, S, 0.6, 0.1, 0.0, 0.2, Stage, EStickAction::Go);
		TestEqual(TEXT("at the share pushed"), D.Scale, 0.6);
	}
	// Let go, at any stage: a pause, never a cancel, however long.
	for (const FStickStage& Stage : { Rolling, StandingStill, Levelling, LevelledAtRest, Crossing })
	{
		Gate.Reset();
		StickDecides(*this, TEXT("let go"), Gate, S, 0.0, 0.0, 0.0, 5.0, Stage, EStickAction::Pause, TEXT("push the stick forward"));
	}

	// Before any leg has moved: back or sideways cancels once read so for
	// ReleaseSeconds -- a tick or two passing through does not.
	Gate.Reset();
	StickDecides(*this, TEXT("rolling in, one tick off the axis"), Gate, S, 0.10, 0.12, 0.0, StickDt, Rolling, EStickAction::Pause,
		TEXT("held so, it cancels"));
	StickDecides(*this, TEXT("then forward"), Gate, S, 0.6, 0.0, 0.0, StickDt, Rolling, EStickAction::Go);
	Gate.Reset();
	StickDecides(*this, TEXT("rolling in, pushed to the side 0.1 s"), Gate, S, 0.0, 0.8, 0.0, 0.1, Rolling, EStickAction::Pause);
	StickDecides(*this, TEXT("rolling in, pushed to the side 0.2 s"), Gate, S, 0.0, 0.8, 0.0, 0.1, Rolling, EStickAction::Cancel,
		TEXT("pushed the stick to the side"));
	Gate.Reset();
	StickDecides(*this, TEXT("standing, pulled back 0.1 s"), Gate, S, -0.8, 0.0, 0.0, 0.1, StandingStill, EStickAction::Pause,
		TEXT("pulled back"));
	StickDecides(*this, TEXT("standing, pulled back 0.2 s"), Gate, S, -0.8, 0.0, 0.0, 0.1, StandingStill, EStickAction::Cancel,
		TEXT("pulled the stick back"));
	// From held forward, back is read only once it has been out of forward for
	// ReleaseSeconds, and then needs ReleaseSeconds more to cancel.
	Gate.Reset();
	StickHold(Gate, S, 0.8, 0.0, 0.0, 0.2);
	StickDecides(*this, TEXT("rolling in, forward to back 0.2 s"), Gate, S, -0.8, 0.0, 0.0, 0.2, Rolling, EStickAction::Pause);
	StickDecides(*this, TEXT("rolling in, forward to back 0.35 s"), Gate, S, -0.8, 0.0, 0.0, 0.15, Rolling, EStickAction::Cancel);

	// Levelling in, or levelled and still rolling to rest: back pauses, and
	// waits to cancel for both.
	Gate.Reset();
	StickDecides(*this, TEXT("levelling, held back 3 s"), Gate, S, -0.8, 0.0, 0.0, 3.0, Levelling, EStickAction::Pause, TEXT("once the robot is at rest"));
	Gate.Reset();
	StickDecides(*this, TEXT("levelled, rolling, held back 3 s"), Gate, S, -0.8, 0.0, 0.0, 3.0, LevelledMoving, EStickAction::Pause,
		TEXT("once the robot is at rest"));
	// Levelled in at rest: back held CancelHoldSeconds cancels, counting down.
	Gate.Reset();
	StickDecides(*this, TEXT("levelled at rest, held back 0.4 s"), Gate, S, -0.8, 0.0, 0.0, 0.4, LevelledAtRest, EStickAction::Pause,
		TEXT("more to cancel"));
	StickDecides(*this, TEXT("levelled at rest, held back 0.8 s"), Gate, S, -0.8, 0.0, 0.0, 0.4, LevelledAtRest, EStickAction::Cancel,
		TEXT("held the stick back"));
	// Sideways once a leg has moved: a pause, never a cancel.
	Gate.Reset();
	StickDecides(*this, TEXT("levelled, pushed to the side 3 s"), Gate, S, 0.0, 0.8, 0.0, 3.0, LevelledAtRest, EStickAction::Pause,
		TEXT("steering is locked"));

	// Crossing: back and sideways pause, never cancel, however long.
	Gate.Reset();
	StickDecides(*this, TEXT("crossing, held back 5 s"), Gate, S, -0.8, 0.0, 0.0, 5.0, Crossing, EStickAction::Pause, TEXT("cannot back out"));
	Gate.Reset();
	StickDecides(*this, TEXT("crossing, pushed to the side 5 s"), Gate, S, 0.0, 0.8, 0.0, 5.0, Crossing, EStickAction::Pause, TEXT("steering is locked"));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStickFinishInPlace,
	"Ramms.Curb.Stick.FinishInPlace",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbStickFinishInPlace::RunTest(const FString& Parameters)
{
	// Once nothing left of a crossing needs the robot to travel -- the rear
	// wheels can touch down on the far surface -- the stick let go, pushed to
	// the side or pulled back no longer pauses it for good: out of forward for
	// FinishInPlaceConfirmSeconds (a pause meanwhile, from which forward goes
	// on), the crossing finishes where the robot stands, and once it does, it
	// holds to that whatever the stick does. Short of there, nothing changes.
	const FRammsCurbStick S;
	const double		  Confirm = S.FinishInPlaceConfirmSeconds;
	TestTrue(FString::Printf(TEXT("a confirm longer than a waver (%.2f s, ReleaseSeconds %.2f)"), Confirm, S.ReleaseSeconds), Confirm > S.ReleaseSeconds);
	const TCHAR* const Confirming = TEXT("finishes where the robot stands");
	const FStickStage  Crossing = StickStageOf(true, true, true, true);
	FStickStage		   PastAtRest = Crossing;
	PastAtRest.bRearCanLand = true;
	FStickStage PastMoving = PastAtRest;
	PastMoving.bAtRest = false;
	FStickStage Finishing = PastAtRest;
	Finishing.bFinishingInPlace = true;
	// Finishing in place, whether or not the stage still says the rear wheels
	// can land: the decision is held, not taken afresh.
	FStickStage Latched = Crossing;
	Latched.bFinishingInPlace = true;
	const TCHAR* const Settling = TEXT("settling into the driving pose");
	FStickGate		   Gate;

	// Short of the point: as before -- let go, sideways or back pause, forward
	// goes.
	Gate.Reset();
	StickDecides(*this, TEXT("short of it, let go 5 s"), Gate, S, 0.0, 0.0, 0.0, 5.0, Crossing, EStickAction::Pause, TEXT("push the stick forward"));
	Gate.Reset();
	StickDecides(*this, TEXT("short of it, to the side 5 s"), Gate, S, 0.0, 0.8, 0.0, 5.0, Crossing, EStickAction::Pause, TEXT("steering is locked"));
	Gate.Reset();
	StickDecides(*this, TEXT("short of it, back 5 s"), Gate, S, -0.8, 0.0, 0.0, 5.0, Crossing, EStickAction::Pause, TEXT("cannot back out"));
	Gate.Reset();
	StickDecides(*this, TEXT("short of it, forward"), Gate, S, 0.6, 0.0, 0.0, 0.2, Crossing, EStickAction::Go);

	// Past it, held forward: on along the plan at the share pushed, and through
	// a waver shorter than ReleaseSeconds.
	Gate.Reset();
	const FStickDecision Pushed = StickDecides(*this, TEXT("past it, forward"), Gate, S, 0.6, 0.0, 0.0, 0.2, PastMoving, EStickAction::Go);
	TestEqual(TEXT("past it, forward: at the share pushed"), Pushed.Scale, 0.6);
	StickDecides(*this, TEXT("past it, a 0.1 s waver"), Gate, S, 0.0, 0.0, 0.0, 0.1, PastMoving, EStickAction::Go);
	// Let go for longer: read so once ReleaseSeconds have passed -- a pause, the
	// confirm under way -- and finishing in place once it has read so for the
	// confirm.
	StickDecides(*this, TEXT("past it, let go from forward 0.2 s"), Gate, S, 0.0, 0.0, 0.0, 0.1, PastMoving, EStickAction::Pause, Confirming);
	StickDecides(*this, TEXT("past it, let go a confirm on"), Gate, S, 0.0, 0.0, 0.0, Confirm + 0.1, PastMoving, EStickAction::FinishInPlace, Settling);

	// Past it, let go, to the side (turning or strafing) or back, on the move or
	// at rest: a pause until read so for the confirm, then finishing in place --
	// no wait to cancel; and back held as long as you like finishes too.
	for (const FStickStage& Stage : { PastMoving, PastAtRest })
	{
		Gate.Reset();
		StickDecides(*this, TEXT("past it, let go"), Gate, S, 0.0, 0.0, 0.0, StickDt, Stage, EStickAction::Pause, Confirming);
		StickDecides(*this, TEXT("past it, let go on"), Gate, S, 0.0, 0.0, 0.0, Confirm + 0.1, Stage, EStickAction::FinishInPlace, Settling);
		Gate.Reset();
		StickDecides(*this, TEXT("past it, turned to the side"), Gate, S, 0.0, 0.8, 0.0, StickDt, Stage, EStickAction::Pause, Confirming);
		StickDecides(*this, TEXT("past it, turned to the side on"), Gate, S, 0.0, 0.8, 0.0, Confirm + 0.1, Stage, EStickAction::FinishInPlace, Settling);
		Gate.Reset();
		StickDecides(*this, TEXT("past it, strafed to the side"), Gate, S, 0.0, 0.0, -0.8, StickDt, Stage, EStickAction::Pause, Confirming);
		StickDecides(*this, TEXT("past it, strafed to the side on"), Gate, S, 0.0, 0.0, -0.8, Confirm + 0.1, Stage, EStickAction::FinishInPlace, Settling);
		Gate.Reset();
		StickDecides(*this, TEXT("past it, back"), Gate, S, -0.8, 0.0, 0.0, StickDt, Stage, EStickAction::Pause, Confirming);
		Gate.Reset();
		StickDecides(*this, TEXT("past it, back 5 s"), Gate, S, -0.8, 0.0, 0.0, 5.0, Stage, EStickAction::FinishInPlace, Settling);
	}

	// The confirm counts the stick out of forward, whichever way: let go, then
	// to the side, then back, together the confirm, finishes in place.
	Gate.Reset();
	StickDecides(*this, TEXT("past it, let go a third of the confirm"), Gate, S, 0.0, 0.0, 0.0, Confirm / 3.0, PastAtRest, EStickAction::Pause, Confirming);
	StickDecides(*this, TEXT("then to the side a third"), Gate, S, 0.0, 0.8, 0.0, Confirm / 3.0, PastAtRest, EStickAction::Pause, Confirming);
	StickDecides(*this, TEXT("then back the rest"), Gate, S, -0.8, 0.0, 0.0, Confirm / 3.0 + 0.1, PastAtRest, EStickAction::FinishInPlace, Settling);

	// A waver past ReleaseSeconds but short of the confirm, then forward again:
	// a pause, then on along the plan -- nothing taken that cannot be undone.
	Gate.Reset();
	StickHold(Gate, S, 0.8, 0.0, 0.0, 0.5);
	StickDecides(*this, TEXT("past it, let go a moment"), Gate, S, 0.0, 0.0, 0.0, S.ReleaseSeconds + 0.1, PastMoving, EStickAction::Pause, Confirming);
	const FStickDecision Resumed = StickDecides(*this, TEXT("past it, forward again"), Gate, S, 0.8, 0.0, 0.0, StickDt, PastMoving, EStickAction::Go);
	TestEqual(TEXT("past it, forward again: at the share pushed"), Resumed.Scale, 0.8);
	StickDecides(*this, TEXT("past it, forward on"), Gate, S, 0.8, 0.0, 0.0, Confirm + 0.1, PastMoving, EStickAction::Go);

	// Paused short of it, the stick still let go, then past it (rolled on into
	// it while braking): no longer paused -- finishing in place at once, the
	// stick out of forward far longer than the confirm.
	Gate.Reset();
	StickDecides(*this, TEXT("paused short of it"), Gate, S, 0.0, 0.0, 0.0, 1.0, Crossing, EStickAction::Pause);
	StickDecides(*this, TEXT("still let go, now past it"), Gate, S, 0.0, 0.0, 0.0, StickDt, PastAtRest, EStickAction::FinishInPlace, Settling);

	// Finishing in place: held to the end, whatever the stick -- forward again
	// included, the drive mode takes the stick once it is done.
	for (const FStickStage& Stage : { Finishing, Latched })
	{
		Gate.Reset();
		StickDecides(*this, TEXT("finishing, let go"), Gate, S, 0.0, 0.0, 0.0, 1.0, Stage, EStickAction::FinishInPlace, Settling);
		const FStickDecision Again = StickDecides(*this, TEXT("finishing, forward again"), Gate, S, 0.8, 0.0, 0.0, 1.0, Stage, EStickAction::FinishInPlace,
			TEXT("the stick takes over"));
		TestEqual(TEXT("finishing, forward again: no share of the plan's speed"), Again.Scale, 0.0);
		StickDecides(*this, TEXT("finishing, to the side"), Gate, S, 0.0, 0.8, 0.0, 1.0, Stage, EStickAction::FinishInPlace, Settling);
		StickDecides(*this, TEXT("finishing, back 5 s"), Gate, S, -0.8, 0.0, 0.0, 5.0, Stage, EStickAction::FinishInPlace, Settling);
	}
	return true;
}

namespace
{
	/** A continuous drive control paired with Pair (none: unpaired). */
	FRammsControlAxis HandBackAxis(FName Id, FName Pair)
	{
		FRammsControlAxis Axis;
		Axis.Id = Id;
		Axis.Kind = ERammsControlKind::Continuous;
		Axis.PairedAxis = Pair;
		return Axis;
	}

	/** What a gamepad's drive stick (X turn, Y forward, Z turn: the drive map's
	 *  binding) feeds on Surface, as "x|y|z". */
	FString HandBackStickOn(const FRammsControlSurface& Surface)
	{
		TArray<FName> Ids[3] = { { RammsControlIds::Drive::Turn() }, { RammsControlIds::Drive::Forward() }, { RammsControlIds::Drive::Turn() } };
		RammsControlInput::ResolveStick(Surface, RammsControlIds::Drive::Forward(), Ids);
		TArray<FString> Parts;
		for (const TArray<FName>& Axis : Ids)
		{
			TArray<FString> Names;
			for (const FName& Id : Axis)
			{
				Names.Add(Id.ToString());
			}
			Parts.Add(FString::Join(Names, TEXT(",")));
		}
		return FString::Join(Parts, TEXT("|"));
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStickHandBack,
	"Ramms.Curb.Stick.HandBack",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbStickHandBack::RunTest(const FString& Parameters)
{
	using RammsCurb::HandBackNeedsCentre;
	using RammsCurb::HandBackWaits;
	const FRammsCurbStick S;
	FStickGate			  Gate;

	// Done with the stick pulled back: not handed back -- the drive mode would
	// reverse the robot over the curb it has just crossed -- but held, saying
	// so, until the stick leaves back.
	StickHold(Gate, S, -0.8, 0.0, 0.0, 1.0);
	TestTrue(TEXT("pulled back: the hand-back waits"), HandBackWaits(Gate));
	TestTrue(TEXT("pulled back: its line"), FString(RammsCurb::HandBackWaitWhy()).Contains(TEXT("release the stick to drive")));
	StickHold(Gate, S, -0.8, 0.3, 0.0, 1.0);
	TestTrue(TEXT("back and a little to the side (still back): waits"), HandBackWaits(Gate));
	// Let go, to the side, back and well to the side, forward: handed back.
	struct FHeld
	{
		const TCHAR* What;
		double		 Forward, Turn, Strafe;
	};
	for (const FHeld& H : { FHeld{ TEXT("let go"), 0.0, 0.0, 0.0 }, FHeld{ TEXT("turned"), 0.0, 0.8, 0.0 }, FHeld{ TEXT("strafed"), 0.0, 0.0, -0.8 },
			 FHeld{ TEXT("back and to the side"), -0.5, 0.8, 0.0 }, FHeld{ TEXT("forward"), 0.8, 0.0, 0.0 } })
	{
		Gate.Reset();
		StickHold(Gate, S, H.Forward, H.Turn, H.Strafe, 0.5);
		TestFalse(FString::Printf(TEXT("%s: handed back (reads %s)"), H.What, RammsCurb::StickIntentName(Gate.GetIntent())), HandBackWaits(Gate));
	}
	// From back, let go: the wait over.
	Gate.Reset();
	StickHold(Gate, S, -0.8, 0.0, 0.0, 1.0);
	StickHold(Gate, S, 0.0, 0.0, 0.0, StickDt);
	TestFalse(TEXT("back, then let go: handed back"), HandBackWaits(Gate));

	// Handed back with the stick at all backward of centre (back and to the side),
	// drive.forward must come back to centre first; centred or forward, it need not.
	TestTrue(TEXT("back and to the side (-0.5): back to centre first"), HandBackNeedsCentre(-0.5));
	TestTrue(TEXT("a little back (-0.2): back to centre first"), HandBackNeedsCentre(-0.2));
	TestFalse(TEXT("within the centre band (-0.05): no"), HandBackNeedsCentre(-0.05));
	TestFalse(TEXT("centred: no"), HandBackNeedsCentre(0.0));
	TestFalse(TEXT("forward: no, it drives on"), HandBackNeedsCentre(0.8));

	// The re-centre, as a gamepad stick's forward axis takes it
	// (URammsControlInputComponent): asked while the stick is held, nothing
	// passes to drive.forward until it has come back within the centre band;
	// then the stick drives again. Asked while not held, nothing waits.
	{
		bool  bAwait = false;
		int32 Seen = 0;
		TestTrue(TEXT("asked while held back: waits"), RammsControlInput::TakeCentreRequest(1, Seen, true, bAwait) && bAwait);
		TestEqual(TEXT("asked: seen"), Seen, 1);
		TestEqual(TEXT("held back: nothing to drive.forward"), RammsControlInput::StepCentre(bAwait, -0.8f), 0.0f);
		TestEqual(TEXT("back and to the side: nothing"), RammsControlInput::StepCentre(bAwait, -0.5f), 0.0f);
		TestTrue(TEXT("still waiting"), bAwait);
		TestEqual(TEXT("back to centre: nothing yet"), RammsControlInput::StepCentre(bAwait, 0.05f), 0.0f);
		TestFalse(TEXT("back to centre: the wait over"), bAwait);
		TestEqual(TEXT("then pushed: the stick drives"), RammsControlInput::StepCentre(bAwait, -0.6f), -0.6f);
		TestFalse(TEXT("the same ask again: nothing new"), RammsControlInput::TakeCentreRequest(1, Seen, true, bAwait) || bAwait);
		TestFalse(TEXT("asked while not held: only noted"), RammsControlInput::TakeCentreRequest(2, Seen, false, bAwait) || bAwait);
		TestEqual(TEXT("asked while not held: seen"), Seen, 2);
	}

	// The stick's X on the crossing's surface (StickPairs) as on the surface of
	// the mode it took the robot from: the same control, so that a stick held
	// to the side through the take-over and the hand-back never waits for
	// centre, and drives on into the mode at Done.
	const FName Forward = RammsControlIds::Drive::Forward();
	const FName Turn = RammsControlIds::Drive::Turn();
	const FName Strafe = RammsControlIds::Drive::Strafe();
	auto		CurbSurface = [&](FName SideAxis) {
		   const RammsCurb::FStickPairs Pairs = RammsCurb::StickPairs(SideAxis);
		   FRammsControlSurface			Out;
		   Out.Add(HandBackAxis(Forward, Pairs.Forward));
		   Out.Add(HandBackAxis(Turn, Pairs.Turn));
		   Out.Add(HandBackAxis(Strafe, Pairs.Strafe));
		   return Out;
	};
	FRammsControlSurface Differential;
	Differential.Add(HandBackAxis(Forward, Turn));
	Differential.Add(HandBackAxis(Turn, Forward));
	FRammsControlSurface Holonomic;
	Holonomic.Add(HandBackAxis(Forward, Strafe));
	Holonomic.Add(HandBackAxis(Strafe, Forward));
	Holonomic.Add(HandBackAxis(Turn, NAME_None));
	const FString OnDifferential = HandBackStickOn(Differential), OnHolonomic = HandBackStickOn(Holonomic);
	const FString CurbFromDifferential = HandBackStickOn(CurbSurface(Turn)), CurbFromHolonomic = HandBackStickOn(CurbSurface(Strafe));
	TestTrue(FString::Printf(TEXT("from a differential drive: X turns there (%s)"), *OnDifferential), OnDifferential.StartsWith(TEXT("drive.turn|drive.forward")));
	TestTrue(FString::Printf(TEXT("from a holonomic drive: X strafes there (%s)"), *OnHolonomic), OnHolonomic.StartsWith(TEXT("drive.strafe|drive.forward")));
	TestEqual(TEXT("a crossing from a differential drive: X turns in it too"), CurbFromDifferential.Left(CurbFromDifferential.Find(TEXT("|"))),
		FString(TEXT("drive.turn")));
	TestEqual(TEXT("a crossing from a holonomic drive: X strafes in it too"), CurbFromHolonomic.Left(CurbFromHolonomic.Find(TEXT("|"))),
		FString(TEXT("drive.strafe")));
	TestEqual(TEXT("a crossing from nothing paired: X turns"), HandBackStickOn(CurbSurface(NAME_None)), CurbFromDifferential);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbStickHoldTimeout,
	"Ramms.Curb.Stick.HoldTimeout",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbStickHoldTimeout::RunTest(const FString& Parameters)
{
	// A hold's time is kept, not restarted, through a pause: letting go and
	// pushing again cannot keep a stuck crossing from its hold timeout.
	double Seconds = 0.0;
	auto   Run = [&Seconds](bool bHolding, bool bPaused, double For) {
		  for (int32 i = FMath::RoundToInt(For / StickDt); i > 0; --i)
		  {
			  Seconds = RammsCurb::StepHoldSeconds(bHolding, bPaused, Seconds, StickDt);
		  }
	};
	Run(true, false, 3.0);
	TestTrue(FString::Printf(TEXT("held 3 s (%.3f)"), Seconds), FMath::IsNearlyEqual(Seconds, 3.0, 0.02));
	Run(false, true, 10.0);
	TestTrue(FString::Printf(TEXT("paused 10 s: still 3 (%.3f)"), Seconds), FMath::IsNearlyEqual(Seconds, 3.0, 0.02));
	Run(true, false, 2.5);
	TestTrue(FString::Printf(TEXT("held again 2.5 s: 5.5, past a 5 s timeout (%.3f)"), Seconds), Seconds > 5.0);
	Run(false, false, StickDt);
	TestEqual(TEXT("the hold over: 0"), Seconds, 0.0);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
