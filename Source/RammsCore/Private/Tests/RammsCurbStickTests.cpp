// Tests for the rider's stick in a curb crossing (Private/Curb/RammsCurbStick).
//
// The stick read each tick on made-up input: forward only once pushed far
// enough within the forward sector, then forward through a brief waver,
// let go, sideways and back read at once out of forward, and how long it has
// been held back.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Curb/RammsCurbStick.h"
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

#endif // WITH_DEV_AUTOMATION_TESTS
