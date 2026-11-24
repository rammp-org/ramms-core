// Tests for the level assist's arbiter (Private/Level/RammsLevelArbiter):
// who has the legs, and the one thing to be done on the tick that changes.
//
// The switch on and off; a curb controller's yield folding the offsets once;
// its hand-back with and without a plane; the cold reset and the start after
// a fault; a mode that asks for a gentle release; a missing attitude; a
// divergence latching until switched off and on; a mode that took the
// legs without saying so; and a start that waits for a stance being changed.
// There are no numbers in the arbiter, so there are
// none here: every test is a sequence of made-up inputs.
//
// Beside each test, the change to the arbiter it is there to catch.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Level/RammsLevelArbiter.h"

#if WITH_DEV_AUTOMATION_TESTS

// Named, not anonymous: the other level tests share a unity file and may
// well call their own helpers the same.
namespace RammsLevelArbiterTests
{
	using namespace RammsLevel;

	/** Switched on, in a mode it may level in, with an attitude. */
	FArbiterInputs Ready()
	{
		FArbiterInputs In;
		In.bEnable = true;
		In.Mode = EMode::Allowed;
		In.bAttitudeValid = true;
		return In;
	}

	/** One tick, and what it must answer. */
	FArbiterStep Expect(FAutomationTestBase& Test, const TCHAR* What, FArbiter& Arbiter, const FArbiterInputs& In, EState State, EAction Action)
	{
		const FArbiterStep Step = Arbiter.Tick(In);
		Test.TestEqual(FString::Printf(TEXT("%s: state"), What), static_cast<int32>(Step.State), static_cast<int32>(State));
		Test.TestEqual(FString::Printf(TEXT("%s: action"), What), static_cast<int32>(Step.Action), static_cast<int32>(Action));
		Test.TestEqual(FString::Printf(TEXT("%s: GetState agrees"), What), static_cast<int32>(Arbiter.GetState()), static_cast<int32>(Step.State));
		return Step;
	}

	/** A fresh arbiter taken to Active. */
	FArbiter MakeActive()
	{
		FArbiter Arbiter;
		Arbiter.Tick(Ready());
		return Arbiter;
	}

	/** ... to Holding (the attitude lost). */
	FArbiter MakeHolding()
	{
		FArbiter	   Arbiter = MakeActive();
		FArbiterInputs In = Ready();
		In.bAttitudeValid = false;
		Arbiter.Tick(In);
		return Arbiter;
	}

	/** ... to Releasing (switched off, the correction not yet out). */
	FArbiter MakeReleasing()
	{
		FArbiter	   Arbiter = MakeActive();
		FArbiterInputs In = Ready();
		In.bEnable = false;
		Arbiter.Tick(In);
		return Arbiter;
	}

	/** ... to Yielded (a taker announced itself and holds the legs). */
	FArbiter MakeYielded()
	{
		FArbiter	   Arbiter = MakeActive();
		FArbiterInputs In = Ready();
		In.Mode = EMode::Taken;
		In.bYield = true;
		Arbiter.Tick(In);
		return Arbiter;
	}

	/** ... to Faulted, still ramping out. */
	FArbiter MakeFaulted()
	{
		FArbiter	   Arbiter = MakeActive();
		FArbiterInputs In = Ready();
		In.bDiverged = true;
		Arbiter.Tick(In);
		return Arbiter;
	}
} // namespace RammsLevelArbiterTests

// Catches: switching off going straight to Off (no release), a release that
// ends before the correction is out, or one that ends without the Clear.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelArbiterEnableDisable,
	"Ramms.Level.Arbiter.EnableDisable",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelArbiterEnableDisable::RunTest(const FString& Parameters)
{
	using namespace RammsLevelArbiterTests;

	FArbiter Arbiter;
	TestFalse(TEXT("Off: no claim"), Arbiter.WantsClaim());
	TestFalse(TEXT("Off: no offsets"), Arbiter.HasOffsets());

	FArbiterInputs Off = Ready();
	Off.bEnable = false;
	Expect(*this, TEXT("off stays off"), Arbiter, Off, EState::Off, EAction::None);

	Expect(*this, TEXT("switched on"), Arbiter, Ready(), EState::Active, EAction::StartFromAttitude);
	TestTrue(TEXT("Active: claims"), Arbiter.WantsClaim());
	TestTrue(TEXT("Active: has offsets"), Arbiter.HasOffsets());
	// The start is asked for once, not on every tick.
	Expect(*this, TEXT("levelling on"), Arbiter, Ready(), EState::Active, EAction::None);

	Expect(*this, TEXT("switched off"), Arbiter, Off, EState::Releasing, EAction::BeginRelease);
	Expect(*this, TEXT("still releasing"), Arbiter, Off, EState::Releasing, EAction::None);
	TestTrue(TEXT("Releasing: still claims"), Arbiter.WantsClaim());
	TestTrue(TEXT("Releasing: has offsets"), Arbiter.HasOffsets());

	// Wanted again part way: it goes on from where it was, with no restart.
	Expect(*this, TEXT("switched back on mid-release"), Arbiter, Ready(), EState::Active, EAction::None);
	Expect(*this, TEXT("switched off again"), Arbiter, Off, EState::Releasing, EAction::BeginRelease);

	Off.bCorrectionZero = true;
	Expect(*this, TEXT("release done"), Arbiter, Off, EState::Off, EAction::Clear);
	TestFalse(TEXT("Off again: no claim"), Arbiter.WantsClaim());
	TestFalse(TEXT("Off again: no offsets"), Arbiter.HasOffsets());
	Expect(*this, TEXT("off, nothing more"), Arbiter, Off, EState::Off, EAction::None);

	// A correction that happens to read zero while levelling ends nothing.
	FArbiterInputs Zero = Ready();
	Zero.bCorrectionZero = true;
	Arbiter.Tick(Zero);
	Expect(*this, TEXT("zero correction while on"), Arbiter, Zero, EState::Active, EAction::None);
	return true;
}

// Catches: a second yield (or one while off, yielded, or waiting) folding
// again, and a yield that forgets the fold from Holding, Releasing or a fault.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelArbiterYieldFoldsOnce,
	"Ramms.Level.Arbiter.YieldFoldsOnce",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelArbiterYieldFoldsOnce::RunTest(const FString& Parameters)
{
	using namespace RammsLevelArbiterTests;

	FArbiterInputs Announced = Ready();
	Announced.Mode = EMode::Taken;
	Announced.bYield = true;
	FArbiterInputs Taken = Ready();
	Taken.Mode = EMode::Taken;

	{
		FArbiter		   Arbiter = MakeActive();
		const FArbiterStep Step = Expect(*this, TEXT("yield from Active"), Arbiter, Announced, EState::Yielded, EAction::Fold);
		TestFalse(TEXT("an announced yield is not unexpected"), Step.bUnexpected);
		TestFalse(TEXT("Yielded: no offsets"), Arbiter.HasOffsets());
		TestTrue(TEXT("Yielded: still claims"), Arbiter.WantsClaim());
		Expect(*this, TEXT("a second yield folds nothing"), Arbiter, Announced, EState::Yielded, EAction::None);
		Expect(*this, TEXT("the taker's mode, tick after tick"), Arbiter, Taken, EState::Yielded, EAction::None);
	}
	{
		// The yield is announced before the mode changes: it still folds.
		FArbiter	   Arbiter = MakeActive();
		FArbiterInputs Early = Ready();
		Early.bYield = true;
		Expect(*this, TEXT("yield ahead of the mode"), Arbiter, Early, EState::Yielded, EAction::Fold);
	}
	{
		FArbiter Arbiter = MakeHolding();
		Expect(*this, TEXT("yield from Holding"), Arbiter, Announced, EState::Yielded, EAction::Fold);
	}
	{
		// Releasing because switched off: the fold still happens, then Off.
		FArbiter	   Arbiter = MakeReleasing();
		FArbiterInputs OffYield = Announced;
		OffYield.bEnable = false;
		Expect(*this, TEXT("yield from Releasing, switched off"), Arbiter, OffYield, EState::Off, EAction::Fold);
		Expect(*this, TEXT("yield while Off"), Arbiter, OffYield, EState::Off, EAction::None);
	}
	{
		// A fault part way through its ramp: folded, and it stays a fault.
		FArbiter Arbiter = MakeFaulted();
		Expect(*this, TEXT("yield from a ramping fault"), Arbiter, Announced, EState::Faulted, EAction::Fold);
		TestTrue(TEXT("the fault is latched by the fold"), Arbiter.IsFaultLatched());
		TestFalse(TEXT("... and has no offsets left"), Arbiter.HasOffsets());
		Expect(*this, TEXT("yield to a latched fault"), Arbiter, Announced, EState::Faulted, EAction::None);
	}
	{
		// Waiting for an attitude: nothing to fold, but the legs are taken.
		FArbiter	   Arbiter;
		FArbiterInputs Blind = Ready();
		Blind.bAttitudeValid = false;
		Expect(*this, TEXT("on with no attitude"), Arbiter, Blind, EState::Suspended, EAction::None);
		Blind.bYield = true;
		Expect(*this, TEXT("yield from Suspended"), Arbiter, Blind, EState::Yielded, EAction::None);
	}
	return true;
}

// Catches: a hand-back that starts from nothing instead of adopting the
// plane, one that reads the (still the taker's) mode, and one acted on
// outside Yielded.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelArbiterResumeAdopts,
	"Ramms.Level.Arbiter.ResumeAdopts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelArbiterResumeAdopts::RunTest(const FString& Parameters)
{
	using namespace RammsLevelArbiterTests;

	// The taker's mode is still the active one on the tick it hands back.
	FArbiterInputs Resume = Ready();
	Resume.Mode = EMode::Taken;
	Resume.bResume = true;
	Resume.bResumeHasPlane = true;

	{
		FArbiter Arbiter = MakeYielded();
		Expect(*this, TEXT("hand-back with a plane"), Arbiter, Resume, EState::Active, EAction::Adopt);
		Expect(*this, TEXT("then levelling on"), Arbiter, Ready(), EState::Active, EAction::None);
	}
	{
		// The plane is out of the baselines either way: it is taken up and
		// held until there is an attitude, then picked up without a jump.
		FArbiter	   Arbiter = MakeYielded();
		FArbiterInputs Blind = Resume;
		Blind.bAttitudeValid = false;
		Expect(*this, TEXT("hand-back with a plane, no attitude"), Arbiter, Blind, EState::Holding, EAction::Adopt);
		TestTrue(TEXT("the adopted plane counts as offsets"), Arbiter.HasOffsets());
		Expect(*this, TEXT("the attitude comes"), Arbiter, Ready(), EState::Active, EAction::ColdResume);
	}
	{
		// A take-over refused in the same call: folded and adopted back,
		// and it is the adoption that is reported.
		FArbiter	   Arbiter = MakeActive();
		FArbiterInputs Both = Resume;
		Both.bYield = true;
		const FArbiterStep Step = Expect(*this, TEXT("yield and hand-back together"), Arbiter, Both, EState::Active, EAction::Adopt);
		TestFalse(TEXT("... which is not unexpected"), Step.bUnexpected);
	}
	{
		// Switched off during the crossing: the plane is not taken up.
		FArbiter	   Arbiter = MakeYielded();
		FArbiterInputs Off = Resume;
		Off.bEnable = false;
		Expect(*this, TEXT("hand-back when switched off"), Arbiter, Off, EState::Off, EAction::Clear);
	}
	{
		// A hand-back nobody was waiting for changes nothing.
		FArbiter	   Arbiter = MakeActive();
		FArbiterInputs Stray = Ready();
		Stray.bResume = true;
		Stray.bResumeHasPlane = true;
		Expect(*this, TEXT("hand-back while Active"), Arbiter, Stray, EState::Active, EAction::None);

		FArbiter Idle;
		Stray.bEnable = false;
		Expect(*this, TEXT("hand-back while Off"), Idle, Stray, EState::Off, EAction::None);
	}
	return true;
}

// Catches: a hand-back with no plane adopting one anyway (Adopt of nothing),
// or leaving the arbiter Yielded for good with no taker.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelArbiterResumeWithoutStanceRestores,
	"Ramms.Level.Arbiter.ResumeWithoutStanceRestores",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelArbiterResumeWithoutStanceRestores::RunTest(const FString& Parameters)
{
	using namespace RammsLevelArbiterTests;

	FArbiterInputs Resume = Ready();
	Resume.Mode = EMode::Taken;
	Resume.bResume = true;

	{
		FArbiter Arbiter = MakeYielded();
		Expect(*this, TEXT("hand-back, no plane"), Arbiter, Resume, EState::Active, EAction::StartFromAttitude);
	}
	{
		FArbiter	   Arbiter = MakeYielded();
		FArbiterInputs Blind = Resume;
		Blind.bAttitudeValid = false;
		Expect(*this, TEXT("hand-back, no plane, no attitude"), Arbiter, Blind, EState::Suspended, EAction::None);
		TestFalse(TEXT("Suspended: no offsets"), Arbiter.HasOffsets());
		Expect(*this, TEXT("the attitude comes"), Arbiter, Ready(), EState::Active, EAction::StartFromAttitude);
	}
	{
		// The crossing was aborted, or the curb mode left by hand: no
		// hand-back ever comes, and an allowed mode starts it again cold.
		FArbiter Arbiter = MakeYielded();
		Expect(*this, TEXT("an allowed mode with no hand-back"), Arbiter, Ready(), EState::Active, EAction::StartFromAttitude);
	}
	{
		// ... and switching off while yielded needs no release.
		FArbiter	   Arbiter = MakeYielded();
		FArbiterInputs Off = Ready();
		Off.bEnable = false;
		Off.Mode = EMode::Taken;
		Expect(*this, TEXT("switched off while yielded"), Arbiter, Off, EState::Off, EAction::None);
	}
	{
		// Refused in the same call with nothing to take back and nothing to
		// level against: the offsets did go into the baselines, so the fold
		// is what must be told.
		FArbiter	   Arbiter = MakeActive();
		FArbiterInputs Both = Resume;
		Both.bYield = true;
		Both.bAttitudeValid = false;
		Expect(*this, TEXT("yield and plane-less hand-back, no attitude"), Arbiter, Both, EState::Suspended, EAction::Fold);

		FArbiter Again = MakeActive();
		Both.bAttitudeValid = true;
		Expect(*this, TEXT("yield and plane-less hand-back"), Again, Both, EState::Active, EAction::StartFromAttitude);
	}
	return true;
}

// Catches: a cold reset that ramps or folds instead of clearing, one that
// lifts a fault, and a latched fault that restarts without the switch going
// off first.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelArbiterColdResumeAfterFault,
	"Ramms.Level.Arbiter.ColdResumeAfterFault",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelArbiterColdResumeAfterFault::RunTest(const FString& Parameters)
{
	using namespace RammsLevelArbiterTests;

	FArbiterInputs Cold = Ready();
	Cold.bColdReset = true;

	{
		// Teleported while levelling: what was applied is gone, start anew.
		FArbiter Arbiter = MakeActive();
		Expect(*this, TEXT("cold reset while Active"), Arbiter, Cold, EState::Active, EAction::StartFromAttitude);
		FArbiter Held = MakeHolding();
		Expect(*this, TEXT("cold reset while Holding, attitude back"), Held, Cold, EState::Active, EAction::StartFromAttitude);
	}
	{
		FArbiter	   Arbiter = MakeHolding();
		FArbiterInputs Blind = Cold;
		Blind.bAttitudeValid = false;
		Expect(*this, TEXT("cold reset with no attitude"), Arbiter, Blind, EState::Suspended, EAction::Clear);
		TestFalse(TEXT("... leaves no offsets"), Arbiter.HasOffsets());
	}
	{
		// No release is run across a reset: straight to Off, cleared.
		FArbiter	   Arbiter = MakeReleasing();
		FArbiterInputs Off = Cold;
		Off.bEnable = false;
		Expect(*this, TEXT("cold reset while releasing to off"), Arbiter, Off, EState::Off, EAction::Clear);
	}
	{
		FArbiter	   Arbiter = MakeActive();
		FArbiterInputs Taken = Cold;
		Taken.Mode = EMode::Taken;
		const FArbiterStep Step = Expect(*this, TEXT("cold reset under a taker"), Arbiter, Taken, EState::Yielded, EAction::Clear);
		TestFalse(TEXT("... is a clear, not an unexpected fold"), Step.bUnexpected);

		// The reset comes before a yield on the same tick: nothing is folded.
		FArbiter Other = MakeActive();
		Taken.bYield = true;
		Expect(*this, TEXT("cold reset with a yield"), Other, Taken, EState::Yielded, EAction::Clear);
	}
	{
		// A fault is not lifted by a reset: it is latched at once instead.
		FArbiter Arbiter = MakeFaulted();
		TestFalse(TEXT("a fresh fault is not latched"), Arbiter.IsFaultLatched());
		Expect(*this, TEXT("cold reset on a ramping fault"), Arbiter, Cold, EState::Faulted, EAction::Clear);
		TestTrue(TEXT("... latches it"), Arbiter.IsFaultLatched());
		Expect(*this, TEXT("ready, but latched"), Arbiter, Ready(), EState::Faulted, EAction::None);

		// Off and on is the way back, and it is a start from nothing.
		FArbiterInputs Off = Ready();
		Off.bEnable = false;
		Expect(*this, TEXT("switched off: the latch goes"), Arbiter, Off, EState::Off, EAction::None);
		TestFalse(TEXT("no latch once Off"), Arbiter.IsFaultLatched());
		Expect(*this, TEXT("switched on again"), Arbiter, Ready(), EState::Active, EAction::StartFromAttitude);
		TestFalse(TEXT("no latch once Active"), Arbiter.IsFaultLatched());
	}
	{
		FArbiter Arbiter = MakeFaulted();
		Arbiter.Reset();
		TestEqual(TEXT("Reset: Off"), static_cast<int32>(Arbiter.GetState()), static_cast<int32>(EState::Off));
		TestFalse(TEXT("Reset: no latch"), Arbiter.IsFaultLatched());
	}
	return true;
}

// Catches: a release-kind mode treated as a taker (a fold, no ramp) or as
// "off" (ending in Off, so the mode coming back would not start it again).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelArbiterLowLevelSuspends,
	"Ramms.Level.Arbiter.LowLevelSuspends",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelArbiterLowLevelSuspends::RunTest(const FString& Parameters)
{
	using namespace RammsLevelArbiterTests;

	FArbiterInputs Low = Ready();
	Low.Mode = EMode::Release;

	{
		FArbiter		   Arbiter = MakeActive();
		const FArbiterStep Step = Expect(*this, TEXT("a release mode"), Arbiter, Low, EState::Releasing, EAction::BeginRelease);
		TestFalse(TEXT("... is not an unexpected take-over"), Step.bUnexpected);
		Expect(*this, TEXT("releasing"), Arbiter, Low, EState::Releasing, EAction::None);

		FArbiterInputs Done = Low;
		Done.bCorrectionZero = true;
		Expect(*this, TEXT("release done"), Arbiter, Done, EState::Suspended, EAction::Clear);
		TestTrue(TEXT("Suspended: the motors stay claimed"), Arbiter.WantsClaim());
		TestFalse(TEXT("Suspended: no offsets"), Arbiter.HasOffsets());
		Expect(*this, TEXT("waiting in the release mode"), Arbiter, Done, EState::Suspended, EAction::None);
		Expect(*this, TEXT("the mode comes back"), Arbiter, Ready(), EState::Active, EAction::StartFromAttitude);
	}
	{
		// The mode comes back part way through: no restart, no step.
		FArbiter Arbiter = MakeActive();
		Arbiter.Tick(Low);
		Expect(*this, TEXT("the mode back mid-release"), Arbiter, Ready(), EState::Active, EAction::None);

		Arbiter.Tick(Low);
		FArbiterInputs Blind = Ready();
		Blind.bAttitudeValid = false;
		Expect(*this, TEXT("the mode back mid-release, no attitude"), Arbiter, Blind, EState::Holding, EAction::Freeze);
	}
	{
		// The release runs with or without an attitude (open loop then).
		FArbiter	   Arbiter = MakeHolding();
		FArbiterInputs Blind = Low;
		Blind.bAttitudeValid = false;
		Expect(*this, TEXT("a release mode while Holding"), Arbiter, Blind, EState::Releasing, EAction::BeginRelease);
	}
	{
		// Switched on in a release mode: it waits, it does not level.
		FArbiter Arbiter;
		Expect(*this, TEXT("switched on in a release mode"), Arbiter, Low, EState::Suspended, EAction::None);
		FArbiterInputs Off = Low;
		Off.bEnable = false;
		Expect(*this, TEXT("switched off while waiting"), Arbiter, Off, EState::Off, EAction::None);
	}
	return true;
}

// Catches: levelling on (or starting) with no attitude, a hold that drops
// the offsets, and a return that restarts from nothing rather than ColdResume.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelArbiterAttitudeLossHolds,
	"Ramms.Level.Arbiter.AttitudeLossHolds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelArbiterAttitudeLossHolds::RunTest(const FString& Parameters)
{
	using namespace RammsLevelArbiterTests;

	FArbiterInputs Blind = Ready();
	Blind.bAttitudeValid = false;

	{
		FArbiter Arbiter = MakeActive();
		Expect(*this, TEXT("attitude lost"), Arbiter, Blind, EState::Holding, EAction::Freeze);
		// Frozen once; the offsets stay where they were.
		Expect(*this, TEXT("still lost"), Arbiter, Blind, EState::Holding, EAction::None);
		TestTrue(TEXT("Holding: has offsets"), Arbiter.HasOffsets());
		TestTrue(TEXT("Holding: claims"), Arbiter.WantsClaim());
		Expect(*this, TEXT("attitude back"), Arbiter, Ready(), EState::Active, EAction::ColdResume);
		Expect(*this, TEXT("levelling on"), Arbiter, Ready(), EState::Active, EAction::None);
	}
	{
		// A divergence judged on no reading is not one: the hold comes first,
		// and a hold does not fault.
		FArbiter	   Arbiter = MakeActive();
		FArbiterInputs Both = Blind;
		Both.bDiverged = true;
		Expect(*this, TEXT("lost and diverged together"), Arbiter, Both, EState::Holding, EAction::Freeze);
		Expect(*this, TEXT("diverged while Holding"), Arbiter, Both, EState::Holding, EAction::None);
	}
	{
		// Switched on blind: it waits with nothing applied.
		FArbiter Arbiter;
		Expect(*this, TEXT("switched on with no attitude"), Arbiter, Blind, EState::Suspended, EAction::None);
		Expect(*this, TEXT("the attitude comes"), Arbiter, Ready(), EState::Active, EAction::StartFromAttitude);
	}
	{
		// Switched off while holding: still let go gently, not dropped.
		FArbiter	   Arbiter = MakeHolding();
		FArbiterInputs Off = Blind;
		Off.bEnable = false;
		Expect(*this, TEXT("switched off while Holding"), Arbiter, Off, EState::Releasing, EAction::BeginRelease);
		Off.bCorrectionZero = true;
		Expect(*this, TEXT("released blind"), Arbiter, Off, EState::Off, EAction::Clear);
	}
	return true;
}

// Catches: a fault that goes back to levelling once the divergence flag
// drops or the ramp ends, one that zeroes at once instead of ramping out,
// and one that keeps the motors claimed after it latches.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelArbiterDivergenceLatches,
	"Ramms.Level.Arbiter.DivergenceLatches",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelArbiterDivergenceLatches::RunTest(const FString& Parameters)
{
	using namespace RammsLevelArbiterTests;

	FArbiterInputs Diverged = Ready();
	Diverged.bDiverged = true;

	FArbiter Arbiter = MakeActive();
	Expect(*this, TEXT("diverged"), Arbiter, Diverged, EState::Faulted, EAction::BeginRelease);
	TestFalse(TEXT("ramping out: not latched"), Arbiter.IsFaultLatched());
	TestTrue(TEXT("ramping out: has offsets"), Arbiter.HasOffsets());
	TestTrue(TEXT("ramping out: claims"), Arbiter.WantsClaim());

	// The flag dropping (the watch is reset by the release) lifts nothing.
	Expect(*this, TEXT("ramping out"), Arbiter, Ready(), EState::Faulted, EAction::None);
	Expect(*this, TEXT("ramping out, flag still up"), Arbiter, Diverged, EState::Faulted, EAction::None);
	TestFalse(TEXT("still not latched"), Arbiter.IsFaultLatched());

	// Switching off mid-ramp does not cut the ramp short.
	FArbiterInputs Off = Ready();
	Off.bEnable = false;
	Expect(*this, TEXT("switched off mid-ramp"), Arbiter, Off, EState::Faulted, EAction::None);

	FArbiterInputs Done = Ready();
	Done.bCorrectionZero = true;
	Expect(*this, TEXT("ramp done"), Arbiter, Done, EState::Faulted, EAction::Clear);
	TestTrue(TEXT("latched"), Arbiter.IsFaultLatched());
	TestFalse(TEXT("latched: no offsets"), Arbiter.HasOffsets());
	TestFalse(TEXT("latched: no claim"), Arbiter.WantsClaim());

	// Everything it would otherwise start on is in front of it: it stays.
	Expect(*this, TEXT("latched, ready"), Arbiter, Done, EState::Faulted, EAction::None);
	FArbiterInputs Low = Ready();
	Low.Mode = EMode::Release;
	Expect(*this, TEXT("latched, a mode change"), Arbiter, Low, EState::Faulted, EAction::None);
	FArbiterInputs Resume = Ready();
	Resume.bResume = true;
	Resume.bResumeHasPlane = true;
	Expect(*this, TEXT("latched, a stray hand-back"), Arbiter, Resume, EState::Faulted, EAction::None);
	TestTrue(TEXT("still latched"), Arbiter.IsFaultLatched());

	Done.bEnable = false;
	Expect(*this, TEXT("switched off"), Arbiter, Done, EState::Off, EAction::None);
	Expect(*this, TEXT("switched on: anew"), Arbiter, Ready(), EState::Active, EAction::StartFromAttitude);

	// Only Active faults: a release under way is already letting go.
	FArbiter	   Releasing = MakeReleasing();
	FArbiterInputs OffDiverged = Diverged;
	OffDiverged.bEnable = false;
	Expect(*this, TEXT("diverged while Releasing"), Releasing, OffDiverged, EState::Releasing, EAction::None);

	// A second fault starts unlatched: the latch does not outlive the first.
	Arbiter.Tick(Diverged);
	TestEqual(TEXT("faulted again"), static_cast<int32>(Arbiter.GetState()), static_cast<int32>(EState::Faulted));
	TestFalse(TEXT("the second fault ramps out too"), Arbiter.IsFaultLatched());
	return true;
}

// Catches: fighting an unannounced taker (staying Active under Taken),
// releasing against it instead of folding, or not flagging it for the shell.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelArbiterUnexpectedModeFoldsOrZeroes,
	"Ramms.Level.Arbiter.UnexpectedModeFoldsOrZeroes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelArbiterUnexpectedModeFoldsOrZeroes::RunTest(const FString& Parameters)
{
	using namespace RammsLevelArbiterTests;

	FArbiterInputs Taken = Ready();
	Taken.Mode = EMode::Taken;

	{
		FArbiter		   Arbiter = MakeActive();
		const FArbiterStep Step = Expect(*this, TEXT("taken with no yield, Active"), Arbiter, Taken, EState::Yielded, EAction::Fold);
		TestTrue(TEXT("... is flagged unexpected"), Step.bUnexpected);
		// Once: the next tick has nothing left to fold or to flag.
		const FArbiterStep Next = Expect(*this, TEXT("the tick after"), Arbiter, Taken, EState::Yielded, EAction::None);
		TestFalse(TEXT("... flags nothing"), Next.bUnexpected);
	}
	{
		FArbiter	   Arbiter = MakeHolding();
		FArbiterInputs Blind = Taken;
		Blind.bAttitudeValid = false;
		const FArbiterStep Step = Expect(*this, TEXT("taken with no yield, Holding"), Arbiter, Blind, EState::Yielded, EAction::Fold);
		TestTrue(TEXT("... is flagged unexpected"), Step.bUnexpected);
	}
	{
		// The taker comes first even when switched off on the same tick: a
		// release would move legs someone else is writing.
		FArbiter	   Arbiter = MakeReleasing();
		FArbiterInputs Off = Taken;
		Off.bEnable = false;
		const FArbiterStep Step = Expect(*this, TEXT("taken with no yield, Releasing"), Arbiter, Off, EState::Yielded, EAction::Fold);
		TestTrue(TEXT("... is flagged unexpected"), Step.bUnexpected);
		Expect(*this, TEXT("then off, with nothing to release"), Arbiter, Off, EState::Off, EAction::None);
	}
	{
		// With nothing applied there is nothing unexpected about it.
		FArbiter		   Arbiter;
		const FArbiterStep Step = Expect(*this, TEXT("switched on under a taker"), Arbiter, Taken, EState::Yielded, EAction::None);
		TestFalse(TEXT("... flags nothing"), Step.bUnexpected);
	}
	{
		// A fault's ramp is the fault's: a taken mode does not fold it.
		FArbiter		   Arbiter = MakeFaulted();
		const FArbiterStep Step = Expect(*this, TEXT("taken with no yield, Faulted"), Arbiter, Taken, EState::Faulted, EAction::None);
		TestFalse(TEXT("... flags nothing"), Step.bUnexpected);
	}
	return true;
}

// Catches: levelling begun on a stance that is still being changed (the
// wanted attitude then walks on top of the stance's own rock), a start kept
// waiting once the stance has arrived, a hand-back's plane refused because
// the stance is about to change, or levelling under way dropped by one.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelArbiterStartWaitsForStance,
	"Ramms.Level.Arbiter.StartWaitsForTheStance",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelArbiterStartWaitsForStance::RunTest(const FString& Parameters)
{
	using namespace RammsLevelArbiterTests;

	FArbiterInputs Changing = Ready();
	Changing.bStanceChanging = true;
	{
		// Switched on in the middle of a mode switch, or the mode it levels
		// in coming up with its stance still being taken: it waits.
		FArbiter Arbiter;
		Expect(*this, TEXT("from Off, the stance changing"), Arbiter, Changing, EState::Suspended, EAction::None);
		TestFalse(TEXT("waiting: no offsets"), Arbiter.HasOffsets());
		Expect(*this, TEXT("still changing"), Arbiter, Changing, EState::Suspended, EAction::None);
		Expect(*this, TEXT("the stance has arrived"), Arbiter, Ready(), EState::Active, EAction::StartFromAttitude);
		// Under way, a stance change is ridden through: nothing is dropped.
		Expect(*this, TEXT("a stance change while levelling"), Arbiter, Changing, EState::Active, EAction::None);
	}
	{
		// A taker's mode left with no hand-back, the next mode's stance being
		// ramped in: the cold start waits for it too.
		FArbiter Arbiter = MakeYielded();
		Expect(*this, TEXT("from Yielded, the stance changing"), Arbiter, Changing, EState::Suspended, EAction::None);
		Expect(*this, TEXT("... and arrived"), Arbiter, Ready(), EState::Active, EAction::StartFromAttitude);
	}
	{
		// A hand-back is not a start: the plane is out of the baselines, and
		// is taken up whatever the stance is about to do.
		FArbiter	   Arbiter = MakeYielded();
		FArbiterInputs Resume = Changing;
		Resume.Mode = EMode::Taken;
		Resume.bResume = true;
		Resume.bResumeHasPlane = true;
		Expect(*this, TEXT("hand-back with a plane, the stance changing"), Arbiter, Resume, EState::Active, EAction::Adopt);
	}
	{
		// A release is not held up either, nor a release taken back.
		FArbiter	   Arbiter = MakeActive();
		FArbiterInputs Off = Changing;
		Off.bEnable = false;
		Expect(*this, TEXT("switched off, the stance changing"), Arbiter, Off, EState::Releasing, EAction::BeginRelease);
		Expect(*this, TEXT("switched on again, the stance changing"), Arbiter, Changing, EState::Active, EAction::None);
	}
	{
		// A cold reset in the middle of one: whatever was applied is gone, and
		// the start waits like any other.
		FArbiter	   Arbiter = MakeActive();
		FArbiterInputs Reset = Changing;
		Reset.bColdReset = true;
		Expect(*this, TEXT("cold reset, the stance changing"), Arbiter, Reset, EState::Suspended, EAction::Clear);
		Expect(*this, TEXT("... then the stance arrives"), Arbiter, Ready(), EState::Active, EAction::StartFromAttitude);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
