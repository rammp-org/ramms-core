// Tests for the robot base's command offset layer (URammsRobotBaseComponent::ClaimCommandOffsetLayer and the calls
// beside it) and for what two controllers do about a command moved under them (URamms5BarLinkageController::
// SyncTargetToCommands and its held axes, URammpGen1ControllerComponent::SyncTargetsToCommands).
//
// A position motor's command is a baseline, which every controller writes through SetMotorCommand as it always has,
// plus an offset, which one object writes. The base tells a claimed motor the sum once a frame, in its tick; a motor
// nobody claimed is told at once, exactly as before. Also where a stance change starts a motor of the base, with and
// without an offset on it (RammsDrive::StanceMotorStart, the drive mode selector's rule).
//
// Checked on a bare game world: an actor with a robot base over a made-up registry (a crank, a profiled lift on a
// reversed joint, two hips, a torque wheel, a speed motor) and an actuation backend with no physics that records
// every call it is given, in order -- so "told once", "told nothing" and "told the same as before" are counted, not
// inferred. The tick is called by hand.

#include "CoreMinimal.h"
#include "Engine/DataTable.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "UObject/Package.h"
#include "UObject/UnrealType.h"

#include "RammpGen1ControllerComponent.h"
#include "Ramms5BarKinematics.h"
#include "Ramms5BarLinkageController.h"
#include "RammsActuationBackend.h"
#include "RammsActuationBackendRegistry.h"
#include "RammsDriveHandOver.h"
#include "RammsMotorSpec.h"
#include "RammsRobotBaseComponent.h"

#include "Tests/RammsCommandOffsetRig.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCommandOffsetComposes,
	"Ramms.Base.CommandOffset.Composes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCommandOffsetComposes::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	FRig R;
	if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
	{
		return false;
	}
	// Before anything is claimed: a command is its own effective command, and there is no offset.
	TestTrue(TEXT("never commanded: no effective command"), FMath::IsNaN(R.Effective(Crank)));
	R.Base->SetMotorCommand(Crank, 0.2f);
	TestEqual(TEXT("unclaimed: told at once"), R.Log->LastFor(Crank), 0.2f);
	TestTrue(TEXT("unclaimed: the effective command is the command"), SameBits(R.Effective(Crank), 0.2f));
	TestEqual(TEXT("unclaimed: no offset"), R.Base->GetMotorCommandOffset(Crank), 0.0f);
	TestFalse(TEXT("unclaimed: an offset is refused"), R.Base->SetMotorCommandOffset(Crank, 0.1f, R.Layer()));

	const FName Ids[] = { Crank, Lift };
	TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	TestEqual(TEXT("a claimed motor starts with no offset"), R.Base->GetMotorCommandOffset(Crank), 0.0f);
	TestTrue(TEXT("...and told what it was"), SameBits(R.Effective(Crank), 0.2f));
	TestTrue(TEXT("an offset set"), R.Base->SetMotorCommandOffset(Crank, 0.3f, R.Layer()));
	// The baseline is what every other controller reads back: the offset must not show in it.
	TestTrue(TEXT("GetMotorCommand is still the baseline"), SameBits(R.Baseline(Crank), 0.2f));
	TestEqual(TEXT("the offset reads back"), R.Base->GetMotorCommandOffset(Crank), 0.3f);
	TestTrue(TEXT("the effective command is the sum"), SameBits(R.Effective(Crank), 0.2f + 0.3f));
	R.Tick();
	TestTrue(TEXT("...which is what the motor is told"), SameBits(R.Log->LastFor(Crank), 0.2f + 0.3f));
	TestTrue(TEXT("...the baseline as it was"), SameBits(R.Baseline(Crank), 0.2f));

	// A reversed, profiled motor: the sum in the robot's sense, then the joint's sign, through its profile.
	TestTrue(TEXT("an offset waits for a first baseline"), R.Base->SetMotorCommandOffset(Lift, 0.25f, R.Layer()));
	TestTrue(TEXT("...with no effective command meanwhile"), FMath::IsNaN(R.Effective(Lift)));
	R.Tick();
	TestEqual(TEXT("...and nothing told"), R.Log->CountFor(Lift), 0);
	R.Base->SetMotorCommand(Lift, 0.5f);
	R.Tick();
	TestEqual(TEXT("the reversed motor told once"), R.Log->CountFor(Lift), 1);
	TestTrue(TEXT("...the sum, in its joint's sense, as a profile"),
		R.Log->Pushes.Last().Kind == FPush::EKind::Profile && SameBits(R.Log->Pushes.Last().Value, -(0.5f + 0.25f)));
	TestTrue(TEXT("...its baseline the command"), SameBits(R.Baseline(Lift), 0.5f));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCommandOffsetOnePush,
	"Ramms.Base.CommandOffset.OnePushPerFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCommandOffsetOnePush::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	FRig R;
	if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
	{
		return false;
	}
	const FName Ids[] = { Crank };
	TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));

	// A frame with both writers, the baseline twice: nothing reaches the motor until the base's tick, and then the
	// sum, once. (Told the bare baseline first, a servo eased at the physics rate would start for it.)
	R.Base->SetMotorCommand(Crank, 0.1f);
	TestEqual(TEXT("a baseline write tells the motor nothing yet"), R.Log->Pushes.Num(), 0);
	TestTrue(TEXT("the offset"), R.Base->SetMotorCommandOffset(Crank, 0.2f, R.Layer()));
	R.Base->SetMotorCommand(Crank, 0.15f);
	TestEqual(TEXT("nor do an offset write and another baseline"), R.Log->Pushes.Num(), 0);
	R.Tick();
	TestEqual(TEXT("the tick tells it once"), R.Log->CountFor(Crank), 1);
	TestTrue(TEXT("...the latest baseline plus the offset"), SameBits(R.Log->LastFor(Crank), 0.15f + 0.2f));

	// Nothing written, nothing told.
	R.Tick();
	TestEqual(TEXT("a frame with no write tells it nothing"), R.Log->CountFor(Crank), 1);

	// Either writer alone.
	TestTrue(TEXT("the offset alone"), R.Base->SetMotorCommandOffset(Crank, 0.25f, R.Layer()));
	TestEqual(TEXT("...waits"), R.Log->CountFor(Crank), 1);
	R.Tick();
	TestEqual(TEXT("...told once"), R.Log->CountFor(Crank), 2);
	TestTrue(TEXT("...the sum"), SameBits(R.Log->LastFor(Crank), 0.15f + 0.25f));
	R.Base->SetMotorCommand(Crank, 0.3f);
	TestEqual(TEXT("the baseline alone waits"), R.Log->CountFor(Crank), 2);
	R.Tick();
	TestEqual(TEXT("...told once"), R.Log->CountFor(Crank), 3);
	TestTrue(TEXT("...the sum, never the bare baseline"), SameBits(R.Log->LastFor(Crank), 0.3f + 0.25f));

	// A write after the tick is told on the next one. The same values again still count as a write: a controller
	// that commands every frame keeps its cadence at the backend.
	R.Base->SetMotorCommand(Crank, 0.3f);
	TestEqual(TEXT("a write after the tick waits for the next"), R.Log->CountFor(Crank), 3);
	R.Tick();
	TestEqual(TEXT("...and is told then"), R.Log->CountFor(Crank), 4);

	// A motor beside it that nobody claimed is told at once, every time.
	R.Base->SetMotorCommand(HipA, 0.4f);
	R.Base->SetMotorCommand(HipA, 0.5f);
	TestEqual(TEXT("an unclaimed motor is told at each write"), R.Log->CountFor(HipA), 2);
	R.Tick();
	TestEqual(TEXT("...and nothing more at the tick"), R.Log->CountFor(HipA), 2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCommandOffsetClamps,
	"Ramms.Base.CommandOffset.ClampsTheSum",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCommandOffsetClamps::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	FRig R;
	if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
	{
		return false;
	}
	const FName Ids[] = { Crank };
	TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	// The crank's range is +-1.
	R.Base->SetMotorCommand(Crank, 0.8f);
	TestTrue(TEXT("an offset past the range is taken"), R.Base->SetMotorCommandOffset(Crank, 0.5f, R.Layer()));
	R.Tick();
	TestEqual(TEXT("the motor is told the range's end"), R.Log->LastFor(Crank), 1.0f);
	TestEqual(TEXT("...which is the effective command"), R.Effective(Crank), 1.0f);
	TestTrue(TEXT("the baseline is not moved by the offset"), SameBits(R.Baseline(Crank), 0.8f));
	TestEqual(TEXT("the offset reads as set"), R.Base->GetMotorCommandOffset(Crank), 0.5f);
	// The other end, and back inside: the clamp is on the sum each time, nothing is kept from the last.
	TestTrue(TEXT("the other way"), R.Base->SetMotorCommandOffset(Crank, -2.5f, R.Layer()));
	R.Tick();
	TestEqual(TEXT("the other end"), R.Log->LastFor(Crank), -1.0f);
	TestTrue(TEXT("back inside"), R.Base->SetMotorCommandOffset(Crank, -0.5f, R.Layer()));
	R.Tick();
	TestTrue(TEXT("the plain sum again"), SameBits(R.Log->LastFor(Crank), 0.8f - 0.5f));
	// A baseline is clamped on its own, as it always was, before the offset is added.
	R.Base->SetMotorCommand(Crank, 1.7f);
	TestEqual(TEXT("a baseline past the range is clamped as before"), R.Baseline(Crank), 1.0f);
	R.Tick();
	TestTrue(TEXT("...and the offset added to that"), SameBits(R.Log->LastFor(Crank), 1.0f - 0.5f));
	TestFalse(TEXT("an offset that is no number is refused"), R.Base->SetMotorCommandOffset(Crank, NAN, R.Layer()));
	TestEqual(TEXT("...the last one standing"), R.Base->GetMotorCommandOffset(Crank), -0.5f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCommandOffsetRebase,
	"Ramms.Base.CommandOffset.RebaseKeepsEffective",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCommandOffsetRebase::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	FRig R;
	if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
	{
		return false;
	}
	const FName Ids[] = { Crank, HipA };
	TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	R.Base->SetMotorCommand(Crank, 0.2f);
	TestTrue(TEXT("an offset"), R.Base->SetMotorCommandOffset(Crank, 0.3f, R.Layer()));
	R.Tick();
	const float Told = R.Effective(Crank);
	const int32 Before = R.Log->Pushes.Num();

	// The fold: the offset into the baseline. What the motor is told is the same to the bit, and it is told nothing.
	TestTrue(TEXT("rebased onto the effective command"), R.Base->RebaseMotorCommand(Crank, Told, R.Layer()));
	TestTrue(TEXT("the effective command is bit-equal"), SameBits(R.Effective(Crank), Told));
	TestTrue(TEXT("the baseline is the new one"), SameBits(R.Baseline(Crank), Told));
	TestEqual(TEXT("the offset is what is left: none"), R.Base->GetMotorCommandOffset(Crank), 0.0f);
	R.Tick();
	TestEqual(TEXT("nothing was told, then or at the tick"), R.Log->Pushes.Num(), Before);

	// And back out, onto a baseline the sum does not round-trip through: still the same bits.
	TestTrue(TEXT("rebased onto another baseline"), R.Base->RebaseMotorCommand(Crank, 1.0f / 3.0f, R.Layer()));
	TestTrue(TEXT("the effective command is bit-equal"), SameBits(R.Effective(Crank), Told));
	TestTrue(TEXT("the baseline is the new one"), SameBits(R.Baseline(Crank), 1.0f / 3.0f));
	TestEqual(TEXT("the offset is the effective command less the baseline"), R.Base->GetMotorCommandOffset(Crank), Told - 1.0f / 3.0f, 1e-7f);
	R.Tick();
	TestEqual(TEXT("nothing was told"), R.Log->Pushes.Num(), Before);
	// The controller whose baseline it is writes it again: the motor is told where it already was.
	R.Base->SetMotorCommand(Crank, 1.0f / 3.0f);
	R.Tick();
	TestEqual(TEXT("the baseline written again: told once"), R.Log->Pushes.Num(), Before + 1);
	TestEqual(TEXT("...where it was (the offset has the right sign)"), R.Log->LastFor(Crank), Told, 1e-6f);

	// A write waiting for the tick is still waiting after a rebase, and is told as it stood.
	R.Base->SetMotorCommand(Crank, 0.1f);
	const float Waiting = R.Effective(Crank);
	TestTrue(TEXT("rebased with a write waiting"), R.Base->RebaseMotorCommand(Crank, -0.4f, R.Layer()));
	TestEqual(TEXT("...nothing told by the rebase"), R.Log->Pushes.Num(), Before + 1);
	R.Tick();
	TestEqual(TEXT("...the write told at the tick"), R.Log->Pushes.Num(), Before + 2);
	TestTrue(TEXT("...as it stood"), SameBits(R.Log->LastFor(Crank), Waiting));

	// A baseline is always inside the range: a rebase past it stops at the end, the motor told the same still.
	TestTrue(TEXT("rebased past the range"), R.Base->RebaseMotorCommand(Crank, 5.0f, R.Layer()));
	TestEqual(TEXT("the baseline at the range's end"), R.Baseline(Crank), 1.0f);
	TestTrue(TEXT("the effective command is bit-equal"), SameBits(R.Effective(Crank), Waiting));
	TestEqual(TEXT("the offset makes up the difference"), R.Base->GetMotorCommandOffset(Crank), Waiting - 1.0f, 1e-7f);

	// Who may, and on what.
	TestFalse(TEXT("another object may not rebase"), R.Base->RebaseMotorCommand(Crank, 0.0f, R.Other()));
	TestFalse(TEXT("a motor not claimed is not rebased"), R.Base->RebaseMotorCommand(HipB, 0.0f, R.Layer()));
	TestFalse(TEXT("nor one nothing has commanded"), R.Base->RebaseMotorCommand(HipA, 0.0f, R.Layer()));
	TestEqual(TEXT("...the baseline as it was"), R.Baseline(Crank), 1.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCommandOffsetSingleOwner,
	"Ramms.Base.CommandOffset.SingleOwner",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCommandOffsetSingleOwner::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	FRig R;
	if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
	{
		return false;
	}
	const FName CrankOnly[] = { Crank };
	const FName LiftOnly[] = { Lift };
	TestFalse(TEXT("nobody cannot claim"), R.Base->ClaimCommandOffsetLayer(nullptr, CrankOnly));
	TestTrue(TEXT("the first to claim holds the layer"), R.Base->ClaimCommandOffsetLayer(R.Layer(), CrankOnly));
	TestFalse(TEXT("a second layer is refused"), R.Base->ClaimCommandOffsetLayer(R.Other(), LiftOnly));
	TestFalse(TEXT("...even for a motor already claimed"), R.Base->ClaimCommandOffsetLayer(R.Other(), CrankOnly));
	TestFalse(TEXT("...and may not write an offset"), R.Base->SetMotorCommandOffset(Crank, 0.1f, R.Other()));
	TestEqual(TEXT("...none written"), R.Base->GetMotorCommandOffset(Crank), 0.0f);
	R.Base->ReleaseCommandOffsetLayer(R.Other());
	TestTrue(TEXT("...nor give the layer up for its holder"), R.Base->SetMotorCommandOffset(Crank, 0.1f, R.Layer()));
	TestFalse(TEXT("the lift is not the holder's yet"), R.Base->SetMotorCommandOffset(Lift, 0.1f, R.Layer()));
	TestTrue(TEXT("the holder may add motors"), R.Base->ClaimCommandOffsetLayer(R.Layer(), LiftOnly));
	TestTrue(TEXT("...and write to them"), R.Base->SetMotorCommandOffset(Lift, 0.1f, R.Layer()));
	TestEqual(TEXT("...the ones it had as they were"), R.Base->GetMotorCommandOffset(Crank), 0.1f);
	TestTrue(TEXT("the offsets brought to zero"), R.Base->SetMotorCommandOffset(Crank, 0.0f, R.Layer()) && R.Base->SetMotorCommandOffset(Lift, 0.0f, R.Layer()));
	R.Base->ReleaseCommandOffsetLayer(R.Layer());
	TestFalse(TEXT("given up: its holder writes no more"), R.Base->SetMotorCommandOffset(Crank, 0.1f, R.Layer()));
	TestTrue(TEXT("...and another may claim"), R.Base->ClaimCommandOffsetLayer(R.Other(), CrankOnly));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCommandOffsetPositionOnly,
	"Ramms.Base.CommandOffset.PositionOnly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCommandOffsetPositionOnly::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	FRig R;
	if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
	{
		return false;
	}
	const FName WheelOnly[] = { Wheel };
	const FName SpinOnly[] = { Spin };
	const FName Unknown[] = { FName(TEXT("no_such_motor")) };
	const FName Mixed[] = { Crank, Wheel };
	const FName CrankOnly[] = { Crank };
	TestFalse(TEXT("a torque motor refuses"), R.Base->ClaimCommandOffsetLayer(R.Layer(), WheelOnly));
	TestFalse(TEXT("a speed motor refuses"), R.Base->ClaimCommandOffsetLayer(R.Layer(), SpinOnly));
	TestFalse(TEXT("a motor the registry does not have refuses"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Unknown));
	// One bad motor refuses the whole claim: none of it is claimed, and the layer is not taken.
	TestFalse(TEXT("a torque motor among position ones refuses the lot"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Mixed));
	TestFalse(TEXT("...the position motor not claimed"), R.Base->SetMotorCommandOffset(Crank, 0.1f, R.Layer()));
	R.Base->SetMotorCommand(Crank, 0.2f);
	TestEqual(TEXT("...and still told at once"), R.Log->CountFor(Crank), 1);
	TestTrue(TEXT("...the layer free for another"), R.Base->ClaimCommandOffsetLayer(R.Other(), CrankOnly));
	// The torque wheel is commanded exactly as before with a layer beside it.
	R.Base->SetMotorCommand(Wheel, 3.0f);
	TestEqual(TEXT("the torque motor told at once"), R.Log->LastFor(Wheel), 3.0f);
	TestEqual(TEXT("...with no offset"), R.Base->GetMotorCommandOffset(Wheel), 0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCommandOffsetUnclaimedIsLegacy,
	"Ramms.Base.CommandOffset.UnclaimedIsLegacy",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCommandOffsetUnclaimedIsLegacy::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	// The same commands to motors nobody claimed, with no layer at all and with one on the crank beside them, where
	// the backend eases profiles and where it leaves them to the base: every call the backend gets, in order, is the
	// one the base made before there was a layer (written out below from that code, not recorded from it: told at
	// each write, the first command to a profiled motor starting from where its joint is, a speed motor told its
	// rate at each tick after the profiles are stepped, a motor let go told so and no more).
	for (const bool bLayerBeside : { false, true })
	{
		for (const bool bBackendProfiles : { true, false })
		{
			FRig R;
			if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
			{
				return false;
			}
			R.Log->bProfiles = bBackendProfiles;
			if (bLayerBeside)
			{
				const FName Ids[] = { Crank };
				R.Base->SetMotorCommand(Crank, 0.1f);
				TestTrue(TEXT("a layer on the crank"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
				TestTrue(TEXT("...with an offset"), R.Base->SetMotorCommandOffset(Crank, 0.2f, R.Layer()));
				R.Tick();
			}
			const int32 Start = R.Log->Pushes.Num();
			// The lift's joint reads 0.4 in its own sense: -0.4 in the robot's (its Direction is -1).
			R.Log->Joints.Add(Lift, 0.4f);
			R.Base->SetMotorCommand(Lift, 0.5f);
			R.Base->SetMotorCommand(Lift, 3.0f);
			R.Base->SetMotorCommand(HipA, 0.25f);
			// A speed for the speed motor: the base's own loop, which tells it at each tick.
			R.Base->SetMotorVelocityCommand(Spin, 2.0f);
			R.Tick(0.1f);
			R.Base->SetMotorCommand(HipA, 0.25f);
			R.Base->SetMotorCommand(Wheel, 3.0f);
			R.Tick(0.1f);
			// Motors nobody claimed let go, a plain one and the profiled one; a direct command ending the speed
			// loop; a tick with nothing left to step; and the profiled motor commanded afresh.
			TestTrue(TEXT("the hip let go"), R.Base->ReleaseMotor(HipA));
			TestTrue(TEXT("the lift let go"), R.Base->ReleaseMotor(Lift));
			R.Base->SetMotorCommand(Spin, 0.5f);
			R.Tick(0.1f);
			R.Base->SetMotorCommand(Lift, 1.0f);

			TArray<FPush> Expected;
			if (bBackendProfiles)
			{
				// Each command a profile target at once, the first from where the joint is; the hip and the wheel
				// at once; the ticks the speed motor's rate and nothing else.
				Expected.Add(ProfileFrom(TEXT("lift"), -0.5f, 0.4f));
				Expected.Add(Profile(TEXT("lift"), -2.0f));
				Expected.Add(Command(TEXT("hip_a"), 0.25f));
				Expected.Add(Command(TEXT("spin"), 2.0f));
				Expected.Add(Command(TEXT("hip_a"), 0.25f));
				Expected.Add(Command(TEXT("wheel"), 3.0f));
				Expected.Add(Command(TEXT("spin"), 2.0f));
				// Let go: the backend's profile with the motor. Then the profile anew, from the joint.
				Expected.Add(Released(TEXT("hip_a")));
				Expected.Add(ProfileCleared(TEXT("lift")));
				Expected.Add(Released(TEXT("lift")));
				Expected.Add(Command(TEXT("spin"), 0.5f));
				Expected.Add(ProfileFrom(TEXT("lift"), -1.0f, 0.4f));
			}
			else
			{
				// The base eases it: held where the joint is at the first command, then 1.5 units/s towards the
				// target (2, clamped) for 0.1 s at each tick, in the joint's sense; the speed motor after it in
				// each tick.
				Expected.Add(Command(TEXT("lift"), 0.4f));
				Expected.Add(Command(TEXT("hip_a"), 0.25f));
				Expected.Add(Command(TEXT("lift"), 0.25f));
				Expected.Add(Command(TEXT("spin"), 2.0f));
				Expected.Add(Command(TEXT("hip_a"), 0.25f));
				Expected.Add(Command(TEXT("wheel"), 3.0f));
				Expected.Add(Command(TEXT("lift"), 0.1f));
				Expected.Add(Command(TEXT("spin"), 2.0f));
				// Let go: no more eased or told at the tick. Then held where the joint is, as a first command.
				Expected.Add(Released(TEXT("hip_a")));
				Expected.Add(Released(TEXT("lift")));
				Expected.Add(Command(TEXT("spin"), 0.5f));
				Expected.Add(Command(TEXT("lift"), 0.4f));
			}
			const FString Case = FString::Printf(TEXT("layer beside %d, backend profiles %d"), bLayerBeside ? 1 : 0, bBackendProfiles ? 1 : 0);
			TestEqual(*(Case + TEXT(": as many calls as before")), R.Log->Pushes.Num() - Start, Expected.Num());
			for (int32 i = 0; i < Expected.Num() && Start + i < R.Log->Pushes.Num(); ++i)
			{
				if (!Matches(R.Log->Pushes[Start + i], Expected[i]))
				{
					AddError(FString::Printf(TEXT("%s: call %d is '%s', was '%s' before"), *Case, i, *R.Log->Pushes[Start + i].ToString(),
						*Expected[i].ToString()));
				}
			}
			TestEqual(*(Case + TEXT(": the lift's command as given")), R.Baseline(Lift), 1.0f);
			TestTrue(*(Case + TEXT(": its effective command the same")), SameBits(R.Effective(Lift), 1.0f));
			TestTrue(*(Case + TEXT(": the hip let go has no command")), FMath::IsNaN(R.Baseline(HipA)) && FMath::IsNaN(R.Effective(HipA)));
			if (bLayerBeside)
			{
				// Letting an unclaimed motor go is nothing to the layer beside it.
				TestEqual(*(Case + TEXT(": the crank's offset as it was")), R.Base->GetMotorCommandOffset(Crank), 0.2f);
				TestTrue(*(Case + TEXT(": its effective command too")), SameBits(R.Effective(Crank), 0.1f + 0.2f));
				TestEqual(*(Case + TEXT(": and told nothing more")), R.Log->CountFor(Crank), 2);
			}
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCommandOffsetReleaseFlushes,
	"Ramms.Base.CommandOffset.ReleaseFlushes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCommandOffsetReleaseFlushes::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	const FName Ids[] = { Crank };
	{
		// A baseline written and not yet told when the layer is given up: told then, since nothing will tell it at
		// the tick any more; and from then on at once.
		FRig R;
		if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
		{
			return false;
		}
		TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
		R.Base->SetMotorCommand(Crank, 0.2f);
		TestEqual(TEXT("waiting"), R.Log->CountFor(Crank), 0);
		R.Base->ReleaseCommandOffsetLayer(R.Layer());
		TestEqual(TEXT("given up: the waiting write told"), R.Log->CountFor(Crank), 1);
		TestEqual(TEXT("...the baseline"), R.Log->LastFor(Crank), 0.2f);
		R.Tick();
		TestEqual(TEXT("...and not again at the tick"), R.Log->CountFor(Crank), 1);
		R.Base->SetMotorCommand(Crank, 0.3f);
		TestEqual(TEXT("commanded at once from then on"), R.Log->CountFor(Crank), 2);
	}
	{
		// Nothing waiting and no offset on: giving the layer up tells the motor nothing.
		FRig R;
		if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
		{
			return false;
		}
		TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
		R.Base->SetMotorCommand(Crank, 0.2f);
		TestTrue(TEXT("an offset"), R.Base->SetMotorCommandOffset(Crank, 0.3f, R.Layer()));
		R.Tick();
		TestTrue(TEXT("...brought back to zero"), R.Base->SetMotorCommandOffset(Crank, 0.0f, R.Layer()));
		R.Tick();
		const int32 Before = R.Log->Pushes.Num();
		TestEqual(TEXT("told its baseline"), R.Log->LastFor(Crank), 0.2f);
		R.Base->ReleaseCommandOffsetLayer(R.Layer());
		TestEqual(TEXT("given up at rest: nothing told"), R.Log->Pushes.Num(), Before);
	}
	{
		// An offset still on: it is dropped (a step, and said so), the motor told its baseline at once.
		FRig R;
		if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
		{
			return false;
		}
		AddExpectedMessagePlain(TEXT("command offset layer was given up with an offset"), ELogVerbosity::Warning,
			EAutomationExpectedMessageFlags::Contains, 1);
		TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
		R.Base->SetMotorCommand(Crank, 0.2f);
		TestTrue(TEXT("an offset"), R.Base->SetMotorCommandOffset(Crank, 0.3f, R.Layer()));
		R.Tick();
		const int32 Before = R.Log->CountFor(Crank);
		R.Base->ReleaseCommandOffsetLayer(R.Layer());
		TestEqual(TEXT("given up with an offset on: told once more"), R.Log->CountFor(Crank), Before + 1);
		TestEqual(TEXT("...its baseline"), R.Log->LastFor(Crank), 0.2f);
		TestEqual(TEXT("...no offset left"), R.Base->GetMotorCommandOffset(Crank), 0.0f);
		TestTrue(TEXT("...its effective command its baseline"), SameBits(R.Effective(Crank), 0.2f));
	}
	{
		// A claimed motor let go (ReleaseMotor) loses its offset with its command: the next command to it is not
		// added to an offset from before.
		FRig R;
		if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
		{
			return false;
		}
		TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
		R.Base->SetMotorCommand(Crank, 0.2f);
		TestTrue(TEXT("an offset"), R.Base->SetMotorCommandOffset(Crank, 0.3f, R.Layer()));
		TestTrue(TEXT("let go"), R.Base->ReleaseMotor(Crank));
		TestEqual(TEXT("...its offset gone"), R.Base->GetMotorCommandOffset(Crank), 0.0f);
		TestTrue(TEXT("...and its command"), FMath::IsNaN(R.Baseline(Crank)) && FMath::IsNaN(R.Effective(Crank)));
		R.Tick();
		TestEqual(TEXT("...nothing told at the tick"), R.Log->CountFor(Crank), 0);
		R.Base->SetMotorCommand(Crank, 0.4f);
		R.Tick();
		TestEqual(TEXT("commanded again: the bare command, at the tick (still claimed)"), R.Log->LastFor(Crank), 0.4f);
		TestTrue(TEXT("the offsets brought to zero"), R.Base->SetMotorCommandOffset(Crank, 0.0f, R.Layer()));
		R.Base->ReleaseCommandOffsetLayer(R.Layer());
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCommandOffsetClaimedProfile,
	"Ramms.Base.CommandOffset.ClaimedProfileEasedByBase",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCommandOffsetClaimedProfile::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	// A claimed motor whose profile the base eases itself (the backend takes no profiles: Chaos, or MuJoCo before its
	// model is compiled). The tick tells it its sum and then steps the profiles, in that order: the profile stepped
	// in a tick is the one that tick's command set.
	FRig R;
	if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
	{
		return false;
	}
	R.Log->bProfiles = false;
	const FName Ids[] = { Lift };
	TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	// The lift's joint reads 0.4 in its own sense; it is reversed, and eased at 1.5 units/s.
	R.Log->Joints.Add(Lift, 0.4f);
	R.Base->SetMotorCommand(Lift, 0.5f);
	TestTrue(TEXT("an offset"), R.Base->SetMotorCommandOffset(Lift, 0.25f, R.Layer()));
	TestEqual(TEXT("nothing told before the tick"), R.Log->Pushes.Num(), 0);
	R.Tick(0.1f);
	// The first command holds the joint where it is, and the same tick eases it a step towards the sum (-0.75 in the
	// joint's sense). Stepped before the command were told, the tick would end at the hold.
	if (TestEqual(TEXT("the first tick: held, then eased"), R.Log->Pushes.Num(), 2))
	{
		TestTrue(TEXT("...held where the joint is"), Matches(R.Log->Pushes[0], Command(TEXT("lift"), 0.4f)));
		TestTrue(TEXT("...and a step towards the sum"), Matches(R.Log->Pushes[1], Command(TEXT("lift"), 0.25f)));
	}
	// On to the sum, one command a tick, and it stays there.
	for (int32 i = 0; i < 8; ++i)
	{
		R.Tick(0.1f);
	}
	TestEqual(TEXT("one command a tick from then on"), R.Log->Pushes.Num(), 2 + 8);
	TestEqual(TEXT("eased to the sum, in the joint's sense"), R.Log->LastFor(Lift), -(0.5f + 0.25f), 1e-5f);
	// Arrived. An offset written now moves the motor in the very tick that tells it, not the one after.
	TestTrue(TEXT("the offset moved"), R.Base->SetMotorCommandOffset(Lift, 0.0f, R.Layer()));
	R.Tick(0.1f);
	TestEqual(TEXT("...one command"), R.Log->Pushes.Num(), 2 + 8 + 1);
	TestEqual(TEXT("...a step towards the new sum already"), R.Log->LastFor(Lift), -(0.75f - 0.15f), 1e-5f);
	// The baseline alone the same.
	R.Base->SetMotorCommand(Lift, 0.9f);
	R.Tick(0.1f);
	TestEqual(TEXT("a baseline written: a step back towards it in its tick"), R.Log->LastFor(Lift), -(0.6f + 0.15f), 1e-5f);
	TestEqual(TEXT("...one command"), R.Log->Pushes.Num(), 2 + 8 + 2);
	R.Base->ReleaseCommandOffsetLayer(R.Layer());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCommandOffsetCollectedHolder,
	"Ramms.Base.CommandOffset.CollectedHolder",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCommandOffsetCollectedHolder::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	// A holder destroyed without giving the layer up must not leave its offsets on: the base finds out at its next
	// tick, or when another claims, whichever is first. Each drops an offset still on, and says so.
	AddExpectedMessagePlain(TEXT("command offset layer was given up with an offset"), ELogVerbosity::Warning,
		EAutomationExpectedMessageFlags::Contains, 2);
	const FName CrankOnly[] = { Crank };
	const FName LiftOnly[] = { Lift };
	for (const bool bFoundAtTick : { true, false })
	{
		FRig R;
		if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
		{
			return false;
		}
		const FString Case = bFoundAtTick ? TEXT("found at the tick") : TEXT("found at the next claim");
		UObject*	  Holder = NewObject<UDataTable>(GetTransientPackage());
		TestTrue(*(Case + TEXT(": claimed")), R.Base->ClaimCommandOffsetLayer(Holder, CrankOnly));
		R.Base->SetMotorCommand(Crank, 0.2f);
		TestTrue(*(Case + TEXT(": an offset")), R.Base->SetMotorCommandOffset(Crank, 0.3f, Holder));
		R.Tick();
		TestTrue(*(Case + TEXT(": told the sum")), SameBits(R.Log->LastFor(Crank), 0.2f + 0.3f));
		const int32 Before = R.Log->CountFor(Crank);

		Holder->MarkAsGarbage();
		if (bFoundAtTick)
		{
			R.Tick();
		}
		else
		{
			TestTrue(*(Case + TEXT(": another may claim")), R.Base->ClaimCommandOffsetLayer(R.Other(), LiftOnly));
		}
		TestEqual(*(Case + TEXT(": the motor told once more")), R.Log->CountFor(Crank), Before + 1);
		TestEqual(*(Case + TEXT(": its baseline")), R.Log->LastFor(Crank), 0.2f);
		TestEqual(*(Case + TEXT(": no offset left")), R.Base->GetMotorCommandOffset(Crank), 0.0f);
		TestTrue(*(Case + TEXT(": its effective command its baseline")), SameBits(R.Effective(Crank), 0.2f));
		// No longer claimed: commanded at once, and nothing more at a tick.
		R.Base->SetMotorCommand(Crank, 0.3f);
		TestEqual(*(Case + TEXT(": commanded at once from then on")), R.Log->CountFor(Crank), Before + 2);
		R.Tick();
		TestEqual(*(Case + TEXT(": nothing more at the tick")), R.Log->CountFor(Crank), Before + 2);
		TestFalse(*(Case + TEXT(": the new holder does not have the old one's motors")), R.Base->SetMotorCommandOffset(Crank, 0.1f, R.Other()));
		if (bFoundAtTick)
		{
			TestTrue(*(Case + TEXT(": another may claim")), R.Base->ClaimCommandOffsetLayer(R.Other(), LiftOnly));
		}
		TestTrue(*(Case + TEXT(": and write its own")), R.Base->SetMotorCommandOffset(Lift, 0.0f, R.Other()));
		R.Base->ReleaseCommandOffsetLayer(R.Other());
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCommandOffsetStanceMotorStart,
	"Ramms.Base.CommandOffset.StanceMotorStart",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCommandOffsetStanceMotorStart::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	// Where the drive mode selector starts a stance ramp on a motor (RammsDrive::StanceMotorStart is the whole of
	// its rule), against a real base. The crank's joint is put where each case wants it.
	const float Tolerance = 0.05f;
	FRig		R;
	if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
	{
		return false;
	}
	const auto Start = [&](FName Motor, float Joint, const float* StanceCommanded) {
		R.Log->Joints.Add(Motor, Joint);
		return RammsDrive::StanceMotorStart(*R.Base, Motor, StanceCommanded, Tolerance);
	};
	const float Near = 0.32f, Far = 0.5f;

	// No offset: as it always was. What a stance last commanded while the joint is still there, else the joint.
	TestEqual(TEXT("no stance command: the joint"), Start(Crank, 0.3f, nullptr), static_cast<double>(0.3f));
	TestEqual(TEXT("the joint where a stance put it: that command"), Start(Crank, 0.3f, &Near), static_cast<double>(Near));
	TestEqual(TEXT("the joint elsewhere: the joint"), Start(Crank, 0.3f, &Far), static_cast<double>(0.3f));
	// Claimed with no offset on is still that rule: the base's baseline (0.2) is not what a stance commanded.
	const FName Ids[] = { Crank, Lift };
	R.Base->SetMotorCommand(Crank, 0.2f);
	TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	TestEqual(TEXT("claimed, no offset: the stance's command still"), Start(Crank, 0.3f, &Near), static_cast<double>(Near));
	TestEqual(TEXT("...or the joint"), Start(Crank, 0.3f, &Far), static_cast<double>(0.3f));

	// An offset on: told 0.5 on a baseline of 0.2. In the baseline's terms, whatever a stance last commanded.
	TestTrue(TEXT("an offset"), R.Base->SetMotorCommandOffset(Crank, 0.3f, R.Layer()));
	R.Tick();
	TestEqual(TEXT("offset on, the joint where it was told: the baseline"), Start(Crank, 0.5f, nullptr), 0.2, 1e-6);
	// (The old rule would take this stance command, which is where the joint is, and the ramp would add the offset
	// to it. The baseline and what the motor was told the wrong way round would give 0.8.)
	TestEqual(TEXT("...a stance command at the joint notwithstanding"), Start(Crank, 0.5f, &Far), 0.2, 1e-6);
	TestEqual(TEXT("...sagging within the tolerance: the baseline"), Start(Crank, 0.47f, nullptr), 0.2, 1e-6);
	TestEqual(TEXT("offset on, the joint elsewhere: the joint less the offset"), Start(Crank, 0.9f, nullptr), 0.6, 1e-6);
	// Against the range (+-1): told 1.0 on a baseline of 0.8, so the offset adds 0.2 of its 0.5.
	R.Base->SetMotorCommand(Crank, 0.8f);
	TestTrue(TEXT("an offset past the range"), R.Base->SetMotorCommandOffset(Crank, 0.5f, R.Layer()));
	R.Tick();
	TestEqual(TEXT("clamped, the joint at the range's end: the baseline"), Start(Crank, 1.0f, nullptr), 0.8, 1e-6);
	TestEqual(TEXT("clamped, the joint elsewhere: less what the offset added"), Start(Crank, 0.6f, nullptr), 0.4, 1e-6);
	// An offset waiting on a motor nothing has commanded has no baseline to start from: the old rule. (The lift is
	// reversed: its joint at 0.4 reads -0.4.)
	TestTrue(TEXT("an offset on a motor nothing commanded"), R.Base->SetMotorCommandOffset(Lift, 0.25f, R.Layer()));
	TestEqual(TEXT("...the joint, in the robot's sense"), Start(Lift, 0.4f, nullptr), static_cast<double>(-0.4f));
	// Back to no offset: the old rule again.
	TestTrue(TEXT("the offsets brought to zero"), R.Base->SetMotorCommandOffset(Crank, 0.0f, R.Layer()) && R.Base->SetMotorCommandOffset(Lift, 0.0f, R.Layer()));
	R.Tick();
	TestEqual(TEXT("the offset gone: the joint again"), Start(Crank, 0.3f, nullptr), static_cast<double>(0.3f));
	R.Base->ReleaseCommandOffsetLayer(R.Layer());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLinkageTargetUnderOffset,
	"Ramms.Linkage.TargetUnderOffset",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLinkageTargetUnderOffset::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	// The hips commanded to the low endpoint by something other than the linkage controller, which then holds no
	// target, and offset to the high one. Whoever asks the control surface for an axis' target and gets none starts
	// from where the axis reads (a stance ramp, a held key): the high endpoint, whose first command would write the
	// offsets into the baselines and step the leg by them. So the controller tells of the baselines' endpoint.
	FLinkageRig R;
	if (!TestTrue(TEXT("a linkage controller on a robot base"), R.IsReady()))
	{
		return false;
	}
	const FVector2D Low(1.0, -2.0), High(3.0, 0.5);
	bool			bLow = false, bHigh = false;
	const FVector2D HipsLow = R.HipsFor(Low, bLow);
	const FVector2D HipsHigh = R.HipsFor(High, bHigh);
	if (!TestTrue(TEXT("both endpoints are in reach"), bLow && bHigh))
	{
		return false;
	}
	IRammsControlContributor* Contributor = R.Linkage;
	const FName				  HeightId = RammsControlIds::Linkage::Height(R.Linkage->GetName());
	const FName				  TranslationId = RammsControlIds::Linkage::Translation(R.Linkage->GetName());
	FVector2D				  Target;

	R.Base->SetMotorCommand(HipA, static_cast<float>(HipsLow.X));
	R.Base->SetMotorCommand(HipB, static_cast<float>(HipsLow.Y));
	// No offset: no target, as ever (the answer the surface takes to mean the axis was let go).
	TestFalse(TEXT("hips commanded by another, no offset: no target told"), R.ReadsTarget(Target));
	const FName Ids[] = { HipA, HipB };
	TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	TestFalse(TEXT("...claimed with no offset on: none still"), R.ReadsTarget(Target));
	TestTrue(TEXT("offsets to the high endpoint"),
		R.Base->SetMotorCommandOffset(HipA, static_cast<float>(HipsHigh.X - HipsLow.X), R.Layer())
			&& R.Base->SetMotorCommandOffset(HipB, static_cast<float>(HipsHigh.Y - HipsLow.Y), R.Layer()));
	R.Tick();

	float Reads = 0.0f;
	TestTrue(TEXT("the height reads"), Contributor->ReadControl(HeightId, Reads));
	TestEqual(TEXT("...the live one, offset and all"), Reads, static_cast<float>(High.Y), 1e-3f);
	if (!TestTrue(TEXT("offsets on: the target told"), R.ReadsTarget(Target)))
	{
		return false;
	}
	TestTrue(TEXT("...is the baselines' endpoint"), Target.Equals(Low, 1e-3));

	// A ramp's first command, each axis to where it starts: the baselines stay where they were, so the leg does.
	TestTrue(TEXT("the fore/aft axis commanded to its start"), Contributor->ApplyControl(TranslationId, static_cast<float>(Target.X)));
	TestTrue(TEXT("the height axis commanded to its start"), Contributor->ApplyControl(HeightId, static_cast<float>(Target.Y)));
	TestEqual(TEXT("hip A's baseline where it was"), R.Baseline(HipA), static_cast<float>(HipsLow.X), 1e-4f);
	TestEqual(TEXT("hip B's baseline where it was"), R.Baseline(HipB), static_cast<float>(HipsLow.Y), 1e-4f);
	R.Tick();
	TestEqual(TEXT("hip A told what it was"), R.Log->LastFor(HipA), static_cast<float>(HipsHigh.X), 1e-4f);
	TestEqual(TEXT("hip B told what it was"), R.Log->LastFor(HipB), static_cast<float>(HipsHigh.Y), 1e-4f);

	// A hip let go has no command to tell of, offset or not (its offset goes with it).
	TestTrue(TEXT("the axis let go"), Contributor->ReleaseControl(HeightId));
	TestFalse(TEXT("let go: no target told"), R.ReadsTarget(Target));
	TestTrue(TEXT("the offsets brought to zero"),
		R.Base->SetMotorCommandOffset(HipA, 0.0f, R.Layer()) && R.Base->SetMotorCommandOffset(HipB, 0.0f, R.Layer()));
	R.Base->ReleaseCommandOffsetLayer(R.Layer());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLinkageSyncTarget,
	"Ramms.Linkage.SyncTarget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLinkageSyncTarget::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	FLinkageRig R;
	if (!TestTrue(TEXT("a linkage controller on a robot base"), R.IsReady()))
	{
		return false;
	}
	const FVector2D Low(1.0, -2.0), High(3.0, 0.5);
	bool			bLow = false, bHigh = false;
	const FVector2D HipsLow = R.HipsFor(Low, bLow);
	const FVector2D HipsHigh = R.HipsFor(High, bHigh);
	if (!TestTrue(TEXT("both endpoints are in reach"), bLow && bHigh))
	{
		return false;
	}

	// With a hip nothing commands there is nothing to take a target from.
	R.Linkage->SyncTargetToCommands();
	TestFalse(TEXT("no commands: no target"), R.HoldsTarget());

	// The controller commands the low endpoint; the layer raises the hips to the high one; the offsets are folded
	// into the baselines, as for a controller about to take the hips over.
	TestTrue(TEXT("commanded low"), R.Linkage->SetEndpointTarget(Low));
	const FName Ids[] = { HipA, HipB };
	TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	TestTrue(TEXT("offsets to the high endpoint"),
		R.Base->SetMotorCommandOffset(HipA, static_cast<float>(HipsHigh.X - HipsLow.X), R.Layer())
			&& R.Base->SetMotorCommandOffset(HipB, static_cast<float>(HipsHigh.Y - HipsLow.Y), R.Layer()));
	R.Tick();
	TestTrue(TEXT("the target is still the controller's own"), R.Linkage->GetLastTarget().Equals(Low, 1e-4));
	TestTrue(TEXT("folded"),
		R.Base->RebaseMotorCommand(HipA, R.Effective(HipA), R.Layer()) && R.Base->RebaseMotorCommand(HipB, R.Effective(HipB), R.Layer()));
	TestTrue(TEXT("...which the controller does not know of"), R.Linkage->GetLastTarget().Equals(Low, 1e-4));

	const int32 Before = R.Log->Pushes.Num();
	R.Linkage->SyncTargetToCommands();
	TestTrue(TEXT("synced: the target is where the commands now put the endpoint"), R.Linkage->GetLastTarget().Equals(High, 1e-3));
	TestTrue(TEXT("...held"), R.HoldsTarget());
	TestEqual(TEXT("...and nothing was commanded"), R.Log->Pushes.Num(), Before);
	TestEqual(TEXT("...the baselines untouched"), R.Baseline(HipA), static_cast<float>(HipsHigh.X), 1e-5f);
	R.Tick();
	TestEqual(TEXT("...nor at the tick"), R.Log->Pushes.Num(), Before);

	// A height command now keeps the fore/aft position the hips are really commanded to.
	TestTrue(TEXT("a height command"), R.Linkage->SetEndpointHeight(0.0f));
	TestEqual(TEXT("...keeps the synced fore/aft position"), R.Linkage->GetLastTarget().X, High.X, 1e-3);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLinkageHeldAxisUnderOffset,
	"Ramms.Linkage.HeldAxisUnderOffset",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLinkageHeldAxisUnderOffset::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	const FVector2D Low(1.0, -2.0), High(3.0, 0.5);
	// The hips commanded to the low endpoint by something other than the linkage controller (which then holds no
	// target), and standing at the high one: by an offset, or -- the case there has always been -- by not being
	// where they were commanded. A height command keeps the fore/aft position, a fore/aft command the height: with
	// an offset on, the baselines' (low), or the command would carry the offset into the baselines; without, the
	// live endpoint's (high), as before.
	for (const bool bOffset : { true, false })
	{
		for (const bool bHeightCommand : { true, false })
		{
			FLinkageRig R;
			if (!TestTrue(TEXT("a linkage controller on a robot base"), R.IsReady()))
			{
				return false;
			}
			bool			bLow = false, bHigh = false;
			const FVector2D HipsLow = R.HipsFor(Low, bLow);
			const FVector2D HipsHigh = R.HipsFor(High, bHigh);
			if (!TestTrue(TEXT("both endpoints are in reach"), bLow && bHigh))
			{
				return false;
			}
			R.Base->SetMotorCommand(HipA, static_cast<float>(HipsLow.X));
			R.Base->SetMotorCommand(HipB, static_cast<float>(HipsLow.Y));
			if (bOffset)
			{
				const FName Ids[] = { HipA, HipB };
				TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
				TestTrue(TEXT("offsets to the high endpoint"),
					R.Base->SetMotorCommandOffset(HipA, static_cast<float>(HipsHigh.X - HipsLow.X), R.Layer())
						&& R.Base->SetMotorCommandOffset(HipB, static_cast<float>(HipsHigh.Y - HipsLow.Y), R.Layer()));
				R.Tick();
			}
			else
			{
				R.Log->Joints.Add(HipA, static_cast<float>(HipsHigh.X));
				R.Log->Joints.Add(HipB, static_cast<float>(HipsHigh.Y));
			}
			TestTrue(TEXT("the live endpoint is the high one"), R.Linkage->GetCurrentEndpoint().Equals(High, 1e-3));
			// It holds no target of its own. Under the offset it tells of the baselines' endpoint all the same
			// (Ramms.Linkage.TargetUnderOffset); without, of none.
			FVector2D Told;
			TestEqual(TEXT("a target told only under an offset"), R.ReadsTarget(Told), bOffset);

			const FVector2D Kept = bOffset ? Low : High;
			const FString	Case = FString::Printf(TEXT("%s, a %s command"), bOffset ? TEXT("offset on") : TEXT("no offset"),
				  bHeightCommand ? TEXT("height") : TEXT("fore/aft"));
			  if (bHeightCommand)
			{
				TestTrue(*(Case + TEXT(": taken")), R.Linkage->SetEndpointHeight(-1.0f));
				TestEqual(*(Case + TEXT(": the height asked for")), R.Linkage->GetLastTarget().Y, -1.0, 1e-4);
				TestEqual(*(Case + (bOffset ? TEXT(": the baselines' fore/aft position kept") : TEXT(": the live fore/aft position kept"))),
					R.Linkage->GetLastTarget().X, Kept.X, 1e-3);
			}
			else
			{
				TestTrue(*(Case + TEXT(": taken")), R.Linkage->SetEndpointTranslation(2.0f));
				TestEqual(*(Case + TEXT(": the position asked for")), R.Linkage->GetLastTarget().X, 2.0, 1e-4);
				TestEqual(*(Case + (bOffset ? TEXT(": the baselines' height kept") : TEXT(": the live height kept"))),
					R.Linkage->GetLastTarget().Y, Kept.Y, 1e-3);
			}
			if (bOffset)
			{
				// Told at the tick with the offsets still on top: the hips' offsets are the layer's, not the command's.
				TestEqual(*(Case + TEXT(": the offset stands")), R.Base->GetMotorCommandOffset(HipA), static_cast<float>(HipsHigh.X - HipsLow.X));
				TestTrue(TEXT("the offsets brought to zero"),
					R.Base->SetMotorCommandOffset(HipA, 0.0f, R.Layer()) && R.Base->SetMotorCommandOffset(HipB, 0.0f, R.Layer()));
				R.Base->ReleaseCommandOffsetLayer(R.Layer());
			}
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsGen1LiftSyncTargets,
	"Ramms.Gen1Lift.SyncTargets",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsGen1LiftSyncTargets::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	// The lift controller's own six motors, each a registry motor by its constraint's name.
	const FName				Arm(TEXT("toy_front_arm")), Plate(TEXT("toy_plate_l"));
	const FName				ArmConstraint(TEXT("front_caster_swing_arm")), PlateConstraint(TEXT("dw_main_plate_l"));
	TArray<FRammsMotorSpec> Specs;
	Specs.Add(MotorRow(TEXT("toy_front_arm"), ERammsActuatorType::Position, FVector2D::ZeroVector, TEXT("front_caster_swing_arm")));
	Specs.Add(MotorRow(TEXT("toy_rear_lift"), ERammsActuatorType::Position, FVector2D::ZeroVector, TEXT("rear_caster_swing_arm")));
	Specs.Add(MotorRow(TEXT("toy_elevator_l"), ERammsActuatorType::Position, FVector2D::ZeroVector, TEXT("motor_swing_arm_l")));
	Specs.Add(MotorRow(TEXT("toy_elevator_r"), ERammsActuatorType::Position, FVector2D::ZeroVector, TEXT("motor_swing_arm_r")));
	Specs.Add(MotorRow(TEXT("toy_plate_l"), ERammsActuatorType::Position, FVector2D::ZeroVector, TEXT("dw_main_plate_l")));
	Specs.Add(MotorRow(TEXT("toy_plate_r"), ERammsActuatorType::Position, FVector2D::ZeroVector, TEXT("dw_main_plate_r")));
	FRig R(Specs);
	if (!TestTrue(TEXT("a robot base on the recording backend"), R.IsReady()))
	{
		return false;
	}
	URammpGen1ControllerComponent* Lifts = NewObject<URammpGen1ControllerComponent>(R.Actor, TEXT("Lifts"));
	// No skeletal mesh here: every motor is the base's, and looking for constraints would only warn.
	if (FBoolProperty* AutoFind = FindFProperty<FBoolProperty>(URammpGen1ControllerComponent::StaticClass(), TEXT("bAutoFindConstraints")))
	{
		AutoFind->SetPropertyValue_InContainer(Lifts, false);
	}
	if (!TestTrue(TEXT("the lift controller begun play"), FRig::BeginPlayOf(*Lifts)) || !TestTrue(TEXT("...on the base"), Lifts->IsUsingRobotBase()))
	{
		return false;
	}
	const auto AngularOf = [&](FName Constraint) {
		for (const FAngularMotorConfig& M : Lifts->GetAngularMotors())
		{
			if (M.ConstraintName == Constraint)
			{
				return M;
			}
		}
		return FAngularMotorConfig();
	};
	const auto LinearOf = [&](FName Constraint) {
		for (const FLinearMotorConfig& M : Lifts->GetLinearMotors())
		{
			if (M.ConstraintName == Constraint)
			{
				return M;
			}
		}
		return FLinearMotorConfig();
	};

	// Both part way up a ramp to their targets: a tick of 0.02 s at their own speeds.
	Lifts->SetAngularMotorTarget(ArmConstraint, 10.0f);
	Lifts->SetLinearMotorTarget(PlateConstraint, 5.0f);
	Lifts->TickComponent(0.02f, LEVELTICK_All, nullptr);
	const float ArmAt = AngularOf(ArmConstraint).CurrentAngle;
	const float PlateAt = LinearOf(PlateConstraint).CurrentPosition;
	TestTrue(TEXT("the arm is on its way"), ArmAt > 0.0f && ArmAt < 10.0f);
	TestTrue(TEXT("the plate is on its way"), PlateAt > 0.0f && PlateAt < 5.0f);
	TestEqual(TEXT("the arm commanded where its ramp is"), R.Baseline(Arm), FMath::DegreesToRadians(ArmAt), 1e-6f);

	// The commands moved under it: an offset folded into each baseline.
	const FName Ids[] = { Arm, Plate };
	TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	TestTrue(TEXT("offsets"), R.Base->SetMotorCommandOffset(Arm, 0.2f, R.Layer()) && R.Base->SetMotorCommandOffset(Plate, 1.5f, R.Layer()));
	R.Tick();
	TestTrue(TEXT("folded"),
		R.Base->RebaseMotorCommand(Arm, R.Effective(Arm), R.Layer()) && R.Base->RebaseMotorCommand(Plate, R.Effective(Plate), R.Layer()));
	const float ArmCommand = R.Baseline(Arm);
	const float PlateCommand = R.Baseline(Plate);
	TestEqual(TEXT("the arm's command moved"), ArmCommand, FMath::DegreesToRadians(ArmAt) + 0.2f, 1e-6f);

	const int32 Before = R.Log->Pushes.Num();
	Lifts->SyncTargetsToCommands();
	const FAngularMotorConfig ArmNow = AngularOf(ArmConstraint);
	const FLinearMotorConfig  PlateNow = LinearOf(PlateConstraint);
	TestEqual(TEXT("synced: the arm's ramp is at its command"), ArmNow.CurrentAngle, FMath::RadiansToDegrees(ArmCommand), 1e-4f);
	TestEqual(TEXT("...going nowhere else"), ArmNow.TargetAngle, ArmNow.CurrentAngle);
	TestEqual(TEXT("...the plate's too"), PlateNow.CurrentPosition, PlateCommand, 1e-5f);
	TestEqual(TEXT("...going nowhere else"), PlateNow.TargetPosition, PlateNow.CurrentPosition);
	TestEqual(TEXT("...and nothing was commanded"), R.Log->Pushes.Num(), Before);
	// It no longer ramps on from where it was: its next ticks write nothing.
	Lifts->TickComponent(0.02f, LEVELTICK_All, nullptr);
	Lifts->TickComponent(0.02f, LEVELTICK_All, nullptr);
	R.Tick();
	TestEqual(TEXT("its ticks command nothing"), R.Log->Pushes.Num(), Before);
	TestEqual(TEXT("...the arm's command where it was put"), R.Baseline(Arm), ArmCommand);
	// The next lift command ramps from the command, not from where the controller last had the motor.
	Lifts->SetAngularMotorTarget(ArmConstraint, FMath::RadiansToDegrees(ArmCommand) + 5.0f);
	TestEqual(TEXT("a new target starts from the command"), R.Baseline(Arm), ArmCommand, 1e-6f);

	TestTrue(TEXT("the offsets brought to zero"), R.Base->SetMotorCommandOffset(Arm, 0.0f, R.Layer()) && R.Base->SetMotorCommandOffset(Plate, 0.0f, R.Layer()));
	R.Base->ReleaseCommandOffsetLayer(R.Layer());
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
