// Tests for what a controller's stick and mode button drive
// (Private/RammsControlInputResolve, RammsDrive::NextInCycle).
//
// A Stick binding's X on the surfaces the drive modes publish -- turn on a
// differential drive and on a curb crossing, strafe on a holonomic drive,
// ControlId where nothing is paired -- and its Z only where X does not already
// turn; a stick axis held through such a change waiting for centre; and the
// modes a mode button steps through.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "RammsControlIds.h"
#include "RammsControlInputResolve.h"
#include "RammsDriveHandOver.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	// Functions, not FName constants: see RammsControlIds.
	FName InputForward()
	{
		return RammsControlIds::Drive::Forward();
	}
	FName InputTurn()
	{
		return RammsControlIds::Drive::Turn();
	}
	FName InputStrafe()
	{
		return RammsControlIds::Drive::Strafe();
	}

	/** A continuous drive axis, paired with Pair (none: unpaired). */
	FRammsControlAxis InputDriveAxis(FName Id, FName Pair)
	{
		FRammsControlAxis Axis;
		Axis.Id = Id;
		Axis.Kind = ERammsControlKind::Continuous;
		Axis.PairedAxis = Pair;
		return Axis;
	}

	FRammsControlSurface InputSurface(std::initializer_list<FRammsControlAxis> Axes)
	{
		FRammsControlSurface Surface;
		for (const FRammsControlAxis& Axis : Axes)
		{
			Surface.Add(Axis);
		}
		return Surface;
	}

	/** The drive stick's binding (X turn, Y forward, Z turn) resolved on
	 *  Surface: what X, Y and Z feed, as "x|y|z" with ids joined by ",". */
	FString InputStickOn(const FRammsControlSurface& Surface)
	{
		TArray<FName> Ids[3] = { { InputTurn() }, { InputForward() }, { InputTurn() } };
		RammsControlInput::ResolveStick(Surface, InputForward(), Ids);
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsInputStickPairing,
	"Ramms.Input.Stick.Pairing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsInputStickPairing::RunTest(const FString& Parameters)
{
	// Each drive mode's surface, as it publishes it.
	const FRammsControlSurface Differential = InputSurface({ InputDriveAxis(InputForward(), InputTurn()), InputDriveAxis(InputTurn(), InputForward()) });
	const FRammsControlSurface Holonomic = InputSurface(
		{ InputDriveAxis(InputForward(), InputStrafe()), InputDriveAxis(InputStrafe(), InputForward()), InputDriveAxis(InputTurn(), NAME_None) });
	const FRammsControlSurface Curb = InputSurface(
		{ InputDriveAxis(InputForward(), InputTurn()), InputDriveAxis(InputTurn(), InputForward()), InputDriveAxis(InputStrafe(), NAME_None) });
	TestEqual(TEXT("differential: X turns, the twist does nothing"), InputStickOn(Differential), FString(TEXT("drive.turn|drive.forward|")));
	TestEqual(TEXT("holonomic: X strafes, the twist turns"), InputStickOn(Holonomic), FString(TEXT("drive.strafe|drive.forward|drive.turn")));
	TestEqual(TEXT("a curb crossing: X turns (and so pauses it), the twist does nothing"), InputStickOn(Curb), FString(TEXT("drive.turn|drive.forward|")));

	// Nothing paired with forward, or its partner not on the surface: X is
	// ControlId, and Z still only what X does not feed.
	TestEqual(TEXT("unpaired: X is ControlId"), InputStickOn(InputSurface({ InputDriveAxis(InputForward(), NAME_None), InputDriveAxis(InputTurn(), NAME_None) })),
		FString(TEXT("drive.turn|drive.forward|")));
	TestEqual(TEXT("partner not on the surface: X is ControlId"),
		InputStickOn(InputSurface({ InputDriveAxis(InputForward(), InputStrafe()), InputDriveAxis(InputTurn(), NAME_None) })), FString(TEXT("drive.turn|drive.forward|")));
	// No forward on the surface (the low-level mode): nothing re-routed.
	TestEqual(TEXT("no drive axes"), InputStickOn(FRammsControlSurface()), FString(TEXT("drive.turn|drive.forward|")));

	// Z drops only what X or Y feed: a twist bound elsewhere keeps its control.
	TArray<FName> Ids[3] = { { InputTurn() }, { InputForward() }, { InputTurn(), FName(TEXT("camera.orbit_yaw")) } };
	RammsControlInput::ResolveStick(Holonomic, InputForward(), Ids);
	TestEqual(TEXT("holonomic, Z also bound to the camera: both kept"), Ids[2].Num(), 2);
	RammsControlInput::ResolveStick(Differential, InputForward(), Ids);
	TestTrue(TEXT("differential, Z also bound to the camera: the camera kept, the turn dropped"),
		Ids[2].Num() == 1 && Ids[2][0] == FName(TEXT("camera.orbit_yaw")));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsInputStickCentre,
	"Ramms.Input.Stick.Centre",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsInputStickCentre::RunTest(const FString& Parameters)
{
	// Not waiting: the stick's value goes straight through.
	bool bAwait = false;
	TestEqual(TEXT("not waiting: passed through"), RammsControlInput::StepCentre(bAwait, 0.8f), 0.8f);

	// Held at 0.8 through the change: nothing, until it reads within the band;
	// then nothing that tick either, and the stick from the next.
	bAwait = true;
	for (const float V : { 0.8f, 0.6f, 0.3f, -0.4f })
	{
		TestEqual(FString::Printf(TEXT("waiting, at %.1f: nothing"), V), RammsControlInput::StepCentre(bAwait, V), 0.0f);
		TestTrue(FString::Printf(TEXT("waiting, at %.1f: still waiting"), V), bAwait);
	}
	TestEqual(TEXT("back to centre: nothing yet"), RammsControlInput::StepCentre(bAwait, 0.05f), 0.0f);
	TestFalse(TEXT("back to centre: done waiting"), bAwait);
	TestEqual(TEXT("then pushed: the stick"), RammsControlInput::StepCentre(bAwait, 0.7f), 0.7f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveCycleMode,
	"Ramms.Drive.CycleMode",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveCycleMode::RunTest(const FString& Parameters)
{
	using RammsDrive::NextInCycle;
	const TArray<FName> Offered = { FName(TEXT("holonomic")), FName(TEXT("differential")), FName(TEXT("low_level")), FName(TEXT("curb")) };
	const TArray<FName> Riding = { FName(TEXT("differential")), FName(TEXT("holonomic")) };
	const TArray<FName> None;

	// No list: every mode in the offered order, wrapping, and none active
	// starting at the first -- the selector's cycle as it always was.
	TestEqual(TEXT("no list, from holonomic"), NextInCycle(Offered, None, 0), 1);
	TestEqual(TEXT("no list, from curb: wraps"), NextInCycle(Offered, None, 3), 0);
	TestEqual(TEXT("no list, none active"), NextInCycle(Offered, None, INDEX_NONE), 0);

	// The list: its order, not the offered one, and nothing outside it.
	TestEqual(TEXT("riding modes, from differential"), NextInCycle(Offered, Riding, 1), 0);
	TestEqual(TEXT("riding modes, from holonomic"), NextInCycle(Offered, Riding, 0), 1);
	TestEqual(TEXT("riding modes, from a curb crossing: refused"), NextInCycle(Offered, Riding, 3), INDEX_NONE);
	TestEqual(TEXT("riding modes, from low level: refused"), NextInCycle(Offered, Riding, 2), INDEX_NONE);
	TestEqual(TEXT("riding modes, none active: refused"), NextInCycle(Offered, Riding, INDEX_NONE), INDEX_NONE);

	// Ids the robot does not have are skipped; one mode is no cycle.
	const TArray<FName> WithMissing = { FName(TEXT("tracked")), FName(TEXT("differential")), FName(TEXT("holonomic")) };
	TestEqual(TEXT("a missing id skipped"), NextInCycle(Offered, WithMissing, 0), 1);
	TestEqual(TEXT("one mode on the robot: nothing to cycle"), NextInCycle({ FName(TEXT("differential")) }, Riding, 0), INDEX_NONE);
	TestEqual(TEXT("one listed mode: nothing to cycle"), NextInCycle(Offered, { FName(TEXT("differential")) }, 1), INDEX_NONE);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
