// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"
#include "RammsRobotDefinition.h"
#include "RammsRobotSelection.h"
#include "UObject/Package.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	URammsRobotDefinition* RobotSelectionTestRobot(const TCHAR* Id, std::initializer_list<const TCHAR*> Aliases = {}, bool bHidden = false)
	{
		URammsRobotDefinition* Def = NewObject<URammsRobotDefinition>(GetTransientPackage());
		Def->RobotId = FName(Id);
		for (const TCHAR* Alias : Aliases)
		{
			Def->Aliases.Add(FName(Alias));
		}
		Def->bHidden = bHidden;
		return Def;
	}

	struct FRobotSelectionTestSet
	{
		URammsRobotDefinition*	 Gen1 = RobotSelectionTestRobot(TEXT("rammp_gen1"), { TEXT("Gen1") });
		URammsRobotDefinition*	 Gen2 = RobotSelectionTestRobot(TEXT("rammp_gen2"), { TEXT("Holonomic"), TEXT("Gen2") });
		URammsRobotDefinition*	 Old = RobotSelectionTestRobot(TEXT("linkage"), {}, /*bHidden=*/true);
		FRammsRobotResolveInputs Inputs() const
		{
			FRammsRobotResolveInputs In;
			In.Definitions = { Gen1, Gen2, Old };
			return In;
		}
	};
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsRobotSelectionOrder, "Ramms.Robots.Resolve.Order", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsRobotSelectionOrder::RunTest(const FString& Parameters)
{
	// Each source decides when the ones before it say nothing: the URL, the
	// console variable, the game mode's default, the project's.
	const FRobotSelectionTestSet T;
	FRammsRobotResolveInputs	 In = T.Inputs();
	In.UrlOption = TEXT("rammp_gen1");
	In.ConsoleValue = TEXT("rammp_gen2");
	In.GameModeDefault = TEXT("rammp_gen2");
	In.ProjectDefault = TEXT("rammp_gen2");
	FRammsRobotChoice C = URammsRobotCatalog::Resolve(In);
	TestTrue(TEXT("the URL wins"), C.Definition == T.Gen1 && C.Source == ERammsRobotChoiceSource::Url);

	In.UrlOption.Reset();
	In.ConsoleValue = TEXT("rammp_gen1");
	C = URammsRobotCatalog::Resolve(In);
	TestTrue(TEXT("then ramms.Robot"), C.Definition == T.Gen1 && C.Source == ERammsRobotChoiceSource::ConsoleVariable);

	In.ConsoleValue.Reset();
	In.GameModeDefault = TEXT("rammp_gen1");
	C = URammsRobotCatalog::Resolve(In);
	TestTrue(TEXT("then the game mode's default"), C.Definition == T.Gen1 && C.Source == ERammsRobotChoiceSource::GameModeDefault);

	In.GameModeDefault.Reset();
	C = URammsRobotCatalog::Resolve(In);
	TestTrue(TEXT("then the project's"), C.Definition == T.Gen2 && C.Source == ERammsRobotChoiceSource::ProjectDefault);

	In.ProjectDefault.Reset();
	C = URammsRobotCatalog::Resolve(In);
	TestTrue(TEXT("two robots offered and nothing chosen: the picker"), C.ShowsPicker() && C.Source == ERammsRobotChoiceSource::Picker && C.Error.IsEmpty());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsRobotSelectionAliases, "Ramms.Robots.Resolve.Aliases", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsRobotSelectionAliases::RunTest(const FString& Parameters)
{
	const FRobotSelectionTestSet T;
	FRammsRobotResolveInputs	 In = T.Inputs();
	In.ConsoleValue = TEXT("holonomic");
	FRammsRobotChoice C = URammsRobotCatalog::Resolve(In);
	TestTrue(TEXT("an alias, without regard to case"), C.Definition == T.Gen2);
	TestEqual(TEXT("reported by the robot's own id"), C.GetId(), FString(TEXT("rammp_gen2")));
	In.ConsoleValue = TEXT("  RAMMP_GEN1 ");
	C = URammsRobotCatalog::Resolve(In);
	TestTrue(TEXT("an id, trimmed, without regard to case"), C.Definition == T.Gen1);
	In.ConsoleValue = TEXT("linkage");
	C = URammsRobotCatalog::Resolve(In);
	TestTrue(TEXT("a hidden robot when asked for by id"), C.Definition == T.Old);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsRobotSelectionExplicitFailure, "Ramms.Robots.Resolve.ExplicitFailure", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsRobotSelectionExplicitFailure::RunTest(const FString& Parameters)
{
	// An explicit choice that matches nothing shows the picker with an error:
	// never a silent fallback to a default.
	const FRobotSelectionTestSet T;
	FRammsRobotResolveInputs	 In = T.Inputs();
	In.UrlOption = TEXT("rammp_gen9");
	In.ConsoleValue = TEXT("rammp_gen2");
	In.ProjectDefault = TEXT("rammp_gen1");
	FRammsRobotChoice C = URammsRobotCatalog::Resolve(In);
	TestTrue(TEXT("no robot"), C.ShowsPicker());
	TestTrue(TEXT("an error naming it and the robots there are"),
		C.Error.Contains(TEXT("rammp_gen9")) && C.Error.Contains(TEXT("rammp_gen1")) && C.Error.Contains(TEXT("rammp_gen2")));
	TestTrue(TEXT("from the URL"), C.Source == ERammsRobotChoiceSource::Url);

	In.UrlOption = TEXT("ask");
	C = URammsRobotCatalog::Resolve(In);
	TestTrue(TEXT("`ask`: the picker, without an error"), C.ShowsPicker() && C.Error.IsEmpty());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsRobotSelectionDefaults, "Ramms.Robots.Resolve.Defaults", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsRobotSelectionDefaults::RunTest(const FString& Parameters)
{
	// A default that matches nothing (a plugin's robot, the plugin not mounted)
	// is passed over with a warning.
	const FRobotSelectionTestSet T;
	FRammsRobotResolveInputs	 In = T.Inputs();
	In.GameModeDefault = TEXT("rammp_gen9");
	In.ProjectDefault = TEXT("rammp_gen1");
	FRammsRobotChoice C = URammsRobotCatalog::Resolve(In);
	TestTrue(TEXT("the next default decides"), C.Definition == T.Gen1 && C.Source == ERammsRobotChoiceSource::ProjectDefault);
	TestEqual(TEXT("the one passed over is warned of"), C.Warnings.Num(), 1);

	// The only robot there is (a hidden one not counted).
	FRammsRobotResolveInputs One;
	One.Definitions = { T.Gen1, T.Old };
	C = URammsRobotCatalog::Resolve(One);
	TestTrue(TEXT("the only robot offered"), C.Definition == T.Gen1 && C.Source == ERammsRobotChoiceSource::OnlyRobot);

	FRammsRobotResolveInputs None;
	C = URammsRobotCatalog::Resolve(None);
	TestTrue(TEXT("no robots at all: the picker"), C.ShowsPicker());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsRobotSelectionClassPath, "Ramms.Robots.Resolve.ClassPath", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsRobotSelectionClassPath::RunTest(const FString& Parameters)
{
	// A pawn class asked for by its path, with no definition (a design under
	// test), is taken as given.
	const FRobotSelectionTestSet T;
	FRammsRobotResolveInputs	 In = T.Inputs();
	In.UrlOption = TEXT("/Game/MyRobot/BP_MyRobot.BP_MyRobot_C");
	const FRammsRobotChoice C = URammsRobotCatalog::Resolve(In);
	TestTrue(TEXT("taken as a class path"), C.Definition == nullptr && C.ClassPath == In.UrlOption && C.HasRobot());
	TestTrue(TEXT("a class path"), URammsRobotCatalog::IsClassPath(TEXT("/Script/Engine.DefaultPawn")));
	TestFalse(TEXT("an id is not"), URammsRobotCatalog::IsClassPath(TEXT("rammp_gen1")));
	TestFalse(TEXT("a package path alone is not"), URammsRobotCatalog::IsClassPath(TEXT("/Game/MyRobot/BP_MyRobot")));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
