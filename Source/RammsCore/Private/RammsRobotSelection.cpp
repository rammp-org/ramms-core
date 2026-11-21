// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRobotSelection.h"
#include "RammsRobotDefinition.h"
#include "AssetRegistry/AssetData.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "AssetRegistry/IAssetRegistry.h"
#include "Engine/AssetManager.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsRobotSelect, Log, All);

namespace
{
	TAutoConsoleVariable<FString> CVarRammsRobot(
		TEXT("ramms.Robot"),
		TEXT(""),
		TEXT("The robot the next level started through ARammsRobotGameMode spawns: a robot id or alias (rammp_gen1, ")
			TEXT("rammp_gen2, Holonomic), a pawn class path (/Game/.../BP_X.BP_X_C), `ask` for the picker, or empty for ")
				TEXT("the level's or the project's default. A ?Robot= option on the map URL wins over it. -RammsRobot=<id> sets ")
					TEXT("it at startup."),
		ECVF_Default);

	const TCHAR* SourceName(ERammsRobotChoiceSource Source)
	{
		switch (Source)
		{
			case ERammsRobotChoiceSource::Url:
				return TEXT("the map URL's ?Robot=");
			case ERammsRobotChoiceSource::ConsoleVariable:
				return TEXT("ramms.Robot");
			case ERammsRobotChoiceSource::GameModeDefault:
				return TEXT("the level's game mode default");
			case ERammsRobotChoiceSource::ProjectDefault:
				return TEXT("the project default");
			case ERammsRobotChoiceSource::OnlyRobot:
				return TEXT("the only robot available");
			case ERammsRobotChoiceSource::Picker:
				return TEXT("nothing chosen");
			default:
				return TEXT("nowhere");
		}
	}

	URammsRobotDefinition* Find(const TArray<URammsRobotDefinition*>& Definitions, const FString& Words)
	{
		for (URammsRobotDefinition* Def : Definitions)
		{
			if (Def && Def->Matches(Words))
			{
				return Def;
			}
		}
		return nullptr;
	}

	FString Available(const TArray<URammsRobotDefinition*>& Definitions)
	{
		TArray<FString> Ids;
		for (const URammsRobotDefinition* Def : Definitions)
		{
			if (Def && !Def->RobotId.IsNone())
			{
				Ids.Add(Def->RobotId.ToString());
			}
		}
		return Ids.Num() ? FString::Join(Ids, TEXT(", ")) : FString(TEXT("none"));
	}

	/** A play world: the PIE or game world (the first, as the bridges take it). */
	UWorld* PlayWorld()
	{
		if (!GEngine)
		{
			return nullptr;
		}
		for (const FWorldContext& Context : GEngine->GetWorldContexts())
		{
			if ((Context.WorldType == EWorldType::PIE || Context.WorldType == EWorldType::Game) && Context.World())
			{
				return Context.World();
			}
		}
		return nullptr;
	}

	FAutoConsoleCommandWithWorldAndArgs CmdRestart(
		TEXT("ramms.Robot.Restart"),
		TEXT("Open the current map again with another robot: ramms.Robot.Restart <id|alias|class path|ask> (without one, ramms.Robot's)."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World) {
			UWorld*		  Target = (World && World->IsGameWorld()) ? World : PlayWorld();
			const FString Robot = Args.Num() ? Args[0] : URammsRobotCatalog::GetConsoleRobot();
			if (!URammsRobotCatalog::RestartWithRobot(Target, Robot))
			{
				UE_LOG(LogRammsRobotSelect, Warning, TEXT("[RobotSelect] ramms.Robot.Restart: no play world to restart"));
			}
		}));
} // namespace

FString FRammsRobotChoice::GetId() const
{
	if (Definition && !Definition->RobotId.IsNone())
	{
		return Definition->RobotId.ToString();
	}
	return ClassPath;
}

FString FRammsRobotChoice::Describe() const
{
	if (Definition)
	{
		return FString::Printf(TEXT("'%s' (%s), from %s"), *GetId(), *Definition->GetDisplayNameOrId().ToString(), SourceName(Source));
	}
	if (!ClassPath.IsEmpty())
	{
		return FString::Printf(TEXT("pawn class %s, from %s"), *ClassPath, SourceName(Source));
	}
	return Error.IsEmpty() ? FString::Printf(TEXT("none (the picker), from %s"), SourceName(Source)) : FString::Printf(TEXT("none (the picker): %s"), *Error);
}

TArray<URammsRobotDefinition*> URammsRobotCatalog::GetDefinitions(bool bIncludeHidden)
{
	TArray<URammsRobotDefinition*> Out;
	IAssetRegistry&				   Registry = FAssetRegistryModule::GetRegistry();
	if (Registry.IsLoadingAssets())
	{
		// Only while an editor or an uncooked game is still scanning at startup.
		Registry.WaitForCompletion();
	}
	TArray<FAssetData> Assets;
	Registry.GetAssetsByClass(URammsRobotDefinition::StaticClass()->GetClassPathName(), Assets, /*bSearchSubClasses=*/true);
	// And the asset manager's RammsRobot primary assets: the folders a project or a plugin lists for the type
	// (PrimaryAssetTypesToScan) are scanned at startup even where the registry is not (a commandlet).
	if (UAssetManager* AssetManager = UAssetManager::GetIfInitialized())
	{
		TArray<FAssetData> Primary;
		AssetManager->GetPrimaryAssetDataList(URammsRobotDefinition::AssetType, Primary);
		Assets.Append(Primary);
	}
	for (const FAssetData& Asset : Assets)
	{
		URammsRobotDefinition* Def = Cast<URammsRobotDefinition>(Asset.GetAsset());
		if (Def && (bIncludeHidden || !Def->bHidden))
		{
			Out.AddUnique(Def);
		}
	}
	Out.Sort([](const URammsRobotDefinition& A, const URammsRobotDefinition& B) {
		return A.SortOrder != B.SortOrder ? A.SortOrder < B.SortOrder : A.GetDisplayNameOrId().CompareTo(B.GetDisplayNameOrId()) < 0;
	});
	return Out;
}

URammsRobotDefinition* URammsRobotCatalog::FindDefinition(const FString& IdOrAlias)
{
	return Find(GetDefinitions(/*bIncludeHidden=*/true), IdOrAlias);
}

bool URammsRobotCatalog::IsClassPath(const FString& Words)
{
	// A long package path to an object: /Root/Folder/Asset.Object (a generated
	// class ends in _C; a native one is /Script/Module.Class).
	const FString W = Words.TrimStartAndEnd();
	return W.StartsWith(TEXT("/")) && W.Contains(TEXT(".")) && !W.Contains(TEXT(" "));
}

FRammsRobotChoice URammsRobotCatalog::Resolve(const FRammsRobotResolveInputs& In)
{
	FRammsRobotChoice Out;
	auto			  Pick = [&In](const FString& Words, ERammsRobotChoiceSource Source, FRammsRobotChoice& Choice) {
		 Choice.Asked = Words;
		 Choice.Source = Source;
		 if (URammsRobotDefinition* Def = Find(In.Definitions, Words))
		 {
			 Choice.Definition = Def;
			 return true;
		 }
		 if (IsClassPath(Words))
		 {
			 Choice.ClassPath = Words.TrimStartAndEnd();
			 return true;
		 }
		 return false;
	};

	// Explicit: the URL, then the console variable. Never a silent fallback.
	const TPair<FString, ERammsRobotChoiceSource> Explicit[] = {
		{ In.UrlOption.TrimStartAndEnd(), ERammsRobotChoiceSource::Url },
		{ In.ConsoleValue.TrimStartAndEnd(), ERammsRobotChoiceSource::ConsoleVariable },
	};
	for (const TPair<FString, ERammsRobotChoiceSource>& E : Explicit)
	{
		if (E.Key.IsEmpty())
		{
			continue;
		}
		if (E.Key.Equals(TEXT("ask"), ESearchCase::IgnoreCase))
		{
			Out.Asked = E.Key;
			Out.Source = E.Value;
			return Out;
		}
		if (Pick(E.Key, E.Value, Out))
		{
			return Out;
		}
		Out.Error = FString::Printf(TEXT("unknown robot '%s' (from %s); available: %s"), *E.Key, SourceName(E.Value), *Available(In.Definitions));
		return Out;
	}

	// Defaults: passed over when they match nothing.
	const TPair<FString, ERammsRobotChoiceSource> Defaults[] = {
		{ In.GameModeDefault.TrimStartAndEnd(), ERammsRobotChoiceSource::GameModeDefault },
		{ In.ProjectDefault.TrimStartAndEnd(), ERammsRobotChoiceSource::ProjectDefault },
	};
	for (const TPair<FString, ERammsRobotChoiceSource>& D : Defaults)
	{
		if (D.Key.IsEmpty())
		{
			continue;
		}
		FRammsRobotChoice Choice;
		if (Pick(D.Key, D.Value, Choice))
		{
			Choice.Warnings = Out.Warnings;
			return Choice;
		}
		Out.Warnings.Add(FString::Printf(TEXT("%s '%s' matches no robot (available: %s)"), SourceName(D.Value), *D.Key, *Available(In.Definitions)));
	}

	TArray<URammsRobotDefinition*> Offered;
	for (URammsRobotDefinition* Def : In.Definitions)
	{
		if (Def && !Def->bHidden)
		{
			Offered.Add(Def);
		}
	}
	if (Offered.Num() == 1)
	{
		Out.Definition = Offered[0];
		Out.Source = ERammsRobotChoiceSource::OnlyRobot;
		Out.Asked.Reset();
		return Out;
	}
	Out.Source = ERammsRobotChoiceSource::Picker;
	return Out;
}

FRammsRobotChoice URammsRobotCatalog::ResolveForWorld(const FString& Options, const FString& GameModeDefault)
{
	FRammsRobotResolveInputs In;
	In.UrlOption = UGameplayStatics::ParseOption(Options, TEXT("Robot"));
	In.ConsoleValue = GetConsoleRobot();
	In.GameModeDefault = GameModeDefault;
	In.ProjectDefault = GetDefault<URammsRobotSettings>()->DefaultRobotId;
	In.Definitions = GetDefinitions(/*bIncludeHidden=*/true);
	return Resolve(In);
}

FString URammsRobotCatalog::GetConsoleRobot()
{
	return CVarRammsRobot.GetValueOnGameThread();
}

void URammsRobotCatalog::SetConsoleRobot(const FString& IdOrAlias)
{
	CVarRammsRobot->Set(*IdOrAlias, ECVF_SetByConsole);
}

void URammsRobotCatalog::SeedConsoleRobotFromCommandLine()
{
	FString Robot;
	if (FParse::Value(FCommandLine::Get(), TEXT("RammsRobot="), Robot))
	{
		CVarRammsRobot->Set(*Robot, ECVF_SetByCommandline);
		UE_LOG(LogRammsRobotSelect, Log, TEXT("[RobotSelect] -RammsRobot=%s: ramms.Robot set"), *Robot);
	}
}

FString URammsRobotCatalog::GetMapPackagePath(const UWorld* World)
{
	if (!World)
	{
		return FString();
	}
	return UWorld::RemovePIEPrefix(World->GetOutermost()->GetName());
}

bool URammsRobotCatalog::RestartWithRobot(UWorld* World, const FString& IdOrAlias)
{
	return OpenMapWithRobot(World, GetMapPackagePath(World), IdOrAlias);
}

bool URammsRobotCatalog::OpenMapWithRobot(UWorld* World, const FString& MapPath, const FString& IdOrAlias)
{
	FString Map = MapPath.TrimStartAndEnd();
	if (!World || Map.IsEmpty() || !GEngine)
	{
		return false;
	}
	// Travelling unloads the world before it loads the map, so a map that is not
	// there would leave the game on the default map: refuse it here, as the
	// engine's own `open` does.
	if (!GEngine->MakeSureMapNameIsValid(Map))
	{
		UE_LOG(LogRammsRobotSelect, Warning, TEXT("[RobotSelect] no map %s: not opening it"), *MapPath);
		return false;
	}
	const FString Robot = IdOrAlias.TrimStartAndEnd();
	UE_LOG(LogRammsRobotSelect, Log, TEXT("[RobotSelect] opening %s with robot '%s'"), *Map, *Robot);
	UGameplayStatics::OpenLevel(World, FName(*Map), /*bAbsolute=*/true, Robot.IsEmpty() ? FString() : FString::Printf(TEXT("Robot=%s"), *Robot));
	return true;
}
