// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Subsystems/WorldSubsystem.h"
#include "RammsRobotSelection.generated.h"

class URammsRobotDefinition;
class UWorld;

/** Where a level's robot came from (URammsRobotCatalog::Resolve). */
UENUM(BlueprintType)
enum class ERammsRobotChoiceSource : uint8
{
	None,
	/** ?Robot=<id> on the map URL (a picker's reload, OpenMapWithRobot, -game Map?Robot=). */
	Url,
	/** The ramms.Robot console variable (-RammsRobot=<id> sets it at startup). */
	ConsoleVariable,
	/** The level's game mode (ARammsRobotGameMode::DefaultRobotId). */
	GameModeDefault,
	/** The project's (URammsRobotSettings::DefaultRobotId, a plugin's Game.ini may set it). */
	ProjectDefault,
	/** The one robot there is to start. */
	OnlyRobot,
	/** Nothing chosen: the picker. */
	Picker,
};

/** The robot a level starts with, and why. */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsRobotChoice
{
	GENERATED_BODY()

	/** The robot's definition, when it has one. */
	UPROPERTY(BlueprintReadOnly, Category = "Robot")
	TObjectPtr<URammsRobotDefinition> Definition;

	/** A pawn class asked for by its path, with no definition
	 *  (/Game/MyRobot/BP_MyRobot.BP_MyRobot_C). */
	UPROPERTY(BlueprintReadOnly, Category = "Robot")
	FString ClassPath;

	UPROPERTY(BlueprintReadOnly, Category = "Robot")
	ERammsRobotChoiceSource Source = ERammsRobotChoiceSource::None;

	/** The words it was asked for by (an id, an alias, a class path, `ask`). */
	UPROPERTY(BlueprintReadOnly, Category = "Robot")
	FString Asked;

	/** An explicit choice that matched nothing: why (the picker is shown, nothing
	 *  is spawned in its place). */
	UPROPERTY(BlueprintReadOnly, Category = "Robot")
	FString Error;

	/** Defaults that matched nothing and were passed over. */
	UPROPERTY(BlueprintReadOnly, Category = "Robot")
	TArray<FString> Warnings;

	bool HasRobot() const { return Definition != nullptr || !ClassPath.IsEmpty(); }
	bool ShowsPicker() const { return !HasRobot(); }

	/** The robot's id, else its class path; empty with none. */
	FString GetId() const;

	/** One line: the robot and where it came from. */
	FString Describe() const;
};

/** What Resolve chooses from (gathered by ResolveForWorld; tests give their own). */
struct RAMMSCORE_API FRammsRobotResolveInputs
{
	FString						   UrlOption;
	FString						   ConsoleValue;
	FString						   GameModeDefault;
	FString						   ProjectDefault;
	TArray<URammsRobotDefinition*> Definitions;
};

/**
 * The project's robot settings ([/Script/RammsCore.RammsRobotSettings] in
 * DefaultGame.ini). A plugin with robots of its own can set the default from
 * its Config/Game.ini, which is layered in only while the plugin is mounted.
 */
UCLASS(Config = Game, meta = (DisplayName = "Ramms Robots"))
class RAMMSCORE_API URammsRobotSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	/** The robot a level starts with when nothing else chooses one (an id, an
	 *  alias or a pawn class path). Read from config; not edited here, so the
	 *  editor never writes a plugin's choice into the project's ini. */
	UPROPERTY(Config, VisibleAnywhere, Category = "Robots")
	FString DefaultRobotId;

	virtual FName GetCategoryName() const override { return FName(TEXT("Plugins")); }
};

/** Finds robot definitions and chooses a level's robot. */
UCLASS()
class RAMMSCORE_API URammsRobotCatalog : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** Every robot definition the asset registry knows (the mounted plugins'
	 *  included), in picker order: SortOrder, then name. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Robots")
	static TArray<URammsRobotDefinition*> GetDefinitions(bool bIncludeHidden = false);

	/** The definition with this id or alias (hidden ones included), or null. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Robots")
	static URammsRobotDefinition* FindDefinition(const FString& IdOrAlias);

	/** Words that name a class by its path (/Game/.../BP_X.BP_X_C). */
	static bool IsClassPath(const FString& Words);

	/**
	 * The robot to start, from (first that decides):
	 *  1. the URL option, 2. the console variable -- each explicit: `ask` shows
	 *     the picker, and one that matches nothing shows it with an error,
	 *     never falling back to a default;
	 *  3. the game mode's default, 4. the project's -- one that matches nothing
	 *     is passed over with a warning;
	 *  5. the only robot there is (hidden ones not counted);
	 *  6. the picker.
	 * An id or alias picks a definition; a class path is taken as given.
	 */
	static FRammsRobotChoice Resolve(const FRammsRobotResolveInputs& In);

	/** Resolve with this world's inputs: ?Robot= in Options, ramms.Robot, the
	 *  game mode's default and the settings. */
	static FRammsRobotChoice ResolveForWorld(const FString& Options, const FString& GameModeDefault);

	/** The ramms.Robot console variable. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Robots")
	static FString GetConsoleRobot();

	UFUNCTION(BlueprintCallable, Category = "Ramms|Robots")
	static void SetConsoleRobot(const FString& IdOrAlias);

	/** -RammsRobot=<id> on the command line into ramms.Robot (at startup). */
	static void SeedConsoleRobotFromCommandLine();

	/** The world's map as a package path, without a PIE prefix
	 *  (/Game/Maps/Curb/Map_Curb_S0_URL). */
	static FString GetMapPackagePath(const UWorld* World);

	/** Open World's map again with ?Robot=<IdOrAlias> (the robot takes effect
	 *  as the level starts). False with no world. */
	static bool RestartWithRobot(UWorld* World, const FString& IdOrAlias);

	/** Open MapPath with ?Robot=<IdOrAlias> in World's game. False with no world
	 *  or a map that is not there (nothing is opened then). */
	static bool OpenMapWithRobot(UWorld* World, const FString& MapPath, const FString& IdOrAlias);
};

/** The robot a world started with (set by its game mode as it starts). */
UCLASS()
class RAMMSCORE_API URammsRobotSelectionSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	void SetActiveChoice(const FRammsRobotChoice& Choice) { Active = Choice; }

	UFUNCTION(BlueprintCallable, Category = "Ramms|Robots")
	FRammsRobotChoice GetActiveChoice() const { return Active; }

	UFUNCTION(BlueprintCallable, Category = "Ramms|Robots")
	FString GetActiveRobotId() const { return Active.GetId(); }

private:
	UPROPERTY(Transient)
	FRammsRobotChoice Active;
};
