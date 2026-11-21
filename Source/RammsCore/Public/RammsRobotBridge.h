// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RammsRobotBridge.generated.h"

/**
 * The robot choice for Remote Control and Python (PUT /remote/object/call on
 * /Script/RammsCore.Default__RammsRobotBridge): which robots there are, which
 * one a play world started with, and starting a level with another.
 *
 * Choose by id or alias (rammp_gen1, rammp_gen2, Holonomic) or a pawn class
 * path; never by a picker's index, which moves as plugins are mounted.
 */
UCLASS()
class RAMMSCORE_API URammsRobotBridge : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** JSON: [{id, name, aliases, tags, hidden, pawn, available, why, root, active}]
	 *  -- every definition, hidden ones included (available: its pawn class
	 *  exists; root: the content root it comes from as a path, /Game or a
	 *  plugin's /MyRobots). */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote|Robots")
	static FString ListRobots();

	/** The robot the play world started with (an id or class path); empty with
	 *  no play world or none started. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote|Robots")
	static FString GetActiveRobotId();

	/** The project's default robot (URammsRobotSettings, which a mounted plugin's
	 *  Game.ini may set): what a level starts with when nothing else chooses. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote|Robots")
	static FString GetDefaultRobotId();

	/** Where it came from, as words (the map URL, ramms.Robot, a default...). */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote|Robots")
	static FString GetActiveRobotSource();

	/** ramms.Robot: the robot the next level started (or PIE session) spawns.
	 *  False when the words name no robot (an id, alias, class path, `ask` or
	 *  empty are taken). */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote|Robots")
	static bool SetNextRobot(const FString& IdOrAlias);

	/** Open the play world's map again with this robot. False with no play world
	 *  or words that name no robot. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote|Robots")
	static bool RestartWithRobot(const FString& IdOrAlias);

	/** Open a map (/Game/Maps/Curb/Map_Curb_S0_URL) in the play world with this
	 *  robot. False with no play world, words that name no robot, or a map that
	 *  is not there. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Remote|Robots")
	static bool OpenMapWithRobot(const FString& MapPath, const FString& IdOrAlias);

	/** The words name a robot: a definition's id or alias, a class path, `ask`,
	 *  or nothing (the defaults). */
	static bool IsKnownRobot(const FString& IdOrAlias);
};
