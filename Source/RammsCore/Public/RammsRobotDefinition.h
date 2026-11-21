// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "RammsRobotDefinition.generated.h"

class APawn;
class APlayerController;

/**
 * A robot a level can be started with: what to spawn, and how to name it.
 *
 * Kept with the robot's own content (a public robot under /Game/Robots, a
 * plugin's under its own mount point), so a level that is started through
 * ARammsRobotGameMode names no robot at all and a robot whose plugin is not
 * mounted is simply not offered. URammsRobotCatalog finds the definitions
 * through the asset registry; the level, the command line (-RammsRobot=), the
 * ramms.Robot console variable or a ?Robot= option on the map URL choose one
 * by id or alias.
 *
 * Scene settings (the MuJoCo timestep, cone, impratio) are the robot's own
 * MJCF's: a level that sets none takes them from the robot as it attaches.
 */
UCLASS(BlueprintType)
class RAMMSCORE_API URammsRobotDefinition : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	/** The primary asset type every robot definition is registered under. */
	static const FPrimaryAssetType AssetType;

	/** The stable id a level, a script or the command line chooses it by
	 *  (rammp_gen1, rammp_gen2). Lowercase, no spaces. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Robot")
	FName RobotId;

	/** Other names it answers to (a harness's own name for it: Holonomic).
	 *  Ids and aliases match without regard to case. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Robot")
	TArray<FName> Aliases;

	/** Shown in the picker (the id when empty). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Robot")
	FText DisplayName;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Robot", meta = (MultiLine = true))
	FText Description;

	/** The pawn spawned at the level's PlayerStart. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Robot")
	TSoftClassPtr<APawn> PawnClass;

	/** The player controller it needs, if not the game mode's own. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Robot")
	TSoftClassPtr<APlayerController> PlayerControllerClass;

	/** What it is and can do (mobile, curb, arm, seat): a level can ask for
	 *  robots that carry a tag. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Robot")
	TArray<FName> Tags;

	/** Added to the PlayerStart's location, in the start's own frame (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Robot")
	FVector SpawnOffset = FVector::ZeroVector;

	/** Order in the picker (lower first; then by name). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Robot")
	int32 SortOrder = 0;

	/** Not offered in the picker, nor taken as the only robot; still started
	 *  when asked for by id. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Robot")
	bool bHidden = false;

	virtual FPrimaryAssetId GetPrimaryAssetId() const override;

	/** Its RobotId or one of its aliases, without regard to case. */
	bool Matches(const FString& IdOrAlias) const;

	/** DisplayName, or the id when it has none. */
	FText GetDisplayNameOrId() const;
};
