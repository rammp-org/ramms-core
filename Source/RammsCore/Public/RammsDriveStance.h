// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "RammsDriveStance.generated.h"

/** One actuator held at one value for the duration of a stance. */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsStanceMotorTarget
{
	GENERATED_BODY()

	/** Registry motor Id (a crank, a hip). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance")
	FName MotorId;

	/** Target in the actuator's own units -- radians for a hinge. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance")
	float Target = 0.0f;
};

/**
 * The pose a drive mode needs the robot standing in.
 *
 * Switching drive mode is not only a software matter on this hardware: the
 * lift-drive runs holonomic on its corner omni wheels with the treaded centre
 * tyres clear of the ground, and differential on those centre tyres with
 * weight on them. Which wheels touch is a function of the whole leg, not of
 * one number -- the 5-bar endpoint height sets where the centre wheels sit,
 * and the front and rear cranks set how far the corners reach down.
 *
 * Those interact. With the cranks driven to about 1.15 rad the corners carry
 * the robot, and the centre wheels clear at an endpoint height of 13 cm --
 * 10 to 11 cm is enough in practice. With the cranks left where they are it
 * takes about 15 cm to get the same clearance, which is near the top of the
 * 5-bar's travel and a worse place to operate.
 *
 * So a mode describes a stance rather than a height, and the selector commands
 * the whole thing.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsDriveStance
{
	GENERATED_BODY()

	/** False when the mode does not care where the linkages are. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance")
	bool bHasLinkageHeight = false;

	/** 5-bar endpoint height (cm) every linkage should hold. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance", meta = (EditCondition = "bHasLinkageHeight"))
	float LinkageHeightCm = 0.0f;

	/** False when the mode does not care where the endpoint sits fore/aft. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance")
	bool bHasLinkageTranslation = false;

	/** 5-bar endpoint fore/aft position (cm). Where along the robot the wheel
	 *  carries matters as much as how low it sits: the lift-drive wants its
	 *  centre wheels further forward when they are the ones driving. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance", meta = (EditCondition = "bHasLinkageTranslation"))
	float LinkageTranslationCm = 0.0f;

	/**
	 * Other actuators the stance depends on, by registry Id.
	 *
	 * Left empty in code on purpose: which motors these are, and what angles
	 * they want, is the robot's geometry rather than the controller's, so they
	 * are authored on the robot's Blueprint. A mode with none of these still
	 * works -- it just needs whatever linkage height clears the wheels on its
	 * own.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stance")
	TArray<FRammsStanceMotorTarget> MotorTargets;

	bool IsEmpty() const
	{
		return !bHasLinkageHeight && !bHasLinkageTranslation && MotorTargets.Num() == 0;
	}
};
