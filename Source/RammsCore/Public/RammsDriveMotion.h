// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "RammsDriveMotion.generated.h"

/**
 * How a base is moving over the ground, in its own frame: the chassis' velocity
 * at its origin along its forward (+X) and right (+Y) axes, and how fast it is
 * turning about the vertical. Unreal's sense throughout: +Y is to the robot's
 * right, and a positive yaw rate turns it from +X towards +Y (to the right,
 * clockwise seen from above).
 *
 * Drive modes hand this to each other when the robot changes mode on the move
 * (IRammsDriveMode::SeedMotion), so that the mode taking over carries on with
 * the motion the robot has instead of starting from rest.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsDriveMotion
{
	GENERATED_BODY()

	/** Along the chassis' forward axis, cm/s. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive")
	float ForwardCmPerSecond = 0.0f;

	/** Sideways, along the chassis' right axis, cm/s. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive")
	float RightCmPerSecond = 0.0f;

	/** Turning, deg/s; positive turns to the right. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive")
	float YawRateDegPerSecond = 0.0f;

	FRammsDriveMotion() = default;
	FRammsDriveMotion(float InForward, float InRight, float InYawRate)
		: ForwardCmPerSecond(InForward)
		, RightCmPerSecond(InRight)
		, YawRateDegPerSecond(InYawRate)
	{
	}

	bool IsNearlyZero(float SpeedTolerance = 0.5f, float YawTolerance = 0.5f) const
	{
		return FMath::Abs(ForwardCmPerSecond) <= SpeedTolerance && FMath::Abs(RightCmPerSecond) <= SpeedTolerance
			&& FMath::Abs(YawRateDegPerSecond) <= YawTolerance;
	}
};
