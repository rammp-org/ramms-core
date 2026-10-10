// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Ramms4BarLegSpec.h"
#include "Ramms4BarKinematics.generated.h"

/**
 * Pure planar kinematics for a crank-rocker leg (FRamms4BarLegSpec). Stateless
 * and physics-free: crank angle to wheel centre, and a wheel height back to
 * the crank angle that reaches it. Positions are chassis x-z cm, angles rad.
 */
UCLASS()
class RAMMSCORE_API URamms4BarKinematics : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Wheel centre for a crank angle. bValid is false when the loop cannot
	 * close there, or closes only with the rocker outside its joint range; the
	 * result is then meaningless.
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|4-Bar")
	static FVector2D ComputeWheelCenter(const FRamms4BarLegSpec& Spec, float CrankAngle, bool& bValid);

	/**
	 * The crank angle that puts the wheel centre at height Z (cm), and the
	 * fore/aft position the wheel then sits at. bReachable is false when Z is
	 * outside what the crank's range reaches; the angle is then the end of the
	 * range nearest Z.
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|4-Bar")
	static float SolveCrankForHeight(const FRamms4BarLegSpec& Spec, float Z, bool& bReachable, float& OutWheelX);

	/** Lowest and highest wheel-centre heights (cm) across the crank range. */
	UFUNCTION(BlueprintPure, Category = "Ramms|4-Bar")
	static FVector2D GetHeightRange(const FRamms4BarLegSpec& Spec);
};
