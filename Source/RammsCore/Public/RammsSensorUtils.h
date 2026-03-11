// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * Shared utility functions for Ramms sensor components.
 */
namespace RammsSensorUtils
{
	/** Generate a Gaussian random value using the Box-Muller transform. */
	FORCEINLINE float GaussianNoise(float StdDev)
	{
		if (StdDev <= 0.0f)
			return 0.0f;
		float U1 = FMath::FRand();
		float U2 = FMath::FRand();
		if (U1 < SMALL_NUMBER)
			U1 = SMALL_NUMBER;
		return StdDev * FMath::Sqrt(-2.0f * FMath::Loge(U1)) * FMath::Cos(2.0f * PI * U2);
	}

	/** Add independent Gaussian noise to each axis of a vector. */
	FORCEINLINE FVector AddVectorNoise(const FVector& Value, float StdDev)
	{
		if (StdDev <= 0.0f)
			return Value;
		return FVector(
			Value.X + GaussianNoise(StdDev),
			Value.Y + GaussianNoise(StdDev),
			Value.Z + GaussianNoise(StdDev));
	}
} // namespace RammsSensorUtils
