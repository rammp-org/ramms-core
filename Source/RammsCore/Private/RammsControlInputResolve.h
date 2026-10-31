// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsControlTypes.h"

/**
 * What URammsControlInputComponent's bindings drive, as functions of the
 * surface and the stick alone, so that tests reach them without a player.
 */
namespace RammsControlInput
{
	/**
	 * A Stick binding's targets on Surface. Ids holds what X / Y / Z would feed
	 * as a plain Axis binding (ControlId, ControlIdY, ControlIdZ, wildcards
	 * expanded); on return X feeds the control the surface pairs with
	 * ControlIdY, when it pairs it with one that is on the surface, and Z only
	 * what X and Y do not.
	 */
	void ResolveStick(const FRammsControlSurface& Surface, FName ControlIdY, TArray<FName> (&Ids)[3]);

	/** How far off centre (of full deflection) a stick still reads as centred
	 *  for StepCentre. */
	constexpr float CentreBand = 0.1f;

	/**
	 * One stick axis waiting to come back to centre (bAwait) before it drives a
	 * control it was not driving when it was pushed: the value to send for
	 * Value this tick -- nothing while it waits -- and bAwait cleared once the
	 * axis reads within CentreBand.
	 */
	float StepCentre(bool& bAwait, float Value);
} // namespace RammsControlInput
