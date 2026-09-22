// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"

#include "RammsDriveMode.generated.h"

UINTERFACE(MinimalAPI)
class URammsDriveMode : public UInterface
{
	GENERATED_BODY()
};

/**
 * One way of driving a base. A robot may carry several -- a differential drive
 * and a holonomic drive on the same chassis -- with exactly one active.
 *
 * They cannot all be live at once: they advertise the same control Ids
 * (drive.forward, drive.turn) and would claim the same motors, so the surface
 * would see duplicates and the motors would be written by two controllers in
 * the same frame. An inactive mode therefore contributes nothing: no controls,
 * no motor claims, no commands.
 *
 * Switching is not only a software matter on this hardware. The lift-drive
 * raises its centre wheels clear of the ground on the 5-bar linkages to run
 * holonomic, and lowers them to run differential -- so a mode also has a stance
 * it needs the robot in. That is why a mode reports a linkage height rather
 * than the selector guessing one.
 */
class RAMMSCORE_API IRammsDriveMode
{
	GENERATED_BODY()

public:
	/** Stable id for this mode, e.g. "differential" or "holonomic". */
	virtual FName GetDriveModeId() const = 0;

	/** Human-readable, for the mode control's display. */
	virtual FText GetDriveModeDisplayName() const = 0;

	virtual bool IsDriveModeActive() const = 0;

	/** Take over, or stand down. A mode standing down must stop writing motors
	 *  and stop advertising controls; the surface is rebuilt around it. */
	virtual void SetDriveModeActive(bool bActive) = 0;

	/**
	 * Endpoint height (cm) the 5-bar linkages need for this mode's wheels to be
	 * the ones on the ground, and false when the mode does not care.
	 *
	 * Holonomic wants the centre wheels lifted; differential wants them down
	 * and carrying load, because they are the treaded tyres that have to
	 * overcome the omni wheels' friction.
	 */
	virtual bool GetRequiredLinkageHeight(float& OutHeightCm) const
	{
		return false;
	}
};
