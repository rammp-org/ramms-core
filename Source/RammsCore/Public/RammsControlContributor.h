// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "RammsControlTypes.h"
#include "RammsControlContributor.generated.h"

UINTERFACE(MinimalAPI)
class URammsControlContributor : public UInterface
{
	GENERATED_BODY()
};

/**
 * A component that contributes controls to its robot's control surface: it
 * describes them (FRammsControlAxis, grouped) and executes them by Id. The
 * robot's URammsRobotControlSurfaceComponent finds every contributor on the
 * actor through this interface — no controller is wired by name — merges the
 * descriptions, and routes SetAxis / TriggerAction / ReleaseAxis here.
 *
 * Motors a contributor drives on the RobotBase (the diff-drive's wheels, the
 * 5-bar's hips) are "claimed" so the surface doesn't also expose them as raw
 * motor axes.
 *
 * C++ only for now (plain virtuals); a Blueprint contributor can be added by
 * mirroring these as BlueprintNativeEvents if a need shows up.
 */
class RAMMSCORE_API IRammsControlContributor
{
	GENERATED_BODY()

public:
	/** Append this component's controls to the robot's surface. */
	virtual void DescribeControls(FRammsControlSurface& OutSurface) const = 0;

	/** Set a Continuous / Position / Velocity control. False = not applied. */
	virtual bool ApplyControl(FName Id, float Value) = 0;

	/** Fire an Action control. False = unknown / refused. */
	virtual bool TriggerControl(FName Id) { return false; }

	/** The driver let go: spring a Continuous axis back, or stop holding a
	 *  Position / Velocity target. False = cannot release (still driven). */
	virtual bool ReleaseControl(FName Id) { return false; }

	/** Live value of a control with readback. False = unknown. */
	virtual bool ReadControl(FName Id, float& OutValue) const { return false; }

	/** RobotBase motor Ids this contributor drives (not exposed as raw motors). */
	virtual void GetClaimedMotorIds(TArray<FName>& OutIds) const {}

	/** Presentation order of this contributor's groups (lower first). */
	virtual int32 GetControlOrder() const { return 100; }
};
