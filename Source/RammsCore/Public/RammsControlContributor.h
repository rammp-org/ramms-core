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

	/** The control's current commanded target, whoever set it (a direct call
	 *  on the controller included). False = no target: released, disabled, or
	 *  never commanded. For a Position / Velocity control a false answer is
	 *  authoritative — the surface reports "no target" rather than falling
	 *  back to the last value commanded through it — so a contributor that
	 *  holds targets must implement this, and one that doesn't must keep the
	 *  default. */
	virtual bool ReadTarget(FName Id, float& OutTarget) const { return false; }

	/** RobotBase motor Ids this contributor drives (not exposed as raw motors). */
	virtual void GetClaimedMotorIds(TArray<FName>& OutIds) const {}

	/** Presentation order of this contributor's groups (lower first). */
	virtual int32 GetControlOrder() const { return 100; }

	/**
	 * Stand down: contribute no controls and claim no motors, without being
	 * removed from the robot.
	 *
	 * The low-level drive mode uses this to hand every actuator over for
	 * direct driving -- a controller that keeps claiming its motors is a
	 * controller you cannot drive around. Contributors that do not implement
	 * it simply keep contributing.
	 */
	virtual void SetContributionSuspended(bool bSuspended) {}
	virtual bool IsContributionSuspended() const { return false; }

	/**
	 * False for a contributor that must keep working whatever else stands
	 * down -- the drive-mode selector above all, since suspending it would
	 * remove the control you need to switch back.
	 */
	virtual bool CanSuspendContribution() const { return true; }
};
