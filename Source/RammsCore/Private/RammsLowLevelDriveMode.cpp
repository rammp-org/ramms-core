// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsLowLevelDriveMode.h"

#include "GameFramework/Actor.h"
#include "RammsControlContributor.h"
#include "RammsRobotControlSurfaceComponent.h"

URammsLowLevelDriveMode::URammsLowLevelDriveMode()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URammsLowLevelDriveMode::SetDriveModeActive(bool bActive)
{
	bDriveModeActive = bActive;

	// This mode's whole effect is on the surface: it drives nothing itself, it
	// decides whether the robot's motors are offered individually. The selector
	// rebuilds the surface after this returns, so the change lands with the
	// same rebuild that removes the other mode's controls.
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}
	if (URammsRobotControlSurfaceComponent* Surface =
			Owner->FindComponentByClass<URammsRobotControlSurfaceComponent>())
	{
		// While a low-level mode exists on the robot, IT owns the raw motor
		// exposure -- that gating is the mode's whole reason for existing, so
		// it overrides the surface's authored flag rather than restoring it.
		// (An earlier attempt to put the authored value back on stand-down
		// ungated the robot completely: the flag defaults to true, so every
		// mode showed every unclaimed motor and the gate did nothing.) The
		// authored flag still decides for robots with no low-level mode.
		Surface->bExposeUnclaimedMotors = bActive;
		if (bActive)
		{
			Surface->UnclaimedMotorGroup = MotorGroup;
		}
	}

	// Standing down always releases, whatever the flag says now. Reading
	// bClaimAllActuators here to decide whether to UN-suspend would strand
	// every contributor suspended for the rest of the session if the flag were
	// turned off while this mode was live -- the robot would come back with no
	// drive controls and its linkages limp, with no way to recover but
	// restarting play.
	ApplyClaimAll(bActive && bClaimAllActuators);
}

void URammsLowLevelDriveMode::SetClaimAllActuators(bool bInClaimAll)
{
	if (bClaimAllActuators == bInClaimAll)
	{
		return;
	}
	bClaimAllActuators = bInClaimAll;
	if (!bDriveModeActive)
	{
		// Takes effect when this mode next goes live.
		return;
	}
	ApplyClaimAll(bInClaimAll);

	// Unlike SetDriveModeActive, nothing rebuilds the surface after this: the
	// selector is not involved, and the controls that just appeared or went
	// away would not reach a panel until something else changed.
	if (const AActor* Owner = GetOwner())
	{
		if (URammsRobotControlSurfaceComponent* Surface =
				Owner->FindComponentByClass<URammsRobotControlSurfaceComponent>())
		{
			Surface->RebuildControlSurface();
		}
	}
}

void URammsLowLevelDriveMode::ApplyClaimAll(bool bClaimAll)
{
	if (!bClaimAll && !bSuspendedOthers)
	{
		return;
	}
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}
	bSuspendedOthers = bClaimAll;

	// Opted in: stand the other contributors down so the actuators they claim
	// -- the 5-bar hips above all -- come back as raw axes. Anything that says
	// it cannot be suspended is left alone; the drive-mode selector says that,
	// because suspending it would remove the control you switch back with.
	for (UActorComponent* Component : Owner->GetComponents())
	{
		IRammsControlContributor* Contributor = Cast<IRammsControlContributor>(Component);
		if (!Contributor || Component == this || !Contributor->CanSuspendContribution())
		{
			continue;
		}
		Contributor->SetContributionSuspended(bClaimAll);
	}
}
