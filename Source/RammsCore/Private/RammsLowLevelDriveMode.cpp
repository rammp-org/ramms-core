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
		if (bActive)
		{
			// Remember what the robot was authored with, so standing down puts
			// it back rather than leaving this mode's choice behind for good.
			if (!bHasSavedExposure)
			{
				bHasSavedExposure = true;
				bSavedExposeUnclaimedMotors = Surface->bExposeUnclaimedMotors;
				SavedUnclaimedMotorGroup = Surface->UnclaimedMotorGroup;
			}
			Surface->bExposeUnclaimedMotors = true;
			Surface->UnclaimedMotorGroup = MotorGroup;
		}
		else if (bHasSavedExposure)
		{
			bHasSavedExposure = false;
			Surface->bExposeUnclaimedMotors = bSavedExposeUnclaimedMotors;
			Surface->UnclaimedMotorGroup = SavedUnclaimedMotorGroup;
		}
	}

	// Standing down always releases, whatever the flag says now. Reading
	// bClaimAllActuators here to decide whether to UN-suspend would strand
	// every contributor suspended for the rest of the session if the flag were
	// turned off while this mode was live -- the robot would come back with no
	// drive controls and its linkages limp, with no way to recover but
	// restarting play.
	const bool bSuspendOthers = bActive && bClaimAllActuators;
	if (!bSuspendOthers && !bSuspendedOthers)
	{
		return;
	}
	bSuspendedOthers = bSuspendOthers;

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
		Contributor->SetContributionSuspended(bSuspendOthers);
	}
}
