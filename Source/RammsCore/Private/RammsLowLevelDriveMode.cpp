// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsLowLevelDriveMode.h"

#include "GameFramework/Actor.h"
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
	if (const AActor* Owner = GetOwner())
	{
		if (URammsRobotControlSurfaceComponent* Surface =
				Owner->FindComponentByClass<URammsRobotControlSurfaceComponent>())
		{
			Surface->bExposeUnclaimedMotors = bActive;
			Surface->UnclaimedMotorGroup = MotorGroup;
		}
	}
}
