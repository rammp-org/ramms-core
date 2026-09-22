// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "RammsDriveMode.h"

#include "RammsLowLevelDriveMode.generated.h"

/**
 * A drive mode that drives nothing, so every motor can be driven by hand.
 *
 * The other modes coordinate wheels into a body twist, which is what you want
 * to operate a robot and exactly what you do not want to diagnose one. This
 * mode claims no motors and offers no drive controls, so the wheels the other
 * modes would have owned appear as raw per-motor axes instead.
 *
 * It also gates that exposure. Raw motor axes are useful for bring-up and noise
 * the rest of the time, so the surface only offers them while this mode is
 * live; selecting differential or holonomic takes them away again.
 *
 * It asks for no linkage stance: whatever the robot is standing on when you
 * switch here stays, because moving the linkages under someone who is
 * debugging them is the opposite of helpful.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsLowLevelDriveMode : public UActorComponent, public IRammsDriveMode
{
	GENERATED_BODY()

public:
	URammsLowLevelDriveMode();

	/** Group the raw motor axes land in while this mode is live. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive Mode")
	FName MotorGroup = FName("Motors");

	/**
	 * Also stand down every other contributor, so actuators they claim -- the
	 * 5-bar hips above all -- become raw axes too.
	 *
	 * Off by default: the linkages hold the robot's stance, and releasing them
	 * drops it onto whatever the motors do next. Turn this on when the thing
	 * being diagnosed IS a claimed actuator.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive Mode")
	bool bClaimAllActuators = false;

	// --- IRammsDriveMode ------------------------------------------------------
	virtual FName GetDriveModeId() const override { return FName("low_level"); }
	virtual FText GetDriveModeDisplayName() const override
	{
		return NSLOCTEXT("Ramms", "DriveModeLowLevel", "Low level");
	}
	virtual bool IsDriveModeActive() const override { return bDriveModeActive; }
	virtual void SetDriveModeActive(bool bActive) override;

	/** Deliberately none: leave the robot standing as it is. */
	virtual bool GetRequiredLinkageHeight(float& OutHeightCm) const override { return false; }

private:
	bool bDriveModeActive = false;

	/** True while this mode holds other contributors suspended, so standing
	 *  down releases them even if bClaimAllActuators changed meanwhile. */
	bool bSuspendedOthers = false;

	/** The surface's authored motor exposure, restored when this mode ends. */
	bool  bHasSavedExposure = false;
	bool  bSavedExposeUnclaimedMotors = true;
	FName SavedUnclaimedMotorGroup = FName("Motors");
};
