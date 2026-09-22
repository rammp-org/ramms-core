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
};
