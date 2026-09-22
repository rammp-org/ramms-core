// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "RammsControlContributor.h"
#include "RammsControlIds.h"

#include "RammsDriveModeSelector.generated.h"

class URammsRobotBaseComponent;

/**
 * Picks which drive mode a robot is running.
 *
 * A lift-drive chassis can run differential or holonomic; both controllers sit
 * on the robot and exactly one is live. This finds every IRammsDriveMode on the
 * actor -- naming no controller class, the way the control surface names no
 * contributor -- and activates one, rebuilding the surface so the inactive
 * mode's controls disappear from the panel rather than sitting there inert.
 *
 * Switching is physical as well as logical. The centre wheels are large treaded
 * tyres: holonomic needs them lifted clear on the 5-bar linkages, differential
 * needs them planted so they can overcome the omni wheels' friction. So a
 * switch also commands the linkages to the incoming mode's stance.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsDriveModeSelector : public UActorComponent, public IRammsControlContributor
{
	GENERATED_BODY()

public:
	URammsDriveModeSelector();

	virtual void BeginPlay() override;

	/** Mode to select at BeginPlay. Empty means the first one found. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive Mode")
	FName InitialModeId;

	/** Also drive the 5-bar linkages to the incoming mode's stance. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive Mode")
	bool bApplyLinkageStance = true;

	/** Ids of the modes found on this robot, in the order they are offered. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Drive Mode")
	TArray<FName> GetAvailableModeIds() const;

	UFUNCTION(BlueprintPure, Category = "Ramms|Drive Mode")
	FName GetActiveModeId() const;

	/** Activate a mode by id. False when no such mode is on this robot. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Drive Mode")
	bool SetActiveMode(FName ModeId);

	/** Next mode in the offered order, wrapping. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Drive Mode")
	bool CycleMode();

	// --- IRammsControlContributor --------------------------------------------
	virtual void  DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool  ApplyControl(FName Id, float Value) override;
	virtual bool  TriggerControl(FName Id) override;
	virtual bool  ReadControl(FName Id, float& OutValue) const override;
	virtual bool  ReadTarget(FName Id, float& OutTarget) const override { return ReadControl(Id, OutTarget); }
	virtual int32 GetControlOrder() const override { return -10; }
	/** Never: suspending this would remove the control you switch back with. */
	virtual bool CanSuspendContribution() const override { return false; }

private:
	/** Components implementing IRammsDriveMode, gathered once at BeginPlay. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UActorComponent>> Modes;

	void GatherModes();
	void ApplyStanceFor(UActorComponent* Mode);

	int32 ActiveIndex = INDEX_NONE;
};
