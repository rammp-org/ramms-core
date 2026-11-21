// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "RammsRobotPickerPawn.generated.h"

class URammsRobotControlSurfaceComponent;
class URammsRobotSelectComponent;
class UCameraComponent;

/**
 * Stands in at the PlayerStart when a level started through
 * ARammsRobotGameMode has no robot to spawn (nothing chosen, `ask`, or a
 * choice that matched nothing): a control surface carrying only the robot
 * choice (URammsRobotSelectComponent), so the HUD, Remote Control and
 * ramms-tools can pick one and start the level with it. Its surface is the
 * same class every robot's is, so the same allow-listed calls reach it.
 *
 * Keys, bound here with no input assets: the D-pad or the arrow keys step the
 * choice (right / down the next, left / up the one before), and the gamepad's
 * bottom face button (A) or Enter starts the level with it.
 */
UCLASS()
class RAMMSCORE_API ARammsRobotPickerPawn : public APawn
{
	GENERATED_BODY()

public:
	ARammsRobotPickerPawn();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Robot")
	TObjectPtr<USceneComponent> Root;

	/** A view of the course from behind and above the start. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Robot")
	TObjectPtr<UCameraComponent> Camera;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Robot")
	TObjectPtr<URammsRobotControlSurfaceComponent> ControlSurface;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Robot")
	TObjectPtr<URammsRobotSelectComponent> RobotSelect;

protected:
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

private:
	void StageNext();
	void StagePrevious();
	void LoadStaged();
};
