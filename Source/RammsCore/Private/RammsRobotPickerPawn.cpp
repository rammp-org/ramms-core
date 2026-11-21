// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRobotPickerPawn.h"
#include "RammsRobotControlSurfaceComponent.h"
#include "RammsRobotSelectComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/InputComponent.h"
#include "InputCoreTypes.h"

ARammsRobotPickerPawn::ARammsRobotPickerPawn()
{
	PrimaryActorTick.bCanEverTick = false;
	AutoPossessPlayer = EAutoReceiveInput::Disabled; // the game mode hands it to the player
	// And no AI either: a pawn spawned while the level starts counts as placed in
	// the world, and one placed in the world with no player to receive gets an AI
	// controller -- the game mode would then spawn a second picker for the player.
	AutoPossessAI = EAutoPossessAI::Disabled;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(Root);
	Camera->SetRelativeLocation(FVector(-250.0, 0.0, 180.0));
	Camera->SetRelativeRotation(FRotator(-20.0, 0.0, 0.0));

	ControlSurface = CreateDefaultSubobject<URammsRobotControlSurfaceComponent>(TEXT("ControlSurface"));
	ControlSurface->RobotDisplayName = NSLOCTEXT("Ramms", "RobotPicker", "Choose a robot");

	RobotSelect = CreateDefaultSubobject<URammsRobotSelectComponent>(TEXT("RobotSelect"));
	RobotSelect->NoRobotHint = NSLOCTEXT("Ramms", "RobotPickerKeys", "Choose one with the D-pad or the arrow keys, then start it with A or Enter.");
}

void ARammsRobotPickerPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	if (!PlayerInputComponent)
	{
		return;
	}
	// Plain key bindings, no input assets: they still run beside Enhanced Input,
	// and the picker has nothing else to drive.
	for (const FKey& Key : { EKeys::Gamepad_DPad_Right, EKeys::Gamepad_DPad_Down, EKeys::Right, EKeys::Down })
	{
		PlayerInputComponent->BindKey(Key, IE_Pressed, this, &ARammsRobotPickerPawn::StageNext);
	}
	for (const FKey& Key : { EKeys::Gamepad_DPad_Left, EKeys::Gamepad_DPad_Up, EKeys::Left, EKeys::Up })
	{
		PlayerInputComponent->BindKey(Key, IE_Pressed, this, &ARammsRobotPickerPawn::StagePrevious);
	}
	for (const FKey& Key : { EKeys::Gamepad_FaceButton_Bottom, EKeys::Enter })
	{
		PlayerInputComponent->BindKey(Key, IE_Pressed, this, &ARammsRobotPickerPawn::LoadStaged);
	}
}

void ARammsRobotPickerPawn::StageNext()
{
	if (RobotSelect)
	{
		RobotSelect->StepStagedRobot(1);
	}
}

void ARammsRobotPickerPawn::StagePrevious()
{
	if (RobotSelect)
	{
		RobotSelect->StepStagedRobot(-1);
	}
}

void ARammsRobotPickerPawn::LoadStaged()
{
	if (RobotSelect)
	{
		RobotSelect->LoadStagedRobot();
	}
}
