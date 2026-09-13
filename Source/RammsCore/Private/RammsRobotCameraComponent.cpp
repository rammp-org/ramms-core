// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRobotCameraComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"

URammsRobotCameraComponent::URammsRobotCameraComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void URammsRobotCameraComponent::BeginPlay()
{
	Super::BeginPlay();

	// Start on the first camera so exactly one is active from the first frame.
	TArray<UCameraComponent*> Cameras;
	GatherCameras(Cameras);
	if (Cameras.Num() > 0)
	{
		UCameraComponent* First = nullptr;
		for (UCameraComponent* C : Cameras)
		{
			if (C->IsActive())
			{
				First = C;
				break;
			}
		}
		SetActiveCamera(First ? First : Cameras[0]);
	}
}

APlayerController* URammsRobotCameraComponent::GetPlayerController() const
{
	const APawn* Pawn = Cast<APawn>(GetOwner());
	return Pawn ? Cast<APlayerController>(Pawn->GetController()) : nullptr;
}

void URammsRobotCameraComponent::GatherCameras(TArray<UCameraComponent*>& Out) const
{
	Out.Reset();
	if (const AActor* Owner = GetOwner())
	{
		TArray<UCameraComponent*> All;
		Owner->GetComponents<UCameraComponent>(All);
		for (UCameraComponent* C : All)
		{
			if (C && (CameraNames.Num() == 0 || CameraNames.Contains(C->GetFName())))
			{
				Out.Add(C);
			}
		}
	}
}

void URammsRobotCameraComponent::SetActiveCamera(UCameraComponent* Camera)
{
	TArray<UCameraComponent*> All;
	if (const AActor* Owner = GetOwner())
	{
		Owner->GetComponents<UCameraComponent>(All);
	}
	// One active camera: AActor::CalcCamera uses the first active one it finds.
	for (UCameraComponent* C : All)
	{
		C->SetActive(C == Camera);
	}
	ActiveCamera = Camera;
	if (USpringArmComponent* Arm = ActiveArm())
	{
		if (!ArmDefaults.Contains(Arm))
		{
			ArmDefaults.Add(Arm, TPair<FRotator, float>(Arm->GetRelativeRotation(), Arm->TargetArmLength));
		}
	}
	UE_LOG(LogTemp, Log, TEXT("[RobotCamera] '%s' active camera -> %s"),
		GetOwner() ? *GetOwner()->GetName() : TEXT("?"), Camera ? *Camera->GetName() : TEXT("none"));
}

USpringArmComponent* URammsRobotCameraComponent::ActiveArm() const
{
	return ActiveCamera ? Cast<USpringArmComponent>(ActiveCamera->GetAttachParent()) : nullptr;
}

void URammsRobotCameraComponent::NextCamera()
{
	// Re-gather every time: URLab adds its possess camera after BeginPlay.
	TArray<UCameraComponent*> Cameras;
	GatherCameras(Cameras);
	if (Cameras.Num() == 0)
	{
		return;
	}
	const int32 Current = Cameras.IndexOfByKey(ActiveCamera);
	SetActiveCamera(Cameras[(Current + 1) % Cameras.Num()]);
}

bool URammsRobotCameraComponent::ActivateCamera(FName CameraName)
{
	TArray<UCameraComponent*> Cameras;
	GatherCameras(Cameras);
	for (UCameraComponent* C : Cameras)
	{
		if (C->GetFName() == CameraName)
		{
			SetActiveCamera(C);
			return true;
		}
	}
	return false;
}

void URammsRobotCameraComponent::Orbit(float DeltaYawDegrees, float DeltaPitchDegrees)
{
	USpringArmComponent* Arm = ActiveArm();
	if (!Arm || (DeltaYawDegrees == 0.0f && DeltaPitchDegrees == 0.0f))
	{
		return;
	}
	FRotator Rot = Arm->GetRelativeRotation();
	Rot.Yaw += DeltaYawDegrees;
	Rot.Pitch = FMath::Clamp(Rot.Pitch + DeltaPitchDegrees, MinPitch, MaxPitch);
	Rot.Roll = 0.0f;
	Arm->SetRelativeRotation(Rot);
}

void URammsRobotCameraComponent::Zoom(float DeltaLength)
{
	if (USpringArmComponent* Arm = ActiveArm())
	{
		Arm->TargetArmLength = FMath::Clamp(Arm->TargetArmLength + DeltaLength, MinArmLength, MaxArmLength);
	}
}

void URammsRobotCameraComponent::ResetOrbit()
{
	if (USpringArmComponent* Arm = ActiveArm())
	{
		if (const TPair<FRotator, float>* Def = ArmDefaults.Find(Arm))
		{
			Arm->SetRelativeRotation(Def->Key);
			Arm->TargetArmLength = Def->Value;
		}
	}
}

void URammsRobotCameraComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	APlayerController* PC = GetPlayerController();
	if (!PC)
	{
		return;
	}

	if (NextCameraKey.IsValid() && PC->WasInputKeyJustPressed(NextCameraKey))
	{
		NextCamera();
	}
	if (ResetKey.IsValid() && PC->WasInputKeyJustPressed(ResetKey))
	{
		ResetOrbit();
	}

	USpringArmComponent* Arm = ActiveArm();
	if (!Arm)
	{
		return;
	}

	// Orbit: mouse delta while the orbit button (or a left drag) is held, or
	// always if no button is configured.
	// "Just released" counts too: a very short drag (touch flick, automation)
	// can press, move and release inside one frame, and its movement would
	// otherwise be thrown away.
	auto	   Held = [PC](const FKey& Key) { return PC->IsInputKeyDown(Key) || PC->WasInputKeyJustReleased(Key); };
	const bool bOrbiting = !OrbitButton.IsValid() || Held(OrbitButton)
		|| (bAlsoOrbitWithLeftDrag && Held(EKeys::LeftMouseButton));

	// Two delta sources: the raw mouse axes (only fed while the viewport has
	// captured the mouse) and the cursor's position change (what a streamed
	// "hovering" cursor or a touch drag produces). Use whichever moved.
	float DX = 0.0f, DY = 0.0f;
	PC->GetInputMouseDelta(DX, DY);
	float	   CursorX = 0.0f, CursorY = 0.0f;
	const bool bHaveCursor = PC->GetMousePosition(CursorX, CursorY);
	if (bOrbiting && bHaveCursor && bHadCursor && DX == 0.0f && DY == 0.0f)
	{
		DX = CursorX - LastCursorX;
		DY = -(CursorY - LastCursorY); // screen y grows downward; mouse axis up is positive
	}
	LastCursorX = CursorX;
	LastCursorY = CursorY;
	bHadCursor = bHaveCursor;

	if (bOrbiting)
	{
		Orbit(DX * OrbitSensitivity, DY * OrbitSensitivity * (bInvertPitch ? -1.0f : 1.0f));
	}

	// Zoom: wheel notches arrive as key presses.
	float ZoomDelta = 0.0f;
	if (PC->WasInputKeyJustPressed(EKeys::MouseScrollUp))
	{
		ZoomDelta -= ZoomStep;
	}
	if (PC->WasInputKeyJustPressed(EKeys::MouseScrollDown))
	{
		ZoomDelta += ZoomStep;
	}
	Zoom(ZoomDelta);
}
