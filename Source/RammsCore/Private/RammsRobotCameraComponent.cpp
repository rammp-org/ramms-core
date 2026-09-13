// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRobotCameraComponent.h"
#include "Camera/CameraComponent.h"
#include "Framework/Application/SlateApplication.h"
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

	// Start on the first camera (CameraNames[0], else the first authored one)
	// so exactly one is active from the first frame. Any camera added at
	// possession before BeginPlay is left for Tab.
	TArray<UCameraComponent*> Cameras;
	GatherCameras(Cameras);
	if (Cameras.Num() > 0)
	{
		SetActiveCamera(Cameras[0]);
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
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}
	TArray<UCameraComponent*> All;
	Owner->GetComponents<UCameraComponent>(All);

	if (CameraNames.Num() > 0)
	{
		// Explicit list: its order is the cycle order and [0] is the default.
		for (const FName& Name : CameraNames)
		{
			for (UCameraComponent* C : All)
			{
				if (C && C->GetFName() == Name)
				{
					Out.Add(C);
					break;
				}
			}
		}
		return;
	}

	// Otherwise the authored cameras (Blueprint / native) come first and
	// runtime-added ones (e.g. URLab's possess camera) last. GetComponents
	// order is the owned-component set's, which can put a component added
	// at possession *before* the authored ones once earlier removals left
	// holes — the start camera must not depend on that.
	for (UCameraComponent* C : All)
	{
		if (C && C->CreationMethod != EComponentCreationMethod::Instance)
		{
			Out.Add(C);
		}
	}
	for (UCameraComponent* C : All)
	{
		if (C && C->CreationMethod == EComponentCreationMethod::Instance)
		{
			Out.Add(C);
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
	//
	// The button state comes from Slate as well as PlayerInput: in an editor
	// PIE viewport the press that starts a drag is taken by the viewport
	// (focus / capture) and never reaches PlayerInput, while the drag's mouse
	// movement does — so IsInputKeyDown alone sees a drag only at its end.
	// Slate's pressed set is the OS (or streamed) button state regardless of
	// which widget consumed the click — but it is global, so it only counts
	// while the cursor is over the game viewport (a drag on an editor panel
	// must not orbit). "Just released" also counts so a drag that begins and
	// ends inside one frame still applies its movement.
	const bool bSlate = FSlateApplication::IsInitialized();
	float	   ViewportX = 0.0f, ViewportY = 0.0f;
	const bool bCursorOverViewport = PC->GetMousePosition(ViewportX, ViewportY);
	auto	   Held = [PC, bSlate, bCursorOverViewport](const FKey& Key) {
		return PC->IsInputKeyDown(Key) || PC->WasInputKeyJustReleased(Key)
			|| (bSlate && bCursorOverViewport && FSlateApplication::Get().GetPressedMouseButtons().Contains(Key));
	};
	const bool bOrbiting = !OrbitButton.IsValid() || Held(OrbitButton)
		|| (bAlsoOrbitWithLeftDrag && Held(EKeys::LeftMouseButton));

	// Two delta sources: the raw mouse axes (fed while the viewport receives
	// mouse moves, including a captured / hidden cursor) and the cursor's
	// position change (Slate's cursor, or the viewport's when Slate has none —
	// what a hovering cursor, a streamed client or a touch drag produces).
	// The raw axes win when they moved; the position delta is the fallback.
	float DX = 0.0f, DY = 0.0f;
	PC->GetInputMouseDelta(DX, DY);
	float CursorX = ViewportX, CursorY = ViewportY;
	bool  bHaveCursor = bCursorOverViewport;
	if (bSlate)
	{
		const FVector2D Pos = FSlateApplication::Get().GetCursorPos();
		CursorX = Pos.X;
		CursorY = Pos.Y;
		bHaveCursor = true;
	}
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
