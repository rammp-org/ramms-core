// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsEndEffectorTeleopComponent.h"

#include "Camera/CameraComponent.h"
#include "KinovaGen3ControllerComponent.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "GripperControllerComponent.h"
#include "InputCoreTypes.h"

URammsEndEffectorTeleopComponent::URammsEndEffectorTeleopComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void URammsEndEffectorTeleopComponent::BeginPlay()
{
	Super::BeginPlay();

	ResolveKinovaController();
	ResolveGripperController();
	ResolveFollowCameraComponents();
	if (KinovaControllerComponent)
	{
		KinovaControllerComponent->AddTickPrerequisiteComponent(this);
	}
	if (bAutoSyncTargetToCurrentPoseOnBeginPlay)
	{
		SyncTargetToCurrentPose();
	}
	// Frame preference lands AFTER the initial sync: the sync commits the live EE
	// pose in whatever frame is active, and the switch then converts it preserving
	// the world pose. Doing it first would convert an uninitialized (identity)
	// target through a possibly not-yet-resolved base — a target at the chassis
	// origin for the first frames.
	if (KinovaControllerComponent)
	{
		KinovaControllerComponent->SetEndEffectorTargetFrame(
			bTargetRidesBase ? EEndEffectorTargetFrame::Base : EEndEffectorTargetFrame::World);
	}
}

void URammsEndEffectorTeleopComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Control-surface rate axes (panels, input maps, remote clients) integrate
	// here at the teleop speeds, independent of the legacy key polling below.
	if (bTeleopEnabled && (!ControlLinear.IsNearlyZero() || !ControlAngular.IsNearlyZero()))
	{
		FVector Linear = ControlLinear;
		Linear.X *= ForwardSign;
		Linear.Y *= StrafeSign;
		Linear.Z *= UpSign;
		FRotator Angular = ControlAngular;
		Angular.Pitch *= PitchSign;
		Angular.Yaw *= YawSign;
		Angular.Roll *= RollSign;
		ApplyTeleopInput(Linear, Angular, DeltaTime, 1.0f);
	}

	if (!bTeleopEnabled || !bEnableKeyboardMouseTeleop)
	{
		if (bEnableFollowCamera)
		{
			UpdateFollowCamera();
		}
		if (bEnableDebugDisplay)
		{
			DrawDebugVisualization();
		}
		return;
	}

	ApplyKeyboardMouseTeleop(DeltaTime);

	if (bEnableFollowCamera)
	{
		UpdateFollowCamera();
	}

	if (bEnableDebugDisplay)
	{
		DrawDebugVisualization();
	}
}

void URammsEndEffectorTeleopComponent::ApplyTeleopInput(const FVector& LinearInput, const FRotator& AngularInput, float DeltaTimeSeconds, float SpeedScale)
{
	if (UKinovaGen3ControllerComponent* Controller = ResolveKinovaController())
	{
		if (bForceEndEffectorControlMode)
		{
			Controller->ArmControlMode = EArmControlMode::EndEffectorControl;
		}
		// Re-assert the frame preference (no-op when unchanged): a programmatic
		// world-target call may have switched the controller to World in between.
		Controller->SetEndEffectorTargetFrame(
			bTargetRidesBase ? EEndEffectorTargetFrame::Base : EEndEffectorTargetFrame::World);

		const float SafeScale = FMath::Max(0.0f, SpeedScale);
		Controller->ApplyEndEffectorTeleopInput(
			LinearInput,
			AngularInput,
			DeltaTimeSeconds,
			bInputInLocalFrame,
			LinearSpeedCmPerSecond * SafeScale,
			AngularSpeedDegPerSecond * SafeScale);
	}
}

void URammsEndEffectorTeleopComponent::OpenGripper()
{
	if (UGripperControllerComponent* Controller = ResolveGripperController())
	{
		Controller->Open();
	}
}

void URammsEndEffectorTeleopComponent::CloseGripper()
{
	if (UGripperControllerComponent* Controller = ResolveGripperController())
	{
		Controller->Close();
	}
}

void URammsEndEffectorTeleopComponent::ToggleGripper()
{
	if (UGripperControllerComponent* Controller = ResolveGripperController())
	{
		Controller->Toggle();
	}
}

void URammsEndEffectorTeleopComponent::DrawDebugVisualization() const
{
	if (!KinovaControllerComponent)
	{
		return;
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	const FTransform TargetTransform = KinovaControllerComponent->GetEndEffectorTargetTransform();
	const FVector	 TargetLocation = TargetTransform.GetLocation();
	const FQuat		 TargetRotation = TargetTransform.GetRotation();
	const bool		 bPersistent = false;
	const float		 LifeTime = 0.0f;

	DrawDebugSphere(World, TargetLocation, DebugTargetSphereRadius, 12, DebugTargetColor, bPersistent, LifeTime, 0, 2.5f);
	FString TargetLabel = bTeleopEnabled ? TEXT("Teleop Target") : TEXT("Teleop Target (disabled)");
	if (GripperControllerComponent)
	{
		if (const UEnum* GripperStateEnum = StaticEnum<EGripperState>())
		{
			const FString GripperStateName = GripperStateEnum->GetNameStringByValue(static_cast<int64>(GripperControllerComponent->GetGripperState()));
			TargetLabel += FString::Printf(TEXT(" | Gripper: %s"), *GripperStateName);
		}
	}
	DrawDebugString(World, TargetLocation + FVector(0.0f, 0.0f, DebugAxisLength * 0.6f),
		*TargetLabel,
		nullptr, DebugTargetColor, LifeTime, true, 1.0f);

	// IK diagnostics: reveal whether the controller is actually in IK mode, whether a
	// stray TargetActor is hijacking the target, and the live solver status. This turns
	// "the arm won't follow" into a precise reading of where the chain breaks.
	{
		const bool	  bInEndEffectorMode = KinovaControllerComponent->ArmControlMode == EArmControlMode::EndEffectorControl;
		const bool	  bTargetActorHijack = KinovaControllerComponent->TargetActor != nullptr;
		const FVector ActualEELocation = KinovaControllerComponent->GetLiveEndEffectorTransform().GetLocation();
		const float	  TargetErrorCm = FVector::Dist(ActualEELocation, TargetLocation);

		// Did the controller actually run a solve since we last drew? If the solver is
		// "latched satisfied" it stops incrementing IKSolveCount even while the target
		// is far away — that distinguishes the satisfied-latch from a drive/limit problem.
		const int32 SolveCount = KinovaControllerComponent->IKSolveCount;
		const bool	bSolverActive = (LastObservedIKSolveCount >= 0) && (SolveCount != LastObservedIKSolveCount);
		LastObservedIKSolveCount = SolveCount;

		const float	 ActualErrCm = KinovaControllerComponent->LastActualPosError;
		const bool	 bStalled = bInEndEffectorMode && !bSolverActive && (ActualErrCm > KinovaControllerComponent->IKPositionTolerance);
		const FColor DiagColor = (!bInEndEffectorMode || bTargetActorHijack || bStalled)
			? FColor::Orange
			: FColor::Green;

		FString DiagLabel = FString::Printf(
			TEXT("Mode: %s%s | Solver: %s | IK: %s err=%.2fcm iters=%d | Target<->EE: %.2fcm (ctrl actual %.2fcm)"),
			bInEndEffectorMode ? TEXT("EndEffector") : TEXT("JOINT (not IK!)"),
			bTargetActorHijack ? TEXT(" | TargetActor SET (overrides teleop!)") : TEXT(""),
			bSolverActive ? TEXT("ACTIVE") : (bStalled ? TEXT("IDLE-LATCHED!") : TEXT("idle")),
			KinovaControllerComponent->bLastIKSuccess ? TEXT("ok") : TEXT("FAIL"),
			KinovaControllerComponent->LastIKPositionError,
			KinovaControllerComponent->LastIKIterations,
			TargetErrorCm,
			ActualErrCm);

		DrawDebugString(World, TargetLocation + FVector(0.0f, 0.0f, DebugAxisLength * 1.0f),
			*DiagLabel, nullptr, DiagColor, LifeTime, true, 1.0f);
	}

	if (bDrawTargetFrame)
	{
		DrawDebugLine(World, TargetLocation, TargetLocation + TargetRotation.GetAxisX() * DebugAxisLength, FColor::Red, bPersistent, LifeTime, 0, 2.0f);
		DrawDebugLine(World, TargetLocation, TargetLocation + TargetRotation.GetAxisY() * DebugAxisLength, FColor::Green, bPersistent, LifeTime, 0, 2.0f);
		DrawDebugLine(World, TargetLocation, TargetLocation + TargetRotation.GetAxisZ() * DebugAxisLength, FColor::Blue, bPersistent, LifeTime, 0, 2.0f);
	}

	if (bDrawCurrentEndEffector || bDrawTargetErrorLine)
	{
		// Live socket read: this component ticks BEFORE the controller, so the
		// cached EndEffectorState is a frame stale and visibly trails the arm
		// whenever the vehicle moves.
		const FTransform LiveEE = KinovaControllerComponent->GetLiveEndEffectorTransform();
		const FVector	 CurrentLocation = LiveEE.GetLocation();
		const FQuat		 CurrentRotation = LiveEE.GetRotation();

		if (bDrawCurrentEndEffector)
		{
			DrawDebugSphere(World, CurrentLocation, DebugTargetSphereRadius * 0.75f, 10, DebugCurrentPoseColor, bPersistent, LifeTime, 0, 2.0f);
			DrawDebugString(World, CurrentLocation + FVector(0.0f, 0.0f, DebugAxisLength * 0.35f),
				TEXT("Current EE"), nullptr, DebugCurrentPoseColor, LifeTime, true, 0.9f);

			if (bDrawTargetFrame)
			{
				const float CurrentAxisLength = DebugAxisLength * 0.75f;
				DrawDebugLine(World, CurrentLocation, CurrentLocation + CurrentRotation.GetAxisX() * CurrentAxisLength, FColor(255, 128, 128), bPersistent, LifeTime, 0, 1.5f);
				DrawDebugLine(World, CurrentLocation, CurrentLocation + CurrentRotation.GetAxisY() * CurrentAxisLength, FColor(128, 255, 128), bPersistent, LifeTime, 0, 1.5f);
				DrawDebugLine(World, CurrentLocation, CurrentLocation + CurrentRotation.GetAxisZ() * CurrentAxisLength, FColor(128, 128, 255), bPersistent, LifeTime, 0, 1.5f);
			}
		}

		if (bDrawTargetErrorLine)
		{
			DrawDebugLine(World, CurrentLocation, TargetLocation, DebugTargetColor, bPersistent, LifeTime, 0, 1.0f);
		}
	}
}

void URammsEndEffectorTeleopComponent::SyncTargetToCurrentPose()
{
	if (UKinovaGen3ControllerComponent* Controller = ResolveKinovaController())
	{
		if (bForceEndEffectorControlMode)
		{
			Controller->ArmControlMode = EArmControlMode::EndEffectorControl;
		}

		Controller->SnapEndEffectorTargetToCurrentPose();
	}
}

void URammsEndEffectorTeleopComponent::SetTeleopEnabled(bool bEnabled)
{
	const bool bWasEnabled = bTeleopEnabled;
	bTeleopEnabled = bEnabled;

	if (!bWasEnabled && bTeleopEnabled)
	{
		SyncTargetToCurrentPose();
	}
}

void URammsEndEffectorTeleopComponent::ToggleTeleopEnabled()
{
	SetTeleopEnabled(!bTeleopEnabled);
}

UKinovaGen3ControllerComponent* URammsEndEffectorTeleopComponent::ResolveKinovaController()
{
	if (KinovaControllerComponent)
	{
		return KinovaControllerComponent;
	}

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return nullptr;
	}

	TArray<UKinovaGen3ControllerComponent*> Controllers;
	Owner->GetComponents<UKinovaGen3ControllerComponent>(Controllers);

	for (UKinovaGen3ControllerComponent* Controller : Controllers)
	{
		if (!Controller)
		{
			continue;
		}

		if (KinovaControllerComponentName == NAME_None || Controller->GetFName() == KinovaControllerComponentName)
		{
			KinovaControllerComponent = Controller;
			bLoggedMissingController = false;
			break;
		}
	}

	if (!KinovaControllerComponent && !bLoggedMissingController)
	{
		bLoggedMissingController = true;
		UE_LOG(LogTemp, Warning, TEXT("[RammsTeleop] %s could not find a KinovaGen3ControllerComponent on %s"),
			*GetName(),
			*GetNameSafe(Owner));
	}

	return KinovaControllerComponent;
}

UGripperControllerComponent* URammsEndEffectorTeleopComponent::ResolveGripperController()
{
	if (GripperControllerComponent)
	{
		return GripperControllerComponent;
	}

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return nullptr;
	}

	TArray<UGripperControllerComponent*> Controllers;
	Owner->GetComponents<UGripperControllerComponent>(Controllers);

	for (UGripperControllerComponent* Controller : Controllers)
	{
		if (!Controller)
		{
			continue;
		}

		if (GripperControllerComponentName == NAME_None || Controller->GetFName() == GripperControllerComponentName)
		{
			GripperControllerComponent = Controller;
			bLoggedMissingGripperController = false;
			break;
		}
	}

	if (!GripperControllerComponent && bEnableGripperTeleop && !bLoggedMissingGripperController)
	{
		bLoggedMissingGripperController = true;
		UE_LOG(LogTemp, Warning, TEXT("[RammsTeleop] %s could not find a GripperControllerComponent on %s"),
			*GetName(),
			*GetNameSafe(Owner));
	}

	return GripperControllerComponent;
}

void URammsEndEffectorTeleopComponent::ResolveFollowCameraComponents()
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	if (!FollowSpringArmComponent)
	{
		TArray<USpringArmComponent*> SpringArms;
		Owner->GetComponents<USpringArmComponent>(SpringArms);
		for (USpringArmComponent* SpringArm : SpringArms)
		{
			if (!SpringArm)
			{
				continue;
			}

			if (FollowSpringArmComponentName == NAME_None || SpringArm->GetFName() == FollowSpringArmComponentName)
			{
				FollowSpringArmComponent = SpringArm;
				break;
			}
		}
	}

	if (!FollowCameraComponent)
	{
		TArray<UCameraComponent*> Cameras;
		Owner->GetComponents<UCameraComponent>(Cameras);
		for (UCameraComponent* Camera : Cameras)
		{
			if (!Camera)
			{
				continue;
			}

			if (FollowCameraComponentName == NAME_None || Camera->GetFName() == FollowCameraComponentName)
			{
				FollowCameraComponent = Camera;
				break;
			}
		}
	}

	if ((!FollowSpringArmComponent || (bAutoActivateFollowCameraView && !FollowCameraComponent))
		&& bEnableFollowCamera
		&& !bLoggedMissingFollowCamera)
	{
		bLoggedMissingFollowCamera = true;
		UE_LOG(LogTemp, Warning, TEXT("[RammsTeleop] %s could not resolve the requested spring arm / camera components on %s"),
			*GetName(),
			*GetNameSafe(Owner));
	}
}

void URammsEndEffectorTeleopComponent::UpdateFollowCamera()
{
	if (!ResolveKinovaController())
	{
		return;
	}

	ResolveFollowCameraComponents();
	if (!FollowSpringArmComponent)
	{
		return;
	}

	const FTransform TargetTransform = KinovaControllerComponent->GetEndEffectorTargetTransform();
	const FVector	 RootLocation = bFollowCameraOffsetInLocalFrame
		   ? TargetTransform.TransformPosition(FollowCameraTargetOffset)
		   : (TargetTransform.GetLocation() + FollowCameraTargetOffset);
	const FQuat		 DesiredRotation = bFollowCameraTargetRotation
			 ? (TargetTransform.GetRotation() * FollowCameraRotationOffset.Quaternion()).GetNormalized()
			 : FollowCameraRotationOffset.Quaternion();

	FollowSpringArmComponent->SetWorldLocationAndRotation(RootLocation, DesiredRotation);

	if (bAutoActivateFollowCameraView && !bFollowCameraViewActivated && FollowCameraComponent)
	{
		if (APlayerController* PlayerController = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr)
		{
			FollowCameraComponent->SetActive(true);
			PlayerController->SetViewTargetWithBlend(GetOwner(), 0.15f);
			bFollowCameraViewActivated = true;
		}
	}
}

void URammsEndEffectorTeleopComponent::ApplyKeyboardMouseTeleop(float DeltaTime)
{
	UKinovaGen3ControllerComponent* Controller = ResolveKinovaController();
	if (!Controller)
	{
		return;
	}

	UWorld*			   World = GetWorld();
	APlayerController* PlayerController = World ? World->GetFirstPlayerController() : nullptr;
	if (!PlayerController || !PlayerController->IsLocalPlayerController())
	{
		return;
	}

	float SpeedScale = 1.0f;
	if (PlayerController->IsInputKeyDown(EKeys::LeftShift) || PlayerController->IsInputKeyDown(EKeys::RightShift))
	{
		SpeedScale *= FastSpeedMultiplier;
	}
	if (PlayerController->IsInputKeyDown(EKeys::LeftControl) || PlayerController->IsInputKeyDown(EKeys::RightControl))
	{
		SpeedScale *= SlowSpeedMultiplier;
	}

	// Semantic inputs (intent) via configurable key pairs, matching URammsMjArmTeleopComponent. A valid
	// FKey that isn't pressed reads 0, and an unset (None) key never contributes.
	auto Axis = [PlayerController](const FKey& Pos, const FKey& Neg) {
		const float p = (Pos.IsValid() && PlayerController->IsInputKeyDown(Pos)) ? 1.f : 0.f;
		const float n = (Neg.IsValid() && PlayerController->IsInputKeyDown(Neg)) ? 1.f : 0.f;
		return p - n;
	};
	const float Fwd = Axis(ForwardKey, BackwardKey);		  // forward / back
	const float Strafe = Axis(StrafeRightKey, StrafeLeftKey); // right / left
	const float Up = Axis(UpKey, DownKey);					  // up / down
	const float Yaw = Axis(YawRightKey, YawLeftKey);
	const float Pitch = Axis(PitchUpKey, PitchDownKey);
	const float Roll = Axis(RollRightKey, RollLeftKey);

	// End-effector frame: forward = X, strafe = Y, up = Z. Per-axis signs flip any inverted direction.
	FVector LinearInput = FVector::ZeroVector;
	LinearInput.X = Fwd * ForwardSign;
	LinearInput.Y = Strafe * StrafeSign;
	LinearInput.Z = Up * UpSign;

	FRotator AngularInput = FRotator::ZeroRotator;
	AngularInput.Pitch = Pitch * PitchSign;
	AngularInput.Yaw = Yaw * YawSign;
	AngularInput.Roll = Roll * RollSign;

	if (ResyncTargetKey.IsValid() && PlayerController->WasInputKeyJustPressed(ResyncTargetKey))
	{
		SyncTargetToCurrentPose();
	}

	if (bEnableGripperTeleop)
	{
		if (OpenGripperKey.IsValid() && PlayerController->WasInputKeyJustPressed(OpenGripperKey))
		{
			OpenGripper();
		}
		if (CloseGripperKey.IsValid() && PlayerController->WasInputKeyJustPressed(CloseGripperKey))
		{
			CloseGripper();
		}
		if (ToggleGripperKey.IsValid() && PlayerController->WasInputKeyJustPressed(ToggleGripperKey))
		{
			ToggleGripper();
		}
	}

	if (!LinearInput.IsNearlyZero() || !AngularInput.IsNearlyZero())
	{
		ApplyTeleopInput(LinearInput, AngularInput, DeltaTime, SpeedScale);
	}

	const bool bAllowMouseRotation = bEnableMouseRotation
		&& (!bRequireRightMouseButtonForMouseRotation || PlayerController->IsInputKeyDown(EKeys::RightMouseButton));
	if (!bAllowMouseRotation)
	{
		return;
	}

	float MouseDeltaX = 0.0f;
	float MouseDeltaY = 0.0f;
	PlayerController->GetInputMouseDelta(MouseDeltaX, MouseDeltaY);

	if (FMath::IsNearlyZero(MouseDeltaX) && FMath::IsNearlyZero(MouseDeltaY))
	{
		SmoothedMouseDeltaX = 0.0f;
		SmoothedMouseDeltaY = 0.0f;
		return;
	}

	if (bForceEndEffectorControlMode)
	{
		Controller->ArmControlMode = EArmControlMode::EndEffectorControl;
	}

	// Low-pass the raw mouse delta to strip per-pixel sensor noise before it drives the target.
	const float MouseAlpha = FMath::Clamp(MouseSmoothingAlpha, 0.01f, 1.0f);
	SmoothedMouseDeltaX = FMath::Lerp(SmoothedMouseDeltaX, MouseDeltaX, MouseAlpha);
	SmoothedMouseDeltaY = FMath::Lerp(SmoothedMouseDeltaY, MouseDeltaY, MouseAlpha);
	MouseDeltaX = SmoothedMouseDeltaX;
	MouseDeltaY = SmoothedMouseDeltaY;

	const float	   MousePitchDir = bInvertMouseY ? 1.0f : -1.0f;
	const FRotator MouseRotationDelta(
		MousePitchDir * PitchSign * MouseDeltaY * MousePitchDegreesPerPixel * SpeedScale,
		YawSign * MouseDeltaX * MouseYawDegreesPerPixel * SpeedScale,
		0.0f);
	if (bInputInLocalFrame)
	{
		Controller->RotateEndEffectorTargetByLocal(MouseRotationDelta);
	}
	else
	{
		Controller->RotateEndEffectorTargetBy(MouseRotationDelta);
	}
}

// --- control surface -----------------------------------------------------------

namespace
{
	const FName ArmForward(TEXT("arm.forward"));
	const FName ArmStrafe(TEXT("arm.strafe"));
	const FName ArmUp(TEXT("arm.up"));
	const FName ArmYaw(TEXT("arm.yaw"));
	const FName ArmPitch(TEXT("arm.pitch"));
	const FName ArmRoll(TEXT("arm.roll"));
	const FName ArmResync(TEXT("arm.resync"));
	const FName GripperOpen(TEXT("gripper.open"));
	const FName GripperClose(TEXT("gripper.close"));
	const FName GripperToggle(TEXT("gripper.toggle"));
	const FName GripperClosed(TEXT("gripper.closed"));
} // namespace

void URammsEndEffectorTeleopComponent::DescribeControls(FRammsControlSurface& OutSurface) const
{
	if (!KinovaControllerComponent)
	{
		return;
	}
	auto Rate = [&OutSurface](FName Id, const TCHAR* Name, FName Paired, int32 Order) {
		FRammsControlAxis Axis;
		Axis.Id = Id;
		Axis.Group = FName("Arm");
		Axis.DisplayName = FText::FromString(Name);
		Axis.Kind = ERammsControlKind::Continuous;
		Axis.Units = ERammsControlUnits::Normalized;
		Axis.Range = FVector2D(-1.0, 1.0);
		Axis.PairedAxis = Paired;
		Axis.Order = Order;
		OutSurface.Add(Axis);
	};
	Rate(ArmForward, TEXT("Forward"), ArmStrafe, 0);
	Rate(ArmStrafe, TEXT("Strafe"), ArmForward, 1);
	Rate(ArmUp, TEXT("Up"), NAME_None, 2);
	Rate(ArmPitch, TEXT("Pitch"), ArmYaw, 3); // paired: lower Order = the joystick's vertical axis
	Rate(ArmYaw, TEXT("Yaw"), ArmPitch, 4);
	Rate(ArmRoll, TEXT("Roll"), NAME_None, 5);

	FRammsControlAxis Resync;
	Resync.Id = ArmResync;
	Resync.Group = FName("Arm");
	Resync.DisplayName = FText::FromString(TEXT("Resync target"));
	Resync.Kind = ERammsControlKind::Action;
	Resync.Units = ERammsControlUnits::None;
	Resync.Order = 6;
	OutSurface.Add(Resync);

	if (GripperControllerComponent)
	{
		auto Action = [&OutSurface](FName Id, const TCHAR* Name, int32 Order) {
			FRammsControlAxis Axis;
			Axis.Id = Id;
			Axis.Group = FName("Gripper");
			Axis.DisplayName = FText::FromString(Name);
			Axis.Kind = ERammsControlKind::Action;
			Axis.Units = ERammsControlUnits::None;
			Axis.Order = Order;
			OutSurface.Add(Axis);
		};
		Action(GripperOpen, TEXT("Open"), 0);
		Action(GripperClose, TEXT("Close"), 1);
		Action(GripperToggle, TEXT("Toggle"), 2);

		// Readback-only state (0 = open, 1 = closed) for panels / status.
		FRammsControlAxis Closed;
		Closed.Id = GripperClosed;
		Closed.Group = FName("Gripper");
		Closed.DisplayName = FText::FromString(TEXT("Closed"));
		Closed.Kind = ERammsControlKind::Position;
		Closed.Units = ERammsControlUnits::Normalized;
		Closed.Range = FVector2D(0.0, 1.0);
		Closed.bReadback = true;
		Closed.bReadOnly = true; // state only: change it through the actions
		Closed.Order = 3;
		OutSurface.Add(Closed);
	}
}

bool URammsEndEffectorTeleopComponent::ApplyControl(FName Id, float Value)
{
	if (!bTeleopEnabled || !KinovaControllerComponent)
	{
		return false;
	}
	const float V = FMath::Clamp(Value, -1.0f, 1.0f);
	if (Id == ArmForward)
	{
		ControlLinear.X = V;
	}
	else if (Id == ArmStrafe)
	{
		ControlLinear.Y = V;
	}
	else if (Id == ArmUp)
	{
		ControlLinear.Z = V;
	}
	else if (Id == ArmYaw)
	{
		ControlAngular.Yaw = V;
	}
	else if (Id == ArmPitch)
	{
		ControlAngular.Pitch = V;
	}
	else if (Id == ArmRoll)
	{
		ControlAngular.Roll = V;
	}
	else
	{
		return false; // gripper.closed is readback-only: use the gripper actions
	}
	return true;
}

bool URammsEndEffectorTeleopComponent::TriggerControl(FName Id)
{
	if (Id == ArmResync)
	{
		SyncTargetToCurrentPose();
		return KinovaControllerComponent != nullptr;
	}
	if (!GripperControllerComponent)
	{
		return false;
	}
	if (Id == GripperOpen)
	{
		OpenGripper();
	}
	else if (Id == GripperClose)
	{
		CloseGripper();
	}
	else if (Id == GripperToggle)
	{
		ToggleGripper();
	}
	else
	{
		return false;
	}
	return true;
}

bool URammsEndEffectorTeleopComponent::ReleaseControl(FName Id)
{
	// Rate axes spring to zero; the arm holds wherever the target is.
	return ApplyControl(Id, 0.0f);
}

bool URammsEndEffectorTeleopComponent::ReadControl(FName Id, float& OutValue) const
{
	if (Id == GripperClosed && GripperControllerComponent)
	{
		// Commanded state: closing counts as closed so a panel reflects the
		// press immediately rather than when the fingers stall.
		const EGripperState State = GripperControllerComponent->GetGripperState();
		OutValue = (State == EGripperState::Closed || State == EGripperState::Closing) ? 1.0f : 0.0f;
		return true;
	}
	if (Id == ArmForward || Id == ArmStrafe || Id == ArmUp)
	{
		OutValue = Id == ArmForward ? ControlLinear.X : (Id == ArmStrafe ? ControlLinear.Y : ControlLinear.Z);
		return true;
	}
	if (Id == ArmYaw || Id == ArmPitch || Id == ArmRoll)
	{
		OutValue = Id == ArmYaw ? ControlAngular.Yaw : (Id == ArmPitch ? ControlAngular.Pitch : ControlAngular.Roll);
		return true;
	}
	return false;
}
