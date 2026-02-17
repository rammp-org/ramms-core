// Copyright Epic Games, Inc. All Rights Reserved.

#include "GripperControllerComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "GameFramework/Actor.h"

UGripperControllerComponent::UGripperControllerComponent()
{
	PrimaryComponentTick.bCanEverTick = true;

	bAutoFindSkeletalMesh = true;
	GripperMeshName = FName("GripperSkMesh");
	CachedGripperMesh = nullptr;
	bEnableDebugLog = false;

	CurrentState = EGripperState::Open;
	PreviousState = EGripperState::Open;

	OpenAngle = -45.0f;
	ClosedAngle = 0.0f;
	AngleTolerance = 2.0f;

	// Initialize default motor configurations
	Finger1Motor.ConstraintName = FName("link_0_r");
	Finger1Motor.ControlAxis = EMotorAxis::Z;
	Finger1Motor.TargetAngle = OpenAngle;
	Finger1Motor.CurrentAngle = OpenAngle;
	Finger1Motor.MaxSpeed = 45.0f;
	Finger1Motor.SpeedMultiplier = 1.0f;
	Finger1Motor.MotorStrength = 100000.0f;
	Finger1Motor.MotorDamping = 10000.0f;
	Finger1Motor.bEnabled = true;
	Finger1Motor.bInvertDirection = true;

	Finger2Motor.ConstraintName = FName("link_0_l");
	Finger2Motor.ControlAxis = EMotorAxis::Z;
	Finger2Motor.TargetAngle = OpenAngle;
	Finger2Motor.CurrentAngle = OpenAngle;
	Finger2Motor.MaxSpeed = 45.0f;
	Finger2Motor.SpeedMultiplier = 1.0f;
	Finger2Motor.MotorStrength = 100000.0f;
	Finger2Motor.MotorDamping = 10000.0f;
	Finger2Motor.bEnabled = true;
}

void UGripperControllerComponent::BeginPlay()
{
	Super::BeginPlay();

	FindConstraints();

	if (bEnableDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("GripperController: Initialized"));
	}
}

void UGripperControllerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UpdateMotors(DeltaTime);
	UpdateGripperState();
	HandleStateChange();
}

void UGripperControllerComponent::Open()
{
	SetFingerAngles(OpenAngle, OpenAngle);

	if (CurrentState == EGripperState::Closed || CurrentState == EGripperState::Closing)
	{
		CurrentState = EGripperState::Opening;
	}

	if (bEnableDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("GripperController: Opening gripper (target angle: %.1f)"), OpenAngle);
	}
}

void UGripperControllerComponent::Close()
{
	SetFingerAngles(ClosedAngle, ClosedAngle);

	if (CurrentState == EGripperState::Open || CurrentState == EGripperState::Opening)
	{
		CurrentState = EGripperState::Closing;
	}

	if (bEnableDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("GripperController: Closing gripper (target angle: %.1f)"), ClosedAngle);
	}
}

void UGripperControllerComponent::Toggle()
{
	if (IsOpen() || CurrentState == EGripperState::Opening)
	{
		Close();
	}
	else
	{
		Open();
	}
}

void UGripperControllerComponent::SetGripperState(EGripperState NewState)
{
	switch (NewState)
	{
		case EGripperState::Open:
		case EGripperState::Opening:
			Open();
			break;
		case EGripperState::Closed:
		case EGripperState::Closing:
			Close();
			break;
	}
}

void UGripperControllerComponent::SetFingerAngles(float Finger1Angle, float Finger2Angle)
{
	Finger1Motor.TargetAngle = Finger1Angle;
	Finger2Motor.TargetAngle = Finger2Angle;
}

void UGripperControllerComponent::GetFingerAngles(float& OutFinger1Angle, float& OutFinger2Angle) const
{
	OutFinger1Angle = Finger1Motor.CurrentAngle;
	OutFinger2Angle = Finger2Motor.CurrentAngle;
}

void UGripperControllerComponent::SetMotorMaxSpeed(float MaxSpeed)
{
	Finger1Motor.MaxSpeed = FMath::Max(0.0f, MaxSpeed);
	Finger2Motor.MaxSpeed = FMath::Max(0.0f, MaxSpeed);
}

void UGripperControllerComponent::SetMotorSpeedMultiplier(float SpeedMultiplier)
{
	float ClampedSpeed = FMath::Clamp(SpeedMultiplier, 0.0f, 1.0f);
	Finger1Motor.SpeedMultiplier = ClampedSpeed;
	Finger2Motor.SpeedMultiplier = ClampedSpeed;
}

void UGripperControllerComponent::FindConstraints()
{
	// Find the skeletal mesh first
	if (bAutoFindSkeletalMesh || GripperMeshName != NAME_None)
	{
		CachedGripperMesh = GetOwnerSkeletalMesh();
	}

	if (!CachedGripperMesh)
	{
		if (bEnableDebugLog)
		{
			UE_LOG(LogTemp, Warning, TEXT("GripperController: No skeletal mesh found"));
		}
		return;
	}

	// Find constraints for both fingers
	Finger1Motor.CachedConstraint = CachedGripperMesh->FindConstraintInstance(Finger1Motor.ConstraintName);
	if (!Finger1Motor.CachedConstraint && bEnableDebugLog)
	{
		UE_LOG(LogTemp, Warning, TEXT("GripperController: Finger1 constraint '%s' not found"),
			*Finger1Motor.ConstraintName.ToString());
	}

	Finger2Motor.CachedConstraint = CachedGripperMesh->FindConstraintInstance(Finger2Motor.ConstraintName);
	if (!Finger2Motor.CachedConstraint && bEnableDebugLog)
	{
		UE_LOG(LogTemp, Warning, TEXT("GripperController: Finger2 constraint '%s' not found"),
			*Finger2Motor.ConstraintName.ToString());
	}

	if (bEnableDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("GripperController: Constraints found - Finger1: %s, Finger2: %s"),
			Finger1Motor.CachedConstraint ? TEXT("OK") : TEXT("MISSING"),
			Finger2Motor.CachedConstraint ? TEXT("OK") : TEXT("MISSING"));
	}
}

USkeletalMeshComponent* UGripperControllerComponent::GetOwnerSkeletalMesh()
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return nullptr;
	}

	// If a specific mesh name is provided, search for it by name
	if (GripperMeshName != NAME_None)
	{
		TArray<USkeletalMeshComponent*> SkeletalMeshComponents;
		Owner->GetComponents<USkeletalMeshComponent>(SkeletalMeshComponents);

		for (USkeletalMeshComponent* MeshComp : SkeletalMeshComponents)
		{
			if (MeshComp && MeshComp->GetFName() == GripperMeshName)
			{
				if (bEnableDebugLog)
				{
					UE_LOG(LogTemp, Log, TEXT("GripperController: Found skeletal mesh by name '%s'"),
						*GripperMeshName.ToString());
				}
				return MeshComp;
			}
		}

		if (bEnableDebugLog)
		{
			UE_LOG(LogTemp, Warning, TEXT("GripperController: Could not find skeletal mesh with name '%s'"),
				*GripperMeshName.ToString());
		}
		return nullptr;
	}

	// Fallback: try to get any skeletal mesh component
	USkeletalMeshComponent* FoundMesh = Owner->FindComponentByClass<USkeletalMeshComponent>();
	if (FoundMesh && bEnableDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("GripperController: Auto-found skeletal mesh '%s'"),
			*FoundMesh->GetFName().ToString());
	}
	return FoundMesh;
}

void UGripperControllerComponent::UpdateMotors(float DeltaTime)
{
	// Update finger 1
	if (Finger1Motor.bEnabled && Finger1Motor.CachedConstraint)
	{
		float EffectiveSpeed = Finger1Motor.MaxSpeed * Finger1Motor.SpeedMultiplier;
		float MaxDelta = EffectiveSpeed * DeltaTime;
		float AngleDiff = Finger1Motor.TargetAngle - Finger1Motor.CurrentAngle;

		if (FMath::Abs(AngleDiff) > MaxDelta)
		{
			Finger1Motor.CurrentAngle += FMath::Sign(AngleDiff) * MaxDelta;
		}
		else
		{
			Finger1Motor.CurrentAngle = Finger1Motor.TargetAngle;
		}

		ApplyMotorSettings(Finger1Motor);
	}

	// Update finger 2
	if (Finger2Motor.bEnabled && Finger2Motor.CachedConstraint)
	{
		float EffectiveSpeed = Finger2Motor.MaxSpeed * Finger2Motor.SpeedMultiplier;
		float MaxDelta = EffectiveSpeed * DeltaTime;
		float AngleDiff = Finger2Motor.TargetAngle - Finger2Motor.CurrentAngle;

		if (FMath::Abs(AngleDiff) > MaxDelta)
		{
			Finger2Motor.CurrentAngle += FMath::Sign(AngleDiff) * MaxDelta;
		}
		else
		{
			Finger2Motor.CurrentAngle = Finger2Motor.TargetAngle;
		}

		ApplyMotorSettings(Finger2Motor);
	}
}

void UGripperControllerComponent::ApplyMotorSettings(FAngularMotorConfig& Motor)
{
	if (!Motor.CachedConstraint)
	{
		return;
	}

	// Enable/disable angular drive
	Motor.CachedConstraint->SetOrientationDriveTwistAndSwing(Motor.bEnabled, Motor.bEnabled);

	if (Motor.bEnabled)
	{
		// Set motor strength and damping
		Motor.CachedConstraint->SetAngularDriveParams(Motor.MotorStrength, Motor.MotorDamping, 0.0f);

		// Apply direction inversion if needed
		float EffectiveAngle = Motor.bInvertDirection ? -Motor.CurrentAngle : Motor.CurrentAngle;

		// Set target orientation based on current interpolated angle
		FQuat TargetQuat = FQuat::Identity;
		switch (Motor.ControlAxis)
		{
			case EMotorAxis::X:
				TargetQuat = FQuat(FVector(1, 0, 0), FMath::DegreesToRadians(EffectiveAngle));
				break;
			case EMotorAxis::Y:
				TargetQuat = FQuat(FVector(0, 1, 0), FMath::DegreesToRadians(EffectiveAngle));
				break;
			case EMotorAxis::Z:
				TargetQuat = FQuat(FVector(0, 0, 1), FMath::DegreesToRadians(EffectiveAngle));
				break;
		}

		Motor.CachedConstraint->SetAngularOrientationTarget(TargetQuat);

		if (bEnableDebugLog)
		{
			// Read actual constraint angle
			float ActualAngle = 0.0f;
			switch (Motor.ControlAxis)
			{
				case EMotorAxis::X:
					ActualAngle = Motor.CachedConstraint->GetCurrentSwing1();
					break;
				case EMotorAxis::Y:
					ActualAngle = Motor.CachedConstraint->GetCurrentSwing2();
					break;
				case EMotorAxis::Z:
					ActualAngle = Motor.CachedConstraint->GetCurrentTwist();
					break;
			}

			UE_LOG(LogTemp, Warning, TEXT("GripperController: %s - Target=%.1f°, Current=%.1f°, Actual=%.1f°, Inverted=%d"),
				*Motor.ConstraintName.ToString(), Motor.TargetAngle, Motor.CurrentAngle, ActualAngle, Motor.bInvertDirection);
		}
	}
}

void UGripperControllerComponent::UpdateGripperState()
{
	// Check if both fingers are at their target positions
	float Finger1Diff = FMath::Abs(Finger1Motor.CurrentAngle - Finger1Motor.TargetAngle);
	float Finger2Diff = FMath::Abs(Finger2Motor.CurrentAngle - Finger2Motor.TargetAngle);

	bool bAtTarget = (Finger1Diff < AngleTolerance) && (Finger2Diff < AngleTolerance);

	// Determine if we're targeting open or closed position
	bool bTargetingOpen = FMath::Abs(Finger1Motor.TargetAngle - OpenAngle) < AngleTolerance;
	bool bTargetingClosed = FMath::Abs(Finger1Motor.TargetAngle - ClosedAngle) < AngleTolerance;

	// Update state based on current position and target
	if (bAtTarget)
	{
		if (bTargetingOpen)
		{
			CurrentState = EGripperState::Open;
		}
		else if (bTargetingClosed)
		{
			CurrentState = EGripperState::Closed;
		}
	}
	else
	{
		if (bTargetingOpen)
		{
			CurrentState = EGripperState::Opening;
		}
		else if (bTargetingClosed)
		{
			CurrentState = EGripperState::Closing;
		}
	}
}

void UGripperControllerComponent::HandleStateChange()
{
	if (CurrentState != PreviousState)
	{
		// Broadcast state changed event
		OnGripperStateChanged.Broadcast(CurrentState);

		// Broadcast specific events
		if (CurrentState == EGripperState::Open)
		{
			OnGripperOpened.Broadcast();

			if (bEnableDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("GripperController: Gripper opened"));
			}
		}
		else if (CurrentState == EGripperState::Closed)
		{
			OnGripperClosed.Broadcast();

			if (bEnableDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("GripperController: Gripper closed"));
			}
		}

		PreviousState = CurrentState;
	}
}
