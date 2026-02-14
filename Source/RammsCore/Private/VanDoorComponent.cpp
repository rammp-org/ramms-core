// Copyright Epic Games, Inc. All Rights Reserved.

#include "VanDoorComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "Curves/CurveFloat.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "DrawDebugHelpers.h"

UVanDoorComponent::UVanDoorComponent()
{
	PrimaryComponentTick.bCanEverTick = true;

	ConstraintName = FName("door");
	bAutoFindSkeletalMesh = true;
	DoorAnimationDuration = 2.0f;
	DoorState = EDoorState::Closed;
	CurrentAnimationTime = 0.0f;
	MotorStrength = 100000.0f;
	bShowKeyframePath = false;
	bShowKeyframeGizmos = false;
	GizmoSize = 50.0f;
	bEnableDebugLog = false;
	CachedSkeletalMesh = nullptr;
	CachedConstraint = nullptr;

	// Default keyframes - typical van sliding door motion
	// Move out from body (Y-axis), then slide back (X-axis)
	DoorKeyframes.Empty();
	DoorKeyframes.Add(FDoorKeyframe(0.0f, FVector::ZeroVector, FRotator::ZeroRotator));
	DoorKeyframes.Add(FDoorKeyframe(0.4f, FVector(0.0f, 15.0f, 0.0f), FRotator::ZeroRotator));
	DoorKeyframes.Add(FDoorKeyframe(1.0f, FVector(-135.0f, 15.0f, 0.0f), FRotator::ZeroRotator));
}

void UVanDoorComponent::BeginPlay()
{
	Super::BeginPlay();

	// Find the skeletal mesh
	if (bAutoFindSkeletalMesh)
	{
		CachedSkeletalMesh = GetOwnerSkeletalMesh();
	}

	if (CachedSkeletalMesh)
	{
		if (bEnableDebugLog)
		{
			UE_LOG(LogTemp, Log, TEXT("VanDoorComponent: Found skeletal mesh '%s'"), *CachedSkeletalMesh->GetName());
		}

		// Find the constraint in the physics asset
		CachedConstraint = CachedSkeletalMesh->FindConstraintInstance(ConstraintName);

		if (CachedConstraint)
		{
			if (bEnableDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("VanDoorComponent: Found constraint '%s' in physics asset"), *ConstraintName.ToString());
			}

			// Unlock all linear axes so the door can move
			CachedConstraint->SetLinearXMotion(ELinearConstraintMotion::LCM_Free);
			CachedConstraint->SetLinearYMotion(ELinearConstraintMotion::LCM_Free);
			CachedConstraint->SetLinearZMotion(ELinearConstraintMotion::LCM_Free);

			// Enable position and velocity drives on all linear axes
			CachedConstraint->SetLinearPositionDrive(true, true, true);
			CachedConstraint->SetLinearVelocityDrive(true, true, true);

			// Configure motor strength
			CachedConstraint->SetLinearDriveParams(MotorStrength, MotorStrength * 0.1f, 0.0f);

			// Set initial target to closed position
			CachedConstraint->SetLinearPositionTarget(FVector::ZeroVector);
			CachedConstraint->SetLinearVelocityTarget(FVector::ZeroVector);

			if (bEnableDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("VanDoorComponent: Motor configured - Strength: %.0f, Keyframes: %d"), 
					MotorStrength, DoorKeyframes.Num());
			}
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("VanDoorComponent: Constraint '%s' not found in physics asset on '%s'"), 
				*ConstraintName.ToString(), *GetOwner()->GetName());
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("VanDoorComponent: No SkeletalMeshComponent found on '%s'"), 
			*GetOwner()->GetName());
	}
}

void UVanDoorComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

	// Clear all debug draws
	if (UWorld* World = GetWorld())
	{
		FlushPersistentDebugLines(World);
	}
}

void UVanDoorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Draw debug visualization
	DrawDebugVisualization();

	if (DoorState == EDoorState::Opening || DoorState == EDoorState::Closing)
	{
		float Direction = (DoorState == EDoorState::Opening) ? 1.0f : -1.0f;
		CurrentAnimationTime += DeltaTime * Direction;

		// Clamp animation time
		bool bReachedTarget = false;
		if (CurrentAnimationTime >= DoorAnimationDuration)
		{
			CurrentAnimationTime = DoorAnimationDuration;
			bReachedTarget = true;
		}
		else if (CurrentAnimationTime <= 0.0f)
		{
			CurrentAnimationTime = 0.0f;
			bReachedTarget = true;
		}

		UpdateDoorConstraint();

		// Check if we've reached the target
		if (bReachedTarget)
		{
			float NormalizedTime = DoorAnimationDuration > 0.0f ? CurrentAnimationTime / DoorAnimationDuration : 0.0f;
			FVector TargetLinear;
			FRotator TargetAngular;
			GetKeyframePositions(NormalizedTime, TargetLinear, TargetAngular);

			if (IsAtTargetPosition(TargetLinear))
			{
				if (DoorState == EDoorState::Opening)
				{
					DoorState = EDoorState::Open;
					if (bEnableDebugLog)
					{
						UE_LOG(LogTemp, Log, TEXT("VanDoorComponent: Door fully OPENED"));
					}
					OnDoorOpened.Broadcast();
				}
				else
				{
					DoorState = EDoorState::Closed;
					if (bEnableDebugLog)
					{
						UE_LOG(LogTemp, Log, TEXT("VanDoorComponent: Door fully CLOSED"));
					}
					OnDoorClosed.Broadcast();
				}
			}
		}
	}
}

void UVanDoorComponent::InteractWithDoor()
{
	if (DoorState == EDoorState::Closed)
	{
		OpenDoor();
	}
	else if (DoorState == EDoorState::Open)
	{
		CloseDoor();
	}
}

void UVanDoorComponent::OpenDoor()
{
	if (DoorState == EDoorState::Closed || DoorState == EDoorState::Closing)
	{
		DoorState = EDoorState::Opening;
		// Ensure physics wakes up and target is applied immediately
		if (CachedSkeletalMesh)
		{
			CachedSkeletalMesh->WakeAllRigidBodies();
		}
		UpdateDoorConstraint();
		if (bEnableDebugLog)
		{
			UE_LOG(LogTemp, Log, TEXT("VanDoorComponent: Starting to OPEN door"));
		}
	}
}

void UVanDoorComponent::CloseDoor()
{
	if (DoorState == EDoorState::Open || DoorState == EDoorState::Opening)
	{
		DoorState = EDoorState::Closing;
		// Ensure physics wakes up and target is applied immediately
		if (CachedSkeletalMesh)
		{
			CachedSkeletalMesh->WakeAllRigidBodies();
		}
		UpdateDoorConstraint();
		if (bEnableDebugLog)
		{
			UE_LOG(LogTemp, Log, TEXT("VanDoorComponent: Starting to CLOSE door"));
		}
	}
}

void UVanDoorComponent::UpdateDoorConstraint()
{
	if (!CachedConstraint)
	{
		return;
	}

	// Calculate normalized time (0 to 1) based on current animation time
	float NormalizedTime = DoorAnimationDuration > 0.0f ? CurrentAnimationTime / DoorAnimationDuration : 0.0f;

	// Apply optional easing curve
	if (DoorAnimationCurve)
	{
		NormalizedTime = DoorAnimationCurve->GetFloatValue(NormalizedTime);
	}

	// Get target position from keyframes
	FVector TargetLinear;
	FRotator TargetAngular;
	GetKeyframePositions(NormalizedTime, TargetLinear, TargetAngular);

	if (bEnableDebugLog)
	{
		UE_LOG(LogTemp, Verbose, TEXT("VanDoorComponent: Setting target position: %s (normalized time: %.2f)"), 
			*TargetLinear.ToString(), NormalizedTime);
	}

	// Set constraint target position (same as Mebot linear motor control)
	CachedConstraint->SetLinearPositionTarget(TargetLinear);
	CachedConstraint->SetLinearVelocityTarget(FVector::ZeroVector);
}

void UVanDoorComponent::GetKeyframePositions(float NormalizedTime, FVector& OutLinear, FRotator& OutAngular) const
{
	// Handle edge cases
	if (DoorKeyframes.Num() == 0)
	{
		OutLinear = FVector::ZeroVector;
		OutAngular = FRotator::ZeroRotator;
		return;
	}
	
	if (DoorKeyframes.Num() == 1)
	{
		OutLinear = DoorKeyframes[0].LinearPosition;
		OutAngular = DoorKeyframes[0].AngularPosition;
		return;
	}

	// Clamp normalized time
	NormalizedTime = FMath::Clamp(NormalizedTime, 0.0f, 1.0f);

	// Find the two keyframes to interpolate between
	int32 NextKeyframeIndex = 0;
	for (int32 i = 0; i < DoorKeyframes.Num(); ++i)
	{
		if (DoorKeyframes[i].Time > NormalizedTime)
		{
			NextKeyframeIndex = i;
			break;
		}
		NextKeyframeIndex = i + 1;
	}

	// Handle boundary cases
	if (NextKeyframeIndex == 0)
	{
		OutLinear = DoorKeyframes[0].LinearPosition;
		OutAngular = DoorKeyframes[0].AngularPosition;
		return;
	}
	
	if (NextKeyframeIndex >= DoorKeyframes.Num())
	{
		OutLinear = DoorKeyframes.Last().LinearPosition;
		OutAngular = DoorKeyframes.Last().AngularPosition;
		return;
	}

	// Get the two keyframes
	const FDoorKeyframe& PrevKeyframe = DoorKeyframes[NextKeyframeIndex - 1];
	const FDoorKeyframe& NextKeyframe = DoorKeyframes[NextKeyframeIndex];

	// Calculate interpolation alpha between these two keyframes
	float KeyframeDelta = NextKeyframe.Time - PrevKeyframe.Time;
	float Alpha = KeyframeDelta > 0.0f ? (NormalizedTime - PrevKeyframe.Time) / KeyframeDelta : 0.0f;

	// Interpolate positions
	OutLinear = FMath::Lerp(PrevKeyframe.LinearPosition, NextKeyframe.LinearPosition, Alpha);
	OutAngular = FMath::Lerp(PrevKeyframe.AngularPosition, NextKeyframe.AngularPosition, Alpha);
}

bool UVanDoorComponent::IsAtTargetPosition(const FVector& TargetLinear) const
{
	// Check if we're close enough to the target by comparing animation time
	float NormalizedTime = DoorAnimationDuration > 0.0f ? CurrentAnimationTime / DoorAnimationDuration : 0.0f;
	
	// Consider at target if within 1% of the end of animation
	if (DoorState == EDoorState::Opening)
	{
		return NormalizedTime >= 0.99f;
	}
	else if (DoorState == EDoorState::Closing)
	{
		return NormalizedTime <= 0.01f;
	}
	
	return false;
}

USkeletalMeshComponent* UVanDoorComponent::GetOwnerSkeletalMesh()
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return nullptr;
	}

	// First try to get skeletal mesh from pawn
	APawn* PawnOwner = Cast<APawn>(Owner);
	if (PawnOwner)
	{
		TArray<USkeletalMeshComponent*> SkeletalMeshComponents;
		PawnOwner->GetComponents<USkeletalMeshComponent>(SkeletalMeshComponents);
		
		if (SkeletalMeshComponents.Num() > 0)
		{
			return SkeletalMeshComponents[0];
		}
	}

	// Fallback: try to get any skeletal mesh component
	return Owner->FindComponentByClass<USkeletalMeshComponent>();
}

void UVanDoorComponent::DrawDebugVisualization()
{
	if (!bShowKeyframeGizmos || DoorKeyframes.Num() == 0)
	{
		return;
	}

	UWorld* World = GetWorld();
	if (!World || !CachedSkeletalMesh)
	{
		return;
	}

	// Use skeletal mesh transform as base reference (handles actor rotation)
	const FTransform BaseTransform = CachedSkeletalMesh->GetComponentTransform();
	const FVector BaseLocation = BaseTransform.GetLocation();
	const FQuat BaseRotation = BaseTransform.GetRotation();

	// Draw each keyframe with short lifetime to prevent accumulation
	FVector PreviousWorldLocation = BaseLocation;
	for (int32 i = 0; i < DoorKeyframes.Num(); ++i)
	{
		const FDoorKeyframe& Keyframe = DoorKeyframes[i];
		
		// Calculate world position for this keyframe
		FVector WorldLocation = BaseTransform.TransformPosition(Keyframe.LinearPosition);
		const FQuat WorldRotation = (BaseRotation * Keyframe.AngularPosition.Quaternion()).GetNormalized();

		// Color gradient from green to red
		FLinearColor LerpedColor = FLinearColor::LerpUsingHSV(FLinearColor::Green, FLinearColor::Red, Keyframe.Time);
		FColor GizmoColor = LerpedColor.ToFColor(true);

		// Draw coordinate system at keyframe location (0.0f lifetime = just this frame)
		DrawDebugCoordinateSystem(World, WorldLocation, WorldRotation.Rotator(), GizmoSize, false, 0.0f, 0, 2.0f);

		// Draw sphere at keyframe location
		DrawDebugSphere(World, WorldLocation, 15.0f, 8, GizmoColor, false, 0.0f, 0, 3.0f);

		// Draw time label
		DrawDebugString(World, WorldLocation + FVector(0, 0, GizmoSize + 20.0f), 
			FString::Printf(TEXT("T: %.2f"), Keyframe.Time), nullptr, FColor::White, 0.0f, true, 1.5f);

		// Draw line connecting to previous keyframe
		if (bShowKeyframePath && i > 0)
		{
			DrawDebugLine(World, PreviousWorldLocation, WorldLocation, FColor::Yellow, false, 0.0f, 0, 3.0f);
		}

		PreviousWorldLocation = WorldLocation;
	}

	// Draw current door state
	float NormalizedTime = DoorAnimationDuration > 0.0f ? CurrentAnimationTime / DoorAnimationDuration : 0.0f;
	FVector CurrentTarget;
	FRotator CurrentAngular;
	GetKeyframePositions(NormalizedTime, CurrentTarget, CurrentAngular);
	FVector CurrentWorldPos = BaseTransform.TransformPosition(CurrentTarget);

	// Draw current target position as a larger sphere
	FColor StateColor = (DoorState == EDoorState::Open || DoorState == EDoorState::Opening) ? FColor::Green : FColor::Blue;
	DrawDebugSphere(World, CurrentWorldPos, 25.0f, 12, StateColor, false, 0.0f, 0, 4.0f);
	
	// Draw state text
	FString StateText = FString::Printf(TEXT("State: %s\nTime: %.2f/%.2f\nConstraint: %s"), 
		DoorState == EDoorState::Closed ? TEXT("CLOSED") :
		DoorState == EDoorState::Opening ? TEXT("OPENING") :
		DoorState == EDoorState::Open ? TEXT("OPEN") : TEXT("CLOSING"),
		CurrentAnimationTime, DoorAnimationDuration,
		CachedConstraint ? TEXT("OK") : TEXT("NOT FOUND"));
	DrawDebugString(World, CurrentWorldPos + FVector(0, 0, 40.0f), StateText, nullptr, FColor::Cyan, 0.0f, true, 1.2f);
}
