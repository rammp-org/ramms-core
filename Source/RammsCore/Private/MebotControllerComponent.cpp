// Copyright Epic Games, Inc. All Rights Reserved.

#include "MebotControllerComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "DrawDebugHelpers.h"

UMebotControllerComponent::UMebotControllerComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	bAutoFindConstraints = true;
	bAutoFindSkeletalMesh = true;
	bEnableDebugLog = false;
	bShowMotorGizmos = true;
	GizmoSize = 50.0f;
	CachedSkeletalMesh = nullptr;

	// Initialize default motor configurations
	// Front caster arm - angular motor
	FAngularMotorConfig FrontCaster;
	FrontCaster.ConstraintName = FName("FrontCasterArm");
	FrontCaster.ControlAxis = EMotorAxis::Y;
	FrontCaster.TargetAngle = 0.0f;
	AngularMotors.Add(FrontCaster);

	// Rear caster arm - angular motor
	FAngularMotorConfig RearCaster;
	RearCaster.ConstraintName = FName("RearCasterArm");
	RearCaster.ControlAxis = EMotorAxis::Y;
	RearCaster.TargetAngle = 0.0f;
	AngularMotors.Add(RearCaster);

	// Left motor elevator - angular motor
	FAngularMotorConfig LeftElevator;
	LeftElevator.ConstraintName = FName("LeftMotorElevator");
	LeftElevator.ControlAxis = EMotorAxis::Y;
	LeftElevator.TargetAngle = 0.0f;
	AngularMotors.Add(LeftElevator);

	// Right motor elevator - angular motor
	FAngularMotorConfig RightElevator;
	RightElevator.ConstraintName = FName("RightMotorElevator");
	RightElevator.ControlAxis = EMotorAxis::Y;
	RightElevator.TargetAngle = 0.0f;
	AngularMotors.Add(RightElevator);

	// Left motor translator - linear motor
	FLinearMotorConfig LeftTranslator;
	LeftTranslator.ConstraintName = FName("LeftMotorTranslator");
	LeftTranslator.ControlAxis = EMotorAxis::Z;
	LeftTranslator.TargetPosition = 0.0f;
	LinearMotors.Add(LeftTranslator);

	// Right motor translator - linear motor
	FLinearMotorConfig RightTranslator;
	RightTranslator.ConstraintName = FName("RightMotorTranslator");
	RightTranslator.ControlAxis = EMotorAxis::Z;
	RightTranslator.TargetPosition = 0.0f;
	LinearMotors.Add(RightTranslator);
}

void UMebotControllerComponent::BeginPlay()
{
	Super::BeginPlay();

	if (bAutoFindConstraints)
	{
		FindConstraints();
	}

	if (bEnableDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("MebotController: Initialized with %d angular motors and %d linear motors"), 
			AngularMotors.Num(), LinearMotors.Num());
	}
}

void UMebotControllerComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

	// Clear debug draws
	if (UWorld* World = GetWorld())
	{
		FlushPersistentDebugLines(World);
	}
}

void UMebotControllerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UpdateAngularMotors();
	UpdateLinearMotors();
	DrawDebugVisualization();
}

void UMebotControllerComponent::SetAngularMotorTarget(FName MotorName, float TargetAngle)
{
	for (FAngularMotorConfig& Motor : AngularMotors)
	{
		if (Motor.ConstraintName == MotorName)
		{
			Motor.TargetAngle = TargetAngle;
			ApplyAngularMotorSettings(Motor);
			
			if (bEnableDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("MebotController: Set angular motor '%s' target to %.2f degrees"), 
					*MotorName.ToString(), TargetAngle);
			}
			return;
		}
	}
	UE_LOG(LogTemp, Warning, TEXT("MebotController: Angular motor '%s' not found"), *MotorName.ToString());
}

void UMebotControllerComponent::SetLinearMotorTarget(FName MotorName, float TargetPosition)
{
	for (FLinearMotorConfig& Motor : LinearMotors)
	{
		if (Motor.ConstraintName == MotorName)
		{
			Motor.TargetPosition = TargetPosition;
			ApplyLinearMotorSettings(Motor);
			
			if (bEnableDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("MebotController: Set linear motor '%s' target to %.2f cm"), 
					*MotorName.ToString(), TargetPosition);
			}
			return;
		}
	}
	UE_LOG(LogTemp, Warning, TEXT("MebotController: Linear motor '%s' not found"), *MotorName.ToString());
}

void UMebotControllerComponent::SetAngularMotorEnabled(FName MotorName, bool bEnabled)
{
	for (FAngularMotorConfig& Motor : AngularMotors)
	{
		if (Motor.ConstraintName == MotorName)
		{
			Motor.bEnabled = bEnabled;
			ApplyAngularMotorSettings(Motor);
			return;
		}
	}
}

void UMebotControllerComponent::SetLinearMotorEnabled(FName MotorName, bool bEnabled)
{
	for (FLinearMotorConfig& Motor : LinearMotors)
	{
		if (Motor.ConstraintName == MotorName)
		{
			Motor.bEnabled = bEnabled;
			ApplyLinearMotorSettings(Motor);
			return;
		}
	}
}

float UMebotControllerComponent::GetAngularMotorCurrentAngle(FName MotorName) const
{
	for (const FAngularMotorConfig& Motor : AngularMotors)
	{
		if (Motor.ConstraintName == MotorName && Motor.CachedConstraint)
		{
			// Get current angular state from constraint
			float CurrentAngle = 0.0f;
			switch (Motor.ControlAxis)
			{
			case EMotorAxis::X:
				CurrentAngle = Motor.CachedConstraint->GetCurrentSwing1();
				break;
			case EMotorAxis::Y:
				CurrentAngle = Motor.CachedConstraint->GetCurrentSwing2();
				break;
			case EMotorAxis::Z:
				CurrentAngle = Motor.CachedConstraint->GetCurrentTwist();
				break;
			}
			return CurrentAngle;
		}
	}
	return 0.0f;
}

float UMebotControllerComponent::GetLinearMotorCurrentPosition(FName MotorName) const
{
	for (const FLinearMotorConfig& Motor : LinearMotors)
	{
		if (Motor.ConstraintName == MotorName && Motor.CachedConstraint)
		{
			// Get current linear state from constraint - not directly available, return target
			return Motor.TargetPosition;
		}
	}
	return 0.0f;
}

void UMebotControllerComponent::ReinitializeMotors()
{
	FindConstraints();
	UpdateAngularMotors();
	UpdateLinearMotors();
	
	if (bEnableDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("MebotController: Motors reinitialized"));
	}
}

FString UMebotControllerComponent::GetMotorDebugInfo() const
{
	FString Info = TEXT("=== Mebot Motor Status ===\n\nAngular Motors:\n");
	
	for (const FAngularMotorConfig& Motor : AngularMotors)
	{
		FString AxisStr = Motor.ControlAxis == EMotorAxis::X ? TEXT("X") : 
						  Motor.ControlAxis == EMotorAxis::Y ? TEXT("Y") : TEXT("Z");
		FString StatusStr = Motor.CachedConstraint ? TEXT("OK") : TEXT("NOT FOUND");
		Info += FString::Printf(TEXT("  %s: Target=%.1f° | Axis=%s | Enabled=%s | Status=%s\n"),
			*Motor.ConstraintName.ToString(),
			Motor.TargetAngle,
			*AxisStr,
			Motor.bEnabled ? TEXT("Yes") : TEXT("No"),
			*StatusStr);
	}
	
	Info += TEXT("\nLinear Motors:\n");
	for (const FLinearMotorConfig& Motor : LinearMotors)
	{
		FString AxisStr = Motor.ControlAxis == EMotorAxis::X ? TEXT("X") : 
						  Motor.ControlAxis == EMotorAxis::Y ? TEXT("Y") : TEXT("Z");
		FString StatusStr = Motor.CachedConstraint ? TEXT("OK") : TEXT("NOT FOUND");
		Info += FString::Printf(TEXT("  %s: Target=%.1fcm | Axis=%s | Enabled=%s | Status=%s\n"),
			*Motor.ConstraintName.ToString(),
			Motor.TargetPosition,
			*AxisStr,
			Motor.bEnabled ? TEXT("Yes") : TEXT("No"),
			*StatusStr);
	}
	
	return Info;
}

void UMebotControllerComponent::FindConstraints()
{
	// Find the skeletal mesh first
	if (bAutoFindSkeletalMesh)
	{
		CachedSkeletalMesh = GetOwnerSkeletalMesh();
	}

	if (!CachedSkeletalMesh)
	{
		UE_LOG(LogTemp, Warning, TEXT("MebotController: No SkeletalMeshComponent found on '%s'"), 
			*GetOwner()->GetName());
		return;
	}

	if (bEnableDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("MebotController: Found skeletal mesh '%s'"), 
			*CachedSkeletalMesh->GetName());
	}

	// Match angular motors to constraints in physics asset
	for (FAngularMotorConfig& Motor : AngularMotors)
	{
		Motor.CachedConstraint = nullptr;
		Motor.CachedConstraint = CachedSkeletalMesh->FindConstraintInstance(Motor.ConstraintName);
		
		if (Motor.CachedConstraint)
		{
			if (bEnableDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("MebotController: Found angular motor constraint '%s' in physics asset"), 
					*Motor.ConstraintName.ToString());
			}
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("MebotController: Angular motor constraint '%s' not found in physics asset"), 
				*Motor.ConstraintName.ToString());
		}
	}

	// Match linear motors to constraints in physics asset
	for (FLinearMotorConfig& Motor : LinearMotors)
	{
		Motor.CachedConstraint = nullptr;
		Motor.CachedConstraint = CachedSkeletalMesh->FindConstraintInstance(Motor.ConstraintName);
		
		if (Motor.CachedConstraint)
		{
			if (bEnableDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("MebotController: Found linear motor constraint '%s' in physics asset"), 
					*Motor.ConstraintName.ToString());
			}
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("MebotController: Linear motor constraint '%s' not found in physics asset"), 
				*Motor.ConstraintName.ToString());
		}
	}
}

USkeletalMeshComponent* UMebotControllerComponent::GetOwnerSkeletalMesh()
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

void UMebotControllerComponent::UpdateAngularMotors()
{
	for (FAngularMotorConfig& Motor : AngularMotors)
	{
		if (Motor.bEnabled && Motor.CachedConstraint)
		{
			ApplyAngularMotorSettings(Motor);
		}
	}
}

void UMebotControllerComponent::UpdateLinearMotors()
{
	for (FLinearMotorConfig& Motor : LinearMotors)
	{
		if (Motor.bEnabled && Motor.CachedConstraint)
		{
			ApplyLinearMotorSettings(Motor);
		}
	}
}

void UMebotControllerComponent::ApplyAngularMotorSettings(FAngularMotorConfig& Motor)
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
		
		// Set target orientation based on axis
		FQuat TargetQuat = FQuat::Identity;
		switch (Motor.ControlAxis)
		{
		case EMotorAxis::X:
			TargetQuat = FQuat(FVector(1, 0, 0), FMath::DegreesToRadians(Motor.TargetAngle));
			break;
		case EMotorAxis::Y:
			TargetQuat = FQuat(FVector(0, 1, 0), FMath::DegreesToRadians(Motor.TargetAngle));
			break;
		case EMotorAxis::Z:
			TargetQuat = FQuat(FVector(0, 0, 1), FMath::DegreesToRadians(Motor.TargetAngle));
			break;
		}
		
		Motor.CachedConstraint->SetAngularOrientationTarget(TargetQuat);
	}
}

void UMebotControllerComponent::ApplyLinearMotorSettings(FLinearMotorConfig& Motor)
{
	if (!Motor.CachedConstraint)
	{
		return;
	}

	// Enable linear motor on the specified axis
	bool bDriveX = (Motor.ControlAxis == EMotorAxis::X) && Motor.bEnabled;
	bool bDriveY = (Motor.ControlAxis == EMotorAxis::Y) && Motor.bEnabled;
	bool bDriveZ = (Motor.ControlAxis == EMotorAxis::Z) && Motor.bEnabled;

	Motor.CachedConstraint->SetLinearPositionDrive(bDriveX, bDriveY, bDriveZ);
	Motor.CachedConstraint->SetLinearVelocityDrive(bDriveX, bDriveY, bDriveZ);

	if (Motor.bEnabled)
	{
		Motor.CachedConstraint->SetLinearDriveParams(Motor.MotorStrength, Motor.MotorDamping, 0.0f);
		
		// Set target position on the controlled axis
		FVector TargetPosition = FVector::ZeroVector;
		switch (Motor.ControlAxis)
		{
		case EMotorAxis::X:
			TargetPosition.X = Motor.TargetPosition;
			break;
		case EMotorAxis::Y:
			TargetPosition.Y = Motor.TargetPosition;
			break;
		case EMotorAxis::Z:
			TargetPosition.Z = Motor.TargetPosition;
			break;
		}
		
		Motor.CachedConstraint->SetLinearPositionTarget(TargetPosition);
		Motor.CachedConstraint->SetLinearVelocityTarget(FVector::ZeroVector);
	}
}

void UMebotControllerComponent::DrawDebugVisualization()
{
	if (!bShowMotorGizmos || !CachedSkeletalMesh)
	{
		return;
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	// Draw angular motors
	float YOffset = 0.0f;
	for (const FAngularMotorConfig& Motor : AngularMotors)
	{
		if (!Motor.CachedConstraint)
		{
			continue;
		}

		// Get constraint location from skeletal mesh
		FVector MotorLocation = CachedSkeletalMesh->GetComponentLocation() + FVector(0, 0, 50.0f * YOffset);
		FColor MotorColor = Motor.bEnabled ? FColor::Green : FColor::Red;

		// Draw sphere at motor location
		DrawDebugSphere(World, MotorLocation, 10.0f, 8, MotorColor, false, 0.0f, 0, 2.0f);

		// Draw axis indicator
		FVector AxisDir = FVector::ZeroVector;
		switch (Motor.ControlAxis)
		{
		case EMotorAxis::X: AxisDir = FVector(1, 0, 0); break;
		case EMotorAxis::Y: AxisDir = FVector(0, 1, 0); break;
		case EMotorAxis::Z: AxisDir = FVector(0, 0, 1); break;
		}
		DrawDebugDirectionalArrow(World, MotorLocation, MotorLocation + AxisDir * GizmoSize, 
			10.0f, FColor::Cyan, false, 0.0f, 0, 2.0f);

		// Draw motor info
		FString MotorInfo = FString::Printf(TEXT("%s\n%.1f°"), 
			*Motor.ConstraintName.ToString(), Motor.TargetAngle);
		DrawDebugString(World, MotorLocation + FVector(0, 0, 20.0f), 
			MotorInfo, nullptr, FColor::White, 0.0f, true, 1.0f);
		
		YOffset += 1.0f;
	}

	// Draw linear motors
	YOffset = 0.0f;
	for (const FLinearMotorConfig& Motor : LinearMotors)
	{
		if (!Motor.CachedConstraint)
		{
			continue;
		}

		FVector MotorLocation = CachedSkeletalMesh->GetComponentLocation() + FVector(0, 100.0f, 50.0f * YOffset);
		FColor MotorColor = Motor.bEnabled ? FColor::Blue : FColor::Orange;

		// Draw box at motor location
		DrawDebugBox(World, MotorLocation, FVector(15.0f), MotorColor, false, 0.0f, 0, 2.0f);

		// Draw axis indicator
		FVector AxisDir = FVector::ZeroVector;
		switch (Motor.ControlAxis)
		{
		case EMotorAxis::X: AxisDir = FVector(1, 0, 0); break;
		case EMotorAxis::Y: AxisDir = FVector(0, 1, 0); break;
		case EMotorAxis::Z: AxisDir = FVector(0, 0, 1); break;
		}
		DrawDebugDirectionalArrow(World, MotorLocation, MotorLocation + AxisDir * GizmoSize, 
			10.0f, FColor::Magenta, false, 0.0f, 0, 2.0f);

		// Draw motor info
		FString MotorInfo = FString::Printf(TEXT("%s\n%.1fcm"), 
			*Motor.ConstraintName.ToString(), Motor.TargetPosition);
		DrawDebugString(World, MotorLocation + FVector(0, 0, 20.0f), 
			MotorInfo, nullptr, FColor::Yellow, 0.0f, true, 1.0f);
		
		YOffset += 1.0f;
	}
}
