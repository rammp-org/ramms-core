// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsDifferentialDriveController.h"
#include "RammsDifferentialDriveLibrary.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/BodyInstance.h"
#include "DrawDebugHelpers.h"

URammsDifferentialDriveController::URammsDifferentialDriveController()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void URammsDifferentialDriveController::BeginPlay()
{
	Super::BeginPlay();

	// Find the skeletal mesh component
	AActor* Owner = GetOwner();
	if (Owner)
	{
		// If a specific component name is provided, find it by name
		if (SkeletalMeshComponentName != NAME_None)
		{
			TArray<USkeletalMeshComponent*> SkeletalMeshes;
			Owner->GetComponents<USkeletalMeshComponent>(SkeletalMeshes);
			
			for (USkeletalMeshComponent* SkelMesh : SkeletalMeshes)
			{
				if (SkelMesh && SkelMesh->GetFName() == SkeletalMeshComponentName)
				{
					SkeletalMeshComponent = SkelMesh;
					break;
				}
			}
		}
		else
		{
			// Auto-find the first skeletal mesh component
			SkeletalMeshComponent = Owner->FindComponentByClass<USkeletalMeshComponent>();
		}

		if (!SkeletalMeshComponent)
		{
			UE_LOG(LogTemp, Warning, TEXT("RammsDifferentialDriveController: Failed to find skeletal mesh component on %s"), *Owner->GetName());
		}
		else
		{
			// Set max angular velocity on wheel body instances
			FBodyInstance* LeftBodyInst = GetBoneBodyInstance(LeftWheelBoneName);
			if (LeftBodyInst)
			{
				float MaxAngularVelRad = URammsDifferentialDriveLibrary::RPMToRadPerSec(LeftMotorParams.MaxRPM);
				LeftBodyInst->SetMaxAngularVelocityInRadians(MaxAngularVelRad, false, true);
				
				if (bEnableDebugLogging)
				{
					UE_LOG(LogTemp, Log, TEXT("[DiffDrive] Set left wheel max angular velocity: %.1f RPM (%.3f rad/s)"),
						LeftMotorParams.MaxRPM, MaxAngularVelRad);
				}
			}
			
			FBodyInstance* RightBodyInst = GetBoneBodyInstance(RightWheelBoneName);
			if (RightBodyInst)
			{
				float MaxAngularVelRad = URammsDifferentialDriveLibrary::RPMToRadPerSec(RightMotorParams.MaxRPM);
				RightBodyInst->SetMaxAngularVelocityInRadians(MaxAngularVelRad, false, true);
				
				if (bEnableDebugLogging)
				{
					UE_LOG(LogTemp, Log, TEXT("[DiffDrive] Set right wheel max angular velocity: %.1f RPM (%.3f rad/s)"),
						RightMotorParams.MaxRPM, MaxAngularVelRad);
				}
			}
		}

		// Initialize odometry with actor's current transform
		Odometry = URammsDifferentialDriveLibrary::ResetOdometry(
			Owner->GetActorLocation(),
			Owner->GetActorRotation());
	}

	// Initialize previous wheel rotations
	PreviousLeftRotation = 0.0f;
	PreviousRightRotation = 0.0f;
}

void URammsDifferentialDriveController::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Update wheel states
	UpdateWheelState(LeftWheelBoneName, LeftWheelState);
	UpdateWheelState(RightWheelBoneName, RightWheelState);

	// Check for braking
	bIsBraking = ShouldApplyBrakes();

	if (bIsBraking)
	{
		ApplyBrakes();
	}
	else
	{
		// Update based on control mode
		switch (ControlMode)
		{
		case EDriveControlMode::TorqueControl:
			UpdateTorqueControl(DeltaTime);
			break;

		case EDriveControlMode::VelocityControl:
			UpdateVelocityControl(DeltaTime);
			break;
		}
	}

	// Update odometry
	UpdateOdometry(DeltaTime);

	// Debug logging
	if (bEnableDebugLogging || bEnableDebugDisplay)
	{
		DebugLogState();
	}
}

void URammsDifferentialDriveController::SetDriveInput(FVector2D Input)
{
	DriveInput = Input;

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[DiffDrive] SetDriveInput: X=%.3f, Y=%.3f"), Input.X, Input.Y);
	}

	// Reset integral terms when input changes significantly (prevents windup)
	if (Input.SizeSquared() < 0.01f)
	{
		LeftIntegralError = 0.0f;
		RightIntegralError = 0.0f;
	}
}

void URammsDifferentialDriveController::ResetOdometry(FVector Position, FRotator Orientation)
{
	Odometry = URammsDifferentialDriveLibrary::ResetOdometry(Position, Orientation);
	PreviousLeftRotation = LeftWheelState.TotalRotation;
	PreviousRightRotation = RightWheelState.TotalRotation;
}

FBodyInstance* URammsDifferentialDriveController::GetBoneBodyInstance(FName BoneName)
{
	if (!SkeletalMeshComponent || BoneName == NAME_None)
	{
		return nullptr;
	}

	return SkeletalMeshComponent->GetBodyInstance(BoneName);
}

void URammsDifferentialDriveController::UpdateWheelState(FName BoneName, FWheelState& OutState)
{
	FBodyInstance* BodyInst = GetBoneBodyInstance(BoneName);
	if (!BodyInst || !BodyInst->IsInstanceSimulatingPhysics())
	{
		return;
	}

	// Get angular velocity in radians per second
	// Assuming wheel spins around its local Y axis (standard for UE wheel components)
	FVector AngularVelocity = BodyInst->GetUnrealWorldAngularVelocityInRadians();
	
	// Get the bone transform to convert to local space
	int32 BoneIndex = SkeletalMeshComponent->GetBoneIndex(BoneName);
	if (BoneIndex == INDEX_NONE)
	{
		return;
	}

	FTransform BoneTransform = SkeletalMeshComponent->GetBoneTransform(BoneIndex);
	FVector LocalAngularVelocity = BoneTransform.InverseTransformVectorNoScale(AngularVelocity);
	
	// Extract rotation speed around wheel's spin axis (typically Y axis)
	OutState.AngularVelocity = LocalAngularVelocity.Y;

	// Calculate linear velocity at contact point
	OutState.LinearVelocity = OutState.AngularVelocity * WheelRadius;

	// Get actual linear velocity of the wheel
	FVector WheelVelocity = BodyInst->GetUnrealWorldVelocity();
	FVector ForwardDir = BoneTransform.GetUnitAxis(EAxis::X);
	float ActualLinearVel = FVector::DotProduct(WheelVelocity, ForwardDir);

	// Calculate slip if enabled
	if (bEnableSlipModeling && FMath::Abs(OutState.LinearVelocity) > SMALL_NUMBER)
	{
		OutState.SlipRatio = URammsDifferentialDriveLibrary::CalculateSlipRatio(
			OutState.LinearVelocity,
			ActualLinearVel);
	}
	else
	{
		OutState.SlipRatio = 0.0f;
	}
}

void URammsDifferentialDriveController::UpdateTorqueControl(float DeltaTime)
{
	// Convert joystick input to differential drive commands (torque values)
	FDifferentialDriveCommand Command = URammsDifferentialDriveLibrary::JoystickToDifferentialDrive(
		DriveInput,
		1.0f,  // Max value is 1.0, we'll scale by max torque later
		InputDeadZone);

	// Scale to actual torque values
	float LeftTorque = Command.LeftCommand * LeftMotorParams.MaxTorque * TorqueMultiplier;
	float RightTorque = Command.RightCommand * RightMotorParams.MaxTorque * TorqueMultiplier;

	// Get actual angular velocity of the wheelchair base for debug display
	float CurrentTurningSpeed = 0.0f;
	if (AActor* Owner = GetOwner())
	{
		if (UPrimitiveComponent* RootPrimitive = Cast<UPrimitiveComponent>(Owner->GetRootComponent()))
		{
			if (RootPrimitive->IsSimulatingPhysics())
			{
				FVector AngularVelocity = RootPrimitive->GetPhysicsAngularVelocityInDegrees();
				CurrentTurningSpeed = AngularVelocity.Z;
			}
		}
	}
	
	float AbsTurningSpeed = FMath::Abs(CurrentTurningSpeed);
	
	// Apply turning speed limit using damping if enabled
	if (MaxTurningSpeed > 0.0f && AbsTurningSpeed > MaxTurningSpeed)
	{
		float TurnSpeedError = AbsTurningSpeed - MaxTurningSpeed;
		float DampingStrength = 0.5f; // Gentler damping - Nm per degree/s of overspeed
		float TurnDampingTorque = TurnSpeedError * DampingStrength;
		
		// Only apply damping if we're significantly over the limit (>10% overshoot)
		if (TurnSpeedError > MaxTurningSpeed * 0.1f)
		{
			float TurnDirection = FMath::Sign(CurrentTurningSpeed);
			LeftTorque += TurnDirection * TurnDampingTorque;
			RightTorque -= TurnDirection * TurnDampingTorque;
			
			if (bEnableDebugLogging)
			{
				UE_LOG(LogTemp, Log, TEXT("[DiffDrive] Damping turn: %.1f deg/s (max: %.1f), damping torque: %.2f Nm"),
					AbsTurningSpeed, MaxTurningSpeed, TurnDampingTorque);
			}
		}
	}

	// Apply resistive torque that opposes motion (simulates back-EMF, friction, drag)
	float LeftResistiveTorque = -FMath::Sign(LeftWheelState.AngularVelocity) * 
	                             ResistiveTorqueCoefficient * 
	                             FMath::Abs(LeftWheelState.AngularVelocity);
	float RightResistiveTorque = -FMath::Sign(RightWheelState.AngularVelocity) * 
	                              ResistiveTorqueCoefficient * 
	                              FMath::Abs(RightWheelState.AngularVelocity);

	// Add resistive torque to commanded torque
	LeftTorque += LeftResistiveTorque;
	RightTorque += RightResistiveTorque;

	if (bEnableDebugLogging)
	{
		float LeftRPM = URammsDifferentialDriveLibrary::RadPerSecToRPM(LeftWheelState.AngularVelocity);
		float RightRPM = URammsDifferentialDriveLibrary::RadPerSecToRPM(RightWheelState.AngularVelocity);
		
		UE_LOG(LogTemp, Log, TEXT("[DiffDrive] TorqueControl - LeftCmd=%.3f, RightCmd=%.3f, LeftTorque=%.3f Nm, RightTorque=%.3f Nm, LeftRPM=%.1f, RightRPM=%.1f, TurnSpeed=%.1f deg/s"),
			Command.LeftCommand, Command.RightCommand, LeftTorque, RightTorque, LeftRPM, RightRPM, AbsTurningSpeed);
	}

	// Store target for debugging
	LeftWheelState.TargetAngularVelocity = Command.LeftCommand;
	RightWheelState.TargetAngularVelocity = Command.RightCommand;

	// Apply torques to wheels
	ApplyWheelTorque(LeftWheelBoneName, LeftTorque, LeftMotorParams, LeftWheelState);
	ApplyWheelTorque(RightWheelBoneName, RightTorque, RightMotorParams, RightWheelState);
}

void URammsDifferentialDriveController::UpdateVelocityControl(float DeltaTime)
{
	// Convert joystick input to differential drive commands (velocity values)
	FDifferentialDriveCommand VelocityCommand = URammsDifferentialDriveLibrary::JoystickToDifferentialDrive(
		DriveInput,
		MaxVelocity,  // Max velocity in cm/s
		InputDeadZone);

	// Convert linear velocities to angular velocities
	float LeftTargetAngularVel = VelocityCommand.LeftCommand / WheelRadius;
	float RightTargetAngularVel = VelocityCommand.RightCommand / WheelRadius;

	// Store targets
	LeftWheelState.TargetAngularVelocity = LeftTargetAngularVel;
	RightWheelState.TargetAngularVelocity = RightTargetAngularVel;

	// Calculate velocity errors
	float LeftError = LeftTargetAngularVel - LeftWheelState.AngularVelocity;
	float RightError = RightTargetAngularVel - RightWheelState.AngularVelocity;

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[DiffDrive] VelocityControl - LeftTarget=%.3f rad/s, LeftActual=%.3f rad/s, LeftError=%.3f"),
			LeftTargetAngularVel, LeftWheelState.AngularVelocity, LeftError);
		UE_LOG(LogTemp, Log, TEXT("[DiffDrive] VelocityControl - RightTarget=%.3f rad/s, RightActual=%.3f rad/s, RightError=%.3f"),
			RightTargetAngularVel, RightWheelState.AngularVelocity, RightError);
	}

	// Calculate PID control outputs (torque)
	float LeftTorque = CalculatePID(LeftError, LeftIntegralError, LeftPreviousError, DeltaTime);
	float RightTorque = CalculatePID(RightError, RightIntegralError, RightPreviousError, DeltaTime);

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[DiffDrive] VelocityControl - PID Output: LeftTorque=%.3f Nm, RightTorque=%.3f Nm"),
			LeftTorque, RightTorque);
	}

	// Apply torques to wheels
	ApplyWheelTorque(LeftWheelBoneName, LeftTorque, LeftMotorParams, LeftWheelState);
	ApplyWheelTorque(RightWheelBoneName, RightTorque, RightMotorParams, RightWheelState);
}

float URammsDifferentialDriveController::CalculatePID(
	float Error,
	float& IntegralError,
	float& PreviousError,
	float DeltaTime)
{
	if (DeltaTime < SMALL_NUMBER)
	{
		return 0.0f;
	}

	// Proportional term
	float P = PID_Kp * Error;

	// Integral term with anti-windup
	IntegralError += Error * DeltaTime;
	IntegralError = FMath::Clamp(IntegralError, -PID_IntegralMax, PID_IntegralMax);
	float I = PID_Ki * IntegralError;

	// Derivative term
	float Derivative = (Error - PreviousError) / DeltaTime;
	float D = PID_Kd * Derivative;

	// Update previous error
	PreviousError = Error;

	return P + I + D;
}

void URammsDifferentialDriveController::ApplyWheelTorque(
	FName BoneName,
	float RequestedTorque,
	const FMotorParameters& MotorParams,
	FWheelState& WheelState)
{
	FBodyInstance* BodyInst = GetBoneBodyInstance(BoneName);
	if (!BodyInst || !BodyInst->IsInstanceSimulatingPhysics())
	{
		WheelState.AppliedTorque = 0.0f;
		return;
	}

	// Convert angular velocity to RPM for motor curve evaluation
	float CurrentRPM = URammsDifferentialDriveLibrary::RadPerSecToRPM(WheelState.AngularVelocity);

	// Apply motor torque curve
	float AvailableTorque = URammsDifferentialDriveLibrary::EvaluateMotorTorque(
		CurrentRPM,
		MotorParams,
		RequestedTorque);

	// Apply slip/traction modeling if enabled
	if (bEnableSlipModeling)
	{
		// Reduce effective torque based on slip
		if (WheelState.SlipRatio > SlipThreshold)
		{
			float TractionFactor = FMath::Lerp(
				TractionCoefficient,
				0.0f,
				(WheelState.SlipRatio - SlipThreshold) / (1.0f - SlipThreshold));
			AvailableTorque *= TractionFactor;
		}
	}

	WheelState.AppliedTorque = AvailableTorque;

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[DiffDrive] ApplyTorque to %s: Requested=%.3f Nm, Available=%.3f Nm, CurrentRPM=%.1f, Slip=%.3f"),
			*BoneName.ToString(), RequestedTorque, AvailableTorque, CurrentRPM, WheelState.SlipRatio);
	}

	// Get bone transform to apply torque in correct direction
	int32 BoneIndex = SkeletalMeshComponent->GetBoneIndex(BoneName);
	if (BoneIndex == INDEX_NONE)
	{
		return;
	}

	FTransform BoneTransform = SkeletalMeshComponent->GetBoneTransform(BoneIndex);
	
	// Apply torque around wheel's local Y axis (spin axis)
	FVector LocalTorque = FVector(0.0f, AvailableTorque, 0.0f);
	FVector WorldTorque = BoneTransform.TransformVectorNoScale(LocalTorque);

	// Convert Newton-meters to Unreal units (N*m to kg*cm²/s²)
	// 1 N*m = 100 kg*cm²/s² in UE units
	WorldTorque *= 100.0f;

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[DiffDrive] Final WorldTorque for %s: X=%.1f, Y=%.1f, Z=%.1f (UE units)"),
			*BoneName.ToString(), WorldTorque.X, WorldTorque.Y, WorldTorque.Z);
	}

	BodyInst->AddTorqueInRadians(WorldTorque, false, true);
}

void URammsDifferentialDriveController::UpdateOdometry(float DeltaTime)
{
	// Check if both wheels are moving slowly enough to ignore
	float AvgVelocity = (FMath::Abs(LeftWheelState.LinearVelocity) + FMath::Abs(RightWheelState.LinearVelocity)) * 0.5f;
	if (AvgVelocity < OdometryVelocityThreshold)
	{
		// Movement too small - ignore to prevent odometry drift
		return;
	}
	
	// Calculate wheel rotation deltas
	float LeftDelta = LeftWheelState.TotalRotation - PreviousLeftRotation;
	float RightDelta = RightWheelState.TotalRotation - PreviousRightRotation;

	// Update total rotations based on angular velocity
	LeftWheelState.TotalRotation += LeftWheelState.AngularVelocity * DeltaTime;
	RightWheelState.TotalRotation += RightWheelState.AngularVelocity * DeltaTime;

	// Recalculate deltas with updated values
	LeftDelta = LeftWheelState.TotalRotation - PreviousLeftRotation;
	RightDelta = RightWheelState.TotalRotation - PreviousRightRotation;

	// Update odometry
	Odometry = URammsDifferentialDriveLibrary::UpdateOdometry(
		Odometry,
		LeftDelta,
		RightDelta,
		TrackWidth,
		WheelRadius,
		DeltaTime);

	// Store current rotations for next frame
	PreviousLeftRotation = LeftWheelState.TotalRotation;
	PreviousRightRotation = RightWheelState.TotalRotation;
}

bool URammsDifferentialDriveController::ShouldApplyBrakes() const
{
	if (!bEnableAutoBraking)
	{
		return false;
	}

	// Check if input is below threshold
	bool bInputAtRest = DriveInput.SizeSquared() < (BrakingThreshold * BrakingThreshold);

	// Check if velocity is above threshold (don't brake if already stopped)
	float AvgVelocity = (FMath::Abs(LeftWheelState.LinearVelocity) + FMath::Abs(RightWheelState.LinearVelocity)) * 0.5f;
	bool bIsMoving = AvgVelocity > BrakingVelocityThreshold;

	bool bShouldBrake = bInputAtRest && bIsMoving;

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[DiffDrive] Brake Check - InputAtRest: %s, AvgVel: %.3f cm/s, Threshold: %.3f, IsMoving: %s, ShouldBrake: %s"),
			bInputAtRest ? TEXT("YES") : TEXT("NO"),
			AvgVelocity,
			BrakingVelocityThreshold,
			bIsMoving ? TEXT("YES") : TEXT("NO"),
			bShouldBrake ? TEXT("YES") : TEXT("NO"));
	}

	return bShouldBrake;
}

void URammsDifferentialDriveController::ApplyBrakes()
{
	// Apply braking by applying damping torque proportional to velocity
	// Add a deadband to prevent oscillation at very low speeds
	const float BrakeDeadband = 0.1f; // rad/s - don't apply brakes below this angular velocity
	
	FBodyInstance* LeftBodyInst = GetBoneBodyInstance(LeftWheelBoneName);
	if (LeftBodyInst && LeftBodyInst->IsInstanceSimulatingPhysics())
	{
		FVector AngVel = LeftBodyInst->GetUnrealWorldAngularVelocityInRadians();
		float AngVelMagnitude = AngVel.Size();
		
		// Only apply braking above deadband to prevent oscillation
		if (AngVelMagnitude > BrakeDeadband)
		{
			// Apply damping torque proportional to angular velocity
			FVector DampingTorque = -AngVel * BrakeTorque * 100.0f; // Convert to UE units
			
			LeftBodyInst->AddTorqueInRadians(DampingTorque, false, true);
			LeftWheelState.AppliedTorque = -FMath::Sign(LeftWheelState.AngularVelocity) * BrakeTorque;

			if (bEnableDebugLogging)
			{
				UE_LOG(LogTemp, Log, TEXT("[DiffDrive] Applying LEFT brake: Damping=%.3f Nm (AngVel: %.3f rad/s)"),
					BrakeTorque, LeftWheelState.AngularVelocity);
			}
		}
		else
		{
			// Below deadband - just zero out the applied torque
			LeftWheelState.AppliedTorque = 0.0f;
		}
	}

	FBodyInstance* RightBodyInst = GetBoneBodyInstance(RightWheelBoneName);
	if (RightBodyInst && RightBodyInst->IsInstanceSimulatingPhysics())
	{
		FVector AngVel = RightBodyInst->GetUnrealWorldAngularVelocityInRadians();
		float AngVelMagnitude = AngVel.Size();
		
		// Only apply braking above deadband to prevent oscillation
		if (AngVelMagnitude > BrakeDeadband)
		{
			// Apply damping torque proportional to angular velocity
			FVector DampingTorque = -AngVel * BrakeTorque * 100.0f; // Convert to UE units
			
			RightBodyInst->AddTorqueInRadians(DampingTorque, false, true);
			RightWheelState.AppliedTorque = -FMath::Sign(RightWheelState.AngularVelocity) * BrakeTorque;

			if (bEnableDebugLogging)
			{
				UE_LOG(LogTemp, Log, TEXT("[DiffDrive] Applying RIGHT brake: Damping=%.3f Nm (AngVel: %.3f rad/s)"),
					BrakeTorque, RightWheelState.AngularVelocity);
			}
		}
		else
		{
			// Below deadband - just zero out the applied torque
			RightWheelState.AppliedTorque = 0.0f;
		}
	}

	// Reset PID integral terms while braking
	LeftIntegralError = 0.0f;
	RightIntegralError = 0.0f;
}

void URammsDifferentialDriveController::DebugLogState()
{
	AActor* Owner = GetOwner();
	FString OwnerName = Owner ? Owner->GetName() : TEXT("None");

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("========== DiffDrive Debug [%s] =========="), *OwnerName);
		UE_LOG(LogTemp, Log, TEXT("Input: X=%.3f, Y=%.3f"), DriveInput.X, DriveInput.Y);
		UE_LOG(LogTemp, Log, TEXT("Control Mode: %s"), ControlMode == EDriveControlMode::TorqueControl ? TEXT("Torque") : TEXT("Velocity"));
		UE_LOG(LogTemp, Log, TEXT("Braking: %s"), bIsBraking ? TEXT("YES") : TEXT("NO"));
		UE_LOG(LogTemp, Log, TEXT("SkeletalMesh: %s"), SkeletalMeshComponent ? *SkeletalMeshComponent->GetName() : TEXT("NULL"));
		UE_LOG(LogTemp, Log, TEXT("Left Wheel [%s]:"), *LeftWheelBoneName.ToString());
		UE_LOG(LogTemp, Log, TEXT("  AngularVel: %.3f rad/s (%.1f RPM)"), LeftWheelState.AngularVelocity, URammsDifferentialDriveLibrary::RadPerSecToRPM(LeftWheelState.AngularVelocity));
		UE_LOG(LogTemp, Log, TEXT("  LinearVel: %.3f cm/s"), LeftWheelState.LinearVelocity);
		UE_LOG(LogTemp, Log, TEXT("  Target: %.3f rad/s"), LeftWheelState.TargetAngularVelocity);
		UE_LOG(LogTemp, Log, TEXT("  Torque: %.3f Nm"), LeftWheelState.AppliedTorque);
		UE_LOG(LogTemp, Log, TEXT("Right Wheel [%s]:"), *RightWheelBoneName.ToString());
		UE_LOG(LogTemp, Log, TEXT("  AngularVel: %.3f rad/s (%.1f RPM)"), RightWheelState.AngularVelocity, URammsDifferentialDriveLibrary::RadPerSecToRPM(RightWheelState.AngularVelocity));
		UE_LOG(LogTemp, Log, TEXT("  LinearVel: %.3f cm/s"), RightWheelState.LinearVelocity);
		UE_LOG(LogTemp, Log, TEXT("  Target: %.3f rad/s"), RightWheelState.TargetAngularVelocity);
		UE_LOG(LogTemp, Log, TEXT("  Torque: %.3f Nm"), RightWheelState.AppliedTorque);
		UE_LOG(LogTemp, Log, TEXT("Parameters: WheelRadius=%.1f cm, TrackWidth=%.1f cm, MaxTorque=%.1f Nm"),
			WheelRadius, TrackWidth, LeftMotorParams.MaxTorque);
		UE_LOG(LogTemp, Log, TEXT("=========================================="));
	}

	if (bEnableDebugDisplay && Owner)
	{
		// Get actual turning speed from root component
		float CurrentTurningSpeed = 0.0f;
		if (UPrimitiveComponent* RootPrimitive = Cast<UPrimitiveComponent>(Owner->GetRootComponent()))
		{
			if (RootPrimitive->IsSimulatingPhysics())
			{
				FVector AngularVelocity = RootPrimitive->GetPhysicsAngularVelocityInDegrees();
				CurrentTurningSpeed = AngularVelocity.Z;
			}
		}
		
		FVector ActorLocation = Owner->GetActorLocation();
		FString DebugText = FString::Printf(
			TEXT("Input: %.2f, %.2f | Mode: %s | Braking: %s\n")
			TEXT("Left: %.1f RPM, %.1f Nm | Right: %.1f RPM, %.1f Nm\n")
			TEXT("Turn Speed: %.1f deg/s (Max: %.1f)\n")
			TEXT("Odom Pos: X=%.0f Y=%.0f Yaw=%.1f°\n")
			TEXT("Odom Vel: %.1f cm/s | Dist: %.1f cm"),
			DriveInput.X, DriveInput.Y,
			ControlMode == EDriveControlMode::TorqueControl ? TEXT("Torque") : TEXT("Velocity"),
			bIsBraking ? TEXT("ON") : TEXT("OFF"),
			URammsDifferentialDriveLibrary::RadPerSecToRPM(LeftWheelState.AngularVelocity), LeftWheelState.AppliedTorque,
			URammsDifferentialDriveLibrary::RadPerSecToRPM(RightWheelState.AngularVelocity), RightWheelState.AppliedTorque,
			CurrentTurningSpeed, MaxTurningSpeed,
			Odometry.Position.X, Odometry.Position.Y, Odometry.Orientation.Yaw,
			Odometry.LinearVelocity.Size(), Odometry.TotalDistance);

		DrawDebugString(GetWorld(), ActorLocation + FVector(0, 0, 100), DebugText, nullptr, FColor::Yellow, 0.0f, true, 1.2f);
	}
}
