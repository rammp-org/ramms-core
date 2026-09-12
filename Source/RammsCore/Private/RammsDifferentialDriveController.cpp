// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsDifferentialDriveController.h"
#include "RammsDifferentialDriveLibrary.h"
#include "RammsDriveBackend.h"
#include "RammsDriveBackendRegistry.h"
#include "RammsChaosSkeletalDriveBackend.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"
#include "DrawDebugHelpers.h"

URammsDifferentialDriveController::URammsDifferentialDriveController()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

URammsDifferentialDriveController::~URammsDifferentialDriveController()
{
	// IRammsDriveBackend has a virtual destructor; delete here where it is a
	// complete type (the backend impl header is included below).
	delete DriveBackend;
	DriveBackend = nullptr;
}

void URammsDifferentialDriveController::BeginPlay()
{
	Super::BeginPlay();

	// Pick the drive backend. When MuJoCo is requested (explicitly or Auto),
	// ask the registry — RammsMujocoSupport registers a factory that drives a
	// URLab articulation's wheel actuators. Fall back to the built-in Chaos
	// skeletal-wheel backend when no factory is registered or the MuJoCo backend
	// can't resolve an articulation (Initialize returns false).
	delete DriveBackend; // guard against a second BeginPlay
	DriveBackend = nullptr;

	if (PhysicsBackend == EDrivePhysicsBackend::Mujoco || PhysicsBackend == EDrivePhysicsBackend::Auto)
	{
		if (IRammsDriveBackend* Mujoco = RammsDriveBackends::CreateMujocoBackend(*this))
		{
			if (Mujoco->Initialize(*this))
			{
				DriveBackend = Mujoco;
			}
			else
			{
				delete Mujoco; // no articulation resolved — fall through to Chaos
			}
		}
		if (!DriveBackend && PhysicsBackend == EDrivePhysicsBackend::Mujoco)
		{
			UE_LOG(LogTemp, Warning, TEXT("[DiffDrive] MuJoCo backend requested but unavailable; using Chaos."));
		}
	}

	if (!DriveBackend)
	{
		FRammsChaosSkeletalDriveBackend* Chaos = new FRammsChaosSkeletalDriveBackend();
		Chaos->Initialize(*this);
		DriveBackend = Chaos;
	}

	// Initialize odometry with the actor's current transform.
	if (AActor* Owner = GetOwner())
	{
		Odometry = URammsDifferentialDriveLibrary::ResetOdometry(
			Owner->GetActorLocation(),
			Owner->GetActorRotation());
	}

	// Initialize previous wheel rotations.
	PreviousLeftRotation = 0.0f;
	PreviousRightRotation = 0.0f;
}

void URammsDifferentialDriveController::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!DriveBackend)
	{
		return;
	}

	// Update wheel states (read from the physics backend)
	DriveBackend->ReadWheelState(ERammsDriveWheel::Left, LeftWheelState);
	DriveBackend->ReadWheelState(ERammsDriveWheel::Right, RightWheelState);

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

void URammsDifferentialDriveController::ApplyDriveInputInternal(FVector2D Input, const TCHAR* SourceLabel)
{
	DriveInput = Input;

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[DiffDrive] %s: X=%.3f, Y=%.3f"), SourceLabel, Input.X, Input.Y);
	}

	// Reset integral terms when input changes significantly (prevents windup)
	if (Input.SizeSquared() < 0.01f)
	{
		LeftIntegralError = 0.0f;
		RightIntegralError = 0.0f;
	}
}

void URammsDifferentialDriveController::SetDriveInput(FVector2D Input)
{
	// The pawn writes this every tick from its input bindings (BP Event Tick →
	// Enhanced Input values) — while an external access/autonomy session holds
	// priority, those per-tick writes must not clobber the external command.
	if (IsExternalDriveActive())
	{
		return;
	}
	ApplyDriveInputInternal(Input, TEXT("SetDriveInput"));
}

void URammsDifferentialDriveController::SetExternalDriveInput(FVector2D Input)
{
	LastExternalInputSeconds = FPlatformTime::Seconds();
	ApplyDriveInputInternal(Input, TEXT("SetExternalDriveInput"));
}

bool URammsDifferentialDriveController::IsExternalDriveActive() const
{
	return LastExternalInputSeconds >= 0.0
		&& (FPlatformTime::Seconds() - LastExternalInputSeconds) < ExternalInputHoldSeconds;
}

void URammsDifferentialDriveController::ResetOdometry(FVector Position, FRotator Orientation)
{
	Odometry = URammsDifferentialDriveLibrary::ResetOdometry(Position, Orientation);
	PreviousLeftRotation = LeftWheelState.TotalRotation;
	PreviousRightRotation = RightWheelState.TotalRotation;
}

void URammsDifferentialDriveController::UpdateTorqueControl(float DeltaTime)
{
	// Convert joystick input to differential drive commands (torque values)
	FDifferentialDriveCommand Command = URammsDifferentialDriveLibrary::JoystickToDifferentialDrive(
		DriveInput,
		1.0f, // Max value is 1.0, we'll scale by max torque later
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
	float LeftResistiveTorque = -FMath::Sign(LeftWheelState.AngularVelocity) * ResistiveTorqueCoefficient * FMath::Abs(LeftWheelState.AngularVelocity);
	float RightResistiveTorque = -FMath::Sign(RightWheelState.AngularVelocity) * ResistiveTorqueCoefficient * FMath::Abs(RightWheelState.AngularVelocity);

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

	// Apply torques to wheels (through the physics backend)
	DriveBackend->ApplyWheelTorque(ERammsDriveWheel::Left, LeftTorque, LeftMotorParams, LeftWheelState);
	DriveBackend->ApplyWheelTorque(ERammsDriveWheel::Right, RightTorque, RightMotorParams, RightWheelState);
}

void URammsDifferentialDriveController::UpdateVelocityControl(float DeltaTime)
{
	// Convert joystick input to differential drive commands (velocity values)
	FDifferentialDriveCommand VelocityCommand = URammsDifferentialDriveLibrary::JoystickToDifferentialDrive(
		DriveInput,
		MaxVelocity, // Max velocity in cm/s
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

	// Apply torques to wheels (through the physics backend)
	DriveBackend->ApplyWheelTorque(ERammsDriveWheel::Left, LeftTorque, LeftMotorParams, LeftWheelState);
	DriveBackend->ApplyWheelTorque(ERammsDriveWheel::Right, RightTorque, RightMotorParams, RightWheelState);
}

float URammsDifferentialDriveController::CalculatePID(
	float  Error,
	float& IntegralError,
	float& PreviousError,
	float  DeltaTime)
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
	bool  bIsMoving = AvgVelocity > BrakingVelocityThreshold;

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
	// Braking damping per wheel is the backend's job (engine-specific); the PID
	// integral reset is the controller's (backend-neutral control state).
	if (DriveBackend)
	{
		DriveBackend->ApplyBrake(ERammsDriveWheel::Left, LeftWheelState);
		DriveBackend->ApplyBrake(ERammsDriveWheel::Right, RightWheelState);
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
		UE_LOG(LogTemp, Log, TEXT("Drive backend: %s"), DriveBackend ? TEXT("active") : TEXT("none"));
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
