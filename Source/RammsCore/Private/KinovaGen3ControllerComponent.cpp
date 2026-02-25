// Copyright Epic Games, Inc. All Rights Reserved.

#include "KinovaGen3ControllerComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"
#include "DrawDebugHelpers.h"
#include "RammsIKLibrary.h"
#include "Engine/SkeletalMeshSocket.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "UObject/UnrealType.h"
#include "Engine/SCS_Node.h"
#if WITH_EDITOR
	#include "Engine/SimpleConstructionScript.h"
	#include "Kismet2/BlueprintEditorUtils.h"
	#include "ScopedTransaction.h"
#endif

UKinovaGen3ControllerComponent::UKinovaGen3ControllerComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

#if WITH_EDITOR
void UKinovaGen3ControllerComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	FName PropertyName = (PropertyChangedEvent.Property != nullptr) ? PropertyChangedEvent.Property->GetFName() : NAME_None;

	// Auto-populate joints when skeletal mesh component name changes
	if (PropertyName == GET_MEMBER_NAME_CHECKED(UKinovaGen3ControllerComponent, SkeletalMeshComponentName))
	{
		if (bAutoPopulateOnSkeletalMeshChange && SkeletalMeshComponentName != NAME_None)
		{
			UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] SkeletalMeshComponentName changed to %s, auto-populating joints..."), *SkeletalMeshComponentName.ToString());
			AutoPopulateJoints(true);
		}
	}
}
#endif

void UKinovaGen3ControllerComponent::BeginPlay()
{
	Super::BeginPlay();

	// Find the skeletal mesh component
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		Owner = GetTypedOuter<AActor>();
	}
	if (Owner)
	{
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
			SkeletalMeshComponent = Owner->FindComponentByClass<USkeletalMeshComponent>();
		}

		if (!SkeletalMeshComponent)
		{
			UE_LOG(LogTemp, Warning, TEXT("KinovaGen3Controller: Failed to find skeletal mesh component on %s"), *Owner->GetName());
		}
		else
		{
			UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Found skeletal mesh component: %s"), *SkeletalMeshComponent->GetName());

			// Initialize current angles from current constraint positions
			for (FRevoluteJointConfig& Joint : Joints)
			{
				if (Joint.BoneName == NAME_None)
					continue;

				FName				 ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
				FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintToUse);

				if (Constraint)
				{
					// Auto-detect best axis: pick the one with largest range
					// Only auto-detect if still on default (Twist) or if that axis is locked
					if (Joint.ControlledAxis == EConstraintAxis::Twist && !IsAxisFreeOrLimited(Constraint, EConstraintAxis::Twist))
					{
						// Twist is default but not available, pick axis with largest range
						float TwistRange = GetConstraintAxisRange(Constraint, EConstraintAxis::Twist);
						float Swing1Range = GetConstraintAxisRange(Constraint, EConstraintAxis::Swing1);
						float Swing2Range = GetConstraintAxisRange(Constraint, EConstraintAxis::Swing2);

						if (Swing2Range >= Swing1Range && Swing2Range >= TwistRange && Swing2Range > 0.0f)
						{
							Joint.ControlledAxis = EConstraintAxis::Swing2;
						}
						else if (Swing1Range >= TwistRange && Swing1Range > 0.0f)
						{
							Joint.ControlledAxis = EConstraintAxis::Swing1;
						}
						// Otherwise stays as Twist
					}

					// Auto-detect axis inversion by checking constraint frame orientation
					if (!Joint.bInvertAxisForIK) // Only auto-detect if not manually set
					{
						Joint.bInvertAxisForIK = ShouldInvertAxisForIK(Constraint, Joint.BoneName, Joint.ControlledAxis);
					}

					// Get current angular position
					Joint.CurrentAngle = GetConstraintAngle(Constraint, Joint.ControlledAxis);
					if (!bPreserveJointTargetsOnBeginPlay || ArmControlMode == EArmControlMode::EndEffectorControl)
					{
						Joint.TargetAngle = Joint.CurrentAngle;
					}
					Joint.SmoothedAngle = Joint.CurrentAngle;
					Joint.bSmoothedAngleInitialized = true;

					if (bEnableDebugLogging)
					{
						UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Initialized joint %s on %s axis at %.2f degrees (inverted=%d)"),
							*Joint.BoneName.ToString(),
							Joint.ControlledAxis == EConstraintAxis::Twist ? TEXT("Twist") : Joint.ControlledAxis == EConstraintAxis::Swing1 ? TEXT("Swing1")
																																			 : TEXT("Swing2"),
							Joint.CurrentAngle,
							Joint.bInvertAxisForIK);
					}
				}
				else
				{
					UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] Could not find constraint for joint %s"),
						*Joint.BoneName.ToString());
				}
			}

			// Initialize constraint drives after joint configuration
			// InitializeJointConstraints(); // drives must be enabled for SetAngularOrientationTarget to work
			// NOTE: This is commented out to avoid reconfiguring constraint drives that user has already set up

			// Initialize null-space bias to zeros if not set
			if (NullSpaceBias.Num() != Joints.Num())
			{
				NullSpaceBias.SetNum(Joints.Num());
				for (int32 i = 0; i < Joints.Num(); i++)
				{
					NullSpaceBias[i] = 0.0f; // Default to zero (no bias)
				}

				if (bEnableDebugLogging)
				{
					UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Initialized null-space bias with %d zeros"), NullSpaceBias.Num());
				}
			}

			UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Initialized with %d joints"), Joints.Num());

			// --- Snap joints to their editor-set TargetAngle immediately ---
			if (bPreserveJointTargetsOnBeginPlay && bSnapToTargetsOnBeginPlay
				&& ArmControlMode == EArmControlMode::JointControl)
			{
				for (FRevoluteJointConfig& Joint : Joints)
				{
					if (Joint.BoneName == NAME_None)
						continue;

					FName				 ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
					FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintToUse);
					if (!Constraint)
						continue;

					// Clamp if software limits are enabled
					float SnapAngle = Joint.TargetAngle;
					if (Joint.bEnableSoftwareLimits)
					{
						SnapAngle = ClampToLimits(Joint, SnapAngle);
					}

					// Set smoothed angle so the speed limiter doesn't fight on the first tick
					Joint.SmoothedAngle = SnapAngle;

					// Command the physics constraint directly
					const float ConstraintAngle = SnapAngle + Joint.AngleOffset;
					SetConstraintAngle(Constraint, Joint.ControlledAxis, ConstraintAngle);
				}

				UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Snapped %d joints to editor TargetAngle values"), Joints.Num());
			}
		}
	}
}

void UKinovaGen3ControllerComponent::InitializeJointConstraints()
{
	if (!SkeletalMeshComponent)
	{
		UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] Cannot initialize constraints: No skeletal mesh"));
		return;
	}

	int32 InitializedCount = 0;

	for (FRevoluteJointConfig& Joint : Joints)
	{
		if (Joint.BoneName == NAME_None)
			continue;

		FName				 ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
		FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintToUse);

		if (!Constraint)
		{
			UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] Cannot find constraint for joint %s"), *Joint.BoneName.ToString());
			continue;
		}

		// Configure angular drive - MUST use TwistAndSwing mode for individual axis control
		Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);

		// Enable only the specific axis we're controlling
		if (Joint.ControlledAxis == EConstraintAxis::Twist)
		{
			// Enable twist drive, disable swing drives
			Constraint->SetOrientationDriveTwistAndSwing(true, false);
			Constraint->SetAngularVelocityDriveTwistAndSwing(false, false);
		}
		else
		{
			// Enable swing drive, disable twist drive
			Constraint->SetOrientationDriveTwistAndSwing(false, true);
			Constraint->SetAngularVelocityDriveTwistAndSwing(false, false);
		}

		// Set drive parameters (Spring, Damping, MaxForce)
		Constraint->SetAngularDriveParams(Joint.PositionStrength, Joint.PositionDamping, Joint.MaxTorque);

		InitializedCount++;

		if (bEnableDebugLogging)
		{
			UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Initialized constraint for joint %s: Axis=%d, Strength=%.0f, Damping=%.0f, MaxTorque=%.1f"),
				*Joint.BoneName.ToString(),
				(int32)Joint.ControlledAxis,
				Joint.PositionStrength,
				Joint.PositionDamping,
				Joint.MaxTorque);
		}
	}

	UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Initialized %d joint constraints"), InitializedCount);

	// ============================================================================
	// Pre-compute and cache joint axes for FK/IK
	// ============================================================================

	// Now compute the axes using the CONSTRAINT Frame1/Frame2 orientation
	// Both axes and local transforms come from constraint frames for consistency
	CachedJointAxesLocal.SetNum(Joints.Num());

	for (int32 i = 0; i < Joints.Num(); i++)
	{
		FName				 ConstraintToUse = Joints[i].ConstraintName != NAME_None ? Joints[i].ConstraintName : Joints[i].BoneName;
		FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintToUse);

		if (!Constraint)
		{
			CachedJointAxesLocal[i] = FVector::XAxisVector;
			continue;
		}

		FTransform Frame1 = Constraint->GetRefFrame(EConstraintFrame::Frame1);

		// Axis directly from Frame1 (in parent body's local frame)
		FVector AxisLocal;
		switch (Joints[i].ControlledAxis)
		{
			case EConstraintAxis::Swing1:
				AxisLocal = Frame1.GetUnitAxis(EAxis::Z);
				break;
			case EConstraintAxis::Swing2:
				AxisLocal = Frame1.GetUnitAxis(EAxis::Y);
				break;
			case EConstraintAxis::Twist:
			default:
				AxisLocal = Frame1.GetUnitAxis(EAxis::X);
				break;
		}

		if (Joints[i].bInvertAxisForIK)
		{
			AxisLocal = -AxisLocal;
		}

		CachedJointAxesLocal[i] = AxisLocal;

		if (bEnableDebugLogging)
		{
			UE_LOG(LogTemp, Verbose, TEXT("[Gen3] Cached J%d Axis: (%.3f, %.3f, %.3f)"),
				i, CachedJointAxesLocal[i].X, CachedJointAxesLocal[i].Y, CachedJointAxesLocal[i].Z);
		}
	}

	UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Cached %d joint axes for FK/IK"), CachedJointAxesLocal.Num());
}

void UKinovaGen3ControllerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!SkeletalMeshComponent)
	{
		if (bEnableDebugLogging)
		{
			UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] TickComponent: No skeletal mesh component"));
		}
		return;
	}

	if (Joints.Num() == 0)
	{
		if (bEnableDebugLogging)
		{
			UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] TickComponent: No joints configured"));
		}
		return;
	}

	// Update control based on arm control mode
	if (ArmControlMode == EArmControlMode::EndEffectorControl)
	{
		// If solver type changes, force a fresh solve
		if (!bIKSolverTypeInitialized || IKSolverType != LastIKSolverType)
		{
			bIKSolverTypeInitialized = true;
			LastIKSolverType = IKSolverType;
			bIKTargetInitialized = false;
			bIKTargetSatisfied = false;
		}

		// Keep target in sync with actor if provided
		if (TargetActor && TargetActor->IsValidLowLevel())
		{
			TargetEndEffectorTransform = TargetActor->GetActorTransform();
		}

		// Only solve IK while we are moving toward a target or the target has changed
		bool bTargetChanged = false;
		if (!bIKTargetInitialized)
		{
			bTargetChanged = true;
			bIKTargetInitialized = true;
		}
		else
		{
			const float PosDelta = FVector::Dist(TargetEndEffectorTransform.GetLocation(), LastIKTargetTransform.GetLocation());
			const float RotDelta = FMath::RadiansToDegrees(
				TargetEndEffectorTransform.GetRotation().AngularDistance(LastIKTargetTransform.GetRotation()));
			bTargetChanged = (PosDelta > IKTargetChangePosThreshold) || (RotDelta > IKTargetChangeRotThreshold);
		}

		if (bTargetChanged)
		{
			bIKTargetSatisfied = false;
			LastIKTargetTransform = TargetEndEffectorTransform;
		}

		// Detect base movement — if the base is moving, always re-solve to avoid
		// "bang-bang" jitter from intermittent solve/skip cycles.
		FVector CurrentBaseLocation = SkeletalMeshComponent->GetComponentTransform().GetLocation();
		if (Joints.Num() > 0)
		{
			FName ParentBone = SkeletalMeshComponent->GetParentBone(Joints[0].BoneName);
			if (ParentBone != NAME_None)
			{
				CurrentBaseLocation = SkeletalMeshComponent->GetSocketTransform(ParentBone, RTS_World).GetLocation();
			}
		}
		const float BaseMoveThreshold = 0.01f;
		const bool bBaseMoved = FVector::DistSquared(CurrentBaseLocation, LastSolveBaseLocation) > (BaseMoveThreshold * BaseMoveThreshold);

		if (!bIKTargetSatisfied || bBaseMoved)
		{
			UpdateInverseKinematics(DeltaTime);
			bIKTargetSatisfied = bLastIKSuccess;
			LastSolveBaseLocation = CurrentBaseLocation;
		}
	}

	// Update joint control based on mode
	switch (ControlMode)
	{
		case EJointControlMode::PositionControl:
			UpdatePositionControl(DeltaTime);
			break;
		case EJointControlMode::VelocityControl:
			UpdateVelocityControl(DeltaTime);
			break;
		case EJointControlMode::TorqueControl:
			UpdateTorqueControl(DeltaTime);
			break;
	}

	// Update end-effector state
	UpdateEndEffectorState();

	// Debug visualization
	if (bEnableDebugDisplay || bShowJointFrames)
	{
		DebugDraw();
	}
}

void UKinovaGen3ControllerComponent::UpdatePositionControl(float DeltaTime)
{
	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] UpdatePositionControl: %d joints"), Joints.Num());
	}

	for (FRevoluteJointConfig& Joint : Joints)
	{
		if (Joint.BoneName == NAME_None)
			continue;

		FName				 ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
		FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintToUse);

		if (!Constraint)
		{
			if (bEnableDebugLogging)
			{
				UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] UpdatePositionControl: Cannot find constraint %s"), *ConstraintToUse.ToString());
			}
			continue;
		}

		// Read current angle from physics (subtract offsets to get FK angle)
		// Joint.CurrentAngle = GetConstraintAngle(Constraint, Joint.ControlledAxis)
		// 	- Joint.ComputedFrameOffset - Joint.AngleOffset;
		Joint.CurrentAngle = GetConstraintAngle(Constraint, Joint.ControlledAxis)
			- Joint.AngleOffset;

		// Note: In IK mode, TargetAngle is set by IK solver directly
		// We still apply it through ApplyJointSettings which handles clamping

		// Apply to constraint drive
		ApplyJointSettings(Joint, DeltaTime);
	}
}

void UKinovaGen3ControllerComponent::UpdateVelocityControl(float DeltaTime)
{
	// Similar to position control but with velocity-based PD control
	// For now, use position control (can be extended later)
	UpdatePositionControl(DeltaTime);
}

void UKinovaGen3ControllerComponent::UpdateTorqueControl(float DeltaTime)
{
	// Torque control mode - apply torques directly based on error
	for (FRevoluteJointConfig& Joint : Joints)
	{
		if (Joint.BoneName == NAME_None)
			continue;

		FName				 ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
		FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintToUse);

		if (!Constraint)
		{
			if (bEnableDebugLogging)
			{
				UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] UpdateTorqueControl: Cannot find constraint %s"), *ConstraintToUse.ToString());
			}
			continue;
		}

		// Get current angle from physics
		Joint.CurrentAngle = GetConstraintAngle(Constraint, Joint.ControlledAxis);

		// Calculate error
		float AngleError = Joint.TargetAngle - Joint.CurrentAngle;
		while (AngleError > 180.0f)
			AngleError -= 360.0f;
		while (AngleError < -180.0f)
			AngleError += 360.0f;

		// Simple P controller for torque
		float Torque = AngleError * 0.1f; // Proportional gain
		Torque = FMath::Clamp(Torque, -Joint.MaxTorque, Joint.MaxTorque);

		// Apply torque (this would need to be done through body instance)
		// For now, fall back to position control
		ApplyJointSettings(Joint, DeltaTime);
	}
}

void UKinovaGen3ControllerComponent::ApplyJointSettings(FRevoluteJointConfig& Joint, float DeltaTime)
{
	if (!SkeletalMeshComponent)
		return;

	FName				 ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
	FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintToUse);

	if (!Constraint)
	{
		if (bEnableDebugLogging)
		{
			UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] Cannot find constraint: %s"), *ConstraintToUse.ToString());
		}
		return;
	}

	// Clamp to limits if enabled
	float TargetForDrive = Joint.TargetAngle;
	if (Joint.bEnableSoftwareLimits)
	{
		TargetForDrive = ClampToLimits(Joint, TargetForDrive);
	}

	// Enforce max speed (deg/sec) by limiting setpoint change per tick.
	// Use current joint angle as the start to smoothly lerp toward the target.
	if (!Joint.bSmoothedAngleInitialized)
	{
		Joint.SmoothedAngle = Joint.CurrentAngle;
		Joint.bSmoothedAngleInitialized = true;
	}

	const float MaxDeltaDeg = Joint.MaxAngularSpeed * Joint.SpeedMultiplier * DeltaTime;
	if (MaxDeltaDeg > 0.0f)
	{
		const float DeltaDeg = FMath::FindDeltaAngleDegrees(Joint.SmoothedAngle, TargetForDrive);
		const float ClampedDelta = FMath::Clamp(DeltaDeg, -MaxDeltaDeg, MaxDeltaDeg);
		Joint.SmoothedAngle = Joint.SmoothedAngle + ClampedDelta;
	}
	else
	{
		Joint.SmoothedAngle = TargetForDrive;
	}

	if (Joint.bEnableSoftwareLimits)
	{
		Joint.SmoothedAngle = ClampToLimits(Joint, Joint.SmoothedAngle);
	}

	TargetForDrive = Joint.SmoothedAngle;

	// Add offsets when commanding (FK angle -> constraint angle)
	const float ConstraintAngle = TargetForDrive + Joint.AngleOffset; // (see note below)
	SetConstraintAngle(Constraint, Joint.ControlledAxis, ConstraintAngle);

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Joint %s: Current=%.2f Target=%.2f"),
			*Joint.BoneName.ToString(),
			Joint.CurrentAngle,
			TargetForDrive);
	}
}

void UKinovaGen3ControllerComponent::UpdateEndEffectorState()
{
	if (!SkeletalMeshComponent)
	{
		if (bEnableDebugLogging)
		{
			UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] UpdateEndEffectorState: No skeletal mesh"));
		}
		return;
	}

	if (EndEffectorBoneName == NAME_None)
	{
		if (bEnableDebugLogging)
		{
			UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] UpdateEndEffectorState: EndEffectorBoneName is None"));
		}
		return;
	}

	// Check if it's a valid bone or socket
	bool bIsBone = SkeletalMeshComponent->GetBoneIndex(EndEffectorBoneName) != INDEX_NONE;
	bool bIsSocket = SkeletalMeshComponent->DoesSocketExist(EndEffectorBoneName);

	if (!bIsBone && !bIsSocket)
	{
		if (bEnableDebugLogging)
		{
			UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] UpdateEndEffectorState: '%s' is neither a bone nor a socket"), *EndEffectorBoneName.ToString());
		}
		return;
	}

	// Get transform in world space (works for both bones and sockets)
	FTransform EndEffectorTransform = SkeletalMeshComponent->GetSocketTransform(EndEffectorBoneName, RTS_World);

	// Update position and rotation
	FVector PreviousPosition = EndEffectorState.Position;
	EndEffectorState.Position = EndEffectorTransform.GetLocation();
	EndEffectorState.Rotation = EndEffectorTransform.Rotator();

	// Estimate velocities (simple finite difference)
	float DeltaTime = GetWorld()->GetDeltaSeconds();
	if (DeltaTime > SMALL_NUMBER)
	{
		EndEffectorState.LinearVelocity = (EndEffectorState.Position - PreviousPosition) / DeltaTime;

		// Angular velocity would require storing previous rotation
		// For now, leave it as zero (can be improved)
	}
}

float UKinovaGen3ControllerComponent::ClampToLimits(const FRevoluteJointConfig& Joint, float Angle) const
{
	// Check if this is a free joint (unlimited rotation): range >= 359 degrees
	const float Range = Joint.MaxAngleLimit - Joint.MinAngleLimit;
	if (Range >= 359.0f)
	{
		// Free joint: wrap angle to [-180, 180] instead of clamping
		return FMath::UnwindDegrees(Angle);
	}

	// Constrained joint: clamp to limits
	return FMath::Clamp(Angle, Joint.MinAngleLimit, Joint.MaxAngleLimit);
}

float UKinovaGen3ControllerComponent::GetConstraintAngle(FConstraintInstance* Constraint, EConstraintAxis Axis) const
{
	if (!Constraint)
		return 0.0f;

	// Get the current angle from the constraint's physics state
	switch (Axis)
	{
		case EConstraintAxis::Twist:
			return FMath::RadiansToDegrees(Constraint->GetCurrentTwist());
		case EConstraintAxis::Swing1:
			return FMath::RadiansToDegrees(Constraint->GetCurrentSwing1());
		case EConstraintAxis::Swing2:
			return FMath::RadiansToDegrees(Constraint->GetCurrentSwing2());
		default:
			return 0.0f;
	}
}

void UKinovaGen3ControllerComponent::SetConstraintAngle(FConstraintInstance* Constraint, EConstraintAxis Axis, float AngleDegrees) const
{
	if (!Constraint)
		return;

	// Ensure angular drives are enabled and configured for this axis
	// (Safety check - drives should be configured in InitializeJointConstraints, but we ensure it here)
	Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);

	if (Axis == EConstraintAxis::Twist)
	{
		Constraint->SetOrientationDriveTwistAndSwing(true, false);
	}
	else // Swing1 or Swing2
	{
		Constraint->SetOrientationDriveTwistAndSwing(false, true);
	}

	// SetAngularOrientationTarget expects a quaternion representing the target orientation
	// of the child body (Frame2) relative to the parent body (Frame1), in Frame1's coordinate system.
	// Since our angle is already in Frame1's axes (Twist=X, Swing1=Z, Swing2=Y), we just create
	// a rotation quaternion around the appropriate axis.

	float AngleRadians = FMath::DegreesToRadians(AngleDegrees);
	FQuat TargetQuat = FQuat::Identity;

	switch (Axis)
	{
		case EConstraintAxis::Swing1:
			TargetQuat = FQuat(FVector(0, 0, 1), AngleRadians); // Z-axis
			break;
		case EConstraintAxis::Swing2:
			TargetQuat = FQuat(FVector(0, 1, 0), AngleRadians); // Y-axis
			break;
		case EConstraintAxis::Twist:
			TargetQuat = FQuat(FVector(1, 0, 0), AngleRadians); // X-axis
			break;
	}

	Constraint->SetAngularOrientationTarget(TargetQuat);
}

bool UKinovaGen3ControllerComponent::IsAxisFreeOrLimited(FConstraintInstance* Constraint, EConstraintAxis Axis) const
{
	if (!Constraint)
		return false;

	EAngularConstraintMotion Motion;

	switch (Axis)
	{
		case EConstraintAxis::Twist:
			Motion = Constraint->GetAngularTwistMotion();
			break;
		case EConstraintAxis::Swing1:
			Motion = Constraint->GetAngularSwing1Motion();
			break;
		case EConstraintAxis::Swing2:
			Motion = Constraint->GetAngularSwing2Motion();
			break;
		default:
			return false;
	}

	return Motion == EAngularConstraintMotion::ACM_Free || Motion == EAngularConstraintMotion::ACM_Limited;
}

float UKinovaGen3ControllerComponent::GetConstraintAxisRange(FConstraintInstance* Constraint, EConstraintAxis Axis) const
{
	if (!Constraint)
		return 0.0f;

	EAngularConstraintMotion Motion;
	float					 Limit1 = 0.0f;
	float					 Limit2 = 0.0f;

	switch (Axis)
	{
		case EConstraintAxis::Twist:
			Motion = Constraint->GetAngularTwistMotion();
			Limit1 = Constraint->GetAngularTwistLimit();
			Limit2 = Limit1; // Twist is symmetric
			break;
		case EConstraintAxis::Swing1:
			Motion = Constraint->GetAngularSwing1Motion();
			Limit1 = Constraint->GetAngularSwing1Limit();
			Limit2 = Limit1; // Swing is symmetric
			break;
		case EConstraintAxis::Swing2:
			Motion = Constraint->GetAngularSwing2Motion();
			Limit1 = Constraint->GetAngularSwing2Limit();
			Limit2 = Limit1; // Swing is symmetric
			break;
		default:
			return 0.0f;
	}

	if (Motion == EAngularConstraintMotion::ACM_Locked)
		return 0.0f;

	if (Motion == EAngularConstraintMotion::ACM_Free)
		return 360.0f; // Full rotation

	// ACM_Limited - return the range
	return Limit1 + Limit2;
}

bool UKinovaGen3ControllerComponent::ShouldInvertAxisForIK(FConstraintInstance* Constraint, FName BoneName, EConstraintAxis Axis) const
{
	if (!Constraint || !SkeletalMeshComponent)
		return false;

	// Get constraint frames
	FTransform ConstraintFrame1 = Constraint->GetRefFrame(EConstraintFrame::Frame1);

	// Get parent body to transform constraint frame to world space
	FBodyInstance* ParentBody = nullptr;
	int32		   BoneIndex = SkeletalMeshComponent->GetBoneIndex(BoneName);
	if (BoneIndex != INDEX_NONE)
	{
		FName ParentBoneName = SkeletalMeshComponent->GetParentBone(BoneName);
		if (ParentBoneName != NAME_None)
		{
			ParentBody = SkeletalMeshComponent->GetBodyInstance(ParentBoneName);
		}
	}

	if (!ParentBody)
		return false; // Can't determine without parent body

	// Transform constraint frame to world space
	FTransform ParentWorldTransform = ParentBody->GetUnrealWorldTransform();
	FTransform ConstraintWorldFrame = ConstraintFrame1 * ParentWorldTransform;

	// Get bone transform for comparison
	FTransform BoneTransform = SkeletalMeshComponent->GetSocketTransform(BoneName, RTS_World);

	// Get the axis direction in world space
	FVector AxisDirection;
	switch (Axis)
	{
		case EConstraintAxis::Twist:
			AxisDirection = ConstraintWorldFrame.GetUnitAxis(EAxis::X);
			break;
		case EConstraintAxis::Swing1:
			AxisDirection = ConstraintWorldFrame.GetUnitAxis(EAxis::Z);
			break;
		case EConstraintAxis::Swing2:
			AxisDirection = ConstraintWorldFrame.GetUnitAxis(EAxis::Y);
			break;
		default:
			return false;
	}

	// Get the "forward" direction of the bone (positive X by convention)
	FVector BoneForward = BoneTransform.GetUnitAxis(EAxis::X);

	// If the axis is pointing more backwards than forwards relative to bone direction,
	// we should invert it for IK
	float DotProduct = FVector::DotProduct(AxisDirection, BoneForward);

	// Invert if axis points significantly backwards (dot product < -0.5 means > 120 degrees)
	return DotProduct < -0.3f;
}

void UKinovaGen3ControllerComponent::AutoPopulateJoints(bool bOverwriteExisting)
{
	UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] AutoPopulateJoints: Template=%d Outer=%s Owner=%s"),
		IsTemplate() ? 1 : 0, *GetNameSafe(GetOuter()), *GetNameSafe(GetOwner()));

	USkeletalMeshComponent* TempSkeletalMesh = SkeletalMeshComponent;

	// If not already set, try to find it
	if (!TempSkeletalMesh)
	{
		AActor* Owner = GetOwner();
		if (Owner)
		{
			// print the name of the owner actor for debugging
			UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] AutoPopulateJoints: Owner actor is %s"), *Owner->GetName());

			if (SkeletalMeshComponentName != NAME_None)
			{
				TArray<USkeletalMeshComponent*> SkeletalMeshes;
				Owner->GetComponents<USkeletalMeshComponent>(SkeletalMeshes);

				for (USkeletalMeshComponent* SkelMesh : SkeletalMeshes)
				{
					if (SkelMesh && SkelMesh->GetFName() == SkeletalMeshComponentName)
					{
						TempSkeletalMesh = SkelMesh;
						break;
					}
				}
			}
			else
			{
				TempSkeletalMesh = Owner->FindComponentByClass<USkeletalMeshComponent>();
			}
		}
		else
		{
			// Try SCS node template directly
			if (USCS_Node* OwningNode = GetTypedOuter<USCS_Node>())
			{
				USkeletalMeshComponent* TemplateSkel = Cast<USkeletalMeshComponent>(OwningNode->ComponentTemplate);
				if (TemplateSkel && (SkeletalMeshComponentName == NAME_None || TemplateSkel->GetFName() == SkeletalMeshComponentName))
				{
					TempSkeletalMesh = TemplateSkel;
				}
			}

			// Blueprint editor/template path: find component on the CDO
			UObject*				  OuterObj = GetOuter();
			UBlueprintGeneratedClass* BPGC = Cast<UBlueprintGeneratedClass>(OuterObj);
			AActor*					  CDOActor = BPGC ? Cast<AActor>(BPGC->GetDefaultObject()) : nullptr;

			if (!CDOActor && OuterObj)
			{
				UClass*	 OuterClass = OuterObj->GetClass();
				UObject* CDOObj = OuterClass ? OuterClass->GetDefaultObject() : nullptr;
				CDOActor = Cast<AActor>(CDOObj);
			}

			if (CDOActor)
			{
				if (bEnableDebugLogging)
				{
					TArray<UActorComponent*> CDOComponents;
					CDOActor->GetComponents(CDOComponents);
					UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] CDO component dump (%d components):"), CDOComponents.Num());
					for (UActorComponent* Comp : CDOComponents)
					{
						USkeletalMeshComponent* SkelComp = Cast<USkeletalMeshComponent>(Comp);
						UE_LOG(LogTemp, Verbose, TEXT("  - %s (%s)"), *GetNameSafe(Comp), SkelComp ? TEXT("SkeletalMeshComponent") : TEXT("Other"));
					}
				}

				if (SkeletalMeshComponentName != NAME_None)
				{
					TArray<USkeletalMeshComponent*> SkeletalMeshes;
					CDOActor->GetComponents<USkeletalMeshComponent>(SkeletalMeshes);

					for (USkeletalMeshComponent* SkelMesh : SkeletalMeshes)
					{
						if (SkelMesh && SkelMesh->GetFName() == SkeletalMeshComponentName)
						{
							TempSkeletalMesh = SkelMesh;
							break;
						}
					}
				}
				else
				{
					TempSkeletalMesh = CDOActor->FindComponentByClass<USkeletalMeshComponent>();
				}
			}

			// If still not found, scan SCS nodes on the blueprint class directly
			if (!TempSkeletalMesh && BPGC && BPGC->SimpleConstructionScript)
			{
				const TArray<USCS_Node*> SCSNodes = BPGC->SimpleConstructionScript->GetAllNodes();
				if (bEnableDebugLogging)
				{
					UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] SCS node dump (%d nodes):"), SCSNodes.Num());
				}
				for (USCS_Node* Node : SCSNodes)
				{
					if (!Node)
					{
						continue;
					}
					UActorComponent*		TemplateComp = Node->ComponentTemplate;
					USkeletalMeshComponent* SkelTemplate = Cast<USkeletalMeshComponent>(TemplateComp);
					if (bEnableDebugLogging)
					{
						UE_LOG(LogTemp, Verbose, TEXT("  - %s (%s)"),
							*GetNameSafe(TemplateComp),
							SkelTemplate ? TEXT("SkeletalMeshComponent") : TEXT("Other"));
					}

					if (!SkelTemplate)
					{
						continue;
					}

					const FName	  TemplateName = SkelTemplate->GetFName();
					const FName	  VariableName = Node->GetVariableName();
					const FString TemplateNameStr = TemplateName.ToString();
					const FString TargetNameStr = SkeletalMeshComponentName.ToString();
					const FString TemplateBase = TemplateNameStr.Replace(TEXT("_GEN_VARIABLE"), TEXT(""));

					const bool bNameMatch =
						SkeletalMeshComponentName == NAME_None || TemplateName == SkeletalMeshComponentName || VariableName == SkeletalMeshComponentName || TemplateBase == TargetNameStr;

					if (bNameMatch)
					{
						TempSkeletalMesh = SkelTemplate;
						break;
					}
				}
			}
		}
	}

	if (!TempSkeletalMesh)
	{
		UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] Cannot auto-populate joints: No skeletal mesh component (Owner=%s Outer=%s OuterClass=%s)"),
			*GetNameSafe(GetOwner()),
			*GetNameSafe(GetOuter()),
			*GetNameSafe(GetOuter() ? GetOuter()->GetClass() : nullptr));
		return;
	}

	UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] AutoPopulateJoints called on %s"), *TempSkeletalMesh->GetName());
	UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] SkeletalMesh: %s"), TempSkeletalMesh->GetSkeletalMeshAsset() ? *TempSkeletalMesh->GetSkeletalMeshAsset()->GetName() : TEXT("None"));
	UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Physics enabled: %d"), TempSkeletalMesh->IsSimulatingPhysics());

	if (bOverwriteExisting)
	{
		Joints.Empty();
	}

	TArray<FRevoluteJointConfig> NewJoints;
	if (!bOverwriteExisting)
	{
		NewJoints = Joints;
	}

	// Get all physics constraints from the skeletal mesh (runtime) or physics asset (editor/CDO)
	TArray<FConstraintInstanceAccessor> ConstraintAccessors;
	TempSkeletalMesh->GetConstraints(false, ConstraintAccessors);

	UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Found %d constraint accessors on skeletal mesh"), ConstraintAccessors.Num());

	auto AddJointFromConstraint = [&](FConstraintInstance* Constraint) {
		if (!Constraint)
		{
			return;
		}

		UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Checking constraint: %s"), *Constraint->JointName.ToString());

		// Check if any axis is free or limited (indicating a revolute joint)
		bool bHasFreeAxis = IsAxisFreeOrLimited(Constraint, EConstraintAxis::Twist) || IsAxisFreeOrLimited(Constraint, EConstraintAxis::Swing1) || IsAxisFreeOrLimited(Constraint, EConstraintAxis::Swing2);

		UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3]   Has free axis: %d (Twist:%d Swing1:%d Swing2:%d)"),
			bHasFreeAxis,
			IsAxisFreeOrLimited(Constraint, EConstraintAxis::Twist),
			IsAxisFreeOrLimited(Constraint, EConstraintAxis::Swing1),
			IsAxisFreeOrLimited(Constraint, EConstraintAxis::Swing2));

		if (!bHasFreeAxis)
		{
			return;
		}

		// Create a joint config for this constraint
		FRevoluteJointConfig NewJoint;

		// Get joint name from the constraint
		NewJoint.ConstraintName = Constraint->JointName;
		NewJoint.BoneName = Constraint->JointName;

		// Auto-detect which axis is controlled
		if (IsAxisFreeOrLimited(Constraint, EConstraintAxis::Twist))
		{
			NewJoint.ControlledAxis = EConstraintAxis::Twist;
		}
		else
		{
			// For swing axes, pick the one with larger motion range
			bool bSwing1Free = IsAxisFreeOrLimited(Constraint, EConstraintAxis::Swing1);
			bool bSwing2Free = IsAxisFreeOrLimited(Constraint, EConstraintAxis::Swing2);

			if (bSwing1Free && bSwing2Free)
			{
				// Both are free/limited - pick the one with larger range
				float Swing1Limit = Constraint->GetAngularSwing1Limit();
				float Swing2Limit = Constraint->GetAngularSwing2Limit();

				if (Swing2Limit > Swing1Limit)
				{
					NewJoint.ControlledAxis = EConstraintAxis::Swing2;
				}
				else
				{
					NewJoint.ControlledAxis = EConstraintAxis::Swing1;
				}
			}
			else if (bSwing1Free)
			{
				NewJoint.ControlledAxis = EConstraintAxis::Swing1;
			}
			else if (bSwing2Free)
			{
				NewJoint.ControlledAxis = EConstraintAxis::Swing2;
			}
		}

		// Read limits from constraint if they exist
		switch (NewJoint.ControlledAxis)
		{
			case EConstraintAxis::Twist:
				if (Constraint->GetAngularTwistMotion() == EAngularConstraintMotion::ACM_Limited)
				{
					NewJoint.MinAngleLimit = -Constraint->GetAngularTwistLimit();
					NewJoint.MaxAngleLimit = Constraint->GetAngularTwistLimit();
				}
				else if (Constraint->GetAngularTwistMotion() == EAngularConstraintMotion::ACM_Free)
				{
					// Free rotation - set very large limits for IK
					NewJoint.MinAngleLimit = -180.0f;
					NewJoint.MaxAngleLimit = 180.0f;
				}
				break;
			case EConstraintAxis::Swing1:
				if (Constraint->GetAngularSwing1Motion() == EAngularConstraintMotion::ACM_Limited)
				{
					NewJoint.MinAngleLimit = -Constraint->GetAngularSwing1Limit();
					NewJoint.MaxAngleLimit = Constraint->GetAngularSwing1Limit();
				}
				else if (Constraint->GetAngularSwing1Motion() == EAngularConstraintMotion::ACM_Free)
				{
					NewJoint.MinAngleLimit = -180.0f;
					NewJoint.MaxAngleLimit = 180.0f;
				}
				break;
			case EConstraintAxis::Swing2:
				if (Constraint->GetAngularSwing2Motion() == EAngularConstraintMotion::ACM_Limited)
				{
					NewJoint.MinAngleLimit = -Constraint->GetAngularSwing2Limit();
					NewJoint.MaxAngleLimit = Constraint->GetAngularSwing2Limit();
				}
				else if (Constraint->GetAngularSwing2Motion() == EAngularConstraintMotion::ACM_Free)
				{
					NewJoint.MinAngleLimit = -180.0f;
					NewJoint.MaxAngleLimit = 180.0f;
				}
				break;
		}

		// Initialize current angle
		NewJoint.CurrentAngle = GetConstraintAngle(Constraint, NewJoint.ControlledAxis);
		NewJoint.TargetAngle = NewJoint.CurrentAngle;

		NewJoints.Add(NewJoint);

		UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Added joint: %s (Constraint: %s, Axis: %s, Limits: %.1f to %.1f)"),
			*NewJoint.BoneName.ToString(),
			*NewJoint.ConstraintName.ToString(),
			NewJoint.ControlledAxis == EConstraintAxis::Twist ? TEXT("Twist") : NewJoint.ControlledAxis == EConstraintAxis::Swing1 ? TEXT("Swing1")
																																   : TEXT("Swing2"),
			NewJoint.MinAngleLimit, NewJoint.MaxAngleLimit);
	};

	if (ConstraintAccessors.Num() > 0)
	{
		for (FConstraintInstanceAccessor& Accessor : ConstraintAccessors)
		{
			AddJointFromConstraint(Accessor.Get());
		}
	}
	else
	{
		// Fallback: use physics asset constraints (works in BP editor/when not simulating)
		USkeletalMesh* SkelAsset = TempSkeletalMesh->GetSkeletalMeshAsset();
		UPhysicsAsset* PhysAsset = SkelAsset ? SkelAsset->GetPhysicsAsset() : nullptr;
		if (!PhysAsset)
		{
			UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] No physics asset found for skeletal mesh"));
			return;
		}

		UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Using PhysicsAsset '%s' with %d constraints"), *PhysAsset->GetName(), PhysAsset->ConstraintSetup.Num());
		for (UPhysicsConstraintTemplate* Template : PhysAsset->ConstraintSetup)
		{
			if (!Template)
			{
				continue;
			}
			AddJointFromConstraint(&Template->DefaultInstance);
		}
	}

	Joints = NewJoints;

	UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Auto-populated %d joints"), Joints.Num());

#if WITH_EDITOR
	// Mark this component as modified so the blueprint saves the changes
	Modify();

	// If we're a component template in a blueprint, mark the blueprint as modified
	if (IsTemplate())
	{
		if (UBlueprint* Blueprint = Cast<UBlueprint>(GetOuter()->GetClass()->ClassGeneratedBy))
		{
			FBlueprintEditorUtils::MarkBlueprintAsStructurallyModified(Blueprint);
		}
	}
#endif
}

void UKinovaGen3ControllerComponent::AutoPopulateJointsFromConstraints()
{
	UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] AutoPopulateJointsFromConstraints invoked. IsTemplate=%d Outer=%s"),
		IsTemplate() ? 1 : 0,
		*GetNameSafe(GetOuter()));
	AutoPopulateJoints(true);
}

void UKinovaGen3ControllerComponent::DebugDraw()
{
	if (!SkeletalMeshComponent)
		return;

	UWorld* World = GetWorld();
	if (!World)
		return;

	// 0 life means single frame, >0 means persistent for that many seconds, negative means infinite
	const float DebugLife = 0.0f;
	const bool	bPersistent = false; // Set to true if you want the debug lines to persist for a while
	if (bPersistent)
	{
		FlushPersistentDebugLines(World);
	}

	// Always draw a small beacon so we can confirm debug draw is active
	DrawDebugSphere(World, SkeletalMeshComponent->GetComponentLocation(), 2.0f, 8, FColor::White, bPersistent, DebugLife, 0, 1.5f);

	// Draw joint frames
	if (bShowJointFrames)
	{
		for (int32 JointIdx = 0; JointIdx < Joints.Num(); JointIdx++)
		{
			const FRevoluteJointConfig& Joint = Joints[JointIdx];
			if (Joint.BoneName == NAME_None)
				continue;

			int32 BoneIndex = SkeletalMeshComponent->GetBoneIndex(Joint.BoneName);
			if (BoneIndex == INDEX_NONE)
				continue;

			// Get bone transform in world space
			FTransform BoneTransform = SkeletalMeshComponent->GetSocketTransform(Joint.BoneName, RTS_World);
			FVector	   Origin = BoneTransform.GetLocation();

			// Draw coordinate frame
			float AxisLength = 10.0f;
			DrawDebugLine(World, Origin, Origin + BoneTransform.GetUnitAxis(EAxis::X) * AxisLength, FColor::Red, bPersistent, DebugLife, 0, 1.0f);
			DrawDebugLine(World, Origin, Origin + BoneTransform.GetUnitAxis(EAxis::Y) * AxisLength, FColor::Green, bPersistent, DebugLife, 0, 1.0f);
			DrawDebugLine(World, Origin, Origin + BoneTransform.GetUnitAxis(EAxis::Z) * AxisLength, FColor::Blue, bPersistent, DebugLife, 0, 1.0f);

			// Draw the actual rotation axis for this joint (in IK mode, show which axis it rotates around)
			if (ArmControlMode == EArmControlMode::EndEffectorControl)
			{
				FName				 ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
				FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintToUse);

				if (Constraint)
				{
					// Get constraint frames
					FTransform ConstraintFrame1 = Constraint->GetRefFrame(EConstraintFrame::Frame1);
					FTransform ConstraintFrame2 = Constraint->GetRefFrame(EConstraintFrame::Frame2);

					// Get parent and child body transforms
					FBodyInstance* ParentBody = nullptr;
					if (BoneIndex != INDEX_NONE)
					{
						int32 ParentIndex = SkeletalMeshComponent->GetBoneIndex(SkeletalMeshComponent->GetParentBone(Joint.BoneName));
						if (ParentIndex != INDEX_NONE)
						{
							ParentBody = SkeletalMeshComponent->GetBodyInstance(SkeletalMeshComponent->GetBoneName(ParentIndex));
						}
					}

					// Transform constraint frame to world space
					FTransform ConstraintWorld1;
					if (ParentBody)
					{
						FTransform ParentWorldTransform = ParentBody->GetUnrealWorldTransform();
						ConstraintWorld1 = ConstraintFrame1 * ParentWorldTransform;
					}
					else
					{
						ConstraintWorld1 = ConstraintFrame2 * BoneTransform;
					}

					// Visualize ALL THREE constraint axes to verify alignment
					FVector XAxis = ConstraintWorld1.GetUnitAxis(EAxis::X); // Twist
					FVector YAxis = ConstraintWorld1.GetUnitAxis(EAxis::Y); // Swing2
					FVector ZAxis = ConstraintWorld1.GetUnitAxis(EAxis::Z); // Swing1

					float DebugAxisLen = AxisLength * 2.0f;

					// Draw all three axes with labels
					DrawDebugLine(World, Origin, Origin + XAxis * DebugAxisLen, FColor::Red, bPersistent, DebugLife, 0, 3.0f);
					DrawDebugString(World, Origin + XAxis * DebugAxisLen, TEXT("X(Twist)"), nullptr, FColor::Red, DebugLife, false, 1.0f);

					DrawDebugLine(World, Origin, Origin + YAxis * DebugAxisLen, FColor::Green, bPersistent, DebugLife, 0, 3.0f);
					DrawDebugString(World, Origin + YAxis * DebugAxisLen, TEXT("Y(Swing2)"), nullptr, FColor::Green, DebugLife, false, 1.0f);

					DrawDebugLine(World, Origin, Origin + ZAxis * DebugAxisLen, FColor::Blue, bPersistent, DebugLife, 0, 3.0f);
					DrawDebugString(World, Origin + ZAxis * DebugAxisLen, TEXT("Z(Swing1)"), nullptr, FColor::Blue, DebugLife, false, 1.0f);

					// Also highlight the axis being used for THIS joint in magenta
					// CORRECT MAPPING: Twist=X, Swing1=Z, Swing2=Y
					FVector ActiveAxis;
					FString AxisName;
					switch (Joint.ControlledAxis)
					{
						case EConstraintAxis::Twist:
							ActiveAxis = XAxis;
							AxisName = TEXT("Twist(X)");
							break;
						case EConstraintAxis::Swing1:
							ActiveAxis = ZAxis;
							AxisName = TEXT("Swing1(Z)");
							break;
						case EConstraintAxis::Swing2:
							ActiveAxis = YAxis;
							AxisName = TEXT("Swing2(Y)");
							break;
					}

					// Apply inversion if configured
					if (Joint.bInvertAxisForIK)
					{
						ActiveAxis = -ActiveAxis;
						AxisName += TEXT(" [INV]");
					}

					DrawDebugLine(World, Origin, Origin + ActiveAxis * DebugAxisLen * 1.2f, FColor::Magenta, bPersistent, DebugLife, 0, 5.0f);
					DrawDebugString(World, Origin + ActiveAxis * DebugAxisLen * 1.2f, AxisName, nullptr, FColor::Magenta, DebugLife, true, 1.2f);
				}
			}

			// Draw a small label with joint name
			DrawDebugString(World, Origin, Joint.BoneName.ToString(), nullptr, FColor::White, DebugLife, true, 0.8f);
		}
	}

	// Draw end-effector
	if (EndEffectorBoneName != NAME_None)
	{
		DrawDebugSphere(World, EndEffectorState.Position, 2.0f, 8, FColor::Yellow, bPersistent, DebugLife, 0, 2.0f);

		// Draw end-effector frame
		FTransform EndEffectorTransform = SkeletalMeshComponent->GetSocketTransform(EndEffectorBoneName, RTS_World);
		FVector	   Origin = EndEffectorTransform.GetLocation();
		float	   AxisLength = 15.0f;
		DrawDebugLine(World, Origin, Origin + EndEffectorTransform.GetUnitAxis(EAxis::X) * AxisLength, FColor::Red, bPersistent, DebugLife, 0, 2.0f);
		DrawDebugLine(World, Origin, Origin + EndEffectorTransform.GetUnitAxis(EAxis::Y) * AxisLength, FColor::Green, bPersistent, DebugLife, 0, 2.0f);
		DrawDebugLine(World, Origin, Origin + EndEffectorTransform.GetUnitAxis(EAxis::Z) * AxisLength, FColor::Blue, bPersistent, DebugLife, 0, 2.0f);
	}

	// Draw IK target when in end effector control mode
	if (ArmControlMode == EArmControlMode::EndEffectorControl)
	{
		FVector TargetPos = TargetEndEffectorTransform.GetLocation();
		FQuat	TargetRot = TargetEndEffectorTransform.GetRotation();

		// Draw target position as cyan sphere
		DrawDebugSphere(World, TargetPos, 2.0f, 12, FColor::Cyan, bPersistent, DebugLife, 0, 3.0f);

		// Draw target orientation frame
		float TargetAxisLength = 20.0f;
		DrawDebugLine(World, TargetPos, TargetPos + TargetRot.GetAxisX() * TargetAxisLength, FColor::Red, bPersistent, DebugLife, 0, 3.0f);
		DrawDebugLine(World, TargetPos, TargetPos + TargetRot.GetAxisY() * TargetAxisLength, FColor::Green, bPersistent, DebugLife, 0, 3.0f);
		DrawDebugLine(World, TargetPos, TargetPos + TargetRot.GetAxisZ() * TargetAxisLength, FColor::Blue, bPersistent, DebugLife, 0, 3.0f);

		// Draw line from current to target end effector
		if (EndEffectorBoneName != NAME_None)
		{
			DrawDebugLine(World, EndEffectorState.Position, TargetPos, FColor::Cyan, bPersistent, DebugLife, 0, 1.0f);
		}

		// Draw label
		DrawDebugString(World, TargetPos + FVector(0, 0, 15), TEXT("IK Target"), nullptr, FColor::Cyan, DebugLife, true, 1.0f);

		// ===== DEBUG: Draw FK skeleton to verify FK computation =====
		if (Joints.Num() > 0)
		{
			// Use cached IK data for debug drawing to ensure exact consistency
			// with the FK model used by the IK solver
			bool bUseCachedData = (CachedJointAxesLocal.Num() == Joints.Num()
				&& CachedJointLocalTransforms.Num() == Joints.Num());

			TArray<FTransform> JointLocalTransforms;
			TArray<FVector>	   JointAxesLocal;
			TArray<float>	   CurrentAngles;

			// Get base transform (runtime)
			FTransform BaseTransform = FTransform::Identity;
			if (Joints[0].BoneName != NAME_None)
			{
				FName ParentBoneName = SkeletalMeshComponent->GetParentBone(Joints[0].BoneName);
				if (ParentBoneName != NAME_None)
				{
					BaseTransform = SkeletalMeshComponent->GetSocketTransform(ParentBoneName, RTS_World);
				}
				else
				{
					BaseTransform = SkeletalMeshComponent->GetComponentTransform();
				}
			}

			if (bUseCachedData)
			{
				JointLocalTransforms = CachedJointLocalTransforms;
				JointAxesLocal = CachedJointAxesLocal;
				for (const FRevoluteJointConfig& Joint : Joints)
				{
					CurrentAngles.Add(Joint.CurrentAngle);
				}
			}
			else
			{
				// Fallback: compute from scratch (before IK has run)
				for (int32 JointIndex = 0; JointIndex < Joints.Num(); JointIndex++)
				{
					const FRevoluteJointConfig& Joint = Joints[JointIndex];
					if (Joint.BoneName == NAME_None)
						continue;

					int32 BoneIndex = SkeletalMeshComponent->GetBoneIndex(Joint.BoneName);
					if (BoneIndex == INDEX_NONE)
						continue;

					const FReferenceSkeleton& RefSkeleton = SkeletalMeshComponent->GetSkeletalMeshAsset()->GetRefSkeleton();
					JointLocalTransforms.Add(RefSkeleton.GetRefBonePose()[BoneIndex]);
					JointAxesLocal.Add(FVector::XAxisVector);
					CurrentAngles.Add(Joint.CurrentAngle);
				}
			}

			// Get end effector offset
			FTransform EndEffectorOffset = FTransform::Identity;
			if (bUseCachedData)
			{
				EndEffectorOffset = CachedEndEffectorOffset;
			}
			else if (EndEffectorBoneName != NAME_None && Joints.Num() > 0)
			{
				const FReferenceSkeleton& RefSkeleton = SkeletalMeshComponent->GetSkeletalMeshAsset()->GetRefSkeleton();
				int32					  LastJointBoneIndex = SkeletalMeshComponent->GetBoneIndex(Joints.Last().BoneName);
				int32					  EEBoneIndex = SkeletalMeshComponent->GetBoneIndex(EndEffectorBoneName);

				if (LastJointBoneIndex != INDEX_NONE && EEBoneIndex != INDEX_NONE)
				{
					auto GetRefPoseAccum = [&RefSkeleton](int32 TargetBoneIdx) -> FTransform {
						TArray<int32> Chain;
						int32		  Cur = TargetBoneIdx;
						while (Cur != INDEX_NONE)
						{
							Chain.Add(Cur);
							Cur = RefSkeleton.GetParentIndex(Cur);
						}

						FTransform Accum = FTransform::Identity;
						for (int32 k = Chain.Num() - 1; k >= 0; --k)
						{
							Accum = RefSkeleton.GetRefBonePose()[Chain[k]] * Accum; // CORRECT ORDER
						}
						return Accum;
					};

					FTransform LastJointRefPose = GetRefPoseAccum(LastJointBoneIndex);
					FTransform EERefPose = GetRefPoseAccum(EEBoneIndex);

					FVector PosDiff = EERefPose.GetTranslation() - LastJointRefPose.GetTranslation();
					FVector LocalOffset = LastJointRefPose.InverseTransformVector(PosDiff);
					EndEffectorOffset = FTransform(FQuat::Identity, LocalOffset);
				}
			}

			// Draw FK chain using RammsIKLibrary
			FVector BasePosWorld = BaseTransform.GetLocation();
			DrawDebugSphere(World, BasePosWorld, 2.0f, 8, FColor::Orange, bPersistent, DebugLife, 0, 2.0f);
			DrawDebugString(World, BasePosWorld, TEXT("FK Base"), nullptr, FColor::Orange, DebugLife, true, 0.8f);

			// FVector PrevPosWorld = BasePosWorld;

			// Build FK incrementally to draw each joint
			FTransform CurrentTransform = BaseTransform;
			FVector	   PrevPosWorld = BaseTransform.GetLocation();

			for (int32 i = 0; i < JointAxesLocal.Num(); i++)
			{
				// 1. Apply joint local transform (translation + rotation from reference pose)
				//    This must match the FK logic in FABRIK!
				FTransform JointFrame = JointLocalTransforms[i] * CurrentTransform;

				FVector JointPivotWorld = JointFrame.GetLocation();

				// Draw link from previous position to this joint pivot
				DrawDebugLine(World, PrevPosWorld, JointPivotWorld, FColor::Orange, bPersistent, DebugLife, 0, 3.0f);

				// Draw joint pivot sphere
				DrawDebugSphere(World, JointPivotWorld, 2.0f, 8, FColor::Orange, bPersistent, DebugLife, 0, 2.0f);
				DrawDebugString(World, JointPivotWorld, FString::Printf(TEXT("J%d"), i), nullptr, FColor::Orange, DebugLife, true, 0.8f);

				// 2. Rotate at this joint pivot
				//    JointAxesLocal[i] is in parent frame, so transform via CurrentTransform (parent)
				FVector WorldAxis = CurrentTransform.TransformVectorNoScale(JointAxesLocal[i]).GetSafeNormal();
				float	AngleRad = FMath::DegreesToRadians(CurrentAngles[i]);
				FQuat	JointRotation(WorldAxis, AngleRad);
				JointFrame.SetRotation(JointRotation * JointFrame.GetRotation());

				// Update for next iteration
				CurrentTransform = JointFrame;

				// Draw joint axis after rotation
				DrawDebugLine(World, JointPivotWorld, JointPivotWorld + WorldAxis * 15.0f, FColor::Purple, bPersistent, DebugLife, 0, 4.0f);

				PrevPosWorld = JointPivotWorld;
			}

			// Draw FK end effector
			FTransform FKEndEffectorWorld = URammsIKLibrary::ComputeForwardKinematics(
				BaseTransform, CurrentAngles, JointLocalTransforms, JointAxesLocal, EndEffectorOffset, false);

			DrawDebugLine(World, PrevPosWorld, FKEndEffectorWorld.GetLocation(), FColor::Orange, bPersistent, DebugLife, 0, 3.0f);
			DrawDebugSphere(World, FKEndEffectorWorld.GetLocation(), 2.0f, 12, FColor::Orange, bPersistent, DebugLife, 0, 3.0f);
			DrawDebugString(World, FKEndEffectorWorld.GetLocation(), TEXT("FK EE"), nullptr, FColor::Orange, DebugLife, true, 1.0f);
		}
	}

	// On-screen debug text
	if (bEnableDebugDisplay)
	{
		FString DebugText = FString::Printf(TEXT("Kinova Gen3 Arm\nArm Mode: %s\nControl Mode: %s\nJoints: %d\n"),
			ArmControlMode == EArmControlMode::JointControl ? TEXT("Joint Control") : TEXT("IK Control"),
			ControlMode == EJointControlMode::PositionControl ? TEXT("Position") : ControlMode == EJointControlMode::VelocityControl ? TEXT("Velocity")
																																	 : TEXT("Torque"),
			Joints.Num());

		if (ArmControlMode == EArmControlMode::EndEffectorControl)
		{
			if (TargetActor && TargetActor->IsValidLowLevel())
			{
				DebugText += FString::Printf(TEXT("IK Target: Tracking '%s'\n"), *TargetActor->GetName());
			}
			else
			{
				DebugText += TEXT("IK Target: Manual\n");
			}
		}

		for (int32 i = 0; i < Joints.Num(); ++i)
		{
			const FRevoluteJointConfig& Joint = Joints[i];
			DebugText += FString::Printf(TEXT("J%d (%s): %.1f° → %.1f°\n"),
				i, *Joint.BoneName.ToString(), Joint.CurrentAngle, Joint.TargetAngle);
		}

		DebugText += FString::Printf(TEXT("\nEnd Effector: (%.1f, %.1f, %.1f)"),
			EndEffectorState.Position.X, EndEffectorState.Position.Y, EndEffectorState.Position.Z);

		// Add IK status if in IK mode
		if (ArmControlMode == EArmControlMode::EndEffectorControl)
		{
			FVector TargetPos = TargetEndEffectorTransform.GetLocation();

			DebugText += FString::Printf(TEXT("\n\n== IK Status =="));
			DebugText += FString::Printf(TEXT("\nFinal Status: %s"), bLastIKSuccess ? TEXT("SUCCESS") : TEXT("FAILED"));
			DebugText += FString::Printf(TEXT("\n[IK Solver Prediction]"));
			DebugText += FString::Printf(TEXT("\n  Solver Says: %s"), bLastIKSolverSuccess ? TEXT("SUCCESS") : TEXT("FAILED"));
			DebugText += FString::Printf(TEXT("\n  FK Error: %.2f cm | %.2f°"), LastIKPositionError, LastIKRotationError);
			DebugText += FString::Printf(TEXT("\n  Iterations: %d"), LastIKIterations);
			DebugText += FString::Printf(TEXT("\n[Actual Skeletal Mesh]"));
			DebugText += FString::Printf(TEXT("\n  Actual Error: %.2f cm | %.2f°"), LastActualPosError, LastActualRotError);
			DebugText += FString::Printf(TEXT("\n  Tolerances: %.2f cm | %.2f°"), IKPositionTolerance, IKRotationTolerance);
			DebugText += FString::Printf(TEXT("\n[FK Model Validation]"));
			DebugText += FString::Printf(TEXT("\n  FK vs Actual: %.2f cm ⚠"), LastFKvsActualError);
			DebugText += FString::Printf(TEXT("\n  FK EE: (%.1f, %.1f, %.1f)"), LastFKEndEffectorPos.X, LastFKEndEffectorPos.Y, LastFKEndEffectorPos.Z);
			DebugText += FString::Printf(TEXT("\n  Actual EE: (%.1f, %.1f, %.1f)"), LastActualEndEffectorPos.X, LastActualEndEffectorPos.Y, LastActualEndEffectorPos.Z);
			DebugText += FString::Printf(TEXT("\n  Target: (%.1f, %.1f, %.1f)"), TargetPos.X, TargetPos.Y, TargetPos.Z);
		}

		GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Cyan, DebugText);
	}
}

// ========== Blueprint API ==========

void UKinovaGen3ControllerComponent::SetJointTarget(int32 JointIndex, float TargetAngle)
{
	if (Joints.IsValidIndex(JointIndex))
	{
		Joints[JointIndex].TargetAngle = TargetAngle;

		if (bEnableDebugLogging)
		{
			UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Set joint %d (%s) target to %.2f degrees"),
				JointIndex, *Joints[JointIndex].BoneName.ToString(), TargetAngle);
		}
	}
}

void UKinovaGen3ControllerComponent::SetAllJointTargets(const TArray<float>& TargetAngles)
{
	int32 NumToSet = FMath::Min(TargetAngles.Num(), Joints.Num());

	for (int32 i = 0; i < NumToSet; ++i)
	{
		Joints[i].TargetAngle = TargetAngles[i];
	}

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Set %d joint targets"), NumToSet);
	}
}

void UKinovaGen3ControllerComponent::SetJointSpeedMultiplier(int32 JointIndex, float SpeedMultiplier)
{
	if (Joints.IsValidIndex(JointIndex))
	{
		Joints[JointIndex].SpeedMultiplier = FMath::Clamp(SpeedMultiplier, 0.0f, 1.0f);
	}
}

void UKinovaGen3ControllerComponent::SetJointMaxSpeed(int32 JointIndex, float MaxSpeed)
{
	if (Joints.IsValidIndex(JointIndex))
	{
		Joints[JointIndex].MaxAngularSpeed = FMath::Max(0.1f, MaxSpeed);
	}
}

float UKinovaGen3ControllerComponent::GetJointAngle(int32 JointIndex) const
{
	if (Joints.IsValidIndex(JointIndex))
	{
		return Joints[JointIndex].CurrentAngle;
	}
	return 0.0f;
}

TArray<float> UKinovaGen3ControllerComponent::GetAllJointAngles() const
{
	TArray<float> Angles;
	Angles.Reserve(Joints.Num());

	for (const FRevoluteJointConfig& Joint : Joints)
	{
		Angles.Add(Joint.CurrentAngle);
	}

	return Angles;
}

void UKinovaGen3ControllerComponent::SetEndEffectorTarget(const FTransform& TargetTransform)
{
	TargetEndEffectorTransform = TargetTransform;
	bIKTargetInitialized = false;
	bIKTargetSatisfied = false;
}

void UKinovaGen3ControllerComponent::SetEndEffectorTargetPosition(const FVector& TargetPosition)
{
	TargetEndEffectorTransform.SetLocation(TargetPosition);
	bIKTargetInitialized = false;
	bIKTargetSatisfied = false;
}

void UKinovaGen3ControllerComponent::SetEndEffectorTargetRotation(const FRotator& TargetRotation)
{
	TargetEndEffectorTransform.SetRotation(TargetRotation.Quaternion());
	bIKTargetInitialized = false;
	bIKTargetSatisfied = false;
}

void UKinovaGen3ControllerComponent::SetEndEffectorTargetRelativeToBase(const FTransform& RelativeTransform)
{
	if (!SkeletalMeshComponent)
	{
		UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] Cannot set target: No skeletal mesh component"));
		return;
	}

	// Convert from component-relative to world space
	FTransform BaseTransform = SkeletalMeshComponent->GetComponentTransform();
	TargetEndEffectorTransform = RelativeTransform * BaseTransform;
	bIKTargetInitialized = false;
	bIKTargetSatisfied = false;

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Set IK target relative to base: World Pos=(%.1f, %.1f, %.1f)"),
			TargetEndEffectorTransform.GetLocation().X,
			TargetEndEffectorTransform.GetLocation().Y,
			TargetEndEffectorTransform.GetLocation().Z);
	}
}

void UKinovaGen3ControllerComponent::SetEndEffectorTargetPositionRelativeToBase(const FVector& RelativePosition)
{
	if (!SkeletalMeshComponent)
	{
		UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] Cannot set target: No skeletal mesh component"));
		return;
	}

	// Convert position from component-relative to world space, preserve current rotation
	FTransform BaseTransform = SkeletalMeshComponent->GetComponentTransform();
	FVector	   WorldPosition = BaseTransform.TransformPosition(RelativePosition);
	TargetEndEffectorTransform.SetLocation(WorldPosition);
	bIKTargetInitialized = false;
	bIKTargetSatisfied = false;

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Set IK target position relative to base: World Pos=(%.1f, %.1f, %.1f)"),
			WorldPosition.X, WorldPosition.Y, WorldPosition.Z);
	}
}

void UKinovaGen3ControllerComponent::SetEndEffectorTargetRelativeToActor(const FTransform& RelativeTransform)
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] Cannot set target: No owner actor"));
		return;
	}

	// Convert from actor-relative to world space
	FTransform ActorTransform = Owner->GetActorTransform();
	TargetEndEffectorTransform = RelativeTransform * ActorTransform;
	bIKTargetInitialized = false;
	bIKTargetSatisfied = false;

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Set IK target relative to actor: World Pos=(%.1f, %.1f, %.1f)"),
			TargetEndEffectorTransform.GetLocation().X,
			TargetEndEffectorTransform.GetLocation().Y,
			TargetEndEffectorTransform.GetLocation().Z);
	}
}

void UKinovaGen3ControllerComponent::SetEndEffectorTargetPositionRelativeToActor(const FVector& RelativePosition)
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] Cannot set target: No owner actor"));
		return;
	}

	// Convert position from actor-relative to world space, preserve current rotation
	FTransform ActorTransform = Owner->GetActorTransform();
	FVector	   WorldPosition = ActorTransform.TransformPosition(RelativePosition);
	TargetEndEffectorTransform.SetLocation(WorldPosition);
	bIKTargetInitialized = false;
	bIKTargetSatisfied = false;

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Set IK target position relative to actor: World Pos=(%.1f, %.1f, %.1f)"),
			WorldPosition.X, WorldPosition.Y, WorldPosition.Z);
	}
}

void UKinovaGen3ControllerComponent::MoveEndEffectorTargetBy(const FVector& Offset)
{
	// Move target by world-space offset
	FVector CurrentPos = TargetEndEffectorTransform.GetLocation();
	TargetEndEffectorTransform.SetLocation(CurrentPos + Offset);
	bIKTargetInitialized = false;
	bIKTargetSatisfied = false;

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Moved IK target by (%.1f, %.1f, %.1f) to (%.1f, %.1f, %.1f)"),
			Offset.X, Offset.Y, Offset.Z,
			TargetEndEffectorTransform.GetLocation().X,
			TargetEndEffectorTransform.GetLocation().Y,
			TargetEndEffectorTransform.GetLocation().Z);
	}
}

void UKinovaGen3ControllerComponent::MoveEndEffectorTargetByLocal(const FVector& LocalOffset)
{
	// Move target by offset in end effector's current orientation
	FQuat	CurrentRotation = TargetEndEffectorTransform.GetRotation();
	FVector WorldOffset = CurrentRotation.RotateVector(LocalOffset);

	FVector CurrentPos = TargetEndEffectorTransform.GetLocation();
	TargetEndEffectorTransform.SetLocation(CurrentPos + WorldOffset);
	bIKTargetInitialized = false;
	bIKTargetSatisfied = false;

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Verbose, TEXT("[KinovaGen3] Moved IK target by local (%.1f, %.1f, %.1f) to world (%.1f, %.1f, %.1f)"),
			LocalOffset.X, LocalOffset.Y, LocalOffset.Z,
			TargetEndEffectorTransform.GetLocation().X,
			TargetEndEffectorTransform.GetLocation().Y,
			TargetEndEffectorTransform.GetLocation().Z);
	}
}

void UKinovaGen3ControllerComponent::UpdateInverseKinematics(float DeltaTime)
{
	if (!SkeletalMeshComponent || Joints.Num() == 0)
		return;

	if (bEnableDebugLogging && GFrameCounter % 60 == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Gen3] *** UpdateInverseKinematics CALLED ***"));
	}

	// Debug: Log joint order once
	static bool bLoggedJointOrder = false;
	if (bEnableDebugLogging && !bLoggedJointOrder)
	{
		UE_LOG(LogTemp, Verbose, TEXT("[Gen3] Joint order (should be base->EE):"));
		for (int32 i = 0; i < Joints.Num(); i++)
		{
			UE_LOG(LogTemp, Verbose, TEXT("  [%d]: %s"), i, *Joints[i].BoneName.ToString());
		}
		bLoggedJointOrder = true;
	}

	// ============================================================================
	// STEP 1: Extract Robot Geometry from Reference Skeleton
	// ============================================================================
	// We extract the robot's link lengths and joint axes from the reference skeleton
	// (bind pose) in a NEUTRAL coordinate frame (identity base).
	// This gives us the robot's pure geometry independent of its world position.

	const FReferenceSkeleton& RefSkeleton = SkeletalMeshComponent->GetSkeletalMeshAsset()->GetRefSkeleton();

	// Helper: Get accumulated reference pose transform from root, starting with identity
	auto GetRefTransform = [&RefSkeleton](int32 BoneIdx) -> FTransform {
		TArray<int32> Chain;
		for (int32 Cur = BoneIdx; Cur != INDEX_NONE; Cur = RefSkeleton.GetParentIndex(Cur))
		{
			Chain.Add(Cur);
		}

		FTransform T = FTransform::Identity; // Neutral base
		for (int32 k = Chain.Num() - 1; k >= 0; --k)
		{
			T = RefSkeleton.GetRefBonePose()[Chain[k]] * T; // Parent-to-world order
		}
		return T;
	};

	// Get reference transforms for all joints
	TArray<FTransform> JointRefTransforms;
	JointRefTransforms.SetNum(Joints.Num());
	for (int32 i = 0; i < Joints.Num(); i++)
	{
		int32 BoneIdx = SkeletalMeshComponent->GetBoneIndex(Joints[i].BoneName);
		if (BoneIdx == INDEX_NONE)
			return;
		JointRefTransforms[i] = GetRefTransform(BoneIdx);
	}

	// Get base reference transform (parent of first joint)
	FTransform BaseRefTransform = FTransform::Identity;
	FName	   FirstJointParent = NAME_None;
	if (Joints.Num() > 0 && Joints[0].BoneName != NAME_None)
	{
		FirstJointParent = SkeletalMeshComponent->GetParentBone(Joints[0].BoneName);
		if (FirstJointParent != NAME_None)
		{
			int32 ParentIdx = SkeletalMeshComponent->GetBoneIndex(FirstJointParent);
			if (ParentIdx != INDEX_NONE)
			{
				BaseRefTransform = GetRefTransform(ParentIdx);

				if (bEnableDebugLogging)
				{
					FVector	 BaseLoc = BaseRefTransform.GetLocation();
					FRotator BaseRot = BaseRefTransform.Rotator();
					UE_LOG(LogTemp, Verbose, TEXT("[Gen3] BaseRef '%s': Loc=(%.1f, %.1f, %.1f) Rot=(P:%.1f, Y:%.1f, R:%.1f)"),
						*FirstJointParent.ToString(), BaseLoc.X, BaseLoc.Y, BaseLoc.Z,
						BaseRot.Pitch, BaseRot.Yaw, BaseRot.Roll);
				}
			}
		}
	}

	// ============================================================================
	// STEP 2 & 3: Compute Joint Local Transforms and Axes from Constraint Frames
	// ============================================================================
	// Both local transforms and rotation axes are derived from the SAME source
	// (the physics constraint Frame1/Frame2) to ensure consistency.
	//
	// After the first tick, local transforms are calibrated from the actual
	// physics body positions and cached — skip recomputation.

	TArray<FTransform> JointLocalTransforms;
	JointLocalTransforms.SetNum(Joints.Num());

	if (bFKLocalTransformsCalibrated)
	{
		// Use calibrated transforms from first tick
		JointLocalTransforms = CachedJointLocalTransforms;
	}
	else
	{
		CachedJointAxesLocal.SetNum(Joints.Num());

		for (int32 i = 0; i < Joints.Num(); i++)
		{
			int32	   BoneIdx = SkeletalMeshComponent->GetBoneIndex(Joints[i].BoneName);
			FTransform BoneRef = RefSkeleton.GetRefBonePose()[BoneIdx];

			FName				 ConstraintToUse = Joints[i].ConstraintName != NAME_None ? Joints[i].ConstraintName : Joints[i].BoneName;
			FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintToUse);

			if (!Constraint)
			{
				JointLocalTransforms[i] = BoneRef;
				CachedJointAxesLocal[i] = FVector::XAxisVector;
				continue;
			}

			FTransform Frame1 = Constraint->GetRefFrame(EConstraintFrame::Frame1);
			FTransform Frame2 = Constraint->GetRefFrame(EConstraintFrame::Frame2);

			// Constraint-derived rest transform: use constraint rotation (consistent with
			// Frame1 axis) but bone reference position (accurate skeleton geometry).
			// Frame2^{-1} * Frame1 gives the correct rotation but its position can be
			// offset if Frame2 has non-zero translation on the child body.
			FTransform ConstraintRest = Frame2.Inverse() * Frame1;
			JointLocalTransforms[i] = FTransform(ConstraintRest.GetRotation(), BoneRef.GetLocation());

			// Axis directly from Frame1 (already in parent body's local frame)
			FVector AxisLocal;
			switch (Joints[i].ControlledAxis)
			{
				case EConstraintAxis::Swing1:
					AxisLocal = Frame1.GetUnitAxis(EAxis::Z);
					break;
				case EConstraintAxis::Swing2:
					AxisLocal = Frame1.GetUnitAxis(EAxis::Y);
					break;
				case EConstraintAxis::Twist:
				default:
					AxisLocal = Frame1.GetUnitAxis(EAxis::X);
					break;
			}

			if (Joints[i].bInvertAxisForIK)
			{
				AxisLocal = -AxisLocal;
			}

			CachedJointAxesLocal[i] = AxisLocal;

			if (bEnableDebugLogging)
			{
				// Log the initial constraint vs bone ref comparison
				FQuat ConstraintRot = JointLocalTransforms[i].GetRotation();
				FQuat BoneRefRot = BoneRef.GetRotation();
				float RotDiff = FMath::RadiansToDegrees((ConstraintRot * BoneRefRot.Inverse()).GetAngle());
				float ConstraintPosDiff = FVector::Dist(ConstraintRest.GetLocation(), BoneRef.GetLocation());

				UE_LOG(LogTemp, Warning, TEXT("[Gen3] J%d Constraint vs BoneRef: RotDiff=%.2f° ConstraintPosDiff=%.3fcm BoneRefLoc=(%.2f,%.2f,%.2f)"),
					i, RotDiff, ConstraintPosDiff, BoneRef.GetLocation().X, BoneRef.GetLocation().Y, BoneRef.GetLocation().Z);
			}
		} // end for loop in STEP 2+3
	}	  // end else (!bFKLocalTransformsCalibrated)

	TArray<FVector> JointAxesLocal = CachedJointAxesLocal;

	// Axes are now pre-computed at initialization and cached!
	// Just log them if debug is enabled
	if (bEnableDebugLogging)
	{
		for (int32 i = 0; i < FMath::Min(7, JointAxesLocal.Num()); i++)
		{
			UE_LOG(LogTemp, Verbose, TEXT("[Gen3] J%d AxisJoint (cached): (%.3f, %.3f, %.3f)"),
				i, JointAxesLocal[i].X, JointAxesLocal[i].Y, JointAxesLocal[i].Z);
		}
	}

	// ============================================================================
	// STEP 4: Compute End Effector Offset
	// ============================================================================

	FTransform EndEffectorOffset = FTransform::Identity;
	if (EndEffectorBoneName != NAME_None)
	{
		int32 EEBoneIdx = SkeletalMeshComponent->GetBoneIndex(EndEffectorBoneName);

		if (bEnableDebugLogging)
		{
			UE_LOG(LogTemp, Verbose, TEXT("[Gen3] EE '%s': IsBone=%d"), *EndEffectorBoneName.ToString(), EEBoneIdx != INDEX_NONE);
		}

		if (EEBoneIdx != INDEX_NONE)
		{
			// It's a bone - compute offset from last joint to EE in reference skeleton
			FTransform LastJointRef = JointRefTransforms.Last();
			FTransform EERefWorld = GetRefTransform(EEBoneIdx);
			EndEffectorOffset = EERefWorld.GetRelativeTransform(LastJointRef);

			if (bEnableDebugLogging)
			{
				UE_LOG(LogTemp, Verbose, TEXT("[Gen3]   LastJoint ref: Loc=(%.1f, %.1f, %.1f)"),
					LastJointRef.GetLocation().X, LastJointRef.GetLocation().Y, LastJointRef.GetLocation().Z);
				UE_LOG(LogTemp, Verbose, TEXT("[Gen3]   EE ref: Loc=(%.1f, %.1f, %.1f)"),
					EERefWorld.GetLocation().X, EERefWorld.GetLocation().Y, EERefWorld.GetLocation().Z);
				UE_LOG(LogTemp, Verbose, TEXT("[Gen3]   EE Offset: Loc=(%.1f, %.1f, %.1f) Rot=(%.1f, %.1f, %.1f)"),
					EndEffectorOffset.GetLocation().X, EndEffectorOffset.GetLocation().Y, EndEffectorOffset.GetLocation().Z,
					EndEffectorOffset.Rotator().Pitch, EndEffectorOffset.Rotator().Yaw, EndEffectorOffset.Rotator().Roll);
			}
		}
		else
		{
			// It's a socket - use socket's local transform relative to LAST JOINT
			USkeletalMeshSocket const* Socket = SkeletalMeshComponent->GetSkeletalMeshAsset()->FindSocket(EndEffectorBoneName);
			if (Socket)
			{
				FName LastJointBoneName = Joints.Last().BoneName;

				// Check if socket parent matches last joint
				if (Socket->BoneName == LastJointBoneName)
				{
					// Socket attached to last joint - build transform the same way Unreal does for sockets
					// Sockets apply: Rotation first, then Translation in the LOCAL (rotated) space
					FQuat	SocketQuat = FQuat(Socket->RelativeRotation);
					FVector SocketScale = Socket->RelativeScale;

					// Create transform: rotation + scale, then translation
					EndEffectorOffset = FTransform(SocketQuat, FVector::ZeroVector, SocketScale);
					EndEffectorOffset.SetTranslation(Socket->RelativeLocation);

					if (bEnableDebugLogging)
					{
						UE_LOG(LogTemp, Log, TEXT("[Gen3]   Socket parent matches last joint: '%s'"), *Socket->BoneName.ToString());
						UE_LOG(LogTemp, Log, TEXT("[Gen3]   Socket RelativeLocation: (%.2f, %.2f, %.2f)"),
							Socket->RelativeLocation.X, Socket->RelativeLocation.Y, Socket->RelativeLocation.Z);
						UE_LOG(LogTemp, Log, TEXT("[Gen3]   Socket RelativeRotation: (%.2f, %.2f, %.2f)"),
							Socket->RelativeRotation.Pitch, Socket->RelativeRotation.Yaw, Socket->RelativeRotation.Roll);

						// Test: compare FK-computed EE position vs actual socket position (pre-calibration)
						// Note: uses previous tick's calibrated transforms; post-calibration accuracy shown in FK Model Validation below
						FTransform TestBaseTransform = SkeletalMeshComponent->GetComponentTransform();
						if (FirstJointParent != NAME_None)
						{
							TestBaseTransform = SkeletalMeshComponent->GetSocketTransform(FirstJointParent, RTS_World);
						}
						TArray<float> TestAngles;
						TestAngles.SetNum(Joints.Num());
						for (int32 ti = 0; ti < Joints.Num(); ti++)
						{
							TestAngles[ti] = Joints[ti].CurrentAngle;
						}
						FTransform FKLastJointTest = URammsIKLibrary::ComputeForwardKinematics(
							TestBaseTransform, TestAngles, JointLocalTransforms, JointAxesLocal, FTransform::Identity, false);
						FTransform FKSocketTest = EndEffectorOffset * FKLastJointTest;
						FTransform ActualSocketTest = SkeletalMeshComponent->GetSocketTransform(EndEffectorBoneName, RTS_World);
						float	   TestError = FVector::Dist(FKSocketTest.GetLocation(), ActualSocketTest.GetLocation());
						UE_LOG(LogTemp, Log, TEXT("[Gen3]   Test FK socket (pre-calib): (%.2f, %.2f, %.2f)"),
							FKSocketTest.GetLocation().X, FKSocketTest.GetLocation().Y, FKSocketTest.GetLocation().Z);
						UE_LOG(LogTemp, Log, TEXT("[Gen3]   Test Actual socket: (%.2f, %.2f, %.2f)"),
							ActualSocketTest.GetLocation().X, ActualSocketTest.GetLocation().Y, ActualSocketTest.GetLocation().Z);
						UE_LOG(LogTemp, Log, TEXT("[Gen3]   Test error: %.2f cm"), TestError);
					}
				}
				else
				{
					// Socket attached to different bone - need to compute offset from last joint to socket in REFERENCE POSE
					// RefSkeleton already declared earlier in the function

					// Get reference pose transform for socket's parent bone
					int32	   SocketParentBoneIdx = SkeletalMeshComponent->GetBoneIndex(Socket->BoneName);
					FTransform SocketParentRefWorld = GetRefTransform(SocketParentBoneIdx);

					// Apply socket's relative transform to get socket's reference world position
					FQuat	   SocketRelativeQuat = FQuat(Socket->RelativeRotation);
					FTransform SocketRelativeTransform(SocketRelativeQuat, Socket->RelativeLocation, Socket->RelativeScale);
					FTransform SocketRefWorld = SocketRelativeTransform * SocketParentRefWorld;

					// Get last joint's reference pose
					FTransform LastJointRef = JointRefTransforms.Last();

					// Compute offset from last joint to socket in reference pose
					EndEffectorOffset = SocketRefWorld.GetRelativeTransform(LastJointRef);

					if (bEnableDebugLogging)
					{
						UE_LOG(LogTemp, Warning, TEXT("[Gen3]   Socket parent '%s' (idx %d) != last joint '%s' - computing offset from reference pose"),
							*Socket->BoneName.ToString(), SocketParentBoneIdx, *LastJointBoneName.ToString());
						UE_LOG(LogTemp, Log, TEXT("[Gen3]   Socket parent ref world: Loc=(%.2f, %.2f, %.2f)"),
							SocketParentRefWorld.GetLocation().X, SocketParentRefWorld.GetLocation().Y, SocketParentRefWorld.GetLocation().Z);
						UE_LOG(LogTemp, Log, TEXT("[Gen3]   Socket ref world: Loc=(%.2f, %.2f, %.2f)"),
							SocketRefWorld.GetLocation().X, SocketRefWorld.GetLocation().Y, SocketRefWorld.GetLocation().Z);
						UE_LOG(LogTemp, Log, TEXT("[Gen3]   Last joint ref: Loc=(%.2f, %.2f, %.2f)"),
							LastJointRef.GetLocation().X, LastJointRef.GetLocation().Y, LastJointRef.GetLocation().Z);
						UE_LOG(LogTemp, Log, TEXT("[Gen3]   Computed offset: Loc=(%.2f, %.2f, %.2f) Rot=(%.2f, %.2f, %.2f)"),
							EndEffectorOffset.GetLocation().X, EndEffectorOffset.GetLocation().Y, EndEffectorOffset.GetLocation().Z,
							EndEffectorOffset.Rotator().Pitch, EndEffectorOffset.Rotator().Yaw, EndEffectorOffset.Rotator().Roll);
					}
				}
			}
		}

		if (bEnableDebugLogging)
		{
			FVector	 EEOffset = EndEffectorOffset.GetLocation();
			FRotator EERot = EndEffectorOffset.Rotator();
			UE_LOG(LogTemp, Verbose, TEXT("[Gen3] EE offset (final): Loc=(%.1f, %.1f, %.1f) Rot=(%.1f, %.1f, %.1f)"),
				EEOffset.X, EEOffset.Y, EEOffset.Z, EERot.Pitch, EERot.Yaw, EERot.Roll);
		}
	}

	// Cache for use by diagnostic functions (local transforms may be updated by calibration below)
	CachedEndEffectorOffset = EndEffectorOffset;
	CachedJointAxesLocal = JointAxesLocal;
	// CachedJointLocalTransforms updated after calibration in STEP 5b, or from the loop above

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[Gen3] EndEffectorOffset being used: Loc=(%.2f, %.2f, %.2f) Rot=(%.2f, %.2f, %.2f)"),
			EndEffectorOffset.GetLocation().X, EndEffectorOffset.GetLocation().Y, EndEffectorOffset.GetLocation().Z,
			EndEffectorOffset.Rotator().Pitch, EndEffectorOffset.Rotator().Yaw, EndEffectorOffset.Rotator().Roll);
	}

	// ============================================================================
	// STEP 5: Get Current State
	// ============================================================================

	// Get CURRENT base transform (where robot actually is now)
	FTransform BaseTransform = SkeletalMeshComponent->GetComponentTransform();
	if (FirstJointParent != NAME_None)
	{
		BaseTransform = SkeletalMeshComponent->GetSocketTransform(FirstJointParent, RTS_World);
	}

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Verbose, TEXT("[Gen3] BaseRefTransform: Loc=(%.1f, %.1f, %.1f) Rot=(%.1f, %.1f, %.1f)"),
			BaseRefTransform.GetLocation().X, BaseRefTransform.GetLocation().Y, BaseRefTransform.GetLocation().Z,
			BaseRefTransform.Rotator().Pitch, BaseRefTransform.Rotator().Yaw, BaseRefTransform.Rotator().Roll);
		UE_LOG(LogTemp, Verbose, TEXT("[Gen3] BaseTransform (current): Loc=(%.1f, %.1f, %.1f) Rot=(%.1f, %.1f, %.1f)"),
			BaseTransform.GetLocation().X, BaseTransform.GetLocation().Y, BaseTransform.GetLocation().Z,
			BaseTransform.Rotator().Pitch, BaseTransform.Rotator().Yaw, BaseTransform.Rotator().Roll);
	}

	// ============================================================================
	// STEP 5b: Per-tick FK position calibration from runtime physics body positions
	// ============================================================================
	// The FK rotation model (axis * local * parent) doesn't perfectly match
	// the physics constraint model (Frame2^-1 * R * Frame1 * parent).
	// By recalibrating positions every tick, the FK chain matches physics at
	// the current pose, giving the IK solver a locally-accurate model.
	// Deferred by 2 ticks to ensure physics has fully initialized.
	if (FKCalibrationTickCounter < 2)
	{
		FKCalibrationTickCounter++;
	}
	else
	{
		FTransform T = BaseTransform;
		bool	   bFirstCalibration = !bFKLocalTransformsCalibrated;

		if (bFirstCalibration)
		{
			UE_LOG(LogTemp, Warning, TEXT("[Gen3] === FK CALIBRATION (initial, tick %d) ==="), FKCalibrationTickCounter);
		}

		for (int32 i = 0; i < Joints.Num(); i++)
		{
			// Get actual bone world position from physics simulation
			FVector ActualPos = SkeletalMeshComponent->GetSocketTransform(Joints[i].BoneName, RTS_World).GetLocation();

			// Compute corrected local position in parent's frame:
			//   FK step: T_after = Local * T  →  T_after.Pos = T.Rot * Local.Pos + T.Pos
			//   Want: T_after.Pos = ActualPos
			//   So:   Local.Pos = T.Rot^{-1} * (ActualPos - T.Pos)
			FVector CorrectedLocalPos = T.GetRotation().UnrotateVector(ActualPos - T.GetLocation());

			if (bFirstCalibration)
			{
				FVector OldLocalPos = JointLocalTransforms[i].GetLocation();
				FVector Delta = CorrectedLocalPos - OldLocalPos;
				UE_LOG(LogTemp, Warning, TEXT("[Gen3] J%d Calib: Old=(%.3f,%.3f,%.3f) New=(%.3f,%.3f,%.3f) Delta=(%.3f,%.3f,%.3f) Angle=%.1f"),
					i, OldLocalPos.X, OldLocalPos.Y, OldLocalPos.Z,
					CorrectedLocalPos.X, CorrectedLocalPos.Y, CorrectedLocalPos.Z,
					Delta.X, Delta.Y, Delta.Z,
					Joints[i].CurrentAngle);

				// Apply full correction on first calibration
				JointLocalTransforms[i].SetLocation(CorrectedLocalPos);
			}
			else
			{
				// Smooth subsequent calibrations with EMA to prevent jitter
				FVector CurrentPos = JointLocalTransforms[i].GetLocation();
				FVector SmoothedPos = FMath::Lerp(CurrentPos, CorrectedLocalPos, CalibrationSmoothingAlpha);
				JointLocalTransforms[i].SetLocation(SmoothedPos);
			}

			// Advance T: axis from parent frame, then local transform, then rotation
			FVector AxisWorld = T.TransformVectorNoScale(CachedJointAxesLocal[i]).GetSafeNormal();
			T = JointLocalTransforms[i] * T;

			float AngleRad = FMath::DegreesToRadians(Joints[i].CurrentAngle);
			T.SetRotation((FQuat(AxisWorld, AngleRad) * T.GetRotation()).GetNormalized());
		}

		CachedJointLocalTransforms = JointLocalTransforms;

		if (bFirstCalibration)
		{
			bFKLocalTransformsCalibrated = true;
			UE_LOG(LogTemp, Warning, TEXT("[Gen3] FK position calibration active (per-tick)"));
		}
	}

	// All solvers use the same bone-derived kinematic chain
	const TArray<FTransform>& JointLocalTransformsForSolver = JointLocalTransforms;
	const TArray<FVector>&	  JointAxesLocalForSolver = JointAxesLocal;
	const FTransform&		  EndEffectorOffsetForSolver = EndEffectorOffset;

	// Get current joint angles (in degrees)
	TArray<float>	  CurrentAngles;
	TArray<FVector2D> JointLimits;
	CurrentAngles.SetNum(Joints.Num());
	JointLimits.SetNum(Joints.Num());

	for (int32 i = 0; i < Joints.Num(); i++)
	{
		CurrentAngles[i] = Joints[i].CurrentAngle;
		JointLimits[i] = FVector2D(Joints[i].MinAngleLimit, Joints[i].MaxAngleLimit);
	}

	// NOTE: CurrentAngles are the IK TARGET angles, not the actual constraint angles
	// The actual skeletal mesh may lag behind due to PD controller dynamics
	// FK validation below uses ActualAngles read from constraints

	// ============================================================================
	// STEP 7: Solve IK Using Bone Transforms (bypasses FK coordinate frame issues)
	// ============================================================================

	// Get current bone world transforms
	TArray<FTransform> CurrentBoneTransforms;
	CurrentBoneTransforms.SetNum(Joints.Num());
	for (int32 i = 0; i < Joints.Num(); i++)
	{
		// Use bone name to get world transform
		// CurrentBoneTransforms[i] = SkeletalMeshComponent->GetSocketTransform(Joints[i].BoneName, RTS_World);

		const int32 BoneIdx = SkeletalMeshComponent->GetBoneIndex(Joints[i].BoneName);
		if (BoneIdx != INDEX_NONE)
		{
			const FTransform BoneCS = SkeletalMeshComponent->GetBoneTransform(BoneIdx);			// component space
			CurrentBoneTransforms[i] = BoneCS * SkeletalMeshComponent->GetComponentTransform(); // world
		}
		else
		{
			CurrentBoneTransforms[i] = FTransform::Identity; // keep deterministic
		}

		// // OR
		// const int32 BoneIdx = SkeletalMeshComponent->GetBoneIndex(Joints[i].BoneName);
		// FTransform BoneCS = SkeletalMeshComponent->GetBoneTransform(BoneIdx); // component space
		// CurrentBoneTransforms[i] = BoneCS * SkeletalMeshComponent->GetComponentTransform();
	}

	// Get current end effector transform
	FTransform CurrentEE = SkeletalMeshComponent->GetSocketTransform(EndEffectorBoneName, RTS_World);

	// Compute joint axes in world space from current bone orientations
	// Axes are stored in parent frame, so use parent bone's current orientation
	TArray<FVector> JointAxesWorld;
	JointAxesWorld.SetNum(Joints.Num());
	for (int32 i = 0; i < Joints.Num(); i++)
	{
		// Axis is in parent frame: use parent bone's current world orientation
		FTransform ParentBoneTransform = (i == 0) ? BaseTransform : CurrentBoneTransforms[i - 1];
		JointAxesWorld[i] = ParentBoneTransform.GetRotation().RotateVector(JointAxesLocal[i]).GetSafeNormal();

		if (JointAxesWorld[i].IsNearlyZero())
		{
			// Fallback to something stable rather than injecting zeros into the Jacobian
			JointAxesWorld[i] = FVector::XAxisVector;
		}
	}

	// Convert bool mask -> float weights expected by IK library
	const bool		bDoPos = (TaskSpaceMask.Num() > 0) ? TaskSpaceMask[0] : true;
	const bool		bDoRot = (TaskSpaceMask.Num() > 1) ? TaskSpaceMask[1] : true;
	const FVector2D TaskMaskWeights(bDoPos ? 1.0f : 0.0f, bDoRot ? 1.0f : 0.0f);

	const float StepClipDeg = FMath::RadiansToDegrees(IKStepClip);

	FIKSolveResult IKResult;

	if (IKSolverType == EIKSolverType::FABRIK)
	{
		// Use FABRIK solver with hard joint limits
		IKResult = URammsIKLibrary::SolveIK_FABRIK(
			BaseTransform,
			CurrentAngles,
			JointLocalTransformsForSolver,
			JointAxesLocalForSolver,
			JointLimits,
			EndEffectorOffsetForSolver,
			TargetEndEffectorTransform,
			TaskSpaceMask,
			FABRIKMaxIterations,
			FABRIKPositionTolerance,
			IKRotationTolerance,
			FABRIKAngleGain,
			FABRIKMaxAngleStepDeg,
			FABRIKLimitEscapeDeg,
			FABRIKOrientationIterations,
			FABRIKOrientationGain);
	}
	else if (IKSolverType == EIKSolverType::CCD)
	{
		IKResult = URammsIKLibrary::SolveIK_CCD(
			BaseTransform,
			CurrentAngles,
			JointLocalTransformsForSolver,
			JointAxesLocalForSolver,
			JointLimits,
			EndEffectorOffsetForSolver,
			TargetEndEffectorTransform,
			TaskSpaceMask,
			CCDMaxIterations,
			IKPositionTolerance,
			IKRotationTolerance,
			CCDPositionGain,
			CCDOrientationGain,
			CCDMaxAngleStepDeg);
	}
	else
	{
		// Use DLS solver (original)
		IKResult = URammsIKLibrary::SolveIK_FKChain(
			BaseTransform,
			CurrentAngles,
			JointLocalTransforms,
			JointAxesLocal,
			EndEffectorOffset,
			TargetEndEffectorTransform,
			JointLimits,
			TaskSpaceMask,
			JointWeights,
			bEnableNullSpaceOptimization,
			NullSpaceGain,
			NullSpaceBias,
			IKDampingFactor,
			StepClipDeg,
			MaxIKIterations,
			IKPositionTolerance,
			IKRotationTolerance);
	}

	for (int32 i = 0; i < FMath::Min(IKResult.JointAngles.Num(), Joints.Num()); i++)
	{
		Joints[i].TargetAngle = IKResult.JointAngles[i];
	}

	LastIKPositionError = IKResult.PositionError;
	LastIKRotationError = IKResult.RotationError;
	LastIKIterations = IKResult.IterationsUsed;
	bLastIKSolverSuccess = IKResult.bSuccess; // What the IK solver thinks

	// Validate FK model: Compare FK prediction vs actual skeletal mesh
	// NOTE: CurrentAngles comes from Joints[i].CurrentAngle which was already updated by
	// UpdatePositionControl earlier this tick using GetConstraintAngle() - Joint.AngleOffset
	// So these ARE the actual physics constraint angles, not just the IK targets.
	FTransform FK_EE = URammsIKLibrary::ComputeForwardKinematics(
		BaseTransform, CurrentAngles, JointLocalTransforms, JointAxesLocal, EndEffectorOffset, false);
	FTransform ActualEE = SkeletalMeshComponent->GetSocketTransform(EndEffectorBoneName, RTS_World);

	LastFKvsActualError = FVector::Dist(FK_EE.GetLocation(), ActualEE.GetLocation());
	LastFKEndEffectorPos = FK_EE.GetLocation();
	LastActualEndEffectorPos = ActualEE.GetLocation();

	if (bEnableDebugLogging && GFrameCounter % 60 == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Gen3] FK Model Validation:"));
		UE_LOG(LogTemp, Warning, TEXT("  BaseTransform: (%.2f, %.2f, %.2f)"), BaseTransform.GetLocation().X, BaseTransform.GetLocation().Y, BaseTransform.GetLocation().Z);
		UE_LOG(LogTemp, Warning, TEXT("  FK EE: (%.2f, %.2f, %.2f)"), FK_EE.GetLocation().X, FK_EE.GetLocation().Y, FK_EE.GetLocation().Z);
		UE_LOG(LogTemp, Warning, TEXT("  Actual EE: (%.2f, %.2f, %.2f)"), ActualEE.GetLocation().X, ActualEE.GetLocation().Y, ActualEE.GetLocation().Z);
		UE_LOG(LogTemp, Warning, TEXT("  Delta: (%.2f, %.2f, %.2f)"),
			FK_EE.GetLocation().X - ActualEE.GetLocation().X,
			FK_EE.GetLocation().Y - ActualEE.GetLocation().Y,
			FK_EE.GetLocation().Z - ActualEE.GetLocation().Z);
		UE_LOG(LogTemp, Warning, TEXT("  Error: %.2f cm"), LastFKvsActualError);

		// Check if BaseTransform matches actual first joint parent
		FTransform ActualBase = SkeletalMeshComponent->GetComponentTransform();
		if (FirstJointParent != NAME_None)
		{
			ActualBase = SkeletalMeshComponent->GetSocketTransform(FirstJointParent, RTS_World);
		}
		float BaseError = FVector::Dist(BaseTransform.GetLocation(), ActualBase.GetLocation());
		UE_LOG(LogTemp, Warning, TEXT("  BaseTransform Error: %.2f cm"), BaseError);

		// Check last joint position - is FK computing it correctly?
		FTransform LastJointActual = SkeletalMeshComponent->GetSocketTransform(Joints.Last().BoneName, RTS_World);
		FTransform FK_LastJoint = URammsIKLibrary::ComputeForwardKinematics(
			BaseTransform, CurrentAngles, JointLocalTransforms, JointAxesLocal, FTransform::Identity, false);
		float LastJointError = FVector::Dist(FK_LastJoint.GetLocation(), LastJointActual.GetLocation());

		UE_LOG(LogTemp, Warning, TEXT("  Last Joint FK: (%.2f, %.2f, %.2f)"),
			FK_LastJoint.GetLocation().X, FK_LastJoint.GetLocation().Y, FK_LastJoint.GetLocation().Z);
		UE_LOG(LogTemp, Warning, TEXT("  Last Joint Actual: (%.2f, %.2f, %.2f)"),
			LastJointActual.GetLocation().X, LastJointActual.GetLocation().Y, LastJointActual.GetLocation().Z);
		UE_LOG(LogTemp, Warning, TEXT("  Last Joint Error: %.2f cm"), LastJointError);

		// Log current vs target angles
		UE_LOG(LogTemp, Warning, TEXT("  Joint Angles (Current vs Target):"));
		for (int32 i = 0; i < FMath::Min(Joints.Num(), 3); i++)
		{
			UE_LOG(LogTemp, Warning, TEXT("    J%d: %.2f° vs %.2f° (diff: %.2f°)"),
				i, Joints[i].CurrentAngle, Joints[i].TargetAngle,
				Joints[i].CurrentAngle - Joints[i].TargetAngle);
		}

		// Per-joint FK vs Actual position comparison
		{
			// Manually step through FK to get per-joint positions
			FTransform T = BaseTransform;
			UE_LOG(LogTemp, Warning, TEXT("  Per-joint FK vs Actual:"));
			for (int32 i = 0; i < Joints.Num(); i++)
			{
				// FK step: extract axis, apply bone transform, apply rotation
				FVector AxisW = T.TransformVectorNoScale(JointAxesLocal[i]).GetSafeNormal();
				T = JointLocalTransforms[i] * T;
				FVector FKPos = T.GetLocation();
				float	AngleRad = FMath::DegreesToRadians(CurrentAngles[i]);
				FQuat	R(AxisW, AngleRad);
				T.SetRotation((R * T.GetRotation()).GetNormalized());

				// Actual position from skeleton
				FTransform ActualJointTransform = SkeletalMeshComponent->GetSocketTransform(Joints[i].BoneName, RTS_World);
				FVector	   ActualPos = ActualJointTransform.GetLocation();
				float	   JointErr = FVector::Dist(FKPos, ActualPos);

				// Also log rotation difference
				FQuat FKRot = T.GetRotation();
				FQuat ActRot = ActualJointTransform.GetRotation();
				float RotErr = FMath::RadiansToDegrees((ActRot * FKRot.Inverse()).GetAngle());

				UE_LOG(LogTemp, Warning, TEXT("    J%d: FK=(%.2f,%.2f,%.2f) Act=(%.2f,%.2f,%.2f) PosErr=%.2fcm RotErr=%.1f° Angle=%.1f°"),
					i, FKPos.X, FKPos.Y, FKPos.Z,
					ActualPos.X, ActualPos.Y, ActualPos.Z,
					JointErr, RotErr, Joints[i].CurrentAngle);
			}
		}
	}

	// Check actual convergence using physical end effector position
	// This accounts for FK/IK coordinate frame mismatches and physics constraints
	float ActualPosError = FVector::Dist(ActualEE.GetLocation(), TargetEndEffectorTransform.GetLocation());
	FQuat ActualRotError = TargetEndEffectorTransform.GetRotation() * ActualEE.GetRotation().Inverse();
	float ActualRotErrorDeg = FMath::RadiansToDegrees(ActualRotError.GetAngle());

	LastActualPosError = ActualPosError;
	LastActualRotError = ActualRotErrorDeg;

	// IK is successful if ACTUAL physical arm is within tolerance, not FK prediction
	// Note: ActualEE reflects the previous frame's pose (before new joint targets are applied)
	bLastIKSuccess = (ActualPosError < IKPositionTolerance) && (ActualRotErrorDeg < IKRotationTolerance);

	if (bEnableDebugLogging && GFrameCounter % 60 == 0)
	{
		UE_LOG(LogTemp, Verbose, TEXT("[Gen3] IK: IKSuccess=%d PosErr=%.2fcm RotErr=%.2fdeg Iter=%d | ActualSuccess=%d ActualPosErr=%.2fcm"),
			IKResult.bSuccess, IKResult.PositionError, IKResult.RotationError, IKResult.IterationsUsed,
			bLastIKSuccess, ActualPosError);
	}
}

// ============================================================================
// FK Diagnostics and Calibration
// ============================================================================

float UKinovaGen3ControllerComponent::ValidateForwardKinematics()
{
	if (!SkeletalMeshComponent || Joints.Num() == 0)
	{
		UE_LOG(LogTemp, Error, TEXT("[Gen3] ValidateFK: No skeletal mesh or joints"));
		return -1.0f;
	}

	// Get base transform
	FName	   FirstJointParent = SkeletalMeshComponent->GetParentBone(Joints[0].BoneName);
	FTransform BaseTransform = (FirstJointParent != NAME_None) ? SkeletalMeshComponent->GetBoneTransform(FirstJointParent, RTS_World) : SkeletalMeshComponent->GetComponentTransform();

	// Read current angles from constraints
	TArray<float> CurrentAngles;
	CurrentAngles.SetNum(Joints.Num());
	for (int32 i = 0; i < Joints.Num(); i++)
	{
		FName				 ConstraintName = (Joints[i].ConstraintName != NAME_None) ? Joints[i].ConstraintName : Joints[i].BoneName;
		FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintName);
		if (Constraint)
		{
			CurrentAngles[i] = GetConstraintAngle(Constraint, Joints[i].ControlledAxis) - Joints[i].AngleOffset;
		}
		else
		{
			CurrentAngles[i] = 0.0f;
		}
	}

	// Get joint local transforms and axes (simplified version of UpdateIK code)
	TArray<FTransform> JointLocalTransforms;
	TArray<FVector>	   JointAxesLocal;
	JointLocalTransforms.SetNum(Joints.Num());
	JointAxesLocal.SetNum(Joints.Num());

	const FReferenceSkeleton& RefSkeleton = SkeletalMeshComponent->GetSkeletalMeshAsset()->GetRefSkeleton();
	auto					  GetRefTransform = [&RefSkeleton](int32 BoneIdx) -> FTransform {
		 TArray<int32> Chain;
		 for (int32 Cur = BoneIdx; Cur != INDEX_NONE; Cur = RefSkeleton.GetParentIndex(Cur))
			 Chain.Add(Cur);
		 FTransform T = FTransform::Identity;
		 for (int32 k = Chain.Num() - 1; k >= 0; --k)
			 T = RefSkeleton.GetRefBonePose()[Chain[k]] * T;
		 return T;
	};

	TArray<FTransform> JointRefTransforms;
	JointRefTransforms.SetNum(Joints.Num());
	for (int32 i = 0; i < Joints.Num(); i++)
	{
		int32 BoneIdx = SkeletalMeshComponent->GetBoneIndex(Joints[i].BoneName);
		if (BoneIdx != INDEX_NONE)
			JointRefTransforms[i] = GetRefTransform(BoneIdx);
	}

	FTransform BaseRefTransform = FTransform::Identity;
	if (FirstJointParent != NAME_None)
	{
		int32 ParentIdx = SkeletalMeshComponent->GetBoneIndex(FirstJointParent);
		if (ParentIdx != INDEX_NONE)
			BaseRefTransform = GetRefTransform(ParentIdx);
	}

	if (bEnableDebugLogging && GFrameCounter % 60 == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Gen3] *** About to compute JointLocalTransforms: Joints.Num()=%d ***"), Joints.Num());
	}

	for (int32 i = 0; i < Joints.Num(); i++)
	{
		FTransform ParentTransform = (i == 0) ? BaseRefTransform : JointRefTransforms[i - 1];
		FTransform ThisTransform = JointRefTransforms[i];

		// Compute LOCAL transform: from parent frame to joint frame
		// This must include BOTH translation AND rotation from the reference skeleton
		JointLocalTransforms[i] = ThisTransform.GetRelativeTransform(ParentTransform);

		if (bEnableDebugLogging && i == 0 && GFrameCounter % 60 == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("[Gen3] *** UpdateInverseKinematics JointLocalTransforms computation ***"));
		}

		if (bEnableDebugLogging && i < 3)
		{
			UE_LOG(LogTemp, Log, TEXT("[Gen3] IK J%d LocalTransform: Loc=(%.2f, %.2f, %.2f) Rot=(%.2f, %.2f, %.2f)"),
				i,
				JointLocalTransforms[i].GetLocation().X, JointLocalTransforms[i].GetLocation().Y, JointLocalTransforms[i].GetLocation().Z,
				JointLocalTransforms[i].Rotator().Pitch, JointLocalTransforms[i].Rotator().Yaw, JointLocalTransforms[i].Rotator().Roll);
		}

		// Get joint axis from constraint (same logic as UpdateIK)
		FName				 ConstraintName = (Joints[i].ConstraintName != NAME_None) ? Joints[i].ConstraintName : Joints[i].BoneName;
		FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintName);

		FVector AxisJoint = FVector::XAxisVector; // Default
		if (Constraint)
		{
			const FTransform Frame1 = Constraint->GetRefFrame(EConstraintFrame::Frame1);

			// EMPIRICAL AXIS EXTRACTION (match UpdateInverseKinematics)
			FTransform ParentRefTransform = (i == 0) ? BaseRefTransform : JointRefTransforms[i - 1];
			FTransform ThisRefTransform = JointRefTransforms[i];
			FTransform LocalRef = ThisRefTransform.GetRelativeTransform(ParentRefTransform);

			FVector AxisLocal = FVector::XAxisVector;
			switch (Joints[i].ControlledAxis)
			{
				case EConstraintAxis::Twist:
					AxisLocal = LocalRef.GetUnitAxis(EAxis::X);
					break;
				case EConstraintAxis::Swing1:
					AxisLocal = LocalRef.GetUnitAxis(EAxis::Z);
					break;
				case EConstraintAxis::Swing2:
					AxisLocal = LocalRef.GetUnitAxis(EAxis::Y);
					break;
			}

			FVector AxisParent = AxisLocal.GetSafeNormal();

			if (Joints[i].bInvertAxisForIK)
				AxisParent = -AxisParent;

			// Frame1 axis is ALREADY in parent space - use directly (no transform!)
			AxisJoint = AxisParent;
		}

		JointAxesLocal[i] = AxisJoint;
	}

	// Compute FK using the SAME EndEffectorOffset as IK solver (cached)
	FTransform FK_EE = URammsIKLibrary::ComputeForwardKinematics(
		BaseTransform, CurrentAngles, CachedJointLocalTransforms, CachedJointAxesLocal, CachedEndEffectorOffset, false);

	// Get actual EE position
	FTransform Actual_EE = SkeletalMeshComponent->GetSocketTransform(EndEffectorBoneName, RTS_World);

	float Error = FVector::Dist(FK_EE.GetLocation(), Actual_EE.GetLocation());

	UE_LOG(LogTemp, Warning, TEXT("=== FK Validation ==="));
	UE_LOG(LogTemp, Warning, TEXT("[Gen3] FK End Effector:     (%.1f, %.1f, %.1f)"),
		FK_EE.GetLocation().X, FK_EE.GetLocation().Y, FK_EE.GetLocation().Z);
	UE_LOG(LogTemp, Warning, TEXT("[Gen3] Actual End Effector: (%.1f, %.1f, %.1f)"),
		Actual_EE.GetLocation().X, Actual_EE.GetLocation().Y, Actual_EE.GetLocation().Z);
	UE_LOG(LogTemp, Warning, TEXT("[Gen3] FK Error: %.2f cm"), Error);

	if (Error > 5.0f)
	{
		UE_LOG(LogTemp, Error, TEXT("[Gen3] FK ERROR TOO LARGE! Calibration needed."));
	}
	else if (Error > 1.0f)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Gen3] FK error is acceptable but could be better."));
	}
	else
	{
		UE_LOG(LogTemp, Display, TEXT("[Gen3] FK is accurate!"));
	}

	return Error;
}

void UKinovaGen3ControllerComponent::AutoCalibrateFK()
{
	if (!SkeletalMeshComponent || Joints.Num() == 0)
	{
		UE_LOG(LogTemp, Error, TEXT("[Gen3] AutoCalibrateFK: No skeletal mesh or joints"));
		return;
	}

	UE_LOG(LogTemp, Warning, TEXT("=== Auto-Calibrating FK ==="));

	// Get base transform
	FName	   FirstJointParent = SkeletalMeshComponent->GetParentBone(Joints[0].BoneName);
	FTransform BaseTransform = (FirstJointParent != NAME_None) ? SkeletalMeshComponent->GetBoneTransform(FirstJointParent, RTS_World) : SkeletalMeshComponent->GetComponentTransform();

	// For each joint, compute what offset would make FK match actual position
	for (int32 i = 0; i < Joints.Num(); i++)
	{
		FName				 ConstraintName = (Joints[i].ConstraintName != NAME_None) ? Joints[i].ConstraintName : Joints[i].BoneName;
		FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintName);

		if (!Constraint)
		{
			UE_LOG(LogTemp, Warning, TEXT("[Gen3] Joint %d: No constraint found"), i);
			continue;
		}

		// Get actual bone transform
		FTransform ActualBone = SkeletalMeshComponent->GetBoneTransform(Joints[i].BoneName, RTS_World);

		// Read raw constraint angle (without any offsets)
		float RawConstraintAngle = GetConstraintAngle(Constraint, Joints[i].ControlledAxis);

		// Get Frame1 rotation to check if there's a coordinate frame issue
		FQuat	 Frame1Rot = Constraint->GetRefFrame(EConstraintFrame::Frame1).GetRotation();
		FRotator Frame1Rotator = Frame1Rot.Rotator();

		// For now, just log the info - user can manually set offsets
		UE_LOG(LogTemp, Warning, TEXT("[Gen3] Joint %d (%s):"), i, *Joints[i].BoneName.ToString());
		UE_LOG(LogTemp, Warning, TEXT("  Raw Constraint Angle: %.2f°"), RawConstraintAngle);
		UE_LOG(LogTemp, Warning, TEXT("  Current Offset: %.2f°"), Joints[i].AngleOffset);
		UE_LOG(LogTemp, Warning, TEXT("  Frame1 Rotation: P=%.1f Y=%.1f R=%.1f"),
			Frame1Rotator.Pitch, Frame1Rotator.Yaw, Frame1Rotator.Roll);

		// Check if Frame1 has significant rotation on the controlled axis
		float AxisRotation = 0.0f;
		switch (Joints[i].ControlledAxis)
		{
			case EConstraintAxis::Twist:
				AxisRotation = Frame1Rotator.Roll;
				break;
			case EConstraintAxis::Swing1:
				AxisRotation = Frame1Rotator.Pitch;
				break;
			case EConstraintAxis::Swing2:
				AxisRotation = Frame1Rotator.Yaw;
				break;
		}

		if (FMath::Abs(AxisRotation) > 5.0f)
		{
			UE_LOG(LogTemp, Warning, TEXT("  -> Frame1 has %.1f° rotation on controlled axis!"), AxisRotation);
			UE_LOG(LogTemp, Warning, TEXT("  -> Suggested offset: %.2f° (add to current: %.2f°)"),
				-AxisRotation, Joints[i].AngleOffset - AxisRotation);
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("=== Calibration Analysis Complete ==="));
	UE_LOG(LogTemp, Warning, TEXT("Review the logs above and manually adjust AngleOffset values in the details panel."));
	UE_LOG(LogTemp, Warning, TEXT("Then run ValidateForwardKinematics() to check if FK is accurate."));
}

void UKinovaGen3ControllerComponent::PrintFKDiagnostics()
{
	if (!SkeletalMeshComponent || Joints.Num() == 0)
	{
		UE_LOG(LogTemp, Error, TEXT("[Gen3] PrintFKDiagnostics: No skeletal mesh or joints"));
		return;
	}

	UE_LOG(LogTemp, Warning, TEXT("=== FK Diagnostics ==="));

	// Validate FK first
	float FKError = ValidateForwardKinematics();

	// Print each joint's info
	for (int32 i = 0; i < Joints.Num(); i++)
	{
		FName				 ConstraintName = (Joints[i].ConstraintName != NAME_None) ? Joints[i].ConstraintName : Joints[i].BoneName;
		FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintName);

		UE_LOG(LogTemp, Warning, TEXT("\nJoint %d: %s"), i, *Joints[i].BoneName.ToString());
		UE_LOG(LogTemp, Warning, TEXT("  Constraint: %s"), *ConstraintName.ToString());
		UE_LOG(LogTemp, Warning, TEXT("  Current Angle: %.2f°"), Joints[i].CurrentAngle);
		UE_LOG(LogTemp, Warning, TEXT("  Target Angle: %.2f°"), Joints[i].TargetAngle);
		UE_LOG(LogTemp, Warning, TEXT("  Angle Offset: %.2f°"), Joints[i].AngleOffset);

		if (Constraint)
		{
			float RawAngle = GetConstraintAngle(Constraint, Joints[i].ControlledAxis);
			UE_LOG(LogTemp, Warning, TEXT("  Raw Constraint Angle: %.2f°"), RawAngle);

			FQuat	 Frame1Rot = Constraint->GetRefFrame(EConstraintFrame::Frame1).GetRotation();
			FRotator Frame1Rotator = Frame1Rot.Rotator();
			UE_LOG(LogTemp, Warning, TEXT("  Frame1 Rotation: P=%.1f° Y=%.1f° R=%.1f°"),
				Frame1Rotator.Pitch, Frame1Rotator.Yaw, Frame1Rotator.Roll);
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("  Constraint NOT FOUND!"));
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("\n=== End FK Diagnostics ==="));
}

float UKinovaGen3ControllerComponent::ValidateFABRIKConstraints()
{
	if (!SkeletalMeshComponent || Joints.Num() == 0)
	{
		UE_LOG(LogTemp, Error, TEXT("[Gen3] ValidateFABRIK: No skeletal mesh or joints"));
		return -1.0f;
	}

	float MaxViolation = 0.0f;
	int32 ViolationCount = 0;

	UE_LOG(LogTemp, Warning, TEXT("=== FABRIK Constraint Validation ==="));

	for (int32 i = 0; i < Joints.Num(); i++)
	{
		const FRevoluteJointConfig& Joint = Joints[i];
		float						CurrentAngle = Joint.CurrentAngle;

		// Check if current angle violates limits
		bool bViolatesMin = (CurrentAngle < Joint.MinAngleLimit);
		bool bViolatesMax = (CurrentAngle > Joint.MaxAngleLimit);

		if (bViolatesMin || bViolatesMax)
		{
			float Violation = 0.0f;
			if (bViolatesMin)
			{
				Violation = Joint.MinAngleLimit - CurrentAngle;
			}
			else
			{
				Violation = CurrentAngle - Joint.MaxAngleLimit;
			}

			MaxViolation = FMath::Max(MaxViolation, Violation);
			ViolationCount++;

			UE_LOG(LogTemp, Warning, TEXT("[Joint %d: %s] CONSTRAINT VIOLATION!"), i, *Joint.BoneName.ToString());
			UE_LOG(LogTemp, Warning, TEXT("  Current: %.2f° | Limits: [%.2f°, %.2f°] | Violation: %.2f°"),
				CurrentAngle, Joint.MinAngleLimit, Joint.MaxAngleLimit, Violation);
		}
		else
		{
			float MinMargin = CurrentAngle - Joint.MinAngleLimit;
			float MaxMargin = Joint.MaxAngleLimit - CurrentAngle;
			float MinToLimit = FMath::Min(MinMargin, MaxMargin);

			UE_LOG(LogTemp, Log, TEXT("[Joint %d: %s] OK - Angle: %.2f° | Limits: [%.2f°, %.2f°] | Margin: %.2f°"),
				i, *Joint.BoneName.ToString(), CurrentAngle, Joint.MinAngleLimit, Joint.MaxAngleLimit, MinToLimit);
		}
	}

	if (ViolationCount > 0)
	{
		UE_LOG(LogTemp, Error, TEXT("=== VALIDATION FAILED: %d joints violate constraints, max violation: %.2f° ==="),
			ViolationCount, MaxViolation);
	}
	else
	{
		UE_LOG(LogTemp, Log, TEXT("=== VALIDATION PASSED: All joints within constraints ==="));
	}

	return MaxViolation;
}
