// Copyright Epic Games, Inc. All Rights Reserved.

#include "KinovaGen3ControllerComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "DrawDebugHelpers.h"
#include "RammsIKLibrary.h"
#include "Engine/SkeletalMeshSocket.h"

UKinovaGen3ControllerComponent::UKinovaGen3ControllerComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
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
			UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] SkeletalMeshComponentName changed to %s, auto-populating joints..."), *SkeletalMeshComponentName.ToString());
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
			UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Found skeletal mesh component: %s"), *SkeletalMeshComponent->GetName());

			// Initialize current angles from current constraint positions
			for (FRevoluteJointConfig& Joint : Joints)
			{
				if (Joint.BoneName == NAME_None)
					continue;

				FName ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
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
					Joint.TargetAngle = Joint.CurrentAngle;

					if (bEnableDebugLogging)
					{
						UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Initialized joint %s on %s axis at %.2f degrees (inverted=%d)"),
							*Joint.BoneName.ToString(), 
							Joint.ControlledAxis == EConstraintAxis::Twist ? TEXT("Twist") :
							Joint.ControlledAxis == EConstraintAxis::Swing1 ? TEXT("Swing1") : TEXT("Swing2"),
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
					UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Initialized null-space bias with %d zeros"), NullSpaceBias.Num());
				}
			}

			UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Initialized with %d joints"), Joints.Num());
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

		FName ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
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
			UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Initialized constraint for joint %s: Axis=%d, Strength=%.0f, Damping=%.0f, MaxTorque=%.1f"),
				*Joint.BoneName.ToString(),
				(int32)Joint.ControlledAxis,
				Joint.PositionStrength,
				Joint.PositionDamping,
				Joint.MaxTorque);
		}
	}

	UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Initialized %d joint constraints"), InitializedCount);
}

void UKinovaGen3ControllerComponent::ReinitializeConstraints()
{
	InitializeJointConstraints();
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
		// Use inverse kinematics to reach target
		UpdateInverseKinematics(DeltaTime);
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
		UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] UpdatePositionControl: %d joints"), Joints.Num());
	}

	for (FRevoluteJointConfig& Joint : Joints)
	{
		if (Joint.BoneName == NAME_None)
			continue;

		FName ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
		FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintToUse);
		
		if (!Constraint)
		{
			if (bEnableDebugLogging)
			{
				UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] UpdatePositionControl: Cannot find constraint %s"), *ConstraintToUse.ToString());
			}
			continue;
		}

		// Read current angle from physics
		Joint.CurrentAngle = GetConstraintAngle(Constraint, Joint.ControlledAxis);

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

		FName ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
		FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintToUse);
		
		if (!Constraint) {
          if (bEnableDebugLogging) {
            UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] UpdateTorqueControl: Cannot find constraint %s"), *ConstraintToUse.ToString());
          }
          continue;
        }

		// Get current angle from physics
		Joint.CurrentAngle = GetConstraintAngle(Constraint, Joint.ControlledAxis);

		// Calculate error
		float AngleError = Joint.TargetAngle - Joint.CurrentAngle;
		while (AngleError > 180.0f) AngleError -= 360.0f;
		while (AngleError < -180.0f) AngleError += 360.0f;

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

	FName ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
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
		TargetForDrive = ClampToLimits(Joint, Joint.TargetAngle);
	}
	
	// Set target orientation based on controlled axis
	SetConstraintAngle(Constraint, Joint.ControlledAxis, TargetForDrive);

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Joint %s: Current=%.2f Target=%.2f"),
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

	// Convert to radians
	float AngleRadians = FMath::DegreesToRadians(AngleDegrees);
	
	// Create a quaternion target based on the controlled axis
	// These are in the constraint's Frame1 local coordinate system
	FQuat TargetQuat = FQuat::Identity;
	
	switch (Axis)
	{
	case EConstraintAxis::Swing1:
		// Swing1 around Z-axis  
		TargetQuat = FQuat(FVector(0, 0, 1), AngleRadians);
		break;
	case EConstraintAxis::Swing2:
		// Swing2 around Y-axis
		TargetQuat = FQuat(FVector(0, 1, 0), AngleRadians);
		break;
	case EConstraintAxis::Twist:
		// Twist around X-axis
		TargetQuat = FQuat(FVector(1, 0, 0), AngleRadians);
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
	float Limit1 = 0.0f;
	float Limit2 = 0.0f;
	
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
	int32 BoneIndex = SkeletalMeshComponent->GetBoneIndex(BoneName);
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
	USkeletalMeshComponent* TempSkeletalMesh = SkeletalMeshComponent;
	
	// If not already set, try to find it
	if (!TempSkeletalMesh)
	{
		AActor* Owner = GetOwner();
		if (!Owner)
		{
			UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] Cannot auto-populate joints: No owner actor"));
			return;
		}

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

	if (!TempSkeletalMesh)
	{
		UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] Cannot auto-populate joints: No skeletal mesh component"));
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] AutoPopulateJoints called on %s"), *TempSkeletalMesh->GetName());
	UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] SkeletalMesh: %s"), TempSkeletalMesh->GetSkeletalMeshAsset() ? *TempSkeletalMesh->GetSkeletalMeshAsset()->GetName() : TEXT("None"));
	UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Physics enabled: %d"), TempSkeletalMesh->IsSimulatingPhysics());

	if (bOverwriteExisting)
	{
		Joints.Empty();
	}

	// Get all physics constraints from the skeletal mesh
	TArray<FConstraintInstanceAccessor> ConstraintAccessors;
	TempSkeletalMesh->GetConstraints(false, ConstraintAccessors);

	UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Found %d constraint accessors on skeletal mesh"), ConstraintAccessors.Num());

	for (FConstraintInstanceAccessor& Accessor : ConstraintAccessors)
	{
		FConstraintInstance* Constraint = Accessor.Get();
		if (!Constraint)
		{
			UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] Constraint accessor returned null constraint"));
			continue;
		}

		UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Checking constraint: %s"), *Constraint->JointName.ToString());

		// Check if any axis is free or limited (indicating a revolute joint)
		bool bHasFreeAxis = IsAxisFreeOrLimited(Constraint, EConstraintAxis::Twist) ||
		                    IsAxisFreeOrLimited(Constraint, EConstraintAxis::Swing1) ||
		                    IsAxisFreeOrLimited(Constraint, EConstraintAxis::Swing2);

		UE_LOG(LogTemp, Log, TEXT("[KinovaGen3]   Has free axis: %d (Twist:%d Swing1:%d Swing2:%d)"), 
			bHasFreeAxis,
			IsAxisFreeOrLimited(Constraint, EConstraintAxis::Twist),
			IsAxisFreeOrLimited(Constraint, EConstraintAxis::Swing1),
			IsAxisFreeOrLimited(Constraint, EConstraintAxis::Swing2));

		if (!bHasFreeAxis)
			continue;

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

		Joints.Add(NewJoint);

		UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Added joint: %s (Constraint: %s, Axis: %s, Limits: %.1f to %.1f)"),
			*NewJoint.BoneName.ToString(),
			*NewJoint.ConstraintName.ToString(),
			NewJoint.ControlledAxis == EConstraintAxis::Twist ? TEXT("Twist") :
			NewJoint.ControlledAxis == EConstraintAxis::Swing1 ? TEXT("Swing1") : TEXT("Swing2"),
			NewJoint.MinAngleLimit, NewJoint.MaxAngleLimit);
	}

	UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Auto-populated %d joints"), Joints.Num());
}

void UKinovaGen3ControllerComponent::AutoPopulateJointsFromConstraints()
{
	AutoPopulateJoints(true);
}

bool UKinovaGen3ControllerComponent::AutoDetectJointAxis(int32 JointIndex)
{
	if (!Joints.IsValidIndex(JointIndex))
		return false;

	FRevoluteJointConfig& Joint = Joints[JointIndex];
	
	if (!SkeletalMeshComponent)
		return false;

	FName ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
	FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintToUse);
	
	if (!Constraint)
		return false;

	// Check each axis in order of preference (Twist -> Swing1 -> Swing2)
	if (IsAxisFreeOrLimited(Constraint, EConstraintAxis::Twist))
	{
		Joint.ControlledAxis = EConstraintAxis::Twist;
	}
	else if (IsAxisFreeOrLimited(Constraint, EConstraintAxis::Swing1))
	{
		Joint.ControlledAxis = EConstraintAxis::Swing1;
	}
	else if (IsAxisFreeOrLimited(Constraint, EConstraintAxis::Swing2))
	{
		Joint.ControlledAxis = EConstraintAxis::Swing2;
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("[KinovaGen3] No free/limited axis found for joint %s"),
			*Joint.BoneName.ToString());
		return false;
	}

	UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Joint %s configured to use %s axis"),
		*Joint.BoneName.ToString(),
		Joint.ControlledAxis == EConstraintAxis::Twist ? TEXT("Twist") :
		Joint.ControlledAxis == EConstraintAxis::Swing1 ? TEXT("Swing1") : TEXT("Swing2"));

	return true;
}

void UKinovaGen3ControllerComponent::AutoDetectAllJointAxes()
{
	int32 SuccessCount = 0;
	
	for (int32 i = 0; i < Joints.Num(); ++i)
	{
		if (AutoDetectJointAxis(i))
		{
			SuccessCount++;
		}
	}

	UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Auto-detected axes for %d/%d joints"),
		SuccessCount, Joints.Num());
}

void UKinovaGen3ControllerComponent::DebugDraw()
{
	if (!SkeletalMeshComponent)
		return;

	UWorld* World = GetWorld();
	if (!World)
		return;

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
			FVector Origin = BoneTransform.GetLocation();
			
			// Draw coordinate frame
			float AxisLength = 10.0f;
			DrawDebugLine(World, Origin, Origin + BoneTransform.GetUnitAxis(EAxis::X) * AxisLength, FColor::Red, false, 0.0f, 0, 1.0f);
			DrawDebugLine(World, Origin, Origin + BoneTransform.GetUnitAxis(EAxis::Y) * AxisLength, FColor::Green, false, 0.0f, 0, 1.0f);
			DrawDebugLine(World, Origin, Origin + BoneTransform.GetUnitAxis(EAxis::Z) * AxisLength, FColor::Blue, false, 0.0f, 0, 1.0f);
			
			// Draw the actual rotation axis for this joint (in IK mode, show which axis it rotates around)
			if (ArmControlMode == EArmControlMode::EndEffectorControl)
			{
				FName ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
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
					DrawDebugLine(World, Origin, Origin + XAxis * DebugAxisLen, FColor::Red, false, 0.0f, 0, 3.0f);
					DrawDebugString(World, Origin + XAxis * DebugAxisLen, TEXT("X(Twist)"), nullptr, FColor::Red, 0.0f, false, 1.0f);
					
					DrawDebugLine(World, Origin, Origin + YAxis * DebugAxisLen, FColor::Green, false, 0.0f, 0, 3.0f);
					DrawDebugString(World, Origin + YAxis * DebugAxisLen, TEXT("Y(Swing2)"), nullptr, FColor::Green, 0.0f, false, 1.0f);
					
					DrawDebugLine(World, Origin, Origin + ZAxis * DebugAxisLen, FColor::Blue, false, 0.0f, 0, 3.0f);
					DrawDebugString(World, Origin + ZAxis * DebugAxisLen, TEXT("Z(Swing1)"), nullptr, FColor::Blue, 0.0f, false, 1.0f);
					
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
					
					DrawDebugLine(World, Origin, Origin + ActiveAxis * DebugAxisLen * 1.2f, FColor::Magenta, false, 0.0f, 0, 5.0f);
					DrawDebugString(World, Origin + ActiveAxis * DebugAxisLen * 1.2f, AxisName, nullptr, FColor::Magenta, 0.0f, true, 1.2f);
				}
			}
			
			// Draw a small label with joint name
			DrawDebugString(World, Origin, Joint.BoneName.ToString(), nullptr, FColor::White, 0.0f, true, 0.8f);
		}
	}

	// Draw end-effector
	if (EndEffectorBoneName != NAME_None)
	{
		DrawDebugSphere(World, EndEffectorState.Position, 5.0f, 8, FColor::Yellow, false, 0.0f, 0, 2.0f);
		
		// Draw end-effector frame
		FTransform EndEffectorTransform = SkeletalMeshComponent->GetSocketTransform(EndEffectorBoneName, RTS_World);
		FVector Origin = EndEffectorTransform.GetLocation();
		float AxisLength = 15.0f;
		DrawDebugLine(World, Origin, Origin + EndEffectorTransform.GetUnitAxis(EAxis::X) * AxisLength, FColor::Red, false, 0.0f, 0, 2.0f);
		DrawDebugLine(World, Origin, Origin + EndEffectorTransform.GetUnitAxis(EAxis::Y) * AxisLength, FColor::Green, false, 0.0f, 0, 2.0f);
		DrawDebugLine(World, Origin, Origin + EndEffectorTransform.GetUnitAxis(EAxis::Z) * AxisLength, FColor::Blue, false, 0.0f, 0, 2.0f);
	}

	// Draw IK target when in end effector control mode
	if (ArmControlMode == EArmControlMode::EndEffectorControl)
	{
		FVector TargetPos = TargetEndEffectorTransform.GetLocation();
		FQuat TargetRot = TargetEndEffectorTransform.GetRotation();
		
		// Draw target position as cyan sphere
		DrawDebugSphere(World, TargetPos, 8.0f, 12, FColor::Cyan, false, 0.0f, 0, 3.0f);
		
		// Draw target orientation frame
		float TargetAxisLength = 20.0f;
		DrawDebugLine(World, TargetPos, TargetPos + TargetRot.GetAxisX() * TargetAxisLength, FColor::Red, false, 0.0f, 0, 3.0f);
		DrawDebugLine(World, TargetPos, TargetPos + TargetRot.GetAxisY() * TargetAxisLength, FColor::Green, false, 0.0f, 0, 3.0f);
		DrawDebugLine(World, TargetPos, TargetPos + TargetRot.GetAxisZ() * TargetAxisLength, FColor::Blue, false, 0.0f, 0, 3.0f);
		
		// Draw line from current to target end effector
		if (EndEffectorBoneName != NAME_None)
		{
			DrawDebugLine(World, EndEffectorState.Position, TargetPos, FColor::Cyan, false, 0.0f, 0, 1.0f);
		}
		
		// Draw label
		DrawDebugString(World, TargetPos + FVector(0, 0, 15), TEXT("IK Target"), nullptr, FColor::Cyan, 0.0f, true, 1.0f);
		
		// ===== DEBUG: Draw FK skeleton to verify FK computation =====
		if (Joints.Num() > 0)
		{
			// Build FK chain data (same as IK does)
			TArray<FTransform> JointLocalTransforms;
			TArray<FVector> JointAxesLocal;
			TArray<float> CurrentAngles;
			
			// Get base transform
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
			
			// Build joint data
			for (int32 JointIndex = 0; JointIndex < Joints.Num(); JointIndex++)
			{
				const FRevoluteJointConfig& Joint = Joints[JointIndex];
				if (Joint.BoneName == NAME_None)
					continue;
					
				int32 BoneIndex = SkeletalMeshComponent->GetBoneIndex(Joint.BoneName);
				if (BoneIndex == INDEX_NONE)
					continue;
					
				// Get reference pose local transform
				const FReferenceSkeleton& RefSkeleton = SkeletalMeshComponent->GetSkeletalMeshAsset()->GetRefSkeleton();
				FTransform BoneLocalTransform = RefSkeleton.GetRefBonePose()[BoneIndex];
				JointLocalTransforms.Add(BoneLocalTransform);
				
				// Get axis (same logic as IK)
				FName ConstraintToUse = Joint.ConstraintName != NAME_None ? Joint.ConstraintName : Joint.BoneName;
				FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintToUse);
				
				FVector JointAxisLocal = FVector::XAxisVector;
				if (Constraint)
				{
					FTransform ConstraintFrame1 = Constraint->GetRefFrame(EConstraintFrame::Frame1);
					FBodyInstance* ParentBody = nullptr;
					FName ParentBoneName = NAME_None;
					
					if (BoneIndex != INDEX_NONE)
					{
						ParentBoneName = SkeletalMeshComponent->GetParentBone(Joint.BoneName);
						if (ParentBoneName != NAME_None)
						{
							ParentBody = SkeletalMeshComponent->GetBodyInstance(ParentBoneName);
						}
					}
					
					if (ParentBody)
					{
						FTransform ParentWorldTransform = ParentBody->GetUnrealWorldTransform();
						FTransform ConstraintWorldFrame = ConstraintFrame1 * ParentWorldTransform;
						
						FVector JointAxisWorld = FVector::XAxisVector;
						switch (Joint.ControlledAxis)
						{
						case EConstraintAxis::Twist:
							JointAxisWorld = ConstraintWorldFrame.GetUnitAxis(EAxis::X);
							break;
						case EConstraintAxis::Swing1:
							JointAxisWorld = ConstraintWorldFrame.GetUnitAxis(EAxis::Z);
							break;
						case EConstraintAxis::Swing2:
							JointAxisWorld = ConstraintWorldFrame.GetUnitAxis(EAxis::Y);
							break;
						}
						
						FTransform ParentLocalTransform;
						if (JointIndex == 0)
						{
							ParentLocalTransform = BaseTransform;
						}
						else
						{
							ParentLocalTransform = SkeletalMeshComponent->GetSocketTransform(ParentBoneName, RTS_World);
						}
						
						JointAxisLocal = ParentLocalTransform.InverseTransformVectorNoScale(JointAxisWorld);
					}
					
					if (Joint.bInvertAxisForIK)
					{
						JointAxisLocal = -JointAxisLocal;
					}
				}
				
				JointAxesLocal.Add(JointAxisLocal);
				CurrentAngles.Add(Joint.CurrentAngle);
			}
			
			// Get end effector offset from reference pose
			FTransform EndEffectorOffset = FTransform::Identity;
			if (EndEffectorBoneName != NAME_None && Joints.Num() > 0)
			{
				const FReferenceSkeleton& RefSkeleton = SkeletalMeshComponent->GetSkeletalMeshAsset()->GetRefSkeleton();
				int32 LastJointBoneIndex = SkeletalMeshComponent->GetBoneIndex(Joints.Last().BoneName);
				int32 EEBoneIndex = SkeletalMeshComponent->GetBoneIndex(EndEffectorBoneName);
				
				if (LastJointBoneIndex != INDEX_NONE && EEBoneIndex != INDEX_NONE)
				{
					auto GetRefPoseAccum = [&RefSkeleton](int32 TargetBoneIdx) -> FTransform
					{
						TArray<int32> Chain;
						int32 Cur = TargetBoneIdx;
						while (Cur != INDEX_NONE)
						{
							Chain.Add(Cur);
							Cur = RefSkeleton.GetParentIndex(Cur);
						}
						
						FTransform Accum = FTransform::Identity;
						for (int32 k = Chain.Num() - 1; k >= 0; --k)
						{
							Accum = RefSkeleton.GetRefBonePose()[Chain[k]] * Accum;  // CORRECT ORDER
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
			DrawDebugSphere(World, BasePosWorld, 3.0f, 8, FColor::Orange, false, 0.0f, 0, 2.0f);
			DrawDebugString(World, BasePosWorld, TEXT("FK Base"), nullptr, FColor::Orange, 0.0f, true, 0.8f);
			
			// FVector PrevPosWorld = BasePosWorld;
			
			// Build FK incrementally to draw each joint
			FTransform CurrentTransform = BaseTransform;
			FVector PrevPosWorld = BaseTransform.GetLocation();
			
			for (int32 i = 0; i < JointAxesLocal.Num(); i++)
			{
				// 1. Translate to joint pivot
				FVector LocalOffset = JointLocalTransforms[i].GetTranslation();
				FVector WorldOffset = CurrentTransform.TransformVectorNoScale(LocalOffset);
				CurrentTransform.AddToTranslation(WorldOffset);
				
				FVector JointPivotWorld = CurrentTransform.GetLocation();
				
				// Draw link from previous position to this joint pivot
				DrawDebugLine(World, PrevPosWorld, JointPivotWorld, FColor::Orange, false, 0.0f, 0, 3.0f);
				
				// Draw joint pivot sphere
				DrawDebugSphere(World, JointPivotWorld, 3.0f, 8, FColor::Orange, false, 0.0f, 0, 2.0f);
				DrawDebugString(World, JointPivotWorld, FString::Printf(TEXT("J%d"), i), nullptr, FColor::Orange, 0.0f, true, 0.8f);
				
				// 2. Rotate at this joint pivot
				FVector WorldAxis = CurrentTransform.TransformVectorNoScale(JointAxesLocal[i]).GetSafeNormal();
				float AngleRad = FMath::DegreesToRadians(CurrentAngles[i]);
				FQuat JointRotation(WorldAxis, AngleRad);
				CurrentTransform.SetRotation(JointRotation * CurrentTransform.GetRotation());
				
				// Draw joint axis after rotation
				DrawDebugLine(World, JointPivotWorld, JointPivotWorld + WorldAxis * 15.0f, FColor::Purple, false, 0.0f, 0, 4.0f);
				
				PrevPosWorld = JointPivotWorld;
			}
			
			// Draw FK end effector
			FTransform FKEndEffectorWorld = URammsIKLibrary::ComputeForwardKinematics(
				BaseTransform, CurrentAngles, JointLocalTransforms, JointAxesLocal, EndEffectorOffset, false);
			
			DrawDebugLine(World, PrevPosWorld, FKEndEffectorWorld.GetLocation(), FColor::Orange, false, 0.0f, 0, 3.0f);
			DrawDebugSphere(World, FKEndEffectorWorld.GetLocation(), 5.0f, 12, FColor::Orange, false, 0.0f, 0, 3.0f);
			DrawDebugString(World, FKEndEffectorWorld.GetLocation(), TEXT("FK EE"), nullptr, FColor::Orange, 0.0f, true, 1.0f);
		}
	}

	// On-screen debug text
	if (bEnableDebugDisplay)
	{
		FString DebugText = FString::Printf(TEXT("Kinova Gen3 Arm\nArm Mode: %s\nControl Mode: %s\nJoints: %d\n"),
			ArmControlMode == EArmControlMode::JointControl ? TEXT("Joint Control") : TEXT("IK Control"),
			ControlMode == EJointControlMode::PositionControl ? TEXT("Position") :
			ControlMode == EJointControlMode::VelocityControl ? TEXT("Velocity") : TEXT("Torque"),
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
			UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Set joint %d (%s) target to %.2f degrees"),
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
		UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Set %d joint targets"), NumToSet);
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
}

void UKinovaGen3ControllerComponent::SetEndEffectorTargetPosition(const FVector& TargetPosition)
{
	TargetEndEffectorTransform.SetLocation(TargetPosition);
}

void UKinovaGen3ControllerComponent::SetEndEffectorTargetRotation(const FRotator& TargetRotation)
{
	TargetEndEffectorTransform.SetRotation(TargetRotation.Quaternion());
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

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Set IK target relative to base: World Pos=(%.1f, %.1f, %.1f)"),
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
	FVector WorldPosition = BaseTransform.TransformPosition(RelativePosition);
	TargetEndEffectorTransform.SetLocation(WorldPosition);

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Set IK target position relative to base: World Pos=(%.1f, %.1f, %.1f)"),
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

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Set IK target relative to actor: World Pos=(%.1f, %.1f, %.1f)"),
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
	FVector WorldPosition = ActorTransform.TransformPosition(RelativePosition);
	TargetEndEffectorTransform.SetLocation(WorldPosition);

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Set IK target position relative to actor: World Pos=(%.1f, %.1f, %.1f)"),
			WorldPosition.X, WorldPosition.Y, WorldPosition.Z);
	}
}

void UKinovaGen3ControllerComponent::MoveEndEffectorTargetBy(const FVector& Offset)
{
	// Move target by world-space offset
	FVector CurrentPos = TargetEndEffectorTransform.GetLocation();
	TargetEndEffectorTransform.SetLocation(CurrentPos + Offset);

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Moved IK target by (%.1f, %.1f, %.1f) to (%.1f, %.1f, %.1f)"),
			Offset.X, Offset.Y, Offset.Z,
			TargetEndEffectorTransform.GetLocation().X,
			TargetEndEffectorTransform.GetLocation().Y,
			TargetEndEffectorTransform.GetLocation().Z);
	}
}

void UKinovaGen3ControllerComponent::MoveEndEffectorTargetByLocal(const FVector& LocalOffset)
{
	// Move target by offset in end effector's current orientation
	FQuat CurrentRotation = TargetEndEffectorTransform.GetRotation();
	FVector WorldOffset = CurrentRotation.RotateVector(LocalOffset);
	
	FVector CurrentPos = TargetEndEffectorTransform.GetLocation();
	TargetEndEffectorTransform.SetLocation(CurrentPos + WorldOffset);

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[KinovaGen3] Moved IK target by local (%.1f, %.1f, %.1f) to world (%.1f, %.1f, %.1f)"),
			LocalOffset.X, LocalOffset.Y, LocalOffset.Z,
			TargetEndEffectorTransform.GetLocation().X,
			TargetEndEffectorTransform.GetLocation().Y,
			TargetEndEffectorTransform.GetLocation().Z);
	}
}

FVector UKinovaGen3ControllerComponent::ComputeEmpiricalRotationAxis(int32 JointIndex, const FTransform& ParentWorldTransform)
{
	if (!SkeletalMeshComponent || JointIndex < 0 || JointIndex >= Joints.Num())
	{
		return FVector::XAxisVector;
	}
	
	FRevoluteJointConfig& Joint = Joints[JointIndex];
	FName ConstraintName = (Joint.ConstraintName != NAME_None) ? Joint.ConstraintName : Joint.BoneName;
	FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintName);
	
	if (!Constraint)
	{
		return FVector::XAxisVector;
	}
	
	int32 BoneIdx = SkeletalMeshComponent->GetBoneIndex(Joint.BoneName);
	if (BoneIdx == INDEX_NONE)
	{
		return FVector::XAxisVector;
	}
	
	// Get current angle
	float OriginalAngle = GetConstraintAngle(Constraint, Joint.ControlledAxis);
	
	// Get bone transform at current angle (component space)
	FTransform OriginalTransformCS = SkeletalMeshComponent->GetBoneTransform(BoneIdx, FTransform::Identity);
	FVector OriginalPosWorld = OriginalTransformCS.GetLocation();
	
	// Apply small perturbation (+5 degrees)
	float PerturbedAngle = OriginalAngle + 5.0f;
	SetConstraintAngle(Constraint, Joint.ControlledAxis, PerturbedAngle);
	
	// Force a physics update to see the change
	// Note: This is a hack - ideally we'd compute this from constraint frames directly
	// For now, we'll just compute from constraint frame orientation
	
	// Restore original angle immediately
	SetConstraintAngle(Constraint, Joint.ControlledAxis, OriginalAngle);
	
	// Instead, let's extract the axis from the constraint frame, but transform it properly
	// through the parent's world transform
	FTransform Frame1 = Constraint->GetRefFrame(EConstraintFrame::Frame1);
	
	// Get axis in Frame1 local space
	FVector AxisInFrame1;
	switch (Joint.ControlledAxis)
	{
	case EConstraintAxis::Swing1:
		AxisInFrame1 = Frame1.GetUnitAxis(EAxis::Z);
		break;
	case EConstraintAxis::Swing2:
		AxisInFrame1 = Frame1.GetUnitAxis(EAxis::Y);
		break;
	case EConstraintAxis::Twist:
	default:
		AxisInFrame1 = Frame1.GetUnitAxis(EAxis::X);
		break;
	}
	
	// Transform axis through parent's world transform to get world-space axis
	FVector AxisWorld = ParentWorldTransform.TransformVectorNoScale(AxisInFrame1);
	
	return AxisWorld.GetSafeNormal();
}

void UKinovaGen3ControllerComponent::UpdateInverseKinematics(float DeltaTime)
{
	if (!SkeletalMeshComponent || Joints.Num() == 0)
		return;

	// Update target transform from target actor if set
	if (TargetActor && TargetActor->IsValidLowLevel())
	{
		TargetEndEffectorTransform = TargetActor->GetActorTransform();
	}
	
	// Debug: Log joint order once
	static bool bLoggedJointOrder = false;
	if (bEnableDebugLogging && !bLoggedJointOrder)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Gen3] Joint order (should be base→EE):"));
		for (int32 i = 0; i < Joints.Num(); i++)
		{
			UE_LOG(LogTemp, Warning, TEXT("  [%d]: %s"), i, *Joints[i].BoneName.ToString());
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
	auto GetRefTransform = [&RefSkeleton](int32 BoneIdx) -> FTransform
	{
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
		if (BoneIdx == INDEX_NONE) return;
		JointRefTransforms[i] = GetRefTransform(BoneIdx);
	}
	
	// Get base reference transform (parent of first joint)
	FTransform BaseRefTransform = FTransform::Identity;
	FName FirstJointParent = NAME_None;
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
					FVector BaseLoc = BaseRefTransform.GetLocation();
					FRotator BaseRot = BaseRefTransform.Rotator();
					UE_LOG(LogTemp, Log, TEXT("[Gen3] BaseRef '%s': Loc=(%.1f, %.1f, %.1f) Rot=(P:%.1f, Y:%.1f, R:%.1f)"),
						*FirstJointParent.ToString(), BaseLoc.X, BaseLoc.Y, BaseLoc.Z, 
						BaseRot.Pitch, BaseRot.Yaw, BaseRot.Roll);
				}
			}
		}
	}
	
	// ============================================================================
	// STEP 2: Compute Joint Local Offsets (Pure Translations)
	// ============================================================================
	// Offset[i] = translation from parent[i] to joint[i] in parent's local frame
	// For i=0: parent is base; for i>0: parent is joint[i-1]
	
	TArray<FTransform> JointLocalTransforms;
	JointLocalTransforms.SetNum(Joints.Num());
	
	for (int32 i = 0; i < Joints.Num(); i++)
	{
		FTransform ParentTransform = (i == 0) ? BaseRefTransform : JointRefTransforms[i - 1];
		FTransform ThisTransform = JointRefTransforms[i];
		
		// Compute offset in world (neutral frame), then convert to parent's local
		FVector DeltaWorld = ThisTransform.GetLocation() - ParentTransform.GetLocation();
		FVector OffsetInParentFrame = ParentTransform.InverseTransformVectorNoScale(DeltaWorld);
		
		// Store as pure translation (no rotation)
		JointLocalTransforms[i] = FTransform(FQuat::Identity, OffsetInParentFrame);
		
		if (bEnableDebugLogging && i < 7)
		{
			FString ParentName = (i == 0) ? TEXT("Base") : *Joints[i - 1].BoneName.ToString();
			FRotator ParentRot = ParentTransform.Rotator();
			UE_LOG(LogTemp, Log, TEXT("[Gen3] Joint[%d] '%s' offset from %s: (%.1f, %.1f, %.1f) cm, ParentRot=(P:%.1f,Y:%.1f,R:%.1f)"),
				i, *Joints[i].BoneName.ToString(), *ParentName, 
				OffsetInParentFrame.X, OffsetInParentFrame.Y, OffsetInParentFrame.Z,
				ParentRot.Pitch, ParentRot.Yaw, ParentRot.Roll);
		}
	}
	
	// ============================================================================
	// STEP 3: Compute Joint Local Axes in Parent Frame
	// ============================================================================
	// Each axis needs to be in the parent's local coordinate frame for FK
	// Frame1 from constraint gives us the axis in parent bone's local space
	
	TArray<FVector> JointAxesLocal;
	JointAxesLocal.SetNum(Joints.Num());
	
	for (int32 i = 0; i < Joints.Num(); i++)
	{
		FRevoluteJointConfig& Joint = Joints[i];
		FName ConstraintName = (Joint.ConstraintName != NAME_None) ? Joint.ConstraintName : Joint.BoneName;
		FConstraintInstance* Constraint = SkeletalMeshComponent->FindConstraintInstance(ConstraintName);
		
		FVector AxisLocal = FVector::XAxisVector; // Default
		
		if (Constraint)
		{
			FTransform Frame1 = Constraint->GetRefFrame(EConstraintFrame::Frame1);
			
			// Frame1 is in parent bone's local space
			// Extract the rotation axis based on which constraint axis we're using
			FVector AxisInFrame1;
			switch (Joint.ControlledAxis)
			{
			case EConstraintAxis::Swing1:
				AxisInFrame1 = FVector(0, 0, 1); // Z axis
				break;
			case EConstraintAxis::Swing2:
				AxisInFrame1 = FVector(0, 1, 0); // Y axis
				break;
			case EConstraintAxis::Twist:
			default:
				AxisInFrame1 = FVector(1, 0, 0); // X axis
				break;
			}
			
			// Rotate by Frame1's rotation to get axis in parent's local frame
			AxisLocal = Frame1.TransformVectorNoScale(AxisInFrame1).GetSafeNormal();
			
			// Apply inversion if configured
			if (Joint.bInvertAxisForIK)
			{
				AxisLocal = -AxisLocal;
			}
		}
		
		JointAxesLocal[i] = AxisLocal;
		
		if (bEnableDebugLogging && i < 7)
		{
			UE_LOG(LogTemp, Log, TEXT("[Gen3] Joint[%d] '%s' axis: (%.3f, %.3f, %.3f) Inverted=%d"),
				i, *Joint.BoneName.ToString(), AxisLocal.X, AxisLocal.Y, AxisLocal.Z, Joint.bInvertAxisForIK ? 1 : 0);
		}
	}
	
	// ============================================================================
	// STEP 4: Compute End Effector Offset
	// ============================================================================
	
	FTransform EndEffectorOffset = FTransform::Identity;
	if (EndEffectorBoneName != NAME_None)
	{
		FTransform LastJointRef = JointRefTransforms.Last();
		
		int32 EEBoneIdx = SkeletalMeshComponent->GetBoneIndex(EndEffectorBoneName);
		
		if (bEnableDebugLogging)
		{
			UE_LOG(LogTemp, Log, TEXT("[Gen3] EE '%s': IsBone=%d"), *EndEffectorBoneName.ToString(), EEBoneIdx != INDEX_NONE);
		}
		
		if (EEBoneIdx != INDEX_NONE)
		{
			// It's a bone - use reference skeleton
			FTransform EERefWorld = GetRefTransform(EEBoneIdx);
			EndEffectorOffset = EERefWorld.GetRelativeTransform(LastJointRef);
			
			if (bEnableDebugLogging)
			{
				UE_LOG(LogTemp, Log, TEXT("[Gen3]   LastJoint ref: Loc=(%.1f, %.1f, %.1f)"),
					LastJointRef.GetLocation().X, LastJointRef.GetLocation().Y, LastJointRef.GetLocation().Z);
				UE_LOG(LogTemp, Log, TEXT("[Gen3]   EE ref: Loc=(%.1f, %.1f, %.1f)"),
					EERefWorld.GetLocation().X, EERefWorld.GetLocation().Y, EERefWorld.GetLocation().Z);
			}
		}
		else
		{
			// It's a socket - get the socket's relative transform from its parent bone
			USkeletalMeshSocket const* Socket = SkeletalMeshComponent->GetSkeletalMeshAsset()->FindSocket(EndEffectorBoneName);
			if (Socket)
			{
				// Socket stores relative transform from its parent bone
				FName SocketParentBone = Socket->BoneName;
				int32 ParentBoneIdx = SkeletalMeshComponent->GetBoneIndex(SocketParentBone);
				
				if (bEnableDebugLogging)
				{
					UE_LOG(LogTemp, Log, TEXT("[Gen3]   Socket parent: '%s', BoneIdx=%d"), *SocketParentBone.ToString(), ParentBoneIdx);
					UE_LOG(LogTemp, Log, TEXT("[Gen3]   Socket local: Loc=(%.1f, %.1f, %.1f) Rot=(%.1f, %.1f, %.1f)"),
						Socket->RelativeLocation.X, Socket->RelativeLocation.Y, Socket->RelativeLocation.Z,
						Socket->RelativeRotation.Pitch, Socket->RelativeRotation.Yaw, Socket->RelativeRotation.Roll);
				}
				
				if (ParentBoneIdx != INDEX_NONE)
				{
					// Get parent bone's reference transform
					FTransform ParentRefWorld = GetRefTransform(ParentBoneIdx);
					
					// Socket's local transform relative to parent bone
					FTransform SocketLocal(Socket->RelativeRotation, Socket->RelativeLocation, Socket->RelativeScale);
					
					// Transform socket to world via parent
					FTransform SocketRefWorld = SocketLocal * ParentRefWorld;
					
					// Get offset from last joint to socket
					EndEffectorOffset = SocketRefWorld.GetRelativeTransform(LastJointRef);
					
					if (bEnableDebugLogging)
					{
						UE_LOG(LogTemp, Log, TEXT("[Gen3]   Socket parent ref: Loc=(%.1f, %.1f, %.1f)"),
							ParentRefWorld.GetLocation().X, ParentRefWorld.GetLocation().Y, ParentRefWorld.GetLocation().Z);
						// UE_LOG(LogTemp, Log, TEXT("[Gen3]   Socket ref world: Loc=(%.1f, %.1f, %.1f)"),
						// 	SocketRefWorld.GetLocation().X, SocketRefWorld.GetLocation().Y, SocketRefWorld.GetLocation().Z);
						// UE_LOG(LogTemp, Log, TEXT("[Gen3]   LastJoint ref: Loc=(%.1f, %.1f, %.1f)"),
						// 	LastJointRef.GetLocation().X, LastJointRef.GetLocation().Y, LastJointRef.GetLocation().Z);
					}
				}
				else
				{
					// Socket parent not found, use socket's local transform
					EndEffectorOffset = FTransform(Socket->RelativeRotation, Socket->RelativeLocation, Socket->RelativeScale);
				}
			}
		}
		
		if (bEnableDebugLogging)
		{
			FVector EEOffset = EndEffectorOffset.GetLocation();
			FRotator EERot = EndEffectorOffset.Rotator();
			UE_LOG(LogTemp, Log, TEXT("[Gen3] EE offset (final): Loc=(%.1f, %.1f, %.1f) Rot=(%.1f, %.1f, %.1f)"),
				EEOffset.X, EEOffset.Y, EEOffset.Z, EERot.Pitch, EERot.Yaw, EERot.Roll);
		}
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
	
	// Get current joint angles (in degrees)
	TArray<float> CurrentAngles;
	TArray<FVector2D> JointLimits;
	CurrentAngles.SetNum(Joints.Num());
	JointLimits.SetNum(Joints.Num());
	
	for (int32 i = 0; i < Joints.Num(); i++)
	{
		CurrentAngles[i] = Joints[i].CurrentAngle;
		JointLimits[i] = FVector2D(Joints[i].MinAngleLimit, Joints[i].MaxAngleLimit);
	}
	
	// ============================================================================
	// STEP 6: Verify FK Matches Reality
	// ============================================================================
	
	if (bEnableDebugLogging)
	{
		// Log current angles being used for FK
		UE_LOG(LogTemp, Log, TEXT("[Gen3] FK angles (deg):"));
		for (int32 i = 0; i < FMath::Min(7, CurrentAngles.Num()); i++)
		{
			UE_LOG(LogTemp, Log, TEXT("  [%d] %.2f°"), i, CurrentAngles[i]);
		}
		
		FTransform FK_EE = URammsIKLibrary::ComputeForwardKinematics(
			BaseTransform, CurrentAngles, JointLocalTransforms, JointAxesLocal, EndEffectorOffset, false);
		
		FTransform Actual_EE = SkeletalMeshComponent->GetSocketTransform(EndEffectorBoneName, RTS_World);
		
		FVector Diff = Actual_EE.GetLocation() - FK_EE.GetLocation();
		
		UE_LOG(LogTemp, Log, TEXT("[Gen3] FK Check:"));
		UE_LOG(LogTemp, Log, TEXT("  Base: (%.1f, %.1f, %.1f)"),
			BaseTransform.GetLocation().X, BaseTransform.GetLocation().Y, BaseTransform.GetLocation().Z);
		UE_LOG(LogTemp, Log, TEXT("  FK EE:     (%.1f, %.1f, %.1f)"),
			FK_EE.GetLocation().X, FK_EE.GetLocation().Y, FK_EE.GetLocation().Z);
		UE_LOG(LogTemp, Log, TEXT("  Actual EE: (%.1f, %.1f, %.1f)"),
			Actual_EE.GetLocation().X, Actual_EE.GetLocation().Y, Actual_EE.GetLocation().Z);
		UE_LOG(LogTemp, Log, TEXT("  Error: %.1f cm"), Diff.Size());
	}
	
	// ============================================================================
	// STEP 7: Solve IK
	// ============================================================================
	
	FIKSolveResult IKResult = URammsIKLibrary::SolveIK_DLS(
		CurrentAngles,
		BaseTransform,
		JointLocalTransforms,
		JointAxesLocal,
		EndEffectorOffset,
		TargetEndEffectorTransform,
		JointLimits,
		TaskSpaceMask,
		JointWeights,
		IKDampingFactor,
		IKStepClip,
		MaxIKIterations,
		IKPositionTolerance,
		IKRotationTolerance,
		NullSpaceBias,
		bEnableNullSpaceOptimization ? NullSpaceGain : 0.0f);
	
	// Apply IK results to joints
	for (int32 i = 0; i < FMath::Min(IKResult.JointAngles.Num(), Joints.Num()); i++)
	{
		Joints[i].TargetAngle = IKResult.JointAngles[i];
	}
	
	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[Gen3] IK: Success=%d, PosErr=%.1f cm, RotErr=%.1f deg, Iter=%d"),
			IKResult.bSuccess, IKResult.PositionError, IKResult.RotationError, IKResult.IterationsUsed);
	}
}
