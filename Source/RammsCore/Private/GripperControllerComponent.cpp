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

namespace
{
	/** Physical constraint angle (degrees) for a finger motor's controlled axis. */
	float GetMotorActualAngleDeg(const FAngularMotorConfig& Motor)
	{
		FConstraintInstance* Constraint = Motor.CachedConstraint;
		if (!Constraint)
		{
			return Motor.CurrentAngle;
		}

		float AngleRad = 0.0f;
		switch (Motor.ControlAxis)
		{
			default:
			case EMotorAxis::Z:
				AngleRad = Constraint->GetCurrentSwing1();
				break;
			case EMotorAxis::Y:
				AngleRad = Constraint->GetCurrentSwing2();
				break;
			case EMotorAxis::X:
				AngleRad = Constraint->GetCurrentTwist();
				break;
		}
		return FMath::RadiansToDegrees(AngleRad);
	}

	/** True while a finger still needs to move: software setpoint mid-interpolation, or the
	 *  physical joint not yet within tolerance of its (direction-corrected) target. */
	bool FingerNeedsMotion(const FAngularMotorConfig& Motor, float AngleTolerance)
	{
		if (!Motor.bEnabled)
		{
			return false;
		}

		if (FMath::Abs(FMath::FindDeltaAngleDegrees(Motor.CurrentAngle, Motor.TargetAngle)) > KINDA_SMALL_NUMBER)
		{
			return true; // software ramp still in progress
		}

		const float TargetEffective = Motor.bInvertDirection ? -Motor.TargetAngle : Motor.TargetAngle;
		return FMath::Abs(FMath::FindDeltaAngleDegrees(GetMotorActualAngleDeg(Motor), TargetEffective)) > AngleTolerance;
	}
} // namespace

void UGripperControllerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UpdateMotors(DeltaTime);
	UpdateGripperState();
	HandleStateChange();

	// Same Chaos auto-sleep issue as the arm: idle finger bodies fall asleep and then ignore
	// the motor drive target, so an Open()/Close() issued while at rest changes state but the
	// fingers don't physically move until something else (e.g. arm motion) wakes them. Keep the
	// gripper bodies awake while either finger still needs to move (and while grasping, where the
	// drive must keep holding force against the object).
	if (IsValid(CachedGripperMesh)
		&& (FingerNeedsMotion(Finger1Motor, AngleTolerance) || FingerNeedsMotion(Finger2Motor, AngleTolerance)))
	{
		CachedGripperMesh->WakeAllRigidBodies();
	}
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

void UGripperControllerComponent::SetFingerDriveParams(float Strength, float Damping, float MaxForce)
{
	Finger1Motor.MotorStrength = FMath::Max(0.0f, Strength);
	Finger1Motor.MotorDamping = FMath::Max(0.0f, Damping);
	Finger1Motor.MaxForce = FMath::Max(0.0f, MaxForce);
	Finger2Motor.MotorStrength = Finger1Motor.MotorStrength;
	Finger2Motor.MotorDamping = Finger1Motor.MotorDamping;
	Finger2Motor.MaxForce = Finger1Motor.MaxForce;

	// Apply immediately so live tuning takes effect without waiting for the next motor update.
	ApplyMotorSettings(Finger1Motor);
	ApplyMotorSettings(Finger2Motor);

	if (bEnableDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("GripperController: drive params set strength=%.0f damping=%.0f maxforce=%.0f"),
			Finger1Motor.MotorStrength, Finger1Motor.MotorDamping, Finger1Motor.MaxForce);
	}
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
	// While both fingers are closing, clamp each finger's command so it can't lead the SLOWER
	// finger's physical angle by more than FingerSyncLeadDegrees. Keeps the grasp centered.
	const float CloseDir = FMath::Sign(ClosedAngle - OpenAngle);
	const bool	bClosing1 = FMath::Abs(Finger1Motor.TargetAngle - ClosedAngle) <= FMath::Abs(Finger1Motor.TargetAngle - OpenAngle);
	const bool	bClosing2 = FMath::Abs(Finger2Motor.TargetAngle - ClosedAngle) <= FMath::Abs(Finger2Motor.TargetAngle - OpenAngle);

	// Only sync when BOTH fingers can actually move. If one motor is disabled or missing its
	// constraint, AdvanceMotor early-outs for it (its physical angle never advances), so it would
	// register as the "slowest" finger and wrongly clamp the enabled finger from closing/opening.
	const bool bBothMotorsDrivable =
		Finger1Motor.bEnabled && Finger1Motor.CachedConstraint && Finger2Motor.bEnabled && Finger2Motor.CachedConstraint;

	bool  bSync = false;
	float SyncMaxCommand = 0.0f;
	if (bSyncFingersWhileClosing && bBothMotorsDrivable && bClosing1 && bClosing2 && !FMath::IsNearlyZero(CloseDir))
	{
		// Physical angles expressed in each finger's own command frame (closing is +CloseDir).
		const float Actual1 = (Finger1Motor.bInvertDirection ? -1.0f : 1.0f) * GetMotorActualAngleDeg(Finger1Motor);
		const float Actual2 = (Finger2Motor.bInvertDirection ? -1.0f : 1.0f) * GetMotorActualAngleDeg(Finger2Motor);
		const float SlowestClosing = (CloseDir > 0.0f) ? FMath::Min(Actual1, Actual2) : FMath::Max(Actual1, Actual2);
		SyncMaxCommand = SlowestClosing + CloseDir * FMath::Max(0.0f, FingerSyncLeadDegrees);
		bSync = true;
	}

	AdvanceMotor(Finger1Motor, DeltaTime, bSync, SyncMaxCommand);
	AdvanceMotor(Finger2Motor, DeltaTime, bSync, SyncMaxCommand);
}

void UGripperControllerComponent::AdvanceMotor(FAngularMotorConfig& Motor, float DeltaTime, bool bApplySyncClamp, float SyncMaxCommand)
{
	if (!Motor.bEnabled || !Motor.CachedConstraint)
	{
		return;
	}

	// Are we closing (toward ClosedAngle) or opening (toward OpenAngle)? Closing is the gentle,
	// force-capped, stall-aware grasp; opening is always fast and at full force so the gripper
	// can always release — gentle/capped closing settings must never trap the fingers shut.
	const bool bClosing = FMath::Abs(Motor.TargetAngle - ClosedAngle) <= FMath::Abs(Motor.TargetAngle - OpenAngle);
	float	   Speed = bClosing ? Motor.MaxSpeed : OpenSpeed;
	// Ease onto the object near full closure for a soft seat instead of slamming.
	if (bClosing && FinalApproachSpeed > 0.0f && FinalApproachBandDegrees > 0.0f
		&& FMath::Abs(Motor.CurrentAngle - ClosedAngle) <= FinalApproachBandDegrees)
	{
		Speed = FMath::Min(Speed, FinalApproachSpeed);
	}

	// Ramp the commanded angle toward the target at the chosen speed.
	const float MaxDelta = Speed * Motor.SpeedMultiplier * DeltaTime;
	const float AngleDiff = Motor.TargetAngle - Motor.CurrentAngle;
	float		Proposed = (FMath::Abs(AngleDiff) > MaxDelta)
			  ? Motor.CurrentAngle + FMath::Sign(AngleDiff) * MaxDelta
			  : Motor.TargetAngle;

	// Stall-aware clamp (closing only): if the finger physically can't keep up with the command
	// (blocked by a grasped object), stop the command running ahead and driving the pad through
	// the object. Hold the command a small lead past the physical angle so the drive keeps a
	// bounded squeeze at the contact (pair with a finite Motor.MaxForce).
	if (bClosing && bStallAwareClosing && StallLeadDegrees > 0.0f)
	{
		// Key the clamp on the CLOSING direction (OpenAngle -> ClosedAngle), not on
		// Sign(Target - Current). The old gate went to zero the instant the software ramp
		// reached ClosedAngle, switching the clamp off right when a near-fully-closed finger
		// is blocked by the object - so the drive commanded ClosedAngle against the block and
		// chattered. Keying on the closing direction keeps the clamp active for the whole
		// closing phase.
		const float CloseDir = FMath::Sign(ClosedAngle - OpenAngle);
		if (!FMath::IsNearlyZero(CloseDir))
		{
			const float RawActual = GetMotorActualAngleDeg(Motor);
			const float ActualInMotorFrame = Motor.bInvertDirection ? -RawActual : RawActual;
			// Never let the command lead the physical finger by more than StallLeadDegrees in
			// the closing direction. While the finger moves freely Actual tracks the command so
			// this never bites; once the finger stalls on the object the command is pinned to
			// (Actual + StallLead), giving a bounded, tracking squeeze instead of ramming shut.
			const float MaxCommand = ActualInMotorFrame + CloseDir * StallLeadDegrees;
			if ((Proposed - MaxCommand) * CloseDir > 0.0f)
			{
				Proposed = MaxCommand;
			}
		}
	}

	// Cross-finger sync clamp (closing only): never lead the slower finger by more than the bound.
	if (bClosing && bApplySyncClamp)
	{
		const float SyncCloseDir = FMath::Sign(ClosedAngle - OpenAngle);
		if (!FMath::IsNearlyZero(SyncCloseDir) && (Proposed - SyncMaxCommand) * SyncCloseDir > 0.0f)
		{
			Proposed = SyncMaxCommand;
		}
	}

	Motor.CurrentAngle = Proposed;
	// Opening uses full (unlimited) drive force; closing uses the configured squeeze cap
	// (GripHoldForce when set, else the finger's own MaxForce via the -1 sentinel).
	const float ClosingForceLimit = (GripHoldForce > 0.0f) ? GripHoldForce : -1.0f;
	ApplyMotorSettings(Motor, bClosing ? ClosingForceLimit : 0.0f);
}

void UGripperControllerComponent::ApplyMotorSettings(FAngularMotorConfig& Motor, float ForceLimitOverride)
{
	if (!Motor.CachedConstraint)
	{
		return;
	}

	// Enable/disable angular drive
	Motor.CachedConstraint->SetOrientationDriveTwistAndSwing(Motor.bEnabled, Motor.bEnabled);

	if (Motor.bEnabled)
	{
		// Set motor strength, damping, and force limit. MaxForce 0 = unlimited; a finite cap
		// gives a compliant grasp that won't ram through the object or break the finger joints.
		// ForceLimitOverride >= 0 replaces the cap (e.g. 0 = unlimited force while opening).
		const float ForceLimit = (ForceLimitOverride >= 0.0f) ? ForceLimitOverride : Motor.MaxForce;
		Motor.CachedConstraint->SetAngularDriveParams(Motor.MotorStrength, Motor.MotorDamping, ForceLimit);

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
			// DIAGNOSTIC: log ALL three constraint axes so we can see which one the finger
			// physically rotates on. The axis whose value changes as the finger moves is the
			// real DOF and is what GetMotorActualAngleDeg must read.
			const float TwistDeg = FMath::RadiansToDegrees(Motor.CachedConstraint->GetCurrentTwist());
			const float Swing1Deg = FMath::RadiansToDegrees(Motor.CachedConstraint->GetCurrentSwing1());
			const float Swing2Deg = FMath::RadiansToDegrees(Motor.CachedConstraint->GetCurrentSwing2());

			UE_LOG(LogTemp, Warning, TEXT("GripperController: %s - Target=%.1f Current=%.1f | Twist=%.2f Swing1=%.2f Swing2=%.2f | Inverted=%d"),
				*Motor.ConstraintName.ToString(), Motor.TargetAngle, Motor.CurrentAngle, TwistDeg, Swing1Deg, Swing2Deg, Motor.bInvertDirection);
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
