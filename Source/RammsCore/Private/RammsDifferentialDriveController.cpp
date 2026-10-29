// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsDifferentialDriveController.h"

#include "RammsControlIds.h"
#include "RammsDifferentialDriveLibrary.h"
#include "RammsDriveHandOver.h"
#include "RammsRobotBaseComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/BodyInstance.h"
#include "DrawDebugHelpers.h"

URammsDifferentialDriveController::URammsDifferentialDriveController()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;

	// Centre wheels down and carrying load, which is where the 5-bar sits when
	// the robot is standing on them.
	PlantedStance.bHasLinkageHeight = true;
	PlantedStance.LinkageHeightCm = 12.7f;

	Carry = MakePimpl<RammsDrive::FCarry>();
	HoldLeft = MakePimpl<RammsDrive::FTorqueHold>();
	HoldRight = MakePimpl<RammsDrive::FTorqueHold>();
	LeftAngleSource = MakePimpl<RammsDrive::FWheelAngle>();
	RightAngleSource = MakePimpl<RammsDrive::FWheelAngle>();
}

void URammsDifferentialDriveController::BeginPlay()
{
	Super::BeginPlay();

	// Find the skeletal mesh component
	AActor* Owner = GetOwner();
	if (Owner)
	{
		// A sibling robot base component, when present, owns the physics I/O:
		// wheel state, torque, and braking route through it by motor Id (Chaos or
		// MuJoCo). Without one, the controller keeps the direct Chaos wheel-bone
		// path below, so existing blueprints keep driving.
		BaseComponent = Owner->FindComponentByClass<URammsRobotBaseComponent>();
		if (BaseComponent)
		{
			UE_LOG(LogTemp, Log,
				TEXT("[DiffDrive] Driving via RammsRobotBaseComponent '%s' (motors '%s' / '%s')."),
				*BaseComponent->GetName(), *LeftMotorId.ToString(), *RightMotorId.ToString());
			// A robot has one base component; with several, "the first" is an
			// arbitrary (and easily unconfigured) one — say so rather than drive
			// through it silently.
			TArray<URammsRobotBaseComponent*> Bases;
			Owner->GetComponents<URammsRobotBaseComponent>(Bases);
			if (Bases.Num() > 1)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("[DiffDrive] '%s' has %d RammsRobotBaseComponents; using '%s'. Remove the extras."),
					*Owner->GetName(), Bases.Num(), *BaseComponent->GetName());
			}
		}

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

		if (!SkeletalMeshComponent && !BaseComponent)
		{
			// Only a problem on the direct Chaos path; a base-component robot
			// (e.g. a MuJoCo articulation) has no wheel skeletal mesh to find.
			UE_LOG(LogTemp, Warning, TEXT("RammsDifferentialDriveController: Failed to find skeletal mesh component on %s"), *Owner->GetName());
		}
		else
		{
			// Set max angular velocity on wheel body instances (through the
			// base's Chaos mapping when a base drives this robot)
			FBodyInstance* LeftBodyInst = GetWheelBody(LeftMotorId, LeftWheelBoneName);
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

			FBodyInstance* RightBodyInst = GetWheelBody(RightMotorId, RightWheelBoneName);
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

	// Once, when the base can report it: the measured drive-motor separation,
	// projected onto the robot's lateral axis (a track width is the distance
	// between the wheel centrelines, not the 3D distance between staggered or
	// unequal-height motors). Logged for comparison with TrackWidth; adopted
	// only if opted in.
	if (!bTrackWidthMeasured && UsesBase())
	{
		FTransform LeftXf, RightXf;
		if (GetOwner() && BaseComponent->GetMotorTransform(LeftMotorId, LeftXf) && BaseComponent->GetMotorTransform(RightMotorId, RightXf))
		{
			const FVector Delta = RightXf.GetLocation() - LeftXf.GetLocation();
			const float	  Lateral = FMath::Abs(FVector::DotProduct(Delta, GetOwner()->GetActorRightVector()));
			if (Lateral > 0.0f)
			{
				bTrackWidthMeasured = true;
				UE_LOG(LogTemp, Log, TEXT("[DiffDrive] Measured lateral drive-motor separation %.2f cm (TrackWidth %.2f cm)%s"),
					Lateral, TrackWidth, bUseMotorSeparationAsTrackWidth ? TEXT(" — adopting as TrackWidth.") : TEXT("."));
				if (bUseMotorSeparationAsTrackWidth)
				{
					TrackWidth = Lateral;
				}
			}
		}
	}

	// Update wheel states
	UpdateWheelState(LeftMotorId, LeftWheelBoneName, LeftWheelState);
	UpdateWheelState(RightMotorId, RightWheelBoneName, RightWheelState);

	const double T = Now();
	double		 Dt = 0.0;
	if (RammsDrive::AdvanceHoldClock(T, FirstTickTime, LastTickTime, Dt))
	{
		// The simulation was reset: the robot is back at its start pose, every
		// wheel's angle with it, and nothing held before stands.
		ResetHoldState();
	}
	// Each drive wheel's angle, for holding it still: the backend's reading
	// while it follows the wheel's rate, else the rate integrated.
	const bool bCanHold = CanHoldWithTorque();
	if (bCanHold)
	{
		LeftAngle = LeftAngleSource->Update(BaseComponent->GetMotorValue(LeftMotorId), LeftWheelState.AngularVelocity, Dt);
		RightAngle = RightAngleSource->Update(BaseComponent->GetMotorValue(RightMotorId), RightWheelState.AngularVelocity, Dt);
	}
	{
		const double Share = Dt > 0.0 ? FMath::Min(1.0, Dt / HoldRateSmoothSeconds) : 1.0;
		SmoothLeftRate += Share * (LeftWheelState.AngularVelocity - SmoothLeftRate);
		SmoothRightRate += Share * (RightWheelState.AngularVelocity - SmoothRightRate);
	}
	if (ReleaseElapsed >= 0.0)
	{
		ReleaseElapsed += Dt;
		if (ReleaseElapsed >= HoldReleaseSeconds)
		{
			ReleaseElapsed = -1.0;
		}
	}
	CarryWeight = 0.0f;
	bool bCarryGrace = false;
	if (bDriveModeActive && Carry && Carry->bActive)
	{
		// A motion handed over on a switch: held while the stance changes and
		// the operator holds the stick where it was, then faded out (see
		// AddCarryTorque). The first tick is the operator's to be heard on.
		bCarryGrace = bFirstCarryTick;
		bFirstCarryTick = false;
		const bool bCommanding = IsOperatorCommanding();
		if (bCommanding && !bCarryInputSet)
		{
			CarryInput = DriveInput;
			bCarryInputSet = true;
		}
		if (!bCommanding && !bCarryGrace)
		{
			// Nobody driving: speed carries across a switch, it does not keep
			// a robot moving that nobody is driving.
			EndCarry();
		}
		else
		{
			const bool bAsBefore = !bCarryInputSet || (DriveInput - CarryInput).GetAbsMax() <= CarryInputTolerance;
			Carry->Step(Dt, bCarryGrace || bAsBefore);
			CarryWeight = Carry->bHolding ? 1.0f : static_cast<float>(1.0 - RammsDrive::CarryBlend(Carry->SinceHold, CarryFadeSeconds));
			if (CarryWeight <= 0.0f)
			{
				EndCarry();
			}
		}
	}

	// Actuate only while this is the live drive mode. Zeroing the input on
	// stand-down is not enough on its own: the braking and control paths run
	// every frame regardless of what was commanded, so an inactive controller
	// went on writing its motors -- over the holonomic mode's wheels, and over
	// the raw per-motor axes the low-level mode hands out. Wheel state and
	// odometry above and below this still update, so the estimate stays live
	// while another mode drives.
	//
	// Holding still (bHoldWhenIdle): with the input at centre each drive wheel
	// braked to rest and held where it stopped -- not while its wheels are
	// still going down (HoldForStance: in the air the hold's gains, set for a
	// loaded wheel, would spin it), and not on a carry's first tick, which is
	// its operator's to be heard on. The input off centre lets go, the holding
	// torque fading into the control mode's.
	const bool bHold = bDriveModeActive && bHoldWhenIdle && bCanHold && RammsDrive::HoldWhenIdleEnabled() && !bHeldForStance
		&& !IsOperatorCommanding() && !bCarryGrace && T - FirstTickTime >= RammsDrive::SpawnSettleSeconds;
	if (!bHold && !(bHoldInactive && !bDriveModeActive) && (HoldLeft->bActive || HoldRight->bActive))
	{
		ReleaseHold();
	}
	// No torque hold on this backend (CanHoldWithTorque): park on the drive
	// wheels' brakes instead once the robot has slowed at centre, under the
	// same conditions the hold has; driving releases them first.
	const bool bWantPark = bDriveModeActive && bHoldWhenIdle && !bCanHold && UsesBase() && RammsDrive::HoldWhenIdleEnabled()
		&& !bHeldForStance && !IsOperatorCommanding() && !bCarryGrace && T - FirstTickTime >= RammsDrive::SpawnSettleSeconds;
	if (!bWantPark)
	{
		SetParkingBrake(false);
		ParkFromSpeed = 0.0f;
	}
	else if (!bParkingBrake)
	{
		// Closed once the treads are slow, or the moment the robot turns back
		// against the way it was going when let go: stopped on a slope, it
		// passes through rest in a frame and gravity takes it back down, often
		// never as slowly as ParkingBrakeSpeed again with the braking against it.
		const float Forward = 0.5f * (LeftWheelState.LinearVelocity + RightWheelState.LinearVelocity);
		if (ParkFromSpeed == 0.0f)
		{
			ParkFromSpeed = FMath::Abs(Forward) > ParkingBrakeSpeed ? Forward : SMALL_NUMBER;
		}
		const bool bSlow = FMath::Max(FMath::Abs(LeftWheelState.LinearVelocity), FMath::Abs(RightWheelState.LinearVelocity)) < ParkingBrakeSpeed;
		const bool bTurnedBack = FMath::Abs(ParkFromSpeed) > ParkingBrakeSpeed && Forward * ParkFromSpeed < 0.0f;
		if (bSlow || bTurnedBack)
		{
			SetParkingBrake(true);
		}
	}
	if (bParkingBrake)
	{
		// Held by the brakes: no wheel torque to fight them.
		bIsBraking = true;
		LeftWheelState.AppliedTorque = 0.0f;
		RightWheelState.AppliedTorque = 0.0f;
	}
	else if (bHold)
	{
		if (!HoldLeft->bActive || !HoldRight->bActive)
		{
			// Braked straight: both sides from the robot's forward rate (their
			// mean), not each from its own -- with nobody driving, a turn the
			// robot was making stops at once rather than running on while it
			// brakes. Begun on a hand-back that told it the ground
			// (SeedGroundSlope), the slope's pull is held from the start, not
			// found as the robot sags back, and the braking is eased to rest
			// (FTorqueHold::bEaseToRest): stopped short, the loop rolls the
			// robot back a little after it stops.
			const double Forward = 0.5 * (SmoothLeftRate + SmoothRightRate);
			const bool	 bHandedOver = SeedExpires >= 0.0 && T <= SeedExpires;
			const double Seed = bHandedOver ? SeedTorque : 0.0;
			BeginHold(Forward, Forward, Seed, Seed, bHandedOver);
			SeedExpires = -1.0;
			ReleaseElapsed = -1.0;
		}
		bIsBraking = true;
		StepHold(Dt);
	}
	else if (bDriveModeActive && bHeldForStance)
	{
		// Its wheels are still going down (HoldForStance): drive nothing. A
		// tread spun up in the air touches down spinning.
		bIsBraking = false;
		ApplyWheelTorque(LeftMotorId, LeftWheelBoneName, 0.0f, LeftMotorParams, LeftWheelState);
		ApplyWheelTorque(RightMotorId, RightWheelBoneName, 0.0f, RightMotorParams, RightWheelState);
	}
	else if (bDriveModeActive)
	{
		// Check for braking -- not on a carry's first tick, which is its
		// operator's to be heard on. Parking (bWantPark) brakes to rest first,
		// auto-braking or not, as a power chair decelerates before its brakes
		// close: left to roll, a robot let go going down a ramp never slowed
		// to ParkingBrakeSpeed.
		bIsBraking = !bCarryGrace && (ShouldApplyBrakes() || bWantPark);

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
	}
	else if (bHoldInactive)
	{
		// Stood down at rest while the next mode's wheels go down: these still
		// carry the robot, held where they are (again where a reset put them).
		if (!HoldLeft->bActive || !HoldRight->bActive)
		{
			BeginHold(SmoothLeftRate, SmoothRightRate, 0.0, 0.0);
		}
		bIsBraking = true;
		StepHold(Dt);
	}
	else
	{
		bIsBraking = false;
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

bool URammsDifferentialDriveController::UsesBase() const
{
	return BaseComponent && BaseComponent->HasBackend();
}

FBodyInstance* URammsDifferentialDriveController::GetWheelBody(FName MotorId, FName BoneName)
{
	if (UsesBase())
	{
		if (USkeletalMeshComponent* BaseMesh = BaseComponent->GetChaosSkeletalMesh())
		{
			const FName ChaosBone = BaseComponent->GetChaosName(MotorId);
			return ChaosBone.IsNone() ? nullptr : BaseMesh->GetBodyInstance(ChaosBone);
		}
		return nullptr;
	}
	return GetBoneBodyInstance(BoneName);
}

FBodyInstance* URammsDifferentialDriveController::GetBoneBodyInstance(FName BoneName)
{
	if (!SkeletalMeshComponent || BoneName == NAME_None)
	{
		return nullptr;
	}

	return SkeletalMeshComponent->GetBodyInstance(BoneName);
}

void URammsDifferentialDriveController::UpdateWheelState(FName MotorId, FName BoneName, FRammsWheelState& OutState)
{
	// The contact-dependent fields (lateral velocity, load, surface friction,
	// slip) need the wheel's simulating Chaos body; the generic motor interface
	// doesn't expose contact data and MuJoCo resolves contact itself. On the
	// base path the body is the one the base's Chaos mapping names (its mesh
	// and the motor's ChaosName), not this controller's legacy bone fields.
	USkeletalMeshComponent* ContactMesh = SkeletalMeshComponent;
	FName					ContactBone = BoneName;
	if (UsesBase())
	{
		if (USkeletalMeshComponent* BaseMesh = BaseComponent->GetChaosSkeletalMesh())
		{
			ContactMesh = BaseMesh;
		}
		ContactBone = BaseComponent->GetChaosName(MotorId);
	}
	FBodyInstance* BodyInst = (ContactMesh && ContactBone != NAME_None) ? ContactMesh->GetBodyInstance(ContactBone) : nullptr;
	const bool	   bHaveBody = BodyInst && BodyInst->IsInstanceSimulatingPhysics();
	const int32	   BoneIndex = bHaveBody ? ContactMesh->GetBoneIndex(ContactBone) : INDEX_NONE;

	if (UsesBase())
	{
		// Base-component path: the wheel joint velocity comes by motor Id.
		OutState.AngularVelocity = BaseComponent->GetMotorVelocity(MotorId);
		OutState.LinearVelocity = OutState.AngularVelocity * WheelRadius;
		if (BoneIndex == INDEX_NONE)
		{
			// No Chaos wheel body to read contact from (MuJoCo, or no bone
			// mapping): neutral contact state, so traction modeling is a
			// pass-through. Say so once if it was asked for.
			OutState.LateralVelocity = 0.0f;
			OutState.SlipRatio = 0.0f;
			OutState.SuspensionLoad = 0.0f;
			OutState.SurfaceFriction = 1.0f;
			if (bEnableSlipModeling && !bWarnedNoContactBody)
			{
				bWarnedNoContactBody = true;
				UE_LOG(LogTemp, Warning, TEXT("[DiffDrive] '%s': slip modeling is enabled but wheel '%s' (motor '%s') has no simulating Chaos body to read contact from — traction modeling is bypassed on this backend."),
					*GetName(), *ContactBone.ToString(), *MotorId.ToString());
			}
			return;
		}
	}
	else
	{
		if (BoneIndex == INDEX_NONE)
		{
			return;
		}

		// Get angular velocity in radians per second
		// Assuming wheel spins around its local Y axis (standard for UE wheel components)
		FVector AngularVelocity = BodyInst->GetUnrealWorldAngularVelocityInRadians();
		// Relative to the body the wheel is mounted on, as an encoder reads it
		// (see FRammsChaosActuationBackend::GetVelocity): the chassis pitching
		// about the axle is not the wheel turning.
		for (FName Mount = ContactMesh->GetParentBone(ContactBone); Mount != NAME_None; Mount = ContactMesh->GetParentBone(Mount))
		{
			const FBodyInstance* MountBody = ContactMesh->GetBodyInstance(Mount);
			if (MountBody && MountBody->IsInstanceSimulatingPhysics())
			{
				AngularVelocity -= MountBody->GetUnrealWorldAngularVelocityInRadians();
				break;
			}
		}

		// Convert to local space to extract the rotation speed around the
		// wheel's spin axis (typically Y axis)
		FTransform SpinTransform = ContactMesh->GetBoneTransform(BoneIndex);
		FVector	   LocalAngularVelocity = SpinTransform.InverseTransformVectorNoScale(AngularVelocity);
		OutState.AngularVelocity = LocalAngularVelocity.Y;

		// Calculate linear velocity at contact point
		OutState.LinearVelocity = OutState.AngularVelocity * WheelRadius;
	}

	// Contact state from the Chaos wheel body (both paths from here on).
	FTransform BoneTransform = ContactMesh->GetBoneTransform(BoneIndex);

	// Get actual linear velocity of the wheel
	FVector WheelVelocity = BodyInst->GetUnrealWorldVelocity();
	FVector ForwardDir = BoneTransform.GetUnitAxis(EAxis::X);
	FVector RightDir = BoneTransform.GetUnitAxis(EAxis::Y);
	float	ActualLinearVel = FVector::DotProduct(WheelVelocity, ForwardDir);

	// Calculate lateral (sideways) velocity for slip resistance
	OutState.LateralVelocity = FVector::DotProduct(WheelVelocity, RightDir);

	// Calculate suspension load (normal force on wheel)
	if (bEnableLoadDependentTraction)
	{
		// Estimate load based on vehicle mass and vertical acceleration
		// Assuming equal weight distribution between wheels
		float GravityForce = VehicleMass * 980.665f;   // Convert kg to Newtons (g = 9.80665 m/s²)
		OutState.SuspensionLoad = GravityForce * 0.5f; // Split between two wheels

		// TODO: Could improve by reading actual suspension compression/force
	}
	else
	{
		OutState.SuspensionLoad = 0.0f;
	}

	// Query surface friction from physical material
	OutState.SurfaceFriction = 1.0f; // Default
	if (bUsePhysicalMaterialFriction)
	{
		// Raycast down from wheel to find ground material
		FVector WheelLocation = BoneTransform.GetLocation();
		FVector TraceStart = WheelLocation;
		FVector TraceEnd = WheelLocation - FVector(0, 0, WheelRadius * 2.0f);

		FHitResult			  HitResult;
		FCollisionQueryParams QueryParams;
		QueryParams.AddIgnoredActor(GetOwner());

		if (GetWorld()->LineTraceSingleByChannel(HitResult, TraceStart, TraceEnd, ECC_Visibility, QueryParams))
		{
			if (HitResult.PhysMaterial.IsValid())
			{
				OutState.SurfaceFriction = HitResult.PhysMaterial->Friction;
			}
		}
	}

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
			// Oppose the turn: a positive (clockwise / right) yaw rate comes from
			// the left wheel outrunning the right, so take torque from the
			// left and give it to the right.
			float TurnDirection = FMath::Sign(CurrentTurningSpeed);
			LeftTorque -= TurnDirection * TurnDampingTorque;
			RightTorque += TurnDirection * TurnDampingTorque;

			if (bEnableDebugLogging)
			{
				UE_LOG(LogTemp, Log, TEXT("[DiffDrive] Damping turn: %.1f deg/s (max: %.1f), damping torque: %.2f Nm"),
					AbsTurningSpeed, MaxTurningSpeed, TurnDampingTorque);
			}
		}
	}

	// Apply resistive torque that opposes motion (simulates back-EMF, friction, drag).
	// Not below the brakes' deadband (ApplyBrakes): a parked robot's wheels read a
	// few hundredths of a rad/s of solver noise, and answering it put a torque on
	// them every frame, which kept the robot from ever coming to rest (each torque
	// wakes it; it nearly slept and was jolted awake again, rocking at ~2-4 Hz).
	constexpr float ResistiveDeadband = 0.1f; // rad/s
	const auto		Resistive = [this](float AngVel) {
		return FMath::Abs(AngVel) > ResistiveDeadband ? -AngVel * ResistiveTorqueCoefficient : 0.0f;
	};
	float LeftResistiveTorque = Resistive(LeftWheelState.AngularVelocity);
	float RightResistiveTorque = Resistive(RightWheelState.AngularVelocity);

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
	AddCarryTorque(LeftTorque, RightTorque);
	AddReleaseTorque(LeftTorque, RightTorque);
	ApplyWheelTorque(LeftMotorId, LeftWheelBoneName, LeftTorque, LeftMotorParams, LeftWheelState);
	ApplyWheelTorque(RightMotorId, RightWheelBoneName, RightTorque, RightMotorParams, RightWheelState);
}

void URammsDifferentialDriveController::UpdateVelocityControl(float DeltaTime)
{
	// Convert joystick input to differential drive commands (velocity values).
	// With a MaxTurningSpeed, full steer asks for that yaw rate rather than
	// MaxVelocity at the wheels: a turn in place at full wheel speed can ask
	// more than the motors have against the tyres' scrub, and once both sides
	// saturate they slip unevenly and the robot drifts while it turns.
	FDifferentialDriveCommand VelocityCommand;
	if (MaxTurningSpeed > 0.0f)
	{
		const FVector2D Input = URammsDifferentialDriveLibrary::ApplyDeadZone2D(DriveInput, InputDeadZone);
		const float		Forward = FMath::Clamp(Input.Y, -1.0f, 1.0f) * MaxVelocity;
		const float		Turn = FMath::Clamp(Input.X, -1.0f, 1.0f) * FMath::DegreesToRadians(MaxTurningSpeed) * TrackWidth * 0.5f;
		VelocityCommand = FDifferentialDriveCommand(Forward + Turn, Forward - Turn);
		// Over MaxVelocity on either side: both scaled alike, keeping the curvature.
		const float Most = FMath::Max(FMath::Abs(VelocityCommand.LeftCommand), FMath::Abs(VelocityCommand.RightCommand));
		if (Most > MaxVelocity && Most > UE_SMALL_NUMBER)
		{
			VelocityCommand.LeftCommand *= MaxVelocity / Most;
			VelocityCommand.RightCommand *= MaxVelocity / Most;
		}
	}
	else
	{
		VelocityCommand = URammsDifferentialDriveLibrary::JoystickToDifferentialDrive(DriveInput, MaxVelocity, InputDeadZone);
	}

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
	AddCarryTorque(LeftTorque, RightTorque);
	AddReleaseTorque(LeftTorque, RightTorque);
	ApplyWheelTorque(LeftMotorId, LeftWheelBoneName, LeftTorque, LeftMotorParams, LeftWheelState);
	ApplyWheelTorque(RightMotorId, RightWheelBoneName, RightTorque, RightMotorParams, RightWheelState);
}

void URammsDifferentialDriveController::StepHold(double DeltaSeconds)
{
	// The wheel's share of the robot about its axle: half the mass at the
	// wheel's radius. The loop's gains follow from it (RammsDrive::TorqueHoldGains).
	const double Radius = FMath::Max(WheelRadius, 1.0f);
	const double Inertia = 0.5 * FMath::Max(VehicleMass, 1.0f) * FMath::Square(Radius / 100.0);
	// A brake, not a drive: with a robot base the hold may use the whole torque
	// the actuator takes (its registry range), not only the MaxTorque the stick
	// scales to -- a slope's pull on a wheel can be more than that. Without
	// one, the motor model's.
	const auto HoldTorque = [this](FName MotorId, const FMotorParameters& Motor) {
		double			Most = Motor.MaxTorque * TorqueMultiplier;
		FRammsMotorSpec Spec;
		if (UsesBase() && BaseComponent->GetMotorSpec(MotorId, Spec) && Spec.ControlRange.X < Spec.ControlRange.Y)
		{
			Most = FMath::Max(Most, FMath::Min(FMath::Abs(static_cast<double>(Spec.ControlRange.X)), FMath::Abs(static_cast<double>(Spec.ControlRange.Y))));
		}
		return Most;
	};
	const RammsDrive::FTorqueHoldGains GainsL = RammsDrive::TorqueHoldGains(Inertia, HoldTorque(LeftMotorId, LeftMotorParams), Radius, HoldDecelerationCmS2);
	const RammsDrive::FTorqueHoldGains GainsR = RammsDrive::TorqueHoldGains(Inertia, HoldTorque(RightMotorId, RightMotorParams), Radius, HoldDecelerationCmS2);
	// The two sides held as one robot: braked alike, on their mean rate, and
	// neither turning it back by more than HoldTurnErrorCm of tread travel
	// against the other.
	double PairLeft = 0.0, PairRight = 0.0;
	RammsDrive::StepTorqueHoldPair(*HoldLeft, LeftAngle, LeftWheelState.AngularVelocity, *HoldRight, RightAngle, RightWheelState.AngularVelocity,
		DeltaSeconds, GainsL, GainsR, HoldTurnErrorCm / Radius, PairLeft, PairRight);
	const float LeftTorque = static_cast<float>(PairLeft);
	const float RightTorque = static_cast<float>(PairRight);
	LeftWheelState.TargetAngularVelocity = static_cast<float>(HoldLeft->TargetRate);
	RightWheelState.TargetAngularVelocity = static_cast<float>(HoldRight->TargetRate);
	if (UsesBase())
	{
		// Straight to the actuator (its range clamps it), past the motor model.
		BaseComponent->SetMotorCommand(LeftMotorId, LeftTorque);
		BaseComponent->SetMotorCommand(RightMotorId, RightTorque);
		LeftWheelState.AppliedTorque = LeftTorque;
		RightWheelState.AppliedTorque = RightTorque;
	}
	else
	{
		ApplyWheelTorque(LeftMotorId, LeftWheelBoneName, LeftTorque, LeftMotorParams, LeftWheelState);
		ApplyWheelTorque(RightMotorId, RightWheelBoneName, RightTorque, RightMotorParams, RightWheelState);
	}
	// The control mode's own state starts afresh when the hold lets go.
	LeftIntegralError = RightIntegralError = 0.0f;
}

bool URammsDifferentialDriveController::CanHoldWithTorque() const
{
	return UsesBase() && BaseComponent->SimulatesMotorJoint(LeftMotorId) && BaseComponent->SimulatesMotorJoint(RightMotorId);
}

bool URammsDifferentialDriveController::HoldsRobotStill() const
{
	return bHoldWhenIdle && CanHoldWithTorque() && RammsDrive::HoldWhenIdleEnabled();
}

void URammsDifferentialDriveController::BeginHold(double ForwardLeft, double ForwardRight, double InitialLeft, double InitialRight, bool bEase)
{
	HoldLeft->Begin(LeftAngle, ForwardLeft, InitialLeft, bEase);
	HoldRight->Begin(RightAngle, ForwardRight, InitialRight, bEase);
}

void URammsDifferentialDriveController::ResetHoldState()
{
	HoldLeft->End();
	HoldRight->End();
	ReleaseElapsed = -1.0;
	LeftAngleSource->Reset();
	RightAngleSource->Reset();
	SmoothLeftRate = SmoothRightRate = 0.0;
	SeedExpires = -1.0;
	bStoodDownHeld = false;
	// Its wheels are back where a reset put them; brake them afresh once at rest.
	SetParkingBrake(false);
}

void URammsDifferentialDriveController::SeedGroundSlope(float ForwardGradeDeg, float RobotMassKg)
{
	SeedExpires = -1.0;
	if (!bDriveModeActive || !CanHoldWithTorque())
	{
		return;
	}
	// Each drive wheel's share of the pull: the two carry the robot along its
	// way (its other wheels, stood on, roll free), as far as the actuator goes.
	const double Mass = RobotMassKg > 0.0f ? RobotMassKg : FMath::Max(VehicleMass, 1.0f);
	double		 Most = FMath::Max(LeftMotorParams.MaxTorque, RightMotorParams.MaxTorque) * TorqueMultiplier;
	for (const FName MotorId : { LeftMotorId, RightMotorId })
	{
		FRammsMotorSpec Spec;
		if (BaseComponent->GetMotorSpec(MotorId, Spec) && Spec.ControlRange.X < Spec.ControlRange.Y)
		{
			Most = FMath::Max(Most, FMath::Min(FMath::Abs(static_cast<double>(Spec.ControlRange.X)), FMath::Abs(static_cast<double>(Spec.ControlRange.Y))));
		}
	}
	SeedTorque = FMath::Clamp(RammsDrive::GroundHoldTorque(Mass, ForwardGradeDeg, WheelRadius, 2), -Most, Most);
	SeedExpires = Now() + SeedValidSeconds;
}

void URammsDifferentialDriveController::HoldWheelsWhileInactive(bool bHold)
{
	if (bDriveModeActive)
	{
		bHoldInactive = false;
		return;
	}
	if (bHold && bHoldWhenIdle && CanHoldWithTorque() && RammsDrive::HoldWhenIdleEnabled())
	{
		// Carrying on with what this mode's own hold held as it was stood down
		// (on a slope, the slope's pull): begun again from zero, the robot
		// would sag a centimetre or two before the integral found it.
		const FVector2D Carried = bStoodDownHeld ? StoodDownIntegral : FVector2D::ZeroVector;
		bStoodDownHeld = false;
		BeginHold(SmoothLeftRate, SmoothRightRate, Carried.X, Carried.Y);
		bHoldInactive = true;
		// And at once: standing down wrote no torque to these wheels, and the
		// next tick is a frame away.
		StepHold(0.0);
		return;
	}
	if (bHoldInactive)
	{
		// Let go, as standing down does.
		bHoldInactive = false;
		HoldLeft->End();
		HoldRight->End();
		if (UsesBase())
		{
			for (const FName MotorId : { LeftMotorId, RightMotorId })
			{
				BaseComponent->ClearMotorVelocityCommand(MotorId);
				BaseComponent->SetMotorCommand(MotorId, 0.0f);
			}
		}
	}
}

void URammsDifferentialDriveController::SetParkingBrake(bool bEngage)
{
	if (bEngage == bParkingBrake || !UsesBase())
	{
		return;
	}
	if (!bEngage)
	{
		BaseComponent->SetMotorBrake(LeftMotorId, false);
		BaseComponent->SetMotorBrake(RightMotorId, false);
		bParkingBrake = false;
		return;
	}
	const bool bLeft = BaseComponent->SetMotorBrake(LeftMotorId, true);
	const bool bRight = BaseComponent->SetMotorBrake(RightMotorId, true);
	bParkingBrake = bLeft && bRight;
	if (!bParkingBrake)
	{
		// One side alone would slew the robot about the braked wheel.
		BaseComponent->SetMotorBrake(LeftMotorId, false);
		BaseComponent->SetMotorBrake(RightMotorId, false);
	}
}

void URammsDifferentialDriveController::ReleaseHold()
{
	// Only what held the robot along its way (the two sides' mean: a slope's
	// pull) fades out into the control mode's torque; what the two sides held
	// against each other -- the robot a little twisted between them -- goes at
	// once: faded out, it would turn the robot as it moves off.
	const double Left = HoldLeft->bActive ? HoldLeft->LastTorque : 0.0;
	const double Right = HoldRight->bActive ? HoldRight->LastTorque : 0.0;
	const double Along = (HoldLeft->bActive && HoldRight->bActive) ? 0.5 * (Left + Right) : 0.0;
	ReleaseTorque = (HoldLeft->bActive && HoldRight->bActive) ? FVector2D(Along, Along) : FVector2D(Left, Right);
	ReleaseElapsed = 0.0;
	HoldLeft->End();
	HoldRight->End();
}

void URammsDifferentialDriveController::AddReleaseTorque(float& LeftTorque, float& RightTorque) const
{
	if (ReleaseElapsed < 0.0)
	{
		return;
	}
	// What held the robot fades into the operator's command over
	// HoldReleaseSeconds: on a slope it was holding the slope's pull, and
	// dropped at once the robot would lurch back before the command took up.
	const double Weight = 1.0 - RammsDrive::SmoothStep(ReleaseElapsed / HoldReleaseSeconds);
	LeftTorque += static_cast<float>(Weight * ReleaseTorque.X);
	RightTorque += static_cast<float>(Weight * ReleaseTorque.Y);
}

void URammsDifferentialDriveController::AddCarryTorque(float& LeftTorque, float& RightTorque) const
{
	if (CarryWeight <= 0.0f)
	{
		return;
	}
	// On top of the operator's command, a pull towards the wheel speeds of the
	// motion handed over: the operator's torque is what holds the robot at the
	// operator's speed, so near it this asks little, and it fades out to the
	// operator's command alone with no step. Proportional only, half the
	// motor's stall torque per rad/s of error: the wheels are on the ground
	// by the time a mode is handed the robot (the selector puts them down
	// first), and on a loaded wheel that is far slower than a game tick.
	const auto Pull = [this](float Target, const FRammsWheelState& Wheel, const FMotorParameters& Motor) {
		return CarryWeight * 0.5f * Motor.MaxTorque * TorqueMultiplier * (Target - Wheel.AngularVelocity);
	};
	LeftTorque += Pull(static_cast<float>(CarryWheelRates.X), LeftWheelState, LeftMotorParams);
	RightTorque += Pull(static_cast<float>(CarryWheelRates.Y), RightWheelState, RightMotorParams);
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

void URammsDifferentialDriveController::ApplyWheelTorque(
	FName					MotorId,
	FName					BoneName,
	float					RequestedTorque,
	const FMotorParameters& MotorParams,
	FRammsWheelState&		WheelState)
{
	// The direct Chaos path needs a simulating wheel body; the base-component
	// path commands by motor Id and has no bone body to guard on.
	if (!UsesBase())
	{
		FBodyInstance* BodyInst = GetBoneBodyInstance(BoneName);
		if (!BodyInst || !BodyInst->IsInstanceSimulatingPhysics())
		{
			WheelState.AppliedTorque = 0.0f;
			return;
		}
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
		// Calculate traction multiplier from slip using realistic tire curve
		float TractionMultiplier = URammsDifferentialDriveLibrary::GetTractionMultiplierFromSlip(
			WheelState.SlipRatio,
			PeakSlipRatio);

		// Calculate available grip force if load-dependent traction is enabled
		if (bEnableLoadDependentTraction && WheelState.SuspensionLoad > 0.0f)
		{
			// Available grip force (Newtons)
			float AvailableGripForce = URammsDifferentialDriveLibrary::CalculateAvailableGrip(
				WheelState.SuspensionLoad,
				WheelState.SurfaceFriction,
				TractionCoefficient);

			// Convert grip force to maximum torque
			// Torque = Force × Radius, convert cm to m
			float MaxTorqueFromGrip = (AvailableGripForce * (WheelRadius / 100.0f)) * TractionMultiplier;

			// Limit available torque to what grip can support
			AvailableTorque = FMath::Min(FMath::Abs(AvailableTorque), MaxTorqueFromGrip) * FMath::Sign(AvailableTorque);
		}
		else
		{
			// Simple traction multiplier without load dependency
			AvailableTorque *= TractionMultiplier;
		}

		// Apply lateral slip resistance (reduces torque when sliding sideways)
		if (bEnableLateralSlipResistance && FMath::Abs(WheelState.LateralVelocity) > LateralSlipThreshold)
		{
			// Reduce torque proportionally to lateral velocity
			float LateralSlipFactor = FMath::Clamp(
				1.0f - (FMath::Abs(WheelState.LateralVelocity) - LateralSlipThreshold) / 100.0f,
				0.3f,
				1.0f);
			AvailableTorque *= LateralSlipFactor;
		}
	}

	WheelState.AppliedTorque = AvailableTorque;

	if (bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[DiffDrive] ApplyTorque to %s: Requested=%.3f Nm, Available=%.3f Nm, CurrentRPM=%.1f, Slip=%.3f, Load=%.1f N, Friction=%.2f, LatVel=%.1f cm/s"),
			*BoneName.ToString(), RequestedTorque, AvailableTorque, CurrentRPM, WheelState.SlipRatio,
			WheelState.SuspensionLoad, WheelState.SurfaceFriction, WheelState.LateralVelocity);
	}

	// Base-component path: command the computed torque (N·m) by motor Id; the
	// resolved backend (Chaos torque about the wheel's spin axis, or MuJoCo
	// <motor> ctrl) applies it and clamps to the motor's range.
	if (UsesBase())
	{
		BaseComponent->SetMotorCommand(MotorId, AvailableTorque);
		return;
	}

	// Direct Chaos path: torque about the wheel bone's local Y (spin) axis.
	FBodyInstance* BodyInst = GetBoneBodyInstance(BoneName);
	int32		   BoneIndex = SkeletalMeshComponent->GetBoneIndex(BoneName);
	if (!BodyInst || BoneIndex == INDEX_NONE)
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

	// A zero torque would still wake the body (see FRammsChaosActuationBackend::SetCommand).
	if (!WorldTorque.IsNearlyZero())
	{
		BodyInst->AddTorqueInRadians(WorldTorque, false, true);
	}
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
	// Apply braking by applying damping torque proportional to velocity
	// Add a deadband to prevent oscillation at very low speeds
	const float BrakeDeadband = 0.1f; // rad/s - don't apply brakes below this angular velocity

	// Base-component path: damping torque proportional to spin velocity, by
	// motor Id — BrakeTorque is the damping coefficient (per rad/s), the same
	// law the direct Chaos path applies below (the backend clamps the command
	// to the motor's range).
	if (UsesBase())
	{
		auto BrakeMotor = [this, BrakeDeadband](FName MotorId, FRammsWheelState& WheelState) {
			const float AngVel = WheelState.AngularVelocity;
			if (FMath::Abs(AngVel) > BrakeDeadband)
			{
				const float BrakeCmd = -AngVel * BrakeTorque;
				BaseComponent->SetMotorCommand(MotorId, BrakeCmd);
				WheelState.AppliedTorque = BrakeCmd;
			}
			else
			{
				BaseComponent->SetMotorCommand(MotorId, 0.0f);
				WheelState.AppliedTorque = 0.0f;
			}
		};
		BrakeMotor(LeftMotorId, LeftWheelState);
		BrakeMotor(RightMotorId, RightWheelState);

		// Reset PID integral terms while braking (as the direct path does), so
		// velocity control doesn't surge on stale error when braking ends.
		LeftIntegralError = 0.0f;
		RightIntegralError = 0.0f;
		return;
	}

	FBodyInstance* LeftBodyInst = GetBoneBodyInstance(LeftWheelBoneName);
	if (LeftBodyInst && LeftBodyInst->IsInstanceSimulatingPhysics())
	{
		FVector AngVel = LeftBodyInst->GetUnrealWorldAngularVelocityInRadians();
		float	AngVelMagnitude = AngVel.Size();

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
		float	AngVelMagnitude = AngVel.Size();

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

// --- control surface -----------------------------------------------------------

void URammsDifferentialDriveController::SetDriveModeActive(bool bActive)
{
	if (bDriveModeActive == bActive)
	{
		return;
	}
	if (!bActive)
	{
		// The next mode drives these wheels.
		SetParkingBrake(false);
	}
	bDriveModeActive = bActive;
	bHeldForStance = false;
	bHoldInactive = false;
	EndCarry();
	// Standing down while holding the robot: what the hold held, for a hold
	// of these wheels through the switch to carry on with
	// (HoldWheelsWhileInactive).
	bStoodDownHeld = !bActive && HoldLeft->bActive && HoldRight->bActive && !HoldLeft->bFree && !HoldRight->bFree;
	StoodDownIntegral = bStoodDownHeld ? FVector2D(HoldLeft->Integral, HoldRight->Integral) : FVector2D::ZeroVector;
	HoldLeft->End();
	HoldRight->End();
	ReleaseElapsed = -1.0;
	SeedExpires = -1.0;
	if (!bActive)
	{
		// Straight to ApplyDriveInputInternal, not SetDriveInput: that one
		// returns early while an external session holds priority, so the
		// stand-down would be a no-op exactly when something else is driving.
		// A mode being switched away from stops, whoever was commanding it.
		ApplyDriveInputInternal(FVector2D::ZeroVector, TEXT("SetDriveModeActive"));
		// And lets go of its wheels. A torque written last stays in the
		// actuator until something writes over it, so a wheel the next mode
		// lifts clear would go on spinning up in the air, and one it rolls on
		// would be dragged or pushed by this mode's last command.
		if (UsesBase())
		{
			for (const FName MotorId : { LeftMotorId, RightMotorId })
			{
				BaseComponent->ClearMotorVelocityCommand(MotorId);
				BaseComponent->SetMotorCommand(MotorId, 0.0f);
			}
		}
	}
}

FRammsDriveMotion URammsDifferentialDriveController::GetCarriableMotion(const FRammsDriveMotion& Motion) const
{
	return RammsDrive::DifferentialCarriable(Motion, MaxTurningSpeed);
}

void URammsDifferentialDriveController::SeedMotion(const FRammsDriveMotion& Motion, float HoldSeconds)
{
	EndCarry();
	if (!bDriveModeActive || !Carry || HoldSeconds <= 0.0f)
	{
		// Nothing to hold: the control mode carries on from the speed the
		// robot has.
		return;
	}
	const FRammsDriveMotion Carriable = GetCarriableMotion(Motion);
	const FVector2D			Ground = RammsDrive::DifferentialWheelSpeeds(Carriable, TrackWidth);
	CarryWheelRates = Ground / FMath::Max(WheelRadius, 1.0f);
	// Begun whether or not the input is off centre yet: an operator whose
	// stick reaches this mode on its own tick, or through the control surface
	// after the switch rebuilt it, is heard on the first tick; one who is not
	// driving by then ends it there (TickComponent).
	Carry->Begin(Carriable, HoldSeconds, /*bOperatorCommanding=*/true);
	bFirstCarryTick = true;
	bCarryInputSet = IsOperatorCommanding();
	CarryInput = DriveInput;
}

void URammsDifferentialDriveController::EndCarry()
{
	bFirstCarryTick = false;
	bCarryInputSet = false;
	if (!Carry || !Carry->bActive)
	{
		return;
	}
	Carry->End();
	CarryWeight = 0.0f;
}

bool URammsDifferentialDriveController::IsOperatorCommanding() const
{
	return !URammsDifferentialDriveLibrary::ApplyDeadZone2D(DriveInput, InputDeadZone).IsNearlyZero();
}

double URammsDifferentialDriveController::Now() const
{
	if (UsesBase())
	{
		const double Sim = BaseComponent->GetSimulationTime();
		if (Sim >= 0.0)
		{
			return Sim;
		}
	}
	const UWorld* World = GetWorld();
	return World ? World->GetTimeSeconds() : 0.0;
}

void URammsDifferentialDriveController::DescribeControls(FRammsControlSurface& OutSurface) const
{
	if (!bDriveModeActive)
	{
		// Another drive mode owns these Ids right now -- or none does, and the
		// low-level mode is handing the motors out individually.
		return;
	}
	const FName		  DriveForwardId = RammsControlIds::Drive::Forward();
	const FName		  DriveTurnId = RammsControlIds::Drive::Turn();
	FRammsControlAxis Forward;
	Forward.Id = DriveForwardId;
	Forward.Group = RammsControlIds::Groups::Drive();
	Forward.DisplayName = NSLOCTEXT("Ramms", "DriveForward", "Forward");
	Forward.Kind = ERammsControlKind::Continuous;
	Forward.Units = ERammsControlUnits::Normalized;
	Forward.Range = FVector2D(-1.0, 1.0);
	Forward.Order = 0;
	Forward.PairedAxis = DriveTurnId;
	OutSurface.Add(Forward);

	FRammsControlAxis Turn = Forward;
	Turn.Id = DriveTurnId;
	Turn.DisplayName = NSLOCTEXT("Ramms", "DriveTurn", "Turn");
	Turn.Order = 1;
	Turn.PairedAxis = DriveForwardId;
	OutSurface.Add(Turn);
}

bool URammsDifferentialDriveController::ApplyControl(FName Id, float Value)
{
	if (!bDriveModeActive)
	{
		return false;
	}
	const FName DriveForwardId = RammsControlIds::Drive::Forward();
	const FName DriveTurnId = RammsControlIds::Drive::Turn();
	if (Id == DriveForwardId)
	{
		ControlDriveInput.Y = Value;
	}
	else if (Id == DriveTurnId)
	{
		ControlDriveInput.X = Value;
	}
	else
	{
		return false;
	}
	// Through the external hold: the surface has already arbitrated between
	// its sources, and the hold makes the command outrank any legacy per-tick
	// joystick writer (the chair pawn's tick) for ExternalInputHoldSeconds
	// after each call — releasing pushes zero the same way, so the legacy
	// path resumes once the hold lapses.
	SetExternalDriveInput(ControlDriveInput);
	return true;
}

bool URammsDifferentialDriveController::ReleaseControl(FName Id)
{
	return ApplyControl(Id, 0.0f);
}

bool URammsDifferentialDriveController::ReadControl(FName Id, float& OutValue) const
{
	const FName DriveForwardId = RammsControlIds::Drive::Forward();
	const FName DriveTurnId = RammsControlIds::Drive::Turn();
	if (Id == DriveForwardId)
	{
		OutValue = static_cast<float>(DriveInput.Y);
		return true;
	}
	if (Id == DriveTurnId)
	{
		OutValue = static_cast<float>(DriveInput.X);
		return true;
	}
	return false;
}

void URammsDifferentialDriveController::GetClaimedMotorIds(TArray<FName>& OutIds) const
{
	if (!bDriveModeActive)
	{
		// Standing down: let the wheels show as raw motor axes instead.
		return;
	}
	OutIds.Add(LeftMotorId);
	OutIds.Add(RightMotorId);
}
