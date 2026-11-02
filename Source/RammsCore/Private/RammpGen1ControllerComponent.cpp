// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammpGen1ControllerComponent.h"
#include "RammsRobotBaseComponent.h"
#include "RammsMotorSpec.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "DrawDebugHelpers.h"
#include "RammsDriveMode.h"

namespace
{
	/** The motors an active drive mode on Self's robot claims (a curb crossing's lifts while it runs). This
	 *  controller leaves them alone and follows them meanwhile, so it takes them back where they stand
	 *  instead of from its own stale ramp. */
	TSet<FName> RammpGen1MotorsClaimedByActiveMode(const UActorComponent* Self)
	{
		TSet<FName>	  Out;
		const AActor* Owner = Self ? Self->GetOwner() : nullptr;
		if (!Owner)
		{
			return Out;
		}
		for (UActorComponent* C : Owner->GetComponents())
		{
			const IRammsDriveMode*			Mode = Cast<IRammsDriveMode>(C);
			const IRammsControlContributor* Contributor = Cast<IRammsControlContributor>(C);
			if (C != Self && Mode && Contributor && Mode->IsDriveModeActive())
			{
				TArray<FName> Ids;
				Contributor->GetClaimedMotorIds(Ids);
				Out.Append(Ids);
			}
		}
		return Out;
	}

	bool RammpGen1MotorYielded(const UActorComponent* Self, FName MotorId)
	{
		return !MotorId.IsNone() && RammpGen1MotorsClaimedByActiveMode(Self).Contains(MotorId);
	}
} // namespace

URammpGen1ControllerComponent::URammpGen1ControllerComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	bAutoFindConstraints = true;
	bAutoFindSkeletalMesh = true;
	bEnableDebugLog = false;
	bShowMotorGizmos = false;
	GizmoSize = 50.0f;
	CachedSkeletalMesh = nullptr;

	// Initialize default motor configurations
	// Front caster arm - angular motor
	FAngularMotorConfig FrontCaster;
	FrontCaster.ConstraintName = FName("front_caster_swing_arm");
	FrontCaster.ControlAxis = EMotorAxis::Y;
	FrontCaster.TargetAngle = 0.0f;
	AngularMotors.Add(FrontCaster);

	// Rear caster arm - angular motor
	FAngularMotorConfig RearCaster;
	RearCaster.ConstraintName = FName("rear_caster_swing_arm");
	RearCaster.ControlAxis = EMotorAxis::Y;
	RearCaster.TargetAngle = 0.0f;
	AngularMotors.Add(RearCaster);

	// Left motor elevator - angular motor
	FAngularMotorConfig LeftElevator;
	LeftElevator.ConstraintName = FName("motor_swing_arm_l");
	LeftElevator.ControlAxis = EMotorAxis::Y;
	LeftElevator.TargetAngle = 0.0f;
	AngularMotors.Add(LeftElevator);

	// Right motor elevator - angular motor
	FAngularMotorConfig RightElevator;
	RightElevator.ConstraintName = FName("motor_swing_arm_r");
	RightElevator.ControlAxis = EMotorAxis::Y;
	RightElevator.TargetAngle = 0.0f;
	AngularMotors.Add(RightElevator);

	// Left motor translator - linear motor
	FLinearMotorConfig LeftTranslator;
	LeftTranslator.ConstraintName = FName("dw_main_plate_l");
	LeftTranslator.ControlAxis = EMotorAxis::X;
	LeftTranslator.TargetPosition = 0.0f;
	LinearMotors.Add(LeftTranslator);

	// Right motor translator - linear motor
	FLinearMotorConfig RightTranslator;
	RightTranslator.ConstraintName = FName("dw_main_plate_r");
	RightTranslator.ControlAxis = EMotorAxis::X;
	RightTranslator.TargetPosition = 0.0f;
	LinearMotors.Add(RightTranslator);
}

void URammpGen1ControllerComponent::BeginPlay()
{
	Super::BeginPlay();

	ResolveRobotBase();
	if (bAutoFindConstraints)
	{
		// Direct-constraint path for motors the base doesn't know (or all of
		// them when there is no base).
		FindConstraints();
	}

	if (bEnableDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("RammpGen1Controller: Initialized with %d angular motors and %d linear motors (robot base: %s)"),
			AngularMotors.Num(), LinearMotors.Num(), IsUsingRobotBase() ? TEXT("yes") : TEXT("no"));
	}
}

bool URammpGen1ControllerComponent::IsUsingRobotBase() const
{
	return bUseRobotBase && RobotBase && RobotBase->HasBackend();
}

FName URammpGen1ControllerComponent::BaseMotorId(FName ConstraintName, FName ExplicitMotorId) const
{
	if (!RobotBase)
	{
		return NAME_None;
	}	// An explicit Id is authoritative: a typo must not silently pick a
	// different motor via the constraint-name fallback.
	if (!ExplicitMotorId.IsNone())
	{
		return RobotBase->HasMotor(ExplicitMotorId) ? ExplicitMotorId : NAME_None;
	}
	return RobotBase->FindMotorIdByChaosName(ConstraintName);
}

void URammpGen1ControllerComponent::ResolveRobotBase()
{
	if (!bUseRobotBase)
	{
		return;
	}
	if (!RobotBase)
	{
		if (AActor* Owner = GetOwner())
		{
			RobotBase = Owner->FindComponentByClass<URammsRobotBaseComponent>();
		}
	}
	if (!IsUsingRobotBase())
	{
		for (FAngularMotorConfig& Motor : AngularMotors)
		{
			Motor.ResolvedMotorId = NAME_None;
		}
		for (FLinearMotorConfig& Motor : LinearMotors)
		{
			Motor.ResolvedMotorId = NAME_None;
		}
		return;
	}
	// Resolve each motor's routing Id once so the per-tick path is a direct
	// lookup; the authored MotorId stays as configured (so "explicit but not in
	// the registry" is still known on a later re-resolve). Motors the registry
	// doesn't list keep driving their constraint directly.
	auto Warn = [this](FName ConstraintName, FName ExplicitId) {
		UE_LOG(LogTemp, Warning, TEXT("RammpGen1Controller: no registry motor on '%s' for constraint '%s'%s — driving it directly."),
			*RobotBase->GetName(), *ConstraintName.ToString(),
			ExplicitId.IsNone() ? TEXT("") : *FString::Printf(TEXT(" (explicit MotorId '%s' is not in the registry)"), *ExplicitId.ToString()));
	};
	for (FAngularMotorConfig& Motor : AngularMotors)
	{
		Motor.ResolvedMotorId = BaseMotorId(Motor.ConstraintName, Motor.MotorId);
		if (Motor.ResolvedMotorId.IsNone())
		{
			Warn(Motor.ConstraintName, Motor.MotorId);
		}
	}
	for (FLinearMotorConfig& Motor : LinearMotors)
	{
		Motor.ResolvedMotorId = BaseMotorId(Motor.ConstraintName, Motor.MotorId);
		if (Motor.ResolvedMotorId.IsNone())
		{
			Warn(Motor.ConstraintName, Motor.MotorId);
		}
	}
	UE_LOG(LogTemp, Log, TEXT("RammpGen1Controller: routing motors through robot base '%s'."), *RobotBase->GetName());
}

void URammpGen1ControllerComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

	// Clear debug draws
	if (UWorld* World = GetWorld())
	{
		FlushPersistentDebugLines(World);
	}
}

void URammpGen1ControllerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	RefreshConstraintCache();

	// Ramp at the backend's clock when it keeps one: a simulator behind real time
	// is slower than the game, and MaxSpeed is the mechanism's rate in sim time.
	// The game's clock otherwise (Chaos).
	float	   Dt = DeltaTime;
	const bool bViaBase = IsUsingRobotBase();
	if (bViaBase)
	{
		const double Now = RobotBase->GetSimulationTime();
		if (Now >= 0.0)
		{
			// A reset sends the clock back: no step across it.
			Dt = (LastSimTime < 0.0 || Now < LastSimTime) ? 0.0f : static_cast<float>(Now - LastSimTime);
			LastSimTime = Now;
		}
	}
	// Which base motors another active mode drives, gathered once a tick.
	const TSet<FName> Yielded = bViaBase ? RammpGen1MotorsClaimedByActiveMode(this) : TSet<FName>();
	UpdateAngularMotors(Dt, Yielded);
	UpdateLinearMotors(Dt, Yielded);
	DrawDebugVisualization();
}

void URammpGen1ControllerComponent::SetAngularMotorTarget(FName MotorName, float TargetAngle)
{
	RefreshConstraintCache(); // applies immediately — revalidate at point of use, not just in tick
	for (FAngularMotorConfig& Motor : AngularMotors)
	{
		if (Motor.ConstraintName == MotorName)
		{
			Motor.TargetAngle = TargetAngle;
			Motor.bPendingApply = true;
			ApplyAngularMotorSettings(Motor);

			if (bEnableDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("RammpGen1Controller: Set angular motor '%s' target to %.2f degrees"),
					*MotorName.ToString(), TargetAngle);
			}
			return;
		}
	}
	UE_LOG(LogTemp, Warning, TEXT("RammpGen1Controller: Angular motor '%s' not found"), *MotorName.ToString());
}

void URammpGen1ControllerComponent::SetLinearMotorTarget(FName MotorName, float TargetPosition)
{
	RefreshConstraintCache(); // applies immediately — revalidate at point of use, not just in tick
	for (FLinearMotorConfig& Motor : LinearMotors)
	{
		if (Motor.ConstraintName == MotorName)
		{
			Motor.TargetPosition = TargetPosition;
			Motor.bPendingApply = true;
			ApplyLinearMotorSettings(Motor);

			if (bEnableDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("RammpGen1Controller: Set linear motor '%s' target to %.2f cm"),
					*MotorName.ToString(), TargetPosition);
			}
			return;
		}
	}
	UE_LOG(LogTemp, Warning, TEXT("RammpGen1Controller: Linear motor '%s' not found"), *MotorName.ToString());
}

void URammpGen1ControllerComponent::SetAngularMotorMaxSpeed(FName MotorName, float MaxSpeed)
{
	for (FAngularMotorConfig& Motor : AngularMotors)
	{
		if (Motor.ConstraintName == MotorName)
		{
			Motor.MaxSpeed = FMath::Max(0.0f, MaxSpeed);
			Motor.bPendingApply = true;

			if (bEnableDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("RammpGen1Controller: Set angular motor '%s' max speed to %.2f deg/s"),
					*MotorName.ToString(), Motor.MaxSpeed);
			}
			return;
		}
	}
	UE_LOG(LogTemp, Warning, TEXT("RammpGen1Controller: Angular motor '%s' not found"), *MotorName.ToString());
}

void URammpGen1ControllerComponent::SetLinearMotorMaxSpeed(FName MotorName, float MaxSpeed)
{
	for (FLinearMotorConfig& Motor : LinearMotors)
	{
		if (Motor.ConstraintName == MotorName)
		{
			Motor.MaxSpeed = FMath::Max(0.0f, MaxSpeed);
			Motor.bPendingApply = true;

			if (bEnableDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("RammpGen1Controller: Set linear motor '%s' max speed to %.2f cm/s"),
					*MotorName.ToString(), Motor.MaxSpeed);
			}
			return;
		}
	}
	UE_LOG(LogTemp, Warning, TEXT("RammpGen1Controller: Linear motor '%s' not found"), *MotorName.ToString());
}

void URammpGen1ControllerComponent::SetAngularMotorSpeedMultiplier(FName MotorName, float SpeedMultiplier)
{
	for (FAngularMotorConfig& Motor : AngularMotors)
	{
		if (Motor.ConstraintName == MotorName)
		{
			Motor.SpeedMultiplier = FMath::Clamp(SpeedMultiplier, 0.0f, 1.0f);
			Motor.bPendingApply = true;

			if (bEnableDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("RammpGen1Controller: Set angular motor '%s' speed multiplier to %.2f"),
					*MotorName.ToString(), Motor.SpeedMultiplier);
			}
			return;
		}
	}
	UE_LOG(LogTemp, Warning, TEXT("RammpGen1Controller: Angular motor '%s' not found"), *MotorName.ToString());
}

void URammpGen1ControllerComponent::SetLinearMotorSpeedMultiplier(FName MotorName, float SpeedMultiplier)
{
	for (FLinearMotorConfig& Motor : LinearMotors)
	{
		if (Motor.ConstraintName == MotorName)
		{
			Motor.SpeedMultiplier = FMath::Clamp(SpeedMultiplier, 0.0f, 1.0f);
			Motor.bPendingApply = true;

			if (bEnableDebugLog)
			{
				UE_LOG(LogTemp, Log, TEXT("RammpGen1Controller: Set linear motor '%s' speed multiplier to %.2f"),
					*MotorName.ToString(), Motor.SpeedMultiplier);
			}
			return;
		}
	}
	UE_LOG(LogTemp, Warning, TEXT("RammpGen1Controller: Linear motor '%s' not found"), *MotorName.ToString());
}

void URammpGen1ControllerComponent::SetAngularMotorEnabled(FName MotorName, bool bEnabled)
{
	RefreshConstraintCache(); // applies immediately — revalidate at point of use, not just in tick
	for (FAngularMotorConfig& Motor : AngularMotors)
	{
		if (Motor.ConstraintName == MotorName)
		{
			if (!bEnabled && IsUsingRobotBase() && !Motor.ResolvedMotorId.IsNone() && !RobotBase->ReleaseMotor(Motor.ResolvedMotorId))
			{
				// The backend can't let go (e.g. an actuator with no off switch):
				// saying "disabled" would be a lie — the motor keeps holding.
				UE_LOG(LogTemp, Warning, TEXT("RammpGen1Controller: the robot base cannot release motor '%s' (%s); leaving it enabled."),
					*Motor.ResolvedMotorId.ToString(), *MotorName.ToString());
				return;
			}
			Motor.bEnabled = bEnabled;
			Motor.bPendingApply = true;
			ApplyAngularMotorSettings(Motor);
			return;
		}
	}
}

void URammpGen1ControllerComponent::SetLinearMotorEnabled(FName MotorName, bool bEnabled)
{
	RefreshConstraintCache(); // applies immediately — revalidate at point of use, not just in tick
	for (FLinearMotorConfig& Motor : LinearMotors)
	{
		if (Motor.ConstraintName == MotorName)
		{
			if (!bEnabled && IsUsingRobotBase() && !Motor.ResolvedMotorId.IsNone() && !RobotBase->ReleaseMotor(Motor.ResolvedMotorId))
			{
				UE_LOG(LogTemp, Warning, TEXT("RammpGen1Controller: the robot base cannot release motor '%s' (%s); leaving it enabled."),
					*Motor.ResolvedMotorId.ToString(), *MotorName.ToString());
				return;
			}
			Motor.bEnabled = bEnabled;
			Motor.bPendingApply = true;
			ApplyLinearMotorSettings(Motor);
			return;
		}
	}
}

float URammpGen1ControllerComponent::GetAngularMotorCurrentAngle(FName MotorName) const
{
	for (const FAngularMotorConfig& Motor : AngularMotors)
	{
		if (Motor.ConstraintName == MotorName)
		{
			// The joint's angle in the motor's own sense, that of TargetAngle: an
			// inverted motor drives the joint to -TargetAngle.
			const float Sign = Motor.bInvertDirection ? -1.0f : 1.0f;
			if (IsUsingRobotBase() && !Motor.ResolvedMotorId.IsNone())
			{
				// The base reads the joint in radians (constraint angle on Chaos,
				// position actuator's joint on MuJoCo).
				return Sign * FMath::RadiansToDegrees(RobotBase->GetMotorValue(Motor.ResolvedMotorId));
			}
			// Const getter callable from Blueprint at any point in the frame: resolve from
			// the mesh's CURRENT physics state instead of trusting the tick-time cache,
			// which dangles after a physics state recreation until the next refresh.
			FConstraintInstance* Constraint = IsValid(CachedSkeletalMesh)
				? CachedSkeletalMesh->FindConstraintInstance(Motor.ConstraintName)
				: nullptr;
			if (Constraint == nullptr)
			{
				return 0.0f;
			}
			// Get current angular state from constraint: the drive turns about X for
			// twist, Y for swing 2 and Z for swing 1 (ApplyAngularMotorSettings).
			float CurrentAngle = 0.0f;
			switch (Motor.ControlAxis)
			{
				case EMotorAxis::X:
					CurrentAngle = Constraint->GetCurrentTwist();
					break;
				case EMotorAxis::Y:
					CurrentAngle = Constraint->GetCurrentSwing2();
					break;
				case EMotorAxis::Z:
					CurrentAngle = Constraint->GetCurrentSwing1();
					break;
			}
			// Chaos reports the constraint angles in radians; this API is degrees
			// on both paths (TargetAngle is degrees).
			return Sign * FMath::RadiansToDegrees(CurrentAngle);
		}
	}
	return 0.0f;
}

float URammpGen1ControllerComponent::GetLinearMotorCurrentPosition(FName MotorName) const
{
	for (const FLinearMotorConfig& Motor : LinearMotors)
	{
		if (Motor.ConstraintName == MotorName)
		{
			if (IsUsingRobotBase() && !Motor.ResolvedMotorId.IsNone())
			{
				// Real travel (cm) from the base
				return RobotBase->GetMotorValue(Motor.ResolvedMotorId);
			}
			if (Motor.CachedConstraint)
			{
				// Get current linear state from constraint - not directly available, return target
				return Motor.TargetPosition;
			}
		}
	}
	return 0.0f;
}

void URammpGen1ControllerComponent::ReinitializeMotors()
{
	ResolveRobotBase();
	FindConstraints();

	// Seed the rate limiters from the live joints (base-routed motors read the
	// base; direct ones the constraint) so a reinit eases from where the
	// mechanism is instead of jumping from a stale value. Targets are kept.
	const bool bViaBase = IsUsingRobotBase();
	for (FAngularMotorConfig& Motor : AngularMotors)
	{
		if (Motor.CachedConstraint || (bViaBase && !Motor.ResolvedMotorId.IsNone()))
		{
			Motor.CurrentAngle = GetAngularMotorCurrentAngle(Motor.ConstraintName);
		}
	}

	for (FLinearMotorConfig& Motor : LinearMotors)
	{
		// The direct path can't read a constraint's linear position; the base can.
		Motor.CurrentPosition = (bViaBase && !Motor.ResolvedMotorId.IsNone()) ? RobotBase->GetMotorValue(Motor.ResolvedMotorId) : Motor.TargetPosition;
	}

	for (FAngularMotorConfig& Motor : AngularMotors)
	{
		Motor.bPendingApply = true;
	}
	for (FLinearMotorConfig& Motor : LinearMotors)
	{
		Motor.bPendingApply = true;
	}
	const TSet<FName> Yielded = bViaBase ? RammpGen1MotorsClaimedByActiveMode(this) : TSet<FName>();
	UpdateAngularMotors(0.0f, Yielded);
	UpdateLinearMotors(0.0f, Yielded);

	if (bEnableDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("RammpGen1Controller: Motors reinitialized"));
	}
}

FString URammpGen1ControllerComponent::GetMotorDebugInfo() const
{
	FString Info = TEXT("=== RAMMP Gen1 Motor Status ===\n\nAngular Motors:\n");

	const bool bViaBase = IsUsingRobotBase();
	for (const FAngularMotorConfig& Motor : AngularMotors)
	{
		FString AxisStr = Motor.ControlAxis == EMotorAxis::X ? TEXT("X") : Motor.ControlAxis == EMotorAxis::Y ? TEXT("Y")
																											  : TEXT("Z");
		FString StatusStr = (bViaBase && !Motor.ResolvedMotorId.IsNone()) ? FString::Printf(TEXT("base:%s"), *Motor.ResolvedMotorId.ToString())
																  : (Motor.CachedConstraint ? TEXT("OK") : TEXT("NOT FOUND"));
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
		FString AxisStr = Motor.ControlAxis == EMotorAxis::X ? TEXT("X") : Motor.ControlAxis == EMotorAxis::Y ? TEXT("Y")
																											  : TEXT("Z");
		FString StatusStr = (bViaBase && !Motor.ResolvedMotorId.IsNone()) ? FString::Printf(TEXT("base:%s"), *Motor.ResolvedMotorId.ToString())
																  : (Motor.CachedConstraint ? TEXT("OK") : TEXT("NOT FOUND"));
		Info += FString::Printf(TEXT("  %s: Target=%.1fcm | Axis=%s | Enabled=%s | Status=%s\n"),
			*Motor.ConstraintName.ToString(),
			Motor.TargetPosition,
			*AxisStr,
			Motor.bEnabled ? TEXT("Yes") : TEXT("No"),
			*StatusStr);
	}

	return Info;
}

void URammpGen1ControllerComponent::RefreshConstraintCache()
{
	// Mirror FindConstraints(): only auto-acquire the mesh when configured to.
	if (!IsValid(CachedSkeletalMesh) && bAutoFindSkeletalMesh)
	{
		CachedSkeletalMesh = GetOwnerSkeletalMesh();
	}
	// FindConstraintInstance returns the instance owned by the CURRENT physics state (null
	// while physics is torn down mid-reregistration); motor updates already null-check and
	// skip the frame instead of driving a freed constraint.
	USkeletalMeshComponent* Mesh = IsValid(CachedSkeletalMesh) ? CachedSkeletalMesh : nullptr;
	// A new instance (physics state recreated) starts from the asset's defaults:
	// mark the motor pending so its target / drive settings are pushed again.
	for (FAngularMotorConfig& Motor : AngularMotors)
	{
		FConstraintInstance* Found = Mesh ? Mesh->FindConstraintInstance(Motor.ConstraintName) : nullptr;
		if (Found && Found != Motor.CachedConstraint)
		{
			Motor.bPendingApply = true;
		}
		Motor.CachedConstraint = Found;
	}
	for (FLinearMotorConfig& Motor : LinearMotors)
	{
		FConstraintInstance* Found = Mesh ? Mesh->FindConstraintInstance(Motor.ConstraintName) : nullptr;
		if (Found && Found != Motor.CachedConstraint)
		{
			Motor.bPendingApply = true;
		}
		Motor.CachedConstraint = Found;
	}
}

void URammpGen1ControllerComponent::FindConstraints()
{
	// Find the skeletal mesh first
	if (bAutoFindSkeletalMesh)
	{
		CachedSkeletalMesh = GetOwnerSkeletalMesh();
	}

	if (!CachedSkeletalMesh)
	{
		UE_LOG(LogTemp, Warning, TEXT("RammpGen1Controller: No SkeletalMeshComponent found on '%s'"),
			*GetOwner()->GetName());
		return;
	}

	if (bEnableDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("RammpGen1Controller: Found skeletal mesh '%s'"),
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
				UE_LOG(LogTemp, Log, TEXT("RammpGen1Controller: Found angular motor constraint '%s' in physics asset"),
					*Motor.ConstraintName.ToString());
			}
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("RammpGen1Controller: Angular motor constraint '%s' not found in physics asset"),
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
				UE_LOG(LogTemp, Log, TEXT("RammpGen1Controller: Found linear motor constraint '%s' in physics asset"),
					*Motor.ConstraintName.ToString());
			}
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("RammpGen1Controller: Linear motor constraint '%s' not found in physics asset"),
				*Motor.ConstraintName.ToString());
		}
	}
}

USkeletalMeshComponent* URammpGen1ControllerComponent::GetOwnerSkeletalMesh()
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

void URammpGen1ControllerComponent::UpdateAngularMotors(float DeltaTime, const TSet<FName>& Yielded)
{
	const bool bViaBase = IsUsingRobotBase();
	for (FAngularMotorConfig& Motor : AngularMotors)
	{
		if (bViaBase && !Motor.ResolvedMotorId.IsNone() && Yielded.Contains(Motor.ResolvedMotorId))
		{
			// Another mode drives it: follow it, writing nothing.
			const float Degrees = FMath::RadiansToDegrees(RobotBase->GetMotorValue(Motor.ResolvedMotorId));
			Motor.CurrentAngle = Motor.TargetAngle = Motor.bInvertDirection ? -Degrees : Degrees;
			Motor.bPendingApply = false;
			continue;
		}
		if (Motor.bEnabled && (Motor.CachedConstraint || (bViaBase && !Motor.ResolvedMotorId.IsNone())))
		{
			const bool bMoving = !FMath::IsNearlyEqual(Motor.CurrentAngle, Motor.TargetAngle);
			if (!bMoving && !Motor.bPendingApply)
			{
				continue; // the drive holds its target; nothing to push
			}
			// Smoothly interpolate current angle toward target at specified speed
			float EffectiveSpeed = Motor.MaxSpeed * Motor.SpeedMultiplier;
			float MaxDelta = EffectiveSpeed * DeltaTime;
			float AngleDiff = Motor.TargetAngle - Motor.CurrentAngle;

			// Clamp the movement to max speed
			if (FMath::Abs(AngleDiff) > MaxDelta)
			{
				Motor.CurrentAngle += FMath::Sign(AngleDiff) * MaxDelta;
			}
			else
			{
				Motor.CurrentAngle = Motor.TargetAngle;
			}

			ApplyAngularMotorSettings(Motor);
			Motor.bPendingApply = false;
		}
	}
}

void URammpGen1ControllerComponent::UpdateLinearMotors(float DeltaTime, const TSet<FName>& Yielded)
{
	const bool bViaBase = IsUsingRobotBase();
	for (FLinearMotorConfig& Motor : LinearMotors)
	{
		if (bViaBase && !Motor.ResolvedMotorId.IsNone() && Yielded.Contains(Motor.ResolvedMotorId))
		{
			// Another mode drives it: follow it, writing nothing.
			Motor.CurrentPosition = Motor.TargetPosition = RobotBase->GetMotorValue(Motor.ResolvedMotorId);
			Motor.bPendingApply = false;
			continue;
		}
		if (Motor.bEnabled && (Motor.CachedConstraint || (bViaBase && !Motor.ResolvedMotorId.IsNone())))
		{
			const bool bMoving = !FMath::IsNearlyEqual(Motor.CurrentPosition, Motor.TargetPosition);
			if (!bMoving && !Motor.bPendingApply)
			{
				continue; // the drive holds its target; nothing to push
			}
			// Smoothly interpolate current position toward target at specified speed
			float EffectiveSpeed = Motor.MaxSpeed * Motor.SpeedMultiplier;
			float MaxDelta = EffectiveSpeed * DeltaTime;
			float PositionDiff = Motor.TargetPosition - Motor.CurrentPosition;

			// Clamp the movement to max speed
			if (FMath::Abs(PositionDiff) > MaxDelta)
			{
				Motor.CurrentPosition += FMath::Sign(PositionDiff) * MaxDelta;
			}
			else
			{
				Motor.CurrentPosition = Motor.TargetPosition;
			}

			ApplyLinearMotorSettings(Motor);
			Motor.bPendingApply = false;
		}
	}
}

void URammpGen1ControllerComponent::ApplyAngularMotorSettings(FAngularMotorConfig& Motor)
{
	// Through the robot base: the interpolated angle (degrees, inverted if
	// asked) becomes a Position command in radians; the backend owns the
	// drive (constraint drive on Chaos, position actuator on MuJoCo).
	if (IsUsingRobotBase() && !Motor.ResolvedMotorId.IsNone())
	{
		if (Motor.bEnabled)
		{
			const float EffectiveAngle = Motor.bInvertDirection ? -Motor.CurrentAngle : Motor.CurrentAngle;
			RobotBase->SetMotorCommand(Motor.ResolvedMotorId, FMath::DegreesToRadians(EffectiveAngle));
		}
		else
		{
			// Disabled: stop servoing (the backend disables its drive), as the
			// direct path does.
			RobotBase->ReleaseMotor(Motor.ResolvedMotorId);
		}
		return;
	}
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

		// Set target orientation based on current interpolated angle (not target angle directly)
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
	}
}

void URammpGen1ControllerComponent::ApplyLinearMotorSettings(FLinearMotorConfig& Motor)
{
	// Through the robot base: the interpolated travel (cm) is the Position command.
	if (IsUsingRobotBase() && !Motor.ResolvedMotorId.IsNone())
	{
		if (Motor.bEnabled)
		{
			RobotBase->SetMotorCommand(Motor.ResolvedMotorId, Motor.CurrentPosition);
		}
		else
		{
			RobotBase->ReleaseMotor(Motor.ResolvedMotorId);
		}
		return;
	}
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

		// Set target position based on current interpolated position (not target position directly)
		FVector TargetPosition = FVector::ZeroVector;
		switch (Motor.ControlAxis)
		{
			case EMotorAxis::X:
				TargetPosition.X = Motor.CurrentPosition;
				break;
			case EMotorAxis::Y:
				TargetPosition.Y = Motor.CurrentPosition;
				break;
			case EMotorAxis::Z:
				TargetPosition.Z = Motor.CurrentPosition;
				break;
		}

		Motor.CachedConstraint->SetLinearPositionTarget(TargetPosition);
		Motor.CachedConstraint->SetLinearVelocityTarget(FVector::ZeroVector);
	}
}

void URammpGen1ControllerComponent::DrawDebugVisualization()
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
		FColor	MotorColor = Motor.bEnabled ? FColor::Green : FColor::Red;

		// Draw sphere at motor location
		DrawDebugSphere(World, MotorLocation, 10.0f, 8, MotorColor, false, 0.0f, 0, 2.0f);

		// Draw axis indicator
		FVector AxisDir = FVector::ZeroVector;
		switch (Motor.ControlAxis)
		{
			case EMotorAxis::X:
				AxisDir = FVector(1, 0, 0);
				break;
			case EMotorAxis::Y:
				AxisDir = FVector(0, 1, 0);
				break;
			case EMotorAxis::Z:
				AxisDir = FVector(0, 0, 1);
				break;
		}
		DrawDebugDirectionalArrow(World, MotorLocation, MotorLocation + AxisDir * GizmoSize,
			10.0f, FColor::Cyan, false, 0.0f, 0, 2.0f);

		// Draw motor info
		FString MotorInfo = FString::Printf(TEXT("%s\n%.1f° -> %.1f°\nSpeed: %.1f deg/s"),
			*Motor.ConstraintName.ToString(), Motor.CurrentAngle, Motor.TargetAngle,
			Motor.MaxSpeed * Motor.SpeedMultiplier);
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
		FColor	MotorColor = Motor.bEnabled ? FColor::Blue : FColor::Orange;

		// Draw box at motor location
		DrawDebugBox(World, MotorLocation, FVector(15.0f), MotorColor, false, 0.0f, 0, 2.0f);

		// Draw axis indicator
		FVector AxisDir = FVector::ZeroVector;
		switch (Motor.ControlAxis)
		{
			case EMotorAxis::X:
				AxisDir = FVector(1, 0, 0);
				break;
			case EMotorAxis::Y:
				AxisDir = FVector(0, 1, 0);
				break;
			case EMotorAxis::Z:
				AxisDir = FVector(0, 0, 1);
				break;
		}
		DrawDebugDirectionalArrow(World, MotorLocation, MotorLocation + AxisDir * GizmoSize,
			10.0f, FColor::Magenta, false, 0.0f, 0, 2.0f);

		// Draw motor info
		FString MotorInfo = FString::Printf(TEXT("%s\n%.1fcm -> %.1fcm\nSpeed: %.1f cm/s"),
			*Motor.ConstraintName.ToString(), Motor.CurrentPosition, Motor.TargetPosition,
			Motor.MaxSpeed * Motor.SpeedMultiplier);
		DrawDebugString(World, MotorLocation + FVector(0, 0, 20.0f),
			MotorInfo, nullptr, FColor::Yellow, 0.0f, true, 1.0f);

		YOffset += 1.0f;
	}
}

// --- control surface -----------------------------------------------------------

FName URammpGen1ControllerComponent::ControlIdFor(FName ConstraintName)
{
	return *FString::Printf(TEXT("lift.%s"), *ConstraintName.ToString());
}

FName URammpGen1ControllerComponent::ConstraintFor(FName ControlId)
{
	FString S = ControlId.ToString();
	return S.RemoveFromStart(TEXT("lift.")) ? FName(*S) : NAME_None;
}

void URammpGen1ControllerComponent::DescribeControls(FRammsControlSurface& OutSurface) const
{
	const bool bViaBase = IsUsingRobotBase();
	auto GroupFor = [](FName Constraint) { return Constraint.ToString().Contains(TEXT("caster")) ? FName("Casters") : FName("Lift"); };
	auto RegistryRange = [&](FName MotorId, FVector2D& Out) {
		FRammsMotorSpec Spec;
		if (bViaBase && !MotorId.IsNone() && RobotBase->GetMotorSpec(MotorId, Spec) && Spec.ControlRange.X < Spec.ControlRange.Y)
		{
			Out = Spec.ControlRange;
			return true;
		}
		return false;
	};
	int32 Order = 0;
	for (const FAngularMotorConfig& M : AngularMotors)
	{
		FRammsControlAxis Axis;
		Axis.Id = ControlIdFor(M.ConstraintName);
		Axis.Group = GroupFor(M.ConstraintName);
		Axis.DisplayName = FText::FromString(M.ConstraintName.ToString().Replace(TEXT("_"), TEXT(" ")));
		Axis.Kind = ERammsControlKind::Position;
		Axis.Units = ERammsControlUnits::Degrees;
		Axis.Order = Order++;
		FVector2D Range;
		if (RegistryRange(M.ResolvedMotorId, Range))
		{
			Axis.Range = FVector2D(FMath::RadiansToDegrees(Range.X), FMath::RadiansToDegrees(Range.Y));
		}
		else if (M.CachedConstraint)
		{
			const float Limit = M.ControlAxis == EMotorAxis::X ? M.CachedConstraint->GetAngularTwistLimit()
				: M.ControlAxis == EMotorAxis::Y				  ? M.CachedConstraint->GetAngularSwing2Limit()
																  : M.CachedConstraint->GetAngularSwing1Limit();
			Axis.Range = Limit > 0.0f ? FVector2D(-Limit, Limit) : FVector2D(-90.0, 90.0);
		}
		else
		{
			Axis.Range = FVector2D(-90.0, 90.0);
		}
		OutSurface.Add(Axis);
	}
	for (const FLinearMotorConfig& M : LinearMotors)
	{
		FRammsControlAxis Axis;
		Axis.Id = ControlIdFor(M.ConstraintName);
		Axis.Group = GroupFor(M.ConstraintName);
		Axis.DisplayName = FText::FromString(M.ConstraintName.ToString().Replace(TEXT("_"), TEXT(" ")));
		Axis.Kind = ERammsControlKind::Position;
		Axis.Units = ERammsControlUnits::Centimeters;
		Axis.Order = Order++;
		FVector2D Range;
		if (RegistryRange(M.ResolvedMotorId, Range))
		{
			Axis.Range = Range;
		}
		else if (M.CachedConstraint && M.CachedConstraint->GetLinearLimit() > 0.0f)
		{
			Axis.Range = FVector2D(-M.CachedConstraint->GetLinearLimit(), M.CachedConstraint->GetLinearLimit());
		}
		else
		{
			Axis.Range = FVector2D(-20.0, 20.0);
		}
		OutSurface.Add(Axis);
	}
}

bool URammpGen1ControllerComponent::ApplyControl(FName Id, float Value)
{
	// A motor another mode drives (a curb crossing's lifts) refuses commands until it is back.
	const FName Constraint = ConstraintFor(Id);
	const bool	bViaBase = IsUsingRobotBase();
	for (FAngularMotorConfig& M : AngularMotors)
	{
		if (M.ConstraintName == Constraint)
		{
			if (bViaBase && RammpGen1MotorYielded(this, M.ResolvedMotorId))
			{
				return false;
			}
			if (!M.bEnabled)
			{
				SetAngularMotorEnabled(Constraint, true);
			}
			SetAngularMotorTarget(Constraint, Value);
			return true;
		}
	}
	for (FLinearMotorConfig& M : LinearMotors)
	{
		if (M.ConstraintName == Constraint)
		{
			if (bViaBase && RammpGen1MotorYielded(this, M.ResolvedMotorId))
			{
				return false;
			}
			if (!M.bEnabled)
			{
				SetLinearMotorEnabled(Constraint, true);
			}
			SetLinearMotorTarget(Constraint, Value);
			return true;
		}
	}
	return false;
}

bool URammpGen1ControllerComponent::ReleaseControl(FName Id)
{
	// Nor does it let go of one: the other mode holds it.
	const FName Constraint = ConstraintFor(Id);
	const bool	bViaBase = IsUsingRobotBase();
	for (const FAngularMotorConfig& M : AngularMotors)
	{
		if (M.ConstraintName == Constraint)
		{
			if (bViaBase && RammpGen1MotorYielded(this, M.ResolvedMotorId))
			{
				return false;
			}
			SetAngularMotorEnabled(Constraint, false);
			return !M.bEnabled; // stays enabled when the backend can't let go
		}
	}
	for (const FLinearMotorConfig& M : LinearMotors)
	{
		if (M.ConstraintName == Constraint)
		{
			if (bViaBase && RammpGen1MotorYielded(this, M.ResolvedMotorId))
			{
				return false;
			}
			SetLinearMotorEnabled(Constraint, false);
			return !M.bEnabled;
		}
	}
	return false;
}

bool URammpGen1ControllerComponent::ReadControl(FName Id, float& OutValue) const
{
	const FName Constraint = ConstraintFor(Id);
	for (const FAngularMotorConfig& M : AngularMotors)
	{
		if (M.ConstraintName == Constraint)
		{
			OutValue = GetAngularMotorCurrentAngle(Constraint);
			return true;
		}
	}
	for (const FLinearMotorConfig& M : LinearMotors)
	{
		if (M.ConstraintName == Constraint)
		{
			OutValue = GetLinearMotorCurrentPosition(Constraint);
			return true;
		}
	}
	return false;
}

void URammpGen1ControllerComponent::GetClaimedMotorIds(TArray<FName>& OutIds) const
{
	for (const FAngularMotorConfig& M : AngularMotors)
	{
		if (!M.ResolvedMotorId.IsNone())
		{
			OutIds.Add(M.ResolvedMotorId);
		}
	}
	for (const FLinearMotorConfig& M : LinearMotors)
	{
		if (!M.ResolvedMotorId.IsNone())
		{
			OutIds.Add(M.ResolvedMotorId);
		}
	}
}

bool URammpGen1ControllerComponent::ReadTarget(FName Id, float& OutTarget) const
{
	// The motor configs hold the commanded target (SetAngularMotorTarget /
	// SetLinearMotorTarget write it), whichever path set it.
	const FName Constraint = ConstraintFor(Id);
	// A released motor (disabled drive) is not holding a target any more.
	for (const FAngularMotorConfig& M : AngularMotors)
	{
		if (M.ConstraintName == Constraint)
		{
			OutTarget = M.TargetAngle;
			return M.bEnabled;
		}
	}
	for (const FLinearMotorConfig& M : LinearMotors)
	{
		if (M.ConstraintName == Constraint)
		{
			OutTarget = M.TargetPosition;
			return M.bEnabled;
		}
	}
	return false;
}
