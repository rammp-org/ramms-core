// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsHolonomicDriveController.h"

#include "GameFramework/Actor.h"
#include "RammsRobotBaseComponent.h"

URammsHolonomicDriveController::URammsHolonomicDriveController()
{
	// Wheel rates are recomputed from the held command every frame.
	PrimaryComponentTick.bCanEverTick = true;
}

void URammsHolonomicDriveController::BeginPlay()
{
	Super::BeginPlay();
	ResolveSpec();

	if (Resolved.Wheels.Num() < 3)
	{
		// Two wheels cannot span the plane, so the base would not be holonomic
		// whatever is commanded. Say so rather than driving something that
		// silently cannot strafe.
		UE_LOG(LogTemp, Warning,
			TEXT("[Holonomic] '%s' has %d wheels configured; at least 3 independent roll "
				 "directions are needed to command forward, strafe and yaw."),
			*GetName(), Resolved.Wheels.Num());
	}
}

void URammsHolonomicDriveController::ResolveSpec()
{
	Resolved = Drive;
	if (KinematicTable && !DriveRow.IsNone())
	{
		if (const FRammsHolonomicDriveSpec* Row =
				KinematicTable->FindRow<FRammsHolonomicDriveSpec>(DriveRow, TEXT("Holonomic")))
		{
			Resolved = *Row;
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("[Holonomic] '%s': row '%s' not in %s; using the inline spec."),
				*GetName(), *DriveRow.ToString(), *KinematicTable->GetName());
		}
	}
}

URammsRobotBaseComponent* URammsHolonomicDriveController::EnsureBase() const
{
	if (!BaseComponent)
	{
		if (const AActor* Owner = GetOwner())
		{
			BaseComponent = Owner->FindComponentByClass<URammsRobotBaseComponent>();
		}
	}
	return BaseComponent;
}

bool URammsHolonomicDriveController::HasBase() const
{
	return EnsureBase() != nullptr;
}

void URammsHolonomicDriveController::SetDriveCommand(FVector ForwardStrafeYaw)
{
	Command.X = FMath::Clamp(ForwardStrafeYaw.X, -1.0, 1.0);
	Command.Y = FMath::Clamp(ForwardStrafeYaw.Y, -1.0, 1.0);
	Command.Z = FMath::Clamp(ForwardStrafeYaw.Z, -1.0, 1.0);
}

TArray<float> URammsHolonomicDriveController::SolveWheelRates() const
{
	TArray<float> Rates;
	Rates.Reserve(Resolved.Wheels.Num());

	// Body twist in real units: cm/s and rad/s.
	const double Vx = Command.X * Resolved.MaxLinearSpeedCmPerSecond;
	const double Vy = Command.Y * Resolved.MaxLinearSpeedCmPerSecond;
	const double Omega = FMath::DegreesToRadians(Command.Z * Resolved.MaxYawRateDegPerSecond);
	const double Radius = FMath::Max(Resolved.WheelRadiusCm, 0.1f);

	for (const FRammsOmniWheelSpec& Wheel : Resolved.Wheels)
	{
		const double Theta = FMath::DegreesToRadians(Wheel.RollDirectionDeg);
		const double Dx = FMath::Cos(Theta);
		const double Dy = FMath::Sin(Theta);

		// The wheel's ground speed is the body velocity at its contact point,
		// projected onto the direction it can actually push:
		//   v_i = d . (v + omega x r)
		// with the cross product in 2D reducing to omega * (x*dy - y*dx).
		const double Speed =
			Dx * Vx + Dy * Vy + Omega * (Wheel.Position.X * Dy - Wheel.Position.Y * Dx);

		const double Rate = (Wheel.bInvert ? -Speed : Speed) / Radius;
		Rates.Add(static_cast<float>(Rate));
	}
	return Rates;
}

void URammsHolonomicDriveController::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	URammsRobotBaseComponent* Base = EnsureBase();
	if (!Base || Resolved.Wheels.Num() == 0)
	{
		return;
	}

	const bool bIdle = Command.IsNearlyZero();
	if (bIdle && !bDrivingMotors)
	{
		// Nothing commanded and nothing left running: leave the motors alone so
		// this does not fight anything else driving them.
		return;
	}

	const TArray<float> Rates = SolveWheelRates();
	for (int32 i = 0; i < Resolved.Wheels.Num() && i < Rates.Num(); ++i)
	{
		if (!Resolved.Wheels[i].MotorId.IsNone())
		{
			Base->SetMotorCommand(Resolved.Wheels[i].MotorId, Rates[i]);
		}
	}
	// One more pass of zeros after the stick centres, then stop writing.
	bDrivingMotors = !bIdle;
}

// --- control surface -----------------------------------------------------------

void URammsHolonomicDriveController::DescribeControls(FRammsControlSurface& OutSurface) const
{
	FRammsControlAxis Forward;
	Forward.Id = RammsControlIds::Drive::Forward();
	Forward.Group = RammsControlIds::Groups::Drive();
	Forward.DisplayName = NSLOCTEXT("Ramms", "HolonomicForward", "Forward");
	Forward.Kind = ERammsControlKind::Continuous;
	Forward.Units = ERammsControlUnits::Normalized;
	Forward.Range = FVector2D(-1.0, 1.0);
	Forward.Order = 0;
	// Forward and strafe are the pair a stick moves together; yaw is its own
	// control, the way a twist axis is.
	Forward.PairedAxis = RammsControlIds::Drive::Strafe();
	OutSurface.Add(Forward);

	FRammsControlAxis Strafe = Forward;
	Strafe.Id = RammsControlIds::Drive::Strafe();
	Strafe.DisplayName = NSLOCTEXT("Ramms", "HolonomicStrafe", "Strafe");
	Strafe.Order = 1;
	Strafe.PairedAxis = RammsControlIds::Drive::Forward();
	OutSurface.Add(Strafe);

	FRammsControlAxis Turn = Forward;
	Turn.Id = RammsControlIds::Drive::Turn();
	Turn.DisplayName = NSLOCTEXT("Ramms", "HolonomicTurn", "Turn");
	Turn.Order = 2;
	Turn.PairedAxis = FName();
	OutSurface.Add(Turn);
}

bool URammsHolonomicDriveController::ApplyControl(FName Id, float Value)
{
	if (Id == RammsControlIds::Drive::Forward())
	{
		Command.X = FMath::Clamp(Value, -1.0f, 1.0f);
		return true;
	}
	if (Id == RammsControlIds::Drive::Strafe())
	{
		Command.Y = FMath::Clamp(Value, -1.0f, 1.0f);
		return true;
	}
	if (Id == RammsControlIds::Drive::Turn())
	{
		Command.Z = FMath::Clamp(Value, -1.0f, 1.0f);
		return true;
	}
	return false;
}

bool URammsHolonomicDriveController::ReleaseControl(FName Id)
{
	if (Id == RammsControlIds::Drive::Forward())
	{
		Command.X = 0.0;
		return true;
	}
	if (Id == RammsControlIds::Drive::Strafe())
	{
		Command.Y = 0.0;
		return true;
	}
	if (Id == RammsControlIds::Drive::Turn())
	{
		Command.Z = 0.0;
		return true;
	}
	return false;
}

bool URammsHolonomicDriveController::ReadControl(FName Id, float& OutValue) const
{
	if (Id == RammsControlIds::Drive::Forward())
	{
		OutValue = static_cast<float>(Command.X);
		return true;
	}
	if (Id == RammsControlIds::Drive::Strafe())
	{
		OutValue = static_cast<float>(Command.Y);
		return true;
	}
	if (Id == RammsControlIds::Drive::Turn())
	{
		OutValue = static_cast<float>(Command.Z);
		return true;
	}
	return false;
}

void URammsHolonomicDriveController::GetClaimedMotorIds(TArray<FName>& OutIds) const
{
	for (const FRammsOmniWheelSpec& Wheel : Resolved.Wheels)
	{
		if (!Wheel.MotorId.IsNone())
		{
			OutIds.Add(Wheel.MotorId);
		}
	}
}
