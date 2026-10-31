// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsHolonomicDriveController.h"

#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "RammsDriveHandOver.h"
#include "RammsRobotBaseComponent.h"

URammsHolonomicDriveController::URammsHolonomicDriveController()
{
	// Wheel rates are recomputed from the held command every frame.
	PrimaryComponentTick.bCanEverTick = true;
	// PrePhysics, matching the robot base. A UActorComponent defaults to
	// TG_DuringPhysics, so without this the base -- which is a tick dependent
	// of this one -- was dragged into the physics phase with it, and whether
	// the torque the velocity loop writes reached the current step was down to
	// timing.
	PrimaryComponentTick.TickGroup = TG_PrePhysics;

	// Enough on its own to lift the centre wheels clear. A robot that also
	// drives its corner cranks down gets the same clearance lower -- see
	// FRammsDriveStance -- but those motor Ids belong to the robot.
	LiftedStance.bHasLinkageHeight = true;
	LiftedStance.LinkageHeightCm = 13.0f;
}

void URammsHolonomicDriveController::BeginPlay()
{
	Super::BeginPlay();
	ResolveSpec();

	// TickGroup is serialised on the Blueprint's component template, so a
	// controller authored before the line above keeps TG_DuringPhysics however
	// the CDO is set. Put it back before anything depends on the order.
	PrimaryComponentTick.TickGroup = TG_PrePhysics;

	// The base closes the velocity loop in its own tick, in the same tick
	// group. Without an explicit order it may run first and use last frame's
	// targets.
	if (URammsRobotBaseComponent* Base = EnsureBase())
	{
		Base->AddTickPrerequisiteComponent(this);
	}

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

void URammsHolonomicDriveController::SetDriveModeActive(bool bActive)
{
	if (bDriveModeActive == bActive)
	{
		return;
	}
	bDriveModeActive = bActive;
	bHoldInactive = false;
	// Either way this mode starts and ends at rest -- unless the selector
	// hands it the robot's motion (SeedMotion) right after taking over.
	// Taking over matters as much as standing down: a SetDriveCommand that
	// arrived while this mode was inactive is still sitting in Command -- Tick
	// refuses to apply it, but activating would then lurch the base on a
	// command nobody issued for it.
	Command = FVector::ZeroVector;
	ResetSwitching();
	if (URammsRobotBaseComponent* Base = EnsureBase())
	{
		// Whatever held the wheels before holds them no more: this mode's own
		// hold, or the switch it was stood down for, brakes them afresh.
		RammsDrive::ReleaseBrakes(*Base, BrakedWheels);
	}
	if (!bActive)
	{
		// Zero the wheels once rather than leaving them spinning at whatever
		// the last command was.
		if (URammsRobotBaseComponent* Base = EnsureBase())
		{
			for (const FRammsOmniWheelSpec& Wheel : Resolved.Wheels)
			{
				if (!Wheel.MotorId.IsNone())
				{
					Base->ClearMotorVelocityCommand(Wheel.MotorId);
					Base->SetMotorCommand(Wheel.MotorId, 0.0f);
				}
			}
		}
		bDrivingMotors = false;
	}
}

void URammsHolonomicDriveController::SetDriveCommand(FVector ForwardStrafeYaw)
{
	Command.X = FMath::Clamp(ForwardStrafeYaw.X, -1.0, 1.0);
	Command.Y = FMath::Clamp(ForwardStrafeYaw.Y, -1.0, 1.0);
	Command.Z = FMath::Clamp(ForwardStrafeYaw.Z, -1.0, 1.0);
}

TArray<float> URammsHolonomicDriveController::SolveWheelRates() const
{
	return SolveWheelRatesFor(RammsDrive::HolonomicMotion(Command, Resolved.MaxLinearSpeedCmPerSecond, Resolved.MaxYawRateDegPerSecond));
}

TArray<float> URammsHolonomicDriveController::SolveWheelRatesFor(const FRammsDriveMotion& Motion) const
{
	TArray<float> Rates;
	Rates.Reserve(Resolved.Wheels.Num());

	// Body twist in real units: cm/s and rad/s.
	const double Vx = Motion.ForwardCmPerSecond;
	const double Vy = Motion.RightCmPerSecond;
	const double Omega = FMath::DegreesToRadians(static_cast<double>(Motion.YawRateDegPerSecond));
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
	const double			  T = Now();
	double					  Dt = 0.0;
	if (RammsDrive::AdvanceHoldClock(T, FirstTickTime, LastTickTime, Dt))
	{
		// The simulation was reset: the robot is back at its start pose, every
		// wheel's angle with it. Nothing held before stands: a wheel still held
		// for a switch is held again where it is now; the mode's own hold
		// waits out the spawn's second again.
		bHoldingStill = false;
		HoldAngles.Reset();
		if (Base)
		{
			RammsDrive::ReleaseBrakes(*Base, BrakedWheels);
		}
		if (Base && bHoldInactive)
		{
			LatchHold(*Base);
		}
	}
	// Nothing held while the robot is put down as it spawns.
	const bool bMayHold = bHoldWhenIdle && RammsDrive::HoldWhenIdleEnabled() && T - FirstTickTime >= RammsDrive::SpawnSettleSeconds;
	if (Base && bHoldInactive && !bDriveModeActive)
	{
		// Stood down at rest while the next mode's wheels go down: these still
		// carry the robot, held where they are.
		StepHoldBrakes(*Base);
		CommandHold(*Base);
		return;
	}
	if (!Base || Resolved.Wheels.Num() == 0 || !bDriveModeActive)
	{
		return;
	}

	// What to drive: the command -- nothing while the stance is still putting
	// this mode's wheels down, and within what the other mode's wheels carry
	// while a hand-over bound holds -- reached at the hand-over rates during a
	// switch (from the motion the robot had, after one on the move), at once
	// otherwise.
	const FRammsDriveMotion Commanded = RammsDrive::HolonomicMotion(Command, Resolved.MaxLinearSpeedCmPerSecond, Resolved.MaxYawRateDegPerSecond);
	FRammsDriveMotion		Target = Commanded;
	const IRammsDriveMode*	Bound = Cast<IRammsDriveMode>(HandOverTarget.Get());
	if (bHeldForStance)
	{
		Target = FRammsDriveMotion();
	}
	else if (Bound)
	{
		Target = Bound->GetCarriableMotion(Commanded);
	}
	// Holding still: a command let go brings the robot to rest at the
	// hand-over rates, not at once -- its wheels are then held (below) -- and
	// a command given while held is taken up from rest at those rates, so the
	// hold lets go smoothly (on a slope the speed loops' integrals carry the
	// pull on).
	const bool bNothingAsked = Target.IsNearlyZero(1e-3f, 1e-3f);
	if (bMayHold && ((bNothingAsked && !Output.IsNearlyZero(1e-3f, 1e-3f)) || (!bNothingAsked && bHoldingStill)))
	{
		bOutputRamping = true;
	}
	if (bOutputRamping || Bound || bHeldForStance)
	{
		Output = RammsDrive::RampMotion(Output, Target, HandOverAcceleration, HandOverYawAcceleration, Dt);
		if (!Bound && !bHeldForStance && RammsDrive::MotionsEqual(Output, Target, 1e-2))
		{
			bOutputRamping = false;
		}
	}
	else
	{
		Output = Target;
	}

	const bool bIdle = Output.IsNearlyZero(1e-3f, 1e-3f);
	if (bMayHold && bIdle && bNothingAsked)
	{
		// At rest with nothing asked (or its wheels still going down): every
		// wheel held where it stopped, its speed loop kept on and asked back
		// towards that angle. On a slope the loop's integral takes the pull.
		if (!bHoldingStill || HoldAngles.Num() != Resolved.Wheels.Num())
		{
			LatchHold(*Base);
			bHoldingStill = true;
		}
		StepHoldBrakes(*Base);
		CommandHold(*Base);
		bDrivingMotors = true;
		return;
	}
	bHoldingStill = false;
	RammsDrive::ReleaseBrakes(*Base, BrakedWheels);
	if (bIdle && !bDrivingMotors)
	{
		// Nothing commanded and nothing left running: leave the motors alone so
		// this does not fight anything else driving them.
		return;
	}

	// Rates are rad/s, so they go out as a speed command. The base closes the
	// loop when the actuator is a torque motor -- writing these numbers into
	// SetMotorCommand applies them as newton-metres instead, which turned the
	// wheels at about 3% of the commanded rate and moved the base 2 cm.
	const TArray<float> Rates = SolveWheelRatesFor(Output);
	for (int32 i = 0; i < Resolved.Wheels.Num() && i < Rates.Num(); ++i)
	{
		if (!Resolved.Wheels[i].MotorId.IsNone())
		{
			Base->SetMotorVelocityCommand(Resolved.Wheels[i].MotorId, Rates[i]);
		}
	}

	if (bIdle)
	{
		// A velocity command persists in the base until something drops it, so
		// "stop writing" is no longer enough to stop driving: the loop would
		// go on holding these wheels at zero every tick, braking them and
		// fighting anyone else who touches them. Hand them back instead, which
		// is what this did when it was writing torque directly.
		for (const FRammsOmniWheelSpec& Wheel : Resolved.Wheels)
		{
			if (!Wheel.MotorId.IsNone())
			{
				Base->ClearMotorVelocityCommand(Wheel.MotorId);
				Base->SetMotorCommand(Wheel.MotorId, 0.0f);
			}
		}
	}
	bDrivingMotors = !bIdle;
}

void URammsHolonomicDriveController::LatchHold(URammsRobotBaseComponent& Base)
{
	HoldAngles.SetNum(Resolved.Wheels.Num());
	for (int32 i = 0; i < Resolved.Wheels.Num(); ++i)
	{
		HoldAngles[i] = Resolved.Wheels[i].MotorId.IsNone() ? 0.0 : Base.GetMotorValue(Resolved.Wheels[i].MotorId);
	}
}

void URammsHolonomicDriveController::CommandHold(URammsRobotBaseComponent& Base) const
{
	for (int32 i = 0; i < Resolved.Wheels.Num() && i < HoldAngles.Num(); ++i)
	{
		const FName Motor = Resolved.Wheels[i].MotorId;
		if (!Motor.IsNone() && !BrakedWheels.Contains(Motor))
		{
			RammsDrive::FWheelHold Hold;
			Hold.Latch(HoldAngles[i]);
			Base.SetMotorVelocityCommand(Motor, static_cast<float>(Hold.Rate(Base.GetMotorValue(Motor))));
		}
	}
}

void URammsHolonomicDriveController::HoldWheelsWhileInactive(bool bHold)
{
	URammsRobotBaseComponent* Base = EnsureBase();
	if (bDriveModeActive || !Base)
	{
		bHoldInactive = false;
		return;
	}
	if (bHold && bHoldWhenIdle)
	{
		LatchHold(*Base);
		bHoldInactive = true;
		CommandHold(*Base);
		return;
	}
	if (bHoldInactive)
	{
		// Let go, as standing down does.
		bHoldInactive = false;
		RammsDrive::ReleaseBrakes(*Base, BrakedWheels);
		for (const FRammsOmniWheelSpec& Wheel : Resolved.Wheels)
		{
			if (!Wheel.MotorId.IsNone())
			{
				Base->ClearMotorVelocityCommand(Wheel.MotorId);
				Base->SetMotorCommand(Wheel.MotorId, 0.0f);
			}
		}
	}
}

void URammsHolonomicDriveController::StepHoldBrakes(URammsRobotBaseComponent& Base)
{
	// The hubs' own rates say whether the robot is at rest: three or more omni
	// wheels rolling along different lines fix the base's motion between them,
	// so all of them still is the robot still -- and a braked hub turning is
	// its brake letting the robot go.
	const double Radius = FMath::Max(Resolved.WheelRadiusCm, 0.1f);
	double		 Fastest = 0.0;
	for (const FRammsOmniWheelSpec& Wheel : Resolved.Wheels)
	{
		if (!Wheel.MotorId.IsNone())
		{
			Fastest = FMath::Max(Fastest, FMath::Abs(static_cast<double>(Base.GetMotorVelocity(Wheel.MotorId))) * Radius);
		}
	}
	if (BrakedWheels.Num() > 0 && Fastest > RammsDrive::BrakeSlipCmS)
	{
		RammsDrive::ReleaseBrakes(Base, BrakedWheels);
		LatchHold(Base);
	}
	else if (BrakedWheels.Num() == 0 && Fastest < RammsDrive::BrakeAtRestCmS)
	{
		TArray<FName, TInlineAllocator<8>> Motors;
		for (const FRammsOmniWheelSpec& Wheel : Resolved.Wheels)
		{
			Motors.Add(Wheel.MotorId);
		}
		RammsDrive::EngageBrakes(Base, Motors, BrakedWheels);
	}
}

// --- switching on the move -------------------------------------------------------

double URammsHolonomicDriveController::Now() const
{
	// The physics clock where there is one: the hand-over rates are physical.
	if (const URammsRobotBaseComponent* Base = EnsureBase())
	{
		const double Sim = Base->GetSimulationTime();
		if (Sim >= 0.0)
		{
			return Sim;
		}
	}
	const UWorld* World = GetWorld();
	return World ? World->GetTimeSeconds() : 0.0;
}

void URammsHolonomicDriveController::ResetSwitching()
{
	Output = FRammsDriveMotion();
	HandOverTarget.Reset();
	bOutputRamping = false;
	bHeldForStance = false;
	bHoldingStill = false;
}

bool URammsHolonomicDriveController::BeginHandOver(const IRammsDriveMode& Other)
{
	if (!bDriveModeActive)
	{
		return false;
	}
	HandOverTarget = Cast<const UObject>(&Other);
	// From the motion driven now, not from the projected command at once.
	bOutputRamping = true;
	return HandOverTarget.IsValid();
}

void URammsHolonomicDriveController::EndHandOver()
{
	HandOverTarget.Reset();
	// Back to the operator's command at the same rate. (Stood down instead,
	// SetDriveModeActive resets everything.)
	bOutputRamping = bDriveModeActive;
}

void URammsHolonomicDriveController::HoldForStance(bool bHold)
{
	bHeldForStance = bHold && bDriveModeActive;
	bOutputRamping = bDriveModeActive;
}

void URammsHolonomicDriveController::SeedMotion(const FRammsDriveMotion& Motion, float HoldSeconds)
{
	if (!bDriveModeActive)
	{
		return;
	}
	// Where this mode starts, not a setpoint it keeps: from here to the
	// operator's command (within any bound the selector sets after this) at
	// the hand-over rate, so that a stick held through the switch carries on
	// at its own speed and one let go brings the robot to rest. A command not
	// yet heard on the first tick costs one tick's worth of that rate.
	Output = Motion;
	bOutputRamping = true;
}

// --- control surface -----------------------------------------------------------

void URammsHolonomicDriveController::DescribeControls(FRammsControlSurface& OutSurface) const
{
	if (!bDriveModeActive)
	{
		// Another drive mode owns drive.forward / drive.turn right now.
		return;
	}
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
	if (!bDriveModeActive)
	{
		return false;
	}
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
	// Same gate as ApplyControl: an inactive mode owns none of these axes, and
	// answering "released" for a control it does not contribute would have the
	// surface record a hold against it.
	if (!bDriveModeActive)
	{
		return false;
	}
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
	if (!bDriveModeActive)
	{
		// Standing down: let the wheels show up as raw motor axes instead.
		return;
	}
	for (const FRammsOmniWheelSpec& Wheel : Resolved.Wheels)
	{
		if (!Wheel.MotorId.IsNone())
		{
			OutIds.Add(Wheel.MotorId);
		}
	}
}
