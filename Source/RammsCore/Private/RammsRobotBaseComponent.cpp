// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRobotBaseComponent.h"
#include "RammsMotionProfile.h"
#include "Components/SkeletalMeshComponent.h"
#include "RammsActuationBackend.h"
#include "RammsActuationBackendRegistry.h"
#include "RammsChaosActuationBackend.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "UObject/UObjectIterator.h"

namespace
{
	TAutoConsoleVariable<int32> CVarPhysicsRateProfiles(TEXT("Ramms.Motor.PhysicsRateProfiles"), 1,
		TEXT("1 (the default): a profiled position motor's setpoint is eased every physics step by a backend that can "
			 "(MuJoCo), each command reached along a line over the time since the one before. 0: once per game tick "
			 "here, as before -- a staircase at the frame rate."),
		ECVF_Default);
} // namespace

URammsRobotBaseComponent::URammsRobotBaseComponent()
{
	// Ticks only to close the velocity loop; see SetMotorVelocityCommand.
	// PrePhysics, so the torque it writes is applied in the same step as the
	// commands the drive controllers wrote a moment earlier.
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;

	// Defaults tuned on an omni-wheel base in PIE: the gain sits where the loop
	// tracks a steady command closely, and falls away quickly either side. A
	// robot whose wheels differ can override these per motor in its registry
	// row.
	DefaultVelocityGains.Kp = 2.5f;
	DefaultVelocityGains.Ki = 5.0f;
	DefaultVelocityGains.MaxIntegralTorque = 10.0f;
}

URammsRobotBaseComponent::~URammsRobotBaseComponent()
{
	// IRammsActuationBackend has a virtual destructor; complete type here.
	delete Backend_;
	Backend_ = nullptr;
}

void URammsRobotBaseComponent::BeginPlay()
{
	Super::BeginPlay();

	// bCanEverTick is serialised on the Blueprint's component template, so a
	// robot authored before this component ticked keeps the old false however
	// the CDO is set -- and its velocity loop would never run, leaving every
	// torque-actuated wheel with no command at all. Turn it on for this
	// instance rather than asking everyone to re-save their Blueprints. (The
	// 5-bar does the same for its jog axes.)
	// TickGroup is serialised the same way, so an old template also keeps
	// TG_DuringPhysics -- enabling its tick without this would run the
	// velocity loop inside the physics phase. Set unconditionally, and before
	// registering, since registration reads it.
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	if (!PrimaryComponentTick.bCanEverTick)
	{
		PrimaryComponentTick.bCanEverTick = true;
		PrimaryComponentTick.SetTickFunctionEnable(true);
		PrimaryComponentTick.RegisterTickFunction(GetComponentLevel());
	}
	LoadMotorRegistry();
	EnsureBackend();
}

void URammsRobotBaseComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	ClearAllMotorTorqueLimits();
	delete Backend_;
	Backend_ = nullptr;
	bBackendResolved = false;
	bMotorsLoaded = false;
	PositionProfiles.Reset();
	LastCommands.Reset();
	CommandOffsets.Reset();
	CommandOffsetLayer.Reset();
	Super::EndPlay(EndPlayReason);
}

void URammsRobotBaseComponent::LoadMotorRegistry() const
{
	// Idempotent, and callable from any accessor: a sibling component may query
	// or command motors from its own BeginPlay before ours has run, and those
	// calls must already see the configured type / range / direction / names.
	if (bMotorsLoaded)
	{
		return;
	}
	bMotorsLoaded = true;
	Motors.Reset();
	MotorOrder.Reset();
	if (!MotorTable)
	{
		return;
	}

	static const FString Context(TEXT("RammsRobotBaseComponent"));
	MotorTable->ForeachRow<FRammsMotorSpec>(Context,
		[this](const FName& RowName, const FRammsMotorSpec& Row) {
			// Id defaults to the row name when the row leaves it unset.
			FRammsMotorSpec Spec = Row;
			if (Spec.Id.IsNone())
			{
				Spec.Id = RowName;
			}
			// A duplicate Id would silently replace an earlier row and make the
			// routing depend on table iteration order: keep the first, report it.
			if (Motors.Contains(Spec.Id))
			{
				UE_LOG(LogTemp, Error,
					TEXT("RammsRobotBaseComponent on '%s': motor table '%s' row '%s' duplicates motor Id '%s' — row ignored (first definition wins)."),
					GetOwner() ? *GetOwner()->GetName() : TEXT("?"), *MotorTable->GetName(),
					*RowName.ToString(), *Spec.Id.ToString());
				return;
			}
			Motors.Add(Spec.Id, Spec);
			MotorOrder.Add(Spec.Id);
		});

	// Non-const delegate on a const path: the registry is logically const state
	// loaded lazily; announcing it doesn't change the component.
	const_cast<URammsRobotBaseComponent*>(this)->OnMotorRegistryLoaded.Broadcast(const_cast<URammsRobotBaseComponent*>(this));
}

void URammsRobotBaseComponent::EnsureBackend() const
{
	if (bBackendResolved)
	{
		return;
	}
	// Only resolve inside a running game world: backends look up live actors /
	// components, which don't exist for the CDO or an editor-preview instance.
	const UWorld* World = GetWorld();
	if (!World || !World->IsGameWorld())
	{
		return;
	}
	// The registry must be in place before a backend initializes against it.
	LoadMotorRegistry();
	bBackendResolved = true;

	// Resolve the backend once.
	//  - Mujoco: the MuJoCo backend from the registry (RammsMujocoSupport).
	//  - Chaos:  the Chaos articulation backend from the registry (the robot's
	//            articulation switched to Chaos), else the native Chaos
	//            skeletal backend (RammsCore, deprecated).
	//  - Auto:   try MuJoCo first (an articulation on/under this actor), else
	//            fall back to the skeletal Chaos backend.
	// Initialize() decides whether a backend can actually drive this robot; a
	// backend that can't is discarded, and an unresolved backend leaves motor
	// commands as safe no-ops.
	URammsRobotBaseComponent& Self = *const_cast<URammsRobotBaseComponent*>(this);

	if (Backend == ERammsPhysicsBackend::Mujoco || Backend == ERammsPhysicsBackend::Auto)
	{
		if (IRammsActuationBackend* Mj = RammsActuationBackends::CreateMujocoBackend(Self))
		{
			if (Mj->Initialize(Self))
			{
				Backend_ = Mj;
			}
			else
			{
				delete Mj;
			}
		}
	}

	if (Backend == ERammsPhysicsBackend::Chaos)
	{
		if (IRammsActuationBackend* Art = RammsActuationBackends::CreateChaosArticulationBackend(Self))
		{
			if (Art->Initialize(Self))
			{
				Backend_ = Art;
			}
			else
			{
				delete Art;
			}
		}
	}

	if (!Backend_ && (Backend == ERammsPhysicsBackend::Chaos || Backend == ERammsPhysicsBackend::Auto))
	{
		IRammsActuationBackend* Chaos = new FRammsChaosActuationBackend();
		if (Chaos->Initialize(Self))
		{
			Backend_ = Chaos;
		}
		else
		{
			delete Chaos;
		}
	}

	if (!Backend_)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("RammsRobotBaseComponent on '%s': no actuation backend resolved for mode %d (motor commands are no-ops)."),
			GetOwner() ? *GetOwner()->GetName() : TEXT("?"), static_cast<int32>(Backend));
	}
}

bool URammsRobotBaseComponent::HasBackend() const
{
	EnsureBackend();
	return Backend_ != nullptr;
}

double URammsRobotBaseComponent::GetSimulationTime() const
{
	EnsureBackend();
	return Backend_ ? Backend_->GetSimulationTime() : -1.0;
}

bool URammsRobotBaseComponent::ReleaseMotor(FName MotorId)
{
	// Letting go means letting go: a loop still driving this motor would put
	// it straight back under command on the next tick.
	EnsureBackend();
	DropVelocityDrive(MotorId);
	DropPositionProfile(MotorId);
	LastCommands.Remove(MotorId);
	if (FCommandOffset* Layered = CommandOffsets.Find(MotorId))
	{
		// Its offset goes with its command: one left standing would be added
		// to whatever commands the motor next, from a layer that may not know
		// it was let go.
		*Layered = FCommandOffset();
	}
	return Backend_ && Backend_->ReleaseMotor(MotorId);
}

bool URammsRobotBaseComponent::SetMotorBrake(FName MotorId, bool bEngaged)
{
	EnsureBackend();
	if (bEngaged)
	{
		// A braked wheel is not also driven at a speed.
		DropVelocityDrive(MotorId);
	}
	return Backend_ && Backend_->SetMotorBrake(MotorId, bEngaged);
}

// --- velocity loop ------------------------------------------------------------

void URammsRobotBaseComponent::SetMotorVelocityCommand(FName MotorId, float RadiansPerSecond)
{
	if (MotorId.IsNone())
	{
		return;
	}
	LoadMotorRegistry();
	if (GetMotorType(MotorId) == ERammsActuatorType::Position)
	{
		// A position servo holds an angle; asking it for a speed would mean
		// integrating one here and fighting whatever else is holding it.
		if (!WarnedVelocityOnPosition.Contains(MotorId))
		{
			WarnedVelocityOnPosition.Add(MotorId);
			UE_LOG(LogTemp, Warning,
				TEXT("RammsRobotBaseComponent on '%s': motor '%s' is a Position actuator, so it "
					 "cannot be velocity-driven; command it with SetMotorCommand instead."),
				*GetNameSafe(GetOwner()), *MotorId.ToString());
		}
		return;
	}
	FVelocityDrive& Drive = VelocityDrives.FindOrAdd(MotorId);
	Drive.Target = RadiansPerSecond;
	LastCommands.Remove(MotorId);
}

void URammsRobotBaseComponent::SetDefaultVelocityGains(float Kp, float Ki, float MaxIntegralTorque)
{
	DefaultVelocityGains.Kp = FMath::Max(0.0f, Kp);
	DefaultVelocityGains.Ki = FMath::Max(0.0f, Ki);
	DefaultVelocityGains.MaxIntegralTorque = FMath::Max(0.0f, MaxIntegralTorque);
	// Whatever the old gains had accumulated means nothing under new ones. A
	// loop the backend runs keeps its integral where this cannot zero it, so
	// stop that loop instead; the next tick starts it again from zero, with
	// the gains now in force.
	for (TPair<FName, FVelocityDrive>& Pair : VelocityDrives)
	{
		Pair.Value.Integral = 0.0f;
		if (Backend_)
		{
			Backend_->ClearVelocityDrive(Pair.Key);
		}
	}
	PeakVelocityError = 0.0f;
}

void URammsRobotBaseComponent::ClearMotorVelocityCommand(FName MotorId)
{
	DropVelocityDrive(MotorId);
}

void URammsRobotBaseComponent::DropVelocityDrive(FName MotorId)
{
	if (VelocityDrives.Remove(MotorId) > 0 && Backend_)
	{
		Backend_->ClearVelocityDrive(MotorId);
	}
}

bool URammsRobotBaseComponent::GetMotorVelocityCommand(FName MotorId, float& OutRadiansPerSecond) const
{
	if (const FVelocityDrive* Drive = VelocityDrives.Find(MotorId))
	{
		OutRadiansPerSecond = Drive->Target;
		return true;
	}
	OutRadiansPerSecond = 0.0f;
	return false;
}

FRammsVelocityGains URammsRobotBaseComponent::GainsFor(FName MotorId) const
{
	FRammsMotorSpec Spec;
	if (GetMotorSpec(MotorId, Spec) && Spec.VelocityGains.IsSet())
	{
		return Spec.VelocityGains;
	}
	return DefaultVelocityGains;
}

void URammsRobotBaseComponent::StepVelocityDrives(float DeltaTime)
{
	if (VelocityDrives.Num() == 0 || DeltaTime <= 0.0f)
	{
		return;
	}
	EnsureBackend();
	if (!Backend_)
	{
		return;
	}

	for (TPair<FName, FVelocityDrive>& Pair : VelocityDrives)
	{
		const FName		MotorId = Pair.Key;
		FVelocityDrive& Drive = Pair.Value;

		FRammsMotorSpec Spec;
		const bool		bHasSpec = GetMotorSpec(MotorId, Spec);
		const bool		bBounded = bHasSpec && Spec.ControlRange.X < Spec.ControlRange.Y;

		if (GetMotorType(MotorId) == ERammsActuatorType::Velocity)
		{
			// The actuator closes its own loop; hand it the rate -- clamped to
			// the authored range like any other command, since for a Velocity
			// actuator that range is in rad/s and SetMotorCommand would have
			// enforced it. (min >= max still means "defer to the backend".)
			float Rate = Drive.Target;
			if (bBounded)
			{
				Rate = FMath::Clamp(Rate, static_cast<float>(Spec.ControlRange.X),
					static_cast<float>(Spec.ControlRange.Y));
			}
			Backend_->SetCommand(MotorId, Rate * DirectionOf(MotorId));
			continue;
		}

		// Torque actuator. Everything here is in the robot's sense --
		// GetMotorVelocity already un-applies Direction -- and the sign is put
		// back on the way out.
		const FRammsVelocityGains Gains = GainsFor(MotorId);
		const float				  Error = Drive.Target - GetMotorVelocity(MotorId);
		PeakVelocityError = FMath::Max(PeakVelocityError, FMath::Abs(Error));

		// A backend that can run the loop at the physics rate does; it gets
		// the target and the torque range in the joint's sense.
		{
			const float Sign = DirectionOf(MotorId);
			float		Lo = 0.0f, Hi = 0.0f;
			if (bBounded)
			{
				Lo = FMath::Min(Spec.ControlRange.X * Sign, Spec.ControlRange.Y * Sign);
				Hi = FMath::Max(Spec.ControlRange.X * Sign, Spec.ControlRange.Y * Sign);
			}
			if (Backend_->SetVelocityDrive(MotorId, Drive.Target * Sign, Gains, Lo, Hi))
			{
				continue;
			}
		}

		// Otherwise close it here, once per tick.

		float Integral = Drive.Integral + Gains.Ki * Error * DeltaTime;
		if (Gains.MaxIntegralTorque > 0.0f)
		{
			Integral = FMath::Clamp(Integral, -Gains.MaxIntegralTorque, Gains.MaxIntegralTorque);
		}

		float Torque = Gains.Kp * Error + Integral;

		// Clamp to the motor's authored range, and stop integrating once there
		// -- otherwise a wheel that cannot reach its target winds the integral
		// up and lurches when the load comes off.
		if (bBounded)
		{
			const float Clamped = FMath::Clamp(Torque,
				static_cast<float>(Spec.ControlRange.X), static_cast<float>(Spec.ControlRange.Y));
			if (Clamped != Torque)
			{
				Integral = Drive.Integral; // saturated: hold, do not accumulate
			}
			Torque = Clamped;
		}
		Drive.Integral = Integral;

		Backend_->SetCommand(MotorId, Torque * DirectionOf(MotorId));
	}
}

void URammsRobotBaseComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	// First: a profile stepped here below starts from this frame's command.
	FlushCommandOffsets();
	StepPositionProfiles(DeltaTime);
	StepVelocityDrives(DeltaTime);
}

void URammsRobotBaseComponent::StepPositionProfiles(float DeltaTime)
{
	if (PositionProfiles.Num() == 0)
	{
		return;
	}
	EnsureBackend();
	if (!Backend_)
	{
		return;
	}
	for (TPair<FName, FPositionProfile>& Pair : PositionProfiles)
	{
		const FName		  MotorId = Pair.Key;
		FPositionProfile& P = Pair.Value;
		if (P.bInBackend)
		{
			continue;
		}
		FRammsMotorSpec Spec;
		if (!GetMotorSpec(MotorId, Spec))
		{
			continue;
		}
		RammsMotion::StepToward(P.Setpoint, P.Rate, P.Target, Spec.MaxVelocity, Spec.MaxAcceleration, DeltaTime);
		Backend_->SetCommand(MotorId, static_cast<float>(P.Setpoint) * DirectionOf(MotorId));
	}
}

USkeletalMeshComponent* URammsRobotBaseComponent::GetChaosSkeletalMesh() const
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return nullptr;
	}
	if (ChaosSkeletalMeshComponentName.IsNone())
	{
		return Owner->FindComponentByClass<USkeletalMeshComponent>();
	}
	TArray<USkeletalMeshComponent*> Meshes;
	Owner->GetComponents<USkeletalMeshComponent>(Meshes);
	for (USkeletalMeshComponent* Candidate : Meshes)
	{
		if (Candidate && Candidate->GetFName() == ChaosSkeletalMeshComponentName)
		{
			return Candidate;
		}
	}
	return nullptr;
}

FName URammsRobotBaseComponent::GetChaosName(FName MotorId) const
{
	LoadMotorRegistry();
	if (const FRammsMotorSpec* Spec = Motors.Find(MotorId))
	{
		if (!Spec->ChaosName.IsNone())
		{
			return Spec->ChaosName;
		}
	}
	return MotorId;
}

TArray<FName> URammsRobotBaseComponent::GetMotorIds() const
{
	LoadMotorRegistry();
	return MotorOrder;
}

TArray<FRammsMotorSpec> URammsRobotBaseComponent::GetMotorSpecs() const
{
	LoadMotorRegistry();
	TArray<FRammsMotorSpec> Out;
	Out.Reserve(MotorOrder.Num());
	for (const FName& Id : MotorOrder)
	{
		if (const FRammsMotorSpec* Spec = Motors.Find(Id))
		{
			Out.Add(*Spec);
		}
	}
	return Out;
}

int32 URammsRobotBaseComponent::GetMotorCount() const
{
	LoadMotorRegistry();
	return MotorOrder.Num();
}

FName URammsRobotBaseComponent::FindMotorIdByChaosName(FName ChaosName) const
{
	LoadMotorRegistry();
	if (ChaosName.IsNone())
	{
		return NAME_None;
	}
	for (const TPair<FName, FRammsMotorSpec>& Pair : Motors)
	{
		if (Pair.Value.ChaosName == ChaosName)
		{
			return Pair.Key;
		}
	}
	return Motors.Contains(ChaosName) ? ChaosName : NAME_None;
}

bool URammsRobotBaseComponent::GetMotorSpec(FName MotorId, FRammsMotorSpec& OutSpec) const
{
	LoadMotorRegistry();
	if (const FRammsMotorSpec* Spec = Motors.Find(MotorId))
	{
		OutSpec = *Spec;
		return true;
	}
	return false;
}

bool URammsRobotBaseComponent::HasMotor(FName MotorId) const
{
	LoadMotorRegistry();
	return Motors.Contains(MotorId);
}

ERammsActuatorType URammsRobotBaseComponent::GetMotorType(FName MotorId) const
{
	LoadMotorRegistry();
	const FRammsMotorSpec* Spec = Motors.Find(MotorId);
	return Spec ? Spec->Type : ERammsActuatorType::Torque;
}

float URammsRobotBaseComponent::DirectionOf(FName MotorId) const
{
	const FRammsMotorSpec* Spec = Motors.Find(MotorId);
	return (Spec && Spec->Direction < 0.0f) ? -1.0f : 1.0f;
}

void URammsRobotBaseComponent::SetMotorCommand(FName MotorId, float Value)
{
	EnsureBackend();
	if (!Backend_)
	{
		return;
	}

	const FRammsMotorSpec* Spec = Motors.Find(MotorId);
	if (!Spec && !WarnedUnregistered.Contains(MotorId))
	{
		WarnedUnregistered.Add(MotorId);
		UE_LOG(LogTemp, Warning,
			TEXT("RammsRobotBaseComponent on '%s': motor '%s' is not in the motor registry%s; routing by Id as-is (Torque, unbounded, Direction +1)."),
			GetOwner() ? *GetOwner()->GetName() : TEXT("?"), *MotorId.ToString(),
			MotorTable ? TEXT("") : TEXT(" (no MotorTable set)"));
	}

	// A direct command wins: whoever wrote this wants THIS value, not the
	// output of a loop that would overwrite it on the next tick.
	DropVelocityDrive(MotorId);

	// Clamp in the robot's sense to the authored range when one is set (a
	// range with min >= max means "no clamp here" — defer to the backend /
	// actuator's own range), then map to the engine's joint sign.
	if (Spec && Spec->ControlRange.X < Spec->ControlRange.Y)
	{
		Value = FMath::Clamp(Value, static_cast<float>(Spec->ControlRange.X),
			static_cast<float>(Spec->ControlRange.Y));
	}
	LastCommands.Add(MotorId, Value);
	if (FCommandOffset* Layered = CommandOffsets.Find(MotorId))
	{
		// Claimed for the offset layer: told once, in the tick, as the sum.
		// Told here it would get the bare baseline now and the sum a moment
		// later, and a servo eased at the physics rate would start for the
		// first.
		Layered->Effective = ClampToControlRange(Spec, Value + Layered->Offset);
		Layered->bDirty = true;
		return;
	}
	PushCommand(MotorId, Spec, Value);
}

void URammsRobotBaseComponent::PushCommand(FName MotorId, const FRammsMotorSpec* Spec, float Value)
{
	// A profiled position servo is eased to the value from where it stands;
	// the first command to it holds the joint where it is, so nothing jumps.
	// The backend eases it every physics step where it can, else
	// StepPositionProfiles does once per tick, from this tick on.
	if (Spec && Spec->Type == ERammsActuatorType::Position && (Spec->MaxVelocity > 0.0f || Spec->MaxAcceleration > 0.0f))
	{
		const float		  Direction = DirectionOf(MotorId);
		FPositionProfile* P = PositionProfiles.Find(MotorId);
		const bool		  bNew = P == nullptr;
		if (bNew)
		{
			P = &PositionProfiles.Add(MotorId);
			P->Setpoint = GetMotorValue(MotorId);
			P->Rate = 0.0;
		}
		P->Target = Value;
		const bool	bPhysicsRate = CVarPhysicsRateProfiles.GetValueOnGameThread() != 0;
		const float StartFrom = P->bInBackend ? NAN : static_cast<float>(P->Setpoint) * Direction;
		if (bPhysicsRate && Backend_->SetPositionProfile(MotorId, Value * Direction, Spec->MaxVelocity, Spec->MaxAcceleration, StartFrom))
		{
			P->bInBackend = true;
			return;
		}
		if (P->bInBackend)
		{
			// The backend has let go of it (its model not compiled), or was told
			// to: carry on here from where the joint is, not a setpoint stale
			// since then.
			if (!bPhysicsRate)
			{
				Backend_->ClearPositionProfile(MotorId);
			}
			P->bInBackend = false;
			P->Setpoint = GetMotorValue(MotorId);
			P->Rate = 0.0;
		}
		if (bNew)
		{
			Backend_->SetCommand(MotorId, static_cast<float>(P->Setpoint) * Direction);
		}
		return;
	}
	DropPositionProfile(MotorId);
	Backend_->SetCommand(MotorId, Value * DirectionOf(MotorId));
}

float URammsRobotBaseComponent::ClampToControlRange(const FRammsMotorSpec* Spec, float Value)
{
	// As SetMotorCommand clamps: a range with min >= max is no range.
	return (Spec && Spec->ControlRange.X < Spec->ControlRange.Y)
		? FMath::Clamp(Value, static_cast<float>(Spec->ControlRange.X), static_cast<float>(Spec->ControlRange.Y))
		: Value;
}

bool URammsRobotBaseComponent::ClaimCommandOffsetLayer(const UObject* Layer, TConstArrayView<FName> MotorIds)
{
	if (!Layer)
	{
		return false;
	}
	LoadMotorRegistry();
	if (CommandOffsets.Num() > 0 && !CommandOffsetLayer.IsValid())
	{
		// The last holder was collected without releasing.
		DropCommandOffsetLayer();
	}
	if (CommandOffsetLayer.IsValid() && CommandOffsetLayer.Get() != Layer)
	{
		return false;
	}
	for (const FName& MotorId : MotorIds)
	{
		// An offset is an angle or a length added to one: it means nothing on
		// a torque or a speed, and a motor with no row has no type to go by.
		const FRammsMotorSpec* Spec = Motors.Find(MotorId);
		if (!Spec || Spec->Type != ERammsActuatorType::Position)
		{
			return false;
		}
	}
	CommandOffsetLayer = Layer;
	for (const FName& MotorId : MotorIds)
	{
		if (!CommandOffsets.Contains(MotorId))
		{
			FCommandOffset& Layered = CommandOffsets.Add(MotorId);
			if (const float* Baseline = LastCommands.Find(MotorId))
			{
				Layered.Effective = *Baseline;
			}
		}
	}
	return true;
}

void URammsRobotBaseComponent::ReleaseCommandOffsetLayer(const UObject* Layer)
{
	if (Layer && CommandOffsetLayer.Get() == Layer)
	{
		DropCommandOffsetLayer();
	}
}

void URammsRobotBaseComponent::DropCommandOffsetLayer()
{
	// Taken out first: from here each motor is commanded at once again, this
	// last command included.
	const TMap<FName, FCommandOffset> Dropped = MoveTemp(CommandOffsets);
	CommandOffsets.Reset();
	CommandOffsetLayer.Reset();
	for (const TPair<FName, FCommandOffset>& Pair : Dropped)
	{
		const float* Baseline = LastCommands.Find(Pair.Key);
		if (!Baseline)
		{
			continue;
		}
		if (Pair.Value.Offset != 0.0f)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("RammsRobotBaseComponent on '%s': the command offset layer was given up with an offset of %g still on motor '%s'; it steps to its baseline."),
				*GetNameSafe(GetOwner()), Pair.Value.Offset, *Pair.Key.ToString());
		}
		if (Backend_ && (Pair.Value.bDirty || Pair.Value.Effective != *Baseline))
		{
			PushCommand(Pair.Key, Motors.Find(Pair.Key), *Baseline);
		}
	}
}

bool URammsRobotBaseComponent::SetMotorCommandOffset(FName MotorId, float Offset, const UObject* Layer)
{
	FCommandOffset* Layered = (Layer && CommandOffsetLayer.Get() == Layer) ? CommandOffsets.Find(MotorId) : nullptr;
	if (!Layered || !FMath::IsFinite(Offset))
	{
		return false;
	}
	Layered->Offset = Offset;
	if (const float* Baseline = LastCommands.Find(MotorId))
	{
		Layered->Effective = ClampToControlRange(Motors.Find(MotorId), *Baseline + Offset);
		Layered->bDirty = true;
	}
	return true;
}

bool URammsRobotBaseComponent::RebaseMotorCommand(FName MotorId, float NewBaseline, const UObject* Layer)
{
	FCommandOffset* Layered = (Layer && CommandOffsetLayer.Get() == Layer) ? CommandOffsets.Find(MotorId) : nullptr;
	float*			Baseline = LastCommands.Find(MotorId);
	if (!Layered || !Baseline || !FMath::IsFinite(NewBaseline))
	{
		return false;
	}
	// A baseline is always within the range, as SetMotorCommand leaves it.
	*Baseline = ClampToControlRange(Motors.Find(MotorId), NewBaseline);
	// Effective stays: what the motor is told does not change, to the bit, and
	// a write still waiting to be told is still waiting.
	Layered->Offset = Layered->Effective - *Baseline;
	return true;
}

bool URammsRobotBaseComponent::GetMotorEffectiveCommand(FName MotorId, float& OutValue) const
{
	const float* Baseline = LastCommands.Find(MotorId);
	if (!Baseline)
	{
		return false;
	}
	const FCommandOffset* Layered = CommandOffsets.Find(MotorId);
	OutValue = Layered ? Layered->Effective : *Baseline;
	return true;
}

float URammsRobotBaseComponent::GetMotorCommandOffset(FName MotorId) const
{
	const FCommandOffset* Layered = CommandOffsets.Find(MotorId);
	return Layered ? Layered->Offset : 0.0f;
}

void URammsRobotBaseComponent::FlushCommandOffsets()
{
	if (CommandOffsets.Num() == 0)
	{
		return;
	}
	if (!CommandOffsetLayer.IsValid())
	{
		// Collected without releasing: its offsets must not stay on for good.
		DropCommandOffsetLayer();
		return;
	}
	if (!Backend_)
	{
		return;
	}
	for (TPair<FName, FCommandOffset>& Pair : CommandOffsets)
	{
		FCommandOffset& Layered = Pair.Value;
		if (!Layered.bDirty)
		{
			continue;
		}
		Layered.bDirty = false;
		if (LastCommands.Contains(Pair.Key))
		{
			PushCommand(Pair.Key, Motors.Find(Pair.Key), Layered.Effective);
		}
	}
}

void URammsRobotBaseComponent::DropPositionProfile(FName MotorId)
{
	FPositionProfile Removed;
	if (PositionProfiles.RemoveAndCopyValue(MotorId, Removed) && Removed.bInBackend && Backend_)
	{
		Backend_->ClearPositionProfile(MotorId);
	}
}

bool URammsRobotBaseComponent::GetMotorCommand(FName MotorId, float& OutValue) const
{
	const float* Value = LastCommands.Find(MotorId);
	if (!Value)
	{
		return false;
	}
	OutValue = *Value;
	return true;
}

float URammsRobotBaseComponent::GetMotorValue(FName MotorId) const
{
	EnsureBackend();
	return Backend_ ? Backend_->GetValue(MotorId) * DirectionOf(MotorId) : 0.0f;
}

float URammsRobotBaseComponent::GetMotorVelocity(FName MotorId) const
{
	EnsureBackend();
	return Backend_ ? Backend_->GetVelocity(MotorId) * DirectionOf(MotorId) : 0.0f;
}

bool URammsRobotBaseComponent::SimulatesMotorJoint(FName MotorId) const
{
	EnsureBackend();
	return Backend_ ? Backend_->SimulatesMotorJoint(MotorId) : false;
}

bool URammsRobotBaseComponent::HasMotorBrake(FName MotorId) const
{
	EnsureBackend();
	return Backend_ && Backend_->HasMotorBrake(MotorId);
}

bool URammsRobotBaseComponent::SetMotorTorqueLimit(FName MotorId, float MinNm, float MaxNm)
{
	EnsureBackend();
	if (!Backend_ || MotorId.IsNone())
	{
		return false;
	}
	if (!(MinNm < MaxNm))
	{
		ClearMotorTorqueLimit(MotorId);
		return true;
	}
	// The robot's sense to the joint's: a reversed motor's range is mirrored.
	const bool	Reversed = DirectionOf(MotorId) < 0.0f;
	const float Lo = Reversed ? -MaxNm : MinNm;
	const float Hi = Reversed ? -MinNm : MaxNm;
	if (!Backend_->SetMotorTorqueLimit(MotorId, Lo, Hi))
	{
		return false;
	}
	TorqueLimited.Add(MotorId);
	return true;
}

void URammsRobotBaseComponent::ClearMotorTorqueLimit(FName MotorId)
{
	if (TorqueLimited.Remove(MotorId) > 0 && Backend_)
	{
		Backend_->SetMotorTorqueLimit(MotorId, 0.0f, 0.0f);
	}
}

void URammsRobotBaseComponent::ClearAllMotorTorqueLimits()
{
	if (Backend_)
	{
		for (const FName& MotorId : TorqueLimited)
		{
			Backend_->SetMotorTorqueLimit(MotorId, 0.0f, 0.0f);
		}
	}
	TorqueLimited.Reset();
}

bool URammsRobotBaseComponent::GetMotorEffort(FName MotorId, double WindowSeconds, FRammsMotorEffort& Out) const
{
	EnsureBackend();
	if (!Backend_ || MotorId.IsNone() || !Backend_->GetMotorEffort(MotorId, WindowSeconds, Out))
	{
		return false;
	}
	if (DirectionOf(MotorId) < 0.0f)
	{
		// The joint's sense to the robot's: the forces negated, the limits
		// mirrored (lower and upper trade places).
		Out.MeanNm = -Out.MeanNm;
		Out.PeakNm = -Out.PeakNm;
		Out.LastNm = -Out.LastNm;
		const double Lower = Out.LowerNm, RatedLower = Out.RatedLowerNm;
		Out.LowerNm = -Out.UpperNm;
		Out.UpperNm = -Lower;
		Out.RatedLowerNm = -Out.RatedUpperNm;
		Out.RatedUpperNm = -RatedLower;
		Swap(Out.AtLowerSeconds, Out.AtUpperSeconds);
	}
	return true;
}

bool URammsRobotBaseComponent::GetMotorTransform(FName MotorId, FTransform& OutWorld) const
{
	EnsureBackend();
	return Backend_ ? Backend_->GetMotorTransform(MotorId, OutWorld) : false;
}

float URammsRobotBaseComponent::GetMotorSeparation(FName MotorIdA, FName MotorIdB) const
{
	FTransform A, B;
	if (GetMotorTransform(MotorIdA, A) && GetMotorTransform(MotorIdB, B))
	{
		return FVector::Dist(A.GetLocation(), B.GetLocation());
	}
	return -1.0f;
}

namespace
{
	/** A torque for the console: N*m, or inf where a side is unbounded. */
	FString TorqueText(double Nm)
	{
		return Nm >= 1e30 ? FString(TEXT("inf")) : Nm <= -1e30 ? FString(TEXT("-inf"))
															   : FString::Printf(TEXT("%.1f"), Nm);
	}

	/** Every robot base playing in World (dev console commands). */
	template <typename FnType>
	int32 ForEachRobotBase(UWorld* World, FnType&& Fn)
	{
		int32 Count = 0;
		for (TObjectIterator<URammsRobotBaseComponent> It; It; ++It)
		{
			URammsRobotBaseComponent* Base = *It;
			if (IsValid(Base) && Base->GetWorld() == World && Base->HasBegunPlay())
			{
				Fn(*Base);
				++Count;
			}
		}
		return Count;
	}

	FAutoConsoleCommandWithWorldAndArgs CmdMotorTorqueLimit(TEXT("Ramms.Motor.TorqueLimit"),
		TEXT("Ramms.Motor.TorqueLimit <Motor> <MinNm> <MaxNm> | <Motor> clear: a motor's torque limit in the robot's sense "
			 "(a drive's per-direction limit, within its own rating), on every robot in this world that has it. A side "
			 "given as inf keeps the motor's own limit there. Dev bench only: the curb controller sets and clears its own."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World) {
			if (Args.Num() < 2)
			{
				UE_LOG(LogTemp, Display, TEXT("Ramms.Motor.TorqueLimit <Motor> <MinNm> <MaxNm> | <Motor> clear"));
				return;
			}
			const FName Motor(*Args[0]);
			const bool	bClear = Args[1].Equals(TEXT("clear"), ESearchCase::IgnoreCase);
			if (!bClear && Args.Num() < 3)
			{
				UE_LOG(LogTemp, Display, TEXT("Ramms.Motor.TorqueLimit <Motor> <MinNm> <MaxNm> | <Motor> clear"));
				return;
			}
			// Any inf on a side (inf, +inf, -inf) leaves that side unbounded: the
			// motor's own limit there.
			const float Max = TNumericLimits<float>::Max();
			const auto	Parse = [](const FString& S, float Unbounded) {
				 return S.Equals(TEXT("inf"), ESearchCase::IgnoreCase) || S.Equals(TEXT("+inf"), ESearchCase::IgnoreCase)
						 || S.Equals(TEXT("-inf"), ESearchCase::IgnoreCase)
					 ? Unbounded
					 : FCString::Atof(*S);
			};
			const float Lo = bClear ? 0.0f : Parse(Args[1], -Max);
			const float Hi = bClear ? 0.0f : Parse(Args[2], Max);
			if (!bClear && !(Lo < Hi))
			{
				UE_LOG(LogTemp, Display, TEXT("Ramms.Motor.TorqueLimit: min %s is not under max %s (use 'clear' to clear a limit)"),
					*TorqueText(Lo), *TorqueText(Hi));
				return;
			}
			const int32 N = ForEachRobotBase(World, [&](URammsRobotBaseComponent& Base) {
				const bool bOk = bClear ? (Base.ClearMotorTorqueLimit(Motor), true) : Base.SetMotorTorqueLimit(Motor, Lo, Hi);
				UE_LOG(LogTemp, Display, TEXT("Ramms.Motor.TorqueLimit: %s '%s' %s"), *GetNameSafe(Base.GetOwner()), *Motor.ToString(),
					bClear	  ? TEXT("cleared")
						: bOk ? *FString::Printf(TEXT("limited to [%s, %s] N*m"), *TorqueText(Lo), *TorqueText(Hi))
							  : TEXT("not limited (unsupported)"));
			});
			if (N == 0)
			{
				UE_LOG(LogTemp, Display, TEXT("Ramms.Motor.TorqueLimit: no robot playing in this world"));
			}
		}));

	FAutoConsoleCommandWithWorldAndArgs CmdMotorEffort(TEXT("Ramms.Motor.Effort"),
		TEXT("Ramms.Motor.Effort <Motor> [WindowSeconds=0.1]: what a motor put out over the last window of simulated time "
			 "(mean, peak, last; its limits; time at them), in the robot's sense. The first call on a motor starts recording it."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World) {
			if (Args.Num() < 1)
			{
				UE_LOG(LogTemp, Display, TEXT("Ramms.Motor.Effort <Motor> [WindowSeconds=0.1]"));
				return;
			}
			const FName	 Motor(*Args[0]);
			const double Window = Args.Num() > 1 ? FCString::Atod(*Args[1]) : 0.1;
			const int32	 N = ForEachRobotBase(World, [&](URammsRobotBaseComponent& Base) {
				 FRammsMotorEffort E;
				 if (!Base.GetMotorEffort(Motor, Window, E))
				 {
					 UE_LOG(LogTemp, Display, TEXT("Ramms.Motor.Effort: %s '%s' no readings (yet, or unsupported)"), *GetNameSafe(Base.GetOwner()),
						 *Motor.ToString());
					 return;
				 }
				 UE_LOG(LogTemp, Display,
					 TEXT("Ramms.Motor.Effort: %s '%s' over %.3f s: mean %.1f, peak %.1f, last %.1f N*m; limits [%s, %s] (rated [%s, %s]); "
						   "%.3f s at lower, %.3f s at upper, %.3f s at rated; t %.3f"),
					 *GetNameSafe(Base.GetOwner()), *Motor.ToString(), E.Seconds, E.MeanNm, E.PeakNm, E.LastNm, *TorqueText(E.LowerNm),
					 *TorqueText(E.UpperNm), *TorqueText(E.RatedLowerNm), *TorqueText(E.RatedUpperNm), E.AtLowerSeconds, E.AtUpperSeconds,
					 E.AtRatedSeconds, E.LatestTime);
			 });
			if (N == 0)
			{
				UE_LOG(LogTemp, Display, TEXT("Ramms.Motor.Effort: no robot playing in this world"));
			}
		}));
} // namespace
