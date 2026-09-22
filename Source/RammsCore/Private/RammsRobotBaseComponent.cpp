// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRobotBaseComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "RammsActuationBackend.h"
#include "RammsActuationBackendRegistry.h"
#include "RammsChaosActuationBackend.h"
#include "Engine/World.h"

URammsRobotBaseComponent::URammsRobotBaseComponent()
{
	// Ticks only to close the velocity loop; see SetMotorVelocityCommand.
	// PrePhysics, so the torque it writes is applied in the same step as the
	// commands the drive controllers wrote a moment earlier.
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;

	// Enough to hold a lift-drive omni wheel at its commanded speed against
	// the other three, well inside the +/-30 N.m those actuators allow.
	DefaultVelocityGains.Kp = 4.0f;
	DefaultVelocityGains.Ki = 8.0f;
	DefaultVelocityGains.MaxIntegralTorque = 15.0f;
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
	LoadMotorRegistry();
	EnsureBackend();
}

void URammsRobotBaseComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	delete Backend_;
	Backend_ = nullptr;
	bBackendResolved = false;
	bMotorsLoaded = false;
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
	//  - Chaos:  the native Chaos skeletal backend (RammsCore, no registry).
	//  - Auto:   try MuJoCo first (an articulation on/under this actor), else
	//            fall back to Chaos.
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

bool URammsRobotBaseComponent::ReleaseMotor(FName MotorId)
{
	// Letting go means letting go: a loop still driving this motor would put
	// it straight back under command on the next tick.
	VelocityDrives.Remove(MotorId);
	EnsureBackend();
	return Backend_ && Backend_->ReleaseMotor(MotorId);
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
}

void URammsRobotBaseComponent::ClearMotorVelocityCommand(FName MotorId)
{
	VelocityDrives.Remove(MotorId);
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

		if (GetMotorType(MotorId) == ERammsActuatorType::Velocity)
		{
			// The actuator closes its own loop; hand it the rate.
			Backend_->SetCommand(MotorId, Drive.Target * DirectionOf(MotorId));
			continue;
		}

		// Torque actuator: close the loop here. Everything is in the robot's
		// sense -- GetMotorVelocity already un-applies Direction -- and the
		// sign is put back on the way out.
		const FRammsVelocityGains Gains = GainsFor(MotorId);
		const float				  Error = Drive.Target - GetMotorVelocity(MotorId);

		float Integral = Drive.Integral + Gains.Ki * Error * DeltaTime;
		if (Gains.MaxIntegralTorque > 0.0f)
		{
			Integral = FMath::Clamp(Integral, -Gains.MaxIntegralTorque, Gains.MaxIntegralTorque);
		}

		float Torque = Gains.Kp * Error + Integral;

		// Clamp to the motor's authored range, and stop integrating once there
		// -- otherwise a wheel that cannot reach its target winds the integral
		// up and lurches when the load comes off.
		FRammsMotorSpec Spec;
		if (GetMotorSpec(MotorId, Spec) && Spec.ControlRange.X < Spec.ControlRange.Y)
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
	StepVelocityDrives(DeltaTime);
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
	VelocityDrives.Remove(MotorId);

	// Clamp in the robot's sense to the authored range when one is set (a
	// range with min >= max means "no clamp here" — defer to the backend /
	// actuator's own range), then map to the engine's joint sign.
	if (Spec && Spec->ControlRange.X < Spec->ControlRange.Y)
	{
		Value = FMath::Clamp(Value, static_cast<float>(Spec->ControlRange.X),
			static_cast<float>(Spec->ControlRange.Y));
	}
	Backend_->SetCommand(MotorId, Value * DirectionOf(MotorId));
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
