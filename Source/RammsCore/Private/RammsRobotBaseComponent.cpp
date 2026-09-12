// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRobotBaseComponent.h"
#include "RammsActuationBackend.h"
#include "RammsActuationBackendRegistry.h"
#include "RammsChaosActuationBackend.h"
#include "Engine/World.h"

URammsRobotBaseComponent::URammsRobotBaseComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
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

	// Load the motor registry.
	Motors.Reset();
	if (MotorTable)
	{
		static const FString Context(TEXT("RammsRobotBaseComponent"));
		MotorTable->ForeachRow<FRammsMotorSpec>(Context,
			[this](const FName& RowName, const FRammsMotorSpec& Row) {
				// Id defaults to the row name when the row leaves it unset.
				FRammsMotorSpec Spec = Row;
				if (Spec.Id.IsNone())
				{
					Spec.Id = RowName;
				}
				Motors.Add(Spec.Id, Spec);
			});
	}

	EnsureBackend();
}

void URammsRobotBaseComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	delete Backend_;
	Backend_ = nullptr;
	bBackendResolved = false;
	Super::EndPlay(EndPlayReason);
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

bool URammsRobotBaseComponent::GetMotorSpec(FName MotorId, FRammsMotorSpec& OutSpec) const
{
	if (const FRammsMotorSpec* Spec = Motors.Find(MotorId))
	{
		OutSpec = *Spec;
		return true;
	}
	return false;
}

bool URammsRobotBaseComponent::HasMotor(FName MotorId) const
{
	return Motors.Contains(MotorId);
}

ERammsActuatorType URammsRobotBaseComponent::GetMotorType(FName MotorId) const
{
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

	// Clamp in the robot's sense to the authored range when one is set
	// (zero-width = defer to the backend / actuator's own range), then map to
	// the engine's joint sign.
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
