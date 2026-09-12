// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRobotBaseComponent.h"
#include "RammsActuationBackend.h"
#include "RammsActuationBackendRegistry.h"

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

	// Resolve the backend once. MuJoCo (explicit or Auto) comes from the
	// registry (RammsMujocoSupport); the Chaos actuation backend is a follow-up,
	// so for now an unresolved backend leaves commands as safe no-ops.
	delete Backend_;
	Backend_ = nullptr;
	if (Backend == ERammsPhysicsBackend::Mujoco || Backend == ERammsPhysicsBackend::Auto)
	{
		if (IRammsActuationBackend* Mj = RammsActuationBackends::CreateMujocoBackend(*this))
		{
			if (Mj->Initialize(*this))
			{
				Backend_ = Mj;
			}
			else
			{
				delete Mj;
			}
		}
	}
	if (!Backend_)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("RammsRobotBaseComponent on '%s': no actuation backend resolved (motor commands are no-ops until one is available)."),
			GetOwner() ? *GetOwner()->GetName() : TEXT("?"));
	}
}

void URammsRobotBaseComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	delete Backend_;
	Backend_ = nullptr;
	Super::EndPlay(EndPlayReason);
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

void URammsRobotBaseComponent::SetMotorCommand(FName MotorId, float Value)
{
	if (!Backend_)
	{
		return;
	}
	// Clamp to the motor's authored range when one is set (zero-width = defer to
	// the backend / actuator's own range).
	if (const FRammsMotorSpec* Spec = Motors.Find(MotorId))
	{
		if (Spec->ControlRange.X < Spec->ControlRange.Y)
		{
			Value = FMath::Clamp(Value, static_cast<float>(Spec->ControlRange.X),
				static_cast<float>(Spec->ControlRange.Y));
		}
	}
	Backend_->SetCommand(MotorId, Value);
}

float URammsRobotBaseComponent::GetMotorValue(FName MotorId) const
{
	return Backend_ ? Backend_->GetValue(MotorId) : 0.0f;
}

float URammsRobotBaseComponent::GetMotorVelocity(FName MotorId) const
{
	return Backend_ ? Backend_->GetVelocity(MotorId) : 0.0f;
}

bool URammsRobotBaseComponent::GetMotorTransform(FName MotorId, FTransform& OutWorld) const
{
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
