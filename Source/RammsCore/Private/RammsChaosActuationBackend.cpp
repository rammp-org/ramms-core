// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsChaosActuationBackend.h"
#include "RammsRobotBaseComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodyInstance.h"

bool FRammsChaosActuationBackend::Initialize(URammsRobotBaseComponent& Base)
{
	BaseComp = &Base;
	Mesh = nullptr;

	// Resolve the skeletal mesh that carries the simulated wheel bodies: the one
	// named on the base component, else the owner's first (mirrors the
	// differential-drive controller's lookup).
	if (AActor* Owner = Base.GetOwner())
	{
		if (!Base.ChaosSkeletalMeshComponentName.IsNone())
		{
			TArray<USkeletalMeshComponent*> Meshes;
			Owner->GetComponents<USkeletalMeshComponent>(Meshes);
			for (USkeletalMeshComponent* Candidate : Meshes)
			{
				if (Candidate && Candidate->GetFName() == Base.ChaosSkeletalMeshComponentName)
				{
					Mesh = Candidate;
					break;
				}
			}
			if (!Mesh.IsValid())
			{
				UE_LOG(LogTemp, Warning,
					TEXT("RammsChaosActuationBackend: no skeletal mesh component named '%s' on '%s'."),
					*Base.ChaosSkeletalMeshComponentName.ToString(), *Owner->GetName());
				return false;
			}
		}
		else
		{
			Mesh = Owner->FindComponentByClass<USkeletalMeshComponent>();
		}
	}
	return Mesh.IsValid();
}

FName FRammsChaosActuationBackend::BoneFor(FName MotorId) const
{
	if (const URammsRobotBaseComponent* Base = BaseComp.Get())
	{
		FRammsMotorSpec Spec;
		if (Base->GetMotorSpec(MotorId, Spec) && !Spec.ChaosName.IsNone())
		{
			return Spec.ChaosName;
		}
	}
	return MotorId;
}

void FRammsChaosActuationBackend::SetCommand(FName MotorId, float Value)
{
	USkeletalMeshComponent* SkelMesh = Mesh.Get();
	if (!SkelMesh)
	{
		return;
	}

	// Only torque motors have a Chaos wheel counterpart here.
	if (const URammsRobotBaseComponent* Base = BaseComp.Get())
	{
		if (Base->HasMotor(MotorId) && Base->GetMotorType(MotorId) != ERammsActuatorType::Torque)
		{
			if (!WarnedUnsupported.Contains(MotorId))
			{
				WarnedUnsupported.Add(MotorId);
				UE_LOG(LogTemp, Warning,
					TEXT("RammsChaosActuationBackend: motor '%s' is not a Torque motor; Chaos position/velocity actuation is unmodelled — command ignored (use the MuJoCo backend for this robot)."),
					*MotorId.ToString());
			}
			return;
		}
	}

	const FName	   Bone = BoneFor(MotorId);
	FBodyInstance* BodyInst = SkelMesh->GetBodyInstance(Bone);
	if (!BodyInst || !BodyInst->IsInstanceSimulatingPhysics())
	{
		return;
	}

	const int32 BoneIndex = SkelMesh->GetBoneIndex(Bone);
	if (BoneIndex == INDEX_NONE)
	{
		return;
	}
	const FTransform BoneTransform = SkelMesh->GetBoneTransform(BoneIndex);

	// About the wheel's local spin axis (Y), ×100 and as an acceleration change —
	// byte-for-byte the pre-existing differential-drive Chaos path (see the
	// header's units caveat); not a physical N·m torque.
	const FVector WorldTorque = BoneTransform.TransformVectorNoScale(FVector(0.0f, Value, 0.0f)) * 100.0f;
	BodyInst->AddTorqueInRadians(WorldTorque, /*bAllowSubstepping=*/false, /*bAccelChange=*/true);
}

float FRammsChaosActuationBackend::GetValue(FName /*MotorId*/) const
{
	// A continuous drive wheel has no meaningful absolute position; consumers
	// that need travel integrate velocity themselves (odometry).
	return 0.0f;
}

float FRammsChaosActuationBackend::GetVelocity(FName MotorId) const
{
	USkeletalMeshComponent* SkelMesh = Mesh.Get();
	if (!SkelMesh)
	{
		return 0.0f;
	}
	const FName	   Bone = BoneFor(MotorId);
	FBodyInstance* BodyInst = SkelMesh->GetBodyInstance(Bone);
	if (!BodyInst || !BodyInst->IsInstanceSimulatingPhysics())
	{
		return 0.0f;
	}
	const int32 BoneIndex = SkelMesh->GetBoneIndex(Bone);
	if (BoneIndex == INDEX_NONE)
	{
		return 0.0f;
	}
	const FTransform BoneTransform = SkelMesh->GetBoneTransform(BoneIndex);
	const FVector	 WorldAngVel = BodyInst->GetUnrealWorldAngularVelocityInRadians();
	// Spin about the bone's local Y axis.
	return BoneTransform.InverseTransformVectorNoScale(WorldAngVel).Y;
}

bool FRammsChaosActuationBackend::GetMotorTransform(FName MotorId, FTransform& OutWorld) const
{
	USkeletalMeshComponent* SkelMesh = Mesh.Get();
	if (!SkelMesh)
	{
		return false;
	}
	const int32 BoneIndex = SkelMesh->GetBoneIndex(BoneFor(MotorId));
	if (BoneIndex == INDEX_NONE)
	{
		return false;
	}
	OutWorld = SkelMesh->GetBoneTransform(BoneIndex);
	return true;
}
