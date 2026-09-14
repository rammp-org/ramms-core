// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsChaosActuationBackend.h"
#include "MebotControllerComponent.h"
#include "RammsRobotBaseComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/ConstraintInstance.h"

bool FRammsChaosActuationBackend::Initialize(URammsRobotBaseComponent& Base)
{
	BaseComp = &Base;
	Mesh = nullptr;
	ConstraintMotors.Reset();

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

bool FRammsChaosActuationBackend::IsConstraintMotor(FName MotorId) const
{
	const URammsRobotBaseComponent* Base = BaseComp.Get();
	return Base && Base->HasMotor(MotorId) && Base->GetMotorType(MotorId) == ERammsActuatorType::Position;
}

FConstraintInstance* FRammsChaosActuationBackend::ResolveConstraintMotor(FName MotorId, FConstraintMotor*& OutInfo) const
{
	OutInfo = nullptr;
	USkeletalMeshComponent* SkelMesh = Mesh.Get();
	if (!SkelMesh)
	{
		return nullptr;
	}
	// The instance pointer belongs to the mesh's current physics state, which
	// is rebuilt on re-registration: look it up every time, cache only the
	// resolved DOF description.
	const FName			 ConstraintName = BoneFor(MotorId);
	FConstraintInstance* CI = SkelMesh->FindConstraintInstance(ConstraintName);
	if (!CI)
	{
		if (!WarnedUnsupported.Contains(MotorId))
		{
			WarnedUnsupported.Add(MotorId);
			UE_LOG(LogTemp, Warning,
				TEXT("RammsChaosActuationBackend: Position motor '%s' names constraint '%s', which the physics asset of '%s' doesn't have — command ignored."),
				*MotorId.ToString(), *ConstraintName.ToString(), *SkelMesh->GetName());
		}
		return nullptr;
	}

	FConstraintMotor* Info = ConstraintMotors.Find(MotorId);
	if (!Info)
	{
		FConstraintMotor New;
		New.ConstraintName = ConstraintName;
		New.ChildBone = CI->GetChildBoneName();
		New.ParentBone = CI->GetParentBoneName();
		// First non-locked degree of freedom, linear before angular.
		const ELinearConstraintMotion Lin[3] = { CI->GetLinearXMotion(), CI->GetLinearYMotion(), CI->GetLinearZMotion() };
		// Angular, in constraint-frame axis order: twist (X), swing2 (Y), swing1 (Z).
		const EAngularConstraintMotion Ang[3] = { CI->GetAngularTwistMotion(), CI->GetAngularSwing2Motion(), CI->GetAngularSwing1Motion() };
		int32						   Axis = INDEX_NONE;
		bool						   bLinear = false;
		for (int32 i = 0; i < 3 && Axis == INDEX_NONE; ++i)
		{
			if (Lin[i] != ELinearConstraintMotion::LCM_Locked)
			{
				Axis = i;
				bLinear = true;
			}
		}
		for (int32 i = 0; i < 3 && Axis == INDEX_NONE; ++i)
		{
			if (Ang[i] != EAngularConstraintMotion::ACM_Locked)
			{
				Axis = i;
			}
		}
		if (Axis == INDEX_NONE)
		{
			if (!WarnedUnsupported.Contains(MotorId))
			{
				WarnedUnsupported.Add(MotorId);
				UE_LOG(LogTemp, Warning,
					TEXT("RammsChaosActuationBackend: constraint '%s' (motor '%s') has every axis locked — nothing to drive."),
					*ConstraintName.ToString(), *MotorId.ToString());
			}
			return nullptr;
		}
		New.bLinear = bLinear;
		New.Axis = Axis;
		if (const USkeletalMesh* Asset = SkelMesh->GetSkeletalMeshAsset())
		{
			const FMatrix ParentRef = Asset->GetComposedRefPoseMatrix(New.ParentBone);
			const FMatrix ChildRef = Asset->GetComposedRefPoseMatrix(New.ChildBone);
			New.RestOffset = ParentRef.InverseFast().TransformPosition(ChildRef.GetOrigin());
		}
		static const TCHAR* AxisNames[3] = { TEXT("X"), TEXT("Y"), TEXT("Z") };
		static const TCHAR* AngNames[3] = { TEXT("twist"), TEXT("swing2"), TEXT("swing1") };
		UE_LOG(LogTemp, Log, TEXT("RammsChaosActuationBackend: motor '%s' -> constraint '%s' (%s -> %s), %s %s%s."),
			*MotorId.ToString(), *ConstraintName.ToString(), *New.ParentBone.ToString(), *New.ChildBone.ToString(),
			bLinear ? TEXT("linear") : TEXT("angular"), AxisNames[Axis], bLinear ? TEXT(" (cm)") : *FString::Printf(TEXT(" / %s (rad)"), AngNames[Axis]));
		Info = &ConstraintMotors.Add(MotorId, New);
	}
	OutInfo = Info;
	return CI;
}

bool FRammsChaosActuationBackend::GetChildInParent(const FConstraintMotor& Info, FTransform& OutChildInParent) const
{
	USkeletalMeshComponent* SkelMesh = Mesh.Get();
	if (!SkelMesh)
	{
		return false;
	}
	const int32 ChildIndex = SkelMesh->GetBoneIndex(Info.ChildBone);
	const int32 ParentIndex = SkelMesh->GetBoneIndex(Info.ParentBone);
	if (ChildIndex == INDEX_NONE || ParentIndex == INDEX_NONE)
	{
		return false;
	}
	OutChildInParent = SkelMesh->GetBoneTransform(ChildIndex).GetRelativeTransform(SkelMesh->GetBoneTransform(ParentIndex));
	return true;
}

void FRammsChaosActuationBackend::TakeOverFromMebotController(FConstraintMotor& Info)
{
	if (Info.bTakenOver)
	{
		return;
	}
	Info.bTakenOver = true;
	const URammsRobotBaseComponent* Base = BaseComp.Get();
	AActor*							Owner = Base ? Base->GetOwner() : nullptr;
	UMebotControllerComponent*		MC = Owner ? Owner->FindComponentByClass<UMebotControllerComponent>() : nullptr;
	// A MebotController routing through the base is the caller, not a rival.
	if (!MC || MC->IsUsingRobotBase())
	{
		return;
	}
	// Otherwise its per-tick update would keep overwriting the drive target we set.
	for (const FAngularMotorConfig& M : MC->GetAngularMotors())
	{
		if (M.ConstraintName == Info.ConstraintName && M.bEnabled)
		{
			MC->SetAngularMotorEnabled(Info.ConstraintName, false);
			UE_LOG(LogTemp, Log, TEXT("RammsChaosActuationBackend: constraint '%s' is now driven through the robot base; disabled the matching MebotController angular motor."), *Info.ConstraintName.ToString());
		}
	}
	for (const FLinearMotorConfig& M : MC->GetLinearMotors())
	{
		if (M.ConstraintName == Info.ConstraintName && M.bEnabled)
		{
			MC->SetLinearMotorEnabled(Info.ConstraintName, false);
			UE_LOG(LogTemp, Log, TEXT("RammsChaosActuationBackend: constraint '%s' is now driven through the robot base; disabled the matching MebotController linear motor."), *Info.ConstraintName.ToString());
		}
	}
}

void FRammsChaosActuationBackend::SetCommand(FName MotorId, float Value)
{
	USkeletalMeshComponent* SkelMesh = Mesh.Get();
	if (!SkelMesh)
	{
		return;
	}

	// Position motors drive a physics-asset constraint.
	if (IsConstraintMotor(MotorId))
	{
		FConstraintMotor*	 Info = nullptr;
		FConstraintInstance* CI = ResolveConstraintMotor(MotorId, Info);
		if (!CI || !Info)
		{
			return;
		}
		TakeOverFromMebotController(*Info);
		const URammsRobotBaseComponent* Base = BaseComp.Get();
		const float						Stiffness = Base ? Base->ChaosPositionDriveStiffness : 100000.0f;
		const float						Damping = Base ? Base->ChaosPositionDriveDamping : 10000.0f;
		const float						ForceLimit = Base ? Base->ChaosPositionDriveForceLimit : 0.0f;
		if (Info->bLinear)
		{
			const bool bX = Info->Axis == 0, bY = Info->Axis == 1, bZ = Info->Axis == 2;
			CI->SetLinearPositionDrive(bX, bY, bZ);
			CI->SetLinearVelocityDrive(bX, bY, bZ);
			CI->SetLinearDriveParams(Stiffness, Damping, ForceLimit);
			FVector Target = FVector::ZeroVector;
			Target[Info->Axis] = Value;
			CI->SetLinearPositionTarget(Target);
			CI->SetLinearVelocityTarget(FVector::ZeroVector);
		}
		else
		{
			CI->SetOrientationDriveTwistAndSwing(true, true);
			CI->SetAngularDriveParams(Stiffness, Damping, ForceLimit);
			FVector AxisVec = FVector::ZeroVector;
			AxisVec[Info->Axis] = 1.0f;
			CI->SetAngularOrientationTarget(FQuat(AxisVec, Value));
		}
		return;
	}

	// Only torque motors have a Chaos wheel counterpart.
	if (const URammsRobotBaseComponent* Base = BaseComp.Get())
	{
		if (Base->HasMotor(MotorId) && Base->GetMotorType(MotorId) != ERammsActuatorType::Torque)
		{
			if (!WarnedUnsupported.Contains(MotorId))
			{
				WarnedUnsupported.Add(MotorId);
				UE_LOG(LogTemp, Warning,
					TEXT("RammsChaosActuationBackend: motor '%s' is a Velocity motor; Chaos velocity actuation is unmodelled — command ignored (use a Position motor on a constraint, or the MuJoCo backend)."),
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

float FRammsChaosActuationBackend::GetValue(FName MotorId) const
{
	if (IsConstraintMotor(MotorId))
	{
		FConstraintMotor*	 Info = nullptr;
		FConstraintInstance* CI = ResolveConstraintMotor(MotorId, Info);
		if (!CI || !Info)
		{
			return 0.0f;
		}
		if (Info->bLinear)
		{
			// Travel along the axis from the reference pose, in the parent
			// bone's space (the constraint frame is authored bone-aligned here).
			FTransform ChildInParent;
			return GetChildInParent(*Info, ChildInParent) ? static_cast<float>(ChildInParent.GetLocation()[Info->Axis] - Info->RestOffset[Info->Axis]) : 0.0f;
		}
		// Chaos reports these in radians.
		switch (Info->Axis)
		{
			case 0:
				return CI->GetCurrentTwist();
			case 1:
				return CI->GetCurrentSwing2();
			default:
				return CI->GetCurrentSwing1();
		}
	}
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

	if (IsConstraintMotor(MotorId))
	{
		FConstraintMotor*	 Info = nullptr;
		FConstraintInstance* CI = ResolveConstraintMotor(MotorId, Info);
		if (!CI || !Info)
		{
			return 0.0f;
		}
		FBodyInstance* Child = SkelMesh->GetBodyInstance(Info->ChildBone);
		FBodyInstance* Parent = SkelMesh->GetBodyInstance(Info->ParentBone);
		const int32	   ParentIndex = SkelMesh->GetBoneIndex(Info->ParentBone);
		if (!Child || ParentIndex == INDEX_NONE)
		{
			return 0.0f;
		}
		const FTransform ParentXf = SkelMesh->GetBoneTransform(ParentIndex);
		// Relative velocity of the child body w.r.t. the parent, in the parent's frame.
		if (Info->bLinear)
		{
			const FVector Rel = Child->GetUnrealWorldVelocity() - (Parent ? Parent->GetUnrealWorldVelocity() : FVector::ZeroVector);
			return static_cast<float>(ParentXf.InverseTransformVectorNoScale(Rel)[Info->Axis]);
		}
		const FVector Rel = Child->GetUnrealWorldAngularVelocityInRadians() - (Parent ? Parent->GetUnrealWorldAngularVelocityInRadians() : FVector::ZeroVector);
		return static_cast<float>(ParentXf.InverseTransformVectorNoScale(Rel)[Info->Axis]);
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
	FName Bone = BoneFor(MotorId);
	if (IsConstraintMotor(MotorId))
	{
		FConstraintMotor* Info = nullptr;
		if (ResolveConstraintMotor(MotorId, Info) && Info)
		{
			Bone = Info->ChildBone;
		}
	}
	const int32 BoneIndex = SkelMesh->GetBoneIndex(Bone);
	if (BoneIndex == INDEX_NONE)
	{
		return false;
	}
	OutWorld = SkelMesh->GetBoneTransform(BoneIndex);
	return true;
}
