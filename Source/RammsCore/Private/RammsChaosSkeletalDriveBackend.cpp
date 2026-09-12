// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsChaosSkeletalDriveBackend.h"
#include "RammsDifferentialDriveController.h"
#include "RammsDifferentialDriveLibrary.h"
#include "RammsDifferentialDriveTypes.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"

bool FRammsChaosSkeletalDriveBackend::Initialize(URammsDifferentialDriveController& InController)
{
	Controller = &InController;
	SkeletalMesh = nullptr;

	AActor* Owner = InController.GetOwner();
	if (!Owner)
	{
		return false;
	}

	// Resolve the skeletal mesh carrying the wheel bones: by explicit name, else
	// the first one on the owner (unchanged from the controller's old BeginPlay).
	USkeletalMeshComponent* Resolved = nullptr;
	if (InController.SkeletalMeshComponentName != NAME_None)
	{
		TArray<USkeletalMeshComponent*> SkeletalMeshes;
		Owner->GetComponents<USkeletalMeshComponent>(SkeletalMeshes);
		for (USkeletalMeshComponent* SkelMesh : SkeletalMeshes)
		{
			if (SkelMesh && SkelMesh->GetFName() == InController.SkeletalMeshComponentName)
			{
				Resolved = SkelMesh;
				break;
			}
		}
	}
	else
	{
		Resolved = Owner->FindComponentByClass<USkeletalMeshComponent>();
	}

	if (!Resolved)
	{
		UE_LOG(LogTemp, Warning, TEXT("RammsDifferentialDriveController: Failed to find skeletal mesh component on %s"), *Owner->GetName());
		return false;
	}
	SkeletalMesh = Resolved;

	// Cap each wheel body's angular velocity at its motor's max RPM.
	if (FBodyInstance* LeftBodyInst = BodyInstanceFor(ERammsDriveWheel::Left))
	{
		const float MaxAngularVelRad = URammsDifferentialDriveLibrary::RPMToRadPerSec(InController.LeftMotorParams.MaxRPM);
		LeftBodyInst->SetMaxAngularVelocityInRadians(MaxAngularVelRad, false, true);
		if (InController.bEnableDebugLogging)
		{
			UE_LOG(LogTemp, Log, TEXT("[DiffDrive] Set left wheel max angular velocity: %.1f RPM (%.3f rad/s)"),
				InController.LeftMotorParams.MaxRPM, MaxAngularVelRad);
		}
	}
	if (FBodyInstance* RightBodyInst = BodyInstanceFor(ERammsDriveWheel::Right))
	{
		const float MaxAngularVelRad = URammsDifferentialDriveLibrary::RPMToRadPerSec(InController.RightMotorParams.MaxRPM);
		RightBodyInst->SetMaxAngularVelocityInRadians(MaxAngularVelRad, false, true);
		if (InController.bEnableDebugLogging)
		{
			UE_LOG(LogTemp, Log, TEXT("[DiffDrive] Set right wheel max angular velocity: %.1f RPM (%.3f rad/s)"),
				InController.RightMotorParams.MaxRPM, MaxAngularVelRad);
		}
	}

	return true;
}

FName FRammsChaosSkeletalDriveBackend::BoneNameFor(ERammsDriveWheel Wheel) const
{
	const URammsDifferentialDriveController* Ctrl = Controller.Get();
	if (!Ctrl)
	{
		return NAME_None;
	}
	return Wheel == ERammsDriveWheel::Left ? Ctrl->LeftWheelBoneName : Ctrl->RightWheelBoneName;
}

FBodyInstance* FRammsChaosSkeletalDriveBackend::BodyInstanceFor(ERammsDriveWheel Wheel) const
{
	USkeletalMeshComponent* Skel = SkeletalMesh.Get();
	const FName				BoneName = BoneNameFor(Wheel);
	if (!Skel || BoneName == NAME_None)
	{
		return nullptr;
	}
	return Skel->GetBodyInstance(BoneName);
}

void FRammsChaosSkeletalDriveBackend::ReadWheelState(ERammsDriveWheel Wheel, FWheelState& OutState)
{
	URammsDifferentialDriveController* Ctrl = Controller.Get();
	USkeletalMeshComponent*			   Skel = SkeletalMesh.Get();
	FBodyInstance*					   BodyInst = BodyInstanceFor(Wheel);
	if (!Ctrl || !Skel || !BodyInst || !BodyInst->IsInstanceSimulatingPhysics())
	{
		return;
	}
	const FName BoneName = BoneNameFor(Wheel);

	// Angular velocity around the wheel's spin axis (local Y).
	FVector AngularVelocity = BodyInst->GetUnrealWorldAngularVelocityInRadians();

	const int32 BoneIndex = Skel->GetBoneIndex(BoneName);
	if (BoneIndex == INDEX_NONE)
	{
		return;
	}

	FTransform BoneTransform = Skel->GetBoneTransform(BoneIndex);
	FVector	   LocalAngularVelocity = BoneTransform.InverseTransformVectorNoScale(AngularVelocity);

	OutState.AngularVelocity = LocalAngularVelocity.Y;
	OutState.LinearVelocity = OutState.AngularVelocity * Ctrl->WheelRadius;

	FVector		WheelVelocity = BodyInst->GetUnrealWorldVelocity();
	FVector		ForwardDir = BoneTransform.GetUnitAxis(EAxis::X);
	FVector		RightDir = BoneTransform.GetUnitAxis(EAxis::Y);
	const float ActualLinearVel = FVector::DotProduct(WheelVelocity, ForwardDir);
	OutState.LateralVelocity = FVector::DotProduct(WheelVelocity, RightDir);

	// Suspension load (normal force on the wheel).
	if (Ctrl->bEnableLoadDependentTraction)
	{
		const float GravityForce = Ctrl->VehicleMass * 980.665f; // kg -> N (g = 9.80665 m/s^2)
		OutState.SuspensionLoad = GravityForce * 0.5f;			 // split between two wheels
	}
	else
	{
		OutState.SuspensionLoad = 0.0f;
	}

	// Surface friction from the physical material under the wheel.
	OutState.SurfaceFriction = 1.0f;
	if (Ctrl->bUsePhysicalMaterialFriction)
	{
		FVector WheelLocation = BoneTransform.GetLocation();
		FVector TraceStart = WheelLocation;
		FVector TraceEnd = WheelLocation - FVector(0, 0, Ctrl->WheelRadius * 2.0f);

		FHitResult			  HitResult;
		FCollisionQueryParams QueryParams;
		QueryParams.AddIgnoredActor(Ctrl->GetOwner());

		if (UWorld* World = Ctrl->GetWorld())
		{
			if (World->LineTraceSingleByChannel(HitResult, TraceStart, TraceEnd, ECC_Visibility, QueryParams))
			{
				if (HitResult.PhysMaterial.IsValid())
				{
					OutState.SurfaceFriction = HitResult.PhysMaterial->Friction;
				}
			}
		}
	}

	// Slip ratio.
	if (Ctrl->bEnableSlipModeling && FMath::Abs(OutState.LinearVelocity) > SMALL_NUMBER)
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

void FRammsChaosSkeletalDriveBackend::ApplyWheelTorque(ERammsDriveWheel Wheel, float RequestedTorque,
	const FMotorParameters& MotorParams, FWheelState& WheelState)
{
	URammsDifferentialDriveController* Ctrl = Controller.Get();
	USkeletalMeshComponent*			   Skel = SkeletalMesh.Get();
	FBodyInstance*					   BodyInst = BodyInstanceFor(Wheel);
	if (!Ctrl || !Skel || !BodyInst || !BodyInst->IsInstanceSimulatingPhysics())
	{
		WheelState.AppliedTorque = 0.0f;
		return;
	}
	const FName BoneName = BoneNameFor(Wheel);

	const float CurrentRPM = URammsDifferentialDriveLibrary::RadPerSecToRPM(WheelState.AngularVelocity);
	float		AvailableTorque = URammsDifferentialDriveLibrary::EvaluateMotorTorque(
		CurrentRPM,
		MotorParams,
		RequestedTorque);

	if (Ctrl->bEnableSlipModeling)
	{
		const float TractionMultiplier = URammsDifferentialDriveLibrary::GetTractionMultiplierFromSlip(
			WheelState.SlipRatio,
			Ctrl->PeakSlipRatio);

		if (Ctrl->bEnableLoadDependentTraction && WheelState.SuspensionLoad > 0.0f)
		{
			const float AvailableGripForce = URammsDifferentialDriveLibrary::CalculateAvailableGrip(
				WheelState.SuspensionLoad,
				WheelState.SurfaceFriction,
				Ctrl->TractionCoefficient);
			const float MaxTorqueFromGrip = (AvailableGripForce * (Ctrl->WheelRadius / 100.0f)) * TractionMultiplier;
			AvailableTorque = FMath::Min(FMath::Abs(AvailableTorque), MaxTorqueFromGrip) * FMath::Sign(AvailableTorque);
		}
		else
		{
			AvailableTorque *= TractionMultiplier;
		}

		if (Ctrl->bEnableLateralSlipResistance && FMath::Abs(WheelState.LateralVelocity) > Ctrl->LateralSlipThreshold)
		{
			const float LateralSlipFactor = FMath::Clamp(
				1.0f - (FMath::Abs(WheelState.LateralVelocity) - Ctrl->LateralSlipThreshold) / 100.0f,
				0.3f,
				1.0f);
			AvailableTorque *= LateralSlipFactor;
		}
	}

	WheelState.AppliedTorque = AvailableTorque;

	if (Ctrl->bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[DiffDrive] ApplyTorque to %s: Requested=%.3f Nm, Available=%.3f Nm, CurrentRPM=%.1f, Slip=%.3f, Load=%.1f N, Friction=%.2f, LatVel=%.1f cm/s"),
			*BoneName.ToString(), RequestedTorque, AvailableTorque, CurrentRPM, WheelState.SlipRatio,
			WheelState.SuspensionLoad, WheelState.SurfaceFriction, WheelState.LateralVelocity);
	}

	const int32 BoneIndex = Skel->GetBoneIndex(BoneName);
	if (BoneIndex == INDEX_NONE)
	{
		return;
	}

	FTransform BoneTransform = Skel->GetBoneTransform(BoneIndex);
	FVector	   LocalTorque = FVector(0.0f, AvailableTorque, 0.0f);
	FVector	   WorldTorque = BoneTransform.TransformVectorNoScale(LocalTorque);
	WorldTorque *= 100.0f; // N*m -> UE units (kg*cm^2/s^2)

	if (Ctrl->bEnableDebugLogging)
	{
		UE_LOG(LogTemp, Log, TEXT("[DiffDrive] Final WorldTorque for %s: X=%.1f, Y=%.1f, Z=%.1f (UE units)"),
			*BoneName.ToString(), WorldTorque.X, WorldTorque.Y, WorldTorque.Z);
	}

	BodyInst->AddTorqueInRadians(WorldTorque, false, true);
}

void FRammsChaosSkeletalDriveBackend::ApplyBrake(ERammsDriveWheel Wheel, FWheelState& WheelState)
{
	URammsDifferentialDriveController* Ctrl = Controller.Get();
	FBodyInstance*					   BodyInst = BodyInstanceFor(Wheel);
	if (!Ctrl || !BodyInst || !BodyInst->IsInstanceSimulatingPhysics())
	{
		return;
	}

	// Deadband: don't brake below this angular velocity, to avoid oscillation.
	const float BrakeDeadband = 0.1f; // rad/s
	FVector		AngVel = BodyInst->GetUnrealWorldAngularVelocityInRadians();
	const float AngVelMagnitude = AngVel.Size();

	if (AngVelMagnitude > BrakeDeadband)
	{
		FVector DampingTorque = -AngVel * Ctrl->BrakeTorque * 100.0f; // -> UE units
		BodyInst->AddTorqueInRadians(DampingTorque, false, true);
		WheelState.AppliedTorque = -FMath::Sign(WheelState.AngularVelocity) * Ctrl->BrakeTorque;

		if (Ctrl->bEnableDebugLogging)
		{
			UE_LOG(LogTemp, Log, TEXT("[DiffDrive] Applying %s brake: Damping=%.3f Nm (AngVel: %.3f rad/s)"),
				Wheel == ERammsDriveWheel::Left ? TEXT("LEFT") : TEXT("RIGHT"),
				Ctrl->BrakeTorque, WheelState.AngularVelocity);
		}
	}
	else
	{
		WheelState.AppliedTorque = 0.0f;
	}
}
