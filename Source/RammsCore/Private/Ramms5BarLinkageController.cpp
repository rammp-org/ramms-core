// Copyright Epic Games, Inc. All Rights Reserved.

#include "Ramms5BarLinkageController.h"
#include "Ramms5BarKinematics.h"
#include "RammsRobotBaseComponent.h"
#include "GameFramework/Actor.h"

URamms5BarLinkageController::URamms5BarLinkageController()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URamms5BarLinkageController::BeginPlay()
{
	Super::BeginPlay();

	// Resolve the kinematic spec: a table row if configured, else the inline one.
	Resolved = Linkage;
	if (KinematicTable && !LinkageRow.IsNone())
	{
		static const FString Context(TEXT("Ramms5BarLinkageController"));
		if (const FRamms5BarLinkageSpec* Row = KinematicTable->FindRow<FRamms5BarLinkageSpec>(LinkageRow, Context))
		{
			Resolved = *Row;
		}
		else
		{
			UE_LOG(LogTemp, Warning,
				TEXT("Ramms5BarLinkageController on '%s': row '%s' not found in the kinematic table; using inline config."),
				GetOwner() ? *GetOwner()->GetName() : TEXT("?"), *LinkageRow.ToString());
		}
	}

	// Prime the base-component cache (also resolved lazily if BeginPlay order
	// hasn't reached the base component yet).
	EnsureBase();
}

URammsRobotBaseComponent* URamms5BarLinkageController::EnsureBase() const
{
	if (!BaseComponent)
	{
		if (const AActor* Owner = GetOwner())
		{
			BaseComponent = Owner->FindComponentByClass<URammsRobotBaseComponent>();
			if (BaseComponent)
			{
				TArray<URammsRobotBaseComponent*> Bases;
				Owner->GetComponents<URammsRobotBaseComponent>(Bases);
				if (Bases.Num() > 1)
				{
					UE_LOG(LogTemp, Warning,
						TEXT("Ramms5BarLinkageController '%s': '%s' has %d RammsRobotBaseComponents; using '%s'. Remove the extras."),
						*GetName(), *Owner->GetName(), Bases.Num(), *BaseComponent->GetName());
				}
			}
		}
	}
	return BaseComponent;
}

bool URamms5BarLinkageController::HasBase() const
{
	return EnsureBase() != nullptr;
}

bool URamms5BarLinkageController::WithinMotorRange(URammsRobotBaseComponent& Base, FName MotorId, double Angle) const
{
	// A geometrically reachable endpoint can still need a hip angle outside the
	// actuator's authored range; the base component would clamp it silently and
	// the linkage would settle somewhere else. Treat that as unreachable. (An
	// unset/zero-width range defers to the backend and can't be checked here.)
	FRammsMotorSpec Spec;
	if (Base.GetMotorSpec(MotorId, Spec) && Spec.ControlRange.X < Spec.ControlRange.Y)
	{
		return Angle >= Spec.ControlRange.X && Angle <= Spec.ControlRange.Y;
	}
	return true;
}

bool URamms5BarLinkageController::SetEndpointTarget(FVector2D TargetXZ)
{
	URammsRobotBaseComponent* Base = EnsureBase();
	if (!Base)
	{
		return false;
	}

	bool			bReachable = false;
	const FVector2D Angles = URamms5BarKinematics::SolveIK(Resolved, TargetXZ, bReachable);
	bReachable = bReachable
		&& WithinMotorRange(*Base, Resolved.ProximalMotorA, Angles.X)
		&& WithinMotorRange(*Base, Resolved.ProximalMotorB, Angles.Y);
	if (!bReachable)
	{
		UE_LOG(LogTemp, Verbose,
			TEXT("Ramms5BarLinkageController: endpoint (%.2f, %.2f) unreachable — not commanding."),
			TargetXZ.X, TargetXZ.Y);
		return false;
	}

	Base->SetMotorCommand(Resolved.ProximalMotorA, static_cast<float>(Angles.X));
	Base->SetMotorCommand(Resolved.ProximalMotorB, static_cast<float>(Angles.Y));
	LastTarget = TargetXZ;
	bHasTarget = true;
	return true;
}

bool URamms5BarLinkageController::SetEndpointHeight(float Z)
{
	// Keep the current (or last-commanded) endpoint X, change only height.
	const float X = bHasTarget ? static_cast<float>(LastTarget.X) : static_cast<float>(GetCurrentEndpoint().X);
	return SetEndpointTarget(FVector2D(X, Z));
}

void URamms5BarLinkageController::SetJointAngles(FVector2D AnglesAB)
{
	URammsRobotBaseComponent* Base = EnsureBase();
	if (!Base)
	{
		return;
	}
	Base->SetMotorCommand(Resolved.ProximalMotorA, static_cast<float>(AnglesAB.X));
	Base->SetMotorCommand(Resolved.ProximalMotorB, static_cast<float>(AnglesAB.Y));
}

FVector2D URamms5BarLinkageController::GetCurrentJointAngles() const
{
	URammsRobotBaseComponent* Base = EnsureBase();
	if (!Base)
	{
		return FVector2D::ZeroVector;
	}
	return FVector2D(Base->GetMotorValue(Resolved.ProximalMotorA), Base->GetMotorValue(Resolved.ProximalMotorB));
}

FVector2D URamms5BarLinkageController::GetCurrentEndpoint() const
{
	bool bValid = false;
	return GetCurrentEndpointChecked(bValid);
}

FVector2D URamms5BarLinkageController::GetCurrentEndpointChecked(bool& bValid) const
{
	return URamms5BarKinematics::ComputeEndpoint(Resolved, GetCurrentJointAngles(), bValid);
}

FVector2D URamms5BarLinkageController::SolveTarget(FVector2D TargetXZ, bool& bReachable) const
{
	const FVector2D Angles = URamms5BarKinematics::SolveIK(Resolved, TargetXZ, bReachable);
	if (URammsRobotBaseComponent* Base = EnsureBase())
	{
		bReachable = bReachable
			&& WithinMotorRange(*Base, Resolved.ProximalMotorA, Angles.X)
			&& WithinMotorRange(*Base, Resolved.ProximalMotorB, Angles.Y);
	}
	return Angles;
}
