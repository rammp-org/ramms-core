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

float URamms5BarLinkageController::HeldX() const
{
	return bHasTarget ? static_cast<float>(LastTarget.X) : static_cast<float>(GetCurrentEndpoint().X);
}

bool URamms5BarLinkageController::SetEndpointHeight(float Z)
{
	// Keep the current (or last-commanded) endpoint X, change only height.
	return SetEndpointTarget(FVector2D(HeldX(), Z));
}

FVector2D URamms5BarLinkageController::GetReachableHeightRange(float X, bool& bValid) const
{
	// Walk up and down from the current height until the IK (with the motors'
	// ControlRanges) first refuses; the mechanism's reach is contiguous there.
	// Starting at the live height rather than the scan floor also keeps a
	// disconnected reachable island elsewhere from being reported.
	bValid = false;
	const float Step = FMath::Max(HeightScanStep, 0.05f);
	const float Lo = static_cast<float>(FMath::Min(HeightScanLimits.X, HeightScanLimits.Y));
	const float Hi = static_cast<float>(FMath::Max(HeightScanLimits.X, HeightScanLimits.Y));
	const float Z0 = FMath::Clamp(static_cast<float>(GetCurrentEndpoint().Y), Lo, Hi);

	auto Reachable = [this, X](float Z) {
		bool bOk = false;
		SolveTarget(FVector2D(X, Z), bOk);
		return bOk;
	};

	// Find any reachable seed near the current height (the live pose can sit a
	// hair outside the range while settling).
	float Seed = Z0;
	bool  bSeed = Reachable(Seed);
	for (float D = Step; !bSeed && D <= 8.0f * Step; D += Step)
	{
		if (Seed - D >= Lo && Reachable(Seed - D))
		{
			Seed -= D;
			bSeed = true;
		}
		else if (Seed + D <= Hi && Reachable(Seed + D))
		{
			Seed += D;
			bSeed = true;
		}
	}
	if (!bSeed)
	{
		return FVector2D(Z0, Z0);
	}

	float Min = Seed;
	while (Min - Step >= Lo && Reachable(Min - Step))
	{
		Min -= Step;
	}
	float Max = Seed;
	while (Max + Step <= Hi && Reachable(Max + Step))
	{
		Max += Step;
	}
	bValid = true;
	return FVector2D(Min, Max);
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

// --- control surface -----------------------------------------------------------

void URamms5BarLinkageController::DescribeControls(FRammsControlSurface& OutSurface) const
{
	FRammsControlAxis Axis;
	Axis.Id = HeightControlId();
	Axis.Group = FName("Linkage");
	Axis.DisplayName = FText::FromString(GetName().Replace(TEXT("Linkage"), TEXT("")).Replace(TEXT("_"), TEXT(" ")) + TEXT(" height"));
	Axis.Kind = ERammsControlKind::Position;
	Axis.Units = ERammsControlUnits::Centimeters;
	Axis.Range = EndpointHeightRange;
	if (EndpointHeightRange.X >= EndpointHeightRange.Y)
	{
		bool bValid = false;
		Axis.Range = GetReachableHeightRange(HeldX(), bValid);
	}
	Axis.DefaultValue = FMath::Clamp(static_cast<float>(GetCurrentEndpoint().Y), static_cast<float>(Axis.Range.X), static_cast<float>(Axis.Range.Y));
	OutSurface.Add(Axis);
}

bool URamms5BarLinkageController::ApplyControl(FName Id, float Value)
{
	return Id == HeightControlId() && SetEndpointHeight(Value);
}

bool URamms5BarLinkageController::ReleaseControl(FName Id)
{
	URammsRobotBaseComponent* Base = EnsureBase();
	if (Id != HeightControlId() || !Base)
	{
		return false;
	}
	const bool bA = Base->ReleaseMotor(Resolved.ProximalMotorA);
	const bool bB = Base->ReleaseMotor(Resolved.ProximalMotorB);
	return bA && bB;
}

bool URamms5BarLinkageController::ReadControl(FName Id, float& OutValue) const
{
	if (Id != HeightControlId())
	{
		return false;
	}
	OutValue = static_cast<float>(GetCurrentEndpoint().Y);
	return true;
}

void URamms5BarLinkageController::GetClaimedMotorIds(TArray<FName>& OutIds) const
{
	if (!Resolved.ProximalMotorA.IsNone())
	{
		OutIds.Add(Resolved.ProximalMotorA);
	}
	if (!Resolved.ProximalMotorB.IsNone())
	{
		OutIds.Add(Resolved.ProximalMotorB);
	}
}
