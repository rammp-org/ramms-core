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
	if (bHasTarget)
	{
		return static_cast<float>(LastTarget.X);
	}
	// The resting pose can sit outside what the motors' ControlRanges allow --
	// the linkage settles under load a little past where the IK will follow, so
	// SolveTarget refuses the very point the endpoint is at. Handing that
	// coordinate to a height command would ask for a target that is infeasible
	// in the axis the caller did not touch, and the move silently does nothing.
	// Clamp into what is actually reachable instead.
	const FVector2D Live = GetCurrentEndpoint();
	bool			bValid = false;
	const FVector2D Range = GetReachableTranslationRange(static_cast<float>(Live.Y), bValid);
	return bValid ? FMath::Clamp(static_cast<float>(Live.X), static_cast<float>(Range.X),
						static_cast<float>(Range.Y))
				  : static_cast<float>(Live.X);
}

bool URamms5BarLinkageController::SetEndpointHeight(float Z)
{
	// Keep the current (or last-commanded) endpoint X, change only height.
	return SetEndpointTarget(FVector2D(HeldX(), Z));
}

float URamms5BarLinkageController::HeldZ() const
{
	if (bHasTarget)
	{
		return static_cast<float>(LastTarget.Y);
	}
	// See HeldX: the live height can be outside the reachable band, and a
	// fore/aft command that inherited it would be refused or do nothing.
	const FVector2D Live = GetCurrentEndpoint();
	bool			bValid = false;
	const FVector2D Range = GetReachableHeightRange(static_cast<float>(Live.X), bValid);
	return bValid ? FMath::Clamp(static_cast<float>(Live.Y), static_cast<float>(Range.X),
						static_cast<float>(Range.Y))
				  : static_cast<float>(Live.Y);
}

bool URamms5BarLinkageController::SetEndpointTranslation(float X)
{
	// The mirror of SetEndpointHeight: hold the height, move fore/aft.
	return SetEndpointTarget(FVector2D(X, HeldZ()));
}

FVector2D URamms5BarLinkageController::ScanReachable(TFunctionRef<bool(float)> Reachable,
	float Start, float Lo, float Hi, float Step, bool& bValid) const
{
	// Walk out from Start until the IK (with the motors' ControlRanges) first
	// refuses; the mechanism's reach is contiguous there. Starting at the live
	// value rather than the scan floor also keeps a disconnected reachable
	// island elsewhere from being reported.
	bValid = false;
	Step = FMath::Max(Step, 0.05f);
	const float S0 = FMath::Clamp(Start, Lo, Hi);

	// Find any reachable seed nearby (the live pose can sit a hair outside the
	// range while settling).
	float Seed = S0;
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
		return FVector2D(S0, S0);
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
	// A single reachable sample (equal scan limits, or an interval narrower
	// than the step) is not a range: a zero-width one would read as unbounded.
	bValid = Max > Min;
	return FVector2D(Min, Max);
}

FVector2D URamms5BarLinkageController::GetReachableHeightRange(float X, bool& bValid) const
{
	auto Reachable = [this, X](float Z) {
		bool bOk = false;
		SolveTarget(FVector2D(X, Z), bOk);
		return bOk;
	};
	return ScanReachable(Reachable, static_cast<float>(GetCurrentEndpoint().Y),
		static_cast<float>(FMath::Min(HeightScanLimits.X, HeightScanLimits.Y)),
		static_cast<float>(FMath::Max(HeightScanLimits.X, HeightScanLimits.Y)),
		HeightScanStep, bValid);
}

FVector2D URamms5BarLinkageController::GetReachableTranslationRange(float Z, bool& bValid) const
{
	auto Reachable = [this, Z](float X) {
		bool bOk = false;
		SolveTarget(FVector2D(X, Z), bOk);
		return bOk;
	};
	return ScanReachable(Reachable, static_cast<float>(GetCurrentEndpoint().X),
		static_cast<float>(FMath::Min(TranslationScanLimits.X, TranslationScanLimits.Y)),
		static_cast<float>(FMath::Max(TranslationScanLimits.X, TranslationScanLimits.Y)),
		HeightScanStep, bValid);
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

	// These angles replace whatever SetEndpointTarget commanded, so the
	// endpoint target has to follow them (forward kinematics) or the control
	// surface keeps reporting the old height. Angles that don't close the
	// linkage describe no endpoint at all: then there is no target.
	bool			bValid = false;
	const FVector2D Endpoint = URamms5BarKinematics::ComputeEndpoint(Resolved, AnglesAB, bValid);
	bHasTarget = bValid;
	if (bValid)
	{
		LastTarget = Endpoint;
	}
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

void URamms5BarLinkageController::DescribeEndpointAxis(FRammsControlSurface& OutSurface, bool bHeight) const
{
	FRammsControlAxis Axis;
	Axis.Id = bHeight ? HeightControlId() : TranslationControlId();
	Axis.Group = RammsControlIds::Groups::Linkage();
	const FString Pretty = GetName().Replace(TEXT("Linkage"), TEXT("")).Replace(TEXT("_"), TEXT(" "));
	Axis.DisplayName = FText::FromString(Pretty + (bHeight ? TEXT(" height") : TEXT(" fore/aft")));
	Axis.Kind = ERammsControlKind::Position;
	Axis.Units = ERammsControlUnits::Centimeters;
	Axis.Order = bHeight ? 0 : 1;
	Axis.Range = bHeight ? EndpointHeightRange : EndpointTranslationRange;
	if (Axis.Range.X >= Axis.Range.Y)
	{
		bool bValid = false;
		// Each axis is scanned with the OTHER one held where the endpoint
		// actually is, which is the pose the slider will move from.
		Axis.Range = bHeight ? GetReachableHeightRange(HeldX(), bValid)
							 : GetReachableTranslationRange(HeldZ(), bValid);
		if (!bValid)
		{
			// Nothing reachable along this axis here (no base, no table, or the
			// live pose is off the mechanism): an unbounded axis would accept
			// targets the setter then refuses, so offer nothing.
			UE_LOG(LogTemp, Verbose,
				TEXT("Ramms5BarLinkageController '%s': no reachable %s range; control not offered."),
				*GetName(), bHeight ? TEXT("height") : TEXT("translation"));
			return;
		}
	}
	const FVector2D Now = GetCurrentEndpoint();
	Axis.DefaultValue = FMath::Clamp(static_cast<float>(bHeight ? Now.Y : Now.X),
		static_cast<float>(Axis.Range.X), static_cast<float>(Axis.Range.Y));
	OutSurface.Add(Axis);
}

void URamms5BarLinkageController::DescribeControls(FRammsControlSurface& OutSurface) const
{
	// A 5-bar puts its endpoint anywhere in a plane, so it offers both degrees
	// of freedom rather than height alone. Both move the same two proximal
	// motors; which one a command changes depends only on which is held.
	DescribeEndpointAxis(OutSurface, /*bHeight=*/true);
	DescribeEndpointAxis(OutSurface, /*bHeight=*/false);
}

bool URamms5BarLinkageController::ApplyControl(FName Id, float Value)
{
	if (Id == HeightControlId())
	{
		return SetEndpointHeight(Value);
	}
	if (Id == TranslationControlId())
	{
		return SetEndpointTranslation(Value);
	}
	return false;
}

bool URamms5BarLinkageController::ReleaseControl(FName Id)
{
	URammsRobotBaseComponent* Base = EnsureBase();
	if ((Id != HeightControlId() && Id != TranslationControlId()) || !Base)
	{
		return false;
	}
	// One pair of motors holds the endpoint, so releasing either axis lets go
	// of the whole endpoint -- there is no way to keep holding the other.
	const bool bA = Base->ReleaseMotor(Resolved.ProximalMotorA);
	const bool bB = Base->ReleaseMotor(Resolved.ProximalMotorB);
	if (bA && bB)
	{
		bHasTarget = false; // no longer holding an endpoint (LastTarget stays as history)
	}
	return bA && bB;
}

bool URamms5BarLinkageController::ReadControl(FName Id, float& OutValue) const
{
	const FVector2D Now = GetCurrentEndpoint();
	if (Id == HeightControlId())
	{
		OutValue = static_cast<float>(Now.Y);
		return true;
	}
	if (Id == TranslationControlId())
	{
		OutValue = static_cast<float>(Now.X);
		return true;
	}
	return false;
}

bool URamms5BarLinkageController::ReadTarget(FName Id, float& OutTarget) const
{
	if (!bHasTarget)
	{
		return false;
	}
	if (Id == HeightControlId())
	{
		OutTarget = static_cast<float>(LastTarget.Y);
		return true;
	}
	if (Id == TranslationControlId())
	{
		OutTarget = static_cast<float>(LastTarget.X);
		return true;
	}
	return false;
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
