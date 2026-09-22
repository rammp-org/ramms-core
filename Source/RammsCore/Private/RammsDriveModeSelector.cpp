// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsDriveModeSelector.h"

#include "GameFramework/Actor.h"
#include "Ramms5BarLinkageController.h"
#include "RammsDriveMode.h"
#include "RammsRobotControlSurfaceComponent.h"

namespace
{
	IRammsDriveMode* AsMode(UActorComponent* Component)
	{
		return Cast<IRammsDriveMode>(Component);
	}
} // namespace

URammsDriveModeSelector::URammsDriveModeSelector()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URammsDriveModeSelector::BeginPlay()
{
	Super::BeginPlay();
	GatherModes();

	if (Modes.Num() == 0)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[DriveMode] '%s' found no drive modes on this robot; nothing to select."),
			*GetNameSafe(GetOwner()));
		return;
	}

	// Whatever the components defaulted to, exactly one ends up live.
	FName Wanted = InitialModeId;
	if (Wanted.IsNone())
	{
		if (IRammsDriveMode* First = AsMode(Modes[0]))
		{
			Wanted = First->GetDriveModeId();
		}
	}
	if (!SetActiveMode(Wanted))
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[DriveMode] '%s': no mode '%s' on this robot; falling back to the first."),
			*GetNameSafe(GetOwner()), *Wanted.ToString());
		if (IRammsDriveMode* First = AsMode(Modes[0]))
		{
			SetActiveMode(First->GetDriveModeId());
		}
	}
}

void URammsDriveModeSelector::GatherModes()
{
	Modes.Reset();
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}
	for (UActorComponent* Component : Owner->GetComponents())
	{
		if (Component && AsMode(Component))
		{
			Modes.Add(Component);
		}
	}
	// Stable order, so the enum's indices do not depend on component iteration.
	Modes.Sort([](const UActorComponent& A, const UActorComponent& B) {
		const IRammsDriveMode* ModeA = Cast<IRammsDriveMode>(&A);
		const IRammsDriveMode* ModeB = Cast<IRammsDriveMode>(&B);
		return ModeA && ModeB
			&& ModeA->GetDriveModeId().LexicalLess(ModeB->GetDriveModeId());
	});
}

TArray<FName> URammsDriveModeSelector::GetAvailableModeIds() const
{
	TArray<FName> Ids;
	for (UActorComponent* Component : Modes)
	{
		if (const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Component))
		{
			Ids.Add(Mode->GetDriveModeId());
		}
	}
	return Ids;
}

FName URammsDriveModeSelector::GetActiveModeId() const
{
	if (Modes.IsValidIndex(ActiveIndex))
	{
		if (const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Modes[ActiveIndex]))
		{
			return Mode->GetDriveModeId();
		}
	}
	return NAME_None;
}

void URammsDriveModeSelector::ApplyStanceFor(UActorComponent* ModeComponent)
{
	const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(ModeComponent);
	float				   Height = 0.0f;
	if (!bApplyLinkageStance || !Mode || !Mode->GetRequiredLinkageHeight(Height))
	{
		return;
	}
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}
	// Command the height rather than waiting for the linkage to be driven: the
	// wheels this mode needs on the ground are only there once the 5-bars hold
	// the right stance.
	TArray<URamms5BarLinkageController*> Linkages;
	Owner->GetComponents<URamms5BarLinkageController>(Linkages);
	for (URamms5BarLinkageController* Linkage : Linkages)
	{
		if (Linkage && !Linkage->SetEndpointHeight(Height))
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[DriveMode] '%s' could not take the %.1f cm stance this mode needs; "
					 "the wrong wheels may be on the ground."),
				*Linkage->GetName(), Height);
		}
	}
}

bool URammsDriveModeSelector::SetActiveMode(FName ModeId)
{
	int32 Wanted = INDEX_NONE;
	for (int32 i = 0; i < Modes.Num(); ++i)
	{
		const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Modes[i]);
		if (Mode && Mode->GetDriveModeId() == ModeId)
		{
			Wanted = i;
			break;
		}
	}
	if (Wanted == INDEX_NONE)
	{
		return false;
	}

	// Stand the others down first: two modes live at once would write the same
	// motors in one frame and offer duplicate control Ids.
	for (int32 i = 0; i < Modes.Num(); ++i)
	{
		if (i != Wanted)
		{
			if (IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Modes[i]))
			{
				Mode->SetDriveModeActive(false);
			}
		}
	}
	if (IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Modes[Wanted]))
	{
		Mode->SetDriveModeActive(true);
	}
	ActiveIndex = Wanted;
	ApplyStanceFor(Modes[Wanted]);

	// The surface is built from what the contributors describe, and the
	// inactive mode now describes nothing -- so its controls leave the panel.
	if (AActor* Owner = GetOwner())
	{
		if (URammsRobotControlSurfaceComponent* Surface =
				Owner->FindComponentByClass<URammsRobotControlSurfaceComponent>())
		{
			Surface->RebuildControlSurface();
		}
	}
	UE_LOG(LogTemp, Log, TEXT("[DriveMode] '%s' -> %s"), *GetNameSafe(GetOwner()), *ModeId.ToString());
	return true;
}

bool URammsDriveModeSelector::CycleMode()
{
	if (Modes.Num() < 2)
	{
		return false;
	}
	const int32			   Next = (ActiveIndex + 1) % Modes.Num();
	const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Modes[Next]);
	return Mode && SetActiveMode(Mode->GetDriveModeId());
}

// --- control surface -----------------------------------------------------------

void URammsDriveModeSelector::DescribeControls(FRammsControlSurface& OutSurface) const
{
	if (Modes.Num() < 2)
	{
		// Nothing to choose between; offering a one-entry selector is noise.
		return;
	}
	FRammsControlAxis Axis;
	Axis.Id = RammsControlIds::Drive::Mode();
	Axis.Group = RammsControlIds::Groups::Drive();
	Axis.DisplayName = NSLOCTEXT("Ramms", "DriveMode", "Drive mode");
	Axis.Kind = ERammsControlKind::Enum;
	Axis.Units = ERammsControlUnits::None;
	Axis.Order = -1; // above the sticks it governs
	for (UActorComponent* Component : Modes)
	{
		if (const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Component))
		{
			Axis.EnumLabels.Add(Mode->GetDriveModeDisplayName());
		}
	}
	Axis.Range = FVector2D(0.0, FMath::Max(0, Axis.EnumLabels.Num() - 1));
	Axis.DefaultValue = static_cast<float>(FMath::Max(ActiveIndex, 0));
	OutSurface.Add(Axis);
}

bool URammsDriveModeSelector::ApplyControl(FName Id, float Value)
{
	if (Id != RammsControlIds::Drive::Mode())
	{
		return false;
	}
	const int32 Index = FMath::RoundToInt(Value);
	if (!Modes.IsValidIndex(Index))
	{
		return false;
	}
	const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Modes[Index]);
	return Mode && SetActiveMode(Mode->GetDriveModeId());
}

bool URammsDriveModeSelector::TriggerControl(FName Id)
{
	return Id == RammsControlIds::Drive::Mode() && CycleMode();
}

bool URammsDriveModeSelector::ReadControl(FName Id, float& OutValue) const
{
	if (Id != RammsControlIds::Drive::Mode())
	{
		return false;
	}
	OutValue = static_cast<float>(FMath::Max(ActiveIndex, 0));
	return true;
}
