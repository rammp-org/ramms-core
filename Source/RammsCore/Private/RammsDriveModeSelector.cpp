// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsDriveModeSelector.h"

#include "GameFramework/Actor.h"
#include "RammsDriveMode.h"
#include "RammsRobotBaseComponent.h"
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

int32 URammsDriveModeSelector::CommandLinkageAxes(URammsRobotControlSurfaceComponent& Surface,
	const TCHAR* IdSuffix, float Value, const TCHAR* What, bool bWarn) const
{
	int32		  Commanded = 0;
	const FString Suffix(IdSuffix);
	for (const FRammsControlAxis& Axis : Surface.DescribeControlSurface().Axes)
	{
		if (Axis.Group != RammsControlIds::Groups::Linkage()
			|| Axis.Kind != ERammsControlKind::Position || Axis.bReadOnly
			|| !Axis.Id.ToString().EndsWith(Suffix))
		{
			continue;
		}
		// Autonomy: taking a stance is the robot's own doing, and it has to
		// win over whatever a panel last left on the axis.
		if (Surface.SetControl(Axis.Id, Value, ERammsControlSource::Autonomy))
		{
			++Commanded;
		}
		else if (bWarn)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[DriveMode] '%s' refused the %.1f cm %s its mode's stance needs; the wrong "
					 "wheels may be on the ground."),
				*Axis.Id.ToString(), Value, What);
		}
	}
	return Commanded;
}

void URammsDriveModeSelector::ApplyStanceFor(UActorComponent* ModeComponent)
{
	const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(ModeComponent);
	FRammsDriveStance	   Stance;
	if (!bApplyLinkageStance || !Mode || !Mode->GetRequiredStance(Stance))
	{
		return;
	}
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	// Command the stance rather than waiting for the robot to be driven into
	// it: the wheels this mode needs on the ground are only there once the
	// legs hold the right pose.
	//
	// Through the control surface by Id, not by calling a linkage controller.
	// Naming URamms5BarLinkageController here would leave this selector
	// depending on ramms-controllers once that class moves, which is exactly
	// what the extraction is meant to avoid -- and it would only ever work for
	// one kind of leg. Anything that advertises a linkage height can take a
	// stance.
	URammsRobotControlSurfaceComponent* Surface =
		Owner->FindComponentByClass<URammsRobotControlSurfaceComponent>();
	if (Stance.bHasLinkageHeight || Stance.bHasLinkageTranslation)
	{
		if (!Surface)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[DriveMode] '%s' needs a linkage stance but the robot has no control surface to "
					 "command it through; the wrong wheels may be on the ground."),
				*GetNameSafe(ModeComponent));
		}
		else
		{
			// Two passes, fore/aft then height each time, because the axes are
			// one endpoint over one pair of motors and the reachable set is a
			// curved region: which order works depends on which way you are
			// going. Reaching a high, narrow pose needs the endpoint brought
			// back to the centre first, or the height is refused at the
			// current fore/aft; reaching a far-out one needs the height
			// dropped first, or the fore/aft is refused. Alternating twice
			// gets there either way, and a command already satisfied is a
			// no-op the second time.
			int32 Commanded = 0;
			for (int32 Pass = 0; Pass < 2; ++Pass)
			{
				Commanded = 0;
				if (Stance.bHasLinkageTranslation)
				{
					Commanded += CommandLinkageAxes(*Surface,
						RammsControlIds::Linkage::TranslationSuffix(), Stance.LinkageTranslationCm,
						TEXT("fore/aft"), /*bWarn=*/Pass == 1);
				}
				if (Stance.bHasLinkageHeight)
				{
					Commanded += CommandLinkageAxes(*Surface, RammsControlIds::Linkage::HeightSuffix(),
						Stance.LinkageHeightCm, TEXT("height"), /*bWarn=*/Pass == 1);
				}
			}
			if (Commanded == 0)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("[DriveMode] '%s' needs a linkage stance but this robot advertises no linkage "
						 "controls to command."),
					*GetNameSafe(ModeComponent));
			}
		}
	}

	// The corner cranks and anything else the stance names. These go through
	// the robot base by Id: they are ordinary registry motors, not something
	// the mode claims, so the low-level mode can still reach them.
	if (Stance.MotorTargets.Num() > 0)
	{
		URammsRobotBaseComponent* Base = Owner->FindComponentByClass<URammsRobotBaseComponent>();
		if (!Base)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[DriveMode] '%s' wants %d motors held for its stance but the robot has no "
					 "RammsRobotBaseComponent to command them through."),
				*GetNameSafe(ModeComponent), Stance.MotorTargets.Num());
			return;
		}
		for (const FRammsStanceMotorTarget& Target : Stance.MotorTargets)
		{
			if (Target.MotorId.IsNone())
			{
				continue;
			}
			if (!Base->HasMotor(Target.MotorId))
			{
				// Naming a motor this robot does not have is an authoring
				// mistake worth hearing about: the stance silently half-applies
				// and the wrong wheels stay down.
				UE_LOG(LogTemp, Warning,
					TEXT("[DriveMode] stance for '%s' names motor '%s', which is not in this "
						 "robot's registry; that part of the stance is not applied."),
					*GetNameSafe(ModeComponent), *Target.MotorId.ToString());
				continue;
			}
			Base->SetMotorCommand(Target.MotorId, Target.Target);
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

	// Rebuild before applying the stance, not after. The surface is built from
	// what the contributors describe, and the inactive mode now describes
	// nothing -- so its controls leave the panel. It also has to be current
	// *before* the stance goes out, because the stance is commanded through it
	// by Id: coming back from the low-level mode the 5-bars have just been
	// un-suspended, and against a stale surface they still advertise nothing,
	// so the stance found no linkage controls to command and the robot stayed
	// in the wrong pose.
	if (AActor* Owner = GetOwner())
	{
		if (URammsRobotControlSurfaceComponent* Surface =
				Owner->FindComponentByClass<URammsRobotControlSurfaceComponent>())
		{
			Surface->RebuildControlSurface();
		}
	}
	ApplyStanceFor(Modes[Wanted]);
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
