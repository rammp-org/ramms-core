// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRobotSelectComponent.h"
#include "RammsRobotDefinition.h"
#include "RammsRobotSelection.h"
#include "Engine/World.h"
#include "UObject/SoftObjectPath.h"

URammsRobotSelectComponent::URammsRobotSelectComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URammsRobotSelectComponent::BeginPlay()
{
	Super::BeginPlay();
	Refresh();
}

void URammsRobotSelectComponent::Refresh()
{
	Choices.Reset();
	for (URammsRobotDefinition* Def : URammsRobotCatalog::GetDefinitions(/*bIncludeHidden=*/false))
	{
		Choices.Add(Def);
	}
	// The robot running is offered even when hidden, so the choice can show it.
	const URammsRobotSelectionSubsystem* Selection = GetWorld() ? GetWorld()->GetSubsystem<URammsRobotSelectionSubsystem>() : nullptr;
	const FRammsRobotChoice				 Active = Selection ? Selection->GetActiveChoice() : FRammsRobotChoice();
	if (Active.Definition && !Choices.Contains(Active.Definition))
	{
		Choices.Add(Active.Definition);
	}
	// A pawn started from its class path is offered first, as itself, so loading
	// with nothing changed starts it again rather than some definition.
	RunningClassPath = Active.Definition ? FString() : Active.ClassPath;
	if (!RunningClassPath.IsEmpty())
	{
		Choices.Insert(nullptr, 0);
	}
	Staged = Active.Definition ? Choices.IndexOfByKey(Active.Definition) : (Choices.Num() ? 0 : -1);
}

int32 URammsRobotSelectComponent::IndexOf(const FString& IdOrAlias) const
{
	const FString Words = IdOrAlias.TrimStartAndEnd();
	return Choices.IndexOfByPredicate([this, &Words](const URammsRobotDefinition* Def) {
		return Def ? Def->Matches(Words) : (!RunningClassPath.IsEmpty() && RunningClassPath.Equals(Words, ESearchCase::IgnoreCase));
	});
}

FString URammsRobotSelectComponent::GetStagedRobot() const
{
	if (!Choices.IsValidIndex(Staged))
	{
		return FString();
	}
	return Choices[Staged] ? Choices[Staged]->RobotId.ToString() : RunningClassPath;
}

bool URammsRobotSelectComponent::StepStagedRobot(int32 Delta)
{
	const int32 Num = Choices.Num();
	if (Num == 0)
	{
		return false;
	}
	const int32 From = Choices.IsValidIndex(Staged) ? Staged : (Delta >= 0 ? -1 : 0);
	Staged = ((From + Delta) % Num + Num) % Num;
	return true;
}

bool URammsRobotSelectComponent::StageRobot(const FString& IdOrAlias)
{
	const int32 Index = IndexOf(IdOrAlias);
	if (Index == INDEX_NONE)
	{
		return false;
	}
	Staged = Index;
	return true;
}

bool URammsRobotSelectComponent::LoadStagedRobot()
{
	const FString Robot = GetStagedRobot();
	return !Robot.IsEmpty() && URammsRobotCatalog::RestartWithRobot(GetWorld(), Robot);
}

void URammsRobotSelectComponent::DescribeControls(FRammsControlSurface& OutSurface) const
{
	FRammsControlAxis Active;
	Active.Id = ActiveControlId();
	Active.Group = GroupName();
	Active.DisplayName = NSLOCTEXT("Ramms", "RobotActive", "Robot");
	Active.Kind = ERammsControlKind::Text;
	Active.Units = ERammsControlUnits::None;
	Active.bReadOnly = true;
	Active.Order = 0;
	OutSurface.Add(Active);

	if (Choices.Num() == 0)
	{
		return;
	}
	FRammsControlAxis Choice;
	Choice.Id = RobotControlId();
	Choice.Group = GroupName();
	Choice.DisplayName = NSLOCTEXT("Ramms", "RobotChoice", "Start with");
	Choice.Kind = ERammsControlKind::Enum;
	Choice.Units = ERammsControlUnits::None;
	Choice.Order = 1;
	// The running class path's entry, by the class's name (BP_MyRobot, not BP_MyRobot_C).
	FString RunningName = FSoftObjectPath(RunningClassPath).GetAssetName();
	RunningName.RemoveFromEnd(TEXT("_C"));
	for (const URammsRobotDefinition* Def : Choices)
	{
		Choice.EnumLabels.Add(Def ? Def->GetDisplayNameOrId()
								  : FText::Format(NSLOCTEXT("Ramms", "RobotRunningClass", "{0} (running)"), FText::FromString(RunningName)));
	}
	Choice.Range = FVector2D(0.0, FMath::Max(0, Choices.Num() - 1));
	Choice.DefaultValue = static_cast<float>(FMath::Max(Staged, 0));
	OutSurface.Add(Choice);

	FRammsControlAxis Load;
	Load.Id = LoadControlId();
	Load.Group = GroupName();
	Load.DisplayName = NSLOCTEXT("Ramms", "RobotLoad", "Restart with this robot");
	Load.Kind = ERammsControlKind::Action;
	Load.Units = ERammsControlUnits::None;
	Load.bReadback = false;
	Load.Order = 2;
	OutSurface.Add(Load);
}

bool URammsRobotSelectComponent::ApplyControl(FName Id, float Value)
{
	if (Id != RobotControlId())
	{
		return false;
	}
	const int32 Index = FMath::RoundToInt(Value);
	if (!Choices.IsValidIndex(Index))
	{
		return false;
	}
	Staged = Index; // staged only: sim.robot.load starts it
	return true;
}

bool URammsRobotSelectComponent::TriggerControl(FName Id)
{
	if (Id == LoadControlId())
	{
		return LoadStagedRobot();
	}
	if (Id == RobotControlId())
	{
		return StepStagedRobot(1); // an Enum's action steps it
	}
	return false;
}

bool URammsRobotSelectComponent::ReadControl(FName Id, float& OutValue) const
{
	if (Id == RobotControlId() && Choices.IsValidIndex(Staged))
	{
		OutValue = static_cast<float>(Staged);
		return true;
	}
	if (Id == ActiveControlId())
	{
		OutValue = 0.0f; // its severity: information
		return true;
	}
	return false;
}

bool URammsRobotSelectComponent::ReadControlText(FName Id, FText& OutText) const
{
	if (Id != ActiveControlId())
	{
		return false;
	}
	const URammsRobotSelectionSubsystem* Selection = GetWorld() ? GetWorld()->GetSubsystem<URammsRobotSelectionSubsystem>() : nullptr;
	OutText = FText::FromString(Selection ? Selection->GetActiveChoice().Describe() : FString(TEXT("not started through a robot game mode")));
	return true;
}
