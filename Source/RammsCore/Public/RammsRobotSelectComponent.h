// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsControlContributor.h"
#include "RammsRobotSelectComponent.generated.h"

class URammsRobotDefinition;

/**
 * The robot choice on a robot's control surface (ARammsRobotGameMode adds it
 * to the robot it spawns, and the picker pawn carries one):
 *
 *  - sim.robot (Enum): the robots there are to start. Setting it only stages
 *    the choice -- a gamepad steps an Enum one choice at a time, and every step
 *    reloading the level would be no use.
 *  - sim.robot.load (Action): open the level again with the staged robot
 *    (?Robot=<id>): the robot is part of the physics scene from the start.
 *  - sim.robot.active (Text): the robot running, and where it came from.
 *
 * A robot started from a pawn class path (no definition) is offered first, as
 * itself, and staged: loading with nothing changed starts that pawn again.
 *
 * Scripts choose by id (URammsRobotBridge, ramms.Robot), never by the Enum's
 * index: the list grows when a plugin with robots is mounted.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsRobotSelectComponent : public UActorComponent, public IRammsControlContributor
{
	GENERATED_BODY()

public:
	URammsRobotSelectComponent();

	static FName RobotControlId() { return FName(TEXT("sim.robot")); }
	static FName LoadControlId() { return FName(TEXT("sim.robot.load")); }
	static FName ActiveControlId() { return FName(TEXT("sim.robot.active")); }
	static FName GroupName() { return FName(TEXT("Simulation")); }

	/** Shown after sim.robot.active's status while no robot is running: how to
	 *  choose one (the picker pawn's keys). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot")
	FText NoRobotHint;

	/** The robot staged for the next load (an id or class path). */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Robots")
	FString GetStagedRobot() const;

	UFUNCTION(BlueprintCallable, Category = "Ramms|Robots")
	bool StageRobot(const FString& IdOrAlias);

	/** Stage the next robot (Delta 1) or the one before (-1), wrapping. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Robots")
	bool StepStagedRobot(int32 Delta);

	/** Open the level again with the staged robot. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Robots")
	bool LoadStagedRobot();

	// --- IRammsControlContributor --------------------------------------------
	virtual void  DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool  ApplyControl(FName Id, float Value) override;
	virtual bool  TriggerControl(FName Id) override;
	virtual bool  ReadControl(FName Id, float& OutValue) const override;
	virtual bool  ReadTarget(FName Id, float& OutTarget) const override { return ReadControl(Id, OutTarget); }
	virtual bool  ReadControlText(FName Id, FText& OutText) const override;
	virtual int32 GetControlOrder() const override { return 900; }

protected:
	virtual void BeginPlay() override;

private:
	/** The robots offered, in picker order (refreshed at BeginPlay). */
	void  Refresh();
	int32 IndexOf(const FString& IdOrAlias) const;
	bool  HasRobotRunning() const;

	/** The robots offered; null stands for RunningClassPath. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<URammsRobotDefinition>> Choices;

	/** The running robot's pawn class path, when it was started from one. */
	FString RunningClassPath;

	/** The choice staged for the next load: an index into Choices, or -1. */
	int32 Staged = -1;
};
