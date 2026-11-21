// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsJointMotionMonitorComponent.generated.h"

class URammsSkeletalPoseComponent;

/** Whether the arm sits in its stowed pose or anywhere else. */
UENUM(BlueprintType)
enum class ERammsArmPoseState : uint8
{
	Stowed UMETA(DisplayName = "Stowed"),
	Active UMETA(DisplayName = "Active")
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnRammsArmPoseStateChanged, ERammsArmPoseState, NewState);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnRammsWheelsMovingChanged, bool, bMoving);

/**
 * Watches the joint targets written to the owner's URammsSkeletalPoseComponent
 * (e.g. by the RCPS "setJoints" command) and classifies them for the UI:
 *
 *  - Arm: Stowed while every arm joint target is within StowedTolerance degrees
 *    of StowedPose; Active otherwise.
 *  - Wheels: moving while a wheel joint target changes between commands;
 *    stopped once no wheel has changed for WheelIdleTimeout seconds.
 *
 * Bind OnArmPoseStateChanged / OnWheelsMovingChanged, or read ArmPoseState /
 * bWheelsMoving directly.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsJointMotionMonitorComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsJointMotionMonitorComponent();

	// ── Arm ─────────────────────────────────────────────────────────

	/** Arm joints, in the same order as StowedPose. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint Motion|Arm")
	TArray<FName> ArmJointNames;

	/** Arm joint targets (degrees) that count as stowed. Defaults to Kinova_Static_Poses "Retracted". */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint Motion|Arm")
	TArray<float> StowedPose;

	/** How far (degrees) each arm joint may be from StowedPose and still count as stowed. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint Motion|Arm", meta = (ClampMin = "0.0"))
	float StowedTolerance = 5.0f;

	// ── Wheels ──────────────────────────────────────────────────────

	/** Wheel joints whose target angle changing between commands means the wheels are moving. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint Motion|Wheels")
	TArray<FName> WheelJointNames;

	/** Smallest change (degrees) in a wheel target between commands that counts as moving. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint Motion|Wheels", meta = (ClampMin = "0.0"))
	float WheelMotionThreshold = 0.5f;

	/** Seconds without a wheel change before the wheels count as stopped. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint Motion|Wheels", meta = (ClampMin = "0.0"))
	float WheelIdleTimeout = 0.5f;

	// ── State ───────────────────────────────────────────────────────

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Joint Motion|State")
	ERammsArmPoseState ArmPoseState = ERammsArmPoseState::Stowed;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Joint Motion|State")
	bool bWheelsMoving = false;

	UFUNCTION(BlueprintPure, Category = "Ramms|Joint Motion")
	bool IsArmStowed() const { return ArmPoseState == ERammsArmPoseState::Stowed; }

	UFUNCTION(BlueprintPure, Category = "Ramms|Joint Motion")
	bool AreWheelsMoving() const { return bWheelsMoving; }

	UPROPERTY(BlueprintAssignable, Category = "Ramms|Joint Motion")
	FOnRammsArmPoseStateChanged OnArmPoseStateChanged;

	UPROPERTY(BlueprintAssignable, Category = "Ramms|Joint Motion")
	FOnRammsWheelsMovingChanged OnWheelsMovingChanged;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint Motion|Debug")
	bool bDebugLog = false;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

private:
	UFUNCTION()
	void HandleJointTargetsSet();

	void ResolveJointIndices();
	void SetArmPoseState(ERammsArmPoseState NewState);
	void SetWheelsMoving(bool bMoving);

	UPROPERTY()
	TObjectPtr<URammsSkeletalPoseComponent> PoseComponent;

	TArray<int32> ArmIndices;
	TArray<int32> WheelIndices;

	/** Wheel targets from the previous command (parallel to WheelIndices). */
	TArray<float> LastWheelTargets;
	bool		  bHaveWheelBaseline = false;

	/** World time of the last wheel change. */
	double LastWheelMotionTime = 0.0;
};
