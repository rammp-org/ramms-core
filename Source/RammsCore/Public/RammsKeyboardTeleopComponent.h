// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "InputCoreTypes.h"
#include "RammsKeyboardTeleopComponent.generated.h"

class URammsRobotBaseComponent;
class URammsRobotControlSurfaceComponent;

/** A pair of keys that nudges the target of a group of position motors. */
USTRUCT(BlueprintType)
struct FRammsMotorKeyBinding
{
	GENERATED_BODY()

	/** Shown in logs / UI. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop")
	FString Label;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop")
	FKey IncreaseKey;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop")
	FKey DecreaseKey;

	/** Motor Ids (in the base component's registry) moved together. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop")
	TArray<FName> MotorIds;

	/** Target change per second while a key is held (rad/s for hinge position
	 *  motors, cm/s for slides). Clamped to each motor's ControlRange. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop", meta = (ClampMin = "0.0"))
	float RatePerSecond = 0.6f;
};

/** Keys that raise / lower the endpoint of the robot's 5-bar linkages. */
USTRUCT(BlueprintType)
struct FRammsLinkageKeyBinding
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop")
	FKey UpKey = EKeys::E;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop")
	FKey DownKey = EKeys::Q;

	/** Endpoint height change per second while held (cm/s, linkage-local z). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop", meta = (ClampMin = "0.0"))
	float RateCmPerSecond = 6.0f;

	/** Component names of the linkage controllers to move; empty = all
	 *  of them on the actor (both legs together). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop")
	TArray<FName> ControllerNames;
};

/**
 * Keyboard teleoperation for a robot built on the base-component architecture.
 * Put it on a pawn that owns a URammsRobotBaseComponent; when that pawn is
 * possessed by a player, each tick it polls the player's keys and:
 *
 *  - feeds the robot's drive controls (drive.forward / drive.turn) joystick-style
 *    input (W/S forward-back, A/D turn) via SetDriveInput;
 *  - raises / lowers every Position control in the Linkage group (E/Q);
 *  - nudges arbitrary groups of position motors by Id (MotorBindings — e.g. the
 *    front and rear cranks of a lift_drive base).
 *
 * Keys are polled (APlayerController::IsInputKeyDown), so this needs no input
 * mapping assets and coexists with Enhanced Input; Pixel Streaming keyboard
 * input reaches it the same way. Everything is data: which keys, which motor
 * Ids, which rates.
 *
 * LEGACY: superseded by URammsControlInputComponent + a URammsControlInputMap
 * (Enhanced Input -> the robot's control surface). Not added to new pawns;
 * kept for one release for existing Blueprints.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsKeyboardTeleopComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsKeyboardTeleopComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// --- Driving ---------------------------------------------------------------

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Drive")
	bool bDriveEnabled = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Drive")
	FKey ForwardKey = EKeys::W;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Drive")
	FKey BackwardKey = EKeys::S;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Drive")
	FKey TurnLeftKey = EKeys::A;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Drive")
	FKey TurnRightKey = EKeys::D;

	/** Scale on the turn axis (1 = full-rate spin). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Drive", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float TurnScale = 0.8f;

	/** Scale on the forward axis. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Drive", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float ForwardScale = 1.0f;

	// --- Linkages / motors -----------------------------------------------------

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Linkage")
	FRammsLinkageKeyBinding Linkage;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Motors")
	TArray<FRammsMotorKeyBinding> MotorBindings;

	/** Log each command change (verbose). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Debug")
	bool bLogCommands = false;

	/** True while a player controller is driving this pawn. */
	UFUNCTION(BlueprintPure, Category = "Teleop")
	bool IsPlayerControlled() const;

private:
	APlayerController* GetPlayerController() const;
	float			   Axis(APlayerController* PC, const FKey& Positive, const FKey& Negative) const;
	/** Zero the drive command this component issued (unpossessed, disabled, end of play). */
	void ReleaseDrive();

	UPROPERTY(Transient)
	TObjectPtr<URammsRobotBaseComponent> Base;

	/** Everything is driven through the robot's surface by control Id, so this
	 *  component names no controller class: a holonomic base, or anything else
	 *  advertising the same Ids, works here unchanged. */
	UPROPERTY(Transient)
	TObjectPtr<URammsRobotControlSurfaceComponent> Surface;

	/** Position controls in the Linkage group, discovered from the surface. */
	TArray<FName> LinkageControlIds;

	/** Commanded height per linkage control (cm). */
	TMap<FName, float> LinkageTargets;

	/** True when ControllerNames is empty or matches this control's component. */
	bool MatchesLinkageFilter(FName ControlId) const;

	/** Per-motor target, seeded from the live value on first use. */
	TMap<FName, float> MotorTargets;

	FVector2D LastDrive = FVector2D::ZeroVector;

	/** True while the drive controller holds a command this component issued. */
	bool bDriveCommandActive = false;
};
