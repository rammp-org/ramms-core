// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "InputCoreTypes.h"
#include "RammsKeyboardTeleopComponent.generated.h"

class URammsRobotBaseComponent;
class URammsDifferentialDriveController;
class URamms5BarLinkageController;

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

	/** Component names of the Ramms5BarLinkageControllers to move; empty = all
	 *  of them on the actor (both legs together). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop")
	TArray<FName> ControllerNames;
};

/**
 * Keyboard teleoperation for a robot built on the base-component architecture.
 * Put it on a pawn that owns a URammsRobotBaseComponent; when that pawn is
 * possessed by a player, each tick it polls the player's keys and:
 *
 *  - feeds the sibling URammsDifferentialDriveController with a joystick-style
 *    input (W/S forward-back, A/D turn) via SetDriveInput;
 *  - raises / lowers the endpoint of the Ramms5BarLinkageControllers (E/Q);
 *  - nudges arbitrary groups of position motors by Id (MotorBindings — e.g. the
 *    front and rear cranks of a lift_drive base).
 *
 * Keys are polled (APlayerController::IsInputKeyDown), so this needs no input
 * mapping assets and coexists with Enhanced Input; Pixel Streaming keyboard
 * input reaches it the same way. Everything is data: which keys, which motor
 * Ids, which rates.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsKeyboardTeleopComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsKeyboardTeleopComponent();

	virtual void BeginPlay() override;
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

	UPROPERTY(Transient)
	TObjectPtr<URammsRobotBaseComponent> Base;

	UPROPERTY(Transient)
	TObjectPtr<URammsDifferentialDriveController> Drive;

	UPROPERTY(Transient)
	TArray<TObjectPtr<URamms5BarLinkageController>> Linkages;

	/** Per-linkage endpoint target (x, z), seeded from the live endpoint. */
	TMap<URamms5BarLinkageController*, FVector2D> LinkageTargets;

	/** Per-motor target, seeded from the live value on first use. */
	TMap<FName, float> MotorTargets;

	FVector2D LastDrive = FVector2D::ZeroVector;
};
