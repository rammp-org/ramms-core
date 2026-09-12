// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsMotorSpec.h"
#include "RammsRobotBaseComponent.generated.h"

class IRammsActuationBackend;

/** Which physics engine simulates this robot's motors. Auto = MuJoCo when a
 *  backend is registered (RammsMujocoSupport) and resolves, else Chaos. */
UENUM(BlueprintType)
enum class ERammsPhysicsBackend : uint8
{
	Auto   UMETA(DisplayName = "Auto"),
	Chaos  UMETA(DisplayName = "Chaos"),
	Mujoco UMETA(DisplayName = "MuJoCo")
};

/**
 * The shared actuation substrate for a robot actor. One per robot; every
 * drive / linkage / holonomic controller consumes it instead of configuring
 * motors and a physics backend itself.
 *
 * It loads the robot's **motor registry** (a DataTable of FRammsMotorSpec) and
 * resolves the physics **backend** once, then acts as a pure router + query
 * surface addressed by motor Id: command a motor, read its value/velocity, ask
 * for its transform, or derived geometry like the separation between two drive
 * motors (skid-steer track width). It holds no notion of what motors are *for*
 * — a controller names the Ids it drives.
 *
 * A 5-bar linkage's mechanical geometry is deliberately NOT modelled here; that
 * controller takes its own kinematic data table and still commands its motors
 * through this component by Id.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsRobotBaseComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsRobotBaseComponent();
	// Out-of-line: Backend is a raw-owned incomplete IRammsActuationBackend here.
	virtual ~URammsRobotBaseComponent() override;

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** The robot's motor registry. Row struct: FRammsMotorSpec. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot")
	TObjectPtr<UDataTable> MotorTable;

	/** Which engine drives this robot's motors. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot")
	ERammsPhysicsBackend Backend = ERammsPhysicsBackend::Auto;

	// --- Motor command / read (by registry Id), routed to the backend --------

	/** Command a motor by Id (interpreted per its ERammsActuatorType). */
	UFUNCTION(BlueprintCallable, Category = "Robot|Motors")
	void SetMotorCommand(FName MotorId, float Value);

	/** Current scalar value of a motor's joint (position/angle), 0 if unknown. */
	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	float GetMotorValue(FName MotorId) const;

	/** Current joint velocity of a motor, 0 if unknown. */
	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	float GetMotorVelocity(FName MotorId) const;

	/** True if the registry contains a motor with this Id. */
	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	bool HasMotor(FName MotorId) const;

	/** The actuator type of a motor (defaults to Torque if unknown). */
	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	ERammsActuatorType GetMotorType(FName MotorId) const;

	/** Copy of a motor's registry spec; false if not found. */
	bool GetMotorSpec(FName MotorId, FRammsMotorSpec& OutSpec) const;

	// --- Geometry queries (derived from the backend's motor transforms) ------

	/** World transform of a motor's joint/body; false if unavailable. */
	UFUNCTION(BlueprintCallable, Category = "Robot|Geometry")
	bool GetMotorTransform(FName MotorId, FTransform& OutWorld) const;

	/** Distance between two motors (e.g. drive-motor separation → skid-steer
	 *  track width). Returns < 0 when either transform is unavailable. */
	UFUNCTION(BlueprintCallable, Category = "Robot|Geometry")
	float GetMotorSeparation(FName MotorIdA, FName MotorIdB) const;

	/** True once a physics backend has been resolved for this robot. */
	UFUNCTION(BlueprintPure, Category = "Robot")
	bool HasBackend() const { return Backend_ != nullptr; }

private:
	/** Resolved motor registry, Id -> spec (built from MotorTable at BeginPlay). */
	TMap<FName, FRammsMotorSpec> Motors;

	/** The resolved physics backend. Raw owning pointer (see the drive-backend
	 *  registry note); created in BeginPlay, freed in the destructor. */
	IRammsActuationBackend* Backend_ = nullptr;
};
