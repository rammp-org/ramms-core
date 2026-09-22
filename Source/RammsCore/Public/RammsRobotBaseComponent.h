// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsMotorSpec.h"
#include "RammsRobotBaseComponent.generated.h"

class USkeletalMeshComponent;

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
 * The backend resolves at BeginPlay, and also lazily on first use, so consumer
 * components may call in from their own BeginPlay regardless of component
 * ordering. Motor Ids are the engine names (MuJoCo actuator name; Chaos bone via
 * ChaosName) — an Id absent from the registry still routes, with a one-time
 * warning, using the Id itself.
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
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	/** The robot's motor registry. Row struct: FRammsMotorSpec. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot")
	TObjectPtr<UDataTable> MotorTable;

	/** Which engine drives this robot's motors. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot")
	ERammsPhysicsBackend Backend = ERammsPhysicsBackend::Auto;

	/** Chaos only: the skeletal mesh component holding the motor bones. Leave
	 *  empty to use the owner's first skeletal mesh. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot|Chaos")
	FName ChaosSkeletalMeshComponentName = NAME_None;

	/** Chaos only: constraint drive stiffness (spring) for Position motors
	 *  mapped to physics-asset constraints. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot|Chaos", meta = (ClampMin = "0.0"))
	float ChaosPositionDriveStiffness = 100000.0f;

	/** Chaos only: constraint drive damping for Position motors. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot|Chaos", meta = (ClampMin = "0.0"))
	float ChaosPositionDriveDamping = 10000.0f;

	/** Chaos only: constraint drive force/torque limit for Position motors
	 *  (0 = unlimited). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot|Chaos", meta = (ClampMin = "0.0"))
	float ChaosPositionDriveForceLimit = 0.0f;

	/** Gains for the velocity loop over Torque actuators, for motors whose own
	 *  registry row does not set them. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot|Velocity Loop")
	FRammsVelocityGains DefaultVelocityGains;

	// --- Motor command / read (by registry Id), routed to the backend --------

	/** Command a motor by Id in the robot's sense (interpreted per its
	 *  ERammsActuatorType; clamped to its ControlRange; Direction applied). */
	UFUNCTION(BlueprintCallable, Category = "Robot|Motors")
	void SetMotorCommand(FName MotorId, float Value);

	/**
	 * Drive a motor at a speed (rad/s in the robot's sense), whatever kind of
	 * actuator it turns out to be.
	 *
	 * This is how a drive controller should command a wheel. A controller
	 * solves a body twist into wheel rates; it has no business knowing whether
	 * the backend gave that wheel a velocity servo or a torque motor. A
	 * Velocity actuator gets the rate passed straight through. A Torque one
	 * gets a loop closed over GetMotorVelocity each tick, using this motor's
	 * VelocityGains or the base's defaults.
	 *
	 * Writing a rate into SetMotorCommand instead applies it as newton-metres,
	 * which looks like a controller that almost works: the wheels turn, slowly,
	 * and the robot does not go anywhere.
	 *
	 * The target is held until changed, cleared, or overridden -- a direct
	 * SetMotorCommand or ReleaseMotor on the same motor drops it, so the loop
	 * never fights something else driving the same actuator.
	 */
	UFUNCTION(BlueprintCallable, Category = "Robot|Motors")
	void SetMotorVelocityCommand(FName MotorId, float RadiansPerSecond);

	/** Stop velocity-driving a motor. Leaves whatever the loop last wrote; use
	 *  ReleaseMotor or command zero to stop the motor itself. */
	UFUNCTION(BlueprintCallable, Category = "Robot|Motors")
	void ClearMotorVelocityCommand(FName MotorId);

	/** The speed this motor is being driven at, if it is. */
	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	bool GetMotorVelocityCommand(FName MotorId, float& OutRadiansPerSecond) const;

	/** Stop actively driving a motor (a position servo lets go of its joint;
	 *  see IRammsActuationBackend::ReleaseMotor). False if the backend cannot
	 *  release it — it is then still driven at its last command. */
	UFUNCTION(BlueprintCallable, Category = "Robot|Motors")
	bool ReleaseMotor(FName MotorId);

	/** Chaos only: the skeletal mesh component the Chaos backend drives
	 *  (ChaosSkeletalMeshComponentName, else the owner's first). */
	UFUNCTION(BlueprintPure, Category = "Robot|Chaos")
	USkeletalMeshComponent* GetChaosSkeletalMesh() const;

	/** The Chaos-side name a motor maps to (its ChaosName, else its Id). */
	UFUNCTION(BlueprintPure, Category = "Robot|Chaos")
	FName GetChaosName(FName MotorId) const;

	/** Current scalar value of a motor's joint (position/angle) in the robot's
	 *  sense, 0 if unknown. */
	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	float GetMotorValue(FName MotorId) const;

	/** Current joint velocity of a motor in the robot's sense, 0 if unknown. */
	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	float GetMotorVelocity(FName MotorId) const;

	/** True if the registry contains a motor with this Id. */
	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	bool HasMotor(FName MotorId) const;

	/** The actuator type of a motor (defaults to Torque if unknown). */
	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	ERammsActuatorType GetMotorType(FName MotorId) const;

	/** Copy of a motor's registry spec; false if not found. */
	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	bool GetMotorSpec(FName MotorId, FRammsMotorSpec& OutSpec) const;

	// --- Registry enumeration (what a UI / remote client needs to discover the robot)

	/** Every registered motor Id, in table order. */
	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	TArray<FName> GetMotorIds() const;

	/** Every registered motor spec, in table order. */
	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	TArray<FRammsMotorSpec> GetMotorSpecs() const;

	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	int32 GetMotorCount() const;

	DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnMotorRegistryLoaded, URammsRobotBaseComponent*, Base);

	/** Fired once the motor table has been loaded (lazily, on first access or
	 *  BeginPlay) — consumers that describe the robot can build from it. */
	UPROPERTY(BlueprintAssignable, Category = "Robot|Motors")
	FOnMotorRegistryLoaded OnMotorRegistryLoaded;

	/** The Id of the registry motor whose ChaosName (or, failing that, Id) is
	 *  ChaosName — lets a Chaos-era consumer that knows a bone / constraint
	 *  find its motor. NAME_None if there is none. */
	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	FName FindMotorIdByChaosName(FName ChaosName) const;

	// --- Geometry queries (derived from the backend's motor transforms) ------

	/** World transform of a motor's joint/body; false if unavailable. */
	UFUNCTION(BlueprintCallable, Category = "Robot|Geometry")
	bool GetMotorTransform(FName MotorId, FTransform& OutWorld) const;

	/** Straight-line (3D) distance between two motors. Returns < 0 when either
	 *  transform is unavailable. For a track width, project the two transforms
	 *  onto the robot's lateral axis instead (see the differential-drive
	 *  controller) — staggered or unequal-height motors make this larger. */
	UFUNCTION(BlueprintCallable, Category = "Robot|Geometry")
	float GetMotorSeparation(FName MotorIdA, FName MotorIdB) const;

	/** True once a physics backend has been resolved for this robot (resolves
	 *  lazily if not yet attempted). */
	UFUNCTION(BlueprintPure, Category = "Robot")
	bool HasBackend() const;

private:
	/** Load MotorTable into Motors once (idempotent; safe from any accessor, so
	 *  callers that arrive before BeginPlay still see the configured specs). */
	void LoadMotorRegistry() const;

	/** Resolve the backend once (idempotent; safe to call from any accessor).
	 *  Loads the registry first, so a backend never initializes against an
	 *  empty one. */
	void EnsureBackend() const;

	/** The registry Direction for a motor (+1 if unregistered). */
	float DirectionOf(FName MotorId) const;

	/** Resolved motor registry, Id -> spec. Mutable: loaded lazily. */
	mutable TMap<FName, FRammsMotorSpec> Motors;
	/** Ids in table order (TMap order is not a contract). */
	mutable TArray<FName> MotorOrder;
	mutable bool		  bMotorsLoaded = false;

	/** The resolved physics backend. Raw owning pointer (see the drive-backend
	 *  registry note); created by EnsureBackend, freed in EndPlay/destructor.
	 *  Mutable so const accessors can resolve lazily. */
	mutable IRammsActuationBackend* Backend_ = nullptr;
	mutable bool					bBackendResolved = false;

	/** Ids commanded without a registry row (warned once each). */
	mutable TSet<FName> WarnedUnregistered;

	/** One motor being held at a speed by the loop below. */
	struct FVelocityDrive
	{
		float Target = 0.0f;   // rad/s, robot sense
		float Integral = 0.0f; // N.m accumulated by the Ki term
	};

	/** Motors under velocity command, by Id. */
	TMap<FName, FVelocityDrive> VelocityDrives;

	/** Velocity-commanded motors that are position servos (warned once each). */
	TSet<FName> WarnedVelocityOnPosition;

	/** Gains in force for a motor: its own row's, else the base's defaults. */
	FRammsVelocityGains GainsFor(FName MotorId) const;

	/** Run one step of the velocity loop over every motor under command. */
	void StepVelocityDrives(float DeltaTime);
};
