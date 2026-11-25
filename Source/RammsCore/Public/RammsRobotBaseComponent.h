// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsMotorSpec.h"
#include "RammsRobotBaseComponent.generated.h"

class USkeletalMeshComponent;

class IRammsActuationBackend;
struct FRammsMotorEffort;

/**
 * Which physics engine simulates this robot's motors.
 *  - MuJoCo: the robot's URLab articulation (RammsMujocoSupport), on whichever
 *    engine simulates it.
 *  - Chaos: the same articulation, switched to Chaos at BeginPlay
 *    (RammsMujocoSupport's URammsChaosArticulationComponent). A robot with no
 *    articulation (a skeletal-mesh robot) falls back to the skeletal Chaos
 *    backend, which is deprecated.
 *  - Auto: MuJoCo when the robot has an articulation, else the skeletal Chaos
 *    backend.
 */
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

	/** Chaos only: stiffness of a wheel's parking brake (SetMotorBrake), the
	 *  drive that holds its joint where it stopped. Stiff enough that the
	 *  robot's weight on a slope barely turns it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot|Chaos", meta = (ClampMin = "0.0"))
	float ChaosParkingBrakeStiffness = 1.0e8f;

	/** Chaos only: damping of a wheel's parking brake. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robot|Chaos", meta = (ClampMin = "0.0"))
	float ChaosParkingBrakeDamping = 1.0e6f;

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

	/** The value last given to this motor through SetMotorCommand, in the
	 *  robot's sense and clamped to its ControlRange -- for a profiled
	 *  position servo the target, not the setpoint eased toward it. False when
	 *  nothing has commanded it this session, or a velocity command or a
	 *  release has since replaced that command. What a controller picking up
	 *  a mechanism someone else left reads, rather than the joint's measured
	 *  value: a loaded servo sits short of its command, and re-commanding
	 *  where it sags would let it sag further each time. */
	UFUNCTION(BlueprintPure, Category = "Robot|Motors")
	bool GetMotorCommand(FName MotorId, float& OutValue) const;

	// --- Command offset layer (plain C++) -------------------------------------
	//
	// A position motor's command can carry an OFFSET on top of what
	// SetMotorCommand gives it, its BASELINE: the motor is told
	// clamp(baseline + offset, ControlRange). Every writer there is keeps
	// writing baselines through SetMotorCommand and reading them back through
	// GetMotorCommand, unaware; one object per robot, the layer, writes the
	// offsets. On the base because it is the one place every motor writer
	// already passes through.
	//
	// A claimed motor is told its command once a frame, in this component's
	// tick: SetMotorCommand and SetMotorCommandOffset only store. Two writers
	// in a frame would otherwise tell it a bare baseline and then the sum, and
	// a servo eased at the physics rate would chase the first for a step. A
	// motor nobody has claimed is told at once, as ever.
	//
	// A baseline written LATER IN THE FRAME than this component ticks -- by a
	// controller whose tick is in a later tick group, which cannot be ordered
	// before this one -- is told at once, as baseline + offset: it is the
	// frame's last word on that motor, as it is with nothing claimed. Held
	// back for the next tick it would be told a frame late, and never at all
	// while another controller writes the same motor earlier in every frame
	// (a seat held where its rider has it, under a drive mode's stance ramp:
	// the ramp's value was what the tick found each frame, and the tilt went
	// with it while the seat's other motors stayed). Such a motor is then left
	// to that late write: the tick after one tells it nothing (what is waiting
	// goes out with the next late write, or at the following tick if none
	// comes), so it is still told once a frame.

	/**
	 * Make Layer the one writer of offsets on MotorIds. False, and nothing
	 * claimed, when another object holds the layer, or any of MotorIds is not
	 * a Position motor of the registry. The holder may call again to add
	 * motors. A claimed motor starts with no offset.
	 */
	bool ClaimCommandOffsetLayer(const UObject* Layer, TConstArrayView<FName> MotorIds);

	/**
	 * Give the layer up: every claimed motor is told its baseline now, if it
	 * was last told anything else or has a write waiting, and is commanded at
	 * once from then on. The layer is expected to have brought its offsets to
	 * zero first; one still on is dropped, a step, and reported.
	 */
	void ReleaseCommandOffsetLayer(const UObject* Layer);

	/**
	 * Set a claimed motor's offset (the robot's sense, the motor's units),
	 * told with its baseline at this component's next tick. False when Layer
	 * does not hold the layer, the motor is not claimed, or Offset is not a
	 * number. An offset on a motor nothing has commanded waits for its first
	 * baseline.
	 */
	bool SetMotorCommandOffset(FName MotorId, float Offset, const UObject* Layer);

	/**
	 * Move a claimed motor's baseline to NewBaseline (clamped to its
	 * ControlRange) and its offset to what keeps the motor told exactly what
	 * it was: nothing is commanded and nothing moves. How the layer folds its
	 * offset into the baseline for a controller about to take the motor over
	 * (NewBaseline = GetMotorEffectiveCommand), and takes it back out after.
	 * Both change here, together, so no reader sees one without the other.
	 * False when Layer does not hold the layer, the motor is not claimed, or
	 * nothing has commanded it.
	 */
	bool RebaseMotorCommand(FName MotorId, float NewBaseline, const UObject* Layer);

	/** What the motor is told: its baseline plus its offset, clamped to its
	 *  ControlRange (for a claimed motor, from its next tick on); the same as
	 *  GetMotorCommand for a motor with no offset layer. False when
	 *  GetMotorCommand is. */
	bool GetMotorEffectiveCommand(FName MotorId, float& OutValue) const;

	/** The offset last set on this motor; 0 for a motor not claimed. As set:
	 *  where the sum runs into the ControlRange, more than what the motor is
	 *  told past its baseline. Reflected so that a recorder can log what a
	 *  motor was told (GetMotorCommand keeps meaning the baseline). */
	UFUNCTION(BlueprintPure, Category = "Ramms|Robot")
	float GetMotorCommandOffset(FName MotorId) const;

	/** Retune the velocity loop and reset its integrators. Gains are the one
	 *  thing here that has to be found against a running robot, and poking the
	 *  property on a live component is not a supported thing to do. */
	UFUNCTION(BlueprintCallable, Category = "Robot|Velocity Loop")
	void SetDefaultVelocityGains(float Kp, float Ki, float MaxIntegralTorque);

	/** Worst |commanded - achieved| speed seen since the last reset (rad/s),
	 *  across every motor under velocity command. Tuning needs a number. */
	UFUNCTION(BlueprintPure, Category = "Robot|Velocity Loop")
	float GetPeakVelocityError() const { return PeakVelocityError; }

	UFUNCTION(BlueprintCallable, Category = "Robot|Velocity Loop")
	void ResetPeakVelocityError() { PeakVelocityError = 0.0f; }

	/** Stop actively driving a motor (a position servo lets go of its joint;
	 *  see IRammsActuationBackend::ReleaseMotor). False if the backend cannot
	 *  release it — it is then still driven at its last command. */
	UFUNCTION(BlueprintCallable, Category = "Robot|Motors")
	bool ReleaseMotor(FName MotorId);

	/** Engage or release a Torque motor's parking brake: its joint held where
	 *  it stands (IRammsActuationBackend::SetMotorBrake). False if the backend
	 *  has no brake for it. */
	UFUNCTION(BlueprintCallable, Category = "Robot|Motors")
	bool SetMotorBrake(FName MotorId, bool bEngaged);

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

	/** Whether the backend simulates this motor's joint as a joint -- its angle
	 *  reported by GetMotorValue and a Torque command applied in N.m
	 *  (IRammsActuationBackend::SimulatesMotorJoint); false without a backend.
	 *  A controller closing its own loop on a wheel's angle checks this first. */
	bool SimulatesMotorJoint(FName MotorId) const;

	/** Whether SetMotorBrake would engage a brake on this motor now
	 *  (IRammsActuationBackend::HasMotorBrake); false without a backend. */
	bool HasMotorBrake(FName MotorId) const;

	/** Hold a motor's output within [MinNm, MaxNm], in the robot's sense (its
	 *  registry Direction applied; IRammsActuationBackend::SetMotorTorqueLimit):
	 *  a side at +-the largest float keeps the actuator's own limit there,
	 *  MinNm >= MaxNm clears it. False when the backend cannot. */
	bool SetMotorTorqueLimit(FName MotorId, float MinNm, float MaxNm);

	/** Clear one motor's torque limit, or every one this base set (EndPlay
	 *  clears them all). */
	void ClearMotorTorqueLimit(FName MotorId);
	void ClearAllMotorTorqueLimits();

	/** What the motor's actuator put out over the last WindowSeconds of
	 *  simulated time, in the robot's sense (IRammsActuationBackend::
	 *  GetMotorEffort; the first call starts recording it). False without
	 *  a reading. */
	bool GetMotorEffort(FName MotorId, double WindowSeconds, FRammsMotorEffort& Out) const;

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

	/** Simulated seconds the physics backend has advanced, or negative when it
	 *  keeps no clock of its own (time then is the game's). See
	 *  IRammsActuationBackend::GetSimulationTime. */
	UFUNCTION(BlueprintPure, Category = "Robot")
	double GetSimulationTime() const;

	/** The simulated time of the poses GetMotorTransform gives, on
	 *  GetSimulationTime's clock, which may be earlier than the time that
	 *  gives now; negative with no such clock. See
	 *  IRammsActuationBackend::GetTransformTime. (Plain C++.) */
	double GetTransformTime() const;

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

	/** The motors SetMotorTorqueLimit limited (ClearAllMotorTorqueLimits). */
	TSet<FName> TorqueLimited;

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

	/** A position motor whose registry row profiles its commands
	 *  (FRammsMotorSpec::MaxVelocity / MaxAcceleration): the target it was last
	 *  given and the setpoint eased toward it, both in the robot's sense. The
	 *  setpoint is this component's only while the backend is not easing it
	 *  every physics step (SetPositionProfile). */
	struct FPositionProfile
	{
		double Target = 0.0;
		double Setpoint = 0.0;
		double Rate = 0.0;
		bool   bInBackend = false;
	};

	/** Position motors under a profiled command, by Id. */
	TMap<FName, FPositionProfile> PositionProfiles;

	/** Stop profiling a motor, here and in the backend. */
	void DropPositionProfile(FName MotorId);

	/** The value each motor was last given through SetMotorCommand (robot
	 *  sense, clamped), by Id; see GetMotorCommand. */
	TMap<FName, float> LastCommands;

	/** Tell the backend a value SetMotorCommand has clamped (the robot's
	 *  sense): through the motor's profile where its row has one. Spec is the
	 *  motor's registry row, null for a motor with none. */
	void PushCommand(FName MotorId, const FRammsMotorSpec* Spec, float Value);

	/** A motor claimed for the command offset layer. */
	struct FCommandOffset
	{
		float Offset = 0.0f;
		/** clamp(baseline + Offset), kept rather than summed again on every
		 *  read so that a rebase leaves it bit for bit as it was. Meaningful
		 *  while the motor has a baseline. */
		float Effective = 0.0f;
		/** Written since it was last told. */
		bool bDirty = false;
		/** Its baseline was written, and told at once, after this component
		 *  ticked (since its last tick). */
		bool bToldLate = false;
	};

	/** The claimed motors, by Id, in the order claimed (nothing is removed
	 *  but all at once). */
	TMap<FName, FCommandOffset> CommandOffsets;

	/** Who writes the offsets. Weak: a layer collected without releasing must
	 *  not leave its offsets on for good. */
	TWeakObjectPtr<const UObject> CommandOffsetLayer;

	/** Value within a motor's ControlRange, where its row sets one. */
	static float ClampToControlRange(const FRammsMotorSpec* Spec, float Value);

	/** Tell every claimed motor written since the last tick its command. */
	void FlushCommandOffsets();

	/** The world is ticking a group after the one this component ticks in:
	 *  whoever writes a motor now is this frame's last writer of it. */
	bool IsPastOwnTick() const;

	/** Forget the layer: each claimed motor told its baseline, where that is
	 *  not what it was last told. */
	void DropCommandOffsetLayer();

	/** Ease every profiled setpoint toward its target and command it. */
	void StepPositionProfiles(float DeltaTime);

	/** Gains in force for a motor: its own row's, else the base's defaults. */
	FRammsVelocityGains GainsFor(FName MotorId) const;

	/** Run one step of the velocity loop over every motor under command. */
	void StepVelocityDrives(float DeltaTime);

	/** Stop velocity-driving a motor, here and in the backend's own loop. */
	void DropVelocityDrive(FName MotorId);

	/** Worst tracking error seen since the last reset, for tuning. */
	float PeakVelocityError = 0.0f;
};
