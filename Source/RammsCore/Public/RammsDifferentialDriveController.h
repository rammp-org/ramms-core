// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsDifferentialDriveTypes.h"
#include "RammsControlContributor.h"
#include "RammsDriveMode.h"
#include "Templates/PimplPtr.h"
#include "RammsDifferentialDriveController.generated.h"

class UPrimitiveComponent;
class URammsRobotBaseComponent;

namespace RammsDrive
{
	struct FCarry;
	struct FTorqueHold;
	struct FWheelAngle;
} // namespace RammsDrive

/**
 * Differential drive controller component for powered wheelchair simulation
 * Manages motor control, applies forces to wheels, tracks odometry
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsDifferentialDriveController : public UActorComponent, public IRammsControlContributor, public IRammsDriveMode
{
	GENERATED_BODY()

public:
	URammsDifferentialDriveController();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// ========== Drive Configuration ==========

	/** Control mode selection */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Control")
	EDriveControlMode ControlMode = EDriveControlMode::TorqueControl;

	/** Name of the skeletal mesh component containing the wheel bones (leave empty to auto-find first skeletal mesh) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Wheels")
	FName SkeletalMeshComponentName = NAME_None;

	/**
	 * Left drive-motor Id in the robot's motor registry, used when a
	 * URammsRobotBaseComponent is present on the actor — physics I/O then routes
	 * through it (Chaos or MuJoCo) instead of touching a wheel bone directly.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Wheels")
	FName LeftMotorId = FName("left_motor");

	/** Right drive-motor Id (see LeftMotorId). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Wheels")
	FName RightMotorId = FName("right_motor");

	/** With a base component: replace TrackWidth with the measured separation
	 *  of the two drive motors on the first tick it's available (the measured
	 *  value is logged either way, so it can be checked against TrackWidth). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Wheels")
	bool bUseMotorSeparationAsTrackWidth = false;

	/** Left wheel bone name — the fallback Chaos path when no base component is present. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Wheels")
	FName LeftWheelBoneName = FName("left_motor");

	/** Right wheel bone name — the fallback Chaos path when no base component is present. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Wheels")
	FName RightWheelBoneName = FName("right_motor");

	/** Wheel radius in cm */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Wheels", meta = (ClampMin = "1.0"))
	float WheelRadius = 18.0f;

	/** Distance between left and right wheels in cm */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Wheels", meta = (ClampMin = "10.0"))
	float TrackWidth = 60.0f;

	// ========== Motor Configuration ==========

	/** Motor parameters for left wheel */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Motor")
	FMotorParameters LeftMotorParams;

	/** Motor parameters for right wheel */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Motor")
	FMotorParameters RightMotorParams;

	// ========== Torque Control Settings ==========

	/** Torque multiplier for torque control mode */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Torque Control", meta = (ClampMin = "0.0"))
	float TorqueMultiplier = 1.0f;

	/** Resistive torque coefficient - opposes wheel motion (simulates back-EMF and friction) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Torque Control", meta = (ClampMin = "0.0"))
	float ResistiveTorqueCoefficient = 0.5f;

	/** Maximum turning speed (angular velocity) in degrees per second */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Torque Control", meta = (ClampMin = "0.0"))
	float MaxTurningSpeed = 90.0f;

	// ========== Velocity Control Settings ==========

	/** Maximum target velocity in cm/s */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Velocity Control", meta = (ClampMin = "0.0"))
	float MaxVelocity = 200.0f;

	/** PID Proportional gain */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Velocity Control|PID", meta = (ClampMin = "0.0"))
	float PID_Kp = 10.0f;

	/** PID Integral gain */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Velocity Control|PID", meta = (ClampMin = "0.0"))
	float PID_Ki = 0.5f;

	/** PID Derivative gain */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Velocity Control|PID", meta = (ClampMin = "0.0"))
	float PID_Kd = 1.0f;

	/** Maximum integral windup value */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Velocity Control|PID", meta = (ClampMin = "0.0"))
	float PID_IntegralMax = 100.0f;

	// ========== Braking ==========

	/** Enable automatic braking when input is at rest */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Braking")
	bool bEnableAutoBraking = false;

	/** Input magnitude threshold below which brakes engage */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Braking", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float BrakingThreshold = 0.01f;

	/** Brake torque to apply when braking (N*m) - acts as damping coefficient */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Braking", meta = (ClampMin = "0.0"))
	float BrakeTorque = 2.0f;

	/** Velocity threshold below which brakes engage (cm/s) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Braking", meta = (ClampMin = "0.0"))
	float BrakingVelocityThreshold = 5.0f;

	/**
	 * With the input at centre, bring the robot to rest -- both drive wheels
	 * braked together from its forward speed, at no more than 30 cm/s^2
	 * (twice that while a slope pulls them on; a turn under way stops) --
	 * and hold it there: each wheel is given the
	 * torque that keeps it at the angle it stopped at (a stiff loop on its
	 * angle, with an integral that takes a slope's pull; the two never more
	 * than half a centimetre of travel apart in what they bring back), so on
	 * a slope the robot stays where it stopped instead of rolling away. Like a brake rather than a drive, with a robot base the
	 * holding torque may reach the actuator's whole range (its registry
	 * ControlRange), not only the motor parameters' MaxTorque the input
	 * scales to. Moving the input off centre lets go, the holding torque
	 * fading out over half a second into the control mode's own rather than
	 * dropping at once (in TorqueControl the input is a torque: one smaller
	 * than a slope's pull lets the robot roll back once the hold has faded,
	 * as it would have without it). Nothing is held for the first second
	 * after the mode's first tick: the robot is being put down as it
	 * spawns, its legs jumping into the initial stance (a leg carrying
	 * its wheel fore/aft), and a wheel held meanwhile would drag the chassis
	 * instead of rolling with its leg. While on, this takes the place of auto-braking
	 * (bEnableAutoBraking). Off, the wheels are left to the control mode with
	 * the input at centre, as before.
	 *
	 * The hold is a torque loop on each wheel's angle with gains from
	 * VehicleMass and WheelRadius, so it runs only where the robot base's
	 * backend simulates both drive wheels' joints -- reports their angles and
	 * applies a torque in N.m (URammsRobotBaseComponent::SimulatesMotorJoint:
	 * MuJoCo). On the Chaos backend, whose wheel torque is an acceleration
	 * change and whose wheels report no angle, and without a robot base, this
	 * has no effect: auto-braking applies as before, and HoldsRobotStill() is
	 * false.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Braking")
	bool bHoldWhenIdle = true;

	// ========== Slip/Traction ==========

	/** Enable slip/traction modeling */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction")
	bool bEnableSlipModeling = false;

	/** Base traction coefficient (multiplied by surface friction) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction", meta = (ClampMin = "0.1", ClampMax = "2.0"))
	float TractionCoefficient = 1.0f;

	/** Use physical material friction from ground surface */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction")
	bool bUsePhysicalMaterialFriction = true;

	/** Enable load-dependent traction (wheels with more weight get more grip) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction")
	bool bEnableLoadDependentTraction = true;

	/** Mass of the vehicle (kg) - used for load calculations */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction", meta = (ClampMin = "1.0"))
	float VehicleMass = 100.0f;

	/** Slip ratio where peak traction occurs (typically 0.1-0.15) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction", meta = (ClampMin = "0.01", ClampMax = "0.3"))
	float PeakSlipRatio = 0.15f;

	/** Enable lateral slip resistance (reduces torque when sliding sideways) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction")
	bool bEnableLateralSlipResistance = true;

	/** Lateral velocity threshold before slip resistance kicks in (cm/s) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Traction", meta = (ClampMin = "0.0"))
	float LateralSlipThreshold = 10.0f;

	// ========== Input ==========

	/** Current drive input (2D joystick style) */
	UPROPERTY(BlueprintReadOnly, Category = "Drive|Input")
	FVector2D DriveInput = FVector2D::ZeroVector;

	/** Input dead zone */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Input", meta = (ClampMin = "0.0", ClampMax = "0.5"))
	float InputDeadZone = 0.05f;

	/**
	 * How long an external drive input (SetExternalDriveInput — access devices,
	 * autonomy) holds priority over the player path. While an external input is
	 * fresher than this window, plain SetDriveInput calls (the pawn's per-tick
	 * joystick write) are ignored — the arbitration a physical assistive
	 * wheelchair performs between its local joystick and alternative inputs.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Input", meta = (ClampMin = "0.05"))
	float ExternalInputHoldSeconds = 0.3f;

	// ========== State ==========

	/** Left wheel state */
	UPROPERTY(BlueprintReadOnly, Category = "Drive|State")
	FRammsWheelState LeftWheelState;

	/** Right wheel state */
	UPROPERTY(BlueprintReadOnly, Category = "Drive|State")
	FRammsWheelState RightWheelState;

	/** Current odometry data */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drive|State")
	FOdometryData Odometry;

	/** Is braking currently engaged */
	UPROPERTY(BlueprintReadOnly, Category = "Drive|State")
	bool bIsBraking = false;

	/** Velocity threshold for odometry updates (cm/s) - movements below this are ignored to prevent drift */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Odometry", meta = (ClampMin = "0.0"))
	float OdometryVelocityThreshold = 0.5f;

	// ========== Blueprint API ==========

	/**
	 * Set drive input from 2D joystick (the player path — the pawn writes this
	 * every tick from Enhanced Input). Ignored while an external input session
	 * is active (see SetExternalDriveInput).
	 * @param Input - Joystick input (X = turn, Y = forward/back), -1 to 1
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	void SetDriveInput(FVector2D Input);

	/**
	 * Set drive input from an external source (access devices via RammsAccess,
	 * scripted control, autonomy). Takes priority over SetDriveInput for
	 * ExternalInputHoldSeconds after each call; when the external stream stops,
	 * the hold expires and the player path resumes automatically.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	void SetExternalDriveInput(FVector2D Input);

	/** True while an external input session holds drive priority. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Differential Drive")
	bool IsExternalDriveActive() const;

	/**
	 * Get current odometry data
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Differential Drive")
	FOdometryData GetOdometry() const { return Odometry; }

	/**
	 * Reset odometry to given position and orientation
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	void ResetOdometry(FVector Position = FVector::ZeroVector, FRotator Orientation = FRotator::ZeroRotator);

	/**
	 * Get left wheel state
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Differential Drive")
	FRammsWheelState GetLeftWheelState() const { return LeftWheelState; }

	/**
	 * Get right wheel state
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|Differential Drive")
	FRammsWheelState GetRightWheelState() const { return RightWheelState; }

	/**
	 * Set control mode
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	void SetControlMode(EDriveControlMode Mode) { ControlMode = Mode; }

	/**
	 * Enable or disable slip modeling
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Differential Drive")
	void SetSlipModelingEnabled(bool bEnabled) { bEnableSlipModeling = bEnabled; }

	// ========== Debug ==========

	/** Enable debug logging */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Debug")
	bool bEnableDebugLogging = false;

	/** Enable on-screen debug display */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive|Debug")
	bool bEnableDebugDisplay = false;

private:
	// Cached skeletal mesh component reference
	UPROPERTY(Transient)
	USkeletalMeshComponent* SkeletalMeshComponent = nullptr;

	/** Sibling robot base component, if this actor has one. When set, wheel
	 *  physics I/O (velocity read, torque command, braking) routes through it
	 *  by motor Id instead of the direct Chaos wheel-bone path. */
	UPROPERTY(Transient)
	URammsRobotBaseComponent* BaseComponent = nullptr;

	/** True when wheel I/O should route through the base component: one is
	 *  present AND it resolved a physics backend. Otherwise the direct Chaos
	 *  wheel-bone path is used (so a misconfigured backend degrades to the
	 *  legacy behaviour rather than a silent no-op). */
	bool UsesBase() const;

	/** Set once the drive-motor separation has been read from the base. */
	bool bTrackWidthMeasured = false;

	/** Wall-clock time of the last SetExternalDriveInput call (-1 = never). */
	double LastExternalInputSeconds = -1.0;

	/** Shared input application (store + debug log + anti-windup reset) for the
	 *  player and external input paths — keeps both sources consistent. */
	void ApplyDriveInputInternal(FVector2D Input, const TCHAR* SourceLabel);

	// PID state for velocity control
	float LeftIntegralError = 0.0f;
	float LeftPreviousError = 0.0f;
	float RightIntegralError = 0.0f;
	float RightPreviousError = 0.0f;

	// Previous wheel rotations for odometry
	float PreviousLeftRotation = 0.0f;
	float PreviousRightRotation = 0.0f;

	/** Get body instance for a specific bone */
	FBodyInstance* GetBoneBodyInstance(FName BoneName);

	/** Update wheel state from physics (base component by MotorId when present,
	 *  else the Chaos wheel bone). */
	void UpdateWheelState(FName MotorId, FName BoneName, FRammsWheelState& OutState); /** The wheel's Chaos body: through the base's Chaos mesh / ChaosName when a
																					   *  base drives this robot, else the legacy mesh / bone name. */
	FBodyInstance* GetWheelBody(FName MotorId, FName BoneName);

	/** One-time notice that slip modeling has no Chaos contact body on this backend. */
	bool bWarnedNoContactBody = false;

	/** Update controller in torque control mode */
	void UpdateTorqueControl(float DeltaTime);

	/** Update controller in velocity control mode */
	void UpdateVelocityControl(float DeltaTime);

	/** Calculate PID control output */
	float CalculatePID(float Error, float& IntegralError, float& PreviousError, float DeltaTime);

	/** Apply torque to wheel with motor and slip modeling (commanded through the
	 *  base component by MotorId when present, else the Chaos wheel bone). */
	void ApplyWheelTorque(FName MotorId, FName BoneName, float RequestedTorque, const FMotorParameters& MotorParams, FRammsWheelState& WheelState);

	/** Update odometry from wheel movements */
	void UpdateOdometry(float DeltaTime);

	/** Check if braking should be applied */
	bool ShouldApplyBrakes() const;

	/** Apply brake torque to both wheels */
	void ApplyBrakes();

	/** Debug log current state */
	void DebugLogState();

	// --- IRammsControlContributor: "drive.forward" / "drive.turn" -------------
	virtual void  DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool  ApplyControl(FName Id, float Value) override;
	virtual bool  ReleaseControl(FName Id) override;
	virtual bool  ReadControl(FName Id, float& OutValue) const override;
	virtual bool  ReadTarget(FName Id, float& OutTarget) const override { return ReadControl(Id, OutTarget); }
	virtual void  GetClaimedMotorIds(TArray<FName>& OutIds) const override;
	virtual int32 GetControlOrder() const override { return 0; }

public:
	// --- IRammsDriveMode ------------------------------------------------------

	/** The stance this mode needs: centre wheels down and carrying load. They
	 *  are the treaded tyres, and they have to overcome the omni wheels'
	 *  friction to steer the base, which they only do with weight on them. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive")
	FRammsDriveStance PlantedStance;

	virtual FName GetDriveModeId() const override { return FName("differential"); }
	virtual FText GetDriveModeDisplayName() const override
	{
		return NSLOCTEXT("Ramms", "DriveModeDifferential", "Differential");
	}
	virtual bool IsDriveModeActive() const override { return bDriveModeActive; }
	virtual void SetDriveModeActive(bool bActive) override;
	virtual bool GetRequiredStance(FRammsDriveStance& OutStance) const override
	{
		OutStance = PlantedStance;
		return !OutStance.IsEmpty();
	}
	/** Its forward speed and its yaw rate up to MaxTurningSpeed: a
	 *  differential drive cannot move sideways. */
	virtual FRammsDriveMotion GetCarriableMotion(const FRammsDriveMotion& Motion) const override;
	/**
	 * For HoldSeconds while its operator holds the stick where it was when
	 * the mode took over, adds to its control mode's torque a pull towards
	 * the wheel speeds Motion needs (half the motor's stall torque per rad/s
	 * of error), then fades that out over half a second, leaving the
	 * operator's command -- which in torque control is a torque, an
	 * acceleration from the speed the robot has, not a speed. Moving the stick
	 * by more than a tenth of its travel either way ends the hold (into the
	 * fade); letting it go to centre ends the carry at once, the mode braking
	 * as it would. An input not yet heard on the first tick after the switch
	 * is given that tick (no auto-brake on it): its operator may write it on
	 * its own tick.
	 */
	virtual void SeedMotion(const FRammsDriveMotion& Motion, float HoldSeconds) override;
	/** Drives on as it is (it has nothing to take off: no mode it hands over
	 *  to carries less than forward and turning) until the selector hands
	 *  over; bounded by a mode that carries more, there is nothing to keep
	 *  within either. */
	virtual bool BeginHandOver(const IRammsDriveMode& Other) override { return bDriveModeActive; }
	/** While held, drives nothing: no torque, no brake (its wheels are still
	 *  going down); they are held once down (bHoldWhenIdle). */
	virtual void HoldForStance(bool bHold) override { bHeldForStance = bHold && bDriveModeActive; }
	/** bHoldWhenIdle, where the backend lets it hold (CanHoldWithTorque). */
	virtual bool HoldsRobotStill() const override;
	/** Stood down at rest with bHoldWhenIdle, its wheels held where they are
	 *  until released (the holding torque, as with no input) -- carrying on
	 *  with what its own hold held the moment it was stood down, rather than
	 *  finding a slope's pull again as the robot sags. */
	virtual void HoldWheelsWhileInactive(bool bHold) override;
	/** The hold it begins within SeedValidSeconds starts with each drive
	 *  wheel's share of the slope's pull (RammsDrive::GroundHoldTorque, from
	 *  RobotMassKg if given, else VehicleMass), and eases its braking to rest
	 *  rather than stopping short (no roll-back from the loop after the stop). */
	virtual void SeedGroundSlope(float ForwardGradeDeg, float RobotMassKg) override;

private:
	/** Inactive modes contribute nothing: no controls, no claims, no commands. */
	bool bDriveModeActive = true;
	/** HoldForStance: its wheels are still going down. */
	bool bHeldForStance = false;

	/** A motion handed over by the selector, pulled towards (SeedMotion). */
	TPimplPtr<RammsDrive::FCarry> Carry;
	/** The first tick of a carry: its operator may not have been heard yet. */
	bool bFirstCarryTick = false;
	/** The input the operator held when the carry began (once heard), and
	 *  whether it has been. */
	FVector2D CarryInput = FVector2D::ZeroVector;
	bool	  bCarryInputSet = false;
	/** How far (normalised, either axis) the input may move from CarryInput
	 *  before the hold ends. */
	static constexpr double CarryInputTolerance = 0.1;
	/** The wheel rates (rad/s) the carried motion needs, left and right. */
	FVector2D CarryWheelRates = FVector2D::ZeroVector;
	/** How much of the carried motion's pull this tick applies (1 holding,
	 *  falling to 0 as it fades). */
	float CarryWeight = 0.0f;
	/** How long the pull takes to fade after the hold (s). */
	static constexpr double CarryFadeSeconds = 0.5;
	/** Add the carried motion's pull to the control mode's wheel torques. */
	void AddCarryTorque(float& LeftTorque, float& RightTorque) const;
	/** The clock (simulated where there is one) of the last tick. */
	double LastTickTime = -1.0;
	/** The physics clock, or the world's when there is none. */
	double Now() const;
	/** True while the operator's input is off centre (past InputDeadZone). */
	bool IsOperatorCommanding() const;
	/** Stop holding a carried motion and leave the wheels to the control mode. */
	void EndCarry();

	/** bHoldWhenIdle: each drive wheel held still (left, right), and the
	 *  angle each is at (rad: the backend's reading while it follows the
	 *  wheel's rate, else the rate integrated -- RammsDrive::FWheelAngle). */
	TPimplPtr<RammsDrive::FTorqueHold> HoldLeft;
	TPimplPtr<RammsDrive::FTorqueHold> HoldRight;
	TPimplPtr<RammsDrive::FWheelAngle> LeftAngleSource;
	TPimplPtr<RammsDrive::FWheelAngle> RightAngleSource;
	double							   LeftAngle = 0.0;
	double							   RightAngle = 0.0;
	/** The hold runs only on a robot base whose backend simulates both drive
	 *  wheels' joints (their angles, a torque in N.m): its gains are physical. */
	bool CanHoldWithTorque() const;
	/** A slope's pull handed over (SeedGroundSlope): each wheel's holding
	 *  torque (N.m), for a hold begun before SeedExpires (the clock). */
	double					SeedTorque = 0.0;
	double					SeedExpires = -1.0;
	static constexpr double SeedValidSeconds = 3.0;
	/** What the hold held the moment the mode was stood down (N.m, left,
	 *  right), for HoldWheelsWhileInactive to carry on with; valid while
	 *  bStoodDownHeld. */
	FVector2D StoodDownIntegral = FVector2D::ZeroVector;
	bool	  bStoodDownHeld = false;
	/** Start the hold on both wheels: braking from Forward (rad/s, each),
	 *  holding InitialLeft / InitialRight (N.m) from the start, the braking
	 *  eased to rest with bEase (RammsDrive::FTorqueHold::bEaseToRest). */
	void BeginHold(double ForwardLeft, double ForwardRight, double InitialLeft, double InitialRight, bool bEase = false);
	/** The simulation was reset (the clock went back): every hold ends. */
	void ResetHoldState();
	/** Each drive wheel's rate (rad/s) smoothed over HoldRateSmoothSeconds: a
	 *  hold begun on a moving robot brakes both wheels from their mean (the
	 *  robot's forward rate) -- one tick's rate reads up to a rad/s off on a
	 *  loaded wheel, and two sides braked each from its own would leave the
	 *  robot to turn as each wheel is brought back to where its own braking
	 *  ended; a turn under way stops at once. A hold begun at rest (stood
	 *  down through a switch) brakes each from its own. */
	double					SmoothLeftRate = 0.0;
	double					SmoothRightRate = 0.0;
	static constexpr double HoldRateSmoothSeconds = 0.15;
	/** The most (cm, of tread travel) the two held wheels' errors may differ
	 *  by: beyond it their targets are moved together (RammsDrive::BoundTurnError),
	 *  so the hold never turns the robot back by more than this, whatever the
	 *  wheels did meanwhile (slipped while braking, dragged by a leg). */
	static constexpr double HoldTurnErrorCm = 0.5;
	/** The holding torque at the moment the operator took over (N.m, left,
	 *  right: the two sides' mean, what held the robot along its way),
	 *  fading out over HoldReleaseSeconds from ReleaseElapsed 0. */
	FVector2D ReleaseTorque = FVector2D::ZeroVector;
	double	  ReleaseElapsed = -1.0;
	/** Stood down, its wheels held (HoldWheelsWhileInactive). */
	bool bHoldInactive = false;
	/** The clock at its first tick: nothing is held for
	 *  RammsDrive::SpawnSettleSeconds after it (the robot being put down). */
	double					FirstTickTime = -1.0;
	static constexpr double HoldReleaseSeconds = 0.5;
	static constexpr double HoldDecelerationCmS2 = 30.0;
	/** One tick of holding still: the torque to each wheel. */
	void StepHold(double DeltaSeconds);
	/** Let go of a hold, its torque fading out into the control mode's. */
	void ReleaseHold();
	/** Add what is left of a released hold's torque. */
	void AddReleaseTorque(float& LeftTorque, float& RightTorque) const;

public:
private:
	/** The two drive axes as last set through the control surface. */
	FVector2D ControlDriveInput = FVector2D::ZeroVector;
};
