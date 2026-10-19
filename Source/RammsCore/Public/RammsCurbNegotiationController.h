// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "RammsControlContributor.h"
#include "RammsCurbNegotiationTypes.h"
#include "RammsDriveMode.h"
#include "Templates/PimplPtr.h"

#include "RammsCurbNegotiationController.generated.h"

class URammsRobotBaseComponent;
class URamms5BarLinkageController;

namespace RammsCurb
{
	struct FRunTelemetry;
	struct FDriveState;
	struct FLevelState;
	struct FTerrainFitInput;
	struct FTerrainWheel;
	struct FFarDepthNeed;
	struct FRobotModel;
} // namespace RammsCurb

/**
 * Climbs or descends a curb by coordinating every leg and wheel on a
 * three-axle lift-drive: the corner wheels on crank-driven 4-bars, the centre
 * wheels on 5-bars, and the drive motors on all six.
 *
 * The gait. Two axles carry the robot while the third crosses the step, one
 * axle at a time, front to rear. For each axle:
 *
 *   Prepare  every wheel down; the chassis rises (or sinks) to the height that
 *            lets this axle clear the step while the other two still reach
 *            their ground, and the centre wheels shift fore/aft so the centre
 *            of mass stays between the two axles that will be carrying it;
 *   Lift     this axle comes up clear of the higher surface;
 *   Advance  the robot drives on the other two until it is over its landing
 *            spot -- past the edge when climbing, clear of it when descending;
 *   Plant    it goes down onto the far side and takes load.
 *
 * Then the chassis settles, level and on every wheel, to its starting ride
 * height over the new ground. This is the sequence of the
 * lift-drive curb-climb CAD study, generalised: the chassis heights and centre
 * shifts are not fixed poses but are planned from the step height and what
 * each leg can actually reach (FRammsCurbTuning, the 4-bar specs and the
 * 5-bar controllers' own reachability), so the same controller takes a 4 in
 * curb or an 8 in one, up or down, on either chassis -- and refuses, before
 * moving, one its legs cannot span.
 *
 * The base is held level against gravity. Every leg that carries the robot
 * takes its height from the plane under its own wheel -- the near surface
 * until its axle crosses, the far one after -- where the wheel will be a
 * servo lag from now, so sloped and twisted surfaces move the legs, not the
 * chassis; the chassis height is planned over the mean of the wheels'
 * surfaces, and the plan checks every leg reaches what the terrain asks of
 * it over the whole stretch each step holds for. Where no one height does,
 * an axle may land further on (never so far that the next axle reaches its
 * braking point before it has landed), or the chassis may follow a path
 * planned through the corridor of heights every leg and the belly allow, at
 * least Tuning.MinCorridorCm wide all the way and no steeper than
 * Tuning.BaseHeightSpeed lets it climb or sink at speed. Lifted wheels are
 * carried clear of the terrain on their way:
 * climbing, at the curb top's height from the lift on; descending, over the
 * high side they are leaving, and no lower than their legs reach past it. A
 * gentle attitude loop (Tuning.AttitudeProportionalGain,
 * AttitudeIntegralGain) trims what is left as a pure rotation of the support
 * legs, and each surface's height is corrected by where its planted wheels
 * find it. The robot drives up in its own stance; the first Prepare levels
 * it in, timed from how long its legs take so that the base is level before
 * any wheel is RammsCurb::LevelBeforeEdgeCm from its edge -- the attitude
 * loop taking over once the legs are on the terrain -- and the first lift
 * waits for it. A level-in that runs late slows the robot (it comes to rest
 * no nearer the edge than that line, allowing for the chassis running a lag
 * behind its command) rather than faulting.
 *
 * Every plan keeps the robot standing well inside its wheels. The robot's
 * centre of mass (Tuning.CenterOfMass, moved by the mass each leg carries with
 * its wheel, FRammsCurbLeg::MovingMassKg against Tuning.RobotMassKg) must stay
 * at least Tuning.MinStabilityMargin inside the contacts of the wheels on the
 * ground in every pose a lift holds while driving, allowing for the shift of
 * braking at Deceleration or speeding up at Acceleration (h a / g, its height h
 * over the contacts): a two-dimensional margin, so a lift is checked across the
 * robot as well as along it. (A lift that must start standing still keeps at
 * least the smaller of Tuning.LiftOffMargin and MinStabilityMargin standing
 * still.) Each wheel touches where its tread meets the surface: on a slope
 * across it, at its uphill rim. The stance the robot drives up in, which is
 * the drive mode's, is not checked: its margin, evaluated as if level, is
 * recorded with the lifts' (the run stats' MinPlannedMarginCm), and on a
 * sloped approach, where the chassis drives it tilted, it can be the least of
 * the crossing. The same margin is measured on every tick of a crossing
 * (GetStabilityMargin, and the run stats).
 *
 * Speed comes from overlapping the steps rather than from moving any one leg
 * fast: the robot keeps rolling while legs reposition and lift. It slows
 * only as far as a deadline needs: the next axle to be lifted must be clear of
 * the step by the time it reaches its braking point, so the speed is capped at
 * what covers the distance left as the leg work in front of that lift --
 * predicted from the legs' speeds and servo lag, and from how far each one is
 * behind its command now -- runs out, braked back at Deceleration from any
 * step that waits on distance. The cap moves continuously; nothing is a step
 * factor. Every stop is a braking curve against a hard limit -- no wheel on
 * the low side reaches the curb face, no loaded wheel on the high side rolls
 * off the edge -- rather than a fixed waypoint, and those hard limits are the
 * only thing that ever asks for a standstill. A step ends when the legs are
 * measured where they were sent, not when a timer says so.
 *
 * The robot drives as it does in differential mode, on its main drive wheels
 * (the lift-drive's treaded centre wheels), with each other wheel driving or
 * rolling free by its role (FRammsCurbLeg::Drive), side by side: the
 * holonomic chassis' omni wheels on one side take over only while that side's
 * centre wheel is off the ground -- its axle's lift, crossing and landing --
 * and the linkage chassis' powered front wheels help throughout while its
 * rear omni wheels just roll. Each
 * driven wheel turns at the ground speed its centre has: the chassis' plus its
 * leg's measured fore/aft rate. A wheel motor that commands its joint relative
 * to the link it hangs from (MuJoCo's hinge) does not turn the wheel over the
 * ground at that rate when the link turns too -- as a 5-bar's (or a swing
 * arm's) does when its leg moves -- so the link's turn is measured each tick
 * (the wheel's world turn less its joint's) and taken off the command, or
 * walking the centre wheels under load drives the chassis backwards. The
 * joint's turn is the one the backend's speed loop holds: its angle's change
 * where the backend reports the angle, else its reported rate. A backend whose
 * wheel rate is the wheel's own spin in the world (Chaos, for a torque-driven
 * wheel) reports no angle, and its loop already turns the wheel over the
 * ground at the commanded rate, so there the link's turn comes out near 0, as
 * it should. Both rates are for the tick to come: what was measured
 * over the last, plus the change the servo lag model predicts under the leg
 * commands just sent (the link's share of it fitted to what was measured), so
 * that a walk starting or ending is met as it happens rather than a tick late.
 * A crossing whose commanded speed stays up while the chassis does not move
 * for Tuning.StallSeconds faults as a stall.
 *
 * Progress is measured, not dead-reckoned: each wheel's position comes from
 * the robot base (GetMotorTransform on its drive motor), and the chassis'
 * attitude from the corner crank pivots, which are fixed on the chassis.
 *
 * The step is found by tracing the world (DetectTerrainAhead): two planes,
 * the near surface the robot stands on and the far one past the edge, and a
 * straight edge between them at any angle to the heading, so neither a slope
 * nor an angle is taken for a step. The crossing begins once the far side is
 * seen to be deep enough for the whole robot, or at least
 * RammsCurb::MinPlanFarDepthCm of it is; the approach then looks again every
 * RammsCurb::RedetectEveryCm of travel until the first lift begins, blending
 * what it sees into the terrain (a look that moved anything by more than
 * 2 cm or 2 deg is thrown away), and rebuilds the plan when the terrain moved
 * by more than 0.5 cm or 0.5 deg from the one it was built on. The far side
 * must be seen deep enough before the approach ends -- the commit point,
 * where the legs first move for the edge, a PrepareDistance short of the
 * first wheels' window. Until it is, the approach cannot end: the robot
 * brakes at Tuning.Deceleration to rest at that point, allowing for its
 * chassis running a lag behind the command, and once at rest looks once more
 * and refuses if it still has not seen enough. The per-axle gait lifts both
 * wheels of an axle at once, so it crosses an edge at most
 * RammsCurb::PerAxleMaxApproachDeg off square. It refuses a start with any
 * wheel over ground more than Tuning.MinStepHeight off the near surface.
 *
 * It is a drive mode ("curb"): while it runs, the other modes stand down and
 * it owns the wheels and cranks. Starting a manoeuvre selects it through the
 * robot's URammsDriveModeSelector and finishing one hands back to the mode
 * that was active, which puts the robot back in that mode's stance. One
 * stopped by Abort, or by a tilt, step-timeout, stall or measurement fault, keeps
 * the robot held in this mode; a retry that succeeds still hands back to the
 * mode from before. A simulation reset faults it too, but hands back as well:
 * the robot is at its start pose, with nothing across a step to hold.
 * Selected by hand with nothing to cross, it hands straight back -- there is
 * nothing to drive in it.
 *
 * Like the other controllers it holds only configuration, and commands motors
 * by Id through the sibling URammsRobotBaseComponent; nothing here is MuJoCo-
 * specific. Of a wheel motor it assumes only that the rate the base reports is
 * the one its speed loop holds, and that the angle it reports, where it
 * reports one (not 0), is that rate's integral.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsCurbNegotiationController : public UActorComponent, public IRammsControlContributor, public IRammsDriveMode
{
	GENERATED_BODY()

public:
	URammsCurbNegotiationController();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	/** Every leg the gait moves: two per axle. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	TArray<FRammsCurbLeg> Legs;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FRammsCurbTuning Tuning;

	/** Drive mode to hand back to when done. None returns to whichever mode
	 *  was active when the manoeuvre began. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FName ReturnModeId;

	/**
	 * Plan the manoeuvre for this step without moving: true when every leg
	 * can reach what each step of the gait asks of it. OutReason says why
	 * not, or summarises the plan.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	bool CheckFeasibility(FRammsCurbProfile Profile, FString& OutReason);

	/** Plan and start. False, with a reason, when the step cannot be crossed
	 *  from here; nothing moves then. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	bool BeginNegotiation(FRammsCurbProfile Profile, FString& OutReason);

	/**
	 * Look for a step ahead by tracing the world below the robot's path, and
	 * report it as a profile: DetectTerrainAhead, with the profile's fields
	 * derived from the terrain it found (which the profile carries). False
	 * when there is none within Tuning.DetectRange, or when what is there is
	 * not a step this will try.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	bool DetectStepAhead(FRammsCurbProfile& OutProfile, FString& OutReason) const;

	/**
	 * Find the terrain ahead by tracing the world in lanes along the robot's
	 * heading, across its width and Tuning.DetectRange past its front wheels:
	 * the plane it stands on, a straight edge, and the plane beyond it. False,
	 * with why, when there is no step there this controller will cross --
	 * none in range, a ramp or a bend rather than a face, an edge that is not
	 * straight or more than Tuning.MaxApproachDeg off square, a step outside
	 * Tuning.MinStepHeight..MaxStepHeight anywhere along the robot's footprint,
	 * a surface steeper than Tuning.MaxSurfaceSlopeDeg or not a plane, a far
	 * side seen to be too shallow for the robot (or too little of it in
	 * view), or the robot not standing on the near side. OutReason otherwise
	 * describes what it found. See GetDetectionText.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	bool DetectTerrainAhead(FRammsCurbTerrain& OutTerrain, FString& OutReason) const;

	/** The terrain the current (or last) crossing is planned on: as detected,
	 *  then refined as the approach looked again; for a profile given without
	 *  one, the level ground and square edge it stands for. Not valid before
	 *  any plan. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	FRammsCurbTerrain GetActiveTerrain() const { return ActiveTerrain; }

	/** What the last DetectTerrainAhead or DetectStepAhead found, or why it
	 *  found nothing to cross -- also when it returned false, which leaves a
	 *  script no other way to read the reason. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	FString GetDetectionText() const { return DetectionText; }

	/** DetectStepAhead, then BeginNegotiation on what it found. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	bool NegotiateStepAhead(FString& OutReason);

	/** Stop driving and hold the legs where they are. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	void Abort();

	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	ERammsCurbPhase GetPhase() const { return Phase; }

	/** The axle the current step is about (meaningful for Prepare / Lift /
	 *  Advance / Plant). */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	ERammsCurbAxle GetPhaseAxle() const;

	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	bool IsNegotiating() const;

	/** One line: where it is, or why it stopped. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	FString GetStatusText() const { return Status; }

	/** Seconds since BeginNegotiation (frozen once it ends). */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	float GetElapsedSeconds() const { return static_cast<float>(Elapsed); }

	/** The profile the current (or last) manoeuvre was planned for. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	FRammsCurbProfile GetActiveProfile() const { return ActiveProfile; }

	/** The planned steps, one line each, and a log of each step as it began. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	TArray<FString> GetPlanDescription() const { return PlanLines; }

	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	TArray<FString> GetStepLog() const { return StepLog; }

	/** Chassis pitch and roll (deg, positive nose up / +Y side up -- the side
	 *  of the legs whose Wheel.Position.Y is positive) as last measured. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	FVector2D GetMeasuredTiltDeg() const { return FVector2D(MeasuredPitchDeg, MeasuredRollDeg); }

	/** Chassis origin height (cm) above the ground the crossing plans its
	 *  chassis height over, as last measured; and the height the plan is
	 *  commanding. That ground is the ground the crossing started on, carried
	 *  along with the surfaces under the wheels where they slope; on level
	 *  ground, the ground it started on. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	FVector2D GetBaseHeight() const { return FVector2D(MeasuredBaseHeight, BaseHeightCmd + CorridorShiftCmd); }

	/** Each leg's wheel position along the path relative to the step edge
	 *  (cm, positive past it), in Legs order, as last measured -- it keeps the
	 *  last crossing's values once that ends. Empty until a plan has been
	 *  built. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	TArray<float> GetWheelProgress() const;

	/** Each leg's wheel-centre height in the chassis frame (cm), in Legs
	 *  order: X commanded, Y measured. Empty when never planned. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	TArray<FVector2D> GetLegHeights() const;

	/** Each wheel's bottom above the surface it follows (cm), in Legs order,
	 *  as last measured: planted, the near surface until its axle crosses the
	 *  edge and the far one after; lifted, the surface under it. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	TArray<float> GetWheelClearance() const;

	/** How the current crossing is going, or how the last one went: speeds,
	 *  stops, attitude, margins and timing, counted on every controller tick
	 *  to Done -- the speeds once the robot is moving. See FRammsCurbRunStats. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	FRammsCurbRunStats GetRunStats() const;

	/** Chassis ground speed (cm/s): X the drive speed the crossing commands
	 *  (0 when none runs), Y the speed measured along the chassis heading over
	 *  the last 0.3 s of simulated time -- measured whether or not a crossing
	 *  runs. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	FVector2D GetDriveSpeed() const;

	/** Each wheel's part in the crossing, in Legs order, as last measured: its
	 *  state, the surface it is on or headed for, where it is against the
	 *  edge, and how high it is over the ground under it (traced on each
	 *  call). The per-axle gait moves a wheel with its axle, so both wheels of
	 *  an axle share a state. Empty until a crossing has begun, and after a
	 *  plan made without crossing (CheckFeasibility). */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	TArray<FRammsCurbWheelStatus> GetWheelStatus() const;

	/** The robot's stability margin as last measured in a crossing (cm): how
	 *  far inside the contacts of its wheels on the ground (their convex hull)
	 *  its centre of mass stands, projected straight down, less the shift of
	 *  braking at Tuning.Deceleration or speeding up at Tuning.Acceleration
	 *  (whichever leaves less); negative outside. The centre of mass is the
	 *  robot model's (Tuning.CenterOfMass, each leg's MovingMassKg) at the
	 *  measured legs. It keeps the last crossing's value once that ends; 0
	 *  before any. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	float GetStabilityMargin() const { return LiveMarginCm; }

	// --- IRammsDriveMode ------------------------------------------------------
	virtual FName GetDriveModeId() const override { return FName("curb"); }
	virtual FText GetDriveModeDisplayName() const override
	{
		return NSLOCTEXT("Ramms", "DriveModeCurb", "Curb");
	}
	virtual bool IsDriveModeActive() const override { return bDriveModeActive; }
	virtual void SetDriveModeActive(bool bActive) override;

	// --- IRammsControlContributor --------------------------------------------
	virtual void  DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool  ApplyControl(FName Id, float Value) override { return false; }
	virtual bool  TriggerControl(FName Id) override;
	virtual bool  ReadControl(FName Id, float& OutValue) const override;
	virtual void  GetClaimedMotorIds(TArray<FName>& OutIds) const override;
	virtual int32 GetControlOrder() const override { return 5; }

	/** Control Ids this contributes. */
	static FName NegotiateControlId() { return FName(TEXT("curb.negotiate")); }
	static FName AbortControlId() { return FName(TEXT("curb.abort")); }
	static FName PhaseControlId() { return FName(TEXT("curb.phase")); }

private:
	/** What a leg is asked to do during a step. */
	enum class ELegGoal : uint8
	{
		/** On the ground at a level. */
		Support,
		/** Clear of the ground, above a level. */
		Swing,
		/** Where it was when the manoeuvre began: the robot approaches the
		 *  step in the stance it was driving in. */
		Hold,
	};

	/** An axle's goal: what its legs do, and on which surface. Level is the
	 *  height of that surface in the level-ground plan (0 the near side,
	 *  the step height the far side; a lifted axle's, the higher of the two);
	 *  each leg follows the real surface under its own wheel. */
	struct FAxleGoal
	{
		ELegGoal Goal = ELegGoal::Support;
		float	 Level = 0.0f;
		/** On the far surface, or (lifted) headed for it. */
		bool bFar = false;
	};

	/** What the terrain does to each leg over the stretch of the crossing one
	 *  set of goals holds for (see SpanGoals). */
	struct FGoalSpan
	{
		/** The least and most each leg's wheel sits above where the
		 *  level-ground plan puts it (cm, chassis frame). */
		TArray<FVector2D> Offset;
		/** The least chassis height (cm, plan) that keeps the belly
		 *  ChassisClearance over the higher surface under it. */
		float Floor = 0.0f;
		/** The same at each sample along the stretch, evenly from SampleFrom
		 *  to SampleTo (cm of chassis travel from where the plan was made,
		 *  about SpanSampleCm apart): its belly floor, each leg's least and
		 *  most offset there ([sample * legs + leg]; a 5-bar's over the
		 *  fore/aft spread; NaN where the leg has none there -- a lifted wheel
		 *  past where it lands), and the terrain reference there (cm over
		 *  GroundZ: a plan height H puts the chassis origin at GroundZ + H +
		 *  this). */
		float			  SampleFrom = 0.0f;
		float			  SampleTo = 0.0f;
		TArray<float>	  SampleFloor;
		TArray<FVector2D> SampleOffset;
		TArray<float>	  SampleReference;
	};

	/** The chassis height a step that follows its corridor is planned to
	 *  hold: its world height (cm over GroundZ) every Step cm of chassis
	 *  travel from From, measured from where the plan was made
	 *  (PlanOriginXY) along the heading. */
	struct FCorridorPath
	{
		float		  From = 0.0f;
		float		  Step = 0.0f;
		TArray<float> Heights;

		bool IsValid() const { return Heights.Num() > 0 && Step > 0.0f; }
		/** The height at Travel, between the samples; held beyond either end. */
		float At(float Travel) const;
	};

	/** One set of goals, as PlanLift weighs it: the goals and their span. */
	struct FGoalSet
	{
		const FAxleGoal* Goals = nullptr;
		const FGoalSpan* Span = nullptr;
	};

	/** One step of the plan. */
	struct FStep
	{
		ERammsCurbPhase Phase = ERammsCurbPhase::Idle;
		ERammsCurbAxle	Axle = ERammsCurbAxle::Front;
		FAxleGoal		Axles[3];
		float			BaseHeight = 0.0f;
		/** Fore/aft target (cm) for each 5-bar leg, in Legs order. */
		TArray<float> LegX;
		/** Advance: the axle's wheels end the step this far past the edge. */
		float AdvanceTo = 0.0f;
		/** Approach: the front wheels end the step this far past the edge. */
		float ApproachTo = 0.0f;
		/** The step does not end before the front wheels are this far past
		 *  the edge (the level-in: where the gait's first Prepare began before
		 *  the base was levelled ahead of it). */
		float GateTo = -TNumericLimits<float>::Max();
		float SpeedFraction = 0.0f;
		/** What the terrain does to each leg over this step (planned). */
		FGoalSpan Span;
		/** No one chassis height over the terrain reference serves the whole
		 *  step: the chassis follows Corridor, a path inside the corridor of
		 *  heights every leg and the belly allow wherever the robot may be in
		 *  this axle's crossing (PlanLiftCorridor), and is kept inside the
		 *  corridor where the robot is (CorridorAt). */
		bool		  bFollowCorridor = false;
		FCorridorPath Corridor;
	};

	/** A leg's live state. */
	struct FLegState
	{
		URamms5BarLinkageController* FiveBar = nullptr;
		FVector2D					 HeightRange = FVector2D::ZeroVector;
		float						 InitialX = 0.0f;
		float						 InitialZ = 0.0f;
		float						 CmdZ = 0.0f;
		float						 CmdX = 0.0f;
		/** The wheel's fore/aft position (cm, chassis frame) at its
		 *  commanded height and fore/aft target. */
		float ModelX = 0.0f;
		/** Measured wheel centre (cm, chassis frame): a 5-bar's fore/aft
		 *  position, a 4-bar's from its crank. */
		float MeasuredX = 0.0f;
		float MeasuredZ = 0.0f;
		float Progress = 0.0f;
		/** The wheel's bottom above the surface it follows (cm). */
		float WheelBottom = 0.0f;
		/** The height the leg's command ramps towards this tick. */
		float GoalZ = 0.0f;
		/** The surface the wheel follows: the far one (or, lifted, headed
		 *  for it); else the near one. */
		bool bOnFar = false;
		/** How much higher (cm, chassis frame) the terrain puts the wheel than
		 *  the level-ground plan does, against the commanded reference: for a
		 *  support leg, its own surface (RammsCurb::TerrainOffsets); for a
		 *  lifted one, the height it is carried clear of the terrain at. */
		float TerrainOffset = 0.0f;
		/** The attitude loop's rotation of this leg (cm), while it supports. */
		float AttitudeDz = 0.0f;
		/** Its crank was read past an end of its range this crossing, and
		 *  that was logged. */
		bool bCrankClampLogged = false;
		/** Measured wheel centre, world. */
		FVector WheelWorld = FVector::ZeroVector;
		/** The wheel's world rotation, and its joint's angle (rad; 0 from a
		 *  backend that reports none) and rate (rad/s) in the motor's sense,
		 *  as last measured, and those of the tick before. */
		FQuat WheelRotation = FQuat::Identity;
		float WheelAngle = 0.0f;
		float WheelRate = 0.0f;
		FQuat LastWheelRotation = FQuat::Identity;
		float LastWheelAngle = 0.0f;
		float LastWheelRate = 0.0f;
		bool  bHasLastWheel = false;
		/** Where the servo lag model puts the wheel centre (chassis frame; a
		 *  4-bar's fore/aft from its modelled height), and how fast it moved
		 *  it over the last tick (cm/s: x, z). */
		float	  LagX = 0.0f;
		float	  LagZ = 0.0f;
		FVector2D LagRate = FVector2D::ZeroVector;
		bool	  bHasWorld = false;
	};

	URammsRobotBaseComponent* EnsureBase() const;
	bool					  ResolveLegs(FString& OutReason);
	bool					  IsPlusY(const FRammsCurbLeg& Leg) const { return Leg.Wheel.Position.Y > 0.0f; }

	/** Measure the robot as it starts, check it can start here, and build
	 *  Steps for a profile (BuildSteps). */
	bool BuildPlan(const FRammsCurbProfile& Profile, FString& OutReason);

	/** Build Steps for a profile from the state measured as the crossing
	 *  began: the edge placed from where the front wheels are now. Also how a
	 *  crossing re-plans on the approach. */
	bool BuildSteps(const FRammsCurbProfile& Profile, FString& OutReason);

	/** Whether the per-axle gait can cross an edge at the profile's angle. */
	bool PerAxleAccepts(const FRammsCurbProfile& Profile, FString& OutReason) const;

	/** Place the edge the per-axle gait reckons progress from: the profile's
	 *  EdgeDistanceCm ahead of the front wheels along the heading. Every
	 *  wheel's progress is taken against it. False when the front wheels
	 *  cannot be located. */
	bool PlaceEdge(const FRammsCurbProfile& Profile, FString& OutReason);

	/** What the terrain does to each leg while one set of goals holds, with
	 *  each 5-bar at LegX (and anywhere within FiveBarSpreadCm of it,
	 *  fore/aft): the chassis moved along the heading from where it is now by
	 *  SFrom..STo (cm), the wheels lifted in it landing once it has gone
	 *  SLand (SwingCentreZ). */
	void SpanGoals(const FAxleGoal Goals[3], const TArray<float>& LegX, float FiveBarSpreadCm, float SFrom, float STo,
		float SLand, FGoalSpan& Out) const;

	/** How high (cm, world) a lifted wheel's centre must be carried at At
	 *  (world xy), ToLandCm along the heading short of where it lands:
	 *  SwingClearance over the terrain its rim could touch (the higher side
	 *  reaching FaceMargin out over a climb's face, EdgeMargin past a
	 *  descent's lip). Climbing, over the rest of its way to the landing:
	 *  it is carried at the curb top's height from the lift on. Descending,
	 *  over its own footprint and RammsCurb::DescentSwingLookCm ahead: it
	 *  follows the high side it is leaving, and nothing ahead rises sharply.
	 *  On level ground both are the higher surface plus the radius. */
	double SwingCentreZ(int32 LegIndex, const FVector2D& At, float ToLandCm) const;

	/** How far (cm, along the heading) leg LegIndex's axle has still to go
	 *  to its landing (its Advance step's end); 0 with none ahead. */
	float ToLandCm(int32 LegIndex) const;

	/** Measure where every wheel's surface is now, and what that makes of
	 *  each leg against the plan's level ground: the plane it follows (by its
	 *  step's goal), its bottom over it, the reference the chassis height is
	 *  planned over, each leg's terrain offset, and (DeltaTime > 0) each
	 *  surface's bias from its planted wheels. */
	void UpdateWheelSurfaces(float DeltaTime);

	/** Leg LegIndex's goal in the current step (Hold before any). */
	FAxleGoal LegGoal(int32 LegIndex) const;

	/** On the ground and carrying: not lifted, and its bottom within reach
	 *  of the surface it follows. */
	bool IsLegPlanted(int32 LegIndex) const;

	/** How high (cm, world) a wheel's bottom must be to count as clear of
	 *  the curb: SwingClearance / 2 over the curb top at its stretch of edge
	 *  (a climb), or over the surface it stands on (a descent). */
	double ClearBottomZ(int32 LegIndex) const;

	/** The attitude loop for this tick: the correction, and each support
	 *  leg's share of it (FLegState::AttitudeDz). */
	void UpdateAttitude(float DeltaTime);

	/** Latch the level-in done once the base is level and the support legs'
	 *  commands are where the terrain wants them. */
	void UpdateLevelIn();

	/** Seconds (predicted) until the level-in is done; 0 once it is. */
	float LevelInSeconds() const;

	/** The speed cap (cm/s) on a level-in that is late: the robot goes no
	 *  nearer the edge than the line the base must be level by. */
	float LateLevelCap() const;

	/** DetectTerrainAhead's work: OutInput, when given, gets what it looked
	 *  from (the wheels, the chassis centre and heading). bBeforeStart: make
	 *  the checks a crossing makes before it starts -- the robot stands on the
	 *  near side, short of the edge, and the far side is deep enough for it,
	 *  or enough of it in view to start; a look again on the approach only
	 *  measures, and the controller judges the far depth itself. */
	bool DetectTerrain(bool bBeforeStart, FRammsCurbTerrain& OutTerrain, FString& OutReason,
		RammsCurb::FTerrainFitInput* OutInput) const;

	/** Every wheel as the terrain detection sees it, from the last
	 *  measurement. */
	void GatherTerrainWheels(TArray<RammsCurb::FTerrainWheel>& Out) const;

	/** How deep the far side must be for this robot (RammsCurb::FFarDepthNeed). */
	void GatherFarDepthNeed(RammsCurb::FFarDepthNeed& Out) const;

	/** On the approach: look at the terrain again once the chassis has gone
	 *  RammsCurb::RedetectEveryCm since the last look (or now, bForce), blend
	 *  it in, confirm the far depth, and re-plan when it moved. */
	void LookAgain(bool bForce);

	/** Rebuild the plan on ActiveTerrain; false, with why, when it cannot be
	 *  crossed. */
	bool Replan(FString& OutReason);

	/** While the far depth is unconfirmed: how far (cm, along the heading) the
	 *  chassis may go before the commit point (CommitTo) -- where the per-axle
	 *  gait's legs start to move for the first axle, PrepareDistance short of
	 *  that axle's first window. The approach does not end while the far
	 *  depth is pending, nor does the level-in begin. */
	float FarDepthStopDistance() const;

	/** Leg height (cm) a step asks for at chassis height BaseHeight on level
	 *  ground, plus Offset: how much higher the terrain puts its wheel. */
	float GoalHeight(int32 LegIndex, const FAxleGoal& Goal, float BaseHeight, float Offset = 0.0f) const;

	/** GoalHeight at the commanded chassis height over the terrain as it is
	 *  now, as actually sent: support legs on the surface under their own
	 *  wheel and turned by the attitude loop, lifted wheels raised further for
	 *  the chassis' measured pitch and roll. */
	float CommandedGoalHeight(int32 LegIndex, const FAxleGoal& Goal) const;

	/** A planned step's leg height at its own chassis height, the terrain
	 *  offset the middle of what it plans for that leg. */
	float PlannedGoalHeight(int32 StepIdx, int32 LegIndex) const;

	/** Whether one leg can hold a goal at BaseHeight, a 5-bar at fore/aft X,
	 *  over the terrain offsets Offset (least, most) the goal meets. */
	bool LegReachable(int32 LegIndex, const FAxleGoal& Goal, float BaseHeight, float X, const FVector2D& Offset,
		FString* OutWhy) const;

	/** Whether every leg can hold a step's goals at BaseHeight, each 5-bar at
	 *  its entry in LegX, over the terrain offsets of Span (none: level). */
	bool IsReachable(const FAxleGoal Axles[3], float BaseHeight, const TArray<float>& LegX, float ChassisFloor,
		const FGoalSpan* Span, FString* OutWhy) const;

	/** The chassis heights (cm, plan) at which every leg reaches its goal in
	 *  Goals, each 5-bar at LegX, over the terrain offsets Offset (per leg,
	 *  the least and most; NaN for none), with the belly over Floor: (least,
	 *  most); least > most when there are none. A lifted leg bounds it from
	 *  below only. OutLo/OutHi name the legs that bound it (-1: the belly, or
	 *  nothing). */
	FVector2D CorridorAt(const FAxleGoal Goals[3], TConstArrayView<FVector2D> Offset, float Floor, const TArray<float>& LegX,
		int32* OutLo = nullptr, int32* OutHi = nullptr) const;

	/** The heights (cm, chassis frame) a 5-bar leg's wheel reaches at fore/aft
	 *  X (least, most; least > most where none), from its controller. */
	FVector2D FiveBarReachAt(int32 LegIndex, float X) const;

	/** Half the belly plate (cm): Tuning.ChassisHalfExtent, or what stands
	 *  in for it. */
	FVector2D BellyHalfExtent() const;

	/** The highest ground (cm, world) under the belly plate with the chassis
	 *  origin over Origin (world xy), facing the heading. */
	double BellyTopAt(const FVector2D& Origin) const;

	/**
	 * PlanLift where no one chassis height over the terrain reference serves
	 * every set over its whole stretch: a path for the chassis' height along
	 * its travel (OutPath). At each sample, every set whose stretch reaches
	 * it may be the one in force -- a step ends when its legs are done, not
	 * at a set distance -- so the path keeps inside all their corridors
	 * (CorridorAt, in world height), Tuning.MinCorridorCm / 2 in from either
	 * side (a corridor narrower than MinCorridorCm fails), and moves no faster
	 * than Tuning.BaseHeightSpeed at the speed the robot may come in at;
	 * failing that, it fails. The path keeps as near PreferredHeight (plan)
	 * as that lets it, and LegRangeMargin further in from the corridor's
	 * sides where there is room. The centre wheels at one fore/aft offset.
	 * OutHeight: the step's plan height, the path's where the first set
	 * begins.
	 */
	bool PlanLiftCorridor(ERammsCurbAxle Axle, TArrayView<const FGoalSet> Sets, float PreferredHeight, float PreferredCenterX,
		float MinMargin, float& OutHeight, TArray<float>& OutLegX, float& OutShift, float& OutMargin, FCorridorPath& OutPath,
		FString& OutReason) const;

	/** For a step that follows its corridor: the chassis' offset from its
	 *  plan height (CorridorShiftCmd) moved towards its planned path a servo
	 *  lag ahead, kept inside the corridor at the terrain as it is now; for
	 *  any other step, back towards 0. */
	void UpdateCorridor(float DeltaTime);

	/** Pick the chassis height and centre-wheel offsets that hold every one
	 *  of Sets while Axle is lifted, each 5-bar as near PreferredCenterX as it
	 *  reaches, with the robot's stability margin (PlannedMargin) at least
	 *  MinMargin in each: braking or speeding up as it drives, or standing
	 *  still (bStanding). OutShift: the centre wheels' mean past the centre of
	 *  mass towards PreferredCenterX; OutMargin: the least margin. */
	bool PlanLift(ERammsCurbAxle Axle, TArrayView<const FGoalSet> Sets, float ChassisFloor, float PreferredHeight,
		float PreferredCenterX, float MinMargin, bool bStanding, float& OutHeight, TArray<float>& OutLegX, float& OutShift,
		float& OutMargin, FString& OutReason) const;

	/**
	 * The robot's stability margin (cm, RammsCurb::PoseMargin) in a planned
	 * pose: the chassis level at plan height BaseHeight, each leg at its goal
	 * in Goals over the middle of its terrain offsets -- SampleOffset's (one
	 * per leg: a corridor sample's, FGoalSpan::SampleOffset) where that has a
	 * range for it, else the whole stretch's Span gives it -- (a lifted leg at
	 * its carry height, a held one where it began), each 5-bar at its LegX;
	 * the support legs those whose goal is Support, or Hold for an axle that
	 * began on the ground. With the shift of braking or speeding up unless
	 * bStanding.
	 */
	float PlannedMargin(const FAxleGoal Goals[3], float BaseHeight, const TArray<float>& LegX, const FGoalSpan* Span,
		bool bStanding, TConstArrayView<FVector2D> SampleOffset = TConstArrayView<FVector2D>()) const;

	/**
	 * The least shift (cm) of the centre wheels past Tuning.CenterOfMassX,
	 * towards the axle Axle lifts, that the robot model needs for a margin of
	 * MinMargin (PlannedMargin) in every one of Sets at plan height
	 * BaseHeight, whatever the legs reach; NaN when no shift up to 30 cm
	 * gives it, 0 for the centre axle (the robot then stands on its corners).
	 * For refusal reasons that say what a placement would need.
	 */
	float RequiredCentreShift(ERammsCurbAxle Axle, TArrayView<const FGoalSet> Sets, float BaseHeight, float MinMargin, bool bStanding) const;

	/** Measure the chassis and every wheel; false, with why, when the robot
	 *  cannot be measured: a wheel or pivot with no transform (none yet), or
	 *  joint readings where a 4-bar or 5-bar linkage does not close. */
	bool Measure(FString& OutReason);

	/** World height of the surface under At, tracing down from AboveZ to
	 *  BelowZ past the robot itself; false when nothing is there. */
	bool TraceSurface(const FVector& At, double AboveZ, double BelowZ, double& OutZ) const;

	/** The mode to hand the robot to when there is no better choice: the
	 *  selector's initial mode, or else the first mode that is not this. */
	FName FallbackModeId() const;

	/** The active drive mode when it is not this one, else None. */
	FName OtherActiveModeId() const;

	void EnterStep(int32 Index);
	void StepPlan(float DeltaTime);
	bool LegsSettled() const;
	bool StepComplete() const;
	/** The drive speed for this tick: the ramp towards the step's speed
	 *  under the deadline cap, clamped by the braking caps. */
	float ComputeDriveSpeed(float DeltaTime) const;
	/** Whether a hard stop holds a wheel back -- one not yet clear of the
	 *  curb top short of the face, a loaded one on the high side short of the
	 *  lip -- and how far (cm, along the heading) it has left to it. */
	bool BrakingLimit(int32 LegIndex, float& OutDistanceCm) const;
	/** The deadline cap (cm/s): the speed at which the next axle to be lifted
	 *  reaches its braking point as the leg work in front of it ends; none
	 *  (the largest float) with no lift ahead. */
	float DeadlineCap() const;
	/** Predicted seconds for step Index's legs to reach its goals and settle:
	 *  from where they are now for the current step (unless bPlanned), from
	 *  the step before's goals for a later one. */
	float StepSeconds(int32 Index, bool bPlanned = false) const;
	/** Predicted seconds until the axle step Index lifts is clear of the curb
	 *  top (a climb) or the surface it stands on (a descent). */
	float LiftClearSeconds(int32 Index) const;
	/** Feed each wheel's feed-forward this tick's measurement against the
	 *  last: its leg's rates, and the turn of the link it hangs from (the
	 *  wheel's world turn about its axle less its joint's). After
	 *  AdvanceLegLag, whose model rates it takes. */
	void TrackLegRates(float DeltaTime);
	/** Tell each wheel's feed-forward what the servo lag model makes of the
	 *  leg commands just sent, over a tick of DeltaTime. */
	void PredictLegRates(float DeltaTime);
	/** A 4-bar leg's wheel fore/aft (cm, chassis frame) at wheel height Z;
	 *  Fallback where the leg cannot reach Z. */
	float FourBarX(int32 LegIndex, float Z, float Fallback) const;
	/** The ground speed (cm/s) of the planted main wheels, from their wheel
	 *  rates (with none planted, the chassis' measured speed): where a
	 *  crossing begun on the move starts its drive speed, and the speed the
	 *  commit point's braking allows the chassis' lag for. */
	float MeasuredWheelSpeed() const;
	/** Ramp every leg towards the step's goals by at most RampStep's worth. */
	void  CommandLegs(float RampStep);
	void  CommandWheels(float Speed, float YawRate);
	void  StopWheels(bool bRelease);
	void  Finish(ERammsCurbPhase EndPhase, const FString& Why);
	void  SetActiveModeThroughSelector(FName ModeId);
	float AxleProgress(ERammsCurbAxle Axle) const;

	// --- telemetry ---
	/** The chassis' horizontal centre and heading (world) from its corner
	 *  crank pivots; false when they cannot be located. */
	bool LocateChassis(FVector2D& OutCentre, FVector2D& OutForward) const;
	/** Feed the chassis-speed window: every tick, crossing or not. */
	void TrackChassisMotion();
	/** Run each leg's servo lag model on over DeltaTime under the commands
	 *  still standing, and return the largest leg error against it. */
	float AdvanceLegLag(float DeltaTime);
	/** Count this tick in the run stats. */
	void RecordRunTick(float DeltaTime, float LegErrorCm);
	/** The least gap between a lifted, clear wheel and the terrain under it,
	 *  when one is being carried across; false when none is. */
	bool MeasureSwingClearance(float& OutCm) const;
	/** The robot's stability margin as measured this tick (GetStabilityMargin):
	 *  the robot model's centre of mass at the measured legs, through the
	 *  measured chassis pose, against the contacts of the wheels planted
	 *  (IsLegPlanted); false when none is. */
	bool MeasureStabilityMargin(float& OutCm) const;

	UPROPERTY(Transient)
	mutable TObjectPtr<URammsRobotBaseComponent> BaseComponent = nullptr;

	TArray<FLegState> LegState;
	TArray<FStep>	  Steps;
	int32			  StepIndex = INDEX_NONE;
	double			  StepElapsed = 0.0;
	double			  Elapsed = 0.0;

	ERammsCurbPhase	  Phase = ERammsCurbPhase::Idle;
	FRammsCurbProfile ActiveProfile;
	FString			  Status = TEXT("Idle");
	TArray<FString>	  PlanLines;
	TArray<FString>	  StepLog;
	/** What measuring the robot for a plan noticed (a crank read past its
	 *  stop), for the plan's lines. */
	TArray<FString> MeasureNotes;

	/** The step edge: a point on it and the horizontal heading it is crossed
	 *  along (world). */
	FVector EdgePoint = FVector::ZeroVector;
	FVector Heading = FVector::ForwardVector;
	/** The chassis' own horizontal forward, as last measured (Heading is the
	 *  one it holds). */
	FVector ChassisForward = FVector::ForwardVector;
	/** The chassis' origin and axes in the world, as last measured (X forward,
	 *  Y towards +Y, Z up out of the belly). */
	FVector ChassisOriginWorld = FVector::ZeroVector;
	FVector ChassisAxisX = FVector::ForwardVector;
	FVector ChassisAxisY = FVector::RightVector;
	FVector ChassisAxisZ = FVector::UpVector;
	/** World height of the ground the manoeuvre started on: the lowest wheel
	 *  bottom. The plan's chassis heights are reckoned from it, plus the
	 *  terrain reference (TerrainReferenceCmd). */
	double GroundZ = 0.0;
	/** The terrain reference (RammsCurb::TerrainOffsets): the mean height of
	 *  the wheels' surfaces over GroundZ, less their levels in the plan; as
	 *  measured, and as commanded (followed at BaseHeightSpeed). The chassis'
	 *  world height is GroundZ + BaseHeightCmd + TerrainReferenceCmd. */
	double TerrainReference = 0.0;
	double TerrainReferenceCmd = 0.0;
	/** The chassis origin's world height, as last measured. */
	double MeasuredOriginZ = 0.0;
	/** The chassis origin (world xy), as last measured. */
	FVector2D ChassisOriginXY = FVector2D::ZeroVector;

	/** The physics clock at the last tick, or negative when the backend keeps
	 *  none; every rate and timeout here is in simulated seconds. */
	double LastSimTime = -1.0;
	/** Pitch and roll, low-passed, for keeping lifted wheels clear on a
	 *  tilted chassis. */
	float FilteredPitchRad = 0.0f;
	float FilteredRollRad = 0.0f;

	float BaseHeightCmd = 0.0f;
	float InitialBaseHeight = 0.0f;
	float DriveSpeed = 0.0f;
	float MeasuredBaseHeight = 0.0f;
	/** How far (cm) the chassis is held off its planned height to stay in a
	 *  step's corridor (bFollowCorridor); 0 otherwise. */
	float CorridorShiftCmd = 0.0f;
	/** The chassis origin (world xy) where the steps were last built: the
	 *  plan's travel is measured from it along the heading. */
	FVector2D PlanOriginXY = FVector2D::ZeroVector;
	/** FiveBarReachAt's answers, by leg and fore/aft position (mm). */
	mutable TMap<int64, FVector2D> FiveBarReachCache;
	/** The level-in's predicted duration (s), as planned. */
	float PlannedLevelInSeconds = 0.0f;
	/** The step height (cm) the steps are planned for: the far surface's
	 *  level in the plan. */
	float PlanStepHeight = 0.0f;
	float MeasuredPitchDeg = 0.0f;
	float MeasuredRollDeg = 0.0f;
	/** Heading error, robot frame, positive when the path lies towards +Y. */
	float HeadingErrorRad = 0.0f;

	/** The mode that had the robot before this one; kept across retries. */
	FName PreviousModeId;
	bool  bDriveModeActive = false;
	bool  bWheelsDriven = false;
	/** Selected without a manoeuvre to run (picked from the mode list): hand
	 *  the robot back on the next tick, outside the selector's switch. */
	bool bHandBackPending = false;

	/** The stability margin last measured in a crossing (GetStabilityMargin). */
	float LiveMarginCm = 0.0f;

	/** What the last plan called for, for the run stats: its slowest step,
	 *  and the least stability margin it allowed (when it planned one). */
	float PlannedMinSpeed = 0.0f;
	float PlannedMinMargin = 0.0f;
	bool  bPlannedMargin = false;
	/** The one axle the stance the robot drove up in holds off the ground,
	 *  which the approach keeps up (ERammsCurbAxle as int), or INDEX_NONE. */
	int32 ApproachLiftedAxle = INDEX_NONE;
	/** Which axles were clear of the ground as the crossing began (front,
	 *  centre, rear). */
	bool bStartsLifted[3] = { false, false, false };

	/** The terrain the crossing runs on (GetActiveTerrain), and the one its
	 *  steps were last built on. */
	FRammsCurbTerrain ActiveTerrain;
	FRammsCurbTerrain PlannedTerrain;
	/** ActiveTerrain was detected, so the approach can look again; a profile
	 *  given without a terrain was not. */
	bool bTerrainDetected = false;
	/** The far side has not yet been seen as deep as the robot needs
	 *  (FarDepthNeededCm): the approach must see it before it ends (the
	 *  commit point), or come to rest there and refuse, saying FarDepthWhy. */
	bool	bFarDepthPending = false;
	float	FarDepthNeededCm = 0.0f;
	FString FarDepthWhy;
	/** The commit point (cm past the edge, the front wheels): the approach's
	 *  end as the per-axle gait sets it, PrepareDistance short of the front
	 *  wheels' first window -- kept when the level-in moves the approach's end
	 *  earlier, since the level-in readies nothing for the edge (and does not
	 *  begin while the far side is unconfirmed). */
	float CommitTo = 0.0f;
	/** Where the chassis was at the last look, and what the last failed look
	 *  said (logged once while it keeps saying it). */
	FVector2D LastLookCentre = FVector2D::ZeroVector;
	bool	  bHasLastLook = false;
	FString	  LastLookFailure;

	/** What the last detection called from outside found, or why not. */
	mutable FString DetectionText;

	/** The measured chassis speed and the run stats (Private/Curb). */
	TPimplPtr<RammsCurb::FRunTelemetry> Telemetry;

	/** The stall watch and each wheel's feed-forward (Private/Curb). */
	TPimplPtr<RammsCurb::FDriveState> Drive;

	/** The level base: the attitude loop, the surfaces' biases and whether
	 *  the level-in is done (Private/Curb). */
	TPimplPtr<RammsCurb::FLevelState> Level;

	/** The robot model: each 4-bar's table, the centre of mass and the legs'
	 *  moving masses, the belly (Private/Curb); built with the legs. */
	TPimplPtr<RammsCurb::FRobotModel> Model;
};
