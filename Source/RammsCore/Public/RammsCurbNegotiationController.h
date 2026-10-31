// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "RammsControlContributor.h"
#include "RammsCurbNegotiationTypes.h"
#include "RammsDriveMode.h"
#include "RammsHUDPanelProvider.h"
#include "Templates/PimplPtr.h"

#include "RammsCurbNegotiationController.generated.h"

class URammsRobotBaseComponent;
class URamms5BarLinkageController;
class UCanvas;
class APlayerController;
class SWidget;
class SRammsCurbScope;
class SRammsCurbLabels;
class UGameViewportClient;
class ULocalPlayer;
namespace RammsCurb
{
	struct FMeasuredSeat;
}

namespace RammsCurb
{
	struct FRunTelemetry;
	struct FDriveState;
	struct FLevelState;
	struct FTerrainFitInput;
	struct FTerrainWheel;
	struct FFarDepthNeed;
	struct FRobotModel;
	struct FCurbSchedule;
	struct FFiveBarGridCache;
	struct FPlannerInput;
	class FPlanTask;
	class FExecutor;
	struct FExecMeasure;
	struct FDebugViewState;
	struct FDebugPainter;
} // namespace RammsCurb

/**
 * Climbs or descends a curb by coordinating every leg and wheel of a robot
 * whose wheels each hang on a leg that can lift it: three axles, the corner
 * wheels on crank-driven 4-bars and the centre wheels on 5-bars, with drive
 * motors on the wheels (FRammsCurbLeg). The crossing keeps the robot rolling
 * and its base level wherever the legs can span the terrain, and refuses,
 * before anything moves, a crossing they cannot.
 *
 * The gait (Tuning.Gait PerWheel, the default). Every wheel crosses on its
 * own. Before anything moves the whole crossing is planned in the chassis'
 * travel along its heading (the per-wheel planner): from the terrain -- the
 * plane the robot stands on, the plane past the edge and a straight edge at
 * any angle between them -- and from what each leg can reach, how fast it
 * moves (Tuning.LegSpeed, LegAcceleration, each crank's
 * FRammsCurbLeg::MaxCrankRateRadS) and how far its servo runs behind its
 * command (ServoLagSeconds), it places each wheel's lift, carry and landing
 * windows, the chassis height and the centre wheels' fore/aft path over the
 * whole crossing, and the speed at every point, so that every lifted wheel
 * clears the terrain under it (SwingClearance), the belly clears the higher
 * surface (ChassisClearance), every carrying leg stays inside its reach
 * (LegRangeMargin) and the robot stays inside its wheels at every instant
 * (below). Its speed envelope never drops under a floor, MinSpeedFraction of
 * DriveSpeed: legs move while the robot rolls, and their moves overlap one
 * another. The planner's fallbacks are, in order: a level crossing at the
 * floor or faster; a slower level crossing (never under the floor); the base
 * tilted by the least inclination that keeps every pose, at most
 * MaxPlannedTiltDeg, eased in and out within TiltRateDegPerSec and
 * TiltAccelDegPerSec2 over only the stretch that needs it and back to level
 * after; a level crossing with planned stops (bAllowPlannedStop); else a
 * refusal that names the leg and the limit that bind. A crossing's plan is
 * made off the game thread (BeginNegotiation; CheckFeasibility plans in the
 * call), sampled on grids laid on the terrain so that the same terrain plans
 * the same whether the robot stands a little nearer or further, and checked
 * over the whole crossing before it is kept.
 *
 * The plan is then run against the robot as measured, each wheel on its own
 * state machine (Planted, Lifting, Clear, Planting, Loading, Held; see
 * GetWheelStatus) triggered by that wheel's measured distance past the edge,
 * not by time: a lift begins only where the wheels it must not be off the
 * ground with are down and the robot as measured, with that wheel up, still
 * stands inside its wheels; a landing is found by the wheel touching the
 * surface (or its leg stopping above where its servo lag model has it), then
 * takes load before the legs that carry with it lead it on. Each leg is
 * commanded a servo lag ahead along the plan, as a trapezoid at LegSpeed and
 * LegAcceleration; each lifted wheel with a motor is spun up to the ground
 * speed it will land at (bPrespinLiftedWheels). The speed is the plan's,
 * capped by each lift's deadline (the speed at which the wheel reaches its
 * window just as the leg work in front of its lift is done, predicted from
 * the legs' speeds, servo lag and how far each is behind its command now),
 * ramped at Acceleration and Deceleration. Every stop is a braking curve
 * against a hard limit -- no wheel not yet clear of the curb goes into its
 * window climbing, no loaded wheel on the high side does descending. Along
 * the way the robot slows rather than stops: besides those limits, only a
 * stop the plan itself has, a level-in it has to make standing still, a hold
 * (phase Hold: waiting on a lift permit, a re-plan or the terrain) and a
 * fault ever bring it to a standstill.
 *
 * The robot drives up to the curb in its drive mode's stance and is levelled
 * in from it -- every leg moved to its level target on an S-curve within
 * LegSpeed / LegAcceleration and the base's turn within TiltRateDegPerSec /
 * TiltAccelDegPerSec2 -- timed so that the base is level before the first
 * wheel is near its edge (made standing still first where, on the way, it
 * would have to be made under the floor speed). Rolling, a plan still being
 * made slows the robot so that its level-in keeps room; a level-in that runs
 * late slows it further rather than faulting. Once every wheel is on the far
 * side the robot rolls on until its rear tyres are FinishClearance past the
 * edge (Settle) and the crossing is Done, with the base back to level.
 *
 * The per-axle gait (Tuning.Gait PerAxle), kept as an option: both wheels of
 * an axle cross together, one axle at a time, front to rear -- Prepare (every
 * wheel down; the chassis to a height that lets this axle clear the step
 * while the other two reach their ground, the centre wheels shifted fore/aft
 * to keep the centre of mass between the axles that will carry it), Lift,
 * Advance (driving on the other two to the landing spot) and Plant -- then
 * Settle. Its chassis heights and centre shifts are planned from the terrain
 * and what each leg can reach; where no one height serves a step it lands an
 * axle further on, or follows a path through the corridor of heights every
 * leg and the belly allow (at least Tuning.MinCorridorCm wide and no steeper
 * than BaseHeightSpeed). It plans in the call, keeps the base level (never
 * tilts or plans a stop), and paces itself by the next axle's deadline the
 * same way; a step ends when the legs are measured where they were sent.
 *
 * The base is held level against gravity in both gaits. Every leg that
 * carries the robot takes its height from the plane under its own wheel --
 * the near surface until the wheel crosses, the far one after -- where the
 * wheel will be a servo lag from now, so sloped and twisted surfaces move the
 * legs, not the chassis. Lifted wheels are carried clear of the terrain on
 * their way. A gentle attitude loop (Tuning.AttitudeProportionalGain,
 * AttitudeIntegralGain) trims what is left as a pure rotation of the support
 * legs, and each surface's height is corrected by where its planted wheels
 * find it.
 *
 * Every plan keeps the robot standing well inside its wheels. The robot's
 * centre of mass (Tuning.CenterOfMass, moved by the mass each leg carries with
 * its wheel, FRammsCurbLeg::MovingMassKg against Tuning.RobotMassKg) must stay
 * at least Tuning.MinStabilityMargin inside the contacts of the wheels on the
 * ground in every pose of the plan, allowing for the shift of braking at
 * Deceleration or speeding up at Acceleration (h a / g, its height h over the
 * contacts): a two-dimensional margin, so a lift is checked across the robot
 * as well as along it. Each wheel touches where its tread meets the surface:
 * on a slope across it, at its uphill rim. The same margin is measured on
 * every tick of a crossing (GetStabilityMargin, and the run stats); the
 * per-wheel gait holds, then faults, under MinLiveStabilityMarginCm, and
 * checks the stance it drives up in against it before it begins.
 *
 * The robot drives as it does in differential mode, on its main drive wheels
 * (FRammsCurbLeg::Drive Main), with each other wheel driving or rolling free
 * by its role, side by side: a Fallback wheel takes over only while the Main
 * wheel on its side is off the ground, an Assist wheel helps throughout, a
 * Passive one only rolls. Each
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
 * and refuses if it still has not seen enough. The per-wheel gait crosses an
 * edge up to Tuning.MaxApproachDeg off square; the per-axle gait lifts both
 * wheels of an axle at once, so it crosses one at most
 * RammsCurb::PerAxleMaxApproachDeg off square. It refuses a start with any
 * wheel over ground more than Tuning.MinStepHeight off the near surface.
 *
 * The rider drives the per-wheel crossing with the stick (Stick, on by
 * default): while it runs it takes the drive controls every drive mode
 * offers (drive.forward, drive.turn, drive.strafe), and it moves only while
 * the stick is held forward -- held part way, at that share of the plan's
 * speed. Let go, or pushed any other way, it pauses (phase Paused): braked
 * to rest at Tuning.Deceleration, the rate every planned pose keeps its
 * margin under, so that it can stop anywhere; no new lift or level-in
 * begins, those under way finish (a lift to clear, a landing to loaded),
 * the legs then hold and, once nothing moves, the wheels are held, braked
 * where they have brakes -- for as long as it takes: a pause is no hold and
 * times nothing out. The stick held forward again goes on from there.
 * Before any leg has moved for the curb, pulled back or pushed to the side
 * cancels the crossing; levelled in with no wheel yet lifted, held back
 * for Stick.CancelHoldSeconds, the robot at rest, does -- either way
 * handing the robot back to the drive mode it came from (phase Aborted),
 * which takes the legs back to its stance. Once a wheel has begun to
 * cross, nothing but forward moves the robot: the plan is laid along one
 * heading and cannot be backed out of. At Done the stick carries on into
 * the drive mode handed back to. The per-axle gait does not read the
 * stick.
 *
 * It is a drive mode ("curb"): while it runs, the other modes stand down and
 * it owns the wheels and cranks. Starting a manoeuvre selects it through the
 * robot's URammsDriveModeSelector -- on the move too: the crossing carries on
 * at the speed the robot has -- and finishing one hands back to the mode that
 * was active without stopping: the robot rolls on, the mode it returns to is
 * handed its motion (IRammsDriveMode::SeedMotion), and the selector ramps the
 * legs from where the crossing left them to that mode's stance, no faster
 * than the base may turn to the attitude that stance takes on the ground
 * (GetStanceChangeMinSeconds: within Tuning.TiltRateDegPerSec and
 * TiltAccelDegPerSec2). On ground sloping more than a degree it hands back
 * only to a mode that holds the robot still with no command
 * (IRammsDriveMode::HoldsRobotStill) -- another such mode if the one it came
 * from does not, else it keeps the robot here, its wheels held. One
 * stopped by Abort, or by a tilt, step-timeout, stall or measurement fault, keeps
 * the robot held in this mode -- per wheel, braked to rest at
 * Tuning.Deceleration (twice that where a wheel is about to meet the curb)
 * rather than at once, the legs holding their targets, then the wheels
 * braked: each held at the angle it stopped at, its speed loop asked back
 * to it, so that on a slope the robot stays put; per axle, braked at once; a
 * retry that succeeds still hands back to the mode from before. A simulation
 * reset faults it too, but hands back as well: the robot is at its start
 * pose, with nothing across a step to hold.
 *
 * The per-wheel gait checks itself as it runs, every tick (see the executor's
 * runtime checks, Private/Curb): it traces the terrain ahead of and under
 * every wheel against the planes it planned on (Tuning.TerrainMismatchCm),
 * checks each landing (LandingToleranceCm), the base against its planned
 * attitude (AttitudeFaultDeg), the robot's measured stability margin
 * (MinLiveStabilityMarginCm: a hold, then a fault), each clear wheel's rim
 * against the traced terrain, each leg against its servo lag model and each
 * lift, landing and level-in against its planned time; and before any wheel
 * has begun to cross it re-plans when the terrain, the heading or a wheel's
 * window moved from the plan. Each fault names the check, the wheel and the
 * numbers (GetStatusText).
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
class RAMMSCORE_API URammsCurbNegotiationController : public UActorComponent, public IRammsControlContributor, public IRammsDriveMode, public IRammsHUDPanelProvider
{
	GENERATED_BODY()

public:
	URammsCurbNegotiationController();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	/** Every leg the gait moves: two per axle. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	TArray<FRammsCurbLeg> Legs;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FRammsCurbTuning Tuning;

	/** The seat the crossing moves, where the robot has one (FRammsCurbSeat):
	 *  its pivot shifted fore and aft for the centre of mass, its platform
	 *  pitched against the base's to keep the rider level. Unset: no seat. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FRammsCurbSeat Seat;

	/** How the rider's stick drives a per-wheel crossing (FRammsCurbStick):
	 *  with Stick.bRequired, it moves only while the stick is held forward. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FRammsCurbStick Stick;

	/** Drive mode to hand back to when done. None returns to whichever mode
	 *  was active when the manoeuvre began. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FName ReturnModeId;

	/** What the debug view draws: the terrain sensed and fitted, the plan, the
	 *  robot's state, graphs (FRammsCurbDebugView). Also switched from the
	 *  robot's control panel, the "Curb view" group. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Debug")
	FRammsCurbDebugView DebugView;

	/**
	 * Plan the manoeuvre for this step without moving: true when every leg
	 * can reach what each step of the gait asks of it. OutReason says why
	 * not, or summarises the plan. With Tuning.Gait PerWheel (the default)
	 * the plan is made in the call, on the game thread, which stalls for as
	 * long as it takes: tens to a few hundred milliseconds for a level plan,
	 * several seconds -- ten or more on a hard case -- where it has to look
	 * for a tilt or a stop (its lines say how long). Ask it of a robot
	 * standing still, and not every frame; BeginNegotiation plans off the
	 * game thread.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	bool CheckFeasibility(FRammsCurbProfile Profile, FString& OutReason);

	/**
	 * Plan and start. False, with a reason, when the crossing cannot start
	 * from here before any plan is made -- the robot cannot be measured or is
	 * not on the near side, the active drive mode is moving it sideways (see
	 * GetCarriableMotion), the profile is not one this gait takes -- and with
	 * the per-axle gait (Tuning.Gait PerAxle), whose plan is made in the call,
	 * when that plan refuses; nothing moves then. The per-wheel gait plans off
	 * the game thread (a plan takes tens of milliseconds to a few seconds),
	 * at rest or on the move: true means the crossing has begun with its plan
	 * being made (phase Approach), the robot held in its stance -- at rest,
	 * every wheel held where it stands; on the move, rolling on, braked short
	 * of where its level-in would have to begin -- and a refusal by the plan
	 * comes on a later tick: the phase Faulted, the status "refused at rest:
	 * <why>" (the robot handed back to the mode it came from: nothing has
	 * moved) or "refused on the move: <why>" (braked to rest and held in this
	 * mode). CheckFeasibility answers in the call instead (and stalls the
	 * game thread while it plans).
	 */
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

	/**
	 * Development and test only (not in shipping builds): BeginNegotiation on
	 * the terrain given rather than one detected -- for fault injection, a
	 * terrain off from the world's, to see the crossing's runtime checks find
	 * it. The crossing runs as on a detected terrain: the approach looks again
	 * as it closes in, and the terrain monitor traces ahead of every wheel.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb", meta = (DevelopmentOnly))
	bool BeginNegotiationOnTerrain(FRammsCurbTerrain Terrain, FString& OutReason);

	/** Stop driving and hold the legs where they are sent: the per-wheel gait
	 *  braked to rest at Tuning.Deceleration (twice that where a wheel is
	 *  about to meet the curb), the per-axle gait at once. Called again while
	 *  the per-wheel gait is braking to rest (after an abort or a fault),
	 *  every wheel is braked at once. */
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

	/** The per-wheel gait's plan (Tuning.Gait PerWheel), one line per event in
	 *  the order of the chassis' travel -- each wheel's lift and landing, the
	 *  level-in, any planned tilt or stop, with the chassis height, the base's
	 *  attitude, the centre wheels' fore/aft and the speed there -- then a
	 *  summary: the plan's class (level, tilt, planned stop) and its floor
	 *  speed, least margin, narrowest corridor and timing, or why it refused.
	 *  Empty before any per-wheel plan. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	TArray<FString> GetScheduleLines() const;

	/** The base tilt (deg; X pitch nose up, Y roll +Y side up) the last
	 *  per-wheel plan calls for at its largest, each axis signed as planned:
	 *  0 when it plans the base level, and for the per-axle gait. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	FVector2D GetPlannedTiltDeg() const;

	// --- IRammsDriveMode ------------------------------------------------------
	virtual FName GetDriveModeId() const override { return FName("curb"); }
	virtual FText GetDriveModeDisplayName() const override
	{
		return NSLOCTEXT("Ramms", "DriveModeCurb", "Curb");
	}
	virtual bool IsDriveModeActive() const override { return bDriveModeActive; }
	virtual void SetDriveModeActive(bool bActive) override;
	/** Its forward speed and any yaw rate, no sideways speed: a crossing
	 *  drives on the main wheels, which cannot move sideways. A crossing asked
	 *  for while the active drive mode moves the robot sideways faster than
	 *  the selector's HandOverMaxSidewaysSpeed is refused, and a mode handed
	 *  back to is held to this until its stance is in place. */
	virtual FRammsDriveMotion GetCarriableMotion(const FRammsDriveMotion& Motion) const override;
	/**
	 * How long the selector's ramp from where a crossing left the legs to the
	 * next mode's stance should take, at least, so that the base turns from
	 * its attitude now to the one that stance takes on the ground under it no
	 * faster than Tuning.TiltRateDegPerSec and TiltAccelDegPerSec2: that
	 * attitude is the ground's plus the stance's own tilt over the ground it
	 * stood on as the crossing began. 0 before any crossing.
	 */
	virtual float GetStanceChangeMinSeconds() const override;

	// --- IRammsControlContributor --------------------------------------------
	virtual void DescribeControls(FRammsControlSurface& OutSurface) const override;
	/** The debug view's switches are writable (the "Curb view" group), and
	 *  while a crossing has the robot with Stick.bRequired, the drive controls
	 *  (the stick; released, at rest). */
	virtual bool  ApplyControl(FName Id, float Value) override;
	virtual bool  TriggerControl(FName Id) override;
	virtual bool  ReleaseControl(FName Id) override;
	virtual bool  ReadControl(FName Id, float& OutValue) const override;
	virtual bool  ReadTarget(FName Id, float& OutTarget) const override;
	virtual void  GetClaimedMotorIds(TArray<FName>& OutIds) const override;
	virtual int32 GetControlOrder() const override { return 5; }

	/** Control Ids this contributes. */
	static FName NegotiateControlId() { return FName(TEXT("curb.negotiate")); }
	static FName AbortControlId() { return FName(TEXT("curb.abort")); }
	static FName PhaseControlId() { return FName(TEXT("curb.phase")); }
	/** The debug view's switches (DebugView): Sensing takes 0 Off, 1 Live,
	 *  2 Live + trail; the others 0 off, 1 on. */
	static FName ViewSensingControlId() { return FName(TEXT("curb.view.sensing")); }
	static FName ViewTerrainControlId() { return FName(TEXT("curb.view.terrain")); }
	static FName ViewPlanControlId() { return FName(TEXT("curb.view.plan")); }
	static FName ViewRobotControlId() { return FName(TEXT("curb.view.robot")); }
	static FName ViewGraphsControlId() { return FName(TEXT("curb.view.graphs")); }
	static FName ViewLabelsControlId() { return FName(TEXT("curb.view.labels")); }

	// --- IRammsHUDPanelProvider ------------------------------------------------
	/** The debug view's colour key and graphs as a HUD panel ("Curb view"),
	 *  wanted while there is something to show; with no HUD to host it they
	 *  are drawn over the viewport instead. */
	virtual void GetHUDPanels(TArray<FRammsHUDPanelDesc>& OutPanels) const override;

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

	/** The per-wheel gait (Tuning.Gait PerWheel): measure the robot, check it
	 *  can start here, and plan the crossing on the profile's terrain (or the
	 *  level ground and square edge a profile without one stands for) with
	 *  RammsCurb::PlanCrossing. The plan is kept (GetScheduleLines); nothing
	 *  moves. */
	bool PlanPerWheel(const FRammsCurbProfile& Profile, FString& OutReason);

	/** PlanPerWheel's checks and the planner's input, without planning. */
	bool PreparePerWheel(const FRammsCurbProfile& Profile, RammsCurb::FPlannerInput& OutInput, FString& OutReason);

	/** A per-wheel re-plan: the same input, planned off the game thread
	 *  (RammsCurb::FPlanTask), the schedule swapped in by PollReplan when it is
	 *  done. A plan takes longer than a tick; a re-plan must not stall one.
	 *  Only between crossings: it measures the robot as PlanPerWheel does,
	 *  which resets what a crossing under way keeps. */
	bool StartReplan(const FRammsCurbProfile& Profile, FString& OutReason);

	/** Take a finished re-plan (each tick): adopted while the gait is PerWheel,
	 *  no crossing runs and no plan has been made since it was launched;
	 *  otherwise dropped. */
	void PollReplan();

	/** The schedule's lines and figures into what the controller reports. */
	void AdoptSchedule();

	/** Each 5-bar leg's knee-limited reach on a grid, from its controller's
	 *  SolveTarget: built once per set of legs (RammsCurb::FFiveBarGridCache). */
	void EnsureFiveBarGrids();

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

	/** True, with why, when the active drive mode is moving the robot sideways
	 *  (as the selector measures it) more than this mode carries: a crossing
	 *  would put the main wheels down sliding. */
	bool MovingSidewaysIntoCrossing(FString& OutWhy) const;

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
	void CommandLegs(float RampStep);
	/** Drive the wheels at Speed (cm/s) and YawRate (rad/s); the carrying
	 *  driven wheels Trim (cm/s) faster (the per-wheel gait's speed trim); the
	 *  chassis Lateral (cm/s) towards its +Y, through the wheels that roll
	 *  partly sideways. */
	void CommandWheels(float Speed, float YawRate, float Trim = 0.0f, float Lateral = 0.0f);
	/** The per-wheel crossing's speed trim and cross-track steer, one tick
	 *  (SpeedTrim, CrossTrackCm, CrossTrackSteerRad); the yaw rate to drive
	 *  at. TrimCapCmS: the hard braking cap binding now (cm/s), which the
	 *  speed trim never takes the wheels past. */
	float PerWheelSteering(float DeltaTime, double TrimCapCmS);
	void  StopWheels(bool bRelease);
	/** The crossing ends: Done hands back; a fault or an abort brakes the robot
	 *  to rest first (bRampDown: StepStopping, the per-wheel executor halted)
	 *  -- not a simulation reset or another mode selected, which let go at
	 *  once. */
	void Finish(ERammsCurbPhase EndPhase, const FString& Why, bool bRampDown = true);
	/** The rider's stick, one tick of a per-wheel crossing (Stick): the share
	 *  of the plan's speed it asks (OutScale) and whether it pauses the
	 *  crossing. False when it has cancelled the crossing (handed back). */
	bool StepStick(float DeltaTime, double& OutScale, bool& bOutPaused);
	/** The rider has cancelled the crossing with the stick: handed back to
	 *  the drive mode it came from. */
	void CancelByRider(const FString& Why);
	/** After a fault or an abort: one tick of braking the robot to rest, the
	 *  legs holding their targets; the wheels braked once it is at rest. */
	void  StepStopping(float DeltaTime);
	void  EndStopping(const FString& Why);
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
	/** The base's pitch filtered slower, for the seat's tilt: it compensates
	 *  a pitch held, not a bump's brief dip (SeatPitchFilterSeconds). */
	float SeatPitchRad = 0.0f;
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
	/** The crossing ending now hands the robot back, as Done does (a refusal
	 *  at rest: nothing moved). */
	bool bHandBackOnFinish = false;
	/** Selected without a manoeuvre to run (picked from the mode list): hand
	 *  the robot back on the next tick, outside the selector's switch. */
	bool bHandBackPending = false;

	/** The wheels held where they stand (StopWheels without letting go): each
	 *  wheel's speed loop asked back towards the angle it was latched at
	 *  (rad, the motor's sense), every tick, until the wheels are driven or
	 *  let go. */
	bool		   bHoldingWheels = false;
	TArray<double> WheelHoldAngles;
	/** The wheels the hold has braked (their parking brakes,
	 *  URammsRobotBaseComponent::SetMotorBrake) once the robot came to rest
	 *  under it; their speed loops are off while they are. Opened when the
	 *  wheels are driven or let go, or when they slip. */
	TArray<FName> BrakedWheels;
	/** The physics clock at the last tick the wheels were held with no
	 *  crossing running (-1 otherwise): a reset is seen there too. */
	double IdleHoldSimTime = -1.0;
	/** Latch every wheel where it stands and hold it (bHoldingWheels). */
	void HoldWheels();
	/** One tick of holding the wheels latched: on their brakes once the robot
	 *  is at rest, where they have them (RammsDrive::BrakeAtRestCmS), each
	 *  other wheel's speed loop asked back towards its angle. */
	void StepWheelHold();
	/** Open the brakes the hold closed (BrakedWheels). */
	void ReleaseWheelBrakes();
	/** Whether the drive mode ModeId holds the robot still with no command
	 *  (IRammsDriveMode::HoldsRobotStill); and the first such mode, or None. */
	bool  ModeHoldsStill(FName ModeId) const;
	FName HoldingModeId() const;
	/** The drive mode ModeId on this actor, or null. */
	IRammsDriveMode* FindDriveMode(FName ModeId) const;
	/** The mode a crossing that has ended hands the robot back to: the one it
	 *  came from (or ReturnModeId), as long as the robot stands on ground
	 *  sloping no more than HandBackLevelDeg or that mode holds it still;
	 *  else a mode that does; else None -- it stays here, its wheels held. */
	FName HandBackModeId(FString& OutNote) const;

	/** The base's attitude (deg: pitch, roll) over the ground it stood on as
	 *  the crossing began -- its drive stance's own tilt -- for the hand-back's
	 *  stance ramp (GetStanceChangeMinSeconds). */
	FVector2D StanceAttitudeOverGround = FVector2D::ZeroVector;
	bool	  bStanceAttitudeKnown = false;
	/** The attitude (deg: pitch nose up, roll +Y side up) a stance standing
	 *  on Plane takes, the chassis facing as it does now. */
	FVector2D GroundAttitudeDeg(const FRammsCurbPlane& Plane) const;

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

	// --- the debug view (DebugView; RammsCurbNegotiationDebug.cpp) ---------------

	/** What the view keeps between frames: the last detection scan's traces
	 *  (mutable: a detection is const and records them as it makes them), the
	 *  terrain monitor's probes and their trail, the graphs' samples
	 *  (Private/Curb). */
	mutable TPimplPtr<RammsCurb::FDebugViewState> DebugState;
	/** The graphs' and colour key's draw over the viewport (UDebugDrawService). */
	FDelegateHandle DebugDrawHandle;

	/** Once a tick, whatever path the tick took: the preview scan, the graphs'
	 *  sample, and the world layers drawn. */
	void TickDebugView(float DeltaTime);
	/** The terrain monitor's probes of this tick, for the sensing layer. */
	void RecordDebugProbes(const RammsCurb::FExecMeasure& Measure);
	/** The layers, in the world. */
	void DrawDebugSensing(const UWorld* World) const;
	void DrawDebugTerrain(const UWorld* World) const;
	void DrawDebugPlan(const UWorld* World) const;
	void DrawDebugRobot(const UWorld* World) const;
	/** The world layers' labels over the viewport, and the colour key and the
	 *  graphs too while no HUD hosts them as a panel. */
	void DrawDebugOverlay(UCanvas* Canvas, APlayerController* Player);
	/** The colour key and the graphs, Width wide (Height tall, or as tall as
	 *  they need when 0), from the painter's origin down; their height. */
	float PaintDebugPanel(const RammsCurb::FDebugPainter& Painter, float Width, float Height, bool bDraw) const;
	/** A layer drawn in the world wants its colour key. */
	bool WantsDebugKey() const;
	/** A HUD hosts the key and graphs panel (its widget is alive). */
	bool IsDebugPanelHosted() const;
	/** That panel's widget, while a HUD holds it. */
	TWeakPtr<SWidget> DebugPanelWidget;
	friend class SRammsCurbScope;

	/** The world layers' labels, on the player's screen under the HUD's panels
	 *  (the debug canvas would draw them over the panels): a Slate overlay,
	 *  there while labels are; the viewport canvas without a game viewport. */
	void UpdateDebugLabelOverlay(bool bWanted);
	/** The labels, each at its point on screen (Project: world to the
	 *  painter's frame; false when off screen). */
	void								PaintDebugLabels(const RammsCurb::FDebugPainter& Painter, TFunctionRef<bool(const FVector&, FVector2D&)> Project) const;
	TSharedPtr<SWidget>					DebugLabelOverlay;
	TWeakObjectPtr<UGameViewportClient> DebugLabelViewport;
	TWeakObjectPtr<ULocalPlayer>		DebugLabelPlayer;
	friend class SRammsCurbLabels;
	/** The terrain the view shows: the crossing's while one runs or after, else
	 *  the last scan's. */
	const FRammsCurbTerrain* DebugTerrain() const;
	/** The chassis' pitch and roll (deg) from its corner pivots, crossing or
	 *  not; false when they cannot be located. */
	bool MeasureDebugAttitude(FVector2D& OutPitchRoll) const;

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

	/** The seat as resolved (ResolveSeat) and commanded (CommandSeat). */
	struct FSeatState
	{
		bool bResolved = false;
		/** The pivot at the zero pose and arm A's distal link angle there (the
		 *  platform level), the height the crossing holds the pivot at and the
		 *  fore/aft it can be held at there. */
		FVector2D ZeroPivot = FVector2D::ZeroVector;
		double	  LinkAngle0 = 0.0;
		double	  HeightCm = 0.0;
		FVector2D XRange = FVector2D(1.0, 0.0);
		/** The 5-bar motor angles last commanded: the solver's hint. */
		FVector2D CmdAngles = FVector2D::ZeroVector;
		bool	  bCmd = false;
		/** The platform's pitch in the world (rad) as the crossing took the
		 *  seat over, which the tilt holds against the base's pitch. */
		double PitchRef = 0.0;
	};
	FSeatState SeatState;

	/** Resolve the seat -- its motors in the registry, its zero pose, its
	 *  reach at its crossing height -- into SeatState and the model's seat; a
	 *  robot without one resolves to none. */
	bool ResolveSeat(FString& OutReason);
	/** The seat as measured now: its motors read through the kinematics. */
	void MeasuredSeat(RammsCurb::FMeasuredSeat& Out) const;
	/** Hold the seat's pivot at fore/aft X (cm, chassis frame) at its
	 *  crossing height, the platform pitched against the base's (filtered)
	 *  pitch to keep the rider level; nothing on a robot without a seat. */
	void CommandSeat(double X);

	/** The last per-wheel plan (Private/Curb). */
	TPimplPtr<RammsCurb::FCurbSchedule> Schedule;

	/** The 5-bar legs' reach grids the per-wheel planner uses (Private/Curb). */
	TPimplPtr<RammsCurb::FFiveBarGridCache> FiveBarGrids;

	/** A per-wheel re-plan under way off the game thread (Private/Curb). */
	TPimplPtr<RammsCurb::FPlanTask> ReplanTask;

	/** Every plan made (either gait) or begun takes the next number; a re-plan
	 *  is adopted only if no other came after the one it was launched as. */
	int32 PlanGeneration = 0;
	int32 ReplanGeneration = INDEX_NONE;

	/** Forget the per-wheel plan and any re-plan under way (a per-axle plan or
	 *  crossing replaces it). */
	void DropPerWheelPlan();

	// --- the per-wheel crossing (Tuning.Gait PerWheel) ---------------------------

	/** The per-wheel gait's executor (Private/Curb), and the input its plan was
	 *  made from. */
	TPimplPtr<RammsCurb::FExecutor>		Executor;
	TPimplPtr<RammsCurb::FPlannerInput> CrossingInput;
	/** The input of a re-plan under way (bReplanning), and the terrain it was
	 *  launched on. */
	TPimplPtr<RammsCurb::FPlannerInput> ReplanInput;
	FRammsCurbTerrain					ReplanTerrain;

	/** The crossing under way (or the last) runs the per-wheel gait. */
	bool bPerWheelCrossing = false;
	/** Its plan is being made off the game thread: begun on the move, the
	 *  robot holds its speed and stance until it comes (braked short of every
	 *  wheel's window and of the line its level-in must be done by); begun at
	 *  rest (bPlanningAtRest), it is held where it stands. */
	bool bAwaitingPlan = false;
	bool bPlanningAtRest = false;
	/** A re-plan on the approach (the terrain looked at again moved) is under
	 *  way off the game thread. */
	bool bReplanning = false;
	/** Seconds the crossing has been in Hold, and the executor's holds and
	 *  planned stops already counted in the run stats. */
	double HoldSeconds = 0.0;
	/** The rider's stick as the drive controls last set it (X forward, Y
	 *  turn, Z strafe; each -1..1); the crossing's seconds it has held the
	 *  crossing paused (left out of the approach's and the crossing's
	 *  timeouts), and how long the robot has stood paused with nothing moving
	 *  (its wheels held from RammsCurb::PauseHoldSeconds on); why it pauses. */
	FVector StickInput = FVector::ZeroVector;
	double	StickPausedSeconds = 0.0;
	double	PausedStillSeconds = 0.0;
	FString StickWhy;
	/** Seconds the chassis has run away: RunawayCmS faster than its command as
	 *  the chassis can follow it (RunawayCommand: the command through
	 *  RunawayLagSeconds), or rolling back faster than RollBackCmS. */
	double RunawaySeconds = 0.0;
	double RunawayCommand = 0.0;
	/** The per-wheel crossing's speed trim (cm/s): an integral of the chassis
	 *  speed's shortfall against its command (as the chassis can follow it),
	 *  added to what the carrying driven wheels are asked -- the wheels' own
	 *  speed loops hold no load without an error, which on a grade is the
	 *  robot rolling away (RammsCurb::SpeedTrimStep). */
	double SpeedTrim = 0.0;
	/** The per-wheel crossing holding the robot still (the executor's command
	 *  and target at rest: a level-in standing still, a planned stop, a hold):
	 *  the chassis held where it was when that began (world xy, the measured
	 *  crank pivots' midpoint), RammsCurb::RestHoldSpeed. */
	bool	  bRestHold = false;
	FVector2D RestHoldAt = FVector2D::ZeroVector;
	/** The command as the measured speed follows it (two lag stages), started
	 *  at the chassis' signed measured speed. */
	double TrimFollowA = 0.0;
	double TrimFollowB = 0.0;
	/** The most the speed trim reached either way (cm/s); the most the chassis
	 *  drifted off its line either way (cm). Logged. */
	double MaxAbsSpeedTrim = 0.0;
	double MaxAbsCrossTrackCm = 0.0;
	/** The chassis' sideways distance (cm, towards its +Y) from the line the
	 *  per-wheel crossing was planned along, and the heading correction it
	 *  asks (rad, the sense of HeadingErrorRad). */
	double CrossTrackCm = 0.0;
	double CrossTrackSteerRad = 0.0;
	/** The chassis origin (world xy) where the crossing began: the line the
	 *  cross-track correction holds runs from it along Heading. */
	FVector2D CrossTrackOrigin = FVector2D::ZeroVector;
	/** The sideways speed asked (cm/s, towards +Y) and its integral (cm/s);
	 *  whether carrying driven wheels on both sides roll partly sideways (omni
	 *  hubs turned more than LateralAuthorityShare off the heading), as of the
	 *  last wheel command -- through one side alone the sideways speed would
	 *  turn the robot, so it is asked only then. */
	double					LateralCmd = 0.0;
	double					LateralTrim = 0.0;
	bool					bLateralDriven = false;
	static constexpr double LateralAuthorityShare = 0.2;
	/** The surface under the chassis slopes (any way) more than
	 *  RammsCurb::SpeedTrimGradeDeg: the fallback wheels drive too whenever they
	 *  carry. */
	bool  bSlopedDrive = false;
	int32 CountedHolds = 0;
	int32 CountedPlannedStops = 0;
	/** While the far side is unconfirmed: where (chassis travel along the plan,
	 *  cm) the robot must come to rest -- short of the first lift. */
	double PerWheelCommitS = 0.0;
	/** A fault or an abort is braking the robot to rest (StepStopping), and for
	 *  how long (simulated s). */
	bool   bStopping = false;
	double StopElapsed = 0.0;
	/** Looks again on the approach in a row that found a surface further off
	 *  the one the crossing runs on than TerrainMismatchCm (thrown away), and
	 *  the largest of them (cm): enough of them, and the terrain is not as
	 *  detected. */
	int32  LookMismatches = 0;
	double LookMismatchCm = 0.0;
	/** What a runtime trigger's re-plan under way changed as it was launched:
	 *  the re-fit (cm; X near, Y far) moved into ActiveTerrain -- once the
	 *  re-plan is adopted, the re-fitted planes' biases start again --, and
	 *  whether the heading held (and the cross-track line's origin) became
	 *  the chassis' own, and what they were. All of it undone if the re-plan
	 *  comes too late to be adopted. */
	FVector2D PendingRefitCm = FVector2D::ZeroVector;
	bool	  bPendingHeading = false;
	FVector	  PendingHeadingWas = FVector::ForwardVector;
	FVector2D PendingCrossTrackOriginWas = FVector2D::ZeroVector;

	/** BeginNegotiation with the per-wheel gait: planned off the game thread,
	 *  a robot at rest held where it stands meanwhile (bPlanningAtRest), one on
	 *  the move (or rolling back) rolling on in its stance; the plan's refusal
	 *  comes on a later tick (TakeCrossingPlan). The plan's own fallback chain
	 *  decides what runs: level, else its least tilt within MaxPlannedTiltDeg,
	 *  else a level planned stop, else a refusal. */
	bool BeginPerWheel(const FRammsCurbProfile& Profile, FString& OutReason);
	/** The per-axle crossing (BuildPlan done): take the robot over and start. */
	void StartPerAxleCrossing(const FRammsCurbProfile& Profile);
	/** What every crossing does as it starts: the drive mode, the clocks, the
	 *  run stats (with the plan's figures), the looks again. */
	void StartCrossingCommon(const FRammsCurbProfile& Profile, float PlannedMinSpeedCmS, float PlannedTiltDeg,
		float PlannedSeconds);
	/** The per-wheel crossing on *Schedule, made from Input: the executor
	 *  begins from the robot as it is measured. */
	bool StartPerWheelRun(const RammsCurb::FPlannerInput& Input, FString& OutReason);
	/** The planner's input from the robot as measured now and ActiveTerrain
	 *  (PreparePerWheel's last part, without its checks and resets). */
	void FillPlannerInput(RammsCurb::FPlannerInput& Out);
	/** One tick of the per-wheel crossing. */
	void StepPerWheel(float DeltaTime);
	/** The executor's measurement from the controller's. */
	void GatherExecMeasure(float DeltaTime, RammsCurb::FExecMeasure& Out) const;
	/** While the plan of a crossing begun on the move is being made: each
	 *  wheel's braking cap short of its window (cm/s); and, where asked, the
	 *  room (cm, along the heading) to the line the level-in must be done by. */
	float AwaitingBrakingCap(double* OutLevelRoomCm = nullptr) const;
	/** The per-wheel crossing's plan arrived off the game thread: run it, or
	 *  end with the plan's refusal (Faulted: at rest handing the robot back,
	 *  on the move braking it to rest). */
	void TakeCrossingPlan();
	/** The per-wheel crossing's re-plan arrived: adopt it if no wheel has begun
	 *  to cross. */
	void TakePerWheelReplan();
	/** Launch a re-plan of the per-wheel crossing on ActiveTerrain. */
	void LaunchPerWheelReplan(const FString& Why);
	/** Send each leg the executor's command (a 5-bar's partial move taken back
	 *  to the executor). */
	void CommandPerWheelLegs(const TArray<double>& LegZ, const TArray<double>& LegX);
};
