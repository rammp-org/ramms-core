// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Ramms4BarLegSpec.h"
#include "Ramms5BarLinkageSpec.h"
#include "RammsCurbTerrain.h"
#include "RammsHolonomicDriveTypes.h"
#include "RammsCurbNegotiationTypes.generated.h"

/**
 * Which wheel pair a leg belongs to, front to back.
 *
 * The robot has three axles. The per-wheel gait (the default) lifts each
 * wheel over the step on its own, as its own distance to the edge comes up;
 * the per-axle gait lifts one whole axle at a time, front to rear, while the
 * other two carry the robot.
 */
UENUM(BlueprintType)
enum class ERammsCurbAxle : uint8
{
	Front  UMETA(DisplayName = "Front"),
	Center UMETA(DisplayName = "Center"),
	Rear   UMETA(DisplayName = "Rear"),
};

/**
 * When a leg's wheel is driven during a crossing.
 *
 * A crossing drives on the main wheels and leans on the others only when it
 * has to. Which others, and when, is set per leg: a fallback wheel takes over
 * only while the main wheel on its side is off the ground, an assist wheel
 * helps throughout, and a passive wheel is never driven.
 */
UENUM(BlueprintType)
enum class ERammsCurbWheelDrive : uint8
{
	/** A main drive wheel: drives while it is planted, rolls free while it is
	 *  lifted. */
	Main UMETA(DisplayName = "Main"),
	/** Drives throughout the crossing, alongside the main wheels and in their
	 *  place while they are lifted. */
	Assist UMETA(DisplayName = "Assist"),
	/** Drives only while a main wheel on its own side of the robot is off the
	 *  ground (or its side has none); rolls free otherwise. */
	Fallback UMETA(DisplayName = "Fallback"),
	/** Never driven: rolls free throughout. */
	Passive UMETA(DisplayName = "Passive"),
};

/** How a leg sets its wheel's height. */
UENUM(BlueprintType)
enum class ERammsCurbLegKind : uint8
{
	/** A crank-rocker leg driven by one crank motor (FRamms4BarLegSpec). */
	FourBar UMETA(DisplayName = "4-bar (crank)"),
	/** A 5-bar leg, through its URamms5BarLinkageController -- which also
	 *  lets the wheel move fore/aft, the way the centre wheels shift weight. */
	FiveBar UMETA(DisplayName = "5-bar"),
};

/**
 * One leg and the wheel on the end of it.
 *
 * Heights are wheel-centre heights in the chassis frame (cm, up positive), the
 * same frame both linkage specs use, so a support leg whose ground is at
 * height G under its wheel, with the chassis origin at height B and level,
 * sits at G + rho - B: rho is WheelRadiusCm on level ground, a little more on
 * a slope (along the wheel, and across it by half ContactWidthCm).
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsCurbLeg
{
	GENERATED_BODY()

	/** For logs and readouts. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FName Name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	ERammsCurbAxle Axle = ERammsCurbAxle::Front;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	ERammsCurbLegKind Kind = ERammsCurbLegKind::FourBar;

	/** The leg's geometry and crank motor, for a FourBar leg. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb", meta = (EditCondition = "Kind == ERammsCurbLegKind::FourBar"))
	FRamms4BarLegSpec FourBar;

	/** Name of the URamms5BarLinkageController component that drives a
	 *  FiveBar leg (the Blueprint variable name, e.g. LeftCenterLinkage). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb", meta = (EditCondition = "Kind == ERammsCurbLegKind::FiveBar"))
	FName FiveBarComponent;

	/**
	 * The wheel's drive motor and roll direction. Position.Y (cm) places the
	 * wheel across the chassis, for heading correction and for telling the
	 * sides apart when measuring roll; any lateral axis does as long as every
	 * leg uses the same one (Unreal's +Y is the robot's right), and the corner
	 * legs must include both signs. Position.X is not used -- the leg's
	 * kinematics say where the wheel is fore/aft, and it moves.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FRammsOmniWheelSpec Wheel;

	/** Rolling radius (cm): wheel centre to ground. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb", meta = (ClampMin = "0.1"))
	float WheelRadiusCm = 10.0f;

	/** The tyre's full width across its axle (cm): the footprint a lifted
	 *  wheel must keep clear of the curb, and how wide the step detection's
	 *  scan reaches beyond the outermost wheels. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb", meta = (ClampMin = "0.0"))
	float TyreWidthCm = 6.0f;

	/** The width (cm) over which the tyre sits flat on the ground: how far
	 *  across its axle a sideways slope lifts the wheel centre (half this
	 *  times the slope). 0, the default, means the tyre's whole width
	 *  (TyreWidthCm); set less for a wheel that only touches over part of it,
	 *  such as an omni wheel's rollers. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb", meta = (ClampMin = "0.0"))
	float ContactWidthCm = 0.0f;

	/**
	 * When this wheel is driven; see ERammsCurbWheelDrive. On a robot with no
	 * Main wheel, every wheel that is not Passive drives throughout.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	ERammsCurbWheelDrive Drive = ERammsCurbWheelDrive::Fallback;

	/**
	 * The mass (kg) that moves with this leg's wheel, lumped at its centre:
	 * the wheel and the share of the leg's links that follows it. As the leg
	 * moves its wheel away from where it is at the leg's zero pose (a 4-bar's
	 * crank at 0, a 5-bar's hips at 0), the robot's centre of mass moves by
	 * this over FRammsCurbTuning::RobotMassKg of that. 0, the default, moves
	 * nothing: the centre of mass stays where the tuning puts it.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb", meta = (ClampMin = "0.0"))
	float MovingMassKg = 0.0f;

	/** How fast the leg's crank may turn on average over a move (rad/s),
	 *  for planning how long its moves take; 0, the default, sets no limit.
	 *  Planner only: the per-wheel planner times a move no faster on average
	 *  than this, but the executor still commands the leg by LegSpeed and
	 *  LegAcceleration, so it does not cap the crank's peak rate -- set it
	 *  from what the crank sustains over a whole move. (The per-axle gait
	 *  times its legs by LegSpeed alone.) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb", meta = (ClampMin = "0.0"))
	float MaxCrankRateRadS = 0.0f;
};

/**
 * The seat mechanism, where the robot has one a crossing may move: a planar
 * 5-bar in the chassis' x-z plane (Linkage; its two proximal motors on the
 * chassis) whose shared endpoint carries the seat platform's pivot
 * PivotBeyondEndpointCm further along arm A's distal link, and a motor at
 * that pivot (TiltMotor) that pitches the platform. A crossing shifts the
 * pivot fore and aft to keep the centre of mass inside the wheels on the
 * ground where the legs alone would not -- so wheels the gait otherwise
 * lifts only one after the other may be up together -- and pitches the
 * platform against the base's pitch to keep the rider level. Left unset
 * (no motors), the robot has no seat the crossing moves.
 *
 * Frames and signs are the 5-bar spec's: cm, x forward, z up, link angles
 * from +x toward +z; TiltAngleSign maps the tilt motor's reading to the
 * platform's pitch the way AngleSignA maps motor A's (-1 for a hinge about
 * +Y, +1 about -Y). At the zero pose (every motor at 0) the platform is
 * taken as level over the chassis.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsCurbSeat
{
	GENERATED_BODY()

	/** The 5-bar and its two chassis motors. TranslationSign is +1: the
	 *  linkage's x is the chassis' x. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat")
	FRamms5BarLinkageSpec Linkage;

	/** How far past the shared endpoint, along arm A's distal link, the
	 *  platform's pivot sits (cm; 0: at the endpoint). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat")
	float PivotBeyondEndpointCm = 0.0f;

	/** The motor at the platform's pivot, pitching the platform relative to
	 *  arm A's distal link; none: the platform turns with the link. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat")
	FName TiltMotor;

	/** +1 or -1: the sense of TiltMotor's reading as a platform pitch. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float TiltAngleSign = -1.0f;

	/** The most the platform is pitched relative to the chassis either way
	 *  (deg), keeping the rider level under a planned base pitch. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat", meta = (ClampMin = "0.0", ClampMax = "80.0"))
	float MaxPitchDeg = 50.0f;

	/** The mass that moves with the pivot (kg): the mechanism's own, as a
	 *  lump at the pivot (fit it from how the robot's centre of mass follows
	 *  the pivot, the way a leg's MovingMassKg is). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat|Mass", meta = (ClampMin = "0.0"))
	float MechanismMassKg = 0.0f;

	/** The seat and rider (kg), a lump PayloadOffsetCm from the pivot in the
	 *  platform's frame (x forward, z up; it turns with the platform's
	 *  pitch). Both move the robot's centre of mass against
	 *  FRammsCurbTuning::RobotMassKg, which must include them. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat|Mass", meta = (ClampMin = "0.0"))
	float PayloadMassKg = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat|Mass")
	FVector2D PayloadOffsetCm = FVector2D(0.0, 40.0);

	/** The height (cm, chassis frame) the pivot is held at through a
	 *  crossing while it shifts fore and aft: low keeps the centre of mass
	 *  low. 0, the default, holds the pivot's zero-pose height. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat")
	float HeightCm = 0.0f;

	/** How fast the pivot is shifted fore and aft (cm/s) toward the plan's
	 *  pose; the motors' own profiles (FRammsMotorSpec::MaxVelocity) still
	 *  bound it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat", meta = (ClampMin = "0.0"))
	float ShiftSpeedCmS = 30.0f;

	/**
	 * While the seat shifts aft against the chassis -- as it does once a
	 * crossing is over, back to its drive stance -- it shifts no faster than
	 * this (cm/s), and the drive speeds up by as much as it is shifting, so the
	 * rider's own ground speed (the base's less the seat's) holds at the
	 * planned speed rather than falling by the seat's shift. 0, the default:
	 * the seat shifts aft at ShiftSpeedCmS and the drive keeps the planned
	 * speed.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat", meta = (ClampMin = "0.0"))
	float AftSpeedUpCmS = 0.0f;

	/**
	 * While the seat shifts forward against the chassis -- as it does once a
	 * crossing's fronts are across -- its shift adds to the rider's ground
	 * speed. Where the two together would pass the crossing's drive speed
	 * (FRammsCurbTuning::DriveSpeed), the drive slows by as much as the seat is
	 * shifting, by up to this (cm/s) and no lower than the floor
	 * (FRammsCurbTuning::MinSpeedFraction of it), so the rider goes no faster
	 * than the approach. Below it the shift is left to make up the rider's
	 * speed where the drive has slowed for the crossing. 0, the default: the
	 * drive keeps the planned speed.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat", meta = (ClampMin = "0.0"))
	float ForeSlowDownCmS = 0.0f;

	/** How fast the seat's shift speeds up and slows down (cm/s^2). At the
	 *  drive's own Acceleration the drive can give up or make up the seat's
	 *  speed as it changes (AftSpeedUpCmS, ForeSlowDownCmS) rather than a beat
	 *  behind it. 0, the default: at once. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat", meta = (ClampMin = "0.0"))
	float ShiftAccelCmS2 = 0.0f;

	/** A seat the crossing moves: both 5-bar motors named. */
	bool IsSet() const { return !Linkage.ProximalMotorA.IsNone() && !Linkage.ProximalMotorB.IsNone(); }
};

/**
 * How the rider's stick drives a per-wheel crossing
 * (URammsCurbNegotiationController::Stick). With bRequired, the crossing
 * moves only while the stick is held forward: held part way, it goes at that
 * share of the plan's speed; let go, or pushed any other way, it pauses --
 * braked to rest at the crossing's Deceleration (the rate every planned pose
 * keeps its margin under), no new lift or level-in begun, those under way
 * finished, then held, the wheels braked -- and goes on when the stick is
 * pushed forward again. Before any leg has moved for the curb, pulled back or
 * pushed to the side for ReleaseSeconds cancels the crossing and hands the robot back to its
 * drive mode; levelled in with no wheel yet lifted, held back for
 * CancelHoldSeconds at rest does the same, the legs taken back to the drive
 * stance; once a wheel has begun to cross, nothing but forward moves it.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsCurbStick
{
	GENERATED_BODY()

	/** The crossing moves only while the stick is held forward; the per-axle
	 *  gait, which does not read the stick, is not begun. Off, a crossing runs
	 *  to Done on its own once begun, and the stick is not read (scripted or
	 *  autonomous crossings). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stick")
	bool bRequired = true;

	/** The stick pushed forward past this share of its travel (0..1), within
	 *  ForwardSectorDeg of straight ahead, takes the crossing on. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stick", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float EngageFraction = 0.25f;

	/** Held forward, the stick is let go once it has been under this share of
	 *  its travel, or out of ForwardSectorDeg, for ReleaseSeconds: a hand that
	 *  wavers (a tremor, a bump) does not stop the robot. Under this share in
	 *  any direction, the stick is at rest. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stick", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float ReleaseFraction = 0.15f;

	/** How long (s) the stick must be out of forward to be let go (above); and
	 *  pulled back or pushed to the side, before any leg has moved for the
	 *  curb, to cancel the crossing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stick", meta = (ClampMin = "0.0"))
	float ReleaseSeconds = 0.15f;

	/** How far round from straight ahead (deg) the stick still reads forward;
	 *  as far round from straight back, it reads back, and between, sideways. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stick", meta = (ClampMin = "0.0", ClampMax = "89.0"))
	float ForwardSectorDeg = 30.0f;

	/** Levelled in with no wheel yet lifted, the stick held back this long (s)
	 *  with the robot at rest cancels the crossing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stick", meta = (ClampMin = "0.0"))
	float CancelHoldSeconds = 0.75f;
};

/** The step the robot is to cross. */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsCurbProfile
{
	GENERATED_BODY()

	/** Height of the far side above the near side (cm): positive for a curb
	 *  to climb, negative for one to descend. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	float StepHeightCm = 15.24f;

	/** Horizontal distance (cm) from the front wheels' centres to the step
	 *  edge, along the robot's heading. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb", meta = (ClampMin = "0.0"))
	float EdgeDistanceCm = 50.0f;

	/**
	 * How much farther the edge is from the farthest front wheel than from
	 * the nearest (cm), when the robot meets it a little off square. The
	 * braking limits keep to the near end of the edge and every landing is
	 * planned past the far end. From a detected terrain it is derived: the
	 * front track times |tan ApproachAngleDeg|.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb", meta = (ClampMin = "0.0"))
	float EdgeSkewCm = 0.0f;

	/**
	 * The angle (deg) between the robot's heading and the normal to the edge:
	 * 0 square on, positive when the robot's +Y wheels meet the edge first.
	 * The per-axle gait lifts both wheels of an axle together, which only
	 * works within 10 deg of square; it refuses an edge farther off.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	float ApproachAngleDeg = 0.0f;

	/**
	 * The terrain the step was found in (DetectTerrainAhead), when it was
	 * detected: the two surfaces and the edge, which the fields above are
	 * derived from. Used when bValid; a profile without one stands for level
	 * ground on both sides of an edge square to the heading.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FRammsCurbTerrain Terrain;
};

/** Where the manoeuvre is. Idle, Approach, Prepare, Settle, Done, Aborted and
 *  Faulted belong to both gaits; Lift, Shift, Advance and Plant are the
 *  per-axle gait's steps, each naming an axle (see
 *  URammsCurbNegotiationController::GetPhaseAxle); Cross, Hold and Paused are
 *  the per-wheel gait's, during which each wheel has its own state
 *  (GetWheelStatus). */
UENUM(BlueprintType)
enum class ERammsCurbPhase : uint8
{
	Idle UMETA(DisplayName = "Idle"),
	/** Driving up to the step in the stance the robot was already driving
	 *  in -- the differential stance, front corners up -- until it is close. */
	Approach UMETA(DisplayName = "Approach"),
	/** All wheels down; the chassis moves to the height, and the centre
	 *  wheels to the fore/aft offset, the next lift needs. */
	Prepare UMETA(DisplayName = "Prepare"),
	/** Per-axle gait: one axle comes up clear of the step. */
	Lift UMETA(DisplayName = "Lift"),
	/** Per-axle gait: with that axle up, the chassis and centre wheels move
	 *  to where the robot can drive on the other two -- only when no one pose
	 *  serves both the lift and the drive. */
	Shift UMETA(DisplayName = "Shift"),
	/** Per-axle gait: driving on the other two axles until the lifted one is
	 *  over its landing spot. */
	Advance UMETA(DisplayName = "Advance"),
	/** Per-axle gait: the lifted axle goes down onto the far side and takes
	 *  load. */
	Plant UMETA(DisplayName = "Plant"),
	/** Every wheel across: the chassis returns to its starting ride height,
	 *  level, on every wheel. The drive mode handed back to then puts the
	 *  robot in its own stance. */
	Settle UMETA(DisplayName = "Settle"),
	Done   UMETA(DisplayName = "Done"),
	/** Stopped on request: by Abort, after which the robot holds where it is
	 *  (per wheel, once braked to rest at Tuning.Deceleration); by selecting
	 *  another drive mode, which takes the robot over; or by the rider's stick
	 *  before any wheel has lifted (FRammsCurbStick), after which the drive
	 *  mode the crossing came from takes the robot back. */
	Aborted UMETA(DisplayName = "Aborted"),
	/** Stopped by one of its own checks: a tilt past MaxTiltDeg, a step that
	 *  overran its timeout, a stall (FRammsCurbTuning::StallSeconds), or a
	 *  robot it can no longer measure -- and, per wheel, the terrain not as
	 *  detected (TerrainMismatchCm), a landing off its surface
	 *  (LandingToleranceCm), the base off its planned attitude
	 *  (AttitudeFaultDeg), the robot measured near tipping, a lifted wheel at
	 *  the terrain, a leg far off its servo, or a lift, landing or level-in
	 *  overrunning -- after which the robot holds where it is (per wheel, once
	 *  braked to rest at Tuning.Deceleration; a robot its driven wheels are
	 *  found not to hold -- running away from its command, or rolling back
	 *  with a wheel up or while it brakes -- has every wheel braked at once);
	 *  or a simulation reset,
	 *  after which the mode from before takes it back. A per-wheel plan made
	 *  off the game thread that refuses ends here too ("refused on the move:
	 *  ..." -- braked to rest and held -- or "refused at rest: ...", the robot
	 *  handed back to the mode it came from: nothing moved). See
	 *  GetStatusText, which names the check, the wheel and the numbers. */
	Faulted UMETA(DisplayName = "Faulted"),
	/** Per-wheel gait: a wheel's crossing is pending or under way (appended
	 *  after Faulted: the values before it keep their indices). */
	Cross UMETA(DisplayName = "Cross"),
	/** Per-wheel gait: braked and waiting, with a reason (a permit, a re-plan,
	 *  the terrain), resuming when it clears. */
	Hold UMETA(DisplayName = "Hold"),
	/** Per-wheel gait: the rider's stick is not held forward
	 *  (URammsCurbNegotiationController::Stick): braked to rest and waiting
	 *  for it, for as long as it takes. */
	Paused UMETA(DisplayName = "Paused"),
};

/** Which gait crosses the curb (FRammsCurbTuning::Gait; PerWheel by default). */
UENUM(BlueprintType)
enum class ERammsCurbGait : uint8
{
	/** Both wheels of an axle together, axle by axle, front to rear, each step
	 *  waiting for the last (within 10 deg of square), the base level. The
	 *  earlier gait, kept as an option. */
	PerAxle UMETA(DisplayName = "Per axle"),
	/** Every wheel on its own, planned in chassis travel before anything moves
	 *  (the per-wheel planner); legs and driving overlap, the base level or
	 *  tilted within MaxPlannedTiltDeg. The default. */
	PerWheel UMETA(DisplayName = "Per wheel"),
};

/** Where one wheel is in the crossing. See FRammsCurbWheelStatus. */
UENUM(BlueprintType)
enum class ERammsCurbWheelState : uint8
{
	/** On the ground, carrying its share of the robot. */
	Planted UMETA(DisplayName = "Planted"),
	/** Coming up off the ground, not yet clear of the terrain under it. */
	Lifting UMETA(DisplayName = "Lifting"),
	/** Up and clear of the terrain under it, being carried across. */
	Clear UMETA(DisplayName = "Clear"),
	/** Going down onto the surface it is to land on, not yet touching. */
	Planting UMETA(DisplayName = "Planting"),
	/** Down on that surface, taking load. */
	Loading UMETA(DisplayName = "Loading"),
	/** Off the ground and held there: in the stance the robot drove up in,
	 *  or where a crossing that stopped left it. */
	Held UMETA(DisplayName = "Held"),
};

/**
 * One wheel's part in the crossing, as last measured.
 * See URammsCurbNegotiationController::GetWheelStatus.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsCurbWheelStatus
{
	GENERATED_BODY()

	/** The leg's Name. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb")
	FName Leg;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb")
	ERammsCurbWheelState State = ERammsCurbWheelState::Planted;

	/** The surface the wheel stands on, or, lifted, the one it is headed
	 *  for: 0 the near side (where the crossing began), 1 the far side. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb")
	int32 Plane = 0;

	/** Wheel centre past the edge along the crossing (cm; negative short of
	 *  it), as URammsCurbNegotiationController::GetWheelProgress. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb")
	float PastEdgeCm = 0.0f;

	/**
	 * The stretch of PastEdgeCm (cm) the wheel must not be in while it
	 * carries load: from where a loaded wheel stops short of the face
	 * (climbing) or the lip (descending), to where it is set down past the
	 * edge.
	 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb")
	float WindowLo = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb")
	float WindowHi = 0.0f;

	/** Wheel bottom above the surface under the wheel centre (cm). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb")
	float ClearanceCm = 0.0f;

	/** World height of the surface under the wheel centre (cm). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb")
	float SurfaceZ = 0.0f;

	/** Wheel-centre height in the chassis frame (cm): commanded. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb")
	float CmdZ = 0.0f;

	/** Wheel-centre height in the chassis frame (cm): measured. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb")
	float MeasZ = 0.0f;

	/** The wheel bottom is on (or pressed into) the surface under it. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb")
	bool bContact = false;
};

/**
 * How a crossing went, accumulated by the controller on every one of its
 * ticks (simulated time) from the crossing's start to Done. Reset when a
 * crossing begins; it keeps the last crossing's values once that ends. A
 * crossing that faults or is aborted stops accumulating there.
 *
 * The speed fields wait for motion, so that a robot starting from a
 * standstill is neither stopped before it sets off nor at its slowest while
 * it speeds up. Stops and SecondsBelowStopSpeed count from first motion, the
 * first tick at which the chassis' measured speed reaches 5 cm/s. The speed
 * minima count from the end of the rise from rest: the first tick after that
 * at which the measured speed reaches a quarter of Tuning.DriveSpeed, or
 * falls 2 cm/s back from the most it reached (the robot slowing short of
 * it); a robot already rolling counts from its first tick. Everything else
 * counts from the start.
 *
 * Speeds are cm/s, along the chassis heading (negative rolling back);
 * attitudes deg (pitch positive nose up, roll positive +Y side up); lengths
 * cm. A value the gait does not measure stays 0; each field says which. The
 * per-axle gait plans a level chassis, no tilt and no timing, and never
 * holds.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsCurbRunStats
{
	GENERATED_BODY()

	/** Least drive speed the controller commanded, from the end of the rise
	 *  from rest (from first motion, when the robot never finished rising). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Speed")
	float MinCommandedSpeed = 0.0f;

	/** Least chassis speed measured, each over the last 0.3 s, from the end of
	 *  the rise from rest (from first motion, when the robot never finished
	 *  rising). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Speed")
	float MinMeasuredSpeed = 0.0f;

	/** Seconds the measured speed spent below 2 cm/s. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Speed")
	float SecondsBelowStopSpeed = 0.0f;

	/** Times the measured speed fell below 2 cm/s and stayed there for
	 *  0.2 s or more: every stop, planned or not. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Speed")
	int32 Stops = 0;

	/** Stops the plan called for: per-axle, each step planned at zero speed
	 *  (an axle lifted standing still) that follows one that was not. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Speed")
	int32 PlannedStops = 0;

	/** Times the robot braked to wait (per-axle: never). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Speed")
	int32 Holds = 0;

	/** Largest |pitch|, standing or moving, the attitude a fault stopped on
	 *  included. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Attitude")
	float MaxAbsPitchDeg = 0.0f;

	/** Largest |roll|, standing or moving, the attitude a fault stopped on
	 *  included. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Attitude")
	float MaxAbsRollDeg = 0.0f;

	/** Largest inclination of the base from level -- its total tilt,
	 *  atan(sqrt(tan^2 pitch + tan^2 roll)), whichever way it leans --
	 *  standing or moving, the attitude a fault stopped on included. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Attitude")
	float MaxInclinationDeg = 0.0f;

	/** Largest pitch or roll away from the attitude the gait meant to hold.
	 *  Per-axle: level, once the level-in has brought the chassis within
	 *  1 deg of level from the stance the robot drove up in (not counted
	 *  while it does). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Attitude")
	float MaxAttitudeErrorDeg = 0.0f;

	/** Pitch at Done (or at the fault or abort that ended the crossing). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Attitude")
	float FinalPitchDeg = 0.0f;

	/** Roll at Done (or at the fault or abort that ended the crossing). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Attitude")
	float FinalRollDeg = 0.0f;

	/** Least stability margin the plan allowed: how far inside the contacts
	 *  of the wheels on the ground the centre of mass stands, less the shift
	 *  of braking at Tuning.Deceleration or speeding up at Tuning.Acceleration
	 *  (whichever leaves less), in each pose the plan holds while driving.
	 *  Per-axle: each axle's lift (a lift made standing still, without the
	 *  shift), and the stance the robot drove up in when that carries an axle
	 *  up through the approach -- that one recorded, not checked against
	 *  Tuning.MinStabilityMargin: it is the drive mode's, and it is evaluated
	 *  as if level although on a slope the robot drives it tilted. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Margins")
	float MinPlannedMarginCm = 0.0f;

	/** Least stability margin measured, the same measure as
	 *  MinPlannedMarginCm, on every tick: the centre of mass from the measured
	 *  legs (FRammsCurbLeg::MovingMassKg, Tuning.CenterOfMass), projected
	 *  straight down, against the contacts of the wheels measured on the
	 *  ground. See URammsCurbNegotiationController::GetStabilityMargin. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Margins")
	float MinMeasuredMarginCm = 0.0f;

	/** Least gap measured between a lifted wheel, clear of the ground and
	 *  being carried across, and the terrain under it, along its rolling
	 *  direction (cm; negative would be a wheel into the terrain). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Margins")
	float MinSwingClearanceCm = 0.0f;

	/** Largest distance of a leg's measured wheel centre from where its
	 *  servo lag model (Tuning.ServoLagSeconds) puts it. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Margins")
	float MaxLegErrorCm = 0.0f;

	/** Least drive speed the plan called for (per-axle: the slowest step's
	 *  own speed, 0 for a lift made standing still; the deadline cap that
	 *  slows it ahead of each lift is live, not planned). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Plan")
	float PlannedMinSpeed = 0.0f;

	/** Largest inclination of the base from level the plan called for (its
	 *  total tilt, which Tuning.MaxPlannedTiltDeg bounds; per-axle: 0, level). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Plan")
	float PlannedMaxTiltDeg = 0.0f;

	/** How long the plan expected the crossing to take (per-axle: not
	 *  planned, 0). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Plan")
	float PlannedCrossingSeconds = 0.0f;

	/** Seconds from the first leg commanded away from where it began to Done:
	 *  the crossing proper, without the approach. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Plan")
	float CrossingSeconds = 0.0f;

	/** Times the plan was rebuilt mid-crossing (per-axle: on the approach,
	 *  when a look again found the terrain moved). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Plan")
	int32 Replans = 0;

	/** The chassis centre's sideways offset from the line it began the
	 *  crossing on (cm, positive towards +Y), at the last tick counted. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Plan")
	float LateralDriftCm = 0.0f;

	/** How far the chassis heading has turned since the crossing began (deg,
	 *  positive towards +Y), at the last tick counted. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Curb|Plan")
	float YawChangeDeg = 0.0f;
};

/**
 * Tuning for the gait. Distances in cm, speeds in cm/s.
 *
 * The defaults are for a 6 in (15.24 cm) curb on the lift-drive at roughly a
 * walking pace; they are robot-agnostic in form but not in value.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsCurbTuning
{
	GENERATED_BODY()

	/** Which gait crosses: PerWheel, the default, or PerAxle (the earlier
	 *  gait, kept as an option). A per-wheel plan is level where it can be,
	 *  else tilted by the least within MaxPlannedTiltDeg at the floor speed or
	 *  more, else level with a planned stop (bAllowPlannedStop), else refused;
	 *  it never hands a crossing to the other gait. Until the per-wheel gait
	 *  became the default, PerAxle was; a robot saved with this field at that
	 *  default crosses per wheel now. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Gait")
	ERammsCurbGait Gait = ERammsCurbGait::PerWheel;

	/** The least speed a per-wheel plan rolls at, as a fraction of DriveSpeed
	 *  (the floor): where it could not keep the base level at or above it,
	 *  it tilts, stops or refuses (see MaxPlannedTiltDeg). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Gait", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MinSpeedFraction = 0.25f;

	/** How fast a leg's vertical speed may change (cm/s^2): with LegSpeed and
	 *  ServoLagSeconds, how long the per-wheel planner takes a lift or a
	 *  landing to be. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Gait", meta = (ClampMin = "1.0"))
	float LegAcceleration = 200.0f;

	/**
	 * Per-wheel gait: a wheel's lift may begin up to this much earlier (s)
	 * than it need, wherever the robot stands with the wheel up that much
	 * longer, its leg rising slower so that it is just clear by its window --
	 * a leg that rises while the robot drives, not a step at the last moment.
	 * A corner carried up in the drive stance whose eased lift would begin at
	 * the level-in's end stays up instead of being set down and lifted again.
	 * 0, the default, lifts at LegSpeed as late as it can.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Gait", meta = (ClampMin = "0.0"))
	float LiftEaseSeconds = 0.0f;

	/** A per-wheel plan that rolls slower than the floor somewhere, without
	 *  stopping, is accepted rather than refused. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Gait")
	bool bAllowSlowCrossing = false;

	/** The per-wheel gait spins each lifted wheel that has a motor at the
	 *  ground speed it will land at, so that it lands without a jolt. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Gait")
	bool bPrespinLiftedWheels = true;

	/** When a per-wheel crossing faults or is aborted, a wheel then off the
	 *  ground over a surface -- not over the edge, inside its keep-off window
	 *  -- is set down on that surface as the robot stops, where the robot then
	 *  stands at least MinLiveStabilityMarginCm inside its wheels. Off, the
	 *  default: every leg holds the target it had. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Gait")
	bool bPlantOnHold = false;

	/** Ground speed when nothing nearby limits it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Drive", meta = (ClampMin = "0.0"))
	float DriveSpeed = 35.0f;

	/**
	 * Deprecated; per-axle gait only. A further cap, as a fraction of
	 * DriveSpeed, on the steps that move legs (Prepare, Lift, Shift, Plant,
	 * Settle). The speed while legs move is the deadline cap's (see
	 * URammsCurbNegotiationController), so the default, 1, adds nothing;
	 * zero makes the gait strictly stop-and-go.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Drive", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float OverlapSpeedFraction = 1.0f;

	/** How fast the drive speed may rise, cm/s^2. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Drive", meta = (ClampMin = "1.0"))
	float Acceleration = 60.0f;

	/** How fast the drive speed may fall, cm/s^2: the ramp down to any lower
	 *  speed, and every stop the gait makes is planned against it -- the
	 *  braking caps and the deadline cap -- so it is also how close it dares
	 *  approach. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Drive", meta = (ClampMin = "1.0"))
	float Deceleration = 60.0f;

	/**
	 * Per-wheel gait: how hard the robot may brake for a wheel that is not up
	 * yet (cm/s^2) -- the hard braking caps, which stop it short of a curb face
	 * or a lip should a lift not happen. Set, every lift is also planned to
	 * begin before the robot comes within this braking distance of the
	 * wheel's window, so the caps act only when a lift is late (a stop that
	 * firm is then the one wanted) rather than braking at every hand-off.
	 * 0, the default, brakes at Deceleration and times lifts only to be clear
	 * by their windows.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Drive", meta = (ClampMin = "0.0"))
	float BrakingDeceleration = 0.0f;

	/**
	 * How much the per-wheel planner lets the speed rise within a crossing:
	 * from the first wheel's lift (or a held wheel's landing) to the last
	 * wheel's taking load, the planned speed stays under this times the
	 * tightest speed any wheel's hand-off allows there. 1 crosses at one
	 * steady pace -- no surging to DriveSpeed between hand-offs only to brake
	 * again for the next; above 1 lets it rise that much between them; below
	 * 1 holds the whole crossing under its tightest hand-off (a slower
	 * crossing, which may fall under the floor and be refused). 0, the
	 * default, leaves the speed free to rise between them.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Drive", meta = (ClampMin = "0.0"))
	float CrossingSpeedRatio = 0.0f;

	/**
	 * The per-wheel planner smooths its speed profile over this much travel
	 * (cm): each change of acceleration spread over about this length, the
	 * speed never raised above what the plan allows anywhere (the least over
	 * the length, then averaged over it). 0, the default, leaves the profile
	 * as its acceleration limits shape it. Not applied to a plan with a
	 * planned stop.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Drive", meta = (ClampMin = "0.0"))
	float SpeedSmoothingCm = 0.0f;

	/** Yaw-rate gain (1/s) holding the heading the manoeuvre began on. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Drive", meta = (ClampMin = "0.0"))
	float HeadingGain = 2.0f;

	/**
	 * Cross-track correction (per-wheel gait): the heading held is turned back
	 * towards the line the crossing was planned along by this much (deg) per cm
	 * the chassis has drifted off it sideways, so that a robot pushed sideways
	 * (a cross slope, omni wheels) steers back onto its path rather than only
	 * holding its heading. On a robot whose wheels roll partly sideways (omni
	 * wheels with their hubs turned off the heading), the same offset also
	 * asks those wheels for a sideways speed back onto the line while they
	 * carry on both sides and no main wheel does. 0 turns both off.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Heading", meta = (ClampMin = "0.0"))
	float CrossTrackGainDegPerCm = 0.1f;

	/** The most the cross-track correction turns the heading held (deg). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Heading", meta = (ClampMin = "0.0"))
	float MaxCrossTrackSteerDeg = 3.0f;

	/** Where the base cannot be held level at a speed at or above the floor,
	 *  the planner may plan a stop with the base level, rather than refuse.
	 *  (Used by the per-wheel planner; the per-axle gait plans none.) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Drive")
	bool bAllowPlannedStop = true;

	/** Vertical speed of a wheel being lifted or planted. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Legs", meta = (ClampMin = "0.1"))
	float LegSpeed = 30.0f;

	/** Vertical speed of the chassis while all wheels carry it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Legs", meta = (ClampMin = "0.1"))
	float BaseHeightSpeed = 18.0f;

	/** Fore/aft speed of the 5-bar wheels. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Legs", meta = (ClampMin = "0.1"))
	float CenterShiftSpeed = 20.0f;

	/** A leg is where it was sent when its measured height -- and a 5-bar's
	 *  fore/aft position -- is within this. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Legs", meta = (ClampMin = "0.05"))
	float SettleTolerance = 1.0f;

	/** How far below the ground a planting wheel is sent, so it takes load
	 *  rather than just touching. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Legs", meta = (ClampMin = "0.0"))
	float PlantPreload = 0.3f;

	/**
	 * Per-wheel gait: a wheel being set down meets the ground at about this
	 * speed (cm/s). Its command comes down no faster than this plus its height
	 * over the ground divided by ServoLagSeconds, so it slows over the last few
	 * centimetres -- the leg servo trails it by about that lag -- rather than
	 * arriving at LegSpeed and braking at LegAcceleration only once there.
	 * With stiff servos and fast legs that hard arrival rocks the base as the
	 * wheel lands. The planner times landings with it. 0, the default, lands
	 * at LegSpeed.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Legs", meta = (ClampMin = "0.0"))
	float TouchdownSpeed = 0.0f;

	/**
	 * How much further again a Main wheel is pressed down while it supports
	 * the robot (cm).
	 *
	 * Every support leg is held at a commanded height, and with six legs that
	 * stiff the chassis' weight goes wherever a millimetre of geometry puts it.
	 * On the holonomic chassis that was onto the corner cranks: with every
	 * wheel down the treaded centre wheels spun in place, carrying almost
	 * nothing, and the robot stalled on the curb top. Pressing the drive
	 * wheels lower than the rest makes them the ones that carry it.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Legs", meta = (ClampMin = "0.0"))
	float MainWheelPreload = 1.0f;

	/** The leg servos' lag (s), modelled as first order: how far behind its
	 *  command a leg is expected to run. Leg error in the run stats is taken
	 *  against this model rather than the raw command. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Legs", meta = (ClampMin = "0.0"))
	float ServoLagSeconds = 0.1f;

	/** Stay this far inside each leg's travel when planning. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Legs", meta = (ClampMin = "0.0"))
	float LegRangeMargin = 0.5f;

	/** Gap kept under a lifted wheel over the higher of the two surfaces. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry", meta = (ClampMin = "0.0"))
	float SwingClearance = 2.5f;

	/** Lowest point of the chassis above its origin (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry")
	float ChassisUnderside = 2.5f;

	/** Gap kept under the chassis over the higher surface. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry", meta = (ClampMin = "0.0"))
	float ChassisClearance = 3.0f;

	/** How close a wheel on the low side may roll to the curb face. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry", meta = (ClampMin = "0.0"))
	float FaceMargin = 3.0f;

	/** How close to a drop edge a wheel still carrying load may roll its
	 *  centre (negative lets it overhang). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry")
	float EdgeMargin = 2.0f;

	/** Climbing: how far past the edge a wheel centre goes before landing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry", meta = (ClampMin = "0.0"))
	float LandingOverlap = 6.0f;

	/** Descending: gap between the edge and a wheel's trailing side before it
	 *  is lowered past the edge. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry", meta = (ClampMin = "0.0"))
	float DropClearance = 2.0f;

	/** Distance before the stopping point at which the legs start to move. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry", meta = (ClampMin = "0.0"))
	float PrepareDistance = 25.0f;

	/**
	 * How far past the edge the rear wheels' trailing side is driven before
	 * the manoeuvre ends.
	 *
	 * A robot left with its rear wheels just over the edge is one the next
	 * stance change can roll back off it: the drive mode handed back to
	 * re-applies its stance, which shifts planted wheels, and with every wheel
	 * then free-rolling the linkage chassis rolled its rear omni wheels back
	 * down the curb it had just climbed.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry", meta = (ClampMin = "0.0"))
	float FinishClearance = 20.0f;

	/**
	 * Half the chassis' belly plate (cm): X fore/aft, Y across, about the
	 * chassis origin. The chassis is kept ChassisClearance over the higher
	 * surface under it. 0 (the default) takes the rectangle of the corner
	 * crank pivots, fore/aft, and of the corner wheels, across.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry", meta = (ClampMin = "0.0"))
	FVector2D ChassisHalfExtent = FVector2D::ZeroVector;

	/**
	 * The robot's centre of mass (cm, chassis frame) with every leg at its
	 * zero pose -- each 4-bar's crank at 0, each 5-bar's hips at 0. The legs'
	 * moving masses (FRammsCurbLeg::MovingMassKg) move it from there as the
	 * legs move (RobotMassKg). The crossing stays stable by keeping it, with
	 * the shift of braking or speeding up, inside the wheels on the ground.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Stability")
	FVector CenterOfMass = FVector::ZeroVector;

	/** The whole robot's mass (kg), against which each leg's MovingMassKg
	 *  moves the centre of mass. 0, the default, keeps the centre of mass
	 *  fixed at CenterOfMass whatever the legs do. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Stability", meta = (ClampMin = "0.0"))
	float RobotMassKg = 0.0f;

	/** Deprecated: CenterOfMass's X, used for it while CenterOfMass.X is 0.
	 *  The per-axle gait still places its centre wheels StabilityMargin
	 *  either side of this, its own fore/aft reference. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Stability")
	float CenterOfMassX = 0.0f;

	/** Per-axle gait only. How far the per-axle gait prefers to stand the
	 *  centre wheels past CenterOfMassX, towards the axle still on the
	 *  ground, while a corner axle is lifted -- the robot stands on two axles
	 *  then, and must not tip onto the lifted one (MinStabilityMargin says by
	 *  how much it must not). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Stability", meta = (ClampMin = "0.0"))
	float StabilityMargin = 8.0f;

	/** The least stability margin (cm) a plan accepts: how far inside the
	 *  wheels on the ground the centre of mass must stay with an axle lifted,
	 *  allowing for the shift of braking at Deceleration or speeding up at
	 *  Acceleration. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Stability", meta = (ClampMin = "0.0"))
	float MinStabilityMargin = 3.0f;

	/** The least stability margin (cm) the robot may be measured at, braking
	 *  and speeding up included: what a pose the planner did not choose -- the
	 *  stance the robot drives up in -- is held to (a planned pose is held to
	 *  MinStabilityMargin). A stance under MinStabilityMargin: standing still
	 *  in one that keeps MinStabilityMargin standing, the per-wheel gait
	 *  levels in first; else, keeping this, it levels in at once; else,
	 *  rolling in one that keeps this at a steady speed, it levels in at once
	 *  holding its speed; else it refuses. Once levelled in, a per-wheel
	 *  crossing measured under it holds -- no wheel lifts, braked to rest --
	 *  until it is back over it, and one measured under 0.5 cm faults; a wheel
	 *  lifts only where the robot measured with it up keeps this. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Stability", meta = (ClampMin = "0.0"))
	float MinLiveStabilityMarginCm = 1.5f;

	/**
	 * Per-axle gait only. The least stability margin accepted for the instant
	 * a corner axle leaves the ground with the robot standing still, when no
	 * pose gives MinStabilityMargin both with that axle still down and with it
	 * lifted.
	 *
	 * The linkage chassis met this lifting its rear axle onto a 6 in curb:
	 * with the rear wheels still on the road the chassis cannot rise far
	 * enough for the centre wheels to reach well behind the centre of mass.
	 * So the axle lifts with the robot stopped and a small margin, then the
	 * chassis rises and the centre wheels move further aft (Shift) before it
	 * drives on two axles.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Stability", meta = (ClampMin = "0.0"))
	float LiftOffMargin = 1.0f;

	/**
	 * The most the base may be inclined on purpose (deg) where its legs cannot
	 * span the terrain with it level: its total tilt from level,
	 * atan(sqrt(tan^2 pitch + tan^2 roll)), whichever way it leans -- not pitch
	 * and roll each (3.5 deg of both is 4.9 deg of inclination). The planner
	 * looks for the least inclination that keeps a pose, up to this. Planned
	 * tilt is eased in and out (TiltRateDegPerSec, TiltAccelDegPerSec2) only
	 * over the stretch that needs it, and the base returns to level after it.
	 * Beyond this the planner stops level (bAllowPlannedStop) or refuses,
	 * naming this field and the inclination it would need. (Used by the
	 * per-wheel planner; the per-axle gait holds the base level.)
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Level", meta = (ClampMin = "0.0", ClampMax = "10.0"))
	float MaxPlannedTiltDeg = 5.0f;

	/** The fastest a planned tilt, and the level-in, may turn the base
	 *  (deg/s): its total turn rate, pitch and roll together, whichever way it
	 *  turns -- not each axis (as MaxPlannedTiltDeg bounds the inclination).
	 *  An easing between two tilts runs along the straight way between them.
	 *  The drive-mode selector's hand-back from a crossing to a drive mode's
	 *  stance turns the base no faster than this either (see
	 *  GetStanceChangeMinSeconds). Until 2026-10-17 this bounded pitch and
	 *  roll each, at 2 deg/s (2.83 deg/s of total turn on a diagonal). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Level", meta = (ClampMin = "0.1"))
	float TiltRateDegPerSec = 3.0f;

	/** How fast that turn rate may change (deg/s^2, pitch and roll
	 *  together): the tilt eases in and out rather than starting and stopping
	 *  at once. Until 2026-10-17 each axis's (so up to 5.66 deg/s^2 of the
	 *  turn's on a diagonal). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Level", meta = (ClampMin = "0.1"))
	float TiltAccelDegPerSec2 = 4.0f;

	/** A chassis-height corridor narrower than this (cm) counts as no level
	 *  pose: the planner tilts a little or slows rather than shave margins.
	 *  The per-axle gait, where no one chassis height serves a step and it
	 *  follows the corridor of heights instead, needs it this wide all the
	 *  way. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Level", meta = (ClampMin = "0.0"))
	float MinCorridorCm = 1.0f;

	/**
	 * The attitude loop's proportional gain: the support legs rotate the base
	 * by this times its attitude error (measured less planned), on top of
	 * following the terrain. A nose-up error retracts the front legs.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Level", meta = (ClampMin = "0.0"))
	float AttitudeProportionalGain = 0.3f;

	/** The attitude loop's integral gain (1/s): what the legs' servos and the
	 *  drive wheels' extra press leave is trimmed out at this rate. Held while
	 *  fewer than three wheels are planted or a wheel is landing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Level", meta = (ClampMin = "0.0"))
	float AttitudeIntegralGain = 2.0f;

	/** Attitude errors within this (deg) are left alone: the leg servos
	 *  chatter at about this much. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Level", meta = (ClampMin = "0.0"))
	float AttitudeDeadbandDeg = 0.15f;

	/** The most the attitude loop rotates the base by (deg), pitch or roll. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Level", meta = (ClampMin = "0.0"))
	float MaxAttitudeCorrectionDeg = 3.0f;

	/** The base's measured inclination (deg) beyond which the manoeuvre
	 *  stops: its total tilt from level, atan(sqrt(tan^2 pitch + tan^2 roll)),
	 *  whichever way it leans -- not pitch and roll each. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Safety", meta = (ClampMin = "1.0"))
	float MaxTiltDeg = 12.0f;

	/** Longest any one step may take before the manoeuvre stops (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Safety", meta = (ClampMin = "0.5"))
	float StepTimeout = 8.0f;

	/** Longest the approach may take (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Safety", meta = (ClampMin = "0.5"))
	float ApproachTimeout = 20.0f;

	/** The manoeuvre stops when the drive speed commanded stays above 5 cm/s
	 *  while the chassis measures under 1 cm/s (over 0.3 s) for this long (s):
	 *  wheels slipping, or something holding the robot back. 0 turns the
	 *  check off. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Safety", meta = (ClampMin = "0.0"))
	float StallSeconds = 2.0f;

	/** Per-wheel gait: a wheel must touch down within this (cm) of the surface
	 *  it lands on, as planned (its plane and the bias its planted wheels
	 *  find): stopped (or pressing on the ground) within it, it has touched
	 *  down. Stopped short of it by more -- on the lip, or something lying
	 *  there -- or finding no ground that far below it (lifted clear again),
	 *  the crossing stops and faults, naming the wheel. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Safety", meta = (ClampMin = "0.1"))
	float LandingToleranceCm = 1.5f;

	/** Per-wheel gait: the terrain the crossing finds as it goes -- traced
	 *  ahead of and under each wheel, and where its planted wheels stand --
	 *  may lie at most this far (cm) off the surfaces it was planned on.
	 *  Further, and the crossing stops and faults ("terrain not as detected").
	 *  Within it, before any wheel has begun to cross, a surface found more
	 *  than 1 cm off is re-fitted and the crossing re-planned; once one has,
	 *  each surface's bias takes up at most 1.5 cm, and a surface further off
	 *  than that stops and faults too. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Safety", meta = (ClampMin = "0.1"))
	float TerrainMismatchCm = 2.0f;

	/** Per-wheel gait: how far (deg) the base may be off the attitude the plan
	 *  holds it at -- its inclination from that attitude, whichever way --
	 *  for 0.25 s before the crossing stops and faults (a slip, a wheel off an
	 *  edge, a landing on something). Half of it slows the crossing to half
	 *  the plan's speed. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Safety", meta = (ClampMin = "0.5"))
	float AttitudeFaultDeg = 4.0f;

	/** How far ahead of the front wheels the step detection scans (cm). It
	 *  has to see far enough past the edge to know the far side is deep
	 *  enough for the whole robot; when it cannot from where the crossing
	 *  begins, the approach looks again as the robot closes in (see
	 *  URammsCurbNegotiationController). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Detection", meta = (ClampMin = "1.0"))
	float DetectRange = 350.0f;

	/** Smallest height change the step detection calls a step (cm): along
	 *  the whole stretch of edge the robot crosses. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Detection", meta = (ClampMin = "0.1"))
	float MinStepHeight = 2.0f;

	/** Largest step the controller will attempt (cm), anywhere along the
	 *  stretch of edge the robot crosses. Planning refuses anything the legs
	 *  cannot reach regardless. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Detection", meta = (ClampMin = "0.1"))
	float MaxStepHeight = 22.0f;

	/** The largest angle (deg) between the heading and the normal to the
	 *  edge the detection accepts. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Detection", meta = (ClampMin = "0.0", ClampMax = "80.0"))
	float MaxApproachDeg = 35.0f;

	/** The steepest either surface may slope, in any direction (deg). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Detection", meta = (ClampMin = "0.0", ClampMax = "45.0"))
	float MaxSurfaceSlopeDeg = 8.0f;

	/** How far (cm) the edge points the scan finds may lie from one straight
	 *  line: farther, and the edge is not straight. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Detection", meta = (ClampMin = "0.0"))
	float MaxEdgeResidualCm = 1.5f;

	/** The RMS (cm) of the samples a surface's plane is fitted to may be no
	 *  more than this: rougher, and it is not a plane. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Detection", meta = (ClampMin = "0.0"))
	float MaxPlaneRmsCm = 0.5f;

	/** A sample farther than this (cm) from a surface's plane is not on it;
	 *  more than a tenth of them off it, and the surface is not a plane. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Detection", meta = (ClampMin = "0.0"))
	float MaxPlaneResidualCm = 1.5f;

	/** How deep past the edge (cm) the far surface's plane is fitted, at most
	 *  (and never across a second edge). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Detection", meta = (ClampMin = "10.0"))
	float FarFitDepthCm = 120.0f;
};

/** What the debug view's sensing layer draws (FRammsCurbDebugView::Sensing). */
UENUM(BlueprintType)
enum class ERammsCurbSensingView : uint8
{
	Off,
	/** The latest detection scan's traces, and during a crossing the terrain
	 *  monitor's probes as each tick makes them. */
	Live,
	/** Live, and the probe hits of the last TrailSeconds: the ground the
	 *  wheels' probes have swept. */
	Trail UMETA(DisplayName = "Live + trail"),
};

/**
 * The curb controller's debug view: what it senses of the terrain, what it
 * makes of it, its plan and its state, drawn in the world (debug lines, points
 * and labels) and, for the graphs, over the viewport. Each layer is switched
 * from the robot's control panel (the "Curb view" group) or here. Drawing
 * costs a little each frame and, with Sensing or Terrain on while no crossing
 * runs, a detection scan every PreviewIntervalSeconds (a few hundred traces);
 * all off, it costs nothing. Nothing in it changes what the controller does.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsCurbDebugView
{
	GENERATED_BODY()

	/** The rays the controller casts and where they hit: the detection scan,
	 *  each sample coloured by the plane it fits (near, far, neither, no hit),
	 *  and during a crossing the terrain monitor's probes ahead of and under
	 *  each wheel, coloured by how far they land from the surface planned on.
	 *  While no crossing runs, a preview scan shows what "Cross curb ahead"
	 *  would find. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Debug")
	ERammsCurbSensingView Sensing = ERammsCurbSensingView::Off;

	/** What the scan is made into: the near and far planes, the edge line and
	 *  the edge each scan lane found, the step, its angle and the planes'
	 *  slopes and fit; or why there is no crossing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Debug")
	bool bTerrain = false;

	/** The plan: each wheel's keep-off window over the edge with the height its
	 *  wheel is carried to, where it lifts and lands, and the chassis' planned
	 *  path coloured by its planned speed against the floor, where it tilts and
	 *  stops. Per-wheel gait only. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Debug")
	bool bPlan = false;

	/** The robot as the controller sees it: each wheel's state, its commanded
	 *  and measured centre, why a lift waits; the support polygon, the centre
	 *  of mass and the stability margin; the base's attitude measured and
	 *  planned; the phase and what sets the speed. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Debug")
	bool bRobot = false;

	/** Strip charts of the ground speed (the chassis' measured, commanded and
	 *  its floor, and the rider's at the seat platform's pivot), the base's
	 *  tilt (measured, planned, the limit) and the stability margin
	 *  (measured, planned), over the last GraphSeconds of simulated time: in
	 *  the HUD's "Curb view" panel (IRammsHUDPanelProvider), else over the
	 *  viewport. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Debug")
	bool bGraphs = false;

	/** Text in the world beside what is drawn, and a colour key (in the
	 *  HUD's "Curb view" panel, else over the viewport). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Debug")
	bool bLabels = true;

	/** How long the Trail keeps a probe hit (s, game time). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Debug", meta = (ClampMin = "0.5", ClampMax = "60.0"))
	float TrailSeconds = 6.0f;

	/** How often the preview scan runs while no crossing does (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Debug", meta = (ClampMin = "0.05", ClampMax = "5.0"))
	float PreviewIntervalSeconds = 0.25f;

	/** The graphs' span (s of simulated time). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Debug", meta = (ClampMin = "2.0", ClampMax = "120.0"))
	float GraphSeconds = 15.0f;

	/** Where the graphs and the colour key sit when no HUD hosts them as a
	 *  panel (the HUD's "Curb view" panel moves like any other): their
	 *  bottom-left corner, as a fraction of the viewport (0,0 top left). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Debug")
	FVector2D OverlayAnchor = FVector2D(0.14, 0.98);

	/** Offer these switches on the robot's control panel ("Curb view"). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Debug")
	bool bExposeControls = true;

	/** Any layer on. */
	bool AnyOn() const { return Sensing != ERammsCurbSensingView::Off || bTerrain || bPlan || bRobot || bGraphs; }
};
