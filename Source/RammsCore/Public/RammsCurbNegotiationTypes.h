// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Ramms4BarLegSpec.h"
#include "RammsHolonomicDriveTypes.h"
#include "RammsCurbNegotiationTypes.generated.h"

/**
 * Which wheel pair a leg belongs to, front to back.
 *
 * Curb negotiation is a three-axle gait: two axles carry the robot while the
 * third is lifted over the step, one axle at a time, front to rear.
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
 * The lift-drive drives like its differential mode -- on the treaded centre
 * wheels -- and leans on the others only when it has to. Which others, and
 * when, is the robot's: the holonomic chassis' omni wheels take over only while
 * the centre wheels are off the ground; the linkage chassis' powered front
 * wheels help throughout, and its rear omni wheels are never driven.
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
	/** Drives only while the main wheels are off the ground; rolls free
	 *  otherwise. */
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
 * same frame both linkage specs use, so a support leg on ground at height G
 * (relative to where the robot started) with the chassis origin at height B
 * sits at G + WheelRadiusCm - B.
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

	/**
	 * When this wheel is driven; see ERammsCurbWheelDrive. On a robot with no
	 * Main wheel, every wheel that is not Passive drives throughout.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	ERammsCurbWheelDrive Drive = ERammsCurbWheelDrive::Fallback;
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
	 * planned past the far end.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb", meta = (ClampMin = "0.0"))
	float EdgeSkewCm = 0.0f;
};

/** Where the manoeuvre is. Each step of the gait names an axle too; see
 *  URammsCurbNegotiationController::GetPhaseAxle. */
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
	/** One axle comes up clear of the step. */
	Lift UMETA(DisplayName = "Lift"),
	/** With that axle up, the chassis and centre wheels move to where the
	 *  robot can drive on the other two -- only when no one pose serves both
	 *  the lift and the drive. */
	Shift UMETA(DisplayName = "Shift"),
	/** Driving on the other two axles until the lifted one is over its
	 *  landing spot. */
	Advance UMETA(DisplayName = "Advance"),
	/** The lifted axle goes down onto the far side and takes load. */
	Plant UMETA(DisplayName = "Plant"),
	/** Every wheel across: the chassis returns to its starting ride height,
	 *  level, on every wheel. The drive mode handed back to then puts the
	 *  robot in its own stance. */
	Settle UMETA(DisplayName = "Settle"),
	Done   UMETA(DisplayName = "Done"),
	/** Stopped by Abort; the robot holds where it is. */
	Aborted UMETA(DisplayName = "Aborted"),
	/** Stopped by a safety check; see GetStatusText. */
	Faulted UMETA(DisplayName = "Faulted"),
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

	/** Ground speed when nothing nearby limits it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Drive", meta = (ClampMin = "0.0"))
	float DriveSpeed = 35.0f;

	/** Fraction of DriveSpeed used while legs reposition (Prepare, Lift) and
	 *  while an axle lands (Plant). Zero makes the gait strictly stop-and-go. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Drive", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float OverlapSpeedFraction = 0.4f;

	/** Acceleration and braking limit, cm/s^2. Every stop the gait makes is
	 *  planned against this, so it is also how close it dares approach. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Drive", meta = (ClampMin = "1.0"))
	float Acceleration = 60.0f;

	/** Yaw-rate gain (1/s) holding the heading the manoeuvre began on. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Drive", meta = (ClampMin = "0.0"))
	float HeadingGain = 2.0f;

	/** Vertical speed of a wheel being lifted or planted. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Legs", meta = (ClampMin = "0.1"))
	float LegSpeed = 30.0f;

	/** Vertical speed of the chassis while all wheels carry it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Legs", meta = (ClampMin = "0.1"))
	float BaseHeightSpeed = 18.0f;

	/** Fore/aft speed of the 5-bar wheels. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Legs", meta = (ClampMin = "0.1"))
	float CenterShiftSpeed = 20.0f;

	/** A leg is where it was sent when its measured height is within this. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Legs", meta = (ClampMin = "0.05"))
	float SettleTolerance = 1.0f;

	/** How far below the ground a planting wheel is sent, so it takes load
	 *  rather than just touching. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Legs", meta = (ClampMin = "0.0"))
	float PlantPreload = 0.3f;

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

	/** Centre of mass, fore/aft, in the chassis frame (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Stability")
	float CenterOfMassX = 0.0f;

	/** How far the centre wheels shift past the centre of mass, towards the
	 *  axle still on the ground, while a corner axle is lifted -- the robot
	 *  stands on two axles then, and must not tip onto the lifted one. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Stability", meta = (ClampMin = "0.0"))
	float StabilityMargin = 8.0f;

	/** The least shift the planner accepts when the leg cannot reach the
	 *  full StabilityMargin at the height it has to hold. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Stability", meta = (ClampMin = "0.0"))
	float MinStabilityMargin = 3.0f;

	/**
	 * The least shift accepted for the instant a corner axle leaves the
	 * ground, when no pose gives MinStabilityMargin both with that axle still
	 * down and with it lifted.
	 *
	 * The linkage chassis meets this lifting its rear axle onto a 6 in curb:
	 * with the rear wheels still on the road the chassis cannot rise far
	 * enough for the centre wheels to reach well behind the centre of mass.
	 * So the axle lifts with the robot stopped and a small margin, then the
	 * chassis rises and the centre wheels move further aft (Shift) before it
	 * drives on two axles.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Stability", meta = (ClampMin = "0.0"))
	float LiftOffMargin = 1.0f;

	/** Pitch or roll (deg) beyond which the manoeuvre stops. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Safety", meta = (ClampMin = "1.0"))
	float MaxTiltDeg = 12.0f;

	/** Longest any one step may take before the manoeuvre stops (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Safety", meta = (ClampMin = "0.5"))
	float StepTimeout = 8.0f;

	/** Longest the approach may take (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Safety", meta = (ClampMin = "0.5"))
	float ApproachTimeout = 20.0f;

	/** How far ahead DetectStepAhead looks (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Detection", meta = (ClampMin = "1.0"))
	float DetectRange = 250.0f;

	/** Smallest height change DetectStepAhead calls a step (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Detection", meta = (ClampMin = "0.1"))
	float MinStepHeight = 2.0f;

	/** Largest step the controller will attempt (cm). Planning refuses
	 *  anything the legs cannot reach regardless. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Detection", meta = (ClampMin = "0.1"))
	float MaxStepHeight = 22.0f;
};
