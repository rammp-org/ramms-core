// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataTable.h"
#include "Ramms5BarLinkageSpec.generated.h"

/**
 * Kinematic configuration of one planar 5-bar linkage — deliberately SEPARATE
 * from the motor registry (FRammsMotorSpec), which holds only intrinsic motor
 * facts and no geometry. A 5-bar needs mechanical parameters the base component
 * can't derive, so its controller takes this table row and still commands its
 * two proximal motors by Id through the base component.
 *
 * The mechanism (e.g. the lift_drive centre leg): two grounded, actuated
 * proximal pivots (A, B) each carry a proximal link to a passive knee, and a
 * distal link from each knee meets at a shared endpoint (the wheel mount, closed
 * in MuJoCo by a <connect> equality). Driving the two proximal joint angles
 * positions that endpoint in the linkage's local x-z plane.
 *
 * All geometry is expressed in that local **x-z plane**, in **centimetres**:
 * `FVector2D(X, Z)` — X along the robot's forward axis, Z (the 2D `.Y`) up.
 * Joint angle (rad, as commanded on / read from the motor) maps to a link's
 * absolute angle in that plane (atan2(z, x)) as:
 * `linkAngle = ZeroDir + Sign * jointAngle`.
 *
 * Sign convention (right-hand rule): a MuJoCo hinge about **+Y** rotates +X
 * toward −Z, i.e. it DECREASES the link angle, so `Sign = -1` for a joint
 * whose axis is `0 1 0` and `Sign = +1` for `0 -1 0`. Getting this backwards
 * is not caught by IK/FK self-consistency checks — the leg simply moves the
 * opposite way in the sim (extends when asked to retract).
 *
 * Defaults below are measured from lift_drive_linkage_ue.xml (left centre leg:
 * hip_a axis +Y, hip_b axis −Y) and were validated live in PIE.
 */
USTRUCT(BlueprintType)
struct FRamms5BarLinkageSpec : public FTableRowBase
{
	GENERATED_BODY()

	/** Proximal motor A — a grounded, actuated (position) pivot in the base
	 *  motor registry. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar")
	FName ProximalMotorA;

	/** Proximal motor B — the other grounded, actuated pivot. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar")
	FName ProximalMotorB;

	/** Grounded pivot A position in the local x-z plane (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Geometry")
	FVector2D PivotA = FVector2D(6.5, 20.993);

	/** Grounded pivot B position in the local x-z plane (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Geometry")
	FVector2D PivotB = FVector2D(-6.5, 20.992);

	/** Proximal link length A (pivot A -> knee A), cm. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Geometry", meta = (ClampMin = "0.01"))
	float ProximalLengthA = 16.0f;

	/** Distal link length A (knee A -> endpoint), cm. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Geometry", meta = (ClampMin = "0.01"))
	float DistalLengthA = 22.5f;

	/** Proximal link length B (pivot B -> knee B), cm. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Geometry", meta = (ClampMin = "0.01"))
	float ProximalLengthB = 16.0f;

	/** Distal link length B (knee B -> endpoint), cm. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Geometry", meta = (ClampMin = "0.01"))
	float DistalLengthB = 22.5f;

	/** Absolute x-z direction (rad) of proximal link A when its joint angle is 0. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Angles")
	float ZeroDirA = -0.2397f;

	/** Sign mapping joint-angle increase to link-angle increase for A (±1):
	 *  -1 for a hinge about +Y, +1 for -Y (see the class doc). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Angles")
	float AngleSignA = -1.0f;

	/** Absolute x-z direction (rad) of proximal link B when its joint angle is 0. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Angles")
	float ZeroDirB = -2.9019f;

	/** Sign mapping joint-angle increase to link-angle increase for B (±1):
	 *  -1 for a hinge about +Y, +1 for -Y (see the class doc). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Angles")
	float AngleSignB = 1.0f;

	/** IK elbow branch for arm A: true = the proximal link sits at
	 *  (pivot->endpoint direction) + interior angle. Must match the physical
	 *  assembly or IK returns the mirrored (crossed-knee) solution. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Angles")
	bool bElbowUpA = true;

	/** IK elbow branch for arm B (see bElbowUpA). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Angles")
	bool bElbowUpB = false;

	/**
	 * Knee limits for arm A (rad, X = lowest, Y = highest): the range the
	 * passive knee's angle must stay in for the mechanism to hold a pose.
	 *
	 * The knee angle is GEOMETRIC: the signed angle from proximal link A's
	 * direction to distal link A's direction in this x-z plane (positive from
	 * +X towards +Z, like the link angles), 0 at a straight knee. An elbow-up
	 * arm (bElbowUpA) therefore bends negative and an elbow-down arm positive,
	 * and a range sits on that side of 0.
	 *
	 * It is not what a simulator's knee joint reads. A joint usually reads 0
	 * in the pose the model was built in, where the knee need not be straight,
	 * and may turn either way. A hinge range [lo, hi] becomes
	 * sorted(Rel0 + S * lo, Rel0 + S * hi), where Rel0 is this angle in the
	 * built pose and S is the sign AngleSignA would have for the knee hinge
	 * (-1 about +Y, +1 about -Y, both flipped when TranslationSign is -1,
	 * since a mirrored frame turns every rotation round). Copying the hinge
	 * range across unchanged bounds a different angle: it either refuses
	 * poses the leg can hold or keeps accepting ones it cannot.
	 *
	 * Enforced by URamms5BarKinematics::SolveIK, so by everything that asks
	 * it whether a target is reachable: the linkage controller's SolveTarget
	 * and SetEndpointTarget, its reachable ranges, extent and outline, and the
	 * curb planner. ComputeEndpoint does not check it: a live reading pressed
	 * against a stop is still a pose. Checked modulo 2*pi, so a range may
	 * straddle +-pi; the default, the whole circle, bounds nothing and leaves
	 * a spec without limits exactly as it was.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Angles")
	FVector2D DistalRangeA = FVector2D(-UE_DOUBLE_PI, UE_DOUBLE_PI);

	/** Knee limits for arm B (rad), in the same convention as DistalRangeA. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Angles")
	FVector2D DistalRangeB = FVector2D(-UE_DOUBLE_PI, UE_DOUBLE_PI);

	/** FK picks the endpoint on a fixed side of the knee-A -> knee-B line
	 *  (the mechanism can't cross that line without the distal links going
	 *  collinear). Default side is the lift_drive assembly (endpoint below the
	 *  knees); flip for a mechanism assembled the other way round. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar|Angles")
	bool bFlipEndpointSide = false;

	/**
	 * +1 when this linkage's local +X points forward on the robot, -1 when the
	 * leg is mounted mirrored and its local +X points aft.
	 *
	 * Without it a pair of legs disagrees about which way "forward" is: on the
	 * lift-drive, commanding +5 cm fore/aft moved the left wheel 5.2 cm
	 * forward and the right wheel 5.1 cm backward, because both reach the same
	 * endpoint in their own frame and those frames are mirror images. Every
	 * coordinate crossing this controller's public surface -- targets, the
	 * live endpoint, the reachable ranges -- is in the robot's sense, and this
	 * is where it is converted.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|5-Bar", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float TranslationSign = 1.0f;
};
