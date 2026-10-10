// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataTable.h"
#include "Ramms4BarLegSpec.generated.h"

/**
 * Planar geometry of a crank-rocker leg: one Position motor (the crank) drives
 * a coupler, which is pinned to a rocker arm that carries a wheel.
 *
 * The lift-drive's corner wheels hang on these. Only the crank is actuated --
 * the coupler and rocker are passive, closed by an equality constraint -- so
 * commanding a wheel height means solving the loop for the crank angle that
 * puts the wheel there. URamms4BarKinematics does that from this spec.
 *
 * Frame: the chassis' x-z plane, in cm, `.X` forward and `.Y` up, the same
 * frame FRamms5BarLinkageSpec uses. Every vector is taken at the zero pose
 * (all joints at 0), where the MJCF bodies are unrotated.
 *
 * A joint turning about the chassis' +Y axis has AngleSign +1 and one turning
 * about -Y has -1; the rear legs on the lift-drive are mirrored that way.
 *
 * A wheel on a parallel-lever mount (the holonomic chassis' omni wheels) does
 * not rotate with the rocker: the lever keeps the mount's orientation, so the
 * wheel centre is the mount pivot, carried on the rocker, plus a constant
 * offset. A wheel fixed on the rocker has WheelOffset zero and WheelOnRocker
 * pointing at the wheel itself. Both are described by:
 *
 *     wheel = RockerPivot + Rotate(RockerAngleSign * rocker, WheelOnRocker) + WheelOffset
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRamms4BarLegSpec : public FTableRowBase
{
	GENERATED_BODY()

	/** The crank's Position motor on the robot base (radians). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4-Bar")
	FName CrankMotor;

	/** Crank pivot on the chassis (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4-Bar")
	FVector2D CrankPivot = FVector2D::ZeroVector;

	/** Crank pivot to coupler pin, at crank angle 0 (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4-Bar")
	FVector2D CrankArm = FVector2D::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4-Bar")
	float CrankAngleSign = 1.0f;

	/** Coupler pin-to-pin length (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4-Bar", meta = (ClampMin = "0.0"))
	float CouplerLength = 0.0f;

	/** Rocker pivot on the chassis (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4-Bar")
	FVector2D RockerPivot = FVector2D::ZeroVector;

	/** Rocker pivot to its coupler pin, at rocker angle 0 (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4-Bar")
	FVector2D RockerPin = FVector2D::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4-Bar")
	float RockerAngleSign = 1.0f;

	/** Rocker pivot to the point that carries the wheel, at rocker angle 0 (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4-Bar")
	FVector2D WheelOnRocker = FVector2D::ZeroVector;

	/** Constant offset from that point to the wheel centre (cm); zero when the
	 *  wheel turns with the rocker. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4-Bar")
	FVector2D WheelOffset = FVector2D::ZeroVector;

	/** Crank angles (rad) the loop closes over with the rocker inside its own
	 *  joint range. Wheel height is monotonic in the crank across it on the
	 *  lift-drive, which is what lets the inverse be a bisection. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4-Bar")
	FVector2D CrankRange = FVector2D(-PI, PI);

	/** The rocker joint's range (rad); a closure that needs the rocker outside
	 *  it is refused. Which of the loop's two closures is the leg's is decided
	 *  by the zero pose's assembly mode, not by this -- near the ends of the
	 *  crank's travel both can lie inside it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "4-Bar")
	FVector2D RockerRange = FVector2D(-PI, PI);
};
