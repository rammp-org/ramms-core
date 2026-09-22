// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataTable.h"
#include "RammsMotorSpec.generated.h"

/**
 * How a motor's command value is interpreted by the physics backend. Intrinsic
 * to the actuator, not a semantic role — a Position motor is a position servo
 * whether it lifts a linkage or elevates a seat.
 */
UENUM(BlueprintType)
enum class ERammsActuatorType : uint8
{
	Torque	 UMETA(DisplayName = "Torque"),	  // command = force/torque (MuJoCo <motor>)
	Position UMETA(DisplayName = "Position"), // command = target position/angle (MuJoCo <position>)
	Velocity UMETA(DisplayName = "Velocity")  // command = target velocity (MuJoCo <velocity>)
};

/**
 * Gains for a velocity loop closed in software over a torque actuator.
 *
 * A torque actuator has no notion of speed: writing a rad/s figure into one
 * just applies that many newton-metres, which is how a holonomic base ended up
 * turning its wheels at 3% of the commanded rate. When a drive controller asks
 * for a wheel speed, something has to close the loop, and doing it once in the
 * robot base means every drive controller expresses speed the same way whatever
 * the backend gives it.
 */
USTRUCT(BlueprintType)
struct FRammsVelocityGains
{
	GENERATED_BODY()

	/** Proportional gain, N.m per rad/s of error. 0 = use the base's default. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity Loop", meta = (ClampMin = "0.0"))
	float Kp = 0.0f;

	/** Integral gain, N.m per rad/s per second. Pulls out the steady-state
	 *  droop a proportional-only loop leaves under load. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity Loop", meta = (ClampMin = "0.0"))
	float Ki = 0.0f;

	/** Cap on the integral term's own contribution (N.m), so a wheel held
	 *  against a wall does not wind up and then lurch when it comes free. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity Loop", meta = (ClampMin = "0.0"))
	float MaxIntegralTorque = 0.0f;

	/** Kp is the sentinel the docs above promise: zero means "use the base's
	 *  defaults". Treating a lone Ki as a set of gains would hand back an
	 *  integral-only controller from a half-filled row. */
	bool IsSet() const { return Kp > 0.0f; }
};

/**
 * One row of a robot's motor registry (a DataTable of these). Describes an
 * addressable motor with only intrinsic facts — no role, no "left/right", no
 * "lift vs seat". What a motor is *for* is expressed by which controller drives
 * its Id, not encoded here.
 *
 * `Id` is the canonical, backend-neutral name a controller references and also
 * the MuJoCo actuator name by default; `ChaosName` overrides the name only for
 * the Chaos backend (skeletal bone / constraint) when the two engines differ.
 */
USTRUCT(BlueprintType)
struct FRammsMotorSpec : public FTableRowBase
{
	GENERATED_BODY()

	/** Canonical motor id controllers reference. Also the MuJoCo actuator name
	 *  unless a backend maps it otherwise. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	FName Id;

	/** Chaos-side name (skeletal wheel bone / constraint) when it differs from
	 *  Id. NAME_None = use Id for Chaos too. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	FName ChaosName = NAME_None;

	/** How the command value is interpreted. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	ERammsActuatorType Type = ERammsActuatorType::Torque;

	/** Command clamp (min, max), in the robot's sense (before Direction). Any
	 *  range with min >= max (the default (0, 0) included) means "no clamp
	 *  here" — defer to the backend's own range (a MuJoCo actuator's
	 *  ctrlrange). A fixed value therefore can't be expressed as (v, v). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	FVector2D ControlRange = FVector2D::ZeroVector;

	/** +1 or -1: maps the robot's positive sense for this motor (e.g. "wheel
	 *  rolls forward") onto the engine's joint sign, which depends on how the
	 *  joint axis is authored. Applied to commands and to value/velocity reads,
	 *  so controllers never carry per-side sign flips. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float Direction = 1.0f;

	/** Gains for the velocity loop the robot base closes on this motor when it
	 *  is a Torque actuator (see URammsRobotBaseComponent::SetMotorVelocityCommand).
	 *  Zero Kp means "use the base's default gains". */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor|Velocity Loop")
	FRammsVelocityGains VelocityGains;
};
