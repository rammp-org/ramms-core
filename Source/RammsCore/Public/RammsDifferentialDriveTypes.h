// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsDifferentialDriveTypes.generated.h"

/**
 * Control mode for differential drive system
 */
UENUM(BlueprintType)
enum class EDriveControlMode : uint8
{
	TorqueControl UMETA(DisplayName = "Torque Control"),
	VelocityControl UMETA(DisplayName = "Velocity Control")
};

/**
 * Motor parameters for electric wheelchair motors
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FMotorParameters
{
	GENERATED_BODY()

	/** Maximum motor RPM */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	float MaxRPM = 100.0f;

	/** Maximum motor torque at stall (Newton-meters) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	float MaxTorque = 7.0f;

	/** Motor torque curve - torque decreases linearly with speed to this fraction at max RPM */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float TorqueAtMaxRPM = 0.3f;

	FMotorParameters()
		: MaxRPM(100.0f)
		, MaxTorque(7.0f)
		, TorqueAtMaxRPM(0.3f)
	{}
};

/**
 * Wheel state information
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FWheelState
{
	GENERATED_BODY()

	/** Current angular velocity in radians per second */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float AngularVelocity = 0.0f;

	/** Cumulative rotation in radians */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float TotalRotation = 0.0f;

	/** Actual linear velocity at wheel contact point (cm/s) */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float LinearVelocity = 0.0f;

	/** Desired angular velocity for velocity control mode (rad/s) */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float TargetAngularVelocity = 0.0f;

	/** Current applied torque */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float AppliedTorque = 0.0f;

	/** Slip ratio (difference between desired and actual velocity) */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float SlipRatio = 0.0f;

	FWheelState()
		: AngularVelocity(0.0f)
		, TotalRotation(0.0f)
		, LinearVelocity(0.0f)
		, TargetAngularVelocity(0.0f)
		, AppliedTorque(0.0f)
		, SlipRatio(0.0f)
	{}
};

/**
 * Odometry data for tracking wheelchair position and motion
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FOdometryData
{
	GENERATED_BODY()

	/** Position in world space (cm) */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Odometry")
	FVector Position = FVector::ZeroVector;

	/** Orientation in world space */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Odometry")
	FRotator Orientation = FRotator::ZeroRotator;

	/** Linear velocity (cm/s) */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Odometry")
	FVector LinearVelocity = FVector::ZeroVector;

	/** Angular velocity (degrees/s) */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Odometry")
	FVector AngularVelocity = FVector::ZeroVector;

	/** Distance traveled by left wheel (cm) */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Odometry")
	float LeftWheelDistance = 0.0f;

	/** Distance traveled by right wheel (cm) */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Odometry")
	float RightWheelDistance = 0.0f;

	/** Total distance traveled (cm) */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Odometry")
	float TotalDistance = 0.0f;

	FOdometryData()
		: Position(FVector::ZeroVector)
		, Orientation(FRotator::ZeroRotator)
		, LinearVelocity(FVector::ZeroVector)
		, AngularVelocity(FVector::ZeroVector)
		, LeftWheelDistance(0.0f)
		, RightWheelDistance(0.0f)
		, TotalDistance(0.0f)
	{}
};

/**
 * Differential drive output containing left and right wheel commands
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FDifferentialDriveCommand
{
	GENERATED_BODY()

	/** Left wheel target (torque or velocity depending on control mode) */
	UPROPERTY(BlueprintReadWrite, Category = "Drive")
	float LeftCommand = 0.0f;

	/** Right wheel target (torque or velocity depending on control mode) */
	UPROPERTY(BlueprintReadWrite, Category = "Drive")
	float RightCommand = 0.0f;

	FDifferentialDriveCommand()
		: LeftCommand(0.0f)
		, RightCommand(0.0f)
	{}

	FDifferentialDriveCommand(float Left, float Right)
		: LeftCommand(Left)
		, RightCommand(Right)
	{}
};
