// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * The rigid-body kinematics every IMU provider shares, as pure functions (Unreal units, world axes, angular rates
 * in rad/s unless named otherwise). A point P at Lever = P - O from a point O of the same rigid body:
 *   v_P = v_O + w x r
 *   a_P = a_O + alpha x r + w x (w x r)
 * The cross product is FVector::CrossProduct, the sense Unreal's quaternions (and Chaos) rotate by.
 */
namespace RammsImuMath
{
	/** The velocity of the point at Lever from O. */
	FORCEINLINE FVector PointVelocity(const FVector& OriginVelocity, const FVector& AngularVelocityRad, const FVector& Lever)
	{
		return OriginVelocity + FVector::CrossProduct(AngularVelocityRad, Lever);
	}

	/** The acceleration of the point at Lever from O. Holds for a proper acceleration too (gravity is uniform). */
	FORCEINLINE FVector PointAcceleration(const FVector& OriginAcceleration, const FVector& AngularAccelerationRad,
		const FVector& AngularVelocityRad, const FVector& Lever)
	{
		return OriginAcceleration + FVector::CrossProduct(AngularAccelerationRad, Lever)
			+ FVector::CrossProduct(AngularVelocityRad, FVector::CrossProduct(AngularVelocityRad, Lever));
	}

	/** What an accelerometer reads: the coordinate acceleration minus gravity (+|g| up at rest, 0 in free fall). */
	FORCEINLINE FVector ProperAcceleration(const FVector& CoordinateAcceleration, const FVector& Gravity)
	{
		return CoordinateAcceleration - Gravity;
	}

	/** A world vector in the frame of a sensor oriented SensorOrientation. */
	FORCEINLINE FVector ToSensorFrame(const FQuat& SensorOrientation, const FVector& World)
	{
		return SensorOrientation.UnrotateVector(World);
	}

	/** The angular velocity (rad/s, world) that turns Previous into Current in Dt: Current = dq * Previous, dq the
	 *  rotation by |w| Dt about w, taken the short way round. Zero for Dt <= 0. */
	FORCEINLINE FVector AngularVelocityBetween(const FQuat& Previous, const FQuat& Current, double Dt)
	{
		if (Dt <= 0.0)
		{
			return FVector::ZeroVector;
		}
		FQuat Delta = Current * Previous.Inverse();
		Delta.Normalize();
		if (Delta.W < 0.0)
		{
			Delta = FQuat(-Delta.X, -Delta.Y, -Delta.Z, -Delta.W); // the same rotation, the short way round
		}
		FVector Axis;
		double	Angle = 0.0;
		Delta.ToAxisAndAngle(Axis, Angle);
		return Axis * (Angle / Dt);
	}

	/** The first difference of a sampled vector (zero for Dt <= 0). */
	FORCEINLINE FVector Rate(const FVector& Previous, const FVector& Current, double Dt)
	{
		return Dt > 0.0 ? (Current - Previous) / Dt : FVector::ZeroVector;
	}

	FORCEINLINE FVector RadiansToDegrees(const FVector& Radians)
	{
		return Radians * (180.0 / UE_DOUBLE_PI);
	}

	FORCEINLINE FVector DegreesToRadians(const FVector& Degrees)
	{
		return Degrees * (UE_DOUBLE_PI / 180.0);
	}
} // namespace RammsImuMath
