// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class URammsRobotBaseComponent;
struct FRammsCurbLeg;

/**
 * The chassis' attitude from its corner crank pivots, which are fixed on the
 * chassis: what the Gen2 curb controller measures the base by
 * (URammsCurbNegotiationController::Measure), lifted out as plain functions so
 * that the level assist reads the same attitude by the same arithmetic and a
 * test can feed it made-up pivots. Not reflected, not exported.
 */
namespace RammsCurb
{
	/** One corner crank pivot as located. */
	struct FPivotSample
	{
		/** Where it is in the world (cm). */
		FVector World = FVector::ZeroVector;
		/** Where it sits on the chassis (cm; X fore/aft, Y up: the leg's
		 *  FRamms4BarLegSpec::CrankPivot). */
		FVector2D Local = FVector2D::ZeroVector;
		/** Its wheel's place across the chassis (cm; the leg's
		 *  Wheel.Position.Y), which shares its side. */
		double WheelY = 0.0;
		/** On the front axle (else the rear). */
		bool bFront = false;
		/** On the +Y side. */
		bool bPlusY = false;
	};

	/** What the pivots say of the chassis. */
	struct FPivotAttitude
	{
		/** Pitch, positive nose up, and roll, positive with the +Y side up
		 *  (rad). Roll is 0 with pivots on one side only. */
		double PitchRad = 0.0;
		double RollRad = 0.0;
		/** The chassis' heading and its +Y side's direction, both horizontal
		 *  unit vectors (Side zero with pivots on one side only). */
		FVector Forward = FVector::ZeroVector;
		FVector Side = FVector::ZeroVector;
		/** The pivots: their world positions summed, how many, their mean
		 *  place on the chassis, their mean world height, and their wheels'
		 *  places across the chassis summed. (Sums where the curb controller
		 *  divides them itself: its numbers are kept to the last bit.) */
		FVector	  SumWorld = FVector::ZeroVector;
		int32	  Count = 0;
		FVector2D MeanLocal = FVector2D::ZeroVector;
		double	  MeanZ = 0.0;
		double	  SumWheelY = 0.0;
	};

	/**
	 * The attitude a set of corner pivots gives: pitch from the line between
	 * the front pivots' mean and the rear pivots' mean, less the slope that
	 * line has on the chassis itself; roll from the line between the +Y
	 * pivots' mean and the -Y pivots' mean. False, Out untouched, without a
	 * front and a rear pivot.
	 */
	bool AttitudeFromPivots(TConstArrayView<FPivotSample> Pivots, FPivotAttitude& Out);

	/**
	 * The corner crank pivots of Legs (the 4-bar legs of the front and rear
	 * axles), each located through Base's GetMotorTransform on its crank
	 * motor, in the legs' order. False when one cannot be located.
	 */
	bool GatherPivots(const URammsRobotBaseComponent& Base, TConstArrayView<FRammsCurbLeg> Legs, TArray<FPivotSample, TInlineAllocator<4>>& Out);
} // namespace RammsCurb
