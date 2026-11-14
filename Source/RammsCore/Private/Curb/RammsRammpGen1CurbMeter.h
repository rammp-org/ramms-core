// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsRammpGen1CurbController.h"

/**
 * The RAMMP Gen1 curb crossing's measurements (FRammsRammpGen1CurbStats), fed one tick at a time so they can be tested
 * on made-up motion. URammsRammpGen1CurbController samples the robot into an FMeterSample each tick; ramms-sim's
 * mujoco/rammp_gen1/curb_sequence.py keeps the same measures for the MuJoCo reference (its Meter).
 */
namespace RammsRammpGen1Curb
{
	/** One tick of a crossing, as measured. Lengths cm, angles deg. */
	struct FMeterSample
	{
		double	  TimeS = 0.0;
		FVector	  ChassisPos = FVector::ZeroVector;
		FVector2D Heading = FVector2D(1.0, 0.0); // unit, in the ground plane
		float	  PitchDeg = 0.0f;				 // nose up positive
		float	  RollDeg = 0.0f;
		FVector	  RiderPos = FVector::ZeroVector;
		/** The wheels on the ground, where they touch it (x-y). */
		TArray<FVector2D> Contacts;
		/** The centre of mass, straight down (x-y). */
		FVector2D				 ComXY = FVector2D::ZeroVector;
		bool					 bFrameOverEdge = false;
		float					 FrameClearanceCm = 0.0f; // the frame's underside over the edge, above it
		ERammsRammpGen1CurbPhase Phase = ERammsRammpGen1CurbPhase::Idle;
	};

	class FCrossingMeter
	{
	public:
		/** The rider's velocity and acceleration, and the pitch rate, are low-passed with this time constant (s). */
		static constexpr double FilterSeconds = 0.05;
		/** The filters settle for this long before their peaks count (s). */
		static constexpr double SettleSeconds = 0.2;
		/** Under this the chassis is stopped (cm/s); for StopSeconds or more, a stop. */
		static constexpr double StopSpeedCmS = 2.0;
		static constexpr double StopSeconds = 0.2;
		/** It has moved once its speed reaches this (cm/s): stops count from then. */
		static constexpr double MovedSpeedCmS = 5.0;

		/** Start measuring a crossing (resets the last one's). */
		void Begin(double TimeS);
		/** A tick's sample; ignored unless measuring, and a time before the last starts the filters again. */
		void Add(const FMeterSample& S);
		/** The front omni wheels reached the edge; the rear omni wheels are down on the far level. The first counts. */
		void MarkFrontAtEdge(double TimeS);
		void MarkRearDown(double TimeS);
		/** Stop measuring: the crossing ended. */
		void End(double TimeS);

		bool							IsOn() const { return bOn; }
		const FRammsRammpGen1CurbStats& GetStats() const { return Stats; }
		/** One line for a log: the times, stops, attitude, rider and margins. */
		FString Summary() const;

	private:
		FRammsRammpGen1CurbStats Stats;
		bool					 bOn = false;
		double					 T0 = 0.0;
		double					 FrontAt = -1.0;
		double					 RearAt = -1.0;
		bool					 bHasPrev = false;
		bool					 bMarginSet = false;
		double					 PrevT = 0.0;
		FVector					 PrevRider = FVector::ZeroVector, RiderVel = FVector::ZeroVector, RiderAcc = FVector::ZeroVector;
		FVector					 PrevChassis = FVector::ZeroVector, ChassisVel = FVector::ZeroVector;
		float					 PrevPitch = 0.0f;
		double					 PitchRate = 0.0;
		bool					 bMoved = false;
		double					 StillSeconds = 0.0;
		double					 SumA2 = 0.0;
		int64					 NumA = 0;
	};
} // namespace RammsRammpGen1Curb
