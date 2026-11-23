// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

#include "RammsLevelTypes.generated.h"

/** Where the level assist reads the chassis' attitude from. */
UENUM(BlueprintType)
enum class ERammsLevelAttitudeSource : uint8
{
	/** The simulation's own pose of the chassis: exact, with no noise. */
	Exact UMETA(DisplayName = "Exact"),
	/** The physics IMU's reading of the chassis body. */
	PhysicsImu UMETA(DisplayName = "Physics IMU"),
	/** A mounted IMU sensor component, with its noise and bias. */
	ImuSensor UMETA(DisplayName = "IMU sensor"),
};

/**
 * The level assist's tuning (URammsLevelAssistComponent::Tuning).
 *
 * The level assist keeps the rider level over sloping ground by adding one
 * rigid plane of height offsets to the legs' own commands. Angles are degrees,
 * lengths cm, times seconds. A pair of angles is (X pitch, positive nose up;
 * Y roll, positive with the robot's +Y side up).
 *
 * Declared whole from the start, so that the parts of the level assist that
 * come later (the drive-speed cap, the ride height, the seat trim) change no
 * reflected type. The law reads a plain copy of this
 * (RammsLevel::FTuning, Private/Level): the same names and defaults, kept in
 * step by a test.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsLevelTuning
{
	GENERATED_BODY()

	/** The attitude "level" means (deg; pitch, roll), before the rider's own
	 *  preference (level.pref.pitch / level.pref.roll) is added to it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level")
	FVector2D LevelReferenceDeg = FVector2D::ZeroVector;

	/** The most the rider may ask to be tilted off that reference (deg;
	 *  pitch, roll), either way: the range of level.pref.pitch and
	 *  level.pref.roll. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level", meta = (ClampMin = "0.0"))
	FVector2D MaxPreferenceDeg = FVector2D(8.0, 5.0);

	/** How much of the ground's own slope the legs are sent against at once
	 *  (1: all of it; 0: none, the loop below then does all the work). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Loop", meta = (ClampMin = "0.0"))
	float FeedForwardGain = 1.0f;

	/** The loop on what is left: proportional gain, integral gain (per s) and
	 *  the most the integral may hold (deg, per axis). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Loop", meta = (ClampMin = "0.0"))
	float ProportionalGain = 0.3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Loop", meta = (ClampMin = "0.0"))
	float IntegralGain = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Loop", meta = (ClampMin = "0.0"))
	float IntegralMaxDeg = 3.0f;

	/** An error smaller than this (deg) is left alone. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Loop", meta = (ClampMin = "0.0"))
	float DeadbandDeg = 0.15f;

	/** Low-pass time constants (s) of the attitude as read and of the ground's
	 *  slope as estimated from it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Loop", meta = (ClampMin = "0.0"))
	float AttitudeFilterSeconds = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Loop", meta = (ClampMin = "0.0"))
	float TerrainFilterSeconds = 0.15f;

	/** The most the legs correct (deg; pitch, roll), either way. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Reach", meta = (ClampMin = "0.0"))
	FVector2D MaxCorrectionDeg = FVector2D(15.0, 15.0);

	/** The correction changes no faster than this (deg/s), and so that no leg
	 *  is asked for more than LegRateFraction of the rate it can move at. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Reach", meta = (ClampMin = "0.0"))
	float MaxCorrectionRateDegPerSec = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Reach", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float LegRateFraction = 0.8f;

	/** The tilt policy: every deliberate change of the rider's attitude
	 *  (switching on and off, a slope past the legs' reach, a change of the
	 *  preference) is at most MaxTiltDeg off the reference where the ground
	 *  forces it, and moves within this rate (deg/s) and acceleration
	 *  (deg/s^2). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Tilt", meta = (ClampMin = "0.0"))
	float MaxTiltDeg = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Tilt", meta = (ClampMin = "0.0"))
	float TiltRateDegPerSec = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Tilt", meta = (ClampMin = "0.0"))
	float TiltAccelDegPerSec2 = 4.0f;

	/** A leg carrying the robot is kept this far (cm) inside its travel. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Reach", meta = (ClampMin = "0.0"))
	float LegRangeMarginCm = 0.5f;

	/** Within this (cm) of that, a leg's motion towards the limit slows. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Reach", meta = (ClampMin = "0.0"))
	float SoftZoneCm = 2.0f;

	/** A leg counts as carrying the robot once its wheel is within In (cm) of
	 *  the lowest wheel in the stance, and stops counting beyond Out. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Reach", meta = (ClampMin = "0.0"))
	float SupportBandInCm = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Reach", meta = (ClampMin = "0.0"))
	float SupportBandOutCm = 2.0f;

	/** A wheel the stance carries off the ground keeps at least this much of
	 *  its clearance (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Reach", meta = (ClampMin = "0.0"))
	float MinLiftedClearanceCm = 2.5f;

	/** The belly keeps at least this (cm) over the ground. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Reach", meta = (ClampMin = "0.0"))
	float MinBellyClearanceCm = 3.0f;

	/** How far (cm) the ride height may rise (Up) or sink (Down) from the
	 *  stance's to make a correction fit, and how fast it moves (cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Ride Height", meta = (ClampMin = "0.0"))
	float RideHeightUpCm = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Ride Height", meta = (ClampMin = "0.0"))
	float RideHeightDownCm = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Ride Height", meta = (ClampMin = "0.0"))
	float RideHeightRateCmPerSec = 3.0f;

	/** The drive-speed cap: the least share of the rider's speed it leaves
	 *  (never 0), the error (deg) where it starts to bind and where it is at
	 *  that floor, and how fast it lets go again (per s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Speed", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float MinSpeedScale = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Speed", meta = (ClampMin = "0.0"))
	float SpeedCapErrorDeg = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Speed", meta = (ClampMin = "0.0"))
	float SpeedCapErrorFullDeg = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Speed", meta = (ClampMin = "0.0"))
	float SpeedScaleRisePerSec = 0.5f;

	/** At rest: once the error has stayed under SettleErrorDeg for
	 *  SettleSeconds nothing more is commanded, until it passes
	 *  WakeErrorDeg or the rider moves the stick. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Rest", meta = (ClampMin = "0.0"))
	float SettleErrorDeg = 0.3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Rest", meta = (ClampMin = "0.0"))
	float SettleSeconds = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Rest", meta = (ClampMin = "0.0"))
	float WakeErrorDeg = 0.5f;

	/** Faults: the tilt (deg off the reference) reported as an error; how long
	 *  a correction that makes things worse may last (s); how long the attitude
	 *  may be missing before it is reported (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Faults", meta = (ClampMin = "0.0"))
	float FaultTiltDeg = 12.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Faults", meta = (ClampMin = "0.0"))
	float FaultSeconds = 1.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Faults", meta = (ClampMin = "0.0"))
	float MeasurementLossSeconds = 0.5f;

	/** The seat's tilt covers what the legs are too slow for: whether it does,
	 *  the most it adds (deg), how fast (deg/s), and the low-pass of the base
	 *  pitch it answers (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Seat")
	bool bSeatCoversTransients = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Seat", meta = (ClampMin = "0.0"))
	float SeatTrimMaxDeg = 8.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Seat", meta = (ClampMin = "0.0"))
	float SeatTrimRateDegPerSec = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level|Seat", meta = (ClampMin = "0.0"))
	float SeatFilterSeconds = 0.1f;
};
