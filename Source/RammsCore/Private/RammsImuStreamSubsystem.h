// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsImuTypes.h"
#include "Subsystems/WorldSubsystem.h"
#include "RammsImuStreamSubsystem.generated.h"

class AActor;

/** The record layout an IMU stream sends and ReadImuSamplesJson reads: 18 values per sample. */
namespace RammsImuRecord
{
	constexpr int32 NumFields = 18;

	/** seq, sim_time, ax, ay, az, gx, gy, gz, alpha_x, alpha_y, alpha_z, vx, vy, vz, qx, qy, qz, qw. */
	extern const TCHAR* const FieldNames[NumFields];

	/**
	 * One sample as the record: 0 seq, 1 sim_time (s), 2-4 linear acceleration (sensor frame, cm/s^2, gravity
	 * included; zero when the sample has none yet), 5-7 angular velocity (sensor frame, deg/s), 8-10 angular
	 * acceleration (sensor frame, deg/s^2; zero when none yet), 11-13 linear velocity (world, cm/s), 14-17 orientation
	 * x, y, z, w (world).
	 */
	void ToRecord(int64 Seq, const FRammsImuSample& Sample, double (&OutRecord)[NumFields]);
} // namespace RammsImuRecord

/** One continuous IMU feed over RMSS (URammsSensingBridge::StartImuStreamJson). */
struct FRammsImuStream
{
	TWeakObjectPtr<AActor> Actor;
	FString				   ActorPath;
	FString				   ActorName;
	FString				   Target;		 // as asked for ("" = the default body)
	FString				   BodyName;	 // what the point rides on
	FString				   StreamId;	 // imu/<actor name>/<target, or the body for "">
	int32				   Handle = 0;	 // its URammsPhysicsImuSubsystem point
	int32				   Channel = -1; // RMSS channel
	int64				   Cursor = -1;	 // the next sample sequence to send (-1: from now)
};

/**
 * The world's IMU streams: each game tick, every stream with new physics-step samples sends them as one RMSS
 * FrameData message ("imu_f64x18") through URammsStreamingSubsystem. Play worlds only.
 */
UCLASS()
class URammsImuStreamSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	/** The lowest channel a stream takes when it is not given one. */
	static constexpr int32 FirstChannel = 1000;

	/** Start sending the samples of a tracked point (the subsystem untracks it when the stream stops). Channel -1:
	 *  the lowest free one from FirstChannel. Restarts a stream of the same actor and target. False (OutError) when
	 *  the channel is taken by another IMU stream or out of range. */
	bool StartStream(const FRammsImuStream& Stream, FString& OutError, FRammsImuStream& OutStarted);

	/** Stop the stream of ActorPath / Target. False when there is none. */
	bool StopStream(const FString& ActorPath, const FString& Target);

	const TArray<FRammsImuStream>& GetStreams() const { return Streams; }

	// UTickableWorldSubsystem
	virtual void	Deinitialize() override;
	virtual void	Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool	IsTickable() const override { return Streams.Num() > 0; }
	// MuJoCo steps on while the game is paused: keep its samples flowing.
	virtual bool IsTickableWhenPaused() const override { return true; }

protected:
	/** Play worlds (PIE, game) only. */
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;

private:
	/** Send what the stream's point gathered since its cursor; false when the point is gone. */
	bool Pump(FRammsImuStream& Stream);

	/** Untrack a stream's point. */
	void Release(FRammsImuStream& Stream);

	TArray<FRammsImuStream> Streams;
};
