// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class UActorComponent;
class UWorld;

/**
 * The RMSS channels a world's streams use, for picking new ones: camera stream sources (each takes C, C+100, C+200
 * and C+300) and IMU streams (URammsImuStreamSubsystem, one channel each). A camera's channels and an IMU stream's
 * channel never coincide, whichever was started first.
 */
namespace RammsStreamChannels
{
	/** Every channel a camera stream source (URammsStreamSourceComponent) in World uses, each source counted with
	 *  all four roles' channels, but IgnoreSource's. */
	TSet<int32> CameraSourceChannels(const UWorld* World, const UActorComponent* IgnoreSource = nullptr);

	/** The channels of World's IMU streams, but the one of IgnoreActorPath / IgnoreTarget (a stream being restarted;
	 *  "" ignores none). */
	TSet<int32> ImuStreamChannels(const UWorld* World, const FString& IgnoreActorPath = FString(),
		const FString& IgnoreTarget = FString());

	/** Whether base channel C is valid and none of its four channels is used by a camera stream source (but
	 *  IgnoreSource) or an IMU stream in World. */
	bool IsCameraBaseFree(const UWorld* World, int32 BaseChannel, const UActorComponent* IgnoreSource = nullptr);

	/** The lowest base channel C >= 0 that IsCameraBaseFree; INDEX_NONE when there is none. */
	int32 FindFreeCameraBase(const UWorld* World);

	/**
	 * The channel for the IMU stream of ActorPath / Target: Requested (>= 0) if no camera stream source uses it (other
	 * IMU streams are checked when it starts); else -- Requested < 0 -- the one the stream already has when it is
	 * restarted and still free of cameras, or the lowest from URammsImuStreamSubsystem::FirstChannel on that no
	 * camera source and no other IMU stream uses. False, with the reason in OutError, when there is none.
	 */
	bool PickImuChannel(const UWorld* World, const FString& ActorPath, const FString& Target, int32 Requested,
		int32& OutChannel, FString& OutError);
} // namespace RammsStreamChannels
