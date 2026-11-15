// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class FJsonValue;
class UWorld;

/** What RammsSensingBridgeImu.cpp (the sensing bridge's IMU functions) gives the rest of the bridge. */
namespace RammsSensingBridgeImu
{
	/** GetStreamStatusJson's entries for World's IMU streams: {"actor", "camera": "", "target", "role": "imu",
	 *  "stream_id", "channel", "message": "FrameData", "fmt": "imu_f64x18"}. */
	void AppendImuStreams(const UWorld* World, TArray<TSharedPtr<FJsonValue>>& OutStreams);
} // namespace RammsSensingBridgeImu
