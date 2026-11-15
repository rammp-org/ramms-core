// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "RammsCameraStreamSession.generated.h"

class UIntrinsicSceneCaptureComponent2D;

/**
 * What camera streams (URammsSensingBridge::StartCameraStreams) changed in a world's capture state, so that
 * StopCameraStreams undoes exactly that and nothing else: capture state is world-wide, and a session somebody
 * started for recording keeps running, with its own settings, when the streams stop. Play worlds only.
 */
UCLASS()
class URammsCameraStreamSession : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	/** The world's session, or null (not a play world). */
	static URammsCameraStreamSession* Get(const UWorld* World);

	/** Camera streams are on: StartCameraStreams ran and StopCameraStreams has not since. Rig cameras join capture
	 *  as they are built only while they are. */
	bool bActive = false;

	/** The streams started the capture session (it was not running), so stopping them stops it. */
	bool bStartedCapture = false;

	/** Capture channels the streams turned on (they were off): stopping turns them off again. */
	bool bTurnedOnColor = false;
	bool bTurnedOnDepth = false;
	bool bTurnedOnMotion = false;
	bool bTurnedOnSegmentation = false;

	/** The capture rate and disk-write setting before the streams changed them (set only when they did). */
	TOptional<int32> PriorRate;
	TOptional<bool>	 PriorSerialization;

	/** Cameras re-registered for a pass the capture subsystem then still did not make (its material did not load,
	 *  invalid depth intrinsics): not tried again, or every start would rebuild all their passes. */
	TSet<TWeakObjectPtr<const UIntrinsicSceneCaptureComponent2D>> CamerasWithoutMotionPass;
	TSet<TWeakObjectPtr<const UIntrinsicSceneCaptureComponent2D>> CamerasWithoutDepthPass;

	/** Forget everything the streams changed (after StopCameraStreams undid it). */
	void Reset();

protected:
	/** PIE and game worlds only. */
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;
};
