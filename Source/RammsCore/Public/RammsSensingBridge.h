// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RammsSensingBridge.generated.h"

class AActor;
class UWorld;
struct FRammsSensorMount;

/** What StartCameraStreamsJson (or a rig's BeginPlay) asks of the stream server and the capture subsystem. */
struct RAMMSCORE_API FRammsCameraStreamOptions
{
	int32 Port = 30030;
	int32 EveryNFrames = 1;
	bool  bColor = true;
	bool  bDepth = true;
	bool  bMotion = true;
	bool  bSegmentation = true;
	/** Leave the capture subsystem's disk serialization on. Off: the cameras only stream. */
	bool bSaveToDisk = false;
	/** Start the RMSS server if it is not running. */
	bool bStartServer = true;
	/** Set the capture channels and start capture. */
	bool bStartCapture = true;
};

/**
 * Robot sensing for programs outside the engine, over the Remote Control API
 * (ramms-tools calls these on the CDO /Script/RammsCore.Default__RammsSensingBridge).
 *
 * Plain strings in, JSON strings out, like URammsWorldGenBridge. Every JSON
 * result has "ok"; failures add "error". Actors are addressed by full object
 * path (as URammsCoreBridge::FindActors returns; an actor's name also works),
 * and "" where allowed means every actor. The world is the play world (PIE or
 * game), else the editor world, the rule URammsCoreBridge uses.
 *
 * Sensors are the components a URammsSensorRigComponent built ("rig": true)
 * and ToF / Sonar / IMU components and capture-registered cameras placed by
 * hand ("rig": false, named by their component name).
 */
UCLASS()
class RAMMSCORE_API URammsSensingBridge : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** {"ok", "sensors": [{actor, actor_name, name, kind, component_path, parent, socket, rig, location, rotation,
	 *  settings, streams}]}: location / rotation world-space (cm, degrees); streams only for cameras. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString ListSensorsJson(const FString& ActorPath);

	/** Add a sensor from a snake_case mount JSON (missing keys take the FRammsSensorMount defaults), adding a
	 *  rig to the actor if it has none, and build it now. {"ok", "name", "component_path"}; a camera adds
	 *  "channel" and "streams". */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString AddSensorJson(const FString& ActorPath, const FString& MountJson);

	/** Remove a rig sensor (and, for a camera, its stream source). False if no rig on the actor has it; a
	 *  hand-placed component is not removed. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static bool RemoveSensor(const FString& ActorPath, const FString& Name);

	/** The latest sample of one ToF / Sonar / IMU; bMeasureNow takes a fresh ToF / Sonar measurement first (CPU
	 *  trace; the IMU has no on-demand measurement and reports its latest). */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString ReadSensorJson(const FString& ActorPath, const FString& Name, bool bMeasureNow);

	/** {"ok", "actor", "readings": [...]}: the latest sample of every ToF / Sonar / IMU on the actor ("" = every
	 *  actor; each reading also names its "actor"). */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString ReadSensorsJson(const FString& ActorPath);

	/**
	 * An IMU reading of any body, no IMU component needed: a virtual sensor on Body, sampled by the physics engine
	 * that simulates it at the physics rate (URammsPhysicsImuSubsystem). Body: a MuJoCo body name, a component name
	 * or a bone name on the actor (or on an actor attached to it); "" the actor's main body (its root, else the first
	 * body an engine simulates). Options (all optional): {"offset": [x, y, z] (cm, body frame), "rotation": [pitch,
	 * yaw, roll] (the sensor frame in the body frame, deg), "source": "auto|chaos|mujoco|kinematic"}.
	 *
	 * The first call starts tracking the point (its first answer may have no sample or no acceleration yet:
	 * "has_sample" / "has_acceleration" false); later calls read it, and it is dropped once not read for 10 s.
	 * {"ok", "name", "kind": "IMU", "actor", "body", "component_path", "bone", "source", "has_sample",
	 * "has_acceleration", "timestamp", "sim_time", "linear_acceleration" (sensor frame, gravity included),
	 * "angular_velocity" (sensor frame), "angular_acceleration" (sensor frame; with has_acceleration),
	 * "linear_velocity" (world), "location" (world), "orientation", "euler", "includes_gravity", "units"}.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString ReadBodyImuJson(const FString& ActorPath, const FString& Body, const FString& OptionsJson);

	/** {"ok", "actor", "bodies": [{name, engine, component, component_path, bone, default}]}: the bodies of the actor
	 *  (and of actors attached to it) a physics engine simulates, which ReadBodyImuJson reads at the physics rate, and
	 *  the actor's root; "default" marks the one Body "" reads. Any other component or bone reads Kinematic. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString ListImuBodiesJson(const FString& ActorPath);

	/** Start the RMSS server if it is not running, set the capture channels and segmentation, turn disk writes
	 *  off unless asked, set the rate and start capture. Options (all optional): {"port", "every_n_frames",
	 *  "color", "depth", "motion", "segmentation", "save_to_disk"}. Returns the stream status. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString StartCameraStreamsJson(const FString& OptionsJson);

	/** Stop capture; the server stays up. False if the world has no capture subsystem. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static bool StopCameraStreams();

	/** {"ok", "server": {running, port, connections}, "capture": {capturing, every_n_frames, color, depth, motion,
	 *  segmentation, save_to_disk, cameras}, "streams": [{actor, camera, role, stream_id, channel, message, fmt}]} */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString GetStreamStatusJson();

	/** {"ok", "classes": [{value, name, coarse, coarse_id}], "coarse": [{id, name}], "reserved": [...]}: what the
	 *  values in a mask stream mean. Needs a play world (the class table lives in its segmentation subsystem). */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString GetSegmentationClassesJson();

	// --- C++ ---------------------------------------------------------------------

	/** The world these functions act on: the play world, else the editor world. */
	static UWorld* GetSensingWorld();

	/** An actor by full object path, or by name in the sensing world. */
	static AActor* FindActor(const FString& ActorPath);

	/** Parse a mount JSON (see AddSensorJson) onto OutMount, which keeps its values for missing keys. */
	static bool ParseMountJson(const FString& MountJson, FRammsSensorMount& OutMount, FString& OutError);

	/** StartCameraStreamsJson on a given world; returns the status JSON. */
	static FString StartCameraStreams(UWorld* World, const FRammsCameraStreamOptions& Options);

	/** GetStreamStatusJson on a given world. */
	static FString StreamStatusJsonFor(UWorld* World);
};
