// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RammsSensingBridge.generated.h"

class AActor;
class UWorld;
struct FRammsSensorMount;

/**
 * What StartCameraStreamsJson (or a rig's BeginPlay) asks of the stream server and the capture subsystem. Capture
 * state is world-wide and other cameras may rely on it, so only what is asked for changes: channels are turned on,
 * never off, and the rate and disk writes change only when given.
 */
struct RAMMSCORE_API FRammsCameraStreamOptions
{
	/** The RMSS port, used when the server is not running yet. */
	int32 Port = 30030;
	/** Capture every N frames. Unset: the capture subsystem keeps its rate. */
	TOptional<int32> EveryNFrames;
	/** Capture channels to turn on (a channel already on stays on). */
	bool bColor = true;
	bool bDepth = true;
	bool bMotion = true;
	bool bSegmentation = true;
	/** Write the frames to disk too (the capture subsystem's serialization). Unset: a capture session this call starts
	 *  only streams; one already running keeps its setting. */
	TOptional<bool> SaveToDisk;
	/** Start the RMSS server if it is not running. */
	bool bStartServer = true;
	/** Turn the channels on, register the rig cameras and start capture. */
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
 *
 * Remote Control is reachable from the network, so what a caller can change is
 * bounded: sensor names are 1 to 64 characters of A-Z a-z 0-9 _ - . and do not
 * end in _depth / _dmv / _seg / _stream; cameras are at most 4096 x 4096
 * (larger is clamped); an actor has at most 16 rig sensors and a world at most
 * 8 rig cameras; and changes (AddSensorJson, RemoveSensor,
 * StartCameraStreamsJson, StopCameraStreams) need a play world (PIE or a game),
 * never the editor's level or a class default. Reads also answer in the editor
 * world.
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

	/** Add a sensor from a snake_case mount JSON (missing keys take the FRammsSensorMount defaults; a sonar also takes
	 *  "max_range_cm" / "cone_deg", the keys ListSensorsJson reports), adding a rig to the actor if it has none, and
	 *  build it now. {"ok", "name", "component_path"}; a camera adds "channel" and "streams" (and "warning" when it
	 *  could not have the channel it asked for). A failure changes nothing. Needs a play world. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString AddSensorJson(const FString& ActorPath, const FString& MountJson);

	/** Remove a rig sensor (and, for a camera, its stream source). False if no rig on the actor has it, or the actor
	 *  is not in a play world; a hand-placed component is not removed. */
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

	/** Start the RMSS server if it is not running (refusing a port another program listens on), turn on the capture
	 *  channels asked for (never off: other cameras may need them), register the rig cameras with capture and start
	 *  it. Disk writes: "save_to_disk" if given, else off for a capture session this starts and unchanged for one
	 *  already running; the rate changes only when "every_n_frames" is given. Options (all optional): {"port",
	 *  "every_n_frames", "color", "depth", "motion", "segmentation", "save_to_disk"}. Needs a play world. Returns the
	 *  stream status. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString StartCameraStreamsJson(const FString& OptionsJson);

	/** Stop capture; the server stays up. False if the world is not a play world or has no capture subsystem. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static bool StopCameraStreams();

	/** {"ok", "server": {running, port, connections}, "capture": {capturing, every_n_frames, color, depth, motion,
	 *  segmentation, save_to_disk, cameras}, "streams": [{actor, camera, role, stream_id, channel, message, fmt}]}.
	 *  "port" is the server's while it runs, else the default 30030. "streams" lists every registered camera and
	 *  every rig camera (registered or not yet). */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString GetStreamStatusJson();

	/** {"ok", "classes": [{value, name, coarse, coarse_id}], "coarse": [{id, name}], "reserved": [...]}: what the
	 *  values in a mask stream mean. Needs a play world (the class table lives in its segmentation subsystem). */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString GetSegmentationClassesJson();

	// --- IMU (RammsSensingBridgeImu.cpp) -----------------------------------------
	// Physics-rate inertial data of any body or IMU sensor (URammsPhysicsImuSubsystem). Units: cm, cm/s, cm/s^2, deg,
	// deg/s, deg/s^2, s. Accelerations read as an accelerometer does (gravity included: +|g| up at rest). A world
	// tracks at most 64 points (URammsPhysicsImuSubsystem::MaxTrackedPoints), of which these functions may hold 32
	// (MaxBridgePoints; the rest stay for IMU components); points they track are dropped after 10 s without a read.
	// Body and target names are at most 256 characters.

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
	 * "has_acceleration", "timestamp", "sim_time" (only with has_sample), "linear_acceleration" (sensor frame,
	 * gravity included), "angular_velocity" (sensor frame), "angular_acceleration" (sensor frame; with
	 * has_acceleration), "linear_velocity" (world), "location" (world), "orientation", "euler", "includes_gravity",
	 * "units"}.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString ReadBodyImuJson(const FString& ActorPath, const FString& Body, const FString& OptionsJson);

	/** {"ok", "actor", "bodies": [{name, engine, component, component_path, bone, default}]}: the bodies of the actor
	 *  (and of actors attached to it) a physics engine simulates, which ReadBodyImuJson reads at the physics rate, and
	 *  the actor's root; "default" marks the one Body "" reads. Any other component or bone reads Kinematic. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString ListImuBodiesJson(const FString& ActorPath);

	/**
	 * Every physics step's sample of an IMU point since a cursor (up to 1024 kept: over 2 s of MuJoCo's 500 Hz).
	 * Target: an IMU sensor's name (as ListSensorsJson names it), else a body as ReadBodyImuJson takes it ("" the
	 * default body). Options (all optional): "offset", "rotation", "source" as ReadBodyImuJson's (body targets only);
	 * "since": the "next" of the last call (omitted or -1: start now, no samples, only "next"); "max": at most this
	 * many samples (default 1024).
	 * {"ok", "actor", "target", "body", "source", "units", "samples": [{"seq", "sim_time" (s),
	 * "linear_acceleration" (sensor frame, cm/s^2, gravity included), "angular_velocity" (sensor frame, deg/s),
	 * "angular_acceleration" (sensor frame, deg/s^2), "linear_velocity" (world, cm/s), "orientation" (world
	 * {x, y, z, w})}] oldest first, "next", "dropped" (samples lost between since and the oldest kept), "rate_hz"
	 * (from the samples' sim_time; 0 with fewer than two)}.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString ReadImuSamplesJson(const FString& ActorPath, const FString& Target, const FString& OptionsJson);

	/**
	 * Stream every physics step's sample of an IMU point over RMSS (needs a play world; starts the RMSS server if it
	 * is not running). Target and options as ReadImuSamplesJson's, plus "port" (30030) and "channel" (-1: the lowest
	 * free channel from 1000). Each game tick with new samples sends one FrameData message on the channel: fmt
	 * "imu_f64x18", w = samples, h = 1, 18 little-endian float64 per sample (seq, sim_time, linear acceleration xyz,
	 * angular velocity xyz, angular acceleration xyz, linear velocity xyz, orientation xyzw), stream_id
	 * "imu/<actor name>/<target>" (the body's name for Target ""). Starting a stream already running restarts it.
	 * At most 8 streams per world (URammsImuStreamSubsystem::MaxStreams), each on one of the bridge's points; a stream
	 * no RMSS client has subscribed to (its channel, or every channel) for 30 s stops by itself. "port" is clamped to
	 * 1..65535.
	 * {"ok", "actor", "target", "body", "source", "stream_id", "channel", "message": "FrameData", "fmt", "server"}.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static FString StartImuStreamJson(const FString& ActorPath, const FString& Target, const FString& OptionsJson);

	/** Stop an IMU stream StartImuStreamJson started. False when there is none for that actor and target. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensing")
	static bool StopImuStream(const FString& ActorPath, const FString& Target);

	// --- C++ ---------------------------------------------------------------------

	/** The world these functions act on: the play world, else the editor world. */
	static UWorld* GetSensingWorld();

	/** An actor by full object path, or by name in the sensing world. Never a class default object or archetype. */
	static AActor* FindActor(const FString& ActorPath);

	/** Whether the bridge may change sensing on Actor: it is in a play world (PIE or a game) and not a template.
	 *  Else OutError says so. */
	static bool CanChangeSensing(const AActor* Actor, FString& OutError);

	/** Whether World is a play world (PIE or a game). Else OutError says so. */
	static bool IsPlayWorld(const UWorld* World, FString& OutError);

	/** The bridge's limits (see the class comment). */
	static constexpr int32 MaxRigSensorsPerActor = 16;
	static constexpr int32 MaxRigCamerasPerWorld = 8;

	/** Parse a mount JSON (see AddSensorJson) onto OutMount, which keeps its values for missing keys. */
	static bool ParseMountJson(const FString& MountJson, FRammsSensorMount& OutMount, FString& OutError);

	/** StartCameraStreamsJson on a given world; returns the status JSON. */
	static FString StartCameraStreams(UWorld* World, const FRammsCameraStreamOptions& Options);

	/** GetStreamStatusJson on a given world. */
	static FString StreamStatusJsonFor(UWorld* World);
};
