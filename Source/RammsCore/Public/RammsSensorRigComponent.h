// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsSensorRigComponent.generated.h"

class UCameraCaptureSubsystem;
class UIntrinsicSceneCaptureComponent2D;
class USceneComponent;

/** What a sensor mount builds. */
UENUM(BlueprintType)
enum class ERammsSensorKind : uint8
{
	/** URammsToFSensorComponent: a single point, or a Rows x Columns grid of zones. */
	ToF,
	/** URammsSonarSensorComponent: the nearest hit in a cone. */
	Sonar,
	/** URammsIMUSensorComponent. */
	IMU,
	/** UIntrinsicSceneCaptureComponent2D registered for capture, plus a URammsStreamSourceComponent streaming it over RMSS. */
	Camera,
};

/**
 * One sensor on a robot: what it is, where it sits and how it is set up.
 *
 * Every sensor traces / looks along its local +X, so Rotation points +X where
 * the sensor should look. Only the fields of the mount's Kind are used.
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsSensorMount
{
	GENERATED_BODY()

	/** Unique on the actor: the created component's name and, for a camera, its capture id and stream name. 1 to 64
	 *  characters of A-Z a-z 0-9 _ - (no '.': it separates the parts of an object path), not ending in _depth, _dmv,
	 *  _seg or _stream (the names of a camera's capture passes and stream source). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor")
	FName Name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor")
	ERammsSensorKind Kind = ERammsSensorKind::ToF;

	/** Component on the actor to attach to (a MuJoCo body component, a mesh ...). None = the root component. A
	 *  mount whose parent does not exist is skipped with a warning. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Mount")
	FName ParentComponent;

	/** Optional socket or bone on the parent. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Mount")
	FName Socket;

	/** cm, relative to the parent (and socket); at most MaxMountOffsetCm along each axis. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Mount")
	FVector Location = FVector::ZeroVector;

	/** Relative to the parent (and socket). The sensor traces / looks along its local +X. Location, Rotation and the
	 *  ranges must be finite: a mount with a NaN or infinite one is not built. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Mount")
	FRotator Rotation = FRotator::ZeroRotator;

	// --- ToF -------------------------------------------------------------------

	/** Grid rows (1 x 1 = single point). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|ToF", meta = (ClampMin = "1", ClampMax = "64"))
	int32 Rows = 8;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|ToF", meta = (ClampMin = "1", ClampMax = "64"))
	int32 Columns = 8;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|ToF", meta = (ClampMin = "1.0", ClampMax = "120.0"))
	float HorizontalFovDeg = 45.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|ToF", meta = (ClampMin = "1.0", ClampMax = "120.0"))
	float VerticalFovDeg = 45.0f;

	/** cm, 1 to MaxSensorRangeCm (larger is clamped). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|ToF", meta = (ClampMin = "1.0", ClampMax = "100000.0"))
	float MaxRangeCm = 400.0f;

	// --- Sonar -----------------------------------------------------------------

	/** cm, 1 to MaxSensorRangeCm (larger is clamped). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Sonar", meta = (ClampMin = "1.0", ClampMax = "100000.0"))
	float SonarMaxRangeCm = 500.0f;

	/** The cone's HALF-angle: URammsSonarSensorComponent::BeamHalfAngle (total beam width is twice this). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Sonar", meta = (ClampMin = "0.0", ClampMax = "90.0"))
	float SonarConeDeg = 15.0f;

	// --- Camera ----------------------------------------------------------------

	/** Pixels, 1 to 4096 (larger is clamped). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Camera", meta = (ClampMin = "1", ClampMax = "4096"))
	int32 Width = 640;

	/** Pixels, 1 to 4096 (larger is clamped). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Camera", meta = (ClampMin = "1", ClampMax = "4096"))
	int32 Height = 480;

	/** Horizontal field of view; square pixels and a centred principal point make the rest of the intrinsics. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Camera", meta = (ClampMin = "1.0", ClampMax = "170.0"))
	float FovDeg = 87.0f;

	/** Stream colour (FrameRGB, channel C). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Camera")
	bool bColor = true;

	/** Stream depth (FrameDepth, channel C + 100). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Camera")
	bool bDepth = true;

	/** Stream motion vectors (FrameMotion, channel C + 200). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Camera")
	bool bMotion = true;

	/** Stream the segmentation class mask (FrameData mono8, channel C + 300). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Camera")
	bool bSegmentation = true;

	/** Base RMSS channel C. -1 = the lowest C whose four channels no stream source and no IMU stream in the world uses
	 *  yet. A C one of whose channels another source or an IMU stream already uses is replaced by a free one (with a
	 *  warning). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Camera", meta = (ClampMin = "-1", ClampMax = "65235"))
	int32 StreamChannel = -1;
};

/** What the rig built for one mount. */
USTRUCT()
struct RAMMSCORE_API FRammsBuiltSensor
{
	GENERATED_BODY()

	/** The mount as it was built (camera size clamped). */
	UPROPERTY()
	FRammsSensorMount Mount;

	/** The sensor: a ToF / Sonar / IMU component, or the camera's UIntrinsicSceneCaptureComponent2D. */
	UPROPERTY()
	TObjectPtr<USceneComponent> Component = nullptr;

	/** A camera's URammsStreamSourceComponent (named "<Name>_stream"); null otherwise. */
	UPROPERTY()
	TObjectPtr<UActorComponent> StreamSource = nullptr;

	/** A camera's base RMSS channel as resolved; -1 for other kinds. */
	UPROPERTY()
	int32 Channel = -1;

	/** A camera this rig registered with the capture subsystem (its render targets and capture passes exist). Cameras
	 *  register when camera streams start, not when they are built, so a rig baked into a pawn costs nothing until
	 *  then; and they are unregistered when the streams stop. False for a camera something else registered. */
	UPROPERTY()
	bool bCaptureRegistered = false;
};

/**
 * A robot's sensors as data: a list of mounts, built into real sensor
 * components on the actor at BeginPlay (and at runtime through AddMount).
 *
 *  - ToF -> URammsToFSensorComponent, Sonar -> URammsSonarSensorComponent,
 *    IMU -> URammsIMUSensorComponent, each attached at the mount.
 *  - Camera -> a UIntrinsicSceneCaptureComponent2D at the mount plus a
 *    URammsStreamSourceComponent "<Name>_stream" that forwards only that camera
 *    (exact filter) on its base channel C: colour C, depth C+100, motion C+200,
 *    mask C+300, under the stream ids camera/<Name>/color, camera/<Name>/depth,
 *    motion/<Name>, mask/<Name>. The camera is registered with the world's
 *    UCameraCaptureSubsystem (render targets, depth / motion / mask passes) only
 *    while camera streams are on (URammsSensingBridge::StartCameraStreams, or
 *    bStartCaptureOnBeginPlay; a camera built while they are joins at once),
 *    and unregistered when they stop (URammsSensingBridge::StopCameraStreams).
 *
 * Built components are transient: Mounts is the persistent description, baked
 * into a pawn Blueprint. URammsSensingBridge exposes the same over Remote
 * Control.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsSensorRigComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsSensorRigComponent();

	/** Largest camera width / height a mount builds; larger is clamped. */
	static constexpr int32 MaxCameraSize = 4096;

	/** Longest mount name. */
	static constexpr int32 MaxMountNameLength = 64;

	/** Farthest a mount may sit from its parent along any axis, cm. */
	static constexpr double MaxMountOffsetCm = 100000.0;

	/** Longest ToF / sonar range a mount builds, cm; larger is clamped. */
	static constexpr float MaxSensorRangeCm = 100000.0f;

	/** The sensors, built at BeginPlay. Names must be unique on the actor. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors")
	TArray<FRammsSensorMount> Mounts;

	/** Start camera capture at BeginPlay, with the channels this rig's cameras ask for (added to any already on). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Streaming")
	bool bStartCaptureOnBeginPlay = false;

	/** Start the RMSS server at BeginPlay (if it is not running yet) on StreamPort. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Streaming")
	bool bStartStreamServerOnBeginPlay = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Streaming", meta = (ClampMin = "1", ClampMax = "65535"))
	int32 StreamPort = 30030;

	/** Capture rate when this rig starts the capture session: 1 = every frame. A session already running keeps its
	 *  rate. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Streaming", meta = (ClampMin = "1"))
	int32 CaptureEveryNFrames = 1;

	/** When this rig starts the capture session, also write the frames to disk (the capture subsystem's
	 *  serialization). Off: a session this rig starts only streams. A session already running keeps its own setting
	 *  either way. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Streaming")
	bool bSaveCaptureToDisk = false;

	/**
	 * Add a mount (replacing one of the same name) and, once play has begun, build it now. Returns false, changing
	 * nothing, if the mount cannot be built (bad or reserved name, missing parent, a name taken by another
	 * component); the log says why.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensors")
	bool AddMount(const FRammsSensorMount& Mount);

	/** AddMount, saying why it failed. */
	bool TryAddMount(const FRammsSensorMount& Mount, FString& OutError);

	/** Remove a mount and destroy what it built. Returns false if there was no such mount. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensors")
	bool RemoveMount(FName MountName);

	/** The component a mount built (the camera component for a camera), or null. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Sensors")
	USceneComponent* GetSensorComponent(FName MountName) const;

	/** A camera mount's URammsStreamSourceComponent, or null. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Sensors")
	UActorComponent* GetStreamSource(FName MountName) const;

	/** A camera mount's base RMSS channel, or -1. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Sensors")
	int32 GetStreamChannel(FName MountName) const;

	/** The mounts currently built, in build order. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Sensors")
	TArray<FName> GetBuiltMountNames() const;

	/** Register this rig's built cameras with the world's capture subsystem (those not registered yet). Returns how
	 *  many it registered. */
	int32 RegisterCamerasForCapture();

	/** Unregister the cameras this rig registered with capture (their capture passes go with them). Returns how many
	 *  it unregistered. */
	int32 UnregisterCamerasFromCapture();

	const FRammsSensorMount*		 FindMount(FName MountName) const;
	const FRammsBuiltSensor*		 FindBuilt(FName MountName) const;
	const FRammsBuiltSensor*		 FindBuiltByComponent(const UActorComponent* Component) const;
	const TArray<FRammsBuiltSensor>& GetBuiltSensors() const { return Built; }

	/** "ToF", "Sonar", "IMU", "Camera". */
	static FString KindToString(ERammsSensorKind Kind);

	/** Case-insensitive "tof", "sonar", "imu", "camera". */
	static bool KindFromString(const FString& Text, ERammsSensorKind& OutKind);

	/** Whether Name may name a mount: 1 to MaxMountNameLength characters of A-Z a-z 0-9 _ - and not ending in a
	 *  reserved suffix (_depth, _dmv, _seg, _stream). Checks the string, so an over-long one never becomes an FName. */
	static bool IsValidMountName(const FString& Name, FString& OutError);

	/** The lowest base channel C >= 0 such that none of C, C+100, C+200, C+300 is a channel of any stream source
	 *  (each counted with all four) or IMU stream already in World; INDEX_NONE when there is none. */
	static int32 FindFreeStreamChannel(const UWorld* World);

	/** Whether none of base channel C's four channels is used by a stream source in World other than Ignore, or by an
	 *  IMU stream. */
	static bool IsStreamChannelFree(const UWorld* World, int32 BaseChannel, const UActorComponent* Ignore = nullptr);

	/** Register every rig camera in World with its capture subsystem (camera streams are starting). Returns how many
	 *  it registered. */
	static int32 RegisterRigCamerasForCapture(UWorld* World);

	/** Unregister every camera a rig in World registered with capture (camera streams are stopping). Returns how many
	 *  it unregistered. */
	static int32 UnregisterRigCamerasFromCapture(UWorld* World);

	/**
	 * Whether the names the capture subsystem gives Camera's passes ("<camera>_depth" / "_dmv" / "_seg") are free on
	 * its actor, or held by those passes themselves. Registering a camera whose pass name another live object holds
	 * would make NewObject replace that object (a fatal error for another class). OutConflict names the object.
	 */
	static bool AreCaptureHelperNamesFree(const UIntrinsicSceneCaptureComponent2D& Camera, FString* OutConflict = nullptr);

	/**
	 * Unregister a camera from capture (which destroys the depth / motion / segmentation captures the subsystem
	 * made for it, "<camera>_depth" / "_dmv" / "_seg") and free those names, so registering a camera of the same
	 * name again creates fresh captures instead of NewObject reusing the destroyed, not yet collected ones.
	 */
	static void UnregisterCaptureCamera(UCameraCaptureSubsystem& Capture, UIntrinsicSceneCaptureComponent2D* Camera);

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	/** Build one mount now (a camera on PreferredChannel if Mount asks for none and that one is free). False, with
	 *  the reason in OutError and the log, if it cannot be. */
	bool BuildMount(const FRammsSensorMount& Mount, FString& OutError, int32 PreferredChannel = -1);

	/** Everything BuildMount would refuse, checked without building. Replacing names a built entry whose components
	 *  are about to go, so its names do not count as taken. */
	bool CheckMount(const FRammsSensorMount& Mount, const FRammsBuiltSensor* Replacing, FString& OutError) const;

	/** Register Built[BuiltIndex]'s camera with Capture. False (with a warning) if its capture pass names are taken. */
	bool RegisterBuiltCamera(int32 BuiltIndex, UCameraCaptureSubsystem& Capture);

	/** Destroy what Built[BuiltIndex] made and forget it. */
	void DestroyBuilt(int32 BuiltIndex);

	USceneComponent* ResolveParent(FName ParentName) const;

	/** True when a live object of this name already exists in the owner. A destroyed one still waiting for garbage
	 *  collection is renamed out of the way instead. */
	bool IsNameTaken(FName ComponentName) const;

	/** Start the stream server and/or capture as the BeginPlay flags ask. */
	void StartStreamingFromBeginPlay();

	UPROPERTY(Transient)
	TArray<FRammsBuiltSensor> Built;
};
