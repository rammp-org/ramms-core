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

	/** Unique on the actor: the created component's name and, for a camera, its capture id and stream name. */
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

	/** cm, relative to the parent (and socket). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Mount")
	FVector Location = FVector::ZeroVector;

	/** Relative to the parent (and socket). The sensor traces / looks along its local +X. */
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

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|ToF", meta = (ClampMin = "1.0"))
	float MaxRangeCm = 400.0f;

	// --- Sonar -----------------------------------------------------------------

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Sonar", meta = (ClampMin = "1.0"))
	float SonarMaxRangeCm = 500.0f;

	/** The cone's HALF-angle: URammsSonarSensorComponent::BeamHalfAngle (total beam width is twice this). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Sonar", meta = (ClampMin = "0.0", ClampMax = "90.0"))
	float SonarConeDeg = 15.0f;

	// --- Camera ----------------------------------------------------------------

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Camera", meta = (ClampMin = "1"))
	int32 Width = 640;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Camera", meta = (ClampMin = "1"))
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

	/** Base RMSS channel C. -1 = the lowest C whose four channels no stream source in the world uses yet. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensor|Camera", meta = (ClampMin = "-1", ClampMax = "65235"))
	int32 StreamChannel = -1;
};

/** What the rig built for one mount. */
USTRUCT()
struct RAMMSCORE_API FRammsBuiltSensor
{
	GENERATED_BODY()

	/** The mount as it was built. */
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
};

/**
 * A robot's sensors as data: a list of mounts, built into real sensor
 * components on the actor at BeginPlay (and at runtime through AddMount).
 *
 *  - ToF -> URammsToFSensorComponent, Sonar -> URammsSonarSensorComponent,
 *    IMU -> URammsIMUSensorComponent, each attached at the mount.
 *  - Camera -> a UIntrinsicSceneCaptureComponent2D at the mount, registered
 *    with the world's UCameraCaptureSubsystem, plus a URammsStreamSourceComponent
 *    "<Name>_stream" that forwards only that camera (exact filter) on its base
 *    channel C: colour C, depth C+100, motion C+200, mask C+300, under the
 *    stream ids camera/<Name>/color, camera/<Name>/depth, motion/<Name>,
 *    mask/<Name>.
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

	/** The sensors, built at BeginPlay. Names must be unique on the actor. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors")
	TArray<FRammsSensorMount> Mounts;

	/** Start camera capture at BeginPlay, with the channels this rig's cameras ask for. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Streaming")
	bool bStartCaptureOnBeginPlay = false;

	/** Start the RMSS server at BeginPlay (if it is not running yet) on StreamPort. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Streaming")
	bool bStartStreamServerOnBeginPlay = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Streaming", meta = (ClampMin = "1", ClampMax = "65535"))
	int32 StreamPort = 30030;

	/** Capture rate when this rig starts capture: 1 = every frame. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Streaming", meta = (ClampMin = "1"))
	int32 CaptureEveryNFrames = 1;

	/** When this rig starts capture, also write the frames to disk (the capture subsystem's serialization). Off:
	 *  the cameras only stream. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sensors|Streaming")
	bool bSaveCaptureToDisk = false;

	/**
	 * Add a mount (replacing one of the same name) and, once play has begun, build it now. Returns false if the
	 * mount has no name or could not be built (missing parent, name taken by another component).
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Sensors")
	bool AddMount(const FRammsSensorMount& Mount);

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

	const FRammsSensorMount*		 FindMount(FName MountName) const;
	const FRammsBuiltSensor*		 FindBuilt(FName MountName) const;
	const FRammsBuiltSensor*		 FindBuiltByComponent(const UActorComponent* Component) const;
	const TArray<FRammsBuiltSensor>& GetBuiltSensors() const { return Built; }

	/** "ToF", "Sonar", "IMU", "Camera". */
	static FString KindToString(ERammsSensorKind Kind);

	/** Case-insensitive "tof", "sonar", "imu", "camera". */
	static bool KindFromString(const FString& Text, ERammsSensorKind& OutKind);

	/** The lowest base channel C >= 0 such that none of C, C+100, C+200, C+300 is a channel of any stream source
	 *  already in World (each source counted with all four). */
	static int32 FindFreeStreamChannel(const UWorld* World);

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
	/** Build one mount now. False (with a warning) if it cannot be. */
	bool BuildMount(const FRammsSensorMount& Mount);

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
