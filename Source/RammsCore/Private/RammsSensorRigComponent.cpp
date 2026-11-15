// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsSensorRigComponent.h"

#include "CameraCaptureSubsystem.h"
#include "CameraIntrinsics.h"
#include "Components/SceneComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "IntrinsicSceneCaptureComponent2D.h"
#include "RammsCameraStreams.h"
#include "RammsIMUSensorComponent.h"
#include "RammsSensingBridge.h"
#include "RammsSonarSensorComponent.h"
#include "RammsStreamSourceComponent.h"
#include "RammsToFSensorComponent.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsSensorRig, Log, All);

namespace RammsSensorRig
{
	/** A pinhole camera of Width x Height pixels whose horizontal field of view is FovDeg: square pixels, the
	 *  principal point at the centre. */
	FCameraIntrinsics MakeIntrinsics(int32 Width, int32 Height, float FovDeg)
	{
		FCameraIntrinsics Intrinsics;
		Intrinsics.ImageWidth = FMath::Max(1, Width);
		Intrinsics.ImageHeight = FMath::Max(1, Height);
		const double HalfFovRad = FMath::DegreesToRadians(FMath::Clamp(static_cast<double>(FovDeg), 1.0, 170.0) * 0.5);
		const double Focal = (Intrinsics.ImageWidth * 0.5) / FMath::Tan(HalfFovRad);
		Intrinsics.FocalLengthX = static_cast<float>(Focal);
		Intrinsics.FocalLengthY = static_cast<float>(Focal);
		Intrinsics.PrincipalPointX = Intrinsics.ImageWidth * 0.5f;
		Intrinsics.PrincipalPointY = Intrinsics.ImageHeight * 0.5f;
		Intrinsics.bMaintainYAxis = false;
		Intrinsics.PresetName = TEXT("SensorRig");
		return Intrinsics;
	}

	FName StreamSourceNameFor(FName MountName)
	{
		return FName(*(MountName.ToString() + TEXT("_stream")));
	}

	/** Give a destroyed object a throwaway name, so one of the same name can be made again before garbage
	 *  collection gets to the old one: NewObject with the name of a destroyed object of the same class would
	 *  rebuild that object in place, and of another class is fatal. */
	void RetireName(UObject* Object)
	{
		if (!Object)
		{
			return;
		}
		UObject*	Outer = Object->GetOuter();
		const FName Retired = MakeUniqueObjectName(Outer, Object->GetClass(), FName(*(Object->GetName() + TEXT("_removed"))));
		Object->Rename(*Retired.ToString(), nullptr, REN_DontCreateRedirectors | REN_NonTransactional | REN_DoNotDirty);
	}

	/** The captures UCameraCaptureSubsystem makes for a registered camera, named after it. */
	const TCHAR* const CaptureHelperSuffixes[] = { TEXT("_depth"), TEXT("_dmv"), TEXT("_seg") };
} // namespace RammsSensorRig

URammsSensorRigComponent::URammsSensorRigComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void URammsSensorRigComponent::BeginPlay()
{
	Super::BeginPlay();

	for (const FRammsSensorMount& Mount : Mounts)
	{
		BuildMount(Mount);
	}

	if (bStartStreamServerOnBeginPlay || bStartCaptureOnBeginPlay)
	{
		StartStreamingFromBeginPlay();
	}
}

void URammsSensorRigComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Only when the rig goes and its actor stays (the component removed on its own) is there anything to take
	// down: the built components would otherwise outlive the rig that describes them. An actor being destroyed
	// takes them with it, and the capture subsystem drops cameras that have gone.
	const AActor* Owner = GetOwner();
	if (EndPlayReason == EEndPlayReason::Destroyed && Owner && !Owner->IsActorBeingDestroyed())
	{
		for (int32 i = Built.Num() - 1; i >= 0; --i)
		{
			DestroyBuilt(i);
		}
	}
	Built.Reset();

	Super::EndPlay(EndPlayReason);
}

void URammsSensorRigComponent::StartStreamingFromBeginPlay()
{
	FRammsCameraStreamOptions Options;
	Options.Port = StreamPort;
	Options.EveryNFrames = CaptureEveryNFrames;
	Options.bSaveToDisk = bSaveCaptureToDisk;
	Options.bStartServer = bStartStreamServerOnBeginPlay;
	Options.bStartCapture = bStartCaptureOnBeginPlay;

	// Capture what this rig's cameras stream; nothing more.
	Options.bColor = Options.bDepth = Options.bMotion = Options.bSegmentation = false;
	bool bAnyCamera = false;
	for (const FRammsBuiltSensor& Entry : Built)
	{
		if (Entry.Mount.Kind == ERammsSensorKind::Camera)
		{
			bAnyCamera = true;
			Options.bColor |= Entry.Mount.bColor;
			Options.bDepth |= Entry.Mount.bDepth;
			Options.bMotion |= Entry.Mount.bMotion;
			Options.bSegmentation |= Entry.Mount.bSegmentation;
		}
	}
	if (bStartCaptureOnBeginPlay && !bAnyCamera)
	{
		UE_LOG(LogRammsSensorRig, Warning, TEXT("%s: bStartCaptureOnBeginPlay, but the rig built no camera; capture not started"),
			*GetPathName());
		Options.bStartCapture = false;
	}
	if (!Options.bStartServer && !Options.bStartCapture)
	{
		return;
	}

	const FString Status = URammsSensingBridge::StartCameraStreams(GetWorld(), Options);
	UE_LOG(LogRammsSensorRig, Log, TEXT("%s: streaming started at BeginPlay: %s"), *GetPathName(), *Status);
}

// ---------------------------------------------------------------------------
// Mounts
// ---------------------------------------------------------------------------

bool URammsSensorRigComponent::AddMount(const FRammsSensorMount& Mount)
{
	if (Mount.Name.IsNone())
	{
		UE_LOG(LogRammsSensorRig, Warning, TEXT("%s: AddMount needs a name"), *GetPathName());
		return false;
	}

	// Replace the description first, so a rig that has not begun play builds the new one at BeginPlay.
	const int32 Existing = Mounts.IndexOfByPredicate([&Mount](const FRammsSensorMount& M) { return M.Name == Mount.Name; });
	if (Existing != INDEX_NONE)
	{
		Mounts[Existing] = Mount;
	}
	else
	{
		Mounts.Add(Mount);
	}

	if (!HasBegunPlay())
	{
		return true;
	}

	const int32 BuiltIndex = Built.IndexOfByPredicate([&Mount](const FRammsBuiltSensor& B) { return B.Mount.Name == Mount.Name; });
	if (BuiltIndex != INDEX_NONE)
	{
		DestroyBuilt(BuiltIndex);
	}
	return BuildMount(Mount);
}

bool URammsSensorRigComponent::RemoveMount(FName MountName)
{
	const int32 Removed = Mounts.RemoveAll([MountName](const FRammsSensorMount& M) { return M.Name == MountName; });
	const int32 BuiltIndex = Built.IndexOfByPredicate([MountName](const FRammsBuiltSensor& B) { return B.Mount.Name == MountName; });
	if (BuiltIndex != INDEX_NONE)
	{
		DestroyBuilt(BuiltIndex);
	}
	return Removed > 0 || BuiltIndex != INDEX_NONE;
}

USceneComponent* URammsSensorRigComponent::GetSensorComponent(FName MountName) const
{
	const FRammsBuiltSensor* Entry = FindBuilt(MountName);
	return Entry && IsValid(Entry->Component) ? Entry->Component.Get() : nullptr;
}

UActorComponent* URammsSensorRigComponent::GetStreamSource(FName MountName) const
{
	const FRammsBuiltSensor* Entry = FindBuilt(MountName);
	return Entry && IsValid(Entry->StreamSource) ? Entry->StreamSource.Get() : nullptr;
}

int32 URammsSensorRigComponent::GetStreamChannel(FName MountName) const
{
	const FRammsBuiltSensor* Entry = FindBuilt(MountName);
	return Entry ? Entry->Channel : -1;
}

TArray<FName> URammsSensorRigComponent::GetBuiltMountNames() const
{
	TArray<FName> Names;
	for (const FRammsBuiltSensor& Entry : Built)
	{
		Names.Add(Entry.Mount.Name);
	}
	return Names;
}

const FRammsSensorMount* URammsSensorRigComponent::FindMount(FName MountName) const
{
	return Mounts.FindByPredicate([MountName](const FRammsSensorMount& M) { return M.Name == MountName; });
}

const FRammsBuiltSensor* URammsSensorRigComponent::FindBuilt(FName MountName) const
{
	return Built.FindByPredicate([MountName](const FRammsBuiltSensor& B) { return B.Mount.Name == MountName; });
}

const FRammsBuiltSensor* URammsSensorRigComponent::FindBuiltByComponent(const UActorComponent* Component) const
{
	if (!Component)
	{
		return nullptr;
	}
	return Built.FindByPredicate([Component](const FRammsBuiltSensor& B) {
		return B.Component.Get() == Component || B.StreamSource.Get() == Component;
	});
}

// ---------------------------------------------------------------------------
// Names and channels
// ---------------------------------------------------------------------------

FString URammsSensorRigComponent::KindToString(ERammsSensorKind Kind)
{
	switch (Kind)
	{
		case ERammsSensorKind::Sonar:
			return TEXT("Sonar");
		case ERammsSensorKind::IMU:
			return TEXT("IMU");
		case ERammsSensorKind::Camera:
			return TEXT("Camera");
		case ERammsSensorKind::ToF:
		default:
			return TEXT("ToF");
	}
}

bool URammsSensorRigComponent::KindFromString(const FString& Text, ERammsSensorKind& OutKind)
{
	const FString Trimmed = Text.TrimStartAndEnd();
	if (Trimmed.Equals(TEXT("tof"), ESearchCase::IgnoreCase))
	{
		OutKind = ERammsSensorKind::ToF;
		return true;
	}
	if (Trimmed.Equals(TEXT("sonar"), ESearchCase::IgnoreCase))
	{
		OutKind = ERammsSensorKind::Sonar;
		return true;
	}
	if (Trimmed.Equals(TEXT("imu"), ESearchCase::IgnoreCase))
	{
		OutKind = ERammsSensorKind::IMU;
		return true;
	}
	if (Trimmed.Equals(TEXT("camera"), ESearchCase::IgnoreCase))
	{
		OutKind = ERammsSensorKind::Camera;
		return true;
	}
	return false;
}

int32 URammsSensorRigComponent::FindFreeStreamChannel(const UWorld* World)
{
	TSet<int32> Used;
	if (World)
	{
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			const AActor* Actor = *It;
			if (!IsValid(Actor))
			{
				continue;
			}
			TInlineComponentArray<URammsStreamSourceComponent*> Sources(Actor);
			for (const URammsStreamSourceComponent* Source : Sources)
			{
				if (!IsValid(Source))
				{
					continue;
				}
				for (const ERammsCameraStreamRole Role : FRammsCameraStreams::AllRoles())
				{
					Used.Add(Source->ChannelID + FRammsCameraStreams::ChannelOffset(Role));
				}
			}
		}
	}

	const int32 Highest = MAX_uint16 - FRammsCameraStreams::MaxChannelOffset;
	for (int32 Base = 0; Base <= Highest; ++Base)
	{
		bool bFree = true;
		for (const ERammsCameraStreamRole Role : FRammsCameraStreams::AllRoles())
		{
			if (Used.Contains(Base + FRammsCameraStreams::ChannelOffset(Role)))
			{
				bFree = false;
				break;
			}
		}
		if (bFree)
		{
			return Base;
		}
	}
	return 0;
}

void URammsSensorRigComponent::UnregisterCaptureCamera(UCameraCaptureSubsystem& Capture, UIntrinsicSceneCaptureComponent2D* Camera)
{
	if (!Camera)
	{
		return;
	}
	Capture.UnregisterCamera(Camera);
	if (AActor* CameraOwner = Camera->GetOwner())
	{
		for (const TCHAR* Suffix : RammsSensorRig::CaptureHelperSuffixes)
		{
			UObject* Helper = FindObjectFast<UObject>(CameraOwner, FName(*(Camera->GetName() + Suffix)));
			if (Helper && !IsValid(Helper))
			{
				RammsSensorRig::RetireName(Helper);
			}
		}
	}
}

// ---------------------------------------------------------------------------
// Building
// ---------------------------------------------------------------------------

USceneComponent* URammsSensorRigComponent::ResolveParent(FName ParentName) const
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return nullptr;
	}
	if (ParentName.IsNone())
	{
		return Owner->GetRootComponent();
	}
	TInlineComponentArray<USceneComponent*> SceneComponents(Owner);
	for (USceneComponent* Candidate : SceneComponents)
	{
		if (IsValid(Candidate) && Candidate->GetFName() == ParentName)
		{
			return Candidate;
		}
	}
	return nullptr;
}

bool URammsSensorRigComponent::IsNameTaken(FName ComponentName) const
{
	AActor*	 Owner = GetOwner();
	UObject* Existing = Owner ? FindObjectFast<UObject>(Owner, ComponentName) : nullptr;
	if (Existing && !IsValid(Existing))
	{
		// Destroyed, not yet collected: move it aside rather than have NewObject rebuild it in place.
		RammsSensorRig::RetireName(Existing);
		return false;
	}
	return Existing != nullptr;
}

bool URammsSensorRigComponent::BuildMount(const FRammsSensorMount& Mount)
{
	AActor* Owner = GetOwner();
	UWorld* World = GetWorld();
	if (!Owner || !World)
	{
		return false;
	}
	if (Mount.Name.IsNone())
	{
		UE_LOG(LogRammsSensorRig, Warning, TEXT("%s: a mount has no name; skipped"), *GetPathName());
		return false;
	}
	if (FindBuilt(Mount.Name))
	{
		UE_LOG(LogRammsSensorRig, Warning, TEXT("%s: mount '%s' is listed twice; the second is skipped"), *GetPathName(),
			*Mount.Name.ToString());
		return false;
	}

	USceneComponent* Parent = ResolveParent(Mount.ParentComponent);
	if (!Parent)
	{
		UE_LOG(LogRammsSensorRig, Warning, TEXT("%s: mount '%s' skipped: %s has no component '%s'"), *GetPathName(),
			*Mount.Name.ToString(), *Owner->GetName(),
			Mount.ParentComponent.IsNone() ? TEXT("(root)") : *Mount.ParentComponent.ToString());
		return false;
	}
	if (!Mount.Socket.IsNone() && !Parent->DoesSocketExist(Mount.Socket))
	{
		UE_LOG(LogRammsSensorRig, Warning, TEXT("%s: mount '%s': %s has no socket '%s'; attached at its origin"),
			*GetPathName(), *Mount.Name.ToString(), *Parent->GetName(), *Mount.Socket.ToString());
	}

	const bool	bCamera = Mount.Kind == ERammsSensorKind::Camera;
	const FName SourceName = RammsSensorRig::StreamSourceNameFor(Mount.Name);
	if (IsNameTaken(Mount.Name) || (bCamera && IsNameTaken(SourceName)))
	{
		UE_LOG(LogRammsSensorRig, Warning, TEXT("%s: mount '%s' skipped: %s already has an object of that name"),
			*GetPathName(), *Mount.Name.ToString(), *Owner->GetName());
		return false;
	}

	USceneComponent* Sensor = nullptr;
	switch (Mount.Kind)
	{
		case ERammsSensorKind::ToF:
		{
			URammsToFSensorComponent* ToF = NewObject<URammsToFSensorComponent>(Owner, Mount.Name, RF_Transient);
			ToF->GridRows = FMath::Clamp(Mount.Rows, 1, 64);
			ToF->GridColumns = FMath::Clamp(Mount.Columns, 1, 64);
			ToF->SensorMode = (ToF->GridRows * ToF->GridColumns > 1) ? EToFSensorMode::Grid : EToFSensorMode::SinglePoint;
			ToF->HorizontalFOV = FMath::Clamp(Mount.HorizontalFovDeg, 1.0f, 120.0f);
			ToF->VerticalFOV = FMath::Clamp(Mount.VerticalFovDeg, 1.0f, 120.0f);
			ToF->MaxRange = FMath::Max(1.0f, Mount.MaxRangeCm);
			Sensor = ToF;
			break;
		}
		case ERammsSensorKind::Sonar:
		{
			URammsSonarSensorComponent* Sonar = NewObject<URammsSonarSensorComponent>(Owner, Mount.Name, RF_Transient);
			Sonar->MaxRange = FMath::Max(1.0f, Mount.SonarMaxRangeCm);
			Sonar->BeamHalfAngle = FMath::Clamp(Mount.SonarConeDeg, 0.0f, 90.0f);
			Sensor = Sonar;
			break;
		}
		case ERammsSensorKind::IMU:
		{
			Sensor = NewObject<URammsIMUSensorComponent>(Owner, Mount.Name, RF_Transient);
			break;
		}
		case ERammsSensorKind::Camera:
		{
			UIntrinsicSceneCaptureComponent2D* Camera = NewObject<UIntrinsicSceneCaptureComponent2D>(Owner, Mount.Name, RF_Transient);
			Camera->bUseCustomIntrinsics = true;
			Camera->bUseIntrinsicsAsset = false;
			Camera->InlineIntrinsics = RammsSensorRig::MakeIntrinsics(Mount.Width, Mount.Height, Mount.FovDeg);
			Camera->FOVAngle = FMath::Clamp(Mount.FovDeg, 1.0f, 170.0f);
			Camera->bDrawFrustumInGame = false;
			Sensor = Camera;
			break;
		}
		default:
			break;
	}
	if (!Sensor)
	{
		return false;
	}

	// Attach and place before registering, so the component starts at the mount and anything its BeginPlay reads
	// (the IMU's physics parent) is already there. Registration computes the world transform from these.
	Sensor->SetupAttachment(Parent, Mount.Socket);
	Sensor->SetRelativeLocation_Direct(Mount.Location);
	Sensor->SetRelativeRotation_Direct(Mount.Rotation);
	Owner->AddInstanceComponent(Sensor);
	Sensor->RegisterComponent();

	FRammsBuiltSensor Entry;
	Entry.Mount = Mount;
	Entry.Component = Sensor;

	if (bCamera)
	{
		UIntrinsicSceneCaptureComponent2D* Camera = CastChecked<UIntrinsicSceneCaptureComponent2D>(Sensor);
		if (UCameraCaptureSubsystem* Capture = World->GetSubsystem<UCameraCaptureSubsystem>())
		{
			// Registered cameras are captured from the next kick on, capture running or not.
			Capture->RegisterCamera(Camera);
		}
		else
		{
			UE_LOG(LogRammsSensorRig, Warning, TEXT("%s: mount '%s': no camera capture subsystem in this world; the camera will not be captured"),
				*GetPathName(), *Mount.Name.ToString());
		}

		const int32 MaxBase = MAX_uint16 - FRammsCameraStreams::MaxChannelOffset;
		Entry.Channel = Mount.StreamChannel >= 0 ? FMath::Min(Mount.StreamChannel, MaxBase) : FindFreeStreamChannel(World);

		URammsStreamSourceComponent* Source = NewObject<URammsStreamSourceComponent>(Owner, SourceName, RF_Transient);
		Source->ChannelID = Entry.Channel;
		Source->CameraFilter = Mount.Name.ToString();
		Source->bExactCameraFilter = true;
		Source->StreamName = Mount.Name.ToString();
		Source->bStreamRGB = Mount.bColor;
		Source->bStreamDepth = Mount.bDepth;
		Source->bStreamRGBD = false;
		Source->bStreamMotion = Mount.bMotion;
		Source->bStreamSegmentation = Mount.bSegmentation;
		Owner->AddInstanceComponent(Source);
		Source->RegisterComponent();
		Entry.StreamSource = Source;

		UE_LOG(LogRammsSensorRig, Log, TEXT("%s: camera '%s' %dx%d streams on channel %d (+100 depth, +200 motion, +300 mask)"),
			*Owner->GetName(), *Mount.Name.ToString(), Mount.Width, Mount.Height, Entry.Channel);
	}

	Built.Add(Entry);
	return true;
}

void URammsSensorRigComponent::DestroyBuilt(int32 BuiltIndex)
{
	if (!Built.IsValidIndex(BuiltIndex))
	{
		return;
	}
	const FRammsBuiltSensor Entry = Built[BuiltIndex];
	Built.RemoveAt(BuiltIndex);

	AActor* Owner = GetOwner();

	if (UActorComponent* Source = Entry.StreamSource.Get(); IsValid(Source))
	{
		if (Owner)
		{
			Owner->RemoveInstanceComponent(Source);
		}
		Source->DestroyComponent();
		RammsSensorRig::RetireName(Source);
	}

	if (USceneComponent* Sensor = Entry.Component.Get(); IsValid(Sensor))
	{
		if (UIntrinsicSceneCaptureComponent2D* Camera = Cast<UIntrinsicSceneCaptureComponent2D>(Sensor))
		{
			// Unregistering also removes the depth / motion / segmentation captures the subsystem made for it.
			if (UWorld* World = GetWorld())
			{
				if (UCameraCaptureSubsystem* Capture = World->GetSubsystem<UCameraCaptureSubsystem>())
				{
					UnregisterCaptureCamera(*Capture, Camera);
				}
			}
		}
		if (Owner)
		{
			Owner->RemoveInstanceComponent(Sensor);
		}
		Sensor->DestroyComponent();
		RammsSensorRig::RetireName(Sensor);
	}
}
