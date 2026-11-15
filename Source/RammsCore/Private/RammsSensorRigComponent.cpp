// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsSensorRigComponent.h"

#include "CameraCaptureSubsystem.h"
#include "CameraIntrinsics.h"
#include "Components/SceneComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "IntrinsicSceneCaptureComponent2D.h"
#include "RammsCameraStreamSession.h"
#include "RammsCameraStreams.h"
#include "RammsIMUSensorComponent.h"
#include "RammsSensingBridge.h"
#include "RammsSonarSensorComponent.h"
#include "RammsStreamChannels.h"
#include "RammsStreamSourceComponent.h"
#include "RammsToFSensorComponent.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsSensorRig, Log, All);

namespace RammsSensorRig
{
	/** A pinhole camera of Width x Height pixels (each clamped to 1..MaxCameraSize) whose horizontal field of view is
	 *  FovDeg: square pixels, the principal point at the centre. */
	FCameraIntrinsics MakeIntrinsics(int32 Width, int32 Height, float FovDeg)
	{
		FCameraIntrinsics Intrinsics;
		Intrinsics.ImageWidth = FMath::Clamp(Width, 1, URammsSensorRigComponent::MaxCameraSize);
		Intrinsics.ImageHeight = FMath::Clamp(Height, 1, URammsSensorRigComponent::MaxCameraSize);
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

	/** No mount may end in these: they are the names of a camera's capture passes and its stream source. */
	const TCHAR* const ReservedNameSuffixes[] = { TEXT("_depth"), TEXT("_dmv"), TEXT("_seg"), TEXT("_stream") };
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

	// Cameras that ask for a channel first, so the free channels the others are given never take theirs.
	const auto IsPinnedCamera = [](const FRammsSensorMount& Mount) {
		return Mount.Kind == ERammsSensorKind::Camera && Mount.StreamChannel >= 0;
	};
	FString Error;
	for (const FRammsSensorMount& Mount : Mounts)
	{
		if (IsPinnedCamera(Mount))
		{
			BuildMount(Mount, Error);
		}
	}
	for (const FRammsSensorMount& Mount : Mounts)
	{
		if (!IsPinnedCamera(Mount))
		{
			BuildMount(Mount, Error);
		}
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
	// The rate and disk writes are this rig's to set only for a session it starts: one already running (somebody's
	// recording, another robot's rig) keeps its own.
	const UWorld*				   World = GetWorld();
	const UCameraCaptureSubsystem* Capture = World ? World->GetSubsystem<UCameraCaptureSubsystem>() : nullptr;
	if (!Capture || !Capture->IsCapturing())
	{
		Options.EveryNFrames = CaptureEveryNFrames;
		if (bSaveCaptureToDisk)
		{
			Options.SaveToDisk = true;
		}
	}
	Options.bStartServer = bStartStreamServerOnBeginPlay;
	Options.bStartCapture = bStartCaptureOnBeginPlay;

	// The channels this rig's cameras ask for, added to whatever is already on.
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
	FString Error;
	if (!TryAddMount(Mount, Error))
	{
		UE_LOG(LogRammsSensorRig, Warning, TEXT("%s: AddMount: %s"), *GetPathName(), *Error);
		return false;
	}
	return true;
}

bool URammsSensorRigComponent::TryAddMount(const FRammsSensorMount& Mount, FString& OutError)
{
	if (Mount.Name.IsNone())
	{
		OutError = TEXT("a mount needs a name");
		return false;
	}
	if (!IsValidMountName(Mount.Name.ToString(), OutError))
	{
		return false;
	}

	// Refuse before anything changes: a replacement that cannot be built must not cost the sensor it replaces.
	const int32 BuiltIndex = Built.IndexOfByPredicate([&Mount](const FRammsBuiltSensor& B) { return B.Mount.Name == Mount.Name; });
	if (HasBegunPlay() && !CheckMount(Mount, BuiltIndex != INDEX_NONE ? &Built[BuiltIndex] : nullptr, OutError))
	{
		return false;
	}

	// The description, so a rig that has not begun play builds it at BeginPlay.
	const int32		  Existing = Mounts.IndexOfByPredicate([&Mount](const FRammsSensorMount& M) { return M.Name == Mount.Name; });
	FRammsSensorMount PreviousMount;
	if (Existing != INDEX_NONE)
	{
		PreviousMount = Mounts[Existing];
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

	// A replaced camera that asks for no particular channel keeps the one it had, so subscribed clients keep it.
	int32			  PreferredChannel = -1;
	bool			  bHadBuilt = false;
	FRammsSensorMount PreviousBuilt;
	if (BuiltIndex != INDEX_NONE)
	{
		bHadBuilt = true;
		PreviousBuilt = Built[BuiltIndex].Mount;
		PreferredChannel = Built[BuiltIndex].Channel;
		DestroyBuilt(BuiltIndex);
	}
	if (BuildMount(Mount, OutError, PreferredChannel))
	{
		return true;
	}

	// CheckMount passed, so this should not happen; put back what was there.
	if (Existing != INDEX_NONE)
	{
		Mounts[Existing] = PreviousMount;
	}
	else
	{
		Mounts.RemoveAll([&Mount](const FRammsSensorMount& M) { return M.Name == Mount.Name; });
	}
	if (bHadBuilt)
	{
		FString Ignored;
		BuildMount(PreviousBuilt, Ignored, PreferredChannel);
	}
	return false;
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

bool URammsSensorRigComponent::IsValidMountName(const FString& Name, FString& OutError)
{
	if (Name.IsEmpty() || Name.Len() > MaxMountNameLength)
	{
		OutError = FString::Printf(TEXT("a sensor name has 1 to %d characters (this one has %d)"), MaxMountNameLength, Name.Len());
		return false;
	}
	for (const TCHAR Char : Name)
	{
		// No '.': it separates the parts of an object path, so the component's path would not name it.
		const bool bAllowed = (Char >= TEXT('A') && Char <= TEXT('Z')) || (Char >= TEXT('a') && Char <= TEXT('z'))
			|| (Char >= TEXT('0') && Char <= TEXT('9')) || Char == TEXT('_') || Char == TEXT('-');
		if (!bAllowed)
		{
			OutError = FString::Printf(TEXT("sensor name \"%s\": only A-Z a-z 0-9 _ - are allowed"), *Name);
			return false;
		}
	}
	for (const TCHAR* Suffix : RammsSensorRig::ReservedNameSuffixes)
	{
		if (Name.EndsWith(Suffix, ESearchCase::IgnoreCase))
		{
			OutError = FString::Printf(
				TEXT("sensor name \"%s\" may not end in %s: a camera's capture passes and stream source are named that way"), *Name,
				Suffix);
			return false;
		}
	}
	return true;
}

int32 URammsSensorRigComponent::FindFreeStreamChannel(const UWorld* World)
{
	return RammsStreamChannels::FindFreeCameraBase(World);
}

bool URammsSensorRigComponent::IsStreamChannelFree(const UWorld* World, int32 BaseChannel, const UActorComponent* Ignore)
{
	return RammsStreamChannels::IsCameraBaseFree(World, BaseChannel, Ignore);
}

bool URammsSensorRigComponent::AreCaptureHelperNamesFree(const UIntrinsicSceneCaptureComponent2D& Camera, FString* OutConflict)
{
	AActor* Owner = Camera.GetOwner();
	if (!Owner)
	{
		return true;
	}
	for (const TCHAR* Suffix : RammsSensorRig::CaptureHelperSuffixes)
	{
		UObject* Existing = FindObjectFast<UObject>(Owner, FName(*(Camera.GetName() + Suffix)));
		if (!Existing)
		{
			continue;
		}
		if (!IsValid(Existing))
		{
			// Destroyed, not yet collected: out of the way.
			RammsSensorRig::RetireName(Existing);
			continue;
		}
		// The camera's own pass (the subsystem attaches each to its camera) is fine.
		const UIntrinsicSceneCaptureComponent2D* Pass = Cast<UIntrinsicSceneCaptureComponent2D>(Existing);
		if (Pass && Pass->GetAttachParent() == &Camera)
		{
			continue;
		}
		if (OutConflict)
		{
			*OutConflict = Existing->GetPathName();
		}
		return false;
	}
	return true;
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

int32 URammsSensorRigComponent::RegisterRigCamerasForCapture(UWorld* World)
{
	int32 Registered = 0;
	if (!World)
	{
		return Registered;
	}
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (!IsValid(*It))
		{
			continue;
		}
		TInlineComponentArray<URammsSensorRigComponent*> Rigs(*It);
		for (URammsSensorRigComponent* Rig : Rigs)
		{
			if (IsValid(Rig))
			{
				Registered += Rig->RegisterCamerasForCapture();
			}
		}
	}
	return Registered;
}

int32 URammsSensorRigComponent::UnregisterRigCamerasFromCapture(UWorld* World)
{
	int32 Unregistered = 0;
	if (!World)
	{
		return Unregistered;
	}
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (!IsValid(*It))
		{
			continue;
		}
		TInlineComponentArray<URammsSensorRigComponent*> Rigs(*It);
		for (URammsSensorRigComponent* Rig : Rigs)
		{
			if (IsValid(Rig))
			{
				Unregistered += Rig->UnregisterCamerasFromCapture();
			}
		}
	}
	return Unregistered;
}

int32 URammsSensorRigComponent::UnregisterCamerasFromCapture()
{
	UWorld*					 World = GetWorld();
	UCameraCaptureSubsystem* Capture = World ? World->GetSubsystem<UCameraCaptureSubsystem>() : nullptr;
	int32					 Unregistered = 0;
	for (FRammsBuiltSensor& Entry : Built)
	{
		if (!Entry.bCaptureRegistered)
		{
			continue;
		}
		Entry.bCaptureRegistered = false;
		UIntrinsicSceneCaptureComponent2D* Camera = Cast<UIntrinsicSceneCaptureComponent2D>(Entry.Component.Get());
		if (Capture && IsValid(Camera))
		{
			UnregisterCaptureCamera(*Capture, Camera);
			++Unregistered;
		}
	}
	return Unregistered;
}

int32 URammsSensorRigComponent::RegisterCamerasForCapture()
{
	UWorld*					 World = GetWorld();
	UCameraCaptureSubsystem* Capture = World ? World->GetSubsystem<UCameraCaptureSubsystem>() : nullptr;
	if (!Capture)
	{
		return 0;
	}
	int32 Registered = 0;
	for (int32 i = 0; i < Built.Num(); ++i)
	{
		if (Built[i].Mount.Kind == ERammsSensorKind::Camera && !Built[i].bCaptureRegistered && RegisterBuiltCamera(i, *Capture))
		{
			++Registered;
		}
	}
	return Registered;
}

bool URammsSensorRigComponent::RegisterBuiltCamera(int32 BuiltIndex, UCameraCaptureSubsystem& Capture)
{
	if (!Built.IsValidIndex(BuiltIndex))
	{
		return false;
	}
	FRammsBuiltSensor&				   Entry = Built[BuiltIndex];
	UIntrinsicSceneCaptureComponent2D* Camera = Cast<UIntrinsicSceneCaptureComponent2D>(Entry.Component.Get());
	if (!IsValid(Camera))
	{
		return false;
	}
	if (Capture.GetRegisteredCameras().Contains(Camera))
	{
		// Registered by something else already: it is theirs to unregister, not the streams'.
		return false;
	}
	FString Conflict;
	if (!AreCaptureHelperNamesFree(*Camera, &Conflict))
	{
		UE_LOG(LogRammsSensorRig, Warning, TEXT("%s: camera '%s' not captured: %s holds the name of one of its capture passes"),
			*GetPathName(), *Entry.Mount.Name.ToString(), *Conflict);
		return false;
	}
	Capture.RegisterCamera(Camera);
	Entry.bCaptureRegistered = true;
	return true;
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

bool URammsSensorRigComponent::CheckMount(const FRammsSensorMount& Mount, const FRammsBuiltSensor* Replacing, FString& OutError) const
{
	AActor* Owner = GetOwner();
	if (!Owner || !GetWorld())
	{
		OutError = TEXT("the rig has no actor or world");
		return false;
	}
	if (Mount.Name.IsNone())
	{
		OutError = TEXT("a mount needs a name");
		return false;
	}
	if (!IsValidMountName(Mount.Name.ToString(), OutError))
	{
		return false;
	}
	const FRammsBuiltSensor* Same = FindBuilt(Mount.Name);
	if (Same && Same != Replacing)
	{
		OutError = FString::Printf(TEXT("mount '%s' is listed twice"), *Mount.Name.ToString());
		return false;
	}

	// Numbers a transform or a trace can use: a NaN or infinite one would put the component (or its traces, or a
	// camera's view) nowhere.
	if (Mount.Location.ContainsNaN() || Mount.Rotation.ContainsNaN())
	{
		OutError = FString::Printf(TEXT("mount '%s' has a location or rotation that is not finite"), *Mount.Name.ToString());
		return false;
	}
	if (Mount.Location.GetAbsMax() > MaxMountOffsetCm)
	{
		OutError = FString::Printf(TEXT("mount '%s' is %.0f cm from its parent; at most %.0f cm along each axis"),
			*Mount.Name.ToString(), Mount.Location.GetAbsMax(), MaxMountOffsetCm);
		return false;
	}
	for (const float Number : { Mount.HorizontalFovDeg, Mount.VerticalFovDeg, Mount.MaxRangeCm, Mount.SonarMaxRangeCm,
			 Mount.SonarConeDeg, Mount.FovDeg })
	{
		if (!FMath::IsFinite(Number))
		{
			OutError = FString::Printf(TEXT("mount '%s' has a field of view, range or cone that is not finite"), *Mount.Name.ToString());
			return false;
		}
	}

	if (Mount.ParentComponent == Mount.Name)
	{
		OutError = FString::Printf(TEXT("mount '%s' cannot be mounted on itself"), *Mount.Name.ToString());
		return false;
	}
	if (!ResolveParent(Mount.ParentComponent))
	{
		OutError = FString::Printf(TEXT("%s has no component '%s' to mount '%s' on"), *Owner->GetName(),
			Mount.ParentComponent.IsNone() ? TEXT("(root)") : *Mount.ParentComponent.ToString(), *Mount.Name.ToString());
		return false;
	}

	// The names this mount creates (and, for a camera, its capture passes will): taken by anything but what a
	// replacement destroys? A capture pass created over another object of its name is a fatal error.
	const auto IsTakenByAnother = [this, Owner, Replacing](FName Name) {
		if (!IsNameTaken(Name))
		{
			return false;
		}
		const UObject* Existing = FindObjectFast<UObject>(Owner, Name);
		if (Replacing && Existing)
		{
			if (Existing == Replacing->Component.Get() || Existing == Replacing->StreamSource.Get())
			{
				return false;
			}
			// The replaced camera's own capture passes go with it.
			const UIntrinsicSceneCaptureComponent2D* Pass = Cast<UIntrinsicSceneCaptureComponent2D>(Existing);
			if (Pass && Replacing->Component && Pass->GetAttachParent() == Replacing->Component.Get())
			{
				return false;
			}
		}
		return true;
	};

	TArray<FName, TInlineAllocator<5>> Names;
	Names.Add(Mount.Name);
	if (Mount.Kind == ERammsSensorKind::Camera)
	{
		Names.Add(RammsSensorRig::StreamSourceNameFor(Mount.Name));
		for (const TCHAR* Suffix : RammsSensorRig::CaptureHelperSuffixes)
		{
			Names.Add(FName(*(Mount.Name.ToString() + Suffix)));
		}
	}
	for (const FName& Name : Names)
	{
		if (IsTakenByAnother(Name))
		{
			OutError = FString::Printf(TEXT("%s already has an object named '%s' (mount '%s')"), *Owner->GetName(), *Name.ToString(),
				*Mount.Name.ToString());
			return false;
		}
	}
	return true;
}

bool URammsSensorRigComponent::BuildMount(const FRammsSensorMount& InMount, FString& OutError, int32 PreferredChannel)
{
	AActor* Owner = GetOwner();
	UWorld* World = GetWorld();
	if (!CheckMount(InMount, nullptr, OutError) || !Owner || !World)
	{
		UE_LOG(LogRammsSensorRig, Warning, TEXT("%s: mount '%s' skipped: %s"), *GetPathName(), *InMount.Name.ToString(), *OutError);
		return false;
	}

	FRammsSensorMount Mount = InMount;
	Mount.Width = FMath::Clamp(Mount.Width, 1, MaxCameraSize);
	Mount.Height = FMath::Clamp(Mount.Height, 1, MaxCameraSize);
	Mount.MaxRangeCm = FMath::Clamp(Mount.MaxRangeCm, 1.0f, MaxSensorRangeCm);
	Mount.SonarMaxRangeCm = FMath::Clamp(Mount.SonarMaxRangeCm, 1.0f, MaxSensorRangeCm);

	USceneComponent* Parent = ResolveParent(Mount.ParentComponent);
	if (!Mount.Socket.IsNone() && !Parent->DoesSocketExist(Mount.Socket))
	{
		UE_LOG(LogRammsSensorRig, Warning, TEXT("%s: mount '%s': %s has no socket '%s'; attached at its origin"),
			*GetPathName(), *Mount.Name.ToString(), *Parent->GetName(), *Mount.Socket.ToString());
	}

	const bool bCamera = Mount.Kind == ERammsSensorKind::Camera;

	// A camera's channel first, so a camera that cannot have one builds nothing. The one it asks for, or (a replaced
	// camera) the one it had, if no other stream source and no IMU stream uses any of its four channels; else a free
	// one.
	int32 CameraChannel = INDEX_NONE;
	if (bCamera)
	{
		if (Mount.StreamChannel >= 0)
		{
			const int32 Pinned = FMath::Min(Mount.StreamChannel, FRammsCameraStreams::MaxBaseChannel);
			if (IsStreamChannelFree(World, Pinned))
			{
				CameraChannel = Pinned;
			}
			else
			{
				UE_LOG(LogRammsSensorRig, Warning,
					TEXT("%s: camera '%s' asked for channel %d, but another stream source or an IMU stream already uses %d, %d, %d or %d; it streams on a free channel"),
					*GetPathName(), *Mount.Name.ToString(), Pinned, Pinned, Pinned + FRammsCameraStreams::DepthChannelOffset,
					Pinned + FRammsCameraStreams::MotionChannelOffset, Pinned + FRammsCameraStreams::MaskChannelOffset);
			}
		}
		else if (PreferredChannel >= 0 && IsStreamChannelFree(World, PreferredChannel))
		{
			CameraChannel = PreferredChannel;
		}
		if (CameraChannel == INDEX_NONE)
		{
			CameraChannel = FindFreeStreamChannel(World);
		}
		if (CameraChannel == INDEX_NONE)
		{
			OutError = FString::Printf(TEXT("camera '%s': no free stream channel left in the world"), *Mount.Name.ToString());
			UE_LOG(LogRammsSensorRig, Warning, TEXT("%s: mount '%s' skipped: %s"), *GetPathName(), *Mount.Name.ToString(), *OutError);
			return false;
		}
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
			ToF->MaxRange = Mount.MaxRangeCm;
			Sensor = ToF;
			break;
		}
		case ERammsSensorKind::Sonar:
		{
			URammsSonarSensorComponent* Sonar = NewObject<URammsSonarSensorComponent>(Owner, Mount.Name, RF_Transient);
			Sonar->MaxRange = Mount.SonarMaxRangeCm;
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
		OutError = TEXT("unknown sensor kind");
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

	UCameraCaptureSubsystem* Capture = bCamera ? World->GetSubsystem<UCameraCaptureSubsystem>() : nullptr;
	if (bCamera)
	{
		if (!Capture)
		{
			UE_LOG(LogRammsSensorRig, Warning, TEXT("%s: mount '%s': no camera capture subsystem in this world; the camera will not be captured"),
				*GetPathName(), *Mount.Name.ToString());
		}

		Entry.Channel = CameraChannel;

		URammsStreamSourceComponent* Source =
			NewObject<URammsStreamSourceComponent>(Owner, RammsSensorRig::StreamSourceNameFor(Mount.Name), RF_Transient);
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

	// A camera joins capture (render targets, capture passes) only while camera streams are on, so a rig baked into a
	// pawn costs nothing until then; StartCameraStreams registers it when they start, StopCameraStreams unregisters
	// it. A capture session somebody else runs does not take rig cameras in.
	const URammsCameraStreamSession* Session = URammsCameraStreamSession::Get(World);
	if (bCamera && Capture && Session && Session->bActive)
	{
		RegisterBuiltCamera(Built.Num() - 1, *Capture);
	}
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
