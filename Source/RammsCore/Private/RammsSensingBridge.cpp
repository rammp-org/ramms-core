// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsSensingBridge.h"

#include "CameraCaptureSegmentation.h"
#include "CameraCaptureSubsystem.h"
#include "CameraIntrinsics.h"
#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Engine/Engine.h"
#include "Engine/GameInstance.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "IntrinsicSceneCaptureComponent2D.h"
#include "RammsCameraStreamSession.h"
#include "RammsCameraStreams.h"
#include "RammsIMUSensorComponent.h"
#include "RammsPhysicsImuSubsystem.h"
#include "RammsSensingBridgeImu.h"
#include "RammsSensorRigComponent.h"
#include "RammsSonarSensorComponent.h"
#include "RammsStreamSourceComponent.h"
#include "RammsStreamingSubsystem.h"
#include "RammsToFSensorComponent.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsSensingBridge, Log, All);

namespace RammsSensingBridgeImpl
{
	FString ToJsonString(const TSharedRef<FJsonObject>& Obj)
	{
		FString																   Out;
		const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer =
			TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Out);
		FJsonSerializer::Serialize(Obj, Writer);
		return Out;
	}

	TSharedRef<FJsonObject> MakeFailure(const FString& Error)
	{
		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetBoolField(TEXT("ok"), false);
		Obj->SetStringField(TEXT("error"), Error);
		return Obj;
	}

	FString FailJson(const FString& Error)
	{
		return ToJsonString(MakeFailure(Error));
	}

	TSharedRef<FJsonObject> VectorJson(const FVector& V)
	{
		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetNumberField(TEXT("x"), V.X);
		Obj->SetNumberField(TEXT("y"), V.Y);
		Obj->SetNumberField(TEXT("z"), V.Z);
		return Obj;
	}

	TSharedRef<FJsonObject> RotatorJson(const FRotator& R)
	{
		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetNumberField(TEXT("pitch"), R.Pitch);
		Obj->SetNumberField(TEXT("yaw"), R.Yaw);
		Obj->SetNumberField(TEXT("roll"), R.Roll);
		return Obj;
	}

	FString NameOrEmpty(FName Name)
	{
		return Name.IsNone() ? FString() : Name.ToString();
	}

	/** Longest component / socket name a caller may pass. FName itself refuses 1024 characters or more with a
	 *  checkf, so a string from the network is measured before it becomes one. */
	constexpr int32 MaxReferenceNameLength = 256;

	/**
	 * The name of something that must already exist (a component, a socket, a mount), from network text: trimmed,
	 * None for "". Looked up, never added (FNAME_Find), so text from the network neither grows the name table nor
	 * reaches FName's length checkf: a name the table does not have names nothing, and bOutFound says so. False,
	 * leaving OutName alone, when the text is longer than MaxLength.
	 */
	bool ExistingNameFromString(const FString& Text, FName& OutName, bool& bOutFound, int32 MaxLength = MaxReferenceNameLength)
	{
		const FString Trimmed = Text.TrimStartAndEnd();
		if (Trimmed.Len() > MaxLength)
		{
			return false;
		}
		if (Trimmed.IsEmpty())
		{
			OutName = NAME_None;
			bOutFound = true;
			return true;
		}
		OutName = FName(*Trimmed, FNAME_Find);
		bOutFound = !OutName.IsNone();
		return true;
	}

	/**
	 * Whether every part of an object path (split at '.' and ':') is short enough to be an FName and is one the name
	 * table already has. An object can only exist under names that do, so a path failing this names nothing, and
	 * StaticFindObject -- which adds every part of the path to the name table, and checkf-fails on one of 1024
	 * characters or more -- never sees it.
	 */
	bool IsPathOfExistingNames(const FString& Path)
	{
		int32 Start = 0;
		for (int32 i = 0; i <= Path.Len(); ++i)
		{
			if (i < Path.Len() && Path[i] != TEXT('.') && Path[i] != SUBOBJECT_DELIMITER_CHAR)
			{
				continue;
			}
			const int32 Length = i - Start;
			if (Length >= NAME_SIZE)
			{
				return false;
			}
			if (Length > 0 && FName(FStringView(*Path + Start, Length), FNAME_Find).IsNone())
			{
				return false;
			}
			Start = i + 1;
		}
		return true;
	}

	/** One sensor on an actor: built by a rig, or placed by hand. */
	struct FSensorRef
	{
		AActor*							Actor = nullptr;
		FString							Name;
		ERammsSensorKind				Kind = ERammsSensorKind::ToF;
		USceneComponent*				Component = nullptr;
		const URammsSensorRigComponent* Rig = nullptr;
		/** Valid only until the rig changes; used at once. */
		const FRammsBuiltSensor* Built = nullptr;
	};

	bool IsCaptureRegistered(const UWorld* World, const UIntrinsicSceneCaptureComponent2D* Camera)
	{
		const UCameraCaptureSubsystem* Capture = World ? World->GetSubsystem<UCameraCaptureSubsystem>() : nullptr;
		if (!Capture || !Camera)
		{
			return false;
		}
		for (const UIntrinsicSceneCaptureComponent2D* Registered : Capture->GetRegisteredCameras())
		{
			if (Registered == Camera)
			{
				return true;
			}
		}
		return false;
	}

	TArray<FSensorRef> GatherSensors(AActor* Actor)
	{
		TArray<FSensorRef> Sensors;
		if (!IsValid(Actor))
		{
			return Sensors;
		}

		TInlineComponentArray<URammsSensorRigComponent*> Rigs(Actor);
		for (const URammsSensorRigComponent* Rig : Rigs)
		{
			if (!IsValid(Rig))
			{
				continue;
			}
			for (const FRammsBuiltSensor& Entry : Rig->GetBuiltSensors())
			{
				if (!IsValid(Entry.Component))
				{
					continue;
				}
				FSensorRef Ref;
				Ref.Actor = Actor;
				Ref.Name = Entry.Mount.Name.ToString();
				Ref.Kind = Entry.Mount.Kind;
				Ref.Component = Entry.Component.Get();
				Ref.Rig = Rig;
				Ref.Built = &Entry;
				Sensors.Add(Ref);
			}
		}

		const auto BuiltByARig = [&Rigs](const UActorComponent* Component) {
			for (const URammsSensorRigComponent* Rig : Rigs)
			{
				if (IsValid(Rig) && Rig->FindBuiltByComponent(Component))
				{
					return true;
				}
			}
			return false;
		};

		TInlineComponentArray<USceneComponent*> SceneComponents(Actor);
		for (USceneComponent* Component : SceneComponents)
		{
			if (!IsValid(Component) || BuiltByARig(Component))
			{
				continue;
			}
			FSensorRef Ref;
			Ref.Actor = Actor;
			Ref.Name = Component->GetName();
			Ref.Component = Component;
			if (Component->IsA<URammsToFSensorComponent>())
			{
				Ref.Kind = ERammsSensorKind::ToF;
			}
			else if (Component->IsA<URammsSonarSensorComponent>())
			{
				Ref.Kind = ERammsSensorKind::Sonar;
			}
			else if (Component->IsA<URammsIMUSensorComponent>())
			{
				Ref.Kind = ERammsSensorKind::IMU;
			}
			else if (const UIntrinsicSceneCaptureComponent2D* Camera = Cast<UIntrinsicSceneCaptureComponent2D>(Component);
					 Camera && IsCaptureRegistered(Actor->GetWorld(), Camera))
			{
				// Registered cameras only: the capture subsystem's own depth / motion helpers are capture
				// components on the same actor, never registered themselves.
				Ref.Kind = ERammsSensorKind::Camera;
			}
			else
			{
				continue;
			}
			Sensors.Add(Ref);
		}
		return Sensors;
	}

	/** Every stream source in the world. */
	TArray<URammsStreamSourceComponent*> GatherStreamSources(const UWorld* World)
	{
		TArray<URammsStreamSourceComponent*> Sources;
		if (!World)
		{
			return Sources;
		}
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			AActor* Actor = *It;
			if (!IsValid(Actor))
			{
				continue;
			}
			TInlineComponentArray<URammsStreamSourceComponent*> ActorSources(Actor);
			for (URammsStreamSourceComponent* Source : ActorSources)
			{
				if (IsValid(Source))
				{
					Sources.Add(Source);
				}
			}
		}
		return Sources;
	}

	TSharedRef<FJsonObject> StreamJson(const FRammsCameraStreamDesc& Desc)
	{
		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetStringField(TEXT("role"), FRammsCameraStreams::RoleName(Desc.Role));
		Obj->SetStringField(TEXT("stream_id"), Desc.StreamId);
		Obj->SetNumberField(TEXT("channel"), Desc.Channel);
		Obj->SetStringField(TEXT("message"), FRammsCameraStreams::MessageName(Desc.Role));
		Obj->SetStringField(TEXT("fmt"), Desc.Format);
		return Obj;
	}

	/** A source's fused RGBD stream, which has no stream id of its own. */
	TSharedRef<FJsonObject> RgbdStreamJson(const URammsStreamSourceComponent& Source)
	{
		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetStringField(TEXT("role"), TEXT("rgbd"));
		Obj->SetStringField(TEXT("stream_id"), FString());
		Obj->SetNumberField(TEXT("channel"), Source.ChannelID);
		Obj->SetStringField(TEXT("message"), TEXT("FrameRGBD"));
		Obj->SetStringField(TEXT("fmt"), TEXT("bgra8+float32"));
		return Obj;
	}

	/** The streams a source sends for one camera, as JSON. */
	void AppendSourceStreams(const URammsStreamSourceComponent& Source, const FString& CameraStreamName,
		TArray<TSharedPtr<FJsonValue>>& OutStreams)
	{
		if (Source.bStreamRGBD)
		{
			OutStreams.Add(MakeShared<FJsonValueObject>(RgbdStreamJson(Source)));
		}
		for (const FRammsCameraStreamDesc& Desc : Source.DescribeStreams(CameraStreamName))
		{
			OutStreams.Add(MakeShared<FJsonValueObject>(StreamJson(Desc)));
		}
	}

	/** A hand-placed camera's streams: whatever sources in the world forward it. Also reports the first such
	 *  source, for the camera's settings. */
	TArray<TSharedPtr<FJsonValue>> HandPlacedCameraStreams(UIntrinsicSceneCaptureComponent2D* Camera,
		const URammsStreamSourceComponent**													  OutFirstSource)
	{
		TArray<TSharedPtr<FJsonValue>> Streams;
		if (OutFirstSource)
		{
			*OutFirstSource = nullptr;
		}
		if (!Camera || !Camera->GetOwner())
		{
			return Streams;
		}
		const FCameraIdentifier Id = FCameraIdentifier::Generate(Camera);
		const FString			OwnerPath = Camera->GetOwner()->GetPathName();
		for (const URammsStreamSourceComponent* Source : GatherStreamSources(Camera->GetWorld()))
		{
			if (!Source->AcceptsCamera(Id, OwnerPath))
			{
				continue;
			}
			if (OutFirstSource && !*OutFirstSource)
			{
				*OutFirstSource = Source;
			}
			AppendSourceStreams(*Source, Source->GetStreamNameFor(Id), Streams);
		}
		return Streams;
	}

	TSharedRef<FJsonObject> SensorJson(const FSensorRef& Ref)
	{
		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetStringField(TEXT("actor"), Ref.Actor->GetPathName());
		Obj->SetStringField(TEXT("actor_name"), Ref.Actor->GetName());
		Obj->SetStringField(TEXT("name"), Ref.Name);
		Obj->SetStringField(TEXT("kind"), URammsSensorRigComponent::KindToString(Ref.Kind));
		Obj->SetStringField(TEXT("component_path"), Ref.Component->GetPathName());
		if (Ref.Built)
		{
			Obj->SetStringField(TEXT("parent"), NameOrEmpty(Ref.Built->Mount.ParentComponent));
			Obj->SetStringField(TEXT("socket"), NameOrEmpty(Ref.Built->Mount.Socket));
		}
		else
		{
			const USceneComponent* AttachParent = Ref.Component->GetAttachParent();
			Obj->SetStringField(TEXT("parent"), AttachParent ? AttachParent->GetName() : FString());
			Obj->SetStringField(TEXT("socket"), NameOrEmpty(Ref.Component->GetAttachSocketName()));
		}
		Obj->SetBoolField(TEXT("rig"), Ref.Rig != nullptr);
		Obj->SetObjectField(TEXT("location"), VectorJson(Ref.Component->GetComponentLocation()));
		Obj->SetObjectField(TEXT("rotation"), RotatorJson(Ref.Component->GetComponentRotation()));

		TSharedRef<FJsonObject>		   Settings = MakeShared<FJsonObject>();
		TArray<TSharedPtr<FJsonValue>> Streams;
		switch (Ref.Kind)
		{
			case ERammsSensorKind::ToF:
				if (const URammsToFSensorComponent* ToF = Cast<URammsToFSensorComponent>(Ref.Component))
				{
					const bool bGrid = ToF->SensorMode == EToFSensorMode::Grid;
					Settings->SetNumberField(TEXT("rows"), bGrid ? ToF->GridRows : 1);
					Settings->SetNumberField(TEXT("columns"), bGrid ? ToF->GridColumns : 1);
					Settings->SetNumberField(TEXT("h_fov_deg"), ToF->HorizontalFOV);
					Settings->SetNumberField(TEXT("v_fov_deg"), ToF->VerticalFOV);
					Settings->SetNumberField(TEXT("max_range_cm"), ToF->MaxRange);
				}
				break;
			case ERammsSensorKind::Sonar:
				if (const URammsSonarSensorComponent* Sonar = Cast<URammsSonarSensorComponent>(Ref.Component))
				{
					Settings->SetNumberField(TEXT("max_range_cm"), Sonar->MaxRange);
					// The component's BeamHalfAngle: the cone's half-angle.
					Settings->SetNumberField(TEXT("cone_deg"), Sonar->BeamHalfAngle);
				}
				break;
			case ERammsSensorKind::IMU:
				break;
			case ERammsSensorKind::Camera:
			{
				UIntrinsicSceneCaptureComponent2D* Camera = Cast<UIntrinsicSceneCaptureComponent2D>(Ref.Component);
				if (!Camera)
				{
					break;
				}
				const FCameraIntrinsics Intrinsics = Camera->GetActiveIntrinsics();
				Settings->SetNumberField(TEXT("width"), Intrinsics.ImageWidth);
				Settings->SetNumberField(TEXT("height"), Intrinsics.ImageHeight);
				Settings->SetNumberField(TEXT("fov_deg"), Camera->FOVAngle);
				if (Ref.Built)
				{
					const FRammsSensorMount& Mount = Ref.Built->Mount;
					Settings->SetBoolField(TEXT("color"), Mount.bColor);
					Settings->SetBoolField(TEXT("depth"), Mount.bDepth);
					Settings->SetBoolField(TEXT("motion"), Mount.bMotion);
					Settings->SetBoolField(TEXT("segmentation"), Mount.bSegmentation);
					Settings->SetNumberField(TEXT("channel"), Ref.Built->Channel);
					if (const URammsStreamSourceComponent* Source = Cast<URammsStreamSourceComponent>(Ref.Built->StreamSource.Get()))
					{
						AppendSourceStreams(*Source, Ref.Name, Streams);
					}
				}
				else
				{
					const URammsStreamSourceComponent* Source = nullptr;
					Streams = HandPlacedCameraStreams(Camera, &Source);
					Settings->SetBoolField(TEXT("color"), Source && (Source->bStreamRGB || Source->bStreamRGBD));
					Settings->SetBoolField(TEXT("depth"), Source && (Source->bStreamDepth || Source->bStreamRGBD));
					Settings->SetBoolField(TEXT("motion"), Source && Source->bStreamMotion);
					Settings->SetBoolField(TEXT("segmentation"), Source && Source->bStreamSegmentation);
					// No source forwards it: it streams on no channel, so there is none to report.
					if (Source)
					{
						Settings->SetNumberField(TEXT("channel"), Source->ChannelID);
					}
				}
				break;
			}
			default:
				break;
		}
		Obj->SetObjectField(TEXT("settings"), Settings);
		Obj->SetArrayField(TEXT("streams"), Streams);
		return Obj;
	}

	TSharedRef<FJsonObject> ReadingJson(const FSensorRef& Ref, bool bMeasureNow)
	{
		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetBoolField(TEXT("ok"), true);
		Obj->SetStringField(TEXT("name"), Ref.Name);
		Obj->SetStringField(TEXT("kind"), URammsSensorRigComponent::KindToString(Ref.Kind));
		Obj->SetStringField(TEXT("actor"), Ref.Actor ? Ref.Actor->GetPathName() : FString());

		switch (Ref.Kind)
		{
			case ERammsSensorKind::ToF:
			{
				URammsToFSensorComponent* ToF = Cast<URammsToFSensorComponent>(Ref.Component);
				if (!ToF)
				{
					return MakeFailure(TEXT("not a ToF component: ") + Ref.Name);
				}
				const FToFSensorData Sample = bMeasureNow ? ToF->MeasureNow() : ToF->GetToFData();
				Obj->SetNumberField(TEXT("timestamp"), Sample.Timestamp);
				Obj->SetNumberField(TEXT("rows"), Sample.NumRows);
				Obj->SetNumberField(TEXT("columns"), Sample.NumColumns);
				TArray<TSharedPtr<FJsonValue>> Distances;
				Distances.Reserve(Sample.Distances.Num());
				for (const float Distance : Sample.Distances)
				{
					Distances.Add(MakeShared<FJsonValueNumber>(Distance));
				}
				Obj->SetArrayField(TEXT("distances_cm"), Distances);
				Obj->SetNumberField(TEXT("min_distance_cm"), Sample.MinDistance);
				Obj->SetBoolField(TEXT("any_hit"), Sample.bAnyHit);
				// False until the sensor has measured once (it measures at its UpdateRateHz).
				Obj->SetBoolField(TEXT("has_sample"), Sample.Distances.Num() > 0);
				break;
			}
			case ERammsSensorKind::Sonar:
			{
				URammsSonarSensorComponent* Sonar = Cast<URammsSonarSensorComponent>(Ref.Component);
				if (!Sonar)
				{
					return MakeFailure(TEXT("not a Sonar component: ") + Ref.Name);
				}
				const FSonarSensorData Sample = bMeasureNow ? Sonar->MeasureNow() : Sonar->GetSonarData();
				Obj->SetNumberField(TEXT("timestamp"), Sample.Timestamp);
				// False until the sensor has measured once: a measurement stamps the world time.
				Obj->SetBoolField(TEXT("has_sample"), bMeasureNow || Sample.Timestamp > 0.0f);
				Obj->SetNumberField(TEXT("distance_cm"), Sample.Distance);
				Obj->SetBoolField(TEXT("hit"), Sample.bHit);
				Obj->SetObjectField(TEXT("hit_location"), VectorJson(Sample.HitLocation));
				Obj->SetObjectField(TEXT("hit_normal"), VectorJson(Sample.HitNormal));
				// The GPU trace path cannot resolve the hit actor; it is empty then.
				Obj->SetStringField(TEXT("hit_actor"), IsValid(Sample.HitActor) ? Sample.HitActor->GetName() : FString());
				break;
			}
			case ERammsSensorKind::IMU:
			{
				const URammsIMUSensorComponent* IMU = Cast<URammsIMUSensorComponent>(Ref.Component);
				if (!IMU)
				{
					return MakeFailure(TEXT("not an IMU component: ") + Ref.Name);
				}
				// The IMU integrates over ticks; there is no on-demand measurement, so this is always its latest.
				const FIMUSensorData Sample = IMU->GetIMUData();
				Obj->SetNumberField(TEXT("timestamp"), Sample.Timestamp);
				// False until the sensor has produced a reading (each one stamps the game time).
				Obj->SetBoolField(TEXT("has_sample"), Sample.Timestamp > 0.0f);
				// The engine the latest reading came from: "chaos" / "mujoco" (physics rate) or "kinematic".
				Obj->SetStringField(TEXT("source"), URammsPhysicsImuSubsystem::SourceName(Sample.Source));
				Obj->SetObjectField(TEXT("linear_acceleration"), VectorJson(Sample.LinearAcceleration));
				Obj->SetObjectField(TEXT("angular_velocity"), VectorJson(Sample.AngularVelocity));
				TSharedRef<FJsonObject> Orientation = MakeShared<FJsonObject>();
				Orientation->SetNumberField(TEXT("x"), Sample.Orientation.X);
				Orientation->SetNumberField(TEXT("y"), Sample.Orientation.Y);
				Orientation->SetNumberField(TEXT("z"), Sample.Orientation.Z);
				Orientation->SetNumberField(TEXT("w"), Sample.Orientation.W);
				Obj->SetObjectField(TEXT("orientation"), Orientation);
				const FRotator			Euler = Sample.Orientation.Rotator();
				TSharedRef<FJsonObject> EulerJson = MakeShared<FJsonObject>();
				EulerJson->SetNumberField(TEXT("roll"), Euler.Roll);
				EulerJson->SetNumberField(TEXT("pitch"), Euler.Pitch);
				EulerJson->SetNumberField(TEXT("yaw"), Euler.Yaw);
				Obj->SetObjectField(TEXT("euler"), EulerJson);
				// What the component reports, unconverted: UE units, in the sensor's local frame; orientation is
				// world-space.
				TSharedRef<FJsonObject> Units = MakeShared<FJsonObject>();
				Units->SetStringField(TEXT("linear_acceleration"), TEXT("cm/s^2"));
				Units->SetStringField(TEXT("angular_velocity"), TEXT("deg/s"));
				Units->SetStringField(TEXT("euler"), TEXT("deg"));
				Units->SetStringField(TEXT("frame"), TEXT("sensor (acceleration, angular velocity); world (orientation)"));
				Obj->SetObjectField(TEXT("units"), Units);
				Obj->SetBoolField(TEXT("includes_gravity"), IMU->bIncludeGravity);
				break;
			}
			case ERammsSensorKind::Camera:
			default:
			{
				TSharedRef<FJsonObject> Failure = MakeFailure(TEXT("cameras stream over RMSS: see its streams"));
				Failure->SetStringField(TEXT("name"), Ref.Name);
				Failure->SetStringField(TEXT("kind"), URammsSensorRigComponent::KindToString(Ref.Kind));
				return Failure;
			}
		}
		return Obj;
	}

	/** Every actor named by ActorPath ("" = every actor in the world), or none if a non-empty path is not found. */
	TArray<AActor*> ResolveActors(const FString& ActorPath)
	{
		TArray<AActor*> Actors;
		if (!ActorPath.IsEmpty())
		{
			if (AActor* Actor = URammsSensingBridge::FindActor(ActorPath))
			{
				Actors.Add(Actor);
			}
			return Actors;
		}
		if (UWorld* World = URammsSensingBridge::GetSensingWorld())
		{
			for (TActorIterator<AActor> It(World); It; ++It)
			{
				if (IsValid(*It))
				{
					Actors.Add(*It);
				}
			}
		}
		return Actors;
	}

	/** Key's number into InOutValue, if present. False (OutError) when it is not finite: JSON's 1e400 reads as
	 *  infinity, which no sensor setting can use. */
	bool ReadInt(const FJsonObject& Obj, const TCHAR* Key, int32& InOutValue, FString& OutError)
	{
		double Value = 0.0;
		if (Obj.TryGetNumberField(Key, Value))
		{
			if (!FMath::IsFinite(Value))
			{
				OutError = FString::Printf(TEXT("\"%s\" is not a finite number"), Key);
				return false;
			}
			// Clamped first: a huge number from the network must not overflow the conversion.
			InOutValue = FMath::RoundToInt32(FMath::Clamp(Value, static_cast<double>(MIN_int32), static_cast<double>(MAX_int32)));
		}
		return true;
	}

	/** As ReadInt, for a float (a number past a float's range is not finite as one either). */
	bool ReadFloat(const FJsonObject& Obj, const TCHAR* Key, float& InOutValue, FString& OutError)
	{
		double Value = 0.0;
		if (Obj.TryGetNumberField(Key, Value))
		{
			// Checked before the conversion: a double outside a float's range does not convert.
			if (!FMath::IsFinite(Value) || FMath::Abs(Value) > static_cast<double>(TNumericLimits<float>::Max()))
			{
				OutError = FString::Printf(TEXT("\"%s\" is not a finite number"), Key);
				return false;
			}
			InOutValue = static_cast<float>(Value);
		}
		return true;
	}

	void ReadBool(const FJsonObject& Obj, const TCHAR* Key, bool& InOutValue)
	{
		bool Value = false;
		if (Obj.TryGetBoolField(Key, Value))
		{
			InOutValue = Value;
		}
	}

	/** [a, b, c] or {KeyA: a, KeyB: b, KeyC: c}, finite numbers. False if Key is present but neither, or a number is
	 *  not finite. */
	bool ReadTriple(const FJsonObject& Obj, const TCHAR* Key, const TCHAR* KeyA, const TCHAR* KeyB, const TCHAR* KeyC,
		double& A, double& B, double& C, bool& bOutPresent)
	{
		bOutPresent = Obj.HasField(Key);
		if (!bOutPresent)
		{
			return true;
		}
		bool								  bRead = false;
		const TArray<TSharedPtr<FJsonValue>>* Array = nullptr;
		const TSharedPtr<FJsonObject>*		  Nested = nullptr;
		if (Obj.TryGetArrayField(Key, Array) && Array && Array->Num() >= 3)
		{
			bRead = (*Array)[0].IsValid() && (*Array)[0]->TryGetNumber(A)
				&& (*Array)[1].IsValid() && (*Array)[1]->TryGetNumber(B)
				&& (*Array)[2].IsValid() && (*Array)[2]->TryGetNumber(C);
		}
		else if (Obj.TryGetObjectField(Key, Nested) && Nested && Nested->IsValid())
		{
			A = B = C = 0.0;
			(*Nested)->TryGetNumberField(KeyA, A);
			(*Nested)->TryGetNumberField(KeyB, B);
			(*Nested)->TryGetNumberField(KeyC, C);
			bRead = true;
		}
		return bRead && FMath::IsFinite(A) && FMath::IsFinite(B) && FMath::IsFinite(C);
	}

	bool ParseObject(const FString& Json, TSharedPtr<FJsonObject>& OutObject)
	{
		const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
		return FJsonSerializer::Deserialize(Reader, OutObject) && OutObject.IsValid();
	}

	/** Drop the cameras that are gone from a set of them. */
	void PruneCameras(TSet<TWeakObjectPtr<const UIntrinsicSceneCaptureComponent2D>>& Cameras)
	{
		for (auto It = Cameras.CreateIterator(); It; ++It)
		{
			if (!It->IsValid())
			{
				It.RemoveCurrent();
			}
		}
	}

	/** Re-register a capture camera that is missing a pass the channels now ask for: the capture subsystem
	 *  creates a camera's depth / motion passes when it is registered, so a camera registered while a channel
	 *  was off would otherwise never get that pass. A camera that still has none after that (the motion material
	 *  did not load, its depth intrinsics are invalid) is remembered in Session and not tried again, or every
	 *  StartCameraStreams would rebuild all its passes. */
	void EnsureCapturePasses(UCameraCaptureSubsystem& Capture, URammsCameraStreamSession& Session, bool bDepth, bool bMotion)
	{
		const bool bSeparateDepthPass = Capture.GetCaptureMode() != ERammsCaptureMode::SingleCaptureColorDepth;

		TSet<TWeakObjectPtr<const UIntrinsicSceneCaptureComponent2D>>& NoMotionPass = Session.CamerasWithoutMotionPass;
		TSet<TWeakObjectPtr<const UIntrinsicSceneCaptureComponent2D>>& NoDepthPass = Session.CamerasWithoutDepthPass;
		PruneCameras(NoMotionPass);
		PruneCameras(NoDepthPass);
		for (UIntrinsicSceneCaptureComponent2D* Camera : Capture.GetRegisteredCameras())
		{
			if (!IsValid(Camera))
			{
				continue;
			}
			const bool bMissingMotion = bMotion && !Capture.GetMotionRenderTarget(Camera) && !NoMotionPass.Contains(Camera);
			const bool bMissingDepth =
				bDepth && bSeparateDepthPass && !Capture.GetDepthRenderTarget(Camera) && !NoDepthPass.Contains(Camera);
			if (!bMissingMotion && !bMissingDepth)
			{
				continue;
			}
			// Registering makes the passes under the camera's name + _depth / _dmv / _seg; another object of
			// such a name would be replaced (fatally, for another class). Leave such a camera as it is.
			FString Conflict;
			if (!URammsSensorRigComponent::AreCaptureHelperNamesFree(*Camera, &Conflict))
			{
				UE_LOG(LogRammsSensingBridge, Warning, TEXT("Camera %s keeps its capture passes: %s holds one of their names"),
					*Camera->GetPathName(), *Conflict);
				continue;
			}
			URammsSensorRigComponent::UnregisterCaptureCamera(Capture, Camera);
			Capture.RegisterCamera(Camera);
			if (bMissingMotion && !Capture.GetMotionRenderTarget(Camera))
			{
				UE_LOG(LogRammsSensingBridge, Warning, TEXT("Camera %s has no motion pass (is the capture subsystem's motion material loaded?)"),
					*Camera->GetPathName());
				NoMotionPass.Add(Camera);
			}
			if (bMissingDepth && !Capture.GetDepthRenderTarget(Camera))
			{
				UE_LOG(LogRammsSensingBridge, Warning, TEXT("Camera %s has no depth pass (are its depth intrinsics valid?)"),
					*Camera->GetPathName());
				NoDepthPass.Add(Camera);
			}
		}
	}

	URammsStreamingSubsystem* GetStreaming(const UWorld* World)
	{
		UGameInstance* GameInstance = World ? World->GetGameInstance() : nullptr;
		return GameInstance ? GameInstance->GetSubsystem<URammsStreamingSubsystem>() : nullptr;
	}

	TSharedRef<FJsonObject> StreamStatus(UWorld* World)
	{
		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetBoolField(TEXT("ok"), World != nullptr);
		if (!World)
		{
			Obj->SetStringField(TEXT("error"), TEXT("no world"));
		}

		// The server's port while it runs, else the one StartCameraStreams would open.
		const URammsStreamingSubsystem* Streaming = GetStreaming(World);
		const bool						bRunning = Streaming && Streaming->IsServerRunning();
		TSharedRef<FJsonObject>			Server = MakeShared<FJsonObject>();
		Server->SetBoolField(TEXT("running"), bRunning);
		Server->SetNumberField(TEXT("port"), bRunning ? Streaming->GetServerPort() : FRammsCameraStreamOptions().Port);
		Server->SetNumberField(TEXT("connections"), bRunning ? Streaming->GetConnectionCount() : 0);
		Obj->SetObjectField(TEXT("server"), Server);

		const UCameraCaptureSubsystem* Capture = World ? World->GetSubsystem<UCameraCaptureSubsystem>() : nullptr;
		TSharedRef<FJsonObject>		   CaptureJson = MakeShared<FJsonObject>();
		CaptureJson->SetBoolField(TEXT("capturing"), Capture && Capture->IsCapturing());
		CaptureJson->SetNumberField(TEXT("every_n_frames"), Capture ? Capture->GetCaptureRate() : 0);
		CaptureJson->SetBoolField(TEXT("color"), Capture && Capture->IsCapturingRGB());
		CaptureJson->SetBoolField(TEXT("depth"), Capture && Capture->IsCapturingDepth());
		CaptureJson->SetBoolField(TEXT("motion"), Capture && Capture->IsCapturingMotionVectors());
		CaptureJson->SetBoolField(TEXT("segmentation"), Capture && Capture->IsCapturingSegmentation());
		CaptureJson->SetBoolField(TEXT("save_to_disk"), Capture && Capture->IsSerializationEnabled());
		CaptureJson->SetNumberField(TEXT("cameras"), Capture ? Capture->GetRegisteredCameraCount() : 0);
		// Whether camera streams are on, and whether they started the capture session (StopCameraStreams then stops
		// it; otherwise it only takes the streams' cameras and changes back out).
		const URammsCameraStreamSession* Session = URammsCameraStreamSession::Get(World);
		CaptureJson->SetBoolField(TEXT("streams_on"), Session && Session->bActive);
		CaptureJson->SetBoolField(TEXT("started_by_streams"), Session && Session->bActive && Session->bStartedCapture);
		Obj->SetObjectField(TEXT("capture"), CaptureJson);

		// Every registered camera and every rig camera (registered once streams start), through every source that
		// forwards it.
		TArray<UIntrinsicSceneCaptureComponent2D*> Cameras;
		if (Capture)
		{
			Cameras = Capture->GetRegisteredCameras();
		}
		if (World)
		{
			for (TActorIterator<AActor> It(World); It; ++It)
			{
				if (!IsValid(*It))
				{
					continue;
				}
				TInlineComponentArray<URammsSensorRigComponent*> Rigs(*It);
				for (const URammsSensorRigComponent* Rig : Rigs)
				{
					if (!IsValid(Rig))
					{
						continue;
					}
					for (const FRammsBuiltSensor& Entry : Rig->GetBuiltSensors())
					{
						if (UIntrinsicSceneCaptureComponent2D* RigCamera = Cast<UIntrinsicSceneCaptureComponent2D>(Entry.Component.Get()))
						{
							Cameras.AddUnique(RigCamera);
						}
					}
				}
			}
		}
		TArray<TSharedPtr<FJsonValue>> Streams;
		{
			const TArray<URammsStreamSourceComponent*> Sources = GatherStreamSources(World);
			for (UIntrinsicSceneCaptureComponent2D* Camera : Cameras)
			{
				if (!IsValid(Camera) || !Camera->GetOwner())
				{
					continue;
				}
				const FCameraIdentifier Id = FCameraIdentifier::Generate(Camera);
				const FString			OwnerPath = Camera->GetOwner()->GetPathName();
				for (const URammsStreamSourceComponent* Source : Sources)
				{
					if (!Source->AcceptsCamera(Id, OwnerPath))
					{
						continue;
					}
					const FString				   CameraStreamName = Source->GetStreamNameFor(Id);
					TArray<TSharedPtr<FJsonValue>> SourceStreams;
					AppendSourceStreams(*Source, CameraStreamName, SourceStreams);
					for (const TSharedPtr<FJsonValue>& Value : SourceStreams)
					{
						const TSharedPtr<FJsonObject>& Stream = Value->AsObject();
						Stream->SetStringField(TEXT("actor"), OwnerPath);
						Stream->SetStringField(TEXT("camera"), CameraStreamName);
						Streams.Add(Value);
					}
				}
			}
		}
		RammsSensingBridgeImu::AppendImuStreams(World, Streams); // IMU feeds (StartImuStreamJson)
		Obj->SetArrayField(TEXT("streams"), Streams);
		return Obj;
	}
} // namespace RammsSensingBridgeImpl

namespace SensingImpl = RammsSensingBridgeImpl;

// ---------------------------------------------------------------------------
// World and actors
// ---------------------------------------------------------------------------

UWorld* URammsSensingBridge::GetSensingWorld()
{
	if (!GEngine)
	{
		return nullptr;
	}
	for (const FWorldContext& Context : GEngine->GetWorldContexts())
	{
		if ((Context.WorldType == EWorldType::PIE || Context.WorldType == EWorldType::Game) && Context.World())
		{
			return Context.World();
		}
	}
	for (const FWorldContext& Context : GEngine->GetWorldContexts())
	{
		if (Context.WorldType == EWorldType::Editor && Context.World())
		{
			return Context.World();
		}
	}
	return nullptr;
}

static_assert(URammsSensingBridge::MaxActorPathLength < NAME_SIZE, "an actor path the bridge looks up must fit an FName");

AActor* URammsSensingBridge::FindActor(const FString& ActorPath)
{
	// The path comes from the network. Measured before anything else: StaticFindObject turns every part of it into
	// an FName (adding it to the name table), and FName checkf-fails on 1024 characters or more.
	const FString Path = ActorPath.TrimStartAndEnd();
	if (Path.IsEmpty() || Path.Len() > MaxActorPathLength)
	{
		return nullptr;
	}
	// A full object path ("/Game/Maps/Map.Map:PersistentLevel.BP_Robot_C_0"), looked up only when every part of it
	// is a name that already exists (no actor has a path made of names that do not). Never a class default object or
	// an archetype: changing one would change every actor made from it.
	if ((Path.Contains(TEXT(".")) || Path.Contains(TEXT("/"))) && SensingImpl::IsPathOfExistingNames(Path))
	{
		if (AActor* Actor = Cast<AActor>(StaticFindObject(AActor::StaticClass(), nullptr, *Path)); IsValid(Actor) && !Actor->IsTemplate())
		{
			return Actor;
		}
	}
	// Else an actor's name (or, in the editor, its label) in the sensing world.
	if (UWorld* World = GetSensingWorld())
	{
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			AActor* Actor = *It;
			if (!IsValid(Actor))
			{
				continue;
			}
			if (Actor->GetName() == Path || Actor->GetPathName() == Path)
			{
				return Actor;
			}
#if WITH_EDITOR
			if (Actor->GetActorLabel() == Path)
			{
				return Actor;
			}
#endif
		}
	}
	return nullptr;
}

bool URammsSensingBridge::IsPlayWorld(const UWorld* World, FString& OutError)
{
	// PIE or a game; not the editor's other game-like worlds (GamePreview, GameRPC).
	if (World && (World->WorldType == EWorldType::PIE || World->WorldType == EWorldType::Game))
	{
		return true;
	}
	OutError = TEXT("changing sensing needs a play world (PIE or a game)");
	return false;
}

bool URammsSensingBridge::CanChangeSensing(const AActor* Actor, FString& OutError)
{
	if (!Actor || Actor->IsTemplate())
	{
		OutError = TEXT("changing sensing needs an actor in a play world (PIE or a game), not a class default");
		return false;
	}
	if (!IsPlayWorld(Actor->GetWorld(), OutError))
	{
		OutError = FString::Printf(TEXT("%s is not in a play world: changing sensing needs a play world (PIE or a game)"), *Actor->GetName());
		return false;
	}
	return true;
}

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------

bool URammsSensingBridge::ParseMountJson(const FString& MountJson, FRammsSensorMount& OutMount, FString& OutError)
{
	TSharedPtr<FJsonObject> Obj;
	if (!SensingImpl::ParseObject(MountJson, Obj))
	{
		OutError = TEXT("mount is not a JSON object");
		return false;
	}

	// Names are checked as strings before any becomes an FName: FName refuses very long ones with a checkf.
	FString Text;
	if (Obj->TryGetStringField(TEXT("name"), Text))
	{
		const FString Name = Text.TrimStartAndEnd();
		if (!URammsSensorRigComponent::IsValidMountName(Name, OutError))
		{
			return false;
		}
		OutMount.Name = FName(*Name);
	}
	if (OutMount.Name.IsNone())
	{
		OutError = TEXT("mount needs a \"name\"");
		return false;
	}
	if (Obj->TryGetStringField(TEXT("kind"), Text))
	{
		ERammsSensorKind Kind = OutMount.Kind;
		if (Text.Len() > SensingImpl::MaxReferenceNameLength || !URammsSensorRigComponent::KindFromString(Text, Kind))
		{
			OutError = FString::Printf(TEXT("unknown kind \"%s\" (tof, sonar, imu, camera)"), *Text.Left(SensingImpl::MaxReferenceNameLength));
			return false;
		}
		OutMount.Kind = Kind;
	}
	// The parent and socket must already exist, so their names are looked up, never added to the name table: a name
	// the table does not have is no component's or socket's.
	bool bFound = true;
	if (Obj->TryGetStringField(TEXT("parent"), Text))
	{
		if (!SensingImpl::ExistingNameFromString(Text, OutMount.ParentComponent, bFound))
		{
			OutError = FString::Printf(TEXT("\"parent\" is longer than %d characters"), SensingImpl::MaxReferenceNameLength);
			return false;
		}
		if (!bFound)
		{
			OutError = FString::Printf(TEXT("\"parent\" \"%s\" names no component"), *Text.TrimStartAndEnd());
			return false;
		}
	}
	if (Obj->TryGetStringField(TEXT("socket"), Text))
	{
		if (!SensingImpl::ExistingNameFromString(Text, OutMount.Socket, bFound))
		{
			OutError = FString::Printf(TEXT("\"socket\" is longer than %d characters"), SensingImpl::MaxReferenceNameLength);
			return false;
		}
		if (!bFound)
		{
			OutError = FString::Printf(TEXT("\"socket\" \"%s\" names no socket or bone"), *Text.TrimStartAndEnd());
			return false;
		}
	}

	double A = 0.0, B = 0.0, C = 0.0;
	bool   bPresent = false;
	if (!SensingImpl::ReadTriple(*Obj, TEXT("location"), TEXT("x"), TEXT("y"), TEXT("z"), A, B, C, bPresent))
	{
		OutError = TEXT("\"location\" must be three finite numbers, [x, y, z] or {\"x\", \"y\", \"z\"}");
		return false;
	}
	if (bPresent)
	{
		OutMount.Location = FVector(A, B, C);
	}
	if (!SensingImpl::ReadTriple(*Obj, TEXT("rotation"), TEXT("pitch"), TEXT("yaw"), TEXT("roll"), A, B, C, bPresent))
	{
		OutError = TEXT("\"rotation\" must be three finite numbers, [pitch, yaw, roll] or {\"pitch\", \"yaw\", \"roll\"}");
		return false;
	}
	if (bPresent)
	{
		OutMount.Rotation = FRotator(A, B, C);
	}
	if (OutMount.Location.GetAbsMax() > URammsSensorRigComponent::MaxMountOffsetCm)
	{
		OutError = FString::Printf(TEXT("\"location\" is at most %.0f cm from the parent along each axis"),
			URammsSensorRigComponent::MaxMountOffsetCm);
		return false;
	}

	// Every number finite (JSON's 1e400 reads as infinity).
	bool bNumbers = SensingImpl::ReadInt(*Obj, TEXT("rows"), OutMount.Rows, OutError)
		&& SensingImpl::ReadInt(*Obj, TEXT("columns"), OutMount.Columns, OutError)
		&& SensingImpl::ReadFloat(*Obj, TEXT("h_fov_deg"), OutMount.HorizontalFovDeg, OutError)
		&& SensingImpl::ReadFloat(*Obj, TEXT("v_fov_deg"), OutMount.VerticalFovDeg, OutError)
		&& SensingImpl::ReadFloat(*Obj, TEXT("max_range_cm"), OutMount.MaxRangeCm, OutError);
	// A sonar also takes the keys ListSensorsJson reports for it ("max_range_cm", "cone_deg"); the sonar_ keys win.
	if (bNumbers && OutMount.Kind == ERammsSensorKind::Sonar)
	{
		bNumbers = SensingImpl::ReadFloat(*Obj, TEXT("max_range_cm"), OutMount.SonarMaxRangeCm, OutError)
			&& SensingImpl::ReadFloat(*Obj, TEXT("cone_deg"), OutMount.SonarConeDeg, OutError);
	}
	bNumbers = bNumbers && SensingImpl::ReadFloat(*Obj, TEXT("sonar_max_range_cm"), OutMount.SonarMaxRangeCm, OutError)
		&& SensingImpl::ReadFloat(*Obj, TEXT("sonar_cone_deg"), OutMount.SonarConeDeg, OutError)
		&& SensingImpl::ReadInt(*Obj, TEXT("width"), OutMount.Width, OutError)
		&& SensingImpl::ReadInt(*Obj, TEXT("height"), OutMount.Height, OutError)
		&& SensingImpl::ReadFloat(*Obj, TEXT("fov_deg"), OutMount.FovDeg, OutError)
		&& SensingImpl::ReadInt(*Obj, TEXT("channel"), OutMount.StreamChannel, OutError);
	if (!bNumbers)
	{
		return false;
	}
	SensingImpl::ReadBool(*Obj, TEXT("color"), OutMount.bColor);
	SensingImpl::ReadBool(*Obj, TEXT("depth"), OutMount.bDepth);
	SensingImpl::ReadBool(*Obj, TEXT("motion"), OutMount.bMotion);
	SensingImpl::ReadBool(*Obj, TEXT("segmentation"), OutMount.bSegmentation);

	// A camera is 1 to 4096 pixels each way, a range 1 cm to 1 km: larger asks are clamped (the sensor reports what
	// was built).
	OutMount.Width = FMath::Clamp(OutMount.Width, 1, URammsSensorRigComponent::MaxCameraSize);
	OutMount.Height = FMath::Clamp(OutMount.Height, 1, URammsSensorRigComponent::MaxCameraSize);
	OutMount.MaxRangeCm = FMath::Clamp(OutMount.MaxRangeCm, 1.0f, URammsSensorRigComponent::MaxSensorRangeCm);
	OutMount.SonarMaxRangeCm = FMath::Clamp(OutMount.SonarMaxRangeCm, 1.0f, URammsSensorRigComponent::MaxSensorRangeCm);
	OutMount.StreamChannel = FMath::Clamp(OutMount.StreamChannel, -1, FRammsCameraStreams::MaxBaseChannel);
	return true;
}

FString URammsSensingBridge::ListSensorsJson(const FString& ActorPath)
{
	const TArray<AActor*> Actors = SensingImpl::ResolveActors(ActorPath);
	if (!ActorPath.IsEmpty() && Actors.Num() == 0)
	{
		return SensingImpl::FailJson(TEXT("actor not found: ") + ActorPath);
	}

	TArray<TSharedPtr<FJsonValue>> Sensors;
	for (AActor* Actor : Actors)
	{
		for (const SensingImpl::FSensorRef& Ref : SensingImpl::GatherSensors(Actor))
		{
			Sensors.Add(MakeShared<FJsonValueObject>(SensingImpl::SensorJson(Ref)));
		}
	}

	TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
	Obj->SetBoolField(TEXT("ok"), true);
	Obj->SetArrayField(TEXT("sensors"), Sensors);
	return SensingImpl::ToJsonString(Obj);
}

FString URammsSensingBridge::AddSensorJson(const FString& ActorPath, const FString& MountJson)
{
	AActor* Actor = FindActor(ActorPath);
	if (!Actor)
	{
		return SensingImpl::FailJson(TEXT("actor not found: ") + ActorPath);
	}

	FRammsSensorMount Mount;
	FString			  Error;
	if (!CanChangeSensing(Actor, Error) || !ParseMountJson(MountJson, Mount, Error))
	{
		return SensingImpl::FailJson(Error);
	}

	URammsSensorRigComponent* Rig = Actor->FindComponentByClass<URammsSensorRigComponent>();

	// Say why a mount cannot be built rather than leaving it to the log.
	USceneComponent* Parent = nullptr;
	if (!Mount.ParentComponent.IsNone())
	{
		TInlineComponentArray<USceneComponent*> SceneComponents(Actor);
		for (USceneComponent* Candidate : SceneComponents)
		{
			if (IsValid(Candidate) && Candidate->GetFName() == Mount.ParentComponent)
			{
				Parent = Candidate;
				break;
			}
		}
		if (!Parent)
		{
			return SensingImpl::FailJson(FString::Printf(TEXT("%s has no component \"%s\" to mount on"), *Actor->GetName(),
				*Mount.ParentComponent.ToString()));
		}
	}
	else
	{
		Parent = Actor->GetRootComponent();
		if (!Parent)
		{
			return SensingImpl::FailJson(FString::Printf(TEXT("%s has no root component to mount on"), *Actor->GetName()));
		}
	}
	// A socket the parent does not have is an error here (a mount baked into a Blueprint is attached at the parent's
	// origin instead, with a warning).
	if (!Mount.Socket.IsNone() && !Parent->DoesSocketExist(Mount.Socket))
	{
		return SensingImpl::FailJson(FString::Printf(TEXT("%s has no socket or bone \"%s\""), *Parent->GetName(), *Mount.Socket.ToString()));
	}
	// Only the sensor a mount of this name built may be replaced; anything else of the name stays.
	const FRammsBuiltSensor* Replaced = Rig ? Rig->FindBuilt(Mount.Name) : nullptr;
	if (UObject* Existing = FindObjectFast<UObject>(Actor, Mount.Name); IsValid(Existing))
	{
		if (!Replaced || Replaced->Component.Get() != Existing)
		{
			return SensingImpl::FailJson(FString::Printf(TEXT("%s already has an object named \"%s\" that is not a rig sensor"),
				*Actor->GetName(), *Mount.Name.ToString()));
		}
	}

	// Limits: sensors per actor (every rig's mounts), rig cameras per world.
	{
		TSet<FName>										 ActorMounts;
		TInlineComponentArray<URammsSensorRigComponent*> ActorRigs(Actor);
		for (const URammsSensorRigComponent* Each : ActorRigs)
		{
			if (!IsValid(Each))
			{
				continue;
			}
			for (const FRammsSensorMount& EachMount : Each->Mounts)
			{
				ActorMounts.Add(EachMount.Name);
			}
		}
		if (!ActorMounts.Contains(Mount.Name) && ActorMounts.Num() >= MaxRigSensorsPerActor)
		{
			return SensingImpl::FailJson(FString::Printf(TEXT("%s already has %d rig sensors, the most an actor may have"),
				*Actor->GetName(), MaxRigSensorsPerActor));
		}
	}
	if (Mount.Kind == ERammsSensorKind::Camera && !(Replaced && Replaced->Mount.Kind == ERammsSensorKind::Camera))
	{
		int32 Cameras = 0;
		for (TActorIterator<AActor> It(Actor->GetWorld()); It; ++It)
		{
			if (!IsValid(*It))
			{
				continue;
			}
			TInlineComponentArray<URammsSensorRigComponent*> Rigs(*It);
			for (const URammsSensorRigComponent* Each : Rigs)
			{
				if (!IsValid(Each))
				{
					continue;
				}
				for (const FRammsBuiltSensor& Entry : Each->GetBuiltSensors())
				{
					Cameras += Entry.Mount.Kind == ERammsSensorKind::Camera ? 1 : 0;
				}
			}
		}
		if (Cameras >= MaxRigCamerasPerWorld)
		{
			return SensingImpl::FailJson(FString::Printf(TEXT("the world already has %d rig cameras, the most it may have"),
				MaxRigCamerasPerWorld));
		}
	}

	bool bNewRig = false;
	if (!Rig)
	{
		const FName RigName = MakeUniqueObjectName(Actor, URammsSensorRigComponent::StaticClass(), FName(TEXT("SensorRig")));
		Rig = NewObject<URammsSensorRigComponent>(Actor, RigName, RF_Transient);
		Actor->AddInstanceComponent(Rig);
		Rig->RegisterComponent(); // begins play now if the actor has
		bNewRig = true;
	}

	if (!Rig->TryAddMount(Mount, Error))
	{
		// A failure changes nothing: not even a rig left behind.
		if (bNewRig)
		{
			Actor->RemoveInstanceComponent(Rig);
			Rig->DestroyComponent();
		}
		return SensingImpl::FailJson(FString::Printf(TEXT("could not add \"%s\": %s"), *Mount.Name.ToString(), *Error));
	}

	TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
	Obj->SetBoolField(TEXT("ok"), true);
	Obj->SetStringField(TEXT("name"), Mount.Name.ToString());
	Obj->SetStringField(TEXT("kind"), URammsSensorRigComponent::KindToString(Mount.Kind));
	Obj->SetStringField(TEXT("rig_path"), Rig->GetPathName());
	const FRammsBuiltSensor* Built = Rig->FindBuilt(Mount.Name);
	Obj->SetBoolField(TEXT("built"), Built != nullptr);
	Obj->SetStringField(TEXT("component_path"), Built && Built->Component ? Built->Component->GetPathName() : FString());
	if (!Built)
	{
		Obj->SetStringField(TEXT("note"), TEXT("the actor has not begun play; the mount is built at BeginPlay"));
	}
	else if (Mount.Kind == ERammsSensorKind::Camera)
	{
		Obj->SetNumberField(TEXT("channel"), Built->Channel);
		if (Mount.StreamChannel >= 0 && Built->Channel != Mount.StreamChannel)
		{
			Obj->SetStringField(TEXT("warning"), FString::Printf(TEXT("channel %d's streams are used by another stream source; streaming on %d"), Mount.StreamChannel, Built->Channel));
		}
		TArray<TSharedPtr<FJsonValue>> Streams;
		if (const URammsStreamSourceComponent* Source = Cast<URammsStreamSourceComponent>(Built->StreamSource.Get()))
		{
			SensingImpl::AppendSourceStreams(*Source, Mount.Name.ToString(), Streams);
		}
		Obj->SetArrayField(TEXT("streams"), Streams);
	}
	return SensingImpl::ToJsonString(Obj);
}

bool URammsSensingBridge::RemoveSensor(const FString& ActorPath, const FString& Name)
{
	AActor* Actor = FindActor(ActorPath);
	FString Error;
	if (!Actor || !CanChangeSensing(Actor, Error))
	{
		return false;
	}
	// A mount that exists has its name in the name table: looked up, never added.
	FName MountName;
	bool  bFound = false;
	if (!SensingImpl::ExistingNameFromString(Name, MountName, bFound, URammsSensorRigComponent::MaxMountNameLength) || MountName.IsNone())
	{
		return false;
	}
	bool											 bRemoved = false;
	TInlineComponentArray<URammsSensorRigComponent*> Rigs(Actor);
	for (URammsSensorRigComponent* Rig : Rigs)
	{
		if (IsValid(Rig) && Rig->RemoveMount(MountName))
		{
			bRemoved = true;
		}
	}
	return bRemoved;
}

FString URammsSensingBridge::ReadSensorJson(const FString& ActorPath, const FString& Name, bool bMeasureNow)
{
	AActor* Actor = FindActor(ActorPath);
	if (!Actor)
	{
		return SensingImpl::FailJson(TEXT("actor not found: ") + ActorPath);
	}
	for (const SensingImpl::FSensorRef& Ref : SensingImpl::GatherSensors(Actor))
	{
		if (Ref.Name.Equals(Name, ESearchCase::IgnoreCase))
		{
			return SensingImpl::ToJsonString(SensingImpl::ReadingJson(Ref, bMeasureNow));
		}
	}
	return SensingImpl::FailJson(FString::Printf(TEXT("no sensor \"%s\" on %s"), *Name, *Actor->GetName()));
}

FString URammsSensingBridge::ReadSensorsJson(const FString& ActorPath)
{
	const TArray<AActor*> Actors = SensingImpl::ResolveActors(ActorPath);
	if (!ActorPath.IsEmpty() && Actors.Num() == 0)
	{
		return SensingImpl::FailJson(TEXT("actor not found: ") + ActorPath);
	}

	TArray<TSharedPtr<FJsonValue>> Readings;
	for (AActor* Actor : Actors)
	{
		for (const SensingImpl::FSensorRef& Ref : SensingImpl::GatherSensors(Actor))
		{
			if (Ref.Kind != ERammsSensorKind::Camera)
			{
				Readings.Add(MakeShared<FJsonValueObject>(SensingImpl::ReadingJson(Ref, false)));
			}
		}
	}

	TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
	Obj->SetBoolField(TEXT("ok"), true);
	Obj->SetStringField(TEXT("actor"), Actors.Num() == 1 && !ActorPath.IsEmpty() ? Actors[0]->GetPathName() : ActorPath);
	Obj->SetArrayField(TEXT("readings"), Readings);
	return SensingImpl::ToJsonString(Obj);
}

// ---------------------------------------------------------------------------
// Streams
// ---------------------------------------------------------------------------

FString URammsSensingBridge::StartCameraStreamsJson(const FString& OptionsJson)
{
	FRammsCameraStreamOptions Options;
	if (!OptionsJson.TrimStartAndEnd().IsEmpty())
	{
		TSharedPtr<FJsonObject> Obj;
		if (!SensingImpl::ParseObject(OptionsJson, Obj))
		{
			return SensingImpl::FailJson(TEXT("options are not a JSON object"));
		}
		FString Error;
		if (!SensingImpl::ReadInt(*Obj, TEXT("port"), Options.Port, Error))
		{
			return SensingImpl::FailJson(Error);
		}
		SensingImpl::ReadBool(*Obj, TEXT("color"), Options.bColor);
		SensingImpl::ReadBool(*Obj, TEXT("depth"), Options.bDepth);
		SensingImpl::ReadBool(*Obj, TEXT("motion"), Options.bMotion);
		SensingImpl::ReadBool(*Obj, TEXT("segmentation"), Options.bSegmentation);
		// Only what is given changes: the rate is world-wide.
		double EveryNFrames = 0.0;
		if (Obj->TryGetNumberField(TEXT("every_n_frames"), EveryNFrames))
		{
			if (!FMath::IsFinite(EveryNFrames))
			{
				return SensingImpl::FailJson(TEXT("\"every_n_frames\" is not a finite number"));
			}
			Options.EveryNFrames = FMath::RoundToInt32(FMath::Clamp(EveryNFrames, 1.0, 10000.0));
		}
		// Disk writes are not for remote callers: up to eight 4096 x 4096 cameras written every frame would fill a
		// disk. A session the streams start writes nothing; one already running keeps its own setting.
		bool bSaveToDisk = false;
		if (Obj->TryGetBoolField(TEXT("save_to_disk"), bSaveToDisk) && bSaveToDisk)
		{
			return SensingImpl::FailJson(TEXT("\"save_to_disk\" is not available over Remote Control: camera streams never write ")
					TEXT("to disk (record with the capture subsystem's own settings in the editor instead)"));
		}
	}
	return StartCameraStreams(GetSensingWorld(), Options);
}

FString URammsSensingBridge::StartCameraStreams(UWorld* World, const FRammsCameraStreamOptions& Options)
{
	FString PlayWorldError;
	if (!IsPlayWorld(World, PlayWorldError))
	{
		return SensingImpl::FailJson(PlayWorldError);
	}

	TArray<FString> Errors;
	if (Options.bStartServer)
	{
		URammsStreamingSubsystem* Streaming = SensingImpl::GetStreaming(World);
		if (!Streaming)
		{
			Errors.Add(TEXT("no game instance, so no RMSS server"));
		}
		else if (!Streaming->IsServerRunning())
		{
			const int32 Port = FMath::Clamp(Options.Port, 1, 65535);
			if (!Streaming->StartServer(Port))
			{
				// Most likely another program already listens on the port; the server does not share it.
				const FString Why = Streaming->GetLastServerError();
				Errors.Add(FString::Printf(TEXT("could not start the RMSS server on port %d%s%s"), Port, Why.IsEmpty() ? TEXT("") : TEXT(": "), *Why));
			}
		}
	}

	if (Options.bStartCapture)
	{
		UCameraCaptureSubsystem* Capture = World->GetSubsystem<UCameraCaptureSubsystem>();
		if (!Capture)
		{
			Errors.Add(TEXT("no camera capture subsystem in this world"));
		}
		else if (URammsCameraStreamSession* Session = URammsCameraStreamSession::Get(World); !Session)
		{
			Errors.Add(TEXT("no camera stream session in this world"));
		}
		else
		{
			// Capture state is world-wide: channels go on, never off (another camera's stream may need one this
			// call does not), and disk writes and the rate change only when asked. The session remembers what
			// changed, so StopCameraStreams undoes exactly that.
			Session->bActive = true;
			const bool bWasCapturing = Capture->IsCapturing();
			Session->bTurnedOnColor |= Options.bColor && !Capture->IsCapturingRGB();
			Session->bTurnedOnDepth |= Options.bDepth && !Capture->IsCapturingDepth();
			Session->bTurnedOnMotion |= Options.bMotion && !Capture->IsCapturingMotionVectors();
			const bool bColor = Capture->IsCapturingRGB() || Options.bColor;
			const bool bDepth = Capture->IsCapturingDepth() || Options.bDepth;
			const bool bMotion = Capture->IsCapturingMotionVectors() || Options.bMotion;
			Capture->SetCaptureChannels(bColor, bDepth, bMotion);
			if (Options.bSegmentation && !Capture->IsCapturingSegmentation())
			{
				Capture->SetCaptureSegmentation(true);
				Session->bTurnedOnSegmentation |= Capture->IsCapturingSegmentation();
			}
			const auto SetSerialization = [Capture, Session](bool bEnabled) {
				if (!Session->PriorSerialization.IsSet())
				{
					Session->PriorSerialization = Capture->IsSerializationEnabled();
				}
				Capture->SetSerializationEnabled(bEnabled);
			};
			if (Options.SaveToDisk.IsSet())
			{
				SetSerialization(Options.SaveToDisk.GetValue());
			}
			else if (!bWasCapturing && Capture->IsSerializationEnabled())
			{
				// A session this call starts streams, and writes nothing to disk unless asked.
				SetSerialization(false);
			}
			if (Options.EveryNFrames.IsSet())
			{
				if (!Session->PriorRate.IsSet())
				{
					Session->PriorRate = Capture->GetCaptureRate();
				}
				Capture->SetCaptureRate(FMath::Max(1, Options.EveryNFrames.GetValue()));
			}

			// Rig cameras join capture now: until camera streams start they have no render targets or passes.
			URammsSensorRigComponent::RegisterRigCamerasForCapture(World);
			SensingImpl::EnsureCapturePasses(*Capture, *Session, bDepth, bMotion);
			if (!bWasCapturing)
			{
				Capture->StartCapture();
				Session->bStartedCapture |= Capture->IsCapturing();
			}
			if (!Capture->IsCapturing())
			{
				Errors.Add(TEXT("capture did not start: no camera is registered (add a camera sensor first)"));
			}
		}
	}

	TSharedRef<FJsonObject> Status = SensingImpl::StreamStatus(World);
	if (Errors.Num() > 0)
	{
		const FString Error = FString::Join(Errors, TEXT("; "));
		Status->SetBoolField(TEXT("ok"), false);
		Status->SetStringField(TEXT("error"), Error);
		UE_LOG(LogRammsSensingBridge, Warning, TEXT("StartCameraStreams: %s"), *Error);
	}
	return SensingImpl::ToJsonString(Status);
}

bool URammsSensingBridge::StopCameraStreams()
{
	return StopCameraStreamsIn(GetSensingWorld());
}

bool URammsSensingBridge::StopCameraStreamsIn(UWorld* World)
{
	FString Error;
	if (!IsPlayWorld(World, Error))
	{
		return false;
	}
	UCameraCaptureSubsystem*   Capture = World->GetSubsystem<UCameraCaptureSubsystem>();
	URammsCameraStreamSession* Session = URammsCameraStreamSession::Get(World);
	if (!Capture || !Session)
	{
		return false;
	}

	// Only what the streams did is undone: their rig cameras leave capture (their render targets and passes go), a
	// session they started stops, and the channels, rate and disk writes they changed go back. A capture session
	// somebody else runs -- a recording -- keeps running with its own cameras and settings.
	URammsSensorRigComponent::UnregisterRigCamerasFromCapture(World);
	if (Session->bStartedCapture && Capture->IsCapturing())
	{
		Capture->StopCapture();
	}
	if (Session->bTurnedOnColor || Session->bTurnedOnDepth || Session->bTurnedOnMotion)
	{
		Capture->SetCaptureChannels(Capture->IsCapturingRGB() && !Session->bTurnedOnColor,
			Capture->IsCapturingDepth() && !Session->bTurnedOnDepth,
			Capture->IsCapturingMotionVectors() && !Session->bTurnedOnMotion);
	}
	if (Session->bTurnedOnSegmentation)
	{
		Capture->SetCaptureSegmentation(false);
	}
	if (Session->PriorRate.IsSet())
	{
		Capture->SetCaptureRate(Session->PriorRate.GetValue());
	}
	if (Session->PriorSerialization.IsSet())
	{
		Capture->SetSerializationEnabled(Session->PriorSerialization.GetValue());
	}
	Session->Reset();
	return true;
}

FString URammsSensingBridge::GetStreamStatusJson()
{
	return StreamStatusJsonFor(GetSensingWorld());
}

FString URammsSensingBridge::StreamStatusJsonFor(UWorld* World)
{
	return SensingImpl::ToJsonString(SensingImpl::StreamStatus(World));
}

FString URammsSensingBridge::GetSegmentationClassesJson()
{
	UWorld*									   World = GetSensingWorld();
	const UCameraCaptureSegmentationSubsystem* Labels = World ? World->GetSubsystem<UCameraCaptureSegmentationSubsystem>() : nullptr;
	if (!Labels)
	{
		return SensingImpl::FailJson(TEXT("no segmentation subsystem: the class table is resolved in a play world"));
	}

	const TSharedRef<const FCameraCaptureSegTable> Table = Labels->GetTable();
	TArray<TSharedPtr<FJsonValue>>				   Classes;
	for (int32 Value = 0; Value < Table->FineNames.Num(); ++Value)
	{
		if (Table->FineNames[Value].IsNone())
		{
			continue;
		}
		const int32				CoarseId = Table->FineToCoarse.IsValidIndex(Value) ? Table->FineToCoarse[Value] : 0;
		TSharedRef<FJsonObject> Class = MakeShared<FJsonObject>();
		Class->SetNumberField(TEXT("value"), Value);
		Class->SetStringField(TEXT("name"), Table->FineNames[Value].ToString());
		Class->SetStringField(TEXT("coarse"), Table->CoarseNames.IsValidIndex(CoarseId) ? Table->CoarseNames[CoarseId].ToString() : FString());
		Class->SetNumberField(TEXT("coarse_id"), CoarseId);
		Classes.Add(MakeShared<FJsonValueObject>(Class));
	}

	TArray<TSharedPtr<FJsonValue>> Coarse;
	for (int32 Id = 0; Id < Table->CoarseNames.Num(); ++Id)
	{
		TSharedRef<FJsonObject> Entry = MakeShared<FJsonObject>();
		Entry->SetNumberField(TEXT("id"), Id);
		Entry->SetStringField(TEXT("name"), Table->CoarseNames[Id].ToString());
		Coarse.Add(MakeShared<FJsonValueObject>(Entry));
	}

	TArray<int32> ReservedValues = Table->Reserved.Array();
	ReservedValues.Sort();
	TArray<TSharedPtr<FJsonValue>> Reserved;
	for (const int32 Value : ReservedValues)
	{
		Reserved.Add(MakeShared<FJsonValueNumber>(Value));
	}

	TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
	Obj->SetBoolField(TEXT("ok"), true);
	Obj->SetArrayField(TEXT("classes"), Classes);
	Obj->SetArrayField(TEXT("coarse"), Coarse);
	Obj->SetArrayField(TEXT("reserved"), Reserved);
	Obj->SetNumberField(TEXT("num_segment_ids"), FRammsCameraStreams::NumSegmentIds(*Table));
	return SensingImpl::ToJsonString(Obj);
}
