// Copyright Epic Games, Inc. All Rights Reserved.

// The sensing bridge's IMU functions: readings of any body (ReadBodyImuJson, ListImuBodiesJson), every physics step's
// samples since a cursor (ReadImuSamplesJson), and a continuous feed over RMSS (StartImuStreamJson / StopImuStream).

#include "RammsSensingBridgeImu.h"

#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Engine/GameInstance.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "RammsIMUSensorComponent.h"
#include "RammsImuMath.h"
#include "RammsImuStreamSubsystem.h"
#include "RammsPhysicsImuSubsystem.h"
#include "RammsSensingBridge.h"
#include "RammsSensorRigComponent.h"
#include "RammsStreamingSubsystem.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

namespace RammsSensingBridgeImuImpl
{
	/** Seconds a point these functions track is kept without a read. */
	constexpr float ImuReadIdleSeconds = 10.0f;

	/** The RMSS port a stream starts the server on when none is given. */
	constexpr int32 DefaultStreamPort = 30030;

	// --- JSON ------------------------------------------------------------------

	FString ToJson(const TSharedRef<FJsonObject>& Obj)
	{
		FString																   Out;
		const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer =
			TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Out);
		FJsonSerializer::Serialize(Obj, Writer);
		return Out;
	}

	FString Fail(const FString& Error)
	{
		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetBoolField(TEXT("ok"), false);
		Obj->SetStringField(TEXT("error"), Error);
		return ToJson(Obj);
	}

	TSharedRef<FJsonObject> XyzJson(double X, double Y, double Z)
	{
		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetNumberField(TEXT("x"), X);
		Obj->SetNumberField(TEXT("y"), Y);
		Obj->SetNumberField(TEXT("z"), Z);
		return Obj;
	}

	TSharedRef<FJsonObject> XyzJson(const FVector& V)
	{
		return XyzJson(V.X, V.Y, V.Z);
	}

	TSharedRef<FJsonObject> XyzwJson(double X, double Y, double Z, double W)
	{
		TSharedRef<FJsonObject> Obj = XyzJson(X, Y, Z);
		Obj->SetNumberField(TEXT("w"), W);
		return Obj;
	}

	TSharedRef<FJsonObject> RollPitchYawJson(const FQuat& Q)
	{
		const FRotator			Euler = Q.Rotator();
		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetNumberField(TEXT("roll"), Euler.Roll);
		Obj->SetNumberField(TEXT("pitch"), Euler.Pitch);
		Obj->SetNumberField(TEXT("yaw"), Euler.Yaw);
		return Obj;
	}

	FString NameText(FName Name)
	{
		return Name.IsNone() ? FString() : Name.ToString();
	}

	bool ParseJsonObject(const FString& Json, TSharedPtr<FJsonObject>& OutObject)
	{
		const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
		return FJsonSerializer::Deserialize(Reader, OutObject) && OutObject.IsValid();
	}

	/** [a, b, c] or {KeyA: a, KeyB: b, KeyC: c}. False if Key is present but neither. */
	bool ReadTripleField(const FJsonObject& Obj, const TCHAR* Key, const TCHAR* KeyA, const TCHAR* KeyB, const TCHAR* KeyC, double& A, double& B,
		double& C, bool& bOutPresent)
	{
		bOutPresent = Obj.HasField(Key);
		if (!bOutPresent)
		{
			return true;
		}
		const TArray<TSharedPtr<FJsonValue>>* Array = nullptr;
		if (Obj.TryGetArrayField(Key, Array) && Array && Array->Num() >= 3)
		{
			return (*Array)[0].IsValid() && (*Array)[0]->TryGetNumber(A) && (*Array)[1].IsValid() && (*Array)[1]->TryGetNumber(B)
				&& (*Array)[2].IsValid() && (*Array)[2]->TryGetNumber(C);
		}
		const TSharedPtr<FJsonObject>* Nested = nullptr;
		if (Obj.TryGetObjectField(Key, Nested) && Nested && Nested->IsValid())
		{
			A = B = C = 0.0;
			(*Nested)->TryGetNumberField(KeyA, A);
			(*Nested)->TryGetNumberField(KeyB, B);
			(*Nested)->TryGetNumberField(KeyC, C);
			return true;
		}
		return false;
	}

	TSharedRef<FJsonObject> SampleUnitsJson()
	{
		TSharedRef<FJsonObject> Units = MakeShared<FJsonObject>();
		Units->SetStringField(TEXT("linear_acceleration"), TEXT("cm/s^2"));
		Units->SetStringField(TEXT("angular_velocity"), TEXT("deg/s"));
		Units->SetStringField(TEXT("angular_acceleration"), TEXT("deg/s^2"));
		Units->SetStringField(TEXT("linear_velocity"), TEXT("cm/s"));
		Units->SetStringField(TEXT("time"), TEXT("s"));
		Units->SetStringField(TEXT("frame"),
			TEXT("sensor (linear_acceleration, angular_velocity, angular_acceleration); world (linear_velocity, orientation)"));
		return Units;
	}

	bool IsPlayWorld(const UWorld* World)
	{
		return World && (World->WorldType == EWorldType::PIE || World->WorldType == EWorldType::Game);
	}

	// --- Options -------------------------------------------------------------------

	/** The options the IMU functions take (each reads the ones it uses). */
	struct FImuOptions
	{
		FVector			Offset = FVector::ZeroVector;
		FRotator		Rotation = FRotator::ZeroRotator;
		ERammsImuSource Preferred = ERammsImuSource::Auto;
		int64			Since = -1;
		int32			Max = URammsPhysicsImuSubsystem::HistoryCapacity;
		int32			Port = DefaultStreamPort;
		int32			Channel = -1;
	};

	bool ParseImuOptions(const FString& OptionsJson, FImuOptions& Out, FString& OutError)
	{
		if (OptionsJson.TrimStartAndEnd().IsEmpty())
		{
			return true;
		}
		TSharedPtr<FJsonObject> Options;
		if (!ParseJsonObject(OptionsJson, Options))
		{
			OutError = TEXT("options are not a JSON object");
			return false;
		}
		double A = 0.0, B = 0.0, C = 0.0;
		bool   bPresent = false;
		if (!ReadTripleField(*Options, TEXT("offset"), TEXT("x"), TEXT("y"), TEXT("z"), A, B, C, bPresent))
		{
			OutError = TEXT("\"offset\" must be [x, y, z] or {\"x\", \"y\", \"z\"}");
			return false;
		}
		if (bPresent)
		{
			Out.Offset = FVector(A, B, C);
		}
		if (!ReadTripleField(*Options, TEXT("rotation"), TEXT("pitch"), TEXT("yaw"), TEXT("roll"), A, B, C, bPresent))
		{
			OutError = TEXT("\"rotation\" must be [pitch, yaw, roll] or {\"pitch\", \"yaw\", \"roll\"}");
			return false;
		}
		if (bPresent)
		{
			Out.Rotation = FRotator(A, B, C);
		}
		FString SourceText;
		if (Options->TryGetStringField(TEXT("source"), SourceText) && !URammsPhysicsImuSubsystem::ParseSource(SourceText, Out.Preferred))
		{
			OutError = FString::Printf(TEXT("unknown source \"%s\" (auto, chaos, mujoco, kinematic)"), *SourceText);
			return false;
		}
		double Number = 0.0;
		if (Options->TryGetNumberField(TEXT("since"), Number))
		{
			Out.Since = Number < 0.0 ? -1 : static_cast<int64>(Number);
		}
		if (Options->TryGetNumberField(TEXT("max"), Number))
		{
			Out.Max = FMath::Clamp(FMath::RoundToInt32(Number), 0, static_cast<int32>(URammsPhysicsImuSubsystem::HistoryCapacity));
		}
		if (Options->TryGetNumberField(TEXT("port"), Number))
		{
			Out.Port = FMath::RoundToInt32(Number);
		}
		if (Options->TryGetNumberField(TEXT("channel"), Number))
		{
			Out.Channel = FMath::RoundToInt32(Number);
		}
		return true;
	}

	// --- Bodies --------------------------------------------------------------------

	/** Actor and every actor attached to it (a composed robot's articulation). */
	TArray<AActor*> ActorAndAttached(AActor& Actor)
	{
		TArray<AActor*> Actors;
		Actor.GetAttachedActors(Actors, /*bResetArray=*/true, /*bRecursivelyIncludeAttachedActors=*/true);
		Actors.Insert(&Actor, 0);
		return Actors;
	}

	/** The body Body "" reads: the actor's root, unless no engine simulates it while one simulates another of the
	 *  actor's bodies (a URLab robot's root is a plain component above its bodies): then the first of those. */
	bool DefaultImuBody(AActor& Actor, URammsPhysicsImuSubsystem& Imu, const TArray<FRammsImuBodyRef>& Bodies, USceneComponent*& OutComponent,
		FName& OutBone)
	{
		USceneComponent* Root = Actor.GetRootComponent();
		if (Root)
		{
			FRammsImuBodyRef Resolved;
			Imu.ResolveBody(*Root, NAME_None, FTransform::Identity, ERammsImuSource::Auto, Resolved);
			if (Bodies.Num() == 0 || Resolved.Source != ERammsImuSource::Kinematic)
			{
				OutComponent = Root;
				OutBone = NAME_None;
				return true;
			}
		}
		for (const FRammsImuBodyRef& Listed : Bodies)
		{
			if (USceneComponent* Component = Listed.Body.Get())
			{
				OutComponent = Component;
				OutBone = Listed.Bone;
				return true;
			}
		}
		OutComponent = Root;
		OutBone = NAME_None;
		return Root != nullptr;
	}

	/** The body Name names on Actor (or an actor attached to it): a body an engine simulates, by the name it
	 *  reports (a MuJoCo body, a bone, a component); else any scene component by name; else a bone or socket of a
	 *  skeletal mesh. "" is the actor's main body (DefaultImuBody). */
	bool FindImuBody(AActor& Actor, URammsPhysicsImuSubsystem& Imu, const FString& Name, USceneComponent*& OutComponent, FName& OutBone,
		FString& OutError)
	{
		TArray<FRammsImuBodyRef> Bodies;
		Imu.ListBodies(Actor, Bodies);
		const FString Wanted = Name.TrimStartAndEnd();
		if (Wanted.IsEmpty())
		{
			if (DefaultImuBody(Actor, Imu, Bodies, OutComponent, OutBone))
			{
				return true;
			}
			OutError = FString::Printf(TEXT("%s has no root component"), *Actor.GetName());
			return false;
		}
		for (const FRammsImuBodyRef& Listed : Bodies)
		{
			if (USceneComponent* Component = Listed.Body.Get(); Component && Listed.BodyName.Equals(Wanted, ESearchCase::IgnoreCase))
			{
				OutComponent = Component;
				OutBone = Listed.Bone;
				return true;
			}
		}
		const TArray<AActor*> Actors = ActorAndAttached(Actor);
		for (AActor* Each : Actors)
		{
			TInlineComponentArray<USceneComponent*> Components(Each);
			for (USceneComponent* Component : Components)
			{
				if (IsValid(Component) && Component->GetName().Equals(Wanted, ESearchCase::IgnoreCase))
				{
					OutComponent = Component;
					OutBone = NAME_None;
					return true;
				}
			}
		}
		const FName AsName(*Wanted);
		for (AActor* Each : Actors)
		{
			TInlineComponentArray<USkeletalMeshComponent*> Meshes(Each);
			for (USkeletalMeshComponent* Mesh : Meshes)
			{
				if (IsValid(Mesh) && !Mesh->GetSocketBoneName(AsName).IsNone())
				{
					OutComponent = Mesh;
					OutBone = AsName;
					return true;
				}
			}
		}
		OutError = FString::Printf(TEXT("%s has no body, component or bone \"%s\" (see ListImuBodiesJson)"), *Actor.GetName(), *Wanted);
		return false;
	}

	/** The IMU sensor Name on Actor: a rig mount's name, or a hand-placed IMU component's name (as ListSensorsJson
	 *  names them). */
	URammsIMUSensorComponent* FindImuSensor(AActor& Actor, const FString& Name)
	{
		const FString Wanted = Name.TrimStartAndEnd();
		if (Wanted.IsEmpty())
		{
			return nullptr;
		}
		TInlineComponentArray<URammsSensorRigComponent*> Rigs(&Actor);
		for (const URammsSensorRigComponent* Rig : Rigs)
		{
			if (!IsValid(Rig))
			{
				continue;
			}
			for (const FRammsBuiltSensor& Entry : Rig->GetBuiltSensors())
			{
				URammsIMUSensorComponent* Sensor = Cast<URammsIMUSensorComponent>(Entry.Component.Get());
				if (IsValid(Sensor) && Entry.Mount.Name.ToString().Equals(Wanted, ESearchCase::IgnoreCase))
				{
					return Sensor;
				}
			}
		}
		TInlineComponentArray<URammsIMUSensorComponent*> Sensors(&Actor);
		for (URammsIMUSensorComponent* Sensor : Sensors)
		{
			if (IsValid(Sensor) && Sensor->GetName().Equals(Wanted, ESearchCase::IgnoreCase))
			{
				return Sensor;
			}
		}
		return nullptr;
	}

	/** What a Target names: the frame to track, how, and the key the bridge keeps its point under. */
	struct FImuTarget
	{
		USceneComponent* Component = nullptr;
		FName			 Bone;
		FTransform		 LocalPose = FTransform::Identity;
		ERammsImuSource	 Preferred = ERammsImuSource::Auto;
		FString			 Key;
	};

	/** An IMU sensor's frame (its own source, no offset), else a body as ReadBodyImuJson takes it (with the options'
	 *  offset, rotation and source). */
	bool ResolveImuTarget(AActor& Actor, URammsPhysicsImuSubsystem& Imu, const FString& Target, const FImuOptions& Options, FImuTarget& Out,
		FString& OutError)
	{
		if (URammsIMUSensorComponent* Sensor = FindImuSensor(Actor, Target))
		{
			Out.Component = Sensor;
			Out.Bone = NAME_None;
			Out.LocalPose = FTransform::Identity;
			Out.Preferred = Sensor->GetRequestedSource();
			Out.Key = FString::Printf(TEXT("sensor|%s|%s"), *Sensor->GetPathName(), *URammsPhysicsImuSubsystem::SourceName(Out.Preferred));
			return true;
		}
		USceneComponent* Component = nullptr;
		FName			 Bone;
		if (!FindImuBody(Actor, Imu, Target, Component, Bone, OutError))
		{
			return false;
		}
		Out.Component = Component;
		Out.Bone = Bone;
		Out.LocalPose = FTransform(Options.Rotation, Options.Offset);
		Out.Preferred = Options.Preferred;
		// One tracked point per body, pose and source, kept while it is read.
		Out.Key = FString::Printf(TEXT("%s|%s|%s|%s|%s"), *Component->GetPathName(), *Bone.ToString(), *Options.Offset.ToString(),
			*Options.Rotation.ToString(), *URammsPhysicsImuSubsystem::SourceName(Options.Preferred));
		return true;
	}

	/** The points these functions track, by target key (game thread). */
	struct FImuReadTrack
	{
		TWeakObjectPtr<UWorld> World;
		int32				   Handle = 0;
	};

	TMap<FString, FImuReadTrack>& ImuReadTracks()
	{
		static TMap<FString, FImuReadTrack> Tracks;
		return Tracks;
	}

	FString TrackRefusal(const URammsPhysicsImuSubsystem& Imu, const USceneComponent& Component)
	{
		if (Imu.GetTrackedCount() >= URammsPhysicsImuSubsystem::MaxTrackedPoints)
		{
			return FString::Printf(TEXT("the world tracks %d IMU points already, the most it may"),
				static_cast<int32>(URammsPhysicsImuSubsystem::MaxTrackedPoints));
		}
		return TEXT("could not track ") + Component.GetName();
	}

	/** The point a target's key names, tracked (dropped after 10 s without a read) on first use. 0 and OutError when
	 *  it cannot be. */
	int32 AcquireTrack(UWorld& World, URammsPhysicsImuSubsystem& Imu, const FImuTarget& Target, FString& OutError)
	{
		TMap<FString, FImuReadTrack>& Tracks = ImuReadTracks();
		for (auto It = Tracks.CreateIterator(); It; ++It)
		{
			// Points their subsystem dropped (not read for a while), or of a world that is gone.
			UWorld*							 TrackWorld = It.Value().World.Get();
			const URammsPhysicsImuSubsystem* Owner = TrackWorld ? TrackWorld->GetSubsystem<URammsPhysicsImuSubsystem>() : nullptr;
			if (!Owner || !Owner->IsTracked(It.Value().Handle))
			{
				It.RemoveCurrent();
			}
		}
		if (const FImuReadTrack* Existing = Tracks.Find(Target.Key); Existing && Existing->World.Get() == &World)
		{
			return Existing->Handle;
		}
		FImuReadTrack Started;
		Started.World = &World;
		Started.Handle = Imu.TrackPose(Target.Component, Target.Bone, Target.LocalPose, Target.Preferred, ImuReadIdleSeconds);
		if (Started.Handle == 0)
		{
			OutError = TrackRefusal(Imu, *Target.Component);
			return 0;
		}
		Tracks.Add(Target.Key, Started);
		return Started.Handle;
	}

	/** A ReadBodyImuJson answer. Without a sample yet, the pose is the sensor frame's as the game thread sees it, the
	 *  rates are zero and there is no "sim_time". */
	TSharedRef<FJsonObject> BodyImuJson(const AActor& Actor, const USceneComponent& Component, FName Bone, const FString& BodyName,
		ERammsImuSource Source, const FRammsImuSample& Sample, bool bHasSample, double GameTime, const FTransform& SensorNow)
	{
		const bool	  bHasAcceleration = bHasSample && Sample.bHasAcceleration;
		const FQuat	  Orientation = bHasSample ? Sample.Orientation : SensorNow.GetRotation();
		const FVector Acceleration = bHasAcceleration ? RammsImuMath::ToSensorFrame(Orientation, Sample.ProperAcceleration) : FVector::ZeroVector;
		const FVector AngularRate = bHasSample ? RammsImuMath::ToSensorFrame(Orientation, Sample.AngularVelocity) : FVector::ZeroVector;

		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetBoolField(TEXT("ok"), true);
		Obj->SetStringField(TEXT("name"), BodyName);
		Obj->SetStringField(TEXT("kind"), TEXT("IMU"));
		Obj->SetStringField(TEXT("actor"), Actor.GetPathName());
		Obj->SetStringField(TEXT("body"), BodyName);
		Obj->SetStringField(TEXT("component_path"), Component.GetPathName());
		Obj->SetStringField(TEXT("bone"), NameText(Bone));
		Obj->SetStringField(TEXT("source"), URammsPhysicsImuSubsystem::SourceName(bHasSample ? Sample.Source : Source));
		Obj->SetBoolField(TEXT("has_sample"), bHasSample);
		Obj->SetBoolField(TEXT("has_acceleration"), bHasAcceleration);
		Obj->SetNumberField(TEXT("timestamp"), GameTime);
		if (bHasSample)
		{
			Obj->SetNumberField(TEXT("sim_time"), Sample.SimTimeSeconds);
		}
		Obj->SetObjectField(TEXT("linear_acceleration"), XyzJson(Acceleration));
		Obj->SetObjectField(TEXT("angular_velocity"), XyzJson(AngularRate));
		if (bHasAcceleration)
		{
			Obj->SetObjectField(TEXT("angular_acceleration"), XyzJson(RammsImuMath::ToSensorFrame(Orientation, Sample.AngularAcceleration)));
		}
		Obj->SetObjectField(TEXT("linear_velocity"), XyzJson(bHasSample ? Sample.LinearVelocity : FVector::ZeroVector));
		Obj->SetObjectField(TEXT("location"), XyzJson(bHasSample ? Sample.Location : SensorNow.GetLocation()));
		Obj->SetObjectField(TEXT("orientation"), XyzwJson(Orientation.X, Orientation.Y, Orientation.Z, Orientation.W));
		Obj->SetObjectField(TEXT("euler"), RollPitchYawJson(Orientation));
		Obj->SetBoolField(TEXT("includes_gravity"), true);

		TSharedRef<FJsonObject> Units = MakeShared<FJsonObject>();
		Units->SetStringField(TEXT("linear_acceleration"), TEXT("cm/s^2"));
		Units->SetStringField(TEXT("angular_velocity"), TEXT("deg/s"));
		Units->SetStringField(TEXT("angular_acceleration"), TEXT("deg/s^2"));
		Units->SetStringField(TEXT("linear_velocity"), TEXT("cm/s"));
		Units->SetStringField(TEXT("location"), TEXT("cm"));
		Units->SetStringField(TEXT("euler"), TEXT("deg"));
		Units->SetStringField(TEXT("sim_time"), TEXT("s"));
		Units->SetStringField(TEXT("frame"),
			TEXT("sensor (linear_acceleration, angular_velocity, angular_acceleration); world (linear_velocity, location, orientation)"));
		Obj->SetObjectField(TEXT("units"), Units);
		return Obj;
	}

	/** One sample as ReadImuSamplesJson lists it (the stream's record, as JSON). */
	TSharedRef<FJsonObject> SampleJson(int64 Seq, const FRammsImuSample& Sample)
	{
		double Record[RammsImuRecord::NumFields];
		RammsImuRecord::ToRecord(Seq, Sample, Record);
		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetNumberField(TEXT("seq"), Record[0]);
		Obj->SetNumberField(TEXT("sim_time"), Record[1]);
		Obj->SetObjectField(TEXT("linear_acceleration"), XyzJson(Record[2], Record[3], Record[4]));
		Obj->SetObjectField(TEXT("angular_velocity"), XyzJson(Record[5], Record[6], Record[7]));
		Obj->SetObjectField(TEXT("angular_acceleration"), XyzJson(Record[8], Record[9], Record[10]));
		Obj->SetObjectField(TEXT("linear_velocity"), XyzJson(Record[11], Record[12], Record[13]));
		Obj->SetObjectField(TEXT("orientation"), XyzwJson(Record[14], Record[15], Record[16], Record[17]));
		return Obj;
	}

	TSharedRef<FJsonObject> StreamEntryJson(const FRammsImuStream& Stream)
	{
		TSharedRef<FJsonObject> Entry = MakeShared<FJsonObject>();
		Entry->SetStringField(TEXT("actor"), Stream.ActorPath);
		Entry->SetStringField(TEXT("camera"), FString());
		Entry->SetStringField(TEXT("target"), Stream.Target);
		Entry->SetStringField(TEXT("role"), TEXT("imu"));
		Entry->SetStringField(TEXT("stream_id"), Stream.StreamId);
		Entry->SetNumberField(TEXT("channel"), Stream.Channel);
		Entry->SetStringField(TEXT("message"), TEXT("FrameData"));
		Entry->SetStringField(TEXT("fmt"), TEXT("imu_f64x18"));
		return Entry;
	}

	URammsStreamingSubsystem* StreamingFor(const UWorld* World)
	{
		UGameInstance* GameInstance = World ? World->GetGameInstance() : nullptr;
		return GameInstance ? GameInstance->GetSubsystem<URammsStreamingSubsystem>() : nullptr;
	}
} // namespace RammsSensingBridgeImuImpl

namespace ImuBridgeImpl = RammsSensingBridgeImuImpl;

void RammsSensingBridgeImu::AppendImuStreams(const UWorld* World, TArray<TSharedPtr<FJsonValue>>& OutStreams)
{
	const URammsImuStreamSubsystem* Streams = World ? World->GetSubsystem<URammsImuStreamSubsystem>() : nullptr;
	if (!Streams)
	{
		return;
	}
	for (const FRammsImuStream& Stream : Streams->GetStreams())
	{
		OutStreams.Add(MakeShared<FJsonValueObject>(ImuBridgeImpl::StreamEntryJson(Stream)));
	}
}

// ---------------------------------------------------------------------------
// Readings of bodies
// ---------------------------------------------------------------------------

FString URammsSensingBridge::ReadBodyImuJson(const FString& ActorPath, const FString& Body, const FString& OptionsJson)
{
	AActor* Actor = FindActor(ActorPath);
	if (!Actor)
	{
		return ImuBridgeImpl::Fail(TEXT("actor not found: ") + ActorPath);
	}
	UWorld*					   ActorWorld = Actor->GetWorld();
	URammsPhysicsImuSubsystem* Imu = URammsPhysicsImuSubsystem::Get(Actor);
	if (!ActorWorld || !Imu)
	{
		return ImuBridgeImpl::Fail(TEXT("no IMU subsystem in the actor's world"));
	}
	ImuBridgeImpl::FImuOptions Options;
	FString					   Error;
	if (!ImuBridgeImpl::ParseImuOptions(OptionsJson, Options, Error))
	{
		return ImuBridgeImpl::Fail(Error);
	}

	ImuBridgeImpl::FImuTarget Target;
	USceneComponent*		  Component = nullptr;
	FName					  Bone;
	if (!ImuBridgeImpl::FindImuBody(*Actor, *Imu, Body, Component, Bone, Error))
	{
		return ImuBridgeImpl::Fail(Error);
	}
	Target.Component = Component;
	Target.Bone = Bone;
	Target.LocalPose = FTransform(Options.Rotation, Options.Offset);
	Target.Preferred = Options.Preferred;
	Target.Key = FString::Printf(TEXT("%s|%s|%s|%s|%s"), *Component->GetPathName(), *Bone.ToString(), *Options.Offset.ToString(),
		*Options.Rotation.ToString(), *URammsPhysicsImuSubsystem::SourceName(Options.Preferred));
	const int32 Handle = ImuBridgeImpl::AcquireTrack(*ActorWorld, *Imu, Target, Error);
	if (Handle == 0)
	{
		return ImuBridgeImpl::Fail(Error);
	}

	FRammsImuSample Sample;
	const bool		bHasSample = Imu->GetSample(Handle, Sample) && Sample.bValid;
	return ImuBridgeImpl::ToJson(ImuBridgeImpl::BodyImuJson(*Actor, *Component, Bone, Imu->GetTrackedBodyName(Handle), Imu->GetTrackedSource(Handle),
		Sample, bHasSample, ActorWorld->GetTimeSeconds(), Target.LocalPose * RammsImuProviders::ComponentFrame(*Component, Bone)));
}

FString URammsSensingBridge::ListImuBodiesJson(const FString& ActorPath)
{
	AActor* Actor = FindActor(ActorPath);
	if (!Actor)
	{
		return ImuBridgeImpl::Fail(TEXT("actor not found: ") + ActorPath);
	}
	URammsPhysicsImuSubsystem* Imu = URammsPhysicsImuSubsystem::Get(Actor);
	if (!Imu)
	{
		return ImuBridgeImpl::Fail(TEXT("no IMU subsystem in the actor's world"));
	}

	TArray<FRammsImuBodyRef> Bodies;
	Imu->ListBodies(*Actor, Bodies);
	USceneComponent* DefaultComponent = nullptr;
	FName			 DefaultBone;
	ImuBridgeImpl::DefaultImuBody(*Actor, *Imu, Bodies, DefaultComponent, DefaultBone);

	TArray<TSharedPtr<FJsonValue>> Entries;
	const auto					   AddEntry = [&Entries, DefaultComponent, DefaultBone](const FString& EntryName, ERammsImuSource Engine, const USceneComponent& Component,
							  FName Bone) {
		TSharedRef<FJsonObject> Entry = MakeShared<FJsonObject>();
		Entry->SetStringField(TEXT("name"), EntryName);
		Entry->SetStringField(TEXT("engine"), URammsPhysicsImuSubsystem::SourceName(Engine));
		Entry->SetStringField(TEXT("component"), Component.GetName());
		Entry->SetStringField(TEXT("component_path"), Component.GetPathName());
		Entry->SetStringField(TEXT("bone"), ImuBridgeImpl::NameText(Bone));
		Entry->SetBoolField(TEXT("default"), &Component == DefaultComponent && Bone == DefaultBone);
		Entries.Add(MakeShared<FJsonValueObject>(Entry));
	};

	USceneComponent* Root = Actor->GetRootComponent();
	bool			 bRootListed = false;
	for (const FRammsImuBodyRef& Listed : Bodies)
	{
		if (const USceneComponent* Component = Listed.Body.Get())
		{
			AddEntry(Listed.BodyName, Listed.Source, *Component, Listed.Bone);
			bRootListed |= Component == Root && Listed.Bone.IsNone();
		}
	}
	if (Root && !bRootListed)
	{
		FRammsImuBodyRef Resolved;
		Imu->ResolveBody(*Root, NAME_None, FTransform::Identity, ERammsImuSource::Auto, Resolved);
		AddEntry(Root->GetName(), Resolved.Source, *Root, NAME_None);
	}

	TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
	Obj->SetBoolField(TEXT("ok"), true);
	Obj->SetStringField(TEXT("actor"), Actor->GetPathName());
	Obj->SetArrayField(TEXT("bodies"), Entries);
	return ImuBridgeImpl::ToJson(Obj);
}

// ---------------------------------------------------------------------------
// Every physics step's samples
// ---------------------------------------------------------------------------

FString URammsSensingBridge::ReadImuSamplesJson(const FString& ActorPath, const FString& Target, const FString& OptionsJson)
{
	AActor* Actor = FindActor(ActorPath);
	if (!Actor)
	{
		return ImuBridgeImpl::Fail(TEXT("actor not found: ") + ActorPath);
	}
	UWorld*					   ActorWorld = Actor->GetWorld();
	URammsPhysicsImuSubsystem* Imu = URammsPhysicsImuSubsystem::Get(Actor);
	if (!ActorWorld || !Imu)
	{
		return ImuBridgeImpl::Fail(TEXT("no IMU subsystem in the actor's world"));
	}
	ImuBridgeImpl::FImuOptions Options;
	FString					   Error;
	if (!ImuBridgeImpl::ParseImuOptions(OptionsJson, Options, Error))
	{
		return ImuBridgeImpl::Fail(Error);
	}
	ImuBridgeImpl::FImuTarget Resolved;
	if (!ImuBridgeImpl::ResolveImuTarget(*Actor, *Imu, Target, Options, Resolved, Error))
	{
		return ImuBridgeImpl::Fail(Error);
	}
	const int32 Handle = ImuBridgeImpl::AcquireTrack(*ActorWorld, *Imu, Resolved, Error);
	if (Handle == 0)
	{
		return ImuBridgeImpl::Fail(Error);
	}

	FRammsImuHistoryRead Read;
	Imu->ReadHistory(Handle, Options.Since, Options.Max, Read);

	TArray<TSharedPtr<FJsonValue>> Samples;
	Samples.Reserve(Read.Samples.Num());
	for (int32 i = 0; i < Read.Samples.Num(); ++i)
	{
		Samples.Add(MakeShared<FJsonValueObject>(ImuBridgeImpl::SampleJson(Read.FirstSeq + i, Read.Samples[i])));
	}
	double RateHz = 0.0;
	if (Read.Samples.Num() >= 2)
	{
		const double Span = Read.Samples.Last().SimTimeSeconds - Read.Samples[0].SimTimeSeconds;
		RateHz = Span > 0.0 ? (Read.Samples.Num() - 1) / Span : 0.0;
	}

	TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
	Obj->SetBoolField(TEXT("ok"), true);
	Obj->SetStringField(TEXT("actor"), Actor->GetPathName());
	Obj->SetStringField(TEXT("target"), Target.TrimStartAndEnd());
	Obj->SetStringField(TEXT("body"), Imu->GetTrackedBodyName(Handle));
	Obj->SetStringField(TEXT("source"), URammsPhysicsImuSubsystem::SourceName(Imu->GetTrackedSource(Handle)));
	Obj->SetObjectField(TEXT("units"), ImuBridgeImpl::SampleUnitsJson());
	Obj->SetArrayField(TEXT("samples"), Samples);
	Obj->SetNumberField(TEXT("next"), static_cast<double>(Read.Next));
	Obj->SetNumberField(TEXT("dropped"), static_cast<double>(Read.Dropped));
	Obj->SetNumberField(TEXT("rate_hz"), RateHz);
	return ImuBridgeImpl::ToJson(Obj);
}

// ---------------------------------------------------------------------------
// The RMSS feed
// ---------------------------------------------------------------------------

FString URammsSensingBridge::StartImuStreamJson(const FString& ActorPath, const FString& Target, const FString& OptionsJson)
{
	AActor* Actor = FindActor(ActorPath);
	if (!Actor)
	{
		return ImuBridgeImpl::Fail(TEXT("actor not found: ") + ActorPath);
	}
	UWorld* ActorWorld = Actor->GetWorld();
	if (!ImuBridgeImpl::IsPlayWorld(ActorWorld))
	{
		return ImuBridgeImpl::Fail(TEXT("StartImuStreamJson needs a play world (PIE or a game)"));
	}
	URammsPhysicsImuSubsystem* Imu = URammsPhysicsImuSubsystem::Get(Actor);
	URammsImuStreamSubsystem*  Streams = ActorWorld->GetSubsystem<URammsImuStreamSubsystem>();
	URammsStreamingSubsystem*  Streaming = ImuBridgeImpl::StreamingFor(ActorWorld);
	if (!Imu || !Streams || !Streaming)
	{
		return ImuBridgeImpl::Fail(TEXT("no IMU or streaming subsystem in the actor's world"));
	}
	ImuBridgeImpl::FImuOptions Options;
	FString					   Error;
	if (!ImuBridgeImpl::ParseImuOptions(OptionsJson, Options, Error))
	{
		return ImuBridgeImpl::Fail(Error);
	}
	ImuBridgeImpl::FImuTarget Resolved;
	if (!ImuBridgeImpl::ResolveImuTarget(*Actor, *Imu, Target, Options, Resolved, Error))
	{
		return ImuBridgeImpl::Fail(Error);
	}
	if (!Streaming->IsServerRunning() && !Streaming->StartServer(Options.Port))
	{
		return ImuBridgeImpl::Fail(FString::Printf(TEXT("the RMSS server did not start on port %d"), Options.Port));
	}

	// The stream's own point: no idle timeout, every step's sample kept for it.
	const int32 Handle = Imu->TrackPose(Resolved.Component, Resolved.Bone, Resolved.LocalPose, Resolved.Preferred);
	if (Handle == 0)
	{
		return ImuBridgeImpl::Fail(ImuBridgeImpl::TrackRefusal(*Imu, *Resolved.Component));
	}
	Imu->EnableHistory(Handle);

	FRammsImuStream Stream;
	Stream.Actor = Actor;
	Stream.ActorPath = Actor->GetPathName();
	Stream.ActorName = Actor->GetName();
	Stream.Target = Target.TrimStartAndEnd();
	Stream.BodyName = Imu->GetTrackedBodyName(Handle);
	Stream.StreamId = FString::Printf(TEXT("imu/%s/%s"), *Stream.ActorName, Stream.Target.IsEmpty() ? *Stream.BodyName : *Stream.Target);
	Stream.Handle = Handle;
	Stream.Channel = Options.Channel;
	FRammsImuStream Started;
	if (!Streams->StartStream(Stream, Error, Started))
	{
		Imu->Untrack(Handle);
		return ImuBridgeImpl::Fail(Error);
	}

	TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
	Obj->SetBoolField(TEXT("ok"), true);
	Obj->SetStringField(TEXT("actor"), Started.ActorPath);
	Obj->SetStringField(TEXT("target"), Started.Target);
	Obj->SetStringField(TEXT("body"), Started.BodyName);
	Obj->SetStringField(TEXT("source"), URammsPhysicsImuSubsystem::SourceName(Imu->GetTrackedSource(Handle)));
	Obj->SetStringField(TEXT("stream_id"), Started.StreamId);
	Obj->SetNumberField(TEXT("channel"), Started.Channel);
	Obj->SetStringField(TEXT("message"), TEXT("FrameData"));
	Obj->SetStringField(TEXT("fmt"), TEXT("imu_f64x18"));
	TSharedRef<FJsonObject> Server = MakeShared<FJsonObject>();
	Server->SetBoolField(TEXT("running"), Streaming->IsServerRunning());
	Server->SetNumberField(TEXT("port"), Streaming->GetServerPort());
	Server->SetNumberField(TEXT("connections"), Streaming->GetConnectionCount());
	Obj->SetObjectField(TEXT("server"), Server);
	return ImuBridgeImpl::ToJson(Obj);
}

bool URammsSensingBridge::StopImuStream(const FString& ActorPath, const FString& Target)
{
	AActor*					  Actor = FindActor(ActorPath);
	UWorld*					  ActorWorld = Actor ? Actor->GetWorld() : nullptr;
	URammsImuStreamSubsystem* Streams = ActorWorld ? ActorWorld->GetSubsystem<URammsImuStreamSubsystem>() : nullptr;
	return Streams && Streams->StopStream(Actor->GetPathName(), Target.TrimStartAndEnd());
}
