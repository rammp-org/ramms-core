// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsImuStreamSubsystem.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Engine/GameInstance.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "RammsImuMath.h"
#include "RammsPhysicsImuSubsystem.h"
#include "RammsStreamingSubsystem.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

static_assert(PLATFORM_LITTLE_ENDIAN, "IMU stream records are little-endian float64, written as the platform stores them");

const TCHAR* const RammsImuRecord::FieldNames[RammsImuRecord::NumFields] = {
	TEXT("seq"), TEXT("sim_time"),
	TEXT("ax"), TEXT("ay"), TEXT("az"),
	TEXT("gx"), TEXT("gy"), TEXT("gz"),
	TEXT("alpha_x"), TEXT("alpha_y"), TEXT("alpha_z"),
	TEXT("vx"), TEXT("vy"), TEXT("vz"),
	TEXT("qx"), TEXT("qy"), TEXT("qz"), TEXT("qw")
};

void RammsImuRecord::ToRecord(int64 Seq, const FRammsImuSample& Sample, double (&OutRecord)[NumFields])
{
	const FQuat&  Frame = Sample.Orientation;
	const FVector Acceleration = Sample.bHasAcceleration ? RammsImuMath::ToSensorFrame(Frame, Sample.ProperAcceleration) : FVector::ZeroVector;
	const FVector Rate = RammsImuMath::ToSensorFrame(Frame, Sample.AngularVelocity);
	const FVector AngularAcceleration = Sample.bHasAcceleration ? RammsImuMath::ToSensorFrame(Frame, Sample.AngularAcceleration) : FVector::ZeroVector;
	OutRecord[0] = static_cast<double>(Seq);
	OutRecord[1] = Sample.SimTimeSeconds;
	OutRecord[2] = Acceleration.X;
	OutRecord[3] = Acceleration.Y;
	OutRecord[4] = Acceleration.Z;
	OutRecord[5] = Rate.X;
	OutRecord[6] = Rate.Y;
	OutRecord[7] = Rate.Z;
	OutRecord[8] = AngularAcceleration.X;
	OutRecord[9] = AngularAcceleration.Y;
	OutRecord[10] = AngularAcceleration.Z;
	OutRecord[11] = Sample.LinearVelocity.X;
	OutRecord[12] = Sample.LinearVelocity.Y;
	OutRecord[13] = Sample.LinearVelocity.Z;
	OutRecord[14] = Frame.X;
	OutRecord[15] = Frame.Y;
	OutRecord[16] = Frame.Z;
	OutRecord[17] = Frame.W;
}

namespace RammsImuStreamImpl
{
	URammsStreamingSubsystem* StreamingOf(const UWorld* World)
	{
		UGameInstance* GameInstance = World ? World->GetGameInstance() : nullptr;
		return GameInstance ? GameInstance->GetSubsystem<URammsStreamingSubsystem>() : nullptr;
	}

	FString CondensedJson(const TSharedRef<FJsonObject>& Obj)
	{
		FString																   Out;
		const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer =
			TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Out);
		FJsonSerializer::Serialize(Obj, Writer);
		return Out;
	}
} // namespace RammsImuStreamImpl

bool URammsImuStreamSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

bool URammsImuStreamSubsystem::StartStream(const FRammsImuStream& Stream, FString& OutError, FRammsImuStream& OutStarted)
{
	int32 Replacing = INDEX_NONE;
	for (int32 i = 0; i < Streams.Num(); ++i)
	{
		if (Streams[i].ActorPath == Stream.ActorPath && Streams[i].Target == Stream.Target)
		{
			Replacing = i;
			break;
		}
	}
	const auto ChannelTaken = [this, Replacing](int32 Channel) {
		for (int32 i = 0; i < Streams.Num(); ++i)
		{
			if (i != Replacing && Streams[i].Channel == Channel)
			{
				return true;
			}
		}
		return false;
	};

	int32 Channel = Stream.Channel;
	if (Channel < 0)
	{
		// A restarted stream keeps its channel; a new one takes the lowest free one from FirstChannel.
		Channel = Replacing != INDEX_NONE ? Streams[Replacing].Channel : FirstChannel;
		while (ChannelTaken(Channel))
		{
			++Channel;
		}
	}
	if (Channel > MAX_uint16)
	{
		OutError = FString::Printf(TEXT("channel %d is out of range (0..65535)"), Channel);
		return false;
	}
	if (ChannelTaken(Channel))
	{
		OutError = FString::Printf(TEXT("channel %d is taken by another IMU stream"), Channel);
		return false;
	}

	if (Replacing != INDEX_NONE)
	{
		Release(Streams[Replacing]);
		Streams.RemoveAt(Replacing);
	}
	OutStarted = Stream;
	OutStarted.Channel = Channel;
	OutStarted.Cursor = -1; // from the next sample on
	Streams.Add(OutStarted);
	return true;
}

bool URammsImuStreamSubsystem::StopStream(const FString& ActorPath, const FString& Target)
{
	for (int32 i = 0; i < Streams.Num(); ++i)
	{
		if (Streams[i].ActorPath == ActorPath && Streams[i].Target == Target)
		{
			Release(Streams[i]);
			Streams.RemoveAt(i);
			return true;
		}
	}
	return false;
}

void URammsImuStreamSubsystem::Release(FRammsImuStream& Stream)
{
	if (URammsPhysicsImuSubsystem* Imu = URammsPhysicsImuSubsystem::Get(GetWorld()))
	{
		Imu->Untrack(Stream.Handle);
	}
	Stream.Handle = 0;
}

bool URammsImuStreamSubsystem::Pump(FRammsImuStream& Stream)
{
	UWorld*					   World = GetWorld();
	URammsPhysicsImuSubsystem* Imu = URammsPhysicsImuSubsystem::Get(World);
	if (!Imu || !Imu->IsTracked(Stream.Handle) || !Stream.Actor.IsValid())
	{
		return false;
	}
	FRammsImuHistoryRead Read;
	Imu->ReadHistory(Stream.Handle, Stream.Cursor, URammsPhysicsImuSubsystem::HistoryCapacity, Read);
	Stream.Cursor = Read.Next;
	const int32				  Count = Read.Samples.Num();
	URammsStreamingSubsystem* Streaming = RammsImuStreamImpl::StreamingOf(World);
	if (Count == 0 || !Streaming || !Streaming->IsServerRunning())
	{
		return true; // nothing new, or nobody to send it to
	}

	constexpr int32 RecordBytes = static_cast<int32>(RammsImuRecord::NumFields * sizeof(double));
	TArray<uint8>	Payload;
	Payload.SetNumUninitialized(Count * RecordBytes);
	double Record[RammsImuRecord::NumFields];
	for (int32 i = 0; i < Count; ++i)
	{
		RammsImuRecord::ToRecord(Read.FirstSeq + i, Read.Samples[i], Record);
		FMemory::Memcpy(Payload.GetData() + static_cast<SIZE_T>(i) * RecordBytes, Record, RecordBytes);
	}

	const FString			Target = Stream.Target.IsEmpty() ? Stream.BodyName : Stream.Target;
	TSharedRef<FJsonObject> Meta = MakeShared<FJsonObject>();
	Meta->SetStringField(TEXT("stream_id"), Stream.StreamId);
	Meta->SetStringField(TEXT("group"), Stream.ActorName);
	Meta->SetStringField(TEXT("role"), TEXT("imu"));
	Meta->SetStringField(TEXT("name"), FString::Printf(TEXT("%s %s IMU"), *Stream.ActorName, *Target));
	Meta->SetStringField(TEXT("fmt"), TEXT("imu_f64x18"));
	TArray<TSharedPtr<FJsonValue>> Fields;
	for (const TCHAR* Field : RammsImuRecord::FieldNames)
	{
		Fields.Add(MakeShared<FJsonValueString>(Field));
	}
	Meta->SetArrayField(TEXT("fields"), Fields);
	TSharedRef<FJsonObject> Units = MakeShared<FJsonObject>();
	Units->SetStringField(TEXT("linear_acceleration"), TEXT("cm/s^2"));
	Units->SetStringField(TEXT("angular_velocity"), TEXT("deg/s"));
	Units->SetStringField(TEXT("angular_acceleration"), TEXT("deg/s^2"));
	Units->SetStringField(TEXT("linear_velocity"), TEXT("cm/s"));
	Units->SetStringField(TEXT("time"), TEXT("s"));
	Meta->SetObjectField(TEXT("units"), Units);
	Meta->SetStringField(TEXT("source"), URammsPhysicsImuSubsystem::SourceName(Imu->GetTrackedSource(Stream.Handle)));
	Meta->SetStringField(TEXT("actor"), Stream.ActorPath);
	Meta->SetStringField(TEXT("target"), Stream.Target);
	Meta->SetStringField(TEXT("body"), Imu->GetTrackedBodyName(Stream.Handle));
	Meta->SetNumberField(TEXT("first_seq"), static_cast<double>(Read.FirstSeq));
	Meta->SetNumberField(TEXT("count"), Count);
	Meta->SetNumberField(TEXT("dropped"), static_cast<double>(Read.Dropped));

	Streaming->BroadcastFrameData(Stream.Channel, Payload, Count, 1, TEXT("imu_f64x18"), RammsImuStreamImpl::CondensedJson(Meta));
	return true;
}

void URammsImuStreamSubsystem::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	for (int32 i = Streams.Num() - 1; i >= 0; --i)
	{
		if (!Pump(Streams[i]))
		{
			// Its actor or its point is gone.
			Release(Streams[i]);
			Streams.RemoveAt(i);
		}
	}
}

TStatId URammsImuStreamSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(URammsImuStreamSubsystem, STATGROUP_Tickables);
}

void URammsImuStreamSubsystem::Deinitialize()
{
	for (FRammsImuStream& Stream : Streams)
	{
		Release(Stream);
	}
	Streams.Reset();
	Super::Deinitialize();
}
