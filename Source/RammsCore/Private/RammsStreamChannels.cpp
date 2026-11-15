// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsStreamChannels.h"

#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "RammsCameraStreams.h"
#include "RammsImuStreamSubsystem.h"
#include "RammsStreamSourceComponent.h"

namespace RammsStreamChannels
{
	TSet<int32> CameraSourceChannels(const UWorld* World, const UActorComponent* IgnoreSource)
	{
		TSet<int32> Used;
		if (!World)
		{
			return Used;
		}
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
				if (!IsValid(Source) || Source == IgnoreSource)
				{
					continue;
				}
				for (const ERammsCameraStreamRole Role : FRammsCameraStreams::AllRoles())
				{
					Used.Add(Source->ChannelID + FRammsCameraStreams::ChannelOffset(Role));
				}
			}
		}
		return Used;
	}

	TSet<int32> ImuStreamChannels(const UWorld* World, const FString& IgnoreActorPath, const FString& IgnoreTarget)
	{
		TSet<int32>						Used;
		const URammsImuStreamSubsystem* Streams = World ? World->GetSubsystem<URammsImuStreamSubsystem>() : nullptr;
		if (!Streams)
		{
			return Used;
		}
		for (const FRammsImuStream& Stream : Streams->GetStreams())
		{
			if (!IgnoreActorPath.IsEmpty() && Stream.ActorPath == IgnoreActorPath && Stream.Target == IgnoreTarget)
			{
				continue;
			}
			Used.Add(Stream.Channel);
		}
		return Used;
	}

	namespace
	{
		bool IsGroupFree(const TSet<int32>& Used, int32 BaseChannel)
		{
			for (const ERammsCameraStreamRole Role : FRammsCameraStreams::AllRoles())
			{
				if (Used.Contains(BaseChannel + FRammsCameraStreams::ChannelOffset(Role)))
				{
					return false;
				}
			}
			return true;
		}

		TSet<int32> AllUsed(const UWorld* World, const UActorComponent* IgnoreSource)
		{
			TSet<int32> Used = CameraSourceChannels(World, IgnoreSource);
			Used.Append(ImuStreamChannels(World));
			return Used;
		}
	} // namespace

	bool IsCameraBaseFree(const UWorld* World, int32 BaseChannel, const UActorComponent* IgnoreSource)
	{
		return FRammsCameraStreams::IsValidBaseChannel(BaseChannel) && IsGroupFree(AllUsed(World, IgnoreSource), BaseChannel);
	}

	int32 FindFreeCameraBase(const UWorld* World)
	{
		const TSet<int32> Used = AllUsed(World, nullptr);
		for (int32 Base = 0; Base <= FRammsCameraStreams::MaxBaseChannel; ++Base)
		{
			if (IsGroupFree(Used, Base))
			{
				return Base;
			}
		}
		return INDEX_NONE;
	}

	bool PickImuChannel(const UWorld* World, const FString& ActorPath, const FString& Target, int32 Requested,
		int32& OutChannel, FString& OutError)
	{
		const TSet<int32> Cameras = CameraSourceChannels(World);
		if (Requested >= 0)
		{
			if (Requested > MAX_uint16)
			{
				OutError = FString::Printf(TEXT("channel %d is out of range (0..65535)"), Requested);
				return false;
			}
			if (Cameras.Contains(Requested))
			{
				OutError = FString::Printf(
					TEXT("channel %d is used by a camera stream source (each takes C, C+100, C+200 and C+300)"), Requested);
				return false;
			}
			OutChannel = Requested;
			return true;
		}

		// A restarted stream keeps its channel, so subscribed clients keep it -- unless a camera has taken it since.
		if (const URammsImuStreamSubsystem* Streams = World ? World->GetSubsystem<URammsImuStreamSubsystem>() : nullptr)
		{
			for (const FRammsImuStream& Stream : Streams->GetStreams())
			{
				if (Stream.ActorPath == ActorPath && Stream.Target == Target && Stream.Channel >= 0 && !Cameras.Contains(Stream.Channel))
				{
					OutChannel = Stream.Channel;
					return true;
				}
			}
		}

		const TSet<int32> Imu = ImuStreamChannels(World, ActorPath, Target);
		for (int32 Channel = URammsImuStreamSubsystem::FirstChannel; Channel <= MAX_uint16; ++Channel)
		{
			if (!Cameras.Contains(Channel) && !Imu.Contains(Channel))
			{
				OutChannel = Channel;
				return true;
			}
		}
		OutError = FString::Printf(TEXT("no free channel from %d"), URammsImuStreamSubsystem::FirstChannel);
		return false;
	}
} // namespace RammsStreamChannels
