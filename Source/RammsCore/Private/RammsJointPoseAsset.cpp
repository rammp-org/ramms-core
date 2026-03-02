// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsJointPoseAsset.h"

bool URammsJointPoseAsset::GetPoseByIndex(int32 Index, TArray<float>& OutJointAngles) const
{
	if (!Poses.IsValidIndex(Index))
	{
		return false;
	}
	OutJointAngles = Poses[Index].JointAngles;
	return true;
}

bool URammsJointPoseAsset::GetPoseByName(const FString& Name, TArray<float>& OutJointAngles) const
{
	for (const FRammsJointPose& Pose : Poses)
	{
		if (Pose.PoseName.Equals(Name, ESearchCase::IgnoreCase))
		{
			OutJointAngles = Pose.JointAngles;
			return true;
		}
	}
	return false;
}

TArray<FString> URammsJointPoseAsset::GetPoseNames() const
{
	TArray<FString> Names;
	Names.Reserve(Poses.Num());
	for (const FRammsJointPose& Pose : Poses)
	{
		Names.Add(Pose.PoseName);
	}
	return Names;
}
