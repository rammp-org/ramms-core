// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRobotDefinition.h"

const FPrimaryAssetType URammsRobotDefinition::AssetType(TEXT("RammsRobot"));

FPrimaryAssetId URammsRobotDefinition::GetPrimaryAssetId() const
{
	// By the robot's id, so a definition renamed on disk keeps its identity.
	return FPrimaryAssetId(AssetType, RobotId.IsNone() ? GetFName() : RobotId);
}

bool URammsRobotDefinition::Matches(const FString& IdOrAlias) const
{
	const FString Wanted = IdOrAlias.TrimStartAndEnd();
	if (Wanted.IsEmpty())
	{
		return false;
	}
	if (!RobotId.IsNone() && RobotId.ToString().Equals(Wanted, ESearchCase::IgnoreCase))
	{
		return true;
	}
	for (const FName& Alias : Aliases)
	{
		if (!Alias.IsNone() && Alias.ToString().Equals(Wanted, ESearchCase::IgnoreCase))
		{
			return true;
		}
	}
	return false;
}

FText URammsRobotDefinition::GetDisplayNameOrId() const
{
	return DisplayName.IsEmpty() ? FText::FromName(RobotId.IsNone() ? GetFName() : RobotId) : DisplayName;
}
