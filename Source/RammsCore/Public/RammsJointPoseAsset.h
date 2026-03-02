// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "RammsJointPoseAsset.generated.h"

/**
 * A single named joint pose (array of joint angles in degrees).
 */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsJointPose
{
	GENERATED_BODY()

	/** Display name for this pose */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose")
	FString PoseName;

	/** Joint angles in degrees, one per joint (ordered to match the controller's Joints array) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose")
	TArray<float> JointAngles;
};

/**
 * Data asset that stores one or more named joint poses for a robotic arm.
 * Can be used to define known arm configurations (e.g. "Home", "Stowed", "Reach")
 * and chain them into sequences.
 */
UCLASS(BlueprintType)
class RAMMSCORE_API URammsJointPoseAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	/** Ordered list of joint poses stored in this asset */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Joint Poses")
	TArray<FRammsJointPose> Poses;

	/** Get a pose by index. Returns false if index is out of range. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Joint Poses")
	bool GetPoseByIndex(int32 Index, TArray<float>& OutJointAngles) const;

	/** Get a pose by name. Returns false if not found. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Joint Poses")
	bool GetPoseByName(const FString& Name, TArray<float>& OutJointAngles) const;

	/** Get the number of poses in this asset */
	UFUNCTION(BlueprintPure, Category = "Ramms|Joint Poses")
	int32 GetNumPoses() const { return Poses.Num(); }

	/** Get all pose names */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Joint Poses")
	TArray<FString> GetPoseNames() const;
};
