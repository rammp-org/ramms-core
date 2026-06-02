// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "ZoneGraphTypes.h"
#include "RammsCrowdTypes.generated.h"

class UMassEntityConfigAsset;

UENUM(BlueprintType)
enum class ERammsCrowdActorKind : uint8
{
	Person	   UMETA(DisplayName = "Person"),
	Robot	   UMETA(DisplayName = "Robot"),
	Pet		   UMETA(DisplayName = "Pet"),
	WildAnimal UMETA(DisplayName = "Wild Animal"),
	Vehicle	   UMETA(DisplayName = "Vehicle")
};

UENUM(BlueprintType)
enum class ERammsCrowdSpawnMode : uint8
{
	BoxArea		   UMETA(DisplayName = "Box Area"),
	ZoneGraphLanes UMETA(DisplayName = "ZoneGraph Lanes")
};

USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsCrowdBoxSpawnSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd")
	FVector LocalCenter = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd")
	FVector BoxExtent = FVector(500.0f, 500.0f, 100.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd", meta = (ClampMin = "0.0"))
	float MinSeparationCm = 75.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd", meta = (ClampMin = "1"))
	int32 MaxPlacementAttemptsPerEntity = 12;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd")
	bool bAlignToOwnerRotation = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd")
	bool bRandomizeYaw = true;
};

USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsCrowdZoneGraphSpawnSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Crowd")
	FZoneGraphTagFilter LaneTagFilter;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd", meta = (ClampMin = "1.0"))
	float MinGapCm = 150.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd", meta = (ClampMin = "1.0"))
	float MaxGapCm = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd")
	bool bAlignToLaneDirection = true;
};

UCLASS(BlueprintType)
class RAMMSCORE_API URammsCrowdAgentProfile : public UDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd")
	FString DisplayName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd")
	ERammsCrowdActorKind ActorKind = ERammsCrowdActorKind::Person;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd")
	TSoftObjectPtr<UMassEntityConfigAsset> MassEntityConfig;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd", meta = (ClampMin = "0"))
	int32 DefaultCount = 20;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd", meta = (ClampMin = "0.01"))
	float DefaultCountScale = 1.0f;

	UFUNCTION(BlueprintPure, Category = "Ramms|Crowd")
	bool IsConfigured() const
	{
		return MassEntityConfig.IsNull() == false;
	}
};
