// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "MassEntitySpawnDataGeneratorBase.h"
#include "ZoneGraphTypes.h"
#include "RammsCrowdZoneGraphSpawnPointsGenerator.generated.h"

class AZoneGraphData;

UCLASS(BlueprintType, EditInlineNew, meta = (DisplayName = "RAMMS ZoneGraph SpawnPoints Generator"))
class RAMMSCORE_API URammsCrowdZoneGraphSpawnPointsGenerator : public UMassEntitySpawnDataGeneratorBase
{
	GENERATED_BODY()

public:
	virtual void Generate(UObject& QueryOwner, TConstArrayView<FMassSpawnedEntityType> EntityTypes, int32 Count, FFinishedGeneratingSpawnDataSignature& FinishedGeneratingSpawnPointsDelegate) const override;

	UPROPERTY(EditAnywhere, Category = "ZoneGraph Generator")
	FZoneGraphTagFilter LaneTagFilter;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ZoneGraph Generator", meta = (ClampMin = "1.0"))
	float MinGapCm = 150.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ZoneGraph Generator", meta = (ClampMin = "1.0"))
	float MaxGapCm = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ZoneGraph Generator")
	bool bAlignToLaneDirection = true;

private:
	void GeneratePointsForZoneGraphData(const AZoneGraphData& ZoneGraphData, TArray<FTransform>& OutTransforms, const FRandomStream& RandomStream) const;
};
