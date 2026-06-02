// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "MassEntitySpawnDataGeneratorBase.h"
#include "RammsCrowdBoxSpawnPointsGenerator.generated.h"

UCLASS(BlueprintType, EditInlineNew, meta = (DisplayName = "RAMMS Box SpawnPoints Generator"))
class RAMMSCORE_API URammsCrowdBoxSpawnPointsGenerator : public UMassEntitySpawnDataGeneratorBase
{
	GENERATED_BODY()

public:
	virtual void Generate(UObject& QueryOwner, TConstArrayView<FMassSpawnedEntityType> EntityTypes, int32 Count, FFinishedGeneratingSpawnDataSignature& FinishedGeneratingSpawnPointsDelegate) const override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box Generator")
	FVector LocalCenter = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box Generator")
	FVector BoxExtent = FVector(500.0f, 500.0f, 100.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box Generator", meta = (ClampMin = "0.0"))
	float MinSeparationCm = 75.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box Generator", meta = (ClampMin = "1"))
	int32 MaxPlacementAttemptsPerEntity = 12;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box Generator")
	bool bAlignToOwnerRotation = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Box Generator")
	bool bRandomizeYaw = true;
};
