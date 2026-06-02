// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "MassSpawner.h"
#include "RammsCrowdTypes.h"
#include "RammsCrowdSpawner.generated.h"

class UMassEntitySpawnDataGeneratorBase;
#if WITH_EDITOR
struct FPropertyChangedEvent;
#endif

UCLASS(BlueprintType)
class RAMMSCORE_API ARammsCrowdSpawner : public AMassSpawner
{
	GENERATED_BODY()

public:
	ARammsCrowdSpawner();

	virtual void BeginPlay() override;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|Crowd")
	TObjectPtr<URammsCrowdAgentProfile> CrowdProfile = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|Crowd", meta = (ClampMin = "0"))
	int32 DesiredCount = 20;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|Crowd", meta = (ClampMin = "0.01"))
	float DesiredCountScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|Crowd")
	ERammsCrowdSpawnMode SpawnMode = ERammsCrowdSpawnMode::BoxArea;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|Crowd")
	bool bAutoApplyConfigurationOnBeginPlay = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|Crowd")
	bool bAutoRebuildConfigurationInEditor = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|Crowd", meta = (EditCondition = "SpawnMode == ERammsCrowdSpawnMode::BoxArea"))
	FRammsCrowdBoxSpawnSettings BoxSpawnSettings;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|Crowd", meta = (EditCondition = "SpawnMode == ERammsCrowdSpawnMode::ZoneGraphLanes"))
	FRammsCrowdZoneGraphSpawnSettings ZoneGraphSpawnSettings;

	UFUNCTION(BlueprintPure, Category = "Ramms|Crowd")
	bool HasValidCrowdProfile(FString& OutIssue) const;

	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Ramms|Crowd")
	void ApplyCrowdConfiguration();

	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Ramms|Crowd")
	void RespawnCrowd();

	UFUNCTION(BlueprintCallable, Category = "Ramms|Crowd")
	void DespawnCrowd();

	UFUNCTION(BlueprintCallable, Category = "Ramms|Crowd")
	void SetDesiredCount(int32 NewCount);

	UFUNCTION(BlueprintCallable, Category = "Ramms|Crowd")
	void SetDesiredCountScale(float NewScale);

private:
	bool							   CanUseMassSpawnerWorldApis() const;
	void							   ResetCrowdConfiguration();
	void							   ValidateConfiguredEntityTypes();
	void							   ConfigureEntityTypes();
	void							   ConfigureSpawnGenerators();
	UMassEntitySpawnDataGeneratorBase* BuildSpawnGenerator() const;
	int32							   GetEffectiveDesiredCount() const;
	float							   GetEffectiveDesiredCountScale() const;

	UFUNCTION()
	void HandleSpawningFinished();

	UFUNCTION()
	void HandleDespawningFinished();
};
