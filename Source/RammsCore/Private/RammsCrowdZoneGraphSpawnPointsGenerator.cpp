// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsCrowdZoneGraphSpawnPointsGenerator.h"

#include "MassSpawnLocationProcessor.h"
#include "Math/RotationMatrix.h"
#include "RammsCrowdLog.h"
#include "ZoneGraphData.h"
#include "ZoneGraphQuery.h"
#include "ZoneGraphSubsystem.h"

void URammsCrowdZoneGraphSpawnPointsGenerator::Generate(UObject& QueryOwner, TConstArrayView<FMassSpawnedEntityType> EntityTypes, int32 Count, FFinishedGeneratingSpawnDataSignature& FinishedGeneratingSpawnPointsDelegate) const
{
	TArray<FMassEntitySpawnDataGeneratorResult> Results;
	if (Count <= 0)
	{
		FinishedGeneratingSpawnPointsDelegate.Execute(Results);
		return;
	}

	UWorld*					   World = QueryOwner.GetWorld();
	const UZoneGraphSubsystem* ZoneGraphSubsystem = World != nullptr ? UWorld::GetSubsystem<UZoneGraphSubsystem>(World) : nullptr;
	if (ZoneGraphSubsystem == nullptr)
	{
		UE_LOG(LogRammsCrowd, Warning, TEXT("[%s] ZoneGraph generator could not find UZoneGraphSubsystem"), *GetNameSafe(&QueryOwner));
		FinishedGeneratingSpawnPointsDelegate.Execute(Results);
		return;
	}

	FRandomStream									RandomStream(GetRandomSelectionSeed());
	TArray<FTransform>								CandidateTransforms;
	const TConstArrayView<FRegisteredZoneGraphData> RegisteredZoneGraphs = ZoneGraphSubsystem->GetRegisteredZoneGraphData();
	for (const FRegisteredZoneGraphData& RegisteredZoneGraph : RegisteredZoneGraphs)
	{
		if (RegisteredZoneGraph.bInUse && RegisteredZoneGraph.ZoneGraphData != nullptr)
		{
			GeneratePointsForZoneGraphData(*RegisteredZoneGraph.ZoneGraphData, CandidateTransforms, RandomStream);
		}
	}

	if (CandidateTransforms.IsEmpty())
	{
		UE_LOG(LogRammsCrowd, Warning, TEXT("[%s] ZoneGraph generator found no candidate lane transforms"), *GetNameSafe(&QueryOwner));
		FinishedGeneratingSpawnPointsDelegate.Execute(Results);
		return;
	}

	for (int32 Index = 0; Index < CandidateTransforms.Num(); ++Index)
	{
		const int32 SwapIndex = RandomStream.RandHelper(CandidateTransforms.Num());
		CandidateTransforms.Swap(Index, SwapIndex);
	}

	if (CandidateTransforms.Num() > Count)
	{
		CandidateTransforms.SetNum(Count);
	}

	BuildResultsFromEntityTypes(Count, EntityTypes, Results);
	if (Results.IsEmpty())
	{
		UE_LOG(LogRammsCrowd, Warning, TEXT("[%s] ZoneGraph generator produced no entity results for Count=%d. Check that the MassEntityConfig loads and validates."),
			*GetNameSafe(&QueryOwner),
			Count);
		FinishedGeneratingSpawnPointsDelegate.Execute(Results);
		return;
	}

	int32		TransformIndex = 0;
	const int32 TransformCount = CandidateTransforms.Num();
	for (FMassEntitySpawnDataGeneratorResult& Result : Results)
	{
		Result.SpawnDataProcessor = UMassSpawnLocationProcessor::StaticClass();
		Result.SpawnData.InitializeAs<FMassTransformsSpawnData>();
		FMassTransformsSpawnData& TransformData = Result.SpawnData.GetMutable<FMassTransformsSpawnData>();
		TransformData.Transforms.Reserve(Result.NumEntities);

		for (int32 EntityIndex = 0; EntityIndex < Result.NumEntities; ++EntityIndex)
		{
			TransformData.Transforms.Add(CandidateTransforms[TransformIndex % TransformCount]);
			++TransformIndex;
		}
	}

	UE_LOG(LogRammsCrowd, Display, TEXT("[%s] ZoneGraph generator produced %d transforms across %d result buckets"),
		*GetNameSafe(&QueryOwner),
		CandidateTransforms.Num(),
		Results.Num());
	FinishedGeneratingSpawnPointsDelegate.Execute(Results);
}

void URammsCrowdZoneGraphSpawnPointsGenerator::GeneratePointsForZoneGraphData(const AZoneGraphData& ZoneGraphData, TArray<FTransform>& OutTransforms, const FRandomStream& RandomStream) const
{
	const float				 EffectiveMinGap = FMath::Max(MinGapCm, 1.0f);
	const float				 EffectiveMaxGap = FMath::Max(MaxGapCm, EffectiveMinGap);
	const FZoneGraphStorage& ZoneGraphStorage = ZoneGraphData.GetStorage();

	for (int32 LaneIndex = 0; LaneIndex < ZoneGraphStorage.Lanes.Num(); ++LaneIndex)
	{
		const FZoneLaneData& Lane = ZoneGraphStorage.Lanes[LaneIndex];
		if (LaneTagFilter.Pass(Lane.Tags) == false)
		{
			continue;
		}

		float LaneLength = 0.0f;
		UE::ZoneGraph::Query::GetLaneLength(ZoneGraphStorage, LaneIndex, LaneLength);
		if (LaneLength <= 0.0f)
		{
			continue;
		}

		float DistanceAlongLane = RandomStream.FRandRange(EffectiveMinGap, EffectiveMaxGap);
		while (DistanceAlongLane <= LaneLength)
		{
			FZoneGraphLaneLocation LaneLocation;
			UE::ZoneGraph::Query::CalculateLocationAlongLane(ZoneGraphStorage, LaneIndex, DistanceAlongLane, LaneLocation);

			FQuat Rotation = FQuat::Identity;
			if (bAlignToLaneDirection)
			{
				const FRotator FacingRotation = FRotationMatrix::MakeFromXZ(LaneLocation.Direction.GetSafeNormal(), LaneLocation.Up.GetSafeNormal()).Rotator();
				Rotation = FacingRotation.Quaternion();
			}

			OutTransforms.Add(FTransform(Rotation, LaneLocation.Position));
			DistanceAlongLane += RandomStream.FRandRange(EffectiveMinGap, EffectiveMaxGap);
		}
	}
}
