// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsCrowdBoxSpawnPointsGenerator.h"

#include "GameFramework/Actor.h"
#include "MassSpawnLocationProcessor.h"
#include "RammsCrowdLog.h"

namespace
{
	static bool IsFarEnoughFromExistingPoints(const TArray<FVector>& ExistingPoints, const FVector& CandidatePoint, const float MinSeparationCm)
	{
		if (MinSeparationCm <= 0.0f)
		{
			return true;
		}

		const float MinSeparationSquared = FMath::Square(MinSeparationCm);
		for (const FVector& ExistingPoint : ExistingPoints)
		{
			if (FVector::DistSquared(ExistingPoint, CandidatePoint) < MinSeparationSquared)
			{
				return false;
			}
		}

		return true;
	}
} // namespace

void URammsCrowdBoxSpawnPointsGenerator::Generate(UObject& QueryOwner, TConstArrayView<FMassSpawnedEntityType> EntityTypes, int32 Count, FFinishedGeneratingSpawnDataSignature& FinishedGeneratingSpawnPointsDelegate) const
{
	TArray<FMassEntitySpawnDataGeneratorResult> Results;
	if (Count <= 0)
	{
		FinishedGeneratingSpawnPointsDelegate.Execute(Results);
		return;
	}

	BuildResultsFromEntityTypes(Count, EntityTypes, Results);
	if (Results.IsEmpty())
	{
		UE_LOG(LogRammsCrowd, Warning, TEXT("[%s] Box generator produced no results for Count=%d. Check that the MassEntityConfig loads and validates."),
			*GetNameSafe(&QueryOwner),
			Count);
		FinishedGeneratingSpawnPointsDelegate.Execute(Results);
		return;
	}

	const AActor*	 OwnerActor = Cast<const AActor>(&QueryOwner);
	const FTransform OwnerTransform = OwnerActor != nullptr ? OwnerActor->GetActorTransform() : FTransform::Identity;
	const FQuat		 BaseRotation = bAlignToOwnerRotation ? OwnerTransform.GetRotation() : FQuat::Identity;
	FRandomStream	 RandomStream(GetRandomSelectionSeed());
	TArray<FVector>	 AcceptedLocations;
	AcceptedLocations.Reserve(Count);

	auto BuildRandomTransform = [&](const FVector& Location) -> FTransform {
		FQuat Rotation = BaseRotation;
		if (bRandomizeYaw)
		{
			const float RandomYawDegrees = RandomStream.FRandRange(-180.0f, 180.0f);
			Rotation = FQuat(FVector::UpVector, FMath::DegreesToRadians(RandomYawDegrees)) * Rotation;
		}

		return FTransform(Rotation, Location);
	};

	for (FMassEntitySpawnDataGeneratorResult& Result : Results)
	{
		Result.SpawnDataProcessor = UMassSpawnLocationProcessor::StaticClass();
		Result.SpawnData.InitializeAs<FMassTransformsSpawnData>();
		FMassTransformsSpawnData& TransformData = Result.SpawnData.GetMutable<FMassTransformsSpawnData>();
		TransformData.Transforms.Reserve(Result.NumEntities);

		for (int32 EntityIndex = 0; EntityIndex < Result.NumEntities; ++EntityIndex)
		{
			FVector SelectedLocation = OwnerTransform.TransformPosition(LocalCenter);
			bool	bFoundSeparatedLocation = false;

			for (int32 AttemptIndex = 0; AttemptIndex < MaxPlacementAttemptsPerEntity; ++AttemptIndex)
			{
				const FVector LocalOffset(
					RandomStream.FRandRange(-BoxExtent.X, BoxExtent.X),
					RandomStream.FRandRange(-BoxExtent.Y, BoxExtent.Y),
					RandomStream.FRandRange(-BoxExtent.Z, BoxExtent.Z));
				const FVector CandidateLocation = OwnerTransform.TransformPosition(LocalCenter + LocalOffset);
				if (IsFarEnoughFromExistingPoints(AcceptedLocations, CandidateLocation, MinSeparationCm))
				{
					SelectedLocation = CandidateLocation;
					bFoundSeparatedLocation = true;
					break;
				}
			}

			if (bFoundSeparatedLocation == false)
			{
				const FVector LocalOffset(
					RandomStream.FRandRange(-BoxExtent.X, BoxExtent.X),
					RandomStream.FRandRange(-BoxExtent.Y, BoxExtent.Y),
					RandomStream.FRandRange(-BoxExtent.Z, BoxExtent.Z));
				SelectedLocation = OwnerTransform.TransformPosition(LocalCenter + LocalOffset);
			}

			AcceptedLocations.Add(SelectedLocation);
			TransformData.Transforms.Add(BuildRandomTransform(SelectedLocation));
		}
	}

	UE_LOG(LogRammsCrowd, Display, TEXT("[%s] Box generator produced %d transforms across %d result buckets"),
		*GetNameSafe(&QueryOwner),
		AcceptedLocations.Num(),
		Results.Num());
	FinishedGeneratingSpawnPointsDelegate.Execute(Results);
}
