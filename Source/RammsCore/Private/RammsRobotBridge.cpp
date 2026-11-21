// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRobotBridge.h"
#include "RammsRobotDefinition.h"
#include "RammsRobotSelection.h"
#include "Dom/JsonObject.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "Misc/PackageName.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "UObject/Package.h"

namespace
{
	UWorld* BridgePlayWorld()
	{
		if (!GEngine)
		{
			return nullptr;
		}
		for (const FWorldContext& Context : GEngine->GetWorldContexts())
		{
			if ((Context.WorldType == EWorldType::PIE || Context.WorldType == EWorldType::Game) && Context.World())
			{
				return Context.World();
			}
		}
		return nullptr;
	}

	const URammsRobotSelectionSubsystem* Selection()
	{
		const UWorld* World = BridgePlayWorld();
		return World ? World->GetSubsystem<URammsRobotSelectionSubsystem>() : nullptr;
	}

	TArray<TSharedPtr<FJsonValue>> Names(const TArray<FName>& In)
	{
		TArray<TSharedPtr<FJsonValue>> Out;
		for (const FName& N : In)
		{
			Out.Add(MakeShared<FJsonValueString>(N.ToString()));
		}
		return Out;
	}
} // namespace

bool URammsRobotBridge::IsKnownRobot(const FString& IdOrAlias)
{
	const FString W = IdOrAlias.TrimStartAndEnd();
	return W.IsEmpty() || W.Equals(TEXT("ask"), ESearchCase::IgnoreCase) || URammsRobotCatalog::IsClassPath(W)
		|| URammsRobotCatalog::FindDefinition(W) != nullptr;
}

FString URammsRobotBridge::ListRobots()
{
	const FString				   ActiveId = GetActiveRobotId();
	TArray<TSharedPtr<FJsonValue>> Robots;
	for (const URammsRobotDefinition* Def : URammsRobotCatalog::GetDefinitions(/*bIncludeHidden=*/true))
	{
		TSharedRef<FJsonObject> O = MakeShared<FJsonObject>();
		const FString			Id = Def->RobotId.ToString();
		const FString			Pawn = Def->PawnClass.ToSoftObjectPath().ToString();
		// Available: a native class is there (it is always loaded); an asset's
		// package is there to load (not loaded here).
		const FString PawnPackage = Def->PawnClass.ToSoftObjectPath().GetLongPackageName();
		const bool	  bAvailable = !PawnPackage.IsEmpty()
			&& (FPackageName::IsScriptPackage(PawnPackage) ? Def->PawnClass.Get() != nullptr : FPackageName::DoesPackageExist(PawnPackage));
		// The content root as a path: /Game, or a plugin's (/MyRobots).
		FString Root = FPackageName::GetPackageMountPoint(Def->GetPackage()->GetName(), /*InWithoutSlashes=*/false).ToString();
		Root.RemoveFromEnd(TEXT("/"));
		O->SetStringField(TEXT("id"), Id);
		O->SetStringField(TEXT("name"), Def->GetDisplayNameOrId().ToString());
		O->SetArrayField(TEXT("aliases"), Names(Def->Aliases));
		O->SetArrayField(TEXT("tags"), Names(Def->Tags));
		O->SetBoolField(TEXT("hidden"), Def->bHidden);
		O->SetStringField(TEXT("pawn"), Pawn);
		O->SetBoolField(TEXT("available"), bAvailable);
		O->SetStringField(TEXT("why"), bAvailable ? FString() : (Pawn.IsEmpty() ? FString(TEXT("no pawn class")) : FString(TEXT("pawn class not found"))));
		O->SetStringField(TEXT("root"), Root);
		O->SetBoolField(TEXT("active"), !ActiveId.IsEmpty() && Def->Matches(ActiveId));
		Robots.Add(MakeShared<FJsonValueObject>(O));
	}
	FString					  Out;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Out);
	FJsonSerializer::Serialize(Robots, Writer);
	return Out;
}

FString URammsRobotBridge::GetActiveRobotId()
{
	const URammsRobotSelectionSubsystem* S = Selection();
	return S ? S->GetActiveRobotId() : FString();
}

FString URammsRobotBridge::GetDefaultRobotId()
{
	return GetDefault<URammsRobotSettings>()->DefaultRobotId;
}

FString URammsRobotBridge::GetActiveRobotSource()
{
	const URammsRobotSelectionSubsystem* S = Selection();
	return S ? S->GetActiveChoice().Describe() : FString();
}

bool URammsRobotBridge::SetNextRobot(const FString& IdOrAlias)
{
	if (!IsKnownRobot(IdOrAlias))
	{
		return false;
	}
	URammsRobotCatalog::SetConsoleRobot(IdOrAlias.TrimStartAndEnd());
	return true;
}

bool URammsRobotBridge::RestartWithRobot(const FString& IdOrAlias)
{
	return IsKnownRobot(IdOrAlias) && URammsRobotCatalog::RestartWithRobot(BridgePlayWorld(), IdOrAlias);
}

bool URammsRobotBridge::OpenMapWithRobot(const FString& MapPath, const FString& IdOrAlias)
{
	return IsKnownRobot(IdOrAlias) && URammsRobotCatalog::OpenMapWithRobot(BridgePlayWorld(), MapPath, IdOrAlias);
}
