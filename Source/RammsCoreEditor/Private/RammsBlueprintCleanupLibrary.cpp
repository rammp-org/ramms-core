// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsBlueprintCleanupLibrary.h"
#include "EdGraph/EdGraph.h"
#include "EdGraph/EdGraphNode.h"
#include "Engine/Blueprint.h"
#include "K2Node_CallFunction.h"
#include "Kismet2/BlueprintEditorUtils.h"
#include "Kismet2/KismetEditorUtilities.h"

namespace
{
	bool IsInputNode(const UEdGraphNode* Node)
	{
		static const TArray<FString> InputClasses = {
			TEXT("K2Node_InputKey"), TEXT("K2Node_InputAction"), TEXT("K2Node_InputAxisEvent"),
			TEXT("K2Node_InputAxisKeyEvent"), TEXT("K2Node_InputTouch"), TEXT("K2Node_EnhancedInputAction"),
			TEXT("K2Node_GetInputActionValue"), TEXT("K2Node_InputDebugKey")
		};
		const FString ClassName = Node->GetClass()->GetName();
		return InputClasses.Contains(ClassName);
	}

	FString Describe(const UEdGraphNode* Node)
	{
		FString Out = Node->GetClass()->GetName();
		if (const UK2Node_CallFunction* Call = Cast<UK2Node_CallFunction>(Node))
		{
			Out += TEXT(":") + Call->FunctionReference.GetMemberName().ToString();
		}
		else
		{
			Out += TEXT(":") + Node->GetNodeTitle(ENodeTitleType::ListView).ToString().Replace(TEXT("\n"), TEXT(" "));
		}
		return Out;
	}
} // namespace

TArray<FString> URammsBlueprintCleanupLibrary::ListInputNodes(UBlueprint* Blueprint)
{
	TArray<FString> Out;
	if (!Blueprint)
	{
		return Out;
	}
	TArray<UEdGraph*> Graphs;
	Blueprint->GetAllGraphs(Graphs);
	for (UEdGraph* Graph : Graphs)
	{
		for (UEdGraphNode* Node : Graph->Nodes)
		{
			if (Node && IsInputNode(Node))
			{
				Out.Add(Graph->GetName() + TEXT("/") + Describe(Node));
			}
		}
	}
	return Out;
}

int32 URammsBlueprintCleanupLibrary::RemoveLegacyInputNodes(UBlueprint* Blueprint, const TArray<FName>& FunctionsToRemove)
{
	if (!Blueprint)
	{
		return 0;
	}
	TArray<UEdGraphNode*> ToRemove;
	TArray<UEdGraph*>	  Graphs;
	Blueprint->GetAllGraphs(Graphs);
	for (UEdGraph* Graph : Graphs)
	{
		for (UEdGraphNode* Node : Graph->Nodes)
		{
			if (!Node)
			{
				continue;
			}
			if (IsInputNode(Node))
			{
				ToRemove.Add(Node);
				continue;
			}
			if (const UK2Node_CallFunction* Call = Cast<UK2Node_CallFunction>(Node))
			{
				if (FunctionsToRemove.Contains(Call->FunctionReference.GetMemberName()))
				{
					ToRemove.Add(Node);
				}
			}
		}
	}
	if (ToRemove.Num() == 0)
	{
		return 0;
	}
	Blueprint->Modify();
	for (UEdGraphNode* Node : ToRemove)
	{
		UE_LOG(LogTemp, Log, TEXT("[BlueprintCleanup] %s: removing %s"), *Blueprint->GetName(), *Describe(Node));
		FBlueprintEditorUtils::RemoveNode(Blueprint, Node, /*bDontRecompile*/ true);
	}
	FBlueprintEditorUtils::MarkBlueprintAsStructurallyModified(Blueprint);
	FKismetEditorUtilities::CompileBlueprint(Blueprint);
	return ToRemove.Num();
}
