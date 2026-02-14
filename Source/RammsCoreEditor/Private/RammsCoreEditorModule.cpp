#include "Modules/ModuleManager.h"
#include "ToolMenus.h"
#include "Editor.h"
#include "Subsystems/AssetEditorSubsystem.h"
#include "Kismet2/BlueprintEditorUtils.h"
#include "Engine/Blueprint.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/SCS_Node.h"
#include "KinovaGen3ControllerComponent.h"
#include "ScopedTransaction.h"

#define LOCTEXT_NAMESPACE "FRammsCoreEditorModule"

class FRammsCoreEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		UToolMenus::RegisterStartupCallback(FSimpleMulticastDelegate::FDelegate::CreateRaw(this, &FRammsCoreEditorModule::RegisterMenus));
	}

	virtual void ShutdownModule() override
	{
		UToolMenus::UnRegisterStartupCallback(this);
		UToolMenus::UnregisterOwner(this);
	}

private:
	static UBlueprint* GetActiveBlueprint()
	{
		if (!GEditor)
		{
			return nullptr;
		}

		UAssetEditorSubsystem* AssetSubsystem = GEditor->GetEditorSubsystem<UAssetEditorSubsystem>();
		if (!AssetSubsystem)
		{
			return nullptr;
		}

		TArray<UObject*> EditedAssets = AssetSubsystem->GetAllEditedAssets();
		for (UObject* Obj : EditedAssets)
		{
			if (UBlueprint* BP = Cast<UBlueprint>(Obj))
			{
				return BP;
			}
		}

		return nullptr;
	}

	static bool BlueprintHasGen3Component(UBlueprint* BP)
	{
		if (!BP || !BP->SimpleConstructionScript)
		{
			return false;
		}

		const TArray<USCS_Node*> Nodes = BP->SimpleConstructionScript->GetAllNodes();
		for (USCS_Node* Node : Nodes)
		{
			if (Node && Node->ComponentTemplate && Node->ComponentTemplate->IsA<UKinovaGen3ControllerComponent>())
			{
				return true;
			}
		}

		return false;
	}

	static void PopulateGen3Joints(UBlueprint* BP)
	{
		if (!BP || !BP->SimpleConstructionScript)
		{
			UE_LOG(LogTemp, Warning, TEXT("[RammsCoreEditor] No blueprint/SCS to populate."));
			return;
		}

		const FScopedTransaction Transaction(LOCTEXT("PopulateGen3Joints", "Populate Gen3 Joints"));
		BP->Modify();

		bool bAny = false;
		const TArray<USCS_Node*> Nodes = BP->SimpleConstructionScript->GetAllNodes();
		for (USCS_Node* Node : Nodes)
		{
			UKinovaGen3ControllerComponent* Gen3 = Node ? Cast<UKinovaGen3ControllerComponent>(Node->ComponentTemplate) : nullptr;
			if (!Gen3)
			{
				continue;
			}

			Gen3->Modify();
			Gen3->AutoPopulateJoints(true);
			bAny = true;
		}

		if (bAny)
		{
			FBlueprintEditorUtils::MarkBlueprintAsStructurallyModified(BP);
			FBlueprintEditorUtils::MarkBlueprintAsModified(BP);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("[RammsCoreEditor] No Gen3 controller components found."));
		}
	}

	void RegisterMenus()
	{
		const TArray<FName> ToolbarNames = {
			FName("AssetEditor.BlueprintEditor.ToolBar"),
			FName("AssetEditor.BlueprintEditor.ToolBar.GraphName"),
			FName("AssetEditor.BlueprintEditor.ToolBar.DefaultsName"),
			FName("AssetEditor.BlueprintEditor.ToolBar.ComponentsName"),
			FName("AssetEditor.BlueprintEditor.ToolBar.InterfaceName"),
			FName("AssetEditor.BlueprintEditor.ToolBar.MacroName")
		};

		for (const FName& MenuName : ToolbarNames)
		{
			UToolMenu* ToolbarMenu = UToolMenus::Get()->ExtendMenu(MenuName);
			if (!ToolbarMenu)
			{
				continue;
			}

			FToolMenuSection& Section = ToolbarMenu->AddSection("RammsCore", LOCTEXT("RammsCoreSection", "RammsCore"));

			FToolMenuEntry Entry = FToolMenuEntry::InitToolBarButton(
				"RammsCore_PopulateGen3Joints",
				FUIAction(
					FExecuteAction::CreateLambda([]()
					{
						UBlueprint* BP = GetActiveBlueprint();
						PopulateGen3Joints(BP);
					}),
					FCanExecuteAction::CreateLambda([]() -> bool
					{
						UBlueprint* BP = GetActiveBlueprint();
						return BlueprintHasGen3Component(BP);
					})
				),
				LOCTEXT("PopulateGen3JointsLabel", "Populate Gen3 Joints"),
				LOCTEXT("PopulateGen3JointsTooltip", "Auto-populate Gen3 joint settings from the Physics Asset constraints."),
				FSlateIcon()
			);

			Section.AddEntry(Entry);
		}
	}
};

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FRammsCoreEditorModule, RammsCoreEditor)
