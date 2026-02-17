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
#include "ToolMenu.h"
#include "ToolMenuContext.h"
#include "Framework/Application/SlateApplication.h"
#include "Widgets/Docking/SDockTab.h"

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
	static UBlueprint* GetActiveBlueprintEditor()
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

		// Try to find the window that has keyboard focus
		TSharedPtr<SWindow> FocusedWindow = FSlateApplication::Get().FindBestParentWindowForDialogs(nullptr);
		if (!FocusedWindow.IsValid())
		{
			FocusedWindow = FSlateApplication::Get().GetActiveTopLevelWindow();
		}

		if (FocusedWindow.IsValid())
		{
			UE_LOG(LogTemp, Log, TEXT("[RammsCoreEditor] Focused window: %s"), *FocusedWindow->GetTitle().ToString());

			// Get all open blueprint editors
			TArray<UObject*> EditedAssets = AssetSubsystem->GetAllEditedAssets();

			for (UObject* Asset : EditedAssets)
			{
				if (UBlueprint* BP = Cast<UBlueprint>(Asset))
				{
					IAssetEditorInstance* Editor = AssetSubsystem->FindEditorForAsset(BP, false);
					if (Editor && Editor->GetAssociatedTabManager())
					{
						TSharedPtr<SDockTab> EditorTab = Editor->GetAssociatedTabManager()->GetOwnerTab();
						if (EditorTab.IsValid())
						{
							// Check if this editor's tab is in the focused window
							TSharedPtr<SWindow> EditorWindow = EditorTab->GetParentWindow();
							bool				bInFocusedWindow = EditorWindow.IsValid() && EditorWindow == FocusedWindow;
							bool				bIsForeground = EditorTab->IsForeground();

							FString EditorTabLabel = EditorTab->GetTabLabel().ToString();
							UE_LOG(LogTemp, Log, TEXT("[RammsCoreEditor]   Blueprint: %s | InFocusedWindow: %d | IsForeground: %d | Tab: %s"),
								*BP->GetName(), bInFocusedWindow, bIsForeground, *EditorTabLabel);

							if (bInFocusedWindow && bIsForeground)
							{
								UE_LOG(LogTemp, Log, TEXT("[RammsCoreEditor] >>> Selected blueprint: %s"), *BP->GetName());
								return BP;
							}
						}
					}
				}
			}
		}

		UE_LOG(LogTemp, Warning, TEXT("[RammsCoreEditor] Could not determine focused blueprint editor"));

		// Fallback: return first blueprint found
		TArray<UObject*> EditedAssets = AssetSubsystem->GetAllEditedAssets();
		for (UObject* Obj : EditedAssets)
		{
			if (UBlueprint* BP = Cast<UBlueprint>(Obj))
			{
				UE_LOG(LogTemp, Warning, TEXT("[RammsCoreEditor] Fallback: returning first blueprint: %s"), *BP->GetName());
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

		bool					 bAny = false;
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

			FToolMenuEntry& Entry = Section.AddEntry(FToolMenuEntry::InitToolBarButton(
				"RammsCore_PopulateGen3Joints",
				FToolUIActionChoice(FToolMenuExecuteAction::CreateStatic(&FRammsCoreEditorModule::ExecutePopulateGen3Joints)),
				LOCTEXT("PopulateGen3JointsLabel", "Populate Gen3 Joints"),
				LOCTEXT("PopulateGen3JointsTooltip", "Auto-populate Gen3 joint settings from the Physics Asset constraints."),
				FSlateIcon()));

			Entry.Visibility = FToolMenuVisibilityChoice(TAttribute<bool>::CreateStatic(&FRammsCoreEditorModule::CanExecutePopulateGen3Joints));
		}
	}

	static void ExecutePopulateGen3Joints(const FToolMenuContext& Context)
	{
		UBlueprint* BP = GetActiveBlueprintEditor();
		if (BP)
		{
			UE_LOG(LogTemp, Log, TEXT("[RammsCoreEditor] Populating joints for blueprint: %s"), *BP->GetName());
		}
		PopulateGen3Joints(BP);
	}

	static bool CanExecutePopulateGen3Joints()
	{
		// Since we can't access the context in TAttribute<bool>, use the fallback method
		if (!GEditor)
		{
			return false;
		}

		UAssetEditorSubsystem* AssetSubsystem = GEditor->GetEditorSubsystem<UAssetEditorSubsystem>();
		if (!AssetSubsystem)
		{
			return false;
		}

		TArray<UObject*> EditedAssets = AssetSubsystem->GetAllEditedAssets();
		for (UObject* Obj : EditedAssets)
		{
			if (UBlueprint* BP = Cast<UBlueprint>(Obj))
			{
				if (BlueprintHasGen3Component(BP))
				{
					return true;
				}
			}
		}

		return false;
	}
};

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FRammsCoreEditorModule, RammsCoreEditor)
