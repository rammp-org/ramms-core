#include "Modules/ModuleManager.h"
#include "ToolMenus.h"
#include "Editor.h"
#include "Subsystems/AssetEditorSubsystem.h"
#include "Kismet2/BlueprintEditorUtils.h"
#include "Engine/Blueprint.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/SCS_Node.h"
#include "KinovaGen3ControllerComponent.h"
#include "RammsSkeletalPoseComponent.h"
#include "Components/PoseableMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/PhysicsConstraintTemplate.h"
#include "PhysicsEngine/ConstraintInstance.h"
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

	// ── Skeletal Pose helpers ───────────────────────────────────────

	static bool BlueprintHasSkeletalPoseComponent(UBlueprint* BP)
	{
		if (!BP || !BP->SimpleConstructionScript)
		{
			return false;
		}

		const TArray<USCS_Node*> Nodes = BP->SimpleConstructionScript->GetAllNodes();
		for (USCS_Node* Node : Nodes)
		{
			if (Node && Node->ComponentTemplate && Node->ComponentTemplate->IsA<URammsSkeletalPoseComponent>())
			{
				return true;
			}
		}

		return false;
	}

	static void PopulateSkeletalPoseJoints(UBlueprint* BP)
	{
		if (!BP || !BP->SimpleConstructionScript)
		{
			UE_LOG(LogTemp, Warning, TEXT("[RammsCoreEditor] No blueprint/SCS to populate skeletal pose joints."));
			return;
		}

		const FScopedTransaction Transaction(LOCTEXT("PopulateSkeletalPoseJoints", "Populate Skeletal Pose Joints"));
		BP->Modify();

		const TArray<USCS_Node*> AllNodes = BP->SimpleConstructionScript->GetAllNodes();

		// Collect all UPoseableMeshComponent templates from SCS, keyed by variable name
		// (variable name matches the runtime component instance name)
		TMap<FName, UPoseableMeshComponent*> PoseableMeshes;
		for (USCS_Node* Node : AllNodes)
		{
			if (!Node || !Node->ComponentTemplate)
			{
				continue;
			}
			UPoseableMeshComponent* PMC = Cast<UPoseableMeshComponent>(Node->ComponentTemplate);
			if (PMC)
			{
				PoseableMeshes.Add(Node->GetVariableName(), PMC);
			}
		}

		if (PoseableMeshes.Num() == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("[RammsCoreEditor] No UPoseableMeshComponent templates found in blueprint."));
			return;
		}

		// Build joints from all poseable mesh skeletons (using physics constraints when available)
		TArray<FKinematicJointConfig> NewJoints;
		for (auto& Pair : PoseableMeshes)
		{
			const FName				MeshName = Pair.Key;
			UPoseableMeshComponent* Mesh = Pair.Value;

			if (!Mesh || !Mesh->GetSkinnedAsset())
			{
				continue;
			}

			const USkeletalMesh* SkelMesh = Cast<USkeletalMesh>(Mesh->GetSkinnedAsset());
			if (!SkelMesh)
			{
				continue;
			}

			// Try physics asset for accurate axis/limit info
			// Check component override first (PhysicsAssetOverride), then skeletal mesh asset
			const UPhysicsAsset* PhysAsset = Mesh->GetPhysicsAsset();
			if (!PhysAsset)
			{
				PhysAsset = SkelMesh->GetPhysicsAsset();
			}
			int32 PhysJointCount = 0;

			if (PhysAsset)
			{
				const FReferenceSkeleton& RefSkeleton = SkelMesh->GetRefSkeleton();

				for (const UPhysicsConstraintTemplate* Template : PhysAsset->ConstraintSetup)
				{
					if (!Template)
					{
						continue;
					}

					const FConstraintInstance& CI = Template->DefaultInstance;

					// Determine which constraint bone is the skeletal child
					FName				   ChildBone = CI.ConstraintBone1;
					EConstraintFrame::Type ChildFrame = EConstraintFrame::Frame1;
					{
						const int32 Bone1Idx = RefSkeleton.FindBoneIndex(CI.ConstraintBone1);
						const int32 Bone2Idx = RefSkeleton.FindBoneIndex(CI.ConstraintBone2);

						if (Bone1Idx != INDEX_NONE && Bone2Idx != INDEX_NONE)
						{
							// Walk up from Bone2 to see if Bone1 is ancestor → Bone2 is child
							for (int32 Idx = RefSkeleton.GetParentIndex(Bone2Idx); Idx != INDEX_NONE; Idx = RefSkeleton.GetParentIndex(Idx))
							{
								if (Idx == Bone1Idx)
								{
									ChildBone = CI.ConstraintBone2;
									ChildFrame = EConstraintFrame::Frame2;
									break;
								}
							}
						}
					}

					// Get constraint frame in child bone's local space
					const FTransform RefFrame = CI.GetRefFrame(ChildFrame);
					const FQuat		 FrameQuat = RefFrame.GetRotation();

					// Constraint axis directions in bone-local space
					const FVector TwistDir = FrameQuat.GetAxisX();
					const FVector Swing1Dir = FrameQuat.GetAxisZ();
					const FVector Swing2Dir = FrameQuat.GetAxisY();

					// Helper to find nearest cardinal axis
					auto FindClosestAxis = [](const FVector& Dir, bool& bNeg) -> EAxis::Type {
						const float AX = FMath::Abs(Dir.X), AY = FMath::Abs(Dir.Y), AZ = FMath::Abs(Dir.Z);
						if (AX >= AY && AX >= AZ)
						{
							bNeg = Dir.X < 0.f;
							return EAxis::X;
						}
						if (AY >= AZ)
						{
							bNeg = Dir.Y < 0.f;
							return EAxis::Y;
						}
						bNeg = Dir.Z < 0.f;
						return EAxis::Z;
					};

					// ── Angular (Revolute) ──────────────────────────────
					auto IsActive = [](EAngularConstraintMotion M) {
						return M == EAngularConstraintMotion::ACM_Free || M == EAngularConstraintMotion::ACM_Limited;
					};
					auto IsLimited = [](EAngularConstraintMotion M) {
						return M == EAngularConstraintMotion::ACM_Limited;
					};

					const EAngularConstraintMotion TwistMotion = CI.GetAngularTwistMotion();
					const EAngularConstraintMotion Swing1Motion = CI.GetAngularSwing1Motion();
					const EAngularConstraintMotion Swing2Motion = CI.GetAngularSwing2Motion();

					const bool bTwist = IsActive(TwistMotion);
					const bool bSwing1 = IsActive(Swing1Motion);
					const bool bSwing2 = IsActive(Swing2Motion);

					if (bTwist || bSwing1 || bSwing2)
					{
						struct AngCandidate
						{
							EAngularConstraintMotion Motion;
							FVector					 Dir;
							float					 Limit;
						};
						TArray<AngCandidate, TInlineAllocator<3>> Cands;
						if (bTwist)
							Cands.Add({ TwistMotion, TwistDir, CI.GetAngularTwistLimit() });
						if (bSwing1)
							Cands.Add({ Swing1Motion, Swing1Dir, CI.GetAngularSwing1Limit() });
						if (bSwing2)
							Cands.Add({ Swing2Motion, Swing2Dir, CI.GetAngularSwing2Limit() });

						// Prefer Limited over Free
						int32 Best = 0;
						for (int32 i = 1; i < Cands.Num(); ++i)
						{
							if (IsLimited(Cands[i].Motion) && !IsLimited(Cands[Best].Motion))
								Best = i;
						}

						FKinematicJointConfig Joint;
						Joint.MeshComponentName = MeshName;
						Joint.BoneName = ChildBone;
						Joint.JointName = CI.JointName;
						Joint.JointType = EKinematicJointType::Revolute;

						bool bNeg = false;
						Joint.Axis = FindClosestAxis(Cands[Best].Dir, bNeg);
						Joint.bInvertDirection = bNeg;

						if (IsLimited(Cands[Best].Motion))
						{
							Joint.MinValue = -Cands[Best].Limit;
							Joint.MaxValue = Cands[Best].Limit;
							Joint.bEnforceLimits = true;
						}
						else
						{
							Joint.MinValue = -180.f;
							Joint.MaxValue = 180.f;
							Joint.bEnforceLimits = false;
						}

						NewJoints.Add(Joint);
						++PhysJointCount;
						continue;
					}

					// ── Linear (Prismatic) ──────────────────────────────
					auto IsLinActive = [](ELinearConstraintMotion M) {
						return M == ELinearConstraintMotion::LCM_Free || M == ELinearConstraintMotion::LCM_Limited;
					};
					auto IsLinLimited = [](ELinearConstraintMotion M) {
						return M == ELinearConstraintMotion::LCM_Limited;
					};

					struct LinCandidate
					{
						ELinearConstraintMotion Motion;
						FVector					Dir;
					};
					TArray<LinCandidate, TInlineAllocator<3>> LinCands;
					if (IsLinActive(CI.GetLinearXMotion()))
						LinCands.Add({ CI.GetLinearXMotion(), FrameQuat.GetAxisX() });
					if (IsLinActive(CI.GetLinearYMotion()))
						LinCands.Add({ CI.GetLinearYMotion(), FrameQuat.GetAxisY() });
					if (IsLinActive(CI.GetLinearZMotion()))
						LinCands.Add({ CI.GetLinearZMotion(), FrameQuat.GetAxisZ() });

					if (LinCands.Num() > 0)
					{
						int32 Best = 0;
						for (int32 i = 1; i < LinCands.Num(); ++i)
						{
							if (IsLinLimited(LinCands[i].Motion) && !IsLinLimited(LinCands[Best].Motion))
								Best = i;
						}

						FKinematicJointConfig Joint;
						Joint.MeshComponentName = MeshName;
						Joint.BoneName = ChildBone;
						Joint.JointName = CI.JointName;
						Joint.JointType = EKinematicJointType::Prismatic;

						bool bNeg = false;
						Joint.Axis = FindClosestAxis(LinCands[Best].Dir, bNeg);
						Joint.bInvertDirection = bNeg;

						const float LinearLimit = CI.GetLinearLimit();
						if (LinearLimit > 0.0f)
						{
							Joint.MinValue = -LinearLimit;
							Joint.MaxValue = LinearLimit;
							Joint.bEnforceLimits = true;
						}
						else
						{
							Joint.MinValue = -100.f;
							Joint.MaxValue = 100.f;
							Joint.bEnforceLimits = false;
						}

						NewJoints.Add(Joint);
						++PhysJointCount;
					}
				}

				UE_LOG(LogTemp, Log, TEXT("[RammsCoreEditor] Mesh '%s': %d joints from physics asset '%s'"),
					*MeshName.ToString(), PhysJointCount, *PhysAsset->GetName());
			}

			if (PhysJointCount == 0)
			{
				// Fallback: one revolute joint per non-root bone
				const FReferenceSkeleton& RefSkeleton = SkelMesh->GetRefSkeleton();
				const int32				  NumBones = RefSkeleton.GetNum();

				for (int32 i = 1; i < NumBones; ++i)
				{
					FKinematicJointConfig Joint;
					Joint.MeshComponentName = MeshName;
					Joint.BoneName = RefSkeleton.GetBoneName(i);
					Joint.JointName = Joint.BoneName;
					Joint.JointType = EKinematicJointType::Revolute;
					Joint.Axis = EAxis::X;
					NewJoints.Add(Joint);
				}

				UE_LOG(LogTemp, Log, TEXT("[RammsCoreEditor] Mesh '%s': %d joints from skeleton (no physics asset)"),
					*MeshName.ToString(), NumBones - 1);
			}
		}

		// Apply joints to all SkeletalPoseComponent templates
		bool bAny = false;
		for (USCS_Node* Node : AllNodes)
		{
			URammsSkeletalPoseComponent* PoseComp = Node ? Cast<URammsSkeletalPoseComponent>(Node->ComponentTemplate) : nullptr;
			if (!PoseComp)
			{
				continue;
			}

			PoseComp->Modify();
			PoseComp->Joints = NewJoints;
			bAny = true;

			UE_LOG(LogTemp, Log, TEXT("[RammsCoreEditor] Populated %d joints across %d meshes for '%s'"),
				NewJoints.Num(), PoseableMeshes.Num(), *PoseComp->GetName());
		}

		if (bAny)
		{
			FBlueprintEditorUtils::MarkBlueprintAsStructurallyModified(BP);
			FBlueprintEditorUtils::MarkBlueprintAsModified(BP);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("[RammsCoreEditor] No RammsSkeletalPoseComponent found."));
		}
	}

	// ── Menu Registration ───────────────────────────────────────────

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

			// Gen3 populate button
			{
				FToolMenuEntry& Entry = Section.AddEntry(FToolMenuEntry::InitToolBarButton(
					"RammsCore_PopulateGen3Joints",
					FToolUIActionChoice(FToolMenuExecuteAction::CreateStatic(&FRammsCoreEditorModule::ExecutePopulateGen3Joints)),
					LOCTEXT("PopulateGen3JointsLabel", "Populate Gen3 Joints"),
					LOCTEXT("PopulateGen3JointsTooltip", "Auto-populate Gen3 joint settings from the Physics Asset constraints."),
					FSlateIcon()));

				Entry.Visibility = FToolMenuVisibilityChoice(TAttribute<bool>::CreateStatic(&FRammsCoreEditorModule::CanExecutePopulateGen3Joints));
			}

			// Skeletal Pose populate button
			{
				FToolMenuEntry& Entry = Section.AddEntry(FToolMenuEntry::InitToolBarButton(
					"RammsCore_PopulateSkeletalPoseJoints",
					FToolUIActionChoice(FToolMenuExecuteAction::CreateStatic(&FRammsCoreEditorModule::ExecutePopulateSkeletalPoseJoints)),
					LOCTEXT("PopulateSkeletalPoseLabel", "Populate Skeletal Pose"),
					LOCTEXT("PopulateSkeletalPoseTooltip", "Auto-populate joints from all poseable mesh skeletons on this actor."),
					FSlateIcon()));

				Entry.Visibility = FToolMenuVisibilityChoice(TAttribute<bool>::CreateStatic(&FRammsCoreEditorModule::CanExecutePopulateSkeletalPose));
			}
		}
	}

	// ── Gen3 Callbacks ──────────────────────────────────────────────

	static void ExecutePopulateGen3Joints(const FToolMenuContext& Context)
	{
		UBlueprint* BP = GetActiveBlueprintEditor();
		if (BP)
		{
			UE_LOG(LogTemp, Log, TEXT("[RammsCoreEditor] Populating Gen3 joints for blueprint: %s"), *BP->GetName());
		}
		PopulateGen3Joints(BP);
	}

	static bool CanExecutePopulateGen3Joints()
	{
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

	// ── Skeletal Pose Callbacks ─────────────────────────────────────

	static void ExecutePopulateSkeletalPoseJoints(const FToolMenuContext& Context)
	{
		UBlueprint* BP = GetActiveBlueprintEditor();
		if (BP)
		{
			UE_LOG(LogTemp, Log, TEXT("[RammsCoreEditor] Populating skeletal pose joints for blueprint: %s"), *BP->GetName());
		}
		PopulateSkeletalPoseJoints(BP);
	}

	static bool CanExecutePopulateSkeletalPose()
	{
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
				if (BlueprintHasSkeletalPoseComponent(BP))
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
