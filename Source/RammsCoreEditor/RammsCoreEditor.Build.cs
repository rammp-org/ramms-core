using UnrealBuildTool;

public class RammsCoreEditor : ModuleRules
{
	public RammsCoreEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"UnrealEd",
				"ToolMenus",
				"Kismet",
				"KismetCompiler",
				"BlueprintGraph",
				"Slate",
				"SlateCore",
				"RammsCore",
				// The control-surface model (its own plugin), which RammsCore's
				// control-surface headers use: the deadman tests (Private/Tests)
				// live here, so their test contributor -- a UCLASS -- stays out of
				// game and Shipping builds.
				"RammsControl"
			}
		);
	}
}
