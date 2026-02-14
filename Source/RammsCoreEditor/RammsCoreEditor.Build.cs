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
				"RammsCore"
			}
		);
	}
}
