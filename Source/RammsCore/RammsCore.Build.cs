// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class RammsCore : ModuleRules
{
	public RammsCore(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
        // Suppress compiler warning C4702 for unreachable code, might be needed for eigen
        PublicDefinitions.Add("EIGEN_IGNORE_UNREACHABLE_CODE_WARNING=1");

		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				// Access Renderer Private headers for FViewInfo / FRayTracingScene.
				// Needed to get RDG-tracked TLAS SRV for proper RDG dependency ordering
				// in the GPU sensor ray trace pipeline.
				System.IO.Path.Combine(EngineDirectory, "Source", "Runtime", "Renderer", "Private"),
				System.IO.Path.Combine(EngineDirectory, "Source", "Runtime", "Renderer", "Internal"),
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				// Public headers expose UActorComponent / UDataTable / FKey, so
				// consumers of this module need these on their include path too.
				"CoreUObject",
				"Engine",
				"InputCore",
				// The control-surface model (RammsUI plugin's light RammsControl
				// module): contributor / provider / sink types in public headers.
				"RammsControl",
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"Slate",
				"SlateCore",
				"PhysicsCore",
				"AnimationCore",
				"RHI",
				"RHICore",
				"RenderCore",
				"Renderer",
				"Projects",
				// ... add private dependencies that you statically link with here ...	
                "Eigen",
				"Json",
				"JsonUtilities", // control surface as JSON for Remote Control
				"RammsUI",       // URammsUISubsystem: control-surface registry
			}
			);

		if (Target.bBuildEditor)
		{
			PrivateDependencyModuleNames.AddRange(
				new string[]
				{
					"UnrealEd"
				}
				);
		}
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
