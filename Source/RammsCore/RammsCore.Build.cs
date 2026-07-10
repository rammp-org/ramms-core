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
				"CoreUObject",
				"Engine",
				// ... add other public dependencies that you statically link with here ...
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
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
