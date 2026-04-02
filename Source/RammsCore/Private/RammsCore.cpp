// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsCore.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "ShaderCore.h"

#define LOCTEXT_NAMESPACE "FRammsCoreModule"

void FRammsCoreModule::StartupModule()
{
	// Register shader source directory so .usf files can be included via /RammsCore/...
	FString PluginShaderDir = FPaths::Combine(
		IPluginManager::Get().FindPlugin(TEXT("RammsCore"))->GetBaseDir(),
		TEXT("Shaders"));
	FPaths::CollapseRelativeDirectories(PluginShaderDir);

	if (FPaths::DirectoryExists(PluginShaderDir))
	{
		AddShaderSourceDirectoryMapping(TEXT("/RammsCore"), PluginShaderDir);
		UE_LOG(LogTemp, Log, TEXT("RammsCore: Registered shader source directory: %s"), *PluginShaderDir);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("RammsCore: Shader directory not found at: %s"), *PluginShaderDir);
	}
}

void FRammsCoreModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FRammsCoreModule, RammsCore)