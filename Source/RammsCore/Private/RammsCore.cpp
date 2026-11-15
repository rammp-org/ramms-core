// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsCore.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "RammsChaosImuProvider.h"
#include "RammsImuProvider.h"
#include "ShaderCore.h"

#define LOCTEXT_NAMESPACE "FRammsCoreModule"

void FRammsCoreModule::StartupModule()
{
	// Register shader source directory so .usf files can be included via /RammsCore/...
	FString				PluginShaderDir;
	TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("RammsCore"));
	if (Plugin.IsValid())
	{
		PluginShaderDir = FPaths::Combine(Plugin->GetBaseDir(), TEXT("Shaders"));
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("RammsCore: Plugin not found via IPluginManager, using fallback path"));
		PluginShaderDir = FPaths::Combine(FPaths::ProjectPluginsDir(), TEXT("RammsCore"), TEXT("Shaders"));
	}
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

	// Physics-rate IMU data from Chaos (URammsPhysicsImuSubsystem); RammsMujocoSupport registers MuJoCo's.
	RammsImuProviders::Register(ERammsImuSource::Chaos, FRammsImuProviderFactory::CreateStatic(&MakeRammsChaosImuProvider));
}

void FRammsCoreModule::ShutdownModule()
{
	RammsImuProviders::Unregister(ERammsImuSource::Chaos);
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FRammsCoreModule, RammsCore)