#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RammsCoreBlueprintLibrary.generated.h"

UCLASS()
class RAMMSCORE_API URammsCoreBlueprintLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	// Allow the blueprint to determine if this is with editor or not. Note: this
	// is the same as RunningInPIE, but kept for semantic clarity in some cases
	UFUNCTION(BlueprintPure, BlueprintCallable, Category = "Ramms|Utils")
	static bool IsWithEditor()
	{
#if WITH_EDITOR
		return true;
#else
		return false;
#endif
	};

	// Allow the blueprint to determine whether the current world is running as PIE
	UFUNCTION(BlueprintPure, BlueprintCallable, Category = "Ramms|Utils", meta = (WorldContext = "WorldContextObject"))
	static bool IsPIE(const UObject* WorldContextObject)
	{
#if WITH_EDITOR
		if (WorldContextObject != nullptr)
		{
			if (const UWorld* World = WorldContextObject->GetWorld())
			{
				return World->WorldType == EWorldType::PIE;
			}
		}
#endif
		return false;
	};

	// Allow the blueprint to determine whether we are running outside of the
	// editor (e.g. in a packaged build or standalone) or not
	UFUNCTION(BlueprintPure, BlueprintCallable, Category = "Ramms|Utils", meta = (WorldContext = "WorldContextObject"))
	static bool IsOutsideEditor(const UObject* WorldContextObject)
	{
#if !WITH_EDITOR
		return true;
#else
		if (WorldContextObject != nullptr)
		{
			if (const UWorld* World = WorldContextObject->GetWorld())
			{
				return World->WorldType == EWorldType::Game;
			}
		}

		return false;
#endif
	};

	// Allow the blueprint to determine whether we are running in a packaged
	// build or not. Note: packaged build = not running with editor, but also
	// excludes some non-editor builds (e.g. some test builds). Development
	// builds are not packaged, but also not with editor, so this is a useful
	// distinction.
	UFUNCTION(BlueprintPure, BlueprintCallable, Category = "Ramms|Utils")
	static bool IsPackagedBuild()
	{
#if UE_BUILD_SHIPPING || UE_BUILD_TEST
		return true;
#else
		return false;
#endif
	};

	// Allow the blueprint to determine whether this is a debug build or not
	UFUNCTION(BlueprintPure, BlueprintCallable, Category = "Ramms|Utils")
	static bool IsDebugBuild()
	{
#if UE_BUILD_DEBUG
		return true;
#else
		return false;
#endif
	};

	// Allow the blueprint to determine whether this is a development build or not
	UFUNCTION(BlueprintPure, BlueprintCallable, Category = "Ramms|Utils")
	static bool IsDevelopmentBuild()
	{
#if UE_BUILD_DEVELOPMENT
		return true;
#else
		return false;
#endif
	};

	// Allow the blueprint to determine whether this is a shipping build or not
	UFUNCTION(BlueprintPure, BlueprintCallable, Category = "Ramms|Utils")
	static bool IsShippingBuild()
	{
#if UE_BUILD_SHIPPING
		return true;
#else
		return false;
#endif
	};

	// Allow the blueprint to determine whether this is a test build or not
	UFUNCTION(BlueprintPure, BlueprintCallable, Category = "Ramms|Utils")
	static bool IsTestBuild()
	{
#if UE_BUILD_TEST
		return true;
#else
		return false;
#endif
	};
};
