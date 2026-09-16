// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsControlTypes.h"
#include "RammsControlInputMap.h"
#include "RammsControlInputComponent.generated.h"

class APlayerController;
class UEnhancedInputComponent;
class UEnhancedInputLocalPlayerSubsystem;
class UInputAction;
class UInputMappingContext;
struct FInputActionInstance;

/**
 * Keyboard / gamepad -> the robot's control surface, through Enhanced Input.
 *
 * Put it on a pawn that has a control sink (its URammsRobotControlSurfaceComponent)
 * and give it one or more URammsControlInputMap assets. Whenever the pawn is
 * possessed by a local player it adds each map's mapping context, binds every
 * action in the map and forwards to the sink with Source = Keyboard (or
 * Gamepad): Axis bindings set the axis and release it on Completed,
 * Action bindings fire on Started, IncrementRate bindings move a Position /
 * Velocity target at a rate while held. It unbinds and releases everything on
 * unpossess. Nothing is wired by name: the map speaks control Ids, so the same
 * component and the same context serve every robot family.
 *
 * InjectAction lets touch panels and tests feed an action as if a key were
 * held, through the same path.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsControlInputComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsControlInputComponent();

	/** The maps to apply; each carries its mapping context and bindings. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Input")
	TArray<TObjectPtr<URammsControlInputMap>> InputMaps;

	/** Reported to the sink for arbitration. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Input")
	ERammsControlSource Source = ERammsControlSource::Keyboard;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Input")
	bool bLogCommands = false;

	/** True while bound to a local player's Enhanced Input component. */
	UFUNCTION(BlueprintPure, Category = "Control Input")
	bool IsBound() const { return BoundInputComponent.IsValid(); }

	/** Feed Action with Value for HoldSeconds (0 = this tick only), as if its
	 *  key were held — the path touch buttons and tests use. */
	UFUNCTION(BlueprintCallable, Category = "Control Input")
	void InjectAction(const UInputAction* Action, FVector Value, float HoldSeconds = 0.0f);

	/** InjectAction by the action asset's name (e.g. "IA_Ramms_Drive"), looked up
	 *  in the maps' bindings. Returns false when no map binds it. */
	UFUNCTION(BlueprintCallable, Category = "Control Input")
	bool InjectActionByName(FName ActionName, FVector Value, float HoldSeconds = 0.0f);

	/** Names of the actions bound right now (diagnostics). */
	UFUNCTION(BlueprintPure, Category = "Control Input")
	TArray<FName> GetBoundActionNames() const;

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

protected:
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	/** One map binding resolved against the sink's surface. */
	struct FSlot
	{
		TWeakObjectPtr<const URammsControlInputMap> Map;
		int32										Index = INDEX_NONE;
		TArray<FName>								Ids[3]; // X / Y / Z targets after wildcard expansion
	};

	struct FInjection
	{
		TWeakObjectPtr<const UInputAction> Action;
		FVector							   Value = FVector::ZeroVector;
		double							   Until = 0.0;
	};

	void	 TryBind();
	void	 Unbind();
	void	 ResolveSlots();
	UObject* ResolveSink();
	bool	 SurfaceAxis(FName Id, FRammsControlAxis& Out) const;
	void	 OnStarted(int32 Slot, const FInputActionInstance& Instance);
	void	 OnTriggered(int32 Slot, const FInputActionInstance& Instance);
	void	 OnCompleted(int32 Slot, const FInputActionInstance& Instance);
	float	 Component(const FVector& V, int32 Axis) const { return Axis == 0 ? static_cast<float>(V.X) : (Axis == 1 ? static_cast<float>(V.Y) : static_cast<float>(V.Z)); }

	UPROPERTY(Transient)
	TObjectPtr<UObject> Sink; // implements IRammsControlSink (+ IRammsControlSurfaceProvider)

	TWeakObjectPtr<UEnhancedInputComponent>			   BoundInputComponent;
	TWeakObjectPtr<APlayerController>				   BoundPC;
	TWeakObjectPtr<UEnhancedInputLocalPlayerSubsystem> BoundSubsystem;
	TArray<uint32>									   BindingHandles;
	/** Mapping contexts this component added (so unbinding removes only those, not ones others own). */
	TArray<TWeakObjectPtr<const UInputMappingContext>> AddedContexts;
	TArray<FSlot>									   Slots;
	TMap<FName, float>								   IncrementTargets;
	TSet<FName>										   HeldAxes;
	TArray<FInjection>								   Injections;
	int32											   SurfaceVersionSeen = -1;
	mutable FRammsControlSurface					   CachedSurface;
};
