// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsControlTypes.h"
#include "RammsControlSurfaceProvider.h"
#include "RammsControlSink.h"
#include "RammsRobotControlSurfaceComponent.generated.h"

class URammsRobotBaseComponent;
class IRammsControlContributor;

/**
 * The robot's control surface: one component per robot actor that
 *
 *  - gathers every IRammsControlContributor on the actor (differential
 *    drive, MeBot lift controller, 5-bar linkages, camera, arm...) and merges
 *    their descriptions into one FRammsControlSurface,
 *  - appends a raw "Motors" group for every RobotBase registry motor no
 *    contributor claims (type / range / units from FRammsMotorSpec), so a
 *    freshly imported robot with only a RobotBase is already controllable,
 *  - implements IRammsControlSurfaceProvider / IRammsControlSink: routes each
 *    command to the owning contributor (or straight to the RobotBase for a
 *    raw motor) and arbitrates between sources — an Autonomy or Remote
 *    command holds an axis over local input for ExternalHoldSeconds, the way
 *    the differential drive's external input does,
 *  - registers itself with URammsControlSurfaceRegistry (ramms-control) so
 *    generic panels and input components find every controllable robot in the
 *    world without this module depending on any of them.
 *
 * Nothing here is wired by name: add a contributor component to the actor and
 * its controls appear. The plain UFUNCTIONs (DescribeControlSurface,
 * SetControl...) mirror the interface for Python / Remote Control callers.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsRobotControlSurfaceComponent : public UActorComponent, public IRammsControlSurfaceProvider, public IRammsControlSink
{
	GENERATED_BODY()

public:
	URammsRobotControlSurfaceComponent();

	/** Shown by panels; empty = the owner's name. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Surface")
	FText RobotDisplayName;

	/** Expose registry motors no contributor claims as raw motor axes. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Surface")
	bool bExposeUnclaimedMotors = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Surface")
	FName UnclaimedMotorGroup = FName("Motors");

	/**
	 * Also gather contributors from actors held by this actor's
	 * UChildActorComponents.
	 *
	 * A child actor is how a robot is composed out of robots -- the Mebot
	 * carries its arm that way -- and from the driver's side that arm is part
	 * of the machine, not a separate one. Gathering only from the owner left
	 * those controls with nowhere to be published: the arm simulated, carried
	 * a teleop contributor, and could not be commanded by anything.
	 *
	 * A child actor with a control surface **of its own** is skipped, along
	 * with everything below it. It publishes itself, and gathering it here too
	 * would put every one of its controls on two surfaces, where releasing on
	 * one would not release on the other.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Surface")
	bool bGatherFromChildActors = true;

	/**
	 * How far to follow child actors. 1 is the arm on the base; deeper is an
	 * arm carrying its own tool.
	 *
	 * Bounded because the walk is over spawned actors rather than a static
	 * tree, and an actor that reaches itself through a chain of child actors
	 * would otherwise be gathered until the stack ran out. Visited actors are
	 * tracked as well, so this is a second line rather than the only one.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Surface", meta = (ClampMin = "0", ClampMax = "8"))
	int32 ChildActorGatherDepth = 2;

	/** The depth the walk will actually honour, whatever it was handed.
	 *
	 *  `ClampMax` above constrains a details panel and nothing else: the field
	 *  is BlueprintReadWrite and serialized, so Blueprint, C++ or an older
	 *  asset can carry any value at all. Since the recursion tests against it,
	 *  an out-of-range one would quietly remove the bound rather than be
	 *  rejected, so the walk clamps rather than trusts. */
	static constexpr int32 MaxChildActorGatherDepth = 8;

	/** How long a Remote / Autonomy command holds a control over local input. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control Surface", meta = (ClampMin = "0.0"))
	float ExternalHoldSeconds = 0.3f;

	/**
	 * Every component this surface LOOKS IN for contributors: the owner's, and
	 * those of the actors it carries.
	 *
	 * Distinct from GetContributorComponents below, which reports the
	 * contributors that actually published something. Both are useful and they
	 * are not interchangeable: a contributor that has been stood down publishes
	 * nothing and so is absent from the published set, which means that set can
	 * never be used to bring it back. Anything toggling suspension has to work
	 * from the scope instead.
	 *
	 * Public because anything acting on "the contributors of this robot" has to
	 * agree with the surface about who they are. The low-level drive mode is
	 * the case that matters: it stands contributors down so their actuators
	 * come back as raw axes, and enumerating the owner alone while the surface
	 * reached into carried actors would leave one claiming and writing motors
	 * the surface had already advertised as free.
	 */
	UFUNCTION(BlueprintCallable, Category = "Control Surface")
	void GetGatherScopeComponents(TArray<UActorComponent*>& OutComponents) const;

	/** Re-gather contributors and registry motors (also done on BeginPlay, on
	 *  the next tick, and when the RobotBase loads its table). */
	UFUNCTION(BlueprintCallable, Category = "Control Surface")
	void RebuildControlSurface();

	// --- Plain-function twins of the interfaces (Python / Remote Control) -----

	UFUNCTION(BlueprintPure, Category = "Control Surface")
	FRammsControlSurface DescribeControlSurface() const;

	UFUNCTION(BlueprintCallable, Category = "Control Surface")
	bool SetControl(FName Id, float Value, ERammsControlSource Source = ERammsControlSource::Script);

	UFUNCTION(BlueprintCallable, Category = "Control Surface")
	bool TriggerControl(FName Id, ERammsControlSource Source = ERammsControlSource::Script);

	UFUNCTION(BlueprintCallable, Category = "Control Surface")
	bool ReleaseControl(FName Id, ERammsControlSource Source = ERammsControlSource::Script);

	UFUNCTION(BlueprintPure, Category = "Control Surface")
	float GetControlValue(FName Id) const;

	/** The control's current commanded target — what a panel shows on a slider,
	 *  as opposed to the live readback of GetControlValue.
	 *
	 *  For a Position / Velocity control routed to a contributor, the
	 *  contributor is authoritative: it reports the target it is actually
	 *  holding, including one set by a direct call on the controller
	 *  (SetAngularMotorTarget, SetEndpointTarget, SetJointAngles...) that
	 *  never passed through this surface — and false once it has released or
	 *  disabled that control, whatever was last commanded here.
	 *
	 *  Otherwise (raw registry motors, rate axes, contributors that don't
	 *  track targets) it is the last value this surface applied, from any
	 *  source.
	 *
	 *  False means no target: nothing commanded yet, or released. */
	UFUNCTION(BlueprintPure, Category = "Control Surface")
	bool GetControlTarget(FName Id, float& OutTarget) const;

	/** The surface as JSON (FJsonObjectConverter), for Remote Control clients. */
	UFUNCTION(BlueprintCallable, Category = "Control Surface")
	FString GetControlSurfaceJson() const;

	/** The components currently contributing controls.
	 *
	 *  For callers that need tick ordering against whatever drives the robot
	 *  without naming controller classes: an input source ticks before these
	 *  so its command lands in the same frame. Returns components, not the
	 *  interface, because that is what AddTickPrerequisiteComponent takes. */
	UFUNCTION(BlueprintPure, Category = "Control Surface")
	TArray<UActorComponent*> GetContributorComponents() const;

	/** Every control surface registered in the world
	 *  (URammsControlSurfaceRegistry), for callers without subsystem access
	 *  such as Python. */
	UFUNCTION(BlueprintPure, Category = "Control Surface", meta = (WorldContext = "WorldContextObject"))
	static TArray<UObject*> FindControlSurfaces(const UObject* WorldContextObject);

	// --- IRammsControlSurfaceProvider ---------------------------------------
	virtual FRammsControlSurface GetControlSurface_Implementation() const override;
	virtual int32				 GetControlSurfaceVersion_Implementation() const override;

	// --- IRammsControlSink ----------------------------------------------------
	virtual bool				SetAxis_Implementation(FName Id, float Value, ERammsControlSource Source) override;
	virtual bool				TriggerAction_Implementation(FName Id, ERammsControlSource Source) override;
	virtual bool				ReleaseAxis_Implementation(FName Id, ERammsControlSource Source) override;
	virtual float				GetAxisValue_Implementation(FName Id) const override;
	virtual ERammsControlSource GetAxisOwner_Implementation(FName Id) const override;
	/** See GetControlTarget for the precedence rules this implements. */
	virtual bool GetAxisTarget_Implementation(FName Id, float& OutTarget) const override;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	/** Where a control routes: a contributor, or a raw RobotBase motor. */
	struct FRoute
	{
		TWeakObjectPtr<UObject> Contributor; // implements IRammsControlContributor
		FName					MotorId;	 // raw motor when Contributor is null
	};

	/** Who last drove a control and when (for source arbitration). */
	struct FHold
	{
		ERammsControlSource Source = ERammsControlSource::Script;
		double				Time = 0.0;
	};

	UFUNCTION()
	void OnRegistryLoaded(URammsRobotBaseComponent* InBase);

	static int32			  Priority(ERammsControlSource Source);
	bool					  MayDrive(FName Id, ERammsControlSource Source) const;
	void					  Hold(FName Id, ERammsControlSource Source);
	IRammsControlContributor* ContributorFor(FName Id) const;
	void					  EnsureBuilt() const;

	UPROPERTY(Transient)
	TObjectPtr<URammsRobotBaseComponent> Base;

	mutable FRammsControlSurface Surface;
	mutable TMap<FName, FRoute>	 Routes;
	mutable TMap<FName, FHold>	 Holds;
	/** Last value commanded per control (any source); cleared when a Position / Velocity axis is released. */
	TMap<FName, float> Targets;
	/** Append Actor's contributor-bearing components, then those of its child
	 *  actors, depth-first and bounded. */
	void GatherContributorComponents(AActor* Actor, int32 Depth, TSet<AActor*>& Visited,
		TArray<UActorComponent*>& OutComponents) const;

	mutable int32 Version = 0;
	mutable bool  bBuilt = false;
};
