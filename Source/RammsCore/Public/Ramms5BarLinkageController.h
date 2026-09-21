// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Ramms5BarLinkageSpec.h"
#include "RammsControlContributor.h"
#include "RammsControlIds.h"
#include "Ramms5BarLinkageController.generated.h"

class URammsRobotBaseComponent;

/**
 * Drives one parallel 5-bar linkage (e.g. a lift_drive centre leg, or a seat
 * elevator) by positioning its shared endpoint in the linkage's local x-z plane.
 *
 * It is a thin consumer of the robot base component: it holds only its own
 * KINEMATIC config (an FRamms5BarLinkageSpec, from a data table or inline —
 * geometry the motor registry deliberately doesn't model) and commands its two
 * proximal Position motors *by Id* through the sibling URammsRobotBaseComponent,
 * which owns the physics backend. Add one component per 5-bar linkage on the
 * robot actor; each names its own two motor Ids.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URamms5BarLinkageController : public UActorComponent, public IRammsControlContributor
{
	GENERATED_BODY()

public:
	URamms5BarLinkageController();

	virtual void BeginPlay() override;

	/** Optional kinematic data table (row struct: FRamms5BarLinkageSpec). When
	 *  set with LinkageRow, its row overrides the inline Linkage below. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar")
	TObjectPtr<UDataTable> KinematicTable;

	/** Row in KinematicTable to use (ignored when KinematicTable is unset). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar")
	FName LinkageRow;

	/** Inline kinematic config, used when no KinematicTable/LinkageRow resolves. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "5-Bar")
	FRamms5BarLinkageSpec Linkage;

	// --- Command / query -----------------------------------------------------

	/** Position the linkage endpoint at TargetXZ (local x-z plane, cm): solve IK
	 *  and command both proximal motors. Returns false (and commands nothing) if
	 *  the target is unreachable or no base component is available. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|5-Bar")
	bool SetEndpointTarget(FVector2D TargetXZ);

	/** Convenience: keep the current endpoint X and move to height Z (cm). */
	UFUNCTION(BlueprintCallable, Category = "Ramms|5-Bar")
	bool SetEndpointHeight(float Z);

	/** Convenience: keep the current endpoint height and move fore/aft to X (cm).
	 *  The other half of what a 5-bar can do -- both proximal motors move for
	 *  either, so this is a translation of the endpoint, not a second joint. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|5-Bar")
	bool SetEndpointTranslation(float X);

	/** Command the two proximal joint angles directly (rad; A = X, B = Y). */
	UFUNCTION(BlueprintCallable, Category = "Ramms|5-Bar")
	void SetJointAngles(FVector2D AnglesAB);

	/** Current endpoint (local x-z, cm) from the live proximal joint angles.
	 *  If the live angles don't close the loop (distal links can't meet, or a
	 *  knee is bent past straight) this is the closest estimate — use
	 *  GetCurrentEndpointChecked to know. */
	UFUNCTION(BlueprintPure, Category = "Ramms|5-Bar")
	FVector2D GetCurrentEndpoint() const;

	/** As GetCurrentEndpoint, with bValid false when the live angles don't
	 *  describe a closed, feasible linkage (the result is then an estimate). */
	UFUNCTION(BlueprintPure, Category = "Ramms|5-Bar")
	FVector2D GetCurrentEndpointChecked(bool& bValid) const;

	/** Live proximal joint angles (rad; A = X, B = Y) read from the base. */
	UFUNCTION(BlueprintPure, Category = "Ramms|5-Bar")
	FVector2D GetCurrentJointAngles() const;

	/** Last endpoint target commanded via SetEndpointTarget/Height. */
	UFUNCTION(BlueprintPure, Category = "Ramms|5-Bar")
	FVector2D GetLastTarget() const { return LastTarget; }

	/** IK preview without commanding: joint angles for TargetXZ. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|5-Bar")
	FVector2D SolveTarget(FVector2D TargetXZ, bool& bReachable) const;

	/** True once a base component has been resolved. */
	UFUNCTION(BlueprintPure, Category = "Ramms|5-Bar")
	bool HasBase() const;

private:
	/** Resolve (and cache) the sibling base component; nullptr if none. */
	URammsRobotBaseComponent* EnsureBase() const;

	/** False when Angle lies outside the motor's authored ControlRange. */
	bool WithinMotorRange(URammsRobotBaseComponent& Base, FName MotorId, double Angle) const;

	/** Cached base component. Mutable: resolved lazily (BeginPlay order-safe). */
	UPROPERTY(Transient)
	mutable TObjectPtr<URammsRobotBaseComponent> BaseComponent = nullptr;

	/** The resolved spec (table row if present, else the inline Linkage). */
	FRamms5BarLinkageSpec Resolved;

	/** Last endpoint target commanded (local x-z, cm). */
	FVector2D LastTarget = FVector2D::ZeroVector;
	bool	  bHasTarget = false;

public:
	/** Endpoint heights (cm) the control surface offers for this linkage. Leave
	 *  zero-width (the default) to derive it from the mechanism: the heights at
	 *  the linkage's current X that the IK and the two motors' ControlRanges
	 *  reach (see GetReachableHeightRange). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|5-Bar")
	FVector2D EndpointHeightRange = FVector2D::ZeroVector;

	/** Contiguous height interval (cm) reachable at X, scanned from
	 *  HeightScanMin..HeightScanMax around the current height. bValid is false
	 *  when nothing is reachable there. */
	UFUNCTION(BlueprintPure, Category = "Ramms|5-Bar")
	FVector2D GetReachableHeightRange(float X, bool& bValid) const;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|5-Bar", meta = (ClampMin = "0.05"))
	float HeightScanStep = 0.25f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|5-Bar")
	FVector2D HeightScanLimits = FVector2D(-100.0, 100.0);

	/** Endpoint translations (cm) the control surface offers. Zero-width (the
	 *  default) derives it from the mechanism, as EndpointHeightRange does. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|5-Bar")
	FVector2D EndpointTranslationRange = FVector2D::ZeroVector;

	/** Contiguous fore/aft interval (cm) reachable at height Z. */
	UFUNCTION(BlueprintPure, Category = "Ramms|5-Bar")
	FVector2D GetReachableTranslationRange(float Z, bool& bValid) const;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|5-Bar")
	FVector2D TranslationScanLimits = FVector2D(-100.0, 100.0);

	// --- IRammsControlContributor --------------------------------------------
	// "linkage.<name>.height" and "linkage.<name>.translation": the endpoint's
	// two degrees of freedom, both driven by the same pair of proximal motors.
	virtual void  DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool  ApplyControl(FName Id, float Value) override;
	virtual bool  ReleaseControl(FName Id) override;
	virtual bool  ReadControl(FName Id, float& OutValue) const override;
	virtual bool  ReadTarget(FName Id, float& OutTarget) const override;
	virtual void  GetClaimedMotorIds(TArray<FName>& OutIds) const override;
	virtual int32 GetControlOrder() const override { return 10; }

private:
	FName HeightControlId() const { return RammsControlIds::Linkage::Height(GetName()); }
	FName TranslationControlId() const { return RammsControlIds::Linkage::Translation(GetName()); }

	/** The X SetEndpointHeight keeps: the last target's, else the live endpoint's. */
	float HeldX() const;

	/** The height SetEndpointTranslation keeps, by the same rule. */
	float HeldZ() const;

	/** Walk out from Start while Reachable holds, bounded by Lo..Hi. Both
	 *  reachable ranges are the same scan along different axes. */
	FVector2D ScanReachable(TFunctionRef<bool(float)> Reachable, float Start, float Lo,
		float Hi, float Step, bool& bValid) const;

	/** Build one endpoint axis; Suffix picks height or translation. */
	void DescribeEndpointAxis(FRammsControlSurface& OutSurface, bool bHeight) const;
};
