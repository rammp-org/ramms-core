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
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

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

	/**
	 * How far this coordinate can travel anywhere in the reachable set, as
	 * opposed to the slice at one pose.
	 *
	 * This is what the control surface advertises. The slice is the honest
	 * answer to "where can the endpoint go from exactly here", but it is a
	 * terrible axis range: the reachable set is a curved region whose fore/aft
	 * width varies about six-fold with height, so near the resting pose the
	 * slice is around a centimetre wide, the surface clamps every command into
	 * it, and the slider cannot be moved. The extent spans the mechanism, and
	 * a combination that is out of reach at the current height is refused by
	 * SetEndpointTarget rather than silently clamped -- which callers already
	 * handle, since they only adopt a target the controller accepts.
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|5-Bar")
	FVector2D GetReachableExtent(bool bHeight, bool& bValid) const;

	/**
	 * The outline of the reachable region, as a closed polygon in (fore/aft,
	 * height) cm.
	 *
	 * The extent above is what a slider can advertise and is deliberately
	 * generous: it is the bounding interval of a curved region, so a good half
	 * of the box it describes with the other axis is not reachable at all. A
	 * pad shows the endpoint's position *inside* the region, which needs the
	 * region.
	 *
	 * Built from the same slice scan the extent uses -- heights sampled across
	 * the reachable span, the fore/aft interval taken at each -- walking up the
	 * near edge and back down the far one. Empty when nothing is reachable, or
	 * when fewer than three points survive: a degenerate polygon is worse than
	 * none, because a renderer would draw a sliver and imply the mechanism is
	 * one.
	 */
	UFUNCTION(BlueprintPure, Category = "Ramms|5-Bar")
	TArray<FVector2D> GetReachableOutline(bool& bValid) const;

	/** Height slices taken when tracing the outline. More is a smoother region
	 *  and a longer scan; each slice is its own reachability scan. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|5-Bar", meta = (ClampMin = "3"))
	int32 OutlineScanSamples = 17;

	/**
	 * Where `linkage.<name>.reset` sends the endpoint (ROBOT-frame x-z, cm).
	 *
	 * Robot frame, not linkage-local, because that is what it is handed to:
	 * SetEndpointTarget and SolveTarget convert through ToLocal like every
	 * other controller-facing coordinate. Calling it local would read correctly
	 * on an unmirrored linkage and silently flip X on a mirrored one -- for the
	 * reachability check and the reset alike.
	 *
	 * Zero is the linkage's own origin, which for these mechanisms is the
	 * neutral pose. It is settable because "rest" is a property of how a
	 * particular linkage is mounted, not of the maths -- and because a reset
	 * that commands somewhere unreachable would simply be refused, which is a
	 * confusing button.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|5-Bar")
	FVector2D RestEndpoint = FVector2D::ZeroVector;

	/** Slices taken across the other axis when computing an extent. More is
	 *  a finer outline of a curved region and a longer scan. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|5-Bar", meta = (ClampMin = "2"))
	int32 ExtentScanSamples = 9;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|5-Bar")
	FVector2D TranslationScanLimits = FVector2D(-100.0, 100.0);

	/** How fast the jog axes move the endpoint at full deflection (cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ramms|5-Bar", meta = (ClampMin = "0.0"))
	float JogRateCmPerSecond = 6.0f;

	// --- IRammsControlContributor --------------------------------------------
	// "linkage.<name>.height" and "linkage.<name>.translation": the endpoint's
	// two degrees of freedom, both driven by the same pair of proximal motors.
	virtual void  DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool  ApplyControl(FName Id, float Value) override;
	virtual bool  TriggerControl(FName Id) override;
	virtual bool  ReleaseControl(FName Id) override;
	virtual bool  ReadControl(FName Id, float& OutValue) const override;
	virtual bool  ReadTarget(FName Id, float& OutTarget) const override;
	virtual void  GetClaimedMotorIds(TArray<FName>& OutIds) const override;
	virtual int32 GetControlOrder() const override { return 10; }
	virtual void  SetContributionSuspended(bool bSuspended) override;
	virtual bool  IsContributionSuspended() const override { return bSuspended; }
	/** Yes: SetContributionSuspended below really does let go of the hips. */
	virtual bool CanSuspendContribution() const override { return true; }

private:
	FName HeightControlId() const { return RammsControlIds::Linkage::Height(GetName()); }
	FName TranslationControlId() const { return RammsControlIds::Linkage::Translation(GetName()); }

	/** -1 when this leg is mounted mirrored; see FRamms5BarLinkageSpec. */
	float Handedness() const;

	/** Robot-frame (x forward) <-> this linkage's own frame. Every coordinate
	 *  in this class's public surface is robot-frame; the kinematics are not. */
	FVector2D ToLocal(FVector2D RobotXZ) const;
	FVector2D ToRobot(FVector2D LocalXZ) const;

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

	FName JogUpControlId() const { return RammsControlIds::Linkage::JogUp(GetName()); }
	FName JogForwardControlId() const { return RammsControlIds::Linkage::JogForward(GetName()); }
	FName ResetControlId() const { return RammsControlIds::Linkage::Reset(GetName()); }

	/** Current jog deflection, -1..1 (X = fore/aft, Y = up). Held until
	 *  changed or released, like a stick that stays where it is put. */
	FVector2D Jog = FVector2D::ZeroVector;

	/** Log the first jog tick, so a stick that does nothing is diagnosable. */
	bool bLoggedJog = false;

	/** Standing down so the low-level mode can drive these motors directly. */
	bool bSuspended = false;
};
