// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Ramms5BarLinkageSpec.h"
#include "RammsControlContributor.h"
#include "RammsCurbNegotiationTypes.h"
#include "RammsSeatMechanismController.generated.h"

class UDataTable;
class URammsRobotBaseComponent;

/**
 * A seat on a planar 5-bar with a tilt motor under its platform: the seat
 * placed by its platform's pivot in the chassis' x-z plane, and the platform's
 * tilt set on its own and held while the seat moves.
 *
 * The platform rides on arm A's distal link (its pivot PivotBeyondEndpointCm
 * past the 5-bar's endpoint along that link) and so turns with it as the
 * 5-bar moves; the tilt motor at the pivot turns it back. While this component
 * holds the seat it sets the tilt motor every frame from the link's measured
 * angle, so the platform keeps the tilt asked of it relative to the chassis
 * as the seat travels and however the loaded 5-bar sags; and it walks the
 * pivot to a new place at PivotSpeedCmPerSecond, solving all three motors at
 * each step, rather than handing each motor its end angle to reach at its own
 * rate (which pitched the platform tens of degrees on the way). The tilt is
 * changed only through the tilt control (or SetTiltDeg).
 *
 * The kinematics are the curb controller's (an FRammsCurbSeat built from
 * these fields), so the two agree on where the seat is. While a sibling
 * URammsCurbNegotiationController is crossing, it drives the seat and this
 * component refuses commands; afterwards it carries on from wherever the
 * crossing (or a drive-mode stance, or anyone else) left the seat, read from
 * the motors' last commands rather than their sagging joints. Anyone else
 * commanding the 5-bar ends the hold until the next command here.
 *
 * Control surface (group "Seat"): "seat.height" / "seat.fore_aft" (cm, the
 * pivot; paired, with the reachable region's outline, so the panel draws a
 * pad), "seat.jog_up" / "seat.jog_fore_aft" (a rate stick), "seat.reset"
 * (back to RestPivot and RestTiltDeg) and "seat.tilt" (deg, the platform's
 * pitch relative to the chassis, nose up positive).
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsSeatMechanismController : public UActorComponent, public IRammsControlContributor
{
	GENERATED_BODY()

public:
	URammsSeatMechanismController();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// --- Mechanism -----------------------------------------------------------

	/** Optional kinematic data table (row struct: FRamms5BarLinkageSpec); with
	 *  LinkageRow set, its row overrides the inline Linkage. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat")
	TObjectPtr<UDataTable> KinematicTable;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat")
	FName LinkageRow;

	/** The seat's 5-bar, in the chassis' x-z plane (cm, x forward), used when
	 *  no KinematicTable row resolves. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat")
	FRamms5BarLinkageSpec Linkage;

	/** How far past the 5-bar's endpoint, along arm A's distal link, the
	 *  platform's pivot sits (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat")
	float PivotBeyondEndpointCm = 0.0f;

	/** The tilt motor at the platform's pivot (None: the platform turns with
	 *  the link and the tilt control is not offered). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat")
	FName TiltMotor;

	/** The tilt motor's sense: +1 when a positive angle pitches the platform
	 *  nose up relative to arm A's distal link (FRammsCurbSeat::TiltAngleSign). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
	float TiltAngleSign = 1.0f;

	/** How far the tilt control pitches the platform either way from level
	 *  (deg, relative to the chassis). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat", meta = (ClampMin = "0.0", ClampMax = "89.0"))
	float MaxTiltDeg = 50.0f;

	// --- Rest and jog --------------------------------------------------------

	/** Where `seat.reset` puts the platform's pivot (chassis x-z, cm). The
	 *  reset is offered only when the mechanism reaches it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat")
	FVector2D RestPivot = FVector2D::ZeroVector;

	/** The platform's tilt `seat.reset` restores (deg, relative to the
	 *  chassis, nose up positive). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat")
	float RestTiltDeg = 0.0f;

	/** How fast the jog stick moves the pivot at full deflection (cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat", meta = (ClampMin = "0.0"))
	float JogRateCmPerSecond = 6.0f;

	/** Below this deflection a jog axis counts as released (lifting the
	 *  suppression a reset puts on it). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float JogNeutralThreshold = 0.05f;

	/** How fast the platform's pivot travels to a new place (cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat", meta = (ClampMin = "0.1"))
	float PivotSpeedCmPerSecond = 10.0f;

	/** How fast the tilt control turns the platform (deg/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat", meta = (ClampMin = "0.1"))
	float TiltSpeedDegPerSecond = 20.0f;

	/** Time constant (s) of the filter on the link angle the tilt is set from. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat", meta = (ClampMin = "0.0"))
	float LinkFilterSeconds = 0.05f;

	/** Samples per motor across its range when measuring the reachable region
	 *  (once; the spec and the motors' ranges do not change in play). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Seat", meta = (ClampMin = "8", ClampMax = "400"))
	int32 RegionSamples = 120;

	// --- Command / query -----------------------------------------------------

	/** Walk the platform's pivot to TargetXZ (chassis x-z, cm), holding the
	 *  platform's tilt. False (nothing changed) when it is out of reach, past
	 *  a motor's range, or the seat is busy. The walk stops short where the
	 *  straight way there leaves the mechanism's reach. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Seat")
	bool SetPivotTarget(FVector2D TargetXZ);

	/** Turn the platform to Deg relative to the chassis (nose up positive;
	 *  clamped to MaxTiltDeg) at TiltSpeedDegPerSecond, the pivot where it is. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Seat")
	bool SetTiltDeg(float Deg);

	/** RestPivot and RestTiltDeg. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Seat")
	bool ResetSeat();

	/** The platform's pivot where the motors stand (chassis x-z, cm). */
	UFUNCTION(BlueprintPure, Category = "Ramms|Seat")
	FVector2D GetPivot(bool& bValid) const;

	/** The platform's pitch relative to the chassis where the motors stand (deg). */
	UFUNCTION(BlueprintPure, Category = "Ramms|Seat")
	float GetTiltDeg(bool& bValid) const;

	/** The reachable region of the pivot, a closed polygon in (fore/aft,
	 *  height) cm; empty (bValid false) when nothing is reachable. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Seat")
	TArray<FVector2D> GetReachableOutline(bool& bValid) const;

	// --- IRammsControlContributor --------------------------------------------
	virtual void  DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool  ApplyControl(FName Id, float Value) override;
	virtual bool  TriggerControl(FName Id) override;
	virtual bool  ReleaseControl(FName Id) override;
	virtual bool  ReadControl(FName Id, float& OutValue) const override;
	virtual bool  ReadTarget(FName Id, float& OutTarget) const override;
	virtual void  GetClaimedMotorIds(TArray<FName>& OutIds) const override;
	virtual int32 GetControlOrder() const override { return 11; }
	virtual void  SetContributionSuspended(bool bInSuspended) override;
	virtual bool  IsContributionSuspended() const override { return bSuspended; }
	virtual bool  CanSuspendContribution() const override { return true; }

	static FName HeightControlId() { return FName(TEXT("seat.height")); }
	static FName ForeAftControlId() { return FName(TEXT("seat.fore_aft")); }
	static FName JogUpControlId() { return FName(TEXT("seat.jog_up")); }
	static FName JogForeAftControlId() { return FName(TEXT("seat.jog_fore_aft")); }
	static FName ResetControlId() { return FName(TEXT("seat.reset")); }
	static FName TiltControlId() { return FName(TEXT("seat.tilt")); }
	static FName Group() { return FName(TEXT("Seat")); }

private:
	URammsRobotBaseComponent* EnsureBase() const;

	/** The mechanism resolved (table row or inline) and its zero pose closes. */
	bool IsResolved() const { return bResolved; }

	/** A sibling curb controller is crossing: it drives the seat. */
	bool IsBusy() const;

	/** A motor's last command, else where it stands. */
	double CommandOf(FName MotorId) const;

	/** The 5-bar motors' last commands (A, B), else where they stand. */
	FVector2D CommandedAngles() const;

	/** The pivot and the platform's pitch (rad, relative to the chassis) the
	 *  motors were last commanded to -- what this component holds and moves
	 *  from, whoever commanded it. */
	bool CommandedPose(FVector2D& OutPivot, double& OutPitchRad) const;

	/** Within the motor's authored range (an unset range always is). */
	bool WithinRange(FName MotorId, double Angle) const;

	/** The 5-bar's angles for Target, from Hint (else the zero pose), within
	 *  the motors' ranges. */
	bool Reachable(FVector2D Target, FVector2D Hint, FVector2D& OutAngles) const;

	/** Still the one commanding the 5-bar: its commands are the ones given here. */
	bool OwnsSeat() const;

	/** Hold the seat from where it was last commanded, by anyone. False when
	 *  it cannot be held (unresolved, suspended, busy). */
	bool TakeOver();

	/** One frame of the hold: the jog, the pivot's walk, the tilt's turn, the
	 *  tilt motor set from the link's measured angle. */
	void StepHold(float DeltaTime);

	/** The reachable region, measured once: per 1 cm of height, the fore/aft
	 *  interval the pivot reaches there. */
	bool MeasureRegion() const;

	UPROPERTY(Transient)
	mutable TObjectPtr<URammsRobotBaseComponent> BaseComponent = nullptr;

	/** The mechanism as the curb's seat kinematics take it. */
	FRammsCurbSeat Mechanism;
	/** Arm A's distal link angle at the zero pose, where the platform is level. */
	double LinkAngle0 = 0.0;
	bool   bResolved = false;

	mutable bool			  bRegionMeasured = false;
	mutable TArray<FVector2D> CachedOutline;
	mutable FVector2D		  ExtentX = FVector2D(1.0, 0.0);
	mutable FVector2D		  ExtentZ = FVector2D(1.0, 0.0);

	/** Holding the seat: walking it to its goals and setting the tilt. */
	bool bHolding = false;
	/** The 5-bar angles last commanded here (A, B). */
	FVector2D HeldAngles = FVector2D::ZeroVector;
	/** Where the pivot has been walked to, and where it is going. */
	FVector2D PivotNow = FVector2D::ZeroVector;
	FVector2D PivotGoal = FVector2D::ZeroVector;
	/** The platform's pitch relative to the chassis (rad) now and asked for. */
	double TiltNowRad = 0.0;
	double TiltGoalRad = 0.0;
	/** Arm A's distal link angle, measured and filtered. */
	double FilteredLinkAngle = 0.0;
	bool   bLinkFiltered = false;

	/** Jog deflection, -1..1 (X fore/aft, Y up), held until changed or released. */
	FVector2D Jog = FVector2D::ZeroVector;
	bool	  bSuppressJogX = false;
	bool	  bSuppressJogY = false;

	bool bSuspended = false;
};
