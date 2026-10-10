// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "RammsControlContributor.h"
#include "RammsCurbNegotiationTypes.h"
#include "RammsDriveMode.h"

#include "RammsCurbNegotiationController.generated.h"

class URammsRobotBaseComponent;
class URamms5BarLinkageController;

/**
 * Climbs or descends a curb by coordinating every leg and wheel on a
 * three-axle lift-drive: the corner wheels on crank-driven 4-bars, the centre
 * wheels on 5-bars, and the drive motors on all six.
 *
 * The gait. Two axles carry the robot while the third crosses the step, one
 * axle at a time, front to rear. For each axle:
 *
 *   Prepare  every wheel down; the chassis rises (or sinks) to the height that
 *            lets this axle clear the step while the other two still reach
 *            their ground, and the centre wheels shift fore/aft so the centre
 *            of mass stays between the two axles that will be carrying it;
 *   Lift     this axle comes up clear of the higher surface;
 *   Advance  the robot drives on the other two until it is over its landing
 *            spot -- past the edge when climbing, clear of it when descending;
 *   Plant    it goes down onto the far side and takes load.
 *
 * Then the chassis settles, level and on every wheel, to its starting ride
 * height over the new ground. This is the sequence of the
 * lift-drive curb-climb CAD study, generalised: the chassis heights and centre
 * shifts are not fixed poses but are planned from the step height and what
 * each leg can actually reach (FRammsCurbTuning, the 4-bar specs and the
 * 5-bar controllers' own reachability), so the same controller takes a 4 in
 * curb or an 8 in one, up or down, on either chassis -- and refuses, before
 * moving, one its legs cannot span.
 *
 * Speed comes from overlapping the steps rather than from moving any one leg
 * fast: the robot keeps rolling while legs reposition and lift, and every
 * stop is a braking curve against a hard limit -- no wheel on the low side
 * reaches the curb face, no loaded wheel on the high side rolls off the edge
 * -- rather than a fixed waypoint. A step ends when the legs are measured
 * where they were sent, not when a timer says so.
 *
 * The robot drives as it does in differential mode, on its main drive wheels
 * (the lift-drive's treaded centre wheels), with each other wheel driving or
 * rolling free by its role (FRammsCurbLeg::Drive): the holonomic chassis' omni
 * wheels take over only while the centre wheels are off the ground -- their
 * own axle's lift, crossing and landing -- and the linkage chassis' powered
 * front wheels help throughout while its rear omni wheels just roll.
 *
 * Progress is measured, not dead-reckoned: each wheel's position comes from
 * the robot base (GetMotorTransform on its drive motor), and the chassis'
 * attitude from the corner crank pivots, which are fixed on the chassis.
 *
 * It is a drive mode ("curb"): while it runs, the other modes stand down and
 * it owns the wheels and cranks. Starting a manoeuvre selects it through the
 * robot's URammsDriveModeSelector and finishing one hands back to the mode
 * that was active, which puts the robot back in that mode's stance. One
 * stopped by Abort, or by a tilt, step-timeout or measurement fault, keeps
 * the robot held in this mode; a retry that succeeds still hands back to the
 * mode from before. A simulation reset faults it too, but hands back as well:
 * the robot is at its start pose, with nothing across a step to hold.
 * Selected by hand with nothing to cross, it hands straight back -- there is
 * nothing to drive in it.
 *
 * Like the other controllers it holds only configuration, and commands motors
 * by Id through the sibling URammsRobotBaseComponent; nothing here is MuJoCo-
 * specific.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsCurbNegotiationController : public UActorComponent, public IRammsControlContributor, public IRammsDriveMode
{
	GENERATED_BODY()

public:
	URammsCurbNegotiationController();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	/** Every leg the gait moves: two per axle. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	TArray<FRammsCurbLeg> Legs;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FRammsCurbTuning Tuning;

	/** Drive mode to hand back to when done. None returns to whichever mode
	 *  was active when the manoeuvre began. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FName ReturnModeId;

	/**
	 * Plan the manoeuvre for this step without moving: true when every leg
	 * can reach what each step of the gait asks of it. OutReason says why
	 * not, or summarises the plan.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	bool CheckFeasibility(FRammsCurbProfile Profile, FString& OutReason);

	/** Plan and start. False, with a reason, when the step cannot be crossed
	 *  from here; nothing moves then. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	bool BeginNegotiation(FRammsCurbProfile Profile, FString& OutReason);

	/**
	 * Look for a step ahead by tracing the world below the robot's path, and
	 * report it as a profile. False when there is none within
	 * Tuning.DetectRange, or when what is there is not a step this will try.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	bool DetectStepAhead(FRammsCurbProfile& OutProfile, FString& OutReason) const;

	/** DetectStepAhead, then BeginNegotiation on what it found. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	bool NegotiateStepAhead(FString& OutReason);

	/** Stop driving and hold the legs where they are. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	void Abort();

	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	ERammsCurbPhase GetPhase() const { return Phase; }

	/** The axle the current step is about (meaningful for Prepare / Lift /
	 *  Advance / Plant). */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	ERammsCurbAxle GetPhaseAxle() const;

	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	bool IsNegotiating() const;

	/** One line: where it is, or why it stopped. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	FString GetStatusText() const { return Status; }

	/** Seconds since BeginNegotiation (frozen once it ends). */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	float GetElapsedSeconds() const { return static_cast<float>(Elapsed); }

	/** The profile the current (or last) manoeuvre was planned for. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	FRammsCurbProfile GetActiveProfile() const { return ActiveProfile; }

	/** The planned steps, one line each, and a log of each step as it began. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	TArray<FString> GetPlanDescription() const { return PlanLines; }

	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	TArray<FString> GetStepLog() const { return StepLog; }

	/** Chassis pitch and roll (deg, positive nose up / +Y side up -- the side
	 *  of the legs whose Wheel.Position.Y is positive) as last measured. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	FVector2D GetMeasuredTiltDeg() const { return FVector2D(MeasuredPitchDeg, MeasuredRollDeg); }

	/** Chassis origin height above the ground the manoeuvre started on (cm),
	 *  as last measured; and the height the plan is commanding. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	FVector2D GetBaseHeight() const { return FVector2D(MeasuredBaseHeight, BaseHeightCmd); }

	/** Each leg's wheel position along the path relative to the step edge
	 *  (cm, positive past it), in Legs order, as last measured -- it keeps the
	 *  last crossing's values once that ends. Empty until a plan has been
	 *  built. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	TArray<float> GetWheelProgress() const;

	/** Each leg's wheel-centre height in the chassis frame (cm), in Legs
	 *  order: X commanded, Y measured. Empty when never planned. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	TArray<FVector2D> GetLegHeights() const;

	/** Each wheel's bottom above the ground the manoeuvre started on (cm), in
	 *  Legs order, as last measured. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	TArray<float> GetWheelClearance() const;

	// --- IRammsDriveMode ------------------------------------------------------
	virtual FName GetDriveModeId() const override { return FName("curb"); }
	virtual FText GetDriveModeDisplayName() const override
	{
		return NSLOCTEXT("Ramms", "DriveModeCurb", "Curb");
	}
	virtual bool IsDriveModeActive() const override { return bDriveModeActive; }
	virtual void SetDriveModeActive(bool bActive) override;

	// --- IRammsControlContributor --------------------------------------------
	virtual void  DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool  ApplyControl(FName Id, float Value) override { return false; }
	virtual bool  TriggerControl(FName Id) override;
	virtual bool  ReadControl(FName Id, float& OutValue) const override;
	virtual void  GetClaimedMotorIds(TArray<FName>& OutIds) const override;
	virtual int32 GetControlOrder() const override { return 5; }

	/** Control Ids this contributes. */
	static FName NegotiateControlId() { return FName(TEXT("curb.negotiate")); }
	static FName AbortControlId() { return FName(TEXT("curb.abort")); }
	static FName PhaseControlId() { return FName(TEXT("curb.phase")); }

private:
	/** What a leg is asked to do during a step. */
	enum class ELegGoal : uint8
	{
		/** On the ground at a level. */
		Support,
		/** Clear of the ground, above a level. */
		Swing,
		/** Where it was when the manoeuvre began: the robot approaches the
		 *  step in the stance it was driving in. */
		Hold,
	};

	struct FAxleGoal
	{
		ELegGoal Goal = ELegGoal::Support;
		float	 Level = 0.0f;
	};

	/** One step of the plan. */
	struct FStep
	{
		ERammsCurbPhase Phase = ERammsCurbPhase::Idle;
		ERammsCurbAxle	Axle = ERammsCurbAxle::Front;
		FAxleGoal		Axles[3];
		float			BaseHeight = 0.0f;
		/** Fore/aft target (cm) for each 5-bar leg, in Legs order. */
		TArray<float> LegX;
		/** Advance: the axle's wheels end the step this far past the edge. */
		float AdvanceTo = 0.0f;
		/** Approach: the front wheels end the step this far past the edge. */
		float ApproachTo = 0.0f;
		float SpeedFraction = 0.0f;
	};

	/** A leg's live state. */
	struct FLegState
	{
		URamms5BarLinkageController* FiveBar = nullptr;
		FVector2D					 HeightRange = FVector2D::ZeroVector;
		float						 InitialX = 0.0f;
		float						 InitialZ = 0.0f;
		float						 CmdZ = 0.0f;
		float						 CmdX = 0.0f;
		float						 ModelX = 0.0f;
		float						 ModelXVelocity = 0.0f;
		float						 MeasuredX = 0.0f;
		float						 MeasuredZ = 0.0f;
		float						 Progress = 0.0f;
		float						 WheelBottom = 0.0f;
		bool						 bHasWorld = false;
	};

	URammsRobotBaseComponent* EnsureBase() const;
	bool					  ResolveLegs(FString& OutReason);
	bool					  IsPlusY(const FRammsCurbLeg& Leg) const { return Leg.Wheel.Position.Y > 0.0f; }

	/** Build Steps for a profile from the legs' live state. */
	bool BuildPlan(const FRammsCurbProfile& Profile, FString& OutReason);

	/** Leg height (cm) a step asks for at chassis height BaseHeight. */
	float GoalHeight(int32 LegIndex, const FAxleGoal& Goal, float BaseHeight) const;

	/** GoalHeight at the commanded chassis height, as actually sent: lifted
	 *  wheels raised further for the chassis' measured pitch. */
	float CommandedGoalHeight(int32 LegIndex, const FAxleGoal& Goal) const;

	/** Whether one leg can hold a goal at BaseHeight, a 5-bar at fore/aft X. */
	bool LegReachable(int32 LegIndex, const FAxleGoal& Goal, float BaseHeight, float X, FString* OutWhy) const;

	/** Whether every leg can hold a step's goals at BaseHeight, each 5-bar at
	 *  its entry in LegX. */
	bool IsReachable(const FAxleGoal Axles[3], float BaseHeight, const TArray<float>& LegX, float ChassisFloor,
		FString* OutWhy) const;

	/** Pick the chassis height and centre-wheel offsets that hold every one
	 *  of Configs while Axle is lifted, with the centre wheels' mean at least
	 *  MinShift past the centre of mass towards PreferredCenterX. */
	bool PlanLift(ERammsCurbAxle Axle, TArrayView<const FAxleGoal* const> Configs, float ChassisFloor,
		float PreferredHeight, float PreferredCenterX, float MinShift, float& OutHeight, TArray<float>& OutLegX,
		FString& OutReason) const;

	/** Measure the chassis and every wheel; false, with why, when the robot
	 *  cannot be measured: a wheel or pivot with no transform (none yet), or
	 *  joint readings where a 4-bar or 5-bar linkage does not close. */
	bool Measure(FString& OutReason);

	/** World height of the surface under At, tracing down from AboveZ to
	 *  BelowZ past the robot itself; false when nothing is there. */
	bool TraceSurface(const FVector& At, double AboveZ, double BelowZ, double& OutZ) const;

	/** The mode to hand the robot to when there is no better choice: the
	 *  selector's initial mode, or else the first mode that is not this. */
	FName FallbackModeId() const;

	/** The active drive mode when it is not this one, else None. */
	FName OtherActiveModeId() const;

	void  EnterStep(int32 Index);
	void  StepPlan(float DeltaTime);
	bool  LegsSettled() const;
	bool  StepComplete() const;
	float ComputeDriveSpeed(float DeltaTime) const;
	/** Ramp every leg towards the step's goals by at most RampStep's worth;
	 *  DeltaTime is the whole tick, over which the wheels roll with them. */
	void CommandLegs(float RampStep, float DeltaTime);
	/** True while every Main wheel is planted on the surface the current step
	 *  wants it on. */
	bool  MainDriveCarrying() const;
	void  CommandWheels(float Speed, float YawRate);
	void  StopWheels(bool bRelease);
	void  Finish(ERammsCurbPhase EndPhase, const FString& Why);
	void  SetActiveModeThroughSelector(FName ModeId);
	float AxleProgress(ERammsCurbAxle Axle) const;

	UPROPERTY(Transient)
	mutable TObjectPtr<URammsRobotBaseComponent> BaseComponent = nullptr;

	TArray<FLegState> LegState;
	TArray<FStep>	  Steps;
	int32			  StepIndex = INDEX_NONE;
	double			  StepElapsed = 0.0;
	double			  Elapsed = 0.0;

	ERammsCurbPhase	  Phase = ERammsCurbPhase::Idle;
	FRammsCurbProfile ActiveProfile;
	FString			  Status = TEXT("Idle");
	TArray<FString>	  PlanLines;
	TArray<FString>	  StepLog;

	/** The step edge: a point on it and the horizontal heading it is crossed
	 *  along (world). */
	FVector EdgePoint = FVector::ZeroVector;
	FVector Heading = FVector::ForwardVector;
	/** World height of the ground the manoeuvre started on. */
	double GroundZ = 0.0;

	/** The physics clock at the last tick, or negative when the backend keeps
	 *  none; every rate and timeout here is in simulated seconds. */
	double LastSimTime = -1.0;
	/** Pitch, low-passed, for keeping lifted wheels clear on a tilted chassis. */
	float FilteredPitchRad = 0.0f;

	float BaseHeightCmd = 0.0f;
	float InitialBaseHeight = 0.0f;
	float DriveSpeed = 0.0f;
	float MeasuredBaseHeight = 0.0f;
	float MeasuredPitchDeg = 0.0f;
	float MeasuredRollDeg = 0.0f;
	/** Heading error, robot frame, positive when the path lies towards +Y. */
	float HeadingErrorRad = 0.0f;

	/** The mode that had the robot before this one; kept across retries. */
	FName PreviousModeId;
	bool  bDriveModeActive = false;
	bool  bWheelsDriven = false;
	/** Selected without a manoeuvre to run (picked from the mode list): hand
	 *  the robot back on the next tick, outside the selector's switch. */
	bool bHandBackPending = false;
};
