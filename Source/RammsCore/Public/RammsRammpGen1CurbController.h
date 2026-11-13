// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "RammsControlContributor.h"
#include "RammsCurbNegotiationTypes.h"
#include "RammsDriveMode.h"
#include "Templates/PimplPtr.h"
#include "RammsRammpGen1CurbController.generated.h"

class URammsRobotBaseComponent;
class USceneComponent;
namespace RammsCurb
{
	class FStickGate;
	class FAssistWatch;
} // namespace RammsCurb

/** Where a RAMMP Gen1 curb crossing is (curb.phase). */
UENUM(BlueprintType)
enum class ERammsRammpGen1CurbPhase : uint8
{
	Idle	  UMETA(DisplayName = "Idle"),
	Level	  UMETA(DisplayName = "Chassis to crossing height"),
	Approach  UMETA(DisplayName = "Approach"),
	FrontUp	  UMETA(DisplayName = "Front omni wheels up"),
	ToEdge	  UMETA(DisplayName = "Drive to the edge"),
	FrontDown UMETA(DisplayName = "Front omni wheels down"),
	DriveUp	  UMETA(DisplayName = "Drive wheels up"),
	DriveOver UMETA(DisplayName = "Drive wheels over the edge"),
	DriveDown UMETA(DisplayName = "Drive wheels down"),
	PullAft	  UMETA(DisplayName = "Drive wheels aft"),
	RearUp	  UMETA(DisplayName = "Rear omni wheels up"),
	RearOver  UMETA(DisplayName = "Rear omni wheels over the edge"),
	Lower	  UMETA(DisplayName = "Chassis down"),
	RearDown  UMETA(DisplayName = "Rear omni wheels down"),
	ClearEdge UMETA(DisplayName = "Clear the edge"),
	Recentre  UMETA(DisplayName = "Translators back"),
	Restore	  UMETA(DisplayName = "Back to the starting pose"),
	Done	  UMETA(DisplayName = "Done"),
	Aborted	  UMETA(DisplayName = "Aborted"),
};

/** How a RAMMP Gen1 crossing moves (curb.style). */
UENUM(BlueprintType)
enum class ERammsRammpGen1CurbStyle : uint8
{
	/** It stops for each step: each pair is lifted, carried over and set down in turn, every touchdown found in small steps. */
	Stepped UMETA(DisplayName = "Stop and step"),
	/** It rolls through the crossing, moving each pair as it reaches the edge, as the real robot's own sequence does.
	 *  Needs the powered front omni wheels (they keep it rolling while the drive wheels are up); without them it steps. */
	Blended UMETA(DisplayName = "Rolling"),
};

/** One support's height table (see RammsRammpGen1Curb::FSupportTable): its wheels' bottom height (cm, chassis
 *  level) from the reference pose as its joint turns (rad), unloaded. Written from the MJCF. */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsRammpGen1SupportTable
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	TArray<float> AngleRad;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	TArray<float> HeightCm;
};

/** The RAMMP Gen1's geometry, motors and crossing settings. ramms-sim's Scripts/rammp_gen1_mujoco/make_rammp_gen1_pawn.py
 *  writes them from mujoco/rammp_gen1 (export_curb_config.py); the defaults are its names. */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsRammpGen1CurbConfig
{
	GENERATED_BODY()

	// --- motors (robot base registry ids); lifts in the base's units: rad, cm for the translators
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Motors")
	FName DriveLeft = "drive_l";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Motors")
	FName DriveRight = "drive_r";
	/** The front omni wheels' hub motors (powered-front variant); None when they are passive. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Motors")
	FName FrontOmniLeft = "front_omni_l";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Motors")
	FName FrontOmniRight = "front_omni_r";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Motors")
	FName ElevatorLeft = "elevator_l";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Motors")
	FName ElevatorRight = "elevator_r";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Motors")
	FName TranslatorLeft = "translator_l";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Motors")
	FName TranslatorRight = "translator_r";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Motors")
	FName FrontArm = "front_arm";
	/** The rear support's driven joint (on the RAMMP Gen1, the suspension arm the rear actuator turns). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Motors")
	FName RearLift = "rear_lift";

	// --- the bodies (the owner's scene components by name) whose positions are measured: wheel hubs
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Bodies")
	FName ChassisBody = "chassis";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Bodies")
	FName DriveWheelLeftBody = "drive_wheel_l";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Bodies")
	FName DriveWheelRightBody = "drive_wheel_r";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Bodies")
	FName FrontOmniLeftBody = "front_omni_l";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Bodies")
	FName FrontOmniRightBody = "front_omni_r";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Bodies")
	FName RearOmniLeftBody = "rear_omni_l";
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Bodies")
	FName RearOmniRightBody = "rear_omni_r";

	// --- geometry
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry", meta = (ClampMin = "1.0"))
	float DriveWheelRadiusCm = 18.4f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry", meta = (ClampMin = "1.0"))
	float FrontOmniRadiusCm = 7.66f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry", meta = (ClampMin = "1.0"))
	float RearOmniRadiusCm = 10.11f;
	/** The frame's lowest point above the ground at the reference pose. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry", meta = (ClampMin = "0.0"))
	float BellyClearanceCm = 5.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry")
	FRammsRammpGen1SupportTable DriveTable;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry")
	FRammsRammpGen1SupportTable FrontTable;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry")
	FRammsRammpGen1SupportTable RearTable;

	// --- the mass model for the rear-lift rule: the chassis-fixed mass (rider included) and its centre of
	// mass ahead of the chassis origin, and each drive assembly, which rides its translator
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Mass", meta = (ClampMin = "0.0"))
	float FixedMassKg = 133.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Mass")
	float FixedComAheadCm = 30.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Mass", meta = (ClampMin = "0.0"))
	float DriveAssemblyMassKg = 14.5f;
	/** A drive assembly's centre of mass ahead of its wheel's hub (cm; negative behind), at the reference pose. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Mass")
	float DriveAssemblyComFromHubCm = 0.0f;

	// --- the actuators' speeds (deg/s, cm/s), as the robot moves them
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Speeds", meta = (ClampMin = "0.1"))
	float ElevatorSpeedDegS = 6.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Speeds", meta = (ClampMin = "0.1"))
	float FrontArmSpeedDegS = 6.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Speeds", meta = (ClampMin = "0.1"))
	float RearLiftSpeedDegS = 8.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Speeds", meta = (ClampMin = "0.1"))
	float TranslatorSpeedCmS = 2.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Speeds", meta = (ClampMin = "1.0"))
	float ApproachSpeedCmS = 25.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Speeds", meta = (ClampMin = "1.0"))
	float CreepSpeedCmS = 15.0f;

	// --- the crossing
	/** A lifted support's clearance over the level it crosses. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Crossing", meta = (ClampMin = "0.0"))
	float ClearanceCm = 3.0f;
	/** The frame's clearance over the upper level. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Crossing", meta = (ClampMin = "0.0"))
	float FrameMarginCm = 2.0f;
	/** Where the translators go for the rear lift (cm, negative is aft). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Crossing")
	float AftCm = -30.0f;
	/** Descents: the lifted drive wheels' centres this far past their radius beyond the edge before they
	 *  swing down, so the swing arms and motors trailing them clear it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Crossing", meta = (ClampMin = "0.0"))
	float LandMarginCm = 12.0f;
	/** The rule: the centre of mass at least this far ahead of the drive axle before the rear lifts. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Crossing", meta = (ClampMin = "0.0"))
	float RuleComMarginCm = 3.0f;
	/** A wheel is down when its bottom is within this of the surface under it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Crossing", meta = (ClampMin = "0.1"))
	float TouchToleranceCm = 0.8f;
	/** How far a touchdown goes on past contact, so the support takes load. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Crossing", meta = (ClampMin = "0.0"))
	float TouchPreloadCm = 1.0f;
	/** Steps smaller than this are driven over; larger than this are refused. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Crossing", meta = (ClampMin = "0.5"))
	float MinStepCm = 3.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Crossing", meta = (ClampMin = "1.0"))
	float MaxStepCm = 17.0f;
	/** How far ahead of the front omni wheels the curb is looked for. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Crossing", meta = (ClampMin = "10.0"))
	float ScanAheadCm = 150.0f;
	/** The most the curb's edge may be off square to the heading (deg). Each pair of wheels goes over it together, so
	 *  the further off square, the further the leading wheel of a pair has gone before the other reaches it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Crossing", meta = (ClampMin = "0.0", ClampMax = "45.0"))
	float MaxApproachDeg = 10.0f;
	/** The sprung rear's lift is trimmed against the chassis's pitch while its wheels are down (rad/s per
	 *  rad of pitch), no faster than RearLiftSpeedDegS. It has to level the chassis while the translators
	 *  re-centre after a climb, or the rear sags as the drive wheels move forward and the frame meets the curb. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Crossing", meta = (ClampMin = "0.0"))
	float LevelGain = 8.0f;

	// --- the rolling crossing (Style Blended): the real robot's own sequence, from videos of it crossing a 15 cm curb,
	// keeps moving at about the translators' speed and takes each pair over in 2-2.5 s
	/** Its speed through the crossing (cm/s); ApproachSpeedCmS until the front omni wheels are BlendNearCm from the edge. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Blended", meta = (ClampMin = "1.0"))
	float CrawlSpeedCmS = 12.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Blended", meta = (ClampMin = "0.0"))
	float BlendNearCm = 30.0f;
	/** How fast its speed changes (cm/s^2). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Blended", meta = (ClampMin = "1.0"))
	float BlendAccelCmS2 = 50.0f;
	/** Held short of the edge for a pair not up yet: cm/s per cm left to where it must stop. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Blended", meta = (ClampMin = "0.1"))
	float BlendStopGain = 2.0f;
	/** A lifted pair is ready for the edge once its lower wheel is this far above the upper level. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Blended", meta = (ClampMin = "0.0"))
	float BlendClearCm = 1.0f;
	/** Climbs: the drive wheels start up when they are this far from the curb's face. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Blended", meta = (ClampMin = "0.0"))
	float BlendLeadUpCm = 5.0f;
	/** Descents: the drive wheels start up off the upper level when their hubs are this far from the edge. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Blended", meta = (ClampMin = "0.0"))
	float BlendLeadDownCm = 10.0f;
};

/**
 * The RAMMP Gen1 curb controller (URammsCurbNegotiationController is Gen2's): a stop-and-step sequence
 * with a level chassis, the reference being ramms-sim's mujoco/rammp_gen1/curb_sequence.py. The chassis rides just under the curb's height (from every support's reach)
 * so its frame clears the edge; each pair of supports -- front omni wheels, drive wheels, rear omni wheels --
 * is lifted over the edge and set down in turn, every touchdown lowered from the wheels' measured height to
 * contact and a little past. The rule: the rear omni wheels are not lifted until both drive wheels are down
 * on the far level, translated aft, with the centre of mass ahead of their axle; the crossing aborts if it
 * does not hold. With hub motors on the front omni wheels, they pull the robot forward while the drive wheels
 * are up (and the translators bring them aft); without, the translators carry the drive wheels over and then,
 * braked, pull the chassis over them. The sprung rear's lift is trimmed against the chassis's pitch.
 * Front omni wheels that start off the ground (the outdoor stance) stay up, at least as high as the crossing needs,
 * until they go down over the edge. A finished crossing puts every lift back where it found it (each lift's last
 * command), so the robot ends in the stance it started in.
 *
 * Style Blended (the default; curb.style) is how the real robot's own sequence crosses, and the reference's
 * --sequence blended: the same order and rule, but it keeps rolling at Config.CrawlSpeedCmS and moves each pair as it
 * reaches the edge -- held short of it, slowing to a stop, only while a pair reaching it is not up yet. Each
 * touchdown is one move, to contact and the preload. The drive wheels go over with the translators where they are and
 * are drawn aft once down, as it rolls on to the rear's step; the translators re-centre as it rolls on after it. The
 * phase (curb.phase) is the step begun last: pairs overlap. It needs the powered front omni wheels, which keep it
 * rolling while the drive wheels are up; without them a crossing steps.
 *
 * The rider's stick drives a crossing (Stick): it goes on only while drive.forward is held, pauses when it is let go,
 * and before any wheel has lifted for the curb, held back, cancels it. Curb assist (Assist, curb.assist) starts one when
 * the rider drives forward at a curb it can cross: within Config.MaxApproachDeg of square, the edge within
 * Assist.TriggerDistanceCm.
 *
 * Controls: curb.negotiate (start, at a curb ahead within ScanAheadCm), curb.abort, curb.phase, curb.assist (Off/On;
 * a button on it flips it), curb.style (Stop and step / Rolling: the next crossing's), and while a crossing has the
 * robot drive.forward / turn / strafe (the stick). It is the
 * "curb" drive mode: starting selects it (through the owner's URammsDriveModeSelector when it has one), and a
 * finished or stopped crossing hands back to ReturnModeId once both drive wheels are on the ground. Stopped
 * with them up, nothing in that mode would hold the robot, so it holds the wheels (the powered front omni
 * wheels' hubs at rest) and leaves the lifts to the operator until they are down. Selected by hand (a panel,
 * the mode button) it starts the crossing ahead, as curb.negotiate does; with none to run it says why (the
 * status) and hands straight back. As a selector's initial mode it starts nothing and hands back. Another mode
 * being selected aborts it. The rear-lift rule's mass model has the robot's arm stowed: where the control
 * surface has one, a crossing stows it (arm.stow) and holds its arm.* rate axes at rest until it ends.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsRammpGen1CurbController : public UActorComponent, public IRammsControlContributor, public IRammsDriveMode
{
	GENERATED_BODY()

public:
	URammsRammpGen1CurbController();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FRammsRammpGen1CurbConfig Config;

	/** The drive mode a finished crossing hands back to. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FName ReturnModeId = "differential";

	/** The rider's stick in a crossing. With Stick.bRequired (the default) a crossing goes on only while the stick is
	 *  held forward (drive.forward, which the curb mode takes while it has the robot) and pauses -- the lifts held where
	 *  they are, the wheels braked -- when it is let go or pushed any other way. Before any wheel has lifted for the
	 *  curb, pulled back (or, before anything has moved, pushed sideways) it cancels: the lifts go back where the
	 *  crossing found them and the robot is handed back. It goes at the planned speeds: the stick's share is not
	 *  applied. Off, a crossing runs to its end once begun. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FRammsCurbStick Stick;

	/** Curb assist (curb.assist, Off/On): on, driving forward at a curb this controller can cross starts the crossing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FRammsCurbAssist Assist;

	/** How a crossing moves (curb.style): rolling through it as the real robot does, or stopping for each step. A
	 *  crossing under way keeps the style it started with. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	ERammsRammpGen1CurbStyle Style = ERammsRammpGen1CurbStyle::Blended;

	/** Start a crossing of the curb ahead. False, with the reason, when it cannot. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	bool StartCrossing(FString& OutReason);

	/** Stop where it is: the lifts hold, the drive wheels brake. It hands back once the drive wheels are down. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	void Abort(const FString& Reason);

	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	ERammsRammpGen1CurbPhase GetPhase() const { return Phase; }

	/** The last crossing's outcome, or why it stopped. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	FString GetStatus() const { return Status; }

	/** The curb ahead's step (cm, positive up) and the crossing height the last start chose. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	void GetCrossing(float& OutStepCm, float& OutCrossingHeightCm) const
	{
		OutStepCm = StepCm;
		OutCrossingHeightCm = CrossingHeightCm;
	}

	// --- IRammsControlContributor
	virtual void DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool ApplyControl(FName Id, float Value) override;
	virtual bool ReleaseControl(FName Id) override;
	virtual bool TriggerControl(FName Id) override;
	virtual bool ReadControl(FName Id, float& OutValue) const override;
	virtual bool ReadTarget(FName Id, float& OutTarget) const override;
	virtual void GetClaimedMotorIds(TArray<FName>& OutIds) const override;

	// --- IRammsDriveMode
	virtual FName GetDriveModeId() const override { return FName("curb"); }
	virtual FText GetDriveModeDisplayName() const override;
	virtual bool  IsDriveModeActive() const override { return bModeActive; }
	virtual void  SetDriveModeActive(bool bActive) override;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	enum class ESupport : uint8
	{
		Drive,
		Front,
		Rear
	};

	/** One lift motor's command, ramped toward its goal at its speed. */
	struct FLift
	{
		FName Motor;
		float Cmd = 0.0f;
		float Goal = 0.0f;
		float Rate = 0.0f;	  // this move's rate (base units/s)
		float MaxRate = 0.0f; // its actuator's
	};

	enum ELiftIndex : int32
	{
		ElevatorL,
		ElevatorR,
		TranslatorL,
		TranslatorR,
		FrontArmLift,
		RearLiftLift,
		NumLifts
	};

	/** The curb ahead, as a crossing of it would run (PlanAhead), until one is started on it (Commit). */
	struct FPlan
	{
		FVector EdgePoint = FVector::ZeroVector;
		FVector EdgeNormal = FVector::ForwardVector;
		float	EdgeAheadCm = 0.0f;
		float	ApproachDeg = 0.0f;
		float	StepCm = 0.0f;
		float	NearZ = 0.0f;
		float	HeightCm = 0.0f;
	};

	// Starting: what stops a start, the curb ahead, and the start on it.
	bool CanStart(FString& OutReason) const;
	bool PlanAhead(FPlan& OutPlan, FString& OutReason, float NoFurtherCm = TNumericLimits<float>::Max()) const;
	bool Commit(const FPlan& Plan, FString& OutReason);

	// The sequence.
	void Begin();
	void Step(float Dt);
	void EnterPhase(ERammsRammpGen1CurbPhase Next);
	void Finish(bool bAborted, const FString& Why);
	bool SelectMode(FName ModeId);

	// The rider's stick (Stick) and curb assist (Assist).
	bool  StepStick(float Dt, bool& bOutPaused); // false: the rider cancelled the crossing
	void  PauseCrossing();
	void  ResumeCrossing();
	void  CancelByRider(const FString& Why);
	void  StepAssist(float Dt);
	float RiderForward() const; // the drive.forward the rider holds, as the drive mode that has the robot reads it

	// Primitives: lift moves (synchronised ramps), touchdowns, driving.
	void StartMove(const TMap<int32, float>& Goals, float SettleSeconds);
	bool StartHeights(const TArray<TPair<ESupport, float>>& HeightsCm, float SettleSeconds);
	bool Moving() const;
	void StartTouchdown(ESupport Support, float LevelZ);
	bool StepTouchdown(float Dt);
	void DriveAt(float SpeedCmS, bool bDriveWheels, bool bFrontOmni);
	void HoldWheels(bool bFrontHubsToo = false); // the front omni wheels' hubs freewheel unless asked
	void ReleaseWheels();
	void StowArm();				   // arm.stow, and the arm's rate axes gathered for HoldArm
	void HoldArm();				   // those axes at rest under this controller's source: no operator input moves the arm
	bool SwitchedToByHand() const; // activated by a switch from another mode, not as the initial mode
	void StepLifts(float Dt);

	// Sensing.
	USceneComponent*				   Body(FName Name) const;
	FVector							   BodyPos(FName Name) const;
	bool							   SurfaceZ(const FVector& At, float& OutZ) const;
	float							   Along(const FVector& P) const;  // past the edge, square to it (cm, negative before it)
	float							   LaggingAlong(ESupport S) const; // the pair's hub further back
	float							   LeadingAlong(ESupport S) const; // the pair's hub further on
	float							   Radius(ESupport S) const;
	float							   Gap(ESupport S, float LevelZ) const; // the higher wheel's bottom above LevelZ
	float							   HeightNow(ESupport S) const;			// its table height at the joint's angle
	float							   PitchRad() const;					// positive nose up
	float							   ComAheadOfAxleCm() const;
	const FRammsRammpGen1SupportTable& TableOf(ESupport S) const;
	bool							   AngleFor(ESupport S, float HeightCm, float& OutAngle, FString& OutWhy) const;
	TArray<int32>					   LiftsOf(ESupport S) const;
	float							   TranslatorReachCm() const;
	bool							   IsCrossing() const;
	bool							   DriveWheelsLoaded();	   // both down on the ground under them, taking load
	bool							   FrontOffGround() const; // both front omni wheels clear of the ground under them
	void							   StopLevelling();		   // the rear's levelling trim into its lift's command, and off

	/** The rear-lift rule, as measured (logged when it holds); bOutDriveAbove: a drive wheel is above the far level. */
	bool CheckRearLift(FString& OutWhy, bool& bOutDriveAbove);

	// The rolling crossing (Style Blended). Each pair's state, by ESupport.
	enum class EPair : uint8
	{
		Near,	  // on the level it is leaving
		Lifting,  // going up off it
		Up,		  // its lower wheel clear of the upper level
		Lowering, // touching down on the far level
		Pressing, // the drive wheels set down again (drawn aft, they carried less)
		Far		  // down on the far level
	};
	struct FBlendTouch
	{
		bool  bActive = false;
		bool  bPreload = false; // contact made: on past it by the preload
		float LevelZ = 0.0f;
		float PlanCm = 0.0f; // the support's height for contact, as measured when it began
	};
	/** Rolling at SpeedCmS: the drive wheels (unless free) at it plus the translators' speed, the front omni wheels' hubs at it. */
	void RollAt(float SpeedCmS);
	/** The log; Display becomes the phase. */
	void NoteBlended(ERammsRammpGen1CurbPhase Display, const FString& What);

	void  BeginBlended();
	void  StepBlended(float Dt);
	void  StartBlendTouch(ESupport S, float LevelZ);
	bool  StepBlendTouch(ESupport S);				   // true the tick it finishes
	float LowGap(ESupport S, float LevelZ) const;	   // the lower wheel's bottom above LevelZ
	float CmdHeight(ESupport S) const;				   // its table height at the lift's command
	float TimeToTouch(ESupport S, float LevelZ) const; // its lift's time to lower it onto LevelZ at full speed (s)
	void  RearGateBlended(float LiftCm);

	UPROPERTY(Transient)
	TObjectPtr<URammsRobotBaseComponent> Base;

	ERammsRammpGen1CurbPhase Phase = ERammsRammpGen1CurbPhase::Idle;
	FString					 Status;
	bool					 bModeActive = false;
	bool					 bStartPending = false;	 // asked to start; begins once the mode is active
	bool					 bHoldUntilDown = false; // stopped with the drive wheels up: no hand-back until they are down
	bool					 bDriveWheelsUp = false; // lifted by this crossing, not yet touched down again
	bool					 bHoldContact[2] = { false, false };
	float					 HoldContactCm[2] = { 0.0f, 0.0f }; // each drive wheel's table height where it met the ground
	bool					 bHandBackWarned = false;
	bool					 bPowered = false; // the front omni wheels have hub motors
	bool					 bClimb = true;
	float					 StepCm = 0.0f;			  // signed: positive up
	float					 CrossingHeightCm = 0.0f; // above the lower level, from the reference pose
	float					 NearZ = 0.0f, FarZ = 0.0f;
	FVector					 EdgePoint = FVector::ZeroVector;
	FVector					 EdgeNormal = FVector::ForwardVector; // square to the edge, onto the far level (Along measures along it)

	// The stick and curb assist.
	FVector							   StickInput = FVector::ZeroVector; // drive.forward, .turn, .strafe while the curb mode has the robot
	TPimplPtr<RammsCurb::FStickGate>   StickGate;
	TPimplPtr<RammsCurb::FAssistWatch> AssistWatch;
	bool							   bStickPaused = false;
	bool							   bLegsMoved = false;	   // the stick has let the crossing go on at least once
	bool							   bCancelling = false;	   // the rider cancelled: the lifts going back, then the hand-back
	bool							   bAssistStarted = false; // curb assist started this crossing
	bool							   bFrontHubsHeld = false; // HoldWheels has the powered front omni wheels' hubs at rest
	FString							   StickWhy, CancelWhy, CrossingStatus;
	float							   SavedDriveCmS = 0.0f; // the drive a pause stopped, for the resume
	bool							   bSavedDriveWheels = false, bSavedDriveFront = false, bSavedFrontHubs = false;

	FLift Lifts[NumLifts];
	float StartCmd[NumLifts] = {}; // where the crossing found each lift (its last command): a finished crossing returns there
	bool  bFrontUp = false;		   // the front omni wheels started off the ground: they stay up until they go down over the edge
	float SettleLeft = 0.0f;
	float PhaseSeconds = 0.0f;
	bool  bLevel = false; // trim the rear lift against the pitch
	float RearTrimRad = 0.0f;

	// The rolling crossing.
	bool		bBlended = false; // this crossing rolls (Style Blended, the front omni wheels powered)
	EPair		PairState[3] = { EPair::Near, EPair::Near, EPair::Near };
	FBlendTouch BlendTouch[3];
	bool		bRaised = false;	 // the chassis at its crossing height
	bool		bDriveFree = false;	 // the drive wheels freewheel: their arms are lifting them off the ground
	bool		bPressed = false;	 // the drive wheels have been set down again once
	int32		BlendEnd = 0;		 // after the rear is down: 1 translators back, 2 to the stance, 3 stopping
	float		BlendV = 0.0f;		 // the speed it rolls at (cm/s)
	double		BlendFrontAt = -1.0; // when the front omni wheels reached the edge, and the rear's touchdown (s)
	double		BlendRearOn = -1.0;

	// Touchdown.
	ESupport TouchSupport = ESupport::Drive;
	float	 TouchLevelZ = 0.0f;
	int32	 TouchStage = 0; // 0 lowering to contact, 1 preload, 2 settle
	float	 TouchTargetCm = 0.0f;
	float	 TouchPlanCm = 0.0f; // the first target: the search goes no more than 4 cm past it

	// Driving (cm/s along the heading; 0 holds).
	float		  DriveCmS = 0.0f;
	bool		  bDriveWheels = false, bDriveFront = false;
	TArray<FName> Braked;
	TArray<FName> ArmAxes; // the arm's rate axes, held at rest while a crossing runs
	double		  LastSimTime = -1.0;
	float		  PhaseValue = 0.0f; // a phase's own number (a translator goal, a target distance)

	// Body's lookups: it runs many times a tick, on a pawn with hundreds of components.
	mutable TMap<FName, TWeakObjectPtr<USceneComponent>> BodyCache;

	TArray<FString> PhaseLog;
};
