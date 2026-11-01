// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "RammsControlContributor.h"
#include "RammsDriveMode.h"
#include "RammsMebotCurbController.generated.h"

class URammsRobotBaseComponent;
class USceneComponent;

/** Where a MeBot curb crossing is (curb.phase). */
UENUM(BlueprintType)
enum class ERammsMebotCurbPhase : uint8
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
	Recentre  UMETA(DisplayName = "Translators to centre"),
	Restore	  UMETA(DisplayName = "Driving pose"),
	Done	  UMETA(DisplayName = "Done"),
	Aborted	  UMETA(DisplayName = "Aborted"),
};

/** One support's height table (see RammsMebotCurb::FSupportTable): its wheels' bottom height (cm, chassis
 *  level) from the reference pose as its joint turns (rad), unloaded. Written from the MJCF. */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsMebotSupportTable
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	TArray<float> AngleRad;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	TArray<float> HeightCm;
};

/** The MeBot's geometry, motors and crossing settings. ramms-sim's Scripts/mebot_mujoco/make_mebot_pawn.py
 *  writes them from mujoco/mebot (export_curb_config.py); the defaults are its names. */
USTRUCT(BlueprintType)
struct RAMMSCORE_API FRammsMebotCurbConfig
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
	/** The rear support's driven joint (on the MeBot, the suspension arm the rear actuator turns). */
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
	FRammsMebotSupportTable DriveTable;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry")
	FRammsMebotSupportTable FrontTable;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Geometry")
	FRammsMebotSupportTable RearTable;

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
	/** The sprung rear's lift is trimmed against the chassis's pitch while its wheels are down (rad/s per
	 *  rad of pitch). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb|Crossing", meta = (ClampMin = "0.0"))
	float LevelGain = 2.0f;
};

/**
 * The MeBot's curb crossing: a stop-and-step sequence with a level chassis, the reference being ramms-sim's
 * mujoco/mebot/curb_sequence.py. The chassis rides just under the curb's height (from every support's reach)
 * so its frame clears the edge; each pair of supports -- front omni wheels, drive wheels, rear omni wheels --
 * is lifted over the edge and set down in turn, every touchdown lowered from the wheels' measured height to
 * contact and a little past. The rule: the rear omni wheels are not lifted until both drive wheels are down
 * on the far level, translated aft, with the centre of mass ahead of their axle; the crossing aborts if it
 * does not hold. With hub motors on the front omni wheels, they pull the robot forward while the drive wheels
 * are up (and the translators bring them aft); without, the translators carry the drive wheels over and then,
 * braked, pull the chassis over them. The sprung rear's lift is trimmed against the chassis's pitch.
 *
 * Controls: curb.negotiate (start, at a curb ahead within ScanAheadCm), curb.abort, curb.phase. It is the
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
class RAMMSCORE_API URammsMebotCurbController : public UActorComponent, public IRammsControlContributor, public IRammsDriveMode
{
	GENERATED_BODY()

public:
	URammsMebotCurbController();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FRammsMebotCurbConfig Config;

	/** The drive mode a finished crossing hands back to. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curb")
	FName ReturnModeId = "differential";

	/** Start a crossing of the curb ahead. False, with the reason, when it cannot. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	bool StartCrossing(FString& OutReason);

	/** Stop where it is: the lifts hold, the drive wheels brake. It hands back once the drive wheels are down. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Curb")
	void Abort(const FString& Reason);

	UFUNCTION(BlueprintPure, Category = "Ramms|Curb")
	ERammsMebotCurbPhase GetPhase() const { return Phase; }

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
	virtual bool TriggerControl(FName Id) override;
	virtual bool ReadControl(FName Id, float& OutValue) const override;
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

	// The sequence.
	void Begin();
	void Step(float Dt);
	void EnterPhase(ERammsMebotCurbPhase Next);
	void Finish(bool bAborted, const FString& Why);
	bool SelectMode(FName ModeId);

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
	USceneComponent*			   Body(FName Name) const;
	FVector						   BodyPos(FName Name) const;
	bool						   SurfaceZ(const FVector& At, float& OutZ) const;
	float						   Along(const FVector& P) const;  // past the edge along the heading (cm, negative before it)
	float						   SupportAlong(ESupport S) const; // the pair's mean hub
	float						   LaggingAlong(ESupport S) const; // the pair's hub further back
	float						   Radius(ESupport S) const;
	float						   Gap(ESupport S, float LevelZ) const; // the higher wheel's bottom above LevelZ
	float						   HeightNow(ESupport S) const;			// its table height at the joint's angle
	float						   PitchRad() const;					// positive nose up
	float						   ComAheadOfAxleCm() const;
	const FRammsMebotSupportTable& TableOf(ESupport S) const;
	bool						   AngleFor(ESupport S, float HeightCm, float& OutAngle, FString& OutWhy) const;
	TArray<int32>				   LiftsOf(ESupport S) const;
	float						   TranslatorReachCm() const;
	bool						   IsCrossing() const;
	bool						   DriveWheelsLoaded(); // both down on the ground under them, taking load

	UPROPERTY(Transient)
	TObjectPtr<URammsRobotBaseComponent> Base;

	ERammsMebotCurbPhase Phase = ERammsMebotCurbPhase::Idle;
	FString				 Status;
	bool				 bModeActive = false;
	bool				 bStartPending = false;	 // asked to start; begins once the mode is active
	bool				 bHoldUntilDown = false; // stopped with the drive wheels up: no hand-back until they are down
	bool				 bDriveWheelsUp = false; // lifted by this crossing, not yet touched down again
	bool				 bHoldContact[2] = { false, false };
	float				 HoldContactCm[2] = { 0.0f, 0.0f }; // each drive wheel's table height where it met the ground
	bool				 bHandBackWarned = false;
	bool				 bPowered = false; // the front omni wheels have hub motors
	bool				 bClimb = true;
	float				 StepCm = 0.0f;			  // signed: positive up
	float				 CrossingHeightCm = 0.0f; // above the lower level, from the reference pose
	float				 NearZ = 0.0f, FarZ = 0.0f;
	FVector				 EdgePoint = FVector::ZeroVector;
	FVector				 Heading = FVector::ForwardVector;

	FLift Lifts[NumLifts];
	float SettleLeft = 0.0f;
	float PhaseSeconds = 0.0f;
	bool  bLevel = false; // trim the rear lift against the pitch
	float RearTrimRad = 0.0f;

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

	TArray<FString> PhaseLog;
};
