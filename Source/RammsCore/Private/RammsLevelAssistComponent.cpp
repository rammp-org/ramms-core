// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsLevelAssistComponent.h"

#include "GameFramework/Actor.h"
#include "Level/RammsLevelAllocate.h"
#include "Level/RammsLevelAttitude.h"
#include "Level/RammsLevelControls.h"
#include "Level/RammsLevelLaw.h"
#include "Level/RammsLevelPlant.h"
#include "RammpGen1ControllerComponent.h"
#include "Ramms5BarLinkageController.h"
#include "RammsControlIds.h"
#include "RammsControlInputComponent.h"
#include "RammsCurbNegotiationController.h"
#include "RammsDriveMode.h"
#include "RammsDriveModeSelector.h"
#include "RammsRammpGen1CurbController.h"
#include "RammsRobotBaseComponent.h"
#include "RammsRobotControlSurfaceComponent.h"
#include "RammsSeatMechanismController.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsLevel, Log, All);

namespace
{
	// What a robot may carry at speed, and the speed cap's first sign of a break: switches for measuring each
	// against the law without it (RammsLevel::FCarryTuning, FSpeedCapTuning). Read every tick.
	// (Each starts as RammsLevel::FCarryTuning has it: the defaults are kept there.)
	TAutoConsoleVariable<int32> CVarLevelCrestRule(TEXT("Ramms.Level.CrestRule"), RammsLevel::FCarryTuning().bCrest ? 1 : 0,
		TEXT("Level assist: 1 = carry no more correction than the legs can take back out while a grade break passes under them at the speed the robot has (a crest then costs no more than switched off; a slope taken fast is left partly in the base). 0 = level all of every slope at any speed (the default: the rule cannot tell a slope that ends in a crest from one that does not, and on slow legs it costs every fast slope more than it saves at a crest)."));
	TAutoConsoleVariable<int32> CVarLevelThreePointRule(TEXT("Ramms.Level.ThreePointRule"), RammsLevel::FCarryTuning().bThreePoint ? 1 : 0,
		TEXT("Level assist, on a robot whose roll correction stands it on three points: 1 = roll levelled standing and up to a walk, eased out as the speed rises past that, not brought back while the robot is being slowed, and a roll under the dead band not corrected. 0 = roll levelled at any speed."));
	TAutoConsoleVariable<int32> CVarLevelThreePointHoldSpeed(TEXT("Ramms.Level.ThreePointHoldSpeed"), RammsLevel::FCarryTuning().bHoldSpeed ? 1 : 0,
		TEXT("Level assist, with the three-point rule: 1 = the drive is held to the speed the roll still carried allows while that roll is eased out. 0 = the drive is not held."));
	TAutoConsoleVariable<float> CVarLevelThreePointFullCmS(TEXT("Ramms.Level.ThreePointFullCmS"), static_cast<float>(RammsLevel::FCarryTuning().ThreePointFullCmS),
		TEXT("Level assist, three-point rule: up to this speed (cm/s) all of the roll limit may be carried."));
	TAutoConsoleVariable<float> CVarLevelThreePointNoneCmS(TEXT("Ramms.Level.ThreePointNoneCmS"), static_cast<float>(RammsLevel::FCarryTuning().ThreePointNoneCmS),
		TEXT("Level assist, three-point rule: from this speed (cm/s) no roll is carried."));
	TAutoConsoleVariable<int32> CVarLevelEarlyCap(TEXT("Ramms.Level.EarlyCap"), 1,
		TEXT("Level assist: 1 = the speed cap slows at the first sign of a grade break (the ground's reading before the terrain estimate's filter), held through the break. 0 = only the filtered turn, as before."));
} // namespace

namespace RammsLevel
{
	/** The robot moved further than this (cm) between two ticks: it was put
	 *  somewhere else by hand (a reset, a teleport), and what was being
	 *  corrected is gone. */
	constexpr double PoseJumpCm = 50.0;
	/** The rider's stick counts as off centre past this share of its travel. */
	constexpr float StickMovedFraction = 0.05f;
	/** A hand-back's plane is taken out of the legs' commands in this many
	 *  rounds: the second is over the legs as the first left them (a wheel's
	 *  place fore and aft moves with its baseline, by a centimetre on a leg
	 *  that swings), so that the offsets left are the law's own to a hair. */
	constexpr int32 AdoptRounds = 2;

	/** A component ticking in Group may be made a prerequisite of a tick that
	 *  must itself start before physics: one in a later group may not. */
	bool MayTickBefore(ETickingGroup Group)
	{
		return Group <= TG_PrePhysics;
	}

	/** What the level assist keeps between ticks that is not configuration. */
	struct FAssistRuntime
	{
		/** The robot as the law sees it, and where its attitude is read; both
		 *  null on a robot that cannot be levelled. */
		TUniquePtr<IPlant>			Plant;
		TUniquePtr<IAttitudeSource> Attitude;
		/** Set, in plain words, when the robot cannot be levelled at all. */
		FString CannotLevel;
		bool	bResolveTried = false;

		/** The law's state, the tuning it last ran with, and what it last
		 *  answered. */
		FState	 Law;
		FTuning	 LawTuning;
		FOutputs Last;
		/** The legs and belly points of this tick; the legs as last measured
		 *  whole, for a tick that cannot measure them. */
		FPlantLegs	 Legs;
		FPlantLegs	 LastLegs;
		FPlantPoints Points;

		/** level.assist, and the rider's preferred tilt (deg). */
		bool	  bSwitch = false;
		FVector2D PreferenceDeg = FVector2D::ZeroVector;
		/** This component holds the base's command offset layer. */
		bool bClaimed = false;
		/** The drive modes its speed scale went to on the last tick, to take
		 *  it back from the moment levelling stops. */
		TArray<TWeakObjectPtr<UObject>, TInlineAllocator<2>> SlowedModes;
		/** CannotLevel is a claim that was refused (something else held the
		 *  layer): worth trying again when the rider next switches on. */
		bool bClaimRefused = false;
		/** The components made this one's tick prerequisites for as long as
		 *  the layer is held, and whether the base was made to tick after it. */
		TArray<TWeakObjectPtr<UActorComponent>> Prerequisites;
		bool									bBaseOrdered = false;
		/** Said once each: a writer that cannot be ordered before this tick,
		 *  and a tick that did not run in the group it was written for. */
		TSet<FName> LateWritersSaid;
		bool		bGroupSaid = false;

		/** Who was yielded to and has not handed back; whether its drive mode
		 *  has been seen active since; and how many ticks it has had to become
		 *  so. */
		TWeakObjectPtr<const UObject> Taker;
		bool						  bTakerSeenActive = false;
		int32						  TakerTicks = 0;
		/** How many more times that taker has called YieldTo while it already
		 *  held the legs (a crossing tried again from a stopped one): each is
		 *  matched by a refusal's ResumeFrom, which must then change nothing. */
		int32 TakerNested = 0;
		/** The law as it stood before the last YieldTo, for a take-over
		 *  refused before it began; and whether motors were folded for it. */
		TOptional<FState>	PreYieldLaw;
		TOptional<FOutputs> PreYieldLast;
		bool				bFoldedMotors = false;
		/** The chassis' attitude as that YieldTo read it (deg), where it could:
		 *  a hand-back that finds every leg as it was folded, and the base
		 *  turned from this, is on other ground than the take-over began on. */
		TOptional<FVector2D> PreYieldAttitudeDeg;
		/** The seat's trim was folded into the seat's command since that
		 *  YieldTo (YieldSeatTo): the take-over is then past undoing exactly. */
		bool bSeatYielded = false;

		/** The physics clock at the last tick, and time as the ticks have
		 *  counted it (for an attitude reading that does not say when it was
		 *  taken). */
		double LastSimTime = -1.0;
		double TickSeconds = 0.0;
		bool   bColdReset = false;
		/** The chassis' speed over the ground and its turning rate about the
		 *  vertical, from the attitude source's readings on the time each was
		 *  taken at; and whether the last reading said when that was. */
		FMotionSense Motion;
		bool		 bMotionStamped = false;
		/** The last step that took time had a reading of the chassis. */
		bool bReadLastStep = false;
		/** The last hand-back to a stance took a plane back out of the legs'
		 *  commands, and left this much of the pose's plane in them (deg) for
		 *  the stance ramp to turn the base by. Unset: no such hand-back since
		 *  the last tick, or one that took nothing back. */
		TOptional<FVector2D> HandBackLeftDeg;
		/** The active drive mode is one it does not level in; the mode
		 *  selector is changing the stance (both for the status). */
		bool bModeNotLevelled = false;
		bool bStanceChanging = false;
		/** The seat as the last tick found it: whether its tilt can take a
		 *  trim, the pitch its own commands hold it at against the chassis
		 *  (deg), and the pitch of the rider's preference it is asked to hold
		 *  (deg; 0 where the base holds it). */
		bool   bSeat = false;
		double SeatOwnDeg = 0.0;
		double SeatPreferenceDeg = 0.0;
		/** The trim the seat carried as an offset when the last tick read it
		 *  (deg): what is shown while the law holds none of its own (standing
		 *  by for a taker that has the legs and not the seat). */
		double SeatTrimOnDeg = 0.0;
	};
} // namespace RammsLevel

URammsLevelAssistComponent::URammsLevelAssistComponent()
	: Runtime(MakePimpl<RammsLevel::FAssistRuntime>())
{
	PrimaryComponentTick.bCanEverTick = true;
	// The robot base's group, and ahead of it (BeginPlay): the base tells the
	// motors in its own tick and must see this frame's offsets.
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

URammsLevelAssistComponent::~URammsLevelAssistComponent() = default;

URammsRobotBaseComponent* URammsLevelAssistComponent::EnsureBase() const
{
	if (!BaseComponent)
	{
		if (const AActor* Owner = GetOwner())
		{
			BaseComponent = Owner->FindComponentByClass<URammsRobotBaseComponent>();
		}
	}
	return BaseComponent;
}

void URammsLevelAssistComponent::BeginPlay()
{
	Super::BeginPlay();
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	Runtime->bSwitch = bEnabledAtStart;
	// No tick order is asked for here: only while the offset layer is held
	// (ClaimMotors), so that a level assist that is switched off leaves every
	// other component ticking in the order it would with none on the robot.
}

void URammsLevelAssistComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (Runtime->bClaimed)
	{
		if (Runtime->Plant)
		{
			Runtime->Plant->ZeroOffsets();
		}
		ReleaseMotors();
	}
	Super::EndPlay(EndPlayReason);
}

void URammsLevelAssistComponent::AddTickPrerequisites()
{
	RammsLevel::FAssistRuntime& R = *Runtime;
	AActor*						Owner = GetOwner();
	if (!Owner || !R.Plant)
	{
		return;
	}
	URammsRobotBaseComponent* Base = EnsureBase();
	TArray<FName>			  Levelled;
	R.Plant->LevelledMotorIds(Levelled);
	// Everything that writes a leg's baseline ticks first, so that this
	// tick's offsets are worked out from this frame's baselines: the mode
	// selector (stance ramps), every drive mode (the curb controller among
	// them), the 5-bar, seat and Gen1 lift controllers, the player's input,
	// and anything else that says it drives a levelled motor.
	for (UActorComponent* Component : Owner->GetComponents())
	{
		if (!Component || Component == this || Component == Base || !Component->PrimaryComponentTick.bCanEverTick)
		{
			continue;
		}
		bool bDrivesLevelled = false;
		if (const IRammsControlContributor* Contributor = Cast<IRammsControlContributor>(Component))
		{
			TArray<FName> Claimed;
			Contributor->GetClaimedMotorIds(Claimed);
			bDrivesLevelled = Claimed.ContainsByPredicate([&Levelled](const FName& Id) { return Levelled.Contains(Id); });
		}
		const bool bWriter = Component->IsA<URammsDriveModeSelector>() || Cast<IRammsDriveMode>(Component) || Component->IsA<URamms5BarLinkageController>()
			|| Component->IsA<URammsSeatMechanismController>() || Component->IsA<URammpGen1ControllerComponent>()
			|| Component->IsA<URammsControlInputComponent>();
		if (!bWriter && !bDrivesLevelled)
		{
			continue;
		}
		if (!RammsLevel::MayTickBefore(Component->PrimaryComponentTick.TickGroup))
		{
			// A tick function starts in the latest group among itself and
			// its prerequisites: this one as a prerequisite would take the
			// level assist, and the robot base behind it, out of the group
			// before physics, and with them every wheel and leg command of
			// the frame. It is left where it is; what it writes in its own
			// tick while the legs are levelled is told at once with its
			// offset (never bare), on the offset this tick worked out from
			// the baseline before it (the base's rule for a late write).
			if (bDrivesLevelled && !R.LateWritersSaid.Contains(Component->GetFName()))
			{
				R.LateWritersSaid.Add(Component->GetFName());
				UE_LOG(LogRammsLevel, Log, TEXT("[Level] '%s': '%s' ticks after the robot base; while the legs are levelled, a command it writes in its own tick is told at once, on the offset worked out before it"),
					*GetNameSafe(Owner), *Component->GetName());
			}
			continue;
		}
		AddTickPrerequisiteComponent(Component);
		R.Prerequisites.Add(Component);
	}
	// ... and the base after this: it tells each claimed motor its baseline
	// plus its offset once, at the start of its own tick.
	if (Base && RammsLevel::MayTickBefore(PrimaryComponentTick.TickGroup))
	{
		Base->AddTickPrerequisiteComponent(this);
		R.bBaseOrdered = true;
	}
}

void URammsLevelAssistComponent::RemoveTickPrerequisites()
{
	RammsLevel::FAssistRuntime& R = *Runtime;
	for (const TWeakObjectPtr<UActorComponent>& Component : R.Prerequisites)
	{
		if (UActorComponent* Was = Component.Get())
		{
			RemoveTickPrerequisiteComponent(Was);
		}
	}
	R.Prerequisites.Reset();
	if (R.bBaseOrdered)
	{
		if (URammsRobotBaseComponent* Base = EnsureBase())
		{
			Base->RemoveTickPrerequisiteComponent(this);
		}
		R.bBaseOrdered = false;
	}
}

void URammsLevelAssistComponent::ResolveRobot()
{
	RammsLevel::FAssistRuntime& R = *Runtime;
	R.bResolveTried = true;
	AActor*					  Owner = GetOwner();
	URammsRobotBaseComponent* Base = EnsureBase();
	// Which sibling describes the robot picks the kind of plant: the Gen2 curb
	// controller's legs (wheels on crank-driven linkages, the attitude read
	// off the corner pivots), or the Gen1 curb controller's config (two drive
	// wheels and two pairs on lifts, the attitude the chassis body's own).
	const URammsCurbNegotiationController* Curb = Owner ? Owner->FindComponentByClass<URammsCurbNegotiationController>() : nullptr;
	const URammsRammpGen1CurbController*   Gen1Curb = Owner && !Curb ? Owner->FindComponentByClass<URammsRammpGen1CurbController>() : nullptr;
	if (!Base || (!Curb && !Gen1Curb))
	{
		R.CannotLevel = TEXT("no robot description");
	}
	else
	{
		TUniquePtr<RammsLevel::IPlant> Plant = Curb ? RammsLevel::MakeLegPlant(*Base, *Curb, this) : RammsLevel::MakeGen1Plant(*Base, *Gen1Curb, this);
		FString						   Why;
		if (!Plant->Resolve(Why))
		{
			R.CannotLevel = Why;
		}
		else if (AttitudeSource != ERammsLevelAttitudeSource::Exact)
		{
			R.CannotLevel = TEXT("only the exact attitude source is built");
		}
		else
		{
			R.Plant = MoveTemp(Plant);
			if (Curb)
			{
				R.Attitude = MakeUnique<RammsLevel::FPivotAttitudeSource>(*Base, *Curb);
			}
			else
			{
				R.Attitude = MakeUnique<RammsLevel::FBodyAttitudeSource>(*Base, *Gen1Curb);
			}
		}
	}
	if (!R.CannotLevel.IsEmpty())
	{
		R.bSwitch = false;
		UE_LOG(LogRammsLevel, Warning, TEXT("[Level] '%s' cannot be levelled: %s"), *GetNameSafe(Owner), *R.CannotLevel);
	}
}

bool URammsLevelAssistComponent::ClaimMotors()
{
	RammsLevel::FAssistRuntime& R = *Runtime;
	URammsRobotBaseComponent*	Base = EnsureBase();
	if (R.bClaimed || !Base || !R.Plant)
	{
		return R.bClaimed;
	}
	TArray<FName> Ids;
	R.Plant->LevelledMotorIds(Ids);
	R.bClaimed = Base->ClaimCommandOffsetLayer(this, Ids);
	if (R.bClaimed)
	{
		// From now until the layer is given up, this tick's place among the
		// others matters (and takes effect from the next frame: this frame's
		// offsets are exactly zero, so its order does not).
		AddTickPrerequisites();
	}
	return R.bClaimed;
}

void URammsLevelAssistComponent::ReleaseMotors()
{
	if (URammsRobotBaseComponent* Base = EnsureBase())
	{
		Base->ReleaseCommandOffsetLayer(this);
	}
	RemoveTickPrerequisites();
	Runtime->bClaimed = false;
}

void URammsLevelAssistComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	RammsLevel::FAssistRuntime& R = *Runtime;
	if (!R.bResolveTried)
	{
		// Here, not at BeginPlay: the linkage controllers resolve their own
		// specs in theirs, in no promised order.
		ResolveRobot();
	}
	if (!R.Plant)
	{
		return;
	}
	// A hand-back's figures are for the mode switch made in the same call.
	R.HandBackLeftDeg.Reset();
	if (ThisTickFunction && !R.bGroupSaid)
	{
		// Written to run before physics, with the base behind it: a
		// prerequisite in a later group (a tick group saved on a Blueprint's
		// component template) would move both, and every command of the frame
		// with them.
		const URammsRobotBaseComponent* OrderedBase = R.bBaseOrdered ? EnsureBase() : nullptr;
		const ETickingGroup				Mine = ThisTickFunction->GetActualTickGroup();
		const ETickingGroup				Bases = OrderedBase ? OrderedBase->PrimaryComponentTick.GetActualTickGroup() : TEnumAsByte<ETickingGroup>(TG_PrePhysics);
		if (!RammsLevel::MayTickBefore(Mine) || !RammsLevel::MayTickBefore(Bases))
		{
			R.bGroupSaid = true;
			UE_LOG(LogRammsLevel, Warning, TEXT("[Level] '%s': the level assist (tick group %d) or the robot base (%d) does not tick before physics; leg and wheel commands reach the simulator later in the frame than they should"),
				*GetNameSafe(GetOwner()), static_cast<int32>(Mine), static_cast<int32>(Bases));
		}
	}
	// Time is the physics clock's where the backend keeps one: a simulator
	// that cannot keep up falls behind the game's, and the legs move on its
	// time. A clock that ran backwards was reset.
	double							Dt = DeltaTime;
	const URammsRobotBaseComponent* Base = EnsureBase();
	const double					Sim = Base ? Base->GetSimulationTime() : -1.0;
	if (Sim >= 0.0)
	{
		if (R.LastSimTime >= 0.0)
		{
			Dt = Sim - R.LastSimTime;
			if (Dt < 0.0)
			{
				R.bColdReset = true;
				Dt = 0.0;
			}
		}
		R.LastSimTime = Sim;
	}
	else
	{
		R.LastSimTime = -1.0;
	}
	StepLaw(FMath::Clamp(Dt, 0.0, RammsLevel::MaxStepSeconds), false, false);
	// A taker that was yielded to has had this tick to become the active mode.
	if (R.Taker.IsValid())
	{
		++R.TakerTicks;
	}
}

void URammsLevelAssistComponent::StepLaw(double ClockDt, bool bYield, bool bResume, const RammsLevel::FAdoption* Adoption, const UObject* HandingBack)
{
	using namespace RammsLevel;
	FAssistRuntime& R = *Runtime;
	AActor*			Owner = GetOwner();

	FInputs In;
	// --- the chassis, and what time this step is counted on ---
	// The law is stepped through the time between two READINGS of the chassis,
	// where the source says when each was taken: every rate made from them
	// (the chassis' speed, how hard it is braked, how fast it turns, how fast
	// the ground turns under it) and every filter they feed is then on the
	// time they really are apart. That is not the time between two ticks' looks
	// at the clock where the physics runs on its own thread: the pose a tick is
	// given was published up to a frame before, a different while each tick, and
	// a distance between two poses over the time between two ticks is a speed
	// that swings by a quarter from one tick to the next at a steady pace --
	// read as a base being braked hard, which the ground estimate rightly
	// distrusts, and the legs then stop following the ground at all.
	// A source that does not say when reads as of the tick: the clock's time.
	// A step in the middle of a frame (a yield, a hand-back) takes no time.
	double	  Dt = ClockDt;
	FAttitude Attitude;
	In.bAttitudeValid = R.Attitude && R.Attitude->Read(Attitude);
	R.TickSeconds += ClockDt;
	if (In.bAttitudeValid)
	{
		In.AttitudeDeg = Attitude.Deg;
		if (ClockDt > 0.0 || !R.Motion.HasReading())
		{
			const bool bStamped = Attitude.TimeSeconds >= 0.0;
			if (bStamped != R.bMotionStamped)
			{
				// Another clock from here on: the last reading's time is not on it.
				R.Motion.Reset();
				R.bMotionStamped = bStamped;
			}
			const bool	 bHadReading = R.Motion.HasReading();
			const double Since = R.Motion.Tick(Attitude.Position, Attitude.Heading, bStamped ? Attitude.TimeSeconds : R.TickSeconds);
			// Readings whose time ran backwards are of a simulation that was
			// reset; a robot far from where it was, of one put elsewhere.
			R.bColdReset |= R.Motion.RanBackwards() || R.Motion.MovedCm() > PoseJumpCm;
			// (After a step that had no reading the law has already been
			// stepped through the clock's time meanwhile: this one is too.)
			if (bStamped && bHadReading && R.bReadLastStep && ClockDt > 0.0)
			{
				Dt = FMath::Clamp(Since, 0.0, MaxStepSeconds);
			}
		}
	}
	if (ClockDt > 0.0)
	{
		R.bReadLastStep = In.bAttitudeValid;
	}
	In.SpeedCmS = R.Motion.GetSpeedCmS();
	In.YawRateDegPerSec = R.Motion.GetYawRateDegPerSec();
	In.bColdReset = R.bColdReset;
	R.bColdReset = false;

	// --- the legs ---
	const bool bLegs = R.Plant->MeasureLegs(R.Legs);
	if (bLegs)
	{
		R.LastLegs = R.Legs;
	}
	else
	{
		// A leg whose joints do not make a height: the law holds what it has,
		// on the legs as they last read.
		R.Legs = R.LastLegs;
		In.bAttitudeValid = false;
	}
	// --- a taker that was yielded to ---
	// It holds the legs from its YieldTo until it hands back, or until its
	// drive mode, once active, no longer is. (It yields before it selects its
	// mode: it has until this component's next tick.) Looked at before the
	// seat is read: a taker that has gone may have left a trim in the seat's
	// command, which is taken back out here.
	bool bTakerHolds = false;
	if (const UObject* Taker = R.Taker.Get())
	{
		const IRammsDriveMode* TakerMode = Cast<IRammsDriveMode>(const_cast<UObject*>(Taker));
		const bool			   bActive = !TakerMode || TakerMode->IsDriveModeActive();
		R.bTakerSeenActive |= TakerMode && bActive;
		bTakerHolds = bActive || (!R.bTakerSeenActive && R.TakerTicks < 1);
		if (!bTakerHolds)
		{
			// It let go without handing back (a crossing stopped, its mode
			// left by hand): levelling starts again cold from what it left --
			// but for the seat's trim, which goes on from what the seat still
			// carries (one folded for a taker that never wrote the seat's tilt
			// is taken back out first).
			R.Taker.Reset();
			R.TakerNested = 0;
			R.PreYieldLaw.Reset();
			R.PreYieldLast.Reset();
			TakeBackSeatTrim();
			R.Plant->ForgetFold();
		}
	}

	// --- the seat, and where the rider's preference is held ---
	// A seat whose tilt can take a trim holds the pitch the rider prefers; the
	// base then stays on its own reference and keeps its legs' reach for the
	// ground. The roll, the pitch where no seat can hold it, and what of the
	// pitch the seat has no room for, are the base's: part of what level means
	// for it.
	double SeatLowDeg = 0.0;
	double SeatHighDeg = 0.0;
	R.SeatOwnDeg = 0.0;
	R.bSeat = R.Plant->HasSeat() && R.Plant->SeatTrimRange(R.SeatOwnDeg, SeatLowDeg, SeatHighDeg);
	const FPreferenceSplit Preference = SplitPreference(GetPreferenceDeg(), R.bSeat, Tuning.bSeatCoversTransients, SeatLowDeg, SeatHighDeg, Tuning.SeatTrimMaxDeg);
	R.SeatPreferenceDeg = Preference.SeatDeg;
	R.LawTuning = MakeTuning(Tuning, Preference.BaseDeg);
	In.bSeat = R.bSeat;
	In.SeatPreferenceDeg = Preference.SeatDeg;
	In.SeatTrimLowDeg = SeatLowDeg;
	In.SeatTrimHighDeg = SeatHighDeg;
	// (What the seat carries as an offset now: a start goes on from it.)
	R.SeatTrimOnDeg = R.bSeat ? R.Plant->SeatTrimOnDeg() : 0.0;
	In.SeatTrimOnDeg = R.SeatTrimOnDeg;

	R.Plant->ClearancePoints(R.LawTuning, R.Points);
	In.Legs = R.Legs;
	In.Points = R.Points;

	// --- the rider's stick: off centre wakes a robot at rest ---
	if (const URammsRobotControlSurfaceComponent* Surface = URammsRobotControlSurfaceComponent::FindGoverningSurface(this))
	{
		for (const FName& Id : { RammsControlIds::Drive::Forward(), RammsControlIds::Drive::Turn(), RammsControlIds::Drive::Strafe() })
		{
			In.bStick |= FMath::Abs(Surface->GetControlValue(Id)) > StickMovedFraction;
		}
	}

	// --- the active drive mode's kind ---
	In.Mode = EMode::Allowed;
	R.bModeNotLevelled = false;
	TArray<UActorComponent*, TInlineAllocator<2>> Driving;
	if (bTakerHolds)
	{
		In.Mode = EMode::Taken;
	}
	if (Owner)
	{
		const URammsDriveModeSelector* Selector = Owner->FindComponentByClass<URammsDriveModeSelector>();
		const FName					   SelectedId = Selector ? Selector->GetActiveModeId() : NAME_None;
		// A drive mode switch under way is ramping the stance, or about to:
		// no rest until it is done, no start, and the wanted attitude waits
		// where it is (the law's rules).
		In.bStanceChanging = Selector && Selector->IsSwitchingMode();
		R.bStanceChanging = In.bStanceChanging;
		// (The leg motors: a mode that drives the seat's tilt has not taken
		// the legs.)
		TArray<FName> Levelled;
		R.Plant->LevelledMotorIds(Levelled);
		Levelled.Remove(R.Plant->SeatMotorId());
		bool bAnyActive = false;
		for (UActorComponent* Component : Owner->GetComponents())
		{
			const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Component);
			if (!Mode)
			{
				continue;
			}
			const FName Id = Mode->GetDriveModeId();
			if (!Mode->IsDriveModeActive() && (SelectedId.IsNone() || Id != SelectedId))
			{
				continue;
			}
			bAnyActive = true;
			if (HandingBack && Component == HandingBack)
			{
				// A taker handing the legs back from inside its own mode, which
				// is the active one for this call still (it selects the mode it
				// hands the robot to straight after): that mode is no reason
				// to give the base back or to stand by. What was put back as it
				// was goes on as it was.
				continue;
			}
			// Whoever drives the robot now may be asked to go slower -- but
			// never a mode that takes the legs (a crossing runs its own pace).
			if (Mode->IsDriveModeActive() && !DisabledModeIds.Contains(Id))
			{
				Driving.Add(Component);
			}
			if (!DisabledModeIds.Contains(Id) && (EnabledModeIds.IsEmpty() || EnabledModeIds.Contains(Id)))
			{
				continue;
			}
			R.bModeNotLevelled = true;
			// A mode it does not level in that drives the leg motors itself
			// has them at once; any other is given the base back gently.
			bool bTakesLegs = false;
			if (const IRammsControlContributor* Contributor = Cast<IRammsControlContributor>(Component))
			{
				TArray<FName> Claimed;
				Contributor->GetClaimedMotorIds(Claimed);
				bTakesLegs = Claimed.ContainsByPredicate([&Levelled](const FName& Motor) { return Levelled.Contains(Motor); });
			}
			In.Mode = bTakesLegs ? EMode::Taken : (In.Mode == EMode::Taken ? EMode::Taken : EMode::Release);
		}
		if (!bAnyActive && !EnabledModeIds.IsEmpty() && In.Mode == EMode::Allowed)
		{
			// No mode has the robot, and it levels only in named ones: wait.
			In.Mode = EMode::Release;
			R.bModeNotLevelled = true;
		}
	}

	// What whoever drives is being asked for by its stick, where it can say:
	// the speed cap goes by that, not by the speed there happens to be.
	for (UActorComponent* Component : Driving)
	{
		const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Component);
		const float			   Asked = Mode ? Mode->GetAskedSpeedCmS() : -1.0f;
		if (Asked >= 0.0f && FMath::IsFinite(Asked))
		{
			In.AskedSpeedCmS = FMath::Max(In.AskedSpeedCmS, static_cast<double>(Asked));
		}
	}

	In.bEnable = R.bSwitch && R.CannotLevel.IsEmpty();
	In.bYield = bYield;
	In.bResume = bResume;
	if (bResume && Adoption)
	{
		// The plane is out of the baselines already (AdoptPose), which the
		// legs measured above show.
		In.bResumeHasPlane = true;
		In.ResumePlaneDeg = Adoption->PlaneDeg;
		In.ResumeDz = Adoption->Dz;
	}

	// (The switches: as they stand now.)
	{
		const FCarryTuning	  CarryDefaults;
		const FSpeedCapTuning CapDefaults;
		FCarryTuning&		  Carry = R.Law.Carry.Tuning;
		Carry.bCrest = CVarLevelCrestRule.GetValueOnGameThread() != 0;
		Carry.bThreePoint = CVarLevelThreePointRule.GetValueOnGameThread() != 0;
		Carry.bHoldSpeed = CVarLevelThreePointHoldSpeed.GetValueOnGameThread() != 0;
		Carry.ThreePointFullCmS = FMath::Max(0.0f, CVarLevelThreePointFullCmS.GetValueOnGameThread());
		Carry.ThreePointNoneCmS = FMath::Max(static_cast<float>(Carry.ThreePointFullCmS), CVarLevelThreePointNoneCmS.GetValueOnGameThread());
		R.Law.SpeedScale.Cap.EarlyHeldSpans = CVarLevelEarlyCap.GetValueOnGameThread() != 0 ? CapDefaults.EarlyHeldSpans : 0.0;
	}
	const FOutputs Out = FLaw::Step(In, Dt, R.LawTuning, R.Law);

	// --- the motors' part of what the law answered ---
	if (Out.bClaim && !R.bClaimed && !ClaimMotors())
	{
		// Something else holds the base's offset layer, or a leg motor is not
		// one an offset can go on: levelling cannot run, and says so.
		R.CannotLevel = TEXT("the leg motors' offsets cannot be taken (another controller holds them)");
		R.bClaimRefused = true;
		R.bSwitch = false;
		R.Law.Reset();
		R.Last = FOutputs();
		UE_LOG(LogRammsLevel, Warning, TEXT("[Level] '%s' cannot be levelled: %s"), *GetNameSafe(Owner), *R.CannotLevel);
		return;
	}
	if (Out.Action == EAction::Fold && Out.bUnexpected && R.bClaimed)
	{
		// The backstop: a mode that drives the legs became active with nobody
		// calling YieldTo. Motor by motor, fold or zero.
		int32 Folded = 0, Zeroed = 0;
		R.Plant->FoldOrZero(Folded, Zeroed);
		UE_LOG(LogRammsLevel, Warning, TEXT("[Level] '%s': a drive mode took the legs without yielding; %d motor offsets folded into their commands, %d dropped"),
			*GetNameSafe(Owner), Folded, Zeroed);
	}
	else if (Out.Action == EAction::Clear && R.bClaimed)
	{
		// A reset (time ran backwards, the robot put elsewhere) throws all of
		// it away at once, and so does the end of a release, which has walked
		// the seat's trim out with the rest (what is left is under the law's
		// own "done"). Any other clear gives up the legs' correction and
		// leaves a trim still on the seat to be walked off it below.
		if (In.bColdReset || FMath::Abs(R.Plant->SeatTrimOnDeg()) <= 2.0 * ReleaseDoneDeg)
		{
			R.Plant->ZeroOffsets();
		}
		else
		{
			R.Plant->ZeroLegOffsets();
		}
	}
	else if (R.bClaimed && bLegs && Out.Dz.Num() == R.Legs.Num() && (Out.bWrite || (R.Law.Arbiter.HasOffsets() && R.Plant->OffsetsLost())))
	{
		// Only when the law says so: every write is a command at the base's
		// next tick, and a robot at rest is told nothing. Or when a leg has
		// lost the offset the law believes is on it (its motors were let go,
		// which clears it, and are commanded again): put back before the base
		// tells the leg its new command, which would otherwise go out bare.
		R.Plant->ApplyOffsets(Out.Dz);
		if (R.bSeat)
		{
			R.Plant->ApplySeatTrim(Out.SeatTrimDeg);
		}
	}
	// --- a trim left on the seat while the law holds none ---
	// Whoever takes the legs has not taken the seat: the seat's trim stays on
	// it through a take-over, as it is, and levelling goes on from it at the
	// hand-back (the law's start takes up what the seat carries). Only when
	// nothing will take it up -- switched off while something had the legs --
	// does it go out here: at the tilt policy's rate, as every deliberate
	// change of the rider's attitude, never in one write.
	bool bSeatTrimLeft = false;
	if (R.bClaimed && R.bSeat && !Out.bClaim)
	{
		const double OnDeg = R.Plant->SeatTrimOnDeg();
		if (OnDeg != 0.0)
		{
			const double StepDeg = FMath::Max(0.0, static_cast<double>(Tuning.TiltRateDegPerSec)) * Dt;
			const double LeftDeg = FMath::Abs(OnDeg) <= StepDeg || Tuning.TiltRateDegPerSec <= 0.0f ? 0.0 : OnDeg - FMath::Sign(OnDeg) * StepDeg;
			if (Dt > 0.0)
			{
				R.Plant->ApplySeatTrim(LeftDeg);
			}
			bSeatTrimLeft = Dt <= 0.0 || LeftDeg != 0.0;
		}
	}
	if (!Out.bClaim && R.bClaimed && !(bYield && R.bFoldedMotors) && !bSeatTrimLeft)
	{
		// (Not inside a YieldTo that folded: the taker may yet be refused,
		// and putting the baselines back as they were needs the layer. The
		// offsets are zero meanwhile, and the next tick lets go. Nor while a
		// trim is still being walked off the seat: letting go drops it.)
		R.Plant->ZeroOffsets();
		ReleaseMotors();
	}
	// --- the drive's pace ---
	// While the legs carry a correction, whoever drives is asked to keep to
	// the speed the legs can follow the ground at: every tick, to stand half a
	// second, so that it lapses by itself if this stops. Never while something
	// has taken the legs (a crossing runs its own pace, and the law's scale is
	// then full again too). When levelling stops the request is not dropped
	// while the scale is still under 1: the drive would step up to full speed
	// in one tick. The law goes on raising it at its own rate, and the request
	// goes with it until it is back at 1.
	const bool bSlows = (R.Law.Arbiter.HasOffsets() || Out.SpeedScale < 1.0) && In.Mode != EMode::Taken && !bYield;
	for (const TWeakObjectPtr<UObject>& Was : R.SlowedModes)
	{
		IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Was.Get());
		if (Mode && !(bSlows && Driving.Contains(Was.Get())))
		{
			Mode->SetDriveSpeedScale(SpeedScaleRequester(), 1.0f, 0.0f);
		}
	}
	R.SlowedModes.Reset();
	if (bSlows)
	{
		for (UActorComponent* Component : Driving)
		{
			if (IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Component))
			{
				Mode->SetDriveSpeedScale(SpeedScaleRequester(), static_cast<float>(Out.SpeedScale), SpeedScaleHoldSeconds);
				R.SlowedModes.Add(Component);
			}
		}
	}
	R.Last = Out;
	if (R.bSeat && R.bClaimed)
	{
		R.SeatTrimOnDeg = R.Plant->SeatTrimOnDeg();
	}
}

double URammsLevelAssistComponent::SeatTrimShownDeg() const
{
	// The law's own while it levels; else whatever is on the seat still.
	const RammsLevel::FAssistRuntime& R = *Runtime;
	return R.Law.Arbiter.HasOffsets() ? R.Last.SeatTrimDeg : (R.bClaimed ? R.SeatTrimOnDeg : 0.0);
}

bool URammsLevelAssistComponent::TakeBackSeatTrim()
{
	RammsLevel::FAssistRuntime& R = *Runtime;
	// Only a level assist that is on, holds the offsets and stands by: one
	// switched off meanwhile leaves the commands as the taker has them, the
	// seat's with the legs'.
	return R.Plant && R.bClaimed && R.bSwitch && R.CannotLevel.IsEmpty() && R.Law.Arbiter.GetState() == RammsLevel::EState::Yielded && R.Plant->UnfoldSeatTrim();
}

void URammsLevelAssistComponent::YieldTo(const UObject* Taker)
{
	RammsLevel::FAssistRuntime& R = *Runtime;
	if (!Taker || !R.Plant)
	{
		return;
	}
	if (R.Taker.Get() == Taker)
	{
		// It holds the legs already (a crossing stopped where it stood, tried
		// again): there is nothing to fold, and a refusal of this try must not
		// give back legs the stopped crossing still holds.
		++R.TakerNested;
		return;
	}
	// (The legs: the seat's trim stays on the seat until Taker takes the seat
	// too, YieldSeatTo, or levelling goes on from it.)
	R.Taker = Taker;
	R.TakerNested = 0;
	R.bTakerSeenActive = false;
	R.TakerTicks = 0;
	R.HandBackLeftDeg.Reset();
	// As things stand now, for a take-over refused before it begins.
	R.PreYieldLaw = R.Law;
	R.PreYieldLast = R.Last;
	R.bFoldedMotors = R.bClaimed;
	R.PreYieldAttitudeDeg.Reset();
	RammsLevel::FAttitude Attitude;
	if (R.Attitude && R.Attitude->Read(Attitude))
	{
		R.PreYieldAttitudeDeg = Attitude.Deg;
	}
	R.bSeatYielded = false;
	if (R.bClaimed)
	{
		R.Plant->Fold();
	}
	// The law is told in the same call (no time passes): by the time Taker
	// reads a command, the correction is in the baselines and the law's is 0.
	StepLaw(0.0, true, false);
}

void URammsLevelAssistComponent::YieldSeatTo(const UObject* Taker)
{
	RammsLevel::FAssistRuntime& R = *Runtime;
	// Only while standing by with the offsets held: a trim the law is still
	// working would be put straight back on top of what was folded.
	if (!Taker || !R.Plant || !R.bClaimed || R.Law.Arbiter.HasOffsets())
	{
		return;
	}
	R.Plant->FoldSeatTrim();
	R.SeatTrimOnDeg = R.Plant->SeatTrimOnDeg();
	R.bSeatYielded = true;
}

bool URammsLevelAssistComponent::AdoptPose(const FRammsDriveStance* NextStance, RammsLevel::FAdoption& OutAdoption)
{
	using namespace RammsLevel;
	FAssistRuntime& R = *Runtime;
	// Only a level assist that is on, holds the legs' offsets and stands by
	// has a plane in the commands to take back. (Switched off meanwhile, the
	// plane stays where the fold put it: the commands are the taker's.)
	if (!R.Plant || !R.bClaimed || !R.bSwitch || !R.CannotLevel.IsEmpty() || R.Law.Arbiter.GetState() != EState::Yielded)
	{
		return false;
	}
	// The pose as the motors are told it (standing by, no leg has an offset:
	// its baseline is what it is told), and where the legs go from here.
	FPlantLegs Legs;
	FLegValues StanceZ;
	if (!R.Plant->MeasureLegs(Legs) || !(NextStance ? R.Plant->StanceHeights(*NextStance, StanceZ) : R.Plant->FoldedHeights(StanceZ)))
	{
		return false;
	}
	// Who stands in that stance is judged on the ground the robot would rest
	// on THERE, not in the pose the taker left (a plant whose stances do not
	// stand level in the chassis' own frame says where that is).
	R.Plant->RestLegsAt(StanceZ, Legs);
	// (The tuning as the last tick made it: the preference where it was held.)
	R.Plant->ClearancePoints(R.LawTuning, R.Points);
	bool bAdopted = false;
	for (int32 Round = 0; Round < AdoptRounds; ++Round)
	{
		FAdoption  Plan;
		FLegValues Left;
		if (!PlanAdoption(Legs, StanceZ, R.Points, R.LawTuning, Plan) || !R.Plant->RebaseLegs(Plan.BaseZ, Left))
		{
			break;
		}
		// What the law goes on from is what the joints could be left with.
		bAdopted = true;
		OutAdoption = MoveTemp(Plan);
		OutAdoption.Dz = MoveTemp(Left);
		// Once more from the legs as they now measure: a wheel's place fore
		// and aft is taken at its baseline, which has just moved, and the
		// law will allocate over the legs it is given next. The pose is the
		// same: each baseline with what was left on it.
		if (Round + 1 == AdoptRounds || !R.Plant->MeasureLegs(Legs) || Legs.Num() != OutAdoption.Dz.Num())
		{
			break;
		}
		for (int32 Index = 0; Index < Legs.Num(); ++Index)
		{
			Legs[Index].BaseZ += OutAdoption.Dz[Index];
		}
		// (Measured again, the legs come with the ground their baselines rest
		// on as they now are: who stands in the stance is still judged there.)
		R.Plant->RestLegsAt(StanceZ, Legs);
	}
	return bAdopted;
}

void URammsLevelAssistComponent::ResumeFrom(const UObject* Taker, const FRammsDriveStance* NextStance)
{
	RammsLevel::FAssistRuntime& R = *Runtime;
	if (!Taker || !R.Plant)
	{
		return;
	}
	if (R.Taker.Get() != Taker)
	{
		// Nobody yielded to it. A drive mode that has the legs all the same
		// (picked by hand: this component's own tick stood by for it, its
		// backstop) may still hand them back to a stance; anything else is
		// nobody's hand-back.
		const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(const_cast<UObject*>(Taker));
		if (R.Taker.IsValid() || !NextStance || !Mode || !Mode->IsDriveModeActive() || R.Law.Arbiter.GetState() != RammsLevel::EState::Yielded)
		{
			return;
		}
	}
	if (!NextStance && R.TakerNested > 0)
	{
		--R.TakerNested;
		return;
	}
	R.Taker.Reset();
	R.TakerNested = 0;
	// The exact undo only with the layer still held, as it was at the fold:
	// without it no baseline can be put back. (And only while the taker had
	// the legs alone: with the seat's trim folded for it too, the law as it
	// stood no longer says what the seat carries.)
	if (!NextStance && !R.bSeatYielded && R.PreYieldLaw.IsSet() && (!R.bFoldedMotors || (R.bClaimed && R.Plant->Unfold())))
	{
		// Nothing was written since the fold: the baselines and the offsets
		// are exactly as they were before it. Whether the LAW may go on as it
		// stood depends on where the robot is. A taker that puts every leg
		// back where it found it may have moved the robot meanwhile (a curb
		// crossing ends on the kerb's other side): the base then stands turned
		// from where it was, on ground the law has never seen. As it stood the
		// law would hold the old ground for true and take the new attitude for
		// a knock: it creeps off it at a standing robot's small rate, the rider
		// left tilted meanwhile. So the plane on the legs is taken up as any
		// hand-back's: the wanted attitude starts from the one the base has and
		// walks to level at the rider's pace, and the offsets go on from what
		// the legs carry (a ride height among them), nothing stepping.
		const RammsLevel::FState& Was = *R.PreYieldLaw;
		RammsLevel::FAttitude	  Now;
		// (Or has simply had the legs a while: a crossing that ran, wherever it
		// ended, ends in another moment than it began in, and the law as it
		// stood then -- its ground, its integral, what it asked of the drive --
		// is from before. Only a take-over that never ran, refused in the tick
		// it was asked or the next, finds the law as it left it. Whether a
		// crossing "ended where it began" was a matter of a few tenths of a
		// degree in where the far side happened to leave the base; no crossing
		// of one kind of robot ever met it.)
		const bool bRan = R.bTakerSeenActive || R.TakerTicks > 1;
		const bool bTurned = R.bFoldedMotors && Was.Arbiter.HasOffsets()
			&& (bRan || (R.PreYieldAttitudeDeg.IsSet() && R.Attitude && R.Attitude->Read(Now) && RammsLevel::Inclination(Now.Deg - *R.PreYieldAttitudeDeg) > R.LawTuning.SettleErrorDeg));
		if (bTurned)
		{
			RammsLevel::FAdoption Kept;
			Kept.PlaneDeg = Kept.FittedDeg = Was.CorrectionDeg;
			Kept.Dz = Was.LastDz;
			R.PreYieldLaw.Reset();
			R.PreYieldLast.Reset();
			R.HandBackLeftDeg.Reset();
			StepLaw(0.0, false, true, &Kept, Taker);
			return;
		}
		// Refused before it began, or ended where it began: the baselines, the
		// offsets and the law exactly as they were, as if nothing had been
		// asked.
		R.Law = *R.PreYieldLaw;
		R.Last = R.PreYieldLast.Get(R.Last);
		R.PreYieldLaw.Reset();
		R.PreYieldLast.Reset();
		// (The drive was let go as the legs were yielded: it is asked again
		// for the scale it had, in the same call. A step in which no time
		// passes advances nothing else.)
		StepLaw(0.0, false, false, nullptr, Taker);
		return;
	}
	R.PreYieldLaw.Reset();
	R.PreYieldLast.Reset();
	// Levelling goes on from the pose the taker left: its plane back out of
	// the commands (while the fold's own commands are still remembered: with
	// no stance to go to, they are what the pose is measured against), and the
	// law told in the same call, before anyone reads a command again.
	// The seat first: its trim is on it still unless the taker took the seat
	// too (YieldSeatTo), and one folded for a taker that then never wrote the
	// seat's tilt after all is taken back out. The law's step below reads what
	// the seat carries and goes on from it.
	TakeBackSeatTrim();
	RammsLevel::FAdoption Adoption;
	const bool			  bAdopted = AdoptPose(NextStance, Adoption);
	R.Plant->ForgetFold();
	// What of the pose's plane could not be taken out stays in the commands,
	// for the stance ramp to turn the base by: whoever times that ramp is owed
	// how much (GetHandBackTurnDeg).
	R.HandBackLeftDeg.Reset();
	if (bAdopted && NextStance)
	{
		R.HandBackLeftDeg = Adoption.FittedDeg - Adoption.PlaneDeg;
	}
	StepLaw(0.0, false, true, bAdopted ? &Adoption : nullptr, Taker);
}

bool URammsLevelAssistComponent::GetHandBackTurnDeg(FVector2D& OutLeftDeg) const
{
	const RammsLevel::FAssistRuntime& R = *Runtime;
	if (!R.HandBackLeftDeg.IsSet() || !R.Law.Arbiter.HasOffsets())
	{
		return false;
	}
	OutLeftDeg = *R.HandBackLeftDeg;
	return true;
}

bool URammsLevelAssistComponent::IsLevelling() const
{
	return Runtime->Law.Arbiter.HasOffsets();
}

FVector2D URammsLevelAssistComponent::GetTargetAttitudeDeg() const
{
	return Runtime->Last.WantedDeg;
}

float URammsLevelAssistComponent::GetFoldedSeatTrimDeg() const
{
	return Runtime->Plant ? static_cast<float>(Runtime->Plant->FoldedSeatTrimDeg()) : 0.0f;
}

void URammsLevelAssistComponent::SetAssistEnabled(bool bEnabled)
{
	RammsLevel::FAssistRuntime& R = *Runtime;
	if (bEnabled && R.bClaimRefused)
	{
		// Whatever held the leg motors' offsets may have let go since: the
		// rider asking again is the moment to find out.
		R.bClaimRefused = false;
		R.CannotLevel.Reset();
	}
	if (bEnabled && !R.CannotLevel.IsEmpty())
	{
		UE_LOG(LogRammsLevel, Warning, TEXT("[Level] '%s': level assist stays off: %s"), *GetNameSafe(GetOwner()), *R.CannotLevel);
		return;
	}
	if (R.bSwitch != bEnabled)
	{
		R.bSwitch = bEnabled;
		UE_LOG(LogRammsLevel, Log, TEXT("[Level] '%s': level assist %s"), *GetNameSafe(GetOwner()), bEnabled ? TEXT("on") : TEXT("off"));
	}
}

bool URammsLevelAssistComponent::IsAssistEnabled() const
{
	return Runtime->bSwitch;
}

void URammsLevelAssistComponent::SetPreferenceDeg(const FVector2D& PreferenceDeg)
{
	Runtime->PreferenceDeg = RammsLevel::ClampPreference(PreferenceDeg, Tuning.MaxPreferenceDeg);
}

FVector2D URammsLevelAssistComponent::GetPreferenceDeg() const
{
	return RammsLevel::ClampPreference(Runtime->PreferenceDeg, Tuning.MaxPreferenceDeg);
}

FString URammsLevelAssistComponent::GetStatusLine(ERammsControlTextSeverity& OutSeverity) const
{
	const RammsLevel::FAssistRuntime& R = *Runtime;
	RammsLevel::FStatusFacts		  Facts;
	Facts.Status = R.Last.Status;
	Facts.OffReferenceDeg = RammsLevel::Inclination(R.Last.AttitudeDeg - R.LawTuning.LevelReferenceDeg);
	if (R.bSeat && Tuning.bSeatCoversTransients)
	{
		// The rider against what they asked for: the base against its
		// reference, with the seat's trim against the pitch it is to hold.
		Facts.RiderOffDeg = RammsLevel::Inclination(R.Last.AttitudeDeg - R.LawTuning.LevelReferenceDeg + FVector2D(R.Last.SeatTrimDeg - R.SeatPreferenceDeg, 0.0));
	}
	Facts.CannotLevel = R.CannotLevel;
	Facts.bOn = R.bSwitch;
	Facts.bModeNotLevelled = R.bModeNotLevelled;
	Facts.bStanceChanging = R.bStanceChanging;
	Facts.ReachLimit = R.Last.ReachLimit;
	return RammsLevel::StatusLine(Facts, OutSeverity);
}

// --- IRammsControlContributor -------------------------------------------------

int32 URammsLevelAssistComponent::GetControlOrder() const
{
	return RammsLevel::ControlOrder;
}

void URammsLevelAssistComponent::DescribeControls(FRammsControlSurface& OutSurface) const
{
	// (By the robot's description, not the plant: the controls are described
	// before the first tick finds the robot.)
	const AActor*						   Owner = GetOwner();
	const URammsCurbNegotiationController* Curb = Owner ? Owner->FindComponentByClass<URammsCurbNegotiationController>() : nullptr;
	const bool							   bSeat = Curb && Curb->Seat.IsSet() && !Curb->Seat.TiltMotor.IsNone();
	RammsLevel::DescribeControls(OutSurface, Runtime->bSwitch, Tuning.MaxPreferenceDeg, bSeat);
}

bool URammsLevelAssistComponent::ApplyControl(FName Id, float Value)
{
	if (Id == RammsLevel::AssistControlId())
	{
		const bool bWanted = FMath::RoundToInt(Value) != 0;
		SetAssistEnabled(bWanted);
		return IsAssistEnabled() == bWanted;
	}
	if (Id == RammsLevel::PreferencePitchControlId())
	{
		SetPreferenceDeg(FVector2D(Value, GetPreferenceDeg().Y));
		return true;
	}
	if (Id == RammsLevel::PreferenceRollControlId())
	{
		SetPreferenceDeg(FVector2D(GetPreferenceDeg().X, Value));
		return true;
	}
	return false;
}

bool URammsLevelAssistComponent::TriggerControl(FName Id)
{
	if (Id == RammsLevel::AssistControlId())
	{
		return ApplyControl(Id, IsAssistEnabled() ? 0.0f : 1.0f); // a button on the switch flips it
	}
	return false;
}

bool URammsLevelAssistComponent::ReadControl(FName Id, float& OutValue) const
{
	const RammsLevel::FAssistRuntime& R = *Runtime;
	if (Id == RammsLevel::AssistControlId())
	{
		OutValue = R.bSwitch ? 1.0f : 0.0f;
		return true;
	}
	if (Id == RammsLevel::StatusControlId())
	{
		ERammsControlTextSeverity Severity = ERammsControlTextSeverity::Info;
		GetStatusLine(Severity);
		OutValue = static_cast<float>(Severity);
		return true;
	}
	if (Id == RammsLevel::StateControlId())
	{
		OutValue = static_cast<float>(R.Last.State);
		return true;
	}
	if (Id == RammsLevel::TiltControlId())
	{
		OutValue = static_cast<float>(R.Last.TiltDeg);
		return true;
	}
	if (Id == RammsLevel::ReachControlId())
	{
		OutValue = static_cast<float>(R.Last.ReachLeft);
		return true;
	}
	if (Id == RammsLevel::SpeedScaleControlId())
	{
		OutValue = static_cast<float>(R.Last.SpeedScale);
		return true;
	}
	if (Id == RammsLevel::SeatTrimControlId())
	{
		OutValue = static_cast<float>(SeatTrimShownDeg());
		return true;
	}
	if (Id == RammsLevel::RiderTiltControlId())
	{
		// The base's attitude with what the seat pitches the rider by on it:
		// its own setting and the trim.
		OutValue = static_cast<float>(RammsLevel::Inclination(R.Last.AttitudeDeg + FVector2D(R.SeatOwnDeg + SeatTrimShownDeg(), 0.0)));
		return true;
	}
	if (Id == RammsLevel::PreferencePitchControlId())
	{
		OutValue = static_cast<float>(GetPreferenceDeg().X);
		return true;
	}
	if (Id == RammsLevel::PreferenceRollControlId())
	{
		OutValue = static_cast<float>(GetPreferenceDeg().Y);
		return true;
	}
	return false;
}

bool URammsLevelAssistComponent::ReadTarget(FName Id, float& OutTarget) const
{
	// The switch and the two preferences are what the rider sets and what
	// holds; the rest are readings.
	if (Id == RammsLevel::AssistControlId() || Id == RammsLevel::PreferencePitchControlId() || Id == RammsLevel::PreferenceRollControlId())
	{
		return ReadControl(Id, OutTarget);
	}
	return false;
}

bool URammsLevelAssistComponent::ReadControlText(FName Id, FText& OutText) const
{
	if (Id != RammsLevel::StatusControlId())
	{
		return false;
	}
	ERammsControlTextSeverity Severity = ERammsControlTextSeverity::Info;
	OutText = FText::FromString(GetStatusLine(Severity));
	return true;
}
