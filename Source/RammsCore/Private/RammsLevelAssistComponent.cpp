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
#include "RammsRobotBaseComponent.h"
#include "RammsRobotControlSurfaceComponent.h"
#include "RammsSeatMechanismController.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsLevel, Log, All);

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

		/** The physics clock and the chassis at the last tick. */
		double	LastSimTime = -1.0;
		FVector LastPosition = FVector::ZeroVector;
		bool	bHasPosition = false;
		double	SpeedCmS = 0.0;
		bool	bColdReset = false;
		/** The active drive mode is one it does not level in; the mode
		 *  selector is changing the stance (both for the status). */
		bool bModeNotLevelled = false;
		bool bStanceChanging = false;
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
			// the frame. It is left where it is; what it writes while the
			// legs are levelled is told at the base's next tick, a frame
			// late and with its offset (never bare).
			if (bDrivesLevelled && !R.LateWritersSaid.Contains(Component->GetFName()))
			{
				R.LateWritersSaid.Add(Component->GetFName());
				UE_LOG(LogRammsLevel, Log, TEXT("[Level] '%s': '%s' ticks after the robot base; while the legs are levelled, a command it writes in its own tick reaches the motor a frame late"),
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
	// Which sibling describes the robot picks the kind of plant. Today: the
	// Gen2 curb controller's legs.
	const URammsCurbNegotiationController* Curb = Owner ? Owner->FindComponentByClass<URammsCurbNegotiationController>() : nullptr;
	if (!Base || !Curb)
	{
		R.CannotLevel = TEXT("no robot description");
	}
	else
	{
		TUniquePtr<RammsLevel::IPlant> Plant = RammsLevel::MakeLegPlant(*Base, *Curb, this);
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
			R.Attitude = MakeUnique<RammsLevel::FPivotAttitudeSource>(*Base, *Curb);
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

void URammsLevelAssistComponent::StepLaw(double Dt, bool bYield, bool bResume, const RammsLevel::FAdoption* Adoption)
{
	using namespace RammsLevel;
	FAssistRuntime& R = *Runtime;
	AActor*			Owner = GetOwner();
	R.LawTuning = MakeTuning(Tuning, R.PreferenceDeg);

	FInputs In;
	// --- the chassis ---
	FAttitude Attitude;
	In.bAttitudeValid = R.Attitude && R.Attitude->Read(Attitude);
	if (In.bAttitudeValid)
	{
		In.AttitudeDeg = Attitude.Deg;
		if (R.bHasPosition && Dt > 0.0)
		{
			const double Moved = FVector::Dist(Attitude.Position, R.LastPosition);
			R.bColdReset |= Moved > PoseJumpCm;
			R.SpeedCmS = Moved / Dt;
		}
		if (Dt > 0.0 || !R.bHasPosition)
		{
			R.LastPosition = Attitude.Position;
			R.bHasPosition = true;
		}
	}
	In.SpeedCmS = R.SpeedCmS;
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
	R.Plant->ClearancePoints(R.LawTuning, R.Points);
	In.Legs = R.Legs;
	In.Points = R.Points;
	In.bSeat = R.Plant->HasSeat();

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
	if (const UObject* Taker = R.Taker.Get())
	{
		// A taker holds the legs from its YieldTo until it hands back, or
		// until its drive mode, once active, no longer is. (It yields before
		// it selects its mode: it has until this component's next tick.)
		const IRammsDriveMode* TakerMode = Cast<IRammsDriveMode>(const_cast<UObject*>(Taker));
		const bool			   bActive = !TakerMode || TakerMode->IsDriveModeActive();
		R.bTakerSeenActive |= TakerMode && bActive;
		if (bActive || (!R.bTakerSeenActive && R.TakerTicks < 1))
		{
			In.Mode = EMode::Taken;
		}
		else
		{
			// It let go without handing back (a crossing stopped, its mode
			// left by hand): levelling starts again cold from what it left.
			R.Taker.Reset();
			R.TakerNested = 0;
			R.PreYieldLaw.Reset();
			R.PreYieldLast.Reset();
			R.Plant->ForgetFold();
		}
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
		TArray<FName> Levelled;
		R.Plant->LevelledMotorIds(Levelled);
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
		R.Plant->ZeroOffsets();
	}
	else if (R.bClaimed && bLegs && Out.Dz.Num() == R.Legs.Num() && (Out.bWrite || (R.Law.Arbiter.HasOffsets() && R.Plant->OffsetsLost())))
	{
		// Only when the law says so: every write is a command at the base's
		// next tick, and a robot at rest is told nothing. Or when a leg has
		// lost the offset the law believes is on it (its motors were let go,
		// which clears it, and are commanded again): put back before the base
		// tells the leg its new command, which would otherwise go out bare.
		R.Plant->ApplyOffsets(Out.Dz);
	}
	if (!Out.bClaim && R.bClaimed && !(bYield && R.bFoldedMotors))
	{
		// (Not inside a YieldTo that folded: the taker may yet be refused,
		// and putting the baselines back as they were needs the layer. The
		// offsets are zero meanwhile, and the next tick lets go.)
		R.Plant->ZeroOffsets();
		ReleaseMotors();
	}
	// --- the drive's pace ---
	// While the legs carry a correction, whoever drives is asked to keep to
	// the speed the legs can follow the ground at: every tick, to stand half a
	// second, so that it lapses by itself if this stops. Never while something
	// has taken the legs; and taken back at once when levelling stops, rather
	// than left to run out.
	const bool bSlows = R.Law.Arbiter.HasOffsets() && In.Mode != EMode::Taken && !bYield;
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
	// Out.SeatTrimDeg is worked out and not yet put on the seat.
	R.Last = Out;
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
	R.Taker = Taker;
	R.TakerNested = 0;
	R.bTakerSeenActive = false;
	R.TakerTicks = 0;
	// As things stand now, for a take-over refused before it begins.
	R.PreYieldLaw = R.Law;
	R.PreYieldLast = R.Last;
	R.bFoldedMotors = R.bClaimed;
	if (R.bClaimed)
	{
		R.Plant->Fold();
	}
	// The law is told in the same call (no time passes): by the time Taker
	// reads a command, the correction is in the baselines and the law's is 0.
	StepLaw(0.0, true, false);
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
	R.LawTuning = MakeTuning(Tuning, R.PreferenceDeg);
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
	// without it no baseline can be put back.
	if (!NextStance && R.PreYieldLaw.IsSet() && (!R.bFoldedMotors || (R.bClaimed && R.Plant->Unfold())))
	{
		// Refused before anything was written: the baselines, the offsets and
		// the law exactly as they were, as if nothing had been asked.
		R.Law = *R.PreYieldLaw;
		R.Last = R.PreYieldLast.Get(R.Last);
		R.PreYieldLaw.Reset();
		R.PreYieldLast.Reset();
		return;
	}
	R.PreYieldLaw.Reset();
	R.PreYieldLast.Reset();
	// Levelling goes on from the pose the taker left: its plane back out of
	// the commands (while the fold's own commands are still remembered: with
	// no stance to go to, they are what the pose is measured against), and the
	// law told in the same call, before anyone reads a command again.
	RammsLevel::FAdoption Adoption;
	const bool			  bAdopted = AdoptPose(NextStance, Adoption);
	R.Plant->ForgetFold();
	StepLaw(0.0, false, true, bAdopted ? &Adoption : nullptr);
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
	// The seat's tilt motor is not levelled yet, so no trim is ever folded
	// into its command.
	return 0.0f;
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
	RammsLevel::DescribeControls(OutSurface, Runtime->bSwitch, Tuning.MaxPreferenceDeg);
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
