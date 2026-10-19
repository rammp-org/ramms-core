// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsDriveModeSelector.h"

#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "RammsControlSink.h"
#include "RammsDriveHandOver.h"
#include "RammsDriveMode.h"
#include "RammsRobotBaseComponent.h"
#include "RammsRobotControlSurfaceComponent.h"
#include "TimerManager.h"

namespace RammsDrive
{
	/** The selector's working state, kept out of its public header. */
	struct FSelectorState
	{
		FChassisMotionWindow Motion;
		FModeSwitcher		 Switcher;
		/** What the stances last commanded each motor they name: where a ramp
		 *  starts from while the joint is still there (its servo error is not
		 *  a move to make). */
		TMap<FName, float> MotorCommands;
	};
} // namespace RammsDrive

namespace
{
	IRammsDriveMode* AsMode(UActorComponent* Component)
	{
		return Cast<IRammsDriveMode>(Component);
	}

	FString ModeName(const IRammsDriveMode* Mode)
	{
		return Mode ? Mode->GetDriveModeId().ToString() : FString(TEXT("none"));
	}

	FString MotionText(const FRammsDriveMotion& M)
	{
		return FString::Printf(TEXT("forward %+.1f cm/s, sideways %+.1f cm/s, yaw %+.1f deg/s"), M.ForwardCmPerSecond, M.RightCmPerSecond,
			M.YawRateDegPerSecond);
	}

	/** A linkage endpoint axis a stance can command: on the linkage group, a
	 *  position, writable. */
	bool IsStanceAxis(const FRammsControlAxis& Axis, const TCHAR* Suffix)
	{
		return Axis.Group == RammsControlIds::Groups::Linkage() && Axis.Kind == ERammsControlKind::Position && !Axis.bReadOnly
			&& Axis.Id.ToString().EndsWith(Suffix);
	}

	/** A motor the stance last commanded is taken to be where it was put if
	 *  its joint reads within this of it (radians for a hinge). */
	constexpr float MotorAtCommandTolerance = 0.05f;
} // namespace

namespace RammsDrive
{
	/** The robot as FModeSwitcher sees it: the selector's modes, its stance
	 *  commands and its log. */
	class FSelectorHost final : public ISwitchHost
	{
	public:
		explicit FSelectorHost(URammsDriveModeSelector& InSelector)
			: Selector(InSelector)
		{
		}

		virtual bool IsValidMode(int32 Mode) const override { return Selector.ModeAt(Mode) != nullptr; }
		virtual bool IsModeActive(int32 Mode) const override
		{
			const IRammsDriveMode* M = Selector.ModeAt(Mode);
			return M && M->IsDriveModeActive();
		}
		virtual bool HasStance(int32 Mode) const override
		{
			FRammsDriveStance	   Stance;
			const IRammsDriveMode* M = Selector.ModeAt(Mode);
			return M && M->GetRequiredStance(Stance);
		}
		virtual FRammsDriveMotion GetCarriableMotion(int32 Mode, const FRammsDriveMotion& Motion) const override
		{
			const IRammsDriveMode* M = Selector.ModeAt(Mode);
			return M ? M->GetCarriableMotion(Motion) : Motion;
		}
		virtual bool BeginHandOver(int32 Mode, int32 Other) override
		{
			IRammsDriveMode* M = Selector.ModeAt(Mode);
			IRammsDriveMode* O = Selector.ModeAt(Other);
			return M && O && M->BeginHandOver(*O);
		}
		virtual void EndHandOver(int32 Mode) override
		{
			if (IRammsDriveMode* M = Selector.ModeAt(Mode))
			{
				M->EndHandOver();
			}
		}
		virtual void HoldOutgoingWheels(int32 Mode, bool bHold) override
		{
			if (IRammsDriveMode* M = Selector.ModeAt(Mode))
			{
				M->HoldWheelsWhileInactive(bHold && !M->IsDriveModeActive());
			}
		}
		virtual void HoldForStance(int32 Mode, bool bHold) override
		{
			if (IRammsDriveMode* M = Selector.ModeAt(Mode))
			{
				M->HoldForStance(bHold);
			}
		}
		virtual FString ActivateMode(int32 Mode, const FRammsDriveMotion* Seed, double HoldSeconds) override
		{
			return Selector.ActivateMode(Mode, Seed, HoldSeconds);
		}
		virtual bool GetStanceChannels(int32 Mode, TArray<FStanceChannel>& OutChannels) override;
		virtual void CommandStance(const TArray<FStanceChannel>& Channels, double Elapsed, double Duration) override;
		virtual void ApplyStance(int32 Mode) override
		{
			if (Selector.Modes.IsValidIndex(Mode))
			{
				Selector.ApplyStanceFor(Selector.Modes[Mode]);
			}
		}
		virtual FString ModeName(int32 Mode) const override { return ::ModeName(Selector.ModeAt(Mode)); }
		virtual void	Report(const FString& Line, bool bWarning) override
		{
			if (bWarning)
			{
				UE_LOG(LogTemp, Warning, TEXT("[DriveMode] '%s': %s"), *GetNameSafe(Selector.GetOwner()), *Line);
			}
			else
			{
				UE_LOG(LogTemp, Log, TEXT("[DriveMode] '%s': %s"), *GetNameSafe(Selector.GetOwner()), *Line);
			}
		}

	private:
		URammsDriveModeSelector& Selector;
	};

	bool FSelectorHost::GetStanceChannels(int32 Mode, TArray<FStanceChannel>& OutChannels)
	{
		OutChannels.Reset();
		const IRammsDriveMode* M = Selector.ModeAt(Mode);
		FRammsDriveStance	   Stance;
		if (!Selector.bApplyLinkageStance || !M || !M->GetRequiredStance(Stance))
		{
			return false;
		}
		// Every actuator the stance names, from where it is: a linkage axis from
		// the target its controller holds (the command it is tracking -- after a
		// curb crossing, that controller's last), else where it reads; a motor
		// from what a stance last commanded it while its joint is still there,
		// else from where its joint is.
		if (URammsRobotControlSurfaceComponent* Surface = URammsRobotControlSurfaceComponent::FindGoverningSurface(&Selector))
		{
			for (const FRammsControlAxis& Axis : Surface->DescribeControlSurface().Axes)
			{
				const bool bHeight = Stance.bHasLinkageHeight && IsStanceAxis(Axis, RammsControlIds::Linkage::HeightSuffix());
				const bool bTranslation = Stance.bHasLinkageTranslation && IsStanceAxis(Axis, RammsControlIds::Linkage::TranslationSuffix());
				if (!bHeight && !bTranslation)
				{
					continue;
				}
				FStanceChannel C;
				C.Id = Axis.Id;
				C.bHeight = bHeight;
				float From = 0.0f;
				C.From = Surface->GetControlTarget(Axis.Id, From) ? From : Surface->GetControlValue(Axis.Id);
				C.To = bHeight ? Stance.LinkageHeightCm : Stance.LinkageTranslationCm;
				OutChannels.Add(C);
			}
		}
		if (URammsRobotBaseComponent* Base = Selector.EnsureBase())
		{
			for (const FRammsStanceMotorTarget& Target : Stance.MotorTargets)
			{
				if (Target.MotorId.IsNone() || !Base->HasMotor(Target.MotorId))
				{
					continue;
				}
				FStanceChannel C;
				C.Id = Target.MotorId;
				C.bMotor = true;
				const float	 At = Base->GetMotorValue(Target.MotorId);
				const float* Commanded = Selector.State ? Selector.State->MotorCommands.Find(Target.MotorId) : nullptr;
				C.From = (Commanded && FMath::Abs(*Commanded - At) <= MotorAtCommandTolerance) ? *Commanded : At;
				C.To = Target.Target;
				OutChannels.Add(C);
			}
		}
		// (A linkage pose with no linkage control to ramp -- the low-level mode
		// withdraws them while it is active -- is planned again once the mode
		// asked for is in: FModeSwitcher::Request.)
		return true;
	}

	void FSelectorHost::CommandStance(const TArray<FStanceChannel>& Channels, double Elapsed, double Duration)
	{
		URammsRobotBaseComponent*			Base = Selector.EnsureBase();
		URammsRobotControlSurfaceComponent* Surface = URammsRobotControlSurfaceComponent::FindGoverningSurface(&Selector);
		// A linkage's two axes are one endpoint: fore/aft first, then height, and
		// a refused one once more after the other has moved (see ApplyStanceFor).
		TArray<int32, TInlineAllocator<8>> Refused;
		for (int32 Pass = 0; Pass < 2; ++Pass)
		{
			for (int32 i = 0; i < Channels.Num(); ++i)
			{
				const FStanceChannel& C = Channels[i];
				if ((Pass == 0) != (C.bMotor || !C.bHeight))
				{
					continue;
				}
				const float Value = static_cast<float>(StanceRampValue(C, Elapsed, Duration));
				if (C.bMotor)
				{
					if (Base)
					{
						Base->SetMotorCommand(C.Id, Value);
						if (Selector.State)
						{
							Selector.State->MotorCommands.Add(C.Id, Value);
						}
					}
				}
				else if (Surface && !Surface->SetControl(C.Id, Value, ERammsControlSource::Autonomy))
				{
					Refused.Add(i);
				}
			}
		}
		for (const int32 i : Refused)
		{
			const FStanceChannel& C = Channels[i];
			Surface->SetControl(C.Id, static_cast<float>(StanceRampValue(C, Elapsed, Duration)), ERammsControlSource::Autonomy);
		}
	}
} // namespace RammsDrive

namespace
{
	RammsDrive::FMotionSample SampleOf(const RammsDrive::FSelectorState* State)
	{
		return State ? RammsDrive::FMotionSample(State->Motion.IsValid(), State->Motion.GetMotion()) : RammsDrive::FMotionSample();
	}
} // namespace

URammsDriveModeSelector::URammsDriveModeSelector()
{
	// Ticks to measure the chassis (a switch on the move hands its motion over,
	// so the window behind it has to be full when one is asked for), to wait
	// out a hand-over and to ramp a stance.
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void URammsDriveModeSelector::BeginPlay()
{
	Super::BeginPlay();
	State = MakePimpl<RammsDrive::FSelectorState>();
	GatherModes();

	if (Modes.Num() == 0)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[DriveMode] '%s' found no drive modes on this robot; nothing to select."),
			*GetNameSafe(GetOwner()));
		return;
	}

	// The joints the stances hold ride on the chassis (corner cranks pivot on
	// it), so where they are is where the chassis is: they are what the
	// chassis' motion is measured from.
	ChassisMotors.Reset();
	ChassisMotorEnds.Reset();
	for (UActorComponent* Component : Modes)
	{
		FRammsDriveStance	   Stance;
		const IRammsDriveMode* Mode = AsMode(Component);
		if (Mode && Mode->GetRequiredStance(Stance))
		{
			for (const FRammsStanceMotorTarget& Target : Stance.MotorTargets)
			{
				if (!Target.MotorId.IsNone())
				{
					ChassisMotors.AddUnique(Target.MotorId);
				}
			}
		}
	}

	// Next tick, not now. Sibling contributors resolve their own dependencies
	// in their BeginPlay in no guaranteed order relative to this one, and
	// selecting a mode rebuilds the surface and commands a stance through it.
	// Run first and the 5-bars have not resolved their kinematic specs yet, so
	// they advertise no axes, the stance finds nothing to command, and nothing
	// ever retries it -- the surface's own deferred rebuild brings the axes
	// back but not the stance.
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().SetTimerForNextTick(this, &URammsDriveModeSelector::SelectInitialMode);
	}
}

void URammsDriveModeSelector::SelectInitialMode()
{
	if (Modes.Num() == 0 || !State)
	{
		return;
	}

	// Whatever the components defaulted to, exactly one ends up live -- in its
	// stance at once: the robot is being put down as it spawns, and nothing is
	// moving to carry across.
	FName Wanted = InitialModeId;
	if (Wanted.IsNone())
	{
		if (IRammsDriveMode* First = AsMode(Modes[0]))
		{
			Wanted = First->GetDriveModeId();
		}
	}
	int32 Index = IndexOf(Wanted);
	if (Index == INDEX_NONE)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[DriveMode] '%s': no mode '%s' on this robot; falling back to the first."),
			*GetNameSafe(GetOwner()), *Wanted.ToString());
		Index = 0;
	}
	const FString Carried = ActivateMode(Index, nullptr, 0.0);
	State->Switcher.Reset(Index);
	ApplyStanceFor(Modes[Index]);
	UE_LOG(LogTemp, Log, TEXT("[DriveMode] '%s': initial mode %s, %s, stance at once"), *GetNameSafe(GetOwner()), *ModeName(ModeAt(Index)), *Carried);
}

void URammsDriveModeSelector::GatherModes()
{
	Modes.Reset();
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}
	for (UActorComponent* Component : Owner->GetComponents())
	{
		if (Component && AsMode(Component))
		{
			Modes.Add(Component);
		}
	}
	// Stable order, so the enum's indices do not depend on component iteration.
	Modes.Sort([](const UActorComponent& A, const UActorComponent& B) {
		const IRammsDriveMode* ModeA = Cast<IRammsDriveMode>(&A);
		const IRammsDriveMode* ModeB = Cast<IRammsDriveMode>(&B);
		return ModeA && ModeB
			&& ModeA->GetDriveModeId().LexicalLess(ModeB->GetDriveModeId());
	});
}

TArray<FName> URammsDriveModeSelector::GetAvailableModeIds() const
{
	TArray<FName> Ids;
	for (UActorComponent* Component : Modes)
	{
		if (const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Component))
		{
			Ids.Add(Mode->GetDriveModeId());
		}
	}
	return Ids;
}

int32 URammsDriveModeSelector::ActiveIndex() const
{
	return State ? State->Switcher.GetActive() : INDEX_NONE;
}

FName URammsDriveModeSelector::GetActiveModeId() const
{
	if (const IRammsDriveMode* Mode = ModeAt(ActiveIndex()))
	{
		return Mode->GetDriveModeId();
	}
	return NAME_None;
}

int32 URammsDriveModeSelector::IndexOf(FName ModeId) const
{
	for (int32 i = 0; i < Modes.Num(); ++i)
	{
		const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Modes[i]);
		if (Mode && Mode->GetDriveModeId() == ModeId)
		{
			return i;
		}
	}
	return INDEX_NONE;
}

IRammsDriveMode* URammsDriveModeSelector::ModeAt(int32 Index) const
{
	return Modes.IsValidIndex(Index) ? Cast<IRammsDriveMode>(Modes[Index]) : nullptr;
}

URammsRobotBaseComponent* URammsDriveModeSelector::EnsureBase() const
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

double URammsDriveModeSelector::Now() const
{
	// The physics clock where there is one: a simulator that runs slower than
	// real time (the lift-drive's roller model, about a quarter) would
	// otherwise see every ramp and timeout here run that many times too fast.
	if (const URammsRobotBaseComponent* Base = EnsureBase())
	{
		const double Sim = Base->GetSimulationTime();
		if (Sim >= 0.0)
		{
			return Sim;
		}
	}
	const UWorld* World = GetWorld();
	return World ? World->GetTimeSeconds() : 0.0;
}

bool URammsDriveModeSelector::LocateChassis(FVector2D& OutCentre, FVector2D& OutForward)
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return false;
	}
	URammsRobotBaseComponent* Base = EnsureBase();
	if (Base && ChassisMotors.Num() >= 2)
	{
		TArray<FVector> Points;
		Points.Reserve(ChassisMotors.Num());
		for (const FName Id : ChassisMotors)
		{
			FTransform Xf;
			if (!Base->GetMotorTransform(Id, Xf))
			{
				return false;
			}
			Points.Add(Xf.GetLocation());
		}
		FVector Mean = FVector::ZeroVector;
		for (const FVector& P : Points)
		{
			Mean += P;
		}
		Mean /= Points.Num();
		if (ChassisMotorEnds.Num() != Points.Num())
		{
			// Which end of the chassis each joint is at, once, in the frame the
			// robot spawned in (the actor's: on a simulated articulation the
			// actor stays where it spawned while the chassis drives off).
			const FVector Along = Owner->GetActorForwardVector().GetSafeNormal2D();
			ChassisMotorEnds.Reset();
			for (const FVector& P : Points)
			{
				ChassisMotorEnds.Add(FVector::DotProduct(P - Mean, Along) >= 0.0 ? 1 : -1);
			}
		}
		FVector Front = FVector::ZeroVector, Rear = FVector::ZeroVector;
		int32	NFront = 0, NRear = 0;
		for (int32 i = 0; i < Points.Num(); ++i)
		{
			if (ChassisMotorEnds[i] > 0)
			{
				Front += Points[i];
				++NFront;
			}
			else
			{
				Rear += Points[i];
				++NRear;
			}
		}
		if (NFront > 0 && NRear > 0)
		{
			const FVector Along = Front / NFront - Rear / NRear;
			if (!FVector2D(Along.X, Along.Y).IsNearlyZero())
			{
				OutCentre = FVector2D(Mean.X, Mean.Y);
				OutForward = FVector2D(Along.X, Along.Y).GetSafeNormal();
				return true;
			}
		}
	}
	// No chassis joints to go by: the actor, where its root is a simulated body
	// (a robot whose chassis is its root). Anywhere else the actor stays where
	// it spawned while the robot drives off, so there is nothing to measure:
	// such a robot switches as at rest, its incoming mode from rest.
	const UPrimitiveComponent* Root = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
	if (!Root || !Root->IsSimulatingPhysics())
	{
		return false;
	}
	const FVector Location = Owner->GetActorLocation();
	const FVector Forward = Owner->GetActorForwardVector();
	if (FVector2D(Forward.X, Forward.Y).IsNearlyZero())
	{
		return false;
	}
	OutCentre = FVector2D(Location.X, Location.Y);
	OutForward = FVector2D(Forward.X, Forward.Y).GetSafeNormal();
	return true;
}

FRammsDriveMotion URammsDriveModeSelector::GetMeasuredMotion() const
{
	return State ? State->Motion.GetMotion() : FRammsDriveMotion();
}

bool URammsDriveModeSelector::IsSwitchingMode() const
{
	return State && State->Switcher.GetPhase() != RammsDrive::ESwitchPhase::Idle;
}

FString URammsDriveModeSelector::GetSwitchStatus() const
{
	return State ? State->Switcher.GetStatus() : FString();
}

namespace
{
	RammsDrive::FSwitchConfig ConfigOf(const URammsDriveModeSelector& S)
	{
		RammsDrive::FSwitchConfig C;
		C.Limits.MaxSidewaysCmS = S.HandOverMaxSidewaysSpeed;
		C.Limits.TimeoutSeconds = S.HandOverTimeoutSeconds;
		C.MinStageSeconds = S.StanceRampSeconds;
		C.MaxLinkageSpeed = S.StanceMaxLinkageSpeed;
		C.MaxMotorRate = S.StanceMaxMotorRate;
		return C;
	}
} // namespace

void URammsDriveModeSelector::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!State)
	{
		return;
	}
	const double T = Now();
	const double Dt = (LastTickTime >= 0.0 && T >= LastTickTime) ? T - LastTickTime : 0.0;
	LastTickTime = T;

	FVector2D Centre, Forward;
	if (LocateChassis(Centre, Forward))
	{
		State->Motion.Add(T, Centre, Forward);
	}
	if (State->Switcher.GetPhase() == RammsDrive::ESwitchPhase::Idle)
	{
		return;
	}
	RammsDrive::FSelectorHost Host(*this);
	State->Switcher.Tick(Host, Dt, SampleOf(State.Get()), ConfigOf(*this));
}

int32 URammsDriveModeSelector::CommandLinkageAxes(URammsRobotControlSurfaceComponent& Surface,
	const TCHAR* IdSuffix, float Value, const TCHAR* What, bool bWarn) const
{
	int32 Commanded = 0;
	for (const FRammsControlAxis& Axis : Surface.DescribeControlSurface().Axes)
	{
		if (!IsStanceAxis(Axis, IdSuffix))
		{
			continue;
		}
		// Autonomy: taking a stance is the robot's own doing, and it has to
		// win over whatever a panel last left on the axis.
		if (Surface.SetControl(Axis.Id, Value, ERammsControlSource::Autonomy))
		{
			++Commanded;
		}
		else if (bWarn)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[DriveMode] '%s' refused the %.1f cm %s its mode's stance needs; the wrong "
					 "wheels may be on the ground."),
				*Axis.Id.ToString(), Value, What);
		}
	}
	return Commanded;
}

void URammsDriveModeSelector::ApplyStanceFor(UActorComponent* ModeComponent)
{
	const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(ModeComponent);
	FRammsDriveStance	   Stance;
	if (!bApplyLinkageStance || !Mode || !Mode->GetRequiredStance(Stance))
	{
		return;
	}
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	// Command the stance rather than waiting for the robot to be driven into
	// it: the wheels this mode needs on the ground are only there once the
	// legs hold the right pose.
	//
	// Through the control surface by Id, not by calling a linkage controller.
	// Naming URamms5BarLinkageController here would leave this selector
	// depending on ramms-controllers once that class moves, which is exactly
	// what the extraction is meant to avoid -- and it would only ever work for
	// one kind of leg. Anything that advertises a linkage height can take a
	// stance.
	URammsRobotControlSurfaceComponent* Surface =
		URammsRobotControlSurfaceComponent::FindGoverningSurface(this);
	if (Stance.bHasLinkageHeight || Stance.bHasLinkageTranslation)
	{
		if (!Surface)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[DriveMode] '%s' needs a linkage stance but the robot has no control surface to "
					 "command it through; the wrong wheels may be on the ground."),
				*GetNameSafe(ModeComponent));
		}
		else
		{
			// Two passes, fore/aft then height each time, because the axes are
			// one endpoint over one pair of motors and the reachable set is a
			// curved region: which order works depends on which way you are
			// going. Reaching a high, narrow pose needs the endpoint brought
			// back to the centre first, or the height is refused at the
			// current fore/aft; reaching a far-out one needs the height
			// dropped first, or the fore/aft is refused. Alternating twice
			// gets there either way, and a command already satisfied is a
			// no-op the second time.
			int32 Commanded = 0;
			for (int32 Pass = 0; Pass < 2; ++Pass)
			{
				Commanded = 0;
				if (Stance.bHasLinkageTranslation)
				{
					Commanded += CommandLinkageAxes(*Surface,
						RammsControlIds::Linkage::TranslationSuffix(), Stance.LinkageTranslationCm,
						TEXT("fore/aft"), /*bWarn=*/Pass == 1);
				}
				if (Stance.bHasLinkageHeight)
				{
					Commanded += CommandLinkageAxes(*Surface, RammsControlIds::Linkage::HeightSuffix(),
						Stance.LinkageHeightCm, TEXT("height"), /*bWarn=*/Pass == 1);
				}
			}
			if (Commanded == 0)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("[DriveMode] '%s' needs a linkage stance but this robot advertises no linkage "
						 "controls to command."),
					*GetNameSafe(ModeComponent));
			}
		}
	}

	// The corner cranks and anything else the stance names. These go through
	// the robot base by Id: they are ordinary registry motors, not something
	// the mode claims, so the low-level mode can still reach them.
	if (Stance.MotorTargets.Num() > 0)
	{
		URammsRobotBaseComponent* Base = EnsureBase();
		if (!Base)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[DriveMode] '%s' wants %d motors held for its stance but the robot has no "
					 "RammsRobotBaseComponent to command them through."),
				*GetNameSafe(ModeComponent), Stance.MotorTargets.Num());
			return;
		}
		for (const FRammsStanceMotorTarget& Target : Stance.MotorTargets)
		{
			if (Target.MotorId.IsNone())
			{
				continue;
			}
			if (!Base->HasMotor(Target.MotorId))
			{
				// Naming a motor this robot does not have is an authoring
				// mistake worth hearing about: the stance silently half-applies
				// and the wrong wheels stay down.
				UE_LOG(LogTemp, Warning,
					TEXT("[DriveMode] stance for '%s' names motor '%s', which is not in this "
						 "robot's registry; that part of the stance is not applied."),
					*GetNameSafe(ModeComponent), *Target.MotorId.ToString());
				continue;
			}
			Base->SetMotorCommand(Target.MotorId, Target.Target);
			if (State)
			{
				State->MotorCommands.Add(Target.MotorId, Target.Target);
			}
		}
	}
}

FString URammsDriveModeSelector::ActivateMode(int32 Index, const FRammsDriveMotion* Seed, double HoldSeconds)
{
	if (!Modes.IsValidIndex(Index))
	{
		return TEXT("no such mode");
	}
	URammsRobotControlSurfaceComponent* Surface = URammsRobotControlSurfaceComponent::FindGoverningSurface(this);

	// On the move, the operator's drive commands carry across with the motion.
	// The surface drops what was held on a control whose contributor changes
	// -- right at rest, where both modes come up at zero -- so a panel stick
	// held still, or a remote client between its messages, would otherwise
	// leave the incoming mode with no command until the next one came, and it
	// would brake. Only what the outgoing mode is following now: a command
	// the surface still records but something else has since overwritten is
	// not the operator's.
	struct FHeldDrive
	{
		FName				Id;
		float				Value = 0.0f;
		ERammsControlSource Source = ERammsControlSource::Script;
	};
	TArray<FHeldDrive, TInlineAllocator<4>> Held;
	if (Seed && Surface)
	{
		for (const FRammsControlAxis& Axis : Surface->DescribeControlSurface().Axes)
		{
			if (Axis.Group != RammsControlIds::Groups::Drive() || Axis.Kind != ERammsControlKind::Continuous)
			{
				continue;
			}
			float Target = 0.0f;
			if (Surface->GetControlTarget(Axis.Id, Target) && !FMath::IsNearlyZero(Target)
				&& FMath::IsNearlyEqual(Surface->GetControlValue(Axis.Id), Target, 1e-3f))
			{
				Held.Add({ Axis.Id, Target, IRammsControlSink::Execute_GetAxisOwner(Surface, Axis.Id) });
			}
		}
	}

	// Stand the others down first: two modes live at once would write the same
	// motors in one frame and offer duplicate control Ids.
	for (int32 i = 0; i < Modes.Num(); ++i)
	{
		if (i != Index)
		{
			if (IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Modes[i]))
			{
				Mode->SetDriveModeActive(false);
			}
		}
	}
	IRammsDriveMode* Incoming = ModeAt(Index);
	if (Incoming)
	{
		Incoming->SetDriveModeActive(true);
	}

	// Rebuild before applying the stance, not after. The surface is built from
	// what the contributors describe, and the inactive mode has dropped the
	// shared controls -- so they leave the panel. It also has to be current
	// *before* the stance goes out, because the stance is commanded through it
	// by Id: coming back from the low-level mode the 5-bars have just been
	// un-suspended, and against a stale surface they still advertise nothing,
	// so the stance found no linkage controls to command and the robot stayed
	// in the wrong pose.
	if (Surface)
	{
		Surface->RebuildControlSurface();
	}
	int32 Carried = 0;
	for (const FHeldDrive& H : Held)
	{
		if (Surface->SetControl(H.Id, H.Value, H.Source))
		{
			++Carried;
		}
	}

	// The motion the robot has, as far as this mode can carry it, so that it
	// carries on rather than starting from rest.
	if (!Seed)
	{
		return TEXT("nothing carried");
	}
	if (!Incoming)
	{
		return TEXT("no mode to carry it");
	}
	const FRammsDriveMotion Motion = Incoming->GetCarriableMotion(*Seed);
	Incoming->SeedMotion(Motion, static_cast<float>(HoldSeconds));
	return FString::Printf(TEXT("carrying %s (stance change %.2f s)%s"), *MotionText(Motion), HoldSeconds,
		Carried > 0 ? *FString::Printf(TEXT(" and the operator's %d drive command%s"), Carried, Carried == 1 ? TEXT("") : TEXT("s")) : TEXT(""));
}

bool URammsDriveModeSelector::SetActiveMode(FName ModeId)
{
	const int32 Wanted = IndexOf(ModeId);
	if (Wanted == INDEX_NONE || !State)
	{
		return false;
	}
	RammsDrive::FSelectorHost Host(*this);
	// The stance changes away from the active mode no faster than it asks: a
	// curb crossing's end, on a slope, has the base turn from level to the
	// ground's attitude within its tilt limits.
	RammsDrive::FSwitchConfig Config = ConfigOf(*this);
	if (const IRammsDriveMode* Outgoing = ModeAt(ActiveIndex()))
	{
		Config.MinStageSeconds = FMath::Max(Config.MinStageSeconds, static_cast<double>(Outgoing->GetStanceChangeMinSeconds()));
	}
	return State->Switcher.Request(Host, Wanted, SampleOf(State.Get()), Config);
}

bool URammsDriveModeSelector::CycleMode()
{
	if (Modes.Num() < 2)
	{
		return false;
	}
	const int32			   Next = (ActiveIndex() + 1) % Modes.Num();
	const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Modes[Next]);
	return Mode && SetActiveMode(Mode->GetDriveModeId());
}

// --- control surface -----------------------------------------------------------

void URammsDriveModeSelector::DescribeControls(FRammsControlSurface& OutSurface) const
{
	if (Modes.Num() < 2)
	{
		// Nothing to choose between; offering a one-entry selector is noise.
		return;
	}
	FRammsControlAxis Axis;
	Axis.Id = RammsControlIds::Drive::Mode();
	Axis.Group = RammsControlIds::Groups::Drive();
	Axis.DisplayName = NSLOCTEXT("Ramms", "DriveMode", "Drive mode");
	Axis.Kind = ERammsControlKind::Enum;
	Axis.Units = ERammsControlUnits::None;
	Axis.Order = -1; // above the sticks it governs
	for (UActorComponent* Component : Modes)
	{
		if (const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Component))
		{
			Axis.EnumLabels.Add(Mode->GetDriveModeDisplayName());
		}
	}
	Axis.Range = FVector2D(0.0, FMath::Max(0, Axis.EnumLabels.Num() - 1));
	Axis.DefaultValue = static_cast<float>(FMath::Max(ActiveIndex(), 0));
	OutSurface.Add(Axis);
}

bool URammsDriveModeSelector::ApplyControl(FName Id, float Value)
{
	if (Id != RammsControlIds::Drive::Mode())
	{
		return false;
	}
	const int32 Index = FMath::RoundToInt(Value);
	if (!Modes.IsValidIndex(Index))
	{
		return false;
	}
	const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Modes[Index]);
	return Mode && SetActiveMode(Mode->GetDriveModeId());
}

bool URammsDriveModeSelector::TriggerControl(FName Id)
{
	return Id == RammsControlIds::Drive::Mode() && CycleMode();
}

bool URammsDriveModeSelector::ReadControl(FName Id, float& OutValue) const
{
	if (Id != RammsControlIds::Drive::Mode())
	{
		return false;
	}
	OutValue = static_cast<float>(FMath::Max(ActiveIndex(), 0));
	return true;
}
