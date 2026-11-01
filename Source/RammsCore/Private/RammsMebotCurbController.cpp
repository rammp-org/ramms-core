// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMebotCurbController.h"

#include "Curb/RammsMebotCurb.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "RammsDriveHandOver.h"
#include "RammsDriveModeSelector.h"
#include "RammsMotorSpec.h"
#include "RammsRobotBaseComponent.h"
#include "RammsRobotControlSurfaceComponent.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsMebotCurb, Log, All);

namespace
{
	FName MebotCurbNegotiateId()
	{
		return FName("curb.negotiate");
	}
	FName MebotCurbAbortId()
	{
		return FName("curb.abort");
	}
	FName MebotCurbPhaseId()
	{
		return FName("curb.phase");
	}

	RammsMebotCurb::FSupportTable MebotCurbTable(const FRammsMebotSupportTable& T)
	{
		RammsMebotCurb::FSupportTable Out;
		Out.AngleRad = T.AngleRad;
		Out.HeightCm = T.HeightCm;
		return Out;
	}

	FString MebotCurbPhaseName(ERammsMebotCurbPhase P)
	{
		return StaticEnum<ERammsMebotCurbPhase>()->GetDisplayNameTextByValue(static_cast<int64>(P)).ToString();
	}
} // namespace

URammsMebotCurbController::URammsMebotCurbController()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void URammsMebotCurbController::BeginPlay()
{
	Super::BeginPlay();
	SetTickGroup(TG_PrePhysics);
	Base = GetOwner() ? GetOwner()->FindComponentByClass<URammsRobotBaseComponent>() : nullptr;
	if (Base)
	{
		Base->AddTickPrerequisiteComponent(this); // it sees this frame's commands
	}
	const FName Motors[NumLifts] = { Config.ElevatorLeft, Config.ElevatorRight, Config.TranslatorLeft, Config.TranslatorRight, Config.FrontArm,
		Config.RearLift };
	const float Rates[NumLifts] = { FMath::DegreesToRadians(Config.ElevatorSpeedDegS), FMath::DegreesToRadians(Config.ElevatorSpeedDegS),
		Config.TranslatorSpeedCmS, Config.TranslatorSpeedCmS, FMath::DegreesToRadians(Config.FrontArmSpeedDegS),
		FMath::DegreesToRadians(Config.RearLiftSpeedDegS) };
	for (int32 i = 0; i < NumLifts; ++i)
	{
		Lifts[i].Motor = Motors[i];
		Lifts[i].MaxRate = Rates[i];
	}
	bPowered = Base && !Config.FrontOmniLeft.IsNone() && Base->HasMotor(Config.FrontOmniLeft) && Base->HasMotor(Config.FrontOmniRight);
}

void URammsMebotCurbController::EndPlay(const EEndPlayReason::Type Reason)
{
	if (bModeActive)
	{
		ReleaseWheels();
	}
	Super::EndPlay(Reason);
}

// --- controls ------------------------------------------------------------------------------------

void URammsMebotCurbController::DescribeControls(FRammsControlSurface& OutSurface) const
{
	const FName		  Group("Curb");
	FRammsControlAxis Go;
	Go.Id = MebotCurbNegotiateId();
	Go.Group = Group;
	Go.DisplayName = NSLOCTEXT("Ramms", "MebotCurbGo", "Cross the curb ahead");
	Go.Kind = ERammsControlKind::Action;
	Go.Units = ERammsControlUnits::None;
	Go.bReadback = false;
	Go.Order = 0;
	OutSurface.Add(Go);

	FRammsControlAxis Stop = Go;
	Stop.Id = MebotCurbAbortId();
	Stop.DisplayName = NSLOCTEXT("Ramms", "MebotCurbStop", "Stop curb crossing");
	Stop.Order = 1;
	OutSurface.Add(Stop);

	FRammsControlAxis State;
	State.Id = MebotCurbPhaseId();
	State.Group = Group;
	State.DisplayName = NSLOCTEXT("Ramms", "MebotCurbPhase", "Curb crossing");
	State.Kind = ERammsControlKind::Enum;
	State.Units = ERammsControlUnits::None;
	State.bReadOnly = true;
	State.Order = 2;
	const UEnum* PhaseEnum = StaticEnum<ERammsMebotCurbPhase>();
	for (int32 i = 0; i < PhaseEnum->NumEnums() - 1; ++i)
	{
		State.EnumLabels.Add(PhaseEnum->GetDisplayNameTextByIndex(i));
	}
	State.Range = FVector2D(0.0, FMath::Max(0, State.EnumLabels.Num() - 1));
	OutSurface.Add(State);
}

bool URammsMebotCurbController::ApplyControl(FName Id, float Value)
{
	return false; // actions and a read-only state only
}

bool URammsMebotCurbController::TriggerControl(FName Id)
{
	if (Id == MebotCurbNegotiateId())
	{
		FString Why;
		if (!StartCrossing(Why))
		{
			UE_LOG(LogRammsMebotCurb, Warning, TEXT("%s: curb crossing refused: %s"), *GetNameSafe(GetOwner()), *Why);
			Status = Why;
			return false;
		}
		return true;
	}
	if (Id == MebotCurbAbortId())
	{
		Abort(TEXT("stopped"));
		return true;
	}
	return false;
}

bool URammsMebotCurbController::ReadControl(FName Id, float& OutValue) const
{
	if (Id == MebotCurbPhaseId())
	{
		OutValue = static_cast<float>(Phase);
		return true;
	}
	return false;
}

void URammsMebotCurbController::GetClaimedMotorIds(TArray<FName>& OutIds) const
{
	if (!bModeActive)
	{
		return;
	}
	OutIds.Append({ Config.DriveLeft, Config.DriveRight });
	if (bPowered)
	{
		OutIds.Append({ Config.FrontOmniLeft, Config.FrontOmniRight });
	}
	// The lifts only while a crossing runs: stopped, they are the operator's (the MeBot controller takes them
	// where they stand).
	if (bStartPending || IsCrossing())
	{
		for (const FLift& L : Lifts)
		{
			OutIds.Add(L.Motor);
		}
	}
}

FText URammsMebotCurbController::GetDriveModeDisplayName() const
{
	return NSLOCTEXT("Ramms", "MebotCurbMode", "Curb crossing");
}

void URammsMebotCurbController::SetDriveModeActive(bool bActive)
{
	if (bActive == bModeActive)
	{
		return;
	}
	bModeActive = bActive;
	LastSimTime = -1.0;
	bHandBackWarned = false;
	if (!bActive)
	{
		if (IsCrossing())
		{
			Phase = ERammsMebotCurbPhase::Aborted;
			Status = TEXT("another drive mode was selected");
			UE_LOG(LogRammsMebotCurb, Warning, TEXT("%s: curb crossing aborted: %s"), *GetNameSafe(GetOwner()), *Status);
		}
		bStartPending = false;
		bHoldUntilDown = false;
		ReleaseWheels();
	}
}

bool URammsMebotCurbController::SelectMode(FName ModeId)
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return false;
	}
	if (URammsDriveModeSelector* Selector = Owner->FindComponentByClass<URammsDriveModeSelector>())
	{
		return Selector->SetActiveMode(ModeId);
	}
	// No selector: switch the owner's drive modes directly.
	bool bFound = false;
	for (UActorComponent* C : Owner->GetComponents())
	{
		if (IRammsDriveMode* Mode = Cast<IRammsDriveMode>(C))
		{
			bFound |= Mode->GetDriveModeId() == ModeId;
		}
	}
	if (!bFound)
	{
		return false;
	}
	for (UActorComponent* C : Owner->GetComponents())
	{
		if (IRammsDriveMode* Mode = Cast<IRammsDriveMode>(C))
		{
			if (Mode->GetDriveModeId() != ModeId)
			{
				Mode->SetDriveModeActive(false);
			}
		}
	}
	for (UActorComponent* C : Owner->GetComponents())
	{
		if (IRammsDriveMode* Mode = Cast<IRammsDriveMode>(C))
		{
			if (Mode->GetDriveModeId() == ModeId)
			{
				Mode->SetDriveModeActive(true);
			}
		}
	}
	if (URammsRobotControlSurfaceComponent* Surface = URammsRobotControlSurfaceComponent::FindGoverningSurface(this))
	{
		Surface->RebuildControlSurface();
	}
	return true;
}

// --- starting and stopping -----------------------------------------------------------------------

bool URammsMebotCurbController::StartCrossing(FString& OutReason)
{
	if (IsCrossing() || bStartPending)
	{
		OutReason = TEXT("a crossing is already under way");
		return false;
	}
	if (bModeActive && bHoldUntilDown)
	{
		OutReason = TEXT("the last crossing stopped with the drive wheels up: lower them first");
		return false;
	}
	if (!Base || !Base->HasBackend())
	{
		OutReason = TEXT("no robot base with a backend");
		return false;
	}
	for (const FRammsMebotSupportTable* T : { &Config.DriveTable, &Config.FrontTable, &Config.RearTable })
	{
		if (!MebotCurbTable(*T).IsValid())
		{
			OutReason = TEXT("a support has no height table (Config.*Table)");
			return false;
		}
	}
	for (FName Name : { Config.ChassisBody, Config.DriveWheelLeftBody, Config.DriveWheelRightBody, Config.FrontOmniLeftBody,
			 Config.FrontOmniRightBody, Config.RearOmniLeftBody, Config.RearOmniRightBody })
	{
		if (!Body(Name))
		{
			OutReason = FString::Printf(TEXT("no body component '%s'"), *Name.ToString());
			return false;
		}
	}

	// The curb ahead: the ground along the heading from under the front omni wheels.
	const USceneComponent* Chassis = Body(Config.ChassisBody);
	FVector				   Fwd = Chassis->GetForwardVector();
	Fwd.Z = 0.0;
	if (!Fwd.Normalize())
	{
		OutReason = TEXT("the chassis has no heading");
		return false;
	}
	const FVector		  From = 0.5 * (BodyPos(Config.FrontOmniLeftBody) + BodyPos(Config.FrontOmniRightBody));
	const auto			  SurfaceAt = [this, From, Fwd](float D, float& OutZ) { return SurfaceZ(From + Fwd * D, OutZ); };
	RammsMebotCurb::FStep Found;
	if (!RammsMebotCurb::FindStep(SurfaceAt, 0.0f, Config.ScanAheadCm, 2.0f, Config.MinStepCm, Found, OutReason))
	{
		OutReason = TEXT("no curb ahead: ") + OutReason;
		return false;
	}
	if (FMath::Abs(Found.HeightCm) > Config.MaxStepCm)
	{
		OutReason = FString::Printf(TEXT("the step ahead is %.1f cm, more than %.0f"), Found.HeightCm, Config.MaxStepCm);
		return false;
	}
	const RammsMebotCurb::FSupportTable Drive = MebotCurbTable(Config.DriveTable), Front = MebotCurbTable(Config.FrontTable),
										Rear = MebotCurbTable(Config.RearTable);
	const RammsMebotCurb::FSupportTable* Tables[] = { &Drive, &Front, &Rear };
	float								 Height = 0.0f;
	if (!RammsMebotCurb::CrossingHeight(Tables, Found.HeightCm, Config.ClearanceCm, Config.BellyClearanceCm, Config.FrameMarginCm, Height,
			OutReason))
	{
		return false;
	}

	Heading = Fwd;
	EdgePoint = From + Fwd * Found.EdgeDistCm;
	StepCm = Found.HeightCm;
	bClimb = StepCm > 0.0f;
	NearZ = Found.NearZ;
	FarZ = Found.NearZ + Found.HeightCm;
	CrossingHeightCm = Height;
	Status = FString::Printf(TEXT("%s %.1f cm, %.0f cm ahead; crossing height %.1f cm"), bClimb ? TEXT("up") : TEXT("down"),
		FMath::Abs(StepCm), Found.EdgeDistCm, Height);
	UE_LOG(LogRammsMebotCurb, Log, TEXT("%s: curb crossing: %s%s"), *GetNameSafe(GetOwner()), *Status,
		bPowered ? TEXT(", the front omni wheels powered") : TEXT(""));

	bStartPending = true;
	if (!bModeActive && !SelectMode(GetDriveModeId()))
	{
		bStartPending = false;
		OutReason = TEXT("could not select the curb drive mode");
		return false;
	}
	return true;
}

void URammsMebotCurbController::Abort(const FString& Reason)
{
	if (IsCrossing())
	{
		Finish(true, Reason);
	}
	else if (bStartPending)
	{
		// Not begun: nothing has moved, and with nothing to run the mode hands back.
		bStartPending = false;
		Phase = ERammsMebotCurbPhase::Aborted;
		Status = Reason;
	}
}

void URammsMebotCurbController::Finish(bool bAborted, const FString& Why)
{
	Phase = bAborted ? ERammsMebotCurbPhase::Aborted : ERammsMebotCurbPhase::Done;
	Status = Why;
	// Braked where it stands, it hands back on the next tick: the other mode holds it from there. Stopped with
	// the drive wheels up (lifted and not yet touched down: a wheel a few millimetres over the curb reads as on
	// it), that mode would hold nothing that carries load: it holds on the front omni wheels' hubs until the
	// operator has the drive wheels down.
	bHoldUntilDown = bDriveWheelsUp;
	bHoldContact[0] = bHoldContact[1] = false;
	HoldWheels(bHoldUntilDown);
	// The lifts stay where they are; the rear keeps its trim.
	if (bLevel)
	{
		Lifts[RearLiftLift].Cmd += RearTrimRad;
		RearTrimRad = 0.0f;
		bLevel = false;
	}
	for (FLift& L : Lifts)
	{
		L.Goal = L.Cmd;
	}
	SettleLeft = 0.0f;
	if (bHoldUntilDown)
	{
		Status += TEXT(" (the drive wheels are up: holding until they are down; the lifts are free)");
	}
	UE_LOG(LogRammsMebotCurb, Log, TEXT("%s: curb crossing %s: %s"), *GetNameSafe(GetOwner()), bAborted ? TEXT("aborted") : TEXT("done"), *Status);
}

void URammsMebotCurbController::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!Base || !bModeActive)
	{
		return;
	}
	// The backend's clock when it keeps one (a simulator behind real time is slower than the game); the game's
	// when it does not.
	float		 Dt = DeltaTime;
	const double Now = Base->GetSimulationTime();
	if (Now >= 0.0)
	{
		Dt = LastSimTime < 0.0 ? 0.0f : static_cast<float>(Now - LastSimTime);
		LastSimTime = Now;
		if (Dt < 0.0f)
		{
			if (IsCrossing())
			{
				Finish(true, TEXT("the simulation was reset"));
			}
			return;
		}
	}
	Dt = FMath::Min(Dt, 0.1f);
	if (bStartPending)
	{
		bStartPending = false;
		Begin();
		return;
	}
	if (IsCrossing())
	{
		Step(Dt);
		return;
	}
	// Nothing to run (finished, stopped, or the mode selected by hand): hand back, unless a crossing stopped with
	// the drive wheels up and they are not down yet.
	if (bHoldUntilDown && !DriveWheelsLoaded())
	{
		return;
	}
	bHoldUntilDown = false;
	if (!SelectMode(ReturnModeId) && !bHandBackWarned)
	{
		bHandBackWarned = true;
		UE_LOG(LogRammsMebotCurb, Warning, TEXT("%s: the curb mode cannot hand back to '%s'"), *GetNameSafe(GetOwner()), *ReturnModeId.ToString());
	}
}

// --- the sequence --------------------------------------------------------------------------------

void URammsMebotCurbController::Begin()
{
	for (FLift& L : Lifts)
	{
		L.Cmd = L.Goal = Base->GetMotorValue(L.Motor); // where each stands (rad, cm)
		L.Rate = L.MaxRate;
	}
	RearTrimRad = 0.0f;
	bLevel = true;
	bDriveWheelsUp = false;
	SettleLeft = 0.0f;
	HoldWheels();
	PhaseLog.Reset();
	EnterPhase(ERammsMebotCurbPhase::Level);
}

void URammsMebotCurbController::EnterPhase(ERammsMebotCurbPhase Next)
{
	const float H = CrossingHeightCm, Hs = FMath::Abs(StepCm), Lift = Hs + Config.ClearanceCm - H;
	PhaseLog.Add(FString::Printf(TEXT("%.1fs %s -> %s (pitch %+.1f deg)"), Base ? Base->GetSimulationTime() : 0.0, *MebotCurbPhaseName(Phase),
		*MebotCurbPhaseName(Next), FMath::RadiansToDegrees(PitchRad())));
	UE_LOG(LogRammsMebotCurb, Log, TEXT("%s: curb %s"), *GetNameSafe(GetOwner()), *PhaseLog.Last());
	Phase = Next;
	PhaseSeconds = 0.0f;
	PhaseValue = 0.0f;
	TouchStage = 0;
	bool bOk = true;
	switch (Next)
	{
		case ERammsMebotCurbPhase::Level:
		{
			// Climbing, every support on the road holds the chassis H up; descending, on the upper level, H - step.
			const float B = bClimb ? -H : Hs - H;
			bOk = StartHeights({ { ESupport::Drive, B }, { ESupport::Front, B }, { ESupport::Rear, B } }, 0.5f);
			break;
		}
		case ERammsMebotCurbPhase::Approach:
			DriveAt(Config.ApproachSpeedCmS, true, false);
			break;
		case ERammsMebotCurbPhase::FrontUp:
			bOk = StartHeights({ { ESupport::Front, Lift } }, 0.5f);
			break;
		case ERammsMebotCurbPhase::ToEdge:
			DriveAt(Config.CreepSpeedCmS, true, false);
			break;
		case ERammsMebotCurbPhase::FrontDown:
			StartTouchdown(ESupport::Front, FarZ);
			break;
		case ERammsMebotCurbPhase::DriveUp:
			bDriveWheelsUp = true; // until their touchdown is done
			bOk = StartHeights({ { ESupport::Drive, Lift } }, 0.5f);
			break;
		case ERammsMebotCurbPhase::DriveOver:
		{
			const float Past = bClimb ? 0.65f * Config.DriveWheelRadiusCm : Config.DriveWheelRadiusCm + Config.LandMarginCm;
			PhaseValue = Past;
			if (bPowered)
			{
				// The front omni wheels pull the robot forward while the translators bring the drive wheels aft.
				StartMove({ { TranslatorL, Config.AftCm }, { TranslatorR, Config.AftCm } }, 0.0f);
			}
			else
			{
				const float To = FMath::Min(TranslatorReachCm(), Lifts[TranslatorL].Cmd + (Past - LaggingAlong(ESupport::Drive)));
				StartMove({ { TranslatorL, To }, { TranslatorR, To } }, 0.5f);
			}
			break;
		}
		case ERammsMebotCurbPhase::DriveDown:
			HoldWheels(); // the front omni wheels freewheel again
			StartTouchdown(ESupport::Drive, FarZ);
			break;
		case ERammsMebotCurbPhase::PullAft:
			HoldWheels(); // braked and loaded, the drive wheels hold: the chassis moves over them
			StartMove({ { TranslatorL, Config.AftCm }, { TranslatorR, Config.AftCm } }, 1.0f);
			break;
		case ERammsMebotCurbPhase::RearUp:
		{
			// The rule, before the rear omni wheels come up: each drive wheel past the edge, over the far level and
			// down on it.
			RammsMebotCurb::FRearLiftCheck Check;
			const FName					   Wheels[2] = { Config.DriveWheelLeftBody, Config.DriveWheelRightBody };
			for (int32 i = 0; i < 2; ++i)
			{
				const FVector Hub = BodyPos(Wheels[i]);
				float		  Ground = 0.0f;
				Check.PastEdgeCm[i] = Along(Hub);
				Check.bGroundFound[i] = SurfaceZ(Hub, Ground);
				Check.GroundCm[i] = Ground - FarZ;
				Check.GapCm[i] = static_cast<float>(Hub.Z) - Config.DriveWheelRadiusCm - FarZ;
			}
			Check.StepCm = StepCm;
			Check.TouchToleranceCm = Config.TouchToleranceCm;
			Check.PreloadCm = Config.TouchPreloadCm;
			Check.TranslatorCm[0] = Base->GetMotorValue(Config.TranslatorLeft);
			Check.TranslatorCm[1] = Base->GetMotorValue(Config.TranslatorRight);
			Check.AftCm = Config.AftCm;
			Check.ComAheadOfAxleCm = ComAheadOfAxleCm();
			Check.MinComMarginCm = Config.RuleComMarginCm;
			FString Why;
			if (!RammsMebotCurb::RearLiftAllowed(Check, Why))
			{
				Finish(true, TEXT("rear-lift rule not met: ") + Why);
				return;
			}
			PhaseLog.Add(FString::Printf(TEXT("        rear-lift rule met: drive wheels down %.1f / %.1f cm past the edge, translators %.1f / %.1f cm, centre of mass %+.1f cm ahead of their axle"),
				Check.PastEdgeCm[0], Check.PastEdgeCm[1], Check.TranslatorCm[0], Check.TranslatorCm[1], Check.ComAheadOfAxleCm));
			UE_LOG(LogRammsMebotCurb, Log, TEXT("%s: curb %s"), *GetNameSafe(GetOwner()), *PhaseLog.Last());
			// The rear leaves the ground: its trim goes into its command.
			Lifts[RearLiftLift].Cmd += RearTrimRad;
			Lifts[RearLiftLift].Goal = Lifts[RearLiftLift].Cmd;
			RearTrimRad = 0.0f;
			bLevel = false;
			bOk = StartHeights({ { ESupport::Rear, Lift } }, 0.5f);
			break;
		}
		case ERammsMebotCurbPhase::RearOver:
			DriveAt(Config.CreepSpeedCmS, true, bPowered);
			break;
		case ERammsMebotCurbPhase::Lower:
		{
			// Past the edge, everything under the frame is over the road: down on the drive and front wheels, so the
			// sprung rear, which reaches least, has less to reach.
			const float Drop = 0.5f * H;
			bOk = StartHeights({ { ESupport::Drive, HeightNow(ESupport::Drive) + Drop }, { ESupport::Front, HeightNow(ESupport::Front) + Drop } }, 0.5f);
			break;
		}
		case ERammsMebotCurbPhase::RearDown:
			StartTouchdown(ESupport::Rear, FarZ);
			break;
		case ERammsMebotCurbPhase::ClearEdge:
			DriveAt(Config.CreepSpeedCmS, true, bPowered);
			break;
		case ERammsMebotCurbPhase::Recentre:
			// The translators back to centre while the drive wheels roll at the plates' speed: the chassis stays put.
			StartMove({ { TranslatorL, 0.0f }, { TranslatorR, 0.0f } }, 0.0f);
			break;
		case ERammsMebotCurbPhase::Restore:
			HoldWheels();
			StartMove({ { ElevatorL, 0.0f }, { ElevatorR, 0.0f }, { FrontArmLift, 0.0f }, { RearLiftLift, 0.0f } }, 1.5f);
			break;
		default:
			break;
	}
	if (!bOk)
	{
		Finish(true, Status);
	}
}

void URammsMebotCurbController::Step(float Dt)
{
	PhaseSeconds += Dt;
	StepLifts(Dt);

	// Driving: the drive wheels (and the powered front omni wheels) at the speed asked; nothing else commanded.
	if (!FMath::IsNearlyZero(DriveCmS))
	{
		if (bDriveWheels)
		{
			const float W = DriveCmS / Config.DriveWheelRadiusCm;
			Base->SetMotorVelocityCommand(Config.DriveLeft, W);
			Base->SetMotorVelocityCommand(Config.DriveRight, W);
		}
		if (bDriveFront && bPowered)
		{
			const float W = DriveCmS / Config.FrontOmniRadiusCm;
			Base->SetMotorVelocityCommand(Config.FrontOmniLeft, W);
			Base->SetMotorVelocityCommand(Config.FrontOmniRight, W);
		}
	}

	const float Hs = FMath::Abs(StepCm);
	const auto	Arrive = [this](bool bThere, ERammsMebotCurbPhase Next) {
		 // A drive to a mark: stop there, settle, then the next phase.
		 if (PhaseValue == 0.0f && bThere)
		 {
			 HoldWheels();
			 SettleLeft = 0.8f;
			 PhaseValue = 1.0f;
		 }
		 if (PhaseValue != 0.0f && !Moving())
		 {
			 EnterPhase(Next);
		 }
	};
	const auto RearGuard = [this]() {
		// While the chassis is pulled forward the rear omni wheels must stay clear of the curb.
		const float Near = SupportAlong(ESupport::Rear) + (bClimb ? Config.RearOmniRadiusCm + 2.0f : 3.0f);
		if (Near > 0.0f)
		{
			Finish(true, TEXT("the rear omni wheels reached the curb while the chassis was pulled forward"));
			return false;
		}
		return true;
	};
	const float TimeLimit = 60.0f;
	if (PhaseSeconds > TimeLimit)
	{
		Finish(true, FString::Printf(TEXT("%s did not finish in %.0f s"), *MebotCurbPhaseName(Phase), TimeLimit));
		return;
	}

	switch (Phase)
	{
		case ERammsMebotCurbPhase::Level:
			if (!Moving())
			{
				EnterPhase(ERammsMebotCurbPhase::Approach);
			}
			break;
		case ERammsMebotCurbPhase::Approach:
			// Climbing: until the front omni wheels are 5 cm short of the face; descending: the drive wheels 7 cm short of the edge.
			if (bClimb)
			{
				Arrive(SupportAlong(ESupport::Front) + Config.FrontOmniRadiusCm >= -5.0f, ERammsMebotCurbPhase::FrontUp);
			}
			else
			{
				Arrive(SupportAlong(ESupport::Drive) >= -7.0f, ERammsMebotCurbPhase::FrontDown);
			}
			break;
		case ERammsMebotCurbPhase::FrontUp:
			if (!Moving())
			{
				EnterPhase(ERammsMebotCurbPhase::ToEdge);
			}
			break;
		case ERammsMebotCurbPhase::ToEdge:
			Arrive(SupportAlong(ESupport::Drive) + Config.DriveWheelRadiusCm >= -3.0f, ERammsMebotCurbPhase::FrontDown);
			break;
		case ERammsMebotCurbPhase::FrontDown:
			if (StepTouchdown(Dt))
			{
				EnterPhase(ERammsMebotCurbPhase::DriveUp);
			}
			break;
		case ERammsMebotCurbPhase::DriveUp:
			if (!Moving())
			{
				EnterPhase(ERammsMebotCurbPhase::DriveOver);
			}
			break;
		case ERammsMebotCurbPhase::DriveOver:
			if (bPowered)
			{
				if (!RearGuard())
				{
					return;
				}
				// The front omni wheels' speed, commanded here: forward until the drive wheels are at their mark,
				// then held at zero (left to freewheel, the robot would coast on: nothing else holds it with the
				// drive wheels up). Stopped there with the translators aft, it settles, then the drive wheels go down.
				const bool	bThere = LaggingAlong(ESupport::Drive) >= PhaseValue;
				const float W = bThere ? 0.0f : Config.CreepSpeedCmS / Config.FrontOmniRadiusCm;
				Base->SetMotorVelocityCommand(Config.FrontOmniLeft, W);
				Base->SetMotorVelocityCommand(Config.FrontOmniRight, W);
				if (TouchStage == 0 && bThere && !Moving())
				{
					TouchStage = 1;
					SettleLeft = 0.8f;
				}
				else if (TouchStage == 1 && !Moving())
				{
					EnterPhase(ERammsMebotCurbPhase::DriveDown);
				}
			}
			else if (!Moving())
			{
				// The translators carried the drive wheels over: down once both hubs are at the mark. Short of it
				// (the chassis gave as they moved), on by the shortfall, as far as the translators reach.
				const float Short = PhaseValue - LaggingAlong(ESupport::Drive);
				const float Reach = TranslatorReachCm();
				if (Short <= 0.5f)
				{
					EnterPhase(ERammsMebotCurbPhase::DriveDown);
				}
				else if (Lifts[TranslatorL].Cmd >= Reach - 0.01f)
				{
					Finish(true, FString::Printf(TEXT("the translators' reach leaves the drive wheels %.1f cm short of their mark"), Short));
				}
				else
				{
					const float To = FMath::Min(Reach, Lifts[TranslatorL].Cmd + Short);
					StartMove({ { TranslatorL, To }, { TranslatorR, To } }, 0.5f);
				}
			}
			break;
		case ERammsMebotCurbPhase::DriveDown:
			if (StepTouchdown(Dt))
			{
				bDriveWheelsUp = false;
				EnterPhase(bPowered ? ERammsMebotCurbPhase::RearUp : ERammsMebotCurbPhase::PullAft);
			}
			break;
		case ERammsMebotCurbPhase::PullAft:
			if (!RearGuard())
			{
				return;
			}
			if (!Moving())
			{
				EnterPhase(ERammsMebotCurbPhase::RearUp);
			}
			break;
		case ERammsMebotCurbPhase::RearUp:
			if (!Moving())
			{
				EnterPhase(ERammsMebotCurbPhase::RearOver);
			}
			break;
		case ERammsMebotCurbPhase::RearOver:
			Arrive(SupportAlong(ESupport::Rear) - Config.RearOmniRadiusCm >= 3.0f, bClimb ? ERammsMebotCurbPhase::RearDown : ERammsMebotCurbPhase::Lower);
			break;
		case ERammsMebotCurbPhase::Lower:
			if (!Moving())
			{
				EnterPhase(ERammsMebotCurbPhase::RearDown);
			}
			break;
		case ERammsMebotCurbPhase::RearDown:
			if (StepTouchdown(Dt))
			{
				bLevel = true;
				EnterPhase(bClimb ? ERammsMebotCurbPhase::Recentre : ERammsMebotCurbPhase::ClearEdge);
			}
			break;
		case ERammsMebotCurbPhase::ClearEdge:
			Arrive(SupportAlong(ESupport::Rear) - Config.RearOmniRadiusCm >= 15.0f, ERammsMebotCurbPhase::Recentre);
			break;
		case ERammsMebotCurbPhase::Recentre:
		{
			const FLift& T = Lifts[TranslatorL];
			const float	 PlateCmS = FMath::IsNearlyEqual(T.Cmd, T.Goal) ? 0.0f : FMath::Sign(T.Goal - T.Cmd) * T.Rate;
			if (FMath::IsNearlyZero(PlateCmS))
			{
				if (!FMath::IsNearlyZero(DriveCmS))
				{
					HoldWheels();
				}
			}
			else if (!FMath::IsNearlyEqual(DriveCmS, PlateCmS))
			{
				DriveAt(PlateCmS, true, false);
			}
			if (!Moving())
			{
				EnterPhase(ERammsMebotCurbPhase::Restore);
			}
			break;
		}
		case ERammsMebotCurbPhase::Restore:
			if (!Moving())
			{
				Finish(false, FString::Printf(TEXT("crossed %s %.1f cm"), bClimb ? TEXT("up") : TEXT("down"), Hs));
			}
			break;
		default:
			break;
	}
}

// --- primitives ----------------------------------------------------------------------------------

void URammsMebotCurbController::StartMove(const TMap<int32, float>& Goals, float SettleSeconds)
{
	float T = 0.0f;
	for (const TPair<int32, float>& G : Goals)
	{
		const FLift& L = Lifts[G.Key];
		T = FMath::Max(T, FMath::Abs(G.Value - L.Cmd) / FMath::Max(L.MaxRate, KINDA_SMALL_NUMBER));
	}
	for (const TPair<int32, float>& G : Goals)
	{
		FLift& L = Lifts[G.Key];
		L.Goal = G.Value;
		L.Rate = T > 0.0f ? FMath::Max(FMath::Abs(G.Value - L.Cmd) / T, KINDA_SMALL_NUMBER) : L.MaxRate;
	}
	SettleLeft = SettleSeconds;
}

bool URammsMebotCurbController::StartHeights(const TArray<TPair<ESupport, float>>& HeightsCm, float SettleSeconds)
{
	TMap<int32, float> Goals;
	for (const TPair<ESupport, float>& H : HeightsCm)
	{
		float Angle = 0.0f;
		if (!AngleFor(H.Key, H.Value, Angle, Status))
		{
			return false;
		}
		for (int32 Index : LiftsOf(H.Key))
		{
			Goals.Add(Index, Angle);
		}
	}
	StartMove(Goals, SettleSeconds);
	return true;
}

bool URammsMebotCurbController::Moving() const
{
	if (SettleLeft > 0.0f)
	{
		return true;
	}
	for (const FLift& L : Lifts)
	{
		if (!FMath::IsNearlyEqual(L.Cmd, L.Goal, 1.0e-5f))
		{
			return true;
		}
	}
	return false;
}

void URammsMebotCurbController::StepLifts(float Dt)
{
	bool bReached = true;
	for (FLift& L : Lifts)
	{
		L.Cmd += FMath::Clamp(L.Goal - L.Cmd, -L.Rate * Dt, L.Rate * Dt);
		bReached &= FMath::IsNearlyEqual(L.Cmd, L.Goal, 1.0e-5f);
	}
	if (bReached && SettleLeft > 0.0f)
	{
		SettleLeft = FMath::Max(0.0f, SettleLeft - Dt);
	}
	if (bLevel)
	{
		// The sprung rear sags as it takes load: its lift trimmed against the pitch (nose up: the rear is low,
		// its wheels go down), inside the motor's range.
		RearTrimRad += Config.LevelGain * PitchRad() * Dt;
		FRammsMotorSpec Spec;
		if (Base->GetMotorSpec(Config.RearLift, Spec) && Spec.ControlRange.X < Spec.ControlRange.Y)
		{
			const float C = Lifts[RearLiftLift].Cmd;
			RearTrimRad = FMath::Clamp(RearTrimRad, Spec.ControlRange.X - C, Spec.ControlRange.Y - C);
		}
	}
	for (int32 i = 0; i < NumLifts; ++i)
	{
		Base->SetMotorCommand(Lifts[i].Motor, Lifts[i].Cmd + (i == RearLiftLift && bLevel ? RearTrimRad : 0.0f));
	}
}

void URammsMebotCurbController::StartTouchdown(ESupport Support, float LevelZ)
{
	TouchSupport = Support;
	TouchLevelZ = LevelZ;
	TouchStage = 0;
	TouchTargetCm = TouchPlanCm = HeightNow(Support) - Gap(Support, LevelZ); // lowered by the measured gap
	if (!StartHeights({ { Support, TouchTargetCm } }, 0.3f))
	{
		Finish(true, Status);
	}
}

bool URammsMebotCurbController::StepTouchdown(float Dt)
{
	if (Moving())
	{
		return false;
	}
	const float G = Gap(TouchSupport, TouchLevelZ);
	if (TouchStage == 0)
	{
		if (G <= Config.TouchToleranceCm)
		{
			// Down: on past contact by the preload, so it takes load.
			TouchStage = 1;
			FString		Why;
			float		Angle = 0.0f;
			const float Target = HeightNow(TouchSupport) - Config.TouchPreloadCm;
			if (AngleFor(TouchSupport, Target, Angle, Why))
			{
				StartHeights({ { TouchSupport, Target } }, 1.0f);
			}
			else
			{
				SettleLeft = 1.0f; // at its reach: as far as it goes
			}
			return false;
		}
		// Not yet: 5 mm further, up to 4 cm past the plan (measured from the plan: the support follows each step).
		TouchTargetCm -= 0.5f;
		if (TouchTargetCm < TouchPlanCm - 4.0f || !StartHeights({ { TouchSupport, TouchTargetCm } }, 0.2f))
		{
			Finish(true, FString::Printf(TEXT("the %s did not touch down (%.1f cm above the level)"), TouchSupport == ESupport::Drive ? TEXT("drive wheels") : TouchSupport == ESupport::Front ? TEXT("front omni wheels")
																																															   : TEXT("rear omni wheels"),
							 G));
		}
		return false;
	}
	return true;
}

void URammsMebotCurbController::DriveAt(float SpeedCmS, bool bDrive, bool bFront)
{
	RammsDrive::ReleaseBrakes(*Base, Braked);
	DriveCmS = SpeedCmS;
	bDriveWheels = bDrive;
	bDriveFront = bFront && bPowered;
	if (!bDrive)
	{
		Base->SetMotorVelocityCommand(Config.DriveLeft, 0.0f);
		Base->SetMotorVelocityCommand(Config.DriveRight, 0.0f);
	}
	if (bPowered && !bDriveFront)
	{
		for (FName M : { Config.FrontOmniLeft, Config.FrontOmniRight })
		{
			Base->ClearMotorVelocityCommand(M);
			Base->SetMotorCommand(M, 0.0f); // freewheel
		}
	}
}

void URammsMebotCurbController::HoldWheels(bool bFrontHubsToo)
{
	if (!Base)
	{
		return;
	}
	DriveCmS = 0.0f;
	bDriveWheels = bDriveFront = false;
	const FName Drive[] = { Config.DriveLeft, Config.DriveRight };
	for (FName M : Drive)
	{
		Base->ClearMotorVelocityCommand(M); // a braked wheel is held by its brake alone
	}
	RammsDrive::EngageBrakes(*Base, Drive, Braked);
	for (FName M : Drive)
	{
		if (!Braked.Contains(M))
		{
			Base->SetMotorVelocityCommand(M, 0.0f); // no brake: its speed loop holds it at rest
		}
	}
	if (bPowered)
	{
		for (FName M : { Config.FrontOmniLeft, Config.FrontOmniRight })
		{
			Base->ClearMotorVelocityCommand(M);
			if (bFrontHubsToo)
			{
				Base->SetMotorVelocityCommand(M, 0.0f);
			}
			else
			{
				Base->SetMotorCommand(M, 0.0f); // freewheel
			}
		}
	}
}

void URammsMebotCurbController::ReleaseWheels()
{
	if (!Base)
	{
		return;
	}
	RammsDrive::ReleaseBrakes(*Base, Braked);
	DriveCmS = 0.0f;
	bDriveWheels = bDriveFront = false;
	TArray<FName> Wheels = { Config.DriveLeft, Config.DriveRight };
	if (bPowered)
	{
		Wheels.Append({ Config.FrontOmniLeft, Config.FrontOmniRight });
	}
	for (FName M : Wheels)
	{
		Base->ClearMotorVelocityCommand(M);
		Base->SetMotorCommand(M, 0.0f);
	}
}

// --- sensing -------------------------------------------------------------------------------------

USceneComponent* URammsMebotCurbController::Body(FName Name) const
{
	if (const AActor* Owner = GetOwner())
	{
		TInlineComponentArray<USceneComponent*> Scene(Owner);
		for (USceneComponent* C : Scene)
		{
			if (C && C->GetFName() == Name)
			{
				return C;
			}
		}
	}
	return nullptr;
}

FVector URammsMebotCurbController::BodyPos(FName Name) const
{
	const USceneComponent* C = Body(Name);
	return C ? C->GetComponentLocation() : FVector::ZeroVector;
}

bool URammsMebotCurbController::SurfaceZ(const FVector& At, float& OutZ) const
{
	const UWorld* World = GetWorld();
	if (!World)
	{
		return false;
	}
	FCollisionQueryParams Params(SCENE_QUERY_STAT(RammsMebotCurb), false, GetOwner());
	FHitResult			  Hit;
	if (!World->LineTraceSingleByChannel(Hit, At + FVector(0, 0, 60), At - FVector(0, 0, 120), ECC_Visibility, Params))
	{
		return false;
	}
	OutZ = Hit.ImpactPoint.Z;
	return true;
}

float URammsMebotCurbController::Along(const FVector& P) const
{
	FVector D = P - EdgePoint;
	D.Z = 0.0;
	return static_cast<float>(D | Heading);
}

float URammsMebotCurbController::SupportAlong(ESupport S) const
{
	switch (S)
	{
		case ESupport::Drive:
			return 0.5f * (Along(BodyPos(Config.DriveWheelLeftBody)) + Along(BodyPos(Config.DriveWheelRightBody)));
		case ESupport::Front:
			return 0.5f * (Along(BodyPos(Config.FrontOmniLeftBody)) + Along(BodyPos(Config.FrontOmniRightBody)));
		default:
			return 0.5f * (Along(BodyPos(Config.RearOmniLeftBody)) + Along(BodyPos(Config.RearOmniRightBody)));
	}
}

float URammsMebotCurbController::LaggingAlong(ESupport S) const
{
	switch (S)
	{
		case ESupport::Drive:
			return FMath::Min(Along(BodyPos(Config.DriveWheelLeftBody)), Along(BodyPos(Config.DriveWheelRightBody)));
		case ESupport::Front:
			return FMath::Min(Along(BodyPos(Config.FrontOmniLeftBody)), Along(BodyPos(Config.FrontOmniRightBody)));
		default:
			return FMath::Min(Along(BodyPos(Config.RearOmniLeftBody)), Along(BodyPos(Config.RearOmniRightBody)));
	}
}

float URammsMebotCurbController::Radius(ESupport S) const
{
	return S == ESupport::Drive ? Config.DriveWheelRadiusCm : S == ESupport::Front ? Config.FrontOmniRadiusCm
																				   : Config.RearOmniRadiusCm;
}

float URammsMebotCurbController::Gap(ESupport S, float LevelZ) const
{
	const FName Left = S == ESupport::Drive ? Config.DriveWheelLeftBody : S == ESupport::Front ? Config.FrontOmniLeftBody
																							   : Config.RearOmniLeftBody;
	const FName Right = S == ESupport::Drive ? Config.DriveWheelRightBody : S == ESupport::Front ? Config.FrontOmniRightBody
																								 : Config.RearOmniRightBody;
	const float Highest = FMath::Max(BodyPos(Left).Z, BodyPos(Right).Z);
	return static_cast<float>(Highest - Radius(S) - LevelZ);
}

const FRammsMebotSupportTable& URammsMebotCurbController::TableOf(ESupport S) const
{
	return S == ESupport::Drive ? Config.DriveTable : S == ESupport::Front ? Config.FrontTable
																		   : Config.RearTable;
}

TArray<int32> URammsMebotCurbController::LiftsOf(ESupport S) const
{
	switch (S)
	{
		case ESupport::Drive:
			return { ElevatorL, ElevatorR };
		case ESupport::Front:
			return { FrontArmLift };
		default:
			return { RearLiftLift };
	}
}

float URammsMebotCurbController::TranslatorReachCm() const
{
	FRammsMotorSpec Spec;
	return Base && Base->GetMotorSpec(Config.TranslatorLeft, Spec) && Spec.ControlRange.X < Spec.ControlRange.Y ? Spec.ControlRange.Y - 1.0f : 39.0f;
}

bool URammsMebotCurbController::IsCrossing() const
{
	return Phase != ERammsMebotCurbPhase::Idle && Phase != ERammsMebotCurbPhase::Done && Phase != ERammsMebotCurbPhase::Aborted;
}

bool URammsMebotCurbController::DriveWheelsLoaded()
{
	// Each on the ground under it and lowered on past where it first met it by half the preload: carrying load,
	// not hovering a few millimetres over it.
	const FName							Wheels[2] = { Config.DriveWheelLeftBody, Config.DriveWheelRightBody };
	const int32							Elevators[2] = { ElevatorL, ElevatorR };
	const RammsMebotCurb::FSupportTable Table = MebotCurbTable(Config.DriveTable);
	bool								bLoaded = true;
	for (int32 i = 0; i < 2; ++i)
	{
		const USceneComponent* Wheel = Body(Wheels[i]);
		float				   Ground = 0.0f;
		if (!Wheel || !SurfaceZ(Wheel->GetComponentLocation(), Ground)
			|| Wheel->GetComponentLocation().Z - Config.DriveWheelRadiusCm - Ground > Config.TouchToleranceCm)
		{
			bHoldContact[i] = false;
			bLoaded = false;
			continue;
		}
		const float Height = Table.HeightAt(Base->GetMotorValue(Lifts[Elevators[i]].Motor));
		if (!bHoldContact[i])
		{
			bHoldContact[i] = true;
			HoldContactCm[i] = Height;
		}
		bLoaded &= Height <= HoldContactCm[i] - 0.5f * Config.TouchPreloadCm;
	}
	return bLoaded;
}

float URammsMebotCurbController::HeightNow(ESupport S) const
{
	float				Angle = 0.0f;
	const TArray<int32> Ids = LiftsOf(S);
	for (int32 Index : Ids)
	{
		Angle += Base->GetMotorValue(Lifts[Index].Motor);
	}
	return MebotCurbTable(TableOf(S)).HeightAt(Angle / Ids.Num());
}

bool URammsMebotCurbController::AngleFor(ESupport S, float HeightCm, float& OutAngle, FString& OutWhy) const
{
	const RammsMebotCurb::FSupportTable T = MebotCurbTable(TableOf(S));
	if (T.AngleFor(HeightCm, OutAngle))
	{
		return true;
	}
	OutWhy = FString::Printf(TEXT("the %s cannot reach %.1f cm (%.1f..%.1f)"),
		S == ESupport::Drive ? TEXT("drive wheels") : S == ESupport::Front ? TEXT("front omni wheels")
																		   : TEXT("rear omni wheels"),
		HeightCm,
		T.MinHeight(), T.MaxHeight());
	return false;
}

float URammsMebotCurbController::PitchRad() const
{
	const USceneComponent* Chassis = Body(Config.ChassisBody);
	return Chassis ? FMath::Asin(FMath::Clamp(static_cast<float>(Chassis->GetForwardVector().Z), -1.0f, 1.0f)) : 0.0f;
}

float URammsMebotCurbController::ComAheadOfAxleCm() const
{
	const USceneComponent* Chassis = Body(Config.ChassisBody);
	if (!Chassis)
	{
		return 0.0f;
	}
	const float Fixed = Along(Chassis->GetComponentTransform().TransformPosition(FVector(Config.FixedComAheadCm, 0.0, 0.0)));
	const float L = Along(BodyPos(Config.DriveWheelLeftBody)), R = Along(BodyPos(Config.DriveWheelRightBody));
	const float Off = Config.DriveAssemblyComFromHubCm;
	return RammsMebotCurb::CentreOfMass(Config.FixedMassKg, Fixed, Config.DriveAssemblyMassKg, L + Off, R + Off) - 0.5f * (L + R);
}
