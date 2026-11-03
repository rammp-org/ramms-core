// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRammpGen1CurbController.h"

#include "Curb/RammsRammpGen1Curb.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "RammsDriveHandOver.h"
#include "RammsDriveModeSelector.h"
#include "RammsMotorSpec.h"
#include "RammsRobotBaseComponent.h"
#include "RammsRobotControlSurfaceComponent.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsRammpGen1Curb, Log, All);

namespace
{
	FName RammpGen1CurbNegotiateId()
	{
		return FName("curb.negotiate");
	}
	FName RammpGen1CurbAbortId()
	{
		return FName("curb.abort");
	}
	FName RammpGen1CurbPhaseId()
	{
		return FName("curb.phase");
	}

	RammsRammpGen1Curb::FSupportTable RammpGen1CurbTable(const FRammsRammpGen1SupportTable& T)
	{
		RammsRammpGen1Curb::FSupportTable Out;
		Out.AngleRad = T.AngleRad;
		Out.HeightCm = T.HeightCm;
		return Out;
	}

	/** The phase's label. Its UMETA DisplayName is editor-only metadata: a packaged build would show the raw name
	 *  ("FrontUp") in the status, the phase log and the curb.phase labels. */
	const TCHAR* RammpGen1CurbPhaseLabel(ERammsRammpGen1CurbPhase P)
	{
		using EPhase = ERammsRammpGen1CurbPhase;
		switch (P)
		{
			case EPhase::Idle:
				return TEXT("Idle");
			case EPhase::Level:
				return TEXT("Chassis to crossing height");
			case EPhase::Approach:
				return TEXT("Approach");
			case EPhase::FrontUp:
				return TEXT("Front omni wheels up");
			case EPhase::ToEdge:
				return TEXT("Drive to the edge");
			case EPhase::FrontDown:
				return TEXT("Front omni wheels down");
			case EPhase::DriveUp:
				return TEXT("Drive wheels up");
			case EPhase::DriveOver:
				return TEXT("Drive wheels over the edge");
			case EPhase::DriveDown:
				return TEXT("Drive wheels down");
			case EPhase::PullAft:
				return TEXT("Drive wheels aft");
			case EPhase::RearUp:
				return TEXT("Rear omni wheels up");
			case EPhase::RearOver:
				return TEXT("Rear omni wheels over the edge");
			case EPhase::Lower:
				return TEXT("Chassis down");
			case EPhase::RearDown:
				return TEXT("Rear omni wheels down");
			case EPhase::ClearEdge:
				return TEXT("Clear the edge");
			case EPhase::Recentre:
				return TEXT("Translators to centre");
			case EPhase::Restore:
				return TEXT("Driving pose");
			case EPhase::Done:
				return TEXT("Done");
			case EPhase::Aborted:
				return TEXT("Aborted");
		}
		return TEXT("?");
	}

	FString RammpGen1CurbPhaseName(ERammsRammpGen1CurbPhase P)
	{
		return RammpGen1CurbPhaseLabel(P);
	}
} // namespace

URammsRammpGen1CurbController::URammsRammpGen1CurbController()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void URammsRammpGen1CurbController::BeginPlay()
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

void URammsRammpGen1CurbController::EndPlay(const EEndPlayReason::Type Reason)
{
	if (bModeActive)
	{
		ReleaseWheels();
	}
	Super::EndPlay(Reason);
}

// --- controls ------------------------------------------------------------------------------------

void URammsRammpGen1CurbController::DescribeControls(FRammsControlSurface& OutSurface) const
{
	const FName		  Group("Curb");
	FRammsControlAxis Go;
	Go.Id = RammpGen1CurbNegotiateId();
	Go.Group = Group;
	Go.DisplayName = NSLOCTEXT("Ramms", "RammpGen1CurbGo", "Cross the curb ahead");
	Go.Kind = ERammsControlKind::Action;
	Go.Units = ERammsControlUnits::None;
	Go.bReadback = false;
	Go.Order = 0;
	OutSurface.Add(Go);

	FRammsControlAxis Stop = Go;
	Stop.Id = RammpGen1CurbAbortId();
	Stop.DisplayName = NSLOCTEXT("Ramms", "RammpGen1CurbStop", "Stop curb crossing");
	Stop.Order = 1;
	OutSurface.Add(Stop);

	FRammsControlAxis State;
	State.Id = RammpGen1CurbPhaseId();
	State.Group = Group;
	State.DisplayName = NSLOCTEXT("Ramms", "RammpGen1CurbPhase", "Curb crossing");
	State.Kind = ERammsControlKind::Enum;
	State.Units = ERammsControlUnits::None;
	State.bReadOnly = true;
	State.Order = 2;
	const UEnum* PhaseEnum = StaticEnum<ERammsRammpGen1CurbPhase>();
	for (int32 i = 0; i < PhaseEnum->NumEnums() - 1; ++i)
	{
		State.EnumLabels.Add(FText::FromString(RammpGen1CurbPhaseLabel(static_cast<ERammsRammpGen1CurbPhase>(PhaseEnum->GetValueByIndex(i)))));
	}
	State.Range = FVector2D(0.0, FMath::Max(0, State.EnumLabels.Num() - 1));
	OutSurface.Add(State);
}

bool URammsRammpGen1CurbController::ApplyControl(FName Id, float Value)
{
	return false; // actions and a read-only state only
}

bool URammsRammpGen1CurbController::TriggerControl(FName Id)
{
	if (Id == RammpGen1CurbNegotiateId())
	{
		FString Why;
		if (!StartCrossing(Why))
		{
			UE_LOG(LogRammsRammpGen1Curb, Warning, TEXT("%s: curb crossing refused: %s"), *GetNameSafe(GetOwner()), *Why);
			Status = Why;
			return false;
		}
		return true;
	}
	if (Id == RammpGen1CurbAbortId())
	{
		Abort(TEXT("stopped"));
		return true;
	}
	return false;
}

bool URammsRammpGen1CurbController::ReadControl(FName Id, float& OutValue) const
{
	if (Id == RammpGen1CurbPhaseId())
	{
		OutValue = static_cast<float>(Phase);
		return true;
	}
	return false;
}

void URammsRammpGen1CurbController::GetClaimedMotorIds(TArray<FName>& OutIds) const
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
	// The lifts only while a crossing runs: stopped, they are the operator's (the RAMMP Gen1 controller takes them
	// where they stand).
	if (bStartPending || IsCrossing())
	{
		for (const FLift& L : Lifts)
		{
			OutIds.Add(L.Motor);
		}
	}
}

FText URammsRammpGen1CurbController::GetDriveModeDisplayName() const
{
	return NSLOCTEXT("Ramms", "RammpGen1CurbMode", "Curb crossing");
}

void URammsRammpGen1CurbController::SetDriveModeActive(bool bActive)
{
	if (bActive == bModeActive)
	{
		return;
	}
	bModeActive = bActive;
	LastSimTime = -1.0;
	bHandBackWarned = false;
	if (bActive && !bStartPending && SwitchedToByHand())
	{
		// Selected by hand (the panel, the mode button) rather than by a start: start the crossing it is for. With
		// none to run -- no curb ahead, or one it cannot cross -- it says why and hands straight back.
		FString Why;
		if (!StartCrossing(Why))
		{
			UE_LOG(LogRammsRammpGen1Curb, Warning, TEXT("%s: curb crossing refused: %s"), *GetNameSafe(GetOwner()), *Why);
			Status = Why;
		}
	}
	if (!bActive)
	{
		if (IsCrossing())
		{
			Phase = ERammsRammpGen1CurbPhase::Aborted;
			Status = TEXT("another drive mode was selected");
			UE_LOG(LogRammsRammpGen1Curb, Warning, TEXT("%s: curb crossing aborted: %s"), *GetNameSafe(GetOwner()), *Status);
		}
		bStartPending = false;
		bHoldUntilDown = false;
		ReleaseWheels();
	}
}

bool URammsRammpGen1CurbController::SwitchedToByHand() const
{
	// Mid-switch the selector still reports the mode it is leaving. The initial selection, as the robot is put
	// down, leaves none: that is not an operator asking to cross.
	const AActor*				   Owner = GetOwner();
	const URammsDriveModeSelector* Selector = Owner ? Owner->FindComponentByClass<URammsDriveModeSelector>() : nullptr;
	if (!Selector)
	{
		return false;
	}
	const FName From = Selector->GetActiveModeId();
	return !From.IsNone() && From != GetDriveModeId();
}

bool URammsRammpGen1CurbController::SelectMode(FName ModeId)
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

bool URammsRammpGen1CurbController::StartCrossing(FString& OutReason)
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
	for (const FRammsRammpGen1SupportTable* T : { &Config.DriveTable, &Config.FrontTable, &Config.RearTable })
	{
		if (!RammpGen1CurbTable(*T).IsValid())
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
	const FVector			  From = 0.5 * (BodyPos(Config.FrontOmniLeftBody) + BodyPos(Config.FrontOmniRightBody));
	const auto				  SurfaceAt = [this, From, Fwd](float D, float& OutZ) { return SurfaceZ(From + Fwd * D, OutZ); };
	RammsRammpGen1Curb::FStep Found;
	if (!RammsRammpGen1Curb::FindStep(SurfaceAt, 0.0f, Config.ScanAheadCm, 2.0f, Config.MinStepCm, Found, OutReason))
	{
		OutReason = TEXT("no curb ahead: ") + OutReason;
		return false;
	}
	if (FMath::Abs(Found.HeightCm) > Config.MaxStepCm)
	{
		OutReason = FString::Printf(TEXT("the step ahead is %.1f cm, more than %.0f"), Found.HeightCm, Config.MaxStepCm);
		return false;
	}
	const RammsRammpGen1Curb::FSupportTable Drive = RammpGen1CurbTable(Config.DriveTable), Front = RammpGen1CurbTable(Config.FrontTable),
											Rear = RammpGen1CurbTable(Config.RearTable);
	const RammsRammpGen1Curb::FSupportTable* Tables[] = { &Drive, &Front, &Rear };
	float									 Height = 0.0f;
	if (!RammsRammpGen1Curb::CrossingHeight(Tables, Found.HeightCm, Config.ClearanceCm, Config.BellyClearanceCm, Config.FrameMarginCm, Height,
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
	UE_LOG(LogRammsRammpGen1Curb, Log, TEXT("%s: curb crossing: %s%s"), *GetNameSafe(GetOwner()), *Status,
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

void URammsRammpGen1CurbController::Abort(const FString& Reason)
{
	if (IsCrossing())
	{
		Finish(true, Reason);
	}
	else if (bStartPending)
	{
		// Not begun: nothing has moved, and with nothing to run the mode hands back.
		bStartPending = false;
		Phase = ERammsRammpGen1CurbPhase::Aborted;
		Status = Reason;
	}
}

void URammsRammpGen1CurbController::Finish(bool bAborted, const FString& Why)
{
	Phase = bAborted ? ERammsRammpGen1CurbPhase::Aborted : ERammsRammpGen1CurbPhase::Done;
	Status = Why;
	// Braked where it stands, it hands back on the next tick: the other mode holds it from there. Stopped with
	// the drive wheels up (lifted and not yet touched down: a wheel a few millimetres over the curb reads as on
	// it), that mode would hold nothing that carries load: it holds on the front omni wheels' hubs until the
	// operator has the drive wheels down.
	bHoldUntilDown = bDriveWheelsUp;
	bHoldContact[0] = bHoldContact[1] = false;
	HoldWheels(bHoldUntilDown);
	ArmAxes.Reset(); // the arm is the operator's again once the hold lapses
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
	UE_LOG(LogRammsRammpGen1Curb, Log, TEXT("%s: curb crossing %s: %s"), *GetNameSafe(GetOwner()), bAborted ? TEXT("aborted") : TEXT("done"), *Status);
}

void URammsRammpGen1CurbController::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
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
		UE_LOG(LogRammsRammpGen1Curb, Warning, TEXT("%s: the curb mode cannot hand back to '%s'"), *GetNameSafe(GetOwner()), *ReturnModeId.ToString());
	}
}

// --- the sequence --------------------------------------------------------------------------------

void URammsRammpGen1CurbController::Begin()
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
	// The wheel radii divide the drive speeds (cm/s to rad/s); the editor's ClampMin does not stop a script or a
	// Blueprint setting 0.
	if (Config.DriveWheelRadiusCm <= 0.0f || Config.FrontOmniRadiusCm <= 0.0f)
	{
		Finish(true, TEXT("the curb config's wheel radii must be above 0"));
		return; // the lifts hold where they stand
	}
	HoldWheels();
	PhaseLog.Reset();
	StowArm();
	EnterPhase(ERammsRammpGen1CurbPhase::Level);
}

void URammsRammpGen1CurbController::StowArm()
{
	// The rear-lift rule's mass model has the arm stowed. Where the robot has an arm on its control surface, it is
	// stowed (arm.stow) as the crossing starts and its rate axes (arm.*) held at rest under this controller's source
	// until the end, so no operator input moves it in between.
	ArmAxes.Reset();
	URammsRobotControlSurfaceComponent* Surface = URammsRobotControlSurfaceComponent::FindGoverningSurface(this);
	if (!Surface)
	{
		return;
	}
	bool bStow = false;
	for (const FRammsControlAxis& Axis : Surface->DescribeControlSurface().Axes)
	{
		if (!Axis.Id.ToString().StartsWith(TEXT("arm.")))
		{
			continue;
		}
		if (Axis.Kind == ERammsControlKind::Continuous && !Axis.bReadOnly)
		{
			ArmAxes.Add(Axis.Id);
		}
		bStow |= Axis.Id == FName("arm.stow");
	}
	if (bStow && Surface->TriggerControl(FName("arm.stow"), ERammsControlSource::Autonomy))
	{
		PhaseLog.Add(TEXT("        the arm stowed for the crossing"));
	}
	HoldArm();
}

void URammsRammpGen1CurbController::HoldArm()
{
	if (ArmAxes.Num() == 0)
	{
		return;
	}
	if (URammsRobotControlSurfaceComponent* Surface = URammsRobotControlSurfaceComponent::FindGoverningSurface(this))
	{
		for (const FName Id : ArmAxes)
		{
			Surface->SetControl(Id, 0.0f, ERammsControlSource::Autonomy);
		}
	}
}

void URammsRammpGen1CurbController::EnterPhase(ERammsRammpGen1CurbPhase Next)
{
	const float H = CrossingHeightCm, Hs = FMath::Abs(StepCm), Lift = Hs + Config.ClearanceCm - H;
	PhaseLog.Add(FString::Printf(TEXT("%.1fs %s -> %s (pitch %+.1f deg)"), Base ? Base->GetSimulationTime() : 0.0, *RammpGen1CurbPhaseName(Phase),
		*RammpGen1CurbPhaseName(Next), FMath::RadiansToDegrees(PitchRad())));
	UE_LOG(LogRammsRammpGen1Curb, Log, TEXT("%s: curb %s"), *GetNameSafe(GetOwner()), *PhaseLog.Last());
	Phase = Next;
	PhaseSeconds = 0.0f;
	PhaseValue = 0.0f;
	TouchStage = 0;
	bool bOk = true;
	switch (Next)
	{
		case ERammsRammpGen1CurbPhase::Level:
		{
			// Climbing, every support on the road holds the chassis H up; descending, on the upper level, H - step.
			const float B = bClimb ? -H : Hs - H;
			bOk = StartHeights({ { ESupport::Drive, B }, { ESupport::Front, B }, { ESupport::Rear, B } }, 0.5f);
			break;
		}
		case ERammsRammpGen1CurbPhase::Approach:
			DriveAt(Config.ApproachSpeedCmS, true, false);
			break;
		case ERammsRammpGen1CurbPhase::FrontUp:
			bOk = StartHeights({ { ESupport::Front, Lift } }, 0.5f);
			break;
		case ERammsRammpGen1CurbPhase::ToEdge:
			DriveAt(Config.CreepSpeedCmS, true, false);
			break;
		case ERammsRammpGen1CurbPhase::FrontDown:
			StartTouchdown(ESupport::Front, FarZ);
			break;
		case ERammsRammpGen1CurbPhase::DriveUp:
			bDriveWheelsUp = true; // until their touchdown is done
			// The front omni wheels' hubs hold the robot from here: braked, the drive wheels grip the road as their
			// arms swing them up and push the chassis back, and once they are off it nothing else stops it rolling.
			HoldWheels(true);
			bOk = StartHeights({ { ESupport::Drive, Lift } }, 0.5f);
			break;
		case ERammsRammpGen1CurbPhase::DriveOver:
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
		case ERammsRammpGen1CurbPhase::DriveDown:
			HoldWheels(); // the front omni wheels freewheel again
			StartTouchdown(ESupport::Drive, FarZ);
			break;
		case ERammsRammpGen1CurbPhase::PullAft:
			HoldWheels(); // braked and loaded, the drive wheels hold: the chassis moves over them
			StartMove({ { TranslatorL, Config.AftCm }, { TranslatorR, Config.AftCm } }, 1.0f);
			break;
		case ERammsRammpGen1CurbPhase::RearUp:
		{
			// The rule, before the rear omni wheels come up: each drive wheel past the edge, over the far level and
			// down on it.
			RammsRammpGen1Curb::FRearLiftCheck Check;
			const FName						   Wheels[2] = { Config.DriveWheelLeftBody, Config.DriveWheelRightBody };
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
			if (!RammsRammpGen1Curb::RearLiftAllowed(Check, Why))
			{
				Finish(true, TEXT("rear-lift rule not met: ") + Why);
				return;
			}
			PhaseLog.Add(FString::Printf(TEXT("        rear-lift rule met: drive wheels down %.1f / %.1f cm past the edge, translators %.1f / %.1f cm, centre of mass %+.1f cm ahead of their axle"),
				Check.PastEdgeCm[0], Check.PastEdgeCm[1], Check.TranslatorCm[0], Check.TranslatorCm[1], Check.ComAheadOfAxleCm));
			UE_LOG(LogRammsRammpGen1Curb, Log, TEXT("%s: curb %s"), *GetNameSafe(GetOwner()), *PhaseLog.Last());
			// The rear leaves the ground: its trim goes into its command.
			Lifts[RearLiftLift].Cmd += RearTrimRad;
			Lifts[RearLiftLift].Goal = Lifts[RearLiftLift].Cmd;
			RearTrimRad = 0.0f;
			bLevel = false;
			bOk = StartHeights({ { ESupport::Rear, Lift } }, 0.5f);
			break;
		}
		case ERammsRammpGen1CurbPhase::RearOver:
			DriveAt(Config.CreepSpeedCmS, true, bPowered);
			break;
		case ERammsRammpGen1CurbPhase::Lower:
		{
			// Past the edge, everything under the frame is over the road: down on the drive and front wheels, so the
			// sprung rear, which reaches least, has less to reach.
			const float Drop = 0.5f * H;
			bOk = StartHeights({ { ESupport::Drive, HeightNow(ESupport::Drive) + Drop }, { ESupport::Front, HeightNow(ESupport::Front) + Drop } }, 0.5f);
			break;
		}
		case ERammsRammpGen1CurbPhase::RearDown:
			StartTouchdown(ESupport::Rear, FarZ);
			break;
		case ERammsRammpGen1CurbPhase::ClearEdge:
			DriveAt(Config.CreepSpeedCmS, true, bPowered);
			break;
		case ERammsRammpGen1CurbPhase::Recentre:
			// The translators back to centre while the drive wheels roll at the plates' speed: the chassis stays put.
			StartMove({ { TranslatorL, 0.0f }, { TranslatorR, 0.0f } }, 0.0f);
			break;
		case ERammsRammpGen1CurbPhase::Restore:
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

void URammsRammpGen1CurbController::Step(float Dt)
{
	PhaseSeconds += Dt;
	HoldArm();
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
	const auto	Arrive = [this](bool bThere, ERammsRammpGen1CurbPhase Next) {
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
		Finish(true, FString::Printf(TEXT("%s did not finish in %.0f s"), *RammpGen1CurbPhaseName(Phase), TimeLimit));
		return;
	}

	switch (Phase)
	{
		case ERammsRammpGen1CurbPhase::Level:
			if (!Moving())
			{
				EnterPhase(ERammsRammpGen1CurbPhase::Approach);
			}
			break;
		case ERammsRammpGen1CurbPhase::Approach:
			// Climbing: until the front omni wheels are 5 cm short of the face; descending: the drive wheels 7 cm short of the edge.
			if (bClimb)
			{
				Arrive(SupportAlong(ESupport::Front) + Config.FrontOmniRadiusCm >= -5.0f, ERammsRammpGen1CurbPhase::FrontUp);
			}
			else
			{
				Arrive(SupportAlong(ESupport::Drive) >= -7.0f, ERammsRammpGen1CurbPhase::FrontDown);
			}
			break;
		case ERammsRammpGen1CurbPhase::FrontUp:
			if (!Moving())
			{
				EnterPhase(ERammsRammpGen1CurbPhase::ToEdge);
			}
			break;
		case ERammsRammpGen1CurbPhase::ToEdge:
			Arrive(SupportAlong(ESupport::Drive) + Config.DriveWheelRadiusCm >= -3.0f, ERammsRammpGen1CurbPhase::FrontDown);
			break;
		case ERammsRammpGen1CurbPhase::FrontDown:
			if (StepTouchdown(Dt))
			{
				EnterPhase(ERammsRammpGen1CurbPhase::DriveUp);
			}
			break;
		case ERammsRammpGen1CurbPhase::DriveUp:
			if (!Moving())
			{
				EnterPhase(ERammsRammpGen1CurbPhase::DriveOver);
			}
			break;
		case ERammsRammpGen1CurbPhase::DriveOver:
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
					EnterPhase(ERammsRammpGen1CurbPhase::DriveDown);
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
					EnterPhase(ERammsRammpGen1CurbPhase::DriveDown);
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
		case ERammsRammpGen1CurbPhase::DriveDown:
			if (StepTouchdown(Dt))
			{
				bDriveWheelsUp = false;
				EnterPhase(bPowered ? ERammsRammpGen1CurbPhase::RearUp : ERammsRammpGen1CurbPhase::PullAft);
			}
			break;
		case ERammsRammpGen1CurbPhase::PullAft:
			if (!RearGuard())
			{
				return;
			}
			if (!Moving())
			{
				EnterPhase(ERammsRammpGen1CurbPhase::RearUp);
			}
			break;
		case ERammsRammpGen1CurbPhase::RearUp:
			if (!Moving())
			{
				EnterPhase(ERammsRammpGen1CurbPhase::RearOver);
			}
			break;
		case ERammsRammpGen1CurbPhase::RearOver:
			Arrive(LaggingAlong(ESupport::Rear) - Config.RearOmniRadiusCm >= 3.0f, bClimb ? ERammsRammpGen1CurbPhase::RearDown : ERammsRammpGen1CurbPhase::Lower);
			break;
		case ERammsRammpGen1CurbPhase::Lower:
		{
			// Lowering on the braked drive wheels swings them on their arms, which can roll the robot back: if a rear
			// omni wheel came back over the edge, creep on until both are past it again, braked and settled there,
			// before setting them down. PhaseValue: 0 lowering or creeping, 1 settling where they were past.
			const bool bPast = LaggingAlong(ESupport::Rear) - Config.RearOmniRadiusCm >= 3.0f;
			if (Moving())
			{
				break;
			}
			if (PhaseValue == 0.0f)
			{
				if (bPast)
				{
					HoldWheels();
					SettleLeft = 0.8f;
					PhaseValue = 1.0f;
				}
				else if (FMath::IsNearlyZero(DriveCmS))
				{
					DriveAt(Config.CreepSpeedCmS, true, bPowered);
				}
				break;
			}
			if (!bPast)
			{
				PhaseValue = 0.0f; // rolled back while settling: creep on again
				break;
			}
			EnterPhase(ERammsRammpGen1CurbPhase::RearDown);
			break;
		}
		case ERammsRammpGen1CurbPhase::RearDown:
			if (StepTouchdown(Dt))
			{
				bLevel = true;
				EnterPhase(bClimb ? ERammsRammpGen1CurbPhase::Recentre : ERammsRammpGen1CurbPhase::ClearEdge);
			}
			break;
		case ERammsRammpGen1CurbPhase::ClearEdge:
			Arrive(SupportAlong(ESupport::Rear) - Config.RearOmniRadiusCm >= 15.0f, ERammsRammpGen1CurbPhase::Recentre);
			break;
		case ERammsRammpGen1CurbPhase::Recentre:
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
				EnterPhase(ERammsRammpGen1CurbPhase::Restore);
			}
			break;
		}
		case ERammsRammpGen1CurbPhase::Restore:
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

void URammsRammpGen1CurbController::StartMove(const TMap<int32, float>& Goals, float SettleSeconds)
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

bool URammsRammpGen1CurbController::StartHeights(const TArray<TPair<ESupport, float>>& HeightsCm, float SettleSeconds)
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

bool URammsRammpGen1CurbController::Moving() const
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

void URammsRammpGen1CurbController::StepLifts(float Dt)
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
		// its wheels go down), no faster than the lift can move (not the last move's rate, which a short move
		// leaves near zero) and inside the motor's range.
		const float MaxStep = Lifts[RearLiftLift].MaxRate * Dt;
		RearTrimRad += FMath::Clamp(Config.LevelGain * PitchRad() * Dt, -MaxStep, MaxStep);
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

void URammsRammpGen1CurbController::StartTouchdown(ESupport Support, float LevelZ)
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

bool URammsRammpGen1CurbController::StepTouchdown(float Dt)
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

void URammsRammpGen1CurbController::DriveAt(float SpeedCmS, bool bDrive, bool bFront)
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

void URammsRammpGen1CurbController::HoldWheels(bool bFrontHubsToo)
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

void URammsRammpGen1CurbController::ReleaseWheels()
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

USceneComponent* URammsRammpGen1CurbController::Body(FName Name) const
{
	if (const TWeakObjectPtr<USceneComponent>* Cached = BodyCache.Find(Name))
	{
		if (USceneComponent* C = Cached->Get())
		{
			return C;
		}
	}
	if (const AActor* Owner = GetOwner())
	{
		TInlineComponentArray<USceneComponent*> Scene(Owner);
		for (USceneComponent* C : Scene)
		{
			if (C && C->GetFName() == Name)
			{
				BodyCache.Add(Name, C);
				return C;
			}
		}
	}
	return nullptr;
}

FVector URammsRammpGen1CurbController::BodyPos(FName Name) const
{
	const USceneComponent* C = Body(Name);
	return C ? C->GetComponentLocation() : FVector::ZeroVector;
}

bool URammsRammpGen1CurbController::SurfaceZ(const FVector& At, float& OutZ) const
{
	const UWorld* World = GetWorld();
	if (!World)
	{
		return false;
	}
	FCollisionQueryParams Params(SCENE_QUERY_STAT(RammsRammpGen1Curb), false, GetOwner());
	FHitResult			  Hit;
	if (!World->LineTraceSingleByChannel(Hit, At + FVector(0, 0, 60), At - FVector(0, 0, 120), ECC_Visibility, Params))
	{
		return false;
	}
	OutZ = Hit.ImpactPoint.Z;
	return true;
}

float URammsRammpGen1CurbController::Along(const FVector& P) const
{
	FVector D = P - EdgePoint;
	D.Z = 0.0;
	return static_cast<float>(D | Heading);
}

float URammsRammpGen1CurbController::SupportAlong(ESupport S) const
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

float URammsRammpGen1CurbController::LaggingAlong(ESupport S) const
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

float URammsRammpGen1CurbController::Radius(ESupport S) const
{
	return S == ESupport::Drive ? Config.DriveWheelRadiusCm : S == ESupport::Front ? Config.FrontOmniRadiusCm
																				   : Config.RearOmniRadiusCm;
}

float URammsRammpGen1CurbController::Gap(ESupport S, float LevelZ) const
{
	const FName Left = S == ESupport::Drive ? Config.DriveWheelLeftBody : S == ESupport::Front ? Config.FrontOmniLeftBody
																							   : Config.RearOmniLeftBody;
	const FName Right = S == ESupport::Drive ? Config.DriveWheelRightBody : S == ESupport::Front ? Config.FrontOmniRightBody
																								 : Config.RearOmniRightBody;
	const float Highest = FMath::Max(BodyPos(Left).Z, BodyPos(Right).Z);
	return static_cast<float>(Highest - Radius(S) - LevelZ);
}

const FRammsRammpGen1SupportTable& URammsRammpGen1CurbController::TableOf(ESupport S) const
{
	return S == ESupport::Drive ? Config.DriveTable : S == ESupport::Front ? Config.FrontTable
																		   : Config.RearTable;
}

TArray<int32> URammsRammpGen1CurbController::LiftsOf(ESupport S) const
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

float URammsRammpGen1CurbController::TranslatorReachCm() const
{
	FRammsMotorSpec Spec;
	return Base && Base->GetMotorSpec(Config.TranslatorLeft, Spec) && Spec.ControlRange.X < Spec.ControlRange.Y ? Spec.ControlRange.Y - 1.0f : 39.0f;
}

bool URammsRammpGen1CurbController::IsCrossing() const
{
	return Phase != ERammsRammpGen1CurbPhase::Idle && Phase != ERammsRammpGen1CurbPhase::Done && Phase != ERammsRammpGen1CurbPhase::Aborted;
}

bool URammsRammpGen1CurbController::DriveWheelsLoaded()
{
	// Each on the ground under it and lowered on past where it first met it by half the preload: carrying load,
	// not hovering a few millimetres over it.
	const FName								Wheels[2] = { Config.DriveWheelLeftBody, Config.DriveWheelRightBody };
	const int32								Elevators[2] = { ElevatorL, ElevatorR };
	const RammsRammpGen1Curb::FSupportTable Table = RammpGen1CurbTable(Config.DriveTable);
	bool									bLoaded = true;
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

float URammsRammpGen1CurbController::HeightNow(ESupport S) const
{
	float				Angle = 0.0f;
	const TArray<int32> Ids = LiftsOf(S);
	for (int32 Index : Ids)
	{
		Angle += Base->GetMotorValue(Lifts[Index].Motor);
	}
	return RammpGen1CurbTable(TableOf(S)).HeightAt(Angle / Ids.Num());
}

bool URammsRammpGen1CurbController::AngleFor(ESupport S, float HeightCm, float& OutAngle, FString& OutWhy) const
{
	const RammsRammpGen1Curb::FSupportTable T = RammpGen1CurbTable(TableOf(S));
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

float URammsRammpGen1CurbController::PitchRad() const
{
	const USceneComponent* Chassis = Body(Config.ChassisBody);
	return Chassis ? FMath::Asin(FMath::Clamp(static_cast<float>(Chassis->GetForwardVector().Z), -1.0f, 1.0f)) : 0.0f;
}

float URammsRammpGen1CurbController::ComAheadOfAxleCm() const
{
	const USceneComponent* Chassis = Body(Config.ChassisBody);
	if (!Chassis)
	{
		return 0.0f;
	}
	const float Fixed = Along(Chassis->GetComponentTransform().TransformPosition(FVector(Config.FixedComAheadCm, 0.0, 0.0)));
	const float L = Along(BodyPos(Config.DriveWheelLeftBody)), R = Along(BodyPos(Config.DriveWheelRightBody));
	const float Off = Config.DriveAssemblyComFromHubCm;
	return RammsRammpGen1Curb::CentreOfMass(Config.FixedMassKg, Fixed, Config.DriveAssemblyMassKg, L + Off, R + Off) - 0.5f * (L + R);
}
