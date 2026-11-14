// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsRammpGen1CurbController.h"

#include "Curb/RammsCurbAssist.h"
#include "Curb/RammsCurbStick.h"
#include "Curb/RammsRammpGen1Curb.h"
#include "Curb/RammsRammpGen1CurbMeter.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "RammsControlIds.h"
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
	FName RammpGen1CurbStyleId()
	{
		return FName("curb.style");
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
				return TEXT("Translators back");
			case EPhase::Restore:
				return TEXT("Back to the starting pose");
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
	State.Order = 3;
	const UEnum* PhaseEnum = StaticEnum<ERammsRammpGen1CurbPhase>();
	for (int32 i = 0; i < PhaseEnum->NumEnums() - 1; ++i)
	{
		State.EnumLabels.Add(FText::FromString(RammpGen1CurbPhaseLabel(static_cast<ERammsRammpGen1CurbPhase>(PhaseEnum->GetValueByIndex(i)))));
	}
	State.Range = FVector2D(0.0, FMath::Max(0, State.EnumLabels.Num() - 1));
	OutSurface.Add(State);
	OutSurface.Add(RammsCurb::MakeStatusControl(2));

	OutSurface.Add(RammsCurb::MakeAssistControl(Assist.bEnabled, 4));

	// How the next crossing moves: a switch the panel steps (a crossing under way keeps the style it started with).
	FRammsControlAxis StyleAxis;
	StyleAxis.Id = RammpGen1CurbStyleId();
	StyleAxis.Group = Group;
	StyleAxis.DisplayName = NSLOCTEXT("Ramms", "RammpGen1CurbStyle", "Curb crossing style");
	StyleAxis.Kind = ERammsControlKind::Enum;
	StyleAxis.Units = ERammsControlUnits::None;
	StyleAxis.EnumLabels = { NSLOCTEXT("Ramms", "RammpGen1CurbStyleStepped", "Stop and step"),
		NSLOCTEXT("Ramms", "RammpGen1CurbStyleBlended", "Rolling") };
	StyleAxis.Range = FVector2D(0.0, 1.0);
	StyleAxis.Order = 5;
	OutSurface.Add(StyleAxis);

	// The rider's stick (Stick), while a crossing has the robot: the drive controls every drive mode offers, so that
	// the stick a rider holds reaches whichever mode has the robot. Strafe too: pushed sideways is a pause.
	if (bModeActive && Stick.bRequired)
	{
		FRammsControlAxis Forward;
		Forward.Id = RammsControlIds::Drive::Forward();
		Forward.Group = RammsControlIds::Groups::Drive();
		Forward.DisplayName = NSLOCTEXT("Ramms", "RammpGen1CurbStickForward", "Forward");
		Forward.Kind = ERammsControlKind::Continuous;
		Forward.Units = ERammsControlUnits::Normalized;
		Forward.Range = FVector2D(-1.0, 1.0);
		Forward.Order = 0;
		Forward.PairedAxis = RammsControlIds::Drive::Turn();
		OutSurface.Add(Forward);

		FRammsControlAxis Turn = Forward;
		Turn.Id = RammsControlIds::Drive::Turn();
		Turn.DisplayName = NSLOCTEXT("Ramms", "RammpGen1CurbStickTurn", "Turn");
		Turn.Order = 1;
		Turn.PairedAxis = RammsControlIds::Drive::Forward();
		OutSurface.Add(Turn);

		FRammsControlAxis Strafe = Forward;
		Strafe.Id = RammsControlIds::Drive::Strafe();
		Strafe.DisplayName = NSLOCTEXT("Ramms", "RammpGen1CurbStickStrafe", "Strafe");
		Strafe.Order = 2;
		Strafe.PairedAxis = FName();
		OutSurface.Add(Strafe);
	}
}

namespace
{
	/** The stick's component a drive control Id names (X forward, Y turn, Z strafe), else null. */
	template <typename TVector>
	auto RammpGen1StickAxis(TVector& Stick, FName Id) -> decltype(&Stick.X)
	{
		return Id == RammsControlIds::Drive::Forward() ? &Stick.X
			: Id == RammsControlIds::Drive::Turn()	   ? &Stick.Y
			: Id == RammsControlIds::Drive::Strafe()   ? &Stick.Z
													   : nullptr;
	}
} // namespace

bool URammsRammpGen1CurbController::ApplyControl(FName Id, float Value)
{
	if (bModeActive && Stick.bRequired)
	{
		if (double* Axis = RammpGen1StickAxis(StickInput, Id))
		{
			*Axis = FMath::Clamp(static_cast<double>(Value), -1.0, 1.0);
			return true;
		}
	}
	if (Id == RammsCurb::AssistControlId())
	{
		const bool bOn = FMath::RoundToInt(Value) != 0;
		if (bOn != Assist.bEnabled)
		{
			Assist.bEnabled = bOn;
			UE_LOG(LogRammsRammpGen1Curb, Log, TEXT("%s: curb assist %s"), *GetNameSafe(GetOwner()), bOn ? TEXT("on") : TEXT("off"));
		}
		return true;
	}
	if (Id == RammpGen1CurbStyleId())
	{
		const ERammsRammpGen1CurbStyle To = FMath::RoundToInt(Value) != 0 ? ERammsRammpGen1CurbStyle::Blended : ERammsRammpGen1CurbStyle::Stepped;
		if (To != Style)
		{
			Style = To;
			UE_LOG(LogRammsRammpGen1Curb, Log, TEXT("%s: curb crossings %s%s"), *GetNameSafe(GetOwner()),
				To == ERammsRammpGen1CurbStyle::Blended ? TEXT("roll") : TEXT("stop and step"), IsCrossing() ? TEXT(" from the next one") : TEXT(""));
		}
		return true;
	}
	return false;
}

bool URammsRammpGen1CurbController::ReleaseControl(FName Id)
{
	if (bModeActive && Stick.bRequired)
	{
		if (double* Axis = RammpGen1StickAxis(StickInput, Id))
		{
			*Axis = 0.0;
			return true;
		}
	}
	return false;
}

bool URammsRammpGen1CurbController::ReadTarget(FName Id, float& OutTarget) const
{
	// The switches' targets are what they are set to, however they were set (the panel steps from them).
	return (Id == RammsCurb::AssistControlId() || Id == RammpGen1CurbStyleId()) && ReadControl(Id, OutTarget);
}

bool URammsRammpGen1CurbController::TriggerControl(FName Id)
{
	if (Id == RammpGen1CurbNegotiateId())
	{
		FString Why;
		if (!StartCrossing(Why))
		{
			UE_LOG(LogRammsRammpGen1Curb, Warning, TEXT("%s: curb crossing refused: %s"), *GetNameSafe(GetOwner()), *Why);
			Status = StartRefusal = Why;
			return false;
		}
		return true;
	}
	if (Id == RammpGen1CurbAbortId())
	{
		Abort(TEXT("stopped"));
		return true;
	}
	if (Id == RammsCurb::AssistControlId())
	{
		return ApplyControl(Id, Assist.bEnabled ? 0.0f : 1.0f); // a button on the switch flips it
	}
	return false;
}

bool URammsRammpGen1CurbController::ReadControl(FName Id, float& OutValue) const
{
	if (bModeActive && Stick.bRequired)
	{
		if (const double* Axis = RammpGen1StickAxis(StickInput, Id))
		{
			OutValue = static_cast<float>(*Axis);
			return true;
		}
	}
	if (Id == RammpGen1CurbPhaseId())
	{
		OutValue = static_cast<float>(Phase);
		return true;
	}
	if (Id == RammsCurb::AssistControlId())
	{
		OutValue = Assist.bEnabled ? 1.0f : 0.0f;
		return true;
	}
	if (Id == RammpGen1CurbStyleId())
	{
		OutValue = Style == ERammsRammpGen1CurbStyle::Blended ? 1.0f : 0.0f;
		return true;
	}
	if (Id == RammsCurb::StatusControlId())
	{
		ERammsControlTextSeverity Severity = ERammsControlTextSeverity::Info;
		GetStatusLine(Severity);
		OutValue = static_cast<float>(Severity);
		return true;
	}
	return false;
}

bool URammsRammpGen1CurbController::ReadControlText(FName Id, FText& OutText) const
{
	if (Id != RammsCurb::StatusControlId())
	{
		return false;
	}
	ERammsControlTextSeverity Severity = ERammsControlTextSeverity::Info;
	OutText = FText::FromString(GetStatusLine(Severity));
	return true;
}

FString URammsRammpGen1CurbController::GetStatusLine(ERammsControlTextSeverity& OutSeverity) const
{
	using ESeverity = ERammsControlTextSeverity;
	OutSeverity = ESeverity::Info;
	if (bStartPending)
	{
		return TEXT("Starting: ") + Status;
	}
	if (IsCrossing())
	{
		const FString Crossing = FString::Printf(TEXT("%s %.1f cm"), bClimb ? TEXT("up") : TEXT("down"), FMath::Abs(StepCm));
		if (bCancelling)
		{
			OutSeverity = ESeverity::Attention;
			return TEXT("Cancelling: the lifts going back where they were");
		}
		if (bStickPaused)
		{
			OutSeverity = ESeverity::Attention;
			return StickWhy.StartsWith(TEXT("Paused")) ? StickWhy : TEXT("Paused: ") + StickWhy; // the stick's reason may say so itself
		}
		if (!WaitWhy.IsEmpty())
		{
			OutSeverity = ESeverity::Attention;
			return HeldSeconds >= 1.0f
				? FString::Printf(TEXT("Waiting, nothing moving (%.0f of %.0f s): %s"), HeldSeconds, Config.WaitSeconds, *WaitWhy)
				: TEXT("Waiting: ") + WaitWhy;
		}
		if (StalledSeconds >= 1.0f)
		{
			OutSeverity = ESeverity::Attention;
			return FString::Printf(TEXT("Not moving (%.0f of %.0f s): driven at %.0f cm/s in %s"), StalledSeconds, Config.StallSeconds,
				CommandedSpeedCmS(), RammpGen1CurbPhaseLabel(Phase));
		}
		return FString::Printf(TEXT("Crossing %s: %s"), *Crossing, RammpGen1CurbPhaseLabel(Phase));
	}
	if (!StartRefusal.IsEmpty())
	{
		OutSeverity = ESeverity::Attention;
		return TEXT("Not crossing: ") + StartRefusal;
	}
	switch (LastEnd)
	{
		case EEnd::Done:
			return TEXT("Done: ") + Status;
		case EEnd::Cancelled:
			return TEXT("Cancelled: ") + CancelWhy;
		case EEnd::Stopped:
			OutSeverity = ESeverity::Attention;
			return (Status == TEXT("stopped") ? FString(TEXT("Stopped.")) : TEXT("Stopped: ") + Status + TEXT(".")) + RetryHint();
		case EEnd::Faulted:
			OutSeverity = ESeverity::Fault;
			return TEXT("Faulted: ") + Status + TEXT(".") + RetryHint();
		default:
			return Assist.bEnabled ? TEXT("Ready: drive at a curb to cross it") : TEXT("Ready");
	}
}

FString URammsRammpGen1CurbController::RetryHint() const
{
	if (bHoldUntilDown)
	{
		return FString(); // the status already says what it holds for
	}
	if (RetryHere())
	{
		return Assist.bEnabled ? TEXT(" Cross the curb ahead, or let go of the stick and push forward again, to try again.")
							   : TEXT(" Cross the curb ahead to try again.");
	}
	return bRetryable ? FString() : FString(TEXT(" A wheel had gone up for the curb: the lifts are the operator's to finish by hand."));
}

bool URammsRammpGen1CurbController::CrossedAny() const
{
	if (bDriveWheelsUp)
	{
		return true;
	}
	if (bBlended)
	{
		return PairState[static_cast<int32>(ESupport::Drive)] != EPair::Near;
	}
	using EPhase = ERammsRammpGen1CurbPhase;
	return static_cast<uint8>(Phase) >= static_cast<uint8>(EPhase::DriveUp) && Phase != EPhase::Done && Phase != EPhase::Aborted;
}

bool URammsRammpGen1CurbController::RetryHere() const
{
	return bRetryable && Body(Config.ChassisBody) && FVector::Dist2D(BodyPos(Config.ChassisBody), RetryAt) <= 50.0f;
}

float URammsRammpGen1CurbController::CommandedSpeedCmS() const
{
	if (bBlended)
	{
		return BlendV;
	}
	if (FrontCreepCmS > 0.0f)
	{
		return FrontCreepCmS;
	}
	if (bDriveFront)
	{
		return DriveCmS;
	}
	if (bDriveWheels)
	{
		// The drive wheels ride the translators' plates: the chassis goes at their speed less the plates'.
		const FLift& T = Lifts[TranslatorL];
		const float	 PlateCmS = FMath::IsNearlyEqual(T.Cmd, T.Goal, 1.0e-5f) ? 0.0f : FMath::Sign(T.Goal - T.Cmd) * T.Rate;
		return DriveCmS - PlateCmS;
	}
	return 0.0f;
}

bool URammsRammpGen1CurbController::WatchStall(float Dt)
{
	if (Dt <= 0.0f)
	{
		return true;
	}
	const float AlongNow = Along(BodyPos(Config.ChassisBody));
	if (bChassisSpeedValid)
	{
		ChassisSpeedCmS += FMath::Clamp(Dt / 0.25f, 0.0f, 1.0f) * ((AlongNow - ChassisAlong) / Dt - ChassisSpeedCmS);
	}
	ChassisAlong = AlongNow;
	bChassisSpeedValid = true;
	const float Commanded = CommandedSpeedCmS();
	StalledSeconds = Commanded >= 3.0f && ChassisSpeedCmS < 0.5f ? StalledSeconds + Dt : 0.0f;
	if (StalledSeconds < Config.StallSeconds)
	{
		return true;
	}
	Finish(true, FString::Printf(TEXT("stalled in %s: driven at %.0f cm/s, the chassis moved %.1f cm/s for %.1f s (a wheel against the curb, "
									  "the frame on its edge, or the wheels slipping)"),
					 RammpGen1CurbPhaseLabel(Phase), Commanded, ChassisSpeedCmS, StalledSeconds));
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
	// back at their last commands).
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
			Status = StartRefusal = Why;
		}
	}
	if (!bActive)
	{
		if (IsCrossing())
		{
			if (Meter.IsValid())
			{
				Meter->End(SimNow());
			}
			LastEnd = EEnd::Stopped;
			bRetryable = !CrossedAny();
			RetryAt = BodyPos(Config.ChassisBody);
			WaitWhy.Reset();
			Phase = ERammsRammpGen1CurbPhase::Aborted;
			Status = TEXT("another drive mode was selected");
			UE_LOG(LogRammsRammpGen1Curb, Warning, TEXT("%s: curb crossing aborted: %s"), *GetNameSafe(GetOwner()), *Status);
			if (!AssistWatch.IsValid())
			{
				AssistWatch = MakePimpl<RammsCurb::FAssistWatch>();
			}
			AssistWatch->CrossingEnded(false); // however it was begun: no other until the stick is let go
			bAssistStarted = false;
		}
		bStartPending = false;
		bHoldUntilDown = false;
		StickInput = FVector::ZeroVector;
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
	FPlan Plan;
	return CanStart(OutReason) && PlanAhead(Plan, OutReason) && Commit(Plan, OutReason);
}

bool URammsRammpGen1CurbController::CanStart(FString& OutReason) const
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
	return true;
}

bool URammsRammpGen1CurbController::PlanAhead(FPlan& OutPlan, FString& OutReason, float NoFurtherCm) const
{
	// The curb ahead: the ground along the heading from under the front omni wheels, on the robot's centre line and on
	// the lines its drive wheels run on, which give the edge's angle; each must find the same step. An edge that
	// begins further than NoFurtherCm along the centre line is not looked for (curb assist's reach).
	const USceneComponent* Chassis = Body(Config.ChassisBody);
	FVector				   Fwd = Chassis->GetForwardVector();
	Fwd.Z = 0.0;
	if (!Fwd.Normalize())
	{
		OutReason = TEXT("the chassis has no heading");
		return false;
	}
	const FVector Right(-Fwd.Y, Fwd.X, 0.0); // the heading turned 90 deg right (Unreal: Z up, left-handed)
	const FVector FrontHubs = 0.5 * (BodyPos(Config.FrontOmniLeftBody) + BodyPos(Config.FrontOmniRightBody));
	// Trying again where a crossing stopped, the front omni wheels may already be over the edge: look from behind the
	// drive wheels instead, and say how far ahead of the front omni wheels it is as before (negative: behind them).
	float Back = 0.0f;
	if (RetryHere())
	{
		const FVector DriveHubs = 0.5 * (BodyPos(Config.DriveWheelLeftBody) + BodyPos(Config.DriveWheelRightBody));
		Back = FMath::Max(0.0f, static_cast<float>((FrontHubs - DriveHubs) | Fwd) + Config.DriveWheelRadiusCm + 5.0f);
	}
	const FVector From = FrontHubs - Fwd * Back;
	const float	  HalfTrack = FMath::Max(10.0f, 0.5f * FMath::Abs(static_cast<float>((BodyPos(Config.DriveWheelRightBody) - BodyPos(Config.DriveWheelLeftBody)) | Right)));
	const auto	  FindAt = [this, From, Fwd, Right](float Lateral, float ToCm, RammsRammpGen1Curb::FStep& OutStep, FString& OutWhy) {
		   const FVector Line = From + Right * Lateral;
		   const auto	 SurfaceAt = [this, Line, Fwd](float D, float& OutZ) { return SurfaceZ(Line + Fwd * D, OutZ); };
		   return RammsRammpGen1Curb::FindStep(SurfaceAt, 0.0f, ToCm, 2.0f, Config.MinStepCm, OutStep, OutWhy);
	};
	RammsRammpGen1Curb::FStep Found, Left, RightStep;
	if (!FindAt(0.0f, FMath::Min(Config.ScanAheadCm, NoFurtherCm) + Back, Found, OutReason))
	{
		OutReason = TEXT("no curb ahead: ") + OutReason;
		return false;
	}
	FString SideWhy;
	if (!FindAt(-HalfTrack, Config.ScanAheadCm + Back, Left, SideWhy) || !FindAt(HalfTrack, Config.ScanAheadCm + Back, RightStep, SideWhy)
		|| FMath::Abs(Left.HeightCm - Found.HeightCm) > 2.0f || FMath::Abs(RightStep.HeightCm - Found.HeightCm) > 2.0f)
	{
		OutReason = FString::Printf(TEXT("the %.1f cm step %.0f cm ahead does not run straight across the robot's path"), Found.HeightCm,
			Found.EdgeDistCm - Back);
		return false;
	}
	const RammsRammpGen1Curb::FEdgeFrame Edge = RammsRammpGen1Curb::EdgeFrame(Fwd, Left.EdgeDistCm, RightStep.EdgeDistCm, HalfTrack);
	if (FMath::Abs(Edge.OffSquareDeg) > Config.MaxApproachDeg)
	{
		OutReason = FString::Printf(TEXT("the curb ahead is %.0f deg off square (at most %.0f): turn to face it"), FMath::Abs(Edge.OffSquareDeg),
			Config.MaxApproachDeg);
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

	OutPlan.EdgePoint = From + Fwd * Found.EdgeDistCm;
	OutPlan.EdgeNormal = Edge.Normal;
	OutPlan.EdgeAheadCm = FMath::Min3(Found.EdgeDistCm, Left.EdgeDistCm, RightStep.EdgeDistCm) - Back;
	OutPlan.ApproachDeg = Edge.OffSquareDeg;
	OutPlan.StepCm = Found.HeightCm;
	OutPlan.NearZ = Found.NearZ;
	OutPlan.HeightCm = Height;
	return true;
}

bool URammsRammpGen1CurbController::Commit(const FPlan& Plan, FString& OutReason)
{
	StartRefusal.Reset();
	EdgePoint = Plan.EdgePoint;
	EdgeNormal = Plan.EdgeNormal;
	StepCm = Plan.StepCm;
	bClimb = StepCm > 0.0f;
	NearZ = Plan.NearZ;
	FarZ = Plan.NearZ + Plan.StepCm;
	CrossingHeightCm = Plan.HeightCm;
	Status = FString::Printf(TEXT("%s %.1f cm, %.0f cm ahead, %.0f deg off square; crossing height %.1f cm"), bClimb ? TEXT("up") : TEXT("down"),
		FMath::Abs(StepCm), Plan.EdgeAheadCm, FMath::Abs(Plan.ApproachDeg), CrossingHeightCm);
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
		PendingEnd = EEnd::Stopped;
		Finish(true, Reason);
	}
	else if (bStartPending)
	{
		// Not begun: nothing has moved, and with nothing to run the mode hands back.
		bStartPending = false;
		Phase = ERammsRammpGen1CurbPhase::Aborted;
		Status = Reason;
		LastEnd = EEnd::Stopped;
		bRetryable = false;
	}
}

FRammsRammpGen1CurbStats URammsRammpGen1CurbController::GetStats() const
{
	return Meter.IsValid() ? Meter->GetStats() : FRammsRammpGen1CurbStats();
}

double URammsRammpGen1CurbController::SimNow() const
{
	const double Now = Base ? Base->GetSimulationTime() : -1.0;
	return Now >= 0.0 ? Now : (GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0);
}

void URammsRammpGen1CurbController::Finish(bool bAborted, const FString& Why)
{
	// How it ended, and whether a start where it stands begins it again: nothing gone up for the curb yet.
	LastEnd = !bAborted ? EEnd::Done : PendingEnd != EEnd::None ? PendingEnd
		: bCancelling											? EEnd::Cancelled
																: EEnd::Faulted;
	PendingEnd = EEnd::None;
	bRetryable = (LastEnd == EEnd::Faulted || LastEnd == EEnd::Stopped) && !CrossedAny();
	RetryAt = BodyPos(Config.ChassisBody);
	WaitWhy.Reset();
	StartRefusal.Reset(); // a start refused while this one ran says nothing about how it ended
	HeldSeconds = StalledSeconds = 0.0f;
	if (Meter.IsValid() && Meter->IsOn())
	{
		Meter->End(SimNow());
		PhaseLog.Add(TEXT("        ") + Meter->Summary());
		UE_LOG(LogRammsRammpGen1Curb, Log, TEXT("%s: curb %s"), *GetNameSafe(GetOwner()), *PhaseLog.Last());
	}
	Phase = bAborted ? ERammsRammpGen1CurbPhase::Aborted : ERammsRammpGen1CurbPhase::Done;
	Status = Why;
	// One that did not finish, however it was begun: curb assist starts no other until the stick is let go (only for one
	// it began, a held stick would restart one begun by hand at once).
	if (!AssistWatch.IsValid())
	{
		AssistWatch = MakePimpl<RammsCurb::FAssistWatch>();
	}
	if (bAborted || bAssistStarted)
	{
		AssistWatch->CrossingEnded(!bAborted);
	}
	bAssistStarted = bCancelling = bStickPaused = false;
	// Braked where it stands, it hands back on the next tick: the other mode holds it from there. Stopped with
	// the drive wheels up (lifted and not yet touched down: a wheel a few millimetres over the curb reads as on
	// it), that mode would hold nothing that carries load: it holds on the front omni wheels' hubs until the
	// operator has the drive wheels down.
	bHoldUntilDown = bDriveWheelsUp;
	bHoldContact[0] = bHoldContact[1] = false;
	HoldWheels(bHoldUntilDown);
	ArmAxes.Reset(); // the arm is the operator's again once the hold lapses
	// The lifts stay where they are; the rear keeps its trim.
	StopLevelling();
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
	if (!Base)
	{
		return;
	}
	if (!bModeActive)
	{
		StepAssist(DeltaTime); // another drive mode has the robot: curb assist watches for a curb to cross
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
		MeasureTick();
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
	if (!Meter.IsValid())
	{
		Meter = MakePimpl<RammsRammpGen1Curb::FCrossingMeter>();
	}
	Meter->Begin(SimNow());
	bRetryable = false;
	WaitWhy.Reset();
	HeldSeconds = StalledSeconds = FrontCreepCmS = 0.0f;
	bChassisSpeedValid = false;
	for (int32 i = 0; i < NumLifts; ++i)
	{
		// Each lift taken over at its last command (a loaded servo sits short of it: re-commanded where it sags, it
		// would sag further), else where it stands (rad, cm); and that is where a finished crossing puts it back.
		FLift& L = Lifts[i];
		if (!Base->GetMotorCommand(L.Motor, StartCmd[i]))
		{
			StartCmd[i] = Base->GetMotorValue(L.Motor);
		}
		L.Cmd = L.Goal = StartCmd[i];
		L.Rate = L.MaxRate;
	}
	bFrontUp = FrontOffGround();
	RearTrimRad = 0.0f;
	bLevel = true;
	bDriveWheelsUp = false;
	if (!StickGate.IsValid())
	{
		StickGate = MakePimpl<RammsCurb::FStickGate>();
	}
	StickGate->Reset();
	bStickPaused = bLegsMoved = bCancelling = false;
	StickWhy.Reset();
	CancelWhy.Reset();
	CrossingStatus = Status;
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
	if (bFrontUp)
	{
		PhaseLog.Add(TEXT("        the front omni wheels start up: they stay up until they go down over the edge"));
	}
	StowArm();
	// Rolling needs the powered front omni wheels: with the drive wheels up, nothing else keeps the robot moving.
	bBlended = Style == ERammsRammpGen1CurbStyle::Blended && bPowered;
	if (Style == ERammsRammpGen1CurbStyle::Blended && !bPowered)
	{
		PhaseLog.Add(TEXT("        rolling needs the powered front omni wheels: this crossing stops and steps"));
	}
	if (bBlended)
	{
		BeginBlended();
		return;
	}
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
	FrontCreepCmS = 0.0f;
	bool bOk = true;
	switch (Next)
	{
		case ERammsRammpGen1CurbPhase::Level:
		{
			// Climbing, every support on the road holds the chassis H up; descending, on the upper level, H - step.
			// Front omni wheels that started up stay up, rather than being set down only to be lifted again at the
			// curb: where they are, or higher if the crossing needs it -- climbing, clear of the curb's top; descending,
			// of the upper level.
			const float B = bClimb ? -H : Hs - H;
			const float Front = bFrontUp ? FMath::Max(HeightNow(ESupport::Front), bClimb ? Lift : B + Config.ClearanceCm) : B;
			bOk = StartHeights({ { ESupport::Drive, B }, { ESupport::Front, Front }, { ESupport::Rear, B } }, 0.5f);
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
			FString Why;
			bool	bAbove = false;
			if (!CheckRearLift(Why, bAbove))
			{
				Finish(true, TEXT("rear-lift rule not met: ") + Why);
				return;
			}
			// The rear leaves the ground: its trim goes into its command.
			StopLevelling();
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
			// The translators back where they started while the drive wheels roll at the plates' speed: the chassis
			// stays put.
			StartMove({ { TranslatorL, StartCmd[TranslatorL] }, { TranslatorR, StartCmd[TranslatorR] } }, 0.0f);
			break;
		case ERammsRammpGen1CurbPhase::Restore:
			// Every lift back where the crossing found it. The rear's levelling trim goes into its command and out with
			// this move, so the rear ends at its own starting angle and the chassis at the pitch it started with.
			HoldWheels();
			StopLevelling();
			StartMove({ { ElevatorL, StartCmd[ElevatorL] }, { ElevatorR, StartCmd[ElevatorR] }, { FrontArmLift, StartCmd[FrontArmLift] },
						  { RearLiftLift, StartCmd[RearLiftLift] } },
				1.5f);
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
	HoldArm();
	// The rider's stick (Stick): held forward, the crossing goes on; let go or pushed any other way, it pauses where it
	// is, the clock stopped; before any wheel has lifted for the curb, held back it cancels. A cancelled crossing's
	// lifts go back without it.
	bool bPaused = false;
	if (!bCancelling && !StepStick(Dt, bPaused))
	{
		return; // cancelled
	}
	if (bPaused)
	{
		if (!bStickPaused)
		{
			PauseCrossing();
		}
		StepLifts(0.0f); // every lift held where it is
		// Stopped by the rider, not held by anything: no stall, no wait.
		bChassisSpeedValid = false;
		StalledSeconds = HeldSeconds = 0.0f;
		return;
	}
	if (bStickPaused)
	{
		ResumeCrossing();
	}
	bLegsMoved = true;
	PhaseSeconds += Dt;
	StepLifts(Dt);
	if (!WatchStall(Dt))
	{
		return;
	}
	if (bBlended && !bCancelling) // a cancelled crossing's lifts go back as a stepped one's do (Restore)
	{
		StepBlended(Dt);
		return;
	}

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
		const float Near = LeadingAlong(ESupport::Rear) + (bClimb ? Config.RearOmniRadiusCm + 2.0f : 3.0f);
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
				const bool bAtFace = LeadingAlong(ESupport::Front) + Config.FrontOmniRadiusCm >= -5.0f;
				if (!bFrontUp)
				{
					Arrive(bAtFace, ERammsRammpGen1CurbPhase::FrontUp);
				}
				else if (bAtFace)
				{
					EnterPhase(ERammsRammpGen1CurbPhase::ToEdge); // already up: on at the creep, without a stop
				}
			}
			else
			{
				Arrive(LeadingAlong(ESupport::Drive) >= -7.0f, ERammsRammpGen1CurbPhase::FrontDown);
			}
			break;
		case ERammsRammpGen1CurbPhase::FrontUp:
			if (!Moving())
			{
				EnterPhase(ERammsRammpGen1CurbPhase::ToEdge);
			}
			break;
		case ERammsRammpGen1CurbPhase::ToEdge:
			Arrive(LeadingAlong(ESupport::Drive) + Config.DriveWheelRadiusCm >= -3.0f, ERammsRammpGen1CurbPhase::FrontDown);
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
				FrontCreepCmS = bThere ? 0.0f : Config.CreepSpeedCmS;
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
				Meter->MarkRearDown(SimNow());
				bLevel = true;
				EnterPhase(bClimb ? ERammsRammpGen1CurbPhase::Recentre : ERammsRammpGen1CurbPhase::ClearEdge);
			}
			break;
		case ERammsRammpGen1CurbPhase::ClearEdge:
			Arrive(LaggingAlong(ESupport::Rear) - Config.RearOmniRadiusCm >= 15.0f, ERammsRammpGen1CurbPhase::Recentre);
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
				if (bCancelling)
				{
					Finish(true, TEXT("cancelled: ") + CancelWhy);
				}
				else
				{
					Finish(false, FString::Printf(TEXT("crossed %s %.1f cm"), bClimb ? TEXT("up") : TEXT("down"), Hs));
				}
			}
			break;
		default:
			break;
	}
}

// --- the rolling crossing (Style Blended) ---------------------------------------------------------

void URammsRammpGen1CurbController::BeginBlended()
{
	const float H = CrossingHeightCm, Hs = FMath::Abs(StepCm), Lift = Hs + Config.ClearanceCm - H;
	const float B = bClimb ? -H : Hs - H;
	// As the stepped crossing levels, but on the way: front omni wheels that started up stay up, at least as high as the
	// crossing needs. Climbing, ones that started down go up for it now: it rides its drive and rear wheels to the curb,
	// as in the outdoor stance.
	const float Front = bClimb ? (bFrontUp ? FMath::Max(HeightNow(ESupport::Front), Lift) : Lift)
							   : (bFrontUp ? FMath::Max(HeightNow(ESupport::Front), B + Config.ClearanceCm) : B);
	for (EPair& P : PairState)
	{
		P = EPair::Near;
	}
	for (FBlendTouch& T : BlendTouch)
	{
		T = FBlendTouch();
	}
	bRaised = bDriveFree = bPressed = false;
	BlendEnd = 0;
	BlendV = 0.0f;
	BlendFrontAt = BlendRearOn = -1.0;
	NoteBlended(ERammsRammpGen1CurbPhase::Level, TEXT("rolling: to the crossing height on the way"));
	if (!StartHeights({ { ESupport::Drive, B }, { ESupport::Front, Front }, { ESupport::Rear, B } }, 0.0f))
	{
		Finish(true, Status);
	}
}

void URammsRammpGen1CurbController::NoteBlended(ERammsRammpGen1CurbPhase Display, const FString& What)
{
	PhaseLog.Add(FString::Printf(TEXT("%.1fs %s [%s] (pitch %+.1f deg, %.0f cm/s)"), Base ? Base->GetSimulationTime() : 0.0, *What,
		*RammpGen1CurbPhaseName(Display), FMath::RadiansToDegrees(PitchRad()), BlendV));
	UE_LOG(LogRammsRammpGen1Curb, Log, TEXT("%s: curb %s"), *GetNameSafe(GetOwner()), *PhaseLog.Last());
	if (Display != Phase)
	{
		Phase = Display;
		PhaseSeconds = 0.0f;
	}
}

void URammsRammpGen1CurbController::StepBlended(float Dt)
{
	using EPhase = ERammsRammpGen1CurbPhase;
	const float TimeLimit = 60.0f;
	if (PhaseSeconds > TimeLimit)
	{
		Finish(true, FString::Printf(TEXT("%s did not finish in %.0f s (rolling)"), *RammpGen1CurbPhaseName(Phase), TimeLimit));
		return;
	}
	const float H = CrossingHeightCm, Hs = FMath::Abs(StepCm), Lift = Hs + Config.ClearanceCm - H;
	const float UpperZ = FMath::Max(NearZ, FarZ);
	const float RD = Config.DriveWheelRadiusCm, RF = Config.FrontOmniRadiusCm, RR = Config.RearOmniRadiusCm;
	EPair&		PD = PairState[static_cast<int32>(ESupport::Drive)];
	EPair&		PF = PairState[static_cast<int32>(ESupport::Front)];
	EPair&		PR = PairState[static_cast<int32>(ESupport::Rear)];

	bool bTouched[3] = { false, false, false };
	for (int32 i = 0; i < 3; ++i)
	{
		bTouched[i] = StepBlendTouch(static_cast<ESupport>(i));
		if (!IsCrossing())
		{
			return;
		}
	}
	const bool bDriveTouched = bTouched[static_cast<int32>(ESupport::Drive)], bFrontTouched = bTouched[static_cast<int32>(ESupport::Front)],
			   bRearTouched = bTouched[static_cast<int32>(ESupport::Rear)];
	const auto TranslatorsAt = [this](float L, float R) {
		return FMath::IsNearlyEqual(Lifts[TranslatorL].Cmd, L, 1.0e-3f) && FMath::IsNearlyEqual(Lifts[TranslatorR].Cmd, R, 1.0e-3f);
	};

	if (!bRaised && !Moving())
	{
		bRaised = true;
		NoteBlended(EPhase::Approach, TEXT("at the crossing height, rolling on"));
	}
	const float FrontLead = LeadingAlong(ESupport::Front) + RF; // the front omni wheels' foremost point past the edge
	if (BlendFrontAt < 0.0 && FrontLead >= 0.0f)
	{
		BlendFrontAt = Base->GetSimulationTime();
		NoteBlended(EPhase::ToEdge, bClimb ? TEXT("front omni wheels at the curb face") : TEXT("front omni wheels at the edge"));
	}

	// Each pair as it reaches the edge.
	if (bClimb)
	{
		// The front omni wheels: up (they may start up); down on the curb once wholly over it.
		if (PF == EPair::Near && LowGap(ESupport::Front, FarZ) >= Config.BlendClearCm)
		{
			PF = EPair::Up;
		}
		if (PF == EPair::Up && LaggingAlong(ESupport::Front) - RF >= 2.0f)
		{
			StartBlendTouch(ESupport::Front, FarZ);
			PF = EPair::Lowering;
			NoteBlended(EPhase::FrontDown, TEXT("front omni wheels over the curb: down on it"));
		}
		if (bFrontTouched)
		{
			PF = EPair::Far;
			NoteBlended(Phase, TEXT("front omni wheels on the curb"));
		}
		// The drive wheels: up as they near the face, freewheeling (braked, they would grip the road as their arms
		// swing them up and push the chassis back); the powered front omni wheels roll it on. Down once over the curb.
		if (PD == EPair::Near && PF == EPair::Far && LeadingAlong(ESupport::Drive) + RD >= -Config.BlendLeadUpCm)
		{
			bDriveFree = bDriveWheelsUp = true;
			if (!StartHeights({ { ESupport::Drive, Lift } }, 0.0f))
			{
				Finish(true, Status);
				return;
			}
			PD = EPair::Lifting;
			NoteBlended(EPhase::DriveUp, TEXT("drive wheels up"));
		}
		if (PD == EPair::Lifting && LowGap(ESupport::Drive, UpperZ) >= Config.BlendClearCm)
		{
			PD = EPair::Up;
			NoteBlended(Phase, TEXT("drive wheels clear of the curb"));
		}
		if (PD == EPair::Up && LaggingAlong(ESupport::Drive) >= 0.65f * RD)
		{
			bDriveFree = false; // spinning at its speed, they meet the curb at it
			StartBlendTouch(ESupport::Drive, FarZ);
			PD = EPair::Lowering;
			NoteBlended(EPhase::DriveDown, TEXT("drive wheels over the curb: down on it"));
		}
	}
	else
	{
		// The front omni wheels: down to the road once wholly past the edge.
		if (PF == EPair::Near && LaggingAlong(ESupport::Front) - RF >= 2.0f)
		{
			StartBlendTouch(ESupport::Front, FarZ);
			PF = EPair::Lowering;
			NoteBlended(EPhase::FrontDown, TEXT("front omni wheels past the edge: down to the road"));
		}
		if (bFrontTouched)
		{
			PF = EPair::Far;
			NoteBlended(Phase, TEXT("front omni wheels on the road"));
		}
		// The drive wheels: up off the upper level as they near the edge, freewheeling; down as they roll off it,
		// starting early enough that they land where their swing arms and motors, trailing them, clear it.
		if (PD == EPair::Near && PF == EPair::Far && LeadingAlong(ESupport::Drive) >= -Config.BlendLeadDownCm)
		{
			bDriveFree = bDriveWheelsUp = true;
			if (!StartHeights({ { ESupport::Drive, Lift } }, 0.0f))
			{
				Finish(true, Status);
				return;
			}
			PD = EPair::Lifting;
			NoteBlended(EPhase::DriveUp, TEXT("drive wheels up off the upper level"));
		}
		if (PD == EPair::Lifting && LowGap(ESupport::Drive, UpperZ) >= Config.BlendClearCm)
		{
			PD = EPair::Up;
			NoteBlended(Phase, TEXT("drive wheels clear of the upper level"));
		}
		const float Lag = LaggingAlong(ESupport::Drive);
		if (PD == EPair::Up && Lag >= 0.3f * RD && Lag + BlendV * TimeToTouch(ESupport::Drive, FarZ) >= RD + Config.LandMarginCm)
		{
			bDriveFree = false;
			StartBlendTouch(ESupport::Drive, FarZ);
			PD = EPair::Lowering;
			NoteBlended(EPhase::DriveDown, TEXT("drive wheels rolling off the edge: down to the road"));
		}
	}
	if (bDriveTouched)
	{
		if (PD == EPair::Pressing)
		{
			NoteBlended(Phase, TEXT("drive wheels down on the far level again"));
		}
		else
		{
			// Down: the translators draw them aft as it rolls on to the rear's step.
			bDriveWheelsUp = false;
			StartMove({ { TranslatorL, Config.AftCm }, { TranslatorR, Config.AftCm } }, 0.0f);
			NoteBlended(EPhase::PullAft, bClimb ? TEXT("drive wheels on the curb: translators aft (rolling)") : TEXT("drive wheels on the road: translators aft (rolling)"));
		}
		PD = EPair::Far;
	}
	// The rear omni wheels: up once the rule holds; down once past the edge (descending, the chassis down on the drive
	// and front wheels with them: the sprung rear reaches least).
	if (PR == EPair::Near && PD == EPair::Far && TranslatorsAt(Config.AftCm, Config.AftCm))
	{
		RearGateBlended(Lift);
		if (!IsCrossing())
		{
			return;
		}
	}
	if (PR == EPair::Lifting && LowGap(ESupport::Rear, UpperZ) >= Config.BlendClearCm)
	{
		PR = EPair::Up;
		NoteBlended(Phase, bClimb ? TEXT("rear omni wheels clear of the curb") : TEXT("rear omni wheels clear of the upper level"));
	}
	if (PR == EPair::Up && LaggingAlong(ESupport::Rear) - RR >= 3.0f)
	{
		if (!bClimb && !StartHeights({ { ESupport::Drive, HeightNow(ESupport::Drive) + 0.5f * H }, { ESupport::Front, HeightNow(ESupport::Front) + 0.5f * H } }, 0.0f))
		{
			Finish(true, Status);
			return;
		}
		StartBlendTouch(ESupport::Rear, FarZ);
		PR = EPair::Lowering;
		NoteBlended(EPhase::RearDown, bClimb ? TEXT("rear omni wheels over the curb: down on it") : TEXT("rear omni wheels past the edge: down, the chassis down"));
	}
	if (bRearTouched)
	{
		PR = EPair::Far;
		bLevel = true;
		BlendRearOn = Base->GetSimulationTime();
		Meter->MarkRearDown(SimNow());
		BlendEnd = 1;
		StartMove({ { TranslatorL, StartCmd[TranslatorL] }, { TranslatorR, StartCmd[TranslatorR] } }, 0.0f);
		NoteBlended(EPhase::Recentre, TEXT("rear omni wheels down: translators back (rolling)"));
	}
	// Rolling on: the translators back, the rear trimmed level; then every lift back where the crossing found it (the
	// front arm and elevators moved there while the rear is still at its crossing height tip the chassis); then it stops.
	if (BlendEnd == 1 && TranslatorsAt(StartCmd[TranslatorL], StartCmd[TranslatorR]))
	{
		StopLevelling();
		StartMove({ { ElevatorL, StartCmd[ElevatorL] }, { ElevatorR, StartCmd[ElevatorR] }, { FrontArmLift, StartCmd[FrontArmLift] },
					  { RearLiftLift, StartCmd[RearLiftLift] } },
			0.0f);
		BlendEnd = 2;
		NoteBlended(EPhase::Restore, TEXT("translators back: the lifts to the starting pose (rolling)"));
	}
	else if (BlendEnd == 2 && !Moving())
	{
		BlendEnd = 3;
		NoteBlended(Phase, TEXT("in the starting pose: stopping"));
	}

	// Its speed: the crawl (faster on the approach), held short of the edge for a pair reaching it that is not up off
	// the level it is leaving, and short of the curb until the chassis is at its crossing height.
	const float FrontNear = -FrontLead;
	float		Want = FrontNear > Config.BlendNearCm ? Config.ApproachSpeedCmS : Config.CrawlSpeedCmS;
	FString		HoldFor; // what a stop short of the edge holds it for, while one does
	const auto	StopAt = [this, &Want, &HoldFor](float Lead, float At, const TFunctionRef<FString()>& For) {
		 const float Cap = FMath::Max(0.0f, Config.BlendStopGain * (At - Lead));
		 if (Cap < Want)
		 {
			 Want = Cap;
			 if (Cap < 0.5f)
			 {
				 HoldFor = For();
			 }
		 }
	};
	const auto NotUp = [](EPair P) { return P == EPair::Near || P == EPair::Lifting; };
	const auto ToGo = [this](const TCHAR* Who, ESupport S, float LevelZ, const TCHAR* Level) {
		return FString::Printf(TEXT("the %s to clear %s (%.1f cm to go)"), Who, Level, FMath::Max(0.0f, Config.BlendClearCm - LowGap(S, LevelZ)));
	};
	const TCHAR* Upper = bClimb ? TEXT("the curb") : TEXT("the upper level");
	if (!bRaised)
	{
		StopAt(FrontLead, -5.0f, [] { return FString(TEXT("the chassis to reach its crossing height")); });
	}
	if (bClimb)
	{
		if (PF == EPair::Near)
		{
			StopAt(FrontLead, -2.0f, [&] { return ToGo(TEXT("front omni wheels"), ESupport::Front, FarZ, Upper); });
		}
		if (NotUp(PD))
		{
			StopAt(LeadingAlong(ESupport::Drive) + RD, -1.0f, [&] {
				return PD == EPair::Lifting ? ToGo(TEXT("drive wheels"), ESupport::Drive, UpperZ, Upper)
											: FString(TEXT("the front omni wheels to be down on the curb, before the drive wheels go up"));
			});
		}
		if (NotUp(PR))
		{
			StopAt(LeadingAlong(ESupport::Rear) + RR, -3.0f, [&] {
				return PR == EPair::Lifting ? ToGo(TEXT("rear omni wheels"), ESupport::Rear, UpperZ, Upper)
											: FString(TEXT("the drive wheels down on the curb and the translators aft, before the rear omni wheels go up"));
			});
		}
	}
	else
	{
		// On the upper level a pair touches it under its hubs.
		if (NotUp(PD))
		{
			StopAt(LeadingAlong(ESupport::Drive), -2.0f, [&] {
				return PD == EPair::Lifting ? ToGo(TEXT("drive wheels"), ESupport::Drive, UpperZ, Upper)
											: FString(TEXT("the front omni wheels to be down on the road, before the drive wheels go up"));
			});
		}
		if (NotUp(PR))
		{
			StopAt(LeadingAlong(ESupport::Rear), -2.0f, [&] {
				return PR == EPair::Lifting ? ToGo(TEXT("rear omni wheels"), ESupport::Rear, UpperZ, Upper)
											: FString(TEXT("the drive wheels down on the road and the translators aft, before the rear omni wheels go up"));
			});
		}
	}
	if (BlendEnd == 3)
	{
		Want = 0.0f;
		HoldFor.Reset();
	}
	// Held there, the status says what for; held with nothing moving -- no lift going anywhere, so nothing that could
	// bring what it waits for -- for Config.WaitSeconds, it is not coming, and the crossing faults rather than wait out
	// the phase's minute.
	if (!HoldFor.IsEmpty() && BlendV <= 0.5f)
	{
		WaitWhy = HoldFor;
		HeldSeconds = Moving() ? 0.0f : HeldSeconds + Dt;
		if (HeldSeconds >= Config.WaitSeconds)
		{
			Finish(true, FString::Printf(TEXT("held short of the edge %.0f s for %s, with nothing moving"), HeldSeconds, *HoldFor));
			return;
		}
	}
	else
	{
		WaitWhy.Reset();
		HeldSeconds = 0.0f;
	}
	const float MaxDv = Config.BlendAccelCmS2 * Dt;
	BlendV += FMath::Clamp(Want - BlendV, -MaxDv, MaxDv);
	if (BlendEnd == 3 && BlendV <= 0.01f)
	{
		BlendV = 0.0f;
		HoldWheels();
		const double Now = Base->GetSimulationTime();
		PhaseLog.Add(FString::Printf(TEXT("        rolling: the front omni wheels at the %s -> the rear omni wheels down %.1f s, -> stopped %.1f s"),
			bClimb ? TEXT("curb face") : TEXT("edge"), BlendRearOn - BlendFrontAt, Now - BlendFrontAt));
		UE_LOG(LogRammsRammpGen1Curb, Log, TEXT("%s: curb %s"), *GetNameSafe(GetOwner()), *PhaseLog.Last());
		Finish(false, FString::Printf(TEXT("crossed %s %.1f cm"), bClimb ? TEXT("up") : TEXT("down"), Hs));
		return;
	}
	RollAt(BlendV);
}

void URammsRammpGen1CurbController::RearGateBlended(float LiftCm)
{
	FString Why;
	bool	bAbove = false;
	if (CheckRearLift(Why, bAbove))
	{
		StopLevelling(); // the rear leaves the ground: its trim goes into its command
		if (!StartHeights({ { ESupport::Rear, LiftCm } }, 0.0f))
		{
			Finish(true, Status);
			return;
		}
		PairState[static_cast<int32>(ESupport::Rear)] = EPair::Lifting;
		NoteBlended(ERammsRammpGen1CurbPhase::RearUp, bClimb ? TEXT("rear omni wheels up") : TEXT("rear omni wheels up off the upper level"));
	}
	else if (bAbove && !bPressed)
	{
		// Drawn aft under the chassis, the drive wheels carry less (the front omni wheels take it) and can come off the
		// ground: down again, once. A stepped crossing sets them down after the translators are aft, so never needs this.
		bPressed = true;
		StartBlendTouch(ESupport::Drive, FarZ);
		PairState[static_cast<int32>(ESupport::Drive)] = EPair::Pressing;
		NoteBlended(Phase, TEXT("drive wheels down again (") + Why + TEXT(")"));
	}
	else
	{
		Finish(true, TEXT("rear-lift rule not met: ") + Why);
	}
}

void URammsRammpGen1CurbController::RollAt(float SpeedCmS)
{
	if (Braked.Num() > 0)
	{
		RammsDrive::ReleaseBrakes(*Base, Braked);
	}
	DriveCmS = 0.0f; // the stepped drive's: rolling, the wheels are commanded here, every tick
	bDriveWheels = bDriveFront = bFrontHubsHeld = false;
	const FLift& T = Lifts[TranslatorL];
	const float	 PlateCmS = FMath::IsNearlyEqual(T.Cmd, T.Goal, 1.0e-5f) ? 0.0f : FMath::Sign(T.Goal - T.Cmd) * T.Rate;
	for (FName M : { Config.DriveLeft, Config.DriveRight })
	{
		if (bDriveFree)
		{
			Base->ClearMotorVelocityCommand(M);
			Base->SetMotorCommand(M, 0.0f);
		}
		else
		{
			// The chassis's speed plus the plates': the chassis keeps its speed while the translators move.
			Base->SetMotorVelocityCommand(M, (SpeedCmS + PlateCmS) / Config.DriveWheelRadiusCm);
		}
	}
	for (FName M : { Config.FrontOmniLeft, Config.FrontOmniRight })
	{
		Base->SetMotorVelocityCommand(M, SpeedCmS / Config.FrontOmniRadiusCm);
	}
}

void URammsRammpGen1CurbController::StartBlendTouch(ESupport S, float LevelZ)
{
	FBlendTouch& T = BlendTouch[static_cast<int32>(S)];
	T.bActive = true;
	T.bPreload = false;
	T.LevelZ = LevelZ;
	T.PlanCm = HeightNow(S) - Gap(S, LevelZ); // where both its wheels meet the level, as measured
	// One move at the lift's speed, on past that (up to 4 cm, as far as it reaches): StepBlendTouch stops it at contact.
	const RammsRammpGen1Curb::FSupportTable Table = RammpGen1CurbTable(TableOf(S));
	if (!StartHeights({ { S, FMath::Max(Table.MinHeight() + 0.01f, T.PlanCm - 4.0f) } }, 0.0f))
	{
		Finish(true, Status);
	}
}

bool URammsRammpGen1CurbController::StepBlendTouch(ESupport S)
{
	FBlendTouch& T = BlendTouch[static_cast<int32>(S)];
	if (!T.bActive)
	{
		return false;
	}
	bool bArrived = true;
	for (int32 Index : LiftsOf(S))
	{
		bArrived &= FMath::IsNearlyEqual(Lifts[Index].Cmd, Lifts[Index].Goal, 1.0e-5f);
	}
	const TCHAR* Name = S == ESupport::Drive ? TEXT("drive wheels") : S == ESupport::Front ? TEXT("front omni wheels")
																						   : TEXT("rear omni wheels");
	if (!T.bPreload)
	{
		if (Gap(S, T.LevelZ) <= Config.TouchToleranceCm)
		{
			// Both down: on past contact by the preload, from wherever the command is, so it takes load.
			T.bPreload = true;
			const float Target = FMath::Min(HeightNow(S), CmdHeight(S)) - Config.TouchPreloadCm;
			float		Angle = 0.0f;
			FString		Why;
			if (AngleFor(S, Target, Angle, Why))
			{
				StartHeights({ { S, Target } }, 0.0f);
			}
			else
			{
				for (int32 Index : LiftsOf(S))
				{
					Lifts[Index].Goal = Lifts[Index].Cmd; // at its reach: as far as it goes
				}
			}
		}
		else if (bArrived && !Moving()) // nothing else moving either: descending, the chassis comes down with the rear
		{
			Finish(true, FString::Printf(TEXT("the %s did not touch down (%.1f cm above the level)"), Name, Gap(S, T.LevelZ)));
		}
		return false;
	}
	if (!bArrived)
	{
		return false;
	}
	T.bActive = false;
	const float Past = T.PlanCm - HeightNow(S);
	if (Past > 0.2f)
	{
		PhaseLog.Add(FString::Printf(TEXT("        the %s touched down %.1f cm past where they were measured to meet the level"), Name, Past));
	}
	return true;
}

float URammsRammpGen1CurbController::LowGap(ESupport S, float LevelZ) const
{
	const FName Left = S == ESupport::Drive ? Config.DriveWheelLeftBody : S == ESupport::Front ? Config.FrontOmniLeftBody
																							   : Config.RearOmniLeftBody;
	const FName Right = S == ESupport::Drive ? Config.DriveWheelRightBody : S == ESupport::Front ? Config.FrontOmniRightBody
																								 : Config.RearOmniRightBody;
	const float Lowest = FMath::Min(BodyPos(Left).Z, BodyPos(Right).Z);
	return static_cast<float>(Lowest - Radius(S) - LevelZ);
}

float URammsRammpGen1CurbController::CmdHeight(ESupport S) const
{
	float				Angle = 0.0f;
	const TArray<int32> Ids = LiftsOf(S);
	for (int32 Index : Ids)
	{
		Angle += Lifts[Index].Cmd;
	}
	return RammpGen1CurbTable(TableOf(S)).HeightAt(Angle / Ids.Num());
}

float URammsRammpGen1CurbController::TimeToTouch(ESupport S, float LevelZ) const
{
	const RammsRammpGen1Curb::FSupportTable Table = RammpGen1CurbTable(TableOf(S));
	float									To = 0.0f;
	if (!Table.AngleFor(FMath::Max(Table.MinHeight() + 0.01f, HeightNow(S) - Gap(S, LevelZ)), To))
	{
		return 0.0f;
	}
	const TArray<int32> Ids = LiftsOf(S);
	float				From = 0.0f;
	for (int32 Index : Ids)
	{
		From += Lifts[Index].Cmd;
	}
	From /= Ids.Num();
	return FMath::Abs(To - From) / FMath::Max(Lifts[Ids[0]].MaxRate, KINDA_SMALL_NUMBER);
}

// --- primitives ----------------------------------------------------------------------------------

// --- the rider's stick and curb assist ------------------------------------------------------------

bool URammsRammpGen1CurbController::StepStick(float Dt, bool& bOutPaused)
{
	bOutPaused = false;
	if (!Stick.bRequired)
	{
		return true;
	}
	RammsCurb::FStickGate& Gate = *StickGate;
	Gate.Step(Stick, StickInput.X, StickInput.Y, StickInput.Z, Dt);
	// Where the crossing is: nothing moved yet; levelling to the crossing height; at it, driving up to the curb with no
	// wheel lifted for it or over it (front omni wheels that started up are not lifted for it: until they reach the
	// edge, going back to the stance lowers them onto the road they left); or crossing, to the end.
	using EPhase = ERammsRammpGen1CurbPhase;
	RammsCurb::FStickStage Stage;
	Stage.bLegsMoved = bLegsMoved;
	Stage.bLevelled = Phase != EPhase::Level;
	Stage.bCrossing = Phase == EPhase::FrontUp
		|| (Phase == EPhase::ToEdge && (!bFrontUp || LeadingAlong(ESupport::Front) + Config.FrontOmniRadiusCm > 0.0f))
		|| static_cast<uint8>(Phase) >= static_cast<uint8>(EPhase::FrontDown);
	const float Still = 2.0f / Config.DriveWheelRadiusCm; // rad/s: under 2 cm/s
	Stage.bAtRest = FMath::IsNearlyZero(DriveCmS) && FMath::Abs(Base->GetMotorVelocity(Config.DriveLeft)) < Still
		&& FMath::Abs(Base->GetMotorVelocity(Config.DriveRight)) < Still;
	const RammsCurb::FStickDecision Decision = RammsCurb::DecideStick(Stick, Gate, Stage);
	switch (Decision.Action)
	{
		case RammsCurb::EStickAction::Go:
			StickWhy.Reset();
			return true;
		case RammsCurb::EStickAction::Cancel:
			CancelByRider(Decision.Why);
			return false;
		default:
			bOutPaused = true;
			StickWhy = Decision.Why;
			Status = StickWhy;
			return true;
	}
}

void URammsRammpGen1CurbController::PauseCrossing()
{
	// The drive it was making is kept for the resume; the wheels braked (with the drive wheels up, the powered front
	// omni wheels' hubs hold the robot); the lifts are held where they are by Step.
	bStickPaused = true;
	BlendV = 0.0f; // rolling, it pulls away again from rest
	SavedDriveCmS = DriveCmS;
	bSavedDriveWheels = bDriveWheels;
	bSavedDriveFront = bDriveFront;
	bSavedFrontHubs = bFrontHubsHeld;
	HoldWheels(bDriveWheelsUp || bFrontHubsHeld);
	PhaseLog.Add(FString::Printf(TEXT("        paused by the stick in %s: %s"), *RammpGen1CurbPhaseName(Phase), *StickWhy));
	UE_LOG(LogRammsRammpGen1Curb, Log, TEXT("%s: curb %s"), *GetNameSafe(GetOwner()), *PhaseLog.Last());
}

void URammsRammpGen1CurbController::ResumeCrossing()
{
	bStickPaused = false;
	if (!FMath::IsNearlyZero(SavedDriveCmS))
	{
		DriveAt(SavedDriveCmS, bSavedDriveWheels, bSavedDriveFront);
	}
	else
	{
		HoldWheels(bSavedFrontHubs);
	}
	Status = CrossingStatus;
	PhaseLog.Add(FString::Printf(TEXT("        the stick goes on in %s"), *RammpGen1CurbPhaseName(Phase)));
	UE_LOG(LogRammsRammpGen1Curb, Log, TEXT("%s: curb %s"), *GetNameSafe(GetOwner()), *PhaseLog.Last());
}

void URammsRammpGen1CurbController::CancelByRider(const FString& Why)
{
	// No wheel has lifted for the curb: the lifts go back where the crossing found them (the translators have not
	// moved), then it hands back.
	bCancelling = true;
	bStickPaused = false;
	CancelWhy = TEXT("the stick held back before any wheel lifted for the curb: the lifts back where they were");
	PhaseLog.Add(TEXT("        cancelled by the stick: ") + Why);
	UE_LOG(LogRammsRammpGen1Curb, Log, TEXT("%s: curb %s"), *GetNameSafe(GetOwner()), *PhaseLog.Last());
	EnterPhase(ERammsRammpGen1CurbPhase::Restore);
}

float URammsRammpGen1CurbController::RiderForward() const
{
	const URammsRobotControlSurfaceComponent* Surface = URammsRobotControlSurfaceComponent::FindGoverningSurface(this);
	return Surface ? Surface->GetControlValue(RammsControlIds::Drive::Forward()) : 0.0f;
}

void URammsRammpGen1CurbController::StepAssist(float Dt)
{
	if (!AssistWatch.IsValid())
	{
		AssistWatch = MakePimpl<RammsCurb::FAssistWatch>();
	}
	if (!AssistWatch->Step(Assist, Stick, RiderForward(), Dt))
	{
		return;
	}
	// The rider drives forward with curb assist on: a curb this controller can cross, within reach, is crossed. One it
	// cannot is left to the rider (the status says why); none in range, nothing.
	// Only as far ahead as it would start one: the side lines and the crossing's plan are not worked out for a curb
	// further off. A refusal is the status (the debug log has each look's: they change with every look as the robot
	// drives on).
	// Where a crossing stopped with nothing across, pushing forward again tries it again, from where it stands: the
	// curb may be under the robot by now, so neither the reach nor the least distance applies.
	const bool bRetry = RetryHere();
	FString	   Why;
	FPlan	   Plan;
	if (!CanStart(Why) || !PlanAhead(Plan, Why, bRetry ? TNumericLimits<float>::Max() : Assist.TriggerDistanceCm))
	{
		if (!Why.StartsWith(TEXT("no curb ahead")))
		{
			Status = StartRefusal = Why;
			UE_LOG(LogRammsRammpGen1Curb, Verbose, TEXT("%s: curb assist: not crossing the curb ahead: %s"), *GetNameSafe(GetOwner()), *Why);
		}
		else
		{
			StartRefusal.Reset(); // no curb in reach: what stopped the last look no longer stands
		}
		return;
	}
	if (!bRetry && Plan.EdgeAheadCm > Assist.TriggerDistanceCm)
	{
		return;
	}
	if (!bRetry && Plan.EdgeAheadCm < Assist.MinDistanceCm)
	{
		Status = StartRefusal = FString::Printf(TEXT("the curb ahead is %.0f cm away, closer than %.0f: back off to cross it"), Plan.EdgeAheadCm,
			Assist.MinDistanceCm);
		UE_LOG(LogRammsRammpGen1Curb, Verbose, TEXT("%s: curb assist: not crossing the curb ahead: %s"), *GetNameSafe(GetOwner()), *Status);
		return;
	}
	if (!Commit(Plan, Why))
	{
		Status = StartRefusal = Why;
		AssistWatch->CrossingEnded(false);
		return;
	}
	bAssistStarted = true;
	UE_LOG(LogRammsRammpGen1Curb, Log, TEXT("%s: curb assist started the crossing"), *GetNameSafe(GetOwner()));
}

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
	bFrontHubsHeld = false;
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
	bFrontHubsHeld = bPowered && bFrontHubsToo;
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
	bDriveWheels = bDriveFront = bFrontHubsHeld = false;
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
	// Nor what it carries on actors of its own (the rider): a look from behind the drive wheels passes under the
	// rider's feet, which are not the ground.
	TArray<AActor*> Carried;
	GetOwner()->GetAttachedActors(Carried, true, true);
	Params.AddIgnoredActors(Carried);
	FHitResult Hit;
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
	return static_cast<float>(D | EdgeNormal);
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

float URammsRammpGen1CurbController::LeadingAlong(ESupport S) const
{
	switch (S)
	{
		case ESupport::Drive:
			return FMath::Max(Along(BodyPos(Config.DriveWheelLeftBody)), Along(BodyPos(Config.DriveWheelRightBody)));
		case ESupport::Front:
			return FMath::Max(Along(BodyPos(Config.FrontOmniLeftBody)), Along(BodyPos(Config.FrontOmniRightBody)));
		default:
			return FMath::Max(Along(BodyPos(Config.RearOmniLeftBody)), Along(BodyPos(Config.RearOmniRightBody)));
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

bool URammsRammpGen1CurbController::FrontOffGround() const
{
	for (FName Name : { Config.FrontOmniLeftBody, Config.FrontOmniRightBody })
	{
		const FVector Hub = BodyPos(Name);
		float		  Ground = 0.0f;
		if (!SurfaceZ(Hub, Ground) || Hub.Z - Config.FrontOmniRadiusCm - Ground <= Config.TouchToleranceCm)
		{
			return false;
		}
	}
	return true;
}

bool URammsRammpGen1CurbController::CheckRearLift(FString& OutWhy, bool& bOutDriveAbove)
{
	RammsRammpGen1Curb::FRearLiftCheck Check;
	const FName						   Wheels[2] = { Config.DriveWheelLeftBody, Config.DriveWheelRightBody };
	bOutDriveAbove = false;
	for (int32 i = 0; i < 2; ++i)
	{
		const FVector Hub = BodyPos(Wheels[i]);
		float		  Ground = 0.0f;
		Check.PastEdgeCm[i] = Along(Hub);
		Check.bGroundFound[i] = SurfaceZ(Hub, Ground);
		Check.GroundCm[i] = Ground - FarZ;
		Check.GapCm[i] = static_cast<float>(Hub.Z) - Config.DriveWheelRadiusCm - FarZ;
		bOutDriveAbove |= Check.GapCm[i] > Config.TouchToleranceCm;
	}
	Check.StepCm = StepCm;
	Check.TouchToleranceCm = Config.TouchToleranceCm;
	Check.PreloadCm = Config.TouchPreloadCm;
	Check.TranslatorCm[0] = Base->GetMotorValue(Config.TranslatorLeft);
	Check.TranslatorCm[1] = Base->GetMotorValue(Config.TranslatorRight);
	Check.AftCm = Config.AftCm;
	Check.ComAheadOfAxleCm = ComAheadOfAxleCm();
	Check.MinComMarginCm = Config.RuleComMarginCm;
	if (!RammsRammpGen1Curb::RearLiftAllowed(Check, OutWhy))
	{
		return false;
	}
	PhaseLog.Add(FString::Printf(TEXT("        rear-lift rule met: drive wheels down %.1f / %.1f cm past the edge, translators %.1f / %.1f cm, centre of mass %+.1f cm ahead of their axle"),
		Check.PastEdgeCm[0], Check.PastEdgeCm[1], Check.TranslatorCm[0], Check.TranslatorCm[1], Check.ComAheadOfAxleCm));
	UE_LOG(LogRammsRammpGen1Curb, Log, TEXT("%s: curb %s"), *GetNameSafe(GetOwner()), *PhaseLog.Last());
	return true;
}

void URammsRammpGen1CurbController::MeasureTick()
{
	const USceneComponent* Chassis = Body(Config.ChassisBody);
	if (!Meter.IsValid() || !Meter->IsOn() || !Chassis)
	{
		return;
	}
	const FTransform				 T = Chassis->GetComponentTransform();
	RammsRammpGen1Curb::FMeterSample S;
	S.TimeS = SimNow();
	S.ChassisPos = T.GetLocation();
	const FVector F = Chassis->GetForwardVector();
	S.Heading = FVector2D(F.X, F.Y).GetSafeNormal();
	S.PitchDeg = FMath::RadiansToDegrees(PitchRad());
	S.RollDeg = static_cast<float>(T.Rotator().Roll);
	const USceneComponent* Seat = Body(Config.SeatBody);
	S.RiderPos = (Seat ? Seat->GetComponentTransform() : T).TransformPosition(Config.RiderOffsetCm);
	S.Phase = Phase;

	// The wheels on the ground: each hub within the touch tolerance of the surface under it, touching it there.
	const TPair<FName, float> Wheels[] = { { Config.DriveWheelLeftBody, Config.DriveWheelRadiusCm }, { Config.DriveWheelRightBody, Config.DriveWheelRadiusCm },
		{ Config.FrontOmniLeftBody, Config.FrontOmniRadiusCm }, { Config.FrontOmniRightBody, Config.FrontOmniRadiusCm },
		{ Config.RearOmniLeftBody, Config.RearOmniRadiusCm }, { Config.RearOmniRightBody, Config.RearOmniRadiusCm } };
	for (const TPair<FName, float>& W : Wheels)
	{
		const FVector Hub = BodyPos(W.Key);
		float		  Ground = 0.0f;
		if (SurfaceZ(Hub, Ground) && Hub.Z - W.Value - Ground <= Config.TouchToleranceCm)
		{
			S.Contacts.Add(FVector2D(Hub.X, Hub.Y));
		}
	}
	// The centre of mass, from the config's mass model: the chassis-fixed mass and each drive assembly at its hub.
	const FVector Fixed = T.TransformPosition(FVector(Config.FixedComAheadCm, 0.0, 0.0));
	const FVector Off = F * Config.DriveAssemblyComFromHubCm;
	const FVector L = BodyPos(Config.DriveWheelLeftBody) + Off, R = BodyPos(Config.DriveWheelRightBody) + Off;
	const float	  Mass = Config.FixedMassKg + 2.0f * Config.DriveAssemblyMassKg;
	if (Mass > 0.0f)
	{
		const FVector Com = (Config.FixedMassKg * Fixed + Config.DriveAssemblyMassKg * (L + R)) / Mass;
		S.ComXY = FVector2D(Com.X, Com.Y);
	}
	// The frame's underside over the edge: on each side, where its bottom edge crosses the edge line, its height over
	// the upper level.
	const float UpperZ = FMath::Max(NearZ, FarZ);
	for (const float Side : { -Config.FrameHalfWidthCm, Config.FrameHalfWidthCm })
	{
		const FVector Back = T.TransformPosition(FVector(Config.FrameBackCm, Side, Config.FrameBottomCm));
		const FVector Front = T.TransformPosition(FVector(Config.FrameFrontCm, Side, Config.FrameBottomCm));
		const float	  A0 = Along(Back), A1 = Along(Front);
		if ((A0 <= 0.0f) == (A1 <= 0.0f) || FMath::IsNearlyEqual(A0, A1))
		{
			continue;
		}
		const float Clear = static_cast<float>(FMath::Lerp(Back.Z, Front.Z, A0 / (A0 - A1))) - UpperZ;
		if (!S.bFrameOverEdge || Clear < S.FrameClearanceCm)
		{
			S.bFrameOverEdge = true;
			S.FrameClearanceCm = Clear;
		}
	}
	Meter->Add(S);
	if (LeadingAlong(ESupport::Front) + Config.FrontOmniRadiusCm >= 0.0f)
	{
		Meter->MarkFrontAtEdge(S.TimeS);
	}
}

void URammsRammpGen1CurbController::StopLevelling()
{
	if (!bLevel)
	{
		return;
	}
	Lifts[RearLiftLift].Cmd += RearTrimRad;
	Lifts[RearLiftLift].Goal = Lifts[RearLiftLift].Cmd;
	RearTrimRad = 0.0f;
	bLevel = false;
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
