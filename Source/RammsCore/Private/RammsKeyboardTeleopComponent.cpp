// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsKeyboardTeleopComponent.h"
#include "RammsRobotBaseComponent.h"
#include "RammsControlIds.h"
#include "RammsRobotControlSurfaceComponent.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"

URammsKeyboardTeleopComponent::URammsKeyboardTeleopComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	// Same group as the consumers; BeginPlay registers this component as their
	// tick prerequisite so the input they read this frame is this frame's keys
	// (peer order inside a tick group is otherwise unspecified).
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

bool URammsKeyboardTeleopComponent::MatchesLinkageFilter(FName ControlId) const
{
	if (Linkage.ControllerNames.Num() == 0)
	{
		return true;
	}
	// Linkage controls are named "linkage.<component>.height", so a filter that
	// used to match component names matches the middle segment.
	const FString Id = ControlId.ToString();
	for (const FName& Name : Linkage.ControllerNames)
	{
		if (Id.Contains(FString::Printf(TEXT(".%s."), *Name.ToString())))
		{
			return true;
		}
	}
	return false;
}

void URammsKeyboardTeleopComponent::BeginPlay()
{
	Super::BeginPlay();

	if (AActor* Owner = GetOwner())
	{
		Base = Owner->FindComponentByClass<URammsRobotBaseComponent>();
		Surface = Owner->FindComponentByClass<URammsRobotControlSurfaceComponent>();

		bool bHasDrive = false;
		if (Surface)
		{
			// Discover what this robot can actually do rather than looking for
			// known controller classes. Linkage controls are per instance
			// ("linkage.<component>.height"), so they are matched by group and
			// kind; the drive axes are well-known Ids.
			const FRammsControlSurface Described = Surface->DescribeControlSurface();
			for (const FRammsControlAxis& Axis : Described.Axes)
			{
				if (Axis.Id == RammsControlIds::Drive::Forward())
				{
					bHasDrive = true;
				}
				if (Axis.Group == RammsControlIds::Groups::Linkage()
					&& Axis.Kind == ERammsControlKind::Position && !Axis.bReadOnly
					&& MatchesLinkageFilter(Axis.Id))
				{
					LinkageControlIds.Add(Axis.Id);
				}
			}
			// Tick before whatever contributes those controls, so a key pressed
			// this frame is applied this frame. Done by component, not by class.
			for (UActorComponent* C : Surface->GetContributorComponents())
			{
				if (C && C != this)
				{
					C->AddTickPrerequisiteComponent(this);
				}
			}
		}
		if (!Surface)
		{
			// Allowed -- a robot need not expose a control surface -- but this
			// component can then drive nothing at all, and silently doing
			// nothing is the worse failure. Motor bindings still work: those
			// go through the robot base directly.
			UE_LOG(LogTemp, Warning,
				TEXT("[KeyboardTeleop] '%s' has no RammsRobotControlSurfaceComponent: drive and linkage keys will do nothing. Add one to the robot to make its controls drivable."),
				*Owner->GetName());
		}
		else if (!bHasDrive && LinkageControlIds.Num() == 0)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[KeyboardTeleop] '%s' has a control surface but it advertises no drive axes and no linkage controls: check the robot's contributors."),
				*Owner->GetName());
		}
		UE_LOG(LogTemp, Log, TEXT("[KeyboardTeleop] '%s': surface=%s drive=%s linkage controls=%d motor bindings=%d base=%s"),
			*Owner->GetName(), Surface ? TEXT("yes") : TEXT("no"), bHasDrive ? TEXT("yes") : TEXT("no"),
			LinkageControlIds.Num(), MotorBindings.Num(), Base ? TEXT("yes") : TEXT("no"));
	}
}

void URammsKeyboardTeleopComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	ReleaseDrive();
	Super::EndPlay(EndPlayReason);
}

void URammsKeyboardTeleopComponent::ReleaseDrive()
{
	// The drive controller latches the last SetDriveInput; without this the
	// robot keeps driving after the player lets go of it (unpossess, drive
	// disabled, end of play). Only our own command is released — the external
	// input path (SetExternalDriveInput) is untouched.
	if (bDriveCommandActive && Surface)
	{
		// The surface refuses a write while a higher-priority source holds the
		// axis, so a false here means the release did not happen -- keep it
		// pending (Tick retries every frame) rather than pretending it did.
		const bool bF = Surface->SetControl(RammsControlIds::Drive::Forward(), 0.0f, ERammsControlSource::Keyboard);
		const bool bT = Surface->SetControl(RammsControlIds::Drive::Turn(), 0.0f, ERammsControlSource::Keyboard);
		if (!bF && !bT)
		{
			return;
		}
		if (bLogCommands)
		{
			UE_LOG(LogTemp, Log, TEXT("[KeyboardTeleop] drive input released"));
		}
	}
	bDriveCommandActive = false;
	LastDrive = FVector2D::ZeroVector;
}

APlayerController* URammsKeyboardTeleopComponent::GetPlayerController() const
{
	const APawn* Pawn = Cast<APawn>(GetOwner());
	return Pawn ? Cast<APlayerController>(Pawn->GetController()) : nullptr;
}

bool URammsKeyboardTeleopComponent::IsPlayerControlled() const
{
	return GetPlayerController() != nullptr;
}

float URammsKeyboardTeleopComponent::Axis(APlayerController* PC, const FKey& Positive, const FKey& Negative) const
{
	float V = 0.0f;
	if (Positive.IsValid() && PC->IsInputKeyDown(Positive))
	{
		V += 1.0f;
	}
	if (Negative.IsValid() && PC->IsInputKeyDown(Negative))
	{
		V -= 1.0f;
	}
	return V;
}

void URammsKeyboardTeleopComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	APlayerController* PC = GetPlayerController();

	// --- Drive -----------------------------------------------------------
	if (PC && bDriveEnabled && Surface)
	{
		// Kept as (turn, forward) to match what this component always sent.
		const FVector2D Input(Axis(PC, TurnRightKey, TurnLeftKey) * TurnScale, Axis(PC, ForwardKey, BackwardKey) * ForwardScale);
		Surface->SetControl(RammsControlIds::Drive::Forward(), Input.Y, ERammsControlSource::Keyboard);
		Surface->SetControl(RammsControlIds::Drive::Turn(), Input.X, ERammsControlSource::Keyboard);
		bDriveCommandActive = true;
		if (bLogCommands && !Input.Equals(LastDrive))
		{
			UE_LOG(LogTemp, Log, TEXT("[KeyboardTeleop] drive input (%.2f, %.2f)"), Input.X, Input.Y);
		}
		LastDrive = Input;
	}
	else
	{
		// Unpossessed or driving disabled: don't leave the last command latched.
		ReleaseDrive();
	}

	if (!PC)
	{
		return;
	}

	// --- 5-bar endpoint height ------------------------------------------
	const float LinkageDir = Axis(PC, Linkage.UpKey, Linkage.DownKey);
	if (LinkageDir != 0.0f)
	{
		const float Dz = LinkageDir * Linkage.RateCmPerSecond * DeltaTime;
		for (const FName& Id : LinkageControlIds)
		{
			float* Target = LinkageTargets.Find(Id);
			if (!Target)
			{
				// Seed from whatever the control currently reports: its own
				// commanded target if it holds one, else the live readback.
				// Only an estimate if the linkage is not closed, which is fine
				// -- a target is adopted only once the control accepts it.
				float Seed = 0.0f;
				if (!Surface->GetControlTarget(Id, Seed))
				{
					Seed = Surface->GetControlValue(Id);
				}
				Target = &LinkageTargets.Add(Id, Seed);
			}
			const float Candidate = *Target + Dz;
			// Only advance the target when the control accepts it (reachable
			// and inside the motors' ranges), so holding a key at a limit does
			// not wind the target off into the unreachable.
			if (Surface->SetControl(Id, Candidate, ERammsControlSource::Keyboard))
			{
				*Target = Candidate;
				if (bLogCommands)
				{
					UE_LOG(LogTemp, Verbose, TEXT("[KeyboardTeleop] %s -> %.2f"), *Id.ToString(), Candidate);
				}
			}
		}
	}

	// --- Position-motor groups -------------------------------------------
	if (Base)
	{
		for (const FRammsMotorKeyBinding& B : MotorBindings)
		{
			const float Dir = Axis(PC, B.IncreaseKey, B.DecreaseKey);
			if (Dir == 0.0f)
			{
				continue;
			}
			for (const FName& Id : B.MotorIds)
			{
				float* Target = MotorTargets.Find(Id);
				if (!Target)
				{
					Target = &MotorTargets.Add(Id, Base->GetMotorValue(Id));
				}
				float			Next = *Target + Dir * B.RatePerSecond * DeltaTime;
				FRammsMotorSpec Spec;
				if (Base->GetMotorSpec(Id, Spec) && Spec.ControlRange.X < Spec.ControlRange.Y)
				{
					Next = FMath::Clamp(Next, static_cast<float>(Spec.ControlRange.X), static_cast<float>(Spec.ControlRange.Y));
				}
				*Target = Next;
				Base->SetMotorCommand(Id, Next);
			}
			if (bLogCommands)
			{
				UE_LOG(LogTemp, Verbose, TEXT("[KeyboardTeleop] %s %s"), *B.Label, Dir > 0 ? TEXT("+") : TEXT("-"));
			}
		}
	}
}
