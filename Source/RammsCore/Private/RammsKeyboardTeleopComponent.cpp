// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsKeyboardTeleopComponent.h"
#include "RammsRobotBaseComponent.h"
#include "RammsDifferentialDriveController.h"
#include "Ramms5BarLinkageController.h"
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

void URammsKeyboardTeleopComponent::BeginPlay()
{
	Super::BeginPlay();

	if (AActor* Owner = GetOwner())
	{
		Base = Owner->FindComponentByClass<URammsRobotBaseComponent>();
		Drive = Owner->FindComponentByClass<URammsDifferentialDriveController>();
		TArray<URamms5BarLinkageController*> Found;
		Owner->GetComponents<URamms5BarLinkageController>(Found);
		for (URamms5BarLinkageController* C : Found)
		{
			if (Linkage.ControllerNames.Num() == 0 || Linkage.ControllerNames.Contains(C->GetFName()))
			{
				Linkages.Add(C);
				C->AddTickPrerequisiteComponent(this);
			}
		}
		if (Drive)
		{
			Drive->AddTickPrerequisiteComponent(this);
		}
		UE_LOG(LogTemp, Log, TEXT("[KeyboardTeleop] '%s': drive=%s linkages=%d motor bindings=%d base=%s"),
			*Owner->GetName(), Drive ? TEXT("yes") : TEXT("no"), Linkages.Num(), MotorBindings.Num(),
			Base ? TEXT("yes") : TEXT("no"));
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
	if (bDriveCommandActive && Drive)
	{
		if (Drive->IsExternalDriveActive())
		{
			// SetDriveInput is ignored while an external command holds
			// priority; keep the release pending (Tick retries every frame)
			// instead of pretending it went through.
			return;
		}
		Drive->SetDriveInput(FVector2D::ZeroVector);
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
	if (PC && bDriveEnabled && Drive)
	{
		const FVector2D Input(Axis(PC, TurnRightKey, TurnLeftKey) * TurnScale, Axis(PC, ForwardKey, BackwardKey) * ForwardScale);
		Drive->SetDriveInput(Input);
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
		for (URamms5BarLinkageController* C : Linkages)
		{
			if (!C)
			{
				continue;
			}
			FVector2D* Target = LinkageTargets.Find(C);
			if (!Target)
			{
				// Seed from the live endpoint. An inconsistent live pose only
				// yields an estimate; that's fine as a seed because a target
				// is only adopted once the linkage accepts it.
				bool			bValid = false;
				const FVector2D Seed = C->GetCurrentEndpointChecked(bValid);
				if (!bValid)
				{
					UE_LOG(LogTemp, Warning, TEXT("[KeyboardTeleop] %s: live joint angles don't close the linkage; seeding the endpoint target from the estimate (%.2f, %.2f)"),
						*C->GetName(), Seed.X, Seed.Y);
				}
				Target = &LinkageTargets.Add(C, Seed);
			}
			const FVector2D Candidate(Target->X, Target->Y + Dz);
			// Only advance the target when the linkage accepts it (reachable and
			// inside the motors' ranges), so holding a key at a limit doesn't
			// wind the target off into the unreachable.
			if (C->SetEndpointTarget(Candidate))
			{
				*Target = Candidate;
				if (bLogCommands)
				{
					UE_LOG(LogTemp, Verbose, TEXT("[KeyboardTeleop] %s endpoint -> (%.2f, %.2f)"), *C->GetName(), Candidate.X, Candidate.Y);
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
