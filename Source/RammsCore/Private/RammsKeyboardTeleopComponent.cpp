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

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}
	Base = Owner->FindComponentByClass<URammsRobotBaseComponent>();
	Surface = URammsRobotControlSurfaceComponent::FindGoverningSurface(this);

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
	// Discovery deliberately does NOT happen here: see RefreshDiscovery.
	UE_LOG(LogTemp, Log, TEXT("[KeyboardTeleop] '%s': surface=%s motor bindings=%d base=%s"),
		*Owner->GetName(), Surface ? TEXT("yes") : TEXT("no"), MotorBindings.Num(),
		Base ? TEXT("yes") : TEXT("no"));
}

void URammsKeyboardTeleopComponent::RefreshDiscovery()
{
	if (!Surface)
	{
		return;
	}
	const int32 Version = IRammsControlSurfaceProvider::Execute_GetControlSurfaceVersion(Surface);
	if (Version == DiscoveredVersion)
	{
		return;
	}
	DiscoveredVersion = Version;

	// Discover what this robot can actually do rather than looking for known
	// controller classes. Linkage controls are per instance
	// ("linkage.<component>.height"), so they are matched by group and kind;
	// the drive axes are well-known Ids.
	LinkageControlIds.Reset();
	bool					   bHasDrive = false;
	const FRammsControlSurface Described = Surface->DescribeControlSurface();
	for (const FRammsControlAxis& Axis : Described.Axes)
	{
		if (Axis.Id == RammsControlIds::Drive::Forward())
		{
			bHasDrive = true;
		}
		// Height only. A 5-bar offers fore/aft on the same group, and the
		// raise/lower keys must not drive both at once -- that would walk the
		// endpoint diagonally. Fore/aft is a slider on the control surface;
		// give it its own keys if it ever wants them.
		if (Axis.Group == RammsControlIds::Groups::Linkage()
			&& Axis.Kind == ERammsControlKind::Position && !Axis.bReadOnly
			&& Axis.Id.ToString().EndsWith(RammsControlIds::Linkage::HeightSuffix())
			&& MatchesLinkageFilter(Axis.Id))
		{
			LinkageControlIds.Add(Axis.Id);
		}
	}

	// A control that is gone took its target with it: keeping the old value
	// would resume the key from a stale height when the control comes back.
	for (auto It = LinkageTargets.CreateIterator(); It; ++It)
	{
		if (!LinkageControlIds.Contains(It.Key()))
		{
			It.RemoveCurrent();
		}
	}

	// Tick before whatever contributes those controls, so a key pressed this
	// frame is applied this frame. Done by component, not by class -- and
	// redone on every rebuild, because a mode switch changes who contributes.
	TArray<UActorComponent*> Contributors = Surface->GetContributorComponents();
	for (const TWeakObjectPtr<UActorComponent>& Weak : TickDependents)
	{
		UActorComponent* Old = Weak.Get();
		if (Old && !Contributors.Contains(Old))
		{
			Old->RemoveTickPrerequisiteComponent(this);
		}
	}
	TickDependents.Reset();
	for (UActorComponent* C : Contributors)
	{
		if (C && C != this)
		{
			C->AddTickPrerequisiteComponent(this);
			TickDependents.Add(C);
		}
	}

	bSurfaceHasDrive = bHasDrive;

	if (!bHasDrive && LinkageControlIds.Num() == 0)
	{
		if (!bWarnedNothingToDrive)
		{
			bWarnedNothingToDrive = true;
			UE_LOG(LogTemp, Warning,
				TEXT("[KeyboardTeleop] '%s' has a control surface but it advertises no drive axes and no linkage controls: check the robot's contributors."),
				*GetNameSafe(GetOwner()));
		}
	}
	else
	{
		// Something to drive again: a later empty surface is worth saying once more.
		bWarnedNothingToDrive = false;
	}

	UE_LOG(LogTemp, Log,
		TEXT("[KeyboardTeleop] '%s': surface version %d -- drive=%s linkage controls=%d"),
		*GetNameSafe(GetOwner()), Version, bHasDrive ? TEXT("yes") : TEXT("no"),
		LinkageControlIds.Num());
}

void URammsKeyboardTeleopComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	ReleaseDrive();
	Super::EndPlay(EndPlayReason);
}

void URammsKeyboardTeleopComponent::ReleaseDrive()
{
	// A drive controller holds its last command, so without this the robot
	// keeps driving after the player lets go of it (keys up, unpossess, drive
	// disabled, end of play).
	//
	// Release rather than write zero. A zero written as Keyboard is still a
	// Keyboard command, and the surface records a hold for it -- which locks
	// out every lower-priority source until the hold ages out. ReleaseControl
	// drops the hold and lets the contributor spring back, which is what
	// letting go actually means.
	if (bDriveCommandActive && Surface && bSurfaceHasDrive)
	{
		// The surface refuses a release while a higher-priority source holds
		// the axis, so a false here means it did not happen -- leave it
		// pending (Tick retries) rather than pretending it did.
		const bool bF = Surface->ReleaseControl(RammsControlIds::Drive::Forward(), ERammsControlSource::Keyboard);
		const bool bT = Surface->ReleaseControl(RammsControlIds::Drive::Turn(), ERammsControlSource::Keyboard);
		if (!bF && !bT)
		{
			return;
		}
		if (bLogCommands)
		{
			UE_LOG(LogTemp, Log, TEXT("[KeyboardTeleop] drive input released"));
		}
	}
	// Nothing to release against (no surface, or the active drive mode offers
	// no drive axes) counts as released: retrying forever would not help.
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

	// Cheap when nothing changed: a version compare.
	RefreshDiscovery();

	APlayerController* PC = GetPlayerController();

	// --- Drive -----------------------------------------------------------
	if (PC && bDriveEnabled && Surface)
	{
		// Kept as (turn, forward) to match what this component always sent.
		const FVector2D Input(Axis(PC, TurnRightKey, TurnLeftKey) * TurnScale, Axis(PC, ForwardKey, BackwardKey) * ForwardScale);

		// Write only while there is something to say. Re-asserting zero every
		// frame looks harmless and is not: the surface records a Keyboard hold
		// on the axis each time, and every lower-priority source -- a script,
		// a remote client -- is then refused for as long as this component
		// exists. It silently stopped a PIE drive test from commanding the
		// robot at all, and would do the same to anything else.
		if (!Input.IsNearlyZero())
		{
			Surface->SetControl(RammsControlIds::Drive::Forward(), Input.Y, ERammsControlSource::Keyboard);
			Surface->SetControl(RammsControlIds::Drive::Turn(), Input.X, ERammsControlSource::Keyboard);
			bDriveCommandActive = true;
		}
		else if (bDriveCommandActive)
		{
			// The keys came up: hand the axes back once, then stay quiet.
			ReleaseDrive();
		}
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
				// What the surface accepted, not what was asked for: SetControl
				// clamps to the axis range and still reports success, so caching
				// the candidate lets a held key wind the target past the limit
				// and the key has to unwind that before the endpoint moves back.
				float Accepted = Candidate;
				if (!Surface->GetControlTarget(Id, Accepted))
				{
					Accepted = Candidate;
				}
				*Target = Accepted;
				if (bLogCommands)
				{
					UE_LOG(LogTemp, Verbose, TEXT("[KeyboardTeleop] %s -> %.2f"), *Id.ToString(), *Target);
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
