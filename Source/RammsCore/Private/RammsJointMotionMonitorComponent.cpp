// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsJointMotionMonitorComponent.h"
#include "RammsSkeletalPoseComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

URammsJointMotionMonitorComponent::URammsJointMotionMonitorComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false; // only ticks while the wheels are moving

	// Kinova Gen3 on BP_RAMMP_RemoteDisplay.
	ArmJointNames = { "link_0", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6" };

	// Kinova_Static_Poses "Retracted".
	StowedPose = { 22.918312f, 0.0f, 1.145916f, 130.061417f, -89.954376f, -48.128457f, 112.872688f };

	// RAMMP Gen1 drive wheels (the casters are passive).
	WheelJointNames = { "drive_wheel_l", "drive_wheel_r" };
}

void URammsJointMotionMonitorComponent::BeginPlay()
{
	Super::BeginPlay();

	AActor* Owner = GetOwner();
	PoseComponent = Owner ? Owner->FindComponentByClass<URammsSkeletalPoseComponent>() : nullptr;
	if (!PoseComponent)
	{
		UE_LOG(LogTemp, Warning, TEXT("RammsJointMotionMonitor on %s: no RammsSkeletalPoseComponent on the actor."),
			Owner ? *Owner->GetName() : TEXT("?"));
		return;
	}

	ResolveJointIndices();
	PoseComponent->OnJointTargetsSet.AddDynamic(this, &URammsJointMotionMonitorComponent::HandleJointTargetsSet);

	// Classify whatever the targets already are, so the state is right before the first command.
	HandleJointTargetsSet();
}

void URammsJointMotionMonitorComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (PoseComponent)
	{
		PoseComponent->OnJointTargetsSet.RemoveDynamic(this, &URammsJointMotionMonitorComponent::HandleJointTargetsSet);
	}
	Super::EndPlay(EndPlayReason);
}

void URammsJointMotionMonitorComponent::ResolveJointIndices()
{
	auto Resolve = [this](const TArray<FName>& Names, TArray<int32>& OutIndices, const TCHAR* What)
	{
		OutIndices.Reset(Names.Num());
		for (const FName& Name : Names)
		{
			const int32 Index = PoseComponent->GetJointIndexByName(Name);
			if (Index == INDEX_NONE)
			{
				UE_LOG(LogTemp, Warning, TEXT("RammsJointMotionMonitor: %s joint '%s' not found on the pose component."),
					What, *Name.ToString());
			}
			OutIndices.Add(Index);
		}
	};

	Resolve(ArmJointNames, ArmIndices, TEXT("arm"));
	Resolve(WheelJointNames, WheelIndices, TEXT("wheel"));

	if (StowedPose.Num() != ArmJointNames.Num())
	{
		UE_LOG(LogTemp, Warning, TEXT("RammsJointMotionMonitor: StowedPose has %d values for %d arm joints; the extra joints are ignored."),
			StowedPose.Num(), ArmJointNames.Num());
	}

	LastWheelTargets.Reset();
	bHaveWheelBaseline = false;
}

void URammsJointMotionMonitorComponent::HandleJointTargetsSet()
{
	if (!PoseComponent)
	{
		return;
	}

	// ── Arm ──
	bool	  bStowed = false;
	const int32 NumArm = FMath::Min(ArmIndices.Num(), StowedPose.Num());
	if (NumArm > 0)
	{
		bStowed = true;
		for (int32 i = 0; i < NumArm; ++i)
		{
			if (ArmIndices[i] == INDEX_NONE)
			{
				continue;
			}
			const float Target = PoseComponent->GetJointTarget(ArmIndices[i]);
			if (FMath::Abs(FMath::FindDeltaAngleDegrees(StowedPose[i], Target)) > StowedTolerance)
			{
				bStowed = false;
				break;
			}
		}
	}
	SetArmPoseState(bStowed ? ERammsArmPoseState::Stowed : ERammsArmPoseState::Active);

	// ── Wheels ──
	if (WheelIndices.Num() == 0)
	{
		return;
	}

	TArray<float> WheelTargets;
	WheelTargets.Reserve(WheelIndices.Num());
	for (int32 Index : WheelIndices)
	{
		WheelTargets.Add(Index != INDEX_NONE ? PoseComponent->GetJointTarget(Index) : 0.0f);
	}

	if (bHaveWheelBaseline)
	{
		bool bChanged = false;
		for (int32 i = 0; i < WheelTargets.Num(); ++i)
		{
			// Wrapped, so a wheel crossing ±180 reads as a small step rather than a full turn.
			if (FMath::Abs(FMath::FindDeltaAngleDegrees(LastWheelTargets[i], WheelTargets[i])) > WheelMotionThreshold)
			{
				bChanged = true;
				break;
			}
		}

		if (bChanged)
		{
			LastWheelMotionTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
			SetWheelsMoving(true);
		}
	}

	LastWheelTargets = MoveTemp(WheelTargets);
	bHaveWheelBaseline = true;
}

void URammsJointMotionMonitorComponent::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// Ticks only while moving: commands may stop arriving altogether when the wheels stop.
	const UWorld* World = GetWorld();
	if (bWheelsMoving && World && World->GetTimeSeconds() - LastWheelMotionTime > WheelIdleTimeout)
	{
		SetWheelsMoving(false);
	}
}

void URammsJointMotionMonitorComponent::SetArmPoseState(ERammsArmPoseState NewState)
{
	if (ArmPoseState == NewState)
	{
		return;
	}
	ArmPoseState = NewState;
	if (bDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("RammsJointMotionMonitor: arm %s"),
			NewState == ERammsArmPoseState::Stowed ? TEXT("stowed") : TEXT("active"));
	}
	OnArmPoseStateChanged.Broadcast(NewState);
}

void URammsJointMotionMonitorComponent::SetWheelsMoving(bool bMoving)
{
	if (bWheelsMoving == bMoving)
	{
		return;
	}
	bWheelsMoving = bMoving;
	SetComponentTickEnabled(bMoving);
	if (bDebugLog)
	{
		UE_LOG(LogTemp, Log, TEXT("RammsJointMotionMonitor: wheels %s"), bMoving ? TEXT("moving") : TEXT("stopped"));
	}
	OnWheelsMovingChanged.Broadcast(bMoving);
}
