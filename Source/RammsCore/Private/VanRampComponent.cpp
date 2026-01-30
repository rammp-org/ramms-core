// Copyright Epic Games, Inc. All Rights Reserved.

#include "VanRampComponent.h"
#include "Curves/CurveFloat.h"
#include "DrawDebugHelpers.h"

UVanRampComponent::UVanRampComponent()
{
	PrimaryComponentTick.bCanEverTick = true;

	RampAnimationDuration = 2.0f;
	RampState = ERampState::Retracted;
	CurrentAnimationTime = 0.0f;
	bShowKeyframeGizmos = true;
	GizmoSize = 50.0f;
	bShowKeyframePath = true;

	// Set collision properties for the ramp
	SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	SetCollisionProfileName(TEXT("BlockAll"));

	// Default keyframes - typical ramp deployment
	// Rotate down from stowed position to deployed on ground
	RampKeyframes.Empty();
	RampKeyframes.Add(FRampKeyframe(0.0f, FTransform(FRotator::ZeroRotator, FVector::ZeroVector, FVector::OneVector)));
	RampKeyframes.Add(FRampKeyframe(0.3f, FTransform(FRotator(-30.0f, 0.0f, 0.0f), FVector(-20.0f, 0.0f, -20.0f), FVector::OneVector)));
	RampKeyframes.Add(FRampKeyframe(1.0f, FTransform(FRotator(-60.0f, 0.0f, 0.0f), FVector(-80.0f, 0.0f, -60.0f), FVector::OneVector)));
}

void UVanRampComponent::OnComponentCreated()
{
	Super::OnComponentCreated();
	// Component is already the mesh, no need to create child
}

void UVanRampComponent::BeginPlay()
{
	Super::BeginPlay();

	// Store the initial retracted transform
	RetractedTransform = GetRelativeTransform();
}

void UVanRampComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

	// Clear all debug draws
	if (UWorld* World = GetWorld())
	{
		FlushPersistentDebugLines(World);
	}
}

#if WITH_EDITOR
void UVanRampComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	// Force a redraw when properties change
	if (PropertyChangedEvent.Property)
	{
		FName PropertyName = PropertyChangedEvent.Property->GetFName();
		if (PropertyName == GET_MEMBER_NAME_CHECKED(UVanRampComponent, RampKeyframes) ||
			PropertyName == GET_MEMBER_NAME_CHECKED(UVanRampComponent, bShowKeyframeGizmos) ||
			PropertyName == GET_MEMBER_NAME_CHECKED(UVanRampComponent, GizmoSize) ||
			PropertyName == GET_MEMBER_NAME_CHECKED(UVanRampComponent, bShowKeyframePath))
		{
			// Clear existing debug draws and redraw
			if (UWorld* World = GetWorld())
			{
				FlushPersistentDebugLines(World);
			}
			MarkRenderStateDirty();
		}
	}
}

void UVanRampComponent::DrawKeyframeGizmos() const
{
	if (!bShowKeyframeGizmos || RampKeyframes.Num() == 0)
	{
		return;
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	FTransform ComponentTransform = GetComponentTransform();

	// Draw each keyframe
	FVector PreviousWorldLocation = FVector::ZeroVector;
	for (int32 i = 0; i < RampKeyframes.Num(); ++i)
	{
		const FRampKeyframe& Keyframe = RampKeyframes[i];
		
		// Calculate world transform for this keyframe
		FTransform KeyframeWorldTransform = Keyframe.Transform * RetractedTransform * ComponentTransform;
		FVector WorldLocation = KeyframeWorldTransform.GetLocation();
		FRotator WorldRotation = KeyframeWorldTransform.Rotator();

		// Draw coordinate system at keyframe location
		FLinearColor LerpedColor = FLinearColor::LerpUsingHSV(FLinearColor::Green, FLinearColor::Red, Keyframe.Time);
		FColor GizmoColor = LerpedColor.ToFColor(true);
		DrawDebugCoordinateSystem(World, WorldLocation, WorldRotation, GizmoSize, true, 0.0f, 0, 2.0f);

		// Draw sphere at keyframe location
		DrawDebugSphere(World, WorldLocation, 10.0f, 8, GizmoColor, true, 0.0f, 0, 2.0f);

		// Draw time label
		DrawDebugString(World, WorldLocation + FVector(0, 0, GizmoSize + 20.0f), 
			FString::Printf(TEXT("T: %.2f"), Keyframe.Time), nullptr, FColor::White, 0.0f, true, 1.5f);

		// Draw line connecting to previous keyframe
		if (bShowKeyframePath && i > 0)
		{
			DrawDebugLine(World, PreviousWorldLocation, WorldLocation, FColor::Yellow, true, 0.0f, 0, 3.0f);
		}

		PreviousWorldLocation = WorldLocation;
	}
}
#endif

void UVanRampComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

#if WITH_EDITOR
	// Draw debug gizmos in editor
	if (GetWorld() && !GetWorld()->IsGameWorld())
	{
		DrawKeyframeGizmos();
	}
#endif

	if (RampState == ERampState::Deploying || RampState == ERampState::Retracting)
	{
		float Direction = (RampState == ERampState::Deploying) ? 1.0f : -1.0f;
		CurrentAnimationTime += DeltaTime * Direction;

		// Clamp animation time
		if (CurrentAnimationTime >= RampAnimationDuration)
		{
			CurrentAnimationTime = RampAnimationDuration;
			RampState = ERampState::Deployed;
			OnRampDeployed.Broadcast();
		}
		else if (CurrentAnimationTime <= 0.0f)
		{
			CurrentAnimationTime = 0.0f;
			RampState = ERampState::Retracted;
			OnRampRetracted.Broadcast();
		}

		UpdateRampTransform();
	}
}

void UVanRampComponent::InteractWithRamp()
{
	if (RampState == ERampState::Retracted)
	{
		DeployRamp();
	}
	else if (RampState == ERampState::Deployed)
	{
		RetractRamp();
	}
}

void UVanRampComponent::DeployRamp()
{
	if (RampState == ERampState::Retracted || RampState == ERampState::Retracting)
	{
		RampState = ERampState::Deploying;
	}
}

void UVanRampComponent::RetractRamp()
{
	if (RampState == ERampState::Deployed || RampState == ERampState::Deploying)
	{
		RampState = ERampState::Retracting;
	}
}

void UVanRampComponent::UpdateRampTransform()
{
	// Calculate normalized time (0 to 1) based on current animation time
	float NormalizedTime = RampAnimationDuration > 0.0f ? CurrentAnimationTime / RampAnimationDuration : 0.0f;

	// Apply optional easing curve
	if (RampAnimationCurve)
	{
		NormalizedTime = RampAnimationCurve->GetFloatValue(NormalizedTime);
	}

	// Interpolate through keyframes
	FTransform TargetTransform = InterpolateKeyframes(NormalizedTime);
	
	// Apply the transform relative to the retracted position
	FTransform FinalTransform = TargetTransform * RetractedTransform;
	SetRelativeTransform(FinalTransform);
}

FTransform UVanRampComponent::InterpolateKeyframes(float NormalizedTime) const
{
	// Handle edge cases
	if (RampKeyframes.Num() == 0)
	{
		return FTransform::Identity;
	}
	
	if (RampKeyframes.Num() == 1)
	{
		return RampKeyframes[0].Transform;
	}

	// Clamp normalized time
	NormalizedTime = FMath::Clamp(NormalizedTime, 0.0f, 1.0f);

	// Find the two keyframes to interpolate between
	int32 NextKeyframeIndex = 0;
	for (int32 i = 0; i < RampKeyframes.Num(); ++i)
	{
		if (RampKeyframes[i].Time > NormalizedTime)
		{
			NextKeyframeIndex = i;
			break;
		}
		NextKeyframeIndex = i + 1;
	}

	// Handle boundary cases
	if (NextKeyframeIndex == 0)
	{
		return RampKeyframes[0].Transform;
	}
	
	if (NextKeyframeIndex >= RampKeyframes.Num())
	{
		return RampKeyframes.Last().Transform;
	}

	// Get the two keyframes
	const FRampKeyframe& PrevKeyframe = RampKeyframes[NextKeyframeIndex - 1];
	const FRampKeyframe& NextKeyframe = RampKeyframes[NextKeyframeIndex];

	// Calculate interpolation alpha between these two keyframes
	float KeyframeDelta = NextKeyframe.Time - PrevKeyframe.Time;
	float Alpha = KeyframeDelta > 0.0f ? (NormalizedTime - PrevKeyframe.Time) / KeyframeDelta : 0.0f;

	// Interpolate transform components
	FVector InterpLocation = FMath::Lerp(PrevKeyframe.Transform.GetLocation(), NextKeyframe.Transform.GetLocation(), Alpha);
	FQuat InterpRotation = FQuat::Slerp(PrevKeyframe.Transform.GetRotation(), NextKeyframe.Transform.GetRotation(), Alpha);
	FVector InterpScale = FMath::Lerp(PrevKeyframe.Transform.GetScale3D(), NextKeyframe.Transform.GetScale3D(), Alpha);

	return FTransform(InterpRotation, InterpLocation, InterpScale);
}
