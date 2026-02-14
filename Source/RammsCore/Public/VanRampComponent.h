// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/StaticMeshComponent.h"
#include "VanRampComponent.generated.h"

class UCurveFloat;

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnRampDeployedSignature);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnRampRetractedSignature);

UENUM(BlueprintType)
enum class ERampState : uint8
{
	Retracted,
	Deploying,
	Deployed,
	Retracting
};

USTRUCT(BlueprintType)
struct FRampKeyframe
{
	GENERATED_BODY()

	// Time offset in the animation (0 = start, 1 = end)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Keyframe", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Time;

	// Transform at this keyframe (relative to component)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Keyframe")
	FTransform Transform;

	FRampKeyframe()
		: Time(0.0f)
		, Transform(FTransform::Identity)
	{}

	FRampKeyframe(float InTime, const FTransform& InTransform)
		: Time(InTime)
		, Transform(InTransform)
	{}
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class RAMMSCORE_API UVanRampComponent : public UStaticMeshComponent
{
	GENERATED_BODY()

public:	
	UVanRampComponent();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// Interaction function to toggle ramp deploy/retract
	UFUNCTION(BlueprintCallable, Category = "Van Ramp")
	void InteractWithRamp();

	// Manual control functions
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Van Ramp")
	void DeployRamp();

	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Van Ramp")
	void RetractRamp();

	// Get current ramp state
	UFUNCTION(BlueprintPure, Category = "Van Ramp")
	ERampState GetRampState() const { return RampState; }

	// Multicast delegates
	UPROPERTY(BlueprintAssignable, Category = "Van Ramp|Events")
	FOnRampDeployedSignature OnRampDeployed;

	UPROPERTY(BlueprintAssignable, Category = "Van Ramp|Events")
	FOnRampRetractedSignature OnRampRetracted;

protected:
	virtual void BeginPlay() override;
	virtual void OnComponentCreated() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	// The static mesh component representing the ramp
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Van Ramp")
	UStaticMeshComponent* RampMesh;

	// Duration of the ramp animation in seconds
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Ramp", meta = (ClampMin = "0.1", ClampMax = "10.0"))
	float RampAnimationDuration;

	// Keyframes defining the ramp animation path (relative to this component)
	// Keyframe times should be normalized 0-1, where 0 is retracted and 1 is fully deployed
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Ramp")
	TArray<FRampKeyframe> RampKeyframes;

	// Optional timeline curve for overall easing (applied to normalized time before keyframe lookup)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Ramp")
	UCurveFloat* RampAnimationCurve;

	// Show debug visualization of keyframes in the editor viewport
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Ramp|Debug")
	bool bShowKeyframeGizmos;

	// Size of the keyframe coordinate system gizmos
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Ramp|Debug", meta = (ClampMin = "1.0", ClampMax = "500.0"))
	float GizmoSize;

	// Show lines connecting keyframes
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Ramp|Debug")
	bool bShowKeyframePath;

	// Current state of the ramp
	UPROPERTY(BlueprintReadOnly, Category = "Van Ramp")
	ERampState RampState;

private:
	// Current animation time
	float CurrentAnimationTime;

	// Initial transform (retracted position)
	FTransform RetractedTransform;

	// Update the ramp mesh transform based on current animation time
	void UpdateRampTransform();

	// Interpolate between keyframes based on normalized time (0-1)
	FTransform InterpolateKeyframes(float NormalizedTime) const;

#if WITH_EDITOR
	// Draw debug visualization for keyframes
	void DrawKeyframeGizmos() const;
#endif
};
