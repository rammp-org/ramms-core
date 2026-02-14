// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "VanDoorComponent.generated.h"

class USkeletalMeshComponent;
class UCurveFloat;
struct FConstraintInstance;

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnDoorOpenedSignature);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnDoorClosedSignature);

UENUM(BlueprintType)
enum class EDoorState : uint8
{
	Closed,
	Opening,
	Open,
	Closing
};

USTRUCT(BlueprintType)
struct FDoorKeyframe
{
	GENERATED_BODY()

	// Time offset in the animation (0 = start, 1 = end)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Keyframe", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Time;

	// Linear position offset at this keyframe (relative to closed position)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Keyframe")
	FVector LinearPosition;

	// Angular position offset at this keyframe (in degrees)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Keyframe")
	FRotator AngularPosition;

	FDoorKeyframe()
		: Time(0.0f)
		, LinearPosition(FVector::ZeroVector)
		, AngularPosition(FRotator::ZeroRotator)
	{}

	FDoorKeyframe(float InTime, const FVector& InLinear, const FRotator& InAngular = FRotator::ZeroRotator)
		: Time(InTime)
		, LinearPosition(InLinear)
		, AngularPosition(InAngular)
	{}
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class RAMMSCORE_API UVanDoorComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	UVanDoorComponent();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// Interaction function to toggle door open/close
	UFUNCTION(BlueprintCallable, Category = "Van Door")
	void InteractWithDoor();

	// Manual control functions
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Van Door")
	void OpenDoor();

	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Van Door")
	void CloseDoor();

	// Get current door state
	UFUNCTION(BlueprintPure, Category = "Van Door")
	EDoorState GetDoorState() const { return DoorState; }

	// Multicast delegates
	UPROPERTY(BlueprintAssignable, Category = "Van Door|Events")
	FOnDoorOpenedSignature OnDoorOpened;

	UPROPERTY(BlueprintAssignable, Category = "Van Door|Events")
	FOnDoorClosedSignature OnDoorClosed;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// Name of the constraint (joint) in the physics asset
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Door")
	FName ConstraintName;

	// If true, automatically find the skeletal mesh component on the owner
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Door")
	bool bAutoFindSkeletalMesh;

	// Duration of the door animation in seconds
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Door", meta = (ClampMin = "0.1", ClampMax = "10.0"))
	float DoorAnimationDuration;

	// Keyframes defining the door animation path
	// Keyframe times should be normalized 0-1, where 0 is closed and 1 is fully open
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Door")
	TArray<FDoorKeyframe> DoorKeyframes;

	// Optional timeline curve for overall easing (applied to normalized time before keyframe lookup)
	UPROPERTY(EditAnywhere, BluePrintReadWrite, Category = "Van Door")
	UCurveFloat* DoorAnimationCurve;

	// Motor strength for driving the constraint (higher = stronger/faster)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Door|Physics", meta = (ClampMin = "0.0"))
	float MotorStrength;

	// Show lines connecting keyframes
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Door|Debug")
	bool bShowKeyframePath;

	// Show debug visualization of keyframes in the editor/game
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Door|Debug")
	bool bShowKeyframeGizmos;

	// Size of the keyframe coordinate system gizmos
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Door|Debug", meta = (ClampMin = "1.0", ClampMax = "500.0"))
	float GizmoSize;

	// Enable debug logging
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Van Door|Debug")
	bool bEnableDebugLog;

	// Current state of the door
	UPROPERTY(BlueprintReadOnly, Category = "Van Door")
	EDoorState DoorState;

private:
	// Current animation time
	float CurrentAnimationTime;

	// Cached skeletal mesh component
	UPROPERTY()
	USkeletalMeshComponent* CachedSkeletalMesh;

	// Cached constraint instance
	FConstraintInstance* CachedConstraint;

	// Update the door constraint motors based on current animation time
	void UpdateDoorConstraint();

	// Interpolate between keyframes based on normalized time (0-1)
	void GetKeyframePositions(float NormalizedTime, FVector& OutLinear, FRotator& OutAngular) const;

	// Check if door has reached target position
	bool IsAtTargetPosition(const FVector& TargetLinear) const;

	// Draw debug visualization for keyframes and door state
	void DrawDebugVisualization();

	// Get skeletal mesh from owner
	USkeletalMeshComponent* GetOwnerSkeletalMesh();
};

