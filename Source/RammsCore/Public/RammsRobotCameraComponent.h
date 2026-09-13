// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "InputCoreTypes.h"
#include "RammsRobotCameraComponent.generated.h"

class UCameraComponent;
class USpringArmComponent;

/**
 * Player camera control for a robot pawn: cycles between the pawn's camera
 * components and lets the mouse orbit / zoom the active one.
 *
 *  - NextCameraKey (Tab) activates the next UCameraComponent on the pawn: the
 *    CameraNames list in order, or else the authored cameras first and any
 *    runtime-added one (URLab's possess camera) last. The first is the start
 *    camera. Exactly one camera is active, so the view target picks it up
 *    with no view-target changes.
 *  - While OrbitButton (right mouse) is held, mouse movement yaws / pitches
 *    the spring arm the active camera hangs from; the wheel changes its length;
 *    ResetKey (Home) restores the arm's authored pose. Cameras without a spring
 *    arm parent are fixed.
 *
 * Polled each tick from the possessing player controller, with Slate's
 * pressed-button / cursor state as the mouse fallback (an editor PIE viewport
 * keeps the press that starts a drag for itself). No input mapping assets.
 * Orbit() / Zoom() / NextCamera() are also callable directly, for a touch UI,
 * a gamepad stick or Blueprint.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsRobotCameraComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsRobotCameraComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// --- Camera switching ------------------------------------------------------

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Switch")
	FKey NextCameraKey = EKeys::Tab;

	/** Restrict cycling to these camera component names, in this order ([0] is
	 *  the start camera). Empty = every UCameraComponent on the pawn, authored
	 *  ones first, then ones added at runtime (e.g. at possession). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Switch")
	TArray<FName> CameraNames;

	// --- Orbit / zoom ----------------------------------------------------------

	/** Hold to orbit with the mouse. Set to an invalid key to orbit whenever the
	 *  mouse moves. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Orbit")
	FKey OrbitButton = EKeys::RightMouseButton;

	/** Also orbit while the left button is held — a touch drag on the Pixel
	 *  Streaming page arrives as a left drag. The pawn has no click actions
	 *  of its own, so this costs nothing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Orbit")
	bool bAlsoOrbitWithLeftDrag = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Orbit")
	FKey ResetKey = EKeys::Home;

	/** Degrees of arm rotation per unit of mouse delta. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Orbit", meta = (ClampMin = "0.0"))
	float OrbitSensitivity = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Orbit")
	bool bInvertPitch = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Orbit")
	float MinPitch = -85.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Orbit")
	float MaxPitch = 15.0f;

	/** Arm length change per wheel notch (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Orbit", meta = (ClampMin = "0.0"))
	float ZoomStep = 40.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Orbit", meta = (ClampMin = "1.0"))
	float MinArmLength = 60.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Orbit", meta = (ClampMin = "1.0"))
	float MaxArmLength = 1500.0f;

	/** Activate the next camera (also bound to NextCameraKey). */
	UFUNCTION(BlueprintCallable, Category = "Camera")
	void NextCamera();

	/** Activate a camera by component name; false if not found. */
	UFUNCTION(BlueprintCallable, Category = "Camera")
	bool ActivateCamera(FName CameraName);

	/** Rotate the active camera's spring arm by the given degrees (pitch is
	 *  clamped to [MinPitch, MaxPitch]). The mouse path calls this; touch UI,
	 *  gamepad sticks or Blueprint can drive it directly. */
	UFUNCTION(BlueprintCallable, Category = "Camera")
	void Orbit(float DeltaYawDegrees, float DeltaPitchDegrees);

	/** Change the active camera's spring arm length by DeltaLength cm (clamped
	 *  to [MinArmLength, MaxArmLength]); negative zooms in. */
	UFUNCTION(BlueprintCallable, Category = "Camera")
	void Zoom(float DeltaLength);

	/** Restore the active camera's spring arm to its authored rotation/length. */
	UFUNCTION(BlueprintCallable, Category = "Camera")
	void ResetOrbit();

	UFUNCTION(BlueprintPure, Category = "Camera")
	UCameraComponent* GetActiveCamera() const { return ActiveCamera; }

private:
	APlayerController*	 GetPlayerController() const;
	void				 GatherCameras(TArray<UCameraComponent*>& Out) const;
	void				 SetActiveCamera(UCameraComponent* Camera);
	USpringArmComponent* ActiveArm() const;

	UPROPERTY(Transient)
	TObjectPtr<UCameraComponent> ActiveCamera;

	/** Authored (pre-orbit) pose per spring arm, captured on first use. */
	TMap<TWeakObjectPtr<USpringArmComponent>, TPair<FRotator, float>> ArmDefaults;

	/** Last cursor position, for the position-delta orbit path. */
	float LastCursorX = 0.0f;
	float LastCursorY = 0.0f;
	bool  bHadCursor = false;
};
