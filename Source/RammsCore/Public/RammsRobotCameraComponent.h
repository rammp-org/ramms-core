// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "InputCoreTypes.h"
#include "RammsControlContributor.h"
#include "RammsRobotCameraComponent.generated.h"

class UCameraComponent;
class USpringArmComponent;

/**
 * Player camera control for a robot pawn: cycles between the pawn's camera
 * components and lets the mouse orbit / zoom the active one.
 *
 *  - NextCameraKey (N) activates the next UCameraComponent on the pawn: the
 *    CameraNames list in order, or else the authored cameras first and any
 *    runtime-added one (URLab's possess camera) last. The first is the start
 *    camera. Exactly one camera is active, so the view target picks it up
 *    with no view-target changes.
 *  - While OrbitButton (right mouse) is held, mouse movement yaws / pitches
 *    the spring arm the active camera hangs from; the wheel eases its length
 *    (proportional steps, sensitivity, smoothing — see the Zoom settings);
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
class RAMMSCORE_API URammsRobotCameraComponent : public UActorComponent, public IRammsControlContributor
{
	GENERATED_BODY()

public:
	URammsRobotCameraComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// --- Camera switching ------------------------------------------------------

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

	/** Degrees of arm rotation per unit of mouse delta. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Orbit", meta = (ClampMin = "0.0"))
	float OrbitSensitivity = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Orbit")
	bool bInvertPitch = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Orbit")
	float MinPitch = -85.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Orbit")
	float MaxPitch = 15.0f;

	// --- Zoom ------------------------------------------------------------------
	// A wheel notch changes the *desired* arm length; the arm then eases toward
	// it (ZoomInterpSpeed), so one notch is a short glide rather than a jump.
	// A physical wheel click arrives as a burst of events; ZoomRepeatDelay
	// collapses the burst into one step, ZoomSensitivity scales the step, and a
	// proportional step keeps it feeling the same whether the camera is close
	// or far.

	/** Multiplier on the zoom step. Lower it if one wheel click zooms too far. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Zoom", meta = (ClampMin = "0.0"))
	float ZoomSensitivity = 1.0f;

	/** Step per notch as a fraction of the current arm length (0.08 = 8 %), so
	 *  zooming is geometric: each notch feels the same at any distance. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Zoom", meta = (EditCondition = "bZoomProportional", ClampMin = "0.0", ClampMax = "1.0"))
	float ZoomStepFraction = 0.08f;

	/** Off: a fixed ZoomStep (cm) per notch instead of a fraction. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Zoom")
	bool bZoomProportional = true;

	/** Arm length change per wheel notch (cm) when not proportional. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Zoom", meta = (EditCondition = "!bZoomProportional", ClampMin = "0.0"))
	float ZoomStep = 20.0f;

	/** One physical wheel click reaches the game as a burst of events spread
	 *  over ~0.15-0.2 s (macOS smooth scrolling: ~16 events for one line;
	 *  high-resolution wheels more). A burst is ONE step: a new step starts
	 *  when the wheel was idle for at least ZoomIdleGap before this event, or
	 *  when ZoomRepeatDelay has passed since the last step (a held wheel then
	 *  repeats at 1 / ZoomRepeatDelay). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Zoom", meta = (ClampMin = "0.0"))
	float ZoomIdleGap = 0.08f;

	/** See ZoomIdleGap. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Zoom", meta = (ClampMin = "0.0"))
	float ZoomRepeatDelay = 0.3f;

	/** How quickly the arm eases toward the desired length (1/s); 0 = instant. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Zoom", meta = (ClampMin = "0.0"))
	float ZoomInterpSpeed = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Zoom", meta = (ClampMin = "1.0"))
	float MinArmLength = 60.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Zoom", meta = (ClampMin = "1.0"))
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

	/** Change the active camera's desired spring arm length by DeltaLength cm
	 *  (clamped to [MinArmLength, MaxArmLength]); negative zooms in. The arm
	 *  eases toward it at ZoomInterpSpeed. */
	UFUNCTION(BlueprintCallable, Category = "Camera")
	void Zoom(float DeltaLength);

	/** Zoom by wheel notches (positive = in), applying ZoomSensitivity and the
	 *  proportional / fixed step — what the wheel calls. */
	UFUNCTION(BlueprintCallable, Category = "Camera")
	void ZoomNotches(float Notches);

	/** The arm length the zoom is easing toward (the arm's own TargetArmLength
	 *  lags it while ZoomInterpSpeed > 0). */
	UFUNCTION(BlueprintPure, Category = "Camera")
	float GetDesiredArmLength() const { return DesiredArmLength; }

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

	/** Arm length the active arm eases toward; re-seeded on camera switch / reset. */
	float DesiredArmLength = 0.0f;

	/** Burst gating: when the last wheel event arrived and the last step fired. */
	double LastWheelEventTime = -1.0;
	double LastZoomStepTime = -1.0;

	/** Last cursor position, for the position-delta orbit path. */
	float LastCursorX = 0.0f;
	float LastCursorY = 0.0f;
	bool  bHadCursor = false;

public:
	// --- Control surface ("camera.*") ----------------------------------------
	/** Orbit rate at full deflection of the camera.orbit_* axes (deg/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Control", meta = (ClampMin = "0.0"))
	float ControlOrbitRateDegPerSec = 90.0f;

	/** Zoom rate at full deflection of camera.zoom, in wheel notches per second. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Control", meta = (ClampMin = "0.0"))
	float ControlZoomNotchesPerSec = 6.0f;

	virtual void  DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool  ApplyControl(FName Id, float Value) override;
	virtual bool  TriggerControl(FName Id) override;
	virtual bool  ReleaseControl(FName Id) override;
	virtual int32 GetControlOrder() const override { return 90; }

private:
	/** Orbit / zoom rates as last set through the control surface; integrated in Tick. */
	FVector2D ControlOrbit = FVector2D::ZeroVector;
	float	  ControlZoom = 0.0f;
};
