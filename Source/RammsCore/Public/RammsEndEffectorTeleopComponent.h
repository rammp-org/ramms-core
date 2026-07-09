// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "InputCoreTypes.h"
#include "RammsEndEffectorTeleopComponent.generated.h"

class UCameraComponent;
class UGripperControllerComponent;
class UKinovaGen3ControllerComponent;
class USpringArmComponent;

/**
 * Simple local teleoperation helper for end-effector IK targets.
 *
 * Add this to an actor that already has a UKinovaGen3ControllerComponent.
 * It polls the local player controller for keyboard / mouse input and nudges
 * the arm's end-effector target pose each tick.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsEndEffectorTeleopComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsEndEffectorTeleopComponent();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Optional component name override when an actor has multiple Kinova controllers */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Target")
	FName KinovaControllerComponentName = NAME_None;

	/** Master teleop enable flag */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|State")
	bool bTeleopEnabled = true;

	/** Poll keyboard / mouse from the first local player controller each tick */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|State")
	bool bEnableKeyboardMouseTeleop = true;

	/** Force the target Kinova controller into EndEffectorControl while teleop is active */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|State")
	bool bForceEndEffectorControlMode = true;

	/** Snap the IK target to the current end-effector pose when play begins */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|State")
	bool bAutoSyncTargetToCurrentPoseOnBeginPlay = true;

	/** Apply translational / rotational input in the end-effector local frame */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Motion")
	bool bInputInLocalFrame = true;

	/** Base translation speed used for keyboard / remote teleop input */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Motion", meta = (ClampMin = "0.1"))
	float LinearSpeedCmPerSecond = 20.0f;

	/** Base angular speed used for keyboard / remote teleop input */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Motion", meta = (ClampMin = "0.1"))
	float AngularSpeedDegPerSecond = 45.0f;

	/** Multiplier while either Shift key is held */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Motion", meta = (ClampMin = "0.1"))
	float FastSpeedMultiplier = 4.0f;

	/** Multiplier while either Control key is held */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Motion", meta = (ClampMin = "0.01"))
	float SlowSpeedMultiplier = 0.25f;

	// Per-axis sign flips — tweak live in the editor to fix any inverted direction without a rebuild.
	// Mapping (end-effector frame): forward = X, strafe = Y, up = Z; yaw/pitch/roll are the matching
	// FRotator components. Kept parallel to URammsMjArmTeleopComponent so both teleop paths behave the same.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Axis Signs")
	float ForwardSign = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Axis Signs")
	float StrafeSign = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Axis Signs")
	float UpSign = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Axis Signs")
	float YawSign = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Axis Signs")
	float PitchSign = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Axis Signs")
	float RollSign = 1.0f;

	// --- Keyboard bindings (configurable). Defaults match URammsMjArmTeleopComponent so the arm and the
	// base can be driven at the same time (they deliberately avoid WASD). Each action is a +/- key pair. ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Keys")
	FKey ForwardKey = EKeys::I;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Keys")
	FKey BackwardKey = EKeys::K;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Keys")
	FKey StrafeLeftKey = EKeys::J;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Keys")
	FKey StrafeRightKey = EKeys::L;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Keys")
	FKey UpKey = EKeys::U;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Keys")
	FKey DownKey = EKeys::O;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Keys")
	FKey RollLeftKey = EKeys::M;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Keys")
	FKey RollRightKey = EKeys::Period;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Keys")
	FKey YawLeftKey = EKeys::Left;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Keys")
	FKey YawRightKey = EKeys::Right;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Keys")
	FKey PitchUpKey = EKeys::Up;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Keys")
	FKey PitchDownKey = EKeys::Down;

	/** Re-sync the IK target to the current end-effector pose (e.g. after the arm drifts). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Keys")
	FKey ResyncTargetKey = EKeys::R;

	/** Enable right-mouse drag pitch / yaw control */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Mouse")
	bool bEnableMouseRotation = true;

	/** Require the right mouse button to be held before mouse motion rotates the target */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Mouse")
	bool bRequireRightMouseButtonForMouseRotation = true;

	/** Invert mouse Y when converting drag into pitch changes */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Mouse")
	bool bInvertMouseY = false;

	/** Mouse yaw sensitivity in degrees per pixel of cursor motion */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Mouse", meta = (ClampMin = "0.001"))
	float MouseYawDegreesPerPixel = 0.2f;

	/** Mouse pitch sensitivity in degrees per pixel of cursor motion */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Mouse", meta = (ClampMin = "0.001"))
	float MousePitchDegreesPerPixel = 0.2f;

	/** Exponential low-pass factor for raw mouse delta (0.01-1). 1 = no smoothing; smaller =
	 *  smoother but more lag. Removes per-pixel sensor noise that otherwise shakes the IK target. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Mouse", meta = (ClampMin = "0.01", ClampMax = "1.0"))
	float MouseSmoothingAlpha = 0.35f;

	/** Draw the current teleop target and related debug helpers every tick. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Debug")
	bool bEnableDebugDisplay = false;

	/** Draw the full target orientation frame. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Debug")
	bool bDrawTargetFrame = false;

	/** Draw a line from the live end effector to the current teleop target. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Debug")
	bool bDrawTargetErrorLine = false;

	/** Draw the live end effector alongside the teleop target. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Debug")
	bool bDrawCurrentEndEffector = false;

	/** Radius of the target debug sphere */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Debug", meta = (ClampMin = "0.1"))
	float DebugTargetSphereRadius = 3.0f;

	/** Length of each debug axis line */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Debug", meta = (ClampMin = "0.1"))
	float DebugAxisLength = 20.0f;

	/** Color used for the teleop target marker */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Debug")
	FColor DebugTargetColor = FColor::Cyan;

	/** Color used for the live end-effector marker */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Debug")
	FColor DebugCurrentPoseColor = FColor::Yellow;

	/** Optional component name override when an actor has multiple gripper controllers */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Gripper")
	FName GripperControllerComponentName = NAME_None;

	/** Enable keyboard control of the gripper controller */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Gripper")
	bool bEnableGripperTeleop = true;

	/** Key that opens the gripper (bracket keys match URammsMjArmTeleopComponent; frees O for Down). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Gripper")
	FKey OpenGripperKey = EKeys::LeftBracket;

	/** Key that closes the gripper */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Gripper")
	FKey CloseGripperKey = EKeys::RightBracket;

	/** Optional key that toggles between open and closed */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Gripper")
	FKey ToggleGripperKey = EKeys::G;

	/** Enable a spring-arm follow camera that tracks the teleop target */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Camera")
	bool bEnableFollowCamera = false;

	/** Optional spring arm component name override for the follow camera rig */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Camera")
	FName FollowSpringArmComponentName = NAME_None;

	/** Optional camera component name override on the owner actor */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Camera")
	FName FollowCameraComponentName = NAME_None;

	/** Automatically switch the local player's view target to the owner actor when a follow camera is found */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Camera")
	bool bAutoActivateFollowCameraView = false;

	/** If true, the spring arm orientation follows the target pose rotation */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Camera")
	bool bFollowCameraTargetRotation = true;

	/** Offset applied to the spring arm root relative to the target pose */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Camera")
	FVector FollowCameraTargetOffset = FVector::ZeroVector;

	/** Interpret FollowCameraTargetOffset in the target's local frame */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Camera")
	bool bFollowCameraOffsetInLocalFrame = true;

	/** Additional camera rig rotation applied after the target rotation */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Teleop|Camera")
	FRotator FollowCameraRotationOffset = FRotator(-20.0f, 0.0f, 0.0f);

	/** Apply normalized teleop input immediately (useful for future SpaceMouse / VR integrations) */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Teleop")
	void ApplyTeleopInput(const FVector& LinearInput, const FRotator& AngularInput, float DeltaTimeSeconds, float SpeedScale = 1.0f);

	/** Snap the current IK target to the live end-effector pose */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Teleop")
	void SyncTargetToCurrentPose();

	/** Enable or disable teleop at runtime */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Teleop")
	void SetTeleopEnabled(bool bEnabled);

	/** Toggle teleop on / off at runtime */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Teleop")
	void ToggleTeleopEnabled();

	/** Open the gripper, if a gripper controller is available */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Teleop")
	void OpenGripper();

	/** Close the gripper, if a gripper controller is available */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Teleop")
	void CloseGripper();

	/** Toggle the gripper, if a gripper controller is available */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Teleop")
	void ToggleGripper();

	/** Get the resolved Kinova controller if one is available */
	UFUNCTION(BlueprintPure, Category = "Ramms|Teleop")
	UKinovaGen3ControllerComponent* GetKinovaController() const
	{
		return KinovaControllerComponent;
	}

	/** Get the resolved gripper controller if one is available */
	UFUNCTION(BlueprintPure, Category = "Ramms|Teleop")
	UGripperControllerComponent* GetGripperController() const
	{
		return GripperControllerComponent;
	}

private:
	UKinovaGen3ControllerComponent* ResolveKinovaController();
	UGripperControllerComponent*	ResolveGripperController();
	void							ResolveFollowCameraComponents();
	void							ApplyKeyboardMouseTeleop(float DeltaTime);
	void							UpdateFollowCamera();
	void							DrawDebugVisualization() const;

	UPROPERTY(Transient)
	UKinovaGen3ControllerComponent* KinovaControllerComponent = nullptr;

	UPROPERTY(Transient)
	UGripperControllerComponent* GripperControllerComponent = nullptr;

	UPROPERTY(Transient)
	USpringArmComponent* FollowSpringArmComponent = nullptr;

	UPROPERTY(Transient)
	UCameraComponent* FollowCameraComponent = nullptr;

	UPROPERTY(Transient)
	bool bLoggedMissingController = false;

	UPROPERTY(Transient)
	bool bLoggedMissingGripperController = false;

	UPROPERTY(Transient)
	bool bLoggedMissingFollowCamera = false;

	UPROPERTY(Transient)
	bool bFollowCameraViewActivated = false;

	/** Diagnostic-only cache: last IK solve count we observed, to detect a stalled solver in DrawDebugVisualization (const). */
	mutable int32 LastObservedIKSolveCount = -1;

	/** Persistent low-pass state for mouse delta smoothing. */
	float SmoothedMouseDeltaX = 0.0f;
	float SmoothedMouseDeltaY = 0.0f;
};
