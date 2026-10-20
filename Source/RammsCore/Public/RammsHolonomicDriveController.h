// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "RammsControlContributor.h"
#include "RammsDriveMode.h"
#include "RammsControlIds.h"
#include "RammsHolonomicDriveTypes.h"

#include "RammsHolonomicDriveController.generated.h"

class URammsRobotBaseComponent;

/**
 * Drives a base whose omni wheels can push in more than one direction, so it
 * can translate sideways as well as forward and turn -- the thing a
 * differential drive cannot do.
 *
 * Like the 5-bar controller, it holds only kinematic configuration and commands
 * motors by Id through the sibling URammsRobotBaseComponent, which owns the
 * physics backend. Nothing here is MuJoCo- or Newton-specific.
 *
 * It advertises drive.forward and drive.turn -- the same Ids the differential
 * drive uses -- so keyboard teleop, the access input and the UI drive it with
 * no change, plus drive.strafe which only a holonomic base can offer. Put one
 * of these OR a differential drive on a robot, not both: they would claim the
 * same controls.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsHolonomicDriveController : public UActorComponent, public IRammsControlContributor, public IRammsDriveMode
{
	GENERATED_BODY()

public:
	URammsHolonomicDriveController();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	/** Optional table (row struct: FRammsHolonomicDriveSpec); overrides Drive. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic")
	TObjectPtr<UDataTable> KinematicTable;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic")
	FName DriveRow;

	/** Used when no table row resolves. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic")
	FRammsHolonomicDriveSpec Drive;

	/**
	 * Body-frame command, each -1..1: X forward, Y strafe (to the robot's
	 * right: the wheel table's +Y, Unreal's sense), Z yaw (positive turns
	 * right). Held until changed, like a stick that stays where it is put.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Holonomic")
	void SetDriveCommand(FVector ForwardStrafeYaw);

	UFUNCTION(BlueprintPure, Category = "Ramms|Holonomic")
	FVector GetDriveCommand() const { return Command; }

	/** Wheel rate (rad/s) each wheel would need for the current command. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Holonomic")
	TArray<float> SolveWheelRates() const;

	UFUNCTION(BlueprintPure, Category = "Ramms|Holonomic")
	bool HasBase() const;

	/**
	 * The stance this mode needs: centre wheels clear of the ground, so the
	 * corner omni wheels are the ones carrying the robot.
	 *
	 * The default asks for a 13 cm 5-bar endpoint height and nothing else.
	 * On the lift-drive that height only clears the centre wheels once the
	 * corner cranks are also driven down (about 1.15 rad) -- with them left
	 * where they are it takes nearer 15 cm, which is close enough to the top
	 * of the 5-bar's travel to be a poor place to operate. So a robot that
	 * needs the cranks says so in its stance; those motor Ids are the
	 * robot's geometry rather than this controller's, and are authored on
	 * its Blueprint.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic")
	FRammsDriveStance LiftedStance;

	/**
	 * How fast (cm/s^2) the body velocity this mode drives may change while
	 * it changes mode: taking off a sideways motion the mode it hands over to
	 * cannot carry, keeping within what the wheels still on the ground carry
	 * while the stance changes, and going from a motion it was handed to its
	 * operator's command (IRammsDriveMode). The rest of the time a command
	 * still applies at once, as it always has.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic|Switching", meta = (ClampMin = "1.0"))
	float HandOverAcceleration = 60.0f;

	/** The same for the yaw rate, deg/s^2. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic|Switching", meta = (ClampMin = "1.0"))
	float HandOverYawAcceleration = 90.0f;

	/**
	 * With no command, bring the robot to rest -- the body velocity ramped to
	 * zero at HandOverAcceleration / HandOverYawAcceleration rather than
	 * dropped -- and hold it there: each wheel's speed loop is kept on and
	 * asked back towards the angle the wheel stopped at, so on a slope the
	 * robot stays where it stopped instead of rolling away (along a slope;
	 * across one the omni wheels' rollers still creep -- HoldsRobotStill is
	 * false). A command takes
	 * the wheels on from there at those same rates, the speed loops' integrals
	 * (what held the slope's pull) carrying on. Nothing is held for the first
	 * second after the mode's first tick: the robot is being put down as it
	 * spawns, its legs jumping into the initial stance. Off, a command
	 * applies at once and the wheels are let go once it is released, as
	 * before.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic|Holding")
	bool bHoldWhenIdle = true;

	// --- IRammsDriveMode ------------------------------------------------------
	virtual FName GetDriveModeId() const override { return FName("holonomic"); }
	virtual FText GetDriveModeDisplayName() const override
	{
		return NSLOCTEXT("Ramms", "DriveModeHolonomic", "Holonomic");
	}
	virtual bool IsDriveModeActive() const override { return bDriveModeActive; }
	virtual void SetDriveModeActive(bool bActive) override;
	virtual bool GetRequiredStance(FRammsDriveStance& OutStance) const override
	{
		OutStance = LiftedStance;
		return !OutStance.IsEmpty();
	}
	/** Brings the body velocity it drives within what Other carries, at
	 *  HandOverAcceleration, the operator's command projected onto it. */
	virtual bool BeginHandOver(const IRammsDriveMode& Other) override;
	virtual void EndHandOver() override;
	/** While held, drives towards rest at HandOverAcceleration whatever is
	 *  commanded (at rest: its wheels held where they are, bHoldWhenIdle, so
	 *  that they touch down still; else nothing); released, goes to the
	 *  command at that rate. */
	virtual void HoldForStance(bool bHold) override;
	/** Starts from Motion (unclamped: it is what the robot is doing) and goes
	 *  to its operator's command -- within any BeginHandOver bound -- at
	 *  HandOverAcceleration; to rest, with no command. Its speed loop holds
	 *  what it is told, so it needs no hold: HoldSeconds is not used. */
	virtual void SeedMotion(const FRammsDriveMotion& Motion, float HoldSeconds) override;
	/** False: with bHoldWhenIdle its wheels are held, which holds the robot
	 *  along a slope, but across one the rollers of its omni wheels -- free
	 *  to turn, and the wheels' motors driving mostly along the robot -- let
	 *  it creep, so a curb crossing that ends on a slope hands the robot back
	 *  to a mode that does hold it still. */
	virtual bool HoldsRobotStill() const override { return false; }
	/** Stood down at rest with bHoldWhenIdle, its wheels held where they are
	 *  until released (their speed loops on, asked back to that angle). */
	virtual void HoldWheelsWhileInactive(bool bHold) override;

	// --- IRammsControlContributor --------------------------------------------
	virtual void  DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool  ApplyControl(FName Id, float Value) override;
	virtual bool  ReleaseControl(FName Id) override;
	virtual bool  ReadControl(FName Id, float& OutValue) const override;
	virtual void  GetClaimedMotorIds(TArray<FName>& OutIds) const override;
	virtual int32 GetControlOrder() const override { return 0; }

private:
	URammsRobotBaseComponent* EnsureBase() const;

	/** Resolve the table row once, else fall back to the inline spec. */
	void ResolveSpec();

	UPROPERTY(Transient)
	mutable TObjectPtr<URammsRobotBaseComponent> BaseComponent = nullptr;

	FRammsHolonomicDriveSpec Resolved;

	/** Commanded body twist, normalised: X forward, Y strafe, Z yaw. */
	FVector Command = FVector::ZeroVector;

	/** Inactive modes contribute nothing: no controls, no claims, no commands. */
	bool bDriveModeActive = true;

	/** True once a non-zero command has been sent, so a released stick still
	 *  writes one zero to the motors rather than leaving them spinning. */
	bool bDrivingMotors = false;

	/** Wheel rates (rad/s) for a body motion (cm/s, deg/s). */
	TArray<float> SolveWheelRatesFor(const FRammsDriveMotion& Motion) const;
	/** Back to rest and to the operator's command alone: no carried motion,
	 *  no hand-over bound, no hold, no ramp. */
	void ResetSwitching();
	/** The physics clock, or the world's when there is none. */
	double Now() const;

	/** The body motion driven last tick (cm/s, deg/s): the command's, or on
	 *  the way to it during a switch. */
	FRammsDriveMotion Output;
	/** The mode whose carriable motion bounds this one's, while BeginHandOver
	 *  holds. */
	TWeakObjectPtr<const UObject> HandOverTarget;
	/** Ramping Output to the command after a switch, until it gets there. */
	bool bOutputRamping = false;
	/** HoldForStance: its wheels are still going down. */
	bool   bHeldForStance = false;
	double LastTickTime = -1.0;

	/** bHoldWhenIdle: the robot has come to rest with nothing commanded, and
	 *  each wheel (Resolved.Wheels order) is held at this angle (rad, the
	 *  motor's sense). */
	bool		   bHoldingStill = false;
	TArray<double> HoldAngles;
	/** Stood down, its wheels held (HoldWheelsWhileInactive). */
	bool bHoldInactive = false;
	/** The clock at its first tick: nothing is held for
	 *  RammsDrive::SpawnSettleSeconds after it (the robot being put down). */
	double FirstTickTime = -1.0;
	/** Latch HoldAngles at the wheels' angles now. */
	void LatchHold(URammsRobotBaseComponent& Base);
	/** Each wheel's speed loop asked back towards its HoldAngles entry. */
	void CommandHold(URammsRobotBaseComponent& Base) const;
};
