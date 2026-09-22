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
	 * Body-frame command, each -1..1: X forward, Y left (strafe), Z yaw.
	 * Held until changed, like a stick that stays where it is put.
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
	 * The default asks for a 13 cm 5-bar endpoint height and nothing else,
	 * which clears the centre wheels on its own. Driving the corner cranks
	 * down as well (about 1.15 rad on the lift-drive) clears them at 10 to
	 * 11 cm instead -- further from the top of the 5-bar's travel, and a
	 * better place to operate. Those motor Ids are the robot's geometry, not
	 * this controller's, so they are authored on the robot's Blueprint.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Holonomic")
	FRammsDriveStance LiftedStance;

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
};
