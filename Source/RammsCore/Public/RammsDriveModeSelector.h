// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "RammsControlContributor.h"
#include "RammsControlIds.h"
#include "RammsDriveMotion.h"
#include "Templates/PimplPtr.h"

#include "RammsDriveModeSelector.generated.h"

class URammsRobotBaseComponent;
class IRammsDriveMode;

namespace RammsDrive
{
	struct FSelectorState;
	class FSelectorHost;
} // namespace RammsDrive

/**
 * Picks which drive mode a robot is running.
 *
 * A lift-drive chassis can run differential or holonomic; both controllers sit
 * on the robot and exactly one is live. This finds every IRammsDriveMode on the
 * actor -- naming no controller class, the way the control surface names no
 * contributor -- and activates one, rebuilding the surface so the inactive
 * modes' shared drive controls disappear from the panel rather than sitting
 * there inert. A mode may keep an entry point of its own; see IRammsDriveMode.
 *
 * Switching is physical as well as logical. The centre wheels are large treaded
 * tyres: holonomic needs them lifted clear on the 5-bar linkages, differential
 * needs them planted so they can overcome the omni wheels' friction. So a
 * switch also commands the linkages to the incoming mode's stance.
 *
 * A switch may happen on the move, and is made so that it can:
 *  - The selector measures how the chassis moves (from the joints the modes'
 *    stances name, which ride on the chassis) and asks the incoming mode what
 *    of that it can carry (IRammsDriveMode::GetCarriableMotion). Whatever it
 *    cannot and the outgoing mode drives -- a differential drive cannot move
 *    sideways -- the outgoing mode brings to zero first (BeginHandOver), and
 *    the selector waits until the measurement shows it, for at most
 *    HandOverTimeoutSeconds; past that the switch is refused and the outgoing
 *    mode stays.
 *  - The stance changes on an S-curve. From one drive mode's stance to
 *    another's it does so in two stages, wheels down before wheels up: where
 *    the incoming stance lowers the linkages (puts the centre wheels down)
 *    they move first and the other motors it names (the corner cranks) after;
 *    where it raises them, the other motors first. The outgoing mode keeps
 *    driving through the first stage -- its wheels still carry -- and hands
 *    over at the end of it, when the incoming mode's wheels are down. The
 *    measurement is watched through that stage too: should the motion the
 *    incoming mode cannot carry come back, the wheels wait where they are,
 *    and the switch is refused once they have waited HandOverTimeoutSeconds.
 *    From a pose no drive mode set (where a curb crossing left the legs, or
 *    the low-level mode) every actuator moves together in one stage.
 *  - At the hand-over the incoming mode is activated and handed the measured
 *    motion (SeedMotion), so the speed carries across -- and so do the
 *    operator's drive commands held on the control surface, which a switch
 *    otherwise drops. Through the rest of the stance change it is held to
 *    what the outgoing mode carries (BeginHandOver): those wheels are still
 *    coming up. An outgoing mode that cannot keep driving (BeginHandOver
 *    false: a curb crossing that has just finished) hands over at once, and
 *    the whole change runs under the incoming mode, held so.
 *  - A refused or cancelled switch brings the legs back to the active mode's
 *    stance from wherever the change had them, the mode held to what the
 *    other one carries until they are.
 * At rest (the measured motion under 2 cm/s and 3 deg/s, or not measurable)
 * the incoming mode takes over at once, as before, while the stance ramps --
 * but it drives nothing until the stance's first stage has put its wheels
 * down (HoldForStance), and nothing the outgoing mode's wheels cannot carry
 * until the change is done. The initial mode's stance at BeginPlay is taken at
 * once: the robot is being put down as it spawns.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsDriveModeSelector : public UActorComponent, public IRammsControlContributor
{
	GENERATED_BODY()

public:
	URammsDriveModeSelector();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Mode to select at BeginPlay. Empty means the first one found. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive Mode")
	FName InitialModeId;

	/** Also drive the 5-bar linkages to the incoming mode's stance. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive Mode")
	bool bApplyLinkageStance = true;

	/**
	 * The least time (simulated seconds) each of a stance change's two stages
	 * takes after the initial one: the actuators of a stage move from where
	 * they are to the stance on one S-curve, all arriving together. 0, with
	 * both rate limits below at 0 too, takes the stance at once, as before.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive Mode|Switching", meta = (ClampMin = "0.0"))
	float StanceRampSeconds = 1.0f;

	/** The fastest (units per simulated second; rad/s for a hinge) a stance
	 *  change moves any motor it names -- the corner cranks -- at the S-curve's
	 *  steepest; a larger move takes longer than StanceRampSeconds. 0 does not
	 *  limit it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive Mode|Switching", meta = (ClampMin = "0.0"))
	float StanceMaxMotorRate = 1.5f;

	/** The fastest (cm per simulated second) a stance change moves a linkage
	 *  endpoint along either axis, at the S-curve's steepest. 0 does not limit
	 *  it. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive Mode|Switching", meta = (ClampMin = "0.0"))
	float StanceMaxLinkageSpeed = 15.0f;

	/** The sideways speed (cm/s, measured at the chassis origin) a mode that
	 *  cannot move sideways is handed the robot at, at most: a switch to it
	 *  waits for the outgoing mode to bring the robot's sideways motion, and
	 *  any yaw rate it cannot turn, below this before its wheels go down. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive Mode|Switching", meta = (ClampMin = "0.0"))
	float HandOverMaxSidewaysSpeed = 2.0f;

	/** How long (simulated seconds) the outgoing mode has to do that before
	 *  the switch is refused, the outgoing mode staying active. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive Mode|Switching", meta = (ClampMin = "0.1"))
	float HandOverTimeoutSeconds = 3.0f;

	/** Ids of the modes found on this robot, in the order they are offered. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Drive Mode")
	TArray<FName> GetAvailableModeIds() const;

	/** The mode active now: during a switch, the outgoing mode until the
	 *  hand-over (IsSwitchingMode). */
	UFUNCTION(BlueprintPure, Category = "Ramms|Drive Mode")
	FName GetActiveModeId() const;

	/**
	 * Switch to a mode by id. False when no such mode is on this robot.
	 * Otherwise the switch is under way: the incoming mode is active when this
	 * returns when the robot is at rest (driving nothing until its wheels are
	 * down), when the outgoing mode cannot keep driving, or when there is no
	 * stance to put its wheels down with and no motion it cannot carry; else
	 * it takes over on a later tick -- once the outgoing mode has brought any
	 * motion it cannot carry to zero and the stance's first stage has put its
	 * wheels down -- or the switch is refused after HandOverTimeoutSeconds.
	 * GetSwitchStatus says which.
	 */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Drive Mode")
	bool SetActiveMode(FName ModeId);

	/** Next mode in the offered order, wrapping. */
	UFUNCTION(BlueprintCallable, Category = "Ramms|Drive Mode")
	bool CycleMode();

	/** True while a switch is waiting to hand over, or the stance is still
	 *  changing after one. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Drive Mode")
	bool IsSwitchingMode() const;

	/** One line on the switch in progress, or on the last one: what it
	 *  waited for, the motion it handed over, how long the stance took, or
	 *  why it was refused. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Drive Mode")
	FString GetSwitchStatus() const;

	/** The chassis' motion as the selector measures it (zero until it can):
	 *  what it hands the incoming mode. */
	UFUNCTION(BlueprintPure, Category = "Ramms|Drive Mode")
	FRammsDriveMotion GetMeasuredMotion() const;

	// --- IRammsControlContributor --------------------------------------------
	virtual void  DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool  ApplyControl(FName Id, float Value) override;
	virtual bool  TriggerControl(FName Id) override;
	virtual bool  ReadControl(FName Id, float& OutValue) const override;
	virtual bool  ReadTarget(FName Id, float& OutTarget) const override { return ReadControl(Id, OutTarget); }
	virtual int32 GetControlOrder() const override { return -10; }
	/** Never: suspending this would remove the control you switch back with. */
	virtual bool CanSuspendContribution() const override { return false; }

private:
	friend class RammsDrive::FSelectorHost;

	/** Components implementing IRammsDriveMode, gathered once at BeginPlay. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UActorComponent>> Modes;

	void GatherModes();
	/** The initial mode, run a tick after BeginPlay so every contributor has
	 *  initialised and the stance has a surface to be commanded through. */
	void SelectInitialMode();

	/** Command a stance outright (the initial mode's, a ramp's end). */
	void ApplyStanceFor(UActorComponent* Mode);

	/** Command every linkage axis whose Id ends in IdSuffix; returns how many
	 *  accepted. Addressed by Id, so no controller class is named here. */
	int32 CommandLinkageAxes(class URammsRobotControlSurfaceComponent& Surface, const TCHAR* IdSuffix,
		float Value, const TCHAR* What, bool bWarn) const;

	int32			 IndexOf(FName ModeId) const;
	IRammsDriveMode* ModeAt(int32 Index) const;
	/** The active mode's index (the switch's), or INDEX_NONE. */
	int32 ActiveIndex() const;

	/** Stand the others down and activate Index, rebuild the surface, and,
	 *  with Seed, carry the operator's held drive commands across and hand
	 *  it the motion (held for HoldSeconds). A line on what it carried. */
	FString ActivateMode(int32 Index, const FRammsDriveMotion* Seed, double HoldSeconds);

	/** The chassis' centre and forward direction (world), from the stances'
	 *  motors; false when it cannot be located. */
	bool LocateChassis(FVector2D& OutCentre, FVector2D& OutForward);
	/** The physics clock, or the world's when there is none. */
	double					  Now() const;
	URammsRobotBaseComponent* EnsureBase() const;

	/** The clock of the last tick (simulated seconds), or negative. */
	double LastTickTime = -1.0;

	/** The chassis motion window, the switch in progress, and what the
	 *  stance ramps last commanded each motor (RammsDriveHandOver.h). */
	TPimplPtr<RammsDrive::FSelectorState> State;

	/** Motors whose joints ride on the chassis (the stances' motor targets),
	 *  and which end each sits at (+1 front, -1 rear), fixed at the first
	 *  measurement, in the robot's frame as it spawned. */
	TArray<FName> ChassisMotors;
	TArray<int32> ChassisMotorEnds;

	UPROPERTY(Transient)
	mutable TObjectPtr<URammsRobotBaseComponent> BaseComponent = nullptr;
};
