// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "RammsControlContributor.h"
#include "RammsLevelTypes.h"
#include "Templates/PimplPtr.h"

#include "RammsLevelAssistComponent.generated.h"

class URammsRobotBaseComponent;
struct FRammsDriveStance;
namespace RammsLevel
{
	struct FAdoption;
	struct FAssistRuntime;
} // namespace RammsLevel

/*
 * (Design notes, kept out of the class doc comment, which becomes a tooltip.)
 *
 * The level assist keeps the rider level while the robot stands or drives on
 * sloping ground. It never writes a leg's own command: the drive modes, the
 * 5-bar controllers, the curb controller and the rider go on writing those
 * (the BASELINES, URammsRobotBaseComponent::SetMotorCommand). It adds one
 * rigid plane of OFFSETS on top, through the base's command offset layer
 * (ClaimCommandOffsetLayer / SetMotorCommandOffset), which it alone holds, and
 * the base tells each leg motor baseline plus offset once a frame.
 *
 * This component is a thin shell. The law is plain C++ in Private/Level
 * (RammsLevel::FLaw::Step, with its state machine RammsLevel::FArbiter): each
 * tick the shell reads the chassis' attitude and the legs (a plant,
 * RammsLevel::IPlant, picked by which sibling describes the robot: the Gen2
 * curb controller's legs today), tells the law the switch, the active drive
 * mode's kind and anything a curb controller has asked (YieldTo, ResumeFrom),
 * and carries out what comes back: the legs' offsets, and the one-shot actions
 * that touch motors (claim, fold, zero, release).
 *
 * "Level" is Tuning.LevelReferenceDeg plus the rider's preferred tilt
 * (level.pref.pitch, level.pref.roll, within Tuning.MaxPreferenceDeg). A
 * change of the preference moves the rider as switching on does: within the
 * tilt policy's rate and acceleration.
 *
 * The seat: on a robot whose seat tilts (the curb controller describes one
 * with a tilt motor) and with Tuning.bSeatCoversTransients, the LEGS level the
 * base and the SEAT does the rest of the rider's pitch, as a trim: an offset
 * over the seat's own tilt command, which the seat's controller, a drive
 * stance and a crossing go on writing (seat.tilt stays the rider's own
 * setting against the chassis). The trim is three things. The pitch the rider
 * prefers (level.pref.pitch) is held there, over a base that stays level, so
 * that it costs the legs no reach; the roll they prefer stays the base's (a
 * seat has no roll axis). What a slope past the legs' reach leaves the base
 * tilted by is taken back out there. And what the base is behind its wanted
 * attitude by, while the legs catch the ground up, is covered there. The
 * first two are changes of the rider's attitude and move within the tilt
 * policy; the cover moves at Tuning.SeatTrimRateDegPerSec; all of it stays
 * within Tuning.SeatTrimMaxDeg and the seat's own range. What the body does
 * by itself (a knock, braking) is not answered: a seat sent after it arrives
 * as the base comes back. level.tilt is the base's tilt; level.rider_tilt the
 * rider's, with the seat's own setting and the trim; level.seat_trim the
 * trim. On a robot with no such seat the whole preference is the base's, and
 * so is whatever of the fore/aft preference the seat has no room for (its
 * trim limit, a seat the rider has tilted near its stop): the preference is
 * held, by the legs where the seat cannot, and never dropped.
 * A take-over of the legs (a crossing, a mode that drives them) leaves the
 * trim on the seat as it is: whoever takes the legs has not taken the seat.
 * It is folded into the seat's own command only when the taker takes the
 * seat itself (YieldSeatTo), and levelling goes on from whatever the seat
 * still carries when the legs come back. Switched off while standing by, the
 * trim is walked off the seat at the tilt policy's rate, never dropped.
 * A robot turning on the spot on a slope sees the slope swing from pitch into
 * roll with no distance travelled; the chassis' turning rate is read with its
 * attitude so that the legs follow that as they follow ground driven over.
 *
 * Where it levels: in the drive modes of EnabledModeIds (empty: every mode)
 * that are not in DisabledModeIds. A mode it does not level in that takes the
 * leg motors for itself (the curb mode) gets them at once, the offsets folded
 * into the baselines so that nothing moves; any other (low level) is given
 * the base back gently, within the tilt policy. A curb controller announces
 * its take-over itself (YieldTo), before it reads a command; the tick's check
 * of the active mode is the backstop for a path that forgot.
 *
 * Ticks in TG_PrePhysics. While it holds the offset layer (and only then: a
 * level assist that is off asks for no tick order, so the robot ticks as it
 * would with none) it ticks after every component that writes a leg's
 * baseline and before the robot base, which tells the motors: an offset is
 * then worked out from this frame's baselines and told with them. A writer
 * that ticks in a later group (the 5-bar, seat and Gen1 lift controllers
 * tick during physics) is NOT made a prerequisite: that would move this
 * tick, and the base's behind it, out of the group before physics. What such
 * a writer commands in its own tick is told a frame late, with its offset.
 * It also means that, for as long as the layer is held (a crossing it stands
 * by for included: the layer is kept, with every offset zero or folded), a
 * command to a levelled motor -- a leg's, the seat's tilt -- is told at the
 * robot base's tick of that frame and not in the call that wrote it, while a
 * command to any other motor (a wheel, the seat's two link motors) is told at
 * once as ever. A simulator stepped once a frame after every tick has run
 * (frame-locked) sees them all before its next step; one stepping freely on
 * its own thread may take one step between the two.
 *
 * A drive mode switch: levelling is not begun, and a release does not begin
 * to give the base back, while the mode selector is still changing the stance
 * (URammsDriveModeSelector::IsSwitchingMode): the stance change rocks the base
 * onto other wheels by itself, and the rider is turned by one thing at a time.
 *
 * A curb crossing: the curb controller yields before it reads a command and
 * hands back (ResumeFrom) with the stance of the drive mode it hands the robot
 * to. The crossing leaves the base level in a pose of its own; the plane by
 * which that pose differs from the stance is taken back out of the legs'
 * commands and carried on as the offsets, the motors told nothing, so that the
 * selector's stance ramp is left only a ride height to change and the base
 * stays level through it.
 *
 * The drive's pace: while the legs carry a correction, the drive mode that is
 * driving is asked every tick to keep to the speed scale the law works out
 * (IRammsDriveMode::SetDriveSpeedScale, to stand half a second of simulated
 * time; level.speed_scale shows it): the speed at which the ground turns under
 * the robot no faster than its legs can follow. Never a mode that takes the
 * legs (a curb crossing runs its own pace), and taken back at once when
 * levelling yields. When levelling stops (switched off, a mode it does not
 * level in) the request goes on with the scale as it rises back to 1, and
 * only then lapses: dropped at once it would step the drive up to full speed.
 *
 * The ride height: with Tuning.RideHeightUpCm / RideHeightDownCm above 0 the
 * chassis is raised or lowered from the stance's height, at
 * RideHeightRateCmPerSec, where that makes a correction fit that would not
 * otherwise (a wheel the stance carries off the ground kept clear of a slope
 * rising ahead), and brought back when it is no longer needed.
 *
 */
/**
 * Keeps the rider level on sloping ground by adding one rigid plane of offsets to the legs' commands (see the
 * design notes above). Switched by the level.assist control.
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSCORE_API URammsLevelAssistComponent : public UActorComponent, public IRammsControlContributor
{
	GENERATED_BODY()

public:
	URammsLevelAssistComponent();
	// Out of line: the runtime state is an incomplete type here.
	virtual ~URammsLevelAssistComponent() override;

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level")
	FRammsLevelTuning Tuning;

	/** Drive modes (by id) it never levels in. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level")
	TArray<FName> DisabledModeIds = { FName("curb"), FName("low_level") };

	/** Drive modes (by id) it levels in; empty: every mode not disabled. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level")
	TArray<FName> EnabledModeIds;

	/** Whether level.assist starts On. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level")
	bool bEnabledAtStart = false;

	/** Where the chassis' attitude is read from. Only Exact is built. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level")
	ERammsLevelAttitudeSource AttitudeSource = ERammsLevelAttitudeSource::Exact;

	/** The IMU sensor component read with AttitudeSource ImuSensor. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level")
	FName ImuComponentName;

	// --- plain C++, for the curb controllers and for tests --------------------

	/**
	 * A controller (Taker) is about to take the leg motors with commands of its
	 * own: the offsets go into the baselines now, the motors told nothing, so
	 * that the commands Taker reads are the pose the robot is in. Call before
	 * reading any command. Levelling then stands by until ResumeFrom, or until
	 * Taker's drive mode is no longer active. The legs only: the seat's trim
	 * stays on the seat, as it is, until Taker takes the seat too
	 * (YieldSeatTo) or levelling goes on from it.
	 */
	void YieldTo(const UObject* Taker);

	/**
	 * Taker, which has the legs, is about to command the seat's tilt with a
	 * command of its own (a crossing holding the rider's pitch): the seat's
	 * trim goes into the tilt's own command now, the motor told nothing, and
	 * GetFoldedSeatTrimDeg says how much. Call before reading the tilt's
	 * command. Nothing while levelling is not standing by, with no trim on,
	 * and on a robot with no seat. A taker that never commands the seat never
	 * calls this, and the trim is then still on the seat when it hands back.
	 */
	void YieldSeatTo(const UObject* Taker);

	/**
	 * Taker gives the legs back. Call before the drive mode it hands the robot
	 * to is selected: the mode selector's stance ramp starts each leg from the
	 * command this leaves it.
	 *
	 * With no NextStance, and nothing written since its YieldTo (a take-over
	 * refused before it began), everything is put back exactly as it was.
	 * Otherwise levelling goes on from the pose Taker left, no motor told
	 * anything: the plane by which the pose differs from NextStance (the
	 * stance the legs are about to be ramped to; with none, from the commands
	 * as they were before Taker had them) is taken back out of the legs'
	 * commands and becomes the offsets. Where that cannot be done levelling
	 * starts again from nothing on the pose as it stands. A level assist that
	 * is switched off does nothing at all.
	 *
	 * Taker is whoever called YieldTo -- or, with a NextStance, a drive mode
	 * that has the legs without having called it (its mode picked by hand: the
	 * level assist's own tick stood by for it).
	 */
	void ResumeFrom(const UObject* Taker, const FRammsDriveStance* NextStance);

	/** Offsets may be on the legs now. */
	bool IsLevelling() const;

	/** The attitude the BASE is being moved to and held at (deg; pitch,
	 *  roll): the law's wanted attitude. (The rider's differs by the pitch
	 *  the seat holds, on a robot whose seat tilts.) */
	FVector2D GetTargetAttitudeDeg() const;

	/** The seat trim (deg, nose up against the chassis) that a YieldSeatTo
	 *  folded into the seat tilt's own command and that is in it still: what
	 *  a taker takes off the pitch it reads back to have the rider's own
	 *  setting. 0 once the tilt's command is no longer the one the fold left
	 *  (something has moved the seat since), after a hand-back that took it
	 *  back out, and on a robot with no seat. */
	float GetFoldedSeatTrimDeg() const;

	/**
	 * After a ResumeFrom with a NextStance, until this component next ticks:
	 * how much of the plane by which the pose differed from that stance was
	 * left in the legs' commands (deg; pitch, roll), which is what the stance
	 * ramp that follows will turn the base by. Zero where the whole plane was
	 * taken back out (the ramp then changes a ride height and no attitude);
	 * the rest where the plane was past what levelling may carry (its
	 * per-axis limit, the legs' reach in that stance). False, OutLeftDeg
	 * untouched, when that hand-back took no plane back out at all (the whole
	 * pose is in the commands still: the ramp turns the base to the ground's
	 * attitude, as with no level assist), or none was made.
	 */
	bool GetHandBackTurnDeg(FVector2D& OutLeftDeg) const;

	/** The switch (level.assist). Refused, staying off, on a robot that cannot
	 *  be levelled. */
	void SetAssistEnabled(bool bEnabled);
	bool IsAssistEnabled() const;

	/** The rider's preferred tilt (deg; pitch, roll), kept within
	 *  Tuning.MaxPreferenceDeg. */
	void	  SetPreferenceDeg(const FVector2D& PreferenceDeg);
	FVector2D GetPreferenceDeg() const;

	/** The words level.status shows, and how much they need the rider. */
	FString GetStatusLine(ERammsControlTextSeverity& OutSeverity) const;

	// --- IRammsControlContributor ----------------------------------------------
	virtual void  DescribeControls(FRammsControlSurface& OutSurface) const override;
	virtual bool  ApplyControl(FName Id, float Value) override;
	virtual bool  TriggerControl(FName Id) override;
	virtual bool  ReadControl(FName Id, float& OutValue) const override;
	virtual bool  ReadTarget(FName Id, float& OutTarget) const override;
	virtual bool  ReadControlText(FName Id, FText& OutText) const override;
	virtual int32 GetControlOrder() const override;

private:
	URammsRobotBaseComponent* EnsureBase() const;

	/** Find the robot's description (which sibling describes it) and build the
	 *  plant and the attitude source; once, on the first tick, when every
	 *  component has begun play. */
	void ResolveRobot();

	/** Tick after every component that writes a leg's baseline (and ticks
	 *  before physics), and before the base: asked for when the offset layer
	 *  is claimed, and taken back when it is given up. */
	void AddTickPrerequisites();
	void RemoveTickPrerequisites();

	/** The claim on the base's command offset layer, taken and given up as
	 *  the law asks. */
	bool ClaimMotors();
	void ReleaseMotors();

	/** One step of the law over Dt, the motors' part of what it answers
	 *  carried out. bYield / bResume: a taker's call, made now; Adoption: the
	 *  plane a hand-back took back out of the legs' commands (AdoptPose). */
	void StepLaw(double Dt, bool bYield, bool bResume, const RammsLevel::FAdoption* Adoption = nullptr);

	/** A hand-back's plane, taken back out of the legs' commands: against
	 *  NextStance, or with none against the commands the last fold remembered.
	 *  False, with nothing changed, when there is none to take (switched off,
	 *  not standing by, the legs' offsets not held) or it cannot be worked out. */
	bool AdoptPose(const FRammsDriveStance* NextStance, RammsLevel::FAdoption& OutAdoption);

	/** A seat trim that was folded into the seat tilt's command for a taker
	 *  that never wrote the tilt after all, taken back out as the seat's
	 *  offset (while on, standing by, the offsets held). False when there was
	 *  none to take. */
	bool TakeBackSeatTrim();

	/** The seat's trim as shown (level.seat_trim): the law's while it levels,
	 *  and what is still on the seat while it stands by. */
	double SeatTrimShownDeg() const;

	UPROPERTY(Transient)
	mutable TObjectPtr<URammsRobotBaseComponent> BaseComponent = nullptr;

	/** Everything kept between ticks that is not configuration. */
	TPimplPtr<RammsLevel::FAssistRuntime> Runtime;
};
