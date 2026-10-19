// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "RammsDriveMotion.h"
#include "RammsDriveStance.h"

#include "RammsDriveMode.generated.h"

UINTERFACE(MinimalAPI)
class URammsDriveMode : public UInterface
{
	GENERATED_BODY()
};

/**
 * One way of driving a base. A robot may carry several -- a differential drive
 * and a holonomic drive on the same chassis -- with exactly one active.
 *
 * They cannot all be live at once: they advertise the same control Ids
 * (drive.forward, drive.turn) and would claim the same motors, so the surface
 * would see duplicates and the motors would be written by two controllers in
 * the same frame. An inactive mode therefore offers none of the controls the
 * modes share, claims no motors and sends no commands.
 *
 * It may keep offering controls under Ids no other mode uses, so long as none
 * of them moves the robot without first selecting the mode. The curb mode's
 * curb.negotiate is one: firing it is how that mode is entered, because the
 * curb mode selected by hand, with no crossing to run, hands straight back.
 *
 * Switching is not only a software matter on this hardware. The lift-drive
 * raises its centre wheels clear of the ground on the 5-bar linkages to run
 * holonomic, and lowers them to run differential -- so a mode also has a stance
 * it needs the robot in. That is why a mode reports its own stance rather than
 * the selector guessing one.
 */
class RAMMSCORE_API IRammsDriveMode
{
	GENERATED_BODY()

public:
	/** Stable id for this mode, e.g. "differential" or "holonomic". */
	virtual FName GetDriveModeId() const = 0;

	/** Human-readable, for the mode control's display. */
	virtual FText GetDriveModeDisplayName() const = 0;

	virtual bool IsDriveModeActive() const = 0;

	/** Take over, or stand down. A mode standing down must stop writing motors
	 *  and stop advertising the controls modes share (see above); the surface
	 *  is rebuilt around it. */
	virtual void SetDriveModeActive(bool bActive) = 0;

	/**
	 * The pose this mode needs the robot standing in, and false when it does
	 * not care.
	 *
	 * Holonomic wants the centre wheels lifted clear; differential wants them
	 * down and carrying load, because they are the treaded tyres that have to
	 * overcome the omni wheels' friction. Which wheels touch depends on the
	 * whole leg -- the 5-bar endpoint height and the corner cranks together --
	 * so this is a stance rather than a single height. See FRammsDriveStance.
	 */
	virtual bool GetRequiredStance(FRammsDriveStance& OutStance) const
	{
		return false;
	}

	// --- Changing mode on the move ---------------------------------------------
	//
	// URammsDriveModeSelector switches modes while the robot moves: the mode
	// coming in is handed the motion the robot has and carries on with it, and
	// the stance changes on a ramp rather than a jump. When the incoming mode
	// cannot carry part of that motion -- a differential drive cannot move its
	// drive axle sideways -- the outgoing one first brings that part to zero.
	// While the stance changes, the mode in charge is held to what the wheels
	// still on the ground carry, and kept still while its own are still going
	// down. All of these have defaults, so a mode that takes no part (it
	// drives nothing, or it runs its own speed control) need not implement any.

	/**
	 * The part of a motion this mode's kinematics can carry: Motion itself
	 * when it can carry anything (the default). A differential drive returns
	 * it without its sideways speed and with its yaw rate within what it can
	 * turn. The selector compares the robot's measured motion with this before
	 * it hands over, and has the outgoing mode remove the difference first.
	 */
	virtual FRammsDriveMotion GetCarriableMotion(const FRammsDriveMotion& Motion) const
	{
		return Motion;
	}

	/**
	 * Drive on, but keep the motion within Other.GetCarriableMotion() -- reached
	 * at a bounded rate, never a step -- until EndHandOver; the operator's
	 * commands still apply to the part Other carries. The selector calls it on
	 * the active mode while some of the robot's weight is, or is about to be,
	 * on wheels Other drives: on the outgoing mode before it hands over (Other
	 * the incoming mode, whose wheels are going down), and on the incoming mode
	 * through the rest of the stance change (Other the outgoing mode, whose
	 * wheels are still coming up). False when this mode cannot (the default):
	 * as the outgoing mode, the selector then hands over at once and says so.
	 */
	virtual bool BeginHandOver(const IRammsDriveMode& Other)
	{
		return false;
	}

	/** The bound BeginHandOver set is over: the stance change is done, or the
	 *  switch refused (the mode stays active). Back to the operator's commands,
	 *  reached at the same bounded rate. (Stood down instead, a mode drops it
	 *  with everything else.) */
	virtual void EndHandOver() {}

	/**
	 * Called on the active mode while a stance change puts the wheels it
	 * drives down: with bHold true, drive nothing -- they are not on the ground
	 * yet, and a wheel spun in the air touches down spinning; with false, they
	 * are down. The selector holds an incoming mode this way through the first
	 * stage of a switch it makes at once (at rest). The default ignores it.
	 */
	virtual void HoldForStance(bool bHold) {}

	/**
	 * Called on the incoming mode right after the selector activates it on the
	 * move: carry on from Motion, the robot's measured motion (within
	 * GetCarriableMotion), instead of starting from rest -- as where the mode
	 * starts, going from it to its operator's command at a bounded rate, not
	 * as a setpoint the operator cannot change. HoldSeconds is how long the
	 * stance change that follows takes, while the wheels this mode drives take
	 * the load: a mode whose command is not a speed (a torque-driven
	 * differential) may pull towards Motion for that long, while its operator
	 * keeps the stick where it was. With no command from the operator, go to
	 * rest from Motion as the mode would: speed carries across a switch, it
	 * does not keep a robot moving that nobody is driving. The default ignores
	 * it (the mode starts from rest, as before).
	 */
	virtual void SeedMotion(const FRammsDriveMotion& Motion, float HoldSeconds) {}
};
