// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Level/RammsLevelTypes.h"
#include "RammsControlTypes.h"

struct FRammsLevelTuning;

/**
 * What the level assist puts on the robot's control surface, and the words of
 * its status: the control ids (a contract with panels, ramms-tools and the
 * runners, so they are literals in one place), the controls themselves, the
 * reflected tuning turned into the law's plain one, and the status line made
 * from the law's status code. Plain C++, so that it can be tested on made-up
 * input. Not reflected, not exported.
 */
namespace RammsLevel
{
	/** Where the level assist's controls sit among the contributors'. */
	constexpr int32 ControlOrder = 6;
	/** The drive-speed scale is sent to the active drive mode every tick
	 *  (IRammsDriveMode::SetDriveSpeedScale, as this requester), to stand this
	 *  long (s of simulated time): a level assist that stops ticking, or
	 *  stops levelling, leaves no cap behind for longer. */
	constexpr float SpeedScaleHoldSeconds = 0.5f;
	FName			SpeedScaleRequester();

	/** level.assist: the Off/On switch. */
	FName AssistControlId();
	/** level.status: what levelling is doing and why, in words. */
	FName StatusControlId();
	/** level.state: the state machine's state (EState), read-only. */
	FName StateControlId();
	/** level.tilt: the base's total tilt (deg), read-only. */
	FName TiltControlId();
	/** level.reach: the least share left (0..1) of whatever is nearest stopping
	 *  the correction -- a leg's planned travel, a raised wheel's clearance,
	 *  the underside's, the most correction allowed on an axis -- read-only. */
	FName ReachControlId();
	/** level.speed_scale: the share of the rider's speed levelling asks the
	 *  drive to keep (its floor..1), read-only. */
	FName SpeedScaleControlId();
	/** level.pref.pitch, level.pref.roll: the tilt the rider prefers to be
	 *  held at (deg; positive nose up, positive with the +Y side up), over
	 *  the level reference. The roll, and on a robot with no seat to hold it
	 *  the pitch, is added to what level means for the base; the pitch is held
	 *  in the seat where there is one (SplitPreference). */
	FName PreferencePitchControlId();
	FName PreferenceRollControlId();
	/** level.seat_trim: what levelling adds to the seat's own tilt (deg, nose
	 *  up against the chassis), read-only: the rider's fore/aft preference,
	 *  a slope past the legs' reach taken back out, and the cover of what the
	 *  base is behind by. Only on a robot whose seat tilts. */
	FName SeatTrimControlId();
	/** level.rider_tilt: the rider's total tilt (deg), read-only: the base's
	 *  attitude with the seat's own pitch and the trim on it, as level.tilt
	 *  is the base's. Only on a robot whose seat tilts. */
	FName RiderTiltControlId();

	/** A preference within what the tuning allows, per axis (a negative limit
	 *  counts as 0; a preference that is not a number as 0). */
	FVector2D ClampPreference(const FVector2D& PreferenceDeg, const FVector2D& MaxPreferenceDeg);

	/**
	 * The law's tuning from the reflected one: every member by its own name,
	 * and the level reference the reflected one's plus PreferenceDeg (clamped
	 * by MaxPreferenceDeg): the part of the rider's preference that the BASE
	 * holds (all of it on a robot with no seat; SplitPreference). A change of
	 * it moves the rider as any change of the wanted attitude does: on the
	 * tilt policy's rate and acceleration.
	 */
	FTuning MakeTuning(const FRammsLevelTuning& Tuning, const FVector2D& PreferenceDeg);

	/** The controls (group Drive): the switch showing bOn, the status, the
	 *  read-only values (the seat's two with bSeat), and the two preference
	 *  sliders within +-MaxPreferenceDeg. */
	void DescribeControls(FRammsControlSurface& OutSurface, bool bOn, const FVector2D& MaxPreferenceDeg, bool bSeat = false);

	/** What the status line is made from. */
	struct FStatusFacts
	{
		EStatus Status = EStatus::Off;
		/** How far the base is off the level reference (deg). */
		double OffReferenceDeg = 0.0;
		/** With a seat that holds part of the rider's attitude: how far the
		 *  RIDER is off what they asked for (deg). Under 0: no such seat, and
		 *  the rider is where the base is. */
		double RiderOffDeg = -1.0;
		/** The robot cannot be levelled at all, and why (plain words; empty:
		 *  it can). */
		FString CannotLevel;
		/** The switch is on. */
		bool bOn = false;
		/** While waiting: the active drive mode is one it does not level in
		 *  (else it waits for the attitude). */
		bool bModeNotLevelled = false;
		/** While waiting: the drive mode's stance is still being taken
		 *  (levelling begins once it has arrived). */
		bool bStanceChanging = false;
		/** At reach: what stops the correction. */
		EReachLimit ReachLimit = EReachLimit::None;
	};

	/** What stops the correction, in plain words, to follow "... reach: ";
	 *  empty for None. */
	const TCHAR* ReachLimitWords(EReachLimit Limit);

	/** The status in plain words, and how much they need the rider. */
	FString StatusLine(const FStatusFacts& Facts, ERammsControlTextSeverity& OutSeverity);
} // namespace RammsLevel
