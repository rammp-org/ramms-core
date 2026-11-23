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

	/** level.assist: the Off/On switch. */
	FName AssistControlId();
	/** level.status: what levelling is doing and why, in words. */
	FName StatusControlId();
	/** level.state: the state machine's state (EState), read-only. */
	FName StateControlId();
	/** level.tilt: the base's total tilt (deg), read-only. */
	FName TiltControlId();
	/** level.reach: the least share of travel left to a leg (0..1), read-only. */
	FName ReachControlId();
	/** level.speed_scale: the share of the rider's speed levelling asks the
	 *  drive to keep (its floor..1), read-only. */
	FName SpeedScaleControlId();
	/** level.pref.pitch, level.pref.roll: the tilt the rider prefers to be
	 *  held at (deg; positive nose up, positive with the +Y side up), added to
	 *  the level reference. */
	FName PreferencePitchControlId();
	FName PreferenceRollControlId();

	/** A preference within what the tuning allows, per axis (a negative limit
	 *  counts as 0; a preference that is not a number as 0). */
	FVector2D ClampPreference(const FVector2D& PreferenceDeg, const FVector2D& MaxPreferenceDeg);

	/**
	 * The law's tuning from the reflected one: every member by its own name,
	 * and the level reference the reflected one's plus the rider's preference
	 * (clamped by MaxPreferenceDeg). That sum is the only place the preference
	 * enters the law, so a change of it moves the rider as any change of the
	 * wanted attitude does: on the tilt policy's rate and acceleration.
	 */
	FTuning MakeTuning(const FRammsLevelTuning& Tuning, const FVector2D& PreferenceDeg);

	/** The controls (group Drive): the switch showing bOn, the status, the
	 *  read-only values, and the two preference sliders within
	 *  +-MaxPreferenceDeg. */
	void DescribeControls(FRammsControlSurface& OutSurface, bool bOn, const FVector2D& MaxPreferenceDeg);

	/** What the status line is made from. */
	struct FStatusFacts
	{
		EStatus Status = EStatus::Off;
		/** How far the base is off the level reference (deg). */
		double OffReferenceDeg = 0.0;
		/** The robot cannot be levelled at all, and why (plain words; empty:
		 *  it can). */
		FString CannotLevel;
		/** The switch is on. */
		bool bOn = false;
		/** While waiting: the active drive mode is one it does not level in
		 *  (else it waits for the attitude). */
		bool bModeNotLevelled = false;
	};

	/** The status in plain words, and how much they need the rider. */
	FString StatusLine(const FStatusFacts& Facts, ERammsControlTextSeverity& OutSeverity);
} // namespace RammsLevel
