// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsControlTypes.h"
#include "RammsCurbNegotiationTypes.h"

struct FRammsControlSurface;

/**
 * A rider's crossing preferences (FRammsCurbPreferences) applied to a curb controller's tuning, and their sliders on
 * the control surface. Plain C++, testable on made-up input. Not reflected, not exported.
 */
namespace RammsCurb
{
	/** The tuning a crossing is planned and run with: Tuning as FRammsCurbPreferences describes it shaped by Prefs
	 *  (each slider clamped to 0..1; 0.5 everywhere gives Tuning back unchanged). */
	FRammsCurbTuning ApplyPreferences(const FRammsCurbTuning& Tuning, const FRammsCurbPreferences& Prefs);

	/** The factor a slider at Value (0..1) gives: AtZero at 0, 1 at 0.5, AtOne at 1, linear either side. */
	double PreferenceFactor(double Value, double AtZero, double AtOne);

	/** curb.pref.speed, curb.pref.smoothness, curb.pref.tilt. */
	FName PreferenceSpeedControlId();
	FName PreferenceSmoothnessControlId();
	FName PreferenceTiltControlId();

	/** The three sliders (group Curb, Position controls 0..1, default 0.5) showing Prefs, from Order on. */
	void DescribePreferenceControls(FRammsControlSurface& OutSurface, const FRammsCurbPreferences& Prefs, int32 Order);

	/** Set the slider Id names to Value (clamped to 0..1); false for an Id that is none of them. */
	bool ApplyPreferenceControl(FRammsCurbPreferences& Prefs, FName Id, float Value);

	/** The slider Id names' value; false for an Id that is none of them. */
	bool ReadPreferenceControl(const FRammsCurbPreferences& Prefs, FName Id, float& OutValue);
} // namespace RammsCurb
