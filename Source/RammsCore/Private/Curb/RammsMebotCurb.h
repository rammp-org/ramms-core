// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Templates/Function.h"

/**
 * The MeBot curb crossing's arithmetic, kept free of the engine so it is tested on its own
 * (URammsMebotCurbController runs it; ramms-sim's mujoco/mebot/curb_sequence.py is the reference).
 */
namespace RammsMebotCurb
{
	/** A support's height table: how far its wheels' bottom rises (cm, chassis level) from the reference
	 *  pose as its joint turns (rad). Heights fall as the angle rises (positive puts the wheels down). */
	struct FSupportTable
	{
		TArray<float> AngleRad;
		TArray<float> HeightCm;

		bool  IsValid() const;
		float MinHeight() const;
		float MaxHeight() const;
		/** The angle for a height, interpolated; false outside the table. */
		bool AngleFor(float HeightCm, float& OutAngleRad) const;
		/** The height at an angle, interpolated and clamped to the table. */
		float HeightAt(float AngleRad) const;
	};

	/**
	 * The chassis's height for a crossing (cm above the lower level, from its reference pose), a
	 * quarter of the way up the band where every support reaches the lower level, a lifted support clears
	 * the upper level by ClearanceCm, and the frame (BellyCm off the ground at the reference pose) clears
	 * the upper level by FrameMarginCm. The band's top is what a sprung support's sag and the touchdowns
	 * eat into, so the height stays near its bottom. False, with the reason, when no height works.
	 */
	bool CrossingHeight(TConstArrayView<const FSupportTable*> Supports, float StepCm, float ClearanceCm, float BellyCm,
		float FrameMarginCm, float& OutHeightCm, FString& OutReason);

	/** Where the ground steps, along a line: SurfaceAt(distance, OutZ) samples it (false: nothing there). */
	struct FStep
	{
		float EdgeDistCm = 0.0f; // from the line's start
		float HeightCm = 0.0f;	 // the far level minus the near one: positive up
		float NearZ = 0.0f;
	};

	/**
	 * The first step of at least MinStepCm whose edge begins between FromCm and ToCm, sampled every StepCm:
	 * a run of intervals steeper than 1:4 (so a ramp is never a step), its height the ground just after the
	 * run minus the ground just before, its edge bisected to a millimetre. The far level must hold, within a
	 * centimetre, for 10 cm past the run. False, with the reason, when there is no step, the ground past it
	 * is not level (a strip, a second step) or the ground is missing at any sample up to there.
	 */
	bool FindStep(TFunctionRef<bool(float DistCm, float& OutZ)> SurfaceAt, float FromCm, float ToCm, float StepCm, float MinStepCm,
		FStep& OutStep, FString& OutReason);

	/** The rule before the rear omni wheels come up, as measured. Pairs are left, right. */
	struct FRearLiftCheck
	{
		// Each drive wheel: its hub past the edge along the heading, and the ground under its hub and its bottom,
		// both from the far level (cm).
		float PastEdgeCm[2] = { 0.0f, 0.0f };
		bool  bGroundFound[2] = { false, false };
		float GroundCm[2] = { 0.0f, 0.0f };
		float GapCm[2] = { 0.0f, 0.0f };
		float StepCm = 0.0f;		   // the ground under a wheel is the far level within half the step
		float TouchToleranceCm = 0.8f; // a bottom this far above the far level is down on it,
		float PreloadCm = 1.0f;		   // and this far (and the tolerance) below it, pressed in
		float TranslatorCm[2] = { 0.0f, 0.0f };
		float AftCm = -30.0f; // where the translators are taken for it
		float AftToleranceCm = 5.0f;
		float ComAheadOfAxleCm = 0.0f; // the centre of mass ahead of the drive axle, along the heading
		float MinComMarginCm = 3.0f;
	};

	/** The rear omni wheels are not lifted until both drive wheels are past the edge, over the far level and
	 *  down on it, translated aft, with the centre of mass ahead of their axle. False, with the reason, when
	 *  not. */
	bool RearLiftAllowed(const FRearLiftCheck& In, FString& OutReason);

	/** The centre of mass along the heading from the chassis-fixed mass and the two drive assemblies
	 *  (which ride the translators), each at its distance along the heading. */
	float CentreOfMass(float FixedMassKg, float FixedDistCm, float DriveAssemblyKg, float LeftDistCm, float RightDistCm);
} // namespace RammsMebotCurb
