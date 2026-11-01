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
	 * The first step of at least MinStepCm along the line from FromCm to ToCm, sampled every StepCm and
	 * its edge refined by bisection to a millimetre; its height is the far level a little past the edge
	 * minus the level at FromCm. False, with the reason, when there is none or the ground is not found.
	 */
	bool FindStep(TFunctionRef<bool(float DistCm, float& OutZ)> SurfaceAt, float FromCm, float ToCm, float StepCm, float MinStepCm,
		FStep& OutStep, FString& OutReason);

	/** The rule before the rear omni wheels come up, as measured. */
	struct FRearLiftCheck
	{
		bool  bDriveWheelsDown = false; // both on the far level
		float TranslatorCm[2] = { 0.0f, 0.0f };
		float AftCm = -30.0f; // where the translators are taken for it
		float AftToleranceCm = 5.0f;
		float ComAheadOfAxleCm = 0.0f; // the centre of mass ahead of the drive axle, along the heading
		float MinComMarginCm = 3.0f;
	};

	/** The rear omni wheels are not lifted until both drive wheels are on the far level, translated aft
	 *  and down, with the centre of mass ahead of their axle. False, with the reason, when not. */
	bool RearLiftAllowed(const FRearLiftCheck& In, FString& OutReason);

	/** The centre of mass along the heading from the chassis-fixed mass and the two drive assemblies
	 *  (which ride the translators), each at its distance along the heading. */
	float CentreOfMass(float FixedMassKg, float FixedDistCm, float DriveAssemblyKg, float LeftDistCm, float RightDistCm);
} // namespace RammsMebotCurb
