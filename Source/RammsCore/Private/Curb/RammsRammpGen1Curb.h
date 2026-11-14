// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Templates/Function.h"

/**
 * The RAMMP Gen1 curb crossing's arithmetic, kept free of the engine so it is tested on its own
 * (URammsRammpGen1CurbController runs it; ramms-sim's mujoco/rammp_gen1/curb_sequence.py is the reference).
 */
namespace RammsRammpGen1Curb
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

	/** The same centre of mass in space: the chassis-fixed mass at FixedAt and the two drive assemblies at
	 *  LeftAt and RightAt; zero with no mass. */
	FVector CentreOfMassAt(float FixedMassKg, const FVector& FixedAt, float DriveAssemblyKg, const FVector& LeftAt,
		const FVector& RightAt);

	/** A wheel as measured: its hub, its radius, and the ground straight under its hub, when a trace found any. */
	struct FWheelSample
	{
		FVector Hub = FVector::ZeroVector;
		float	RadiusCm = 0.0f;
		bool	bGroundFound = false;
		float	GroundZ = 0.0f;

		/** Its bottom over the ground under it (cm, negative: pressed into it); 0 with no ground found. */
		float GapCm() const { return bGroundFound ? static_cast<float>(Hub.Z) - RadiusCm - GroundZ : 0.0f; }
		/** On the ground: there is ground under it, and its bottom is within ToleranceCm of it. */
		bool IsDown(float ToleranceCm) const { return bGroundFound && GapCm() <= ToleranceCm; }
	};

	/** Where the wheels that are down (FWheelSample::IsDown) touch the ground: under each one's hub, at the
	 *  ground's height there. */
	TArray<FVector> GroundContacts(TConstArrayView<FWheelSample> Wheels, float ToleranceCm);

	/** The stability margin with no wheel on the ground: the robot is off it, the lowest margin there is. */
	inline constexpr float NoSupportMarginCm = -1.0e4f;

	/** The stability margin (cm): how far inside the wheels on the ground -- the convex hull of where they touch
	 *  it, Contacts -- the centre of mass stands, straight down (RammsCurb::HullMargin: negative outside it, and
	 *  with fewer than three contacts off one line, at best 0); NoSupportMarginCm with none. */
	float SupportMarginCm(TConstArrayView<FVector2D> Contacts, const FVector2D& ComXY);

	/** An edge's frame, from where two lines parallel to the heading -- HalfTrackCm either side of the robot's centre
	 *  line, left and right -- meet it, each that far along the heading. */
	struct FEdgeFrame
	{
		/** How far off square the edge is (deg): positive when the right line meets it nearer. */
		float OffSquareDeg = 0.0f;
		/** Square to the edge, in the ground plane, onto the far level: the heading turned toward the side that meets
		 *  it nearer (in Unreal's frame, a positive angle about Z turns toward +Y, the right). */
		FVector Normal = FVector::ForwardVector;
	};
	FEdgeFrame EdgeFrame(const FVector& Heading, float LeftDistCm, float RightDistCm, float HalfTrackCm);
} // namespace RammsRammpGen1Curb
