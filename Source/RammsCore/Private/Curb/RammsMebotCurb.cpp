// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsMebotCurb.h"

namespace RammsMebotCurb
{
	bool FSupportTable::IsValid() const
	{
		return AngleRad.Num() >= 2 && AngleRad.Num() == HeightCm.Num();
	}

	float FSupportTable::MinHeight() const
	{
		return IsValid() ? FMath::Min(HeightCm[0], HeightCm.Last()) : 0.0f;
	}

	float FSupportTable::MaxHeight() const
	{
		return IsValid() ? FMath::Max(HeightCm[0], HeightCm.Last()) : 0.0f;
	}

	bool FSupportTable::AngleFor(float Height, float& OutAngleRad) const
	{
		if (!IsValid())
		{
			return false;
		}
		for (int32 i = 1; i < HeightCm.Num(); ++i)
		{
			const float H0 = HeightCm[i - 1], H1 = HeightCm[i];
			if ((Height - H0) * (Height - H1) <= 0.0f)
			{
				const float A = FMath::IsNearlyEqual(H0, H1) ? 0.0f : (Height - H0) / (H1 - H0);
				OutAngleRad = FMath::Lerp(AngleRad[i - 1], AngleRad[i], A);
				return true;
			}
		}
		return false;
	}

	float FSupportTable::HeightAt(float Angle) const
	{
		if (!IsValid())
		{
			return 0.0f;
		}
		if (Angle <= AngleRad[0])
		{
			return HeightCm[0];
		}
		for (int32 i = 1; i < AngleRad.Num(); ++i)
		{
			if (Angle <= AngleRad[i])
			{
				const float Span = AngleRad[i] - AngleRad[i - 1];
				return FMath::Lerp(HeightCm[i - 1], HeightCm[i], Span > 0.0f ? (Angle - AngleRad[i - 1]) / Span : 0.0f);
			}
		}
		return HeightCm.Last();
	}

	bool CrossingHeight(TConstArrayView<const FSupportTable*> Supports, float StepCm, float ClearanceCm, float BellyCm,
		float FrameMarginCm, float& OutHeightCm, FString& OutReason)
	{
		const float Step = FMath::Abs(StepCm);
		float		Lo = Step + FrameMarginCm - BellyCm;
		float		Hi = TNumericLimits<float>::Max();
		for (const FSupportTable* S : Supports)
		{
			if (!S || !S->IsValid())
			{
				OutReason = TEXT("a support has no height table");
				return false;
			}
			Hi = FMath::Min(Hi, -S->MinHeight());					  // reaches the lower level
			Lo = FMath::Max(Lo, Step + ClearanceCm - S->MaxHeight()); // lifted, clears the upper one
		}
		if (Lo > Hi)
		{
			OutReason = FString::Printf(TEXT("no crossing height works for a %.1f cm step: the frame and the lifts need %.1f cm, the supports reach the lower level only from %.1f cm"),
				Step, Lo, Hi);
			return false;
		}
		OutHeightCm = Lo + 0.25f * (Hi - Lo);
		return true;
	}

	bool FindStep(TFunctionRef<bool(float, float&)> SurfaceAt, float FromCm, float ToCm, float StepCm, float MinStepCm, FStep& OutStep,
		FString& OutReason)
	{
		// An edge is a run of sample intervals steeper than 1:4, all rising or all falling: a curb's face is near
		// vertical, a ramp 1:12 or gentler, so a ramp never adds up to a step however long it runs. Its height is
		// the run's rise, from the ground just before it to the ground just after, and the far level must hold for
		// LevelCheckCm past it: a strip, or a second step, is not a curb.
		constexpr float SteepSlope = 0.25f;
		constexpr float LevelCheckCm = 10.0f;
		constexpr float LevelToleranceCm = 1.0f;
		const auto		Sample = [&](float D, float& OutZ) {
			 if (SurfaceAt(D, OutZ))
			 {
				 return true;
			 }
			 OutReason = FString::Printf(TEXT("no ground %.0f cm along the scan"), D);
			 return false;
		};
		float PrevD = FromCm, PrevZ = 0.0f;
		if (!Sample(FromCm, PrevZ))
		{
			return false;
		}
		float RunD = FromCm, RunZ = PrevZ; // where the current run began
		int32 RunSign = 0;				   // +1 rising, -1 falling, 0 no run
		bool  bFound = false;
		for (float D = FromCm + StepCm; D <= ToCm + LevelCheckCm + 100.0f; D += StepCm)
		{
			if (RunSign == 0 && PrevD >= ToCm - KINDA_SMALL_NUMBER)
			{
				break; // no edge begins within the scan
			}
			float Z = 0.0f;
			if (!Sample(D, Z))
			{
				return false;
			}
			const float Dz = Z - PrevZ;
			const int32 Sign = FMath::Abs(Dz) > SteepSlope * StepCm ? (Dz > 0.0f ? 1 : -1) : 0;
			if (Sign != 0 && Sign == RunSign)
			{
				PrevD = D;
				PrevZ = Z;
				continue; // the run goes on
			}
			if (RunSign != 0 && FMath::Abs(PrevZ - RunZ) >= MinStepCm)
			{
				bFound = true; // the run ended at PrevD, high enough to be a step
				break;
			}
			RunSign = Sign; // a run, or level ground, from PrevD
			RunD = PrevD;
			RunZ = PrevZ;
			PrevD = D;
			PrevZ = Z;
		}
		if (!bFound)
		{
			OutReason = FString::Printf(TEXT("no step of %.0f cm or more within %.0f cm"), MinStepCm, ToCm - FromCm);
			return false;
		}
		const float NearZ = RunZ, FarZ = PrevZ;
		for (float D = PrevD + StepCm; D <= PrevD + LevelCheckCm + KINDA_SMALL_NUMBER; D += StepCm)
		{
			float Z = 0.0f;
			if (!Sample(D, Z))
			{
				return false;
			}
			if (FMath::Abs(Z - FarZ) > LevelToleranceCm)
			{
				OutReason = FString::Printf(TEXT("the ground %.0f cm past the %.1f cm edge at %.0f cm is %.1f cm off its level: not a curb"),
					D - PrevD, FarZ - NearZ, RunD, Z - FarZ);
				return false;
			}
		}
		// The edge lies in the run: bisect it, to a millimetre, where the ground is past half the rise.
		const float Half = 0.5f * FMath::Abs(FarZ - NearZ);
		float		A = RunD, B = PrevD;
		while (B - A > 0.1f)
		{
			const float M = 0.5f * (A + B);
			float		Zm = 0.0f;
			if (!Sample(M, Zm))
			{
				return false;
			}
			((Zm - NearZ) * RunSign >= Half ? B : A) = M;
		}
		OutStep.EdgeDistCm = 0.5f * (A + B);
		OutStep.HeightCm = FarZ - NearZ;
		OutStep.NearZ = NearZ;
		return true;
	}

	bool RearLiftAllowed(const FRearLiftCheck& In, FString& OutReason)
	{
		for (int32 i = 0; i < 2; ++i)
		{
			const TCHAR* Side = i ? TEXT("right") : TEXT("left");
			if (In.PastEdgeCm[i] < 0.0f)
			{
				OutReason = FString::Printf(TEXT("the %s drive wheel is %.1f cm short of the edge"), Side, -In.PastEdgeCm[i]);
				return false;
			}
			if (!In.bGroundFound[i] || FMath::Abs(In.GroundCm[i]) > 0.5f * FMath::Abs(In.StepCm))
			{
				OutReason = In.bGroundFound[i]
					? FString::Printf(TEXT("the ground under the %s drive wheel is %.1f cm off the far level"), Side, In.GroundCm[i])
					: FString::Printf(TEXT("no ground under the %s drive wheel"), Side);
				return false;
			}
			if (In.GapCm[i] > In.TouchToleranceCm || In.GapCm[i] < -(In.PreloadCm + In.TouchToleranceCm))
			{
				OutReason = FString::Printf(TEXT("the %s drive wheel is %.1f cm %s the far level, not down on it"), Side, FMath::Abs(In.GapCm[i]),
					In.GapCm[i] > 0.0f ? TEXT("above") : TEXT("below"));
				return false;
			}
		}
		for (int32 i = 0; i < 2; ++i)
		{
			if (In.TranslatorCm[i] > In.AftCm + In.AftToleranceCm)
			{
				OutReason = FString::Printf(TEXT("the %s translator is at %.1f cm, not aft (%.0f cm)"), i ? TEXT("right") : TEXT("left"),
					In.TranslatorCm[i], In.AftCm);
				return false;
			}
		}
		if (In.ComAheadOfAxleCm < In.MinComMarginCm)
		{
			OutReason = FString::Printf(TEXT("the centre of mass is %.1f cm ahead of the drive axle (needs %.1f)"), In.ComAheadOfAxleCm,
				In.MinComMarginCm);
			return false;
		}
		return true;
	}

	float CentreOfMass(float FixedMassKg, float FixedDistCm, float DriveAssemblyKg, float LeftDistCm, float RightDistCm)
	{
		const float Total = FixedMassKg + 2.0f * DriveAssemblyKg;
		return Total > 0.0f ? (FixedMassKg * FixedDistCm + DriveAssemblyKg * (LeftDistCm + RightDistCm)) / Total : 0.0f;
	}
} // namespace RammsMebotCurb
