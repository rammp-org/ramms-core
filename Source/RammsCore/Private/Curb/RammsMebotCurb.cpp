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
		float NearZ = 0.0f;
		if (!SurfaceAt(FromCm, NearZ))
		{
			OutReason = TEXT("no ground under the start of the scan");
			return false;
		}
		const auto Stepped = [&](float D, float& OutZ) { return SurfaceAt(D, OutZ) && FMath::Abs(OutZ - NearZ) >= MinStepCm; };
		float	   Prev = FromCm;
		for (float D = FromCm + StepCm; D <= ToCm + KINDA_SMALL_NUMBER; D += StepCm)
		{
			float Z = 0.0f;
			if (!Stepped(D, Z))
			{
				Prev = D;
				continue;
			}
			// The edge lies between Prev (near level) and D (stepped): bisect it to a millimetre.
			float A = Prev, B = D;
			while (B - A > 0.1f)
			{
				const float M = 0.5f * (A + B);
				float		Zm = 0.0f;
				(Stepped(M, Zm) ? B : A) = M;
			}
			float FarZ = Z;
			SurfaceAt(B + FMath::Max(5.0f, StepCm), FarZ);
			OutStep.EdgeDistCm = 0.5f * (A + B);
			OutStep.HeightCm = FarZ - NearZ;
			OutStep.NearZ = NearZ;
			return true;
		}
		OutReason = FString::Printf(TEXT("no step of %.0f cm or more within %.0f cm"), MinStepCm, ToCm - FromCm);
		return false;
	}

	bool RearLiftAllowed(const FRearLiftCheck& In, FString& OutReason)
	{
		if (!In.bDriveWheelsDown)
		{
			OutReason = TEXT("the drive wheels are not both down on the far level");
			return false;
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
