// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbPreferences.h"

namespace RammsCurb
{
	namespace
	{
		/** Each slider's ends, as factors of the tuning's own value (FRammsCurbPreferences). */
		constexpr double SpeedAtZero = 0.6, SpeedAtOne = 1.3;
		constexpr double BriskFactor = 1.5, GentleFactor = 0.6;
		constexpr double TiltAtOne = 1.6;
		/** MaxPlannedTiltDeg's own ceiling (its ClampMax). */
		constexpr double MaxPlannedTiltCeilingDeg = 10.0;
		/** The share of the seat's forward reach planned with at SeatTravel 0. */
		constexpr double SeatReachShareAtZero = 0.25;

		FRammsControlAxis MakeSlider(FName Id, const FText& Name, int32 Order, float Default = 0.5f)
		{
			FRammsControlAxis Slider;
			Slider.Id = Id;
			Slider.Group = FName("Curb");
			Slider.DisplayName = Name;
			Slider.Kind = ERammsControlKind::Position;
			Slider.Units = ERammsControlUnits::Normalized;
			Slider.Range = FVector2D(0.0, 1.0);
			Slider.DefaultValue = Default;
			Slider.Order = Order;
			return Slider;
		}

		template <typename TPrefs>
		auto Field(TPrefs& Prefs, FName Id) -> decltype(&Prefs.Speed)
		{
			return Id == PreferenceSpeedControlId()		? &Prefs.Speed
				: Id == PreferenceSmoothnessControlId() ? &Prefs.Smoothness
				: Id == PreferenceTiltControlId()		? &Prefs.Tilt
				: Id == PreferenceSeatControlId()		? &Prefs.SeatTravel
														: nullptr;
		}
	} // namespace

	double PreferenceFactor(double Value, double AtZero, double AtOne)
	{
		const double V = FMath::Clamp(Value, 0.0, 1.0);
		return V <= 0.5 ? FMath::Lerp(AtZero, 1.0, V / 0.5) : FMath::Lerp(1.0, AtOne, (V - 0.5) / 0.5);
	}

	FRammsCurbTuning ApplyPreferences(const FRammsCurbTuning& Tuning, const FRammsCurbPreferences& Prefs)
	{
		FRammsCurbTuning Out = Tuning;
		// (Exactly the tuning at the middle: a factor of 1 multiplies nothing away.)
		const double Pace = PreferenceFactor(Prefs.Speed, SpeedAtZero, SpeedAtOne);
		const double Brisk = PreferenceFactor(Prefs.Smoothness, BriskFactor, GentleFactor);
		const double Tilt = PreferenceFactor(Prefs.Tilt, 0.0, TiltAtOne);
		Out.DriveSpeed = static_cast<float>(Tuning.DriveSpeed * Pace);
		Out.Acceleration = static_cast<float>(Tuning.Acceleration * Brisk);
		Out.Deceleration = static_cast<float>(Tuning.Deceleration * Brisk);
		Out.TiltRateDegPerSec = static_cast<float>(Tuning.TiltRateDegPerSec * Brisk);
		Out.TiltAccelDegPerSec2 = static_cast<float>(Tuning.TiltAccelDegPerSec2 * Brisk);
		Out.MaxPlannedTiltDeg = static_cast<float>(FMath::Min(Tuning.MaxPlannedTiltDeg * Tilt, FMath::Max(MaxPlannedTiltCeilingDeg, static_cast<double>(Tuning.MaxPlannedTiltDeg))));
		return Out;
	}

	FName PreferenceSpeedControlId()
	{
		return FName("curb.pref.speed");
	}

	FName PreferenceSmoothnessControlId()
	{
		return FName("curb.pref.smoothness");
	}

	FName PreferenceTiltControlId()
	{
		return FName("curb.pref.tilt");
	}

	FName PreferenceSeatControlId()
	{
		return FName("curb.pref.seat");
	}

	double SeatReachShare(const FRammsCurbPreferences& Prefs)
	{
		return FMath::Lerp(SeatReachShareAtZero, 1.0, FMath::Clamp(static_cast<double>(Prefs.SeatTravel), 0.0, 1.0));
	}

	FVector2D MarginDynamics(const FRammsCurbTuning& Authored, const FRammsCurbTuning& Crossing)
	{
		return FVector2D(FMath::Max(Authored.Acceleration, Crossing.Acceleration), FMath::Max(Authored.Deceleration, Crossing.Deceleration));
	}

	void DescribePreferenceControls(FRammsControlSurface& OutSurface, const FRammsCurbPreferences& Prefs, int32 Order)
	{
		OutSurface.Add(MakeSlider(PreferenceSpeedControlId(), NSLOCTEXT("Ramms", "CurbPrefSpeed", "Crossing speed"), Order));
		OutSurface.Add(MakeSlider(PreferenceSmoothnessControlId(), NSLOCTEXT("Ramms", "CurbPrefSmoothness", "Crossing smoothness"), Order + 1));
		OutSurface.Add(MakeSlider(PreferenceTiltControlId(), NSLOCTEXT("Ramms", "CurbPrefTilt", "Crossing tilt"), Order + 2));
		OutSurface.Add(MakeSlider(PreferenceSeatControlId(), NSLOCTEXT("Ramms", "CurbPrefSeat", "Crossing seat travel"), Order + 3, 1.0f));
	}

	bool ApplyPreferenceControl(FRammsCurbPreferences& Prefs, FName Id, float Value)
	{
		float* F = Field(Prefs, Id);
		if (!F)
		{
			return false;
		}
		*F = FMath::Clamp(Value, 0.0f, 1.0f);
		return true;
	}

	bool ReadPreferenceControl(const FRammsCurbPreferences& Prefs, FName Id, float& OutValue)
	{
		const float* F = Field(Prefs, Id);
		if (!F)
		{
			return false;
		}
		OutValue = *F;
		return true;
	}
} // namespace RammsCurb
