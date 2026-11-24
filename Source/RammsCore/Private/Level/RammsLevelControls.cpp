// Copyright Epic Games, Inc. All Rights Reserved.

#include "Level/RammsLevelControls.h"

#include "RammsControlIds.h"
#include "RammsLevelAssistComponent.h"

namespace RammsLevel
{
	FName AssistControlId()
	{
		return FName("level.assist");
	}

	FName StatusControlId()
	{
		return FName("level.status");
	}

	FName StateControlId()
	{
		return FName("level.state");
	}

	FName TiltControlId()
	{
		return FName("level.tilt");
	}

	FName ReachControlId()
	{
		return FName("level.reach");
	}

	FName SpeedScaleControlId()
	{
		return FName("level.speed_scale");
	}

	FName PreferencePitchControlId()
	{
		return FName("level.pref.pitch");
	}

	FName PreferenceRollControlId()
	{
		return FName("level.pref.roll");
	}

	FVector2D ClampPreference(const FVector2D& PreferenceDeg, const FVector2D& MaxPreferenceDeg)
	{
		const FVector2D Finite(FMath::IsFinite(PreferenceDeg.X) ? PreferenceDeg.X : 0.0, FMath::IsFinite(PreferenceDeg.Y) ? PreferenceDeg.Y : 0.0);
		return ClampAxes(Finite, MaxPreferenceDeg);
	}

	FTuning MakeTuning(const FRammsLevelTuning& Tuning, const FVector2D& PreferenceDeg)
	{
		FTuning Out;
		Out.LevelReferenceDeg = Tuning.LevelReferenceDeg + ClampPreference(PreferenceDeg, Tuning.MaxPreferenceDeg);
		Out.FeedForwardGain = Tuning.FeedForwardGain;
		Out.ProportionalGain = Tuning.ProportionalGain;
		Out.IntegralGain = Tuning.IntegralGain;
		Out.IntegralMaxDeg = Tuning.IntegralMaxDeg;
		Out.DeadbandDeg = Tuning.DeadbandDeg;
		Out.AttitudeFilterSeconds = Tuning.AttitudeFilterSeconds;
		Out.TerrainFilterSeconds = Tuning.TerrainFilterSeconds;
		Out.MaxCorrectionDeg = Tuning.MaxCorrectionDeg;
		Out.MaxCorrectionRateDegPerSec = Tuning.MaxCorrectionRateDegPerSec;
		Out.LegRateFraction = Tuning.LegRateFraction;
		Out.MaxTiltDeg = Tuning.MaxTiltDeg;
		Out.TiltRateDegPerSec = Tuning.TiltRateDegPerSec;
		Out.TiltAccelDegPerSec2 = Tuning.TiltAccelDegPerSec2;
		Out.LegRangeMarginCm = Tuning.LegRangeMarginCm;
		Out.SoftZoneCm = Tuning.SoftZoneCm;
		Out.SupportBandInCm = Tuning.SupportBandInCm;
		Out.SupportBandOutCm = Tuning.SupportBandOutCm;
		Out.MinLiftedClearanceCm = Tuning.MinLiftedClearanceCm;
		Out.MinBellyClearanceCm = Tuning.MinBellyClearanceCm;
		Out.RideHeightUpCm = Tuning.RideHeightUpCm;
		Out.RideHeightDownCm = Tuning.RideHeightDownCm;
		Out.RideHeightRateCmPerSec = Tuning.RideHeightRateCmPerSec;
		Out.MinSpeedScale = Tuning.MinSpeedScale;
		Out.SpeedCapErrorDeg = Tuning.SpeedCapErrorDeg;
		Out.SpeedCapErrorFullDeg = Tuning.SpeedCapErrorFullDeg;
		Out.SpeedScaleRisePerSec = Tuning.SpeedScaleRisePerSec;
		Out.SettleErrorDeg = Tuning.SettleErrorDeg;
		Out.SettleSeconds = Tuning.SettleSeconds;
		Out.WakeErrorDeg = Tuning.WakeErrorDeg;
		Out.FaultTiltDeg = Tuning.FaultTiltDeg;
		Out.FaultSeconds = Tuning.FaultSeconds;
		Out.MeasurementLossSeconds = Tuning.MeasurementLossSeconds;
		Out.bSeatCoversTransients = Tuning.bSeatCoversTransients;
		Out.SeatTrimMaxDeg = Tuning.SeatTrimMaxDeg;
		Out.SeatTrimRateDegPerSec = Tuning.SeatTrimRateDegPerSec;
		Out.SeatFilterSeconds = Tuning.SeatFilterSeconds;
		return Out;
	}

	void DescribeControls(FRammsControlSurface& OutSurface, bool bOn, const FVector2D& MaxPreferenceDeg)
	{
		const FName Group = RammsControlIds::Groups::Drive();
		// After the drive mode's own controls in the group.
		int32 Order = 20;

		FRammsControlAxis Switch;
		Switch.Id = AssistControlId();
		Switch.Group = Group;
		Switch.DisplayName = NSLOCTEXT("Ramms", "LevelAssist", "Level assist");
		Switch.Kind = ERammsControlKind::Enum;
		Switch.Units = ERammsControlUnits::None;
		Switch.Order = Order++;
		Switch.EnumLabels = { NSLOCTEXT("Ramms", "LevelAssistOff", "Off"), NSLOCTEXT("Ramms", "LevelAssistOn", "On") };
		Switch.Range = FVector2D(0.0, 1.0);
		Switch.DefaultValue = bOn ? 1.0f : 0.0f;
		OutSurface.Add(Switch);

		FRammsControlAxis Status;
		Status.Id = StatusControlId();
		Status.Group = Group;
		Status.DisplayName = NSLOCTEXT("Ramms", "LevelStatus", "Level status");
		Status.Kind = ERammsControlKind::Text;
		Status.Units = ERammsControlUnits::None;
		Status.bReadOnly = true;
		Status.Order = Order++;
		Status.Range = FVector2D(0.0, static_cast<double>(ERammsControlTextSeverity::Fault));
		OutSurface.Add(Status);

		// The rider's preferred tilt: targets that hold until changed.
		auto Preference = [&](FName Id, const FText& Name, double MaxDeg) {
			FRammsControlAxis Slider;
			Slider.Id = Id;
			Slider.Group = Group;
			Slider.DisplayName = Name;
			Slider.Kind = ERammsControlKind::Position;
			Slider.Units = ERammsControlUnits::Degrees;
			const double Limit = FMath::Max(0.0, MaxDeg);
			Slider.Range = FVector2D(-Limit, Limit);
			Slider.DefaultValue = 0.0f;
			Slider.Order = Order++;
			OutSurface.Add(Slider);
		};
		Preference(PreferencePitchControlId(), NSLOCTEXT("Ramms", "LevelPrefPitch", "Preferred tilt, back (+) or forward"), MaxPreferenceDeg.X);
		Preference(PreferenceRollControlId(), NSLOCTEXT("Ramms", "LevelPrefRoll", "Preferred tilt, sideways"), MaxPreferenceDeg.Y);

		FRammsControlAxis State;
		State.Id = StateControlId();
		State.Group = Group;
		State.DisplayName = NSLOCTEXT("Ramms", "LevelState", "Levelling");
		State.Kind = ERammsControlKind::Enum;
		State.Units = ERammsControlUnits::None;
		State.bReadOnly = true;
		State.Order = Order++;
		// In EState's order: the control's value is the state's number.
		State.EnumLabels = { NSLOCTEXT("Ramms", "LevelStateOff", "Off"), NSLOCTEXT("Ramms", "LevelStateActive", "Active"),
			NSLOCTEXT("Ramms", "LevelStateHolding", "Holding"), NSLOCTEXT("Ramms", "LevelStateReleasing", "Releasing"),
			NSLOCTEXT("Ramms", "LevelStateYielded", "Yielded"), NSLOCTEXT("Ramms", "LevelStateSuspended", "Suspended"),
			NSLOCTEXT("Ramms", "LevelStateFaulted", "Faulted") };
		State.Range = FVector2D(0.0, State.EnumLabels.Num() - 1);
		OutSurface.Add(State);

		auto Value = [&](FName Id, const FText& Name, ERammsControlUnits Units, const FVector2D& Range, float Default) {
			FRammsControlAxis Axis;
			Axis.Id = Id;
			Axis.Group = Group;
			Axis.DisplayName = Name;
			Axis.Kind = ERammsControlKind::Continuous;
			Axis.Units = Units;
			Axis.Range = Range;
			Axis.DefaultValue = Default;
			Axis.bReadOnly = true;
			Axis.Order = Order++;
			OutSurface.Add(Axis);
		};
		Value(TiltControlId(), NSLOCTEXT("Ramms", "LevelTilt", "Base tilt"), ERammsControlUnits::Degrees, FVector2D(0.0, 45.0), 0.0f);
		Value(ReachControlId(), NSLOCTEXT("Ramms", "LevelReach", "Levelling reach left"), ERammsControlUnits::None, FVector2D(0.0, 1.0), 1.0f);
		Value(SpeedScaleControlId(), NSLOCTEXT("Ramms", "LevelSpeedScale", "Level speed limit"), ERammsControlUnits::None, FVector2D(0.0, 1.0), 1.0f);
	}

	const TCHAR* ReachLimitWords(EReachLimit Limit)
	{
		switch (Limit)
		{
			case EReachLimit::LegTravel:
				return TEXT("a leg is at the end of its travel");
			case EReachLimit::LiftedClearance:
				return TEXT("a raised wheel would come too near the ground");
			case EReachLimit::Underside:
				return TEXT("the underside would come too near the ground");
			case EReachLimit::AxisLimit:
				return TEXT("the most correction allowed");
			case EReachLimit::None:
			default:
				return TEXT("");
		}
	}

	FString StatusLine(const FStatusFacts& Facts, ERammsControlTextSeverity& OutSeverity)
	{
		using ESeverity = ERammsControlTextSeverity;
		OutSeverity = ESeverity::Info;
		if (!Facts.CannotLevel.IsEmpty())
		{
			OutSeverity = ESeverity::Fault;
			return TEXT("Level: ") + Facts.CannotLevel;
		}
		switch (Facts.Status)
		{
			case EStatus::Levelling:
				return TEXT("Levelling");
			case EStatus::Settled:
				return TEXT("Level, at rest");
			case EStatus::AtReach:
			{
				// The base is held off level on purpose: say by how much once
				// it is a degree or more, and what stops the legs.
				OutSeverity = ESeverity::Attention;
				FString Line = TEXT("At the end of levelling reach");
				if (Facts.OffReferenceDeg >= 0.95)
				{
					Line += FString::Printf(TEXT(", %.0f deg off"), Facts.OffReferenceDeg);
				}
				if (Facts.ReachLimit != EReachLimit::None)
				{
					Line += FString::Printf(TEXT(": %s"), ReachLimitWords(Facts.ReachLimit));
				}
				return Line;
			}
			case EStatus::PastReach:
			{
				OutSeverity = ESeverity::Attention;
				FString Line = FString::Printf(TEXT("Slope past levelling reach: %.0f deg"), Facts.OffReferenceDeg);
				if (Facts.ReachLimit != EReachLimit::None)
				{
					Line += FString::Printf(TEXT(" (%s)"), ReachLimitWords(Facts.ReachLimit));
				}
				return Line;
			}
			case EStatus::TiltFault:
				OutSeverity = ESeverity::Fault;
				return FString::Printf(TEXT("Tilted %.0f deg: this slope is far past levelling reach"), Facts.OffReferenceDeg);
			case EStatus::Holding:
				return TEXT("Holding: reading the tilt");
			case EStatus::AttitudeLost:
				OutSeverity = ESeverity::Attention;
				return TEXT("Holding: the tilt cannot be read");
			case EStatus::Releasing:
				return Facts.bOn ? TEXT("Returning to the drive stance: not levelling in this drive mode") : TEXT("Returning to the drive stance");
			case EStatus::Yielded:
				return TEXT("Standing by: a curb crossing has the legs");
			case EStatus::Waiting:
				if (Facts.bModeNotLevelled)
				{
					return TEXT("Waiting: not levelling in this drive mode");
				}
				return Facts.bStanceChanging ? TEXT("Waiting: the drive mode is taking its stance") : TEXT("Waiting: the tilt cannot be read yet");
			case EStatus::Diverged:
				OutSeverity = ESeverity::Fault;
				return TEXT("Stopped: levelling was making the tilt worse. Switch it off and on to try again.");
			case EStatus::Off:
			default:
				return TEXT("Off");
		}
	}
} // namespace RammsLevel
