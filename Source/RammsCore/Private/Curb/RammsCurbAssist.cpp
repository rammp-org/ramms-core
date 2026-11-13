// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbAssist.h"

#include "RammsCurbNegotiationTypes.h"

namespace RammsCurb
{
	FName AssistControlId()
	{
		return FName("curb.assist");
	}

	FRammsControlAxis MakeAssistControl(bool bOn, int32 Order)
	{
		FRammsControlAxis Switch;
		Switch.Id = AssistControlId();
		Switch.Group = FName("Curb");
		Switch.DisplayName = NSLOCTEXT("Ramms", "CurbAssist", "Curb assist");
		Switch.Kind = ERammsControlKind::Enum;
		Switch.Units = ERammsControlUnits::None;
		Switch.Order = Order;
		Switch.EnumLabels = { NSLOCTEXT("Ramms", "CurbAssistOff", "Off"), NSLOCTEXT("Ramms", "CurbAssistOn", "On") };
		Switch.Range = FVector2D(0.0, 1.0);
		Switch.DefaultValue = bOn ? 1.0f : 0.0f;
		return Switch;
	}

	FName StatusControlId()
	{
		return FName("curb.status");
	}

	FRammsControlAxis MakeStatusControl(int32 Order)
	{
		FRammsControlAxis Status;
		Status.Id = StatusControlId();
		Status.Group = FName("Curb");
		Status.DisplayName = NSLOCTEXT("Ramms", "CurbStatus", "Curb status");
		Status.Kind = ERammsControlKind::Text;
		Status.Units = ERammsControlUnits::None;
		Status.bReadOnly = true;
		Status.Order = Order;
		Status.Range = FVector2D(0.0, static_cast<double>(ERammsControlTextSeverity::Fault));
		return Status;
	}

	void FAssistWatch::Reset()
	{
		bArmed = true;
		ScanLeft = 0.0;
	}

	bool FAssistWatch::Step(const FRammsCurbAssist& Assist, const FRammsCurbStick& Stick, double Forward, double Dt)
	{
		if (!Assist.bEnabled)
		{
			Reset();
			return false;
		}
		if (!bArmed)
		{
			bArmed = Forward < Stick.ReleaseFraction; // let go (or pulled back): it may look again
			ScanLeft = 0.0;
			return false;
		}
		if (Forward < Stick.EngageFraction)
		{
			ScanLeft = 0.0; // the next push forward looks at once
			return false;
		}
		ScanLeft -= Dt;
		if (ScanLeft > 1.0e-6) // the interval to within rounding: ticks that add up to it do not wait one more
		{
			return false;
		}
		ScanLeft = Assist.ScanIntervalSeconds;
		return true;
	}

	void FAssistWatch::CrossingEnded(bool bFinished)
	{
		bArmed = bFinished;
		ScanLeft = 0.0;
	}
} // namespace RammsCurb
