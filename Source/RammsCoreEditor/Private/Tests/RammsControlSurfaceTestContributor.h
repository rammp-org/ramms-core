// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsControlContributor.h"
#include "RammsRobotControlSurfaceComponent.h"
#include "RammsControlSurfaceTestContributor.generated.h"

/**
 * A control-surface contributor for the surface's tests (RammsControlSurfaceDeadmanTests.cpp), with no robot behind
 * it: only a record of what the surface asked of it.
 *
 *  - "test.rate": Continuous; its release springs it back to 0 here.
 *  - "test.trim": Continuous with DefaultValue TrimDefault; it refuses a release, so the surface springs it back
 *    through ApplyControl(DefaultValue) instead. Offered only while bOfferTrim.
 *  - "test.position": Position; it holds the target until released.
 *  - "test.speed": Velocity; it holds the rate until changed.
 *  - "test.choice": Enum of three.
 *  - "test.go": Action.
 *
 * In the editor module, not RammsCore: a UCLASS cannot be compiled out of a module's builds (UHT does not honour
 * WITH_DEV_AUTOMATION_TESTS), and here it stays out of game and Shipping builds. Not a spawnable component either:
 * nothing outside the tests should carry one.
 */
UCLASS(NotBlueprintable, Transient, HideDropdown)
class URammsControlSurfaceTestContributor : public UActorComponent, public IRammsControlContributor
{
	GENERATED_BODY()

public:
	static FName		   RateId() { return FName(TEXT("test.rate")); }
	static FName		   TrimId() { return FName(TEXT("test.trim")); }
	static FName		   PositionId() { return FName(TEXT("test.position")); }
	static FName		   SpeedId() { return FName(TEXT("test.speed")); }
	static FName		   ChoiceId() { return FName(TEXT("test.choice")); }
	static FName		   GoId() { return FName(TEXT("test.go")); }
	static constexpr float TrimDefault = 0.25f;

	/** The value each control was last set to (absent: never set, or a released servo). */
	TMap<FName, float> Values;
	/** How many times ApplyControl / ReleaseControl / TriggerControl was asked of each control. */
	TMap<FName, int32> Applies;
	TMap<FName, int32> Releases;
	TMap<FName, int32> Triggers;

	/** Offer test.trim (a rebuild after clearing it drops the control). */
	bool bOfferTrim = true;
	/** Releasing test.rate rebuilds this surface first -- as the drive-mode selector's contributors rebuild theirs --
	 *  and, with bDropTrimOnRateRelease, stops offering test.trim in that rebuild. */
	TWeakObjectPtr<URammsRobotControlSurfaceComponent> RebuildOnRateRelease;
	bool											   bDropTrimOnRateRelease = false;

	static int32 CountOf(const TMap<FName, int32>& Counts, FName Id)
	{
		const int32* Count = Counts.Find(Id);
		return Count ? *Count : 0;
	}
	int32 AppliesOf(FName Id) const { return CountOf(Applies, Id); }
	int32 ReleasesOf(FName Id) const { return CountOf(Releases, Id); }
	int32 TriggersOf(FName Id) const { return CountOf(Triggers, Id); }

	virtual void DescribeControls(FRammsControlSurface& OutSurface) const override
	{
		FRammsControlAxis RateAxis;
		RateAxis.Id = RateId();
		RateAxis.Group = FName(TEXT("Test"));
		RateAxis.Kind = ERammsControlKind::Continuous;
		RateAxis.Range = FVector2D(-1.0, 1.0);
		RateAxis.Order = 0;
		OutSurface.Add(RateAxis);

		if (bOfferTrim)
		{
			FRammsControlAxis TrimAxis = RateAxis;
			TrimAxis.Id = TrimId();
			TrimAxis.DefaultValue = TrimDefault;
			TrimAxis.Order = 1;
			OutSurface.Add(TrimAxis);
		}

		FRammsControlAxis PositionAxis = RateAxis;
		PositionAxis.Id = PositionId();
		PositionAxis.Kind = ERammsControlKind::Position;
		PositionAxis.Units = ERammsControlUnits::Centimeters;
		PositionAxis.Range = FVector2D(-100.0, 100.0);
		PositionAxis.Order = 2;
		OutSurface.Add(PositionAxis);

		FRammsControlAxis SpeedAxis = RateAxis;
		SpeedAxis.Id = SpeedId();
		SpeedAxis.Kind = ERammsControlKind::Velocity;
		SpeedAxis.Units = ERammsControlUnits::RadiansPerSecond;
		SpeedAxis.Range = FVector2D(-5.0, 5.0);
		SpeedAxis.Order = 3;
		OutSurface.Add(SpeedAxis);

		FRammsControlAxis ChoiceAxis = RateAxis;
		ChoiceAxis.Id = ChoiceId();
		ChoiceAxis.Kind = ERammsControlKind::Enum;
		ChoiceAxis.Units = ERammsControlUnits::None;
		ChoiceAxis.EnumLabels = { FText::FromString(TEXT("A")), FText::FromString(TEXT("B")), FText::FromString(TEXT("C")) };
		ChoiceAxis.Range = FVector2D(0.0, 2.0);
		ChoiceAxis.Order = 4;
		OutSurface.Add(ChoiceAxis);

		FRammsControlAxis GoAxis = RateAxis;
		GoAxis.Id = GoId();
		GoAxis.Kind = ERammsControlKind::Action;
		GoAxis.Units = ERammsControlUnits::None;
		GoAxis.bReadback = false;
		GoAxis.Order = 5;
		OutSurface.Add(GoAxis);
	}

	virtual bool ApplyControl(FName Id, float Value) override
	{
		const bool bOffered = Id == RateId() || (Id == TrimId() && bOfferTrim) || Id == PositionId() || Id == SpeedId() || Id == ChoiceId();
		if (!bOffered)
		{
			return false;
		}
		++Applies.FindOrAdd(Id, 0);
		Values.Add(Id, Value);
		return true;
	}

	virtual bool TriggerControl(FName Id) override
	{
		if (Id != GoId())
		{
			return false;
		}
		++Triggers.FindOrAdd(Id, 0);
		return true;
	}

	virtual bool ReleaseControl(FName Id) override
	{
		++Releases.FindOrAdd(Id, 0);
		if (Id == RateId())
		{
			if (URammsRobotControlSurfaceComponent* Surface = RebuildOnRateRelease.Get())
			{
				bOfferTrim = bOfferTrim && !bDropTrimOnRateRelease;
				Surface->RebuildControlSurface();
			}
			Values.Add(Id, 0.0f);
			return true;
		}
		if (Id == PositionId() || Id == SpeedId())
		{
			Values.Remove(Id);
			return true;
		}
		return false; // test.trim cannot let go: the surface applies its DefaultValue
	}

	virtual bool ReadControl(FName Id, float& OutValue) const override
	{
		if (const float* Found = Values.Find(Id))
		{
			OutValue = *Found;
			return true;
		}
		return false;
	}

	virtual bool ReadTarget(FName Id, float& OutTarget) const override
	{
		// Servo controls report the target they hold; the rate axes keep the default.
		if (Id == PositionId() || Id == SpeedId())
		{
			return ReadControl(Id, OutTarget);
		}
		return false;
	}
};
