// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsControlContributor.h"
#include "RammsControlSurfaceTestContributor.generated.h"

/**
 * A control-surface contributor for the surface's tests (RammsControlSurfaceDeadmanTests.cpp), with no robot behind
 * it: only a record of what the surface asked of it.
 *
 *  - "test.rate": Continuous; its release springs it back to 0 here.
 *  - "test.trim": Continuous with DefaultValue TrimDefault; it refuses a release, so the surface springs it back
 *    through ApplyControl(DefaultValue) instead.
 *  - "test.position": Position; it holds the target until released.
 *
 * Not a spawnable component: nothing outside the tests should carry one.
 */
UCLASS(NotBlueprintable, Transient, HideDropdown)
class URammsControlSurfaceTestContributor : public UActorComponent, public IRammsControlContributor
{
	GENERATED_BODY()

public:
	static FName		   RateId() { return FName(TEXT("test.rate")); }
	static FName		   TrimId() { return FName(TEXT("test.trim")); }
	static FName		   PositionId() { return FName(TEXT("test.position")); }
	static constexpr float TrimDefault = 0.25f;

	/** The value each control was last set to (absent: never set, or a released position). */
	TMap<FName, float> Values;
	/** How many times ReleaseControl was asked of each control. */
	TMap<FName, int32> Releases;

	int32 ReleasesOf(FName Id) const
	{
		const int32* Count = Releases.Find(Id);
		return Count ? *Count : 0;
	}

	virtual void DescribeControls(FRammsControlSurface& OutSurface) const override
	{
		FRammsControlAxis RateAxis;
		RateAxis.Id = RateId();
		RateAxis.Group = FName(TEXT("Test"));
		RateAxis.Kind = ERammsControlKind::Continuous;
		RateAxis.Range = FVector2D(-1.0, 1.0);
		RateAxis.Order = 0;
		OutSurface.Add(RateAxis);

		FRammsControlAxis TrimAxis = RateAxis;
		TrimAxis.Id = TrimId();
		TrimAxis.DefaultValue = TrimDefault;
		TrimAxis.Order = 1;
		OutSurface.Add(TrimAxis);

		FRammsControlAxis PositionAxis = RateAxis;
		PositionAxis.Id = PositionId();
		PositionAxis.Kind = ERammsControlKind::Position;
		PositionAxis.Units = ERammsControlUnits::Centimeters;
		PositionAxis.Range = FVector2D(-100.0, 100.0);
		PositionAxis.Order = 2;
		OutSurface.Add(PositionAxis);
	}

	virtual bool ApplyControl(FName Id, float Value) override
	{
		if (Id != RateId() && Id != TrimId() && Id != PositionId())
		{
			return false;
		}
		Values.Add(Id, Value);
		return true;
	}

	virtual bool ReleaseControl(FName Id) override
	{
		++Releases.FindOrAdd(Id, 0);
		if (Id == RateId())
		{
			Values.Add(Id, 0.0f);
			return true;
		}
		if (Id == PositionId())
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
		// A position contributor that holds targets reports them; the rate axes keep the default.
		if (Id == PositionId())
		{
			return ReadControl(Id, OutTarget);
		}
		return false;
	}
};
