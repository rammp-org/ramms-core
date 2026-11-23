// Copyright Epic Games, Inc. All Rights Reserved.

#include "Level/RammsLevelAttitude.h"

#include "Curb/RammsCurbMeasure.h"
#include "RammsCurbNegotiationController.h"
#include "RammsRobotBaseComponent.h"

namespace RammsLevel
{
	FPivotAttitudeSource::FPivotAttitudeSource(const URammsRobotBaseComponent& InBase, const URammsCurbNegotiationController& InCurb)
		: Base(&InBase)
		, Curb(&InCurb)
	{
	}

	bool FPivotAttitudeSource::Read(FAttitude& Out)
	{
		const URammsRobotBaseComponent*		   BasePtr = Base.Get();
		const URammsCurbNegotiationController* CurbPtr = Curb.Get();
		if (!BasePtr || !CurbPtr)
		{
			return false;
		}
		TArray<RammsCurb::FPivotSample, TInlineAllocator<4>> Pivots;
		RammsCurb::FPivotAttitude							 Measured;
		if (!RammsCurb::GatherPivots(*BasePtr, CurbPtr->Legs, Pivots) || !RammsCurb::AttitudeFromPivots(Pivots, Measured))
		{
			return false;
		}
		const FVector2D Deg(FMath::RadiansToDegrees(Measured.PitchRad), FMath::RadiansToDegrees(Measured.RollRad));
		const FVector	Position = Measured.SumWorld / static_cast<double>(FMath::Max(1, Measured.Count));
		// A simulation not yet running, or one that has blown up, reads as
		// numbers that are not numbers: no reading at all is the honest answer.
		if (!FMath::IsFinite(Deg.X) || !FMath::IsFinite(Deg.Y) || Position.ContainsNaN())
		{
			return false;
		}
		Out.Deg = Deg;
		Out.Position = Position;
		return true;
	}
} // namespace RammsLevel
