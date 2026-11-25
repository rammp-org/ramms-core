// Copyright Epic Games, Inc. All Rights Reserved.

#include "Level/RammsLevelAttitude.h"

#include "Components/SceneComponent.h"
#include "Curb/RammsCurbMeasure.h"
#include "GameFramework/Actor.h"
#include "RammsCurbNegotiationController.h"
#include "RammsRammpGen1CurbController.h"
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
		Out.Heading = FVector2D(Measured.Forward.X, Measured.Forward.Y).GetSafeNormal();
		// The pivots are where the backend's poses have them: as of those
		// poses' time, which need not be the clock's now.
		Out.TimeSeconds = BasePtr->GetTransformTime();
		return true;
	}

	bool ReadBodyFrame(const USceneComponent& Body, FBodyFrame& Out)
	{
		const FTransform& World = Body.GetComponentTransform();
		const FVector	  Forward = World.GetUnitAxis(EAxis::X);
		const FVector	  Up = World.GetUnitAxis(EAxis::Z);
		const FVector	  Side = World.GetUnitAxis(EAxis::Y);
		const FVector	  Origin = World.GetLocation();
		if (Forward.ContainsNaN() || Up.ContainsNaN() || Side.ContainsNaN() || Origin.ContainsNaN())
		{
			return false;
		}
		// (Its axes are its rotation's: a scale on the body, a mirrored one
		// too, does not turn them. The side is then always the one that is +Y
		// when the robot heads +X.)
		if (FVector2D(Forward.X, Forward.Y).GetSafeNormal().IsNearlyZero())
		{
			return false;
		}
		Out.Origin = Origin;
		Out.Forward = Forward;
		Out.Side = Side;
		Out.Up = Up;
		return true;
	}

	USceneComponent* FindBody(const AActor* Owner, FName Name)
	{
		if (Owner && !Name.IsNone())
		{
			TInlineComponentArray<USceneComponent*> Scene(Owner);
			for (USceneComponent* Component : Scene)
			{
				if (Component && Component->GetFName() == Name)
				{
					return Component;
				}
			}
		}
		return nullptr;
	}

	FBodyAttitudeSource::FBodyAttitudeSource(const URammsRobotBaseComponent& InBase, const URammsRammpGen1CurbController& InCurb)
		: Base(&InBase)
		, Curb(&InCurb)
	{
	}

	bool FBodyAttitudeSource::Read(FAttitude& Out)
	{
		const URammsRobotBaseComponent*		 BasePtr = Base.Get();
		const URammsRammpGen1CurbController* CurbPtr = Curb.Get();
		if (!BasePtr || !CurbPtr)
		{
			return false;
		}
		if (!Chassis.IsValid())
		{
			Chassis = FindBody(CurbPtr->GetOwner(), CurbPtr->Config.ChassisBody);
		}
		const USceneComponent* Body = Chassis.Get();
		FBodyFrame			   Frame;
		if (!Body || !ReadBodyFrame(*Body, Frame))
		{
			return false;
		}
		const FVector2D Deg = Frame.AttitudeDeg();
		if (!FMath::IsFinite(Deg.X) || !FMath::IsFinite(Deg.Y))
		{
			return false;
		}
		Out.Deg = Deg;
		Out.Position = Frame.Origin;
		Out.Heading = FVector2D(Frame.Forward.X, Frame.Forward.Y).GetSafeNormal();
		// The body is where the backend's poses have it: as of those poses'
		// time, which need not be the clock's now.
		Out.TimeSeconds = BasePtr->GetTransformTime();
		return true;
	}
} // namespace RammsLevel
