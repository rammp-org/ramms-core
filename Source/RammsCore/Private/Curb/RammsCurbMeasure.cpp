// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbMeasure.h"

#include "RammsCurbNegotiationTypes.h"
#include "RammsRobotBaseComponent.h"

namespace RammsCurb
{
	namespace
	{
		FVector HorizontalOf(const FVector& V)
		{
			return FVector(V.X, V.Y, 0.0);
		}
	} // namespace

	bool AttitudeFromPivots(TConstArrayView<FPivotSample> Pivots, FPivotAttitude& Out)
	{
		// The sums and divisions below are in the order, and of the kinds, the
		// curb controller's Measure made them in before this was lifted out of
		// it: a crossing's numbers must not move by a bit.
		FVector	  FrontW = FVector::ZeroVector, RearW = FVector::ZeroVector;
		FVector	  PlusW = FVector::ZeroVector, MinusW = FVector::ZeroVector, AllW = FVector::ZeroVector;
		FVector2D FrontL = FVector2D::ZeroVector, RearL = FVector2D::ZeroVector, AllL = FVector2D::ZeroVector;
		double	  AllZ = 0.0;
		double	  AllY = 0.0;
		int32	  NFront = 0, NRear = 0, NPlus = 0, NMinus = 0, NAll = 0;
		for (const FPivotSample& Pivot : Pivots)
		{
			const FVector P = Pivot.World;
			if (Pivot.bFront)
			{
				FrontW += P;
				FrontL += Pivot.Local;
				++NFront;
			}
			else
			{
				RearW += P;
				RearL += Pivot.Local;
				++NRear;
			}
			if (Pivot.bPlusY)
			{
				PlusW += P;
				++NPlus;
			}
			else
			{
				MinusW += P;
				++NMinus;
			}
			AllZ += P.Z;
			AllW += P;
			AllY += Pivot.WheelY;
			AllL += Pivot.Local;
			++NAll;
		}
		if (NFront == 0 || NRear == 0)
		{
			return false;
		}
		FrontW /= NFront;
		RearW /= NRear;
		FrontL /= NFront;
		RearL /= NRear;
		AllL /= NAll;
		AllZ /= NAll;

		const FVector Along = FrontW - RearW;
		const double  Pitch =
			FMath::Atan2(Along.Z, HorizontalOf(Along).Size()) - FMath::Atan2(FrontL.Y - RearL.Y, FrontL.X - RearL.X);
		const FVector Forward = HorizontalOf(Along).GetSafeNormal();
		FVector		  Side = FVector::ZeroVector;
		double		  Roll = 0.0;
		if (NPlus > 0 && NMinus > 0)
		{
			const FVector Across = PlusW / NPlus - MinusW / NMinus;
			Roll = FMath::Atan2(Across.Z, HorizontalOf(Across).Size());
			Side = HorizontalOf(Across).GetSafeNormal();
		}
		Out.PitchRad = Pitch;
		Out.RollRad = Roll;
		Out.Forward = Forward;
		Out.Side = Side;
		Out.SumWorld = AllW;
		Out.Count = NAll;
		Out.MeanLocal = AllL;
		Out.MeanZ = AllZ;
		Out.SumWheelY = AllY;
		return true;
	}

	bool GatherPivots(const URammsRobotBaseComponent& Base, TConstArrayView<FRammsCurbLeg> Legs, TArray<FPivotSample, TInlineAllocator<4>>& Out)
	{
		Out.Reset();
		for (const FRammsCurbLeg& Leg : Legs)
		{
			if (Leg.Kind != ERammsCurbLegKind::FourBar || Leg.Axle == ERammsCurbAxle::Center)
			{
				continue;
			}
			FTransform Pivot;
			if (!Base.GetMotorTransform(Leg.FourBar.CrankMotor, Pivot))
			{
				return false;
			}
			FPivotSample& Sample = Out.AddDefaulted_GetRef();
			Sample.World = Pivot.GetLocation();
			Sample.Local = Leg.FourBar.CrankPivot;
			Sample.WheelY = Leg.Wheel.Position.Y;
			Sample.bFront = Leg.Axle == ERammsCurbAxle::Front;
			// As URammsCurbNegotiationController::IsPlusY.
			Sample.bPlusY = Leg.Wheel.Position.Y > 0.0f;
		}
		return true;
	}
} // namespace RammsCurb
