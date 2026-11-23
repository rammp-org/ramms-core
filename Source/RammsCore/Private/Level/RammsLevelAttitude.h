// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"

class URammsCurbNegotiationController;
class URammsRobotBaseComponent;

/**
 * Where the level assist reads the chassis' attitude: one small interface, so
 * that the law does not care whether the reading is the simulation's own
 * (exact) or, later, an IMU's. Plain C++, not reflected, not exported. Signs
 * as RammsLevelTypes.h: degrees; X pitch, positive nose up; Y roll, positive
 * with the robot's +Y side up.
 */
namespace RammsLevel
{
	/** One reading of the chassis. */
	struct FAttitude
	{
		/** (pitch, roll), deg, unfiltered. */
		FVector2D Deg = FVector2D::ZeroVector;
		/** A point fixed on the chassis, in the world (cm): for how fast the
		 *  robot is moving and whether it was picked up and put elsewhere. */
		FVector Position = FVector::ZeroVector;
	};

	class IAttitudeSource
	{
	public:
		virtual ~IAttitudeSource() = default;

		/** Read the chassis now. False, Out untouched, when it cannot be read
		 *  (the law then holds its offsets as they are). */
		virtual bool Read(FAttitude& Out) = 0;
	};

	/**
	 * The exact attitude of a robot whose legs hang from corner crank pivots
	 * (the Gen2 kind): from those pivots' world positions, by the curb
	 * controller's own arithmetic (RammsCurb::AttitudeFromPivots), so that the
	 * level assist and a crossing agree on what level is. Which pivots, and
	 * where each sits on the chassis, are the sibling curb controller's Legs.
	 */
	class FPivotAttitudeSource final : public IAttitudeSource
	{
	public:
		FPivotAttitudeSource(const URammsRobotBaseComponent& InBase, const URammsCurbNegotiationController& InCurb);

		virtual bool Read(FAttitude& Out) override;

	private:
		TWeakObjectPtr<const URammsRobotBaseComponent>		  Base;
		TWeakObjectPtr<const URammsCurbNegotiationController> Curb;
	};
} // namespace RammsLevel
