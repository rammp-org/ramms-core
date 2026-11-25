// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"

class URammsCurbNegotiationController;
class URammsRammpGen1CurbController;
class URammsRobotBaseComponent;
class USceneComponent;

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
		/** The chassis' heading, a horizontal unit vector in the world: for how
		 *  fast the robot is turning about the vertical (a turn on a slope
		 *  swings the slope from pitch into roll with no distance travelled).
		 *  Zero where the source does not know it. */
		FVector2D Heading = FVector2D::ZeroVector;
		/** When the reading was TAKEN (s, on a clock that runs with the
		 *  physics; only differences are used): the time every rate made from
		 *  two readings must be counted on, and every filter fed by them. It
		 *  is not the time of the tick that reads it where the pose is one the
		 *  simulator published a while before. Negative: the source cannot
		 *  say, and the reading is taken as of the tick that read it. */
		double TimeSeconds = -1.0;
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

	/**
	 * A chassis body's own frame in the world: Forward is the body's X axis,
	 * Side its Y axis (the side that is +Y when the robot heads along +X,
	 * Unreal's right) and Up its Z axis, each a unit vector whatever the
	 * body's scale. A point of the robot is then at (x along Forward, y along
	 * Side, z along Up) from Origin, and "the +Y side up" is Side rising.
	 */
	struct FBodyFrame
	{
		FVector Origin = FVector::ZeroVector;
		FVector Forward = FVector::ForwardVector;
		FVector Side = FVector::RightVector;
		FVector Up = FVector::UpVector;

		/** A world point in this frame (cm). */
		FVector ToLocal(const FVector& World) const
		{
			const FVector D = World - Origin;
			return FVector(FVector::DotProduct(D, Forward), FVector::DotProduct(D, Side), FVector::DotProduct(D, Up));
		}
		/** (pitch, roll), deg: the nose's rise, and the +Y side's (the Gen2
		 *  kind's convention, RammsCurb::AttitudeFromPivots). */
		FVector2D AttitudeDeg() const
		{
			return FVector2D(FMath::RadiansToDegrees(FMath::Asin(FMath::Clamp(Forward.Z, -1.0, 1.0))), FMath::RadiansToDegrees(FMath::Asin(FMath::Clamp(Side.Z, -1.0, 1.0))));
		}
	};

	/** Body's frame. False when its transform is not numbers, or it stands on
	 *  its nose (no heading to tell its sides by). */
	bool ReadBodyFrame(const USceneComponent& Body, FBodyFrame& Out);

	/** The scene component called Name on Owner; null when there is none. */
	USceneComponent* FindBody(const AActor* Owner, FName Name);

	/**
	 * The exact attitude of a robot whose chassis is one body with nothing to
	 * tilt between it and the rider (the Gen1 kind): that body's own axes, as
	 * its curb controller reads its pitch (the nose's rise), with the roll the
	 * same way (the +Y side's rise). Which body is the sibling curb
	 * controller's Config.ChassisBody. The reading is as old as the body's
	 * pose: the time the backend's poses were taken at.
	 */
	class FBodyAttitudeSource final : public IAttitudeSource
	{
	public:
		FBodyAttitudeSource(const URammsRobotBaseComponent& InBase, const URammsRammpGen1CurbController& InCurb);

		virtual bool Read(FAttitude& Out) override;

	private:
		TWeakObjectPtr<const URammsRobotBaseComponent>		Base;
		TWeakObjectPtr<const URammsRammpGen1CurbController> Curb;
		TWeakObjectPtr<USceneComponent>						Chassis;
	};
} // namespace RammsLevel
