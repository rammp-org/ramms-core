// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Containers/BitArray.h"
#include "Ramms4BarLegSpec.h"
#include "Ramms5BarLinkageSpec.h"
#include "RammsCurbNegotiationTypes.h"
#include "Templates/Function.h"

class URamms5BarLinkageController;

/**
 * The curb controller's robot model, as plain C++ fed made-up robots so each
 * piece can be tested on its own. Not reflected, not exported.
 *
 * What the gait needs to know of the robot beyond each leg's own kinematics:
 * where each corner 4-bar carries its wheel fore/aft as it raises it (a table
 * over its crank), where each centre 5-bar can put its wheel (a grid over its
 * knee-limited reach, from the leg's own reachability test), where the centre
 * of mass is with the legs where they are (a fixed centre of mass at the legs'
 * zero pose, moved by the mass each leg carries with its wheel), the belly
 * plate, and how far inside the wheels on the ground the robot stands.
 *
 * Stability is two-dimensional. The wheels on the ground
 * span a support polygon, the convex hull of their contact points; the centre
 * of mass projects straight down (gravity) into the plane; the margin is how
 * far inside the hull it is (negative outside). A robot speeding up or braking
 * at a, its centre of mass h over the ground, shifts its zero-moment point
 * h a / g against the acceleration; the dynamic margin is the least over
 * braking at Deceleration and speeding up at Acceleration.
 */
namespace RammsCurb
{
	/** Standard gravity (cm/s^2). */
	constexpr double GravityCmS2 = 980.665;

	/** Samples (intervals) a 4-bar's table takes over its crank range. */
	constexpr int32 FourBarTableSamples = 256;

	/** A 5-bar's reach grid: fore/aft and height steps (cm). */
	constexpr double FiveBarGridXStepCm = 1.0;
	constexpr double FiveBarGridZStepCm = 0.25;

	/** Within this angle (deg) of parallel to the ground, a wheel's axle puts
	 *  its contact part way from the middle of its tread to the rim nearer the
	 *  ground (WheelContact). */
	constexpr double RimContactRampDeg = 1.0;

	/**
	 * A corner 4-bar's wheel centre (chassis x-z, cm) over its crank range,
	 * tabulated against the wheel's height: the fore/aft position a height
	 * puts the wheel at, and the crank angle that holds it there, by linear
	 * interpolation between FourBarTableSamples + 1 cranks evenly over
	 * CrankRange.
	 */
	class FFourBarTable
	{
	public:
		/** False, leaving the table empty, when the linkage does not close at
		 *  one of the cranks sampled or the height is not monotonic in the
		 *  crank across the range. */
		bool Build(const FRamms4BarLegSpec& Spec, int32 Samples = FourBarTableSamples);

		bool IsValid() const { return Z.Num() >= 2; }

		/** The least and most wheel heights (cm). */
		FVector2D HeightRange() const;

		/** The wheel's fore/aft position (cm) at Height, held at an end of the
		 *  range beyond it. */
		double XAt(double Height) const;

		/** The crank angle (rad) at Height, held at an end beyond it. */
		double CrankAt(double Height) const;

		/** The table as it is, to an archive and back (development: a plan's
		 *  input replayed off the game). */
		friend FArchive& operator<<(FArchive& Ar, FFourBarTable& Table)
		{
			return Ar << Table.Z << Table.X << Table.Crank;
		}

	private:
		double Interpolate(const TArray<double>& Values, double Height) const;

		/** By height, rising. */
		TArray<double> Z;
		TArray<double> X;
		TArray<double> Crank;
	};

	/**
	 * A centre 5-bar's reach: a grid of cells XStep apart fore/aft and ZStep
	 * apart in height (cm, chassis frame) over the box it is built on, each
	 * reachable or not by the leg's own test -- for a leg on the robot, its
	 * controller's SolveTarget, which bounds the knees (FRamms5BarLinkageSpec's
	 * DistalRangeA/B) and the hips' motor ranges.
	 */
	class FFiveBarGrid
	{
	public:
		/** Test every cell centre of the box XRange x ZRange (cm, least and
		 *  most) with Reachable(x, z). */
		void Build(TFunctionRef<bool(double X, double Z)> Reachable, const FVector2D& XRange, const FVector2D& ZRange,
			double XStep = FiveBarGridXStepCm, double ZStep = FiveBarGridZStepCm);

		bool IsValid() const { return NumX > 0 && NumZ > 0; }

		/** The cell nearest (X, Z) is reachable; outside the grid, no. */
		bool Contains(double X, double Z) const;

		/** The least and most reachable heights (cm) in the column nearest X;
		 *  least > most where none is. */
		FVector2D HeightRangeAt(double X) const;

		/** The least and most reachable fore/aft positions (cm) in the row
		 *  nearest Z; least > most where none is. */
		FVector2D XRangeAt(double Z) const;

		/** How many cells are reachable. */
		int32 CountReachable() const;

		/** The grid as it is, to an archive and back (development). */
		friend FArchive& operator<<(FArchive& Ar, FFiveBarGrid& Grid)
		{
			return Ar << Grid.Cells << Grid.Origin << Grid.XStep << Grid.ZStep << Grid.NumX << Grid.NumZ;
		}

	private:
		bool Cell(int32 I, int32 J) const { return Cells[J * NumX + I]; }

		TBitArray<> Cells;
		FVector2D	Origin = FVector2D::ZeroVector;
		double		XStep = FiveBarGridXStepCm;
		double		ZStep = FiveBarGridZStepCm;
		int32		NumX = 0;
		int32		NumZ = 0;
	};

	/** One leg of the model. */
	struct FModelLeg
	{
		ERammsCurbLegKind Kind = ERammsCurbLegKind::FourBar;
		ERammsCurbAxle	  Axle = ERammsCurbAxle::Front;
		/** Across the chassis (cm, +Y), the wheel's Position.Y. */
		double Y = 0.0;
		double RadiusCm = 10.0;
		/** How wide its tread touches the ground across its axle (cm):
		 *  RammsCurb::ContactWidth of the leg's tyre and contact widths. */
		double ContactWidthCm = 0.0;
		/** The mass (kg) that moves with the wheel (FRammsCurbLeg::MovingMassKg). */
		double MovingMassKg = 0.0;
		/** The wheel centre (chassis x-z, cm) at the leg's zero pose: a 4-bar's
		 *  crank at 0, a 5-bar's hips at 0. The centre of mass is reckoned from
		 *  there. */
		FVector2D ZeroPose = FVector2D::ZeroVector;
		/** The direction the wheel rolls in, in the chassis frame (unit x-y). */
		FVector2D RollDirection = FVector2D(1.0, 0.0);
		/** A 4-bar's table. */
		FFourBarTable FourBar;
	};

	/** The seat mechanism of the model (FRammsCurbSeat), as the stability
	 *  checks see it: lumps that move with the platform's pivot. */
	struct FModelSeat
	{
		bool bHas = false;
		/** The pivot at the zero pose (chassis x-z, cm): CenterOfMass is
		 *  reckoned with the seat there, level. */
		FVector2D ZeroPivot = FVector2D::ZeroVector;
		/** The mechanism's lump at the pivot and the rider's PayloadOffset
		 *  from it in the platform's frame (kg, cm). */
		double	  MechanismMassKg = 0.0;
		double	  PayloadMassKg = 0.0;
		FVector2D PayloadOffset = FVector2D::ZeroVector;
		/** The height the crossing holds the pivot at and the fore/aft it
		 *  can be held at there (cm; min > max: none). */
		double	  HeightCm = 0.0;
		FVector2D XRange = FVector2D(1.0, 0.0);
		/** The most the platform pitches against the chassis (rad), and
		 *  whether it pitches at all (a tilt motor). */
		double MaxPitchRad = 0.0;
		bool   bTilts = false;
		double ShiftSpeedCmS = 0.0;
		double MovingMassKg() const { return MechanismMassKg + PayloadMassKg; }
		bool   Shifts() const { return bHas && XRange.X < XRange.Y; }
	};

	/** The robot, as the stability checks and the planner see it. */
	struct FRobotModel
	{
		TArray<FModelLeg> Legs;
		FModelSeat		  Seat;
		/** The centre of mass (chassis frame, cm) with every leg at its zero
		 *  pose. */
		FVector CenterOfMass = FVector::ZeroVector;
		/** The whole robot's mass (kg); 0 leaves the centre of mass fixed at
		 *  CenterOfMass whatever the legs do. */
		double MassKg = 0.0;
		/** Half the belly plate (cm, X fore/aft, Y across) about the chassis
		 *  origin, and its underside's height over the origin (cm). */
		FVector2D BellyHalfExtent = FVector2D::ZeroVector;
		double	  BellyUndersideCm = 0.0;

		/** The centre of mass (chassis frame, cm) with each leg's wheel centre
		 *  at WheelXZ[i] (chassis x-z, cm): CenterOfMass plus, for each leg,
		 *  MovingMassKg / MassKg of how far its wheel is from its zero pose. */
		FVector CenterOfMassAt(TConstArrayView<FVector2D> WheelXZ) const;

		/** The same with the seat's pivot at SeatPivot (chassis x-z, cm) and
		 *  the platform pitched SeatPitchRad against the chassis (nose up):
		 *  the mechanism's lump moves with the pivot, the rider's with it and
		 *  round it. The seat at its zero pose, level, is the plain one. */
		FVector CenterOfMassAt(TConstArrayView<FVector2D> WheelXZ, const FVector2D& SeatPivot, double SeatPitchRad) const;

		/** Points over the belly plate (chassis x-y, cm): Along x Across of
		 *  them, evenly from one edge to the other. */
		void BellyPoints(TArray<FVector2D>& Out, int32 Along = 5, int32 Across = 3) const;
	};

	/** The centre of mass a tuning sets (chassis frame, cm): CenterOfMass,
	 *  its X taken from the deprecated CenterOfMassX while CenterOfMass.X is
	 *  0. */
	FVector TunedCenterOfMass(const FRammsCurbTuning& Tuning);

	/** Half the belly plate: Tuning.ChassisHalfExtent, either axis 0 taken from
	 *  the corner legs -- their crank pivots fore/aft, their wheels across. */
	FVector2D BellyHalfExtentOf(TConstArrayView<FRammsCurbLeg> Legs, const FRammsCurbTuning& Tuning);

	/** A 5-bar controller's wheel centre at its zero pose (chassis x-z, cm):
	 *  both hips at 0, on the spec it resolves (its table's row, else its
	 *  inline one). False when the linkage does not close there. */
	bool FiveBarZeroPose(const URamms5BarLinkageController& FiveBar, FVector2D& Out);

	/**
	 * The model of the robot Legs and Tuning describe. FiveBarZeroPoses holds,
	 * for each 5-bar leg (in Legs order; ignored for a 4-bar), its zero pose
	 * (FiveBarZeroPose). False, with why, when a mass is negative, the legs'
	 * moving masses add up to the robot's or more, or a leg that carries mass
	 * has no zero pose (a 4-bar that does not close at crank 0), or a 4-bar's
	 * table cannot be built.
	 */
	bool BuildRobotModel(TConstArrayView<FRammsCurbLeg> Legs, const FRammsCurbTuning& Tuning,
		TConstArrayView<FVector2D> FiveBarZeroPoses, FRobotModel& Out, FString& OutReason);

	/**
	 * The signed distance (cm) from P to the boundary of the convex hull of
	 * Points: positive inside, negative outside (the least, over the hull's
	 * edges, of P's distance from the edge's line, positive on the hull's
	 * side). Points closer than 1e-6 cm count once. With fewer than three
	 * points off one line the hull is a segment or a point, and the margin is
	 * minus P's distance from it (0 at best); with none, the lowest double.
	 */
	double HullMargin(TConstArrayView<FVector2D> Points, const FVector2D& P);

	/** How far (cm) the zero-moment point moves from under the centre of mass
	 *  of a robot accelerating at AccelCmS2 with its centre of mass HeightCm
	 *  over the ground: h a / g, against the acceleration. */
	double ZmpShiftCm(double HeightCm, double AccelCmS2);

	/**
	 * The stability margin of a robot that may brake or speed up along Heading
	 * (unit, x-y): the least HullMargin of its zero-moment point,
	 * ComXY - (HeightCm / g) a Heading, for a = +Acceleration (speeding up,
	 * the point moves back) and a = -Deceleration (braking, it moves forward).
	 */
	double DynamicMargin(TConstArrayView<FVector2D> Points, const FVector2D& ComXY, double HeightCm, const FVector2D& Heading,
		double Acceleration, double Deceleration);

	/**
	 * Where a wheel touches a plane: the point of a cylinder of RadiusCm,
	 * centred at Centre, its tread ContactWidthCm wide along its axle (Axle,
	 * 3-D, either sense), lowest against the plane of Gradient (dz/dx, dz/dy;
	 * the same frame) -- as MuJoCo's collision cylinders touch.
	 *
	 * Along the wheel's rolling direction it leans RadiusCm s / sqrt(1 + s^2)
	 * towards the higher ground, s the slope along it (a level axle). Across,
	 * an axle not parallel to the plane touches on the rim nearer it,
	 * ContactWidthCm / 2 from the middle: a level axle on a slope across it,
	 * the uphill rim -- the contact the wheel-centre height rho
	 * (RammsCurb::WheelCentreHeight, (w_c / 2) |g . a|) assumes; a chassis
	 * rolled with the slope, the middle, R sin(slope) uphill of the centre's
	 * foot. A rigid tread jumps from rim to rim as its axle passes parallel;
	 * within RimContactRampDeg of parallel the contact is taken part way, in
	 * proportion, as a tyre's give spreads it, so a fitted gradient's or a
	 * measured roll's noise does not throw it across the tread.
	 */
	FVector WheelContact(const FVector& Centre, const FVector& Axle, const FVector2D& Gradient, double RadiusCm, double ContactWidthCm);

	/** One leg in a pose. */
	struct FPoseLeg
	{
		/** Its wheel centre (chassis x-z, cm). */
		FVector2D XZ = FVector2D::ZeroVector;
		/** On the ground, carrying the robot. */
		bool bSupport = false;
		/** The gradient of the ground under it (dz/dx, dz/dy, chassis
		 *  frame). */
		FVector2D Gradient = FVector2D::ZeroVector;
	};

	/** A pose's stability. */
	struct FPoseMargin
	{
		/** The centre of mass (cm): chassis frame for PoseMargin, world for
		 *  MeasuredMargin. */
		FVector CenterOfMass = FVector::ZeroVector;
		/** The centre of mass over the mean of the support contacts (cm). */
		double HeightCm = 0.0;
		/** The margin standing still, and braking or speeding up (cm). */
		double StaticCm = 0.0;
		double DynamicCm = 0.0;
		int32  Support = 0;
	};

	/**
	 * The stability of a pose with the chassis level, in its own frame: the
	 * centre of mass from the legs' wheel centres (FRobotModel::CenterOfMassAt),
	 * the contact of each support wheel (WheelContact: its centre at the leg's
	 * Y, its axle level and square to its roll direction), the centre of
	 * mass's height over the mean of those contacts, and the margins
	 * (HullMargin; DynamicMargin along the chassis' +X). Pose holds one entry
	 * per model leg.
	 */
	FPoseMargin PoseMargin(const FRobotModel& Model, TConstArrayView<FPoseLeg> Pose, double Acceleration, double Deceleration);

	/** The chassis' pose in the world: its origin and axes (unit; Z up out of
	 *  the belly). */
	struct FChassisFrame
	{
		FVector Origin = FVector::ZeroVector;
		FVector X = FVector::ForwardVector;
		FVector Y = FVector::RightVector;
		FVector Z = FVector::UpVector;

		FVector ToWorld(const FVector& Local) const { return Origin + X * Local.X + Y * Local.Y + Z * Local.Z; }
	};

	/**
	 * The chassis frame from its measured attitude: X along Forward (unit,
	 * horizontal) raised PitchRad (nose up), Y across raised RollRad (+Y up)
	 * and squared to X, Z = X x Y; the origin where the corner crank pivots'
	 * mean, PivotMeanWorld, puts it, their mean in the chassis frame being
	 * PivotMeanLocal.
	 */
	FChassisFrame ChassisFrameFrom(const FVector& Forward, double PitchRad, double RollRad, const FVector& PivotMeanWorld,
		const FVector& PivotMeanLocal);

	/** One wheel as measured, for MeasuredMargin: one per model leg, in its
	 *  order (its radius, tread and roll direction are the leg's). */
	struct FMeasuredWheel
	{
		/** Its centre (world, cm). */
		FVector Centre = FVector::ZeroVector;
		bool	bSupport = false;
		/** The gradient of the surface under it (world). */
		FVector2D Gradient = FVector2D::ZeroVector;
	};

	/**
	 * The stability of the robot as measured, in the world: the centre of mass
	 * from each leg's measured wheel centre (LegXZ, chassis x-z), placed in the
	 * world through Frame, projected straight down; the contacts of the
	 * support wheels (WheelContact: their measured centres, each axle square to
	 * its leg's roll direction in the chassis as Frame tilts it) on the surface
	 * each stands on; the centre of mass's height over their mean; the margins
	 * (DynamicMargin along Heading, unit horizontal). Wheels holds one entry
	 * per model leg. Support 0 when no wheel is on the ground.
	 */
	FPoseMargin MeasuredMargin(const FRobotModel& Model, const FChassisFrame& Frame, TConstArrayView<FVector2D> LegXZ,
		TConstArrayView<FMeasuredWheel> Wheels, const FVector2D& Heading, double Acceleration, double Deceleration);

	/** The seat as measured: its pivot (chassis x-z, cm) and the platform's
	 *  pitch against the chassis (rad). */
	struct FMeasuredSeat
	{
		bool	  bValid = false;
		FVector2D Pivot = FVector2D::ZeroVector;
		double	  PitchRad = 0.0;
	};

	/** MeasuredMargin with the seat where it is measured (a model without a
	 *  seat, or an invalid measure, is the plain one). */
	FPoseMargin MeasuredMargin(const FRobotModel& Model, const FChassisFrame& Frame, TConstArrayView<FVector2D> LegXZ,
		TConstArrayView<FMeasuredWheel> Wheels, const FMeasuredSeat& Seat, const FVector2D& Heading, double Acceleration, double Deceleration);
} // namespace RammsCurb
