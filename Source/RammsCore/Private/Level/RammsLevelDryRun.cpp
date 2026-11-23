// Copyright Epic Games, Inc. All Rights Reserved.

#include "Level/RammsLevelDryRun.h"

#include "Math/RandomStream.h"
#include "RammsMotionProfile.h"

namespace RammsLevel::DryRun
{
	// Named, not anonymous: in a unity build the other Level files share this
	// translation unit, and their helpers may well be called the same.
	namespace DryRunImpl
	{
		/** A bottom this far (cm) under the ground still counts as on it: the
		 *  rest solve's own rounding. */
		constexpr double GapSlackCm = 1.0e-7;
		/** A load share this far under zero still counts as bearing: the
		 *  centre of mass on the edge of the touching polygon (about 0.01 cm
		 *  outside it, at these robots' sizes) is held. */
		constexpr double LoadSlack = 2.0e-4;
		/** Three bottoms in line span no plane: their basis is skipped. */
		constexpr double MinBasis = 1.0e-6;
		/** The rest solve has settled when a pass turns the chassis by less
		 *  than this (rad); it never turns it further than MaxTurnRad in one
		 *  pass, and gives up after MaxPasses. */
		constexpr double SettledRad = 1.0e-8;
		constexpr double MaxTurnRad = 0.2;
		constexpr int32	 MaxPasses = 40;
		/** The ground's slope under a bottom is taken over this far (cm). */
		constexpr double GradientStepCm = 0.01;
		/** Times that should be equal are, to within this (s). */
		constexpr double TimeHair = 1.0e-9;

		/** The chassis' rotation, world = Rz(Yaw) Ry(-Pitch) Rx(Roll). */
		struct FTurn
		{
			double SinP = 0.0;
			double CosP = 1.0;
			double SinR = 0.0;
			double CosR = 1.0;
			double SinY = 0.0;
			double CosY = 1.0;
		};

		FTurn MakeTurn(double PitchRad, double RollRad, double YawRad)
		{
			FTurn Turn;
			Turn.SinP = FMath::Sin(PitchRad);
			Turn.CosP = FMath::Cos(PitchRad);
			Turn.SinR = FMath::Sin(RollRad);
			Turn.CosR = FMath::Cos(RollRad);
			Turn.SinY = FMath::Sin(YawRad);
			Turn.CosY = FMath::Cos(YawRad);
			return Turn;
		}

		/** A chassis point turned into the world's axes, with the values on
		 *  the way that its derivatives in pitch and roll are made of. */
		struct FTurned
		{
			/** After the roll: y and z. After the pitch too: x. */
			double Y1 = 0.0;
			double Z1 = 0.0;
			double X2 = 0.0;
			/** In the world's axes, about the chassis origin. */
			FVector World = FVector::ZeroVector;
		};

		FTurned TurnPoint(const FTurn& Turn, const FVector& Point)
		{
			FTurned Out;
			Out.Y1 = Point.Y * Turn.CosR - Point.Z * Turn.SinR;
			Out.Z1 = Point.Y * Turn.SinR + Point.Z * Turn.CosR;
			Out.X2 = Point.X * Turn.CosP - Out.Z1 * Turn.SinP;
			const double Z2 = Point.X * Turn.SinP + Out.Z1 * Turn.CosP;
			Out.World = FVector(Out.X2 * Turn.CosY - Out.Y1 * Turn.SinY, Out.X2 * Turn.SinY + Out.Y1 * Turn.CosY, Z2);
			return Out;
		}

		using FRow = TArray<double, TInlineAllocator<8>>;

		/**
		 * The lowest rest of the linearised problem: minimise
		 *   Z + CPitch dPitch + CRoll dRoll
		 * subject to Z + H[i] + DPitch[i] dPitch + DRoll[i] dRoll >= 0 for
		 * every bottom. Three unknowns, so the answer sits where three of the
		 * limits meet; it is the lowest when the centre of mass's pull can be
		 * shared among those three with no share negative (the three bottoms
		 * hold it). The triples are few, so they are simply tried in order.
		 * False when none will do: the robot tips, or balances on two.
		 */
		bool LowestRest(const FRow& H, const FRow& DPitch, const FRow& DRoll, double CPitch, double CRoll, double& OutZ, double& OutDPitch,
			double& OutDRoll)
		{
			const int32 Num = H.Num();
			for (int32 I = 0; I < Num; ++I)
			{
				for (int32 J = I + 1; J < Num; ++J)
				{
					for (int32 K = J + 1; K < Num; ++K)
					{
						const double Ux = DPitch[J] - DPitch[I];
						const double Uy = DRoll[J] - DRoll[I];
						const double Vx = DPitch[K] - DPitch[I];
						const double Vy = DRoll[K] - DRoll[I];
						const double Det = Ux * Vy - Uy * Vx;
						if (FMath::Abs(Det) < MinBasis)
						{
							continue;
						}
						// The shares of the load: they sum to 1 and balance
						// the centre of mass in pitch and in roll.
						const double Wx = CPitch - DPitch[I];
						const double Wy = CRoll - DRoll[I];
						const double ShareJ = (Wx * Vy - Wy * Vx) / Det;
						const double ShareK = (Ux * Wy - Uy * Wx) / Det;
						const double ShareI = 1.0 - ShareJ - ShareK;
						if (FMath::Min3(ShareI, ShareJ, ShareK) < -LoadSlack)
						{
							continue;
						}
						// Where the three limits meet.
						const double R1 = -(H[J] - H[I]);
						const double R2 = -(H[K] - H[I]);
						const double Dp = (R1 * Vy - R2 * Uy) / Det;
						const double Dr = (Ux * R2 - Vx * R1) / Det;
						const double Z = -H[I] - DPitch[I] * Dp - DRoll[I] * Dr;
						bool		 bClear = true;
						for (int32 M = 0; M < Num && bClear; ++M)
						{
							bClear = Z + H[M] + DPitch[M] * Dp + DRoll[M] * Dr >= -GapSlackCm;
						}
						if (bClear)
						{
							OutZ = Z;
							OutDPitch = Dp;
							OutDRoll = Dr;
							return true;
						}
					}
				}
			}
			return false;
		}

		/**
		 * Each touching bottom's share of the weight: the least-norm loads
		 * that sum to 1 and balance about the centre of mass, seen from above,
		 * clipped at zero and brought back to a sum of 1. Even shares where
		 * the touching bottoms do not span the plane.
		 */
		void LoadShares(TConstArrayView<FVector> Bottoms, const FRestPose& Pose, double YawDeg, const FVector& CentreOfMass, FRow& OutShares)
		{
			const int32 Num = Bottoms.Num();
			OutShares.Init(0.0, Num);
			if (Pose.Touching <= 0 || Pose.bTouching.Num() != Num)
			{
				return;
			}
			const FTurn	  Turn = MakeTurn(FMath::DegreesToRadians(Pose.AttitudeDeg.X), FMath::DegreesToRadians(Pose.AttitudeDeg.Y), FMath::DegreesToRadians(YawDeg));
			const FVector Centre = TurnPoint(Turn, CentreOfMass).World;

			TArray<FVector2D, TInlineAllocator<8>> Arm;
			Arm.SetNumZeroed(Num);
			double Sx = 0.0;
			double Sy = 0.0;
			double Sxx = 0.0;
			double Sxy = 0.0;
			double Syy = 0.0;
			for (int32 Index = 0; Index < Num; ++Index)
			{
				if (Pose.bTouching[Index])
				{
					const FVector World = TurnPoint(Turn, Bottoms[Index]).World;
					Arm[Index] = FVector2D(World.X - Centre.X, World.Y - Centre.Y);
					Sx += Arm[Index].X;
					Sy += Arm[Index].Y;
					Sxx += Arm[Index].X * Arm[Index].X;
					Sxy += Arm[Index].X * Arm[Index].Y;
					Syy += Arm[Index].Y * Arm[Index].Y;
				}
			}
			// Loads f = m0 + m1 x + m2 y with (sum f, sum f x, sum f y) =
			// (1, 0, 0): a 3 x 3 system in m, solved by its first column's
			// cofactors.
			const double N = Pose.Touching;
			const double C0 = Sxx * Syy - Sxy * Sxy;
			const double C1 = -(Sx * Syy - Sxy * Sy);
			const double C2 = Sx * Sxy - Sxx * Sy;
			const double Det = N * C0 + Sx * C1 + Sy * C2;
			double		 Sum = 0.0;
			if (FMath::Abs(Det) > 1.0e-6)
			{
				for (int32 Index = 0; Index < Num; ++Index)
				{
					if (Pose.bTouching[Index])
					{
						OutShares[Index] = FMath::Max(0.0, (C0 + C1 * Arm[Index].X + C2 * Arm[Index].Y) / Det);
						Sum += OutShares[Index];
					}
				}
			}
			for (int32 Index = 0; Index < Num; ++Index)
			{
				if (Pose.bTouching[Index])
				{
					OutShares[Index] = Sum > 0.0 ? OutShares[Index] / Sum : 1.0 / N;
				}
			}
		}

		/** A stance ramp under way. */
		struct FRamp
		{
			int32  Leg = INDEX_NONE;
			double RateCmS = 0.0;
			double LeftCm = 0.0;
		};

		FRobotLeg MakeLeg(double X, double Y, double PairHalfTrackCm, double RadiusCm, double StanceZ, double MinZ, double MaxZ, double RateCmS,
			double AccelCmS2, bool bRoll)
		{
			FRobotLeg Leg;
			Leg.XY = FVector2D(X, Y);
			Leg.PairHalfTrackCm = PairHalfTrackCm;
			Leg.RadiusCm = RadiusCm;
			Leg.StanceZ = StanceZ;
			Leg.MinZ = MinZ;
			Leg.MaxZ = MaxZ;
			Leg.MaxRateCmS = RateCmS;
			Leg.MaxAccelCmS2 = AccelCmS2;
			Leg.bRoll = bRoll;
			return Leg;
		}

		FClearancePoint MakePoint(double X, double Y, double UndersideZ, double MinCm)
		{
			FClearancePoint Point;
			Point.XY = FVector2D(X, Y);
			Point.UndersideZ = UndersideZ;
			Point.MinCm = MinCm;
			return Point;
		}
	} // namespace DryRunImpl

	FGround Plane(double PitchDeg, double RollDeg)
	{
		const double TanPitch = FMath::Tan(FMath::DegreesToRadians(PitchDeg));
		const double TanRoll = FMath::Tan(FMath::DegreesToRadians(RollDeg));
		return [TanPitch, TanRoll](double X, double Y) { return X * TanPitch + Y * TanRoll; };
	}

	FGround Profile(TConstArrayView<FVector2D> Knots, double CrossDeg)
	{
		TArray<FVector2D> Line;
		Line.Append(Knots.GetData(), Knots.Num());
		const double TanCross = FMath::Tan(FMath::DegreesToRadians(CrossDeg));
		return [Line, TanCross](double X, double Y) {
			double Height = 0.0;
			if (Line.Num() > 0)
			{
				if (X <= Line[0].X)
				{
					Height = Line[0].Y;
				}
				else if (X >= Line.Last().X)
				{
					Height = Line.Last().Y;
				}
				else
				{
					for (int32 Index = 1; Index < Line.Num(); ++Index)
					{
						if (X <= Line[Index].X)
						{
							const FVector2D& From = Line[Index - 1];
							const FVector2D& To = Line[Index];
							const double	 Span = To.X - From.X;
							Height = Span > 0.0 ? From.Y + (To.Y - From.Y) * (X - From.X) / Span : To.Y;
							break;
						}
					}
				}
			}
			return Height + Y * TanCross;
		};
	}

	FGround Crown(double Deg)
	{
		const double TanFall = FMath::Tan(FMath::DegreesToRadians(Deg));
		return [TanFall](double /*X*/, double Y) { return -FMath::Abs(Y) * TanFall; };
	}

	FGround Fold(double Deg, double FoldYawDeg, double X0)
	{
		const double TanRise = FMath::Tan(FMath::DegreesToRadians(Deg));
		// The way straight away from the fold line: +X turned by the yaw.
		const double AwayX = FMath::Cos(FMath::DegreesToRadians(FoldYawDeg));
		const double AwayY = FMath::Sin(FMath::DegreesToRadians(FoldYawDeg));
		return [TanRise, AwayX, AwayY, X0](double X, double Y) { return FMath::Max(0.0, (X - X0) * AwayX + Y * AwayY) * TanRise; };
	}

	FRobot MakeSixWheelRobot()
	{
		FRobot			Robot;
		const FVector2D Wheels[6] = { { 45.0, -25.0 }, { 45.0, 25.0 }, { 0.0, -27.0 }, { 0.0, 27.0 }, { -45.0, -25.0 }, { -45.0, 25.0 } };
		for (const FVector2D& Wheel : Wheels)
		{
			FRobotLeg Leg;
			Leg.XY = Wheel;
			Robot.Legs.Add(Leg);
		}
		Robot.CentreOfMass = FVector(-8.0, 0.0, 10.0);
		const FVector2D Underside[4] = { { 40.0, -20.0 }, { 40.0, 20.0 }, { -40.0, -20.0 }, { -40.0, 20.0 } };
		for (const FVector2D& Point : Underside)
		{
			Robot.Points.Add(DryRunImpl::MakePoint(Point.X, Point.Y, -24.0, 3.0));
		}
		Robot.bSeat = true;
		return Robot;
	}

	FRobot MakeTwoDrivePairsRobot()
	{
		FRobot Robot;
		Robot.Legs.Add(DryRunImpl::MakeLeg(0.0, -28.0, 0.0, 15.0, -15.0, -23.0, -7.0, 6.0, 40.0, true));
		Robot.Legs.Add(DryRunImpl::MakeLeg(0.0, 28.0, 0.0, 15.0, -15.0, -23.0, -7.0, 6.0, 40.0, true));
		Robot.Legs.Add(DryRunImpl::MakeLeg(-40.0, 0.0, 20.0, 6.0, -24.0, -32.0, -16.0, 6.0, 40.0, false));
		Robot.Legs.Add(DryRunImpl::MakeLeg(45.0, 0.0, 22.0, 6.0, -20.0, -30.0, -14.0, 6.0, 40.0, false));
		Robot.CentreOfMass = FVector(-12.0, 0.0, 5.0);
		const FVector2D Underside[4] = { { 30.0, -20.0 }, { 30.0, 20.0 }, { -30.0, -20.0 }, { -30.0, 20.0 } };
		for (const FVector2D& Point : Underside)
		{
			Robot.Points.Add(DryRunImpl::MakePoint(Point.X, Point.Y, -22.0, 3.0));
		}
		Robot.bSeat = false;
		return Robot;
	}

	bool SolveRest(TConstArrayView<FVector> Bottoms, FGroundRef Ground, const FVector2D& ChassisXY, double YawDeg, const FVector& CentreOfMass,
		const FRestPose* Warm, FRestPose& Out)
	{
		using namespace DryRunImpl;

		// Worked out in a local: Warm may well be Out.
		const int32 Num = Bottoms.Num();
		FRestPose	Rest;
		Rest.GapCm.Init(0.0, Num);
		Rest.bTouching.Init(false, Num);
		if (Num == 0)
		{
			Out = MoveTemp(Rest);
			return false;
		}

		const double YawRad = FMath::DegreesToRadians(YawDeg);
		double		 PitchRad = Warm ? FMath::DegreesToRadians(Warm->AttitudeDeg.X) : 0.0;
		double		 RollRad = Warm ? FMath::DegreesToRadians(Warm->AttitudeDeg.Y) : 0.0;
		double		 Z = Warm ? Warm->Z : 0.0;

		// Each pass: every gap and the centre of mass's height are taken as
		// straight lines in (Z, pitch, roll) about the pose so far -- they
		// very nearly are -- the lowest rest of that is found, and the pose
		// moves there. A pass that hardly moves it has found the rest.
		FRow H;
		FRow DPitch;
		FRow DRoll;
		H.SetNumZeroed(Num);
		DPitch.SetNumZeroed(Num);
		DRoll.SetNumZeroed(Num);
		bool bRest = false;
		for (int32 Pass = 0; Pass < MaxPasses; ++Pass)
		{
			const FTurn Turn = MakeTurn(PitchRad, RollRad, YawRad);
			for (int32 Index = 0; Index < Num; ++Index)
			{
				const FTurned Bottom = TurnPoint(Turn, Bottoms[Index]);
				const double  WorldX = ChassisXY.X + Bottom.World.X;
				const double  WorldY = ChassisXY.Y + Bottom.World.Y;
				const double  GroundZ = Ground(WorldX, WorldY);
				const double  SlopeX = (Ground(WorldX + GradientStepCm, WorldY) - GroundZ) / GradientStepCm;
				const double  SlopeY = (Ground(WorldX, WorldY + GradientStepCm) - GroundZ) / GradientStepCm;
				H[Index] = Bottom.World.Z - GroundZ;
				// Turning the chassis lifts the bottom and also slides it
				// along the ground, which on a slope changes the ground's
				// height under it.
				DPitch[Index] = Bottom.X2 + Bottom.World.Z * (SlopeX * Turn.CosY + SlopeY * Turn.SinY);
				DRoll[Index] = Bottom.Y1 * Turn.CosP - SlopeX * (-Bottom.Y1 * Turn.SinP * Turn.CosY + Bottom.Z1 * Turn.SinY)
					- SlopeY * (-Bottom.Y1 * Turn.SinP * Turn.SinY - Bottom.Z1 * Turn.CosY);
			}
			const FTurned Centre = TurnPoint(Turn, CentreOfMass);

			double NewZ = Z;
			double StepPitch = 0.0;
			double StepRoll = 0.0;
			bRest = LowestRest(H, DPitch, DRoll, Centre.X2, Centre.Y1 * Turn.CosP, NewZ, StepPitch, StepRoll);
			if (!bRest)
			{
				break;
			}
			const double Largest = FMath::Max(FMath::Abs(StepPitch), FMath::Abs(StepRoll));
			const double Share = Largest > MaxTurnRad ? MaxTurnRad / Largest : 1.0;
			PitchRad += StepPitch * Share;
			RollRad += StepRoll * Share;
			Z = NewZ;
			if (Largest < SettledRad)
			{
				break;
			}
			// Not settled by the last pass: not a rest that can be trusted.
			bRest = Pass + 1 < MaxPasses;
		}

		// The gaps as they really are at the pose found, the chassis set down
		// (or picked up) so that the lowest bottom is on the ground.
		const FTurn Turn = MakeTurn(PitchRad, RollRad, YawRad);
		double		Lowest = 0.0;
		for (int32 Index = 0; Index < Num; ++Index)
		{
			const FVector World = TurnPoint(Turn, Bottoms[Index]).World;
			Rest.GapCm[Index] = World.Z - Ground(ChassisXY.X + World.X, ChassisXY.Y + World.Y);
			Lowest = Index == 0 ? Rest.GapCm[Index] : FMath::Min(Lowest, Rest.GapCm[Index]);
		}
		Rest.Z = -Lowest;
		for (int32 Index = 0; Index < Num; ++Index)
		{
			Rest.GapCm[Index] = FMath::Max(0.0, Rest.GapCm[Index] - Lowest);
			Rest.bTouching[Index] = Rest.GapCm[Index] <= RestTouchCm;
			Rest.Touching += Rest.bTouching[Index] ? 1 : 0;
		}
		Rest.AttitudeDeg = FVector2D(FMath::RadiansToDegrees(PitchRad), FMath::RadiansToDegrees(RollRad));
		Rest.bStable = bRest && Rest.Touching >= 3;
		Out = MoveTemp(Rest);
		return Out.bStable;
	}

	FResult Run(const FRobot& Robot, FGroundRef Ground, const FOptions& Options)
	{
		using namespace DryRunImpl;
		using FLawLegs = TArray<FLegReach, TInlineAllocator<8>>;
		using FMask = TArray<bool, TInlineAllocator<8>>;

		FResult		Result;
		const int32 NumLegs = Robot.Legs.Num();
		if (NumLegs == 0)
		{
			return Result;
		}
		// A copy: an event may change what level means on the way (EEvent::SetReference).
		FTuning		   Tuning = Options.Tuning;
		const double   FrameSeconds = FMath::Max(Options.FrameSeconds, 1.0e-3);
		const double   Tick = FMath::Clamp(Options.PhysicsStepSeconds, 1.0e-4, FrameSeconds);
		const int32	   PoseEvery = FMath::Max(1, FMath::RoundToInt32(Options.PoseStepSeconds / Tick));
		const double   PoseSeconds = PoseEvery * Tick;
		const double   DriveShare = Options.DriveLagSeconds > 0.0 ? FMath::Min(1.0, Tick / Options.DriveLagSeconds) : 1.0;
		const double   ServoShare = Options.ServoLagSeconds > 0.0 ? FMath::Min(1.0, Tick / Options.ServoLagSeconds) : 1.0;

		// ---- the plant ----
		FLegValues											Baseline;
		FLegValues											Offset;
		FLegValues											Joint;
		FLegValues											SagCm;
		FLegValues											Gain;
		FLegValues											StallUntil;
		TArray<RammsMotion::FFollower, TInlineAllocator<8>> Followers;
		Followers.SetNum(NumLegs);
		TArray<FVector, TInlineAllocator<8>> Bottoms;
		TArray<int32, TInlineAllocator<8>>	 BottomLeg;
		for (int32 Index = 0; Index < NumLegs; ++Index)
		{
			const FRobotLeg& Leg = Robot.Legs[Index];
			Baseline.Add(Leg.StanceZ);
			Offset.Add(0.0);
			Joint.Add(Leg.StanceZ);
			SagCm.Add(0.0);
			Gain.Add(Leg.Gain);
			StallUntil.Add(-1.0);
			Followers[Index].Reset(Leg.StanceZ);
			if (Leg.PairHalfTrackCm > 0.0)
			{
				Bottoms.Add(FVector(Leg.XY.X, Leg.XY.Y - Leg.PairHalfTrackCm, 0.0));
				BottomLeg.Add(Index);
				Bottoms.Add(FVector(Leg.XY.X, Leg.XY.Y + Leg.PairHalfTrackCm, 0.0));
				BottomLeg.Add(Index);
			}
			else
			{
				Bottoms.Add(FVector(Leg.XY.X, Leg.XY.Y, 0.0));
				BottomLeg.Add(Index);
			}
		}
		const int32 NumBottoms = Bottoms.Num();
		// The wheel's true height: the joint's, but for a leg whose real
		// travel is not what the law believes.
		const auto PlaceBottoms = [&]() {
			for (int32 Index = 0; Index < NumBottoms; ++Index)
			{
				const FRobotLeg& Leg = Robot.Legs[BottomLeg[Index]];
				Bottoms[Index].Z = Leg.StanceZ + Gain[BottomLeg[Index]] * (Joint[BottomLeg[Index]] - Leg.StanceZ) - Leg.RadiusCm;
			}
		};
		const auto MakeLegs = [&](FLawLegs& OutLegs) {
			OutLegs.Reset();
			for (int32 Index = 0; Index < NumLegs; ++Index)
			{
				const FRobotLeg& Leg = Robot.Legs[Index];
				FLegReach&		 Reach = OutLegs.AddDefaulted_GetRef();
				Reach.XY = Leg.XY;
				Reach.BaseZ = Baseline[Index];
				Reach.MeasuredZ = Joint[Index];
				Reach.MinZ = Leg.MinZ;
				Reach.MaxZ = Leg.MaxZ;
				Reach.MaxRateCmS = Leg.MaxRateCmS;
				Reach.RadiusCm = Leg.RadiusCm;
				Reach.bRoll = Leg.bRoll;
			}
		};

		FVector2D		Origin = Options.StartXY;
		const FVector2D Heading(FMath::Cos(FMath::DegreesToRadians(Options.YawDeg)), FMath::Sin(FMath::DegreesToRadians(Options.YawDeg)));
		double			Speed = 0.0;
		double			Travel = 0.0;
		double			SpeedCommand = Options.SpeedCmS;
		double			SpeedScale = 1.0;
		FVector2D		Kick = FVector2D::ZeroVector;
		double			KickSeconds = 0.0;

		FRestPose Pose;
		PlaceBottoms();
		bool bUnstable = !SolveRest(Bottoms, Ground, Origin, Options.YawDeg, Robot.CentreOfMass, nullptr, Pose);

		// ---- what the events have set going ----
		bool		  bEnable = Options.bEnabledAtStart;
		bool		  bTaken = false;
		double		  TakenUntil = 0.0;
		double		  WrongUntil = -1.0;
		uint32		  WrongMask = 0;
		double		  LossUntil = -1.0;
		double		  ReleaseUntil = -1.0;
		TArray<FRamp> Ramps;
		TArray<bool>  bFired;
		bFired.Init(false, Options.Events.Num());

		// Who stands, by the dry run's own count (for the floating wheels and
		// the hand-back's plane); not looked at while a taker has the legs,
		// when the baselines carry the folded plane.
		FMask Standing;
		{
			FLawLegs Legs;
			MakeLegs(Legs);
			SupportMask(Legs, TConstArrayView<bool>(), Tuning);
			for (const FLegReach& Leg : Legs)
			{
				Standing.Add(Leg.bSupport);
			}
		}

		FState		  Law;
		FRandomStream Random(Options.Seed);
		FVector2D	  LastReading = FVector2D::ZeroVector;
		bool		  bHaveReading = false;

		// ---- what is kept of every frame, for the figures at the end ----
		const int32		  ExpectedFrames = FMath::Max(1, FMath::CeilToInt32(FMath::Max(0.0, Options.MaxSeconds) / FrameSeconds) + 2);
		TArray<double>	  Times;
		TArray<FVector2D> TrueDeg;
		TArray<FVector2D> WantedDeg;
		TArray<bool>	  bWantedPlanned;
		TArray<bool>	  bWantedSet;
		TArray<bool>	  bWrote;
		TArray<double>	  Commands;
		TArray<int32>	  EventFrames;
		Times.Reserve(ExpectedFrames);
		TrueDeg.Reserve(ExpectedFrames);
		WantedDeg.Reserve(ExpectedFrames);
		bWantedPlanned.Reserve(ExpectedFrames);
		bWantedSet.Reserve(ExpectedFrames);
		bWrote.Reserve(ExpectedFrames);
		Commands.Reserve(ExpectedFrames * NumLegs);
		if (Options.bKeepTrace)
		{
			Result.Trace.Reserve(ExpectedFrames);
		}

		FRow FloatSpell;
		FRow Shares;
		FloatSpell.Init(0.0, NumBottoms);
		double SpeedSum = 0.0;
		double OffLevelSpell = 0.0;
		bool   bHasMoved = false;
		bool   bHaveMinSpeed = false;

		int64 StepIndex = 0;
		int64 FrameIndex = 0;
		while (true)
		{
			const double Time = StepIndex * Tick;

			if (Time >= FrameIndex * FrameSeconds - TimeHair)
			{
				// ======== one frame ========
				bool	  bBaselineMoved = false;
				bool	  bYield = false;
				bool	  bResume = false;
				FVector2D ResumePlaneDeg = FVector2D::ZeroVector;

				// The hand-back of a take-over that has run its time: the
				// plane the folded baselines carry is taken back out of them
				// and given to the law as offsets.
				if (bTaken && Time >= TakenUntil - TimeHair)
				{
					TArray<FVector2D, TInlineAllocator<8>> LegXY;
					FLegValues							   Carried;
					FMask								   Roll;
					FLawLegs							   Legs;
					MakeLegs(Legs);
					for (int32 Index = 0; Index < NumLegs; ++Index)
					{
						LegXY.Add(Robot.Legs[Index].XY);
						Carried.Add(Baseline[Index] - Robot.Legs[Index].StanceZ);
						Roll.Add(Robot.Legs[Index].bRoll);
						Legs[Index].bSupport = Standing[Index];
					}
					double	   HeightCm = 0.0;
					FLegValues Rebase;
					FitPlane(LegXY, Carried, Standing, Roll, ResumePlaneDeg, HeightCm);
					RigidOffsets(Legs, ResumePlaneDeg, Rebase);
					for (int32 Index = 0; Index < NumLegs; ++Index)
					{
						Baseline[Index] -= Rebase[Index];
						Offset[Index] = Rebase[Index];
					}
					bTaken = false;
					bResume = true;
					bBaselineMoved = true;
				}

				for (int32 EventIndex = 0; EventIndex < Options.Events.Num(); ++EventIndex)
				{
					const FDryEvent& Event = Options.Events[EventIndex];
					if (bFired[EventIndex] || Event.AtSeconds > Time + TimeHair)
					{
						continue;
					}
					bFired[EventIndex] = true;
					switch (Event.Kind)
					{
						case EEvent::Enable:
							bEnable = true;
							break;
						case EEvent::Disable:
							bEnable = false;
							break;
						case EEvent::SetSpeed:
							SpeedCommand = Event.Value;
							break;
						case EEvent::AttitudeKick:
							Kick += Event.PlaneDeg;
							KickSeconds = Event.Seconds;
							break;
						case EEvent::StallLeg:
							if (StallUntil.IsValidIndex(Event.Leg))
							{
								StallUntil[Event.Leg] = Time + Event.Seconds;
							}
							break;
						case EEvent::LegGain:
							for (int32 Index = 0; Index < NumLegs; ++Index)
							{
								if (Event.Leg == INDEX_NONE || Event.Leg == Index)
								{
									Gain[Index] = Event.Value;
								}
							}
							break;
						case EEvent::WrongSupport:
							WrongUntil = Time + Event.Seconds;
							WrongMask = Event.Mask;
							break;
						case EEvent::StanceRamp:
						{
							FRamp Ramp;
							Ramp.Leg = Event.Leg;
							Ramp.LeftCm = Event.Value;
							Ramp.RateCmS = Event.Seconds > 0.0 ? FMath::Abs(Event.Value) / Event.Seconds : 0.0;
							Ramps.Add(Ramp);
							break;
						}
						case EEvent::FoldAdopt:
							if (!bTaken)
							{
								bTaken = true;
								bYield = true;
								TakenUntil = Time + FMath::Max(Event.Seconds, FrameSeconds);
							}
							break;
						case EEvent::AttitudeLoss:
							LossUntil = Time + Event.Seconds;
							break;
						case EEvent::ModeRelease:
							ReleaseUntil = Time + Event.Seconds;
							break;
						case EEvent::SetReference:
							Tuning.LevelReferenceDeg = Event.PlaneDeg;
							break;
						default:
							break;
					}
				}

				// A drive mode's stance ramp: baseline writes, a frame at a time.
				for (int32 RampIndex = Ramps.Num() - 1; RampIndex >= 0; --RampIndex)
				{
					FRamp&		 Ramp = Ramps[RampIndex];
					const double Most = Ramp.RateCmS > 0.0 ? Ramp.RateCmS * FrameSeconds : FMath::Abs(Ramp.LeftCm);
					const double Move = FMath::Clamp(Ramp.LeftCm, -Most, Most);
					for (int32 Index = 0; Index < NumLegs; ++Index)
					{
						if (Ramp.Leg == INDEX_NONE || Ramp.Leg == Index)
						{
							Baseline[Index] += Move;
						}
					}
					Ramp.LeftCm -= Move;
					bBaselineMoved = bBaselineMoved || Move != 0.0;
					if (Ramp.LeftCm == 0.0)
					{
						Ramps.RemoveAt(RampIndex);
					}
				}

				// The sensor.
				const FVector2D TrueNow = Pose.AttitudeDeg + Kick;
				FVector2D		Reading = TrueNow + Options.BiasDeg;
				if (Options.NoiseDeg > 0.0)
				{
					Reading.X += Options.NoiseDeg * Random.FRandRange(-1.0f, 1.0f);
					Reading.Y += Options.NoiseDeg * Random.FRandRange(-1.0f, 1.0f);
				}
				const FVector2D Given = Options.bDelayOneFrame && bHaveReading ? LastReading : Reading;
				LastReading = Reading;
				bHaveReading = true;

				// The law.
				FLawLegs Legs;
				MakeLegs(Legs);
				if (!bTaken)
				{
					FLawLegs Own = Legs;
					SupportMask(Own, Standing, Tuning);
					for (int32 Index = 0; Index < NumLegs; ++Index)
					{
						Standing[Index] = Own[Index].bSupport;
					}
				}
				FInputs In;
				In.bSupportGiven = Time < WrongUntil - TimeHair;
				if (In.bSupportGiven)
				{
					for (int32 Index = 0; Index < NumLegs; ++Index)
					{
						Legs[Index].bSupport = Index < 32 && (WrongMask & (1u << Index)) != 0;
						Legs[Index].BaselineClearanceCm = 0.0;
					}
				}
				In.Legs = Legs;
				In.Points = Robot.Points;
				In.AttitudeDeg = Given;
				In.bAttitudeValid = !(Time < LossUntil - TimeHair);
				In.SpeedCmS = FMath::Abs(Speed);
				In.bStick = SpeedCommand != 0.0;
				In.bSeat = Robot.bSeat;
				In.bEnable = bEnable;
				In.Mode = bTaken ? EMode::Taken : (Time < ReleaseUntil - TimeHair ? EMode::Release : EMode::Allowed);
				In.bYield = bYield;
				In.bResume = bResume;
				In.bResumeHasPlane = bResume;
				In.ResumePlaneDeg = ResumePlaneDeg;
				const FOutputs Out = FLaw::Step(In, FrameSeconds, Tuning, Law);

				// The shell's part: fold, write, push.
				if (Out.Action == EAction::Fold)
				{
					for (int32 Index = 0; Index < NumLegs; ++Index)
					{
						Baseline[Index] += Offset[Index];
						Offset[Index] = 0.0;
					}
				}
				if (Out.bWrite && Out.Dz.Num() == NumLegs)
				{
					for (int32 Index = 0; Index < NumLegs; ++Index)
					{
						Offset[Index] = Out.Dz[Index];
					}
				}
				if (Out.bWrite || bBaselineMoved)
				{
					for (int32 Index = 0; Index < NumLegs; ++Index)
					{
						Followers[Index].Command(Baseline[Index] + Offset[Index], Time);
					}
				}
				SpeedScale = Out.SpeedScale;

				// ---- the frame's record ----
				if (Out.Action == EAction::Fold || Out.Action == EAction::Adopt)
				{
					EventFrames.Add(Times.Num());
				}
				Times.Add(Time);
				TrueDeg.Add(TrueNow);
				WantedDeg.Add(Out.WantedDeg);
				bWantedPlanned.Add((Out.State == EState::Active || Out.State == EState::Releasing) && !Out.bOpenLoop);
				bWantedSet.Add(Out.Action == EAction::StartFromAttitude || Out.Action == EAction::Adopt || Out.Action == EAction::ColdResume);
				bWrote.Add(Out.bWrite);
				double MaxOffsetCm = 0.0;
				for (int32 Index = 0; Index < NumLegs; ++Index)
				{
					Commands.Add(Baseline[Index] + Offset[Index]);
					MaxOffsetCm = FMath::Max(MaxOffsetCm, FMath::Abs(Offset[Index]));
				}
				int32 Floating = 0;
				for (int32 Index = 0; Index < NumBottoms; ++Index)
				{
					Floating += Standing[BottomLeg[Index]] && Pose.GapCm[Index] > Options.FloatGapCm ? 1 : 0;
				}

				Result.UnstableFrames += bUnstable ? 1 : 0;
				bUnstable = false;
				const double OffLevel = Inclination(TrueNow - Tuning.LevelReferenceDeg);
				Result.PeakTiltDeg = FMath::Max(Result.PeakTiltDeg, OffLevel);
				OffLevelSpell = OffLevel > Options.LagTiltDeg ? OffLevelSpell + FrameSeconds : 0.0;
				Result.LongestOffLevelSeconds = FMath::Max(Result.LongestOffLevelSeconds, OffLevelSpell);
				Result.FinalTiltDeg = OffLevel;
				Result.MinSpeedScale = FMath::Min(Result.MinSpeedScale, Out.SpeedScale);
				Result.SecondsAtReach += Out.Fraction < 1.0 ? FrameSeconds : 0.0;
				Result.MinReachLeft = FMath::Min(Result.MinReachLeft, Out.ReachLeft);
				Result.MaxCorrectionDeg = FMath::Max(Result.MaxCorrectionDeg, Out.AppliedDeg.Size());
				Result.MaxOffsetCm = FMath::Max(Result.MaxOffsetCm, MaxOffsetCm);
				Result.FinalMaxOffsetCm = MaxOffsetCm;
				Result.FinalState = Out.State;
				Result.FinalStatus = Out.Status;
				Result.bEverFaulted = Result.bEverFaulted || Out.State == EState::Faulted;
				SpeedSum += Speed;
				bHasMoved = bHasMoved || Speed > 2.0;
				if (bHasMoved && SpeedCommand > 0.0)
				{
					Result.MinSpeedCmS = bHaveMinSpeed ? FMath::Min(Result.MinSpeedCmS, Speed) : Speed;
					bHaveMinSpeed = true;
				}
				if (Options.bKeepTrace)
				{
					FSample& Sample = Result.Trace.AddDefaulted_GetRef();
					Sample.Seconds = Time;
					Sample.TravelCm = Travel;
					Sample.SpeedCmS = Speed;
					Sample.TrueAttitudeDeg = TrueNow;
					Sample.Out = Out;
					Sample.BaselineZ = Baseline;
					for (int32 Index = 0; Index < NumLegs; ++Index)
					{
						Sample.CommandZ.Add(Baseline[Index] + Offset[Index]);
					}
					Sample.Floating = Floating;
					Sample.Touching = Pose.Touching;
				}
				++FrameIndex;
			}

			Result.Seconds = Time;
			Result.TravelCm = Travel;
			if (Time >= Options.MaxSeconds - TimeHair || (Options.DistanceCm > 0.0 && Travel >= Options.DistanceCm))
			{
				break;
			}

			// ======== one physics step ========
			Speed += (SpeedCommand * SpeedScale - Speed) * DriveShare;
			Origin += Heading * (Speed * Tick);
			Travel += Speed * Tick;
			for (int32 Index = 0; Index < NumLegs; ++Index)
			{
				if (Time < StallUntil[Index])
				{
					continue; // a stalled leg holds, and its setpoint waits with it
				}
				const FRobotLeg& Leg = Robot.Legs[Index];
				const double	 Setpoint = Followers[Index].Step(Time, Tick, Leg.MaxRateCmS, Leg.MaxAccelCmS2);
				Joint[Index] += (Setpoint + SagCm[Index] - Joint[Index]) * ServoShare;
				Joint[Index] = FMath::Clamp(Joint[Index], Leg.MinZ, Leg.MaxZ);
			}
			if (KickSeconds > 0.0)
			{
				Kick *= FMath::Max(0.0, 1.0 - Tick / KickSeconds);
			}
			else
			{
				Kick = FVector2D::ZeroVector;
			}

			++StepIndex;
			if (StepIndex % PoseEvery == 0)
			{
				PlaceBottoms();
				bUnstable = !SolveRest(Bottoms, Ground, Origin, Options.YawDeg, Robot.CentreOfMass, &Pose, Pose) || bUnstable;

				bool bAnyFloating = false;
				for (int32 Index = 0; Index < NumBottoms; ++Index)
				{
					if (Standing[BottomLeg[Index]] && Pose.GapCm[Index] > Options.FloatGapCm)
					{
						FloatSpell[Index] += PoseSeconds;
						Result.LongestFloatSeconds = FMath::Max(Result.LongestFloatSeconds, FloatSpell[Index]);
						bAnyFloating = true;
					}
					else
					{
						FloatSpell[Index] = 0.0;
					}
				}
				Result.FloatingSeconds += bAnyFloating ? PoseSeconds : 0.0;

				if (Options.SagCm > 0.0)
				{
					// A joint gives under the load it carries.
					LoadShares(Bottoms, Pose, Options.YawDeg, Robot.CentreOfMass, Shares);
					for (int32 Index = 0; Index < NumLegs; ++Index)
					{
						SagCm[Index] = 0.0;
					}
					for (int32 Index = 0; Index < NumBottoms; ++Index)
					{
						SagCm[BottomLeg[Index]] += Options.SagCm * Shares[Index] * NumLegs;
					}
				}
			}
		}

		// ======== the figures ========
		const int32 NumFrames = Times.Num();
		Result.bCompleted = Result.UnstableFrames == 0;
		Result.MeanSpeedCmS = NumFrames > 0 ? SpeedSum / NumFrames : 0.0;

		// The wanted attitude's rate and acceleration, frame to frame.
		for (int32 Index = 1; Index < NumFrames; ++Index)
		{
			if (!bWantedPlanned[Index] || !bWantedPlanned[Index - 1] || bWantedSet[Index])
			{
				continue;
			}
			const FVector2D Rate = (WantedDeg[Index] - WantedDeg[Index - 1]) / FrameSeconds;
			Result.MaxWantedRateDegPerSec = FMath::Max(Result.MaxWantedRateDegPerSec, Rate.Size());
			if (Index >= 2 && bWantedPlanned[Index - 2] && !bWantedSet[Index - 1])
			{
				const FVector2D Before = (WantedDeg[Index - 1] - WantedDeg[Index - 2]) / FrameSeconds;
				Result.MaxWantedAccelDegPerSec2 = FMath::Max(Result.MaxWantedAccelDegPerSec2, (Rate - Before).Size() / FrameSeconds);
			}
		}

		// The true attitude's rate over 0.3 s, and how that rate changes.
		const int32	 Window = FMath::Max(1, FMath::RoundToInt32(0.3 / FrameSeconds));
		const double WindowSeconds = Window * FrameSeconds;
		for (int32 Index = Window; Index < NumFrames; ++Index)
		{
			const FVector2D Rate = (TrueDeg[Index] - TrueDeg[Index - Window]) / WindowSeconds;
			Result.MaxTrueRateDegPerSec = FMath::Max(Result.MaxTrueRateDegPerSec, Rate.Size());
			if (Index >= 2 * Window)
			{
				const FVector2D Before = (TrueDeg[Index - Window] - TrueDeg[Index - 2 * Window]) / WindowSeconds;
				Result.MaxTrueAccelDegPerSec2 = FMath::Max(Result.MaxTrueAccelDegPerSec2, (Rate - Before).Size() / WindowSeconds);
			}
		}

		// The legs' commands, frame to frame, and across each fold and adopt.
		for (int32 Index = 1; Index < NumFrames; ++Index)
		{
			double StepCm = 0.0;
			for (int32 LegIndex = 0; LegIndex < NumLegs; ++LegIndex)
			{
				StepCm = FMath::Max(StepCm, FMath::Abs(Commands[Index * NumLegs + LegIndex] - Commands[(Index - 1) * NumLegs + LegIndex]));
			}
			Result.MaxLegStepCm = FMath::Max(Result.MaxLegStepCm, StepCm);
			if (EventFrames.Contains(Index) || EventFrames.Contains(Index - 1))
			{
				Result.MaxEventLegStepCm = FMath::Max(Result.MaxEventLegStepCm, StepCm);
			}
		}

		// The steady figures, over the last stretch.
		if (NumFrames > 0)
		{
			const double From = Times.Last() - Options.SteadySeconds;
			double		 Sum = 0.0;
			int32		 Count = 0;
			FVector2D	 Low = TrueDeg.Last();
			FVector2D	 High = TrueDeg.Last();
			FLegValues	 LegLow;
			FLegValues	 LegHigh;
			for (int32 LegIndex = 0; LegIndex < NumLegs; ++LegIndex)
			{
				LegLow.Add(Commands[(NumFrames - 1) * NumLegs + LegIndex]);
				LegHigh.Add(LegLow[LegIndex]);
			}
			for (int32 Index = NumFrames - 1; Index >= 0 && Times[Index] >= From - TimeHair; --Index)
			{
				Sum += Inclination(TrueDeg[Index] - Tuning.LevelReferenceDeg);
				++Count;
				Low = FVector2D(FMath::Min(Low.X, TrueDeg[Index].X), FMath::Min(Low.Y, TrueDeg[Index].Y));
				High = FVector2D(FMath::Max(High.X, TrueDeg[Index].X), FMath::Max(High.Y, TrueDeg[Index].Y));
				for (int32 LegIndex = 0; LegIndex < NumLegs; ++LegIndex)
				{
					const double Command = Commands[Index * NumLegs + LegIndex];
					LegLow[LegIndex] = FMath::Min(LegLow[LegIndex], Command);
					LegHigh[LegIndex] = FMath::Max(LegHigh[LegIndex], Command);
				}
				Result.SteadyWrites += bWrote[Index] ? 1 : 0;
			}
			Result.SteadyResidualDeg = Count > 0 ? Sum / Count : 0.0;
			Result.SteadyTiltRangeDeg = FMath::Max(High.X - Low.X, High.Y - Low.Y);
			for (int32 LegIndex = 0; LegIndex < NumLegs; ++LegIndex)
			{
				Result.SteadyLegRangeCm = FMath::Max(Result.SteadyLegRangeCm, LegHigh[LegIndex] - LegLow[LegIndex]);
			}
		}
		return Result;
	}
} // namespace RammsLevel::DryRun
