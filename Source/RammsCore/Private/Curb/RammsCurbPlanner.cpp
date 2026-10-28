// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbPlanner.h"

#include "Curb/RammsCurbSpeedLaw.h"
#include "Curb/RammsCurbTerrainFit.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "Hash/CityHash.h"
#include "Async/ParallelFor.h"
#include "Misc/ScopeExit.h"

#include <atomic>

namespace
{
	TAutoConsoleVariable<int32> CVarCurbPlanDebug(TEXT("Ramms.Curb.PlanDebug"), 0,
		TEXT("1: the per-wheel planner adds one line per sample of the plan it keeps (or refuses with), and the tilt search's failing "
			 "samples, to its plan lines."),
		ECVF_Default);

	bool PlanDebug()
	{
		return CVarCurbPlanDebug.GetValueOnAnyThread() > 0;
	}

	/** 2 or more: also the per-sample lines of each tilt candidate's last
	 *  attempt that fails. */
	bool PlanDebugAttempts()
	{
		return CVarCurbPlanDebug.GetValueOnAnyThread() > 1;
	}

	TAutoConsoleVariable<int32> CVarCurbTiltAtSpeed(TEXT("Ramms.Curb.TiltAtSpeedSamples"), 0,
		TEXT("Development: how the least-tilt search takes the samples where the level plan at its own speeds finds no pose standing "
			 "(at the floor speed one stands there): 0 (the default) joined to the tilt stretches like the floor's failures, searched "
			 "with the floor's sets; 1 joined, searched with the sets the plan at its speeds has off the ground; 2 only widening a "
			 "stretch of the floor's failures they lie within the merge distance of, never making or joining stretches."),
		ECVF_Default);

	int32 TiltAtSpeedMode()
	{
		return FMath::Clamp(CVarCurbTiltAtSpeed.GetValueOnAnyThread(), 0, 2);
	}

	TAutoConsoleVariable<int32> CVarCurbTiltPicks(TEXT("Ramms.Curb.TiltPicks"), 0,
		TEXT("Development: which picks the tilt fallback's search tries for each stretch: 0 (the default) both, the ring's (the middle of the "
			 "widest arc of a ring of inclination that keeps a pose) and the square's (the first tilt of a larger axis that does); 1 the "
			 "ring's only; 2 the square's only. Kept for comparison."),
		ECVF_Default);

	int32 TiltPicksMode()
	{
		return FMath::Clamp(CVarCurbTiltPicks.GetValueOnAnyThread(), 0, 2);
	}

	TAutoConsoleVariable<int32> CVarCurbMaxTiltPlans(TEXT("Ramms.Curb.MaxTiltPlans"), 12,
		TEXT("The most tilted plans with stretches made larger the tilt fallback's search makes for one crossing (each costs tens to "
			 "hundreds of milliseconds), the least inclined first; every candidate's least tilts are planned besides. 0 or less: no "
			 "limit."),
		ECVF_Default);

	int32 MaxTiltPlans()
	{
		const int32 V = CVarCurbMaxTiltPlans.GetValueOnAnyThread();
		return V > 0 ? V : TNumericLimits<int32>::Max();
	}

	TAutoConsoleVariable<float> CVarCurbPreferFloor(TEXT("Ramms.Curb.PreferFloor"), 1.3f,
		TEXT("Development: over 0 (the default 1.3) the level search keeps trying the walking knots while its best "
			 "level plan's floor is under this share of the floor speed, and of level plans keeps the one with the most floor up to that, "
			 "then the widest corridor, then the most margin; 0 the first level plan found, as before. Kept for comparison."),
		ECVF_Default);

	bool PreferFloorOn()
	{
		return CVarCurbPreferFloor.GetValueOnAnyThread() > 0.0f;
	}

	double PreferFloorShare()
	{
		return FMath::Max(1.0, static_cast<double>(CVarCurbPreferFloor.GetValueOnAnyThread()));
	}

	TAutoConsoleVariable<int32> CVarCurbGentleTilt(TEXT("Ramms.Curb.GentleTilt"), 1,
		TEXT("Development: 1 (the default) a tilted plan that verifies is made again with its easings at GentleTiltShare of "
			 "TiltRateDegPerSec where its speeds give them the time, and kept so where it still verifies; 0 the whole rate always, "
			 "as before. Kept for comparison."),
		ECVF_Default);

	bool GentleTiltOn()
	{
		return CVarCurbGentleTilt.GetValueOnAnyThread() != 0;
	}

	TAutoConsoleVariable<int32> CVarCurbSlowBeforeTilt(TEXT("Ramms.Curb.SlowBeforeTilt"), 1,
		TEXT("Development: 1 (the default) a tilted plan whose samples fail at its own speeds is made again slower over them (never "
			 "under the floor) before the tilt search makes its tilt larger; 0 not, as before. Kept for comparison."),
		ECVF_Default);

	bool SlowBeforeTiltOn()
	{
		return CVarCurbSlowBeforeTilt.GetValueOnAnyThread() != 0;
	}

	TAutoConsoleVariable<float> CVarCurbLandingApproach(TEXT("Ramms.Curb.LandingApproach"), 0.0f,
		TEXT("Development: the planned chassis height moves at only LandingHeightShare of BaseHeightSpeed from this long (s) before a "
			 "wheel's touchdown (as it does from the touchdown until LandingSettleSeconds after it has taken load); 0 (the default) "
			 "from the touchdown, as before."),
		ECVF_Default);

	double LandingApproachSeconds()
	{
		return FMath::Max(0.0, static_cast<double>(CVarCurbLandingApproach.GetValueOnAnyThread()));
	}

	TAutoConsoleVariable<float> CVarCurbPlantedWalkShare(TEXT("Ramms.Curb.PlantedWalkFloorShare"), 0.85f,
		TEXT("Of the level plans whose floor is at least this share of the fastest one's, the one whose centre wheels walk least on the "
			 "ground is kept (walking them while they are lifted instead: a planted wheel walking fore/aft rolls at another speed than "
			 "the robot, and lands or lifts walking); 1 keeps the plan the floor alone picks, as before."),
		ECVF_Default);

	double PlantedWalkFloorShare()
	{
		return FMath::Clamp(static_cast<double>(CVarCurbPlantedWalkShare.GetValueOnAnyThread()), 0.0, 1.0);
	}

	TAutoConsoleVariable<int32> CVarCurbChoreography(TEXT("Ramms.Curb.Choreography"), 1,
		TEXT("1 (the default): the level plan walks the centre wheels through the crossing -- forward "
			 "(ChoreographyFrontCm from the chassis origin) while a front corner is off the ground, aft (ChoreographyRearCm) while "
			 "a rear one is, to ChoreographyLiftCm to lift and ChoreographyLandCm to land -- and that plan is kept whenever it is "
			 "level and its floor is at least PlantedWalkFloorShare of the no-walk plan's; 0: the centre wheels walk only where the "
			 "plan cannot otherwise stand, as before."),
		ECVF_Default);
	TAutoConsoleVariable<float> CVarCurbChoreographyFront(TEXT("Ramms.Curb.ChoreographyFrontCm"), 12.0f,
		TEXT("Where the centre wheels stand (cm ahead of the chassis origin) while a front corner is off the ground: the support's "
			 "front edge under the mass. Bounded by how far forward the centre legs reach with the chassis still inside its corner "
			 "legs' corridor; tune per robot with ramms.Curb.PlanReplay."),
		ECVF_Default);
	TAutoConsoleVariable<float> CVarCurbChoreographyRear(TEXT("Ramms.Curb.ChoreographyRearCm"), -4.0f,
		TEXT("Where the centre wheels stand (cm ahead of the chassis origin, so negative is aft) while a rear corner is off the "
			 "ground. Bounded by how far aft the centres can land with the rear legs still leaving the chassis a corridor; tune per "
			 "robot with ramms.Curb.PlanReplay."),
		ECVF_Default);
	TAutoConsoleVariable<float> CVarCurbChoreographyLift(TEXT("Ramms.Curb.ChoreographyLiftCm"), 6.0f,
		TEXT("Where a centre wheel comes to lift (cm ahead of the chassis origin): best where its 5-bar lifts highest, usually "
			 "where the drive stance holds it."),
		ECVF_Default);
	TAutoConsoleVariable<float> CVarCurbChoreographyLand(TEXT("Ramms.Curb.ChoreographyLandCm"), 6.0f,
		TEXT("Where a centre wheel lands (cm ahead of the chassis origin): landing aft of where it lifts can cost the plan its "
			 "floor."),
		ECVF_Default);

	TAutoConsoleVariable<int32> CVarCurbChoreographyRearDuringLift(TEXT("Ramms.Curb.ChoreographyRearDuringLift"), 0,
		TEXT("1: the centre wheels need stand at ChoreographyRearCm only once the rear corners are clear, walking aft from where "
			 "they landed while the rears lift (the seat forward meanwhile): landed further forward, they leave the rears a "
			 "longer run-up to the curb (a straight climb's floor 18 -> 20-24 cm/s in replays). The stability check at every "
			 "sample still bounds the walk. Off by default: in PIE it crossed no faster, and with landings disturbed the oblique "
			 "dry runs held and tilted more. 0 (the default): at ChoreographyRearCm before a rear corner leaves the ground."),
		ECVF_Default);

	bool ChoreographyOn()
	{
		return CVarCurbChoreography.GetValueOnAnyThread() != 0;
	}
	double ChoreographyCm(const TAutoConsoleVariable<float>& Var)
	{
		return static_cast<double>(Var.GetValueOnAnyThread());
	}
	bool RearDuringLiftOn()
	{
		return CVarCurbChoreographyRearDuringLift.GetValueOnAnyThread() != 0;
	}

	TAutoConsoleVariable<int32> CVarCurbSeatPath(TEXT("Ramms.Curb.SeatPath"), 1,
		TEXT("1 (the default): the plan's seat is the seat as it follows each instant's choice -- no faster than ShiftSpeedCmS at "
			 "the planned speed, never ahead of the choice -- verified where it has got to, the speed slowed where a pose needs it "
			 "further on. 0: each instant's own choice, the seat jumping where its home changes (the executor rate-limits it and "
			 "its measured margin gates the lifts)."),
		ECVF_Default);
	bool						SeatPathOn()
	{
		return CVarCurbSeatPath.GetValueOnAnyThread() != 0;
	}

	TAutoConsoleVariable<int32> CVarCurbSeatDrive(TEXT("Ramms.Curb.SeatDrive"), 1,
		TEXT("1 (the default): with the seat's ForeSlowDownCmS set, the plan's seat shifts as the executor moves it (eased by "
			 "ShiftAccelCmS2) and the crossing is timed with the drive slowed under each forward shift, as the executor runs it. "
			 "The lifts that follow the fronts' landing then take less of the travel, and the landing itself -- which the seat "
			 "cannot help, it waits for the fronts to carry -- is slowed less. 0: the plan times the crossing at its own speed and "
			 "the drive slows under it unplanned."),
		ECVF_Default);
	bool						SeatDriveOn()
	{
		return CVarCurbSeatDrive.GetValueOnAnyThread() != 0;
	}

	TAutoConsoleVariable<int32> CVarCurbRiseEarly(TEXT("Ramms.Curb.RiseEarly"), 1,
		TEXT("1 (the default): the chassis rises to the height the crossing's lifts and landings need before the first lift, during "
			 "the level-in on every wheel, and holds it until the last wheel has landed; 0: it rises as each lift needs it, as "
			 "before."),
		ECVF_Default);

	bool RiseEarlyOn()
	{
		return CVarCurbRiseEarly.GetValueOnAnyThread() != 0;
	}

	TAutoConsoleVariable<int32> CVarCurbWalkEarly(TEXT("Ramms.Curb.WalkEarly"), 1,
		TEXT("1 (the default): a centre wheel walks from one planned position to the next over the whole stretch between them, a "
			 "gentle ramp while the robot drives; 0: it stays where it is and walks as late as it can, at CenterShiftSpeed."),
		ECVF_Default);

	bool WalkEarlyOn()
	{
		return CVarCurbWalkEarly.GetValueOnAnyThread() != 0;
	}

	TAutoConsoleVariable<float> CVarCurbWalkSmooth(TEXT("Ramms.Curb.WalkSmoothCm"), 0.0f,
		TEXT("The centre wheels' walks (knots) are also planned smoothly: averaged over this much travel (cm) either side, so their "
			 "fore/aft speed changes over about twice that rather than at once (each kept over the plain walk unless its floor is "
			 "under Ramms.Curb.SmoothWalkFloorShare of it); 0 (the default) plans only the plain walks."),
		ECVF_Default);

	TAutoConsoleVariable<float> CVarCurbWalkHold(TEXT("Ramms.Curb.WalkHoldCm"), 0.0f,
		TEXT("A smooth centre wheel walk also stands still on the chassis this much travel (cm) either side of the wheel's lift-off and "
			 "its landing (Ramms.Curb.WalkHoldSpan says over what); 0 (the default) not: a smooth walk's wheel meets the ground at its "
			 "walk's own speed, which its spin is matched to."),
		ECVF_Default);

	TAutoConsoleVariable<int32> CVarCurbWalkHoldSpan(TEXT("Ramms.Curb.WalkHoldSpan"), 0,
		TEXT("Where a smooth walk's holds (Ramms.Curb.WalkHoldCm) fall: 0 (the default) around the lift-off and landing timed at the "
			 "cruise speed; 1 around them at any speed from the floor to the cruise."),
		ECVF_Default);

	TAutoConsoleVariable<float> CVarCurbSmoothWalkShare(TEXT("Ramms.Curb.SmoothWalkFloorShare"), 0.9f,
		TEXT("A smooth centre wheel walk is kept over the plain one of the same knots unless its floor is under this share of the "
			 "plain one's."),
		ECVF_Default);

	double WalkSmoothCm()
	{
		return FMath::Max(0.0, static_cast<double>(CVarCurbWalkSmooth.GetValueOnAnyThread()));
	}

	double WalkHoldCm()
	{
		return FMath::Max(0.0, static_cast<double>(CVarCurbWalkHold.GetValueOnAnyThread()));
	}

	bool WalkHoldWholeSpan()
	{
		return CVarCurbWalkHoldSpan.GetValueOnAnyThread() != 0;
	}

	double SmoothWalkFloorShare()
	{
		return FMath::Clamp(static_cast<double>(CVarCurbSmoothWalkShare.GetValueOnAnyThread()), 0.0, 1.0);
	}

	TAutoConsoleVariable<float> CVarCurbLiftHeightShare(TEXT("Ramms.Curb.LiftHeightShare"), 0.0f,
		TEXT("Development: while a wheel lifts (from where its lift begins until it is clear) the planned chassis height moves at only "
			 "this share of BaseHeightSpeed; 0 (the default) at all of it, as before."),
		ECVF_Default);

	double LiftHeightShare()
	{
		return FMath::Clamp(static_cast<double>(CVarCurbLiftHeightShare.GetValueOnAnyThread()), 0.0, 1.0);
	}

	TAutoConsoleVariable<float> CVarCurbLevelInAtRestShare(TEXT("Ramms.Curb.LevelInAtRestShare"), 1.3f,
		TEXT("Development: over 0 (the default 1.3) a robot standing still whose level-in on the way would leave it under this share "
			 "of the floor speed (its room too short for the level-in's time at that speed) levels in standing still instead, before it "
			 "moves; 0 it levels in on the way, the plan's speed held at the floor there, as before. Kept for comparison."),
		ECVF_Default);

	double LevelInAtRestShare()
	{
		return static_cast<double>(CVarCurbLevelInAtRestShare.GetValueOnAnyThread());
	}

	/** A grid stopped early (PlanCrossing): its summary begins so. */
	const TCHAR* const GridStoppedPrefix = TEXT("stopped: ");

	TAutoConsoleVariable<float> CVarCurbGridBudget(TEXT("Ramms.Curb.GridBudget"), 1.5f,
		TEXT("Development: over 0 (the default 1.5), on the move the crossing's two grids stop the one whose work (instants evaluated) "
			 "is over this share of the other's finished valid plan's and GridBudgetWork more, and a plan so over is not kept (the "
			 "robot does not wait seconds for the slower grid); 0 no budget, both grids' plans compared whatever they cost. "
			 "Kept for comparison."),
		ECVF_Default);

	/** The CVar's share, or 0 for no budget. */
	double GridBudgetShare()
	{
		const double V = static_cast<double>(CVarCurbGridBudget.GetValueOnAnyThread());
		return V > 0.0 ? FMath::Max(1.0, V) : 0.0;
	}

	TAutoConsoleVariable<int32> CVarCurbGridBudgetWork(TEXT("Ramms.Curb.GridBudgetWork"), 60000,
		TEXT("Development: with Ramms.Curb.GridBudget, the instants a grid's work may run over that share of the other grid's finished "
			 "plan's before it is over its budget on the move (60000, the default: a few hundred milliseconds' worth)."),
		ECVF_Default);

	TAutoConsoleVariable<int32> CVarCurbSlowTiltWhenNone(TEXT("Ramms.Curb.SlowTiltWhenNone"), 1,
		TEXT("Development: 1 (the default) where the tilt search finds no tilt within MaxPlannedTiltDeg at the plans' speeds, it is run "
			 "again with the plans made slower where they fail (never under the floor), up to the limit, before the planned stops; 0 "
			 "only where it found one (for a tilt a degree less), as before. Kept for comparison."),
		ECVF_Default);

	bool SlowTiltWhenNoneOn()
	{
		return CVarCurbSlowTiltWhenNone.GetValueOnAnyThread() != 0;
	}

	TAutoConsoleVariable<float> CVarCurbAnchorPhase(TEXT("Ramms.Curb.AnchorPhase"), 0.0f,
		TEXT("Development: with Ramms.Curb.AnchorGrid, the edge falls this share of a sample (0..1) past a sample rather than on one."),
		ECVF_Default);

	TAutoConsoleVariable<int32> CVarCurbAnchorGrid(TEXT("Ramms.Curb.AnchorGrid"), 2,
		TEXT("Development: 2 (the default) the crossing is planned twice, side by side, its samples laid on the terrain with the edge on "
			 "a sample and half a sample past one (the plan's origin moved back along the heading), and the better plan kept; 1 once, the "
			 "edge on a sample (Ramms.Curb.AnchorPhase past it); 0 from wherever the robot stands, as before. Kept for comparison."),
		ECVF_Default);

} // namespace

namespace RammsCurb
{
	// --- small pure helpers ------------------------------------------------------------

	/** The seat's fore/aft is chosen every this many cm across its reach, and a
	 *  pose whose margin is this much over the requirement needs no more:
	 *  nearer the zero pose wins among those. */
	constexpr double SeatStepCm = 2.0;
	constexpr double SeatEnoughCm = 3.0;

	bool IsForbiddenSet(const FRobotModel& Model, uint32 Airborne)
	{
		// With a seat the crossing shifts, the margin decides what may be up
		// together: a set the gait never lifted at once stands if the seat's
		// shift keeps the centre of mass inside the wheels left down.
		if (Model.Seat.Shifts())
		{
			return false;
		}
		int32 FrontUp = 0, RearUp = 0, CentreUp = 0;
		for (int32 Side = 0; Side < 2; ++Side)
		{
			bool bCorner = false, bCentre = false;
			for (int32 i = 0; i < Model.Legs.Num(); ++i)
			{
				const FModelLeg& L = Model.Legs[i];
				if ((L.Y > 0.0 ? 1 : 0) != Side || !((Airborne >> i) & 1u))
				{
					continue;
				}
				if (L.Axle == ERammsCurbAxle::Center)
				{
					bCentre = true;
				}
				else
				{
					bCorner = true;
				}
			}
			if (bCorner && bCentre)
			{
				return true;
			}
		}
		for (int32 i = 0; i < Model.Legs.Num(); ++i)
		{
			if ((Airborne >> i) & 1u)
			{
				const ERammsCurbAxle A = Model.Legs[i].Axle;
				FrontUp += A == ERammsCurbAxle::Front;
				RearUp += A == ERammsCurbAxle::Rear;
				CentreUp += A == ERammsCurbAxle::Center;
			}
		}
		return CentreUp > 0 && (FrontUp >= 2 || RearUp >= 2);
	}

	double TrapezoidSeconds(double DistanceCm, double SpeedCmS, double AccelCmS2)
	{
		const double D = FMath::Abs(DistanceCm);
		if (D <= 0.0)
		{
			return 0.0;
		}
		const double V = FMath::Max(SpeedCmS, 1e-3);
		const double A = FMath::Max(AccelCmS2, 1e-3);
		return D >= V * V / A ? D / V + V / A : 2.0 * FMath::Sqrt(D / A);
	}

	double TrapezoidAt(double Seconds, double DistanceCm, double SpeedCmS, double AccelCmS2)
	{
		const double D = FMath::Abs(DistanceCm);
		const double Total = TrapezoidSeconds(D, SpeedCmS, AccelCmS2);
		if (Seconds <= 0.0 || D <= 0.0)
		{
			return 0.0;
		}
		if (Seconds >= Total)
		{
			return D;
		}
		const double V = FMath::Max(SpeedCmS, 1e-3);
		const double A = FMath::Max(AccelCmS2, 1e-3);
		double		 Ramp = V / A;
		if (D < V * V / A)
		{
			Ramp = 0.5 * Total; // a triangle
		}
		if (Seconds < Ramp)
		{
			return 0.5 * A * Seconds * Seconds;
		}
		if (Seconds > Total - Ramp)
		{
			const double Left = Total - Seconds;
			return D - 0.5 * A * Left * Left;
		}
		return 0.5 * A * Ramp * Ramp + V * (Seconds - Ramp);
	}

	double SCurveSeconds(double Delta, double Rate, double Accel)
	{
		return TrapezoidSeconds(Delta, Rate, Accel);
	}

	double SCurveShare(double T, double Delta, double Rate, double Accel)
	{
		const double D = FMath::Abs(Delta);
		return D <= 0.0 ? 1.0 : TrapezoidAt(T, D, Rate, Accel) / D;
	}

	bool TiltProfile(TConstArrayView<double> SampleSeconds, TConstArrayView<double> SampleS, TConstArrayView<FTiltHold> Holds, double Rate,
		double Accel, TArray<FVector2D>& OutTilt, TArray<FTiltGap>* OutTight, TConstArrayView<uint8> TightModes)
	{
		const int32 N = SampleSeconds.Num();
		OutTilt.Init(FVector2D::ZeroVector, N);
		if (OutTight)
		{
			OutTight->Reset();
		}
		if (N == 0)
		{
			return true;
		}
		Rate = FMath::Max(Rate, 1e-3);
		Accel = FMath::Max(Accel, 1e-3);
		TArray<FTiltHold> Sorted(Holds.GetData(), Holds.Num());
		Sorted.StableSort([](const FTiltHold& A, const FTiltHold& B) { return A.First < B.First; });
		auto			   At = [&](int32 k) { return SampleSeconds[FMath::Clamp(k, 0, N - 1)]; };
		auto			   SAt = [&](int32 k) { return SampleS.IsValidIndex(k) ? SampleS[k] : static_cast<double>(k); };
		auto			   Sc = [&](double D) { return SCurveSeconds(FMath::Abs(D), Rate, Accel); };
		auto			   Share = [&](double Tau, double D) { return Tau <= 0.0 ? 0.0 : SCurveShare(Tau, D, Rate, Accel); };
		bool			   bFits = true;
		TArray<FVector2D>& V = OutTilt;
		for (const FTiltHold& H : Sorted)
		{
			for (int32 k = FMath::Max(0, H.First); k <= FMath::Min(N - 1, H.Last); ++k)
			{
				V[k] = H.Tilt;
			}
		}
		// The transitions: from level into the first hold, between holds, out of
		// the last back to level. Each is (from sample E at tilt A to sample S at
		// tilt B), one S-curve along the straight way from A to B -- pitch and
		// roll together, so (the inclination's level sets being convex) the base
		// never leans further on the way than the more inclined of A and B --
		// timed by the whole turn, so that the base turns (pitch and roll
		// together) no faster than Rate nor its turn rate changes faster than
		// Accel; level at either end of the plan is free to begin or end
		// anywhere.
		for (int32 h = 0; h <= Sorted.Num(); ++h)
		{
			const bool		bFirst = h == 0;
			const bool		bLast = h == Sorted.Num();
			const int32		E = bFirst ? 0 : FMath::Min(N - 1, Sorted[h - 1].Last);
			const int32		S = bLast ? N - 1 : FMath::Max(0, Sorted[h].First);
			const FVector2D A = bFirst ? FVector2D::ZeroVector : Sorted[h - 1].Tilt;
			const FVector2D B = bLast ? FVector2D::ZeroVector : Sorted[h].Tilt;
			if (bFirst && bLast)
			{
				break;
			}
			const double TE = At(E), TS = At(S);
			const double G = TS - TE;
			const double DA = A.Size(), DB = B.Size(), DAB = (B - A).Size();
			const double TA = Sc(DA), TB = Sc(DB), TAB = Sc(DAB);
			auto		 Tight = [&]() {
				bFits = false;
				if (OutTight)
				{
					FTiltGap Gap;
					Gap.First = E;
					Gap.Last = S;
					Gap.Hold = h - 1;
					Gap.NeedSeconds = TAB;
					Gap.CapCmS = TAB > 0.0 ? FMath::Max(0.0, SAt(S) - SAt(E)) / TAB : TNumericLimits<double>::Max();
					// One gap per pair of holds: the tighter.
					for (FTiltGap& Old : *OutTight)
					{
						if (Old.First == Gap.First && Old.Last == Gap.Last)
						{
							if (Gap.CapCmS < Old.CapCmS)
							{
								Old = Gap;
							}
							return;
						}
					}
					OutTight->Add(Gap);
				}
			};
			if (bFirst)
			{
				// Eased in from level, ending as the hold begins (or, with too
				// little time from the start, from the start, into the hold).
				double TEnd = TS;
				if (TB > TS - At(0) + 1e-9)
				{
					Tight();
					TEnd = At(0) + TB;
				}
				for (int32 k = 0; k < N && At(k) < TEnd; ++k)
				{
					V[k] = B * (1.0 - Share(TEnd - At(k), DB));
				}
				continue;
			}
			if (bLast)
			{
				// Eased out to level from where the last hold ends (or, with too
				// little time to the end, so as to be level there).
				double TBegin = TE;
				if (TA > At(N - 1) - TE + 1e-9)
				{
					Tight();
					TBegin = At(N - 1) - TA;
				}
				for (int32 k = N - 1; k >= 0 && At(k) > TBegin; --k)
				{
					V[k] = A * (1.0 - Share(At(k) - TBegin, DA));
				}
				continue;
			}
			if (A == B && (A.IsZero() || S <= E + 1 || G < TA + TB - 1e-9))
			{
				// The same tilt either side and no time to level between: held.
				for (int32 k = E + 1; k < S; ++k)
				{
					V[k] = A;
				}
				continue;
			}
			if (S > E && G >= TA + TB - 1e-9)
			{
				// Back to level after the one and out again before the other.
				for (int32 k = E + 1; k < S; ++k)
				{
					V[k] = A * (1.0 - Share(At(k) - TE, DA)) + B * (1.0 - Share(TS - At(k), DB));
				}
				continue;
			}
			// Straight from one to the other: out of the more inclined early,
			// into the more inclined late; centred where they lean apart. With
			// too little time between them, centred on the gap and running into
			// the holds either side (they are shortened, not the easing hurried;
			// whether their samples stand so is the plan's verification's to
			// say).
			double T0 = TE;
			if (FVector2D::DotProduct(A, B) < 0.0)
			{
				T0 = TE + 0.5 * FMath::Max(0.0, G - TAB);
			}
			else if (DB > DA)
			{
				T0 = TS - TAB;
			}
			// (Holds that touch or overlap have no time between them at all:
			// the easing is centred where they meet.)
			int32 KA = FMath::Min(E + 1, S), KB = FMath::Max(S - 1, E);
			if (TAB > G + 1e-9)
			{
				Tight();
				// Centred on the gap, or ending as the later hold begins (into
				// the earlier), or beginning as the earlier ends (into the later).
				const uint8 Mode = TightModes.IsValidIndex(h) ? TightModes[h] : 0;
				T0 = Mode == 1 ? TS - TAB : (Mode == 2 ? TE : 0.5 * (TE + TS) - 0.5 * TAB);
				while (KA > 0 && At(KA - 1) > T0)
				{
					--KA;
				}
				while (KB < N - 1 && At(KB + 1) < T0 + TAB)
				{
					++KB;
				}
			}
			for (int32 k = KA; k <= KB; ++k)
			{
				V[k] = A + (B - A) * Share(At(k) - T0, DAB);
			}
		}
		return bFits;
	}

	TArray<FVector2D> TiltCandidates(double T, double Grid)
	{
		TArray<FVector2D> Out;
		if (T <= 0.0)
		{
			return Out;
		}
		Grid = FMath::Max(Grid, 1e-3);
		// A quarter of the ring in Quarter steps of azimuth, about Grid apart
		// along it; each step from the nearest axis taken on all eight sides
		// (along pitch and along roll, both signs of each), the axes first.
		const int32	 Quarter = FMath::Max(1, FMath::CeilToInt(0.5 * UE_DOUBLE_PI * T / Grid - 1e-9));
		const double Step = 0.5 * UE_DOUBLE_PI / Quarter;
		const double TanT = FMath::Tan(FMath::DegreesToRadians(T));
		for (int32 j = 0; j <= Quarter / 2; ++j)
		{
			const double	Big = j == 0 ? T : FMath::RadiansToDegrees(FMath::Atan(TanT * FMath::Cos(j * Step)));
			const double	Small = j == 0 ? 0.0 : FMath::RadiansToDegrees(FMath::Atan(TanT * FMath::Sin(j * Step)));
			const FVector2D Ring[8] = { { Big, Small }, { -Big, Small }, { Small, Big }, { Small, -Big }, { Big, -Small }, { -Big, -Small },
				{ -Small, Big }, { -Small, -Big } };
			for (const FVector2D& D : Ring)
			{
				Out.AddUnique(D);
			}
		}
		return Out;
	}

	FVector2D TiltAtInclination(const FVector2D& Direction, double InclinationDeg)
	{
		const FVector2D Tan(FMath::Tan(FMath::DegreesToRadians(Direction.X)), FMath::Tan(FMath::DegreesToRadians(Direction.Y)));
		const double	Length = Tan.Size();
		if (Length < 1e-12 || InclinationDeg <= 0.0)
		{
			return FVector2D::ZeroVector;
		}
		const double TanI = FMath::Tan(FMath::DegreesToRadians(InclinationDeg));
		return FVector2D(FMath::RadiansToDegrees(FMath::Atan(TanI * Tan.X / Length)), FMath::RadiansToDegrees(FMath::Atan(TanI * Tan.Y / Length)));
	}

	bool LeastInclinationAt(double T, double Grid, TFunctionRef<bool(const FVector2D&)> Stands, FVector2D& Out)
	{
		const TArray<FVector2D> Ring = TiltCandidates(T, Grid);
		const int32				N = Ring.Num();
		if (N == 0)
		{
			return false;
		}
		// Every tilt of the ring, in order round it (by the way it leans).
		TArray<int32> Round;
		Round.SetNum(N);
		for (int32 i = 0; i < N; ++i)
		{
			Round[i] = i;
		}
		auto Azimuth = [&](int32 i) {
			return FMath::Atan2(FMath::Tan(FMath::DegreesToRadians(Ring[i].Y)), FMath::Tan(FMath::DegreesToRadians(Ring[i].X)));
		};
		Round.StableSort([&](int32 A, int32 B) { return Azimuth(A) < Azimuth(B); });
		TArray<uint8> Ok;
		Ok.SetNumZeroed(N);
		int32 NumOk = 0;
		for (int32 j = 0; j < N; ++j)
		{
			Ok[j] = Stands(Ring[Round[j]]) ? 1 : 0;
			NumOk += Ok[j];
		}
		if (NumOk == 0)
		{
			return false;
		}
		// The one furthest round the ring from a tilt that does not stand: the
		// middle of the widest arc that stands (ties: the first in
		// TiltCandidates' order; all standing: the first).
		int32 Best = INDEX_NONE, BestGap = -1;
		for (int32 j = 0; j < N; ++j)
		{
			if (!Ok[j])
			{
				continue;
			}
			int32 Gap = N;
			if (NumOk < N)
			{
				int32 Left = 1, Right = 1;
				while (Ok[(j - Left + N) % N])
				{
					++Left;
				}
				while (Ok[(j + Right) % N])
				{
					++Right;
				}
				Gap = FMath::Min(Left, Right);
			}
			if (Gap > BestGap || (Gap == BestGap && Round[j] < Round[Best]))
			{
				Best = j;
				BestGap = Gap;
			}
		}
		Out = Ring[Round[Best]];
		return true;
	}

	TArray<FVector2D> SquareCandidates(double T, double Grid)
	{
		TArray<FVector2D> Out;
		if (T <= 0.0)
		{
			return Out;
		}
		Grid = FMath::Max(Grid, 1e-3);
		const int32 Steps = FMath::FloorToInt(T / Grid + 1e-9);
		for (int32 j = 0; j <= Steps; ++j)
		{
			const double	U = FMath::Min(j * Grid, T);
			const FVector2D Square[8] = { { T, U }, { -T, U }, { U, T }, { U, -T }, { T, -U }, { -T, -U }, { -U, T }, { -U, -T } };
			for (const FVector2D& D : Square)
			{
				Out.AddUnique(D);
			}
		}
		return Out;
	}

	double PickSize(ETiltPick Pick, const FVector2D& Tilt)
	{
		return Pick == ETiltPick::Square ? FMath::Max(FMath::Abs(Tilt.X), FMath::Abs(Tilt.Y)) : InclinationDeg(Tilt.X, Tilt.Y);
	}

	FVector2D ScaledTilt(ETiltPick Pick, const FVector2D& Tilt, double Size)
	{
		if (Pick == ETiltPick::Ring)
		{
			return TiltAtInclination(Tilt, Size);
		}
		const double Now = PickSize(Pick, Tilt);
		return Now > 1e-12 && Size > 0.0 ? Tilt * (Size / Now) : FVector2D::ZeroVector;
	}

	bool PickTiltAt(ETiltPick Pick, double Size, double Grid, double MaxTilt, TFunctionRef<bool(const FVector2D&)> Stands, FVector2D& Out)
	{
		if (Size <= 0.0)
		{
			return false;
		}
		if (Pick == ETiltPick::Ring)
		{
			return Size <= MaxTilt + 1e-9 && LeastInclinationAt(Size, Grid, Stands, Out);
		}
		for (const FVector2D& D : SquareCandidates(Size, Grid))
		{
			if (InclinationDeg(D.X, D.Y) <= MaxTilt + 1e-9 && Stands(D))
			{
				Out = D;
				return true;
			}
		}
		return false;
	}

	bool PickLeastTilt(ETiltPick Pick, double Grid, double MaxTilt, TFunctionRef<bool(const FVector2D&)> Stands, FVector2D& Out)
	{
		Grid = FMath::Max(Grid, 1e-3);
		// Sizes up to MaxTilt: a ring's is its inclination, and a square of a
		// larger axis over MaxTilt inclines the base more than that everywhere.
		const int32 Num = FMath::FloorToInt(MaxTilt / Grid + 1e-9);
		for (int32 n = 1; n <= Num; ++n)
		{
			if (PickTiltAt(Pick, n * Grid, Grid, MaxTilt, Stands, Out))
			{
				return true;
			}
		}
		return false;
	}

	double TiltStep(double Size, double Step)
	{
		Step = FMath::Max(Step, 1e-3);
		return FMath::CeilToDouble(Size / Step - 1e-6) * Step;
	}

	void SortTiltTries(TArray<FTiltTry>& Tries, double Step)
	{
		Tries.StableSort([Step](const FTiltTry& A, const FTiltTry& B) {
			const double SA = TiltStep(A.Size, Step), SB = TiltStep(B.Size, Step);
			if (FMath::Abs(SA - SB) > 1e-9)
			{
				return SA < SB;
			}
			if ((A.More > 0.0) != (B.More > 0.0))
			{
				return A.More <= 0.0;
			}
			if (A.Candidate != B.Candidate)
			{
				return A.Candidate < B.Candidate;
			}
			return A.More != B.More ? A.More < B.More : A.Way < B.Way;
		});
	}

	FTiltSearchResult SearchTilts(TConstArrayView<FTiltSearchCandidate> Candidates, double MaxTilt, double Grid, TConstArrayView<double> Steps,
		int32 MaxPlans, TFunctionRef<bool(int32, int32, const FVector2D&)> Stands, TFunctionRef<bool(const FTiltTry&, TArray<int32>&)> Plan)
	{
		FTiltSearchResult Result;
		auto			  SizeOf = [](TConstArrayView<FVector2D> Tilts) {
			 double Most = 0.0;
			 for (const FVector2D& T : Tilts)
			 {
				 Most = FMath::Max(Most, InclinationDeg(T.X, T.Y));
			 }
			 return Most;
		};
		auto			 Cut = [&](double Incl) { Result.CutDeg = Result.CutDeg < 0.0 ? Incl : FMath::Min(Result.CutDeg, Incl); };
		TArray<FTiltTry> Queue;
		for (int32 c = 0; c < Candidates.Num(); ++c)
		{
			FTiltTry Try;
			Try.Candidate = c;
			Try.Tilts = Candidates[c].Tilts;
			Try.Size = SizeOf(Try.Tilts);
			if (Try.Tilts.Num() == 0)
			{
				continue;
			}
			if (Try.Size > MaxTilt + 1e-9)
			{
				Cut(Try.Size);
				continue;
			}
			Queue.Add(MoveTemp(Try));
		}
		SortTiltTries(Queue, TiltOrderDeg);
		// The plans made: whose candidate plan, which tilts, and what their
		// failure was about.
		struct FMade
		{
			int32			  Base = 0;
			TArray<FVector2D> Tilts;
			TArray<int32>	  Failing;
		};
		TArray<FMade> Made;
		auto		  Same = [](TConstArrayView<FVector2D> A, TConstArrayView<FVector2D> B) {
			 if (A.Num() != B.Num())
			 {
				 return false;
			 }
			 for (int32 i = 0; i < A.Num(); ++i)
			 {
				 if (!A[i].Equals(B[i], 1e-9))
				 {
					 return false;
				 }
			 }
			 return true;
		};
		while (Queue.Num() > 0)
		{
			FTiltTry Try = Queue[0];
			Queue.RemoveAt(0);
			const FTiltSearchCandidate& C = Candidates[Try.Candidate];
			// (A tilt set already planned on the same candidate plan -- two
			// picks or ways agreeing -- is not planned again; a least tilt's
			// escalations still join, from that plan's failure.)
			const FMade*  Done = Made.FindByPredicate([&](const FMade& M) { return M.Base == C.Base && Same(M.Tilts, Try.Tilts); });
			TArray<int32> Failing;
			if (Done)
			{
				Failing = Done->Failing;
			}
			else
			{
				// Every candidate's least tilts are planned; of the larger ones,
				// at most MaxPlans (the least inclined: what is left untried is
				// the most inclined, and no candidate's least tilt).
				if (Try.More > 0.0)
				{
					if (Result.LargerPlans >= MaxPlans)
					{
						Result.bCapped = true;
						continue;
					}
					++Result.LargerPlans;
				}
				++Result.Plans;
				if (Plan(Try, Failing))
				{
					Result.bFound = true;
					Result.Planned = MoveTemp(Try);
					return Result;
				}
				Made.Add({ C.Base, Try.Tilts, Failing });
			}
			if (Try.More > 0.0)
			{
				continue;
			}
			// A candidate's least tilts did not verify: the stretches its plan
			// failed about, each step larger, join the tries.
			for (const double More : Steps)
			{
				for (int32 Way = 0; Way < 2 && More > 0.0; ++Way)
				{
					FTiltTry Larger;
					Larger.Candidate = Try.Candidate;
					Larger.More = More;
					Larger.Way = Way;
					Larger.Tilts = C.Tilts;
					for (const int32 r : Failing)
					{
						if (!C.Tilts.IsValidIndex(r))
						{
							continue;
						}
						const double Size = PickSize(C.Pick, C.Tilts[r]) + More;
						FVector2D	 To = ScaledTilt(C.Pick, C.Tilts[r], Size);
						FVector2D	 Picked;
						if (Way == 0 && PickTiltAt(C.Pick, Size, Grid, MaxTilt, [&](const FVector2D& D) { return Stands(Try.Candidate, r, D); }, Picked))
						{
							To = Picked;
						}
						const double Incl = InclinationDeg(To.X, To.Y);
						if (Incl > MaxTilt + 1e-9)
						{
							Cut(Incl);
							continue;
						}
						Larger.Tilts[r] = To;
						Larger.Larger.Add(r);
					}
					if (Larger.Larger.Num() == 0)
					{
						continue;
					}
					Larger.Size = SizeOf(Larger.Tilts);
					Queue.Add(MoveTemp(Larger));
				}
			}
			SortTiltTries(Queue, TiltOrderDeg);
		}
		return Result;
	}

	void KeepOffWindow(double RadiusCm, double TyreWidthCm, const FVector2D& RollDirection, const FVector2D& EdgeNormal,
		bool bClimb, const FRammsCurbTuning& Tuning, double& OutUp, double& OutDown, double& OutEps, double& OutLambda)
	{
		const FVector2D F = RollDirection.GetSafeNormal();
		const FVector2D A(-F.Y, F.X);
		OutLambda = 0.5 * FMath::Max(0.0, TyreWidthCm) * FMath::Abs(FVector2D::DotProduct(A, EdgeNormal));
		OutEps = RadiusCm * FMath::Abs(FVector2D::DotProduct(F, EdgeNormal)) + OutLambda;
		if (bClimb)
		{
			OutUp = -Tuning.FaceMargin - OutEps;
			OutDown = Tuning.LandingOverlap + OutLambda;
		}
		else
		{
			OutUp = -Tuning.EdgeMargin - OutLambda;
			OutDown = Tuning.DropClearance + OutEps;
		}
	}

	// --- the per-instant model --------------------------------------------------------------

	namespace
	{
		constexpr double ColumnStepCm = 1.0;
		constexpr double ColumnFromCm = -40.0;
		constexpr int32	 NumColumns = 81;

		FVector2D XYOf(const FVector& V)
		{
			return FVector2D(V.X, V.Y);
		}
	} // namespace

	FPlanModel::FPlanModel(const FPlannerInput& In)
		: Model(In.Model)
		, Terrain(In.Terrain)
		, Tuning(In.Tuning)
	{
		Heading = In.Heading.GetSafeNormal();
		if (Heading.IsNearlyZero())
		{
			Heading = FVector2D(1.0, 0.0);
		}
		// +Y is the robot's right.
		Right = FVector2D(-Heading.Y, Heading.X);
		Origin = In.Origin;
		CosApproach = FMath::Max(1e-3, FVector2D::DotProduct(Heading, Terrain.EdgeNormal));
		bClimb = Terrain.StepHeightCm >= 0.0f;
		Dilation = bClimb ? Tuning.FaceMargin : Tuning.EdgeMargin;
		Level.X = FVector(Heading.X, Heading.Y, 0.0);
		Level.Y = FVector(Right.X, Right.Y, 0.0);
		Level.Z = FVector::UpVector;
		const int32 N = FMath::Min(Model.Legs.Num(), In.Legs.Num());
		Legs.SetNum(N);
		for (int32 i = 0; i < N; ++i)
		{
			const FModelLeg&   ML = Model.Legs[i];
			const FPlannerLeg& PL = In.Legs[i];
			FLeg&			   L = Legs[i];
			L.bFiveBar = ML.Kind == ERammsCurbLegKind::FiveBar;
			L.Axle = ML.Axle;
			L.Y = ML.Y;
			L.Radius = ML.RadiusCm;
			L.Contact = ML.ContactWidthCm;
			L.Tyre = PL.TyreWidthCm;
			L.Preload = PL.PreloadCm;
			L.RollLocal = ML.RollDirection.GetSafeNormal();
			L.RollWorld = (Heading * L.RollLocal.X + Right * L.RollLocal.Y).GetSafeNormal();
			if (L.bFiveBar)
			{
				L.GridX0 = ColumnFromCm;
				L.GridDx = ColumnStepCm;
				L.Columns.SetNum(NumColumns);
				for (int32 c = 0; c < NumColumns; ++c)
				{
					L.Columns[c] = PL.Grid.IsValid() ? PL.Grid.HeightRangeAt(ColumnFromCm + c * ColumnStepCm) : FVector2D(1.0, 0.0);
				}
			}
			else
			{
				L.Table = &ML.FourBar;
				L.TableRange = ML.FourBar.HeightRange();
			}
			for (int32 Far = 0; Far < 2; ++Far)
			{
				L.LevelOffset[Far] = ContactOffset(i, Level, Far != 0);
			}
		}
	}

	FPlanModel::FAxes FPlanModel::Axes(double PitchDeg, double RollDeg) const
	{
		if (PitchDeg == 0.0 && RollDeg == 0.0)
		{
			return Level;
		}
		const FChassisFrame F = ChassisFrameFrom(FVector(Heading.X, Heading.Y, 0.0), FMath::DegreesToRadians(PitchDeg),
			FMath::DegreesToRadians(RollDeg), FVector::ZeroVector, FVector::ZeroVector);
		FAxes				A;
		A.X = F.X;
		A.Y = F.Y;
		A.Z = F.Z;
		return A;
	}

	FVector FPlanModel::ContactOffset(int32 Leg, const FAxes& A, bool bFar) const
	{
		const FLeg&		 L = Legs[Leg];
		const FVector2D& F = L.RollLocal;
		// The axle square to the roll direction in the chassis, tilted with it.
		const FVector Axle = A.X * (-F.Y) + A.Y * F.X;
		return WheelContact(FVector::ZeroVector, Axle, (bFar ? Terrain.Far : Terrain.Near).Gradient, L.Radius, L.Contact);
	}

	double FPlanModel::DistancePast(int32 Leg, double S, double X) const
	{
		return Terrain.DistancePastEdge(WheelXY(Leg, S, X));
	}

	double FPlanModel::TravelAt(int32 Leg, double D, double X) const
	{
		return (D - DistancePast(Leg, 0.0, X)) / CosApproach;
	}

	FVector2D FPlanModel::WheelXY(int32 Leg, double S, double X) const
	{
		return OriginAt(S) + Heading * X + Right * Legs[Leg].Y;
	}

	double FPlanModel::ClearanceZ(int32 Leg, const FVector2D& XY) const
	{
		const FLeg& L = Legs[Leg];
		return Tuning.SwingClearance + ClearanceHeight(Terrain, XY, L.RollWorld, L.Radius, L.Tyre, Dilation);
	}

	double FPlanModel::SupportZ(int32 Leg, const FVector2D& XY, bool bFar) const
	{
		const FLeg&			   L = Legs[Leg];
		const FVector&		   Off = L.LevelOffset[bFar ? 1 : 0];
		const FRammsCurbPlane& P = bFar ? Terrain.Far : Terrain.Near;
		return P.HeightAt(XY + XYOf(Off)) - Off.Z - L.Preload;
	}

	double FPlanModel::ContactCentreZ(int32 Leg, const FVector2D& XY, bool bFar, double PitchDeg, double RollDeg) const
	{
		const FLeg&			   L = Legs[Leg];
		const FVector		   Off = (PitchDeg == 0.0 && RollDeg == 0.0) ? L.LevelOffset[bFar ? 1 : 0] : ContactOffset(Leg, Axes(PitchDeg, RollDeg), bFar);
		const FRammsCurbPlane& P = bFar ? Terrain.Far : Terrain.Near;
		return P.HeightAt(XY + XYOf(Off)) - Off.Z;
	}

	double FPlanModel::CornerX(int32 Leg, double Z) const
	{
		const FLeg& L = Legs[Leg];
		return L.Table ? L.Table->XAt(Z) : 0.0;
	}

	double FPlanModel::CornerCrank(int32 Leg, double Z) const
	{
		const FLeg& L = Legs[Leg];
		return L.Table ? L.Table->CrankAt(Z) : 0.0;
	}

	double FPlanModel::LegZOnSurface(int32 Leg, double S, double X, double B, double PitchDeg, double RollDeg, bool bFar) const
	{
		const FLeg&			   L = Legs[Leg];
		const FAxes			   A = Axes(PitchDeg, RollDeg);
		const FVector2D		   XY0 = OriginAt(S) + XYOf(A.X * X + A.Y * L.Y);
		const FVector		   Off = (A.Z.Z == 1.0) ? L.LevelOffset[bFar ? 1 : 0] : ContactOffset(Leg, A, bFar);
		const FRammsCurbPlane& P = bFar ? Terrain.Far : Terrain.Near;
		const double		   K = P.HeightAt(XY0 + XYOf(Off)) - L.Preload - A.X.Z * X - A.Y.Z * L.Y - Off.Z;
		const double		   Dd = A.Z.Z - (P.Gradient.X * A.Z.X + P.Gradient.Y * A.Z.Y);
		return (K - B) / Dd;
	}

	double FPlanModel::LegZForWorldZ(int32 Leg, double X, double WorldZ, double B, double PitchDeg, double RollDeg) const
	{
		const FAxes A = Axes(PitchDeg, RollDeg);
		return (WorldZ - B - A.X.Z * X - A.Y.Z * Legs[Leg].Y) / A.Z.Z;
	}

	FVector FPlanModel::WheelCentre(int32 Leg, double S, double X, double Z, double B, double PitchDeg, double RollDeg) const
	{
		const FAxes		A = Axes(PitchDeg, RollDeg);
		const FVector2D O = OriginAt(S);
		return FVector(O.X, O.Y, B) + A.X * X + A.Y * Legs[Leg].Y + A.Z * Z;
	}

	double FPlanModel::PoseMarginAt(TConstArrayView<FVector2D> LegXZ, uint32 Support, uint32 OnFar, double S, double B, double PitchDeg,
		double RollDeg, bool bDynamic) const
	{
		const FAxes										 A = Axes(PitchDeg, RollDeg);
		const FVector2D									 O = OriginAt(S);
		const FVector									 O3(O.X, O.Y, B);
		TArray<FVector2D, TInlineAllocator<MaxPlanLegs>> XZ;
		TArray<FVector2D, TInlineAllocator<MaxPlanLegs>> Contacts;
		double											 Ground = 0.0;
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			const FVector2D P = LegXZ.IsValidIndex(i) ? LegXZ[i] : FVector2D::ZeroVector;
			XZ.Add(P);
			if ((Support >> i) & 1u)
			{
				const bool	  bFar = (OnFar >> i) & 1u;
				const FVector Off = (A.Z.Z == 1.0) ? Legs[i].LevelOffset[bFar ? 1 : 0] : ContactOffset(i, A, bFar);
				const FVector C = O3 + A.X * P.X + A.Y * Legs[i].Y + A.Z * P.Y + Off;
				Contacts.Add(XYOf(C));
				Ground += C.Z;
			}
		}
		FVector2D SeatPivot;
		double	  SeatPitch = 0.0;
		SeatAt(PitchDeg, FreeSeatX, Support, OnFar, 0u, XZ, Contacts, Ground, A, O3, SeatPivot, SeatPitch);
		const FVector Com = Model.Seat.bHas ? Model.CenterOfMassAt(XZ, SeatPivot, SeatPitch) : Model.CenterOfMassAt(XZ);
		const FVector W = O3 + A.X * Com.X + A.Y * Com.Y + A.Z * Com.Z;
		if (Contacts.Num() < 3)
		{
			return Contacts.Num() == 0 ? -1e9 : HullMargin(Contacts, XYOf(W));
		}
		if (!bDynamic)
		{
			return HullMargin(Contacts, XYOf(W));
		}
		return DynamicMargin(Contacts, XYOf(W), W.Z - Ground / Contacts.Num(), Heading, Tuning.Acceleration, Tuning.Deceleration);
	}

	double FPlanModel::MarginFor(const FVector& Com, TConstArrayView<FVector2D> Contacts, double Ground, const FAxes& A, const FVector& O3) const
	{
		const FVector W = O3 + A.X * Com.X + A.Y * Com.Y + A.Z * Com.Z;
		if (Contacts.Num() < 3)
		{
			return Contacts.Num() == 0 ? -1e9 : HullMargin(Contacts, XYOf(W));
		}
		return DynamicMargin(Contacts, XYOf(W), W.Z - Ground / Contacts.Num(), Heading, Tuning.Acceleration, Tuning.Deceleration);
	}

	void FPlanModel::SeatAt(double PitchDeg, double SeatX, uint32 Support, uint32 OnFar, uint32 Touching, TConstArrayView<FVector2D> LegXZ,
		TConstArrayView<FVector2D> Contacts, double Ground, const FAxes& A, const FVector& O3, FVector2D& OutPivot, double& OutPitchRad,
		FVector2D* OutBand) const
	{
		if (OutBand)
		{
			*OutBand = FVector2D(1.0, 0.0);
		}
		const FModelSeat& Seat = Model.Seat;
		OutPivot = Seat.ZeroPivot;
		OutPitchRad = 0.0;
		if (!Seat.bHas)
		{
			return;
		}
		// The rider kept level against the base's pitch, as far as the platform
		// turns.
		if (Seat.bTilts)
		{
			OutPitchRad = FMath::Clamp(-FMath::DegreesToRadians(PitchDeg), -Seat.MaxPitchRad, Seat.MaxPitchRad);
		}
		if (!Seat.Shifts())
		{
			return;
		}
		const FVector2D& R = Seat.XRange;
		OutPivot.Y = Seat.HeightCm;
		if (!IsSeatFree(SeatX))
		{
			OutPivot.X = FMath::Clamp(SeatX, R.X, R.Y);
			return;
		}
		// Free: the fore/aft, every SeatStepCm across the reach, that keeps the
		// pose best -- its margin counts up to SeatEnoughCm over the requirement,
		// no further -- and nearest home among equals. Home follows the
		// crossing: until a wheel stands on the far
		// side, the aft end of the reach, the rider's mass carried over the
		// rear four wheels while the fronts go over; once some wheel stands
		// across with others still to come, the forward end, the mass carried
		// over the far contacts as soon as there are any, so that the lifts
		// still to come begin with margin to spare rather than wait for it;
		// with every wheel across, the aft end again: the stance the robot
		// drives in outdoors carries the seat back and
		// raised, the mass over the rear four, and the drive mode handed back
		// to lifts the front corners -- with the mass still forward that
		// hand-back tipped the robot onto them. (Descending the same: the mass
		// over the wheels still on the high side while the fronts go down,
		// over the fronts once they stand below.) A wheel landing but not yet
		// loaded (Touching) is not across: the seat stays forward for the last
		// axle until it carries, not sent back as its touchdown is planned.
		const uint32 Legs6 = Legs.Num() >= 32 ? ~0u : (1u << Legs.Num()) - 1u;
		const uint32 Across = Support & OnFar & ~Touching;
		const bool	 bSomeAcross = Across != 0 && (Legs6 & ~Across) != 0;
		const double Home = bSomeAcross ? R.Y : R.X;
		const double Enough = Tuning.MinStabilityMargin + SeatEnoughCm;
		double		 BestX = Home, BestScore = -1e18;
		// Every fore/aft tried and its margin, for the band.
		TArray<FVector2D, TInlineAllocator<64>> Tried;
		auto									Try = [&](double X) {
			   const double Margin = MarginFor(Model.CenterOfMassAt(LegXZ, FVector2D(X, OutPivot.Y), OutPitchRad), Contacts, Ground, A, O3);
			   const double Score = FMath::Min(Margin, Enough);
			   if (OutBand)
			   {
				   Tried.Add(FVector2D(X, Margin));
			   }
			   if (Score > BestScore + 1e-9 || (FMath::Abs(Score - BestScore) <= 1e-9 && FMath::Abs(X - Home) < FMath::Abs(BestX - Home)))
			   {
				   BestScore = Score;
				   BestX = X;
			   }
		};
		Try(Home);
		for (double X = Home + SeatStepCm; X <= R.Y + 1e-9; X += SeatStepCm)
		{
			Try(X);
		}
		for (double X = Home - SeatStepCm; X >= R.X - 1e-9; X -= SeatStepCm)
		{
			Try(X);
		}
		OutPivot.X = BestX;
		if (OutBand)
		{
			// The run of fore/afts about the choice that keep the requirement.
			Tried.Sort([](const FVector2D& L, const FVector2D& Rt) { return L.X < Rt.X; });
			const int32	 At = Tried.IndexOfByPredicate([BestX](const FVector2D& E) { return FMath::Abs(E.X - BestX) < 1e-9; });
			const double Need = Tuning.MinStabilityMargin - 1e-9;
			if (At != INDEX_NONE && Tried[At].Y >= Need)
			{
				int32 Lo = At, Hi = At;
				while (Lo > 0 && Tried[Lo - 1].Y >= Need)
				{
					--Lo;
				}
				while (Hi < Tried.Num() - 1 && Tried[Hi + 1].Y >= Need)
				{
					++Hi;
				}
				*OutBand = FVector2D(Tried[Lo].X, Tried[Hi].X);
			}
		}
	}

	FVector2D FPlanModel::Reach(int32 Leg, double X) const
	{
		const FLeg& L = Legs[Leg];
		if (!L.bFiveBar)
		{
			return L.TableRange;
		}
		const int32 C = FMath::RoundToInt((X - L.GridX0) / L.GridDx);
		return L.Columns.IsValidIndex(C) ? L.Columns[C] : FVector2D(1.0, 0.0);
	}

	double FPlanModel::Belly(const FAxes& A, const FVector2D& O) const
	{
		const FVector2D H = Model.BellyHalfExtent;
		const double	U = Model.BellyUndersideCm;
		const FVector2D Corners[4] = { { H.X, H.Y }, { -H.X, H.Y }, { -H.X, -H.Y }, { H.X, -H.Y } };
		FVector2D		XY[4];
		double			Dz[4];
		double			D[4];
		for (int32 c = 0; c < 4; ++c)
		{
			const FVector P = A.X * Corners[c].X + A.Y * Corners[c].Y + A.Z * U;
			XY[c] = O + XYOf(P);
			Dz[c] = P.Z;
			D[c] = Terrain.DistancePastEdge(XY[c]);
		}
		double Floor = -TNumericLimits<double>::Max();
		auto   Need = [&](const FVector2D& P, double OffsetZ, double Surface) {
			  Floor = FMath::Max(Floor, Surface + Tuning.ChassisClearance - OffsetZ);
		};
		for (int32 c = 0; c < 4; ++c)
		{
			Need(XY[c], Dz[c], D[c] >= 0.0 ? Terrain.Far.HeightAt(XY[c]) : Terrain.Near.HeightAt(XY[c]));
			const int32 n = (c + 1) % 4;
			if ((D[c] < 0.0) != (D[n] < 0.0))
			{
				// Where this side of the plate crosses the edge: under it, both
				// surfaces meet (the higher one counts).
				const double	T = D[c] / (D[c] - D[n]);
				const FVector2D P = XY[c] + (XY[n] - XY[c]) * T;
				const double	Oz = Dz[c] + (Dz[n] - Dz[c]) * T;
				Need(P, Oz, FMath::Max(Terrain.Near.HeightAt(P), Terrain.Far.HeightAt(P)));
			}
		}
		return Floor;
	}

	FInstantResult FPlanModel::Evaluate(const FInstantQuery& Q, double BGuess, bool bAtGuess) const
	{
		FInstantResult	R;
		const FAxes		A = Axes(Q.PitchDeg, Q.RollDeg);
		const FVector2D O = OriginAt(Q.S);
		const double	M = Tuning.LegRangeMargin;
		R.Lo = Belly(A, O);
		R.LoLeg = -1;
		R.Hi = TNumericLimits<double>::Max();
		R.HiLeg = -2;
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			const FLeg& L = Legs[i];
			// A leg touching down stands on its far surface as a support leg does,
			// though it carries nothing yet.
			const bool bTouch = (Q.Touching >> i) & 1u;
			const bool bUp = ((Q.Airborne >> i) & 1u) && !bTouch;
			const bool bFar = ((Q.OnFar >> i) & 1u) || bTouch;
			double	   X = L.bFiveBar ? Q.X[i] : CornerX(i, 0.5 * (L.TableRange.X + L.TableRange.Y));
			double	   Z = 0.5 * (L.TableRange.X + L.TableRange.Y);
			double	   Lo = -TNumericLimits<double>::Max(), Hi = TNumericLimits<double>::Max();
			for (int32 Pass = 0; Pass < 3; ++Pass)
			{
				const FVector2D Reach2 = Reach(i, X);
				if (Reach2.X > Reach2.Y)
				{
					Lo = TNumericLimits<double>::Max();
					break;
				}
				const double	ZLo = Reach2.X + M, ZHi = Reach2.Y - M;
				const FVector2D XY0 = O + XYOf(A.X * X + A.Y * L.Y);
				if (!bUp)
				{
					const FVector&		   Off = (A.Z.Z == 1.0) ? L.LevelOffset[bFar ? 1 : 0] : ContactOffset(i, A, bFar);
					const FRammsCurbPlane& P = bFar ? Terrain.Far : Terrain.Near;
					const double		   K = P.HeightAt(XY0 + XYOf(Off)) - L.Preload - A.X.Z * X - A.Y.Z * L.Y - Off.Z;
					const double		   Dd = A.Z.Z - (P.Gradient.X * A.Z.X + P.Gradient.Y * A.Z.Y);
					Lo = K - Dd * ZHi;
					Hi = K - Dd * ZLo;
					Z = (K - BGuess) / Dd;
				}
				else
				{
					const double Need = ClearanceZ(i, XY0 + XYOf(A.Z * Z));
					Lo = Need - A.X.Z * X - A.Y.Z * L.Y - A.Z.Z * ZHi;
					Hi = TNumericLimits<double>::Max();
					Z = FMath::Max((Need - BGuess - A.X.Z * X - A.Y.Z * L.Y) / A.Z.Z, ZLo);
				}
				if (L.bFiveBar)
				{
					break; // its fore/aft is given
				}
				X = CornerX(i, FMath::Clamp(Z, L.TableRange.X, L.TableRange.Y));
			}
			if (Lo > R.Lo)
			{
				R.Lo = Lo;
				R.LoLeg = i;
			}
			if (Hi < R.Hi)
			{
				R.Hi = Hi;
				R.HiLeg = i;
			}
		}
		const double B = bAtGuess || R.Lo > R.Hi ? BGuess : 0.5 * (R.Lo + FMath::Min(R.Hi, R.Lo + 60.0));
		PoseAt(Q, B, R);
		return R;
	}

	void FPlanModel::PoseAt(const FInstantQuery& Q, double B, FInstantResult& R) const
	{
		const FAxes		A = Axes(Q.PitchDeg, Q.RollDeg);
		const FVector2D O = OriginAt(Q.S);
		const FVector	O3(O.X, O.Y, B);
		const double	M = Tuning.LegRangeMargin;
		R.B = B;
		R.bReach = true;
		TArray<FVector2D, TInlineAllocator<MaxPlanLegs>> XZ;
		TArray<FVector2D, TInlineAllocator<MaxPlanLegs>> Contacts;
		double											 Ground = 0.0;
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			const FLeg& L = Legs[i];
			const bool	bTouch = (Q.Touching >> i) & 1u;
			const bool	bUp = ((Q.Airborne >> i) & 1u) && !bTouch;
			const bool	bFar = ((Q.OnFar >> i) & 1u) || bTouch;
			double		X = L.bFiveBar ? Q.X[i] : CornerX(i, 0.5 * (L.TableRange.X + L.TableRange.Y));
			double		Z = 0.0;
			bool		bOk = true;
			FVector		Off = FVector::ZeroVector;
			for (int32 Pass = 0; Pass < 3; ++Pass)
			{
				const FVector2D Reach2 = Reach(i, X);
				const double	ZLo = Reach2.X + M, ZHi = Reach2.Y - M;
				const FVector2D XY0 = O + XYOf(A.X * X + A.Y * L.Y);
				if (!bUp)
				{
					Off = (A.Z.Z == 1.0) ? L.LevelOffset[bFar ? 1 : 0] : ContactOffset(i, A, bFar);
					const FRammsCurbPlane& P = bFar ? Terrain.Far : Terrain.Near;
					const double		   K = P.HeightAt(XY0 + XYOf(Off)) - L.Preload - A.X.Z * X - A.Y.Z * L.Y - Off.Z;
					const double		   Dd = A.Z.Z - (P.Gradient.X * A.Z.X + P.Gradient.Y * A.Z.Y);
					Z = (K - B) / Dd;
					bOk = Reach2.X <= Reach2.Y && Z >= ZLo - 1e-9 && Z <= ZHi + 1e-9;
				}
				else
				{
					const double Need = ClearanceZ(i, XY0 + XYOf(A.Z * Z));
					const double Want = (Need - B - A.X.Z * X - A.Y.Z * L.Y) / A.Z.Z;
					bOk = Reach2.X <= Reach2.Y && Want <= ZHi + 1e-9;
					Z = FMath::Max(Want, ZLo);
				}
				if (L.bFiveBar)
				{
					break;
				}
				X = CornerX(i, FMath::Clamp(Z, L.TableRange.X, L.TableRange.Y));
			}
			R.bReach = R.bReach && bOk;
			R.LegX[i] = X;
			R.LegZ[i] = Z;
			XZ.Add(FVector2D(X, Z));
			if (!bUp && !bTouch) // one touching down carries nothing yet
			{
				const FVector C = O3 + A.X * X + A.Y * L.Y + A.Z * Z + Off;
				Contacts.Add(XYOf(C));
				Ground += C.Z;
			}
		}
		R.Support = Contacts.Num();
		FVector2D SeatPivot;
		double	  SeatPitch = 0.0;
		SeatAt(Q.PitchDeg, Q.SeatX, ~Q.Airborne | Q.Touching, Q.OnFar | Q.Touching, Q.Touching, XZ, Contacts, Ground, A, O3, SeatPivot, SeatPitch,
			&R.SeatBand);
		R.SeatX = SeatPivot.X;
		R.SeatPitchRad = SeatPitch;
		const FVector Com = Model.Seat.bHas ? Model.CenterOfMassAt(XZ, SeatPivot, SeatPitch) : Model.CenterOfMassAt(XZ);
		const FVector W = O3 + A.X * Com.X + A.Y * Com.Y + A.Z * Com.Z;
		if (Contacts.Num() < 3)
		{
			R.Margin = R.StaticMargin = Contacts.Num() == 0 ? -1e9 : HullMargin(Contacts, XYOf(W));
			return;
		}
		const double H = W.Z - Ground / Contacts.Num();
		R.StaticMargin = HullMargin(Contacts, XYOf(W));
		R.Margin = DynamicMargin(Contacts, XYOf(W), H, Heading, Tuning.Acceleration, Tuning.Deceleration);
	}

	// --- the planner -------------------------------------------------------------------------

	namespace
	{
		/** The search checks every this many samples; the plan is verified at
		 *  every one. */
		constexpr int32 SearchEvery = 2;
		/** Rising early, the chassis keeps this much (cm) under a corridor's top
		 *  where the corridor allows. */
		constexpr double RiseSlackCm = 2.0;
		/** The grid search over the centre wheels' paths: stages this far apart
		 *  (cm), fore/aft steps (cm), and the stretch of the chassis (cm, x)
		 *  it lets them stand in -- with where they began always on the grid. */
		constexpr double DpStageCm = 4.0;
		constexpr double DpStepCm = 2.0;
		/** The search over both centre wheels' pairs keeps each within this
		 *  (cm) of where it stands as the stretch begins, and spans this much
		 *  travel (cm) either side of a stretch that fails. */
		constexpr double JointSpanCm = 16.0;
		/** The search over one wheel's path keeps it within this (cm) of where
		 *  it began (and its reach). */
		constexpr double CentreSpanCm = 20.0;
		constexpr double JointPadCm = 24.0;
		/** The slowest speed (cm/s) taken for timing between samples; a planned
		 *  stop's dwell (its re-pose, at rest) is timed apart. */
		constexpr double MinTimingSpeed = 0.5;
		/** A planned stop's re-pose is checked at this many poses between where
		 *  the robot arrives and where it leaves. */
		constexpr int32 DwellPoseSteps = 8;
		/** The level-in's poses are checked at this many points along it. */
		constexpr int32 LevelInPoseSteps = 4;
		/** The servo lag model's step (s). */
		constexpr double SimDt = 0.01;
		/** A lifted wheel need only be within this of its clearance profile (cm):
		 *  the profile keeps SwingClearance, and a first-order lag never quite
		 *  arrives. */
		constexpr double ProfileToleranceCm = 0.25;
		/** A swing leg's command looks this far ahead (s) along its profile. */
		constexpr double SwingLookSeconds = 0.3;
		/** The margin is looked for at this many chassis heights across a
		 *  corridor when its middle has too little. */
		constexpr int32 MarginProbes = 6;
		/** A centre wheel holding a knot holds it this far (cm) beyond the
		 *  stretch that needs it. */
		constexpr double EventPadCm = 1.0;
		/** An early walk home (Ramms.Curb.WalkEarly) is made over this much
		 *  travel (cm) at most after the last planned position. */
		constexpr double WalkHomeCm = 30.0;
		/** An eased lift (LiftEaseSeconds) rises no slower than this (cm/s). */
		constexpr double MinEasedLiftSpeed = 8.0;
		/** ...and begins no nearer the level-in's end than this (cm of travel):
		 *  the executor brakes the level-in short of the first lift. */
		constexpr double EaseLevelInPadCm = 10.0;

		/** A centre wheel's path (samples SampleCm apart) averaged over +-HalfCm.
		 *  Where the path keeps to a bound, a rate or a hold over a whole window,
		 *  so does the average there. Toward its ends the window narrows evenly,
		 *  so the first and last samples stay where they are. */
		TArray<double> SmoothCentrePath(const TArray<double>& Path, double HalfCm, double SampleCm)
		{
			const int32 Half = FMath::RoundToInt(HalfCm / FMath::Max(1e-3, SampleCm));
			if (Half <= 0 || Path.Num() == 0)
			{
				return Path;
			}
			const int32	   N = Path.Num();
			TArray<double> Sum, Out;
			Sum.SetNumUninitialized(N + 1);
			Sum[0] = 0.0;
			for (int32 k = 0; k < N; ++k)
			{
				Sum[k + 1] = Sum[k] + Path[k];
			}
			Out.SetNumUninitialized(N);
			for (int32 k = 0; k < N; ++k)
			{
				const int32 H = FMath::Min(Half, FMath::Min(k, N - 1 - k));
				Out[k] = (Sum[k + H + 1] - Sum[k - H]) / (2 * H + 1);
			}
			return Out;
		}
		/** What may cap the speed (FPlan::CapBy), past the legs' own indices. */
		constexpr int32 CapLevelIn = 100;
		constexpr int32 CapWalk = 101;
		constexpr int32 CapTilt = 102;
		constexpr int32 CapSeat = 103;
		constexpr int32 CapFore = 104;
		constexpr int32 CapAfterLevelIn = 200;
		/** Rounds of the planned seat and the drive under its forward shifts
		 *  (PlanSeat), each timing the crossing at the last round's drive. */
		constexpr int32 SeatDriveRounds = 4;
		/** The drive's speed under the seat's shift, cm/s: rounds that change it
		 *  by no more than this anywhere have settled. */
		constexpr double SeatDriveSettledCmS = 0.5;
		/** Rounds of the speed envelope (each may add caps). */
		constexpr int32 MaxSpeedRounds = 24;
		/** A tilted plan's easings are made at this share of TiltRateDegPerSec
		 *  where its speeds give them the time (and the plan still verifies),
		 *  else at the whole rate. */
		constexpr double GentleTiltShare = 2.0 / 3.0;
		/** A tilted plan whose samples fail at its own speeds (but stand still)
		 *  is made again slower over them -- this share of its speed there, this
		 *  far (cm) either side, never under the floor -- at most this many times
		 *  before its tilt is made larger (TiltPlan). */
		constexpr double SlowTiltShare = 0.7;
		constexpr double SlowTiltPadCm = 20.0;
		constexpr int32	 MaxSlowTiltRounds = 3;
		constexpr int32	 MaxSlowTiltAttempts = 12;
		/** The slowed tilt search looks only for a tilt at least this much (deg of
		 *  inclination) less than the one found at the plans' speeds: a degree is
		 *  worth the floor speed the slowing costs; a fraction of one is not. */
		constexpr double SlowTiltGainDeg = 1.0;
		constexpr int32	 SlowTiltLargerTries = 4;
		/** Slowed where the search at the plans' speeds found no tilt within the
		 *  limit: the candidates' least tilts, and this many larger tries; its
		 *  work at most this share of the planner's before it (OverSlowWork). */
		constexpr int32	 SlowTiltWhenNoneLargerTries = 0;
		constexpr double SlowTiltWhenNoneWorkShare = 0.5;
		/** Up to this many planned stops. */
		constexpr int32 MaxStops = 4;
		/** Each fallback (tilt; tilt and stops; stops) is planned on at most this
		 *  many of the level search's candidates (those it suits best first). */
		constexpr int32 MaxFallbackCandidates = 3;
		/** Where no candidate's least tilt verifies at the plan's speeds, the
		 *  stretches a candidate's plan failed about are tried this much (deg,
		 *  as its pick measures them) larger (up to MaxPlannedTiltDeg;
		 *  SearchTilts). */
		constexpr double TiltEscalationDeg[] = { 0.25, 0.5, 1.0 };
		/** Failing samples closer than this (cm) are one stretch for the
		 *  fallbacks. */
		constexpr double MergeFailCm = 12.0;
		/** Kept in hand between one wheel taking load and the next lifting (s). */
		constexpr double GapMarginSeconds = 0.1;
		/** A pose this far short of the margin at the corridor's middle is not
		 *  looked for elsewhere in it (cm): the corners' fore/aft and the centre
		 *  of mass move it much less. */
		constexpr double HopelessMarginCm = 8.0;
		/** Lift and landing times are kept per speed to this (cm/s). */
		constexpr double TimingSpeedStep = 0.5;
		/** No value: a path's free start, or a sample with no wish (hold). */
		constexpr double NoValue = 1e300;
		/** No cap (cm/s). */
		constexpr double NoCap = 1e9;
		/** Rounds of slowing for the seat's shift (PlanSeat). */
		constexpr int32 SeatPathRounds = 6;

		enum class EFail : uint8
		{
			None,
			/** No pose even standing still, with these centre-wheel paths. */
			Static,
			/** Only below the floor speed. */
			Slow,
			/** The chassis height cannot follow its corridor. */
			Height,
			/** The verification found something. */
			Verify,
		};

		/** Bands and asymmetric rates: the path through every band, rising no
		 *  more than Up[k] and falling no more than Down[k] from sample k - 1 to
		 *  k, as near Wish (NoValue: hold) as it may. Start, unless NoValue, is
		 *  where it must begin. */
		bool RatePath(TConstArrayView<FVector2D> Bands, TConstArrayView<double> Up, TConstArrayView<double> Down, double Start,
			TConstArrayView<double> Wish, TArray<double>& Out, int32& OutStuck)
		{
			const int32 N = Bands.Num();
			OutStuck = INDEX_NONE;
			Out.SetNum(N);
			if (N == 0)
			{
				return false;
			}
			TArray<FVector2D> Tube(Bands.GetData(), N);
			if (Start < NoValue)
			{
				Tube[0].X = FMath::Max(Tube[0].X, Start);
				Tube[0].Y = FMath::Min(Tube[0].Y, Start);
			}
			for (int32 k = 0; k < N; ++k)
			{
				if (k > 0)
				{
					Tube[k].X = FMath::Max(Tube[k].X, Tube[k - 1].X - Down[k]);
					Tube[k].Y = FMath::Min(Tube[k].Y, Tube[k - 1].Y + Up[k]);
				}
				if (Tube[k].X > Tube[k].Y + 1e-9)
				{
					OutStuck = k;
					return false;
				}
			}
			for (int32 k = N - 2; k >= 0; --k)
			{
				Tube[k].X = FMath::Max(Tube[k].X, Tube[k + 1].X - Up[k + 1]);
				Tube[k].Y = FMath::Min(Tube[k].Y, Tube[k + 1].Y + Down[k + 1]);
				if (Tube[k].X > Tube[k].Y + 1e-9)
				{
					OutStuck = k;
					return false;
				}
			}
			for (int32 k = 0; k < N; ++k)
			{
				double Lo = Tube[k].X, Hi = FMath::Max(Tube[k].X, Tube[k].Y);
				if (k > 0)
				{
					Lo = FMath::Max(Lo, Out[k - 1] - Down[k]);
					Hi = FMath::Min(Hi, Out[k - 1] + Up[k]);
				}
				const double Want = Wish.IsValidIndex(k) && Wish[k] < NoValue ? Wish[k] : (k > 0 ? Out[k - 1] : 0.5 * (Lo + Hi));
				Out[k] = FMath::Clamp(Want, Lo, FMath::Max(Lo, Hi));
			}
			return true;
		}

		/** A trapezoid tracker: a command at C moving at Cv toward Target, at
		 *  most Speed, changing its speed by at most Accel. One step of Dt. */
		void TrackStep(double& C, double& Cv, double Target, double Speed, double Accel, double Dt)
		{
			const double Gap = Target - C;
			const double Want = FMath::Sign(Gap) * FMath::Min(Speed, FMath::Sqrt(2.0 * Accel * FMath::Abs(Gap)));
			Cv += FMath::Clamp(Want - Cv, -Accel * Dt, Accel * Dt);
			C += Cv * Dt;
			if ((Target - C) * Gap < 0.0 && FMath::Abs(Cv) <= Accel * Dt * 1.01)
			{
				C = Target;
				Cv = 0.0;
			}
		}

		/** A lifted wheel's clearance profile along the travel: the world height
		 *  its centre needs at each sample from K0 on. */
		struct FProfile
		{
			int32		   K0 = 0;
			TArray<double> Z;

			double At(double S, double Ds) const
			{
				if (Z.Num() == 0)
				{
					return 0.0;
				}
				const double F = S / Ds - K0;
				const int32	 I = FMath::Clamp(FMath::FloorToInt(F), 0, Z.Num() - 1);
				const int32	 J = FMath::Min(I + 1, Z.Num() - 1);
				return FMath::Lerp(Z[I], Z[J], FMath::Clamp(F - I, 0.0, 1.0));
			}

			double MaxOver(double SA, double SB, double Ds) const
			{
				double Best = FMath::Max(At(SA, Ds), At(SB, Ds));
				for (int32 k = FMath::CeilToInt(SA / Ds) - K0; k <= FMath::FloorToInt(SB / Ds) - K0; ++k)
				{
					if (Z.IsValidIndex(k))
					{
						Best = FMath::Max(Best, Z[k]);
					}
				}
				return Best;
			}
		};

		/** What a leg's crossing is, given the centre wheels' paths and the
		 *  chassis heights: where its window begins and ends, its clearance
		 *  profile, its planted and landed heights there, and its lift and
		 *  landing times by speed (filled as asked). */
		struct FLegGeom
		{
			int32	 KUp = 0;
			int32	 KDown = 0;
			FProfile Prof;
			double	 ZPlant = 0.0;
			double	 ZLand = 0.0;
			double	 ZTop = 0.0;
			/** The profile is highest here before the landing (a descent's drop
			 *  begins over the lip). */
			double SDropFrom = 0.0;
			/** The leg's vertical speed (cm/s) for its lift and its landing:
			 *  LegSpeed, or less where its crank's rate limit
			 *  (FPlannerLeg::MaxCrankRateRadS) would not turn it through the move
			 *  in the time LegSpeed would take. */
			double						SpeedUp = 0.0;
			double						SpeedDown = 0.0;
			mutable TMap<int32, double> Lift;
			mutable TMap<int32, double> Plant;
		};

		/** One crossing planned on two grids side by side (PlanCrossing): each
		 *  grid's plan once made -- its stage and its work, then bDone -- for
		 *  the other to read at its checks (FPlanner::Stopped). */
		struct FGridRace
		{
			std::atomic<bool> bDone[2];
			EPlanStage		  Stage[2] = { EPlanStage::Refused, EPlanStage::Refused };
			int64			  Work[2] = { 0, 0 };
			/** On the move: the work budget applies (OverGridBudget). */
			bool bBudget = false;

			FGridRace()
			{
				bDone[0].store(false);
				bDone[1].store(false);
			}

			void Publish(int32 Slot, const FCurbSchedule& Plan)
			{
				Stage[Slot] = Plan.IsValid() ? Plan.Stage : EPlanStage::Refused;
				Work[Slot] = Plan.PlanWork;
				bDone[Slot].store(true, std::memory_order_release);
			}
		};
	} // namespace

	class FPlanner
	{
	public:
		explicit FPlanner(const FPlannerInput& InInput, FGridRace* InRace = nullptr, int32 InSlot = 0)
			: In(InInput)
			, M(InInput)
			, T(InInput.Tuning)
			, Race(InRace)
			, Slot(InSlot)
		{
		}

		FCurbSchedule Run();

	private:
		/** One candidate plan, sample by sample. */
		struct FPlan
		{
			FString		   Name;
			TArray<double> X[2];
			TArray<double> Pitch;
			TArray<double> Roll;
			TArray<double> VCap;
			/** Caps the planned tilt's easings ask for (cm/s): they keep from one
			 *  speed envelope to the next. */
			TArray<double> TiltCap;
			/** What set each sample's cap: INDEX_NONE nothing, CapLevelIn,
			 *  CapWalk, CapTilt, CapSeat, CapFore, CapAfterLevelIn + leg, or the
			 *  leg whose lead or tail it parts from the others. */
			TArray<int32>  CapBy;
			TArray<double> V;
			/** The drive's speed under the planned seat's forward shifts (cm/s,
			 *  NoCap elsewhere; PlanSeat), kept from one speed envelope to the
			 *  next as SeatCap is: the executor slows the base by the seat's
			 *  speed (FModelSeat::ForeSlowDownCmS), and every lift and landing
			 *  about a shift is timed at that. */
			TArray<double> ForeCap;
			/** Where ForeCap slows the drive anywhere: the speed without it (the
			 *  rider's, near enough, where the seat shifts forward), from which
			 *  the floor is taken; else empty. */
			TArray<double> VNoFore;
			TArray<uint8>  Stop;
			TArray<double> B;
			TArray<double> BLo;
			TArray<double> BHi;
			TArray<double> Margin;
			/** The seat's pivot fore/aft the pose was verified with (cm). */
			TArray<double> SeatX;
			/** Where the seat can be at each sample, following SeatX no faster
			 *  than it shifts (FixSeat); empty, the seat is free at every
			 *  instant (FPlanModel::SeatAt chooses). Queries take it. */
			TArray<double> SeatFix;
			/** Where the seat may be at each sample (FInstantResult::SeatBand),
			 *  as Verify last found with the seat free. */
			TArray<FVector2D> SeatBand;
			/** Caps the seat's shift asks for (cm/s), kept from one speed
			 *  envelope to the next as TiltCap is (PlanSeat). */
			TArray<double> SeatCap;
			/** SeatFix is the plan's seat (PlanSeat), kept by EaseLifts; and the
			 *  last instant's own choice, where it goes once the crossing is over. */
			bool   bSeatPlanned = false;
			double SeatEndX = 0.0;
			/** Off the ground and on the far side, as planned at the planned speed;
			 *  and standing still. */
			TArray<uint32> Air;
			TArray<uint32> Far;
			/** Off the ground and touching down (FInstantQuery::Touching). */
			TArray<uint32>	   Touch;
			TArray<uint32>	   StaticAir;
			TArray<uint32>	   StaticFar;
			TArray<uint8>	   StaticOk;
			TArray<EFail>	   Fail;
			TArray<FWheelPlan> Wheels;
			TArray<FLegGeom>   Geom;
			/** Whether an instant stands, by everything it was asked with. */
			mutable TMap<uint64, bool> Memo;
			/** The chassis heights an instant allows with the margin, likewise. */
			mutable TMap<uint64, FVector2D> BandMemo;
			bool							bBuilt = false;
			bool							bHeightOk = false;
			bool							bVerified = false;
			double							VFloor = 0.0;
			int32							Stops = 0;
			int32							FirstFail = INDEX_NONE;
			FString							FailWhy;
			/** Why a sample failed verification or the chassis height path, by sample. */
			TMap<int32, FString> WhyAt;

			int32 CountFails() const
			{
				int32 C = 0;
				for (EFail F : Fail)
				{
					C += F != EFail::None;
				}
				return C;
			}
			/** Samples that do not stand even standing still: no speed or stop
			 *  mends those. */
			int32 CountStatic() const
			{
				int32 C = 0;
				for (EFail F : Fail)
				{
					C += F == EFail::Static;
				}
				return C;
			}
			/** Fewer standing failures, then fewer failures. */
			bool FailsLessThan(const FPlan& O) const
			{
				const int32 Mine = CountStatic(), Theirs = O.CountStatic();
				return Mine != Theirs ? Mine < Theirs : CountFails() < O.CountFails();
			}
		};

		bool	Setup(FString& OutWhy);
		int32	Index(double S) const { return FMath::Clamp(FMath::RoundToInt(S / Ds), 0, NumS - 1); }
		double	SAt(int32 K) const { return K * Ds; }
		double	CentreX0(int32 Side) const { return Centre[Side] != INDEX_NONE ? In.Legs[Centre[Side]].Measured.X : 0.0; }
		FString LegName(int32 Leg) const { return In.Legs.IsValidIndex(Leg) ? In.Legs[Leg].Name.ToString() : FString(TEXT("?")); }

		void   InitPlan(FPlan& P, const TCHAR* Name) const;
		void   ApplyLevelInCap(FPlan& P) const;
		void   Passes(FPlan& P) const;
		double LegXAt(int32 Leg, const FPlan& P, int32 K, bool bLanded) const;
		double ProfileZ(int32 Leg, double S, const FPlan& P) const;
		void   ComputeGeometry(FPlan& P) const;
		void   ComputeStatic(FPlan& P) const;
		void   ComputeTiming(FPlan& P) const;
		double LiftLead(const FLegGeom& G, double V) const;
		double PlantTime(int32 Leg, const FLegGeom& G, double V) const;
		double MarchBack(const FPlan& P, double S, double Seconds) const;
		/** LiftLead with the leg rising at Speed (cm/s); not cached. */
		double LiftLeadAt(const FLegGeom& G, double V, double Speed) const;
		/** The slowest rise (cm/s, MinEasedLiftSpeed at least) that still
		 *  clears the window with the lift begun Lead s before it at speed V. */
		double SlowestLiftSpeed(const FLegGeom& G, double V, double Lead) const;
		/** Which wheels are off the ground, on the far side and touching down at
		 *  each sample, from the wheels' events (SLiftStart .. SLoaded). */
		void ComputeAirMasks(FPlan& P) const;
		/** Seconds from sample KA to sample KB at P's speeds, dwells included. */
		double SecondsBetween(const FPlan& P, int32 KA, int32 KB) const;
		/** Every lift begun as early as the robot stands with the wheel up (up
		 *  to LiftEaseSeconds earlier), its leg rising slower; a corner carried
		 *  up in the drive stance whose eased lift reaches the level-in stays
		 *  up. True when any lift was eased (the air masks rebuilt). */
		bool EaseLifts(FPlan& P) const;
		bool FixSeat(FPlan& P) const;
		/** The seat as it follows the instants' choices (FixSeat), the plan
		 *  slowed where a pose needs it further on (SeatCap) and verified with
		 *  it -- with the drive under its forward shifts (ForeCap, SeatDriveOn)
		 *  where that verifies, else without. False (P to be dropped) where
		 *  neither does. */
		bool PlanSeat(FPlan& P) const;
		/** PlanSeat, with the drive under the seat's forward shifts (bDrive) or
		 *  without. */
		bool PlanSeatPass(FPlan& P, bool bDrive) const;
		/** The seat from X0 at sample K0 as the executor moves it: toward each
		 *  sample's choice within Lo .. Hi at up to ShiftSpeedCmS, eased by
		 *  ShiftAccelCmS2, the drive slowed under its forward shifts as the
		 *  executor slows it (into OutFore, NoCap elsewhere). */
		void		  DriveSeatPath(const FPlan& P, int32 K0, double X0, TConstArrayView<double> Lo, TConstArrayView<double> Hi, TArray<double>& OutPath,
					 TArray<double>& OutFore) const;
		double		  MarchFwd(const FPlan& P, double S, double Seconds) const;
		FInstantQuery Query(const FPlan& P, int32 K, uint32 Air, uint32 Far) const;
		bool		  Feasible(const FInstantQuery& Q, double BGuess, FInstantResult* Out = nullptr) const;
		/** Feasible, remembered in the plan by everything it is asked with. */
		bool   Stands(const FPlan& P, const FInstantQuery& Q, double BGuess) const;
		uint64 InstantKey(const FInstantQuery& Q, double BGuess) const;
		double BGuessAt(const FPlan& P, int32 K) const { return P.B.IsValidIndex(K) ? P.B[K] : B0; }
		void   SpeedEnvelope(FPlan& P) const;
		/** bPerSide: each centre wheel's forward and aft stretches are its own
		 *  side's front and rear corner's time off the ground (a curb met at an
		 *  angle: one side's corners go first), not where both corners' overlap. */
		bool CentrePathsKnots(FPlan& P, double XF, double XLift, double XLand, double XR, bool bSmooth, bool bPerSide = false) const;
		void CentrePathsDp(FPlan& P) const;
		/** Both centre wheels' paths searched together, on a grid of their
		 *  pairs, over samples KA to KB, from and back to the paths there. */
		void CentrePathsJoint(FPlan& P, int32 KA, int32 KB) const;
		/** The search's stages from KA to KB: every DpStageCm, and either side
		 *  of each planned stop. */
		TArray<int32> DpStages(const FPlan& Q, int32 KA, int32 KB) const;
		/** What an instant keeps, for the search: the least of its margin over
		 *  the requirement and its corridor over the least (very negative where
		 *  it has no pose). */
		double DpScore(const FInstantQuery& Q, double BGuess) const;
		/** The time (s) from sample K - 1 to K at the planned speed (a planned
		 *  stop's dwell apart). */
		double StepSeconds(const FPlan& P, int32 K) const
		{
			return Ds / FMath::Max(MinTimingSpeed, 0.5 * (P.V[K] + P.V[K - 1]));
		}
		/** The slowest the robot can be at sample K braking from the speed it
		 *  enters at (cm/s): the entry speed braked at Deceleration. */
		double EntryCurve(int32 K) const
		{
			const double E = FMath::Max(0.0, In.EntrySpeedCmS);
			const double Dec = FMath::Max(1.0, static_cast<double>(T.Deceleration));
			return FMath::Sqrt(FMath::Max(0.0, E * E - 2.0 * Dec * SAt(K)));
		}
		/** The speed (cm/s) a sample is capped at when nothing caps it: the
		 *  cruise speed -- or, entering faster, braking down to it. */
		double FreeCap(int32 K) const { return FMath::Max(Cruise, EntryCurve(K)); }
		/** The step from sample K - 1 to K is a planned stop's dwell: the robot at
		 *  rest at K - 1 while the centre wheels and the chassis re-pose. */
		static bool IsDwell(const FPlan& P, int32 K) { return K > 0 && P.Stop.IsValidIndex(K - 1) && P.Stop[K - 1] != 0; }
		/** How long the dwell at stop sample K lasts (s): the centre wheels'
		 *  and the chassis' moves to sample K + 1's, within CenterShiftSpeed and
		 *  BaseHeightSpeed, and a settle. */
		double DwellSeconds(const FPlan& P, int32 K) const;
		/** The reach of a centre wheel's 5-bar fore/aft (cm): the columns of its
		 *  grid with a span of heights inside LegRangeMargin. */
		FVector2D CentreReachX(int32 Side) const;
		/** How far (cm) the plan walks its centre wheels fore/aft while they
		 *  are on the ground, from the level-in's end: both sides together. */
		double PlantedWalkCm(const FPlan& P) const;
		void   CentreStatus(const FPlan& P, int32 Leg, int32 K, double X, double V, bool& bAir, bool& bFar) const;
		/** By sample: inside the slowing into, or the rise out of, a planned stop
		 *  (a run under the floor that holds a stop); only there may the plan run
		 *  under the floor. */
		TArray<uint8> StopRuns(const FPlan& P) const
		{
			TArray<uint8> Out;
			Out.Init(0, NumS);
			for (int32 k = 0; k < NumS; ++k)
			{
				if (!P.Stop[k])
				{
					continue;
				}
				for (int32 j = k; j >= 0 && (j == k || P.V[j] < Floor - 1e-6); --j)
				{
					Out[j] = 1;
				}
				for (int32 j = k; j < NumS && (j == k || P.V[j] < Floor - 1e-6); ++j)
				{
					Out[j] = 1;
				}
			}
			return Out;
		}
		/** The first sample after the plan's rise from rest (0 when it starts at or above the floor). */
		int32 RiseEnd(const FPlan& P) const;
		/** Whether leg Leg, a 5-bar at fore/aft X, is inside its keep-off window
		 *  at sample K: off the ground whatever the speed. */
		bool InWindow(int32 Leg, const FPlan& P, int32 K, double X) const
		{
			const double D = M.DistancePast(Leg, SAt(K), X);
			return D >= DUp[Leg] && D < DDown[Leg];
		}
		bool HeightPath(FPlan& P) const;
		bool Verify(FPlan& P) const;
		void Settle(FPlan& P) const;
		/** Speed, centre-wheel paths (bSearchX: searched; bJoint: and both wheels'
		 *  pairs searched where that leaves it failing), chassis height and
		 *  verification. */
		void Evaluate(FPlan& P, bool bSearchX, bool bJoint = false) const;
		bool IsOk(const FPlan& P) const { return P.bHeightOk && P.bVerified && P.FirstFail == INDEX_NONE; }
		/** The sample to name a plan's failure by: the first that does not stand,
		 *  is too slow or has no chassis height, else the first that fails
		 *  verification -- a height path that failed leaves later checks failing
		 *  only because of it. */
		static int32 FirstFailOf(const FPlan& P)
		{
			int32 First = INDEX_NONE;
			for (int32 k = 0; k < P.Fail.Num(); ++k)
			{
				if (P.Fail[k] == EFail::Static || P.Fail[k] == EFail::Slow || P.Fail[k] == EFail::Height)
				{
					return k;
				}
				if (First == INDEX_NONE && P.Fail[k] != EFail::None)
				{
					First = k;
				}
			}
			return First;
		}

		TArray<FIntPoint> FailRanges(const FPlan& P) const;
		bool			  TryTilt(TConstArrayView<const FPlan*> Bases, FPlan& Out, FString& OutWhy) const;
		/** A tilt search's candidate plan (TryTilt): its stretches and their
		 *  least tilts. */
		struct FBaseTilt
		{
			const FPlan*	  Base = nullptr;
			TArray<FIntPoint> Ranges;
			/** Each stretch's least tilt by the ring and by the square, and
			 *  whether each pick has one for every stretch within the limit. */
			TArray<FVector2D> Ring, Square;
			bool			  bRing = false, bSquare = false;
			TArray<uint8>	  Has;
			/** The most a stretch needs by the ring, within the limit or not
			 *  (LeastTilt), and the least inclination either pick comes to. */
			double	Need = 0.0;
			double	Least = TNumericLimits<double>::Max();
			FString Why;
			/** The plan the least tilts were searched on. */
			FPlan Floor;
		};
		/** The candidate plans' least tilts up to MaxPlannedTiltDeg, as the
		 *  first tilt search found them: a second search up to the same limit
		 *  (slowed, where the first found none) takes them as they are. */
		mutable TArray<FBaseTilt> TiltBasesAtLimit;
		mutable bool			  bTiltBasesAtLimit = false;
		/** The samples of Base that do not stand level at the floor speed, in
		 *  stretches, and for each the least tilt -- the least inclination of
		 *  the base -- that keeps a pose there (Has[r]: one within
		 *  MaxPlannedTiltDeg); adjacent stretches whose tilts
		 *  could not be eased from one to the other even at the floor speed are
		 *  taken as one. OutNeed: the most any stretch needs (or would need).
		 *  True when every stretch has one. OutFloor: the plan the search was
		 *  made on (Base at the floor speed, with Base's own sets where Base at
		 *  its speeds found no pose standing). */
		bool LeastTilt(const FPlan& Base, TArray<FIntPoint>& Ranges, TArray<FVector2D>& Tilts, TArray<uint8>& Has, double& OutNeed,
			FString& OutWhy, FPlan* OutFloor = nullptr) const;
		/** Whether every SearchEvery-th sample of Range of P stands at Tilt
		 *  (pitch, roll; deg). */
		bool StandsOver(const FPlan& P, const FIntPoint& Range, const FVector2D& Tilt) const;
		/** The least inclination at which every sample of Range of P stands, on
		 *  rings TiltGridDeg apart (the ring pick: a ring's middle of its
		 *  widest standing arc), up to MaxTilt; beyond it, OutBeyond is the
		 *  least ring (to twice the limit) with a tilt that would, and Out that
		 *  tilt (-1: none). */
		bool RangeTilt(const FPlan& P, const FIntPoint& Range, double MaxTilt, FVector2D& Out, double& OutBeyond) const;
		/** P at the floor speed (the fewest wheels off the ground together a plan
		 *  may keep), its failing samples marked Static where they do not stand. */
		FPlan AtFloor(const FPlan& Base) const;
		/** The plan tilted over Ranges (and stopping at StopsAt); with
		 *  bPlacements, where an easing between stretches has too little time
		 *  even at the floor, its other placements and the smaller stretch left
		 *  out are tried too. OutFailing (when given and the plan fails): the
		 *  stretches its failing samples are nearest (both of two they lie
		 *  between). */
		bool TiltPlan(const FPlan& Base, TConstArrayView<FIntPoint> Ranges, TConstArrayView<FVector2D> Tilts, TConstArrayView<int32> StopsAt,
			FPlan& Out, FString& OutWhy, bool bPlacements = true, TArray<int32>* OutFailing = nullptr) const;
		bool TryStops(const FPlan& Base, FPlan& Out, FString& OutWhy) const;
		bool TryStopsOver(TConstArrayView<const FPlan*> Bases, FPlan& Out, FString& OutWhy) const;
		/** The least tilt where it is enough and a planned stop where it is not
		 *  (where every stretch has a tilt within the limit, a stop at the one
		 *  that needs the most). */
		bool TryMixed(TConstArrayView<const FPlan*> Bases, FPlan& Out, FString& OutWhy) const;
		/** The planned tilt over P's samples at P's speeds (TiltProfile); the
		 *  easings that do not fit, in OutTight. */
		bool ApplyTilt(FPlan& P, TConstArrayView<FIntPoint> Ranges, TConstArrayView<FVector2D> Tilts, TArray<FTiltGap>* OutTight = nullptr,
			TConstArrayView<uint8> Modes = {}, double RateShare = 1.0) const;
		/** The drive stance as measured, and how it levels in: its margin, and
		 *  whether the robot must level in standing still. False, with why,
		 *  when it cannot drive off in it at all. */
		bool ApproachStance(FString& OutWhy);
		/** Where a level-in that begins at once, driving off or rolling on, is
		 *  done (S, cm). */
		double EarlyLevelFrom() const;
		/** Standing still, a level-in on the way that would leave the robot under
		 *  Ramms.Curb.LevelInAtRestShare of the floor speed is made standing
		 *  still instead (bLevelInAtRest), where the stance stands still. */
		void LevelInAtRestIfSlow();
		/** The level-in's least margin, by a straight move from the drive stance
		 *  to the plan's first level pose (LevelInMarginCm; reported). */
		void	MeasureLevelIn(const FPlan& P) const;
		FString DescribeFail(const FPlan& P, int32 K) const;
		void	Output(const FPlan& P, EPlanClass Class, FCurbSchedule& Out) const;
		/** Output, with P's lifts eased (EaseLifts) where the eased plan still
		 *  verifies. */
		void OutputEased(const FPlan& P, EPlanClass Class, FCurbSchedule& Out) const;

		const FPlannerInput&	In;
		FPlanModel				M;
		const FRammsCurbTuning& T;
		int32					N = 0;
		int32					NumS = 0;
		double					Ds = 1.0;
		double					Floor = 8.75;
		double					Cruise = 35.0;
		double					Req = 3.0;
		double					LevelFromS = 0.0;
		double					LevelInSeconds = 0.7;
		/** Where the level-in begins (S, cm): the drive stance up to here; and
		 *  where it must end at the latest, 30 cm short of the first window. */
		double LevelInFromS = 0.0;
		double LevelInLatest = 0.0;
		/** Rolling in a stance with the measured floor at a steady speed only:
		 *  the speed is held (no braking, no speeding up) until the level-in
		 *  ends. */
		bool bHoldThroughLevelIn = false;
		/** A stance under MinStabilityMargin (over the measured floor): the
		 *  level-in begins as the crossing does. */
		bool bLevelInAtOnce = false;
		/** The least margin of the level-in, by a straight move from the drive
		 *  stance to the first level pose (cm; reported, not required). */
		mutable double LevelInMarginCm = 0.0;
		/** The drive stance's dynamic and static margins (cm), and whether the
		 *  robot levels in standing still. */
		double		   StanceMargin = 0.0;
		double		   StanceStaticMargin = 0.0;
		bool		   bLevelInAtRest = false;
		double		   B0 = 0.0;
		TArray<double> DUp, DDown, Eps, Lambda;
		/** The 5-bar legs, -Y side first ([0]), +Y ([1]); INDEX_NONE where none. */
		int32 Centre[2] = { INDEX_NONE, INDEX_NONE };
		/** For the plan lines: how long each part took (s). */
		mutable double TimeGeometry = 0.0, TimeSpeed = 0.0, TimeDp = 0.0, TimeHeight = 0.0, TimeVerify = 0.0, TimeTilt = 0.0;
		mutable int32  CountFeasible = 0;
		/** The plan's work: instants of the robot model evaluated (the same for
		 *  the same input; FCurbSchedule::PlanWork). */
		mutable int64 Work = 0;
		/** One line per candidate tried, for the plan lines. */
		mutable TArray<FString> Tried;
		/** Said in the plan's summary line when the tilt planned is more than
		 *  the least (its least did not verify at the plan's speeds). */
		mutable FString TiltNote;
		/** The two grids a crossing is planned on side by side (PlanCrossing),
		 *  and which this is; the stage this grid is in (no plan it can still
		 *  make ranks above it); and once it has stopped early, why. */
		FGridRace*		   Race = nullptr;
		int32			   Slot = 0;
		mutable EPlanStage Stage = EPlanStage::Level;
		mutable FString	   StoppedWhy;
		/** Whether to stop: the other grid has finished with a plan that ranks
		 *  above anything left to this one (its stage later than this one's
		 *  now -- and, on the move, sure to be kept against this one's work),
		 *  or, on the move, this one's work is over its budget against it.
		 *  Either way the plan this grid would make is not the one kept, so
		 *  stopping changes nothing but the time. Sticky. */
		bool Stopped() const;
		/** A grid stopped early: a refusal saying why. */
		FCurbSchedule& StoppedPlan(FCurbSchedule& Out) const;
		/** The tilted plans' attempts made slower (TiltPlan) left for this
		 *  crossing: each costs a plan, so a crossing every tilt of which fails
		 *  does not spend MaxSlowTiltRounds on each. */
		mutable int32 SlowAttemptsLeft = MaxSlowTiltAttempts;
		/** The slowed tilt search where none was found at the plans' speeds
		 *  makes no plan once the planner's work is past this (instants; under 0
		 *  no limit): SlowTiltWhenNoneWorkShare of the work before it. */
		mutable int64 SlowWorkLimit = -1;
		bool		  OverSlowWork() const { return SlowWorkLimit >= 0 && Work > SlowWorkLimit; }
		/** The tilt search's second pass: its plans may be made slower over the
		 *  samples that fail at their speeds (TiltPlan), and it looks only for a
		 *  tilt under TiltCeilingDeg (deg of inclination; under 0 none: up to
		 *  MaxPlannedTiltDeg). */
		mutable bool   bSlowTilt = false;
		mutable double TiltCeilingDeg = -1.0;
	};

	// --- setup -----------------------------------------------------------------------------

	bool FPlanner::Setup(FString& OutWhy)
	{
		N = M.NumLegs();
		if (N < 3 || N > MaxPlanLegs)
		{
			OutWhy = FString::Printf(TEXT("the planner takes 3 to %d legs, not %d"), MaxPlanLegs, N);
			return false;
		}
		if (!In.Terrain.bValid)
		{
			OutWhy = TEXT("no terrain to plan on");
			return false;
		}
		Cruise = FMath::Max(1.0, static_cast<double>(T.DriveSpeed));
		Floor = FMath::Clamp(static_cast<double>(T.MinSpeedFraction), 0.0, 1.0) * Cruise;
		Req = T.MinStabilityMargin;
		B0 = In.OriginZ;
		int32 NumCentres = 0;
		for (int32 i = 0; i < N; ++i)
		{
			if (M.IsCentre(i))
			{
				++NumCentres;
				Centre[M.Side(i)] = i;
			}
		}
		if (NumCentres > 2 || (NumCentres == 2 && (Centre[0] == INDEX_NONE || Centre[1] == INDEX_NONE)))
		{
			OutWhy = TEXT("the planner walks at most two 5-bar legs, one on either side");
			return false;
		}
		DUp.SetNum(N);
		DDown.SetNum(N);
		Eps.SetNum(N);
		Lambda.SetNum(N);
		double SLast = 0.0;
		double SFirst = TNumericLimits<double>::Max();
		for (int32 i = 0; i < N; ++i)
		{
			KeepOffWindow(M.Radius(i), M.TyreWidth(i), M.RollDirection(i), In.Terrain.EdgeNormal, M.IsClimb(), T, DUp[i], DDown[i],
				Eps[i], Lambda[i]);
			const double X = In.Legs[i].Measured.X;
			if (M.DistancePast(i, 0.0, X) >= DUp[i] - 1e-6)
			{
				OutWhy = FString::Printf(TEXT("the '%s' wheel is already %.1f cm past the edge, inside its keep-off window (from %.1f "
											  "cm): a crossing begun part way is not planned yet"),
					*LegName(i), M.DistancePast(i, 0.0, X), DUp[i]);
				return false;
			}
			SLast = FMath::Max(SLast, M.TravelAt(i, DDown[i], X));
			SFirst = FMath::Min(SFirst, M.TravelAt(i, DUp[i], X));
		}
		// Past the last window: the longest landing tail at the cruise speed,
		// the centre wheels' walk home, and room.
		const double Tail = Cruise * 1.5 + 40.0 + T.FinishClearance;
		NumS = FMath::Max(2, FMath::CeilToInt((SLast + Tail) / Ds) + 1);

		// The level-in: from the drive stance to every leg on the near surface
		// under its wheel, level, at the chassis height the robot stands at now --
		// the legs' move (from a tilted stance, the base turning no faster than
		// TiltRateDegPerSec / TiltAccelDegPerSec2), its lag, and a spare 0.3 s. It is done before
		// any wheel comes within 30 cm of its window; on a near surface sloping
		// more than EarlyLevelInSlopeDeg it begins at once (the drive stance is
		// least stable there), and the plan is level from where it ends.
		double Move = 0.0;
		for (int32 i = 0; i < N; ++i)
		{
			const double X = In.Legs[i].Measured.X;
			const double Z = M.SupportZ(i, M.WheelXY(i, 0.0, X), false) - B0;
			Move = FMath::Max(Move, FMath::Abs(Z - In.Legs[i].Measured.Y));
		}
		// Every leg moves together along one S-curve in time (the executor's
		// ramp): its share of the move bounded so that no leg goes faster than
		// LegSpeed / LegAcceleration and the base turns -- pitch and roll
		// together, along the straight way to level -- no faster than
		// TiltRateDegPerSec / TiltAccelDegPerSec2.
		const double Turn = In.AttitudeDeg.Size();
		const double ShareRate = FMath::Min(FMath::Max(0.1, static_cast<double>(T.TiltRateDegPerSec)) / FMath::Max(Turn, 1e-3),
			FMath::Max(1.0, static_cast<double>(T.LegSpeed)) / FMath::Max(Move, 1e-3));
		const double ShareAccel = FMath::Min(FMath::Max(0.1, static_cast<double>(T.TiltAccelDegPerSec2)) / FMath::Max(Turn, 1e-3),
			FMath::Max(1.0, static_cast<double>(T.LegAcceleration)) / FMath::Max(Move, 1e-3));
		LevelInSeconds = SCurveSeconds(1.0, ShareRate, ShareAccel) + T.ServoLagSeconds + 0.3;
		LevelInLatest = FMath::Max(0.0, SFirst - 30.0 / M.NormalShare());
		// On a near surface sloping more than EarlyLevelInSlopeDeg the level-in
		// begins as soon as the crossing does (the drive stance is least stable
		// there); elsewhere it ends 30 cm short of the first window.
		LevelFromS = In.Terrain.Near.SlopeDeg() > EarlyLevelInSlopeDeg ? EarlyLevelFrom() : LevelInLatest;
		if (!ApproachStance(OutWhy))
		{
			return false;
		}
		LevelInAtRestIfSlow();
		return true;
	}

	void FPlanner::LevelInAtRestIfSlow()
	{
		// A robot standing still whose level-in on the way would be made under
		// the floor speed -- the plan holds its speed at the floor there
		// (ApplyLevelInCap: a late level-in is a slower start, not a refusal),
		// but its room is too short for the level-in's time at that speed, and
		// the executor's late level-in cap crawls under it -- levels in standing
		// still instead, before it moves, where its stance stands still: the
		// same level-in, then the crossing from rest at the plan's own speeds.
		// A slower start, not a crawl once under way. (LevelInAtRestShare of the
		// floor, 1.3: a level-in can run over its planned time, which the
		// executor's late level-in cap then takes off the speed, and the
		// measured speed lags the command.)
		const double Share = LevelInAtRestShare();
		if (Share <= 0.0 || bLevelInAtRest || bHoldThroughLevelIn || LevelFromS <= 0.0 || LevelInSeconds <= 0.0)
		{
			return;
		}
		const double E = FMath::Max(0.0, In.EntrySpeedCmS);
		if (E >= StopSpeedCmS || StanceStaticMargin < Req - 1e-9)
		{
			return;
		}
		// The time to LevelFromS rising from E at Acceleration to C, then held at C.
		const double C = FMath::Max(E + 1e-3, FMath::Min(Cruise, Share * Floor));
		const double Acc = FMath::Max(1.0, static_cast<double>(T.Acceleration));
		const double TRise = (C - E) / Acc;
		const double DRise = E * TRise + 0.5 * Acc * TRise * TRise;
		const double Time = LevelFromS <= DRise ? (FMath::Sqrt(E * E + 2.0 * Acc * LevelFromS) - E) / Acc : TRise + (LevelFromS - DRise) / C;
		if (Time >= LevelInSeconds)
		{
			return;
		}
		bLevelInAtRest = true;
		bLevelInAtOnce = false;
		LevelFromS = 0.0;
	}

	double FPlanner::EarlyLevelFrom() const
	{
		// Levelling in from where it is, driving off: the distance the level-in
		// takes from the entry speed, rising at Acceleration to the cruise speed
		// or braking at Deceleration down to it (never past the 30 cm line).
		const double V0 = FMath::Max(0.0, In.EntrySpeedCmS);
		const double TL = LevelInSeconds;
		double		 Dist = 0.0;
		if (V0 <= Cruise)
		{
			const double Acc = FMath::Max(1.0, static_cast<double>(T.Acceleration));
			const double TRise = (Cruise - V0) / Acc;
			Dist = TL <= TRise ? V0 * TL + 0.5 * Acc * TL * TL : V0 * TRise + 0.5 * Acc * TRise * TRise + Cruise * (TL - TRise);
		}
		else
		{
			const double Dec = FMath::Max(1.0, static_cast<double>(T.Deceleration));
			const double TFall = (V0 - Cruise) / Dec;
			Dist = TL <= TFall ? V0 * TL - 0.5 * Dec * TL * TL : V0 * TFall - 0.5 * Dec * TFall * TFall + Cruise * (TL - TFall);
		}
		return FMath::Min(Dist, LevelInLatest);
	}

	bool FPlanner::ApproachStance(FString& OutWhy)
	{
		// The robot as it stands now, in the stance it drove up in: its legs
		// where they are, the base at its measured attitude, the wheels on the
		// ground carrying it. It drives on in that stance until the level-in
		// is done. A planned pose keeps MinStabilityMargin braking and speeding
		// up; this one the planner did not choose, and is held to the measured
		// floor, MinLiveStabilityMarginCm:
		//  - with MinStabilityMargin to spare, it levels in on the way, as
		//    planned;
		//  - standing still in it with too little to drive off but enough to
		//    stand, it levels in first, standing still;
		//  - otherwise, with MinLiveStabilityMarginCm braking and speeding up,
		//    it levels in at once, driving off or rolling on;
		//  - rolling with less than that, but that much at a steady speed, it
		//    levels in at once holding its speed (no braking, no speeding up)
		//    until it is level -- if it is level before the first wheel comes
		//    within 30 cm of its window;
		//  - else it refuses.
		TArray<FVector2D, TInlineAllocator<MaxPlanLegs>> XZ;
		uint32											 Support = 0;
		for (int32 i = 0; i < N; ++i)
		{
			XZ.Add(In.Legs[i].Measured);
			Support |= (In.Legs[i].bOffGround ? 0u : 1u) << i;
		}
		StanceMargin = M.PoseMarginAt(XZ, Support, 0, 0.0, B0, In.AttitudeDeg.X, In.AttitudeDeg.Y, true);
		StanceStaticMargin = M.PoseMarginAt(XZ, Support, 0, 0.0, B0, In.AttitudeDeg.X, In.AttitudeDeg.Y, false);
		bLevelInAtRest = false;
		bHoldThroughLevelIn = false;
		bLevelInAtOnce = false;
		if (StanceMargin >= Req - 1e-9)
		{
			return true;
		}
		const double E = FMath::Max(0.0, In.EntrySpeedCmS);
		const double Live = FMath::Max(0.0, static_cast<double>(T.MinLiveStabilityMarginCm));
		if (E < StopSpeedCmS && StanceStaticMargin >= Req - 1e-9)
		{
			bLevelInAtRest = true;
			LevelFromS = 0.0;
			return true;
		}
		if (StanceMargin >= Live - 1e-9)
		{
			bLevelInAtOnce = true;
			LevelFromS = FMath::Min(LevelFromS, EarlyLevelFrom());
			return true;
		}
		if (E >= StopSpeedCmS && StanceStaticMargin >= Live - 1e-9)
		{
			bHoldThroughLevelIn = true;
			LevelFromS = E * LevelInSeconds;
			if (LevelFromS <= LevelInLatest + 1e-9)
			{
				return true;
			}
			OutWhy = FString::Printf(TEXT("the drive stance the robot rolls in at %.1f cm/s (pitch %+.1f, roll %+.1f deg) stands only %.2f cm "
										  "inside its wheels braking or speeding up (MinLiveStabilityMarginCm %.1f): holding that speed it would "
										  "level in only %.0f cm on, past the %.0f cm it has before the first wheel nears the edge"),
				E, In.AttitudeDeg.X, In.AttitudeDeg.Y, StanceMargin, Live, LevelFromS, LevelInLatest);
			return false;
		}
		OutWhy = FString::Printf(TEXT("the drive stance the robot is %s in (pitch %+.1f, roll %+.1f deg) stands only %.2f cm inside its wheels "
									  "braking or speeding up and %.2f standing still (MinLiveStabilityMarginCm %.1f, MinStabilityMargin %.1f)"),
			E < StopSpeedCmS ? TEXT("standing") : *FString::Printf(TEXT("rolling at %.1f cm/s"), E), In.AttitudeDeg.X, In.AttitudeDeg.Y,
			StanceMargin, StanceStaticMargin, Live, Req);
		return false;
	}

	double FPlanner::DwellSeconds(const FPlan& P, int32 K) const
	{
		if (!P.Stop.IsValidIndex(K) || !P.Stop[K] || K + 1 >= NumS)
		{
			return 0.0;
		}
		const double CSS = FMath::Max(0.1, static_cast<double>(T.CenterShiftSpeed));
		const double BH = FMath::Max(0.1, static_cast<double>(T.BaseHeightSpeed));
		double		 Move = FMath::Abs(P.B[K + 1] - P.B[K]) / BH;
		for (int32 Side = 0; Side < 2; ++Side)
		{
			if (Centre[Side] != INDEX_NONE)
			{
				Move = FMath::Max(Move, FMath::Abs(P.X[Side][K + 1] - P.X[Side][K]) / CSS);
			}
		}
		// The walk, its servo lag and a settle; a stop is a stop even with
		// nothing to move.
		return Move + T.ServoLagSeconds + LegSettleSeconds;
	}

	double FPlanner::PlantedWalkCm(const FPlan& P) const
	{
		double Walk = 0.0;
		for (int32 Side = 0; Side < 2; ++Side)
		{
			const int32 Leg = Centre[Side];
			if (Leg == INDEX_NONE || !P.Wheels.IsValidIndex(Leg) || P.X[Side].Num() != NumS)
			{
				continue;
			}
			const FWheelPlan& W = P.Wheels[Leg];
			for (int32 k = FMath::Max(1, Index(LevelFromS) + 1); k < NumS; ++k)
			{
				const double S = SAt(k);
				if (S < W.SLiftStart || S > W.SLoaded)
				{
					Walk += FMath::Abs(P.X[Side][k] - P.X[Side][k - 1]);
				}
			}
		}
		return Walk;
	}

	FVector2D FPlanner::CentreReachX(int32 Side) const
	{
		const int32 Leg = Centre[Side];
		if (Leg == INDEX_NONE)
		{
			return FVector2D(0.0, -1.0);
		}
		// Where the 5-bar can stand its wheel: a column of its grid with heights
		// to spare inside LegRangeMargin either end.
		const double M2 = 2.0 * T.LegRangeMargin + 1.0;
		double		 Lo = TNumericLimits<double>::Max(), Hi = -TNumericLimits<double>::Max();
		for (double X = -40.0; X <= 40.0 + 1e-9; X += 1.0)
		{
			const FVector2D R = M.Reach(Leg, X);
			if (R.Y - R.X >= M2)
			{
				Lo = FMath::Min(Lo, X);
				Hi = FMath::Max(Hi, X);
			}
		}
		const double X0 = CentreX0(Side);
		return Lo <= Hi ? FVector2D(FMath::Min(Lo, X0), FMath::Max(Hi, X0)) : FVector2D(X0, X0);
	}

	void FPlanner::InitPlan(FPlan& P, const TCHAR* Name) const
	{
		P = FPlan();
		P.Name = Name;
		for (int32 Side = 0; Side < 2; ++Side)
		{
			P.X[Side].Init(CentreX0(Side), NumS);
		}
		P.Pitch.Init(0.0, NumS);
		P.Roll.Init(0.0, NumS);
		P.VCap.SetNum(NumS);
		for (int32 k = 0; k < NumS; ++k)
		{
			P.VCap[k] = FreeCap(k);
		}
		P.TiltCap.Init(NoCap, NumS);
		P.CapBy.Init(INDEX_NONE, NumS);
		P.V.Init(Cruise, NumS);
		P.Stop.Init(0, NumS);
		P.B.Init(B0, NumS);
		P.BLo.Init(0.0, NumS);
		P.BHi.Init(0.0, NumS);
		P.Margin.Init(0.0, NumS);
		P.SeatX.Init(In.Model.Seat.ZeroPivot.X, NumS);
		P.SeatBand.Init(FVector2D(1.0, 0.0), NumS);
		P.SeatCap.Init(NoCap, NumS);
		P.ForeCap.Init(NoCap, NumS);
		P.Air.Init(0, NumS);
		P.Far.Init(0, NumS);
		P.Touch.Init(0, NumS);
		P.StaticAir.Init(0, NumS);
		P.StaticFar.Init(0, NumS);
		P.StaticOk.Init(1, NumS);
		P.Fail.Init(EFail::None, NumS);
		P.Wheels.SetNum(N);
		P.Geom.SetNum(N);
		ApplyLevelInCap(P);
	}

	void FPlanner::ApplyLevelInCap(FPlan& P) const
	{
		// A start too close for the level-in at full speed: slow enough that it
		// is done by LevelFromS -- the most speed C at which, rising from the
		// entry speed at Acceleration (or braking from it at Deceleration, no
		// sooner) and held at C, the robot takes LevelInSeconds to get there.
		// Never under the floor: a level-in that is late is a slower start, not
		// a refusal.
		if (LevelFromS <= 0.0 || LevelInSeconds <= 0.0)
		{
			return;
		}
		const double E = FMath::Max(0.0, In.EntrySpeedCmS);
		if (bHoldThroughLevelIn)
		{
			// At the speed it rolls in at, no more, no less, until it is level.
			for (int32 k = 0; k <= Index(LevelFromS); ++k)
			{
				P.VCap[k] = E;
				P.CapBy[k] = CapLevelIn;
			}
			return;
		}
		const double Acc = FMath::Max(1.0, static_cast<double>(T.Acceleration));
		const double Dec = FMath::Max(1.0, static_cast<double>(T.Deceleration));
		const int32	 KL = Index(LevelFromS);
		auto		 SpeedAt = [&](double C, int32 k) {
			const double S = SAt(k);
			return E > C ? FMath::Max(C, FMath::Sqrt(FMath::Max(0.0, E * E - 2.0 * Dec * S))) : FMath::Min(C, FMath::Sqrt(E * E + 2.0 * Acc * S));
		};
		auto Seconds = [&](double C) {
			double Time = 0.0;
			for (int32 k = 1; k <= KL; ++k)
			{
				Time += Ds / FMath::Max(MinTimingSpeed, 0.5 * (SpeedAt(C, k - 1) + SpeedAt(C, k)));
			}
			return Time;
		};
		const double Most = FMath::Max(Cruise, E);
		if (Seconds(Most) >= LevelInSeconds)
		{
			return;
		}
		double Lo = FMath::Min(Floor, Most), Hi = Most;
		if (Seconds(Lo) >= LevelInSeconds)
		{
			for (int32 n = 0; n < 30; ++n)
			{
				const double Mid = 0.5 * (Lo + Hi);
				(Seconds(Mid) >= LevelInSeconds ? Lo : Hi) = Mid;
			}
		}
		for (int32 k = 0; k <= KL; ++k)
		{
			const double Cap = SpeedAt(Lo, k) + (E > Lo ? 1e-9 : 0.0);
			if (P.VCap[k] > Cap)
			{
				P.VCap[k] = Cap;
				P.CapBy[k] = CapLevelIn;
			}
		}
	}

	void FPlanner::Passes(FPlan& P) const
	{
		const double Acc = FMath::Max(1.0, static_cast<double>(T.Acceleration));
		const double Dec = FMath::Max(1.0, static_cast<double>(T.Deceleration));
		// Within the crossing (as last timed), no faster than CrossingSpeedRatio
		// times the tightest cap there: one steady pace rather than a surge
		// between hand-offs.
		int32  SteadyFrom = NumS, SteadyTo = -1;
		double SteadyCap = TNumericLimits<double>::Max();
		if (T.CrossingSpeedRatio > 0.0f && P.Wheels.Num() > 0)
		{
			double First = TNumericLimits<double>::Max(), Last = -TNumericLimits<double>::Max();
			for (const FWheelPlan& W : P.Wheels)
			{
				First = FMath::Min(First, W.bHeldFromStart ? W.SLand : W.SLiftStart);
				Last = FMath::Max(Last, W.SLoaded);
			}
			SteadyFrom = FMath::Max(Index(FMath::Max(First, LevelFromS)), 0);
			SteadyTo = FMath::Min(Index(Last), NumS - 1);
			double Least = TNumericLimits<double>::Max();
			for (int32 k = SteadyFrom; k <= SteadyTo; ++k)
			{
				// (The drive slowed under the seat's shift is no hand-off to pace by.)
				const bool bFore = P.ForeCap.IsValidIndex(k) && P.ForeCap[k] < NoCap && P.VCap[k] >= P.ForeCap[k] - 1e-9;
				if (!P.Stop[k] && !bFore)
				{
					Least = FMath::Min(Least, FMath::Min(P.VCap[k], FreeCap(k)));
				}
			}
			if (Least < TNumericLimits<double>::Max())
			{
				SteadyCap = static_cast<double>(T.CrossingSpeedRatio) * Least;
			}
		}
		bool bStops = false;
		for (int32 k = 0; k < NumS; ++k)
		{
			bStops |= P.Stop[k] != 0;
			double V = P.Stop[k] ? 0.0 : FMath::Min(P.VCap[k], FreeCap(k));
			if (k >= SteadyFrom && k <= SteadyTo)
			{
				V = FMath::Min(V, SteadyCap);
			}
			V = k == 0 ? FMath::Min(V, FMath::Max(0.0, In.EntrySpeedCmS)) : FMath::Min(V, FMath::Sqrt(P.V[k - 1] * P.V[k - 1] + 2.0 * Acc * Ds));
			P.V[k] = V;
		}
		for (int32 k = NumS - 2; k >= 0; --k)
		{
			P.V[k] = FMath::Min(P.V[k], FMath::Sqrt(P.V[k + 1] * P.V[k + 1] + 2.0 * Dec * Ds));
		}
		// Smoothed over SpeedSmoothingCm: the least over each window, then that
		// averaged over it. Every sample's window holds each sample that its
		// average draws on, so the result is nowhere above the speed it smooths.
		const int32 Half = FMath::RoundToInt(0.5 * static_cast<double>(T.SpeedSmoothingCm) / Ds);
		if (Half > 0 && !bStops)
		{
			TArray<double> Least;
			Least.SetNumUninitialized(NumS);
			for (int32 k = 0; k < NumS; ++k)
			{
				double L = P.V[k];
				for (int32 j = FMath::Max(0, k - Half); j <= FMath::Min(NumS - 1, k + Half); ++j)
				{
					L = FMath::Min(L, P.V[j]);
				}
				Least[k] = L;
			}
			for (int32 k = 0; k < NumS; ++k)
			{
				double		Sum = 0.0;
				const int32 A = FMath::Max(0, k - Half), B = FMath::Min(NumS - 1, k + Half);
				for (int32 j = A; j <= B; ++j)
				{
					Sum += Least[j];
				}
				P.V[k] = FMath::Min(P.V[k], Sum / (B - A + 1));
			}
		}
	}

	int32 FPlanner::RiseEnd(const FPlan& P) const
	{
		// As the run stats count it: the robot under way from rest has risen
		// once its speed stops climbing (or it started at the floor or more).
		if (P.V.Num() == 0 || P.V[0] >= Floor - 1e-6)
		{
			return 0;
		}
		for (int32 k = 0; k + 1 < NumS; ++k)
		{
			if (P.V[k + 1] <= P.V[k] + 1e-9)
			{
				return k;
			}
		}
		return NumS - 1;
	}

	double FPlanner::MarchBack(const FPlan& P, double S, double Seconds) const
	{
		double Time = 0.0;
		int32  k = Index(S);
		while (k > 0 && Time < Seconds)
		{
			const double Dt = Ds / FMath::Max(MinTimingSpeed, 0.5 * (P.V[k] + P.V[k - 1]));
			if (Time + Dt >= Seconds)
			{
				return SAt(k) - Ds * (Seconds - Time) / Dt;
			}
			Time += Dt;
			--k;
		}
		return SAt(k);
	}

	double FPlanner::MarchFwd(const FPlan& P, double S, double Seconds) const
	{
		double Time = 0.0;
		int32  k = Index(S);
		while (k < NumS - 1 && Time < Seconds)
		{
			const double Dt = Ds / FMath::Max(MinTimingSpeed, 0.5 * (P.V[k] + P.V[k + 1]));
			if (Time + Dt >= Seconds)
			{
				return SAt(k) + Ds * (Seconds - Time) / Dt;
			}
			Time += Dt;
			++k;
		}
		return SAt(k);
	}

	double FPlanner::LegXAt(int32 Leg, const FPlan& P, int32 K, bool bLanded) const
	{
		if (M.IsCentre(Leg))
		{
			return P.X[M.Side(Leg)][K];
		}
		// A corner's fore/aft at the height it holds there: on the plane it
		// stands on at the planned chassis height (before any is planned, at the
		// height it stood at when the plan began).
		if (!P.bBuilt)
		{
			return In.Legs[Leg].Measured.X;
		}
		double X = In.Legs[Leg].Measured.X;
		for (int32 Pass = 0; Pass < 2; ++Pass)
		{
			X = M.CornerX(Leg, M.SupportZ(Leg, M.WheelXY(Leg, SAt(K), X), bLanded) - P.B[K]);
		}
		return X;
	}

	double FPlanner::ProfileZ(int32 Leg, double S, const FPlan& P) const
	{
		// A lifted wheel's clearance where it is at travel S: a corner at the
		// fore/aft its carry height puts it (the plan's chassis height there).
		const int32 K = Index(S);
		double		X = M.IsCentre(Leg) ? P.X[M.Side(Leg)][K] : In.Legs[Leg].Measured.X;
		double		Need = M.ClearanceZ(Leg, M.WheelXY(Leg, S, X));
		if (!M.IsCentre(Leg) && P.bBuilt)
		{
			for (int32 Pass = 0; Pass < 2; ++Pass)
			{
				const FVector2D R = M.Reach(Leg, X);
				X = M.CornerX(Leg, FMath::Clamp(Need - P.B[K], R.X, R.Y));
				Need = M.ClearanceZ(Leg, M.WheelXY(Leg, S, X));
			}
		}
		return Need;
	}

	void FPlanner::ComputeGeometry(FPlan& P) const
	{
		const double Started = FPlatformTime::Seconds();
		for (int32 i = 0; i < N; ++i)
		{
			FLegGeom& G = P.Geom[i];
			G = FLegGeom();
			// Where it reaches its window, and leaves it, at the fore/aft its leg
			// holds there.
			G.KUp = NumS - 1;
			G.KDown = NumS - 1;
			for (int32 k = 0; k < NumS; ++k)
			{
				if (M.DistancePast(i, SAt(k), LegXAt(i, P, k, false)) >= DUp[i])
				{
					G.KUp = k;
					break;
				}
			}
			for (int32 k = G.KUp; k < NumS; ++k)
			{
				if (M.DistancePast(i, SAt(k), LegXAt(i, P, k, true)) >= DDown[i])
				{
					G.KDown = k;
					break;
				}
			}
			const int32 Reach = FMath::CeilToInt(3.0 * M.Radius(i) / Ds) + 2;
			G.Prof.K0 = FMath::Max(0, G.KUp - Reach);
			const int32 KB = FMath::Min(NumS - 1, G.KDown + Reach);
			G.Prof.Z.SetNum(KB - G.Prof.K0 + 1);
			for (int32 k = G.Prof.K0; k <= KB; ++k)
			{
				G.Prof.Z[k - G.Prof.K0] = ProfileZ(i, SAt(k), P);
			}
			const double SUp = SAt(G.KUp), SLand = SAt(G.KDown);
			G.ZPlant = M.SupportZ(i, M.WheelXY(i, SUp, LegXAt(i, P, G.KUp, false)), false);
			G.ZLand = M.SupportZ(i, M.WheelXY(i, SLand, LegXAt(i, P, G.KDown, true)), true);
			G.ZTop = FMath::Max(G.ZPlant, G.Prof.MaxOver(SUp, SLand, Ds));
			G.SDropFrom = SLand;
			// A crank that may turn only so fast: no faster on average over the
			// move than turns it from where it starts to where it ends (a corner's
			// 4-bar table, at the chassis height planned there).
			const double LegSpeed = FMath::Max(1.0, static_cast<double>(T.LegSpeed));
			const double CrankRate = In.Legs[i].MaxCrankRateRadS;
			G.SpeedUp = G.SpeedDown = LegSpeed;
			if (CrankRate > 0.0 && !M.IsCentre(i))
			{
				const double BUp = BGuessAt(P, G.KUp), BDown = BGuessAt(P, G.KDown);
				const double TurnUp = FMath::Abs(M.CornerCrank(i, G.ZTop - BUp) - M.CornerCrank(i, G.ZPlant - BUp));
				const double TurnDown = FMath::Abs(M.CornerCrank(i, G.ZTop - BDown) - M.CornerCrank(i, G.ZLand - BDown));
				if (TurnUp > 1e-9)
				{
					G.SpeedUp = FMath::Min(LegSpeed, FMath::Max(0.1, (G.ZTop - G.ZPlant) * CrankRate / TurnUp));
				}
				if (TurnDown > 1e-9)
				{
					G.SpeedDown = FMath::Min(LegSpeed, FMath::Max(0.1, FMath::Abs(G.ZTop - G.ZLand) * CrankRate / TurnDown));
				}
			}
			double ZMax = G.Prof.At(SLand, Ds);
			for (double S = SLand; S >= SLand - 2.5 * M.Radius(i); S -= Ds)
			{
				const double Z = G.Prof.At(S, Ds);
				if (Z > ZMax + 1e-6)
				{
					ZMax = Z;
					G.SDropFrom = S;
				}
			}
		}
		TimeGeometry += FPlatformTime::Seconds() - Started;
	}

	void FPlanner::ComputeStatic(FPlan& P) const
	{
		// Standing still: which wheels are up at each sample, which down on the
		// far side, and whether the robot stands so.
		const double Started = FPlatformTime::Seconds();
		const int32	 K0 = Index(LevelFromS);
		for (int32 k = 0; k < NumS; ++k)
		{
			uint32 Air = 0, Far = 0;
			for (int32 i = 0; i < N; ++i)
			{
				const bool bHeld = P.Wheels[i].bHeldFromStart;
				if ((k >= P.Geom[i].KUp || bHeld) && k < P.Geom[i].KDown)
				{
					Air |= 1u << i;
				}
				else if (k >= P.Geom[i].KDown)
				{
					Far |= 1u << i;
				}
			}
			P.StaticAir[k] = Air;
			P.StaticFar[k] = Far;
			P.StaticOk[k] = 1;
		}
		for (int32 k = K0; k < NumS; ++k)
		{
			const bool bOk = Stands(P, Query(P, k, P.StaticAir[k], P.StaticFar[k]), BGuessAt(P, k));
			P.StaticOk[k] = bOk ? 1 : 0;
		}
		TimeGeometry += FPlatformTime::Seconds() - Started;
	}

	double FPlanner::LiftLead(const FLegGeom& G, double V) const
	{
		const int32 Key = FMath::RoundToInt(V / TimingSpeedStep);
		if (const double* Found = G.Lift.Find(Key))
		{
			return *Found;
		}
		V = Key * TimingSpeedStep;
		const double Lead = LiftLeadAt(G, V, G.SpeedUp > 0.0 ? G.SpeedUp : FMath::Max(1.0, static_cast<double>(T.LegSpeed)));
		G.Lift.Add(Key, Lead);
		return Lead;
	}

	double FPlanner::LiftLeadAt(const FLegGeom& G, double V, double Speed) const
	{
		// The wheel stands on its surface, its centre at ZPlant; lifted, it must
		// be over its clearance profile from its window on. Its command rises to
		// the most that profile asks over the window (a trapezoid at Speed and
		// LegAcceleration) and the wheel follows it a servo lag behind. How long
		// before reaching the window must the lift begin?
		const double Tau = FMath::Max(1e-3, static_cast<double>(T.ServoLagSeconds));
		const double Accel = FMath::Max(1.0, static_cast<double>(T.LegAcceleration));
		const double SUp = SAt(G.KUp), SLand = SAt(G.KDown);
		double		 Lead = LegSettleSeconds;
		if (G.ZTop > G.ZPlant)
		{
			auto Clears = [&](double L) {
				double C = G.ZPlant, Cv = 0.0, H = G.ZPlant;
				for (double t = 0.0; t < L + 6.0; t += SimDt)
				{
					TrackStep(C, Cv, G.ZTop, Speed, Accel, SimDt);
					H += (C - H) * FMath::Min(1.0, SimDt / Tau);
					if (t + 1e-9 >= L)
					{
						const double S = SUp + V * (t - L);
						const double Need = G.Prof.At(FMath::Min(S, SLand), Ds) - ProfileToleranceCm;
						if (H < Need)
						{
							return false;
						}
						if (S > SLand || H >= G.ZTop - ProfileToleranceCm || V < MinTimingSpeed)
						{
							// Past the window, or at the top (it stays there), or
							// standing still at the window's edge, up in time.
							return true;
						}
					}
				}
				return true;
			};
			double Lo = 0.0, Hi = 0.5;
			while (!Clears(Hi) && Hi < 8.0)
			{
				Lo = Hi;
				Hi *= 2.0;
			}
			for (int32 n = 0; n < 10; ++n)
			{
				const double Mid = 0.5 * (Lo + Hi);
				(Clears(Mid) ? Hi : Lo) = Mid;
			}
			Lead = Hi + LegSettleSeconds;
		}
		return Lead;
	}

	double FPlanner::SlowestLiftSpeed(const FLegGeom& G, double V, double Lead) const
	{
		const double Fast = G.SpeedUp > 0.0 ? G.SpeedUp : FMath::Max(1.0, static_cast<double>(T.LegSpeed));
		double		 Lo = FMath::Min(MinEasedLiftSpeed, Fast), Hi = Fast;
		if (LiftLeadAt(G, V, Lo) <= Lead)
		{
			return Lo;
		}
		if (LiftLeadAt(G, V, Hi) > Lead)
		{
			return Hi;
		}
		// The lead falls as the speed rises: the slowest that fits.
		for (int32 n = 0; n < 12; ++n)
		{
			const double Mid = 0.5 * (Lo + Hi);
			(LiftLeadAt(G, V, Mid) <= Lead ? Hi : Lo) = Mid;
		}
		return Hi;
	}

	double FPlanner::PlantTime(int32 Leg, const FLegGeom& G, double V) const
	{
		// From over its profile to down on the far surface: the command tracks
		// the profile a look ahead (it may come down as the terrain under the
		// wheel falls away) until the wheel may land, then goes to the landing
		// height; the wheel follows a lag behind. Seconds from the window's end
		// until it touches, and settles.
		const int32 Key = FMath::RoundToInt(V / TimingSpeedStep);
		if (const double* Found = G.Plant.Find(Key))
		{
			return *Found;
		}
		V = Key * TimingSpeedStep;
		const double Tau = FMath::Max(1e-3, static_cast<double>(T.ServoLagSeconds));
		const double Speed = G.SpeedDown > 0.0 ? G.SpeedDown : FMath::Max(1.0, static_cast<double>(T.LegSpeed));
		const double Accel = FMath::Max(1.0, static_cast<double>(T.LegAcceleration));
		const double SLand = SAt(G.KDown);
		const double Touch = G.ZLand + M.Preload(Leg);
		const bool	 bMoving = V >= MinTimingSpeed;
		const double SStart = bMoving ? G.SDropFrom : SLand;
		const double TLand = bMoving ? (SLand - SStart) / V : 0.0;
		double		 C = G.Prof.At(SStart, Ds), Cv = 0.0, H = C;
		double		 Out = 8.0;
		for (double t = 0.0; t < TLand + 8.0; t += SimDt)
		{
			const double S = SStart + (bMoving ? V * t : 0.0);
			const double Target = t < TLand ? FMath::Max(G.ZLand, G.Prof.MaxOver(S, FMath::Min(SLand, S + V * SwingLookSeconds), Ds)) : G.ZLand;
			// Slowed near the ground as the executor sets it down (TouchdownSpeed).
			TrackStep(C, Cv, Target, FMath::Min(Speed, TouchdownSpeedCap(C - Touch, T)), Accel, SimDt);
			H += (C - H) * FMath::Min(1.0, SimDt / Tau);
			if (t >= TLand && H <= Touch + 0.05)
			{
				Out = FMath::Max(0.0, t - TLand) + LegSettleSeconds;
				break;
			}
		}
		G.Plant.Add(Key, Out);
		return Out;
	}

	void FPlanner::ComputeTiming(FPlan& P) const
	{
		for (int32 i = 0; i < N; ++i)
		{
			const FLegGeom& G = P.Geom[i];
			FWheelPlan&		W = P.Wheels[i];
			W.Leg = i;
			W.DUp = DUp[i];
			W.DDown = DDown[i];
			W.Eps = Eps[i];
			W.Lambda = Lambda[i];
			W.FromPlane = 0;
			W.ToPlane = 1;
			W.SClear = SAt(G.KUp);
			W.SLand = SAt(G.KDown);
			// Timed at the fastest the robot goes while the wheel is over its window
			// (from where its drop begins, for the landing): the faster it goes, the
			// sooner the wheel meets the face and the less time it has to come down
			// before the landing.
			double VUp = 0.0, VDrop = 0.0;
			for (int32 k = G.KUp; k <= G.KDown; ++k)
			{
				VUp = FMath::Max(VUp, P.V[k]);
			}
			for (int32 k = Index(G.SDropFrom); k <= G.KDown; ++k)
			{
				VDrop = FMath::Max(VDrop, P.V[k]);
			}
			// With BrakingDeceleration set: begun before the robot comes within its
			// braking cap's distance of the window with the wheel still down. The
			// executor caps the speed there as if the lift might not happen, and a
			// lift timed only to be clear by the window has it brake every
			// hand-off.
			W.TLift = LiftLead(G, VUp);
			if (T.BrakingDeceleration > 0.0f)
			{
				W.TLift = FMath::Max(W.TLift, BrakeLeadSeconds(VUp, DriveLagSeconds, T));
			}
			W.TLiftMin = W.TLift;
			W.TEase = 0.0;
			W.LiftSpeed = 0.0;
			W.TPlant = PlantTime(i, G, VDrop);
			W.TLoad = LoadSeconds;
			W.SLiftStart = MarchBack(P, W.SClear, W.TLift);
			W.SDown = MarchFwd(P, W.SLand, W.TPlant);
			W.SLoaded = MarchFwd(P, W.SDown, LoadSeconds);
			// A corner the drive stance holds up stays up when its lift would come
			// soon after the level-in anyway.
			W.bHeldFromStart = In.Legs[i].bOffGround && !M.IsCentre(i) && W.SLiftStart <= LevelFromS + HeldCornerLeadCm;
			if (W.bHeldFromStart)
			{
				W.SLiftStart = 0.0;
			}
		}
		ComputeAirMasks(P);
	}

	void FPlanner::ComputeAirMasks(FPlan& P) const
	{
		// A sample stands for the half sample either side of it: a wheel off the
		// ground anywhere in that is off the ground there (so that a lift and a
		// landing a fraction of a sample apart are not missed between samples).
		const double Pad = 0.5 * Ds;
		for (int32 k = 0; k < NumS; ++k)
		{
			const double S = SAt(k);
			uint32		 Air = 0, Far = 0, Touch = 0;
			for (int32 i = 0; i < N; ++i)
			{
				const FWheelPlan& W = P.Wheels[i];
				if (S + Pad >= W.SLiftStart && S - Pad < W.SLoaded)
				{
					Air |= 1u << i;
					Touch |= (S + Pad >= W.SDown ? 1u : 0u) << i;
				}
				else if (S - Pad >= W.SLoaded)
				{
					Far |= 1u << i;
				}
			}
			P.Air[k] = Air;
			P.Far[k] = Far;
			P.Touch[k] = Touch;
		}
	}

	double FPlanner::SecondsBetween(const FPlan& P, int32 KA, int32 KB) const
	{
		double Seconds = 0.0;
		for (int32 k = FMath::Max(0, KA); k < KB && k + 1 < NumS; ++k)
		{
			Seconds += StepSeconds(P, k + 1) + DwellSeconds(P, k);
		}
		return Seconds;
	}

	bool FPlanner::FixSeat(FPlan& P) const
	{
		// The seat as it can follow the plan's: from where the crossing begins
		// with it, toward each sample's chosen fore/aft no faster than it shifts
		// at the planned speed -- the plan's seat jumps where its home changes,
		// the seat takes its time getting there.
		P.SeatFix.Reset();
		const FModelSeat& Seat = In.Model.Seat;
		if (!Seat.bHas || !Seat.Shifts() || Seat.ShiftSpeedCmS <= 0.0 || P.SeatX.Num() != NumS || P.V.Num() != NumS)
		{
			return false;
		}
		const int32 K0 = FMath::Clamp(Index(LevelFromS), 0, NumS - 1);
		P.SeatFix.SetNumUninitialized(NumS);
		double X = P.SeatX[K0];
		for (int32 k = 0; k < NumS; ++k)
		{
			if (k > K0)
			{
				const double V = FMath::Max(MinTimingSpeed, FMath::Max(P.V[k], P.V[k - 1]));
				const double Step = Ds * Seat.ShiftSpeedCmS / V;
				X += FMath::Clamp(P.SeatX[k] - X, -Step, Step);
			}
			P.SeatFix[k] = X;
		}
		return true;
	}

	bool FPlanner::PlanSeat(FPlan& P) const
	{
		// With the drive under the seat's forward shifts where that verifies:
		// it times the crossing as the executor runs it.
		if (SeatDriveOn() && In.Model.Seat.ForeSlowDownCmS > 0.0)
		{
			FPlan Driven = P;
			if (PlanSeatPass(Driven, true))
			{
				P = MoveTemp(Driven);
				return true;
			}
			Tried.Add(FString::Printf(TEXT("%s: the seat with the drive under its shifts did not plan (%s); planned without"), *P.Name,
				Driven.FailWhy.IsEmpty() ? TEXT("its rounds did not settle") : *Driven.FailWhy));
		}
		return PlanSeatPass(P, false);
	}

	void FPlanner::DriveSeatPath(const FPlan& P, int32 K0, double X0, TConstArrayView<double> Lo, TConstArrayView<double> Hi, TArray<double>& OutPath,
		TArray<double>& OutFore) const
	{
		// The seat toward each choice at up to ShiftSpeedCmS, its speed changed
		// no faster than ShiftAccelCmS2 and slowed in time to stop at the choice
		// -- as the executor moves it -- over time at the drive's speed: under a
		// forward shift the executor slows the base by the seat's speed, by up
		// to ForeSlowDownCmS and no lower than the floor (FExecutor::Tick).
		// The speed the drive is slowed from is the plan's without that (the
		// last round's slowing is not slowed again).
		const FModelSeat&	  Seat = In.Model.Seat;
		const TArray<double>& VPlan = P.VNoFore.Num() == NumS ? P.VNoFore : P.V;
		const double		  Fast = Seat.ShiftSpeedCmS;
		const double		  Accel = Seat.ShiftAccelCmS2;
		auto				  Drive = [&](double V, double Rate) {
			 return Rate > 0.0 ? FMath::Min(V, FMath::Max(Floor, Cruise - FMath::Min(Rate, Seat.ForeSlowDownCmS))) : V;
		};
		OutPath.Init(X0, NumS);
		OutFore.Init(NoCap, NumS);
		double Rate = 0.0;
		for (int32 k = K0 + 1; k < NumS; ++k)
		{
			const double Prev = OutPath[k - 1];
			const double V = FMath::Max(MinTimingSpeed, FMath::Max(VPlan[k], VPlan[k - 1]));
			const double Dt = Ds / FMath::Max(MinTimingSpeed, Drive(V, Rate)) + DwellSeconds(P, k);
			const double Togo = P.SeatX[k] - Prev;
			double		 Next = Prev + FMath::Clamp(Togo, -Fast * Dt, Fast * Dt);
			if (Accel > 0.0)
			{
				const double Goal = FMath::Sign(Togo) * FMath::Min(Fast, FMath::Sqrt(2.0 * Accel * FMath::Abs(Togo)));
				const double R = Rate + FMath::Clamp(Goal - Rate, -Accel * Dt, Accel * Dt);
				Next = R * Togo >= 0.0 && FMath::Abs(R * Dt) >= FMath::Abs(Togo) ? P.SeatX[k] : Prev + R * Dt;
			}
			// Within the bands to come, and the seat's own speed.
			const double L = FMath::Max(Lo[k], Prev - Fast * Dt);
			const double H = FMath::Min(Hi[k], Prev + Fast * Dt);
			Next = FMath::Clamp(Next, L, FMath::Max(L, H));
			Rate = (Next - Prev) / Dt;
			OutPath[k] = Next;
			if (Rate > 1e-6)
			{
				OutFore[k] = Drive(Cruise, Rate);
			}
		}
	}

	bool FPlanner::PlanSeatPass(FPlan& P, bool bDrive) const
	{
		// The seat as it can follow the free seat's choices -- toward where
		// each instant wants it, no faster than ShiftSpeedCmS at the planned
		// speed, never ahead of it: forward once the fronts are across, aft
		// again once every wheel is, as the crossing's seat policy has it --
		// and no further along any move than it can come back from in time for
		// the samples after (each sample's SeatBand, the fore/aft the robot
		// stands with there). Where two samples' bands are nearer in time than
		// the seat can shift between them, that stretch is slowed (SeatCap) and
		// the speed, heights and bands found again. Moving the seat early
		// instead, to arrive with each choice, took the margin to its limit with
		// the fronts in the air and brought the mass back while the last axle
		// was still up: both tipped or turned the robot in PIE.
		const FModelSeat& Seat = In.Model.Seat;
		P.bSeatPlanned = false;
		if (!Seat.bHas || !Seat.Shifts() || Seat.ShiftSpeedCmS <= 0.0 || P.SeatX.Num() != NumS || P.SeatBand.Num() != NumS
			|| P.SeatCap.Num() != NumS || P.V.Num() != NumS)
		{
			return false;
		}
		const int32 K0 = FMath::Clamp(Index(LevelFromS), 0, NumS - 1);
		auto		Reverify = [&](bool bWithSeat) {
			   for (EFail& F : P.Fail)
			   {
				   if (F == EFail::Verify)
				   {
					   F = EFail::None;
				   }
			   }
			   P.WhyAt.Reset();
			   P.FailWhy.Reset();
			   if (!bWithSeat)
			   {
				   P.SeatFix.Reset();
			   }
			   Verify(P);
			   P.FirstFail = FirstFailOf(P);
			   return IsOk(P);
		};
		// The bands and choices are the free seat's at these speeds.
		if (!Reverify(false))
		{
			return false;
		}
		int32 DriveRounds = 0;
		for (int32 Round = 0; Round < SeatPathRounds + (bDrive ? SeatDriveRounds : 0) && !Stopped(); ++Round)
		{
			// How far the seat may move from sample k - 1 to k.
			TArray<double> Step;
			Step.Init(0.0, NumS);
			for (int32 k = K0 + 1; k < NumS; ++k)
			{
				const double V = FMath::Max(MinTimingSpeed, FMath::Max(P.V[k], P.V[k - 1]));
				Step[k] = Seat.ShiftSpeedCmS * (Ds / V + DwellSeconds(P, k));
			}
			// Backward: where the seat must be at each sample to make its own band
			// and every later one's, and which sample's band sets each bound.
			const double   Open = 1e9;
			TArray<double> Lo, Hi;
			TArray<int32>  LoBy, HiBy;
			Lo.Init(-Open, NumS);
			Hi.Init(Open, NumS);
			LoBy.Init(INDEX_NONE, NumS);
			HiBy.Init(INDEX_NONE, NumS);
			for (int32 k = NumS - 1; k >= K0; --k)
			{
				const FVector2D& B = P.SeatBand[k];
				if (B.X <= B.Y)
				{
					Lo[k] = B.X;
					Hi[k] = B.Y;
					LoBy[k] = HiBy[k] = k;
				}
				if (k + 1 < NumS)
				{
					if (Lo[k + 1] - Step[k + 1] > Lo[k])
					{
						Lo[k] = Lo[k + 1] - Step[k + 1];
						LoBy[k] = LoBy[k + 1];
					}
					if (Hi[k + 1] + Step[k + 1] < Hi[k])
					{
						Hi[k] = Hi[k + 1] + Step[k + 1];
						HiBy[k] = HiBy[k + 1];
					}
				}
			}
			// Where the seat starts, and the first sample whose bounds cross.
			const double X0 = P.SeatX[K0];
			int32		 Bad = INDEX_NONE;
			for (int32 k = K0; k < NumS && Bad == INDEX_NONE; ++k)
			{
				if (Lo[k] > Hi[k] + 1e-6)
				{
					Bad = k;
				}
			}
			const bool bStartOut = Bad == INDEX_NONE && (X0 < Lo[K0] - 1e-6 || X0 > Hi[K0] + 1e-6);
			if (Bad == INDEX_NONE && !bStartOut)
			{
				// Forward: toward each choice, within the shift and the bounds.
				TArray<double> Path;
				if (bDrive)
				{
					// The seat as the executor moves it, and the drive under it. Timed
					// again at that drive until it settles: the lifts after the fronts'
					// landing take less of the travel, the landing more, and the seat
					// waits for the landing.
					TArray<double> Fore;
					DriveSeatPath(P, K0, X0, Lo, Hi, Path, Fore);
					bool   bSettled = true;
					double Change = 0.0, Least = Cruise;
					int32  KA = INDEX_NONE, KB = INDEX_NONE;
					for (int32 k = 0; k < NumS; ++k)
					{
						const double D = FMath::Abs(FMath::Min(Fore[k], Cruise) - FMath::Min(P.ForeCap[k], Cruise));
						bSettled = bSettled && D <= SeatDriveSettledCmS;
						Change = FMath::Max(Change, D);
						if (Fore[k] < NoCap)
						{
							KA = KA == INDEX_NONE ? k : KA;
							KB = k;
							Least = FMath::Min(Least, Fore[k]);
						}
					}
					if (PlanDebug())
					{
						Tried.Add(FString::Printf(TEXT("%s: the seat with the drive under it, round %d: the seat forward from %.0f to %.0f cm, the drive "
													   "down to %.1f cm/s; %.1f cm/s off the last round's"),
							*P.Name, DriveRounds, KA == INDEX_NONE ? 0.0 : SAt(KA), KB == INDEX_NONE ? 0.0 : SAt(KB), Least, Change));
					}
					if (!bSettled)
					{
						// Timed at this round's drive: with rounds to go, the seat found
						// again at it; on the last, this seat kept where it verifies at it
						// (the rounds can swing a sample or two about where the seat
						// sets off).
						P.ForeCap = MoveTemp(Fore);
						SpeedEnvelope(P);
						P.bHeightOk = HeightPath(P);
						if (++DriveRounds < SeatDriveRounds)
						{
							if (!Reverify(false))
							{
								return false;
							}
							continue;
						}
					}
				}
				else
				{
					Path.Init(X0, NumS);
					for (int32 k = K0 + 1; k < NumS; ++k)
					{
						const double Prev = Path[k - 1];
						const double L = FMath::Max(Lo[k], Prev - Step[k]);
						const double H = FMath::Min(Hi[k], Prev + Step[k]);
						Path[k] = FMath::Clamp(Prev + FMath::Clamp(P.SeatX[k] - Prev, -Step[k], Step[k]), L, FMath::Max(L, H));
					}
				}
				P.SeatEndX = P.SeatX[NumS - 1];
				P.SeatFix = MoveTemp(Path);
				if (!Reverify(true))
				{
					P.SeatFix.Reset();
					return false;
				}
				P.bSeatPlanned = true;
				// The seat's own slowing (SeatCap) gives it the time to shift between
				// two bands; with the drive under it the seat gets there sooner (or
				// shifts before the stretch at all), and where it stands still the
				// drive need not wait for it: released where the plan still verifies.
				if (bDrive)
				{
					const TArray<double> Kept = P.SeatCap;
					bool				 bReleased = false;
					for (int32 k = K0 + 1; k < NumS; ++k)
					{
						const bool bStill = FMath::Abs(P.SeatFix[k] - P.SeatFix[k - 1]) <= 1e-3
							&& (k + 1 >= NumS || FMath::Abs(P.SeatFix[k + 1] - P.SeatFix[k]) <= 1e-3);
						if (P.SeatCap[k] < NoCap && bStill)
						{
							P.SeatCap[k] = NoCap;
							bReleased = true;
						}
					}
					if (bReleased)
					{
						SpeedEnvelope(P);
						P.bHeightOk = HeightPath(P);
						if (!Reverify(true))
						{
							P.SeatCap = Kept;
							SpeedEnvelope(P);
							P.bHeightOk = HeightPath(P);
							if (!Reverify(true))
							{
								P.SeatFix.Reset();
								P.bSeatPlanned = false;
								return false;
							}
						}
					}
				}
				return true;
			}
			// Two bands too near in time for the seat to shift between them (or
			// the start and a band): the one that holds it forward of From, the
			// other aft of To, the stretch between slowed so that it can.
			int32  KA = K0, KB = K0;
			double From = X0, To = X0;
			if (bStartOut)
			{
				const bool bBelow = X0 < Lo[K0];
				KA = K0;
				KB = bBelow ? LoBy[K0] : HiBy[K0];
				From = X0;
				To = bBelow ? Lo[KB] : Hi[KB];
			}
			else
			{
				const int32 A = LoBy[Bad], B = HiBy[Bad];
				if (A == INDEX_NONE || B == INDEX_NONE)
				{
					return false;
				}
				KA = FMath::Min(A, B);
				KB = FMath::Max(A, B);
				From = P.SeatBand[B].Y;
				To = P.SeatBand[A].X;
			}
			const double Dist = FMath::Abs(To - From);
			if (KB <= KA || Dist <= 1e-6)
			{
				return false;
			}
			double Dwell = 0.0;
			for (int32 k = KA + 1; k <= KB; ++k)
			{
				Dwell += DwellSeconds(P, k);
			}
			const double Run = (KB - KA) * Ds;
			const double Seconds = Dist / Seat.ShiftSpeedCmS - Dwell;
			if (Seconds <= 1e-6)
			{
				return false;
			}
			const double Cap = 0.95 * Run / Seconds;
			bool		 bNew = false;
			for (int32 k = KA; k <= KB; ++k)
			{
				if (P.SeatCap[k] > Cap + 1e-6)
				{
					P.SeatCap[k] = Cap;
					bNew = true;
				}
			}
			if (!bNew)
			{
				return false;
			}
			Tried.Add(FString::Printf(TEXT("%s: the seat's shift (%.1f cm, %+.1f to %+.1f, between %.0f and %.0f cm) slows them to %.1f cm/s"), *P.Name,
				Dist, From, To, SAt(KA), SAt(KB), Cap));
			SpeedEnvelope(P);
			P.bHeightOk = HeightPath(P);
			if (!Reverify(false))
			{
				return false;
			}
		}
		return false;
	}

	bool FPlanner::EaseLifts(FPlan& P) const
	{
		const double MaxEase = static_cast<double>(T.LiftEaseSeconds);
		if (MaxEase <= 0.0 || P.Wheels.Num() != N || P.Geom.Num() != N || P.Air.Num() != NumS)
		{
			return false;
		}
		// Whether the robot stands with a lift moved earlier is asked with the
		// seat where it can be by then, not where the instant would put it: the
		// seat swung forward once the fronts carry takes a second or more to get
		// there, and a lift eased onto its arrival was refused by the measured
		// margin at the executor.
		const bool bPlanned = P.bSeatPlanned && P.SeatFix.Num() == NumS;
		if (!bPlanned)
		{
			FixSeat(P);
		}
		ON_SCOPE_EXIT
		{
			if (!bPlanned)
			{
				P.SeatFix.Reset();
			}
		};
		const int32 K0 = Index(LevelFromS);
		// In the order they lift: an eased lift is in the masks the next is
		// checked against.
		TArray<int32> Order;
		for (int32 i = 0; i < N; ++i)
		{
			Order.Add(i);
		}
		Order.Sort([&P](int32 A, int32 B) { return P.Wheels[A].SLiftStart < P.Wheels[B].SLiftStart; });
		bool bAny = false;
		for (const int32 i : Order)
		{
			FWheelPlan&		W = P.Wheels[i];
			const FLegGeom& G = P.Geom[i];
			if (W.bHeldFromStart || G.ZTop <= G.ZPlant + 1e-6 || W.SLiftStart <= LevelFromS)
			{
				continue;
			}
			const int32 KStart = Index(W.SLiftStart);
			// No earlier than: the level-in's end (and the room the executor brakes
			// it short of the first lift by); the loads of the wheels its lift waits
			// on (those landing before it with which, both up, the robot would not
			// stand: Output's must-be-down set); and MaxEase before its start.
			double SMin = FMath::Max(LevelFromS + EaseLevelInPadCm, MarchBack(P, W.SLiftStart, MaxEase));
			for (int32 j = 0; j < N; ++j)
			{
				const FWheelPlan& Wj = P.Wheels[j];
				if (j == i || Wj.SLoaded > W.SLiftStart)
				{
					continue;
				}
				const uint32 Set = P.Air[KStart] | (1u << i) | (1u << j);
				const uint32 Far = P.Far[KStart] & ~(1u << j);
				if (!Stands(P, Query(P, KStart, Set, Far), BGuessAt(P, KStart)))
				{
					SMin = FMath::Max(SMin, Wj.SLoaded + EventPadCm);
				}
			}
			// Back from its start, sample by sample, while the robot stands with it
			// up as well.
			int32 KE = KStart;
			for (int32 k = KStart - 1; k > K0 && SAt(k) >= SMin - 1e-9; --k)
			{
				const uint32 Air = P.Air[k] | (1u << i);
				if (IsForbiddenSet(In.Model, Air) || !Stands(P, Query(P, k, Air, P.Far[k]), BGuessAt(P, k)))
				{
					break;
				}
				KE = k;
			}
			if (KE >= KStart)
			{
				continue;
			}
			double VUp = 0.0;
			for (int32 k = G.KUp; k <= G.KDown && k < NumS; ++k)
			{
				VUp = FMath::Max(VUp, P.V[k]);
			}
			// The lead from there, the slowest rise that still clears the window
			// with it -- and, where that rise is the least allowed, only the lead
			// it needs.
			const double Lead = SecondsBetween(P, KE, Index(W.SClear));
			const double Speed = SlowestLiftSpeed(G, VUp, Lead);
			double		 Need = LiftLeadAt(G, VUp, Speed);
			if (T.BrakingDeceleration > 0.0f)
			{
				Need = FMath::Max(Need, BrakeLeadSeconds(VUp, DriveLagSeconds, T));
			}
			double SE = SAt(KE), TL = Lead;
			if (Need < Lead - 1e-6)
			{
				SE = FMath::Max(SE, MarchBack(P, W.SClear, Need));
				TL = Need;
			}
			if (SE >= W.SLiftStart - 0.5 * Ds)
			{
				continue;
			}
			W.TEase = TL - W.TLiftMin;
			W.TLift = TL;
			W.SLiftStart = SE;
			W.LiftSpeed = Speed;
			// A corner carried up in the drive stance whose eased lift begins at the
			// level-in's end stays up through it rather than being set down there and
			// lifted again.
			if (In.Legs[i].bOffGround && !M.IsCentre(i) && SE <= LevelFromS + EaseLevelInPadCm + Ds)
			{
				W.bHeldFromStart = true;
				W.SLiftStart = 0.0;
				W.LiftSpeed = 0.0;
			}
			ComputeAirMasks(P);
			bAny = true;
		}
		if (bAny)
		{
			ComputeStatic(P);
		}
		return bAny;
	}

	void FPlanner::OutputEased(const FPlan& Plan, EPlanClass Class, FCurbSchedule& Out) const
	{
		// The seat planned at its own speed, where a path verifies; else each
		// instant's own choice, as before.
		FPlan Seated;
		if ((Class == EPlanClass::Level || Class == EPlanClass::Tilt) && SeatPathOn() && In.Model.Seat.Shifts())
		{
			Seated = Plan;
			if (!PlanSeat(Seated))
			{
				Seated = FPlan();
			}
		}
		const FPlan& P = Seated.bSeatPlanned ? Seated : Plan;
		if (Class != EPlanClass::Refused && T.LiftEaseSeconds > 0.0f)
		{
			FPlan E = P;
			if (EaseLifts(E))
			{
				for (EFail& F : E.Fail)
				{
					F = EFail::None;
				}
				E.WhyAt.Reset();
				E.FailWhy.Reset();
				E.bHeightOk = HeightPath(E);
				Verify(E);
				E.FirstFail = FirstFailOf(E);
				if (IsOk(E))
				{
					Output(E, Class, Out);
					return;
				}
			}
		}
		Output(P, Class, Out);
	}

	FInstantQuery FPlanner::Query(const FPlan& P, int32 K, uint32 Air, uint32 Far) const
	{
		FInstantQuery Q;
		Q.S = SAt(K);
		Q.PitchDeg = P.Pitch[K];
		Q.RollDeg = P.Roll[K];
		Q.Airborne = Air;
		Q.OnFar = Far;
		Q.Touching = (P.Touch.IsValidIndex(K) ? P.Touch[K] : 0u) & Air;
		for (int32 Side = 0; Side < 2; ++Side)
		{
			if (Centre[Side] != INDEX_NONE)
			{
				Q.X[Centre[Side]] = P.X[Side][K];
			}
		}
		if (P.SeatFix.IsValidIndex(K))
		{
			Q.SeatX = P.SeatFix[K];
		}
		return Q;
	}

	uint64 FPlanner::InstantKey(const FInstantQuery& Q, double BGuess) const
	{
		// Everything the instant depends on, quantised finer than any of it
		// matters: its travel, attitude, sets, the centre wheels' fore/aft, the
		// chassis height it is guessed at and the seat, where it is held.
		int64 Key[8] = { FMath::RoundToInt64(Q.S * 100.0),
			FMath::RoundToInt64(Q.PitchDeg * 1000.0) * 100000 + FMath::RoundToInt64(Q.RollDeg * 1000.0),
			(static_cast<int64>(Q.Airborne) << 32) | Q.OnFar, 0, 0, FMath::RoundToInt64(BGuess * 100.0), Q.Touching,
			IsSeatFree(Q.SeatX) ? TNumericLimits<int64>::Min() : FMath::RoundToInt64(Q.SeatX * 100.0) };
		for (int32 Side = 0; Side < 2; ++Side)
		{
			Key[3 + Side] = Centre[Side] != INDEX_NONE ? FMath::RoundToInt64(Q.X[Centre[Side]] * 100.0) : 0;
		}
		return CityHash64(reinterpret_cast<const char*>(Key), sizeof(Key));
	}

	bool FPlanner::Stands(const FPlan& P, const FInstantQuery& Q, double BGuess) const
	{
		const uint64 Hash = InstantKey(Q, BGuess);
		if (const bool* Found = P.Memo.Find(Hash))
		{
			return *Found;
		}
		const bool bOk = Feasible(Q, BGuess);
		P.Memo.Add(Hash, bOk);
		return bOk;
	}

	bool FPlanner::Feasible(const FInstantQuery& Q, double BGuess, FInstantResult* Out) const
	{
		++CountFeasible;
		++Work;
		FInstantResult R = M.Evaluate(Q, BGuess);
		// The corners' fore/aft were taken at the guess; where the corridor that
		// gives sits well away from it, take them again at its middle.
		for (int32 Again = 0; Again < 2 && R.Lo <= R.Hi && FMath::Abs(R.B - BGuess) > 0.5; ++Again)
		{
			const bool bOk0 = R.Hi - R.Lo >= T.MinCorridorCm - 1e-9 && R.bReach && R.Margin >= Req - 1e-9;
			if (bOk0)
			{
				break;
			}
			BGuess = R.B;
			++Work;
			const FInstantResult R2 = M.Evaluate(Q, BGuess);
			if (R2.Hi - R2.Lo < R.Hi - R.Lo - 1e-9)
			{
				break;
			}
			R = R2;
		}
		const double Width = R.Hi - R.Lo;
		// A set the gait never lifts together does not stand, whatever its margin.
		const bool bAllowed = !IsForbiddenSet(In.Model, Q.Airborne);
		bool	   bOk = bAllowed && Width >= T.MinCorridorCm - 1e-9 && R.bReach && R.Margin >= Req - 1e-9;
		if (bAllowed && !bOk && Width >= T.MinCorridorCm - 1e-9 && (!R.bReach || R.Margin > Req - HopelessMarginCm))
		{
			// Somewhere else in the corridor, perhaps: the margin moves with the
			// corners' fore/aft and the centre of mass's height.
			const double Hi = FMath::Min(R.Hi, R.Lo + 60.0);
			for (int32 p = 0; p <= MarginProbes && !bOk; ++p)
			{
				FInstantResult Rp = R;
				M.PoseAt(Q, R.Lo + 0.25 + (Hi - R.Lo - 0.5) * p / MarginProbes, Rp);
				if (Rp.bReach && (!R.bReach || Rp.Margin > R.Margin))
				{
					R = Rp;
					bOk = R.Margin >= Req - 1e-9;
				}
			}
		}
		if (bOk)
		{
			// The margin must hold over MinCorridorCm of chassis heights, as the
			// chassis height's path asks (HeightPath), not at one height only:
			// about the pose found, else about another in the corridor.
			const double Half = 0.5 * T.MinCorridorCm;
			auto		 HoldsAt = [&](double B) {
				FInstantResult X = R;
				M.PoseAt(Q, B, X);
				return X.bReach && X.Margin >= Req - 1e-9;
			};
			if (!(HoldsAt(R.B - Half) && HoldsAt(R.B + Half)))
			{
				bOk = false;
				const double Lo = R.Lo + Half, Hi = FMath::Min(R.Hi, R.Lo + 60.0) - Half;
				for (int32 p = 0; p <= MarginProbes && !bOk && Hi >= Lo; ++p)
				{
					const double C = Lo + (Hi - Lo) * p / MarginProbes;
					if (HoldsAt(C - Half) && HoldsAt(C) && HoldsAt(C + Half))
					{
						M.PoseAt(Q, C, R);
						bOk = true;
					}
				}
			}
		}
		if (Out)
		{
			*Out = R;
		}
		return bOk;
	}

	void FPlanner::SpeedEnvelope(FPlan& P) const
	{
		const double Started = FPlatformTime::Seconds();
		const int32	 K0 = Index(LevelFromS);
		// From scratch for these paths: a cap another path needed may not be
		// needed now.
		for (int32 k = 0; k < NumS; ++k)
		{
			P.VCap[k] = FreeCap(k);
			P.CapBy[k] = INDEX_NONE;
		}
		ApplyLevelInCap(P);
		// What the planned tilt's easings need.
		for (int32 k = 0; k < NumS && P.TiltCap.Num() == NumS; ++k)
		{
			if (P.VCap[k] > P.TiltCap[k])
			{
				P.VCap[k] = P.TiltCap[k];
				P.CapBy[k] = CapTilt;
			}
		}
		// What the seat's shift needs (PlanSeat).
		for (int32 k = 0; k < NumS && P.SeatCap.Num() == NumS; ++k)
		{
			if (P.VCap[k] > P.SeatCap[k])
			{
				P.VCap[k] = P.SeatCap[k];
				P.CapBy[k] = CapSeat;
			}
		}
		// A centre wheel walks within CenterShiftSpeed: no faster travel than
		// lets it cover its walk (a planned stop's re-pose is made at rest).
		const double CSS = FMath::Max(0.1, static_cast<double>(T.CenterShiftSpeed));
		for (int32 Side = 0; Side < 2; ++Side)
		{
			for (int32 k = 1; Centre[Side] != INDEX_NONE && k < NumS; ++k)
			{
				const double Dx = FMath::Abs(P.X[Side][k] - P.X[Side][k - 1]);
				if (Dx > 1e-9 && !IsDwell(P, k))
				{
					for (const int32 j : { k - 1, k })
					{
						if (P.VCap[j] > Ds * CSS / Dx)
						{
							P.VCap[j] = Ds * CSS / Dx;
							P.CapBy[j] = CapWalk;
						}
					}
				}
			}
		}
		// The drive under the planned seat's forward shifts (PlanSeat): every
		// lead and tail about them is timed at the speed the base will run.
		// The caps without it are kept beside (each cap below goes on both), for
		// the speed without it.
		TArray<double> CapNoFore;
		for (int32 k = 0; k < NumS && P.ForeCap.Num() == NumS; ++k)
		{
			if (P.VCap[k] > P.ForeCap[k])
			{
				if (CapNoFore.Num() == 0)
				{
					CapNoFore = P.VCap;
				}
				P.VCap[k] = P.ForeCap[k];
				P.CapBy[k] = CapFore;
			}
		}
		const bool bFore = CapNoFore.Num() == NumS;
		// The legs' geometry with these paths; which corners stay up from the
		// start depends on the timing, and what stands standing still on that.
		ComputeGeometry(P);
		Passes(P);
		ComputeTiming(P);
		ComputeStatic(P);
		// What each sample was last found with (its sets off the ground and on
		// the far side): within this call the paths do not change, so a sample
		// whose sets did not either needs no second look.
		TArray<uint64> Checked;
		TArray<uint8>  CheckedOk;
		Checked.Init(TNumericLimits<uint64>::Max(), NumS);
		CheckedOk.Init(0, NumS);
		for (int32 Round = 0; Round < MaxSpeedRounds; ++Round)
		{
			Passes(P);
			ComputeTiming(P);
			bool bNew = false;
			// Caps the stretch [A, B] at Cap and times the plan again at once (the
			// samples ahead see this cap's leads and tails, so one pass settles a
			// stretch). A cap under the floor tries the floor first: where sample K
			// stands with it, the floor it is. True when it stays under.
			auto CapStretch = [&](int32 A, int32 B, double Cap, int32 K, int32 By) {
				// The cap is worked out as if the speed were constant over the lead or
				// tail (and the leg's times did not change with it): where the stretch
				// is already that slow and the sample still fails, a little slower.
				double Most = 0.0;
				for (int32 j = FMath::Max(0, A); j <= FMath::Min(NumS - 1, B); ++j)
				{
					Most = FMath::Max(Most, P.VCap[j]);
				}
				if (Cap >= Most - 1e-6)
				{
					Cap = 0.97 * Most;
				}
				auto Apply = [&](double C) {
					for (int32 j = FMath::Max(0, A); j <= FMath::Min(NumS - 1, B); ++j)
					{
						if (P.VCap[j] > C + 1e-6)
						{
							P.VCap[j] = C;
							P.CapBy[j] = By;
							bNew = true;
						}
						if (bFore)
						{
							CapNoFore[j] = FMath::Min(CapNoFore[j], C);
						}
					}
					Passes(P);
					ComputeTiming(P);
				};
				if (Cap >= Floor - 1e-6)
				{
					Apply(Cap);
					return false;
				}
				Apply(FMath::Min(Cruise, Floor));
				if (Stands(P, Query(P, K, P.Air[K], P.Far[K]), BGuessAt(P, K)))
				{
					return false;
				}
				Apply(Cap);
				return true;
			};
			for (int32 k = 0; k < NumS; ++k)
			{
				P.Fail[k] = P.StaticOk[k] || k < K0 ? EFail::None : EFail::Static;
			}
			// No lift begins before the level-in is done: slow enough that its
			// lead fits after it.
			for (int32 i = 0; i < N; ++i)
			{
				const FWheelPlan& W = P.Wheels[i];
				if (!W.bHeldFromStart && W.SLiftStart < LevelFromS - 1e-6 && W.SClear > LevelFromS)
				{
					const double Cap = 0.97 * (W.SClear - LevelFromS) / FMath::Max(1e-3, W.TLift);
					for (int32 j = Index(LevelFromS); j <= Index(W.SClear); ++j)
					{
						if (P.VCap[j] > Cap + 1e-6)
						{
							P.VCap[j] = Cap;
							P.CapBy[j] = CapAfterLevelIn + i;
							bNew = true;
						}
						if (bFore)
						{
							CapNoFore[j] = FMath::Min(CapNoFore[j], Cap);
						}
					}
				}
			}
			for (int32 k = K0; k < NumS; ++k)
			{
				if (!P.StaticOk[k] || P.Air[k] == P.StaticAir[k])
				{
					continue;
				}
				const uint64 Key = (static_cast<uint64>(P.Air[k]) << 32) | P.Far[k];
				if (Checked[k] != Key)
				{
					Checked[k] = Key;
					CheckedOk[k] = Stands(P, Query(P, k, P.Air[k], P.Far[k]), BGuessAt(P, k)) ? 1 : 0;
				}
				if (CheckedOk[k])
				{
					continue;
				}
				const double S = SAt(k);
				// Up here only for the speed: a wheel lifting ahead of its window
				// (its lead) or landing after it (its tail). Slowing to v_w over its
				// lead or tail takes it off the ground here. Of those whose absence
				// lets the robot stand, the one that slows least -- a local gap cap
				// (a gap cap): where the centre wheels stand so that a
				// pair may be up together, they may overlap there.
				double Best = -1.0;
				int32  BestW = INDEX_NONE;
				bool   bBestLead = false;
				for (int32 i = 0; i < N; ++i)
				{
					if (!((P.Air[k] >> i) & 1u) || ((P.StaticAir[k] >> i) & 1u))
					{
						continue;
					}
					const FWheelPlan& W = P.Wheels[i];
					const bool		  bLead = S < W.SClear;
					const double	  V = bLead ? (W.SClear - S) / FMath::Max(1e-3, W.TLift) : (S - W.SLand) / FMath::Max(1e-3, W.TPlant + LoadSeconds);
					if (V <= Best)
					{
						continue;
					}
					// Pointless where the wheel's own window has no pose standing
					// still: that fails there whatever the speed.
					bool bStaticBad = false;
					for (int32 j = P.Geom[i].KUp; j < P.Geom[i].KDown && !bStaticBad; ++j)
					{
						bStaticBad = !P.StaticOk[j];
					}
					if (bStaticBad)
					{
						continue;
					}
					const uint32 Set = P.Air[k] & ~(1u << i);
					const uint32 SetFar = P.Far[k] | (bLead ? 0u : (1u << i));
					if (Stands(P, Query(P, k, Set, SetFar), BGuessAt(P, k)))
					{
						Best = V;
						BestW = i;
						bBestLead = bLead;
					}
				}
				if (BestW != INDEX_NONE)
				{
					const FWheelPlan& WP = P.Wheels[BestW];
					if (CapStretch(bBestLead ? k : Index(WP.SLand), bBestLead ? Index(WP.SClear) : k, 0.97 * Best, k, BestW))
					{
						P.Fail[k] = EFail::Slow;
					}
					continue;
				}
				// Otherwise: the wheels up here only for the speed, added slowest
				// first until the set fails; that one's speed is the cap, over the
				// stretch its lead or tail runs.
				struct FInduced
				{
					int32  Leg;
					double V;
					bool   bLead;
				};
				TArray<FInduced, TInlineAllocator<MaxPlanLegs>> Induced;
				for (int32 i = 0; i < N; ++i)
				{
					if (!((P.Air[k] >> i) & 1u) || ((P.StaticAir[k] >> i) & 1u))
					{
						continue;
					}
					const FWheelPlan& W = P.Wheels[i];
					Induced.Add(S < W.SClear ? FInduced{ i, (W.SClear - S) / FMath::Max(1e-3, W.TLift), true }
											 : FInduced{ i, (S - W.SLand) / FMath::Max(1e-3, W.TPlant + LoadSeconds), false });
				}
				Induced.Sort([](const FInduced& A, const FInduced& B) { return A.V < B.V; });
				uint32 Set = P.StaticAir[k], SetFar = P.StaticFar[k];
				for (const FInduced& W : Induced)
				{
					Set |= 1u << W.Leg;
					SetFar &= ~(1u << W.Leg);
					if (Stands(P, Query(P, k, Set, SetFar), BGuessAt(P, k)))
					{
						continue;
					}
					// Pointless where the wheel's own window has no pose standing
					// still: that fails there whatever the speed.
					const FLegGeom& G = P.Geom[W.Leg];
					bool			bStaticBad = false;
					for (int32 j = G.KUp; j < G.KDown && !bStaticBad; ++j)
					{
						bStaticBad = !P.StaticOk[j];
					}
					if (bStaticBad)
					{
						break;
					}
					const FWheelPlan& WP = P.Wheels[W.Leg];
					if (CapStretch(W.bLead ? k : Index(WP.SLand), W.bLead ? Index(WP.SClear) : k, FMath::Max(0.0, 0.97 * W.V), k, W.Leg))
					{
						P.Fail[k] = EFail::Slow;
					}
					break;
				}
			}
			if (!bNew)
			{
				break;
			}
		}
		Passes(P);
		ComputeTiming(P);
		// What the caps left: a sample whose sets at the planned speed still do
		// not stand fails for the speed (the caps could not part them).
		for (int32 k = K0; k < NumS; ++k)
		{
			if (P.Fail[k] == EFail::None && P.StaticOk[k] && P.Air[k] != P.StaticAir[k]
				&& !Stands(P, Query(P, k, P.Air[k], P.Far[k]), BGuessAt(P, k)))
			{
				P.Fail[k] = EFail::Slow;
			}
		}
		// The speed without the drive's slowing under the seat: the seat's shift
		// makes that up for the rider.
		P.VNoFore.Reset();
		if (bFore)
		{
			TArray<double> V = P.V;
			Swap(P.VCap, CapNoFore);
			Passes(P);
			Swap(P.VNoFore, P.V);
			Swap(P.VCap, CapNoFore);
			P.V = MoveTemp(V);
		}
		// The least speed once risen from rest -- from where the speed first
		// stops rising -- planned stops apart (the drive's slowing under the
		// seat apart).
		const TArray<double>& VRef = bFore ? P.VNoFore : P.V;
		P.VFloor = Cruise;
		const int32			KRise = RiseEnd(P);
		const TArray<uint8> AtStop = StopRuns(P);
		for (int32 k = KRise; k < NumS; ++k)
		{
			if (!AtStop[k])
			{
				P.VFloor = FMath::Min(P.VFloor, VRef[k]);
			}
			// Under the floor fails unless a slow crossing is allowed; stopped
			// (StopSpeedCmS) fails anywhere but at a planned stop.
			if (k >= K0 && P.Fail[k] == EFail::None && !AtStop[k]
				&& (P.V[k] <= StopSpeedCmS + 1e-6 || (P.V[k] < Floor - 1e-6 && !T.bAllowSlowCrossing)))
			{
				P.Fail[k] = EFail::Slow;
			}
		}
		TimeSpeed += FPlatformTime::Seconds() - Started;
	}

	bool FPlanner::CentrePathsKnots(FPlan& P, double XF, double XLift, double XLand, double XR, bool bSmooth, bool bPerSide) const
	{
		// Each centre wheel stands at least XF (cm past where it began) forward
		// while either front corner is off the ground, at least -XR aft while
		// either rear one is; it heads for XLift as it comes to lift and for
		// XLand as it lands; elsewhere it stays where it is, as late as it can,
		// and goes home at the end -- within CenterShiftSpeed, a planted wheel
		// never walking aft faster than MaxAftWalkPerCm of the travel. It may
		// walk from the start. "Both front corners up" is where their stretches
		// off the ground overlap (at an angle, one lifts as the other lands), at
		// no less than the floor speed: a cap that parts them is not to be
		// planned around, the overlap the plan means to keep is.
		FPlan AtFloor = P;
		for (int32 k = 0; k < NumS; ++k)
		{
			AtFloor.VCap[k] = FMath::Max(AtFloor.VCap[k], FMath::Min(Cruise, Floor));
		}
		Passes(AtFloor);
		ComputeTiming(AtFloor);
		// And, for a smooth walk's holds, at the cruise speed: a centre wheel's
		// lift and landing fall somewhere between the two, wherever the plan's
		// speed puts them.
		const bool bHolds = bSmooth && WalkHoldCm() > 0.0;
		FPlan	   AtCruise;
		if (bHolds)
		{
			AtCruise = P;
			for (int32 k = 0; k < NumS; ++k)
			{
				AtCruise.VCap[k] = Cruise;
			}
			Passes(AtCruise);
			ComputeTiming(AtCruise);
		}
		// The stretches a front corner and a rear corner are off the ground
		// (travel, cm): over both corners of an axle, where their stretches
		// overlap (FrontB < FrontA: none); or, per side, that side's corner's
		// own, so that at an angle each centre wheel walks forward as its own
		// front goes over and aft as its own rear does.
		auto Windows = [&](int32 OnSide, double& FrontA, double& FrontB, double& RearA, double& RearB, double& RearC) {
			FrontA = RearA = RearC = -TNumericLimits<double>::Max();
			FrontB = RearB = TNumericLimits<double>::Max();
			int32 NumFront = 0, NumRear = 0;
			for (int32 i = 0; i < N; ++i)
			{
				if (M.IsCentre(i) || (OnSide >= 0 && M.Side(i) != OnSide))
				{
					continue;
				}
				const FWheelPlan& W = AtFloor.Wheels[i];
				if (M.Axle(i) == ERammsCurbAxle::Front)
				{
					FrontA = FMath::Max(FrontA, W.SLiftStart);
					FrontB = FMath::Min(FrontB, W.SLoaded);
					++NumFront;
				}
				else if (M.Axle(i) == ERammsCurbAxle::Rear)
				{
					RearA = FMath::Max(RearA, W.SLiftStart);
					RearB = FMath::Min(RearB, W.SLoaded);
					RearC = FMath::Max(RearC, W.SClear);
					++NumRear;
				}
			}
			if (NumFront == 0 || FrontB < FrontA)
			{
				FrontB = -1.0;
			}
			if (NumRear == 0 || RearB < RearA)
			{
				RearB = -1.0;
			}
		};
		const double CSS = FMath::Max(0.1, static_cast<double>(T.CenterShiftSpeed));
		for (int32 Side = 0; Side < 2; ++Side)
		{
			const int32 Leg = Centre[Side];
			if (Leg == INDEX_NONE)
			{
				continue;
			}
			double FrontA, FrontB, RearA, RearB, RearC;
			Windows(bPerSide ? Side : -1, FrontA, FrontB, RearA, RearB, RearC);
			// With Ramms.Curb.ChoreographyRearDuringLift the aft stand is due once
			// the rear corners are clear rather than as they leave the ground: the
			// walk from the landing overlaps their lift.
			const double	  RearFrom = RearDuringLiftOn() && RearC > RearA ? FMath::Min(RearC, RearB) : RearA;
			const double	  X0 = CentreX0(Side);
			const FWheelPlan& W = AtFloor.Wheels[Leg];
			const FWheelPlan& WC = bHolds ? AtCruise.Wheels[Leg] : W;
			const FVector2D	  Reach = CentreReachX(Side);
			const double	  Low = Reach.X, High = Reach.Y;
			int32			  Stuck = INDEX_NONE;
			bool			  bPath = false;
			// Smooth: still on the chassis as it leaves the ground and as it lands,
			// its walks eased in and out. Else the walk as fast and as late as its
			// knots allow (as before v3); the planner tries both.
			{
				TArray<FVector2D> Bands;
				TArray<double>	  Up, Down, Wish;
				TArray<uint8>	  Wanted;
				Bands.Init(FVector2D(Reach.X - 0.01, Reach.Y + 0.01), NumS);
				Up.Init(0.0, NumS);
				Down.Init(0.0, NumS);
				Wish.Init(X0, NumS);
				Wanted.Init(0, NumS);
				auto Bound = [&](double A, double B, double Lo, double Hi) {
					for (int32 k = FMath::Max(0, Index(A)); k <= Index(B) && k < NumS; ++k)
					{
						Bands[k] = FVector2D(FMath::Max(Bands[k].X, Lo), FMath::Min(Bands[k].Y, Hi));
					}
				};
				auto Want = [&](double A, double B, double X) {
					for (int32 k = FMath::Max(0, Index(A)); k <= Index(B) && k < NumS; ++k)
					{
						Wish[k] = X;
						Wanted[k] = 1;
					}
				};
				// The smooth path is averaged over +-Smooth at the end
				// (SmoothCentrePath), so every bound and hold is set that much
				// wider: the average over a window wholly inside one keeps to it.
				const double Smooth = bSmooth ? WalkSmoothCm() : 0.0;
				const double Hold = bSmooth ? WalkHoldCm() : 0.0;
				// Still on the chassis while it leaves the ground and while it lands
				// and takes load -- a wheel that touches down walking meets the
				// ground at another speed than its tread's -- over where its lift
				// and landing fall at any speed from the floor to the cruise.
				const FWheelPlan& WF = WalkHoldWholeSpan() ? W : WC;
				const FVector2D	  LiftHold(FMath::Min(WF.SLiftStart, WC.SLiftStart) - Hold - Smooth, FMath::Max(WF.SLiftStart, WC.SLiftStart) + Hold + Smooth);
				const FVector2D	  LandHold(FMath::Min(WF.SDown, WC.SDown) - Hold - Smooth, FMath::Max(WF.SLoaded, WC.SLoaded) + Hold + Smooth);
				auto			  Held = [&](int32 k) {
					 const double S = SAt(k);
					 return Hold > 0.0 && ((S >= LiftHold.X && S <= LiftHold.Y) || (S >= LandHold.X && S <= LandHold.Y));
				};
				// Its own crossing: towards XLift from the end of the front
				// corners' to its lift, towards XLand from there to its landing (or,
				// unsmoothed, to its window and its load).
				// (Wanted only around the lift and the landing, so that with
				// Ramms.Curb.WalkEarly the walks to them are ramps; without it the
				// tube still brings the wheel there as late as it can.)
				const bool bEarly = WalkEarlyOn();
				if (Hold > 0.0)
				{
					Want(bEarly ? LiftHold.X : (FrontB >= 0.0 ? FrontB : 0.0), LiftHold.Y, XLift);
					Want(bEarly ? LandHold.X : LiftHold.Y, LandHold.Y, XLand);
				}
				else
				{
					Want(bEarly ? W.SLiftStart - EventPadCm : (FrontB >= 0.0 ? FrontB : 0.0), W.SClear, XLift);
					Want(bEarly ? W.SDown - EventPadCm : W.SClear, W.SLoaded + EventPadCm, XLand);
				}
				if (FrontB >= 0.0)
				{
					Want(FrontA - EventPadCm - Smooth, FrontB + EventPadCm + Smooth, XF);
					Bound(FMath::Max(FrontA - EventPadCm, LevelFromS) - Smooth, FrontB + EventPadCm + Smooth, XF - 0.01, High + 0.01);
				}
				if (RearB >= 0.0)
				{
					Want(RearFrom - EventPadCm - Smooth, RearB + EventPadCm + Smooth, XR);
					Bound(FMath::Max(RearFrom - EventPadCm, LevelFromS) - Smooth, RearB + EventPadCm + Smooth, Low - 0.01, XR + 0.01);
				}
				Bound(SAt(NumS - 1) - Smooth, SAt(NumS - 1), X0 - 0.01, X0 + 0.01);
				// Between two wanted positions (and from the level-in to the first, and
				// from the last home): a ramp over the whole stretch, so the wheel
				// walks gently as the robot drives rather than as late as it can at
				// CenterShiftSpeed. The tube still bounds it.
				if (WalkEarlyOn())
				{
					const int32 KL = FMath::Max(0, Index(LevelFromS));
					for (int32 k = KL; k < NumS;)
					{
						if (Wanted[k])
						{
							++k;
							continue;
						}
						const int32 A = k;
						while (k < NumS && !Wanted[k])
						{
							++k;
						}
						const int32	 B = k;
						const double From = A > KL ? Wish[A - 1] : X0;
						const double To = B < NumS ? Wish[B] : X0;
						// The walk home at the end over WalkHomeCm at most: Settle waits
						// for the centre wheels to be home.
						const int32 Over = B < NumS ? B - A + 1 : FMath::Min(B - A + 1, FMath::Max(1, FMath::RoundToInt(WalkHomeCm / Ds)));
						for (int32 j = A; j < B; ++j)
						{
							Wish[j] = From + (To - From) * FMath::Min(1.0, static_cast<double>(j - A + 1) / static_cast<double>(Over));
						}
					}
				}
				// At the cruise speed first: a walk that keeps to it keeps to any
				// slower one, and never asks the robot to slow for it. Where that
				// does not fit, at the floor speed: the speed envelope then slows
				// the robot where it walks (its walk cap).
				for (const double V : { Cruise, FMath::Max(Floor, MinTimingSpeed) })
				{
					const double Step = Ds * CSS / V;
					for (int32 k = 1; k < NumS; ++k)
					{
						// Fast aft only inside its own window, where it is off the
						// ground at any speed.
						const bool bAir = InWindow(Leg, P, k, P.X[Side][k]) && InWindow(Leg, P, k - 1, P.X[Side][k - 1]);
						Up[k] = Step;
						Down[k] = bAir ? Step : FMath::Min(Step, MaxAftWalkPerCm * Ds);
						if (IsDwell(P, k))
						{
							// At rest at a planned stop: any re-pose.
							Up[k] = Down[k] = 1e6;
						}
						else if (Held(k))
						{
							Up[k] = Down[k] = 0.0;
						}
					}
					TArray<double> Path;
					if (RatePath(Bands, Up, Down, X0, Wish, Path, Stuck))
					{
						P.X[Side] = SmoothCentrePath(Path, Smooth, Ds);
						bPath = true;
						break;
					}
				}
			}
			if (!bPath)
			{
				P.FailWhy = FString::Printf(TEXT("the %s centre wheel cannot walk between its knots in time (at %.0f cm)"),
					Side ? TEXT("+Y") : TEXT("-Y"), SAt(FMath::Max(0, Stuck)));
				return false;
			}
		}
		return true;
	}

	void FPlanner::CentreStatus(const FPlan& P, int32 Leg, int32 K, double X, double V, bool& bAir, bool& bFar) const
	{
		// From its own distance past the edge: off the ground from its lift
		// lead before the window to its landing tail after it.
		const FWheelPlan& W = P.Wheels[Leg];
		const double	  D = M.DistancePast(Leg, SAt(K), X);
		const double	  Share = M.NormalShare();
		const double	  LeadD = V * W.TLift * Share;
		const double	  TailD = V * (W.TPlant + LoadSeconds) * Share;
		bAir = D >= DUp[Leg] - LeadD && D < DDown[Leg] + TailD;
		bFar = D >= DDown[Leg] + TailD;
	}

	TArray<int32> FPlanner::DpStages(const FPlan& Q, int32 KA, int32 KB) const
	{
		// Every DpStageCm, and either side of every planned stop's dwell (where
		// a re-pose is made at rest), from KA to KB.
		const int32	  StageEvery = FMath::Max(1, FMath::RoundToInt(DpStageCm / Ds));
		TArray<int32> Stages;
		for (int32 k = KA; k <= KB; k += StageEvery)
		{
			Stages.Add(k);
		}
		Stages.AddUnique(KB);
		for (int32 k = KA; k < KB; ++k)
		{
			if (Q.Stop.IsValidIndex(k) && Q.Stop[k])
			{
				Stages.AddUnique(k);
				Stages.AddUnique(k + 1);
			}
		}
		Stages.Sort();
		return Stages;
	}

	double FPlanner::DpScore(const FInstantQuery& Q, double BGuess) const
	{
		// What an instant keeps: its margin over the requirement and its
		// corridor over the least, whichever is less; a pose that does not
		// reach, or a set never lifted together, is no pose.
		++Work;
		const FInstantResult R = M.Evaluate(Q, BGuess);
		++CountFeasible;
		return R.Lo > R.Hi || !R.bReach || IsForbiddenSet(In.Model, Q.Airborne) ? -1e6 : FMath::Min(R.Margin - Req, R.Width() - T.MinCorridorCm);
	}

	void FPlanner::CentrePathsDp(FPlan& P) const
	{
		// Each centre wheel's fore/aft on a grid every DpStageCm of travel: the
		// path whose worst instant keeps the most (margin over the requirement,
		// corridor over its least), then the least shortfall summed over the
		// instants, then the least walking. First both wheels together (the
		// other walking as this one does: a robot standing on its centre wheels
		// needs both forward at once, which neither finds alone), then each with
		// the other's path held, one side and then the other (CentrePathsJoint
		// searches their pairs where this leaves the plan failing). Searched at
		// the floor speed: the fewest wheels off the ground together that a plan
		// may keep; the speed envelope then lets the robot go faster where they
		// allow. Each wheel's grid is its 5-bar's reach. At a planned stop the
		// robot is at rest: a wheel may re-pose anywhere its reach allows, every
		// pose on the way checked.
		const double Started = FPlatformTime::Seconds();
		FPlan		 Q = P;
		const int32	 K0 = Index(LevelFromS);
		for (int32 k = K0; k < NumS; ++k)
		{
			Q.VCap[k] = Q.Stop[k] ? Q.VCap[k] : FMath::Min(Cruise, Floor);
		}
		Passes(Q);
		ComputeTiming(Q);
		const TArray<double>& VDp = Q.V;
		const TArray<int32>	  Stages = DpStages(Q, 0, NumS - 1);
		const double		  CSS = FMath::Max(0.1, static_cast<double>(T.CenterShiftSpeed));
		struct FCell
		{
			double Score = 0.0;
			double Total = 0.0;
			double Walk = 0.0;
			int32  Prev = INDEX_NONE;
			bool   bReached = false;
		};
		TArray<FCell> Cells;
		const bool	  bBoth = Centre[0] != INDEX_NONE && Centre[1] != INDEX_NONE;
		for (int32 Sweep = 0; Sweep < 3; ++Sweep)
		{
			for (int32 Side = 0; Side < 2; ++Side)
			{
				const int32 Leg = Centre[Side];
				// The first sweep: both together, searched once (from this side).
				const bool bTogether = Sweep == 0 && bBoth;
				if (Leg == INDEX_NONE || (Sweep == 0 && (!bBoth || Side == 1)))
				{
					continue;
				}
				const int32	 Other = Centre[1 - Side];
				const double X0 = CentreX0(Side);
				const double OtherFrom = bTogether ? CentreX0(1 - Side) - X0 : 0.0;
				// Its grid: where it began, and every DpStepCm from there over its
				// reach (both wheels' together).
				FVector2D Reach = CentreReachX(Side);
				Reach = FVector2D(FMath::Max(Reach.X, X0 - CentreSpanCm), FMath::Min(Reach.Y, X0 + CentreSpanCm));
				if (bTogether)
				{
					const FVector2D R2 = CentreReachX(1 - Side);
					Reach = FVector2D(FMath::Max(Reach.X, R2.X - OtherFrom), FMath::Min(Reach.Y, R2.Y - OtherFrom));
				}
				const int32 J0 = FMath::Max(0, FMath::FloorToInt((X0 - Reach.X) / DpStepCm + 1e-9));
				const int32 NX = J0 + FMath::Max(0, FMath::FloorToInt((Reach.Y - X0) / DpStepCm + 1e-9)) + 1;
				auto		XOf = [&](int32 J) { return X0 + DpStepCm * (J - J0); };
				Cells.Reset();
				Cells.SetNum(Stages.Num() * NX);
				// The robot at sample k with this wheel at X (the other on its
				// path, or with it).
				auto QueryAt = [&](int32 k, double X, double V) {
					FInstantQuery QI = Query(Q, k, 0, 0);
					for (int32 i = 0; i < N; ++i)
					{
						if (i != Leg && i != Other)
						{
							QI.Airborne |= Q.Air[k] & (1u << i);
							QI.OnFar |= Q.Far[k] & (1u << i);
						}
					}
					bool bAir = false, bFar = false;
					CentreStatus(Q, Leg, k, X, V, bAir, bFar);
					QI.X[Leg] = X;
					QI.Airborne |= (bAir ? 1u : 0u) << Leg;
					QI.OnFar |= (bFar ? 1u : 0u) << Leg;
					if (Other != INDEX_NONE)
					{
						const double XO = bTogether ? X + OtherFrom : Q.X[1 - Side][k];
						bool		 bOAir = false, bOFar = false;
						CentreStatus(Q, Other, k, XO, V, bOAir, bOFar);
						QI.X[Other] = XO;
						QI.Airborne |= (bOAir ? 1u : 0u) << Other;
						QI.OnFar |= (bOFar ? 1u : 0u) << Other;
					}
					return QI;
				};
				for (int32 s = 0; s < Stages.Num(); ++s)
				{
					const int32 k = Stages[s];
					const bool	bDwell = s > 0 && Q.Stop[Stages[s - 1]] && k == Stages[s - 1] + 1;
					double		VMax = FMath::Max(MinTimingSpeed, Q.V[k]), VMin = Q.V[k];
					if (s > 0)
					{
						for (int32 j = Stages[s - 1]; j <= k; ++j)
						{
							VMax = FMath::Max(VMax, Q.V[j]);
							VMin = FMath::Min(VMin, Q.V[j]);
						}
					}
					// About a planned stop the robot is all but at rest: a planted
					// wheel may re-pose aft as well.
					const bool	 bNearRest = Q.Stops > 0 && VMin < 0.5 * Floor;
					const double StageCm = s > 0 ? SAt(k) - SAt(Stages[s - 1]) : 0.0;
					const double Step = bDwell ? 1e6 : StageCm * CSS / VMax;
					for (int32 J = 0; J < NX; ++J)
					{
						const double X = XOf(J);
						const double Here = k >= K0 ? DpScore(QueryAt(k, X, VDp[k]), BGuessAt(Q, k)) : TNumericLimits<double>::Max();
						FCell&		 C = Cells[s * NX + J];
						const double Short = FMath::Min(Here, 0.0);
						if (s == 0)
						{
							if (J == J0)
							{
								C.Score = Here;
								C.Total = Short;
								C.bReached = true;
							}
							continue;
						}
						const double Aft = InWindow(Leg, Q, k, X) || bNearRest || bDwell ? Step : FMath::Min(Step, MaxAftWalkPerCm * StageCm);
						for (int32 PJ = 0; PJ < NX; ++PJ)
						{
							const FCell& PC = Cells[(s - 1) * NX + PJ];
							const double Dx = DpStepCm * (J - PJ);
							if (!PC.bReached || Dx > Step + 1e-6 || -Dx > Aft + 1e-6)
							{
								continue;
							}
							double		 Score = FMath::Min(PC.Score, Here);
							const double Total = PC.Total + Short;
							const double W = PC.Walk + FMath::Abs(Dx);
							auto		 Better = [&]() {
								return !C.bReached || Score > C.Score + 1e-9
									|| (FMath::IsNearlyEqual(Score, C.Score, 1e-9)
										&& (Total > C.Total + 1e-9 || (FMath::IsNearlyEqual(Total, C.Total, 1e-9) && W < C.Walk)));
							};
							if (!Better())
							{
								continue;
							}
							if (bDwell && J != PJ && Stages[s - 1] >= K0)
							{
								// The re-pose at rest, at the stop: every pose on the way.
								const int32 KS = Stages[s - 1];
								for (int32 m = 1; m < DwellPoseSteps && Score > -1e5; m += 2)
								{
									const double Xm = FMath::Lerp(XOf(PJ), X, double(m) / DwellPoseSteps);
									Score = FMath::Min(Score, DpScore(QueryAt(KS, Xm, 0.0), BGuessAt(Q, KS)));
								}
								if (!Better())
								{
									continue;
								}
							}
							C.Score = Score;
							C.Total = Total;
							C.Walk = W;
							C.Prev = PJ;
							C.bReached = true;
						}
					}
				}
				// The best end -- home if it is as good -- back to the start.
				int32 Best = INDEX_NONE;
				for (int32 J = 0; J < NX; ++J)
				{
					const FCell& C = Cells[(Stages.Num() - 1) * NX + J];
					if (!C.bReached)
					{
						continue;
					}
					const FCell* B = Best != INDEX_NONE ? &Cells[(Stages.Num() - 1) * NX + Best] : nullptr;
					const bool	 bSame = B && FMath::IsNearlyEqual(C.Score, B->Score, 1e-9) && FMath::IsNearlyEqual(C.Total, B->Total, 1e-9);
					if (!B || C.Score > B->Score + 1e-9 || (FMath::IsNearlyEqual(C.Score, B->Score, 1e-9) && C.Total > B->Total + 1e-9)
						|| (bSame && (J == J0 || (Best != J0 && C.Walk < B->Walk))))
					{
						Best = J;
					}
				}
				if (Best == INDEX_NONE)
				{
					continue;
				}
				TArray<int32> Path;
				Path.SetNum(Stages.Num());
				for (int32 s = Stages.Num() - 1; s >= 0; --s)
				{
					Path[s] = Best;
					Best = s > 0 ? Cells[s * NX + Best].Prev : Best;
				}
				for (int32 s = 0; s < Stages.Num(); ++s)
				{
					const int32	 KA = Stages[s];
					const int32	 KB = s + 1 < Stages.Num() ? Stages[s + 1] : NumS - 1;
					const double XA = XOf(Path[s]);
					const double XB = s + 1 < Stages.Num() ? XOf(Path[s + 1]) : XA;
					for (int32 k = KA; k <= KB; ++k)
					{
						// A dwell's re-pose is all made at the stop: the sample after it
						// stands where the robot leaves from.
						const bool	 bStep = KB == KA + 1 && Q.Stop[KA];
						const double A = bStep ? (k == KA ? 0.0 : 1.0) : (KB > KA ? double(k - KA) / (KB - KA) : 0.0);
						Q.X[Side][k] = FMath::Lerp(XA, XB, A);
						if (bTogether)
						{
							Q.X[1 - Side][k] = Q.X[Side][k] + OtherFrom;
						}
					}
				}
			}
		}
		P.X[0] = Q.X[0];
		P.X[1] = Q.X[1];
		TimeDp += FPlatformTime::Seconds() - Started;
	}

	void FPlanner::CentrePathsJoint(FPlan& P, int32 KA, int32 KB) const
	{
		// Both centre wheels' fore/aft searched together over KA..KB, on a grid
		// of their pairs every DpStepCm within JointSpanCm of where each stands at
		// KA (and its reach): what a search of one wheel with the other held, or
		// of both alike, cannot find -- one forward as the other goes aft, say.
		// From the paths at KA and back to them at KB; scored as CentrePathsDp,
		// at the floor speed, a planned stop's dwell a re-pose at rest.
		if (Centre[0] == INDEX_NONE || Centre[1] == INDEX_NONE)
		{
			return;
		}
		KA = FMath::Clamp(KA, 0, NumS - 1);
		KB = FMath::Clamp(KB, KA, NumS - 1);
		if (KB - KA < 2)
		{
			return;
		}
		const double Started = FPlatformTime::Seconds();
		FPlan		 Q = P;
		const int32	 K0 = Index(LevelFromS);
		for (int32 k = K0; k < NumS; ++k)
		{
			Q.VCap[k] = Q.Stop[k] ? Q.VCap[k] : FMath::Min(Cruise, Floor);
		}
		Passes(Q);
		ComputeTiming(Q);
		const TArray<int32> Stages = DpStages(Q, KA, KB);
		const double		CSS = FMath::Max(0.1, static_cast<double>(T.CenterShiftSpeed));
		const int32			L = Centre[0], R = Centre[1];
		double				XFrom[2], XTo[2];
		int32				J0[2], NX[2];
		for (int32 Side = 0; Side < 2; ++Side)
		{
			XFrom[Side] = Q.X[Side][KA];
			XTo[Side] = Q.X[Side][KB];
			const FVector2D Reach = CentreReachX(Side);
			const double	Lo = FMath::Max(Reach.X, XFrom[Side] - JointSpanCm), Hi = FMath::Min(Reach.Y, XFrom[Side] + JointSpanCm);
			J0[Side] = FMath::Max(0, FMath::FloorToInt((XFrom[Side] - Lo) / DpStepCm + 1e-9));
			NX[Side] = J0[Side] + FMath::Max(0, FMath::FloorToInt((Hi - XFrom[Side]) / DpStepCm + 1e-9)) + 1;
		}
		auto		XOf = [&](int32 Side, int32 J) { return XFrom[Side] + DpStepCm * (J - J0[Side]); };
		const int32 NS = NX[0] * NX[1];
		struct FCell
		{
			double Score = 0.0;
			double Total = 0.0;
			double Walk = 0.0;
			int32  Prev = INDEX_NONE;
			bool   bReached = false;
		};
		TArray<FCell> Cells;
		Cells.SetNum(Stages.Num() * NS);
		auto QueryAt = [&](int32 k, double XL, double XR, double V) {
			FInstantQuery QI = Query(Q, k, 0, 0);
			for (int32 i = 0; i < N; ++i)
			{
				if (i != L && i != R)
				{
					QI.Airborne |= Q.Air[k] & (1u << i);
					QI.OnFar |= Q.Far[k] & (1u << i);
				}
			}
			const double Xs[2] = { XL, XR };
			for (int32 Side = 0; Side < 2; ++Side)
			{
				const int32 Leg = Centre[Side];
				bool		bAir = false, bFar = false;
				CentreStatus(Q, Leg, k, Xs[Side], V, bAir, bFar);
				QI.X[Leg] = Xs[Side];
				QI.Airborne |= (bAir ? 1u : 0u) << Leg;
				QI.OnFar |= (bFar ? 1u : 0u) << Leg;
			}
			return QI;
		};
		const int32 Last = Stages.Num() - 1;
		for (int32 s = 0; s < Stages.Num(); ++s)
		{
			const int32 k = Stages[s];
			const bool	bDwell = s > 0 && Q.Stop[Stages[s - 1]] && k == Stages[s - 1] + 1;
			double		VMax = FMath::Max(MinTimingSpeed, Q.V[k]), VMin = Q.V[k];
			if (s > 0)
			{
				for (int32 j = Stages[s - 1]; j <= k; ++j)
				{
					VMax = FMath::Max(VMax, Q.V[j]);
					VMin = FMath::Min(VMin, Q.V[j]);
				}
			}
			const bool	 bNearRest = Q.Stops > 0 && VMin < 0.5 * Floor;
			const double StageCm = s > 0 ? SAt(k) - SAt(Stages[s - 1]) : 0.0;
			const double Step = bDwell ? 1e6 : StageCm * CSS / VMax;
			for (int32 JL = 0; JL < NX[0]; ++JL)
			{
				for (int32 JR = 0; JR < NX[1]; ++JR)
				{
					// The first and last stages are the paths' own.
					const bool bEnd = s == 0 || s == Last;
					if (bEnd && (s == 0 ? (JL != J0[0] || JR != J0[1]) : (JL != 0 || JR != 0)))
					{
						continue;
					}
					const double XL = s == Last ? XTo[0] : XOf(0, JL);
					const double XR = s == Last ? XTo[1] : XOf(1, JR);
					const double Here = k >= K0 ? DpScore(QueryAt(k, XL, XR, Q.V[k]), BGuessAt(Q, k)) : TNumericLimits<double>::Max();
					FCell&		 C = Cells[s * NS + JL * NX[1] + JR];
					const double Short = FMath::Min(Here, 0.0);
					if (s == 0)
					{
						C.Score = Here;
						C.Total = Short;
						C.bReached = true;
						continue;
					}
					double Aft[2];
					for (int32 Side = 0; Side < 2; ++Side)
					{
						const double X = Side == 0 ? XL : XR;
						Aft[Side] = InWindow(Centre[Side], Q, k, X) || bNearRest || bDwell ? Step : FMath::Min(Step, MaxAftWalkPerCm * StageCm);
					}
					for (int32 PL = 0; PL < NX[0]; ++PL)
					{
						const double DxL = XL - XOf(0, PL);
						if (DxL > Step + 1e-6 || -DxL > Aft[0] + 1e-6)
						{
							continue;
						}
						for (int32 PR = 0; PR < NX[1]; ++PR)
						{
							const FCell& PC = Cells[(s - 1) * NS + PL * NX[1] + PR];
							const double DxR = XR - XOf(1, PR);
							if (!PC.bReached || DxR > Step + 1e-6 || -DxR > Aft[1] + 1e-6)
							{
								continue;
							}
							double		 Score = FMath::Min(PC.Score, Here);
							const double Total = PC.Total + Short;
							const double W = PC.Walk + FMath::Abs(DxL) + FMath::Abs(DxR);
							auto		 Better = [&]() {
								return !C.bReached || Score > C.Score + 1e-9
									|| (FMath::IsNearlyEqual(Score, C.Score, 1e-9)
										&& (Total > C.Total + 1e-9 || (FMath::IsNearlyEqual(Total, C.Total, 1e-9) && W < C.Walk)));
							};
							if (!Better())
							{
								continue;
							}
							if (bDwell && (FMath::Abs(DxL) > 1e-9 || FMath::Abs(DxR) > 1e-9) && Stages[s - 1] >= K0)
							{
								const int32 KS = Stages[s - 1];
								for (int32 m = 1; m < DwellPoseSteps && Score > -1e5; m += 2)
								{
									const double A = double(m) / DwellPoseSteps;
									Score = FMath::Min(Score, DpScore(QueryAt(KS, XOf(0, PL) + A * DxL, XOf(1, PR) + A * DxR, 0.0), BGuessAt(Q, KS)));
								}
								if (!Better())
								{
									continue;
								}
							}
							C.Score = Score;
							C.Total = Total;
							C.Walk = W;
							C.Prev = PL * NX[1] + PR;
							C.bReached = true;
						}
					}
				}
			}
		}
		const FCell& End = Cells[Last * NS];
		if (!End.bReached)
		{
			TimeDp += FPlatformTime::Seconds() - Started;
			return;
		}
		TArray<int32> Path;
		Path.SetNum(Stages.Num());
		int32 Cur = 0;
		for (int32 s = Last; s >= 0; --s)
		{
			Path[s] = Cur;
			Cur = s > 0 ? Cells[s * NS + Cur].Prev : Cur;
		}
		for (int32 s = 0; s < Last; ++s)
		{
			const int32 SA = Stages[s], SB = Stages[s + 1];
			for (int32 Side = 0; Side < 2; ++Side)
			{
				const int32	 JA = Side == 0 ? Path[s] / NX[1] : Path[s] % NX[1];
				const int32	 JB = Side == 0 ? Path[s + 1] / NX[1] : Path[s + 1] % NX[1];
				const double XA = s == 0 ? XFrom[Side] : XOf(Side, JA);
				const double XB = s + 1 == Last ? XTo[Side] : XOf(Side, JB);
				for (int32 k = SA; k <= SB; ++k)
				{
					const bool	 bStep = SB == SA + 1 && Q.Stop[SA];
					const double A = bStep ? (k == SA ? 0.0 : 1.0) : double(k - SA) / FMath::Max(1, SB - SA);
					P.X[Side][k] = FMath::Lerp(XA, XB, A);
				}
			}
		}
		TimeDp += FPlatformTime::Seconds() - Started;
	}

	bool FPlanner::HeightPath(FPlan& P) const
	{
		// At every sample from the level-in on: the corridor of chassis heights
		// (every leg reaching, the belly clear) and, where the margin does not
		// hold across all of it, the part of it that has it, around its best.
		// Then a path through them, no faster than BaseHeightSpeed at the
		// planned speed, as near their middles as it may.
		const double Started = FPlatformTime::Seconds();
		P.WhyAt.Reset();
		const int32		  K0 = Index(LevelFromS);
		const int32		  Count = NumS - K0;
		TArray<FVector2D> Bands;
		TArray<double>	  Up, Down, Wish;
		Bands.SetNum(Count);
		Up.Init(0.0, Count);
		Down.Init(0.0, Count);
		Wish.SetNum(Count);
		const double BH = FMath::Max(0.1, static_cast<double>(T.BaseHeightSpeed));
		bool		 bOk = true;
		auto		 Holds = [&](const FInstantQuery& Q, const FInstantResult& R, double B) {
			FInstantResult X = R;
			M.PoseAt(Q, B, X);
			return X.bReach && X.Margin >= Req - 1e-9;
		};
		for (int32 c = 0; c < Count; ++c)
		{
			const int32			k = K0 + c;
			const FInstantQuery Q = Query(P, k, P.Air[k], P.Far[k]);
			if (c > 0)
			{
				const double V = FMath::Max(MinTimingSpeed, FMath::Max(P.V[k], P.V[k - 1]));
				// At rest at a planned stop the chassis may move as far as it likes.
				// While a wheel touches down and takes load, and LandingSettleSeconds
				// after, only a share of BaseHeightSpeed: a leg just set down has to
				// turn from going down to following the chassis, and a chassis that
				// moves fast then has it push the base over.
				Up[c] = Down[c] = IsDwell(P, k) ? 1e6 : Ds * BH / V;
				if (!IsDwell(P, k))
				{
					const double S = SAt(k);
					const double Lift = LiftHeightShare();
					for (const FWheelPlan& W : P.Wheels)
					{
						if (S >= W.SDown - V * LandingApproachSeconds() - 0.5 * Ds && S <= W.SLoaded + V * LandingSettleSeconds)
						{
							Up[c] = Down[c] = Ds * BH * LandingHeightShare / V;
							break;
						}
						if (Lift > 0.0 && !W.bHeldFromStart && S >= W.SLiftStart - 0.5 * Ds && S <= W.SClear)
						{
							// (Development, LiftHeightShare: while a wheel unloads the
							// chassis moves slower too.)
							Up[c] = Down[c] = FMath::Min(Up[c], Ds * BH * Lift / V);
						}
					}
				}
			}
			if (P.Fail[k] == EFail::Static || P.Fail[k] == EFail::Slow)
			{
				// Known to fail already: carried through on the legs' reach alone,
				// so that the rest of the path is still planned.
				++Work;
				const FInstantResult R0 = M.Evaluate(Q, BGuessAt(P, k));
				const bool			 bBand = R0.Lo <= R0.Hi;
				Bands[c] = bBand ? FVector2D(R0.Lo, FMath::Min(R0.Hi, R0.Lo + 80.0)) : FVector2D(-1e6, 1e6);
				Wish[c] = bBand ? 0.5 * (Bands[c].X + Bands[c].Y) : BGuessAt(P, k);
				P.BLo[k] = R0.Lo;
				P.BHi[k] = R0.Hi;
				continue;
			}
			const uint64 Key = InstantKey(Q, BGuessAt(P, k));
			double		 Lo = 0.0, Hi = 0.0;
			if (const FVector2D* Found = P.BandMemo.Find(Key))
			{
				Lo = Found->X;
				Hi = Found->Y;
			}
			else
			{
				FInstantResult R;
				const bool	   bAny = Feasible(Q, BGuessAt(P, k), &R);
				Lo = R.Lo;
				Hi = FMath::Min(R.Hi, R.Lo + 80.0);
				if (!bAny || !(Holds(Q, R, Lo) && Holds(Q, R, Hi)))
				{
					if (bAny)
					{
						// Out from the best height while the margin holds, 0.5 cm
						// steps.
						const double Best = R.B;
						double		 A = Best, Z = Best;
						while (A - 0.5 >= Lo - 1e-9 && Holds(Q, R, A - 0.5))
						{
							A -= 0.5;
						}
						while (Z + 0.5 <= Hi + 1e-9 && Holds(Q, R, Z + 0.5))
						{
							Z += 0.5;
						}
						Lo = A;
						Hi = Z;
					}
					else
					{
						Hi = Lo - 1.0;
					}
				}
				P.BandMemo.Add(Key, FVector2D(Lo, Hi));
			}
			P.BLo[k] = Lo;
			P.BHi[k] = Hi;
			if (Hi - Lo < T.MinCorridorCm - 1e-9)
			{
				if (P.Fail[k] == EFail::None)
				{
					P.Fail[k] = EFail::Height;
				}
				bOk = false;
				const double Mid = 0.5 * (Lo + Hi);
				Lo = Hi = Mid;
			}
			Bands[c] = FVector2D(Lo, Hi);
			Wish[c] = 0.5 * (Lo + Hi);
			if (Q.Touching != 0)
			{
				// A leg touching down bounds the chassis from above only where it
				// must reach: the path aims where it would without it, kept inside.
				FInstantQuery Q0 = Q;
				Q0.Touching = 0;
				++Work;
				const FInstantResult R0 = M.Evaluate(Q0, BGuessAt(P, k));
				if (R0.Lo <= R0.Hi)
				{
					Wish[c] = FMath::Clamp(0.5 * (R0.Lo + FMath::Min(R0.Hi, R0.Lo + 80.0)), Lo, Hi);
				}
			}
		}
		// Rising early (Ramms.Curb.RiseEarly): the chassis is asked up to the
		// most any sample up to the last landing wishes, from the level-in on
		// -- within each sample's own corridor -- so that it rises on every
		// wheel before the first lift and holds there through the crossing
		// rather than rising under each lift. After the last landing it settles
		// as before.
		if (bOk && RiseEarlyOn())
		{
			double LastLoaded = LevelFromS;
			for (const FWheelPlan& W : P.Wheels)
			{
				LastLoaded = FMath::Max(LastLoaded, W.SLoaded);
			}
			// The level-in's own target (the first sample) is left as it was:
			// the rise runs from there, not jumped in the level-in; and it keeps
			// RiseSlackCm under a corridor's top where it can, so the legs are
			// not sent to the edge of their reach.
			// With bPlantOnHold, no higher than the lifted wheels, set down on
			// their own surface, could still reach (a halt sets them down; a leg
			// short of the ground then is a fault), RiseSlackCm under that too.
			// Without it a halt holds every leg where it is -- a corner carried
			// up in the drive stance stays up -- and the rise need not wait for
			// the fronts to be over the curb.
			const int32	   CEnd = FMath::Clamp(Index(LastLoaded) - K0, 0, Count - 1);
			TArray<double> Own;
			Own.SetNumUninitialized(CEnd + 1);
			for (int32 c = 0; c <= CEnd; ++c)
			{
				Own[c] = Wish[c];
			}
			double Need = Wish[CEnd];
			for (int32 c = CEnd - 1; c >= 1; --c)
			{
				const int32 k = K0 + c;
				Need = FMath::Max(Need, Wish[c]);
				double Top = FMath::Max(0.5 * (Bands[c].X + Bands[c].Y), Bands[c].Y - RiseSlackCm);
				if (T.bPlantOnHold && P.Air[k] != 0)
				{
					// (The lifted wheels set down on the near side: a halt short of
					// the window sets them down there.)
					FInstantQuery Q0 = Query(P, k, 0u, P.Far[k] & ~P.Air[k]);
					++Work;
					FInstantResult R0;
					if (Feasible(Q0, BGuessAt(P, k), &R0) && R0.Lo <= R0.Hi)
					{
						Top = FMath::Min(Top, FMath::Max(Bands[c].X, R0.Hi - RiseSlackCm));
					}
				}
				Wish[c] = FMath::Clamp(Need, Bands[c].X, Top);
			}
			// A ramp rather than a step: the chassis rises evenly from the
			// level-in's height to the crossing's over the stretch before the
			// first sample that wants that height of its own (each sample still
			// at least what it wants itself), so that it rises at the gentlest
			// pace the approach allows rather than at BaseHeightSpeed at once --
			// a fast rise on the planted legs pitches the base.
			int32 CNeed = CEnd;
			for (int32 c = 1; c <= CEnd; ++c)
			{
				if (Own[c] >= Wish[c] - 0.5)
				{
					CNeed = c;
					break;
				}
			}
			for (int32 c = 1; c < CNeed; ++c)
			{
				const double Ramp = Own[0] + (Wish[CNeed] - Own[0]) * static_cast<double>(c) / static_cast<double>(CNeed);
				Wish[c] = FMath::Clamp(FMath::Max(Ramp, Own[c]), Bands[c].X, FMath::Max(Bands[c].X, Wish[c]));
			}
		}
		bool bPath = false;
		if (bOk)
		{
			TArray<double> Path;
			int32		   Stuck = INDEX_NONE;
			bPath = RatePath(Bands, Up, Down, NoValue, Wish, Path, Stuck);
			if (bPath)
			{
				for (int32 c = 0; c < Count; ++c)
				{
					P.B[K0 + c] = Path[c];
				}
				for (int32 k = 0; k < K0; ++k)
				{
					P.B[k] = Path[0];
				}
			}
			else
			{
				const int32 k = K0 + FMath::Max(0, Stuck);
				if (P.Fail[k] == EFail::None)
				{
					P.Fail[k] = EFail::Height;
				}
				P.FailWhy = FString::Printf(TEXT("the corridor of chassis heights moves faster than BaseHeightSpeed lets the chassis follow "
												 "at %.0f cm"),
					SAt(k));
				P.WhyAt.Add(k, P.FailWhy);
			}
		}
		if (!bPath)
		{
			// Still a guess at every sample, for the next round's geometry.
			for (int32 c = 0; c < Count; ++c)
			{
				P.B[K0 + c] = Wish[c];
			}
		}
		TimeHeight += FPlatformTime::Seconds() - Started;
		return bPath;
	}

	void FPlanner::Settle(FPlan& P) const
	{
		// After the last wheel has taken load the centre wheels walk home
		// (within CenterShiftSpeed) -- already in the knot paths; the search's
		// may end anywhere.
		double Last = 0.0;
		for (const FWheelPlan& W : P.Wheels)
		{
			Last = FMath::Max(Last, W.SLoaded);
		}
		const double CSS = FMath::Max(0.1, static_cast<double>(T.CenterShiftSpeed));
		for (int32 Side = 0; Side < 2; ++Side)
		{
			if (Centre[Side] == INDEX_NONE)
			{
				continue;
			}
			const double X0 = CentreX0(Side);
			for (int32 k = Index(Last) + 1; k < NumS; ++k)
			{
				const double V = FMath::Max(MinTimingSpeed, FMath::Max(P.V[k], P.V[k - 1]));
				const double Step = FMath::Min(Ds * CSS / V, MaxAftWalkPerCm * Ds);
				P.X[Side][k] = P.X[Side][k - 1] + FMath::Clamp(X0 - P.X[Side][k - 1], -Step, Ds * CSS / V);
			}
		}
	}

	bool FPlanner::Verify(FPlan& P) const
	{
		const double Started = FPlatformTime::Seconds();
		const int32	 K0 = Index(LevelFromS);
		const double CSS = FMath::Max(0.1, static_cast<double>(T.CenterShiftSpeed));
		const double BH = FMath::Max(0.1, static_cast<double>(T.BaseHeightSpeed));
		const double Rate = FMath::Max(0.1, static_cast<double>(T.TiltRateDegPerSec));
		const double TiltAcc = FMath::Max(0.1, static_cast<double>(T.TiltAccelDegPerSec2));
		const double Dec = FMath::Max(1.0, static_cast<double>(T.Deceleration));
		const double Entry = FMath::Max(0.0, In.EntrySpeedCmS);
		bool		 bOk = true;
		auto		 Fail = [&](int32 k, const FString& Why) {
			if (P.Fail[k] == EFail::None)
			{
				P.Fail[k] = EFail::Verify;
			}
			if (!P.WhyAt.Contains(k))
			{
				P.WhyAt.Add(k, Why);
			}
			if (bOk)
			{
				P.FailWhy = Why;
			}
			bOk = false;
		};
		// A pose: reaching, inside its margin and its corridor.
		auto CheckPose = [&](int32 k, const FInstantQuery& Q, double B, const TCHAR* What) {
			++Work;
			const FInstantResult R = M.Evaluate(Q, B, true);
			if (!R.bReach)
			{
				Fail(k, FString::Printf(TEXT("%s: a leg cannot reach at %.0f cm"), What, SAt(k)));
			}
			else if (R.Margin < Req - 1e-6)
			{
				Fail(k, FString::Printf(TEXT("%s: the margin is %.2f cm at %.0f cm (%.1f at least)"), What, R.Margin, SAt(k), Req));
			}
			else if (B < R.Lo - 1e-6 || B > R.Hi + 1e-6)
			{
				Fail(k, FString::Printf(TEXT("%s: the chassis height leaves its corridor at %.0f cm"), What, SAt(k)));
			}
		};
		const int32			KRise = RiseEnd(P);
		const TArray<uint8> AtStop = StopRuns(P);
		for (int32 k = K0; k < NumS; ++k)
		{
			const FInstantQuery Q = Query(P, k, P.Air[k], P.Far[k]);
			++Work;
			const FInstantResult R = M.Evaluate(Q, P.B[k], true);
			P.Margin[k] = R.Margin;
			P.SeatX[k] = R.SeatX;
			if (P.SeatBand.IsValidIndex(k) && !P.SeatFix.IsValidIndex(k))
			{
				P.SeatBand[k] = R.SeatBand;
			}
			if (k == K0)
			{
				// The approach's samples, before the level-in, are not evaluated:
				// their seat is the crossing's first, so the approach holds the
				// seat where the crossing begins with it (its stance) rather than
				// moving it to the zero pose and back.
				for (int32 j = 0; j < K0; ++j)
				{
					P.SeatX[j] = R.SeatX;
				}
			}
			if (!R.bReach)
			{
				Fail(k, FString::Printf(TEXT("a leg cannot reach at %.0f cm"), SAt(k)));
			}
			if (IsForbiddenSet(In.Model, P.Air[k]))
			{
				Fail(k, FString::Printf(TEXT("a set the gait never lifts together is off the ground at %.0f cm"), SAt(k)));
			}
			if (R.Margin < Req - 1e-6)
			{
				Fail(k, FString::Printf(TEXT("the margin is %.2f cm at %.0f cm (%.1f at least)"), R.Margin, SAt(k), Req));
			}
			if (P.B[k] < R.Lo - 1e-6 || P.B[k] > R.Hi + 1e-6)
			{
				Fail(k, FString::Printf(TEXT("the chassis height leaves its corridor at %.0f cm"), SAt(k)));
			}
			if (P.BHi[k] - P.BLo[k] < T.MinCorridorCm - 1e-6)
			{
				Fail(k, FString::Printf(TEXT("the corridor is %.2f cm at %.0f cm (%.1f at least)"), P.BHi[k] - P.BLo[k], SAt(k), T.MinCorridorCm));
			}
			// Stopped only at a planned stop; under the floor only there, or where
			// a slow crossing is allowed.
			if (k >= KRise && !AtStop[k] && P.V[k] <= StopSpeedCmS + 1e-6)
			{
				Fail(k, FString::Printf(TEXT("the robot comes to rest at %.0f cm (%.1f cm/s) without a planned stop"), SAt(k), P.V[k]));
			}
			else if (k >= KRise && !T.bAllowSlowCrossing && !AtStop[k] && P.V[k] < Floor - 1e-6)
			{
				Fail(k, FString::Printf(TEXT("the speed is %.1f cm/s at %.0f cm, under the floor %.1f"), P.V[k], SAt(k), Floor));
			}
			// No loaded wheel inside its keep-off window.
			for (int32 i = 0; i < N; ++i)
			{
				if ((P.Air[k] >> i) & 1u)
				{
					continue;
				}
				const double D = M.DistancePast(i, SAt(k), R.LegX[i]);
				if (D > DUp[i] + 1e-6 && D < DDown[i] - 1e-6)
				{
					Fail(k, FString::Printf(TEXT("the '%s' wheel is loaded %.1f cm past the edge, inside its keep-off window %.1f..%.1f, "
												 "at %.0f cm"),
								*LegName(i), D, DUp[i], DDown[i], SAt(k)));
				}
			}
			if (k <= K0)
			{
				continue;
			}
			if (IsDwell(P, k))
			{
				// A planned stop: at rest at k - 1 while the centre wheels and the
				// chassis move to where it leaves from, the wheels off the ground as
				// they came to rest (any lift for leaving begins after); every pose on
				// the way, and that one, stands there.
				const int32 KS = k - 1;
				for (int32 m = 1; m <= DwellPoseSteps; ++m)
				{
					const double  A = double(m) / DwellPoseSteps;
					FInstantQuery QD = Query(P, KS, P.Air[KS], P.Far[KS]);
					for (int32 Side = 0; Side < 2; ++Side)
					{
						if (Centre[Side] != INDEX_NONE)
						{
							QD.X[Centre[Side]] = FMath::Lerp(P.X[Side][KS], P.X[Side][k], A);
						}
					}
					CheckPose(KS, QD, FMath::Lerp(P.B[KS], P.B[k], A), TEXT("re-posing at the planned stop"));
				}
				continue;
			}
			const double V = FMath::Max(MinTimingSpeed, FMath::Max(P.V[k], P.V[k - 1]));
			if (FMath::Abs(P.B[k] - P.B[k - 1]) > Ds * BH / V + 1e-6)
			{
				Fail(k, FString::Printf(TEXT("the chassis height changes faster than BaseHeightSpeed at %.0f cm"), SAt(k)));
			}
			for (int32 Side = 0; Side < 2; ++Side)
			{
				const int32 Leg = Centre[Side];
				if (Leg == INDEX_NONE)
				{
					continue;
				}
				const double Dx = P.X[Side][k] - P.X[Side][k - 1];
				const bool	 bAir = ((P.Air[k] >> Leg) & 1u) && ((P.Air[k - 1] >> Leg) & 1u);
				// Within CenterShiftSpeed; a planted wheel aft no faster than
				// MaxAftWalkPerCm of the travel, but about a planned stop, all but
				// at rest, it may re-pose aft as fast as forward.
				const bool	 bAtRest = AtStop[k] && AtStop[k - 1] && FMath::Min(P.V[k], P.V[k - 1]) < 0.5 * Floor;
				const double AftMost = bAir || bAtRest ? Ds * CSS / V : FMath::Min(Ds * CSS / V, MaxAftWalkPerCm * Ds);
				if (Dx > Ds * CSS / V + 1e-6 || -Dx > AftMost + 1e-6)
				{
					Fail(k, FString::Printf(TEXT("the '%s' wheel walks faster than allowed at %.0f cm"), *LegName(Leg), SAt(k)));
				}
			}
		}
		// From the speed it enters at the robot brakes no harder than
		// Deceleration: no plan asks it to be slower sooner.
		for (int32 k = 0; k < NumS; ++k)
		{
			const double Least = FMath::Sqrt(FMath::Max(0.0, Entry * Entry - 2.0 * Dec * SAt(k)));
			if (P.V[k] < Least - 0.01)
			{
				Fail(FMath::Max(k, K0), FString::Printf(TEXT("from the speed it enters at, %.1f cm/s, the robot cannot brake to %.1f cm/s by %.0f cm "
															 "(Deceleration %.0f)"),
											Entry, P.V[k], SAt(k), Dec));
				break;
			}
		}
		// The tilt: the base turning -- pitch and roll together -- within
		// TiltRateDegPerSec and TiltAccelDegPerSec2 in time at the planned
		// speed, wherever it is planned (its easing may begin in the
		// level-in); held, not changing, through a planned stop.
		FVector2D LastRate = FVector2D::ZeroVector;
		double	  LastDt = 0.0;
		for (int32 k = 1; k < NumS; ++k)
		{
			const double	Dt = StepSeconds(P, k);
			const FVector2D Step(P.Pitch[k] - P.Pitch[k - 1], P.Roll[k] - P.Roll[k - 1]);
			const bool		bStop = P.Stop[k] || P.Stop[k - 1];
			const FVector2D R = Step / Dt;
			if (bStop && Step.Size() > 1e-9)
			{
				Fail(FMath::Max(k, K0), FString::Printf(TEXT("the tilt changes through the planned stop at %.0f cm"), SAt(k)));
			}
			if (R.Size() > Rate * (1.0 + 1e-6) + 1e-6)
			{
				Fail(FMath::Max(k, K0), FString::Printf(TEXT("the base turns faster than TiltRateDegPerSec at %.0f cm (%.2f deg/s)"), SAt(k), R.Size()));
			}
			if (k > 1)
			{
				const double A = ((R - LastRate) / (0.5 * (Dt + LastDt))).Size();
				if (A > TiltAcc * (1.0 + 1e-6) + 1e-6)
				{
					Fail(FMath::Max(k, K0), FString::Printf(TEXT("the base's turn rate changes faster than TiltAccelDegPerSec2 at %.0f cm (%.2f deg/s^2)"), SAt(k), A));
				}
			}
			LastRate = R;
			LastDt = Dt;
		}
		// And never inclined more than MaxPlannedTiltDeg -- its total tilt, not
		// pitch and roll each -- the easings included (an easing between two
		// tilts runs along the straight way between them, which leans no
		// further than the more inclined; a hold run into by a tight easing,
		// and the level-in's, are checked here all the same).
		const double MaxIncl = FMath::Max(0.0, static_cast<double>(T.MaxPlannedTiltDeg));
		for (int32 k = 0; k < NumS; ++k)
		{
			const double Incl = InclinationDeg(P.Pitch[k], P.Roll[k]);
			if (Incl > MaxIncl + 1e-6)
			{
				Fail(FMath::Max(k, K0), FString::Printf(TEXT("the base is inclined %.2f deg at %.0f cm (%+.2f pitch, %+.2f roll), over MaxPlannedTiltDeg %.2f"), Incl, SAt(k), P.Pitch[k], P.Roll[k], MaxIncl));
				break;
			}
		}
		// Holding its speed through the level-in where its stance can only keep
		// the margin at a steady speed.
		for (int32 k = 0; bHoldThroughLevelIn && k <= K0; ++k)
		{
			if (FMath::Abs(P.V[k] - In.EntrySpeedCmS) > 0.01)
			{
				Fail(K0, FString::Printf(TEXT("the drive stance cannot brake or speed up (%.2f cm inside its wheels so, %.1f at least), but the "
											  "plan changes the speed to %.1f cm/s at %.0f cm, before the level-in ends at %.0f cm"),
							 StanceMargin, Req, P.V[k], SAt(k), LevelFromS));
				break;
			}
		}
		// The level-in, from the drive stance to the first level pose: its
		// least margin by a straight move (reported).
		MeasureLevelIn(P);
		P.bVerified = bOk;
		TimeVerify += FPlatformTime::Seconds() - Started;
		return bOk;
	}

	void FPlanner::MeasureLevelIn(const FPlan& P) const
	{
		// The level-in from the drive stance -- legs where they are, the base at
		// its measured attitude -- to the plan's first level pose, taken as a
		// straight move of legs and base together, the wheels on the ground
		// those down in both and one coming down once within half a centimetre
		// of its surface: its least margin, braking and speeding up (holding
		// its speed, or standing still: without). Reported, not required: how
		// the legs are ordered through it is the executor's (its Prepare), and
		// a straight move is the worst of them for a wheel coming down.
		const int32 K0 = Index(LevelFromS);
		++Work;
		const FInstantResult							 R = M.Evaluate(Query(P, K0, P.Air[K0], P.Far[K0]), P.B[K0], true);
		TArray<FVector2D, TInlineAllocator<MaxPlanLegs>> From, To, XZ;
		uint32											 Down = 0, Lowered = 0;
		for (int32 i = 0; i < N; ++i)
		{
			From.Add(In.Legs[i].Measured);
			To.Add(FVector2D(R.LegX[i], R.LegZ[i]));
			const bool bUpAtK0 = (P.Air[K0] >> i) & 1u;
			Down |= (!In.Legs[i].bOffGround && !bUpAtK0 ? 1u : 0u) << i;
			Lowered |= (In.Legs[i].bOffGround && !bUpAtK0 ? 1u : 0u) << i;
		}
		XZ.SetNum(N);
		const double SFrom = bLevelInAtRest ? 0.0 : MarchBack(P, LevelFromS, LevelInSeconds);
		const bool	 bSteady = bLevelInAtRest || bHoldThroughLevelIn;
		LevelInMarginCm = TNumericLimits<double>::Max();
		for (int32 m = 0; m <= LevelInPoseSteps; ++m)
		{
			const double U = double(m) / LevelInPoseSteps;
			const double S = bLevelInAtRest ? 0.0 : FMath::Lerp(SFrom, LevelFromS, U);
			const double B = FMath::Lerp(B0, P.B[K0], U);
			const double Pitch = FMath::Lerp(In.AttitudeDeg.X, P.Pitch[K0], U);
			const double Roll = FMath::Lerp(In.AttitudeDeg.Y, P.Roll[K0], U);
			uint32		 Support = Down;
			for (int32 i = 0; i < N; ++i)
			{
				XZ[i] = FMath::Lerp(From[i], To[i], U);
				if (((Lowered >> i) & 1u) && XZ[i].Y <= M.LegZOnSurface(i, S, XZ[i].X, B, Pitch, Roll, false) + 0.5)
				{
					Support |= 1u << i;
				}
			}
			LevelInMarginCm = FMath::Min(LevelInMarginCm, M.PoseMarginAt(XZ, Support, 0, S, B, Pitch, Roll, !bSteady));
		}
	}

	void FPlanner::Evaluate(FPlan& P, bool bSearchX, bool bJoint) const
	{
		// Rounds of: the speed with these paths, the centre wheels' paths with
		// this speed (when searching them), the chassis height; the corners'
		// fore/aft follow their heights and the centre wheels' windows their
		// paths, so the geometry is taken again each round.
		for (int32 Round = 0; Round < 2; ++Round)
		{
			SpeedEnvelope(P);
			if (bSearchX)
			{
				CentrePathsDp(P);
				SpeedEnvelope(P);
			}
			Settle(P);
			P.bHeightOk = HeightPath(P);
			P.bBuilt = true;
		}
		SpeedEnvelope(P);
		P.bHeightOk = HeightPath(P);
		Verify(P);
		P.FirstFail = FirstFailOf(P);
		if (bJoint && bSearchX && !IsOk(P) && Centre[0] != INDEX_NONE && Centre[1] != INDEX_NONE)
		{
			// Where it still fails: both centre wheels searched together about
			// each stretch, kept if it fails less.
			FPlan J = P;
			for (const FIntPoint& R : FailRanges(J))
			{
				const int32 Pad = FMath::RoundToInt(JointPadCm / Ds);
				CentrePathsJoint(J, FMath::Max(0, R.X - Pad), FMath::Min(NumS - 1, R.Y + Pad));
			}
			for (int32 Round = 0; Round < 2; ++Round)
			{
				SpeedEnvelope(J);
				Settle(J);
				J.bHeightOk = HeightPath(J);
			}
			SpeedEnvelope(J);
			J.bHeightOk = HeightPath(J);
			Verify(J);
			J.FirstFail = FirstFailOf(J);
			// Kept only where it finds a level plan: a plan that still fails is
			// a base for the tilt and the stops, which take their stretches from
			// it, and fewer failing samples there is not a better one.
			if (IsOk(J))
			{
				P = MoveTemp(J);
			}
		}
	}

	TArray<FIntPoint> FPlanner::FailRanges(const FPlan& P) const
	{
		TArray<FIntPoint> Out;
		const int32		  Merge = FMath::RoundToInt(MergeFailCm / Ds);
		for (int32 k = 0; k < NumS; ++k)
		{
			if (P.Fail[k] == EFail::None)
			{
				continue;
			}
			if (Out.Num() > 0 && k - Out.Last().Y <= Merge)
			{
				Out.Last().Y = k;
			}
			else
			{
				Out.Add(FIntPoint(k, k));
			}
		}
		return Out;
	}

	bool FPlanner::ApplyTilt(FPlan& P, TConstArrayView<FIntPoint> Ranges, TConstArrayView<FVector2D> Tilts, TArray<FTiltGap>* OutTight,
		TConstArrayView<uint8> Modes, double RateShare) const
	{
		// Each stretch's tilt held over it, eased between stretches and from and
		// back to level on S-curves in time at the planned speed (TiltProfile):
		// never a jump, never a rate over TiltRateDegPerSec nor its change over
		// TiltAccelDegPerSec2 where an easing has the time; where one has not,
		// OutTight says how slow the robot must go over it.
		TArray<double> Seconds, S;
		Seconds.SetNum(NumS);
		S.SetNum(NumS);
		double Time = 0.0;
		for (int32 k = 0; k < NumS; ++k)
		{
			Time += k > 0 ? StepSeconds(P, k) : 0.0;
			Seconds[k] = Time;
			S[k] = SAt(k);
		}
		TArray<FTiltHold> Holds;
		for (int32 r = 0; r < Ranges.Num() && r < Tilts.Num(); ++r)
		{
			Holds.Add({ Ranges[r].X, Ranges[r].Y, Tilts[r] });
		}
		TArray<FVector2D> Tilt;
		const bool		  bFits = TiltProfile(Seconds, S, Holds, FMath::Max(0.1, RateShare * static_cast<double>(T.TiltRateDegPerSec)),
				   FMath::Max(0.1, static_cast<double>(T.TiltAccelDegPerSec2)), Tilt, OutTight, Modes);
		for (int32 k = 0; k < NumS; ++k)
		{
			P.Pitch[k] = Tilt[k].X;
			P.Roll[k] = Tilt[k].Y;
		}
		return bFits;
	}

	FPlanner::FPlan FPlanner::AtFloor(const FPlan& Base) const
	{
		// At the floor speed -- the least a crossing may slow to, so the fewest
		// wheels off the ground together -- the samples that do not stand: those
		// a tilt (or a stop) is for. (Where a plan failed only for its speeds,
		// paths or heights, neither is what it needs.)
		FPlan		P = Base;
		const int32 K0 = Index(LevelFromS);
		for (int32 k = K0; k < NumS; ++k)
		{
			P.VCap[k] = FMath::Min(Cruise, Floor);
		}
		Passes(P);
		ComputeTiming(P);
		for (int32 k = 0; k < NumS; ++k)
		{
			P.Fail[k] = EFail::None;
		}
		for (int32 k = K0; k < NumS; ++k)
		{
			if (!Stands(P, Query(P, k, P.Air[k], P.Far[k]), BGuessAt(P, k)))
			{
				P.Fail[k] = EFail::Static;
			}
		}
		return P;
	}

	bool FPlanner::RangeTilt(const FPlan& P, const FIntPoint& Range, double MaxTilt, FVector2D& Out, double& OutBeyond) const
	{
		OutBeyond = -1.0;
		auto Stands = [&](const FVector2D& D) { return StandsOver(P, Range, D); };
		// Within the limit: rings of the base's inclination, least first, and
		// of a ring's tilts that keep a pose there, the middle of the widest
		// arc of them (LeastInclinationAt): the one furthest from failing.
		if (PickLeastTilt(ETiltPick::Ring, TiltGridDeg, MaxTilt, Stands, Out))
		{
			return true;
		}
		// Beyond it, to twice the limit, just to say how much it would need:
		// first along the axes and the diagonals every 0.5 deg (cheap: a tilt
		// that keeps a pose there bounds the need), then the whole of every
		// ring of the grid between the limit and that bound, least first --
		// the least ring with a tilt that keeps a pose, and its first such tilt.
		const int32 From = FMath::FloorToInt(MaxTilt / TiltGridDeg + 1e-9) + 1;
		const int32 To = FMath::CeilToInt(2.0 * MaxTilt / TiltGridDeg - 1e-9);
		int32		Bound = INDEX_NONE;
		for (int32 nt = From; nt <= To && Bound == INDEX_NONE; ++nt)
		{
			const double t = nt * TiltGridDeg;
			if (FMath::Abs(t / 0.5 - FMath::RoundToDouble(t / 0.5)) > 1e-6 && nt != To)
			{
				continue;
			}
			const FVector2D Diag = TiltAtInclination(FVector2D(1.0, 1.0), t);
			for (const FVector2D& D : { FVector2D(t, 0), FVector2D(-t, 0), FVector2D(0, t), FVector2D(0, -t), Diag, FVector2D(-Diag.X, Diag.Y),
					 FVector2D(Diag.X, -Diag.Y), FVector2D(-Diag.X, -Diag.Y) })
			{
				if (Stands(D))
				{
					Bound = nt;
					break;
				}
			}
		}
		for (int32 nt = From; Bound != INDEX_NONE && nt <= Bound; ++nt)
		{
			const double t = nt * TiltGridDeg;
			for (const FVector2D& D : TiltCandidates(t, TiltGridDeg))
			{
				if (Stands(D))
				{
					Out = D;
					OutBeyond = t;
					return false;
				}
			}
		}
		return false;
	}

	bool FPlanner::StandsOver(const FPlan& P, const FIntPoint& Range, const FVector2D& Tilt) const
	{
		for (int32 k = Range.X; k <= Range.Y; k += (k + SearchEvery <= Range.Y ? SearchEvery : 1))
		{
			FInstantQuery Q = Query(P, k, P.Air[k], P.Far[k]);
			Q.PitchDeg = Tilt.X;
			Q.RollDeg = Tilt.Y;
			if (!Stands(P, Q, BGuessAt(P, k)))
			{
				return false;
			}
		}
		return true;
	}

	bool FPlanner::LeastTilt(const FPlan& Base, TArray<FIntPoint>& Ranges, TArray<FVector2D>& Tilts, TArray<uint8>& Has, double& OutNeed,
		FString& OutWhy, FPlan* OutFloor) const
	{
		const double MaxTilt = FMath::Max(0.0, static_cast<double>(T.MaxPlannedTiltDeg));
		OutNeed = 0.0;
		Ranges.Reset();
		Tilts.Reset();
		Has.Reset();
		FPlan P = AtFloor(Base);
		// And where the plan at its own speeds found no pose standing: at the
		// floor another set may be off the ground there and stand level, but the
		// tilted plan runs at the plan's speeds. Such a sample is a failure of
		// the tilt's stretches too, searched with the floor's sets: standing
		// level there at the floor, it takes the least tilt on the grid -- which
		// is what a corridor just under MinCorridorCm at speed needs (0.25-0.5
		// deg buys most of a centimetre of corridor; the plan verifies it at
		// its speeds). Searched with the sets the plan at its speeds has off the
		// ground (which the tilted plan, re-timed, need not keep), or only
		// widening the floor's stretches, the planner refuses crossings this
		// way plans (Ramms.Curb.TiltAtSpeedSamples 1 and 2, kept for
		// comparison).
		const int32	  Mode = TiltAtSpeedMode();
		TArray<int32> AtSpeed;
		for (int32 k = Index(LevelFromS); k < NumS; ++k)
		{
			if (P.Fail[k] == EFail::None && Base.Fail.IsValidIndex(k) && Base.Fail[k] == EFail::Static)
			{
				AtSpeed.Add(k);
				if (Mode == 2)
				{
					continue;
				}
				P.Fail[k] = EFail::Static;
				if (Mode == 1)
				{
					P.Air[k] = Base.Air[k];
					P.Far[k] = Base.Far[k];
					if (P.Touch.IsValidIndex(k) && Base.Touch.IsValidIndex(k))
					{
						P.Touch[k] = Base.Touch[k];
					}
				}
			}
		}
		Ranges = FailRanges(P);
		if (Mode == 2)
		{
			const int32 Merge = FMath::RoundToInt(MergeFailCm / Ds);
			for (const int32 k : AtSpeed)
			{
				for (int32 r = 0; r < Ranges.Num(); ++r)
				{
					FIntPoint&	R = Ranges[r];
					const int32 Lo = FMath::Min(R.X, k), Hi = FMath::Max(R.Y, k);
					const bool	bNear = k >= R.X - Merge && k <= R.Y + Merge;
					const bool	bApart = (r == 0 || Lo - Ranges[r - 1].Y > Merge) && (r + 1 == Ranges.Num() || Ranges[r + 1].X - Hi > Merge);
					if (bNear && bApart)
					{
						R = FIntPoint(Lo, Hi);
						break;
					}
				}
			}
		}
		if (Ranges.Num() == 0)
		{
			OutWhy = TEXT("nothing to tilt for");
			return false;
		}
		TArray<double> Beyond;
		auto		   Search = [&](int32 r) {
			  FVector2D	 Found = FVector2D::ZeroVector;
			  double	 B = -1.0;
			  const bool bOk = RangeTilt(P, Ranges[r], MaxTilt, Found, B);
			  Tilts[r] = Found;
			  Has[r] = bOk ? 1 : 0;
			  Beyond[r] = B;
		};
		Tilts.SetNum(Ranges.Num());
		Has.SetNum(Ranges.Num());
		Beyond.SetNum(Ranges.Num());
		for (int32 r = 0; r < Ranges.Num(); ++r)
		{
			Search(r);
		}
		bool bAll = true;
		for (int32 r = 0; r < Ranges.Num(); ++r)
		{
			const double Size = Has[r] ? InclinationDeg(Tilts[r].X, Tilts[r].Y) : Beyond[r];
			OutNeed = FMath::Max(OutNeed, Size < 0.0 ? TNumericLimits<double>::Max() : Size);
			if (Has[r])
			{
				continue;
			}
			if (bAll)
			{
				OutWhy = Beyond[r] > 0.0
					? FString::Printf(TEXT("at %.0f..%.0f cm the least tilt that keeps a pose inclines the base %.2f deg (%+.2f pitch, %+.2f roll) > "
										   "MaxPlannedTiltDeg %.2f (the inclination)"),
						  SAt(Ranges[r].X), SAt(Ranges[r].Y), Beyond[r], Tilts[r].X, Tilts[r].Y, MaxTilt)
					: FString::Printf(TEXT("at %.0f..%.0f cm no tilt keeps a pose within MaxPlannedTiltDeg %.2f, nor any along the axes or the "
										   "diagonals up to %.1f deg"),
						  SAt(Ranges[r].X), SAt(Ranges[r].Y), MaxTilt, 2.0 * MaxTilt);
			}
			bAll = false;
			if (PlanDebug())
			{
				const double t = FMath::FloorToDouble(MaxTilt / TiltGridDeg) * TiltGridDeg;
				for (const FVector2D D : { FVector2D(t, 0), FVector2D(-t, 0), FVector2D(0, t), FVector2D(0, -t) })
				{
					FString Line = FString::Printf(TEXT("dbg tilt (%s) %+.2f/%+.2f over %.0f..%.0f fails at"), *Base.Name, D.X, D.Y, SAt(Ranges[r].X),
						SAt(Ranges[r].Y));
					int32	Count = 0;
					for (int32 k = Ranges[r].X; k <= Ranges[r].Y; ++k)
					{
						FInstantQuery Q = Query(P, k, P.Air[k], P.Far[k]);
						Q.PitchDeg = D.X;
						Q.RollDeg = D.Y;
						FInstantResult R;
						if (!Feasible(Q, BGuessAt(P, k), &R) && Count++ < 12)
						{
							Line += FString::Printf(TEXT(" %.0f[%02x %+.1f..%+.1f L%d H%d m%.1f%s]"), SAt(k), P.Air[k], R.Lo - B0, R.Hi - B0, R.LoLeg,
								R.HiLeg, R.Margin, R.bReach ? TEXT("") : TEXT(" noreach"));
						}
					}
					Tried.Add(Line + FString::Printf(TEXT(" (%d)"), Count));
				}
			}
		}
		if (OutFloor)
		{
			*OutFloor = MoveTemp(P);
		}
		return bAll;
	}

	bool FPlanner::TiltPlan(const FPlan& Base, TConstArrayView<FIntPoint> Ranges, TConstArrayView<FVector2D> Tilts, TConstArrayView<int32> StopsAt,
		FPlan& Out, FString& OutWhy, bool bPlacements, TArray<int32>* OutFailing) const
	{
		// Tilt over those stretches (and stop at StopsAt) and plan again. An
		// easing with too little time slows the robot over it (the cap kept from
		// one attempt to the next); a ramp whose own poses fail widens the hold.
		// Where an easing between two stretches has too little time even at the
		// floor speed it runs into them: centred on the gap, then into the
		// earlier stretch only, then into the later only, then -- the smaller
		// stretch's tilt left out -- straight on; the first that verifies.
		FPlan	P = Base;
		FString Tight;
		auto	Run = [&](TArray<FIntPoint> Hold, TConstArrayView<FVector2D> TiltsV, uint8 Mode, FPlan& Last, TArray<int32>& OutTightPairs) {
			   TArray<double> TiltCap;
			   TiltCap.Init(NoCap, NumS);
			   TArray<uint8> Modes;
			   Modes.Init(Mode, Hold.Num() + 1);
			   OutTightPairs.Reset();
			   int32 SlowRounds = 0;
			   for (int32 Attempt = 0; Attempt < (Mode == 0 ? 4 : 3) + (bSlowTilt ? MaxSlowTiltRounds : 0); ++Attempt)
			   {
				   FPlan Q = Base;
				   for (int32 k = 0; k < NumS; ++k)
				   {
					   Q.Fail[k] = EFail::None;
					   Q.VCap[k] = FreeCap(k);
					   Q.Stop[k] = 0;
				   }
				   Q.TiltCap = TiltCap;
				   for (const int32 S : StopsAt)
				   {
					   Q.Stop[FMath::Clamp(S, 0, NumS - 1)] = 1;
				   }
				   Q.Stops = StopsAt.Num();
				   ApplyLevelInCap(Q);
				   // The easings timed on the base plan's speeds, then on the tilted
				   // plan's own (twice: the speeds move with the tilt).
				   ApplyTilt(Q, Hold, TiltsV, nullptr, Modes);
				   Evaluate(Q, false);
				   ApplyTilt(Q, Hold, TiltsV, nullptr, Modes);
				   Evaluate(Q, false);
				   if (!IsOk(Q))
				   {
					   FPlan QS = Q;
					   Evaluate(QS, true);
					   if (IsOk(QS) || QS.CountFails() < Q.CountFails())
					   {
						   Q = QS;
					   }
				   }
				   // The easings once more, on the speeds this plan keeps: the
				   // S-curve holds in time as the robot will drive it.
				   TArray<FTiltGap> Gaps;
				   ApplyTilt(Q, Hold, TiltsV, &Gaps, Modes);
				   for (int32 k = 0; k < NumS; ++k)
				   {
					   if (Q.Fail[k] == EFail::Height || Q.Fail[k] == EFail::Verify)
					   {
						   Q.Fail[k] = EFail::None;
					   }
				   }
				   Q.bHeightOk = HeightPath(Q);
				   Verify(Q);
				   Q.FirstFail = FirstFailOf(Q);
				   Last = Q;
				   if (IsOk(Q))
				   {
					   // Gentler easings where the plan's own speeds give them the
					   // time (GentleTiltShare of TiltRateDegPerSec): the same plan,
					   // its base turning slower, kept where it verifies so -- the
					   // limit is a bound, and a plan easing at it leaves nothing for
					   // the legs' lag and the landings' jolts.
					   if (GentleTiltOn())
					   {
						   FPlan			G = Q;
						   TArray<FTiltGap> GentleGaps;
						   if (ApplyTilt(G, Hold, TiltsV, &GentleGaps, Modes, GentleTiltShare) && GentleGaps.Num() == 0)
						   {
							   for (int32 k = 0; k < NumS; ++k)
							   {
								   G.Fail[k] = G.Fail[k] == EFail::Height || G.Fail[k] == EFail::Verify ? EFail::None : G.Fail[k];
							   }
							   G.bHeightOk = HeightPath(G);
							   Verify(G);
							   G.FirstFail = FirstFailOf(G);
							   if (IsOk(G))
							   {
								   Last = G;
							   }
						   }
					   }
					   return true;
				   }
				   bool bChanged = false;
				   // Easings with too little time between their stretches: slower
				   // over them, where a speed at or above the floor gives them the
				   // time (the stretches then held whole).
				   for (const FTiltGap& G : Gaps)
				   {
					   if (G.CapCmS < Floor - 1e-6)
					   {
						   OutTightPairs.AddUnique(G.Hold);
						   Tight = FString::Printf(TEXT("the tilt cannot be eased from one stretch to the next between %.0f and %.0f cm (%.2f s) at "
														   "the floor speed, and the stretches do not stand with the easing run into them"),
							   SAt(G.First), SAt(G.Last), G.NeedSeconds);
						   continue;
					   }
					   for (int32 k = FMath::Max(0, G.First); k <= FMath::Min(NumS - 1, G.Last); ++k)
					   {
						   if (TiltCap[k] > 0.97 * G.CapCmS + 1e-6)
						   {
							   TiltCap[k] = 0.97 * G.CapCmS;
							   bChanged = true;
						   }
					   }
				   }
				   if (!bChanged)
				   {
					   // Ramps whose poses fail: hold the tilt over them.
					   for (int32 k = 0; k < NumS; ++k)
					   {
						   if (Q.Fail[k] == EFail::None)
						   {
							   continue;
						   }
						   for (int32 r = 0; r < Hold.Num(); ++r)
						   {
							   // Widened within 40 cm, never into the next stretch's hold.
							   const int32 Reach = FMath::RoundToInt(40.0 / Ds);
							   const int32 Lowest = r > 0 ? Hold[r - 1].Y + 1 : 0;
							   const int32 Highest = r + 1 < Hold.Num() ? Hold[r + 1].X - 1 : NumS - 1;
							   if (k >= FMath::Max(Lowest, Hold[r].X - Reach) && k < Hold[r].X)
							   {
								   Hold[r].X = k;
								   bChanged = true;
							   }
							   else if (k > Hold[r].Y && k <= FMath::Min(Highest, Hold[r].Y + Reach))
							   {
								   Hold[r].Y = k;
								   bChanged = true;
							   }
						   }
					   }
				   }
				   if (!bChanged && bSlowTilt && SlowRounds < MaxSlowTiltRounds && SlowAttemptsLeft > 0)
				   {
					   --SlowAttemptsLeft;
					   // Samples that fail at the plan's speeds but stand still (the
					   // least tilt is taken where they stand at the floor): slower
					   // over them, never under the floor -- a tilt is not made
					   // larger for want of time the robot can take by slowing.
					   ++SlowRounds;
					   const int32 Pad = FMath::RoundToInt(SlowTiltPadCm / Ds);
					   for (int32 k = Index(LevelFromS); k < NumS; ++k)
					   {
						   if (Q.Fail[k] == EFail::None || Q.Fail[k] == EFail::Static)
						   {
							   continue;
						   }
						   for (int32 j = FMath::Max(0, k - Pad); j <= FMath::Min(NumS - 1, k + Pad); ++j)
						   {
							   const double Want = FMath::Max(Floor, SlowTiltShare * FMath::Max(Q.V[j], Floor));
							   if (TiltCap[j] > Want + 1e-6 && Q.V[j] > Floor + 1e-6)
							   {
								   TiltCap[j] = Want;
								   bChanged = true;
							   }
						   }
					   }
				   }
				   if (!bChanged || Stopped() || OverSlowWork())
				   {
					   break;
				   }
			   }
			   return false;
		};
		if (Stopped())
		{
			// (Another grid's plan is kept: no attempt made.)
			OutWhy = GridStoppedPrefix + StoppedWhy;
			if (OutFailing)
			{
				OutFailing->Reset();
			}
			return false;
		}
		TArray<int32>			TightPairs;
		FPlan					Last;
		const TArray<FIntPoint> Holds(Ranges.GetData(), Ranges.Num());
		if (Run(Holds, Tilts, 0, Last, TightPairs))
		{
			Out = Last;
			return true;
		}
		P = Last;
		if (OutFailing)
		{
			// The stretches the failure is about: each failing sample's nearest
			// (both of two it lies between). (Not the two an easing between found
			// too little time: a larger tilt only asks more time of it.)
			OutFailing->Reset();
			for (int32 k = Index(LevelFromS); k < NumS; ++k)
			{
				if (Last.Fail[k] == EFail::None)
				{
					continue;
				}
				int32 Nearest = INDEX_NONE;
				int32 Best = TNumericLimits<int32>::Max();
				for (int32 r = 0; r < Holds.Num(); ++r)
				{
					const int32 Gap = k < Holds[r].X ? Holds[r].X - k : (k > Holds[r].Y ? k - Holds[r].Y : 0);
					if (Gap < Best)
					{
						Best = Gap;
						Nearest = r;
					}
					if (r + 1 < Holds.Num() && k > Holds[r].Y && k < Holds[r + 1].X)
					{
						OutFailing->AddUnique(r);
						OutFailing->AddUnique(r + 1);
					}
				}
				if (Nearest != INDEX_NONE)
				{
					OutFailing->AddUnique(Nearest);
				}
			}
			OutFailing->Sort();
		}
		if (bPlacements && TightPairs.Num() > 0)
		{
			// Too little time even at the floor: into the earlier stretch only,
			// into the later only, then without the smaller stretch of each pair.
			for (const uint8 Mode : { uint8(1), uint8(2) })
			{
				TArray<int32> Ignored;
				if (Run(Holds, Tilts, Mode, Last, Ignored))
				{
					Out = Last;
					return true;
				}
			}
			TArray<FIntPoint> Fewer;
			TArray<FVector2D> FewerTilts;
			TSet<int32>		  Drop;
			for (const int32 g : TightPairs)
			{
				// The easing out of hold g into hold g + 1.
				if (g >= 0 && g + 1 < Holds.Num())
				{
					const FVector2D A = Tilts[g], B = Tilts[g + 1];
					Drop.Add(InclinationDeg(A.X, A.Y) < InclinationDeg(B.X, B.Y) ? g : g + 1);
				}
			}
			for (int32 r = 0; r < Holds.Num(); ++r)
			{
				if (!Drop.Contains(r))
				{
					Fewer.Add(Holds[r]);
					FewerTilts.Add(Tilts[r]);
				}
			}
			TArray<int32> Ignored;
			if (Drop.Num() > 0 && Fewer.Num() > 0 && Run(Fewer, FewerTilts, 0, Last, Ignored))
			{
				Out = Last;
				return true;
			}
		}
		if (PlanDebugAttempts())
		{
			for (int32 k = FMath::Max(0, Index(LevelFromS) - 1); k < NumS; ++k)
			{
				Tried.Add(FString::Printf(TEXT("dbgtilt (%s) %5.0f v %7.3f cap %5.1f by %d air %02x/%02x far %02x fail %d corr %+6.1f..%+6.1f b %+6.1f x "
											   "%+5.1f/%+5.1f m %6.2f tilt %+.5f/%+.5f%s"),
					*Base.Name, SAt(k), P.V[k], P.VCap[k], P.CapBy[k], P.Air[k], P.StaticAir[k], P.Far[k], static_cast<int32>(P.Fail[k]), P.BLo[k] - B0,
					P.BHi[k] - B0, P.B[k] - B0, P.X[0][k], P.X[1][k], P.Margin[k], P.Pitch[k], P.Roll[k], P.Stop[k] ? TEXT(" STOP") : TEXT("")));
			}
		}
		// The most inclined of the stretches' tilts, by its inclination.
		FVector2D Most = FVector2D::ZeroVector;
		for (const FVector2D& Tlt : Tilts)
		{
			Most = InclinationDeg(Tlt.X, Tlt.Y) > InclinationDeg(Most.X, Most.Y) ? Tlt : Most;
		}
		OutWhy = FString::Printf(TEXT("tilting %.2f deg (%+.2f pitch, %+.2f roll) where level fails, the plan still does not verify: %s"),
			InclinationDeg(Most.X, Most.Y), Most.X, Most.Y, *DescribeFail(P, P.FirstFail));
		if (!Tight.IsEmpty())
		{
			OutWhy += TEXT("; ") + Tight;
		}
		return false;
	}

	bool FPlanner::TryTilt(TConstArrayView<const FPlan*> Bases, FPlan& Out, FString& OutWhy) const
	{
		// The least tilt over the candidate plans (each its own centre-wheel
		// paths: where the centre wheels cross moves where the chassis is when
		// they do), each stretch's least by both picks, then planned, least
		// inclined first (SearchTilts).
		const double Started = FPlatformTime::Seconds();
		const double Limit = FMath::Max(0.0, static_cast<double>(T.MaxPlannedTiltDeg));
		const double MaxTilt = TiltCeilingDeg >= 0.0 ? FMath::Min(Limit, TiltCeilingDeg) : Limit;
		ON_SCOPE_EXIT
		{
			TimeTilt += FPlatformTime::Seconds() - Started;
		};
		if (MaxTilt < TiltGridDeg)
		{
			OutWhy = TEXT("no planned tilt is allowed (MaxPlannedTiltDeg 0)");
			return false;
		}
		const int32 Picks = TiltPicksMode();
		auto		SizeOf = [](TConstArrayView<FVector2D> Tilts) {
			   double Most = 0.0;
			   for (const FVector2D& Tlt : Tilts)
			   {
				   Most = FMath::Max(Most, InclinationDeg(Tlt.X, Tlt.Y));
			   }
			   return Most;
		};
		auto Describe = [&](const FBaseTilt& C, TConstArrayView<FVector2D> Tilts) {
			FString Stretches;
			for (int32 r = 0; r < C.Ranges.Num() && r < Tilts.Num(); ++r)
			{
				Stretches += FString::Printf(TEXT("%s%.0f..%.0f cm %+.2f/%+.2f"), r ? TEXT(", ") : TEXT(""), SAt(C.Ranges[r].X), SAt(C.Ranges[r].Y),
					Tilts[r].X, Tilts[r].Y);
			}
			return FString::Printf(TEXT("%.2f deg over %d stretch%s (%s)"), SizeOf(Tilts), C.Ranges.Num(), C.Ranges.Num() == 1 ? TEXT("") : TEXT("es"),
				*Stretches);
		};
		// The candidates' least tilts: as a search up to the same limit found
		// them, or searched now (and kept, searched up to the limit).
		const bool		  bAtLimit = MaxTilt >= Limit - 1e-9;
		const bool		  bCached = bAtLimit && bTiltBasesAtLimit && TiltBasesAtLimit.Num() == Bases.Num();
		TArray<FBaseTilt> Found;
		if (bCached)
		{
			Found = TiltBasesAtLimit;
			Tried.Add(TEXT("tilt: the candidates' least tilts as the search up to the same limit found them"));
		}
		for (const FPlan* B : Bases)
		{
			if (bCached)
			{
				break;
			}
			if (Stopped())
			{
				return false;
			}
			FBaseTilt& C = Found.AddDefaulted_GetRef();
			C.Base = B;
			C.bRing = LeastTilt(*B, C.Ranges, C.Ring, C.Has, C.Need, C.Why, &C.Floor);
			if (Picks != 1 && C.bRing && C.Ranges.Num() > 0)
			{
				// The square's least of each stretch, on the same plan -- only
				// where the rings found one for every stretch within the limit
				// (the squares sample the same tilts no finer; searched where the
				// rings found none they cost the most and find the least).
				C.bSquare = true;
				C.Square.SetNum(C.Ranges.Num());
				for (int32 r = 0; r < C.Ranges.Num() && C.bSquare; ++r)
				{
					C.bSquare = PickLeastTilt(ETiltPick::Square, TiltGridDeg, MaxTilt, [&](const FVector2D& D) { return StandsOver(C.Floor, C.Ranges[r], D); }, C.Square[r]);
				}
			}
			C.bRing = C.bRing && Picks != 2;
			if (C.bRing)
			{
				C.Least = SizeOf(C.Ring);
			}
			if (C.bSquare)
			{
				C.Least = FMath::Min(C.Least, SizeOf(C.Square));
			}
			FString Line = C.bRing ? Describe(C, C.Ring) : C.Why;
			if (C.bSquare)
			{
				Line += TEXT("; per axis ") + Describe(C, C.Square);
			}
			Tried.Add(FString::Printf(TEXT("tilt for %s: %s"), *B->Name, *Line));
		}
		Found.StableSort([](const FBaseTilt& A, const FBaseTilt& B) {
			const bool	 bA = A.bRing || A.bSquare, bB = B.bRing || B.bSquare;
			const double LA = TiltStep(A.Least, TiltOrderDeg), LB = TiltStep(B.Least, TiltOrderDeg);
			return bA != bB ? bA : (bA ? LA < LB - 1e-9 : A.Need < B.Need);
		});
		if (bAtLimit && !bCached)
		{
			TiltBasesAtLimit = Found;
			bTiltBasesAtLimit = true;
		}
		if (Found.Num() == 0)
		{
			OutWhy = TEXT("nothing to tilt for");
			return false;
		}
		// The few candidate plans whose least tilts are least (a plan costs tens
		// to hundreds of milliseconds), each by both picks.
		TArray<FTiltSearchCandidate> Cands;
		int32						 FirstOf[2] = { INDEX_NONE, INDEX_NONE };
		for (int32 b = 0; b < Found.Num() && b < MaxFallbackCandidates; ++b)
		{
			const FBaseTilt& C = Found[b];
			for (const ETiltPick Pick : { ETiltPick::Ring, ETiltPick::Square })
			{
				if (Pick == ETiltPick::Ring ? C.bRing : C.bSquare)
				{
					int32& First = FirstOf[Pick == ETiltPick::Ring ? 0 : 1];
					First = First == INDEX_NONE ? Cands.Num() : First;
					Cands.Add({ b, Pick, Pick == ETiltPick::Ring ? C.Ring : C.Square });
				}
			}
		}
		// What tilting more would need: the least of the candidate plans whose
		// least tilt is over the limit.
		double Beyond = -1.0;
		for (const FBaseTilt& C : Found)
		{
			if (!C.bRing && !C.bSquare && C.Need > MaxTilt && C.Need < TNumericLimits<double>::Max())
			{
				Beyond = Beyond < 0.0 ? C.Need : FMath::Min(Beyond, C.Need);
			}
		}
		FString Why = Found[0].Why;
		bool	bFirstPlan = true;
		auto	PickName = [](const FTiltSearchCandidate& C) { return C.Pick == ETiltPick::Square ? TEXT(", per axis") : TEXT(""); };
		auto	HowOf = [](const FTiltTry& Try, ETiltPick Pick) {
			   return Try.Way == 0 ? (Pick == ETiltPick::Square ? TEXT("the least larger axis") : TEXT("the least inclination"))
								   : (Pick == ETiltPick::Square ? TEXT("its direction, per axis") : TEXT("its direction"));
		};
		auto Larger = [&](const FTiltTry& Try, const FBaseTilt& C) {
			FString Which, Dirs;
			for (const int32 r : Try.Larger)
			{
				Which += FString::Printf(TEXT("%s%.0f..%.0f cm"), Which.IsEmpty() ? TEXT("") : TEXT(", "), SAt(C.Ranges[r].X), SAt(C.Ranges[r].Y));
				Dirs += FString::Printf(TEXT("%s%+.2f/%+.2f"), Dirs.IsEmpty() ? TEXT("") : TEXT(", "), Try.Tilts[r].X, Try.Tilts[r].Y);
			}
			return FString::Printf(TEXT("%.2f deg larger at %s (%s, %s)"), Try.More, *Which, *Dirs, HowOf(Try, Cands[Try.Candidate].Pick));
		};
		const FTiltSearchResult R = SearchTilts(
			Cands, MaxTilt, TiltGridDeg, MakeArrayView(TiltEscalationDeg),
			// (Slowed for a tilt less than one found, a few larger tries; slowed
			// where none was found, the candidates' least tilts only.)
			bSlowTilt ? (TiltCeilingDeg >= 0.0 ? FMath::Min(MaxTiltPlans(), SlowTiltLargerTries) : SlowTiltWhenNoneLargerTries) : MaxTiltPlans(),
			[&](int32 c, int32 r, const FVector2D& D) {
				const FBaseTilt& C = Found[Cands[c].Base];
				return C.Ranges.IsValidIndex(r) && StandsOver(C.Floor, C.Ranges[r], D);
			},
			[&](const FTiltTry& Try, TArray<int32>& OutFailing) {
				if (Stopped() || OverSlowWork())
				{
					// (No failing stretches: the search runs out at once.)
					return false;
				}
				const FTiltSearchCandidate& SC = Cands[Try.Candidate];
				const FBaseTilt&			C = Found[SC.Base];
				// (The other placements of a tight easing on each pick's first
				// candidate's least tilt only: each placement is a plan.)
				const bool bPlacements = Try.More <= 0.0 && Try.Candidate == FirstOf[SC.Pick == ETiltPick::Ring ? 0 : 1];
				FString	   PlanWhy;
				if (TiltPlan(*C.Base, C.Ranges, Try.Tilts, {}, Out, PlanWhy, bPlacements, &OutFailing))
				{
					Tried.Add(FString::Printf(TEXT("tilt for %s%s: planned %s"), *C.Base->Name, PickName(SC),
						Try.More > 0.0 ? *Larger(Try, C) : *Describe(C, Try.Tilts)));
					return true;
				}
				Tried.Add(Try.More > 0.0 ? FString::Printf(TEXT("tilt for %s%s, %s: %s"), *C.Base->Name, PickName(SC), *Larger(Try, C), *PlanWhy)
										 : FString::Printf(TEXT("tilt for %s%s: %s"), *C.Base->Name, PickName(SC), *PlanWhy));
				if (bFirstPlan)
				{
					Why = PlanWhy;
					bFirstPlan = false;
				}
				return false;
			});
		Tried.Add(FString::Printf(TEXT("tilt search: %d tilted plan%s (%d with stretches made larger)%s"), R.Plans, R.Plans == 1 ? TEXT("") : TEXT("s"), R.LargerPlans,
			R.bCapped ? TEXT(", larger ones past Ramms.Curb.MaxTiltPlans untried") : TEXT("")));
		if (R.bFound)
		{
			const FTiltSearchCandidate& SC = Cands[R.Planned.Candidate];
			const FBaseTilt&			C = Found[SC.Base];
			if (R.Planned.More > 0.0)
			{
				TiltNote = FString::Printf(TEXT("the least tilt (%.2f deg) did not verify at the plan's speeds: %s; %d tilted plans"), SizeOf(SC.Tilts),
					*Larger(R.Planned, C), R.Plans);
			}
			else if (R.Plans > 1 || SC.Pick == ETiltPick::Square)
			{
				TiltNote = FString::Printf(TEXT("the least tilt%s; %d tilted plan%s"), SC.Pick == ETiltPick::Square ? TEXT(" per axis") : TEXT(""), R.Plans,
					R.Plans == 1 ? TEXT("") : TEXT("s"));
			}
			return true;
		}
		// What would cure it, named: a tilt over MaxPlannedTiltDeg that was not
		// tried (a larger one of a stretch, or another candidate's least), and a
		// search cut short.
		const double Over = R.CutDeg < 0.0 ? Beyond : (Beyond < 0.0 ? R.CutDeg : FMath::Min(R.CutDeg, Beyond));
		if (Over > 0.0 && !bFirstPlan)
		{
			Why += FString::Printf(TEXT("; tilts inclining the base more than MaxPlannedTiltDeg %.2f were not tried (the least of them %.2f deg)"), MaxTilt,
				Over);
		}
		if (R.bCapped)
		{
			Why += FString::Printf(TEXT("; the tilt search made %d tilted plans and left the more inclined larger tries untried "
										"(Ramms.Curb.MaxTiltPlans)"),
				R.Plans);
		}
		OutWhy = Why;
		return false;
	}

	bool FPlanner::TryMixed(TConstArrayView<const FPlan*> Bases, FPlan& Out, FString& OutWhy) const
	{
		// Stretch by stretch: the least tilt where one up to MaxPlannedTiltDeg
		// keeps a pose, a level planned stop at the start of each stretch where
		// none does (or, where every stretch has one, at the one that needs the
		// most) -- over the candidates that stand still most.
		const double Started = FPlatformTime::Seconds();
		ON_SCOPE_EXIT
		{
			TimeTilt += FPlatformTime::Seconds() - Started;
		};
		if (T.MaxPlannedTiltDeg < TiltGridDeg)
		{
			OutWhy = TEXT("no planned tilt is allowed (MaxPlannedTiltDeg 0)");
			return false;
		}
		TArray<const FPlan*> Order(Bases.GetData(), Bases.Num());
		Order.StableSort([](const FPlan& A, const FPlan& B) { return A.CountStatic() < B.CountStatic(); });
		OutWhy = TEXT("no stretch has a tilt within MaxPlannedTiltDeg where another needs a stop");
		const double MaxTilt = FMath::Max(0.0, static_cast<double>(T.MaxPlannedTiltDeg));
		bool		 bFirst = true;
		for (int32 c = 0; c < FMath::Min(MaxFallbackCandidates, Order.Num()) && !Stopped(); ++c)
		{
			TArray<FIntPoint> Ranges;
			TArray<FVector2D> Tilts;
			TArray<uint8>	  Has;
			double			  Need = 0.0;
			FString			  Why;
			FPlan			  AtFloorPlan;
			if (LeastTilt(*Order[c], Ranges, Tilts, Has, Need, Why, &AtFloorPlan))
			{
				// Every stretch has a tilt within the limit, yet the tilt
				// fallback found no plan that verifies: stop at the stretch that
				// needs the most and tilt the others (what a tighter limit would
				// leave to a stop).
				if (Ranges.Num() < 2)
				{
					continue;
				}
				int32 Most = 0;
				for (int32 r = 1; r < Ranges.Num(); ++r)
				{
					Most = InclinationDeg(Tilts[r].X, Tilts[r].Y) > InclinationDeg(Tilts[Most].X, Tilts[Most].Y) ? r : Most;
				}
				Has[Most] = 0;
			}
			if (!Has.Contains(1))
			{
				continue;
			}
			// Each stretch that has a tilt: the less inclined of the ring's and
			// the square's least (to TiltOrderDeg; alike, the ring's), then -- if
			// that plan does not verify -- the other where they differ.
			TArray<FVector2D> Other = Tilts;
			if (TiltPicksMode() != 1)
			{
				for (int32 r = 0; r < Ranges.Num(); ++r)
				{
					FVector2D Sq;
					if (!Has[r]
						|| !PickLeastTilt(ETiltPick::Square, TiltGridDeg, MaxTilt, [&](const FVector2D& D) { return StandsOver(AtFloorPlan, Ranges[r], D); }, Sq))
					{
						continue;
					}
					if (TiltStep(InclinationDeg(Sq.X, Sq.Y), TiltOrderDeg) < TiltStep(InclinationDeg(Tilts[r].X, Tilts[r].Y), TiltOrderDeg) - 1e-9)
					{
						Other[r] = Tilts[r];
						Tilts[r] = Sq;
					}
					else
					{
						Other[r] = Sq;
					}
				}
			}
			for (int32 Way = 0; Way < 2; ++Way)
			{
				const TArray<FVector2D>& Use = Way == 0 ? Tilts : Other;
				if (Way == 1 && Other == Tilts)
				{
					break;
				}
				TArray<FIntPoint> TiltRanges;
				TArray<FVector2D> TiltValues;
				TArray<int32>	  StopsAt;
				for (int32 r = 0; r < Ranges.Num(); ++r)
				{
					if (Has[r])
					{
						TiltRanges.Add(Ranges[r]);
						TiltValues.Add(Use[r]);
					}
					else
					{
						StopsAt.Add(FMath::Max(Index(LevelFromS), Ranges[r].X - 2));
					}
				}
				if (StopsAt.Num() > MaxStops)
				{
					break;
				}
				FString PlanWhy;
				if (TiltPlan(*Order[c], TiltRanges, TiltValues, StopsAt, Out, PlanWhy, c == 0 && Way == 0))
				{
					return true;
				}
				Tried.Add(FString::Printf(TEXT("tilt and stops for %s%s: %s"), *Order[c]->Name, Way ? TEXT(" (the other pick)") : TEXT(""), *PlanWhy));
				if (bFirst)
				{
					OutWhy = PlanWhy;
					bFirst = false;
				}
			}
		}
		return false;
	}

	bool FPlanner::TryStopsOver(TConstArrayView<const FPlan*> Bases, FPlan& Out, FString& OutWhy) const
	{
		// Over the candidates that stand standing still most, a few of them: a
		// stop mends speed, not a pose the robot has no matter how slow.
		TArray<const FPlan*> Order(Bases.GetData(), Bases.Num());
		Order.StableSort([](const FPlan& A, const FPlan& B) { return A.CountStatic() < B.CountStatic(); });
		FString First;
		for (int32 c = 0; c < FMath::Min(MaxFallbackCandidates, Order.Num()) && !Stopped(); ++c)
		{
			FString Why;
			if (TryStops(*Order[c], Out, Why))
			{
				return true;
			}
			Tried.Add(FString::Printf(TEXT("stops for %s: %s"), *Order[c]->Name, *Why));
			if (c == 0)
			{
				First = Why;
			}
		}
		OutWhy = First;
		return false;
	}

	bool FPlanner::TryStops(const FPlan& Base, FPlan& Out, FString& OutWhy) const
	{
		// Level, stopping where it cannot roll: the robot comes to rest at the
		// start of each stretch that fails, the centre wheels re-pose standing
		// still, and it goes on.
		TArray<int32> Stops;
		for (const FIntPoint& R : FailRanges(Base))
		{
			Stops.Add(FMath::Max(Index(LevelFromS), R.X - 2));
		}
		FPlan P;
		for (int32 Attempt = 0; Attempt <= MaxStops && Stops.Num() <= MaxStops; ++Attempt)
		{
			P = Base;
			for (int32 k = 0; k < NumS; ++k)
			{
				P.Pitch[k] = P.Roll[k] = 0.0;
				P.Stop[k] = 0;
				P.VCap[k] = FreeCap(k);
				P.TiltCap[k] = NoCap;
				P.Fail[k] = EFail::None;
			}
			ApplyLevelInCap(P);
			for (int32 S : Stops)
			{
				P.Stop[S] = 1;
			}
			P.Stops = Stops.Num();
			// The candidate's own centre-wheel paths first, then searched again
			// about the stops.
			FPlan PS = P;
			Evaluate(P, false);
			if (!IsOk(P))
			{
				Evaluate(PS, true);
				if (IsOk(PS) || PS.FailsLessThan(P))
				{
					P = PS;
				}
			}
			if (IsOk(P))
			{
				Out = P;
				return true;
			}
			bool bAdded = false;
			for (const FIntPoint& R : FailRanges(P))
			{
				const int32 At = FMath::Max(Index(LevelFromS), R.X - 2);
				bool		bHave = false;
				for (int32 S : Stops)
				{
					bHave = bHave || FMath::Abs(S - At) <= 2;
				}
				if (!bHave)
				{
					Stops.Add(At);
					bAdded = true;
					break;
				}
			}
			if (!bAdded)
			{
				break;
			}
		}
		OutWhy = DescribeFail(P, P.FirstFail);
		return false;
	}

	FString FPlanner::DescribeFail(const FPlan& P, int32 K) const
	{
		if (!P.Fail.IsValidIndex(K))
		{
			return P.FailWhy;
		}
		const bool			bStatic = P.Fail[K] == EFail::Static;
		const uint32		Air = bStatic ? P.StaticAir[K] : P.Air[K];
		const uint32		Far = bStatic ? P.StaticFar[K] : P.Far[K];
		const FInstantQuery Q = Query(P, K, Air, Far);
		FInstantResult		R;
		Feasible(Q, BGuessAt(P, K), &R);
		FString Up;
		for (int32 i = 0; i < N; ++i)
		{
			if ((Air >> i) & 1u)
			{
				Up += (Up.IsEmpty() ? TEXT("") : TEXT(", ")) + LegName(i);
			}
		}
		if (Up.IsEmpty())
		{
			Up = TEXT("no wheel");
		}
		const FString Where = FString::Printf(TEXT("%.0f cm on (%s off the ground%s)"), SAt(K), *Up, bStatic ? TEXT(", standing still") : TEXT(""));
		if (IsForbiddenSet(In.Model, Air))
		{
			return FString::Printf(TEXT("%s: a set the gait never lifts together (a same-side corner and centre, both fronts and a centre, "
										"or a centre and both rears)"),
				*Where);
		}
		auto Bound = [&](int32 Leg) {
			return Leg == -1 ? FString(TEXT("the belly")) : (Leg >= 0 ? FString::Printf(TEXT("leg '%s'"), *LegName(Leg)) : FString(TEXT("nothing")));
		};
		if (R.Lo > R.Hi)
		{
			return FString::Printf(TEXT("%s: %s needs the chassis at %.1f cm or more, but %s needs it at %.1f cm or less"), *Where,
				*Bound(R.LoLeg), R.Lo - B0, *Bound(R.HiLeg), R.Hi - B0);
		}
		if (R.Width() < T.MinCorridorCm)
		{
			return FString::Printf(TEXT("%s: %s and %s leave the chassis a corridor of %.2f cm (%.1f at least)"), *Where, *Bound(R.LoLeg),
				*Bound(R.HiLeg), R.Width(), T.MinCorridorCm);
		}
		if (R.Margin < Req)
		{
			return FString::Printf(TEXT("%s: the robot stands at most %.2f cm inside its wheels (%.1f at least)"), *Where, R.Margin, Req);
		}
		const int32 By = P.CapBy.IsValidIndex(K) ? P.CapBy[K] : INDEX_NONE;
		if (P.Fail[K] == EFail::Slow && (By == CapLevelIn || By == CapWalk || By == CapTilt || By == CapSeat || By >= CapAfterLevelIn))
		{
			// Slowed under the floor by something other than parting wheels.
			const FString What = By == CapLevelIn ? FString(TEXT("the level-in from the drive stance"))
				: By == CapWalk					  ? FString(TEXT("the centre wheels' walk (CenterShiftSpeed)"))
				: By == CapTilt					  ? FString(TEXT("easing the planned tilt (TiltRateDegPerSec, TiltAccelDegPerSec2)"))
				: By == CapSeat					  ? FString(TEXT("the seat's shift (ShiftSpeedCmS)"))
												  : FString::Printf(TEXT("the '%s' wheel's lift (%.2f s ahead of its window at %.1f cm), which may not begin before the "
																						   "level-in ends at %.1f cm"),
														*LegName(By - CapAfterLevelIn), P.Wheels[By - CapAfterLevelIn].TLift, P.Wheels[By - CapAfterLevelIn].SClear,
														LevelFromS);
			return FString::Printf(TEXT("%s: %s holds the robot to %.1f cm/s, under the floor %.1f"), *Where, *What, P.V[K], Floor);
		}
		if (P.Fail[K] == EFail::Slow)
		{
			// Which of them are up only for the speed, and what parting each pair
			// would cost.
			FString		 Roles;
			const double S = SAt(K);
			for (int32 i = 0; i < N; ++i)
			{
				if ((Air >> i) & 1u)
				{
					const FWheelPlan& W = P.Wheels[i];
					Roles += FString::Printf(TEXT(" %s %s [%.0f..%.0f, up %.0f..%.0f]"), *LegName(i),
						S < W.SClear ? TEXT("lifting") : (S >= W.SLand ? TEXT("landing") : TEXT("crossing")), W.SClear, W.SLand,
						W.SLiftStart, W.SLoaded);
				}
			}
			FString Pairs;
			for (int32 i = 0; i < N; ++i)
			{
				for (int32 j = 0; j < N; ++j)
				{
					if (i == j || !((Air >> i) & 1u) || !((Air >> j) & 1u) || P.Wheels[i].SLand > P.Wheels[j].SClear)
					{
						continue;
					}
					const FWheelPlan& Wi = P.Wheels[i];
					const FWheelPlan& Wj = P.Wheels[j];
					const bool		  bNoI = Feasible(Query(P, K, Air & ~(1u << i), Far | (S >= Wi.SLand ? (1u << i) : 0u)), BGuessAt(P, K));
					const bool		  bNoJ = Feasible(Query(P, K, Air & ~(1u << j), Far), BGuessAt(P, K));
					Pairs += FString::Printf(TEXT(" %s->%s %.1f cm/s (%s/%s)"), *LegName(i), *LegName(j),
						(Wj.SClear - Wi.SLand) / FMath::Max(1e-3, Wi.TPlant + LoadSeconds + Wj.TLift + GapMarginSeconds),
						bNoI ? TEXT("ok") : TEXT("no"), bNoJ ? TEXT("ok") : TEXT("no"));
				}
			}
			if (Roles.IsEmpty() && By >= 0 && By < N)
			{
				// Slowed here for a stretch that reaches here: a lift's lead or a
				// landing's tail elsewhere.
				const FWheelPlan& W = P.Wheels[By];
				return FString::Printf(TEXT("%s: the '%s' wheel's lift lead (%.2f s to its window at %.1f cm) or landing tail (%.2f s from %.1f cm), "
											"parted from wheels it may not be off the ground with, holds the robot to %.1f cm/s, under the floor %.1f"),
					*Where, *LegName(By), W.TLift, W.SClear, W.TPlant + LoadSeconds, W.SLand, P.V[K], Floor);
			}
			if (Roles.IsEmpty())
			{
				return FString::Printf(TEXT("%s: the speed is %.1f cm/s, under the floor %.1f (the ramps between the caps about it)"), *Where, P.V[K],
					Floor);
			}
			return FString::Printf(TEXT("%s: the wheels' lifts and landings overlap unless the robot slows under the floor %.1f cm/s "
										"(v %.1f;%s;%s)"),
				*Where, Floor, P.V[K], *Roles, *Pairs);
		}
		const FString* At = P.WhyAt.Find(K);
		return FString::Printf(TEXT("%s: %s"), *Where, At ? **At : *P.FailWhy);
	}

	void FPlanner::Output(const FPlan& P, EPlanClass Class, FCurbSchedule& Out) const
	{
		Out.Class = Class;
		Out.Origin = In.Origin;
		Out.Heading = M.GetHeading();
		Out.Terrain = In.Terrain;
		Out.Wheels = P.Wheels;
		Out.SampleCm = Ds;
		Out.LevelFromS = LevelFromS;
		Out.LevelInSeconds = LevelInSeconds;
		Out.LevelInLatestS = LevelInLatest;
		// Where the executor begins the level-in: at once where the planner has
		// it begin as the crossing does, else the level-in's time before its end
		// at the planned speed.
		const bool bEarly = bLevelInAtRest || bLevelInAtOnce || bHoldThroughLevelIn || In.Terrain.Near.SlopeDeg() > EarlyLevelInSlopeDeg;
		Out.LevelInFromS = bEarly ? 0.0 : FMath::Max(0.0, MarchBack(P, LevelFromS, LevelInSeconds));
		Out.bLevelInAtRest = bLevelInAtRest;
		Out.bLevelInAtOnce = bLevelInAtOnce;
		Out.bHoldSpeedThroughLevelIn = bHoldThroughLevelIn;
		Out.LevelInMarginCm = LevelInMarginCm;
		Out.ApproachMarginCm = StanceMargin;
		Out.Samples.SetNum(NumS);
		Out.MinMargin = TNumericLimits<double>::Max();
		Out.NarrowestCorridorCm = TNumericLimits<double>::Max();
		Out.PlannedStops = 0;
		const int32 K0 = Index(LevelFromS);
		double		MaxPitch = 0.0, MaxRoll = 0.0, MaxIncl = 0.0;
		Out.TiltRangeCm = FVector2D::ZeroVector;
		bool bTilted = false;
		for (int32 k = 0; k < NumS; ++k)
		{
			FScheduleSample& S = Out.Samples[k];
			S.S = SAt(k);
			// Without the drive's slowing under the seat's forward shifts: the
			// executor slows the base by the seat's speed as it moves, not as
			// planned (the seat follows the plan by travel, so a base slowed on
			// the plan's schedule slows the seat with it).
			S.V = P.VNoFore.IsValidIndex(k) ? P.VNoFore[k] : P.V[k];
			S.B = P.B[k];
			S.PitchDeg = P.Pitch[k];
			S.RollDeg = P.Roll[k];
			S.X[0] = P.X[0][k];
			S.X[1] = P.X[1][k];
			S.AirborneMask = P.Air[k];
			S.Margin = P.Margin[k];
			S.SeatX = P.SeatX.IsValidIndex(k) ? P.SeatX[k] : In.Model.Seat.ZeroPivot.X;
			Out.bSeatPlanned = P.bSeatPlanned;
			Out.SeatEndX = P.bSeatPlanned ? P.SeatEndX : S.SeatX;
			S.BLo = P.BLo[k];
			S.BHi = P.BHi[k];
			S.bApproach = k < K0;
			S.bStop = P.Stop[k] != 0;
			S.DwellSeconds = DwellSeconds(P, k);
			Out.PlannedStops += P.Stop[k] != 0;
			if (k >= K0)
			{
				Out.MinMargin = FMath::Min(Out.MinMargin, P.Margin[k]);
				Out.NarrowestCorridorCm = FMath::Min(Out.NarrowestCorridorCm, P.BHi[k] - P.BLo[k]);
			}
			MaxPitch = FMath::Abs(P.Pitch[k]) > FMath::Abs(MaxPitch) ? P.Pitch[k] : MaxPitch;
			MaxRoll = FMath::Abs(P.Roll[k]) > FMath::Abs(MaxRoll) ? P.Roll[k] : MaxRoll;
			MaxIncl = FMath::Max(MaxIncl, InclinationDeg(P.Pitch[k], P.Roll[k]));
			if (FMath::Abs(P.Pitch[k]) > 1e-9 || FMath::Abs(P.Roll[k]) > 1e-9)
			{
				Out.TiltRangeCm = bTilted ? FVector2D(Out.TiltRangeCm.X, S.S) : FVector2D(S.S, S.S);
				bTilted = true;
			}
		}
		Out.MinMargin = Out.MinMargin == TNumericLimits<double>::Max() ? 0.0 : Out.MinMargin;
		Out.NarrowestCorridorCm = Out.NarrowestCorridorCm == TNumericLimits<double>::Max() ? 0.0 : Out.NarrowestCorridorCm;
		Out.PlannedTiltDeg = FVector2D(MaxPitch, MaxRoll);
		Out.PlannedMaxTiltDeg = MaxIncl;
		Out.VFloor = P.VFloor;
		// From the first lift as late as it could have begun (an eased lift
		// begins earlier by choice, not need: the same measure as un-eased) to
		// the last load.
		double First = TNumericLimits<double>::Max(), Last = 0.0;
		for (const FWheelPlan& W : P.Wheels)
		{
			if (!W.bHeldFromStart)
			{
				First = FMath::Min(First, W.TEase > 0.0 ? MarchBack(P, W.SClear, W.TLiftMin) : W.SLiftStart);
			}
			Last = FMath::Max(Last, W.SLoaded);
		}
		Out.PlannedCrossingSeconds = SecondsBetween(P, Index(First), Index(Last));

		// Must-be-down: the wheels whose landing a lift waits on -- those that
		// land before it and with which, both up, the robot would not stand.
		for (int32 j = 0; j < N; ++j)
		{
			FWheelPlan& Wj = Out.Wheels[j];
			Wj.MustBeDownMask = 0;
			const int32 K = Index(Wj.SLiftStart);
			for (int32 i = 0; i < N; ++i)
			{
				const FWheelPlan& Wi = P.Wheels[i];
				if (i == j || Wi.SLoaded > Wj.SLiftStart || Wj.bHeldFromStart)
				{
					continue;
				}
				const uint32 Set = P.Air[K] | (1u << i) | (1u << j);
				const uint32 Far = P.Far[K] & ~(1u << i);
				if (!Stands(P, Query(P, K, Set, Far), BGuessAt(P, K)))
				{
					Wj.MustBeDownMask |= 1u << i;
				}
			}
		}

		struct FEvent
		{
			double	S;
			FString Text;
		};
		TArray<FEvent> Events;
		auto		   State = [&](double S) {
			  const FScheduleSample* Sm = Out.SampleAt(S);
			  if (!Sm)
			  {
				  return FString();
			  }
			  FString Line = FString::Printf(TEXT("base %+.1f"), Sm->B - B0);
			  Line += (Sm->PitchDeg != 0.0 || Sm->RollDeg != 0.0) ? FString::Printf(TEXT(" tilt %+.2f/%+.2f"), Sm->PitchDeg, Sm->RollDeg)
																		  : FString(TEXT(" level"));
			  if (Centre[0] != INDEX_NONE || Centre[1] != INDEX_NONE)
			  {
				  Line += FString::Printf(TEXT("  centres %+.1f/%+.1f"), Sm->X[0] - CentreX0(0), Sm->X[1] - CentreX0(1));
			  }
			  return Line + FString::Printf(TEXT("  v %.1f"), Sm->V);
		};
		for (int32 i = 0; i < N; ++i)
		{
			const FWheelPlan& W = Out.Wheels[i];
			FString			  Must;
			for (int32 j = 0; j < N; ++j)
			{
				if ((W.MustBeDownMask >> j) & 1u)
				{
					Must += (Must.IsEmpty() ? TEXT("; after ") : TEXT(", ")) + LegName(j);
				}
			}
			Events.Add({ W.SLiftStart,
				W.bHeldFromStart
					? FString::Printf(TEXT("%7.1f  held   %-12s (up from the start; window %.1f..%.1f)  %s"), 0.0, *LegName(i), W.DUp, W.DDown,
						  *State(0.0))
					: FString::Printf(TEXT("%7.1f  lift   %-12s (clear by %.1f, %.2f s ahead%s)  %s"), W.SLiftStart, *LegName(i), W.SClear,
						  W.TLift, *Must, *State(W.SLiftStart)) });
			Events.Add({ W.SLand, FString::Printf(TEXT("%7.1f  land   %-12s (past %.1f cm; down %.2f s later, at %.1f)  %s"), W.SLand, *LegName(i), W.DDown, W.TPlant, W.SDown, *State(W.SLand)) });
		}
		if (K0 > 0 || bLevelInAtRest)
		{
			Events.Add({ LevelFromS,
				FString::Printf(TEXT("%7.1f  level  (levelled in from the drive stance %s, %.2f s; the stance stands %.2f cm inside its wheels "
									 "braking or speeding up, %.2f standing; a straight level-in %.2f at least)  %s"),
					LevelFromS,
					bLevelInAtRest			  ? TEXT("standing still, before it moves")
						: bHoldThroughLevelIn ? TEXT("at once, holding its speed")
						: bLevelInAtOnce	  ? TEXT("at once")
											  : TEXT("by here"),
					LevelInSeconds, StanceMargin, StanceStaticMargin, LevelInMarginCm, *State(LevelFromS)) });
		}
		for (int32 k = 0; k < NumS; ++k)
		{
			if (P.Stop[k])
			{
				const int32 Kn = FMath::Min(NumS - 1, k + 1);
				Events.Add({ SAt(k), FString::Printf(TEXT("%7.1f  stop   (planned: at rest %.2f s while the centre wheels re-pose %+.1f/%+.1f -> %+.1f/%+.1f "
														  "and the chassis %+.1f -> %+.1f)  %s"),
										 SAt(k), DwellSeconds(P, k), P.X[0][k] - CentreX0(0), P.X[1][k] - CentreX0(1), P.X[0][Kn] - CentreX0(0), P.X[1][Kn] - CentreX0(1), P.B[k] - B0, P.B[Kn] - B0, *State(SAt(k))) });
			}
		}
		if (bTilted)
		{
			Events.Add({ Out.TiltRangeCm.X, FString::Printf(TEXT("%7.1f  tilt   (eased to %+.2f pitch / %+.2f roll, level again by %.1f)  %s"), Out.TiltRangeCm.X, MaxPitch, MaxRoll, Out.TiltRangeCm.Y, *State(Out.TiltRangeCm.X)) });
		}
		Events.StableSort([](const FEvent& A, const FEvent& B) { return A.S < B.S; });
		for (const FEvent& E : Events)
		{
			Out.Lines.Add(E.Text);
		}
		// What sets the floor: the slowest sample once risen (planned stops
		// apart; the drive's slowing under the seat apart, said after), and the
		// cap nearest it that holds the speed that low.
		{
			const bool			  bFore = P.VNoFore.Num() == NumS;
			const TArray<double>& VRef = bFore ? P.VNoFore : P.V;
			const int32			  KRise = RiseEnd(P);
			const TArray<uint8>	  AtStop = StopRuns(P);
			int32				  KMin = INDEX_NONE;
			for (int32 k = KRise; k < NumS; ++k)
			{
				if (!AtStop[k] && (KMin == INDEX_NONE || VRef[k] < VRef[KMin]))
				{
					KMin = k;
				}
			}
			if (KMin != INDEX_NONE && VRef[KMin] < Cruise - 0.05)
			{
				int32 KCap = INDEX_NONE;
				for (int32 d = 0; d < NumS && KCap == INDEX_NONE; ++d)
				{
					for (const int32 j : { KMin - d, KMin + d })
					{
						if (P.CapBy.IsValidIndex(j) && P.CapBy[j] != INDEX_NONE && P.CapBy[j] != CapFore && P.VCap[j] <= VRef[KMin] + 0.5)
						{
							KCap = j;
							break;
						}
					}
				}
				FString Why = TEXT("the ramps between the caps about it");
				if (KCap != INDEX_NONE)
				{
					const int32 By = P.CapBy[KCap];
					Why = By == CapLevelIn ? FString(TEXT("the level-in"))
						: By == CapWalk	   ? FString(TEXT("the centre wheels' walk (CenterShiftSpeed)"))
						: By == CapTilt	   ? FString(TEXT("easing the planned tilt (TiltRateDegPerSec, TiltAccelDegPerSec2)"))
						: By == CapSeat	   ? FString(TEXT("the seat's shift (ShiftSpeedCmS)"))
						: By >= CapAfterLevelIn
						? FString::Printf(TEXT("the '%s' wheel's lift lead, fitted after the level-in"), *LegName(By - CapAfterLevelIn))
						: FString::Printf(TEXT("the '%s' wheel's lift lead or landing tail, parted from wheels it may not be off the ground with"),
							  *LegName(By));
					Why += FString::Printf(TEXT(" (capped at %.0f cm)"), SAt(KCap));
				}
				Out.Lines.Add(FString::Printf(TEXT("floor %.1f cm/s at %.0f cm: %s"), VRef[KMin], SAt(KMin), *Why));
			}
			// The drive under the seat's forward shifts: each stretch, and the
			// least the base runs there.
			for (int32 k = 0; bFore && k < NumS;)
			{
				if (P.CapBy[k] != CapFore)
				{
					++k;
					continue;
				}
				const int32 A = k;
				double		Least = P.V[k];
				for (; k < NumS && P.CapBy[k] == CapFore; ++k)
				{
					Least = FMath::Min(Least, P.V[k]);
				}
				Out.Lines.Add(FString::Printf(TEXT("drive slowed under the seat's forward shift from %.0f to %.0f cm: down to %.1f cm/s, the seat "
												   "making it up for the rider (ForeSlowDownCmS)"),
					SAt(A), SAt(k - 1), Least));
			}
		}
		if (PlanDebug())
		{
			// Each wheel's keep-off window, then one line per sample: its speed and
			// cap, the sets off the ground (as planned / standing still) and on the
			// far side, the failure kind, the corridor and chassis height (over
			// where it began), the centre wheels' fore/aft, the margin, the tilt,
			// and at the planned pose each wheel's distance past the edge (d) and
			// its leg's chassis x-z (xz), in leg order.
			FString Windows = TEXT("dbg windows");
			for (int32 i = 0; i < N; ++i)
			{
				Windows += FString::Printf(TEXT(" %s %.2f %.2f"), *LegName(i), DUp[i], DDown[i]);
			}
			Out.Lines.Add(Windows);
			for (int32 k = FMath::Max(0, K0 - 1); k < NumS; ++k)
			{
				const FInstantResult R = M.Evaluate(Query(P, k, P.Air[k], P.Far[k]), P.B[k], true);
				FString				 D = TEXT(" d"), XZ = TEXT(" xz");
				for (int32 i = 0; i < N; ++i)
				{
					D += FString::Printf(TEXT(" %.2f"), M.DistancePast(i, SAt(k), R.LegX[i]));
					XZ += FString::Printf(TEXT(" %.2f,%.2f"), R.LegX[i], R.LegZ[i]);
				}
				Out.Lines.Add(FString::Printf(TEXT("dbg %5.0f v %7.3f cap %5.1f by %d air %02x/%02x far %02x fail %d corr %+6.1f..%+6.1f (L%d H%d) b %+6.1f x "
												   "%+5.1f/%+5.1f m %6.2f seat %+5.1f tilt %+.5f/%+.5f%s%s%s"),
					SAt(k), P.V[k], P.VCap[k], P.CapBy.IsValidIndex(k) ? P.CapBy[k] : -1, P.Air[k], P.StaticAir[k], P.Far[k], static_cast<int32>(P.Fail[k]),
					P.BLo[k] - B0, P.BHi[k] - B0, R.LoLeg, R.HiLeg, P.B[k] - B0, P.X[0][k], P.X[1][k], P.Margin[k], P.SeatX.IsValidIndex(k) ? P.SeatX[k] : 0.0, P.Pitch[k], P.Roll[k], *D, *XZ,
					P.Stop[k] ? TEXT(" STOP") : TEXT("")));
			}
		}
	}

	FCurbSchedule FPlanner::Run()
	{
		const double  Started = FPlatformTime::Seconds();
		FCurbSchedule Out;
		FString		  Why;
		auto		  Done = [&](FCurbSchedule& S) -> FCurbSchedule& {
			 S.PlanMs = 1000.0 * (FPlatformTime::Seconds() - Started);
			 const FString Head = S.IsValid()
						 ? FString::Printf(TEXT("per-wheel plan: %s; floor %.1f cm/s, margin %.2f cm, corridor %.2f cm, tilt %.2f deg (%+.2f "
														 "pitch, %+.2f roll), %d planned stop%s, crossing %.1f s; planned in %.1f ms"),
							   PlanClassName(S.Class), S.VFloor, S.MinMargin, S.NarrowestCorridorCm, S.PlannedMaxTiltDeg, S.PlannedTiltDeg.X,
							   S.PlannedTiltDeg.Y, S.PlannedStops, S.PlannedStops == 1 ? TEXT("") : TEXT("s"), S.PlannedCrossingSeconds, S.PlanMs)
						 : FString::Printf(TEXT("per-wheel plan refused: %s (planned in %.1f ms)"), *S.Summary, S.PlanMs);
			 const FString Note = S.IsValid() && !TiltNote.IsEmpty() ? TEXT("; ") + TiltNote : FString();
			 S.PlanWork = Work;
			 S.Lines.Add(Head + Note);
			 S.Lines.Append(Tried);
			 S.Lines.Add(FString::Printf(TEXT("plan time: geometry %.1f ms, speed %.1f, paths %.1f, heights %.1f, verify %.1f, tilt %.1f; %d "
															   "instants feasible, work %lld"),
						 1000.0 * TimeGeometry, 1000.0 * TimeSpeed, 1000.0 * TimeDp, 1000.0 * TimeHeight, 1000.0 * TimeVerify, 1000.0 * TimeTilt,
						 CountFeasible, static_cast<long long>(Work)));
			 if (S.IsValid())
			 {
				 S.Summary = Head + Note;
			 }
			 return S;
		};
		if (!Setup(Why))
		{
			Out.Class = EPlanClass::Refused;
			Out.Summary = Why;
			return Done(Out);
		}

		// Level first: the centre wheels not walking, then walking knots, then
		// the search over their paths.
		struct FKnot
		{
			double XF, XLift, XLand, XR;
			/** Walked in the air: lifted where the fronts needed it, landed where
			 *  the rears need it -- no walk on the ground between, at the cost of
			 *  a longer swing. Tried even once a plan is good enough, for
			 *  PlantedWalkFloorShare to choose. */
			bool bAir = false;
		};
		const FKnot	  Knots[] = { { 4.0, 0.0, 0.0, -4.0 }, { 6.0, 0.0, 0.0, -6.0 }, { 8.0, 0.0, 0.0, -8.0 }, { 12.0, -4.0, 4.0, -12.0 },
			  { 16.0, -8.0, 8.0, -12.0 }, { 6.0, 6.0, -6.0, -6.0, true }, { 8.0, 8.0, -8.0, -8.0, true }, { 6.0, 0.0, -6.0, -6.0, true } };
		FPlan		  Best;
		bool		  bHaveBest = false;
		TArray<FPlan> Candidates;
		// Of two level plans the better: more floor up to the preferred one (to a
		// quarter of a cm/s), then the wider narrowest corridor, then the more
		// margin -- the plan furthest from its limits, so that a start a little
		// different draws the same. A level plan whose floor reaches the
		// preferred one ends the search for a faster one.
		// The level search keeps looking (the walking knots) while its best level
		// plan's floor is under PreferFloorShare of the floor speed (1.3: about
		// 11.4 cm/s at the default 8.75): a plan's floor is what the drive's lag
		// and the executor's caps then take from, and one close to the floor is
		// not kept in a crossing that has a faster one.
		const double PreferredFloor = FMath::Min(Cruise, PreferFloorShare() * Floor);
		auto		 Headroom = [&](const FPlan& P) {
			double Corridor = TNumericLimits<double>::Max(), Margin = TNumericLimits<double>::Max();
			for (int32 k = Index(LevelFromS); k < NumS && P.BLo.IsValidIndex(k); ++k)
			{
				Corridor = FMath::Min(Corridor, P.BHi[k] - P.BLo[k]);
				Margin = FMath::Min(Margin, P.Margin[k]);
			}
			return FVector(FMath::FloorToDouble(FMath::Min(P.VFloor, PreferredFloor) * 4.0) / 4.0, Corridor, Margin);
		};
		auto LevelBetter = [&](const FPlan& A, const FPlan& B) {
			if (!PreferFloorOn())
			{
				return A.VFloor > B.VFloor + 0.05;
			}
			const FVector HA = Headroom(A), HB = Headroom(B);
			if (HA.X != HB.X)
			{
				return HA.X > HB.X;
			}
			if (FMath::Abs(HA.Y - HB.Y) > 0.05)
			{
				return HA.Y > HB.Y;
			}
			return HA.Z > HB.Z + 0.05;
		};
		auto Enough = [&](const FPlan& P) { return IsOk(P) && (!PreferFloorOn() || P.VFloor >= PreferredFloor - 0.05); };
		auto Offer = [&](FPlan& P) {
			Candidates.Add(P);
			int32 Kinds[5] = { 0, 0, 0, 0, 0 };
			for (EFail F : P.Fail)
			{
				++Kinds[static_cast<int32>(F)];
			}
			FString HeightNote;
			for (int32 k = 0; k < NumS && HeightNote.IsEmpty(); ++k)
			{
				if (P.Fail[k] == EFail::Height)
				{
					HeightNote = FString::Printf(TEXT("; height fails first at %.0f cm:"), SAt(k));
					for (int32 j = FMath::Max(0, k - 3); j <= FMath::Min(NumS - 1, k + 3); ++j)
					{
						HeightNote += FString::Printf(TEXT(" [%.1f..%.1f v%.0f]"), P.BLo[j] - B0, P.BHi[j] - B0, P.V[j]);
					}
				}
			}
			Tried.Add(FString::Printf(TEXT("tried %s: %s, floor %.1f cm/s; failing samples: %d standing, %d slow, %d height, %d verify%s%s%s"),
				*P.Name, IsOk(P) ? TEXT("level") : TEXT("not level"), P.VFloor, Kinds[1], Kinds[2], Kinds[3], Kinds[4],
				P.FirstFail != INDEX_NONE ? TEXT("; first: ") : TEXT(""),
				P.FirstFail != INDEX_NONE ? *DescribeFail(P, P.FirstFail) : TEXT(""), *HeightNote));
			bool bBetter = !bHaveBest;
			if (!bBetter)
			{
				const bool bA = IsOk(P), bB = IsOk(Best);
				bBetter = bA != bB ? bA : (bA ? LevelBetter(P, Best) : P.FailsLessThan(Best));
			}
			if (bBetter)
			{
				Best = P;
				bHaveBest = true;
			}
		};
		if (In.bTryNoWalk)
		{
			FPlan P;
			InitPlan(P, TEXT("no walk"));
			Evaluate(P, false);
			Offer(P);
		}
		const bool bWalk = Centre[0] != INDEX_NONE || Centre[1] != INDEX_NONE;
		// A knot set planned: walked smoothly and as before; the smooth walk is
		// kept unless it does not verify or plans a floor more than
		// SmoothWalkFloorShare under the other's. Returns whether either walk
		// had paths; OutKept is the one kept.
		auto PlanKnots = [&](const FKnot& K, const TCHAR* Label, FPlan& OutKept, bool bPerSide = false) {
			FPlan Walked[2];
			bool  bWalked[2] = { false, false };
			for (int32 Smooth = 0; Smooth < (WalkSmoothCm() > 0.0 ? 2 : 1); ++Smooth)
			{
				FPlan& P = Walked[Smooth];
				InitPlan(P, Label);
				bool bOk = true;
				for (int32 Round = 0; Round < 4 && bOk; ++Round)
				{
					SpeedEnvelope(P);
					bOk = CentrePathsKnots(P, K.XF, K.XLift, K.XLand, K.XR, Smooth == 1, bPerSide);
					if (bOk && Round > 0)
					{
						P.bHeightOk = HeightPath(P);
						P.bBuilt = true;
					}
				}
				if (!bOk)
				{
					Tried.Add(FString::Printf(TEXT("tried %s %+.0f/%+.0f/%+.0f/%+.0f%s: no paths: %s"), Label, K.XF, K.XLift, K.XLand, K.XR,
						Smooth ? TEXT(" smooth") : TEXT(""), *P.FailWhy));
					continue;
				}
				SpeedEnvelope(P);
				P.bHeightOk = HeightPath(P);
				Verify(P);
				P.FirstFail = FirstFailOf(P);
				P.Name = FString::Printf(TEXT("%s %+.0f/%+.0f/%+.0f/%+.0f%s"), Label, K.XF, K.XLift, K.XLand, K.XR, Smooth ? TEXT(" smooth") : TEXT(""));
				bWalked[Smooth] = true;
			}
			if (!bWalked[0] && !bWalked[1])
			{
				return false;
			}
			const bool bSmoothKept = bWalked[1]
				&& (!bWalked[0] || (IsOk(Walked[1]) && (!IsOk(Walked[0]) || Walked[1].VFloor >= SmoothWalkFloorShare() * Walked[0].VFloor)));
			if (bWalked[0] && bWalked[1])
			{
				// The other is reported as tried.
				const FPlan& Other = Walked[bSmoothKept ? 0 : 1];
				Tried.Add(FString::Printf(TEXT("tried %s: %s, floor %.1f cm/s (the %s walk kept)"), *Other.Name,
					IsOk(Other) ? TEXT("level") : TEXT("not level"), Other.VFloor, bSmoothKept ? TEXT("smooth") : TEXT("plain")));
			}
			OutKept = Walked[bSmoothKept ? 1 : 0];
			return true;
		};
		// The choreography (Ramms.Curb.Choreography): the centre wheels walked
		// through the crossing -- forward while a front corner
		// is up, aft while a rear one is -- tried first and kept whenever it is
		// level and its floor is at least PlantedWalkFloorShare of the no-walk
		// plan's: the support's edge carried under the mass is what balances the
		// lifts, and the walk keeps the motion continuous.
		bool bChoreographed = false;
		if (bWalk && In.bTryKnots && ChoreographyOn() && !Stopped())
		{
			double X0 = 0.0;
			int32  Sides = 0;
			for (int32 Side = 0; Side < 2; ++Side)
			{
				if (Centre[Side] != INDEX_NONE)
				{
					X0 += CentreX0(Side);
					++Sides;
				}
			}
			X0 /= FMath::Max(1, Sides);
			const FKnot Chor = { ChoreographyCm(CVarCurbChoreographyFront) - X0, ChoreographyCm(CVarCurbChoreographyLift) - X0,
				ChoreographyCm(CVarCurbChoreographyLand) - X0, ChoreographyCm(CVarCurbChoreographyRear) - X0 };
			FPlan		Kept;
			if (PlanKnots(Chor, TEXT("choreography"), Kept, /*bPerSide=*/true))
			{
				const bool bFast = !bHaveBest || !IsOk(Best) || Kept.VFloor >= PlantedWalkFloorShare() * Best.VFloor - 1e-6;
				Offer(Kept);
				if (IsOk(Kept) && bFast)
				{
					if (Best.Name != Kept.Name)
					{
						Tried.Add(FString::Printf(TEXT("kept %s: level, floor %.1f cm/s, its centre wheels walking %.1f cm on the ground (%s: %.1f cm/s)"),
							*Kept.Name, Kept.VFloor, PlantedWalkCm(Kept), *Best.Name, Best.VFloor));
					}
					Best = Kept;
					bChoreographed = true;
				}
				else if (IsOk(Kept))
				{
					Tried.Add(FString::Printf(TEXT("%s level but its floor %.1f cm/s is under %.2f of %s's %.1f: not kept"), *Kept.Name,
						Kept.VFloor, PlantedWalkFloorShare(), *Best.Name, Best.VFloor));
				}
			}
		}
		if (bWalk && In.bTryKnots && !Enough(Best))
		{
			// The walks walked in the air are tried even once a plan is good
			// enough, for PlantedWalkFloorShare to choose; of the rest, no more
			// then.
			TArray<int32> Level;
			for (const FKnot& K : Knots)
			{
				if (Stopped())
				{
					break;
				}
				// (With PlantedWalkFloorShare 1 nothing would choose them over a
				// plan already good enough: the search ends there, as before.)
				if ((!K.bAir || PlantedWalkFloorShare() >= 1.0) && Enough(Best))
				{
					continue;
				}
				FPlan Kept;
				if (!PlanKnots(K, TEXT("knots"), Kept))
				{
					continue;
				}
				Offer(Kept);
				if (IsOk(Kept))
				{
					Level.Add(Candidates.Num() - 1);
				}
			}
			// Of the level plans fast enough (PlantedWalkFloorShare of the fastest
			// floor), the one whose centre wheels walk least on the ground: a
			// planted wheel walking fore/aft rolls at another speed than the robot,
			// and meets the ground or leaves it walking. (Not over the
			// choreography, whose walk is the point.)
			const double Share = PlantedWalkFloorShare();
			if (Share < 1.0 && Level.Num() > 1 && !bChoreographed)
			{
				double Fastest = 0.0;
				for (const int32 C : Level)
				{
					Fastest = FMath::Max(Fastest, Candidates[C].VFloor);
				}
				const FPlan* Calmest = nullptr;
				double		 Least = TNumericLimits<double>::Max();
				for (const int32 C : Level)
				{
					const FPlan& L = Candidates[C];
					const double Walk = PlantedWalkCm(L);
					if (L.VFloor >= Share * Fastest - 1e-6 && Walk < Least - 0.5)
					{
						Least = Walk;
						Calmest = &L;
					}
				}
				if (Calmest && Calmest->Name != Best.Name)
				{
					Tried.Add(FString::Printf(TEXT("kept %s: its centre wheels walk %.1f cm on the ground, floor %.1f cm/s (%s: %.1f cm, %.1f cm/s)"),
						*Calmest->Name, Least, Calmest->VFloor, *Best.Name, PlantedWalkCm(Best), Best.VFloor));
					Best = *Calmest;
				}
			}
		}
		if (bWalk && In.bTrySearch && !IsOk(Best) && !Stopped())
		{
			FPlan P;
			if (bHaveBest)
			{
				P = Best;
			}
			else
			{
				InitPlan(P, TEXT("search"));
			}
			P.Name = TEXT("search");
			for (int32 k = 0; k < NumS; ++k)
			{
				P.Fail[k] = EFail::None;
				P.VCap[k] = FreeCap(k);
			}
			ApplyLevelInCap(P);
			// (The search over both wheels' pairs, the costliest part, only
			// here: for a level plan.)
			Evaluate(P, true, true);
			Offer(P);
		}
		if (!bHaveBest)
		{
			FPlan P;
			InitPlan(P, TEXT("no walk"));
			Evaluate(P, false);
			Offer(P);
		}
		if (Stopped())
		{
			return StoppedPlan(Out);
		}
		if (IsOk(Best))
		{
			OutputEased(Best, EPlanClass::Level, Out);
			Out.Stage = EPlanStage::Level;
			Out.Lines.Add(FString::Printf(TEXT("centre wheels: %s, from %+.1f / %+.1f cm"), *Best.Name, CentreX0(0), CentreX0(1)));
			return Done(Out);
		}
		const FString LevelWhy = DescribeFail(Best, Best.FirstFail);
		// From here on no plan is level at its own speeds: where the other grid
		// has one, this one stops (Stopped).
		Stage = EPlanStage::LevelSlowed;
		if (Stopped())
		{
			return StoppedPlan(Out);
		}

		// Then level, slower: a candidate whose samples fail at its own speeds
		// but stand still -- the corridor of chassis heights moving faster than
		// the chassis may follow, an easing or a walk short of time -- planned
		// again slower over them, never under the floor (SlowTiltShare of its
		// speed there, SlowTiltPadCm either side, at most MaxSlowTiltRounds
		// times), before anything is tilted: level first, and slower rather than
		// tilted. The best few candidates (as the fallbacks take them).
		if (SlowBeforeTiltOn())
		{
			TArray<const FPlan*> Order;
			Order.Add(&Best);
			for (const FPlan& C : Candidates)
			{
				if (C.Name != Best.Name && Order.Num() < MaxFallbackCandidates)
				{
					Order.Add(&C);
				}
			}
			for (const FPlan* C : Order)
			{
				if (Stopped())
				{
					return StoppedPlan(Out);
				}
				bool bStatic = false;
				for (int32 k = Index(LevelFromS); k < NumS && !bStatic; ++k)
				{
					bStatic = C->Fail[k] == EFail::Static;
				}
				if (bStatic || !C->bBuilt)
				{
					continue;
				}
				FPlan		S = *C;
				const int32 Pad = FMath::RoundToInt(SlowTiltPadCm / Ds);
				for (int32 Round = 0; Round < MaxSlowTiltRounds && !IsOk(S) && !Stopped(); ++Round)
				{
					bool bSlowed = false;
					for (int32 k = Index(LevelFromS); k < NumS; ++k)
					{
						if (S.Fail[k] == EFail::None)
						{
							continue;
						}
						for (int32 j = FMath::Max(0, k - Pad); j <= FMath::Min(NumS - 1, k + Pad); ++j)
						{
							const double Want = FMath::Max(Floor, SlowTiltShare * FMath::Max(S.V[j], Floor));
							if (S.TiltCap[j] > Want + 1e-6 && S.V[j] > Floor + 1e-6)
							{
								S.TiltCap[j] = Want;
								bSlowed = true;
							}
						}
					}
					if (!bSlowed)
					{
						break;
					}
					for (int32 k = 0; k < NumS; ++k)
					{
						S.Fail[k] = EFail::None;
					}
					Evaluate(S, false);
				}
				Tried.Add(FString::Printf(TEXT("tried %s slower where it failed: %s, floor %.1f cm/s"), *C->Name, IsOk(S) ? TEXT("level") : TEXT("not level"),
					S.VFloor));
				if (Stopped())
				{
					return StoppedPlan(Out);
				}
				if (IsOk(S))
				{
					S.Name = C->Name + TEXT(", slower");
					OutputEased(S, EPlanClass::Level, Out);
					Out.Stage = EPlanStage::LevelSlowed;
					Out.Lines.Add(FString::Printf(TEXT("centre wheels: %s, from %+.1f / %+.1f cm"), *S.Name, CentreX0(0), CentreX0(1)));
					return Done(Out);
				}
			}
		}

		// Then the least tilt, over every candidate's centre-wheel paths.
		Stage = EPlanStage::Tilt;
		if (Stopped())
		{
			return StoppedPlan(Out);
		}
		TArray<const FPlan*> Bases;
		Bases.Add(&Best);
		for (const FPlan& C : Candidates)
		{
			if (C.Name != Best.Name)
			{
				Bases.Add(&C);
			}
		}
		FPlan	   Tilted;
		FString	   TiltWhy;
		const bool bTilted = TryTilt(Bases, Tilted, TiltWhy);
		if (Stopped())
		{
			return StoppedPlan(Out);
		}
		// Then the least tilt again, its plans made slower over the samples that
		// fail at their own speeds (never under the floor; at most
		// SlowTiltLargerTries tries made larger, MaxSlowTiltAttempts slowed
		// attempts in all): for a tilt less than the one found at the plans'
		// speeds (by SlowTiltGainDeg) -- a stretch is not tilted more for want of
		// time the robot can take by slowing, level first, and a plan is not
		// slowed where that does not take its tilt down -- and, where none was
		// found within MaxPlannedTiltDeg at the plans' speeds, for any up to it
		// (Ramms.Curb.SlowTiltWhenNone) before the planned stops: a tilt the
		// limit allows is not left for want of the time slowing gives.
		if (SlowBeforeTiltOn())
		{
			double Most = 0.0;
			for (int32 k = 0; bTilted && k < NumS; ++k)
			{
				Most = FMath::Max(Most, InclinationDeg(Tilted.Pitch[k], Tilted.Roll[k]));
			}
			const bool bLess = bTilted && Most - SlowTiltGainDeg >= TiltGridDeg - 1e-9;
			const bool bAny = !bTilted && SlowTiltWhenNoneOn();
			if (bLess || bAny)
			{
				bSlowTilt = true;
				TiltCeilingDeg = bLess ? Most - SlowTiltGainDeg : -1.0;
				SlowWorkLimit = bAny ? Work + static_cast<int64>(SlowTiltWhenNoneWorkShare * static_cast<double>(Work)) : -1;
				FPlan		  Slow;
				FString		  SlowWhy;
				const FString FastNote = TiltNote;
				const bool	  bSlowFound = TryTilt(Bases, Slow, SlowWhy);
				if (OverSlowWork())
				{
					SlowWhy += TEXT("; its work spent (SlowTiltWhenNoneWorkShare)");
				}
				bSlowTilt = false;
				TiltCeilingDeg = -1.0;
				SlowWorkLimit = -1;
				if (Stopped())
				{
					return StoppedPlan(Out);
				}
				if (bSlowFound)
				{
					OutputEased(Slow, EPlanClass::Tilt, Out);
					Out.Stage = EPlanStage::Tilt;
					Out.Lines.Add(bTilted ? FString::Printf(TEXT("slowed where its least tilt did not verify at its speeds: %.2f deg of inclination, where "
																 "the plan at its speeds tilted %.2f"),
												Out.PlannedMaxTiltDeg, Most)
										  : FString::Printf(TEXT("slowed where its least tilt did not verify at its speeds: %.2f deg of inclination, where "
																 "the plans at their speeds found no tilt within %.2f"),
												Out.PlannedMaxTiltDeg, static_cast<double>(T.MaxPlannedTiltDeg)));
					return Done(Out);
				}
				TiltNote = FastNote;
				if (bAny)
				{
					TiltWhy += FString::Printf(TEXT("; slowed where they failed, none either: %s"), *SlowWhy);
				}
			}
		}
		if (bTilted)
		{
			OutputEased(Tilted, EPlanClass::Tilt, Out);
			Out.Stage = EPlanStage::Tilt;
			return Done(Out);
		}
		// Then, where a stretch has no tilt within the limit, a planned stop
		// there (and the least tilt elsewhere); then level with planned stops.
		Stage = EPlanStage::Stops;
		if (Stopped())
		{
			return StoppedPlan(Out);
		}
		FString StopWhy = TEXT("planned stops are not allowed (bAllowPlannedStop)");
		if (T.bAllowPlannedStop)
		{
			FPlan	WithStops;
			FString MixedWhy;
			if (TryMixed(Bases, WithStops, MixedWhy))
			{
				OutputEased(WithStops, EPlanClass::PlannedStop, Out);
				Out.Stage = EPlanStage::Stops;
				return Done(Out);
			}
			if (Stopped())
			{
				return StoppedPlan(Out);
			}
			if (TryStopsOver(Bases, WithStops, StopWhy))
			{
				OutputEased(WithStops, EPlanClass::PlannedStop, Out);
				Out.Stage = EPlanStage::Stops;
				return Done(Out);
			}
		}
		Stage = EPlanStage::Refused;
		if (Stopped())
		{
			return StoppedPlan(Out);
		}
		Output(Best, EPlanClass::Refused, Out);
		Out.Class = EPlanClass::Refused;
		Out.Stage = EPlanStage::Refused;
		Out.Summary = FString::Printf(TEXT("no level plan: %s; %s; and no planned stop: %s"), *LevelWhy, *TiltWhy, *StopWhy);
		return Done(Out);
	}

	bool FPlanner::Stopped() const
	{
		if (!StoppedWhy.IsEmpty())
		{
			return true;
		}
		if (!Race || !Race->bDone[1 - Slot].load(std::memory_order_acquire))
		{
			return false;
		}
		const int32		 Other = 1 - Slot;
		const EPlanStage Theirs = Race->Stage[Other];
		if (Theirs == EPlanStage::Refused)
		{
			// Refused: nothing this grid makes loses to it.
			return false;
		}
		// Their plan outranks anything left to this grid -- and is kept against
		// it whatever this grid's work comes to (off the move always; on it,
		// once this grid's work is enough that theirs can no longer be over its
		// budget against it: it only grows).
		if (static_cast<uint8>(Theirs) > static_cast<uint8>(Stage) && (!Race->bBudget || !OverGridBudget(Race->Work[Other], Work)))
		{
			StoppedWhy = FString::Printf(TEXT("the other grid's plan (%s) ranks above anything left to this one"),
				Theirs == EPlanStage::Level ? TEXT("level") : (Theirs == EPlanStage::LevelSlowed ? TEXT("level, slowed") : (Theirs == EPlanStage::Tilt ? TEXT("tilt") : TEXT("planned stops"))));
			return true;
		}
		if (Race->bBudget && OverGridBudget(Work, Race->Work[Other]))
		{
			StoppedWhy = FString::Printf(TEXT("its work (%lld) over the budget on the move against the other grid's plan (%lld)"), static_cast<long long>(Work),
				static_cast<long long>(Race->Work[Other]));
			return true;
		}
		return false;
	}

	FCurbSchedule& FPlanner::StoppedPlan(FCurbSchedule& Out) const
	{
		Out = FCurbSchedule();
		Out.Class = EPlanClass::Refused;
		Out.Stage = EPlanStage::Refused;
		Out.Summary = GridStoppedPrefix + StoppedWhy;
		Out.PlanWork = Work;
		return Out;
	}

	bool AnchorPlanOrigin(FPlannerInput& In, double SampleCm, double PhaseShare)
	{
		// The travel from the chassis origin to the edge along the heading; the
		// origin moved back by what that falls short of a whole number of
		// samples (under one), at the near surface's height there. The plan's
		// samples then lie at the same places on the terrain from every start --
		// a start a fraction of a sample further on no longer moves every
		// sample against the terrain, which made a crossing's plan (its class,
		// tilt and floor) turn on a fraction of a centimetre -- and the robot
		// begins under one sample into its plan.
		const FVector2D H = In.Heading.GetSafeNormal();
		const double	Cos = FVector2D::DotProduct(H, In.Terrain.EdgeNormal);
		if (!In.Terrain.bValid || H.IsNearlyZero() || Cos < 1e-3 || SampleCm <= 0.0)
		{
			return false;
		}
		const double ToEdge = -In.Terrain.DistancePastEdge(In.Origin) / Cos;
		if (ToEdge <= 0.0)
		{
			return false;
		}
		const double Phase = FMath::Clamp(PhaseShare, 0.0, 1.0) * SampleCm;
		const double Back = SampleCm * FMath::CeilToDouble((ToEdge - Phase) / SampleCm - 1e-9) - (ToEdge - Phase);
		if (Back <= 1e-9)
		{
			return false;
		}
		const FVector2D Was = In.Origin;
		In.Origin -= H * Back;
		In.OriginZ += In.Terrain.Near.HeightAt(In.Origin) - In.Terrain.Near.HeightAt(Was);
		return true;
	}

	void AtPlanOrigin(FPlannerInput& In, const FCurbSchedule& Plan)
	{
		In.OriginZ += In.Terrain.Near.HeightAt(Plan.Origin) - In.Terrain.Near.HeightAt(In.Origin);
		In.Origin = Plan.Origin;
	}

	bool OverGridBudget(int64 Mine, int64 Theirs)
	{
		const double Share = GridBudgetShare();
		const double Over = static_cast<double>(FMath::Max(0, CVarCurbGridBudgetWork.GetValueOnAnyThread()));
		return Share > 0.0 && static_cast<double>(Mine) > Share * static_cast<double>(FMath::Max<int64>(0, Theirs)) + Over;
	}

	bool IsBetterPlan(const FCurbSchedule& A, const FCurbSchedule& B)
	{
		auto Rank = [](const FCurbSchedule& S) {
			if (!S.IsValid())
			{
				return 0;
			}
			if (S.Stage != EPlanStage::Refused)
			{
				return static_cast<int32>(S.Stage);
			}
			// A plan that does not say its stage: its class's.
			return S.Class == EPlanClass::Level ? static_cast<int32>(EPlanStage::Level)
												: static_cast<int32>(S.Class == EPlanClass::Tilt ? EPlanStage::Tilt : EPlanStage::Stops);
		};
		if (Rank(A) != Rank(B))
		{
			return Rank(A) > Rank(B);
		}
		if (!A.IsValid())
		{
			return false;
		}
		if (FMath::Abs(A.PlannedMaxTiltDeg - B.PlannedMaxTiltDeg) > 0.05)
		{
			return A.PlannedMaxTiltDeg < B.PlannedMaxTiltDeg;
		}
		if (A.PlannedStops != B.PlannedStops)
		{
			return A.PlannedStops < B.PlannedStops;
		}
		return A.VFloor > B.VFloor + 0.05;
	}

	FCurbSchedule PlanCrossing(const FPlannerInput& In)
	{
		// The plan's samples laid on the terrain (AnchorPlanOrigin): where the
		// robot stands no longer decides where they fall against the edge. A
		// plan sampled every centimetre still turns on where its samples fall,
		// so it is made twice, side by side -- the edge on a sample, and half a
		// sample past one -- and the better kept (IsBetterPlan): the same two
		// grids from every start, and a crossing planned on either of them.
		const int32 Anchor = CVarCurbAnchorGrid.GetValueOnAnyThread();
		if (Anchor >= 2)
		{
			FPlannerInput Inputs[2] = { In, In };
			const bool	  bAnchored[2] = { AnchorPlanOrigin(Inputs[0], 1.0, 0.0), AnchorPlanOrigin(Inputs[1], 1.0, 0.5) };
			if (bAnchored[0] || bAnchored[1] || In.Terrain.bValid)
			{
				// Each grid stops once the other's finished plan is sure to be kept
				// over anything it can still make (FPlanner::Stopped), and on the
				// move once its work is over its budget against that plan: the
				// robot does not wait seconds for the slower grid. Which grid stops
				// when depends on the threads; the plan kept does not -- a plan
				// over its budget is not kept even where it finished.
				FCurbSchedule Plans[2];
				FGridRace	  Race;
				Race.bBudget = GridBudgetShare() > 0.0 && (In.bOnTheMove || In.EntrySpeedCmS > StopSpeedCmS);
				const double Started = FPlatformTime::Seconds();
				ParallelFor(2, [&](int32 i) {
					FPlanner P(Inputs[i], &Race, i);
					Plans[i] = P.Run();
					Race.Publish(i, Plans[i]);
				});
				const double Wall = 1000.0 * (FPlatformTime::Seconds() - Started);
				bool		 bKeepable[2];
				for (int32 i = 0; i < 2; ++i)
				{
					const FCurbSchedule& Other = Plans[1 - i];
					bKeepable[i] = Plans[i].IsValid() && !(Race.bBudget && Other.IsValid() && OverGridBudget(Plans[i].PlanWork, Other.PlanWork));
				}
				const int32			 Keep = bKeepable[0] != bKeepable[1] ? (bKeepable[1] ? 1 : 0) : (IsBetterPlan(Plans[1], Plans[0]) ? 1 : 0);
				FCurbSchedule		 Out = MoveTemp(Plans[Keep]);
				const FCurbSchedule& Other = Plans[1 - Keep];
				FString				 OtherWhat;
				if (Other.Summary.StartsWith(GridStoppedPrefix))
				{
					OtherWhat = Other.Summary;
				}
				else if (!Other.IsValid())
				{
					OtherWhat = TEXT("refused");
				}
				else
				{
					OtherWhat = FString::Printf(TEXT("%s, %.2f deg, floor %.1f cm/s%s"), PlanClassName(Other.Class), Other.PlannedMaxTiltDeg, Other.VFloor,
						bKeepable[1 - Keep] ? TEXT("") : TEXT(", over its budget on the move: not kept"));
				}
				Out.Lines.Add(FString::Printf(TEXT("sampled with the edge %s a sample (the other grid: %s; work %lld / %lld; both planned in %.0f ms)"),
					Keep == 0 ? TEXT("on") : TEXT("half way past"), *OtherWhat, static_cast<long long>(Out.PlanWork), static_cast<long long>(Other.PlanWork),
					Wall));
				if (Out.IsValid())
				{
					Out.Summary += FString::Printf(TEXT("; both grids in %.0f ms"), Wall);
				}
				return Out;
			}
		}
		else if (Anchor == 1)
		{
			FPlannerInput Anchored = In;
			if (AnchorPlanOrigin(Anchored, 1.0, CVarCurbAnchorPhase.GetValueOnAnyThread()))
			{
				FPlanner P(Anchored);
				return P.Run();
			}
		}
		FPlanner P(In);
		return P.Run();
	}
} // namespace RammsCurb
