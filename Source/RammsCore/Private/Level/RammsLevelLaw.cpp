// Copyright Epic Games, Inc. All Rights Reserved.

#include "Level/RammsLevelLaw.h"

namespace RammsLevel
{
	// Named, not anonymous: in a unity build the other Level files share this
	// translation unit, and their helpers may well be called the same.
	namespace LawImpl
	{
		using FLegs = TArray<FLegReach, TInlineAllocator<8>>;

		bool IsFinite(const FVector2D& Deg)
		{
			return FMath::IsFinite(Deg.X) && FMath::IsFinite(Deg.Y);
		}

		/** Nothing is applied any more: the correction, the integral, the ride
		 *  shift and the offsets are zero, and the last allocation's figures
		 *  read as "no offsets". */
		void ZeroCorrection(FState& State, int32 NumLegs)
		{
			State.CorrectionDeg = FVector2D::ZeroVector;
			State.Loop.Reset();
			State.RideCm = 0.0;
			State.HeightRateCmS = 0.0;
			State.LastDz.Init(0.0, NumLegs);
			State.HeldCm.Init(0.0, NumLegs);
			State.KeptCm.Reset();
			State.HeldBaseZ.Reset();
			State.Fraction = 1.0;
			State.ReachLeft = 1.0;
			State.MinMarginCm = NoTravelMarginCm;
			State.LimitingLeg = INDEX_NONE;
			State.LimitingPoint = INDEX_NONE;
			State.Limit = EReachLimit::None;
			State.NearestLimit = EReachLimit::None;
			State.bShort = false;
			State.GoalLimit = EReachLimit::None;
			State.bSaturated = false;
			State.bAxisLimited = false;
			State.bRateLimited = false;
			State.bLegsBehind = false;
			State.bLoopOpen = false;
		}

		/** The base's wanted attitude W: the rider's (FState::Tilt) less the
		 *  part of its pitch that the seat holds. */
		FVector2D BaseWanted(const FState& State)
		{
			return State.Tilt.Get() - FVector2D(State.SeatShare.Get().X, 0.0);
		}

		/** The seat's trim has gone into the seat's own command, or is gone:
		 *  the seat holds none of the rider's wanted pitch now. */
		void ZeroSeat(FState& State)
		{
			State.SeatTrim.Zero();
			State.SeatShare.Reset(FVector2D::ZeroVector);
		}

		/** The seat's trim is what the seat carries as an offset now
		 *  (FInputs::SeatTrimOnDeg; none where that is not a number): what the
		 *  law goes on from when it starts with no trim of its own. The share
		 *  is set from it by whoever starts the wanted attitudes next
		 *  (StartWantedFrom). */
		void TakeUpSeat(FState& State, const FInputs& In)
		{
			State.SeatTrim.Set(FMath::IsFinite(In.SeatTrimOnDeg) ? In.SeatTrimOnDeg : 0.0);
			State.SeatShare.Reset(FVector2D::ZeroVector);
		}

		/**
		 * The wanted attitudes start again from where things stand: the BASE's
		 * at Deg, at rest, and the rider's at that with the trim the seat
		 * already carries -- which is where the rider is -- so that nothing is
		 * asked to move, the seat included. What the seat carries is all share
		 * from here (it is walked to where it belongs on the tilt policy).
		 */
		void StartWantedFrom(FState& State, const FVector2D& Deg)
		{
			const FVector2D Share(State.SeatTrim.Get(), 0.0);
			State.SeatShare.Reset(Share);
			State.Tilt.Reset(Deg + Share);
		}

		/** A leg, or a point, every number of which is one. */
		bool IsFinite(const FLegReach& Leg)
		{
			return IsFinite(Leg.XY) && FMath::IsFinite(Leg.BaseZ) && FMath::IsFinite(Leg.MeasuredZ) && FMath::IsFinite(Leg.MinZ) && FMath::IsFinite(Leg.MaxZ)
				&& FMath::IsFinite(Leg.MaxRateCmS) && FMath::IsFinite(Leg.RadiusCm) && FMath::IsFinite(Leg.BaselineClearanceCm) && FMath::IsFinite(Leg.CarryShare)
				&& FMath::IsFinite(Leg.StanceGroundZ);
		}

		bool IsFinite(const FClearancePoint& Point)
		{
			return IsFinite(Point.XY) && FMath::IsFinite(Point.UndersideZ) && FMath::IsFinite(Point.MinCm);
		}

		/**
		 * Go on from the correction as it stands. The filters start from this
		 * step's readings (what they held is from before whatever interrupted
		 * the levelling), the wanted attitude starts at the attitude as found,
		 * at rest, and the integral is set so the loop's next answer is the
		 * correction already applied: nothing steps.
		 */
		void ColdResume(FState& State, const FInputs& In, const FVector2D& AchievedDeg, bool bValid, const FTuning& Tuning)
		{
			if (bValid)
			{
				State.Attitude.Seed(In.AttitudeDeg);
				State.Ground.Seed(In.AttitudeDeg + AchievedDeg);
				State.Terrain.Seed(In.AttitudeDeg + AchievedDeg);
				State.SeatGround.Seed(In.AttitudeDeg + AchievedDeg);
			}
			if (State.Attitude.IsSeeded())
			{
				StartWantedFrom(State, State.Attitude.Get());
				State.Loop.StartFrom(State.CorrectionDeg, BaseWanted(State), State.Terrain.Get(), Tuning);
			}
			else
			{
				// No attitude has ever been read (a plane adopted blind): there
				// is nothing to start the loop from. The resume that follows
				// the first reading does it.
				StartWantedFrom(State, Tuning.LevelReferenceDeg);
				State.Loop.Reset();
			}
			State.Quiet.Reset();
			State.Divergence.Reset();
			State.Support.Reset();
			State.bSaturated = false;
			State.bAxisLimited = false;
			State.bRateLimited = false;
			State.bLegsBehind = false;
			State.bShort = false;
			State.GoalLimit = EReachLimit::None;
			State.bReleaseOpen = false;
			State.bLoopOpen = false;
			State.PrevGroundDeg = State.Terrain.Get();
			// What may be carried starts again from what is carried.
			State.Carry.Reset();
			// (The estimates were taken afresh: what the stance leaned by before is not gone on from.)
			State.bStanceLean = false;
		}

		/** The legs with who stands, each lifted leg's clearance and how far
		 *  each carries: found from the baselines, going on from the set in
		 *  Was, and from how far each joint is behind the offset in Dz; or as
		 *  the plant gave them. */
		void MaskLegs(const FInputs& In, const FTuning& Tuning, TConstArrayView<bool> Was, TConstArrayView<double> Dz, FLegs& OutLegs)
		{
			OutLegs.Reset();
			OutLegs.Append(In.Legs.GetData(), In.Legs.Num());
			if (!In.bSupportGiven)
			{
				SupportMask(OutLegs, Was, Tuning);
				DiscountLateLegs(OutLegs, Dz, Tuning);
			}
			else
			{
				for (FLegReach& Leg : OutLegs)
				{
					Leg.CarryShare = Leg.bSupport ? 1.0 : 0.0;
				}
			}
		}

		/** The legs as the allocation needs them: who stands, and each lifted
		 *  leg's clearance. Returns how many stand. */
		int32 PrepareLegs(const FInputs& In, const FTuning& Tuning, FState& State, FLegs& OutLegs)
		{
			MaskLegs(In, Tuning, State.Support, State.LastDz, OutLegs);
			// Kept either way, so the hysteresis has something to go on from
			// when the plant stops giving the set.
			State.Support.Reset();
			int32 Standing = 0;
			for (const FLegReach& Leg : OutLegs)
			{
				State.Support.Add(Leg.bSupport);
				Standing += Leg.bSupport ? 1 : 0;
			}
			return Standing;
		}

		/**
		 * The last word on the legs' rate. The slew keeps the PLANE within the
		 * legs' rate, but two things move every offset without passing through
		 * it: the height term (the mean over the legs that stand, which jumps
		 * when a leg joins or leaves them) and the allocation's cut at reach
		 * (which can fall in one step when a baseline or the support set
		 * moves). So the offsets go from the ones last given towards the
		 * allocation's as a whole, in two parts.
		 *
		 * THE COMMON HEIGHT first: the mean, over the legs that stand (over
		 * every leg when none does), of what the step asks. The allocation
		 * keeps that mean at zero while the same legs stand, so it is the
		 * height term catching up and nothing else, and it is a change of
		 * ride height: it goes at the ride height's own rate
		 * (RideHeightRateCmPerSec), picked up and lost again over
		 * RideHeightRampSeconds, every leg alike, and the rest of it waits
		 * for the next steps. Legs quick enough to make the whole of it inside
		 * one frame would otherwise all be told it at once, and legs do not
		 * all answer alike: the base heaves and pitches. InOutHeightRate is
		 * how fast it is going (cm/s), kept from step to step. (No such rate,
		 * 0 or less: the legs' rate below is the only limit on it.)
		 *
		 * THE LEGS' RATE then: what is left goes by the share that keeps every
		 * leg that counts within LegRateFraction of its rate. A blend of two
		 * rigid planes is a rigid plane, and so is one moved by a common
		 * height. The legs that count are the ones the robot stands on, and a
		 * lifted leg being brought UP (it is carried with the plane to keep
		 * its clearance, and late in coming up it loses some). A lifted leg
		 * being let DOWN does not count: where it is changes nothing about
		 * the base, and held to its pace the legs that carry the robot would
		 * unwind a correction at a fraction of what they can do. It is given
		 * its own rate instead, on its own, and catches the plane up after
		 * (bOutLiftedBehind: an offset is then short of the allocation's,
		 * which a release must wait for, and no reason to hold the integral).
		 *
		 * Returns whether either held the PLANE back; the leg-rate use the
		 * step asked of the legs that count (before any cut) is taken into
		 * InOutRateUse.
		 */
		bool LimitLegSteps(const FLegs& Legs, const FLegValues& From, FLegValues& InOutDz, double Dt, const FTuning& Tuning, double& InOutRateUse,
			double& InOutHeightRate, bool& bOutLiftedBehind)
		{
			bOutLiftedBehind = false;
			if (Dt <= 0.0 || From.Num() != InOutDz.Num() || Legs.Num() != InOutDz.Num() || Legs.Num() == 0)
			{
				return false;
			}
			// A lifted leg let down is not the plane's business.
			const auto Counts = [&Legs](int32 Index, double Move) { return Legs[Index].bSupport || Move > 0.0; };
			double	   Use = 0.0;
			double	   SupportSum = 0.0;
			double	   AllSum = 0.0;
			int32	   SupportCount = 0;
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				const double Move = InOutDz[Index] - From[Index];
				if (Legs[Index].MaxRateCmS > 0.0 && Counts(Index, Move))
				{
					Use = FMath::Max(Use, FMath::Abs(Move) / (Dt * Legs[Index].MaxRateCmS));
				}
				AllSum += Move;
				if (Legs[Index].bSupport)
				{
					SupportSum += Move;
					++SupportCount;
				}
			}
			InOutRateUse = FMath::Max(InOutRateUse, Use);

			// The common height, at the ride height's rate: as fast as can
			// still stop on what is asked, the speed changing no faster than
			// brings it from rest to the rate in RideHeightRampSeconds.
			const double Common = SupportCount > 0 ? SupportSum / SupportCount : AllSum / Legs.Num();
			const double MaxRate = Tuning.RideHeightRateCmPerSec;
			const double MaxChange = MaxRate > 0.0 ? MaxRate * Dt / RideHeightRampSeconds : 0.0;
			double		 Held = 0.0;
			if (MaxRate <= 0.0 || (FMath::Abs(Common) <= RideStepSlackCm && FMath::Abs(InOutHeightRate) <= MaxChange))
			{
				// Nothing to catch up (the mean of a plane about itself, to the
				// last bit), or no rate to do it at: all of it, and at rest.
				InOutHeightRate = 0.0;
			}
			else
			{
				const double Accel = MaxRate / RideHeightRampSeconds;
				const double CanStop = FMath::Sqrt(FMath::Square(Accel * Dt) + 2.0 * Accel * FMath::Abs(Common)) - Accel * Dt;
				const double Wanted = FMath::Sign(Common) * FMath::Min(MaxRate, CanStop);
				InOutHeightRate += FMath::Clamp(Wanted - InOutHeightRate, -MaxChange, MaxChange);
				double Given = InOutHeightRate * Dt;
				if (Given * Common > 0.0 && FMath::Abs(Given) >= FMath::Abs(Common))
				{
					Given = Common;
				}
				Held = Common - Given;
			}

			// The legs' rate, on what is left.
			double Share = 1.0;
			if (Tuning.LegRateFraction > 0.0)
			{
				double LeftUse = 0.0;
				for (int32 Index = 0; Index < Legs.Num(); ++Index)
				{
					const double Move = InOutDz[Index] - Held - From[Index];
					if (Legs[Index].MaxRateCmS > 0.0 && Counts(Index, Move))
					{
						LeftUse = FMath::Max(LeftUse, FMath::Abs(Move) / (Dt * Legs[Index].MaxRateCmS));
					}
				}
				// (The hair: a plane the slew has just cut to the fraction
				// exactly is not held back again over the last bit of the
				// arithmetic.)
				if (LeftUse > Tuning.LegRateFraction * (1.0 + 1.0e-9))
				{
					Share = Tuning.LegRateFraction / LeftUse;
				}
			}
			const bool bPlaneHeld = Held != 0.0 || Share != 1.0;
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				const FLegReach& Leg = Legs[Index];
				double			 Move = (InOutDz[Index] - Held - From[Index]) * Share;
				// A lifted leg let down goes at its own rate, whatever the
				// plane's: the rest of its way waits for the next steps.
				bool bOwnPace = false;
				if (!Counts(Index, Move) && Leg.MaxRateCmS > 0.0 && Tuning.LegRateFraction > 0.0)
				{
					const double Most = Tuning.LegRateFraction * Leg.MaxRateCmS * Dt;
					if (-Move > Most * (1.0 + 1.0e-9))
					{
						Move = -Most;
						bOwnPace = true;
						bOutLiftedBehind = true;
					}
				}
				if (!bPlaneHeld && !bOwnPace)
				{
					continue; // the allocation's own, to the last bit
				}
				// Allocate's last resort, kept: never past the travel.
				InOutDz[Index] = FMath::Clamp(From[Index] + Move, FMath::Min(0.0, Leg.MinZ - Leg.BaseZ), FMath::Max(0.0, Leg.MaxZ - Leg.BaseZ));
			}
			return bPlaneHeld;
		}

		/** Keep an allocation's figures for the steps that reuse them. */
		void KeepAllocation(FState& State, const FAllocation& Alloc)
		{
			State.LastDz = Alloc.Dz;
			State.Fraction = Alloc.Fraction;
			State.ReachLeft = Alloc.ReachLeft;
			State.MinMarginCm = Alloc.MinMarginCm;
			State.LimitingLeg = Alloc.LimitingLeg;
			State.LimitingPoint = Alloc.LimitingPoint;
			State.Limit = Alloc.Limit;
			State.NearestLimit = Alloc.NearestLimit;
		}

		/** The plane the legs make of their baselines when each stands Cm[i]
		 *  (cm) off its baseline: MeasuredCorrection's, with its weights. */
		FVector2D PlaneOfHeights(const FLegs& Legs, TConstArrayView<double> Cm)
		{
			FLegs At = Legs;
			for (int32 Index = 0; Index < At.Num(); ++Index)
			{
				At[Index].MeasuredZ = At[Index].BaseZ + (Cm.IsValidIndex(Index) ? Cm[Index] : 0.0);
			}
			return MeasuredCorrection(At);
		}

		/**
		 * Whether a leg's baseline has moved from where it was last found
		 * still (BaselineStillCm); when one has, every leg's is taken as found.
		 * A stance being ramped, a height being jogged: something is moving
		 * the base under the offsets, and a base at rest would be carried off
		 * level with nothing written.
		 */
		bool BaselinesMoved(const FInputs& In, FState& State)
		{
			bool	   bMoved = State.StillBaseZ.Num() != In.Legs.Num();
			const bool bFirst = bMoved;
			// (A leg nothing commands has no baseline to move: where its joint
			// is says nothing, and it keeps the place it was last commanded at,
			// so that being taken back there is no move either.)
			for (int32 Index = 0; Index < In.Legs.Num() && !bMoved; ++Index)
			{
				bMoved = In.Legs[Index].bHasBaseline && FMath::Abs(In.Legs[Index].BaseZ - State.StillBaseZ[Index]) > BaselineStillCm;
			}
			if (bMoved)
			{
				for (int32 Index = 0; Index < In.Legs.Num(); ++Index)
				{
					if (bFirst)
					{
						State.StillBaseZ.Add(In.Legs[Index].BaseZ);
					}
					else if (In.Legs[Index].bHasBaseline)
					{
						State.StillBaseZ[Index] = In.Legs[Index].BaseZ;
					}
				}
			}
			// (The first sight of the legs is not a move.)
			return bMoved && !bFirst;
		}

		/**
		 * The law's step 0a: carried legs held clear of a stance ground that
		 * rises under them. Moves State.HeldCm on (and KeptCm, HeldBaseZ) and
		 * gives the legs as held in OutLegs. False, OutLegs untouched, when no
		 * leg is held: the legs are then as given.
		 */
		bool HoldCarried(const FInputs& Given, double Dt, const FTuning& Tuning, FState& State, FLegs& OutLegs)
		{
			const int32 Num = Given.Legs.Num();
			if (State.HeldCm.Num() != Num)
			{
				State.HeldCm.Init(0.0, Num);
				State.KeptCm.Reset();
				State.HeldBaseZ.Reset();
			}
			const auto AsHeld = [&Given, &State, Num](FLegs& Legs) {
				bool bAny = false;
				for (int32 Index = 0; Index < Num; ++Index)
				{
					bAny = bAny || State.HeldCm[Index] != 0.0;
				}
				if (!bAny)
				{
					return false;
				}
				Legs.Reset();
				Legs.Append(Given.Legs.GetData(), Num);
				for (int32 Index = 0; Index < Num; ++Index)
				{
					Legs[Index].BaseZ += State.HeldCm[Index];
				}
				// The ground the stance rests on with the held legs where they are held.
				StanceGround(Legs, Given.StancePairHalfTrackCm, Given.StanceCentreXY);
				return true;
			};
			if (!Given.bStanceCentre || Given.bSupportGiven || Num == 0 || !State.Arbiter.HasOffsets())
			{
				State.HeldCm.Init(0.0, Num);
				State.KeptCm.Reset();
				State.HeldBaseZ.Reset();
				return false;
			}
			const EState Was = State.Arbiter.GetState();
			const bool	 bLevelling = Was == EState::Active && Given.bEnable && Given.Mode == EMode::Allowed && !Given.bYield && !Given.bResume;
			// A leg held further or less far is told nothing by that alone: what
			// it carries over its own baseline stays, and the offset it had over
			// the baseline as held changes by as much the other way. (The legs'
			// rate is then counted on what a leg is really asked to move.)
			const auto HoldAt = [&State](int32 Index, double HeldCm) {
				if (State.LastDz.IsValidIndex(Index))
				{
					State.LastDz[Index] += State.HeldCm[Index] - HeldCm;
				}
				State.HeldCm[Index] = HeldCm;
			};
			if (Dt > 0.0 && bLevelling && !Given.bStanceChanging)
			{
				// Who stands, the legs as held so far: as the last step left it.
				FLegs Held;
				if (!AsHeld(Held))
				{
					Held.Append(Given.Legs.GetData(), Num);
				}
				SupportMask(Held, State.Support, Tuning);
				const auto Over = [](const FLegReach& Leg) { return Leg.BaseZ - Leg.RadiusCm - Leg.StanceGroundZ; };
				double	   Lowest = Over(Held[0]);
				for (const FLegReach& Leg : Held)
				{
					Lowest = FMath::Min(Lowest, Over(Leg));
				}
				const bool bAfresh = State.KeptCm.Num() != Num || State.HeldBaseZ.Num() != Num;
				if (bAfresh)
				{
					State.KeptCm.Init(0.0, Num);
					State.HeldBaseZ.Init(0.0, Num);
				}
				for (int32 Index = 0; Index < Num; ++Index)
				{
					const FLegReach& Leg = Given.Legs[Index];
					// Its clearance as held, and as its baseline alone would have it.
					const double ClearCm = Over(Held[Index]) - Lowest;
					const double AloneCm = ClearCm - State.HeldCm[Index];
					if (bAfresh || Held[Index].bSupport || !Leg.bHasBaseline)
					{
						State.KeptCm[Index] = ClearCm;
					}
					else
					{
						// (Its own baseline's moves are its own: lowered by its
						// command, it comes down.)
						State.KeptCm[Index] += Leg.BaseZ - State.HeldBaseZ[Index];
						const double ShortCm = State.KeptCm[Index] - AloneCm;
						// No further than its travel lets it be held, with the offset it last had.
						const double LastCm = State.LastDz.IsValidIndex(Index) ? State.LastDz[Index] : 0.0;
						HoldAt(Index, FMath::Min(ShortCm > HeldSlackCm ? ShortCm - HeldSlackCm : 0.0, FMath::Max(0.0, Leg.MaxZ - Leg.BaseZ - LastCm)));
					}
					State.HeldBaseZ[Index] = Leg.BaseZ;
				}
			}
			else if (Dt > 0.0 && !bLevelling && Was != EState::Holding)
			{
				// Given back: no leg is held, and each goes from where it was
				// held to where its baseline and the plane have it as any
				// lifted leg is let down (LimitLegSteps: at its own rate).
				for (int32 Index = 0; Index < Num; ++Index)
				{
					HoldAt(Index, 0.0);
				}
				State.KeptCm.Reset();
				State.HeldBaseZ.Reset();
			}
			else if (Dt > 0.0)
			{
				// A stance being changed, no attitude: held as it is, the
				// clearances taken afresh after.
				State.KeptCm.Reset();
				State.HeldBaseZ.Reset();
			}
			return AsHeld(OutLegs);
		}
	} // namespace LawImpl

	FOutputs FLaw::Step(const FInputs& Given, double InDt, const FTuning& Tuning, FState& State)
	{
		// ---- 0. Housekeeping ----
		const double Dt = FMath::IsFinite(InDt) ? FMath::Clamp(InDt, 0.0, MaxStepSeconds) : 0.0;
		// ---- 0a. Carried legs held clear: from here on the legs are the legs as held ----
		FInputs		   In = Given;
		LawImpl::FLegs HeldLegs;
		{
			bool bNumbers = true;
			for (const FLegReach& Leg : Given.Legs)
			{
				bNumbers = bNumbers && LawImpl::IsFinite(Leg);
			}
			if (bNumbers && State.LegCount == Given.Legs.Num() && !Given.bColdReset && LawImpl::HoldCarried(Given, Dt, Tuning, State, HeldLegs))
			{
				In.Legs = HeldLegs;
			}
		}
		const int32		NumLegs = In.Legs.Num();
		const FVector2D Ref = Tuning.LevelReferenceDeg;

		// Other legs than last step's: nothing kept per leg means anything now.
		const bool bColdReset = In.bColdReset || State.LegCount != NumLegs;
		State.LegCount = NumLegs;
		if (State.LastDz.Num() != NumLegs)
		{
			State.LastDz.Init(0.0, NumLegs);
		}
		if (bColdReset)
		{
			State.PrevBaseZ.Reset();
			State.StillBaseZ.Reset();
			State.SagCm.Reset();
			State.HeldCm.Init(0.0, NumLegs);
			State.KeptCm.Reset();
			State.HeldBaseZ.Reset();
		}
		double HeldMostCm = 0.0;
		for (const double HeldCm : State.HeldCm)
		{
			HeldMostCm = FMath::Max(HeldMostCm, FMath::Abs(HeldCm));
		}

		// Every number the legs and the points come with, not only the two
		// heights: one that is not a number anywhere would go through the
		// plane fit into the terrain estimate, which keeps it for good.
		bool bLegsValid = true;
		for (const FLegReach& Leg : In.Legs)
		{
			bLegsValid = bLegsValid && LawImpl::IsFinite(Leg);
		}
		for (const FClearancePoint& Point : In.Points)
		{
			bLegsValid = bLegsValid && LawImpl::IsFinite(Point);
		}
		const bool bValid = bLegsValid && In.bAttitudeValid && LawImpl::IsFinite(In.AttitudeDeg);

		// The seat: whether its trim runs, and the room it has either way about
		// where the rider has it (a seat described with numbers that are not
		// numbers is no seat).
		const bool	 bSeatRuns = In.bSeat && Tuning.bSeatCoversTransients && FMath::IsFinite(In.SeatPreferenceDeg) && FMath::IsFinite(In.SeatTrimLowDeg) && FMath::IsFinite(In.SeatTrimHighDeg);
		const double SeatMaxDeg = FMath::Max(0.0, Tuning.SeatTrimMaxDeg);
		const double SeatLowDeg = bSeatRuns ? FMath::Max(-SeatMaxDeg, FMath::Min(0.0, In.SeatTrimLowDeg)) : -SeatMaxDeg;
		const double SeatHighDeg = bSeatRuns ? FMath::Min(SeatMaxDeg, FMath::Max(0.0, In.SeatTrimHighDeg)) : SeatMaxDeg;

		// What the last step left, for "is anything moving" further down.
		const FVector2D PrevGroundDeg = State.PrevGroundDeg;
		const FVector2D PrevSeenDeg = State.Ground.Get();
		const bool		bSeenBefore = State.Ground.IsSeeded();
		// ... and for what the seat is to cover: where the ground as the seat
		// hears it, and the correction, stood.
		const bool	 bSeatGroundBefore = State.SeatGround.IsSeeded();
		const double SeatGroundBefore = State.SeatGround.Get().X;
		const double CorrectionBefore = State.CorrectionDeg.X;

		// The offsets a hand-back left on the legs, where it says.
		bool bResumeDz = In.bResume && In.bResumeHasPlane && In.ResumeDz.Num() == NumLegs && NumLegs > 0;
		for (int32 Index = 0; bResumeDz && Index < NumLegs; ++Index)
		{
			bResumeDz = FMath::IsFinite(In.ResumeDz[Index]);
		}

		// ---- 1. Sensing ----
		FVector2D AchievedDeg = FVector2D::ZeroVector;
		FVector2D SagDeg = FVector2D::ZeroVector;
		FVector2D NowSagDeg = FVector2D::ZeroVector;
		if (bValid)
		{
			// The correction the joints have really made, not the one asked
			// for: sag, rate limits and travel stops then cancel in G. Made by
			// the legs the robot stands on: who they are is found here as the
			// allocation will find it (the set kept in the state is not
			// touched: that is PrepareLegs' to move on).
			// (A plane handed back with the offsets it left: those are what
			// the legs carry now, and who stood before the take-over says
			// nothing of who stands in the pose it left.)
			LawImpl::FLegs Sensed;
			if (bResumeDz)
			{
				LawImpl::MaskLegs(In, Tuning, TConstArrayView<bool>(), In.ResumeDz, Sensed);
			}
			else
			{
				LawImpl::MaskLegs(In, Tuning, State.Support, State.LastDz, Sensed);
			}
			AchievedDeg = MeasuredCorrection(Sensed);
			// What each joint stands off its BASELINE by with no offset on it:
			// its standing shortfall under the load, read while there are no
			// offsets (so that a leg whose travel is not what its model says,
			// short of its offset by a share of it, is not taken for one that
			// sags) and kept from there. The plane the joints make of that (the
			// correction they read with nothing asked of them) is what a
			// release must not take for ground, and what the underside's
			// clearance is short by.
			{
				const TConstArrayView<double> Told = bResumeDz ? In.ResumeDz : TConstArrayView<double>(State.LastDz);
				bool						  bNoOffsets = true;
				for (const double Dz : Told)
				{
					bNoOffsets = bNoOffsets && Dz == 0.0;
				}
				const bool bSeedSag = State.SagCm.Num() != NumLegs;
				if (bSeedSag)
				{
					State.SagCm.Init(0.0, NumLegs);
				}
				if (bNoOffsets)
				{
					const double Share = SagFilterSeconds > 0.0 ? FMath::Clamp(Dt / SagFilterSeconds, 0.0, 1.0) : 1.0;
					for (int32 Index = 0; Index < NumLegs; ++Index)
					{
						const FLegReach& Leg = In.Legs[Index];
						const double	 Off = Leg.bHasBaseline ? Leg.MeasuredZ - Leg.BaseZ : 0.0;
						State.SagCm[Index] = bSeedSag ? Off : State.SagCm[Index] + (Off - State.SagCm[Index]) * Share;
					}
				}
				SagDeg = LawImpl::PlaneOfHeights(Sensed, State.SagCm);
				// ... and what they stand off the plane they are TOLD by now.
				NowSagDeg = AchievedDeg - LawImpl::PlaneOfHeights(Sensed, Told);
			}
			const FVector2D Filtered = State.Attitude.Tick(In.AttitudeDeg, Dt, Tuning.AttitudeFilterSeconds);
			// The ground as far as it can have turned: by the distance
			// travelled over the standing legs' span, and by what the legs'
			// own baselines turned the stance; and, turning on the spot on a
			// slope, by the slope swinging from pitch into roll in the
			// chassis' own frame. What the attitude did beyond that (a knock,
			// braking, a rock onto a leg set down) is the body's, and neither
			// the terrain estimate nor the loop sees it.
			const double	TurnDegPerCm = State.Ground.Tuning.BreakDeg / StandingSpanCm(Sensed);
			const double	ExplainedDeg = BaselineTurnDeg(Sensed, State.PrevBaseZ);
			const FVector2D SeenDeg = State.Ground.Tick(Filtered + AchievedDeg, In.SpeedCmS, TurnDegPerCm, ExplainedDeg, Dt, In.YawRateDegPerSec);
			// How the stance itself leans on the flat (the plane of the ground
			// it rests on under the legs that stand, where the plant gives one)
			// is KNOWN, and so is what a baseline that moved since the last step
			// turned it by, to the last bit, on the step it moved: the stance's
			// own attitude (the estimate's G) turned by exactly that, the other
			// way. Left to the estimate's low-pass, a linkage jogged under a
			// levelled robot is followed a filter's time behind for as long as
			// the jog runs, the integral makes the lag up, and has it the wrong
			// way round the moment the jog stops or turns back: the base then
			// swings by twice the lag (a simulator's finding, at the turn of a
			// jog at its real rate). So the estimates take that part whole.
			// (While levelling, and not while a drive mode changes the stance:
			// its ramp is waited out, not followed. Nothing on a robot whose
			// stances stand level in its own frame: its lean is none.)
			{
				TArray<FVector2D, TInlineAllocator<8>> XY;
				TArray<bool, TInlineAllocator<8>>	   Stands;
				FLegValues							   GroundZ;
				bool								   bMoved = false;
				for (int32 Index = 0; Index < Sensed.Num(); ++Index)
				{
					XY.Add(Sensed[Index].XY);
					Stands.Add(Sensed[Index].bSupport);
					GroundZ.Add(Sensed[Index].StanceGroundZ);
					bMoved = bMoved || (State.PrevBaseZ.IsValidIndex(Index) && Sensed[Index].BaseZ != State.PrevBaseZ[Index]);
				}
				FVector2D  LeanDeg = FVector2D::ZeroVector;
				double	   HeightCm = 0.0;
				const bool bLean = Stands.Contains(true) && FitPlane(XY, GroundZ, Stands, TConstArrayView<bool>(), LeanDeg, HeightCm);
				if (Dt > 0.0 && bLean && State.bStanceLean && bMoved && !In.bStanceChanging && State.Arbiter.GetState() == EState::Active && State.Terrain.IsSeeded())
				{
					const FVector2D TurnedDeg = LeanDeg - State.StanceLeanDeg;
					if (!TurnedDeg.IsZero())
					{
						State.Terrain.Seed(State.Terrain.Get() - TurnedDeg);
						if (State.SeatGround.IsSeeded())
						{
							State.SeatGround.Seed(State.SeatGround.Get() - TurnedDeg);
						}
					}
				}
				if (Dt > 0.0)
				{
					State.StanceLeanDeg = LeanDeg;
					State.bStanceLean = bLean;
				}
			}
			State.Terrain.Tick(SeenDeg - AchievedDeg, AchievedDeg, Dt, Tuning.TerrainFilterSeconds);
			State.SeatGround.Tick(SeenDeg, Dt, Tuning.SeatFilterSeconds);
			State.LossSeconds = 0.0;
		}
		else
		{
			State.LossSeconds += Dt;
		}
		if (bLegsValid && Dt > 0.0)
		{
			// (Only on a step that takes time: a rebase carried out in the
			// middle of a frame is then part of the next step's move.)
			State.PrevBaseZ.Reset();
			for (const FLegReach& Leg : In.Legs)
			{
				State.PrevBaseZ.Add(Leg.BaseZ);
			}
		}
		// Something is moving the stance under the offsets.
		const bool bBaselinesMoving = (bLegsValid && LawImpl::BaselinesMoved(In, State)) || In.bStanceChanging;

		// ---- 2. The arbiter, and what it asks for ----
		FArbiterInputs Ask;
		Ask.bEnable = In.bEnable;
		Ask.Mode = In.Mode;
		Ask.bAttitudeValid = bValid;
		Ask.bYield = In.bYield;
		Ask.bResume = In.bResume;
		Ask.bResumeHasPlane = In.bResumeHasPlane && LawImpl::IsFinite(In.ResumePlaneDeg);
		Ask.bColdReset = bColdReset;
		Ask.bCorrectionZero = State.CorrectionDeg.Size() < ReleaseDoneDeg && FMath::Abs(State.RideCm) < ReleaseDoneCm
			&& FMath::Abs(State.SeatTrim.Get()) < ReleaseDoneDeg && !State.bLegsBehind && HeldMostCm < ReleaseDoneCm;
		Ask.bDiverged = State.Divergence.HasDiverged();
		Ask.bStanceChanging = In.bStanceChanging;
		const FArbiterStep Arbiter = State.Arbiter.Tick(Ask);

		// The wanted attitude was set this step: it starts moving on the next,
		// so that this step's correction is exactly the one already applied.
		bool bWantedSet = false;
		bool bAdopted = false;
		bool bCleared = false;
		switch (Arbiter.Action)
		{
			case EAction::StartFromAttitude:
				LawImpl::ZeroCorrection(State, NumLegs);
				// What the seat carries as an offset already (a trim taken back
				// out of its command at a hand-back, one a take-over nobody
				// announced left on it) is gone on from: zeroed here it would be
				// stepped off the seat, and ignored it would be added to.
				LawImpl::TakeUpSeat(State, In);
				LawImpl::ColdResume(State, In, AchievedDeg, bValid, Tuning);
				bWantedSet = true;
				break;
			case EAction::ColdResume:
				LawImpl::ColdResume(State, In, AchievedDeg, bValid, Tuning);
				bWantedSet = true;
				break;
			case EAction::Adopt:
				// A fold came before (on this very step, even), and what was
				// folded stays in the baselines but for this plane.
				State.CorrectionDeg = In.ResumePlaneDeg;
				State.RideCm = 0.0;
				State.HeightRateCmS = 0.0;
				// (The seat: none where the fold's trim stayed in the seat's own
				// command for a taker that commanded the seat; the trim itself
				// where the shell has taken it back out of a command nobody
				// wrote.)
				LawImpl::TakeUpSeat(State, In);
				LawImpl::ColdResume(State, In, AchievedDeg, bValid, Tuning);
				bWantedSet = true;
				bAdopted = true;
				break;
			case EAction::Fold:
				LawImpl::ZeroCorrection(State, NumLegs);
				LawImpl::ZeroSeat(State);
				// Whoever has taken the legs runs its own pace, and the drive is
				// let go at once: the scale goes on from full, as the drive
				// does, not from where it was (a hand-back a moment later would
				// otherwise ask a drive at full speed for a part of it in one go).
				State.SpeedScale.Reset();
				break;
			case EAction::Clear:
				LawImpl::ZeroCorrection(State, NumLegs);
				LawImpl::ZeroSeat(State);
				bCleared = true;
				break;
			case EAction::BeginRelease:
				State.ReleaseSeconds = 0.0;
				State.bReleaseOpen = false;
				State.bReleaseSag = false;
				State.Quiet.Reset();
				break;
			case EAction::Freeze:
			case EAction::None:
			default:
				break;
		}

		// ---- 3. The state's work ----
		const EState Now = Arbiter.State;
		const bool	 bActive = Now == EState::Active;
		const bool	 bRamping = Now == EState::Faulted && !State.Arbiter.IsFaultLatched();
		// Closed loop: levelling, or giving the base back by walking the wanted
		// attitude to the ground's. Open loop: the correction ramped straight
		// out (a release's last part, a release with no attitude, a fault).
		const bool bClosed = bActive || (Now == EState::Releasing && bValid && !State.bReleaseOpen);
		const bool bOpen = (Now == EState::Releasing && !bClosed) || bRamping;

		const FVector2D AttitudeDeg = State.Attitude.Get();
		const FVector2D GroundDeg = State.Terrain.Get();
		// What the loop steers by: the attitude less what only the body can
		// have done to it. (The attitude itself whenever the gate lets the
		// reading through.)
		const FVector2D LoopAttitudeDeg = bValid && State.Ground.IsSeeded() ? State.Ground.Get() - AchievedDeg : AttitudeDeg;

		LawImpl::FLegs Legs;
		int32		   Standing = 0;
		if (bClosed || bOpen || bAdopted)
		{
			Standing = LawImpl::PrepareLegs(In, Tuning, State, Legs);
		}
		// THE LIMITS AS THE LAW COUNTS THEM, and the underside as it stands.
		//  - The most correction per axis is counted from the STANCE: a robot
		//    whose stance leans on the flat (FLegReach::StanceGroundZ) spends
		//    that lean of its correction before any ground has turned, so each
		//    axis' limit is the tuning's plus the lean's size about it. (Either
		//    way: against the lean the limit is then twice the lean past the
		//    tuning's, where the legs' travel and the underside stop it first.)
		//  - The underside's clearance is worked out from the baselines; the
		//    joints stand off those under the load, and the ground lies under
		//    the points where the JOINTS put it: each point's underside is
		//    taken lower by the plane through what the standing legs' joints
		//    stand off their commands by.
		FTuning Full = Tuning;
		Full.MaxCorrectionDeg = FVector2D(FMath::Max(0.0, Tuning.MaxCorrectionDeg.X), FMath::Max(0.0, Tuning.MaxCorrectionDeg.Y));
		TArray<FClearancePoint, TInlineAllocator<8>> Points;
		Points.Append(In.Points.GetData(), In.Points.Num());
		if (bLegsValid && Legs.Num() > 0)
		{
			TArray<FVector2D, TInlineAllocator<8>> XY;
			TArray<bool, TInlineAllocator<8>>	   Use;
			TArray<bool, TInlineAllocator<8>>	   Roll;
			FLegValues							   GroundZ;
			bool								   bAnyStands = false;
			for (const FLegReach& Leg : Legs)
			{
				XY.Add(Leg.XY);
				Use.Add(Leg.bSupport);
				Roll.Add(Leg.bRoll);
				GroundZ.Add(Leg.StanceGroundZ);
				bAnyStands = bAnyStands || Leg.bSupport;
			}
			FVector2D PlaneDeg = FVector2D::ZeroVector;
			double	  HeightCm = 0.0;
			// (Not while a stance is being changed: baselines on their way to
			// a stance lean by what the ramp has yet to turn the base by, which
			// is the ramp's to take out and not more correction to be had.)
			if (bAnyStands && !In.bStanceChanging && FitPlane(XY, GroundZ, Use, TConstArrayView<bool>(), PlaneDeg, HeightCm))
			{
				Full.MaxCorrectionDeg += FVector2D(FMath::Abs(PlaneDeg.X), FMath::Abs(PlaneDeg.Y));
			}
			if (bAnyStands && State.SagCm.Num() == Legs.Num() && FitPlane(XY, State.SagCm, Use, Roll, PlaneDeg, HeightCm))
			{
				for (FClearancePoint& Point : Points)
				{
					Point.UndersideZ -= PlaneRise(Point.XY, true, PlaneDeg) + HeightCm;
				}
			}
		}

		if (bAdopted && bLegsValid)
		{
			// The offsets the plant has just put on the motors: the adopted
			// plane, allocated with no ride shift and no slew.
			LawImpl::KeepAllocation(State, Allocate(Legs, Points, State.CorrectionDeg, Full, 0.0, 0.0));
			if (bResumeDz)
			{
				// ... but for a leg the plant could not leave it on exactly:
				// what it did leave is where the next steps start from.
				State.LastDz.Reset();
				State.LastDz.Append(In.ResumeDz.GetData(), NumLegs);
			}
		}

		bool bQuiet = false;
		bool bWrite = bCleared;
		// The legs' rate use, as reported: the slew's, with the offsets' own
		// taken in where that is larger.
		double RateUse = 0.0;
		if (bClosed)
		{
			if (bActive)
			{
				State.bReleaseOpen = false;
			}
			if (State.bLoopOpen && Dt > 0.0)
			{
				// Back from an open-loop ramp (a release taken back, or one
				// that lost its attitude and has it again). The loop was not
				// consulted meanwhile: its integral is from before, and would
				// ask again for what was just ramped out. Go on from the
				// correction as it stands, as a resume does.
				LawImpl::StartWantedFrom(State, LoopAttitudeDeg);
				State.Loop.StartFrom(State.CorrectionDeg, LawImpl::BaseWanted(State), GroundDeg, Tuning);
				State.bLoopOpen = false;
				State.bSaturated = false;
				State.bRateLimited = false;
				bWantedSet = true;
			}
			// Where the base is wanted, and how much of the rider's wanted pitch
			// the seat is to hold on top of that. A release gives the base back
			// to its stance and the seat back to the rider.
			// (The stance as the joints will stand in it with no offsets on: what
			// they stand off their commands by under the load is not ground,
			// and a walk to the ground's attitude as the joints read it would
			// go past the stance by that and come back.)
			// (Only as far as the joints stand off what they are told NOW the
			// same way: a load that the correction has moved onto other legs
			// leaves them standing otherwise, and then nothing is known.)
			if (!bActive && !State.bReleaseSag)
			{
				const auto Agreed = [](double Then, double Now) { return Then * Now > 0.0 ? FMath::Sign(Then) * FMath::Min(FMath::Abs(Then), FMath::Abs(Now)) : 0.0; };
				State.ReleaseSagDeg = FVector2D(Agreed(SagDeg.X, NowSagDeg.X), Agreed(SagDeg.Y, NowSagDeg.Y));
				State.bReleaseSag = true;
			}
			FVector2D GoalDeg = GroundDeg - State.ReleaseSagDeg;
			double	  ShareGoalDeg = 0.0;
			// The limits the correction is held inside: the law's own, or less
			// while the robot may not carry as much at the speed it has.
			FTuning Eff = Full;
			// Where the base would be wanted with only the legs' own limits.
			FVector2D ReachGoalDeg = GoalDeg;
			// Standing still with nothing left to walk and nothing moving the
			// stance: all that is between here and a rest is the settle time.
			bool bAboutToRest = false;
			if (bActive)
			{
				// Level where the legs can hold it; and how much they must
				// leave, and why, said steadily (the gap between On and Off).
				// What may be carried at this speed (FCarryLimit): the base is left
				// tilted by the rest, as by anything else the legs cannot give,
				// but that is the speed's doing and not being at reach.
				FCarryInputs CarryIn;
				CarryIn.FullDeg = Full.MaxCorrectionDeg;
				CarryIn.NeededDeg = GroundDeg - Ref;
				CarryIn.SpeedCmS = In.SpeedCmS;
				CarryIn.AskedSpeedCmS = In.AskedSpeedCmS;
				CarryIn.SpanCm = StandingSpanCm(Legs);
				CarryIn.LegsDegPerSec = FVector2D(PlaneRateDegPerSec(Legs, FVector2D(1.0, 0.0), Tuning), PlaneRateDegPerSec(Legs, FVector2D(0.0, 1.0), Tuning));
				bool bStandsWithoutRoll = false, bRollReaches = false;
				for (const FLegReach& Leg : Legs)
				{
					bStandsWithoutRoll = bStandsWithoutRoll || (Leg.bSupport && !Leg.bRoll);
					bRollReaches = bRollReaches || (Leg.bSupport && Leg.bRoll);
				}
				CarryIn.bThreePoint = bStandsWithoutRoll && bRollReaches;
				CarryIn.bMayRise = State.Ground.GetTrust() >= 1.0;
				CarryIn.CarriedDeg = State.CorrectionDeg;
				CarryIn.GroundRollDeg = GroundDeg.Y;
				CarryIn.bGroundRollKnown = State.Terrain.IsSeeded();
				const FVector2D CarryDeg = State.Carry.Tick(CarryIn, Dt, Tuning);
				Eff.MaxCorrectionDeg = FVector2D(FMath::Min(Full.MaxCorrectionDeg.X, CarryDeg.X), FMath::Min(Full.MaxCorrectionDeg.Y, CarryDeg.Y));
				State.CarryDeg = Eff.MaxCorrectionDeg;
				const bool bLower = Eff.MaxCorrectionDeg.X < Full.MaxCorrectionDeg.X - 1.0e-9 || Eff.MaxCorrectionDeg.Y < Full.MaxCorrectionDeg.Y - 1.0e-9;

				// (At reach is said of the legs' own limits, not of the speed's.)
				const FTiltGoal Reach = WantedTiltGoalWithLimit(GroundDeg, Legs, Points, Full);
				GoalDeg = Reach.GoalDeg;
				// Held back for the speed: the base is wanted somewhere else
				// than the legs' own limits would have it, by what the speed's
				// limits leave over theirs. That is added whole to where the
				// legs' reach has the base wanted. The tilt policy's bound on
				// how far the base may be left off the reference is a bound on
				// what the LEGS leave (past it the robot is past reach and
				// crawls); a tilt the robot has decided not to carry at its
				// speed is where the base will be, and a wanted attitude held
				// short of it would be an error no leg may close, which the
				// speed cap would slow for and a crawl would not cure.
				FVector2D EasedDeg = FVector2D::ZeroVector;
				if (bLower)
				{
					FTuning OpenEff = Eff;
					FTuning OpenFull = Full;
					OpenEff.MaxTiltDeg = OpenFull.MaxTiltDeg = UnboundedTiltDeg;
					EasedDeg = WantedTiltGoalWithLimit(GroundDeg, Legs, Points, OpenEff).GoalDeg - WantedTiltGoalWithLimit(GroundDeg, Legs, Points, OpenFull).GoalDeg;
				}
				State.bEased = bLower && EasedDeg.Size() > AxisLimitSlackDeg;
				State.EasedDeg = State.bEased ? EasedDeg : FVector2D::ZeroVector;
				GoalDeg += State.EasedDeg;
				State.bShort = Reach.ShortDeg > (State.bShort ? AtReachOffDeg : AtReachOnDeg);
				State.GoalLimit = Reach.Limit;
				ReachGoalDeg = Reach.GoalDeg;
				if (bSeatRuns)
				{
					// The pitch the rider prefers; and what the legs must leave
					// the base tilted by, taken back out at the seat. (What
					// the LEGS' reach leaves: a tilt left in the base because
					// the robot may not carry more correction at the speed it
					// has is not put in the seat, where it would be carried
					// over a crest just the same.)
					ShareGoalDeg = FMath::Clamp(In.SeatPreferenceDeg - (ReachGoalDeg.X - Ref.X), SeatLowDeg, SeatHighDeg);
				}
			}
			const FVector2D ShareGoal(ShareGoalDeg, 0.0);
			const FVector2D RiderGoalDeg = GoalDeg + ShareGoal;
			if (bActive)
			{
				// No rest while something moves the stance under the offsets:
				// nothing would be written, and the base would go with it. Nor
				// while an offset is still on its way to the allocation's (a
				// lifted leg being let down at its own pace): it would be left
				// where the rest found it.
				// (Nor while the seat is still on its way.)
				const bool bMayRest = State.Tilt.IsNear(RiderGoalDeg) && State.SeatShare.IsNear(ShareGoal) && !State.SeatTrim.IsBehind() && !bBaselinesMoving && !State.bLegsBehind;
				bQuiet = State.Quiet.Tick(In.SpeedCmS, (AttitudeDeg - LawImpl::BaseWanted(State)).Size(), In.bStick, bMayRest, Dt, Tuning);
				bAboutToRest = bMayRest && !In.bStick && FMath::Abs(In.SpeedCmS) < QuietSpeedCmS;
			}
			// At rest nothing is written, so the servos simply hold; and a step
			// in which no time passes advances nothing.
			if (!bQuiet && Dt > 0.0)
			{
				if (!bWantedSet)
				{
					// One deliberate change of the rider's attitude at a time:
					// while a drive mode's stance is being changed under the
					// legs the base rocks onto other wheels by itself, and the
					// walk waits for that to be over (it comes to rest within
					// the policy's acceleration, and goes on from there).
					// (The rider's wanted attitude is what the policy is about;
					// the seat's share of its pitch moves on the same limits,
					// and the base's is the one less the other.)
					// (Nor does a release walk while the ground itself is turning
					// under the robot: a grade break turns the rider already.)
					if (In.bStanceChanging || (!bActive && State.SpeedScale.GetTurnDegPerSec().Size() > ReleaseWaitTurnDegPerSec))
					{
						State.Tilt.Brake(Dt, Tuning.TiltAccelDegPerSec2);
						State.SeatShare.Brake(Dt, Tuning.TiltAccelDegPerSec2);
					}
					else
					{
						State.Tilt.Tick(RiderGoalDeg, Dt, Tuning.TiltRateDegPerSec, Tuning.TiltAccelDegPerSec2);
						State.SeatShare.Tick(ShareGoal, Dt, Tuning.TiltRateDegPerSec, Tuning.TiltAccelDegPerSec2);
					}
				}
				const FVector2D WantedNow = LawImpl::BaseWanted(State);

				// The integral only trims while levelling, on a stance that can
				// be levelled, and not while the legs are already going as fast
				// as they can.
				const bool bIntegrate = bActive && Standing >= MinSupportForIntegral && !State.bRateLimited;
				// (About to rest, the integral trims what a walk or a knock left
				// in it down to a narrower band than the loop's: the rest is
				// then on the wanted attitude, not on the deadband's edge.)
				const FVector2D RawDeg = State.Loop.Tick(LoopAttitudeDeg, WantedNow, GroundDeg, Dt, bIntegrate, State.bSaturated, Eff, bAboutToRest ? RestTrimDeadbandDeg : -1.0);

				// The per-axis limit is a limit like the legs' reach: asking to
				// pass it is being at reach. (The loop holds its own integral
				// on an axis that is at its limit.) What the speed holds the
				// correction back by is not: that is said of the law's own
				// limits.
				const FVector2D AskedDeg = ClampAxes(RawDeg, Eff.MaxCorrectionDeg);
				const FVector2D WithinDeg = ClampAxes(RawDeg, Full.MaxCorrectionDeg);
				State.bAxisLimited = FMath::Abs(RawDeg.X - WithinDeg.X) > AxisLimitSlackDeg || FMath::Abs(RawDeg.Y - WithinDeg.Y) > AxisLimitSlackDeg;

				// (A carried leg held clear stands above where the plane has
				// it: the plane coming up to it does not move it, and is not
				// slowed to its rate.)
				FLegValues AboveCm;
				if (HeldMostCm > 0.0)
				{
					FLegValues Rigid;
					RigidOffsets(Legs, State.CorrectionDeg, Rigid);
					for (int32 Index = 0; Index < Legs.Num() && Index < Rigid.Num(); ++Index)
					{
						const bool bHeld = State.HeldCm.IsValidIndex(Index) && State.HeldCm[Index] > 0.0 && State.LastDz.IsValidIndex(Index);
						AboveCm.Add(bHeld ? FMath::Max(0.0, State.LastDz[Index] - (Rigid[Index] - State.RideCm)) : 0.0);
					}
				}
				const FVector2D SlewedDeg = SlewCorrection(Legs, State.CorrectionDeg, AskedDeg, Dt, Tuning.MaxCorrectionRateDegPerSec, Tuning, RateUse, State.bRateLimited, AboveCm);

				FAllocation Alloc = Allocate(Legs, Points, SlewedDeg, Full, State.RideCm, FMath::Max(0.0, Tuning.RideHeightRateCmPerSec) * Dt);
				State.RideCm = Alloc.HeightCm;
				State.bSaturated = Alloc.Fraction < 1.0;
				State.CorrectionDeg = SlewedDeg;
				if (State.bSaturated)
				{
					// The base takes the rest. The loop is told what the legs'
					// reach cut from the plane it was moving to, so the
					// integral gives up its own part of that and no more.
					State.CorrectionDeg = Alloc.AppliedDeg;
					State.Loop.BackCalculate(SlewedDeg, Alloc.AppliedDeg, Eff);
				}
				bool	   bLiftedBehind = false;
				const bool bPlaneHeld = LawImpl::LimitLegSteps(Legs, State.LastDz, Alloc.Dz, Dt, Tuning, RateUse, State.HeightRateCmS, bLiftedBehind);
				State.bLegsBehind = bPlaneHeld || bLiftedBehind;
				State.bRateLimited = State.bRateLimited || bPlaneHeld;
				LawImpl::KeepAllocation(State, Alloc);
				bWrite = true;

				if (bActive)
				{
					// (Against the correction the joints have MADE, not the one
					// asked: a ground that steps is answered on the same step,
					// and would read as the base going with its legs.)
					State.Divergence.Tick((AttitudeDeg - WantedNow).Size(), AttitudeDeg, AchievedDeg, Alloc.Fraction >= 1.0, Dt, Tuning);
				}
				else
				{
					// What is left of the correction once the wanted attitude
					// is the ground's is the integral's trim, which no ground
					// explains: it is ramped out open loop.
					// (A walk kept waiting by a stance change is not one that
					// cannot get there: the wait does not count.)
					State.ReleaseSeconds += In.bStanceChanging ? 0.0 : Dt;
					// ("Reached" as IsNear has it: the ground's attitude is an
					// estimate that never stands quite still.)
					if ((State.Tilt.IsNear(RiderGoalDeg) && State.SeatShare.IsNear(ShareGoal)) || State.ReleaseSeconds > ReleaseMaxSeconds)
					{
						State.bReleaseOpen = true;
					}
				}
			}
		}
		else if (bOpen)
		{
			// (Legs that do not read as numbers cannot be allocated over: the
			// offsets then hold until they do.)
			if (Dt > 0.0 && bLegsValid)
			{
				if (!State.bLoopOpen)
				{
					// The ramp starts here, from the correction as it stands,
					// at rest.
					State.Ramp.Reset(State.CorrectionDeg);
					State.bLoopOpen = true;
				}
				// The way out is the tilt policy's: its rate AND its
				// acceleration, as every deliberate change of the rider's
				// attitude. The path is walked by the limiter the wanted
				// attitude uses; the correction follows it within the legs'
				// own limits and no plane rate of its own.
				const FVector2D PathDeg = State.Ramp.Tick(FVector2D::ZeroVector, Dt, Tuning.TiltRateDegPerSec, Tuning.TiltAccelDegPerSec2);
				State.CorrectionDeg = SlewCorrection(Legs, State.CorrectionDeg, PathDeg, Dt, 0.0, Tuning, RateUse, State.bRateLimited);

				FAllocation Alloc = Allocate(Legs, Points, State.CorrectionDeg, Full, State.RideCm, FMath::Max(0.0, Tuning.RideHeightRateCmPerSec) * Dt);
				State.RideCm = Alloc.HeightCm;
				State.bSaturated = Alloc.Fraction < 1.0;
				State.bAxisLimited = false;
				bool	   bLiftedBehind = false;
				const bool bPlaneHeld = LawImpl::LimitLegSteps(Legs, State.LastDz, Alloc.Dz, Dt, Tuning, RateUse, State.HeightRateCmS, bLiftedBehind);
				State.bLegsBehind = bPlaneHeld || bLiftedBehind;
				State.bRateLimited = State.bRateLimited || bPlaneHeld;
				LawImpl::KeepAllocation(State, Alloc);
				bWrite = true;
			}
			if (Dt > 0.0)
			{
				// What the seat holds of the rider's wanted pitch goes back out
				// with the correction, on the same limits.
				State.SeatShare.Tick(FVector2D::ZeroVector, Dt, Tuning.TiltRateDegPerSec, Tuning.TiltAccelDegPerSec2);
			}
			if (bValid)
			{
				// Nothing is being steered to: the error reads zero.
				State.Tilt.Reset(AttitudeDeg + FVector2D(State.SeatShare.Get().X, 0.0));
			}
		}

		if (!bActive)
		{
			// Only levelling can be short of what would hold the reference, or
			// be held back for the speed.
			State.bShort = false;
			State.GoalLimit = EReachLimit::None;
			State.bEased = false;
			State.EasedDeg = FVector2D::ZeroVector;
			State.CarryDeg = Full.MaxCorrectionDeg;
		}
		const bool bHasOffsets = State.Arbiter.HasOffsets();

		// ---- 4. The seat trim ----
		// It holds with the offsets (no attitude; at rest, where nothing is
		// written); elsewhere it runs, or runs out.
		if (Now != EState::Holding && !bQuiet && (State.Attitude.IsSeeded() || State.SeatTrim.Get() != 0.0))
		{
			// The cover runs while the loop does (levelling, and a release while
			// it is still walked). It answers where the base's pitch will be,
			// against where it is wanted, when the seat's command lands a
			// command interval from now: the ground as the gate lets it be read
			// (the body's own pitching left out), less the plane the legs'
			// joints have made and what the legs were told to add to it this
			// tick, which they will have done by then. A seat sent after the
			// error as it stands answers again what the legs have just been
			// sent after: and a knock on a robot on the move, which passes the
			// gate as ground and sends the legs after it, would be answered
			// twice over (the dry run's knocks at a crawl and at a walk: a tenth
			// more swing at the rider; with the legs' step taken off, no more
			// than with no seat at all).
			const bool bCover = bSeatRuns && bClosed && bValid && State.SeatGround.IsSeeded();
			double	   AnswerDeg = 0.0;
			if (bCover)
			{
				const double GroundNow = State.SeatGround.Get().X;
				const double WantedPitch = LawImpl::BaseWanted(State).X;
				AnswerDeg = GroundNow - AchievedDeg.X - WantedPitch;
				if (bSeatGroundBefore && !bWantedSet && Dt > 0.0)
				{
					// (Against the wanted attitude as it stands: while that is
					// walked the legs are told to follow it, and with their step
					// taken off here the seat has no part in the walk at all. The
					// rider is then a command interval behind their wanted
					// attitude, as the base is behind its own. The ground's step
					// is left out unless asked for: FSeatCoverTuning::GroundAhead.)
					AnswerDeg += State.SeatTrim.Cover.GroundAhead * (GroundNow - SeatGroundBefore) - (State.CorrectionDeg.X - CorrectionBefore);
				}
			}
			State.SeatTrim.Tick(bHasOffsets ? State.SeatShare.Get().X : 0.0, AnswerDeg, bCover, SeatLowDeg, SeatHighDeg, Dt, Tuning);
		}

		// ---- 5. The speed scale ----
		const FVector2D WantedDeg = LawImpl::BaseWanted(State);
		const double	TiltDeg = State.Attitude.IsSeeded() ? Inclination(AttitudeDeg) : 0.0;
		const double	OffReferenceDeg = State.Attitude.IsSeeded() ? Inclination(AttitudeDeg - Ref) : 0.0;
		const double	ErrorDeg = bHasOffsets && State.Attitude.IsSeeded() ? (AttitudeDeg - WantedDeg).Size() : 0.0;
		// At reach: the correction that would hold the reference cannot all
		// be given. For as long as that is so, whether the allocation is being
		// cut this tick or the wanted tilt has already taken the rest.
		const bool bAtReach = bActive && (State.bShort || State.Fraction < 1.0 || State.bAxisLimited);
		// Past it: the legs themselves stopped (the allocation cut, an axis
		// held at its limit) with the base beyond the tilt policy. Not while
		// the wanted tilt is still walking to where a slope a little past the
		// planned reach will leave it: switched on there, the base starts
		// further off than the policy allows and is on its way in.
		// (Off the reference by what the LEGS' reach leaves: a tilt the robot
		// has decided not to carry at its speed is not past reach. Counted in,
		// a cross fall of more than the policy's bound met at a speed at
		// which no roll is carried read as past reach on every tick the
		// allocation was cut for anything at all, and put the drive on the
		// scale's floor.)
		const double PastReachDeg = State.Attitude.IsSeeded() ? Inclination(AttitudeDeg - Ref - State.EasedDeg) : 0.0;
		const bool	 bPastReach = bActive && (State.Fraction < 1.0 || State.bAxisLimited) && PastReachDeg > Tuning.MaxTiltDeg;
		if (bActive || Now == EState::Releasing || bRamping)
		{
			// The cap goes by how fast the ground turns under the robot against
			// how fast the legs can turn the correction (FSpeedScale). Nothing
			// in it looks at how much reach is left, nor at how steep a steady
			// slope is: standing at a limit, or on a slope, is not a turn.
			FSpeedCapInputs Cap;
			Cap.ErrorDeg = ErrorDeg;
			// A roll the robot has decided not to carry at its speed is not
			// followed: neither a ground that turns in roll under it nor what
			// that leaves between the base and where it is wanted (the wanted
			// attitude walks after the ground within the tilt policy, the base
			// goes with the ground at once) is a reason to slow. The drive
			// would be held back for a turn no leg is answering. (By the share
			// of the roll limit that is left, while that roll is out or on its
			// way out.)
			const double RollShare = bActive && State.Carry.IsRollOut() && Full.MaxCorrectionDeg.Y > 0.0 ? FMath::Clamp(State.CarryDeg.Y / Full.MaxCorrectionDeg.Y, 0.0, 1.0) : 1.0;
			if (RollShare < 1.0 && bHasOffsets && State.Attitude.IsSeeded())
			{
				const FVector2D Off = AttitudeDeg - WantedDeg;
				Cap.ErrorDeg = FVector2D(Off.X, Off.Y * RollShare).Size();
			}
			Cap.bPastReach = bPastReach;
			Cap.SpeedCmS = In.SpeedCmS;
			Cap.AskedSpeedCmS = In.AskedSpeedCmS;
			Cap.YawRateDegPerSec = In.YawRateDegPerSec;
			if (State.Terrain.IsSeeded() && bLegsValid)
			{
				Cap.SlopeDeg = Inclination(GroundDeg);
				Cap.SpanCm = StandingSpanCm(Legs);
				// Not the ground's doing: an estimate just seeded afresh, and
				// one that a stance being changed is turning.
				Cap.bGroundRateKnown = Dt > 0.0 && bValid && !bWantedSet && !bBaselinesMoving;
				if (Cap.bGroundRateKnown)
				{
					Cap.GroundRateDegPerSec = (GroundDeg - PrevGroundDeg) / Dt;
				}
				// ... and where a break shows first: the ground as the gate
				// lets it be read, before the estimate's own low-pass.
				// (No break turns the ground faster than the sharpest the gate
				// follows whole: what reads as more is not ground.)
				Cap.MaxTurnDegPerCm = Cap.SpanCm > 0.0 ? FMath::Max(0.0, State.Ground.Tuning.BreakDeg) / Cap.SpanCm : 0.0;
				Cap.bSeenRateKnown = Cap.bGroundRateKnown && bSeenBefore && State.Ground.IsSeeded();
				if (Cap.bSeenRateKnown)
				{
					Cap.SeenRateDegPerSec = (State.Ground.Get() - PrevSeenDeg) / Dt;
				}
				// (The ground's turning in roll: as the error above.)
				Cap.GroundRateDegPerSec.Y *= RollShare;
				Cap.SeenRateDegPerSec.Y *= RollShare;
				// What the legs can turn the correction at: the lesser of the
				// way the slope would be taken out and the way the ground is
				// turning now, where there is either.
				const double Along[2] = { PlaneRateDegPerSec(Legs, GroundDeg, Tuning), PlaneRateDegPerSec(Legs, Cap.GroundRateDegPerSec, Tuning) };
				Cap.LegsDegPerSec = Along[0] > 0.0 && Along[1] > 0.0 ? FMath::Min(Along[0], Along[1]) : FMath::Max(Along[0], Along[1]);
			}
			// A three-point robot still carrying roll is kept to the speed
			// that roll allows.
			Cap.HoldSpeedCmS = bActive ? State.Carry.HoldSpeedCmS(State.CorrectionDeg.Y, Full.MaxCorrectionDeg.Y) : 0.0;
			State.SpeedScale.Tick(Cap, Dt, Tuning);
		}
		else if (Now != EState::Holding)
		{
			State.SpeedScale.Relax(Dt, Tuning);
		}
		State.PrevGroundDeg = GroundDeg;

		// ---- 6. Outputs ----
		FOutputs Out;
		if (bHasOffsets)
		{
			// (Over the legs' own baselines: with what each is held clear by.)
			Out.Dz = State.TrueOffsets();
		}
		else
		{
			Out.Dz.Init(0.0, NumLegs);
		}
		Out.bWrite = bWrite;
		Out.bClaim = State.Arbiter.WantsClaim();
		Out.State = Now;
		Out.Action = Arbiter.Action;
		Out.bUnexpected = Arbiter.bUnexpected;
		Out.bOpenLoop = bOpen;

		Out.AttitudeDeg = AttitudeDeg;
		Out.GroundDeg = GroundDeg;
		Out.WantedDeg = WantedDeg;
		Out.AppliedDeg = State.CorrectionDeg;
		Out.RideCm = State.RideCm;
		Out.SeatTrimDeg = State.SeatTrim.Get();
		Out.SeatShareDeg = State.SeatShare.Get().X;
		Out.RiderWantedDeg = State.Tilt.Get();
		Out.SpeedScale = State.SpeedScale.Get();

		Out.TiltDeg = TiltDeg;
		Out.ErrorDeg = ErrorDeg;
		Out.Fraction = State.Fraction;
		Out.ReachLeft = State.ReachLeft;
		Out.MinMarginCm = State.MinMarginCm;
		Out.LimitingLeg = State.LimitingLeg;
		Out.LimitingPoint = State.LimitingPoint;
		if (bAtReach)
		{
			// What stops it: the allocation's own limit where it is being
			// cut, else what the goal found, else an axis held at its limit,
			// else whatever is nearest.
			Out.ReachLimit = State.Fraction < 1.0 ? State.Limit : EReachLimit::None;
			if (Out.ReachLimit == EReachLimit::None && State.bShort)
			{
				Out.ReachLimit = State.GoalLimit;
			}
			if (Out.ReachLimit == EReachLimit::None && State.bAxisLimited)
			{
				Out.ReachLimit = EReachLimit::AxisLimit;
			}
			if (Out.ReachLimit == EReachLimit::None)
			{
				Out.ReachLimit = State.NearestLimit;
			}
		}
		Out.LoopAttitudeDeg = LoopAttitudeDeg;
		Out.RateUse = RateUse;
		Out.bRateLimited = State.bRateLimited;
		Out.bQuiet = bQuiet;
		Out.CarryDeg = State.CarryDeg;
		Out.bEased = State.bEased;
		Out.SagDeg = SagDeg;

		switch (Now)
		{
			case EState::Faulted:
				Out.Status = EStatus::Diverged;
				break;
			case EState::Holding:
				Out.Status = State.LossSeconds >= Tuning.MeasurementLossSeconds ? EStatus::AttitudeLost : EStatus::Holding;
				break;
			case EState::Off:
				Out.Status = EStatus::Off;
				break;
			case EState::Yielded:
				Out.Status = EStatus::Yielded;
				break;
			case EState::Suspended:
				Out.Status = EStatus::Waiting;
				break;
			case EState::Releasing:
				Out.Status = EStatus::Releasing;
				break;
			case EState::Active:
			default:
				if (OffReferenceDeg > Tuning.FaultTiltDeg)
				{
					Out.Status = EStatus::TiltFault;
				}
				else if (bPastReach)
				{
					Out.Status = EStatus::PastReach;
				}
				else if (bAtReach)
				{
					Out.Status = EStatus::AtReach;
				}
				else
				{
					Out.Status = bQuiet ? EStatus::Settled : EStatus::Levelling;
				}
				break;
		}
		return Out;
	}
} // namespace RammsLevel
