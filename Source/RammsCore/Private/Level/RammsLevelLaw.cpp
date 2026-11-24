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
			State.Fraction = 1.0;
			State.ReachLeft = 1.0;
			State.MinMarginCm = NoTravelMarginCm;
			State.LimitingLeg = INDEX_NONE;
			State.LimitingPoint = INDEX_NONE;
			State.bSaturated = false;
			State.bAxisLimited = false;
			State.bRateLimited = false;
			State.bLegsBehind = false;
			State.bLoopOpen = false;
		}

		/** A leg, or a point, every number of which is one. */
		bool IsFinite(const FLegReach& Leg)
		{
			return IsFinite(Leg.XY) && FMath::IsFinite(Leg.BaseZ) && FMath::IsFinite(Leg.MeasuredZ) && FMath::IsFinite(Leg.MinZ) && FMath::IsFinite(Leg.MaxZ)
				&& FMath::IsFinite(Leg.MaxRateCmS) && FMath::IsFinite(Leg.RadiusCm) && FMath::IsFinite(Leg.BaselineClearanceCm) && FMath::IsFinite(Leg.CarryShare);
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
				State.Terrain.Seed(In.AttitudeDeg + AchievedDeg);
			}
			if (State.Attitude.IsSeeded())
			{
				State.Tilt.Reset(State.Attitude.Get());
				State.Loop.StartFrom(State.CorrectionDeg, State.Tilt.Get(), State.Terrain.Get(), Tuning);
			}
			else
			{
				// No attitude has ever been read (a plane adopted blind): there
				// is nothing to start the loop from. The resume that follows
				// the first reading does it.
				State.Tilt.Reset(Tuning.LevelReferenceDeg);
				State.Loop.Reset();
			}
			State.Quiet.Reset();
			State.Divergence.Reset();
			State.Support.Reset();
			State.bSaturated = false;
			State.bAxisLimited = false;
			State.bRateLimited = false;
			State.bLegsBehind = false;
			State.bReleaseOpen = false;
			State.bLoopOpen = false;
			State.PrevGroundDeg = State.Terrain.Get();
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
		 * leg within LegRateFraction of its rate. A blend of two rigid planes
		 * is a rigid plane, and so is one moved by a common height.
		 *
		 * Returns whether either held the offsets back; the leg-rate use the
		 * step asked (before any cut) is taken into InOutRateUse.
		 */
		bool LimitLegSteps(const FLegs& Legs, const FLegValues& From, FLegValues& InOutDz, double Dt, const FTuning& Tuning, double& InOutRateUse,
			double& InOutHeightRate)
		{
			if (Dt <= 0.0 || From.Num() != InOutDz.Num() || Legs.Num() != InOutDz.Num() || Legs.Num() == 0)
			{
				return false;
			}
			double Use = 0.0;
			double SupportSum = 0.0;
			double AllSum = 0.0;
			int32  SupportCount = 0;
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				const double Move = InOutDz[Index] - From[Index];
				if (Legs[Index].MaxRateCmS > 0.0)
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
					if (Legs[Index].MaxRateCmS > 0.0)
					{
						LeftUse = FMath::Max(LeftUse, FMath::Abs(InOutDz[Index] - Held - From[Index]) / (Dt * Legs[Index].MaxRateCmS));
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
			if (Held == 0.0 && Share == 1.0)
			{
				return false;
			}
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				const FLegReach& Leg = Legs[Index];
				const double	 Dz = From[Index] + (InOutDz[Index] - Held - From[Index]) * Share;
				// Allocate's last resort, kept: never past the travel.
				InOutDz[Index] = FMath::Clamp(Dz, FMath::Min(0.0, Leg.MinZ - Leg.BaseZ), FMath::Max(0.0, Leg.MaxZ - Leg.BaseZ));
			}
			return true;
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
		}
	} // namespace LawImpl

	FOutputs FLaw::Step(const FInputs& In, double InDt, const FTuning& Tuning, FState& State)
	{
		// ---- 0. Housekeeping ----
		const double	Dt = FMath::IsFinite(InDt) ? FMath::Clamp(InDt, 0.0, MaxStepSeconds) : 0.0;
		const int32		NumLegs = In.Legs.Num();
		const FVector2D Ref = Tuning.LevelReferenceDeg;

		// Other legs than last step's: nothing kept per leg means anything now.
		const bool bColdReset = In.bColdReset || State.LegCount != NumLegs;
		State.LegCount = NumLegs;
		if (State.LastDz.Num() != NumLegs)
		{
			State.LastDz.Init(0.0, NumLegs);
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

		// What the last step left, for "is anything moving" further down.
		const FVector2D PrevGroundDeg = State.PrevGroundDeg;
		const FVector2D PrevCorrectionDeg = State.CorrectionDeg;
		const FVector2D PrevWantedDeg = State.Tilt.Get();

		// ---- 1. Sensing ----
		FVector2D AchievedDeg = FVector2D::ZeroVector;
		if (bValid)
		{
			// The correction the joints have really made, not the one asked
			// for: sag, rate limits and travel stops then cancel in G. Made by
			// the legs the robot stands on: who they are is found here as the
			// allocation will find it (the set kept in the state is not
			// touched: that is PrepareLegs' to move on).
			LawImpl::FLegs Sensed;
			LawImpl::MaskLegs(In, Tuning, State.Support, State.LastDz, Sensed);
			AchievedDeg = MeasuredCorrection(Sensed);
			const FVector2D Filtered = State.Attitude.Tick(In.AttitudeDeg, Dt, Tuning.AttitudeFilterSeconds);
			State.Terrain.Tick(Filtered, AchievedDeg, Dt, Tuning.TerrainFilterSeconds);
			State.LossSeconds = 0.0;
		}
		else
		{
			State.LossSeconds += Dt;
		}

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
			&& FMath::Abs(State.SeatTrim.Get()) < ReleaseDoneDeg && !State.bLegsBehind;
		Ask.bDiverged = State.Divergence.HasDiverged();
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
				State.SeatTrim.Zero();
				LawImpl::ColdResume(State, In, AchievedDeg, bValid, Tuning);
				bWantedSet = true;
				bAdopted = true;
				break;
			case EAction::Fold:
				LawImpl::ZeroCorrection(State, NumLegs);
				State.SeatTrim.Zero();
				break;
			case EAction::Clear:
				LawImpl::ZeroCorrection(State, NumLegs);
				State.SeatTrim.Zero();
				bCleared = true;
				break;
			case EAction::BeginRelease:
				State.ReleaseSeconds = 0.0;
				State.bReleaseOpen = false;
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

		LawImpl::FLegs Legs;
		int32		   Standing = 0;
		if (bClosed || bOpen || bAdopted)
		{
			Standing = LawImpl::PrepareLegs(In, Tuning, State, Legs);
		}
		if (bAdopted && bLegsValid)
		{
			// The offsets the plant has just put on the motors: the adopted
			// plane, allocated with no ride shift and no slew.
			LawImpl::KeepAllocation(State, Allocate(Legs, In.Points, State.CorrectionDeg, Tuning, 0.0, 0.0));
		}

		bool bQuiet = false;
		bool bWrite = bCleared;
		// The legs' rate use: the slew's alone (what the ground asks of the
		// legs, which is the speed cap's business), and with the offsets' own
		// taken in (what is reported).
		double SlewUse = 0.0;
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
				State.Tilt.Reset(AttitudeDeg);
				State.Loop.StartFrom(State.CorrectionDeg, State.Tilt.Get(), GroundDeg, Tuning);
				State.bLoopOpen = false;
				State.bSaturated = false;
				State.bRateLimited = false;
				bWantedSet = true;
			}
			const FVector2D GoalDeg = bActive ? WantedTiltGoal(GroundDeg, Legs, In.Points, Tuning) : GroundDeg;
			if (bActive)
			{
				bQuiet = State.Quiet.Tick(In.SpeedCmS, (AttitudeDeg - State.Tilt.Get()).Size(), In.bStick, State.Tilt.IsNear(GoalDeg), Dt, Tuning);
			}
			// At rest nothing is written, so the servos simply hold; and a step
			// in which no time passes advances nothing.
			if (!bQuiet && Dt > 0.0)
			{
				if (!bWantedSet)
				{
					State.Tilt.Tick(GoalDeg, Dt, Tuning.TiltRateDegPerSec, Tuning.TiltAccelDegPerSec2);
				}
				const FVector2D WantedNow = State.Tilt.Get();

				// The integral only trims while levelling, on a stance that can
				// be levelled, and not while the legs are already going as fast
				// as they can.
				const bool		bIntegrate = bActive && Standing >= MinSupportForIntegral && !State.bRateLimited;
				const FVector2D RawDeg = State.Loop.Tick(AttitudeDeg, WantedNow, GroundDeg, Dt, bIntegrate, State.bSaturated, Tuning);

				// The per-axis limit is a limit like the legs' reach: asking to
				// pass it is being at reach. (The loop holds its own integral
				// on an axis that is at its limit.)
				const FVector2D AskedDeg = ClampAxes(RawDeg, Tuning.MaxCorrectionDeg);
				State.bAxisLimited = FMath::Abs(RawDeg.X - AskedDeg.X) > AxisLimitSlackDeg || FMath::Abs(RawDeg.Y - AskedDeg.Y) > AxisLimitSlackDeg;

				const FVector2D SlewedDeg = SlewCorrection(Legs, State.CorrectionDeg, AskedDeg, Dt, Tuning.MaxCorrectionRateDegPerSec, Tuning, SlewUse, State.bRateLimited);
				RateUse = SlewUse;

				FAllocation Alloc = Allocate(Legs, In.Points, SlewedDeg, Tuning, State.RideCm, FMath::Max(0.0, Tuning.RideHeightRateCmPerSec) * Dt);
				State.RideCm = Alloc.HeightCm;
				State.bSaturated = Alloc.Fraction < 1.0;
				State.CorrectionDeg = SlewedDeg;
				if (State.bSaturated)
				{
					// The base takes the rest. The loop is told what the legs'
					// reach cut from the plane it was moving to, so the
					// integral gives up its own part of that and no more.
					State.CorrectionDeg = Alloc.AppliedDeg;
					State.Loop.BackCalculate(SlewedDeg, Alloc.AppliedDeg, Tuning);
				}
				State.bLegsBehind = LawImpl::LimitLegSteps(Legs, State.LastDz, Alloc.Dz, Dt, Tuning, RateUse, State.HeightRateCmS);
				State.bRateLimited = State.bRateLimited || State.bLegsBehind;
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
					State.ReleaseSeconds += Dt;
					// ("Reached" as IsNear has it: the ground's attitude is an
					// estimate that never stands quite still.)
					if (State.Tilt.IsNear(GroundDeg) || State.ReleaseSeconds > ReleaseMaxSeconds)
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
				State.CorrectionDeg = SlewCorrection(Legs, State.CorrectionDeg, PathDeg, Dt, 0.0, Tuning, SlewUse, State.bRateLimited);
				RateUse = SlewUse;

				FAllocation Alloc = Allocate(Legs, In.Points, State.CorrectionDeg, Tuning, State.RideCm, FMath::Max(0.0, Tuning.RideHeightRateCmPerSec) * Dt);
				State.RideCm = Alloc.HeightCm;
				State.bSaturated = Alloc.Fraction < 1.0;
				State.bAxisLimited = false;
				State.bLegsBehind = LawImpl::LimitLegSteps(Legs, State.LastDz, Alloc.Dz, Dt, Tuning, RateUse, State.HeightRateCmS);
				State.bRateLimited = State.bRateLimited || State.bLegsBehind;
				LawImpl::KeepAllocation(State, Alloc);
				bWrite = true;
			}
			if (bValid)
			{
				// Nothing is being steered to: the error reads zero.
				State.Tilt.Reset(AttitudeDeg);
			}
		}

		const bool bHasOffsets = State.Arbiter.HasOffsets();

		// ---- 4. The seat trim ----
		// It holds with the offsets (no attitude; at rest, where nothing is
		// written); elsewhere it runs, or runs out.
		if (Now != EState::Holding && !bQuiet && (State.Attitude.IsSeeded() || State.SeatTrim.Get() != 0.0))
		{
			const bool	 bRun = In.bSeat && Tuning.bSeatCoversTransients && bActive && bValid;
			const double BasePitchDeg = State.Attitude.IsSeeded() ? AttitudeDeg.X - Ref.X : 0.0;
			State.SeatTrim.Tick(BasePitchDeg, bRun, Dt, Tuning);
		}

		// ---- 5. The speed scale ----
		const FVector2D WantedDeg = State.Tilt.Get();
		const double	TiltDeg = State.Attitude.IsSeeded() ? Inclination(AttitudeDeg) : 0.0;
		const double	OffReferenceDeg = State.Attitude.IsSeeded() ? Inclination(AttitudeDeg - Ref) : 0.0;
		const double	ErrorDeg = bHasOffsets && State.Attitude.IsSeeded() ? (AttitudeDeg - WantedDeg).Size() : 0.0;
		const bool		bAtReach = bActive && (State.Fraction < 1.0 || State.bAxisLimited);
		const bool		bPastReach = bAtReach && OffReferenceDeg > Tuning.MaxTiltDeg;
		if (bActive || Now == EState::Releasing || bRamping)
		{
			bool bMoving = false;
			if (Dt > 0.0 && !bWantedSet)
			{
				const double Moved = FMath::Max3((GroundDeg - PrevGroundDeg).Size(), (State.CorrectionDeg - PrevCorrectionDeg).Size(), (WantedDeg - PrevWantedDeg).Size());
				bMoving = Moved > SpeedCapMovingDegPerSec * Dt;
			}
			// (The slew's use, not the offsets': a height term catching up
			// after a support change is not the ground outrunning the legs,
			// and driving slower would not help it.)
			State.SpeedScale.Tick(ErrorDeg, State.MinMarginCm, bMoving, bQuiet ? 0.0 : SlewUse, bPastReach, Dt, Tuning);
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
			Out.Dz = State.LastDz;
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
		Out.SpeedScale = State.SpeedScale.Get();

		Out.TiltDeg = TiltDeg;
		Out.ErrorDeg = ErrorDeg;
		Out.Fraction = State.Fraction;
		Out.ReachLeft = State.ReachLeft;
		Out.MinMarginCm = State.MinMarginCm;
		Out.LimitingLeg = State.LimitingLeg;
		Out.LimitingPoint = State.LimitingPoint;
		Out.RateUse = RateUse;
		Out.bRateLimited = State.bRateLimited;
		Out.bQuiet = bQuiet;

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
