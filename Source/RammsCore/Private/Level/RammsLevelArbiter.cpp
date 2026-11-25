// Copyright Epic Games, Inc. All Rights Reserved.

#include "Level/RammsLevelArbiter.h"

namespace RammsLevel
{
	FArbiterStep FArbiter::Tick(const FArbiterInputs& In)
	{
		// The rules are the header's, in the header's order, and the first that
		// applies ends the tick. Each branch below is one row of that table.
		const bool bReady = In.bEnable && In.Mode == EMode::Allowed && In.bAttitudeValid;

		EAction Action = EAction::None;
		bool	bUnexpected = false;

		// A start from nothing waits for a stance that is being changed: the
		// drive mode's ramp is rocking the base onto other wheels, and a wanted
		// attitude set walking now would turn the rider on top of that.
		const bool bMayStart = bReady && !In.bStanceChanging;

		// Where a state with no offsets goes when nothing was handed back: on
		// to levelling if it may, else to the state that says why it waits.
		// Rules 5 and 6 are this and nothing else.
		const auto EnterWithoutOffsets = [&]() {
			if (!In.bEnable)
			{
				State = EState::Off;
			}
			else if (bMayStart)
			{
				State = EState::Active;
				Action = EAction::StartFromAttitude;
			}
			else if (In.Mode == EMode::Taken)
			{
				State = EState::Yielded;
			}
			else
			{
				State = EState::Suspended;
			}
		};

		// Rule 3, a taker handing the legs back. The mode is not read: the
		// taker's own mode is still the active one on this tick.
		const auto Resume = [&]() {
			if (!In.bEnable)
			{
				State = EState::Off;
				Action = EAction::Clear;
			}
			else if (In.bResumeHasPlane)
			{
				// The plane is out of the baselines already, so it has to be
				// taken up even with no attitude to level against: held.
				State = In.bAttitudeValid ? EState::Active : EState::Holding;
				Action = EAction::Adopt;
			}
			else if (In.bAttitudeValid)
			{
				State = EState::Active;
				Action = EAction::StartFromAttitude;
			}
			else
			{
				State = EState::Suspended;
				Action = EAction::None;
			}
		};

		if (In.bColdReset)
		{
			// Rule 1. Nothing that was applied can be trusted to still be
			// there, so there is nothing to ramp out or fold: zero at once.
			if (State == EState::Faulted)
			{
				bFaultLatched = true;
				Action = EAction::Clear;
			}
			else if (bMayStart)
			{
				State = EState::Active;
				Action = EAction::StartFromAttitude;
			}
			else
			{
				State = !In.bEnable ? EState::Off : (In.Mode == EMode::Taken ? EState::Yielded : EState::Suspended);
				Action = EAction::Clear;
			}
		}
		else if (In.bYield)
		{
			// Rule 2. The offsets go into the baselines exactly once: a state
			// that has none (a second yield, a yield while off) folds nothing.
			if (HasOffsets())
			{
				Action = EAction::Fold;
				if (State == EState::Faulted)
				{
					// A fault does not get its levelling back from a crossing.
					bFaultLatched = true;
				}
				else
				{
					State = In.bEnable ? EState::Yielded : EState::Off;
				}
			}
			else if (State == EState::Suspended)
			{
				State = EState::Yielded;
			}

			if (In.bResume && State == EState::Yielded)
			{
				// A take-over refused inside the same call. The hand-back's
				// action is the one reported; only where the hand-back asks
				// for nothing is the fold still the thing to be told, or the
				// law would keep a correction the baselines now hold.
				const EAction Folded = Action;
				Resume();
				if (Action == EAction::None)
				{
					Action = Folded;
				}
			}
		}
		else if (In.bResume && State == EState::Yielded)
		{
			Resume();
		}
		else if (State == EState::Faulted)
		{
			// Rule 4.
			if (bFaultLatched)
			{
				if (!In.bEnable)
				{
					// Switched off: that is what clears the latch.
					State = EState::Off;
				}
			}
			else if (In.bCorrectionZero)
			{
				bFaultLatched = true;
				Action = EAction::Clear;
			}
		}
		else if (State == EState::Off || State == EState::Yielded || State == EState::Suspended)
		{
			// Rules 5 and 6.
			EnterWithoutOffsets();
		}
		else if (In.Mode == EMode::Taken)
		{
			// Rule 7. Active, Holding or Releasing, and something took the legs
			// without saying so: fold rather than fight it, and say it was not
			// announced so the shell can sort the motors and warn.
			State = EState::Yielded;
			Action = EAction::Fold;
			bUnexpected = true;
		}
		else if (!In.bEnable || In.Mode == EMode::Release)
		{
			// Rule 8. Let go gently, and only stop once the base is back on
			// its stance.
			if (State != EState::Releasing)
			{
				State = EState::Releasing;
				Action = EAction::BeginRelease;
			}
			else if (In.bCorrectionZero)
			{
				State = In.bEnable ? EState::Suspended : EState::Off;
				Action = EAction::Clear;
			}
		}
		else if (State == EState::Releasing)
		{
			// Rule 9. Wanted again part way through a release: go on from
			// where the release had got to, with no restart.
			if (In.bAttitudeValid)
			{
				State = EState::Active;
			}
			else
			{
				State = EState::Holding;
				Action = EAction::Freeze;
			}
		}
		else if (State == EState::Active)
		{
			// Rule 10. A missing attitude comes first: a divergence judged on
			// no reading is not one.
			if (!In.bAttitudeValid)
			{
				State = EState::Holding;
				Action = EAction::Freeze;
			}
			else if (In.bDiverged)
			{
				State = EState::Faulted;
				Action = EAction::BeginRelease;
			}
		}
		else if (In.bAttitudeValid)
		{
			// Rule 11. Holding, and the attitude is back.
			State = EState::Active;
			Action = EAction::ColdResume;
		}

		// The latch belongs to Faulted alone: whatever left it starts anew.
		if (State != EState::Faulted)
		{
			bFaultLatched = false;
		}

		FArbiterStep Step;
		Step.State = State;
		Step.Action = Action;
		Step.bUnexpected = bUnexpected;
		return Step;
	}
} // namespace RammsLevel
