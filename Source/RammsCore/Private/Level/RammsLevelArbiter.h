// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Level/RammsLevelTypes.h"

/**
 * Who has the legs: the level assist's state machine, as plain C++ with no
 * numbers in it. It decides the state (EState) and, on the tick a state
 * changes, the one thing to be done (EAction); FLaw::Step runs it first each
 * tick and does what it says. It moves nothing itself.
 *
 * THE RULES. Each Tick takes the first rule that applies, top to bottom.
 * "Ready" is bEnable and Mode == Allowed and bAttitudeValid. "Has offsets"
 * is Active, Holding, Releasing, or Faulted not yet latched.
 *
 *  1. bColdReset (time ran backwards, the robot was teleported, the legs
 *     changed): whatever was applied is gone at once.
 *       Faulted                    -> Faulted, latched, Clear
 *       Ready                      -> Active, StartFromAttitude
 *       not bEnable                -> Off, Clear
 *       else, Mode == Taken        -> Yielded, Clear
 *       else                       -> Suspended, Clear
 *
 *  2. bYield (a curb controller is taking the legs, YieldTo). The offsets go
 *     into the baselines, once:
 *       has offsets, Faulted       -> Faulted, latched, Fold
 *       has offsets, bEnable       -> Yielded, Fold
 *       has offsets, not bEnable   -> Off, Fold
 *       Suspended                  -> Yielded, None
 *       Yielded, Off, latched      -> unchanged, None (nothing to fold)
 *     If bResume is set on the same tick and this leaves Yielded, rule 3 is
 *     then applied and its action is the one reported (a take-over refused in
 *     the same call: folded and adopted back with nothing in between) -- but
 *     where rule 3's action is None, a Fold from this rule is still reported:
 *     the offsets did go into the baselines, and the law must be told to zero
 *     its correction. Otherwise the tick ends here, with the action above.
 *
 *  3. bResume (the taker hands back, ResumeFrom), only in Yielded; ignored
 *     in every other state (the rules below then apply). Mode is not looked
 *     at on this tick: the taker's mode is still the active one.
 *       not bEnable                              -> Off, Clear
 *       bResumeHasPlane, bAttitudeValid          -> Active, Adopt
 *       bResumeHasPlane, attitude not valid      -> Holding, Adopt
 *       no plane, bAttitudeValid                 -> Active, StartFromAttitude
 *       no plane, attitude not valid             -> Suspended, None
 *
 *  4. Faulted:
 *       latched, not bEnable       -> Off, None (switched off: the latch is
 *                                     cleared, and switching on starts anew)
 *       latched                    -> unchanged, None
 *       bCorrectionZero            -> latched, Clear
 *       else                       -> unchanged, None (ramping out)
 *
 *  5. Off:
 *       not bEnable                -> Off, None
 *       Ready                      -> Active, StartFromAttitude
 *       Mode == Taken              -> Yielded, None
 *       else                       -> Suspended, None
 *
 *  6. Yielded or Suspended (no offsets):
 *       not bEnable                -> Off, None
 *       Ready                      -> Active, StartFromAttitude
 *       Mode == Taken              -> Yielded, None
 *       else                       -> Suspended, None
 *     (A Yielded that sees an allowed mode with no hand-back -- a crossing
 *     aborted, the curb mode left by hand -- starts again cold. The shell
 *     reports Taken for as long as a taker it yielded to holds the legs,
 *     whatever mode is active.)
 *
 *  7. Active, Holding or Releasing, with Mode == Taken and no yield (a path
 *     that forgot to yield): -> Yielded, Fold, bUnexpected. The shell folds
 *     each motor whose baseline was left alone and zeroes the offset of each
 *     that was rewritten, and warns.
 *
 *  8. Active, Holding or Releasing, with not bEnable or Mode == Release:
 *       Releasing, bCorrectionZero -> Clear, and Off (not bEnable) or
 *                                     Suspended
 *       Releasing                  -> unchanged, None
 *       Active or Holding          -> Releasing, BeginRelease
 *
 *  9. Releasing (and so bEnable and Mode == Allowed: switched back on, or
 *     the mode came back):
 *       bAttitudeValid             -> Active, None (it goes on from where
 *                                     the release had got to)
 *       else                       -> Holding, Freeze
 *
 * 10. Active:
 *       not bAttitudeValid         -> Holding, Freeze
 *       bDiverged                  -> Faulted (not latched), BeginRelease
 *       else                       -> unchanged, None
 *
 * 11. Holding:
 *       bAttitudeValid             -> Active, ColdResume
 *       else                       -> unchanged, None
 */
namespace RammsLevel
{
	/** What one tick of the arbiter is told. */
	struct FArbiterInputs
	{
		/** The switch (level.assist): a level, not an edge. */
		bool bEnable = false;
		/** The active drive mode's kind. */
		EMode Mode = EMode::Allowed;
		/** The attitude was read and is a number. */
		bool bAttitudeValid = true;
		/** Once: a taker called YieldTo since the last tick. */
		bool bYield = false;
		/** Once: the taker called ResumeFrom since the last tick ... */
		bool bResume = false;
		/** ... and the plant took a plane back out of the baselines for the
		 *  law to go on with. */
		bool bResumeHasPlane = false;
		/** Once: time ran backwards, the pose jumped, or the legs changed. */
		bool bColdReset = false;
		/** The law's correction, ride shift and seat trim are all under their
		 *  ReleaseDone thresholds (as they stood before this tick). */
		bool bCorrectionZero = false;
		/** The law's divergence watch has tripped. */
		bool bDiverged = false;
	};

	/** What it answers. */
	struct FArbiterStep
	{
		EState	State = EState::Off;
		EAction Action = EAction::None;
		/** Rule 7: the Fold is for a take-over nobody announced. */
		bool bUnexpected = false;
	};

	class FArbiter
	{
	public:
		void Reset() { *this = FArbiter(); }

		/** One tick of the rules above. */
		FArbiterStep Tick(const FArbiterInputs& In);

		EState GetState() const { return State; }
		/** Faulted and done ramping out: off until switched off and on. */
		bool IsFaultLatched() const { return State == EState::Faulted && bFaultLatched; }
		/** Offsets may be non-zero: Active, Holding, Releasing, or a Faulted
		 *  not yet latched. */
		bool HasOffsets() const
		{
			return State == EState::Active || State == EState::Holding || State == EState::Releasing
				|| (State == EState::Faulted && !bFaultLatched);
		}
		/** The motors should be claimed: every state but Off and a latched
		 *  Faulted. */
		bool WantsClaim() const { return State != EState::Off && !IsFaultLatched(); }

	private:
		EState State = EState::Off;
		bool   bFaultLatched = false;
	};
} // namespace RammsLevel
