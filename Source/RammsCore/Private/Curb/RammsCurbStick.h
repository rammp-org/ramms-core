// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

struct FRammsCurbStick;

/**
 * The rider's stick in a crossing (FRammsCurbStick), read as plain C++ so
 * that it can be tested on made-up input. Not reflected, not exported.
 */
namespace RammsCurb
{
	/** Paused by the stick with nothing moving for this long (s), the crossing
	 *  holds every wheel where it stands, braked where it has a brake. */
	constexpr double PauseHoldSeconds = 0.5;

	/** What the rider asks of a crossing with the stick. */
	enum class EStickIntent : uint8
	{
		/** Held forward: go on, at FStickGate::GetScale of the plan's speed. */
		Forward,
		/** At rest, or forward too little to go. */
		LetGo,
		/** Pushed to one side, or forward too far round to one side. */
		Sideways,
		/** Pulled back. */
		Back,
	};

	const TCHAR* StickIntentName(EStickIntent Intent);

	/**
	 * The stick, read once a tick. It reads forward once pushed past
	 * EngageFraction within ForwardSectorDeg of straight ahead, and stays
	 * forward until it has been under ReleaseFraction, or out of that sector,
	 * for ReleaseSeconds -- keeping the share it last had meanwhile -- so that
	 * a hand that wavers does not stop the robot. Out of forward it reads what
	 * it is at once: at rest (under ReleaseFraction, or forward under
	 * EngageFraction), back (within ForwardSectorDeg of straight back) or
	 * sideways.
	 */
	class FStickGate
	{
	public:
		void Reset();

		/** One tick of Dt (s): Forward, Turn and Strafe as the drive controls
		 *  carry them, each -1..1 (forward, turning right, to the right). */
		EStickIntent Step(const FRammsCurbStick& Settings, double Forward, double Turn, double Strafe, double Dt);

		EStickIntent GetIntent() const { return Intent; }
		/** Held forward, how far (0..1): the share of the plan's speed asked
		 *  for; else 0. */
		double GetScale() const { return Scale; }
		/** How long (s) the stick has read what it reads now, without a break. */
		double GetSeconds() const { return Seconds; }
		/** How long (s) the stick has read back without a break; 0 when it does
		 *  not. */
		double GetBackSeconds() const { return Intent == EStickIntent::Back ? Seconds : 0.0; }

	private:
		EStickIntent Intent = EStickIntent::LetGo;
		double		 Scale = 0.0;
		/** Held forward, how long (s) it has been out of forward. */
		double OutSeconds = 0.0;
		double Seconds = 0.0;
	};

	/** Where a crossing is, for what the stick does to it. */
	struct FStickStage
	{
		/** A leg has moved for the curb: the level-in has begun. */
		bool bLegsMoved = false;
		/** A wheel has begun to cross, or every wheel is across (settling). */
		bool bCrossing = false;
		/** The level-in is done. */
		bool bLevelled = false;
		/** The robot is at rest: its command at 0 and its chassis measured still. */
		bool bAtRest = false;
	};

	/** What the stick does to a crossing on a tick. */
	enum class EStickAction : uint8
	{
		/** Go on, at FStickDecision::Scale of the plan's speed. */
		Go,
		/** Brake to rest and wait (the crossing's phase Paused). */
		Pause,
		/** End the crossing and hand the robot back to its drive mode. */
		Cancel,
	};

	struct FStickDecision
	{
		EStickAction Action = EStickAction::Pause;
		/** Go: the share of the plan's speed the stick asks. */
		double Scale = 0.0;
		/** Pause: the status line, saying why and what the stick does next;
		 *  Cancel: why, for the log. */
		FString Why;
	};

	/**
	 * What the stick, as Gate reads it now, does to a crossing at Stage. Held
	 * forward, go at its share. Before any leg has moved for the curb, back or
	 * sideways read for ReleaseSeconds cancels. Levelled in with no wheel yet
	 * lifted, back read for CancelHoldSeconds with the robot at rest cancels.
	 * Anything else -- let go, or any direction once a wheel has begun to
	 * cross -- pauses.
	 */
	FStickDecision DecideStick(const FRammsCurbStick& Settings, const FStickGate& Gate, const FStickStage& Stage);

	/** A crossing's hold time (s) one tick of Dt on: counted while it holds,
	 *  kept as it is while the rider's stick pauses it (letting go and pushing
	 *  again does not restart a hold's timeout), and 0 otherwise. */
	double StepHoldSeconds(bool bHolding, bool bPaused, double Seconds, double Dt);
} // namespace RammsCurb
