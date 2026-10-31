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
} // namespace RammsCurb
