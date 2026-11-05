// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsControlTypes.h"

struct FRammsCurbAssist;
struct FRammsCurbStick;

/**
 * Curb assist (FRammsCurbAssist), shared by the curb controllers (RAMMP Gen1's and Gen2's): the curb.assist switch
 * they put on the control surface, and when, with it on, a controller looks for a curb ahead to cross. Plain C++, so
 * that it can be tested on made-up input. Not reflected, not exported.
 */
namespace RammsCurb
{
	/** curb.assist: the Off/On switch. */
	FName AssistControlId();

	/** The switch as a control (group Curb, an Enum Off/On) showing bOn. */
	FRammsControlAxis MakeAssistControl(bool bOn, int32 Order);

	/**
	 * When curb assist looks ahead, read once a tick while another drive mode has the robot: with the switch on and
	 * the rider's stick forward past Stick.EngageFraction, every Assist.ScanIntervalSeconds. A crossing it started
	 * that did not finish (stopped, cancelled, refused) leaves it waiting until the stick has been let go (under
	 * Stick.ReleaseFraction), so that it does not start another at the same curb while the rider still holds forward;
	 * one that finished does not (the curb is behind the robot).
	 */
	class FAssistWatch
	{
	public:
		void Reset();

		/** One tick of Dt (s), Forward the rider's drive.forward (-1..1): true to look ahead now. */
		bool Step(const FRammsCurbAssist& Assist, const FRammsCurbStick& Stick, double Forward, double Dt);

		/** A crossing it started is over: bFinished (Done), or not. */
		void CrossingEnded(bool bFinished);

		/** Waiting for the stick to be let go before it looks again. */
		bool IsWaitingForLetGo() const { return !bArmed; }

	private:
		bool   bArmed = true;
		double ScanLeft = 0.0;
	};
} // namespace RammsCurb
