// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbStick.h"

#include "RammsCurbNegotiationTypes.h"

namespace RammsCurb
{
	const TCHAR* StickIntentName(EStickIntent Intent)
	{
		switch (Intent)
		{
			case EStickIntent::Forward:
				return TEXT("forward");
			case EStickIntent::Sideways:
				return TEXT("sideways");
			case EStickIntent::Back:
				return TEXT("back");
			default:
				return TEXT("let go");
		}
	}

	void FStickGate::Reset()
	{
		Intent = EStickIntent::LetGo;
		Scale = 0.0;
		OutSeconds = 0.0;
		Seconds = 0.0;
	}

	EStickIntent FStickGate::Step(const FRammsCurbStick& Settings, double Forward, double Turn, double Strafe, double Dt)
	{
		Dt = FMath::Max(0.0, Dt);
		const double Release = FMath::Clamp(static_cast<double>(Settings.ReleaseFraction), 0.0, 1.0);
		const double Engage = FMath::Clamp(static_cast<double>(Settings.EngageFraction), Release, 1.0);
		const double Sector = FMath::Clamp(static_cast<double>(Settings.ForwardSectorDeg), 0.0, 89.0);
		const double Side = FMath::Sqrt(Turn * Turn + Strafe * Strafe);
		const double Size = FMath::Sqrt(Forward * Forward + Side * Side);
		// How far round from straight ahead the stick points (deg): 0 ahead, 180
		// back.
		const double Round = Size > 1e-6 ? FMath::RadiansToDegrees(FMath::Atan2(Side, Forward)) : 0.0;
		const bool	 bAhead = Round <= Sector + 1e-9;

		if (Intent == EStickIntent::Forward)
		{
			const bool bStill = bAhead && Forward >= Release;
			OutSeconds = bStill ? 0.0 : OutSeconds + Dt;
			if (bStill)
			{
				Scale = FMath::Clamp(Forward, 0.0, 1.0);
			}
			if (bStill || OutSeconds < Settings.ReleaseSeconds)
			{
				// Held forward still -- or wavering, on at the share it had.
				Seconds += Dt;
				return Intent;
			}
		}
		EStickIntent Now = EStickIntent::Forward;
		if (bAhead && Forward >= Engage)
		{
			Scale = FMath::Clamp(Forward, 0.0, 1.0);
		}
		else
		{
			Now = Size < Release || bAhead ? EStickIntent::LetGo : (Round >= 180.0 - Sector - 1e-9 ? EStickIntent::Back : EStickIntent::Sideways);
			Scale = 0.0;
		}
		OutSeconds = 0.0;
		Seconds = Now == Intent ? Seconds + Dt : Dt;
		Intent = Now;
		return Intent;
	}

	FStickDecision DecideStick(const FRammsCurbStick& Settings, const FStickGate& Gate, const FStickStage& Stage)
	{
		FStickDecision	   Out;
		const EStickIntent Intent = Gate.GetIntent();
		if (Intent == EStickIntent::Forward)
		{
			Out.Action = EStickAction::Go;
			Out.Scale = Gate.GetScale();
			return Out;
		}
		const bool bBack = Intent == EStickIntent::Back;
		// Nothing moved for the curb yet: the rider turning or backing away from
		// it -- once the stick has read so for ReleaseSeconds, as letting go is
		// read, so that a stick on its way forward that passes off the axis, or
		// a bump, does not cancel.
		if (!Stage.bLegsMoved && (bBack || Intent == EStickIntent::Sideways))
		{
			if (Gate.GetSeconds() >= Settings.ReleaseSeconds)
			{
				Out.Action = EStickAction::Cancel;
				Out.Why = bBack ? TEXT("the rider pulled the stick back before any leg moved")
								: TEXT("the rider pushed the stick to the side before any leg moved");
				return Out;
			}
			Out.Why = bBack ? TEXT("Paused: pulled back -- held so, it cancels the crossing")
							: TEXT("Paused: pushed to the side -- held so, it cancels the crossing");
			return Out;
		}
		// Levelled in with no wheel lifted: held back, at rest, back out -- the
		// drive mode takes the legs back to its stance.
		if (bBack && !Stage.bCrossing)
		{
			const bool bReady = Stage.bAtRest && Stage.bLevelled;
			if (bReady && Gate.GetBackSeconds() >= Settings.CancelHoldSeconds)
			{
				Out.Action = EStickAction::Cancel;
				Out.Why = TEXT("the rider held the stick back with no wheel lifted: the legs back to the drive stance");
				return Out;
			}
			Out.Why = bReady ? FString::Printf(TEXT("Paused: hold the stick back %.1f s more to cancel"),
								   FMath::Max(0.0, Settings.CancelHoldSeconds - Gate.GetBackSeconds()))
							 : FString(TEXT("Paused: held back -- it cancels once the robot is at rest, levelled in"));
			return Out;
		}
		if (bBack)
		{
			Out.Why = TEXT("Paused: a wheel has begun to cross, so it cannot back out -- push forward to finish");
		}
		else if (Intent == EStickIntent::Sideways)
		{
			Out.Why = TEXT("Paused: steering is locked while crossing -- push forward to go on");
		}
		else
		{
			Out.Why = TEXT("Paused: push the stick forward to go on");
		}
		return Out;
	}

	double StepHoldSeconds(bool bHolding, bool bPaused, double Seconds, double Dt)
	{
		return bHolding ? Seconds + Dt : (bPaused ? Seconds : 0.0);
	}
} // namespace RammsCurb
