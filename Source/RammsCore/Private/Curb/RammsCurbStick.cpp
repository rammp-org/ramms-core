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
} // namespace RammsCurb
