// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsMotionProfile.h"

namespace RammsMotion
{
	double StepToward(double& Setpoint, double& Rate, double Target, double MaxRate, double MaxAccel, double Dt)
	{
		if (!(Dt > 0.0) || !FMath::IsFinite(Target))
		{
			return Setpoint;
		}
		const double D = Target - Setpoint;
		if (MaxRate <= 0.0 && MaxAccel <= 0.0)
		{
			Setpoint = Target;
			Rate = 0.0;
			return Setpoint;
		}
		if (MaxAccel <= 0.0)
		{
			// A pure rate limit: the whole of the way when it fits in the step.
			const double Most = MaxRate * Dt;
			const double Step = FMath::Clamp(D, -Most, Most);
			Rate = Step / Dt;
			Setpoint += Step;
			return Setpoint;
		}
		// The fastest rate this step may end at and still stop at the target:
		// the setpoint moves by the mean of the old and new rates (midpoint), and
		// what is left after that must be the new rate's stopping distance,
		// v^2 / 2a -- the positive root of v^2 / 2a + v dt / 2 = Dist - Rate dt / 2.
		// Planned this way the rate reaches 0 as the setpoint reaches the
		// target (a plan from the distance alone lands a step late, still
		// moving, and the stop then breaks the acceleration limit).
		const double Sign = D >= 0.0 ? 1.0 : -1.0;
		const double Dist = FMath::Abs(D);
		const double Left = Dist - 0.5 * Rate * Sign * Dt;
		const double Half = 0.5 * MaxAccel * Dt;
		double		 Want = -Half + FMath::Sqrt(Half * Half + 2.0 * MaxAccel * FMath::Max(0.0, Left));
		if (MaxRate > 0.0)
		{
			Want = FMath::Min(Want, MaxRate);
		}
		Want *= Sign;
		const double DRate = MaxAccel * Dt;
		const double NewRate = FMath::Clamp(Want, Rate - DRate, Rate + DRate);
		double		 Step = 0.5 * (Rate + NewRate) * Dt;
		Rate = NewRate;
		if ((Sign > 0.0 && Step >= D) || (Sign < 0.0 && Step <= D))
		{
			Step = D; // the last step lands on the target
			Rate = 0.0;
		}
		Setpoint += Step;
		if (FMath::Abs(Target - Setpoint) < 1e-12)
		{
			Setpoint = Target;
			Rate = 0.0;
		}
		return Setpoint;
	}
} // namespace RammsMotion
