// Tests for RammsMotion::StepToward, the easing the robot base puts every
// profiled position command through (FRammsMotorSpec::MaxVelocity /
// MaxAcceleration): a setpoint that never moves faster than the actuator it
// stands for, never changes its rate faster, and still arrives.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "RammsMotionProfile.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	/** Run the profile to Target from rest at 0 and report the worst rate and
	 *  acceleration seen and the time it took (seconds; -1 never arrived). */
	struct FRun
	{
		double MaxRate = 0.0;
		double MaxAccel = 0.0;
		double Seconds = -1.0;
		double Final = 0.0;
		bool   bOvershot = false;
	};

	FRun Run(double Target, double MaxRate, double MaxAccel, double Dt, double Limit = 30.0)
	{
		FRun   Out;
		double X = 0.0, V = 0.0, PrevV = 0.0, PrevX = 0.0;
		for (double T = 0.0; T < Limit; T += Dt)
		{
			RammsMotion::StepToward(X, V, Target, MaxRate, MaxAccel, Dt);
			Out.MaxRate = FMath::Max(Out.MaxRate, FMath::Abs(X - PrevX) / Dt);
			Out.MaxAccel = FMath::Max(Out.MaxAccel, FMath::Abs(V - PrevV) / Dt);
			Out.bOvershot = Out.bOvershot || (Target >= 0.0 ? X > Target + 1e-9 : X < Target - 1e-9);
			PrevV = V;
			PrevX = X;
			if (FMath::IsNearlyEqual(X, Target, 1e-9) && Out.Seconds < 0.0)
			{
				Out.Seconds = T + Dt;
			}
		}
		Out.Final = X;
		return Out;
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsMotionProfileRateLimit, "Ramms.Motion.Profile.RateLimit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRammsMotionProfileRateLimit::RunTest(const FString& Parameters)
{
	// A rate limit alone: 3.14 rad/s to a 1.57 rad target takes 0.5 s, never
	// faster than the limit, lands exactly.
	const FRun R = Run(1.57, 3.14, 0.0, 0.01);
	TestTrue(TEXT("arrives"), R.Seconds > 0.0);
	TestTrue(TEXT("in about half a second"), FMath::Abs(R.Seconds - 0.5) < 0.02);
	TestTrue(TEXT("never over the rate"), R.MaxRate <= 3.14 + 1e-9);
	TestFalse(TEXT("never past the target"), R.bOvershot);
	TestEqual(TEXT("lands on the target"), R.Final, 1.57, 1e-9);
	// The other way too.
	const FRun L = Run(-1.0, 2.0, 0.0, 0.02);
	TestTrue(TEXT("arrives (negative)"), L.Seconds > 0.0 && FMath::Abs(L.Seconds - 0.5) < 0.03);
	TestEqual(TEXT("lands on a negative target"), L.Final, -1.0, 1e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsMotionProfileTrapezoid, "Ramms.Motion.Profile.Trapezoid",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRammsMotionProfileTrapezoid::RunTest(const FString& Parameters)
{
	// Rate and acceleration: a trapezoid. 3.14 rad/s at 12 rad/s^2 over 2 rad:
	// ramps of 0.26 s, a cruise, 2 / 3.14 + 3.14 / 12 = 0.90 s in all.
	const FRun R = Run(2.0, 3.14, 12.0, 0.005);
	TestTrue(TEXT("arrives"), R.Seconds > 0.0);
	TestTrue(TEXT("in the trapezoid's time"), FMath::Abs(R.Seconds - 0.90) < 0.03);
	TestTrue(TEXT("never over the rate"), R.MaxRate <= 3.14 + 1e-6);
	TestTrue(TEXT("never over the acceleration"), R.MaxAccel <= 12.0 + 1e-6);
	TestFalse(TEXT("never past the target"), R.bOvershot);
	TestEqual(TEXT("lands on the target"), R.Final, 2.0, 1e-9);
	// Too short to cruise: a triangle, sqrt(d / a) each way = 2 * 0.0913 s.
	const FRun T = Run(0.1, 3.14, 12.0, 0.002);
	TestTrue(TEXT("a short move arrives"), T.Seconds > 0.0 && FMath::Abs(T.Seconds - 0.183) < 0.02);
	TestTrue(TEXT("a short move never over the acceleration"), T.MaxAccel <= 12.0 + 1e-6);
	TestFalse(TEXT("a short move never past the target"), T.bOvershot);
	// Coarse ticks (the editor's 60 Hz) still land without overshoot.
	const FRun C = Run(-1.5, 3.14, 12.0, 1.0 / 60.0);
	TestFalse(TEXT("coarse ticks never past the target"), C.bOvershot);
	TestEqual(TEXT("coarse ticks land"), C.Final, -1.5, 1e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsMotionProfileRetarget, "Ramms.Motion.Profile.Retarget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRammsMotionProfileRetarget::RunTest(const FString& Parameters)
{
	// Turned round mid-move, the rate changes no faster than the limit and the
	// setpoint comes back to the new target without a step.
	double		 X = 0.0, V = 0.0, PrevV = 0.0, PrevX = 0.0, MaxAccel = 0.0, MaxRate = 0.0;
	const double Dt = 0.005;
	double		 Target = 2.0;
	for (int32 i = 0; i < 400; ++i)
	{
		if (i == 60)
		{
			Target = -0.5;
		}
		RammsMotion::StepToward(X, V, Target, 3.14, 12.0, Dt);
		MaxAccel = FMath::Max(MaxAccel, FMath::Abs(V - PrevV) / Dt);
		MaxRate = FMath::Max(MaxRate, FMath::Abs(X - PrevX) / Dt);
		PrevV = V;
		PrevX = X;
	}
	TestTrue(TEXT("never over the acceleration"), MaxAccel <= 12.0 + 1e-6);
	TestTrue(TEXT("never over the rate"), MaxRate <= 3.14 + 1e-6);
	TestEqual(TEXT("settles on the new target"), X, -0.5, 1e-9);
	// No limits: the setpoint is the target.
	double Y = 1.0, W = 0.0;
	RammsMotion::StepToward(Y, W, 5.0, 0.0, 0.0, Dt);
	TestEqual(TEXT("unlimited jumps"), Y, 5.0, 1e-12);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
