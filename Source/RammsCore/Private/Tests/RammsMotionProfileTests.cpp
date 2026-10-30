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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsMotionProfileMovingTarget, "Ramms.Motion.Profile.MovingTarget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRammsMotionProfileMovingTarget::RunTest(const FString& Parameters)
{
	const double Dt = 0.001;
	// A still target: exactly StepToward.
	double A = 0.0, Va = 0.0, B = 0.0, Vb = 0.0;
	for (int32 i = 0; i < 500; ++i)
	{
		RammsMotion::StepToward(A, Va, 0.7, 3.14, 12.0, Dt);
		RammsMotion::StepTowardMoving(B, Vb, 0.7, 0.0, 3.14, 12.0, Dt);
	}
	TestEqual(TEXT("a still target moves it as StepToward does"), B, A, 1e-12);

	// A target moving at 1/s from 0.2 ahead: caught, then followed with no lag,
	// never over the rate or the acceleration.
	double X = 0.0, V = 0.0, PrevV = 0.0, WorstRate = 0.0, WorstAccel = 0.0;
	for (int32 i = 0; i < 2000; ++i)
	{
		const double T = i * Dt;
		RammsMotion::StepTowardMoving(X, V, 0.2 + T, 1.0, 3.14, 12.0, Dt);
		WorstRate = FMath::Max(WorstRate, FMath::Abs(V));
		WorstAccel = FMath::Max(WorstAccel, FMath::Abs(V - PrevV) / Dt);
		PrevV = V;
	}
	TestEqual(TEXT("follows a moving target with no lag"), X, 0.2 + 2000 * Dt, 1e-6);
	TestEqual(TEXT("at its rate"), V, 1.0, 1e-6);
	TestTrue(TEXT("never over the rate (moving)"), WorstRate <= 3.14 + 1e-9);
	TestTrue(TEXT("never over the acceleration (moving)"), WorstAccel <= 12.0 + 1e-6);

	// A target faster than the servo: chased at the servo's rate.
	double Y = 0.0, W = 0.0;
	for (int32 i = 0; i < 1000; ++i)
	{
		RammsMotion::StepTowardMoving(Y, W, 5.0 * i * Dt, 5.0, 3.14, 0.0, Dt);
	}
	TestEqual(TEXT("chases a faster target at the rate limit"), W, 3.14, 1e-9);
	return true;
}

namespace
{
	/** A ramp at Speed from 0 to End, commanded every Frame seconds of sim time
	 *  (uneven by +-Jitter: 1 - j, 1, 1 + j in turn) and stepped every 1 ms,
	 *  through FFollower or (bStaircase) through StepToward on the last command,
	 *  as the base once did each frame. */
	struct FStream
	{
		double Ripple = 0.0;	// worst |rate - Speed| while the ramp runs, past its start
		double LagError = 0.0;	// worst |setpoint - the ramp 1.3 frames ago| over the same span
		double Overshoot = 0.0; // furthest past End
		double WorstRate = 0.0;
		double WorstAccel = 0.0;
		double Final = 0.0;
	};

	FStream Stream(double Speed, double End, double Frame, bool bStaircase, double Jitter = 0.0)
	{
		const double		   Dt = 0.001, MaxRate = 3.77, MaxAccel = 40.0;
		FStream				   Out;
		RammsMotion::FFollower F;
		F.Reset(0.0);
		double Last = 0.0, Setpoint = 0.0, Rate = 0.0, PrevRate = 0.0, NextFrame = 0.0;
		int32  FrameNo = 0;
		for (int32 i = 0; i * Dt < End / Speed + 2.0; ++i)
		{
			const double T = i * Dt;
			if (T >= NextFrame - 1e-9)
			{
				Last = FMath::Min(End, Speed * T);
				F.Command(Last, T);
				++FrameNo;
				NextFrame += Frame * (1.0 + Jitter * ((FrameNo % 3) - 1));
			}
			if (bStaircase)
			{
				RammsMotion::StepToward(Setpoint, Rate, Last, MaxRate, MaxAccel, Dt);
			}
			else
			{
				Setpoint = F.Step(T, Dt, MaxRate, MaxAccel);
				Rate = F.Rate;
			}
			if (T > 0.4 && T < End / Speed - 0.15)
			{
				Out.Ripple = FMath::Max(Out.Ripple, FMath::Abs(Rate - Speed));
				Out.LagError = FMath::Max(Out.LagError, FMath::Abs(Setpoint - Speed * (T + Dt - RammsMotion::FollowDelayIntervals * Frame)));
			}
			Out.Overshoot = FMath::Max(Out.Overshoot, Setpoint - End);
			Out.WorstRate = FMath::Max(Out.WorstRate, FMath::Abs(Rate));
			Out.WorstAccel = FMath::Max(Out.WorstAccel, FMath::Abs(Rate - PrevRate) / Dt);
			PrevRate = Rate;
		}
		Out.Final = Setpoint;
		return Out;
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsMotionProfileFollower, "Ramms.Motion.Profile.Follower",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRammsMotionProfileFollower::RunTest(const FString& Parameters)
{
	// A crank commanded along a 1 rad/s ramp at 16 Hz (the curb runner's frame
	// rate) and stepped at 1 kHz: the follower moves at the ramp's rate, 1.3
	// command periods behind; easing to each command as it came moved in a
	// staircase, stopping and starting every frame.
	const FStream Smooth = Stream(1.0, 1.0, 0.062, false);
	const FStream Stairs = Stream(1.0, 1.0, 0.062, true);
	TestTrue(FString::Printf(TEXT("a smooth rate (within %.3f of the ramp's)"), Smooth.Ripple), Smooth.Ripple < 0.01);
	TestTrue(FString::Printf(TEXT("the staircase it replaces stops and starts (%.3f)"), Stairs.Ripple), Stairs.Ripple > 0.5);
	TestTrue(FString::Printf(TEXT("1.3 command periods behind (%.4f)"), Smooth.LagError), Smooth.LagError < 0.002);
	TestTrue(FString::Printf(TEXT("hardly past where the ramp stopped (%.4f; v^2/2a = 0.0125)"), Smooth.Overshoot),
		Smooth.Overshoot < 0.0125 + 1e-3);
	TestEqual(TEXT("settles on the last command"), Smooth.Final, 1.0, 1e-9);
	TestTrue(TEXT("never over the rate"), Smooth.WorstRate <= 3.77 + 1e-9);
	TestTrue(TEXT("never over the acceleration"), Smooth.WorstAccel <= 40.0 + 1e-6);

	// Frames of uneven length (62 ms +-25%): the line's slopes come from the
	// commands' stamps, so the motion stays all but smooth.
	const FStream Uneven = Stream(1.0, 1.0, 0.062, false, 0.25);
	TestTrue(FString::Printf(TEXT("uneven frames, still smooth (%.3f)"), Uneven.Ripple), Uneven.Ripple < 0.25);
	TestTrue(TEXT("uneven frames, never over the acceleration"), Uneven.WorstAccel <= 40.0 + 1e-6);
	TestEqual(TEXT("uneven frames settle"), Uneven.Final, 1.0, 1e-9);

	// Stopped half way: settles on the last command.
	const FStream Half = Stream(1.0, 0.5, 0.062, false);
	TestEqual(TEXT("stopped half way, settles there"), Half.Final, 0.5, 1e-9);
	TestTrue(TEXT("stopped half way, hardly past"), Half.Overshoot < 0.0125 + 1e-3);

	// A jump faster than the servo is chased at its rate, never passed.
	RammsMotion::FFollower J;
	J.Reset(0.0);
	double Worst = 0.0;
	for (int32 i = 0; i < 3000; ++i)
	{
		const double T = i * 0.001;
		if (i % 62 == 0)
		{
			J.Command(T < 0.5 ? 0.0 : 1.0, T);
		}
		Worst = FMath::Max(Worst, J.Step(T, 0.001, 3.77, 40.0));
	}
	TestTrue(TEXT("a jump is never passed"), Worst <= 1.0 + 1e-9);
	TestEqual(TEXT("a jump lands"), J.Setpoint, 1.0, 1e-9);

	// A command after a silence starts a new line and lands.
	RammsMotion::FFollower Q;
	Q.Reset(0.0);
	for (int32 i = 0; i < 4000; ++i)
	{
		const double T = i * 0.001;
		if (i == 0 || i == 62 || i == 124 || i == 2000)
		{
			Q.Command(i < 2000 ? 0.0 : 0.3, T);
		}
		Q.Step(T, 0.001, 3.77, 40.0);
	}
	TestEqual(TEXT("a command after a silence lands"), Q.Setpoint, 0.3, 1e-9);

	// No command yet: it holds where it was reset.
	RammsMotion::FFollower H;
	H.Reset(0.4);
	TestEqual(TEXT("holds with no command"), H.Step(0.0, 0.001, 3.77, 40.0), 0.4, 1e-12);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
