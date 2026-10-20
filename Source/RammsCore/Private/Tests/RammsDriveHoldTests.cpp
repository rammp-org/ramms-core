// Tests for holding a robot still with no command (Private/RammsDriveHandOver):
// a speed-looped wheel asked back to where it stopped, a torque-driven wheel
// held by a loop on its angle -- on a made-up wheel carrying a made-up share of
// a robot on a 6 deg slope, its loop closed once a game tick -- and how long a
// stance change must take for the base to turn within a rate and an
// acceleration; and two held wheels of one robot kept from turning it back.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "RammsDriveHandOver.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	/** A made-up wheel on a slope: Mass kg of the robot on it, RadiusCm, the
	 *  slope pulling it downhill (negative angle); a motor of MaxTorque. The
	 *  physics steps every millisecond, the hold's loop once a tick of
	 *  TickSeconds with the state measured at the tick, its torque held over
	 *  the tick. */
	struct FToyWheel
	{
		double MassKg = 60.0;
		double RadiusCm = 12.0;
		double SlopeDeg = 6.0;
		double MaxTorque = 15.0;
		double Angle = 0.0;
		double Rate = 0.0;

		double Inertia() const { return MassKg * FMath::Square(RadiusCm / 100.0); }
		double PullTorque() const { return MassKg * 9.81 * FMath::Sin(FMath::DegreesToRadians(SlopeDeg)) * RadiusCm / 100.0; }
		double GroundCm() const { return Angle * RadiusCm; }
		double SpeedCmS() const { return Rate * RadiusCm; }

		/** Run Seconds with the hold's loop every TickSeconds; Sample(t) after
		 *  each tick. */
		template <typename F>
		void Run(RammsDrive::FTorqueHold& Hold, const RammsDrive::FTorqueHoldGains& G, double TickSeconds, double Seconds, F&& Sample)
		{
			constexpr double Dt = 0.001;
			const int32		 PerTick = FMath::Max(1, FMath::RoundToInt(TickSeconds / Dt));
			const int32		 Ticks = FMath::CeilToInt(Seconds / TickSeconds);
			for (int32 k = 0; k < Ticks; ++k)
			{
				const double Torque = FMath::Clamp(Hold.Step(Angle, Rate, TickSeconds, G), -MaxTorque, MaxTorque);
				for (int32 s = 0; s < PerTick; ++s)
				{
					Rate += (Torque - PullTorque()) / Inertia() * Dt;
					Angle += Rate * Dt;
				}
				Sample((k + 1) * TickSeconds);
			}
		}
	};
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHoldWheelHold,
	"Ramms.Drive.Hold.WheelHold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHoldWheelHold::RunTest(const FString& Parameters)
{
	// A speed-looped wheel is asked back towards the angle it was held at, in
	// proportion, never faster than MaxRate; not held, nothing.
	RammsDrive::FWheelHold Hold;
	TestEqual(TEXT("not held: no rate"), Hold.Rate(3.0), 0.0);
	Hold.Latch(2.0);
	TestNearlyEqual(TEXT("on its angle: no rate"), Hold.Rate(2.0), 0.0, 1e-12);
	TestNearlyEqual(TEXT("0.1 rad past: back at Gain x 0.1"), Hold.Rate(2.1), -RammsDrive::FWheelHold::Gain * 0.1, 1e-12);
	TestNearlyEqual(TEXT("0.1 rad short: on at Gain x 0.1"), Hold.Rate(1.9), RammsDrive::FWheelHold::Gain * 0.1, 1e-12);
	TestNearlyEqual(TEXT("far off: no faster than MaxRate"), Hold.Rate(12.0), -RammsDrive::FWheelHold::MaxRate, 1e-12);
	TestNearlyEqual(TEXT("far off the other way: no faster than MaxRate"), Hold.Rate(-8.0), RammsDrive::FWheelHold::MaxRate, 1e-12);
	Hold.Release();
	TestEqual(TEXT("released: no rate"), Hold.Rate(2.5), 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHoldTorqueHoldOnSlope,
	"Ramms.Drive.Hold.TorqueHoldOnSlope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHoldTorqueHoldOnSlope::RunTest(const FString& Parameters)
{
	// Let go at rest on a 6 deg slope, the wheel is held where it was: it sags a
	// little before the integral takes the slope's pull, then comes back and
	// stays, at the controller's tick of 16 ms and of 62.5 ms.
	for (const double Tick : { 1.0 / 60.0, 0.0625 })
	{
		FToyWheel						   Wheel;
		const RammsDrive::FTorqueHoldGains G = RammsDrive::TorqueHoldGains(Wheel.Inertia(), Wheel.MaxTorque, Wheel.RadiusCm, 60.0);
		RammsDrive::FTorqueHold			   Hold;
		Hold.Begin(Wheel.Angle, Wheel.Rate);
		double MostSag = 0.0, LateSpeed = 0.0, LateFrom = 0.0, LateTo = 0.0;
		Wheel.Run(Hold, G, Tick, 4.0, [&](double T) {
			MostSag = FMath::Max(MostSag, -Wheel.GroundCm());
			if (T >= 2.0)
			{
				LateSpeed = FMath::Max(LateSpeed, FMath::Abs(Wheel.SpeedCmS()));
				LateFrom = T <= 2.0 + Tick ? Wheel.GroundCm() : LateFrom;
				LateTo = Wheel.GroundCm();
			}
		});
		const FString At = FString::Printf(TEXT("at a %.1f ms tick"), 1000.0 * Tick);
		TestTrue(FString::Printf(TEXT("%s: sagged at most %.2f cm (under 3)"), *At, MostSag), MostSag < 3.0);
		TestTrue(FString::Printf(TEXT("%s: from 2 s on no faster than %.3f cm/s (under 0.5)"), *At, LateSpeed), LateSpeed < 0.5);
		TestTrue(FString::Printf(TEXT("%s: held within %.3f cm of where it was let go after 4 s (under 0.3)"), *At, FMath::Abs(Wheel.GroundCm())),
			FMath::Abs(Wheel.GroundCm()) < 0.3);
		TestTrue(FString::Printf(TEXT("%s: the integral carries the pull (%.2f of %.2f N.m)"), *At, Hold.Integral, Wheel.PullTorque()),
			FMath::IsNearlyEqual(Hold.Integral, Wheel.PullTorque(), 0.05 * Wheel.PullTorque()));
	}

	// Let go rolling downhill at 30 cm/s: braked to rest -- its target at
	// 60 cm/s^2, twice that while the slope pulls the wheel on ahead of it,
	// the wheel following within the motor's torque -- in about half a
	// second, then held where the target stopped.
	{
		FToyWheel Wheel;
		Wheel.Rate = -30.0 / Wheel.RadiusCm;
		const RammsDrive::FTorqueHoldGains G = RammsDrive::TorqueHoldGains(Wheel.Inertia(), Wheel.MaxTorque, Wheel.RadiusCm, 60.0);
		RammsDrive::FTorqueHold			   Hold;
		Hold.Begin(Wheel.Angle, Wheel.Rate);
		const double Tick = 0.0625;
		double		 RestAt = -1.0, Overshoot = 0.0, LastSpeed = Wheel.SpeedCmS(), MostDecel = 0.0;
		const double TargetEndCm = -0.5 * 30.0 * 30.0 / 60.0; // where the target comes to rest
		Wheel.Run(Hold, G, Tick, 3.0, [&](double T) {
			const double Speed = Wheel.SpeedCmS();
			// Braking: the speed's fall over a tick (the slope's pull speeding it
			// up at first, before the integral takes it, is not braking).
			MostDecel = FMath::Max(MostDecel, (FMath::Abs(LastSpeed) - FMath::Abs(Speed)) / Tick);
			LastSpeed = Speed;
			if (RestAt < 0.0 && FMath::Abs(Speed) < 1.0 && Hold.IsBraked())
			{
				RestAt = T;
			}
			Overshoot = FMath::Max(Overshoot, TargetEndCm - Wheel.GroundCm());
		});
		TestTrue(FString::Printf(TEXT("rolling: at rest (under 1 cm/s) %.2f s after it was let go (within 2: the braking, then the slope's pull taken up)"), RestAt), RestAt > 0.0 && RestAt <= 2.0);
		TestTrue(FString::Printf(TEXT("rolling: braked at most %.1f cm/s^2 over a tick (the target's 60, 120 while pulled on; with the loop "
									  "catching up on the slope's pull: under 180)"),
					 MostDecel),
			MostDecel < 180.0);
		TestTrue(FString::Printf(TEXT("rolling: went at most %.2f cm past where a target braking at 60 alone stops (%.1f cm on; the slope's "
									  "pull before the integral takes it: under 1.5)"),
					 Overshoot, TargetEndCm),
			Overshoot < 1.5);
		const double StoppedCm = Hold.Target * Wheel.RadiusCm;
		TestTrue(FString::Printf(TEXT("rolling: held within %.2f cm of where its target stopped (%.2f cm on) after 3 s (under 0.3)"),
					 FMath::Abs(Wheel.GroundCm() - StoppedCm), StoppedCm),
			FMath::Abs(Wheel.GroundCm() - StoppedCm) < 0.3);
	}

	// Let go rolling uphill at 35 cm/s, the target braking at only 30 cm/s^2:
	// the slope slows the wheel far faster than the target, which follows it
	// rather than running ahead -- the hold never drives it on uphill, never
	// lets go of it as a wheel in the air (it would roll back down freely),
	// and holds it where it stopped, rolled back no more than a little.
	{
		FToyWheel Wheel;
		Wheel.Rate = 35.0 / Wheel.RadiusCm;
		const RammsDrive::FTorqueHoldGains G = RammsDrive::TorqueHoldGains(Wheel.Inertia(), Wheel.MaxTorque, Wheel.RadiusCm, 30.0);
		RammsDrive::FTorqueHold			   Hold;
		Hold.Begin(Wheel.Angle, Wheel.Rate);
		double Furthest = 0.0, MostDrive = 0.0;
		int32  FreeTicks = 0;
		double LastSpeed = Wheel.SpeedCmS();
		Wheel.Run(Hold, G, 0.0625, 4.0, [&](double) {
			Furthest = FMath::Max(Furthest, Wheel.GroundCm());
			FreeTicks += Hold.bFree ? 1 : 0;
			// Driving it on: a forward torque while it still rolls forward and
			// slows by less than the slope alone would slow it.
			if (Wheel.Rate > 0.0 && Hold.LastTorque > Wheel.PullTorque())
			{
				MostDrive = FMath::Max(MostDrive, Hold.LastTorque - Wheel.PullTorque());
			}
			LastSpeed = Wheel.SpeedCmS();
		});
		TestEqual(TEXT("uphill: never let go of"), FreeTicks, 0);
		TestTrue(FString::Printf(TEXT("uphill: never driven on past what holds the slope (%.2f N.m over)"), MostDrive), MostDrive < 0.25 * Wheel.PullTorque());
		TestTrue(FString::Printf(TEXT("uphill: held within %.2f cm of the furthest it went, %.2f cm on (under 3, as a wheel let go at rest sags)"), Furthest - Wheel.GroundCm(), Furthest),
			Furthest - Wheel.GroundCm() < 3.0);
		TestTrue(FString::Printf(TEXT("uphill: at rest at the end (%.3f cm/s)"), LastSpeed), FMath::Abs(LastSpeed) < 0.5);
	}

	// The torque never past the motor's, and the integral not wound up against
	// a pull the motor cannot hold (here twice what it gives): let go there,
	// the wheel rolls, the hold at its limit.
	{
		FToyWheel Wheel;
		Wheel.MaxTorque = 0.5 * Wheel.PullTorque();
		const RammsDrive::FTorqueHoldGains G = RammsDrive::TorqueHoldGains(Wheel.Inertia(), Wheel.MaxTorque, Wheel.RadiusCm, 60.0);
		RammsDrive::FTorqueHold			   Hold;
		Hold.Begin(Wheel.Angle, Wheel.Rate);
		double MostTorque = 0.0;
		Wheel.Run(Hold, G, 0.0625, 1.0, [&](double) { MostTorque = FMath::Max(MostTorque, FMath::Abs(Hold.LastTorque)); });
		TestTrue(FString::Printf(TEXT("too steep: the torque at most the motor's (%.2f of %.2f)"), MostTorque, Wheel.MaxTorque),
			MostTorque <= Wheel.MaxTorque + 1e-9);
		TestTrue(FString::Printf(TEXT("too steep: the integral no more than the motor's torque (%.2f)"), Hold.Integral),
			FMath::Abs(Hold.Integral) <= Wheel.MaxTorque + 1e-9);
	}
	// A wheel that carries nothing (in the air: a hundredth of the inertia the
	// gains are set for, no slope) turning when the hold begins: the loop,
	// closed once a 62.5 ms tick on gains for a loaded wheel, would spin it up
	// further every tick; the hold lets go of it (no torque) once it has found
	// it turning FreeSpinRadS off its target FreeSpinTicks ticks running, and
	// drives it no faster.
	{
		FToyWheel Light;
		Light.MassKg = 0.5;
		Light.SlopeDeg = 0.0;
		Light.Rate = -0.4;
		const RammsDrive::FTorqueHoldGains G = RammsDrive::TorqueHoldGains(FToyWheel().Inertia(), Light.MaxTorque, Light.RadiusCm, 60.0);
		RammsDrive::FTorqueHold			   Hold;
		Hold.Begin(Light.Angle, Light.Rate);
		int32  Tick = 0, OverAt = -1, FreeAt = -1, TorqueAfterFree = 0;
		double RateAtFree = 0.0, MostAfterFree = 0.0;
		Light.Run(Hold, G, 0.0625, 2.0, [&](double) {
			++Tick;
			if (FreeAt >= 0)
			{
				MostAfterFree = FMath::Max(MostAfterFree, FMath::Abs(Light.Rate));
				TorqueAfterFree += Hold.LastTorque != 0.0 ? 1 : 0;
			}
			if (OverAt < 0 && FMath::Abs(Light.Rate) > RammsDrive::FTorqueHold::FreeSpinRadS)
			{
				OverAt = Tick;
			}
			if (FreeAt < 0 && Hold.bFree)
			{
				FreeAt = Tick;
				RateAtFree = FMath::Abs(Light.Rate);
			}
		});
		TestTrue(FString::Printf(TEXT("a wheel in the air: let go of (tick %d) within FreeSpinTicks of its turning past FreeSpinRadS (tick %d)"), FreeAt, OverAt),
			OverAt > 0 && FreeAt > 0 && FreeAt <= OverAt + RammsDrive::FTorqueHold::FreeSpinTicks);
		TestEqual(TEXT("...no torque after that (it never slows here: no friction)"), TorqueAfterFree, 0);
		TestTrue(FString::Printf(TEXT("...driven no faster after it was let go of (%.2f rad/s, %.2f then)"), MostAfterFree, RateAtFree),
			MostAfterFree <= RateAtFree + 1e-6);
	}
	// Slowed to rest, it is held again where it is.
	{
		RammsDrive::FTorqueHold Hold;
		Hold.Begin(0.0, 0.0);
		const RammsDrive::FTorqueHoldGains G = RammsDrive::TorqueHoldGains(1.0, 10.0, 10.0, 60.0);
		Hold.Step(-0.2, 0.0, 0.05, G);
		TestTrue(TEXT("short of its target: driven on"), Hold.LastTorque > 0.0 && !Hold.bFree);
		// Spun by its own loop: each tick far off the way the last torque drove
		// it, the torque swinging back each time.
		Hold.Step(0.0, 10.0, 0.05, G);
		TestTrue(TEXT("one tick 10 rad/s off the way it was driven: still held"), !Hold.bFree && Hold.LastTorque < 0.0);
		Hold.Step(0.0, -12.0, 0.05, G);
		TestTrue(TEXT("two ticks: still held"), !Hold.bFree && Hold.LastTorque > 0.0);
		Hold.Step(0.0, 14.0, 0.05, G);
		TestTrue(TEXT("three ticks running: let go of"), Hold.bFree && Hold.LastTorque == 0.0);
		Hold.Step(0.3, 2.0, 0.05, G);
		TestTrue(TEXT("still turning 2 rad/s: still let go of"), Hold.bFree);
		Hold.Step(0.4, 0.1, 0.05, G);
		TestTrue(TEXT("slow again: held again, where it is"), !Hold.bFree && FMath::IsNearlyEqual(Hold.Target, 0.4, 1e-12));
	}
	// Run off its target against the torque (a slope's pull too much for it
	// for a moment): held on, braking at full torque -- not let go of.
	{
		RammsDrive::FTorqueHold Hold;
		Hold.Begin(0.0, 0.0);
		const RammsDrive::FTorqueHoldGains G = RammsDrive::TorqueHoldGains(1.0, 10.0, 10.0, 60.0);
		Hold.Step(0.2, 0.0, 0.05, G);
		TestTrue(TEXT("past its target: braked"), Hold.LastTorque < 0.0);
		Hold.Step(0.5, 5.0, 0.05, G);
		TestTrue(FString::Printf(TEXT("running 5 rad/s on against the torque: held on, braking (%.2f N.m)"), Hold.LastTorque), !Hold.bFree && Hold.LastTorque < 0.0);
	}
	// Not active: nothing.
	RammsDrive::FTorqueHold Idle;
	TestEqual(TEXT("not begun: no torque"), Idle.Step(1.0, 1.0, 0.05, RammsDrive::TorqueHoldGains(1.0, 10.0, 10.0, 60.0)), 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHoldStanceTurnSeconds,
	"Ramms.Drive.Hold.StanceTurnSeconds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHoldStanceTurnSeconds::RunTest(const FString& Parameters)
{
	// A turn of the base taken over StanceTurnSeconds on the selector's S-curve
	// (SmoothStep) turns no faster than the rate and changes its rate no faster
	// than the acceleration, one of them reached; nothing to turn takes no time.
	const double TurnRate = 3.0, TurnAccel = 4.0;
	TestEqual(TEXT("no turn: no time"), RammsDrive::StanceTurnSeconds(0.0, TurnRate, TurnAccel), 0.0);
	TestEqual(TEXT("no limits: no time"), RammsDrive::StanceTurnSeconds(5.0, 0.0, 0.0), 0.0);
	for (const double Turn : { 0.5, 1.0, 3.5, 6.0, 10.0 })
	{
		const double T = RammsDrive::StanceTurnSeconds(Turn, TurnRate, TurnAccel);
		const int32	 N = 4000;
		const double D = T / N;
		double		 MostRate = 0.0, MostAccel = 0.0;
		for (int32 k = 1; k < N; ++k)
		{
			const double A = Turn * RammsDrive::SmoothStep((k - 1) * D / T);
			const double B = Turn * RammsDrive::SmoothStep(k * D / T);
			const double C = Turn * RammsDrive::SmoothStep((k + 1) * D / T);
			MostRate = FMath::Max(MostRate, FMath::Abs(B - A) / D);
			MostAccel = FMath::Max(MostAccel, FMath::Abs(C - 2.0 * B + A) / (D * D));
		}
		TestTrue(FString::Printf(TEXT("%.1f deg over %.3f s: at most %.4f deg/s (%.1f)"), Turn, T, MostRate, TurnRate), MostRate <= TurnRate * (1.0 + 1e-3));
		TestTrue(FString::Printf(TEXT("%.1f deg over %.3f s: its rate changing at most %.4f deg/s^2 (%.1f)"), Turn, T, MostAccel, TurnAccel),
			MostAccel <= TurnAccel * (1.0 + 1e-2));
		TestTrue(FString::Printf(TEXT("%.1f deg: one limit reached (%.4f / %.4f)"), Turn, MostRate / TurnRate, MostAccel / TurnAccel),
			MostRate >= TurnRate * 0.99 || MostAccel >= TurnAccel * 0.98);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHoldTurnErrorBounded,
	"Ramms.Drive.Hold.TurnErrorBounded",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHoldTurnErrorBounded::RunTest(const FString& Parameters)
{
	// Two held wheels whose errors differ by more than the bound: their targets
	// are moved together, half the excess each -- the mean error, what holds
	// the robot along its way, unchanged -- until they differ by the bound;
	// within it, nothing moves; a wheel let go of, or not held, leaves both.
	RammsDrive::FTorqueHold L, R;
	L.Begin(1.0, 0.0);
	R.Begin(2.0, 0.0);
	// The left wheel ran 0.4 rad past its target, the right fell 0.1 short.
	const double AngleL = 1.4, AngleR = 1.9;
	const double MeanWas = 0.5 * ((L.Target - AngleL) + (R.Target - AngleR));
	RammsDrive::BoundTurnError(L, AngleL, R, AngleR, 0.05);
	const double Difference = (L.Target - AngleL) - (R.Target - AngleR);
	TestNearlyEqual(TEXT("the errors now differ by the bound"), FMath::Abs(Difference), 0.05, 1e-12);
	TestNearlyEqual(TEXT("their mean unchanged"), 0.5 * ((L.Target - AngleL) + (R.Target - AngleR)), MeanWas, 1e-12);
	const double TargetL = L.Target, TargetR = R.Target;
	RammsDrive::BoundTurnError(L, AngleL, R, AngleR, 0.05);
	TestNearlyEqual(TEXT("within the bound: the left target stays"), L.Target, TargetL, 1e-12);
	TestNearlyEqual(TEXT("...and the right"), R.Target, TargetR, 1e-12);
	RammsDrive::FTorqueHold Free = L, Held = R;
	Free.bFree = true;
	RammsDrive::BoundTurnError(Free, 3.0, Held, AngleR, 0.05);
	TestNearlyEqual(TEXT("one let go of: nothing moves"), Held.Target, TargetR, 1e-12);
	RammsDrive::FTorqueHold Idle;
	RammsDrive::BoundTurnError(Idle, 3.0, Held, AngleR, 0.05);
	TestNearlyEqual(TEXT("one not held: nothing moves"), Held.Target, TargetR, 1e-12);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHoldPairBrakesAsOne,
	"Ramms.Drive.Hold.PairBrakesAsOne",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHoldPairBrakesAsOne::RunTest(const FString& Parameters)
{
	// Two held wheels of one robot, braking from 3 rad/s: one wheel's reading
	// alone far below its target (a noisy tick) moves neither target; the
	// robot slowing faster than its targets brings both to it alike; the
	// robot running on ahead of them brakes both twice as hard alike. The
	// two targets' rates stay equal throughout.
	const RammsDrive::FTorqueHoldGains G = RammsDrive::TorqueHoldGains(1.0, 30.0, 10.0, 30.0);
	const double					   Dt = 0.0625;
	RammsDrive::FTorqueHold			   L, R;
	L.Begin(0.0, 3.0);
	R.Begin(0.0, 3.0);
	double TL = 0.0, TR = 0.0;
	// The left reads 0.7 rad/s under its target for a tick, the right on it.
	RammsDrive::StepTorqueHoldPair(L, 0.1, 2.3, R, 0.1, 3.0, Dt, G, G, 0.05, TL, TR);
	const double Braked = 3.0 - G.Decel * Dt;
	TestNearlyEqual(TEXT("one wheel's low reading: the left target brakes as usual"), L.TargetRate, Braked, 1e-9);
	TestNearlyEqual(TEXT("...and the right alike"), R.TargetRate, Braked, 1e-9);
	// Both slowed to 1 rad/s (uphill): both targets follow, to 1.5, alike.
	const double Before = L.TargetRate;
	RammsDrive::StepTorqueHoldPair(L, 0.3, 1.0, R, 0.3, 1.0, Dt, G, G, 0.05, TL, TR);
	TestTrue(FString::Printf(TEXT("the robot slowing faster: both targets brought to it (%.3f, %.3f, were %.3f)"), L.TargetRate, R.TargetRate, Before),
		L.TargetRate < 1.5 + 1e-9 && FMath::IsNearlyEqual(L.TargetRate, R.TargetRate, 1e-12));
	// Both running on at 3 rad/s over targets about 1.4 (downhill): braked at
	// twice the Decel, alike.
	const double Was = L.TargetRate;
	RammsDrive::StepTorqueHoldPair(L, 0.5, 3.0, R, 0.5, 3.0, Dt, G, G, 0.05, TL, TR);
	TestNearlyEqual(TEXT("the robot pulled on: both targets brake twice as hard"), L.TargetRate, Was - RammsDrive::FTorqueHold::PulledOnDecelScale * G.Decel * Dt, 1e-9);
	TestNearlyEqual(TEXT("...alike"), R.TargetRate, L.TargetRate, 1e-12);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHoldSeededOnSlope,
	"Ramms.Drive.Hold.SeededOnSlope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHoldSeededOnSlope::RunTest(const FString& Parameters)
{
	// GroundHoldTorque is the slope's pull on each wheel; a hold begun with it
	// as its integral (a hand-back that knows the ground, or a hold carried on
	// through a switch) takes the pull at once: at rest on a 6 deg slope it
	// barely sags, where one that finds the pull from nothing sags over a
	// centimetre first; braking a robot that runs uphill, it stops it and
	// holds it there without rolling it back.
	FToyWheel Probe;
	TestNearlyEqual(TEXT("one wheel's pull"), RammsDrive::GroundHoldTorque(Probe.MassKg, Probe.SlopeDeg, Probe.RadiusCm, 1), Probe.PullTorque(), 1e-9);
	TestNearlyEqual(TEXT("two wheels: half each"), RammsDrive::GroundHoldTorque(2.0 * Probe.MassKg, Probe.SlopeDeg, Probe.RadiusCm, 2), Probe.PullTorque(), 1e-9);
	TestNearlyEqual(TEXT("downhill: negative"), RammsDrive::GroundHoldTorque(Probe.MassKg, -Probe.SlopeDeg, Probe.RadiusCm, 1), -Probe.PullTorque(), 1e-9);
	TestEqual(TEXT("no mass: none"), RammsDrive::GroundHoldTorque(0.0, 6.0, 12.0, 2), 0.0);
	for (const double Tick : { 1.0 / 60.0, 0.0625 })
	{
		const FString At = FString::Printf(TEXT("at a %.1f ms tick"), 1000.0 * Tick);
		double		  Sag[2] = { 0.0, 0.0 };
		for (int32 Seeded = 0; Seeded < 2; ++Seeded)
		{
			FToyWheel						   Wheel;
			const RammsDrive::FTorqueHoldGains G = RammsDrive::TorqueHoldGains(Wheel.Inertia(), Wheel.MaxTorque, Wheel.RadiusCm, 30.0);
			RammsDrive::FTorqueHold			   Hold;
			Hold.Begin(Wheel.Angle, Wheel.Rate, Seeded ? RammsDrive::GroundHoldTorque(Wheel.MassKg, Wheel.SlopeDeg, Wheel.RadiusCm, 1) : 0.0);
			Wheel.Run(Hold, G, Tick, 3.0, [&](double) { Sag[Seeded] = FMath::Max(Sag[Seeded], -Wheel.GroundCm()); });
		}
		TestTrue(FString::Printf(TEXT("%s: begun with the pull, sagged %.3f cm (under 0.2)"), *At, Sag[1]), Sag[1] < 0.2);
		TestTrue(FString::Printf(TEXT("%s: begun from nothing, sagged %.2f cm first (over 1: what the seed saves)"), *At, Sag[0]), Sag[0] > 1.0);
	}
	// Running uphill at 25 cm/s when the hold begins: with the pull, it brakes
	// and holds as on the flat (what it rolls back is the loop's own, a tick's
	// worth); from nothing, at a 16.7 ms tick, it rolls back several cm/s as
	// the integral finds the pull -- what a hand-back on a slope did.
	const auto Uphill = [](double SlopeDeg, bool bSeeded, double Tick, double& OutBack, double& OutLeast) {
		FToyWheel Wheel;
		Wheel.SlopeDeg = SlopeDeg;
		Wheel.Rate = 25.0 / Wheel.RadiusCm;
		const RammsDrive::FTorqueHoldGains G = RammsDrive::TorqueHoldGains(Wheel.Inertia(), Wheel.MaxTorque, Wheel.RadiusCm, 30.0);
		RammsDrive::FTorqueHold			   Hold;
		Hold.Begin(Wheel.Angle, Wheel.Rate, bSeeded ? RammsDrive::GroundHoldTorque(Wheel.MassKg, Wheel.SlopeDeg, Wheel.RadiusCm, 1) : 0.0);
		double Furthest = 0.0;
		OutLeast = 0.0;
		Wheel.Run(Hold, G, Tick, 4.0, [&](double) {
			Furthest = FMath::Max(Furthest, Wheel.GroundCm());
			OutLeast = FMath::Min(OutLeast, Wheel.SpeedCmS());
		});
		OutBack = Furthest - Wheel.GroundCm();
	};
	for (const double Tick : { 1.0 / 60.0, 0.0625 })
	{
		const FString At = FString::Printf(TEXT("at a %.1f ms tick"), 1000.0 * Tick);
		double		  BackSeeded, LeastSeeded, BackFlat, LeastFlat;
		Uphill(6.0, true, Tick, BackSeeded, LeastSeeded);
		Uphill(0.0, false, Tick, BackFlat, LeastFlat);
		TestTrue(FString::Printf(TEXT("%s: uphill with the pull, back %.3f cm from the furthest (on the flat %.3f; within 0.15)"), *At, BackSeeded, BackFlat),
			FMath::Abs(BackSeeded - BackFlat) < 0.15);
		TestTrue(FString::Printf(TEXT("%s: uphill with the pull, least speed %.2f cm/s (on the flat %.2f; within 0.5)"), *At, LeastSeeded, LeastFlat),
			FMath::Abs(LeastSeeded - LeastFlat) < 0.5);
	}
	{
		double BackSeeded, LeastSeeded, BackBare, LeastBare;
		Uphill(6.0, true, 1.0 / 60.0, BackSeeded, LeastSeeded);
		Uphill(6.0, false, 1.0 / 60.0, BackBare, LeastBare);
		TestTrue(FString::Printf(TEXT("uphill at 16.7 ms: from nothing it rolls back at %.2f cm/s (over 3), with the pull %.2f (under 1.5)"), -LeastBare, -LeastSeeded),
			LeastBare < -3.0 && LeastSeeded > -1.5);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHoldEasedToRest,
	"Ramms.Drive.Hold.EasedToRest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHoldEasedToRest::RunTest(const FString& Parameters)
{
	// A hold begun eased (a hand-back) eases its target's braking off as it
	// comes to rest: braked from 25 cm/s on the flat, its loop closed once a
	// 62.5 ms tick, it rolls back after the stop at well under 1 cm/s, where
	// stopped short it rolls back about 2; a slope pulling the wheel on
	// downhill turns the easing off for the rest of that braking, so it stops
	// as soon as one not eased.
	const auto Brake = [](double V0, double SlopeDeg, bool bEase, double& OutLeast, double& OutFurthest) {
		FToyWheel Wheel;
		Wheel.SlopeDeg = SlopeDeg;
		Wheel.Rate = V0 / Wheel.RadiusCm;
		const RammsDrive::FTorqueHoldGains G = RammsDrive::TorqueHoldGains(Wheel.Inertia(), Wheel.MaxTorque, Wheel.RadiusCm, 30.0);
		RammsDrive::FTorqueHold			   Hold;
		Hold.Begin(Wheel.Angle, Wheel.Rate, 0.0, bEase);
		OutLeast = 0.0;
		OutFurthest = 0.0;
		Wheel.Run(Hold, G, 0.0625, 4.0, [&](double) {
			OutLeast = FMath::Min(OutLeast, FMath::Sign(V0) * Wheel.SpeedCmS());
			OutFurthest = FMath::Max(OutFurthest, FMath::Abs(Wheel.GroundCm()));
		});
	};
	double LeastEased, FarEased, LeastShort, FarShort;
	Brake(25.0, 0.0, true, LeastEased, FarEased);
	Brake(25.0, 0.0, false, LeastShort, FarShort);
	TestTrue(FString::Printf(TEXT("flat, eased: rolled back at most %.2f cm/s after the stop (under 0.75)"), -LeastEased), LeastEased > -0.75);
	TestTrue(FString::Printf(TEXT("flat, stopped short: %.2f cm/s (over 1.5: what easing saves)"), -LeastShort), LeastShort < -1.5);
	TestTrue(FString::Printf(TEXT("flat, eased: stopped %.2f cm on, at most 2 cm further than stopped short (%.2f)"), FarEased, FarShort), FarEased - FarShort < 2.0);
	// Downhill (the slope pulling it on, nothing seeded): as far as not eased.
	Brake(-30.0, 6.0, true, LeastEased, FarEased);
	Brake(-30.0, 6.0, false, LeastShort, FarShort);
	TestTrue(FString::Printf(TEXT("downhill, eased: stopped %.2f cm on, as not eased (%.2f; within 0.1)"), FarEased, FarShort), FMath::Abs(FarEased - FarShort) < 0.1);
	// Begun without easing, nothing changes: Begin's default.
	RammsDrive::FTorqueHold Plain;
	Plain.Begin(0.0, 1.0);
	TestFalse(TEXT("not eased unless asked"), Plain.bEaseToRest);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHoldUnreportedAngle,
	"Ramms.Drive.Hold.UnreportedAngle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHoldUnreportedAngle::RunTest(const FString& Parameters)
{
	// A backend that reports no angle for a continuous wheel reads 0 whatever
	// the wheel does. A hold given that reading would brake its target ahead of
	// an angle that never moves, and the loop -- finding the wheel ever short
	// of the target -- would drive it on. Through FWheelAngle, which takes the
	// rate's travel once the reading is seen not to follow it, the same hold
	// stops the wheel and keeps it stopped; a reading that does follow the
	// rate is used as it is.
	const auto Run = [](bool bThroughSource, double& OutLateSpeed, double& OutStopCm) {
		FToyWheel Wheel;
		Wheel.SlopeDeg = 0.0;
		Wheel.Rate = 100.0 / Wheel.RadiusCm;
		const RammsDrive::FTorqueHoldGains G = RammsDrive::TorqueHoldGains(Wheel.Inertia(), Wheel.MaxTorque, Wheel.RadiusCm, 30.0);
		RammsDrive::FTorqueHold			   Hold;
		RammsDrive::FWheelAngle			   Source;
		const double					   Tick = 0.0625, Dt = 0.001;
		const double					   Reading = 0.0; // what such a backend reports, throughout
		Hold.Begin(bThroughSource ? Source.Update(Reading, Wheel.Rate, 0.0) : Reading, Wheel.Rate);
		OutLateSpeed = 0.0;
		for (int32 k = 0; k < 96; ++k)
		{
			const double Angle = bThroughSource ? Source.Update(Reading, Wheel.Rate, k == 0 ? 0.0 : Tick) : Reading;
			const double Torque = FMath::Clamp(Hold.Step(Angle, Wheel.Rate, Tick, G), -Wheel.MaxTorque, Wheel.MaxTorque);
			for (int32 s = 0; s < 62; ++s)
			{
				Wheel.Rate += Torque / Wheel.Inertia() * Dt;
				Wheel.Angle += Wheel.Rate * Dt;
			}
			if (k * Tick >= 4.5)
			{
				OutLateSpeed = FMath::Max(OutLateSpeed, FMath::Abs(Wheel.SpeedCmS()));
			}
		}
		OutStopCm = Wheel.GroundCm();
	};
	double LateSource = 0.0, StopSource = 0.0, LateRaw = 0.0, StopRaw = 0.0;
	Run(true, LateSource, StopSource);
	Run(false, LateRaw, StopRaw);
	TestTrue(FString::Printf(TEXT("through FWheelAngle: let go at 1 m/s, at rest after 4.5 s (%.3f cm/s) %.1f cm on"), LateSource, StopSource),
		LateSource < 0.5 && StopSource > 0.0 && StopSource < 250.0);
	TestTrue(FString::Printf(TEXT("the reading alone: driven on (%.1f cm/s after 4.5 s, %.0f cm on) -- what FWheelAngle is for"), LateRaw, StopRaw),
		LateRaw > 20.0);
	// A reading that follows the rate is the angle.
	RammsDrive::FWheelAngle Follows;
	Follows.Update(5.0, 2.0, 0.0);
	double Angle = 0.0;
	for (int32 k = 1; k <= 10; ++k)
	{
		Angle = Follows.Update(5.0 + 0.1 * k, 2.0, 0.05);
	}
	TestTrue(TEXT("a reading that follows: decided so"), Follows.bDecided && Follows.bReadingFollows);
	TestNearlyEqual(TEXT("...and used as it is"), Angle, 6.0, 1e-9);
	// Not decided before the rate has turned the wheel CheckRad: the reading.
	RammsDrive::FWheelAngle Early;
	Early.Update(0.0, 0.5, 0.0);
	TestNearlyEqual(TEXT("before CheckRad of travel: the reading"), Early.Update(0.0, 0.5, 0.1), 0.0, 1e-12);
	TestFalse(TEXT("...not yet decided"), Early.bDecided);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHoldClockReset,
	"Ramms.Drive.Hold.ClockReset",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHoldClockReset::RunTest(const FString& Parameters)
{
	// The drive modes' clock: each tick's seconds since the last, the first
	// tick kept for the spawn's second without a hold; the clock going back (a
	// simulation reset) says so, and the first tick -- the second without a
	// hold -- starts again from there.
	double First = -1.0, Last = -1.0, Dt = 0.0;
	TestFalse(TEXT("first tick: no reset"), RammsDrive::AdvanceHoldClock(0.02, First, Last, Dt));
	TestNearlyEqual(TEXT("...no time since a last"), Dt, 0.0, 1e-12);
	TestNearlyEqual(TEXT("...the first tick kept"), First, 0.02, 1e-12);
	TestFalse(TEXT("on: no reset"), RammsDrive::AdvanceHoldClock(12.5, First, Last, Dt));
	TestNearlyEqual(TEXT("...the time since the last"), Dt, 12.48, 1e-9);
	TestFalse(TEXT("the same time again (no new physics step): no reset"), RammsDrive::AdvanceHoldClock(12.5, First, Last, Dt));
	TestNearlyEqual(TEXT("...no time"), Dt, 0.0, 1e-12);
	TestTrue(TEXT("back to 0.01: a reset"), RammsDrive::AdvanceHoldClock(0.01, First, Last, Dt));
	TestNearlyEqual(TEXT("...no time since the last"), Dt, 0.0, 1e-12);
	TestNearlyEqual(TEXT("...the spawn's second from here"), First, 0.01, 1e-12);
	TestFalse(TEXT("on from there: no reset"), RammsDrive::AdvanceHoldClock(0.5, First, Last, Dt));
	TestTrue(TEXT("...still within the spawn's second"), 0.5 - First < RammsDrive::SpawnSettleSeconds);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
