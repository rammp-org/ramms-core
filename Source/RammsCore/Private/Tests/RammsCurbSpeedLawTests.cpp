// Tests for the curb controller's speed law (Private/Curb).
//
// The ramp, the deadline cap, the leg-move time model behind it, the stall
// watch, braking a lagging chassis to rest at a stop and watching it come to
// rest there, and the rates the wheels are driven with, each fed made-up
// numbers: what speeds the law asks for, and that only a hard braking cap ever
// asks for a standstill.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Curb/RammsCurbSpeedLaw.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	using RammsCurb::FCarrierFit;
	using RammsCurb::FDeadlineChain;
	using RammsCurb::FStallWatch;

	constexpr float DriveSpeed = 35.0f;
	constexpr float Accel = 60.0f;
	constexpr float Decel = 60.0f;
	constexpr float Tau = 0.1f;
	constexpr float Tolerance = 1.0f;

	/** Seconds for a ramp to take Speed from From to To on ticks of Dt. */
	float RampSeconds(float From, float To, float Acceleration, float Deceleration, float Dt)
	{
		float Speed = From;
		float T = 0.0f;
		for (int32 i = 0; i < 100000 && !FMath::IsNearlyEqual(Speed, To, 1e-4f); ++i)
		{
			Speed = RammsCurb::RampSpeed(Speed, To, Acceleration, Deceleration, Dt);
			T += Dt;
		}
		return T;
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSpeedLawRamp,
	"Ramms.Curb.SpeedLaw.Ramp",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSpeedLawRamp::RunTest(const FString& Parameters)
{
	// The ramp alone, on explicit limits; DriveSpeedFromTuning checks the
	// controller's per-tick law takes them from its tuning.
	const float Dt = 0.001f;
	const float Up = RampSeconds(0.0f, DriveSpeed, Accel, Decel, Dt);
	const float Down = RampSeconds(DriveSpeed, 0.0f, Accel, Decel, Dt);
	TestEqual(TEXT("0 to 35 cm/s at 60 cm/s^2 takes 35/60 s"), Up, DriveSpeed / Accel, 0.002f);
	TestEqual(TEXT("35 to 0 cm/s takes as long at a deceleration of 60: no 2x ramp-down"), Down, Up, 0.002f);
	TestEqual(TEXT("the deceleration is its own limit, not the acceleration's"),
		RampSeconds(DriveSpeed, 0.0f, Accel, 2.0f * Decel, Dt), 0.5f * DriveSpeed / Decel, 0.002f);
	TestEqual(TEXT("one tick slows by Deceleration * dt"), RammsCurb::RampSpeed(30.0f, 0.0f, Accel, Decel, 0.1f), 24.0f, 1e-4f);
	TestEqual(TEXT("one tick speeds up by Acceleration * dt"), RammsCurb::RampSpeed(10.0f, 35.0f, Accel, Decel, 0.1f), 16.0f, 1e-4f);
	TestEqual(TEXT("a ramp stops at its target"), RammsCurb::RampSpeed(10.0f, 12.0f, Accel, Decel, 0.1f), 12.0f, 1e-4f);
	TestEqual(TEXT("a ramp never goes below 0"), RammsCurb::RampSpeed(2.0f, -5.0f, Accel, Decel, 0.1f), 0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSpeedLawDriveSpeedFromTuning,
	"Ramms.Curb.SpeedLaw.DriveSpeedFromTuning",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSpeedLawDriveSpeedFromTuning::RunTest(const FString& Parameters)
{
	// The per-tick law the controller runs (ComputeDriveSpeed hands it its
	// Tuning), with an acceleration and a deceleration that differ, so taking
	// one for the other shows.
	FRammsCurbTuning Tuning;
	Tuning.DriveSpeed = 30.0f;
	Tuning.Acceleration = 60.0f;
	Tuning.Deceleration = 25.0f;
	const float			NoDeadline = TNumericLimits<float>::Max();
	const TArray<float> NoStop;
	const float			Dt = 0.1f;
	TestEqual(TEXT("speeding up at Tuning.Acceleration"), RammsCurb::NextDriveSpeed(Tuning, 10.0f, 1.0f, NoDeadline, NoStop, Dt), 16.0f, 1e-4f);
	TestEqual(TEXT("towards Tuning.DriveSpeed, not past it"), RammsCurb::NextDriveSpeed(Tuning, 29.0f, 1.0f, NoDeadline, NoStop, Dt), 30.0f, 1e-4f);
	TestEqual(TEXT("slowing for a step's speed at Tuning.Deceleration, not Acceleration"),
		RammsCurb::NextDriveSpeed(Tuning, 30.0f, 0.5f, NoDeadline, NoStop, Dt), 27.5f, 1e-4f);
	TestEqual(TEXT("slowing for the deadline at Tuning.Deceleration"),
		RammsCurb::NextDriveSpeed(Tuning, 30.0f, 1.0f, 12.0f, NoStop, Dt), 27.5f, 1e-4f);
	TestEqual(TEXT("a planned stop is ramped down to, at Tuning.Deceleration"),
		RammsCurb::NextDriveSpeed(Tuning, 1.0f, 0.0f, NoDeadline, NoStop, Dt), 0.0f);
	{
		// A hard stop 3 cm ahead: its braking curve at Tuning.Deceleration
		// (sqrt(2 * 25 * 3)), which cuts below the ramp at once.
		const TArray<float> Stops = { 40.0f, 3.0f };
		TestEqual(TEXT("the nearest braking cap binds, shaped at Tuning.Deceleration"),
			RammsCurb::NextDriveSpeed(Tuning, 30.0f, 1.0f, NoDeadline, Stops, Dt), FMath::Sqrt(150.0f), 1e-4f);
		const TArray<float> AtStop = { 0.0f };
		TestEqual(TEXT("and only a braking cap reaches 0 at once"), RammsCurb::NextDriveSpeed(Tuning, 30.0f, 1.0f, NoDeadline, AtStop, Dt), 0.0f);
		const TArray<float> Past = { -2.0f };
		TestEqual(TEXT("as it does past its stop"), RammsCurb::NextDriveSpeed(Tuning, 30.0f, 1.0f, NoDeadline, Past, Dt), 0.0f);
	}
	{
		// From 30 to a standstill at 25 cm/s^2: 1.2 s, not the 0.5 s at the
		// acceleration, nor the 0.25 s of the old 2x ramp-down.
		float Speed = 30.0f;
		float T = 0.0f;
		for (int32 i = 0; i < 10000 && Speed > 0.0f; ++i)
		{
			Speed = RammsCurb::NextDriveSpeed(Tuning, Speed, 0.0f, NoDeadline, NoStop, 0.001f);
			T += 0.001f;
		}
		TestEqual(TEXT("the ramp down takes DriveSpeed / Tuning.Deceleration"), T, 30.0f / 25.0f, 0.002f);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSpeedLawEntrySpeed,
	"Ramms.Curb.SpeedLaw.EntrySpeed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSpeedLawEntrySpeed::RunTest(const FString& Parameters)
{
	using RammsCurb::FEntryWheel;
	// Made-up wheels, 10 cm in radius: what BeginNegotiation measures as it
	// starts a crossing.
	auto Wheel = [](bool bMain, bool bOnGround, float GroundCmS, float RollDeg = 0.0f) {
		FEntryWheel W;
		W.bMain = bMain;
		W.bOnGround = bOnGround;
		W.RadiusCm = 10.0f;
		W.RollForward = FMath::Cos(FMath::DegreesToRadians(RollDeg));
		// A wheel rolling at RollDeg off the heading turns at the chassis
		// speed's part along its rolling direction.
		W.JointRate = GroundCmS * W.RollForward / W.RadiusCm;
		return W;
	};
	{
		// Two main wheels planted, one a little faster (turning), a fallback
		// wheel spinning fast and a main wheel in the air: the planted main
		// wheels' mean.
		const TArray<FEntryWheel> Wheels = { Wheel(true, true, 20.0f), Wheel(true, true, 22.0f), Wheel(false, true, 50.0f),
			Wheel(true, false, 80.0f) };
		TestEqual(TEXT("the planted main wheels' mean ground speed"), RammsCurb::EntrySpeed(Wheels, true, 5.0f), 21.0f, 1e-4f);
	}
	{
		const TArray<FEntryWheel> Wheels = { Wheel(true, true, 21.0f, 30.0f), Wheel(true, true, 21.0f, -30.0f) };
		TestEqual(TEXT("wheels rolling 30 deg off the heading: the chassis speed along it"), RammsCurb::EntrySpeed(Wheels, true, 5.0f), 21.0f,
			1e-3f);
		const TArray<FEntryWheel> Across = { Wheel(true, true, 21.0f, 80.0f) };
		TestEqual(TEXT("a wheel rolling across the heading does not count"), RammsCurb::EntrySpeed(Across, true, 17.0f), 17.0f);
	}
	{
		const TArray<FEntryWheel> Lifted = { Wheel(true, false, 21.0f), Wheel(false, true, 21.0f) };
		TestEqual(TEXT("no main wheel on the ground: the chassis' measured speed"), RammsCurb::EntrySpeed(Lifted, true, 17.0f), 17.0f);
		TestEqual(TEXT("and with no valid measurement, a standstill"), RammsCurb::EntrySpeed(Lifted, false, 17.0f), 0.0f);
		const TArray<FEntryWheel> Backing = { Wheel(true, true, -6.0f) };
		TestEqual(TEXT("never below 0"), RammsCurb::EntrySpeed(Backing, true, -6.0f), 0.0f);
	}
	{
		// A crossing begun at 21 cm/s carries on from there: towards the drive
		// speed it only rises, and never from a standstill.
		FRammsCurbTuning		  Tuning;
		const TArray<float>		  NoStop;
		const TArray<FEntryWheel> Wheels = { Wheel(true, true, 21.0f), Wheel(true, true, 21.0f) };
		float					  Speed = RammsCurb::EntrySpeed(Wheels, true, 21.0f);
		float					  Least = Speed;
		for (int32 i = 0; i < 100; ++i)
		{
			Speed = RammsCurb::NextDriveSpeed(Tuning, Speed, 1.0f, TNumericLimits<float>::Max(), NoStop, 0.016f);
			Least = FMath::Min(Least, Speed);
		}
		TestEqual(TEXT("rising from the entry speed, it never dips"), Least, 21.0f, 1e-4f);
		TestEqual(TEXT("and reaches the drive speed"), Speed, Tuning.DriveSpeed);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSpeedLawCarrierRate,
	"Ramms.Curb.SpeedLaw.CarrierRate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSpeedLawCarrierRate::RunTest(const FString& Parameters)
{
	// A wheel whose link turns at -1.2 rad/s while the wheel turns at 3 rad/s
	// relative to it: 1.8 rad/s in the world, over a 20 ms tick.
	const float	 Dt = 0.02f, Link = -1.2f, Joint = 3.0f;
	const double World = (Link + Joint) * Dt;
	{
		// A backend that reports the joint, angle and rate, relative to the
		// link (MuJoCo's hinge): the link's turn.
		TestEqual(TEXT("joint angle reported: world less joint"), RammsCurb::CarrierRate(World, 0.4f, 0.4f + Joint * Dt, Joint, Joint, Dt), Link,
			1e-3f);
		TestEqual(TEXT("across the angle's wrap"),
			RammsCurb::CarrierRate(World, PI - 0.01f, -PI - 0.01f + Joint * Dt, Joint, Joint, Dt), Link, 1e-3f);
		TestEqual(TEXT("leaving its zero"), RammsCurb::CarrierRate(World, 0.0f, Joint * Dt, Joint, Joint, Dt), Link, 1e-3f);
		// The angle is exact where the rates read at the ends of a tick are
		// not: here they read 2 and 5 rad/s while the joint turned 3 * dt.
		TestEqual(TEXT("the angle wins over the end rates"), RammsCurb::CarrierRate(World, 0.4f, 0.4f + Joint * Dt, 2.0f, 5.0f, Dt), Link, 1e-3f);
	}
	{
		// The same joint on a backend that reports its rate but no angle.
		TestEqual(TEXT("no angle: world less the joint's rate over the tick"), RammsCurb::CarrierRate(World, 0.0f, 0.0f, 2.0f, 4.0f, Dt), Link,
			1e-3f);
	}
	{
		// A backend whose wheel rate is the wheel's own spin in the world
		// (Chaos, for a torque-driven wheel on a bone): no angle, and the rate
		// is 1.7 then 1.9 rad/s across the tick. Its speed loop holds that spin,
		// which already rolls the wheel over the ground at the commanded rate,
		// so nothing is to come off the command: near 0, not the whole spin.
		const float Carrier = RammsCurb::CarrierRate(World, 0.0f, 0.0f, 1.7f, 1.9f, Dt);
		TestEqual(TEXT("world-frame rate: the link's turn comes out near 0"), Carrier, 0.0f, 1e-3f);
		TestTrue(TEXT("where taking the missing angle for 0 would call the wheel's whole spin the link's"),
			FMath::Abs(static_cast<float>(World / Dt) - Carrier) > 1.7f);
	}
	TestEqual(TEXT("a tick of no length measures nothing"), RammsCurb::CarrierRate(World, 0.4f, 0.5f, 3.0f, 3.0f, 0.0f), 0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSpeedLawLegMoveSeconds,
	"Ramms.Curb.SpeedLaw.LegMoveSeconds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSpeedLawLegMoveSeconds::RunTest(const FString& Parameters)
{
	// A command 9 cm from its goal at 18 cm/s arrives in 0.5 s, the leg a lag
	// (1.8 cm) behind it, which a first-order servo closes to 1 cm in
	// tau * ln(1.8).
	TestEqual(TEXT("ramp, then the lag's tail"), RammsCurb::LegMoveSeconds(9.0f, 18.0f, 0.0f, Tau, Tolerance),
		0.5f + Tau * FMath::Loge(1.8f), 1e-5f);
	TestEqual(TEXT("the sign of the move does not matter"), RammsCurb::LegMoveSeconds(-9.0f, 18.0f, 0.0f, Tau, Tolerance),
		RammsCurb::LegMoveSeconds(9.0f, 18.0f, 0.0f, Tau, Tolerance), 1e-6f);
	TestEqual(TEXT("a leg lagging more than the model trails by its own error"),
		RammsCurb::LegMoveSeconds(9.0f, 18.0f, 4.0f, Tau, Tolerance), 0.5f + Tau * FMath::Loge(4.0f), 1e-5f);
	TestEqual(TEXT("a command already there: only the leg's own error to close"),
		RammsCurb::LegMoveSeconds(0.0f, 18.0f, 3.0f, Tau, Tolerance), Tau * FMath::Loge(3.0f), 1e-5f);
	TestEqual(TEXT("a settled leg is done"), RammsCurb::LegMoveSeconds(0.0f, 18.0f, 0.6f, Tau, Tolerance), 0.0f);
	TestEqual(TEXT("a slow ramp inside the tolerance has no tail"), RammsCurb::LegMoveSeconds(2.0f, 5.0f, 0.0f, Tau, Tolerance),
		0.4f, 1e-5f);

	// A lift from 0 towards 15 cm at 30 cm/s: the command gets there in 0.5 s,
	// the leg a lag later; with the command already past it, the leg closes on
	// it exponentially.
	TestEqual(TEXT("reach behind a ramping command"), RammsCurb::LegReachSeconds(0.0f, 0.0f, 30.0f, 15.0f, Tau), 0.5f + Tau, 1e-5f);
	TestEqual(TEXT("reach behind a command past the target"), RammsCurb::LegReachSeconds(10.0f, 20.0f, 30.0f, 15.0f, Tau),
		Tau * FMath::Loge(2.0f), 1e-5f);
	TestEqual(TEXT("already there"), RammsCurb::LegReachSeconds(16.0f, 20.0f, 30.0f, 15.0f, Tau), 0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSpeedLawDeadlineCap,
	"Ramms.Curb.SpeedLaw.DeadlineCap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSpeedLawDeadlineCap::RunTest(const FString& Parameters)
{
	const float M = RammsCurb::DeadlineMarginCm;
	const float S = RammsCurb::DeadlineMarginSeconds;

	// The deadline speed: held for the work, it leaves just the braking
	// distance to the hard stop -- v T + v^2 / 2a = room -- and with no work
	// left it is the stop's own braking curve.
	{
		const float V = RammsCurb::DeadlineSpeed(29.0f, 1.1f, Decel);
		TestEqual(TEXT("v T + v^2 / 2a = room"), V * 1.1f + V * V / (2.0f * Decel), 29.0f, 1e-3f);
		TestEqual(TEXT("with no braking limit, room over time"), RammsCurb::DeadlineSpeed(29.0f, 1.1f, 0.0f), 29.0f / 1.1f, 1e-4f);
		TestEqual(TEXT("with no time, the braking curve"), RammsCurb::DeadlineSpeed(29.0f, 0.0f, Decel),
			RammsCurb::DeadlineSpeed(29.0f, RammsCurb::MinDeadlineSeconds, Decel), 1e-6f);
		TestTrue(TEXT("at the least time, close to the braking curve"),
			FMath::Abs(RammsCurb::DeadlineSpeed(29.0f, 0.0f, Decel) - RammsCurb::BrakingSpeed(29.0f, Decel)) < 4.0f);
		TestEqual(TEXT("no room: stop"), RammsCurb::DeadlineSpeed(0.0f, 1.0f, Decel), 0.0f);
	}

	// Time only: the deadline speed of the room over all the work.
	{
		FDeadlineChain Chain;
		Chain.AddTime(0.6f);
		Chain.AddTime(0.4f);
		TestEqual(TEXT("time-gated work: the room over its time"), Chain.Speed(30.0f, Decel),
			RammsCurb::DeadlineSpeed(30.0f - M, 1.0f + S, Decel), 1e-4f);
		TestEqual(TEXT("no work: the least time is divided by, not 0"), FDeadlineChain().Speed(30.0f, Decel),
			RammsCurb::DeadlineSpeed(30.0f - M, FMath::Max(S, RammsCurb::MinDeadlineSeconds), Decel), 1e-3f);
	}

	// A gate: the work after it cannot start before the chassis gets there,
	// so behind it only the room beyond the gate is left -- a speed the law
	// brakes down to from now, at Deceleration.
	{
		FDeadlineChain Chain;
		Chain.AddTime(0.2f);
		Chain.AddGate(25.0f);
		Chain.AddTime(1.0f);
		const float After = RammsCurb::DeadlineSpeed(30.0f - M - 25.0f, 1.0f + S, Decel);
		const float Braked = FMath::Sqrt(After * After + 2.0f * Decel * 25.0f);
		const float Constant = RammsCurb::DeadlineSpeed(30.0f - M, 1.2f + S, Decel);
		TestEqual(TEXT("far from the gate the bound over all the work binds"), Chain.Speed(30.0f, Decel), FMath::Min(Braked, Constant), 1e-3f);
		FDeadlineChain AtGate;
		AtGate.AddGate(0.0f);
		AtGate.AddTime(1.0f);
		TestEqual(TEXT("at the gate, what is left of the room over the work after it"), AtGate.Speed(5.0f + M, Decel),
			RammsCurb::DeadlineSpeed(5.0f, 1.0f + S, Decel), 1e-4f);
		FDeadlineChain Near;
		Near.AddGate(20.0f);
		Near.AddTime(1.0f);
		const float NearAfter = RammsCurb::DeadlineSpeed(80.0f - M - 20.0f, 1.0f + S, Decel);
		TestEqual(TEXT("ahead of a gate it may go faster, and brake to the gate's speed at Deceleration"), Near.Speed(80.0f, Decel),
			FMath::Min(RammsCurb::DeadlineSpeed(80.0f - M, 1.0f + S, Decel), FMath::Sqrt(NearAfter * NearAfter + 2.0f * Decel * 20.0f)),
			1e-3f);
	}

	// Driving at the cap: the robot reaches the point it must be clear by no
	// sooner than the work ends, and slows smoothly without stopping.
	{
		float		Room = 30.0f;
		float		Work = 1.0f;
		float		Speed = DriveSpeed;
		float		Least = Speed;
		float		LargestDrop = 0.0f;
		float		LastCap = -1.0f;
		const float Dt = 0.016f;
		while (Work > 0.0f)
		{
			FDeadlineChain Chain;
			Chain.AddTime(Work);
			const float Cap = Chain.Speed(Room, Decel);
			if (LastCap >= 0.0f)
			{
				LargestDrop = FMath::Max(LargestDrop, LastCap - Cap);
			}
			LastCap = Cap;
			Speed = RammsCurb::RampSpeed(Speed, FMath::Min(DriveSpeed, Cap), Accel, Decel, Dt);
			Least = FMath::Min(Least, Speed);
			Room -= Speed * Dt;
			Work -= Dt;
		}
		TestTrue(TEXT("the work ends before the chassis uses up its room"), Room > 0.0f);
		TestTrue(TEXT("the deadline never stops the robot"), Least > 15.0f);
		TestTrue(TEXT("the cap moves continuously (no jump between ticks)"), LargestDrop < 1.0f);
	}

	// Only a used-up room reaches zero.
	{
		FDeadlineChain Chain;
		Chain.AddTime(3.0f);
		TestTrue(TEXT("a long wait with room left is slow, not a stop"), Chain.Speed(4.0f, Decel) > 0.0f);
		TestEqual(TEXT("no room left: stop"), Chain.Speed(M, Decel), 0.0f);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSpeedLawOnlyBrakingCapReachesZero,
	"Ramms.Curb.SpeedLaw.OnlyBrakingCapReachesZero",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSpeedLawOnlyBrakingCapReachesZero::RunTest(const FString& Parameters)
{
	// A robot heads for a hard stop 30 cm ahead (a wheel short of the face)
	// while 0.9 s of leg work clears that wheel: at its drive speed it would
	// get there first. The deadline slows it so the work ends with the robot
	// still able to brake short of the stop; the braking cap -- the only term
	// allowed to reach 0 -- never has to act, and the robot never stops.
	float		Stop = 30.0f;
	float		Work = 0.9f;
	float		Speed = DriveSpeed;
	float		Least = Speed;
	bool		bBrakingBound = false;
	const float Dt = 0.01f;
	while (Work > 0.0f)
	{
		FDeadlineChain Chain;
		Chain.AddTime(Work);
		const float Deadline = Chain.Speed(Stop, Decel);
		const float Ramped = RammsCurb::RampSpeed(Speed, FMath::Min(DriveSpeed, Deadline), Accel, Decel, Dt);
		const float Braking = RammsCurb::BrakingSpeed(Stop, Decel);
		bBrakingBound |= Braking < Ramped;
		Speed = FMath::Min(Ramped, Braking);
		Least = FMath::Min(Least, Speed);
		Stop -= Speed * Dt;
		Work -= Dt;
	}
	TestFalse(TEXT("the braking cap never had to act"), bBrakingBound);
	TestTrue(TEXT("the robot slowed for the work, but kept rolling"), Least > 20.0f && Least < 30.0f);
	TestTrue(TEXT("and could still brake short of the hard stop as the work ended"),
		Stop > Speed * Speed / (2.0f * Decel));

	// The deadline is zero only where the hard stop's own braking curve is.
	TestEqual(TEXT("the braking cap is zero at its stop"), RammsCurb::BrakingSpeed(0.0f, Decel), 0.0f);
	TestEqual(TEXT("and past it"), RammsCurb::BrakingSpeed(-3.0f, Decel), 0.0f);
	TestEqual(TEXT("braking curve sqrt(2 a d)"), RammsCurb::BrakingSpeed(10.0f, Decel), FMath::Sqrt(1200.0f), 1e-4f);
	for (float Room = RammsCurb::DeadlineMarginCm + 0.5f; Room < 100.0f; Room += 7.3f)
	{
		for (float Seconds = 0.0f; Seconds < 5.0f; Seconds += 0.7f)
		{
			FDeadlineChain Chain;
			Chain.AddTime(Seconds);
			if (!TestTrue(FString::Printf(TEXT("room %.1f cm, %.1f s of work: slower, never stopped"), Room, Seconds),
					Chain.Speed(Room, Decel) > 0.0f))
			{
				return false;
			}
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSpeedLawNoLagStepFactor,
	"Ramms.Curb.SpeedLaw.NoLagStepFactor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSpeedLawNoLagStepFactor::RunTest(const FString& Parameters)
{
	// The old rule cut the speed to 0.4 of itself the moment any leg lagged
	// 2 cm. Here a lagging leg only lengthens the predicted work, and the cap
	// falls with it continuously: no jump anywhere as the lag grows.
	float Last = -1.0f;
	float LargestRatio = 1.0f;
	for (float Error = 0.0f; Error <= 6.0f; Error += 0.01f)
	{
		FDeadlineChain Chain;
		Chain.AddTime(RammsCurb::LegMoveSeconds(10.0f, 30.0f, Error, Tau, Tolerance));
		const float Cap = Chain.Speed(25.0f, Decel);
		if (Last > 0.0f)
		{
			LargestRatio = FMath::Max(LargestRatio, Last / Cap);
		}
		Last = Cap;
	}
	TestTrue(TEXT("no step between neighbouring lags (under 2% per 0.1 mm)"), LargestRatio < 1.02f);
	TestTrue(TEXT("and a 6 cm lag still leaves the robot rolling"), Last > 0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSpeedLawStall,
	"Ramms.Curb.SpeedLaw.Stall",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSpeedLawStall::RunTest(const FString& Parameters)
{
	const float Dt = 0.02f;
	auto		Run = [Dt](FStallWatch& Watch, float Seconds, float Commanded, float Measured, bool bValid = true) {
		   bool bStalled = false;
		   for (int32 i = 0; i < FMath::RoundToInt(Seconds / Dt); ++i)
		   {
			   bStalled |= Watch.Tick(Commanded, Measured, bValid, Dt, 2.0f);
		   }
		   return bStalled;
	};
	{
		FStallWatch Watch;
		TestFalse(TEXT("1.9 s commanded 14 cm/s without moving is not yet a stall"), Run(Watch, 1.9f, 14.0f, 0.5f));
		TestTrue(TEXT("2 s is"), Run(Watch, 0.12f, 14.0f, 0.5f));
	}
	{
		FStallWatch Watch;
		Run(Watch, 1.9f, 14.0f, 0.5f);
		Run(Watch, 0.1f, 14.0f, 3.0f);
		TestFalse(TEXT("moving again starts the count over"), Run(Watch, 1.9f, 14.0f, 0.5f));
	}
	{
		FStallWatch Watch;
		TestFalse(TEXT("a speed of 5 cm/s or less commanded is never a stall"), Run(Watch, 5.0f, 5.0f, 0.0f));
		TestFalse(TEXT("nor is a robot moving at 1 cm/s"), Run(Watch, 5.0f, 14.0f, 1.0f));
		TestFalse(TEXT("nor rolling backwards with nothing commanded"), Run(Watch, 5.0f, 0.0f, -3.0f));
		TestTrue(TEXT("rolling back while commanded forwards is"), Run(Watch, 2.1f, 14.0f, -3.0f));
	}
	{
		FStallWatch Watch;
		Run(Watch, 1.5f, 14.0f, 0.5f);
		TestFalse(TEXT("a measurement not yet valid neither counts"), Run(Watch, 3.0f, 14.0f, 0.5f, false));
		TestEqual(TEXT("nor clears the count"), Watch.GetSeconds(), 1.5f, 1e-3f);
	}
	{
		FStallWatch Watch;
		bool		bStalled = false;
		for (int32 i = 0; i < 500; ++i)
		{
			bStalled |= Watch.Tick(14.0f, 0.0f, true, Dt, 0.0f);
		}
		TestFalse(TEXT("StallSeconds 0 turns the check off"), bStalled);
	}
	return true;
}

namespace
{
	/** Where a braked chassis came to rest, and how hard it and its command
	 *  slowed on the way. */
	struct FStopRun
	{
		/** Past the stop (cm); negative short of it. */
		double OverrunCm = 0.0;
		double FinalSpeed = 0.0;
		/** The chassis' largest deceleration (cm/s^2). */
		double MaxDecel = 0.0;
		/** The command's largest fall (cm/s^2) while it was above 2 cm/s. */
		float MaxCommandFall = 0.0f;
	};

	/**
	 * A chassis following its drive command as a first-order lag of PlantLag
	 * (s), at 30 cm/s with a stop 50 cm ahead, driven by the per-tick law on
	 * ticks of Dt: braked on the stop's lag-aware distance for a lag of
	 * ModelLag, or on the raw distance (bLagAware false).
	 */
	FStopRun BrakeToStop(float ModelLag, float PlantLag, float Dt, bool bLagAware)
	{
		FRammsCurbTuning Tuning;
		Tuning.DriveSpeed = 30.0f;
		Tuning.Acceleration = Accel;
		Tuning.Deceleration = Decel;
		const float NoDeadline = TNumericLimits<float>::Max();
		const float Stop = 50.0f;
		double		X = 0.0;
		double		V = Tuning.DriveSpeed;
		float		Command = Tuning.DriveSpeed;
		FStopRun	Out;
		for (int32 i = 0; i < FMath::RoundToInt(6.0f / Dt); ++i)
		{
			const float			Distance = static_cast<float>(Stop - X);
			const TArray<float> Braking = { bLagAware ? RammsCurb::LagBrakingDistance(Distance, static_cast<float>(V), ModelLag) : Distance };
			const float			Next = RammsCurb::NextDriveSpeed(Tuning, Command, 1.0f, NoDeadline, Braking, Dt);
			if (Next > 2.0f)
			{
				Out.MaxCommandFall = FMath::Max(Out.MaxCommandFall, (Command - Next) / Dt);
			}
			Command = Next;
			// Over the tick, its command held: the first order's exact step.
			const double E = FMath::Exp(-static_cast<double>(Dt) / PlantLag);
			const double VNext = Command + (V - Command) * E;
			X += Command * Dt + (V - Command) * PlantLag * (1.0 - E);
			Out.MaxDecel = FMath::Max(Out.MaxDecel, (V - VNext) / Dt);
			V = VNext;
		}
		Out.OverrunCm = X - Stop;
		Out.FinalSpeed = V;
		return Out;
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSpeedLawLagAwareStop,
	"Ramms.Curb.SpeedLaw.LagAwareStop",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSpeedLawLagAwareStop::RunTest(const FString& Parameters)
{
	TestEqual(TEXT("the lag's run-on comes off the distance"), RammsCurb::LagBrakingDistance(10.0f, 20.0f, 0.15f), 7.0f, 1e-5f);
	TestEqual(TEXT("a chassis rolling back adds no distance"), RammsCurb::LagBrakingDistance(10.0f, -5.0f, 0.15f), 10.0f);
	TestEqual(TEXT("no lag, no change"), RammsCurb::LagBrakingDistance(10.0f, 20.0f, 0.0f), 10.0f);

	const float Lag = 0.12f;
	for (const float Dt : { 1.0f / 120.0f, 1.0f / 60.0f, 1.0f / 30.0f })
	{
		const FString  At = FString::Printf(TEXT("ticks of %.1f ms"), 1000.0f * Dt);
		const FStopRun Run = BrakeToStop(Lag, Lag, Dt, true);
		TestTrue(FString::Printf(TEXT("%s: the chassis comes to rest on the stop (%+.2f cm, %.3f cm/s)"), *At, Run.OverrunCm, Run.FinalSpeed),
			FMath::Abs(Run.OverrunCm) <= 0.25 && Run.FinalSpeed < 0.05);
		TestTrue(FString::Printf(TEXT("%s: slowing at about Deceleration (chassis %.0f, command %.0f cm/s^2)"), *At, Run.MaxDecel, Run.MaxCommandFall),
			Run.MaxDecel <= 1.35 * Decel && Run.MaxCommandFall <= 1.3f * Decel);

		// The same stop braked on the raw distance: the reason for the lag term.
		const FStopRun Raw = BrakeToStop(Lag, Lag, Dt, false);
		TestTrue(FString::Printf(TEXT("%s: on the raw distance the chassis overruns (%+.2f cm) and slows about twice as hard (%.0f)"), *At,
					 Raw.OverrunCm, Raw.MaxDecel),
			Raw.OverrunCm > 1.5 && Raw.MaxDecel > 1.6 * Decel);

		// A chassis lagging more than allowed for stops a little past, by the
		// difference times its speed as the command reaches 0; one lagging
		// less stops short and creeps on to the stop.
		const FStopRun Slow = BrakeToStop(Lag, 0.16f, Dt, true);
		TestTrue(FString::Printf(TEXT("%s: a slower chassis rests just past the stop (%+.2f cm)"), *At, Slow.OverrunCm),
			Slow.OverrunCm >= 0.0 && Slow.OverrunCm <= 1.0);
		const FStopRun Quick = BrakeToStop(Lag, 0.08f, Dt, true);
		TestTrue(FString::Printf(TEXT("%s: a quicker one rests on it (%+.2f cm)"), *At, Quick.OverrunCm),
			FMath::Abs(Quick.OverrunCm) <= 0.25 && Quick.FinalSpeed < 0.05);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSpeedLawRestWatch,
	"Ramms.Curb.SpeedLaw.RestWatch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSpeedLawRestWatch::RunTest(const FString& Parameters)
{
	using RammsCurb::ERest;
	using RammsCurb::FRestWatch;
	const float Dt = 0.02f;
	const float Stopped = 1.0f;
	const float Limit = 2.0f;
	auto		Hold = [&](FRestWatch& Watch, float Seconds, float Measured, bool bValid = true) {
		   ERest Last = ERest::Moving;
		   for (int32 i = 0; i < FMath::RoundToInt(Seconds / Dt); ++i)
		   {
			   Last = Watch.Tick(true, Measured, bValid, Dt, Stopped, Limit);
		   }
		   return Last;
	};
	{
		FRestWatch Watch;
		TestTrue(TEXT("held with the command at 0 but the chassis still at 17 cm/s: not at rest"), Hold(Watch, 0.04f, 17.0f) == ERest::Moving);
		TestTrue(TEXT("at rest once measured under 1 cm/s"), Hold(Watch, 0.02f, 0.4f) == ERest::Rested);
	}
	{
		FRestWatch Watch;
		TestTrue(TEXT("a measurement not yet valid is not rest"), Hold(Watch, 0.5f, 0.0f, false) == ERest::Moving);
		TestTrue(TEXT("rolling back at 0.5 cm/s counts as at rest"), Hold(Watch, 0.02f, -0.5f) == ERest::Rested);
	}
	{
		FRestWatch Watch;
		TestTrue(TEXT("rolling back at 3 cm/s is not"), Hold(Watch, 1.9f, -3.0f) == ERest::Moving);
		TestTrue(TEXT("held as long as allowed without coming to rest: timed out"), Hold(Watch, 0.12f, 3.0f) == ERest::TimedOut);
		TestTrue(TEXT("released, the count starts over"), Watch.Tick(false, 3.0f, true, Dt, Stopped, Limit) == ERest::Moving);
		TestEqual(TEXT("from 0"), Watch.GetSeconds(), 0.0f);
		TestTrue(TEXT("and held again, it waits again"), Hold(Watch, 1.9f, 3.0f) == ERest::Moving);
	}
	{
		FRestWatch Watch;
		TestTrue(TEXT("no limit: it waits for rest however long"), [&] {
			ERest Last = ERest::Moving;
			for (int32 i = 0; i < 1000; ++i)
			{
				Last = Watch.Tick(true, 3.0f, true, Dt, Stopped, 0.0f);
			}
			return Last == ERest::Moving;
		}());
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSpeedLawCarrierFit,
	"Ramms.Curb.SpeedLaw.CarrierFit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSpeedLawCarrierFit::RunTest(const FString& Parameters)
{
	// A made-up carrier that turns 0.06 rad per cm the leg walks and -0.02
	// per cm it rises: walks one way and the other, then a rise and a fall.
	FCarrierFit Fit;
	TestEqual(TEXT("before any motion it predicts nothing"), Fit.Predict(20.0f, 18.0f), 0.0f);
	const float Dt = 0.016f;
	auto		Feed = [&Fit, Dt](float Seconds, float XRate, float ZRate, float Kx, float Kz) {
		   for (int32 i = 0; i < FMath::RoundToInt(Seconds / Dt); ++i)
		   {
			   Fit.Add(XRate, ZRate, Kx * XRate + Kz * ZRate, Dt);
		   }
	};
	Feed(0.4f, 20.0f, 0.0f, 0.06f, -0.02f);
	Feed(0.4f, -20.0f, 0.0f, 0.06f, -0.02f);
	TestEqual(TEXT("walking alone teaches the fore/aft gain"), static_cast<float>(Fit.GetGains().X), 0.06f, 0.003f);
	TestEqual(TEXT("and leaves the vertical one at 0, never having seen it"), static_cast<float>(Fit.GetGains().Y), 0.0f, 1e-4f);
	Feed(0.25f, 0.0f, 18.0f, 0.06f, -0.02f);
	Feed(0.25f, 0.0f, -18.0f, 0.06f, -0.02f);
	TestEqual(TEXT("rising and falling teach the vertical gain"), static_cast<float>(Fit.GetGains().Y), -0.02f, 0.001f);
	TestEqual(TEXT("and keep the fore/aft one"), static_cast<float>(Fit.GetGains().X), 0.06f, 0.003f);
	TestEqual(TEXT("the prediction is the fitted line"), Fit.Predict(10.0f, 5.0f), 0.06f * 10.0f - 0.02f * 5.0f, 0.04f);
	// The gain changes as the pose does: the fit follows within its memory.
	Feed(1.0f, 20.0f, 0.0f, 0.03f, -0.02f);
	Feed(1.0f, -20.0f, 0.0f, 0.03f, -0.02f);
	TestEqual(TEXT("a new gain is learnt within two memories"), static_cast<float>(Fit.GetGains().X), 0.03f, 0.003f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSpeedLawWheelFeed,
	"Ramms.Curb.SpeedLaw.WheelFeed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSpeedLawWheelFeed::RunTest(const FString& Parameters)
{
	// A 5-bar walks its wheel 8 cm at 20 cm/s and back, three times, its
	// servo a first-order lag of 0.1 s on the command, and the link the wheel
	// hangs from turning -0.12 rad per cm walked. The controller ticks every
	// 37 ms and needs each rate over the tick to come. Measured alone, the
	// rates are a tick late, worst as a walk starts and ends; with the change
	// the lag model predicts, they are on time once the fit has seen a walk.
	const double		  Tick = 0.037, Kc = -0.12;
	double				  X = 0.0, Cmd = 0.0, LagX = 0.0, LastLagX = 0.0;
	double				  LastTrueX = 0.0;
	RammsCurb::FWheelFeed Feed;
	float				  WorstFed = 0.0f, WorstMeasured = 0.0f, WorstCarrierFed = 0.0f, WorstCarrierMeasured = 0.0f;
	float				  LastMeasuredX = 0.0f, LastMeasuredRate = 0.0f;
	double				  Target = 8.0;
	int32				  Walks = 0;
	for (int32 k = 0; k < 400 && Walks < 6; ++k)
	{
		// Measure the tick just gone: where the wheel is, how fast the link
		// turned over it, and what the model did over it.
		const double TrueRate = (X - LastTrueX) / Tick;
		Feed.Measure(static_cast<float>(Tick), FVector2D(X, 0.0), static_cast<float>(Kc * TrueRate),
			FVector2D((LagX - LastLagX) / Tick, 0.0));
		LastMeasuredRate = k > 0 ? static_cast<float>((X - LastMeasuredX) / Tick) : 0.0f;
		LastMeasuredX = static_cast<float>(X);
		LastTrueX = X;
		LastLagX = LagX;

		// Send this tick's command: walk at 20 cm/s, and turn round once the
		// wheel has arrived.
		if (FMath::IsNearlyEqual(Cmd, Target, 1e-6) && FMath::Abs(X - Target) < 0.01)
		{
			Target = Target > 0.0 ? 0.0 : 8.0;
			++Walks;
		}
		Cmd += FMath::Clamp(Target - Cmd, -20.0 * Tick, 20.0 * Tick);
		const double NextLag = RammsCurb::LagStep(static_cast<float>(LagX), static_cast<float>(Cmd), static_cast<float>(Tick), 0.1f);
		Feed.Predict(FVector2D((NextLag - LagX) / Tick, 0.0));
		LagX = NextLag;

		// The servo over the tick to come, in fine steps.
		const double From = X;
		for (int32 s = 0; s < 37; ++s)
		{
			X += (Cmd - X) * (1.0 - FMath::Exp(-0.001 / 0.1));
		}
		const double Rate = (X - From) / Tick;
		if (Walks >= 2) // once the fit has seen a walk both ways
		{
			WorstFed = FMath::Max(WorstFed, FMath::Abs(Feed.GetLegXRate() - static_cast<float>(Rate)));
			WorstMeasured = FMath::Max(WorstMeasured, FMath::Abs(LastMeasuredRate - static_cast<float>(Rate)));
			WorstCarrierFed = FMath::Max(WorstCarrierFed, FMath::Abs(Feed.GetCarrierRate() - static_cast<float>(Kc * Rate)));
			WorstCarrierMeasured = FMath::Max(WorstCarrierMeasured, FMath::Abs(static_cast<float>(Kc * LastMeasuredRate - Kc * Rate)));
		}
	}
	TestTrue(TEXT("the walks ran"), Walks >= 4);
	TestTrue(FString::Printf(TEXT("measured alone, the walk rate is up to %.1f cm/s off"), WorstMeasured), WorstMeasured > 4.0f);
	TestTrue(FString::Printf(TEXT("fed forward, %.2f cm/s"), WorstFed), WorstFed < 0.1f * WorstMeasured);
	TestTrue(FString::Printf(TEXT("and the carrier %.3f rad/s, against %.3f measured alone"), WorstCarrierFed, WorstCarrierMeasured),
		WorstCarrierFed < 0.15f * WorstCarrierMeasured);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSpeedLawCarrierTurn,
	"Ramms.Curb.SpeedLaw.CarrierTurn",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSpeedLawCarrierTurn::RunTest(const FString& Parameters)
{
	// The sense: a wheel rolling forward (+X) turns its top towards +X, which
	// in the quaternion algebra is a positive turn about Up x Forward.
	const FVector Forward = FVector::ForwardVector;
	const FVector Axle = FVector::CrossProduct(FVector::UpVector, Forward);
	TestTrue(TEXT("a positive turn about Up x Forward rolls the wheel forward"),
		FQuat(Axle, 0.1).RotateVector(FVector::UpVector).X > 0.0);

	// A wheel on a chassis yawed 30 deg, hanging from a link that turns at
	// -1.2 rad/s (a 5-bar walking its wheel forward) while its joint turns at
	// 3 rad/s: its world turn about the axle is the sum, so the world turn less
	// the joint's is the link's.
	for (const double RollDeg : { 0.0, 150.0 })
	{
		const FQuat	  Yaw(FVector::UpVector, FMath::DegreesToRadians(30.0));
		const FVector F = Yaw.RotateVector(Forward);
		const FVector R = FVector::CrossProduct(FVector::UpVector, F);
		const double  Theta = FMath::DegreesToRadians(RollDeg);
		const FVector A = (R * FMath::Cos(Theta) - F * FMath::Sin(Theta)).GetSafeNormal();
		const double  Dt = 0.02, Link = -1.2, Joint = 3.0;
		const FVector AxleInBody = Yaw.Inverse().RotateVector(A);
		auto		  At = [&](double T) { return Yaw * FQuat(AxleInBody, Link * T) * FQuat(AxleInBody, 0.4 + Joint * T); };
		const double  Turn = RammsCurb::TurnAbout(At(1.0), At(1.0 + Dt), A);
		TestEqual(FString::Printf(TEXT("roll direction %.0f deg: world turn = link + joint"), RollDeg), Turn, (Link + Joint) * Dt, 1e-6);
		TestEqual(FString::Printf(TEXT("roll direction %.0f deg: the link's rate"), RollDeg), Turn / Dt - Joint, Link, 1e-4);
		TestEqual(TEXT("a turn about another axis does not count"),
			RammsCurb::TurnAbout(FQuat::Identity, FQuat(FVector::UpVector, 0.3), A), 0.0, 1e-9);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
