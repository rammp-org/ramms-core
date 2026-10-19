// Tests for the curb controller's run statistics (Private/Curb).
//
// The speed window and the stats tracker are fed made-up motion, one tick at
// a time, the way the controller feeds them: what counts as under way, as a
// stop, as the least speed, and what is kept once a crossing ends.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Curb/RammsCurbRunStats.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	using RammsCurb::FMotionWindow;
	using RammsCurb::FRunStatsTracker;
	using RammsCurb::FRunTick;

	constexpr float TickSeconds = 0.01f;
	/** A made-up drive speed of 30 cm/s: the speed minima start at 7.5. */
	constexpr float CruiseFloor = 0.25f * 30.0f;

	/** Feeds a tracker ticks of TickSeconds, keeping the clock. */
	struct FTickFeeder
	{
		FRunStatsTracker& Tracker;
		double			  Elapsed = 0.0;

		/** Seconds of ticks at these speeds, everything else at rest. */
		void Run(float Seconds, float Commanded, float Measured, bool bLegsMoved = false)
		{
			const int32 N = FMath::RoundToInt(Seconds / TickSeconds);
			for (int32 i = 0; i < N; ++i)
			{
				Elapsed += TickSeconds;
				FRunTick T;
				T.DeltaSeconds = TickSeconds;
				T.Elapsed = Elapsed;
				T.CommandedSpeed = Commanded;
				T.MeasuredSpeed = Measured;
				T.bMeasuredSpeedValid = true;
				T.bLegsMoved = bLegsMoved;
				Tracker.Tick(T);
			}
		}
	};

	/**
	 * Drives a tracker the way the controller does: a speed that ramps at a
	 * set acceleration, integrated into chassis positions and measured
	 * through FMotionWindow, on ticks of a typical editor frame.
	 */
	struct FRobotFeeder
	{
		static constexpr double Dt = 0.038;

		FRunStatsTracker& Tracker;
		FMotionWindow	  Window;
		double			  Time = 0.0;
		double			  X = 0.0;
		double			  Speed = 0.0;

		/** Seconds of ticks, the commanded speed ramping towards Target at
		 *  Accel (cm/s^2) and the chassis moving at it. */
		void Drive(double Seconds, double Target, double Accel = 60.0)
		{
			const int32 N = FMath::RoundToInt(Seconds / Dt);
			for (int32 i = 0; i < N; ++i)
			{
				Time += Dt;
				Speed = Speed < Target ? FMath::Min(Speed + Accel * Dt, Target) : FMath::Max(Speed - Accel * Dt, Target);
				X += Speed * Dt;
				Window.Add(Time, FVector2D(X, 0.0), FVector2D(1.0, 0.0));
				FRunTick T;
				T.DeltaSeconds = static_cast<float>(Dt);
				T.Elapsed = Time;
				T.CommandedSpeed = static_cast<float>(Speed);
				T.MeasuredSpeed = Window.GetSpeed();
				T.bMeasuredSpeedValid = Window.IsFull();
				Tracker.Tick(T);
			}
		}
	};
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbRunStatsMotionWindowSpeed,
	"Ramms.Curb.RunStats.MotionWindow.Speed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbRunStatsMotionWindowSpeed::RunTest(const FString& Parameters)
{
	// 20 cm/s along a heading 30 deg off X, sampled every 16 ms.
	const FVector2D Forward(FMath::Cos(FMath::DegreesToRadians(30.0)), FMath::Sin(FMath::DegreesToRadians(30.0)));
	FMotionWindow	Window;
	TestFalse(TEXT("an empty window is not full"), Window.IsFull());
	TestEqual(TEXT("an empty window reads 0"), Window.GetSpeed(), 0.0f);
	double T = 10.0;
	for (int32 i = 0; i < 15; ++i, T += 0.016)
	{
		Window.Add(T, FVector2D(100.0, 50.0) + Forward * 20.0 * (T - 10.0), Forward);
	}
	TestFalse(TEXT("0.22 s of samples do not fill a 0.3 s window"), Window.IsFull());
	for (int32 i = 0; i < 60; ++i, T += 0.016)
	{
		Window.Add(T, FVector2D(100.0, 50.0) + Forward * 20.0 * (T - 10.0), Forward);
	}
	TestTrue(TEXT("a second of samples fills it"), Window.IsFull());
	TestEqual(TEXT("steady motion along the heading reads its speed"), Window.GetSpeed(), 20.0f, 0.01f);

	// Sideways to the heading is no progress; backwards is negative.
	FMotionWindow	Sideways, Backwards;
	const FVector2D Side(-Forward.Y, Forward.X);
	for (int32 i = 0; i < 40; ++i)
	{
		Sideways.Add(0.016 * i, Side * 20.0 * 0.016 * i, Forward);
		Backwards.Add(0.016 * i, -Forward * 15.0 * 0.016 * i, Forward);
	}
	TestEqual(TEXT("motion across the heading reads 0"), Sideways.GetSpeed(), 0.0f, 0.01f);
	TestEqual(TEXT("rolling back reads negative"), Backwards.GetSpeed(), -15.0f, 0.01f);

	// The window keeps only the last 0.3 s or so: a stop shows within it.
	FMotionWindow Stopping;
	double		  X = 0.0;
	for (int32 i = 0; i < 100; ++i)
	{
		X += (i < 60 ? 30.0 : 0.0) * 0.016;
		Stopping.Add(0.016 * i, FVector2D(X, 0.0), FVector2D(1.0, 0.0));
	}
	TestEqual(TEXT("0.6 s after stopping the window reads 0"), Stopping.GetSpeed(), 0.0f, 0.01f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbRunStatsMotionWindowReset,
	"Ramms.Curb.RunStats.MotionWindow.ClockReset",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbRunStatsMotionWindowReset::RunTest(const FString& Parameters)
{
	FMotionWindow Window;
	for (int32 i = 0; i < 40; ++i)
	{
		Window.Add(5.0 + 0.016 * i, FVector2D(10.0 * 0.016 * i, 0.0), FVector2D(1.0, 0.0));
	}
	TestTrue(TEXT("full before the reset"), Window.IsFull());
	// The simulation restarts: the clock goes back and the robot is elsewhere.
	Window.Add(0.0, FVector2D(-500.0, 0.0), FVector2D(1.0, 0.0));
	TestFalse(TEXT("a clock going back starts the window again"), Window.IsFull());
	TestEqual(TEXT("one sample reads 0, not the jump"), Window.GetSpeed(), 0.0f);
	// A repeated time replaces the last sample rather than dividing by zero.
	Window.Add(0.0, FVector2D(-400.0, 0.0), FVector2D(1.0, 0.0));
	TestEqual(TEXT("a repeated time is no motion"), Window.GetSpeed(), 0.0f);
	TestTrue(TEXT("and keeps the latest position"), Window.GetPosition().Equals(FVector2D(-400.0, 0.0)));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbRunStatsFirstMotion,
	"Ramms.Curb.RunStats.WaitsForFirstMotion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbRunStatsFirstMotion::RunTest(const FString& Parameters)
{
	FRunStatsTracker Tracker;
	Tracker.Begin(14.0f, 0.0f, 0.0f, TOptional<float>(), CruiseFloor);
	FTickFeeder Feed{ Tracker };
	// Standing, then accelerating but not yet at 5 cm/s measured: none of it
	// is a stop or a minimum.
	Feed.Run(0.5f, 0.0f, 0.0f);
	Feed.Run(0.2f, 10.0f, 3.0f);
	TestFalse(TEXT("not under way below 5 cm/s"), Tracker.IsUnderWay());
	TestEqual(TEXT("no stop before first motion"), Tracker.Get().Stops, 0);
	TestEqual(TEXT("no time stopped before first motion"), Tracker.Get().SecondsBelowStopSpeed, 0.0f);

	// A tick whose window is not full does not start it either.
	FRunTick Partial;
	Partial.DeltaSeconds = TickSeconds;
	Partial.Elapsed = Feed.Elapsed += TickSeconds;
	Partial.CommandedSpeed = 30.0f;
	Partial.MeasuredSpeed = 30.0f;
	Partial.bMeasuredSpeedValid = false;
	Tracker.Tick(Partial);
	TestFalse(TEXT("a partial window does not start it"), Tracker.IsUnderWay());

	// Straight to 18 cm/s, past the floor on its first tick under way, as a
	// robot already rolling is: its rise is over at once.
	Feed.Run(1.0f, 20.0f, 18.0f);
	TestTrue(TEXT("under way at 18 cm/s"), Tracker.IsUnderWay());
	TestTrue(TEXT("and done rising, above the floor"), Tracker.HasRisen());
	TestEqual(TEXT("the least commanded speed is from first motion on"), Tracker.Get().MinCommandedSpeed, 20.0f);
	TestEqual(TEXT("the least measured speed is from first motion on"), Tracker.Get().MinMeasuredSpeed, 18.0f);
	TestEqual(TEXT("the plan's slowest step is kept"), Tracker.Get().PlannedMinSpeed, 14.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbRunStatsStandingStart,
	"Ramms.Curb.RunStats.StandingStart",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbRunStatsStandingStart::RunTest(const FString& Parameters)
{
	// From rest to a steady 30 cm/s at 60 cm/s^2, measured over the 0.3 s
	// window every 38 ms: the speed passes every value below the floor on
	// its way up, and none of them is the robot slowing.
	FRunStatsTracker Tracker;
	Tracker.Begin(14.0f, 0.0f, 0.0f, TOptional<float>(), CruiseFloor);
	FRobotFeeder Robot{ Tracker };
	Robot.Drive(0.5, 0.0);
	TestFalse(TEXT("standing is not under way"), Tracker.IsUnderWay());
	Robot.Drive(3.0, 30.0);
	const FRammsCurbRunStats& S = Tracker.Get();
	TestTrue(TEXT("under way"), Tracker.IsUnderWay());
	TestTrue(TEXT("and done rising"), Tracker.HasRisen());
	TestTrue(FString::Printf(TEXT("a standing start's least measured speed (%.2f) is at or above the floor"), S.MinMeasuredSpeed),
		S.MinMeasuredSpeed >= CruiseFloor);
	TestTrue(FString::Printf(TEXT("and within a tick's acceleration of it (%.2f)"), S.MinMeasuredSpeed),
		S.MinMeasuredSpeed <= CruiseFloor + 60.0f * FRobotFeeder::Dt + 0.01f);
	TestTrue(FString::Printf(TEXT("its least command (%.2f) too"), S.MinCommandedSpeed), S.MinCommandedSpeed >= CruiseFloor);
	TestEqual(TEXT("no stop"), S.Stops, 0);
	TestEqual(TEXT("no time stopped"), S.SecondsBelowStopSpeed, 0.0f);

	// Slowing to 4 cm/s once rolling is the least speed, though no stop.
	Robot.Drive(1.5, 4.0);
	Robot.Drive(1.5, 30.0);
	TestEqual(TEXT("slowing once rolling is the least speed"), Tracker.Get().MinMeasuredSpeed, 4.0f, 0.05f);
	TestEqual(TEXT("and the least command"), Tracker.Get().MinCommandedSpeed, 4.0f, 1e-3f);
	TestEqual(TEXT("slowing is no stop"), Tracker.Get().Stops, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbRunStatsRiseShortOfFloor,
	"Ramms.Curb.RunStats.RiseShortOfFloor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbRunStatsRiseShortOfFloor::RunTest(const FString& Parameters)
{
	// Up to 6.5 cm/s, under way but short of the floor, then back down to
	// 2.5 and up to 30: the turn back ends the rise, so the dip counts.
	{
		FRunStatsTracker Tracker;
		Tracker.Begin(14.0f, 0.0f, 0.0f, TOptional<float>(), CruiseFloor);
		FRobotFeeder Robot{ Tracker };
		Robot.Drive(0.5, 0.0);
		Robot.Drive(1.0, 6.5);
		TestTrue(TEXT("under way at 6.5 cm/s"), Tracker.IsUnderWay());
		TestFalse(TEXT("but still rising"), Tracker.HasRisen());
		Robot.Drive(1.0, 2.5);
		TestTrue(TEXT("turning back ends the rise"), Tracker.HasRisen());
		Robot.Drive(2.0, 30.0);
		TestEqual(TEXT("the dip short of the floor is the least speed"), Tracker.Get().MinMeasuredSpeed, 2.5f, 0.05f);
		TestEqual(TEXT("and the least command"), Tracker.Get().MinCommandedSpeed, 2.5f, 1e-3f);
	}
	// Up to 6.5 cm/s and held there to the end: it never finished rising, so
	// its minima run from first motion and stay below the floor.
	{
		FRunStatsTracker Tracker;
		Tracker.Begin(14.0f, 0.0f, 0.0f, TOptional<float>(), CruiseFloor);
		FRobotFeeder Robot{ Tracker };
		Robot.Drive(0.5, 0.0);
		Robot.Drive(2.0, 6.5);
		Tracker.End(Robot.Time, 0.0f, 0.0f);
		TestFalse(TEXT("never done rising"), Tracker.HasRisen());
		const float Least = Tracker.Get().MinMeasuredSpeed;
		TestTrue(FString::Printf(TEXT("the least speed (%.2f) is from first motion"), Least),
			Least >= RammsCurb::FirstMotionSpeedCmS && Least < RammsCurb::FirstMotionSpeedCmS + 1.0f);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbRunStatsStops,
	"Ramms.Curb.RunStats.Stops",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbRunStatsStops::RunTest(const FString& Parameters)
{
	FRunStatsTracker Tracker;
	Tracker.Begin(14.0f, 0.0f, 0.0f, TOptional<float>(), CruiseFloor);
	FTickFeeder Feed{ Tracker };
	Feed.Run(1.0f, 30.0f, 30.0f);

	// A dip below 2 cm/s for 0.15 s: slow, not a stop.
	Feed.Run(0.15f, 1.0f, 1.5f);
	Feed.Run(0.5f, 30.0f, 30.0f);
	TestEqual(TEXT("0.15 s below 2 cm/s is not a stop"), Tracker.Get().Stops, 0);
	TestEqual(TEXT("but its time is counted"), Tracker.Get().SecondsBelowStopSpeed, 0.15f, 1e-4f);
	TestEqual(TEXT("and its speed is the least"), Tracker.Get().MinMeasuredSpeed, 1.5f);
	TestEqual(TEXT("as is its command"), Tracker.Get().MinCommandedSpeed, 1.0f);

	// 0.3 s at 0.5 cm/s is one stop, counted once however long it lasts.
	Feed.Run(0.3f, 0.0f, 0.5f);
	TestEqual(TEXT("0.3 s below 2 cm/s is a stop"), Tracker.Get().Stops, 1);
	Feed.Run(1.0f, 0.0f, 0.0f);
	TestEqual(TEXT("one stop however long"), Tracker.Get().Stops, 1);
	Feed.Run(0.5f, 30.0f, 30.0f);
	Feed.Run(0.25f, 0.0f, -1.0f);
	TestEqual(TEXT("a second spell is a second stop"), Tracker.Get().Stops, 2);
	TestEqual(TEXT("rolling back is the least speed"), Tracker.Get().MinMeasuredSpeed, -1.0f);
	TestEqual(TEXT("time below 2 cm/s adds up"), Tracker.Get().SecondsBelowStopSpeed, 0.15f + 0.3f + 1.0f + 0.25f, 1e-3f);

	Tracker.AddPlannedStop();
	Tracker.AddHold();
	Tracker.AddReplan();
	TestEqual(TEXT("planned stops are counted"), Tracker.Get().PlannedStops, 1);
	TestEqual(TEXT("holds are counted"), Tracker.Get().Holds, 1);
	TestEqual(TEXT("re-plans are counted"), Tracker.Get().Replans, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbRunStatsAttitudeAndEnd,
	"Ramms.Curb.RunStats.AttitudeAndEnd",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbRunStatsAttitudeAndEnd::RunTest(const FString& Parameters)
{
	FRunStatsTracker Tracker;
	Tracker.Begin(14.0f, 0.5f, 9.0f, TOptional<float>(4.0f), CruiseFloor);
	TestEqual(TEXT("the planned margin is kept"), Tracker.Get().MinPlannedMarginCm, 4.0f);
	TestEqual(TEXT("the planned tilt is kept"), Tracker.Get().PlannedMaxTiltDeg, 0.5f);
	TestEqual(TEXT("the planned time is kept"), Tracker.Get().PlannedCrossingSeconds, 9.0f);

	FTickFeeder Feed{ Tracker };
	Feed.Run(1.0f, 30.0f, 30.0f);
	// The legs start moving 1 s in.
	Feed.Run(0.5f, 30.0f, 30.0f, /*bLegsMoved=*/true);

	auto Tick = [&](float Pitch, float Roll, bool bPlanned, TOptional<float> Swing, TOptional<float> Margin, float LegError) {
		FRunTick T;
		T.DeltaSeconds = TickSeconds;
		T.Elapsed = Feed.Elapsed += TickSeconds;
		T.CommandedSpeed = 30.0f;
		T.MeasuredSpeed = 30.0f;
		T.bMeasuredSpeedValid = true;
		T.bLegsMoved = true;
		T.PitchDeg = Pitch;
		T.RollDeg = Roll;
		T.bAttitudePlanned = bPlanned;
		T.SwingClearanceCm = Swing;
		T.MeasuredMarginCm = Margin;
		T.LegErrorCm = LegError;
		T.LateralDriftCm = 1.5f;
		T.YawChangeDeg = -0.4f;
		Tracker.Tick(T);
	};
	// Pitched 2.8 deg in a stance the gait does not plan: tilt, not error.
	Tick(2.8f, 0.1f, false, {}, {}, 0.2f);
	Tick(-1.2f, -3.1f, true, 6.0f, 9.0f, 1.3f);
	Tick(0.4f, 0.2f, true, 2.5f, {}, 0.4f);
	Tick(0.1f, 0.0f, true, {}, 12.0f, 0.1f);
	const FRammsCurbRunStats& S = Tracker.Get();
	TestEqual(TEXT("largest |pitch|"), S.MaxAbsPitchDeg, 2.8f);
	TestEqual(TEXT("largest |roll|"), S.MaxAbsRollDeg, 3.1f);
	// The largest inclination is the most inclined tick's (-1.2 pitch, -3.1
	// roll: 3.32 deg), not the larger axis of either tick, nor the largest
	// pitch and roll taken together (2.8 and 3.1: 4.18).
	TestEqual(TEXT("largest inclination, of the most inclined tick"), S.MaxInclinationDeg,
		static_cast<float>(RammsCurb::InclinationDeg(-1.2, -3.1)), 1e-5f);
	TestTrue(TEXT("more than its larger axis"), S.MaxInclinationDeg > 3.3f && S.MaxInclinationDeg < 3.35f);
	TestEqual(TEXT("attitude error only where an attitude is planned"), S.MaxAttitudeErrorDeg, 3.1f);
	TestEqual(TEXT("least swing clearance, from the ticks that measured one"), S.MinSwingClearanceCm, 2.5f);
	TestEqual(TEXT("least measured margin, from the ticks that measured one"), S.MinMeasuredMarginCm, 9.0f);
	TestEqual(TEXT("largest leg error"), S.MaxLegErrorCm, 1.3f);
	TestEqual(TEXT("drift as of the last tick"), S.LateralDriftCm, 1.5f);
	TestEqual(TEXT("yaw change as of the last tick"), S.YawChangeDeg, -0.4f);

	Tracker.End(Feed.Elapsed, 0.3f, -0.1f);
	TestEqual(TEXT("final pitch at the end"), Tracker.Get().FinalPitchDeg, 0.3f);
	TestEqual(TEXT("final roll at the end"), Tracker.Get().FinalRollDeg, -0.1f);
	TestEqual(TEXT("crossing time from the first leg move"), Tracker.Get().CrossingSeconds, 0.53f, 1e-3f);

	// Nothing after the end counts: the hand-back re-tilts and stops the robot.
	Tick(8.0f, 8.0f, true, -1.0f, -1.0f, 9.0f);
	Feed.Run(1.0f, 0.0f, 0.0f, true);
	Tracker.AddPlannedStop();
	Tracker.End(Feed.Elapsed, 5.0f, 5.0f);
	TestEqual(TEXT("no tilt after the end"), Tracker.Get().MaxAbsPitchDeg, 2.8f);
	TestTrue(TEXT("nor inclination"), Tracker.Get().MaxInclinationDeg < 3.35f);
	TestEqual(TEXT("no stop after the end"), Tracker.Get().Stops, 0);
	TestEqual(TEXT("no planned stop after the end"), Tracker.Get().PlannedStops, 0);
	TestEqual(TEXT("the first end stands"), Tracker.Get().FinalPitchDeg, 0.3f);
	TestEqual(TEXT("and so does its time"), Tracker.Get().CrossingSeconds, 0.53f, 1e-3f);

	// The next crossing starts clean.
	Tracker.Begin(10.0f, 0.0f, 0.0f, TOptional<float>(), CruiseFloor);
	TestEqual(TEXT("Begin clears the stats"), Tracker.Get().MaxAbsPitchDeg, 0.0f);
	TestEqual(TEXT("Begin clears an unplanned margin to 0"), Tracker.Get().MinPlannedMarginCm, 0.0f);
	TestFalse(TEXT("Begin clears the end"), Tracker.HasEnded());
	TestFalse(TEXT("and first motion"), Tracker.IsUnderWay());

	// A tilt fault ends the crossing on a tick the stats never saw: the tilt
	// that stopped it is still the largest.
	FTickFeeder Again{ Tracker };
	Again.Run(0.5f, 30.0f, 30.0f);
	Tracker.End(Again.Elapsed, -12.5f, 4.0f);
	TestEqual(TEXT("the faulting pitch is the largest"), Tracker.Get().MaxAbsPitchDeg, 12.5f);
	TestEqual(TEXT("and the faulting roll"), Tracker.Get().MaxAbsRollDeg, 4.0f);
	TestEqual(TEXT("and the faulting inclination"), Tracker.Get().MaxInclinationDeg, static_cast<float>(RammsCurb::InclinationDeg(-12.5, 4.0)),
		1e-5f);

	// Standing still is no reason to miss a tilt, a lagging leg or a poor
	// margin: those count from Begin, only the speeds wait for motion.
	FRunStatsTracker Standing;
	Standing.Begin(10.0f, 0.0f, 0.0f, TOptional<float>(), CruiseFloor);
	FRunTick Still;
	Still.DeltaSeconds = TickSeconds;
	Still.Elapsed = TickSeconds;
	Still.bMeasuredSpeedValid = true;
	Still.PitchDeg = 9.5f;
	Still.RollDeg = -2.0f;
	Still.LegErrorCm = 1.7f;
	Still.MeasuredMarginCm = 4.5f;
	Standing.Tick(Still);
	TestFalse(TEXT("standing is not under way"), Standing.IsUnderWay());
	TestEqual(TEXT("a tilt before first motion counts"), Standing.Get().MaxAbsPitchDeg, 9.5f);
	TestEqual(TEXT("as does its roll"), Standing.Get().MaxAbsRollDeg, 2.0f);
	TestEqual(TEXT("and its inclination"), Standing.Get().MaxInclinationDeg, static_cast<float>(RammsCurb::InclinationDeg(9.5, -2.0)), 1e-5f);
	TestEqual(TEXT("a leg error before first motion counts"), Standing.Get().MaxLegErrorCm, 1.7f);
	TestEqual(TEXT("a margin before first motion counts"), Standing.Get().MinMeasuredMarginCm, 4.5f);
	TestEqual(TEXT("but no speed is counted"), Standing.Get().MinMeasuredSpeed, 0.0f);
	TestEqual(TEXT("nor time stopped"), Standing.Get().SecondsBelowStopSpeed, 0.0f);
	Standing.End(0.1, -11.0f, 1.0f);
	TestEqual(TEXT("an end before first motion keeps its attitude as final"), Standing.Get().FinalPitchDeg, -11.0f);
	TestEqual(TEXT("and as the largest tilt"), Standing.Get().MaxAbsPitchDeg, 11.0f);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
