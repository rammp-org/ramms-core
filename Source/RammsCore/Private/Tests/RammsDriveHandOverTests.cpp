// Tests for changing drive mode on the move (Private/RammsDriveHandOver).
//
// The chassis motion measured from its pose, the wait for the outgoing mode to
// take off what the incoming one cannot carry, the stance ramp, the rate-limited
// twist, what a differential drive carries and its wheel speeds, the holonomic
// command for a motion, and a carried motion's hold -- each on made-up numbers --
// and the switch itself (FModeSwitcher) on a made-up robot with treads on a
// linkage and corner wheels on a crank: on the move, at rest, refused, waiting
// in its first stage, cancelled part way, and from modes with no stance.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "RammsDriveHandOver.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	using RammsDrive::EHandOverGate;
	using RammsDrive::FCarry;
	using RammsDrive::FChassisMotionWindow;
	using RammsDrive::FHandOverGate;
	using RammsDrive::FHandOverLimits;
	using RammsDrive::FStanceChannel;

	/** A chassis on the ground plane: its centre (cm), heading (rad), and
	 *  motion in its own frame, stepped forward in time. */
	struct FToyChassis
	{
		FVector2D		  Centre = FVector2D(100.0, -50.0);
		double			  Yaw = 0.3;
		FRammsDriveMotion Motion;

		FVector2D Forward() const { return FVector2D(FMath::Cos(Yaw), FMath::Sin(Yaw)); }
		FVector2D Right() const { return FVector2D(-FMath::Sin(Yaw), FMath::Cos(Yaw)); }

		void Step(double Dt)
		{
			Centre += (Forward() * Motion.ForwardCmPerSecond + Right() * Motion.RightCmPerSecond) * Dt;
			Yaw += FMath::DegreesToRadians(static_cast<double>(Motion.YawRateDegPerSecond)) * Dt;
		}
	};
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHandOverMotionWindow,
	"Ramms.Drive.HandOver.MotionWindow",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHandOverMotionWindow::RunTest(const FString& Parameters)
{
	FChassisMotionWindow Window(0.15, 0.04);
	FToyChassis			 Chassis;
	Chassis.Motion = FRammsDriveMotion(30.0f, -10.0f, 20.0f);
	double T = 5.0;
	TestFalse(TEXT("no motion before any pose"), Window.IsValid());
	for (int32 i = 0; i < 30; ++i)
	{
		Window.Add(T, Chassis.Centre, Chassis.Forward() * 3.0);
		if (i == 1)
		{
			TestFalse(TEXT("two poses 4 ms apart do not span the least span"), Window.IsValid());
			TestTrue(TEXT("...and read as no motion"), Window.GetMotion().IsNearlyZero(1e-6f, 1e-6f));
		}
		Chassis.Step(0.004);
		T += 0.004;
	}
	TestTrue(TEXT("valid once the poses span the window"), Window.IsValid());
	FRammsDriveMotion M = Window.GetMotion();
	// A steady turn: the chord between the window's ends lies along the mean
	// heading, so the reading is exact up to the chord's shortening (well under
	// 0.1% over 0.15 s at 20 deg/s).
	TestEqual(TEXT("forward speed in the chassis' own frame"), M.ForwardCmPerSecond, 30.0f, 0.05f);
	TestEqual(TEXT("sideways speed: negative is to the left"), M.RightCmPerSecond, -10.0f, 0.05f);
	TestEqual(TEXT("yaw rate: positive turns right (towards +Y)"), M.YawRateDegPerSecond, 20.0f, 0.01f);

	// The heading through +-180 deg reads as the turn it is.
	FChassisMotionWindow Wrap(0.15, 0.04);
	FToyChassis			 Turning;
	Turning.Yaw = PI - 0.05;
	Turning.Motion = FRammsDriveMotion(0.0f, 0.0f, 45.0f);
	for (int32 i = 0; i < 60; ++i)
	{
		Wrap.Add(i * 0.004, Turning.Centre, Turning.Forward());
		Turning.Step(0.004);
	}
	TestEqual(TEXT("a turn through 180 deg is not a jump of 360"), Wrap.GetMotion().YawRateDegPerSecond, 45.0f, 0.01f);

	// The same physics step seen twice adds nothing; time going back starts
	// again; a degenerate heading is ignored.
	// (The last pose went in at T - 0.004.)
	const double Back = T - 0.1;
	Window.Add(Back, Chassis.Centre + FVector2D(500.0, 0.0), Chassis.Forward());
	TestFalse(TEXT("a pose earlier than the last (a reset) starts the window again"), Window.IsValid());
	TestEqual(TEXT("...from that pose"), Window.GetLatestTime(), Back, 1e-9);
	Window.Add(T, Chassis.Centre, FVector2D::ZeroVector);
	TestEqual(TEXT("a pose with no heading is not taken"), Window.GetLatestTime(), Back, 1e-9);
	Window.Add(T, Chassis.Centre, Chassis.Forward());
	Window.Add(T, Chassis.Centre + FVector2D(10.0, 0.0), Chassis.Forward());
	TestEqual(TEXT("a second pose at the same time is not taken"), Window.GetLatestTime(), T, 1e-9);

	// The window keeps only the last WindowSeconds: a stop shows within it.
	FChassisMotionWindow Stopping(0.15, 0.04);
	FToyChassis			 Car;
	Car.Motion = FRammsDriveMotion(40.0f, 0.0f, 0.0f);
	for (int32 i = 0; i < 100; ++i)
	{
		if (i == 50)
		{
			Car.Motion = FRammsDriveMotion();
		}
		Stopping.Add(i * 0.004, Car.Centre, Car.Forward());
		Car.Step(0.004);
	}
	TestEqual(TEXT("0.2 s after stopping, the window reads at rest"), Stopping.GetMotion().ForwardCmPerSecond, 0.0f, 1e-3f);

	// A pose read 30 ms after the time it is stamped with -- the physics clock
	// and the joints read at different instants -- as the newest: the
	// difference of the window's ends would read 30 * 0.03 / 0.15 = 6 cm/s
	// fast; the fit through every pose moves by a share of that.
	FChassisMotionWindow Jitter(0.15, 0.04);
	FToyChassis			 Steady;
	Steady.Motion = FRammsDriveMotion(30.0f, 0.0f, 0.0f);
	double TJ = 0.0;
	for (int32 i = 0; i < 38; ++i)
	{
		Jitter.Add(TJ, Steady.Centre, Steady.Forward());
		Steady.Step(0.004);
		TJ += 0.004;
	}
	Jitter.Add(TJ, Steady.Centre + Steady.Forward() * (30.0 * 0.03), Steady.Forward());
	const double JitterError = Jitter.GetMotion().ForwardCmPerSecond - 30.0;
	TestTrue(*FString::Printf(TEXT("one mistimed pose moves the reading under 1.5 cm/s (%.2f)"), JitterError), FMath::Abs(JitterError) < 1.5);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHandOverGate,
	"Ramms.Drive.HandOver.Gate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHandOverGate::RunTest(const FString& Parameters)
{
	FHandOverLimits Limits;
	Limits.MaxSidewaysCmS = 2.0;
	Limits.MaxYawExcessDegS = 3.0;
	Limits.SustainSeconds = 0.1;
	Limits.TimeoutSeconds = 3.0;
	const FRammsDriveMotion Carried(25.0f, 0.0f, 10.0f);

	TestEqual(TEXT("the excess is the sideways speed and the yaw rate past what is carried"),
		RammsDrive::MotionExcess(FRammsDriveMotion(25.0f, -3.0f, 14.0f), Carried), FVector2D(3.0, 4.0));
	TestFalse(TEXT("forward speed is never an excess"), RammsDrive::NeedsHandOver(FRammsDriveMotion(80.0f, 1.9f, 12.9f), Carried, Limits));
	TestTrue(TEXT("sideways past the limit needs a hand-over"), RammsDrive::NeedsHandOver(FRammsDriveMotion(25.0f, 2.1f, 10.0f), Carried, Limits));
	TestTrue(TEXT("...either way"), RammsDrive::NeedsHandOver(FRammsDriveMotion(25.0f, -2.1f, 10.0f), Carried, Limits));
	TestTrue(TEXT("so does a yaw rate past what is carried"), RammsDrive::NeedsHandOver(FRammsDriveMotion(25.0f, 0.0f, 13.1f), Carried, Limits));
	TestTrue(TEXT("at rest: under 2 cm/s each way and 3 deg/s"), RammsDrive::IsAtRest(FRammsDriveMotion(1.9f, -1.9f, 2.9f)));
	TestFalse(TEXT("moving forward is not at rest"), RammsDrive::IsAtRest(FRammsDriveMotion(-2.1f, 0.0f, 0.0f)));
	TestFalse(TEXT("nor is sliding sideways"), RammsDrive::IsAtRest(FRammsDriveMotion(0.0f, 2.1f, 0.0f)));
	TestFalse(TEXT("nor is turning on the spot"), RammsDrive::IsAtRest(FRammsDriveMotion(0.0f, 0.0f, 3.1f)));

	// Ready only after SustainSeconds within the limits; a reading past them
	// starts the count again.
	FHandOverGate Gate;
	const double  Dt = 0.01;
	EHandOverGate State = EHandOverGate::Waiting;
	int32		  Ticks = 0;
	for (; Ticks < 5; ++Ticks)
	{
		State = Gate.Step(FRammsDriveMotion(25.0f, 1.0f, 10.0f), Carried, Limits, Dt);
	}
	TestTrue(TEXT("0.05 s within the limits is not yet enough"), State == EHandOverGate::Waiting);
	State = Gate.Step(FRammsDriveMotion(25.0f, 2.5f, 10.0f), Carried, Limits, Dt);
	TestTrue(TEXT("a reading past them..."), State == EHandOverGate::Waiting);
	TestEqual(TEXT("...starts the count again"), Gate.Within, 0.0, 1e-12);
	int32 ToReady = 0;
	do
	{
		State = Gate.Step(FRammsDriveMotion(25.0f, -1.5f, 9.0f), Carried, Limits, Dt);
		++ToReady;
	}
	while (State == EHandOverGate::Waiting && ToReady < 1000);
	TestTrue(TEXT("ready after holding within the limits"), State == EHandOverGate::Ready);
	TestEqual(TEXT("...for SustainSeconds"), ToReady, 10);

	// Never within them: refused at the timeout, not before.
	FHandOverGate Stuck;
	double		  T = 0.0;
	do
	{
		State = Stuck.Step(FRammsDriveMotion(25.0f, 5.0f, 10.0f), Carried, Limits, Dt);
		T += Dt;
	}
	while (State == EHandOverGate::Waiting && T < 10.0);
	TestTrue(TEXT("still sideways at the timeout: refused"), State == EHandOverGate::TimedOut);
	TestEqual(TEXT("...at TimeoutSeconds"), T, 3.0, 0.011);
	Stuck.Reset();
	TestEqual(TEXT("reset clears the clock"), Stuck.Elapsed, 0.0, 1e-12);

	// No measurement is never within the limits: a window that read zero
	// motion for want of poses would otherwise open the gate on no data.
	FHandOverGate Blind;
	T = 0.0;
	do
	{
		State = Blind.Step(FRammsDriveMotion(), Carried, Limits, Dt, /*bMeasured=*/false);
		T += Dt;
	}
	while (State == EHandOverGate::Waiting && T < 10.0);
	TestTrue(TEXT("unmeasured: never ready, refused at the timeout"), State == EHandOverGate::TimedOut);

	// Only what the outgoing mode drives is its to take off.
	const FRammsDriveMotion Sliding(25.0f, 4.0f, 0.0f);
	const FRammsDriveMotion NoSideways(25.0f, 0.0f, 0.0f);
	TestTrue(TEXT("sideways under a mode that drives it, into one that cannot: a hand-over"),
		RammsDrive::NeedsHandOverBetween(Sliding, Sliding, NoSideways, Limits));
	TestFalse(TEXT("a slide under a mode that cannot move sideways either is not its to take off"),
		RammsDrive::NeedsHandOverBetween(Sliding, NoSideways, NoSideways, Limits));
	TestFalse(TEXT("into a mode that carries it: none"), RammsDrive::NeedsHandOverBetween(Sliding, Sliding, Sliding, Limits));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHandOverStanceRamp,
	"Ramms.Drive.HandOver.StanceRamp",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHandOverStanceRamp::RunTest(const FString& Parameters)
{
	// A made-up stance change: a linkage lowered 9 cm and moved 4 cm forward,
	// two cranks turned 1.4 and 0.3 rad.
	TArray<FStanceChannel> Channels;
	Channels.Add({ FName("linkage.left.height"), false, 11.0, 2.0 });
	Channels.Add({ FName("linkage.left.translation"), false, 0.0, 4.0 });
	Channels.Add({ FName("crank_a"), true, 1.0, -0.4 });
	Channels.Add({ FName("crank_b"), true, 1.0, 0.7 });

	TestEqual(TEXT("no limits and no least time: a jump"), RammsDrive::StanceRampSeconds(Channels, 0.0, 0.0, 0.0), 0.0, 1e-12);
	TestEqual(TEXT("nothing to move: a jump whatever the least time"),
		RammsDrive::StanceRampSeconds({ FStanceChannel{ FName("crank_a"), true, 0.5, 0.5 } }, 1.0, 15.0, 2.0), 0.0, 1e-12);
	TestEqual(TEXT("the least time when the rates allow it"), RammsDrive::StanceRampSeconds(Channels, 2.0, 15.0, 2.0), 2.0, 1e-9);
	// At the S-curve's steepest a channel moves 1.5 times its mean rate.
	const double Seconds = RammsDrive::StanceRampSeconds(Channels, 0.5, 15.0, 1.0);
	TestEqual(TEXT("the largest crank turn at 1 rad/s at its steepest sets the time"), Seconds, 1.5 * 1.4 / 1.0, 1e-9);
	TestEqual(TEXT("the linkage at 5 cm/s at its steepest sets it instead"), RammsDrive::StanceRampSeconds(Channels, 0.5, 5.0, 1.0), 1.5 * 9.0 / 5.0,
		1e-9);

	// Step it: every channel starts where it is, never outruns its limit,
	// moves one way only, and all arrive together.
	const double   Dt = 0.002;
	TArray<double> Last;
	for (const FStanceChannel& C : Channels)
	{
		Last.Add(C.From);
		TestEqual(*FString::Printf(TEXT("%s starts where it is"), *C.Id.ToString()), RammsDrive::StanceRampValue(C, 0.0, Seconds), C.From, 1e-12);
	}
	double FastestCrank = 0.0, FastestLinkage = 0.0;
	bool   bOneWay = true;
	for (double T = Dt; T <= Seconds + 1e-9; T += Dt)
	{
		for (int32 i = 0; i < Channels.Num(); ++i)
		{
			const double V = RammsDrive::StanceRampValue(Channels[i], T, Seconds);
			const double Rate = (V - Last[i]) / Dt;
			double&		 Fastest = Channels[i].bMotor ? FastestCrank : FastestLinkage;
			Fastest = FMath::Max(Fastest, FMath::Abs(Rate));
			bOneWay &= Rate * (Channels[i].To - Channels[i].From) >= -1e-9;
			Last[i] = V;
		}
	}
	TestTrue(TEXT("no motor turns faster than its limit"), FastestCrank <= 1.0 + 1e-3);
	TestTrue(TEXT("...and the largest turn reaches it (the time is no longer than it needs)"), FastestCrank >= 1.0 - 2e-3);
	TestTrue(TEXT("no linkage axis moves faster than its limit"), FastestLinkage <= 15.0);
	TestTrue(TEXT("every channel moves one way only: no overshoot"), bOneWay);
	for (const FStanceChannel& C : Channels)
	{
		TestEqual(*FString::Printf(TEXT("%s ends on the stance"), *C.Id.ToString()), RammsDrive::StanceRampValue(C, Seconds, Seconds), C.To, 1e-12);
		TestEqual(*FString::Printf(TEXT("%s stays on it"), *C.Id.ToString()), RammsDrive::StanceRampValue(C, Seconds + 5.0, Seconds), C.To, 1e-12);
		TestEqual(*FString::Printf(TEXT("%s is half way at half time (one S-curve for all)"), *C.Id.ToString()),
			RammsDrive::StanceRampValue(C, 0.5 * Seconds, Seconds), 0.5 * (C.From + C.To), 1e-9);
	}
	TestEqual(TEXT("a zero-length change is the stance at once"), RammsDrive::StanceRampValue(Channels[2], 0.0, 0.0), Channels[2].To, 1e-12);
	TestEqual(TEXT("the S-curve starts flat"), (RammsDrive::SmoothStep(1e-4) - RammsDrive::SmoothStep(0.0)) / 1e-4, 0.0, 1e-3);
	TestEqual(TEXT("...is steepest at half way, 1.5 times its mean"), (RammsDrive::SmoothStep(0.5 + 1e-5) - RammsDrive::SmoothStep(0.5 - 1e-5)) / 2e-5,
		1.5, 1e-6);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHandOverStanceStages,
	"Ramms.Drive.HandOver.StanceStages",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHandOverStanceStages::RunTest(const FString& Parameters)
{
	// A made-up robot whose main wheels hang from two linkages and whose
	// corner legs are on two cranks: wheels down before wheels up.
	const auto Make = [](bool bDown) {
		TArray<FStanceChannel> C;
		C.Add({ FName("linkage.l.height"), false, bDown ? 12.0 : 3.0, bDown ? 3.0 : 12.0, true });
		C.Add({ FName("linkage.r.height"), false, bDown ? 12.0 : 3.0, bDown ? 3.0 : 12.0, true });
		C.Add({ FName("linkage.l.translation"), false, bDown ? 0.0 : 5.0, bDown ? 5.0 : 0.0, false });
		C.Add({ FName("crank_front"), true, bDown ? 1.0 : -0.6, bDown ? -0.6 : 1.0, false });
		C.Add({ FName("crank_rear"), true, bDown ? 1.0 : 0.6, bDown ? 0.6 : 1.0, false });
		return C;
	};
	TArray<FStanceChannel> First, Second;
	RammsDrive::SplitStanceStages(Make(true), First, Second);
	TestEqual(TEXT("main wheels going down: the linkages first (heights and fore/aft)"), First.Num(), 3);
	TestTrue(TEXT("...none of them a motor"), First.FindByPredicate([](const FStanceChannel& C) { return C.bMotor; }) == nullptr);
	TestEqual(TEXT("...the corner legs after"), Second.Num(), 2);
	TestTrue(TEXT("...all of them motors"), Second.FindByPredicate([](const FStanceChannel& C) { return !C.bMotor; }) == nullptr);

	RammsDrive::SplitStanceStages(Make(false), First, Second);
	TestEqual(TEXT("main wheels going up: the corner legs first"), First.Num(), 2);
	TestTrue(TEXT("...all of them motors"), First.FindByPredicate([](const FStanceChannel& C) { return !C.bMotor; }) == nullptr);
	TestEqual(TEXT("...the linkages after"), Second.Num(), 3);

	// No height moving: one stage. And on the whole: a linkage lowered more
	// than another is raised counts as down.
	TArray<FStanceChannel> Level = Make(true);
	Level[0].To = Level[0].From;
	Level[1].To = Level[1].From;
	RammsDrive::SplitStanceStages(Level, First, Second);
	TestEqual(TEXT("no linkage height moving: all of it at once"), First.Num(), 5);
	TestEqual(TEXT("...and nothing after"), Second.Num(), 0);
	TArray<FStanceChannel> Mixed = Make(true);
	Mixed[1].From = 3.0;
	Mixed[1].To = 5.0;
	RammsDrive::SplitStanceStages(Mixed, First, Second);
	TestTrue(TEXT("down 9 cm and up 2 on the whole is down: the linkages first"),
		First.Num() == 3 && First.FindByPredicate([](const FStanceChannel& C) { return C.bMotor; }) == nullptr);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHandOverRampMotion,
	"Ramms.Drive.HandOver.RampMotion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHandOverRampMotion::RunTest(const FString& Parameters)
{
	const FRammsDriveMotion From(30.0f, 12.0f, 20.0f);
	const FRammsDriveMotion To(30.0f, 0.0f, 0.0f);
	const FRammsDriveMotion One = RammsDrive::RampMotion(From, To, 60.0, 90.0, 0.05);
	TestEqual(TEXT("the sideways speed comes off by the acceleration times the step"), One.RightCmPerSecond, 9.0f, 1e-4f);
	TestEqual(TEXT("the forward speed it was not asked to change stays"), One.ForwardCmPerSecond, 30.0f, 1e-4f);
	TestEqual(TEXT("the yaw rate by its own"), One.YawRateDegPerSecond, 15.5f, 1e-4f);
	const FRammsDriveMotion Diagonal = RammsDrive::RampMotion(FRammsDriveMotion(0.0f, 0.0f, 0.0f), FRammsDriveMotion(30.0f, 40.0f, 0.0f), 50.0, 0.0, 0.1);
	TestEqual(TEXT("velocity along the straight way: 5 cm/s of a 3-4-5"), Diagonal.ForwardCmPerSecond, 3.0f, 1e-4f);
	TestEqual(TEXT("...forward and sideways together"), Diagonal.RightCmPerSecond, 4.0f, 1e-4f);
	TestTrue(TEXT("a limit of 0 does not limit"), RammsDrive::MotionsEqual(RammsDrive::RampMotion(From, To, 0.0, 0.0, 0.01), To));
	FRammsDriveMotion M = From;
	int32			  Ticks = 0;
	for (; Ticks < 1000 && !RammsDrive::MotionsEqual(M, To); ++Ticks)
	{
		M = RammsDrive::RampMotion(M, To, 60.0, 90.0, 0.01);
	}
	TestTrue(TEXT("the ramp reaches its target and stops there"), RammsDrive::MotionsEqual(M, To));
	TestEqual(TEXT("...in the time the slower term needs (20 deg/s at 90 deg/s^2)"), Ticks, 23);
	TestTrue(TEXT("a step of no time changes nothing"), RammsDrive::MotionsEqual(RammsDrive::RampMotion(From, To, 60.0, 90.0, 0.0), From));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHandOverKinematics,
	"Ramms.Drive.HandOver.Kinematics",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHandOverKinematics::RunTest(const FString& Parameters)
{
	// A differential drive carries forward and turning, not sideways.
	const FRammsDriveMotion M(28.0f, -6.0f, 120.0f);
	const FRammsDriveMotion C = RammsDrive::DifferentialCarriable(M, 90.0);
	TestEqual(TEXT("differential: the forward speed carried as it is"), C.ForwardCmPerSecond, 28.0f);
	TestEqual(TEXT("...no sideways speed"), C.RightCmPerSecond, 0.0f);
	TestEqual(TEXT("...the yaw rate up to what it turns"), C.YawRateDegPerSecond, 90.0f);
	TestEqual(TEXT("...either way"), RammsDrive::DifferentialCarriable(FRammsDriveMotion(0.0f, 0.0f, -95.0f), 90.0).YawRateDegPerSecond, -90.0f);
	TestEqual(TEXT("...and any yaw rate with no limit"), RammsDrive::DifferentialCarriable(M, 0.0).YawRateDegPerSecond, 120.0f);

	// Its wheels: the outer one of a right turn is the left, the two average to
	// the forward speed, and differ by the turn times the track.
	const FVector2D Wheels = RammsDrive::DifferentialWheelSpeeds(FRammsDriveMotion(20.0f, 0.0f, 30.0f), 50.0);
	TestTrue(TEXT("a right turn runs the left wheel faster"), Wheels.X > Wheels.Y);
	TestEqual(TEXT("the wheels average to the forward speed"), 0.5 * (Wheels.X + Wheels.Y), 20.0, 1e-9);
	TestEqual(TEXT("...and differ by the turn times the track"), Wheels.X - Wheels.Y, FMath::DegreesToRadians(30.0) * 50.0, 1e-9);
	TestEqual(TEXT("sideways speed does not reach the wheels"), RammsDrive::DifferentialWheelSpeeds(FRammsDriveMotion(20.0f, 9.0f, 0.0f), 50.0),
		FVector2D(20.0, 20.0));

	// A holonomic command for a motion, and back.
	const FVector Command = RammsDrive::HolonomicCommand(FRammsDriveMotion(30.0f, -15.0f, 45.0f), 60.0, 90.0);
	TestEqual(TEXT("holonomic: a command scaled by full stick"), Command, FVector(0.5, -0.25, 0.5));
	TestEqual(TEXT("...clamped to full stick"), RammsDrive::HolonomicCommand(FRammsDriveMotion(90.0f, 0.0f, -200.0f), 60.0, 90.0), FVector(1.0, 0.0, -1.0));
	TestTrue(TEXT("...and back to the motion"),
		RammsDrive::MotionsEqual(RammsDrive::HolonomicMotion(Command, 60.0, 90.0), FRammsDriveMotion(30.0f, -15.0f, 45.0f)));
	TestEqual(TEXT("no full stick: no command"), RammsDrive::HolonomicCommand(FRammsDriveMotion(30.0f, 0.0f, 0.0f), 0.0, 0.0), FVector::ZeroVector);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHandOverCarry,
	"Ramms.Drive.HandOver.Carry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHandOverCarry::RunTest(const FString& Parameters)
{
	const FRammsDriveMotion M(25.0f, 0.0f, 0.0f);
	FCarry					Carry;
	Carry.Begin(M, 1.2, /*bOperatorCommanding=*/true);
	TestTrue(TEXT("held while its operator drives"), Carry.bActive && Carry.bHolding);
	double T = 0.0;
	while (Carry.bHolding && T < 5.0)
	{
		Carry.Step(0.01, true);
		T += 0.01;
	}
	TestEqual(TEXT("...for the hold it was handed"), T, 1.2, 0.011);
	TestTrue(TEXT("still carried after the hold, for the blend"), Carry.bActive);
	Carry.Step(0.25, true);
	TestEqual(TEXT("the blend's clock runs from the hold's end"), Carry.SinceHold, 0.25, 1e-9);
	TestEqual(TEXT("the operator's share of the blend"), RammsDrive::CarryBlend(Carry.SinceHold, 1.0), RammsDrive::SmoothStep(0.25), 1e-12);
	TestEqual(TEXT("...all of it once the blend is over"), RammsDrive::CarryBlend(1.5, 1.0), 1.0, 1e-12);
	TestEqual(TEXT("...at once with no blend"), RammsDrive::CarryBlend(0.0, 0.0), 1.0, 1e-12);

	FCarry Released;
	Released.Begin(M, 1.2, true);
	Released.Step(0.3, true);
	Released.Step(0.01, false);
	TestFalse(TEXT("the hold ends as soon as the operator lets go"), Released.bHolding);
	TestTrue(TEXT("...and gives the motion up from there"), Released.bActive && Released.SinceHold == 0.0);

	FCarry Nobody;
	Nobody.Begin(M, 1.2, /*bOperatorCommanding=*/false);
	TestFalse(TEXT("nobody driving: no hold at all"), Nobody.bHolding);
	FCarry NoTime;
	NoTime.Begin(M, 0.0, true);
	TestFalse(TEXT("no stance change: no hold"), NoTime.bHolding);
	NoTime.End();
	TestFalse(TEXT("ended"), NoTime.bActive);
	return true;
}

// --- the switch itself, on a made-up robot -------------------------------------

namespace
{
	using RammsDrive::ESwitchPhase;
	using RammsDrive::FModeSwitcher;
	using RammsDrive::FMotionSample;
	using RammsDrive::FSwitchConfig;

	/**
	 * A made-up robot for FModeSwitcher. Its main wheels (treads) hang from a
	 * linkage at height L -- on the ground at or below TreadTouchL -- and its
	 * corner wheels from a crank at angle K -- on the ground at or above
	 * CornerTouchK. Four modes: a differential drive on the treads (its stance
	 * puts them down and lifts the corners; it cannot move sideways), a
	 * holonomic drive on the corners (the other way round; it carries
	 * anything), a curb mode (no stance, cannot keep driving through a
	 * hand-over, cannot move sideways) and a low-level mode (no stance, drives
	 * nothing, cannot hand over, and withdraws the linkage from what a stance
	 * can command while it is active). The active mode drives its operator's
	 * stick, within any bound and not at all while held; the chassis follows
	 * on a 60 cm/s^2 ramp and a 0.12 s lag, plus whatever a slope slides it,
	 * and is measured as the selector measures it (FChassisMotionWindow).
	 */
	class FToyRobot final : public RammsDrive::ISwitchHost
	{
	public:
		enum : int32
		{
			Diff = 0,
			Holo = 1,
			Curb = 2,
			Low = 3,
			NumModes = 4
		};
		static constexpr double TreadsDownL = 12.0;
		static constexpr double TreadsUpL = 16.0;
		static constexpr double TreadTouchL = 13.0;
		static constexpr double CornersUpK = 0.0;
		static constexpr double CornersDownK = 1.0;
		static constexpr double CornerTouchK = 0.5;

		double L = TreadsUpL;
		double K = CornersDownK;
		int32  Active = Holo;
		int32  BoundTo[NumModes] = { INDEX_NONE, INDEX_NONE, INDEX_NONE, INDEX_NONE };
		bool   Held[NumModes] = { false, false, false, false };
		/** A mode stood down whose wheels are held (HoldOutgoingWheels). */
		bool			  OutgoingHeld[NumModes] = { false, false, false, false };
		FRammsDriveMotion Stick;
		FRammsDriveMotion Slide;
		bool			  bMeasured = true;
		int32			  Activations[NumModes] = { 0, 0, 0, 0 };
		int32			  Applied[NumModes] = { 0, 0, 0, 0 };
		int32			  StanceCommands = 0;
		/** Linkage heights a ramp commanded (not ApplyStance's). */
		TArray<double> RampedL;
		/** The forward speed handed over at the last activation, if any. */
		double SeededForward = -1.0;
		double T = 0.0;

		/** Ticks on which a mode drove sideways with the treads on the ground,
		 *  and on which the active mode drove its own wheels in the air. */
		int32 SidewaysOnTreads = 0;
		int32 DrivenInAir = 0;

		FToyRobot()
		{
			// Some poses behind it, so the window is full from the start.
			for (int32 i = 0; i < 50; ++i)
			{
				Advance(0.004);
			}
		}

		bool TreadsDown() const { return L <= TreadTouchL + 1e-9; }
		bool CornersDown() const { return K >= CornerTouchK - 1e-9; }
		bool OwnWheelsDown(int32 Mode) const { return Mode == Holo ? CornersDown() : (Mode == Diff ? TreadsDown() : true); }

		/** What the active mode drives now. */
		FRammsDriveMotion Driven() const
		{
			if (Active == INDEX_NONE || Active == Low || Held[Active])
			{
				return FRammsDriveMotion();
			}
			FRammsDriveMotion D = Stick;
			if (Active == Diff || Active == Curb)
			{
				D.RightCmPerSecond = 0.0f;
			}
			if (BoundTo[Active] != INDEX_NONE)
			{
				D = GetCarriableMotion(BoundTo[Active], D);
			}
			return D;
		}

		/** Start the chassis moving as the active mode drives, settled. */
		void Settle(const FRammsDriveMotion& Moving)
		{
			Out = Wheels = Moving;
			for (int32 i = 0; i < 50; ++i)
			{
				Advance(0.004);
			}
		}

		FMotionSample Sample() const
		{
			return FMotionSample(bMeasured && Window.IsValid(), bMeasured ? Window.GetMotion() : FRammsDriveMotion());
		}

		/** One tick: the robot moves, the invariants are checked, the switch
		 *  steps on what it measures. */
		void Tick(FModeSwitcher& Switcher, const FSwitchConfig& Config, double Dt = 0.01)
		{
			const FRammsDriveMotion D = Driven();
			if (TreadsDown() && FMath::Abs(D.RightCmPerSecond) > 1e-6f)
			{
				++SidewaysOnTreads;
			}
			if (!OwnWheelsDown(Active) && !D.IsNearlyZero(1e-6f, 1e-6f))
			{
				++DrivenInAir;
			}
			for (int32 i = 0; i < 5; ++i)
			{
				Out = RammsDrive::RampMotion(Out, D, 60.0, 90.0, Dt / 5.0);
				Advance(Dt / 5.0);
			}
			Switcher.Tick(*this, Dt, Sample(), Config);
		}

		// --- ISwitchHost ---------------------------------------------------------
		virtual bool			  IsValidMode(int32 Mode) const override { return Mode >= 0 && Mode < NumModes; }
		virtual bool			  IsModeActive(int32 Mode) const override { return Mode == Active && Mode != INDEX_NONE; }
		virtual bool			  HasStance(int32 Mode) const override { return Mode == Diff || Mode == Holo; }
		virtual FRammsDriveMotion GetCarriableMotion(int32 Mode, const FRammsDriveMotion& Motion) const override
		{
			return (Mode == Diff || Mode == Curb) ? RammsDrive::DifferentialCarriable(Motion, 0.0) : Motion;
		}
		virtual bool BeginHandOver(int32 Mode, int32 Other) override
		{
			if (Mode != Active || Mode == Curb || Mode == Low)
			{
				return false;
			}
			BoundTo[Mode] = Other;
			return true;
		}
		virtual void	EndHandOver(int32 Mode) override { BoundTo[Mode] = INDEX_NONE; }
		virtual void	HoldForStance(int32 Mode, bool bHold) override { Held[Mode] = bHold && Mode == Active; }
		virtual void	HoldOutgoingWheels(int32 Mode, bool bHold) override { OutgoingHeld[Mode] = bHold && Mode != Active; }
		virtual FString ActivateMode(int32 Mode, const FRammsDriveMotion* Seed, double HoldSeconds) override
		{
			for (int32 i = 0; i < NumModes; ++i)
			{
				if (i != Mode)
				{
					// Stood down: everything a switch set on it goes.
					BoundTo[i] = INDEX_NONE;
					Held[i] = false;
				}
			}
			Active = Mode;
			OutgoingHeld[Mode] = false;
			++Activations[Mode];
			SeededForward = Seed ? Seed->ForwardCmPerSecond : -1.0;
			return Seed ? TEXT("carried") : TEXT("nothing carried");
		}
		virtual bool GetStanceChannels(int32 Mode, TArray<FStanceChannel>& OutChannels) override
		{
			OutChannels.Reset();
			if (!HasStance(Mode))
			{
				return false;
			}
			if (Active != Low)
			{
				OutChannels.Add({ FName("linkage.main.height"), false, L, Mode == Diff ? TreadsDownL : TreadsUpL, true });
			}
			OutChannels.Add({ FName("crank"), true, K, Mode == Diff ? CornersUpK : CornersDownK, false });
			return true;
		}
		virtual void CommandStance(const TArray<FStanceChannel>& Channels, double Elapsed, double Duration) override
		{
			++StanceCommands;
			for (const FStanceChannel& C : Channels)
			{
				const double V = RammsDrive::StanceRampValue(C, Elapsed, Duration);
				if (C.bMotor)
				{
					K = V;
				}
				else
				{
					L = V;
					RampedL.Add(V);
				}
			}
		}
		virtual void ApplyStance(int32 Mode) override
		{
			if (HasStance(Mode))
			{
				L = Mode == Diff ? TreadsDownL : TreadsUpL;
				K = Mode == Diff ? CornersUpK : CornersDownK;
				++Applied[Mode];
			}
		}
		virtual FString ModeName(int32 Mode) const override
		{
			static const TCHAR* Names[] = { TEXT("diff"), TEXT("holo"), TEXT("curb"), TEXT("low") };
			return IsValidMode(Mode) ? Names[Mode] : TEXT("none");
		}

	private:
		FToyChassis			 Chassis;
		FChassisMotionWindow Window;
		/** What the wheels drive (on the ramp) and what the chassis does (the
		 *  lag behind it). */
		FRammsDriveMotion Out;
		FRammsDriveMotion Wheels;

		void Advance(double Dt)
		{
			const double A = Dt / 0.12;
			Wheels.ForwardCmPerSecond += static_cast<float>(A * (Out.ForwardCmPerSecond - Wheels.ForwardCmPerSecond));
			Wheels.RightCmPerSecond += static_cast<float>(A * (Out.RightCmPerSecond - Wheels.RightCmPerSecond));
			Chassis.Motion = FRammsDriveMotion(Wheels.ForwardCmPerSecond + Slide.ForwardCmPerSecond, Wheels.RightCmPerSecond + Slide.RightCmPerSecond, 0.0f);
			Chassis.Step(Dt);
			T += Dt;
			Window.Add(T, Chassis.Centre, Chassis.Forward());
		}
	};

	FSwitchConfig ToyConfig()
	{
		FSwitchConfig C;
		C.Limits.MaxSidewaysCmS = 2.0;
		C.Limits.TimeoutSeconds = 3.0;
		C.MinStageSeconds = 1.0;
		C.MaxLinkageSpeed = 15.0;
		C.MaxMotorRate = 1.5;
		return C;
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHandOverSwitchOnTheMove,
	"Ramms.Drive.HandOver.Switch.OnTheMove",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHandOverSwitchOnTheMove::RunTest(const FString& Parameters)
{
	// Holonomic at 30 forward and 15 sideways, switched to differential with
	// the stick still asking to strafe.
	const FSwitchConfig Config = ToyConfig();
	FToyRobot			R;
	R.Stick = FRammsDriveMotion(30.0f, 15.0f, 0.0f);
	R.Settle(R.Stick);
	FModeSwitcher S;
	S.Reset(FToyRobot::Holo);

	TestTrue(TEXT("asked for"), S.Request(R, FToyRobot::Diff, R.Sample(), Config));
	TestTrue(TEXT("moving sideways into a mode that cannot: the outgoing mode takes it off first"), S.GetPhase() == ESwitchPhase::Constraining);
	TestEqual(TEXT("...still the active one"), R.Active, static_cast<int32>(FToyRobot::Holo));
	TestEqual(TEXT("...bounded by what the incoming one carries"), R.BoundTo[FToyRobot::Holo], static_cast<int32>(FToyRobot::Diff));

	bool   bStanceWhileWaiting = false;
	double HandOverL = -1.0, TouchdownSideways = 1e9, TouchdownForward = 0.0;
	bool   bTouched = false;
	bool   bBoundAfterHandOver = false;
	int32  Ticks = 0;
	for (; Ticks < 1000 && S.GetPhase() != ESwitchPhase::Idle; ++Ticks)
	{
		const bool	bWaiting = S.GetPhase() == ESwitchPhase::Constraining;
		const int32 Before = R.StanceCommands;
		const bool	bTreadsWereUp = !R.TreadsDown();
		R.Tick(S, Config);
		bStanceWhileWaiting |= bWaiting && S.GetPhase() == ESwitchPhase::Constraining && R.StanceCommands != Before;
		if (bTreadsWereUp && R.TreadsDown() && !bTouched)
		{
			bTouched = true;
			TouchdownSideways = R.Sample().Motion.RightCmPerSecond;
			TouchdownForward = R.Sample().Motion.ForwardCmPerSecond;
		}
		if (R.Active == FToyRobot::Diff && HandOverL < 0.0)
		{
			HandOverL = R.L;
			bBoundAfterHandOver = R.BoundTo[FToyRobot::Diff] == FToyRobot::Holo;
		}
	}
	TestFalse(TEXT("no stance moves while the sideways motion comes off"), bStanceWhileWaiting);
	TestTrue(TEXT("the treads touch down"), bTouched);
	TestTrue(TEXT("...with the sideways speed under the hand-over limit"), FMath::Abs(TouchdownSideways) < Config.Limits.MaxSidewaysCmS);
	TestEqual(TEXT("...and the forward speed carried"), TouchdownForward, 30.0, 0.5);
	TestEqual(TEXT("the differential is activated only once the first stage has its treads down"), HandOverL, FToyRobot::TreadsDownL, 1e-9);
	TestEqual(TEXT("...once"), R.Activations[FToyRobot::Diff], 1);
	TestEqual(TEXT("...handed the forward speed"), R.SeededForward, 30.0, 0.5);
	TestTrue(TEXT("...and held to what the corners carry while they come up"), bBoundAfterHandOver);
	TestTrue(TEXT("the switch completes"), S.GetPhase() == ESwitchPhase::Idle);
	TestEqual(TEXT("...in the differential's stance, applied outright at the end"), R.Applied[FToyRobot::Diff], 1);
	TestEqual(TEXT("...corners up"), R.K, FToyRobot::CornersUpK, 1e-9);
	TestEqual(TEXT("...and the bound gone"), R.BoundTo[FToyRobot::Diff], static_cast<int32>(INDEX_NONE));
	TestEqual(TEXT("never driven sideways with the treads on the ground"), R.SidewaysOnTreads, 0);
	TestEqual(TEXT("never driving wheels in the air"), R.DrivenInAir, 0);
	TestTrue(TEXT("in a few seconds"), Ticks * 0.01 < 4.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHandOverSwitchRefused,
	"Ramms.Drive.HandOver.Switch.Refused",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHandOverSwitchRefused::RunTest(const FString& Parameters)
{
	// On a cross slope that keeps the chassis sliding whatever its wheels do.
	const FSwitchConfig Config = ToyConfig();
	FToyRobot			R;
	R.Stick = FRammsDriveMotion(30.0f, 0.0f, 0.0f);
	R.Slide = FRammsDriveMotion(0.0f, 4.0f, 0.0f);
	R.Settle(R.Stick);
	FModeSwitcher S;
	S.Reset(FToyRobot::Holo);
	S.Request(R, FToyRobot::Diff, R.Sample(), Config);
	TestTrue(TEXT("waiting for the slide to stop"), S.GetPhase() == ESwitchPhase::Constraining);
	int32 Ticks = 0;
	for (; Ticks < 1000 && S.GetPhase() != ESwitchPhase::Idle; ++Ticks)
	{
		R.Tick(S, Config);
	}
	TestEqual(TEXT("refused at the timeout"), Ticks * 0.01, Config.Limits.TimeoutSeconds, 0.015);
	TestTrue(TEXT("...and says so"), S.GetStatus().Contains(TEXT("refused")));
	TestEqual(TEXT("the holonomic mode stays"), R.Active, static_cast<int32>(FToyRobot::Holo));
	TestEqual(TEXT("...back on its operator's command"), R.BoundTo[FToyRobot::Holo], static_cast<int32>(INDEX_NONE));
	TestEqual(TEXT("the differential never activated"), R.Activations[FToyRobot::Diff], 0);
	TestEqual(TEXT("the stance never touched"), R.StanceCommands, 0);
	TestEqual(TEXT("...treads up"), R.L, FToyRobot::TreadsUpL, 1e-12);

	// No measurement at all: never handed over on no data.
	FToyRobot Blind;
	Blind.Stick = FRammsDriveMotion(30.0f, 15.0f, 0.0f);
	Blind.Settle(Blind.Stick);
	FModeSwitcher B;
	B.Reset(FToyRobot::Holo);
	B.Request(Blind, FToyRobot::Diff, Blind.Sample(), Config);
	Blind.bMeasured = false;
	for (Ticks = 0; Ticks < 1000 && B.GetPhase() != ESwitchPhase::Idle; ++Ticks)
	{
		Blind.Tick(B, Config);
	}
	TestTrue(TEXT("unmeasured: refused"), B.GetStatus().Contains(TEXT("refused")));
	TestEqual(TEXT("...the differential never activated"), Blind.Activations[FToyRobot::Diff], 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHandOverSwitchAtRest,
	"Ramms.Drive.HandOver.Switch.AtRest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHandOverSwitchAtRest::RunTest(const FString& Parameters)
{
	const FSwitchConfig Config = ToyConfig();
	{
		// Differential to holonomic at rest, the stick pushed sideways at once.
		FToyRobot R;
		R.L = FToyRobot::TreadsDownL;
		R.K = FToyRobot::CornersUpK;
		R.Active = FToyRobot::Diff;
		FModeSwitcher S;
		S.Reset(FToyRobot::Diff);
		S.Request(R, FToyRobot::Holo, R.Sample(), Config);
		R.Stick = FRammsDriveMotion(0.0f, 20.0f, 0.0f);
		TestEqual(TEXT("at rest the incoming mode is active at once"), R.Active, static_cast<int32>(FToyRobot::Holo));
		TestTrue(TEXT("...held still while its corners go down"), R.Held[FToyRobot::Holo]);
		TestEqual(TEXT("...and bounded by what the treads carry"), R.BoundTo[FToyRobot::Holo], static_cast<int32>(FToyRobot::Diff));
		TestEqual(TEXT("...nothing handed over"), R.SeededForward, -1.0);
		TestTrue(TEXT("...the treads, stood down, held where they are meanwhile"), R.OutgoingHeld[FToyRobot::Diff]);
		double ReleasedAtK = -1.0, TreadsLetGoAtK = -1.0;
		int32  TreadsLetGoUp = 0;
		for (int32 i = 0; i < 1000 && S.GetPhase() != ESwitchPhase::Idle; ++i)
		{
			const bool bWasHeld = R.Held[FToyRobot::Holo];
			const bool bTreadsWereHeld = R.OutgoingHeld[FToyRobot::Diff];
			R.Tick(S, Config);
			if (bWasHeld && !R.Held[FToyRobot::Holo])
			{
				ReleasedAtK = R.K;
			}
			if (bTreadsWereHeld && !R.OutgoingHeld[FToyRobot::Diff])
			{
				TreadsLetGoAtK = R.K;
				TreadsLetGoUp += R.TreadsDown() ? 0 : 1;
			}
		}
		TestEqual(TEXT("released once the first stage has its corners down"), ReleasedAtK, FToyRobot::CornersDownK, 1e-9);
		TestEqual(TEXT("the treads held until then, and let go then (still on the ground)"), TreadsLetGoAtK, FToyRobot::CornersDownK, 1e-9);
		TestEqual(TEXT("...not after they had come up"), TreadsLetGoUp, 0);
		TestFalse(TEXT("...nor held at the end"), R.OutgoingHeld[FToyRobot::Diff]);
		TestEqual(TEXT("never strafed with the treads down"), R.SidewaysOnTreads, 0);
		TestEqual(TEXT("never drove its corners in the air"), R.DrivenInAir, 0);
		TestEqual(TEXT("the bound gone at the end"), R.BoundTo[FToyRobot::Holo], static_cast<int32>(INDEX_NONE));
		TestEqual(TEXT("...strafing as asked"), R.Driven().RightCmPerSecond, 20.0f);
	}
	{
		// Holonomic to differential at rest, pushed forward at once.
		FToyRobot	  R;
		FModeSwitcher S;
		S.Reset(FToyRobot::Holo);
		S.Request(R, FToyRobot::Diff, R.Sample(), Config);
		R.Stick = FRammsDriveMotion(25.0f, 0.0f, 0.0f);
		TestEqual(TEXT("active at once"), R.Active, static_cast<int32>(FToyRobot::Diff));
		TestTrue(TEXT("...held still while its treads go down"), R.Held[FToyRobot::Diff]);
		TestTrue(TEXT("...the corners, stood down, held where they are meanwhile"), R.OutgoingHeld[FToyRobot::Holo]);
		double ReleasedAtL = -1.0, CornersLetGoAtL = -1.0;
		for (int32 i = 0; i < 1000 && S.GetPhase() != ESwitchPhase::Idle; ++i)
		{
			const bool bWasHeld = R.Held[FToyRobot::Diff];
			const bool bCornersWereHeld = R.OutgoingHeld[FToyRobot::Holo];
			R.Tick(S, Config);
			if (bWasHeld && !R.Held[FToyRobot::Diff])
			{
				ReleasedAtL = R.L;
			}
			if (bCornersWereHeld && !R.OutgoingHeld[FToyRobot::Holo])
			{
				CornersLetGoAtL = R.L;
			}
		}
		TestEqual(TEXT("released once its treads are down"), ReleasedAtL, FToyRobot::TreadsDownL, 1e-9);
		TestEqual(TEXT("the corners held until then, and let go then"), CornersLetGoAtL, FToyRobot::TreadsDownL, 1e-9);
		TestFalse(TEXT("...nor held at the end"), R.OutgoingHeld[FToyRobot::Holo]);
		TestEqual(TEXT("never drove its treads in the air"), R.DrivenInAir, 0);
		TestEqual(TEXT("in its stance at the end"), R.K, FToyRobot::CornersUpK, 1e-9);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHandOverSwitchFirstStageWaits,
	"Ramms.Drive.HandOver.Switch.FirstStageWaits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHandOverSwitchFirstStageWaits::RunTest(const FString& Parameters)
{
	const FSwitchConfig Config = ToyConfig();
	// Holonomic straight ahead to differential: nothing to take off, so the
	// treads start down at once -- and stop where they are if the chassis
	// starts sliding.
	for (const bool bSlideStays : { false, true })
	{
		FToyRobot R;
		R.Stick = FRammsDriveMotion(30.0f, 0.0f, 0.0f);
		R.Settle(R.Stick);
		FModeSwitcher S;
		S.Reset(FToyRobot::Holo);
		S.Request(R, FToyRobot::Diff, R.Sample(), Config);
		TestTrue(TEXT("straight into the first stage"), S.GetPhase() == ESwitchPhase::OutgoingStage);
		while (R.L > 14.0 && S.GetPhase() == ESwitchPhase::OutgoingStage)
		{
			R.Tick(S, Config);
		}
		R.Slide = FRammsDriveMotion(0.0f, 5.0f, 0.0f);
		for (int32 i = 0; i < 30; ++i)
		{
			R.Tick(S, Config);
		}
		const double PausedL = R.L;
		for (int32 i = 0; i < 50; ++i)
		{
			R.Tick(S, Config);
		}
		TestEqual(TEXT("sliding: the treads wait where they are"), R.L, PausedL, 1e-12);
		TestTrue(TEXT("...above the ground"), !R.TreadsDown());
		TestEqual(TEXT("...the holonomic mode still in charge"), R.Active, static_cast<int32>(FToyRobot::Holo));
		if (!bSlideStays)
		{
			R.Slide = FRammsDriveMotion();
			for (int32 i = 0; i < 1000 && S.GetPhase() != ESwitchPhase::Idle; ++i)
			{
				R.Tick(S, Config);
			}
			TestEqual(TEXT("the slide over: the switch goes on to the differential"), R.Active, static_cast<int32>(FToyRobot::Diff));
			TestEqual(TEXT("...in its stance"), R.L, FToyRobot::TreadsDownL, 1e-9);
		}
		else
		{
			int32 Ticks = 0;
			for (; Ticks < 1000 && S.GetPhase() == ESwitchPhase::OutgoingStage; ++Ticks)
			{
				R.Tick(S, Config);
			}
			TestTrue(TEXT("still sliding: refused"), S.GetStatus().Contains(TEXT("refused")));
			TestEqual(TEXT("...after waiting the timeout in all"), (Ticks + 80) * 0.01, Config.Limits.TimeoutSeconds, 0.3);
			for (int32 i = 0; i < 1000 && S.GetPhase() != ESwitchPhase::Idle; ++i)
			{
				R.Tick(S, Config);
			}
			TestEqual(TEXT("...the treads back up"), R.L, FToyRobot::TreadsUpL, 1e-9);
			TestEqual(TEXT("...the holonomic mode stays"), R.Active, static_cast<int32>(FToyRobot::Holo));
			TestEqual(TEXT("...its bound gone once they are"), R.BoundTo[FToyRobot::Holo], static_cast<int32>(INDEX_NONE));
			TestEqual(TEXT("...the differential never activated"), R.Activations[FToyRobot::Diff], 0);
		}
		TestEqual(TEXT("never driven sideways with the treads down"), R.SidewaysOnTreads, 0);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHandOverSwitchCancelResumes,
	"Ramms.Drive.HandOver.Switch.CancelResumes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHandOverSwitchCancelResumes::RunTest(const FString& Parameters)
{
	// Differential to holonomic at rest; while the treads are still coming up,
	// back to differential asked for while sliding on a slope -- refused at
	// the timeout. The legs were part way: they go on to the holonomic stance,
	// the mode held to what the treads carry until they are up.
	const FSwitchConfig Config = ToyConfig();
	FToyRobot			R;
	R.L = FToyRobot::TreadsDownL;
	R.K = FToyRobot::CornersUpK;
	R.Active = FToyRobot::Diff;
	FModeSwitcher S;
	S.Reset(FToyRobot::Diff);
	S.Request(R, FToyRobot::Holo, R.Sample(), Config);
	R.Stick = FRammsDriveMotion(20.0f, 10.0f, 0.0f);
	while (S.GetPhase() != ESwitchPhase::Idle && !(S.GetStage() == 1 && R.L > 12.2))
	{
		R.Tick(S, Config);
	}
	TestTrue(TEXT("part way through lifting the treads"), S.GetStage() == 1 && R.L < 13.0);
	R.Slide = FRammsDriveMotion(0.0f, 10.0f, 0.0f);
	for (int32 i = 0; i < 15; ++i)
	{
		R.Tick(S, Config);
	}
	const double AskedL = R.L;
	TestTrue(TEXT("...the treads still on the ground"), R.TreadsDown());
	S.Request(R, FToyRobot::Diff, R.Sample(), Config);
	TestTrue(TEXT("back to differential, sliding: waiting"), S.GetPhase() == ESwitchPhase::Constraining);
	bool bMovedWhileWaiting = false;
	while (S.GetPhase() == ESwitchPhase::Constraining)
	{
		R.Tick(S, Config);
		bMovedWhileWaiting |= R.L != AskedL;
	}
	TestFalse(TEXT("the legs wait while it waits"), bMovedWhileWaiting);
	TestTrue(TEXT("refused"), S.GetStatus().Contains(TEXT("refused")));
	TestTrue(TEXT("...and the stance change it interrupted resumes"), S.GetPhase() == ESwitchPhase::IncomingStage);
	TestEqual(TEXT("...the holonomic mode held to what the treads carry"), R.BoundTo[FToyRobot::Holo], static_cast<int32>(FToyRobot::Diff));
	for (int32 i = 0; i < 1000 && S.GetPhase() != ESwitchPhase::Idle; ++i)
	{
		R.Tick(S, Config);
	}
	TestEqual(TEXT("the treads up"), R.L, FToyRobot::TreadsUpL, 1e-9);
	TestEqual(TEXT("...the bound gone once they are"), R.BoundTo[FToyRobot::Holo], static_cast<int32>(INDEX_NONE));
	TestEqual(TEXT("never strafed with the treads down"), R.SidewaysOnTreads, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveHandOverSwitchFromNoStance,
	"Ramms.Drive.HandOver.Switch.FromNoStance",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveHandOverSwitchFromNoStance::RunTest(const FString& Parameters)
{
	const FSwitchConfig Config = ToyConfig();
	{
		// A curb crossing's end, rolling, all wheels down, the stick asking to
		// strafe: handed to holonomic at once (the curb mode cannot keep
		// driving), in one stage, held to no sideways motion until the treads
		// are up.
		FToyRobot R;
		R.L = 12.5;
		R.K = 0.8;
		R.Active = FToyRobot::Curb;
		R.Stick = FRammsDriveMotion(20.0f, 10.0f, 0.0f);
		R.Settle(FRammsDriveMotion(20.0f, 0.0f, 0.0f));
		FModeSwitcher S;
		S.Reset(FToyRobot::Curb);
		S.Request(R, FToyRobot::Holo, R.Sample(), Config);
		TestEqual(TEXT("handed over at once"), R.Active, static_cast<int32>(FToyRobot::Holo));
		TestEqual(TEXT("...carrying the forward speed"), R.SeededForward, 20.0, 0.5);
		TestEqual(TEXT("...bounded by what the curb mode carries"), R.BoundTo[FToyRobot::Holo], static_cast<int32>(FToyRobot::Curb));
		TestFalse(TEXT("...not held still: its wheels are down"), R.Held[FToyRobot::Holo]);
		TestEqual(TEXT("...in one stage"), S.GetStageSeconds(1), 0.0);
		for (int32 i = 0; i < 1000 && S.GetPhase() != ESwitchPhase::Idle; ++i)
		{
			R.Tick(S, Config);
		}
		TestEqual(TEXT("never strafed with the treads down"), R.SidewaysOnTreads, 0);
		TestEqual(TEXT("strafing once they are up"), R.Driven().RightCmPerSecond, 10.0f);
	}
	{
		// From the low-level mode, which withdraws the linkage while it is
		// active: the stance is planned once the mode asked for is in, so the
		// linkage ramps rather than jumping at the end.
		FToyRobot R;
		R.L = FToyRobot::TreadsDownL;
		R.K = FToyRobot::CornersUpK;
		R.Active = FToyRobot::Low;
		FModeSwitcher S;
		S.Reset(FToyRobot::Low);
		S.Request(R, FToyRobot::Holo, R.Sample(), Config);
		for (int32 i = 0; i < 1000 && S.GetPhase() != ESwitchPhase::Idle; ++i)
		{
			R.Tick(S, Config);
		}
		const bool bRamped = R.RampedL.ContainsByPredicate([](double V) { return V > 13.5 && V < 14.5; });
		TestTrue(TEXT("the linkage ramped through the middle of its travel"), bRamped);
		TestEqual(TEXT("...to the stance"), R.L, FToyRobot::TreadsUpL, 1e-9);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
