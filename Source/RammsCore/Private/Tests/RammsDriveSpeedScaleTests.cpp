// Tests for a drive mode being asked to go slower (IRammsDriveMode::
// SetDriveSpeedScale): the requests a mode holds and how they lapse
// (RammsDrive::FSpeedScaleRequests), what a scale does to a differential
// drive's wheel speeds and to a holonomic drive's motion (both parts alike,
// the path kept), the two drive controllers taking, refusing and dropping
// requests, and what each then asks of its wheels. Every number is made up.
//
// Beside each test is what it must catch.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "RammsDifferentialDriveController.h"
#include "RammsDifferentialDriveLibrary.h"
#include "RammsDriveHandOver.h"
#include "RammsHolonomicDriveController.h"
#include "RammsRobotBaseComponent.h"
#include "Tests/RammsCommandOffsetRig.h"

#if WITH_DEV_AUTOMATION_TESTS

// Catches: a request that never lapses (a layer that stopped asking would
// leave the robot slow for good); the last request winning over the least; a
// requester's new request added beside its old one; a request that can stop
// the robot (no floor), or stand for as long as its requester likes; one that
// outlives a simulation reset.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveSpeedScaleExpires,
	"Ramms.Drive.SpeedScale.Expires",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveSpeedScaleExpires::RunTest(const FString& Parameters)
{
	using RammsDrive::FSpeedScaleRequests;
	const FName Level(TEXT("level"));
	const FName Safety(TEXT("safety"));
	{
		FSpeedScaleRequests Requests;
		TestEqual(TEXT("nothing asked: full speed"), Requests.Get(10.0), 1.0);
		Requests.Set(Level, 0.6, 0.5, 10.0);
		TestEqual(TEXT("asked 0.6: 0.6"), Requests.Get(10.0), 0.6);
		TestEqual(TEXT("0.49 s on: still"), Requests.Get(10.49), 0.6);
		TestEqual(TEXT("0.5 s on, not renewed: lapsed"), Requests.Get(10.5), 1.0);
		TestEqual(TEXT("... and dropped"), Requests.Num(), 0);

		// Renewed every tick it stands; its requester's new request replaces
		// the old one, up as well as down.
		Requests.Set(Level, 0.6, 0.5, 20.0);
		Requests.Set(Level, 0.8, 0.5, 20.4);
		TestEqual(TEXT("renewed at 0.8: one request"), Requests.Num(), 1);
		TestEqual(TEXT("... 0.8, past the first one's end"), Requests.Get(20.7), 0.8);
		TestEqual(TEXT("... and lapsed half a second after the renewal"), Requests.Get(20.9), 1.0);
	}
	{
		// Two requesters: the least of what stands.
		FSpeedScaleRequests Requests;
		Requests.Set(Level, 0.7, 0.5, 5.0);
		Requests.Set(Safety, 0.4, 0.2, 5.0);
		TestEqual(TEXT("two requesters: the least"), Requests.Get(5.1), 0.4);
		TestEqual(TEXT("the lower one lapsed: the other"), Requests.Get(5.3), 0.7);
		Requests.Set(Safety, 0.9, 1.0, 5.3);
		TestEqual(TEXT("the last to ask does not win over the least"), Requests.Get(5.35), 0.7);

		// Withdrawn: full speed asked, or no time.
		Requests.Set(Level, 1.0, 0.5, 5.4);
		TestEqual(TEXT("withdrawn by asking full speed: the other's"), Requests.Get(5.4), 0.9);
		Requests.Set(Safety, 0.9, 0.0, 5.4);
		TestEqual(TEXT("withdrawn by asking no time: full speed"), Requests.Get(5.4), 1.0);
		TestEqual(TEXT("... nothing kept"), Requests.Num(), 0);
	}
	{
		// Never to a stop, never for long, never from before a reset.
		FSpeedScaleRequests Requests;
		Requests.Set(Level, 0.0, 0.5, 1.0);
		TestEqual(TEXT("asked to stop: the floor"), Requests.Get(1.0), RammsDrive::MinDriveSpeedScale);
		Requests.Set(Level, -3.0, 0.5, 1.0);
		TestEqual(TEXT("asked backwards: the floor"), Requests.Get(1.0), RammsDrive::MinDriveSpeedScale);
		Requests.Set(Level, 0.5, 1000.0, 1.0);
		TestEqual(TEXT("asked for a thousand seconds: stands"), Requests.Get(2.9), 0.5);
		TestEqual(TEXT("... for two"), Requests.Get(3.0), 1.0);
		Requests.Set(Level, 0.5, 0.5, 50.0);
		TestEqual(TEXT("the clock set back (a reset): what was asked before it is gone"), Requests.Get(0.1), 1.0);
		Requests.Set(Level, FMath::Sqrt(-1.0), 0.5, 1.0);
		TestEqual(TEXT("not a number: nothing asked"), Requests.Get(1.0), 1.0);
		Requests.Set(Level, 0.5, 0.5, 1.0);
		Requests.Reset();
		TestEqual(TEXT("reset: full speed"), Requests.Get(1.0), 1.0);
	}
	return true;
}

// Catches: a scale put on one wheel or one axis (the robot would turn off the
// path its operator steers), on the stick before its own mix is cut to the
// top speed (a stick held in a corner would not be slowed at all), or
// changing anything at full speed (the frame-locked runs of a robot nobody
// slows must repeat to the last bit).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveSpeedScaleBothAxes,
	"Ramms.Drive.SpeedScale.ScalesBothAxes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveSpeedScaleBothAxes::RunTest(const FString& Parameters)
{
	// A differential drive's stick, mixed as the controller's library mixes it
	// (top speed 60 cm/s), then scaled.
	const FVector2D Sticks[] = { { 0.0, 1.0 }, { 0.3, 1.0 }, { 1.0, 1.0 }, { -0.5, 0.4 }, { 1.0, 0.0 }, { 0.2, -0.8 } };
	for (const FVector2D& Stick : Sticks)
	{
		const FDifferentialDriveCommand Full = URammsDifferentialDriveLibrary::JoystickToDifferentialDrive(Stick, 60.0f, 0.05f);
		float							Left = Full.LeftCommand;
		float							Right = Full.RightCommand;
		RammsDrive::ScaleWheelSpeeds(Left, Right, 0.5);
		const FString Which = FString::Printf(TEXT("stick (%.1f, %.1f)"), Stick.X, Stick.Y);
		TestEqual(Which + TEXT(": the left wheel at half its speed"), Left, 0.5f * Full.LeftCommand, 1.0e-5f);
		TestEqual(Which + TEXT(": the right wheel at half its speed"), Right, 0.5f * Full.RightCommand, 1.0e-5f);
		// The curve driven: the turn over the travel, the same.
		TestEqual(Which + TEXT(": the same curve"), (Left - Right) * (Full.LeftCommand + Full.RightCommand), (Full.LeftCommand - Full.RightCommand) * (Left + Right), 1.0e-3f);
		// (A stick held in the corner has its mix cut to the top speed: it is
		// slowed all the same.)
		TestTrue(Which + TEXT(": slowed"), FMath::Max(FMath::Abs(Left), FMath::Abs(Right)) <= 30.0f + 1.0e-4f);

		Left = Full.LeftCommand;
		Right = Full.RightCommand;
		RammsDrive::ScaleWheelSpeeds(Left, Right, 1.0);
		TestTrue(Which + TEXT(": at full speed, untouched to the last bit"), Left == Full.LeftCommand && Right == Full.RightCommand);
		RammsDrive::ScaleWheelSpeeds(Left, Right, 0.0);
		TestEqual(Which + TEXT(": never to a stop (left)"), Left, static_cast<float>(RammsDrive::MinDriveSpeedScale) * Full.LeftCommand, 1.0e-5f);
		TestEqual(Which + TEXT(": never to a stop (right)"), Right, static_cast<float>(RammsDrive::MinDriveSpeedScale) * Full.RightCommand, 1.0e-5f);
	}

	// A holonomic drive's motion: forward, sideways and turning alike.
	const FRammsDriveMotion Motion(40.0f, -25.0f, 30.0f);
	const FRammsDriveMotion Half = RammsDrive::ScaledMotion(Motion, 0.5);
	TestEqual(TEXT("holonomic: forward at half"), Half.ForwardCmPerSecond, 20.0f, 1.0e-5f);
	TestEqual(TEXT("holonomic: sideways at half"), Half.RightCmPerSecond, -12.5f, 1.0e-5f);
	TestEqual(TEXT("holonomic: turning at half"), Half.YawRateDegPerSecond, 15.0f, 1.0e-5f);
	const FRammsDriveMotion Whole = RammsDrive::ScaledMotion(Motion, 1.0);
	TestTrue(TEXT("holonomic: at full speed, untouched to the last bit"),
		Whole.ForwardCmPerSecond == Motion.ForwardCmPerSecond && Whole.RightCmPerSecond == Motion.RightCmPerSecond && Whole.YawRateDegPerSecond == Motion.YawRateDegPerSecond);
	const FRammsDriveMotion Least = RammsDrive::ScaledMotion(Motion, 0.0);
	TestEqual(TEXT("holonomic: never to a stop"), Least.ForwardCmPerSecond, static_cast<float>(RammsDrive::MinDriveSpeedScale) * 40.0f, 1.0e-5f);
	return true;
}

// Catches: a drive mode that keeps a request across a change of mode (asked
// to go slower before a curb crossing, it would come back from the crossing
// still slow; asked while another mode drives, it would start slow), or that
// takes one while it is not the mode driving.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveSpeedScaleModes,
	"Ramms.Drive.SpeedScale.ModesTakeAndDropRequests",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveSpeedScaleModes::RunTest(const FString& Parameters)
{
	const FName Level(TEXT("level"));
	const FName Other(TEXT("other"));
	// Never begun, on no robot: they drive nothing here, and their clock
	// stands at zero.
	URammsDifferentialDriveController* Differential = NewObject<URammsDifferentialDriveController>(GetTransientPackage());
	URammsHolonomicDriveController*	   Holonomic = NewObject<URammsHolonomicDriveController>(GetTransientPackage());
	struct FCase
	{
		const TCHAR*	   Name;
		IRammsDriveMode*   Mode;
		TFunction<float()> Scale;
	};
	const FCase Cases[2] = { { TEXT("differential"), Differential, [Differential]() { return Differential->GetDriveSpeedScale(); } },
		{ TEXT("holonomic"), Holonomic, [Holonomic]() { return Holonomic->GetDriveSpeedScale(); } } };
	for (const FCase& Case : Cases)
	{
		const auto Say = [&Case](const TCHAR* What) { return FString::Printf(TEXT("%s: %s"), Case.Name, What); };
		Case.Mode->SetDriveModeActive(true);
		TestEqual(Say(TEXT("nothing asked: full speed")), Case.Scale(), 1.0f);
		Case.Mode->SetDriveSpeedScale(Level, 0.6f, 0.5f);
		TestEqual(Say(TEXT("asked 0.6")), Case.Scale(), 0.6f);
		Case.Mode->SetDriveSpeedScale(Other, 0.4f, 0.5f);
		TestEqual(Say(TEXT("a second requester: the least")), Case.Scale(), 0.4f);
		Case.Mode->SetDriveSpeedScale(Other, 1.0f, 0.0f);
		TestEqual(Say(TEXT("withdrawn: the first again")), Case.Scale(), 0.6f);

		// Stood down (a crossing takes over): nothing is kept, and nothing is
		// taken while it is not the mode driving.
		Case.Mode->SetDriveModeActive(false);
		TestEqual(Say(TEXT("stood down: every request dropped")), Case.Scale(), 1.0f);
		Case.Mode->SetDriveSpeedScale(Level, 0.3f, 0.5f);
		TestEqual(Say(TEXT("not active: takes no request")), Case.Scale(), 1.0f);
		Case.Mode->SetDriveModeActive(true);
		TestEqual(Say(TEXT("active again: at full speed")), Case.Scale(), 1.0f);
		Case.Mode->SetDriveSpeedScale(Level, 0.0f, 0.5f);
		TestEqual(Say(TEXT("asked to stop: its floor")), Case.Scale(), static_cast<float>(RammsDrive::MinDriveSpeedScale));
		Case.Mode->SetDriveModeActive(false);
	}
	return true;
}

// Catches: a request the drive controllers hold and never use (the wheels
// asked the same speeds as before); one that turns the robot off the curve its
// operator steers; and one that stops a robot whose operator holds the stick
// a little way out.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsDriveSpeedScaleControllers,
	"Ramms.Drive.SpeedScale.ControllersDriveSlower",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsDriveSpeedScaleControllers::RunTest(const FString& Parameters)
{
	using namespace RammsCommandOffsetTests;
	const FName Level(TEXT("level"));
	const float Dt = 0.05f;
	// A made-up base: two drive wheels a differential drive turns with torque,
	// four omni wheels a holonomic drive turns by speed.
	const TCHAR* const		Ids[4] = { TEXT("wheel_fl"), TEXT("wheel_fr"), TEXT("wheel_rl"), TEXT("wheel_rr") };
	TArray<FRammsMotorSpec> Specs;
	Specs.Add(MotorRow(TEXT("left_motor"), ERammsActuatorType::Torque));
	Specs.Add(MotorRow(TEXT("right_motor"), ERammsActuatorType::Torque));
	for (const TCHAR* Id : Ids)
	{
		Specs.Add(MotorRow(Id, ERammsActuatorType::Velocity));
	}
	FRig R(Specs);
	if (!TestTrue(TEXT("a made-up base"), R.IsReady()))
	{
		return false;
	}
	{
		// A differential drive in velocity control: top speed 60 cm/s on
		// 10 cm wheels, full steer asking 40 deg/s over a 50 cm track.
		URammsDifferentialDriveController* Drive = NewObject<URammsDifferentialDriveController>(R.Actor, TEXT("Differential"));
		IRammsDriveMode*				   Mode = Drive;
		Drive->ControlMode = EDriveControlMode::VelocityControl;
		Drive->MaxVelocity = 60.0f;
		Drive->WheelRadius = 10.0f;
		Drive->TrackWidth = 50.0f;
		Drive->MaxTurningSpeed = 40.0f;
		Drive->bHoldWhenIdle = false;
		FRig::BeginPlayOf(*Drive);
		Mode->SetDriveModeActive(true);
		const auto Targets = [Drive, Dt](const FVector2D& Stick) {
			Drive->SetExternalDriveInput(Stick);
			Drive->TickComponent(Dt, LEVELTICK_All, nullptr);
			return FVector2D(Drive->GetLeftWheelState().TargetAngularVelocity, Drive->GetRightWheelState().TargetAngularVelocity);
		};
		const FVector2D Stick(0.5, 1.0);
		const FVector2D Full = Targets(Stick);
		TestTrue(TEXT("differential: driven on a curve (the left wheel the faster)"), Full.X > Full.Y && Full.Y > 1.0);
		Mode->SetDriveSpeedScale(Level, 0.5f, 0.5f);
		const FVector2D Half = Targets(Stick);
		TestEqual(TEXT("differential, asked half: the left wheel at half its speed"), Half.X, 0.5 * Full.X, 1.0e-4);
		TestEqual(TEXT("differential, asked half: the right wheel at half its speed"), Half.Y, 0.5 * Full.Y, 1.0e-4);
		// A stick only just out of its dead zone, and the least any request
		// can leave: still driven.
		Mode->SetDriveSpeedScale(Level, 0.0f, 0.5f);
		const FVector2D Crawl = Targets(FVector2D(0.0, 0.1));
		TestTrue(*FString::Printf(TEXT("differential, asked to stop with the stick a little way out: still driven (%.3f rad/s)"), Crawl.X), Crawl.X > 0.0 && Crawl.Y > 0.0);
		Mode->SetDriveSpeedScale(Level, 1.0f, 0.0f);
		const FVector2D Again = Targets(Stick);
		TestTrue(TEXT("differential, the request withdrawn: as before, to the last bit"), Again.X == Full.X && Again.Y == Full.Y);
		Mode->SetDriveModeActive(false);
	}
	{
		// A holonomic drive, its four wheels at 45 deg.
		URammsHolonomicDriveController* Drive = NewObject<URammsHolonomicDriveController>(R.Actor, TEXT("Holonomic"));
		IRammsDriveMode*				Mode = Drive;
		const FVector2D					Places[4] = { { 30.0, -25.0 }, { 30.0, 25.0 }, { -30.0, -25.0 }, { -30.0, 25.0 } };
		const float						Rolls[4] = { 45.0f, -45.0f, -45.0f, 45.0f };
		for (int32 Index = 0; Index < 4; ++Index)
		{
			FRammsOmniWheelSpec& Omni = Drive->Drive.Wheels.AddDefaulted_GetRef();
			Omni.MotorId = Ids[Index];
			Omni.Position = Places[Index];
			Omni.RollDirectionDeg = Rolls[Index];
		}
		Drive->bHoldWhenIdle = false;
		FRig::BeginPlayOf(*Drive);
		const auto Rates = [&](const FVector& Command) {
			Drive->SetDriveCommand(Command);
			Drive->TickComponent(Dt, LEVELTICK_All, nullptr);
			TArray<float> Out;
			for (const TCHAR* Id : Ids)
			{
				float Rate = 0.0f;
				R.Base->GetMotorVelocityCommand(Id, Rate);
				Out.Add(Rate);
			}
			return Out;
		};
		const FVector		Command(1.0, 0.5, 0.3); // forward, sideways and turning at once
		const TArray<float> Full = Rates(Command);
		Mode->SetDriveSpeedScale(Level, 0.5f, 0.5f);
		const TArray<float> Half = Rates(Command);
		Mode->SetDriveSpeedScale(Level, 1.0f, 0.0f);
		const TArray<float> Again = Rates(Command);
		float				Fastest = 0.0f;
		for (int32 Index = 0; Index < 4; ++Index)
		{
			Fastest = FMath::Max(Fastest, FMath::Abs(Full[Index]));
			TestEqual(FString::Printf(TEXT("holonomic, asked half: wheel %d at half its speed"), Index), Half[Index], 0.5f * Full[Index], 1.0e-4f);
			TestTrue(FString::Printf(TEXT("holonomic, the request withdrawn: wheel %d as before, to the last bit"), Index), Again[Index] == Full[Index]);
		}
		TestTrue(TEXT("holonomic: (the wheels are driven)"), Fastest > 1.0f);
		Mode->SetDriveModeActive(false);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
