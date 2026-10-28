// Tests for the seat mechanism's kinematics (Private/Curb/RammsCurbSeat) and
// its place in the robot model's centre of mass.
//
// The linkage here is a made-up inverted 5-bar in the proportions of a
// wheelchair's seat lift -- arm A a long proximal and short distal, arm B two
// equal links, the platform's pivot 10 cm past the endpoint, the platform
// level at the zero pose -- and its poses were worked out by hand from that
// geometry, not read from any robot.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Curb/RammsCurbRobotModel.h"
#include "Curb/RammsCurbSeat.h"
#include "RammsCurbNegotiationTypes.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	FRammsCurbSeat MakeSeat()
	{
		FRammsCurbSeat		   Seat;
		FRamms5BarLinkageSpec& L = Seat.Linkage;
		L.ProximalMotorA = TEXT("seat_a");
		L.ProximalMotorB = TEXT("seat_b");
		L.PivotA = FVector2D(2.0, 10.845);
		L.PivotB = FVector2D(-15.0, 10.845);
		L.ProximalLengthA = 31.0f;
		L.DistalLengthA = 17.0f;
		L.ProximalLengthB = 27.0f;
		L.DistalLengthB = 27.0f;
		L.ZeroDirA = -2.9924f; // arm A's proximal points aft and a little down at the zero pose
		L.AngleSignA = -1.0f;  // a hinge about +Y
		L.ZeroDirB = 0.7435f;  // arm B's proximal points forward and up
		L.AngleSignB = 1.0f;   // about -Y
		L.bElbowUpA = true;
		L.bElbowUpB = false;
		L.bFlipEndpointSide = false;
		L.TranslationSign = 1.0f;
		Seat.PivotBeyondEndpointCm = 10.0f;
		Seat.TiltMotor = TEXT("seat_tilt");
		Seat.TiltAngleSign = -1.0f;
		return Seat;
	}

	/** Motor angles (A, B, tilt) and where they put the platform's pivot, the
	 *  platform level over the chassis in every one. */
	struct FKnownPose
	{
		const TCHAR* Name;
		FVector2D	 Angles;
		double		 Tilt;
		FVector2D	 Pivot;
	};
	const FKnownPose KnownPoses[] = {
		{ TEXT("zero"), FVector2D(0.0, 0.0), 0.0, FVector2D(-16.5, 30.345) },
		{ TEXT("high"), FVector2D(1.3690, 0.0626), 0.1793, FVector2D(-1.0, 65.845) },
		{ TEXT("forward"), FVector2D(1.4167, -0.7273), -1.2370, FVector2D(19.5, 36.845) },
		{ TEXT("middle"), FVector2D(0.5525, -0.6489), -0.7704, FVector2D(-1.0, 31.845) },
		{ TEXT("aft"), FVector2D(0.4103, 0.2843), 0.4688, FVector2D(-28.0, 45.845) },
		{ TEXT("climb"), FVector2D(1.1402, -0.7337), -1.1384, FVector2D(12.0, 35.845) },
	};
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSeatForward, "Ramms.Curb.Seat.Forward",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRammsCurbSeatForward::RunTest(const FString& Parameters)
{
	const FRammsCurbSeat	   Seat = MakeSeat();
	const RammsCurb::FSeatPose Zero = RammsCurb::SeatForward(Seat, FVector2D::ZeroVector);
	TestTrue(TEXT("the zero pose closes"), Zero.bValid);
	for (const FKnownPose& K : KnownPoses)
	{
		const RammsCurb::FSeatPose P = RammsCurb::SeatForward(Seat, K.Angles);
		TestTrue(FString::Printf(TEXT("%s closes"), K.Name), P.bValid);
		TestTrue(FString::Printf(TEXT("%s puts the pivot at (%.1f, %.1f): got (%.3f, %.3f)"), K.Name, K.Pivot.X, K.Pivot.Y, P.Pivot.X, P.Pivot.Y),
			(P.Pivot - K.Pivot).Size() < 0.05);
		// The platform is level in every known pose: the tilt motor undoes the
		// link's turn from the zero pose.
		const double Pitch = RammsCurb::SeatPlatformPitch(Seat, P.LinkAngle, Zero.LinkAngle, K.Tilt);
		TestTrue(FString::Printf(TEXT("%s holds the platform level: pitch %.4f rad"), K.Name, Pitch), FMath::Abs(Pitch) < 2e-3);
		// And the tilt for level is the known tilt.
		const double Tilt = RammsCurb::SeatTiltFor(Seat, P.LinkAngle, Zero.LinkAngle, 0.0);
		TestTrue(FString::Printf(TEXT("%s: the tilt for level is %.4f, got %.4f"), K.Name, K.Tilt, Tilt), FMath::Abs(Tilt - K.Tilt) < 2e-3);
	}
	// A pitch asked for comes out as asked: 0.2 rad nose up on the high pose.
	const RammsCurb::FSeatPose High = RammsCurb::SeatForward(Seat, KnownPoses[1].Angles);
	const double			   Tilt = RammsCurb::SeatTiltFor(Seat, High.LinkAngle, Zero.LinkAngle, 0.2);
	TestEqual(TEXT("pitch round trip"), RammsCurb::SeatPlatformPitch(Seat, High.LinkAngle, Zero.LinkAngle, Tilt), 0.2, 1e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSeatSolve, "Ramms.Curb.Seat.Solve",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRammsCurbSeatSolve::RunTest(const FString& Parameters)
{
	const FRammsCurbSeat Seat = MakeSeat();
	// Every known pose solved from the zero pose, and from the pose before it.
	FVector2D Hint = FVector2D::ZeroVector;
	for (const FKnownPose& K : KnownPoses)
	{
		FVector2D			 Angles;
		RammsCurb::FSeatPose Pose;
		const bool			 bOk = RammsCurb::SeatSolve(Seat, K.Pivot, FVector2D::ZeroVector, Angles, &Pose);
		TestTrue(FString::Printf(TEXT("%s solves from zero"), K.Name), bOk);
		TestTrue(FString::Printf(TEXT("%s solves to its angles from zero: got (%.4f, %.4f)"), K.Name, Angles.X, Angles.Y),
			(Angles - K.Angles).Size() < 2e-3);
		const bool bFromLast = RammsCurb::SeatSolve(Seat, K.Pivot, Hint, Angles, &Pose);
		TestTrue(FString::Printf(TEXT("%s solves from the pose before"), K.Name), bFromLast && (Angles - K.Angles).Size() < 2e-3);
		TestTrue(FString::Printf(TEXT("%s: the solved pose lands"), K.Name), (Pose.Pivot - K.Pivot).Size() < 1e-3);
		Hint = Angles;
	}
	// Out of reach: 80 cm up is more than the links span.
	FVector2D Angles;
	TestFalse(TEXT("an unreachable pivot is refused"), RammsCurb::SeatSolve(Seat, FVector2D(0.0, 80.0), FVector2D::ZeroVector, Angles));
	// The fore/aft reach at the zero height spans the known fore and aft poses'
	// stretch at least (they sit 1.5 and 6.5 cm higher, within the same reach).
	bool			bValid = false;
	const FVector2D Reach = RammsCurb::SeatReachX(Seat, 30.345, FVector2D::ZeroVector, FVector2D(1.0, 0.0), FVector2D(1.0, 0.0), bValid);
	TestTrue(TEXT("the reach at the zero height is found"), bValid);
	TestTrue(FString::Printf(TEXT("the reach spans the zero pose: %.1f..%.1f"), Reach.X, Reach.Y), Reach.X <= -16.5 && Reach.Y >= -16.5);
	TestTrue(TEXT("the reach is a span"), Reach.Y - Reach.X > 20.0);
	// Bounded by motor ranges: motor A held to [0, 0.3] shortens the forward
	// reach (the middle pose, 15 cm ahead, already needs 0.55).
	const FVector2D Bounded = RammsCurb::SeatReachX(Seat, 30.345, FVector2D::ZeroVector, FVector2D(0.0, 0.3), FVector2D(1.0, 0.0), bValid);
	TestTrue(FString::Printf(TEXT("a motor range bounds the reach: %.1f..%.1f within %.1f..%.1f"), Bounded.X, Bounded.Y, Reach.X, Reach.Y),
		bValid && Bounded.Y < Reach.Y && Bounded.X >= Reach.X - 1e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSeatCentreOfMass, "Ramms.Curb.Seat.CentreOfMass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRammsCurbSeatCentreOfMass::RunTest(const FString& Parameters)
{
	// A 200 kg robot whose seat carries 90 kg of rider and seat 40 cm over
	// the pivot and 18 kg of mechanism at it: the centre of mass follows the
	// pivot by (90 + 18) / 200 of its shift, and the rider's lump turns with
	// the platform's pitch.
	RammsCurb::FRobotModel Model;
	Model.MassKg = 200.0;
	Model.CenterOfMass = FVector(0.0, 0.0, 20.0);
	Model.Seat.bHas = true;
	Model.Seat.ZeroPivot = FVector2D(-16.5, 30.345);
	Model.Seat.MechanismMassKg = 18.0;
	Model.Seat.PayloadMassKg = 90.0;
	Model.Seat.PayloadOffset = FVector2D(0.0, 40.0);
	const TArray<FVector2D> NoLegs;
	const FVector			AtZero = Model.CenterOfMassAt(NoLegs, Model.Seat.ZeroPivot, 0.0);
	TestTrue(TEXT("at the zero pose the centre of mass is the tuning's"), AtZero.Equals(Model.CenterOfMass, 1e-9));
	const FVector Shifted = Model.CenterOfMassAt(NoLegs, Model.Seat.ZeroPivot + FVector2D(20.0, 0.0), 0.0);
	TestEqual(TEXT("20 cm of seat shifts the centre of mass by its share"), Shifted.X, 20.0 * 108.0 / 200.0, 1e-9);
	TestEqual(TEXT("a fore/aft shift leaves the height"), Shifted.Z, 20.0, 1e-9);
	const FVector Raised = Model.CenterOfMassAt(NoLegs, Model.Seat.ZeroPivot + FVector2D(0.0, 10.0), 0.0);
	TestEqual(TEXT("10 cm of seat lift raises it by its share"), Raised.Z, 20.0 + 10.0 * 108.0 / 200.0, 1e-9);
	// Pitched 0.2 rad nose up, the rider's lump 40 cm up moves aft 40 sin 0.2
	// and down 40 (1 - cos 0.2); only the payload turns.
	const FVector Pitched = Model.CenterOfMassAt(NoLegs, Model.Seat.ZeroPivot, 0.2);
	TestEqual(TEXT("pitching moves the rider aft"), Pitched.X, -40.0 * FMath::Sin(0.2) * 90.0 / 200.0, 1e-9);
	TestEqual(TEXT("pitching lowers the rider"), Pitched.Z, 20.0 - 40.0 * (1.0 - FMath::Cos(0.2)) * 90.0 / 200.0, 1e-9);
	// Without a seat the overload is the plain centre of mass.
	Model.Seat.bHas = false;
	TestTrue(TEXT("no seat: unchanged"), Model.CenterOfMassAt(NoLegs, FVector2D(50.0, 50.0), 1.0).Equals(Model.CenterOfMass, 1e-9));
	// Massless: no effect however it moves.
	Model.Seat.bHas = true;
	Model.MassKg = 0.0;
	TestTrue(TEXT("a massless model does not move"), Model.CenterOfMassAt(NoLegs, FVector2D(50.0, 50.0), 1.0).Equals(Model.CenterOfMass, 1e-9));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
