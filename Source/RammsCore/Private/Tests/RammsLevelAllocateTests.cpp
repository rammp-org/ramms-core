// Tests for the level assist's geometry (Private/Level/RammsLevelAllocate).
//
// Who stands; the rigid plane of leg offsets and its signs; the ride height
// kept, and shifted where that buys reach; the plane scaled back at a travel
// limit, at a lifted leg's clearance and at a belly point; roll kept off the
// legs it must not move; the plane fitted back through a set of heights; and
// the travel a moved leg has left. Every number is made up: a six-wheeled
// robot (corners 90 cm apart fore/aft and 50 across, the middle pair 54
// across) on 9 cm wheels whose centres sit 20 cm under the chassis origin
// with 10 cm of travel either way, and a four-legged one whose front and
// rear legs are ganged pairs -- no robot's own.
//
// Beside each test: the change to the implementation it is there to catch.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Level/RammsLevelAllocate.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace RammsLevelAllocateTests
{
	using RammsLevel::FAllocation;
	using RammsLevel::FClearancePoint;
	using RammsLevel::FLegReach;
	using RammsLevel::FTuning;

	/** Rigid to this (cm), exact to this (deg). */
	constexpr double Tight = 1.0e-9;
	/** What 12 halvings of the share leave of a leg's offset (cm), with room. */
	constexpr double SearchCm = 0.01;

	void CheckNear(FAutomationTestBase& Test, const FString& What, double Actual, double Expected, double Tolerance)
	{
		Test.TestNearlyEqual(What, Actual, Expected, Tolerance);
	}

	double TanDeg(double Deg)
	{
		return FMath::Tan(FMath::DegreesToRadians(Deg));
	}

	FLegReach MakeLeg(double X, double Y, bool bRoll = true)
	{
		FLegReach Leg;
		Leg.XY = FVector2D(X, Y);
		Leg.BaseZ = -20.0;
		Leg.MeasuredZ = -20.0;
		Leg.MinZ = -30.0;
		Leg.MaxZ = -10.0;
		Leg.RadiusCm = 9.0;
		Leg.bRoll = bRoll;
		return Leg;
	}

	/** Front, centre, rear; -Y then +Y of each. */
	TArray<FLegReach> SixWheels()
	{
		return { MakeLeg(45.0, -25.0), MakeLeg(45.0, 25.0), MakeLeg(0.0, -27.0), MakeLeg(0.0, 27.0), MakeLeg(-45.0, -25.0), MakeLeg(-45.0, 25.0) };
	}

	/** Two drive legs roll reaches, then a front and a rear ganged pair it
	 *  does not. The pairs are put 6 cm off the centre line on purpose: at
	 *  y = 0 a roll that leaked into them would move nothing and no test
	 *  could tell. */
	TArray<FLegReach> TwoDrivesTwoPairs()
	{
		return { MakeLeg(0.0, -28.0), MakeLeg(0.0, 28.0), MakeLeg(40.0, 6.0, false), MakeLeg(-35.0, 6.0, false) };
	}

	void Stand(TArray<FLegReach>& Legs, const FTuning& Tuning)
	{
		RammsLevel::SupportMask(Legs, TConstArrayView<bool>(), Tuning);
	}

	FAllocation AllocateFor(const TArray<FLegReach>& Legs, const FVector2D& Deg, const FTuning& Tuning, double PrevRideCm = 0.0, double MaxRideStepCm = 0.0)
	{
		return RammsLevel::Allocate(Legs, TConstArrayView<FClearancePoint>(), Deg, Tuning, PrevRideCm, MaxRideStepCm);
	}

	/** The mean rise of a plane over the standing legs. */
	double SupportMean(const TArray<FLegReach>& Legs, const FVector2D& Deg)
	{
		double Sum = 0.0;
		int32  Count = 0;
		for (const FLegReach& Leg : Legs)
		{
			if (Leg.bSupport)
			{
				Sum += RammsLevel::PlaneRise(Leg.XY, Leg.bRoll, Deg);
				++Count;
			}
		}
		return Count > 0 ? Sum / Count : 0.0;
	}

	/** What leg Index gets from a rigid plane with nothing limiting it. */
	double RigidDz(const TArray<FLegReach>& Legs, int32 Index, const FVector2D& Deg, double HeightCm)
	{
		return RammsLevel::PlaneRise(Legs[Index].XY, Legs[Index].bRoll, Deg) - SupportMean(Legs, Deg) - HeightCm;
	}

	/** How far (cm) the worst standing leg is from the rigid plane the
	 *  allocation says it applied. */
	double WorstOffPlane(const TArray<FLegReach>& Legs, const FAllocation& A)
	{
		double Worst = 0.0;
		for (int32 Index = 0; Index < Legs.Num(); ++Index)
		{
			if (Legs[Index].bSupport)
			{
				Worst = FMath::Max(Worst, FMath::Abs(A.Dz[Index] - RigidDz(Legs, Index, A.AppliedDeg, A.HeightCm)));
			}
		}
		return Worst;
	}
} // namespace RammsLevelAllocateTests

// Catches: a per-leg clamp, taper or gain applied to a leg inside its travel
// (the offsets stop being one plane); the per-axis clamp cutting a plane that
// is inside it.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocatePlaneIsRigid,
	"Ramms.Level.Allocate.PlaneIsRigid",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocatePlaneIsRigid::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const FTuning	  Tuning;
	TArray<FLegReach> Legs = SixWheels();
	Stand(Legs, Tuning);

	const FVector2D	  Wanted(3.0, 2.0);
	const FAllocation A = AllocateFor(Legs, Wanted, Tuning);
	TestEqual(TEXT("an offset per leg"), A.Dz.Num(), Legs.Num());
	CheckNear(*this, TEXT("all of the plane fits"), A.Fraction, 1.0, Tight);
	CheckNear(*this, TEXT("applied pitch"), A.AppliedDeg.X, 3.0, Tight);
	CheckNear(*this, TEXT("applied roll"), A.AppliedDeg.Y, 2.0, Tight);
	CheckNear(*this, TEXT("no ride shift"), A.HeightCm, 0.0, Tight);
	TestEqual(TEXT("no limiting leg"), A.LimitingLeg, static_cast<int32>(INDEX_NONE));
	TestEqual(TEXT("no limiting point"), A.LimitingPoint, static_cast<int32>(INDEX_NONE));
	CheckNear(*this, TEXT("the rate use is the law's to fill"), A.RateUse, 0.0, Tight);
	for (int32 Index = 0; Index < Legs.Num() && Index < A.Dz.Num(); ++Index)
	{
		// Affine in the wheel's place: x tan(pitch) + y tan(roll), nothing else.
		const double Expected = Legs[Index].XY.X * TanDeg(3.0) + Legs[Index].XY.Y * TanDeg(2.0);
		CheckNear(*this, FString::Printf(TEXT("leg %d on the plane"), Index), A.Dz[Index], Expected, Tight);
	}
	return true;
}

// Catches: the pitch term negated (fronts extending for a nose-up correction).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocatePitchSign,
	"Ramms.Level.Allocate.PitchSign",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocatePitchSign::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const FTuning	  Tuning;
	TArray<FLegReach> Legs = SixWheels();
	Stand(Legs, Tuning);

	const FAllocation A = AllocateFor(Legs, FVector2D(2.0, 0.0), Tuning);
	if (!TestEqual(TEXT("an offset per leg"), A.Dz.Num(), 6))
	{
		return true;
	}
	// A positive pitch correction drops the nose: the fronts retract (+), the
	// rears extend (-), the middle pair stays.
	const double Rise = 45.0 * TanDeg(2.0);
	CheckNear(*this, TEXT("front -Y retracts"), A.Dz[0], Rise, Tight);
	CheckNear(*this, TEXT("front +Y retracts"), A.Dz[1], Rise, Tight);
	CheckNear(*this, TEXT("centre -Y stays"), A.Dz[2], 0.0, Tight);
	CheckNear(*this, TEXT("centre +Y stays"), A.Dz[3], 0.0, Tight);
	CheckNear(*this, TEXT("rear -Y extends"), A.Dz[4], -Rise, Tight);
	CheckNear(*this, TEXT("rear +Y extends"), A.Dz[5], -Rise, Tight);
	return true;
}

// Catches: x and y swapped in the plane (a roll correction pitching the
// robot), or the roll term negated.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocateRollSign,
	"Ramms.Level.Allocate.RollSign",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocateRollSign::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const FTuning	  Tuning;
	TArray<FLegReach> Legs = SixWheels();
	Stand(Legs, Tuning);

	const FAllocation A = AllocateFor(Legs, FVector2D(0.0, 2.0), Tuning);
	if (!TestEqual(TEXT("an offset per leg"), A.Dz.Num(), 6))
	{
		return true;
	}
	// A positive roll correction drops the +Y side: +Y legs retract, whatever
	// their fore/aft place; the middle pair, further out, moves the most.
	CheckNear(*this, TEXT("front -Y extends"), A.Dz[0], -25.0 * TanDeg(2.0), Tight);
	CheckNear(*this, TEXT("front +Y retracts"), A.Dz[1], 25.0 * TanDeg(2.0), Tight);
	CheckNear(*this, TEXT("centre -Y extends"), A.Dz[2], -27.0 * TanDeg(2.0), Tight);
	CheckNear(*this, TEXT("centre +Y retracts"), A.Dz[3], 27.0 * TanDeg(2.0), Tight);
	CheckNear(*this, TEXT("rear -Y extends"), A.Dz[4], -25.0 * TanDeg(2.0), Tight);
	CheckNear(*this, TEXT("rear +Y retracts"), A.Dz[5], 25.0 * TanDeg(2.0), Tight);
	return true;
}

// Catches: the mean taken over every leg instead of the standing ones (a
// lifted leg would then change the ride height), or not taken off at all.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocateKeepsRideHeight,
	"Ramms.Level.Allocate.KeepsRideHeight",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocateKeepsRideHeight::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const FTuning	  Tuning;
	TArray<FLegReach> Legs = SixWheels();
	// The front +Y wheel held 5 cm up: five legs stand.
	Legs[1].BaseZ = -15.0;
	Stand(Legs, Tuning);
	TestFalse(TEXT("the raised leg does not stand"), Legs[1].bSupport);

	const FVector2D	  Wanted(3.0, 1.0);
	const FAllocation A = AllocateFor(Legs, Wanted, Tuning);
	if (!TestEqual(TEXT("an offset per leg"), A.Dz.Num(), 6))
	{
		return true;
	}
	CheckNear(*this, TEXT("all of the plane fits"), A.Fraction, 1.0, Tight);
	double SupportSum = 0.0;
	for (int32 Index = 0; Index < 6; ++Index)
	{
		if (Legs[Index].bSupport)
		{
			SupportSum += A.Dz[Index];
		}
	}
	CheckNear(*this, TEXT("the standing legs' offsets sum to nothing"), SupportSum, 0.0, Tight);
	// The lifted leg rides the same plane: the standing five's mean is not
	// zero on this lopsided set, and the lifted leg carries the difference.
	const double Mean = SupportMean(Legs, Wanted);
	TestTrue(TEXT("the standing set is lopsided (the test can tell the two means apart)"), FMath::Abs(Mean) > 0.3);
	CheckNear(*this, TEXT("the lifted leg on the standing legs' plane"), A.Dz[1], 45.0 * TanDeg(3.0) + 25.0 * TanDeg(1.0) - Mean, Tight);
	return true;
}

// Catches: the ride shift's sign (a positive shift must extend every leg),
// the shift not used at all, its allowance's two ends swapped, the slew
// window around last tick's shift ignored, and a shift held on after it is
// no longer needed.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocateShiftsHeightForReach,
	"Ramms.Level.Allocate.ShiftsHeightForReach",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocateShiftsHeightForReach::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const FVector2D Wanted(3.0, 0.0);
	const double	Rise = 45.0 * TanDeg(3.0); // 2.36 cm at the fronts

	// Legs near the top of their travel: 1 cm of retraction left inside the
	// margin, plenty of extension.
	FTuning			  Tuning;
	TArray<FLegReach> Legs = SixWheels();
	for (FLegReach& Leg : Legs)
	{
		Leg.MaxZ = -18.5;
	}
	Stand(Legs, Tuning);
	{
		const FAllocation A = AllocateFor(Legs, Wanted, Tuning);
		TestTrue(TEXT("no allowance: the plane is scaled back"), A.Fraction < 0.9);
		CheckNear(*this, TEXT("no allowance: no shift"), A.HeightCm, 0.0, Tight);
	}
	Tuning.RideHeightUpCm = 3.0;
	{
		// Raising the chassis extends every leg, which gives the fronts back
		// the retraction the plane needs. No more of it than that.
		const FAllocation A = AllocateFor(Legs, Wanted, Tuning, 0.0, 10.0);
		CheckNear(*this, TEXT("raised: all of the plane fits"), A.Fraction, 1.0, Tight);
		CheckNear(*this, TEXT("raised: by just what the fronts are short of"), A.HeightCm, Rise - 1.0, 1.0e-6);
		if (TestEqual(TEXT("raised: an offset per leg"), A.Dz.Num(), 6))
		{
			CheckNear(*this, TEXT("raised: the fronts on their margin"), A.Dz[0], 1.0, 1.0e-6);
			CheckNear(*this, TEXT("raised: the rears extended by the plane and the shift"), A.Dz[4], -Rise - (Rise - 1.0), 1.0e-6);
			CheckNear(*this, TEXT("raised: still one plane"), WorstOffPlane(Legs, A), 0.0, Tight);
		}
		CheckNear(*this, TEXT("raised: the fronts' margin is the travel left"), A.MinMarginCm, 0.5, 1.0e-6);
	}
	{
		// The shift may only move 0.5 cm this tick: the plane waits for it.
		const FAllocation A = AllocateFor(Legs, Wanted, Tuning, 0.0, 0.5);
		TestTrue(TEXT("slewed: only part of the plane yet"), A.Fraction > 0.1 && A.Fraction < 0.9);
		// (To within the search: the shift is the least that fits the share found.)
		TestTrue(TEXT("slewed: as far as the slew allows, no further"), A.HeightCm <= 0.5 + Tight && A.HeightCm > 0.5 - SearchCm);
		if (TestEqual(TEXT("slewed: an offset per leg"), A.Dz.Num(), 6))
		{
			TestTrue(TEXT("slewed: the fronts on their margin"), A.Dz[0] <= 1.0 + Tight && A.Dz[0] > 1.0 - SearchCm);
		}
	}
	{
		// Nothing wanted any more: the shift goes home at the slew rate, not
		// at once and not never.
		const FAllocation A = AllocateFor(Legs, FVector2D::ZeroVector, Tuning, 1.4, 0.5);
		CheckNear(*this, TEXT("return: all of nothing fits"), A.Fraction, 1.0, Tight);
		CheckNear(*this, TEXT("return: one slew step back"), A.HeightCm, 0.9, Tight);
		if (TestEqual(TEXT("return: an offset per leg"), A.Dz.Num(), 6))
		{
			CheckNear(*this, TEXT("return: every leg still extended by the shift"), A.Dz[3], -0.9, Tight);
		}
	}
	{
		// The other way: legs near the bottom of their travel, and leave to
		// sink. The rears cannot extend 2.36, so the chassis comes down.
		FTuning Sink;
		Sink.RideHeightDownCm = 3.0;
		TArray<FLegReach> Low = SixWheels();
		for (FLegReach& Leg : Low)
		{
			Leg.MinZ = -21.5;
		}
		Stand(Low, Sink);
		const FAllocation A = AllocateFor(Low, Wanted, Sink, 0.0, 10.0);
		CheckNear(*this, TEXT("sunk: all of the plane fits"), A.Fraction, 1.0, Tight);
		CheckNear(*this, TEXT("sunk: by just what the rears are short of"), A.HeightCm, -(Rise - 1.0), 1.0e-6);
	}
	{
		// The shift left high by an earlier plane, and a stance that allows
		// none (one leg on its upper margin, one on its lower): nothing fits
		// this tick. The rate limit wins, and the hard clamp keeps the leg
		// that cannot follow inside its travel.
		TArray<FLegReach> Stuck = SixWheels();
		Stuck[0].MaxZ = -19.7;
		Stuck[5].MinZ = -20.2;
		Stand(Stuck, Tuning);
		const FAllocation A = AllocateFor(Stuck, FVector2D::ZeroVector, Tuning, 2.0, 0.5);
		CheckNear(*this, TEXT("stuck: no share"), A.Fraction, 0.0, Tight);
		CheckNear(*this, TEXT("stuck: one slew step towards the stance's height"), A.HeightCm, 1.5, Tight);
		if (TestEqual(TEXT("stuck: an offset per leg"), A.Dz.Num(), 6))
		{
			CheckNear(*this, TEXT("stuck: a free leg follows the shift"), A.Dz[2], -1.5, Tight);
			CheckNear(*this, TEXT("stuck: the short leg stops at its travel"), A.Dz[5], -0.2, Tight);
		}
	}
	return true;
}

// Catches: the travel margin left out (a leg run to its stop), the search
// returning the end that does not fit (a leg past its margin), a share cut
// when the plane still fits, the limiting leg not named, and the extra margin
// of AllocateFraction ignored.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocateScalesBackAtReach,
	"Ramms.Level.Allocate.ScalesBackAtReach",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocateScalesBackAtReach::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const FTuning Tuning;
	// 3 cm of travel either way; 2.5 inside the margin.
	TArray<FLegReach> Legs = SixWheels();
	for (FLegReach& Leg : Legs)
	{
		Leg.MinZ = -23.0;
		Leg.MaxZ = -17.0;
	}
	Stand(Legs, Tuning);
	const TConstArrayView<FClearancePoint> NoPoints;

	// The front +Y leg is asked for 4.4 cm, the rear -Y leg for as much the
	// other way.
	const FVector2D	  Wanted(5.0, 1.0);
	const FAllocation A = AllocateFor(Legs, Wanted, Tuning);
	TestTrue(TEXT("scaled back"), A.Fraction > 0.3 && A.Fraction < 0.8);
	CheckNear(*this, TEXT("applied pitch is the share of the wanted"), A.AppliedDeg.X, 5.0 * A.Fraction, Tight);
	CheckNear(*this, TEXT("applied roll is the share of the wanted"), A.AppliedDeg.Y, 1.0 * A.Fraction, Tight);
	TestTrue(TEXT("one of the two legs at their limits is named"), A.LimitingLeg == 1 || A.LimitingLeg == 4);
	TestEqual(TEXT("no point is named"), A.LimitingPoint, static_cast<int32>(INDEX_NONE));
	if (TestEqual(TEXT("an offset per leg"), A.Dz.Num(), 6))
	{
		TestTrue(TEXT("the limiting leg sits on its margin, not past it"), A.Dz[1] <= 2.5 + Tight && A.Dz[1] > 2.5 - SearchCm);
		TestTrue(TEXT("and its opposite on the other one"), A.Dz[4] >= -2.5 - Tight && A.Dz[4] < -2.5 + SearchCm);
		CheckNear(*this, TEXT("still one plane"), WorstOffPlane(Legs, A), 0.0, Tight);
	}
	TestTrue(TEXT("the margin is what is left of the travel"), A.MinMarginCm >= 0.5 - Tight && A.MinMarginCm < 0.5 + SearchCm);
	TestTrue(TEXT("no reach left"), A.ReachLeft >= 0.0 && A.ReachLeft < 0.01);

	CheckNear(*this, TEXT("AllocateFraction finds the same share"), RammsLevel::AllocateFraction(Legs, NoPoints, Wanted, Tuning, 0.0), A.Fraction, Tight);
	const double Pulled = RammsLevel::AllocateFraction(Legs, NoPoints, Wanted, Tuning, 1.0);
	// 1.5 cm instead of 2.5: about three fifths of the share.
	TestTrue(TEXT("a further margin leaves less"), Pulled < 0.7 * A.Fraction && Pulled > 0.5 * A.Fraction);

	// "Exactly when": a hair inside the margin all of it fits, a hair past
	// it does not.
	const double	  JustInDeg = FMath::RadiansToDegrees(FMath::Atan(2.49 / 45.0));
	const double	  JustOutDeg = FMath::RadiansToDegrees(FMath::Atan(2.51 / 45.0));
	const FAllocation In = AllocateFor(Legs, FVector2D(JustInDeg, 0.0), Tuning);
	const FAllocation Out = AllocateFor(Legs, FVector2D(JustOutDeg, 0.0), Tuning);
	CheckNear(*this, TEXT("just inside: all of it"), In.Fraction, 1.0, Tight);
	TestEqual(TEXT("just inside: no limiting leg"), In.LimitingLeg, static_cast<int32>(INDEX_NONE));
	TestTrue(TEXT("just inside: no limit named"), In.Limit == RammsLevel::EReachLimit::None);
	// The reach left is of the travel the wanted tilt is PLANNED with: the
	// soft zone (2 cm) is kept back, which leaves these legs 0.5 cm of it.
	CheckNear(*this, TEXT("just inside the margin, far past the planned reach: none left"), In.ReachLeft, 0.0, Tight);
	TestTrue(TEXT("... and it is a leg's travel that is nearest"), In.NearestLimit == RammsLevel::EReachLimit::LegTravel);
	{
		const FAllocation Half = AllocateFor(Legs, FVector2D(FMath::RadiansToDegrees(FMath::Atan(0.25 / 45.0)), 0.0), Tuning);
		CheckNear(*this, TEXT("0.25 cm of a planned 0.5: half the reach left"), Half.ReachLeft, 0.5, 1.0e-6);
		FTuning NoZone = Tuning;
		NoZone.SoftZoneCm = 0.0;
		const FAllocation Hard = AllocateFor(Legs, FVector2D(JustInDeg, 0.0), NoZone);
		TestTrue(TEXT("with no soft zone the planned reach is the margin's: a little left"), Hard.ReachLeft > 0.0 && Hard.ReachLeft < 0.01);
	}
	TestTrue(TEXT("scaled back: the limit is a leg's travel"), A.Limit == RammsLevel::EReachLimit::LegTravel);
	TestTrue(TEXT("just outside: not all of it"), Out.Fraction < 1.0 && Out.Fraction > 0.99);
	TestTrue(TEXT("just outside: a front or rear leg named"), Out.LimitingLeg == 0 || Out.LimitingLeg == 1 || Out.LimitingLeg == 4 || Out.LimitingLeg == 5);
	return true;
}

// Catches: a lifted leg held to a standing leg's limits (the whole plane cut
// for a wheel that is in the air), and a lifted leg's offset not stopped at
// its travel.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocateLiftedLegClampsQuietly,
	"Ramms.Level.Allocate.LiftedLegClampsQuietly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocateLiftedLegClampsQuietly::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const FTuning	  Tuning;
	TArray<FLegReach> Legs = SixWheels();
	// The front +Y wheel held 8 cm up, with 2 cm of retraction and 1 cm of
	// extension left to it.
	Legs[1].BaseZ = -12.0;
	Legs[1].MinZ = -13.0;
	Stand(Legs, Tuning);

	const FAllocation Up = AllocateFor(Legs, FVector2D(4.0, 0.0), Tuning);
	const FAllocation Down = AllocateFor(Legs, FVector2D(-4.0, 0.0), Tuning);
	if (!TestEqual(TEXT("an offset per leg"), Up.Dz.Num(), 6) || !TestEqual(TEXT("an offset per leg"), Down.Dz.Num(), 6))
	{
		return true;
	}
	TestTrue(TEXT("the plane asks the lifted leg for more than it has"), RigidDz(Legs, 1, FVector2D(4.0, 0.0), 0.0) > 3.5);
	CheckNear(*this, TEXT("retracting: all of the plane fits"), Up.Fraction, 1.0, Tight);
	TestEqual(TEXT("retracting: nothing limits"), Up.LimitingLeg, static_cast<int32>(INDEX_NONE));
	CheckNear(*this, TEXT("retracting: the lifted leg stops at its travel"), Up.Dz[1], 2.0, Tight);
	CheckNear(*this, TEXT("retracting: the standing legs keep the whole plane"), WorstOffPlane(Legs, Up), 0.0, Tight);
	// A lifted leg resting on its stop is not the robot out of reach: it has
	// its clearance to spend. Held 8 cm up it may lose 5.5 of that (2.5 kept),
	// and with its 2 cm of travel may be asked 7.5 cm; the plane asks
	// 45 tan(4 deg) + (the standing legs' mean, 9 tan(4 deg)) = 3.78. That is
	// what the reach left and the margin are now counted from: of whatever is
	// nearest stopping the plane, a leg's travel or not.
	const double AskedCm = RigidDz(Legs, 1, FVector2D(4.0, 0.0), 0.0);
	CheckNear(*this, TEXT("retracting: the lifted leg's clearance is what is nearest used up"), Up.ReachLeft, 1.0 - AskedCm / 7.5, 1.0e-6);
	CheckNear(*this, TEXT("retracting: the room left is its clearance over the least it may keep"), Up.MinMarginCm, 7.5 - AskedCm, 1.0e-6);
	TestTrue(TEXT("retracting: (a good part of it)"), Up.ReachLeft > 0.4 && Up.MinMarginCm > 3.0);
	{
		// (The other front leg, which stands, is asked as much of a planned
		// 7.5 cm: a tie. With no soft zone it has 9.5, and the lifted leg's
		// clearance is the nearer by itself.)
		FTuning NoZone = Tuning;
		NoZone.SoftZoneCm = 0.0;
		const FAllocation Hard = AllocateFor(Legs, FVector2D(4.0, 0.0), NoZone);
		CheckNear(*this, TEXT("retracting, no soft zone: the same share of the lifted leg's clearance"), Hard.ReachLeft, 1.0 - AskedCm / 7.5, 1.0e-6);
		TestTrue(TEXT("retracting, no soft zone: ... and it is named as that"), Hard.NearestLimit == RammsLevel::EReachLimit::LiftedClearance);
	}
	{
		// Extending, a lifted leg only ends up higher: it uses nothing up.
		TestTrue(TEXT("extending: the lifted leg is not what is nearest"), Down.NearestLimit != RammsLevel::EReachLimit::LiftedClearance);
	}

	CheckNear(*this, TEXT("extending: all of the plane fits"), Down.Fraction, 1.0, Tight);
	CheckNear(*this, TEXT("extending: the lifted leg stops at its travel"), Down.Dz[1], -1.0, Tight);
	return true;
}

// Catches: a lifted leg never limiting anything (its wheel driven into the
// ground by the standing legs' plane), and MinLiftedClearanceCm not read.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocateLiftedClearanceBinds,
	"Ramms.Level.Allocate.LiftedClearanceBinds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocateLiftedClearanceBinds::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	FTuning			  Tuning;
	TArray<FLegReach> Legs = SixWheels();
	// The front +Y wheel held 3 cm up with half a centimetre of retraction
	// left: it may fall 0.5 cm short of the plane before its clearance is
	// under the 2.5 asked.
	Legs[1].BaseZ = -17.0;
	Legs[1].MaxZ = -16.5;
	Stand(Legs, Tuning);
	TestFalse(TEXT("the raised leg does not stand"), Legs[1].bSupport);
	CheckNear(*this, TEXT("its clearance is the stance's"), Legs[1].BaselineClearanceCm, 3.0, Tight);

	const FVector2D	  Wanted(3.0, 0.0);
	const FAllocation A = AllocateFor(Legs, Wanted, Tuning);
	TestTrue(TEXT("scaled back"), A.Fraction > 0.2 && A.Fraction < 0.5);
	TestEqual(TEXT("the lifted leg is named"), A.LimitingLeg, 1);
	TestEqual(TEXT("no point is named"), A.LimitingPoint, static_cast<int32>(INDEX_NONE));
	if (TestEqual(TEXT("an offset per leg"), A.Dz.Num(), 6))
	{
		CheckNear(*this, TEXT("the lifted leg at its travel"), A.Dz[1], 0.5, Tight);
		// The plane at the lifted wheel has come up 1 cm: 0.5 followed, 0.5
		// of clearance spent.
		const double AtWheel = RigidDz(Legs, 1, A.AppliedDeg, A.HeightCm);
		TestTrue(TEXT("the plane stops where the clearance is used up"), AtWheel <= 1.0 + Tight && AtWheel > 1.0 - SearchCm);
	}

	// Allowed to touch down, it has 3.5 cm against the 2.8 asked.
	Tuning.MinLiftedClearanceCm = 0.0;
	const FAllocation Free = AllocateFor(Legs, Wanted, Tuning);
	CheckNear(*this, TEXT("no clearance asked: all of the plane fits"), Free.Fraction, 1.0, Tight);
	return true;
}

// Catches: clearance points ignored, the ground's rise at a point taken with
// the wrong sign, the ground under the point assumed flat instead of fitted
// through the standing wheels, and the point's MinCm not read.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocateBellyPointBinds,
	"Ramms.Level.Allocate.BellyPointBinds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocateBellyPointBinds::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	FTuning			  Tuning;
	TArray<FLegReach> Legs = SixWheels();
	Stand(Legs, Tuning);

	// A point of the underside 40 cm ahead, 3 cm over the ground the wheels
	// stand on (their bottoms are at -29), which must keep 2: the ground may
	// come up 1 cm there.
	TArray<FClearancePoint> Points;
	FClearancePoint			Belly;
	Belly.XY = FVector2D(40.0, 0.0);
	Belly.UndersideZ = -26.0;
	Belly.MinCm = 2.0;
	Points.Add(Belly);

	const FVector2D Wanted(3.0, 0.0);
	{
		const FAllocation A = RammsLevel::Allocate(Legs, Points, Wanted, Tuning, 0.0, 0.0);
		TestTrue(TEXT("scaled back"), A.Fraction > 0.4 && A.Fraction < 0.55);
		TestEqual(TEXT("the point is named"), A.LimitingPoint, 0);
		TestEqual(TEXT("no leg is named"), A.LimitingLeg, static_cast<int32>(INDEX_NONE));
		const double GroundRise = RammsLevel::PlaneRise(Belly.XY, true, A.AppliedDeg);
		TestTrue(TEXT("the ground under the point has come up its 1 cm"), GroundRise <= 1.0 + Tight && GroundRise > 1.0 - SearchCm);
		CheckNear(*this, TEXT("AllocateFraction reads the points too"), RammsLevel::AllocateFraction(Legs, Points, Wanted, Tuning, 0.0), A.Fraction, Tight);
	}
	{
		const FAllocation A = AllocateFor(Legs, Wanted, Tuning);
		CheckNear(*this, TEXT("without the point all of it fits"), A.Fraction, 1.0, Tight);
	}
	{
		// The other way the ground falls away from the point.
		const FAllocation A = RammsLevel::Allocate(Legs, Points, FVector2D(-3.0, 0.0), Tuning, 0.0, 0.0);
		CheckNear(*this, TEXT("nose up: the point does not bind"), A.Fraction, 1.0, Tight);
	}
	{
		// Asked to keep only 0.5 cm, the point has 2.5 against the 2.1 needed.
		TArray<FClearancePoint> Lenient = Points;
		Lenient[0].MinCm = 0.5;
		const FAllocation A = RammsLevel::Allocate(Legs, Lenient, Wanted, Tuning, 0.0, 0.0);
		CheckNear(*this, TEXT("a smaller clearance asked: all of it fits"), A.Fraction, 1.0, Tight);
	}
	{
		// A stance already nose down (fronts extended 2 cm, rears retracted
		// 2): the ground under the point is 1.8 cm lower than a level stance
		// would put it, and the point has room for the whole plane.
		FTuning Wide;
		Wide.SupportBandInCm = 10.0;
		Wide.SupportBandOutCm = 10.0;
		TArray<FLegReach> Tilted = SixWheels();
		Tilted[0].BaseZ = -22.0;
		Tilted[1].BaseZ = -22.0;
		Tilted[4].BaseZ = -18.0;
		Tilted[5].BaseZ = -18.0;
		Stand(Tilted, Wide);
		TestTrue(TEXT("tilted stance: every leg stands"), Tilted[4].bSupport && Tilted[5].bSupport);
		const FAllocation A = RammsLevel::Allocate(Tilted, Points, Wanted, Wide, 0.0, 0.0);
		CheckNear(*this, TEXT("tilted stance: the fitted ground leaves room"), A.Fraction, 1.0, Tight);
	}
	return true;
}

// Catches: bRoll ignored (roll leaking into a ganged pair, here 6 cm off the
// centre line so it shows), in the offsets or in the mean that keeps the
// ride height.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocateRollOnlyOnDriveSupports,
	"Ramms.Level.Allocate.RollOnlyOnDriveSupports",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocateRollOnlyOnDriveSupports::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const FTuning	  Tuning;
	TArray<FLegReach> Legs = TwoDrivesTwoPairs();
	Stand(Legs, Tuning);

	const FAllocation A = AllocateFor(Legs, FVector2D(2.0, 3.0), Tuning);
	if (!TestEqual(TEXT("an offset per leg"), A.Dz.Num(), 4))
	{
		return true;
	}
	CheckNear(*this, TEXT("all of the plane fits"), A.Fraction, 1.0, Tight);
	// Pitch over all four, roll as +- half the track on the two drives only.
	const double Mean = (40.0 - 35.0) * TanDeg(2.0) / 4.0;
	CheckNear(*this, TEXT("drive -Y: roll only"), A.Dz[0], -28.0 * TanDeg(3.0) - Mean, Tight);
	CheckNear(*this, TEXT("drive +Y: roll only"), A.Dz[1], 28.0 * TanDeg(3.0) - Mean, Tight);
	CheckNear(*this, TEXT("front pair: pitch only"), A.Dz[2], 40.0 * TanDeg(2.0) - Mean, Tight);
	CheckNear(*this, TEXT("rear pair: pitch only"), A.Dz[3], -35.0 * TanDeg(2.0) - Mean, Tight);
	CheckNear(*this, TEXT("the ride height is kept"), A.Dz[0] + A.Dz[1] + A.Dz[2] + A.Dz[3], 0.0, Tight);

	// A pure roll leaves the pairs alone altogether.
	const FAllocation Roll = AllocateFor(Legs, FVector2D(0.0, 3.0), Tuning);
	if (TestEqual(TEXT("an offset per leg"), Roll.Dz.Num(), 4))
	{
		CheckNear(*this, TEXT("pure roll: front pair still"), Roll.Dz[2], 0.0, Tight);
		CheckNear(*this, TEXT("pure roll: rear pair still"), Roll.Dz[3], 0.0, Tight);
	}
	return true;
}

// Catches: MaxCorrectionDeg not applied, applied as one total length instead
// of per axis (the pitch would be cut with the roll), or applied to pitch
// with the roll's limit.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocateRollCap,
	"Ramms.Level.Allocate.RollCap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocateRollCap::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	FTuning Tuning;
	Tuning.MaxCorrectionDeg = FVector2D(15.0, 6.0);
	TArray<FLegReach> Legs = TwoDrivesTwoPairs();
	Stand(Legs, Tuning);

	const FAllocation A = AllocateFor(Legs, FVector2D(2.0, -10.0), Tuning);
	// The cap is not "reach": the share is of the capped plane, and whole.
	CheckNear(*this, TEXT("all of the capped plane fits"), A.Fraction, 1.0, Tight);
	CheckNear(*this, TEXT("pitch untouched"), A.AppliedDeg.X, 2.0, Tight);
	CheckNear(*this, TEXT("roll at its cap, sign kept"), A.AppliedDeg.Y, -6.0, Tight);
	if (TestEqual(TEXT("an offset per leg"), A.Dz.Num(), 4))
	{
		CheckNear(*this, TEXT("the drives split the capped roll"), A.Dz[0] - A.Dz[1], 56.0 * TanDeg(6.0), Tight);
		CheckNear(*this, TEXT("the pairs split the whole pitch"), A.Dz[2] - A.Dz[3], 75.0 * TanDeg(2.0), Tight);
	}
	return true;
}

// Catches: the memory ignored (one band, so a leg at the edge chatters), the
// two bands swapped, the measured height used instead of the baseline (an
// offset would then change who stands), and the clearance not written.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocateSupportBandHysteresis,
	"Ramms.Level.Allocate.SupportBandHysteresis",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocateSupportBandHysteresis::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const FTuning	  Tuning; // in at 1 cm, out at 2
	TArray<FLegReach> Legs = SixWheels();
	// Front -Y 0.8 cm up, front +Y 1.5, centre -Y 2.5; the rest on the ground.
	Legs[0].BaseZ = -19.2;
	Legs[1].BaseZ = -18.5;
	Legs[2].BaseZ = -17.5;
	// The joints say the centre -Y wheel is down (an offset is holding it
	// there): that must not make it a support leg.
	Legs[2].MeasuredZ = -20.0;
	// Stale flags, to see they are overwritten.
	Legs[0].bSupport = false;
	Legs[2].bSupport = true;
	Legs[3].bSupport = false;

	RammsLevel::SupportMask(Legs, TConstArrayView<bool>(), Tuning);
	TestTrue(TEXT("no memory: 0.8 cm stands"), Legs[0].bSupport);
	TestFalse(TEXT("no memory: 1.5 cm does not"), Legs[1].bSupport);
	TestFalse(TEXT("no memory: 2.5 cm does not"), Legs[2].bSupport);
	TestTrue(TEXT("no memory: on the ground stands"), Legs[3].bSupport);
	CheckNear(*this, TEXT("clearance of the 0.8 cm leg"), Legs[0].BaselineClearanceCm, 0.8, Tight);
	CheckNear(*this, TEXT("clearance of the 1.5 cm leg"), Legs[1].BaselineClearanceCm, 1.5, Tight);
	CheckNear(*this, TEXT("clearance of the 2.5 cm leg"), Legs[2].BaselineClearanceCm, 2.5, Tight);
	CheckNear(*this, TEXT("clearance of a leg on the ground"), Legs[4].BaselineClearanceCm, 0.0, Tight);

	const TArray<bool> AllStood = { true, true, true, true, true, true };
	RammsLevel::SupportMask(Legs, AllStood, Tuning);
	TestTrue(TEXT("stood before: 1.5 cm still stands"), Legs[1].bSupport);
	TestFalse(TEXT("stood before: 2.5 cm has left"), Legs[2].bSupport);

	const TArray<bool> NoneStood = { false, false, false, false, false, false };
	RammsLevel::SupportMask(Legs, NoneStood, Tuning);
	TestTrue(TEXT("did not stand: 0.8 cm joins"), Legs[0].bSupport);
	TestFalse(TEXT("did not stand: 1.5 cm stays out"), Legs[1].bSupport);

	// A memory of the wrong length is no memory.
	const TArray<bool> Short = { true, true };
	RammsLevel::SupportMask(Legs, Short, Tuning);
	TestFalse(TEXT("wrong-length memory: 1.5 cm is judged by the In band"), Legs[1].bSupport);
	return true;
}

// Catches: a lifted leg or an unmoved leg counted (a stance resting a leg on
// a stop read as "at reach"), the margin measured to the wrong end of the
// travel, and the tuning's margin taken off (this is the true travel).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocateTravelMargin,
	"Ramms.Level.Allocate.TravelMargin",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocateTravelMargin::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const FTuning	  Tuning;
	TArray<FLegReach> Legs = SixWheels();
	// The centre -Y leg's stance rests it on its upper stop; the rear +Y
	// wheel is held up, out of the support set, right at its stop.
	Legs[2].MaxZ = -20.0;
	Legs[5].BaseZ = -14.0;
	Legs[5].MaxZ = -13.0;
	// The front +Y leg has 4 cm of extension, 10 of retraction.
	Legs[1].MinZ = -24.0;
	Stand(Legs, Tuning);

	const TArray<double> None = { 0.0, 0.0, 0.005, 0.0, 0.0, 0.0 };
	CheckNear(*this, TEXT("no leg moved: no margin to report"), RammsLevel::TravelMargin(Legs, None), RammsLevel::NoTravelMarginCm, Tight);

	// Front -Y retracted 3 (7 left to the top), front +Y extended 3 (1 left
	// to the bottom, 13 to the top), the lifted leg moved to its stop.
	const TArray<double> Moved = { 3.0, -3.0, 0.0, 0.0, 0.0, 1.0 };
	CheckNear(*this, TEXT("the least travel left in the direction moved"), RammsLevel::TravelMargin(Legs, Moved), 1.0, Tight);

	const TArray<double> Retract = { 3.0, 0.0, 0.0, 0.0, 0.0, 1.0 };
	CheckNear(*this, TEXT("a retracted leg is measured to the top of its travel"), RammsLevel::TravelMargin(Legs, Retract), 7.0, Tight);

	const TArray<double> Past = { 0.0, -5.0, 0.0, 0.0, 0.0, 0.0 };
	CheckNear(*this, TEXT("never negative"), RammsLevel::TravelMargin(Legs, Past), 0.0, Tight);
	return true;
}

// Catches: FitPlane returning slopes instead of degrees, the roll flags
// ignored in the fit, the height not taken out before the slopes, and
// MeasuredCorrection fitting the heights themselves instead of their
// difference from the baseline.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelFitPlaneInvertsAllocate,
	"Ramms.Level.FitPlane.InvertsAllocate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelFitPlaneInvertsAllocate::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const FTuning Tuning;
	for (int32 Robot = 0; Robot < 2; ++Robot)
	{
		TArray<FLegReach> Legs = Robot == 0 ? SixWheels() : TwoDrivesTwoPairs();
		const FVector2D	  Wanted = Robot == 0 ? FVector2D(3.0, -2.0) : FVector2D(-2.0, 3.0);
		const TCHAR*	  Name = Robot == 0 ? TEXT("six wheels") : TEXT("two drives, two pairs");
		// A stance that is not flat, so heights and height changes differ.
		Legs[0].BaseZ = -20.6;
		Legs[1].BaseZ = -19.8;
		Stand(Legs, Tuning);

		const FAllocation A = AllocateFor(Legs, Wanted, Tuning);
		if (!TestEqual(FString::Printf(TEXT("%s: an offset per leg"), Name), A.Dz.Num(), Legs.Num()))
		{
			continue;
		}
		TArray<FVector2D> XY;
		TArray<double>	  Values;
		TArray<bool>	  Roll;
		for (int32 Index = 0; Index < Legs.Num(); ++Index)
		{
			XY.Add(Legs[Index].XY);
			// The offsets and a common 4 cm on top.
			Values.Add(A.Dz[Index] + 4.0);
			Roll.Add(Legs[Index].bRoll);
			// The joints have followed the offsets, and sit 0.7 cm high all round.
			Legs[Index].MeasuredZ = Legs[Index].BaseZ + A.Dz[Index] + 0.7;
		}
		FVector2D  Deg(99.0, 99.0);
		double	   HeightCm = 99.0;
		const bool bFitted = RammsLevel::FitPlane(XY, Values, TConstArrayView<bool>(), Roll, Deg, HeightCm);
		TestTrue(FString::Printf(TEXT("%s: fitted"), Name), bFitted);
		CheckNear(*this, FString::Printf(TEXT("%s: pitch back"), Name), Deg.X, Wanted.X, Tight);
		CheckNear(*this, FString::Printf(TEXT("%s: roll back"), Name), Deg.Y, Wanted.Y, Tight);
		CheckNear(*this, FString::Printf(TEXT("%s: the height at the origin"), Name), HeightCm, 4.0 - SupportMean(Legs, Wanted), Tight);

		const FVector2D Measured = RammsLevel::MeasuredCorrection(Legs);
		CheckNear(*this, FString::Printf(TEXT("%s: measured pitch correction"), Name), Measured.X, Wanted.X, Tight);
		CheckNear(*this, FString::Printf(TEXT("%s: measured roll correction"), Name), Measured.Y, Wanted.Y, Tight);

		// RigidOffsets is the same plane with nothing limiting it.
		RammsLevel::FLegValues Rigid;
		RammsLevel::RigidOffsets(Legs, Wanted, Rigid);
		if (TestEqual(FString::Printf(TEXT("%s: a rigid offset per leg"), Name), Rigid.Num(), Legs.Num()))
		{
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				CheckNear(*this, FString::Printf(TEXT("%s: rigid offset %d"), Name, Index), Rigid[Index], A.Dz[Index], Tight);
			}
		}
	}
	return true;
}

// Catches: a warp or one odd leg bending the fitted plane (it must be left
// as residual), the Use mask ignored, and no rank tolerance (points in line
// dividing by a zero spread: wild slopes or NaN instead of the minimum-norm
// plane).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelFitPlaneLeavesHeightAndShapeOut,
	"Ramms.Level.FitPlane.LeavesHeightAndShapeOut",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelFitPlaneLeavesHeightAndShapeOut::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const TArray<FLegReach>		Legs = SixWheels();
	const FVector2D				Plane(2.0, -1.5);
	const TConstArrayView<bool> All;

	TArray<FVector2D> XY;
	TArray<double>	  Warped;
	for (const FLegReach& Leg : Legs)
	{
		XY.Add(Leg.XY);
		// The plane, 4 cm of common height, and a twist: the corners on one
		// diagonal 0.8 cm up, on the other 0.8 down, the middle pair not.
		const double Twist = Leg.XY.X == 0.0 ? 0.0 : (Leg.XY.X * Leg.XY.Y > 0.0 ? 0.8 : -0.8);
		Warped.Add(RammsLevel::PlaneRise(Leg.XY, true, Plane) + 4.0 + Twist);
	}
	{
		FVector2D Deg(99.0, 99.0);
		double	  HeightCm = 99.0;
		TestTrue(TEXT("warped: fitted"), RammsLevel::FitPlane(XY, Warped, All, All, Deg, HeightCm));
		CheckNear(*this, TEXT("warped: the twist is not pitch"), Deg.X, 2.0, Tight);
		CheckNear(*this, TEXT("warped: the twist is not roll"), Deg.Y, -1.5, Tight);
		CheckNear(*this, TEXT("warped: the common height is the height"), HeightCm, 4.0, Tight);
	}
	{
		// One wheel 3 cm off the plane, masked out: the other five give the
		// plane exactly.
		TArray<double> OneOff = Warped;
		for (int32 Index = 0; Index < OneOff.Num(); ++Index)
		{
			OneOff[Index] = RammsLevel::PlaneRise(XY[Index], true, Plane) + 4.0;
		}
		OneOff[1] += 3.0;
		const TArray<bool> Use = { true, false, true, true, true, true };
		FVector2D		   Deg(99.0, 99.0);
		double			   HeightCm = 99.0;
		TestTrue(TEXT("masked: fitted"), RammsLevel::FitPlane(XY, OneOff, Use, All, Deg, HeightCm));
		CheckNear(*this, TEXT("masked: pitch"), Deg.X, 2.0, Tight);
		CheckNear(*this, TEXT("masked: roll"), Deg.Y, -1.5, Tight);
		CheckNear(*this, TEXT("masked: height"), HeightCm, 4.0, Tight);
	}
	{
		// Only the two -Y corners: they span fore/aft and nothing across.
		const TArray<bool> Use = { true, false, false, false, true, false };
		FVector2D		   Deg(99.0, 99.0);
		double			   HeightCm = 99.0;
		TestTrue(TEXT("two points: fitted"), RammsLevel::FitPlane(XY, Warped, Use, All, Deg, HeightCm));
		const double Slope = (Warped[0] - Warped[4]) / 90.0;
		CheckNear(*this, TEXT("two points: the pitch between them"), Deg.X, FMath::RadiansToDegrees(FMath::Atan(Slope)), Tight);
		CheckNear(*this, TEXT("two points: no roll"), Deg.Y, 0.0, Tight);
		CheckNear(*this, TEXT("two points: the height midway"), HeightCm, 0.5 * (Warped[0] + Warped[4]), Tight);
	}
	{
		// Three points in a line that is square to neither axis: the slope
		// along the line, none across it.
		const TArray<FVector2D> Line = { FVector2D(-30.0, -10.0), FVector2D(0.0, 0.0), FVector2D(30.0, 10.0) };
		const TArray<double>	Heights = { -2.0, 1.0, 4.0 };
		FVector2D				Deg(99.0, 99.0);
		double					HeightCm = 99.0;
		TestTrue(TEXT("in line: fitted"), RammsLevel::FitPlane(Line, Heights, All, All, Deg, HeightCm));
		// 3 cm per sqrt(1000) cm along (3, 1) / sqrt(10): a gradient of
		// (0.09, 0.03).
		CheckNear(*this, TEXT("in line: pitch"), Deg.X, FMath::RadiansToDegrees(FMath::Atan(0.09)), Tight);
		CheckNear(*this, TEXT("in line: roll"), Deg.Y, FMath::RadiansToDegrees(FMath::Atan(0.03)), Tight);
		CheckNear(*this, TEXT("in line: height"), HeightCm, 1.0, Tight);
	}
	{
		// No point roll reaches: the across direction is not spanned.
		const TArray<bool> NoRoll = { false, false, false, false, false, false };
		TArray<double>	   Pitched;
		for (const FVector2D& P : XY)
		{
			Pitched.Add(RammsLevel::PlaneRise(P, false, Plane) + 0.02 * P.Y);
		}
		FVector2D Deg(99.0, 99.0);
		double	  HeightCm = 99.0;
		TestTrue(TEXT("no roll: fitted"), RammsLevel::FitPlane(XY, Pitched, All, NoRoll, Deg, HeightCm));
		CheckNear(*this, TEXT("no roll: pitch"), Deg.X, 2.0, Tight);
		CheckNear(*this, TEXT("no roll: roll is zero, not a guess"), Deg.Y, 0.0, Tight);
	}
	{
		const TArray<bool> One = { false, false, false, true, false, false };
		FVector2D		   Deg(99.0, 99.0);
		double			   HeightCm = 99.0;
		TestTrue(TEXT("one point: fitted"), RammsLevel::FitPlane(XY, Warped, One, All, Deg, HeightCm));
		CheckNear(*this, TEXT("one point: level"), Deg.Size(), 0.0, Tight);
		CheckNear(*this, TEXT("one point: at its value"), HeightCm, Warped[3], Tight);

		const TArray<bool> Nothing = { false, false, false, false, false, false };
		TestFalse(TEXT("no point: not fitted"), RammsLevel::FitPlane(XY, Warped, Nothing, All, Deg, HeightCm));
		CheckNear(*this, TEXT("no point: zero plane"), Deg.Size(), 0.0, Tight);
		CheckNear(*this, TEXT("no point: zero height"), HeightCm, 0.0, Tight);
	}
	return true;
}

// Catches: the achieved correction fitted through legs that do not carry the
// robot (a raised leg resting on its retract stop, or lagging in the air,
// bends a fit through every leg: the terrain estimate then reads short of the
// ground, the base settles short of level and a release stalls on it), no
// fallback where the carrying legs do not span a direction, and a fit that
// takes a nearly-in-line stance's noise for a slope.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelMeasuredCorrectionCarryingLegs,
	"Ramms.Level.FitPlane.MeasuredCorrectionIsTheCarryingLegs",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelMeasuredCorrectionCarryingLegs::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const FTuning Tuning;

	// The front pair carried 6 cm clear, 0.3 cm short of its retract stop:
	// the robot stands on the other four.
	const auto FrontsRaised = []() {
		TArray<FLegReach> Legs = SixWheels();
		for (int32 Index = 0; Index < 2; ++Index)
		{
			Legs[Index].BaseZ = -14.0;
			Legs[Index].MeasuredZ = -14.0;
			Legs[Index].MaxZ = -13.7;
		}
		return Legs;
	};
	{
		// A nose-up slope taken out: the fronts are asked to retract, and stop.
		TArray<FLegReach> Legs = FrontsRaised();
		Stand(Legs, Tuning);
		TestTrue(TEXT("raised: the fronts do not stand"), !Legs[0].bSupport && !Legs[1].bSupport && Legs[2].bSupport && Legs[5].bSupport);
		TestTrue(TEXT("raised: ... and carry nothing, the others all of their share"), Legs[0].CarryShare == 0.0 && Legs[1].CarryShare == 0.0 && Legs[2].CarryShare == 1.0 && Legs[5].CarryShare == 1.0);
		const FVector2D	  Wanted(2.0, -1.0);
		const FAllocation A = AllocateFor(Legs, Wanted, Tuning);
		if (TestEqual(TEXT("raised: an offset per leg"), A.Dz.Num(), Legs.Num()))
		{
			CheckNear(*this, TEXT("raised: all of the plane fits"), A.Fraction, 1.0, Tight);
			CheckNear(*this, TEXT("raised: a front leg stops at its travel"), A.Dz[1], 0.3, Tight);
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				Legs[Index].MeasuredZ = Legs[Index].BaseZ + A.Dz[Index];
			}
			FVector2D Measured = RammsLevel::MeasuredCorrection(Legs);
			CheckNear(*this, TEXT("raised, on their stop: the pitch the standing legs made"), Measured.X, Wanted.X, Tight);
			CheckNear(*this, TEXT("raised, on their stop: the roll the standing legs made"), Measured.Y, Wanted.Y, Tight);

			// ... and wherever the raised legs are on their way to.
			Legs[0].MeasuredZ = Legs[0].BaseZ - 2.5;
			Legs[1].MeasuredZ = Legs[1].BaseZ + 0.1;
			Measured = RammsLevel::MeasuredCorrection(Legs);
			CheckNear(*this, TEXT("raised, lagging: pitch"), Measured.X, Wanted.X, Tight);
			CheckNear(*this, TEXT("raised, lagging: roll"), Measured.Y, Wanted.Y, Tight);
		}
	}
	{
		// A leg being lifted fades out of the plane; it does not drop out of
		// it in one step. The front pair lags a stance ramp by 1 cm, on a
		// robot whose offsets are all zero: what that reads as (0.64 deg on
		// the ground) shrinks with the height the ramp has reached, no
		// quarter centimetre of it taking more than half, and is nothing by
		// the inner band (1 cm) -- well before the pair leaves the support
		// set, at 2 cm.
		double Last = 0.0;
		for (int32 Step = 0; Step <= 9; ++Step)
		{
			const double	   LiftCm = 0.25 * Step;
			TArray<FLegReach>  Legs = SixWheels();
			const TArray<bool> Stood = { true, true, true, true, true, true };
			for (int32 Index = 0; Index < 2; ++Index)
			{
				Legs[Index].BaseZ += LiftCm;
				Legs[Index].MeasuredZ = Legs[Index].BaseZ - 1.0;
			}
			RammsLevel::SupportMask(Legs, Stood, Tuning);
			CheckNear(*this, FString::Printf(TEXT("lifted %.2f cm: a front leg's share"), LiftCm), Legs[0].CarryShare, FMath::Max(0.0, 1.0 - LiftCm), Tight);
			TestEqual(FString::Printf(TEXT("lifted %.2f cm: it stands up to the outer band"), LiftCm), Legs[0].bSupport, LiftCm <= 2.0);
			const double Pitch = RammsLevel::MeasuredCorrection(Legs).X;
			if (Step == 0)
			{
				TestTrue(TEXT("on the ground: a lagging front pair is in the plane"), Pitch < -0.6);
			}
			else
			{
				TestTrue(FString::Printf(TEXT("lifted %.2f cm: it counts for no more than at %.2f (%.3f deg, %.3f before)"), LiftCm, LiftCm - 0.25, Pitch, Last), FMath::Abs(Pitch) <= FMath::Abs(Last));
				TestTrue(FString::Printf(TEXT("lifted %.2f cm: ... and for less by no more than half of it"), LiftCm), FMath::Abs(Pitch - Last) <= 0.32);
			}
			if (LiftCm >= 1.0)
			{
				CheckNear(*this, FString::Printf(TEXT("lifted %.2f cm: nothing of it is left"), LiftCm), Pitch, 0.0, Tight);
			}
			Last = Pitch;
		}
	}
	{
		// A leg being set down: its baseline is on the ground before its
		// joint is. Until the joint has come down it stands on nothing, and
		// its lateness is not a correction. The front pair's baseline is
		// down, its joints 0.6 cm behind; every joint also sags 0.3 cm under
		// the load, which is nobody's lateness.
		TArray<FLegReach>			 Legs = SixWheels();
		const RammsLevel::FLegValues Dz = { 0.5, 0.5, 0.0, 0.0, -0.5, -0.5 };
		for (int32 Index = 0; Index < Legs.Num(); ++Index)
		{
			Legs[Index].MeasuredZ = Legs[Index].BaseZ + Dz[Index] + 0.3 + (Index < 2 ? 0.6 : 0.0);
		}
		Stand(Legs, Tuning);
		const double Before = RammsLevel::MeasuredCorrection(Legs).X;
		RammsLevel::DiscountLateLegs(Legs, Dz, Tuning);
		CheckNear(*this, TEXT("set down: a late leg's share"), Legs[0].CarryShare, 0.4, Tight);
		CheckNear(*this, TEXT("set down: a sag all round is not lateness"), Legs[4].CarryShare, 1.0, Tight);
		const double After = RammsLevel::MeasuredCorrection(Legs).X;
		const double TrueDeg = FMath::RadiansToDegrees(FMath::Atan(1.0 / 90.0));
		TestTrue(FString::Printf(TEXT("set down: the late pair counts for less (%.3f deg off the plane, %.3f before)"), After - TrueDeg, Before - TrueDeg),
			FMath::Abs(After - TrueDeg) < 0.6 * FMath::Abs(Before - TrueDeg) && FMath::Abs(Before - TrueDeg) > 0.3);
		// ... and a leg late in LIFTING (its wheel below where it is told) is
		// on the ground for as long as it is late: left alone.
		Legs[0].CarryShare = 1.0;
		Legs[0].MeasuredZ = Legs[0].BaseZ + Dz[0] - 0.6;
		RammsLevel::DiscountLateLegs(Legs, Dz, Tuning);
		CheckNear(*this, TEXT("lifting late: left alone"), Legs[0].CarryShare, 1.0, Tight);
	}
	{
		// Only the centre pair stands (a line across the robot): the roll is
		// theirs; fore and aft they say nothing, and the plane through every
		// leg stands in.
		TArray<FLegReach> Legs = SixWheels();
		for (int32 Index = 0; Index < Legs.Num(); ++Index)
		{
			const bool bCentre = Index == 2 || Index == 3;
			Legs[Index].bSupport = bCentre;
			Legs[Index].CarryShare = bCentre ? 1.0 : 0.0;
			Legs[Index].MeasuredZ = Legs[Index].BaseZ + RammsLevel::PlaneRise(Legs[Index].XY, true, bCentre ? FVector2D(0.0, 3.0) : FVector2D(2.0, 1.0));
		}
		const FVector2D Measured = RammsLevel::MeasuredCorrection(Legs);
		CheckNear(*this, TEXT("a pair across: the roll is the pair's"), Measured.Y, 3.0, Tight);
		CheckNear(*this, TEXT("a pair across: the pitch is every leg's"), Measured.X, 2.0, Tight);
	}
	{
		// No leg is said to stand: the plane through every leg, as before.
		TArray<FLegReach> Legs = SixWheels();
		const FVector2D	  Plane(-3.0, 1.5);
		for (FLegReach& Leg : Legs)
		{
			Leg.bSupport = false;
			Leg.CarryShare = 0.0;
			Leg.MeasuredZ = Leg.BaseZ + RammsLevel::PlaneRise(Leg.XY, true, Plane) + 0.4;
		}
		const FVector2D Measured = RammsLevel::MeasuredCorrection(Legs);
		CheckNear(*this, TEXT("none stands: pitch"), Measured.X, Plane.X, Tight);
		CheckNear(*this, TEXT("none stands: roll"), Measured.Y, Plane.Y, Tight);
	}
	{
		// Three standing legs all but in line fore and aft (1 cm across in
		// 90), the middle one 0.2 cm off the line; a raised pair either side
		// following a roll of 2 deg. Across the line the three carry no
		// information worth the name (0.2 cm in 1 is 11 deg): every leg's
		// plane stands in there too.
		TArray<FLegReach> Legs = { MakeLeg(45.0, 0.5), MakeLeg(0.0, -0.5), MakeLeg(-45.0, 0.5), MakeLeg(0.0, -27.0), MakeLeg(0.0, 27.0) };
		const FVector2D	  Plane(1.0, 2.0);
		for (int32 Index = 0; Index < Legs.Num(); ++Index)
		{
			Legs[Index].bSupport = Index < 3;
			Legs[Index].CarryShare = Index < 3 ? 1.0 : 0.0;
			Legs[Index].MeasuredZ = Legs[Index].BaseZ + RammsLevel::PlaneRise(Legs[Index].XY, true, Plane);
		}
		Legs[1].MeasuredZ += 0.2;
		const FVector2D Measured = RammsLevel::MeasuredCorrection(Legs);
		CheckNear(*this, TEXT("all but in line: the pitch along the line"), Measured.X, Plane.X, 0.05);
		CheckNear(*this, TEXT("all but in line: the roll is not the line's noise"), Measured.Y, Plane.Y, 0.3);
	}
	return true;
}

// Catches: a stance moved up or down as a whole read as a turn of it; the
// turn's sign or size; a leg that does not carry the robot counted in it.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocateBaselinePlaneMove,
	"Ramms.Level.Allocate.BaselinePlaneMoveIsTheStancesOwnTurn",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocateBaselinePlaneMove::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const FTuning Tuning;
	const auto	  BaseOf = [](const TArray<FLegReach>& Legs) {
		   RammsLevel::FLegValues Out;
		   for (const FLegReach& Leg : Legs)
		   {
			   Out.Add(Leg.BaseZ);
		   }
		   return Out;
	};
	{
		TArray<FLegReach> Legs = SixWheels();
		Stand(Legs, Tuning);
		const RammsLevel::FLegValues Before = BaseOf(Legs);
		TestTrue(TEXT("nothing moved: no turn"), RammsLevel::BaselinePlaneMove(Legs, Before).IsNearlyZero(Tight));
		for (FLegReach& Leg : Legs)
		{
			Leg.BaseZ += 1.3;
		}
		Stand(Legs, Tuning);
		TestTrue(TEXT("every leg moved alike (the ride height): no turn"), RammsLevel::BaselinePlaneMove(Legs, Before).IsNearlyZero(Tight));
		const RammsLevel::FLegValues TooFew = { -20.0, -20.0 };
		TestTrue(TEXT("baselines that are not the legs': no turn"), RammsLevel::BaselinePlaneMove(Legs, TooFew).IsZero());
	}
	{
		// Each baseline moved by a plane of (0.5, -0.3) deg, and 0.4 cm besides.
		TArray<FLegReach> Legs = SixWheels();
		Stand(Legs, Tuning);
		const RammsLevel::FLegValues Before = BaseOf(Legs);
		const FVector2D				 Plane(0.5, -0.3);
		for (FLegReach& Leg : Legs)
		{
			Leg.BaseZ += RammsLevel::PlaneRise(Leg.XY, true, Plane) + 0.4;
		}
		Stand(Legs, Tuning);
		const FVector2D Turn = RammsLevel::BaselinePlaneMove(Legs, Before);
		CheckNear(*this, TEXT("a plane: its pitch"), Turn.X, 0.5, 1.0e-6);
		CheckNear(*this, TEXT("a plane: its roll"), Turn.Y, -0.3, 1.0e-6);
	}
	{
		// The front +Y wheel is carried 8 cm up, and is moved alone.
		TArray<FLegReach> Legs = SixWheels();
		Legs[1].BaseZ = -12.0;
		Stand(Legs, Tuning);
		const RammsLevel::FLegValues Before = BaseOf(Legs);
		Legs[1].BaseZ += 2.0;
		Stand(Legs, Tuning);
		TestTrue(TEXT("a leg carried off the ground, moved alone: the stance is not turned"), RammsLevel::BaselinePlaneMove(Legs, Before).IsNearlyZero(Tight));
		// The rear pair, which the robot stands on, drawn up 0.6 cm: it sits back.
		Legs[4].BaseZ += 0.6;
		Legs[5].BaseZ += 0.6;
		Stand(Legs, Tuning);
		const FVector2D Turn = RammsLevel::BaselinePlaneMove(Legs, Before);
		TestTrue(TEXT("legs it stands on, moved at one end: a turn that way"), Turn.X < -0.2);
	}
	{
		// The rear pair drawn up 1.5 cm: past where a leg counts as carrying
		// (1 cm), inside the band it stays in the support set by (2 cm). The
		// robot may well have sat back onto it. As far as the legs carry the
		// stance has not turned; as far as it CAN have, it has, by the plane
		// through all six: 1.5 cm at one end of 90.
		TArray<FLegReach> Legs = SixWheels();
		Stand(Legs, Tuning);
		const RammsLevel::FLegValues Before = BaseOf(Legs);
		const TArray<bool>			 Stood = { true, true, true, true, true, true };
		Legs[4].BaseZ += 1.5;
		Legs[5].BaseZ += 1.5;
		RammsLevel::SupportMask(Legs, Stood, Tuning);
		TestTrue(TEXT("(the rear pair is still of the support set, and carries nothing)"), Legs[4].bSupport && Legs[5].bSupport && Legs[4].CarryShare == 0.0);
		TestTrue(TEXT("as far as the legs carry: no turn"), RammsLevel::BaselinePlaneMove(Legs, Before).IsNearlyZero(Tight));
		const double Can = RammsLevel::BaselineTurnDeg(Legs, Before);
		TestTrue(FString::Printf(TEXT("as far as the stance can have turned: the plane through the support set (%.2f deg)"), Can), Can > 0.6 && Can < 1.0);
		// Drawn up clear of the support band: no longer a leg it can be sitting on.
		Legs[4].BaseZ += 1.5;
		Legs[5].BaseZ += 1.5;
		const RammsLevel::FLegValues Mid = BaseOf(Legs);
		RammsLevel::SupportMask(Legs, Stood, Tuning);
		Legs[4].BaseZ += 1.0;
		Legs[5].BaseZ += 1.0;
		CheckNear(*this, TEXT("legs already clear of the support band, drawn further: no turn either way"), RammsLevel::BaselineTurnDeg(Legs, Mid), 0.0, Tight);
		// Never less than the plane the carrying legs give.
		TArray<FLegReach> Plane = SixWheels();
		Stand(Plane, Tuning);
		const RammsLevel::FLegValues PlaneBefore = BaseOf(Plane);
		for (FLegReach& Leg : Plane)
		{
			Leg.BaseZ += RammsLevel::PlaneRise(Leg.XY, true, FVector2D(0.5, -0.3));
		}
		Stand(Plane, Tuning);
		CheckNear(*this, TEXT("a plane: its size"), RammsLevel::BaselineTurnDeg(Plane, PlaneBefore), FVector2D(0.5, -0.3).Size(), 1.0e-6);
		CheckNear(*this, TEXT("baselines that are not the legs': none"), RammsLevel::BaselineTurnDeg(Plane, TConstArrayView<double>()), 0.0, 0.0);
	}
	return true;
}

// Catches: the standing legs' span taken along one axis only, a lifted leg
// counted in it, legs in a line read as no span at all (the ground then free
// to turn at any rate).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocateStandingSpan,
	"Ramms.Level.Allocate.StandingSpanIsTheShorterSide",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocateStandingSpan::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	const FTuning	  Tuning;
	TArray<FLegReach> Legs = SixWheels();
	Stand(Legs, Tuning);
	CheckNear(*this, TEXT("six standing: 90 long, 54 wide: the width"), RammsLevel::StandingSpanCm(Legs), 54.0, Tight);
	Legs[0].bSupport = false;
	Legs[1].bSupport = false;
	CheckNear(*this, TEXT("the front pair lifted: 45 long, 54 wide: the length"), RammsLevel::StandingSpanCm(Legs), 45.0, Tight);
	for (int32 Index = 0; Index < 6; ++Index)
	{
		Legs[Index].bSupport = Index == 2 || Index == 3;
	}
	CheckNear(*this, TEXT("only the centre pair: a line across, 54 wide"), RammsLevel::StandingSpanCm(Legs), 54.0, Tight);
	for (int32 Index = 0; Index < 6; ++Index)
	{
		Legs[Index].bSupport = Index == 2;
	}
	CheckNear(*this, TEXT("one leg: no span, the least there is"), RammsLevel::StandingSpanCm(Legs), RammsLevel::GroundSpanMinCm, Tight);
	for (FLegReach& Leg : Legs)
	{
		Leg.bSupport = false;
	}
	CheckNear(*this, TEXT("none standing: every leg counts"), RammsLevel::StandingSpanCm(Legs), 54.0, Tight);
	TArray<FLegReach> Pairs = TwoDrivesTwoPairs();
	Stand(Pairs, Tuning);
	CheckNear(*this, TEXT("two drives and two pairs: 75 long, 56 wide"), RammsLevel::StandingSpanCm(Pairs), 56.0, Tight);
	CheckNear(*this, TEXT("no legs: the least there is"), RammsLevel::StandingSpanCm(TConstArrayView<FLegReach>()), RammsLevel::GroundSpanMinCm, Tight);
	return true;
}

// Catches: the reach left and the margin read off the standing legs' travel
// alone (a raised wheel's clearance, the underside's, or an axis' limit used
// up with "reach left" still reading most of it and the speed cap seeing
// nothing), and the limit's kind not named, or named wrongly.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAllocateReachNearest,
	"Ramms.Level.Allocate.ReachIsOfWhateverIsNearest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAllocateReachNearest::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAllocateTests;
	using RammsLevel::EReachLimit;
	const TConstArrayView<FClearancePoint> NoPoints;
	{
		// A raised wheel's clearance (as LiftedClearanceBinds): the front +Y
		// wheel 3 cm up with 0.5 cm of retraction left may be asked 1 cm.
		const FTuning	  Tuning;
		TArray<FLegReach> Legs = SixWheels();
		Legs[1].BaseZ = -17.0;
		Legs[1].MaxZ = -16.5;
		Stand(Legs, Tuning);
		const FVector2D	  Wanted(3.0, 0.0);
		const FAllocation A = AllocateFor(Legs, Wanted, Tuning);
		TestTrue(TEXT("raised wheel: cut back, and by its clearance"), A.Fraction < 0.5 && A.Limit == EReachLimit::LiftedClearance);
		TestTrue(TEXT("raised wheel: no reach left"), A.ReachLeft < 0.02 && A.NearestLimit == EReachLimit::LiftedClearance);
		TestTrue(TEXT("raised wheel: no room left (the standing legs have most of their travel)"), A.MinMarginCm < 0.02 && RammsLevel::TravelMargin(Legs, A.Dz) > 5.0);
		const RammsLevel::FReach Reach = RammsLevel::AllocateReach(Legs, NoPoints, Wanted, Tuning, 0.0);
		CheckNear(*this, TEXT("raised wheel: AllocateReach finds the same share"), Reach.Share, A.Fraction, Tight);
		TestTrue(TEXT("raised wheel: ... and names the same limit"), Reach.Limit == EReachLimit::LiftedClearance);
		// Half of that plane: half its clearance left, 0.5 cm of room, nothing cut.
		const FAllocation Half = AllocateFor(Legs, A.AppliedDeg * 0.5, Tuning);
		CheckNear(*this, TEXT("raised wheel, half the plane: all of it fits"), Half.Fraction, 1.0, Tight);
		TestTrue(TEXT("raised wheel, half the plane: no limit"), Half.Limit == EReachLimit::None);
		CheckNear(*this, TEXT("raised wheel, half the plane: half the reach left"), Half.ReachLeft, 0.5, 0.01);
		CheckNear(*this, TEXT("raised wheel, half the plane: half a centimetre of room"), Half.MinMarginCm, 0.5, 0.01);
		TestTrue(TEXT("raised wheel, half the plane: still the nearest"), Half.NearestLimit == EReachLimit::LiftedClearance);
	}
	{
		// The underside (as BellyPointBinds): the ground under the point may
		// come up 1 cm.
		const FTuning	  Tuning;
		TArray<FLegReach> Legs = SixWheels();
		Stand(Legs, Tuning);
		TArray<FClearancePoint> Points;
		FClearancePoint			Belly;
		Belly.XY = FVector2D(40.0, 0.0);
		Belly.UndersideZ = -26.0;
		Belly.MinCm = 2.0;
		Points.Add(Belly);
		const FVector2D	  Wanted(3.0, 0.0);
		const FAllocation A = RammsLevel::Allocate(Legs, Points, Wanted, Tuning, 0.0, 0.0);
		TestTrue(TEXT("underside: cut back, and by it"), A.Fraction < 0.55 && A.Limit == EReachLimit::Underside);
		TestTrue(TEXT("underside: no reach left"), A.ReachLeft < 0.02 && A.NearestLimit == EReachLimit::Underside);
		TestTrue(TEXT("underside: no room left"), A.MinMarginCm < 0.02);
		TestTrue(TEXT("underside: AllocateReach names it"), RammsLevel::AllocateReach(Legs, Points, Wanted, Tuning, 0.0).Limit == EReachLimit::Underside);
		const FAllocation Away = RammsLevel::Allocate(Legs, Points, FVector2D(-3.0, 0.0), Tuning, 0.0, 0.0);
		TestTrue(TEXT("underside: the ground falling away from it uses nothing of it"), Away.NearestLimit != EReachLimit::Underside);
	}
	{
		// An axis' own limit: 4 deg of pitch allowed, 3 applied. The legs
		// have 10 cm, 7.5 planned, and use 2.36: 0.69 of that left.
		FTuning Tuning;
		Tuning.MaxCorrectionDeg = FVector2D(4.0, 15.0);
		TArray<FLegReach> Legs = SixWheels();
		Stand(Legs, Tuning);
		const FAllocation A = AllocateFor(Legs, FVector2D(3.0, 0.0), Tuning);
		CheckNear(*this, TEXT("axis limit: all of the plane fits"), A.Fraction, 1.0, Tight);
		CheckNear(*this, TEXT("axis limit: a quarter of it left"), A.ReachLeft, 0.25, 1.0e-9);
		TestTrue(TEXT("axis limit: named as the nearest"), A.NearestLimit == EReachLimit::AxisLimit);
		// The degree left, at the corners' 45 cm.
		CheckNear(*this, TEXT("axis limit: the room is the degree left, as a height at the furthest standing leg"), A.MinMarginCm, 45.0 * TanDeg(1.0), 1.0e-9);
		const FAllocation Over = AllocateFor(Legs, FVector2D(6.0, 0.0), Tuning);
		CheckNear(*this, TEXT("axis limit, asked past it: held on it"), Over.AppliedDeg.X, 4.0, Tight);
		CheckNear(*this, TEXT("axis limit, asked past it: none left"), Over.ReachLeft, 0.0, Tight);
		CheckNear(*this, TEXT("axis limit, asked past it: no room"), Over.MinMarginCm, 0.0, Tight);
		// With the limit far away it is the legs' travel that is nearest.
		Tuning.MaxCorrectionDeg = FVector2D(15.0, 15.0);
		const FAllocation Far = AllocateFor(Legs, FVector2D(3.0, 0.0), Tuning);
		TestTrue(TEXT("axis limit far off: a leg's travel is the nearest"), Far.NearestLimit == EReachLimit::LegTravel);
		CheckNear(*this, TEXT("axis limit far off: the share of the planned travel left"), Far.ReachLeft, 1.0 - 45.0 * TanDeg(3.0) / 7.5, 1.0e-9);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
