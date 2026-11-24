// Tests for the level assist's control pieces (Private/Level/RammsLevelLoop).
//
// The attitude filter and the terrain estimate; the loop's terms, signs,
// deadband, freeze and back-calculation, alone and closed round a toy base;
// the correction's slew (plane rate, leg rate, soft zone); the wanted tilt's
// goal and its rate-and-acceleration limiter; the seat trim; the drive-speed
// cap; the rest detector; the divergence mark; the gate that lets the
// ground's attitude turn only as far as the robot has travelled; and the
// integral's own narrower band. Every number is made up -- the
// made-up six-wheeled robot of the curb's tests (corners 90 cm apart fore/aft
// and 50 across, the middle pair 54 across), on 9 cm wheels -- no robot's own.
//
// Beside each test is the one change to the implementation it must catch.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Level/RammsLevelAllocate.h"
#include "Level/RammsLevelLoop.h"
#include "Level/RammsLevelTypes.h"

#if WITH_DEV_AUTOMATION_TESTS

// Named, not anonymous: the other level tests share a unity file and may
// well call their own helpers the same.
namespace RammsLevelLoopTests
{
	using namespace RammsLevel;

	void Near(FAutomationTestBase& Test, const FString& What, double Actual, double Expected, double Tolerance)
	{
		Test.TestNearlyEqual(What, Actual, Expected, Tolerance);
	}

	void NearBoth(FAutomationTestBase& Test, const FString& What, const FVector2D& Actual, const FVector2D& Expected, double Tolerance)
	{
		Test.TestNearlyEqual(What + TEXT(" (pitch)"), Actual.X, Expected.X, Tolerance);
		Test.TestNearlyEqual(What + TEXT(" (roll)"), Actual.Y, Expected.Y, Tolerance);
	}

	/** The made-up robot's wheels (chassis x, y): front, centre, rear, each side. */
	const FVector2D WheelXY[6] = { { 45.0, -25.0 }, { 45.0, 25.0 }, { 0.0, -27.0 }, { 0.0, 27.0 }, { -45.0, -25.0 }, { -45.0, 25.0 } };

	/** The six legs, all standing, each with UpCm of travel above its baseline
	 *  and DownCm below, and RateCmS of rate (0: not limited). */
	TArray<FLegReach> SixLegs(double UpCm, double DownCm, double RateCmS)
	{
		TArray<FLegReach> Out;
		for (const FVector2D& P : WheelXY)
		{
			FLegReach& L = Out.AddDefaulted_GetRef();
			L.XY = P;
			L.BaseZ = -30.0;
			L.MeasuredZ = L.BaseZ;
			L.MinZ = L.BaseZ - DownCm;
			L.MaxZ = L.BaseZ + UpCm;
			L.MaxRateCmS = RateCmS;
			L.RadiusCm = 9.0;
		}
		return Out;
	}

	/** How a run of the toy base went (pitch only, deg). */
	struct FToyRun
	{
		/** The base's pitch at the end, and its least and largest on the way. */
		double	  Final = 0.0;
		double	  Least = 0.0;
		double	  Most = 0.0;
		FVector2D Integral = FVector2D::ZeroVector;
	};

	/**
	 * A toy base in pitch: the ground under it tilts by Ground(t), the legs
	 * take out what they have achieved, and what they have achieved follows
	 * the loop's correction with a first-order lag (stepped at 1 ms; none for
	 * LagSeconds 0). Gain is how much attitude a degree of achieved correction
	 * really takes out, where the joints report a whole degree. The loop and
	 * the terrain estimate run once a frame, the wanted attitude level.
	 */
	FToyRun RunToy(double FrameSeconds, double Seconds, TFunctionRef<double(double)> Ground, double LagSeconds, double Gain, const FTuning& Tuning)
	{
		FTerrainEstimate Terrain;
		FLevelLoop		 Loop;
		FToyRun			 Out;
		double			 Achieved = 0.0;
		double			 Time = 0.0;
		const int32		 Frames = static_cast<int32>(FMath::RoundToDouble(Seconds / FrameSeconds));
		const int32		 Substeps = FMath::Max(1, static_cast<int32>(FMath::RoundToDouble(FrameSeconds / 0.001)));
		for (int32 Frame = 0; Frame < Frames; ++Frame)
		{
			const FVector2D Attitude(Ground(Time) - Gain * Achieved, 0.0);
			const FVector2D Estimate = Terrain.Tick(Attitude, FVector2D(Achieved, 0.0), FrameSeconds, Tuning.TerrainFilterSeconds);
			const FVector2D Correction = Loop.Tick(Attitude, FVector2D::ZeroVector, Estimate, FrameSeconds, true, false, Tuning);
			if (LagSeconds > 0.0)
			{
				const double Share = FMath::Min(1.0, FrameSeconds / Substeps / LagSeconds);
				for (int32 Sub = 0; Sub < Substeps; ++Sub)
				{
					Achieved += (Correction.X - Achieved) * Share;
				}
			}
			else
			{
				Achieved = Correction.X;
			}
			Time += FrameSeconds;
			const double Now = Ground(Time) - Gain * Achieved;
			Out.Final = Now;
			Out.Least = Frame == 0 ? Now : FMath::Min(Out.Least, Now);
			Out.Most = Frame == 0 ? Now : FMath::Max(Out.Most, Now);
		}
		Out.Integral = Loop.GetIntegral();
		return Out;
	}

	/**
	 * The speed scale after ten seconds of a robot driven with its stick
	 * asking StickCmS: its speed is that times the scale as its drive has
	 * answered it (the cap's own lag), and the ground turns under it at
	 * TurnAtStickDegPerSec for that stick speed and in proportion at less.
	 * In's SpeedCmS, GroundRateDegPerSec and bGroundRateKnown are filled here.
	 */
	double DrivenScale(FSpeedScale& Scale, FSpeedCapInputs In, double StickCmS, const FVector2D& TurnAtStickDegPerSec, const FTuning& Tuning)
	{
		const double Dt = 0.1;
		double		 Driven = Scale.Get();
		double		 Out = Driven;
		for (int32 i = 0; i < 100; ++i)
		{
			Driven += (Scale.Get() - Driven) * FMath::Min(1.0, Dt / Scale.Cap.DriveLagSeconds);
			In.SpeedCmS = StickCmS * Driven;
			In.GroundRateDegPerSec = TurnAtStickDegPerSec * Driven;
			In.bGroundRateKnown = true;
			Out = Scale.Tick(In, Dt, Tuning);
		}
		return Out;
	}

	/** The speed scale after ten seconds of the same inputs. */
	double SettledScale(FSpeedScale& Scale, const FSpeedCapInputs& In, const FTuning& Tuning)
	{
		double Out = Scale.Get();
		for (int32 i = 0; i < 100; ++i)
		{
			Out = Scale.Tick(In, 0.1, Tuning);
		}
		return Out;
	}

	/** The speed cap told of an error and nothing else. */
	FSpeedCapInputs CapError(double ErrorDeg, bool bPastReach = false)
	{
		FSpeedCapInputs In;
		In.ErrorDeg = ErrorDeg;
		In.bPastReach = bPastReach;
		return In;
	}

	/** The speed cap told of a slope under legs SpanCm apart that turn the
	 *  correction at LegsDegPerSec. */
	FSpeedCapInputs CapSlope(double SlopeDeg, double SpanCm, double LegsDegPerSec)
	{
		FSpeedCapInputs In;
		In.SlopeDeg = SlopeDeg;
		In.SpanCm = SpanCm;
		In.LegsDegPerSec = LegsDegPerSec;
		return In;
	}

	/** The speed cap as it was before it led the drive down and left a steady
	 *  slope alone: slowing all along a slope for its end, a fall straight to
	 *  its target, no word on how steadily the drive is asked. For the checks
	 *  of the terms' own forms, which those would otherwise shape. */
	FSpeedScale PlainCap(bool bSlopeEnd)
	{
		FSpeedScale Scale;
		Scale.Cap.bSlopeEnd = bSlopeEnd;
		Scale.Cap.LeadCmS = 0.0;
		Scale.Cap.LeadShare = 0.0;
		Scale.Cap.OverspeedGain = 0.0;
		Scale.Cap.FallSeconds = 0.0;
		Scale.Cap.SteadyAskCmS2 = 1.0e9;
		Scale.Cap.UnsteadyAskCmS2 = 2.0e9;
		Scale.Cap.SteadySpeedCmS2 = 1.0e9;
		Scale.Cap.UnsteadySpeedCmS2 = 2.0e9;
		return Scale;
	}

	/** Ticks of a base at rest with the given error; returns the last answer. */
	bool RestFor(FQuiet& Quiet, int32 Ticks, double ErrorDeg, const FTuning& Tuning)
	{
		bool bOut = Quiet.IsQuiet();
		for (int32 i = 0; i < Ticks; ++i)
		{
			bOut = Quiet.Tick(0.0, ErrorDeg, false, true, 0.1, Tuning);
		}
		return bOut;
	}
} // namespace RammsLevelLoopTests

// ---- the filter and the terrain estimate ----

// Catches: the first reading blended in from zero; the share not Dt / Seconds;
// a Dt of 0 moving the value.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelTerrainFilterForm,
	"Ramms.Level.Terrain.FilterIsTheCurbsForm",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelTerrainFilterForm::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	FAttitudeFilter Filter;
	TestFalse(TEXT("not seeded before a reading"), Filter.IsSeeded());
	// Even with no time passing, a filter with nothing in it takes the reading.
	NearBoth(*this, TEXT("the first reading is taken whole"), Filter.Tick(FVector2D(3.0, -2.0), 0.0, 0.15), FVector2D(3.0, -2.0), 1e-12);
	TestTrue(TEXT("seeded after it"), Filter.IsSeeded());
	NearBoth(*this, TEXT("a third of the gap in a third of the time constant"), Filter.Tick(FVector2D(6.0, 1.0), 0.05, 0.15), FVector2D(4.0, -1.0), 1e-9);
	NearBoth(*this, TEXT("no time, no change"), Filter.Tick(FVector2D(60.0, 60.0), 0.0, 0.15), FVector2D(4.0, -1.0), 1e-12);
	NearBoth(*this, TEXT("a step longer than the time constant takes the reading, no more"), Filter.Tick(FVector2D(5.0, 0.0), 1.0, 0.15), FVector2D(5.0, 0.0), 1e-12);
	NearBoth(*this, TEXT("no time constant: the reading"), Filter.Tick(FVector2D(1.0, 2.0), 0.01, 0.0), FVector2D(1.0, 2.0), 1e-12);
	Filter.Reset();
	TestFalse(TEXT("Reset forgets"), Filter.IsSeeded());

	// The terrain estimate is the same filter on A + c_meas.
	FTerrainEstimate Terrain;
	Terrain.Seed(FVector2D::ZeroVector);
	NearBoth(*this, TEXT("the ground steps 3 deg: a third of it after a third of the time constant"),
		Terrain.Tick(FVector2D(1.0, 0.0), FVector2D(2.0, 0.0), 0.05, 0.15), FVector2D(1.0, 0.0), 1e-9);
	return true;
}

// Catches: the sign of c_meas (G = A - c_meas), or c_meas left out.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelTerrainInvariantToCorrection,
	"Ramms.Level.Terrain.InvariantToCorrection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelTerrainInvariantToCorrection::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	// The same ground under whatever plane the legs hold: the base reads the
	// ground less the plane, and the estimate must not move.
	const FVector2D	 Ground(5.0, -3.0);
	const FVector2D	 Planes[] = { { 0.0, 0.0 }, { 5.0, -3.0 }, { 2.0, 4.0 }, { -6.0, 1.5 } };
	FTerrainEstimate Terrain;
	for (const FVector2D& Plane : Planes)
	{
		const FVector2D G = Terrain.Tick(Ground - Plane, Plane, 0.02, 0.15);
		NearBoth(*this, FString::Printf(TEXT("plane (%.1f, %.1f): the ground as it is"), Plane.X, Plane.Y), G, Ground, 1e-9);
	}
	return true;
}

// Catches: the estimate built from the correction asked for (the loop's C)
// in place of the one handed in as achieved.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelTerrainInvariantToSag,
	"Ramms.Level.Terrain.InvariantToSag",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelTerrainInvariantToSag::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	// The legs are asked for a plane that grows, and give 70 % of it (sag, a
	// rate limit, a stop). The base reads the ground less what they gave.
	const FVector2D	 Ground(4.0, 2.0);
	FTerrainEstimate Terrain;
	for (int32 i = 0; i <= 20; ++i)
	{
		const FVector2D Asked = FVector2D(4.0, 2.0) * (i / 20.0);
		const FVector2D Given = Asked * 0.7;
		const FVector2D G = Terrain.Tick(Ground - Given, Given, 0.02, 0.15);
		NearBoth(*this, FString::Printf(TEXT("tick %d: the ground, however far the legs are behind"), i), G, Ground, 1e-9);
	}
	return true;
}

// Catches: MeasuredCorrection reading the baseline or a command where it
// should read MeasuredZ, or letting a common height into the slopes.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelTerrainUsesMeasured,
	"Ramms.Level.Terrain.UsesMeasuredNotCommanded",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelTerrainUsesMeasured::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	// The legs were told (4, -2) and stand at (2.5, -1.5), all 0.7 cm low
	// besides. The estimate is fed from the legs as they measure.
	const FVector2D	  Ground(3.0, -1.0);
	const FVector2D	  Given(2.5, -1.5);
	TArray<FLegReach> Legs = SixLegs(20.0, 20.0, 0.0);
	for (FLegReach& L : Legs)
	{
		L.MeasuredZ = L.BaseZ + PlaneRise(L.XY, true, Given) - 0.7;
	}
	const FVector2D Achieved = MeasuredCorrection(Legs);
	NearBoth(*this, TEXT("the correction the joints achieved"), Achieved, Given, 1e-6);
	FTerrainEstimate Terrain;
	NearBoth(*this, TEXT("the ground, from the measured legs"), Terrain.Tick(Ground - Given, Achieved, 0.02, 0.15), Ground, 1e-6);
	return true;
}

// ---- the loop ----

// Catches: a term's sign or gain (Kp, Ki Dt, Kff (G - W)); the integral's
// clamp; integrating with bIntegrate false; a Dt of 0 integrating.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopTerms,
	"Ramms.Level.Loop.TermsAndSigns",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopTerms::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning Tuning;
	{
		// 1.15 deg nose up of the wanted, 0.1 s: the error past the deadband
		// is 1.0, the integral 2 * 1.0 * 0.1, the proportional term 0.3. Roll
		// the other way.
		FLevelLoop		Loop;
		const FVector2D C = Loop.Tick(FVector2D(1.65, -0.85), FVector2D(0.5, 0.3), FVector2D(0.5, 0.3), 0.1, true, false, Tuning);
		NearBoth(*this, TEXT("nose up, +Y down: Ki e dt + Kp e, each its own sign"), C, FVector2D(0.5, -0.5), 1e-9);
		NearBoth(*this, TEXT("the integral"), Loop.GetIntegral(), FVector2D(0.2, -0.2), 1e-9);
		NearBoth(*this, TEXT("GetCorrection is what Tick returned"), Loop.GetCorrection(), C, 1e-12);
		NearBoth(*this, TEXT("no time: the last answer"), Loop.Tick(FVector2D(9.0, 9.0), FVector2D::ZeroVector, FVector2D(5.0, 5.0), 0.0, true, false, Tuning), C, 1e-12);
		NearBoth(*this, TEXT("no time: the integral where it was"), Loop.GetIntegral(), FVector2D(0.2, -0.2), 1e-12);
	}
	{
		// The feed-forward alone: on the wanted attitude, the correction is
		// the ground's tilt off the wanted one.
		FLevelLoop		Loop;
		const FVector2D C = Loop.Tick(FVector2D(1.0, 0.0), FVector2D(1.0, 0.0), FVector2D(5.0, -2.0), 0.1, true, false, Tuning);
		NearBoth(*this, TEXT("Kff (G - W)"), C, FVector2D(4.0, -2.0), 1e-9);
		FTuning Half;
		Half.FeedForwardGain = 0.5;
		FLevelLoop Other;
		NearBoth(*this, TEXT("at half the gain"), Other.Tick(FVector2D(1.0, 0.0), FVector2D(1.0, 0.0), FVector2D(5.0, -2.0), 0.1, true, false, Half), FVector2D(2.0, -1.0), 1e-9);
	}
	{
		// A long error: the integral stops at its limit, per axis.
		FLevelLoop Loop;
		for (int32 i = 0; i < 200; ++i)
		{
			Loop.Tick(FVector2D(4.0, -0.65), FVector2D::ZeroVector, FVector2D::ZeroVector, 0.1, true, false, Tuning);
		}
		Near(*this, TEXT("the integral at its limit"), Loop.GetIntegral().X, 3.0, 1e-9);
		TestTrue(TEXT("roll's integral at its own limit, the other way"), FMath::IsNearlyEqual(Loop.GetIntegral().Y, -3.0, 1e-9));
	}
	{
		// Not allowed to integrate: the proportional term only.
		FLevelLoop		Loop;
		const FVector2D C = Loop.Tick(FVector2D(1.15, 0.0), FVector2D::ZeroVector, FVector2D::ZeroVector, 0.1, false, false, Tuning);
		Near(*this, TEXT("bIntegrate false: Kp e alone"), C.X, 0.3, 1e-9);
		Near(*this, TEXT("bIntegrate false: no integral"), Loop.GetIntegral().X, 0.0, 1e-12);
	}
	return true;
}

// Catches: the deadband left out of the error (the integral then creeps on
// noise and the legs never rest).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopDeadband,
	"Ramms.Level.Loop.DeadbandHolds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopDeadband::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning Tuning;
	FLevelLoop	  Loop;
	FVector2D	  C = FVector2D::ZeroVector;
	for (int32 i = 0; i < 100; ++i)
	{
		C = Loop.Tick(FVector2D(0.1, -0.14), FVector2D::ZeroVector, FVector2D(2.0, 1.0), 0.02, true, false, Tuning);
	}
	NearBoth(*this, TEXT("inside the band for 2 s: the feed-forward and nothing else"), C, FVector2D(2.0, 1.0), 1e-12);
	NearBoth(*this, TEXT("inside the band: no integral"), Loop.GetIntegral(), FVector2D::ZeroVector, 1e-12);
	C = Loop.Tick(FVector2D(0.25, 0.0), FVector2D::ZeroVector, FVector2D(2.0, 1.0), 0.1, true, false, Tuning);
	Near(*this, TEXT("0.1 deg past the band: only that 0.1 counts"), C.X, 2.0 + 2.0 * 0.1 * 0.1 + 0.3 * 0.1, 1e-9);
	return true;
}

// Catches: bSaturated ignored (the integral winds into a limit); and, for the
// unwind, the freeze applied to both directions or decided from the error
// in place of the last correction's sign.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopFreezes,
	"Ramms.Level.Loop.FreezesIntoSaturationUnwindsAtOnce",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopFreezes::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning	Tuning;
	const FVector2D Zero = FVector2D::ZeroVector;
	FLevelLoop		Loop;
	// One free tick: the correction is (0.5, -0.5), the integral (0.2, -0.2).
	Loop.Tick(FVector2D(1.15, -1.15), Zero, Zero, 0.1, true, false, Tuning);
	for (int32 i = 0; i < 10; ++i)
	{
		Loop.Tick(FVector2D(1.15, -1.15), Zero, Zero, 0.1, true, true, Tuning);
	}
	NearBoth(*this, TEXT("saturated, the error pushing further in: the integral stays"), Loop.GetIntegral(), FVector2D(0.2, -0.2), 1e-9);

	// Roll's error turns round while pitch's does not: each axis by itself.
	Loop.Tick(FVector2D(1.15, 1.15), Zero, Zero, 0.1, true, true, Tuning);
	NearBoth(*this, TEXT("saturated: pitch still frozen, roll unwinding the same tick"), Loop.GetIntegral(), FVector2D(0.2, 0.0), 1e-9);

	// Pitch's turns round: it comes back on that very tick...
	const FVector2D C = Loop.Tick(FVector2D(-1.15, 0.0), Zero, Zero, 0.1, true, true, Tuning);
	Near(*this, TEXT("saturated, the error coming back: the integral unwinds at once"), Loop.GetIntegral().X, 0.0, 1e-9);
	Near(*this, TEXT("... and the correction is now the other way"), C.X, -0.3, 1e-9);
	// ... and is then pushing into the other limit, so it freezes again.
	Loop.Tick(FVector2D(-1.15, 0.0), Zero, Zero, 0.1, true, true, Tuning);
	Near(*this, TEXT("then frozen the other way"), Loop.GetIntegral().X, 0.0, 1e-9);

	// An axis' own limit holds that axis and no other. Roll may give 2 deg
	// and the ground asks 5 of it: roll's integral does not wind against the
	// limit (which no allocation reports), while pitch's goes on trimming.
	{
		FTuning Capped;
		Capped.MaxCorrectionDeg = FVector2D(15.0, 2.0);
		const FVector2D Ground(0.0, 5.0);
		FLevelLoop		Axis;
		for (int32 i = 0; i < 10; ++i)
		{
			Axis.Tick(FVector2D(1.15, 1.15), Zero, Ground, 0.1, true, false, Capped);
		}
		// The first tick finds no last correction, so roll takes that one step.
		NearBoth(*this, TEXT("roll past its limit: its integral stays, pitch's winds on"), Axis.GetIntegral(), FVector2D(2.0, 0.2), 1e-9);
		Axis.Tick(FVector2D(1.15, -1.15), Zero, Ground, 0.1, true, false, Capped);
		Near(*this, TEXT("... and comes back at once when the error turns"), Axis.GetIntegral().Y, 0.0, 1e-9);
	}
	return true;
}

// Catches: the back-calculation's sign, the integral taken past zero to
// make up for what the feed-forward asked beyond a limit (it would hold the
// correction short when the limit lets go), or C_raw not re-made from the
// new integral; StartFrom putting the whole plane in the integral (the plane
// applied twice at a feed-forward gain of 1).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopBackCalculate,
	"Ramms.Level.Loop.BackCalculateAndStartFrom",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopBackCalculate::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning	Tuning;
	const FVector2D Zero = FVector2D::ZeroVector;
	{
		// Asked 4.5 (4 of feed-forward, 0.2 of integral, 0.3 proportional).
		// Given 4.45: the 0.05 cut is the integral's to give up.
		FLevelLoop Loop;
		Near(*this, TEXT("asked"), Loop.Tick(FVector2D(1.15, 0.0), Zero, FVector2D(4.0, 0.0), 0.1, true, false, Tuning).X, 4.5, 1e-9);
		Loop.BackCalculate(FVector2D(4.5, 0.0), FVector2D(4.45, 0.0), Tuning);
		Near(*this, TEXT("the integral moved by given - asked"), Loop.GetIntegral().X, 0.15, 1e-9);
		Near(*this, TEXT("the correction is now what was given"), Loop.GetCorrection().X, 4.45, 1e-9);
		// Given 3 of 4.45: more was cut than the integral asked for. It gives
		// up all it had and stops at zero; the rest was the feed-forward's.
		Loop.BackCalculate(FVector2D(4.45, 0.0), FVector2D(3.0, 0.0), Tuning);
		Near(*this, TEXT("never past zero"), Loop.GetIntegral().X, 0.0, 1e-12);
		Near(*this, TEXT("the correction: the other two terms alone"), Loop.GetCorrection().X, 4.3, 1e-9);
		// With nothing held there is nothing to give up, either way.
		Loop.BackCalculate(FVector2D(3.0, 0.0), FVector2D(5.0, 0.0), Tuning);
		Near(*this, TEXT("nothing held, nothing moved"), Loop.GetIntegral().X, 0.0, 1e-12);
	}
	{
		// Each axis by itself, and each sign: the integral is (-0.2, 0.2).
		FLevelLoop Loop;
		Loop.Tick(FVector2D(-1.15, 1.15), Zero, FVector2D(-4.0, 4.0), 0.1, true, false, Tuning);
		NearBoth(*this, TEXT("the integral to start with"), Loop.GetIntegral(), FVector2D(-0.2, 0.2), 1e-9);
		// Pitch asked -4.5 and was given -4.4: it comes back 0.1 towards zero.
		// Roll was GIVEN more than asked (a cut the other way): it does not
		// grow to meet it.
		Loop.BackCalculate(FVector2D(-4.5, 4.5), FVector2D(-4.4, 4.8), Tuning);
		NearBoth(*this, TEXT("towards zero only, per axis"), Loop.GetIntegral(), FVector2D(-0.1, 0.2), 1e-9);
	}
	{
		// A plane already applied, picked up with no step.
		const FVector2D Plane(3.0, -1.0);
		const FVector2D Wanted(0.5, 0.2);
		const FVector2D Ground(2.5, -0.3);
		FLevelLoop		Loop;
		Loop.StartFrom(Plane, Wanted, Ground, Tuning);
		NearBoth(*this, TEXT("the integral holds only what the feed-forward does not"), Loop.GetIntegral(), FVector2D(1.0, -0.5), 1e-9);
		NearBoth(*this, TEXT("the correction is the plane at once"), Loop.GetCorrection(), Plane, 1e-9);
		NearBoth(*this, TEXT("and the next tick, on the wanted attitude, asks the same plane"), Loop.Tick(Wanted, Wanted, Ground, 0.05, true, false, Tuning), Plane, 1e-9);

		// With no feed-forward the integral holds it all, as far as it may.
		FTuning NoFeed;
		NoFeed.FeedForwardGain = 0.0;
		Loop.StartFrom(FVector2D(2.0, -5.0), Wanted, Ground, NoFeed);
		NearBoth(*this, TEXT("no feed-forward: the plane, within the integral's limit"), Loop.GetIntegral(), FVector2D(2.0, -3.0), 1e-9);
	}
	return true;
}

// Catches: the feed-forward's sign (the integral, capped at 3 deg, cannot
// then hold a 4 deg slope), or the correction's sense against the base.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopStepConverges,
	"Ramms.Level.Loop.StepConverges",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopStepConverges::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning Tuning;
	// The ground is suddenly 4 deg nose up, the legs follow with a 0.1 s lag.
	const FToyRun Run = RunToy(1.0 / 60.0, 3.0, [](double) { return 4.0; }, 0.1, 1.0, Tuning);
	TestTrue(FString::Printf(TEXT("level to within the deadband after 3 s (%.3f deg)"), Run.Final), FMath::Abs(Run.Final) <= Tuning.DeadbandDeg + 0.02);
	TestTrue(FString::Printf(TEXT("and never more than a degree past level (%.3f deg)"), Run.Least), Run.Least > -1.0);
	return true;
}

// Catches: with the feed-forward, its term dropped from C_raw; without, the
// integral's gain or its Dt (the error on a ramp is exactly rate / Ki).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopRampError,
	"Ramms.Level.Loop.RampErrorSmallWithFeedForwardRateOverKiWithout",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopRampError::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	// The ground tilts at 1 deg/s for 6 s under legs with no lag. The band is
	// taken out so the figures are the loop's own.
	FTuning Tuning;
	Tuning.DeadbandDeg = 0.0;
	Tuning.IntegralMaxDeg = 20.0;
	const auto Ground = [](double T) { return 1.0 * T; };

	FTuning NoFeed = Tuning;
	NoFeed.FeedForwardGain = 0.0;
	const FToyRun Without = RunToy(1.0 / 60.0, 6.0, Ground, 0.0, 1.0, NoFeed);
	Near(*this, TEXT("no feed-forward: the error on a ramp is its rate over Ki"), Without.Final, 1.0 / NoFeed.IntegralGain, 0.01);

	const FToyRun With = RunToy(1.0 / 60.0, 6.0, Ground, 0.0, 1.0, Tuning);
	TestTrue(FString::Printf(TEXT("feed-forward: no steady error (%.4f deg)"), With.Final), FMath::Abs(With.Final) < 0.01);
	TestTrue(FString::Printf(TEXT("feed-forward: never over 0.15 deg on the way (%.4f deg)"), With.Most), With.Most < 0.15 && With.Least > -0.15);
	return true;
}

// Catches: the integral not accumulating (the proportional term alone leaves
// over 3 deg of a 4 deg slope when the legs give 15 % less than modelled).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopModelError,
	"Ramms.Level.Loop.ModelErrorTrimmedByIntegral",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopModelError::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const auto Ground = [](double) { return 4.0; };
	{
		// The default law. Each degree the joints report takes out 0.85.
		const FTuning Tuning;
		const FToyRun Run = RunToy(1.0 / 60.0, 8.0, Ground, 0.1, 0.85, Tuning);
		TestTrue(FString::Printf(TEXT("feed-forward and trim: the residual under the deadband (%.3f deg)"), Run.Final), FMath::Abs(Run.Final) <= Tuning.DeadbandDeg + 0.01);
	}
	{
		// The configured fallback, integral only: it has to find all 4 / 0.85.
		FTuning Tuning;
		Tuning.FeedForwardGain = 0.0;
		Tuning.IntegralMaxDeg = 20.0;
		const FToyRun Run = RunToy(1.0 / 60.0, 8.0, Ground, 0.1, 0.85, Tuning);
		TestTrue(FString::Printf(TEXT("integral only: the residual under the deadband (%.3f deg)"), Run.Final), FMath::Abs(Run.Final) <= Tuning.DeadbandDeg + 0.01);
		TestTrue(FString::Printf(TEXT("integral only: the integral carries the correction (%.3f deg)"), Run.Integral.X), Run.Integral.X > 4.0);
	}
	return true;
}

// Catches: the integral stepped per tick and not per second (I += Ki e),
// which rings at either rate; anything that makes the answer depend on the
// frame rate.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopFrameRates,
	"Ramms.Level.Loop.StableAt16And60Fps",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopFrameRates::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning  Tuning;
	const auto	   Ground = [](double) { return 5.0; };
	const FToyRun  Slow = RunToy(1.0 / 16.0, 6.0, Ground, 0.1, 1.0, Tuning);
	const FToyRun  Fast = RunToy(1.0 / 60.0, 6.0, Ground, 0.1, 1.0, Tuning);
	const FToyRun* Runs[] = { &Slow, &Fast };
	for (const FToyRun* Run : Runs)
	{
		const TCHAR* Which = Run == &Slow ? TEXT("16 fps") : TEXT("60 fps");
		TestTrue(FString::Printf(TEXT("%s: level to within the deadband after 6 s (%.3f deg)"), Which, Run->Final), FMath::Abs(Run->Final) <= Tuning.DeadbandDeg + 0.01);
		TestTrue(FString::Printf(TEXT("%s: at most a degree past level on the way (%.3f deg)"), Which, Run->Least), Run->Least > -1.0);
	}
	Near(*this, TEXT("the two rates end in the same place"), Slow.Final, Fast.Final, 0.01);
	Near(*this, TEXT("and pass level by about as much"), Slow.Least, Fast.Least, 0.25);
	return true;
}

// Catches: the leg-rate cut applied leg by leg (the plane bends) or to one
// axis; the rate use reported after the cut and so never over 1; the deg/s
// cap applied per axis.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopRateLimited,
	"Ramms.Level.Loop.RateLimitedByLegRate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopRateLimited::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning	Tuning;
	const FVector2D Zero = FVector2D::ZeroVector;
	const double	Dt = 0.1;
	double			Use = -1.0;
	bool			bLimited = false;
	{
		// Legs with no rate of their own: only the plane's 10 deg/s counts,
		// as a length over pitch and roll together.
		const TArray<FLegReach> Legs = SixLegs(20.0, 20.0, 0.0);
		const FVector2D			C = SlewCorrection(Legs, Zero, FVector2D(6.0, 8.0), Dt, 10.0, Tuning, Use, bLimited);
		NearBoth(*this, TEXT("1 deg of the way, along the way"), C, FVector2D(0.6, 0.8), 1e-9);
		TestTrue(TEXT("limited by the plane's rate"), bLimited);
		Near(*this, TEXT("no leg has a rate: no use"), Use, 0.0, 1e-12);
	}
	{
		// Legs at 2 cm/s: the same step would ask the busiest corner for
		// several times that.
		const TArray<FLegReach> Legs = SixLegs(20.0, 20.0, 2.0);
		const FVector2D			Whole(0.6, 0.8);
		double					Busiest = 0.0;
		for (const FLegReach& L : Legs)
		{
			Busiest = FMath::Max(Busiest, FMath::Abs(PlaneRise(L.XY, true, Whole)));
		}
		const FVector2D C = SlewCorrection(Legs, Zero, FVector2D(6.0, 8.0), Dt, 10.0, Tuning, Use, bLimited);
		Near(*this, TEXT("the use is measured before the cut, and passes 1"), Use, Busiest / (Dt * 2.0), 1e-6);
		TestTrue(TEXT("(it does pass 1 here)"), Use > 3.0);
		TestTrue(TEXT("limited"), bLimited);
		Near(*this, TEXT("the plane keeps its direction: rigid"), C.X * 0.8 - C.Y * 0.6, 0.0, 1e-9);
		double Moved = 0.0;
		for (const FLegReach& L : Legs)
		{
			Moved = FMath::Max(Moved, FMath::Abs(PlaneRise(L.XY, true, C)));
		}
		Near(*this, TEXT("the busiest leg moves 0.8 of its rate"), Moved, Tuning.LegRateFraction * 2.0 * Dt, 1e-4);

		// A step every leg can make is made whole.
		const FVector2D Small = SlewCorrection(Legs, Zero, FVector2D(0.05, 0.0), Dt, 10.0, Tuning, Use, bLimited);
		NearBoth(*this, TEXT("a small step, whole"), Small, FVector2D(0.05, 0.0), 1e-12);
		TestFalse(TEXT("not limited"), bLimited);
		Near(*this, TEXT("its use"), Use, 45.0 * FMath::Tan(FMath::DegreesToRadians(0.05)) / (Dt * 2.0), 1e-6);

		// No time: nothing moves.
		Use = -1.0;
		bLimited = true;
		NearBoth(*this, TEXT("no time: the plane it had"), SlewCorrection(Legs, FVector2D(1.0, 2.0), FVector2D(6.0, 8.0), 0.0, 10.0, Tuning, Use, bLimited), FVector2D(1.0, 2.0), 1e-12);
		TestFalse(TEXT("no time: not limited"), bLimited);
		Near(*this, TEXT("no time: no use"), Use, 0.0, 1e-12);
	}
	return true;
}

// Catches: the taper applied to motion away from the limit, measured from the
// true stop and not the margin, or without its floor (the plane would stick
// at the limit, or meet it at full rate).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopSoftZone,
	"Ramms.Level.Loop.SoftZoneTapers",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopSoftZone::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	// The front legs can retract 1.5 cm, of which the margin keeps 0.5: their
	// limit is 1 cm up. Everything else has room to spare.
	const FTuning	  Tuning;
	TArray<FLegReach> Legs = SixLegs(20.0, 20.0, 0.0);
	Legs[0].MaxZ = Legs[0].BaseZ + 1.5;
	Legs[1].MaxZ = Legs[1].BaseZ + 1.5;
	const double Dt = 0.1;
	double		 Use = 0.0;
	bool		 bLimited = false;
	{
		// From level, nose-up correction (the front legs retract): 1 cm to
		// go, half the soft zone, so 0.1 + 0.9 / 2 of the 1 deg step.
		const FVector2D C = SlewCorrection(Legs, FVector2D::ZeroVector, FVector2D(5.0, 0.0), Dt, 10.0, Tuning, Use, bLimited);
		NearBoth(*this, TEXT("half way into the soft zone: 0.55 of the step"), C, FVector2D(0.55, 0.0), 1e-9);
		TestTrue(TEXT("limited, by the plane's rate (5 deg asked, 1 deg a tick)"), bLimited);
	}
	{
		// A step no rate cuts, tapered all the same: the taper is not a rate
		// limit, and must not read as one (the law holds its integral while
		// rate-limited, and the taper lasts the whole approach to a limit).
		const FVector2D C = SlewCorrection(Legs, FVector2D::ZeroVector, FVector2D(0.4, 0.0), Dt, 10.0, Tuning, Use, bLimited);
		NearBoth(*this, TEXT("a small step in the soft zone: 0.55 of it"), C, FVector2D(0.22, 0.0), 1e-9);
		TestFalse(TEXT("... and not reported as rate-limited"), bLimited);
	}
	{
		// The other way the front legs extend and the rear ones have room.
		const FVector2D C = SlewCorrection(Legs, FVector2D::ZeroVector, FVector2D(-0.5, 0.0), Dt, 10.0, Tuning, Use, bLimited);
		NearBoth(*this, TEXT("away from the limit: the whole step"), C, FVector2D(-0.5, 0.0), 1e-12);
		TestFalse(TEXT("not limited"), bLimited);
	}
	{
		// On the limit: a tenth, never nothing; and away from it freely.
		const double	OnLimitDeg = FMath::RadiansToDegrees(FMath::Atan(1.0 / 45.0));
		const FVector2D From(OnLimitDeg, 0.0);
		const FVector2D In = SlewCorrection(Legs, From, FVector2D(5.0, 0.0), Dt, 10.0, Tuning, Use, bLimited);
		Near(*this, TEXT("on the limit: the floor, a tenth of the step"), In.X - OnLimitDeg, 0.1, 1e-6);
		const FVector2D Out = SlewCorrection(Legs, From, FVector2D(OnLimitDeg - 0.5, 0.0), Dt, 10.0, Tuning, Use, bLimited);
		Near(*this, TEXT("on the limit, coming away: the whole step"), Out.X - OnLimitDeg, -0.5, 1e-9);
		TestFalse(TEXT("coming away is not limited"), bLimited);
	}
	{
		// A lifted leg near its stop slows nothing: it stops there quietly.
		Legs[0].bSupport = false;
		Legs[1].bSupport = false;
		const FVector2D C = SlewCorrection(Legs, FVector2D::ZeroVector, FVector2D(0.5, 0.0), Dt, 10.0, Tuning, Use, bLimited);
		// (With the front pair lifted the standing legs' mean is no longer
		// zero, but they all have room, so the step is whole.)
		NearBoth(*this, TEXT("lifted legs do not taper the plane"), C, FVector2D(0.5, 0.0), 1e-12);
	}
	return true;
}

// Catches: a large error that merely stays taken for a divergence (a
// correction held at its limit on a slope past it would fault for good), the
// evidence's sign turned (a working loop would fault and a backwards one
// not), the timer not starting again when the error comes back (a kick being
// taken out would fault), or not latching.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopDivergence,
	"Ramms.Level.Loop.DivergenceNeedsAnErrorThatStays",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopDivergence::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning	Tuning; // 1.5 s
	const FVector2D Zero = FVector2D::ZeroVector;
	{
		// Wired backwards: the correction grows by 0.1 deg a tick and the
		// base goes with it, degree for degree.
		FDivergence Mark;
		for (int32 i = 0; i < 14; ++i)
		{
			Mark.Tick(4.0 + 0.1 * i, FVector2D(4.0 + 0.1 * i, 0.0), FVector2D(0.1 * i, 0.0), true, 0.1, Tuning);
		}
		TestFalse(TEXT("growing with the legs for 1.4 s: not yet"), Mark.HasDiverged());
		TestTrue(TEXT("for 1.5 s: diverged"), Mark.Tick(5.4, FVector2D(5.4, 0.0), FVector2D(1.4, 0.0), true, 0.1, Tuning));
		TestTrue(TEXT("and it stays, whatever follows"), Mark.Tick(0.0, Zero, Zero, false, 0.1, Tuning));
		Mark.Reset();
		TestFalse(TEXT("until Reset"), Mark.HasDiverged());
	}
	{
		// The same, but it ran into a limit after 0.4 s and has stood there
		// since: the 0.4 deg it went with the legs is under the degree asked.
		// Then a second run of 0.7 deg makes it up.
		FDivergence Mark;
		for (int32 i = 0; i < 40; ++i)
		{
			const double C = 0.1 * FMath::Min(i, 4);
			Mark.Tick(4.0 + C, FVector2D(4.0 + C, 0.0), FVector2D(C, 0.0), true, 0.1, Tuning);
		}
		TestFalse(TEXT("under a degree with the legs: not a divergence, however long"), Mark.HasDiverged());
		for (int32 i = 1; i <= 7; ++i)
		{
			const double C = 0.4 + 0.1 * i;
			Mark.Tick(4.0 + C, FVector2D(4.0 + C, 0.0), FVector2D(C, 0.0), true, 0.1, Tuning);
		}
		TestTrue(TEXT("a degree with the legs, the timer long run: diverged"), Mark.HasDiverged());
	}
	{
		// A correction held at its limit on a slope past it: the error grows
		// as the wanted attitude gives way, then stands. The legs never
		// move, so nothing says they are wired wrong.
		FDivergence Mark;
		for (int32 i = 0; i < 100; ++i)
		{
			Mark.Tick(FMath::Min(3.5 + 0.1 * i, 6.0), FVector2D(0.0, 9.0), FVector2D(0.0, 6.0), true, 0.1, Tuning);
		}
		TestFalse(TEXT("large and steady with the legs still: not a divergence"), Mark.HasDiverged());
	}
	{
		// Wired the right way and losing all the same (the ground turning
		// under it): the base moves AGAINST each move of the correction.
		FDivergence Mark;
		for (int32 i = 0; i < 40; ++i)
		{
			Mark.Tick(4.0 + 0.05 * i, FVector2D(4.0 - 0.1 * i, 0.0), FVector2D(0.1 * i, 0.0), true, 0.1, Tuning);
		}
		TestFalse(TEXT("the base moving against its correction: not a divergence"), Mark.HasDiverged());
	}
	{
		// A roll runaway the other way round: the evidence is along the
		// correction's own direction, whatever its axis and sign.
		FDivergence Mark;
		for (int32 i = 0; i < 16; ++i)
		{
			Mark.Tick(4.0 + 0.1 * i, FVector2D(0.0, -4.0 - 0.1 * i), FVector2D(0.0, -0.1 * i), true, 0.1, Tuning);
		}
		TestTrue(TEXT("roll, negative: diverged"), Mark.HasDiverged());
	}
	{
		// A kick being taken out: large for 3 s, but always coming back.
		FDivergence Mark;
		for (int32 i = 0; i < 30; ++i)
		{
			Mark.Tick(12.0 - 0.3 * i, FVector2D(12.0 - 0.3 * i, 0.0), FVector2D(0.3 * i, 0.0), true, 0.1, Tuning);
		}
		TestFalse(TEXT("an error on its way back is not a divergence"), Mark.HasDiverged());
	}
	{
		// Going with the legs all the while, but the legs had no room on
		// some ticks: each of those ends the episode, evidence and all.
		FDivergence Mark;
		for (int32 i = 0; i < 100; ++i)
		{
			Mark.Tick(6.0, FVector2D(0.1 * i, 0.0), FVector2D(0.1 * i, 0.0), i % 10 != 0, 0.1, Tuning);
		}
		TestFalse(TEXT("ticks that do not count start it all again"), Mark.HasDiverged());
		// Under the threshold it never counts.
		for (int32 i = 0; i < 100; ++i)
		{
			Mark.Tick(3.0, FVector2D(0.1 * i, 0.0), FVector2D(0.1 * i, 0.0), true, 0.1, Tuning);
		}
		TestFalse(TEXT("3 deg is not over 3 deg"), Mark.HasDiverged());
		// No time passes: no count.
		for (int32 i = 0; i < 100; ++i)
		{
			Mark.Tick(6.0, FVector2D(0.1 * i, 0.0), FVector2D(0.1 * i, 0.0), true, 0.0, Tuning);
		}
		TestFalse(TEXT("no time, no count"), Mark.HasDiverged());
	}
	return true;
}

// ---- the wanted tilt ----

// Catches: the rate or the acceleration limited per axis (a diagonal move
// would then be 1.4 times too brisk), or the velocity set straight to the
// wanted one.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelTiltTargetBounded,
	"Ramms.Level.TiltTarget.RateAndAccelBounded",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelTiltTargetBounded::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const double MaxRate = 3.0;
	const double MaxAccel = 4.0;
	const double FrameTimes[] = { 1.0 / 16.0, 1.0 / 60.0 };
	for (const double Dt : FrameTimes)
	{
		// A diagonal goal, moved somewhere else a second in.
		const FVector2D First(4.0, -3.0);
		const FVector2D Second(-2.0, 2.0);
		FTiltTarget		Target;
		Target.Reset(FVector2D::ZeroVector);
		double		Fastest = 0.0;
		double		Briskest = 0.0;
		double		WorstStep = 0.0;
		const int32 Ticks = static_cast<int32>(FMath::RoundToDouble(8.0 / Dt));
		for (int32 i = 0; i < Ticks; ++i)
		{
			const FVector2D Was = Target.Get();
			const FVector2D RateWas = Target.GetRate();
			const FVector2D Now = Target.Tick(i * Dt < 1.0 ? First : Second, Dt, MaxRate, MaxAccel);
			Fastest = FMath::Max(Fastest, Target.GetRate().Size());
			Briskest = FMath::Max(Briskest, (Target.GetRate() - RateWas).Size() / Dt);
			// What W moved by is its velocity: no step hides from the two limits.
			WorstStep = FMath::Max(WorstStep, ((Now - Was) / Dt).Size());
		}
		const FString At = FString::Printf(TEXT("%.0f fps: "), 1.0 / Dt);
		TestTrue(At + FString::Printf(TEXT("never over 3 deg/s in total (%.6f)"), Fastest), Fastest <= MaxRate + 1e-9);
		TestTrue(At + FString::Printf(TEXT("W itself never steps faster (%.6f)"), WorstStep), WorstStep <= MaxRate + 1e-9);
		TestTrue(At + TEXT("and it does get up to its rate"), Fastest >= MaxRate - 1e-6);
		TestTrue(At + FString::Printf(TEXT("never over 4 deg/s^2 in total (%.6f)"), Briskest), Briskest <= MaxAccel + 1e-6);
		TestTrue(At + TEXT("arrived on the second goal"), Target.HasArrived(Second));
		NearBoth(*this, At + TEXT("exactly on it"), Target.Get(), Second, 1e-12);
	}
	return true;
}

// Catches: the continuous stopping speed sqrt(2 a d), which is a tick late
// and passes the goal; the goal never reached exactly.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelTiltTargetNoOvershoot,
	"Ramms.Level.TiltTarget.NoOvershoot",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelTiltTargetNoOvershoot::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const double FrameTimes[] = { 0.1, 1.0 / 16.0, 1.0 / 60.0 };
	for (const double Dt : FrameTimes)
	{
		// 6 deg off level, along a diagonal, back to level.
		const FVector2D Start(4.8, -3.6);
		const FVector2D Along = Start / 6.0;
		FTiltTarget		Target;
		Target.Reset(Start);
		double		Past = 0.0;
		double		Aside = 0.0;
		double		ArrivedAt = -1.0;
		const int32 Ticks = static_cast<int32>(FMath::RoundToDouble(5.0 / Dt));
		for (int32 i = 0; i < Ticks; ++i)
		{
			const FVector2D W = Target.Tick(FVector2D::ZeroVector, Dt, 3.0, 4.0);
			// How far past level along the line, and how far off the line.
			Past = FMath::Max(Past, -(W.X * Along.X + W.Y * Along.Y));
			Aside = FMath::Max(Aside, FMath::Abs(W.X * Along.Y - W.Y * Along.X));
			if (ArrivedAt < 0.0 && Target.HasArrived(FVector2D::ZeroVector))
			{
				ArrivedAt = (i + 1) * Dt;
			}
		}
		const FString At = FString::Printf(TEXT("%.0f fps: "), 1.0 / Dt);
		TestTrue(At + FString::Printf(TEXT("never past the goal (%.9f deg)"), Past), Past <= 1e-9);
		TestTrue(At + TEXT("along the straight line"), Aside <= 1e-9);
		NearBoth(*this, At + TEXT("ends exactly on the goal"), Target.Get(), FVector2D::ZeroVector, 1e-12);
		NearBoth(*this, At + TEXT("at rest"), Target.GetRate(), FVector2D::ZeroVector, 1e-12);
		// 6 deg at 3 deg/s and 4 deg/s^2 is 2.75 s; whole ticks cost a little.
		TestTrue(At + FString::Printf(TEXT("in about 2.75 s (%.3f s)"), ArrivedAt), ArrivedAt >= 2.75 - 1e-6 && ArrivedAt <= 2.75 + 8.0 * Dt);
	}
	return true;
}

// Catches: the first tick jumping (W leaving the measured attitude by more
// than the acceleration allows); the special cases of no limit; a Dt of 0
// moving W.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelTiltTargetStarts,
	"Ramms.Level.TiltTarget.StartsAtMeasured",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelTiltTargetStarts::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FVector2D Measured(5.0, -2.0);
	const double	Dt = 1.0 / 60.0;
	{
		FTiltTarget Target;
		Target.Reset(Measured);
		NearBoth(*this, TEXT("W is the measured attitude"), Target.Get(), Measured, 1e-12);
		NearBoth(*this, TEXT("no time: still there"), Target.Tick(FVector2D::ZeroVector, 0.0, 3.0, 4.0), Measured, 1e-12);
		const FVector2D W = Target.Tick(FVector2D::ZeroVector, Dt, 3.0, 4.0);
		Near(*this, TEXT("the first tick moves a Dt^2 of the way: from rest"), (W - Measured).Size(), 4.0 * Dt * Dt, 1e-9);
		TestTrue(TEXT("towards level"), W.X < Measured.X && W.Y > Measured.Y);
		TestFalse(TEXT("not arrived"), Target.HasArrived(FVector2D::ZeroVector));
		TestTrue(TEXT("on its goal and still: arrived"), FTiltTarget().HasArrived(FVector2D(0.01, 0.0)));
	}
	{
		// No acceleration limit: a pure rate limit.
		FTiltTarget Target;
		Target.Reset(FVector2D(3.0, 4.0));
		NearBoth(*this, TEXT("rate only: 3 deg/s along the way from the first tick"), Target.Tick(FVector2D::ZeroVector, 0.1, 3.0, 0.0), FVector2D(3.0, 4.0) * (4.7 / 5.0), 1e-9);
		for (int32 i = 0; i < 30; ++i)
		{
			Target.Tick(FVector2D::ZeroVector, 0.1, 3.0, 0.0);
		}
		NearBoth(*this, TEXT("rate only: lands on the goal"), Target.Get(), FVector2D::ZeroVector, 1e-12);
		TestTrue(TEXT("rate only: arrived"), Target.HasArrived(FVector2D::ZeroVector));
	}
	{
		// No limit at all: there at once.
		FTiltTarget Target;
		Target.Reset(Measured);
		NearBoth(*this, TEXT("no limits: the goal"), Target.Tick(FVector2D(1.0, 1.0), Dt, 0.0, 0.0), FVector2D(1.0, 1.0), 1e-12);
		NearBoth(*this, TEXT("no limits: at rest"), Target.GetRate(), FVector2D::ZeroVector, 1e-12);
	}
	return true;
}

// Catches: the goal not level when the legs can take the whole slope out (a
// share dropped, or the reference added twice).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelWantedTiltZeroInside,
	"Ramms.Level.WantedTilt.ZeroInsideReach",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelWantedTiltZeroInside::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	FTuning Tuning;
	NearBoth(*this, TEXT("the whole share: level"), WantedTiltGoalForShare(FVector2D(6.0, -4.0), 1.0, Tuning), FVector2D::ZeroVector, 1e-12);
	Tuning.LevelReferenceDeg = FVector2D(1.0, -0.5);
	NearBoth(*this, TEXT("the whole share: the reference, where that is not level"), WantedTiltGoalForShare(FVector2D(6.0, -4.0), 1.0, Tuning), FVector2D(1.0, -0.5), 1e-12);
	NearBoth(*this, TEXT("ground at the reference: the reference at any share"), WantedTiltGoalForShare(FVector2D(1.0, -0.5), 0.0, Tuning), FVector2D(1.0, -0.5), 1e-12);

	// Through the legs: travel to spare, so the whole slope is in reach.
	const TArray<FLegReach> Legs = SixLegs(30.0, 30.0, 0.0);
	NearBoth(*this, TEXT("legs with travel to spare: the reference"), WantedTiltGoal(FVector2D(6.0, -4.0), Legs, TConstArrayView<FClearancePoint>(), Tuning), FVector2D(1.0, -0.5), 1e-9);
	return true;
}

// Catches: the excess not (1 - s0) of the ground's tilt, or not along it
// (say, the share applied to pitch only).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelWantedTiltExcess,
	"Ramms.Level.WantedTilt.ExcessAlongGround",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelWantedTiltExcess::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	FTuning Tuning;
	NearBoth(*this, TEXT("three quarters in reach: a quarter kept, along the ground"), WantedTiltGoalForShare(FVector2D(8.0, -6.0), 0.75, Tuning), FVector2D(2.0, -1.5), 1e-12);
	Tuning.LevelReferenceDeg = FVector2D(1.0, 0.0);
	NearBoth(*this, TEXT("measured off the reference"), WantedTiltGoalForShare(FVector2D(5.0, 2.0), 0.5, Tuning), FVector2D(3.0, 1.0), 1e-12);

	// A robot whose roll correction is capped keeps the roll it cannot take
	// out, even with all its travel: it does not want a level it cannot reach.
	FTuning Capped;
	Capped.MaxCorrectionDeg = FVector2D(15.0, 2.0);
	NearBoth(*this, TEXT("roll capped at 2: 2 of 4 deg kept"), WantedTiltGoalForShare(FVector2D(3.0, 4.0), 1.0, Capped), FVector2D(0.0, 2.0), 1e-12);
	return true;
}

// Catches: the cap left off, applied per axis (5 deg each way is 7 in all), or
// measured from level in place of the reference.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelWantedTiltCapped,
	"Ramms.Level.WantedTilt.CappedAtMaxTilt",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelWantedTiltCapped::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning Tuning; // 5 deg
	NearBoth(*this, TEXT("10 deg out of reach: 5 kept, along the ground"), WantedTiltGoalForShare(FVector2D(8.0, 6.0), 0.0, Tuning), FVector2D(4.0, 3.0), 1e-12);
	NearBoth(*this, TEXT("4 deg out of reach: all 4 kept"), WantedTiltGoalForShare(FVector2D(0.0, -4.0), 0.0, Tuning), FVector2D(0.0, -4.0), 1e-12);

	// Through the legs: no travel at all, so nothing is in reach.
	TArray<FLegReach> Legs = SixLegs(0.0, 0.0, 0.0);
	NearBoth(*this, TEXT("legs with no travel: the ground's tilt, capped"), WantedTiltGoal(FVector2D(8.0, 6.0), Legs, TConstArrayView<FClearancePoint>(), Tuning), FVector2D(4.0, 3.0), 1e-9);

	FTuning Off;
	Off.LevelReferenceDeg = FVector2D(2.0, 0.0);
	NearBoth(*this, TEXT("the cap is off the reference"), WantedTiltGoalForShare(FVector2D(12.0, 0.0), 0.0, Off), FVector2D(7.0, 0.0), 1e-12);
	return true;
}

// Catches: the goal made from anything the loop moves (W, C or A kept in a
// static or fed back), which is what would let the two chatter.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelWantedTiltNoW,
	"Ramms.Level.WantedTilt.DoesNotDependOnW",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelWantedTiltNoW::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	// The same ground and the same legs, asked again and again while the
	// wanted tilt chases the answer and the loop runs: the answer never moves.
	const FTuning			Tuning;
	const FVector2D			Ground(9.0, -3.0);
	const TArray<FLegReach> Legs = SixLegs(0.0, 0.0, 0.0);
	const FVector2D			First = WantedTiltGoal(Ground, Legs, TConstArrayView<FClearancePoint>(), Tuning);
	FTiltTarget				Target;
	Target.Reset(Ground);
	FLevelLoop Loop;
	double	   Moved = 0.0;
	for (int32 i = 0; i < 240; ++i)
	{
		const FVector2D Goal = WantedTiltGoal(Ground, Legs, TConstArrayView<FClearancePoint>(), Tuning);
		Moved = FMath::Max(Moved, (Goal - First).Size());
		const FVector2D W = Target.Tick(Goal, 1.0 / 60.0, Tuning.TiltRateDegPerSec, Tuning.TiltAccelDegPerSec2);
		Loop.Tick(Ground, W, Ground, 1.0 / 60.0, true, false, Tuning);
	}
	TestTrue(TEXT("the goal stands still while W and the loop move"), Moved == 0.0);
	TestTrue(TEXT("and W comes to rest on it"), Target.HasArrived(First));
	return true;
}

// ---- the seat trim ----

// Catches: the cover's sign (the seat pitching with the base), the deadband
// left out or as wide as it once was, the share left out of the trim or added
// with the wrong sign, and a cover that runs when told not to.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelSeatTrimCancels,
	"Ramms.Level.SeatTrim.CancelsBasePitch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelSeatTrimCancels::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning Tuning;
	const double  Dt = 1.0 / 60.0;
	const double  NoLimit = 1.0e6;
	const double  Errors[] = { 3.0, -3.0 };
	for (const double Error : Errors)
	{
		// The base 3 deg off where it is wanted, nothing held in the seat.
		FSeatTrim Trim;
		for (int32 i = 0; i < 60; ++i)
		{
			Trim.Tick(0.0, Error, true, -NoLimit, NoLimit, Dt, Tuning);
		}
		// The rider is left with the deadband, no more.
		Near(*this, FString::Printf(TEXT("base %.0f deg off: the seat the other way, less the deadband"), Error), Trim.Get(), -(Error - FMath::Sign(Error) * SeatTrimDeadbandDeg), 1e-9);
		Near(*this, TEXT("base and trim together: the deadband"), FMath::Abs(Error + Trim.Get()), SeatTrimDeadbandDeg, 1e-9);
		TestTrue(TEXT("the deadband is a small one"), SeatTrimDeadbandDeg <= 0.1);

		// The same with 2 deg of the rider's wanted pitch held in the seat:
		// the share is added, whole and as it is.
		FSeatTrim Held;
		for (int32 i = 0; i < 60; ++i)
		{
			Held.Tick(2.0, Error, true, -NoLimit, NoLimit, Dt, Tuning);
		}
		Near(*this, TEXT("with a share: the share and the cover together"), Held.Get(), 2.0 + Trim.Get(), 1e-9);

		// Told not to cover: the share alone, whatever the base does.
		FSeatTrim Bare;
		for (int32 i = 0; i < 60; ++i)
		{
			Bare.Tick(2.0, Error, false, -NoLimit, NoLimit, Dt, Tuning);
		}
		Near(*this, TEXT("not covering: the share alone"), Bare.Get(), 2.0, 1e-12);
	}
	FSeatTrim Small;
	for (int32 i = 0; i < 60; ++i)
	{
		Small.Tick(0.0, 0.5 * SeatTrimDeadbandDeg, true, -NoLimit, NoLimit, Dt, Tuning);
	}
	Near(*this, TEXT("inside the deadband: no trim"), Small.Get(), 0.0, 1e-12);
	return true;
}

// Catches: the rate limit left off (the trim jumping to its target), the
// clamp at SeatTrimMaxDeg left off, the seat's own room not kept (either
// side), and "behind" not said while the rate holds the trim back.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelSeatTrimRateClamp,
	"Ramms.Level.SeatTrim.RateAndClampHold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelSeatTrimRateClamp::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning Tuning; // 8 deg, 10 deg/s
	const double  Dt = 1.0 / 60.0;
	const double  NoLimit = 1.0e6;
	FSeatTrim	  Trim;
	double		  Fastest = 0.0;
	double		  Was = 0.0;
	bool		  bBehindOnTheWay = true;
	for (int32 i = 0; i < 120; ++i)
	{
		const double Now = Trim.Tick(0.0, 20.0, true, -NoLimit, NoLimit, Dt, Tuning);
		Fastest = FMath::Max(Fastest, FMath::Abs(Now - Was) / Dt);
		bBehindOnTheWay = bBehindOnTheWay && (i >= 40 || Trim.IsBehind());
		Was = Now;
	}
	Near(*this, TEXT("a base 20 deg off: the trim moves at its rate, no faster"), Fastest, Tuning.SeatTrimRateDegPerSec, 1e-9);
	Near(*this, TEXT("and stops at its limit"), Trim.Get(), -Tuning.SeatTrimMaxDeg, 1e-12);
	TestTrue(TEXT("behind its target while the rate held it"), bBehindOnTheWay);
	TestFalse(TEXT("... and not once it is there"), Trim.IsBehind());
	Near(*this, TEXT("no time: the trim it had"), Trim.Tick(0.0, 0.0, false, -NoLimit, NoLimit, 0.0, Tuning), -Tuning.SeatTrimMaxDeg, 1e-12);

	// A share past the limit is cut to it, the other way too.
	FSeatTrim Share;
	for (int32 i = 0; i < 120; ++i)
	{
		Share.Tick(20.0, 0.0, false, -NoLimit, NoLimit, Dt, Tuning);
	}
	Near(*this, TEXT("a share of 20 deg: held at the limit"), Share.Get(), Tuning.SeatTrimMaxDeg, 1e-12);

	// The seat's own room, where that is the nearer: 1.5 deg down, 0.5 up.
	FSeatTrim Down;
	FSeatTrim Up;
	for (int32 i = 0; i < 120; ++i)
	{
		Down.Tick(0.0, 20.0, true, -1.5, 0.5, Dt, Tuning);
		Up.Tick(0.0, -20.0, true, -1.5, 0.5, Dt, Tuning);
	}
	Near(*this, TEXT("the seat's own room, nose down"), Down.Get(), -1.5, 1e-12);
	Near(*this, TEXT("the seat's own room, nose up"), Up.Get(), 0.5, 1e-12);
	return true;
}

// Catches: the cover holding when the base is back where it is wanted or when
// it is told not to run, and a trim that does not start from nothing after
// Zero.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelSeatTrimDecays,
	"Ramms.Level.SeatTrim.DecaysAsBaseLevels",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelSeatTrimDecays::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning Tuning;
	const double  Dt = 1.0 / 60.0;
	const double  NoLimit = 1.0e6;
	const double  Covered = 3.0 - SeatTrimDeadbandDeg;
	{
		// The legs bring the base from 3 deg off to where it is wanted over a second.
		FSeatTrim Trim;
		for (int32 i = 0; i < 60; ++i)
		{
			Trim.Tick(0.0, 3.0, true, -NoLimit, NoLimit, Dt, Tuning);
		}
		double Largest = 0.0;
		for (int32 i = 0; i < 120; ++i)
		{
			const double Error = FMath::Max(0.0, 3.0 - 3.0 * (i + 1) / 60.0);
			const double Now = Trim.Tick(0.0, Error, true, -NoLimit, NoLimit, Dt, Tuning);
			// The cover goes as the error does: the two together are never
			// more than the deadband.
			Largest = FMath::Max(Largest, FMath::Abs(Error + Now));
		}
		Near(*this, TEXT("base where it is wanted again: the cover is gone, exactly"), Trim.Get(), 0.0, 1e-12);
		TestTrue(FString::Printf(TEXT("the rider within the deadband on the way (%.3f deg)"), Largest), Largest <= SeatTrimDeadbandDeg + 1e-9);
	}
	{
		// Told not to cover with the base still off: it runs out at its rate.
		FSeatTrim Trim;
		for (int32 i = 0; i < 60; ++i)
		{
			Trim.Tick(0.0, 3.0, true, -NoLimit, NoLimit, Dt, Tuning);
		}
		Near(*this, TEXT("one tick of running out"), Trim.Tick(0.0, 3.0, false, -NoLimit, NoLimit, Dt, Tuning), -Covered + Tuning.SeatTrimRateDegPerSec * Dt, 1e-9);
		for (int32 i = 0; i < 30; ++i)
		{
			Trim.Tick(0.0, 3.0, false, -NoLimit, NoLimit, Dt, Tuning);
		}
		Near(*this, TEXT("not covering: zero"), Trim.Get(), 0.0, 1e-12);
		// The base has meanwhile gone to 3 deg off the other way: the cover
		// sets off against that at once.
		TestTrue(TEXT("covering again: against the error as it now is"), Trim.Tick(0.0, -3.0, true, -NoLimit, NoLimit, Dt, Tuning) > 0.0);
		for (int32 i = 0; i < 60; ++i)
		{
			Trim.Tick(0.0, -3.0, true, -NoLimit, NoLimit, Dt, Tuning);
		}
		// Folded into the seat's own command: zero with no motion, and from
		// there it is rate-limited as ever.
		Trim.Zero();
		Near(*this, TEXT("Zero: zero"), Trim.Get(), 0.0, 1e-12);
		Near(*this, TEXT("after Zero: back out at its rate"), Trim.Tick(0.0, -3.0, true, -NoLimit, NoLimit, Dt, Tuning), Tuning.SeatTrimRateDegPerSec * Dt, 1e-9);
	}
	return true;
}

// ---- the speed cap ----

// Catches: a term multiplying the floor down (two floors make 0.06), or the
// floor taken as 0 when the tuning says 0: the rider must always be able to
// drive.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelSpeedScaleFloor,
	"Ramms.Level.SpeedScale.FloorNeverZero",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelSpeedScaleFloor::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	// Everything at its worst at once: far off, past reach, on a steep slope
	// over a short span with legs that hardly move, the ground spinning under
	// a robot at a run.
	FSpeedCapInputs Worst = CapSlope(30.0, 10.0, 0.1);
	Worst.ErrorDeg = 30.0;
	Worst.bPastReach = true;
	Worst.GroundRateDegPerSec = FVector2D(600.0, 800.0);
	Worst.bGroundRateKnown = true;
	Worst.SpeedCmS = 500.0;
	{
		const FTuning Tuning;
		FSpeedScale	  Scale;
		double		  Least = 1.0;
		for (int32 i = 0; i < 300; ++i)
		{
			Least = FMath::Min(Least, Scale.Tick(Worst, 0.1, Tuning));
		}
		Near(*this, TEXT("every term at its worst: the floor, not under it"), Least, Tuning.MinSpeedScale, 1e-12);
	}
	{
		FTuning Tuning;
		Tuning.MinSpeedScale = 0.0;
		FSpeedScale Scale;
		for (int32 i = 0; i < 300; ++i)
		{
			Scale.Tick(Worst, 0.1, Tuning);
		}
		TestTrue(TEXT("a floor of 0 in the tuning still leaves a crawl"), Scale.Get() > 0.0);
	}
	return true;
}

// Catches: the error term's ends or slope (1 at 1 deg, the floor at 3), or
// bPastReach not going to the floor outright.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelSpeedScaleErrorRamp,
	"Ramms.Level.SpeedScale.ErrorRamp",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelSpeedScaleErrorRamp::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning Tuning;
	const double  Errors[] = { 0.5, 1.0, 2.0, 2.5, 3.0, 9.0 };
	const double  Scales[] = { 1.0, 1.0, 0.625, 0.4375, 0.25, 0.25 };
	const int32	  Count = UE_ARRAY_COUNT(Errors);
	for (int32 i = 0; i < Count; ++i)
	{
		FSpeedScale Scale;
		Near(*this, FString::Printf(TEXT("%.1f deg of error"), Errors[i]), SettledScale(Scale, CapError(Errors[i]), Tuning), Scales[i], 1e-9);
	}
	FSpeedScale Past;
	Near(*this, TEXT("past reach: the floor, whatever the error"), SettledScale(Past, CapError(0.0, true), Tuning), Tuning.MinSpeedScale, 1e-9);
	return true;
}

// Catches: a steady slope costing the rider speed with the term off (as the
// cap comes); and, switched on, the slope's-end term's form (the speed at
// which the ground would turn at what the legs can follow when the slope
// ends: Follow * span / slope), a robot slowed for standing on a slope, or
// for a speed its stick asks and it does not have; and anything that slows
// for how little reach is left (the cap is not told).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelSpeedScaleSlopesEnd,
	"Ramms.Level.SpeedScale.SlowsForTheSlopesEnd",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelSpeedScaleSlopesEnd::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning	Tuning;
	const FVector2D Still = FVector2D::ZeroVector;
	// Legs that turn the correction at 10 deg/s could be asked 8; the ground
	// may turn at 6 (the cap's own most). 6 deg over legs 45 cm apart comes
	// back within 45 cm: 45 cm/s.
	const FSpeedCapInputs Six = CapSlope(6.0, 45.0, 10.0);
	{
		// The term is off as the cap comes: a steady slope the legs hold level
		// costs nothing, at any speed.
		FSpeedScale Scale;
		Near(*this, TEXT("as it comes, 6 deg at 90 cm/s: full speed"), DrivenScale(Scale, Six, 90.0, Still, Tuning), 1.0, 1e-12);
		Near(*this, TEXT("as it comes, 12 deg at 600 cm/s: full speed"), DrivenScale(Scale, CapSlope(12.0, 45.0, 10.0), 600.0, Still, Tuning), 1.0, 1e-12);
		Near(*this, TEXT("... and no speed kept for a slope's end"), Scale.GetExitSpeedCmS(), 0.0, 1e-12);
	}
	{
		FSpeedScale Scale = PlainCap(true);
		Near(*this, TEXT("6 deg, the stick asking 45 cm/s: full speed"), DrivenScale(Scale, Six, 45.0, Still, Tuning), 1.0, 1e-9);
		Near(*this, TEXT("... what the legs can follow"), Scale.GetFollowDegPerSec(), 6.0, 1e-9);
		Near(*this, TEXT("... the speed the slope's end can be taken at"), Scale.GetExitSpeedCmS(), 45.0, 1e-9);
	}
	{
		FSpeedScale Scale = PlainCap(true);
		Near(*this, TEXT("6 deg, the stick asking 90: half"), DrivenScale(Scale, Six, 90.0, Still, Tuning), 0.5, 1e-6);
		Near(*this, TEXT("... the speed the stick is taken to ask"), Scale.GetFreeSpeedCmS(), 90.0, 1e-6);
		Near(*this, TEXT("the stick let go to 45: full speed again"), DrivenScale(Scale, Six, 45.0, Still, Tuning), 1.0, 1e-6);
	}
	{
		FSpeedScale Scale = PlainCap(true);
		Near(*this, TEXT("6 deg, the stick asking 600: the floor"), DrivenScale(Scale, Six, 600.0, Still, Tuning), Tuning.MinSpeedScale, 1e-9);
	}
	{
		// Twice the slope is half the speed; twice the span twice it.
		FSpeedScale Steep = PlainCap(true), Long = PlainCap(true);
		Near(*this, TEXT("12 deg at 45: half"), DrivenScale(Steep, CapSlope(12.0, 45.0, 10.0), 45.0, Still, Tuning), 0.5, 1e-6);
		Near(*this, TEXT("12 deg over legs 90 cm apart, at 45: full speed"), DrivenScale(Long, CapSlope(12.0, 90.0, 10.0), 45.0, Still, Tuning), 1.0, 1e-9);
	}
	{
		// Slower legs: 0.8 of 5 deg/s is 4, and 30 cm/s.
		FSpeedScale Slow = PlainCap(true), Quick = PlainCap(true), NoRate = PlainCap(true);
		Near(*this, TEXT("legs at 5 deg/s, 6 deg at 60: half"), DrivenScale(Slow, CapSlope(6.0, 45.0, 5.0), 60.0, Still, Tuning), 0.5, 1e-6);
		// Legs however quick: the ground no faster than 6 deg/s all the same.
		Near(*this, TEXT("legs at 100 deg/s, 6 deg at 90: half still"), DrivenScale(Quick, CapSlope(6.0, 45.0, 100.0), 90.0, Still, Tuning), 0.5, 1e-6);
		Near(*this, TEXT("legs of no known rate: the same"), DrivenScale(NoRate, CapSlope(6.0, 45.0, 0.0), 90.0, Still, Tuning), 0.5, 1e-6);
	}
	{
		// The slope begins to end. Half way through the break the slope left
		// is half -- and so is the way left to lose it in: the drive is not
		// let go into it. It keeps to the speed the whole slope's end is
		// taken at until the robot is a span and a half on.
		FSpeedScale Scale = PlainCap(true);
		DrivenScale(Scale, Six, 90.0, Still, Tuning); // at half: 45 cm/s
		FSpeedCapInputs Ending = CapSlope(3.0, 45.0, 10.0);
		Ending.SpeedCmS = 45.0;
		for (int32 i = 0; i < 10; ++i)
		{
			Scale.Tick(Ending, 0.1, Tuning);
		}
		Near(*this, TEXT("a span on, half the slope gone: still the whole slope's speed"), Scale.GetExitSpeedCmS(), 45.0, 1e-9);
		Near(*this, TEXT("... and the drive still held to it"), Scale.Get(), 0.5, 1e-6);
		for (int32 i = 0; i < 6; ++i)
		{
			Scale.Tick(Ending, 0.1, Tuning);
		}
		Near(*this, TEXT("a span and a half on: the slope as it now is"), Scale.GetExitSpeedCmS(), 90.0, 1e-9);
		TestTrue(TEXT("... and the drive let go"), Scale.Get() > 0.5);
		// A slope that steepens is taken at once.
		Scale.Tick(CapSlope(9.0, 45.0, 10.0), 0.1, Tuning);
		Near(*this, TEXT("steeper: at once"), Scale.GetExitSpeedCmS(), 30.0, 1e-9);
	}
	{
		// Standing on the slope, and on level ground at any speed: nothing.
		FSpeedScale Standing = PlainCap(true), Level = PlainCap(true), NoSpan = PlainCap(true);
		Near(*this, TEXT("standing on 12 deg: full speed"), DrivenScale(Standing, CapSlope(12.0, 45.0, 10.0), 0.0, Still, Tuning), 1.0, 1e-12);
		Near(*this, TEXT("level ground at 600 cm/s: full speed"), DrivenScale(Level, CapSlope(0.0, 45.0, 10.0), 600.0, Still, Tuning), 1.0, 1e-12);
		Near(*this, TEXT("no span known: full speed"), DrivenScale(NoSpan, CapSlope(12.0, 0.0, 10.0), 600.0, Still, Tuning), 1.0, 1e-12);
	}
	{
		// A robot that is not getting the speed its stick asks (a drive at
		// its limit on the ramp) crawls at 20 cm/s whatever the scale: it is
		// not slowed for a speed it does not have.
		FSpeedScale		Scale = PlainCap(true);
		FSpeedCapInputs In = CapSlope(12.0, 45.0, 10.0);
		In.SpeedCmS = 20.0;
		Near(*this, TEXT("12 deg at a crawl of 20 cm/s: full speed"), SettledScale(Scale, In, Tuning), 1.0, 1e-12);
	}
	return true;
}

// Catches: the turn term's form (the scale at which the ground would turn at
// what the legs can follow), it biting at or under that rate, a turn that is
// not the ground's counted, or one tick's reading taken whole (a noisy
// estimate would slow the drive).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelSpeedScaleTurn,
	"Ramms.Level.SpeedScale.SlowsForAGroundThatTurns",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelSpeedScaleTurn::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning Tuning;
	// Level ground that curves: no slope to end, legs at 10 deg/s (the ground
	// may turn at 6). The ground turns in proportion to the speed.
	const FSpeedCapInputs Curve = CapSlope(0.0, 45.0, 10.0);
	const double		  AtStick[] = { 3.0, 6.0, 12.0, 24.0, 240.0 };
	const double		  Scales[] = { 1.0, 1.0, 0.5, 0.25, 0.25 };
	const int32			  Count = UE_ARRAY_COUNT(AtStick);
	for (int32 i = 0; i < Count; ++i)
	{
		FSpeedScale Scale;
		Near(*this, FString::Printf(TEXT("the ground turning %.0f deg/s at the stick's speed"), AtStick[i]), DrivenScale(Scale, Curve, 60.0, FVector2D(0.6, 0.8) * AtStick[i], Tuning), Scales[i], 1e-6);
	}
	{
		// A turn that is not the ground's (the estimate reseeded, a stance
		// being changed): not counted, and what was counted dies away.
		FSpeedScale		Scale;
		FSpeedCapInputs In = Curve;
		In.GroundRateDegPerSec = FVector2D(100.0, 0.0);
		In.bGroundRateKnown = false;
		Near(*this, TEXT("a turn that is not the ground's: full speed"), SettledScale(Scale, In, Tuning), 1.0, 1e-12);
		Near(*this, TEXT("... nothing kept of it"), Scale.GetTurnDegPerSec().Size(), 0.0, 1e-12);
	}
	{
		// One tick of 20 deg/s in a steady ground (a sensor's noise): it is
		// low-passed, and the drive is not slowed for it.
		FSpeedScale		Scale;
		FSpeedCapInputs In = Curve;
		In.bGroundRateKnown = true;
		In.GroundRateDegPerSec = FVector2D(20.0, 0.0);
		const double Once = Scale.Tick(In, 1.0 / 60.0, Tuning);
		Near(*this, TEXT("one tick of 20 deg/s: not slowed"), Once, 1.0, 1e-12);
		TestTrue(TEXT("... a twelfth of it kept"), Scale.GetTurnDegPerSec().X > 1.0 && Scale.GetTurnDegPerSec().X < 2.0);
	}
	return true;
}

// Catches: the scale jumping to its target, the two rates swapped (slow to
// slow down, quick to speed up), or a cap that brakes a fast robot harder
// than the ground's estimate can see through.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelSpeedScaleSlew,
	"Ramms.Level.SpeedScale.FallsFastRisesSlow",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelSpeedScaleSlew::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning Tuning; // falls at 2 per s, rises at 0.5
	{
		FSpeedScale Scale = PlainCap(false);
		Near(*this, TEXT("starts at full speed"), Scale.Get(), 1.0, 1e-12);
		Near(*this, TEXT("a large error: down 0.2 in 0.1 s"), Scale.Tick(CapError(9.0), 0.1, Tuning), 0.8, 1e-9);
		Near(*this, TEXT("and again"), Scale.Tick(CapError(9.0), 0.1, Tuning), 0.6, 1e-9);
		Near(*this, TEXT("no time: where it was"), Scale.Tick(CapError(0.0), 0.0, Tuning), 0.6, 1e-12);
		Near(*this, TEXT("the error gone: up 0.05 in 0.1 s"), Scale.Tick(CapError(0.0), 0.1, Tuning), 0.65, 1e-9);
		Near(*this, TEXT("Relax rises the same"), Scale.Relax(0.1, Tuning), 0.7, 1e-9);
		Near(*this, TEXT("Relax with no time: where it was"), Scale.Relax(0.0, Tuning), 0.7, 1e-12);
		for (int32 i = 0; i < 100; ++i)
		{
			Scale.Relax(0.1, Tuning);
		}
		Near(*this, TEXT("Relax ends at 1, not over"), Scale.Get(), 1.0, 1e-12);
	}
	{
		// At 300 cm/s a fall of 2 per s would brake at 600 cm/s^2: held to
		// 150, half a scale a second.
		FSpeedScale		Scale = PlainCap(false);
		FSpeedCapInputs In = CapError(9.0);
		In.SpeedCmS = 300.0;
		Near(*this, TEXT("a large error at 300 cm/s: down 0.05 in 0.1 s"), Scale.Tick(In, 0.1, Tuning), 0.95, 1e-9);
		Scale.Cap.BrakeCmS2 = 0.0;
		Near(*this, TEXT("with no such limit: 0.2"), Scale.Tick(In, 0.1, Tuning), 0.75, 1e-9);
	}
	return true;
}

// Catches: the legs' plane rate taken from a leg that does not carry the
// robot, without the fraction of a leg's rate the slew keeps to, along the
// wrong axis, or over the plane's own rate.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopPlaneRate,
	"Ramms.Level.Loop.PlaneRateOfTheStandingLegs",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopPlaneRate::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	FTuning Tuning;
	Tuning.MaxCorrectionRateDegPerSec = 0.0; // the legs alone
	// All six standing at 4 cm/s: in pitch the corners, 45 cm from the middle,
	// are the busiest; in roll the centre legs, 27 cm out.
	TArray<FLegReach> Legs = SixLegs(20.0, 20.0, 4.0);
	const double	  PerDegPitch = 45.0 * FMath::Tan(FMath::DegreesToRadians(1.0));
	const double	  PerDegRoll = 27.0 * FMath::Tan(FMath::DegreesToRadians(1.0));
	Near(*this, TEXT("pitch: 0.8 of the corners' rate over their arm"), PlaneRateDegPerSec(Legs, FVector2D(3.0, 0.0), Tuning), 0.8 * 4.0 / PerDegPitch, 1e-6);
	Near(*this, TEXT("nose down the same"), PlaneRateDegPerSec(Legs, FVector2D(-0.1, 0.0), Tuning), 0.8 * 4.0 / PerDegPitch, 1e-6);
	Near(*this, TEXT("roll: over the centre legs' arm"), PlaneRateDegPerSec(Legs, FVector2D(0.0, 2.0), Tuning), 0.8 * 4.0 / PerDegRoll, 1e-6);
	Near(*this, TEXT("no direction: nothing to say"), PlaneRateDegPerSec(Legs, FVector2D::ZeroVector, Tuning), 0.0, 1e-12);

	// The front pair lifted: the four that stand have their middle at -22.5,
	// and are 22.5 cm from it. The lifted pair, 67.5 cm ahead and slow, does
	// not count.
	for (int32 i = 0; i < 2; ++i)
	{
		Legs[i].bSupport = false;
		Legs[i].MaxRateCmS = 0.5;
	}
	const double PerDegOnFour = 22.5 * FMath::Tan(FMath::DegreesToRadians(1.0));
	Near(*this, TEXT("on four: over the standing legs' arm, the lifted pair left out"), PlaneRateDegPerSec(Legs, FVector2D(1.0, 0.0), Tuning), 0.8 * 4.0 / PerDegOnFour, 1e-6);

	Tuning.MaxCorrectionRateDegPerSec = 5.0;
	Near(*this, TEXT("no faster than the plane's own rate"), PlaneRateDegPerSec(Legs, FVector2D(1.0, 0.0), Tuning), 5.0, 1e-12);
	Tuning.MaxCorrectionRateDegPerSec = 20.0;
	Near(*this, TEXT("... which does not raise the legs'"), PlaneRateDegPerSec(Legs, FVector2D(1.0, 0.0), Tuning), 0.8 * 4.0 / PerDegOnFour, 1e-6);
	Tuning.LegRateFraction = 0.0;
	Near(*this, TEXT("with no leg-rate cut: the plane's rate alone"), PlaneRateDegPerSec(Legs, FVector2D(1.0, 0.0), Tuning), 20.0, 1e-12);
	Tuning.MaxCorrectionRateDegPerSec = 0.0;
	Near(*this, TEXT("with neither: nothing limits it"), PlaneRateDegPerSec(Legs, FVector2D(1.0, 0.0), Tuning), 0.0, 1e-12);

	// Nobody stands: every leg counts.
	Tuning = FTuning();
	Tuning.MaxCorrectionRateDegPerSec = 0.0;
	TArray<FLegReach> Lifted = SixLegs(20.0, 20.0, 4.0);
	for (FLegReach& Leg : Lifted)
	{
		Leg.bSupport = false;
	}
	Near(*this, TEXT("no leg stands: all of them count"), PlaneRateDegPerSec(Lifted, FVector2D(1.0, 0.0), Tuning), 0.8 * 4.0 / PerDegPitch, 1e-6);
	return true;
}

// Catches: a leg carried off the ground holding the whole plane back while it
// is let down, or while it rests on its stop and is not moving at all (far
// ahead of the legs that stand, it is the busiest leg by far, and the
// correction then unwinds at a fraction of what the standing legs can do);
// and the other way, one that must come UP to keep its clearance not counted.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopLiftedLegRate,
	"Ramms.Level.Loop.LiftedLegHoldsThePlaneBackOnlyComingUp",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopLiftedLegRate::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	FTuning Tuning;
	Tuning.SoftZoneCm = 0.0; // the rates alone
	const double Dt = 0.1;
	double		 Use = -1.0;
	bool		 bLimited = false;

	// On centre and rear legs at 60 cm/s; the front pair lifted, 67.5 cm
	// ahead of their middle, at 5 cm/s, with travel both ways.
	TArray<FLegReach> Legs = SixLegs(20.0, 20.0, 60.0);
	for (int32 i = 0; i < 2; ++i)
	{
		Legs[i].bSupport = false;
		Legs[i].MaxRateCmS = 5.0;
	}
	const double StandingUse = 22.5 * FMath::Tan(FMath::DegreesToRadians(1.0)) / (Dt * 60.0);
	const double LiftedUse = 67.5 * FMath::Tan(FMath::DegreesToRadians(1.0)) / (Dt * 5.0);
	{
		// A degree nose down (the front legs let down): the plane's whole
		// step, at the standing legs' use.
		const FVector2D C = SlewCorrection(Legs, FVector2D::ZeroVector, FVector2D(-1.0, 0.0), Dt, 10.0, Tuning, Use, bLimited);
		NearBoth(*this, TEXT("let down: the whole step"), C, FVector2D(-1.0, 0.0), 1e-12);
		TestFalse(TEXT("let down: not limited"), bLimited);
		Near(*this, TEXT("let down: the standing legs' use alone"), Use, StandingUse, 1e-6);
	}
	{
		// A degree nose up (the front legs brought up, to keep clear): they
		// count, and hold the plane to 0.8 of their rate.
		const FVector2D C = SlewCorrection(Legs, FVector2D::ZeroVector, FVector2D(1.0, 0.0), Dt, 10.0, Tuning, Use, bLimited);
		Near(*this, TEXT("brought up: the lifted legs' use, over 1"), Use, LiftedUse, 1e-6);
		TestTrue(TEXT("brought up: limited"), bLimited);
		Near(*this, TEXT("brought up: the plane held to 0.8 of their rate"), 67.5 * FMath::Tan(FMath::DegreesToRadians(C.X)), 0.8 * 5.0 * Dt, 1e-4);
	}
	{
		// On its retract stop: asked up, it cannot move, and holds nothing.
		TArray<FLegReach> OnStop = Legs;
		for (int32 i = 0; i < 2; ++i)
		{
			OnStop[i].MaxZ = OnStop[i].BaseZ;
		}
		const FVector2D C = SlewCorrection(OnStop, FVector2D::ZeroVector, FVector2D(1.0, 0.0), Dt, 10.0, Tuning, Use, bLimited);
		NearBoth(*this, TEXT("on its stop: the whole step"), C, FVector2D(1.0, 0.0), 1e-12);
		TestFalse(TEXT("on its stop: not limited"), bLimited);
		Near(*this, TEXT("on its stop: the standing legs' use alone"), Use, StandingUse, 1e-6);

		// A little short of its stop: only the bit it can still make counts.
		for (int32 i = 0; i < 2; ++i)
		{
			OnStop[i].MaxZ = OnStop[i].BaseZ + 0.2;
		}
		SlewCorrection(OnStop, FVector2D::ZeroVector, FVector2D(1.0, 0.0), Dt, 10.0, Tuning, Use, bLimited);
		Near(*this, TEXT("0.2 cm short of its stop: that much of its move counts"), Use, 0.2 / (Dt * 5.0), 1e-6);
	}
	{
		// A leg the robot stands on counts both ways, as ever.
		TArray<FLegReach> Standing = Legs;
		for (int32 i = 0; i < 2; ++i)
		{
			Standing[i].bSupport = true;
		}
		SlewCorrection(Standing, FVector2D::ZeroVector, FVector2D(-1.0, 0.0), Dt, 10.0, Tuning, Use, bLimited);
		TestTrue(TEXT("a standing leg let down: limited"), bLimited);
		Near(*this, TEXT("... by its own use"), Use, 45.0 * FMath::Tan(FMath::DegreesToRadians(1.0)) / (Dt * 5.0), 1e-6);
	}
	return true;
}

// ---- at rest ----

// Catches: quiet reached early or with the stick held; no wake at the wake
// error, on the stick, on motion or when the leave falls; the band between
// the two errors ending a quiet (or counting towards one).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelQuietStopsAndWakes,
	"Ramms.Level.Quiet.StopsWritingAndWakes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelQuietStopsAndWakes::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning Tuning; // settled under 0.3 deg for 1 s; awake at 0.5
	FQuiet		  Quiet;
	TestFalse(TEXT("0.9 s at rest: not yet"), RestFor(Quiet, 9, 0.1, Tuning));
	for (int32 i = 0; i < 5; ++i)
	{
		Quiet.Tick(0.0, 0.1, false, true, 0.0, Tuning);
	}
	TestFalse(TEXT("ticks with no time do not count"), Quiet.IsQuiet());
	TestTrue(TEXT("1 s at rest: quiet"), RestFor(Quiet, 1, 0.1, Tuning));
	TestTrue(TEXT("0.4 deg: between the two, a quiet is kept"), RestFor(Quiet, 5, 0.4, Tuning));
	TestFalse(TEXT("0.5 deg: awake"), RestFor(Quiet, 1, 0.5, Tuning));

	// A count under way is broken by the band between.
	RestFor(Quiet, 5, 0.1, Tuning);
	RestFor(Quiet, 1, 0.4, Tuning);
	TestFalse(TEXT("0.5 s, a tick at 0.4 deg, 0.9 s: the count began again"), RestFor(Quiet, 9, 0.1, Tuning));
	TestTrue(TEXT("and runs out a tick later"), RestFor(Quiet, 1, 0.1, Tuning));

	TestFalse(TEXT("the stick moves: awake"), Quiet.Tick(0.0, 0.1, true, true, 0.1, Tuning));
	TestFalse(TEXT("with the stick held it never settles"), [&]() {
		bool bOut = false;
		for (int32 i = 0; i < 30; ++i)
		{
			bOut = Quiet.Tick(0.0, 0.1, true, true, 0.1, Tuning);
		}
		return bOut;
	}());
	TestTrue(TEXT("settles again"), RestFor(Quiet, 10, 0.1, Tuning));
	TestFalse(TEXT("the chassis moves at 2 cm/s: awake"), Quiet.Tick(-QuietSpeedCmS, 0.1, false, true, 0.1, Tuning));
	TestTrue(TEXT("settles again, creeping under 2 cm/s"), [&]() {
		bool bOut = false;
		for (int32 i = 0; i < 10; ++i)
		{
			bOut = Quiet.Tick(1.5, 0.1, false, true, 0.1, Tuning);
		}
		return bOut;
	}());
	TestFalse(TEXT("the wanted tilt sets off (no leave to rest): awake"), Quiet.Tick(0.0, 0.1, false, false, 0.1, Tuning));
	TestTrue(TEXT("settles again"), RestFor(Quiet, 10, 0.1, Tuning));
	Quiet.Reset();
	TestFalse(TEXT("Reset: awake"), Quiet.IsQuiet());
	return true;
}

// Catches: a reading followed at any rate while the robot stands (a knock
// taken for ground), the travel's allowance left out (a grade break at speed
// left behind), a bound on each axis by itself where it is on the two
// together, a reading inside the allowance not followed exactly, and a gate
// that cannot be switched off.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopGroundGateTravel,
	"Ramms.Level.Loop.GroundGateFollowsOnlyAsFarAsTravelled",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopGroundGateTravel::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const double Dt = 0.05;
	// The ground may turn 0.2 deg for every cm travelled.
	const double Turn = 0.2;
	{
		// Standing: a reading that steps 2 deg is followed at the settling
		// allowance and no faster.
		FGroundGate	 Gate;
		const double Settle = Gate.Tuning.SettleDegPerSec;
		NearBoth(*this, TEXT("the first reading is taken whole"), Gate.Tick(FVector2D(1.0, -0.5), 0.0, Turn, 0.0, Dt), FVector2D(1.0, -0.5), 0.0);
		const FVector2D Stepped = Gate.Tick(FVector2D(3.0, -0.5), 0.0, Turn, 0.0, Dt);
		NearBoth(*this, TEXT("standing: a step is followed at the settling allowance"), Stepped, FVector2D(1.0 + Settle * Dt, -0.5), 1.0e-12);
		Near(*this, TEXT("... which is what it reports"), Gate.GetAllowanceDegPerSec(), Settle, 1.0e-12);
		NearBoth(*this, TEXT("no time passes: nothing moves"), Gate.Tick(FVector2D(9.0, 9.0), 0.0, Turn, 0.0, 0.0), Stepped, 0.0);
		FVector2D Seen = Stepped;
		for (int32 Index = 0; Index < 40; ++Index)
		{
			Seen = Gate.Tick(FVector2D(3.0, -0.5), 0.0, Turn, 0.0, Dt);
		}
		NearBoth(*this, TEXT("a reading that stays is caught up, and exactly"), Seen, FVector2D(3.0, -0.5), 1.0e-12);
		Gate.Reset();
		TestFalse(TEXT("Reset: nothing held"), Gate.IsSeeded());
	}
	{
		// Travelling at 40 cm/s: 8 deg/s more, over pitch and roll together.
		FGroundGate Gate;
		Gate.Seed(FVector2D::ZeroVector);
		const double	Allowed = (Gate.Tuning.SettleDegPerSec + Turn * 40.0) * Dt;
		const FVector2D Seen = Gate.Tick(FVector2D(3.0, 4.0), 40.0, Turn, 0.0, Dt);
		NearBoth(*this, TEXT("travelling: the settling allowance and the travel's, along the reading's own direction"), Seen, FVector2D(0.6, 0.8) * Allowed, 1.0e-12);
		// A reading that turns slower than that (6 deg/s) is the ground's:
		// followed with nothing held back.
		FGroundGate Ramp;
		Ramp.Seed(FVector2D::ZeroVector);
		double Worst = 0.0;
		for (int32 Index = 1; Index <= 20; ++Index)
		{
			const FVector2D Reading(6.0 * Index * Dt, -3.0 * Index * Dt);
			Worst = FMath::Max(Worst, (Ramp.Tick(Reading, 40.0, Turn, 0.0, Dt) - Reading).Size());
		}
		Near(*this, TEXT("a reading inside the allowance is followed exactly"), Worst, 0.0, 1.0e-12);
		// Backwards counts as forwards: it is the distance that matters.
		FGroundGate Back;
		Back.Seed(FVector2D::ZeroVector);
		NearBoth(*this, TEXT("a speed is a speed, whichever way"), Back.Tick(FVector2D(3.0, 4.0), -40.0, Turn, 0.0, Dt), Seen, 1.0e-12);
	}
	{
		// No bound asked for: the reading is the ground, at once.
		FGroundGate Gate;
		Gate.Tuning.BreakDeg = 0.0;
		Gate.Seed(FVector2D::ZeroVector);
		NearBoth(*this, TEXT("no bound: through at once"), Gate.Tick(FVector2D(5.0, -5.0), 0.0, Turn, 0.0, Dt), FVector2D(5.0, -5.0), 0.0);
		Gate.Reset();
		Near(*this, TEXT("Reset keeps the tuning"), Gate.Tuning.BreakDeg, 0.0, 0.0);
	}
	return true;
}

// Catches: a base being braked hard still read for the ground (the knock of
// a stop followed), the settling allowance left on meanwhile, the belief back
// at once (the rock after the stop followed), a driving-off not counted as
// braking is, and a gentle change of speed distrusted (a drive slowed by the
// speed cap, or meeting a grade, left behind).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopGroundGateBraked,
	"Ramms.Level.Loop.GroundGateDistrustsABaseBeingBraked",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopGroundGateBraked::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const double Dt = 0.05;
	const double Turn = 0.2;
	FGroundGate	 Gate;
	Gate.Seed(FVector2D::ZeroVector);
	const FGroundTuning& T = Gate.Tuning;
	TestTrue(TEXT("(the made-up speeds below straddle the tuning's two accelerations)"), T.BodyAccelStartCmS2 == 150.0 && T.BodyAccelFullCmS2 == 300.0 && T.BodySettleSeconds == 0.5);
	Gate.Tick(FVector2D::ZeroVector, 60.0, Turn, 0.0, Dt);
	Near(*this, TEXT("cruising: believed"), Gate.GetTrust(), 1.0, 0.0);

	// 60 to 40 cm/s in a tick: 400 cm/s^2. The reading dips 1.5 deg with it.
	const FVector2D Knock(-1.5, 0.0);
	FVector2D		Seen = Gate.Tick(Knock, 40.0, Turn, 0.0, Dt);
	Near(*this, TEXT("braked hard: not believed at all"), Gate.GetTrust(), 0.0, 0.0);
	NearBoth(*this, TEXT("... and nothing of the dip is taken, not even at the settling allowance"), Seen, FVector2D::ZeroVector, 0.0);

	// Steady again: the belief comes back a tenth a tick (0.05 s of 0.5 s).
	Seen = Gate.Tick(Knock, 40.0, Turn, 0.0, Dt);
	Near(*this, TEXT("steady again: a tenth believed after a tick"), Gate.GetTrust(), 0.1, 1.0e-12);
	Near(*this, TEXT("... and a tenth of the allowance given"), Seen.X, -0.1 * (T.SettleDegPerSec + Turn * 40.0) * Dt, 1.0e-12);
	for (int32 Index = 0; Index < 8; ++Index)
	{
		Gate.Tick(Seen, 40.0, Turn, 0.0, Dt);
	}
	Near(*this, TEXT("nine ticks: nine tenths"), Gate.GetTrust(), 0.9, 1.0e-9);
	Gate.Tick(Seen, 40.0, Turn, 0.0, Dt);
	Near(*this, TEXT("half a second: believed again"), Gate.GetTrust(), 1.0, 1.0e-9);

	// 40 to 35 in a tick, 100 cm/s^2: a drive slowed gently is believed.
	Gate.Tick(Seen, 35.0, Turn, 0.0, Dt);
	Near(*this, TEXT("slowed gently: still believed"), Gate.GetTrust(), 1.0, 0.0);
	// 35 to 23.75, 225 cm/s^2: halfway between the two.
	Gate.Tick(Seen, 23.75, Turn, 0.0, Dt);
	Near(*this, TEXT("halfway between the two accelerations: half believed"), Gate.GetTrust(), 0.5, 1.0e-9);
	// Driving off is the same to a body as braking: 23.75 to 43.75.
	Gate.Tick(Seen, 43.75, Turn, 0.0, Dt);
	Near(*this, TEXT("driven off hard: not believed"), Gate.GetTrust(), 0.0, 0.0);

	// Seeding takes the reading whole and believes the base from there.
	Gate.Seed(FVector2D(2.0, 1.0));
	Near(*this, TEXT("Seed: believed"), Gate.GetTrust(), 1.0, 0.0);
	NearBoth(*this, TEXT("Seed: the reading whole"), Gate.Get(), FVector2D(2.0, 1.0), 0.0);
	// ... and forgets the speed before it: no acceleration on the next tick.
	Gate.Tick(FVector2D(2.0, 1.0), 0.0, Turn, 0.0, Dt);
	Near(*this, TEXT("Seed: the speed before it is forgotten"), Gate.GetTrust(), 1.0, 0.0);
	return true;
}

// Catches: a stance being ramped under a standing robot taken for a knock
// (what its own baselines turned it by not let through), and that let through
// only while the base is believed.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopGroundGateStance,
	"Ramms.Level.Loop.GroundGateLetsTheStancesOwnTurnThrough",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopGroundGateStance::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const double Dt = 0.05;
	FGroundGate	 Gate;
	Gate.Seed(FVector2D::ZeroVector);
	// Standing; the baselines turned the stance 0.3 deg this tick (6 deg/s,
	// far over the settling allowance), and the reading with it.
	FVector2D Seen = Gate.Tick(FVector2D(0.3, 0.0), 0.0, 0.2, 0.3, Dt);
	NearBoth(*this, TEXT("the stance's own turn is followed whole"), Seen, FVector2D(0.3, 0.0), 0.0);
	// A reading that moved further than the baselines explain: the rest at
	// the settling allowance.
	Seen = Gate.Tick(FVector2D(1.3, 0.0), 0.0, 0.2, 0.2, Dt);
	Near(*this, TEXT("more than explained: the rest at the settling allowance"), Seen.X, 0.3 + 0.2 + Gate.Tuning.SettleDegPerSec * Dt, 1.0e-12);
	// A base being braked is not believed, but what its baselines did is known.
	Gate.Seed(FVector2D::ZeroVector);
	Gate.Tick(FVector2D::ZeroVector, 60.0, 0.2, 0.0, Dt);
	Seen = Gate.Tick(FVector2D(0.25, 0.0), 20.0, 0.2, 0.25, Dt);
	Near(*this, TEXT("(braked: not believed)"), Gate.GetTrust(), 0.0, 0.0);
	NearBoth(*this, TEXT("braked: the stance's own turn is followed all the same"), Seen, FVector2D(0.25, 0.0), 1.0e-12);
	return true;
}

// Catches: the wanted tilt's goal not saying how much the legs must leave of
// the correction that would hold the reference, or why: a leg's travel, or an
// axis held at the most it may correct (the clamp's cut taken for the legs').
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopTiltGoalLimit,
	"Ramms.Level.Loop.WantedTiltGoalSaysWhatTheLegsLeaveAndWhy",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopTiltGoalLimit::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const TConstArrayView<FClearancePoint> NoPoints;
	{
		// Inside reach: level, nothing left, nothing to name.
		const FTuning			Tuning;
		const TArray<FLegReach> Legs = SixLegs(12.0, 12.0, 0.0);
		const FTiltGoal			Goal = WantedTiltGoalWithLimit(FVector2D(4.0, 2.0), Legs, NoPoints, Tuning);
		NearBoth(*this, TEXT("inside reach: the goal is level"), Goal.GoalDeg, FVector2D::ZeroVector, 1.0e-12);
		Near(*this, TEXT("inside reach: nothing left to the base"), Goal.ShortDeg, 0.0, 1.0e-12);
		TestTrue(TEXT("inside reach: no limit"), Goal.Limit == EReachLimit::None);
	}
	{
		// 4 cm of travel: 1.5 planned at the corners, 1.909 deg of the 6.
		const FTuning			Tuning;
		const TArray<FLegReach> Legs = SixLegs(4.0, 4.0, 0.0);
		const double			PlannedDeg = FMath::RadiansToDegrees(FMath::Atan(1.5 / 45.0));
		const FTiltGoal			Goal = WantedTiltGoalWithLimit(FVector2D(6.0, 0.0), Legs, NoPoints, Tuning);
		NearBoth(*this, TEXT("short legs: the goal is WantedTiltGoal's"), Goal.GoalDeg, WantedTiltGoal(FVector2D(6.0, 0.0), Legs, NoPoints, Tuning), 0.0);
		Near(*this, TEXT("short legs: what they leave"), Goal.ShortDeg, 6.0 - PlannedDeg, 0.01);
		TestTrue(TEXT("short legs: a leg's travel"), Goal.Limit == EReachLimit::LegTravel);
		// Far past: the goal stops at the tilt policy's 5 deg; what the legs
		// leave is still all of it.
		const FTiltGoal Far = WantedTiltGoalWithLimit(FVector2D(12.0, 0.0), Legs, NoPoints, Tuning);
		Near(*this, TEXT("far past: the goal stops at the tilt policy"), Far.GoalDeg.X, Tuning.MaxTiltDeg, 1.0e-9);
		Near(*this, TEXT("far past: what the legs leave is not cut with it"), Far.ShortDeg, 12.0 - PlannedDeg, 0.01);
	}
	{
		// Plenty of travel, roll held to 3 deg: 5 deg of cross fall leaves 2.
		FTuning Tuning;
		Tuning.MaxCorrectionDeg = FVector2D(15.0, 3.0);
		const TArray<FLegReach> Legs = SixLegs(12.0, 12.0, 0.0);
		const FTiltGoal			Goal = WantedTiltGoalWithLimit(FVector2D(0.0, 5.0), Legs, NoPoints, Tuning);
		NearBoth(*this, TEXT("axis held: the goal keeps what the axis cannot take"), Goal.GoalDeg, FVector2D(0.0, 2.0), 1.0e-9);
		Near(*this, TEXT("axis held: what is left"), Goal.ShortDeg, 2.0, 1.0e-9);
		TestTrue(TEXT("axis held: named as that"), Goal.Limit == EReachLimit::AxisLimit);
		// With legs that run out first, it is the legs that are named.
		const TArray<FLegReach> Short = SixLegs(2.6, 2.6, 0.0);
		TestTrue(TEXT("legs out of travel before the axis is: a leg's travel"), WantedTiltGoalWithLimit(FVector2D(0.0, 5.0), Short, NoPoints, Tuning).Limit == EReachLimit::LegTravel);
		// The reference counts: on ground that is the reference, nothing is left.
		Tuning.LevelReferenceDeg = FVector2D(0.0, 5.0);
		const FTiltGoal AtReference = WantedTiltGoalWithLimit(FVector2D(0.0, 5.0), Legs, NoPoints, Tuning);
		Near(*this, TEXT("on ground that is the reference: nothing left"), AtReference.ShortDeg, 0.0, 1.0e-12);
		TestTrue(TEXT("... and no limit"), AtReference.Limit == EReachLimit::None);
	}
	return true;
}

// Catches: the integral's narrower band (given while the base stands still
// before a rest) also narrowing the proportional term's deadband, applied
// when none is given, or allowed to be wider than the loop's own.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLoopTrimBand,
	"Ramms.Level.Loop.IntegralTrimBand",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLoopTrimBand::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning	Tuning;
	const FVector2D Ground(2.0, 1.0);
	// An error inside the loop's deadband (0.15) on both axes.
	const FVector2D Attitude(0.10, -0.12);
	{
		FLevelLoop		Loop;
		const FVector2D C = Loop.Tick(Attitude, FVector2D::ZeroVector, Ground, 0.1, true, false, Tuning, 0.05);
		NearBoth(*this, TEXT("a trim band of 0.05: the integral takes the error past it"), Loop.GetIntegral(), FVector2D(2.0 * 0.05 * 0.1, 2.0 * -0.07 * 0.1), 1e-12);
		// The proportional term still sees nothing inside the loop's deadband.
		NearBoth(*this, TEXT("... and the proportional term none of it"), C, Ground + Loop.GetIntegral(), 1e-12);
	}
	{
		FLevelLoop Loop;
		Loop.Tick(Attitude, FVector2D::ZeroVector, Ground, 0.1, true, false, Tuning);
		NearBoth(*this, TEXT("no trim band given: the loop's deadband, no integral"), Loop.GetIntegral(), FVector2D::ZeroVector, 1e-12);
		Loop.Tick(Attitude, FVector2D::ZeroVector, Ground, 0.1, true, false, Tuning, 1.0);
		NearBoth(*this, TEXT("a trim band wider than the deadband: the deadband"), Loop.GetIntegral(), FVector2D::ZeroVector, 1e-12);
		Loop.Tick(Attitude, FVector2D::ZeroVector, Ground, 0.1, false, false, Tuning, 0.05);
		NearBoth(*this, TEXT("not integrating: a trim band changes nothing"), Loop.GetIntegral(), FVector2D::ZeroVector, 1e-12);
	}
	return true;
}

// Catches: a ground estimate that cannot follow the ground under a robot turning on the spot on a slope (the slope
// swings from pitch into roll in the chassis' own frame with nothing travelled, so an allowance by the distance alone
// leaves the reading a slope behind in a quarter turn), and one that lets a turn on LEVEL ground, or a knock while
// turning, through as ground.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelGroundGateTurnOnASlope,
	"Ramms.Level.Ground.FollowsASlopeSwungByATurnOnTheSpot",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelGroundGateTurnOnASlope::RunTest(const FString& Parameters)
{
	using namespace RammsLevel;
	const double Dt = 1.0 / 30.0;
	const double SlopeDeg = 6.0;
	const double YawRate = 60.0;
	const auto	 Ground = [SlopeDeg](double YawDeg) {
		  const double Yaw = FMath::DegreesToRadians(YawDeg);
		  return FVector2D(SlopeDeg * FMath::Cos(Yaw), -SlopeDeg * FMath::Sin(Yaw));
	};
	for (const bool bToldTheTurn : { true, false })
	{
		FGroundGate Gate;
		Gate.Seed(Ground(0.0));
		double Behind = 0.0;
		// A quarter turn and a half, standing on the spot (no speed at all).
		for (int32 Index = 1; Index <= 68; ++Index)
		{
			const FVector2D Raw = Ground(YawRate * Dt * Index);
			Behind = FMath::Max(Behind, (Gate.Tick(Raw, 0.0, 0.1, 0.0, Dt, bToldTheTurn ? YawRate : 0.0) - Raw).Size());
		}
		AddInfo(FString::Printf(TEXT("turning at %.0f deg/s on %.0f deg, the turn %s: the gate's reading is %.2f deg behind the ground at most"), YawRate, SlopeDeg, bToldTheTurn ? TEXT("told") : TEXT("not told"), Behind));
		if (bToldTheTurn)
		{
			TestTrue(TEXT("told the turn: the reading keeps up with the ground"), Behind <= 0.05);
		}
		else
		{
			TestTrue(TEXT("(not told, it falls degrees behind: the settling allowance is all it has)"), Behind >= 3.0);
		}
	}
	{
		// Level ground: a turn swings nothing, so it buys no allowance, and a knock while turning is still left out.
		FGroundGate Gate;
		Gate.Seed(FVector2D::ZeroVector);
		const FVector2D After = Gate.Tick(FVector2D(2.0, 0.0), 0.0, 0.1, 0.0, Dt, YawRate);
		TestEqual(TEXT("level ground, a 2 deg knock while turning: followed only as fast as a robot settling"), After.Size(), Gate.Tuning.SettleDegPerSec * Dt, 1.0e-9);
		// The allowance is by the slope believed, which is small: a knock on a gentle slope is mostly left out too.
		FGroundGate Gentle;
		Gentle.Seed(FVector2D(1.0, 0.0));
		const FVector2D Knocked = Gentle.Tick(FVector2D(4.0, 0.0), 0.0, 0.1, 0.0, Dt, YawRate);
		TestEqual(TEXT("a 1 deg slope, a 3 deg knock while turning: the settling allowance and the slope's swing, no more"), Knocked.X - 1.0,
			(Gentle.Tuning.SettleDegPerSec + 1.0 * FMath::DegreesToRadians(YawRate)) * Dt, 1.0e-9);
	}
	return true;
}

// Catches: a speed taken over the wrong time (the tick's, not the readings'), a reading given twice taken for a
// stop, a clock that ran backwards taken for a huge speed, a turn's sign or size.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelMotionSenseTest,
	"Ramms.Level.Loop.MotionSenseIsOnTheReadingsOwnTime",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelMotionSenseTest::RunTest(const FString& Parameters)
{
	using namespace RammsLevel;

	FMotionSense	Motion;
	const FVector2D East(1.0, 0.0);
	TestEqual(TEXT("the first reading takes no time"), Motion.Tick(FVector(0.0, 0.0, 0.0), East, 10.0), 0.0);
	TestEqual(TEXT("... and gives no speed"), Motion.GetSpeedCmS(), 0.0);
	TestTrue(TEXT("... but is a reading"), Motion.HasReading());

	// 50 cm/s, read at uneven moments: the speed is the same at every one.
	double		 At = 10.0;
	const double Gaps[5] = { 0.02, 0.07, 0.033, 0.014, 0.052 };
	for (const double Gap : Gaps)
	{
		At += Gap;
		const double Since = Motion.Tick(FVector(50.0 * (At - 10.0), 0.0, 0.0), East, At);
		TestEqual(TEXT("the time since the last reading is the readings' own"), Since, Gap, 1.0e-12);
		TestEqual(TEXT("a steady pace reads steady, however unevenly it is read"), Motion.GetSpeedCmS(), 50.0, 1.0e-6);
		TestEqual(TEXT("... the distance with it"), Motion.MovedCm(), 50.0 * Gap, 1.0e-9);
	}

	// The same reading again: nothing new, and not a stop.
	TestEqual(TEXT("a reading given twice takes no time"), Motion.Tick(FVector(50.0 * (At - 10.0), 0.0, 0.0), East, At), 0.0);
	TestEqual(TEXT("... and changes no speed"), Motion.GetSpeedCmS(), 50.0, 1.0e-6);
	TestEqual(TEXT("... nor moved anything"), Motion.MovedCm(), 0.0);

	// A turn of 9 deg in 0.1 s, going nowhere.
	const double Was = Motion.GetSampleSeconds();
	Motion.Tick(FVector(50.0 * (At - 10.0), 0.0, 0.0), FVector2D(FMath::Cos(FMath::DegreesToRadians(9.0)), FMath::Sin(FMath::DegreesToRadians(9.0))), Was + 0.1);
	TestEqual(TEXT("a turn on the spot: no speed"), Motion.GetSpeedCmS(), 0.0, 1.0e-9);
	TestEqual(TEXT("... 90 deg/s"), Motion.GetYawRateDegPerSec(), 90.0, 1.0e-6);
	Motion.Tick(FVector(50.0 * (At - 10.0), 0.0, 0.0), FVector2D::ZeroVector, Was + 0.2);
	TestEqual(TEXT("a heading that is not known: no turning rate"), Motion.GetYawRateDegPerSec(), 0.0);

	// A reading that is not a number is no reading.
	TestEqual(TEXT("a time that is not a number takes none"), Motion.Tick(FVector(1.0e6, 0.0, 0.0), East, NAN), 0.0);
	TestEqual(TEXT("... and is not kept"), Motion.GetSampleSeconds(), Was + 0.2, 1.0e-12);

	// Time ran backwards (a simulation reset): begun again, no speed made of it.
	TestEqual(TEXT("a reading from before the last takes no time"), Motion.Tick(FVector(0.0, 0.0, 0.0), East, 1.0), 0.0);
	TestTrue(TEXT("... says the clock ran backwards"), Motion.RanBackwards());
	TestEqual(TEXT("... and gives no speed"), Motion.GetSpeedCmS(), 0.0);
	Motion.Tick(FVector(3.0, 0.0, 0.0), East, 1.1);
	TestFalse(TEXT("... once"), Motion.RanBackwards());
	TestEqual(TEXT("... and goes on from there"), Motion.GetSpeedCmS(), 30.0, 1.0e-6);
	return true;
}

// Catches: a cap that stops the robot. The drive it asks is a speed loop: asked for much less than the speed the
// robot has, it brakes through what was asked. So the speed asked is led down from the speed there is (never more
// than a fifth of it and 5 cm/s under), the last of the way gently; and a robot going FASTER than its stick asks
// (run away down the far side of a crest) is asked for less in proportion to the excess, only while it is.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelSpeedScaleLed,
	"Ramms.Level.SpeedScale.LedDownNeverThrough",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelSpeedScaleLed::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning Tuning;
	const double  Dt = 1.0 / 16.0;
	{
		// At 60 cm/s, its stick asking 60, the legs suddenly far behind (the cap wants its floor): a drive that
		// slows at 60 cm/s^2 whatever it is asked. The speed asked is never far under the speed there is.
		FSpeedScale		Scale;
		FSpeedCapInputs In = CapError(9.0);
		In.AskedSpeedCmS = 60.0;
		double Speed = 60.0;
		double LeastLead = 0.0;
		double Biggest = 0.0;
		double Was = 1.0;
		for (int32 i = 0; i < 64; ++i)
		{
			In.SpeedCmS = Speed;
			const double Now = Scale.Tick(In, Dt, Tuning);
			LeastLead = FMath::Min(LeastLead, Now * 60.0 - (0.8 * Speed - 5.0));
			Biggest = FMath::Max(Biggest, Was - Now);
			Was = Now;
			Speed = FMath::Max(Now * 60.0, Speed - 60.0 * Dt);
		}
		TestTrue(FString::Printf(TEXT("the speed asked is never more than a fifth of the speed there is, and 5 cm/s, under it (%.2f cm/s)"), LeastLead), LeastLead >= -1.0e-6);
		Near(*this, TEXT("... and the floor is reached once the robot has come down to it"), Scale.Get(), Tuning.MinSpeedScale, 1e-6);
		TestTrue(FString::Printf(TEXT("... the scale never falling faster than its rate (%.3f a tick)"), Biggest), Biggest <= 2.0 * Dt + 1.0e-9);
	}
	{
		// A robot that does not slow at all (gravity has it): the scale waits a fifth under its speed, for good.
		FSpeedScale		Scale;
		FSpeedCapInputs In = CapError(9.0);
		In.AskedSpeedCmS = 60.0;
		In.SpeedCmS = 60.0;
		Near(*this, TEXT("a robot that is not slowing is not asked for a crawl"), SettledScale(Scale, In, Tuning), (0.8 * 60.0 - 5.0) / 60.0, 1e-6);
	}
	{
		// The last of the way down is gentle: closing on a target a tenth away, the step is the gap over a quarter
		// of a second, not the rate's.
		FSpeedScale		Scale;
		FSpeedCapInputs In = CapError(1.2); // 0.925 of full speed
		Near(*this, TEXT("a small way down: a quarter of the gap in a sixteenth of a second"), Scale.Tick(In, Dt, Tuning), 1.0 - 0.075 * Dt / 0.25, 1e-9);
	}
	{
		// Faster than its stick asks: asked for less by twice the excess, at once, and back at once.
		FSpeedScale		Scale;
		FSpeedCapInputs In;
		In.AskedSpeedCmS = 60.0;
		In.SpeedCmS = 60.0;
		Near(*this, TEXT("at the speed asked: full"), Scale.Tick(In, Dt, Tuning), 1.0, 1e-12);
		In.SpeedCmS = 66.0;
		Near(*this, TEXT("a tenth over: asked for a fifth less, at once"), Scale.Tick(In, Dt, Tuning), 0.8, 1e-9);
		In.SpeedCmS = 120.0;
		Near(*this, TEXT("twice the speed asked: the floor, never under it"), Scale.Tick(In, Dt, Tuning), Tuning.MinSpeedScale, 1e-12);
		In.SpeedCmS = 63.0;
		Near(*this, TEXT("nearly back: nearly full, at once (it fades as the robot comes back)"), Scale.Tick(In, Dt, Tuning), 0.9, 1e-9);
		In.SpeedCmS = 55.0;
		Near(*this, TEXT("under the speed asked: full"), Scale.Tick(In, Dt, Tuning), 1.0, 1e-12);
		// A drive that cannot say what its stick asks: no such term (the speed there is, is all that is known).
		FSpeedScale		Blind;
		FSpeedCapInputs Unknown;
		Unknown.SpeedCmS = 60.0;
		Blind.Tick(Unknown, Dt, Tuning);
		Unknown.SpeedCmS = 90.0;
		TestTrue(TEXT("a drive that cannot say what is asked is not taken to be running away"), Blind.Tick(Unknown, Dt, Tuning) > 0.95);
	}
	return true;
}

// Catches: a cap that answers what its own braking does (a drive made to slow pitches the body nose down, the
// estimate follows some of it, and the cap slows for that: the speed hunts), or a start; and one that slows for the
// slope swinging round in the chassis' frame when the chassis is turned about the vertical, which going slower
// does nothing for.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelSpeedScaleSteady,
	"Ramms.Level.SpeedScale.DoesNotAnswerItsOwnBrakingNorATurnOnASlope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelSpeedScaleSteady::RunTest(const FString& Parameters)
{
	using namespace RammsLevelLoopTests;
	const FTuning Tuning;
	const double  Dt = 1.0 / 16.0;
	const auto	  Turning = [](double AskedCmS, double SpeedCmS, const FVector2D& RateDegPerSec) {
		   FSpeedCapInputs In = CapSlope(0.0, 45.0, 10.0);
		   In.AskedSpeedCmS = AskedCmS;
		   In.SpeedCmS = SpeedCmS;
		   In.bGroundRateKnown = true;
		   In.GroundRateDegPerSec = RateDegPerSec;
		   return In;
	};
	{
		// A start: the stick goes from nothing to 60 cm/s, and for the next half second the estimate turns at
		// 20 deg/s (the body pitching back as the drive pushes off). Not the ground: not answered.
		FSpeedScale Scale;
		Scale.Tick(Turning(0.0, 0.0, FVector2D::ZeroVector), Dt, Tuning);
		double Least = 1.0;
		for (int32 i = 0; i < 8; ++i)
		{
			Least = FMath::Min(Least, Scale.Tick(Turning(60.0, 7.0 * (i + 1), FVector2D(20.0, 0.0)), Dt, Tuning));
		}
		Near(*this, TEXT("a start's pitch is not answered"), Least, 1.0, 1e-12);
		TestTrue(TEXT("... nor kept"), Scale.GetTurnDegPerSec().Size() < 0.5);
		// Driven steadily after, the same turning IS the ground's.
		for (int32 i = 0; i < 16; ++i)
		{
			Least = FMath::Min(Least, Scale.Tick(Turning(60.0, 60.0, FVector2D(20.0, 0.0)), Dt, Tuning));
		}
		TestTrue(FString::Printf(TEXT("the same turning under a robot driven steadily is slowed for (%.2f)"), Least), Least < 0.8);
	}
	{
		// Slowing for a ground that turns at 12 deg/s: while the cap itself is bringing the speed asked down, a
		// reading twice as large (the body pitching with the braking) is not taken up; what was read stands.
		FSpeedScale Scale;
		for (int32 i = 0; i < 16; ++i)
		{
			Scale.Tick(Turning(60.0, 60.0, FVector2D::ZeroVector), Dt, Tuning);
		}
		for (int32 i = 0; i < 4; ++i)
		{
			Scale.Tick(Turning(60.0, 60.0, FVector2D(12.0, 0.0)), Dt, Tuning);
		}
		const double Read = Scale.GetTurnDegPerSec().X;
		TestTrue(FString::Printf(TEXT("(the ground's turning read, %.1f deg/s, and the scale coming down, %.2f)"), Read, Scale.Get()), Read > 6.0 && Scale.Get() < 1.0);
		TestTrue(FString::Printf(TEXT("(the drive is being asked to slow: the turning is not being taken up, %.2f)"), Scale.GetSteady()), Scale.GetSteady() < 0.5);
		double Most = Read;
		for (int32 i = 0; i < 3; ++i)
		{
			Scale.Tick(Turning(60.0, 50.0 - 10.0 * i, FVector2D(24.0, 0.0)), Dt, Tuning);
			Most = FMath::Max(Most, Scale.GetTurnDegPerSec().X);
		}
		TestTrue(FString::Printf(TEXT("a reading made while the cap is braking is not taken for the ground's (%.1f against %.1f deg/s)"), Most, Read), Most <= Read + 1.5);
	}
	{
		// On a 6 deg slope, turned about the vertical at 60 deg/s: the slope swings 6.3 deg/s in the chassis' frame.
		// That much of the estimate's turning is not a break in the ground.
		FSpeedCapInputs In = Turning(60.0, 60.0, FVector2D(-3.0, 5.2)); // 6 deg/s
		In.SlopeDeg = 6.0;
		In.YawRateDegPerSec = 60.0;
		FSpeedScale Turned;
		Near(*this, TEXT("a slope swung round by the chassis' own turning: full speed"), SettledScale(Turned, In, Tuning), 1.0, 1e-12);
		Near(*this, TEXT("... nothing of it kept as the ground's turning"), Turned.GetTurnDegPerSec().Size(), 0.0, 1e-9);
		In.GroundRateDegPerSec = FVector2D(-6.0, 10.4); // 12 deg/s: 5.7 of it is not the turn's
		FSpeedScale Both;
		SettledScale(Both, In, Tuning);
		Near(*this, TEXT("what is over the swing is the ground's"), Both.GetTurnDegPerSec().Size(), 12.0 - 6.0 * FMath::DegreesToRadians(60.0), 0.05);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
