// Tests for the per-wheel crossing's runtime checks (Private/Curb/RammsCurbSafety):
// the time a condition has held, the base against a planned attitude, the
// terrain monitor's look at the traces ahead of and under each wheel, a lifted
// wheel's rim over the traced terrain, the live margin's watch, and the
// runaway watch that has a robot its driven wheels do not hold braked at once
// -- each fed made-up numbers on a made-up terrain.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Curb/RammsCurbExecutor.h"
#include "Curb/RammsCurbRobotModel.h"
#include "Curb/RammsCurbRunStats.h"
#include "Curb/RammsCurbSafety.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	/** A made-up curb: the near surface at height 0 sloping NearGrad, an edge
	 *  square to +X at x = 100, the far surface Step above it there sloping
	 *  FarGrad, Depth deep. */
	FRammsCurbTerrain SafetyTerrain(double Step, double Depth, const FVector2D& NearGrad = FVector2D::ZeroVector,
		const FVector2D& FarGrad = FVector2D::ZeroVector)
	{
		FRammsCurbTerrain T;
		T.bValid = true;
		T.EdgeNormal = FVector2D(1.0, 0.0);
		T.EdgeDirection = FVector2D(0.0, 1.0);
		T.Near.Origin = FVector::ZeroVector;
		T.Near.Gradient = NearGrad;
		T.EdgePoint = FVector(100.0, 0.0, T.Near.HeightAt(FVector2D(100.0, 0.0)));
		T.Far.Origin = FVector(100.0, 0.0, T.EdgePoint.Z + Step);
		T.Far.Gradient = FarGrad;
		T.StepHeightCm = static_cast<float>(Step);
		T.FarDepthCm = static_cast<float>(Depth);
		T.bFarDepthObserved = true;
		return T;
	}

	RammsCurb::FTerrainProbe Probe(int32 Leg, double X, double Y, double Z, bool bHit = true)
	{
		RammsCurb::FTerrainProbe P;
		P.Leg = Leg;
		P.XY = FVector2D(X, Y);
		P.Z = Z;
		P.bHit = bHit;
		P.AboveZ = Z + 40.0;
		P.BelowZ = Z - 60.0;
		return P;
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSafetyPersist,
	"Ramms.Curb.Safety.Persist",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSafetyPersist::RunTest(const FString& Parameters)
{
	// True once the condition has held the time asked, without a break; any
	// break starts it again.
	RammsCurb::FPersist P;
	int32				Ticks = 0;
	while (!P.Tick(true, 0.01, 0.25) && Ticks < 1000)
	{
		++Ticks;
	}
	TestEqual(TEXT("held 0.25 s at 0.01 s a tick: true on the 25th"), Ticks + 1, 25);
	TestFalse(TEXT("a break: false"), P.Tick(false, 0.01, 0.25));
	TestEqual(TEXT("a break: from 0 again"), P.Seconds, 0.0);
	TestFalse(TEXT("after the break, not yet"), P.Tick(true, 0.2, 0.25));
	TestTrue(TEXT("held long enough again"), P.Tick(true, 0.06, 0.25));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSafetyAttitudeError,
	"Ramms.Curb.Safety.AttitudeError",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSafetyAttitudeError::RunTest(const FString& Parameters)
{
	// Against a level plan the error is the base's inclination; against
	// itself 0; between two attitudes the angle between the planes the chassis
	// lies in -- the one its up axes make, checked against chassis frames.
	using RammsCurb::AttitudeErrorDeg;
	for (const FVector2D A : { FVector2D(4.0, 0.0), FVector2D(0.0, -3.0), FVector2D(3.0, 3.0), FVector2D(-2.5, 1.5) })
	{
		TestTrue(FString::Printf(TEXT("(%+.1f, %+.1f) against level: its inclination"), A.X, A.Y),
			FMath::IsNearlyEqual(AttitudeErrorDeg(A, FVector2D::ZeroVector), RammsCurb::InclinationDeg(A.X, A.Y), 1e-6));
		TestTrue(FString::Printf(TEXT("(%+.1f, %+.1f) against itself: 0"), A.X, A.Y), AttitudeErrorDeg(A, A) < 1e-6);
	}
	struct FCase
	{
		FVector2D Measured;
		FVector2D Planned;
	};
	for (const FCase& C : { FCase{ FVector2D(3.0, 1.0), FVector2D(-1.0, 0.5) }, FCase{ FVector2D(0.0, 4.5), FVector2D(2.0, 2.0) },
			 FCase{ FVector2D(-3.5, -2.0), FVector2D(1.25, 2.5) } })
	{
		const RammsCurb::FChassisFrame Fm = RammsCurb::ChassisFrameFrom(FVector::ForwardVector, FMath::DegreesToRadians(C.Measured.X),
			FMath::DegreesToRadians(C.Measured.Y), FVector::ZeroVector, FVector::ZeroVector);
		const RammsCurb::FChassisFrame Fp = RammsCurb::ChassisFrameFrom(FVector::ForwardVector, FMath::DegreesToRadians(C.Planned.X),
			FMath::DegreesToRadians(C.Planned.Y), FVector::ZeroVector, FVector::ZeroVector);
		const double				   Want = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(FVector::DotProduct(Fm.Z, Fp.Z), -1.0, 1.0)));
		const double				   Got = AttitudeErrorDeg(C.Measured, C.Planned);
		TestTrue(FString::Printf(TEXT("(%+.2f, %+.2f) against (%+.2f, %+.2f): %.4f deg, the frames' up axes %.4f"), C.Measured.X, C.Measured.Y,
					 C.Planned.X, C.Planned.Y, Got, Want),
			FMath::IsNearlyEqual(Got, Want, 1e-4));
		TestTrue(TEXT("either way round"), FMath::IsNearlyEqual(Got, AttitudeErrorDeg(C.Planned, C.Measured), 1e-9));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSafetyCheckTerrain,
	"Ramms.Curb.Safety.CheckTerrain",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSafetyCheckTerrain::RunTest(const FString& Parameters)
{
	using RammsCurb::CheckTerrain;
	using RammsCurb::FTerrainCheck;
	using RammsCurb::FTerrainProbe;
	// A 12 cm curb, both surfaces sloping 4 deg in different ways, its far side
	// 80 cm deep.
	const double			Grad = FMath::Tan(FMath::DegreesToRadians(4.0));
	const FRammsCurbTerrain T = SafetyTerrain(12.0, 80.0, FVector2D(Grad, 0.0), FVector2D(0.0, -Grad));
	auto					On = [&](int32 Leg, double X, double Y, double Off) {
		   const FVector2D XY(X, Y);
		   const double	   Z = (T.DistancePastEdge(XY) >= 0.0 ? T.Far : T.Near).HeightAt(XY) + Off;
		   return Probe(Leg, X, Y, Z);
	};
	{
		// On the planes: nothing off; a probe per surface counted, the face band
		// and past the far depth left out.
		const TArray<FTerrainProbe> P = { On(0, 50.0, 10.0, 0.0), On(1, 90.0, -20.0, 0.0), On(2, 98.0, 0.0, 6.0), On(3, 103.0, 5.0, -4.0),
			On(4, 130.0, 15.0, 0.0), On(5, 179.0, 0.0, 9.0) };
		const FTerrainCheck			C = CheckTerrain(T, P, FVector2D::ZeroVector);
		TestTrue(FString::Printf(TEXT("on the planes: worst %.4f cm"), C.WorstCm), FMath::Abs(C.WorstCm) < 1e-9);
		TestEqual(TEXT("two probes on the near surface (one in the face band left out)"), C.Probes[0], 2);
		TestEqual(TEXT("one on the far (one in the band, one past its depth left out)"), C.Probes[1], 1);
	}
	{
		// The far surface 2.6 cm lower than detected: the worst, signed, its leg
		// and surface; the medians by surface.
		TArray<FTerrainProbe> P = { On(0, 60.0, 0.0, 0.1), On(1, 70.0, 5.0, -0.1), On(2, 110.0, 0.0, -2.6), On(3, 120.0, 10.0, -2.6),
			On(4, 140.0, -10.0, -2.6) };
		P[3].AheadCm = 20.0;
		const FTerrainCheck C = CheckTerrain(T, P, FVector2D::ZeroVector);
		TestTrue(FString::Printf(TEXT("the worst %.3f cm, leg %d, surface %d"), C.WorstCm, C.WorstLeg, C.WorstPlane),
			FMath::IsNearlyEqual(C.WorstCm, -2.6, 1e-6) && C.WorstPlane == 1 && C.WorstLeg >= 2);
		TestTrue(FString::Printf(TEXT("the near median %.3f"), C.MedianCm[0]), FMath::Abs(C.MedianCm[0]) < 1e-6);
		TestTrue(FString::Printf(TEXT("the far median %.3f"), C.MedianCm[1]), FMath::IsNearlyEqual(C.MedianCm[1], -2.6, 1e-6));
		// Against a far bias of -1.5 the same probes are 1.1 cm off.
		const FTerrainCheck B = CheckTerrain(T, P, FVector2D(0.0, -1.5));
		TestTrue(FString::Printf(TEXT("against the bias: %.3f"), B.WorstCm), FMath::IsNearlyEqual(B.WorstCm, -1.1, 1e-6));
	}
	{
		// A ridge 2.5 cm high on the far surface under one probe: the worst, the
		// median of the rest not moved by one.
		const TArray<FTerrainProbe> P = { On(0, 110.0, 0.0, 0.0), On(1, 125.0, 0.0, 2.5), On(2, 140.0, 0.0, 0.0) };
		const FTerrainCheck			C = CheckTerrain(T, P, FVector2D::ZeroVector);
		TestTrue(FString::Printf(TEXT("the ridge: worst %+.2f from leg %d"), C.WorstCm, C.WorstLeg), FMath::IsNearlyEqual(C.WorstCm, 2.5, 1e-6) && C.WorstLeg == 1);
		TestTrue(FString::Printf(TEXT("the median %.2f"), C.MedianCm[1]), FMath::Abs(C.MedianCm[1]) < 1e-6);
	}
	{
		// A trace that hit nothing where its surface lies within the span traced:
		// a drop at least down to the span's bottom. One whose surface lies
		// outside the span says nothing.
		FTerrainProbe Hole = On(0, 60.0, 0.0, 0.0);
		Hole.bHit = false;
		FTerrainProbe Far = On(1, 60.0, 30.0, 0.0);
		Far.bHit = false;
		Far.AboveZ = Far.Z - 50.0;
		Far.BelowZ = Far.Z - 100.0;
		const FTerrainCheck C = CheckTerrain(T, TArray<FTerrainProbe>{ Hole, Far }, FVector2D::ZeroVector);
		TestTrue(FString::Printf(TEXT("a drop: %.1f cm, leg %d"), C.WorstCm, C.WorstLeg), FMath::IsNearlyEqual(C.WorstCm, -60.0, 1e-6) && C.WorstLeg == 0);
		TestEqual(TEXT("the trace that could not have hit it left out"), C.Probes[0], 1);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSafetyRimGap,
	"Ramms.Curb.Safety.RimGap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSafetyRimGap::RunTest(const FString& Parameters)
{
	using RammsCurb::FTerrainProbe;
	using RammsCurb::RimGap;
	// A 10 cm wheel rolling along +X, 6 cm wide, its centre 12 cm over flat
	// ground at 0: 2 cm under its bottom.
	const FVector		  C(0.0, 0.0, 12.0);
	const FVector2D		  F(1.0, 0.0);
	const double		  R = 10.0;
	TArray<FTerrainProbe> P = { Probe(3, -5.0, 0.0, 0.0), Probe(3, 0.0, 0.0, 0.0), Probe(3, 5.0, 0.0, 0.0) };
	TestTrue(FString::Printf(TEXT("flat ground: %.4f cm under its bottom"), RimGap(3, C, R, F, 6.0, P)),
		FMath::IsNearlyEqual(RimGap(3, C, R, F, 6.0, P), 2.0, 1e-9));
	// A step 8 cm up 5 cm ahead: the rim there is sqrt(100 - 25) under the
	// centre, 3.34 cm over the ground -- 4.66 cm into the step.
	P[2].Z = 8.0;
	const double Want = C.Z - FMath::Sqrt(R * R - 25.0) - 8.0;
	TestTrue(FString::Printf(TEXT("a step ahead: %.4f against %.4f"), RimGap(3, C, R, F, 6.0, P), Want), FMath::IsNearlyEqual(RimGap(3, C, R, F, 6.0, P), Want, 1e-9));
	// Another leg's probes, ones off the tread or past the rim, do not count.
	TArray<FTerrainProbe> Q = { Probe(2, 0.0, 0.0, 11.0), Probe(3, 0.0, 4.0, 11.0), Probe(3, 10.0, 0.0, 11.0) };
	TestEqual(TEXT("nothing under it: the largest double"), RimGap(3, C, R, F, 6.0, Q), TNumericLimits<double>::Max());
	// Turned 30 deg off the probes' line: a probe 5 cm along +X is 4.33 along
	// its rolling direction, 2.5 across -- inside a 6 cm tread.
	const FVector2D		  F30(FMath::Cos(FMath::DegreesToRadians(30.0)), FMath::Sin(FMath::DegreesToRadians(30.0)));
	TArray<FTerrainProbe> T30 = { Probe(3, 5.0, 0.0, 4.0) };
	const double		  U = 5.0 * F30.X;
	TestTrue(TEXT("turned 30 deg: the rim along its own rolling direction"),
		FMath::IsNearlyEqual(RimGap(3, C, R, F30, 6.0, T30), C.Z - FMath::Sqrt(R * R - U * U) - 4.0, 1e-9));
	TestEqual(TEXT("turned 30 deg on a 4 cm tread: off it"), RimGap(3, C, R, F30, 4.0, T30), TNumericLimits<double>::Max());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSafetyMarginWatch,
	"Ramms.Curb.Safety.MarginWatch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSafetyMarginWatch::RunTest(const FString& Parameters)
{
	// Over the floor: carry on. Under it: hold, until back over it by
	// MarginReleaseCm. Under MarginFaultCm for MarginFaultSeconds: fault, not
	// before.
	using RammsCurb::EMarginWatch;
	RammsCurb::FMarginWatch W;
	const double			Live = 1.5;
	TestTrue(TEXT("3 cm: ok"), W.Tick(3.0, Live, 0.01) == EMarginWatch::Ok);
	TestTrue(TEXT("1.4 cm: hold"), W.Tick(1.4, Live, 0.01) == EMarginWatch::Hold);
	TestTrue(TEXT("1.6 cm: still holding (under the release)"), W.Tick(1.6, Live, 0.01) == EMarginWatch::Hold);
	TestTrue(TEXT("1.8 cm: over the release, ok"), W.Tick(1.8, Live, 0.01) == EMarginWatch::Ok);
	int32 Ticks = 0;
	while (W.Tick(0.3, Live, 0.01) != EMarginWatch::Fault && Ticks < 100)
	{
		++Ticks;
	}
	TestTrue(FString::Printf(TEXT("0.3 cm: a fault after %d ticks of 0.01 s (%.2f s)"), Ticks + 1, RammsCurb::MarginFaultSeconds),
		Ticks + 1 >= FMath::RoundToInt(RammsCurb::MarginFaultSeconds / 0.01) && Ticks + 1 <= FMath::RoundToInt(RammsCurb::MarginFaultSeconds / 0.01) + 1);
	RammsCurb::FMarginWatch V;
	V.Tick(0.3, Live, 0.05);
	TestTrue(TEXT("a dip under 0.5 for less than the fault time: a hold, no fault"), V.Tick(1.0, Live, 0.05) == EMarginWatch::Hold);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSafetyPressWatch,
	"Ramms.Curb.Safety.PressWatch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSafetyPressWatch::RunTest(const FString& Parameters)
{
	// A landing leg going down with its wheel: not pressing. Its wheel held
	// still while the leg goes on down (lifting the robot): pressing, at the
	// end of a window. A leg standing still, or one going down less than
	// PressLegCm over a window, or a wheel moving more than PressWheelCm:
	// not.
	const double Dt = 0.01;
	auto		 Run = [Dt](double WheelRate, double LegRate, double Seconds, double Jitter = 0.0) {
		RammsCurb::FPressWatch W;
		int32				   Presses = 0;
		double				   FirstAt = -1.0;
		for (int32 k = 0; k * Dt <= Seconds + 1e-9; ++k)
		{
			const double T = k * Dt;
			const double Wiggle = (k % 2 == 0 ? 1.0 : -1.0) * Jitter;
			if (W.Tick(100.0 + WheelRate * T + Wiggle, -5.0 + LegRate * T, Dt))
			{
				FirstAt = FirstAt < 0.0 ? T : FirstAt;
				++Presses;
			}
		}
		return TPair<int32, double>(Presses, FirstAt);
	};
	const TPair<int32, double> Down = Run(-20.0, -20.0, 1.0);
	TestEqual(TEXT("wheel and leg going down together (20 cm/s): never pressing"), Down.Key, 0);
	const TPair<int32, double> Press = Run(0.0, -2.0, 1.0, 0.05);
	TestTrue(FString::Printf(TEXT("the wheel still (0.05 cm jitter), its leg going down 2 cm/s: pressing, first at %.2f s"), Press.Value),
		Press.Key >= 2 && Press.Value >= RammsCurb::PressWindowSeconds - 1e-9 && Press.Value <= RammsCurb::PressWindowSeconds + 2.0 * Dt + 1e-9);
	TestEqual(TEXT("the wheel still, its leg still: not pressing"), Run(0.0, 0.0, 1.0).Key, 0);
	TestEqual(TEXT("the wheel still, its leg going down 1 cm/s (0.3 cm a window): not pressing"), Run(0.0, -1.0, 1.0).Key, 0);
	TestEqual(TEXT("the wheel going down 1 cm/s, its leg 3 cm/s: not pressing"), Run(-1.0, -3.0, 1.0).Key, 0);
	TestEqual(TEXT("the wheel still, its leg going up (the robot settling onto it): not pressing"), Run(0.0, 2.0, 1.0).Key, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbSafetyRunawayWatch,
	"Ramms.Curb.Safety.RunawayWatch",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbSafetyRunawayWatch::RunTest(const FString& Parameters)
{
	// The runaway watch, as a robot being braked to rest has it (both ways
	// armed): braked from 30 cm/s at 60 or 120 cm/s^2, the chassis following
	// its command 0.15 s behind -- never. The command at rest but the chassis
	// rolling on at 30 cm/s (its driven wheels not holding it): running away,
	// within a second and a half. Rolling back at 8 cm/s: within a tick or two
	// of RunawaySeconds; at 4 cm/s (under RollBackCmS), or with the way back
	// not armed: never. A roll back broken for a tick starts again.
	const double Dt = 1.0 / 60.0;
	struct FRun
	{
		RammsCurb::ERunaway Found = RammsCurb::ERunaway::None;
		double				At = -1.0;
	};
	auto Run = [Dt](TFunctionRef<double(double)> Command, TFunctionRef<double(double, double, double)> Measured, double Seconds, bool bAhead, bool bBack) {
		FRun   R;
		double Followed = Command(0.0), Runaway = 0.0, Chassis = Command(0.0);
		for (int32 k = 1; k * Dt <= Seconds + 1e-9; ++k)
		{
			const double T = k * Dt;
			const double Cmd = Command(T);
			Chassis = Measured(T, Cmd, Chassis);
			const RammsCurb::ERunaway Found = RammsCurb::RunawayStep(Followed, Runaway, Cmd, Chassis, bAhead, bBack, Dt);
			if (Found != RammsCurb::ERunaway::None && R.Found == RammsCurb::ERunaway::None)
			{
				R.Found = Found;
				R.At = T;
			}
		}
		return R;
	};
	auto Brake = [](double Decel) { return [Decel](double T) { return FMath::Max(0.0, 30.0 - Decel * T); }; };
	auto Follows = [Dt](double, double Cmd, double Was) { return Was + (Cmd - Was) * FMath::Min(1.0, Dt / 0.15); };
	TestEqual(TEXT("braked at 60 cm/s^2, the chassis following: never"), static_cast<int32>(Run(Brake(60.0), Follows, 3.0, true, true).Found), 0);
	TestEqual(TEXT("braked at 120 cm/s^2, the chassis following: never"), static_cast<int32>(Run(Brake(120.0), Follows, 3.0, true, true).Found), 0);
	const FRun Away = Run(Brake(60.0), [](double, double, double) { return 30.0; }, 3.0, true, true);
	TestTrue(FString::Printf(TEXT("the chassis rolling on at 30 cm/s as the command falls to 0: running away, at %.2f s"), Away.At),
		Away.Found == RammsCurb::ERunaway::Ahead && Away.At > RammsCurb::RunawaySeconds && Away.At <= 1.5);
	auto	   Rest = [](double) { return 0.0; };
	const FRun Back = Run(Rest, [](double, double, double) { return -8.0; }, 2.0, true, true);
	TestTrue(FString::Printf(TEXT("rolling back at 8 cm/s: rolling back, at %.3f s"), Back.At),
		Back.Found == RammsCurb::ERunaway::Back && Back.At > RammsCurb::RunawaySeconds && Back.At <= RammsCurb::RunawaySeconds + 2.0 * Dt + 1e-9);
	TestEqual(TEXT("rolling back at 4 cm/s (under RollBackCmS): never"),
		static_cast<int32>(Run(Rest, [](double, double, double) { return -4.0; }, 2.0, true, true).Found), 0);
	TestEqual(TEXT("rolling back at 8 cm/s, the way back not armed: never"),
		static_cast<int32>(Run(Rest, [](double, double, double) { return -8.0; }, 2.0, true, false).Found), 0);
	// Back at 8 cm/s for 0.3 s, a tick at rest, then back again: from the tick
	// at rest on.
	const FRun Broken = Run(
		Rest, [Dt](double T, double, double) { return FMath::Abs(T - 0.3) < 0.5 * Dt ? 0.0 : -8.0; }, 2.0, true, true);
	TestTrue(FString::Printf(TEXT("a roll back broken for a tick at 0.3 s: counted again from there, at %.3f s"), Broken.At),
		Broken.Found == RammsCurb::ERunaway::Back && Broken.At > 0.3 + RammsCurb::RunawaySeconds);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
