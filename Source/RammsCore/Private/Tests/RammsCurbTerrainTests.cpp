// Tests for the curb controller's terrain model (Private/Curb).
//
// The step detection (RammsCurb::FitTerrain) run over made-up analytic
// terrain -- two planes either side of a straight edge, sloped, graded,
// twisted, noisy, approached at an angle, or bent into something that is not
// a step -- with a made-up six-wheeled robot standing on it; and the contact
// geometry (the wheel-centre height on a slope, the clearance a lifted wheel
// needs over the edge) against closed forms and brute force.

#include "CoreMinimal.h"
#include "Math/RandomStream.h"
#include "Misc/AutomationTest.h"

#include "Curb/RammsCurbTerrainFit.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	using RammsCurb::FTerrainFitInput;
	using RammsCurb::FTerrainFitReport;
	using RammsCurb::FTerrainWheel;

	constexpr EAutomationTestFlags TestFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;

	double Tand(double Deg)
	{
		return FMath::Tan(FMath::DegreesToRadians(Deg));
	}

	FVector2D Dir(double Deg)
	{
		return FVector2D(FMath::Cos(FMath::DegreesToRadians(Deg)), FMath::Sin(FMath::DegreesToRadians(Deg)));
	}

	/**
	 * Made-up ground: a near plane and a far plane either side of a straight
	 * edge through EdgeAt with normal Normal (near to far), each given by its
	 * height at EdgeAt and its gradient; optionally a second edge FarDepth past
	 * the first, beyond which a third plane; boxes standing on it; trace noise.
	 * Custom, when set, replaces all of it.
	 */
	struct FTestGround
	{
		FVector2D EdgeAt = FVector2D(180.0, 0.0);
		FVector2D Normal = FVector2D(1.0, 0.0);
		double	  NearZ = 0.0;
		FVector2D NearG = FVector2D::ZeroVector;
		double	  FarZ = 15.0;
		FVector2D FarG = FVector2D::ZeroVector;
		double	  FarDepth = 1e9;
		double	  BeyondZ = 0.0; // at the second edge, on the centreline's crossing of it
		FVector2D BeyondG = FVector2D::ZeroVector;
		/** Boxes standing on the ground: xy extent, height over the ground. */
		struct FBump
		{
			FVector2D Lo, Hi;
			double	  Height;
		};
		TArray<FBump>							   Bumps;
		double									   NoiseCm = 0.0;
		mutable FRandomStream					   Noise{ 7 };
		TFunction<bool(const FVector2D&, double&)> Custom;

		double D(const FVector2D& P) const { return FVector2D::DotProduct(P - EdgeAt, Normal); }
		double NearAt(const FVector2D& P) const { return NearZ + FVector2D::DotProduct(NearG, P - EdgeAt); }
		double FarAt(const FVector2D& P) const { return FarZ + FVector2D::DotProduct(FarG, P - EdgeAt); }

		bool SurfaceAt(const FVector2D& P, double& Z) const
		{
			if (Custom)
			{
				return Custom(P, Z);
			}
			const double Dist = D(P);
			if (Dist < 0.0)
			{
				Z = NearAt(P);
			}
			else if (Dist < FarDepth)
			{
				Z = FarAt(P);
			}
			else
			{
				Z = BeyondZ + FVector2D::DotProduct(BeyondG, P - (EdgeAt + Normal * FarDepth));
			}
			for (const FBump& B : Bumps)
			{
				if (P.X >= B.Lo.X && P.X <= B.Hi.X && P.Y >= B.Lo.Y && P.Y <= B.Hi.Y)
				{
					Z += B.Height;
				}
			}
			if (NoiseCm > 0.0)
			{
				const double U1 = FMath::Max(1e-12, static_cast<double>(Noise.GetFraction()));
				const double U2 = Noise.GetFraction();
				Z += NoiseCm * FMath::Sqrt(-2.0 * FMath::Loge(U1)) * FMath::Cos(2.0 * PI * U2);
			}
			return true;
		}
	};

	/** A made-up six-wheeled robot: corners at +-45 cm fore/aft and +-25 cm
	 *  across, a middle pair at 0 and +-27; 10 cm wheels, 6 cm tyres. Its
	 *  tyre edges reach 30 cm either side. */
	constexpr double TestHalfWidth = 30.0;

	FTerrainFitInput StandRobot(const FTestGround& G, const FVector2D& Origin, double HeadingDeg)
	{
		FTerrainFitInput In;
		In.Origin = Origin;
		In.Heading = Dir(HeadingDeg);
		const FVector2D Side(-In.Heading.Y, In.Heading.X);
		struct FSpot
		{
			const TCHAR* Name;
			double		 X, Y;
		};
		const FSpot Spots[] = { { TEXT("front_left"), 45.0, -25.0 }, { TEXT("front_right"), 45.0, 25.0 },
			{ TEXT("middle_left"), 0.0, -27.0 }, { TEXT("middle_right"), 0.0, 27.0 }, { TEXT("rear_left"), -45.0, -25.0 },
			{ TEXT("rear_right"), -45.0, 25.0 } };
		for (const FSpot& S : Spots)
		{
			FTerrainWheel W;
			W.Name = S.Name;
			W.RadiusCm = 10.0f;
			W.TyreWidthCm = 6.0f;
			W.ContactWidthCm = 6.0f;
			W.bFront = S.X > 0.0;
			const FVector2D P = Origin + In.Heading * S.X + Side * S.Y;
			// Standing on whatever is under it: the near plane unless a test
			// puts it elsewhere; on a custom ground, its surface there and its
			// slope by finite differences.
			double	  Gz = G.D(P) < 0.0 ? G.NearAt(P) : G.FarAt(P);
			FVector2D Grad = G.D(P) < 0.0 ? G.NearG : G.FarG;
			if (G.Custom)
			{
				double Z0 = 0.0, Xp = 0.0, Xm = 0.0, Yp = 0.0, Ym = 0.0;
				G.Custom(P, Z0);
				G.Custom(P + FVector2D(0.5, 0.0), Xp);
				G.Custom(P - FVector2D(0.5, 0.0), Xm);
				G.Custom(P + FVector2D(0.0, 0.5), Yp);
				G.Custom(P - FVector2D(0.0, 0.5), Ym);
				Gz = Z0;
				Grad = FVector2D(Xp - Xm, Yp - Ym);
			}
			W.Centre = FVector(P.X, P.Y, Gz + RammsCurb::WheelCentreHeight(W.RadiusCm, W.ContactWidthCm, Grad, In.Heading));
			In.Wheels.Add(W);
		}
		return In;
	}

	bool Fit(const FTestGround& G, const FTerrainFitInput& In, FRammsCurbTerrain& Out, FString& Why, FTerrainFitReport* Report = nullptr)
	{
		return RammsCurb::FitTerrain(
			In, [&G](const FVector2D& P, double& Z) { return G.SurfaceAt(P, Z); }, Out, Why, Report);
	}

	double GradientErrorDeg(const FVector2D& A, const FVector2D& B)
	{
		return FMath::RadiansToDegrees(FMath::Atan((A - B).Size()));
	}

	/** What a fit of G seen from the robot should find: the approach angle,
	 *  the step at the centreline and at the footprint band's ends. */
	struct FTruth
	{
		double Alpha;
		double Step;
		double StepLo, StepHi;
	};

	FTruth TruthOf(const FTestGround& G, const FVector2D& Origin, double HeadingDeg)
	{
		const FVector2D H = Dir(HeadingDeg);
		FTruth			T;
		T.Alpha = RammsCurb::ApproachAngleDeg(H, G.Normal);
		const double	NH = FVector2D::DotProduct(G.Normal, H);
		const FVector2D E0 = Origin + H * (FVector2D::DotProduct(G.Normal, G.EdgeAt - Origin) / NH);
		const FVector2D E(-G.Normal.Y, G.Normal.X);
		auto			StepAt = [&](double U) { return G.FarAt(E0 + E * U) - G.NearAt(E0 + E * U); };
		const double	Band = TestHalfWidth / FMath::Cos(FMath::DegreesToRadians(T.Alpha)) + RammsCurb::FootprintBandMarginCm;
		T.Step = StepAt(0.0);
		T.StepLo = FMath::Min(StepAt(-Band), StepAt(Band));
		T.StepHi = FMath::Max(StepAt(-Band), StepAt(Band));
		return T;
	}

	/** Fit G as seen from a robot at Origin heading HeadingDeg, and check it
	 *  against the analytic truth within the plan's limits: the angle within
	 *  0.1 deg, the step and its range within 0.2 cm, both gradients within
	 *  0.2 deg. */
	bool CheckFit(FAutomationTestBase& Test, const FString& Label, const FTestGround& G, const FVector2D& Origin = FVector2D::ZeroVector,
		double HeadingDeg = 0.0, FRammsCurbTerrain* OutTerrain = nullptr)
	{
		const FTerrainFitInput In = StandRobot(G, Origin, HeadingDeg);
		FRammsCurbTerrain	   T;
		FString				   Why;
		const bool			   bFound = Fit(G, In, T, Why);
		if (!Test.TestTrue(FString::Printf(TEXT("%s: finds the step (%s)"), *Label, *Why), bFound))
		{
			return false;
		}
		const FTruth Truth = TruthOf(G, Origin, HeadingDeg);
		bool		 bOk = true;
		bOk &= Test.TestEqual(FString::Printf(TEXT("%s: approach angle (deg)"), *Label), static_cast<double>(T.ApproachAngleDeg), Truth.Alpha, 0.1);
		bOk &= Test.TestEqual(FString::Printf(TEXT("%s: step at the centreline (cm)"), *Label), static_cast<double>(T.StepHeightCm), Truth.Step, 0.2);
		bOk &= Test.TestEqual(FString::Printf(TEXT("%s: least step across the footprint (cm)"), *Label), static_cast<double>(T.StepHeightRangeCm.X), Truth.StepLo, 0.2);
		bOk &= Test.TestEqual(FString::Printf(TEXT("%s: greatest step across the footprint (cm)"), *Label), static_cast<double>(T.StepHeightRangeCm.Y), Truth.StepHi, 0.2);
		bOk &= Test.TestTrue(FString::Printf(TEXT("%s: near gradient within 0.2 deg (%.3f)"), *Label, GradientErrorDeg(T.Near.Gradient, G.NearG)),
			GradientErrorDeg(T.Near.Gradient, G.NearG) <= 0.2);
		bOk &= Test.TestTrue(FString::Printf(TEXT("%s: far gradient within 0.2 deg (%.3f)"), *Label, GradientErrorDeg(T.Far.Gradient, G.FarG)),
			GradientErrorDeg(T.Far.Gradient, G.FarG) <= 0.2);
		bOk &= Test.TestTrue(FString::Printf(TEXT("%s: the edge normal points the robot's way"), *Label),
			FVector2D::DotProduct(T.EdgeNormal, Dir(HeadingDeg)) > 0.0);
		bOk &= Test.TestTrue(FString::Printf(TEXT("%s: the edge line passes within 0.2 cm of the true edge (%.3f)"), *Label,
								 FMath::Abs(T.DistancePastEdge(G.EdgeAt))),
			FMath::Abs(T.DistancePastEdge(G.EdgeAt)) <= 0.2);
		if (OutTerrain)
		{
			*OutTerrain = T;
		}
		return bOk;
	}

	/** Expect a refusal whose reason holds Substring. */
	bool CheckRefusal(FAutomationTestBase& Test, const FString& Label, const FTestGround& G, const TCHAR* Substring,
		const FTerrainFitInput& In)
	{
		FRammsCurbTerrain T;
		FString			  Why;
		const bool		  bFound = Fit(G, In, T, Why);
		Test.AddInfo(FString::Printf(TEXT("%s: %s"), *Label, *Why));
		return Test.TestFalse(FString::Printf(TEXT("%s: refused"), *Label), bFound)
			&& Test.TestTrue(FString::Printf(TEXT("%s: the reason says '%s' (%s)"), *Label, Substring, *Why), Why.Contains(Substring))
			&& Test.TestFalse(FString::Printf(TEXT("%s: no terrain on a refusal"), *Label), T.bValid);
	}

	bool CheckRefusal(FAutomationTestBase& Test, const FString& Label, const FTestGround& G, const TCHAR* Substring)
	{
		return CheckRefusal(Test, Label, G, Substring, StandRobot(G, FVector2D::ZeroVector, 0.0));
	}

	/** An edge at Deg to the robot's heading through the usual point. */
	FTestGround Angled(double Deg)
	{
		FTestGround G;
		G.Normal = Dir(Deg);
		return G;
	}

	/** A plane gradient of Deg along the unit direction D. */
	FVector2D Slope(double Deg, const FVector2D& D)
	{
		return D * Tand(Deg);
	}
} // namespace

// --- fits ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainFitSquare, "Ramms.Curb.Terrain.Fit.Square", TestFlags)
bool FRammsCurbTerrainFitSquare::RunTest(const FString& Parameters)
{
	FTestGround		  G;
	FRammsCurbTerrain T;
	FTerrainFitReport Report;
	CheckFit(*this, TEXT("square 15 cm climb"), G, FVector2D::ZeroVector, 0.0, &T);
	FString Why;
	Fit(G, StandRobot(G, FVector2D::ZeroVector, 0.0), T, Why, &Report);
	AddInfo(Why);
	TestEqual(TEXT("the edge is 135 cm ahead of the front wheels"), Report.EdgeDistanceCm, 135.0f, 0.05f);
	TestEqual(TEXT("seven lanes"), Report.Lanes.Num(), 7);
	TestEqual(TEXT("no lane dropped"), Report.DroppedLane, static_cast<int32>(INDEX_NONE));
	TestFalse(TEXT("no second edge seen"), T.bFarDepthObserved);
	// The lanes end DetectRange (350) past the front wheels: 215 cm past the edge.
	TestEqual(TEXT("far side seen to the end of view"), T.FarDepthCm, 215.0f, 2.0f);
	TestTrue(TEXT("the edge point is on the centreline, at the near surface"), T.EdgePoint.Equals(FVector(180.0, 0.0, 0.0), 0.05));
	TestTrue(TEXT("planes fit exactly"), T.Near.FitRmsCm < 1e-3f && T.Far.FitRmsCm < 1e-3f);
	FTestGround Down;
	Down.NearZ = 15.0;
	Down.FarZ = 0.0;
	CheckFit(*this, TEXT("square 15 cm descent"), Down);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainFitYaw15, "Ramms.Curb.Terrain.Fit.Yaw15", TestFlags)
bool FRammsCurbTerrainFitYaw15::RunTest(const FString& Parameters)
{
	CheckFit(*this, TEXT("15 deg"), Angled(15.0));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainFitYaw30, "Ramms.Curb.Terrain.Fit.Yaw30", TestFlags)
bool FRammsCurbTerrainFitYaw30::RunTest(const FString& Parameters)
{
	FRammsCurbTerrain T;
	if (CheckFit(*this, TEXT("30 deg"), Angled(30.0), FVector2D::ZeroVector, 0.0, &T))
	{
		TestTrue(TEXT("positive: the robot's +Y wheels meet the edge first"), T.ApproachAngleDeg > 29.0f);
	}
	// The same edge, the robot turned 20 deg and placed off to one side: the
	// angle is the edge's to the heading, not to the world's X.
	FTestGround G = Angled(50.0);
	CheckFit(*this, TEXT("30 deg to a robot heading 20 deg"), G, FVector2D(20.0, -40.0), 20.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainFitYawNeg30, "Ramms.Curb.Terrain.Fit.YawNeg30", TestFlags)
bool FRammsCurbTerrainFitYawNeg30::RunTest(const FString& Parameters)
{
	FRammsCurbTerrain T;
	if (CheckFit(*this, TEXT("-30 deg"), Angled(-30.0), FVector2D::ZeroVector, 0.0, &T))
	{
		TestTrue(TEXT("negative: the -Y wheels meet the edge first"), T.ApproachAngleDeg < -29.0f);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainFitNearSlope6, "Ramms.Curb.Terrain.Fit.NearSlope6", TestFlags)
bool FRammsCurbTerrainFitNearSlope6::RunTest(const FString& Parameters)
{
	// The road the robot stands on sloping 6 deg towards the curb, away from
	// it, and either way along it -- squarely and at 30 deg.
	for (const double Approach : { 0.0, 30.0 })
	{
		for (const double Toward : { 0.0, 180.0, 90.0, -90.0 })
		{
			FTestGround G = Angled(Approach);
			G.NearG = Slope(6.0, Dir(Approach + Toward));
			CheckFit(*this, FString::Printf(TEXT("near 6 deg towards %+.0f deg, approach %.0f"), Approach + Toward, Approach), G);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainFitFarSlope6, "Ramms.Curb.Terrain.Fit.FarSlope6", TestFlags)
bool FRammsCurbTerrainFitFarSlope6::RunTest(const FString& Parameters)
{
	for (const double Approach : { 0.0, -30.0 })
	{
		for (const double Toward : { 0.0, 180.0, 90.0, -90.0 })
		{
			FTestGround G = Angled(Approach);
			G.FarG = Slope(6.0, Dir(Approach + Toward));
			CheckFit(*this, FString::Printf(TEXT("far 6 deg towards %+.0f deg, approach %.0f"), Approach + Toward, Approach), G);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainFitGrade6, "Ramms.Curb.Terrain.Fit.Grade6", TestFlags)
bool FRammsCurbTerrainFitGrade6::RunTest(const FString& Parameters)
{
	// A street graded 6 deg along the curb: both sides the same grade, so the
	// step is the same all along, and a graded road is no step.
	FTestGround G;
	G.NearG = G.FarG = Slope(6.0, FVector2D(0.0, 1.0));
	CheckFit(*this, TEXT("6 deg grade"), G);
	FTestGround Flat;
	Flat.FarZ = 0.0;
	Flat.NearG = Flat.FarG = Slope(6.0, FVector2D(0.0, 1.0));
	FRammsCurbTerrain T;
	FString			  Why;
	TestFalse(TEXT("a graded road with no curb is no step"), Fit(Flat, StandRobot(Flat, FVector2D::ZeroVector, 0.0), T, Why));
	TestTrue(FString::Printf(TEXT("... and says so (%s)"), *Why), Why.Contains(TEXT("no step")));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainFitTwist6, "Ramms.Curb.Terrain.Fit.Twist6", TestFlags)
bool FRammsCurbTerrainFitTwist6::RunTest(const FString& Parameters)
{
	// The top twisted 6 deg along the edge against a level road: the step
	// grows along the edge, and its range across the footprint is what counts.
	FTestGround G;
	G.FarG = Slope(6.0, FVector2D(0.0, 1.0));
	FRammsCurbTerrain T;
	if (CheckFit(*this, TEXT("6 deg twist"), G, FVector2D::ZeroVector, 0.0, &T))
	{
		TestTrue(TEXT("the step's range spans the twist"), T.StepHeightRangeCm.Y - T.StepHeightRangeCm.X > 8.0f);
	}
	G.Normal = Dir(-30.0);
	CheckFit(*this, TEXT("6 deg twist at -30 deg"), G);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainFitGutter, "Ramms.Curb.Terrain.Fit.Gutter", TestFlags)
bool FRammsCurbTerrainFitGutter::RunTest(const FString& Parameters)
{
	// The road falling 6 deg into the curb; the far road rising 6 deg out of
	// the drop on the way down.
	FTestGround Up;
	Up.NearG = Slope(6.0, FVector2D(-1.0, 0.0));
	CheckFit(*this, TEXT("gutter, climbing"), Up);
	FTestGround Down;
	Down.NearZ = 15.0;
	Down.FarZ = 0.0;
	Down.FarG = Slope(6.0, FVector2D(1.0, 0.0));
	CheckFit(*this, TEXT("gutter, descending"), Down);
	FTestGround Angle = Up;
	Angle.Normal = Dir(30.0);
	Angle.NearG = Slope(6.0, -Angle.Normal);
	CheckFit(*this, TEXT("gutter at 30 deg"), Angle);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainFitNoise2mm, "Ramms.Curb.Terrain.Fit.Noise2mm", TestFlags)
bool FRammsCurbTerrainFitNoise2mm::RunTest(const FString& Parameters)
{
	// 2 mm RMS on every trace: the planes average it out, the edge's bisection
	// is against half the step, and no spike is taken for an edge.
	FTestGround G = Angled(30.0);
	G.NearG = Slope(4.0, Dir(90.0));
	G.NoiseCm = 0.2;
	FRammsCurbTerrain T;
	if (CheckFit(*this, TEXT("2 mm noise, 30 deg, near graded 4 deg"), G, FVector2D::ZeroVector, 0.0, &T))
	{
		TestTrue(FString::Printf(TEXT("the near fit's RMS is the noise's (%.3f)"), T.Near.FitRmsCm), T.Near.FitRmsCm > 0.12f && T.Near.FitRmsCm < 0.3f);
	}
	FTestGround Square;
	Square.NoiseCm = 0.2;
	Square.Noise.Initialize(11);
	CheckFit(*this, TEXT("2 mm noise, square"), Square);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainFitOutlierLane, "Ramms.Curb.Terrain.Fit.OutlierLane", TestFlags)
bool FRammsCurbTerrainFitOutlierLane::RunTest(const FString& Parameters)
{
	// Something 3 cm tall on one lane 20 cm short of the curb: that lane's
	// first jump is it, its edge point is far off the line, and the lane is
	// dropped -- the edge, the planes and the far depth come from the rest.
	FTestGround G;
	G.Bumps.Add({ FVector2D(156.0, 13.0), FVector2D(164.0, 17.0), 3.0 });
	FRammsCurbTerrain T;
	FTerrainFitReport Report;
	CheckFit(*this, TEXT("an object on the +Y middle lane"), G, FVector2D::ZeroVector, 0.0, &T);
	FString Why;
	Fit(G, StandRobot(G, FVector2D::ZeroVector, 0.0), T, Why, &Report);
	if (TestTrue(TEXT("one lane dropped"), Report.DroppedLane != INDEX_NONE))
	{
		TestEqual(TEXT("... the one with the object"), Report.Lanes[Report.DroppedLane].OffsetCm, 15.0f, 0.01f);
	}
	TestEqual(TEXT("the far depth is the view's, not the object's"), T.FarDepthCm, 215.0f, 2.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainFitLeaningFace, "Ramms.Curb.Terrain.Fit.LeaningFace", TestFlags)
bool FRammsCurbTerrainFitLeaningFace::RunTest(const FString& Parameters)
{
	// A face that leans back into view, 15 cm over 1.6 cm, both ways, the
	// robot a little off square so that every lane's samples fall at another
	// place on it: the edge is where the face is halfway up, on every lane,
	// so the line through them keeps the angle.
	for (const double Step : { 15.0, -15.0 })
	{
		for (const double Deg : { 0.4, -7.0, 23.0 })
		{
			const FVector2D N = Dir(Deg);
			FTestGround		G;
			G.Normal = N;
			G.FarZ = Step;
			G.Custom = [N, Step](const FVector2D& P, double& Z) {
				const double D = FVector2D::DotProduct(P - FVector2D(180.0, 0.0), N);
				Z = Step * FMath::Clamp(D / 1.6, 0.0, 1.0);
				return true;
			};
			const FTerrainFitInput In = StandRobot(G, FVector2D::ZeroVector, 0.0);
			FRammsCurbTerrain	   T;
			FString				   Why;
			const bool			   bFound = Fit(G, In, T, Why);
			const FString		   Label = FString::Printf(TEXT("step %+.0f at %+.1f deg"), Step, Deg);
			if (!TestTrue(FString::Printf(TEXT("%s: found (%s)"), *Label, *Why), bFound))
			{
				continue;
			}
			TestEqual(FString::Printf(TEXT("%s: the angle"), *Label), static_cast<double>(T.ApproachAngleDeg), Deg, 0.1);
			TestTrue(FString::Printf(TEXT("%s: the edge halfway down the face (%.3f)"), *Label, T.DistancePastEdge(FVector2D(180.0, 0.0) + N * 0.8)),
				FMath::Abs(T.DistancePastEdge(FVector2D(180.0, 0.0) + N * 0.8)) < 0.2);
			TestTrue(FString::Printf(TEXT("%s: edge residual under 0.2 cm (%.3f)"), *Label, T.EdgeFitResidualCm), T.EdgeFitResidualCm < 0.2f);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainFarAfterEdge, "Ramms.Curb.Terrain.FarAfterEdge", TestFlags)
bool FRammsCurbTerrainFarAfterEdge::RunTest(const FString& Parameters)
{
	// A 15 cm curb with a sidewalk 150 cm deep, then road again. Judged
	// against the near plane, every far sample is a 15 cm jump; the far side
	// is judged against its own running fit, so the one second edge is the
	// sidewalk's end -- also on a sidewalk that rises 6 deg away.
	for (const double Rise : { 0.0, 6.0 })
	{
		FTestGround G;
		G.FarG = Slope(Rise, FVector2D(1.0, 0.0));
		G.FarDepth = 150.0;
		G.BeyondZ = 0.0;
		FTerrainFitInput In = StandRobot(G, FVector2D::ZeroVector, 0.0);
		In.FarDepthNeed.WheelbaseCm = 90.0f;
		In.FarDepthNeed.ExtraCm = 40.0f;
		FRammsCurbTerrain T;
		FString			  Why;
		FTerrainFitReport Report;
		const FString	  Label = FString::Printf(TEXT("sidewalk rising %.0f deg"), Rise);
		if (!TestTrue(FString::Printf(TEXT("%s: found (%s)"), *Label, *Why), Fit(G, In, T, Why, &Report)))
		{
			continue;
		}
		TestTrue(FString::Printf(TEXT("%s: the second edge was seen"), *Label), T.bFarDepthObserved);
		TestEqual(FString::Printf(TEXT("%s: the far depth is the sidewalk's"), *Label), T.FarDepthCm, 150.0f, 0.2f);
		for (const RammsCurb::FTerrainLane& L : Report.Lanes)
		{
			if (L.bUsed)
			{
				TestTrue(FString::Printf(TEXT("%s: lane %+.0f: its second edge at the sidewalk's end, not just past the curb (%.2f)"),
							 *Label, L.OffsetCm, L.FarDepthCm),
					L.bSecondEdge && FMath::Abs(L.FarDepthCm - 150.0f) < 0.2f);
			}
		}
		TestEqual(FString::Printf(TEXT("%s: the step"), *Label), T.StepHeightCm, 15.0f, 0.2f);
		TestTrue(FString::Printf(TEXT("%s: the far gradient"), *Label), GradientErrorDeg(T.Far.Gradient, G.FarG) <= 0.2);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainFarDepth, "Ramms.Curb.Terrain.FarDepth", TestFlags)
bool FRammsCurbTerrainFarDepth::RunTest(const FString& Parameters)
{
	// What the far depth allows: enough; enough to start, the rest to be seen
	// on the way (60 cm and more in view); too little in view; seen too
	// shallow. The need grows with the approach angle.
	RammsCurb::FFarDepthNeed Need;
	Need.WheelbaseCm = 90.0f;
	Need.TrackCm = 50.0f;
	Need.ExtraCm = 40.0f;
	TestEqual(TEXT("square on: wheelbase plus extra"), Need.At(0.0f), 130.0f, 1e-3f);
	TestEqual(TEXT("at 30 deg: the footprint's extent across the edge"), Need.At(30.0f), 90.0f * FMath::Cos(PI / 6.0f) + 25.0f + 40.0f, 1e-3f);
	TestEqual(TEXT("either sense"), Need.At(-30.0f), Need.At(30.0f), 1e-4f);

	FTestGround		 G;
	FTerrainFitInput In = StandRobot(G, FVector2D::ZeroVector, 0.0);
	In.FarDepthNeed = Need;
	In.DetectRangeCm = 230.0f; // the view ends 95 cm past the edge
	FRammsCurbTerrain T;
	FString			  Why;
	const bool		  bStarts = Fit(G, In, T, Why);
	if (TestTrue(FString::Printf(TEXT("95 cm in view of a need of 130: starts (%s)"), *Why), bStarts))
	{
		FString Verdict;
		TestTrue(TEXT("... pending"), RammsCurb::FarDepthVerdict(T, Need.At(0.0f), &Verdict) == RammsCurb::EFarDepth::Pending);
		TestEqual(TEXT("... seen 95 cm deep"), T.FarDepthCm, 95.0f, 2.0f);
		TestFalse(TEXT("... to the end of view"), T.bFarDepthObserved);
	}
	In.DetectRangeCm = 180.0f; // 45 cm in view
	CheckRefusal(*this, TEXT("45 cm in view"), G, TEXT("in view"), In);
	In.DetectRangeCm = 350.0f;
	TestTrue(TEXT("all of it in view"), Fit(G, In, T, Why) && RammsCurb::FarDepthVerdict(T, Need.At(0.0f)) == RammsCurb::EFarDepth::Enough);

	FRammsCurbTerrain Seen = T;
	Seen.bFarDepthObserved = true;
	Seen.FarDepthCm = 100.0f;
	FString Verdict;
	TestTrue(TEXT("seen 100 cm deep of 130: too shallow"), RammsCurb::FarDepthVerdict(Seen, 130.0f, &Verdict) == RammsCurb::EFarDepth::TooShallow);
	TestTrue(FString::Printf(TEXT("... saying how deep (%s)"), *Verdict), Verdict.Contains(TEXT("100 cm deep")));
	return true;
}

// --- refusals ------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainRefuseCurvedEdge, "Ramms.Curb.Terrain.Refuse.CurvedEdge", TestFlags)
bool FRammsCurbTerrainRefuseCurvedEdge::RunTest(const FString& Parameters)
{
	// A curb bulging towards the robot on a 100 cm radius.
	FTestGround		G;
	const FVector2D C(280.0, 0.0);
	G.Custom = [C](const FVector2D& P, double& Z) {
		Z = (FVector2D::Distance(P, C) < 100.0 || P.X > C.X) ? 15.0 : 0.0;
		return true;
	};
	CheckRefusal(*this, TEXT("curved edge"), G, TEXT("not straight"));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainRefuseKinkedEdge, "Ramms.Curb.Terrain.Refuse.KinkedEdge", TestFlags)
bool FRammsCurbTerrainRefuseKinkedEdge::RunTest(const FString& Parameters)
{
	// Two straight curbs meeting on the centreline, 20 deg apart.
	FTestGround		G;
	const FVector2D E(180.0, 0.0);
	G.Custom = [E](const FVector2D& P, double& Z) {
		const FVector2D N = P.Y < 0.0 ? FVector2D(1.0, 0.0) : Dir(20.0);
		Z = FVector2D::DotProduct(P - E, N) >= 0.0 ? 15.0 : 0.0;
		return true;
	};
	CheckRefusal(*this, TEXT("kinked edge"), G, TEXT("not straight"));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainRefuseRamp10, "Ramms.Curb.Terrain.Refuse.Ramp10", TestFlags)
bool FRammsCurbTerrainRefuseRamp10::RunTest(const FString& Parameters)
{
	// A 10 deg curb cut up to 15 cm, no face: never a jump, only a bend.
	FTestGround G;
	G.Custom = [](const FVector2D& P, double& Z) {
		Z = FMath::Clamp((P.X - 180.0) * Tand(10.0), 0.0, 15.0);
		return true;
	};
	CheckRefusal(*this, TEXT("10 deg ramp"), G, TEXT("ramp"));
	// A steep one is seen as a jump, but its face is too wide.
	FTestGround Steep;
	Steep.Custom = [](const FVector2D& P, double& Z) {
		Z = FMath::Clamp((P.X - 180.0) * Tand(60.0), 0.0, 15.0);
		return true;
	};
	CheckRefusal(*this, TEXT("60 deg ramp"), Steep, TEXT("a ramp, not a step"));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainRefuseCrown2pc, "Ramms.Curb.Terrain.Refuse.Crown2pc", TestFlags)
bool FRammsCurbTerrainRefuseCrown2pc::RunTest(const FString& Parameters)
{
	// Crossing a road crowned 2 %: up to the crown 100 cm ahead, then down.
	FTestGround G;
	G.Custom = [](const FVector2D& P, double& Z) {
		Z = -0.02 * FMath::Abs(P.X - 100.0);
		return true;
	};
	CheckRefusal(*this, TEXT("2 % crown"), G, TEXT("changes slope"));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainRefuseStepTooSmallAtOneTrack, "Ramms.Curb.Terrain.Refuse.StepTooSmallAtOneTrack", TestFlags)
bool FRammsCurbTerrainRefuseStepTooSmallAtOneTrack::RunTest(const FString& Parameters)
{
	// The top twisted so the curb is 1.5 cm at the -Y tyres and 6 cm at the +Y.
	FTestGround G;
	G.FarZ = 3.75;
	G.FarG = FVector2D(0.0, 4.5 / 60.0);
	CheckRefusal(*this, TEXT("1.5 cm at one track"), G, TEXT("under MinStepHeight"));
	// Just inside the tracks, but under 2 cm within the band beyond them.
	FTestGround Band;
	Band.FarZ = 5.0;
	Band.FarG = FVector2D(0.0, 2.9 / 30.0);
	CheckRefusal(*this, TEXT("2.1 cm at one track, under 2 within the band"), Band, TEXT("under MinStepHeight"));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainRefuseStepSignChange, "Ramms.Curb.Terrain.Refuse.StepSignChange", TestFlags)
bool FRammsCurbTerrainRefuseStepSignChange::RunTest(const FString& Parameters)
{
	// Up 6 cm at one track, down 6 cm at the other.
	FTestGround G;
	G.FarZ = 0.0;
	G.FarG = FVector2D(0.0, 0.2);
	CheckRefusal(*this, TEXT("sign change"), G, TEXT("changes sign"));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainRefuseTooSteep, "Ramms.Curb.Terrain.Refuse.TooSteep", TestFlags)
bool FRammsCurbTerrainRefuseTooSteep::RunTest(const FString& Parameters)
{
	FTestGround G;
	G.NearG = Slope(10.0, FVector2D(1.0, 0.0));
	CheckRefusal(*this, TEXT("10 deg road"), G, TEXT("MaxSurfaceSlopeDeg"));
	FTestGround Far;
	Far.FarG = Slope(9.0, FVector2D(0.0, 1.0));
	Far.FarZ = 12.0;
	CheckRefusal(*this, TEXT("9 deg sidewalk"), Far, TEXT("MaxSurfaceSlopeDeg"));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainRefuseTooOblique40, "Ramms.Curb.Terrain.Refuse.TooOblique40", TestFlags)
bool FRammsCurbTerrainRefuseTooOblique40::RunTest(const FString& Parameters)
{
	CheckRefusal(*this, TEXT("40 deg"), Angled(40.0), TEXT("MaxApproachDeg"));
	CheckRefusal(*this, TEXT("-40 deg"), Angled(-40.0), TEXT("MaxApproachDeg"));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainRefuseNarrowSidewalk, "Ramms.Curb.Terrain.Refuse.NarrowSidewalk", TestFlags)
bool FRammsCurbTerrainRefuseNarrowSidewalk::RunTest(const FString& Parameters)
{
	// An 80 cm sidewalk for a robot that needs 130.
	FTestGround G;
	G.FarDepth = 80.0;
	FTerrainFitInput In = StandRobot(G, FVector2D::ZeroVector, 0.0);
	In.FarDepthNeed.WheelbaseCm = 90.0f;
	In.FarDepthNeed.ExtraCm = 40.0f;
	CheckRefusal(*this, TEXT("80 cm sidewalk"), G, TEXT("80 cm deep"), In);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainRefuseStandingAcross, "Ramms.Curb.Terrain.Refuse.StandingAcross", TestFlags)
bool FRammsCurbTerrainRefuseStandingAcross::RunTest(const FString& Parameters)
{
	// The front wheels already up on the curb.
	FTestGround G;
	G.EdgeAt = FVector2D(30.0, 0.0);
	CheckRefusal(*this, TEXT("front wheels on top"), G, TEXT("standing across a step"));
	// A rear wheel down in a hole.
	FTestGround		 Hole;
	FTerrainFitInput In = StandRobot(Hole, FVector2D::ZeroVector, 0.0);
	In.Wheels[4].Centre.Z -= 4.0;
	CheckRefusal(*this, TEXT("a wheel 4 cm low"), Hole, TEXT("below the surface"), In);
	// Not checked on a second look with the legs moving.
	In.bCheckWheels = false;
	FRammsCurbTerrain T;
	FString			  Why;
	TestTrue(FString::Printf(TEXT("a second look does not check the wheels (%s)"), *Why), Fit(Hole, In, T, Why));
	return true;
}

// --- the legacy profile and the approach's second looks -------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainLegacyProfile, "Ramms.Curb.Terrain.LegacyProfile", TestFlags)
bool FRammsCurbTerrainLegacyProfile::RunTest(const FString& Parameters)
{
	// A 10 deg edge: the profile's distance is the nearer front wheel's along
	// the heading, its skew the front track's.
	FTestGround			   G = Angled(10.0);
	const FTerrainFitInput In = StandRobot(G, FVector2D::ZeroVector, 0.0);
	FRammsCurbTerrain	   T;
	FString				   Why;
	if (!TestTrue(TEXT("found"), Fit(G, In, T, Why)))
	{
		return false;
	}
	FRammsCurbProfile P;
	RammsCurb::FillLegacyProfile(T, In.Wheels, In.Origin, In.Heading, P);
	// The +Y front wheel (y = +25) meets it first: the edge crosses y = 25 at
	// x = 180 - 25 tan 10.
	TestEqual(TEXT("edge distance from the nearer front wheel"), P.EdgeDistanceCm, static_cast<float>(180.0 - 25.0 * Tand(10.0) - 45.0), 0.05f);
	TestEqual(TEXT("skew over the 50 cm front track"), P.EdgeSkewCm, static_cast<float>(50.0 * Tand(10.0)), 0.05f);
	TestEqual(TEXT("the step"), P.StepHeightCm, 15.0f, 0.05f);
	TestEqual(TEXT("the angle"), P.ApproachAngleDeg, 10.0f, 0.05f);
	TestTrue(TEXT("the terrain rides along"), P.Terrain.bValid);

	const FRammsCurbTerrain L = RammsCurb::LevelTerrain(-200.0, -15.0, FVector2D(100.0, 50.0), FVector2D(0.0, 1.0));
	TestTrue(TEXT("a level terrain: valid"), L.bValid);
	TestTrue(TEXT("... its edge square to the heading"), L.EdgeNormal.Equals(FVector2D(0.0, 1.0), 1e-9));
	TestEqual(TEXT("... the step"), L.StepAt(37.0), -15.0, 1e-6);
	TestEqual(TEXT("... its far side 15 cm down"), L.SurfaceAt(FVector2D(100.0, 51.0)), -215.0, 1e-6);
	TestEqual(TEXT("... its near side level at the ground"), L.SurfaceAt(FVector2D(300.0, 49.0)), -200.0, 1e-6);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbTerrainRedetect, "Ramms.Curb.Terrain.Redetect", TestFlags)
bool FRammsCurbTerrainRedetect::RunTest(const FString& Parameters)
{
	// Looking again: how far a second look moved, and the blend of the two.
	FTestGround		  G;
	FRammsCurbTerrain A;
	FString			  Why;
	if (!TestTrue(TEXT("found"), Fit(G, StandRobot(G, FVector2D::ZeroVector, 0.0), A, Why)))
	{
		return false;
	}
	const RammsCurb::FTerrainChange Same = RammsCurb::CompareTerrain(A, A);
	TestTrue(TEXT("no change from itself"), Same.MaxCm() < 1e-6 && Same.MaxDeg() < 1e-6);

	FTestGround Moved;
	Moved.EdgeAt = FVector2D(181.0, 0.0);
	Moved.FarZ = 16.0;
	Moved.Normal = Dir(1.0);
	FRammsCurbTerrain B;
	if (!TestTrue(TEXT("found again"), Fit(Moved, StandRobot(Moved, FVector2D(10.0, 0.0), 0.0), B, Why)))
	{
		return false;
	}
	const RammsCurb::FTerrainChange C = RammsCurb::CompareTerrain(A, B);
	TestEqual(TEXT("the edge moved 1 cm"), C.EdgeCm, 1.0, 0.05);
	TestEqual(TEXT("... and turned 1 deg"), C.EdgeDeg, 1.0, 0.05);
	TestEqual(TEXT("the far side rose 1 cm"), C.FarCm, 1.0, 0.05);
	TestTrue(TEXT("the near side did not move"), C.NearCm < 0.05 && C.NearDeg < 0.05);

	const FRammsCurbTerrain Half = RammsCurb::BlendTerrain(A, B, 0.5, FVector2D(1.0, 0.0));
	TestEqual(TEXT("halfway: the step"), Half.StepHeightCm, 15.5f, 0.05f);
	TestEqual(TEXT("halfway: the angle"), Half.ApproachAngleDeg, 0.5f, 0.05f);
	TestEqual(TEXT("halfway: the edge"), FMath::Abs(Half.DistancePastEdge(FVector2D(180.5, 0.0))), 0.0, 0.05);

	// The far depth: a second edge seen beats a view; the deeper of two views.
	FRammsCurbTerrain V1 = A, V2 = A;
	V1.FarDepthCm = 100.0f;
	V2.FarDepthCm = 120.0f;
	TestEqual(TEXT("the deeper view"), RammsCurb::BlendTerrain(V1, V2, 0.5, FVector2D(1.0, 0.0)).FarDepthCm, 120.0f);
	V1.bFarDepthObserved = true;
	const FRammsCurbTerrain S = RammsCurb::BlendTerrain(V1, V2, 0.5, FVector2D(1.0, 0.0));
	TestTrue(TEXT("a seen edge stays seen"), S.bFarDepthObserved && S.FarDepthCm == 100.0f);
	return true;
}

// --- contact geometry ----------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbContactRhoOnSlope, "Ramms.Curb.Contact.RhoOnSlope", TestFlags)
bool FRammsCurbContactRhoOnSlope::RunTest(const FString& Parameters)
{
	// Against brute force: a cylinder of radius R and contact width W, axle
	// level, resting on a plane. Its centre stands over the plane point below
	// it by the most any rim point's height over the plane falls short.
	const double	R = 10.0;
	const double	W = 6.0;
	const FVector2D F(1.0, 0.0);
	const FVector2D A(0.0, 1.0);
	for (const FVector2D G : { Slope(6.0, F), Slope(6.0, A), Slope(6.0, -A), Slope(4.24, F) + Slope(4.24, A), Slope(8.0, Dir(-150.0)) })
	{
		double Brute = -TNumericLimits<double>::Max();
		for (int32 i = 0; i < 3600; ++i)
		{
			const double Theta = 2.0 * PI * i / 3600.0;
			for (int32 j = 0; j <= 20; ++j)
			{
				const double V = -0.5 * W + W * j / 20.0;
				// The plane under the rim point (cos, v), less the rim point's height.
				Brute = FMath::Max(Brute, G.X * R * FMath::Cos(Theta) + G.Y * V - R * FMath::Sin(Theta));
			}
		}
		const double Rho = RammsCurb::WheelCentreHeight(R, W, G, F);
		TestEqual(FString::Printf(TEXT("rho on (%.3f, %.3f)"), G.X, G.Y), Rho, Brute, 2e-3);
	}
	// The closed forms: 6 deg along the roll adds R (sqrt(1 +
	// tan^2) - 1); 6 deg across adds half the contact width times tan 6.
	TestEqual(TEXT("6 deg along"), RammsCurb::WheelCentreHeight(R, W, Slope(6.0, F), F) - R, R * (1.0 / FMath::Cos(FMath::DegreesToRadians(6.0)) - 1.0), 1e-9);
	TestEqual(TEXT("6 deg across"), RammsCurb::WheelCentreHeight(R, W, Slope(6.0, A), F) - R, 0.5 * W * Tand(6.0), 1e-9);
	TestEqual(TEXT("a narrower contact lifts it less"), RammsCurb::WheelCentreHeight(R, 0.5 * W, Slope(6.0, A), F) - R, 0.25 * W * Tand(6.0), 1e-9);
	TestEqual(TEXT("level ground"), RammsCurb::WheelCentreHeight(R, W, FVector2D::ZeroVector, F), R, 1e-12);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbContactWidthDefault, "Ramms.Curb.Contact.ContactWidthDefault", TestFlags)
bool FRammsCurbContactWidthDefault::RunTest(const FString& Parameters)
{
	// A leg that sets only its tyre's width sits flat across all of it; one
	// that also sets a narrower contact (rollers) uses that.
	const FRammsCurbLeg Leg;
	TestEqual(TEXT("a leg's contact width is unset by default"), Leg.ContactWidthCm, 0.0f);
	TestEqual(TEXT("unset, it is the tyre's width"), RammsCurb::ContactWidth(Leg.TyreWidthCm, Leg.ContactWidthCm), Leg.TyreWidthCm);
	TestEqual(TEXT("a wider tyre, unset: its own width"), RammsCurb::ContactWidth(14.0f, 0.0f), 14.0f);
	TestEqual(TEXT("set: as set"), RammsCurb::ContactWidth(14.0f, 4.0f), 4.0f);
	// And so on a cross slope the wider tyre lifts its centre by half its own
	// width times the slope, not half the old fixed default's.
	const FVector2D F(1.0, 0.0);
	TestEqual(TEXT("rho across a 6 deg slope from a 14 cm tyre's width"),
		RammsCurb::WheelCentreHeight(10.0, RammsCurb::ContactWidth(14.0f, 0.0f), Slope(6.0, FVector2D(0.0, 1.0)), F) - 10.0,
		7.0 * Tand(6.0), 1e-6);
	return true;
}

namespace
{
	/** A flat curb: Near at 0, Far at Step, the edge at x = 0 square to +X. */
	FRammsCurbTerrain FlatCurb(double Step, double EdgeDeg = 0.0)
	{
		FRammsCurbTerrain T = RammsCurb::LevelTerrain(0.0, Step, FVector2D::ZeroVector, Dir(EdgeDeg));
		return T;
	}

	/** The rim's height over its bottom at U along the roll, for |U| <= R. */
	double Rim(double R, double U)
	{
		return FMath::Sqrt(FMath::Max(0.0, R * R - U * U));
	}

	/** Brute force: the most of surface + rim over a dense footprint grid. */
	double BruteClearance(const FRammsCurbTerrain& T, const FVector2D& C, const FVector2D& F, double R, double Width, double Dilation)
	{
		const bool		bClimb = T.StepHeightCm >= 0.0f;
		const FVector2D A(-F.Y, F.X);
		double			Best = -TNumericLimits<double>::Max();
		for (int32 i = 0; i <= 2000; ++i)
		{
			const double U = -R + 2.0 * R * i / 2000.0;
			for (int32 j = 0; j <= 60; ++j)
			{
				const double	V = -0.5 * Width + Width * j / 60.0;
				const FVector2D P = C + F * U + A * V;
				const double	D = T.DistancePastEdge(P);
				const bool		bHigh = bClimb ? D >= -Dilation : D < Dilation;
				const double	S = bHigh == bClimb ? T.Far.HeightAt(P) : T.Near.HeightAt(P);
				Best = FMath::Max(Best, S + Rim(R, U));
			}
		}
		return Best;
	}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbContactClearanceProfileClimb, "Ramms.Curb.Contact.ClearanceProfileClimb", TestFlags)
bool FRammsCurbContactClearanceProfileClimb::RunTest(const FString& Parameters)
{
	// Square on: with the curb top taken Dilation out over its face, the
	// lip is at u* = -Dilation - d along the wheel from its centre at d. The
	// rim clears the top at sqrt(R^2 - u*^2) over it while the lip is ahead of
	// the centre, the full radius once it is under or behind it.
	const double			R = 10.0, Step = 15.0, Dil = 3.0;
	const FRammsCurbTerrain T = FlatCurb(Step);
	for (double D = -30.0; D <= 10.0; D += 0.5)
	{
		const double U = -Dil - D;
		double		 Closed = R; // the road under the near half
		if (U <= 0.0)
		{
			Closed = Step + R;
		}
		else if (U <= R)
		{
			Closed = FMath::Max(R, Step + Rim(R, U));
		}
		const double Got = RammsCurb::ClearanceHeight(T, FVector2D(D, 0.0), FVector2D(1.0, 0.0), R, 6.0, Dil);
		TestEqual(FString::Printf(TEXT("centre %+.1f cm from the edge"), D), Got, Closed, 1e-6);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbContactClearanceProfileDescent, "Ramms.Curb.Contact.ClearanceProfileDescent", TestFlags)
bool FRammsCurbContactClearanceProfileDescent::RunTest(const FString& Parameters)
{
	// Down a curb: the top taken Dilation out past its lip, at u* = Dilation
	// - d. The top is under the wheel's trailing part until the lip is behind
	// it; ahead of the centre the full radius over the top.
	const double			R = 10.0, Step = -15.0, Dil = 2.0;
	const FRammsCurbTerrain T = FlatCurb(Step);
	for (double D = -20.0; D <= 30.0; D += 0.5)
	{
		const double U = Dil - D; // the top is under u < U
		double		 Closed = Step + R;
		if (U > 0.0)
		{
			Closed = R;
		}
		else if (U >= -R) // the top's corner is the rim's rearmost point's to clear at U = -R
		{
			Closed = FMath::Max(Step + R, Rim(R, U));
		}
		const double Got = RammsCurb::ClearanceHeight(T, FVector2D(D, 0.0), FVector2D(1.0, 0.0), R, 6.0, Dil);
		TestEqual(FString::Printf(TEXT("centre %+.1f cm from the lip"), D), Got, Closed, 1e-6);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbContactObliqueLip30, "Ramms.Curb.Contact.ObliqueLip30", TestFlags)
bool FRammsCurbContactObliqueLip30::RunTest(const FString& Parameters)
{
	// The edge 30 deg to the wheel's roll, a wide tyre: its leading corner
	// meets the lip first. Nine samples along, three across, and the exact
	// lip crossing on each line find what a dense grid does, never less --
	// and more only by the grid's own resolution where the rim is steep
	// (near the end of its reach, the next grid point along can sit 0.1 cm
	// lower on the rim). The middle line alone misses the corner.
	const double R = 11.0, Width = 10.0;
	for (const double Step : { 15.0, -15.0 })
	{
		const FRammsCurbTerrain T = FlatCurb(Step, 30.0);
		const double			Dil = Step > 0.0 ? 3.0 : 2.0;
		double					Worst = 0.0;
		double					CentreOnlyShort = 0.0;
		for (double D = -25.0; D <= 25.0; D += 0.25)
		{
			const FVector2D C = Dir(30.0) * D; // centre D past the edge, on its normal through the origin
			const double	Got = RammsCurb::ClearanceHeight(T, C, FVector2D(1.0, 0.0), R, Width, Dil);
			const double	Brute = BruteClearance(T, C, FVector2D(1.0, 0.0), R, Width, Dil);
			if (Got < Brute - 1e-6)
			{
				AddError(FString::Printf(TEXT("step %+.0f, centre %+.2f: %.4f, under the dense grid's %.4f"), Step, D, Got, Brute));
			}
			Worst = FMath::Max(Worst, Got - Brute);
			CentreOnlyShort = FMath::Max(CentreOnlyShort, Brute - RammsCurb::ClearanceHeight(T, C, FVector2D(1.0, 0.0), R, 0.0, Dil));
		}
		TestTrue(FString::Printf(TEXT("step %+.0f: within 0.2 cm of the dense grid everywhere (%.4f)"), Step, Worst), Worst < 0.2);
		TestTrue(FString::Printf(TEXT("step %+.0f: the tyre's middle alone falls short by %.2f cm somewhere"), Step, CentreOnlyShort),
			CentreOnlyShort > 1.0);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbContactLipDilation, "Ramms.Curb.Contact.LipDilation", TestFlags)
bool FRammsCurbContactLipDilation::RunTest(const FString& Parameters)
{
	// A wheel whose front is 2 cm short of the face, inside a 3 cm dilation:
	// it must already clear the curb top, though no part of it is over it.
	const double			R = 10.0, Step = 15.0;
	const FRammsCurbTerrain T = FlatCurb(Step);
	const FVector2D			C(-R - 2.0, 0.0);
	const double			Dilated = RammsCurb::ClearanceHeight(T, C, FVector2D(1.0, 0.0), R, 6.0, 3.0);
	const double			Bare = RammsCurb::ClearanceHeight(T, C, FVector2D(1.0, 0.0), R, 6.0, 0.0);
	TestEqual(TEXT("without the dilation only the road counts"), Bare, R, 1e-9);
	TestEqual(TEXT("with it the top's corner, 1 cm inside the rim's reach"), Dilated, Step + Rim(R, R - 1.0), 1e-9);
	// Descending: the top reaches EdgeMargin past the lip.
	const FRammsCurbTerrain Down = FlatCurb(-Step);
	TestEqual(TEXT("down: a wheel 1 cm past the lip still clears the top"),
		RammsCurb::ClearanceHeight(Down, FVector2D(R + 1.0, 0.0), FVector2D(1.0, 0.0), R, 6.0, 2.0), Rim(R, -(R - 1.0)), 1e-9);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
