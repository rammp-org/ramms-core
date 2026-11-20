// Tests for the curb controller's debug view (Private/Curb/RammsCurbDebugView.h):
// how a scan sample is classed against the planes its scan found, a terrain
// probe's error against the surface planned on, the support polygon's hull,
// a graph's range, the graphs' sample buffer, and a strip chart and a colour
// key as a painter is asked to draw them (Private/Curb/RammsCurbScope.h, what
// RAMMP Gen1's view shares) -- on a made-up step, made-up samples -- and the
// view's switches on the control surface, set and read back as the panel does,
// and the same on both robots.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Curb/RammsCurbDebugView.h"
#include "RammsControlTypes.h"
#include "RammsCurbNegotiationController.h"
#include "RammsRammpGen1CurbController.h"
#include "UObject/Package.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	/** A made-up step: level near ground at z 0, the edge along +Y at x 100
	 *  crossed towards +X, a level far side 15 cm up. */
	FRammsCurbTerrain MadeUpStep()
	{
		FRammsCurbTerrain T;
		T.bValid = true;
		T.EdgePoint = FVector(100.0, 0.0, 0.0);
		T.EdgeDirection = FVector2D(0.0, 1.0);
		T.EdgeNormal = FVector2D(1.0, 0.0);
		T.Near.Origin = FVector(50.0, 0.0, 0.0);
		T.Far.Origin = FVector(150.0, 0.0, 15.0);
		T.StepHeightCm = 15.0f;
		T.FarDepthCm = 120.0f;
		return T;
	}

	RammsCurb::FDebugScanSample Hit(double X, double Y, double Z)
	{
		return { FVector2D(X, Y), 120.0, -100.0, Z, true };
	}

	/** A painter that draws nothing and keeps what it was asked to draw: each
	 *  text, how many lines (a polyline's segments counted among them) and
	 *  boxes, how many polylines, their most points and each one's thickness.
	 *  Every character is 6 wide. */
	struct FCurbScopeTestPainter : RammsCurb::FDebugPainter
	{
		mutable TArray<FString> Texts;
		mutable int32			Lines = 0;
		mutable int32			Boxes = 0;
		mutable int32			Polylines = 0;
		mutable int32			MostPoints = 0;
		mutable TArray<float>	Thicknesses;

		virtual void  Box(float, float, float, float, const FLinearColor&) const override { ++Boxes; }
		virtual void  Line(float, float, float, float, const FLinearColor&, float) const override { ++Lines; }
		virtual float Text(float, float, const FString& S, const FLinearColor&) const override
		{
			Texts.Add(S);
			return Width(S);
		}
		virtual float Width(const FString& S) const override { return 6.0f * S.Len(); }
		virtual void  Polyline(TConstArrayView<FVector2f> Points, const FLinearColor&, float Thickness) const override
		{
			++Polylines;
			Lines += Points.Num() - 1;
			MostPoints = FMath::Max(MostPoints, Points.Num());
			Thicknesses.Add(Thickness);
		}
	};
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbDebugViewClassify,
	"Ramms.Curb.DebugView.ClassifyScanSample",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbDebugViewClassify::RunTest(const FString& Parameters)
{
	using RammsCurb::EScanSampleKind;
	const FRammsCurbTerrain T = MadeUpStep();
	auto					Kind = [&T](const RammsCurb::FDebugScanSample& S) { return RammsCurb::ClassifyScanSample(S, T, 1.5f); };

	TestTrue(TEXT("on the near plane, short of the edge"), Kind(Hit(50.0, 10.0, 0.3)) == EScanSampleKind::Near);
	TestTrue(TEXT("on the far plane, past it"), Kind(Hit(150.0, -10.0, 15.4)) == EScanSampleKind::Far);
	TestTrue(TEXT("half way up the face"), Kind(Hit(99.0, 0.0, 7.5)) == EScanSampleKind::Off);
	TestTrue(TEXT("an outlier on the near side"), Kind(Hit(40.0, 0.0, 4.0)) == EScanSampleKind::Off);
	TestTrue(TEXT("at near height well past the edge (a gap, a second edge)"), Kind(Hit(170.0, 0.0, 0.0)) == EScanSampleKind::Off);
	TestTrue(TEXT("on the near plane just past the fitted edge"), Kind(Hit(102.0, 0.0, 0.2)) == EScanSampleKind::Near);
	TestTrue(TEXT("on the far plane just short of the fitted edge"), Kind(Hit(98.0, 0.0, 15.0)) == EScanSampleKind::Far);

	RammsCurb::FDebugScanSample Miss = Hit(50.0, 0.0, -100.0);
	Miss.bHit = false;
	TestTrue(TEXT("no hit"), Kind(Miss) == EScanSampleKind::Miss);
	TestTrue(TEXT("a scan that found no curb puts every hit on neither plane"),
		RammsCurb::ClassifyScanSample(Hit(50.0, 0.0, 0.0), FRammsCurbTerrain(), 1.5f) == EScanSampleKind::Off);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbDebugViewProbeError,
	"Ramms.Curb.DebugView.ProbeError",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbDebugViewProbeError::RunTest(const FString& Parameters)
{
	const FRammsCurbTerrain T = MadeUpStep();
	const FVector2D			Bias(0.5, -0.3);
	TestNearlyEqual(TEXT("near side: against the near plane plus its bias"),
		RammsCurb::ProbeErrorCm(T, Bias, FVector2D(50.0, 0.0), true, 1.0, -60.0), 0.5, 1e-9);
	TestNearlyEqual(TEXT("far side: against the far plane plus its bias"),
		RammsCurb::ProbeErrorCm(T, Bias, FVector2D(150.0, 0.0), true, 15.0, -60.0), 0.3, 1e-9);
	TestNearlyEqual(TEXT("a miss counts its depth to the bottom of its ray"),
		RammsCurb::ProbeErrorCm(T, Bias, FVector2D(150.0, 0.0), false, 0.0, -40.0), -40.0 - 14.7, 1e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbDebugViewHull,
	"Ramms.Curb.DebugView.ConvexHull",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbDebugViewHull::RunTest(const FString& Parameters)
{
	// A square with a point inside, a point on an edge and a repeated corner.
	const TArray<FVector2D> P = { FVector2D(0, 0), FVector2D(40, 0), FVector2D(40, 30), FVector2D(0, 30), FVector2D(20, 15),
		FVector2D(20, 0), FVector2D(40, 30) };
	const TArray<int32>		Hull = RammsCurb::ConvexHull2D(P);
	TestEqual(TEXT("the four corners only"), Hull.Num(), 4);
	double Area2 = 0.0;
	for (int32 k = 0; k < Hull.Num(); ++k)
	{
		Area2 += FVector2D::CrossProduct(P[Hull[k]], P[Hull[(k + 1) % Hull.Num()]]);
	}
	TestNearlyEqual(TEXT("counter-clockwise, enclosing the square"), Area2, 2.0 * 40.0 * 30.0, 1e-6);
	TestFalse(TEXT("the inner point is not on it"), Hull.Contains(4));
	TestFalse(TEXT("the point on an edge is not on it"), Hull.Contains(5));

	TestEqual(TEXT("two points: both"), RammsCurb::ConvexHull2D({ FVector2D(0, 0), FVector2D(5, 5) }).Num(), 2);
	TestEqual(TEXT("one point repeated: one"), RammsCurb::ConvexHull2D({ FVector2D(1, 1), FVector2D(1, 1) }).Num(), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbDebugViewGraphRange,
	"Ramms.Curb.DebugView.GraphRange",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbDebugViewGraphRange::RunTest(const FString& Parameters)
{
	using RammsCurb::EGraphSeries;
	TArray<RammsCurb::FDebugGraphSample> Samples;
	for (int32 k = 0; k <= 10; ++k)
	{
		RammsCurb::FDebugGraphSample& S = Samples.AddDefaulted_GetRef();
		S.T = k * 0.1;
		S.Set(EGraphSeries::Speed, 2.0f * k);
		if (k % 2 == 0)
		{
			S.Set(EGraphSeries::Tilt, 100.0f); // another series: ignored
		}
	}
	const EGraphSeries Speed[] = { EGraphSeries::Speed };
	const float		   Zero[] = { 0.0f };
	const FVector2D	   R = RammsCurb::GraphRange(Samples, Speed, Zero, 5.0f);
	TestTrue(TEXT("covers the samples and the zero line"), R.X <= 0.0 && R.Y >= 20.0);
	TestTrue(TEXT("padded, not much"), R.Y < 23.0 && R.X > -3.0);
	TestTrue(TEXT("the other series is not in it"), R.Y < 100.0);

	const FVector2D Flat = RammsCurb::GraphRange(TConstArrayView<RammsCurb::FDebugGraphSample>(Samples).Left(1), Speed, {}, 5.0f);
	TestNearlyEqual(TEXT("one value: at least MinSpan tall"), Flat.Y - Flat.X, 5.0, 1e-4);
	const FVector2D None = RammsCurb::GraphRange({}, Speed, {}, 5.0f);
	TestTrue(TEXT("nothing to show: 0 to MinSpan"), None.Equals(FVector2D(0.0, 5.0)));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbDebugViewGraphBuffer,
	"Ramms.Curb.DebugView.GraphBuffer",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbDebugViewGraphBuffer::RunTest(const FString& Parameters)
{
	using RammsCurb::EGraphSeries;
	TArray<RammsCurb::FDebugGraphSample> Graph;
	// Ten seconds at 10 Hz, keeping the last 2.05.
	for (int32 k = 0; k <= 100; ++k)
	{
		RammsCurb::FDebugGraphSample S;
		S.T = k * 0.1;
		S.Set(EGraphSeries::Speed, static_cast<float>(k));
		RammsCurb::AddGraphSample(Graph, S, 2.05);
	}
	TestEqual(TEXT("the last 2.05 s: 21 samples"), Graph.Num(), 21);
	TestTrue(TEXT("oldest first, none older than the span"), Graph.Num() > 0 && Graph[0].T >= 10.0 - 2.05 && FMath::IsNearlyEqual(Graph.Last().T, 10.0, 1e-9));
	TestTrue(TEXT("each keeps its values"), Graph.Num() > 0 && Graph.Last().Has(EGraphSeries::Speed) && Graph.Last().Get(EGraphSeries::Speed) == 100.0f);
	TestFalse(TEXT("...and only those set"), Graph.Num() > 0 && Graph.Last().Has(EGraphSeries::SpeedCommand));

	// A time before the last: the simulation was reset, the graphs start again.
	RammsCurb::FDebugGraphSample Reset;
	Reset.T = 1.0;
	RammsCurb::AddGraphSample(Graph, Reset, 2.05);
	TestEqual(TEXT("a reset starts again"), Graph.Num(), 1);

	// The same time again (the simulation paused, the game still ticking): no reset, and no new sample either -- the
	// latest replaces the one there, however many frames the pause lasts.
	RammsCurb::FDebugGraphSample Paused = Reset;
	Paused.Set(EGraphSeries::Speed, 7.0f);
	for (int32 Frame = 0; Frame < 1000; ++Frame)
	{
		RammsCurb::AddGraphSample(Graph, Paused, 2.05);
	}
	TestEqual(TEXT("the same time again replaces the last: no pile-up while paused"), Graph.Num(), 1);
	TestTrue(TEXT("...with the latest's values"), Graph.Num() == 1 && Graph.Last().Has(EGraphSeries::Speed) && Graph.Last().Get(EGraphSeries::Speed) == 7.0f);
	RammsCurb::FDebugGraphSample Later = Reset;
	Later.T = 1.1;
	RammsCurb::AddGraphSample(Graph, Later, 2.05);
	TestEqual(TEXT("time going on again adds"), Graph.Num(), 2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbDebugViewDrawing,
	"Ramms.Curb.DebugView.Drawing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbDebugViewDrawing::RunTest(const FString& Parameters)
{
	using RammsCurb::EGraphSeries;
	// A second of samples: the speed 0..10 throughout, its command 2k from the sixth on.
	TArray<RammsCurb::FDebugGraphSample> Samples;
	for (int32 k = 0; k <= 10; ++k)
	{
		RammsCurb::FDebugGraphSample& S = Samples.AddDefaulted_GetRef();
		S.T = k * 0.1;
		S.Set(EGraphSeries::Speed, static_cast<float>(k));
		if (k >= 5)
		{
			S.Set(EGraphSeries::SpeedCommand, 2.0f * k);
		}
	}
	const RammsCurb::FGraphLine Lines[] = { { EGraphSeries::Speed, FLinearColor::White, TEXT("chassis") },
		{ EGraphSeries::SpeedCommand, FLinearColor::White, nullptr, true } };
	const RammsCurb::FGraphMark Marks[] = { { 0.0f, FLinearColor::White } };
	FCurbScopeTestPainter		P;
	P.LineHeight = 10.0f;
	const float Bottom = RammsCurb::DrawGraph(P, 0.0f, 5.0f, 200.0f, 40.0f, TEXT("speed"), Samples, 0.0, 1.0, Lines, Marks, 5.0f, TEXT("cm/s"));
	TestEqual(TEXT("its bottom: the title's line, then the chart"), Bottom, 5.0f + 10.0f + 2.0f + 40.0f);
	TestTrue(TEXT("the title"), P.Texts.Contains(FString(TEXT("speed"))));
	TestTrue(TEXT("a named line's latest value"), P.Texts.Contains(FString(TEXT("chassis 10.0"))));
	TestEqual(TEXT("no words for a line with no name: the title, one value, the range's ends"), P.Texts.Num(), 4);
	TestTrue(TEXT("the range covers both lines (the command's 20)"), P.Texts.Contains(FString(TEXT("22 cm/s"))));
	TestEqual(TEXT("one background"), P.Boxes, 1);
	// Ten segments of the speed, five of the command, and the zero line dashed every 8 px across 200.
	TestEqual(TEXT("the lines drawn"), P.Lines, 10 + 5 + 25);
	// Each series one polyline (no gap in either): the speed 2 px, the command thin.
	TestEqual(TEXT("a polyline a series"), P.Polylines, 2);
	TestTrue(TEXT("...the speed 2 px, the command (bThin) 1 px"), P.Thicknesses.Num() == 2 && P.Thicknesses[0] == 2.0f && P.Thicknesses[1] == 1.0f);

	// A gap in a series breaks its line: the speed missing at the fourth sample is two runs, 0..2 and 4..10.
	TArray<RammsCurb::FDebugGraphSample> Gapped = Samples;
	Gapped[3] = RammsCurb::FDebugGraphSample();
	Gapped[3].T = 0.3;
	FCurbScopeTestPainter G;
	G.LineHeight = 10.0f;
	const RammsCurb::FGraphLine SpeedOnly[] = { { EGraphSeries::Speed, FLinearColor::White, TEXT("chassis") } };
	RammsCurb::DrawGraph(G, 0.0f, 0.0f, 200.0f, 40.0f, TEXT("speed"), Gapped, 0.0, 1.0, SpeedOnly, {}, 5.0f, TEXT("cm/s"));
	TestEqual(TEXT("a gap breaks the line in two"), G.Polylines, 2);
	TestEqual(TEXT("...two segments before it, six after"), G.Lines, 2 + 6);

	// Many more samples than the chart is pixels wide: no more points than pixels.
	TArray<RammsCurb::FDebugGraphSample> Dense;
	for (int32 k = 0; k < 1000; ++k)
	{
		RammsCurb::FDebugGraphSample& S = Dense.AddDefaulted_GetRef();
		S.T = k * 0.01;
		S.Set(EGraphSeries::Speed, static_cast<float>(k % 7));
	}
	for (const float ChartWidth : { 100.0f, 333.0f, 999.0f, 1000.0f })
	{
		FCurbScopeTestPainter D;
		D.LineHeight = 10.0f;
		RammsCurb::DrawGraph(D, 0.0f, 0.0f, ChartWidth, 40.0f, TEXT("speed"), Dense, 0.0, 10.0, SpeedOnly, {}, 5.0f, TEXT("cm/s"));
		TestTrue(*FString::Printf(TEXT("%.0f px wide: one polyline of at most %.0f points (%d)"), ChartWidth, ChartWidth, D.MostPoints),
			D.Polylines == 1 && D.MostPoints <= static_cast<int32>(ChartWidth) && D.MostPoints >= static_cast<int32>(ChartWidth) / 2);
	}

	// A key, wide enough for a row on one line, and too narrow for any two chips on one.
	const RammsCurb::FKeyRow Rows[] = { { TEXT("wheels"), { { FColor::Green, TEXT("down") }, { FColor::Cyan, TEXT("up") }, { FColor::Yellow, TEXT("lifting") } } } };
	FCurbScopeTestPainter	 K;
	K.LineHeight = 10.0f;
	TestEqual(TEXT("a key on one line"), RammsCurb::DrawKey(K, 0.0f, 0.0f, 1000.0f, Rows, /*bDraw=*/true), 10.0f + 4.0f);
	TestEqual(TEXT("...its title and its chips' words"), K.Texts.Num(), 4);
	TestEqual(TEXT("...a square each"), K.Boxes, 3);
	FCurbScopeTestPainter Narrow;
	Narrow.LineHeight = 10.0f;
	TestEqual(TEXT("a narrow key wraps before each chip"), RammsCurb::DrawKey(Narrow, 0.0f, 0.0f, 60.0f, Rows, /*bDraw=*/false), 3.0f * (10.0f + 2.0f) + 10.0f + 4.0f);
	TestEqual(TEXT("...and, measured only, draws nothing"), Narrow.Texts.Num() + Narrow.Boxes + Narrow.Lines, 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbDebugViewControls,
	"Ramms.Curb.DebugView.Controls",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbDebugViewControls::RunTest(const FString& Parameters)
{
	URammsCurbNegotiationController* Curb = NewObject<URammsCurbNegotiationController>(GetTransientPackage());
	using C = URammsCurbNegotiationController;

	FRammsControlSurface Surface;
	Curb->DescribeControls(Surface);
	TArray<FRammsControlAxis> View;
	Surface.GetAxesInGroup(FName(TEXT("Curb view")), View);
	TestEqual(TEXT("six switches in their own group"), View.Num(), 6);
	for (const FRammsControlAxis& A : View)
	{
		TestTrue(*FString::Printf(TEXT("%s: a writable choice"), *A.Id.ToString()), A.Kind == ERammsControlKind::Enum && !A.bReadOnly);
		TestEqual(*FString::Printf(TEXT("%s: its range covers its labels"), *A.Id.ToString()), static_cast<int32>(A.Range.Y),
			A.EnumLabels.Num() - 1);
		TestEqual(*FString::Printf(TEXT("%s: shown as it is set"), *A.Id.ToString()), A.DefaultValue,
			A.Id == C::ViewLabelsControlId() ? 1.0f : 0.0f);
	}
	const FRammsControlAxis* Sensing = View.FindByPredicate([](const FRammsControlAxis& A) { return A.Id == C::ViewSensingControlId(); });
	TestTrue(TEXT("sensing: off, live, live and trail"), Sensing && Sensing->EnumLabels.Num() == 3);

	float Value = -1.0f;
	TestTrue(TEXT("sensing set"), Curb->ApplyControl(C::ViewSensingControlId(), 2.0f));
	TestTrue(TEXT("...to the trail"), Curb->DebugView.Sensing == ERammsCurbSensingView::Trail);
	TestTrue(TEXT("...read back"), Curb->ReadControl(C::ViewSensingControlId(), Value) && Value == 2.0f);
	TestTrue(TEXT("...its target is what it is"), Curb->ReadTarget(C::ViewSensingControlId(), Value) && Value == 2.0f);
	Curb->ApplyControl(C::ViewSensingControlId(), 7.0f);
	TestTrue(TEXT("an index past the end is the last"), Curb->DebugView.Sensing == ERammsCurbSensingView::Trail);

	TestTrue(TEXT("plan on"), Curb->ApplyControl(C::ViewPlanControlId(), 1.0f) && Curb->DebugView.bPlan);
	TestTrue(TEXT("labels off"), Curb->ApplyControl(C::ViewLabelsControlId(), 0.0f) && !Curb->DebugView.bLabels);
	Curb->DebugView.bGraphs = true;
	TestTrue(TEXT("set directly, read back"), Curb->ReadControl(C::ViewGraphsControlId(), Value) && Value == 1.0f);
	TestTrue(TEXT("...and the panel steps from it"), Curb->ReadTarget(C::ViewGraphsControlId(), Value) && Value == 1.0f);

	TestFalse(TEXT("the phase is read-only"), Curb->ApplyControl(C::PhaseControlId(), 1.0f));
	TestFalse(TEXT("an unknown Id is refused"), Curb->ApplyControl(FName(TEXT("curb.view.nothing")), 1.0f));
	TestFalse(TEXT("the phase has no target"), Curb->ReadTarget(C::PhaseControlId(), Value));

	Curb->DebugView.bExposeControls = false;
	FRammsControlSurface Hidden;
	Curb->DescribeControls(Hidden);
	TArray<FRammsControlAxis> HiddenView, Own;
	Hidden.GetAxesInGroup(FName(TEXT("Curb view")), HiddenView);
	Hidden.GetAxesInGroup(FName(TEXT("Curb")), Own);
	TestEqual(TEXT("not offered when asked not to"), HiddenView.Num(), 0);
	TestEqual(TEXT("...the curb's own controls still are (negotiate, abort, status, phase, assist, the four preferences)"), Own.Num(), 9);
	// The rider's preferences: sliders, set and read back through the controller.
	const FName				 Speed(TEXT("curb.pref.speed"));
	const FRammsControlAxis* SpeedAxis = Own.FindByPredicate([&](const FRammsControlAxis& A) { return A.Id == Speed; });
	TestTrue(TEXT("curb.pref.speed: a slider"), SpeedAxis && SpeedAxis->Kind == ERammsControlKind::Position);
	TestTrue(TEXT("curb.pref.speed: set"), Curb->ApplyControl(Speed, 0.25f));
	TestTrue(TEXT("curb.pref.speed: read"), Curb->ReadControl(Speed, Value) && FMath::IsNearlyEqual(Value, 0.25f));
	TestTrue(TEXT("curb.pref.speed: its target is its value"), Curb->ReadTarget(Speed, Value) && FMath::IsNearlyEqual(Value, 0.25f));
	TestTrue(TEXT("the crossing's tuning slower for it"), Curb->GetCrossingTuning().DriveSpeed < Curb->Tuning.DriveSpeed);
	Curb->Preferences = FRammsCurbPreferences();
	TestEqual(TEXT("in the middle: the tuning's own pace"), Curb->GetCrossingTuning().DriveSpeed, Curb->Tuning.DriveSpeed);

	// The status: read-only words, and how much they need the rider.
	const FRammsControlAxis* StatusAxis = Own.FindByPredicate([](const FRammsControlAxis& A) { return A.Id == FName(TEXT("curb.status")); });
	TestTrue(TEXT("curb.status: a read-only Text control"), StatusAxis && StatusAxis->Kind == ERammsControlKind::Text && StatusAxis->bReadOnly);
	FText Words;
	TestTrue(TEXT("status: idle, ready"), Curb->ReadControlText(FName(TEXT("curb.status")), Words) && Words.ToString().StartsWith(TEXT("Ready")));
	TestTrue(TEXT("status: information"), Curb->ReadControl(FName(TEXT("curb.status")), Value) && Value == static_cast<float>(ERammsControlTextSeverity::Info));
	TestFalse(TEXT("status: nothing to set"), Curb->ApplyControl(FName(TEXT("curb.status")), 1.0f));
	TestFalse(TEXT("status: no words for other controls"), Curb->ReadControlText(C::PhaseControlId(), Words));

	// Curb assist's switch: off at first, set and read back, a button on it flips it.
	const FName Assist(TEXT("curb.assist"));
	TestTrue(TEXT("assist: off at first"), Curb->ReadControl(Assist, Value) && Value == 0.0f && !Curb->Assist.bEnabled);
	TestTrue(TEXT("assist: on"), Curb->ApplyControl(Assist, 1.0f) && Curb->Assist.bEnabled);
	TestTrue(TEXT("assist: the panel steps from it"), Curb->ReadTarget(Assist, Value) && Value == 1.0f);
	TestTrue(TEXT("assist: a button flips it off"), Curb->TriggerControl(Assist) && !Curb->Assist.bEnabled);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsCurbDebugViewSharedSwitches,
	"Ramms.Curb.DebugView.SharedSwitches",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsCurbDebugViewSharedSwitches::RunTest(const FString& Parameters)
{
	using C = URammsCurbNegotiationController;
	namespace Layer = RammsCurb::ViewLayer;

	// The shared helper's Ids are Gen2's public ones (C::View*ControlId), layer by layer.
	const TPair<uint8, FName> Ids[] = { { Layer::Sensing, C::ViewSensingControlId() }, { Layer::Terrain, C::ViewTerrainControlId() },
		{ Layer::Plan, C::ViewPlanControlId() }, { Layer::Robot, C::ViewRobotControlId() }, { Layer::Graphs, C::ViewGraphsControlId() },
		{ Layer::Labels, C::ViewLabelsControlId() } };
	for (const TPair<uint8, FName>& Id : Ids)
	{
		TestEqual(*FString::Printf(TEXT("%s: the shared Id is Gen2's"), *Id.Value.ToString()), RammsCurb::ViewControlId(Id.Key), Id.Value);
	}
	TestTrue(TEXT("no Id for two layers at once"), RammsCurb::ViewControlId(Layer::Terrain | Layer::Robot).IsNone());

	// Both robots' switches, each view set alike: Gen2's are every layer's as the shared helper describes them, and each
	// of Gen1's is Gen2's own -- the same name, choices, order and value -- in Gen2's order.
	URammsCurbNegotiationController* Gen2 = NewObject<URammsCurbNegotiationController>(GetTransientPackage());
	URammsRammpGen1CurbController*	 Gen1 = NewObject<URammsRammpGen1CurbController>(GetTransientPackage());
	for (FRammsCurbDebugView* View : { &Gen2->DebugView, &Gen1->DebugView })
	{
		View->bTerrain = true;
		View->bGraphs = true;
		View->bLabels = false;
	}
	const auto Switches = [](const FRammsControlSurface& Surface) {
		TArray<FRammsControlAxis> Out;
		Surface.GetAxesInGroup(FName(TEXT("Curb view")), Out);
		return Out;
	};
	const auto Alike = [this](const TCHAR* What, const FRammsControlAxis& A, const FRammsControlAxis& B) {
		bool bLabels = A.EnumLabels.Num() == B.EnumLabels.Num();
		for (int32 k = 0; bLabels && k < A.EnumLabels.Num(); ++k)
		{
			bLabels = A.EnumLabels[k].ToString() == B.EnumLabels[k].ToString();
		}
		TestTrue(*FString::Printf(TEXT("%s, %s: the same name, kind, order, range, choices and value"), What, *A.Id.ToString()),
			A.Id == B.Id && A.Group == B.Group && A.DisplayName.ToString() == B.DisplayName.ToString() && A.Kind == B.Kind
				&& A.Units == B.Units && A.Order == B.Order && A.Range == B.Range && A.DefaultValue == B.DefaultValue
				&& A.bReadOnly == B.bReadOnly && bLabels);
	};
	FRammsControlSurface Gen2Surface, Gen1Surface, SharedSurface;
	Gen2->DescribeControls(Gen2Surface);
	Gen1->DescribeControls(Gen1Surface);
	RammsCurb::DescribeViewControls(SharedSurface, Gen2->DebugView, Layer::All);
	const TArray<FRammsControlAxis> Gen2View = Switches(Gen2Surface), Gen1View = Switches(Gen1Surface), SharedView = Switches(SharedSurface);

	TestEqual(TEXT("Gen2: a switch for every layer"), Gen2View.Num(), 6);
	TestEqual(TEXT("...as many as the shared helper describes"), Gen2View.Num(), SharedView.Num());
	for (int32 k = 0; k < FMath::Min(Gen2View.Num(), SharedView.Num()); ++k)
	{
		Alike(TEXT("Gen2 and the shared helper"), Gen2View[k], SharedView[k]);
	}
	TestEqual(TEXT("Gen1: a switch for each of its four layers"), Gen1View.Num(), 4);
	int32 LastIndex = -1;
	for (const FRammsControlAxis& A : Gen1View)
	{
		const int32 Index = Gen2View.IndexOfByPredicate([&A](const FRammsControlAxis& B) { return B.Id == A.Id; });
		TestTrue(*FString::Printf(TEXT("Gen1's %s is one of Gen2's, in Gen2's order"), *A.Id.ToString()), Index != INDEX_NONE && Index > LastIndex);
		if (Index != INDEX_NONE)
		{
			Alike(TEXT("Gen1 and Gen2"), A, Gen2View[Index]);
			LastIndex = Index;
		}
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
