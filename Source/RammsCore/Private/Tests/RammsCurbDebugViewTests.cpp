// Tests for the curb controller's debug view (Private/Curb/RammsCurbDebugView.h):
// how a scan sample is classed against the planes its scan found, a terrain
// probe's error against the surface planned on, the support polygon's hull,
// a graph's range -- on a made-up step, made-up samples -- and the view's
// switches on the control surface, set and read back as the panel does.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Curb/RammsCurbDebugView.h"
#include "RammsControlTypes.h"
#include "RammsCurbNegotiationController.h"
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
	TestEqual(TEXT("...the curb's own controls still are"), Own.Num(), 3);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
