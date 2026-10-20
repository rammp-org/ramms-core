// Copyright Epic Games, Inc. All Rights Reserved.

// The curb controller's debug view (URammsCurbNegotiationController::DebugView):
// what it senses of the terrain, what it makes of it, its plan and its state,
// drawn in the world each tick, and graphs and a colour key over the viewport.
// Read-only: nothing here changes what the controller does.

#include "RammsCurbNegotiationController.h"

#include "CanvasItem.h"
#include "Curb/RammsCurbDebugView.h"
#include "Curb/RammsCurbExecutor.h"
#include "Curb/RammsCurbLevel.h"
#include "Curb/RammsCurbPlanner.h"
#include "Curb/RammsCurbRobotModel.h"
#include "Curb/RammsCurbRunStats.h"
#include "Curb/RammsCurbSafety.h"
#include "Curb/RammsCurbSchedule.h"
#include "DrawDebugHelpers.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerController.h"
#include "RammsRobotBaseComponent.h"
#include "RenderUtils.h"
#include "SceneInterface.h"
#include "SceneView.h"

// --- the sums the view draws from (tested in RammsCurbDebugViewTests) -----------

namespace RammsCurb
{
	EScanSampleKind ClassifyScanSample(const FDebugScanSample& Sample, const FRammsCurbTerrain& Terrain, float ResidualCm)
	{
		if (!Sample.bHit)
		{
			return EScanSampleKind::Miss;
		}
		if (!Terrain.bValid)
		{
			return EScanSampleKind::Off;
		}
		const double D = Terrain.DistancePastEdge(Sample.XY);
		const bool	 bOnNear = FMath::Abs(Sample.Z - Terrain.Near.HeightAt(Sample.XY)) <= ResidualCm;
		const bool	 bOnFar = FMath::Abs(Sample.Z - Terrain.Far.HeightAt(Sample.XY)) <= ResidualCm;
		// The edge is a line fitted through the lanes' edges: within a few cm of
		// it a sample may sit on either plane.
		const bool bAtEdge = FMath::Abs(D) <= NearShortOfEdgeCm;
		if (D < 0.0)
		{
			return bOnNear ? EScanSampleKind::Near : (bOnFar && bAtEdge ? EScanSampleKind::Far : EScanSampleKind::Off);
		}
		return bOnFar ? EScanSampleKind::Far : (bOnNear && bAtEdge ? EScanSampleKind::Near : EScanSampleKind::Off);
	}

	double ProbeErrorCm(const FRammsCurbTerrain& Terrain, const FVector2D& BiasCm, const FVector2D& XY, bool bHit, double Z,
		double BottomZ)
	{
		const bool	 bFar = Terrain.DistancePastEdge(XY) >= 0.0;
		const double Expected = (bFar ? Terrain.Far : Terrain.Near).HeightAt(XY) + (bFar ? BiasCm.Y : BiasCm.X);
		return (bHit ? Z : BottomZ) - Expected;
	}

	FVector2D GraphRange(TConstArrayView<FDebugGraphSample> Samples, TConstArrayView<EGraphSeries> Series,
		TConstArrayView<float> Always, float MinSpan)
	{
		float Lo = TNumericLimits<float>::Max(), Hi = -TNumericLimits<float>::Max();
		for (const FDebugGraphSample& S : Samples)
		{
			for (const EGraphSeries Which : Series)
			{
				if (S.Has(Which))
				{
					Lo = FMath::Min(Lo, S.Get(Which));
					Hi = FMath::Max(Hi, S.Get(Which));
				}
			}
		}
		for (const float A : Always)
		{
			Lo = FMath::Min(Lo, A);
			Hi = FMath::Max(Hi, A);
		}
		if (Lo > Hi)
		{
			return FVector2D(0.0, MinSpan);
		}
		const float Pad = 0.08f * (Hi - Lo);
		Lo -= Pad;
		Hi += Pad;
		if (Hi - Lo < MinSpan)
		{
			const float Mid = 0.5f * (Lo + Hi);
			Lo = Mid - 0.5f * MinSpan;
			Hi = Mid + 0.5f * MinSpan;
		}
		return FVector2D(Lo, Hi);
	}

	TArray<int32> ConvexHull2D(TConstArrayView<FVector2D> Points)
	{
		TArray<int32> Order;
		for (int32 i = 0; i < Points.Num(); ++i)
		{
			Order.Add(i);
		}
		Order.Sort([&Points](int32 A, int32 B) {
			return Points[A].X < Points[B].X || (Points[A].X == Points[B].X && Points[A].Y < Points[B].Y);
		});
		// Distinct points only.
		TArray<int32> P;
		for (const int32 i : Order)
		{
			if (P.Num() == 0 || !Points[P.Last()].Equals(Points[i], 1e-6))
			{
				P.Add(i);
			}
		}
		if (P.Num() < 3)
		{
			return P;
		}
		auto Cross = [&Points](int32 O, int32 A, int32 B) {
			return FVector2D::CrossProduct(Points[A] - Points[O], Points[B] - Points[O]);
		};
		TArray<int32> Hull;
		Hull.SetNum(2 * P.Num());
		int32 K = 0;
		for (int32 i = 0; i < P.Num(); ++i)
		{
			while (K >= 2 && Cross(Hull[K - 2], Hull[K - 1], P[i]) <= 0.0)
			{
				--K;
			}
			Hull[K++] = P[i];
		}
		for (int32 i = P.Num() - 2, Lower = K + 1; i >= 0; --i)
		{
			while (K >= Lower && Cross(Hull[K - 2], Hull[K - 1], P[i]) <= 0.0)
			{
				--K;
			}
			Hull[K++] = P[i];
		}
		Hull.SetNum(K - 1);
		return Hull;
	}
} // namespace RammsCurb

// --- drawing helpers ----------------------------------------------------------------

namespace
{
	using namespace RammsCurb;

	// Colours, shared by the world layers and the colour key.
	const FColor NearColour(70, 210, 110);
	const FColor FarColour(70, 150, 255);
	const FColor OffColour(255, 160, 40);
	const FColor MissColour(235, 70, 60);
	const FColor UnfittedColour(170, 170, 170);
	const FColor EdgeColour(255, 225, 60);
	const FColor RayColour(150, 150, 165);
	const FColor NearGridColour(40, 140, 70);
	const FColor FarGridColour(40, 95, 175);
	const FColor TextColour(235, 238, 242);
	const FColor WarnColour(255, 160, 40);
	const FColor BadColour(235, 70, 60);
	const FColor GoodColour(90, 210, 120);
	const FColor MeasuredColour(0, 220, 255);
	const FColor PlannedColour(230, 90, 255);

	const FColor WheelPalette[] = {
		FColor(255, 110, 110),
		FColor(110, 190, 255),
		FColor(255, 200, 80),
		FColor(130, 240, 130),
		FColor(220, 140, 255),
		FColor(255, 250, 140),
	};

	FColor WheelColour(int32 Leg)
	{
		constexpr int32 N = static_cast<int32>(UE_ARRAY_COUNT(WheelPalette));
		return WheelPalette[((Leg % N) + N) % N];
	}

	FColor StateColour(ERammsCurbWheelState State)
	{
		switch (State)
		{
			case ERammsCurbWheelState::Planted:
				return FColor(90, 210, 120);
			case ERammsCurbWheelState::Lifting:
				return FColor(255, 225, 60);
			case ERammsCurbWheelState::Clear:
				return FColor(0, 220, 255);
			case ERammsCurbWheelState::Planting:
				return FColor(255, 160, 40);
			case ERammsCurbWheelState::Loading:
				return FColor(220, 90, 255);
			case ERammsCurbWheelState::Held:
			default:
				return FColor(160, 160, 160);
		}
	}

	FColor KindColour(EScanSampleKind Kind, bool bFound)
	{
		switch (Kind)
		{
			case EScanSampleKind::Near:
				return NearColour;
			case EScanSampleKind::Far:
				return FarColour;
			case EScanSampleKind::Miss:
				return MissColour;
			case EScanSampleKind::Off:
			default:
				return bFound ? OffColour : UnfittedColour;
		}
	}

	FColor ProbeColour(double ErrorCm, float MismatchCm)
	{
		const double A = FMath::Abs(ErrorCm);
		return A <= 1.0 ? GoodColour : (A <= MismatchCm ? WarnColour : BadColour);
	}

	/** A leg's name, short when it reads as a corner or a side (FL, CR...). */
	FString ShortLegName(FName Name)
	{
		const FString S = Name.ToString().ToLower();
		const TCHAR*  Along = S.Contains(TEXT("front"))											   ? TEXT("F")
			 : S.Contains(TEXT("rear")) || S.Contains(TEXT("back"))								   ? TEXT("R")
			 : S.Contains(TEXT("center")) || S.Contains(TEXT("centre")) || S.Contains(TEXT("mid")) ? TEXT("C")
																								   : nullptr;
		const TCHAR*  Side = S.Contains(TEXT("left")) ? TEXT("L") : S.Contains(TEXT("right")) ? TEXT("R")
																							  : nullptr;
		return Along && Side ? FString(Along) + Side : Name.ToString();
	}

	double Inclination(double PitchDeg, double RollDeg)
	{
		return FMath::RadiansToDegrees(FMath::Atan(FMath::Sqrt(FMath::Square(FMath::Tan(FMath::DegreesToRadians(PitchDeg)))
			+ FMath::Square(FMath::Tan(FMath::DegreesToRadians(RollDeg))))));
	}

	/** The chassis' up axis for an attitude: X along Forward (horizontal, unit)
	 *  raised Pitch, Y across raised Roll (+Y up), Z = X x Y. */
	FVector UpFor(const FVector& Forward, double PitchDeg, double RollDeg)
	{
		const FVector F = FVector(Forward.X, Forward.Y, 0.0).GetSafeNormal();
		const FVector R = FVector::CrossProduct(FVector::UpVector, F);
		const double  P = FMath::DegreesToRadians(PitchDeg), Q = FMath::DegreesToRadians(RollDeg);
		const FVector X = F * FMath::Cos(P) + FVector::UpVector * FMath::Sin(P);
		const FVector Y = R * FMath::Cos(Q) + FVector::UpVector * FMath::Sin(Q);
		return FVector::CrossProduct(X, Y).GetSafeNormal();
	}

	/** Text beside something drawn in the world, for this tick: drawn over
	 *  the viewport where At is on screen (DrawDebugOverlay). */
	void Label(FDebugViewState& State, const FVector& At, const FString& Text, const FColor& Colour = TextColour)
	{
		State.Labels.Add({ At, Text, Colour });
	}

	constexpr uint8 OnTop = SDPG_Foreground;
	constexpr uint8 InWorld = SDPG_World;
} // namespace

// --- per tick -----------------------------------------------------------------------

void URammsCurbNegotiationController::RecordDebugProbes(const RammsCurb::FExecMeasure& Measure)
{
	if (!DebugState || DebugView.Sensing == ERammsCurbSensingView::Off)
	{
		return;
	}
	const UWorld*				World = GetWorld();
	const double				Now = World ? World->GetTimeSeconds() : 0.0;
	const FVector2D				Bias = Level ? Level->GetBias() : FVector2D::ZeroVector;
	RammsCurb::FDebugViewState& S = *DebugState;
	S.Probes.Reset();
	for (const RammsCurb::FTerrainProbe& P : Measure.Probes)
	{
		RammsCurb::FDebugProbe& D = S.Probes.AddDefaulted_GetRef();
		D.Leg = P.Leg;
		D.XY = P.XY;
		D.TopZ = P.AboveZ;
		D.BottomZ = P.BelowZ;
		D.Z = P.Z;
		D.bHit = P.bHit;
		D.ErrorCm = ActiveTerrain.bValid ? RammsCurb::ProbeErrorCm(ActiveTerrain, Bias, P.XY, P.bHit, P.Z, P.BelowZ) : 0.0;
		D.Time = Now;
		if (DebugView.Sensing == ERammsCurbSensingView::Trail && P.bHit)
		{
			S.Trail.Add(D);
		}
	}
}

void URammsCurbNegotiationController::TickDebugView(float DeltaTime)
{
	if (!DebugState)
	{
		return;
	}
	RammsCurb::FDebugViewState& S = *DebugState;
	const UWorld*				World = GetWorld();
	if (!World || !DebugView.AnyOn())
	{
		S.Probes.Reset();
		S.Trail.Reset();
		S.Graph.Reset();
		S.Labels.Reset();
		return;
	}
	const double Now = World->GetTimeSeconds();

	// The probes of a tick that made none (no crossing) are gone; the trail
	// keeps its last TrailSeconds.
	S.Probes.RemoveAll([Now](const RammsCurb::FDebugProbe& P) { return Now - P.Time > 0.25 || Now < P.Time; });
	if (DebugView.Sensing == ERammsCurbSensingView::Trail)
	{
		const double Keep = DebugView.TrailSeconds;
		S.Trail.RemoveAll([Now, Keep](const RammsCurb::FDebugProbe& P) { return Now - P.Time > Keep || Now < P.Time; });
	}
	else
	{
		S.Trail.Reset();
	}

	// While no crossing runs, a preview of what a crossing asked for now would
	// find, every PreviewIntervalSeconds.
	const bool bIdle = !IsNegotiating() && !bStopping;
	if (bIdle && (DebugView.Sensing != ERammsCurbSensingView::Off || DebugView.bTerrain)
		&& (S.LastPreviewTime < 0.0 || Now < S.LastPreviewTime || Now - S.LastPreviewTime >= DebugView.PreviewIntervalSeconds))
	{
		S.LastPreviewTime = Now;
		S.bPreviewScan = true;
		FRammsCurbTerrain Found;
		FString			  Why;
		DetectTerrain(/*bBeforeStart=*/true, Found, Why, nullptr);
		S.bPreviewScan = false;
	}

	// The graphs' sample: simulated time, as the controller runs on it.
	if (DebugView.bGraphs)
	{
		const URammsRobotBaseComponent* Base = EnsureBase();
		double							T = Base ? Base->GetSimulationTime() : -1.0;
		if (T < 0.0)
		{
			T = Now;
		}
		if (S.Graph.Num() > 0 && T < S.Graph.Last().T)
		{
			S.Graph.Reset(); // the simulation was reset
		}
		using RammsCurb::EGraphSeries;
		RammsCurb::FDebugGraphSample G;
		G.T = T;
		if (Telemetry && Telemetry->Motion.IsFull())
		{
			G.Set(EGraphSeries::Speed, Telemetry->Motion.GetSpeed());
		}
		const bool bCrossing = IsNegotiating() && bDriveModeActive;
		const bool bRunning = bPerWheelCrossing && Executor && Executor->IsRunning();
		if (bCrossing || bStopping)
		{
			G.Set(EGraphSeries::SpeedCommand, DriveSpeed);
		}
		G.Set(EGraphSeries::SpeedFloor, bRunning && Executor->GetPlan().IsValid() ? static_cast<float>(Executor->GetPlan().VFloor) : Tuning.MinSpeedFraction * Tuning.DriveSpeed);
		FVector2D PitchRoll(MeasuredPitchDeg, MeasuredRollDeg);
		if (bCrossing || MeasureDebugAttitude(PitchRoll))
		{
			G.Set(EGraphSeries::Tilt, static_cast<float>(Inclination(PitchRoll.X, PitchRoll.Y)));
		}
		if (bRunning)
		{
			const FVector2D Planned = Executor->GetPlannedAttitude();
			G.Set(EGraphSeries::TiltPlanned, static_cast<float>(Inclination(Planned.X, Planned.Y)));
			G.Set(EGraphSeries::Margin, LiveMarginCm);
			G.Set(EGraphSeries::MarginPlanned, static_cast<float>(Executor->GetDiag().PlanMarginCm));
		}
		S.Graph.Add(G);
		const double From = T - DebugView.GraphSeconds;
		int32		 Old = 0;
		while (Old < S.Graph.Num() && S.Graph[Old].T < From)
		{
			++Old;
		}
		S.Graph.RemoveAt(0, Old, EAllowShrinking::No);
	}
	else
	{
		S.Graph.Reset();
	}

	S.Labels.Reset();
#if ENABLE_DRAW_DEBUG
	if (DebugView.Sensing != ERammsCurbSensingView::Off)
	{
		DrawDebugSensing(World);
	}
	if (DebugView.bTerrain)
	{
		DrawDebugTerrain(World);
	}
	if (DebugView.bPlan)
	{
		DrawDebugPlan(World);
	}
	if (DebugView.bRobot)
	{
		DrawDebugRobot(World);
	}
#endif
}

const FRammsCurbTerrain* URammsCurbNegotiationController::DebugTerrain() const
{
	if ((IsNegotiating() || bStopping) && ActiveTerrain.bValid)
	{
		return &ActiveTerrain;
	}
	if (DebugState && DebugState->Scan.bValid && DebugState->Scan.bFound && DebugState->Scan.Terrain.bValid)
	{
		return &DebugState->Scan.Terrain;
	}
	return nullptr;
}

bool URammsCurbNegotiationController::MeasureDebugAttitude(FVector2D& OutPitchRoll) const
{
	URammsRobotBaseComponent* Base = EnsureBase();
	if (!Base)
	{
		return false;
	}
	FVector Front = FVector::ZeroVector, Rear = FVector::ZeroVector, Left = FVector::ZeroVector, Right = FVector::ZeroVector;
	int32	NF = 0, NR = 0, NL = 0, NRt = 0;
	for (const FRammsCurbLeg& Leg : Legs)
	{
		FTransform Xf;
		if (Leg.Kind != ERammsCurbLegKind::FourBar || Leg.Axle == ERammsCurbAxle::Center || Leg.FourBar.CrankMotor.IsNone()
			|| !Base->GetMotorTransform(Leg.FourBar.CrankMotor, Xf))
		{
			continue;
		}
		const FVector P = Xf.GetLocation();
		(Leg.Axle == ERammsCurbAxle::Front ? Front : Rear) += P;
		++(Leg.Axle == ERammsCurbAxle::Front ? NF : NR);
		(Leg.Wheel.Position.Y >= 0.0f ? Right : Left) += P;
		++(Leg.Wheel.Position.Y >= 0.0f ? NRt : NL);
	}
	if (NF == 0 || NR == 0 || NL == 0 || NRt == 0)
	{
		return false;
	}
	const FVector Along = Front / NF - Rear / NR;
	const FVector Across = Right / NRt - Left / NL;
	OutPitchRoll = FVector2D(FMath::RadiansToDegrees(FMath::Atan2(Along.Z, Along.Size2D())),
		FMath::RadiansToDegrees(FMath::Atan2(Across.Z, Across.Size2D())));
	return true;
}

// --- the layers, in the world --------------------------------------------------------

void URammsCurbNegotiationController::DrawDebugSensing(const UWorld* World) const
{
	const RammsCurb::FDebugViewState& S = *DebugState;
	const RammsCurb::FDebugScan&	  Scan = S.Scan;
	if (Scan.bValid)
	{
		const FRammsCurbTerrain	 NoTerrain;
		const FRammsCurbTerrain& T = Scan.bFound ? Scan.Terrain : NoTerrain;
		for (int32 i = 0; i < Scan.Samples.Num(); ++i)
		{
			const RammsCurb::FDebugScanSample& P = Scan.Samples[i];
			const RammsCurb::EScanSampleKind   Kind = RammsCurb::ClassifyScanSample(P, T, Scan.ResidualCm);
			const FColor					   Colour = KindColour(Kind, Scan.bFound);
			if (Kind == RammsCurb::EScanSampleKind::Miss)
			{
				DrawDebugLine(World, FVector(P.XY, P.BottomZ), FVector(P.XY, P.BottomZ + 8.0), Colour, false, -1.0f, OnTop, 1.0f);
				continue;
			}
			// Every so often the last of the ray, straight down onto its hit (the
			// whole ray starts TopZ up: drawn whole they hide the rest).
			if (i % 40 == 0)
			{
				DrawDebugLine(World, FVector(P.XY, FMath::Min(P.TopZ, P.Z + 35.0)), FVector(P.XY, P.Z), RayColour, false, -1.0f, InWorld, 0.0f);
			}
			DrawDebugPoint(World, FVector(P.XY, P.Z + 0.5), 5.0f, Colour, false, -1.0f, OnTop);
		}
		// Where each scan lane found the edge: those the edge line went through
		// bright, any it left out dim.
		const FVector2D Right(-Scan.Heading.Y, Scan.Heading.X);
		for (const RammsCurb::FTerrainLane& Lane : Scan.Lanes)
		{
			if (!Lane.bEdge)
			{
				continue;
			}
			const FVector2D XY = Scan.Origin + Scan.Heading * Lane.EdgeAtCm + Right * Lane.OffsetCm;
			const double	Z = T.bValid ? FMath::Max(T.Near.HeightAt(XY), T.Far.HeightAt(XY)) : S.Scan.Terrain.EdgePoint.Z;
			DrawDebugPoint(World, FVector(XY, Z + 1.0), 11.0f, Lane.bUsed ? EdgeColour : FColor(130, 115, 30), false, -1.0f,
				OnTop);
		}
		if (DebugView.bLabels)
		{
			const double Age = World->GetTimeSeconds() - Scan.Time;
			Label(*DebugState, FVector(Scan.Origin, (T.bValid ? T.Near.HeightAt(Scan.Origin) : Scan.Terrain.EdgePoint.Z) + 130.0),
				FString::Printf(TEXT("%s scan: %d traces, %.1f ms, %.1f s ago"), Scan.bPreview ? TEXT("preview") : TEXT("detection"),
					Scan.Samples.Num(), Scan.Ms, Age));
		}
	}

	// The terrain monitor: each probe's ray, and where it hit against the
	// surface the crossing planned on there.
	for (const RammsCurb::FDebugProbe& P : S.Probes)
	{
		const FColor Colour = ProbeColour(P.ErrorCm, Tuning.TerrainMismatchCm);
		const double End = P.bHit ? P.Z : P.BottomZ;
		DrawDebugLine(World, FVector(P.XY, P.TopZ), FVector(P.XY, End), FColor(200, 200, 210), false, -1.0f, InWorld, 0.0f);
		DrawDebugPoint(World, FVector(P.XY, End + 0.5), P.bHit ? 8.0f : 12.0f, P.bHit ? Colour : MissColour, false, -1.0f, OnTop);
	}
	for (const RammsCurb::FDebugProbe& P : S.Trail)
	{
		DrawDebugPoint(World, FVector(P.XY, P.Z + 0.5), 4.0f, ProbeColour(P.ErrorCm, Tuning.TerrainMismatchCm), false, -1.0f, OnTop);
	}
}

void URammsCurbNegotiationController::DrawDebugTerrain(const UWorld* World) const
{
	const FRammsCurbTerrain*  TerrainPtr = DebugTerrain();
	URammsRobotBaseComponent* Base = EnsureBase();
	if (!TerrainPtr)
	{
		if (DebugView.bLabels && DebugState->Scan.bValid && !DebugState->Scan.bFound)
		{
			const FVector2D O = DebugState->Scan.Origin;
			double			Z = 0.0;
			FTransform		Xf;
			if (Base && Legs.Num() > 0 && Base->GetMotorTransform(Legs[0].Wheel.MotorId, Xf))
			{
				Z = Xf.GetLocation().Z;
			}
			Label(*DebugState, FVector(O, Z + 110.0), FString::Printf(TEXT("no curb: %s"), *DebugState->Scan.Why), BadColour);
		}
		return;
	}
	const FRammsCurbTerrain& T = *TerrainPtr;
	const FVector2D			 E(T.EdgePoint.X, T.EdgePoint.Y);
	const FVector2D			 U = T.EdgeDirection;
	const FVector2D			 N = T.EdgeNormal;

	// How much of the planes to draw: back past the rearmost wheel, along the
	// edge past the outermost, across to the far side's depth.
	double NearDepth = 60.0, HalfU = 80.0;
	if (Base)
	{
		for (const FRammsCurbLeg& Leg : Legs)
		{
			FTransform Xf;
			if (Base->GetMotorTransform(Leg.Wheel.MotorId, Xf))
			{
				const FVector2D P(Xf.GetLocation().X, Xf.GetLocation().Y);
				NearDepth = FMath::Max(NearDepth, -T.DistancePastEdge(P) + Leg.WheelRadiusCm + 25.0);
				HalfU = FMath::Max(HalfU, FMath::Abs(T.AlongEdge(P)) + Leg.WheelRadiusCm + 25.0);
			}
		}
	}
	NearDepth = FMath::Min(NearDepth, 400.0);
	HalfU = FMath::Min(HalfU, 250.0);
	const double FarDepth = FMath::Clamp(T.FarDepthCm > 0.0f ? T.FarDepthCm : Tuning.FarFitDepthCm, 20.0f, 200.0f);
	const double Along0 = T.AlongEdge(E);
	auto		 At = [&](double Ucm, double Dcm, const FRammsCurbPlane& Plane, double Lift = 0.4) {
		const FVector2D XY = E + U * (Ucm - Along0) + N * Dcm;
		return FVector(XY, Plane.HeightAt(XY) + Lift);
	};
	auto Grid = [&](double D0, double D1, const FRammsCurbPlane& Plane, const FColor& Colour) {
		const double Step = 20.0;
		for (double D = D0; D <= D1 + 0.01; D += Step)
		{
			DrawDebugLine(World, At(-HalfU, D, Plane), At(HalfU, D, Plane), Colour, false, -1.0f, InWorld, 0.0f);
		}
		for (double Ucm = -HalfU; Ucm <= HalfU + 0.01; Ucm += Step)
		{
			DrawDebugLine(World, At(Ucm, D0, Plane), At(Ucm, D1, Plane), Colour, false, -1.0f, InWorld, 0.0f);
		}
	};
	Grid(-NearDepth, 0.0, T.Near, NearGridColour);
	Grid(0.0, FarDepth, T.Far, FarGridColour);

	// The edge: the face's foot on the near plane and its top on the far one,
	// joined every 20 cm; the arrow points across it, near side to far.
	DrawDebugLine(World, At(-HalfU, 0.0, T.Near, 0.8), At(HalfU, 0.0, T.Near, 0.8), EdgeColour, false, -1.0f, OnTop, 2.0f);
	DrawDebugLine(World, At(-HalfU, 0.0, T.Far, 0.8), At(HalfU, 0.0, T.Far, 0.8), EdgeColour, false, -1.0f, OnTop, 2.0f);
	for (double Ucm = -HalfU; Ucm <= HalfU + 0.01; Ucm += 20.0)
	{
		DrawDebugLine(World, At(Ucm, 0.0, T.Near, 0.8), At(Ucm, 0.0, T.Far, 0.8), EdgeColour, false, -1.0f, InWorld, 0.0f);
	}
	DrawDebugDirectionalArrow(World, At(0.0, 0.0, T.Far, 2.0), At(0.0, 45.0, T.Far, 2.0), 12.0f, EdgeColour, false, -1.0f, OnTop, 2.0f);
	if (T.bFarDepthObserved && T.FarDepthCm > 0.0f)
	{
		DrawDebugLine(World, At(-HalfU, FarDepth, T.Far, 0.8), At(HalfU, FarDepth, T.Far, 0.8), FarColour, false, -1.0f, OnTop, 2.0f);
	}

	if (DebugView.bLabels)
	{
		const FVector Top = At(0.0, 0.0, T.Far, 0.0);
		Label(*DebugState, Top + FVector(0.0, 0.0, 70.0),
			FString::Printf(TEXT("step %+.1f cm (%.1f..%.1f across the robot), met %.1f deg off square"), T.StepHeightCm,
				T.StepHeightRangeCm.X, T.StepHeightRangeCm.Y, T.ApproachAngleDeg),
			EdgeColour);
		Label(*DebugState, Top + FVector(0.0, 0.0, 55.0),
			FString::Printf(TEXT("edge: straight to %.2f cm  |  far side %.0f cm deep%s"), T.EdgeFitResidualCm, T.FarDepthCm,
				T.bFarDepthObserved ? TEXT(" (to a second edge)") : TEXT(" (as far as seen)")));
		Label(*DebugState, At(0.0, -0.5 * NearDepth, T.Near, 25.0),
			FString::Printf(TEXT("near plane: slope %.1f deg, fit %.2f cm rms over %d samples"), T.Near.SlopeDeg(), T.Near.FitRmsCm,
				T.Near.Samples),
			NearColour);
		Label(*DebugState, At(0.0, 0.5 * FarDepth, T.Far, 25.0),
			FString::Printf(TEXT("far plane: slope %.1f deg, fit %.2f cm rms over %d samples"), T.Far.SlopeDeg(), T.Far.FitRmsCm,
				T.Far.Samples),
			FarColour);
		if ((IsNegotiating() || bStopping) && Level)
		{
			const FVector2D Bias = Level->GetBias();
			Label(*DebugState, Top + FVector(0.0, 0.0, 40.0),
				FString::Printf(TEXT("re-fit by the probes: near %+.1f cm, far %+.1f cm"), Bias.X, Bias.Y));
		}
	}
}

void URammsCurbNegotiationController::DrawDebugPlan(const UWorld* World) const
{
	const bool bRunning = bPerWheelCrossing && Executor && Executor->IsRunning();
	if (!bRunning)
	{
		if (DebugView.bLabels && IsNegotiating())
		{
			const FString What = bAwaitingPlan ? FString(TEXT("plan: being made"))
				: bPerWheelCrossing			   ? FString(TEXT("plan: none running"))
											   : FString(TEXT("plan: the per-axle gait's plan is not drawn"));
			Label(*DebugState, ChassisOriginWorld + FVector(0.0, 0.0, 120.0), What);
		}
		return;
	}
	const RammsCurb::FCurbSchedule&		 Plan = Executor->GetPlan();
	const FRammsCurbTerrain&			 T = ActiveTerrain.bValid ? ActiveTerrain : Plan.Terrain;
	const double						 SNow = Executor->GetS();
	const FVector2D						 H = Plan.Heading;
	const TArray<RammsCurb::FExecWheel>& Wheels = Executor->GetWheels();
	auto								 OriginAt = [&Plan, &H](double S) { return Plan.Origin + H * S; };

	// The chassis origin's planned path, coloured by the speed planned along it
	// against the floor; a planned stop as a post.
	const float						  Floor = static_cast<float>(FMath::Max(Plan.VFloor, 0.1));
	const RammsCurb::FScheduleSample* Prev = nullptr;
	for (int32 k = 0; k < Plan.Samples.Num(); k += 4)
	{
		const RammsCurb::FScheduleSample& Smp = Plan.Samples[k];
		if (Prev)
		{
			const FColor Colour = Smp.bStop || Smp.V < 0.01 ? BadColour
				: Smp.V < Floor - 0.01						? WarnColour
				: Smp.V < 1.5f * Floor						? EdgeColour
															: GoodColour;
			DrawDebugLine(World, FVector(OriginAt(Prev->S), Prev->B), FVector(OriginAt(Smp.S), Smp.B), Colour, false, -1.0f, OnTop, 2.5f);
		}
		if (Smp.bStop)
		{
			DrawDebugLine(World, FVector(OriginAt(Smp.S), Smp.B), FVector(OriginAt(Smp.S), Smp.B + 25.0), BadColour, false, -1.0f, OnTop, 3.0f);
			if (DebugView.bLabels)
			{
				Label(*DebugState, FVector(OriginAt(Smp.S), Smp.B + 30.0), FString::Printf(TEXT("stop %.1f s"), Smp.DwellSeconds), BadColour);
			}
		}
		Prev = &Smp;
	}
	if (const RammsCurb::FScheduleSample* Here = Plan.SampleAt(SNow))
	{
		DrawDebugSphere(World, FVector(OriginAt(SNow), Here->B), 3.0f, 8, FColor::White, false, -1.0f, OnTop, 1.0f);
	}
	// Where the plan tilts the base: a band over the path.
	if (Plan.TiltRangeCm.Y > Plan.TiltRangeCm.X)
	{
		const RammsCurb::FScheduleSample* A = Plan.SampleAt(Plan.TiltRangeCm.X);
		const RammsCurb::FScheduleSample* B = Plan.SampleAt(Plan.TiltRangeCm.Y);
		if (A && B)
		{
			const FVector PA(OriginAt(A->S), A->B + 12.0), PB(OriginAt(B->S), B->B + 12.0);
			DrawDebugLine(World, PA, PB, PlannedColour, false, -1.0f, OnTop, 4.0f);
			if (DebugView.bLabels)
			{
				Label(*DebugState, 0.5 * (PA + PB) + FVector(0.0, 0.0, 10.0),
					FString::Printf(TEXT("planned tilt: pitch %.1f, roll %.1f deg (at most %.1f)"), Plan.PlannedTiltDeg.X,
						Plan.PlannedTiltDeg.Y, Plan.PlannedMaxTiltDeg),
					PlannedColour);
			}
		}
	}

	// Each wheel: its keep-off window over the edge (where its tyre must not
	// touch), drawn on the terrain along its track, the height it is carried
	// over it at, and where it lifts and lands.
	const double CosA = FVector2D::DotProduct(H, T.EdgeNormal);
	for (const RammsCurb::FWheelPlan& W : Plan.Wheels)
	{
		const int32 i = W.Leg;
		if (!Legs.IsValidIndex(i) || !LegState.IsValidIndex(i) || !LegState[i].bHasWorld || CosA < 0.2)
		{
			continue;
		}
		const FColor	Colour = WheelColour(i);
		const FVector2D P0(LegState[i].WheelWorld.X, LegState[i].WheelWorld.Y);
		const double	D0 = T.DistancePastEdge(P0);
		auto			TrackAt = [&](double D) { return P0 + H * ((D - D0) / CosA); };
		auto			Ground = [&](double D, double Lift) {
			   const FVector2D XY = TrackAt(D);
			   return FVector(XY, T.SurfaceAt(XY) + Lift);
		};
		const double DUp = FMath::Min(W.DUp, 0.0), DDown = FMath::Max(W.DDown, 0.0);
		DrawDebugLine(World, Ground(DUp, 1.0), FVector(TrackAt(0.0), T.Near.HeightAt(TrackAt(0.0)) + 1.0), Colour, false, -1.0f, OnTop, 3.0f);
		DrawDebugLine(World, FVector(TrackAt(0.0), T.Near.HeightAt(TrackAt(0.0)) + 1.0), FVector(TrackAt(0.0), T.Far.HeightAt(TrackAt(0.0)) + 1.0),
			Colour, false, -1.0f, OnTop, 3.0f);
		DrawDebugLine(World, FVector(TrackAt(0.0), T.Far.HeightAt(TrackAt(0.0)) + 1.0), Ground(DDown, 1.0), Colour, false, -1.0f, OnTop, 3.0f);
		if (Wheels.IsValidIndex(i) && Wheels[i].ZTop > 0.0)
		{
			// The tyre's bottom at the height the lifted wheel is carried to.
			const double  Bottom = Wheels[i].ZTop - Legs[i].WheelRadiusCm;
			const FVector A(TrackAt(DUp), Bottom), B(TrackAt(DDown), Bottom);
			DrawDebugLine(World, A, B, Colour, false, -1.0f, InWorld, 1.0f);
			DrawDebugLine(World, A, Ground(DUp, 1.0), Colour, false, -1.0f, InWorld, 0.0f);
			DrawDebugLine(World, B, Ground(DDown, 1.0), Colour, false, -1.0f, InWorld, 0.0f);
		}
		// Lift and landing, where the chassis' travel puts this wheel then.
		const FVector2D Lift = P0 + H * (W.SLiftStart - SNow);
		const FVector2D Land = P0 + H * (W.SLand - SNow);
		const double	ZLift = T.SurfaceAt(Lift), ZLand = T.SurfaceAt(Land);
		DrawDebugDirectionalArrow(World, FVector(Lift, ZLift + 1.0), FVector(Lift, ZLift + 16.0), 6.0f, Colour, false, -1.0f, OnTop, 2.0f);
		DrawDebugDirectionalArrow(World, FVector(Land, ZLand + 16.0), FVector(Land, ZLand + 1.0), 6.0f, Colour, false, -1.0f, OnTop, 2.0f);
		if (DebugView.bLabels)
		{
			const FString Name = ShortLegName(Legs[i].Name);
			Label(*DebugState, FVector(Lift, ZLift + 20.0), FString::Printf(TEXT("%s lift"), *Name), Colour);
			Label(*DebugState, FVector(Land, ZLand + 20.0), FString::Printf(TEXT("%s land"), *Name), Colour);
		}
	}

	if (DebugView.bLabels)
	{
		Label(*DebugState, ChassisOriginWorld + FVector(0.0, 0.0, 135.0),
			FString::Printf(TEXT("plan: %s, %.1f s, floor %.1f cm/s, %d planned stop%s, margin at least %.1f cm"),
				RammsCurb::PlanClassName(Plan.Class), Plan.PlannedCrossingSeconds, Plan.VFloor, Plan.PlannedStops,
				Plan.PlannedStops == 1 ? TEXT("") : TEXT("s"), Plan.MinMargin));
	}
}

void URammsCurbNegotiationController::DrawDebugRobot(const UWorld* World) const
{
	URammsRobotBaseComponent* Base = EnsureBase();
	if (!Base)
	{
		return;
	}
	const bool					   bRunning = bPerWheelCrossing && Executor && Executor->IsRunning() && Executor->GetWheels().Num() == Legs.Num();
	const bool					   bFresh = (IsNegotiating() || bStopping) && LegState.Num() == Legs.Num();
	const RammsCurb::FChassisFrame Frame{ ChassisOriginWorld, ChassisAxisX, ChassisAxisY, ChassisAxisZ };

	// The chassis: where it is and which way it faces, crossing or not.
	FVector	  Centre = FVector::ZeroVector;
	int32	  NC = 0;
	FVector2D PitchRoll(MeasuredPitchDeg, MeasuredRollDeg);
	FVector	  Forward = ChassisAxisX;
	if (bFresh)
	{
		Centre = ChassisOriginWorld;
		NC = 1;
	}
	else
	{
		for (const FRammsCurbLeg& Leg : Legs)
		{
			FTransform Xf;
			if (Base->GetMotorTransform(Leg.Wheel.MotorId, Xf))
			{
				Centre += Xf.GetLocation();
				++NC;
			}
		}
		FVector2D C, F;
		if (LocateChassis(C, F))
		{
			Forward = FVector(F.X, F.Y, 0.0);
		}
		MeasureDebugAttitude(PitchRoll);
	}
	if (NC == 0)
	{
		return;
	}
	Centre /= NC;

	// Each wheel: a ring in its state's colour; while crossing, its commanded
	// centre (a dot) tied to the measured one, and why a lift waits.
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FRammsCurbLeg& Leg = Legs[i];
		FTransform			 Xf;
		if (!Base->GetMotorTransform(Leg.Wheel.MotorId, Xf))
		{
			continue;
		}
		const FVector Wheel = Xf.GetLocation();
		FColor		  Colour(200, 200, 200);
		FString		  Text = ShortLegName(Leg.Name);
		if (bRunning)
		{
			const RammsCurb::FExecWheel& W = Executor->GetWheels()[i];
			Colour = StateColour(W.State);
			Text += FString::Printf(TEXT(" %s  d %+.1f  gap %.1f cm"), RammsCurb::WheelStateName(W.State), W.PastEdgeCm, W.BottomCm);
			if (W.bBlocked && !W.BlockedWhy.IsEmpty())
			{
				Text += FString::Printf(TEXT("\nwaits: %s"), *W.BlockedWhy);
			}
		}
		const FVector Axle = FVector::CrossProduct(FVector::UpVector, Forward.GetSafeNormal2D());
		DrawDebugCircle(World, Wheel, Leg.WheelRadiusCm, 24, Colour, false, -1.0f, OnTop, 2.0f, Forward.GetSafeNormal2D(), FVector::UpVector,
			false);
		DrawDebugLine(World, Wheel - Axle * 0.5f * Leg.TyreWidthCm, Wheel + Axle * 0.5f * Leg.TyreWidthCm, Colour, false, -1.0f, OnTop, 1.0f);
		if (bFresh && LegState.IsValidIndex(i))
		{
			const FLegState& L = LegState[i];
			const float		 X = L.FiveBar ? L.CmdX : L.ModelX;
			const FVector	 Cmd = Frame.ToWorld(FVector(X, Leg.Wheel.Position.Y, L.CmdZ));
			DrawDebugPoint(World, Cmd, 7.0f, FColor::White, false, -1.0f, OnTop);
			DrawDebugLine(World, Cmd, Wheel, FColor::White, false, -1.0f, OnTop, 1.0f);
		}
		if (DebugView.bLabels)
		{
			Label(*DebugState, Wheel + FVector(0.0, 0.0, Leg.WheelRadiusCm + 12.0), Text, Colour);
		}
	}

	// Stability, while crossing: the support polygon -- the contacts of the
	// wheels carrying the robot -- and the centre of mass over it.
	if (bFresh && Model && Model->Legs.Num() == Legs.Num())
	{
		TArray<FVector2D, TInlineAllocator<8>>				   XZ;
		TArray<RammsCurb::FMeasuredWheel, TInlineAllocator<8>> Measured;
		TArray<FVector>										   Contacts;
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			const FLegState& L = LegState[i];
			XZ.Add(FVector2D(L.MeasuredX, L.MeasuredZ));
			RammsCurb::FMeasuredWheel& W = Measured.AddDefaulted_GetRef();
			W.Centre = L.WheelWorld;
			W.bSupport = L.bHasWorld && (bRunning ? Executor->IsSupport(i) : IsLegPlanted(i));
			W.Gradient = (L.bOnFar ? ActiveTerrain.Far : ActiveTerrain.Near).Gradient;
			if (W.bSupport)
			{
				const FVector2D F = Model->Legs[i].RollDirection;
				const FVector	A = Frame.X * (-F.Y) + Frame.Y * F.X;
				Contacts.Add(RammsCurb::WheelContact(W.Centre, A, W.Gradient, Legs[i].WheelRadiusCm,
					RammsCurb::ContactWidth(Legs[i].TyreWidthCm, Legs[i].ContactWidthCm)));
			}
		}
		const RammsCurb::FPoseMargin M = RammsCurb::MeasuredMargin(*Model, Frame, XZ, Measured,
			FVector2D(ChassisForward.X, ChassisForward.Y), Tuning.Acceleration, Tuning.Deceleration);
		if (Contacts.Num() > 0)
		{
			TArray<FVector2D> Flat;
			double			  MeanZ = 0.0;
			for (const FVector& C : Contacts)
			{
				Flat.Add(FVector2D(C.X, C.Y));
				MeanZ += C.Z;
			}
			MeanZ /= Contacts.Num();
			const TArray<int32> Hull = RammsCurb::ConvexHull2D(Flat);
			for (int32 k = 0; k < Hull.Num(); ++k)
			{
				const FVector& A = Contacts[Hull[k]];
				const FVector& B = Contacts[Hull[(k + 1) % Hull.Num()]];
				DrawDebugLine(World, A + FVector(0, 0, 1), B + FVector(0, 0, 1), FColor::White, false, -1.0f, OnTop, 2.0f);
			}
			for (const FVector& C : Contacts)
			{
				DrawDebugPoint(World, C + FVector(0, 0, 1), 8.0f, FColor::White, false, -1.0f, OnTop);
			}
			const float	  Margin = static_cast<float>(M.DynamicCm);
			const FColor  MarginColour = Margin > 5.0f ? GoodColour : (Margin > 0.0f ? WarnColour : BadColour);
			const FVector Com = M.CenterOfMass;
			DrawDebugSphere(World, Com, 4.0f, 10, MarginColour, false, -1.0f, OnTop, 1.5f);
			DrawDebugLine(World, Com, FVector(Com.X, Com.Y, MeanZ + 1.0), MarginColour, false, -1.0f, OnTop, 1.0f);
			DrawDebugPoint(World, FVector(Com.X, Com.Y, MeanZ + 1.0), 10.0f, MarginColour, false, -1.0f, OnTop);
			if (DebugView.bLabels)
			{
				Label(*DebugState, Com + FVector(0.0, 0.0, 18.0),
					FString::Printf(TEXT("centre of mass: margin %.1f cm standing, %.1f braking%s"), M.StaticCm, M.DynamicCm,
						bRunning ? *FString::Printf(TEXT(" (plan %.1f)"), Executor->GetDiag().PlanMarginCm) : TEXT("")),
					MarginColour);
			}
		}
	}

	// The base's attitude: straight up (white), measured (cyan), planned
	// (magenta).
	const FVector Up0 = Centre + FVector(0.0, 0.0, 30.0);
	DrawDebugLine(World, Up0, Up0 + FVector(0.0, 0.0, 60.0), FColor(150, 150, 150), false, -1.0f, OnTop, 1.0f);
	DrawDebugDirectionalArrow(World, Up0, Up0 + UpFor(Forward, PitchRoll.X, PitchRoll.Y) * 60.0, 8.0f, MeasuredColour, false, -1.0f, OnTop, 2.0f);
	FString Attitude = FString::Printf(TEXT("pitch %+.1f  roll %+.1f deg"), PitchRoll.X, PitchRoll.Y);
	if (bRunning)
	{
		const FVector2D Planned = Executor->GetPlannedAttitude();
		DrawDebugDirectionalArrow(World, Up0, Up0 + UpFor(Forward, Planned.X, Planned.Y) * 60.0, 8.0f, PlannedColour, false, -1.0f, OnTop, 2.0f);
		Attitude += FString::Printf(TEXT("  (plan %+.1f, %+.1f)"), Planned.X, Planned.Y);
	}

	if (DebugView.bLabels)
	{
		FString Lines = FString::Printf(TEXT("curb: %s"), *Status);
		if (bRunning)
		{
			const RammsCurb::FExecDiag& D = Executor->GetDiag();
			Lines += FString::Printf(TEXT("\n%s  S %.0f cm  %.1f s"), RammsCurb::ExecPhaseName(Executor->GetPhase()), Executor->GetS(),
				Executor->GetElapsed());
			Lines += FString::Printf(TEXT("\nspeed %.1f / %.1f cm/s, set by %s"),
				Telemetry && Telemetry->Motion.IsFull() ? Telemetry->Motion.GetSpeed() : 0.0f, DriveSpeed, RammsCurb::ExecTermName(D.TargetBy));
		}
		Lines += TEXT("\n") + Attitude;
		Label(*DebugState, Up0 + FVector(0.0, 0.0, 75.0), Lines);
	}
}

// --- over the viewport: the colour key and the graphs ---------------------------------

namespace
{
	struct FOverlay
	{
		UCanvas*	 Canvas = nullptr;
		const UFont* Font = nullptr;
		float		 LineHeight = 14.0f;

		void Box(float X, float Y, float W, float H, const FLinearColor& Colour) const
		{
			FCanvasTileItem Tile(FVector2D(X, Y), GWhiteTexture, FVector2D(W, H), Colour);
			Tile.BlendMode = SE_BLEND_Translucent;
			Canvas->DrawItem(Tile);
		}
		void Line(float X0, float Y0, float X1, float Y1, const FLinearColor& Colour, float Thickness = 1.0f) const
		{
			FCanvasLineItem L(FVector2D(X0, Y0), FVector2D(X1, Y1));
			L.SetColor(Colour);
			L.LineThickness = Thickness;
			Canvas->DrawItem(L);
		}
		void Dashed(float X0, float X1, float Y, const FLinearColor& Colour) const
		{
			for (float X = X0; X < X1; X += 8.0f)
			{
				Line(X, Y, FMath::Min(X + 4.0f, X1), Y, Colour);
			}
		}
		float Text(float X, float Y, const FString& S, const FLinearColor& Colour) const
		{
			FCanvasTextItem Item(FVector2D(X, Y), FText::FromString(S), Font, Colour);
			Item.EnableShadow(FLinearColor::Black);
			Canvas->DrawItem(Item);
			float W = 0.0f, H = 0.0f;
			Canvas->TextSize(Font, S, W, H);
			return W;
		}
		float Width(const FString& S) const
		{
			float W = 0.0f, H = 0.0f;
			Canvas->TextSize(Font, S, W, H);
			return W;
		}
	};

	/** A colour key: chips (a square and its words) laid out in rows Width
	 *  wide, bottom up from Bottom; returns the top. */
	struct FKeyChip
	{
		FColor	Colour;
		FString Words;
	};

	struct FKeyRow
	{
		FString			 Title;
		TArray<FKeyChip> Chips;
	};

	float DrawKey(const FOverlay& O, float X, float Bottom, float Width, TConstArrayView<FKeyRow> Rows)
	{
		// Lay out first (rows wrap), then draw top down.
		struct FPlaced
		{
			float	X, Y;
			FColor	Colour;
			FString Words;
			bool	bTitle;
		};
		TArray<FPlaced> Placed;
		float			Y = 0.0f;
		for (const FKeyRow& Row : Rows)
		{
			float Cx = 0.0f;
			Placed.Add({ Cx, Y, FColor::White, Row.Title, true });
			Cx += O.Width(Row.Title) + 10.0f;
			for (const FKeyChip& Chip : Row.Chips)
			{
				const float W = 14.0f + O.Width(Chip.Words) + 12.0f;
				if (Cx + W > Width)
				{
					Y += O.LineHeight + 2.0f;
					Cx = 18.0f;
				}
				Placed.Add({ Cx, Y, Chip.Colour, Chip.Words, false });
				Cx += W;
			}
			Y += O.LineHeight + 4.0f;
		}
		const float Top = Bottom - Y - 8.0f;
		O.Box(X - 6.0f, Top - 4.0f, Width + 12.0f, Y + 10.0f, FLinearColor(0.02f, 0.03f, 0.04f, 0.72f));
		for (const FPlaced& P : Placed)
		{
			if (P.bTitle)
			{
				O.Text(X + P.X, Top + P.Y, P.Words, FLinearColor(0.6f, 0.65f, 0.7f));
				continue;
			}
			O.Box(X + P.X, Top + P.Y + 3.0f, 10.0f, 10.0f, FLinearColor(P.Colour));
			O.Text(X + P.X + 14.0f, Top + P.Y, P.Words, FLinearColor::White);
		}
		return Top - 4.0f;
	}

	/** A level line across a graph: a limit, a floor, zero. */
	struct FGraphMark
	{
		float		 Value;
		FLinearColor Colour;
	};

	struct FGraphLine
	{
		EGraphSeries Series;
		FLinearColor Colour;
		const TCHAR* Name;
		bool		 bDashed = false;
	};

	/** One strip chart, Height tall, its top at Y; returns its bottom. */
	float DrawGraph(const FOverlay& O, float X, float Y, float Width, float Height, const TCHAR* Title,
		TConstArrayView<FDebugGraphSample> Samples, double TFrom, double Span, TConstArrayView<FGraphLine> Lines,
		TConstArrayView<FGraphMark> Marks, float MinSpan, const TCHAR* Units)
	{
		TArray<EGraphSeries> Series;
		for (const FGraphLine& L : Lines)
		{
			Series.Add(L.Series);
		}
		TArray<float> Always;
		for (const FGraphMark& M : Marks)
		{
			Always.Add(M.Value);
		}
		const FVector2D Range = GraphRange(Samples, Series, Always, MinSpan);
		auto			ToX = [&](double T) { return X + static_cast<float>((T - TFrom) / Span) * Width; };
		auto			ToY = [&](float V) { return Y + Height - static_cast<float>((V - Range.X) / (Range.Y - Range.X)) * Height; };

		// The title, and each line's latest value in its colour.
		float Tx = X + O.Text(X, Y - O.LineHeight - 2.0f, Title, FLinearColor(0.75f, 0.8f, 0.85f)) + 10.0f;
		for (const FGraphLine& L : Lines)
		{
			for (int32 k = Samples.Num() - 1; k >= 0; --k)
			{
				if (Samples[k].Has(L.Series))
				{
					Tx += O.Text(Tx, Y - O.LineHeight - 2.0f, FString::Printf(TEXT("%s %.1f"), L.Name, Samples[k].Get(L.Series)), L.Colour) + 10.0f;
					break;
				}
			}
		}
		O.Box(X, Y, Width, Height, FLinearColor(0.02f, 0.03f, 0.04f, 0.72f));
		O.Text(X + 3.0f, Y + 1.0f, FString::Printf(TEXT("%.0f %s"), Range.Y, Units), FLinearColor(0.5f, 0.55f, 0.6f));
		O.Text(X + 3.0f, Y + Height - O.LineHeight - 1.0f, FString::Printf(TEXT("%.0f"), Range.X), FLinearColor(0.5f, 0.55f, 0.6f));
		for (const FGraphMark& M : Marks)
		{
			O.Dashed(X, X + Width, ToY(M.Value), M.Colour);
		}
		// At most a sample a pixel.
		const int32 Stride = FMath::Max(1, Samples.Num() / FMath::Max(1, FMath::RoundToInt(Width)));
		for (const FGraphLine& L : Lines)
		{
			bool  bHave = false;
			float Px = 0.0f, Py = 0.0f;
			for (int32 k = 0; k < Samples.Num(); k += Stride)
			{
				const FDebugGraphSample& S = Samples[k];
				if (!S.Has(L.Series))
				{
					bHave = false;
					continue;
				}
				const float Nx = ToX(S.T), Ny = ToY(FMath::Clamp(S.Get(L.Series), static_cast<float>(Range.X), static_cast<float>(Range.Y)));
				if (bHave)
				{
					O.Line(Px, Py, Nx, Ny, L.Colour, L.bDashed ? 1.0f : 2.0f);
				}
				Px = Nx;
				Py = Ny;
				bHave = true;
			}
		}
		return Y + Height;
	}
} // namespace

void URammsCurbNegotiationController::DrawDebugOverlay(UCanvas* Canvas, APlayerController* Player)
{
	if (!Canvas || !DebugState || !GEngine)
	{
		return;
	}
	// Only the viewport of this controller's world.
	const UWorld* World = GetWorld();
	if (Player ? Player->GetWorld() != World
			   : !(Canvas->SceneView && Canvas->SceneView->Family && Canvas->SceneView->Family->Scene
					 && Canvas->SceneView->Family->Scene->GetWorld() == World))
	{
		return;
	}
	const bool bKey = DebugView.bLabels
		&& (DebugView.Sensing != ERammsCurbSensingView::Off || DebugView.bTerrain || DebugView.bPlan || DebugView.bRobot);
	const bool bText = DebugView.bLabels && DebugState->Labels.Num() > 0;
	if (!DebugView.bGraphs && !bKey && !bText)
	{
		return;
	}

	FOverlay O;
	O.Canvas = Canvas;
	O.Font = GEngine->GetSmallFont();
	O.LineHeight = 15.0f;

	// The world layers' labels, each where its point is on screen: its lines
	// centred over it, on a dark band.
	if (bText)
	{
		for (const RammsCurb::FDebugLabel& L : DebugState->Labels)
		{
			const FVector Screen = Canvas->Project(L.At);
			if (Screen.Z <= 0.0 || Screen.X < 0.0 || Screen.Y < 0.0 || Screen.X > Canvas->ClipX || Screen.Y > Canvas->ClipY)
			{
				continue;
			}
			TArray<FString> Lines;
			L.Text.ParseIntoArrayLines(Lines, /*bCullEmpty=*/false);
			float Widest = 0.0f;
			for (const FString& Line : Lines)
			{
				Widest = FMath::Max(Widest, O.Width(Line));
			}
			const float Height = Lines.Num() * O.LineHeight;
			const float Left = static_cast<float>(Screen.X) - 0.5f * Widest;
			const float Top = static_cast<float>(Screen.Y) - Height;
			O.Box(Left - 3.0f, Top - 1.0f, Widest + 6.0f, Height + 2.0f, FLinearColor(0.0f, 0.0f, 0.0f, 0.45f));
			for (int32 k = 0; k < Lines.Num(); ++k)
			{
				O.Text(Left, Top + k * O.LineHeight, Lines[k], FLinearColor(L.Colour));
			}
		}
	}
	if (!DebugView.bGraphs && !bKey)
	{
		return;
	}
	const float X = FMath::Clamp(static_cast<float>(DebugView.OverlayAnchor.X), 0.0f, 0.9f) * Canvas->ClipX;
	const float Width = FMath::Clamp(Canvas->ClipX * 0.36f, 300.0f, 560.0f);
	float		Bottom = FMath::Clamp(static_cast<float>(DebugView.OverlayAnchor.Y), 0.1f, 1.0f) * Canvas->ClipY - 6.0f;

	if (DebugView.bGraphs)
	{
		using RammsCurb::EGraphSeries;
		const TArray<RammsCurb::FDebugGraphSample>& Samples = DebugState->Graph;
		const double								Span = DebugView.GraphSeconds;
		const double								TTo = Samples.Num() > 0 ? Samples.Last().T : 0.0;
		const double								TFrom = TTo - Span;
		const float									H = 64.0f, Gap = O.LineHeight + 10.0f;
		const float									Top = Bottom - 3.0f * (H + Gap);
		const FLinearColor							Meas(MeasuredColour), Plan(PlannedColour), Cmd(FLinearColor::White), Lim(BadColour), Flo(WarnColour);
		float										Y = Top + Gap;
		const FGraphLine							SpeedLines[] = { { EGraphSeries::Speed, Meas, TEXT("measured") }, { EGraphSeries::SpeedCommand, Cmd, TEXT("commanded") },
									   { EGraphSeries::SpeedFloor, Flo, TEXT("floor"), true } };
		const FGraphMark							Zero[] = { { 0.0f, FLinearColor(0.4f, 0.4f, 0.45f) } };
		Y = DrawGraph(O, X, Y, Width, H, TEXT("chassis speed"), Samples, TFrom, Span, SpeedLines, Zero, 10.0f, TEXT("cm/s")) + Gap;
		const FGraphLine TiltLines[] = { { EGraphSeries::Tilt, Meas, TEXT("measured") }, { EGraphSeries::TiltPlanned, Plan, TEXT("planned") } };
		const FGraphMark TiltMarks[] = { { 0.0f, FLinearColor(0.4f, 0.4f, 0.45f) }, { Tuning.MaxPlannedTiltDeg, Lim } };
		Y = DrawGraph(O, X, Y, Width, H, TEXT("base tilt (inclination, deg; limit dashed)"), Samples, TFrom, Span, TiltLines, TiltMarks, 2.0f,
				TEXT("deg"))
			+ Gap;
		const FGraphLine MarginLines[] = { { EGraphSeries::Margin, Meas, TEXT("measured") }, { EGraphSeries::MarginPlanned, Plan, TEXT("planned") } };
		const FGraphMark MarginMarks[] = { { 0.0f, Lim } };
		DrawGraph(O, X, Y, Width, H, TEXT("stability margin (cm, while crossing)"), Samples, TFrom, Span, MarginLines, MarginMarks, 10.0f, TEXT("cm"));
		const FString SpanText = FString::Printf(TEXT("last %.0f s"), Span);
		O.Text(X + Width - O.Width(SpanText), Top - 2.0f, SpanText, FLinearColor(0.5f, 0.55f, 0.6f));
		Bottom = Top - 4.0f;
	}

	if (bKey)
	{
		TArray<FKeyRow> Rows;
		if (DebugView.Sensing != ERammsCurbSensingView::Off || DebugView.bTerrain)
		{
			Rows.Add({ TEXT("scan, planes"), { { NearColour, TEXT("near") }, { FarColour, TEXT("far") }, { OffColour, TEXT("neither") }, { UnfittedColour, TEXT("no curb") }, { MissColour, TEXT("no hit") }, { EdgeColour, TEXT("edge") } } });
		}
		if (DebugView.Sensing != ERammsCurbSensingView::Off)
		{
			Rows.Add({ TEXT("probes vs plan"), { { GoodColour, TEXT("1 cm") }, { WarnColour, FString::Printf(TEXT("%.1f cm"), Tuning.TerrainMismatchCm) }, { BadColour, TEXT("beyond") } } });
		}
		if (DebugView.bPlan)
		{
			Rows.Add({ TEXT("planned speed"), { { GoodColour, TEXT("1.5x floor") }, { EdgeColour, TEXT("above floor") }, { WarnColour, TEXT("below") }, { BadColour, TEXT("stop") }, { PlannedColour, TEXT("tilt") } } });
		}
		if (DebugView.bRobot)
		{
			Rows.Add({ TEXT("wheels"), { { StateColour(ERammsCurbWheelState::Planted), TEXT("planted") }, { StateColour(ERammsCurbWheelState::Lifting), TEXT("lifting") }, { StateColour(ERammsCurbWheelState::Clear), TEXT("clear") }, { StateColour(ERammsCurbWheelState::Planting), TEXT("planting") }, { StateColour(ERammsCurbWheelState::Loading), TEXT("loading") }, { StateColour(ERammsCurbWheelState::Held), TEXT("held/idle") } } });
			Rows.Add({ TEXT("up"), { { MeasuredColour, TEXT("measured") }, { PlannedColour, TEXT("planned") }, { FColor::White, TEXT("commanded wheel, support") } } });
		}
		DrawKey(O, X, Bottom, Width, Rows);
	}
}
