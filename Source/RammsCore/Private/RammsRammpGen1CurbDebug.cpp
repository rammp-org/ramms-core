// Copyright Epic Games, Inc. All Rights Reserved.

// The RAMMP Gen1 curb controller's debug view (URammsRammpGen1CurbController::DebugView): the curb it finds, and the
// robot as the crossing sees it -- each wheel and its support's lift, commanded and measured, the wheels on the ground and
// the centre of mass over them, the frame over the edge, the chassis's attitude, the step it is in and what sets its
// speed -- drawn in the world each tick; and strip charts of its speeds, tilt, margins, lifts and translators, with a
// colour key, in the HUD's "Curb view" panel or over the viewport, drawn as RAMMP Gen2's are (Curb/RammsCurbScope).
// Read-only: nothing here changes what the controller does.

#include "RammsRammpGen1CurbController.h"

#include "Components/SceneComponent.h"
#include "Curb/RammsCurbRunStats.h"
#include "Curb/RammsCurbScope.h"
#include "Curb/RammsRammpGen1Curb.h"
#include "Curb/RammsRammpGen1CurbView.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "RammsRobotBaseComponent.h"

// Every name here is the RAMMP Gen1's own: a unity build puts this file beside Gen2's view, whose helpers have the plain
// names.
namespace
{
	// Each support's colour, and the translators', in the graphs; the attitude's and the margins' lines.
	constexpr FColor RammpGen1FrontColour(110, 190, 255);
	constexpr FColor RammpGen1DriveColour(255, 110, 110);
	constexpr FColor RammpGen1RearColour(130, 240, 130);
	constexpr FColor RammpGen1TranslatorColour(255, 200, 80);
	constexpr FColor RammpGen1PitchColour(255, 150, 90);
	constexpr FColor RammpGen1RollColour(220, 140, 255);
	constexpr FColor RammpGen1ComColour(255, 200, 80);
	constexpr FColor RammpGen1FrameColour(220, 140, 255);
	constexpr FColor RammpGen1IdleFrameColour(150, 150, 165);
	constexpr FColor RammpGen1VerticalColour(150, 150, 150);

	constexpr uint8 RammpGen1OnTop = SDPG_Foreground;
	constexpr uint8 RammpGen1WorldDepth = SDPG_World;

	/** Text beside something drawn in the world, for this tick: drawn over the viewport where At is on screen. */
	void RammpGen1Label(RammsCurb::FDebugScope& Scope, const FVector& At, const FString& Text, const FColor& Colour = RammsCurb::ViewColours::Text)
	{
		Scope.Labels.Add({ At, Text, Colour });
	}

	/** A wheel's name, by its place in SampleWheels' order. */
	const TCHAR* RammpGen1WheelName(int32 Index)
	{
		static const TCHAR* const Names[] = { TEXT("drive L"), TEXT("drive R"), TEXT("front L"), TEXT("front R"), TEXT("rear L"), TEXT("rear R") };
		return Index >= 0 && Index < static_cast<int32>(UE_ARRAY_COUNT(Names)) ? Names[Index] : TEXT("?");
	}

	/** A stability margin's colour: well inside, inside, outside. */
	FColor RammpGen1MarginColour(float MarginCm)
	{
		return MarginCm > 5.0f ? RammsCurb::ViewColours::Good : (MarginCm > 0.0f ? RammsCurb::ViewColours::Warn : RammsCurb::ViewColours::Bad);
	}

	/** Points in the ground plane. */
	TArray<FVector2D> RammpGen1Flat(TConstArrayView<FVector> Points)
	{
		TArray<FVector2D> Out;
		for (const FVector& P : Points)
		{
			Out.Add(FVector2D(P.X, P.Y));
		}
		return Out;
	}

	/** The chassis's roll (deg), as the measurements take it. */
	float RammpGen1RollDeg(const USceneComponent& Chassis)
	{
		return static_cast<float>(Chassis.GetComponentTransform().Rotator().Roll);
	}
} // namespace

// --- per tick ----------------------------------------------------------------------------------------

void URammsRammpGen1CurbController::TickDebugView(float DeltaTime)
{
	if (!DebugState)
	{
		return;
	}
	RammsRammpGen1Curb::FViewState& S = *DebugState;
	const UWorld*					World = GetWorld();
	// All off, it costs nothing (not even the chassis's lookup: a search of the pawn's components when it has none).
	const bool			   bAnyOn = DebugView.bTerrain || DebugView.bRobot || DebugView.bGraphs;
	const USceneComponent* Chassis = bAnyOn && World && Base ? Body(Config.ChassisBody) : nullptr;
	if (!Chassis)
	{
		S.Graph.Reset();
		S.Labels.Reset();
		S.ChassisMotion.Reset();
		S.RiderMotion.Reset();
		S.Preview = RammsRammpGen1Curb::FViewCurb();
		RammsCurb::UpdateLabelOverlay(S, *this, false);
		return;
	}
	const double Now = SimNow();

	// The chassis's and the rider's ground speed, along the chassis's heading (the simulation's clock, as the controller
	// runs on it).
	const FVector	 F = Chassis->GetForwardVector();
	const FVector2D	 Heading = FVector2D(F.X, F.Y).GetSafeNormal();
	const FTransform Xf = Chassis->GetComponentTransform();
	S.ChassisMotion.Add(Now, FVector2D(Xf.GetLocation().X, Xf.GetLocation().Y), Heading);
	const USceneComponent* Seat = Body(Config.SeatBody);
	const FVector		   RiderAt = (Seat ? Seat->GetComponentTransform() : Xf).TransformPosition(Config.RiderOffsetCm);
	S.RiderMotion.Add(Now, FVector2D(RiderAt.X, RiderAt.Y), Heading);

	// While no crossing runs, the curb a crossing asked for now would cross (or why none), every PreviewIntervalSeconds of
	// game time: a few hundred traces each.
	const bool bIdle = !IsCrossing() && !bStartPending;
	if (DebugView.bTerrain && bIdle)
	{
		const double				   GameNow = World->GetTimeSeconds();
		RammsRammpGen1Curb::FViewCurb& P = S.Preview;
		if (!P.bValid || GameNow < P.Time || GameNow - P.Time >= DebugView.PreviewIntervalSeconds)
		{
			P = RammsRammpGen1Curb::FViewCurb();
			P.bValid = true;
			P.Time = GameNow;
			FPlan Plan;
			P.bFound = CanStart(P.Why) && PlanAhead(Plan, P.Why);
			if (P.bFound)
			{
				P.EdgePoint = Plan.EdgePoint;
				P.EdgeNormal = Plan.EdgeNormal;
				P.StepCm = Plan.StepCm;
				P.NearZ = Plan.NearZ;
				P.HeightCm = Plan.HeightCm;
			}
		}
	}
	else
	{
		S.Preview = RammsRammpGen1Curb::FViewCurb();
	}

	// The graphs' sample.
	if (DebugView.bGraphs)
	{
		using RammsCurb::EGraphSeries;
		RammsCurb::FDebugGraphSample G;
		G.T = Now;
		if (S.ChassisMotion.IsFull())
		{
			G.Set(EGraphSeries::Speed, S.ChassisMotion.GetSpeed());
		}
		if (S.RiderMotion.IsFull())
		{
			G.Set(EGraphSeries::RiderSpeed, S.RiderMotion.GetSpeed());
		}
		const bool bCrossing = IsCrossing();
		if (bCrossing && bModeActive)
		{
			G.Set(EGraphSeries::SpeedCommand, CommandedSpeedCmS());
			// Rolling, what the speed law asks for (a paused or cancelled crossing's law is not running).
			if (bBlended && !bStickPaused && !bCancelling)
			{
				G.Set(EGraphSeries::SpeedTarget, BlendWantCmS);
			}
		}

		const float PitchDeg = FMath::RadiansToDegrees(PitchRad());
		const float RollDeg = RammpGen1RollDeg(*Chassis);
		G.Set(EGraphSeries::Pitch, PitchDeg);
		G.Set(EGraphSeries::Roll, RollDeg);
		G.Set(EGraphSeries::Tilt, static_cast<float>(RammsCurb::InclinationDeg(PitchDeg, RollDeg)));

		// The margins: the centre of mass inside the wheels on the ground, and ahead of the drive axle (the rear-lift
		// rule's); the frame over the edge, in a crossing.
		TArray<RammsRammpGen1Curb::FWheelSample> Wheels;
		SampleWheels(Wheels);
		const TArray<FVector> Contacts = RammsRammpGen1Curb::GroundContacts(Wheels, Config.TouchToleranceCm);
		const FVector		  Com = ComNow();
		if (Contacts.Num() > 0)
		{
			G.Set(EGraphSeries::Margin, RammsRammpGen1Curb::SupportMarginCm(RammpGen1Flat(Contacts), FVector2D(Com.X, Com.Y)));
		}
		G.Set(EGraphSeries::ComAheadOfAxle, ComAheadForView(Com));
		float Clear = 0.0f;
		if (bCrossing && FrameOverEdge(Clear))
		{
			G.Set(EGraphSeries::FrameClearance, Clear);
		}

		// The wheels at the tread: the drive wheels, and the front omni wheels where they have hub motors; measured and,
		// while a speed is asked of them, commanded.
		const auto TreadSpeeds = [this, &G](FName Left, FName Right, float RadiusCm, EGraphSeries Measured, EGraphSeries Commanded) {
			if (Left.IsNone() || Right.IsNone() || !Base->HasMotor(Left) || !Base->HasMotor(Right))
			{
				return;
			}
			G.Set(Measured, 0.5f * (Base->GetMotorVelocity(Left) + Base->GetMotorVelocity(Right)) * RadiusCm);
			float L = 0.0f, R = 0.0f;
			if (Base->GetMotorVelocityCommand(Left, L) && Base->GetMotorVelocityCommand(Right, R))
			{
				G.Set(Commanded, 0.5f * (L + R) * RadiusCm);
			}
		};
		TreadSpeeds(Config.DriveLeft, Config.DriveRight, Config.DriveWheelRadiusCm, EGraphSeries::DriveWheelSpeed, EGraphSeries::DriveWheelCommand);
		if (bPowered)
		{
			TreadSpeeds(Config.FrontOmniLeft, Config.FrontOmniRight, Config.FrontOmniRadiusCm, EGraphSeries::FrontHubSpeed, EGraphSeries::FrontHubCommand);
		}

		// Each support's height, measured and commanded, and the translators' travel.
		const auto Lift = [this, &G](ESupport Sup, EGraphSeries Measured, EGraphSeries Commanded) {
			float Cm = 0.0f;
			if (LiftHeightCm(Sup, false, Cm))
			{
				G.Set(Measured, Cm);
			}
			if (LiftHeightCm(Sup, true, Cm))
			{
				G.Set(Commanded, Cm);
			}
		};
		Lift(ESupport::Front, EGraphSeries::FrontLift, EGraphSeries::FrontLiftCommand);
		Lift(ESupport::Drive, EGraphSeries::DriveLift, EGraphSeries::DriveLiftCommand);
		Lift(ESupport::Rear, EGraphSeries::RearLift, EGraphSeries::RearLiftCommand);
		if (Base->HasMotor(Config.TranslatorLeft) && Base->HasMotor(Config.TranslatorRight))
		{
			G.Set(EGraphSeries::Translator, 0.5f * (Base->GetMotorValue(Config.TranslatorLeft) + Base->GetMotorValue(Config.TranslatorRight)));
			float L = 0.0f, R = 0.0f;
			if (Base->GetMotorCommand(Config.TranslatorLeft, L) && Base->GetMotorCommand(Config.TranslatorRight, R))
			{
				G.Set(EGraphSeries::TranslatorCommand, 0.5f * (L + R));
			}
		}
		RammsCurb::AddGraphSample(S.Graph, G, DebugView.GraphSeconds);
	}
	else
	{
		S.Graph.Reset();
	}

	S.Labels.Reset();
	RammsCurb::UpdateLabelOverlay(S, *this, DebugView.bLabels && World->IsGameWorld() && (DebugView.bTerrain || DebugView.bRobot));
#if ENABLE_DRAW_DEBUG
	if (DebugView.bTerrain)
	{
		DrawDebugTerrain(World);
	}
	if (DebugView.bRobot)
	{
		DrawDebugRobot(World);
	}
#endif
}

float URammsRammpGen1CurbController::ComAheadForView(const FVector& Com) const
{
	if (IsCrossing())
	{
		return ComAheadOfAxleCm();
	}
	const USceneComponent* Chassis = Body(Config.ChassisBody);
	if (!Chassis)
	{
		return 0.0f;
	}
	const FVector F = Chassis->GetForwardVector();
	const FVector Heading = FVector(F.X, F.Y, 0.0).GetSafeNormal();
	const FVector Axle = 0.5 * (BodyPos(Config.DriveWheelLeftBody) + BodyPos(Config.DriveWheelRightBody));
	return static_cast<float>((Com - Axle) | Heading);
}

// --- the layers, in the world ------------------------------------------------------------------------

void URammsRammpGen1CurbController::DrawDebugTerrain(const UWorld* World) const
{
	namespace Colours = RammsCurb::ViewColours;
	RammsRammpGen1Curb::FViewState& S = *DebugState;
	const USceneComponent*			Chassis = Body(Config.ChassisBody);
	if (!Chassis)
	{
		return;
	}
	// The curb the crossing runs on while one does; else the one a crossing asked for now would cross.
	RammsRammpGen1Curb::FViewCurb Curb;
	const bool					  bCrossing = IsCrossing() || bStartPending;
	if (bCrossing)
	{
		Curb.bValid = Curb.bFound = true;
		Curb.EdgePoint = EdgePoint;
		Curb.EdgeNormal = EdgeNormal;
		Curb.StepCm = StepCm;
		Curb.NearZ = NearZ;
		Curb.HeightCm = CrossingHeightCm;
	}
	else
	{
		Curb = S.Preview;
	}
	const FVector FrontHubs = 0.5 * (BodyPos(Config.FrontOmniLeftBody) + BodyPos(Config.FrontOmniRightBody));
	if (!Curb.bValid)
	{
		return;
	}
	if (!Curb.bFound)
	{
		if (DebugView.bLabels)
		{
			RammpGen1Label(S, FrontHubs + FVector(0.0, 0.0, 90.0), TEXT("no curb: ") + Curb.Why, Colours::Bad);
		}
		return;
	}
	const FVector2D N = FVector2D(Curb.EdgeNormal.X, Curb.EdgeNormal.Y).GetSafeNormal();
	const FVector2D U(-N.Y, N.X); // along the edge
	const FVector2D E(Curb.EdgePoint.X, Curb.EdgePoint.Y);
	const double	ZNear = Curb.NearZ, ZFar = Curb.NearZ + Curb.StepCm;

	// How much to draw: along the edge past the outermost wheel, back past the rearmost, the far level 60 cm deep.
	double HalfU = 60.0, NearDepth = 60.0;
	for (const FName Name : { Config.DriveWheelLeftBody, Config.DriveWheelRightBody, Config.FrontOmniLeftBody, Config.FrontOmniRightBody,
			 Config.RearOmniLeftBody, Config.RearOmniRightBody })
	{
		const FVector	P = BodyPos(Name);
		const FVector2D D = FVector2D(P.X, P.Y) - E;
		HalfU = FMath::Max(HalfU, FMath::Abs(FVector2D::DotProduct(D, U)) + 25.0);
		NearDepth = FMath::Max(NearDepth, -FVector2D::DotProduct(D, N) + 25.0);
	}
	HalfU = FMath::Min(HalfU, 200.0);
	NearDepth = FMath::Min(NearDepth, 300.0);
	const double FarDepth = 60.0;
	const auto	 At = [&E, &U, &N](double Ucm, double Dcm, double Z) { return FVector(E + U * Ucm + N * Dcm, Z); };
	const auto	 Grid = [&](double D0, double D1, double Z, const FColor& Colour) {
		  for (double D = D0; D <= D1 + 0.01; D += 20.0)
		  {
			  DrawDebugLine(World, At(-HalfU, D, Z + 0.4), At(HalfU, D, Z + 0.4), Colour, false, -1.0f, RammpGen1WorldDepth, 0.0f);
		  }
		  for (double Ucm = -HalfU; Ucm <= HalfU + 0.01; Ucm += 20.0)
		  {
			  DrawDebugLine(World, At(Ucm, D0, Z + 0.4), At(Ucm, D1, Z + 0.4), Colour, false, -1.0f, RammpGen1WorldDepth, 0.0f);
		  }
	};
	Grid(-NearDepth, 0.0, ZNear, Colours::NearGrid);
	Grid(0.0, FarDepth, ZFar, Colours::FarGrid);

	// The edge: the face's foot on the near level and its top on the far one, joined every 20 cm; the arrow points across
	// it, onto the far level.
	DrawDebugLine(World, At(-HalfU, 0.0, ZNear + 0.8), At(HalfU, 0.0, ZNear + 0.8), Colours::Edge, false, -1.0f, RammpGen1OnTop, 2.0f);
	DrawDebugLine(World, At(-HalfU, 0.0, ZFar + 0.8), At(HalfU, 0.0, ZFar + 0.8), Colours::Edge, false, -1.0f, RammpGen1OnTop, 2.0f);
	for (double Ucm = -HalfU; Ucm <= HalfU + 0.01; Ucm += 20.0)
	{
		DrawDebugLine(World, At(Ucm, 0.0, ZNear + 0.8), At(Ucm, 0.0, ZFar + 0.8), Colours::Edge, false, -1.0f, RammpGen1WorldDepth, 0.0f);
	}
	DrawDebugDirectionalArrow(World, At(0.0, 0.0, ZFar + 2.0), At(0.0, 45.0, ZFar + 2.0), 12.0f, Colours::Edge, false, -1.0f, RammpGen1OnTop, 2.0f);

	if (DebugView.bLabels)
	{
		// How far off square the chassis meets it now, and where its front omni wheels are against it.
		const FVector	F = Chassis->GetForwardVector();
		const FVector2D Heading = FVector2D(F.X, F.Y).GetSafeNormal();
		const double	OffSquare = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(FVector2D::DotProduct(N, Heading), -1.0, 1.0)));
		const double	FrontPast = FVector2D::DotProduct(FVector2D(FrontHubs.X, FrontHubs.Y) - E, N) + Config.FrontOmniRadiusCm;
		const FVector	Top = At(0.0, 0.0, ZFar);
		RammpGen1Label(S, Top + FVector(0.0, 0.0, 70.0),
			FString::Printf(TEXT("step %+.1f cm, met %.1f deg off square; crossing height %.1f cm"), Curb.StepCm, OffSquare, Curb.HeightCm),
			Colours::Edge);
		RammpGen1Label(S, Top + FVector(0.0, 0.0, 55.0),
			FString::Printf(TEXT("%s: the front omni wheels %.0f cm %s the edge"), bCrossing ? TEXT("crossing") : TEXT("curb ahead"),
				FMath::Abs(FrontPast), FrontPast < 0.0 ? TEXT("short of") : TEXT("past")));
		RammpGen1Label(S, At(0.0, -0.5 * NearDepth, ZNear + 25.0), TEXT("near level"), Colours::Near);
		RammpGen1Label(S, At(0.0, 0.5 * FarDepth, ZFar + 25.0), FString::Printf(TEXT("far level, %+.1f cm"), Curb.StepCm), Colours::Far);
	}
}

void URammsRammpGen1CurbController::DrawDebugRobot(const UWorld* World) const
{
	namespace Colours = RammsCurb::ViewColours;
	RammsRammpGen1Curb::FViewState& S = *DebugState;
	const USceneComponent*			Chassis = Body(Config.ChassisBody);
	if (!Chassis || !Base)
	{
		return;
	}
	const FTransform Xf = Chassis->GetComponentTransform();
	const FVector	 F = Chassis->GetForwardVector();
	const FVector	 Forward = FVector(F.X, F.Y, 0.0).GetSafeNormal();
	const FVector	 Up = Chassis->GetUpVector();
	const bool		 bCrossing = IsCrossing();
	// A rolling crossing moves each pair as it reaches the edge: its wheels show their pair's state. Otherwise a wheel is
	// down or up.
	const bool bPairs = bCrossing && bBlended && !bCancelling;
	const auto PairColour = [](EPair P) {
		switch (P)
		{
			case EPair::Lifting:
				return Colours::Lifting;
			case EPair::Up:
				return Colours::Clear;
			case EPair::Lowering:
				return Colours::Planting;
			case EPair::Pressing:
				return Colours::Loading;
			case EPair::Near:
			case EPair::Far:
			default:
				return Colours::Planted;
		}
	};
	const auto PairName = [](EPair P) {
		switch (P)
		{
			case EPair::Near:
				return TEXT("on the level it leaves");
			case EPair::Lifting:
				return TEXT("lifting");
			case EPair::Up:
				return TEXT("up, clear");
			case EPair::Lowering:
				return TEXT("lowering");
			case EPair::Pressing:
				return TEXT("pressed down again");
			case EPair::Far:
			default:
				return TEXT("on the far level");
		}
	};

	// Each wheel: a ring in its state's colour, where its support's command would put it (a dot, tied to it), and its gap
	// over the ground under it.
	TArray<RammsRammpGen1Curb::FWheelSample> Wheels;
	SampleWheels(Wheels);
	for (int32 i = 0; i < Wheels.Num(); ++i)
	{
		const RammsRammpGen1Curb::FWheelSample& W = Wheels[i];
		const ESupport							Sup = static_cast<ESupport>(i / 2);
		const EPair								Pair = PairState[i / 2];
		const bool								bDown = W.IsDown(Config.TouchToleranceCm);
		const FColor							Colour = bPairs ? PairColour(Pair) : (bDown ? Colours::Planted : Colours::Clear);
		DrawDebugCircle(World, W.Hub, W.RadiusCm, 24, Colour, false, -1.0f, RammpGen1OnTop, 2.0f, Forward, FVector::UpVector, false);
		// The support's height table puts a wheel up (+) or down against the chassis: its command, against where it is,
		// along the chassis's up.
		float Commanded = 0.0f, Measured = 0.0f;
		if (LiftHeightCm(Sup, true, Commanded) && LiftHeightCm(Sup, false, Measured) && FMath::Abs(Commanded - Measured) > 0.05f)
		{
			const FVector Cmd = W.Hub + Up * (Commanded - Measured);
			DrawDebugPoint(World, Cmd, 7.0f, FColor::White, false, -1.0f, RammpGen1OnTop);
			DrawDebugLine(World, Cmd, W.Hub, FColor::White, false, -1.0f, RammpGen1OnTop, 1.0f);
		}
		if (DebugView.bLabels)
		{
			FString Text = FString::Printf(TEXT("%s  %s"), RammpGen1WheelName(i), bPairs ? PairName(Pair) : (bDown ? TEXT("down") : TEXT("up")));
			Text += W.bGroundFound ? FString::Printf(TEXT("  gap %.1f cm"), W.GapCm()) : FString(TEXT("  no ground under it"));
			if (bCrossing)
			{
				Text += FString::Printf(TEXT("\n%+.0f cm past the edge"), Along(W.Hub));
			}
			RammpGen1Label(S, W.Hub + FVector(0.0, 0.0, W.RadiusCm + 12.0), Text, Colour);
		}
	}

	// Stability: the support polygon -- where the wheels on the ground touch it -- and the centre of mass over it.
	const TArray<FVector> Contacts = RammsRammpGen1Curb::GroundContacts(Wheels, Config.TouchToleranceCm);
	const FVector		  Com = ComNow();
	if (Contacts.Num() > 0)
	{
		const TArray<FVector2D> Flat = RammpGen1Flat(Contacts);
		double					MeanZ = 0.0;
		for (const FVector& C : Contacts)
		{
			MeanZ += C.Z;
		}
		MeanZ /= Contacts.Num();
		const TArray<int32> Hull = RammsCurb::ConvexHull2D(Flat);
		for (int32 k = 0; k < Hull.Num(); ++k)
		{
			const FVector& A = Contacts[Hull[k]];
			const FVector& B = Contacts[Hull[(k + 1) % Hull.Num()]];
			DrawDebugLine(World, A + FVector(0, 0, 1), B + FVector(0, 0, 1), FColor::White, false, -1.0f, RammpGen1OnTop, 2.0f);
		}
		for (const FVector& C : Contacts)
		{
			DrawDebugPoint(World, C + FVector(0, 0, 1), 8.0f, FColor::White, false, -1.0f, RammpGen1OnTop);
		}
		const float	 Margin = RammsRammpGen1Curb::SupportMarginCm(Flat, FVector2D(Com.X, Com.Y));
		const FColor MarginColour = RammpGen1MarginColour(Margin);
		DrawDebugSphere(World, Com, 4.0f, 10, MarginColour, false, -1.0f, RammpGen1OnTop, 1.5f);
		DrawDebugLine(World, Com, FVector(Com.X, Com.Y, MeanZ + 1.0), MarginColour, false, -1.0f, RammpGen1OnTop, 1.0f);
		DrawDebugPoint(World, FVector(Com.X, Com.Y, MeanZ + 1.0), 10.0f, MarginColour, false, -1.0f, RammpGen1OnTop);
		if (DebugView.bLabels)
		{
			RammpGen1Label(S, Com + FVector(0.0, 0.0, 18.0),
				FString::Printf(TEXT("centre of mass: margin %.1f cm over %d wheels down\n%+.1f cm ahead of the drive axle (the rear lifts at %.1f or more)"),
					Margin, Contacts.Num(), ComAheadForView(Com), Config.RuleComMarginCm),
				MarginColour);
		}
	}

	// The frame's underside, a flat box under the chassis; in a crossing, coloured by its height over the edge while it is
	// over it.
	float		  Clear = 0.0f;
	const bool	  bOver = bCrossing && FrameOverEdge(Clear);
	const FColor  FrameColour = !bOver ? RammpGen1IdleFrameColour
		 : Clear > 2.0f				   ? Colours::Good
		 : Clear > 0.0f				   ? Colours::Warn
									   : Colours::Bad;
	const float	  HalfWidth = Config.FrameHalfWidthCm;
	const FVector Corners[4] = { Xf.TransformPosition(FVector(Config.FrameBackCm, -HalfWidth, Config.FrameBottomCm)),
		Xf.TransformPosition(FVector(Config.FrameFrontCm, -HalfWidth, Config.FrameBottomCm)),
		Xf.TransformPosition(FVector(Config.FrameFrontCm, HalfWidth, Config.FrameBottomCm)),
		Xf.TransformPosition(FVector(Config.FrameBackCm, HalfWidth, Config.FrameBottomCm)) };
	for (int32 k = 0; k < 4; ++k)
	{
		DrawDebugLine(World, Corners[k], Corners[(k + 1) % 4], FrameColour, false, -1.0f, RammpGen1OnTop, bOver ? 2.0f : 1.0f);
	}
	if (bOver && DebugView.bLabels)
	{
		RammpGen1Label(S, 0.25 * (Corners[0] + Corners[1] + Corners[2] + Corners[3]) - Up * 5.0,
			FString::Printf(TEXT("frame %.1f cm over the edge"), Clear), FrameColour);
	}

	// The chassis's attitude: straight up (grey), measured (cyan).
	const FVector Up0 = Xf.GetLocation() + FVector(0.0, 0.0, 40.0);
	DrawDebugLine(World, Up0, Up0 + FVector(0.0, 0.0, 60.0), RammpGen1VerticalColour, false, -1.0f, RammpGen1OnTop, 1.0f);
	DrawDebugDirectionalArrow(World, Up0, Up0 + Up * 60.0, 8.0f, Colours::Measured, false, -1.0f, RammpGen1OnTop, 2.0f);

	// What the crossing is doing: its status, the step it is in, what sets the speed.
	if (DebugView.bLabels)
	{
		ERammsControlTextSeverity Severity = ERammsControlTextSeverity::Info;
		FString					  Lines = TEXT("curb: ") + GetStatusLine(Severity);
		if (bCrossing)
		{
			Lines += FString::Printf(TEXT("\n%s, %.1f s in this step"), bBlended ? TEXT("rolling") : TEXT("stop and step"), PhaseSeconds);
			Lines += FString::Printf(TEXT("\nspeed %.1f / %.1f cm/s, set by %s"), S.ChassisMotion.IsFull() ? S.ChassisMotion.GetSpeed() : 0.0f, CommandedSpeedCmS(),
				*SpeedSetBy());
		}
		Lines += FString::Printf(TEXT("\npitch %+.1f  roll %+.1f deg"), FMath::RadiansToDegrees(PitchRad()), RammpGen1RollDeg(*Chassis));
		const FColor Colour = Severity == ERammsControlTextSeverity::Fault ? Colours::Bad
			: Severity == ERammsControlTextSeverity::Attention			   ? Colours::Warn
																		   : Colours::Text;
		RammpGen1Label(S, Up0 + FVector(0.0, 0.0, 75.0), Lines, Colour);
	}
}

// --- the colour key and the graphs: a HUD panel, or over the viewport ---------------------------------

bool URammsRammpGen1CurbController::WantsDebugKey() const
{
	return DebugView.bLabels && (DebugView.bTerrain || DebugView.bRobot);
}

void URammsRammpGen1CurbController::GetHUDPanels(TArray<FRammsHUDPanelDesc>& OutPanels) const
{
	if (!DebugState)
	{
		return;
	}
	const TWeakObjectPtr<const URammsRammpGen1CurbController> Self(this);
	RammsCurb::AddScopePanel(
		OutPanels, *this, *DebugState,
		[Self](const RammsCurb::FDebugPainter& P, float Width, float Height, bool bDraw) {
			const URammsRammpGen1CurbController* C = Self.Get();
			return C ? C->PaintDebugPanel(P, Width, Height, bDraw) : 0.0f;
		},
		// Shown while it has something to show: the graphs, or the colour key of a layer drawn in the world.
		[Self]() {
			const URammsRammpGen1CurbController* C = Self.Get();
			return C && (C->DebugView.bGraphs || C->WantsDebugKey());
		});
}

float URammsRammpGen1CurbController::PaintDebugPanel(const RammsCurb::FDebugPainter& P, float Width, float Height, bool bDraw) const
{
	namespace Colours = RammsCurb::ViewColours;
	if (!DebugState)
	{
		return 0.0f;
	}
	float		Y = 0.0f;
	const float X = 0.0f;
	if (WantsDebugKey())
	{
		TArray<RammsCurb::FKeyRow> Rows;
		if (DebugView.bTerrain)
		{
			Rows.Add({ TEXT("curb"), { { Colours::Near, TEXT("near level") }, { Colours::Far, TEXT("far level") }, { Colours::Edge, TEXT("edge") }, { Colours::Bad, TEXT("no curb") } } });
		}
		if (DebugView.bRobot)
		{
			Rows.Add({ TEXT("wheels"), { { Colours::Planted, TEXT("down") }, { Colours::Lifting, TEXT("lifting") }, { Colours::Clear, TEXT("up") }, { Colours::Planting, TEXT("lowering") }, { Colours::Loading, TEXT("pressed down again") } } });
			Rows.Add({ TEXT("support"), { { FColor::White, TEXT("wheels down, commanded height") }, { Colours::Good, TEXT("centre of mass 5 cm in") }, { Colours::Warn, TEXT("inside") }, { Colours::Bad, TEXT("outside") } } });
			Rows.Add({ TEXT("up"), { { Colours::Measured, TEXT("measured") }, { RammpGen1VerticalColour, TEXT("vertical") } } });
		}
		Y += RammsCurb::DrawKey(P, X, Y, Width, Rows, bDraw) + 4.0f;
	}
	if (DebugView.bGraphs)
	{
		using RammsCurb::EGraphSeries;
		using RammsCurb::FGraphLine;
		using RammsCurb::FGraphMark;
		constexpr int32 Charts = 6;
		const float		Row = P.LineHeight + 2.0f, Gap = 8.0f;
		// Six charts share whatever height the panel is given (48 each by default).
		const float H = Height > 0.0f ? FMath::Clamp((Height - Y - Charts * (Row + Gap)) / Charts, 30.0f, 400.0f) : 48.0f;
		if (bDraw)
		{
			const TArray<RammsCurb::FDebugGraphSample>& Samples = DebugState->Graph;
			const double								Span = DebugView.GraphSeconds;
			const double								TTo = Samples.Num() > 0 ? Samples.Last().T : 0.0;
			const double								TFrom = TTo - Span;
			const FLinearColor							Meas(Colours::Measured), Cmd(FLinearColor::White), Lim(Colours::Bad), Wants(Colours::Warn), Zero(0.4f, 0.4f, 0.45f);
			const FLinearColor							Front(RammpGen1FrontColour), Drive(RammpGen1DriveColour), Rear(RammpGen1RearColour);
			const FString								SpanText = FString::Printf(TEXT("last %.0f s"), Span);
			P.Text(X + Width - P.Width(SpanText), Y, SpanText, FLinearColor(0.5f, 0.55f, 0.6f));
			const FGraphMark ZeroMark[] = { { 0.0f, Zero } };
			float			 G = Y;

			// The chassis's speed, the rider's, the command and what the speed law wants (rolling).
			const FGraphLine SpeedLines[] = { { EGraphSeries::Speed, Meas, TEXT("chassis") }, { EGraphSeries::RiderSpeed, FLinearColor(Colours::Rider), TEXT("rider") },
				{ EGraphSeries::SpeedCommand, Cmd, TEXT("commanded") }, { EGraphSeries::SpeedTarget, Wants, TEXT("wants"), true } };
			G = RammsCurb::DrawGraph(P, X, G, Width, H, TEXT("ground speed (rider: at the seat)"), Samples, TFrom, Span, SpeedLines, ZeroMark, 10.0f, TEXT("cm/s")) + Gap;

			// The wheels at the tread: against the chassis's speed, they show slip (the drive wheels' less the translators').
			const FGraphLine WheelLines[] = { { EGraphSeries::DriveWheelSpeed, Drive, TEXT("drive") }, { EGraphSeries::DriveWheelCommand, Drive, nullptr, true },
				{ EGraphSeries::FrontHubSpeed, Front, TEXT("front omni") }, { EGraphSeries::FrontHubCommand, Front, nullptr, true } };
			G = RammsCurb::DrawGraph(P, X, G, Width, H, TEXT("wheels at the tread (thin: commanded)"), Samples, TFrom, Span, WheelLines, ZeroMark, 10.0f, TEXT("cm/s")) + Gap;

			const FGraphLine TiltLines[] = { { EGraphSeries::Tilt, Meas, TEXT("tilt") }, { EGraphSeries::Pitch, FLinearColor(RammpGen1PitchColour), TEXT("pitch") },
				{ EGraphSeries::Roll, FLinearColor(RammpGen1RollColour), TEXT("roll") } };
			G = RammsCurb::DrawGraph(P, X, G, Width, H, TEXT("base tilt (deg)"), Samples, TFrom, Span, TiltLines, ZeroMark, 2.0f, TEXT("deg")) + Gap;

			// The rear lifts only once the centre of mass is RuleComMarginCm ahead of the drive axle (dashed).
			const FGraphLine MarginLines[] = { { EGraphSeries::Margin, Meas, TEXT("stability") }, { EGraphSeries::ComAheadOfAxle, FLinearColor(RammpGen1ComColour), TEXT("CoM past axle") },
				{ EGraphSeries::FrameClearance, FLinearColor(RammpGen1FrameColour), TEXT("frame over edge") } };
			const FGraphMark MarginMarks[] = { { 0.0f, Lim }, { Config.RuleComMarginCm, FLinearColor(RammpGen1ComColour) } };
			G = RammsCurb::DrawGraph(P, X, G, Width, H, TEXT("margins (cm; dashed: the rear-lift rule)"), Samples, TFrom, Span, MarginLines, MarginMarks, 10.0f, TEXT("cm")) + Gap;

			const FGraphLine LiftLines[] = { { EGraphSeries::FrontLift, Front, TEXT("front") }, { EGraphSeries::FrontLiftCommand, Front, nullptr, true },
				{ EGraphSeries::DriveLift, Drive, TEXT("drive") }, { EGraphSeries::DriveLiftCommand, Drive, nullptr, true },
				{ EGraphSeries::RearLift, Rear, TEXT("rear") }, { EGraphSeries::RearLiftCommand, Rear, nullptr, true } };
			G = RammsCurb::DrawGraph(P, X, G, Width, H, TEXT("lifts (wheel height, + up; thin: commanded)"), Samples, TFrom, Span, LiftLines, ZeroMark, 4.0f, TEXT("cm")) + Gap;

			// Aft (dashed) for the rear's lift.
			const FGraphLine TranslatorLines[] = { { EGraphSeries::Translator, FLinearColor(RammpGen1TranslatorColour), TEXT("measured") },
				{ EGraphSeries::TranslatorCommand, Cmd, TEXT("commanded"), true } };
			const FGraphMark TranslatorMarks[] = { { 0.0f, Zero }, { Config.AftCm, Wants } };
			RammsCurb::DrawGraph(P, X, G, Width, H, TEXT("translators (+ forward; dashed: aft)"), Samples, TFrom, Span, TranslatorLines, TranslatorMarks, 5.0f, TEXT("cm"));
		}
		Y += Charts * (Row + H + Gap);
	}
	return Y;
}

void URammsRammpGen1CurbController::DrawDebugOverlay(UCanvas* Canvas, APlayerController* Player)
{
	if (!DebugState)
	{
		return;
	}
	RammsCurb::DrawScopeOverlay(*DebugState, *this, Canvas, Player, DebugView.bLabels, DebugView.bGraphs || WantsDebugKey(), DebugView.OverlayAnchor,
		[this](const RammsCurb::FDebugPainter& P, float Width, float Height, bool bDraw) { return PaintDebugPanel(P, Width, Height, bDraw); });
}
