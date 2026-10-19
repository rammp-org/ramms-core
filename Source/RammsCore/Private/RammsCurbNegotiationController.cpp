// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsCurbNegotiationController.h"

#include "Curb/RammsCurbLevel.h"
#include "Curb/RammsCurbRunStats.h"
#include "Curb/RammsCurbSpeedLaw.h"
#include "Curb/RammsCurbTerrainFit.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "Ramms4BarKinematics.h"
#include "Ramms5BarLinkageController.h"
#include "RammsDriveModeSelector.h"
#include "RammsRobotBaseComponent.h"
#include "RammsRobotControlSurfaceComponent.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsCurb, Log, All);

// A diagnostic switch, for comparing runs with and without the carrier term.
static TAutoConsoleVariable<int32> CVarRammsCurbCarrierSpin(
	TEXT("ramms.Curb.CarrierSpinFeedForward"),
	1,
	TEXT("Curb crossing: take the measured turn of the link each driven wheel hangs from off that wheel's speed\n")
		TEXT("command (1, the default), or leave it out (0). Without it, walking a 5-bar under load drives the chassis\n")
			TEXT("backwards: the wheel's joint is held at speed relative to a link that is itself turning."),
	ECVF_Default);

// A diagnostic switch, for planning with and without the corridor fallback.
static TAutoConsoleVariable<int32> CVarRammsCurbCorridor(
	TEXT("ramms.Curb.CorridorFallback"),
	1,
	TEXT("Curb crossing: where no one chassis height serves an axle's crossing, let the plan follow the corridor of heights\n")
		TEXT("every leg and the belly allow (1, the default), or refuse (0)."),
	ECVF_Default);

namespace
{
	const TCHAR* AxleName(ERammsCurbAxle Axle)
	{
		switch (Axle)
		{
			case ERammsCurbAxle::Front:
				return TEXT("front");
			case ERammsCurbAxle::Center:
				return TEXT("centre");
			default:
				return TEXT("rear");
		}
	}

	const TCHAR* PhaseName(ERammsCurbPhase Phase)
	{
		switch (Phase)
		{
			case ERammsCurbPhase::Idle:
				return TEXT("Idle");
			case ERammsCurbPhase::Approach:
				return TEXT("Approach");
			case ERammsCurbPhase::Prepare:
				return TEXT("Prepare");
			case ERammsCurbPhase::Lift:
				return TEXT("Lift");
			case ERammsCurbPhase::Shift:
				return TEXT("Shift");
			case ERammsCurbPhase::Advance:
				return TEXT("Advance");
			case ERammsCurbPhase::Plant:
				return TEXT("Plant");
			case ERammsCurbPhase::Settle:
				return TEXT("Settle");
			case ERammsCurbPhase::Done:
				return TEXT("Done");
			case ERammsCurbPhase::Aborted:
				return TEXT("Aborted");
			default:
				return TEXT("Faulted");
		}
	}

	bool PhaseNamesAxle(ERammsCurbPhase Phase)
	{
		return Phase == ERammsCurbPhase::Prepare || Phase == ERammsCurbPhase::Lift || Phase == ERammsCurbPhase::Shift
			|| Phase == ERammsCurbPhase::Advance || Phase == ERammsCurbPhase::Plant;
	}

	float StepToward(float Value, float Target, float MaxStep)
	{
		return Value + FMath::Clamp(Target - Value, -MaxStep, MaxStep);
	}

	FVector Horizontal(const FVector& V)
	{
		return FVector(V.X, V.Y, 0.0);
	}

	/** The most one tick moves a ramp. Commands reach the physics once a tick,
	 *  so a longer gap -- a hitch, a coarse external step -- is time the robot
	 *  spent on the commands it already had, and ramping across all of it at
	 *  once would be the jump the ramp is there to prevent. */
	constexpr float MaxRampStep = 0.1f;

	/** A wheel whose bottom is within this of the surface it should be on
	 *  (cm) is taken to be on it. */
	constexpr float ContactToleranceCm = 1.5f;

	/** A leg commanded this far (cm) from where it began has moved. */
	constexpr float LegMovedCm = 0.05f;

	/** The lifted-wheel clearance is sampled at this many points along the
	 *  wheel's rolling direction, across its diameter. */
	constexpr int32 SwingClearanceSamples = 9;

	/** A wheel's turn is measured between ticks no further apart than this
	 *  (s); across a longer gap the rate it last had is kept. */
	constexpr float MaxWheelTurnSampleSeconds = 0.25f;

	/** Within this (cm) of the commit point with the far depth still
	 *  unconfirmed, the robot is held there: once it has come to rest --
	 *  measured below FarDepthStoppedCmS (cm/s) -- it refuses; held
	 *  FarDepthRestSeconds (s) without coming to rest, it refuses anyway. */
	constexpr float FarDepthStopCm = 2.0f;
	constexpr float FarDepthStoppedCmS = 1.0f;
	constexpr float FarDepthRestSeconds = 2.0f;

	/** The plan checks each set of goals against the terrain every this far
	 *  (cm) along the chassis' travel, and this far beyond either end of the
	 *  stretch it holds over: the 4-bars carry their wheels a few cm fore and
	 *  aft as they rise and fall. */
	constexpr float SpanSampleCm = 2.0f;
	constexpr float SpanSlackCm = 3.0f;

	/** Where no pose serves an axle landing where the gait means it to, it
	 *  may land up to this much further on (cm), tried in these steps. */
	constexpr float MaxExtraLandingCm = 30.0f;
	constexpr float ExtraLandingStepCm = 2.0f;
	constexpr int32 NumExtraLandings = static_cast<int32>(MaxExtraLandingCm / ExtraLandingStepCm) + 1;

	/** A landing moved on comes at least this much chassis travel (cm)
	 *  before the next axle's braking point. */
	constexpr float NextLiftRoomCm = 1.0f;

	/** A support leg's command has caught up with its goal within this (cm). */
	constexpr float CaughtUpCm = 0.05f;

	/** The step detection traces this far above and below the lowest wheel
	 *  bottom, beyond the steepest the surfaces may climb or fall over the
	 *  scan and the largest step (cm). */
	constexpr double DetectTraceMarginCm = 20.0;
	constexpr double DetectBehindCm = 150.0;

	bool IsFinite(const FVector& V)
	{
		return FMath::IsFinite(V.X) && FMath::IsFinite(V.Y) && FMath::IsFinite(V.Z);
	}

	bool IsFinite(const FVector2D& V)
	{
		return FMath::IsFinite(V.X) && FMath::IsFinite(V.Y);
	}

	/** Every number a terrain carries is one, its edge vectors are unit and
	 *  at right angles. */
	bool TerrainIsSound(const FRammsCurbTerrain& T)
	{
		for (const FRammsCurbPlane* P : { &T.Near, &T.Far })
		{
			if (!IsFinite(P->Origin) || !IsFinite(P->Gradient))
			{
				return false;
			}
		}
		return IsFinite(T.EdgePoint) && IsFinite(T.EdgeNormal) && IsFinite(T.EdgeDirection)
			&& FMath::IsFinite(T.ApproachAngleDeg) && FMath::IsFinite(T.StepHeightCm) && IsFinite(T.StepHeightRangeCm)
			&& FMath::IsFinite(T.FarDepthCm) && FMath::IsNearlyEqual(T.EdgeNormal.Size(), 1.0, 1e-3)
			&& FMath::IsNearlyEqual(T.EdgeDirection.Size(), 1.0, 1e-3)
			&& FMath::Abs(FVector2D::DotProduct(T.EdgeNormal, T.EdgeDirection)) < 1e-3;
	}

	/** A 4-bar leg's wheel centre (chassis x-z, cm) from its live crank angle.
	 *  A reading a hair past an end of its range -- a servo pressed against
	 *  its stop -- is taken at that end (RammsCurb::MeasureFourBarCrank;
	 *  OutReading says what it read, bOutClamped that it was). False, with
	 *  why, when the authored linkage cannot close at that angle and the
	 *  reading is further out: the leg is then somewhere its spec does not
	 *  describe, and has no height to plan or check against. */
	bool MeasureFourBar(const URammsRobotBaseComponent& Base, const FRammsCurbLeg& Leg, FVector2D& OutWheelCenter,
		FString& OutReason, float* OutReading = nullptr, bool* bOutClamped = nullptr)
	{
		const float Crank = Base.GetMotorValue(Leg.FourBar.CrankMotor);
		float		Used = Crank;
		bool		bClamped = false;
		const bool	bValid = RammsCurb::MeasureFourBarCrank(Leg.FourBar, Crank, RammsCurb::CrankReadingToleranceRad,
			 OutWheelCenter, Used, bClamped);
		if (!bValid)
		{
			OutReason = FString::Printf(TEXT("leg '%s': its crank reads %.3f rad, where its 4-bar does not close (crank range "
											 "%.3f..%.3f; a reading up to %.3f rad past an end is taken at it)"),
				*Leg.Name.ToString(), Crank, Leg.FourBar.CrankRange.X, Leg.FourBar.CrankRange.Y,
				RammsCurb::CrankReadingToleranceRad);
		}
		if (OutReading)
		{
			*OutReading = Crank;
		}
		if (bOutClamped)
		{
			*bOutClamped = bValid && bClamped;
		}
		return bValid;
	}

	/** The centre wheels' fore/aft offsets (cm) a lift of Axle tries, best
	 *  first: a corner axle's from Preferred (the full stability margin) back
	 *  towards the centre of mass; the centre axle's, which has no stability
	 *  preference, outward from Preferred, 10 cm either way. */
	TArray<float> CentreCandidates(ERammsCurbAxle Axle, float Preferred, float CenterOfMassX)
	{
		TArray<float> Out;
		if (Axle == ERammsCurbAxle::Center)
		{
			for (int32 i = 0; i <= 40; ++i)
			{
				Out.Add(Preferred + 0.5f * ((i + 1) / 2) * ((i % 2) ? 1.0f : -1.0f));
			}
		}
		else
		{
			const int32 Count = FMath::Max(1, FMath::CeilToInt(FMath::Abs(Preferred - CenterOfMassX) / 0.5f));
			for (int32 i = 0; i <= Count; ++i)
			{
				Out.Add(FMath::Lerp(Preferred, CenterOfMassX, float(i) / Count));
			}
		}
		return Out;
	}

	/** A wheel's rolling direction (world xy) on a chassis heading Forward. */
	FVector2D RollDirectionOf(const FRammsCurbLeg& Leg, const FVector& Forward)
	{
		const FVector2D F = FVector2D(Forward.X, Forward.Y).GetSafeNormal();
		const double	Theta = FMath::DegreesToRadians(Leg.Wheel.RollDirectionDeg);
		// +Y is the robot's right: a positive roll direction turns towards it.
		const FVector2D Right(-F.Y, F.X);
		return (F * FMath::Cos(Theta) + Right * FMath::Sin(Theta)).GetSafeNormal();
	}
} // namespace

URammsCurbNegotiationController::URammsCurbNegotiationController()
	: Telemetry(MakePimpl<RammsCurb::FRunTelemetry>())
	, Drive(MakePimpl<RammsCurb::FDriveState>())
	, Level(MakePimpl<RammsCurb::FLevelState>())
{
	PrimaryComponentTick.bCanEverTick = true;
	// Same group as the robot base, and ahead of it: the base closes the wheel
	// velocity loops in its own tick and must see this frame's targets.
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void URammsCurbNegotiationController::BeginPlay()
{
	Super::BeginPlay();
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	if (URammsRobotBaseComponent* Base = EnsureBase())
	{
		Base->AddTickPrerequisiteComponent(this);
	}
}

URammsRobotBaseComponent* URammsCurbNegotiationController::EnsureBase() const
{
	if (!BaseComponent)
	{
		if (const AActor* Owner = GetOwner())
		{
			BaseComponent = Owner->FindComponentByClass<URammsRobotBaseComponent>();
		}
	}
	return BaseComponent;
}

bool URammsCurbNegotiationController::IsNegotiating() const
{
	return Phase != ERammsCurbPhase::Idle && Phase != ERammsCurbPhase::Done
		&& Phase != ERammsCurbPhase::Aborted && Phase != ERammsCurbPhase::Faulted;
}

ERammsCurbAxle URammsCurbNegotiationController::GetPhaseAxle() const
{
	return Steps.IsValidIndex(StepIndex) ? Steps[StepIndex].Axle : ERammsCurbAxle::Front;
}

TArray<float> URammsCurbNegotiationController::GetWheelProgress() const
{
	TArray<float> Out;
	for (const FLegState& State : LegState)
	{
		Out.Add(State.Progress);
	}
	return Out;
}

TArray<FVector2D> URammsCurbNegotiationController::GetLegHeights() const
{
	TArray<FVector2D> Out;
	for (const FLegState& State : LegState)
	{
		Out.Add(FVector2D(State.CmdZ, State.MeasuredZ));
	}
	return Out;
}

TArray<float> URammsCurbNegotiationController::GetWheelClearance() const
{
	TArray<float> Out;
	for (const FLegState& State : LegState)
	{
		Out.Add(State.WheelBottom);
	}
	return Out;
}

FRammsCurbRunStats URammsCurbNegotiationController::GetRunStats() const
{
	return Telemetry ? Telemetry->Stats.Get() : FRammsCurbRunStats();
}

FVector2D URammsCurbNegotiationController::GetDriveSpeed() const
{
	return FVector2D(DriveSpeed, Telemetry ? Telemetry->Motion.GetSpeed() : 0.0f);
}

TArray<FRammsCurbWheelStatus> URammsCurbNegotiationController::GetWheelStatus() const
{
	TArray<FRammsCurbWheelStatus> Out;
	if (Phase == ERammsCurbPhase::Idle || LegState.Num() != Legs.Num() || !Steps.IsValidIndex(StepIndex))
	{
		return Out;
	}
	const FStep& S = Steps[StepIndex];
	const float	 StepHeight = ActiveProfile.StepHeightCm;
	const bool	 bClimb = StepHeight > 0.0f;
	const bool	 bStopped = Phase == ERammsCurbPhase::Faulted || Phase == ERammsCurbPhase::Aborted;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FRammsCurbLeg& Leg = Legs[i];
		const FLegState&	 State = LegState[i];
		const FAxleGoal&	 Goal = S.Axles[static_cast<int32>(Leg.Axle)];
		const float			 R = Leg.WheelRadiusCm;

		FRammsCurbWheelStatus W;
		W.Leg = Leg.Name;
		W.PastEdgeCm = State.Progress;
		// The braking limits and the landing spot of the per-axle gait: a
		// loaded wheel stops FaceMargin short of the face (its tyre, a radius
		// ahead of its centre) or EdgeMargin short of the lip, and lands
		// past the far end of a skewed edge.
		W.WindowLo = bClimb ? -Tuning.FaceMargin - R : -Tuning.EdgeMargin;
		W.WindowHi = (bClimb ? Tuning.LandingOverlap : R + Tuning.DropClearance) + ActiveProfile.EdgeSkewCm;
		W.CmdZ = State.CmdZ;
		W.MeasZ = State.MeasuredZ;

		double Surface = 0.0;
		if (State.bHasWorld
			&& TraceSurface(State.WheelWorld, State.WheelWorld.Z + 2.0 * R, State.WheelWorld.Z - R - 100.0, Surface))
		{
			W.SurfaceZ = static_cast<float>(Surface);
			W.ClearanceCm = static_cast<float>(State.WheelWorld.Z - R - Surface);
			W.bContact = W.ClearanceCm <= ContactToleranceCm;
		}
		else
		{
			// Nothing under it the traces can see: the surface it follows, as
			// detected.
			W.SurfaceZ = static_cast<float>(State.WheelWorld.Z - R - State.WheelBottom);
			W.ClearanceCm = State.WheelBottom;
		}

		// The far side, or on its way there.
		W.Plane = Goal.bFar ? 1 : 0;

		const bool bStepAxle = PhaseNamesAxle(Phase) && S.Axle == Leg.Axle;
		switch (Goal.Goal)
		{
			case ELegGoal::Swing:
				W.State = bStepAxle && Phase == ERammsCurbPhase::Lift ? ERammsCurbWheelState::Lifting
																	  : ERammsCurbWheelState::Clear;
				break;
			case ELegGoal::Support:
				W.State = bStepAxle && Phase == ERammsCurbPhase::Plant
					? (W.bContact ? ERammsCurbWheelState::Loading : ERammsCurbWheelState::Planting)
					: ERammsCurbWheelState::Planted;
				break;
			default: // Hold: the stance the robot drove up in
				W.State = W.bContact ? ERammsCurbWheelState::Planted : ERammsCurbWheelState::Held;
				break;
		}
		if (bStopped && W.State != ERammsCurbWheelState::Planted)
		{
			// A crossing that stopped leaves every leg on its last target.
			W.State = ERammsCurbWheelState::Held;
		}
		Out.Add(W);
	}
	return Out;
}

// --- set-up ----------------------------------------------------------------------

bool URammsCurbNegotiationController::ResolveLegs(FString& OutReason)
{
	AActor* Owner = GetOwner();
	if (!Owner || !EnsureBase())
	{
		OutReason = TEXT("no RammsRobotBaseComponent on this robot");
		return false;
	}

	FiveBarReachCache.Reset();
	int32 PerAxle[3] = { 0, 0, 0 };
	int32 CornerPivots[3] = { 0, 0, 0 };
	int32 CornerSides[2] = { 0, 0 }; // corner 4-bars at +Y, at -Y
	LegState.SetNum(Legs.Num());
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FRammsCurbLeg& Leg = Legs[i];
		FLegState&			 State = LegState[i];
		State = FLegState();
		++PerAxle[static_cast<int32>(Leg.Axle)];
		if (Leg.Wheel.MotorId.IsNone())
		{
			OutReason = FString::Printf(TEXT("leg '%s' names no wheel motor"), *Leg.Name.ToString());
			return false;
		}
		if (Leg.Kind == ERammsCurbLegKind::FourBar)
		{
			if (Leg.FourBar.CrankMotor.IsNone())
			{
				OutReason = FString::Printf(TEXT("leg '%s' names no crank motor"), *Leg.Name.ToString());
				return false;
			}
			State.HeightRange = URamms4BarKinematics::GetHeightRange(Leg.FourBar);
			if (State.HeightRange.X >= State.HeightRange.Y)
			{
				OutReason = FString::Printf(TEXT("leg '%s': its 4-bar spec closes nowhere in its crank range"),
					*Leg.Name.ToString());
				return false;
			}
			// The crank solver brackets a height between the two ends of
			// CrankRange, so the leg has to close at both ends and move its wheel
			// one way only in between. A range authored past where the linkage
			// assembles would send every height command to one end stop.
			const int32 Samples = 64;
			double		PrevZ = 0.0;
			double		Trend = 0.0;
			for (int32 s = 0; s <= Samples; ++s)
			{
				bool		 bValid = false;
				const float	 Crank = FMath::Lerp(Leg.FourBar.CrankRange.X, Leg.FourBar.CrankRange.Y, float(s) / Samples);
				const double Z = URamms4BarKinematics::ComputeWheelCenter(Leg.FourBar, Crank, bValid).Y;
				if (!bValid)
				{
					OutReason = FString::Printf(TEXT("leg '%s': its 4-bar does not close at crank %.3f rad, inside "
													 "its crank range %.3f..%.3f"),
						*Leg.Name.ToString(), Crank, Leg.FourBar.CrankRange.X, Leg.FourBar.CrankRange.Y);
					return false;
				}
				if (s > 0)
				{
					Trend = Trend != 0.0 ? Trend : FMath::Sign(Z - PrevZ);
					if ((Z - PrevZ) * Trend < -1e-3)
					{
						OutReason = FString::Printf(TEXT("leg '%s': its wheel height is not monotonic in the "
														 "crank across its crank range"),
							*Leg.Name.ToString());
						return false;
					}
				}
				PrevZ = Z;
			}
			++CornerPivots[static_cast<int32>(Leg.Axle)];
			if (Leg.Axle != ERammsCurbAxle::Center)
			{
				++(IsPlusY(Leg) ? CornerSides[0] : CornerSides[1]);
			}
		}
		else
		{
			for (UActorComponent* Component : Owner->GetComponents())
			{
				URamms5BarLinkageController* FiveBar = Cast<URamms5BarLinkageController>(Component);
				if (FiveBar && FiveBar->GetFName() == Leg.FiveBarComponent)
				{
					State.FiveBar = FiveBar;
					break;
				}
			}
			if (!State.FiveBar)
			{
				OutReason = FString::Printf(TEXT("leg '%s': no 5-bar controller named '%s'"),
					*Leg.Name.ToString(), *Leg.FiveBarComponent.ToString());
				return false;
			}
			bool bValid = false;
			State.HeightRange = State.FiveBar->GetReachableExtent(true, bValid);
		}
	}
	for (int32 Axle = 0; Axle < 3; ++Axle)
	{
		if (PerAxle[Axle] == 0)
		{
			OutReason = FString::Printf(TEXT("no legs on the %s axle"), AxleName(static_cast<ERammsCurbAxle>(Axle)));
			return false;
		}
	}
	// The chassis is located from the corner crank pivots, which are fixed on
	// it; that needs pivots both ahead of and behind the middle.
	if (CornerPivots[0] == 0 || CornerPivots[2] == 0)
	{
		OutReason = TEXT("need 4-bar legs on both the front and rear axles to measure the chassis");
		return false;
	}
	// Roll -- and with it the tilt safety check -- comes from corner pivots on
	// either side, told apart by the sign of their wheel's Position.Y.
	if (CornerSides[0] == 0 || CornerSides[1] == 0)
	{
		OutReason = TEXT("need corner 4-bar legs on both sides (Wheel.Position.Y > 0 and < 0) to measure roll");
		return false;
	}
	return true;
}

// --- measurement ---------------------------------------------------------------

bool URammsCurbNegotiationController::Measure(FString& OutReason)
{
	const TCHAR* const Unlocated = TEXT("the robot's wheels and pivots cannot be located (is the simulation running?)");

	URammsRobotBaseComponent* Base = EnsureBase();
	if (!Base || LegState.Num() != Legs.Num())
	{
		OutReason = Unlocated;
		return false;
	}

	// Chassis attitude from the crank pivots.
	FVector	  FrontW = FVector::ZeroVector, RearW = FVector::ZeroVector;
	FVector	  PlusW = FVector::ZeroVector, MinusW = FVector::ZeroVector, AllW = FVector::ZeroVector;
	FVector2D FrontL = FVector2D::ZeroVector, RearL = FVector2D::ZeroVector, AllL = FVector2D::ZeroVector;
	double	  AllZ = 0.0;
	double	  AllY = 0.0;
	int32	  NFront = 0, NRear = 0, NPlus = 0, NMinus = 0, NAll = 0;
	for (const FRammsCurbLeg& Leg : Legs)
	{
		if (Leg.Kind != ERammsCurbLegKind::FourBar || Leg.Axle == ERammsCurbAxle::Center)
		{
			continue;
		}
		FTransform Pivot;
		if (!Base->GetMotorTransform(Leg.FourBar.CrankMotor, Pivot))
		{
			OutReason = Unlocated;
			return false;
		}
		const FVector P = Pivot.GetLocation();
		if (Leg.Axle == ERammsCurbAxle::Front)
		{
			FrontW += P;
			FrontL += Leg.FourBar.CrankPivot;
			++NFront;
		}
		else
		{
			RearW += P;
			RearL += Leg.FourBar.CrankPivot;
			++NRear;
		}
		if (IsPlusY(Leg))
		{
			PlusW += P;
			++NPlus;
		}
		else
		{
			MinusW += P;
			++NMinus;
		}
		AllZ += P.Z;
		AllW += P;
		AllY += Leg.Wheel.Position.Y;
		AllL += Leg.FourBar.CrankPivot;
		++NAll;
	}
	if (NFront == 0 || NRear == 0)
	{
		OutReason = Unlocated;
		return false;
	}
	FrontW /= NFront;
	RearW /= NRear;
	FrontL /= NFront;
	RearL /= NRear;
	AllL /= NAll;
	AllZ /= NAll;

	const FVector Along = FrontW - RearW;
	const double  Pitch =
		FMath::Atan2(Along.Z, Horizontal(Along).Size()) - FMath::Atan2(FrontL.Y - RearL.Y, FrontL.X - RearL.X);
	const FVector Forward = Horizontal(Along).GetSafeNormal();
	FVector		  Side = FVector::ZeroVector;
	double		  Roll = 0.0;
	if (NPlus > 0 && NMinus > 0)
	{
		const FVector Across = PlusW / NPlus - MinusW / NMinus;
		Roll = FMath::Atan2(Across.Z, Horizontal(Across).Size());
		Side = Horizontal(Across).GetSafeNormal();
	}
	MeasuredPitchDeg = static_cast<float>(FMath::RadiansToDegrees(Pitch));
	MeasuredRollDeg = static_cast<float>(FMath::RadiansToDegrees(Roll));
	const double OriginZ = AllZ - (AllL.X * FMath::Sin(Pitch) + AllL.Y * FMath::Cos(Pitch));
	MeasuredOriginZ = OriginZ;
	MeasuredBaseHeight = static_cast<float>(OriginZ - GroundZ - TerrainReferenceCmd);
	// The chassis origin over the ground: the pivots' middle, less where it
	// sits in the chassis frame (fore/aft from their crank pivots, across from
	// their wheels, which share their side).
	{
		const FVector2D Mid(AllW.X / NAll, AllW.Y / NAll);
		const FVector2D F(Forward.X, Forward.Y);
		const FVector2D Right(-F.Y, F.X);
		ChassisOriginXY = Mid - F * AllL.X - Right * (AllY / NAll);
	}

	if (!IsNegotiating())
	{
		Heading = Forward;
	}
	ChassisForward = Forward;
	// Signed in the robot's own frame -- positive when the path lies towards its
	// +Y wheels -- so the correction matches CommandWheels' use of Position.Y
	// whichever way round the legs' lateral axis was authored.
	HeadingErrorRad = Side.IsZero()
		? 0.0f
		: static_cast<float>(FMath::Atan2(FVector::DotProduct(Heading, Side), FVector::DotProduct(Heading, Forward)));

	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FRammsCurbLeg& Leg = Legs[i];
		FLegState&			 State = LegState[i];
		if (Leg.Kind == ERammsCurbLegKind::FourBar)
		{
			FVector2D WheelCenter = FVector2D::ZeroVector;
			float	  Reading = 0.0f;
			bool	  bClamped = false;
			if (!MeasureFourBar(*Base, Leg, WheelCenter, OutReason, &Reading, &bClamped))
			{
				return false;
			}
			if (bClamped && !State.bCrankClampLogged)
			{
				// Once a crossing: a crank held a hair past its stop reads so on
				// every tick until it moves.
				State.bCrankClampLogged = true;
				const FString Line = FString::Printf(TEXT("leg '%s': its crank reads %.3f rad, past its range %.3f..%.3f; "
														  "measured at the end of the range"),
					*Leg.Name.ToString(), Reading, Leg.FourBar.CrankRange.X, Leg.FourBar.CrankRange.Y);
				UE_LOG(LogRammsCurb, Warning, TEXT("[Curb] '%s' %s"), *GetNameSafe(GetOwner()), *Line);
				if (IsNegotiating())
				{
					StepLog.Add(FString::Printf(TEXT("t=%5.2fs %s"), Elapsed, *Line));
				}
				else
				{
					MeasureNotes.Add(Line);
				}
			}
			State.MeasuredX = static_cast<float>(WheelCenter.X);
			State.MeasuredZ = static_cast<float>(WheelCenter.Y);
		}
		else if (State.FiveBar)
		{
			// As for a 4-bar: joint angles that do not close the linkage put
			// the wheel nowhere, and an estimate of where is no basis for
			// settling a step or checking a limit.
			bool			bClosed = false;
			const FVector2D Endpoint = State.FiveBar->GetCurrentEndpointChecked(bClosed);
			if (!bClosed)
			{
				OutReason = FString::Printf(TEXT("leg '%s': its 5-bar's joint angles do not close the linkage"),
					*Leg.Name.ToString());
				return false;
			}
			State.MeasuredX = static_cast<float>(Endpoint.X);
			State.MeasuredZ = static_cast<float>(Endpoint.Y);
		}
		// Every wheel, not most of them: the braking limits and each axle's
		// progress are taken over the wheels that were measured, so one that
		// drops out would leave its mate to finish the axle alone.
		FTransform Wheel;
		if (!Base->GetMotorTransform(Leg.Wheel.MotorId, Wheel))
		{
			OutReason = FString::Printf(TEXT("the '%s' wheel cannot be located"), *Leg.Name.ToString());
			return false;
		}
		State.bHasWorld = true;
		const FVector W = Wheel.GetLocation();
		State.WheelWorld = W;
		State.WheelRotation = Wheel.GetRotation();
		State.WheelAngle = Base->GetMotorValue(Leg.Wheel.MotorId);
		State.WheelRate = Base->GetMotorVelocity(Leg.Wheel.MotorId);
		State.Progress = static_cast<float>(FVector::DotProduct(Horizontal(W - EdgePoint), Heading));
		// The wheel's bottom over the surface it follows: UpdateWheelSurfaces,
		// which needs the step's goals.
	}
	return true;
}

// --- planning --------------------------------------------------------------------

float URammsCurbNegotiationController::GoalHeight(int32 LegIndex, const FAxleGoal& Goal, float BaseHeight, float Offset) const
{
	const FRammsCurbLeg& Leg = Legs[LegIndex];
	switch (Goal.Goal)
	{
		case ELegGoal::Support:
			return Goal.Level + Leg.WheelRadiusCm - BaseHeight - Tuning.PlantPreload
				- (Leg.Drive == ERammsCurbWheelDrive::Main ? Tuning.MainWheelPreload : 0.0f) + Offset;
		case ELegGoal::Hold:
			return LegState[LegIndex].InitialZ;
		default:
			return Goal.Level + Leg.WheelRadiusCm + Tuning.SwingClearance - BaseHeight + Offset;
	}
}

float URammsCurbNegotiationController::PlannedGoalHeight(int32 StepIdx, int32 LegIndex) const
{
	const FStep&	 S = Steps[StepIdx];
	const FAxleGoal& Goal = S.Axles[static_cast<int32>(Legs[LegIndex].Axle)];
	const float		 Offset = S.Span.Offset.IsValidIndex(LegIndex) ? static_cast<float>(0.5 * (S.Span.Offset[LegIndex].X + S.Span.Offset[LegIndex].Y))
																   : 0.0f;
	return GoalHeight(LegIndex, Goal, S.BaseHeight, Offset);
}

float URammsCurbNegotiationController::CommandedGoalHeight(int32 LegIndex, const FAxleGoal& Goal) const
{
	const FLegState& State = LegState[LegIndex];
	if (Goal.Goal == ELegGoal::Hold)
	{
		return State.InitialZ;
	}
	// Over the terrain as it is under the wheel now: a support leg on the
	// surface under its own wheel, a lifted one at the height that clears its
	// whole crossing (UpdateWheelSurfaces).
	float Z = GoalHeight(LegIndex, Goal, BaseHeightCmd + CorridorShiftCmd, State.TerrainOffset);
	if (Goal.Goal == ELegGoal::Support)
	{
		return Z + State.AttitudeDz;
	}
	// A lifted wheel is carried no lower than its leg reaches: past a
	// descent's lip the terrain under it may ask for less than that, and the
	// plan only made sure the leg reaches the most it asks (LegReachable).
	if (Legs[LegIndex].Kind == ERammsCurbLegKind::FourBar)
	{
		Z = FMath::Max(Z, State.HeightRange.X + Tuning.LegRangeMargin);
	}
	else if (State.FiveBar)
	{
		const FVector2D Reach = FiveBarReachAt(LegIndex, State.CmdX);
		if (Reach.X <= Reach.Y)
		{
			Z = FMath::Max(Z, static_cast<float>(Reach.X) + Tuning.LegRangeMargin);
		}
	}
	// Clearance is over the ground, not the chassis. Two axles carrying the
	// robot let it pitch a degree or three, and at the far end of a 90 cm
	// wheelbase that is the whole clearance; a slope across it rolls it too.
	// A lifted wheel the chassis has tipped towards the ground is lifted
	// further -- as far as the leg reaches, so the command never asks for more
	// than the leg can be measured doing.
	const float Wanted = Z + RammsCurb::SwingTiltRaise(State.ModelX, Legs[LegIndex].Wheel.Position.Y, FilteredPitchRad, FilteredRollRad);
	if (Wanted <= Z)
	{
		return Z;
	}
	if (Legs[LegIndex].Kind == ERammsCurbLegKind::FourBar)
	{
		return FMath::Max(Z, FMath::Min(Wanted, State.HeightRange.Y - Tuning.LegRangeMargin));
	}
	if (!State.FiveBar)
	{
		return Z;
	}
	auto Reaches = [&](float Height) {
		bool bReachable = false;
		State.FiveBar->SolveTarget(FVector2D(State.CmdX, Height + Tuning.LegRangeMargin), bReachable);
		return bReachable;
	};
	if (Reaches(Wanted))
	{
		return Wanted;
	}
	float Lo = Z, Hi = Wanted;
	for (int32 i = 0; i < 10; ++i)
	{
		const float Mid = 0.5f * (Lo + Hi);
		(Reaches(Mid) ? Lo : Hi) = Mid;
	}
	return Lo;
}

bool URammsCurbNegotiationController::LegReachable(int32 LegIndex, const FAxleGoal& Goal, float BaseHeight,
	float X, const FVector2D& Offset, FString* OutWhy) const
{
	const FRammsCurbLeg& Leg = Legs[LegIndex];
	const FLegState&	 State = LegState[LegIndex];
	if (Goal.Goal == ELegGoal::Hold)
	{
		return true; // where it already is
	}
	// The terrain moves the leg over Offset as the robot goes; it must reach
	// all of it.
	const float Z = GoalHeight(LegIndex, Goal, BaseHeight);
	const float ZLo = Z + static_cast<float>(Offset.X);
	const float ZHi = Z + static_cast<float>(Offset.Y);
	const float Margin = Tuning.LegRangeMargin;
	auto		Heights = [&]() {
		   return FMath::IsNearlyEqual(ZLo, ZHi, 0.05f) ? FString::Printf(TEXT("%.1f cm"), ZLo)
															: FString::Printf(TEXT("%.1f..%.1f cm"), ZLo, ZHi);
	};
	if (Goal.Goal == ELegGoal::Swing)
	{
		// A lifted wheel need only be carried high enough: the highest the
		// terrain asks of it. Where it asks less than the leg's lowest reach --
		// a wheel past a descent's lip, over the lower side -- it is carried at
		// that lowest reach instead, higher than it need be (CommandedGoalHeight).
		const FVector2D Reach = Leg.Kind == ERammsCurbLegKind::FourBar ? State.HeightRange : FiveBarReachAt(LegIndex, X);
		if (Reach.X > Reach.Y || FMath::Max(ZHi, static_cast<float>(Reach.X) + Margin) > Reach.Y - Margin)
		{
			if (OutWhy)
			{
				*OutWhy = FString::Printf(TEXT("leg '%s' (lifted) would need its wheel at %.1f cm; it reaches %.1f..%.1f%s"),
					*Leg.Name.ToString(), ZHi, Reach.X, Reach.Y,
					Leg.Kind == ERammsCurbLegKind::FiveBar ? *FString::Printf(TEXT(" at %.1f cm fore/aft"), X) : TEXT(""));
			}
			return false;
		}
		return true;
	}
	if (Leg.Kind == ERammsCurbLegKind::FourBar)
	{
		if (ZLo < State.HeightRange.X + Margin || ZHi > State.HeightRange.Y - Margin)
		{
			if (OutWhy)
			{
				*OutWhy = FString::Printf(TEXT("leg '%s' would need its wheel at %s; it reaches %.1f..%.1f"),
					*Leg.Name.ToString(), *Heights(), State.HeightRange.X, State.HeightRange.Y);
			}
			return false;
		}
		return true;
	}
	if (!State.FiveBar)
	{
		return false;
	}
	// The heights a 5-bar reaches at one fore/aft position are one stretch,
	// so its ends (with the margin) stand for everything between.
	for (const float Dz : { ZLo, ZLo - Margin, ZHi, ZHi + Margin })
	{
		bool bReachable = false;
		State.FiveBar->SolveTarget(FVector2D(X, Dz), bReachable);
		if (!bReachable)
		{
			if (OutWhy)
			{
				*OutWhy = FString::Printf(TEXT("leg '%s' cannot put its wheel at (%.1f, %.1f) cm%s"),
					*Leg.Name.ToString(), X, Dz,
					FMath::IsNearlyEqual(ZLo, ZHi, 0.05f) ? TEXT("") : *FString::Printf(TEXT(" (the terrain asks %s)"), *Heights()));
			}
			return false;
		}
	}
	return true;
}

bool URammsCurbNegotiationController::IsReachable(const FAxleGoal Axles[3], float BaseHeight,
	const TArray<float>& LegX, float ChassisFloor, const FGoalSpan* Span, FString* OutWhy) const
{
	if (BaseHeight < ChassisFloor - KINDA_SMALL_NUMBER)
	{
		if (OutWhy)
		{
			*OutWhy = FString::Printf(TEXT("the chassis must stay above %.1f cm to clear the step"), ChassisFloor);
		}
		return false;
	}
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const float		X = LegX.IsValidIndex(i) ? LegX[i] : 0.0f;
		const FVector2D Offset = Span && Span->Offset.IsValidIndex(i) ? Span->Offset[i] : FVector2D::ZeroVector;
		if (!LegReachable(i, Axles[static_cast<int32>(Legs[i].Axle)], BaseHeight, X, Offset, OutWhy))
		{
			return false;
		}
	}
	return true;
}

bool URammsCurbNegotiationController::PlanLift(ERammsCurbAxle Axle, TArrayView<const FGoalSet> Sets, float ChassisFloor,
	float PreferredHeight, float PreferredCenterX, float MinShift, float& OutHeight, TArray<float>& OutLegX, float& OutShift,
	FString& OutReason) const
{
	auto OffsetOf = [](const FGoalSet& Set, int32 i) {
		return Set.Span && Set.Span->Offset.IsValidIndex(i) ? Set.Span->Offset[i] : FVector2D::ZeroVector;
	};
	const TArray<float> Candidates = CentreCandidates(Axle, PreferredCenterX, Tuning.CenterOfMassX);
	const float			Required = Axle == ERammsCurbAxle::Center ? 0.0f : MinShift;
	const float			Wanted = Axle == ERammsCurbAxle::Center ? 0.0f : FMath::Abs(PreferredCenterX - Tuning.CenterOfMassX);

	// Each centre wheel is placed on its own. The two 5-bars need not reach the
	// same offsets -- on the linkage chassis they are mirror images, one
	// reaching far forward and the other far aft -- and stability only asks
	// that the line between their contacts passes the centre of mass on the
	// side away from the lifted axle: that their MEAN does.
	auto Place = [&](float H, TArray<float>& LegX, float& Shift) {
		LegX.Init(0.0f, Legs.Num());
		float Sum = 0.0f;
		int32 Count = 0;
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			const int32 A = static_cast<int32>(Legs[i].Axle);
			if (Legs[i].Kind != ERammsCurbLegKind::FiveBar)
			{
				for (const FGoalSet& Set : Sets)
				{
					if (!LegReachable(i, Set.Goals[A], H, 0.0f, OffsetOf(Set, i), nullptr))
					{
						return false;
					}
				}
				continue;
			}
			bool bPlaced = false;
			for (const float X : Candidates)
			{
				bool bAll = true;
				for (const FGoalSet& Set : Sets)
				{
					bAll = bAll && LegReachable(i, Set.Goals[A], H, X, OffsetOf(Set, i), nullptr);
				}
				if (bAll)
				{
					LegX[i] = X;
					Sum += X;
					++Count;
					bPlaced = true;
					break;
				}
			}
			if (!bPlaced)
			{
				return false;
			}
		}
		const float Mean = Count ? Sum / Count : Tuning.CenterOfMassX;
		// Signed shift towards the side the robot must not tip away from.
		Shift = Axle == ERammsCurbAxle::Front ? Mean - Tuning.CenterOfMassX
			: Axle == ERammsCurbAxle::Rear	  ? Tuning.CenterOfMassX - Mean
											  : 0.0f;
		return Shift >= Required - KINDA_SMALL_NUMBER;
	};

	const float Step = 0.25f;
	const float Top = ChassisFloor + 60.0f;
	const float Keep = 1.0f; // stay this far from the edge of the feasible band when there is room
	float		BestCost = TNumericLimits<float>::Max();
	for (float H = ChassisFloor; H <= Top; H += Step)
	{
		TArray<float> LegX;
		float		  Shift = 0.0f;
		if (!Place(H, LegX, Shift))
		{
			continue;
		}
		// Prefer the chassis height nearest the last one, the full stability
		// margin, and room either side: a plan on the very edge of a leg's
		// travel leaves nothing for the chassis settling under load.
		TArray<float> Scratch;
		float		  Unused = 0.0f;
		const bool	  bRoomy = Place(H - Keep, Scratch, Unused) && Place(H + Keep, Scratch, Unused);
		const float	  Cost = FMath::Abs(H - PreferredHeight) + 2.0f * FMath::Max(0.0f, Wanted - Shift) + (bRoomy ? 0.0f : 100.0f);
		if (Cost < BestCost)
		{
			BestCost = Cost;
			OutHeight = H;
			OutLegX = LegX;
			OutShift = Shift;
		}
	}
	if (BestCost < TNumericLimits<float>::Max())
	{
		return true;
	}

	// Say what stopped it, at the least shift the stability tuning accepts.
	FString		  Why;
	const float	  H = FMath::Max(PreferredHeight, ChassisFloor);
	TArray<float> LegX;
	LegX.Init(Tuning.CenterOfMassX + FMath::Sign(PreferredCenterX - Tuning.CenterOfMassX) * Required, Legs.Num());
	bool bAllReachable = true;
	for (const FGoalSet& Set : Sets)
	{
		bAllReachable = bAllReachable && IsReachable(Set.Goals, H, LegX, ChassisFloor, Set.Span, &Why);
	}
	if (bAllReachable)
	{
		Why = FString::Printf(TEXT("the centre wheels cannot shift %.1f cm past the centre of mass"), Required);
	}

	// What binds: the chassis heights (plan) each leg allows over the stretch
	// each set of goals holds for, and the belly's floor -- the highest least
	// against the lowest most.
	float	Lo = ChassisFloor, Hi = TNumericLimits<float>::Max();
	FString LoWhy = FString::Printf(TEXT("the belly needs the chassis at %.1f cm or more"), ChassisFloor), HiWhy;
	auto	GoalName = [](const FAxleGoal& Goal) {
		   return Goal.Goal == ELegGoal::Swing ? TEXT("lifted") : (Goal.bFar ? TEXT("on the far side") : TEXT("on the near side"));
	};
	for (const FGoalSet& Set : Sets)
	{
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			const FAxleGoal& Goal = Set.Goals[static_cast<int32>(Legs[i].Axle)];
			if (Goal.Goal == ELegGoal::Hold)
			{
				continue;
			}
			float HMin = TNumericLimits<float>::Max(), HMax = -TNumericLimits<float>::Max();
			for (float HH = ChassisFloor - 40.0f; HH <= Top; HH += Step)
			{
				bool bOk = false;
				if (Legs[i].Kind == ERammsCurbLegKind::FiveBar)
				{
					for (const float X : Candidates)
					{
						if (LegReachable(i, Goal, HH, X, OffsetOf(Set, i), nullptr))
						{
							bOk = true;
							break;
						}
					}
				}
				else
				{
					bOk = LegReachable(i, Goal, HH, 0.0f, OffsetOf(Set, i), nullptr);
				}
				if (bOk)
				{
					HMin = FMath::Min(HMin, HH);
					HMax = FMath::Max(HMax, HH);
				}
			}
			if (HMin > HMax)
			{
				continue;
			}
			if (HMin > Lo)
			{
				Lo = HMin;
				LoWhy = FString::Printf(TEXT("leg '%s' (%s) needs the chassis at %.1f cm or more"), *Legs[i].Name.ToString(), GoalName(Goal), HMin);
			}
			if (HMax < Hi)
			{
				Hi = HMax;
				HiWhy = FString::Printf(TEXT("leg '%s' (%s) needs it at %.1f cm or less"), *Legs[i].Name.ToString(), GoalName(Goal), HMax);
			}
		}
	}
	if (Lo > Hi)
	{
		Why = FString::Printf(TEXT("%s, but %s (%s)"), *LoWhy, *HiWhy, *Why);
	}
	OutReason = FString::Printf(TEXT("no chassis height lets the %s axle cross: %s"), AxleName(Axle), *Why);
	return false;
}

bool URammsCurbNegotiationController::BuildPlan(const FRammsCurbProfile& Profile, FString& OutReason)
{
	// The profile first, before anything is resolved or measured. ClampMin
	// binds only the editor: a profile built in C++, Blueprint or Python can
	// carry anything, and an edge behind the front wheels would end the
	// approach at once and lift for a step that is not there.
	const float StepHeight = Profile.StepHeightCm;
	const bool	bTerrain = Profile.Terrain.bValid;
	if (!FMath::IsFinite(StepHeight) || !FMath::IsFinite(Profile.EdgeDistanceCm) || !FMath::IsFinite(Profile.EdgeSkewCm)
		|| !FMath::IsFinite(Profile.ApproachAngleDeg))
	{
		OutReason = TEXT("the step profile is not a number");
		return false;
	}
	if (bTerrain && !TerrainIsSound(Profile.Terrain))
	{
		OutReason = TEXT("the step profile's terrain is not a number, or its edge vectors are not unit and square");
		return false;
	}
	if (Profile.EdgeDistanceCm < 0.0f)
	{
		OutReason = TEXT("EdgeDistanceCm cannot be negative: the edge must be ahead of the front wheels");
		return false;
	}
	if (Profile.EdgeSkewCm < 0.0f)
	{
		OutReason = TEXT("EdgeSkewCm cannot be negative");
		return false;
	}
	if (!PerAxleAccepts(Profile, OutReason))
	{
		return false;
	}
	if (!ResolveLegs(OutReason))
	{
		return false;
	}
	if (FMath::Abs(StepHeight) < Tuning.MinStepHeight)
	{
		OutReason = FString::Printf(TEXT("a %.1f cm step is no step; just drive"), StepHeight);
		return false;
	}
	if (FMath::Abs(StepHeight) > Tuning.MaxStepHeight)
	{
		OutReason = FString::Printf(TEXT("a %.1f cm step exceeds the %.1f cm limit"), StepHeight, Tuning.MaxStepHeight);
		return false;
	}

	// Ground first: the lowest wheel bottom, which the plan's chassis heights
	// are reckoned from. No step is planned while this measures, so every
	// wheel counts as on the near side.
	GroundZ = 0.0;
	TerrainReference = 0.0;
	TerrainReferenceCmd = 0.0;
	EdgePoint = FVector::ZeroVector;
	Phase = ERammsCurbPhase::Idle;
	StepIndex = INDEX_NONE;
	PlanStepHeight = StepHeight;
	ActiveTerrain = FRammsCurbTerrain();
	MeasureNotes.Reset();
	if (Level)
	{
		Level->Reset();
	}
	if (!Measure(OutReason))
	{
		return false;
	}
	double Lowest = TNumericLimits<double>::Max();
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		if (LegState[i].bHasWorld)
		{
			Lowest = FMath::Min(Lowest, LegState[i].WheelWorld.Z - Legs[i].WheelRadiusCm);
		}
	}
	GroundZ = Lowest;
	if (!Measure(OutReason))
	{
		return false;
	}

	// The terrain the crossing runs on: the one detected, or the level ground
	// and square edge a profile without one stands for (its edge at the near
	// end of a skewed one, where progress is reckoned from). Every leg takes
	// its height from it, each from the surface under its own wheel.
	if (!PlaceEdge(Profile, OutReason))
	{
		return false;
	}
	ActiveTerrain = bTerrain ? Profile.Terrain
							 : RammsCurb::LevelTerrain(GroundZ, StepHeight, FVector2D(EdgePoint.X, EdgePoint.Y), FVector2D(Heading.X, Heading.Y));
	UpdateWheelSurfaces(0.0f);
	TerrainReferenceCmd = TerrainReference;
	UpdateWheelSurfaces(0.0f);

	// The gait starts with every wheel over the near surface. A robot already
	// standing across a step -- a retry after a crossing stopped halfway --
	// would have its wheels on the higher side sent down to the lower one.
	// With a detected terrain, every wheel must stand on its near surface (or
	// above it) and short of its edge; and whatever the terrain, the ground
	// traced under each wheel must be that surface, within MinStepHeight.
	if (bTerrain)
	{
		TArray<RammsCurb::FTerrainWheel> Wheels;
		GatherTerrainWheels(Wheels);
		if (!RammsCurb::CheckWheelsOnNear(Profile.Terrain, Wheels, FVector2D(Heading.X, Heading.Y), OutReason))
		{
			return false;
		}
	}
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		if (!LegState[i].bHasWorld)
		{
			continue;
		}
		const FVector At = LegState[i].WheelWorld;
		const double  Near = ActiveTerrain.Near.HeightAt(FVector2D(At.X, At.Y));
		double		  Surface = 0.0;
		if (!TraceSurface(At, At.Z, FMath::Min(GroundZ, Near) - 60.0, Surface))
		{
			OutReason = FString::Printf(TEXT("nothing under the '%s' wheel; drive back onto solid ground first"),
				*Legs[i].Name.ToString());
			return false;
		}
		if (FMath::Abs(Surface - Near) > Tuning.MinStepHeight)
		{
			OutReason = FString::Printf(TEXT("the robot is standing across a step (the '%s' wheel is over ground "
											 "%+.1f cm from the surface the rest stand on); drive clear of it first"),
				*Legs[i].Name.ToString(), Surface - Near);
			return false;
		}
	}

	// The far side must be deep enough for the whole robot: seen so already,
	// or enough of it in view to start, the approach looking for the rest.
	bFarDepthPending = false;
	FarDepthWhy.Reset();
	FarDepthNeededCm = 0.0f;
	if (bTerrain)
	{
		RammsCurb::FFarDepthNeed Need;
		GatherFarDepthNeed(Need);
		FarDepthNeededCm = Need.At(Profile.Terrain.ApproachAngleDeg);
		switch (RammsCurb::FarDepthVerdict(Profile.Terrain, FarDepthNeededCm, &FarDepthWhy))
		{
			case RammsCurb::EFarDepth::Enough:
				break;
			case RammsCurb::EFarDepth::Pending:
				bFarDepthPending = true;
				break;
			default:
				OutReason = FarDepthWhy;
				return false;
		}
	}

	// Where the legs are, and which carry the robot now.
	float SupportSum = 0.0f;
	int32 SupportCount = 0;
	int32 FrontCount = 0;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		FLegState& State = LegState[i];
		State.InitialZ = State.MeasuredZ;
		State.CmdZ = State.MeasuredZ;
		State.LagZ = State.MeasuredZ;
		State.LagX = State.MeasuredX;
		if (State.FiveBar)
		{
			State.InitialX = State.MeasuredX;
			State.CmdX = State.InitialX;
			State.ModelX = State.InitialX;
		}
		else
		{
			FVector2D WheelCenter = FVector2D::ZeroVector;
			if (!MeasureFourBar(*EnsureBase(), Legs[i], WheelCenter, OutReason))
			{
				return false;
			}
			State.ModelX = static_cast<float>(WheelCenter.X);
		}
		// A wheel on the ground puts the chassis its radius over its ground,
		// less how high its leg holds it: in the plan's terms, over the level
		// ground the terrain offset stands for.
		if (State.WheelBottom < 1.5f)
		{
			SupportSum += Legs[i].WheelRadiusCm + State.TerrainOffset - State.MeasuredZ;
			++SupportCount;
		}
		if (Legs[i].Axle == ERammsCurbAxle::Front && State.bHasWorld)
		{
			++FrontCount;
		}
	}
	if (SupportCount == 0 || FrontCount == 0)
	{
		OutReason = TEXT("cannot tell which wheels are on the ground");
		return false;
	}
	InitialBaseHeight = SupportSum / SupportCount;

	// An axle already clear of the ground at the start -- the front corners in
	// the differential stance -- is not put down just to be lifted again: it
	// stays up from the approach straight into its own crossing.
	for (int32 a = 0; a < 3; ++a)
	{
		bStartsLifted[a] = true;
	}
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		if (!LegState[i].bHasWorld || LegState[i].WheelBottom < Tuning.SwingClearance)
		{
			bStartsLifted[static_cast<int32>(Legs[i].Axle)] = false;
		}
	}

	if (!BuildSteps(Profile, OutReason))
	{
		return false;
	}
	bTerrainDetected = bTerrain;
	PlannedTerrain = ActiveTerrain;
	PlanLines.Append(MeasureNotes);
	if (bFarDepthPending)
	{
		const FString Line = FString::Printf(TEXT("far side: %s; the robot must see the rest before the front wheels are %.1f cm "
												  "from the edge (the commit point)"),
			*FarDepthWhy, -CommitTo);
		PlanLines.Add(Line);
		OutReason += TEXT("; ") + Line;
	}
	return true;
}

bool URammsCurbNegotiationController::PerAxleAccepts(const FRammsCurbProfile& Profile, FString& OutReason) const
{
	// Lifting both wheels of an axle together puts one of them over the curb
	// face while the other is still a skew short of it: past about 10 deg
	// there is no axle pose that serves both.
	const float Approach = Profile.Terrain.bValid ? Profile.Terrain.ApproachAngleDeg : Profile.ApproachAngleDeg;
	if (FMath::Abs(Approach) > RammsCurb::PerAxleMaxApproachDeg)
	{
		OutReason = FString::Printf(TEXT("the per-axle gait lifts both wheels of an axle together, which works only within %.0f "
										 "deg of square; this edge is %+.1f deg off"),
			RammsCurb::PerAxleMaxApproachDeg, Approach);
		return false;
	}
	return true;
}

bool URammsCurbNegotiationController::PlaceEdge(const FRammsCurbProfile& Profile, FString& OutReason)
{
	// The edge, along the heading the crossing holds, from where the front
	// wheels are now: progress is reckoned from its near end.
	FVector FrontSum = FVector::ZeroVector;
	int32	FrontCount = 0;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		if (Legs[i].Axle == ERammsCurbAxle::Front && LegState[i].bHasWorld)
		{
			FrontSum += LegState[i].WheelWorld;
			++FrontCount;
		}
	}
	if (FrontCount == 0)
	{
		OutReason = TEXT("the front wheels cannot be located");
		return false;
	}
	EdgePoint = Horizontal(FrontSum / FrontCount) + Heading * Profile.EdgeDistanceCm;
	// Measure() reckoned progress from the edge as it stood then (none yet):
	// against the edge just placed, so that the plan's first step -- and
	// GetWheelProgress after CheckFeasibility -- see where the wheels are.
	for (FLegState& State : LegState)
	{
		if (State.bHasWorld)
		{
			State.Progress = static_cast<float>(FVector::DotProduct(Horizontal(State.WheelWorld - EdgePoint), Heading));
		}
	}
	return true;
}

double URammsCurbNegotiationController::SwingCentreZ(int32 LegIndex, const FVector2D& At, float ToLandCm) const
{
	// The highest the wheel's rim would touch -- the higher surface reaching
	// FaceMargin out over the face on a climb, EdgeMargin past the lip on a
	// descent, so the tyre keeps that much from the corner sideways as well --
	// plus SwingClearance under it. Climbing, over the rest of the way to
	// where it lands; descending, over its footprint and a little ahead.
	const FRammsCurbLeg&					Leg = Legs[LegIndex];
	const bool								bClimb = PlanStepHeight > 0.0f;
	const double							Dilation = bClimb ? Tuning.FaceMargin : Tuning.EdgeMargin;
	const FVector2D							H(Heading.X, Heading.Y);
	const float								Ahead = bClimb ? FMath::Max(0.0f, ToLandCm) : RammsCurb::DescentSwingLookCm;
	const int32								N = FMath::Max(2, FMath::CeilToInt(Ahead / SpanSampleCm) + 1);
	TArray<FVector2D, TInlineAllocator<64>> Path;
	for (int32 k = 0; k < N; ++k)
	{
		Path.Add(At + H * (Ahead * k / (N - 1)));
	}
	return RammsCurb::PathClearanceHeight(ActiveTerrain, Path, RollDirectionOf(Leg, Heading), Leg.WheelRadiusCm, Leg.TyreWidthCm,
			   Dilation)
		+ Tuning.SwingClearance;
}

float URammsCurbNegotiationController::ToLandCm(int32 LegIndex) const
{
	const ERammsCurbAxle Axle = Legs[LegIndex].Axle;
	for (int32 k = FMath::Max(0, StepIndex); k < Steps.Num(); ++k)
	{
		if (Steps[k].Phase == ERammsCurbPhase::Advance && Steps[k].Axle == Axle)
		{
			return FMath::Max(0.0f, Steps[k].AdvanceTo - AxleProgress(Axle));
		}
	}
	return 0.0f;
}

FVector2D URammsCurbNegotiationController::BellyHalfExtent() const
{
	// As tuned, or the corner crank pivots fore/aft and the corner wheels
	// across.
	FVector2D Half = Tuning.ChassisHalfExtent;
	if (Half.X <= 0.0 || Half.Y <= 0.0)
	{
		double X = 0.0, Y = 0.0;
		for (const FRammsCurbLeg& Leg : Legs)
		{
			if (Leg.Kind == ERammsCurbLegKind::FourBar && Leg.Axle != ERammsCurbAxle::Center)
			{
				X = FMath::Max(X, FMath::Abs(static_cast<double>(Leg.FourBar.CrankPivot.X)));
				Y = FMath::Max(Y, FMath::Abs(static_cast<double>(Leg.Wheel.Position.Y)));
			}
		}
		Half = FVector2D(Half.X > 0.0 ? Half.X : X, Half.Y > 0.0 ? Half.Y : Y);
	}
	return Half;
}

double URammsCurbNegotiationController::BellyTopAt(const FVector2D& Origin) const
{
	// The highest ground under the belly plate with the chassis origin over
	// Origin, facing the heading: the surface under each point of the plate,
	// and the higher side's lip where the plate crosses the edge.
	const FVector2D Half = BellyHalfExtent();
	const FVector2D H(Heading.X, Heading.Y);
	const FVector2D Right(-H.Y, H.X);
	const double	NH = FVector2D::DotProduct(ActiveTerrain.EdgeNormal, H);
	double			Top = -TNumericLimits<double>::Max();
	for (int32 b = -1; b <= 1; ++b)
	{
		const FVector2D Line = Origin + Right * (b * Half.Y);
		for (int32 a = -2; a <= 2; ++a)
		{
			Top = FMath::Max(Top, ActiveTerrain.SurfaceAt(Line + H * (0.5 * a * Half.X)));
		}
		if (FMath::Abs(NH) > 1e-6)
		{
			const double X = -ActiveTerrain.DistancePastEdge(Line) / NH;
			if (FMath::Abs(X) <= Half.X)
			{
				const FVector2D P = Line + H * X;
				Top = FMath::Max(Top, FMath::Max(ActiveTerrain.Near.HeightAt(P), ActiveTerrain.Far.HeightAt(P)));
			}
		}
	}
	return Top;
}

void URammsCurbNegotiationController::SpanGoals(const FAxleGoal Goals[3], const TArray<float>& LegX, float FiveBarSpreadCm,
	float SFrom, float STo, float SLand, FGoalSpan& Out) const
{
	// The chassis moved along the heading by s, every 2 cm over the stretch
	// (and a little either side): each wheel's surface over the reference
	// the chassis height is planned on, and the belly's ground.
	const int32 NumLegs = Legs.Num();
	Out.Offset.Init(FVector2D(TNumericLimits<double>::Max(), -TNumericLimits<double>::Max()), NumLegs);
	Out.Floor = -TNumericLimits<float>::Max();
	const FVector2D H(Heading.X, Heading.Y);
	const float		Lo = FMath::Min(SFrom, STo) - SpanSlackCm;
	const float		Hi = FMath::Max(SFrom, STo) + SpanSlackCm;
	const int32		N = FMath::Max(2, FMath::CeilToInt((Hi - Lo) / SpanSampleCm) + 1);
	Out.SampleFrom = Lo;
	Out.SampleTo = Hi;
	Out.SampleFloor.SetNum(N);
	Out.SampleReference.SetNum(N);
	Out.SampleOffset.Init(FVector2D(NAN, NAN), N * NumLegs);

	TArray<RammsCurb::FTerrainLeg, TInlineAllocator<8>> Wheels;
	TArray<double, TInlineAllocator<8>>					Offsets;
	Wheels.SetNum(NumLegs);
	Offsets.SetNum(NumLegs);
	for (int32 k = 0; k < N; ++k)
	{
		const double S = Lo + (Hi - Lo) * k / (N - 1);
		for (int32 i = 0; i < NumLegs; ++i)
		{
			const FRammsCurbLeg&	Leg = Legs[i];
			const FLegState&		State = LegState[i];
			const float				Dx = State.FiveBar && LegX.IsValidIndex(i) ? LegX[i] - State.MeasuredX : 0.0f;
			RammsCurb::FTerrainLeg& W = Wheels[i];
			const FAxleGoal&		Goal = Goals[static_cast<int32>(Leg.Axle)];
			W.XY = FVector2D(State.WheelWorld.X, State.WheelWorld.Y) + H * (S + Dx);
			// A lifted wheel counts the surface under it; a planted one the
			// one it stands on.
			W.bFar = Goal.Goal == ELegGoal::Swing ? ActiveTerrain.DistancePastEdge(W.XY) >= 0.0 : Goal.bFar;
			W.RadiusCm = Leg.WheelRadiusCm;
			W.ContactWidthCm = RammsCurb::ContactWidth(Leg.TyreWidthCm, Leg.ContactWidthCm);
			W.RollDirection = RollDirectionOf(Leg, Heading);
		}
		const double D = RammsCurb::TerrainOffsets(ActiveTerrain, Wheels, GroundZ, PlanStepHeight, FVector2D::ZeroVector, Offsets);
		for (int32 i = 0; i < NumLegs; ++i)
		{
			const FRammsCurbLeg& Leg = Legs[i];
			const FAxleGoal&	 Goal = Goals[static_cast<int32>(Leg.Axle)];
			FVector2D&			 Range = Out.Offset[i];
			FVector2D&			 Here = Out.SampleOffset[k * NumLegs + i];
			// Into the stretch's range, and (bHere) this sample's.
			auto Add = [&Range, &Here](double V, bool bHere = true) {
				Range.X = FMath::Min(Range.X, V);
				Range.Y = FMath::Max(Range.Y, V);
				if (bHere)
				{
					Here.X = FMath::IsNaN(Here.X) ? V : FMath::Min(Here.X, V);
					Here.Y = FMath::IsNaN(Here.Y) ? V : FMath::Max(Here.Y, V);
				}
			};
			switch (Goal.Goal)
			{
				case ELegGoal::Hold:
					Add(0.0, /*bHere=*/false);
					break;
				case ELegGoal::Swing:
				{
					// Carried clear of the terrain over the world, against a
					// reference that moves with the wheels -- up to where it
					// lands, past which it is planted, not carried: the
					// stretch's slack beyond the landing is not its.
					if (S > SLand && Range.X <= Range.Y)
					{
						break;
					}
					const double LevelZ = Goal.Level + Leg.WheelRadiusCm + Tuning.SwingClearance;
					const double Clear = SwingCentreZ(i, Wheels[i].XY, static_cast<float>(FMath::Max(0.0, SLand - S))) - GroundZ;
					Add(Clear - D - LevelZ);
					break;
				}
				default:
				{
					Add(Offsets[i]);
					if (LegState[i].FiveBar && FiveBarSpreadCm > 0.0f)
					{
						// Walked fore/aft, the wheel stands a little higher or
						// lower on its surface.
						const double At = RammsCurb::SurfaceHeight(ActiveTerrain, Goal.bFar, Wheels[i].XY);
						for (const double Sign : { -1.0, 1.0 })
						{
							Add(Offsets[i] + RammsCurb::SurfaceHeight(ActiveTerrain, Goal.bFar, Wheels[i].XY + H * (Sign * FiveBarSpreadCm))
								- At);
						}
					}
					break;
				}
			}
		}
		// The belly over the highest ground under it.
		const float Floor = static_cast<float>(BellyTopAt(ChassisOriginXY + H * S) - GroundZ - D) + Tuning.ChassisClearance
			- Tuning.ChassisUnderside;
		Out.SampleFloor[k] = Floor;
		Out.SampleReference[k] = static_cast<float>(D);
		Out.Floor = FMath::Max(Out.Floor, Floor);
	}
}

FVector2D URammsCurbNegotiationController::FiveBarReachAt(int32 LegIndex, float X) const
{
	const FLegState& State = LegState[LegIndex];
	if (!State.FiveBar)
	{
		return FVector2D(1.0, 0.0);
	}
	const int64 Key = (static_cast<int64>(LegIndex) << 32) | static_cast<uint32>(FMath::RoundToInt(X * 10.0f));
	if (const FVector2D* Found = FiveBarReachCache.Find(Key))
	{
		return *Found;
	}
	auto Reaches = [&](double Z) {
		bool bReachable = false;
		State.FiveBar->SolveTarget(FVector2D(X, Z), bReachable);
		return bReachable;
	};
	// A reachable height first, scanning the leg's extent; then each end of
	// the stretch around it, by halving.
	const double Lo0 = State.HeightRange.X - 1.0, Hi0 = State.HeightRange.Y + 1.0;
	double		 Inside = TNumericLimits<double>::Max();
	for (double Z = Lo0; Z <= Hi0; Z += 0.5)
	{
		if (Reaches(Z))
		{
			Inside = Z;
			break;
		}
	}
	FVector2D Reach(1.0, 0.0);
	if (Inside < TNumericLimits<double>::Max())
	{
		double A = Lo0, B = Inside;
		for (int32 n = 0; n < 16; ++n)
		{
			const double M = 0.5 * (A + B);
			(Reaches(M) ? B : A) = M;
		}
		Reach.X = B;
		double C = Inside, E = Hi0;
		for (double Z = Inside; Z <= Hi0; Z += 0.5)
		{
			if (!Reaches(Z))
			{
				E = Z;
				break;
			}
			C = Z;
		}
		for (int32 n = 0; n < 16; ++n)
		{
			const double M = 0.5 * (C + E);
			(Reaches(M) ? C : E) = M;
		}
		Reach.Y = C;
	}
	FiveBarReachCache.Add(Key, Reach);
	return Reach;
}

FVector2D URammsCurbNegotiationController::CorridorAt(const FAxleGoal Goals[3], TConstArrayView<FVector2D> Offset, float Floor,
	const TArray<float>& LegX, int32* OutLo, int32* OutHi) const
{
	// Each leg's goal height falls one for one as the chassis rises, so each
	// allows a stretch of chassis heights -- over the least and most the
	// terrain puts its wheel at; the belly a least one. A lifted leg bounds it
	// from below only: a chassis higher than it needs carries the wheel at
	// the leg's lowest reach, higher than it need be.
	const float Margin = Tuning.LegRangeMargin;
	double		Lo = Floor, Hi = TNumericLimits<double>::Max();
	int32		LoLeg = -1, HiLeg = -1;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FAxleGoal& Goal = Goals[static_cast<int32>(Legs[i].Axle)];
		if (Goal.Goal == ELegGoal::Hold || !Offset.IsValidIndex(i) || !IsFinite(Offset[i]))
		{
			continue;
		}
		const double ALo = GoalHeight(i, Goal, 0.0f, static_cast<float>(Offset[i].X));
		const double AHi = GoalHeight(i, Goal, 0.0f, static_cast<float>(Offset[i].Y));
		FVector2D	 Z = LegState[i].HeightRange;
		if (LegState[i].FiveBar)
		{
			Z = FiveBarReachAt(i, LegX.IsValidIndex(i) ? LegX[i] : LegState[i].InitialX);
		}
		if (Z.X > Z.Y)
		{
			Lo = 1.0;
			Hi = 0.0;
			LoLeg = HiLeg = i;
			break;
		}
		if (AHi - (Z.Y - Margin) > Lo)
		{
			Lo = AHi - (Z.Y - Margin);
			LoLeg = i;
		}
		if (Goal.Goal != ELegGoal::Swing && ALo - (Z.X + Margin) < Hi)
		{
			Hi = ALo - (Z.X + Margin);
			HiLeg = i;
		}
	}
	if (OutLo)
	{
		*OutLo = LoLeg;
	}
	if (OutHi)
	{
		*OutHi = HiLeg;
	}
	return FVector2D(Lo, Hi);
}

float URammsCurbNegotiationController::FCorridorPath::At(float Travel) const
{
	if (!IsValid())
	{
		return 0.0f;
	}
	const float K = FMath::Clamp((Travel - From) / Step, 0.0f, static_cast<float>(Heights.Num() - 1));
	const int32 I = FMath::Min(FMath::FloorToInt(K), Heights.Num() - 1);
	const int32 J = FMath::Min(I + 1, Heights.Num() - 1);
	return FMath::Lerp(Heights[I], Heights[J], K - I);
}

bool URammsCurbNegotiationController::PlanLiftCorridor(ERammsCurbAxle Axle, TArrayView<const FGoalSet> Sets, float PreferredHeight,
	float PreferredCenterX, float MinShift, float& OutHeight, TArray<float>& OutLegX, float& OutShift, FCorridorPath& OutPath,
	FString& OutReason) const
{
	// The centre wheels' offsets, best first, as PlanLift tries them; both at
	// one offset here.
	const TArray<float> Candidates = CentreCandidates(Axle, PreferredCenterX, Tuning.CenterOfMassX);
	const float			Required = Axle == ERammsCurbAxle::Center ? 0.0f : MinShift;
	const float			Wanted = Axle == ERammsCurbAxle::Center ? 0.0f : FMath::Abs(PreferredCenterX - Tuning.CenterOfMassX);
	const int32			NumLegs = Legs.Num();
	auto				Name = [this](int32 Leg) {
		   return Legs.IsValidIndex(Leg) ? FString::Printf(TEXT("leg '%s'"), *Legs[Leg].Name.ToString()) : FString(TEXT("the belly"));
	};

	// One grid over every set's stretch. The chassis may rise or sink by at
	// most Rate between samples: BaseHeightSpeed over the time the robot
	// takes between them at the most it may come in at.
	float From = TNumericLimits<float>::Max(), To = -TNumericLimits<float>::Max();
	for (const FGoalSet& Set : Sets)
	{
		if (Set.Span && Set.Span->SampleFloor.Num() >= 2)
		{
			From = FMath::Min(From, Set.Span->SampleFrom);
			To = FMath::Max(To, Set.Span->SampleTo);
		}
	}
	if (From >= To)
	{
		OutReason = FString::Printf(TEXT("no chassis path lets the %s axle cross: nothing to plan it over"), AxleName(Axle));
		return false;
	}
	const int32 N = FMath::Max(2, FMath::CeilToInt((To - From) / SpanSampleCm) + 1);
	const float Ds = (To - From) / (N - 1);
	const float VPlan = FMath::Max3(Tuning.DriveSpeed, MeasuredWheelSpeed(), 1.0f);
	const float Rate = Tuning.BaseHeightSpeed * Ds / VPlan;
	const float Half = 0.5f * Tuning.MinCorridorCm;

	float	BestCost = TNumericLimits<float>::Max();
	FString FirstWhy;
	for (const float X : Candidates)
	{
		TArray<float> LegX;
		LegX.Init(0.0f, NumLegs);
		for (int32 i = 0; i < NumLegs; ++i)
		{
			if (LegState[i].FiveBar)
			{
				LegX[i] = X;
			}
		}
		const float Shift = Axle == ERammsCurbAxle::Front ? X - Tuning.CenterOfMassX
			: Axle == ERammsCurbAxle::Rear				  ? Tuning.CenterOfMassX - X
														  : 0.0f;
		if (Shift < Required - KINDA_SMALL_NUMBER)
		{
			continue;
		}

		// At each sample, the heights (world, over GroundZ) every set that
		// may be in force there allows.
		TArray<FVector2D> Band;
		TArray<double>	  Wish;
		Band.SetNum(N);
		Wish.Init(NAN, N);
		double Narrowest = TNumericLimits<double>::Max();
		bool   bOk = true;
		for (int32 k = 0; k < N && bOk; ++k)
		{
			const float S = From + Ds * k;
			double		Lo = -TNumericLimits<double>::Max(), Hi = TNumericLimits<double>::Max();
			int32		LoLeg = -1, HiLeg = -1;
			double		Reference = 0.0;
			bool		bAny = false;
			for (const FGoalSet& Set : Sets)
			{
				const FGoalSpan* Span = Set.Span;
				const int32		 M = Span ? Span->SampleFloor.Num() : 0;
				if (M < 2 || S < Span->SampleFrom - 1e-3f || S > Span->SampleTo + 1e-3f)
				{
					continue;
				}
				const int32		J = FMath::Clamp(FMath::RoundToInt((S - Span->SampleFrom) / (Span->SampleTo - Span->SampleFrom) * (M - 1)), 0, M - 1);
				int32			SetLo = -1, SetHi = -1;
				const FVector2D C = CorridorAt(Set.Goals, TConstArrayView<FVector2D>(Span->SampleOffset.GetData() + J * NumLegs, NumLegs),
					Span->SampleFloor[J], LegX, &SetLo, &SetHi);
				const double	D = Span->SampleReference[J];
				if (!bAny)
				{
					Reference = D;
					bAny = true;
				}
				if (C.X + D > Lo)
				{
					Lo = C.X + D;
					LoLeg = SetLo;
				}
				if (C.Y + D < Hi)
				{
					Hi = C.Y + D;
					HiLeg = SetHi;
				}
			}
			if (!bAny)
			{
				// None in force here (none should be: the sets' stretches
				// overlap); the path holds.
				Band[k] = FVector2D(-1e6, 1e6);
				continue;
			}
			if (Hi - Lo < Tuning.MinCorridorCm)
			{
				bOk = false;
				if (FirstWhy.IsEmpty())
				{
					FirstWhy = FString::Printf(TEXT("%s needs the chassis at %.1f cm or more where %s needs it at %.1f cm or less, a "
													"corridor of %.1f cm (%.1f at least)"),
						*Name(LoLeg), Lo - Reference, *Name(HiLeg), Hi - Reference, Hi - Lo, Tuning.MinCorridorCm);
				}
				break;
			}
			Band[k] = FVector2D(Lo + Half, Hi - Half);
			Narrowest = FMath::Min(Narrowest, Hi - Lo);
			// The preferred height over the terrain there, a margin in from
			// the band's sides where it has room.
			const double Keep = FMath::Min(static_cast<double>(Tuning.LegRangeMargin), 0.5 * (Band[k].Y - Band[k].X));
			Wish[k] = FMath::Clamp(PreferredHeight + Reference, Band[k].X + Keep, Band[k].Y - Keep);
		}
		if (!bOk)
		{
			continue;
		}

		// A path inside every band, moving no faster than Rate a sample, as
		// near the preferred height as that lets it (RammsCurb::CorridorPath).
		TArray<float> Path;
		int32		  Stuck = INDEX_NONE;
		if (!RammsCurb::CorridorPath(Band, Wish, Rate, Path, Stuck))
		{
			if (FirstWhy.IsEmpty())
			{
				FirstWhy = FString::Printf(TEXT("its corridor moves faster than the chassis can follow (BaseHeightSpeed %.0f cm/s at "
												"%.0f cm/s), %.0f cm into the axle's crossing"),
					Tuning.BaseHeightSpeed, VPlan, From + Ds * Stuck);
			}
			continue;
		}

		// The full stability margin first, then the widest corridor.
		const float Cost = 2.0f * FMath::Max(0.0f, Wanted - Shift) - 0.01f * static_cast<float>(Narrowest);
		if (Cost >= BestCost)
		{
			continue;
		}
		BestCost = Cost;
		OutLegX = LegX;
		OutShift = Shift;
		OutPath.From = From;
		OutPath.Step = Ds;
		OutPath.Heights = MoveTemp(Path);
		// The step's plan height: the path's where the first set begins, over
		// that set's terrain reference.
		const FGoalSpan* First = Sets[0].Span;
		const float		 Start = First ? First->SampleFrom : From;
		const float		 Reference = First && First->SampleReference.Num() > 0 ? First->SampleReference[0] : 0.0f;
		OutHeight = OutPath.At(Start) - Reference;
	}
	if (BestCost < TNumericLimits<float>::Max())
	{
		return true;
	}
	OutReason = FString::Printf(TEXT("no chassis path lets the %s axle cross: %s"), AxleName(Axle),
		FirstWhy.IsEmpty() ? TEXT("the centre wheels cannot shift far enough past the centre of mass") : *FirstWhy);
	return false;
}

bool URammsCurbNegotiationController::BuildSteps(const FRammsCurbProfile& Profile, FString& OutReason)
{
	const float StepHeight = Profile.StepHeightCm;
	if (!PerAxleAccepts(Profile, OutReason))
	{
		return false;
	}
	if (!PlaceEdge(Profile, OutReason))
	{
		return false;
	}
	PlanStepHeight = StepHeight;
	// Every span's travel, and a corridor path's, is measured from here.
	PlanOriginXY = ChassisOriginXY;

	TArray<float> InitialLegX;
	for (const FLegState& State : LegState)
	{
		InitialLegX.Add(State.InitialX);
	}

	float FrontRadius = 0.0f;
	float AxleRadius[3] = { 0.0f, 0.0f, 0.0f };
	for (const FRammsCurbLeg& Leg : Legs)
	{
		float& R = AxleRadius[static_cast<int32>(Leg.Axle)];
		R = FMath::Max(R, Leg.WheelRadiusCm);
	}
	FrontRadius = AxleRadius[0];

	const bool	bClimb = StepHeight > 0.0f;
	const float High = FMath::Max(0.0f, StepHeight);

	Steps.Reset();
	PlanLines.Reset();
	bPlannedMargin = false;
	PlannedMinMargin = 0.0f;
	// The least centre-wheel shift past the centre of mass planned for a
	// corner axle's lift: the margin this gait plans. A centre axle's lift
	// leaves the robot on both corner axles and plans none.
	auto NotePlannedMargin = [this](ERammsCurbAxle Axle, float Shift) {
		if (Axle != ERammsCurbAxle::Center)
		{
			PlannedMinMargin = bPlannedMargin ? FMath::Min(PlannedMinMargin, Shift) : Shift;
			bPlannedMargin = true;
		}
	};

	// Where along the chassis' travel from here (s, cm along the heading)
	// each axle's steps can come: its wheels' braking points, which its lift
	// must be done by, and where it has advanced to its landing. Each set of
	// goals is checked over every s it can hold at, each leg on the terrain
	// under its own wheel there.
	float Progress0[3], SBrake[3], AdvanceTo[3], SAdv[3];
	for (int32 a = 0; a < 3; ++a)
	{
		Progress0[a] = TNumericLimits<float>::Max();
		SBrake[a] = TNumericLimits<float>::Max();
	}
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		if (!LegState[i].bHasWorld)
		{
			continue;
		}
		const int32 a = static_cast<int32>(Legs[i].Axle);
		const float Brake = bClimb ? -Tuning.FaceMargin - Legs[i].WheelRadiusCm : -Tuning.EdgeMargin;
		Progress0[a] = FMath::Min(Progress0[a], LegState[i].Progress);
		SBrake[a] = FMath::Min(SBrake[a], Brake - LegState[i].Progress);
	}
	for (int32 a = 0; a < 3; ++a)
	{
		// Past the edge where it is farthest: progress is measured from its
		// near end, and a wheel landing short of its own stretch of edge lands
		// on the corner.
		AdvanceTo[a] = (bClimb ? Tuning.LandingOverlap : AxleRadius[a] + Tuning.DropClearance) + Profile.EdgeSkewCm;
		SAdv[a] = AdvanceTo[a] - Progress0[a];
	}
	const float SettleTo = AxleRadius[2] + Tuning.FinishClearance + Profile.EdgeSkewCm;
	const float SSettle = SettleTo - Progress0[2];
	// Where the gait's first Prepare begins, PrepareDistance short of the
	// front wheels' first window; the level-in starts earlier still, so that
	// the base is level before any wheel is LevelBeforeEdgeCm from its edge.
	const float PrepareFrom = (bClimb ? -(FrontRadius + Tuning.FaceMargin) : -Tuning.EdgeMargin) - Tuning.PrepareDistance;
	const float ApproachDeg = Profile.Terrain.bValid ? Profile.Terrain.ApproachAngleDeg : Profile.ApproachAngleDeg;
	const float VPlan = FMath::Max(Tuning.DriveSpeed, MeasuredWheelSpeed());
	const float EarliestLevelIn = FMath::Min(PrepareFrom, -RammsCurb::LevelInStartCm(RammsCurb::LevelInPlanSeconds, VPlan, ApproachDeg));
	// Where each axle's legs may start to move for it: the level-in for the
	// front, the axle before's landing for the others (moved on below when
	// that landing is).
	float SFromNext = FMath::Max(0.0f, EarliestLevelIn - Progress0[0]);

	// A stance with every wheel down, as near the given height as the legs
	// allow. The robot may have started in one that is neither: the
	// differential stance holds the front corners clear and pitches the
	// chassis, and the gait's heights are all reckoned for a level chassis.
	auto AllDown = [this](float Level, bool bFar, float NearHeight, const TArray<float>& LegX, const FGoalSpan& Span,
					   float& OutHeight) {
		FAxleGoal Down[3];
		for (int32 a = 0; a < 3; ++a)
		{
			Down[a] = { ELegGoal::Support, Level, bFar };
		}
		for (float D = 0.0f; D < 20.0f; D += 0.25f)
		{
			for (const float H : { NearHeight + D, NearHeight - D })
			{
				if (IsReachable(Down, H, LegX, -1000.0f, &Span, nullptr))
				{
					OutHeight = H;
					return true;
				}
			}
		}
		return false;
	};

	// Approach in the stance the robot is driving in -- outdoors, the
	// differential stance, front corners up -- and change it only once close.
	FStep Approach;
	Approach.Phase = ERammsCurbPhase::Approach;
	for (int32 a = 0; a < 3; ++a)
	{
		Approach.Axles[a] = { ELegGoal::Hold, 0.0f, false };
	}
	Approach.LegX = InitialLegX;
	Approach.BaseHeight = InitialBaseHeight;
	Approach.Span.Offset.Init(FVector2D::ZeroVector, Legs.Num());

	// An axle already clear of the ground at the start (bStartsLifted, as
	// BuildPlan measured it) stays up from the approach into its own crossing.
	bool bHasLegs[3] = { false, false, false };
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		bHasLegs[static_cast<int32>(Legs[i].Axle)] = true;
	}

	// The approach carries the stance's lifted axle up all the way to the
	// edge, standing on the other two: when that is one corner axle, the
	// stance is planned like a lift of it, and its margin -- the centre wheels
	// where they started -- counts with the lifts'. A front axle kept up into
	// its own crossing then has its centre wheels moved from there to the
	// lift's offsets, past no margin below either end. Two axles up leave no
	// 1-D margin to speak of.
	ApproachLiftedAxle = INDEX_NONE;
	int32 LiftedAxles = 0;
	for (int32 a = 0; a < 3; ++a)
	{
		if (bHasLegs[a] && bStartsLifted[a])
		{
			ApproachLiftedAxle = a;
			++LiftedAxles;
		}
	}
	if (LiftedAxles != 1)
	{
		ApproachLiftedAxle = INDEX_NONE;
	}
	if (ApproachLiftedAxle != INDEX_NONE && ApproachLiftedAxle != static_cast<int32>(ERammsCurbAxle::Center))
	{
		float Sum = 0.0f;
		int32 Count = 0;
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			if (Legs[i].Kind == ERammsCurbLegKind::FiveBar)
			{
				Sum += InitialLegX[i];
				++Count;
			}
		}
		if (Count > 0)
		{
			const ERammsCurbAxle Up = static_cast<ERammsCurbAxle>(ApproachLiftedAxle);
			const float			 Mean = Sum / Count;
			NotePlannedMargin(Up, Up == ERammsCurbAxle::Front ? Mean - Tuning.CenterOfMassX : Tuning.CenterOfMassX - Mean);
		}
	}
	Approach.ApproachTo = PrepareFrom;
	Approach.SpeedFraction = 1.0f;
	Steps.Add(Approach);

	float Height = Approach.BaseHeight;
	for (int32 AxleIndex = 0; AxleIndex < 3; ++AxleIndex)
	{
		const ERammsCurbAxle Axle = static_cast<ERammsCurbAxle>(AxleIndex);
		FAxleGoal			 Before[3], Swing[3], After[3];
		for (int32 a = 0; a < 3; ++a)
		{
			Before[a] = { ELegGoal::Support, a < AxleIndex ? StepHeight : 0.0f, a < AxleIndex };
		}
		FMemory::Memcpy(Swing, Before, sizeof(Before));
		FMemory::Memcpy(After, Before, sizeof(Before));
		Swing[AxleIndex] = { ELegGoal::Swing, High, true };
		After[AxleIndex] = { ELegGoal::Support, StepHeight, true };
		if (AxleIndex == 0 && bStartsLifted[0])
		{
			Before[0] = Swing[0];
		}

		// While a corner axle is up the robot stands on the centre wheels and
		// the other corner, so the centre wheels go past the centre of mass
		// towards that corner.
		float Preferred = Tuning.CenterOfMassX;
		if (Axle == ERammsCurbAxle::Front)
		{
			Preferred += Tuning.StabilityMargin;
		}
		else if (Axle == ERammsCurbAxle::Rear)
		{
			Preferred -= Tuning.StabilityMargin;
		}

		// What the terrain asks of each leg over each set of goals: the centre
		// wheels taken anywhere between the offsets PlanLift may pick. Where
		// no pose serves the axle landing where the gait means it to, it lands
		// a little further on: a robot standing on a slope may not reach the
		// lower surface with the landed wheels just past the edge while its
		// belly is still over the high side behind.
		const bool	  bCentreLift = Axle == ERammsCurbAxle::Center;
		const float	  SpanX = bCentreLift ? Preferred : 0.5f * (Preferred + Tuning.CenterOfMassX);
		const float	  Spread = bCentreLift ? 10.5f : 0.5f * FMath::Abs(Preferred - Tuning.CenterOfMassX) + 0.5f;
		TArray<float> SpanLegX;
		SpanLegX.Init(SpanX, Legs.Num());
		const float	  SFromA = SFromNext;
		FGoalSpan	  BeforeSpan, SwingSpan, AfterSpan;
		float		  LiftHeight = 0.0f, DriveHeight = 0.0f;
		float		  LiftShift = 0.0f, DriveShift = 0.0f;
		TArray<float> LiftLegX, DriveLegX;
		bool		  bOnePose = false, bPlanned = false, bCorridor = false;
		FCorridorPath CorridorPath;
		FString		  FirstReason, CorridorReason, LaterReason;
		float		  Extra = 0.0f;

		// Plan the axle's crossing with it landing Extra further on than the
		// gait means it to, the goals after the landing holding until the next
		// axle lifts: at one chassis height (or the legacy lift-then-drive
		// pair), or (bByCorridor) the chassis following a path through its
		// corridor.
		auto PlanLanding = [&](bool bByCorridor, FString& OutWhy) {
			const float SLand = SAdv[AxleIndex] + Extra;
			const float SUntil = FMath::Max(SLand, AxleIndex < 2 ? SBrake[AxleIndex + 1] : SSettle);
			SpanGoals(Before, SpanLegX, Spread, SFromA, SBrake[AxleIndex], SLand, BeforeSpan);
			SpanGoals(Swing, SpanLegX, Spread, SFromA, SLand, SLand, SwingSpan);
			SpanGoals(After, SpanLegX, Spread, SLand, SUntil, SLand, AfterSpan);
			const float ChassisFloor = FMath::Max3(BeforeSpan.Floor, SwingSpan.Floor, AfterSpan.Floor);

			// One pose for the whole crossing when there is one. When there is
			// not, lift from a pose that still reaches the ground behind, with
			// the robot stopped, then move to one it can drive on two axles
			// from.
			const FGoalSet All[] = { { Before, &BeforeSpan }, { Swing, &SwingSpan }, { After, &AfterSpan } };
			const FGoalSet Off[] = { { Before, &BeforeSpan }, { Swing, &SwingSpan } };
			const FGoalSet On[] = { { Swing, &SwingSpan }, { After, &AfterSpan } };
			bCorridor = false;
			if (!bByCorridor)
			{
				bOnePose = PlanLift(Axle, All, ChassisFloor, Height, Preferred, Tuning.MinStabilityMargin, LiftHeight, LiftLegX, LiftShift,
					OutWhy);
				if (bOnePose)
				{
					DriveHeight = LiftHeight;
					DriveLegX = LiftLegX;
					return true;
				}
				FString Why;
				return PlanLift(Axle, Off, ChassisFloor, Height, Preferred, FMath::Min(Tuning.LiftOffMargin, Tuning.MinStabilityMargin),
						   LiftHeight, LiftLegX, LiftShift, Why)
					&& PlanLift(Axle, On, ChassisFloor, LiftHeight, Preferred, Tuning.MinStabilityMargin, DriveHeight, DriveLegX, DriveShift,
						Why);
			}
			// No one height over the terrain reference serves the whole of a
			// set's stretch: a path for the chassis inside the corridor of
			// heights every leg and the belly allow wherever the robot may be,
			// at least Tuning.MinCorridorCm wide everywhere, with the robot
			// moving on throughout.
			bOnePose = bCorridor = PlanLiftCorridor(Axle, All, Height, Preferred, Tuning.MinStabilityMargin, LiftHeight, LiftLegX,
				LiftShift, CorridorPath, OutWhy);
			if (bCorridor)
			{
				DriveHeight = LiftHeight;
				DriveLegX = LiftLegX;
			}
			return bCorridor;
		};

		// How far (cm, chassis travel) the robot would still have to go to
		// end this axle's Advance -- its wheels in their swing pose, their
		// measured progress at the landing -- once the next axle's nearest
		// wheel, where this axle's plan holds it (a centre wheel at this
		// plan's fore/aft offset), is NextLiftRoomCm short of its braking
		// point. Positive: that braking cap would hold the robot short of the
		// landing for good. 0 or less: room enough.
		auto AdvanceShort = [&]() {
			const float SLand = SAdv[AxleIndex] + Extra;
			const float Target = AdvanceTo[AxleIndex] + Extra;
			const int32 NumLegs = Legs.Num();
			const int32 M = SwingSpan.SampleFloor.Num();
			const int32 K = M > 1 ? FMath::Clamp(FMath::FloorToInt((SLand - SwingSpan.SampleFrom) / (SwingSpan.SampleTo - SwingSpan.SampleFrom) * (M - 1)), 0, M - 1)
								  : 0;
			// The plan height where it lands: the corridor's path there, over
			// the terrain reference, or the height it drives on.
			const float Reference = SwingSpan.SampleReference.IsValidIndex(K) ? SwingSpan.SampleReference[K] : 0.0f;
			const float Land = bCorridor ? CorridorPath.At(SLand) - Reference : DriveHeight;
			float		SEnd = -TNumericLimits<float>::Max();
			for (int32 i = 0; i < NumLegs; ++i)
			{
				const FLegState& State = LegState[i];
				if (Legs[i].Axle != Axle || !State.bHasWorld)
				{
					continue;
				}
				float Dx = 0.0f;
				if (State.FiveBar)
				{
					Dx = DriveLegX.IsValidIndex(i) ? DriveLegX[i] - State.MeasuredX : 0.0f;
				}
				else if (Legs[i].Kind == ERammsCurbLegKind::FourBar)
				{
					// A 4-bar carries its wheel fore/aft with its height: in the
					// swing pose the plan holds it at by the landing, as low as
					// its leg reaches at least.
					float Offset = NAN;
					for (int32 k = K; k >= 0 && FMath::IsNaN(Offset); --k)
					{
						const int32 Idx = k * NumLegs + i;
						Offset = SwingSpan.SampleOffset.IsValidIndex(Idx) ? static_cast<float>(SwingSpan.SampleOffset[Idx].Y) : 0.0f;
					}
					const float Z = FMath::Clamp(GoalHeight(i, Swing[AxleIndex], Land, FMath::IsNaN(Offset) ? 0.0f : Offset),
						State.HeightRange.X + Tuning.LegRangeMargin, State.HeightRange.Y - Tuning.LegRangeMargin);
					Dx = FourBarX(i, Z, State.MeasuredX) - State.MeasuredX;
				}
				SEnd = FMath::Max(SEnd, Target - (State.Progress + Dx));
			}
			float SNext = TNumericLimits<float>::Max();
			for (int32 i = 0; i < NumLegs; ++i)
			{
				const FLegState& State = LegState[i];
				if (static_cast<int32>(Legs[i].Axle) != AxleIndex + 1 || !State.bHasWorld)
				{
					continue;
				}
				const float Brake = bClimb ? -Tuning.FaceMargin - Legs[i].WheelRadiusCm : -Tuning.EdgeMargin;
				const float Dx = State.FiveBar && DriveLegX.IsValidIndex(i) ? DriveLegX[i] - State.MeasuredX : 0.0f;
				SNext = FMath::Min(SNext, Brake - (State.Progress + Dx));
			}
			return SEnd == -TNumericLimits<float>::Max() || SNext == TNumericLimits<float>::Max() ? 0.0f
																								  : SEnd - (SNext - NextLiftRoomCm);
		};

		// The least of the centre wheels' fore/aft offsets in this plan.
		auto CentresAt = [&]() {
			float Least = TNumericLimits<float>::Max();
			for (int32 i = 0; i < Legs.Num(); ++i)
			{
				if (LegState[i].FiveBar && DriveLegX.IsValidIndex(i))
				{
					Least = FMath::Min(Least, DriveLegX[i]);
				}
			}
			return Least;
		};

		// The landing where the gait means it to, then further on, a step at
		// a time up to MaxExtraLandingCm: each at one chassis height, then with
		// the chassis following its corridor. Whichever plans first, and also
		// ends its Advance before the next axle reaches its braking point.
		const bool bCorridorAllowed = CVarRammsCurbCorridor.GetValueOnGameThread() != 0;
		for (int32 Landing = 0; Landing < NumExtraLandings && !bPlanned; ++Landing)
		{
			Extra = ExtraLandingStepCm * Landing;
			for (int32 ByCorridor = 0; ByCorridor < (bCorridorAllowed ? 2 : 1) && !bPlanned; ++ByCorridor)
			{
				FString Why;
				bPlanned = PlanLanding(ByCorridor == 1, Why);
				if (Landing == 0)
				{
					(ByCorridor == 0 ? FirstReason : CorridorReason) = Why;
				}
				const float Short = bPlanned && AxleIndex < 2 ? AdvanceShort() : 0.0f;
				if (Short <= 0.0f)
				{
					continue;
				}
				bPlanned = false;
				if (LaterReason.IsEmpty())
				{
					// The front axle's centre wheels stand forward of the centre
					// of mass while it is up; say what standing them as far aft
					// as MinStabilityMargin lets them would leave.
					const float Aft = AxleIndex == 0 ? FMath::Max(0.0f, CentresAt() - (Tuning.CenterOfMassX + Tuning.MinStabilityMargin)) : 0.0f;
					LaterReason = FString::Printf(TEXT("landing %s, the %s wheels would reach their braking point %.1f cm before the %s axle "
													   "has landed%s"),
						Extra > 0.0f ? *FString::Printf(TEXT("%.0f cm further on"), Extra) : TEXT("where it should"),
						AxleName(static_cast<ERammsCurbAxle>(AxleIndex + 1)), Short, AxleName(Axle),
						Aft > 0.0f ? *FString::Printf(TEXT(" (%.1f cm with them as far aft as MinStabilityMargin lets them stand)"), Short - Aft)
								   : TEXT(""));
				}
			}
		}
		if (!bPlanned)
		{
			// Why it cannot land where the gait means it to, and (the first
			// landing that planned but left no room) why not later.
			OutReason = CorridorReason.IsEmpty() ? FirstReason : FString::Printf(TEXT("%s; and %s"), *FirstReason, *CorridorReason);
			if (!LaterReason.IsEmpty())
			{
				OutReason += FString::Printf(TEXT("; nor does a later landing help: %s"), *LaterReason);
			}
			return false;
		}
		NotePlannedMargin(Axle, LiftShift);
		if (!bOnePose)
		{
			NotePlannedMargin(Axle, DriveShift);
		}
		SFromNext = SAdv[AxleIndex] + Extra;
		Height = DriveHeight;

		FStep S;
		S.Axle = Axle;
		S.BaseHeight = LiftHeight;
		S.LegX = LiftLegX;
		S.bFollowCorridor = bCorridor;
		if (bCorridor)
		{
			S.Corridor = CorridorPath;
		}

		S.Phase = ERammsCurbPhase::Prepare;
		FMemory::Memcpy(S.Axles, Before, sizeof(Before));
		S.SpeedFraction = Tuning.OverlapSpeedFraction;
		S.Span = BeforeSpan;
		Steps.Add(S);

		// Lifting off on a reduced margin is done standing still.
		S.Phase = ERammsCurbPhase::Lift;
		FMemory::Memcpy(S.Axles, Swing, sizeof(Swing));
		S.SpeedFraction = bOnePose ? Tuning.OverlapSpeedFraction : 0.0f;
		S.Span = SwingSpan;
		Steps.Add(S);

		if (!bOnePose)
		{
			S.Phase = ERammsCurbPhase::Shift;
			S.BaseHeight = DriveHeight;
			S.LegX = DriveLegX;
			Steps.Add(S);
		}

		S.Phase = ERammsCurbPhase::Advance;
		S.AdvanceTo = AdvanceTo[AxleIndex] + Extra;
		S.SpeedFraction = 1.0f;
		Steps.Add(S);

		S.Phase = ERammsCurbPhase::Plant;
		FMemory::Memcpy(S.Axles, After, sizeof(After));
		S.SpeedFraction = Tuning.OverlapSpeedFraction;
		S.Span = AfterSpan;
		Steps.Add(S);
	}

	// Every wheel across: back to the starting ride height over the new ground,
	// all wheels down, then the starting stance.
	FStep Settle;
	Settle.Phase = ERammsCurbPhase::Settle;
	for (int32 a = 0; a < 3; ++a)
	{
		Settle.Axles[a] = { ELegGoal::Support, StepHeight, true };
	}
	Settle.LegX = InitialLegX;
	// Settle while driving clear of the edge, so the robot is not left with a
	// wheel on its lip.
	Settle.Axle = ERammsCurbAxle::Rear;
	Settle.AdvanceTo = SettleTo;
	Settle.SpeedFraction = Tuning.OverlapSpeedFraction;
	SpanGoals(Settle.Axles, InitialLegX, 0.5f, SFromNext, SSettle, SSettle, Settle.Span);
	if (!AllDown(StepHeight, true, InitialBaseHeight + StepHeight, InitialLegX, Settle.Span, Settle.BaseHeight))
	{
		OutReason = TEXT("no level stance with every wheel down is reachable on the far side");
		return false;
	}
	Steps.Add(Settle);

	PlannedMinSpeed = Tuning.DriveSpeed;
	for (const FStep& S : Steps)
	{
		PlannedMinSpeed = FMath::Min(PlannedMinSpeed, Tuning.DriveSpeed * S.SpeedFraction);
	}

	// The level-in: the approach ends, and the first Prepare brings the base
	// level from the stance the robot drove up in, where that is predicted to
	// be done LevelInMarginCm before any wheel is LevelBeforeEdgeCm from its
	// edge, at the speed it may come in at. That Prepare still ends no sooner
	// than it used to begin.
	PlannedLevelInSeconds = Steps.Num() > 1 ? StepSeconds(1, /*bPlanned=*/true) : 0.0f;
	const float LevelTo = -RammsCurb::LevelInStartCm(PlannedLevelInSeconds, VPlan, ApproachDeg);
	// The commit point: where the gait's legs first move for the
	// edge, the approach's end before the level-in moves it. The level-in
	// readies nothing for the edge, so the far side need only be seen by
	// here; while it is not, the approach does not end (StepComplete).
	CommitTo = Steps[0].ApproachTo;
	if (Steps.Num() > 1 && LevelTo < Steps[0].ApproachTo)
	{
		Steps[1].GateTo = FMath::Max(Steps[1].GateTo, Steps[0].ApproachTo);
		Steps[0].ApproachTo = LevelTo;
	}

	for (const FStep& S : Steps)
	{
		FString Line = FString::Printf(TEXT("%-8s"), PhaseName(S.Phase));
		if (PhaseNamesAxle(S.Phase))
		{
			Line += FString::Printf(TEXT(" %-6s"), AxleName(S.Axle));
		}
		Line += FString::Printf(TEXT(" chassis %.1f cm, centre wheels"), S.BaseHeight);
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			if (Legs[i].Kind == ERammsCurbLegKind::FiveBar && S.LegX.IsValidIndex(i))
			{
				Line += FString::Printf(TEXT(" %+.1f"), S.LegX[i]);
			}
		}
		Line += TEXT(" cm");
		if (S.Phase == ERammsCurbPhase::Advance || S.Phase == ERammsCurbPhase::Settle)
		{
			Line += FString::Printf(TEXT(", until %.1f cm past the edge"), S.AdvanceTo);
		}
		if (S.Phase == ERammsCurbPhase::Approach)
		{
			Line += FString::Printf(TEXT(", until the front wheels are %.1f cm from the edge, then levelling in (%.2f s)"), -S.ApproachTo,
				PlannedLevelInSeconds);
		}
		if (S.GateTo > -TNumericLimits<float>::Max())
		{
			Line += FString::Printf(TEXT(", not ending before the %s wheels are %.1f cm from the edge"), AxleName(S.Axle), -S.GateTo);
		}
		if (S.bFollowCorridor)
		{
			Line += TEXT(", the chassis following its corridor");
		}
		// What the terrain does to the legs, where it does anything.
		double Lo = 0.0, Hi = 0.0;
		for (const FVector2D& O : S.Span.Offset)
		{
			Lo = FMath::Min(Lo, O.X);
			Hi = FMath::Max(Hi, O.Y);
		}
		if (Hi - Lo > 0.05)
		{
			Line += FString::Printf(TEXT(", terrain %+.1f..%+.1f cm"), Lo, Hi);
		}
		PlanLines.Add(Line);
	}
	OutReason = FString::Printf(TEXT("%s a %.1f cm step %.1f cm ahead: %d steps, chassis %.1f..%.1f cm"),
		bClimb ? TEXT("climb") : TEXT("descend"), FMath::Abs(StepHeight), Profile.EdgeDistanceCm, Steps.Num(),
		InitialBaseHeight, Height);
	return true;
}

bool URammsCurbNegotiationController::CheckFeasibility(FRammsCurbProfile Profile, FString& OutReason)
{
	if (IsNegotiating())
	{
		OutReason = TEXT("a manoeuvre is already running");
		return false;
	}
	const bool bFeasible = BuildPlan(Profile, OutReason);
	if (!bFeasible)
	{
		Status = OutReason;
	}
	return bFeasible;
}

// --- running -------------------------------------------------------------------

void URammsCurbNegotiationController::SetActiveModeThroughSelector(FName ModeId)
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}
	if (URammsDriveModeSelector* Selector = Owner->FindComponentByClass<URammsDriveModeSelector>())
	{
		if (Selector->GetActiveModeId() != ModeId)
		{
			Selector->SetActiveMode(ModeId);
		}
		return;
	}
	// No selector: switch the modes directly, the way it would.
	for (UActorComponent* Component : Owner->GetComponents())
	{
		if (IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Component))
		{
			Mode->SetDriveModeActive(Mode->GetDriveModeId() == ModeId);
		}
	}
	if (URammsRobotControlSurfaceComponent* Surface = URammsRobotControlSurfaceComponent::FindGoverningSurface(this))
	{
		Surface->RebuildControlSurface();
	}
}

FName URammsCurbNegotiationController::OtherActiveModeId() const
{
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return NAME_None;
	}
	if (const URammsDriveModeSelector* Selector = Owner->FindComponentByClass<URammsDriveModeSelector>())
	{
		const FName Active = Selector->GetActiveModeId();
		return Active != GetDriveModeId() ? Active : NAME_None;
	}
	for (UActorComponent* Component : Owner->GetComponents())
	{
		const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Component);
		if (Mode && Mode != this && Mode->IsDriveModeActive())
		{
			return Mode->GetDriveModeId();
		}
	}
	return NAME_None;
}

FName URammsCurbNegotiationController::FallbackModeId() const
{
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return NAME_None;
	}
	if (const URammsDriveModeSelector* Selector = Owner->FindComponentByClass<URammsDriveModeSelector>())
	{
		if (!Selector->InitialModeId.IsNone() && Selector->InitialModeId != GetDriveModeId())
		{
			return Selector->InitialModeId;
		}
		for (const FName Id : Selector->GetAvailableModeIds())
		{
			if (Id != GetDriveModeId())
			{
				return Id;
			}
		}
		return NAME_None;
	}
	for (UActorComponent* Component : Owner->GetComponents())
	{
		const IRammsDriveMode* Mode = Cast<IRammsDriveMode>(Component);
		if (Mode && Mode != this)
		{
			return Mode->GetDriveModeId();
		}
	}
	return NAME_None;
}

bool URammsCurbNegotiationController::BeginNegotiation(FRammsCurbProfile Profile, FString& OutReason)
{
	if (IsNegotiating())
	{
		OutReason = TEXT("a manoeuvre is already running");
		return false;
	}
	if (!BuildPlan(Profile, OutReason))
	{
		UE_LOG(LogRammsCurb, Warning, TEXT("[Curb] '%s' will not attempt it: %s"), *GetNameSafe(GetOwner()), *OutReason);
		Status = OutReason;
		return false;
	}
	UE_LOG(LogRammsCurb, Log, TEXT("[Curb] '%s': %s"), *GetNameSafe(GetOwner()), *OutReason);
	for (const FString& Line : PlanLines)
	{
		UE_LOG(LogRammsCurb, Log, TEXT("[Curb]   %s"), *Line);
	}
	// The leg and wheel rates start at rest, and from the first measurement
	// the gait's own ticks take: this one (BuildPlan's) was made off the
	// physics clock those ticks keep.
	for (FLegState& State : LegState)
	{
		State.bHasLastWheel = false;
		State.LagRate = FVector2D::ZeroVector;
	}
	if (Drive)
	{
		Drive->Feeds.Init(RammsCurb::FWheelFeed(), Legs.Num());
		Drive->Stall.Reset();
		Drive->CommitRest.Reset();
	}

	// Take over the wheels from whatever mode has them, and remember it -- only
	// when it is another mode: a retry after an abort or a fault starts with
	// this one still selected, and must still hand back to the one before.
	const FName Other = OtherActiveModeId();
	if (!Other.IsNone())
	{
		PreviousModeId = Other;
	}
	SetActiveModeThroughSelector(GetDriveModeId());
	bDriveModeActive = true;
	bHandBackPending = false;

	ActiveProfile = Profile;
	LastSimTime = -1.0;
	FilteredPitchRad = FMath::DegreesToRadians(MeasuredPitchDeg);
	FilteredRollRad = FMath::DegreesToRadians(MeasuredRollDeg);
	BaseHeightCmd = InitialBaseHeight;
	CorridorShiftCmd = 0.0f;
	// The level base starts afresh: no trim, no bias, not yet levelled in.
	if (Level)
	{
		Level->Reset();
	}
	// From the speed the robot has, not from a standstill: a crossing asked for
	// on the move carries on at it, and the ramp takes it from there.
	DriveSpeed = MeasuredWheelSpeed();
	Elapsed = 0.0;
	StepLog.Reset();
	if (Telemetry)
	{
		// The per-axle gait plans a level chassis and no timing.
		Telemetry->Stats.Begin(PlannedMinSpeed, 0.0f, 0.0f,
			bPlannedMargin ? TOptional<float>(PlannedMinMargin) : TOptional<float>(),
			Tuning.DriveSpeed * RammsCurb::CruiseFloorFraction);
		FVector2D Centre, Forward;
		if (LocateChassis(Centre, Forward))
		{
			Telemetry->StartPosition = Centre;
			Telemetry->StartForward = Forward;
		}
	}
	// The plan's own detection looked from here: the approach looks again
	// from RedetectEveryCm on.
	{
		FVector2D Centre, Forward;
		bHasLastLook = LocateChassis(Centre, Forward);
		LastLookCentre = Centre;
		LastLookFailure.Reset();
	}
	EnterStep(0);
	if (bFarDepthPending)
	{
		const FString Line = FString::Printf(TEXT("t=%5.2fs far side not yet confirmed: %s"), Elapsed, *FarDepthWhy);
		StepLog.Add(Line);
		UE_LOG(LogRammsCurb, Log, TEXT("[Curb] %s"), *Line);
	}
	return true;
}

bool URammsCurbNegotiationController::NegotiateStepAhead(FString& OutReason)
{
	FRammsCurbProfile Profile;
	if (!DetectStepAhead(Profile, OutReason))
	{
		Status = OutReason;
		UE_LOG(LogRammsCurb, Warning, TEXT("[Curb] '%s': %s"), *GetNameSafe(GetOwner()), *OutReason);
		return false;
	}
	return BeginNegotiation(Profile, OutReason);
}

void URammsCurbNegotiationController::Abort()
{
	if (IsNegotiating())
	{
		Finish(ERammsCurbPhase::Aborted, TEXT("aborted"));
	}
}

void URammsCurbNegotiationController::EnterStep(int32 Index)
{
	StepIndex = Index;
	StepElapsed = 0.0;
	const FStep& S = Steps[Index];
	Phase = S.Phase;
	Status = PhaseNamesAxle(S.Phase) ? FString::Printf(TEXT("%s %s"), PhaseName(S.Phase), AxleName(S.Axle))
									 : FString(PhaseName(S.Phase));
	FString		Line = FString::Printf(TEXT("t=%5.2fs %-14s chassis %.1f (measured %.1f) cm, pitch %+.1f deg, speed %.1f cm/s"),
			Elapsed, *Status, S.BaseHeight, MeasuredBaseHeight, MeasuredPitchDeg, DriveSpeed);
	const float Deadline = DeadlineCap();
	if (Deadline < TNumericLimits<float>::Max())
	{
		Line += FString::Printf(TEXT(", deadline %.1f cm/s"), Deadline);
	}
	StepLog.Add(Line);
	UE_LOG(LogRammsCurb, Log, TEXT("[Curb] %s"), *Line);
	// A step planned at a standstill after one that was not is a stop the plan
	// makes; the steps after it at a standstill are the same stop.
	if (Telemetry && S.SpeedFraction <= 0.0f && (Index == 0 || Steps[Index - 1].SpeedFraction > 0.0f))
	{
		Telemetry->Stats.AddPlannedStop();
	}
}

void URammsCurbNegotiationController::Finish(ERammsCurbPhase EndPhase, const FString& Why)
{
	Phase = EndPhase;
	Status = Why;
	const FString Line = FString::Printf(TEXT("t=%5.2fs %s: %s"), Elapsed, PhaseName(EndPhase), *Why);
	StepLog.Add(Line);
	UE_LOG(LogRammsCurb, Log, TEXT("[Curb] %s"), *Line);
	// What each wheel feed learnt about the link its wheel hangs from: how
	// much of a leg's motion turns up as the wheel's turn.
	for (int32 i = 0; Drive && i < Legs.Num() && i < Drive->Feeds.Num(); ++i)
	{
		const FVector2D Gains = Drive->Feeds[i].GetFit().GetGains();
		if (!Gains.IsNearlyZero(1e-3))
		{
			const FString Fit = FString::Printf(TEXT("        %s: the link its wheel hangs from turned %+.3f rad per cm the leg "
													 "moved fore/aft, %+.3f per cm up (fitted)"),
				*Legs[i].Name.ToString(), Gains.X, Gains.Y);
			StepLog.Add(Fit);
			UE_LOG(LogRammsCurb, Log, TEXT("[Curb] %s"), *Fit);
		}
	}
	// And what the level base learnt: the attitude loop's trim, and how far
	// the planted wheels found each surface off the plane detected.
	if (Level && Level->bLevelled)
	{
		const FVector2D Trim = Level->Attitude.GetIntegral();
		const FVector2D Bias = Level->GetBias();
		const FString	LevelLine = FString::Printf(TEXT("        level: attitude trim %+.2f deg pitch, %+.2f deg roll; surfaces off "
														   "their planes by %+.2f cm (near), %+.2f cm (far)"),
			  Trim.X, Trim.Y, Bias.X, Bias.Y);
		StepLog.Add(LevelLine);
		UE_LOG(LogRammsCurb, Log, TEXT("[Curb] %s"), *LevelLine);
	}
	DriveSpeed = 0.0f;
	if (Telemetry)
	{
		Telemetry->Stats.End(Elapsed, MeasuredPitchDeg, MeasuredRollDeg);
	}

	if (EndPhase == ERammsCurbPhase::Done)
	{
		StopWheels(/*bRelease=*/true);
		FName Next = !ReturnModeId.IsNone() ? ReturnModeId : PreviousModeId;
		if (Next.IsNone() || Next == GetDriveModeId())
		{
			Next = FallbackModeId();
		}
		if (!Next.IsNone())
		{
			SetActiveModeThroughSelector(Next);
		}
	}
	else
	{
		// Hold: brake the wheels and leave the legs on their last targets. The
		// robot may be standing across the curb on two axles; letting anything
		// go here is how it would fall.
		StopWheels(/*bRelease=*/false);
	}
}

void URammsCurbNegotiationController::SetDriveModeActive(bool bActive)
{
	if (bDriveModeActive == bActive)
	{
		return;
	}
	if (bActive)
	{
		// The selector still reports the outgoing mode while it switches.
		const FName Other = OtherActiveModeId();
		if (!Other.IsNone())
		{
			PreviousModeId = Other;
		}
		// BeginNegotiation clears this straight after selecting the mode; any
		// other way in -- the mode list, cycling, a selector whose initial mode
		// is this -- leaves the robot in a mode with nothing to do.
		bHandBackPending = true;
	}
	bDriveModeActive = bActive;
	if (!bActive)
	{
		bHandBackPending = false;
		if (IsNegotiating())
		{
			Finish(ERammsCurbPhase::Aborted, TEXT("another drive mode was selected"));
		}
		if (bWheelsDriven)
		{
			StopWheels(/*bRelease=*/true);
		}
	}
}

float URammsCurbNegotiationController::AxleProgress(ERammsCurbAxle Axle) const
{
	float Least = TNumericLimits<float>::Max();
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		if (Legs[i].Axle == Axle && LegState[i].bHasWorld)
		{
			Least = FMath::Min(Least, LegState[i].Progress);
		}
	}
	return Least;
}

bool URammsCurbNegotiationController::LegsSettled() const
{
	for (const FLegState& State : LegState)
	{
		if (FMath::Abs(State.MeasuredZ - State.CmdZ) > Tuning.SettleTolerance)
		{
			return false;
		}
		// A 5-bar's fore/aft offset is what keeps the centre of mass between
		// the two axles that will carry the robot, so it too has to be measured
		// where it was sent before the next step lifts an axle or drives on it.
		if (State.FiveBar && FMath::Abs(State.MeasuredX - State.CmdX) > Tuning.SettleTolerance)
		{
			return false;
		}
	}
	return true;
}

void URammsCurbNegotiationController::CommandLegs(float RampStep)
{
	URammsRobotBaseComponent* Base = EnsureBase();
	const FStep&			  S = Steps[StepIndex];
	BaseHeightCmd = StepToward(BaseHeightCmd, S.BaseHeight, Tuning.BaseHeightSpeed * RampStep);

	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FRammsCurbLeg& Leg = Legs[i];
		FLegState&			 State = LegState[i];
		const FAxleGoal&	 Goal = S.Axles[static_cast<int32>(Leg.Axle)];
		const float			 GoalZ = CommandedGoalHeight(i, Goal);
		const float			 PrevZ = State.CmdZ;
		State.GoalZ = GoalZ;
		State.CmdZ = StepToward(State.CmdZ, GoalZ, Tuning.LegSpeed * RampStep);

		if (Leg.Kind == ERammsCurbLegKind::FourBar)
		{
			bool		bReachable = false;
			float		WheelX = 0.0f;
			const float Crank = URamms4BarKinematics::SolveCrankForHeight(Leg.FourBar, State.CmdZ, bReachable, WheelX);
			Base->SetMotorCommand(Leg.FourBar.CrankMotor, Crank);
			State.ModelX = WheelX;
		}
		else if (State.FiveBar)
		{
			const float TargetX = S.LegX.IsValidIndex(i) ? S.LegX[i] : State.InitialX;
			const float GoalX = StepToward(State.CmdX, TargetX, Tuning.CenterShiftSpeed * RampStep);
			const float PrevCmdX = State.CmdX;
			// A 5-bar's region is curved, so a straight move between two
			// reachable points can cross an unreachable corner. Take whichever
			// half of the move the linkage accepts and finish it next frame.
			if (State.FiveBar->SetEndpointTarget(FVector2D(GoalX, State.CmdZ)))
			{
				State.CmdX = GoalX;
			}
			else if (State.FiveBar->SetEndpointTarget(FVector2D(PrevCmdX, State.CmdZ)))
			{
			}
			else if (State.FiveBar->SetEndpointTarget(FVector2D(GoalX, PrevZ)))
			{
				State.CmdX = GoalX;
				State.CmdZ = PrevZ;
			}
			else
			{
				State.CmdZ = PrevZ;
			}
			State.ModelX = State.CmdX;
		}
	}
}

bool URammsCurbNegotiationController::BrakingLimit(int32 LegIndex, float& OutDistanceCm) const
{
	const FLegState& State = LegState[LegIndex];
	if (!State.bHasWorld)
	{
		return false;
	}
	const float		StepHeight = ActiveProfile.StepHeightCm;
	const float		R = Legs[LegIndex].WheelRadiusCm;
	const FVector2D P(State.WheelWorld.X, State.WheelWorld.Y);
	const FVector2D Bias = Level ? Level->GetBias() : FVector2D::ZeroVector;
	// Each against the surfaces under the wheel itself: its bottom over the
	// curb top, or nearer the higher surface than the lower one.
	const double Bottom = State.WheelWorld.Z - R;
	const double Middle = 0.5 * (ActiveTerrain.Near.HeightAt(P) + Bias.X + ActiveTerrain.Far.HeightAt(P) + Bias.Y);
	const bool	 bClear = Bottom >= ClearBottomZ(LegIndex);
	if (StepHeight > 0.0f)
	{
		// A wheel on the high side is past the face and limits nothing. Any
		// other wheel not yet clear of the curb top must stop short of it.
		const bool bOnTop = State.Progress > 0.0f && Bottom > Middle;
		if (bClear || bOnTop)
		{
			return false;
		}
		OutDistanceCm = -Tuning.FaceMargin - (State.Progress + R);
		return true;
	}
	// A wheel still on the high side, carrying, must not roll off it. One
	// already out past the edge -- an axle being lowered, which Advance left a
	// radius and more beyond it -- is not on the high side whatever its height
	// on the way down, and limits nothing; nor does a lifted one past the lip,
	// which got there clear and carries nothing (its rim curves away from the
	// lip, so it is carried lower as it goes on).
	if (LegGoal(LegIndex).Goal == ELegGoal::Swing && State.Progress >= 0.0f)
	{
		return false;
	}
	const bool bOnTop = Bottom > Middle && State.Progress < R;
	if (bClear || !bOnTop)
	{
		return false;
	}
	OutDistanceCm = -Tuning.EdgeMargin - State.Progress;
	return true;
}

float URammsCurbNegotiationController::StepSeconds(int32 Index, bool bPlanned) const
{
	// Every leg moves at once: the step lasts as long as its slowest move, the
	// command's ramp and then the servo lag's tail down to SettleTolerance,
	// which is when StepComplete lets the gait on. The current step's goals
	// are over the terrain as it is now; a later one's over what its plan
	// expects there.
	const FStep& S = Steps[Index];
	const bool	 bLive = !bPlanned && Index == StepIndex;
	const FStep* Before = Index > 0 ? &Steps[Index - 1] : nullptr;
	const float	 Tau = Tuning.ServoLagSeconds;
	const float	 Tol = Tuning.SettleTolerance;
	const float	 FromBase = bLive ? BaseHeightCmd : (Before ? Before->BaseHeight : InitialBaseHeight);
	// A corridor's shift off the plan height comes back to 0 alongside (and
	// StepComplete waits for it) where this step does not follow one.
	const float ShiftBack = bLive && !S.bFollowCorridor ? FMath::Abs(CorridorShiftCmd) : 0.0f;
	const float BaseSeconds = FMath::Max(FMath::Abs(S.BaseHeight - FromBase), ShiftBack) / Tuning.BaseHeightSpeed;
	float		Seconds = BaseSeconds;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FLegState& State = LegState[i];
		const int32		 A = static_cast<int32>(Legs[i].Axle);
		const float		 ToZ = bLive ? GoalHeight(i, S.Axles[A], S.BaseHeight, State.TerrainOffset) : PlannedGoalHeight(Index, i);
		const float		 FromZ = bLive ? State.CmdZ : (Before ? PlannedGoalHeight(Index - 1, i) : State.InitialZ);
		// A support leg's goal moves with the chassis, so its command can be
		// no quicker than the chassis' ramp.
		const float Dz = FMath::Abs(ToZ - FromZ);
		const float Ramp = FMath::Max(BaseSeconds, Dz / Tuning.LegSpeed);
		const float Rate = Ramp > KINDA_SMALL_NUMBER ? Dz / Ramp : Tuning.LegSpeed;
		Seconds = FMath::Max(Seconds,
			RammsCurb::LegMoveSeconds(Dz, Rate, bLive ? State.MeasuredZ - State.CmdZ : 0.0f, Tau, Tol));
		if (State.FiveBar)
		{
			const float ToX = S.LegX.IsValidIndex(i) ? S.LegX[i] : State.InitialX;
			const float FromX = bLive ? State.CmdX : (Before && Before->LegX.IsValidIndex(i) ? Before->LegX[i] : State.InitialX);
			Seconds = FMath::Max(Seconds, RammsCurb::LegMoveSeconds(ToX - FromX, Tuning.CenterShiftSpeed, bLive ? State.MeasuredX - State.CmdX : 0.0f, Tau, Tol));
		}
	}
	return Seconds;
}

float URammsCurbNegotiationController::LiftClearSeconds(int32 Index) const
{
	// The lifted wheels need only come clear (what lets their braking limit
	// go), not finish the step: clear is SwingClearance / 2 short of the swing
	// height the step sends them to.
	const FStep& S = Steps[Index];
	const bool	 bLive = Index == StepIndex;
	float		 Seconds = 0.0f;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		if (Legs[i].Axle != S.Axle)
		{
			continue;
		}
		const FLegState& State = LegState[i];
		if (bLive)
		{
			// From where the wheel is measured over the terrain.
			const float Need = static_cast<float>(ClearBottomZ(i) - (State.WheelWorld.Z - Legs[i].WheelRadiusCm));
			Seconds = FMath::Max(Seconds, RammsCurb::LegReachSeconds(State.MeasuredZ, State.CmdZ, Tuning.LegSpeed, State.MeasuredZ + Need, Tuning.ServoLagSeconds));
			continue;
		}
		const float ClearZ = PlannedGoalHeight(Index, i) - 0.5f * Tuning.SwingClearance;
		const float FromZ = Index > 0 ? PlannedGoalHeight(Index - 1, i) : State.InitialZ;
		Seconds = FMath::Max(Seconds, RammsCurb::LegReachSeconds(FromZ, FromZ, Tuning.LegSpeed, ClearZ, Tuning.ServoLagSeconds));
	}
	return Seconds;
}

float URammsCurbNegotiationController::DeadlineCap() const
{
	const float None = TNumericLimits<float>::Max();
	if (!Steps.IsValidIndex(StepIndex) || LegState.Num() != Legs.Num())
	{
		return None;
	}
	// The next lift whose axle a braking limit still holds back. An axle
	// already clear -- the front corners the drive stance carries up, on a
	// descent -- needs no deadline; the one after it does.
	auto Held = [this](ERammsCurbAxle Axle) {
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			float Distance = 0.0f;
			if (Legs[i].Axle == Axle && BrakingLimit(i, Distance))
			{
				return true;
			}
		}
		return false;
	};
	int32 Lift = INDEX_NONE;
	for (int32 k = StepIndex; k < Steps.Num(); ++k)
	{
		if (Steps[k].Phase == ERammsCurbPhase::Lift && Held(Steps[k].Axle))
		{
			Lift = k;
			break;
		}
	}
	if (Lift == INDEX_NONE)
	{
		return None;
	}

	// The room: how far the chassis can go before that axle's nearest wheel
	// reaches its braking point, plus however far the leg carries the wheel
	// aft (relative to the chassis) before it lifts -- a centre wheel walked
	// aft arrives that much later.
	const FStep& L = Steps[Lift];
	float		 Room = None;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		float Distance = 0.0f;
		if (Legs[i].Axle != L.Axle || !BrakingLimit(i, Distance))
		{
			continue;
		}
		const FLegState& State = LegState[i];
		float			 LiftX = State.MeasuredX;
		if (State.FiveBar)
		{
			LiftX = L.LegX.IsValidIndex(i) ? L.LegX[i] : State.InitialX;
		}
		else if (Legs[i].Kind == ERammsCurbLegKind::FourBar)
		{
			// A 4-bar carries its wheel fore/aft as it lifts it.
			LiftX = FourBarX(i, PlannedGoalHeight(Lift, i) - 0.5f * Tuning.SwingClearance, State.MeasuredX);
		}
		Room = FMath::Min(Room, Distance + (State.MeasuredX - LiftX));
	}
	if (Room == None)
	{
		return None;
	}

	// The work in front of the lift: the steps before it, each waiting on its
	// legs or on distance, then the lift itself up to clear.
	RammsCurb::FDeadlineChain Chain;
	for (int32 k = StepIndex; k <= Lift; ++k)
	{
		const FStep& S = Steps[k];
		if (k == Lift)
		{
			Chain.AddTime(LiftClearSeconds(k));
		}
		else if (S.Phase == ERammsCurbPhase::Approach)
		{
			Chain.AddGate(S.ApproachTo - AxleProgress(ERammsCurbAxle::Front));
		}
		else if (S.Phase == ERammsCurbPhase::Advance)
		{
			Chain.AddGate(S.AdvanceTo - AxleProgress(S.Axle));
		}
		else
		{
			Chain.AddTime(StepSeconds(k));
			// The level-in's Prepare also ends no sooner than the front
			// wheels reach where it used to begin.
			if (S.GateTo > -TNumericLimits<float>::Max())
			{
				Chain.AddGate(S.GateTo - AxleProgress(S.Axle));
			}
		}
	}
	return Chain.Speed(Room, Tuning.Deceleration);
}

float URammsCurbNegotiationController::ComputeDriveSpeed(float DeltaTime) const
{
	// Towards the step's speed, no faster than the deadline lets the next lift
	// keep up, ramped at Acceleration / Deceleration: every term moves
	// continuously, and none of them asks for a standstill. The braking caps
	// of the hard stops ahead are limits on top, and the only terms that reach
	// zero (RammsCurb::NextDriveSpeed).
	TArray<float, TInlineAllocator<8>> Braking;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		float Distance = 0.0f;
		if (BrakingLimit(i, Distance))
		{
			Braking.Add(Distance);
		}
	}
	// A far side not yet seen deep enough: the approach cannot end, and the
	// robot comes to rest at its end, the commit point. Braked on the
	// distance less what the chassis, running a lag behind its command, will
	// still cover, so that the command falls at Deceleration and the chassis
	// follows it to rest there -- not on the raw distance, which the chassis
	// eats into faster than the command slows, so that the command collapses
	// to 0 over the last few cm with the chassis still rolling.
	if (bFarDepthPending)
	{
		Braking.Add(RammsCurb::LagBrakingDistance(FarDepthStopDistance(), MeasuredWheelSpeed(), RammsCurb::DriveLagSeconds));
	}
	// A level-in that is late slows the robot rather than faulting it: it
	// goes no nearer the edge than the line the base must be level by.
	const float Deadline = FMath::Min(DeadlineCap(), LateLevelCap());
	return RammsCurb::NextDriveSpeed(Tuning, DriveSpeed, Steps[StepIndex].SpeedFraction, Deadline, Braking, DeltaTime);
}

float URammsCurbNegotiationController::MeasuredWheelSpeed() const
{
	const URammsRobotBaseComponent*						Base = EnsureBase();
	TArray<RammsCurb::FEntryWheel, TInlineAllocator<8>> Wheels;
	for (int32 i = 0; Base && i < Legs.Num(); ++i)
	{
		const FRammsCurbLeg&	Leg = Legs[i];
		const FLegState&		State = LegState[i];
		RammsCurb::FEntryWheel& Wheel = Wheels.AddDefaulted_GetRef();
		Wheel.bMain = Leg.Drive == ERammsCurbWheelDrive::Main;
		Wheel.bOnGround = State.bHasWorld && State.WheelBottom <= ContactToleranceCm;
		Wheel.JointRate = Base->GetMotorVelocity(Leg.Wheel.MotorId) * (Leg.Wheel.bInvert ? -1.0f : 1.0f);
		Wheel.RadiusCm = Leg.WheelRadiusCm;
		Wheel.RollForward = FMath::Cos(FMath::DegreesToRadians(Leg.Wheel.RollDirectionDeg));
	}
	// With no main wheel on the ground: the chassis' own measured speed.
	return RammsCurb::EntrySpeed(Wheels, Telemetry && Telemetry->Motion.IsFull(),
		Telemetry ? Telemetry->Motion.GetSpeed() : 0.0f);
}

void URammsCurbNegotiationController::TrackLegRates(float DeltaTime)
{
	// The axis a wheel rolling forward along its roll direction d turns about
	// is Up x d: the chassis' right for d forward. Its world turn about that
	// axis, less its joint's, is the turn of the link it hangs from -- the
	// joint's as the backend's speed loop sees it: from its angle where the
	// backend reports one, else from its rate (RammsCurb::CarrierRate).
	if (!Drive)
	{
		return;
	}
	Drive->Feeds.SetNum(Legs.Num());
	const FVector Forward = ChassisForward;
	const FVector Right = FVector::CrossProduct(FVector::UpVector, Forward);
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FRammsCurbLeg&   Leg = Legs[i];
		FLegState&			   State = LegState[i];
		RammsCurb::FWheelFeed& Feed = Drive->Feeds[i];
		if (!State.bHasWorld)
		{
			continue;
		}
		float Carrier = 0.0f;
		if (State.bHasLastWheel && DeltaTime > 0.0f && DeltaTime <= MaxWheelTurnSampleSeconds)
		{
			const double  Theta = FMath::DegreesToRadians(Leg.Wheel.RollDirectionDeg);
			const FVector Axis = (Right * FMath::Cos(Theta) - Forward * FMath::Sin(Theta)).GetSafeNormal();
			const double  World = RammsCurb::TurnAbout(State.LastWheelRotation, State.WheelRotation, Axis);
			const float	  Sense = Leg.Wheel.bInvert ? -1.0f : 1.0f;
			Carrier = RammsCurb::CarrierRate(World, Sense * State.LastWheelAngle, Sense * State.WheelAngle,
				Sense * State.LastWheelRate, Sense * State.WheelRate, DeltaTime);
		}
		else
		{
			// Nothing to measure a rate over, or a gap too long to trust a turn
			// across: start the leg's feed again from here.
			Feed.Reset();
		}
		Feed.Measure(DeltaTime, FVector2D(State.MeasuredX, State.MeasuredZ), Carrier, State.LagRate);
		State.LastWheelRotation = State.WheelRotation;
		State.LastWheelAngle = State.WheelAngle;
		State.LastWheelRate = State.WheelRate;
		State.bHasLastWheel = true;
	}
}

void URammsCurbNegotiationController::PredictLegRates(float DeltaTime)
{
	if (!Drive || DeltaTime <= 0.0f)
	{
		return;
	}
	Drive->Feeds.SetNum(Legs.Num());
	const float Tau = Tuning.ServoLagSeconds;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FLegState& State = LegState[i];
		const float		 NextZ = RammsCurb::LagStep(State.LagZ, State.CmdZ, DeltaTime, Tau);
		const float		 NextX = State.FiveBar ? RammsCurb::LagStep(State.LagX, State.CmdX, DeltaTime, Tau)
											   : (Legs[i].Kind == ERammsCurbLegKind::FourBar ? FourBarX(i, NextZ, State.LagX) : State.LagX);
		Drive->Feeds[i].Predict((FVector2D(NextX, NextZ) - FVector2D(State.LagX, State.LagZ)) / DeltaTime);
	}
}

float URammsCurbNegotiationController::FourBarX(int32 LegIndex, float Z, float Fallback) const
{
	bool  bReachable = false;
	float WheelX = 0.0f;
	URamms4BarKinematics::SolveCrankForHeight(Legs[LegIndex].FourBar, Z, bReachable, WheelX);
	return bReachable ? WheelX : Fallback;
}

URammsCurbNegotiationController::FAxleGoal URammsCurbNegotiationController::LegGoal(int32 LegIndex) const
{
	if (Steps.IsValidIndex(StepIndex) && Legs.IsValidIndex(LegIndex))
	{
		return Steps[StepIndex].Axles[static_cast<int32>(Legs[LegIndex].Axle)];
	}
	// Before any step: where it stands, on the near side.
	return { ELegGoal::Hold, 0.0f, false };
}

bool URammsCurbNegotiationController::IsLegPlanted(int32 LegIndex) const
{
	// On the ground means both: the step wants the wheel down, and it is --
	// measured, within a wheel's settling of the surface it follows. The step
	// alone would hand drive back while a landing wheel is still in the air,
	// and the measurement alone would keep it on a wheel being lifted.
	const FLegState& State = LegState[LegIndex];
	return LegGoal(LegIndex).Goal != ELegGoal::Swing && State.bHasWorld && State.WheelBottom <= ContactToleranceCm;
}

void URammsCurbNegotiationController::CommandWheels(float Speed, float YawRate)
{
	URammsRobotBaseComponent* Base = EnsureBase();
	// Drive on the main wheels whenever they are planted, as the differential
	// mode does, side by side: a side whose main wheel is off the ground hands
	// its drive to that side's fallback wheels (RammsCurb::DriveRoles).
	TArray<RammsCurb::FDriveWheel, TInlineAllocator<8>> Roles;
	TArray<bool, TInlineAllocator<8>>					Driven;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		RammsCurb::FDriveWheel& W = Roles.AddDefaulted_GetRef();
		W.Role = Legs[i].Drive;
		W.bPlusY = IsPlusY(Legs[i]);
		W.bPlanted = IsLegPlanted(i);
	}
	Driven.Init(false, Legs.Num());
	RammsCurb::DriveRoles(Roles, Driven);
	const bool bCarrierTerm = CVarRammsCurbCarrierSpin.GetValueOnGameThread() != 0;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FRammsCurbLeg& Leg = Legs[i];
		const FLegState&	 State = LegState[i];
		const bool			 bDriven = Driven[i];
		if (!bDriven)
		{
			// Rolling free: no speed loop, no torque.
			Base->ClearMotorVelocityCommand(Leg.Wheel.MotorId);
			Base->SetMotorCommand(Leg.Wheel.MotorId, 0.0f);
			continue;
		}
		const RammsCurb::FWheelFeed* Feed = Drive && Drive->Feeds.IsValidIndex(i) ? &Drive->Feeds[i] : nullptr;
		const double				 Theta = FMath::DegreesToRadians(Leg.Wheel.RollDirectionDeg);
		const double				 Dx = FMath::Cos(Theta);
		const double				 Dy = FMath::Sin(Theta);
		// The wheel's centre moves over the ground at the chassis speed plus
		// however fast its leg carries it fore/aft -- as measured, brought up to
		// the tick to come; a wheel that does not roll along with its leg drags
		// the robot instead.
		const double Vx = Speed + (Feed ? Feed->GetLegXRate() : 0.0f);
		const double Ground = Dx * Vx + YawRate * (State.MeasuredX * Dy - Leg.Wheel.Position.Y * Dx);
		double		 Rate = Ground / FMath::Max(Leg.WheelRadiusCm, 0.1f);
		// The motor turns the wheel relative to the link it hangs from. When
		// that link turns too -- a 5-bar's as it walks the wheel fore/aft, a
		// swing arm's as it lifts -- the wheel's turn over the ground is the sum
		// of the two, so the link's share comes off the joint's.
		if (bCarrierTerm && Feed)
		{
			Rate -= Feed->GetCarrierRate();
		}
		Base->SetMotorVelocityCommand(Leg.Wheel.MotorId, static_cast<float>(Leg.Wheel.bInvert ? -Rate : Rate));
	}
	bWheelsDriven = true;
}

void URammsCurbNegotiationController::StopWheels(bool bRelease)
{
	URammsRobotBaseComponent* Base = EnsureBase();
	if (!Base)
	{
		return;
	}
	for (const FRammsCurbLeg& Leg : Legs)
	{
		if (bRelease)
		{
			Base->ClearMotorVelocityCommand(Leg.Wheel.MotorId);
			Base->SetMotorCommand(Leg.Wheel.MotorId, 0.0f);
		}
		else
		{
			Base->SetMotorVelocityCommand(Leg.Wheel.MotorId, 0.0f);
		}
	}
	bWheelsDriven = !bRelease;
}

bool URammsCurbNegotiationController::StepComplete() const
{
	const FStep& S = Steps[StepIndex];
	switch (S.Phase)
	{
		case ERammsCurbPhase::Approach:
			// Not with the far side still unconfirmed: the robot drives on in
			// its own stance and comes to rest at the commit point, which lies
			// at or beyond the approach's end. The level-in, which would raise
			// and walk the legs while it brakes (carrying the chassis on ahead
			// of its wheels), waits for the far side; once it is seen, a
			// level-in begun late slows the robot (LateLevelCap).
			return !bFarDepthPending && AxleProgress(ERammsCurbAxle::Front) >= S.ApproachTo;
		case ERammsCurbPhase::Advance:
			return AxleProgress(S.Axle) >= S.AdvanceTo;
		case ERammsCurbPhase::Settle:
			if (AxleProgress(S.Axle) < S.AdvanceTo)
			{
				return false;
			}
			break;
		default:
			break;
	}
	// The level-in's Prepare: no sooner than the front wheels reach where it
	// used to begin; and the base level within LevelInToleranceDeg, which the
	// first lift waits for.
	if (S.GateTo > -TNumericLimits<float>::Max() && AxleProgress(S.Axle) < S.GateTo)
	{
		return false;
	}
	if (S.Phase == ERammsCurbPhase::Prepare && Level && !Level->bLevelled)
	{
		return false;
	}
	if (!FMath::IsNearlyEqual(BaseHeightCmd, S.BaseHeight, 1e-3f))
	{
		return false;
	}
	// A step that does not follow a corridor brings the chassis back to its
	// planned height first: the next lift's reach was planned there.
	if (!S.bFollowCorridor && !FMath::IsNearlyZero(CorridorShiftCmd, 1e-3f))
	{
		return false;
	}
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		// Each leg's command at the goal it ramped towards this tick: over the
		// terrain under its wheel, which moves as the robot does.
		if (!FMath::IsNearlyEqual(LegState[i].CmdZ, LegState[i].GoalZ, 1e-3f))
		{
			return false;
		}
		const float TargetX = S.LegX.IsValidIndex(i) ? S.LegX[i] : LegState[i].InitialX;
		if (LegState[i].FiveBar && !FMath::IsNearlyEqual(LegState[i].CmdX, TargetX, 1e-3f))
		{
			return false;
		}
	}
	return LegsSettled();
}

void URammsCurbNegotiationController::StepPlan(float DeltaTime)
{
	// The clocks and the pitch filter take every simulated second, so elapsed
	// time and the timeouts stay the physics' own; only the ramps are held to
	// MaxRampStep.
	Elapsed += DeltaTime;
	StepElapsed += DeltaTime;
	const float RampStep = FMath::Min(DeltaTime, MaxRampStep);
	FString		Unmeasured;
	if (!Measure(Unmeasured))
	{
		// Every check below reads the measurement -- tilt, the step timeout,
		// where each leg is -- and skipping them would leave the wheels rolling
		// on at their last speed with nothing watching. Stop and hold, as a
		// tilt does.
		Finish(ERammsCurbPhase::Faulted, FString::Printf(TEXT("cannot measure the robot during %s: %s"), *Status, *Unmeasured));
		return;
	}
	// Against the commands that stood over the time just simulated, before
	// this tick replaces them.
	const float LegError = AdvanceLegLag(DeltaTime);
	TrackLegRates(DeltaTime);
	const float PitchRad = FMath::DegreesToRadians(MeasuredPitchDeg);
	const float RollRad = FMath::DegreesToRadians(MeasuredRollDeg);
	FilteredPitchRad += (PitchRad - FilteredPitchRad) * FMath::Clamp(DeltaTime / 0.15f, 0.0f, 1.0f);
	FilteredRollRad += (RollRad - FilteredRollRad) * FMath::Clamp(DeltaTime / 0.15f, 0.0f, 1.0f);

	if (FMath::Abs(MeasuredPitchDeg) > Tuning.MaxTiltDeg || FMath::Abs(MeasuredRollDeg) > Tuning.MaxTiltDeg)
	{
		Finish(ERammsCurbPhase::Faulted,
			FString::Printf(TEXT("tilted %.1f deg pitch / %.1f deg roll during %s"), MeasuredPitchDeg,
				MeasuredRollDeg, *Status));
		return;
	}
	const float Timeout = Phase == ERammsCurbPhase::Approach ? Tuning.ApproachTimeout : Tuning.StepTimeout;
	if (StepElapsed > Timeout)
	{
		FString Lagging;
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			const FLegState& State = LegState[i];
			if (FMath::Abs(State.MeasuredZ - State.CmdZ) > Tuning.SettleTolerance)
			{
				Lagging += FString::Printf(TEXT(" %s(%.1f vs %.1f)"), *Legs[i].Name.ToString(), State.MeasuredZ, State.CmdZ);
			}
			if (State.FiveBar && FMath::Abs(State.MeasuredX - State.CmdX) > Tuning.SettleTolerance)
			{
				Lagging += FString::Printf(TEXT(" %s(fore/aft %.1f vs %.1f)"), *Legs[i].Name.ToString(), State.MeasuredX, State.CmdX);
			}
		}
		// A level-in that never brought the base within LevelInToleranceDeg
		// says so: the first lift waits for it.
		FString NotLevel;
		if (Phase == ERammsCurbPhase::Prepare && Level && !Level->bLevelled)
		{
			NotLevel = FString::Printf(TEXT("; not levelled in: the base is at %+.2f deg pitch, %+.2f deg roll (within %.1f needed)%s"),
				Level->FilteredAttitudeDeg.X, Level->FilteredAttitudeDeg.Y, RammsCurb::LevelInToleranceDeg,
				Level->bAttitudeActive ? TEXT(", the attitude loop running") : TEXT(", the legs not yet on the terrain"));
		}
		Finish(ERammsCurbPhase::Faulted,
			FString::Printf(TEXT("%s took longer than %.0f s%s%s%s"), *Status, Timeout,
				Lagging.IsEmpty() ? TEXT("") : TEXT("; legs off target:"), *Lagging, *NotLevel));
		return;
	}
	// The speed that stood over the time just measured, against how far the
	// chassis went: driving that gets nowhere is wheels slipping, or something
	// holding the robot, not a crossing.
	if (Drive && Telemetry
		&& Drive->Stall.Tick(DriveSpeed, Telemetry->Motion.GetSpeed(), Telemetry->Motion.IsFull(), DeltaTime, Tuning.StallSeconds))
	{
		Finish(ERammsCurbPhase::Faulted,
			FString::Printf(TEXT("stalled during %s: commanded %.1f cm/s, but the chassis moved at %.1f cm/s for %.1f s "
								 "(wheels slipping, or something holding it)"),
				*Status, DriveSpeed, Telemetry->Motion.GetSpeed(), Drive->Stall.GetSeconds()));
		return;
	}

	// The approach looks at the terrain again as it closes in, until the
	// first lift -- Steps[1] is the first Prepare, the last step before an
	// axle leaves the ground -- and for as long as the far side has not been
	// seen deep enough (which holds the approach short of its end, the commit
	// point). Stopped there with it still unseen, the crossing refuses.
	if (bTerrainDetected && (StepIndex <= 1 || bFarDepthPending))
	{
		LookAgain(false);
		if (!IsNegotiating())
		{
			return;
		}
	}
	// At the commit point with the far side still unconfirmed: the braking
	// cap brings the command to 0 there, and the wheels stay driven by it
	// until the chassis, a lag behind its command, has come to rest (one
	// that stopped a little short creeps on, as the cap allows); then it
	// looks once more and refuses if it still has not seen enough. Refusing
	// on the command alone would hard-brake a chassis still rolling.
	const bool			   bHeldAtCommit = bFarDepthPending && FarDepthStopDistance() <= FarDepthStopCm;
	const float			   ChassisSpeed = Telemetry ? Telemetry->Motion.GetSpeed() : 0.0f;
	const RammsCurb::ERest Rest = Drive
		? Drive->CommitRest.Tick(bHeldAtCommit, ChassisSpeed, Telemetry && Telemetry->Motion.IsFull(), DeltaTime,
			  FarDepthStoppedCmS, FarDepthRestSeconds)
		: RammsCurb::ERest::Moving;
	if (Rest != RammsCurb::ERest::Moving)
	{
		LookAgain(true);
		if (!IsNegotiating())
		{
			return;
		}
		if (bFarDepthPending)
		{
			Finish(ERammsCurbPhase::Faulted,
				Rest == RammsCurb::ERest::Rested
					? FString::Printf(TEXT("stopped short of the edge: %s"), *FarDepthWhy)
					: FString::Printf(TEXT("held short of the edge for %.1f s without coming to rest (%.1f cm/s): %s"),
						  Drive->CommitRest.GetSeconds(), ChassisSpeed, *FarDepthWhy));
			return;
		}
	}

	// Every leg over the terrain under its own wheel, the base turned by the
	// attitude loop; then whether the level-in is done.
	UpdateWheelSurfaces(DeltaTime);
	UpdateAttitude(DeltaTime);
	UpdateCorridor(DeltaTime);
	CommandLegs(RampStep);
	UpdateLevelIn();
	// The next tick is taken to be as long as this one.
	PredictLegRates(DeltaTime);
	DriveSpeed = ComputeDriveSpeed(RampStep);
	CommandWheels(DriveSpeed, Tuning.HeadingGain * HeadingErrorRad);
	RecordRunTick(DeltaTime, LegError);

	if (StepComplete())
	{
		if (StepIndex + 1 < Steps.Num())
		{
			EnterStep(StepIndex + 1);
		}
		else
		{
			Finish(ERammsCurbPhase::Done,
				FString::Printf(TEXT("%s %.1f cm in %.1f s"), ActiveProfile.StepHeightCm > 0.0f ? TEXT("climbed") : TEXT("descended"),
					FMath::Abs(ActiveProfile.StepHeightCm), Elapsed));
		}
	}
}

// --- the level base --------------------------------------------------------------

void URammsCurbNegotiationController::UpdateWheelSurfaces(float DeltaTime)
{
	if (LegState.Num() != Legs.Num())
	{
		return;
	}
	const FVector2D Bias = Level ? Level->GetBias() : FVector2D::ZeroVector;
	// Each wheel where it will be a servo lag from now, so the legs follow the
	// terrain without falling behind it (none while planning: nothing moves).
	const float		Ahead = DeltaTime > 0.0f ? DriveSpeed * Tuning.ServoLagSeconds : 0.0f;
	const FVector2D Forward(ChassisForward.X, ChassisForward.Y);

	TArray<RammsCurb::FTerrainLeg, TInlineAllocator<8>> Wheels;
	TArray<double, TInlineAllocator<8>>					Offsets;
	Wheels.SetNum(Legs.Num());
	Offsets.SetNum(Legs.Num());
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FRammsCurbLeg&	Leg = Legs[i];
		const FLegState&		State = LegState[i];
		RammsCurb::FTerrainLeg& W = Wheels[i];
		const FAxleGoal			Goal = LegGoal(i);
		W.XY = FVector2D(State.WheelWorld.X, State.WheelWorld.Y) + Forward * Ahead;
		// A lifted wheel counts the surface under it; a planted one the one
		// its step says it stands on (near until its axle crosses, far after).
		W.bFar = Goal.Goal == ELegGoal::Swing ? ActiveTerrain.DistancePastEdge(W.XY) >= 0.0 : Goal.bFar;
		W.RadiusCm = Leg.WheelRadiusCm;
		W.ContactWidthCm = RammsCurb::ContactWidth(Leg.TyreWidthCm, Leg.ContactWidthCm);
		W.RollDirection = RollDirectionOf(Leg, ChassisForward);
	}
	TerrainReference = RammsCurb::TerrainOffsets(ActiveTerrain, Wheels, GroundZ, PlanStepHeight, Bias, Offsets);
	// The chassis follows the reference no faster than it may rise or sink; a
	// plan (no time) leaves it where BuildPlan put it.
	if (DeltaTime > 0.0f)
	{
		TerrainReferenceCmd = StepToward(static_cast<float>(TerrainReferenceCmd), static_cast<float>(TerrainReference),
			Tuning.BaseHeightSpeed * FMath::Min(DeltaTime, MaxRampStep));
	}
	MeasuredBaseHeight = static_cast<float>(MeasuredOriginZ - GroundZ - TerrainReferenceCmd);

	TArray<float, TInlineAllocator<8>> NearResiduals, FarResiduals;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FRammsCurbLeg&   Leg = Legs[i];
		FLegState&			   State = LegState[i];
		const FAxleGoal		   Goal = LegGoal(i);
		const bool			   bFar = Wheels[i].bFar;
		const FRammsCurbPlane& Plane = bFar ? ActiveTerrain.Far : ActiveTerrain.Near;
		const float			   PlaneBias = static_cast<float>(bFar ? Bias.Y : Bias.X);
		const double		   Rho = RammsCurb::WheelCentreHeight(Leg.WheelRadiusCm, Wheels[i].ContactWidthCm, Plane.Gradient,
					  Wheels[i].RollDirection);
		State.bOnFar = bFar;
		if (!State.bHasWorld)
		{
			continue;
		}
		// Its bottom over the surface it follows, where it is now.
		const double Raw = State.WheelWorld.Z - Rho - Plane.HeightAt(FVector2D(State.WheelWorld.X, State.WheelWorld.Y));
		State.WheelBottom = static_cast<float>(Raw - PlaneBias);
		switch (Goal.Goal)
		{
			case ELegGoal::Swing:
			{
				// Carried clear of the terrain on its way (SwingCentreZ), over
				// the world: against the commanded reference, which moves.
				const double LevelZ = Goal.Level + Leg.WheelRadiusCm + Tuning.SwingClearance;
				const double Clear = SwingCentreZ(i, Wheels[i].XY, ToLandCm(i)) - GroundZ;
				State.TerrainOffset = static_cast<float>(Clear - TerrainReferenceCmd - LevelZ);
				break;
			}
			default:
				State.TerrainOffset = static_cast<float>(Offsets[i] + TerrainReference - TerrainReferenceCmd);
				// A planted wheel says where its surface is.
				if (FMath::Abs(Raw) <= ContactToleranceCm)
				{
					(Goal.bFar ? FarResiduals : NearResiduals).Add(static_cast<float>(Raw));
				}
				break;
		}
	}
	if (Level && DeltaTime > 0.0f)
	{
		Level->NearBias.Tick(NearResiduals, DeltaTime);
		Level->FarBias.Tick(FarResiduals, DeltaTime);
	}
}

double URammsCurbNegotiationController::ClearBottomZ(int32 LegIndex) const
{
	const FLegState& State = LegState[LegIndex];
	const FVector2D	 P(State.WheelWorld.X, State.WheelWorld.Y);
	const FVector2D	 Bias = Level ? Level->GetBias() : FVector2D::ZeroVector;
	const double	 Half = 0.5 * Tuning.SwingClearance;
	if (ActiveProfile.StepHeightCm > 0.0f)
	{
		// The curb top where this wheel crosses its edge.
		const FVector2D Lip = FVector2D(ActiveTerrain.EdgePoint.X, ActiveTerrain.EdgePoint.Y)
			+ ActiveTerrain.EdgeDirection * ActiveTerrain.AlongEdge(P);
		return ActiveTerrain.Far.HeightAt(Lip) + Bias.Y + Half;
	}
	return ActiveTerrain.Near.HeightAt(P) + Bias.X + Half;
}

void URammsCurbNegotiationController::UpdateAttitude(float DeltaTime)
{
	if (!Level || LegState.Num() != Legs.Num())
	{
		return;
	}
	// The loop runs from the level-in on, once the support legs are where the
	// terrain wants them (UpdateLevelIn): before, the base holds the stance
	// the robot drove up in, or is on its way from it. The integral waits for
	// three wheels carrying the robot and none landing.
	bool bIntegrate = false;
	if (Steps.IsValidIndex(StepIndex))
	{
		const FStep& S = Steps[StepIndex];
		int32		 Planted = 0;
		bool		 bLoading = false;
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			if (!IsLegPlanted(i))
			{
				continue;
			}
			if (S.Phase == ERammsCurbPhase::Plant && Legs[i].Axle == S.Axle)
			{
				bLoading = true;
			}
			else
			{
				++Planted;
			}
		}
		bIntegrate = Planted >= RammsCurb::MinPlantedForIntegral && !bLoading;
	}
	// The per-axle gait holds the base level: a planned attitude of 0.
	const FVector2D Correction = Level->TickAttitude(FVector2D(MeasuredPitchDeg, MeasuredRollDeg), FVector2D::ZeroVector, DeltaTime,
		bIntegrate, Tuning);

	// Each support leg's share: a pure rotation about their middle.
	TArray<FVector2D, TInlineAllocator<8>> XY;
	TArray<bool, TInlineAllocator<8>>	   Support;
	TArray<float, TInlineAllocator<8>>	   Dz;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		XY.Add(FVector2D(LegState[i].MeasuredX, Legs[i].Wheel.Position.Y));
		Support.Add(LegGoal(i).Goal == ELegGoal::Support);
	}
	Dz.Init(0.0f, Legs.Num());
	RammsCurb::AttitudeOffsets(XY, Support, Correction, Dz);
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		LegState[i].AttitudeDz = Dz[i];
	}
}

void URammsCurbNegotiationController::UpdateCorridor(float DeltaTime)
{
	// Off the plan's height only for a step that follows its corridor: along
	// the path planned for it, a servo lag ahead (as the legs look ahead), and
	// never outside the corridor where the robot is now -- on the terrain
	// under the wheels as it is now, the support legs as the attitude loop
	// turns them, each 5-bar where it is sent, the belly over the ground under
	// it a servo lag from now. Any other step brings it back to 0, and does not
	// end before it has (StepComplete).
	float Target = 0.0f;
	if (Steps.IsValidIndex(StepIndex) && Steps[StepIndex].bFollowCorridor && LegState.Num() == Legs.Num())
	{
		const FStep&	S = Steps[StepIndex];
		const FVector2D H(Heading.X, Heading.Y);
		const float		Ahead = DriveSpeed * Tuning.ServoLagSeconds;
		if (S.Corridor.IsValid())
		{
			const float Travel = static_cast<float>(FVector2D::DotProduct(ChassisOriginXY - PlanOriginXY, H)) + Ahead;
			Target = S.Corridor.At(Travel) - BaseHeightCmd - static_cast<float>(TerrainReferenceCmd);
		}
		TArray<FVector2D, TInlineAllocator<8>> Offset;
		TArray<float>						   LegX;
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			const FLegState& State = LegState[i];
			const FAxleGoal& Goal = S.Axles[static_cast<int32>(Legs[i].Axle)];
			const float		 Here = Goal.Goal == ELegGoal::Hold
					 ? NAN
					 : State.TerrainOffset + (Goal.Goal == ELegGoal::Support ? State.AttitudeDz : 0.0f);
			Offset.Add(FVector2D(Here, Here));
			LegX.Add(State.FiveBar ? State.CmdX : 0.0f);
		}
		const FVector2D Forward(ChassisForward.X, ChassisForward.Y);
		const float		Floor = static_cast<float>(BellyTopAt(ChassisOriginXY + Forward * Ahead) - GroundZ - TerrainReferenceCmd)
			+ Tuning.ChassisClearance - Tuning.ChassisUnderside;
		const FVector2D C = CorridorAt(S.Axles, Offset, Floor, LegX);
		if (C.X <= C.Y)
		{
			Target = FMath::Clamp(Target, static_cast<float>(C.X) - BaseHeightCmd, static_cast<float>(C.Y) - BaseHeightCmd);
		}
		else if (!S.Corridor.IsValid())
		{
			Target = static_cast<float>(0.5 * (C.X + C.Y)) - BaseHeightCmd;
		}
	}
	CorridorShiftCmd = StepToward(CorridorShiftCmd, Target, Tuning.BaseHeightSpeed * FMath::Min(DeltaTime, MaxRampStep));
}

void URammsCurbNegotiationController::UpdateLevelIn()
{
	if (!Level || Level->bLevelled || LegState.Num() != Legs.Num())
	{
		return;
	}
	// Under way once the first Prepare has begun; the support legs' commands
	// where the terrain wants them; then the attitude loop trims the base
	// level (RammsCurb::FLevelState::TickLevelIn).
	bool bCaughtUp = true;
	for (int32 i = 0; i < Legs.Num() && bCaughtUp; ++i)
	{
		bCaughtUp = LegGoal(i).Goal != ELegGoal::Support || FMath::Abs(LegState[i].CmdZ - LegState[i].GoalZ) <= CaughtUpCm;
	}
	const bool		bWasActive = Level->bAttitudeActive;
	const bool		bDone = Level->TickLevelIn(StepIndex >= 1, bCaughtUp);
	const FVector2D Attitude = Level->FilteredAttitudeDeg;
	if (Level->bAttitudeActive && !bWasActive)
	{
		const FString Line = FString::Printf(TEXT("t=%5.2fs levelling in: legs on the terrain, pitch %+.2f, roll %+.2f deg; the "
												  "attitude loop takes the rest"),
			Elapsed, Attitude.X, Attitude.Y);
		StepLog.Add(Line);
		UE_LOG(LogRammsCurb, Log, TEXT("[Curb] %s"), *Line);
	}
	if (bDone)
	{
		const FString Line = FString::Printf(TEXT("t=%5.2fs level: pitch %+.2f, roll %+.2f deg, the front wheels %.1f cm from the edge"),
			Elapsed, Attitude.X, Attitude.Y, -AxleProgress(ERammsCurbAxle::Front));
		StepLog.Add(Line);
		UE_LOG(LogRammsCurb, Log, TEXT("[Curb] %s"), *Line);
	}
}

float URammsCurbNegotiationController::LevelInSeconds() const
{
	if (!Level || Level->bLevelled)
	{
		return 0.0f;
	}
	if (StepIndex < 1)
	{
		return PlannedLevelInSeconds;
	}
	// Under way: the support legs' commands catching up with where the
	// terrain wants them, and their servos' tail.
	float Seconds = Tuning.ServoLagSeconds;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FLegState& State = LegState[i];
		if (LegGoal(i).Goal == ELegGoal::Support)
		{
			Seconds = FMath::Max(Seconds, RammsCurb::LegMoveSeconds(State.GoalZ - State.CmdZ, Tuning.LegSpeed, State.MeasuredZ - State.CmdZ, Tuning.ServoLagSeconds, 0.5f * Tuning.SettleTolerance));
		}
	}
	return Seconds;
}

float URammsCurbNegotiationController::LateLevelCap() const
{
	if (!Level || Level->bLevelled || Steps.Num() < 2 || !Steps.IsValidIndex(StepIndex))
	{
		return TNumericLimits<float>::Max();
	}
	// The lead wheel's distance to its edge along the heading: the front
	// axle's progress is reckoned from the near end of the edge.
	const float Distance = -AxleProgress(ERammsCurbAxle::Front);
	const float Gate = StepIndex == 0 ? FMath::Max(0.0f, Steps[0].ApproachTo - AxleProgress(ERammsCurbAxle::Front)) : 0.0f;
	// The chassis runs a lag behind its command: braked on the raw distance
	// it would come to rest past the line, by that lag times its speed.
	return RammsCurb::LateLevelSpeed(Distance, ActiveTerrain.ApproachAngleDeg, Gate, LevelInSeconds(), Tuning.Deceleration,
		MeasuredWheelSpeed(), RammsCurb::DriveLagSeconds);
}

void URammsCurbNegotiationController::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	TrackChassisMotion();
	if (bHandBackPending)
	{
		bHandBackPending = false;
		if (bDriveModeActive && !IsNegotiating())
		{
			FName Next = !PreviousModeId.IsNone() ? PreviousModeId : FallbackModeId();
			if (Next == GetDriveModeId())
			{
				Next = FallbackModeId();
			}
			if (!Next.IsNone())
			{
				UE_LOG(LogRammsCurb, Log, TEXT("[Curb] '%s': selected with no crossing to run; back to '%s'"),
					*GetNameSafe(GetOwner()), *Next.ToString());
				SetActiveModeThroughSelector(Next);
			}
		}
	}
	if (!IsNegotiating() || !bDriveModeActive || !Steps.IsValidIndex(StepIndex))
	{
		LastSimTime = -1.0;
		return;
	}

	// Time the gait against the physics, not the game. A simulator that cannot
	// keep up falls behind the game clock -- the lift-drive's roller model
	// steps at about a quarter of real time -- and every leg ramp, speed and
	// timeout here would otherwise run that many times too fast for it.
	float Step = DeltaTime;
	if (const URammsRobotBaseComponent* Base = EnsureBase())
	{
		const double Now = Base->GetSimulationTime();
		if (Now >= 0.0 && LastSimTime >= 0.0 && Now < LastSimTime)
		{
			// The physics was reset: the robot is back at its start pose and
			// the plan -- edge, ground, every leg target -- no longer applies.
			// No one asked for the stop, so it is a fault rather than an
			// Abort; but nothing is left standing across a step to hold, so
			// the mode from before takes the robot back too.
			Finish(ERammsCurbPhase::Faulted, TEXT("the simulation was reset"));
			LastSimTime = -1.0;
			bHandBackPending = true;
			return;
		}
		if (Now >= 0.0)
		{
			Step = LastSimTime >= 0.0 ? static_cast<float>(Now - LastSimTime) : 0.0f;
			LastSimTime = Now;
		}
	}
	if (Step > 0.0f)
	{
		StepPlan(Step);
	}
}

// --- telemetry -------------------------------------------------------------------

bool URammsCurbNegotiationController::LocateChassis(FVector2D& OutCentre, FVector2D& OutForward) const
{
	const URammsRobotBaseComponent* Base = EnsureBase();
	if (!Base)
	{
		return false;
	}
	FVector Front = FVector::ZeroVector, Rear = FVector::ZeroVector;
	int32	NFront = 0, NRear = 0;
	for (const FRammsCurbLeg& Leg : Legs)
	{
		if (Leg.Kind != ERammsCurbLegKind::FourBar || Leg.Axle == ERammsCurbAxle::Center || Leg.FourBar.CrankMotor.IsNone())
		{
			continue;
		}
		FTransform Pivot;
		if (!Base->GetMotorTransform(Leg.FourBar.CrankMotor, Pivot))
		{
			return false;
		}
		const bool bFront = Leg.Axle == ERammsCurbAxle::Front;
		(bFront ? Front : Rear) += Pivot.GetLocation();
		++(bFront ? NFront : NRear);
	}
	if (NFront == 0 || NRear == 0)
	{
		return false;
	}
	Front /= NFront;
	Rear /= NRear;
	const FVector Along = Horizontal(Front - Rear);
	if (Along.IsNearlyZero())
	{
		return false;
	}
	const FVector Forward = Along.GetSafeNormal();
	OutCentre = FVector2D(0.5 * (Front.X + Rear.X), 0.5 * (Front.Y + Rear.Y));
	OutForward = FVector2D(Forward.X, Forward.Y);
	return true;
}

void URammsCurbNegotiationController::TrackChassisMotion()
{
	// Measured on every tick, crossing or not, so that a crossing begun on the
	// move has a full window behind it from its first tick.
	const URammsRobotBaseComponent* Base = EnsureBase();
	FVector2D						Centre, Forward;
	if (!Telemetry || !Base || !LocateChassis(Centre, Forward))
	{
		return;
	}
	double Now = Base->GetSimulationTime();
	if (Now < 0.0)
	{
		const UWorld* World = GetWorld();
		if (!World)
		{
			return;
		}
		Now = World->GetTimeSeconds();
	}
	Telemetry->Motion.Add(Now, Centre, Forward);
}

float URammsCurbNegotiationController::AdvanceLegLag(float DeltaTime)
{
	// First order: each leg closes on its command with the servos' time
	// constant. A leg that lags more than that -- stalled, overloaded, pressed
	// into the ground -- shows as error.
	const float Tau = Tuning.ServoLagSeconds;
	float		Worst = 0.0f;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		FLegState&		State = LegState[i];
		const FVector2D Before(State.LagX, State.LagZ);
		State.LagZ = RammsCurb::LagStep(State.LagZ, State.CmdZ, DeltaTime, Tau);
		float Error = FMath::Abs(State.MeasuredZ - State.LagZ);
		if (State.FiveBar)
		{
			State.LagX = RammsCurb::LagStep(State.LagX, State.CmdX, DeltaTime, Tau);
			Error = FMath::Sqrt(FMath::Square(Error) + FMath::Square(State.MeasuredX - State.LagX));
		}
		else if (Legs[i].Kind == ERammsCurbLegKind::FourBar)
		{
			// A 4-bar carries its wheel fore/aft with its height; the error is
			// still its height's alone.
			State.LagX = FourBarX(i, State.LagZ, State.LagX);
		}
		State.LagRate = DeltaTime > 0.0f ? (FVector2D(State.LagX, State.LagZ) - Before) / DeltaTime : FVector2D::ZeroVector;
		Worst = FMath::Max(Worst, Error);
	}
	return Worst;
}

bool URammsCurbNegotiationController::MeasureSwingClearance(float& OutCm) const
{
	// The per-axle gait carries an axle across, clear, in Shift and Advance:
	// it has finished lifting and does not yet go down.
	if ((Phase != ERammsCurbPhase::Shift && Phase != ERammsCurbPhase::Advance) || !Steps.IsValidIndex(StepIndex)
		|| LegState.Num() != Legs.Num())
	{
		return false;
	}
	const FStep& S = Steps[StepIndex];
	if (S.Axles[static_cast<int32>(S.Axle)].Goal != ELegGoal::Swing)
	{
		return false;
	}
	bool bAny = false;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FLegState& State = LegState[i];
		if (Legs[i].Axle != S.Axle || !State.bHasWorld)
		{
			continue;
		}
		// The wheel's rim against the terrain under it, along its rolling
		// direction: at offset U from the centre the rim is sqrt(R^2 - U^2)
		// below it, so a lip a radius ahead is cleared by the centre alone.
		const double  R = Legs[i].WheelRadiusCm;
		const FVector C = State.WheelWorld;
		double		  Need = -TNumericLimits<double>::Max();
		for (int32 k = 0; k < SwingClearanceSamples; ++k)
		{
			const double U = -R + 2.0 * R * k / (SwingClearanceSamples - 1);
			double		 Z = 0.0;
			if (TraceSurface(C + Heading * U, C.Z + 2.0 * R, C.Z - R - 100.0, Z))
			{
				Need = FMath::Max(Need, Z + FMath::Sqrt(FMath::Max(0.0, R * R - U * U)));
			}
		}
		if (Need == -TNumericLimits<double>::Max())
		{
			continue;
		}
		const float Gap = static_cast<float>(C.Z - Need);
		OutCm = bAny ? FMath::Min(OutCm, Gap) : Gap;
		bAny = true;
	}
	return bAny;
}

bool URammsCurbNegotiationController::MeasureStabilityMargin(float& OutCm) const
{
	// The model the per-axle gait plans with: along the chassis only, the
	// robot stands on the two axles not lifted, and the margin is how far the
	// centre of mass is inside the span between them. Each axle is placed at
	// its wheels' mean measured fore/aft position. The approach holds the
	// stance the robot drove up in, and with it any axle that stance carries
	// up; every other step says which axle it has up.
	if (!Steps.IsValidIndex(StepIndex) || LegState.Num() != Legs.Num())
	{
		return false;
	}
	const FStep& S = Steps[StepIndex];
	int32		 Lifted = S.Phase == ERammsCurbPhase::Approach ? ApproachLiftedAxle : INDEX_NONE;
	for (int32 a = 0; a < 3; ++a)
	{
		if (S.Axles[a].Goal == ELegGoal::Swing)
		{
			if (Lifted != INDEX_NONE)
			{
				return false;
			}
			Lifted = a;
		}
	}
	if (Lifted == INDEX_NONE)
	{
		return false;
	}
	float SumX[3] = { 0.0f, 0.0f, 0.0f };
	int32 Count[3] = { 0, 0, 0 };
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const int32 a = static_cast<int32>(Legs[i].Axle);
		SumX[a] += LegState[i].MeasuredX;
		++Count[a];
	}
	float Lo = TNumericLimits<float>::Max(), Hi = -TNumericLimits<float>::Max();
	for (int32 a = 0; a < 3; ++a)
	{
		if (a == Lifted)
		{
			continue;
		}
		if (Count[a] == 0)
		{
			return false;
		}
		const float X = SumX[a] / Count[a];
		Lo = FMath::Min(Lo, X);
		Hi = FMath::Max(Hi, X);
	}
	OutCm = FMath::Min(Tuning.CenterOfMassX - Lo, Hi - Tuning.CenterOfMassX);
	return true;
}

void URammsCurbNegotiationController::RecordRunTick(float DeltaTime, float LegErrorCm)
{
	if (!Telemetry)
	{
		return;
	}
	RammsCurb::FRunTick T;
	T.DeltaSeconds = DeltaTime;
	T.Elapsed = Elapsed;
	T.CommandedSpeed = DriveSpeed;
	T.MeasuredSpeed = Telemetry->Motion.GetSpeed();
	T.bMeasuredSpeedValid = Telemetry->Motion.IsFull();
	T.PitchDeg = MeasuredPitchDeg;
	T.RollDeg = MeasuredRollDeg;
	// The gait holds the chassis level once the level-in has brought it there
	// from the stance the robot drove up in, which the approach holds whatever
	// its attitude. Until then the attitude is that stance's, or on its way
	// from it, and no error.
	T.bAttitudePlanned = Level && Level->bLevelled;
	for (const FLegState& State : LegState)
	{
		if (FMath::Abs(State.CmdZ - State.InitialZ) > LegMovedCm
			|| (State.FiveBar && FMath::Abs(State.CmdX - State.InitialX) > LegMovedCm))
		{
			T.bLegsMoved = true;
			break;
		}
	}
	T.LegErrorCm = LegErrorCm;
	float Cm = 0.0f;
	if (MeasureSwingClearance(Cm))
	{
		T.SwingClearanceCm = Cm;
	}
	if (MeasureStabilityMargin(Cm))
	{
		T.MeasuredMarginCm = Cm;
	}
	if (Telemetry->Motion.HasSample())
	{
		const FVector2D F0 = Telemetry->StartForward;
		const FVector2D F = Telemetry->Motion.GetForward();
		const FVector2D D = Telemetry->Motion.GetPosition() - Telemetry->StartPosition;
		// Towards +Y of the starting heading: its right, in Unreal's frame.
		T.LateralDriftCm = static_cast<float>(D.Y * F0.X - D.X * F0.Y);
		T.YawChangeDeg = FMath::FindDeltaAngleDegrees(static_cast<float>(FMath::RadiansToDegrees(FMath::Atan2(F0.Y, F0.X))),
			static_cast<float>(FMath::RadiansToDegrees(FMath::Atan2(F.Y, F.X))));
	}
	Telemetry->Stats.Tick(T);
}

// --- detection -------------------------------------------------------------------

bool URammsCurbNegotiationController::TraceSurface(const FVector& At, double AboveZ, double BelowZ, double& OutZ) const
{
	const AActor* Owner = GetOwner();
	const UWorld* World = GetWorld();
	if (!Owner || !World)
	{
		return false;
	}
	FCollisionQueryParams Params(SCENE_QUERY_STAT(RammsCurbSurface), /*bTraceComplex=*/false, Owner);
	TArray<AActor*>		  Attached;
	Owner->GetAttachedActors(Attached, true, true);
	Params.AddIgnoredActors(Attached);
	FHitResult Hit;
	if (!World->LineTraceSingleByChannel(Hit, FVector(At.X, At.Y, AboveZ), FVector(At.X, At.Y, BelowZ), ECC_Visibility,
			Params))
	{
		return false;
	}
	OutZ = Hit.ImpactPoint.Z;
	return true;
}

bool URammsCurbNegotiationController::DetectStepAhead(FRammsCurbProfile& OutProfile, FString& OutReason) const
{
	FRammsCurbTerrain			Terrain;
	RammsCurb::FTerrainFitInput In;
	const bool					bFound = DetectTerrain(/*bBeforeStart=*/true, Terrain, OutReason, &In);
	DetectionText = OutReason;
	if (!bFound)
	{
		return false;
	}
	RammsCurb::FillLegacyProfile(Terrain, In.Wheels, In.Origin, In.Heading, OutProfile);
	return true;
}

bool URammsCurbNegotiationController::DetectTerrainAhead(FRammsCurbTerrain& OutTerrain, FString& OutReason) const
{
	const bool bFound = DetectTerrain(/*bBeforeStart=*/true, OutTerrain, OutReason, nullptr);
	DetectionText = OutReason;
	return bFound;
}

void URammsCurbNegotiationController::GatherTerrainWheels(TArray<RammsCurb::FTerrainWheel>& Out) const
{
	Out.Reset();
	for (int32 i = 0; i < Legs.Num() && i < LegState.Num(); ++i)
	{
		if (!LegState[i].bHasWorld)
		{
			continue;
		}
		const FRammsCurbLeg&	  Leg = Legs[i];
		RammsCurb::FTerrainWheel& W = Out.AddDefaulted_GetRef();
		W.Name = Leg.Name;
		W.Centre = LegState[i].WheelWorld;
		W.RadiusCm = Leg.WheelRadiusCm;
		W.TyreWidthCm = Leg.TyreWidthCm;
		W.ContactWidthCm = RammsCurb::ContactWidth(Leg.TyreWidthCm, Leg.ContactWidthCm);
		W.bFront = Leg.Axle == ERammsCurbAxle::Front;
	}
}

void URammsCurbNegotiationController::GatherFarDepthNeed(RammsCurb::FFarDepthNeed& Out) const
{
	// The corner wheels bound the robot fore and aft: the front ones as far
	// forward as their 4-bars carry them, the rear ones as far aft, at any
	// height (the crossing moves them through all of it).
	double FrontX = -TNumericLimits<double>::Max(), RearX = TNumericLimits<double>::Max();
	double YLo = TNumericLimits<double>::Max(), YHi = -TNumericLimits<double>::Max();
	float  RearRadius = 0.0f;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FRammsCurbLeg& Leg = Legs[i];
		if (Leg.Axle == ERammsCurbAxle::Center)
		{
			continue;
		}
		YLo = FMath::Min(YLo, static_cast<double>(Leg.Wheel.Position.Y));
		YHi = FMath::Max(YHi, static_cast<double>(Leg.Wheel.Position.Y));
		double XLo = TNumericLimits<double>::Max(), XHi = -TNumericLimits<double>::Max();
		if (Leg.Kind == ERammsCurbLegKind::FourBar)
		{
			const int32 Samples = 64;
			for (int32 s = 0; s <= Samples; ++s)
			{
				bool			bValid = false;
				const float		Crank = FMath::Lerp(Leg.FourBar.CrankRange.X, Leg.FourBar.CrankRange.Y, float(s) / Samples);
				const FVector2D C = URamms4BarKinematics::ComputeWheelCenter(Leg.FourBar, Crank, bValid);
				if (bValid)
				{
					XLo = FMath::Min(XLo, C.X);
					XHi = FMath::Max(XHi, C.X);
				}
			}
		}
		if (XLo > XHi && LegState.IsValidIndex(i))
		{
			XLo = XHi = LegState[i].MeasuredX;
		}
		if (XLo > XHi)
		{
			continue;
		}
		if (Leg.Axle == ERammsCurbAxle::Front)
		{
			FrontX = FMath::Max(FrontX, XHi);
		}
		else
		{
			RearX = FMath::Min(RearX, XLo);
			RearRadius = FMath::Max(RearRadius, Leg.WheelRadiusCm);
		}
	}
	Out = RammsCurb::FFarDepthNeed();
	if (FrontX > RearX)
	{
		Out.WheelbaseCm = static_cast<float>(FrontX - RearX);
	}
	if (YHi > YLo)
	{
		Out.TrackCm = static_cast<float>(YHi - YLo);
	}
	Out.ExtraCm = RearRadius + Tuning.FinishClearance + RammsCurb::FarDepthMarginCm;
}

bool URammsCurbNegotiationController::DetectTerrain(bool bBeforeStart, FRammsCurbTerrain& OutTerrain, FString& OutReason,
	RammsCurb::FTerrainFitInput* OutInput) const
{
	OutTerrain = FRammsCurbTerrain();
	URammsRobotBaseComponent* Base = EnsureBase();
	const AActor*			  Owner = GetOwner();
	const UWorld*			  World = GetWorld();
	if (!Owner || !World || !Base)
	{
		OutReason = TEXT("no world or robot base to look from");
		return false;
	}

	// The chassis from its corner crank pivots -- their centroid and the
	// heading from the rear ones to the front -- and every wheel: all of them,
	// or no answer, since a wheel that cannot be located would leave the scan
	// short of the robot's footprint.
	RammsCurb::FTerrainFitInput In;
	In.SetTuning(Tuning);
	In.bCheckWheels = bBeforeStart;
	FVector	  FrontPivots = FVector::ZeroVector, RearPivots = FVector::ZeroVector;
	FVector2D PivotSum = FVector2D::ZeroVector;
	int32	  NF = 0, NR = 0;
	double	  Ground = TNumericLimits<double>::Max();
	for (const FRammsCurbLeg& Leg : Legs)
	{
		FTransform Xf;
		if (Leg.Kind == ERammsCurbLegKind::FourBar && Leg.Axle != ERammsCurbAxle::Center)
		{
			if (Leg.FourBar.CrankMotor.IsNone() || !Base->GetMotorTransform(Leg.FourBar.CrankMotor, Xf))
			{
				OutReason = FString::Printf(TEXT("the '%s' crank pivot cannot be located"), *Leg.Name.ToString());
				return false;
			}
			const FVector P = Xf.GetLocation();
			(Leg.Axle == ERammsCurbAxle::Front ? FrontPivots : RearPivots) += P;
			++(Leg.Axle == ERammsCurbAxle::Front ? NF : NR);
			PivotSum += FVector2D(P.X, P.Y);
		}
		if (!Base->GetMotorTransform(Leg.Wheel.MotorId, Xf))
		{
			OutReason = FString::Printf(TEXT("the '%s' wheel cannot be located"), *Leg.Name.ToString());
			return false;
		}
		RammsCurb::FTerrainWheel& W = In.Wheels.AddDefaulted_GetRef();
		W.Name = Leg.Name;
		W.Centre = Xf.GetLocation();
		W.RadiusCm = Leg.WheelRadiusCm;
		W.TyreWidthCm = Leg.TyreWidthCm;
		W.ContactWidthCm = RammsCurb::ContactWidth(Leg.TyreWidthCm, Leg.ContactWidthCm);
		W.bFront = Leg.Axle == ERammsCurbAxle::Front;
		Ground = FMath::Min(Ground, W.Centre.Z - Leg.WheelRadiusCm);
	}
	if (NF == 0 || NR == 0 || In.Wheels.Num() == 0)
	{
		OutReason = TEXT("the robot's wheels and pivots cannot be located yet");
		return false;
	}
	const FVector Along = Horizontal(FrontPivots / NF - RearPivots / NR);
	if (Along.IsNearlyZero())
	{
		OutReason = TEXT("the robot's heading cannot be told from its pivots");
		return false;
	}
	In.Origin = PivotSum / (NF + NR);
	In.Heading = FVector2D(Along.X, Along.Y).GetSafeNormal();
	if (bBeforeStart)
	{
		GatherFarDepthNeed(In.FarDepthNeed);
	}

	// Trace straight down past the robot itself, over a band about the lowest
	// wheel bottom wide enough for the steepest surfaces the detection accepts
	// across the whole scan, and the largest step.
	const double Reach = Tuning.MaxStepHeight
		+ (Tuning.DetectRange + DetectBehindCm) * FMath::Tan(FMath::DegreesToRadians(FMath::Min(Tuning.MaxSurfaceSlopeDeg, 45.0f)))
		+ DetectTraceMarginCm;
	FCollisionQueryParams Params(SCENE_QUERY_STAT(RammsCurbSurface), /*bTraceComplex=*/false, Owner);
	TArray<AActor*>		  Attached;
	Owner->GetAttachedActors(Attached, true, true);
	Params.AddIgnoredActors(Attached);
	auto SurfaceAt = [&](const FVector2D& P, double& OutZ) {
		FHitResult Hit;
		if (!World->LineTraceSingleByChannel(Hit, FVector(P.X, P.Y, Ground + Reach), FVector(P.X, P.Y, Ground - Reach),
				ECC_Visibility, Params))
		{
			return false;
		}
		OutZ = Hit.ImpactPoint.Z;
		return true;
	};
	RammsCurb::FTerrainFitReport Report;
	const double				 Start = FPlatformTime::Seconds();
	const bool					 bFound = RammsCurb::FitTerrain(In, SurfaceAt, OutTerrain, OutReason, &Report);
	UE_LOG(LogRammsCurb, Verbose, TEXT("[Curb] '%s' detection: %d traces in %.1f ms: %s"), *GetNameSafe(Owner), Report.Traces,
		1000.0 * (FPlatformTime::Seconds() - Start), *OutReason);
	if (OutInput)
	{
		*OutInput = MoveTemp(In);
	}
	return bFound;
}

void URammsCurbNegotiationController::LookAgain(bool bForce)
{
	FVector2D Centre, Forward;
	if (!LocateChassis(Centre, Forward))
	{
		return;
	}
	if (!bForce && bHasLastLook && FVector2D::Distance(Centre, LastLookCentre) < RammsCurb::RedetectEveryCm)
	{
		return;
	}
	LastLookCentre = Centre;
	bHasLastLook = true;

	auto Note = [this](const FString& What) {
		const FString Line = FString::Printf(TEXT("t=%5.2fs %s"), Elapsed, *What);
		StepLog.Add(Line);
		UE_LOG(LogRammsCurb, Log, TEXT("[Curb] %s"), *Line);
	};
	// With legs moving the robot no longer stands as it did at the start, so
	// the look does not check it does; and it measures the far depth without
	// judging it, which is the verdict below.
	FRammsCurbTerrain Seen;
	FString			  Why;
	if (!DetectTerrain(/*bBeforeStart=*/false, Seen, Why, nullptr))
	{
		if (Why != LastLookFailure)
		{
			Note(FString::Printf(TEXT("looked again and found nothing to use (%s); keeping the terrain it has"), *Why));
			LastLookFailure = Why;
		}
		return;
	}
	LastLookFailure.Reset();
	const RammsCurb::FTerrainChange Change = RammsCurb::CompareTerrain(ActiveTerrain, Seen);
	if (Change.MaxCm() > RammsCurb::RedetectRejectCm || Change.MaxDeg() > RammsCurb::RedetectRejectDeg)
	{
		Note(FString::Printf(TEXT("looked again and threw it away: it moved %s"), *Change.Describe()));
		return;
	}
	// Before the first lift the whole terrain is refined; after it -- the far
	// side still unconfirmed -- only how deep that is.
	const bool		  bBeforeLift = StepIndex <= 1;
	FRammsCurbTerrain Blended = RammsCurb::BlendTerrain(ActiveTerrain, Seen, RammsCurb::RedetectBlend, FVector2D(Heading.X, Heading.Y));
	if (bBeforeLift)
	{
		ActiveTerrain = Blended;
	}
	else
	{
		ActiveTerrain.FarDepthCm = Blended.FarDepthCm;
		ActiveTerrain.bFarDepthObserved = Blended.bFarDepthObserved;
	}

	if (bFarDepthPending)
	{
		FString Verdict;
		switch (RammsCurb::FarDepthVerdict(ActiveTerrain, FarDepthNeededCm, &Verdict))
		{
			case RammsCurb::EFarDepth::Enough:
				bFarDepthPending = false;
				FarDepthWhy.Reset();
				Note(FString::Printf(TEXT("far side confirmed: %.0f cm deep%s, the robot needs %.0f"), ActiveTerrain.FarDepthCm,
					ActiveTerrain.bFarDepthObserved ? TEXT("") : TEXT(" at least"), FarDepthNeededCm));
				break;
			case RammsCurb::EFarDepth::TooShallow:
				// Seen to be too shallow: the stop at the commit point stands,
				// and the crossing refuses there.
				if (FarDepthWhy != Verdict)
				{
					Note(FString::Printf(TEXT("far side: %s; stopping short of the edge"), *Verdict));
				}
				FarDepthWhy = Verdict;
				break;
			default:
				FarDepthWhy = Verdict;
				break;
		}
	}

	const RammsCurb::FTerrainChange Drift = RammsCurb::CompareTerrain(PlannedTerrain, ActiveTerrain);
	if (bBeforeLift && (Drift.MaxCm() > RammsCurb::ReplanCm || Drift.MaxDeg() > RammsCurb::ReplanDeg))
	{
		FString Failed;
		if (!Replan(Failed))
		{
			Finish(ERammsCurbPhase::Faulted, FString::Printf(TEXT("the terrain looked at again cannot be crossed: %s"), *Failed));
		}
	}
}

bool URammsCurbNegotiationController::Replan(FString& OutReason)
{
	FVector2D Centre, Forward;
	if (!LocateChassis(Centre, Forward))
	{
		OutReason = TEXT("the chassis cannot be located");
		return false;
	}
	TArray<RammsCurb::FTerrainWheel> Wheels;
	GatherTerrainWheels(Wheels);
	FRammsCurbProfile Profile = ActiveProfile;
	RammsCurb::FillLegacyProfile(ActiveTerrain, Wheels, Centre, FVector2D(Heading.X, Heading.Y), Profile);
	if (FMath::Abs(Profile.StepHeightCm) < Tuning.MinStepHeight || FMath::Abs(Profile.StepHeightCm) > Tuning.MaxStepHeight)
	{
		OutReason = FString::Printf(TEXT("the step is now %.1f cm"), Profile.StepHeightCm);
		return false;
	}

	// The old plan stays if the new one fails.
	const TArray<FStep>				OldSteps = Steps;
	const TArray<FString>			OldLines = PlanLines;
	const FVector					OldEdge = EdgePoint;
	const float						OldMinSpeed = PlannedMinSpeed;
	const float						OldMinMargin = PlannedMinMargin;
	const bool						bOldMargin = bPlannedMargin;
	const int32						OldLifted = ApproachLiftedAxle;
	const float						OldLevelIn = PlannedLevelInSeconds;
	const float						OldStepHeight = PlanStepHeight;
	const FVector2D					OldOrigin = PlanOriginXY;
	const float						OldCommit = CommitTo;
	const RammsCurb::FTerrainChange Drift = RammsCurb::CompareTerrain(PlannedTerrain, ActiveTerrain);
	const double					Start = FPlatformTime::Seconds();
	FString							Summary;
	if (!BuildSteps(Profile, Summary))
	{
		Steps = OldSteps;
		PlanLines = OldLines;
		EdgePoint = OldEdge;
		PlannedMinSpeed = OldMinSpeed;
		PlannedMinMargin = OldMinMargin;
		bPlannedMargin = bOldMargin;
		ApproachLiftedAxle = OldLifted;
		PlannedLevelInSeconds = OldLevelIn;
		PlanStepHeight = OldStepHeight;
		PlanOriginXY = OldOrigin;
		CommitTo = OldCommit;
		for (FLegState& State : LegState)
		{
			if (State.bHasWorld)
			{
				State.Progress = static_cast<float>(FVector::DotProduct(Horizontal(State.WheelWorld - EdgePoint), Heading));
			}
		}
		OutReason = Summary;
		return false;
	}
	const double Ms = 1000.0 * (FPlatformTime::Seconds() - Start);
	ActiveProfile = Profile;
	PlannedTerrain = ActiveTerrain;
	if (bFarDepthPending)
	{
		PlanLines.Add(FString::Printf(TEXT("far side: %s; the robot must see the rest before the front wheels are %.1f cm from the "
										   "edge (the commit point)"),
			*FarDepthWhy, -CommitTo));
	}
	if (Telemetry)
	{
		Telemetry->Stats.AddReplan(PlannedMinSpeed, bPlannedMargin ? TOptional<float>(PlannedMinMargin) : TOptional<float>());
	}
	const FString Line = FString::Printf(TEXT("t=%5.2fs re-planned in %.1f ms, the terrain having moved %s: %s"), Elapsed, Ms,
		*Drift.Describe(), *Summary);
	StepLog.Add(Line);
	UE_LOG(LogRammsCurb, Log, TEXT("[Curb] %s"), *Line);
	for (const FString& Plan : PlanLines)
	{
		UE_LOG(LogRammsCurb, Log, TEXT("[Curb]   %s"), *Plan);
	}
	return true;
}

float URammsCurbNegotiationController::FarDepthStopDistance() const
{
	// The commit point is where the per-axle gait's legs start to move for
	// the first axle, PrepareDistance short of its first window (CommitTo:
	// the approach's end before the level-in moved it earlier), and nothing
	// has yet been readied for the edge. Measured as the approach's end is,
	// by the trailing front wheel along the heading from the edge's near end.
	// Braking to a stop there leaves the servo and wheel-loop lag (a few cm
	// at cruise) well short of any window.
	if (Steps.Num() > 0 && Steps[0].Phase == ERammsCurbPhase::Approach)
	{
		return CommitTo - AxleProgress(ERammsCurbAxle::Front);
	}
	// A plan with no approach: each wheel's first window -- on a climb
	// FaceMargin short of the face at its tyre's front, on a descent
	// EdgeMargin short of the lip at its centre.
	const bool bClimb = ActiveProfile.StepHeightCm > 0.0f;
	float	   Least = TNumericLimits<float>::Max();
	for (int32 i = 0; i < Legs.Num() && i < LegState.Num(); ++i)
	{
		if (!LegState[i].bHasWorld)
		{
			continue;
		}
		const float Window = bClimb ? -Tuning.FaceMargin - Legs[i].WheelRadiusCm : -Tuning.EdgeMargin;
		Least = FMath::Min(Least, Window - LegState[i].Progress);
	}
	return Least;
}

// --- control surface -----------------------------------------------------------

void URammsCurbNegotiationController::DescribeControls(FRammsControlSurface& OutSurface) const
{
	// Offered whatever mode is active, which the drive-mode contract allows
	// for Ids no other mode uses. Starting a manoeuvre is what switches the
	// robot into this mode, and the mode selected by hand hands straight back,
	// so gating this on bDriveModeActive would leave the action unreachable.
	// Nothing moves until the mode is live: the claims and the gait wait.
	const FName Group(TEXT("Curb"));

	FRammsControlAxis Negotiate;
	Negotiate.Id = NegotiateControlId();
	Negotiate.Group = Group;
	Negotiate.DisplayName = NSLOCTEXT("Ramms", "CurbNegotiate", "Cross curb ahead");
	Negotiate.Kind = ERammsControlKind::Action;
	Negotiate.Units = ERammsControlUnits::None;
	Negotiate.bReadback = false;
	Negotiate.Order = 0;
	OutSurface.Add(Negotiate);

	FRammsControlAxis Stop = Negotiate;
	Stop.Id = AbortControlId();
	Stop.DisplayName = NSLOCTEXT("Ramms", "CurbAbort", "Stop curb crossing");
	Stop.Order = 1;
	OutSurface.Add(Stop);

	FRammsControlAxis State;
	State.Id = PhaseControlId();
	State.Group = Group;
	State.DisplayName = NSLOCTEXT("Ramms", "CurbPhase", "Curb crossing");
	State.Kind = ERammsControlKind::Enum;
	State.Units = ERammsControlUnits::None;
	State.bReadOnly = true;
	State.Order = 2;
	const UEnum* PhaseEnum = StaticEnum<ERammsCurbPhase>();
	for (int32 i = 0; i < PhaseEnum->NumEnums() - 1; ++i)
	{
		State.EnumLabels.Add(PhaseEnum->GetDisplayNameTextByIndex(i));
	}
	State.Range = FVector2D(0.0, FMath::Max(0, State.EnumLabels.Num() - 1));
	OutSurface.Add(State);
}

bool URammsCurbNegotiationController::TriggerControl(FName Id)
{
	if (Id == NegotiateControlId())
	{
		FString Reason;
		return NegotiateStepAhead(Reason);
	}
	if (Id == AbortControlId())
	{
		Abort();
		return true;
	}
	return false;
}

bool URammsCurbNegotiationController::ReadControl(FName Id, float& OutValue) const
{
	if (Id == PhaseControlId())
	{
		OutValue = static_cast<float>(Phase);
		return true;
	}
	return false;
}

void URammsCurbNegotiationController::GetClaimedMotorIds(TArray<FName>& OutIds) const
{
	if (!bDriveModeActive)
	{
		return;
	}
	for (const FRammsCurbLeg& Leg : Legs)
	{
		OutIds.AddUnique(Leg.Wheel.MotorId);
		if (Leg.Kind == ERammsCurbLegKind::FourBar)
		{
			OutIds.AddUnique(Leg.FourBar.CrankMotor);
		}
	}
}
