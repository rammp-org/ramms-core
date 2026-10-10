// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsCurbNegotiationController.h"

#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Ramms4BarKinematics.h"
#include "Ramms5BarLinkageController.h"
#include "RammsDriveModeSelector.h"
#include "RammsRobotBaseComponent.h"
#include "RammsRobotControlSurfaceComponent.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsCurb, Log, All);

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

	/** Braking-limited speed with Distance left to a hard stop. */
	float BrakingSpeed(float Distance, float Deceleration)
	{
		return Distance <= 0.0f ? 0.0f : FMath::Sqrt(2.0f * Deceleration * Distance);
	}

	FVector Horizontal(const FVector& V)
	{
		return FVector(V.X, V.Y, 0.0);
	}
} // namespace

URammsCurbNegotiationController::URammsCurbNegotiationController()
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

// --- set-up ----------------------------------------------------------------------

bool URammsCurbNegotiationController::ResolveLegs(FString& OutReason)
{
	AActor* Owner = GetOwner();
	if (!Owner || !EnsureBase())
	{
		OutReason = TEXT("no RammsRobotBaseComponent on this robot");
		return false;
	}

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

bool URammsCurbNegotiationController::Measure()
{
	URammsRobotBaseComponent* Base = EnsureBase();
	if (!Base || LegState.Num() != Legs.Num())
	{
		return false;
	}

	// Chassis attitude from the crank pivots.
	FVector	  FrontW = FVector::ZeroVector, RearW = FVector::ZeroVector;
	FVector	  PlusW = FVector::ZeroVector, MinusW = FVector::ZeroVector;
	FVector2D FrontL = FVector2D::ZeroVector, RearL = FVector2D::ZeroVector, AllL = FVector2D::ZeroVector;
	double	  AllZ = 0.0;
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
		AllL += Leg.FourBar.CrankPivot;
		++NAll;
	}
	if (NFront == 0 || NRear == 0)
	{
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
	MeasuredBaseHeight = static_cast<float>(OriginZ - GroundZ);

	if (!IsNegotiating())
	{
		Heading = Forward;
	}
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
			bool bValid = false;
			State.MeasuredZ = static_cast<float>(
				URamms4BarKinematics::ComputeWheelCenter(Leg.FourBar, Base->GetMotorValue(Leg.FourBar.CrankMotor), bValid).Y);
		}
		else if (State.FiveBar)
		{
			State.MeasuredZ = static_cast<float>(State.FiveBar->GetCurrentEndpoint().Y);
		}
		FTransform Wheel;
		State.bHasWorld = Base->GetMotorTransform(Leg.Wheel.MotorId, Wheel);
		if (State.bHasWorld)
		{
			const FVector W = Wheel.GetLocation();
			State.Progress = static_cast<float>(FVector::DotProduct(Horizontal(W - EdgePoint), Heading));
			State.WheelBottom = static_cast<float>(W.Z - Leg.WheelRadiusCm - GroundZ);
		}
	}
	return true;
}

// --- planning --------------------------------------------------------------------

float URammsCurbNegotiationController::GoalHeight(int32 LegIndex, const FAxleGoal& Goal, float BaseHeight) const
{
	const FRammsCurbLeg& Leg = Legs[LegIndex];
	switch (Goal.Goal)
	{
		case ELegGoal::Support:
			return Goal.Level + Leg.WheelRadiusCm - BaseHeight - Tuning.PlantPreload
				- (Leg.Drive == ERammsCurbWheelDrive::Main ? Tuning.MainWheelPreload : 0.0f);
		case ELegGoal::Hold:
			return LegState[LegIndex].InitialZ;
		default:
			return Goal.Level + Leg.WheelRadiusCm + Tuning.SwingClearance - BaseHeight;
	}
}

float URammsCurbNegotiationController::CommandedGoalHeight(int32 LegIndex, const FAxleGoal& Goal) const
{
	const float Z = GoalHeight(LegIndex, Goal, BaseHeightCmd);
	if (Goal.Goal != ELegGoal::Swing)
	{
		return Z;
	}
	// Clearance is over the ground, not the chassis. Two axles carrying the
	// robot let it pitch a degree or three, and at the far end of a 90 cm
	// wheelbase that is the whole clearance: a lifted wheel the chassis has
	// tipped towards the ground is lifted further -- as far as the leg reaches,
	// so the command never asks for more than the leg can be measured doing.
	const FLegState& State = LegState[LegIndex];
	const float		 Wanted = Z - State.ModelX * FMath::Sin(FilteredPitchRad);
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
	float X, FString* OutWhy) const
{
	const FRammsCurbLeg& Leg = Legs[LegIndex];
	const FLegState&	 State = LegState[LegIndex];
	if (Goal.Goal == ELegGoal::Hold)
	{
		return true; // where it already is
	}
	const float Z = GoalHeight(LegIndex, Goal, BaseHeight);
	const float Margin = Tuning.LegRangeMargin;
	if (Leg.Kind == ERammsCurbLegKind::FourBar)
	{
		if (Z < State.HeightRange.X + Margin || Z > State.HeightRange.Y - Margin)
		{
			if (OutWhy)
			{
				*OutWhy = FString::Printf(TEXT("leg '%s' would need its wheel at %.1f cm; it reaches %.1f..%.1f"),
					*Leg.Name.ToString(), Z, State.HeightRange.X, State.HeightRange.Y);
			}
			return false;
		}
		return true;
	}
	if (!State.FiveBar)
	{
		return false;
	}
	for (const float Dz : { 0.0f, -Margin, Margin })
	{
		bool bReachable = false;
		State.FiveBar->SolveTarget(FVector2D(X, Z + Dz), bReachable);
		if (!bReachable)
		{
			if (OutWhy)
			{
				*OutWhy = FString::Printf(TEXT("leg '%s' cannot put its wheel at (%.1f, %.1f) cm"),
					*Leg.Name.ToString(), X, Z + Dz);
			}
			return false;
		}
	}
	return true;
}

bool URammsCurbNegotiationController::IsReachable(const FAxleGoal Axles[3], float BaseHeight,
	const TArray<float>& LegX, float ChassisFloor, FString* OutWhy) const
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
		const float X = LegX.IsValidIndex(i) ? LegX[i] : 0.0f;
		if (!LegReachable(i, Axles[static_cast<int32>(Legs[i].Axle)], BaseHeight, X, OutWhy))
		{
			return false;
		}
	}
	return true;
}

bool URammsCurbNegotiationController::PlanLift(ERammsCurbAxle Axle, TArrayView<const FAxleGoal* const> Configs,
	float ChassisFloor, float PreferredHeight, float PreferredCenterX, float MinShift, float& OutHeight,
	TArray<float>& OutLegX, FString& OutReason) const
{
	// Centre-wheel offsets to try, best first: from the full stability margin
	// back towards the centre of mass. Lifting the centre axle itself has no
	// stability preference; try outward from the middle.
	TArray<float> Candidates;
	if (Axle == ERammsCurbAxle::Center)
	{
		for (int32 i = 0; i <= 40; ++i)
		{
			Candidates.Add(PreferredCenterX + 0.5f * ((i + 1) / 2) * ((i % 2) ? 1.0f : -1.0f));
		}
	}
	else
	{
		const float Toward = Tuning.CenterOfMassX;
		const int32 Count = FMath::Max(1, FMath::CeilToInt(FMath::Abs(PreferredCenterX - Toward) / 0.5f));
		for (int32 i = 0; i <= Count; ++i)
		{
			Candidates.Add(FMath::Lerp(PreferredCenterX, Toward, float(i) / Count));
		}
	}
	const float Required = Axle == ERammsCurbAxle::Center ? 0.0f : MinShift;
	const float Wanted = Axle == ERammsCurbAxle::Center ? 0.0f : FMath::Abs(PreferredCenterX - Tuning.CenterOfMassX);

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
				for (const FAxleGoal* Config : Configs)
				{
					if (!LegReachable(i, Config[A], H, 0.0f, nullptr))
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
				for (const FAxleGoal* Config : Configs)
				{
					bAll = bAll && LegReachable(i, Config[A], H, X, nullptr);
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
	for (const FAxleGoal* Config : Configs)
	{
		bAllReachable = bAllReachable && IsReachable(Config, H, LegX, ChassisFloor, &Why);
	}
	if (bAllReachable)
	{
		Why = FString::Printf(TEXT("the centre wheels cannot shift %.1f cm past the centre of mass"), Required);
	}
	OutReason = FString::Printf(TEXT("no chassis height lets the %s axle cross: %s"), AxleName(Axle), *Why);
	return false;
}

bool URammsCurbNegotiationController::BuildPlan(const FRammsCurbProfile& Profile, FString& OutReason)
{
	if (!ResolveLegs(OutReason))
	{
		return false;
	}
	const float StepHeight = Profile.StepHeightCm;
	if (Profile.EdgeSkewCm < 0.0f)
	{
		OutReason = TEXT("EdgeSkewCm cannot be negative");
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

	// Ground first: the lowest wheel bottom. Measure() reads heights relative
	// to it, so it has to be in place before the real measurement.
	GroundZ = 0.0;
	EdgePoint = FVector::ZeroVector;
	Phase = ERammsCurbPhase::Idle;
	if (!Measure())
	{
		OutReason = TEXT("the robot's wheels and pivots cannot be located yet (is the simulation running?)");
		return false;
	}
	double Lowest = TNumericLimits<double>::Max();
	for (const FLegState& State : LegState)
	{
		if (State.bHasWorld)
		{
			Lowest = FMath::Min(Lowest, static_cast<double>(State.WheelBottom));
		}
	}
	GroundZ = Lowest;
	Measure();

	// The gait starts with every wheel over one surface. A robot already
	// standing across a step -- a retry after a crossing stopped halfway --
	// would have its wheels on the higher side sent down to the lower one.
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		FTransform Wheel;
		if (!LegState[i].bHasWorld || !EnsureBase()->GetMotorTransform(Legs[i].Wheel.MotorId, Wheel))
		{
			continue;
		}
		const FVector At = Wheel.GetLocation();
		double		  Surface = 0.0;
		if (!TraceSurface(At, At.Z, GroundZ - 60.0, Surface))
		{
			OutReason = FString::Printf(TEXT("nothing under the '%s' wheel; drive back onto solid ground first"),
				*Legs[i].Name.ToString());
			return false;
		}
		if (FMath::Abs(Surface - GroundZ) > Tuning.MinStepHeight)
		{
			OutReason = FString::Printf(TEXT("the robot is standing across a step (the '%s' wheel is over ground "
											 "%+.1f cm from the rest); drive clear of it first"),
				*Legs[i].Name.ToString(), Surface - GroundZ);
			return false;
		}
	}

	// Where the legs are, and which carry the robot now.
	float	SupportSum = 0.0f;
	int32	SupportCount = 0;
	FVector FrontSum = FVector::ZeroVector;
	int32	FrontCount = 0;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		FLegState& State = LegState[i];
		State.InitialZ = State.MeasuredZ;
		State.CmdZ = State.MeasuredZ;
		if (State.FiveBar)
		{
			State.InitialX = static_cast<float>(State.FiveBar->GetCurrentEndpoint().X);
			State.CmdX = State.InitialX;
			State.ModelX = State.InitialX;
		}
		else
		{
			bool bValid = false;
			State.ModelX = static_cast<float>(
				URamms4BarKinematics::ComputeWheelCenter(Legs[i].FourBar,
					EnsureBase()->GetMotorValue(Legs[i].FourBar.CrankMotor), bValid)
					.X);
		}
		if (State.WheelBottom < 1.5f)
		{
			SupportSum += Legs[i].WheelRadiusCm - State.MeasuredZ;
			++SupportCount;
		}
		if (Legs[i].Axle == ERammsCurbAxle::Front && State.bHasWorld)
		{
			FTransform Wheel;
			EnsureBase()->GetMotorTransform(Legs[i].Wheel.MotorId, Wheel);
			FrontSum += Wheel.GetLocation();
			++FrontCount;
		}
	}
	if (SupportCount == 0 || FrontCount == 0)
	{
		OutReason = TEXT("cannot tell which wheels are on the ground");
		return false;
	}
	InitialBaseHeight = SupportSum / SupportCount;
	TArray<float> InitialLegX;
	for (const FLegState& State : LegState)
	{
		InitialLegX.Add(State.InitialX);
	}
	EdgePoint = Horizontal(FrontSum / FrontCount) + Heading * Profile.EdgeDistanceCm;

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
	const float ChassisFloor = High + Tuning.ChassisClearance - Tuning.ChassisUnderside;

	Steps.Reset();
	PlanLines.Reset();

	// A stance with every wheel down, as near the given height as the legs
	// allow. The robot may have started in one that is neither: the
	// differential stance holds the front corners clear and pitches the
	// chassis, and the gait's heights are all reckoned for a level chassis.
	auto AllDown = [this](float Level, float NearHeight, const TArray<float>& LegX, float& OutHeight) {
		FAxleGoal Down[3];
		for (int32 a = 0; a < 3; ++a)
		{
			Down[a] = { ELegGoal::Support, Level };
		}
		for (float D = 0.0f; D < 20.0f; D += 0.25f)
		{
			for (const float H : { NearHeight + D, NearHeight - D })
			{
				if (IsReachable(Down, H, LegX, -1000.0f, nullptr))
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
		Approach.Axles[a] = { ELegGoal::Hold, 0.0f };
	}
	Approach.LegX = InitialLegX;
	Approach.BaseHeight = InitialBaseHeight;

	// An axle already clear of the ground at the start -- the front corners in
	// the differential stance -- is not put down just to be lifted again: it
	// stays up from the approach straight into its own crossing.
	bool bStartsLifted[3] = { true, true, true };
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		if (!LegState[i].bHasWorld || LegState[i].WheelBottom < Tuning.SwingClearance)
		{
			bStartsLifted[static_cast<int32>(Legs[i].Axle)] = false;
		}
	}
	Approach.ApproachTo = (bClimb ? -(FrontRadius + Tuning.FaceMargin) : -Tuning.EdgeMargin) - Tuning.PrepareDistance;
	Approach.SpeedFraction = 1.0f;
	Steps.Add(Approach);

	float Height = Approach.BaseHeight;
	for (int32 AxleIndex = 0; AxleIndex < 3; ++AxleIndex)
	{
		const ERammsCurbAxle Axle = static_cast<ERammsCurbAxle>(AxleIndex);
		FAxleGoal			 Before[3], Swing[3], After[3];
		for (int32 a = 0; a < 3; ++a)
		{
			Before[a] = { ELegGoal::Support, a < AxleIndex ? StepHeight : 0.0f };
		}
		FMemory::Memcpy(Swing, Before, sizeof(Before));
		FMemory::Memcpy(After, Before, sizeof(Before));
		Swing[AxleIndex] = { ELegGoal::Swing, High };
		After[AxleIndex] = { ELegGoal::Support, StepHeight };
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

		// One pose for the whole crossing when there is one. When there is
		// not, lift from a pose that still reaches the ground behind, with
		// the robot stopped, then move to one it can drive on two axles from.
		const FAxleGoal* All[] = { Before, Swing, After };
		const FAxleGoal* Off[] = { Before, Swing };
		const FAxleGoal* On[] = { Swing, After };
		float			 LiftHeight = 0.0f, DriveHeight = 0.0f;
		TArray<float>	 LiftLegX, DriveLegX;
		const bool		 bOnePose = PlanLift(Axle, All, ChassisFloor, Height, Preferred, Tuning.MinStabilityMargin,
				  LiftHeight, LiftLegX, OutReason);
		if (bOnePose)
		{
			DriveHeight = LiftHeight;
			DriveLegX = LiftLegX;
		}
		else
		{
			FString OnePoseReason = OutReason;
			if (!PlanLift(Axle, Off, ChassisFloor, Height, Preferred, FMath::Min(Tuning.LiftOffMargin, Tuning.MinStabilityMargin),
					LiftHeight, LiftLegX, OutReason)
				|| !PlanLift(Axle, On, ChassisFloor, LiftHeight, Preferred, Tuning.MinStabilityMargin, DriveHeight,
					DriveLegX, OutReason))
			{
				OutReason = OnePoseReason;
				return false;
			}
		}
		Height = DriveHeight;

		FStep S;
		S.Axle = Axle;
		S.BaseHeight = LiftHeight;
		S.LegX = LiftLegX;

		S.Phase = ERammsCurbPhase::Prepare;
		FMemory::Memcpy(S.Axles, Before, sizeof(Before));
		S.SpeedFraction = Tuning.OverlapSpeedFraction;
		Steps.Add(S);

		// Lifting off on a reduced margin is done standing still.
		S.Phase = ERammsCurbPhase::Lift;
		FMemory::Memcpy(S.Axles, Swing, sizeof(Swing));
		S.SpeedFraction = bOnePose ? Tuning.OverlapSpeedFraction : 0.0f;
		Steps.Add(S);

		if (!bOnePose)
		{
			S.Phase = ERammsCurbPhase::Shift;
			S.BaseHeight = DriveHeight;
			S.LegX = DriveLegX;
			Steps.Add(S);
		}

		// Past the edge where it is farthest: progress is measured from its
		// near end, and a wheel landing short of its own stretch of edge lands
		// on the corner.
		S.Phase = ERammsCurbPhase::Advance;
		S.AdvanceTo = (bClimb ? Tuning.LandingOverlap : AxleRadius[AxleIndex] + Tuning.DropClearance) + Profile.EdgeSkewCm;
		S.SpeedFraction = 1.0f;
		Steps.Add(S);

		S.Phase = ERammsCurbPhase::Plant;
		FMemory::Memcpy(S.Axles, After, sizeof(After));
		S.SpeedFraction = Tuning.OverlapSpeedFraction;
		Steps.Add(S);
	}

	// Every wheel across: back to the starting ride height over the new ground,
	// all wheels down, then the starting stance.
	FStep Settle;
	Settle.Phase = ERammsCurbPhase::Settle;
	for (int32 a = 0; a < 3; ++a)
	{
		Settle.Axles[a] = { ELegGoal::Support, StepHeight };
	}
	Settle.LegX = InitialLegX;
	// Settle while driving clear of the edge, so the robot is not left with a
	// wheel on its lip.
	Settle.Axle = ERammsCurbAxle::Rear;
	Settle.AdvanceTo = AxleRadius[2] + Tuning.FinishClearance + Profile.EdgeSkewCm;
	Settle.SpeedFraction = Tuning.OverlapSpeedFraction;
	if (!AllDown(StepHeight, InitialBaseHeight + StepHeight, InitialLegX, Settle.BaseHeight))
	{
		OutReason = TEXT("no level stance with every wheel down is reachable on the far side");
		return false;
	}
	Steps.Add(Settle);

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
			Line += FString::Printf(TEXT(", until the front wheels are %.1f cm from the edge"), -S.ApproachTo);
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
	BaseHeightCmd = InitialBaseHeight;
	DriveSpeed = 0.0f;
	Elapsed = 0.0;
	StepLog.Reset();
	EnterStep(0);
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
	const FString Line = FString::Printf(TEXT("t=%5.2fs %-14s chassis %.1f (measured %.1f) cm, pitch %+.1f deg"),
		Elapsed, *Status, S.BaseHeight, MeasuredBaseHeight, MeasuredPitchDeg);
	StepLog.Add(Line);
	UE_LOG(LogRammsCurb, Log, TEXT("[Curb] %s"), *Line);
}

void URammsCurbNegotiationController::Finish(ERammsCurbPhase EndPhase, const FString& Why)
{
	Phase = EndPhase;
	Status = Why;
	const FString Line = FString::Printf(TEXT("t=%5.2fs %s: %s"), Elapsed, PhaseName(EndPhase), *Why);
	StepLog.Add(Line);
	UE_LOG(LogRammsCurb, Log, TEXT("[Curb] %s"), *Line);
	DriveSpeed = 0.0f;

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
	}
	return true;
}

void URammsCurbNegotiationController::CommandLegs(float DeltaTime)
{
	URammsRobotBaseComponent* Base = EnsureBase();
	const FStep&			  S = Steps[StepIndex];
	BaseHeightCmd = StepToward(BaseHeightCmd, S.BaseHeight, Tuning.BaseHeightSpeed * DeltaTime);

	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FRammsCurbLeg& Leg = Legs[i];
		FLegState&			 State = LegState[i];
		const FAxleGoal&	 Goal = S.Axles[static_cast<int32>(Leg.Axle)];
		const float			 GoalZ = CommandedGoalHeight(i, Goal);
		const float			 PrevX = State.ModelX;
		const float			 PrevZ = State.CmdZ;
		State.CmdZ = StepToward(State.CmdZ, GoalZ, Tuning.LegSpeed * DeltaTime);

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
			const float GoalX = StepToward(State.CmdX, TargetX, Tuning.CenterShiftSpeed * DeltaTime);
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
		State.ModelXVelocity = DeltaTime > 0.0f ? (State.ModelX - PrevX) / DeltaTime : 0.0f;
	}
}

float URammsCurbNegotiationController::ComputeDriveSpeed(float DeltaTime) const
{
	const FStep& S = Steps[StepIndex];
	float		 Target = Tuning.DriveSpeed * S.SpeedFraction;

	const float StepHeight = ActiveProfile.StepHeightCm;
	const bool	bClimb = StepHeight > 0.0f;
	const float High = FMath::Max(0.0f, StepHeight);
	const float Decel = Tuning.Acceleration;
	float		Cap = TNumericLimits<float>::Max();

	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FLegState& State = LegState[i];
		if (!State.bHasWorld)
		{
			continue;
		}
		const float R = Legs[i].WheelRadiusCm;
		const bool	bClear = State.WheelBottom >= High + 0.5f * Tuning.SwingClearance;
		if (bClimb)
		{
			// A wheel on the high side is past the face and limits nothing. Any
			// other wheel not yet clear of the curb top must stop short of it.
			const bool bOnTop = State.Progress > 0.0f && State.WheelBottom > 0.5f * StepHeight;
			if (!bClear && !bOnTop)
			{
				Cap = FMath::Min(Cap, BrakingSpeed(-Tuning.FaceMargin - (State.Progress + R), Decel));
			}
		}
		else
		{
			// A wheel still on the high side, carrying, must not roll off it.
			// One already out past the edge -- an axle being lowered, which
			// Advance left a radius and more beyond it -- is not on the high
			// side whatever its height on the way down, and limits nothing.
			const bool bOnTop = State.WheelBottom > 0.5f * StepHeight && State.Progress < R;
			if (!bClear && bOnTop)
			{
				Cap = FMath::Min(Cap, BrakingSpeed(-Tuning.EdgeMargin - State.Progress, Decel));
			}
		}
	}

	// Legs lagging their targets: give them time rather than drive on.
	float Lag = 0.0f;
	for (const FLegState& State : LegState)
	{
		Lag = FMath::Max(Lag, FMath::Abs(State.MeasuredZ - State.CmdZ));
	}
	if (Lag > 2.0f * Tuning.SettleTolerance)
	{
		Target *= 0.4f;
	}

	// Ramp towards the target; the braking caps are hard limits on top, and are
	// already shaped as braking curves, so they need no ramp of their own.
	const float Speed = Target > DriveSpeed
		? FMath::Min(DriveSpeed + Tuning.Acceleration * DeltaTime, Target)
		: FMath::Max(DriveSpeed - 2.0f * Tuning.Acceleration * DeltaTime, Target);
	return FMath::Max(0.0f, FMath::Min(Speed, Cap));
}

bool URammsCurbNegotiationController::MainDriveCarrying() const
{
	// On the ground means both: the step wants the wheel down, and it is --
	// measured, within a wheel's settling of the surface it should be on. The
	// step alone would hand drive back while a landing wheel is still in the
	// air, and the measurement alone would keep it on a wheel being lifted.
	const float Contact = 1.5f;
	bool		bAny = false;
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		if (Legs[i].Drive != ERammsCurbWheelDrive::Main)
		{
			continue;
		}
		bAny = true;
		const FAxleGoal& Goal = Steps[StepIndex].Axles[static_cast<int32>(Legs[i].Axle)];
		if (Goal.Goal == ELegGoal::Swing || !LegState[i].bHasWorld
			|| FMath::Abs(LegState[i].WheelBottom - Goal.Level) > Contact)
		{
			return false;
		}
	}
	return bAny;
}

void URammsCurbNegotiationController::CommandWheels(float Speed, float YawRate)
{
	URammsRobotBaseComponent* Base = EnsureBase();
	// Drive on the main wheels whenever they are planted, as the differential
	// mode does; every other wheel drives or rolls by its role.
	const bool bHasMain = Legs.ContainsByPredicate(
		[](const FRammsCurbLeg& Leg) { return Leg.Drive == ERammsCurbWheelDrive::Main; });
	const bool bMainDrives = bHasMain && MainDriveCarrying();
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FRammsCurbLeg& Leg = Legs[i];
		const FLegState&	 State = LegState[i];
		bool				 bDriven = true;
		switch (Leg.Drive)
		{
			case ERammsCurbWheelDrive::Main:
				bDriven = bMainDrives;
				break;
			case ERammsCurbWheelDrive::Fallback:
				bDriven = !bMainDrives;
				break;
			case ERammsCurbWheelDrive::Passive:
				bDriven = false;
				break;
			default:
				break;
		}
		if (!bDriven)
		{
			// Rolling free: no speed loop, no torque.
			Base->ClearMotorVelocityCommand(Leg.Wheel.MotorId);
			Base->SetMotorCommand(Leg.Wheel.MotorId, 0.0f);
			continue;
		}
		const double Theta = FMath::DegreesToRadians(Leg.Wheel.RollDirectionDeg);
		const double Dx = FMath::Cos(Theta);
		const double Dy = FMath::Sin(Theta);
		// The wheel's centre moves over the ground at the chassis speed plus
		// however fast its leg is carrying it fore/aft; a wheel that does not
		// roll along with its leg drags the robot instead.
		const double Vx = Speed + State.ModelXVelocity;
		const double Ground = Dx * Vx + YawRate * (State.ModelX * Dy - Leg.Wheel.Position.Y * Dx);
		const double Rate = Ground / FMath::Max(Leg.WheelRadiusCm, 0.1f);
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
			return AxleProgress(ERammsCurbAxle::Front) >= S.ApproachTo;
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
	if (!FMath::IsNearlyEqual(BaseHeightCmd, S.BaseHeight, 1e-3f))
	{
		return false;
	}
	for (int32 i = 0; i < Legs.Num(); ++i)
	{
		const FAxleGoal& Goal = S.Axles[static_cast<int32>(Legs[i].Axle)];
		if (!FMath::IsNearlyEqual(LegState[i].CmdZ, CommandedGoalHeight(i, Goal), 1e-3f))
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
	Elapsed += DeltaTime;
	StepElapsed += DeltaTime;
	if (!Measure())
	{
		return;
	}
	const float PitchRad = FMath::DegreesToRadians(MeasuredPitchDeg);
	FilteredPitchRad += (PitchRad - FilteredPitchRad) * FMath::Clamp(DeltaTime / 0.15f, 0.0f, 1.0f);

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
		}
		Finish(ERammsCurbPhase::Faulted,
			FString::Printf(TEXT("%s took longer than %.0f s%s%s"), *Status, Timeout,
				Lagging.IsEmpty() ? TEXT("") : TEXT("; legs off target:"), *Lagging));
		return;
	}

	CommandLegs(DeltaTime);
	DriveSpeed = ComputeDriveSpeed(DeltaTime);
	CommandWheels(DriveSpeed, Tuning.HeadingGain * HeadingErrorRad);

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

void URammsCurbNegotiationController::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
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
			// Nothing is left standing across a step to hold, so the mode from
			// before takes the robot back too.
			Finish(ERammsCurbPhase::Aborted, TEXT("the simulation was reset"));
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
		StepPlan(FMath::Min(Step, 0.1f));
	}
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
	URammsRobotBaseComponent* Base = EnsureBase();
	if (!GetOwner() || !GetWorld() || !Base)
	{
		OutReason = TEXT("no world or robot base to look from");
		return false;
	}

	// Heading from the crank pivots, and where the front wheels and the
	// ground under them are.
	FVector			FrontPivots = FVector::ZeroVector, RearPivots = FVector::ZeroVector;
	int32			NF = 0, NR = 0;
	TArray<FVector> FrontWheels;
	double			Ground = TNumericLimits<double>::Max();
	for (const FRammsCurbLeg& Leg : Legs)
	{
		FTransform Xf;
		if (Leg.Kind == ERammsCurbLegKind::FourBar && Leg.Axle != ERammsCurbAxle::Center
			&& Base->GetMotorTransform(Leg.FourBar.CrankMotor, Xf))
		{
			(Leg.Axle == ERammsCurbAxle::Front ? FrontPivots : RearPivots) += Xf.GetLocation();
			++(Leg.Axle == ERammsCurbAxle::Front ? NF : NR);
		}
		if (Base->GetMotorTransform(Leg.Wheel.MotorId, Xf))
		{
			Ground = FMath::Min(Ground, Xf.GetLocation().Z - Leg.WheelRadiusCm);
			if (Leg.Axle == ERammsCurbAxle::Front)
			{
				FrontWheels.Add(Xf.GetLocation());
			}
		}
	}
	if (NF == 0 || NR == 0 || FrontWheels.Num() == 0)
	{
		OutReason = TEXT("the robot's wheels and pivots cannot be located yet");
		return false;
	}
	const FVector Forward = Horizontal(FrontPivots / NF - RearPivots / NR).GetSafeNormal();

	auto SurfaceAt = [&](const FVector& At, double& OutZ) {
		return TraceSurface(At, Ground + 60.0, Ground - 60.0, OutZ);
	};
	// The surface has changed from Reference -- or is not there at all: a drop
	// deeper than the trace reaches is a change too, not a clear path.
	auto ChangedAt = [&](const FVector& At, double Reference) {
		double Z = 0.0;
		return !SurfaceAt(At, Z) || FMath::Abs(Z - Reference) > Tuning.MinStepHeight;
	};

	// Walk out along each front wheel's track until the surface changes height,
	// then bisect for the edge and sample either side of it.
	const double   Sample = 2.0;
	const double   Flat = 1.0; // a level surface varies by no more than this
	TArray<double> Edges, Heights;
	for (const FVector& Wheel : FrontWheels)
	{
		const FVector Start = Horizontal(Wheel);
		// The step is measured from the surface under the wheel. One already
		// off the ground the robot stands on puts the edge at or behind the
		// wheel -- a lifted front wheel overhanging a drop, say -- where every
		// limit reckoned from "ahead" would be wrong.
		double Under = 0.0;
		if (!SurfaceAt(Start, Under) || FMath::Abs(Under - Ground) > Tuning.MinStepHeight)
		{
			OutReason = TEXT("a front wheel is already over the step; back away from the edge first");
			return false;
		}
		double Near = 0.0;
		double Far = -1.0;
		for (double D = Sample; D <= Tuning.DetectRange; D += Sample)
		{
			if (ChangedAt(Start + Forward * D, Under))
			{
				Far = D;
				break;
			}
			Near = D;
		}
		if (Far < 0.0)
		{
			OutReason = FString::Printf(TEXT("no step within %.0f cm ahead"), Tuning.DetectRange);
			return false;
		}
		for (int32 i = 0; i < 12; ++i)
		{
			const double Mid = 0.5 * (Near + Far);
			(ChangedAt(Start + Forward * Mid, Under) ? Far : Near) = Mid;
		}

		// A step is level up to the edge, level beyond it, and changes all at
		// once between. A ramp creeps past MinStepHeight with the ground just
		// short of that point already well off level, and keeps going beyond.
		TArray<double> Beyond;
		for (const double D : { 4.0, 8.0, 12.0, 16.0 })
		{
			double Z = 0.0;
			if (SurfaceAt(Start + Forward * (Far + D), Z))
			{
				Beyond.Add(Z);
			}
		}
		if (Beyond.Num() < 4)
		{
			OutReason = FString::Printf(
				TEXT("an edge %.0f cm ahead with nothing beyond it -- a drop deeper than the robot can see"), Far);
			return false;
		}
		Beyond.Sort();
		double Before = 0.0;
		if (!SurfaceAt(Start + Forward * FMath::Max(0.0, Near - Sample), Before) || FMath::Abs(Before - Under) > Flat
			|| Beyond.Last() - Beyond[0] > Flat)
		{
			OutReason = FString::Printf(TEXT("the ground %.0f cm ahead slopes rather than steps"), Far);
			return false;
		}
		Edges.Add(Far);
		Heights.Add(0.5 * (Beyond[1] + Beyond[2]) - Before);
	}

	Edges.Sort();
	const double Skew = Edges.Last() - Edges[0];
	if (Skew > 10.0)
	{
		OutReason = FString::Printf(TEXT("the edge is %.0f cm closer to one front wheel than the other; "
										 "line up square to it"),
			Skew);
		return false;
	}
	double Height = 0.0;
	for (const double H : Heights)
	{
		Height += H;
	}
	Height /= Heights.Num();
	if (FMath::Abs(Height) > Tuning.MaxStepHeight)
	{
		OutReason = FString::Printf(TEXT("the step ahead is %.1f cm, beyond the %.1f cm limit"), Height, Tuning.MaxStepHeight);
		return false;
	}
	OutProfile.StepHeightCm = static_cast<float>(Height);
	OutProfile.EdgeDistanceCm = static_cast<float>(Edges[0]);
	OutProfile.EdgeSkewCm = static_cast<float>(Skew);
	OutReason = FString::Printf(TEXT("%.1f cm step %.1f cm ahead (skew %.1f cm)"), Height, Edges[0], Skew);
	return true;
}

// --- control surface -----------------------------------------------------------

void URammsCurbNegotiationController::DescribeControls(FRammsControlSurface& OutSurface) const
{
	// Offered whatever mode is active: starting a manoeuvre is what switches
	// the robot into this one.
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
