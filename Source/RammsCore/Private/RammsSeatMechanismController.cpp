// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsSeatMechanismController.h"

#include "Curb/RammsCurbSeat.h"
#include "Engine/DataTable.h"
#include "GameFramework/Actor.h"
#include "RammsCurbNegotiationController.h"
#include "RammsRobotBaseComponent.h"

URammsSeatMechanismController::URammsSeatMechanismController()
{
	// The jog stick integrates a rate into the pivot each frame.
	PrimaryComponentTick.bCanEverTick = true;
}

void URammsSeatMechanismController::BeginPlay()
{
	Super::BeginPlay();

	FRamms5BarLinkageSpec Resolved = Linkage;
	if (KinematicTable && !LinkageRow.IsNone())
	{
		static const FString Context(TEXT("RammsSeatMechanismController"));
		if (const FRamms5BarLinkageSpec* Row = KinematicTable->FindRow<FRamms5BarLinkageSpec>(LinkageRow, Context))
		{
			Resolved = *Row;
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("RammsSeatMechanismController on '%s': row '%s' not found in the kinematic table; using the inline linkage."),
				*GetNameSafe(GetOwner()), *LinkageRow.ToString());
		}
	}
	Mechanism = FRammsCurbSeat();
	Mechanism.Linkage = Resolved;
	Mechanism.PivotBeyondEndpointCm = PivotBeyondEndpointCm;
	Mechanism.TiltMotor = TiltMotor;
	Mechanism.TiltAngleSign = TiltAngleSign < 0.0f ? -1.0f : 1.0f;
	Mechanism.MaxPitchDeg = MaxTiltDeg;

	const RammsCurb::FSeatPose Zero = RammsCurb::SeatForward(Mechanism, FVector2D::ZeroVector);
	bResolved = Mechanism.IsSet() && Zero.bValid;
	LinkAngle0 = Zero.LinkAngle;
	if (!bResolved)
	{
		UE_LOG(LogTemp, Warning, TEXT("RammsSeatMechanismController on '%s': the seat's 5-bar is unset or does not close at its zero pose; no seat controls."),
			*GetNameSafe(GetOwner()));
	}
	bRegionMeasured = false;
	CachedOutline.Reset();
	EnsureBase();
}

URammsRobotBaseComponent* URammsSeatMechanismController::EnsureBase() const
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

bool URammsSeatMechanismController::IsBusy() const
{
	const AActor*						   Owner = GetOwner();
	const URammsCurbNegotiationController* Curb = Owner ? Owner->FindComponentByClass<URammsCurbNegotiationController>() : nullptr;
	return Curb && Curb->IsNegotiating();
}

double URammsSeatMechanismController::CommandOf(FName MotorId) const
{
	const URammsRobotBaseComponent* Base = EnsureBase();
	if (!Base || MotorId.IsNone())
	{
		return 0.0;
	}
	float Commanded = 0.0f;
	return Base->GetMotorCommand(MotorId, Commanded) ? static_cast<double>(Commanded) : static_cast<double>(Base->GetMotorValue(MotorId));
}

FVector2D URammsSeatMechanismController::CommandedAngles() const
{
	return FVector2D(CommandOf(Mechanism.Linkage.ProximalMotorA), CommandOf(Mechanism.Linkage.ProximalMotorB));
}

bool URammsSeatMechanismController::CommandedPose(FVector2D& OutPivot, double& OutPitchRad) const
{
	if (!bResolved || !EnsureBase())
	{
		return false;
	}
	const RammsCurb::FSeatPose Pose = RammsCurb::SeatForward(Mechanism, CommandedAngles());
	if (!Pose.bValid)
	{
		return false;
	}
	OutPivot = Pose.Pivot;
	OutPitchRad = RammsCurb::SeatPlatformPitch(Mechanism, Pose.LinkAngle, LinkAngle0, CommandOf(Mechanism.TiltMotor));
	return true;
}

bool URammsSeatMechanismController::WithinRange(FName MotorId, double Angle) const
{
	FRammsMotorSpec					Spec;
	const URammsRobotBaseComponent* Base = EnsureBase();
	if (Base && Base->GetMotorSpec(MotorId, Spec) && Spec.ControlRange.X < Spec.ControlRange.Y)
	{
		return Angle >= Spec.ControlRange.X && Angle <= Spec.ControlRange.Y;
	}
	return true;
}

bool URammsSeatMechanismController::Reachable(FVector2D Target, FVector2D Hint, FVector2D& OutAngles) const
{
	if (!bResolved || !FMath::IsFinite(Target.X) || !FMath::IsFinite(Target.Y))
	{
		return false;
	}
	const auto Within = [this, &OutAngles]() {
		return WithinRange(Mechanism.Linkage.ProximalMotorA, OutAngles.X) && WithinRange(Mechanism.Linkage.ProximalMotorB, OutAngles.Y);
	};
	// From the hint (the mechanism's own branch), else from the zero pose for
	// a target too far for Newton to walk to from where the seat is.
	return (RammsCurb::SeatSolve(Mechanism, Target, Hint, OutAngles) && Within())
		|| (RammsCurb::SeatSolve(Mechanism, Target, FVector2D::ZeroVector, OutAngles) && Within());
}

bool URammsSeatMechanismController::OwnsSeat() const
{
	const URammsRobotBaseComponent* Base = EnsureBase();
	float							A = 0.0f;
	float							B = 0.0f;
	return bHolding && Base && Base->GetMotorCommand(Mechanism.Linkage.ProximalMotorA, A)
		&& Base->GetMotorCommand(Mechanism.Linkage.ProximalMotorB, B) && FMath::IsNearlyEqual(A, HeldAngles.X, 1e-4)
		&& FMath::IsNearlyEqual(B, HeldAngles.Y, 1e-4);
}

bool URammsSeatMechanismController::TakeOver()
{
	URammsRobotBaseComponent* Base = EnsureBase();
	if (!bResolved || !Base || bSuspended || IsBusy())
	{
		return false;
	}
	if (OwnsSeat())
	{
		return true;
	}
	FVector2D Pivot;
	double	  Pitch = 0.0;
	if (!CommandedPose(Pivot, Pitch))
	{
		return false;
	}
	const double	Max = FMath::DegreesToRadians(static_cast<double>(MaxTiltDeg));
	const FVector2D Angles = CommandedAngles();
	// Kept as the floats the base stores, so OwnsSeat compares like with like.
	HeldAngles = FVector2D(static_cast<float>(Angles.X), static_cast<float>(Angles.Y));
	PivotNow = PivotGoal = Pivot;
	TiltNowRad = TiltGoalRad = FMath::Clamp(Pitch, -Max, Max);
	bLinkFiltered = false;
	bHolding = true;
	// Commanded where it already was: a 5-bar nobody had commanded yet (its
	// angles read where they stand) is held from here on.
	Base->SetMotorCommand(Mechanism.Linkage.ProximalMotorA, static_cast<float>(HeldAngles.X));
	Base->SetMotorCommand(Mechanism.Linkage.ProximalMotorB, static_cast<float>(HeldAngles.Y));
	return true;
}

bool URammsSeatMechanismController::SetPivotTarget(FVector2D TargetXZ)
{
	FVector2D Angles;
	if (!TakeOver() || !Reachable(TargetXZ, HeldAngles, Angles))
	{
		return false;
	}
	PivotGoal = TargetXZ;
	return true;
}

bool URammsSeatMechanismController::SetTiltDeg(float Deg)
{
	if (Mechanism.TiltMotor.IsNone() || !FMath::IsFinite(Deg) || !TakeOver())
	{
		return false;
	}
	TiltGoalRad = FMath::DegreesToRadians(FMath::Clamp(static_cast<double>(Deg), -static_cast<double>(MaxTiltDeg), static_cast<double>(MaxTiltDeg)));
	return true;
}

bool URammsSeatMechanismController::ResetSeat()
{
	FVector2D Angles;
	if (!TakeOver() || !Reachable(RestPivot, HeldAngles, Angles))
	{
		return false;
	}
	PivotGoal = RestPivot;
	if (!Mechanism.TiltMotor.IsNone())
	{
		TiltGoalRad = FMath::DegreesToRadians(
			FMath::Clamp(static_cast<double>(RestTiltDeg), -static_cast<double>(MaxTiltDeg), static_cast<double>(MaxTiltDeg)));
	}
	return true;
}

FVector2D URammsSeatMechanismController::GetPivot(bool& bValid) const
{
	const URammsRobotBaseComponent* Base = EnsureBase();
	bValid = false;
	if (!bResolved || !Base)
	{
		return FVector2D::ZeroVector;
	}
	const RammsCurb::FSeatPose Pose = RammsCurb::SeatForward(Mechanism,
		FVector2D(Base->GetMotorValue(Mechanism.Linkage.ProximalMotorA), Base->GetMotorValue(Mechanism.Linkage.ProximalMotorB)));
	bValid = Pose.bValid;
	return Pose.Pivot;
}

float URammsSeatMechanismController::GetTiltDeg(bool& bValid) const
{
	const URammsRobotBaseComponent* Base = EnsureBase();
	bValid = false;
	if (!bResolved || !Base)
	{
		return 0.0f;
	}
	const RammsCurb::FSeatPose Pose = RammsCurb::SeatForward(Mechanism,
		FVector2D(Base->GetMotorValue(Mechanism.Linkage.ProximalMotorA), Base->GetMotorValue(Mechanism.Linkage.ProximalMotorB)));
	bValid = Pose.bValid;
	const double Tilt = Mechanism.TiltMotor.IsNone() ? 0.0 : static_cast<double>(Base->GetMotorValue(Mechanism.TiltMotor));
	return static_cast<float>(FMath::RadiansToDegrees(RammsCurb::SeatPlatformPitch(Mechanism, Pose.LinkAngle, LinkAngle0, Tilt)));
}

void URammsSeatMechanismController::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (bSuspended || DeltaTime <= 0.0f)
	{
		return;
	}
	if (IsBusy())
	{
		// A crossing has the seat: the stick's deflection is dropped, not
		// saved up for when it hands the seat back, and the hold ends.
		Jog = FVector2D::ZeroVector;
		bHolding = false;
		return;
	}
	StepHold(DeltaTime);
}

void URammsSeatMechanismController::StepHold(float DeltaTime)
{
	if (!Jog.IsNearlyZero() && TakeOver())
	{
		// The stick moves the goal; against the edge of the reach it slides
		// along it on whichever axis still moves.
		const FVector2D Step = Jog * (JogRateCmPerSecond * DeltaTime);
		FVector2D		Angles;
		for (const FVector2D& Try : { Step, FVector2D(Step.X, 0.0), FVector2D(0.0, Step.Y) })
		{
			if (!Try.IsNearlyZero() && Reachable(PivotGoal + Try, HeldAngles, Angles))
			{
				PivotGoal += Try;
				break;
			}
		}
	}
	URammsRobotBaseComponent* Base = EnsureBase();
	if (!bHolding || !Base)
	{
		return;
	}
	if (!OwnsSeat())
	{
		// Someone else has commanded the 5-bar (a drive-mode stance, a curb
		// crossing): theirs now, until the next command here.
		bHolding = false;
		return;
	}

	// The pivot walked toward its goal, all three motors solved at each step.
	const FVector2D Delta = PivotGoal - PivotNow;
	if (!Delta.IsNearlyZero(1e-6))
	{
		const double	MaxStep = static_cast<double>(PivotSpeedCmPerSecond) * DeltaTime;
		const FVector2D Next = Delta.Size() <= MaxStep ? PivotGoal : PivotNow + Delta.GetSafeNormal() * MaxStep;
		FVector2D		Angles;
		if (Reachable(Next, HeldAngles, Angles))
		{
			HeldAngles = FVector2D(static_cast<float>(Angles.X), static_cast<float>(Angles.Y));
			PivotNow = Next;
			Base->SetMotorCommand(Mechanism.Linkage.ProximalMotorA, static_cast<float>(HeldAngles.X));
			Base->SetMotorCommand(Mechanism.Linkage.ProximalMotorB, static_cast<float>(HeldAngles.Y));
		}
		else
		{
			// The straight way leaves the reach: it stops where it got to.
			PivotGoal = PivotNow;
		}
	}

	if (Mechanism.TiltMotor.IsNone())
	{
		return;
	}
	const double TiltStep = FMath::DegreesToRadians(static_cast<double>(TiltSpeedDegPerSecond)) * DeltaTime;
	TiltNowRad += FMath::Clamp(TiltGoalRad - TiltNowRad, -TiltStep, TiltStep);
	// The tilt from the link where it stands, not where it was sent: the
	// loaded 5-bar sits short of its command, by enough near the top of its
	// travel to pitch the platform several degrees.
	const RammsCurb::FSeatPose Measured = RammsCurb::SeatForward(Mechanism,
		FVector2D(Base->GetMotorValue(Mechanism.Linkage.ProximalMotorA), Base->GetMotorValue(Mechanism.Linkage.ProximalMotorB)));
	const double			   Link = Measured.bValid ? Measured.LinkAngle : RammsCurb::SeatForward(Mechanism, HeldAngles).LinkAngle;
	if (!bLinkFiltered)
	{
		FilteredLinkAngle = Link;
		bLinkFiltered = true;
	}
	else
	{
		const double Alpha = LinkFilterSeconds > 0.0f ? FMath::Clamp(DeltaTime / static_cast<double>(LinkFilterSeconds), 0.0, 1.0) : 1.0;
		FilteredLinkAngle += FMath::FindDeltaAngleRadians(FilteredLinkAngle, Link) * Alpha;
	}
	Base->SetMotorCommand(Mechanism.TiltMotor, static_cast<float>(RammsCurb::SeatTiltFor(Mechanism, FilteredLinkAngle, LinkAngle0, TiltNowRad)));
}

bool URammsSeatMechanismController::MeasureRegion() const
{
	if (bRegionMeasured)
	{
		return CachedOutline.Num() >= 3;
	}
	const URammsRobotBaseComponent* Base = EnsureBase();
	if (!bResolved || !Base)
	{
		return false;
	}
	bRegionMeasured = true;
	CachedOutline.Reset();
	// The image of the motors' ranges, sampled: every pose the 5-bar closes in
	// with its knees inside their ranges (SeatSolve from the pose itself checks
	// them), binned by the pivot's height. Sampling the motors rather than
	// scanning targets needs no seed and finds every part of the region.
	auto RangeOf = [Base](FName Motor) {
		FRammsMotorSpec Spec;
		return Base->GetMotorSpec(Motor, Spec) && Spec.ControlRange.X < Spec.ControlRange.Y ? Spec.ControlRange : FVector2D(-PI, PI);
	};
	const FVector2D		   RangeA = RangeOf(Mechanism.Linkage.ProximalMotorA);
	const FVector2D		   RangeB = RangeOf(Mechanism.Linkage.ProximalMotorB);
	const int32			   N = FMath::Max(RegionSamples, 8);
	TMap<int32, FVector2D> Rows; // height (cm, rounded) -> fore/aft min, max
	for (int32 i = 0; i <= N; ++i)
	{
		for (int32 j = 0; j <= N; ++j)
		{
			const FVector2D			   Angles(FMath::Lerp(RangeA.X, RangeA.Y, static_cast<double>(i) / N), FMath::Lerp(RangeB.X, RangeB.Y, static_cast<double>(j) / N));
			const RammsCurb::FSeatPose Pose = RammsCurb::SeatForward(Mechanism, Angles);
			FVector2D				   Check;
			if (!Pose.bValid || !RammsCurb::SeatSolve(Mechanism, Pose.Pivot, Angles, Check))
			{
				continue;
			}
			const int32 Key = FMath::RoundToInt32(Pose.Pivot.Y);
			if (FVector2D* Row = Rows.Find(Key))
			{
				Row->X = FMath::Min(Row->X, Pose.Pivot.X);
				Row->Y = FMath::Max(Row->Y, Pose.Pivot.X);
			}
			else
			{
				Rows.Add(Key, FVector2D(Pose.Pivot.X, Pose.Pivot.X));
			}
		}
	}
	if (Rows.Num() < 2)
	{
		return false;
	}
	TArray<int32> Heights;
	Rows.GetKeys(Heights);
	Heights.Sort();
	// Up the aft edge, down the forward one.
	for (const int32 Z : Heights)
	{
		CachedOutline.Add(FVector2D(Rows[Z].X, Z));
	}
	for (int32 k = Heights.Num() - 1; k >= 0; --k)
	{
		CachedOutline.Add(FVector2D(Rows[Heights[k]].Y, Heights[k]));
	}
	ExtentZ = FVector2D(Heights[0], Heights.Last());
	ExtentX = FVector2D(TNumericLimits<double>::Max(), TNumericLimits<double>::Lowest());
	for (const TPair<int32, FVector2D>& Row : Rows)
	{
		ExtentX.X = FMath::Min(ExtentX.X, Row.Value.X);
		ExtentX.Y = FMath::Max(ExtentX.Y, Row.Value.Y);
	}
	return true;
}

TArray<FVector2D> URammsSeatMechanismController::GetReachableOutline(bool& bValid) const
{
	bValid = MeasureRegion();
	return bValid ? CachedOutline : TArray<FVector2D>();
}

void URammsSeatMechanismController::DescribeControls(FRammsControlSurface& OutSurface) const
{
	if (bSuspended || !bResolved || !EnsureBase() || !MeasureRegion())
	{
		return;
	}
	bool			bPivot = false;
	const FVector2D Now = GetPivot(bPivot);

	// The pivot: a pad over the region it reaches.
	FRammsControlAxis Height;
	Height.Id = HeightControlId();
	Height.Group = Group();
	Height.DisplayName = NSLOCTEXT("RammsSeat", "Height", "Seat height");
	Height.Kind = ERammsControlKind::Position;
	Height.Units = ERammsControlUnits::Centimeters;
	Height.Range = ExtentZ;
	Height.DefaultValue = bPivot ? FMath::Clamp(static_cast<float>(Now.Y), static_cast<float>(ExtentZ.X), static_cast<float>(ExtentZ.Y)) : static_cast<float>(ExtentZ.X);
	Height.Order = 0;
	Height.PairedAxis = ForeAftControlId();
	Height.RegionOutline = CachedOutline;
	OutSurface.Add(Height);

	FRammsControlAxis ForeAft = Height;
	ForeAft.Id = ForeAftControlId();
	ForeAft.DisplayName = NSLOCTEXT("RammsSeat", "ForeAft", "Seat fore/aft");
	ForeAft.Range = ExtentX;
	ForeAft.DefaultValue = bPivot ? FMath::Clamp(static_cast<float>(Now.X), static_cast<float>(ExtentX.X), static_cast<float>(ExtentX.Y)) : static_cast<float>(ExtentX.X);
	ForeAft.Order = 1;
	ForeAft.PairedAxis = HeightControlId();
	ForeAft.RegionOutline.Reset();
	OutSurface.Add(ForeAft);

	// A rate stick for the same two, which never has to claim a range.
	FRammsControlAxis Up;
	Up.Id = JogUpControlId();
	Up.Group = Group();
	Up.DisplayName = NSLOCTEXT("RammsSeat", "JogUp", "Seat jog up");
	Up.Kind = ERammsControlKind::Continuous;
	Up.Units = ERammsControlUnits::Normalized;
	Up.Range = FVector2D(-1.0, 1.0);
	Up.bReadback = false;
	Up.Order = 2;
	Up.PairedAxis = JogForeAftControlId();
	OutSurface.Add(Up);

	FRammsControlAxis Fwd = Up;
	Fwd.Id = JogForeAftControlId();
	Fwd.DisplayName = NSLOCTEXT("RammsSeat", "JogForeAft", "Seat jog fore/aft");
	Fwd.Order = 3;
	Fwd.PairedAxis = JogUpControlId();
	OutSurface.Add(Fwd);

	// Back to rest -- only when the mechanism reaches it, since a button that
	// always refuses is worse than none.
	FVector2D Check;
	if (Reachable(RestPivot, CommandedAngles(), Check))
	{
		FRammsControlAxis Reset;
		Reset.Id = ResetControlId();
		Reset.Group = Group();
		Reset.DisplayName = NSLOCTEXT("RammsSeat", "Reset", "Seat reset");
		Reset.Kind = ERammsControlKind::Action;
		Reset.bReadback = false;
		Reset.Order = 4;
		OutSurface.Add(Reset);
	}

	// The tilt, on its own: the pivot's controls above keep it.
	if (!Mechanism.TiltMotor.IsNone())
	{
		bool			  bTilt = false;
		const float		  TiltNow = GetTiltDeg(bTilt);
		FRammsControlAxis Tilt;
		Tilt.Id = TiltControlId();
		Tilt.Group = Group();
		Tilt.DisplayName = NSLOCTEXT("RammsSeat", "Tilt", "Seat tilt");
		Tilt.Kind = ERammsControlKind::Position;
		Tilt.Units = ERammsControlUnits::Degrees;
		Tilt.Range = FVector2D(-MaxTiltDeg, MaxTiltDeg);
		Tilt.DefaultValue = bTilt ? FMath::Clamp(TiltNow, -MaxTiltDeg, MaxTiltDeg) : 0.0f;
		Tilt.Order = 5;
		OutSurface.Add(Tilt);
	}
}

bool URammsSeatMechanismController::ApplyControl(FName Id, float Value)
{
	if (bSuspended)
	{
		return false;
	}
	if (Id == HeightControlId() || Id == ForeAftControlId())
	{
		// On the goal, so a pad's two axes written one after the other compose.
		if (!TakeOver())
		{
			return false;
		}
		FVector2D Goal = PivotGoal;
		(Id == HeightControlId() ? Goal.Y : Goal.X) = Value;
		return SetPivotTarget(Goal);
	}
	if (Id == TiltControlId())
	{
		return SetTiltDeg(Value);
	}
	if (Id == JogUpControlId() || Id == JogForeAftControlId())
	{
		const bool	bUp = Id == JogUpControlId();
		const float Clamped = FMath::Clamp(Value, -1.0f, 1.0f);
		bool&		bSuppressed = bUp ? bSuppressJogY : bSuppressJogX;
		// Held through a reset: swallowed until it comes back to neutral
		// (URamms5BarLinkageController's reasons).
		if (bSuppressed)
		{
			if (FMath::Abs(Clamped) > JogNeutralThreshold)
			{
				return true;
			}
			bSuppressed = false;
		}
		(bUp ? Jog.Y : Jog.X) = Clamped;
		return true;
	}
	return false;
}

bool URammsSeatMechanismController::TriggerControl(FName Id)
{
	if (bSuspended || Id != ResetControlId())
	{
		return false;
	}
	Jog = FVector2D::ZeroVector;
	bSuppressJogX = true;
	bSuppressJogY = true;
	return ResetSeat();
}

bool URammsSeatMechanismController::ReleaseControl(FName Id)
{
	// Letting go of the stick stops the seat where it got to; letting go of a
	// position holds it where it was put -- a seat under its rider is never
	// let go of.
	if (Id == JogUpControlId())
	{
		Jog.Y = 0.0f;
		bSuppressJogY = false;
		return true;
	}
	if (Id == JogForeAftControlId())
	{
		Jog.X = 0.0f;
		bSuppressJogX = false;
		return true;
	}
	return Id == HeightControlId() || Id == ForeAftControlId() || Id == TiltControlId();
}

bool URammsSeatMechanismController::ReadControl(FName Id, float& OutValue) const
{
	bool bValid = false;
	if (Id == HeightControlId() || Id == ForeAftControlId())
	{
		const FVector2D Pivot = GetPivot(bValid);
		OutValue = static_cast<float>(Id == HeightControlId() ? Pivot.Y : Pivot.X);
		return bValid;
	}
	if (Id == TiltControlId())
	{
		OutValue = GetTiltDeg(bValid);
		return bValid;
	}
	return false;
}

bool URammsSeatMechanismController::ReadTarget(FName Id, float& OutTarget) const
{
	// Holding: where it is going. Otherwise where it was last sent, by anyone.
	FVector2D Pivot = PivotGoal;
	double	  Pitch = TiltGoalRad;
	if (!OwnsSeat() && !CommandedPose(Pivot, Pitch))
	{
		return false;
	}
	if (Id == HeightControlId() || Id == ForeAftControlId())
	{
		OutTarget = static_cast<float>(Id == HeightControlId() ? Pivot.Y : Pivot.X);
		return true;
	}
	if (Id == TiltControlId())
	{
		OutTarget = static_cast<float>(FMath::RadiansToDegrees(Pitch));
		return true;
	}
	return false;
}

void URammsSeatMechanismController::GetClaimedMotorIds(TArray<FName>& OutIds) const
{
	if (bSuspended || !bResolved)
	{
		return;
	}
	OutIds.Add(Mechanism.Linkage.ProximalMotorA);
	OutIds.Add(Mechanism.Linkage.ProximalMotorB);
	if (!Mechanism.TiltMotor.IsNone())
	{
		OutIds.Add(Mechanism.TiltMotor);
	}
}

void URammsSeatMechanismController::SetContributionSuspended(bool bInSuspended)
{
	// Standing down stops the stick and offers nothing, but lets go of no
	// motor: the seat carries its rider, and the motors keep their last
	// commands until whoever now drives them by hand moves them.
	bSuspended = bInSuspended;
	if (bSuspended)
	{
		Jog = FVector2D::ZeroVector;
		bHolding = false;
	}
}
