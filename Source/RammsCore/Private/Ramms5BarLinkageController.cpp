// Copyright Epic Games, Inc. All Rights Reserved.

#include "Ramms5BarLinkageController.h"
#include "Ramms5BarKinematics.h"
#include "RammsRobotBaseComponent.h"
#include "GameFramework/Actor.h"

URamms5BarLinkageController::URamms5BarLinkageController()
{
	// The jog axes integrate a rate into the endpoint target each frame.
	PrimaryComponentTick.bCanEverTick = true;
}

void URamms5BarLinkageController::BeginPlay()
{
	Super::BeginPlay();

	// bCanEverTick is serialised on the Blueprint's component template, so a
	// linkage authored before the jog axes existed will never tick however the
	// CDO is set -- and the stick would silently do nothing on every existing
	// robot. Turn it on for this instance instead of asking everyone to
	// re-save their Blueprints.
	if (!PrimaryComponentTick.bCanEverTick)
	{
		PrimaryComponentTick.bCanEverTick = true;
		PrimaryComponentTick.SetTickFunctionEnable(true);
		PrimaryComponentTick.RegisterTickFunction(GetComponentLevel());
	}

	// Resolve the kinematic spec: a table row if configured, else the inline one.
	Resolved = Linkage;
	if (KinematicTable && !LinkageRow.IsNone())
	{
		static const FString Context(TEXT("Ramms5BarLinkageController"));
		if (const FRamms5BarLinkageSpec* Row = KinematicTable->FindRow<FRamms5BarLinkageSpec>(LinkageRow, Context))
		{
			Resolved = *Row;
		}
		else
		{
			UE_LOG(LogTemp, Warning,
				TEXT("Ramms5BarLinkageController on '%s': row '%s' not found in the kinematic table; using inline config."),
				GetOwner() ? *GetOwner()->GetName() : TEXT("?"), *LinkageRow.ToString());
		}
	}

	// Prime the base-component cache (also resolved lazily if BeginPlay order
	// hasn't reached the base component yet).
	EnsureBase();
}

URammsRobotBaseComponent* URamms5BarLinkageController::EnsureBase() const
{
	if (!BaseComponent)
	{
		if (const AActor* Owner = GetOwner())
		{
			BaseComponent = Owner->FindComponentByClass<URammsRobotBaseComponent>();
			if (BaseComponent)
			{
				TArray<URammsRobotBaseComponent*> Bases;
				Owner->GetComponents<URammsRobotBaseComponent>(Bases);
				if (Bases.Num() > 1)
				{
					UE_LOG(LogTemp, Warning,
						TEXT("Ramms5BarLinkageController '%s': '%s' has %d RammsRobotBaseComponents; using '%s'. Remove the extras."),
						*GetName(), *Owner->GetName(), Bases.Num(), *BaseComponent->GetName());
				}
			}
		}
	}
	return BaseComponent;
}

bool URamms5BarLinkageController::HasBase() const
{
	return EnsureBase() != nullptr;
}

float URamms5BarLinkageController::Handedness() const
{
	// Anything but a clean -1 is treated as +1: a zero or unset sign would
	// collapse every fore/aft command to the centre of travel.
	return Resolved.TranslationSign < 0.0f ? -1.0f : 1.0f;
}

FVector2D URamms5BarLinkageController::ToLocal(FVector2D RobotXZ) const
{
	return FVector2D(RobotXZ.X * Handedness(), RobotXZ.Y);
}

FVector2D URamms5BarLinkageController::ToRobot(FVector2D LocalXZ) const
{
	// The sign is its own inverse, so this is ToLocal again -- named for the
	// direction it is used in, because which frame a coordinate is in is the
	// easy thing to get wrong here.
	return FVector2D(LocalXZ.X * Handedness(), LocalXZ.Y);
}

bool URamms5BarLinkageController::WithinMotorRange(URammsRobotBaseComponent& Base, FName MotorId, double Angle) const
{
	// A geometrically reachable endpoint can still need a hip angle outside the
	// actuator's authored range; the base component would clamp it silently and
	// the linkage would settle somewhere else. Treat that as unreachable. (An
	// unset/zero-width range defers to the backend and can't be checked here.)
	FRammsMotorSpec Spec;
	if (Base.GetMotorSpec(MotorId, Spec) && Spec.ControlRange.X < Spec.ControlRange.Y)
	{
		return Angle >= Spec.ControlRange.X && Angle <= Spec.ControlRange.Y;
	}
	return true;
}

bool URamms5BarLinkageController::SetEndpointTarget(FVector2D TargetXZ)
{
	URammsRobotBaseComponent* Base = EnsureBase();
	if (!Base)
	{
		return false;
	}

	bool			bReachable = false;
	const FVector2D Angles = URamms5BarKinematics::SolveIK(Resolved, ToLocal(TargetXZ), bReachable);
	bReachable = bReachable
		&& WithinMotorRange(*Base, Resolved.ProximalMotorA, Angles.X)
		&& WithinMotorRange(*Base, Resolved.ProximalMotorB, Angles.Y);
	if (!bReachable)
	{
		UE_LOG(LogTemp, Verbose,
			TEXT("Ramms5BarLinkageController: endpoint (%.2f, %.2f) unreachable — not commanding."),
			TargetXZ.X, TargetXZ.Y);
		return false;
	}

	Base->SetMotorCommand(Resolved.ProximalMotorA, static_cast<float>(Angles.X));
	Base->SetMotorCommand(Resolved.ProximalMotorB, static_cast<float>(Angles.Y));
	LastTarget = TargetXZ;
	bHasTarget = true;
	return true;
}

float URamms5BarLinkageController::HeldX() const
{
	if (bHasTarget)
	{
		return static_cast<float>(LastTarget.X);
	}
	// The resting pose can sit outside what the motors' ControlRanges allow --
	// the linkage settles under load a little past where the IK will follow, so
	// SolveTarget refuses the very point the endpoint is at. Handing that
	// coordinate to a height command would ask for a target that is infeasible
	// in the axis the caller did not touch, and the move silently does nothing.
	// Clamp into what is actually reachable instead.
	const FVector2D Live = GetCurrentEndpoint();
	bool			bValid = false;
	const FVector2D Range = GetReachableTranslationRange(static_cast<float>(Live.Y), bValid);
	return bValid ? FMath::Clamp(static_cast<float>(Live.X), static_cast<float>(Range.X),
						static_cast<float>(Range.Y))
				  : static_cast<float>(Live.X);
}

bool URamms5BarLinkageController::SetEndpointHeight(float Z)
{
	// Keep the current (or last-commanded) endpoint X, change only height.
	return SetEndpointTarget(FVector2D(HeldX(), Z));
}

float URamms5BarLinkageController::HeldZ() const
{
	if (bHasTarget)
	{
		return static_cast<float>(LastTarget.Y);
	}
	// See HeldX: the live height can be outside the reachable band, and a
	// fore/aft command that inherited it would be refused or do nothing.
	const FVector2D Live = GetCurrentEndpoint();
	bool			bValid = false;
	const FVector2D Range = GetReachableHeightRange(static_cast<float>(Live.X), bValid);
	return bValid ? FMath::Clamp(static_cast<float>(Live.Y), static_cast<float>(Range.X),
						static_cast<float>(Range.Y))
				  : static_cast<float>(Live.Y);
}

bool URamms5BarLinkageController::SetEndpointTranslation(float X)
{
	// The mirror of SetEndpointHeight: hold the height, move fore/aft.
	return SetEndpointTarget(FVector2D(X, HeldZ()));
}

FVector2D URamms5BarLinkageController::ScanReachable(TFunctionRef<bool(float)> Reachable,
	float Start, float Lo, float Hi, float Step, bool& bValid) const
{
	// Walk out from Start until the IK (with the motors' ControlRanges) first
	// refuses; the mechanism's reach is contiguous there. Starting at the live
	// value rather than the scan floor also keeps a disconnected reachable
	// island elsewhere from being reported.
	bValid = false;
	Step = FMath::Max(Step, 0.05f);
	const float S0 = FMath::Clamp(Start, Lo, Hi);

	// Find any reachable seed nearby (the live pose can sit a hair outside the
	// range while settling).
	float Seed = S0;
	bool  bSeed = Reachable(Seed);
	for (float D = Step; !bSeed && D <= 8.0f * Step; D += Step)
	{
		if (Seed - D >= Lo && Reachable(Seed - D))
		{
			Seed -= D;
			bSeed = true;
		}
		else if (Seed + D <= Hi && Reachable(Seed + D))
		{
			Seed += D;
			bSeed = true;
		}
	}
	if (!bSeed)
	{
		return FVector2D(S0, S0);
	}

	float Min = Seed;
	while (Min - Step >= Lo && Reachable(Min - Step))
	{
		Min -= Step;
	}
	float Max = Seed;
	while (Max + Step <= Hi && Reachable(Max + Step))
	{
		Max += Step;
	}
	// A single reachable sample (equal scan limits, or an interval narrower
	// than the step) is not a range: a zero-width one would read as unbounded.
	bValid = Max > Min;
	return FVector2D(Min, Max);
}

FVector2D URamms5BarLinkageController::GetReachableHeightRange(float X, bool& bValid) const
{
	auto Reachable = [this, X](float Z) {
		bool bOk = false;
		SolveTarget(FVector2D(X, Z), bOk);
		return bOk;
	};
	return ScanReachable(Reachable, static_cast<float>(GetCurrentEndpoint().Y),
		static_cast<float>(FMath::Min(HeightScanLimits.X, HeightScanLimits.Y)),
		static_cast<float>(FMath::Max(HeightScanLimits.X, HeightScanLimits.Y)),
		HeightScanStep, bValid);
}

FVector2D URamms5BarLinkageController::GetReachableTranslationRange(float Z, bool& bValid) const
{
	auto Reachable = [this, Z](float X) {
		bool bOk = false;
		SolveTarget(FVector2D(X, Z), bOk);
		return bOk;
	};
	return ScanReachable(Reachable, static_cast<float>(GetCurrentEndpoint().X),
		static_cast<float>(FMath::Min(TranslationScanLimits.X, TranslationScanLimits.Y)),
		static_cast<float>(FMath::Max(TranslationScanLimits.X, TranslationScanLimits.Y)),
		HeightScanStep, bValid);
}

FVector2D URamms5BarLinkageController::GetReachableExtent(bool bHeight, bool& bValid) const
{
	bValid = false;

	// Walk the other axis across its own reachable span and union the slices.
	// Seeded from where the endpoint is, so the span followed is the one this
	// mechanism is actually on.
	const FVector2D Live = GetCurrentEndpoint();
	bool			bAcross = false;
	const FVector2D Across = bHeight ? GetReachableTranslationRange(static_cast<float>(Live.Y), bAcross)
									 : GetReachableHeightRange(static_cast<float>(Live.X), bAcross);
	if (!bAcross)
	{
		// Nothing reachable across: fall back to the slice at the live pose so
		// an axis is still offered if one exists there at all.
		return bHeight ? GetReachableHeightRange(static_cast<float>(Live.X), bValid)
					   : GetReachableTranslationRange(static_cast<float>(Live.Y), bValid);
	}

	const int32 Samples = FMath::Max(2, ExtentScanSamples);
	FVector2D	Extent(TNumericLimits<float>::Max(), -TNumericLimits<float>::Max());
	for (int32 i = 0; i < Samples; ++i)
	{
		const float		T = static_cast<float>(i) / static_cast<float>(Samples - 1);
		const float		At = FMath::Lerp(static_cast<float>(Across.X), static_cast<float>(Across.Y), T);
		bool			bSlice = false;
		const FVector2D Slice = bHeight ? GetReachableHeightRange(At, bSlice)
										: GetReachableTranslationRange(At, bSlice);
		if (!bSlice)
		{
			continue;
		}
		Extent.X = FMath::Min(Extent.X, Slice.X);
		Extent.Y = FMath::Max(Extent.Y, Slice.Y);
		bValid = true;
	}
	return bValid ? Extent : FVector2D::ZeroVector;
}

TArray<FVector2D> URamms5BarLinkageController::GetReachableOutline(bool& bValid) const
{
	bValid = false;
	TArray<FVector2D> Outline;

	// The whole height the mechanism reaches, not the slice above where the
	// endpoint happens to be standing. The slice is what GetReachableHeightRange
	// gives, and walking it made the published region CHANGE SHAPE as the
	// endpoint moved fore/aft -- heights valid at other X were never sampled,
	// so a pad drew a different region depending on the pose it was describing.
	// GetReachableExtent unions the slices across X and keeps the same
	// live-seeded branch selection, since a 5-bar's reachable set is per
	// assembly branch.
	bool			bSpan = false;
	const FVector2D Span = GetReachableExtent(/*bHeight=*/true, bSpan);
	if (!bSpan)
	{
		return Outline;
	}

	const int32 Samples = FMath::Max(3, OutlineScanSamples);

	// Near edge going up, far edge coming back down, so the points are already
	// in traversal order and the polygon closes on itself without a renderer
	// having to sort anything.
	TArray<FVector2D> Near;
	TArray<FVector2D> Far;
	Near.Reserve(Samples);
	Far.Reserve(Samples);

	for (int32 i = 0; i < Samples; ++i)
	{
		const float		T = static_cast<float>(i) / static_cast<float>(Samples - 1);
		const float		Z = FMath::Lerp(static_cast<float>(Span.X), static_cast<float>(Span.Y), T);
		bool			bSlice = false;
		const FVector2D Slice = GetReachableTranslationRange(Z, bSlice);
		if (!bSlice)
		{
			// A height with nothing reachable across it. Skipped rather than
			// closed off: the scan walks a bounding span, so the ends of it can
			// fall outside a region that is narrower than its own extremes.
			continue;
		}
		Near.Emplace(Slice.X, Z);
		Far.Emplace(Slice.Y, Z);
	}

	if (Near.Num() < 2)
	{
		// One slice, or none. A polygon cannot be made of that, and a renderer
		// handed two points would draw a line and imply the mechanism is one.
		return Outline;
	}

	Outline.Reserve(Near.Num() * 2);
	Outline.Append(Near);
	for (int32 i = Far.Num() - 1; i >= 0; --i)
	{
		Outline.Add(Far[i]);
	}

	bValid = Outline.Num() >= 3;
	if (!bValid)
	{
		Outline.Reset();
	}
	return Outline;
}

void URamms5BarLinkageController::SetJointAngles(FVector2D AnglesAB)
{
	URammsRobotBaseComponent* Base = EnsureBase();
	if (!Base)
	{
		return;
	}
	Base->SetMotorCommand(Resolved.ProximalMotorA, static_cast<float>(AnglesAB.X));
	Base->SetMotorCommand(Resolved.ProximalMotorB, static_cast<float>(AnglesAB.Y));

	// These angles replace whatever SetEndpointTarget commanded, so the
	// endpoint target has to follow them (forward kinematics) or the control
	// surface keeps reporting the old height. Angles that don't close the
	// linkage describe no endpoint at all: then there is no target.
	bool			bValid = false;
	const FVector2D Endpoint = URamms5BarKinematics::ComputeEndpoint(Resolved, AnglesAB, bValid);
	bHasTarget = bValid;
	if (bValid)
	{
		LastTarget = ToRobot(Endpoint);
	}
}

FVector2D URamms5BarLinkageController::GetCurrentJointAngles() const
{
	URammsRobotBaseComponent* Base = EnsureBase();
	if (!Base)
	{
		return FVector2D::ZeroVector;
	}
	return FVector2D(Base->GetMotorValue(Resolved.ProximalMotorA), Base->GetMotorValue(Resolved.ProximalMotorB));
}

FVector2D URamms5BarLinkageController::GetCurrentEndpoint() const
{
	bool bValid = false;
	return GetCurrentEndpointChecked(bValid);
}

FVector2D URamms5BarLinkageController::GetCurrentEndpointChecked(bool& bValid) const
{
	return ToRobot(URamms5BarKinematics::ComputeEndpoint(Resolved, GetCurrentJointAngles(), bValid));
}

FVector2D URamms5BarLinkageController::SolveTarget(FVector2D TargetXZ, bool& bReachable) const
{
	const FVector2D Angles = URamms5BarKinematics::SolveIK(Resolved, ToLocal(TargetXZ), bReachable);
	if (URammsRobotBaseComponent* Base = EnsureBase())
	{
		bReachable = bReachable
			&& WithinMotorRange(*Base, Resolved.ProximalMotorA, Angles.X)
			&& WithinMotorRange(*Base, Resolved.ProximalMotorB, Angles.Y);
	}
	return Angles;
}

void URamms5BarLinkageController::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (bSuspended || Jog.IsNearlyZero() || DeltaTime <= 0.0f)
	{
		return;
	}

	// Integrate from where we are holding, not from the live endpoint: the
	// live pose lags the target under load, and seeding from it every frame
	// would make the stick fight the mechanism instead of commanding it.
	const FVector2D From(HeldX(), HeldZ());
	FVector2D		Candidate = From + Jog * (JogRateCmPerSecond * DeltaTime);

	// The reachable set is a curved region, not a rectangle, so stepping one
	// axis can leave it even when both coordinates are individually in range --
	// and SetEndpointTarget then refuses, every frame, and the stick does
	// nothing. Clamp the moving axis into the band reachable at the other.
	bool bValid = false;
	if (!FMath::IsNearlyZero(Jog.Y))
	{
		const FVector2D Band = GetReachableHeightRange(static_cast<float>(Candidate.X), bValid);
		if (bValid)
		{
			Candidate.Y = FMath::Clamp(Candidate.Y, Band.X, Band.Y);
		}
	}
	if (!FMath::IsNearlyZero(Jog.X))
	{
		const FVector2D Band = GetReachableTranslationRange(static_cast<float>(Candidate.Y), bValid);
		if (bValid)
		{
			Candidate.X = FMath::Clamp(Candidate.X, Band.X, Band.Y);
		}
	}

	// Only adopt the new target when the linkage accepts it, so holding the
	// stick at a limit does not wind the target off into the unreachable --
	// the same rule the keyboard raise/lower uses.
	const bool bTookIt = SetEndpointTarget(Candidate);
	if (!bLoggedJog)
	{
		bLoggedJog = true;
		bool bSolvable = false;
		SolveTarget(Candidate, bSolvable);
		const FVector2D Live = GetCurrentEndpoint();
		// One line on the first jog, so a stick that appears to do nothing can
		// be told apart from one whose target is being refused.
		UE_LOG(LogTemp, Verbose,
			TEXT("[5Bar] '%s' jog: live=(%.3f, %.3f) from=(%.3f, %.3f) candidate=(%.3f, %.3f) "
				 "solvable=%d accepted=%d"),
			*GetName(), Live.X, Live.Y, From.X, From.Y, Candidate.X, Candidate.Y,
			bSolvable ? 1 : 0, bTookIt ? 1 : 0);
	}
}

// --- control surface -----------------------------------------------------------

void URamms5BarLinkageController::DescribeEndpointAxis(FRammsControlSurface& OutSurface, bool bHeight) const
{
	FRammsControlAxis Axis;
	Axis.Id = bHeight ? HeightControlId() : TranslationControlId();
	Axis.Group = RammsControlIds::Groups::Linkage();
	const FString Pretty = GetName().Replace(TEXT("Linkage"), TEXT("")).Replace(TEXT("_"), TEXT(" "));
	Axis.DisplayName = FText::FromString(Pretty + (bHeight ? TEXT(" height") : TEXT(" fore/aft")));
	Axis.Kind = ERammsControlKind::Position;
	Axis.Units = ERammsControlUnits::Centimeters;
	Axis.Order = bHeight ? 0 : 1;
	Axis.Range = bHeight ? EndpointHeightRange : EndpointTranslationRange;
	if (Axis.Range.X >= Axis.Range.Y)
	{
		bool bValid = false;
		// The mechanism's whole travel, not the slice at the current pose. A
		// slice is only valid while the other axis stays put, and the surface
		// clamps to whatever is advertised -- so publishing one froze the
		// fore/aft slider into the roughly one-centimetre band available at
		// the resting pose and left it unusable.
		Axis.Range = GetReachableExtent(bHeight, bValid);
		if (!bValid)
		{
			// Nothing reachable along this axis here (no base, no table, or the
			// live pose is off the mechanism): an unbounded axis would accept
			// targets the setter then refuses, so offer nothing.
			UE_LOG(LogTemp, Verbose,
				TEXT("Ramms5BarLinkageController '%s': no reachable %s range; control not offered."),
				*GetName(), bHeight ? TEXT("height") : TEXT("translation"));
			return;
		}
	}
	const FVector2D Now = GetCurrentEndpoint();
	Axis.DefaultValue = FMath::Clamp(static_cast<float>(bHeight ? Now.Y : Now.X),
		static_cast<float>(Axis.Range.X), static_cast<float>(Axis.Range.Y));

	OutSurface.Add(Axis);
}

void URamms5BarLinkageController::SetContributionSuspended(bool bInSuspended)
{
	if (bSuspended == bInSuspended)
	{
		return;
	}
	bSuspended = bInSuspended;
	if (bSuspended)
	{
		// Let go of the endpoint as well as the controls: a held target would
		// fight whoever is now driving these motors by hand.
		Jog = FVector2D::ZeroVector;
		if (URammsRobotBaseComponent* Base = EnsureBase())
		{
			Base->ReleaseMotor(Resolved.ProximalMotorA);
			Base->ReleaseMotor(Resolved.ProximalMotorB);
		}
		bHasTarget = false;
	}
}

void URamms5BarLinkageController::DescribeControls(FRammsControlSurface& OutSurface) const
{
	if (bSuspended)
	{
		return;
	}

	// A 5-bar puts its endpoint anywhere in a plane, so it offers both degrees
	// of freedom rather than height alone. Both move the same two proximal
	// motors; which one a command changes depends only on which is held.
	DescribeEndpointAxis(OutSurface, /*bHeight=*/true);
	DescribeEndpointAxis(OutSurface, /*bHeight=*/false);

	// Paired only once BOTH halves are on the surface. Either can be dropped --
	// DescribeEndpointAxis offers nothing when that coordinate has no reachable
	// range here -- and an axis naming a partner that was never published sends
	// a panel hunting a control that does not exist.
	const auto FindAxis = [&OutSurface](FName Id) -> FRammsControlAxis* {
		return OutSurface.Axes.FindByPredicate([Id](const FRammsControlAxis& A) { return A.Id == Id; });
	};
	FRammsControlAxis* HeightAxis = FindAxis(HeightControlId());
	FRammsControlAxis* TranslationAxis = FindAxis(TranslationControlId());
	if (HeightAxis && TranslationAxis)
	{
		// Height is the lower Order, so it is the vertical half by the pairing
		// convention -- and the one that reads as vertical on the robot too.
		HeightAxis->PairedAxis = TranslationAxis->Id;
		TranslationAxis->PairedAxis = HeightAxis->Id;

		// The region goes on the vertical half alone, so there is one per pair
		// rather than two answers nothing keeps agreeing. Published here rather
		// than with the axis because a region describes a PAIR: with no second
		// coordinate to plot against it means nothing.
		bool			  bOutline = false;
		TArray<FVector2D> Outline = GetReachableOutline(bOutline);
		if (bOutline)
		{
			HeightAxis->RegionOutline = MoveTemp(Outline);
		}
	}

	// And a rate pair. The position axes above are the honest representation of
	// a 5-bar -- its reachable set is a curved region, and a slider per axis
	// shows only the slice at the other axis's current value. A stick that
	// commands a delta sidesteps that: it never has to claim a range. Paired
	// Continuous axes are what the surface panel renders as a joystick.
	FRammsControlAxis Up;
	Up.Id = JogUpControlId();
	Up.Group = RammsControlIds::Groups::Linkage();
	const FString Pretty = GetName().Replace(TEXT("Linkage"), TEXT("")).Replace(TEXT("_"), TEXT(" "));
	Up.DisplayName = FText::FromString(Pretty + TEXT(" jog up"));
	Up.Kind = ERammsControlKind::Continuous;
	Up.Units = ERammsControlUnits::Normalized;
	Up.Range = FVector2D(-1.0, 1.0);
	Up.bReadback = false; // a rate command; there is nothing to read back
	Up.Order = 2;		  // lower Order is the vertical axis of the joystick
	Up.PairedAxis = JogForwardControlId();
	OutSurface.Add(Up);

	FRammsControlAxis Fwd = Up;
	Fwd.Id = JogForwardControlId();
	Fwd.DisplayName = FText::FromString(Pretty + TEXT(" jog fore/aft"));
	Fwd.Order = 3;
	Fwd.PairedAxis = JogUpControlId();
	OutSurface.Add(Fwd);

	// And the one command pointing at a pad cannot express: go back. Offered
	// only when the rest pose is actually reachable, because a button that
	// always refuses is worse than no button.
	// HasBase as well as reachability: SolveTarget leaves its flag alone when
	// there is no base to check motor ranges against, so a geometrically valid
	// rest point reads as reachable on a misconfigured actor -- while
	// SetEndpointTarget refuses outright without one. That combination offers a
	// button that can only ever refuse, which is the thing this gate exists to
	// prevent.
	bool bRestReachable = false;
	SolveTarget(RestEndpoint, bRestReachable);
	if (bRestReachable && HasBase())
	{
		FRammsControlAxis Reset;
		Reset.Id = ResetControlId();
		Reset.Group = RammsControlIds::Groups::Linkage();
		Reset.DisplayName = FText::FromString(Pretty + TEXT(" reset"));
		Reset.Kind = ERammsControlKind::Action;
		Reset.bReadback = false;
		Reset.Order = 4;
		OutSurface.Add(Reset);
	}
}

bool URamms5BarLinkageController::TriggerControl(FName Id)
{
	if (bSuspended)
	{
		return false;
	}
	if (Id == ResetControlId())
	{
		// Through SetEndpointTarget like any other command, so an unreachable
		// rest pose is refused rather than half-applied.
		//
		// Zeroing Jog is not enough on its own: an input source re-sends its
		// axis every frame it is held, so a stick still deflected writes a
		// nonzero rate straight back and the next tick walks off the rest pose
		// again. Each axis is therefore suppressed until it reports neutral,
		// which is the driver letting go rather than a timer guessing when they
		// did.
		Jog = FVector2D::ZeroVector;
		bSuppressJogX = true;
		bSuppressJogY = true;
		return SetEndpointTarget(RestEndpoint);
	}
	return false;
}

bool URamms5BarLinkageController::ApplyControl(FName Id, float Value)
{
	if (bSuspended)
	{
		return false;
	}
	if (Id == HeightControlId())
	{
		return SetEndpointHeight(Value);
	}
	if (Id == TranslationControlId())
	{
		return SetEndpointTranslation(Value);
	}
	if (Id == JogUpControlId() || Id == JogForwardControlId())
	{
		const bool	bUp = Id == JogUpControlId();
		const float Clamped = FMath::Clamp(Value, -1.0f, 1.0f);
		bool&		bSuppressed = bUp ? bSuppressJogY : bSuppressJogX;

		// Held through a reset: swallow it, and keep swallowing until it comes
		// back to neutral. Accepted as soon as it does, so letting go and
		// pushing again works immediately.
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

bool URamms5BarLinkageController::ReleaseControl(FName Id)
{
	// Letting go of the stick stops the motion but keeps the endpoint held
	// where it got to -- the motors are not released.
	if (Id == JogUpControlId())
	{
		Jog.Y = 0.0f;
		return true;
	}
	if (Id == JogForwardControlId())
	{
		Jog.X = 0.0f;
		return true;
	}

	URammsRobotBaseComponent* Base = EnsureBase();
	if ((Id != HeightControlId() && Id != TranslationControlId()) || !Base)
	{
		return false;
	}
	// One pair of motors holds the endpoint, so releasing either axis lets go
	// of the whole endpoint -- there is no way to keep holding the other.
	const bool bA = Base->ReleaseMotor(Resolved.ProximalMotorA);
	const bool bB = Base->ReleaseMotor(Resolved.ProximalMotorB);
	if (bA && bB)
	{
		bHasTarget = false; // no longer holding an endpoint (LastTarget stays as history)
	}
	return bA && bB;
}

bool URamms5BarLinkageController::ReadControl(FName Id, float& OutValue) const
{
	const FVector2D Now = GetCurrentEndpoint();
	if (Id == HeightControlId())
	{
		OutValue = static_cast<float>(Now.Y);
		return true;
	}
	if (Id == TranslationControlId())
	{
		OutValue = static_cast<float>(Now.X);
		return true;
	}
	return false;
}

bool URamms5BarLinkageController::ReadTarget(FName Id, float& OutTarget) const
{
	if (!bHasTarget)
	{
		return false;
	}
	if (Id == HeightControlId())
	{
		OutTarget = static_cast<float>(LastTarget.Y);
		return true;
	}
	if (Id == TranslationControlId())
	{
		OutTarget = static_cast<float>(LastTarget.X);
		return true;
	}
	return false;
}

void URamms5BarLinkageController::GetClaimedMotorIds(TArray<FName>& OutIds) const
{
	if (bSuspended)
	{
		// Suspended: the hips show up as raw motor axes instead.
		return;
	}
	if (!Resolved.ProximalMotorA.IsNone())
	{
		OutIds.Add(Resolved.ProximalMotorA);
	}
	if (!Resolved.ProximalMotorB.IsNone())
	{
		OutIds.Add(Resolved.ProximalMotorB);
	}
}
