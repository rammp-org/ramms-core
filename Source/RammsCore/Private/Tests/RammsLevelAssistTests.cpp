// Tests for the level assist's shell: the attitude read from the corner crank pivots (RammsCurb::AttitudeFromPivots,
// which the curb controller's Measure now calls too), the controls and the status words, the plant of the Gen2 kind
// (RammsLevel::MakeLegPlant: leg heights from joints, height offsets back onto motors, fold and unfold), and the
// component itself (URammsLevelAssistComponent) on the command offset tests' rig, a take-over's hand-back to a drive
// mode's stance among it, by hand and through the curb controller's own tick.
//
// The robot is made up: four corner legs on parallelogram 4-bars and two centre legs on 5-bars, on a robot base over a
// backend with no physics that records what it is told. Where a test needs the base to answer the legs, the chassis is
// posed by hand each frame from what the motors were told (the ground's slope less the plane the legs have made): a
// kinematic stand-in, enough for signs, rates, and what is and is not written. No robot's own numbers.
//
// Beside each test is what it must catch.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Curb/RammsCurbMeasure.h"
#include "Curb/RammsCurbSeat.h"
#include "Level/RammsLevelAttitude.h"
#include "Level/RammsLevelControls.h"
#include "Level/RammsLevelLaw.h"
#include "Level/RammsLevelPlant.h"
#include "Level/RammsLevelTypes.h"
#include "Ramms4BarKinematics.h"
#include "RammsCurbNegotiationController.h"
#include "RammsDifferentialDriveController.h"
#include "RammsDriveHandOver.h"
#include "RammsDriveModeSelector.h"
#include "RammsHolonomicDriveController.h"
#include "RammsLevelAssistComponent.h"
#include "Tests/RammsCommandOffsetRig.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace RammsLevelAssistTests
{
	using RammsCommandOffsetTests::FRig;
	using RammsCommandOffsetTests::MotorRow;
	using RammsCommandOffsetTests::SameBits;

	const FName CrankFL(TEXT("crank_fl"));
	const FName CrankFR(TEXT("crank_fr"));
	const FName CrankRL(TEXT("crank_rl"));
	const FName CrankRR(TEXT("crank_rr"));
	const FName HipLA(TEXT("hip_la"));
	const FName HipLB(TEXT("hip_lb"));
	const FName HipRA(TEXT("hip_ra"));
	const FName HipRB(TEXT("hip_rb"));
	const FName SeatA(TEXT("seat_a"));
	const FName SeatB(TEXT("seat_b"));
	const FName SeatTilt(TEXT("seat_tilt"));

	/** The stance every leg is commanded to: its wheel centre this high in the chassis frame (cm). */
	constexpr double StanceZ = -16.0;
	/** The corner wheels' and the centre wheels' places (cm): fore/aft, and across. */
	constexpr double CornerX = 45.0;
	constexpr double CornerY = 25.0;
	constexpr double CentreY = 27.0;
	/** The centre wheels stand a little forward of their linkages' middle: there the two hips of a 5-bar turn by
	 *  different amounts for a height change, so a test can tell them apart. */
	constexpr double CentreX = 3.0;

	/** The made-up registry: four cranks bounded at +-0.6 rad, four unbounded hips. */
	TArray<FRammsMotorSpec> SixLegMotors()
	{
		TArray<FRammsMotorSpec> Specs;
		for (const TCHAR* Id : { TEXT("crank_fl"), TEXT("crank_fr"), TEXT("crank_rl"), TEXT("crank_rr") })
		{
			Specs.Add(MotorRow(Id, ERammsActuatorType::Position, FVector2D(-0.6, 0.6)));
		}
		for (const TCHAR* Id : { TEXT("hip_la"), TEXT("hip_lb"), TEXT("hip_ra"), TEXT("hip_rb") })
		{
			Specs.Add(MotorRow(Id, ERammsActuatorType::Position));
		}
		return Specs;
	}

	/** The registry of the same robot with a seat: its 5-bar's two motors, unbounded, and a tilt motor that turns from
	 *  -0.6 to 0.9 rad. */
	TArray<FRammsMotorSpec> SixLegAndSeatMotors()
	{
		TArray<FRammsMotorSpec> Specs = SixLegMotors();
		Specs.Add(MotorRow(TEXT("seat_a"), ERammsActuatorType::Position));
		Specs.Add(MotorRow(TEXT("seat_b"), ERammsActuatorType::Position));
		Specs.Add(MotorRow(TEXT("seat_tilt"), ERammsActuatorType::Position, FVector2D(-0.6, 0.9)));
		// (One that no offset can go on, for a seat described with it.)
		Specs.Add(MotorRow(TEXT("wheel_free"), ERammsActuatorType::Velocity));
		// (The six legs' wheels, which a curb controller stops and lets go as a crossing ends.)
		for (const TCHAR* Id : { TEXT("wheel_fl"), TEXT("wheel_fr"), TEXT("wheel_cl"), TEXT("wheel_cr"), TEXT("wheel_rl"), TEXT("wheel_rr") })
		{
			Specs.Add(MotorRow(Id, ERammsActuatorType::Velocity));
		}
		return Specs;
	}

	/** The made-up seat of the curb's seat tests (a 5-bar carrying the platform's pivot, a tilt motor at the pivot
	 *  that reads against the platform's pitch), its platform free to pitch 40 deg either way. */
	FRammsCurbSeat RigSeat()
	{
		FRammsCurbSeat		   Seat;
		FRamms5BarLinkageSpec& L = Seat.Linkage;
		L.ProximalMotorA = SeatA;
		L.ProximalMotorB = SeatB;
		L.PivotA = FVector2D(0.0, 10.0);
		L.PivotB = FVector2D(-18.0, 10.0);
		L.ProximalLengthA = 30.0f;
		L.DistalLengthA = 18.0f;
		L.ProximalLengthB = 26.0f;
		L.DistalLengthB = 26.0f;
		L.ZeroDirA = -3.0f;
		L.AngleSignA = -1.0f;
		L.ZeroDirB = 0.75f;
		L.AngleSignB = 1.0f;
		L.bElbowUpA = true;
		L.bElbowUpB = false;
		L.bFlipEndpointSide = false;
		L.TranslationSign = 1.0f;
		Seat.PivotBeyondEndpointCm = 10.0f;
		Seat.TiltMotor = SeatTilt;
		Seat.TiltAngleSign = -1.0f;
		Seat.MaxPitchDeg = 40.0f;
		return Seat;
	}

	/** A parallelogram crank-rocker (the leg of the kinematics tests) with its crank pivot at PivotX: the wheel, 25 cm
	 *  out on the rocker, is at (PivotX + 25, -16) with the crank at 0 and rises and falls with it over +-0.6 rad. */
	FRamms4BarLegSpec CornerLeg(FName Crank, double PivotX)
	{
		FRamms4BarLegSpec Spec;
		Spec.CrankMotor = Crank;
		Spec.CrankPivot = FVector2D(PivotX, 0.0);
		Spec.CrankArm = FVector2D(15.0, 0.0);
		Spec.CrankAngleSign = 1.0f;
		Spec.CouplerLength = 6.0f;
		Spec.RockerPivot = FVector2D(PivotX, -6.0);
		Spec.RockerPin = FVector2D(15.0, 0.0);
		Spec.RockerAngleSign = 1.0f;
		Spec.WheelOnRocker = FVector2D(25.0, -10.0);
		Spec.WheelOffset = FVector2D::ZeroVector;
		Spec.CrankRange = FVector2D(-0.6, 0.6);
		Spec.RockerRange = FVector2D(-1.0, 1.0);
		return Spec;
	}

	/** The command offset tests' 5-bar, lowered so that its endpoint hangs at the corner wheels' height. */
	FRamms5BarLinkageSpec CentreLinkage(FName HipA, FName HipB)
	{
		FRamms5BarLinkageSpec Spec = RammsCommandOffsetTests::ToyFiveBar();
		Spec.ProximalMotorA = HipA;
		Spec.ProximalMotorB = HipB;
		Spec.PivotA = FVector2D(5.0, 4.0);
		Spec.PivotB = FVector2D(-5.0, 4.0);
		return Spec;
	}

	FRammsCurbLeg CurbLeg(const TCHAR* Name, ERammsCurbAxle Axle, double Y)
	{
		FRammsCurbLeg Leg;
		Leg.Name = Name;
		Leg.Axle = Axle;
		Leg.Wheel.MotorId = FName(*FString::Printf(TEXT("wheel_%s"), Name));
		Leg.Wheel.Position = FVector2D(0.0, Y);
		Leg.WheelRadiusCm = 9.0f;
		return Leg;
	}

	/**
	 * The rig with the made-up six-legged robot on it: two 5-bar controllers, a curb controller that only describes
	 * the legs (it is never begun or ticked; set active by hand it stands for "the mode that drives the legs"), and,
	 * with bWithLevel, the level assist. Legs in the curb controller's order: front -Y, front +Y, centre -Y,
	 * centre +Y, rear -Y, rear +Y. With bWithSeat the curb controller describes a seat too (RigSeat), whose motors
	 * nothing commands until StandSeat.
	 */
	struct FLevelRig : FRig
	{
		URamms5BarLinkageController*	 Left = nullptr;
		URamms5BarLinkageController*	 Right = nullptr;
		URammsCurbNegotiationController* Curb = nullptr;
		URammsLevelAssistComponent*		 Level = nullptr;
		/** The slope the robot stands on (deg; pitch, roll): what it reads with no correction on. */
		FVector2D GroundDeg = FVector2D::ZeroVector;
		/** Which way the chassis faces (deg about the vertical, from +X towards +Y) and where it is (cm): a test that
		 *  turns the robot or drives it moves these before it poses the chassis. */
		double	YawDeg = 0.0;
		FVector Origin = FVector(0.0, 0.0, 30.0);

		explicit FLevelRig(bool bWithCurb = true, bool bWithLevel = true, bool bStartOn = false, bool bWithSeat = false)
			: FRig(bWithSeat ? SixLegAndSeatMotors() : SixLegMotors())
		{
			if (Actor == nullptr || Base == nullptr)
			{
				return;
			}
			Left = MakeLinkage(TEXT("LeftLinkage"), HipLA, HipLB);
			Right = MakeLinkage(TEXT("RightLinkage"), HipRA, HipRB);
			if (bWithCurb)
			{
				Curb = NewObject<URammsCurbNegotiationController>(Actor, TEXT("Curb"));
				auto Corner = [this](const TCHAR* Name, ERammsCurbAxle Axle, double Y, FName Crank) {
					FRammsCurbLeg Leg = CurbLeg(Name, Axle, Y);
					Leg.Kind = ERammsCurbLegKind::FourBar;
					Leg.FourBar = CornerLeg(Crank, Axle == ERammsCurbAxle::Front ? CornerX - 25.0 : -CornerX - 25.0);
					Curb->Legs.Add(Leg);
				};
				auto Centre = [this](const TCHAR* Name, double Y, const TCHAR* Linkage) {
					FRammsCurbLeg Leg = CurbLeg(Name, ERammsCurbAxle::Center, Y);
					Leg.Kind = ERammsCurbLegKind::FiveBar;
					Leg.FiveBarComponent = Linkage;
					Curb->Legs.Add(Leg);
				};
				Corner(TEXT("fl"), ERammsCurbAxle::Front, -CornerY, CrankFL);
				Corner(TEXT("fr"), ERammsCurbAxle::Front, CornerY, CrankFR);
				Centre(TEXT("cl"), -CentreY, TEXT("LeftLinkage"));
				Centre(TEXT("cr"), CentreY, TEXT("RightLinkage"));
				Corner(TEXT("rl"), ERammsCurbAxle::Rear, -CornerY, CrankRL);
				Corner(TEXT("rr"), ERammsCurbAxle::Rear, CornerY, CrankRR);
				if (bWithSeat)
				{
					Curb->Seat = RigSeat();
				}
				Curb->Tuning.LegSpeed = 12.0f;
				Curb->Tuning.ChassisHalfExtent = FVector2D(40.0, 20.0);
				Curb->Tuning.ChassisUnderside = -2.0f;
				// As its BeginPlay leaves it (it is never begun here): the group the level assist may tick after.
				Curb->PrimaryComponentTick.TickGroup = TG_PrePhysics;
			}
			if (bWithLevel)
			{
				Level = NewObject<URammsLevelAssistComponent>(Actor, TEXT("LevelAssist"));
				Level->bEnabledAtStart = bStartOn;
				BeginPlayOf(*Level);
			}
			Stand();
		}

		URamms5BarLinkageController* MakeLinkage(const TCHAR* Name, FName HipA, FName HipB)
		{
			URamms5BarLinkageController* Linkage = NewObject<URamms5BarLinkageController>(Actor, Name);
			Linkage->Linkage = CentreLinkage(HipA, HipB);
			Linkage->HeightScanLimits = FVector2D(-30.0, 30.0);
			Linkage->TranslationScanLimits = FVector2D(-12.0, 12.0);
			BeginPlayOf(*Linkage);
			return Linkage;
		}

		bool IsReady() const
		{
			return FRig::IsReady() && Left != nullptr && Right != nullptr && Left->HasBegunPlay() && Right->HasBegunPlay();
		}

		/** Every leg commanded to the stance, as a drive mode would, and told. */
		void Stand()
		{
			if (!IsReady())
			{
				return;
			}
			for (const FName& Crank : { CrankFL, CrankFR, CrankRL, CrankRR })
			{
				Base->SetMotorCommand(Crank, 0.0f);
			}
			Left->SetEndpointTarget(FVector2D(CentreX, StanceZ));
			Right->SetEndpointTarget(FVector2D(CentreX, StanceZ));
			Tick();
			PoseChassis(GroundDeg);
		}

		/** The seat's three motors commanded, as its controller or a drive stance would, and told. */
		void StandSeat(float A, float B, float Tilt) const
		{
			Base->SetMotorCommand(SeatA, A);
			Base->SetMotorCommand(SeatB, B);
			Base->SetMotorCommand(SeatTilt, Tilt);
			Tick();
		}

		/** The six legs' motors, in the legs' order (a 5-bar's two hips together). */
		static TArray<FName> LegMotors() { return { CrankFL, CrankFR, HipLA, HipLB, HipRA, HipRB, CrankRL, CrankRR }; }

		/** A leg's wheel-centre height (cm) at what its motors are told (bEffective) or at their baselines. */
		double LegZ(int32 Leg, bool bEffective) const
		{
			auto Read = [this, bEffective](FName Id) { return bEffective ? Effective(Id) : Baseline(Id); };
			bool bValid = false;
			if (Leg == 2 || Leg == 3)
			{
				const FRamms5BarLinkageSpec Spec = Leg == 2 ? CentreLinkage(HipLA, HipLB) : CentreLinkage(HipRA, HipRB);
				return URamms5BarKinematics::ComputeEndpoint(Spec, FVector2D(Read(Spec.ProximalMotorA), Read(Spec.ProximalMotorB)), bValid).Y;
			}
			const FName Crank = Leg == 0 ? CrankFL : Leg == 1 ? CrankFR
				: Leg == 4									  ? CrankRL
															  : CrankRR;
			return URamms4BarKinematics::ComputeWheelCenter(CornerLeg(Crank, 0.0), Read(Crank), bValid).Y;
		}

		/** How far each leg's wheel is retracted past its baseline by what its motors are told (cm). */
		double LegDz(int32 Leg) const { return LegZ(Leg, true) - LegZ(Leg, false); }

		/** The plane the corner legs have made (deg; pitch, roll), from what their motors are told. */
		FVector2D CorrectionDeg() const
		{
			const double Pitch = FMath::Atan(((LegDz(0) + LegDz(1)) - (LegDz(4) + LegDz(5))) / 2.0 / (2.0 * CornerX));
			const double Roll = FMath::Atan(((LegDz(1) + LegDz(5)) - (LegDz(0) + LegDz(4))) / 2.0 / (2.0 * CornerY));
			return FVector2D(FMath::RadiansToDegrees(Pitch), FMath::RadiansToDegrees(Roll));
		}

		/** Put the corner crank pivots in the world where a chassis at Deg (pitch nose up, roll +Y up) has them. */
		void PoseChassis(const FVector2D& Deg) const
		{
			const double  P = FMath::DegreesToRadians(Deg.X);
			const double  R = FMath::DegreesToRadians(Deg.Y);
			const double  Y0 = FMath::DegreesToRadians(YawDeg);
			const FVector Forward(FMath::Cos(P) * FMath::Cos(Y0), FMath::Cos(P) * FMath::Sin(Y0), FMath::Sin(P));
			const FVector Side(-FMath::Cos(R) * FMath::Sin(Y0), FMath::Cos(R) * FMath::Cos(Y0), FMath::Sin(R));
			auto		  Place = [&](FName Crank, double X, double Y) {
				 Log->Transforms.Add(Crank, FTransform(Origin + Forward * X + Side * Y));
			};
			Place(CrankFL, CornerX - 25.0, -CornerY);
			Place(CrankFR, CornerX - 25.0, CornerY);
			Place(CrankRL, -CornerX - 25.0, -CornerY);
			Place(CrankRR, -CornerX - 25.0, CornerY);
		}

		/** One frame: the level assist's tick, the base's, and the chassis answering what the legs were told. */
		void Frame(float Dt = 1.0f / 30.0f) const
		{
			if (Level != nullptr)
			{
				Level->TickComponent(Dt, LEVELTICK_All, nullptr);
			}
			Tick(Dt);
			PoseChassis(GroundDeg - CorrectionDeg());
		}

		void Frames(int32 Count, float Dt = 1.0f / 30.0f) const
		{
			for (int32 Index = 0; Index < Count; ++Index)
			{
				Frame(Dt);
			}
		}

		/** How many commands the leg motors have been given in all. */
		int32 LegPushes() const
		{
			int32 Count = 0;
			for (const FName& Id : LegMotors())
			{
				Count += Log->CountFor(Id);
			}
			return Count;
		}

		float Control(FName Id) const
		{
			float							Value = NAN;
			const IRammsControlContributor* Contributor = Level;
			return Contributor->ReadControl(Id, Value) ? Value : NAN;
		}

		RammsLevel::EState State() const { return static_cast<RammsLevel::EState>(FMath::RoundToInt(Control(RammsLevel::StateControlId()))); }

		FString Status() const
		{
			FText Words;
			static_cast<const IRammsControlContributor*>(Level)->ReadControlText(RammsLevel::StatusControlId(), Words);
			return Words.ToString();
		}

		/** Whether Of's tick has been made to wait for Before's. */
		static bool TicksAfter(const UActorComponent* Of, const UActorComponent* Before)
		{
			return Of->PrimaryComponentTick.GetPrerequisites().ContainsByPredicate([Before](const FTickPrerequisite& P) { return P.PrerequisiteObject.Get() == Before; });
		}

		/** The group Of's tick really starts in, by the engine's rule: the latest among its own and its prerequisites'. */
		static ETickingGroup StartGroup(const UActorComponent* Of)
		{
			ETickingGroup Group = Of->PrimaryComponentTick.TickGroup;
			for (const FTickPrerequisite& P : Of->PrimaryComponentTick.GetPrerequisites())
			{
				if (const UActorComponent* Before = Cast<UActorComponent>(P.PrerequisiteObject.Get()))
				{
					Group = FMath::Max(Group, StartGroup(Before));
				}
			}
			return Group;
		}
	};

	/** The attitude as URammsCurbNegotiationController::Measure worked it out before RammsCurb::AttitudeFromPivots
	 *  was lifted out of it, line for line: the reference the lifted function must match to the bit. */
	void AttitudeAsTheCurbMeasured(TConstArrayView<RammsCurb::FPivotSample> Pivots, double& OutPitch, double& OutRoll, FVector& OutForward,
		FVector& OutSide)
	{
		auto	  Horizontal = [](const FVector& V) { return FVector(V.X, V.Y, 0.0); };
		FVector	  FrontW = FVector::ZeroVector, RearW = FVector::ZeroVector;
		FVector	  PlusW = FVector::ZeroVector, MinusW = FVector::ZeroVector;
		FVector2D FrontL = FVector2D::ZeroVector, RearL = FVector2D::ZeroVector;
		int32	  NFront = 0, NRear = 0, NPlus = 0, NMinus = 0;
		for (const RammsCurb::FPivotSample& Pivot : Pivots)
		{
			const FVector P = Pivot.World;
			if (Pivot.bFront)
			{
				FrontW += P;
				FrontL += Pivot.Local;
				++NFront;
			}
			else
			{
				RearW += P;
				RearL += Pivot.Local;
				++NRear;
			}
			if (Pivot.bPlusY)
			{
				PlusW += P;
				++NPlus;
			}
			else
			{
				MinusW += P;
				++NMinus;
			}
		}
		FrontW /= NFront;
		RearW /= NRear;
		FrontL /= NFront;
		RearL /= NRear;
		const FVector Along = FrontW - RearW;
		OutPitch = FMath::Atan2(Along.Z, Horizontal(Along).Size()) - FMath::Atan2(FrontL.Y - RearL.Y, FrontL.X - RearL.X);
		OutForward = Horizontal(Along).GetSafeNormal();
		OutSide = FVector::ZeroVector;
		OutRoll = 0.0;
		if (NPlus > 0 && NMinus > 0)
		{
			const FVector Across = PlusW / NPlus - MinusW / NMinus;
			OutRoll = FMath::Atan2(Across.Z, Horizontal(Across).Size());
			OutSide = Horizontal(Across).GetSafeNormal();
		}
	}

	/** Four corner pivots of a made-up chassis (front pivots 3 cm higher on it than the rear ones) at PitchDeg and
	 *  RollDeg, turned YawDeg about the vertical, its origin at Origin. */
	TArray<RammsCurb::FPivotSample> PosedPivots(double PitchDeg, double RollDeg, double YawDeg, const FVector& Origin)
	{
		const double  P = FMath::DegreesToRadians(PitchDeg);
		const double  R = FMath::DegreesToRadians(RollDeg);
		const double  Y = FMath::DegreesToRadians(YawDeg);
		const FVector Heading(FMath::Cos(Y), FMath::Sin(Y), 0.0);
		const FVector Across(-FMath::Sin(Y), FMath::Cos(Y), 0.0);
		// The chassis' axes: forward raised by the pitch, +Y raised by the roll, up square to both.
		const FVector					Forward = Heading * FMath::Cos(P) + FVector::UpVector * FMath::Sin(P);
		const FVector					Side = Across * FMath::Cos(R) + FVector::UpVector * FMath::Sin(R);
		const FVector					Up = FVector::CrossProduct(Forward, Side).GetSafeNormal();
		TArray<RammsCurb::FPivotSample> Pivots;
		for (const bool bFront : { true, false })
		{
			for (const bool bPlusY : { false, true })
			{
				RammsCurb::FPivotSample& Pivot = Pivots.AddDefaulted_GetRef();
				Pivot.Local = bFront ? FVector2D(38.0, 5.0) : FVector2D(-42.0, 2.0);
				Pivot.WheelY = bPlusY ? 26.0 : -26.0;
				Pivot.bFront = bFront;
				Pivot.bPlusY = bPlusY;
				Pivot.World = Origin + Forward * Pivot.Local.X + Side * Pivot.WheelY + Up * Pivot.Local.Y;
			}
		}
		return Pivots;
	}
} // namespace RammsLevelAssistTests

// Catches: the lifted attitude function differing from what the curb controller measured by (a crossing's numbers
// would move), a roll or pitch of the wrong sign, and the slope the pivots have on the chassis itself not taken off.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAttitudePivotMatchesCurbMeasure,
	"Ramms.Level.Attitude.PivotMatchesCurbMeasure",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAttitudePivotMatchesCurbMeasure::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	struct FPose
	{
		double Pitch, Roll, Yaw;
	};
	for (const FPose& Pose : { FPose{ 0.0, 0.0, 0.0 }, FPose{ 4.0, 0.0, 0.0 }, FPose{ 0.0, 3.0, 0.0 }, FPose{ -6.0, 2.5, 37.0 }, FPose{ 5.0, -4.0, -120.0 } })
	{
		const FString						  Case = FString::Printf(TEXT("pitch %.1f roll %.1f yaw %.0f"), Pose.Pitch, Pose.Roll, Pose.Yaw);
		const TArray<RammsCurb::FPivotSample> Pivots = PosedPivots(Pose.Pitch, Pose.Roll, Pose.Yaw, FVector(310.0, -45.0, 62.0));
		RammsCurb::FPivotAttitude			  Got;
		if (!TestTrue(*(Case + TEXT(": read")), RammsCurb::AttitudeFromPivots(Pivots, Got)))
		{
			return false;
		}
		// To the bit, what the curb controller worked out before.
		double	Pitch = 0.0, Roll = 0.0;
		FVector Forward, Side;
		AttitudeAsTheCurbMeasured(Pivots, Pitch, Roll, Forward, Side);
		TestTrue(*(Case + TEXT(": the pitch is the curb's, to the bit")), Got.PitchRad == Pitch);
		TestTrue(*(Case + TEXT(": the roll is the curb's, to the bit")), Got.RollRad == Roll);
		TestTrue(*(Case + TEXT(": the heading and the side are the curb's")), Got.Forward == Forward && Got.Side == Side);
		// And the attitude the chassis was posed at: nose up positive, +Y side up positive, the front pivots' own
		// 3 cm of height on the chassis not read as pitch.
		TestEqual(*(Case + TEXT(": the pitch it was posed at")), FMath::RadiansToDegrees(Got.PitchRad), Pose.Pitch, 0.02);
		TestEqual(*(Case + TEXT(": the roll it was posed at")), FMath::RadiansToDegrees(Got.RollRad), Pose.Roll, 0.02);
		TestEqual(*(Case + TEXT(": four pivots")), Got.Count, 4);
		TestEqual(*(Case + TEXT(": their mean place on the chassis")), Got.MeanLocal.X, -2.0, 1.0e-9);
	}
	// Front pivots only: no pitch to be had, and it says so rather than reading 0.
	TArray<RammsCurb::FPivotSample> FrontOnly = PosedPivots(2.0, 1.0, 0.0, FVector::ZeroVector);
	FrontOnly.SetNum(2);
	RammsCurb::FPivotAttitude Unread;
	Unread.PitchRad = 7.0;
	TestFalse(TEXT("with no rear pivot there is no attitude"), RammsCurb::AttitudeFromPivots(FrontOnly, Unread));
	TestEqual(TEXT("... and what was there is untouched"), Unread.PitchRad, 7.0);
	return true;
}

// Catches: the reflected tuning and the law's plain copy drifting apart (a default changed in one; a member left out of
// the copy, or copied into another: each is given a value of its own and must arrive), and the rider's preference not
// reaching the level reference, or reaching it past its bounds.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelControlsTuningMirrors,
	"Ramms.Level.Controls.TuningMirrorsAndPreference",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelControlsTuningMirrors::RunTest(const FString& Parameters)
{
	using namespace RammsLevel;
	// Every number of the tuning but the two attitudes (below), once, for both checks.
	// clang-format off
	#define RAMMS_LEVEL_NUMBERS(X) \
		X(FeedForwardGain) X(ProportionalGain) X(IntegralGain) X(IntegralMaxDeg) \
		X(DeadbandDeg) X(AttitudeFilterSeconds) X(TerrainFilterSeconds) X(MaxCorrectionRateDegPerSec) \
		X(LegRateFraction) X(MaxTiltDeg) X(TiltRateDegPerSec) X(TiltAccelDegPerSec2) \
		X(LegRangeMarginCm) X(SoftZoneCm) X(SupportBandInCm) X(SupportBandOutCm) \
		X(MinLiftedClearanceCm) X(MinBellyClearanceCm) X(RideHeightUpCm) X(RideHeightDownCm) \
		X(RideHeightRateCmPerSec) X(MinSpeedScale) X(SpeedCapErrorDeg) X(SpeedCapErrorFullDeg) \
		X(SpeedScaleRisePerSec) X(SettleErrorDeg) X(SettleSeconds) X(WakeErrorDeg) \
		X(FaultTiltDeg) X(FaultSeconds) X(MeasurementLossSeconds) X(SeatTrimMaxDeg) \
		X(SeatTrimRateDegPerSec) X(SeatFilterSeconds)
	// clang-format on

	// The defaults: the reflected tuning's are the law's own.
	const FTuning Plain;
	const FTuning Made = MakeTuning(FRammsLevelTuning(), FVector2D::ZeroVector);
	#define RAMMS_LEVEL_SAME(Member) TestEqual(TEXT("default " #Member), static_cast<double>(Made.Member), static_cast<double>(Plain.Member), 1.0e-6);
	RAMMS_LEVEL_NUMBERS(RAMMS_LEVEL_SAME)
	#undef RAMMS_LEVEL_SAME
	TestTrue(TEXT("default LevelReferenceDeg"), Made.LevelReferenceDeg.Equals(Plain.LevelReferenceDeg, 1.0e-9));
	TestTrue(TEXT("default MaxCorrectionDeg"), Made.MaxCorrectionDeg.Equals(Plain.MaxCorrectionDeg, 1.0e-9));
	TestEqual(TEXT("default bSeatCoversTransients"), Made.bSeatCoversTransients, Plain.bSeatCoversTransients);
	// The plain copy has no member this list does not name: a member added to it must be added here and to the copy.
	TestEqual(TEXT("the plain tuning's size (a member added: list it above and in MakeTuning)"), static_cast<int32>(sizeof(FTuning)), 39 * 8);

	// The copy: every member given a value no other has (eighths, exact in a float), and each must arrive as itself.
	// A member the copy leaves out arrives as its default, and two crossed over arrive as each other.
	FRammsLevelTuning Odd;
	float			  Next = 100.0f;
	#define RAMMS_LEVEL_SET(Member) Odd.Member = (Next += 0.125f);
	RAMMS_LEVEL_NUMBERS(RAMMS_LEVEL_SET)
	#undef RAMMS_LEVEL_SET
	Odd.LevelReferenceDeg = FVector2D(200.125, 200.25);
	Odd.MaxCorrectionDeg = FVector2D(200.375, 200.5);
	Odd.bSeatCoversTransients = !Plain.bSeatCoversTransients;
	const FTuning Copied = MakeTuning(Odd, FVector2D::ZeroVector);
	#define RAMMS_LEVEL_COPIED(Member) TestEqual(TEXT("copied " #Member), static_cast<double>(Copied.Member), static_cast<double>(Odd.Member));
	RAMMS_LEVEL_NUMBERS(RAMMS_LEVEL_COPIED)
	#undef RAMMS_LEVEL_COPIED
	#undef RAMMS_LEVEL_NUMBERS
	TestTrue(TEXT("copied LevelReferenceDeg"), Copied.LevelReferenceDeg == Odd.LevelReferenceDeg);
	TestTrue(TEXT("copied MaxCorrectionDeg"), Copied.MaxCorrectionDeg == Odd.MaxCorrectionDeg);
	TestEqual(TEXT("copied bSeatCoversTransients"), Copied.bSeatCoversTransients, Odd.bSeatCoversTransients);

	// The preference is added to the reference, each axis within its own bound, either way.
	FRammsLevelTuning Tuning;
	Tuning.LevelReferenceDeg = FVector2D(1.0, -0.5);
	Tuning.MaxPreferenceDeg = FVector2D(8.0, 5.0);
	TestTrue(TEXT("a preference is added to the reference"), MakeTuning(Tuning, FVector2D(2.0, -1.5)).LevelReferenceDeg.Equals(FVector2D(3.0, -2.0), 1.0e-9));
	TestTrue(TEXT("past its bounds it stops at them"), MakeTuning(Tuning, FVector2D(20.0, -20.0)).LevelReferenceDeg.Equals(FVector2D(9.0, -5.5), 1.0e-9));
	TestTrue(TEXT("one that is no number counts as none"), MakeTuning(Tuning, FVector2D(NAN, 1.0)).LevelReferenceDeg.Equals(FVector2D(1.0, 0.5), 1.0e-9));
	return true;
}

// Catches: a status with no words or the wrong weight (a fault shown as information), the slope's size missing from the
// past-reach line, and a robot that cannot be levelled reading as merely off.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelControlsStatusWords,
	"Ramms.Level.Controls.StatusWords",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelControlsStatusWords::RunTest(const FString& Parameters)
{
	using namespace RammsLevel;
	using ESeverity = ERammsControlTextSeverity;
	auto Line = [](EStatus Status, ESeverity& OutSeverity, double OffDeg = 0.0) {
		FStatusFacts Facts;
		Facts.Status = Status;
		Facts.OffReferenceDeg = OffDeg;
		Facts.bOn = Status != EStatus::Off;
		return StatusLine(Facts, OutSeverity);
	};
	ESeverity Severity = ESeverity::Fault;
	TestEqual(TEXT("off"), Line(EStatus::Off, Severity), FString(TEXT("Off")));
	TestTrue(TEXT("off is information"), Severity == ESeverity::Info);
	for (const EStatus Status : { EStatus::Levelling, EStatus::Settled, EStatus::Holding, EStatus::Releasing, EStatus::Yielded, EStatus::Waiting })
	{
		const FString Words = Line(Status, Severity);
		TestTrue(*FString::Printf(TEXT("status %d has words of its own"), static_cast<int32>(Status)), !Words.IsEmpty() && Words != TEXT("Off"));
		TestTrue(*FString::Printf(TEXT("status %d is information"), static_cast<int32>(Status)), Severity == ESeverity::Info);
	}
	TestTrue(TEXT("at reach asks for attention"), !Line(EStatus::AtReach, Severity).IsEmpty() && Severity == ESeverity::Attention);
	{
		// Waiting says what for: a mode it does not level in first, then a stance still being taken, else the tilt.
		FStatusFacts Facts;
		Facts.bOn = true;
		Facts.Status = EStatus::Waiting;
		TestEqual(TEXT("waiting for the tilt"), StatusLine(Facts, Severity), FString(TEXT("Waiting: the tilt cannot be read yet")));
		Facts.bStanceChanging = true;
		TestEqual(TEXT("waiting for the drive mode's stance"), StatusLine(Facts, Severity), FString(TEXT("Waiting: the drive mode is taking its stance")));
		Facts.bModeNotLevelled = true;
		TestEqual(TEXT("waiting in a mode it does not level in"), StatusLine(Facts, Severity), FString(TEXT("Waiting: not levelling in this drive mode")));
	}
	{
		// At reach, and past it, say what stops the legs, each kind in its own
		// words; and at reach says how far off level the base is held once
		// that is a degree.
		TSet<FString> Words;
		for (const EReachLimit Limit : { EReachLimit::LegTravel, EReachLimit::LiftedClearance, EReachLimit::Underside, EReachLimit::AxisLimit })
		{
			FStatusFacts Facts;
			Facts.bOn = true;
			Facts.Status = EStatus::AtReach;
			Facts.ReachLimit = Limit;
			Facts.OffReferenceDeg = 0.4;
			const FString Near = StatusLine(Facts, Severity);
			const FString Why = ReachLimitWords(Limit);
			TestTrue(*FString::Printf(TEXT("limit %d has words"), static_cast<int32>(Limit)), !Why.IsEmpty());
			Words.Add(Why);
			TestEqual(*FString::Printf(TEXT("at reach, limit %d: what stops the legs"), static_cast<int32>(Limit)), Near, FString::Printf(TEXT("At the end of levelling reach: %s"), *Why));
			Facts.OffReferenceDeg = 1.8;
			TestEqual(*FString::Printf(TEXT("at reach, limit %d: and how far off the base is held"), static_cast<int32>(Limit)), StatusLine(Facts, Severity),
				FString::Printf(TEXT("At the end of levelling reach, 2 deg off: %s"), *Why));
			Facts.Status = EStatus::PastReach;
			Facts.OffReferenceDeg = 7.4;
			TestEqual(*FString::Printf(TEXT("past reach, limit %d"), static_cast<int32>(Limit)), StatusLine(Facts, Severity), FString::Printf(TEXT("Slope past levelling reach: 7 deg (%s)"), *Why));
		}
		TestEqual(TEXT("each kind of limit in words of its own"), Words.Num(), 4);
		TestEqual(TEXT("the words for a raised wheel"), FString(ReachLimitWords(EReachLimit::LiftedClearance)), FString(TEXT("a raised wheel would come too near the ground")));
		TestEqual(TEXT("no limit: no words"), FString(ReachLimitWords(EReachLimit::None)), FString());
		TestEqual(TEXT("at reach with no limit named"), Line(EStatus::AtReach, Severity, 0.2), FString(TEXT("At the end of levelling reach")));
	}
	TestEqual(TEXT("past reach names the tilt"), Line(EStatus::PastReach, Severity, 7.4), FString(TEXT("Slope past levelling reach: 7 deg")));
	TestTrue(TEXT("... and asks for attention"), Severity == ESeverity::Attention);
	TestTrue(TEXT("a tilt past the fault line names it"), Line(EStatus::TiltFault, Severity, 13.2).Contains(TEXT("13 deg")));
	TestTrue(TEXT("... as a fault"), Severity == ESeverity::Fault);
	TestTrue(TEXT("the attitude lost asks for attention"), !Line(EStatus::AttitudeLost, Severity).IsEmpty() && Severity == ESeverity::Attention);
	TestTrue(TEXT("a divergence says how to try again"), Line(EStatus::Diverged, Severity).Contains(TEXT("off and on")));
	TestTrue(TEXT("... as a fault"), Severity == ESeverity::Fault);
	// A robot that cannot be levelled says why, whatever the law's status.
	FStatusFacts Cannot;
	Cannot.CannotLevel = TEXT("no robot description");
	TestEqual(TEXT("no description"), StatusLine(Cannot, Severity), FString(TEXT("Level: no robot description")));
	TestTrue(TEXT("... as a fault"), Severity == ESeverity::Fault);
	return true;
}

// Catches: a leg's place, height, travel or rate read wrongly from the robot's description; the baseline height taken
// from the joint (or the measured one from the command); a levelled motor left out of the claim.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLegPlantMeasuresLegs,
	"Ramms.Level.LegPlant.MeasuresLegs",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLegPlantMeasuresLegs::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/false);
	if (!TestTrue(TEXT("the made-up robot on the recording backend"), R.IsReady()))
	{
		return false;
	}
	TUniquePtr<RammsLevel::IPlant> Plant = RammsLevel::MakeLegPlant(*R.Base, *R.Curb, R.Layer());
	FString						   Why;
	if (!TestTrue(TEXT("the robot's description resolves"), Plant->Resolve(Why)))
	{
		AddError(Why);
		return false;
	}
	TArray<FName> Ids;
	Plant->LevelledMotorIds(Ids);
	TestEqual(TEXT("four cranks and four hips are levelled"), Ids.Num(), 8);
	for (const FName& Id : FLevelRig::LegMotors())
	{
		TestTrue(*FString::Printf(TEXT("%s is among them"), *Id.ToString()), Ids.Contains(Id));
	}
	TestFalse(TEXT("the robot has no seat tilt"), Plant->HasSeat());

	RammsLevel::FPlantLegs Legs;
	if (!TestTrue(TEXT("the legs measure"), Plant->MeasureLegs(Legs)) || !TestEqual(TEXT("six legs"), Legs.Num(), 6))
	{
		return false;
	}
	const FVector2D Places[6] = { FVector2D(CornerX, -CornerY), FVector2D(CornerX, CornerY), FVector2D(CentreX, -CentreY), FVector2D(CentreX, CentreY),
		FVector2D(-CornerX, -CornerY), FVector2D(-CornerX, CornerY) };
	for (int32 Index = 0; Index < 6; ++Index)
	{
		const RammsLevel::FLegReach& Leg = Legs[Index];
		const FString				 Case = FString::Printf(TEXT("leg %d"), Index);
		TestTrue(*(Case + TEXT(": where its wheel is")), Leg.XY.Equals(Places[Index], 0.05));
		TestEqual(*(Case + TEXT(": its baseline height is the stance's")), Leg.BaseZ, StanceZ, 0.02);
		TestEqual(*(Case + TEXT(": its joints are at their commands")), Leg.MeasuredZ, Leg.BaseZ, 0.02);
		TestEqual(*(Case + TEXT(": its wheel's radius")), Leg.RadiusCm, 9.0, 1.0e-6);
		TestTrue(*(Case + TEXT(": it can extend and retract from the stance")), Leg.MinZ < StanceZ - 3.0 && Leg.MaxZ > StanceZ + 3.0);
		TestEqual(*(Case + TEXT(": it moves at the curb's leg speed, no motor's row limiting it")), Leg.MaxRateCmS, 12.0, 1.0e-6);
		TestTrue(*(Case + TEXT(": roll reaches it")), Leg.bRoll);
	}
	AddInfo(FString::Printf(TEXT("travel: corner legs %.2f..%.2f cm, centre legs %.2f..%.2f cm"), Legs[0].MinZ, Legs[0].MaxZ, Legs[2].MinZ, Legs[2].MaxZ));
	// A 4-bar's travel is its crank range's, a 5-bar's what its linkage reaches where it stands.
	const FVector2D CornerRange = URamms4BarKinematics::GetHeightRange(CornerLeg(CrankFL, 20.0));
	TestEqual(TEXT("a corner leg's lowest reach"), Legs[0].MinZ, CornerRange.X, 1.0e-6);
	TestEqual(TEXT("a corner leg's highest reach"), Legs[0].MaxZ, CornerRange.Y, 1.0e-6);
	bool bReach = false;
	R.Left->SolveTarget(FVector2D(CentreX, Legs[2].MinZ), bReach);
	TestTrue(TEXT("a centre leg's lowest reach is reachable"), bReach);
	R.Left->SolveTarget(FVector2D(CentreX, Legs[2].MinZ - 0.5), bReach);
	TestFalse(TEXT("... and half a centimetre past it is not"), bReach);

	// A joint short of its command: the measured height moves, the baseline height does not.
	R.Log->Joints.Add(CrankFL, 0.1f);
	bool		 bValid = false;
	const double Sagged = URamms4BarKinematics::ComputeWheelCenter(CornerLeg(CrankFL, 20.0), 0.1f, bValid).Y;
	TestTrue(TEXT("measured again"), Plant->MeasureLegs(Legs));
	TestEqual(TEXT("the measured height follows the joint"), Legs[0].MeasuredZ, Sagged, 1.0e-4);
	TestEqual(TEXT("the baseline height stays the command's"), Legs[0].BaseZ, StanceZ, 0.02);
	TestTrue(TEXT("(the joint really is somewhere else)"), FMath::Abs(Sagged - StanceZ) > 1.0);
	R.Log->Joints.Remove(CrankFL);

	// The belly plate's corners, each asking the tuning's clearance.
	RammsLevel::FTuning Tuning;
	Tuning.MinBellyClearanceCm = 3.5;
	RammsLevel::FPlantPoints Points;
	Plant->ClearancePoints(Tuning, Points);
	if (TestEqual(TEXT("four belly points"), Points.Num(), 4))
	{
		for (const RammsLevel::FClearancePoint& Point : Points)
		{
			TestTrue(TEXT("a corner of the belly plate"), FMath::IsNearlyEqual(FMath::Abs(Point.XY.X), 40.0) && FMath::IsNearlyEqual(FMath::Abs(Point.XY.Y), 20.0));
			TestEqual(TEXT("its underside"), Point.UndersideZ, -2.0, 1.0e-6);
			TestEqual(TEXT("the clearance asked"), Point.MinCm, 3.5, 1.0e-9);
		}
	}

	// A motor whose row limits its speed limits its leg's rate, by how far the wheel rises per radian there.
	{
		TArray<FRammsMotorSpec> Specs = SixLegMotors();
		Specs[0].MaxVelocity = 0.2f;
		FRig Slow(Specs);
		if (TestTrue(TEXT("a robot with a slow crank"), Slow.IsReady()))
		{
			Slow.Base->SetMotorCommand(CrankFL, 0.0f);
			URammsCurbNegotiationController* Curb = NewObject<URammsCurbNegotiationController>(Slow.Actor, TEXT("Curb"));
			FRammsCurbLeg					 Leg = CurbLeg(TEXT("fl"), ERammsCurbAxle::Front, -CornerY);
			Leg.FourBar = CornerLeg(CrankFL, 20.0);
			Curb->Legs.Add(Leg);
			Curb->Tuning.LegSpeed = 12.0f;
			TUniquePtr<RammsLevel::IPlant> SlowPlant = RammsLevel::MakeLegPlant(*Slow.Base, *Curb, Slow.Layer());
			RammsLevel::FPlantLegs		   SlowLegs;
			if (TestTrue(TEXT("resolves and measures"), SlowPlant->Resolve(Why) && SlowPlant->MeasureLegs(SlowLegs)) && SlowLegs.Num() == 1)
			{
				bool		 bA = false, bB = false;
				const double PerRad = (URamms4BarKinematics::ComputeWheelCenter(Leg.FourBar, 0.01f, bB).Y - URamms4BarKinematics::ComputeWheelCenter(Leg.FourBar, -0.01f, bA).Y) / 0.02;
				TestEqual(TEXT("its leg's rate is the crank's times the wheel's rise per radian"), SlowLegs[0].MaxRateCmS, 0.2 * FMath::Abs(PerRad), 0.01);
				TestTrue(TEXT("... which is under the curb's leg speed"), SlowLegs[0].MaxRateCmS < 12.0 && SlowLegs[0].MaxRateCmS > 1.0);
			}
		}
	}

	// A description the robot does not match is refused, in words.
	{
		FLevelRig Other(/*bWithCurb=*/true, /*bWithLevel=*/false);
		Other.Curb->Legs[2].FiveBarComponent = TEXT("NoSuchLinkage");
		TUniquePtr<RammsLevel::IPlant> Bad = RammsLevel::MakeLegPlant(*Other.Base, *Other.Curb, Other.Layer());
		TestFalse(TEXT("a leg naming a linkage the robot lacks is refused"), Bad->Resolve(Why));
		TestTrue(TEXT("... saying which"), Why.Contains(TEXT("NoSuchLinkage")));
		Other.Curb->Legs[2].FiveBarComponent = TEXT("LeftLinkage");
		Other.Curb->Legs[0].FourBar.CrankMotor = TEXT("wheel_fl");
		TestFalse(TEXT("a crank the motor table lacks is refused"), Bad->Resolve(Why));
		TestTrue(TEXT("... saying which"), Why.Contains(TEXT("wheel_fl")));
	}
	return true;
}

// Catches: an offset that does not make the height asked (wrong sign, wrong kinematics, the 5-bar's hips swapped), a
// zero offset that is not exactly zero, an offset written through the 5-bar controller (which would move the target a
// stance ramp and a jog start from), and a baseline moved by an offset.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLegPlantFiveBarOffsetLeavesLastTarget,
	"Ramms.Level.LegPlant.FiveBarOffsetLeavesLastTarget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLegPlantFiveBarOffsetLeavesLastTarget::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/false);
	if (!TestTrue(TEXT("the made-up robot on the recording backend"), R.IsReady()))
	{
		return false;
	}
	TUniquePtr<RammsLevel::IPlant> Plant = RammsLevel::MakeLegPlant(*R.Base, *R.Curb, R.Layer());
	FString						   Why;
	RammsLevel::FPlantLegs		   Legs;
	TArray<FName>				   Ids;
	if (!TestTrue(TEXT("resolved, measured"), Plant->Resolve(Why) && Plant->MeasureLegs(Legs)))
	{
		return false;
	}
	Plant->LevelledMotorIds(Ids);
	TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	TMap<FName, float> Baselines;
	for (const FName& Id : Ids)
	{
		Baselines.Add(Id, R.Baseline(Id));
	}
	auto ReadTarget = [](const URamms5BarLinkageController* Linkage) {
		float							X = NAN, Z = NAN;
		const IRammsControlContributor* Contributor = Linkage;
		Contributor->ReadTarget(RammsControlIds::Linkage::Translation(Linkage->GetName()), X);
		Contributor->ReadTarget(RammsControlIds::Linkage::Height(Linkage->GetName()), Z);
		return FVector2D(X, Z);
	};
	const FVector2D TargetBefore = ReadTarget(R.Left);
	TestTrue(TEXT("the linkage holds the stance as its target"), TargetBefore.Equals(FVector2D(CentreX, StanceZ), 1.0e-4));

	// No offset asked is exactly none given.
	const double None[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	Plant->ApplyOffsets(None);
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s: no height asked, an offset of exactly zero"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
	}

	// A height each: positive retracts (the wheel comes up).
	const double Dz[6] = { 2.5, -1.0, 1.5, -2.0, -3.0, 0.75 };
	Plant->ApplyOffsets(Dz);
	R.Tick();
	for (int32 Index = 0; Index < 6; ++Index)
	{
		TestEqual(*FString::Printf(TEXT("leg %d: told the height asked"), Index), R.LegZ(Index, true), StanceZ + Dz[Index], 0.02);
		TestEqual(*FString::Printf(TEXT("leg %d: its baseline height as it was"), Index), R.LegZ(Index, false), StanceZ, 0.02);
	}
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s: its baseline to the bit"), *Id.ToString()), SameBits(R.Baseline(Id), Baselines[Id]));
		TestTrue(*FString::Printf(TEXT("%s: told its baseline plus its offset"), *Id.ToString()), SameBits(R.Log->LastFor(Id), R.Effective(Id)));
	}
	// A 5-bar's endpoint rose straight up: no fore/aft with it.
	{
		bool			bValid = false;
		const FVector2D End = URamms5BarKinematics::ComputeEndpoint(CentreLinkage(HipLA, HipLB), FVector2D(R.Effective(HipLA), R.Effective(HipLB)), bValid);
		TestTrue(TEXT("the centre leg's hips close the linkage"), bValid);
		// (Its two hips turned by different amounts: were they given each other's, the wheel would be elsewhere.)
		TestTrue(TEXT("the two hips' offsets differ"), FMath::Abs(R.Base->GetMotorCommandOffset(HipLA) - R.Base->GetMotorCommandOffset(HipLB)) > 0.005f);
		TestEqual(TEXT("... with the wheel where it was fore and aft"), End.X, CentreX, 0.02);
	}
	// The target the 5-bar controller holds is what a stance ramp, a held key and a jog start from: an offset must
	// not show in it.
	TestTrue(TEXT("the linkage's target is untouched by the offset"), ReadTarget(R.Left).Equals(TargetBefore, 1.0e-6));
	// ... so a height commanded through the controller moves the baseline, and the offset rides on top of it.
	TestTrue(TEXT("a height commanded through the linkage"), R.Left->SetEndpointHeight(static_cast<float>(StanceZ - 1.0)));
	TestTrue(TEXT("measured again"), Plant->MeasureLegs(Legs));
	TestEqual(TEXT("its baseline height moved by as much"), Legs[2].BaseZ, StanceZ - 1.0, 0.02);
	Plant->ApplyOffsets(Dz);
	R.Tick();
	TestEqual(TEXT("... and the leg is told that height plus the same offset"), R.LegZ(2, true), StanceZ - 1.0 + Dz[2], 0.02);

	// A height past a leg's travel stops at its end rather than asking the motor past its range.
	const double Far[6] = { 60.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	Plant->ApplyOffsets(Far);
	R.Tick();
	TestEqual(TEXT("a height past the travel stops at the travel's end"), R.LegZ(0, true), Legs[0].MaxZ, 0.02);

	Plant->ZeroOffsets();
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s: zeroed"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
	}
	R.Base->ReleaseCommandOffsetLayer(R.Layer());
	return true;
}

// Catches: a fold that moves a motor or tells it anything, a fold that leaves the 5-bar controller holding the old
// target, an unfold that does not put the baselines back to the bit, an unfold made after the taker has written, and
// the backstop folding a motor the taker has already rewritten (its new command would carry the old offset).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLegPlantFoldAndUnfold,
	"Ramms.Level.LegPlant.FoldAndUnfold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLegPlantFoldAndUnfold::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/false);
	if (!TestTrue(TEXT("the made-up robot on the recording backend"), R.IsReady()))
	{
		return false;
	}
	TUniquePtr<RammsLevel::IPlant> Plant = RammsLevel::MakeLegPlant(*R.Base, *R.Curb, R.Layer());
	FString						   Why;
	RammsLevel::FPlantLegs		   Legs;
	TArray<FName>				   Ids;
	if (!TestTrue(TEXT("resolved, measured"), Plant->Resolve(Why) && Plant->MeasureLegs(Legs)))
	{
		return false;
	}
	Plant->LevelledMotorIds(Ids);
	TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	const double Dz[6] = { 2.0, 1.0, 0.5, -0.5, -2.0, -1.0 };
	Plant->ApplyOffsets(Dz);
	R.Tick();
	TMap<FName, float> Baselines, Told;
	for (const FName& Id : Ids)
	{
		Baselines.Add(Id, R.Baseline(Id));
		Told.Add(Id, R.Effective(Id));
	}
	const int32 Before = R.Log->Pushes.Num();

	// The fold: every offset into its baseline, nothing told, the 5-bar's target moved with its hips' commands.
	Plant->Fold();
	R.Tick();
	TestEqual(TEXT("the fold told the motors nothing, then or at the tick"), R.Log->Pushes.Num(), Before);
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s: told the same, to the bit"), *Id.ToString()), SameBits(R.Effective(Id), Told[Id]));
		TestTrue(*FString::Printf(TEXT("%s: its baseline is now what it is told"), *Id.ToString()), SameBits(R.Baseline(Id), Told[Id]));
		TestTrue(*FString::Printf(TEXT("%s: no offset left"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
	}
	{
		float							Z = NAN;
		const IRammsControlContributor* Contributor = R.Left;
		TestTrue(TEXT("the linkage holds a target"), Contributor->ReadTarget(RammsControlIds::Linkage::Height(R.Left->GetName()), Z));
		TestEqual(TEXT("... the folded height, not the stance's"), static_cast<double>(Z), StanceZ + Dz[2], 0.02);
	}

	// The unfold, nothing written meanwhile: exactly as before the fold.
	TestTrue(TEXT("unfolded"), Plant->Unfold());
	R.Tick();
	TestEqual(TEXT("the unfold told the motors nothing either"), R.Log->Pushes.Num(), Before);
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s: its baseline back, to the bit"), *Id.ToString()), SameBits(R.Baseline(Id), Baselines[Id]));
		TestTrue(*FString::Printf(TEXT("%s: still told the same, to the bit"), *Id.ToString()), SameBits(R.Effective(Id), Told[Id]));
	}
	{
		float							Z = NAN;
		const IRammsControlContributor* Contributor = R.Left;
		Contributor->ReadTarget(RammsControlIds::Linkage::Height(R.Left->GetName()), Z);
		TestEqual(TEXT("the linkage's target back at the stance"), static_cast<double>(Z), StanceZ, 0.02);
	}
	TestFalse(TEXT("there is no second unfold"), Plant->Unfold());

	// Folded again, and the taker writes one motor: what it wrote stands, and nothing is put back.
	Plant->Fold();
	R.Base->SetMotorCommand(CrankRL, 0.2f);
	TestFalse(TEXT("an unfold after the taker wrote is refused"), Plant->Unfold());
	TestTrue(TEXT("... the motor it wrote as it wrote it"), SameBits(R.Baseline(CrankRL), 0.2f));
	TestTrue(TEXT("... and the others still folded"), SameBits(R.Baseline(CrankFL), Told[CrankFL]));

	// The backstop. Offsets on again (from the baselines as they now stand), measured twice so that "since the
	// last tick" has a tick to compare with; then one motor is rewritten and the rest are left alone.
	TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
	Plant->ApplyOffsets(Dz);
	R.Tick();
	TestTrue(TEXT("measured again"), Plant->MeasureLegs(Legs));
	TMap<FName, float> Carried;
	for (const FName& Id : Ids)
	{
		Carried.Add(Id, R.Effective(Id));
	}
	R.Base->SetMotorCommand(CrankFR, -0.15f);
	TestTrue(TEXT("measured with one motor rewritten"), Plant->MeasureLegs(Legs));
	int32 Folded = 0, Zeroed = 0;
	Plant->FoldOrZero(Folded, Zeroed);
	TestEqual(TEXT("the rewritten motor's offset is dropped"), Zeroed, 1);
	TestEqual(TEXT("the seven left alone are folded"), Folded, 7);
	TestTrue(TEXT("the rewritten motor is told its new command, bare"), SameBits(R.Effective(CrankFR), -0.15f));
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s: no offset left"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
		if (Id != CrankFR)
		{
			TestTrue(*FString::Printf(TEXT("%s: told the same, to the bit"), *Id.ToString()), SameBits(R.Effective(Id), Carried[Id]));
		}
	}
	R.Base->ReleaseCommandOffsetLayer(R.Layer());
	return true;
}

// Catches: motors claimed while the switch is off (every baseline write would then wait a frame for nothing), a first
// frame that steps the legs, a correction of the wrong sign, a wanted attitude that outruns the tilt policy, a
// robot at rest still being commanded, and a switch-off that leaves an offset on or the layer held.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistClaimsLevelsAndReleases,
	"Ramms.Level.Assist.ClaimsLevelsRestsAndReleases",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistClaimsLevelsAndReleases::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	FLevelRig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
	{
		return false;
	}
	const float Dt = 1.0f / 30.0f;
	R.GroundDeg = FVector2D(4.0, 2.0);
	R.PoseChassis(R.GroundDeg);

	// Off: nothing is claimed, so a baseline write is told at once, as it always was.
	R.Frames(3);
	TestTrue(TEXT("off to begin with"), !R.Level->IsAssistEnabled() && R.State() == EState::Off && R.Status() == TEXT("Off"));
	TestEqual(TEXT("off: level.tilt reads the slope the base stands on"), static_cast<double>(R.Control(RammsLevel::TiltControlId())), RammsLevel::Inclination(R.GroundDeg), 0.01);
	TestTrue(TEXT("off: no tick order is asked of anything"), R.Level->PrimaryComponentTick.GetPrerequisites().IsEmpty() && !FLevelRig::TicksAfter(R.Base, R.Level));
	int32 Before = R.Log->CountFor(CrankFL);
	R.Base->SetMotorCommand(CrankFL, 0.0f);
	TestEqual(TEXT("off: a leg's command is told at once (not claimed)"), R.Log->CountFor(CrankFL), Before + 1);
	TestFalse(TEXT("off: not levelling"), R.Level->IsLevelling());

	// On: claimed on the first frame, with offsets of exactly zero, the wanted attitude where the base is.
	IRammsControlContributor* Controls = R.Level;
	TestTrue(TEXT("switched on through its control"), Controls->ApplyControl(RammsLevel::AssistControlId(), 1.0f));
	R.Frame(Dt);
	TestTrue(TEXT("on: levelling"), R.State() == EState::Active && R.Level->IsLevelling());
	Before = R.Log->CountFor(CrankFL);
	R.Base->SetMotorCommand(CrankFL, 0.0f);
	TestEqual(TEXT("on: a leg's command waits for the base's tick (claimed)"), R.Log->CountFor(CrankFL), Before);
	for (const FName& Id : FLevelRig::LegMotors())
	{
		TestTrue(*FString::Printf(TEXT("%s: the first frame's offset is exactly zero"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
	}
	TestTrue(TEXT("the wanted attitude starts where the base is"), R.Level->GetTargetAttitudeDeg().Equals(R.GroundDeg, 0.01));
	TestFalse(TEXT("another object cannot take the offset layer meanwhile"), R.Base->ClaimCommandOffsetLayer(R.Other(), TArray<FName>{ CrankFL }));

	// It walks level: the wanted attitude within the policy's 3 deg/s, the base following, the nose-up front legs
	// retracting and the high side's too.
	FVector2D Wanted = R.Level->GetTargetAttitudeDeg();
	double	  MaxRate = 0.0;
	for (int32 Index = 0; Index < 150; ++Index)
	{
		R.Frame(Dt);
		const FVector2D Now = R.Level->GetTargetAttitudeDeg();
		MaxRate = FMath::Max(MaxRate, (Now - Wanted).Size() / Dt);
		Wanted = Now;
	}
	TestTrue(TEXT("the wanted attitude moved, and no faster than the tilt policy"), MaxRate > 1.0 && MaxRate <= 3.0 + 0.05);
	const FVector2D Attitude = R.GroundDeg - R.CorrectionDeg();
	AddInfo(FString::Printf(TEXT("after 5 s on (4, 2) deg: attitude (%.2f, %.2f), wanted rate at most %.2f deg/s, status '%s'"), Attitude.X, Attitude.Y, MaxRate, *R.Status()));
	TestTrue(TEXT("the base is level"), Attitude.Size() <= 0.05);
	TestTrue(TEXT("the wanted attitude is level"), R.Level->GetTargetAttitudeDeg().Size() <= 0.05);
	TestTrue(TEXT("nose up: the front legs retracted, the rear extended"), R.LegDz(0) + R.LegDz(1) > 4.0 && R.LegDz(4) + R.LegDz(5) < -4.0);
	TestTrue(TEXT("+Y side up: the +Y legs retracted more than the -Y ones"), R.LegDz(1) - R.LegDz(0) > 1.0 && R.LegDz(3) - R.LegDz(2) > 1.0);
	TestEqual(TEXT("the ride height is kept: the legs' offsets sum to nothing"), R.LegDz(0) + R.LegDz(1) + R.LegDz(2) + R.LegDz(3) + R.LegDz(4) + R.LegDz(5), 0.0, 0.3);
	TestEqual(TEXT("level.tilt reads the base's tilt"), static_cast<double>(R.Control(RammsLevel::TiltControlId())), RammsLevel::Inclination(Attitude), 0.05);
	{
		// level.reach: the least share of its planned travel (to the margin and the soft zone) any leg has left, from
		// the made-up legs' own travel and how far each has been moved. (No leg is carried off the ground and the
		// correction is far inside its per-axis limit: a leg's travel is what is nearest.)
		TUniquePtr<RammsLevel::IPlant> Probe = RammsLevel::MakeLegPlant(*R.Base, *R.Curb, R.Other());
		FString						   Why;
		RammsLevel::FPlantLegs		   Legs;
		double						   Expected = 1.0;
		if (TestTrue(TEXT("the legs measure"), Probe->Resolve(Why) && Probe->MeasureLegs(Legs) && Legs.Num() == 6))
		{
			const double Margin = R.Level->Tuning.LegRangeMarginCm + R.Level->Tuning.SoftZoneCm;
			for (int32 Index = 0; Index < 6; ++Index)
			{
				const double Dz = R.LegDz(Index);
				const double Limit = Dz > 0.0 ? Legs[Index].MaxZ - Margin - Legs[Index].BaseZ : Legs[Index].MinZ + Margin - Legs[Index].BaseZ;
				Expected = FMath::Min(Expected, 1.0 - Dz / Limit);
			}
		}
		AddInfo(FString::Printf(TEXT("level.reach reads %.3f; the legs' travel gives %.3f"), R.Control(RammsLevel::ReachControlId()), Expected));
		TestTrue(TEXT("(the legs have used a good part of their travel, and not all)"), Expected > 0.3 && Expected < 0.85);
		TestEqual(TEXT("level.reach reads the least share of planned travel a leg has left"), static_cast<double>(R.Control(RammsLevel::ReachControlId())), Expected, 0.02);
	}
	TestEqual(TEXT("level.speed_scale reads full speed"), R.Control(RammsLevel::SpeedScaleControlId()), 1.0f);
	// While it holds the legs' offsets it ticks after the curb controller (a baseline writer that ticks before
	// physics), and the base after it; never after a 5-bar controller, which ticks during physics and would take
	// both out of the group before physics.
	TestTrue(TEXT("on: it ticks after the curb controller"), FLevelRig::TicksAfter(R.Level, R.Curb));
	TestTrue(TEXT("on: the base ticks after it"), FLevelRig::TicksAfter(R.Base, R.Level));

	// At rest: after the settle time nothing more is written, so the base tells the motors nothing.
	R.Frames(60);
	TestEqual(TEXT("at rest the status says so"), R.Status(), FString(TEXT("Level, at rest")));
	Before = R.LegPushes();
	R.Frames(30);
	TestEqual(TEXT("at rest no leg motor is told anything"), R.LegPushes(), Before);

	// The rider's preference: the wanted attitude walks to it, within the policy, and the base follows.
	TestTrue(TEXT("a preferred pitch"), Controls->ApplyControl(RammsLevel::PreferencePitchControlId(), 3.0f));
	TestTrue(TEXT("a preferred roll"), Controls->ApplyControl(RammsLevel::PreferenceRollControlId(), -1.5f));
	TestEqual(TEXT("the preference reads back"), R.Control(RammsLevel::PreferencePitchControlId()), 3.0f);
	Wanted = R.Level->GetTargetAttitudeDeg();
	MaxRate = 0.0;
	for (int32 Index = 0; Index < 150; ++Index)
	{
		R.Frame(Dt);
		const FVector2D Now = R.Level->GetTargetAttitudeDeg();
		MaxRate = FMath::Max(MaxRate, (Now - Wanted).Size() / Dt);
		Wanted = Now;
	}
	const FVector2D Preferred = R.GroundDeg - R.CorrectionDeg();
	AddInfo(FString::Printf(TEXT("preference (3, -1.5): attitude (%.2f, %.2f), wanted rate at most %.2f deg/s"), Preferred.X, Preferred.Y, MaxRate));
	TestTrue(TEXT("the wanted attitude went to the preference"), R.Level->GetTargetAttitudeDeg().Equals(FVector2D(3.0, -1.5), 0.05));
	TestTrue(TEXT("... within the tilt policy"), MaxRate > 1.0 && MaxRate <= 3.0 + 0.05);
	TestTrue(TEXT("the base is held at the preference"), (Preferred - FVector2D(3.0, -1.5)).Size() <= 0.05);
	TestEqual(TEXT("level.tilt reads the tilt it is held at (the preference's, not 0)"), static_cast<double>(R.Control(RammsLevel::TiltControlId())), RammsLevel::Inclination(FVector2D(3.0, -1.5)), 0.05);
	TestTrue(TEXT("a preference past its bound is taken"), Controls->ApplyControl(RammsLevel::PreferenceRollControlId(), 40.0f));
	TestEqual(TEXT("... as the bound"), R.Control(RammsLevel::PreferenceRollControlId()), 5.0f);
	TestTrue(TEXT("back to none"), Controls->ApplyControl(RammsLevel::PreferenceRollControlId(), 0.0f) && Controls->ApplyControl(RammsLevel::PreferencePitchControlId(), 0.0f));
	R.Frames(150);

	// The ground under it changes at once by more than the legs can follow in a frame: the tilt reads what the base
	// really has, and the speed limit comes down from full (shown; nothing is slowed by it yet).
	R.GroundDeg += FVector2D(2.5, 0.0);
	R.PoseChassis(R.GroundDeg - R.CorrectionDeg());
	float  LeastScale = 1.0f;
	double MostTilt = 0.0;
	for (int32 Index = 0; Index < 20; ++Index)
	{
		R.Frame(Dt);
		MostTilt = FMath::Max(MostTilt, static_cast<double>(R.Control(RammsLevel::TiltControlId())));
		LeastScale = FMath::Min(LeastScale, R.Control(RammsLevel::SpeedScaleControlId()));
	}
	AddInfo(FString::Printf(TEXT("ground stepped 2.5 deg: level.tilt at most %.2f deg, level.speed_scale down to %.2f"), MostTilt, LeastScale));
	// (The law's attitude is filtered, and the legs answer meanwhile: most of the step, never more than it.)
	TestTrue(TEXT("level.tilt read the step"), MostTilt > 1.5 && MostTilt <= 2.5 + 0.01);
	TestTrue(TEXT("level.speed_scale came down, and not under its floor"), LeastScale < 0.9f && LeastScale >= R.Level->Tuning.MinSpeedScale - 1.0e-4f);
	R.Frames(240);
	TestEqual(TEXT("... and is back at full speed once level"), R.Control(RammsLevel::SpeedScaleControlId()), 1.0f);

	// Off again: the base is given back to its stance within the policy; then every offset is exactly zero, the
	// motors told exactly their baselines, and the layer given up.
	TestTrue(TEXT("switched off by the button on the switch"), Controls->TriggerControl(RammsLevel::AssistControlId()) && !R.Level->IsAssistEnabled());
	R.Frame(Dt);
	TestTrue(TEXT("releasing, not dropped"), R.State() == EState::Releasing && R.Level->IsLevelling());
	TestTrue(TEXT("... the offsets still on"), FMath::Abs(R.LegDz(0)) > 1.0);
	R.Frames(240);
	TestTrue(TEXT("off"), R.State() == EState::Off && !R.Level->IsLevelling());
	for (const FName& Id : FLevelRig::LegMotors())
	{
		TestTrue(*FString::Printf(TEXT("%s: no offset left"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
		TestTrue(*FString::Printf(TEXT("%s: told exactly its baseline"), *Id.ToString()), SameBits(R.Log->LastFor(Id), R.Baseline(Id)));
	}
	TestTrue(TEXT("the base is back on the slope"), (R.GroundDeg - R.CorrectionDeg()).Equals(R.GroundDeg, 1.0e-6));
	Before = R.Log->CountFor(CrankFL);
	R.Base->SetMotorCommand(CrankFL, 0.0f);
	TestEqual(TEXT("off again: a leg's command is told at once (the layer given up)"), R.Log->CountFor(CrankFL), Before + 1);
	TestTrue(TEXT("off again: no tick order is asked of anything"), R.Level->PrimaryComponentTick.GetPrerequisites().IsEmpty() && !FLevelRig::TicksAfter(R.Base, R.Level));
	TestTrue(TEXT("... and another object may take the layer"), R.Base->ClaimCommandOffsetLayer(R.Other(), TArray<FName>{ CrankFL }));
	return true;
}

// Catches: a yield that moves a motor, tells it anything or leaves an offset on; a refused take-over that does not put
// everything back; levelling carrying on under a taker that holds the legs; a hand-back treated as a refusal; a
// hand-back with no stance that tells a motor anything, or leaves the pose's plane in the commands.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistFoldOnYield,
	"Ramms.Level.Assist.FoldOnYieldKeepsCommands",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistFoldOnYield::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	FLevelRig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
	{
		return false;
	}
	R.GroundDeg = FVector2D(-3.0, 2.0);
	R.PoseChassis(R.GroundDeg);
	R.Level->SetAssistEnabled(true);
	R.Frames(150);
	if (!TestTrue(TEXT("levelled on the slope, offsets on"), R.State() == EState::Active && FMath::Abs(R.LegDz(0)) > 1.0))
	{
		return false;
	}
	const TArray<FName> Ids = FLevelRig::LegMotors();
	TMap<FName, float>	Baselines, Told;
	for (const FName& Id : Ids)
	{
		Baselines.Add(Id, R.Baseline(Id));
		Told.Add(Id, R.Effective(Id));
	}
	const FVector2D WantedBefore = R.Level->GetTargetAttitudeDeg();

	// The yield: in the call, before the taker reads a command.
	int32 Before = R.Log->Pushes.Num();
	R.Level->YieldTo(R.Curb);
	TestEqual(TEXT("the yield told the motors nothing"), R.Log->Pushes.Num(), Before);
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s: still told the same, to the bit"), *Id.ToString()), SameBits(R.Effective(Id), Told[Id]));
		TestTrue(*FString::Printf(TEXT("%s: the command the taker reads is what the motor is told"), *Id.ToString()), SameBits(R.Baseline(Id), Told[Id]));
		TestTrue(*FString::Printf(TEXT("%s: no offset left"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
	}
	TestTrue(TEXT("standing by"), R.State() == EState::Yielded && !R.Level->IsLevelling());
	TestEqual(TEXT("no seat trim is folded (the seat's tilt is not levelled yet)"), R.Level->GetFoldedSeatTrimDeg(), 0.0f);

	// Refused before it began (nothing written): everything exactly as it was, and levelling goes on with no step.
	R.Level->ResumeFrom(R.Curb, nullptr);
	TestEqual(TEXT("the refusal told the motors nothing"), R.Log->Pushes.Num(), Before);
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s: its baseline back, to the bit"), *Id.ToString()), SameBits(R.Baseline(Id), Baselines[Id]));
		TestTrue(*FString::Printf(TEXT("%s: told the same, to the bit"), *Id.ToString()), SameBits(R.Effective(Id), Told[Id]));
	}
	TestTrue(TEXT("levelling again, as before"), R.State() == EState::Active && R.Level->IsLevelling());
	TestTrue(TEXT("the wanted attitude as it was"), R.Level->GetTargetAttitudeDeg().Equals(WantedBefore, 1.0e-9));
	R.Frame();
	double Step = 0.0;
	for (const FName& Id : Ids)
	{
		Step = FMath::Max(Step, static_cast<double>(FMath::Abs(R.Effective(Id) - Told[Id])));
	}
	TestTrue(TEXT("no motor's command stepped on the frame after"), Step <= 2.0e-3);

	// A take-over that goes ahead: the taker writes its own commands and its mode has the robot. Levelling stands
	// by for as long as it does, writing nothing.
	R.Frames(30);
	R.Level->YieldTo(R.Curb);
	IRammsDriveMode* CurbMode = R.Curb;
	CurbMode->SetDriveModeActive(true);
	const float TakersCommand = R.Baseline(CrankFL) + 0.05f;
	R.Base->SetMotorCommand(CrankFL, TakersCommand);
	R.Frames(20);
	TestTrue(TEXT("standing by while the taker's mode has the robot"), R.State() == EState::Yielded);
	TestEqual(TEXT("... and saying so"), R.Status(), FString(TEXT("Standing by: a curb crossing has the legs")));
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s: no offset under the taker"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
	}
	TestTrue(TEXT("the taker's command is what its motor is told"), SameBits(R.Log->LastFor(CrankFL), TakersCommand));
	// It tries again from where it stopped and is refused: the legs are still its own.
	R.Level->YieldTo(R.Curb);
	R.Level->ResumeFrom(R.Curb, nullptr);
	TestTrue(TEXT("a refused second try gives nothing back"), R.State() == EState::Yielded && SameBits(R.Baseline(CrankFL), TakersCommand));
	// Its mode lets go with no hand-back: levelling starts again from the pose as it was left, with no step.
	CurbMode->SetDriveModeActive(false);
	Before = R.Log->Pushes.Num();
	R.Level->TickComponent(1.0f / 30.0f, LEVELTICK_All, nullptr);
	TestTrue(TEXT("levelling again once the taker's mode is gone"), R.State() == EState::Active);
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s: it starts with no offset"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
	}
	TestTrue(TEXT("... on the commands the taker left"), SameBits(R.Baseline(CrankFL), TakersCommand));

	// A take-over that wrote and then hands back with no stance to go to (on a robot levelled from its stance): what
	// it wrote stands (this is no refusal), no motor is told anything, and the plane the pose still has over the
	// commands from before the take-over goes back into the offsets: levelling carries on from the pose, rather than
	// keeping it as the stance for good.
	{
		FLevelRig Fresh;
		if (!TestTrue(TEXT("a second made-up robot"), Fresh.IsReady() && Fresh.Level != nullptr && Fresh.Level->HasBegunPlay()))
		{
			return false;
		}
		Fresh.GroundDeg = FVector2D(-3.0, 2.0);
		Fresh.PoseChassis(Fresh.GroundDeg);
		Fresh.Level->SetAssistEnabled(true);
		Fresh.Frames(150);
		const FVector2D Carried = Fresh.CorrectionDeg();
		Fresh.Level->YieldTo(Fresh.Curb);
		const float Wrote = Fresh.Baseline(CrankRR) + 0.02f;
		Fresh.Base->SetMotorCommand(CrankRR, Wrote);
		TMap<FName, float> ToldNow;
		for (const FName& Id : Ids)
		{
			ToldNow.Add(Id, Fresh.Effective(Id));
		}
		const int32 Pushed = Fresh.Log->Pushes.Num();
		Fresh.Level->ResumeFrom(Fresh.Curb, nullptr);
		TestEqual(TEXT("handed back after writing: no motor is told anything by it"), Fresh.Log->Pushes.Num(), Pushed);
		TestTrue(TEXT("... what it wrote is what its motor is told"), SameBits(Fresh.Effective(CrankRR), Wrote));
		for (const FName& Id : Ids)
		{
			TestTrue(*FString::Printf(TEXT("%s: handed back after writing: told the same, to the bit"), *Id.ToString()), SameBits(Fresh.Effective(Id), ToldNow[Id]));
		}
		TestTrue(TEXT("... levelling from there"), Fresh.State() == EState::Active);
		AddInfo(FString::Printf(TEXT("hand-back with no stance after one crank was written: the legs carried (%.2f, %.2f) deg before, (%.2f, %.2f) after"), Carried.X, Carried.Y,
			Fresh.CorrectionDeg().X, Fresh.CorrectionDeg().Y));
		// (The one crank it moved is no plane: what the legs carried comes back, and the rest stays a command.)
		TestTrue(TEXT("... the plane the pose carries back in the offsets"), Carried.Size() > 3.0 && (Fresh.CorrectionDeg() - Carried).Size() <= 0.4);
	}
	return true;
}

// Catches: a mode that drives the legs becoming active with nobody yielding, and the offsets left on top of its
// commands (every leg would be off by the plane); or the offset of a motor it has not written yet dropped (a step).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistBackstop,
	"Ramms.Level.Assist.BackstopFoldsOrZeroes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistBackstop::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	FLevelRig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
	{
		return false;
	}
	AddExpectedMessagePlain(TEXT("took the legs without yielding"), ELogVerbosity::Warning, EAutomationExpectedMessageFlags::Contains, 1);
	R.GroundDeg = FVector2D(3.0, -2.0);
	R.PoseChassis(R.GroundDeg);
	R.Level->SetAssistEnabled(true);
	R.Frames(150);
	if (!TestTrue(TEXT("levelled on the slope, offsets on"), R.State() == EState::Active && FMath::Abs(R.LegDz(0)) > 1.0))
	{
		return false;
	}
	const TArray<FName> Ids = FLevelRig::LegMotors();
	TMap<FName, float>	Told;
	for (const FName& Id : Ids)
	{
		Told.Add(Id, R.Effective(Id));
	}
	// The curb mode active with no YieldTo, one crank already rewritten by it this frame.
	IRammsDriveMode* CurbMode = R.Curb;
	CurbMode->SetDriveModeActive(true);
	R.Base->SetMotorCommand(CrankRL, 0.25f);
	R.Level->TickComponent(1.0f / 30.0f, LEVELTICK_All, nullptr);
	TestTrue(TEXT("standing by at once"), R.State() == EState::Yielded && !R.Level->IsLevelling());
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s: no offset left"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
		if (Id == CrankRL)
		{
			TestTrue(TEXT("the rewritten crank is told its new command, bare"), SameBits(R.Effective(Id), 0.25f));
		}
		else
		{
			TestTrue(*FString::Printf(TEXT("%s: left alone, it is told the same, to the bit"), *Id.ToString()), SameBits(R.Effective(Id), Told[Id]));
		}
	}
	R.Tick();
	TestTrue(TEXT("the base tells the rewritten crank its command"), SameBits(R.Log->LastFor(CrankRL), 0.25f));
	return true;
}

// Catches: the switch taken on a robot the level assist cannot describe (it would claim nothing and say it levels),
// and levelling running in a drive mode it was told to keep out of.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistRefusesAndWaits,
	"Ramms.Level.Assist.RefusesWithNoDescriptionAndWaitsForItsMode",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistRefusesAndWaits::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	{
		AddExpectedMessagePlain(TEXT("cannot be levelled: no robot description"), ELogVerbosity::Warning, EAutomationExpectedMessageFlags::Contains, 1);
		AddExpectedMessagePlain(TEXT("level assist stays off"), ELogVerbosity::Warning, EAutomationExpectedMessageFlags::Contains, 1);
		FLevelRig R(/*bWithCurb=*/false);
		if (!TestTrue(TEXT("a robot with no description"), R.IsReady() && R.Level != nullptr))
		{
			return false;
		}
		R.Frame();
		IRammsControlContributor* Controls = R.Level;
		TestFalse(TEXT("the switch refuses on"), Controls->ApplyControl(RammsLevel::AssistControlId(), 1.0f));
		TestFalse(TEXT("... and stays off"), R.Level->IsAssistEnabled());
		TestEqual(TEXT("the status says why"), R.Status(), FString(TEXT("Level: no robot description")));
		TestEqual(TEXT("... as a fault"), R.Control(RammsLevel::StatusControlId()), static_cast<float>(ERammsControlTextSeverity::Fault));
		R.Frames(3);
		R.Base->SetMotorCommand(CrankFL, 0.1f);
		TestTrue(TEXT("nothing was claimed"), SameBits(R.Log->LastFor(CrankFL), 0.1f));
	}
	{
		// It levels only in a named mode, and no such mode has the robot: on, waiting, no offsets.
		FLevelRig R;
		if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr))
		{
			return false;
		}
		R.Level->EnabledModeIds = { FName(TEXT("differential")) };
		R.GroundDeg = FVector2D(4.0, 0.0);
		R.PoseChassis(R.GroundDeg);
		R.Level->SetAssistEnabled(true);
		R.Frames(30);
		TestTrue(TEXT("on, and waiting for its mode"), R.Level->IsAssistEnabled() && R.State() == EState::Suspended && !R.Level->IsLevelling());
		TestEqual(TEXT("... saying so"), R.Status(), FString(TEXT("Waiting: not levelling in this drive mode")));
		TestEqual(TEXT("no leg was moved"), R.LegDz(0), 0.0, 1.0e-9);
		// Any mode will do once none is named.
		R.Level->EnabledModeIds.Reset();
		R.Frames(5);
		TestTrue(TEXT("levelling once its mode has the robot"), R.State() == EState::Active);
		// The controls it offers, in the Drive group.
		FRammsControlSurface Surface;
		static_cast<const IRammsControlContributor*>(R.Level)->DescribeControls(Surface);
		for (const FName& Id : { RammsLevel::AssistControlId(), RammsLevel::StatusControlId(), RammsLevel::StateControlId(), RammsLevel::TiltControlId(),
				 RammsLevel::ReachControlId(), RammsLevel::SpeedScaleControlId(), RammsLevel::PreferencePitchControlId(), RammsLevel::PreferenceRollControlId() })
		{
			const FRammsControlAxis* Axis = Surface.Find(Id);
			if (TestNotNull(*FString::Printf(TEXT("%s is offered"), *Id.ToString()), Axis))
			{
				TestTrue(*FString::Printf(TEXT("%s is in the Drive group"), *Id.ToString()), Axis->Group == RammsControlIds::Groups::Drive());
			}
		}
		if (const FRammsControlAxis* Pitch = Surface.Find(RammsLevel::PreferencePitchControlId()))
		{
			TestTrue(TEXT("the pitch preference spans its bound, in degrees"), Pitch->Range.Equals(FVector2D(-8.0, 8.0)) && Pitch->Units == ERammsControlUnits::Degrees && !Pitch->bReadOnly);
		}
		if (const FRammsControlAxis* Roll = Surface.Find(RammsLevel::PreferenceRollControlId()))
		{
			TestTrue(TEXT("the roll preference spans its bound"), Roll->Range.Equals(FVector2D(-5.0, 5.0)));
		}
		if (const FRammsControlAxis* Switch = Surface.Find(RammsLevel::AssistControlId()))
		{
			TestTrue(TEXT("the switch shows on"), Switch->Kind == ERammsControlKind::Enum && Switch->EnumLabels.Num() == 2 && Switch->DefaultValue == 1.0f);
		}
		TestEqual(TEXT("its contributor order"), static_cast<const IRammsControlContributor*>(R.Level)->GetControlOrder(), 6);
	}
	return true;
}

// Catches: an unfold that says it put the baselines back when the base refused (the offset layer given up since the
// fold): the caller would then go on as if nothing had been asked, with the correction still in the baselines, and add
// it a second time.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLegPlantUnfoldNeedsTheLayer,
	"Ramms.Level.LegPlant.UnfoldNeedsTheLayer",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLegPlantUnfoldNeedsTheLayer::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/false);
	if (!TestTrue(TEXT("the made-up robot on the recording backend"), R.IsReady()))
	{
		return false;
	}
	TUniquePtr<RammsLevel::IPlant> Plant = RammsLevel::MakeLegPlant(*R.Base, *R.Curb, R.Layer());
	FString						   Why;
	RammsLevel::FPlantLegs		   Legs;
	TArray<FName>				   Ids;
	if (!TestTrue(TEXT("resolved, measured"), Plant->Resolve(Why) && Plant->MeasureLegs(Legs)))
	{
		return false;
	}
	Plant->LevelledMotorIds(Ids);
	TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	const double Dz[6] = { 2.0, 1.0, 0.5, -0.5, -2.0, -1.0 };
	Plant->ApplyOffsets(Dz);
	R.Tick();
	TMap<FName, float> Stance, Told, Folded;
	for (const FName& Id : Ids)
	{
		Stance.Add(Id, R.Baseline(Id));
		Told.Add(Id, R.Effective(Id));
	}
	Plant->Fold();
	R.Base->ReleaseCommandOffsetLayer(R.Layer());
	for (const FName& Id : Ids)
	{
		Folded.Add(Id, R.Baseline(Id));
	}
	TestTrue(TEXT("(the fold moved the baselines)"), !SameBits(Folded[CrankFL], Stance[CrankFL]));
	// With the layer given up no baseline can be put back: it must say so, and leave everything as it is.
	TestFalse(TEXT("an unfold without the layer is refused"), Plant->Unfold());
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s: its baseline as the fold left it"), *Id.ToString()), SameBits(R.Baseline(Id), Folded[Id]));
	}
	// The fold is still remembered: with the layer held again (nothing written meanwhile) it is undone exactly.
	TestTrue(TEXT("claimed again"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	TestTrue(TEXT("with the layer held the unfold goes through"), Plant->Unfold());
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s: its baseline back at the stance, to the bit"), *Id.ToString()), SameBits(R.Baseline(Id), Stance[Id]));
		TestTrue(*FString::Printf(TEXT("%s: told what it was told before the fold, to the bit"), *Id.ToString()), SameBits(R.Effective(Id), Told[Id]));
	}
	Plant->ZeroOffsets();
	R.Base->ReleaseCommandOffsetLayer(R.Layer());
	return true;
}

// Catches: an offset left waiting on a motor nothing commands (it would be added to whatever commands it next), a fold
// or unfold that trips over such a motor, and a leg commanded again after being let go that is not noticed to have lost
// its offset (it would be told its command bare, a step of its whole correction).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLegPlantReleasedMotor,
	"Ramms.Level.LegPlant.ReleasedMotorAndLostOffsets",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLegPlantReleasedMotor::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/false);
	if (!TestTrue(TEXT("the made-up robot on the recording backend"), R.IsReady()))
	{
		return false;
	}
	TUniquePtr<RammsLevel::IPlant> Plant = RammsLevel::MakeLegPlant(*R.Base, *R.Curb, R.Layer());
	FString						   Why;
	RammsLevel::FPlantLegs		   Legs;
	TArray<FName>				   Ids;
	if (!TestTrue(TEXT("resolved, measured"), Plant->Resolve(Why) && Plant->MeasureLegs(Legs)))
	{
		return false;
	}
	Plant->LevelledMotorIds(Ids);
	TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	TestTrue(TEXT("before any offset is applied every leg is owed one"), Plant->OffsetsLost());
	const double Dz[6] = { 2.0, 1.0, 0.5, -0.5, -2.0, -1.0 };
	Plant->ApplyOffsets(Dz);
	R.Tick();
	TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
	TestFalse(TEXT("applied: nothing is lost"), Plant->OffsetsLost());

	// A crank is let go (driven by hand): the base drops its command and its offset.
	R.Base->ReleaseMotor(CrankRR);
	float Unused = 0.0f;
	TestFalse(TEXT("(the crank has no command now)"), R.Base->GetMotorCommand(CrankRR, Unused));
	TestTrue(TEXT("the legs still measure (it stands where its joint is)"), Plant->MeasureLegs(Legs) && Legs.Num() == 6);
	TestFalse(TEXT("a leg nothing commands is owed nothing"), Plant->OffsetsLost());
	int32 Before = R.Log->CountFor(CrankRR);
	Plant->ApplyOffsets(Dz);
	R.Tick();
	TestTrue(TEXT("no offset waits on the crank nothing commands"), R.Base->GetMotorCommandOffset(CrankRR) == 0.0f);
	TestEqual(TEXT("... and it is told nothing"), R.Log->CountFor(CrankRR), Before);
	TestEqual(TEXT("the other legs keep their offsets"), R.LegZ(0, true), StanceZ + Dz[0], 0.02);

	// A fold and its undo with that crank still let go: the others exactly, the crank left alone.
	TMap<FName, float> Stance, Told;
	for (const FName& Id : Ids)
	{
		Stance.Add(Id, R.Baseline(Id));
		Told.Add(Id, R.Effective(Id));
	}
	Plant->Fold();
	TestFalse(TEXT("folded: the crank still has no command"), R.Base->GetMotorCommand(CrankRR, Unused));
	TestTrue(TEXT("folded: another motor's baseline is what it is told"), SameBits(R.Baseline(CrankFL), Told[CrankFL]));
	TestTrue(TEXT("unfolded"), Plant->Unfold());
	for (const FName& Id : Ids)
	{
		if (Id != CrankRR)
		{
			TestTrue(*FString::Printf(TEXT("%s: its baseline back, to the bit"), *Id.ToString()), SameBits(R.Baseline(Id), Stance[Id]));
			TestTrue(*FString::Printf(TEXT("%s: told the same, to the bit"), *Id.ToString()), SameBits(R.Effective(Id), Told[Id]));
		}
	}
	TestFalse(TEXT("unfolded: the crank still has no command"), R.Base->GetMotorCommand(CrankRR, Unused));

	// Commanded again: it is owed its offset, and gets it before the base tells it anything.
	R.Base->SetMotorCommand(CrankRR, 0.0f);
	TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
	TestTrue(TEXT("commanded again, the leg is owed its offset"), Plant->OffsetsLost());
	Plant->ApplyOffsets(Dz);
	R.Tick();
	TestEqual(TEXT("... and is told its height with the offset"), R.LegZ(5, true), StanceZ + Dz[5], 0.02);
	TestTrue(TEXT("... the motor told its baseline plus its offset"), SameBits(R.Log->LastFor(CrankRR), R.Effective(CrankRR)) && !SameBits(R.Effective(CrankRR), R.Baseline(CrankRR)));
	TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
	TestFalse(TEXT("nothing is lost any more"), Plant->OffsetsLost());

	// The same through a 5-bar controller letting go of its hips and taking them back.
	IRammsControlContributor* LeftControls = R.Left;
	LeftControls->SetContributionSuspended(true);
	TestTrue(TEXT("the hips' offsets went with their commands"), R.Base->GetMotorCommandOffset(HipLA) == 0.0f && R.Base->GetMotorCommandOffset(HipLB) == 0.0f);
	TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
	TestFalse(TEXT("a linkage let go is owed nothing"), Plant->OffsetsLost());
	LeftControls->SetContributionSuspended(false);
	R.Left->SetEndpointTarget(FVector2D(CentreX, StanceZ));
	TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
	TestTrue(TEXT("taken back, the linkage's leg is owed its offset"), Plant->OffsetsLost());
	Plant->ApplyOffsets(Dz);
	R.Tick();
	TestEqual(TEXT("... and is told its height with the offset"), R.LegZ(2, true), StanceZ + Dz[2], 0.02);
	TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
	TestFalse(TEXT("nothing is lost any more"), Plant->OffsetsLost());
	Plant->ZeroOffsets();
	R.Base->ReleaseCommandOffsetLayer(R.Layer());
	return true;
}

// Catches: a speed scale that is worked out and shown and never reaches the drive; one sent to a mode while a curb
// crossing has the legs (the crossing runs its own pace), or left standing on the drive after levelling has stopped;
// and one sent to a mode that is not driving.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistSlowsTheDrive,
	"Ramms.Level.Assist.AsksTheDriveToGoSlowerWhileLevelling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistSlowsTheDrive::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	FLevelRig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
	{
		return false;
	}
	// The mode driving the robot: never begun (it drives nothing here), active as it is made.
	URammsHolonomicDriveController* Drive = NewObject<URammsHolonomicDriveController>(R.Actor, TEXT("Holonomic"));
	IRammsDriveMode*				DriveMode = Drive;
	IRammsDriveMode*				CurbMode = R.Curb;
	const float						Dt = 1.0f / 30.0f;
	R.GroundDeg = FVector2D(3.0, 1.0);
	R.PoseChassis(R.GroundDeg);

	// Off: nothing is asked of the drive.
	R.Frames(5, Dt);
	TestEqual(TEXT("off: the drive at full speed"), Drive->GetDriveSpeedScale(), 1.0f);
	R.Level->SetAssistEnabled(true);
	R.Frames(150, Dt);
	TestTrue(TEXT("levelling, level"), R.State() == EState::Active && (R.GroundDeg - R.CorrectionDeg()).Size() <= 0.3);
	TestEqual(TEXT("level, nothing turning: the drive at full speed"), Drive->GetDriveSpeedScale(), 1.0f);

	// The ground steps by more than the legs can follow in a frame: the scale comes down, and the drive has it on
	// every frame.
	R.GroundDeg += FVector2D(2.5, 0.0);
	R.PoseChassis(R.GroundDeg - R.CorrectionDeg());
	float LeastAsked = 1.0f;
	int32 Differing = 0;
	for (int32 Index = 0; Index < 12; ++Index)
	{
		R.Frame(Dt);
		const float Shown = R.Control(RammsLevel::SpeedScaleControlId());
		LeastAsked = FMath::Min(LeastAsked, Drive->GetDriveSpeedScale());
		Differing += FMath::IsNearlyEqual(Drive->GetDriveSpeedScale(), Shown, 1.0e-6f) ? 0 : 1;
	}
	TestTrue(*FString::Printf(TEXT("the drive is asked to go slower (%.2f)"), LeastAsked), LeastAsked < 0.9f && LeastAsked >= R.Level->Tuning.MinSpeedScale - 1.0e-4f);
	TestEqual(TEXT("... by what level.speed_scale shows, on every frame"), Differing, 0);
	TestTrue(TEXT("(still asked as the crossing begins)"), Drive->GetDriveSpeedScale() < 1.0f);

	// A curb crossing takes the legs: what was asked of the drive is taken back in the same call, and nothing is
	// asked while the crossing has them.
	R.Level->YieldTo(R.Curb);
	TestEqual(TEXT("yielded: the drive is let go at once"), Drive->GetDriveSpeedScale(), 1.0f);
	CurbMode->SetDriveModeActive(true);
	R.Frames(4, Dt);
	TestTrue(TEXT("(the crossing has the legs)"), R.State() == EState::Yielded);
	TestEqual(TEXT("during the crossing: nothing asked of the drive"), Drive->GetDriveSpeedScale(), 1.0f);
	CurbMode->SetDriveModeActive(false);
	R.Level->ResumeFrom(R.Curb, nullptr);
	R.Frames(150, Dt);
	TestTrue(TEXT("handed back: levelling again"), R.State() == EState::Active);

	// A mode that is not driving is asked nothing; and when levelling stops, what was asked is taken back.
	DriveMode->SetDriveModeActive(false);
	R.GroundDeg += FVector2D(-2.5, 0.0);
	R.PoseChassis(R.GroundDeg - R.CorrectionDeg());
	R.Frames(3, Dt);
	TestTrue(TEXT("(the scale is down again)"), R.Control(RammsLevel::SpeedScaleControlId()) < 1.0f);
	TestEqual(TEXT("a mode that is not driving is asked nothing"), Drive->GetDriveSpeedScale(), 1.0f);
	DriveMode->SetDriveModeActive(true);
	R.Frame(Dt);
	TestTrue(TEXT("driving again: asked"), Drive->GetDriveSpeedScale() < 1.0f);
	R.Level->SetAssistEnabled(false);
	R.Frame(Dt);
	TestTrue(TEXT("switched off: still asked while the base is given back"), R.State() == EState::Releasing && Drive->GetDriveSpeedScale() < 1.0f);
	R.Frames(300, Dt);
	TestTrue(TEXT("off"), R.State() == EState::Off);
	TestEqual(TEXT("off: the drive at full speed"), Drive->GetDriveSpeedScale(), 1.0f);

	// A mode it is told never to level in is not asked to go slower either, though it is the one driving: such a
	// mode keeps its own pace (a crossing does), and the base is only given back to it. (That a mode which is not
	// driving is asked nothing is, in the end, the drive's own doing: it takes no request while it is not active,
	// Ramms.Drive.SpeedScale.ModesTakeAndDropRequests. The level assist not sending it one is belt and braces, and
	// cannot be told apart from here.)
	R.Level->SetAssistEnabled(true);
	R.Frames(150, Dt);
	R.GroundDeg += FVector2D(2.5, 0.0);
	R.PoseChassis(R.GroundDeg - R.CorrectionDeg());
	R.Frames(3, Dt);
	TestTrue(TEXT("(levelling, the drive asked to go slower again)"), R.State() == EState::Active && Drive->GetDriveSpeedScale() < 1.0f);
	R.Level->DisabledModeIds.Add(DriveMode->GetDriveModeId());
	R.Frame(Dt);
	TestTrue(TEXT("(a mode it never levels in drives: the base is being given back)"), R.State() == EState::Releasing && R.Control(RammsLevel::SpeedScaleControlId()) < 1.0f);
	TestEqual(TEXT("... and that mode is not asked to go slower"), Drive->GetDriveSpeedScale(), 1.0f);
	return true;
}

// Catches: a level assist that, merely by being on the robot, changes the order the robot's components tick in (the
// curb's frame-locked runs would no longer repeat); one that makes a component ticking during physics its prerequisite
// (the engine starts a tick in the latest group among its prerequisites, so the level assist and the robot base behind
// it would leave the group before physics, and every wheel and leg command with them); and one that holds the legs'
// offsets without ticking after the writers of their baselines and before the base.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistTickOrder,
	"Ramms.Level.Assist.TickOrderOnlyWhileLevelling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistTickOrder::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	FLevelRig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
	{
		return false;
	}
	// A second drive mode, never begun: it drives nothing here, and stands for "a baseline writer that ticks before
	// physics".
	URammsHolonomicDriveController* Holonomic = NewObject<URammsHolonomicDriveController>(R.Actor, TEXT("Holonomic"));
	static_cast<IRammsDriveMode*>(Holonomic)->SetDriveModeActive(false);
	TestTrue(TEXT("(the base and the level assist tick before physics)"),
		R.Base->PrimaryComponentTick.TickGroup == TG_PrePhysics && R.Level->PrimaryComponentTick.TickGroup == TG_PrePhysics);
	TestTrue(TEXT("(a drive mode ticks before physics)"), Holonomic->PrimaryComponentTick.TickGroup == TG_PrePhysics);
	TestTrue(TEXT("(a 5-bar controller ticks during physics)"),
		R.Left->PrimaryComponentTick.TickGroup == TG_DuringPhysics && R.Right->PrimaryComponentTick.TickGroup == TG_DuringPhysics);
	auto NoOrder = [&](const TCHAR* When) {
		TestTrue(*FString::Printf(TEXT("%s: the level assist waits for nothing"), When), R.Level->PrimaryComponentTick.GetPrerequisites().IsEmpty());
		TestFalse(*FString::Printf(TEXT("%s: the base does not wait for the level assist"), When), FLevelRig::TicksAfter(R.Base, R.Level));
		TestFalse(*FString::Printf(TEXT("%s: the base does not wait for a drive mode because of it"), When), FLevelRig::TicksAfter(R.Base, Holonomic));
	};
	auto Ordered = [&](const TCHAR* When) {
		TestTrue(*FString::Printf(TEXT("%s: after the curb controller"), When), FLevelRig::TicksAfter(R.Level, R.Curb));
		TestTrue(*FString::Printf(TEXT("%s: the base after the level assist"), When), FLevelRig::TicksAfter(R.Base, R.Level));
		TestFalse(*FString::Printf(TEXT("%s: never after a 5-bar controller"), When), FLevelRig::TicksAfter(R.Level, R.Left) || FLevelRig::TicksAfter(R.Level, R.Right));
		TestTrue(*FString::Printf(TEXT("%s: the level assist still starts before physics"), When), FLevelRig::StartGroup(R.Level) == TG_PrePhysics);
		TestTrue(*FString::Printf(TEXT("%s: the base still starts before physics"), When), FLevelRig::StartGroup(R.Base) == TG_PrePhysics);
	};
	R.GroundDeg = FVector2D(3.0, 1.0);
	R.PoseChassis(R.GroundDeg);

	// Present and off: as if it were not there.
	R.Frames(5);
	NoOrder(TEXT("off"));

	// On: after the baseline writers that tick before physics, the base after it.
	R.Level->SetAssistEnabled(true);
	R.Frame();
	TestTrue(TEXT("levelling"), R.State() == EState::Active);
	Ordered(TEXT("on"));
	TestTrue(TEXT("on: after the other drive mode"), FLevelRig::TicksAfter(R.Level, Holonomic));
	R.Frames(60);
	Ordered(TEXT("levelled"));

	// Standing by for a crossing it still holds the layer (the taker's commands are told by the base's tick).
	R.Level->YieldTo(R.Curb);
	static_cast<IRammsDriveMode*>(R.Curb)->SetDriveModeActive(true);
	R.Level->TickComponent(1.0f / 30.0f, LEVELTICK_All, nullptr);
	R.Tick();
	TestTrue(TEXT("standing by"), R.State() == EState::Yielded);
	Ordered(TEXT("standing by"));
	static_cast<IRammsDriveMode*>(R.Curb)->SetDriveModeActive(false);
	R.Level->TickComponent(1.0f / 30.0f, LEVELTICK_All, nullptr);
	R.Tick();

	// Off again: once the layer is given up, no order is left behind.
	R.Level->SetAssistEnabled(false);
	R.Frames(300);
	TestTrue(TEXT("off"), R.State() == EState::Off);
	NoOrder(TEXT("off again"));

	// A drive mode whose tick group was saved later (a Blueprint's component template keeps its own): not waited
	// for either, so that the base stays where it is.
	Holonomic->PrimaryComponentTick.TickGroup = TG_PostPhysics;
	R.Level->SetAssistEnabled(true);
	R.Frame();
	Ordered(TEXT("on, with a late drive mode"));
	TestFalse(TEXT("a drive mode ticking after physics is not waited for"), FLevelRig::TicksAfter(R.Level, Holonomic));
	return true;
}

// Catches: a crossing refused while the level assist is on its way out (switched off, the base still being walked back
// to its stance) that leaves the correction in the legs' commands and then applies it again on top (a step of the whole
// correction, and the base left off its stance for good); and the layer kept after a crossing that did go ahead.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistRefusedYieldMidRelease,
	"Ramms.Level.Assist.RefusedYieldWhileSwitchingOffPutsAllBack",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistRefusedYieldMidRelease::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	const float Dt = 1.0f / 30.0f;
	for (const bool bRefused : { true, false })
	{
		FLevelRig R;
		if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
		{
			return false;
		}
		const TArray<FName> Ids = FLevelRig::LegMotors();
		TMap<FName, float>	Stance, Told;
		for (const FName& Id : Ids)
		{
			Stance.Add(Id, R.Baseline(Id));
		}
		R.GroundDeg = FVector2D(4.0, -2.0);
		R.PoseChassis(R.GroundDeg);
		R.Level->SetAssistEnabled(true);
		R.Frames(150);
		R.Level->SetAssistEnabled(false);
		R.Frames(8);
		const FVector2D Correction = R.CorrectionDeg();
		if (!TestTrue(TEXT("switched off on the slope: on its way out, most of the correction still on"), R.State() == EState::Releasing && Correction.Size() > 2.5))
		{
			return false;
		}
		for (const FName& Id : Ids)
		{
			TestTrue(*FString::Printf(TEXT("%s: (its baseline is the stance's)"), *Id.ToString()), SameBits(R.Baseline(Id), Stance[Id]));
			Told.Add(Id, R.Effective(Id));
		}

		// The crossing asks for the legs: folded, and with the switch off there is nothing to stand by for.
		const int32 Before = R.Log->Pushes.Num();
		R.Level->YieldTo(R.Curb);
		TestEqual(TEXT("the yield told the motors nothing"), R.Log->Pushes.Num(), Before);
		TestTrue(TEXT("off for the taker"), R.State() == EState::Off && !R.Level->IsLevelling());
		for (const FName& Id : Ids)
		{
			TestTrue(*FString::Printf(TEXT("%s: the command the taker reads is what the motor is told"), *Id.ToString()), SameBits(R.Baseline(Id), Told[Id]));
			TestTrue(*FString::Printf(TEXT("%s: no offset left"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
		}

		if (bRefused)
		{
			// Refused before it began: the baselines, the offsets and the walk back to the stance exactly as they
			// were, and the walk goes on with no step.
			R.Level->ResumeFrom(R.Curb, nullptr);
			TestEqual(TEXT("the refusal told the motors nothing"), R.Log->Pushes.Num(), Before);
			for (const FName& Id : Ids)
			{
				TestTrue(*FString::Printf(TEXT("%s: its baseline back at the stance, to the bit"), *Id.ToString()), SameBits(R.Baseline(Id), Stance[Id]));
				TestTrue(*FString::Printf(TEXT("%s: told the same, to the bit"), *Id.ToString()), SameBits(R.Effective(Id), Told[Id]));
			}
			TestTrue(TEXT("on its way out again, as before"), R.State() == EState::Releasing && R.Level->IsLevelling());
			FVector2D Last = R.CorrectionDeg();
			double	  MostStepDeg = 0.0;
			for (int32 Index = 0; Index < 6; ++Index)
			{
				R.Frame(Dt);
				MostStepDeg = FMath::Max(MostStepDeg, (R.CorrectionDeg() - Last).Size());
				Last = R.CorrectionDeg();
			}
			AddInfo(FString::Printf(TEXT("refused mid-release with %.2f deg still on: the legs' plane moved at most %.3f deg in a frame after"), Correction.Size(), MostStepDeg));
			TestTrue(TEXT("the frames after: the base goes on at the tilt policy's rate, no step"), MostStepDeg <= 3.0 * Dt + 0.02);
			TestTrue(TEXT("... towards the stance"), R.CorrectionDeg().Size() < Correction.Size());
			R.Frames(240);
			TestTrue(TEXT("off"), R.State() == EState::Off && !R.Level->IsLevelling());
			for (const FName& Id : Ids)
			{
				TestTrue(*FString::Printf(TEXT("%s: no offset left"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
				TestTrue(*FString::Printf(TEXT("%s: its baseline still the stance's, to the bit"), *Id.ToString()), SameBits(R.Baseline(Id), Stance[Id]));
				TestTrue(*FString::Printf(TEXT("%s: told exactly its baseline"), *Id.ToString()), SameBits(R.Log->LastFor(Id), R.Baseline(Id)));
			}
			TestTrue(TEXT("the base is back on its stance"), R.CorrectionDeg().Size() <= 1.0e-6);
		}
		else
		{
			// It goes ahead: the taker's mode has the robot and writes. From the level assist's next tick the layer
			// is given up (a command is then told at once), and what the taker read and wrote stands.
			static_cast<IRammsDriveMode*>(R.Curb)->SetDriveModeActive(true);
			const float TakersCommand = R.Baseline(CrankFL) + 0.05f;
			R.Base->SetMotorCommand(CrankFL, TakersCommand);
			R.Level->TickComponent(Dt, LEVELTICK_All, nullptr);
			R.Tick();
			TestTrue(TEXT("still off"), R.State() == EState::Off && !R.Level->IsLevelling());
			TestTrue(TEXT("the taker's command is what its motor is told"), SameBits(R.Log->LastFor(CrankFL), TakersCommand));
			TestTrue(TEXT("a motor it has not written keeps the folded command"), SameBits(R.Baseline(CrankRR), Told[CrankRR]) && SameBits(R.Effective(CrankRR), Told[CrankRR]));
			const int32 Count = R.Log->CountFor(CrankRL);
			R.Base->SetMotorCommand(CrankRL, R.Baseline(CrankRL));
			TestEqual(TEXT("the layer is given up: a leg's command is told at once"), R.Log->CountFor(CrankRL), Count + 1);
			TestTrue(TEXT("... and no tick order is left behind"), R.Level->PrimaryComponentTick.GetPrerequisites().IsEmpty() && !FLevelRig::TicksAfter(R.Base, R.Level));
			// A refusal now (it has written) puts nothing back.
			R.Level->ResumeFrom(R.Curb, nullptr);
			TestTrue(TEXT("a hand-back after it wrote leaves its command"), SameBits(R.Baseline(CrankFL), TakersCommand));
		}
	}
	return true;
}

// Catches: a taker that yields before its drive mode is the active one taken for gone on the very next tick (levelling
// would start again under the crossing), or never taken for gone (levelling would stand by for ever after a take-over
// that never happened); and a drive mode it does not level in that leaves the legs alone being handed them at once
// (a step) instead of the base being walked back to its stance.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistTakerGrace,
	"Ramms.Level.Assist.TakerGraceAndGentleRelease",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistTakerGrace::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	const float			Dt = 1.0f / 30.0f;
	const TArray<FName> Ids = FLevelRig::LegMotors();
	{
		FLevelRig R;
		if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
		{
			return false;
		}
		R.GroundDeg = FVector2D(-3.0, 2.0);
		R.PoseChassis(R.GroundDeg);
		R.Level->SetAssistEnabled(true);
		R.Frames(150);
		if (!TestTrue(TEXT("levelled on the slope, offsets on"), R.State() == EState::Active && FMath::Abs(R.LegDz(0)) > 1.0))
		{
			return false;
		}
		TMap<FName, float> Told;
		for (const FName& Id : Ids)
		{
			Told.Add(Id, R.Effective(Id));
		}
		// (The chassis is left posed level through this: the legs' plane is in their commands after the fold.)
		auto TickOnly = [&]() {
			R.Level->TickComponent(Dt, LEVELTICK_All, nullptr);
			R.Tick();
		};
		auto Unmoved = [&](const TCHAR* When) {
			for (const FName& Id : Ids)
			{
				TestTrue(*FString::Printf(TEXT("%s: %s has no offset"), When, *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
				TestTrue(*FString::Printf(TEXT("%s: %s is told the same, to the bit"), When, *Id.ToString()), SameBits(R.Effective(Id), Told[Id]));
			}
		};
		IRammsDriveMode* CurbMode = R.Curb;

		// It yields; its drive mode is not the active one yet. It has the level assist's next tick to become so.
		R.Level->YieldTo(R.Curb);
		TickOnly();
		TestTrue(TEXT("the first tick after a yield: still standing by, though the taker's mode is not active yet"), R.State() == EState::Yielded);
		Unmoved(TEXT("the first tick"));
		// It never did: on the second tick levelling starts again, cold, from the commands as the fold left them.
		TickOnly();
		TestTrue(TEXT("the second tick, the taker's mode never active: levelling again"), R.State() == EState::Active);
		Unmoved(TEXT("the second tick"));

		// It yields again and its mode is active by the second tick: standing by for as long as it is.
		R.Level->YieldTo(R.Curb);
		TickOnly();
		TestTrue(TEXT("yielded again"), R.State() == EState::Yielded);
		CurbMode->SetDriveModeActive(true);
		for (int32 Index = 0; Index < 5; ++Index)
		{
			TickOnly();
			TestTrue(TEXT("standing by while the taker's mode has the robot"), R.State() == EState::Yielded);
		}
		Unmoved(TEXT("standing by"));
	}
	{
		// A drive mode it does not level in, which leaves the legs alone: the base is walked back to its stance.
		FLevelRig R;
		if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
		{
			return false;
		}
		URammsHolonomicDriveController* Holonomic = NewObject<URammsHolonomicDriveController>(R.Actor, TEXT("Holonomic"));
		IRammsDriveMode*				HolonomicMode = Holonomic;
		// (A drive mode is made active: it stands down first, as under a selector that has another mode up.)
		HolonomicMode->SetDriveModeActive(false);
		R.Level->DisabledModeIds.Add(HolonomicMode->GetDriveModeId());
		R.GroundDeg = FVector2D(3.0, -2.0);
		R.PoseChassis(R.GroundDeg);
		R.Level->SetAssistEnabled(true);
		R.Frames(150);
		const FVector2D Correction = R.CorrectionDeg();
		if (!TestTrue(TEXT("levelled on the slope"), R.State() == EState::Active && Correction.Size() > 3.0))
		{
			return false;
		}
		HolonomicMode->SetDriveModeActive(true);
		TArray<FName> Claimed;
		static_cast<IRammsControlContributor*>(Holonomic)->GetClaimedMotorIds(Claimed);
		TestFalse(TEXT("(that mode drives no leg motor)"), Claimed.ContainsByPredicate([&Ids](const FName& Id) { return Ids.Contains(Id); }));
		FVector2D Last = Correction;
		double	  MostStepDeg = 0.0;
		R.Frame(Dt);
		TestTrue(TEXT("walking back to the stance, not dropped and not folded"), R.State() == EState::Releasing && R.Level->IsLevelling());
		TestEqual(TEXT("... saying why"), R.Status(), FString(TEXT("Returning to the drive stance: not levelling in this drive mode")));
		for (int32 Index = 0; Index < 240; ++Index)
		{
			MostStepDeg = FMath::Max(MostStepDeg, (R.CorrectionDeg() - Last).Size());
			Last = R.CorrectionDeg();
			R.Frame(Dt);
		}
		AddInfo(FString::Printf(TEXT("a mode it does not level in: the legs' plane moved at most %.3f deg in a frame on the way back"), MostStepDeg));
		TestTrue(TEXT("the way back is within the tilt policy's rate"), MostStepDeg > 0.01 && MostStepDeg <= 3.0 * Dt + 0.02);
		TestTrue(TEXT("then waiting, on the stance"), R.State() == EState::Suspended && !R.Level->IsLevelling() && R.CorrectionDeg().Size() <= 1.0e-6);
		TestEqual(TEXT("... saying why"), R.Status(), FString(TEXT("Waiting: not levelling in this drive mode")));
		for (const FName& Id : Ids)
		{
			TestTrue(*FString::Printf(TEXT("%s: no offset while waiting"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
		}
		// That mode stood down: levelling again.
		HolonomicMode->SetDriveModeActive(false);
		R.Frames(150);
		TestTrue(TEXT("levelling again once that mode is gone"), R.State() == EState::Active && (R.GroundDeg - R.CorrectionDeg()).Size() <= 0.05);
	}
	return true;
}

// Catches: a leg let go and taken back while the robot is level and at rest (nothing being written) told its new
// command bare, without the offset the law still believes is on it: the leg would step by its whole correction.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistLostOffset,
	"Ramms.Level.Assist.LostOffsetPutBackBeforeItIsTold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistLostOffset::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	FLevelRig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
	{
		return false;
	}
	R.GroundDeg = FVector2D(3.0, -2.0);
	R.PoseChassis(R.GroundDeg);
	R.Level->SetAssistEnabled(true);
	R.Frames(240);
	const double DzBefore = R.LegDz(2);
	if (!TestTrue(TEXT("level and at rest on the slope, the left centre leg's offset on"), R.State() == EState::Active && R.Status() == TEXT("Level, at rest") && FMath::Abs(DzBefore) > 0.5))
	{
		return false;
	}
	// The left linkage lets go of its hips (driven by hand for a moment).
	IRammsControlContributor* LeftControls = R.Left;
	LeftControls->SetContributionSuspended(true);
	int32 Before = R.LegPushes();
	R.Frames(5);
	TestTrue(TEXT("let go: no offset waits on its hips"), R.Base->GetMotorCommandOffset(HipLA) == 0.0f && R.Base->GetMotorCommandOffset(HipLB) == 0.0f);
	TestEqual(TEXT("let go: the robot is still at rest, no leg motor told anything"), R.LegPushes(), Before);

	// It takes them back at the stance. The level assist's next tick puts the offset back, and the base's tick after
	// it tells the hips once: baseline plus offset.
	LeftControls->SetContributionSuspended(false);
	const int32 FirstPush = R.Log->Pushes.Num();
	R.Left->SetEndpointTarget(FVector2D(CentreX, StanceZ));
	R.Frame();
	TestEqual(TEXT("taken back: the leg is told the height it had, offset and all"), R.LegDz(2), DzBefore, 1.0e-3);
	for (const FName& Hip : { HipLA, HipLB })
	{
		const float Bare = R.Baseline(Hip);
		TestTrue(*FString::Printf(TEXT("%s: (its offset is not nothing)"), *Hip.ToString()), FMath::Abs(R.Effective(Hip) - Bare) > 1.0e-3f);
		int32 ToldTimes = 0;
		for (int32 Index = FirstPush; Index < R.Log->Pushes.Num(); ++Index)
		{
			const RammsCommandOffsetTests::FPush& Push = R.Log->Pushes[Index];
			if (Push.Motor == Hip && (Push.Kind == RammsCommandOffsetTests::FPush::EKind::Command || Push.Kind == RammsCommandOffsetTests::FPush::EKind::Profile))
			{
				++ToldTimes;
				TestTrue(*FString::Printf(TEXT("%s: never told its command bare"), *Hip.ToString()), SameBits(Push.Value, R.Effective(Hip)));
			}
		}
		TestEqual(*FString::Printf(TEXT("%s: told once"), *Hip.ToString()), ToldTimes, 1);
	}
	TestTrue(TEXT("still levelling"), R.State() == EState::Active);
	// ... and at rest again: nothing more is written for it.
	R.Frames(60);
	Before = R.LegPushes();
	R.Frames(30);
	TestEqual(TEXT("at rest again no leg motor is told anything"), R.LegPushes(), Before);
	return true;
}

// Catches: an attitude source that is not built taken as if it were (the base would be levelled on no reading); the
// leg motors' offsets held by something else and the switch left on, or the refusal never lifted once they are free;
// and a level assist set to start on that does not, or that steps the legs on its first frame.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistOtherRefusals,
	"Ramms.Level.Assist.RefusesOtherSourcesAndAHeldLayerAndStartsOn",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistOtherRefusals::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	AddExpectedMessagePlain(TEXT("cannot be levelled: only the exact attitude source is built"), ELogVerbosity::Warning, EAutomationExpectedMessageFlags::Contains, 1);
	AddExpectedMessagePlain(TEXT("level assist stays off: only the exact attitude source is built"), ELogVerbosity::Warning, EAutomationExpectedMessageFlags::Contains, 1);
	AddExpectedMessagePlain(TEXT("cannot be levelled: the leg motors' offsets cannot be taken"), ELogVerbosity::Warning, EAutomationExpectedMessageFlags::Contains, 1);
	{
		FLevelRig R;
		if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr))
		{
			return false;
		}
		R.Level->AttitudeSource = ERammsLevelAttitudeSource::ImuSensor;
		R.Frame();
		IRammsControlContributor* Controls = R.Level;
		TestFalse(TEXT("an attitude source that is not built: the switch refuses on"), Controls->ApplyControl(RammsLevel::AssistControlId(), 1.0f));
		TestFalse(TEXT("... and stays off"), R.Level->IsAssistEnabled());
		TestEqual(TEXT("the status says why"), R.Status(), FString(TEXT("Level: only the exact attitude source is built")));
		R.Frames(3);
		R.Base->SetMotorCommand(CrankFL, 0.1f);
		TestTrue(TEXT("nothing was claimed"), SameBits(R.Log->LastFor(CrankFL), 0.1f));
	}
	{
		// Something else holds the base's offset layer when the law asks for the legs.
		FLevelRig R;
		if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr))
		{
			return false;
		}
		R.GroundDeg = FVector2D(3.0, 1.0);
		R.PoseChassis(R.GroundDeg);
		R.Frame();
		TestTrue(TEXT("another object holds the offset layer"), R.Base->ClaimCommandOffsetLayer(R.Other(), TArray<FName>{ CrankFL }));
		IRammsControlContributor* Controls = R.Level;
		TestTrue(TEXT("the switch is taken (the robot can be described)"), Controls->ApplyControl(RammsLevel::AssistControlId(), 1.0f));
		R.Frames(3);
		TestTrue(TEXT("refused: the switch is off again, nothing levelled"), !R.Level->IsAssistEnabled() && R.State() == EState::Off && !R.Level->IsLevelling());
		TestEqual(TEXT("the status says why"), R.Status(), FString(TEXT("Level: the leg motors' offsets cannot be taken (another controller holds them)")));
		TestEqual(TEXT("... as a fault"), R.Control(RammsLevel::StatusControlId()), static_cast<float>(ERammsControlTextSeverity::Fault));
		TestTrue(TEXT("the other holder still has the layer"), R.Base->SetMotorCommandOffset(CrankFL, 0.0f, R.Other()));
		for (const FName& Id : FLevelRig::LegMotors())
		{
			TestTrue(*FString::Printf(TEXT("%s: no offset was put on it"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
		}
		// The holder lets go: asked again, it levels.
		R.Base->ReleaseCommandOffsetLayer(R.Other());
		TestTrue(TEXT("asked again once the layer is free: taken"), Controls->ApplyControl(RammsLevel::AssistControlId(), 1.0f) && R.Level->IsAssistEnabled());
		R.Frames(150);
		TestTrue(TEXT("... and it levels"), R.State() == EState::Active && (R.GroundDeg - R.CorrectionDeg()).Size() <= 0.05);
	}
	{
		// Set to start on: claimed on its first tick, with offsets of exactly zero.
		FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/true, /*bStartOn=*/true);
		if (!TestTrue(TEXT("the made-up robot with a level assist set to start on"), R.IsReady() && R.Level != nullptr))
		{
			return false;
		}
		R.GroundDeg = FVector2D(3.0, 1.0);
		R.PoseChassis(R.GroundDeg);
		TestTrue(TEXT("on before its first tick"), R.Level->IsAssistEnabled());
		R.Frame();
		TestTrue(TEXT("levelling from its first tick"), R.State() == EState::Active && R.Level->IsLevelling());
		for (const FName& Id : FLevelRig::LegMotors())
		{
			TestTrue(*FString::Printf(TEXT("%s: the first frame's offset is exactly zero"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
		}
		const int32 Before = R.Log->CountFor(CrankFL);
		R.Base->SetMotorCommand(CrankFL, 0.0f);
		TestEqual(TEXT("the legs are claimed (a command waits for the base's tick)"), R.Log->CountFor(CrankFL), Before);
		FRammsControlSurface Surface;
		static_cast<const IRammsControlContributor*>(R.Level)->DescribeControls(Surface);
		if (const FRammsControlAxis* Switch = Surface.Find(RammsLevel::AssistControlId()))
		{
			TestEqual(TEXT("the switch shows on"), Switch->DefaultValue, 1.0f);
		}
		R.Frames(150);
		TestTrue(TEXT("... and it levels"), (R.GroundDeg - R.CorrectionDeg()).Size() <= 0.05);
	}
	return true;
}

// Catches: the slope-past-reach status measured against the horizontal when the rider has asked for a tilt of their
// own (it would read less than they are really off what they asked for, or nothing at all).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistPastReachAgainstPreference,
	"Ramms.Level.Assist.PastReachIsMeasuredAgainstThePreference",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistPastReachAgainstPreference::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	FLevelRig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
	{
		return false;
	}
	// The legs may turn the base 2 deg at most; the ground is 6 deg nose up and the rider asks for 3 deg nose down:
	// 9 deg asked of 2. The base ends 4 deg nose up: inside the tilt policy's 5 deg off the horizontal, and 7 deg off
	// what was asked, which is past it.
	R.Level->Tuning.MaxCorrectionDeg = FVector2D(2.0, 2.0);
	R.GroundDeg = FVector2D(6.0, 0.0);
	R.PoseChassis(R.GroundDeg);
	IRammsControlContributor* Controls = R.Level;
	TestTrue(TEXT("a preferred pitch, nose down"), Controls->ApplyControl(RammsLevel::PreferencePitchControlId(), -3.0f));
	R.Level->SetAssistEnabled(true);
	R.Frames(300);
	const FVector2D Attitude = R.GroundDeg - R.CorrectionDeg();
	AddInfo(FString::Printf(TEXT("6 deg ground, -3 deg preferred, 2 deg of correction allowed: attitude (%.2f, %.2f), status '%s', level.tilt %.2f, level.reach %.2f"), Attitude.X,
		Attitude.Y, *R.Status(), R.Control(RammsLevel::TiltControlId()), R.Control(RammsLevel::ReachControlId())));
	TestTrue(TEXT("the legs gave what they may"), Attitude.Equals(FVector2D(4.0, 0.0), 0.05));
	TestTrue(TEXT("(off the horizontal the base is inside the tilt policy)"), RammsLevel::Inclination(Attitude) < R.Level->Tuning.MaxTiltDeg - 0.5);
	TestEqual(TEXT("the status names how far the base is off what the rider asked, not off the horizontal, and what stops the legs"), R.Status(),
		FString(TEXT("Slope past levelling reach: 7 deg (the most correction allowed)")));
	TestEqual(TEXT("level.reach reads none left"), static_cast<double>(R.Control(RammsLevel::ReachControlId())), 0.0, 0.01);
	TestEqual(TEXT("... and asks for attention"), R.Control(RammsLevel::StatusControlId()), static_cast<float>(ERammsControlTextSeverity::Attention));
	TestEqual(TEXT("level.tilt is the base's own tilt"), static_cast<double>(R.Control(RammsLevel::TiltControlId())), 4.0, 0.05);
	return true;
}

namespace RammsLevelAssistTests
{
	/** The stance the rig's legs stand in (FLevelRig::Stand), as a drive mode names its own. */
	FRammsDriveStance RigStance()
	{
		FRammsDriveStance Stance;
		for (const FName& Crank : { CrankFL, CrankFR, CrankRL, CrankRR })
		{
			FRammsStanceMotorTarget& Target = Stance.MotorTargets.AddDefaulted_GetRef();
			Target.MotorId = Crank;
			Target.Target = 0.0f;
		}
		Stance.bHasLinkageHeight = true;
		Stance.LinkageHeightCm = static_cast<float>(StanceZ);
		Stance.bHasLinkageTranslation = true;
		Stance.LinkageTranslationCm = static_cast<float>(CentreX);
		return Stance;
	}

	/** The plane by which the corner legs, as their motors are told, differ from the stance (deg; pitch, roll): what
	 *  the whole pose turns the base by, whoever commanded it and however it is split between command and offset. */
	FVector2D PoseDeg(const FLevelRig& R)
	{
		auto		 Up = [&R](int32 Leg) { return R.LegZ(Leg, true) - StanceZ; };
		const double Pitch = FMath::Atan(((Up(0) + Up(1)) - (Up(4) + Up(5))) / 2.0 / (2.0 * CornerX));
		const double Roll = FMath::Atan(((Up(1) + Up(5)) - (Up(0) + Up(4))) / 2.0 / (2.0 * CornerY));
		return FVector2D(FMath::RadiansToDegrees(Pitch), FMath::RadiansToDegrees(Roll));
	}

	/** One frame with the chassis answering the legs' whole pose: the level assist's tick, the base's, the chassis
	 *  where the ground and the pose put it. (FLevelRig::Frame answers the offsets alone, which is the same thing
	 *  for as long as every baseline is the stance's.) */
	void FrameOnPose(const FLevelRig& R, float Dt = 1.0f / 30.0f)
	{
		R.Level->TickComponent(Dt, LEVELTICK_All, nullptr);
		R.Tick(Dt);
		R.PoseChassis(R.GroundDeg - PoseDeg(R));
	}

	/** Every leg drawn Cm further up from where its motors are told to be, as a controller that has the legs commands
	 *  them: the base that much lower, its attitude as it was. */
	void LowerBase(const FLevelRig& R, double Cm)
	{
		const FName Cranks[4] = { CrankFL, CrankFR, CrankRL, CrankRR };
		const int32 CrankLegs[4] = { 0, 1, 4, 5 };
		for (int32 Index = 0; Index < 4; ++Index)
		{
			bool		bReachable = false;
			float		WheelX = 0.0f;
			const float Crank = URamms4BarKinematics::SolveCrankForHeight(CornerLeg(Cranks[Index], 0.0), static_cast<float>(R.LegZ(CrankLegs[Index], true) + Cm), bReachable, WheelX);
			R.Base->SetMotorCommand(Cranks[Index], Crank);
		}
		R.Left->SetEndpointTarget(FVector2D(CentreX, R.LegZ(2, true) + Cm));
		R.Right->SetEndpointTarget(FVector2D(CentreX, R.LegZ(3, true) + Cm));
	}

	/** The largest difference (cm) between the six legs' baseline heights over the stance: 0 when the baselines are
	 *  the stance moved up or down as a whole, with no tilt left in them. */
	double BaselineSpreadCm(const FLevelRig& R)
	{
		double Low = R.LegZ(0, false), High = Low;
		for (int32 Leg = 1; Leg < 6; ++Leg)
		{
			Low = FMath::Min(Low, R.LegZ(Leg, false));
			High = FMath::Max(High, R.LegZ(Leg, false));
		}
		return High - Low;
	}
} // namespace RammsLevelAssistTests

// Catches: a hand-back to a stance that starts levelling again from nothing (the drive mode's stance ramp then turns
// the base to the ground's attitude and levelling turns it back); a hand-back that tells a motor anything, or changes
// what one is told; baselines left holding part of the tilt; the 5-bar controllers left holding the taker's targets;
// a switched-off level assist that touches the legs at a hand-back; and a take-over nobody announced not handed back.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistHandBackAdopts,
	"Ramms.Level.Assist.HandBackAdoptsThePose",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistHandBackAdopts::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	const TArray<FName>		Ids = FLevelRig::LegMotors();
	const FRammsDriveStance Stance = RigStance();
	{
		FLevelRig R;
		if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
		{
			return false;
		}
		R.GroundDeg = FVector2D(-3.0, 2.0);
		R.PoseChassis(R.GroundDeg);
		R.Level->SetAssistEnabled(true);
		for (int32 Index = 0; Index < 150; ++Index)
		{
			FrameOnPose(R);
		}
		const FVector2D Levelled = R.CorrectionDeg();
		if (!TestTrue(TEXT("levelled on the slope"), R.State() == EState::Active && (R.GroundDeg - Levelled).Size() <= 0.1))
		{
			return false;
		}

		// A take-over that goes ahead: the taker has the legs, and leaves the base level but 1.5 cm lower.
		R.Level->YieldTo(R.Curb);
		IRammsDriveMode* CurbMode = R.Curb;
		CurbMode->SetDriveModeActive(true);
		LowerBase(R, 1.5);
		for (int32 Index = 0; Index < 10; ++Index)
		{
			FrameOnPose(R);
		}
		TestTrue(TEXT("standing by, the base level in the taker's pose"), R.State() == EState::Yielded && (R.GroundDeg - PoseDeg(R)).Size() <= 0.1 && R.CorrectionDeg().Size() <= 1.0e-6);
		TMap<FName, float> Told;
		for (const FName& Id : Ids)
		{
			Told.Add(Id, R.Effective(Id));
		}

		// The hand-back to the drive mode's stance: the plane back out of the commands, nothing told.
		const int32 Before = R.Log->Pushes.Num();
		R.Level->ResumeFrom(R.Curb, &Stance);
		TestEqual(TEXT("the hand-back told the motors nothing"), R.Log->Pushes.Num(), Before);
		for (const FName& Id : Ids)
		{
			TestTrue(*FString::Printf(TEXT("%s: told the same, to the bit"), *Id.ToString()), SameBits(R.Effective(Id), Told[Id]));
		}
		TestTrue(TEXT("levelling again at once"), R.State() == EState::Active && R.Level->IsLevelling());
		const FVector2D Adopted = R.CorrectionDeg();
		AddInfo(FString::Printf(TEXT("hand-back on (-3, 2) deg: the offsets carry (%.3f, %.3f) deg; the baselines are the stance within %.3f cm, %.2f cm up"), Adopted.X, Adopted.Y, BaselineSpreadCm(R),
			R.LegZ(4, false) - StanceZ));
		TestTrue(TEXT("the offsets carry the plane that holds the base level"), (Adopted - R.GroundDeg).Size() <= 0.1);
		TestTrue(TEXT("the baselines are left the stance, 1.5 cm up: a height for the stance ramp, and no tilt"), BaselineSpreadCm(R) <= 0.1 && FMath::IsNearlyEqual(R.LegZ(4, false) - StanceZ, 1.5, 0.1));
		TestTrue(TEXT("the wanted attitude is the attitude the base has"), (R.Level->GetTargetAttitudeDeg() - (R.GroundDeg - PoseDeg(R))).Size() <= 0.05);
		{
			// The 5-bar controller holds the baseline, not the taker's target: the stance ramp starts from there.
			float							Z = NAN;
			const IRammsControlContributor* Contributor = R.Left;
			TestTrue(TEXT("the linkage holds a target"), Contributor->ReadTarget(RammsControlIds::Linkage::Height(R.Left->GetName()), Z));
			TestEqual(TEXT("... its baseline's height"), static_cast<double>(Z), R.LegZ(2, false), 0.02);
		}

		// The selector stands the taker down and ramps the stance in; the base stays level through it.
		CurbMode->SetDriveModeActive(false);
		FrameOnPose(R);
		double Step = 0.0;
		for (const FName& Id : Ids)
		{
			Step = FMath::Max(Step, static_cast<double>(FMath::Abs(R.Effective(Id) - Told[Id])));
		}
		AddInfo(FString::Printf(TEXT("frame after the hand-back: the motors' commands moved %.5f rad at most"), Step));
		// (A crank offset of 0.13 rad was taken back out: a wheel's place fore and aft, taken at its baseline, moved a
		// centimetre with it, which over this slope is 0.003 rad of crank if the offsets are not worked out again
		// over the legs as the rebase left them.)
		TestTrue(TEXT("no motor's command stepped on the frame after"), Step <= 5.0e-4);
		TMap<FName, float> From;
		for (const FName& Crank : { CrankFL, CrankFR, CrankRL, CrankRR })
		{
			From.Add(Crank, R.Baseline(Crank));
		}
		const double FromLeft = R.LegZ(2, false), FromRight = R.LegZ(3, false);
		double		 MostOff = 0.0;
		for (int32 Index = 1; Index <= 30; ++Index)
		{
			const double Share = Index / 30.0;
			for (const TPair<FName, float>& Crank : From)
			{
				R.Base->SetMotorCommand(Crank.Key, Index == 30 ? 0.0f : static_cast<float>(Crank.Value * (1.0 - Share)));
			}
			R.Left->SetEndpointTarget(FVector2D(CentreX, FMath::Lerp(FromLeft, StanceZ, Share)));
			R.Right->SetEndpointTarget(FVector2D(CentreX, FMath::Lerp(FromRight, StanceZ, Share)));
			FrameOnPose(R);
			MostOff = FMath::Max(MostOff, (R.GroundDeg - PoseDeg(R)).Size());
		}
		AddInfo(FString::Printf(TEXT("the stance ramped in under the adopted plane: the base at most %.3f deg off level"), MostOff));
		TestTrue(TEXT("level through the stance ramp"), MostOff <= 0.2);
		for (const FName& Crank : { CrankFL, CrankFR, CrankRL, CrankRR })
		{
			TestTrue(*FString::Printf(TEXT("%s: its baseline is the stance's"), *Crank.ToString()), SameBits(R.Baseline(Crank), 0.0f));
		}
		for (int32 Index = 0; Index < 90; ++Index)
		{
			FrameOnPose(R);
		}
		TestTrue(TEXT("level on the stance, the offsets holding it"), (R.GroundDeg - R.CorrectionDeg()).Size() <= 0.1 && BaselineSpreadCm(R) <= 0.02);
		TestEqual(TEXT("at rest"), R.Status(), FString(TEXT("Level, at rest")));
	}
	{
		// Switched off: a hand-back changes nothing at all.
		FLevelRig R;
		if (!TestTrue(TEXT("the made-up robot, the level assist off"), R.IsReady() && R.Level != nullptr))
		{
			return false;
		}
		R.GroundDeg = FVector2D(-3.0, 2.0);
		R.PoseChassis(R.GroundDeg);
		R.Frames(5);
		R.Level->YieldTo(R.Curb);
		LowerBase(R, 1.5);
		R.Base->SetMotorCommand(CrankFL, R.Baseline(CrankFL) + 0.05f);
		R.Tick();
		TMap<FName, float> Was;
		for (const FName& Id : Ids)
		{
			Was.Add(Id, R.Baseline(Id));
		}
		const int32 Before = R.Log->Pushes.Num();
		R.Level->ResumeFrom(R.Curb, &Stance);
		R.Frames(3);
		TestEqual(TEXT("off: the hand-back and the frames after told the motors nothing"), R.Log->Pushes.Num(), Before);
		for (const FName& Id : Ids)
		{
			TestTrue(*FString::Printf(TEXT("%s: off: its command as the taker left it, to the bit, and no offset"), *Id.ToString()), SameBits(R.Baseline(Id), Was[Id]) && R.Base->GetMotorCommandOffset(Id) == 0.0f);
		}
		TestTrue(TEXT("off: still off"), R.State() == EState::Off && !R.Level->IsLevelling());
		const int32 Count = R.Log->CountFor(CrankFL);
		R.Base->SetMotorCommand(CrankFL, 0.0f);
		TestEqual(TEXT("off: a leg's command is still told at once (nothing was claimed)"), R.Log->CountFor(CrankFL), Count + 1);
	}
	{
		// A mode that took the legs with nobody announcing it (the tick's backstop folded for it) hands back to a
		// stance: the plane is taken back out all the same.
		FLevelRig R;
		if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
		{
			return false;
		}
		AddExpectedMessagePlain(TEXT("took the legs without yielding"), ELogVerbosity::Warning, EAutomationExpectedMessageFlags::Contains, 1);
		R.GroundDeg = FVector2D(3.0, -2.0);
		R.PoseChassis(R.GroundDeg);
		R.Level->SetAssistEnabled(true);
		R.Frames(150);
		const FVector2D	 Levelled = R.CorrectionDeg();
		IRammsDriveMode* CurbMode = R.Curb;
		CurbMode->SetDriveModeActive(true);
		R.Level->TickComponent(1.0f / 30.0f, LEVELTICK_All, nullptr);
		R.Tick();
		TMap<FName, float> Told;
		for (const FName& Id : Ids)
		{
			Told.Add(Id, R.Effective(Id));
		}
		if (TestTrue(TEXT("unannounced: folded, standing by"), R.State() == EState::Yielded && R.CorrectionDeg().Size() <= 1.0e-6))
		{
			// (Another object's hand-back is nobody's: only a drive mode that has the legs may give them back.)
			R.Level->ResumeFrom(R.Other(), &Stance);
			TestTrue(TEXT("unannounced: a stranger's hand-back changes nothing"), R.State() == EState::Yielded);
			R.Level->ResumeFrom(R.Curb, &Stance);
			TestTrue(TEXT("unannounced: handed back, levelling again"), R.State() == EState::Active);
			TestTrue(TEXT("unannounced: the plane is back in the offsets"), (R.CorrectionDeg() - Levelled).Size() <= 0.1 && BaselineSpreadCm(R) <= 0.1);
			for (const FName& Id : Ids)
			{
				TestTrue(*FString::Printf(TEXT("%s: unannounced: told the same, to the bit"), *Id.ToString()), SameBits(R.Effective(Id), Told[Id]));
			}
		}
	}
	return true;
}

// Catches: the curb controller handing the robot back to a drive mode without telling the level assist (levelling then
// starts again from nothing under that mode's stance ramp), or telling it after the mode has been switched.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistCurbHandsBack,
	"Ramms.Level.Assist.CurbControllerHandsBackThroughIt",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistCurbHandsBack::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	FLevelRig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
	{
		return false;
	}
	AddExpectedMessagePlain(TEXT("took the legs without yielding"), ELogVerbosity::Warning, EAutomationExpectedMessageFlags::Contains, 1);
	// A drive mode to hand back to, with the rig's stance for its own.
	URammsHolonomicDriveController* Holonomic = NewObject<URammsHolonomicDriveController>(R.Actor, TEXT("Holonomic"));
	IRammsDriveMode*				HolonomicMode = Holonomic;
	Holonomic->LiftedStance = RigStance();
	HolonomicMode->SetDriveModeActive(false);
	HolonomicMode->SetDriveModeActive(true);
	R.GroundDeg = FVector2D(3.0, -2.0);
	R.PoseChassis(R.GroundDeg);
	R.Level->SetAssistEnabled(true);
	R.Frames(150);
	const FVector2D		Levelled = R.CorrectionDeg();
	const TArray<FName> Ids = FLevelRig::LegMotors();
	if (!TestTrue(TEXT("levelled on the slope in that mode"), R.State() == EState::Active && (R.GroundDeg - Levelled).Size() <= 0.1))
	{
		return false;
	}

	// The curb mode picked by hand, with no crossing to run: it has the legs for a frame (nobody yielded: the level
	// assist's own tick folds for it) and then hands straight back.
	IRammsDriveMode* CurbMode = R.Curb;
	HolonomicMode->SetDriveModeActive(false);
	CurbMode->SetDriveModeActive(true);
	R.Level->TickComponent(1.0f / 30.0f, LEVELTICK_All, nullptr);
	R.Tick();
	TMap<FName, float> Told;
	for (const FName& Id : Ids)
	{
		Told.Add(Id, R.Effective(Id));
	}
	if (!TestTrue(TEXT("the curb mode has the legs: folded, standing by"), R.State() == EState::Yielded && CurbMode->IsDriveModeActive()))
	{
		return false;
	}
	const int32 Before = R.LegPushes();
	// (Its own tick, which the rig's other tests never run: a component must be registered to be ticked.)
	R.Curb->RegisterComponent();
	if (!TestTrue(TEXT("the curb controller is registered"), R.Curb->IsRegistered()))
	{
		return false;
	}
	R.Curb->TickComponent(1.0f / 30.0f, LEVELTICK_All, nullptr);
	TestTrue(TEXT("the curb controller handed the robot back to the mode it came from"), !CurbMode->IsDriveModeActive() && HolonomicMode->IsDriveModeActive());
	TestTrue(TEXT("... through the level assist: levelling again at once, on the plane it had"), R.State() == EState::Active && (R.CorrectionDeg() - Levelled).Size() <= 0.1);
	TestTrue(TEXT("... the baselines the stance's again"), BaselineSpreadCm(R) <= 0.1);
	TestEqual(TEXT("... no leg motor told anything by it"), R.LegPushes(), Before);
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s: told the same, to the bit"), *Id.ToString()), SameBits(R.Effective(Id), Told[Id]));
	}
	R.Frames(60);
	TestTrue(TEXT("level in that mode afterwards"), R.State() == EState::Active && (R.GroundDeg - R.CorrectionDeg()).Size() <= 0.1);
	return true;
}

namespace RammsLevelAssistTests
{
	/** The platform's pitch against the chassis (deg, nose up) that the made-up seat's motors are told (bEffective) or
	 *  commanded at their baselines to hold. */
	double SeatPitchDeg(const FLevelRig& R, bool bEffective)
	{
		const FRammsCurbSeat	   Seat = RigSeat();
		auto					   Read = [&R, bEffective](FName Id) { return static_cast<double>(bEffective ? R.Effective(Id) : R.Baseline(Id)); };
		const RammsCurb::FSeatPose Zero = RammsCurb::SeatForward(Seat, FVector2D::ZeroVector);
		const RammsCurb::FSeatPose Pose = RammsCurb::SeatForward(Seat, FVector2D(Read(SeatA), Read(SeatB)));
		return FMath::RadiansToDegrees(RammsCurb::SeatPlatformPitch(Seat, Pose.LinkAngle, Zero.LinkAngle, Read(SeatTilt)));
	}
} // namespace RammsLevelAssistTests

// Catches: the seat's tilt motor left out of the claim; a trim put on it with the wrong sign or through the wrong
// linkage angle (the platform must pitch by the trim exactly, wherever the seat's 5-bar stands: the SeatTiltFor round
// trip); a trim of nothing that is not exactly no offset; the seat's room taken about the zero pose rather than about
// where the rider has it, or without its motor's range; an offset left waiting on a tilt nothing commands; and a
// folded trim misreported: forgotten, kept after something has commanded the tilt, or not added to by a second fold.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLegPlantSeatTrim,
	"Ramms.Level.LegPlant.SeatTrimOnTheTiltMotor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLegPlantSeatTrim::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/false, /*bStartOn=*/false, /*bWithSeat=*/true);
	if (!TestTrue(TEXT("the made-up robot with a seat on the recording backend"), R.IsReady()))
	{
		return false;
	}
	TUniquePtr<RammsLevel::IPlant> Plant = RammsLevel::MakeLegPlant(*R.Base, *R.Curb, R.Layer());
	FString						   Why;
	RammsLevel::FPlantLegs		   Legs;
	TArray<FName>				   Ids;
	if (!TestTrue(TEXT("resolved"), Plant->Resolve(Why)))
	{
		return false;
	}
	Plant->LevelledMotorIds(Ids);
	TestTrue(TEXT("the robot has a seat tilt"), Plant->HasSeat());
	TestTrue(TEXT("... which is among the levelled motors"), Ids.Contains(SeatTilt) && Plant->SeatMotorId() == SeatTilt);
	TestEqual(TEXT("... with the legs' eight"), Ids.Num(), 9);
	TestTrue(TEXT("claimed"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));

	// Nothing has commanded the tilt yet: it cannot take a trim, and none is left waiting on it.
	double Own = 1.0, Low = 1.0, High = 1.0;
	TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
	TestFalse(TEXT("a tilt nothing commands takes no trim"), Plant->SeatTrimRange(Own, Low, High));
	Plant->ApplySeatTrim(3.0);
	TestTrue(TEXT("... and none is put on it"), R.Base->GetMotorCommandOffset(SeatTilt) == 0.0f);

	// Commanded at its zero pose: the platform level, the whole of its own range either way.
	R.StandSeat(0.0f, 0.0f, 0.0f);
	TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
	if (!TestTrue(TEXT("commanded, it takes a trim"), Plant->SeatTrimRange(Own, Low, High)))
	{
		return false;
	}
	TestEqual(TEXT("the rider's own pitch at the zero pose"), Own, 0.0, 1.0e-6);
	// The platform may pitch 40 deg either way. The made-up tilt motor reads against the platform's pitch and turns
	// from -0.6 to 0.9 rad: nose up it stops first, at 0.6 rad of pitch; nose down the platform does.
	TestEqual(TEXT("its room nose down: the platform's own limit"), Low, -40.0, 1.0e-4);
	TestEqual(TEXT("its room nose up: the motor's range"), High, FMath::RadiansToDegrees(0.6), 1.0e-4);

	// A trim: an offset on the tilt motor that pitches the platform by exactly that, the seat's own command untouched.
	for (const double Trim : { 3.0, -2.0 })
	{
		Plant->ApplySeatTrim(Trim);
		TestTrue(*FString::Printf(TEXT("a trim of %.0f deg: the seat's own command is not moved"), Trim), SameBits(R.Baseline(SeatTilt), 0.0f));
		// (The made-up tilt motor reads against the platform's pitch.)
		TestEqual(*FString::Printf(TEXT("a trim of %.0f deg: the tilt motor's offset"), Trim), static_cast<double>(R.Base->GetMotorCommandOffset(SeatTilt)), -FMath::DegreesToRadians(Trim), 1.0e-6);
		TestEqual(*FString::Printf(TEXT("a trim of %.0f deg: the platform pitched by it"), Trim), SeatPitchDeg(R, true) - SeatPitchDeg(R, false), Trim, 1.0e-4);
	}
	Plant->ApplySeatTrim(0.0);
	TestTrue(TEXT("no trim is exactly no offset"), R.Base->GetMotorCommandOffset(SeatTilt) == 0.0f);

	// The seat tilted by its rider: the room is about where they have it.
	R.StandSeat(0.0f, 0.0f, 0.3f);
	TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
	TestTrue(TEXT("the seat as its rider has it takes a trim"), Plant->SeatTrimRange(Own, Low, High));
	TestEqual(TEXT("the rider's own pitch is what the seat's commands hold: 0.3 rad nose down"), Own, -FMath::RadiansToDegrees(0.3), 1.0e-4);
	TestEqual(TEXT("its room nose down: what is left of the platform's 40 deg"), Low, -40.0 - Own, 1.0e-4);
	TestEqual(TEXT("its room nose up: what is left of the motor's range"), High, FMath::RadiansToDegrees(0.9), 1.0e-3);

	// ... and with its 5-bar moved too: the trim still pitches the platform by exactly itself.
	R.StandSeat(0.25f, -0.15f, 0.3f);
	TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
	TestTrue(TEXT("the seat moved by its rider takes a trim"), Plant->SeatTrimRange(Own, Low, High));
	TestEqual(TEXT("the rider's own pitch is what the seat's commands hold"), Own, SeatPitchDeg(R, false), 1.0e-6);
	TestTrue(TEXT("(the link has turned: the pitch is not the tilt motor's alone)"), FMath::Abs(Own + FMath::RadiansToDegrees(0.3)) > 1.0);
	Plant->ApplySeatTrim(4.0);
	TestEqual(TEXT("moved by its rider: the platform pitched by the trim"), SeatPitchDeg(R, true) - SeatPitchDeg(R, false), 4.0, 1.0e-4);
	// (The legs carrying the law's offsets too: none.)
	const double NoDz[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	Plant->ApplyOffsets(NoDz);
	TestFalse(TEXT("the offset is as it was left"), Plant->OffsetsLost());
	// Something else took it off: said, so that it is put back.
	R.Base->SetMotorCommandOffset(SeatTilt, 0.0f, R.Layer());
	TestTrue(TEXT("the seat's trim taken off by something else is noticed"), Plant->OffsetsLost());
	Plant->ApplySeatTrim(4.0);
	R.Tick();

	// A fold of the legs leaves the seat alone: whoever takes the legs has not taken the seat.
	const float Told = R.Effective(SeatTilt);
	const float Before = R.Baseline(SeatTilt);
	const float OffsetOn = R.Base->GetMotorCommandOffset(SeatTilt);
	TestEqual(TEXT("before a fold nothing is folded"), Plant->FoldedSeatTrimDeg(), 0.0, 0.0);
	TestEqual(TEXT("... and the seat carries its trim"), Plant->SeatTrimOnDeg(), 4.0, 1.0e-4);
	const int32 Pushes = R.Log->CountFor(SeatTilt);
	Plant->Fold();
	R.Tick();
	TestEqual(TEXT("the legs' fold told the tilt motor nothing"), R.Log->CountFor(SeatTilt), Pushes);
	TestTrue(TEXT("the legs' fold leaves the seat's command and its trim as they were, to the bit"), SameBits(R.Baseline(SeatTilt), Before) && SameBits(R.Base->GetMotorCommandOffset(SeatTilt), OffsetOn));
	TestEqual(TEXT("... and nothing of the trim is in the seat's command"), Plant->FoldedSeatTrimDeg(), 0.0, 0.0);
	TestTrue(TEXT("the legs' fold undone with a trim on the seat"), Plant->Unfold());
	TestTrue(TEXT("... which is on it still"), SameBits(R.Base->GetMotorCommandOffset(SeatTilt), OffsetOn));

	// The seat taken too: the trim into the tilt's own command, the motor told nothing, and the taker told how much.
	Plant->FoldSeatTrim();
	R.Tick();
	TestEqual(TEXT("the seat's fold told the tilt motor nothing"), R.Log->CountFor(SeatTilt), Pushes);
	TestTrue(TEXT("folded: the tilt's command is what it was told"), SameBits(R.Baseline(SeatTilt), Told) && R.Base->GetMotorCommandOffset(SeatTilt) == 0.0f);
	TestEqual(TEXT("folded: the trim that went into the command"), Plant->FoldedSeatTrimDeg(), 4.0, 1.0e-4);
	TestEqual(TEXT("folded: the seat carries none as an offset"), Plant->SeatTrimOnDeg(), 0.0, 0.0);
	// Folded again with nothing on: still in there. With more on: added to.
	Plant->FoldSeatTrim();
	TestEqual(TEXT("folded again with nothing on: the first fold's trim is in the command still"), Plant->FoldedSeatTrimDeg(), 4.0, 1.0e-4);
	TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
	TestTrue(TEXT("ready"), Plant->SeatTrimRange(Own, Low, High));
	Plant->ApplySeatTrim(1.0);
	R.Tick();
	const int32 PushesFolded = R.Log->CountFor(SeatTilt);
	Plant->FoldSeatTrim();
	TestEqual(TEXT("a second trim folded on top of the first: both are in the command"), Plant->FoldedSeatTrimDeg(), 5.0, 1.0e-4);
	// Handed back by a taker that never wrote the tilt after all: taken back out, the motor told nothing.
	const float ToldFolded = R.Effective(SeatTilt);
	TestTrue(TEXT("the folded trim taken back out"), Plant->UnfoldSeatTrim());
	TestEqual(TEXT("taken back out: the seat's command is the rider's own again"), static_cast<double>(R.Baseline(SeatTilt)), static_cast<double>(Before), 1.0e-6);
	TestTrue(TEXT("... what the tilt is told the same, to the bit"), SameBits(R.Effective(SeatTilt), ToldFolded));
	TestEqual(TEXT("... the trim its offset again"), Plant->SeatTrimOnDeg(), 5.0, 1.0e-4);
	TestEqual(TEXT("... and nothing folded any more"), Plant->FoldedSeatTrimDeg(), 0.0, 0.0);
	TestFalse(TEXT("nothing more to take back out"), Plant->UnfoldSeatTrim());
	R.Tick();
	TestEqual(TEXT("... and the tilt motor told nothing by either"), R.Log->CountFor(SeatTilt), PushesFolded);
	// The taker commands the tilt: the trim is no longer in its command, and cannot be taken back out.
	Plant->FoldSeatTrim();
	R.Base->SetMotorCommand(SeatTilt, 0.1f);
	TestEqual(TEXT("the taker commanded the tilt: nothing of the trim is in its command"), Plant->FoldedSeatTrimDeg(), 0.0, 0.0);
	TestFalse(TEXT("... and there is none to take back out"), Plant->UnfoldSeatTrim());
	TestTrue(TEXT("... the taker's command stands"), SameBits(R.Baseline(SeatTilt), 0.1f));

	// Nose down: the command without the folded trim is the rider's own setting.
	R.StandSeat(0.25f, -0.15f, Before);
	TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
	TestTrue(TEXT("ready"), Plant->SeatTrimRange(Own, Low, High));
	Plant->ApplySeatTrim(-3.0);
	Plant->FoldSeatTrim();
	TestEqual(TEXT("folded: nose down this time"), Plant->FoldedSeatTrimDeg(), -3.0, 1.0e-4);
	TestEqual(TEXT("... and the command without it is the rider's own setting"),
		RammsCurb::SeatTiltLessTrim(RigSeat(), R.Baseline(SeatTilt), FMath::DegreesToRadians(Plant->FoldedSeatTrimDeg())), static_cast<double>(Before), 1.0e-6);
	TestTrue(TEXT("taken back out"), Plant->UnfoldSeatTrim());
	Plant->ApplySeatTrim(0.0);

	// The backstop (a mode took the legs and nobody said): the seat's trim is neither folded nor dropped, whether its
	// own command stands still or something goes on writing it (the seat's controller rewrites it every tick).
	for (const bool bRewritten : { false, true })
	{
		TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
		TestTrue(TEXT("ready"), Plant->SeatTrimRange(Own, Low, High));
		Plant->ApplySeatTrim(2.0);
		const double Dz[6] = { 0.5, 0.5, 0.0, 0.0, -0.5, -0.5 };
		Plant->ApplyOffsets(Dz);
		R.Tick();
		if (bRewritten)
		{
			R.Base->SetMotorCommand(SeatTilt, Before + 1.0e-4f);
		}
		TestTrue(TEXT("measured again"), Plant->MeasureLegs(Legs));
		const float SeatOffset = R.Base->GetMotorCommandOffset(SeatTilt);
		const float SeatCommand = R.Baseline(SeatTilt);
		int32		Folded = 0, Zeroed = 0;
		Plant->FoldOrZero(Folded, Zeroed);
		const TCHAR* const Case = bRewritten ? TEXT("the seat's command rewritten") : TEXT("the seat's command left alone");
		TestTrue(*FString::Printf(TEXT("the backstop, %s: the legs' offsets are folded (%d, %d)"), Case, Folded, Zeroed), Folded == 4 && Zeroed == 0 && R.Base->GetMotorCommandOffset(CrankFL) == 0.0f);
		TestTrue(*FString::Printf(TEXT("the backstop, %s: the seat's trim stays on it, its command as it was, to the bit"), Case),
			SameBits(R.Base->GetMotorCommandOffset(SeatTilt), SeatOffset) && SameBits(R.Baseline(SeatTilt), SeatCommand));
		TestEqual(*FString::Printf(TEXT("the backstop, %s: nothing of the trim is in the seat's command"), Case), Plant->FoldedSeatTrimDeg(), 0.0, 0.0);
		Plant->ApplySeatTrim(0.0);
		R.Stand();
		R.StandSeat(0.25f, -0.15f, Before);
	}
	Plant->ZeroOffsets();
	R.Base->ReleaseCommandOffsetLayer(R.Layer());

	// A seat with no tilt motor, or one that is not a position motor: no seat trim, and the legs are levelled all the same.
	for (const int32 Which : { 0, 1 })
	{
		FLevelRig Other(/*bWithCurb=*/true, /*bWithLevel=*/false, /*bStartOn=*/false, /*bWithSeat=*/true);
		Other.Curb->Seat.TiltMotor = Which == 0 ? FName() : FName(TEXT("wheel_free"));
		TUniquePtr<RammsLevel::IPlant> None = RammsLevel::MakeLegPlant(*Other.Base, *Other.Curb, Other.Layer());
		TArray<FName>				   NoneIds;
		TestTrue(TEXT("a robot whose seat cannot be trimmed still resolves"), None->Resolve(Why));
		None->LevelledMotorIds(NoneIds);
		TestTrue(TEXT("... with no seat for the law"), !None->HasSeat() && None->SeatMotorId().IsNone() && NoneIds.Num() == 8 && !None->SeatTrimRange(Own, Low, High));
	}
	return true;
}

// Catches: the rider's fore/aft preference tilting the base on a robot whose seat can hold it (or lost on one whose
// seat cannot); the lateral one sent to the seat; the seat's own command moved by the trim; the readings a rider is
// shown not telling the base's tilt from their own; a take-over that leaves the trim as an offset under the taker's
// commands, or does not say how much of it is in the tilt's command; and a refused take-over that does not put the
// seat back exactly.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistSeat,
	"Ramms.Level.Assist.SeatHoldsThePreferenceAndIsFoldedForATaker",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistSeat::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/true, /*bStartOn=*/false, /*bWithSeat=*/true);
	if (!TestTrue(TEXT("the made-up robot with a seat and a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
	{
		return false;
	}
	R.StandSeat(0.0f, 0.0f, 0.0f);
	R.GroundDeg = FVector2D(4.0, 2.0);
	R.PoseChassis(R.GroundDeg);
	IRammsControlContributor* Controls = R.Level;
	{
		FRammsControlSurface Surface;
		Controls->DescribeControls(Surface);
		TestTrue(TEXT("a robot whose seat tilts shows the rider's tilt and the seat's trim"),
			Surface.Axes.ContainsByPredicate([](const FRammsControlAxis& Axis) { return Axis.Id == RammsLevel::RiderTiltControlId() && Axis.bReadOnly; })
				&& Surface.Axes.ContainsByPredicate([](const FRammsControlAxis& Axis) { return Axis.Id == RammsLevel::SeatTrimControlId() && Axis.bReadOnly; }));
	}
	TestTrue(TEXT("a preferred tilt: back 3 deg, and 2 deg to one side"),
		Controls->ApplyControl(RammsLevel::PreferencePitchControlId(), 3.0f) && Controls->ApplyControl(RammsLevel::PreferenceRollControlId(), -2.0f));
	R.Level->SetAssistEnabled(true);
	R.Frames(300);
	const FVector2D Base = R.GroundDeg - R.CorrectionDeg();
	AddInfo(FString::Printf(TEXT("4, 2 deg of ground, (3, -2) preferred: base (%.2f, %.2f), seat pitched %.2f deg by a trim of %.2f; level.tilt %.2f, level.rider_tilt %.2f, level.seat_trim %.2f; status '%s'"), Base.X, Base.Y,
		SeatPitchDeg(R, true), SeatPitchDeg(R, true) - SeatPitchDeg(R, false), R.Control(RammsLevel::TiltControlId()), R.Control(RammsLevel::RiderTiltControlId()), R.Control(RammsLevel::SeatTrimControlId()), *R.Status()));
	TestTrue(TEXT("levelling, with the seat's tilt among the motors it holds"), R.State() == EState::Active);
	// The base: level fore and aft, the lateral preference held.
	TestEqual(TEXT("the base's pitch is level: the preference is not the legs' to hold"), Base.X, 0.0, 0.2);
	TestEqual(TEXT("the base's roll is the preferred one"), Base.Y, -2.0, 0.2);
	// The seat: the preferred pitch, as an offset over a command nobody else's has moved.
	TestTrue(TEXT("the seat's own command is as its rider left it"), SameBits(R.Baseline(SeatTilt), 0.0f));
	TestEqual(TEXT("the seat is pitched back by the preferred 3 deg"), SeatPitchDeg(R, true), 3.0, 0.2);
	// What the rider is shown.
	TestEqual(TEXT("level.seat_trim: what levelling adds to the seat"), static_cast<double>(R.Control(RammsLevel::SeatTrimControlId())), 3.0, 0.2);
	TestEqual(TEXT("level.tilt: the base's"), static_cast<double>(R.Control(RammsLevel::TiltControlId())), 2.0, 0.2);
	TestEqual(TEXT("level.rider_tilt: the rider's own"), static_cast<double>(R.Control(RammsLevel::RiderTiltControlId())), RammsLevel::Inclination(FVector2D(3.0, -2.0)), 0.25);
	TestEqual(TEXT("level.pref.pitch still reads what was asked"), static_cast<double>(R.Control(RammsLevel::PreferencePitchControlId())), 3.0, 1.0e-6);
	TestTrue(TEXT("the wanted attitude a curb controller is given is the base's"), R.Level->GetTargetAttitudeDeg().Equals(FVector2D(0.0, -2.0), 0.01));

	// The rider tilts the seat themselves (its own controls write its own command): the trim rides on top of that.
	R.Base->SetMotorCommand(SeatTilt, -0.1f);
	R.Frames(30);
	TestTrue(TEXT("the rider's own seat tilt is theirs: the command as they set it"), SameBits(R.Baseline(SeatTilt), -0.1f));
	TestEqual(TEXT("... and the preferred pitch on top of it"), SeatPitchDeg(R, true) - SeatPitchDeg(R, false), 3.0, 0.2);
	TestEqual(TEXT("... level.rider_tilt with both"), static_cast<double>(R.Control(RammsLevel::RiderTiltControlId())),
		RammsLevel::Inclination(FVector2D(SeatPitchDeg(R, true) + Base.X, -2.0)), 0.25);

	// A take-over of the legs: the seat is not taken with them. Its trim stays on it, to the bit, and is still shown.
	const float ToldBefore = R.Effective(SeatTilt);
	const float OffsetBefore = R.Base->GetMotorCommandOffset(SeatTilt);
	const int32 Pushes = R.Log->CountFor(SeatTilt);
	TestEqual(TEXT("nothing is folded while levelling"), R.Level->GetFoldedSeatTrimDeg(), 0.0f);
	R.Level->YieldTo(R.Curb);
	TestTrue(TEXT("yielded: the seat's own command and its trim as they were, to the bit"),
		R.State() == EState::Yielded && SameBits(R.Baseline(SeatTilt), -0.1f) && SameBits(R.Base->GetMotorCommandOffset(SeatTilt), OffsetBefore));
	TestEqual(TEXT("... nothing of the trim in the seat's command"), R.Level->GetFoldedSeatTrimDeg(), 0.0f);
	TestEqual(TEXT("... level.seat_trim still reads it"), static_cast<double>(R.Control(RammsLevel::SeatTrimControlId())), 3.0, 0.2);

	// Refused before it began: exactly as it was.
	R.Level->ResumeFrom(R.Curb, nullptr);
	TestTrue(TEXT("refused: levelling again"), R.State() == EState::Active);
	TestTrue(TEXT("... the seat's own command and its trim as they were, to the bit"), SameBits(R.Baseline(SeatTilt), -0.1f) && SameBits(R.Base->GetMotorCommandOffset(SeatTilt), OffsetBefore));
	TestEqual(TEXT("... and the seat told nothing"), R.Log->CountFor(SeatTilt), Pushes);
	R.Frames(10);
	TestEqual(TEXT("... the preferred pitch on the seat still, once"), SeatPitchDeg(R, true) - SeatPitchDeg(R, false), 3.0, 0.2);

	// Taken again, and this taker takes the seat too: the trim into the seat's own command, nothing told, and the
	// taker told how much.
	const float ToldAgain = R.Effective(SeatTilt);
	const int32 PushesAgain = R.Log->CountFor(SeatTilt);
	R.Level->YieldTo(R.Curb);
	R.Level->YieldSeatTo(R.Curb);
	TestTrue(TEXT("the seat taken: its trim is in its command, no offset left"), R.State() == EState::Yielded && SameBits(R.Baseline(SeatTilt), ToldAgain) && R.Base->GetMotorCommandOffset(SeatTilt) == 0.0f);
	TestEqual(TEXT("... the seat told nothing"), R.Log->CountFor(SeatTilt), PushesAgain);
	TestEqual(TEXT("... and the taker is told the trim folded into it"), static_cast<double>(R.Level->GetFoldedSeatTrimDeg()), 3.0, 0.2);
	TestEqual(TEXT("... level.seat_trim reads none"), R.Control(RammsLevel::SeatTrimControlId()), 0.0f);

	// Called off with the seat never written after all (no stance to go to): levelling goes on, the trim taken back
	// out of the seat's command and held once, nothing told.
	R.Level->ResumeFrom(R.Curb, nullptr);
	TestTrue(TEXT("called off: levelling again"), R.State() == EState::Active);
	TestEqual(TEXT("... the seat's own command the rider's again"), static_cast<double>(R.Baseline(SeatTilt)), -0.1, 1.0e-6);
	TestTrue(TEXT("... what it is told the same, to the bit"), SameBits(R.Effective(SeatTilt), ToldAgain));
	TestEqual(TEXT("... nothing folded any more"), R.Level->GetFoldedSeatTrimDeg(), 0.0f);
	TestEqual(TEXT("... the seat told nothing by the hand-back"), R.Log->CountFor(SeatTilt), PushesAgain);
	double SeatMoved = 0.0;
	for (int32 Index = 0; Index < 60; ++Index)
	{
		R.Frame();
		SeatMoved = FMath::Max(SeatMoved, static_cast<double>(FMath::Abs(R.Effective(SeatTilt) - ToldAgain)));
	}
	TestTrue(*FString::Printf(TEXT("... nor moved after it (%.5f rad at most)"), SeatMoved), SeatMoved <= 1.0e-3);
	TestEqual(TEXT("... and the preferred pitch held once, not twice"), SeatPitchDeg(R, true) - SeatPitchDeg(R, false), 3.0, 0.2);

	// Taken for good: once the taker commands the tilt, none of the trim is in its command.
	R.Level->YieldTo(R.Curb);
	R.Level->YieldSeatTo(R.Curb);
	R.Base->SetMotorCommand(SeatTilt, 0.05f);
	TestEqual(TEXT("the taker has commanded the tilt: nothing of the trim is left in it"), R.Level->GetFoldedSeatTrimDeg(), 0.0f);

	// A robot of the same kind with no seat: the fore/aft preference is the base's, as it was.
	{
		FLevelRig Plain;
		Plain.GroundDeg = FVector2D(4.0, 2.0);
		Plain.PoseChassis(Plain.GroundDeg);
		IRammsControlContributor* PlainControls = Plain.Level;
		FRammsControlSurface	  Surface;
		PlainControls->DescribeControls(Surface);
		TestFalse(TEXT("no seat: no seat readings on the panel"),
			Surface.Axes.ContainsByPredicate([](const FRammsControlAxis& Axis) { return Axis.Id == RammsLevel::RiderTiltControlId() || Axis.Id == RammsLevel::SeatTrimControlId(); }));
		PlainControls->ApplyControl(RammsLevel::PreferencePitchControlId(), 3.0f);
		PlainControls->ApplyControl(RammsLevel::PreferenceRollControlId(), -2.0f);
		Plain.Level->SetAssistEnabled(true);
		Plain.Frames(300);
		const FVector2D PlainBase = Plain.GroundDeg - Plain.CorrectionDeg();
		TestTrue(*FString::Printf(TEXT("no seat: the base holds the whole preference (%.2f, %.2f)"), PlainBase.X, PlainBase.Y), PlainBase.Equals(FVector2D(3.0, -2.0), 0.2));
		TestEqual(TEXT("no seat: the rider's tilt is the base's"), Plain.Control(RammsLevel::RiderTiltControlId()), Plain.Control(RammsLevel::TiltControlId()));
	}

	// The seat's trim switched off in the tuning: the base holds the pitch too, and the seat is left alone.
	{
		FLevelRig Off(/*bWithCurb=*/true, /*bWithLevel=*/true, /*bStartOn=*/false, /*bWithSeat=*/true);
		Off.StandSeat(0.0f, 0.0f, 0.0f);
		Off.Level->Tuning.bSeatCoversTransients = false;
		Off.GroundDeg = FVector2D(4.0, 2.0);
		Off.PoseChassis(Off.GroundDeg);
		IRammsControlContributor* OffControls = Off.Level;
		OffControls->ApplyControl(RammsLevel::PreferencePitchControlId(), 3.0f);
		Off.Level->SetAssistEnabled(true);
		Off.Frames(300);
		const FVector2D OffBase = Off.GroundDeg - Off.CorrectionDeg();
		TestEqual(TEXT("trim off in the tuning: the base holds the preferred pitch"), OffBase.X, 3.0, 0.2);
		TestTrue(TEXT("... and the seat carries no offset"), Off.Base->GetMotorCommandOffset(SeatTilt) == 0.0f);
	}
	return true;
}

// Catches: anything that keeps levelling out of a second drive mode once it is named among the modes to level in: a
// mode that stands on all six legs, drives no leg motor, and is the one driving. (Whether a robot should level in its
// holonomic mode is its own asset's choice; nothing here must stand in the way.)
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistSecondMode,
	"Ramms.Level.Assist.LevelsInEveryModeItIsNamedFor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistSecondMode::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/true, /*bStartOn=*/false, /*bWithSeat=*/true);
	if (!TestTrue(TEXT("the made-up robot with a seat and a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
	{
		return false;
	}
	R.StandSeat(0.0f, 0.0f, 0.0f);
	URammsHolonomicDriveController* Holonomic = NewObject<URammsHolonomicDriveController>(R.Actor, TEXT("Holonomic"));
	IRammsDriveMode*				HolonomicMode = Holonomic;
	HolonomicMode->SetDriveModeActive(false);
	HolonomicMode->SetDriveModeActive(true);
	const FName HolonomicId = HolonomicMode->GetDriveModeId();
	R.GroundDeg = FVector2D(3.0, -4.0);
	R.PoseChassis(R.GroundDeg);

	// Named for another mode only: it waits.
	R.Level->EnabledModeIds = { FName(TEXT("differential")) };
	R.Level->SetAssistEnabled(true);
	R.Frames(60);
	TestTrue(TEXT("not named for the mode that drives: waiting, nothing on the legs"), R.State() == EState::Suspended && R.CorrectionDeg().Size() <= 1.0e-6);

	// Named for it too: it levels, on all six legs as one plane, the seat's tilt among its motors.
	R.Level->EnabledModeIds.Add(HolonomicId);
	R.Frames(300);
	const FVector2D Base = R.GroundDeg - R.CorrectionDeg();
	TestTrue(*FString::Printf(TEXT("named for the mode that drives: level (%.2f, %.2f)"), Base.X, Base.Y), R.State() == EState::Active && Base.Size() <= 0.2);
	// A cross slope and a grade together, on six legs that all stand: the centre pair takes the roll with the corners
	// and none of the pitch (it is under the middle), so the six offsets are one plane.
	const double CornerRollCm = 0.5 * ((R.LegDz(1) + R.LegDz(5)) - (R.LegDz(0) + R.LegDz(4))) / 2.0;
	const double CentreRollCm = 0.5 * (R.LegDz(3) - R.LegDz(2));
	TestEqual(TEXT("the centre legs carry the roll in the corners' plane"), CentreRollCm / CentreY, CornerRollCm / CornerY, 0.004);
	TestTrue(TEXT("... and stand at the corners' mean height (but for where they sit fore and aft)"),
		FMath::Abs(0.5 * (R.LegDz(2) + R.LegDz(3)) - 0.25 * (R.LegDz(0) + R.LegDz(1) + R.LegDz(4) + R.LegDz(5)) - CentreX * FMath::Tan(FMath::DegreesToRadians(R.CorrectionDeg().X))) <= 0.05);
	TArray<FName> Claimed;
	static_cast<IRammsControlContributor*>(Holonomic)->GetClaimedMotorIds(Claimed);
	TestFalse(TEXT("(that mode drives neither a leg motor nor the seat's tilt)"), Claimed.Contains(SeatTilt) || Claimed.ContainsByPredicate([](const FName& Id) { return FLevelRig::LegMotors().Contains(Id); }));
	return true;
}

// Catches: the status of a robot at the end of its legs' reach telling a rider whom the seat holds level that they
// are tilted; and saying nothing of the seat having moved.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelControlsSeatStatus,
	"Ramms.Level.Controls.AtReachSaysWhatTheSeatMakesUp",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelControlsSeatStatus::RunTest(const FString& Parameters)
{
	using namespace RammsLevel;
	ERammsControlTextSeverity Severity = ERammsControlTextSeverity::Info;
	FStatusFacts			  Facts;
	Facts.bOn = true;
	Facts.Status = EStatus::AtReach;
	Facts.ReachLimit = EReachLimit::LiftedClearance;
	// The base 2 deg off, and no seat to speak of: the rider is 2 deg off.
	Facts.OffReferenceDeg = 1.8;
	TestEqual(TEXT("no seat: how far off the rider is"), StatusLine(Facts, Severity), FString(TEXT("At the end of levelling reach, 2 deg off: a raised wheel would come too near the ground")));
	// A seat that makes it up: the rider is level, and told what the seat is doing.
	Facts.RiderOffDeg = 0.1;
	TestEqual(TEXT("the seat makes it up"), StatusLine(Facts, Severity),
		FString(TEXT("At the end of levelling reach, the seat making up 2 deg: a raised wheel would come too near the ground")));
	TestTrue(TEXT("... which still asks for attention"), Severity == ERammsControlTextSeverity::Attention);
	// A seat that cannot make all of it up: how far off the rider is left.
	Facts.OffReferenceDeg = 4.2;
	Facts.RiderOffDeg = 1.2;
	TestEqual(TEXT("the seat makes up part"), StatusLine(Facts, Severity), FString(TEXT("At the end of levelling reach, 1 deg off: a raised wheel would come too near the ground")));
	// Neither off by a degree: no figure.
	Facts.OffReferenceDeg = 0.4;
	Facts.RiderOffDeg = 0.1;
	TestEqual(TEXT("under a degree either way"), StatusLine(Facts, Severity), FString(TEXT("At the end of levelling reach: a raised wheel would come too near the ground")));
	return true;
}

/** The automation tests' way to the curb controller's private steps (declared a friend in its header): the end of a
 *  crossing and the seat's take-over, which no public call reaches without a whole crossing run in a simulator. */
struct FRammsCurbControllerTestAccess
{
	/** As BeginNegotiation leaves the controller once a crossing has begun: in one of a crossing's phases, nothing
	 *  waiting to be handed straight back, the mode it came from remembered. */
	static void BeginAsCrossing(URammsCurbNegotiationController& Curb, FName CameFrom)
	{
		Curb.PreviousModeId = CameFrom;
		Curb.Phase = ERammsCurbPhase::Approach;
		Curb.bHandBackPending = false;
	}
	static void	  Finish(URammsCurbNegotiationController& Curb, ERammsCurbPhase EndPhase, const TCHAR* Why) { Curb.Finish(EndPhase, Why); }
	static void	  CancelByRider(URammsCurbNegotiationController& Curb) { Curb.CancelByRider(TEXT("the test's rider let the stick go")); }
	static void	  HandLegsBackFor(URammsCurbNegotiationController& Curb, FName Next) { Curb.HandLegsBackFor(Next); }
	static bool	  ResolveSeat(URammsCurbNegotiationController& Curb, FString& OutWhy) { return Curb.ResolveSeat(OutWhy); }
	static void	  CommandSeat(URammsCurbNegotiationController& Curb, double X) { Curb.CommandSeat(X); }
	static double SeatZeroX(const URammsCurbNegotiationController& Curb) { return Curb.SeatState.ZeroPivot.X; }
	static double SeatPitchOwnDeg(const URammsCurbNegotiationController& Curb) { return FMath::RadiansToDegrees(Curb.SeatState.PitchOwn); }
	static double SeatPitchRefDeg(const URammsCurbNegotiationController& Curb) { return FMath::RadiansToDegrees(Curb.SeatState.PitchRef); }
	/** The base's pitch as the crossing's seat hears it (rad): what the tilt makes up for. */
	static void SetBasePitchRad(URammsCurbNegotiationController& Curb, float Rad) { Curb.SeatPitchRad = Rad; }
	/** What a crossing remembers for its hand-back's stance ramp: level ground under the robot, the base level on it
	 *  now, and a drive stance that stood StanceDeg off that ground as the crossing began. */
	static void SetStanceOverLevelGround(URammsCurbNegotiationController& Curb, const FVector2D& StanceDeg)
	{
		Curb.ActiveTerrain = FRammsCurbTerrain();
		Curb.ActiveTerrain.bValid = true;
		Curb.bStanceAttitudeKnown = true;
		Curb.StanceAttitudeOverGround = StanceDeg;
		Curb.MeasuredPitchDeg = 0.0f;
		Curb.MeasuredRollDeg = 0.0f;
	}
};

namespace RammsLevelAssistTests
{
	/** The curb controller's spare on a hand-back's stance ramp (its HandBackRampSpare). */
	constexpr double RampSpare = 1.35;

	/** The stance the rig's legs stand in, every leg by its motors as they are commanded now (call it while they stand
	 *  as FLevelRig::Stand left them): one a mode selector can ramp on a robot with no control surface. With
	 *  SeatTiltTarget, the seat's tilt is part of the stance too. */
	FRammsDriveStance MotorStance(const FLevelRig& R, TOptional<float> SeatTiltTarget = TOptional<float>())
	{
		FRammsDriveStance Stance;
		for (const FName& Id : FLevelRig::LegMotors())
		{
			FRammsStanceMotorTarget& Target = Stance.MotorTargets.AddDefaulted_GetRef();
			Target.MotorId = Id;
			Target.Target = R.Baseline(Id);
		}
		if (SeatTiltTarget.IsSet())
		{
			FRammsStanceMotorTarget& Target = Stance.MotorTargets.AddDefaulted_GetRef();
			Target.MotorId = SeatTilt;
			Target.Target = SeatTiltTarget.GetValue();
		}
		return Stance;
	}

	/** One frame on the backend's clock: the mode selector's tick where there is one, the level assist's, the base's,
	 *  and the chassis where the ground and the legs' whole pose put it. */
	void FrameOnClock(const FLevelRig& R, URammsDriveModeSelector* Selector, float Dt = 1.0f / 30.0f)
	{
		R.Log->SimTime += Dt;
		if (Selector != nullptr)
		{
			Selector->TickComponent(Dt, LEVELTICK_All, nullptr);
		}
		R.Level->TickComponent(Dt, LEVELTICK_All, nullptr);
		R.Tick(Dt);
		R.PoseChassis(R.GroundDeg - PoseDeg(R));
	}

	/** What the level assist adds to the seat (deg): the platform's pitch as its tilt is told, less as its own command
	 *  has it. */
	double SeatTrimDeg(const FLevelRig& R)
	{
		return SeatPitchDeg(R, true) - SeatPitchDeg(R, false);
	}
} // namespace RammsLevelAssistTests

// Catches: the end of a crossing (URammsCurbNegotiationController::Finish, the main path: done, cancelled, refused at
// rest) switching the mode without handing the legs to the level assist first, or after; the mode selector's stance
// ramp started from the levelled joints rather than from the commands the hand-back left (the adopted plane ramped
// away); the level assist starting while the selector is still changing the stance (it is told by the selector itself,
// on this robot's own components); and a stance ramp not timed for the turn it really makes: about none where the
// whole of the pose's plane went into the offsets, the tilt limits' time where part of it is past what the level
// assist may carry and stays in the commands.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistCrossingEnd,
	"Ramms.Level.Assist.CrossingEndHandsBackThroughTheSelector",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistCrossingEnd::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	const float			Dt = 1.0f / 30.0f;
	const TArray<FName> Ids = FLevelRig::LegMotors();
	for (const bool bPast : { false, true })
	{
		const auto Say = [bPast](const TCHAR* What) {
			return FString::Printf(TEXT("%s: %s"), bPast ? TEXT("a slope past what may be carried") : TEXT("a slope within reach"), What);
		};
		FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/true, /*bStartOn=*/false, /*bWithSeat=*/true);
		if (!TestTrue(Say(TEXT("the made-up robot with a level assist")), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
		{
			return false;
		}
		R.Log->SimTime = 100.0;
		R.StandSeat(0.0f, 0.0f, 0.0f);
		const FRammsDriveStance			Stance = MotorStance(R);
		URammsHolonomicDriveController* Holonomic = NewObject<URammsHolonomicDriveController>(R.Actor, TEXT("Holonomic"));
		IRammsDriveMode*				HolonomicMode = Holonomic;
		IRammsDriveMode*				CurbMode = R.Curb;
		URammsDriveModeSelector*		Selector = NewObject<URammsDriveModeSelector>(R.Actor, TEXT("Selector"));
		FRig::BeginPlayOf(*Selector);
		Holonomic->LiftedStance = Stance;
		const FName HolonomicId = HolonomicMode->GetDriveModeId();
		R.GroundDeg = FVector2D(3.0, -2.0);
		R.PoseChassis(R.GroundDeg);

		// The drive mode selected from a pose that is not its stance (the corner cranks a little way off), so that its
		// stance is brought in over a second; the switch thrown meanwhile. Levelling waits for the selector to be done.
		for (const FName& Crank : { CrankFL, CrankFR, CrankRL, CrankRR })
		{
			R.Base->SetMotorCommand(Crank, 0.05f);
		}
		R.Tick();
		TestTrue(Say(TEXT("the drive mode is selected")), Selector->SetActiveMode(HolonomicId));
		R.Level->SetAssistEnabled(true);
		int32 SwitchFrames = 0, StartedEarly = 0;
		while (Selector->IsSwitchingMode() && SwitchFrames < 600)
		{
			FrameOnClock(R, Selector, Dt);
			++SwitchFrames;
			StartedEarly += Selector->IsSwitchingMode() && (R.State() == EState::Active || R.CorrectionDeg().Size() > 1.0e-6) ? 1 : 0;
		}
		TestTrue(Say(*FString::Printf(TEXT("(the stance took %d frames to bring in)"), SwitchFrames)), SwitchFrames >= 20 && SwitchFrames < 600);
		TestEqual(Say(TEXT("levelling does not start while the selector is changing the stance")), StartedEarly, 0);
		for (int32 Index = 0; Index < 200; ++Index)
		{
			FrameOnClock(R, Selector, Dt);
		}
		const FVector2D Levelled = R.CorrectionDeg();
		if (!TestTrue(Say(TEXT("levelled on the slope once it is done")), R.State() == EState::Active && (R.GroundDeg - Levelled).Size() <= 0.1))
		{
			return false;
		}

		// A crossing takes the legs, as BeginNegotiation does, and leaves the base level, 1.5 cm lower.
		R.Level->YieldTo(R.Curb);
		TestTrue(Say(TEXT("the curb mode is selected")), Selector->SetActiveMode(CurbMode->GetDriveModeId()));
		FRammsCurbControllerTestAccess::BeginAsCrossing(*R.Curb, HolonomicId);
		TestTrue(Say(TEXT("a crossing has the robot")), CurbMode->IsDriveModeActive() && !HolonomicMode->IsDriveModeActive() && R.Curb->IsNegotiating());
		LowerBase(R, 1.5);
		if (bPast)
		{
			// (The crossing's own level reach is the larger: it ends level on a slope the level assist may carry only
			// part of in the stance it hands back to.)
			R.Level->Tuning.MaxCorrectionDeg = FVector2D(1.0, 15.0);
		}
		for (int32 Index = 0; Index < 10; ++Index)
		{
			FrameOnClock(R, Selector, Dt);
		}
		TestTrue(Say(TEXT("standing by, the base level in the crossing's pose")), R.State() == EState::Yielded && (R.GroundDeg - PoseDeg(R)).Size() <= 0.1);
		TMap<FName, float> Told;
		for (const FName& Id : Ids)
		{
			Told.Add(Id, R.Effective(Id));
		}
		const int32 Before = R.LegPushes();

		// The crossing ends: Done.
		FRammsCurbControllerTestAccess::Finish(*R.Curb, ERammsCurbPhase::Done, TEXT("crossed"));
		TestTrue(Say(TEXT("the robot is handed back to the mode it came from, through the selector")),
			!CurbMode->IsDriveModeActive() && HolonomicMode->IsDriveModeActive() && Selector->GetActiveModeId() == HolonomicId && Selector->IsSwitchingMode());
		TestTrue(Say(TEXT("... and the legs to the level assist first: levelling at once")), R.State() == EState::Active && R.Level->IsLevelling());
		TestEqual(Say(TEXT("no leg motor was told anything by it")), R.LegPushes(), Before);
		for (const FName& Id : Ids)
		{
			TestEqual(Say(*FString::Printf(TEXT("%s: commanded as it was"), *Id.ToString())), R.Effective(Id), Told[Id], 1.0e-6f);
		}
		const FVector2D Adopted = R.CorrectionDeg();
		FVector2D		Left(100.0, 100.0);
		const bool		bLeft = R.Level->GetHandBackTurnDeg(Left);
		const FVector2D ExpectedLeft = bPast ? FVector2D(2.0, 0.0) : FVector2D::ZeroVector;
		AddInfo(Say(*FString::Printf(TEXT("the offsets carry (%.3f, %.3f) deg of the pose's (3, -2); (%.3f, %.3f) left in the commands"), Adopted.X, Adopted.Y, Left.X, Left.Y)));
		TestTrue(Say(TEXT("the offsets carry what of the plane the level assist may")), (Adopted - (R.GroundDeg - ExpectedLeft)).Size() <= 0.1);
		TestTrue(Say(TEXT("... and it says what is left in the commands for the ramp to turn the base by")), bLeft && (Left - ExpectedLeft).Size() <= 0.1);

		// The selector's ramp: timed for the turn it makes, and the base no further off level than that turn.
		const double TurnSeconds = RampSpare * RammsDrive::StanceTurnSeconds(ExpectedLeft.Size(), R.Curb->Tuning.TiltRateDegPerSec, R.Curb->Tuning.TiltAccelDegPerSec2);
		const double Expected = FMath::Max(static_cast<double>(Selector->StanceRampSeconds), TurnSeconds);
		double		 Seconds = 0.0, MostOff = 0.0, Fastest = 0.0;
		FVector2D	 Was = R.GroundDeg - PoseDeg(R);
		while (Selector->IsSwitchingMode() && Seconds < 30.0)
		{
			FrameOnClock(R, Selector, Dt);
			Seconds += Dt;
			const FVector2D Now = R.GroundDeg - PoseDeg(R);
			Fastest = FMath::Max(Fastest, (Now - Was).Size() / Dt);
			MostOff = FMath::Max(MostOff, Now.Size());
			Was = Now;
		}
		AddInfo(Say(*FString::Printf(TEXT("the stance ramp took %.2f s (%.2f expected); the base at most %.3f deg off level, turning at %.2f deg/s at most"), Seconds, Expected, MostOff, Fastest)));
		TestTrue(Say(TEXT("the stance ramp is timed for the turn it makes")), Seconds >= Expected - 0.1 && Seconds <= Expected + 0.4);
		if (bPast)
		{
			TestTrue(Say(TEXT("(the base turns by what was left in the commands)")), MostOff >= 1.5 && MostOff <= 2.3);
			TestTrue(Say(TEXT("... well inside the tilt rate")), Fastest <= 0.6 * R.Curb->Tuning.TiltRateDegPerSec);
		}
		else
		{
			TestTrue(Say(TEXT("level through the stance ramp")), MostOff <= 0.2);
		}
		for (const FRammsStanceMotorTarget& Target : Stance.MotorTargets)
		{
			TestEqual(Say(*FString::Printf(TEXT("%s: its command is the stance's"), *Target.MotorId.ToString())), R.Baseline(Target.MotorId), Target.Target, 1.0e-6f);
		}
		for (int32 Index = 0; Index < 150; ++Index)
		{
			FrameOnClock(R, Selector, Dt);
		}
		if (!bPast)
		{
			TestTrue(Say(TEXT("level on the stance, the offsets holding it")), (R.GroundDeg - R.CorrectionDeg()).Size() <= 0.1);
			TestEqual(Say(TEXT("at rest")), R.Status(), FString(TEXT("Level, at rest")));
		}
	}
	return true;
}

// Catches: a hand-back's stance ramp timed by anything but the turn the ramp will make. Four hand-backs to the same
// stance from a crossing that remembers a drive stance 4 deg off the ground: the whole plane taken into the offsets
// (no turn); part of it past what the level assist may carry (that part); nothing taken back though the level assist
// is on and levelling again (the whole turn to the ground's attitude: the mode selector applies no stance the legs
// could be read against, so every bit of the pose is in the commands); and the level assist off (the same).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistStanceTime,
	"Ramms.Level.Assist.StanceChangeTimedForWhatTheRampTurns",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistStanceTime::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	enum class ECase : uint8
	{
		Whole,
		Part,
		None,
		Off
	};
	for (const ECase Case : { ECase::Whole, ECase::Part, ECase::None, ECase::Off })
	{
		const TCHAR* const Name = Case == ECase::Whole ? TEXT("the whole plane adopted") : Case == ECase::Part ? TEXT("part of the plane adopted")
			: Case == ECase::None																			   ? TEXT("nothing adopted, levelling on")
																											   : TEXT("the level assist off");
		const auto		   Say = [Name](const TCHAR* What) { return FString::Printf(TEXT("%s: %s"), Name, What); };
		FLevelRig		   R;
		if (!TestTrue(Say(TEXT("the made-up robot with a level assist")), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
		{
			return false;
		}
		URammsHolonomicDriveController* Holonomic = NewObject<URammsHolonomicDriveController>(R.Actor, TEXT("Holonomic"));
		IRammsDriveMode*				HolonomicMode = Holonomic;
		IRammsDriveMode*				CurbMode = R.Curb;
		Holonomic->LiftedStance = RigStance();
		HolonomicMode->SetDriveModeActive(false);
		HolonomicMode->SetDriveModeActive(true);
		if (Case == ECase::None)
		{
			// A mode selector that applies no stance at all: there is nothing the legs are about to be taken to.
			URammsDriveModeSelector* Selector = NewObject<URammsDriveModeSelector>(R.Actor, TEXT("Selector"));
			FRig::BeginPlayOf(*Selector);
			Selector->bApplyLinkageStance = false;
		}
		R.GroundDeg = FVector2D(3.0, -2.0);
		R.PoseChassis(R.GroundDeg);
		R.Level->SetAssistEnabled(Case != ECase::Off);
		for (int32 Index = 0; Index < 150; ++Index)
		{
			FrameOnPose(R);
		}
		R.Level->YieldTo(R.Curb);
		HolonomicMode->SetDriveModeActive(false);
		CurbMode->SetDriveModeActive(true);
		FRammsCurbControllerTestAccess::SetStanceOverLevelGround(*R.Curb, FVector2D(4.0, 0.0));
		if (Case == ECase::Part)
		{
			R.Level->Tuning.MaxCorrectionDeg = FVector2D(1.0, 15.0);
		}
		for (int32 Index = 0; Index < 3; ++Index)
		{
			FrameOnPose(R);
		}
		TMap<FName, float> Was;
		for (const FName& Id : FLevelRig::LegMotors())
		{
			Was.Add(Id, R.Baseline(Id));
		}
		FRammsCurbControllerTestAccess::HandLegsBackFor(*R.Curb, HolonomicMode->GetDriveModeId());
		const double Seconds = CurbMode->GetStanceChangeMinSeconds();
		const auto	 TimeFor = [&R](double TurnDeg) { return RampSpare * RammsDrive::StanceTurnSeconds(TurnDeg, R.Curb->Tuning.TiltRateDegPerSec, R.Curb->Tuning.TiltAccelDegPerSec2); };
		AddInfo(Say(*FString::Printf(TEXT("the stance change is given %.2f s at least (a 4 deg turn takes %.2f, a 2 deg one %.2f)"), Seconds, TimeFor(4.0), TimeFor(2.0))));
		switch (Case)
		{
			case ECase::Whole:
				TestTrue(Say(TEXT("levelling again")), R.State() == EState::Active && R.Level->IsLevelling());
				TestEqual(Say(TEXT("the ramp turns the base by nothing: no time asked")), Seconds, 0.0, 0.05);
				break;
			case ECase::Part:
				TestTrue(Say(TEXT("levelling again")), R.State() == EState::Active && R.Level->IsLevelling());
				TestEqual(Say(TEXT("the ramp turns the base by the 2 deg left in the commands: timed for that")), Seconds, TimeFor(2.0), 0.08);
				break;
			case ECase::None:
			{
				FVector2D Left;
				TestTrue(Say(TEXT("levelling again, from nothing")), R.State() == EState::Active && R.Level->IsLevelling() && R.CorrectionDeg().Size() <= 1.0e-6);
				TestFalse(Say(TEXT("... and it says no plane was taken back out")), R.Level->GetHandBackTurnDeg(Left));
				for (const FName& Id : FLevelRig::LegMotors())
				{
					TestTrue(Say(*FString::Printf(TEXT("%s: its command as the crossing left it"), *Id.ToString())), SameBits(R.Baseline(Id), Was[Id]));
				}
				TestEqual(Say(TEXT("the whole pose is in the commands: timed for the turn to the ground's attitude")), Seconds, TimeFor(4.0), 1.0e-3);
				break;
			}
			default:
				TestTrue(Say(TEXT("still off")), R.State() == EState::Off && !R.Level->IsLevelling());
				TestEqual(Say(TEXT("timed for the turn to the ground's attitude, as with no level assist")), Seconds, TimeFor(4.0), 1.0e-3);
				break;
		}
	}
	return true;
}

// Catches: a seat trim that does not survive a take-over by something that never takes the seat. A crossing called off
// before it ran (the rider let go while the plan was awaited: CancelByRider, the same hand-back a refusal at rest
// makes) hands back to a drive mode whose stance names the seat's tilt: the trim must neither be stepped off the seat
// (the stance ramp started from a command that still held it), nor ramped out, nor walked in a second time on top of
// a command that still holds it. And a mode that takes the legs with nobody saying so (the tick's backstop) while the
// seat's own controller goes on rewriting the tilt's command every tick: the trim must not be dropped in one write.
// (The seat's controller itself is not on this rig: what it does to the tilt's command while it holds the seat is.)
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistSeatKept,
	"Ramms.Level.Assist.SeatTrimKeptWhereTheTakerNeverTakesTheSeat",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistSeatKept::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	const float Dt = 1.0f / 30.0f;
	{
		FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/true, /*bStartOn=*/false, /*bWithSeat=*/true);
		if (!TestTrue(TEXT("the made-up robot with a seat and a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
		{
			return false;
		}
		R.Log->SimTime = 100.0;
		R.StandSeat(0.0f, 0.0f, 0.0f);
		// The drive mode's stance names the seat's tilt: 0.08 rad.
		const float						StanceTilt = 0.08f;
		const FRammsDriveStance			Stance = MotorStance(R, StanceTilt);
		URammsHolonomicDriveController* Holonomic = NewObject<URammsHolonomicDriveController>(R.Actor, TEXT("Holonomic"));
		IRammsDriveMode*				HolonomicMode = Holonomic;
		IRammsDriveMode*				CurbMode = R.Curb;
		URammsDriveModeSelector*		Selector = NewObject<URammsDriveModeSelector>(R.Actor, TEXT("Selector"));
		FRig::BeginPlayOf(*Selector);
		Holonomic->LiftedStance = Stance;
		const FName HolonomicId = HolonomicMode->GetDriveModeId();
		TestTrue(TEXT("the drive mode is selected"), Selector->SetActiveMode(HolonomicId));
		for (int32 Index = 0; Index < 600 && Selector->IsSwitchingMode(); ++Index)
		{
			FrameOnClock(R, Selector, Dt);
		}
		TestTrue(TEXT("(its stance is in: the seat's tilt where it names it)"), !Selector->IsSwitchingMode() && SameBits(R.Baseline(SeatTilt), StanceTilt));
		// The rider tilts the seat themselves, and prefers 3 deg back; the robot stands on a slope.
		R.Base->SetMotorCommand(SeatTilt, -0.1f);
		static_cast<IRammsControlContributor*>(R.Level)->ApplyControl(RammsLevel::PreferencePitchControlId(), 3.0f);
		R.GroundDeg = FVector2D(3.0, -2.0);
		R.PoseChassis(R.GroundDeg);
		R.Level->SetAssistEnabled(true);
		for (int32 Index = 0; Index < 300; ++Index)
		{
			FrameOnClock(R, Selector, Dt);
		}
		if (!TestTrue(*FString::Printf(TEXT("levelled, the seat pitched back by the preferred 3 deg (%.2f)"), SeatTrimDeg(R)), R.State() == EState::Active && FMath::IsNearlyEqual(SeatTrimDeg(R), 3.0, 0.2)))
		{
			return false;
		}

		// A crossing takes the legs, and waits for its plan: it never commands the seat.
		R.Level->YieldTo(R.Curb);
		TestTrue(TEXT("the curb mode is selected"), Selector->SetActiveMode(CurbMode->GetDriveModeId()));
		FRammsCurbControllerTestAccess::BeginAsCrossing(*R.Curb, HolonomicId);
		const float ToldSeat = R.Effective(SeatTilt);
		const int32 SeatPushes = R.Log->CountFor(SeatTilt);
		int32		Moved = 0;
		for (int32 Index = 0; Index < 15; ++Index)
		{
			FrameOnClock(R, Selector, Dt);
			Moved += SameBits(R.Effective(SeatTilt), ToldSeat) ? 0 : 1;
		}
		TestTrue(TEXT("(the crossing has the legs)"), R.State() == EState::Yielded && CurbMode->IsDriveModeActive());
		TestEqual(TEXT("while the crossing has the legs and not the seat: the seat's tilt is told the same, to the bit"), Moved, 0);
		TestEqual(TEXT("... and told nothing"), R.Log->CountFor(SeatTilt), SeatPushes);
		TestTrue(TEXT("... its own command the rider's, the trim on top of it still"), SameBits(R.Baseline(SeatTilt), -0.1f) && R.Level->GetFoldedSeatTrimDeg() == 0.0f);
		TestEqual(TEXT("... which level.seat_trim still shows"), static_cast<double>(R.Control(RammsLevel::SeatTrimControlId())), 3.0, 0.2);

		// The rider lets go before the crossing has run: handed back, the stance ramped in by the selector.
		FRammsCurbControllerTestAccess::CancelByRider(*R.Curb);
		TestTrue(TEXT("called off: handed back through the selector, levelling again at once"),
			R.State() == EState::Active && HolonomicMode->IsDriveModeActive() && !CurbMode->IsDriveModeActive() && Selector->IsSwitchingMode());
		TestEqual(TEXT("the hand-back did not move the seat"), R.Effective(SeatTilt), ToldSeat, 1.0e-6f);
		double LeastTrim = SeatTrimDeg(R), MostTrim = LeastTrim, BiggestStep = 0.0;
		float  Last = R.Effective(SeatTilt);
		int32  After = 0;
		for (int32 Index = 0; Index < 900 && After < 90; ++Index)
		{
			FrameOnClock(R, Selector, Dt);
			After += Selector->IsSwitchingMode() ? 0 : 1;
			LeastTrim = FMath::Min(LeastTrim, SeatTrimDeg(R));
			MostTrim = FMath::Max(MostTrim, SeatTrimDeg(R));
			BiggestStep = FMath::Max(BiggestStep, static_cast<double>(FMath::Abs(R.Effective(SeatTilt) - Last)));
			Last = R.Effective(SeatTilt);
		}
		AddInfo(FString::Printf(TEXT("through the hand-back and the stance ramp: the seat's trim between %.2f and %.2f deg, its tilt told %.4f rad more or less from one frame to the next at most"), LeastTrim, MostTrim, BiggestStep));
		TestTrue(TEXT("the trim stays on the seat throughout: never stepped off, never ramped out, never twice"), LeastTrim >= 2.7 && MostTrim <= 3.3);
		// (The stance takes the seat's own command 0.18 rad, over a second at least, on a curve 1.5 times as steep at
		// its middle as a straight line: 0.009 rad a frame.)
		TestTrue(TEXT("the seat's tilt never stepped"), BiggestStep <= 0.012);
		TestTrue(TEXT("the seat's own command ends where the stance names it"), SameBits(R.Baseline(SeatTilt), StanceTilt));
	}
	{
		// The backstop, with the seat's own controller holding the seat.
		FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/true, /*bStartOn=*/false, /*bWithSeat=*/true);
		if (!TestTrue(TEXT("the made-up robot with a seat and a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
		{
			return false;
		}
		AddExpectedMessagePlain(TEXT("took the legs without yielding"), ELogVerbosity::Warning, EAutomationExpectedMessageFlags::Contains, 2);
		R.StandSeat(0.0f, 0.0f, -0.1f);
		URammsHolonomicDriveController* Holonomic = NewObject<URammsHolonomicDriveController>(R.Actor, TEXT("Holonomic"));
		IRammsDriveMode*				HolonomicMode = Holonomic;
		IRammsDriveMode*				CurbMode = R.Curb;
		Holonomic->LiftedStance = RigStance();
		HolonomicMode->SetDriveModeActive(false);
		HolonomicMode->SetDriveModeActive(true);
		static_cast<IRammsControlContributor*>(R.Level)->ApplyControl(RammsLevel::PreferencePitchControlId(), 3.0f);
		R.GroundDeg = FVector2D(3.0, -2.0);
		R.PoseChassis(R.GroundDeg);
		R.Level->SetAssistEnabled(true);
		// (What the seat's controller does to the tilt's command while it holds the seat: writes it again every tick,
		// from a measured angle that is never the same twice.)
		int32	   Frame = 0;
		const auto SeatControllerHolds = [&R, &Frame]() { R.Base->SetMotorCommand(SeatTilt, -0.1f + (++Frame % 2 == 0 ? 1.0e-5f : -1.0e-5f)); };
		for (int32 Index = 0; Index < 300; ++Index)
		{
			SeatControllerHolds();
			R.Frame(Dt);
		}
		if (!TestTrue(*FString::Printf(TEXT("levelled, the seat pitched back by the preferred 3 deg (%.2f)"), SeatTrimDeg(R)), R.State() == EState::Active && FMath::IsNearlyEqual(SeatTrimDeg(R), 3.0, 0.2)))
		{
			return false;
		}
		const float ToldSeat = R.Effective(SeatTilt);
		// The curb mode picked by hand: nobody yields, and the level assist's own tick stands by for it.
		HolonomicMode->SetDriveModeActive(false);
		CurbMode->SetDriveModeActive(true);
		SeatControllerHolds();
		R.Level->TickComponent(Dt, LEVELTICK_All, nullptr);
		R.Tick(Dt);
		TestTrue(TEXT("unannounced: standing by, the legs' offsets folded"), R.State() == EState::Yielded && R.CorrectionDeg().Size() <= 1.0e-6);
		TestEqual(TEXT("... the seat's tilt told what it was (the trim not dropped)"), R.Effective(SeatTilt), ToldSeat, 1.0e-4f);
		TestTrue(TEXT("... the trim still on the seat, none of it in the seat's command"), R.Base->GetMotorCommandOffset(SeatTilt) != 0.0f && R.Level->GetFoldedSeatTrimDeg() == 0.0f);
		// With nothing to cross it hands straight back; levelling goes on with the trim it finds on the seat.
		R.Curb->RegisterComponent();
		SeatControllerHolds();
		R.Curb->TickComponent(Dt, LEVELTICK_All, nullptr);
		TestTrue(TEXT("handed straight back: levelling again"), R.State() == EState::Active && HolonomicMode->IsDriveModeActive());
		double LeastTrim = SeatTrimDeg(R), MostTrim = LeastTrim;
		for (int32 Index = 0; Index < 90; ++Index)
		{
			SeatControllerHolds();
			R.Frame(Dt);
			LeastTrim = FMath::Min(LeastTrim, SeatTrimDeg(R));
			MostTrim = FMath::Max(MostTrim, SeatTrimDeg(R));
		}
		AddInfo(FString::Printf(TEXT("a take-over nobody announced, the seat's controller holding the seat: the trim between %.2f and %.2f deg throughout"), LeastTrim, MostTrim));
		TestTrue(TEXT("the trim stays on the seat throughout"), LeastTrim >= 2.7 && MostTrim <= 3.3);

		// Switched off while something has the legs, which hands them back in the same frame: the trim is walked off
		// the seat, not dropped with the legs' correction, and the seat's motor is let go only once it is.
		HolonomicMode->SetDriveModeActive(false);
		CurbMode->SetDriveModeActive(true);
		R.Level->TickComponent(Dt, LEVELTICK_All, nullptr);
		R.Tick(Dt);
		TestTrue(TEXT("(standing by again, the trim on the seat)"), R.State() == EState::Yielded && FMath::IsNearlyEqual(SeatTrimDeg(R), 3.0, 0.3));
		R.Level->SetAssistEnabled(false);
		R.Curb->TickComponent(Dt, LEVELTICK_All, nullptr);
		TestTrue(TEXT("switched off and handed back: off, the trim still on the seat"), R.State() == EState::Off && HolonomicMode->IsDriveModeActive() && FMath::IsNearlyEqual(SeatTrimDeg(R), 3.0, 0.3));
		double FastestDegPerSec = 0.0, WasTrim = SeatTrimDeg(R);
		int32  FramesToZero = 0;
		for (int32 Index = 0; Index < 150; ++Index)
		{
			R.Level->TickComponent(Dt, LEVELTICK_All, nullptr);
			R.Tick(Dt);
			FastestDegPerSec = FMath::Max(FastestDegPerSec, FMath::Abs(SeatTrimDeg(R) - WasTrim) / Dt);
			WasTrim = SeatTrimDeg(R);
			FramesToZero += FMath::Abs(WasTrim) > 1.0e-6 ? 1 : 0;
		}
		AddInfo(FString::Printf(TEXT("switched off while standing by: the trim walked off the seat in %d frames, at %.2f deg/s at most"), FramesToZero, FastestDegPerSec));
		TestTrue(TEXT("switched off: the trim goes at the tilt policy's rate"), FastestDegPerSec <= R.Level->Tuning.TiltRateDegPerSec * 1.05 && FramesToZero >= 25);
		TestTrue(TEXT("... to nothing, and the level assist is then off"), R.Base->GetMotorCommandOffset(SeatTilt) == 0.0f && R.State() == EState::Off);
		const int32 Count = R.Log->CountFor(SeatTilt);
		R.Base->SetMotorCommand(SeatTilt, -0.1f);
		TestEqual(TEXT("... its motors let go (a command to the seat's tilt is told at once again)"), R.Log->CountFor(SeatTilt), Count + 1);
	}
	return true;
}

// Catches: a crossing that takes the seat without the level assist's trim being put into the seat's command first
// (the trim then stays under the crossing's commands for the whole crossing), or after it has read the command; the
// rider's own setting read with the trim still in it (the rider is brought to the wrong pitch as the base levels in);
// the pitch the crossing first holds taken without the trim (the rider stepped by it at take-over); and a retry that
// no longer finds the seat as the last crossing left it because a trim was folded on top. Also pins when each of the
// crossing's seat commands is told: the two link motors at once, the tilt, which the level assist holds an offset
// layer on, at the robot base's tick of the same frame, bare.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistCrossingSeat,
	"Ramms.Level.Assist.CrossingTakesTheSeatWithItsTrim",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistCrossingSeat::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/true, /*bStartOn=*/false, /*bWithSeat=*/true);
	if (!TestTrue(TEXT("the made-up robot with a seat and a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
	{
		return false;
	}
	IRammsDriveMode* CurbMode = R.Curb;
	// The rider has the seat tilted 0.1 rad back themselves and prefers 3 deg more.
	R.StandSeat(0.0f, 0.0f, -0.1f);
	static_cast<IRammsControlContributor*>(R.Level)->ApplyControl(RammsLevel::PreferencePitchControlId(), 3.0f);
	R.GroundDeg = FVector2D(2.0, 1.0);
	R.PoseChassis(R.GroundDeg);
	R.Level->SetAssistEnabled(true);
	R.Frames(300);
	const double Own = SeatPitchDeg(R, false);
	const double With = SeatPitchDeg(R, true);
	if (!TestTrue(*FString::Printf(TEXT("levelled: the rider's own %.2f deg, %.2f with the preferred 3"), Own, With), R.State() == EState::Active && FMath::IsNearlyEqual(With - Own, 3.0, 0.2)))
	{
		return false;
	}

	// The crossing takes the legs, and, when it first commands the seat, the seat.
	R.Level->YieldTo(R.Curb);
	CurbMode->SetDriveModeActive(true);
	FString Why;
	if (!TestTrue(TEXT("the seat resolves"), FRammsCurbControllerTestAccess::ResolveSeat(*R.Curb, Why)))
	{
		AddError(Why);
		return false;
	}
	const double X = FRammsCurbControllerTestAccess::SeatZeroX(*R.Curb);
	const int32	 TiltPushes = R.Log->CountFor(SeatTilt);
	const int32	 LinkPushes = R.Log->CountFor(SeatA);
	FRammsCurbControllerTestAccess::CommandSeat(*R.Curb, X);
	TestTrue(TEXT("the crossing has the seat: nothing of the level assist's is left under its command"), R.Base->GetMotorCommandOffset(SeatTilt) == 0.0f);
	TestEqual(TEXT("the rider's own setting is the one without the trim"), FRammsCurbControllerTestAccess::SeatPitchOwnDeg(*R.Curb), Own, 1.0e-3);
	TestEqual(TEXT("the pitch the crossing first holds is where the rider is, trim and all"), FRammsCurbControllerTestAccess::SeatPitchRefDeg(*R.Curb), With, 1.0e-3);
	TestEqual(TEXT("... so its first command leaves the platform where it was"), SeatPitchDeg(R, false), With, 1.0e-3);
	// When each is told.
	TestEqual(TEXT("the seat's link motors are told at once"), R.Log->CountFor(SeatA), LinkPushes + 1);
	TestEqual(TEXT("the tilt, under the level assist's offset layer, not yet"), R.Log->CountFor(SeatTilt), TiltPushes);
	R.Tick();
	TestEqual(TEXT("... but at the robot base's tick of the same frame, once"), R.Log->CountFor(SeatTilt), TiltPushes + 1);
	TestTrue(TEXT("... the crossing's command, bare"), SameBits(R.Log->LastFor(SeatTilt), R.Baseline(SeatTilt)));

	// The base pitches under the crossing and the tilt makes up for it; the crossing stops there, and the level
	// assist, handed the legs back, puts its trim on again over the seat as the crossing left it.
	FRammsCurbControllerTestAccess::SetBasePitchRad(*R.Curb, 0.05f);
	FRammsCurbControllerTestAccess::CommandSeat(*R.Curb, X);
	R.Tick();
	TestEqual(TEXT("(the tilt makes up for the base's pitch)"), SeatPitchDeg(R, false), With - FMath::RadiansToDegrees(0.05), 1.0e-3);
	TestEqual(TEXT("(the crossing has moved the tilt: none of the trim counts as in its command any more)"), R.Level->GetFoldedSeatTrimDeg(), 0.0f);
	const float LeftAt = R.Baseline(SeatTilt);
	CurbMode->SetDriveModeActive(false);
	R.Level->ResumeFrom(R.Curb, nullptr);
	R.Frames(300);
	TestTrue(*FString::Printf(TEXT("handed back: levelling, the trim on again (%.2f) over the seat as the crossing left it"), SeatTrimDeg(R)),
		R.State() == EState::Active && FMath::IsNearlyEqual(SeatTrimDeg(R), 3.0, 0.2) && SameBits(R.Baseline(SeatTilt), LeftAt));

	// Tried again: the seat still stands as that crossing left it, so the setting kept is the rider's -- not the pitch
	// read back, which has the crossing's making-up in it, nor that with the trim.
	R.Level->YieldTo(R.Curb);
	CurbMode->SetDriveModeActive(true);
	TestTrue(TEXT("the seat resolves again"), FRammsCurbControllerTestAccess::ResolveSeat(*R.Curb, Why));
	FRammsCurbControllerTestAccess::SetBasePitchRad(*R.Curb, 0.0f);
	const double WithAgain = SeatPitchDeg(R, true);
	FRammsCurbControllerTestAccess::CommandSeat(*R.Curb, X);
	TestEqual(TEXT("tried again: the rider's own setting is the one the last crossing held"), FRammsCurbControllerTestAccess::SeatPitchOwnDeg(*R.Curb), Own, 1.0e-3);
	TestEqual(TEXT("... and the platform is left where it was"), SeatPitchDeg(R, false), WithAgain, 1.0e-3);
	TestTrue(TEXT("(which is not the setting: the last crossing's making-up and the trim are in it)"), FMath::Abs(WithAgain - Own) > 0.1);
	return true;
}

// Catches: a drive mode that drives the seat's tilt, and no leg, taken for one that has taken the legs. Levelling does
// not run in that mode (it is not named for it), so the base is given back to its stance: gently, within the tilt
// policy, as for any mode that leaves the legs alone -- not folded into the legs' commands at once as for a crossing,
// with a warning that somebody forgot to yield.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistSeatDrivingMode,
	"Ramms.Level.Assist.AModeThatDrivesTheSeatHasNotTakenTheLegs",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistSeatDrivingMode::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/true, /*bStartOn=*/false, /*bWithSeat=*/true);
	if (!TestTrue(TEXT("the made-up robot with a seat and a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
	{
		return false;
	}
	R.StandSeat(0.0f, 0.0f, 0.0f);
	URammsHolonomicDriveController* Holonomic = NewObject<URammsHolonomicDriveController>(R.Actor, TEXT("Holonomic"));
	IRammsDriveMode*				HolonomicMode = Holonomic;
	HolonomicMode->SetDriveModeActive(false);
	HolonomicMode->SetDriveModeActive(true);
	// A made-up mode that says it drives the seat's tilt motor (and a wheel): never begun, it drives nothing here.
	URammsDifferentialDriveController* SeatDriver = NewObject<URammsDifferentialDriveController>(R.Actor, TEXT("SeatDriver"));
	IRammsDriveMode*				   SeatDriverMode = SeatDriver;
	SeatDriver->LeftMotorId = SeatTilt;
	SeatDriver->RightMotorId = TEXT("wheel_free");
	SeatDriverMode->SetDriveModeActive(false);
	R.Level->EnabledModeIds = { HolonomicMode->GetDriveModeId() };
	R.GroundDeg = FVector2D(3.0, -2.0);
	R.PoseChassis(R.GroundDeg);
	R.Level->SetAssistEnabled(true);
	R.Frames(200);
	const FVector2D Levelled = R.CorrectionDeg();
	if (!TestTrue(TEXT("levelled in the mode it is named for"), R.State() == EState::Active && (R.GroundDeg - Levelled).Size() <= 0.2))
	{
		return false;
	}
	HolonomicMode->SetDriveModeActive(false);
	SeatDriverMode->SetDriveModeActive(true);
	TArray<FName> Claimed;
	static_cast<IRammsControlContributor*>(SeatDriver)->GetClaimedMotorIds(Claimed);
	TestTrue(TEXT("(that mode says it drives the seat's tilt, and no leg motor)"), Claimed.Contains(SeatTilt) && !Claimed.ContainsByPredicate([](const FName& Id) { return FLevelRig::LegMotors().Contains(Id); }));
	R.Frame();
	TestTrue(TEXT("a mode that drives the seat's tilt: the base is given back gently, the legs not folded for it"), R.State() == EState::Releasing);
	TestTrue(TEXT("... the plane still in the offsets a frame on"), (R.CorrectionDeg() - Levelled).Size() <= 0.3);
	return true;
}

// Catches: a fore/aft preference the seat has no room for dropped instead of being held by the legs: a trim limit
// under the preference's own, and a seat the rider has tilted to near its stop.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistPreferencePastTheSeat,
	"Ramms.Level.Assist.PreferenceTheSeatHasNoRoomForIsTheLegs",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistPreferencePastTheSeat::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	using RammsLevel::FPreferenceSplit;
	using RammsLevel::SplitPreference;
	// The split itself.
	{
		const FPreferenceSplit Fits = SplitPreference(FVector2D(3.0, -2.0), true, true, -40.0, 30.0, 8.0);
		TestTrue(TEXT("a preference the seat has room for: the pitch in the seat, the roll in the base"), Fits.SeatDeg == 3.0 && Fits.BaseDeg.Equals(FVector2D(0.0, -2.0), 0.0));
		const FPreferenceSplit Limit = SplitPreference(FVector2D(8.0, -2.0), true, true, -40.0, 30.0, 3.0);
		TestTrue(TEXT("past the trim's limit: the limit in the seat, the rest in the base"), Limit.SeatDeg == 3.0 && Limit.BaseDeg.Equals(FVector2D(5.0, -2.0), 1.0e-12));
		const FPreferenceSplit Room = SplitPreference(FVector2D(-6.0, 0.0), true, true, -1.5, 30.0, 8.0);
		TestTrue(TEXT("past the seat's own room, nose down: what fits in the seat, the rest in the base"), Room.SeatDeg == -1.5 && Room.BaseDeg.Equals(FVector2D(-4.5, 0.0), 1.0e-12));
		const FPreferenceSplit NoRoom = SplitPreference(FVector2D(4.0, 1.0), true, true, -10.0, 0.0, 8.0);
		TestTrue(TEXT("a seat on its stop that way: all of it in the base"), NoRoom.SeatDeg == 0.0 && NoRoom.BaseDeg.Equals(FVector2D(4.0, 1.0), 0.0));
		const FPreferenceSplit NoSeat = SplitPreference(FVector2D(4.0, 1.0), false, true, -40.0, 30.0, 8.0);
		const FPreferenceSplit TrimOff = SplitPreference(FVector2D(4.0, 1.0), true, false, -40.0, 30.0, 8.0);
		TestTrue(TEXT("no seat, or its trim switched off: all of it in the base"),
			NoSeat.SeatDeg == 0.0 && NoSeat.BaseDeg.Equals(FVector2D(4.0, 1.0), 0.0) && TrimOff.SeatDeg == 0.0 && TrimOff.BaseDeg.Equals(FVector2D(4.0, 1.0), 0.0));
		const FPreferenceSplit NotANumber = SplitPreference(FVector2D(4.0, 1.0), true, true, NAN, 30.0, 8.0);
		TestTrue(TEXT("a room that is not a number is no seat: all of it in the base"), NotANumber.SeatDeg == 0.0 && NotANumber.BaseDeg.Equals(FVector2D(4.0, 1.0), 0.0));
	}
	// On the robot: the rider gets the whole preference either way.
	for (const bool bByTheLimit : { true, false })
	{
		const TCHAR* const Name = bByTheLimit ? TEXT("a trim limit of 3 deg") : TEXT("a seat tilted to near its stop");
		FLevelRig		   R(/*bWithCurb=*/true, /*bWithLevel=*/true, /*bStartOn=*/false, /*bWithSeat=*/true);
		if (!TestTrue(TEXT("the made-up robot with a seat and a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
		{
			return false;
		}
		// (The made-up tilt motor stops at -0.6 rad, which is 0.6 rad of platform pitch nose up: tilted to -0.55 rad
		// by its rider, the seat has under 3 deg left that way.)
		R.StandSeat(0.0f, 0.0f, bByTheLimit ? 0.0f : -0.55f);
		if (bByTheLimit)
		{
			R.Level->Tuning.SeatTrimMaxDeg = 3.0f;
		}
		R.GroundDeg = FVector2D(-2.0, 1.0);
		R.PoseChassis(R.GroundDeg);
		static_cast<IRammsControlContributor*>(R.Level)->ApplyControl(RammsLevel::PreferencePitchControlId(), 7.0f);
		R.Level->SetAssistEnabled(true);
		R.Frames(450);
		const FVector2D Base = R.GroundDeg - R.CorrectionDeg();
		const double	Trim = SeatTrimDeg(R);
		AddInfo(FString::Printf(TEXT("%s, 7 deg preferred: the seat holds %.2f deg, the base %.2f; the rider %.2f"), Name, Trim, Base.X, Base.X + Trim));
		TestTrue(*FString::Printf(TEXT("%s: levelling"), Name), R.State() == EState::Active);
		TestTrue(*FString::Printf(TEXT("%s: the seat holds what it has room for, and no more"), Name), Trim >= 2.0 && Trim <= 3.1);
		TestEqual(FString::Printf(TEXT("%s: the rider is pitched back by the whole 7 deg they asked for"), Name), Base.X + Trim, 7.0, 0.3);
		TestEqual(FString::Printf(TEXT("%s: the roll is level"), Name), Base.Y, 0.0, 0.2);
	}
	return true;
}

// Catches: the drive's speed request dropped in one tick when levelling stops, with the scale still under 1 (the
// commanded speed steps up to full); the drive then left at full speed while level.speed_scale shows something else;
// and, levelling begun again soon after a take-over, a drive at full speed asked for a part of it in one go.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistSpeedScaleLetGo,
	"Ramms.Level.Assist.DriveLetGoGentlyWhenLevellingStops",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistSpeedScaleLetGo::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	FLevelRig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
	{
		return false;
	}
	URammsHolonomicDriveController* Drive = NewObject<URammsHolonomicDriveController>(R.Actor, TEXT("Holonomic"));
	IRammsDriveMode*				CurbMode = R.Curb;
	const float						Dt = 1.0f / 30.0f;
	R.Log->SimTime = 50.0;
	// Driven along a 9 deg slope at 150 cm/s (the chassis moved by hand): the cap holds the speed the slope's end can
	// be taken at for as long as the robot stands on it, whatever is left to correct.
	const auto DriveOn = [&R, Dt]() {
		R.Log->SimTime += Dt;
		R.Origin.X += 150.0 * Dt;
		R.Frame(Dt);
	};
	R.GroundDeg = FVector2D(9.0, 0.0);
	R.PoseChassis(R.GroundDeg);
	R.Level->SetAssistEnabled(true);
	for (int32 Index = 0; Index < 240; ++Index)
	{
		DriveOn();
	}
	const float Held = Drive->GetDriveSpeedScale();
	if (!TestTrue(*FString::Printf(TEXT("levelled and driving on the slope, the drive held to a part of its speed (%.2f)"), Held), R.State() == EState::Active && Held < 0.95f))
	{
		return false;
	}

	// Switched off while driving: the base is given back, and then the drive -- at the scale's own rise, never in a step.
	R.Level->SetAssistEnabled(false);
	const double Rise = R.Level->Tuning.SpeedScaleRisePerSec * Dt + 1.0e-4;
	float		 Was = Held;
	double		 BiggestRise = 0.0;
	int32		 Differing = 0, FramesReleasing = 0, FramesAfter = 0;
	float		 AtTheEnd = 1.0f;
	for (int32 Index = 0; Index < 600 && Drive->GetDriveSpeedScale() < 1.0f; ++Index)
	{
		const bool bWasReleasing = R.State() == EState::Releasing || Index == 0;
		DriveOn();
		const float Now = Drive->GetDriveSpeedScale();
		BiggestRise = FMath::Max(BiggestRise, static_cast<double>(Now - Was));
		Differing += FMath::IsNearlyEqual(Now, R.Control(RammsLevel::SpeedScaleControlId()), 1.0e-6f) ? 0 : 1;
		if (R.State() == EState::Releasing)
		{
			++FramesReleasing;
		}
		else
		{
			if (bWasReleasing)
			{
				AtTheEnd = Was;
			}
			++FramesAfter;
		}
		Was = Now;
	}
	AddInfo(FString::Printf(TEXT("switched off at a scale of %.2f: the release took %d frames and ended with the scale at %.2f; the drive was let go over %d more; it rose %.4f a frame at most (the scale's own rise is %.4f)"), Held, FramesReleasing, AtTheEnd,
		FramesAfter, BiggestRise, Rise));
	TestTrue(TEXT("(the release ended with the scale still under 1)"), R.State() == EState::Off && FramesReleasing > 0 && AtTheEnd < 0.95f);
	TestTrue(TEXT("the drive's scale never steps up: it rises at the scale's own rate, through the end of the release"), BiggestRise <= Rise);
	TestEqual(TEXT("... and is what level.speed_scale shows on every frame until it is 1"), Differing, 0);
	TestEqual(TEXT("... which it then is, the request withdrawn"), Drive->GetDriveSpeedScale(), 1.0f);

	// Levelling again, slowed again; a crossing takes the legs for a moment and hands them back: the drive, let go at
	// once for the crossing, is not asked for a part of its speed in one go when levelling goes on.
	R.Level->SetAssistEnabled(true);
	for (int32 Index = 0; Index < 240; ++Index)
	{
		DriveOn();
	}
	const float HeldAgain = Drive->GetDriveSpeedScale();
	TestTrue(*FString::Printf(TEXT("levelling again, the drive held again (%.2f)"), HeldAgain), R.State() == EState::Active && HeldAgain < 0.95f);
	// (A take-over refused in the same call changes nothing: the drive keeps the scale it had.)
	R.Level->YieldTo(R.Curb);
	R.Level->ResumeFrom(R.Curb, nullptr);
	TestEqual(TEXT("a take-over refused at once: the drive keeps its scale"), Drive->GetDriveSpeedScale(), HeldAgain);
	R.Level->YieldTo(R.Curb);
	CurbMode->SetDriveModeActive(true);
	TestEqual(TEXT("a crossing takes the legs: the drive is let go at once"), Drive->GetDriveSpeedScale(), 1.0f);
	DriveOn();
	TestEqual(TEXT("... and level.speed_scale says so"), R.Control(RammsLevel::SpeedScaleControlId()), 1.0f);
	CurbMode->SetDriveModeActive(false);
	const FRammsDriveStance Stance = RigStance();
	R.Level->ResumeFrom(R.Curb, &Stance);
	DriveOn();
	const float First = Drive->GetDriveSpeedScale();
	AddInfo(FString::Printf(TEXT("handed back two frames later: the first scale asked of the drive is %.3f"), First));
	TestTrue(TEXT("handed back a moment later: the drive is slowed again from full speed, not stepped down to where it was"), R.State() == EState::Active && First > 0.5f * (1.0f + HeldAgain));
	return true;
}

// Catches: a robot turning on the spot on a slope left to tilt (the slope swings from pitch into roll in the chassis'
// own frame with no distance travelled, and a ground estimate that follows the ground only as fast as the robot
// travels over it does not see it), and then held to its slowest by the error that leaves.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistTurnOnASlope,
	"Ramms.Level.Assist.LevelThroughATurnOnTheSpotOnASlope",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistTurnOnASlope::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	using RammsLevel::EState;
	FLevelRig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
	{
		return false;
	}
	URammsHolonomicDriveController* Drive = NewObject<URammsHolonomicDriveController>(R.Actor, TEXT("Holonomic"));
	const float						Dt = 1.0f / 30.0f;
	// A 5 deg slope rising along the world's +X. Facing YawDeg off that, the chassis reads its pitch as the slope's
	// rise along its heading and its roll as the rise across it.
	const double SlopeDeg = 5.0;
	const auto	 Ground = [SlopeDeg](double YawDeg) {
		  const double Yaw = FMath::DegreesToRadians(YawDeg);
		  return FVector2D(SlopeDeg * FMath::Cos(Yaw), -SlopeDeg * FMath::Sin(Yaw));
	};
	R.GroundDeg = Ground(R.YawDeg);
	R.PoseChassis(R.GroundDeg);
	R.Level->SetAssistEnabled(true);
	R.Frames(200, Dt);
	if (!TestTrue(TEXT("levelled on the slope, facing up it"), R.State() == EState::Active && (R.GroundDeg - R.CorrectionDeg()).Size() <= 0.2 && Drive->GetDriveSpeedScale() == 1.0f))
	{
		return false;
	}
	// Turned on the spot, the stick asking 60 deg/s: the drive turns at that times the scale it is asked to keep to.
	// About the middle of its corner pivots, which is where the level assist reads the robot's place: that point
	// goes nowhere at all, so nothing but the turn itself says the ground is turning. (The pivots' middle is 25 cm
	// behind the chassis' own origin on this robot.)
	const double PivotsBehindCm = 25.0;
	const auto	 Place = [&R, PivotsBehindCm]() {
		  const double Yaw = FMath::DegreesToRadians(R.YawDeg);
		  R.Origin = FVector(PivotsBehindCm * (FMath::Cos(Yaw) - 1.0), PivotsBehindCm * FMath::Sin(Yaw), 30.0);
	};
	const double AskedDegPerSec = 60.0;
	double		 MostOff = 0.0, LeastScale = 1.0, ScaleSum = 0.0, Turned = 0.0;
	const int32	 Frames = 180;
	for (int32 Index = 0; Index < Frames; ++Index)
	{
		const double Scale = Drive->GetDriveSpeedScale();
		R.YawDeg += AskedDegPerSec * Scale * Dt;
		Turned += AskedDegPerSec * Scale * Dt;
		R.GroundDeg = Ground(R.YawDeg);
		Place();
		R.PoseChassis(R.GroundDeg - R.CorrectionDeg());
		R.Level->TickComponent(Dt, LEVELTICK_All, nullptr);
		R.Tick(Dt);
		R.PoseChassis(R.GroundDeg - R.CorrectionDeg());
		MostOff = FMath::Max(MostOff, (R.GroundDeg - R.CorrectionDeg()).Size());
		LeastScale = FMath::Min(LeastScale, Scale);
		ScaleSum += Scale;
	}
	AddInfo(FString::Printf(TEXT("turned on the spot on %.0f deg, 60 deg/s asked, for %.0f s: turned %.0f deg; the base at most %.2f deg off level; the drive's scale %.2f at least, %.2f on average (its floor is %.2f)"), SlopeDeg, Frames * Dt, Turned, MostOff,
		LeastScale, ScaleSum / Frames, R.Level->Tuning.MinSpeedScale));
	TestTrue(TEXT("still levelling"), R.State() == EState::Active);
	TestTrue(TEXT("the base stays level through the turn"), MostOff <= 1.0);
	TestTrue(TEXT("the turn is not slowed for it: the legs can follow this"), ScaleSum / Frames >= 0.9 && LeastScale >= 0.75);
	TestTrue(TEXT("(it turned most of the way round)"), Turned >= 200.0);
	return true;
}

// Catches: a hand-back's plane taken against a stance the mode selector will not apply as given: a linkage it leaves
// out of every stance (its leg stays where it is commanded, and must not be taken for one about to go to the stance's
// height), and a selector that applies no stance at all (nothing is about to move: there is no plane to take).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelLegPlantStanceHeights,
	"Ramms.Level.LegPlant.StanceHeightsAsTheSelectorAppliesThem",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelLegPlantStanceHeights::RunTest(const FString& Parameters)
{
	using namespace RammsLevelAssistTests;
	FLevelRig R(/*bWithCurb=*/true, /*bWithLevel=*/false);
	if (!TestTrue(TEXT("the made-up robot on the recording backend"), R.IsReady()))
	{
		return false;
	}
	TUniquePtr<RammsLevel::IPlant> Plant = RammsLevel::MakeLegPlant(*R.Base, *R.Curb, R.Layer());
	FString						   Why;
	RammsLevel::FPlantLegs		   Legs;
	if (!TestTrue(TEXT("resolved, measured"), Plant->Resolve(Why) && Plant->MeasureLegs(Legs)))
	{
		return false;
	}
	// A stance 2 cm higher at the centre legs than where they stand, the corner cranks as they are.
	FRammsDriveStance Stance = RigStance();
	Stance.LinkageHeightCm = static_cast<float>(StanceZ + 2.0);
	RammsLevel::FLegValues Z;
	TestTrue(TEXT("no selector on the robot: the stance as given"), Plant->StanceHeights(Stance, Z) && Z.Num() == 6);
	if (Z.Num() == 6)
	{
		TestEqual(TEXT("a corner leg: at its crank's stance angle"), Z[0], StanceZ, 0.01);
		TestEqual(TEXT("the left centre leg: at the stance's linkage height"), Z[2], StanceZ + 2.0, 1.0e-4);
		TestEqual(TEXT("the right centre leg: the same"), Z[3], StanceZ + 2.0, 1.0e-4);
	}
	URammsDriveModeSelector* Selector = NewObject<URammsDriveModeSelector>(R.Actor, TEXT("Selector"));
	FRig::BeginPlayOf(*Selector);
	Selector->StanceExcludedLinkages = { R.Left->GetFName() };
	TestTrue(TEXT("a selector that leaves the left linkage out of every stance"), Plant->StanceHeights(Stance, Z) && Z.Num() == 6);
	if (Z.Num() == 6)
	{
		TestEqual(TEXT("the left centre leg stays where it is commanded"), Z[2], StanceZ, 0.01);
		TestEqual(TEXT("the right centre leg goes to the stance's linkage height"), Z[3], StanceZ + 2.0, 1.0e-4);
	}
	Selector->bApplyLinkageStance = false;
	TestFalse(TEXT("a selector that applies no stance: there is none to read the legs against"), Plant->StanceHeights(Stance, Z));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
