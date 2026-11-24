// Tests for the level assist on a robot of the Gen1 kind: where the flat ground a stance rests on lies
// (RammsLevel::StanceGround, which SupportMask judges who stands by on a robot whose stances do not stand level in
// its own frame), the chassis body's attitude (RammsLevel::FBodyAttitudeSource), the plant
// (RammsLevel::MakeGen1Plant: four legs on lift motors with height tables, the wheels' places read from the bodies,
// height offsets back onto the lifts, fold, unfold and rebase), and the component itself
// (URammsLevelAssistComponent) picking that plant, levelling pitch and roll with it, and a curb crossing
// (URammsRammpGen1CurbController) taking the lifts from it and handing them back.
//
// The robot is made up: two drive wheels on elevators and two pairs of small wheels on arms, each lift's wheels'
// height a straight line of its angle, on the command offset tests' rig (a robot base over a backend with no physics
// that records what it is told). Its bodies are scene components posed by hand each frame from what the lifts were
// told (the ground's slope with the plane the told heights make): a kinematic stand-in, enough for signs, sizes and
// what is and is not written. No robot's own numbers.
//
// Beside each test is what it must catch.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Components/SceneComponent.h"
#include "Level/RammsLevelAllocate.h"
#include "Level/RammsLevelAttitude.h"
#include "Level/RammsLevelControls.h"
#include "Level/RammsLevelLaw.h"
#include "Level/RammsLevelPlant.h"
#include "Level/RammsLevelTypes.h"
#include "RammpGen1ControllerComponent.h"
#include "RammsDifferentialDriveController.h"
#include "RammsLevelAssistComponent.h"
#include "RammsRammpGen1CurbController.h"
#include "Tests/RammsCommandOffsetRig.h"

#if WITH_DEV_AUTOMATION_TESTS

/** The automation tests' way to the Gen1 curb controller's private steps (declared a friend in its header): a
 *  crossing committed without a curb to find, and ended without one to cross. */
struct FRammsRammpGen1CurbTestAccess
{
	/** As StartCrossing commits a crossing of a step it found ahead: the curb mode selected, the crossing to begin on
	 *  the controller's next tick. */
	static bool Commit(URammsRammpGen1CurbController& Curb, FString& OutWhy)
	{
		URammsRammpGen1CurbController::FPlan Plan;
		Plan.EdgePoint = FVector(200.0, 0.0, 0.0);
		Plan.EdgeAheadCm = 120.0f;
		Plan.StepCm = 10.0f;
		return Curb.Commit(Plan, OutWhy);
	}
	static void	 Finish(URammsRammpGen1CurbController& Curb, bool bAborted, const TCHAR* Why) { Curb.Finish(bAborted, Why); }
	static bool	 IsCrossing(const URammsRammpGen1CurbController& Curb) { return Curb.IsCrossing(); }
	static bool	 IsStartPending(const URammsRammpGen1CurbController& Curb) { return Curb.bStartPending; }
	static float StartCommand(const URammsRammpGen1CurbController& Curb, int32 Lift) { return Curb.StartCmd[Lift]; }
	/** The debug view's commanded height of the drive wheels (cm, the table's). */
	static bool DriveCommandedHeight(const URammsRammpGen1CurbController& Curb, float& OutCm)
	{
		return Curb.LiftHeightCm(URammsRammpGen1CurbController::ESupport::Drive, /*bCommanded=*/true, OutCm);
	}
	/** The lifts in the controller's own order: elevator left, elevator right, the two translators, the front arm, the
	 *  rear lift. */
	static constexpr int32 ElevatorL = URammsRammpGen1CurbController::ElevatorL;
	static constexpr int32 ElevatorR = URammsRammpGen1CurbController::ElevatorR;
	static constexpr int32 FrontArmLift = URammsRammpGen1CurbController::FrontArmLift;
	static constexpr int32 RearLiftLift = URammsRammpGen1CurbController::RearLiftLift;
};

namespace RammsLevelGen1Tests
{
	using RammsCommandOffsetTests::FRig;
	using RammsCommandOffsetTests::MotorRow;
	using RammsCommandOffsetTests::SameBits;
	using namespace RammsLevel;

	const FName ElevatorL(TEXT("elevator_l"));
	const FName ElevatorR(TEXT("elevator_r"));
	const FName RearLift(TEXT("rear_lift"));
	const FName FrontArm(TEXT("front_arm"));
	const FName TranslatorL(TEXT("translator_l"));
	const FName TranslatorR(TEXT("translator_r"));

	/** The legs' order everywhere: drive left, drive right, the rear pair, the front pair. */
	constexpr int32 DriveLeft = 0;
	constexpr int32 DriveRight = 1;
	constexpr int32 Rear = 2;
	constexpr int32 Front = 3;

	/** Where the wheels are on the made-up chassis (cm): fore and aft, and half the distance across. */
	constexpr double DriveX = 40.0;
	constexpr double DriveHalf = 28.0;
	constexpr double RearX = -30.0;
	constexpr double RearHalf = 10.0;
	constexpr double FrontX = 70.0;
	constexpr double FrontHalf = 16.0;
	/** The wheels' radii (cm). */
	constexpr double DriveRadius = 15.0;
	constexpr double SmallRadius = 6.0;
	/** Each lift's wheels' bottom height (cm) is a straight line of its angle (rad): this much per radian, falling as
	 *  the angle rises; and the rear pair's is this far below the others' with its lift at zero. */
	constexpr double DrivePerRad = 30.0;
	constexpr double PairPerRad = 20.0;
	constexpr double RearAtZero = -2.0;
	/** The stance: the drive wheels let down 4.5 cm, the rear pair at its zero, the front pair carried 4 cm up. On the
	 *  flat it stands on its drive wheels and its rear pair, two degrees nose up. */
	constexpr float StanceElevator = 0.15f;
	constexpr float StanceRear = 0.0f;
	constexpr float StanceFront = -0.2f;

	double DriveHeight(double Angle)
	{
		return -DrivePerRad * Angle;
	}
	double RearHeight(double Angle)
	{
		return RearAtZero - PairPerRad * Angle;
	}
	double FrontHeight(double Angle)
	{
		return -PairPerRad * Angle;
	}

	FRammsRammpGen1SupportTable LineTable(double FromRad, double ToRad, TFunctionRef<double(double)> HeightAt)
	{
		FRammsRammpGen1SupportTable Table;
		const int32					Steps = 24;
		for (int32 Index = 0; Index <= Steps; ++Index)
		{
			const double Angle = FMath::Lerp(FromRad, ToRad, static_cast<double>(Index) / Steps);
			Table.AngleRad.Add(static_cast<float>(Angle));
			Table.HeightCm.Add(static_cast<float>(HeightAt(Angle)));
		}
		return Table;
	}

	/** The made-up registry: the four lifts, bounded; the two translators; the wheels. */
	TArray<FRammsMotorSpec> Gen1Motors()
	{
		TArray<FRammsMotorSpec> Specs;
		Specs.Add(MotorRow(TEXT("elevator_l"), ERammsActuatorType::Position, FVector2D(-0.6, 0.6)));
		Specs.Add(MotorRow(TEXT("elevator_r"), ERammsActuatorType::Position, FVector2D(-0.6, 0.6)));
		Specs.Add(MotorRow(TEXT("rear_lift"), ERammsActuatorType::Position, FVector2D(-0.4, 0.6)));
		Specs.Add(MotorRow(TEXT("front_arm"), ERammsActuatorType::Position, FVector2D(-0.6, 0.6)));
		Specs.Add(MotorRow(TEXT("translator_l"), ERammsActuatorType::Position, FVector2D(-40.0, 40.0)));
		Specs.Add(MotorRow(TEXT("translator_r"), ERammsActuatorType::Position, FVector2D(-40.0, 40.0)));
		for (const TCHAR* Id : { TEXT("drive_l"), TEXT("drive_r"), TEXT("front_omni_l"), TEXT("front_omni_r") })
		{
			Specs.Add(MotorRow(Id, ERammsActuatorType::Velocity));
		}
		return Specs;
	}

	/**
	 * The rig with the made-up robot of the Gen1 kind on it: its bodies as scene components, a Gen1 curb controller
	 * that describes it (begun, so that it knows its lifts; it runs no crossing unless a test commits one), a
	 * differential drive mode to hand back to, and, with bWithLevel, the level assist.
	 */
	struct FGen1Rig : FRig
	{
		URammsRammpGen1CurbController*	   Curb = nullptr;
		URammsDifferentialDriveController* Drive = nullptr;
		URammsLevelAssistComponent*		   Level = nullptr;
		/** The lift controller the robot's rider moves the lifts with, where a test adds one (AddLiftController). */
		URammpGen1ControllerComponent* LiftController = nullptr;
		TMap<FName, USceneComponent*>  Bodies;
		/** The slope the robot stands on (deg; pitch, roll), where it is, which way it faces, and how far forward of
		 *  their middle its translators have its two drive wheels (cm). */
		FVector2D GroundDeg = FVector2D::ZeroVector;
		FVector	  Origin = FVector(0.0, 0.0, 30.0);
		double	  YawDeg = 0.0;
		double	  TranslatorCm[2] = { 0.0, 0.0 };

		explicit FGen1Rig(bool bWithLevel = true)
			: FRig(Gen1Motors())
		{
			if (Actor == nullptr || Base == nullptr)
			{
				return;
			}
			for (const TCHAR* Name : { TEXT("chassis"), TEXT("drive_wheel_l"), TEXT("drive_wheel_r"), TEXT("front_omni_l"), TEXT("front_omni_r"), TEXT("rear_omni_l"), TEXT("rear_omni_r") })
			{
				USceneComponent* Body = NewObject<USceneComponent>(Actor, Name);
				Body->RegisterComponent();
				Bodies.Add(Name, Body);
			}
			Curb = NewObject<URammsRammpGen1CurbController>(Actor, TEXT("Curb"));
			FRammsRammpGen1CurbConfig& Config = Curb->Config;
			Config.DriveTable = LineTable(-0.7, 0.7, DriveHeight);
			Config.RearTable = LineTable(-0.5, 0.7, RearHeight);
			Config.FrontTable = LineTable(-0.7, 0.7, FrontHeight);
			Config.DriveWheelRadiusCm = DriveRadius;
			Config.RearOmniRadiusCm = SmallRadius;
			Config.FrontOmniRadiusCm = SmallRadius;
			Config.ElevatorSpeedDegS = 18.0f;
			Config.RearLiftSpeedDegS = 26.0f;
			Config.FrontArmSpeedDegS = 30.0f;
			Config.FixedMassKg = 100.0f;
			Config.FixedComAheadCm = 25.0f;
			Config.DriveAssemblyMassKg = 10.0f;
			Config.DriveAssemblyComFromHubCm = 0.0f;
			Config.BellyClearanceCm = 5.0f;
			Config.FrameBackCm = -5.0f;
			Config.FrameFrontCm = 55.0f;
			Config.FrameHalfWidthCm = 15.0f;
			BeginPlayOf(*Curb);
			Drive = NewObject<URammsDifferentialDriveController>(Actor, TEXT("Differential"));
			IRammsDriveMode* DriveMode = Drive;
			DriveMode->SetDriveModeActive(false);
			DriveMode->SetDriveModeActive(true);
			if (bWithLevel)
			{
				Level = NewObject<URammsLevelAssistComponent>(Actor, TEXT("LevelAssist"));
				Level->Tuning.MaxCorrectionDeg = FVector2D(10.0, 6.0);
				BeginPlayOf(*Level);
			}
			Stand();
		}

		bool IsReady() const { return FRig::IsReady() && Curb != nullptr && Curb->HasBegunPlay() && Bodies.Num() == 7; }

		/** Every lift commanded to the stance, as the lift controller would, and told. */
		void Stand()
		{
			if (!IsReady())
			{
				return;
			}
			Base->SetMotorCommand(ElevatorL, StanceElevator);
			Base->SetMotorCommand(ElevatorR, StanceElevator);
			Base->SetMotorCommand(RearLift, StanceRear);
			Base->SetMotorCommand(FrontArm, StanceFront);
			Tick();
			Pose();
		}

		static TArray<FName> Lifts() { return { ElevatorL, ElevatorR, RearLift, FrontArm }; }

		/** A leg's wheels' bottom height (cm) at what its lift is told (bEffective) or at its baseline. */
		double Height(int32 Leg, bool bEffective) const
		{
			const FName	 Id = Lifts()[Leg];
			const double Angle = bEffective ? Effective(Id) : Baseline(Id);
			return Leg == Rear ? RearHeight(Angle) : Leg == Front ? FrontHeight(Angle)
																  : DriveHeight(Angle);
		}
		/** How far each leg's wheels are retracted past its baseline by what its lift is told (cm). */
		double Dz(int32 Leg) const { return Height(Leg, true) - Height(Leg, false); }

		/** The chassis' attitude standing on its drive wheels and its rear pair with the lifts as told, on the ground's
		 *  slope (deg; pitch nose up, roll +Y up): the rear's wheels higher than the drive wheels' puts the nose up, the
		 *  +Y drive wheel higher than the other puts that side down. */
		FVector2D AttitudeDeg() const
		{
			const double DriveMean = 0.5 * (Height(DriveLeft, true) + Height(DriveRight, true));
			const double Pitch = FMath::RadiansToDegrees(FMath::Atan((Height(Rear, true) - DriveMean) / (DriveX - RearX)));
			const double Roll = FMath::RadiansToDegrees(FMath::Atan((Height(DriveLeft, true) - Height(DriveRight, true)) / (2.0 * DriveHalf)));
			return GroundDeg + FVector2D(Pitch, Roll);
		}

		/** Put the bodies in the world where a chassis at Deg has them. bMirrored: the chassis body is scaled -1 across
		 *  the robot (a mesh brought in mirrored), which turns none of its axes. */
		void PoseAt(const FVector2D& Deg, bool bMirrored = false) const
		{
			const double	 P = FMath::DegreesToRadians(Deg.X);
			const double	 R = FMath::DegreesToRadians(Deg.Y);
			const double	 Y0 = FMath::DegreesToRadians(YawDeg);
			const FVector	 ForwardHint(FMath::Cos(P) * FMath::Cos(Y0), FMath::Cos(P) * FMath::Sin(Y0), FMath::Sin(P));
			const FVector	 SideHint(-FMath::Cos(R) * FMath::Sin(Y0), FMath::Cos(R) * FMath::Cos(Y0), FMath::Sin(R));
			const FQuat		 Turn(FRotationMatrix::MakeFromXY(ForwardHint, SideHint));
			const FVector	 Forward = Turn.GetAxisX();
			const FVector	 Side = Turn.GetAxisY();
			USceneComponent* Chassis = Bodies.FindRef(TEXT("chassis"));
			Chassis->SetWorldLocationAndRotation(Origin, Turn);
			Chassis->SetWorldScale3D(FVector(1.0, bMirrored ? -1.0 : 1.0, 1.0));
			auto Place = [&](const TCHAR* Name, double X, double Y) {
				Bodies.FindRef(Name)->SetWorldLocation(Origin + Forward * X + Side * Y);
			};
			// "Left" is the -Y side.
			Place(TEXT("drive_wheel_l"), DriveX + TranslatorCm[0], -DriveHalf);
			Place(TEXT("drive_wheel_r"), DriveX + TranslatorCm[1], DriveHalf);
			Place(TEXT("rear_omni_l"), RearX, -RearHalf);
			Place(TEXT("rear_omni_r"), RearX, RearHalf);
			Place(TEXT("front_omni_l"), FrontX, -FrontHalf);
			Place(TEXT("front_omni_r"), FrontX, FrontHalf);
		}
		void Pose() const { PoseAt(AttitudeDeg()); }

		/**
		 * The robot's own lift controller on the four lifts, each named by its motor as the robot's asset script names
		 * them and turned at the lifts' speeds, begun and holding every lift where it is commanded now. Its motors are
		 * not public: they are set as a script sets them, by name.
		 */
		bool AddLiftController()
		{
			LiftController = NewObject<URammpGen1ControllerComponent>(Actor, TEXT("Lifts"));
			UClass*				  Class = URammpGen1ControllerComponent::StaticClass();
			const FArrayProperty* Angular = FindFProperty<FArrayProperty>(Class, TEXT("AngularMotors"));
			const FArrayProperty* Linear = FindFProperty<FArrayProperty>(Class, TEXT("LinearMotors"));
			const FBoolProperty*  FindConstraints = FindFProperty<FBoolProperty>(Class, TEXT("bAutoFindConstraints"));
			const FBoolProperty*  FindMesh = FindFProperty<FBoolProperty>(Class, TEXT("bAutoFindSkeletalMesh"));
			if (Angular == nullptr || Linear == nullptr || FindConstraints == nullptr || FindMesh == nullptr)
			{
				return false;
			}
			TArray<FAngularMotorConfig>& AngularMotors = *Angular->ContainerPtrToValuePtr<TArray<FAngularMotorConfig>>(LiftController);
			AngularMotors.Reset();
			Linear->ContainerPtrToValuePtr<TArray<FLinearMotorConfig>>(LiftController)->Reset();
			const float Speeds[4] = { 18.0f, 18.0f, 26.0f, 30.0f };
			for (int32 Index = 0; Index < 4; ++Index)
			{
				FAngularMotorConfig& Motor = AngularMotors.AddDefaulted_GetRef();
				Motor.ConstraintName = Motor.MotorId = Lifts()[Index];
				Motor.bEnabled = true;
				Motor.MaxSpeed = Speeds[Index];
			}
			// (This robot has no skeleton to look for constraints on.)
			FindConstraints->SetPropertyValue_InContainer(LiftController, false);
			FindMesh->SetPropertyValue_InContainer(LiftController, false);
			if (!BeginPlayOf(*LiftController))
			{
				return false;
			}
			LiftController->SyncTargetsToCommands();
			return true;
		}

		/** The lift controller's own tick. On the robot it comes after the base's, in a later tick group. */
		void LiftTick(float Dt = 1.0f / 30.0f) const
		{
			if (LiftController != nullptr)
			{
				static_cast<UActorComponent*>(LiftController)->TickComponent(Dt, LEVELTICK_All, nullptr);
			}
		}

		/** One frame: the level assist's tick, the base's, the chassis answering what the lifts were told, and the lift
		 *  controller's tick where the robot has one. */
		void Frame(float Dt = 1.0f / 30.0f) const
		{
			if (Level != nullptr)
			{
				Level->TickComponent(Dt, LEVELTICK_All, nullptr);
			}
			Tick(Dt);
			Pose();
			LiftTick(Dt);
		}
		void Frames(int32 Count, float Dt = 1.0f / 30.0f) const
		{
			for (int32 Index = 0; Index < Count; ++Index)
			{
				Frame(Dt);
			}
		}

		/** The curb controller's own tick (protected on it; a component's is not). */
		void CurbTick(float Dt = 1.0f / 30.0f) const { static_cast<UActorComponent*>(Curb)->TickComponent(Dt, LEVELTICK_All, nullptr); }

		/** How many commands the lifts, and every other motor, have been given in all. */
		int32 LiftPushes() const
		{
			int32 Count = 0;
			for (const FName& Id : Lifts())
			{
				Count += Log->CountFor(Id);
			}
			return Count;
		}
		int32 OtherPushes() const
		{
			int32 Count = 0;
			for (const TCHAR* Id : { TEXT("translator_l"), TEXT("translator_r"), TEXT("drive_l"), TEXT("drive_r"), TEXT("front_omni_l"), TEXT("front_omni_r") })
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
		EState	State() const { return static_cast<EState>(FMath::RoundToInt(Control(StateControlId()))); }
		FString Status() const
		{
			FText Words;
			static_cast<const IRammsControlContributor*>(Level)->ReadControlText(StatusControlId(), Words);
			return Words.ToString();
		}
	};

	/** Four made-up legs of that shape for StanceGround, their wheels' bottoms at the heights given (cm). */
	TArray<FLegReach, TInlineAllocator<4>> ShapeLegs(double DriveLeftBottom, double DriveRightBottom, double RearBottom, double FrontBottom)
	{
		TArray<FLegReach, TInlineAllocator<4>> Legs;
		auto								   Add = [&Legs](double X, double Y, double Radius, double Bottom, bool bRoll) {
			  FLegReach& Leg = Legs.AddDefaulted_GetRef();
			  Leg.XY = FVector2D(X, Y);
			  Leg.RadiusCm = Radius;
			  Leg.BaseZ = Leg.MeasuredZ = Bottom + Radius;
			  Leg.MinZ = Leg.BaseZ - 20.0;
			  Leg.MaxZ = Leg.BaseZ + 20.0;
			  Leg.bRoll = bRoll;
		};
		Add(DriveX, -DriveHalf, DriveRadius, DriveLeftBottom, true);
		Add(DriveX, DriveHalf, DriveRadius, DriveRightBottom, true);
		Add(RearX, 0.0, SmallRadius, RearBottom, false);
		Add(FrontX, 0.0, SmallRadius, FrontBottom, false);
		return Legs;
	}
	const double ShapeHalfTracks[4] = { 0.0, 0.0, RearHalf, FrontHalf };
} // namespace RammsLevelGen1Tests

// Catches: who stands judged from the lowest wheel bottom in the chassis' frame on a robot that stands nose up or
// nose down on the flat (the legs that carry then read as lifted); the ground taken through the wrong legs (a centre
// of mass on the other side of the drive axle rocks the robot onto the other pair); a pair taken for one wheel on the
// centre line (it then cannot hold the robot against roll, and the ground comes out rolled); and a robot that stands
// level in its own frame disturbed at all.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelStanceGround,
	"Ramms.Level.Allocate.StanceGroundIsWhereItRests",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelStanceGround::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	const FTuning	Tuning;
	const FVector2D Aft(27.5, 0.0);	 // between the rear pair and the drive wheels
	const FVector2D Fore(50.0, 0.0); // ahead of the drive wheels

	// The drive wheels 4.5 cm down, the rear pair 2 cm down, the front pair 4 cm up: on the flat it rests on the drive
	// wheels and the rear pair, the front pair in the air.
	{
		TArray<FLegReach, TInlineAllocator<4>> Legs = ShapeLegs(-4.5, -4.5, -2.0, 4.0);
		// By the lowest bottom alone only the drive wheels stand: the rear pair is 2.5 cm "up".
		SupportMask(Legs, TConstArrayView<bool>(), Tuning);
		TestFalse(TEXT("judged from the lowest wheel, the rear pair that carries reads as lifted"), Legs[Rear].bSupport);
		TestTrue(TEXT("the ground it rests on is found"), StanceGround(Legs, MakeArrayView(ShapeHalfTracks), Aft));
		SupportMask(Legs, TConstArrayView<bool>(), Tuning);
		TestTrue(TEXT("on the ground it rests on: both drive wheels and the rear pair stand"), Legs[DriveLeft].bSupport && Legs[DriveRight].bSupport && Legs[Rear].bSupport);
		TestEqual(TEXT("... each carrying whole"), Legs[Rear].CarryShare, 1.0, 1.0e-9);
		TestFalse(TEXT("... the front pair does not"), Legs[Front].bSupport);
		// The ground under the front pair: the line from the rear's -2 at -30 through the drive's -4.5 at 40, at 70.
		const double GroundAtFront = -4.5 + (-4.5 - -2.0) / (DriveX - RearX) * (FrontX - DriveX);
		TestEqual(TEXT("... which is carried over it by its 4 cm and the stance's own lean"), Legs[Front].BaselineClearanceCm, 4.0 - GroundAtFront, 1.0e-6);
		TestEqual(TEXT("the ground under a leg that carries is its wheel bottom"), Legs[Rear].StanceGroundZ, -2.0, 1.0e-9);
	}
	// The same wheels with the weight ahead of the drive wheels: it rocks onto the front pair, which is 4 cm up, and
	// the rear pair comes off the ground.
	{
		TArray<FLegReach, TInlineAllocator<4>> Legs = ShapeLegs(-4.5, -4.5, -2.0, 4.0);
		TestTrue(TEXT("weight forward: the ground is found"), StanceGround(Legs, MakeArrayView(ShapeHalfTracks), Fore));
		SupportMask(Legs, TConstArrayView<bool>(), Tuning);
		TestTrue(TEXT("weight forward: the drive wheels and the front pair stand"), Legs[DriveLeft].bSupport && Legs[DriveRight].bSupport && Legs[Front].bSupport);
		TestFalse(TEXT("... and the rear pair is in the air"), Legs[Rear].bSupport);
	}
	// Every wheel down, the rear pair lowest and the drive wheels a little above the line from the rear to the front:
	// it rests on the two pairs (four wheels, which hold it against roll), the drive wheels just clear.
	{
		TArray<FLegReach, TInlineAllocator<4>> Legs = ShapeLegs(0.0, 0.0, -3.0, 0.0);
		TestTrue(TEXT("every wheel down: the ground is found"), StanceGround(Legs, MakeArrayView(ShapeHalfTracks), Aft));
		SupportMask(Legs, TConstArrayView<bool>(), Tuning);
		const double Line = -3.0 + 3.0 * (DriveX - RearX) / (FrontX - RearX);
		TestEqual(TEXT("every wheel down: both drive wheels as far over the ground as each other"), Legs[DriveLeft].BaselineClearanceCm, Legs[DriveRight].BaselineClearanceCm, 1.0e-9);
		TestEqual(TEXT("... by their height over the line between the pairs"), Legs[DriveLeft].BaselineClearanceCm, 0.0 - Line, 1.0e-6);
		TestTrue(TEXT("... the pairs carry"), Legs[Rear].CarryShare >= 1.0 - 1.0e-9 && Legs[Front].CarryShare >= 1.0 - 1.0e-9);
		// Taken for single wheels on the centre line, the two pairs are a line: the ground then leans on one drive wheel.
		TArray<FLegReach, TInlineAllocator<4>> Single = ShapeLegs(0.0, 0.0, -3.0, 0.0);
		StanceGround(Single, TConstArrayView<double>(), Aft);
		TestTrue(TEXT("(taken for single wheels the same legs rest rolled onto a drive wheel)"), FMath::Abs(Single[DriveLeft].StanceGroundZ - Single[DriveRight].StanceGroundZ) > 0.5);
	}
	// The drive wheels set apart in height (a roll correction in the stance): it rests on both and on one wheel of the
	// rear pair, and the pair's leg is on the ground at that wheel.
	{
		TArray<FLegReach, TInlineAllocator<4>> Legs = ShapeLegs(-2.5, -6.5, -2.0, 4.0);
		TestTrue(TEXT("drive wheels apart: the ground is found"), StanceGround(Legs, MakeArrayView(ShapeHalfTracks), Aft));
		SupportMask(Legs, TConstArrayView<bool>(), Tuning);
		TestTrue(TEXT("drive wheels apart: three legs stand"), Legs[DriveLeft].bSupport && Legs[DriveRight].bSupport && Legs[Rear].bSupport);
		TestEqual(TEXT("... the rear pair on the ground at one of its wheels"), Legs[Rear].BaselineClearanceCm, 0.0, 1.0e-6);
	}
	// A robot whose stance stands level: nothing to give, and nothing changes when it is given.
	{
		TArray<FLegReach, TInlineAllocator<4>> Level = ShapeLegs(-3.0, -3.0, -3.0, 1.0);
		TArray<FLegReach, TInlineAllocator<4>> Plain = Level;
		StanceGround(Level, MakeArrayView(ShapeHalfTracks), Aft);
		SupportMask(Level, TConstArrayView<bool>(), Tuning);
		SupportMask(Plain, TConstArrayView<bool>(), Tuning);
		for (int32 Leg = 0; Leg < 4; ++Leg)
		{
			TestTrue(FString::Printf(TEXT("a level stance, leg %d: the same with and without"), Leg),
				Level[Leg].bSupport == Plain[Leg].bSupport && FMath::IsNearlyEqual(Level[Leg].BaselineClearanceCm, Plain[Leg].BaselineClearanceCm, 1.0e-9) && Level[Leg].CarryShare == Plain[Leg].CarryShare);
		}
	}
	// A centre of mass no three wheels hold (the robot would tip): no ground, every leg's left at zero.
	{
		TArray<FLegReach, TInlineAllocator<4>> Legs = ShapeLegs(-4.5, -4.5, -2.0, 4.0);
		TestFalse(TEXT("a centre of mass beyond the wheels: no ground"), StanceGround(Legs, MakeArrayView(ShapeHalfTracks), FVector2D(120.0, 0.0)));
		TestTrue(TEXT("... and every leg's is zero"), Legs[0].StanceGroundZ == 0.0 && Legs[1].StanceGroundZ == 0.0 && Legs[2].StanceGroundZ == 0.0 && Legs[3].StanceGroundZ == 0.0);
	}
	return true;
}

// Catches: a pitch or a roll of the wrong sign, the two swapped, a reading that changes with a mirroring scale on the
// body, and a reading stamped with the clock's time where the pose is older.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAttitudeBodySigns,
	"Ramms.Level.Attitude.BodySigns",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAttitudeBodySigns::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	FGen1Rig R(/*bWithLevel=*/false);
	if (!TestTrue(TEXT("the made-up robot"), R.IsReady()))
	{
		return false;
	}
	FBodyAttitudeSource Source(*R.Base, *R.Curb);
	struct FPose
	{
		double Pitch, Roll, Yaw;
	};
	for (const FPose& Pose : { FPose{ 0.0, 0.0, 0.0 }, FPose{ 4.0, 0.0, 0.0 }, FPose{ 0.0, 3.0, 0.0 }, FPose{ -6.0, 2.5, 37.0 }, FPose{ 5.0, -4.0, -120.0 } })
	{
		for (const bool bMirrored : { false, true })
		{
			const FString Case = FString::Printf(TEXT("pitch %.1f roll %.1f yaw %.0f%s"), Pose.Pitch, Pose.Roll, Pose.Yaw, bMirrored ? TEXT(", the body scaled -1 across") : TEXT(""));
			R.YawDeg = Pose.Yaw;
			R.Origin = FVector(310.0, -45.0, 62.0);
			R.PoseAt(FVector2D(Pose.Pitch, Pose.Roll), bMirrored);
			FAttitude Read;
			if (!TestTrue(*(Case + TEXT(": read")), Source.Read(Read)))
			{
				return false;
			}
			// (Pitched and rolled at once, the rig's own pose is a few hundredths of a degree off the roll it was asked.)
			TestEqual(*(Case + TEXT(": nose up is a positive pitch")), Read.Deg.X, Pose.Pitch, 0.02);
			TestEqual(*(Case + TEXT(": the +Y side up is a positive roll")), Read.Deg.Y, Pose.Roll, Pose.Pitch != 0.0 && Pose.Roll != 0.0 ? 0.05 : 0.02);
			TestTrue(*(Case + TEXT(": where the chassis is")), Read.Position.Equals(R.Origin, 1.0e-6));
			const double Yaw = FMath::DegreesToRadians(Pose.Yaw);
			TestTrue(*(Case + TEXT(": its heading, level")), Read.Heading.Equals(FVector2D(FMath::Cos(Yaw), FMath::Sin(Yaw)), 1.0e-6));
		}
	}
	// The reading is as old as the pose it was made from.
	R.Log->SimTime = 12.0;
	R.Log->TransformTime = 11.95;
	FAttitude Read;
	TestTrue(TEXT("read on a backend with a clock"), Source.Read(Read));
	TestEqual(TEXT("the reading's time is the pose's, not the clock's"), Read.TimeSeconds, 11.95, 1.0e-9);
	return true;
}

// Catches: a leg's height not its table's at its lift's angle; an offset of the wrong sign or size on a lift (the
// height it was asked would not be the height it is told), or one that is not exactly nothing for no height asked;
// the wheels' places taken from the model and not from the bodies (a drive wheel its translator has moved would be
// levelled as if it had not); a pair that roll reaches; a travel past the motor's own range; a rate that is not the
// lift's speed through its table; and a translator or a wheel written.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelGen1PlantTable,
	"Ramms.Level.Gen1Plant.TableRoundTrip",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelGen1PlantTable::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	FGen1Rig R(/*bWithLevel=*/false);
	if (!TestTrue(TEXT("the made-up robot"), R.IsReady()))
	{
		return false;
	}
	TUniquePtr<IPlant> Plant = MakeGen1Plant(*R.Base, *R.Curb, R.Layer());
	FString			   Why;
	if (!TestTrue(*FString::Printf(TEXT("the plant finds the robot (%s)"), *Why), Plant->Resolve(Why)))
	{
		return false;
	}
	TArray<FName> Ids;
	Plant->LevelledMotorIds(Ids);
	TestTrue(TEXT("the four lifts are levelled, and nothing else"), Ids == FGen1Rig::Lifts());
	TestFalse(TEXT("no seat"), Plant->HasSeat());
	TestTrue(TEXT("the layer is taken"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));

	// The drive wheels where their translators have them: the left one 12 cm forward.
	R.TranslatorCm[0] = 12.0;
	R.Pose();
	FPlantLegs Legs;
	if (!TestTrue(TEXT("the legs are measured"), Plant->MeasureLegs(Legs)) || !TestEqual(TEXT("four legs"), Legs.Num(), 4))
	{
		return false;
	}
	TestTrue(TEXT("the left drive wheel where its translator has it"), Legs[DriveLeft].XY.Equals(FVector2D(DriveX + 12.0, -DriveHalf), 1.0e-6));
	TestTrue(TEXT("the right one where it was"), Legs[DriveRight].XY.Equals(FVector2D(DriveX, DriveHalf), 1.0e-6));
	TestTrue(TEXT("the rear pair at the middle of its two wheels"), Legs[Rear].XY.Equals(FVector2D(RearX, 0.0), 1.0e-6));
	TestTrue(TEXT("the front pair at the middle of its two"), Legs[Front].XY.Equals(FVector2D(FrontX, 0.0), 1.0e-6));
	TestTrue(TEXT("roll reaches the drive legs only"), Legs[DriveLeft].bRoll && Legs[DriveRight].bRoll && !Legs[Rear].bRoll && !Legs[Front].bRoll);
	// Heights: the table's at the baseline, with the wheel's radius.
	TestEqual(TEXT("a drive leg's height: its table's at its command, and its radius"), Legs[DriveLeft].BaseZ, DriveHeight(StanceElevator) + DriveRadius, 1.0e-4);
	TestEqual(TEXT("the rear pair's"), Legs[Rear].BaseZ, RearHeight(StanceRear) + SmallRadius, 1.0e-4);
	TestEqual(TEXT("the front pair's"), Legs[Front].BaseZ, FrontHeight(StanceFront) + SmallRadius, 1.0e-4);
	TestEqual(TEXT("measured where the joint is"), Legs[DriveRight].MeasuredZ, Legs[DriveRight].BaseZ, 1.0e-6);
	// Travel: the table's, cut to the motor's own range (the elevators' +-0.6 of the table's +-0.7; the rear lift's
	// -0.4 of the table's -0.5).
	TestEqual(TEXT("a drive leg's lowest: its motor's range, not the table's end"), Legs[DriveLeft].MinZ, DriveHeight(0.6) + DriveRadius, 1.0e-4);
	TestEqual(TEXT("a drive leg's highest"), Legs[DriveLeft].MaxZ, DriveHeight(-0.6) + DriveRadius, 1.0e-4);
	TestEqual(TEXT("the rear pair's highest: its motor's range"), Legs[Rear].MaxZ, RearHeight(-0.4) + SmallRadius, 1.0e-4);
	// Rates: the lift's speed through its table.
	TestEqual(TEXT("a drive leg's rate: 18 deg/s through 30 cm a radian"), Legs[DriveLeft].MaxRateCmS, FMath::DegreesToRadians(18.0) * DrivePerRad, 1.0e-2);
	TestEqual(TEXT("the rear pair's: 26 deg/s through 20"), Legs[Rear].MaxRateCmS, FMath::DegreesToRadians(26.0) * PairPerRad, 1.0e-2);
	TestEqual(TEXT("the front pair's: 30 deg/s through 20"), Legs[Front].MaxRateCmS, FMath::DegreesToRadians(30.0) * PairPerRad, 1.0e-2);

	// No height asked is exactly no offset.
	const int32 Before = R.Log->Pushes.Num();
	Plant->ApplyOffsets(TArray<double>{ 0.0, 0.0, 0.0, 0.0 });
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("no height asked of %s is exactly no offset"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
	}
	// Heights asked: each lift is told the angle its table has for that height.
	const double Asked[4] = { 2.0, -1.5, 3.0, -2.5 };
	Plant->ApplyOffsets(MakeArrayView(Asked));
	for (int32 Leg = 0; Leg < 4; ++Leg)
	{
		TestEqual(*FString::Printf(TEXT("leg %d is told to the height asked"), Leg), R.Dz(Leg), Asked[Leg], 2.0e-3);
		TestTrue(*FString::Printf(TEXT("leg %d's own command stays"), Leg), SameBits(R.Baseline(Ids[Leg]), Leg == Rear ? StanceRear : Leg == Front ? StanceFront
																																				   : StanceElevator));
	}
	TestTrue(TEXT("a leg drawn up is a lift turned back (its table falls as its angle rises)"), R.Base->GetMotorCommandOffset(ElevatorL) < 0.0f && R.Base->GetMotorCommandOffset(ElevatorR) > 0.0f);
	// Past its travel: told to the end, and no further.
	const double Far[4] = { 40.0, 0.0, 0.0, 0.0 };
	Plant->ApplyOffsets(MakeArrayView(Far));
	TestEqual(TEXT("asked past its travel a lift goes to its end"), R.Effective(ElevatorL), -0.6f, 1.0e-4f);
	R.Tick();
	TestEqual(TEXT("neither translator nor any wheel was written"), R.OtherPushes(), 0);
	TestTrue(TEXT("(the lifts were)"), R.Log->Pushes.Num() > Before);

	// The frame's underside: four corners at the height the reference pose has them, each asking the tuning's clearance.
	FTuning Tuning;
	Tuning.MinBellyClearanceCm = 4.0;
	FPlantPoints Points;
	Plant->ClearancePoints(Tuning, Points);
	if (TestEqual(TEXT("four underside points"), Points.Num(), 4))
	{
		double MinX = 1.0e6, MaxX = -1.0e6, MaxY = 0.0;
		for (const FClearancePoint& Point : Points)
		{
			MinX = FMath::Min(MinX, Point.XY.X);
			MaxX = FMath::Max(MaxX, Point.XY.X);
			MaxY = FMath::Max(MaxY, FMath::Abs(Point.XY.Y));
			TestEqual(TEXT("an underside point at the frame's height over the reference ground"), Point.UndersideZ, 5.0, 1.0e-6);
			TestEqual(TEXT("... asking the tuning's clearance"), Point.MinCm, 4.0, 1.0e-9);
		}
		TestTrue(TEXT("the frame's box: from its back to its front, its half width either side"), FMath::IsNearlyEqual(MinX, -5.0) && FMath::IsNearlyEqual(MaxX, 55.0) && FMath::IsNearlyEqual(MaxY, 15.0));
	}
	return true;
}

// Catches: the plant leaving who stands to the lowest wheel bottom (this stance's rear pair, which carries, is then
// "lifted": two legs stand and the integral never runs); a ground worked out with the weight in the wrong place (the
// drive assemblies not taken where their translators have them); and a hand-back's stance judged on the ground of
// the pose it is not in.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelGen1PlantStands,
	"Ramms.Level.Gen1Plant.StandsOnTheGroundItRestsOn",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelGen1PlantStands::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	FGen1Rig R(/*bWithLevel=*/false);
	if (!TestTrue(TEXT("the made-up robot"), R.IsReady()))
	{
		return false;
	}
	TUniquePtr<IPlant> Plant = MakeGen1Plant(*R.Base, *R.Curb, R.Layer());
	FString			   Why;
	if (!TestTrue(TEXT("the plant finds the robot"), Plant->Resolve(Why)))
	{
		return false;
	}
	const FTuning Tuning;
	FPlantLegs	  Legs;
	TestTrue(TEXT("measured"), Plant->MeasureLegs(Legs));
	SupportMask(Legs, TConstArrayView<bool>(), Tuning);
	TestTrue(TEXT("in the stance: both drive wheels and the rear pair stand"), Legs[DriveLeft].bSupport && Legs[DriveRight].bSupport && Legs[Rear].bSupport);
	TestFalse(TEXT("... and the raised front pair does not"), Legs[Front].bSupport);
	TestTrue(TEXT("... carried clear by more than its 4 cm (the stance leans nose up)"), Legs[Front].BaselineClearanceCm > 8.0);

	// The translators take the drive wheels 30 cm aft: the weight is then ahead of them, and with the front pair let
	// down the robot rests on the drive wheels and the front pair.
	R.TranslatorCm[0] = R.TranslatorCm[1] = -30.0;
	R.Base->SetMotorCommand(FrontArm, 0.15f);
	R.Tick();
	R.Pose();
	TestTrue(TEXT("measured again"), Plant->MeasureLegs(Legs));
	SupportMask(Legs, TConstArrayView<bool>(), Tuning);
	TestTrue(TEXT("drive wheels aft, the front pair down: it stands on the drive wheels and the front pair"), Legs[DriveLeft].bSupport && Legs[DriveRight].bSupport && Legs[Front].bSupport);
	TestFalse(TEXT("... the rear pair off the ground"), Legs[Rear].bSupport);

	// The drive assemblies are a good part of the weight, and go with their wheels. Here the chassis' own weight is
	// 5 cm ahead of drive wheels the translators have taken 20 cm forward, and the two assemblies are on their hubs:
	// the weight is ahead of the drive wheels, and the robot rests on them and the front pair. (Counted where the
	// model has the hubs with the translators at zero, the assemblies would put the weight behind the drive wheels.)
	R.TranslatorCm[0] = R.TranslatorCm[1] = 20.0;
	R.Curb->Config.FixedComAheadCm = static_cast<float>(DriveX + 20.0 + 5.0);
	R.Curb->Config.DriveAssemblyMassKg = 20.0f;
	R.Pose();
	TestTrue(TEXT("measured with the drive wheels forward"), Plant->MeasureLegs(Legs));
	SupportMask(Legs, TConstArrayView<bool>(), Tuning);
	TestTrue(TEXT("drive wheels forward, the weight just ahead of them: it stands on the drive wheels and the front pair"), Legs[DriveLeft].bSupport && Legs[DriveRight].bSupport && Legs[Front].bSupport && !Legs[Rear].bSupport);
	R.Curb->Config.FixedComAheadCm = 25.0f;
	R.Curb->Config.DriveAssemblyMassKg = 10.0f;
	R.Base->SetMotorCommand(FrontArm, StanceFront);

	// Back in the stance, and a pose that is the stance with a plane on it (as a fold leaves the lifts' commands): who
	// stands in the STANCE is judged on the ground the stance rests on.
	R.TranslatorCm[0] = R.TranslatorCm[1] = 0.0;
	R.Stand();
	const double StanceZ[4] = { DriveHeight(StanceElevator) + DriveRadius, DriveHeight(StanceElevator) + DriveRadius, RearHeight(StanceRear) + SmallRadius, FrontHeight(StanceFront) + SmallRadius };
	// A 5 deg nose-down plane in the commands: the drive legs let down, the rear pair drawn up.
	R.Base->SetMotorCommand(ElevatorL, StanceElevator + 0.1f);
	R.Base->SetMotorCommand(ElevatorR, StanceElevator + 0.1f);
	R.Base->SetMotorCommand(RearLift, StanceRear - 0.15f);
	R.Tick();
	TestTrue(TEXT("measured in the pose"), Plant->MeasureLegs(Legs));
	FPlantLegs InStance = Legs;
	for (int32 Leg = 0; Leg < 4; ++Leg)
	{
		InStance[Leg].BaseZ = StanceZ[Leg];
	}
	FPlantLegs OnThePosesGround = InStance;
	SupportMask(OnThePosesGround, TConstArrayView<bool>(), Tuning);
	TestFalse(TEXT("(on the pose's ground the stance's legs do not all read as standing)"), OnThePosesGround[DriveLeft].bSupport && OnThePosesGround[Rear].bSupport);
	Plant->RestLegsAt(MakeArrayView(StanceZ), InStance);
	SupportMask(InStance, TConstArrayView<bool>(), Tuning);
	TestTrue(TEXT("on the stance's own ground: both drive wheels and the rear pair stand in it"), InStance[DriveLeft].bSupport && InStance[DriveRight].bSupport && InStance[Rear].bSupport && !InStance[Front].bSupport);
	return true;
}

// Catches: a fold that tells a lift anything, or leaves an offset on; an unfold that is not exact; the heights before
// a fold forgotten; and a rebase that steps a lift, or leaves offsets that are not the heights taken out. (That the
// lift controller is told its lifts' commands moved under it is Ramms.Level.Assist.Gen1.LiftsJoggedByTheirRiderUnderThePlane's.)
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelGen1PlantFold,
	"Ramms.Level.Gen1Plant.FoldUnfoldAndRebase",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelGen1PlantFold::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	FGen1Rig R(/*bWithLevel=*/false);
	if (!TestTrue(TEXT("the made-up robot"), R.IsReady()))
	{
		return false;
	}
	TUniquePtr<IPlant> Plant = MakeGen1Plant(*R.Base, *R.Curb, R.Layer());
	FString			   Why;
	TArray<FName>	   Ids;
	if (!TestTrue(TEXT("the plant finds the robot"), Plant->Resolve(Why)))
	{
		return false;
	}
	Plant->LevelledMotorIds(Ids);
	TestTrue(TEXT("the layer is taken"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	FPlantLegs Legs;
	Plant->MeasureLegs(Legs);
	const double Asked[4] = { 2.0, -1.5, 3.0, -2.5 };
	Plant->ApplyOffsets(MakeArrayView(Asked));
	R.Tick();
	TMap<FName, float> Told, Offsets;
	for (const FName& Id : Ids)
	{
		Told.Add(Id, R.Effective(Id));
		Offsets.Add(Id, R.Base->GetMotorCommandOffset(Id));
	}
	const int32 Before = R.LiftPushes();

	// Fold: what each lift is told stays, to the bit; it is all its own command now.
	Plant->Fold();
	R.Tick();
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("fold: %s is told what it was, to the bit"), *Id.ToString()), SameBits(R.Effective(Id), Told[Id]));
		TestTrue(*FString::Printf(TEXT("fold: %s's own command is that now"), *Id.ToString()), SameBits(R.Baseline(Id), Told[Id]) && R.Base->GetMotorCommandOffset(Id) == 0.0f);
	}
	TestEqual(TEXT("fold: no lift was told anything"), R.LiftPushes(), Before);
	// The heights from before the fold are remembered: the stance's.
	FLegValues Folded;
	if (TestTrue(TEXT("the heights before the fold are remembered"), Plant->FoldedHeights(Folded)) && TestEqual(TEXT("... four"), Folded.Num(), 4))
	{
		TestEqual(TEXT("... a drive leg's: the stance's"), Folded[DriveLeft], DriveHeight(StanceElevator) + DriveRadius, 1.0e-4);
		TestEqual(TEXT("... the rear pair's"), Folded[Rear], RearHeight(StanceRear) + SmallRadius, 1.0e-4);
		TestEqual(TEXT("... the front pair's"), Folded[Front], FrontHeight(StanceFront) + SmallRadius, 1.0e-4);
	}
	// Unfold, nothing written since: exactly as it was.
	TestTrue(TEXT("unfold, nothing written since"), Plant->Unfold());
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("unfold: %s's offset as it was, to the bit"), *Id.ToString()), SameBits(R.Base->GetMotorCommandOffset(Id), Offsets[Id]));
		TestTrue(*FString::Printf(TEXT("unfold: %s told what it was"), *Id.ToString()), SameBits(R.Effective(Id), Told[Id]));
	}
	TestFalse(TEXT("a second unfold has nothing to undo"), Plant->Unfold());
	// Folded again, then a lift written by whoever took them: the fold stands.
	Plant->Fold();
	R.Base->SetMotorCommand(RearLift, 0.05f);
	TestFalse(TEXT("unfold after a lift was written: refused"), Plant->Unfold());
	R.Tick();

	// Rebase: each leg's own command moved to a height, what it is told staying; the rest is its offset.
	Plant->MeasureLegs(Legs);
	TMap<FName, float> ToldNow;
	for (const FName& Id : Ids)
	{
		ToldNow.Add(Id, R.Effective(Id));
	}
	const double Stance[4] = { DriveHeight(StanceElevator) + DriveRadius, DriveHeight(StanceElevator) + DriveRadius, RearHeight(StanceRear) + SmallRadius, FrontHeight(StanceFront) + SmallRadius };
	FLegValues	 Left;
	const int32	 BeforeRebase = R.LiftPushes();
	if (TestTrue(TEXT("rebase to the stance's heights"), Plant->RebaseLegs(MakeArrayView(Stance), Left)) && TestEqual(TEXT("... four offsets left"), Left.Num(), 4))
	{
		R.Tick();
		for (int32 Leg = 0; Leg < 4; ++Leg)
		{
			TestTrue(*FString::Printf(TEXT("rebase: leg %d is told what it was"), Leg), FMath::IsNearlyEqual(R.Effective(Ids[Leg]), ToldNow[Ids[Leg]], 1.0e-6f));
			TestEqual(*FString::Printf(TEXT("rebase: leg %d's own command is the stance's height"), Leg), R.Height(Leg, false) + (Leg <= DriveRight ? DriveRadius : SmallRadius), Stance[Leg], 2.0e-3);
			TestEqual(*FString::Printf(TEXT("rebase: leg %d's offset is the height taken out"), Leg), Left[Leg], R.Dz(Leg), 1.0e-4);
		}
		TestEqual(TEXT("rebase: no lift was told anything"), R.LiftPushes(), BeforeRebase);
	}
	return true;
}

// Catches: the backstop for a take-over nobody announced putting an offset into a command that was rewritten since
// the tick before (that command is where its writer wants the lift: the offset on top of it would be an error), or
// dropping one from a command that was not (the lift would step back by the whole plane); and a lift whose offset
// was cleared under the plant not noticed, so that it would be told its command bare.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelGen1PlantBackstop,
	"Ramms.Level.Gen1Plant.BackstopFoldsOrDropsAndALostOffsetIsSeen",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelGen1PlantBackstop::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	FGen1Rig R(/*bWithLevel=*/false);
	if (!TestTrue(TEXT("the made-up robot"), R.IsReady()))
	{
		return false;
	}
	TUniquePtr<IPlant> Plant = MakeGen1Plant(*R.Base, *R.Curb, R.Layer());
	FString			   Why;
	TArray<FName>	   Ids;
	if (!TestTrue(TEXT("the plant finds the robot"), Plant->Resolve(Why)))
	{
		return false;
	}
	Plant->LevelledMotorIds(Ids);
	TestTrue(TEXT("the layer is taken"), R.Base->ClaimCommandOffsetLayer(R.Layer(), Ids));
	FPlantLegs Legs;
	Plant->MeasureLegs(Legs);
	const double Asked[4] = { 2.0, -1.5, 3.0, -2.5 };
	Plant->ApplyOffsets(MakeArrayView(Asked));
	R.Tick();
	Plant->MeasureLegs(Legs);
	TestFalse(TEXT("every lift carries the offset it was given"), Plant->OffsetsLost());

	// Something clears one lift's offset under the plant.
	const float Given = R.Base->GetMotorCommandOffset(ElevatorR);
	R.Base->SetMotorCommandOffset(ElevatorR, 0.0f, R.Layer());
	Plant->MeasureLegs(Legs);
	TestTrue(TEXT("a lift whose offset was cleared is seen"), Plant->OffsetsLost());
	Plant->ApplyOffsets(MakeArrayView(Asked));
	TestTrue(TEXT("... and has it again once the offsets are applied"), SameBits(R.Base->GetMotorCommandOffset(ElevatorR), Given));
	R.Tick();
	Plant->MeasureLegs(Legs);
	TestFalse(TEXT("... nothing lost any more"), Plant->OffsetsLost());

	// A mode takes the lifts without a word: it has rewritten the rear lift's command this tick, and not the others.
	TMap<FName, float> Told;
	for (const FName& Id : Ids)
	{
		Told.Add(Id, R.Effective(Id));
	}
	R.Base->SetMotorCommand(RearLift, 0.07f);
	Plant->MeasureLegs(Legs);
	const int32 Before = R.LiftPushes();
	int32		Folded = 0, Dropped = 0;
	Plant->FoldOrZero(Folded, Dropped);
	TestEqual(TEXT("three lifts' offsets went into their commands"), Folded, 3);
	TestEqual(TEXT("one was dropped"), Dropped, 1);
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s carries no offset any more"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
		if (Id == RearLift)
		{
			TestTrue(TEXT("the rewritten rear lift's command is as its writer left it"), SameBits(R.Baseline(Id), 0.07f));
		}
		else
		{
			TestTrue(*FString::Printf(TEXT("%s, not rewritten: its command is now what it was told, to the bit"), *Id.ToString()), SameBits(R.Baseline(Id), Told[Id]));
		}
	}
	TestEqual(TEXT("no lift was told anything by it"), R.LiftPushes(), Before);
	return true;
}

// Catches: the component not finding a robot of this kind (no plant: it cannot be levelled), a pitch or a roll
// correction of the wrong sign on the lifts (the base would run away from level), roll given to a pair, the roll
// allowed (6 deg) not kept to, and a translator or a wheel written by levelling.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistGen1Levels,
	"Ramms.Level.Assist.Gen1.LevelsPitchAndRollOnTheLifts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistGen1Levels::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	{
		FGen1Rig R;
		if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr && R.Level->HasBegunPlay()))
		{
			return false;
		}
		// On the flat its stance stands two degrees nose up; switched on, that goes too.
		R.Frames(3);
		TestEqual(TEXT("off: the stance's own lean"), R.AttitudeDeg().X, FMath::RadiansToDegrees(FMath::Atan(2.5 / 70.0)), 1.0e-3);
		TestEqual(TEXT("off: nothing told"), R.LiftPushes(), 4);
		R.Level->SetAssistEnabled(true);
		R.Frames(240);
		TestTrue(*FString::Printf(TEXT("on the flat: levelling (state %d, '%s')"), static_cast<int32>(R.State()), *R.Status()), R.State() == EState::Active);
		TestTrue(*FString::Printf(TEXT("on the flat: level (%.3f, %.3f deg)"), R.AttitudeDeg().X, R.AttitudeDeg().Y), R.AttitudeDeg().Size() <= 0.1);

		// A grade with a cross fall.
		R.GroundDeg = FVector2D(3.0, -2.0);
		R.Pose();
		R.Frames(300);
		TestTrue(*FString::Printf(TEXT("on (3, -2) deg: level (%.3f, %.3f deg)"), R.AttitudeDeg().X, R.AttitudeDeg().Y), R.AttitudeDeg().Size() <= 0.1);
		// Nose up: the drive legs drawn up against the rear pair. +Y side down: the +Y drive leg let down, the other up.
		TestTrue(TEXT("the nose brought down: the drive legs drawn up against the rear pair"), 0.5 * (R.Dz(DriveLeft) + R.Dz(DriveRight)) - R.Dz(Rear) > 3.0);
		TestTrue(TEXT("the +Y side brought up: its drive leg let down, the other drawn up"), R.Dz(DriveRight) < R.Dz(DriveLeft) - 1.5);
		TestEqual(TEXT("the ride height kept: the standing legs' offsets about nothing"), (R.Dz(DriveLeft) + R.Dz(DriveRight) + R.Dz(Rear)) / 3.0, 0.0, 0.05);
		TestEqual(TEXT("neither translator nor any wheel was written"), R.OtherPushes(), 0);
	}
	{
		// Across a slope alone: the roll is the two elevators', and the pairs are left where they are.
		FGen1Rig R;
		R.GroundDeg = FVector2D(-FMath::RadiansToDegrees(FMath::Atan(2.5 / 70.0)), 4.0);
		R.Pose();
		R.Level->SetAssistEnabled(true);
		R.Frames(300);
		TestTrue(*FString::Printf(TEXT("across 4 deg: level (%.3f, %.3f deg)"), R.AttitudeDeg().X, R.AttitudeDeg().Y), R.AttitudeDeg().Size() <= 0.1);
		TestEqual(TEXT("across 4 deg: the +Y drive leg drawn up by half the track at that roll"), R.Dz(DriveRight), DriveHalf * FMath::Tan(FMath::DegreesToRadians(4.0)), 0.1);
		TestEqual(TEXT("... the other let down as far"), R.Dz(DriveLeft), -DriveHalf * FMath::Tan(FMath::DegreesToRadians(4.0)), 0.1);
		TestTrue(*FString::Printf(TEXT("... and neither pair moved for it (%.3f, %.3f cm)"), R.Dz(Rear), R.Dz(Front)), FMath::Abs(R.Dz(Rear)) <= 0.05 && FMath::Abs(R.Dz(Front)) <= 0.05);
	}
	{
		// Across more than the roll allowed: held at 6 deg, the rest left, and said.
		FGen1Rig R;
		R.GroundDeg = FVector2D(-FMath::RadiansToDegrees(FMath::Atan(2.5 / 70.0)), -9.0);
		R.Pose();
		R.Level->SetAssistEnabled(true);
		R.Frames(450);
		const double RollGiven = FMath::RadiansToDegrees(FMath::Atan((R.Dz(DriveRight) - R.Dz(DriveLeft)) / (2.0 * DriveHalf)));
		TestEqual(TEXT("across 9 deg: the roll correction held at the 6 allowed"), RollGiven, -6.0, 0.1);
		TestEqual(TEXT("... the base left the other 3"), R.AttitudeDeg().Y, -3.0, 0.15);
		TestTrue(*FString::Printf(TEXT("... and it says so ('%s')"), *R.Status()), R.Status().Contains(TEXT("reach")));
	}
	return true;
}

// Catches: a crossing that reads its lifts' commands before the level assist has put its plane into them (it would
// start from the stance, and step every lift by the plane on its first write); one that tells the level assist only
// once it has begun, a tick after its drive mode took the lifts (the level assist's own tick then takes the mode for a
// take-over nobody announced, and the commands as they were are lost to the hand-back); a hand-back that leaves the
// plane in the lifts' commands (switching off would not give the stance back, and the lift controller would hold the
// tilted pose as the rider's); a hand-back made after the mode is switched; and any lift told anything by either.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistGen1Crossing,
	"Ramms.Level.Assist.Gen1.CrossingTakesTheLiftsAndHandsThemBack",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistGen1Crossing::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	FGen1Rig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr))
	{
		return false;
	}
	IRammsDriveMode* CurbMode = R.Curb;
	IRammsDriveMode* DriveMode = R.Drive;
	R.GroundDeg = FVector2D(3.0, -2.0);
	R.Pose();
	R.Level->SetAssistEnabled(true);
	R.Frames(300);
	const TArray<FName> Ids = FGen1Rig::Lifts();
	if (!TestTrue(TEXT("levelled on the slope"), R.State() == EState::Active && R.AttitudeDeg().Size() <= 0.1))
	{
		return false;
	}
	TMap<FName, float> Told, Offsets;
	for (const FName& Id : Ids)
	{
		Told.Add(Id, R.Effective(Id));
		Offsets.Add(Id, R.Base->GetMotorCommandOffset(Id));
	}
	TestTrue(TEXT("(with offsets on the lifts)"), FMath::Abs(Offsets[ElevatorL]) > 0.02f && FMath::Abs(Offsets[RearLift]) > 0.02f);
	const int32 Before = R.LiftPushes();

	// A crossing is committed: the curb mode has the robot from here, and begins on its next tick.
	FString Why;
	if (!TestTrue(*FString::Printf(TEXT("a crossing is committed (%s)"), *Why), FRammsRammpGen1CurbTestAccess::Commit(*R.Curb, Why)))
	{
		return false;
	}
	TestTrue(TEXT("the curb mode has the robot"), CurbMode->IsDriveModeActive() && !DriveMode->IsDriveModeActive());
	TestTrue(TEXT("the level assist stands by from the moment it is committed"), R.State() == EState::Yielded);
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("committed: %s's own command is what it was told, to the bit, with nothing on top"), *Id.ToString()),
			SameBits(R.Baseline(Id), Told[Id]) && R.Base->GetMotorCommandOffset(Id) == 0.0f);
	}
	// The frame between the commit and the crossing's first tick: the level assist's own tick sees the curb mode with
	// the lifts, and has already been told.
	R.Frame();
	TestTrue(TEXT("a frame on: standing by still"), R.State() == EState::Yielded);
	// The crossing begins: it finds each lift at the levelled pose, each elevator's own.
	R.CurbTick();
	TestEqual(TEXT("the crossing found the left elevator where levelling had it"), FRammsRammpGen1CurbTestAccess::StartCommand(*R.Curb, FRammsRammpGen1CurbTestAccess::ElevatorL), Told[ElevatorL], 1.0e-6f);
	TestEqual(TEXT("... the right one at its own"), FRammsRammpGen1CurbTestAccess::StartCommand(*R.Curb, FRammsRammpGen1CurbTestAccess::ElevatorR), Told[ElevatorR], 1.0e-6f);
	TestEqual(TEXT("... the rear lift"), FRammsRammpGen1CurbTestAccess::StartCommand(*R.Curb, FRammsRammpGen1CurbTestAccess::RearLiftLift), Told[RearLift], 1.0e-6f);
	TestEqual(TEXT("... the front arm"), FRammsRammpGen1CurbTestAccess::StartCommand(*R.Curb, FRammsRammpGen1CurbTestAccess::FrontArmLift), Told[FrontArm], 1.0e-6f);
	TestTrue(TEXT("(the two elevators apart: the roll is in them)"), FMath::Abs(Told[ElevatorL] - Told[ElevatorR]) > 0.02f);
	R.Frame();
	TestEqual(TEXT("no lift was told anything by the take-over"), R.LiftPushes(), Before);

	// The crossing ends with every lift back where it found it (here: never moved), and hands the robot back.
	if (FRammsRammpGen1CurbTestAccess::IsCrossing(*R.Curb))
	{
		FRammsRammpGen1CurbTestAccess::Finish(*R.Curb, /*bAborted=*/false, TEXT("the test's crossing is over"));
	}
	R.CurbTick();
	TestTrue(TEXT("the robot is handed back to the differential mode"), DriveMode->IsDriveModeActive() && !CurbMode->IsDriveModeActive());
	TestTrue(*FString::Printf(TEXT("levelling again at once (state %d)"), static_cast<int32>(R.State())), R.State() == EState::Active);
	// The plane is back out of the lifts' commands: each lift's own command is the stance's, its offset the plane's.
	TestEqual(TEXT("the elevators' own commands are the stance's again"), R.Baseline(ElevatorL), StanceElevator, 2.0e-4f);
	TestEqual(TEXT("... both"), R.Baseline(ElevatorR), StanceElevator, 2.0e-4f);
	TestEqual(TEXT("... the rear lift's"), R.Baseline(RearLift), StanceRear, 2.0e-4f);
	TestEqual(TEXT("... the front arm's"), R.Baseline(FrontArm), StanceFront, 2.0e-4f);
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("handed back: %s is told what it was"), *Id.ToString()), FMath::IsNearlyEqual(R.Effective(Id), Told[Id], 1.0e-6f));
	}
	TestEqual(TEXT("no lift was told anything by the hand-back"), R.LiftPushes(), Before);
	R.Frames(60);
	TestTrue(*FString::Printf(TEXT("level a while on (%.3f deg)"), R.AttitudeDeg().Size()), R.AttitudeDeg().Size() <= 0.1);
	// Switched off, the stance comes back exactly.
	R.Level->SetAssistEnabled(false);
	R.Frames(400);
	TestTrue(TEXT("switched off: off"), R.State() == EState::Off);
	for (const FName& Id : Ids)
	{
		TestEqual(*FString::Printf(TEXT("switched off: %s back at the stance"), *Id.ToString()), R.Effective(Id), Id == RearLift ? StanceRear : Id == FrontArm ? StanceFront
																																							   : StanceElevator,
			2.0e-4f);
	}
	return true;
}

// Catches: a hand-back from a crossing that left the lifts moved (stopped part way, the chassis raised for the curb)
// starting again from nothing or stepping a lift, where the plane the pose makes over the stance is taken up and
// levelled on from; a second crossing started while the first's mode still holds the lifts counted as a take-over
// inside a take-over (the one hand-back would then end only the inner one, and levelling never come back); and a
// crossing after the curb mode was left by hand finding the level assist never told.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistGen1CrossingMoved,
	"Ramms.Level.Assist.Gen1.CrossingThatMovedTheLiftsIsLevelledOnFrom",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistGen1CrossingMoved::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	FGen1Rig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr))
	{
		return false;
	}
	IRammsDriveMode* CurbMode = R.Curb;
	IRammsDriveMode* DriveMode = R.Drive;
	R.GroundDeg = FVector2D(3.0, -2.0);
	R.Pose();
	R.Level->SetAssistEnabled(true);
	R.Frames(300);
	const TArray<FName> Ids = FGen1Rig::Lifts();
	if (!TestTrue(TEXT("levelled on the slope"), R.State() == EState::Active && R.AttitudeDeg().Size() <= 0.1))
	{
		return false;
	}
	FString Why;
	TestTrue(TEXT("a crossing is committed"), FRammsRammpGen1CurbTestAccess::Commit(*R.Curb, Why));
	R.Frame();
	R.CurbTick();
	// The crossing raises the chassis for the curb (both elevators let down further) and trims its rear lift, then
	// stops where it is.
	R.Base->SetMotorCommand(ElevatorL, R.Baseline(ElevatorL) + 0.05f);
	R.Base->SetMotorCommand(ElevatorR, R.Baseline(ElevatorR) + 0.05f);
	R.Base->SetMotorCommand(RearLift, R.Baseline(RearLift) + 0.02f);
	R.Frame();
	TestTrue(TEXT("standing by through the crossing"), R.State() == EState::Yielded);
	FRammsRammpGen1CurbTestAccess::Finish(*R.Curb, /*bAborted=*/true, TEXT("the test's crossing stopped"));
	// Begun again where it stopped, while this mode still has the lifts: one take-over still, not two.
	TestTrue(TEXT("a crossing is committed again where the first stopped"), FRammsRammpGen1CurbTestAccess::Commit(*R.Curb, Why));
	R.Frame();
	R.CurbTick();
	if (FRammsRammpGen1CurbTestAccess::IsCrossing(*R.Curb))
	{
		FRammsRammpGen1CurbTestAccess::Finish(*R.Curb, /*bAborted=*/true, TEXT("the test's second crossing stopped"));
	}
	TMap<FName, float> Told;
	for (const FName& Id : Ids)
	{
		Told.Add(Id, R.Effective(Id));
	}
	const int32 Before = R.LiftPushes();
	R.CurbTick();
	TestTrue(TEXT("the robot is handed back"), DriveMode->IsDriveModeActive() && !CurbMode->IsDriveModeActive());
	TestTrue(*FString::Printf(TEXT("levelling again at once, from the pose the crossing left (state %d)"), static_cast<int32>(R.State())), R.State() == EState::Active);
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("handed back: %s is told what it was"), *Id.ToString()), FMath::IsNearlyEqual(R.Effective(Id), Told[Id], 1.0e-6f));
	}
	TestEqual(TEXT("no lift was told anything by the hand-back"), R.LiftPushes(), Before);
	// The plane the pose makes over the stance is the level assist's again, the two elevators' difference in it.
	TestTrue(TEXT("the roll is out of the elevators' own commands"), FMath::Abs(R.Baseline(ElevatorL) - R.Baseline(ElevatorR)) <= 2.0e-4f);
	TestTrue(TEXT("... and on them as offsets"), R.Dz(DriveRight) < R.Dz(DriveLeft) - 1.5);
	// What the crossing left the base off level by is levelled out from there.
	R.Frames(150);
	TestTrue(*FString::Printf(TEXT("level again a while on (%.3f deg)"), R.AttitudeDeg().Size()), R.AttitudeDeg().Size() <= 0.1);

	// The curb mode left by hand in the middle of a crossing: no hand-back is made, and the level assist starts again
	// from the pose as it stands. The next crossing tells it as the first did.
	TestTrue(TEXT("a third crossing is committed"), FRammsRammpGen1CurbTestAccess::Commit(*R.Curb, Why));
	R.Frame();
	R.CurbTick();
	AddExpectedMessagePlain(TEXT("curb crossing aborted"), ELogVerbosity::Warning, EAutomationExpectedMessageFlags::Contains, 0);
	CurbMode->SetDriveModeActive(false);
	DriveMode->SetDriveModeActive(true);
	R.Frames(150);
	TestTrue(TEXT("left by hand: levelling again"), R.State() == EState::Active && R.AttitudeDeg().Size() <= 0.15);
	TestTrue(TEXT("a fourth crossing is committed"), FRammsRammpGen1CurbTestAccess::Commit(*R.Curb, Why));
	TestTrue(TEXT("... and the level assist stands by for it, told as before"), R.State() == EState::Yielded);
	R.Frame();
	R.Curb->Abort(TEXT("the test is over"));
	R.CurbTick();
	R.CurbTick();
	return true;
}

// Catches: a crossing that ends on other ground than it began on (a kerb between a road's camber and a pavement's
// fall) handed back with the law as it stood on the ground it left: the law would take the base's new attitude for a
// knock and creep off it, the rider left tilted for seconds. Handed back as any other hand-back, the rider is walked
// to level at the tilt policy's pace -- with no lift stepped (a ride height the legs carried at the take-over
// included), and, where the crossing put every lift back, with the lifts' own commands still the stance's to the bit.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistGen1CrossingOtherGround,
	"Ramms.Level.Assist.Gen1.CrossingOntoOtherGroundIsLevelledAtTheRidersPace",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistGen1CrossingOtherGround::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	struct FCase
	{
		const TCHAR* Name;
		/** The crossing leaves every lift where it found it; else its rear lift trimmed. */
		bool bLiftsPutBack;
		/** The ground it starts on is past what the rear lift's travel levels: the chassis is let up for it. */
		bool	  bRaised;
		FVector2D NearDeg;
		FVector2D FarDeg;
	};
	const FCase Cases[3] = { { TEXT("every lift put back where it was found"), true, false, FVector2D(3.0, -2.0), FVector2D(-1.0, 2.0) },
		{ TEXT("the rear lift left trimmed"), false, false, FVector2D(3.0, -2.0), FVector2D(-1.0, 2.0) },
		{ TEXT("every lift put back, the chassis let up for the slope"), true, true, FVector2D(-13.0, 0.0), FVector2D(-7.0, 1.0) } };
	for (const FCase& Case : Cases)
	{
		const FString Name = Case.Name;
		FGen1Rig	  R;
		if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr))
		{
			return false;
		}
		const float			Dt = 1.0f / 30.0f;
		const TArray<FName> Ids = FGen1Rig::Lifts();
		R.Level->Tuning.RideHeightUpCm = Case.bRaised ? 3.0f : 0.0f;
		R.GroundDeg = Case.NearDeg;
		R.Pose();
		R.Level->SetAssistEnabled(true);
		R.Frames(400, Dt);
		if (!TestTrue(*(Name + TEXT(": levelling on the near ground")), R.State() == EState::Active && (Case.bRaised || R.AttitudeDeg().Size() <= 0.1)))
		{
			return false;
		}
		// (Negative: every standing leg let down, the chassis up.)
		const double RideCm = (R.Dz(DriveLeft) + R.Dz(DriveRight) + R.Dz(Rear)) / 3.0;
		if (Case.bRaised)
		{
			TestTrue(*(Name + FString::Printf(TEXT(": (the legs carry a ride height: the standing legs' offsets are %.2f cm about)"), RideCm)), RideCm < -0.5);
		}
		FString Why;
		TestTrue(*(Name + TEXT(": a crossing is committed")), FRammsRammpGen1CurbTestAccess::Commit(*R.Curb, Why));
		R.Frame(Dt);
		R.CurbTick(Dt);
		// The crossing carries the robot onto ground that lies another way.
		R.GroundDeg = Case.FarDeg;
		if (!Case.bLiftsPutBack)
		{
			R.Base->SetMotorCommand(RearLift, R.Baseline(RearLift) + 0.02f);
		}
		R.Frames(10, Dt);
		const FVector2D Left = R.AttitudeDeg();
		TestTrue(*(Name + FString::Printf(TEXT(": (the far ground leaves the base %.2f deg off level)"), Left.Size())), Left.Size() > 3.0);
		if (FRammsRammpGen1CurbTestAccess::IsCrossing(*R.Curb))
		{
			FRammsRammpGen1CurbTestAccess::Finish(*R.Curb, /*bAborted=*/false, TEXT("the test's crossing is over"));
		}
		TMap<FName, float> Told;
		for (const FName& Id : Ids)
		{
			Told.Add(Id, R.Effective(Id));
		}
		R.CurbTick(Dt);
		TestTrue(*(Name + TEXT(": handed back, levelling")), R.State() == EState::Active);
		for (const FName& Id : Ids)
		{
			TestTrue(*(Name + FString::Printf(TEXT(": handed back, %s is told what it was"), *Id.ToString())), FMath::IsNearlyEqual(R.Effective(Id), Told[Id], 1.0e-6f));
		}
		if (Case.bLiftsPutBack)
		{
			TestTrue(*(Name + TEXT(": the lifts' own commands are the stance's, to the bit")),
				SameBits(R.Baseline(ElevatorL), StanceElevator) && SameBits(R.Baseline(ElevatorR), StanceElevator) && SameBits(R.Baseline(RearLift), StanceRear) && SameBits(R.Baseline(FrontArm), StanceFront));
		}
		// From there to level: how fast the rider is turned over any 0.3 s, how long it takes, and the most any lift's
		// wheels are told to move in a frame.
		TArray<FVector2D> Trace;
		Trace.Add(R.AttitudeDeg());
		double Fastest = 0.0;
		double LargestStepCm = 0.0;
		int32  LevelAfter = INDEX_NONE;
		for (int32 Index = 1; Index <= 300; ++Index)
		{
			double Before[4];
			for (int32 Leg = 0; Leg < 4; ++Leg)
			{
				Before[Leg] = R.Height(Leg, true);
			}
			R.Frame(Dt);
			for (int32 Leg = 0; Leg < 4; ++Leg)
			{
				LargestStepCm = FMath::Max(LargestStepCm, FMath::Abs(R.Height(Leg, true) - Before[Leg]));
			}
			Trace.Add(R.AttitudeDeg());
			if (Index >= 9)
			{
				Fastest = FMath::Max(Fastest, (Trace[Index] - Trace[Index - 9]).Size() / (9 * Dt));
			}
			if (Trace[Index].Size() > (Case.bRaised ? 0.3 : 0.2))
			{
				LevelAfter = INDEX_NONE;
			}
			else if (LevelAfter == INDEX_NONE)
			{
				LevelAfter = Index;
			}
		}
		AddInfo(FString::Printf(TEXT("%s: %.2f deg off level at the hand-back, the rider turned at up to %.2f deg/s, level after %.2f s; a lift's wheels told to move %.3f cm in a frame at most"), *Name, Trace[0].Size(),
			Fastest, LevelAfter * Dt, LargestStepCm));
		TestTrue(*(Name + FString::Printf(TEXT(": no step of the base at the hand-back (%.2f deg, was %.2f)"), Trace[0].Size(), Left.Size())), (Trace[0] - Left).Size() <= 0.05);
		// (The lifts move their wheels at some 10 cm/s at most: a third of a centimetre in a frame.)
		TestTrue(*(Name + FString::Printf(TEXT(": no lift stepped afterwards (%.3f cm in a frame)"), LargestStepCm)), LargestStepCm <= 0.4);
		TestTrue(*(Name + FString::Printf(TEXT(": the rider is turned to level within the tilt policy's rate (%.2f deg/s)"), Fastest)), Fastest <= R.Level->Tuning.TiltRateDegPerSec + 0.5);
		TestTrue(*(Name + FString::Printf(TEXT(": level within 4 s (%.2f s)"), LevelAfter * Dt)), LevelAfter != INDEX_NONE && LevelAfter * Dt <= 4.0);
	}
	return true;
}

// Catches: a crossing called off before it began (its start refused, or stopped at once) leaving the level assist
// standing by, or levelling again from nothing: everything is as if nothing had been asked.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistGen1CalledOff,
	"Ramms.Level.Assist.Gen1.CrossingCalledOffBeforeItBeginsChangesNothing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistGen1CalledOff::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	FGen1Rig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr))
	{
		return false;
	}
	IRammsDriveMode* CurbMode = R.Curb;
	IRammsDriveMode* DriveMode = R.Drive;
	R.GroundDeg = FVector2D(3.0, -2.0);
	R.Pose();
	R.Level->SetAssistEnabled(true);
	R.Frames(300);
	const TArray<FName> Ids = FGen1Rig::Lifts();
	TMap<FName, float>	Baselines, Offsets;
	for (const FName& Id : Ids)
	{
		Baselines.Add(Id, R.Baseline(Id));
		Offsets.Add(Id, R.Base->GetMotorCommandOffset(Id));
	}
	const int32 Before = R.LiftPushes();
	FString		Why;
	TestTrue(TEXT("a crossing is committed"), FRammsRammpGen1CurbTestAccess::Commit(*R.Curb, Why));
	TestTrue(TEXT("standing by"), R.State() == EState::Yielded);
	// Stopped before its first tick: nothing has moved, and with nothing to run the mode hands back.
	R.Curb->Abort(TEXT("the test's rider thought better of it"));
	TestFalse(TEXT("nothing is pending"), FRammsRammpGen1CurbTestAccess::IsStartPending(*R.Curb) || FRammsRammpGen1CurbTestAccess::IsCrossing(*R.Curb));
	R.CurbTick();
	TestTrue(TEXT("handed back"), DriveMode->IsDriveModeActive() && !CurbMode->IsDriveModeActive());
	TestTrue(TEXT("levelling, as before"), R.State() == EState::Active);
	for (const FName& Id : Ids)
	{
		TestTrue(*FString::Printf(TEXT("%s's own command as it was, to the bit"), *Id.ToString()), SameBits(R.Baseline(Id), Baselines[Id]));
		TestTrue(*FString::Printf(TEXT("%s's offset as it was, to the bit"), *Id.ToString()), SameBits(R.Base->GetMotorCommandOffset(Id), Offsets[Id]));
	}
	R.Frames(30);
	TestEqual(TEXT("no lift was told anything through it all"), R.LiftPushes(), Before);
	return true;
}

// Catches: a crossing on a robot whose level assist is switched off, or that has none, changed at all by the calls a
// crossing now makes to it: the same commands to the same motors in the same order, lift for lift.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistGen1OffIsAsBefore,
	"Ramms.Level.Assist.Gen1.CrossingWithTheAssistOffOrAbsentIsAsBefore",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistGen1OffIsAsBefore::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	TArray<TArray<RammsCommandOffsetTests::FPush>> Logs;
	for (const bool bWithLevel : { false, true })
	{
		FGen1Rig R(bWithLevel);
		if (!TestTrue(TEXT("the made-up robot"), R.IsReady()))
		{
			return false;
		}
		IRammsDriveMode* CurbMode = R.Curb;
		IRammsDriveMode* DriveMode = R.Drive;
		R.GroundDeg = FVector2D(3.0, -2.0);
		R.Pose();
		// (Switched off, its tick runs and finds the robot: off, it asks nothing.)
		R.Frames(5);
		FString Why;
		TestTrue(TEXT("a crossing is committed"), FRammsRammpGen1CurbTestAccess::Commit(*R.Curb, Why));
		R.Frame();
		R.CurbTick();
		R.Frame();
		for (const FName& Id : FGen1Rig::Lifts())
		{
			TestTrue(*FString::Printf(TEXT("%s: no offset on %s"), bWithLevel ? TEXT("switched off") : TEXT("no level assist"), *Id.ToString()), R.Base->GetMotorCommandOffset(Id) == 0.0f);
		}
		TestEqual(TEXT("the crossing found the elevators at the stance"), FRammsRammpGen1CurbTestAccess::StartCommand(*R.Curb, FRammsRammpGen1CurbTestAccess::ElevatorL), StanceElevator, 1.0e-6f);
		if (FRammsRammpGen1CurbTestAccess::IsCrossing(*R.Curb))
		{
			FRammsRammpGen1CurbTestAccess::Finish(*R.Curb, /*bAborted=*/false, TEXT("the test's crossing is over"));
		}
		R.CurbTick();
		R.Frame();
		TestTrue(TEXT("handed back"), DriveMode->IsDriveModeActive() && !CurbMode->IsDriveModeActive());
		if (bWithLevel)
		{
			TestTrue(TEXT("the level assist is off still"), R.State() == EState::Off && !R.Level->IsAssistEnabled());
		}
		Logs.Add(R.Log->Pushes);
	}
	if (TestEqual(TEXT("as many commands with a level assist switched off as with none"), Logs[1].Num(), Logs[0].Num()))
	{
		for (int32 Index = 0; Index < Logs[0].Num(); ++Index)
		{
			if (!RammsCommandOffsetTests::Matches(Logs[0][Index], Logs[1][Index]))
			{
				AddError(FString::Printf(TEXT("command %d differs: '%s' with none, '%s' with one switched off"), Index, *Logs[0][Index].ToString(), *Logs[1][Index].ToString()));
				break;
			}
		}
	}
	return true;
}

// Catches: the curb view's "command" mark and graph line showing a lift's own command while the level assist has an
// offset on it (off the measured height by the whole of the levelling), where they should show what it is told.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistGen1DebugView,
	"Ramms.Level.Assist.Gen1.CurbViewShowsWhatTheLiftsAreTold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistGen1DebugView::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	FGen1Rig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr))
	{
		return false;
	}
	R.GroundDeg = FVector2D(4.0, 0.0);
	R.Pose();
	R.Level->SetAssistEnabled(true);
	R.Frames(300);
	float Shown = 0.0f;
	if (TestTrue(TEXT("the view has a commanded height for the drive wheels"), FRammsRammpGen1CurbTestAccess::DriveCommandedHeight(*R.Curb, Shown)))
	{
		const double Told = 0.5 * (R.Height(DriveLeft, true) + R.Height(DriveRight, true));
		const double Own = 0.5 * (R.Height(DriveLeft, false) + R.Height(DriveRight, false));
		TestTrue(TEXT("(levelling has the drive wheels well off their own command)"), FMath::Abs(Told - Own) > 1.0);
		TestEqual(TEXT("it is the height they are told to, levelling and all"), static_cast<double>(Shown), Told, 0.05);
	}
	return true;
}

// Catches: a lift its rider moves while the robot is levelled held against them, or told its own command bare (the
// plane dropped from under the rider for a frame); the base left off level on the stance as jogged; and the lift
// controller not told that a crossing's fold and hand-back moved its lifts' commands under it (its next move would
// start from where it last had the lift, the levelled pose, and step the lift's own command by the whole plane).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistGen1LiftJog,
	"Ramms.Level.Assist.Gen1.LiftsJoggedByTheirRiderUnderThePlane",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistGen1LiftJog::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	FGen1Rig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist and its lift controller"), R.IsReady() && R.Level != nullptr && R.AddLiftController()))
	{
		return false;
	}
	const float Dt = 1.0f / 30.0f;
	R.GroundDeg = FVector2D(3.0, -2.0);
	R.Pose();
	R.Level->SetAssistEnabled(true);
	R.Frames(300, Dt);
	if (!TestTrue(TEXT("levelled on the slope, the lift controller ticking"), R.State() == EState::Active && R.AttitudeDeg().Size() <= 0.1))
	{
		return false;
	}
	TestTrue(TEXT("the lift controller wrote nothing meanwhile: the elevators' own commands are the stance's, to the bit"), SameBits(R.Baseline(ElevatorL), StanceElevator) && SameBits(R.Baseline(ElevatorR), StanceElevator));
	TestTrue(TEXT("(with the plane on them as offsets)"), FMath::Abs(R.Base->GetMotorCommandOffset(ElevatorL)) > 0.02f && FMath::Abs(R.Base->GetMotorCommandOffset(ElevatorR)) > 0.02f);

	// The rider lets both drive wheels down a further 3 deg of elevator. Frame by frame: the level assist, the base,
	// the chassis, then the lift controller.
	const float StanceDeg = FMath::RadiansToDegrees(StanceElevator);
	R.LiftController->SetAngularMotorTarget(ElevatorL, StanceDeg + 3.0f);
	R.LiftController->SetAngularMotorTarget(ElevatorR, StanceDeg + 3.0f);
	int32  Bare = 0;
	int32  NotTheSum = 0;
	float  LargestStep = 0.0f;
	double MostOff = 0.0;
	for (int32 Index = 0; Index < 30; ++Index)
	{
		R.Level->TickComponent(Dt, LEVELTICK_All, nullptr);
		R.Tick(Dt);
		for (const FName& Id : { ElevatorL, ElevatorR })
		{
			// What the base has just told the lift: its own command with the plane's offset, never the one without.
			const float Told = R.Log->LastFor(Id);
			Bare += FMath::IsNearlyEqual(Told, R.Baseline(Id), 5.0e-3f) ? 1 : 0;
			NotTheSum += FMath::IsNearlyEqual(Told, R.Effective(Id), 1.0e-5f) ? 0 : 1;
		}
		R.Pose();
		MostOff = FMath::Max(MostOff, R.AttitudeDeg().Size());
		const float Before = R.Baseline(ElevatorL);
		R.LiftTick(Dt);
		LargestStep = FMath::Max(LargestStep, FMath::Abs(R.Baseline(ElevatorL) - Before));
	}
	TestEqual(TEXT("jogged: no lift was ever told its own command bare"), Bare, 0);
	TestEqual(TEXT("... but always its command with its offset"), NotTheSum, 0);
	TestEqual(TEXT("the left elevator's own command went where its rider sent it"), R.Baseline(ElevatorL), StanceElevator + FMath::DegreesToRadians(3.0f), 1.0e-5f);
	TestEqual(TEXT("... the right one's too"), R.Baseline(ElevatorR), StanceElevator + FMath::DegreesToRadians(3.0f), 1.0e-5f);
	TestEqual(TEXT("... at the lift's own speed, 18 deg/s"), LargestStep, FMath::DegreesToRadians(18.0f) * Dt, 1.0e-4f);
	AddInfo(FString::Printf(TEXT("the base was at most %.2f deg off level while its drive wheels were let down"), MostOff));
	R.Frames(150, Dt);
	TestTrue(*FString::Printf(TEXT("level again on the stance as jogged (%.3f deg)"), R.AttitudeDeg().Size()), R.State() == EState::Active && R.AttitudeDeg().Size() <= 0.1);
	TestEqual(TEXT("neither translator nor any wheel was written"), R.OtherPushes(), 0);

	// A crossing takes the lifts and puts them back: the plane goes into the lifts' commands and comes back out.
	const float Jogged = R.Baseline(ElevatorL);
	FString		Why;
	TestTrue(TEXT("a crossing is committed"), FRammsRammpGen1CurbTestAccess::Commit(*R.Curb, Why));
	R.Frame(Dt);
	R.CurbTick(Dt);
	R.Frame(Dt);
	TestTrue(TEXT("(the crossing has the lifts at the levelled pose: the elevator's own command is off the stance)"), FMath::Abs(R.Baseline(ElevatorL) - Jogged) > 0.02f);
	if (FRammsRammpGen1CurbTestAccess::IsCrossing(*R.Curb))
	{
		FRammsRammpGen1CurbTestAccess::Finish(*R.Curb, /*bAborted=*/false, TEXT("the test's crossing is over"));
	}
	R.CurbTick(Dt);
	TestTrue(TEXT("handed back, levelling"), R.State() == EState::Active);
	TestEqual(TEXT("the elevator's own command is the jogged stance's again"), R.Baseline(ElevatorL), Jogged, 2.0e-4f);
	R.Frames(5, Dt);
	// The rider moves the lift again: from where its command is now, at the lift's speed.
	const float Handed = R.Baseline(ElevatorL);
	R.LiftController->SetAngularMotorTarget(ElevatorL, StanceDeg + 5.0f);
	TestEqual(TEXT("asked to move again: the lift's own command does not jump to where the crossing had it"), R.Baseline(ElevatorL), Handed, 1.0e-5f);
	R.LiftTick(Dt);
	TestEqual(TEXT("... and its first step is one tick of the lift's speed from where it is"), R.Baseline(ElevatorL) - Handed, FMath::DegreesToRadians(18.0f) * Dt, 1.0e-4f);
	R.Frames(60, Dt);
	TestEqual(TEXT("... on to where its rider sent it"), R.Baseline(ElevatorL), StanceElevator + FMath::DegreesToRadians(5.0f), 1.0e-5f);
	TestTrue(*FString::Printf(TEXT("level on that stance too (%.3f deg)"), R.AttitudeDeg().Size()), R.AttitudeDeg().Size() <= 0.15);
	return true;
}

// Catches: the tilt its rider prefers asked of a seat this robot does not have (the pitch would then be nobody's:
// the chassis level, the rider not where they asked), or only its roll put in the chassis; a seat's read-outs shown
// for it; and the differential drive, which drives this robot, never asked to go slower while the lifts catch the
// ground up.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsLevelAssistGen1Preference,
	"Ramms.Level.Assist.Gen1.PreferredTiltIsTheChassisAndTheDriveIsSlowed",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsLevelAssistGen1Preference::RunTest(const FString& Parameters)
{
	using namespace RammsLevelGen1Tests;
	FGen1Rig R;
	if (!TestTrue(TEXT("the made-up robot with a level assist"), R.IsReady() && R.Level != nullptr))
	{
		return false;
	}
	IRammsControlContributor* Controls = R.Level;
	R.GroundDeg = FVector2D(3.0, -2.0);
	R.Pose();
	R.Frames(3);
	{
		FRammsControlSurface Surface;
		Controls->DescribeControls(Surface);
		TestFalse(TEXT("a robot whose seat is fixed shows no seat trim and no rider's tilt of its own"),
			Surface.Axes.ContainsByPredicate([](const FRammsControlAxis& Axis) { return Axis.Id == RammsLevel::RiderTiltControlId() || Axis.Id == RammsLevel::SeatTrimControlId(); }));
		TestTrue(TEXT("... and both preferences"),
			Surface.Axes.ContainsByPredicate([](const FRammsControlAxis& Axis) { return Axis.Id == RammsLevel::PreferencePitchControlId(); })
				&& Surface.Axes.ContainsByPredicate([](const FRammsControlAxis& Axis) { return Axis.Id == RammsLevel::PreferenceRollControlId(); }));
	}
	TestTrue(TEXT("a preferred tilt: back 2 deg, and 1.5 deg to one side"), Controls->ApplyControl(RammsLevel::PreferencePitchControlId(), 2.0f) && Controls->ApplyControl(RammsLevel::PreferenceRollControlId(), -1.5f));
	R.Level->SetAssistEnabled(true);
	R.Frames(400);
	TestTrue(TEXT("levelling"), R.State() == EState::Active);
	TestEqual(TEXT("the chassis is pitched back by the preferred 2 deg: there is no seat to hold it"), R.AttitudeDeg().X, 2.0, 0.15);
	TestEqual(TEXT("... and rolled by the preferred 1.5"), R.AttitudeDeg().Y, -1.5, 0.15);
	TestTrue(TEXT("the wanted attitude a curb controller is given is that"), R.Level->GetTargetAttitudeDeg().Equals(FVector2D(2.0, -1.5), 0.01));
	TestEqual(TEXT("level.tilt reads the chassis' own tilt, which is the rider's: the preference"), static_cast<double>(R.Control(RammsLevel::TiltControlId())), RammsLevel::Inclination(FVector2D(2.0, -1.5)), 0.2);
	// Back to none: the chassis level again.
	Controls->ApplyControl(RammsLevel::PreferencePitchControlId(), 0.0f);
	Controls->ApplyControl(RammsLevel::PreferenceRollControlId(), 0.0f);
	R.Frames(300);
	TestTrue(*FString::Printf(TEXT("no preference: level (%.3f deg)"), R.AttitudeDeg().Size()), R.AttitudeDeg().Size() <= 0.1);

	// The drive: the differential mode, asked nothing while the ground is steady, and to go slower when the ground
	// steps by more than the lifts follow in a frame.
	TestEqual(TEXT("level on steady ground: the differential drive at full speed"), R.Drive->GetDriveSpeedScale(), 1.0f);
	R.GroundDeg += FVector2D(2.5, 0.0);
	R.Pose();
	float LeastAsked = 1.0f;
	int32 Differing = 0;
	for (int32 Index = 0; Index < 12; ++Index)
	{
		R.Frame();
		LeastAsked = FMath::Min(LeastAsked, R.Drive->GetDriveSpeedScale());
		Differing += FMath::IsNearlyEqual(R.Drive->GetDriveSpeedScale(), R.Control(RammsLevel::SpeedScaleControlId()), 1.0e-6f) ? 0 : 1;
	}
	TestTrue(*FString::Printf(TEXT("the differential drive is asked to go slower (%.2f)"), LeastAsked), LeastAsked < 0.9f && LeastAsked >= R.Level->Tuning.MinSpeedScale - 1.0e-4f);
	TestEqual(TEXT("... by what level.speed_scale shows, on every frame"), Differing, 0);
	R.Frames(300);
	TestEqual(TEXT("level again: the drive at full speed"), R.Drive->GetDriveSpeedScale(), 1.0f);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
