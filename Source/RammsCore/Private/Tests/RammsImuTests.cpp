// Tests for physics-sourced IMU data: the rigid-body arithmetic every provider shares (pure), the game-rate
// differencer, the sample ring and the stream record, the Chaos sample from two steps, which engine Auto picks for a
// point (and picks again when the body changes), and Chaos itself in a ticking world: a box at rest reads +|g| up, a
// box in free fall reads only the centripetal acceleration of its spin, a box spinning about an off-centre centre of
// mass is read about that centre, a box with gravity off reads nothing, IMU components read the interval's motion in
// their own frame, and every step's samples reach Remote Control through a cursor.

#include "CoreMinimal.h"
#include "Components/BoxComponent.h"
#include "Components/SceneComponent.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include "RammsChaosImuProvider.h"
#include "RammsIMUSensorComponent.h"
#include "RammsImuMath.h"
#include "RammsImuStreamSubsystem.h"
#include "RammsPhysicsImuSubsystem.h"
#include "RammsSensingBridge.h"
#include "RammsSensingBridgeImu.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace RammsImuTests
{
	/** cm/s^2: what Chaos' contact jitter and the differencing leave on a body at rest or falling. */
	constexpr double ImuChaosAccelSlackCm = 50.0;

	/** Unreal's default gravity, cm/s^2. */
	constexpr double ImuGravityCm = 980.0;

	bool NearlyEqualVectors(FAutomationTestBase& Test, const FString& What, const FVector& Actual, const FVector& Expected, double Slack)
	{
		const bool bNear = (Actual - Expected).GetAbsMax() <= Slack;
		Test.TestTrue(FString::Printf(TEXT("%s: %s, expected %s (+-%g)"), *What, *Actual.ToString(), *Expected.ToString(), Slack), bNear);
		return bNear;
	}

	TSharedPtr<FJsonObject> ParseImuJson(const FString& Json)
	{
		TSharedPtr<FJsonObject>			Obj;
		const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
		return FJsonSerializer::Deserialize(Reader, Obj) ? Obj : nullptr;
	}

	FRammsImuSample SampleAt(double Time, double Value)
	{
		FRammsImuSample Sample;
		Sample.bValid = true;
		Sample.SimTimeSeconds = Time;
		Sample.LinearVelocity = FVector(Value, 0.0, 0.0);
		return Sample;
	}

	/** A game world with a physics scene that simulates when ticked, torn down with everything in it. */
	struct FImuTestWorld
	{
		UWorld* World = nullptr;

		FImuTestWorld()
		{
			UWorld::InitializationValues Values;
			Values.CreatePhysicsScene(true).ShouldSimulatePhysics(true).EnableTraceCollision(true).CreateNavigation(false).CreateAISystem(false);
			World = UWorld::CreateWorld(EWorldType::Game, /*bInformEngineOfWorld=*/false, NAME_None, nullptr, /*bAddToRoot=*/true,
				ERHIFeatureLevel::Num, &Values);
			if (World != nullptr)
			{
				GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
			}
		}

		~FImuTestWorld()
		{
			if (World != nullptr)
			{
				GEngine->DestroyWorldContext(World);
				World->DestroyWorld(false);
			}
		}

		/** An actor whose root is a plain scene component. */
		AActor* SpawnPlain(const FVector& Location)
		{
			AActor*			 Actor = World->SpawnActor<AActor>();
			USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("Root"));
			Actor->SetRootComponent(Root);
			Root->SetWorldLocation(Location);
			Root->RegisterComponent();
			return Actor;
		}

		/** An actor whose root is a box (half extent Extent) with collision; simulating or static. */
		AActor* SpawnBox(const FVector& Location, const FVector& Extent, bool bSimulate, UBoxComponent** OutBox = nullptr)
		{
			AActor*		   Actor = World->SpawnActor<AActor>();
			UBoxComponent* Box = NewObject<UBoxComponent>(Actor, TEXT("Box"));
			Box->SetBoxExtent(Extent);
			Box->SetCollisionProfileName(bSimulate ? TEXT("PhysicsActor") : TEXT("BlockAll"));
			Box->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
			Box->SetSimulatePhysics(bSimulate);
			Box->SetLinearDamping(0.0f);
			Box->SetAngularDamping(0.0f);
			Actor->SetRootComponent(Box);
			Box->SetWorldLocation(Location);
			Box->RegisterComponent();
			if (OutBox)
			{
				*OutBox = Box;
			}
			return Actor;
		}

		/** A plain scene component attached to Parent at Offset. */
		static USceneComponent* AddChild(AActor* Actor, USceneComponent* Parent, const TCHAR* Name, const FVector& Offset)
		{
			USceneComponent* Child = NewObject<USceneComponent>(Actor, Name);
			Child->SetupAttachment(Parent);
			Child->SetRelativeLocation(Offset);
			Child->RegisterComponent();
			return Child;
		}

		/** An IMU component measuring every tick, attached to Parent at Offset and turned by Rotation. */
		static URammsIMUSensorComponent* AddImu(AActor* Actor, USceneComponent* Parent, const TCHAR* Name, const FVector& Offset,
			const FRotator& Rotation, bool bUsePhysicsVelocity = true)
		{
			URammsIMUSensorComponent* Sensor = NewObject<URammsIMUSensorComponent>(Actor, Name);
			Sensor->SetupAttachment(Parent);
			Sensor->SetRelativeLocationAndRotation(Offset, Rotation);
			Sensor->UpdateRateHz = 0.0f;
			Sensor->bUsePhysicsVelocity = bUsePhysicsVelocity;
			Sensor->RegisterComponent();
			return Sensor;
		}
	};
} // namespace RammsImuTests

// ---------------------------------------------------------------------------
// The arithmetic
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsImuLeverArmTest, "Ramms.Sensing.Imu.Math.LeverArm",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsImuLeverArmTest::RunTest(const FString& Parameters)
{
	using namespace RammsImuTests;

	// The cross product turns the way Unreal's quaternions do: a point turned by w dt moves by (w x r) dt.
	const FVector Spin(0.3, -0.2, 1.5); // rad/s
	const FVector Lever(40.0, 10.0, -25.0);
	const double  Dt = 1.0e-5;
	const FQuat	  Turn(Spin.GetSafeNormal(), Spin.Size() * Dt);
	NearlyEqualVectors(*this, TEXT("w x r is the velocity a turn gives"), RammsImuMath::PointVelocity(FVector::ZeroVector, Spin, Lever),
		(Turn.RotateVector(Lever) - Lever) / Dt, 1.0e-2);

	// Spinning about Z at 2 rad/s with the origin at rest: the point 50 cm out moves at 100 cm/s and accelerates at
	// w^2 r = 200 cm/s^2 toward the axis.
	const FVector Axial(0.0, 0.0, 2.0);
	const FVector Out(50.0, 0.0, 0.0);
	NearlyEqualVectors(*this, TEXT("tangential velocity"), RammsImuMath::PointVelocity(FVector::ZeroVector, Axial, Out), FVector(0.0, 100.0, 0.0), 1.0e-9);
	NearlyEqualVectors(*this, TEXT("centripetal acceleration"), RammsImuMath::PointAcceleration(FVector::ZeroVector, FVector::ZeroVector, Axial, Out),
		FVector(-200.0, 0.0, 0.0), 1.0e-9);
	// Angular acceleration alone: alpha x r.
	NearlyEqualVectors(*this, TEXT("tangential acceleration"), RammsImuMath::PointAcceleration(FVector(1.0, 2.0, 3.0), FVector(0.0, 0.0, 3.0), FVector::ZeroVector, Out),
		FVector(1.0, 152.0, 3.0), 1.0e-9);

	// An accelerometer reads +|g| up at rest and nothing in free fall.
	const FVector Gravity(0.0, 0.0, -ImuGravityCm);
	NearlyEqualVectors(*this, TEXT("at rest"), RammsImuMath::ProperAcceleration(FVector::ZeroVector, Gravity), FVector(0.0, 0.0, ImuGravityCm), 1.0e-9);
	NearlyEqualVectors(*this, TEXT("in free fall"), RammsImuMath::ProperAcceleration(Gravity, Gravity), FVector::ZeroVector, 1.0e-9);

	// A sensor yawed 90 degrees (its X along world Y) reads world Y on its X.
	const FQuat Yawed(FVector::UpVector, UE_DOUBLE_HALF_PI);
	NearlyEqualVectors(*this, TEXT("world Y in a yawed sensor"), RammsImuMath::ToSensorFrame(Yawed, FVector(0.0, 1.0, 0.0)), FVector(1.0, 0.0, 0.0), 1.0e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsImuRotationRateTest, "Ramms.Sensing.Imu.Math.RotationRate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsImuRotationRateTest::RunTest(const FString& Parameters)
{
	using namespace RammsImuTests;

	const FQuat	  Start = FQuat(FRotator(10.0, 20.0, 30.0));
	const FVector Rate(0.5, -1.0, 2.0); // rad/s, world
	const double  Dt = 0.01;
	const FQuat	  End = FQuat(Rate.GetSafeNormal(), Rate.Size() * Dt) * Start;
	NearlyEqualVectors(*this, TEXT("the rate that turned Start into End"), RammsImuMath::AngularVelocityBetween(Start, End, Dt), Rate, 1.0e-6);

	// Most of a turn one way is a short turn the other.
	const FQuat Far(FVector::UpVector, 1.9 * UE_DOUBLE_PI);
	NearlyEqualVectors(*this, TEXT("the short way round"), RammsImuMath::AngularVelocityBetween(FQuat::Identity, Far, 1.0),
		FVector(0.0, 0.0, -0.1 * UE_DOUBLE_PI), 1.0e-6);
	NearlyEqualVectors(*this, TEXT("no time"), RammsImuMath::AngularVelocityBetween(Start, End, 0.0), FVector::ZeroVector, 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsImuDifferencerTest, "Ramms.Sensing.Imu.Math.Differencer",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsImuDifferencerTest::RunTest(const FString& Parameters)
{
	using namespace RammsImuTests;

	// A point thrown up at 300 cm/s and falling (x = v0 t + a t^2 / 2), turning at a constant rate: the second
	// difference is the acceleration exactly, so an accelerometer on it reads nothing.
	const FVector		 Gravity(0.0, 0.0, -ImuGravityCm);
	const FVector		 Spin(0.0, 0.0, 1.0); // rad/s
	const double		 Dt = 0.02;
	FRammsImuDifferencer Differencer;
	FRammsImuSample		 Sample;
	for (int32 Step = 0; Step < 4; ++Step)
	{
		const double  T = Step * Dt;
		const FVector Location = FVector(0.0, 0.0, 300.0) * T + 0.5 * Gravity * T * T;
		const FQuat	  Rotation(Spin.GetSafeNormal(), Spin.Size() * T);
		Sample = Differencer.Step(T, FTransform(Rotation, Location), Gravity);
		TestEqual(*FString::Printf(TEXT("step %d: acceleration from the third pose on"), Step), Sample.bHasAcceleration, Step >= 2);
	}
	TestTrue(TEXT("a sample"), Sample.bValid);
	TestEqual(TEXT("Kinematic"), Sample.Source, ERammsImuSource::Kinematic);
	NearlyEqualVectors(*this, TEXT("free fall reads nothing"), Sample.ProperAcceleration, FVector::ZeroVector, 1.0e-6);
	NearlyEqualVectors(*this, TEXT("the turn rate (deg/s)"), Sample.AngularVelocity, RammsImuMath::RadiansToDegrees(Spin), 1.0e-6);
	NearlyEqualVectors(*this, TEXT("no angular acceleration"), Sample.AngularAcceleration, FVector::ZeroVector, 1.0e-6);
	const FRammsImuSample Again = Differencer.Step(3 * Dt, FTransform::Identity, Gravity);
	NearlyEqualVectors(*this, TEXT("a second step at the same time keeps the sample"), Again.Location, Sample.Location, 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsImuDifferencerUnevenTest, "Ramms.Sensing.Imu.Math.DifferencerUnevenFrames",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsImuDifferencerUnevenTest::RunTest(const FString& Parameters)
{
	using namespace RammsImuTests;

	// The same throw at an uneven frame rate (a hitch: 16, 33, 16 ms): the velocities are half an interval off each
	// frame, so the second difference must divide by their spacing, not by the last frame's time.
	const FVector		 Gravity(0.0, 0.0, -ImuGravityCm);
	const double		 Times[] = { 0.0, 0.016, 0.049, 0.065, 0.098 };
	FRammsImuDifferencer Differencer;
	for (const double T : Times)
	{
		const FRammsImuSample& Sample = Differencer.Step(T, FTransform(FVector(0.0, 0.0, 300.0) * T + 0.5 * Gravity * T * T), Gravity);
		if (Sample.bHasAcceleration)
		{
			NearlyEqualVectors(*this, FString::Printf(TEXT("free fall reads nothing at %.3f s"), T), Sample.ProperAcceleration, FVector::ZeroVector, 1.0e-6);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsImuSampleRingTest, "Ramms.Sensing.Imu.Math.SampleRing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsImuSampleRingTest::RunTest(const FString& Parameters)
{
	using namespace RammsImuTests;

	FRammsImuSampleRing Ring;
	Ring.Push(SampleAt(0.5, -1.0));
	TestEqual(TEXT("no capacity keeps nothing"), Ring.GetNextSeq(), int64(0));

	Ring.SetCapacity(4);
	for (int32 i = 1; i <= 6; ++i)
	{
		Ring.Push(SampleAt(i * 0.01, i));
	}
	TestEqual(TEXT("six pushed"), Ring.GetNextSeq(), int64(6));
	TestEqual(TEXT("the last four kept"), Ring.GetOldestSeq(), int64(2));

	TArray<FRammsImuSample> Read;
	TestEqual(TEXT("a read from before the oldest starts at the oldest"), Ring.Read(0, 10, Read), int64(2));
	TestEqual(TEXT("four read"), Read.Num(), 4);
	TestTrue(TEXT("oldest first"), Read.Num() == 4 && Read[0].LinearVelocity.X == 3.0 && Read[3].LinearVelocity.X == 6.0);

	Read.Reset();
	TestEqual(TEXT("a read from the middle"), Ring.Read(4, 1, Read), int64(4));
	TestTrue(TEXT("one sample, seq 4"), Read.Num() == 1 && Read[0].LinearVelocity.X == 5.0);

	// The same step again (a sub-step): its final state replaces it, the sequence does not move.
	Ring.Push(SampleAt(0.06, 60.0));
	TestEqual(TEXT("a re-push of the newest time adds nothing"), Ring.GetNextSeq(), int64(6));
	Read.Reset();
	Ring.Read(5, 1, Read);
	TestTrue(TEXT("and replaces it"), Read.Num() == 1 && Read[0].LinearVelocity.X == 60.0);

	// Resizing drops what it held; the sequence goes on.
	Ring.SetCapacity(8);
	TestEqual(TEXT("empty after a resize"), Ring.GetOldestSeq(), int64(6));
	Ring.Push(SampleAt(0.07, 7.0));
	Read.Reset();
	TestEqual(TEXT("the next sample is seq 6"), Ring.Read(0, 8, Read), int64(6));
	TestEqual(TEXT("one kept"), Read.Num(), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsImuRecordTest, "Ramms.Sensing.Imu.Math.Record",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsImuRecordTest::RunTest(const FString& Parameters)
{
	using namespace RammsImuTests;

	// A sensor yawed 90 degrees (its X along world Y): the record carries sensor-frame accelerometer, gyro and
	// angular acceleration, world velocity and orientation.
	FRammsImuSample Sample;
	Sample.bValid = true;
	Sample.bHasAcceleration = true;
	Sample.SimTimeSeconds = 12.5;
	Sample.Orientation = FQuat(FVector::UpVector, UE_DOUBLE_HALF_PI);
	Sample.ProperAcceleration = FVector(0.0, 100.0, 980.0);
	Sample.AngularVelocity = FVector(0.0, 30.0, 0.0);
	Sample.AngularAcceleration = FVector(0.0, 0.0, 7.0);
	Sample.LinearVelocity = FVector(1.0, 2.0, 3.0);
	double Record[RammsImuRecord::NumFields];
	RammsImuRecord::ToRecord(42, Sample, Record);
	TestEqual(TEXT("seq"), Record[0], 42.0);
	TestEqual(TEXT("sim_time"), Record[1], 12.5);
	NearlyEqualVectors(*this, TEXT("accelerometer, sensor frame"), FVector(Record[2], Record[3], Record[4]), FVector(100.0, 0.0, 980.0), 1.0e-9);
	NearlyEqualVectors(*this, TEXT("gyro, sensor frame"), FVector(Record[5], Record[6], Record[7]), FVector(30.0, 0.0, 0.0), 1.0e-9);
	NearlyEqualVectors(*this, TEXT("angular acceleration, sensor frame"), FVector(Record[8], Record[9], Record[10]), FVector(0.0, 0.0, 7.0), 1.0e-9);
	NearlyEqualVectors(*this, TEXT("velocity, world"), FVector(Record[11], Record[12], Record[13]), FVector(1.0, 2.0, 3.0), 0.0);
	TestTrue(TEXT("orientation x, y, z, w"), Record[14] == Sample.Orientation.X && Record[15] == Sample.Orientation.Y && Record[16] == Sample.Orientation.Z && Record[17] == Sample.Orientation.W);
	TestEqual(TEXT("18 fields named"), FString(RammsImuRecord::FieldNames[17]), FString(TEXT("qw")));

	// A first sample has no acceleration yet: zeros, not stale values.
	Sample.bHasAcceleration = false;
	RammsImuRecord::ToRecord(43, Sample, Record);
	NearlyEqualVectors(*this, TEXT("no acceleration yet"), FVector(Record[2], Record[3], Record[4]), FVector::ZeroVector, 0.0);
	NearlyEqualVectors(*this, TEXT("no angular acceleration yet"), FVector(Record[8], Record[9], Record[10]), FVector::ZeroVector, 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsImuChaosSampleTest, "Ramms.Sensing.Imu.Math.ChaosSample",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsImuChaosSampleTest::RunTest(const FString& Parameters)
{
	using namespace RammsImuTests;
	using FStepState = FRammsChaosImuProvider::FStepState;

	// Two Chaos steps 10 ms apart of a body falling and spinning at 2 rad/s about Z: the point 50 cm from its
	// centre of mass reads only the centripetal acceleration, toward the axis.
	const FVector Gravity(0.0, 0.0, -ImuGravityCm);
	FStepState	  Previous;
	Previous.bValid = true;
	Previous.Time = 1.0;
	Previous.ComVelocity = FVector(0.0, 0.0, -100.0);
	Previous.AngularVelocityRad = FVector(0.0, 0.0, 2.0);
	FStepState Current = Previous;
	Current.Time = 1.01;
	Current.ComVelocity = Previous.ComVelocity + Gravity * 0.01;
	const FVector		  Lever(50.0, 0.0, 0.0);
	const FRammsImuSample Sample = FRammsChaosImuProvider::MakeSample(Previous, Current, FVector(1.0, 2.0, 3.0), FQuat::Identity, Lever, Gravity);
	TestEqual(TEXT("Chaos"), Sample.Source, ERammsImuSource::Chaos);
	TestTrue(TEXT("an acceleration from two steps"), Sample.bHasAcceleration);
	TestEqual(TEXT("stamped with the later step"), Sample.SimTimeSeconds, 1.01);
	NearlyEqualVectors(*this, TEXT("the point's velocity"), Sample.LinearVelocity, Current.ComVelocity + FVector(0.0, 100.0, 0.0), 1.0e-9);
	NearlyEqualVectors(*this, TEXT("centripetal only"), Sample.ProperAcceleration, FVector(-200.0, 0.0, 0.0), 1.0e-6);
	NearlyEqualVectors(*this, TEXT("deg/s"), Sample.AngularVelocity, FVector(0.0, 0.0, 2.0 * 180.0 / UE_DOUBLE_PI), 1.0e-9);

	FStepState Invalid;
	TestFalse(TEXT("no acceleration from one step"), FRammsChaosImuProvider::MakeSample(Invalid, Current, FVector::ZeroVector, FQuat::Identity, Lever, Gravity).bHasAcceleration);
	TestFalse(TEXT("no acceleration from the same step twice"), FRammsChaosImuProvider::MakeSample(Current, Current, FVector::ZeroVector, FQuat::Identity, Lever, Gravity).bHasAcceleration);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsImuSourceNamesTest, "Ramms.Sensing.Imu.SourceNames",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsImuSourceNamesTest::RunTest(const FString& Parameters)
{
	const ERammsImuSource All[] = { ERammsImuSource::Auto, ERammsImuSource::Chaos, ERammsImuSource::MuJoCo, ERammsImuSource::Kinematic };
	for (const ERammsImuSource Source : All)
	{
		ERammsImuSource Parsed = ERammsImuSource::Auto;
		const FString	Name = URammsPhysicsImuSubsystem::SourceName(Source);
		TestTrue(*FString::Printf(TEXT("%s round-trips"), *Name), URammsPhysicsImuSubsystem::ParseSource(Name.ToUpper(), Parsed) && Parsed == Source);
	}
	ERammsImuSource Unused = ERammsImuSource::Auto;
	TestFalse(TEXT("an unknown source"), URammsPhysicsImuSubsystem::ParseSource(TEXT("physx"), Unused));
	return true;
}

// ---------------------------------------------------------------------------
// Which engine serves a point
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsImuSelectionTest, "Ramms.Sensing.Imu.Selection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsImuSelectionTest::RunTest(const FString& Parameters)
{
	using namespace RammsImuTests;
	FImuTestWorld Scratch;
	if (!TestNotNull(TEXT("world"), Scratch.World))
	{
		return false;
	}
	URammsPhysicsImuSubsystem* Imu = URammsPhysicsImuSubsystem::Get(Scratch.World);
	if (!TestNotNull(TEXT("the IMU subsystem"), Imu))
	{
		return false;
	}

	// Nothing simulates a plain component: Kinematic, on the component itself.
	AActor*			 Plain = Scratch.SpawnPlain(FVector(0.0, 0.0, 500.0));
	USceneComponent* Mast = FImuTestWorld::AddChild(Plain, Plain->GetRootComponent(), TEXT("Mast"), FVector(0.0, 0.0, 30.0));
	const int32		 PlainHandle = Imu->Track(Mast, FVector::ZeroVector);
	TestTrue(TEXT("tracked"), PlainHandle > 0 && Imu->IsTracked(PlainHandle));
	TestEqual(TEXT("a plain component is Kinematic"), Imu->GetTrackedSource(PlainHandle), ERammsImuSource::Kinematic);
	FRammsImuSample Sample;
	TestTrue(TEXT("Kinematic has a pose at once"), Imu->GetSample(PlainHandle, Sample) && Sample.bValid);
	NearlyEqualVectors(*this, TEXT("Kinematic pose"), Sample.Location, Mast->GetComponentLocation(), 1.0e-6);

	// A component on a simulating box: Chaos, the box being the body, one hop up; the point is the component's
	// offset plus the local offset, in the box's frame.
	UBoxComponent*	 Box = nullptr;
	AActor*			 Crate = Scratch.SpawnBox(FVector(1000.0, 0.0, 200.0), FVector(20.0), /*bSimulate=*/true, &Box);
	USceneComponent* Mount = FImuTestWorld::AddChild(Crate, Box, TEXT("Mount"), FVector(10.0, 0.0, 20.0));
	TestTrue(TEXT("the box simulates"), Box->IsSimulatingPhysics());
	FRammsImuBodyRef Ref;
	TestTrue(TEXT("resolves"), Imu->ResolveBody(*Mount, NAME_None, FTransform(FVector(5.0, 0.0, 0.0)), ERammsImuSource::Auto, Ref));
	TestEqual(TEXT("Auto picks Chaos on a simulating primitive"), Ref.Source, ERammsImuSource::Chaos);
	TestTrue(TEXT("the box is the body"), Ref.Body.Get() == Box);
	TestEqual(TEXT("one hop up"), Ref.Depth, 1);
	NearlyEqualVectors(*this, TEXT("the point in the box's frame"), Ref.PointInBody.GetLocation(), FVector(15.0, 0.0, 20.0), 1.0e-3);
	const int32 ChaosHandle = Imu->Track(Mount, FVector(5.0, 0.0, 0.0));
	TestEqual(TEXT("tracked on Chaos"), Imu->GetTrackedSource(ChaosHandle), ERammsImuSource::Chaos);
	TestEqual(TEXT("named after the box"), Imu->GetTrackedBodyName(ChaosHandle), Box->GetName());

	// Asked for by name: Kinematic is Kinematic; an engine that does not simulate the body falls back to Auto.
	TestTrue(TEXT("resolves Kinematic"), Imu->ResolveBody(*Mount, NAME_None, FTransform::Identity, ERammsImuSource::Kinematic, Ref));
	TestEqual(TEXT("Kinematic by name"), Ref.Source, ERammsImuSource::Kinematic);
	TestTrue(TEXT("resolves MuJoCo"), Imu->ResolveBody(*Mount, NAME_None, FTransform::Identity, ERammsImuSource::MuJoCo, Ref));
	TestEqual(TEXT("MuJoCo by name on a Chaos body falls back to Chaos"), Ref.Source, ERammsImuSource::Chaos);

	// A static body is not simulated: Kinematic under Auto, Chaos only by name.
	UBoxComponent* Wall = nullptr;
	Scratch.SpawnBox(FVector(-1000.0, 0.0, 200.0), FVector(50.0), /*bSimulate=*/false, &Wall);
	TestTrue(TEXT("resolves the wall"), Imu->ResolveBody(*Wall, NAME_None, FTransform::Identity, ERammsImuSource::Auto, Ref));
	TestEqual(TEXT("a static body is Kinematic under Auto"), Ref.Source, ERammsImuSource::Kinematic);

	// The bodies an engine simulates, main body first.
	TArray<FRammsImuBodyRef> Bodies;
	Imu->ListBodies(*Crate, Bodies);
	TestTrue(TEXT("the crate lists its box first"), Bodies.Num() >= 1 && Bodies[0].Body.Get() == Box && Bodies[0].Source == ERammsImuSource::Chaos);

	// A point's history: every sample from a cursor on, its sequence going on when the point moves to another engine.
	Imu->EnableHistory(PlainHandle);
	Imu->Tick(0.0f); // the Kinematic point takes its frame (the same game time: one sample)
	FRammsImuHistoryRead Read;
	TestTrue(TEXT("reads the history"), Imu->ReadHistory(PlainHandle, 0, 16, Read));
	TestTrue(TEXT("one sample, seq 0"), Read.Samples.Num() == 1 && Read.FirstSeq == 0 && Read.Next == 1 && Read.Dropped == 0);
	TestTrue(TEXT("start now: no samples, the cursor"), Imu->ReadHistory(PlainHandle, -1, 16, Read) && Read.Samples.Num() == 0 && Read.Next == 1);

	// The body a point rides on changes: re-attached under the simulating box, the plain point moves to Chaos; its
	// sequence goes on from 1 (the sample before is gone with the Kinematic point, and counted as dropped).
	Mast->AttachToComponent(Box, FAttachmentTransformRules::KeepWorldTransform);
	Imu->RefreshPoints(/*bForceResolve=*/true);
	TestEqual(TEXT("re-attached under a simulating box: Chaos"), Imu->GetTrackedSource(PlainHandle), ERammsImuSource::Chaos);
	TestTrue(TEXT("riding on the box"), Imu->GetTrackedBody(PlainHandle) == Box);
	TestTrue(TEXT("the sequence goes on"), Imu->ReadHistory(PlainHandle, 1, 16, Read) && Read.Next == 1 && Read.Dropped == 0);
	TestTrue(TEXT("the moved point's old samples count as dropped"), Imu->ReadHistory(PlainHandle, 0, 16, Read) && Read.Next == 1 && Read.Dropped == 1);

	// ...and when its component moves onto another body.
	UBoxComponent* OtherBox = nullptr;
	Scratch.SpawnBox(FVector(3000.0, 0.0, 200.0), FVector(20.0), /*bSimulate=*/true, &OtherBox);
	Mount->AttachToComponent(OtherBox, FAttachmentTransformRules::KeepRelativeTransform);
	Imu->RefreshPoints(/*bForceResolve=*/true);
	TestTrue(TEXT("the point follows its component to the other box"), Imu->GetTrackedBody(ChaosHandle) == OtherBox);

	// At most MaxTrackedPoints per world.
	TArray<int32> Filled;
	while (Imu->GetTrackedCount() < URammsPhysicsImuSubsystem::MaxTrackedPoints)
	{
		const int32 Handle = Imu->Track(Mast, FVector::ZeroVector, ERammsImuSource::Kinematic);
		if (!TestTrue(TEXT("tracks up to the limit"), Handle > 0))
		{
			break;
		}
		Filled.Add(Handle);
	}
	AddExpectedMessagePlain(TEXT("not tracking an IMU point"), ELogVerbosity::Warning, EAutomationExpectedMessageFlags::Contains, 1);
	TestEqual(TEXT("one more is refused"), Imu->Track(Mast, FVector::ZeroVector), 0);
	for (const int32 Handle : Filled)
	{
		Imu->Untrack(Handle);
	}

	Imu->Untrack(PlainHandle);
	Imu->Untrack(ChaosHandle);
	TestFalse(TEXT("untracked"), Imu->IsTracked(PlainHandle) || Imu->IsTracked(ChaosHandle));
	return true;
}

// ---------------------------------------------------------------------------
// IMU streams (channels and the stream status; the feed itself needs a game instance's RMSS server)
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsImuStreamChannelsTest, "Ramms.Sensing.Imu.StreamChannels",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsImuStreamChannelsTest::RunTest(const FString& Parameters)
{
	using namespace RammsImuTests;
	FImuTestWorld Scratch;
	if (!TestNotNull(TEXT("world"), Scratch.World))
	{
		return false;
	}
	URammsImuStreamSubsystem* Streams = Scratch.World->GetSubsystem<URammsImuStreamSubsystem>();
	if (!TestNotNull(TEXT("a play world has the IMU stream subsystem"), Streams))
	{
		return false;
	}
	const auto Make = [](const TCHAR* Actor, const TCHAR* Target, int32 Channel) {
		FRammsImuStream Stream;
		Stream.ActorPath = Actor;
		Stream.ActorName = Actor;
		Stream.Target = Target;
		Stream.StreamId = FString::Printf(TEXT("imu/%s/%s"), Actor, Target);
		Stream.Channel = Channel;
		return Stream;
	};
	FString			Error;
	FRammsImuStream Started;
	TestTrue(TEXT("a first stream"), Streams->StartStream(Make(TEXT("A"), TEXT("chassis"), -1), Error, Started));
	TestEqual(TEXT("takes the first IMU channel"), Started.Channel, URammsImuStreamSubsystem::FirstChannel);
	TestTrue(TEXT("a second stream"), Streams->StartStream(Make(TEXT("A"), TEXT("seat"), -1), Error, Started));
	TestEqual(TEXT("takes the next"), Started.Channel, URammsImuStreamSubsystem::FirstChannel + 1);
	TestTrue(TEXT("restarting the first"), Streams->StartStream(Make(TEXT("A"), TEXT("chassis"), -1), Error, Started));
	TestEqual(TEXT("keeps its channel"), Started.Channel, URammsImuStreamSubsystem::FirstChannel);
	TestFalse(TEXT("a channel another IMU stream has is refused"), Streams->StartStream(Make(TEXT("B"), TEXT("chassis"), URammsImuStreamSubsystem::FirstChannel + 1), Error, Started));
	TestFalse(TEXT("a channel past 65535 is refused"), Streams->StartStream(Make(TEXT("B"), TEXT("chassis"), 70000), Error, Started));
	TestEqual(TEXT("two streams"), Streams->GetStreams().Num(), 2);

	TArray<TSharedPtr<FJsonValue>> Listed;
	RammsSensingBridgeImu::AppendImuStreams(Scratch.World, Listed);
	TestEqual(TEXT("the stream status lists both"), Listed.Num(), 2);
	if (Listed.Num() > 0)
	{
		const TSharedPtr<FJsonObject> Entry = Listed[0]->AsObject();
		TestEqual(TEXT("role"), Entry->GetStringField(TEXT("role")), FString(TEXT("imu")));
		TestEqual(TEXT("message"), Entry->GetStringField(TEXT("message")), FString(TEXT("FrameData")));
		TestEqual(TEXT("fmt"), Entry->GetStringField(TEXT("fmt")), FString(TEXT("imu_f64x18")));
		TestEqual(TEXT("no camera"), Entry->GetStringField(TEXT("camera")), FString());
		TestTrue(TEXT("names its target and stream id"), Entry->HasField(TEXT("target")) && Entry->HasField(TEXT("stream_id")) && Entry->HasField(TEXT("channel")));
	}
	TestTrue(TEXT("stops one"), Streams->StopStream(TEXT("A"), TEXT("seat")));
	TestFalse(TEXT("not twice"), Streams->StopStream(TEXT("A"), TEXT("seat")));
	TestTrue(TEXT("stops the other"), Streams->StopStream(TEXT("A"), TEXT("chassis")));
	return true;
}

// ---------------------------------------------------------------------------
// Chaos, simulating
// ---------------------------------------------------------------------------

namespace RammsImuTests
{
	struct FChaosRun
	{
		FImuTestWorld			   Scratch;
		URammsPhysicsImuSubsystem* Imu = nullptr;
		UBoxComponent*			   Resting = nullptr;
		UBoxComponent*			   Falling = nullptr;
		UBoxComponent*			   Offcentre = nullptr;
		UBoxComponent*			   Floating = nullptr;
		URammsIMUSensorComponent*  Sensor = nullptr;
		URammsIMUSensorComponent*  FallSensor = nullptr;
		URammsIMUSensorComponent*  SpinSensor = nullptr;
		AActor*					   Spinner = nullptr;
		FString					   RestingPath;
		int32					   RestHandle = 0;
		int32					   FallHandle = 0;
		int32					   OffcentreHandle = 0;
		int32					   FloatHandle = 0;
		int32					   Frames = 0;
		double					   Started = 0.0;
		/** Spin of the falling boxes (deg/s about Z), the point tracked on the falling one 20 cm off its axis, and
		 *  the centre-of-mass offset of the other (the point tracked on it is its origin). */
		double	SpinDeg = 180.0;
		FVector FallPoint = FVector(0.0, 20.0, 0.0);
		FVector ComOffset = FVector(10.0, 0.0, 0.0);
		/** The kinematic spinner's turn rate (deg/s about Z) and angle so far. */
		double SpinnerRateDeg = 90.0;
		double SpinnerYawDeg = 0.0;
		/** ReadImuSamplesJson's cursor on the IMU sensor, and what the reads returned. */
		int64 Cursor = -1;
		int64 SamplesRead = 0;
		int64 SamplesDropped = 0;
		bool  bCursorBroken = false;
	};
} // namespace RammsImuTests

/** Ticks the world until the boxes have been simulated for a few seconds, then checks what they read. */
class FRammsImuChaosRunCommand : public IAutomationLatentCommand
{
public:
	FRammsImuChaosRunCommand(FAutomationTestBase* InTest, TSharedRef<RammsImuTests::FChaosRun> InRun)
		: Test(InTest)
		, Run(InRun)
	{
	}

	/** One ReadImuSamplesJson of the resting box's IMU sensor from the run's cursor: the samples must follow on. */
	void ReadSamples()
	{
		using namespace RammsImuTests;
		const FString				  Options = FString::Printf(TEXT("{\"since\": %lld}"), Run->Cursor);
		const TSharedPtr<FJsonObject> Reply = ParseImuJson(URammsSensingBridge::ReadImuSamplesJson(Run->RestingPath, TEXT("Imu"), Options));
		bool						  bOk = false;
		if (!Reply.IsValid() || !Reply->TryGetBoolField(TEXT("ok"), bOk) || !bOk)
		{
			Run->bCursorBroken = true;
			return;
		}
		const int64							  Next = static_cast<int64>(Reply->GetNumberField(TEXT("next")));
		const int64							  Dropped = static_cast<int64>(Reply->GetNumberField(TEXT("dropped")));
		const TArray<TSharedPtr<FJsonValue>>* Samples = nullptr;
		Reply->TryGetArrayField(TEXT("samples"), Samples);
		const int32 Count = Samples ? Samples->Num() : 0;
		if (Run->Cursor >= 0)
		{
			// Seq Cursor + Dropped on, one after another, and the next cursor right after the last.
			int64  Expected = Run->Cursor + Dropped;
			double LastTime = -1.0;
			for (int32 i = 0; i < Count; ++i)
			{
				const TSharedPtr<FJsonObject> Entry = (*Samples)[i]->AsObject();
				const double				  Time = Entry->GetNumberField(TEXT("sim_time"));
				Run->bCursorBroken |= static_cast<int64>(Entry->GetNumberField(TEXT("seq"))) != Expected++ || Time <= LastTime;
				LastTime = Time;
			}
			Run->bCursorBroken |= Next != Expected;
			Run->SamplesRead += Count;
			Run->SamplesDropped += Dropped;
		}
		else
		{
			Run->bCursorBroken |= Count != 0; // the first call starts now
		}
		Run->Cursor = Next;
	}

	virtual bool Update() override
	{
		using namespace RammsImuTests;
		UWorld* World = Run->Scratch.World;
		if (!World || !Run->Imu)
		{
			return true;
		}
		// The kinematic spinner turns at a steady rate about world Z before each tick.
		const double Delta = 1.0 / 60.0;
		Run->SpinnerYawDeg += Run->SpinnerRateDeg * Delta;
		Run->Spinner->SetActorRotation(FRotator(0.0, Run->SpinnerYawDeg, 0.0));

		// The scratch world is not in the engine's loop: tick it (physics included).
		World->Tick(LEVELTICK_All, static_cast<float>(Delta));
		++Run->Frames;
		// Read every frame, as a client polling it would: a point not read for 10 s is dropped.
		const FString ReadingText = URammsSensingBridge::ReadBodyImuJson(Run->RestingPath, TEXT(""), TEXT("{\"offset\": [0, 0, 20]}"));
		ReadSamples();

		FRammsImuSample Rest;
		FRammsImuSample Fall;
		FRammsImuSample Offcentre;
		FRammsImuSample Float;
		const bool		bRest = Run->Imu->GetSample(Run->RestHandle, Rest) && Rest.bHasAcceleration;
		const bool		bFall = Run->Imu->GetSample(Run->FallHandle, Fall) && Fall.bHasAcceleration;
		const bool		bOffcentre = Run->Imu->GetSample(Run->OffcentreHandle, Offcentre) && Offcentre.bHasAcceleration;
		const bool		bFloat = Run->Imu->GetSample(Run->FloatHandle, Float) && Float.bHasAcceleration;
		// Three seconds of physics: the resting box has landed and settled, the others still fall (or float).
		if (!(bRest && bFall && bOffcentre && bFloat && Rest.SimTimeSeconds > 3.0 && Fall.SimTimeSeconds > 3.0))
		{
			if (Run->Frames > 3000 || FPlatformTime::Seconds() - Run->Started > 120.0)
			{
				Test->AddError(FString::Printf(TEXT("no Chaos samples after %d frames (rest %d at %.2f s, fall %d at %.2f s, offcentre %d, float %d)"),
					Run->Frames, bRest, Rest.SimTimeSeconds, bFall, Fall.SimTimeSeconds, bOffcentre, bFloat));
				return true;
			}
			return false;
		}

		Test->TestEqual(TEXT("the resting box is sampled by Chaos"), Rest.Source, ERammsImuSource::Chaos);
		NearlyEqualVectors(*Test, TEXT("at rest: +|g| up"), Rest.ProperAcceleration, FVector(0.0, 0.0, -World->GetGravityZ()), ImuChaosAccelSlackCm);
		NearlyEqualVectors(*Test, TEXT("at rest: still"), Rest.LinearVelocity, FVector::ZeroVector, 5.0);
		NearlyEqualVectors(*Test, TEXT("gravity is the solver's"), Rest.Gravity, FVector(0.0, 0.0, World->GetGravityZ()), 1.0e-2);

		Test->TestEqual(TEXT("the falling box is sampled by Chaos"), Fall.Source, ERammsImuSource::Chaos);
		Test->TestTrue(FString::Printf(TEXT("still falling (%s)"), *Fall.LinearVelocity.ToString()), Fall.LinearVelocity.Z < -1000.0);
		// In free fall only the spin is felt: w^2 r toward the axis, at the point 20 cm off it. The point is where it
		// was tracked on the box: 20 cm from the axis (a point that lost its lever arm would sit on the axis).
		const double  W = FMath::DegreesToRadians(Run->SpinDeg);
		const FVector Radial = Fall.Location - Run->Falling->GetComponentLocation();
		const FVector RadialXY(Radial.X, Radial.Y, 0.0);
		Test->TestTrue(FString::Printf(TEXT("the point is 20 cm off the axis (%.2f)"), RadialXY.Size()),
			FMath::Abs(RadialXY.Size() - Run->FallPoint.Size()) < 0.5);
		NearlyEqualVectors(*Test, TEXT("free fall: only the centripetal acceleration"), Fall.ProperAcceleration, -W * W * RadialXY, ImuChaosAccelSlackCm);
		NearlyEqualVectors(*Test, TEXT("free fall: the spin (deg/s)"), Fall.AngularVelocity, FVector(0.0, 0.0, Run->SpinDeg), 2.0);
		NearlyEqualVectors(*Test, TEXT("free fall: the point moves around the axis"), FVector(Fall.LinearVelocity.X, Fall.LinearVelocity.Y, 0.0),
			FVector::CrossProduct(FVector(0.0, 0.0, W), RadialXY), 2.0);

		// A box spinning about an off-centre centre of mass: its origin (the point tracked) circles that centre.
		const FVector Com = Run->Offcentre->GetCenterOfMass();
		const FVector Lever(Offcentre.Location.X - Com.X, Offcentre.Location.Y - Com.Y, 0.0);
		Test->TestTrue(FString::Printf(TEXT("off-centre: the origin is %.1f cm from the centre of mass (%.2f)"), Run->ComOffset.Size(), Lever.Size()),
			FMath::Abs(Lever.Size() - Run->ComOffset.Size()) < 0.5);
		NearlyEqualVectors(*Test, TEXT("off-centre: velocity about the centre of mass"), FVector(Offcentre.LinearVelocity.X, Offcentre.LinearVelocity.Y, 0.0),
			FVector::CrossProduct(FVector(0.0, 0.0, W), Lever), 2.0);
		NearlyEqualVectors(*Test, TEXT("off-centre: centripetal about the centre of mass"), Offcentre.ProperAcceleration, -W * W * Lever, ImuChaosAccelSlackCm);

		// A box whose gravity is off floats: no gravity applied, nothing felt.
		NearlyEqualVectors(*Test, TEXT("gravity off: none applied"), Float.Gravity, FVector::ZeroVector, 1.0e-6);
		NearlyEqualVectors(*Test, TEXT("gravity off: reads nothing"), Float.ProperAcceleration, FVector::ZeroVector, 1.0);

		// The IMU component on the resting box reads Chaos too, gravity included, in its own frame.
		if (Run->Sensor)
		{
			Test->TestEqual(TEXT("the IMU component's source"), Run->Sensor->GetActiveSource(), ERammsImuSource::Chaos);
			Test->TestTrue(TEXT("the IMU component's reading carries the engine's time"), Run->Sensor->GetIMUData().SimTimeSeconds > 3.0);
			NearlyEqualVectors(*Test, TEXT("the IMU component at rest"), Run->Sensor->GetIMUData().LinearAcceleration,
				Run->Sensor->GetComponentQuat().UnrotateVector(FVector(0.0, 0.0, -World->GetGravityZ())), ImuChaosAccelSlackCm);
		}
		// The IMU component on the falling box, rolled 90 degrees: over each interval it reads the centripetal
		// acceleration of its point and the spin, in its own frame (world Z is fixed in it while the box yaws).
		if (Run->FallSensor)
		{
			const FIMUSensorData Reading = Run->FallSensor->GetIMUData();
			Test->TestEqual(TEXT("the falling IMU component's source"), Run->FallSensor->GetActiveSource(), ERammsImuSource::Chaos);
			Test->TestTrue(FString::Printf(TEXT("falling IMU component: w^2 r over the interval (%.1f cm/s^2)"), Reading.LinearAcceleration.Size()),
				FMath::Abs(Reading.LinearAcceleration.Size() - W * W * Run->FallPoint.Size()) < 10.0);
			NearlyEqualVectors(*Test, TEXT("falling IMU component: the spin in its own frame"), Reading.AngularVelocity,
				Run->FallSensor->GetComponentQuat().UnrotateVector(FVector(0.0, 0.0, Run->SpinDeg)), 2.0);
		}
		// A Kinematic IMU component pitched 90 degrees on a body turned at a steady rate about world Z: the turn,
		// in its own frame.
		if (Run->SpinSensor)
		{
			Test->TestEqual(TEXT("the spinner's IMU is Kinematic"), Run->SpinSensor->GetActiveSource(), ERammsImuSource::Kinematic);
			NearlyEqualVectors(*Test, TEXT("kinematic gyro: the turn in the sensor's frame"), Run->SpinSensor->GetIMUData().AngularVelocity,
				Run->SpinSensor->GetComponentQuat().UnrotateVector(FVector(0.0, 0.0, Run->SpinnerRateDeg)), 1.0);
		}

		// Every step's samples of the IMU sensor reached Remote Control, in order, none lost.
		Test->TestFalse(TEXT("ReadImuSamplesJson: each read follows on from the last"), Run->bCursorBroken);
		Test->TestTrue(FString::Printf(TEXT("ReadImuSamplesJson: every step's samples (%lld)"), Run->SamplesRead), Run->SamplesRead >= 100);
		Test->TestEqual(TEXT("ReadImuSamplesJson: none dropped"), Run->SamplesDropped, int64(0));

		// The bridge reads a body without an IMU component.
		const TSharedPtr<FJsonObject> Reading = ParseImuJson(ReadingText);
		if (Test->TestTrue(TEXT("ReadBodyImuJson parses"), Reading.IsValid()))
		{
			bool bOk = false;
			Test->TestTrue(TEXT("ReadBodyImuJson ok"), Reading->TryGetBoolField(TEXT("ok"), bOk) && bOk);
			Test->TestEqual(TEXT("ReadBodyImuJson source"), Reading->GetStringField(TEXT("source")), FString(TEXT("chaos")));
			Test->TestTrue(TEXT("ReadBodyImuJson has a sample by now"), Reading->GetBoolField(TEXT("has_sample")));
			const TSharedPtr<FJsonObject>* Acceleration = nullptr;
			if (Test->TestTrue(TEXT("linear_acceleration"), Reading->TryGetObjectField(TEXT("linear_acceleration"), Acceleration) && Acceleration))
			{
				Test->TestTrue(TEXT("reads about +|g| on the sensor's Z"), FMath::Abs((*Acceleration)->GetNumberField(TEXT("z")) - ImuGravityCm) < ImuChaosAccelSlackCm);
			}
			for (const TCHAR* Key : { TEXT("body"), TEXT("sim_time"), TEXT("has_acceleration"), TEXT("angular_velocity"), TEXT("linear_velocity"), TEXT("units") })
			{
				Test->TestTrue(FString::Printf(TEXT("ReadBodyImuJson has \"%s\""), Key), Reading->HasField(Key));
			}
		}
		const TSharedPtr<FJsonObject>		  Listed = ParseImuJson(URammsSensingBridge::ListImuBodiesJson(Run->RestingPath));
		const TArray<TSharedPtr<FJsonValue>>* Bodies = nullptr;
		if (Test->TestTrue(TEXT("ListImuBodiesJson lists"), Listed.IsValid() && Listed->TryGetArrayField(TEXT("bodies"), Bodies) && Bodies && Bodies->Num() > 0))
		{
			const TSharedPtr<FJsonObject> First = (*Bodies)[0]->AsObject();
			Test->TestEqual(TEXT("the box, on Chaos"), First->GetStringField(TEXT("engine")), FString(TEXT("chaos")));
			Test->TestTrue(TEXT("the default body"), First->GetBoolField(TEXT("default")));
		}
		return true;
	}

private:
	FAutomationTestBase*				 Test;
	TSharedRef<RammsImuTests::FChaosRun> Run;
};

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsImuChaosRestAndFreeFallTest, "Ramms.Sensing.Imu.ChaosRestAndFreeFall",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsImuChaosRestAndFreeFallTest::RunTest(const FString& Parameters)
{
	using namespace RammsImuTests;
	TSharedRef<FChaosRun> Run = MakeShared<FChaosRun>();
	if (!TestNotNull(TEXT("world"), Run->Scratch.World))
	{
		return false;
	}
	Run->Imu = URammsPhysicsImuSubsystem::Get(Run->Scratch.World);
	if (!TestNotNull(TEXT("the IMU subsystem"), Run->Imu))
	{
		return false;
	}

	// A floor and a box dropped 80 cm onto it; far away, with nothing under them: a spinning box, a box spinning
	// about an off-centre centre of mass, and a box with its gravity off.
	Run->Scratch.SpawnBox(FVector(0.0, 0.0, -10.0), FVector(2000.0, 2000.0, 10.0), /*bSimulate=*/false);
	AActor* Crate = Run->Scratch.SpawnBox(FVector(0.0, 0.0, 100.0), FVector(20.0), /*bSimulate=*/true, &Run->Resting);
	AActor* Faller = Run->Scratch.SpawnBox(FVector(50000.0, 0.0, 0.0), FVector(20.0), /*bSimulate=*/true, &Run->Falling);
	Run->Falling->SetPhysicsAngularVelocityInDegrees(FVector(0.0, 0.0, Run->SpinDeg));
	Run->Scratch.SpawnBox(FVector(0.0, 50000.0, 0.0), FVector(20.0), /*bSimulate=*/true, &Run->Offcentre);
	Run->Offcentre->SetCenterOfMass(Run->ComOffset);
	Run->Offcentre->SetPhysicsAngularVelocityInDegrees(FVector(0.0, 0.0, Run->SpinDeg));
	Run->Scratch.SpawnBox(FVector(-50000.0, 0.0, 0.0), FVector(20.0), /*bSimulate=*/true, &Run->Floating);
	Run->Floating->SetEnableGravity(false);
	Run->RestingPath = Crate->GetPathName();

	// IMU components (begun play, so they tick): on top of the resting box; on the falling box 20 cm off its axis,
	// rolled 90 degrees; pitched 90 degrees on a plain actor the run turns about world Z (Kinematic: it only moves).
	Run->Sensor = FImuTestWorld::AddImu(Crate, Run->Resting, TEXT("Imu"), FVector(0.0, 0.0, 20.0), FRotator::ZeroRotator);
	Run->FallSensor = FImuTestWorld::AddImu(Faller, Run->Falling, TEXT("FallImu"), Run->FallPoint, FRotator(0.0, 0.0, 90.0));
	Run->Spinner = Run->Scratch.SpawnPlain(FVector(0.0, -50000.0, 0.0));
	Run->SpinSensor = FImuTestWorld::AddImu(Run->Spinner, Run->Spinner->GetRootComponent(), TEXT("SpinImu"), FVector(30.0, 0.0, 0.0), FRotator(90.0, 0.0, 0.0),
		/*bUsePhysicsVelocity=*/false);
	Crate->DispatchBeginPlay();
	Faller->DispatchBeginPlay();
	Run->Spinner->DispatchBeginPlay();

	Run->RestHandle = Run->Imu->Track(Run->Resting, FVector(10.0, 0.0, 20.0));
	Run->FallHandle = Run->Imu->Track(Run->Falling, Run->FallPoint);
	Run->OffcentreHandle = Run->Imu->Track(Run->Offcentre, FVector::ZeroVector);
	Run->FloatHandle = Run->Imu->Track(Run->Floating, FVector::ZeroVector);
	TestEqual(TEXT("the resting box is on Chaos"), Run->Imu->GetTrackedSource(Run->RestHandle), ERammsImuSource::Chaos);
	TestEqual(TEXT("the falling box is on Chaos"), Run->Imu->GetTrackedSource(Run->FallHandle), ERammsImuSource::Chaos);
	TestEqual(TEXT("the off-centre box is on Chaos"), Run->Imu->GetTrackedSource(Run->OffcentreHandle), ERammsImuSource::Chaos);
	// Tracking starts on the first read; its first answer has no sample yet (and so no sim_time).
	const TSharedPtr<FJsonObject> First = ParseImuJson(URammsSensingBridge::ReadBodyImuJson(Run->RestingPath, TEXT(""), TEXT("{\"offset\": [0, 0, 20]}")));
	TestTrue(TEXT("the first ReadBodyImuJson answers"), First.IsValid() && First->HasField(TEXT("has_acceleration")));
	TestTrue(TEXT("no sim_time without a sample"), First.IsValid() && (First->GetBoolField(TEXT("has_sample")) || !First->HasField(TEXT("sim_time"))));
	// StartImuStreamJson needs a game instance's RMSS server, which the scratch world has none of.
	const TSharedPtr<FJsonObject> Stream = ParseImuJson(URammsSensingBridge::StartImuStreamJson(Run->RestingPath, TEXT(""), TEXT("")));
	TestTrue(TEXT("no stream without a streaming subsystem"), Stream.IsValid() && !Stream->GetBoolField(TEXT("ok")));

	Run->Started = FPlatformTime::Seconds();
	ADD_LATENT_AUTOMATION_COMMAND(FRammsImuChaosRunCommand(this, Run));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
