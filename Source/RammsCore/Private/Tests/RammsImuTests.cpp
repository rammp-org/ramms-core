// Tests for physics-sourced IMU data: the rigid-body arithmetic every provider shares (pure), the game-rate
// differencer, the Chaos sample from two steps, which engine Auto picks for a point, and Chaos itself in a ticking
// world: a box at rest reads +|g| up, a box in free fall reads only the centripetal acceleration of its spin.

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
#include "RammsPhysicsImuSubsystem.h"
#include "RammsSensingBridge.h"
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
	const double  Dt = 1.0e-4;
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

	Imu->Untrack(PlainHandle);
	Imu->Untrack(ChaosHandle);
	TestFalse(TEXT("untracked"), Imu->IsTracked(PlainHandle) || Imu->IsTracked(ChaosHandle));
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
		URammsIMUSensorComponent*  Sensor = nullptr;
		FString					   RestingPath;
		int32					   RestHandle = 0;
		int32					   FallHandle = 0;
		int32					   Frames = 0;
		double					   Started = 0.0;
		/** Spin of the falling box (deg/s about Z) and the point tracked on it, 20 cm off its axis. */
		double	SpinDeg = 180.0;
		FVector FallPoint = FVector(0.0, 20.0, 0.0);
	};
} // namespace RammsImuTests

/** Ticks the world until both boxes have been simulated for a few seconds, then checks what they read. */
class FRammsImuChaosRunCommand : public IAutomationLatentCommand
{
public:
	FRammsImuChaosRunCommand(FAutomationTestBase* InTest, TSharedRef<RammsImuTests::FChaosRun> InRun)
		: Test(InTest)
		, Run(InRun)
	{
	}

	virtual bool Update() override
	{
		using namespace RammsImuTests;
		UWorld* World = Run->Scratch.World;
		if (!World || !Run->Imu)
		{
			return true;
		}
		// The scratch world is not in the engine's loop: tick it (physics included).
		World->Tick(LEVELTICK_All, 1.0f / 60.0f);
		++Run->Frames;
		// Read every frame, as a client polling it would: a point not read for 10 s is dropped.
		const FString ReadingText = URammsSensingBridge::ReadBodyImuJson(Run->RestingPath, TEXT(""), TEXT("{\"offset\": [0, 0, 20]}"));

		FRammsImuSample Rest;
		FRammsImuSample Fall;
		const bool		bRest = Run->Imu->GetSample(Run->RestHandle, Rest) && Rest.bHasAcceleration;
		const bool		bFall = Run->Imu->GetSample(Run->FallHandle, Fall) && Fall.bHasAcceleration;
		// Three seconds of physics: the resting box has landed and settled, the other is still falling.
		if (!(bRest && bFall && Rest.SimTimeSeconds > 3.0 && Fall.SimTimeSeconds > 3.0))
		{
			if (Run->Frames > 3000 || FPlatformTime::Seconds() - Run->Started > 120.0)
			{
				Test->AddError(FString::Printf(TEXT("no Chaos samples after %d frames (rest %d at %.2f s, fall %d at %.2f s)"), Run->Frames,
					bRest, Rest.SimTimeSeconds, bFall, Fall.SimTimeSeconds));
				return true;
			}
			return false;
		}

		Test->TestEqual(TEXT("the resting box is sampled by Chaos"), Rest.Source, ERammsImuSource::Chaos);
		NearlyEqualVectors(*Test, TEXT("at rest: +|g| up"), Rest.ProperAcceleration, FVector(0.0, 0.0, -World->GetGravityZ()), ImuChaosAccelSlackCm);
		NearlyEqualVectors(*Test, TEXT("at rest: still"), Rest.LinearVelocity, FVector::ZeroVector, 5.0);

		Test->TestEqual(TEXT("the falling box is sampled by Chaos"), Fall.Source, ERammsImuSource::Chaos);
		Test->TestTrue(FString::Printf(TEXT("still falling (%s)"), *Fall.LinearVelocity.ToString()), Fall.LinearVelocity.Z < -1000.0);
		// In free fall only the spin is felt: w^2 r toward the axis, at the point 20 cm off it.
		const double  W = FMath::DegreesToRadians(Run->SpinDeg);
		const FVector Radial = Fall.Location - Run->Falling->GetComponentLocation();
		const FVector Expected = -W * W * FVector(Radial.X, Radial.Y, 0.0).GetSafeNormal() * Run->FallPoint.Size();
		NearlyEqualVectors(*Test, TEXT("free fall: only the centripetal acceleration"), Fall.ProperAcceleration, Expected, ImuChaosAccelSlackCm);
		NearlyEqualVectors(*Test, TEXT("free fall: the spin (deg/s)"), Fall.AngularVelocity, FVector(0.0, 0.0, Run->SpinDeg), 2.0);

		// The IMU component on the resting box reads Chaos too, gravity included, in its own frame.
		if (Run->Sensor)
		{
			Test->TestEqual(TEXT("the IMU component's source"), Run->Sensor->GetActiveSource(), ERammsImuSource::Chaos);
			NearlyEqualVectors(*Test, TEXT("the IMU component at rest"), Run->Sensor->GetIMUData().LinearAcceleration,
				Run->Sensor->GetComponentQuat().UnrotateVector(FVector(0.0, 0.0, -World->GetGravityZ())), ImuChaosAccelSlackCm);
		}

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

	// A floor, a box dropped 80 cm onto it, and a spinning box far away with nothing under it.
	Run->Scratch.SpawnBox(FVector(0.0, 0.0, -10.0), FVector(2000.0, 2000.0, 10.0), /*bSimulate=*/false);
	AActor* Crate = Run->Scratch.SpawnBox(FVector(0.0, 0.0, 100.0), FVector(20.0), /*bSimulate=*/true, &Run->Resting);
	Run->Scratch.SpawnBox(FVector(50000.0, 0.0, 0.0), FVector(20.0), /*bSimulate=*/true, &Run->Falling);
	Run->Falling->SetPhysicsAngularVelocityInDegrees(FVector(0.0, 0.0, Run->SpinDeg));
	Run->RestingPath = Crate->GetPathName();

	// An IMU component on top of the resting box, Source Auto (begun play, so it ticks).
	Run->Sensor = NewObject<URammsIMUSensorComponent>(Crate, TEXT("Imu"));
	Run->Sensor->SetupAttachment(Run->Resting);
	Run->Sensor->SetRelativeLocation(FVector(0.0, 0.0, 20.0));
	Run->Sensor->UpdateRateHz = 0.0f;
	Run->Sensor->RegisterComponent();
	Crate->DispatchBeginPlay();

	Run->RestHandle = Run->Imu->Track(Run->Resting, FVector(10.0, 0.0, 20.0));
	Run->FallHandle = Run->Imu->Track(Run->Falling, Run->FallPoint);
	TestEqual(TEXT("the resting box is on Chaos"), Run->Imu->GetTrackedSource(Run->RestHandle), ERammsImuSource::Chaos);
	TestEqual(TEXT("the falling box is on Chaos"), Run->Imu->GetTrackedSource(Run->FallHandle), ERammsImuSource::Chaos);
	// Tracking starts on the first read; its first answer has no sample yet.
	const TSharedPtr<FJsonObject> First = ParseImuJson(URammsSensingBridge::ReadBodyImuJson(Run->RestingPath, TEXT(""), TEXT("{\"offset\": [0, 0, 20]}")));
	TestTrue(TEXT("the first ReadBodyImuJson answers"), First.IsValid() && First->HasField(TEXT("has_acceleration")));

	Run->Started = FPlatformTime::Seconds();
	ADD_LATENT_AUTOMATION_COMMAND(FRammsImuChaosRunCommand(this, Run));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
