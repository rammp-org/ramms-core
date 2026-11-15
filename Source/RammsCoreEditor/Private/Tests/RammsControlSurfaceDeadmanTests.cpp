// Tests for the control surface's drive deadman (URammsRobotControlSurfaceComponent::ExpireQuietHolds and its ticker).
//
// A Continuous or Velocity control a Remote or Autonomy source holds springs back once nothing has been heard from
// that source on the surface for DeadmanSeconds (plus a grace of the last two frames): a Continuous one as a release
// from that source would, a Velocity one by being commanded its default. Nothing else is touched.
//
// Checked on a bare game world: actors with a surface and a test contributor (Continuous controls that let go
// themselves or refuse to, a Position, a Velocity, an Enum and an Action), with the differential and holonomic drives
// (whose external input is latched behind the control), and with raw registry motors on a fake actuation backend.
// Every surface runs on a fake clock (SetClockForTesting), so nothing waits on real time.
//
// In the editor module with their test contributor (a UCLASS), which this keeps out of game and Shipping builds.

#include "CoreMinimal.h"
#include "Engine/DataTable.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "RammsActuationBackend.h"
#include "RammsActuationBackendRegistry.h"
#include "RammsControlIds.h"
#include "RammsControlSurfaceTestContributor.h"
#include "RammsDifferentialDriveController.h"
#include "RammsHolonomicDriveController.h"
#include "RammsMotorSpec.h"
#include "RammsRobotBaseComponent.h"
#include "RammsRobotControlSurfaceComponent.h"
#include "UObject/Package.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace RammsDeadmanTests
{
	using FContributor = URammsControlSurfaceTestContributor;
	using FSurface = URammsRobotControlSurfaceComponent;

	/** A bare game world, torn down with everything spawned in it. */
	struct FDeadmanWorld
	{
		UWorld*			World = nullptr;
		TArray<AActor*> Spawned;

		FDeadmanWorld()
		{
			World = UWorld::CreateWorld(EWorldType::Game, /*bInformEngineOfWorld=*/false);
			if (World != nullptr)
			{
				GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
			}
		}

		~FDeadmanWorld()
		{
			if (World == nullptr)
			{
				return;
			}
			for (AActor* Actor : Spawned)
			{
				if (IsValid(Actor))
				{
					Actor->Destroy();
				}
			}
			GEngine->DestroyWorldContext(World);
			World->DestroyWorld(false);
		}

		AActor* SpawnBare()
		{
			AActor* Actor = World != nullptr ? World->SpawnActor<AActor>() : nullptr;
			if (Actor != nullptr)
			{
				Spawned.Add(Actor);
			}
			return Actor;
		}

		/** An actor with a control surface (0.3 s holds and deadman) and a component of type T beside it, neither
		 *  registered nor begun play: the surface gathers what the actor owns, which is all the deadman needs. */
		template <typename T>
		FSurface* SpawnRobot(T*& OutComponent)
		{
			OutComponent = nullptr;
			AActor* Actor = SpawnBare();
			if (Actor == nullptr)
			{
				return nullptr;
			}
			OutComponent = NewObject<T>(Actor, TEXT("Controller"));
			FSurface* Surface = NewObject<FSurface>(Actor, TEXT("ControlSurface"));
			Surface->ExternalHoldSeconds = 0.3f;
			Surface->DeadmanSeconds = 0.3f;
			return Surface;
		}
	};

	/** The clock a surface runs on in these tests: advanced by hand. Shared, so a surface outliving its test (until
	 *  it is collected) never reads a dead stack frame. */
	struct FFakeClock
	{
		double Now = 1000.0;
	};

	TSharedRef<FFakeClock> UseFakeClock(FSurface& Surface)
	{
		TSharedRef<FFakeClock> Clock = MakeShared<FFakeClock>();
		Surface.SetClockForTesting([Clock]() { return Clock->Now; });
		return Clock;
	}

	float ValueOf(const FSurface& Surface, FName Id)
	{
		return Surface.GetControlValue(Id);
	}

	ERammsControlSource OwnerOf(const FSurface& Surface, FName Id)
	{
		return Surface.GetAxisOwner_Implementation(Id);
	}

	/** Register Component and begin its play, as its actor's BeginPlay would. */
	bool BeginPlayOf(UActorComponent& Component)
	{
		Component.RegisterComponent();
		if (!Component.IsRegistered())
		{
			return false;
		}
		Component.BeginPlay();
		return Component.HasBegunPlay();
	}

	void EndPlayOf(UActorComponent& Component)
	{
		if (Component.HasBegunPlay())
		{
			Component.EndPlay(EEndPlayReason::EndPlayInEditor);
		}
	}

	/** What the fake actuation backend was asked to do. */
	struct FFakeBackendLog
	{
		TMap<FName, float> Commands;
		TArray<FName>	   Released;
	};

	/** An actuation backend with no physics: it records commands and reads them back; like a MuJoCo <velocity>
	 *  actuator, it cannot let go of a motor. */
	class FFakeBackend final : public IRammsActuationBackend
	{
	public:
		explicit FFakeBackend(const TSharedRef<FFakeBackendLog>& InLog)
			: Log(InLog)
		{
		}

		virtual bool  Initialize(URammsRobotBaseComponent&) override { return true; }
		virtual void  SetCommand(FName MotorId, float Value) override { Log->Commands.Add(MotorId, Value); }
		virtual float GetValue(FName MotorId) const override
		{
			const float* Value = Log->Commands.Find(MotorId);
			return Value ? *Value : 0.0f;
		}
		virtual float GetVelocity(FName) const override { return 0.0f; }
		virtual bool  ReleaseMotor(FName MotorId) override
		{
			Log->Released.Add(MotorId);
			return false;
		}
		virtual bool GetMotorTransform(FName, FTransform& OutWorld) const override
		{
			OutWorld = FTransform::Identity;
			return true;
		}

	private:
		TSharedRef<FFakeBackendLog> Log;
	};
} // namespace RammsDeadmanTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanExpiry,
	"Ramms.Control.Deadman.Expiry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanExpiry::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld TestWorld;
	FContributor* C = nullptr;
	FSurface*	  Surface = TestWorld.SpawnRobot(C);
	if (!TestNotNull(TEXT("a robot"), Surface) || !TestNotNull(TEXT("its contributor"), C))
	{
		return false;
	}
	const TSharedRef<FFakeClock> Clock = UseFakeClock(*Surface);
	const FName					 Rate = FContributor::RateId();
	const FName					 Trim = FContributor::TrimId();

	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.8f, ERammsControlSource::Remote));
	TestTrue(TEXT("Autonomy sets the trim"), Surface->SetControl(Trim, -0.5f, ERammsControlSource::Autonomy));
	const double Set = Clock->Now;
	TestEqual(TEXT("both watched"), Surface->GetDeadmanWatchCount(), 2);

	// Inside the window: nothing happens.
	TestEqual(TEXT("0.2 s on: nothing sprung back"), Surface->ExpireQuietHolds(Set + 0.2), 0);
	TestEqual(TEXT("...the rate as set"), ValueOf(*Surface, Rate), 0.8f);
	TestEqual(TEXT("...the trim as set"), ValueOf(*Surface, Trim), -0.5f);
	TestEqual(TEXT("...still held by Remote"), OwnerOf(*Surface, Rate), ERammsControlSource::Remote);

	// Past it: both spring back, each the way a release from its source does.
	TestEqual(TEXT("0.31 s on: both sprung back"), Surface->ExpireQuietHolds(Set + 0.31), 2);
	TestEqual(TEXT("the rate: let go by its contributor, back to 0"), ValueOf(*Surface, Rate), 0.0f);
	TestEqual(TEXT("...through its ReleaseControl, once"), C->ReleasesOf(Rate), 1);
	TestEqual(TEXT("the trim: its contributor refuses a release, so the surface applied its DefaultValue"), ValueOf(*Surface, Trim),
		FContributor::TrimDefault);
	TestEqual(TEXT("...after asking it to let go, once"), C->ReleasesOf(Trim), 1);
	TestEqual(TEXT("the holds are gone (no owner)"), OwnerOf(*Surface, Rate), ERammsControlSource::Script);
	TestEqual(TEXT("...on the trim too"), OwnerOf(*Surface, Trim), ERammsControlSource::Script);
	float Target = -1.0f;
	TestTrue(TEXT("the rate's target is its default"), Surface->GetControlTarget(Rate, Target) && Target == 0.0f);
	TestTrue(TEXT("the trim's target is its default"), Surface->GetControlTarget(Trim, Target) && Target == FContributor::TrimDefault);
	TestEqual(TEXT("nothing left to watch"), Surface->GetDeadmanWatchCount(), 0);
	TestEqual(TEXT("a second look does nothing more"), Surface->ExpireQuietHolds(Set + 5.0), 0);
	TestEqual(TEXT("...no second release"), C->ReleasesOf(Rate), 1);

	// A source that releases its own command leaves the deadman nothing to do.
	Clock->Now = Set + 6.0;
	TestTrue(TEXT("Remote sets the rate again"), Surface->SetControl(Rate, 0.4f, ERammsControlSource::Remote));
	TestTrue(TEXT("...and releases it"), Surface->ReleaseControl(Rate, ERammsControlSource::Remote));
	TestEqual(TEXT("released: not watched"), Surface->GetDeadmanWatchCount(), 0);
	TestEqual(TEXT("...and not released again later"), Surface->ExpireQuietHolds(Clock->Now + 5.0), 0);
	TestEqual(TEXT("...two releases in all"), C->ReleasesOf(Rate), 2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanKeepAlive,
	"Ramms.Control.Deadman.KeepAlivePerSource",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanKeepAlive::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld TestWorld;
	FContributor* C = nullptr;
	FSurface*	  Surface = TestWorld.SpawnRobot(C);
	if (!TestNotNull(TEXT("a robot"), Surface) || !TestNotNull(TEXT("its contributor"), C))
	{
		return false;
	}
	const TSharedRef<FFakeClock> Clock = UseFakeClock(*Surface);
	const FName					 Rate = FContributor::RateId();
	const FName					 Trim = FContributor::TrimId();
	const FName					 Position = FContributor::PositionId();

	// A client sets the trim once and keeps sending the rate: it is still there, so both stand.
	const double T0 = Clock->Now;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.6f, ERammsControlSource::Remote));
	TestTrue(TEXT("...and the trim"), Surface->SetControl(Trim, 0.9f, ERammsControlSource::Remote));
	for (const double Dt : { 0.2, 0.4, 0.6 })
	{
		Clock->Now = T0 + Dt;
		TestTrue(TEXT("Remote sends the rate again"), Surface->SetControl(Rate, 0.6f, ERammsControlSource::Remote));
		TestEqual(TEXT("...nothing sprung back"), Surface->ExpireQuietHolds(Clock->Now), 0);
	}
	TestEqual(TEXT("the trim, set once 0.65 s ago, stands while its source is heard"), Surface->ExpireQuietHolds(T0 + 0.65), 0);
	TestEqual(TEXT("...as set"), ValueOf(*Surface, Trim), 0.9f);

	// Another source's commands are not this one being there.
	Clock->Now = T0 + 0.7;
	TestTrue(TEXT("Autonomy sets the position"), Surface->SetControl(Position, 10.0f, ERammsControlSource::Autonomy));
	TestEqual(TEXT("0.29 s after Remote was last heard: nothing"), Surface->ExpireQuietHolds(T0 + 0.89), 0);
	TestEqual(TEXT("0.31 s after: both of Remote's spring back"), Surface->ExpireQuietHolds(T0 + 0.91), 2);
	TestEqual(TEXT("...the rate"), ValueOf(*Surface, Rate), 0.0f);
	TestEqual(TEXT("...the trim"), ValueOf(*Surface, Trim), FContributor::TrimDefault);

	// A trigger is heard...
	const double T1 = T0 + 2.0;
	Clock->Now = T1;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.5f, ERammsControlSource::Remote));
	Clock->Now = T1 + 0.2;
	TestTrue(TEXT("Remote fires the action"), Surface->TriggerControl(FContributor::GoId(), ERammsControlSource::Remote));
	TestEqual(TEXT("...which keeps the rate"), Surface->ExpireQuietHolds(T1 + 0.31), 0);
	TestEqual(TEXT("...until it too is 0.3 s old"), Surface->ExpireQuietHolds(T1 + 0.51), 1);

	// ...so is a release of something else...
	const double T2 = T1 + 2.0;
	Clock->Now = T2;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.5f, ERammsControlSource::Remote));
	TestTrue(TEXT("...and the position"), Surface->SetControl(Position, 20.0f, ERammsControlSource::Remote));
	Clock->Now = T2 + 0.2;
	TestTrue(TEXT("Remote releases the position"), Surface->ReleaseControl(Position, ERammsControlSource::Remote));
	TestEqual(TEXT("...which keeps the rate"), Surface->ExpireQuietHolds(T2 + 0.31), 0);
	TestEqual(TEXT("...until that is 0.3 s old"), Surface->ExpireQuietHolds(T2 + 0.51), 1);

	// ...but a command the surface refused is not.
	const double T3 = T2 + 2.0;
	Clock->Now = T3;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.5f, ERammsControlSource::Remote));
	Clock->Now = T3 + 0.1;
	TestTrue(TEXT("Autonomy sets the trim"), Surface->SetControl(Trim, 0.1f, ERammsControlSource::Autonomy));
	Clock->Now = T3 + 0.2;
	TestFalse(TEXT("Remote's trim is refused: Autonomy holds it"), Surface->SetControl(Trim, 0.2f, ERammsControlSource::Remote));
	TestEqual(TEXT("0.31 s after Remote was last applied: its rate springs back, Autonomy's trim stays"), Surface->ExpireQuietHolds(T3 + 0.31), 1);
	TestEqual(TEXT("...the rate"), ValueOf(*Surface, Rate), 0.0f);
	TestEqual(TEXT("...the trim, Autonomy's"), ValueOf(*Surface, Trim), 0.1f);
	TestEqual(TEXT("...still held by Autonomy"), OwnerOf(*Surface, Trim), ERammsControlSource::Autonomy);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanGrace,
	"Ramms.Control.Deadman.FrameGrace",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanGrace::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld TestWorld;
	FContributor* C = nullptr;
	FSurface*	  Surface = TestWorld.SpawnRobot(C);
	FContributor* C2 = nullptr;
	FSurface*	  Bare = TestWorld.SpawnRobot(C2);
	if (!TestNotNull(TEXT("a robot"), Surface) || !TestNotNull(TEXT("a second robot"), Bare))
	{
		return false;
	}
	const TSharedRef<FFakeClock>   Clock = UseFakeClock(*Surface);
	const TSharedRef<FFakeClock>   BareClock = UseFakeClock(*Bare);
	const FName					   Rate = FContributor::RateId();
	const TWeakObjectPtr<FSurface> Weak(Surface);

	const double T0 = Clock->Now;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.7f, ERammsControlSource::Remote));
	TestTrue(TEXT("...and the deadman's ticker runs"), Surface->IsDeadmanTickerActive());

	// Frames of 0, 0.25 and 0.35 s: the grace is the last two frames' length, so 0.6 s of quiet is within
	// 0.3 + 0.25 + 0.35. Without it -- the second robot, looked at with no grace -- the same quiet is too long.
	Clock->Now = T0;
	TestTrue(TEXT("the first frame (no gap yet)"), FSurface::StepDeadmanTicker(Weak));
	Clock->Now = T0 + 0.25;
	TestTrue(TEXT("a 0.25 s frame"), FSurface::StepDeadmanTicker(Weak));
	Clock->Now = T0 + 0.6;
	TestTrue(TEXT("a 0.35 s frame after it: 0.6 s quiet, within 0.3 + 0.6"), FSurface::StepDeadmanTicker(Weak));
	TestEqual(TEXT("...the rate as set"), ValueOf(*Surface, Rate), 0.7f);
	BareClock->Now = T0;
	TestTrue(TEXT("the second robot: Remote sets its rate"), Bare->SetControl(Rate, 0.7f, ERammsControlSource::Remote));
	TestEqual(TEXT("...0.6 s quiet with no grace: sprung back"), Bare->ExpireQuietHolds(T0 + 0.6, 0.0), 1);

	// Short frames again: the grace shrinks back.
	Clock->Now = T0 + 0.7;
	TestTrue(TEXT("a 0.1 s frame: 0.7 s quiet, within 0.3 + 0.35 + 0.1"), FSurface::StepDeadmanTicker(Weak));
	TestEqual(TEXT("...the rate as set"), ValueOf(*Surface, Rate), 0.7f);
	Clock->Now = T0 + 0.8;
	TestFalse(TEXT("another 0.1 s frame: 0.8 s quiet, past 0.3 + 0.1 + 0.1 -- sprung back, and the ticker stops"),
		FSurface::StepDeadmanTicker(Weak));
	TestEqual(TEXT("...the rate back to 0"), ValueOf(*Surface, Rate), 0.0f);
	TestFalse(TEXT("...the ticker's entry removed"), Surface->IsDeadmanTickerActive());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanTicker,
	"Ramms.Control.Deadman.TickerLifetime",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanTicker::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld TestWorld;
	const FName	  Rate = FContributor::RateId();

	// Started by the first command it has to watch, stopped once nothing is left, started again by the next.
	{
		FContributor* C = nullptr;
		FSurface*	  Surface = TestWorld.SpawnRobot(C);
		if (!TestNotNull(TEXT("a robot"), Surface))
		{
			return false;
		}
		const TSharedRef<FFakeClock>   Clock = UseFakeClock(*Surface);
		const TWeakObjectPtr<FSurface> Weak(Surface);
		TestFalse(TEXT("idle: no ticker"), Surface->IsDeadmanTickerActive());
		TestTrue(TEXT("the keyboard sets the rate"), Surface->SetControl(Rate, 0.5f, ERammsControlSource::Keyboard));
		TestFalse(TEXT("...which starts nothing"), Surface->IsDeadmanTickerActive());
		Clock->Now += 1.0;
		TestTrue(TEXT("Remote sets the position"), Surface->SetControl(FContributor::PositionId(), 10.0f, ERammsControlSource::Remote));
		TestFalse(TEXT("...which starts nothing either"), Surface->IsDeadmanTickerActive());
		TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.5f, ERammsControlSource::Remote));
		TestTrue(TEXT("...which starts the ticker"), Surface->IsDeadmanTickerActive());

		// (A single 5 s frame would be its own grace: the ticker cannot tell a frame that long from silence. So the
		// spring-back itself is done by hand, and the ticker's next frame finds nothing left.)
		Clock->Now += 0.1;
		TestTrue(TEXT("a frame on: still watching"), FSurface::StepDeadmanTicker(Weak));
		TestEqual(TEXT("5 s of quiet: sprung back"), Surface->ExpireQuietHolds(Clock->Now + 5.0), 1);
		Clock->Now += 5.0;
		TestFalse(TEXT("...and the next frame, with nothing left, removes the entry"), FSurface::StepDeadmanTicker(Weak));
		TestFalse(TEXT("...stopped"), Surface->IsDeadmanTickerActive());
		TestEqual(TEXT("...nothing watched"), Surface->GetDeadmanWatchCount(), 0);

		const double T1 = Clock->Now;
		TestTrue(TEXT("Remote sets the rate again"), Surface->SetControl(Rate, 0.4f, ERammsControlSource::Remote));
		TestTrue(TEXT("...the ticker starts again"), Surface->IsDeadmanTickerActive());
		// Its frames counted afresh from the command that started it: nothing of the frames before the stop.
		Clock->Now = T1 + 0.2;
		TestTrue(TEXT("its first frame, 0.2 s on: still watching"), FSurface::StepDeadmanTicker(Weak));
		Clock->Now = T1 + 0.4;
		TestTrue(TEXT("0.4 s on (0.3 + two 0.2 s frames): still watching"), FSurface::StepDeadmanTicker(Weak));
		Clock->Now = T1 + 0.5;
		TestTrue(TEXT("0.5 s on (0.3 + 0.2 + 0.1): still watching"), FSurface::StepDeadmanTicker(Weak));
		Clock->Now = T1 + 0.6;
		TestFalse(TEXT("0.6 s on (0.3 + two 0.1 s frames): sprung back, stopped"), FSurface::StepDeadmanTicker(Weak));
		TestEqual(TEXT("...the rate back to 0"), ValueOf(*Surface, Rate), 0.0f);
		TestFalse(TEXT("...stopped"), Surface->IsDeadmanTickerActive());
	}

	// EndPlay removes it (a world ending, PIE stopping).
	{
		FContributor* C = nullptr;
		FSurface*	  Surface = TestWorld.SpawnRobot(C);
		if (!TestNotNull(TEXT("a robot to end"), Surface) || !TestTrue(TEXT("...begun play"), BeginPlayOf(*Surface)))
		{
			return false;
		}
		UseFakeClock(*Surface);
		TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.5f, ERammsControlSource::Remote));
		TestTrue(TEXT("...the ticker runs"), Surface->IsDeadmanTickerActive());
		EndPlayOf(*Surface);
		TestFalse(TEXT("EndPlay: the ticker's entry removed"), Surface->IsDeadmanTickerActive());
	}

	// A surface that is gone takes its entry with it.
	{
		FContributor* C = nullptr;
		FSurface*	  Surface = TestWorld.SpawnRobot(C);
		if (!TestNotNull(TEXT("a robot to destroy"), Surface))
		{
			return false;
		}
		UseFakeClock(*Surface);
		TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.5f, ERammsControlSource::Remote));
		const TWeakObjectPtr<FSurface> Weak(Surface);
		Surface->GetOwner()->Destroy();
		TestFalse(TEXT("destroyed: the weak pointer no longer resolves"), Weak.IsValid());
		TestFalse(TEXT("...and the ticker's entry ends itself"), FSurface::StepDeadmanTicker(Weak));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanOtherSources,
	"Ramms.Control.Deadman.OtherSources",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanOtherSources::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld TestWorld;
	FContributor* C = nullptr;
	FSurface*	  Surface = TestWorld.SpawnRobot(C);
	if (!TestNotNull(TEXT("a robot"), Surface) || !TestNotNull(TEXT("its contributor"), C))
	{
		return false;
	}
	const TSharedRef<FFakeClock> Clock = UseFakeClock(*Surface);
	const FName					 Rate = FContributor::RateId();

	// Local and scripted commands stand until their source lets go.
	struct FNamedSource
	{
		ERammsControlSource Source;
		const TCHAR*		Name;
	};
	const FNamedSource Local[] = { { ERammsControlSource::Keyboard, TEXT("Keyboard") }, { ERammsControlSource::Gamepad, TEXT("Gamepad") },
		{ ERammsControlSource::Touch, TEXT("Touch") }, { ERammsControlSource::Script, TEXT("Script") } };
	for (const FNamedSource& Entry : Local)
	{
		const ERammsControlSource Source = Entry.Source;
		const FString			  What = Entry.Name;
		TestFalse(What + TEXT(": not a deadman source"), FSurface::IsDeadmanSource(Source));
		TestTrue(What + TEXT(" sets the rate"), Surface->SetControl(Rate, 0.5f, Source));
		TestEqual(What + TEXT(": not watched"), Surface->GetDeadmanWatchCount(), 0);
		TestEqual(What + TEXT(": 5 s on, nothing sprung back"), Surface->ExpireQuietHolds(Clock->Now + 5.0), 0);
		TestEqual(What + TEXT(": the command stands"), ValueOf(*Surface, Rate), 0.5f);
		TestTrue(What + TEXT(" lets go itself"), Surface->ReleaseControl(Rate, Source));
	}
	TestEqual(TEXT("released only by the sources themselves"), C->ReleasesOf(Rate), 4);
	TestTrue(TEXT("Remote is a deadman source"), FSurface::IsDeadmanSource(ERammsControlSource::Remote));
	TestTrue(TEXT("Autonomy is a deadman source"), FSurface::IsDeadmanSource(ERammsControlSource::Autonomy));

	// Taken over: once a Remote command's hold has lapsed, a local source may drive, and its command is not the
	// deadman's to spring back.
	const double T0 = Clock->Now;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.9f, ERammsControlSource::Remote));
	TestEqual(TEXT("...watched"), Surface->GetDeadmanWatchCount(), 1);
	Clock->Now = T0 + 0.1;
	TestFalse(TEXT("0.1 s on, the keyboard is refused: Remote holds it"), Surface->SetControl(Rate, -0.3f, ERammsControlSource::Keyboard));
	Clock->Now = T0 + 0.31;
	TestTrue(TEXT("0.31 s on, the keyboard takes it over"), Surface->SetControl(Rate, -0.3f, ERammsControlSource::Keyboard));
	TestEqual(TEXT("...no longer watched"), Surface->GetDeadmanWatchCount(), 0);
	TestEqual(TEXT("...and not sprung back"), Surface->ExpireQuietHolds(Clock->Now + 5.0), 0);
	TestEqual(TEXT("...the keyboard's command stands"), ValueOf(*Surface, Rate), -0.3f);
	TestEqual(TEXT("...held by the keyboard"), OwnerOf(*Surface, Rate), ERammsControlSource::Keyboard);
	TestEqual(TEXT("...no release behind its back"), C->ReleasesOf(Rate), 4);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanStandingKinds,
	"Ramms.Control.Deadman.PositionEnumActionUnaffected",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanStandingKinds::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld TestWorld;
	FContributor* C = nullptr;
	FSurface*	  Surface = TestWorld.SpawnRobot(C);
	if (!TestNotNull(TEXT("a robot"), Surface) || !TestNotNull(TEXT("its contributor"), C))
	{
		return false;
	}
	const TSharedRef<FFakeClock> Clock = UseFakeClock(*Surface);
	const FName					 Position = FContributor::PositionId();
	const FName					 Choice = FContributor::ChoiceId();

	// A position target is where the robot is asked to be, not a command that must keep coming; a choice and an
	// action likewise.
	for (const ERammsControlSource Source : { ERammsControlSource::Remote, ERammsControlSource::Autonomy })
	{
		Clock->Now += 10.0;
		TestTrue(TEXT("a deadman source sets the position"), Surface->SetControl(Position, 40.0f, Source));
		TestTrue(TEXT("...picks a choice"), Surface->SetControl(Choice, 2.0f, Source));
		TestTrue(TEXT("...and fires the action"), Surface->TriggerControl(FContributor::GoId(), Source));
		TestEqual(TEXT("...none of them watched"), Surface->GetDeadmanWatchCount(), 0);
		TestFalse(TEXT("...no ticker"), Surface->IsDeadmanTickerActive());
		TestEqual(TEXT("...5 s on, nothing sprung back"), Surface->ExpireQuietHolds(Clock->Now + 5.0), 0);
		TestEqual(TEXT("...the target held"), ValueOf(*Surface, Position), 40.0f);
		TestEqual(TEXT("...the choice made"), ValueOf(*Surface, Choice), 2.0f);
		float Target = 0.0f;
		TestTrue(TEXT("...and reported"), Surface->GetControlTarget(Position, Target) && Target == 40.0f);
		TestEqual(TEXT("...never released"), C->ReleasesOf(Position), 0);
	}
	TestEqual(TEXT("the action fired twice"), C->TriggersOf(FContributor::GoId()), 2);

	// A Remote rate beside them springs back on its own: the position is left where it is.
	Clock->Now += 10.0;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(FContributor::RateId(), 1.0f, ERammsControlSource::Remote));
	TestEqual(TEXT("the rate sprang back"), Surface->ExpireQuietHolds(Clock->Now + 5.0), 1);
	TestEqual(TEXT("...the position still held"), ValueOf(*Surface, Position), 40.0f);
	TestEqual(TEXT("...and still not released"), C->ReleasesOf(Position), 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanVelocity,
	"Ramms.Control.Deadman.Velocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanVelocity::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld TestWorld;
	FContributor* C = nullptr;
	FSurface*	  Surface = TestWorld.SpawnRobot(C);
	if (!TestNotNull(TEXT("a robot"), Surface) || !TestNotNull(TEXT("its contributor"), C))
	{
		return false;
	}
	const TSharedRef<FFakeClock> Clock = UseFakeClock(*Surface);
	const FName					 Speed = FContributor::SpeedId();

	int32 Applies = 0;
	for (const ERammsControlSource Source : { ERammsControlSource::Remote, ERammsControlSource::Autonomy })
	{
		Clock->Now += 10.0;
		const double T = Clock->Now;
		TestTrue(TEXT("a deadman source sets a speed"), Surface->SetControl(Speed, 3.0f, Source));
		++Applies;
		TestEqual(TEXT("...watched"), Surface->GetDeadmanWatchCount(), 1);
		TestEqual(TEXT("0.2 s on: nothing"), Surface->ExpireQuietHolds(T + 0.2), 0);
		TestEqual(TEXT("...the speed as set"), ValueOf(*Surface, Speed), 3.0f);
		TestEqual(TEXT("0.31 s on: stopped"), Surface->ExpireQuietHolds(T + 0.31), 1);
		++Applies;
		TestEqual(TEXT("...commanded its default (0) through ApplyControl"), ValueOf(*Surface, Speed), 0.0f);
		TestEqual(TEXT("...one more ApplyControl"), C->AppliesOf(Speed), Applies);
		TestEqual(TEXT("...not released (a released speed servo may coast)"), C->ReleasesOf(Speed), 0);
		TestEqual(TEXT("...the hold gone"), OwnerOf(*Surface, Speed), ERammsControlSource::Script);
		float Target = -1.0f;
		TestTrue(TEXT("...its target is 0"), Surface->GetControlTarget(Speed, Target) && Target == 0.0f);
		TestEqual(TEXT("...nothing left to watch"), Surface->GetDeadmanWatchCount(), 0);
	}

	// A local speed stands.
	TestTrue(TEXT("the gamepad sets a speed"), Surface->SetControl(Speed, -2.0f, ERammsControlSource::Gamepad));
	TestEqual(TEXT("...not watched"), Surface->GetDeadmanWatchCount(), 0);
	TestEqual(TEXT("...5 s on, nothing sprung back"), Surface->ExpireQuietHolds(Clock->Now + 5.0), 0);
	TestEqual(TEXT("...still -2"), ValueOf(*Surface, Speed), -2.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanLatchedDrives,
	"Ramms.Control.Deadman.LatchedDrives",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanLatchedDrives::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	const FName	  Forward = RammsControlIds::Drive::Forward();
	const FName	  Turn = RammsControlIds::Drive::Turn();
	const FName	  Strafe = RammsControlIds::Drive::Strafe();
	FDeadmanWorld TestWorld;

	// The differential drive latches the last command as its external drive input: nothing else zeroes it once the
	// client is gone, which is what the deadman is for.
	URammsDifferentialDriveController* Diff = nullptr;
	FSurface*						   DiffSurface = TestWorld.SpawnRobot(Diff);
	if (!TestNotNull(TEXT("a differential robot"), DiffSurface) || !TestNotNull(TEXT("its drive"), Diff))
	{
		return false;
	}
	const TSharedRef<FFakeClock> DiffClock = UseFakeClock(*DiffSurface);
	TestTrue(TEXT("Remote drives forward"), DiffSurface->SetControl(Forward, 0.6f, ERammsControlSource::Remote));
	TestTrue(TEXT("...and turns"), DiffSurface->SetControl(Turn, -0.4f, ERammsControlSource::Remote));
	const double DiffSet = DiffClock->Now;
	TestEqual(TEXT("the drive input latched (forward)"), Diff->DriveInput.Y, 0.6, 1e-6);
	TestEqual(TEXT("the drive input latched (turn)"), Diff->DriveInput.X, -0.4, 1e-6);
	TestEqual(TEXT("not yet quiet"), DiffSurface->ExpireQuietHolds(DiffSet + 0.1), 0);
	TestEqual(TEXT("...still driving"), Diff->DriveInput.Y, 0.6, 1e-6);
	TestEqual(TEXT("the client went quiet: both axes sprung back"), DiffSurface->ExpireQuietHolds(DiffSet + 0.31), 2);
	TestEqual(TEXT("...the latched input is zero (forward)"), Diff->DriveInput.Y, 0.0, 1e-9);
	TestEqual(TEXT("...the latched input is zero (turn)"), Diff->DriveInput.X, 0.0, 1e-9);
	TestEqual(TEXT("...and so is what the surface reads back"), ValueOf(*DiffSurface, Forward), 0.0f);

	// A keyboard command on the same drive is not the deadman's.
	TestTrue(TEXT("the keyboard drives forward"), DiffSurface->SetControl(Forward, 0.5f, ERammsControlSource::Keyboard));
	TestEqual(TEXT("...not sprung back"), DiffSurface->ExpireQuietHolds(DiffClock->Now + 5.0), 0);
	TestEqual(TEXT("...still driving"), Diff->DriveInput.Y, 0.5, 1e-6);

	// The holonomic drive keeps its command until told otherwise too.
	URammsHolonomicDriveController* Holo = nullptr;
	FSurface*						HoloSurface = TestWorld.SpawnRobot(Holo);
	if (!TestNotNull(TEXT("a holonomic robot"), HoloSurface) || !TestNotNull(TEXT("its drive"), Holo))
	{
		return false;
	}
	const TSharedRef<FFakeClock> HoloClock = UseFakeClock(*HoloSurface);
	TestTrue(TEXT("Autonomy drives forward"), HoloSurface->SetControl(Forward, 0.3f, ERammsControlSource::Autonomy));
	TestTrue(TEXT("...strafes"), HoloSurface->SetControl(Strafe, -0.2f, ERammsControlSource::Autonomy));
	TestTrue(TEXT("...and turns"), HoloSurface->SetControl(Turn, 0.1f, ERammsControlSource::Autonomy));
	const double HoloSet = HoloClock->Now;
	TestEqual(TEXT("the command held (strafe)"), ValueOf(*HoloSurface, Strafe), -0.2f);
	TestEqual(TEXT("quiet: all three sprung back"), HoloSurface->ExpireQuietHolds(HoloSet + 0.31), 3);
	TestEqual(TEXT("...forward 0"), ValueOf(*HoloSurface, Forward), 0.0f);
	TestEqual(TEXT("...strafe 0"), ValueOf(*HoloSurface, Strafe), 0.0f);
	TestEqual(TEXT("...turn 0"), ValueOf(*HoloSurface, Turn), 0.0f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanModeSwitch,
	"Ramms.Control.Deadman.ModeSwitchDropsStaleHold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanModeSwitch::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	const FName	  Forward = RammsControlIds::Drive::Forward();
	const FName	  Turn = RammsControlIds::Drive::Turn();
	FDeadmanWorld TestWorld;

	// One robot, two drive modes on the same drive controls: the differential drive active, the holonomic stood down.
	AActor* Actor = TestWorld.SpawnBare();
	if (!TestNotNull(TEXT("a robot"), Actor))
	{
		return false;
	}
	URammsDifferentialDriveController* Diff = NewObject<URammsDifferentialDriveController>(Actor, TEXT("Diff"));
	URammsHolonomicDriveController*	   Holo = NewObject<URammsHolonomicDriveController>(Actor, TEXT("Holo"));
	FSurface*						   Surface = NewObject<FSurface>(Actor, TEXT("ControlSurface"));
	Holo->SetDriveModeActive(false);
	const TSharedRef<FFakeClock> Clock = UseFakeClock(*Surface);

	TestTrue(TEXT("Remote drives forward"), Surface->SetControl(Forward, 0.6f, ERammsControlSource::Remote));
	TestTrue(TEXT("...and turns"), Surface->SetControl(Turn, 0.4f, ERammsControlSource::Remote));
	TestEqual(TEXT("both watched"), Surface->GetDeadmanWatchCount(), 2);

	// The switch: the controls change hands, so the surface drops the holds -- without releasing anything: the mode
	// that stood down let go of its command itself.
	Diff->SetDriveModeActive(false);
	Holo->SetDriveModeActive(true);
	Surface->RebuildControlSurface();
	TestEqual(TEXT("the hold dropped (no owner)"), OwnerOf(*Surface, Forward), ERammsControlSource::Script);
	TestEqual(TEXT("the differential drive stood down at rest (forward)"), Diff->DriveInput.Y, 0.0, 1e-9);
	TestEqual(TEXT("...(turn)"), Diff->DriveInput.X, 0.0, 1e-9);
	TestEqual(TEXT("the deadman has nothing to spring back"), Surface->ExpireQuietHolds(Clock->Now + 5.0), 0);
	TestEqual(TEXT("...nor anything left to watch"), Surface->GetDeadmanWatchCount(), 0);
	TestEqual(TEXT("the holonomic drive came up at rest"), ValueOf(*Surface, Forward), 0.0f);

	// And back. A single-axis command must not bring the turn from before the switch back with it.
	Holo->SetDriveModeActive(false);
	Diff->SetDriveModeActive(true);
	Surface->RebuildControlSurface();
	TestTrue(TEXT("the keyboard drives forward alone"), Surface->SetControl(Forward, 0.5f, ERammsControlSource::Keyboard));
	TestEqual(TEXT("...forward 0.5"), Diff->DriveInput.Y, 0.5, 1e-6);
	TestEqual(TEXT("...and no turn left over from before the switch"), Diff->DriveInput.X, 0.0, 1e-9);
	TestTrue(TEXT("...let go"), Surface->ReleaseControl(Forward, ERammsControlSource::Keyboard));
	TestEqual(TEXT("...at rest, not turning"), Diff->DriveInput.X, 0.0, 1e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanReleaseRebuilds,
	"Ramms.Control.Deadman.ReleaseRebuildsSurface",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanReleaseRebuilds::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld TestWorld;
	FContributor* C = nullptr;
	FSurface*	  Surface = TestWorld.SpawnRobot(C);
	if (!TestNotNull(TEXT("a robot"), Surface) || !TestNotNull(TEXT("its contributor"), C))
	{
		return false;
	}
	const TSharedRef<FFakeClock> Clock = UseFakeClock(*Surface);
	const FName					 Rate = FContributor::RateId();
	const FName					 Trim = FContributor::TrimId();
	C->RebuildOnRateRelease = Surface;

	// Releasing the rate rebuilds the surface in the middle of the deadman's pass: both still spring back.
	const double T0 = Clock->Now;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.8f, ERammsControlSource::Remote));
	TestTrue(TEXT("...and the trim"), Surface->SetControl(Trim, -0.5f, ERammsControlSource::Remote));
	const int32 VersionBefore = Surface->GetControlSurfaceVersion_Implementation();
	TestEqual(TEXT("quiet: both sprung back"), Surface->ExpireQuietHolds(T0 + 0.31), 2);
	TestTrue(TEXT("...the surface was rebuilt on the way"), Surface->GetControlSurfaceVersion_Implementation() > VersionBefore);
	TestEqual(TEXT("...the rate let go"), ValueOf(*Surface, Rate), 0.0f);
	TestEqual(TEXT("...the trim back to its default"), ValueOf(*Surface, Trim), FContributor::TrimDefault);
	TestEqual(TEXT("...no holds"), OwnerOf(*Surface, Trim), ERammsControlSource::Script);
	TestEqual(TEXT("...nothing watched"), Surface->GetDeadmanWatchCount(), 0);

	// This time the rebuild drops the trim: whichever comes first, the trim is gone with its hold and nothing is
	// left over -- sprung back if it came first, forgotten if the rate's rebuild took it.
	C->bDropTrimOnRateRelease = true;
	const double T1 = T0 + 2.0;
	Clock->Now = T1;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.6f, ERammsControlSource::Remote));
	TestTrue(TEXT("...and the trim"), Surface->SetControl(Trim, 0.7f, ERammsControlSource::Remote));
	TestTrue(TEXT("quiet: the rate at least sprung back"), Surface->ExpireQuietHolds(T1 + 0.31) >= 1);
	TestEqual(TEXT("...released (twice in all)"), C->ReleasesOf(Rate), 2);
	TestNull(TEXT("...the trim no longer on the surface"), Surface->DescribeControlSurface().Find(Trim));
	TestEqual(TEXT("...and not held"), OwnerOf(*Surface, Trim), ERammsControlSource::Script);
	TestEqual(TEXT("...nothing watched"), Surface->GetDeadmanWatchCount(), 0);
	TestEqual(TEXT("...and nothing more later"), Surface->ExpireQuietHolds(T1 + 5.0), 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanRawMotors,
	"Ramms.Control.Deadman.RawMotors",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanRawMotors::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld TestWorld;
	AActor*		  Actor = TestWorld.SpawnBare();
	if (!TestNotNull(TEXT("a robot"), Actor))
	{
		return false;
	}

	// A registry of three motors no contributor claims: the surface exposes each as a raw motor.<id> control.
	UScriptStruct* SpecStruct = FindObject<UScriptStruct>(nullptr, TEXT("/Script/RammsCore.RammsMotorSpec"));
	if (!TestNotNull(TEXT("the motor spec struct"), SpecStruct))
	{
		return false;
	}
	UDataTable* Table = NewObject<UDataTable>(GetTransientPackage());
	Table->RowStruct = SpecStruct;
	FRammsMotorSpec Wheel;
	Wheel.Id = FName(TEXT("wheel"));
	Wheel.Type = ERammsActuatorType::Velocity;
	Wheel.ControlRange = FVector2D(-10.0, 10.0);
	Table->AddRow(Wheel.Id, Wheel);
	FRammsMotorSpec Drive;
	Drive.Id = FName(TEXT("drive"));
	Drive.Type = ERammsActuatorType::Torque;
	Table->AddRow(Drive.Id, Drive);
	FRammsMotorSpec Lift;
	Lift.Id = FName(TEXT("lift"));
	Lift.Type = ERammsActuatorType::Position;
	Lift.ControlRange = FVector2D(-1.0, 1.0);
	Table->AddRow(Lift.Id, Lift);

	// A fake actuation backend in the MuJoCo slot for the length of the test, the registered one put back after.
	const TSharedRef<FFakeBackendLog>	Log = MakeShared<FFakeBackendLog>();
	const FRammsActuationBackendFactory Previous = RammsActuationBackends::GetMujocoFactory();
	ON_SCOPE_EXIT
	{
		if (Previous.IsBound())
		{
			RammsActuationBackends::RegisterMujocoFactory(Previous);
		}
		else
		{
			RammsActuationBackends::UnregisterMujocoFactory();
		}
	};
	RammsActuationBackends::RegisterMujocoFactory(FRammsActuationBackendFactory::CreateLambda(
		[Log](URammsRobotBaseComponent&) -> IRammsActuationBackend* { return new FFakeBackend(Log); }));

	URammsRobotBaseComponent* Base = NewObject<URammsRobotBaseComponent>(Actor, TEXT("RobotBase"));
	Base->MotorTable = Table;
	FSurface* Surface = NewObject<FSurface>(Actor, TEXT("ControlSurface"));
	// BeginPlay is where the surface finds the robot's base.
	if (!TestTrue(TEXT("the surface begun play"), BeginPlayOf(*Surface)) || !TestTrue(TEXT("...with a backend"), Base->HasBackend()))
	{
		EndPlayOf(*Surface);
		return false;
	}
	const TSharedRef<FFakeClock> Clock = UseFakeClock(*Surface);
	const FName					 WheelId(TEXT("motor.wheel"));
	const FName					 DriveId(TEXT("motor.drive"));
	const FName					 LiftId(TEXT("motor.lift"));
	const FRammsControlSurface	 Description = Surface->DescribeControlSurface();
	const FRammsControlAxis*	 WheelAxis = Description.Find(WheelId);
	const FRammsControlAxis*	 DriveAxis = Description.Find(DriveId);
	TestTrue(TEXT("the wheel is a Velocity control"), WheelAxis && WheelAxis->Kind == ERammsControlKind::Velocity);
	TestTrue(TEXT("the torque motor is a Continuous one"), DriveAxis && DriveAxis->Kind == ERammsControlKind::Continuous);

	const double T0 = Clock->Now;
	TestTrue(TEXT("Remote spins the wheel"), Surface->SetControl(WheelId, 2.0f, ERammsControlSource::Remote));
	TestTrue(TEXT("...pushes the torque motor"), Surface->SetControl(DriveId, 5.0f, ERammsControlSource::Remote));
	TestTrue(TEXT("...and moves the lift"), Surface->SetControl(LiftId, 0.5f, ERammsControlSource::Remote));
	TestEqual(TEXT("the wheel commanded"), Log->Commands.FindRef(FName(TEXT("wheel"))), 2.0f);
	TestEqual(TEXT("the speed and the torque watched, not the lift"), Surface->GetDeadmanWatchCount(), 2);

	TestEqual(TEXT("quiet: the wheel and the torque motor sprang back"), Surface->ExpireQuietHolds(T0 + 0.31), 2);
	TestEqual(TEXT("...the wheel commanded 0 (stopped)"), Log->Commands.FindRef(FName(TEXT("wheel"))), 0.0f);
	TestEqual(TEXT("...the torque 0"), Log->Commands.FindRef(FName(TEXT("drive"))), 0.0f);
	TestEqual(TEXT("...nothing released (the backend could not have)"), Log->Released.Num(), 0);
	TestEqual(TEXT("...the lift where it was put"), Log->Commands.FindRef(FName(TEXT("lift"))), 0.5f);
	TestEqual(TEXT("...the wheel's hold gone"), OwnerOf(*Surface, WheelId), ERammsControlSource::Script);
	float Target = -1.0f;
	TestTrue(TEXT("...and its target 0"), Surface->GetControlTarget(WheelId, Target) && Target == 0.0f);
	TestEqual(TEXT("...the lift still Remote's"), OwnerOf(*Surface, LiftId), ERammsControlSource::Remote);

	EndPlayOf(*Surface);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanRobotOwn,
	"Ramms.Control.Deadman.RobotOwnCommands",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanRobotOwn::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld TestWorld;
	FContributor* C = nullptr;
	FSurface*	  Surface = TestWorld.SpawnRobot(C);
	if (!TestNotNull(TEXT("a robot"), Surface) || !TestNotNull(TEXT("its contributor"), C))
	{
		return false;
	}
	const TSharedRef<FFakeClock> Clock = UseFakeClock(*Surface);
	const FName					 Rate = FContributor::RateId();
	const FName					 Trim = FContributor::TrimId();

	// An Autonomy client drives; the robot's own controllers command as Autonomy too (a stance, the arm held still).
	const double T0 = Clock->Now;
	TestTrue(TEXT("an Autonomy client sets the rate"), Surface->SetControl(Rate, 0.5f, ERammsControlSource::Autonomy));
	Clock->Now = T0 + 0.2;
	TestTrue(TEXT("the robot holds the trim"), Surface->SetControlInternal(Trim, 0.1f, ERammsControlSource::Autonomy));
	TestTrue(TEXT("...and fires the action"), Surface->TriggerControlInternal(FContributor::GoId(), ERammsControlSource::Autonomy));
	TestEqual(TEXT("...its trim not watched"), Surface->GetDeadmanWatchCount(), 1);
	TestEqual(TEXT("the robot's commands keep no client alive: the client's rate springs back"), Surface->ExpireQuietHolds(T0 + 0.31), 1);
	TestEqual(TEXT("...to 0"), ValueOf(*Surface, Rate), 0.0f);
	TestEqual(TEXT("the robot's trim is never sprung back"), Surface->ExpireQuietHolds(T0 + 5.0), 0);
	TestEqual(TEXT("...as it set it"), ValueOf(*Surface, Trim), 0.1f);
	Clock->Now = T0 + 0.2 + 0.31;
	TestTrue(TEXT("...and its hold lapses as any does"), Surface->SetControl(Trim, 0.3f, ERammsControlSource::Keyboard));

	// The robot taking over a client's control takes it off the watch.
	Clock->Now = T0 + 2.0;
	TestTrue(TEXT("a Remote client sets the rate"), Surface->SetControl(Rate, 0.7f, ERammsControlSource::Remote));
	TestEqual(TEXT("...watched"), Surface->GetDeadmanWatchCount(), 1);
	Clock->Now = T0 + 2.1;
	TestTrue(TEXT("the robot takes the rate (Autonomy outranks Remote)"), Surface->SetControlInternal(Rate, 0.0f, ERammsControlSource::Autonomy));
	TestEqual(TEXT("...no longer watched"), Surface->GetDeadmanWatchCount(), 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanCarried,
	"Ramms.Control.Deadman.CarriedKeepsStamp",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanCarried::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld TestWorld;
	FContributor* C = nullptr;
	FSurface*	  Surface = TestWorld.SpawnRobot(C);
	if (!TestNotNull(TEXT("a robot"), Surface) || !TestNotNull(TEXT("its contributor"), C))
	{
		return false;
	}
	const TSharedRef<FFakeClock> Clock = UseFakeClock(*Surface);
	const FName					 Rate = FContributor::RateId();

	// A client that went quiet before a mode switch: the switch carries its command, as of when it was sent.
	const double T0 = Clock->Now;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.8f, ERammsControlSource::Remote));
	Clock->Now = T0 + 0.2;
	double HoldTime = 0.0;
	TestTrue(TEXT("its hold time"), Surface->GetHoldTime(Rate, HoldTime) && HoldTime == T0);
	TestTrue(TEXT("carried over 0.2 s later"), Surface->ReapplyHeldControl(Rate, 0.8f, ERammsControlSource::Remote, HoldTime));
	TestTrue(TEXT("...keeping its time"), Surface->GetHoldTime(Rate, HoldTime) && HoldTime == T0);
	TestEqual(TEXT("...watched still"), Surface->GetDeadmanWatchCount(), 1);
	TestEqual(TEXT("0.31 s after it was sent: sprung back on schedule"), Surface->ExpireQuietHolds(T0 + 0.31), 1);

	// The client sending again is another matter.
	const double T1 = T0 + 2.0;
	Clock->Now = T1;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.8f, ERammsControlSource::Remote));
	Clock->Now = T1 + 0.2;
	TestTrue(TEXT("...and sends it again"), Surface->SetControl(Rate, 0.8f, ERammsControlSource::Remote));
	TestEqual(TEXT("0.31 s after the first send: kept"), Surface->ExpireQuietHolds(T1 + 0.31), 0);

	// A carried command of a client still being heard stands.
	const double T2 = T1 + 2.0;
	Clock->Now = T2;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.5f, ERammsControlSource::Remote));
	Clock->Now = T2 + 0.25;
	TestTrue(TEXT("...and the position"), Surface->SetControl(FContributor::PositionId(), 5.0f, ERammsControlSource::Remote));
	TestTrue(TEXT("the rate carried over"), Surface->ReapplyHeldControl(Rate, 0.5f, ERammsControlSource::Remote, T2));
	TestEqual(TEXT("0.31 s after the rate was sent, 0.06 s after the client was heard: kept"), Surface->ExpireQuietHolds(T2 + 0.31), 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanMultiControlClient,
	"Ramms.Control.Deadman.MultiControlClient",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanMultiControlClient::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	const FName	  Forward = RammsControlIds::Drive::Forward();
	const FName	  Strafe = RammsControlIds::Drive::Strafe();
	const FName	  Turn = RammsControlIds::Drive::Turn();
	FDeadmanWorld TestWorld;

	// A client holding forward, strafe and turn, one request per frame (a Remote Control client sending each control
	// in its own call is answered once a frame), on an editor throttled to 3 frames a second in the background: each
	// control is sent every third frame, a second apart, far past 0.3 s -- but the client is heard every frame.
	struct FSend
	{
		FName Id;
		float Value;
	};
	const FSend	 Sends[] = { { Forward, 0.5f }, { Strafe, 0.3f }, { Turn, 0.2f } };
	const double Frame = 1.0 / 3.0;

	for (const bool bAnsweredFirst : { true, false })
	{
		const FString					What = bAnsweredFirst ? TEXT("answered before the deadman looks: ") : TEXT("the deadman looks first: ");
		URammsHolonomicDriveController* Holo = nullptr;
		FSurface*						Surface = TestWorld.SpawnRobot(Holo);
		if (!TestNotNull(TEXT("a holonomic robot"), Surface))
		{
			return false;
		}
		const TSharedRef<FFakeClock>   Clock = UseFakeClock(*Surface);
		const TWeakObjectPtr<FSurface> Weak(Surface);
		const double				   T0 = Clock->Now;
		constexpr int32				   SendFrames = 12;
		bool						   bSprangBack = false;
		for (int32 k = 0; k < SendFrames; ++k)
		{
			Clock->Now = T0 + k * Frame;
			const FSend& Send = Sends[k % 3];
			if (bAnsweredFirst)
			{
				Surface->SetControl(Send.Id, Send.Value, ERammsControlSource::Remote);
				FSurface::StepDeadmanTicker(Weak);
			}
			else
			{
				FSurface::StepDeadmanTicker(Weak);
				Surface->SetControl(Send.Id, Send.Value, ERammsControlSource::Remote);
			}
			if (k >= 2)
			{
				for (const FSend& Each : Sends)
				{
					bSprangBack |= ValueOf(*Surface, Each.Id) != Each.Value;
				}
			}
		}
		TestFalse(What + TEXT("nothing sprang back while the client sent"), bSprangBack);

		// The client stops. Its last request was at frame 11; it is gone once 0.3 s plus the last two frames have
		// passed: not at +1/3 or +2/3 s, at +1 s, all three at once.
		const double Last = T0 + (SendFrames - 1) * Frame;
		Clock->Now = Last + Frame;
		FSurface::StepDeadmanTicker(Weak);
		Clock->Now = Last + 2.0 * Frame;
		FSurface::StepDeadmanTicker(Weak);
		TestEqual(What + TEXT("2 frames after the last send, still forward"), ValueOf(*Surface, Forward), 0.5f);
		Clock->Now = Last + 3.0 * Frame;
		TestFalse(What + TEXT("3 frames after: sprung back, the ticker stops"), FSurface::StepDeadmanTicker(Weak));
		TestEqual(What + TEXT("...forward 0"), ValueOf(*Surface, Forward), 0.0f);
		TestEqual(What + TEXT("...strafe 0"), ValueOf(*Surface, Strafe), 0.0f);
		TestEqual(What + TEXT("...turn 0"), ValueOf(*Surface, Turn), 0.0f);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanClamps,
	"Ramms.Control.Deadman.WindowsClamped",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanClamps::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld TestWorld;
	FContributor* C = nullptr;
	FSurface*	  Surface = TestWorld.SpawnRobot(C);
	if (!TestNotNull(TEXT("a robot"), Surface) || !TestNotNull(TEXT("its contributor"), C))
	{
		return false;
	}
	const TSharedRef<FFakeClock> Clock = UseFakeClock(*Surface);
	const FName					 Rate = FContributor::RateId();

	// Whatever they are set to (ExternalHoldSeconds is Blueprint- and so Remote Control-writable), both windows are
	// used within [0.1, 2] s.
	Surface->ExternalHoldSeconds = 0.0f;
	TestEqual(TEXT("a hold of 0 is used as 0.1 s"), Surface->GetExternalHoldSecondsClamped(), 0.1f);
	Surface->ExternalHoldSeconds = 1.0e9f;
	TestEqual(TEXT("an endless hold as 2 s"), Surface->GetExternalHoldSecondsClamped(), 2.0f);
	Surface->ExternalHoldSeconds = -5.0f;
	TestEqual(TEXT("a negative one as 0.1 s"), Surface->GetExternalHoldSecondsClamped(), 0.1f);

	// The arbitration holds for 0.1 s at the least...
	Surface->ExternalHoldSeconds = 0.0f;
	double T = Clock->Now;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.5f, ERammsControlSource::Remote));
	Clock->Now = T + 0.05;
	TestFalse(TEXT("...0.05 s on, the keyboard is still refused"), Surface->SetControl(Rate, 0.2f, ERammsControlSource::Keyboard));
	Clock->Now = T + 0.11;
	TestTrue(TEXT("...0.11 s on, it may drive"), Surface->SetControl(Rate, 0.2f, ERammsControlSource::Keyboard));

	// ...and the deadman's window likewise.
	Surface->DeadmanSeconds = 0.0f;
	TestEqual(TEXT("a deadman window of 0 is used as 0.1 s"), Surface->GetDeadmanSecondsClamped(), 0.1f);
	Clock->Now += 1.0;
	T = Clock->Now;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.5f, ERammsControlSource::Remote));
	TestEqual(TEXT("...0.05 s on: kept"), Surface->ExpireQuietHolds(T + 0.05), 0);
	TestEqual(TEXT("...0.11 s on: sprung back"), Surface->ExpireQuietHolds(T + 0.11), 1);

	Surface->DeadmanSeconds = 100.0f;
	TestEqual(TEXT("a deadman window of 100 s is used as 2 s"), Surface->GetDeadmanSecondsClamped(), 2.0f);
	Clock->Now += 1.0;
	T = Clock->Now;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.5f, ERammsControlSource::Remote));
	TestEqual(TEXT("...1.9 s on: kept"), Surface->ExpireQuietHolds(T + 1.9), 0);
	TestEqual(TEXT("...2.01 s on: sprung back"), Surface->ExpireQuietHolds(T + 2.01), 1);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
