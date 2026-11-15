// Tests for the control surface's drive deadman (URammsRobotControlSurfaceComponent::ExpireQuietHolds).
//
// A Continuous control set by a Remote or Autonomy source springs back to its DefaultValue, as a ReleaseControl
// from that source would, once that source has not set it again for ExternalHoldSeconds; nothing else is touched.
// Checked on a bare game world: an actor with a surface and a test contributor (Continuous controls that let go
// themselves or refuse to, and a Position control), and actors with the differential and holonomic drives, whose
// external input is latched behind the control. The deadman is stepped on its own clock, starting from the time the
// surface stamped on the command (taken right after it), so nothing waits on real time but one short refresh gap.

#include "CoreMinimal.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include "RammsControlIds.h"
#include "RammsControlSurfaceTestContributor.h"
#include "RammsDifferentialDriveController.h"
#include "RammsHolonomicDriveController.h"
#include "RammsRobotControlSurfaceComponent.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace RammsDeadmanTests
{
	using FContributor = URammsControlSurfaceTestContributor;

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

		/** An actor with a control surface (0.3 s holds) and a component of type T beside it, neither registered
		 *  nor begun play: the surface gathers what the actor owns, which is all the deadman needs. */
		template <typename T>
		URammsRobotControlSurfaceComponent* SpawnRobot(T*& OutComponent)
		{
			OutComponent = nullptr;
			AActor* Actor = World != nullptr ? World->SpawnActor<AActor>() : nullptr;
			if (Actor == nullptr)
			{
				return nullptr;
			}
			Spawned.Add(Actor);
			OutComponent = NewObject<T>(Actor, TEXT("Controller"));
			URammsRobotControlSurfaceComponent* Surface = NewObject<URammsRobotControlSurfaceComponent>(Actor, TEXT("ControlSurface"));
			Surface->ExternalHoldSeconds = 0.3f;
			return Surface;
		}
	};

	/** The surface's clock (FPlatformTime) just after a command: the hold it stamped is no later than this. */
	double Stamp()
	{
		return FPlatformTime::Seconds();
	}

	float ValueOf(const URammsRobotControlSurfaceComponent& Surface, FName Id)
	{
		return Surface.GetControlValue(Id);
	}

	ERammsControlSource OwnerOf(const URammsRobotControlSurfaceComponent& Surface, FName Id)
	{
		return Surface.GetAxisOwner_Implementation(Id);
	}
} // namespace RammsDeadmanTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanExpiry,
	"Ramms.Control.Deadman.Expiry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanExpiry::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld						TestWorld;
	FContributor*						C = nullptr;
	URammsRobotControlSurfaceComponent* Surface = TestWorld.SpawnRobot(C);
	if (!TestNotNull(TEXT("a robot"), Surface) || !TestNotNull(TEXT("its contributor"), C))
	{
		return false;
	}
	const FName Rate = FContributor::RateId();
	const FName Trim = FContributor::TrimId();

	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.8f, ERammsControlSource::Remote));
	TestTrue(TEXT("Autonomy sets the trim"), Surface->SetControl(Trim, -0.5f, ERammsControlSource::Autonomy));
	const double Set = Stamp();
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
	TestTrue(TEXT("Remote sets the rate again"), Surface->SetControl(Rate, 0.4f, ERammsControlSource::Remote));
	TestTrue(TEXT("...and releases it"), Surface->ReleaseControl(Rate, ERammsControlSource::Remote));
	TestEqual(TEXT("released: not watched"), Surface->GetDeadmanWatchCount(), 0);
	TestEqual(TEXT("...and not released again later"), Surface->ExpireQuietHolds(Stamp() + 5.0), 0);
	TestEqual(TEXT("...two releases in all"), C->ReleasesOf(Rate), 2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanRefresh,
	"Ramms.Control.Deadman.Refresh",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanRefresh::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld						TestWorld;
	FContributor*						C = nullptr;
	URammsRobotControlSurfaceComponent* Surface = TestWorld.SpawnRobot(C);
	if (!TestNotNull(TEXT("a robot"), Surface) || !TestNotNull(TEXT("its contributor"), C))
	{
		return false;
	}
	const FName Rate = FContributor::RateId();
	const FName Trim = FContributor::TrimId();

	// Both set together; then, 20 ms on, the rate is set again and the trim is not.
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.6f, ERammsControlSource::Remote));
	TestTrue(TEXT("Remote sets the trim"), Surface->SetControl(Trim, 0.9f, ERammsControlSource::Remote));
	const double First = Stamp();
	while (FPlatformTime::Seconds() < First + 0.02)
	{
		FPlatformProcess::Sleep(0.001f);
	}
	TestTrue(TEXT("Remote sets the rate again"), Surface->SetControl(Rate, 0.7f, ERammsControlSource::Remote));
	const double Again = Stamp();

	// 0.31 s after the first commands, under 0.3 s after the second: only the trim has gone quiet.
	TestEqual(TEXT("0.31 s after the first set: one sprung back"), Surface->ExpireQuietHolds(First + 0.31), 1);
	TestEqual(TEXT("the refreshed rate keeps its command"), ValueOf(*Surface, Rate), 0.7f);
	TestEqual(TEXT("...not released"), C->ReleasesOf(Rate), 0);
	TestEqual(TEXT("...still held by Remote"), OwnerOf(*Surface, Rate), ERammsControlSource::Remote);
	TestEqual(TEXT("the trim, set once, sprang back"), ValueOf(*Surface, Trim), FContributor::TrimDefault);
	TestEqual(TEXT("still watching the rate"), Surface->GetDeadmanWatchCount(), 1);

	// A long frame is not silence: a command heard before it is given that frame to be heard again.
	TestEqual(TEXT("a 0.5 s frame after the refresh: not sprung back"), Surface->ExpireQuietHolds(Again + 0.5, /*Grace=*/0.5), 0);
	TestEqual(TEXT("...the rate as set"), ValueOf(*Surface, Rate), 0.7f);
	TestEqual(TEXT("the next frame, still unheard: sprung back"), Surface->ExpireQuietHolds(Again + 0.52, /*Grace=*/0.02), 1);
	TestEqual(TEXT("...to 0"), ValueOf(*Surface, Rate), 0.0f);
	TestEqual(TEXT("...released once"), C->ReleasesOf(Rate), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanOtherSources,
	"Ramms.Control.Deadman.OtherSources",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanOtherSources::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld						TestWorld;
	FContributor*						C = nullptr;
	URammsRobotControlSurfaceComponent* Surface = TestWorld.SpawnRobot(C);
	if (!TestNotNull(TEXT("a robot"), Surface) || !TestNotNull(TEXT("its contributor"), C))
	{
		return false;
	}
	const FName Rate = FContributor::RateId();

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
		TestFalse(What + TEXT(": not a deadman source"), URammsRobotControlSurfaceComponent::IsDeadmanSource(Source));
		TestTrue(What + TEXT(" sets the rate"), Surface->SetControl(Rate, 0.5f, Source));
		TestEqual(What + TEXT(": not watched"), Surface->GetDeadmanWatchCount(), 0);
		TestEqual(What + TEXT(": 5 s on, nothing sprung back"), Surface->ExpireQuietHolds(Stamp() + 5.0), 0);
		TestEqual(What + TEXT(": the command stands"), ValueOf(*Surface, Rate), 0.5f);
		TestTrue(What + TEXT(" lets go itself"), Surface->ReleaseControl(Rate, Source));
	}
	TestEqual(TEXT("released only by the sources themselves"), C->ReleasesOf(Rate), 4);
	TestTrue(TEXT("Remote is a deadman source"), URammsRobotControlSurfaceComponent::IsDeadmanSource(ERammsControlSource::Remote));
	TestTrue(TEXT("Autonomy is a deadman source"), URammsRobotControlSurfaceComponent::IsDeadmanSource(ERammsControlSource::Autonomy));

	// Taken over: once a Remote command's hold has lapsed, a local source may drive, and its command is not the
	// deadman's to spring back. (No hold window here, so the keyboard does not have to wait for it.)
	Surface->ExternalHoldSeconds = 0.0f;
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(Rate, 0.9f, ERammsControlSource::Remote));
	TestEqual(TEXT("...watched"), Surface->GetDeadmanWatchCount(), 1);
	TestTrue(TEXT("the keyboard takes it over"), Surface->SetControl(Rate, -0.3f, ERammsControlSource::Keyboard));
	TestEqual(TEXT("...no longer watched"), Surface->GetDeadmanWatchCount(), 0);
	TestEqual(TEXT("...and not sprung back"), Surface->ExpireQuietHolds(Stamp() + 5.0), 0);
	TestEqual(TEXT("...the keyboard's command stands"), ValueOf(*Surface, Rate), -0.3f);
	TestEqual(TEXT("...held by the keyboard"), OwnerOf(*Surface, Rate), ERammsControlSource::Keyboard);
	TestEqual(TEXT("...no release behind its back"), C->ReleasesOf(Rate), 4);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsControlDeadmanPosition,
	"Ramms.Control.Deadman.PositionUnaffected",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRammsControlDeadmanPosition::RunTest(const FString& Parameters)
{
	using namespace RammsDeadmanTests;
	FDeadmanWorld						TestWorld;
	FContributor*						C = nullptr;
	URammsRobotControlSurfaceComponent* Surface = TestWorld.SpawnRobot(C);
	if (!TestNotNull(TEXT("a robot"), Surface) || !TestNotNull(TEXT("its contributor"), C))
	{
		return false;
	}
	const FName Position = FContributor::PositionId();

	// A position target is where the robot is asked to be, not a command that must keep coming.
	for (const ERammsControlSource Source : { ERammsControlSource::Remote, ERammsControlSource::Autonomy })
	{
		TestTrue(TEXT("a deadman source sets the position"), Surface->SetControl(Position, 40.0f, Source));
		TestEqual(TEXT("...not watched"), Surface->GetDeadmanWatchCount(), 0);
		TestEqual(TEXT("...5 s on, nothing sprung back"), Surface->ExpireQuietHolds(Stamp() + 5.0), 0);
		TestEqual(TEXT("...the target held"), ValueOf(*Surface, Position), 40.0f);
		float Target = 0.0f;
		TestTrue(TEXT("...and reported"), Surface->GetControlTarget(Position, Target) && Target == 40.0f);
		TestEqual(TEXT("...never released"), C->ReleasesOf(Position), 0);
	}

	// A Remote rate beside it springs back on its own: the position is left where it is.
	TestTrue(TEXT("Remote sets the rate"), Surface->SetControl(FContributor::RateId(), 1.0f, ERammsControlSource::Remote));
	TestEqual(TEXT("the rate sprang back"), Surface->ExpireQuietHolds(Stamp() + 5.0), 1);
	TestEqual(TEXT("...the position still held"), ValueOf(*Surface, Position), 40.0f);
	TestEqual(TEXT("...and still not released"), C->ReleasesOf(Position), 0);
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
	URammsDifferentialDriveController*	Diff = nullptr;
	URammsRobotControlSurfaceComponent* DiffSurface = TestWorld.SpawnRobot(Diff);
	if (!TestNotNull(TEXT("a differential robot"), DiffSurface) || !TestNotNull(TEXT("its drive"), Diff))
	{
		return false;
	}
	TestTrue(TEXT("Remote drives forward"), DiffSurface->SetControl(Forward, 0.6f, ERammsControlSource::Remote));
	TestTrue(TEXT("...and turns"), DiffSurface->SetControl(Turn, -0.4f, ERammsControlSource::Remote));
	const double DiffSet = Stamp();
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
	TestEqual(TEXT("...not sprung back"), DiffSurface->ExpireQuietHolds(Stamp() + 5.0), 0);
	TestEqual(TEXT("...still driving"), Diff->DriveInput.Y, 0.5, 1e-6);

	// The holonomic drive keeps its command until told otherwise too.
	URammsHolonomicDriveController*		Holo = nullptr;
	URammsRobotControlSurfaceComponent* HoloSurface = TestWorld.SpawnRobot(Holo);
	if (!TestNotNull(TEXT("a holonomic robot"), HoloSurface) || !TestNotNull(TEXT("its drive"), Holo))
	{
		return false;
	}
	TestTrue(TEXT("Autonomy drives forward"), HoloSurface->SetControl(Forward, 0.3f, ERammsControlSource::Autonomy));
	TestTrue(TEXT("...strafes"), HoloSurface->SetControl(Strafe, -0.2f, ERammsControlSource::Autonomy));
	TestTrue(TEXT("...and turns"), HoloSurface->SetControl(Turn, 0.1f, ERammsControlSource::Autonomy));
	const double HoloSet = Stamp();
	TestEqual(TEXT("the command held (strafe)"), ValueOf(*HoloSurface, Strafe), -0.2f);
	TestEqual(TEXT("quiet: all three sprung back"), HoloSurface->ExpireQuietHolds(HoloSet + 0.31), 3);
	TestEqual(TEXT("...forward 0"), ValueOf(*HoloSurface, Forward), 0.0f);
	TestEqual(TEXT("...strafe 0"), ValueOf(*HoloSurface, Strafe), 0.0f);
	TestEqual(TEXT("...turn 0"), ValueOf(*HoloSurface, Turn), 0.0f);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
