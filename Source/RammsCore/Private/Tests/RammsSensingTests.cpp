// Tests for the sensor rig, the sensing bridge and the camera stream conventions.
//
// The rig is checked on a bare game world: one actor with a root and a
// "chassis" component, mounts built at BeginPlay or added at runtime, and the
// bridge's JSON parsed back and checked for the keys ramms-tools reads. The
// stream conventions (stream ids, channels, frame metadata) are pure functions
// of RammsStreaming and are checked without a world.

#include "CoreMinimal.h"
#include "CameraCaptureSegmentation.h"
#include "CameraCaptureSubsystem.h"
#include "Components/SceneComponent.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "IntrinsicSceneCaptureComponent2D.h"
#include "Misc/AutomationTest.h"
#include "RammsCameraStreams.h"
#include "RammsIMUSensorComponent.h"
#include "RammsSensingBridge.h"
#include "RammsSensorRigComponent.h"
#include "RammsSonarSensorComponent.h"
#include "RammsStreamSourceComponent.h"
#include "RammsToFSensorComponent.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace RammsSensingTests
{
	/** A bare game world, torn down with everything spawned in it. */
	struct FTestWorld
	{
		UWorld*			World = nullptr;
		TArray<AActor*> Spawned;

		FTestWorld()
		{
			World = UWorld::CreateWorld(EWorldType::Game, /*bInformEngineOfWorld=*/false);
			if (World != nullptr)
			{
				GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
			}
		}

		~FTestWorld()
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

		/** An actor with a "Root" scene component and a "chassis" child 10 cm forward, 20 cm up. Not begun play. */
		AActor* SpawnRobot(USceneComponent** OutChassis = nullptr)
		{
			if (World == nullptr)
			{
				return nullptr;
			}
			AActor* Actor = World->SpawnActor<AActor>();
			if (Actor == nullptr)
			{
				return nullptr;
			}
			Spawned.Add(Actor);
			USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("Root"));
			Actor->SetRootComponent(Root);
			Root->RegisterComponent();
			USceneComponent* Chassis = NewObject<USceneComponent>(Actor, TEXT("chassis"));
			Chassis->SetupAttachment(Root);
			Chassis->SetRelativeLocation(FVector(10.0, 0.0, 20.0));
			Chassis->RegisterComponent();
			if (OutChassis)
			{
				*OutChassis = Chassis;
			}
			return Actor;
		}

		/** A rig with these mounts on Actor, registered (it builds at the actor's BeginPlay). */
		static URammsSensorRigComponent* AddRig(AActor* Actor, const TArray<FRammsSensorMount>& InMounts)
		{
			URammsSensorRigComponent* Rig = NewObject<URammsSensorRigComponent>(Actor, TEXT("SensorRig"));
			Rig->Mounts = InMounts;
			Rig->RegisterComponent();
			return Rig;
		}
	};

	FRammsSensorMount MakeMount(const TCHAR* Name, ERammsSensorKind Kind, const TCHAR* Parent = nullptr)
	{
		FRammsSensorMount Mount;
		Mount.Name = FName(Name);
		Mount.Kind = Kind;
		Mount.ParentComponent = Parent ? FName(Parent) : FName(NAME_None);
		return Mount;
	}

	/** Sensors log a warning when GPU tracing is unavailable, which a test world may well be; that is not what is
	 *  under test. Neither is the capture subsystem's material loading. */
	void IgnoreEnvironmentWarnings(FAutomationTestBase& Test)
	{
		Test.AddExpectedMessagePlain(TEXT("GPU ray tracing requested but not available"), ELogVerbosity::Warning,
			EAutomationExpectedMessageFlags::Contains, -1);
		Test.AddExpectedMessagePlain(TEXT("[CameraCaptureSubsystem]"), ELogVerbosity::Warning,
			EAutomationExpectedMessageFlags::Contains, -1);
	}

	TSharedPtr<FJsonObject> ParseJson(const FString& Json)
	{
		TSharedPtr<FJsonObject>			Obj;
		const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
		if (!FJsonSerializer::Deserialize(Reader, Obj))
		{
			return nullptr;
		}
		return Obj;
	}

	bool HasKeys(FAutomationTestBase& Test, const FString& What, const TSharedPtr<FJsonObject>& Obj, const TArray<FString>& Keys)
	{
		if (!Obj.IsValid())
		{
			Test.AddError(What + TEXT(": not a JSON object"));
			return false;
		}
		bool bAll = true;
		for (const FString& Key : Keys)
		{
			if (!Obj->HasField(Key))
			{
				Test.AddError(FString::Printf(TEXT("%s: missing \"%s\""), *What, *Key));
				bAll = false;
			}
		}
		return bAll;
	}

	/** The entry of a ListSensorsJson result named Name, or null. */
	TSharedPtr<FJsonObject> FindSensor(const TSharedPtr<FJsonObject>& List, const FString& Name)
	{
		const TArray<TSharedPtr<FJsonValue>>* Sensors = nullptr;
		if (!List.IsValid() || !List->TryGetArrayField(TEXT("sensors"), Sensors) || !Sensors)
		{
			return nullptr;
		}
		for (const TSharedPtr<FJsonValue>& Value : *Sensors)
		{
			const TSharedPtr<FJsonObject>* Entry = nullptr;
			if (Value.IsValid() && Value->TryGetObject(Entry) && Entry && (*Entry)->GetStringField(TEXT("name")) == Name)
			{
				return *Entry;
			}
		}
		return nullptr;
	}

	bool IsOk(const TSharedPtr<FJsonObject>& Obj)
	{
		bool bOk = false;
		return Obj.IsValid() && Obj->TryGetBoolField(TEXT("ok"), bOk) && bOk;
	}
} // namespace RammsSensingTests

// ---------------------------------------------------------------------------
// The rig
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsSensingRigBuildsMounts, "Ramms.Sensing.Rig.BuildsMounts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsSensingRigBuildsMounts::RunTest(const FString& Parameters)
{
	using namespace RammsSensingTests;
	IgnoreEnvironmentWarnings(*this);
	AddExpectedMessagePlain(TEXT("has no component 'nope'"), ELogVerbosity::Warning, EAutomationExpectedMessageFlags::Contains, 1);

	FTestWorld		 TestWorld;
	USceneComponent* Chassis = nullptr;
	AActor*			 Robot = TestWorld.SpawnRobot(&Chassis);
	if (!TestNotNull(TEXT("robot"), Robot))
	{
		return false;
	}

	FRammsSensorMount ToF = MakeMount(TEXT("front_tof"), ERammsSensorKind::ToF, TEXT("chassis"));
	ToF.Location = FVector(5.0, 0.0, 0.0);
	ToF.Rotation = FRotator(-10.0, 30.0, 0.0);
	ToF.Rows = 4;
	ToF.Columns = 6;
	ToF.HorizontalFovDeg = 60.0f;
	ToF.VerticalFovDeg = 40.0f;
	ToF.MaxRangeCm = 250.0f;

	FRammsSensorMount Sonar = MakeMount(TEXT("rear_sonar"), ERammsSensorKind::Sonar); // root
	Sonar.Location = FVector(-30.0, 0.0, 5.0);
	Sonar.Rotation = FRotator(0.0, 180.0, 0.0);
	Sonar.SonarMaxRangeCm = 320.0f;
	Sonar.SonarConeDeg = 12.0f;

	FRammsSensorMount Imu = MakeMount(TEXT("imu"), ERammsSensorKind::IMU, TEXT("chassis"));

	FRammsSensorMount Camera = MakeMount(TEXT("front_cam"), ERammsSensorKind::Camera, TEXT("chassis"));
	Camera.Location = FVector(20.0, 0.0, 30.0);
	Camera.Width = 320;
	Camera.Height = 240;
	Camera.FovDeg = 90.0f;
	Camera.bSegmentation = false;

	FRammsSensorMount Ghost = MakeMount(TEXT("ghost"), ERammsSensorKind::ToF, TEXT("nope"));

	URammsSensorRigComponent* Rig = FTestWorld::AddRig(Robot, { ToF, Sonar, Imu, Camera, Ghost });
	Robot->DispatchBeginPlay();
	TestTrue(TEXT("the rig began play"), Rig->HasBegunPlay());

	// ToF: the component class, the parent, the relative transform, the grid.
	const URammsToFSensorComponent* ToFComp = Cast<URammsToFSensorComponent>(Rig->GetSensorComponent(TEXT("front_tof")));
	if (TestNotNull(TEXT("ToF built as URammsToFSensorComponent"), ToFComp))
	{
		TestEqual(TEXT("ToF component named after the mount"), ToFComp->GetFName(), FName(TEXT("front_tof")));
		TestTrue(TEXT("ToF attached to chassis"), ToFComp->GetAttachParent() == Chassis);
		TestTrue(TEXT("ToF at its relative location"), ToFComp->GetRelativeLocation().Equals(ToF.Location, 0.01));
		TestTrue(TEXT("ToF at its relative rotation"), ToFComp->GetRelativeRotation().Equals(ToF.Rotation, 0.01));
		TestTrue(TEXT("ToF in grid mode"), ToFComp->SensorMode == EToFSensorMode::Grid);
		TestEqual(TEXT("ToF rows"), ToFComp->GridRows, 4);
		TestEqual(TEXT("ToF columns"), ToFComp->GridColumns, 6);
		TestEqual(TEXT("ToF h fov"), ToFComp->HorizontalFOV, 60.0f);
		TestEqual(TEXT("ToF v fov"), ToFComp->VerticalFOV, 40.0f);
		TestEqual(TEXT("ToF range"), ToFComp->MaxRange, 250.0f);
		TestTrue(TEXT("ToF began play"), ToFComp->HasBegunPlay());
	}

	// Sonar: no parent named = the root.
	const URammsSonarSensorComponent* SonarComp = Cast<URammsSonarSensorComponent>(Rig->GetSensorComponent(TEXT("rear_sonar")));
	if (TestNotNull(TEXT("Sonar built as URammsSonarSensorComponent"), SonarComp))
	{
		TestTrue(TEXT("Sonar attached to the root"), SonarComp->GetAttachParent() == Robot->GetRootComponent());
		TestTrue(TEXT("Sonar at its relative location"), SonarComp->GetRelativeLocation().Equals(Sonar.Location, 0.01));
		TestEqual(TEXT("Sonar range"), SonarComp->MaxRange, 320.0f);
		TestEqual(TEXT("Sonar cone half-angle"), SonarComp->BeamHalfAngle, 12.0f);
	}

	TestNotNull(TEXT("IMU built as URammsIMUSensorComponent"), Cast<URammsIMUSensorComponent>(Rig->GetSensorComponent(TEXT("imu"))));

	// Camera: a capture component with the mount's intrinsics, registered for capture, and its own stream source.
	UIntrinsicSceneCaptureComponent2D* CameraComp = Cast<UIntrinsicSceneCaptureComponent2D>(Rig->GetSensorComponent(TEXT("front_cam")));
	if (TestNotNull(TEXT("camera built as UIntrinsicSceneCaptureComponent2D"), CameraComp))
	{
		TestTrue(TEXT("camera attached to chassis"), CameraComp->GetAttachParent() == Chassis);
		TestTrue(TEXT("camera at its relative location"), CameraComp->GetRelativeLocation().Equals(Camera.Location, 0.01));
		TestTrue(TEXT("camera uses inline intrinsics"), CameraComp->bUseCustomIntrinsics && !CameraComp->bUseIntrinsicsAsset);
		const FCameraIntrinsics Intrinsics = CameraComp->GetActiveIntrinsics();
		TestEqual(TEXT("camera width"), Intrinsics.ImageWidth, 320);
		TestEqual(TEXT("camera height"), Intrinsics.ImageHeight, 240);
		// 90 degrees across 320 px: f = 160 / tan(45) = 160.
		TestTrue(TEXT("camera focal length from the fov"), FMath::IsNearlyEqual(Intrinsics.FocalLengthX, 160.0f, 0.01f));
		TestTrue(TEXT("camera principal point centred"),
			FMath::IsNearlyEqual(Intrinsics.PrincipalPointX, 160.0f) && FMath::IsNearlyEqual(Intrinsics.PrincipalPointY, 120.0f));

		const UCameraCaptureSubsystem* Capture = TestWorld.World->GetSubsystem<UCameraCaptureSubsystem>();
		if (TestNotNull(TEXT("capture subsystem"), Capture))
		{
			TestTrue(TEXT("camera registered for capture"), Capture->GetRegisteredCameras().Contains(CameraComp));
		}
	}
	const URammsStreamSourceComponent* Source = Cast<URammsStreamSourceComponent>(Rig->GetStreamSource(TEXT("front_cam")));
	if (TestNotNull(TEXT("camera stream source"), Source))
	{
		TestEqual(TEXT("source named <mount>_stream"), Source->GetFName(), FName(TEXT("front_cam_stream")));
		TestEqual(TEXT("source filters on the camera"), Source->CameraFilter, FString(TEXT("front_cam")));
		TestTrue(TEXT("source filter is exact"), Source->bExactCameraFilter);
		TestEqual(TEXT("source stream name"), Source->StreamName, FString(TEXT("front_cam")));
		TestEqual(TEXT("first camera gets channel 0"), Source->ChannelID, 0);
		TestTrue(TEXT("colour, depth, motion on; mask off; no RGBD"),
			Source->bStreamRGB && Source->bStreamDepth && Source->bStreamMotion && !Source->bStreamSegmentation && !Source->bStreamRGBD);
		TestTrue(TEXT("source began play"), Source->HasBegunPlay());
	}

	// The mount with a missing parent: skipped, nothing built under its name.
	TestNull(TEXT("missing parent: not built"), Rig->GetSensorComponent(TEXT("ghost")));
	TestFalse(TEXT("missing parent: not in the built list"), Rig->GetBuiltMountNames().Contains(FName(TEXT("ghost"))));
	TestEqual(TEXT("four mounts built"), Rig->GetBuiltMountNames().Num(), 4);
	TestEqual(TEXT("the description keeps all five"), Rig->Mounts.Num(), 5);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsSensingRigAddRemove, "Ramms.Sensing.Rig.AddAndRemoveMounts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsSensingRigAddRemove::RunTest(const FString& Parameters)
{
	using namespace RammsSensingTests;
	IgnoreEnvironmentWarnings(*this);

	FTestWorld TestWorld;
	AActor*	   Robot = TestWorld.SpawnRobot();
	if (!TestNotNull(TEXT("robot"), Robot))
	{
		return false;
	}
	URammsSensorRigComponent* Rig = FTestWorld::AddRig(Robot, {});
	Robot->DispatchBeginPlay();

	// Added at runtime: built at once.
	FRammsSensorMount First = MakeMount(TEXT("probe"), ERammsSensorKind::ToF, TEXT("chassis"));
	First.Rows = 8;
	First.Columns = 8;
	TestTrue(TEXT("AddMount builds"), Rig->AddMount(First));
	USceneComponent* FirstComp = Rig->GetSensorComponent(TEXT("probe"));
	TestNotNull(TEXT("built at runtime"), Cast<URammsToFSensorComponent>(FirstComp));

	// Same name again: replaced, not duplicated -- even as another kind.
	FRammsSensorMount Second = MakeMount(TEXT("probe"), ERammsSensorKind::Sonar);
	Second.Location = FVector(1.0, 2.0, 3.0);
	TestTrue(TEXT("AddMount replaces"), Rig->AddMount(Second));
	USceneComponent* SecondComp = Rig->GetSensorComponent(TEXT("probe"));
	TestNotNull(TEXT("replacement is a sonar"), Cast<URammsSonarSensorComponent>(SecondComp));
	TestTrue(TEXT("the old component is gone"), !IsValid(FirstComp));
	TestEqual(TEXT("one mount of that name"), Rig->Mounts.Num(), 1);
	TestEqual(TEXT("one built of that name"), Rig->GetBuiltMountNames().Num(), 1);
	TestTrue(TEXT("the replacement keeps the mount name"), SecondComp && SecondComp->GetFName() == FName(TEXT("probe")));
	TestTrue(TEXT("the description was replaced"), Rig->FindMount(TEXT("probe")) && Rig->FindMount(TEXT("probe"))->Kind == ERammsSensorKind::Sonar);

	// A camera replaced keeps one camera registered and one stream source.
	FRammsSensorMount Cam = MakeMount(TEXT("cam"), ERammsSensorKind::Camera);
	TestTrue(TEXT("camera added"), Rig->AddMount(Cam));
	UActorComponent* OldSource = Rig->GetStreamSource(TEXT("cam"));
	Cam.Width = 160;
	Cam.Height = 120;
	TestTrue(TEXT("camera replaced"), Rig->AddMount(Cam));
	TestTrue(TEXT("old stream source gone"), !IsValid(OldSource));
	const UCameraCaptureSubsystem* Capture = TestWorld.World->GetSubsystem<UCameraCaptureSubsystem>();
	if (TestNotNull(TEXT("capture subsystem"), Capture))
	{
		TestEqual(TEXT("one camera registered"), Capture->GetRegisteredCameraCount(), 1);
	}

	// Removed: the component and the description go.
	TestTrue(TEXT("RemoveMount"), Rig->RemoveMount(TEXT("probe")));
	TestNull(TEXT("removed: no component"), Rig->GetSensorComponent(TEXT("probe")));
	TestTrue(TEXT("removed: component destroyed"), !IsValid(SecondComp));
	TestNull(TEXT("removed: no description"), Rig->FindMount(TEXT("probe")));
	TestFalse(TEXT("RemoveMount of an unknown name"), Rig->RemoveMount(TEXT("probe")));

	UActorComponent* CamSource = Rig->GetStreamSource(TEXT("cam"));
	TestTrue(TEXT("RemoveMount camera"), Rig->RemoveMount(TEXT("cam")));
	TestTrue(TEXT("camera source destroyed"), !IsValid(CamSource));
	if (Capture)
	{
		TestEqual(TEXT("camera unregistered"), Capture->GetRegisteredCameraCount(), 0);
	}

	// The name is free again at once (no garbage collection in between).
	TestTrue(TEXT("re-add after remove"), Rig->AddMount(MakeMount(TEXT("probe"), ERammsSensorKind::IMU)));
	TestNotNull(TEXT("re-added"), Cast<URammsIMUSensorComponent>(Rig->GetSensorComponent(TEXT("probe"))));

	// A rig that has not begun play only records the mount.
	AActor*					  Idle = TestWorld.SpawnRobot();
	URammsSensorRigComponent* IdleRig = FTestWorld::AddRig(Idle, {});
	TestTrue(TEXT("before play: recorded"), IdleRig->AddMount(MakeMount(TEXT("later"), ERammsSensorKind::IMU)));
	TestNull(TEXT("before play: not built"), IdleRig->GetSensorComponent(TEXT("later")));
	Idle->DispatchBeginPlay();
	TestNotNull(TEXT("built at BeginPlay"), IdleRig->GetSensorComponent(TEXT("later")));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsSensingRigChannels, "Ramms.Sensing.Rig.StreamChannels",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsSensingRigChannels::RunTest(const FString& Parameters)
{
	using namespace RammsSensingTests;
	IgnoreEnvironmentWarnings(*this);

	FTestWorld TestWorld;
	AActor*	   Robot = TestWorld.SpawnRobot();
	if (!TestNotNull(TEXT("robot"), Robot))
	{
		return false;
	}

	FRammsSensorMount Pinned = MakeMount(TEXT("cam_c"), ERammsSensorKind::Camera);
	Pinned.StreamChannel = 7;
	URammsSensorRigComponent* Rig = FTestWorld::AddRig(Robot,
		{ MakeMount(TEXT("cam_a"), ERammsSensorKind::Camera), MakeMount(TEXT("cam_b"), ERammsSensorKind::Camera), Pinned,
			MakeMount(TEXT("cam_d"), ERammsSensorKind::Camera) });
	Robot->DispatchBeginPlay();

	TestEqual(TEXT("cam_a: first free"), Rig->GetStreamChannel(TEXT("cam_a")), 0);
	TestEqual(TEXT("cam_b: next free"), Rig->GetStreamChannel(TEXT("cam_b")), 1);
	TestEqual(TEXT("cam_c: as asked"), Rig->GetStreamChannel(TEXT("cam_c")), 7);
	TestEqual(TEXT("cam_d: next free"), Rig->GetStreamChannel(TEXT("cam_d")), 2);
	TestEqual(TEXT("non-camera: no channel"), Rig->GetStreamChannel(TEXT("nothing")), -1);

	// Another robot's rig in the same world continues the count.
	AActor*					  Other = TestWorld.SpawnRobot();
	URammsSensorRigComponent* OtherRig = FTestWorld::AddRig(Other, { MakeMount(TEXT("cam_a"), ERammsSensorKind::Camera) });
	Other->DispatchBeginPlay();
	TestEqual(TEXT("second robot: next free in the world"), OtherRig->GetStreamChannel(TEXT("cam_a")), 3);

	// A hand-placed source's channels are taken too.
	URammsStreamSourceComponent* HandPlaced = NewObject<URammsStreamSourceComponent>(Other, TEXT("HandSource"));
	HandPlaced->ChannelID = 4;
	HandPlaced->CameraFilter = TEXT("no_such_camera"); // forwards nothing here, so the stream counts below stay the rigs'
	HandPlaced->RegisterComponent();
	TestEqual(TEXT("free channel skips a hand-placed source"), URammsSensorRigComponent::FindFreeStreamChannel(TestWorld.World), 5);
	TestTrue(TEXT("runtime camera"), OtherRig->AddMount(MakeMount(TEXT("cam_e"), ERammsSensorKind::Camera)));
	TestEqual(TEXT("runtime camera: next free"), OtherRig->GetStreamChannel(TEXT("cam_e")), 5);

	// What the rig says a camera streams: four channels, C + 0/100/200/300, under the agreed stream ids.
	const TSharedPtr<FJsonObject> List = ParseJson(URammsSensingBridge::ListSensorsJson(Robot->GetPathName()));
	TestTrue(TEXT("list ok"), IsOk(List));
	const TSharedPtr<FJsonObject> CamC = FindSensor(List, TEXT("cam_c"));
	if (TestTrue(TEXT("cam_c listed"), CamC.IsValid()))
	{
		const TArray<TSharedPtr<FJsonValue>>* Streams = nullptr;
		TestTrue(TEXT("cam_c has streams"), CamC->TryGetArrayField(TEXT("streams"), Streams) && Streams && Streams->Num() == 4);
		if (Streams && Streams->Num() == 4)
		{
			const TCHAR* Roles[] = { TEXT("color"), TEXT("depth"), TEXT("motion"), TEXT("mask") };
			const TCHAR* Ids[] = { TEXT("camera/cam_c/color"), TEXT("camera/cam_c/depth"), TEXT("motion/cam_c"), TEXT("mask/cam_c") };
			const TCHAR* Messages[] = { TEXT("FrameRGB"), TEXT("FrameDepth"), TEXT("FrameMotion"), TEXT("FrameData") };
			const TCHAR* Formats[] = { TEXT("bgra8"), TEXT("float32"), TEXT("rg32f"), TEXT("mono8") };
			const int32	 Channels[] = { 7, 107, 207, 307 };
			for (int32 i = 0; i < 4; ++i)
			{
				const TSharedPtr<FJsonObject> Stream = (*Streams)[i]->AsObject();
				if (!HasKeys(*this, FString::Printf(TEXT("stream %d"), i), Stream,
						{ TEXT("role"), TEXT("stream_id"), TEXT("channel"), TEXT("message"), TEXT("fmt") }))
				{
					continue;
				}
				TestEqual(FString::Printf(TEXT("stream %d role"), i), Stream->GetStringField(TEXT("role")), FString(Roles[i]));
				TestEqual(FString::Printf(TEXT("stream %d id"), i), Stream->GetStringField(TEXT("stream_id")), FString(Ids[i]));
				TestEqual(FString::Printf(TEXT("stream %d message"), i), Stream->GetStringField(TEXT("message")), FString(Messages[i]));
				TestEqual(FString::Printf(TEXT("stream %d fmt"), i), Stream->GetStringField(TEXT("fmt")), FString(Formats[i]));
				TestEqual(FString::Printf(TEXT("stream %d channel"), i), static_cast<int32>(Stream->GetNumberField(TEXT("channel"))), Channels[i]);
			}
		}
		const TSharedPtr<FJsonObject>* Settings = nullptr;
		if (TestTrue(TEXT("cam_c settings"), CamC->TryGetObjectField(TEXT("settings"), Settings) && Settings))
		{
			HasKeys(*this, TEXT("camera settings"), *Settings,
				{ TEXT("width"), TEXT("height"), TEXT("fov_deg"), TEXT("color"), TEXT("depth"), TEXT("motion"), TEXT("segmentation"), TEXT("channel") });
			TestEqual(TEXT("settings channel"), static_cast<int32>((*Settings)->GetNumberField(TEXT("channel"))), 7);
		}
	}

	// Only the enabled roles are listed.
	FRammsSensorMount ColourOnly = MakeMount(TEXT("cam_rgb"), ERammsSensorKind::Camera);
	ColourOnly.bDepth = ColourOnly.bMotion = ColourOnly.bSegmentation = false;
	Rig->AddMount(ColourOnly);
	const TSharedPtr<FJsonObject>		  RgbEntry = FindSensor(ParseJson(URammsSensingBridge::ListSensorsJson(Robot->GetPathName())), TEXT("cam_rgb"));
	const TArray<TSharedPtr<FJsonValue>>* RgbStreams = nullptr;
	TestTrue(TEXT("colour-only camera lists one stream"),
		RgbEntry.IsValid() && RgbEntry->TryGetArrayField(TEXT("streams"), RgbStreams) && RgbStreams && RgbStreams->Num() == 1);

	// The capture subsystem's helper captures (a camera's "_dmv" motion pass) are not listed as cameras.
	int32								  Cameras = 0;
	const TSharedPtr<FJsonObject>		  All = ParseJson(URammsSensingBridge::ListSensorsJson(Robot->GetPathName()));
	const TArray<TSharedPtr<FJsonValue>>* Sensors = nullptr;
	if (All.IsValid() && All->TryGetArrayField(TEXT("sensors"), Sensors) && Sensors)
	{
		for (const TSharedPtr<FJsonValue>& Value : *Sensors)
		{
			Cameras += Value->AsObject()->GetStringField(TEXT("kind")) == TEXT("Camera") ? 1 : 0;
		}
	}
	TestEqual(TEXT("one listed camera per camera mount"), Cameras, 5);

	// The stream status lists every stream of every registered camera with its actor and camera.
	const TSharedPtr<FJsonObject> Status = ParseJson(URammsSensingBridge::StreamStatusJsonFor(TestWorld.World));
	TestTrue(TEXT("status ok"), IsOk(Status));
	HasKeys(*this, TEXT("status"), Status, { TEXT("server"), TEXT("capture"), TEXT("streams") });
	const TSharedPtr<FJsonObject>* Server = nullptr;
	if (Status.IsValid() && Status->TryGetObjectField(TEXT("server"), Server) && Server)
	{
		HasKeys(*this, TEXT("status.server"), *Server, { TEXT("running"), TEXT("port"), TEXT("connections") });
	}
	const TSharedPtr<FJsonObject>* CaptureJson = nullptr;
	if (Status.IsValid() && Status->TryGetObjectField(TEXT("capture"), CaptureJson) && CaptureJson)
	{
		HasKeys(*this, TEXT("status.capture"), *CaptureJson,
			{ TEXT("capturing"), TEXT("every_n_frames"), TEXT("color"), TEXT("depth"), TEXT("motion"), TEXT("segmentation"),
				TEXT("save_to_disk"), TEXT("cameras") });
		TestEqual(TEXT("status counts the registered cameras"), static_cast<int32>((*CaptureJson)->GetNumberField(TEXT("cameras"))), 7);
	}
	const TArray<TSharedPtr<FJsonValue>>* StatusStreams = nullptr;
	if (Status.IsValid() && Status->TryGetArrayField(TEXT("streams"), StatusStreams) && StatusStreams)
	{
		// 6 four-stream cameras + 1 colour-only.
		TestEqual(TEXT("status streams"), StatusStreams->Num(), 6 * 4 + 1);
		if (StatusStreams->Num() > 0)
		{
			HasKeys(*this, TEXT("status stream"), (*StatusStreams)[0]->AsObject(),
				{ TEXT("actor"), TEXT("camera"), TEXT("role"), TEXT("stream_id"), TEXT("channel"), TEXT("message"), TEXT("fmt") });
		}
	}
	return true;
}

// ---------------------------------------------------------------------------
// The bridge
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsSensingBridgeParseMount, "Ramms.Sensing.Bridge.ParseMount",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsSensingBridgeParseMount::RunTest(const FString& Parameters)
{
	FRammsSensorMount Mount;
	FString			  Error;
	TestTrue(TEXT("a full mount parses"),
		URammsSensingBridge::ParseMountJson(
			TEXT("{\"name\": \"front_tof\", \"kind\": \"TOF\", \"parent\": \"chassis\", \"socket\": \"\", \"location\": [1, 2, 3],")
				TEXT(" \"rotation\": [10, 20, 30], \"rows\": 4, \"columns\": 5, \"h_fov_deg\": 50, \"v_fov_deg\": 40, \"max_range_cm\": 300,")
					TEXT(" \"sonar_max_range_cm\": 450, \"sonar_cone_deg\": 11, \"width\": 320, \"height\": 200, \"fov_deg\": 70,")
						TEXT(" \"color\": false, \"depth\": true, \"motion\": false, \"segmentation\": true, \"channel\": 12}"),
			Mount, Error));
	TestEqual(TEXT("name"), Mount.Name, FName(TEXT("front_tof")));
	TestTrue(TEXT("kind is case-insensitive"), Mount.Kind == ERammsSensorKind::ToF);
	TestEqual(TEXT("parent"), Mount.ParentComponent, FName(TEXT("chassis")));
	TestTrue(TEXT("empty socket is none"), Mount.Socket.IsNone());
	TestTrue(TEXT("location [x, y, z]"), Mount.Location.Equals(FVector(1.0, 2.0, 3.0)));
	TestTrue(TEXT("rotation [pitch, yaw, roll]"), Mount.Rotation.Equals(FRotator(10.0, 20.0, 30.0)));
	TestEqual(TEXT("rows"), Mount.Rows, 4);
	TestEqual(TEXT("columns"), Mount.Columns, 5);
	TestEqual(TEXT("h fov"), Mount.HorizontalFovDeg, 50.0f);
	TestEqual(TEXT("v fov"), Mount.VerticalFovDeg, 40.0f);
	TestEqual(TEXT("range"), Mount.MaxRangeCm, 300.0f);
	TestEqual(TEXT("sonar range"), Mount.SonarMaxRangeCm, 450.0f);
	TestEqual(TEXT("sonar cone"), Mount.SonarConeDeg, 11.0f);
	TestEqual(TEXT("width"), Mount.Width, 320);
	TestEqual(TEXT("height"), Mount.Height, 200);
	TestEqual(TEXT("fov"), Mount.FovDeg, 70.0f);
	TestTrue(TEXT("flags"), !Mount.bColor && Mount.bDepth && !Mount.bMotion && Mount.bSegmentation);
	TestEqual(TEXT("channel"), Mount.StreamChannel, 12);

	// Missing keys keep the struct defaults; objects work for location / rotation.
	FRammsSensorMount Sparse;
	TestTrue(TEXT("a sparse mount parses"),
		URammsSensingBridge::ParseMountJson(
			TEXT("{\"name\": \"cam\", \"kind\": \"camera\", \"location\": {\"x\": 4, \"y\": 5, \"z\": 6}, \"rotation\": {\"yaw\": 90}}"),
			Sparse, Error));
	const FRammsSensorMount Defaults;
	TestTrue(TEXT("camera kind"), Sparse.Kind == ERammsSensorKind::Camera);
	TestTrue(TEXT("location object"), Sparse.Location.Equals(FVector(4.0, 5.0, 6.0)));
	TestTrue(TEXT("rotation object"), Sparse.Rotation.Equals(FRotator(0.0, 90.0, 0.0)));
	TestTrue(TEXT("parent defaults to the root"), Sparse.ParentComponent.IsNone());
	TestEqual(TEXT("default width"), Sparse.Width, Defaults.Width);
	TestEqual(TEXT("default fov"), Sparse.FovDeg, Defaults.FovDeg);
	TestEqual(TEXT("default channel"), Sparse.StreamChannel, -1);
	TestTrue(TEXT("default flags"), Sparse.bColor && Sparse.bDepth && Sparse.bMotion && Sparse.bSegmentation);

	FRammsSensorMount Bad;
	TestFalse(TEXT("no name"), URammsSensingBridge::ParseMountJson(TEXT("{\"kind\": \"imu\"}"), Bad, Error));
	TestFalse(TEXT("unknown kind"), URammsSensingBridge::ParseMountJson(TEXT("{\"name\": \"x\", \"kind\": \"lidar\"}"), Bad, Error));
	TestTrue(TEXT("unknown kind says so"), Error.Contains(TEXT("lidar")));
	TestFalse(TEXT("not JSON"), URammsSensingBridge::ParseMountJson(TEXT("front_tof"), Bad, Error));
	TestFalse(TEXT("bad location"), URammsSensingBridge::ParseMountJson(TEXT("{\"name\": \"x\", \"location\": \"here\"}"), Bad, Error));

	ERammsSensorKind Kind = ERammsSensorKind::ToF;
	TestTrue(TEXT("Sonar"), URammsSensorRigComponent::KindFromString(TEXT("Sonar"), Kind) && Kind == ERammsSensorKind::Sonar);
	TestTrue(TEXT("imu"), URammsSensorRigComponent::KindFromString(TEXT("imu"), Kind) && Kind == ERammsSensorKind::IMU);
	TestEqual(TEXT("kind names"), URammsSensorRigComponent::KindToString(ERammsSensorKind::ToF), FString(TEXT("ToF")));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsSensingBridgeJson, "Ramms.Sensing.Bridge.JsonShapes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsSensingBridgeJson::RunTest(const FString& Parameters)
{
	using namespace RammsSensingTests;
	IgnoreEnvironmentWarnings(*this);

	FTestWorld TestWorld;
	AActor*	   Robot = TestWorld.SpawnRobot();
	if (!TestNotNull(TEXT("robot"), Robot))
	{
		return false;
	}
	// A ToF placed by hand, not by a rig.
	URammsToFSensorComponent* HandToF = NewObject<URammsToFSensorComponent>(Robot, TEXT("HandToF"));
	HandToF->SetupAttachment(Robot->GetRootComponent());
	HandToF->RegisterComponent();
	Robot->DispatchBeginPlay();
	const FString Path = Robot->GetPathName();

	// Add: the actor has no rig, so one is added.
	TestNull(TEXT("no rig yet"), Robot->FindComponentByClass<URammsSensorRigComponent>());
	const TSharedPtr<FJsonObject> Added = ParseJson(URammsSensingBridge::AddSensorJson(Path,
		TEXT("{\"name\": \"front_tof\", \"kind\": \"tof\", \"parent\": \"chassis\", \"location\": [5, 0, 0], \"rows\": 2, \"columns\": 3}")));
	TestTrue(TEXT("add ok"), IsOk(Added));
	HasKeys(*this, TEXT("add"), Added, { TEXT("ok"), TEXT("name"), TEXT("component_path") });
	URammsSensorRigComponent* Rig = Robot->FindComponentByClass<URammsSensorRigComponent>();
	if (!TestNotNull(TEXT("a rig was added"), Rig))
	{
		return false;
	}
	if (Added.IsValid())
	{
		TestEqual(TEXT("add name"), Added->GetStringField(TEXT("name")), FString(TEXT("front_tof")));
		const USceneComponent* Built = Rig->GetSensorComponent(TEXT("front_tof"));
		TestTrue(TEXT("add component_path is the built component"),
			Built && Added->GetStringField(TEXT("component_path")) == Built->GetPathName());
	}
	TestTrue(TEXT("add sonar"), IsOk(ParseJson(URammsSensingBridge::AddSensorJson(Path, TEXT("{\"name\": \"rear_sonar\", \"kind\": \"Sonar\", \"rotation\": [0, 180, 0], \"sonar_cone_deg\": 9}")))));
	TestTrue(TEXT("add imu"), IsOk(ParseJson(URammsSensingBridge::AddSensorJson(Path, TEXT("{\"name\": \"imu\", \"kind\": \"IMU\"}")))));
	const TSharedPtr<FJsonObject> AddedCam = ParseJson(URammsSensingBridge::AddSensorJson(Path,
		TEXT("{\"name\": \"cam\", \"kind\": \"camera\", \"parent\": \"chassis\", \"width\": 64, \"height\": 48}")));
	TestTrue(TEXT("add camera"), IsOk(AddedCam));
	HasKeys(*this, TEXT("add camera"), AddedCam, { TEXT("channel"), TEXT("streams") });

	// Failures say why.
	const TSharedPtr<FJsonObject> NoParent = ParseJson(URammsSensingBridge::AddSensorJson(Path, TEXT("{\"name\": \"x\", \"kind\": \"tof\", \"parent\": \"nope\"}")));
	TestFalse(TEXT("missing parent fails"), IsOk(NoParent));
	HasKeys(*this, TEXT("missing parent"), NoParent, { TEXT("ok"), TEXT("error") });
	TestFalse(TEXT("bad kind fails"), IsOk(ParseJson(URammsSensingBridge::AddSensorJson(Path, TEXT("{\"name\": \"x\", \"kind\": \"radar\"}")))));
	TestFalse(TEXT("taken name fails"), IsOk(ParseJson(URammsSensingBridge::AddSensorJson(Path, TEXT("{\"name\": \"HandToF\", \"kind\": \"imu\"}")))));
	TestFalse(TEXT("unknown actor fails"), IsOk(ParseJson(URammsSensingBridge::AddSensorJson(TEXT("NoSuchActor_Sensing"), TEXT("{\"name\": \"x\"}")))));

	// List: every sensor, with the agreed keys.
	const TSharedPtr<FJsonObject> List = ParseJson(URammsSensingBridge::ListSensorsJson(Path));
	TestTrue(TEXT("list ok"), IsOk(List));
	const TSharedPtr<FJsonObject> ToFEntry = FindSensor(List, TEXT("front_tof"));
	if (HasKeys(*this, TEXT("ToF entry"), ToFEntry,
			{ TEXT("actor"), TEXT("actor_name"), TEXT("name"), TEXT("kind"), TEXT("component_path"), TEXT("parent"), TEXT("socket"),
				TEXT("rig"), TEXT("location"), TEXT("rotation"), TEXT("settings"), TEXT("streams") }))
	{
		TestEqual(TEXT("ToF actor"), ToFEntry->GetStringField(TEXT("actor")), Path);
		TestEqual(TEXT("ToF actor_name"), ToFEntry->GetStringField(TEXT("actor_name")), Robot->GetName());
		TestEqual(TEXT("ToF kind"), ToFEntry->GetStringField(TEXT("kind")), FString(TEXT("ToF")));
		TestEqual(TEXT("ToF parent"), ToFEntry->GetStringField(TEXT("parent")), FString(TEXT("chassis")));
		TestTrue(TEXT("ToF rig"), ToFEntry->GetBoolField(TEXT("rig")));
		const TSharedPtr<FJsonObject> Location = ToFEntry->GetObjectField(TEXT("location"));
		HasKeys(*this, TEXT("location"), Location, { TEXT("x"), TEXT("y"), TEXT("z") });
		// World space: chassis at (10, 0, 20), the ToF 5 cm in front of it.
		TestTrue(TEXT("ToF world location"), FMath::IsNearlyEqual(Location->GetNumberField(TEXT("x")), 15.0, 0.01) && FMath::IsNearlyEqual(Location->GetNumberField(TEXT("z")), 20.0, 0.01));
		HasKeys(*this, TEXT("rotation"), ToFEntry->GetObjectField(TEXT("rotation")), { TEXT("pitch"), TEXT("yaw"), TEXT("roll") });
		HasKeys(*this, TEXT("ToF settings"), ToFEntry->GetObjectField(TEXT("settings")),
			{ TEXT("rows"), TEXT("columns"), TEXT("h_fov_deg"), TEXT("v_fov_deg"), TEXT("max_range_cm") });
		TestEqual(TEXT("ToF settings rows"), static_cast<int32>(ToFEntry->GetObjectField(TEXT("settings"))->GetNumberField(TEXT("rows"))), 2);
		TestEqual(TEXT("ToF has no streams"), ToFEntry->GetArrayField(TEXT("streams")).Num(), 0);
	}
	const TSharedPtr<FJsonObject> SonarEntry = FindSensor(List, TEXT("rear_sonar"));
	if (TestTrue(TEXT("sonar listed"), SonarEntry.IsValid()))
	{
		HasKeys(*this, TEXT("Sonar settings"), SonarEntry->GetObjectField(TEXT("settings")), { TEXT("max_range_cm"), TEXT("cone_deg") });
		TestEqual(TEXT("Sonar parent: root, said as empty"), SonarEntry->GetStringField(TEXT("parent")), FString());
	}
	const TSharedPtr<FJsonObject> HandEntry = FindSensor(List, TEXT("HandToF"));
	if (TestTrue(TEXT("hand-placed ToF listed under its component name"), HandEntry.IsValid()))
	{
		TestFalse(TEXT("hand-placed: rig false"), HandEntry->GetBoolField(TEXT("rig")));
		TestEqual(TEXT("hand-placed: kind"), HandEntry->GetStringField(TEXT("kind")), FString(TEXT("ToF")));
	}
	TestTrue(TEXT("camera listed"), FindSensor(List, TEXT("cam")).IsValid());
	TestTrue(TEXT("\"\" lists every actor"), FindSensor(ParseJson(URammsSensingBridge::ListSensorsJson(FString())), TEXT("front_tof")).IsValid() || URammsSensingBridge::GetSensingWorld() != TestWorld.World);

	// Read: one sample per kind, with the agreed keys.
	const TSharedPtr<FJsonObject> ToFRead = ParseJson(URammsSensingBridge::ReadSensorJson(Path, TEXT("front_tof"), true));
	TestTrue(TEXT("ToF read ok"), IsOk(ToFRead));
	if (HasKeys(*this, TEXT("ToF read"), ToFRead,
			{ TEXT("ok"), TEXT("name"), TEXT("kind"), TEXT("timestamp"), TEXT("rows"), TEXT("columns"), TEXT("distances_cm"),
				TEXT("min_distance_cm"), TEXT("any_hit") }))
	{
		TestEqual(TEXT("ToF read rows"), static_cast<int32>(ToFRead->GetNumberField(TEXT("rows"))), 2);
		TestEqual(TEXT("ToF read columns"), static_cast<int32>(ToFRead->GetNumberField(TEXT("columns"))), 3);
		TestEqual(TEXT("ToF measured now: rows x columns distances"), ToFRead->GetArrayField(TEXT("distances_cm")).Num(), 6);
	}
	HasKeys(*this, TEXT("Sonar read"), ParseJson(URammsSensingBridge::ReadSensorJson(Path, TEXT("rear_sonar"), true)),
		{ TEXT("ok"), TEXT("name"), TEXT("kind"), TEXT("timestamp"), TEXT("distance_cm"), TEXT("hit"), TEXT("hit_location"),
			TEXT("hit_normal"), TEXT("hit_actor") });
	const TSharedPtr<FJsonObject> ImuRead = ParseJson(URammsSensingBridge::ReadSensorJson(Path, TEXT("imu"), false));
	if (HasKeys(*this, TEXT("IMU read"), ImuRead,
			{ TEXT("ok"), TEXT("name"), TEXT("kind"), TEXT("timestamp"), TEXT("linear_acceleration"), TEXT("angular_velocity"),
				TEXT("orientation"), TEXT("euler"), TEXT("units") }))
	{
		HasKeys(*this, TEXT("IMU orientation"), ImuRead->GetObjectField(TEXT("orientation")), { TEXT("x"), TEXT("y"), TEXT("z"), TEXT("w") });
		HasKeys(*this, TEXT("IMU euler"), ImuRead->GetObjectField(TEXT("euler")), { TEXT("roll"), TEXT("pitch"), TEXT("yaw") });
		HasKeys(*this, TEXT("IMU units"), ImuRead->GetObjectField(TEXT("units")), { TEXT("linear_acceleration"), TEXT("angular_velocity") });
	}
	const TSharedPtr<FJsonObject> CamRead = ParseJson(URammsSensingBridge::ReadSensorJson(Path, TEXT("cam"), true));
	TestFalse(TEXT("camera read is not ok"), IsOk(CamRead));
	TestTrue(TEXT("camera read points at the streams"),
		CamRead.IsValid() && CamRead->GetStringField(TEXT("error")) == TEXT("cameras stream over RMSS: see its streams"));
	TestFalse(TEXT("unknown sensor read fails"), IsOk(ParseJson(URammsSensingBridge::ReadSensorJson(Path, TEXT("nothing"), false))));

	// ReadSensors: every ToF / Sonar / IMU, rig-built or not, and no camera.
	const TSharedPtr<FJsonObject> All = ParseJson(URammsSensingBridge::ReadSensorsJson(Path));
	TestTrue(TEXT("read all ok"), IsOk(All));
	HasKeys(*this, TEXT("read all"), All, { TEXT("ok"), TEXT("actor"), TEXT("readings") });
	if (All.IsValid() && All->HasField(TEXT("readings")))
	{
		TestEqual(TEXT("four readings (ToF, Sonar, IMU, hand ToF)"), All->GetArrayField(TEXT("readings")).Num(), 4);
	}

	// Remove: rig sensors only.
	TestTrue(TEXT("remove ToF"), URammsSensingBridge::RemoveSensor(Path, TEXT("front_tof")));
	TestFalse(TEXT("removed ToF no longer listed"), FindSensor(ParseJson(URammsSensingBridge::ListSensorsJson(Path)), TEXT("front_tof")).IsValid());
	TestFalse(TEXT("remove again"), URammsSensingBridge::RemoveSensor(Path, TEXT("front_tof")));
	TestFalse(TEXT("a hand-placed sensor is not removed"), URammsSensingBridge::RemoveSensor(Path, TEXT("HandToF")));
	TestTrue(TEXT("hand-placed still there"), IsValid(HandToF));
	TestTrue(TEXT("remove camera"), URammsSensingBridge::RemoveSensor(Path, TEXT("cam")));
	return true;
}

// ---------------------------------------------------------------------------
// The stream conventions (pure)
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRammsSensingStreamConventions, "Ramms.Sensing.Streams.Conventions",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRammsSensingStreamConventions::RunTest(const FString& Parameters)
{
	using namespace RammsSensingTests;

	TestEqual(TEXT("color id"), FRammsCameraStreams::StreamId(TEXT("wrist"), ERammsCameraStreamRole::Color), FString(TEXT("camera/wrist/color")));
	TestEqual(TEXT("depth id"), FRammsCameraStreams::StreamId(TEXT("wrist"), ERammsCameraStreamRole::Depth), FString(TEXT("camera/wrist/depth")));
	TestEqual(TEXT("motion id"), FRammsCameraStreams::StreamId(TEXT("wrist"), ERammsCameraStreamRole::Motion), FString(TEXT("motion/wrist")));
	TestEqual(TEXT("mask id"), FRammsCameraStreams::StreamId(TEXT("wrist"), ERammsCameraStreamRole::Mask), FString(TEXT("mask/wrist")));
	const FRammsCameraStreamDesc Mask = FRammsCameraStreams::Describe(TEXT("wrist"), ERammsCameraStreamRole::Mask, 3);
	TestEqual(TEXT("mask channel C+300"), Mask.Channel, 303);
	TestTrue(TEXT("mask is FrameData"), Mask.Message == ERammsStreamMessageType::FrameData);
	TestEqual(TEXT("mask fmt"), Mask.Format, FString(TEXT("mono8")));
	TestTrue(TEXT("motion is FrameMotion"), FRammsCameraStreams::MessageType(ERammsCameraStreamRole::Motion) == ERammsStreamMessageType::FrameMotion);
	TestEqual(TEXT("depth channel C+100"), FRammsCameraStreams::Describe(TEXT("wrist"), ERammsCameraStreamRole::Depth, 3).Channel, 103);

	// One frame: 8x6 colour, 4x3 depth, 4x3 motion, 8x6 class mask.
	FCaptureData Data;
	Data.CameraID.ActorName = TEXT("Robot_C_0");
	Data.CameraID.ComponentName = TEXT("wrist");
	Data.CameraID.UniqueID = TEXT("Robot_C_0::wrist");
	Data.ActorPath = TEXT("/Temp/Map.Map:PersistentLevel.Robot_C_0");
	Data.FrameNumber = 42;
	Data.Timestamp = 1.5;
	Data.Width = 8;
	Data.Height = 6;
	Data.DepthWidth = 4;
	Data.DepthHeight = 3;
	Data.MotionWidth = 4;
	Data.MotionHeight = 3;
	Data.ClassWidth = 8;
	Data.ClassHeight = 6;
	Data.ClassData.Init(0, 48);
	Data.ClassData[5] = 9;

	const TArray<FString> BaseKeys = { TEXT("w"), TEXT("h"), TEXT("depth_w"), TEXT("depth_h"), TEXT("frame"), TEXT("timestamp"),
		TEXT("camera"), TEXT("actor"), TEXT("transform"), TEXT("intrinsics"), TEXT("stream_id"), TEXT("group"), TEXT("role"),
		TEXT("name"), TEXT("fmt") };

	const TSharedPtr<FJsonObject> Color = ParseJson(FRammsCameraStreams::BuildFrameMetadataJson(Data, TEXT("wrist"), ERammsCameraStreamRole::Color));
	if (HasKeys(*this, TEXT("color meta"), Color, BaseKeys))
	{
		TestEqual(TEXT("color stream_id"), Color->GetStringField(TEXT("stream_id")), FString(TEXT("camera/wrist/color")));
		TestEqual(TEXT("color group"), Color->GetStringField(TEXT("group")), FString(TEXT("wrist")));
		TestEqual(TEXT("color role"), Color->GetStringField(TEXT("role")), FString(TEXT("color")));
		TestEqual(TEXT("color name"), Color->GetStringField(TEXT("name")), FString(TEXT("wrist Color")));
		TestEqual(TEXT("color fmt"), Color->GetStringField(TEXT("fmt")), FString(TEXT("bgra8")));
		TestEqual(TEXT("color w"), static_cast<int32>(Color->GetNumberField(TEXT("w"))), 8);
		TestEqual(TEXT("camera key kept"), Color->GetStringField(TEXT("camera")), FString(TEXT("Robot_C_0::wrist")));
	}

	const TSharedPtr<FJsonObject> Motion = ParseJson(FRammsCameraStreams::BuildFrameMetadataJson(Data, TEXT("wrist"), ERammsCameraStreamRole::Motion));
	if (HasKeys(*this, TEXT("motion meta"), Motion, BaseKeys) && HasKeys(*this, TEXT("motion meta"), Motion, { TEXT("motion_w"), TEXT("motion_h") }))
	{
		TestEqual(TEXT("motion w is the motion grid"), static_cast<int32>(Motion->GetNumberField(TEXT("w"))), 4);
		TestEqual(TEXT("motion h is the motion grid"), static_cast<int32>(Motion->GetNumberField(TEXT("h"))), 3);
		TestEqual(TEXT("motion fmt"), Motion->GetStringField(TEXT("fmt")), FString(TEXT("rg32f")));
		TestEqual(TEXT("motion stream_id"), Motion->GetStringField(TEXT("stream_id")), FString(TEXT("motion/wrist")));
	}

	// The class table: unlabeled 0, Car 3, Bench 7 -> NumSegmentIDs 8.
	FCameraCaptureSegTable Table;
	Table.FineNames[3] = FName(TEXT("Car"));
	Table.FineNames[7] = FName(TEXT("Bench"));
	TestEqual(TEXT("NumSegmentIds: highest class + 1"), FRammsCameraStreams::NumSegmentIds(Table), 8);

	const TSharedPtr<FJsonObject> MaskMeta = ParseJson(FRammsCameraStreams::BuildFrameMetadataJson(Data, TEXT("wrist"), ERammsCameraStreamRole::Mask, &Table));
	if (HasKeys(*this, TEXT("mask meta"), MaskMeta, BaseKeys)
		&& HasKeys(*this, TEXT("mask meta"), MaskMeta, { TEXT("material_params"), TEXT("MaterialScalarParameters"), TEXT("classes") }))
	{
		TestEqual(TEXT("mask fmt"), MaskMeta->GetStringField(TEXT("fmt")), FString(TEXT("mono8")));
		TestEqual(TEXT("mask role"), MaskMeta->GetStringField(TEXT("role")), FString(TEXT("mask")));
		TestEqual(TEXT("mask stream_id"), MaskMeta->GetStringField(TEXT("stream_id")), FString(TEXT("mask/wrist")));
		TestEqual(TEXT("mask w is the class grid"), static_cast<int32>(MaskMeta->GetNumberField(TEXT("w"))), 8);
		TestEqual(TEXT("NumSegmentIDs"), static_cast<int32>(MaskMeta->GetObjectField(TEXT("material_params"))->GetNumberField(TEXT("NumSegmentIDs"))), 8);
		TestEqual(TEXT("MaterialScalarParameters.NumSegmentIDs"),
			static_cast<int32>(MaskMeta->GetObjectField(TEXT("MaterialScalarParameters"))->GetNumberField(TEXT("NumSegmentIDs"))), 8);
		const TSharedPtr<FJsonObject> Classes = MaskMeta->GetObjectField(TEXT("classes"));
		TestEqual(TEXT("classes[3]"), Classes->GetStringField(TEXT("3")), FString(TEXT("Car")));
		TestEqual(TEXT("classes[7]"), Classes->GetStringField(TEXT("7")), FString(TEXT("Bench")));
		TestTrue(TEXT("classes[0] is the unlabeled class"), Classes->HasField(TEXT("0")));
		TestEqual(TEXT("only defined classes"), Classes->Values.Num(), 3);
	}

	// Without a table, NumSegmentIDs covers the mask's own values.
	const TSharedPtr<FJsonObject> Bare = ParseJson(FRammsCameraStreams::BuildFrameMetadataJson(Data, TEXT("wrist"), ERammsCameraStreamRole::Mask));
	TestTrue(TEXT("no table: highest pixel + 1"),
		Bare.IsValid() && static_cast<int32>(Bare->GetObjectField(TEXT("material_params"))->GetNumberField(TEXT("NumSegmentIDs"))) == 10);

	// The exact camera filter: the named component on this actor, nothing else.
	FCameraIdentifier Front;
	Front.ActorName = TEXT("Robot_C_0");
	Front.ComponentName = TEXT("front");
	Front.UniqueID = TEXT("Robot_C_0::front");
	FCameraIdentifier FrontWide = Front;
	FrontWide.ComponentName = TEXT("front_wide");
	FrontWide.UniqueID = TEXT("Robot_C_0::front_wide");
	const FString Here = TEXT("/Temp/Map.Map:PersistentLevel.Robot_C_0");
	const FString There = TEXT("/Temp/Map.Map:PersistentLevel.Robot_C_1");
	TestTrue(TEXT("exact: its own camera"), URammsStreamSourceComponent::CameraMatchesFilter(TEXT("front"), true, Front, Here, Here));
	TestFalse(TEXT("exact: not a longer name"), URammsStreamSourceComponent::CameraMatchesFilter(TEXT("front"), true, FrontWide, Here, Here));
	TestFalse(TEXT("exact: not another actor's"), URammsStreamSourceComponent::CameraMatchesFilter(TEXT("front"), true, Front, There, Here));
	TestTrue(TEXT("substring: the old behaviour"), URammsStreamSourceComponent::CameraMatchesFilter(TEXT("front"), false, FrontWide, There, Here));
	TestTrue(TEXT("empty filter: everything"), URammsStreamSourceComponent::CameraMatchesFilter(FString(), true, FrontWide, There, Here));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
