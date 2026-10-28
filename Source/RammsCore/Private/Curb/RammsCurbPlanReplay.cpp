// Copyright Epic Games, Inc. All Rights Reserved.

// The per-wheel planner's input to a file and back, and two development
// tools on it: a console variable that has the curb controller write each
// per-wheel plan's input to a file, and a console command that plans such
// files off the game -- so a plan made in the game can be made again,
// exactly, as often as wanted (the planner is pure: the same input gives the
// same plan). The tools are not in shipping builds.

#include "Curb/RammsCurbPlanner.h"

#include "Curb/RammsCurbExecutor.h"
#include "HAL/FileManager.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/MemoryReader.h"
#include "Serialization/MemoryWriter.h"
#include "Serialization/ObjectAndNameAsStringProxyArchive.h"

#if !UE_BUILD_SHIPPING
	#include "Dom/JsonObject.h"
	#include "Serialization/JsonSerializer.h"
	#include "Serialization/JsonWriter.h"
	#include "UObject/UnrealType.h"
#endif

namespace RammsCurb
{
	namespace
	{
		constexpr uint32 PlanInputMagic = 0x49504352; // "RCPI"
		/** 2: the robot moving as it asked (FPlannerInput::bOnTheMove); a version-1
		 *  file reads with it false. */
		constexpr int32 PlanInputVersion = 4; // 3: the model's seat; 4: its aft speed-up, fore slow-down and shift easing

		/** A reflected struct by property name (tagged), so that a file written
		 *  before a field was added still reads, the field at its default. */
		template <typename TStruct>
		void SerializeTagged(FArchive& Ar, TStruct& Value)
		{
			FObjectAndNameAsStringProxyArchive Proxy(Ar, false);
			TStruct::StaticStruct()->SerializeItem(Proxy, &Value, nullptr);
		}

		bool SerializeInput(FArchive& Ar, FPlannerInput& In)
		{
			uint32 Magic = PlanInputMagic;
			int32  Version = PlanInputVersion;
			Ar << Magic << Version;
			if (Magic != PlanInputMagic || Version < 1 || Version > PlanInputVersion)
			{
				return false;
			}
			int32 NumModelLegs = In.Model.Legs.Num();
			Ar << NumModelLegs;
			if (Ar.IsLoading())
			{
				if (NumModelLegs < 0 || NumModelLegs > 64)
				{
					return false;
				}
				In.Model.Legs.SetNum(NumModelLegs);
			}
			for (FModelLeg& L : In.Model.Legs)
			{
				uint8 Kind = static_cast<uint8>(L.Kind);
				uint8 Axle = static_cast<uint8>(L.Axle);
				Ar << Kind << Axle << L.Y << L.RadiusCm << L.ContactWidthCm << L.MovingMassKg << L.ZeroPose << L.RollDirection << L.FourBar;
				L.Kind = static_cast<ERammsCurbLegKind>(Kind);
				L.Axle = static_cast<ERammsCurbAxle>(Axle);
			}
			Ar << In.Model.CenterOfMass << In.Model.MassKg << In.Model.BellyHalfExtent << In.Model.BellyUndersideCm;
			if (Version >= 3)
			{
				FModelSeat& S = In.Model.Seat;
				Ar << S.bHas << S.ZeroPivot << S.MechanismMassKg << S.PayloadMassKg << S.PayloadOffset << S.HeightCm << S.XRange << S.MaxPitchRad
				   << S.bTilts << S.ShiftSpeedCmS;
			}
			if (Version >= 4)
			{
				FModelSeat& S = In.Model.Seat;
				Ar << S.AftSpeedUpCmS << S.ForeSlowDownCmS << S.ShiftAccelCmS2;
			}
			int32 NumLegs = In.Legs.Num();
			Ar << NumLegs;
			if (Ar.IsLoading())
			{
				if (NumLegs < 0 || NumLegs > 64)
				{
					return false;
				}
				In.Legs.SetNum(NumLegs);
			}
			for (FPlannerLeg& L : In.Legs)
			{
				FString Name = L.Name.ToString();
				Ar << Name << L.TyreWidthCm << L.PreloadCm << L.Measured << L.bOffGround << L.MaxCrankRateRadS << L.Grid;
				L.Name = FName(*Name);
			}
			SerializeTagged(Ar, In.Terrain);
			SerializeTagged(Ar, In.Tuning);
			Ar << In.Origin << In.OriginZ << In.Heading << In.AttitudeDeg << In.EntrySpeedCmS << In.bTryNoWalk << In.bTryKnots << In.bTrySearch;
			if (Version >= 2)
			{
				Ar << In.bOnTheMove;
			}
			return !Ar.IsError();
		}
	} // namespace

	bool SavePlannerInput(const FPlannerInput& In, const FString& Path, FString& OutWhy)
	{
		TArray<uint8>  Bytes;
		FMemoryWriter  Writer(Bytes);
		FPlannerInput& Copy = const_cast<FPlannerInput&>(In); // saving reads it only
		if (!SerializeInput(Writer, Copy))
		{
			OutWhy = TEXT("the planner input could not be written");
			return false;
		}
		if (!FFileHelper::SaveArrayToFile(Bytes, *Path))
		{
			OutWhy = FString::Printf(TEXT("cannot write %s"), *Path);
			return false;
		}
		return true;
	}

	bool LoadPlannerInput(const FString& Path, FPlannerInput& Out, FString& OutWhy)
	{
		TArray<uint8> Bytes;
		if (!FFileHelper::LoadFileToArray(Bytes, *Path))
		{
			OutWhy = FString::Printf(TEXT("cannot read %s"), *Path);
			return false;
		}
		FMemoryReader Reader(Bytes);
		Out = FPlannerInput();
		if (!SerializeInput(Reader, Out))
		{
			OutWhy = FString::Printf(TEXT("%s is not a planner input (or not this version of one)"), *Path);
			return false;
		}
		return true;
	}

#if !UE_BUILD_SHIPPING
	namespace
	{
		TAutoConsoleVariable<FString> CVarCurbPlanDumpPath(TEXT("Ramms.Curb.PlanDumpPath"), TEXT(""),
			TEXT("Development: the curb controller writes the next per-wheel plan's input to this file (ramms.Curb.PlanReplay plans it again "
				 "off the game), then clears it."),
			ECVF_Default);

		TAutoConsoleVariable<float> CVarCurbReplayExecDt(TEXT("Ramms.Curb.ReplayExecDt"), 0.0f,
			TEXT("Development: with a tick (s) over 0, ramms.Curb.PlanReplay also runs the executor closed round its kinematic robot "
				 "(ExecDryRun) on each plan at that tick, and writes what it did (the least commanded speed, holds, the log and a trace)."),
			ECVF_Default);

		TAutoConsoleVariable<int32> CVarCurbReplayExecTrace(TEXT("Ramms.Curb.ReplayExecTrace"), 1,
			TEXT("Development: the executor replay's trace takes a line every this many ticks (0: none)."), ECVF_Default);

		TAutoConsoleVariable<float> CVarCurbReplayShiftCm(TEXT("Ramms.Curb.ReplayShiftCm"), 0.0f,
			TEXT("Development: ramms.Curb.PlanReplay plans each input with the robot this far (cm) further back along its heading -- the "
				 "same pose, its chassis origin on the near surface there -- to see how a plan depends on where it starts."),
			ECVF_Default);

		TAutoConsoleVariable<float> CVarCurbReplayCrankRate(TEXT("Ramms.Curb.ReplayCrankRateRadS"), -1.0f,
			TEXT("Development: ramms.Curb.PlanReplay plans each input with every leg's MaxCrankRateRadS set to this (rad/s; 0 no limit) "
				 "instead of the one recorded; negative, the default, keeps the recorded one. A leg field, which the replay's "
				 "Field=Value overrides of FRammsCurbTuning cannot reach."),
			ECVF_Default);

		TAutoConsoleVariable<FString> CVarCurbReplaySeat(TEXT("Ramms.Curb.ReplaySeat"), TEXT(""),
			TEXT("Development: ramms.Curb.PlanReplay plans each input with the seat's ForeSlowDownCmS and ShiftAccelCmS2 set to these "
				 "(\"25,60\"; empty, the default, keeps the recorded ones -- a version-3 input recorded neither). Seat fields, which "
				 "the replay's Field=Value overrides of FRammsCurbTuning cannot reach."),
			ECVF_Default);

		/**
		 * ramms.Curb.PlanReplay <file or folder> <out.json> [Field=Value ...]:
		 * plan each planner input (a file, or every *.planin in a folder, in name
		 * order) and write, per input, its plan's class, summary, figures and
		 * lines (with Ramms.Curb.PlanDebug 1, the per-sample ones too). Each
		 * Field=Value sets that FRammsCurbTuning field (by its name, the value as
		 * the property's text) on every input before it is planned: the same
		 * robot states planned with another tuning.
		 */
		void RunPlanReplay(const TArray<FString>& Args, FOutputDevice& Ar)
		{
			if (Args.Num() < 2)
			{
				Ar.Log(TEXT("usage: ramms.Curb.PlanReplay <file or folder> <out.json> [TuningField=Value ...]"));
				return;
			}
			TArray<TPair<FProperty*, FString>> Overrides;
			for (int32 a = 2; a < Args.Num(); ++a)
			{
				FString	   Field, Value;
				FProperty* Property = Args[a].Split(TEXT("="), &Field, &Value) ? FRammsCurbTuning::StaticStruct()->FindPropertyByName(FName(*Field)) : nullptr;
				if (!Property)
				{
					Ar.Logf(TEXT("ramms.Curb.PlanReplay: '%s' is not FRammsCurbTuning Field=Value"), *Args[a]);
					return;
				}
				Overrides.Emplace(Property, Value);
			}
			TArray<FString> Files;
			if (IFileManager::Get().DirectoryExists(*Args[0]))
			{
				IFileManager::Get().FindFiles(Files, *FPaths::Combine(Args[0], TEXT("*.planin")), true, false);
				Files.Sort();
				for (FString& F : Files)
				{
					F = FPaths::Combine(Args[0], F);
				}
			}
			else
			{
				Files.Add(Args[0]);
			}
			TArray<TSharedPtr<FJsonValue>> Rows;
			for (const FString& File : Files)
			{
				TSharedRef<FJsonObject> Row = MakeShared<FJsonObject>();
				Row->SetStringField(TEXT("file"), FPaths::GetCleanFilename(File));
				FPlannerInput In;
				FString		  Why;
				if (!LoadPlannerInput(File, In, Why))
				{
					Row->SetStringField(TEXT("error"), Why);
					Rows.Add(MakeShared<FJsonValueObject>(Row));
					continue;
				}
				for (const TPair<FProperty*, FString>& O : Overrides)
				{
					if (!O.Key->ImportText_InContainer(*O.Value, &In.Tuning, nullptr, PPF_None))
					{
						Row->SetStringField(TEXT("error"), FString::Printf(TEXT("cannot set %s to '%s'"), *O.Key->GetName(), *O.Value));
					}
				}
				const double CrankRate = CVarCurbReplayCrankRate.GetValueOnGameThread();
				if (CrankRate >= 0.0)
				{
					for (FPlannerLeg& L : In.Legs)
					{
						L.MaxCrankRateRadS = CrankRate;
					}
					Row->SetNumberField(TEXT("crank_rate"), CrankRate);
				}
				const FString SeatArgs = CVarCurbReplaySeat.GetValueOnGameThread();
				if (!SeatArgs.IsEmpty())
				{
					FString Fore, Accel;
					if (SeatArgs.Split(TEXT(","), &Fore, &Accel))
					{
						In.Model.Seat.ForeSlowDownCmS = FCString::Atod(*Fore);
						In.Model.Seat.ShiftAccelCmS2 = FCString::Atod(*Accel);
						Row->SetStringField(TEXT("seat"), SeatArgs);
					}
				}
				const double Shift = CVarCurbReplayShiftCm.GetValueOnGameThread();
				if (Shift != 0.0)
				{
					// Further back along the heading by Shift, at the near surface's
					// height there; the pose, the attitude and the terrain as recorded.
					const FVector2D Was = In.Origin;
					In.Origin -= In.Heading.GetSafeNormal() * Shift;
					In.OriginZ += In.Terrain.Near.HeightAt(In.Origin) - In.Terrain.Near.HeightAt(Was);
					Row->SetNumberField(TEXT("shift_cm"), Shift);
				}
				const double		Started = FPlatformTime::Seconds();
				const FCurbSchedule S = PlanCrossing(In);
				Row->SetNumberField(TEXT("wall_ms"), 1000.0 * (FPlatformTime::Seconds() - Started));
				{
					// What the input held (the start the plan was made from), to tell two
					// starts apart.
					TSharedRef<FJsonObject> Start = MakeShared<FJsonObject>();
					Start->SetNumberField(TEXT("to_edge_cm"),
						-In.Terrain.DistancePastEdge(In.Origin) / FMath::Max(1e-3, FVector2D::DotProduct(In.Heading.GetSafeNormal(), In.Terrain.EdgeNormal)));
					Start->SetNumberField(TEXT("origin_z"), In.OriginZ);
					Start->SetNumberField(TEXT("pitch"), In.AttitudeDeg.X);
					Start->SetNumberField(TEXT("roll"), In.AttitudeDeg.Y);
					Start->SetNumberField(TEXT("entry_speed"), In.EntrySpeedCmS);
					Start->SetNumberField(TEXT("heading_deg"), FMath::RadiansToDegrees(FMath::Atan2(In.Heading.Y, In.Heading.X)));
					Start->SetNumberField(TEXT("step_cm"), In.Terrain.StepHeightCm);
					Start->SetNumberField(TEXT("approach_deg"), In.Terrain.ApproachAngleDeg);
					TArray<TSharedPtr<FJsonValue>> Legs;
					for (const FPlannerLeg& L : In.Legs)
					{
						Legs.Add(MakeShared<FJsonValueString>(FString::Printf(TEXT("%s %.2f %.2f%s"), *L.Name.ToString(), L.Measured.X, L.Measured.Y,
							L.bOffGround ? TEXT(" off") : TEXT(""))));
					}
					Start->SetArrayField(TEXT("legs"), Legs);
					Row->SetObjectField(TEXT("start"), Start);
				}
				Row->SetStringField(TEXT("class"), PlanClassName(S.Class));
				Row->SetStringField(TEXT("summary"), S.Summary);
				Row->SetNumberField(TEXT("floor"), S.VFloor);
				Row->SetNumberField(TEXT("tilt"), S.PlannedMaxTiltDeg);
				Row->SetNumberField(TEXT("pitch"), S.PlannedTiltDeg.X);
				Row->SetNumberField(TEXT("roll"), S.PlannedTiltDeg.Y);
				Row->SetNumberField(TEXT("stops"), S.PlannedStops);
				Row->SetNumberField(TEXT("margin"), S.MinMargin);
				Row->SetNumberField(TEXT("corridor"), S.NarrowestCorridorCm);
				Row->SetNumberField(TEXT("seconds"), S.PlannedCrossingSeconds);
				Row->SetNumberField(TEXT("plan_ms"), S.PlanMs);
				Row->SetNumberField(TEXT("approach_margin"), S.ApproachMarginCm);
				Row->SetBoolField(TEXT("level_in_at_rest"), S.bLevelInAtRest);
				TArray<TSharedPtr<FJsonValue>> Lines;
				for (const FString& L : S.Lines)
				{
					Lines.Add(MakeShared<FJsonValueString>(L));
				}
				Row->SetArrayField(TEXT("schedule"), Lines);
				const double ExecDt = CVarCurbReplayExecDt.GetValueOnGameThread();
				if (ExecDt > 0.0 && S.IsValid())
				{
					// The executor on this plan, closed round its kinematic robot at the
					// tick asked (the controller ticks once per frame).
					FExecDryRunOptions Options;
					Options.Dt = ExecDt;
					Options.TraceEvery = FMath::Max(0, CVarCurbReplayExecTrace.GetValueOnGameThread());
					const FExecDryRunResult R = ExecDryRun(In, S, Options);
					TSharedRef<FJsonObject> Exec = MakeShared<FJsonObject>();
					Exec->SetBoolField(TEXT("done"), R.bDone);
					Exec->SetNumberField(TEXT("seconds"), R.Seconds);
					Exec->SetNumberField(TEXT("min_speed"), R.MinSpeedCmS);
					Exec->SetNumberField(TEXT("stops"), R.Stops);
					Exec->SetNumberField(TEXT("holds"), R.Counts.Holds);
					Exec->SetNumberField(TEXT("deadline_legs"), R.Counts.DeadlineLegs);
					Exec->SetNumberField(TEXT("braking_ticks"), R.Counts.BrakingTicks);
					Exec->SetNumberField(TEXT("max_tilt"), R.MaxTiltDeg);
					Exec->SetNumberField(TEXT("deadline_under_plan"), R.MaxDeadlineUnderPlanCmS);
					Exec->SetNumberField(TEXT("final_pitch"), R.FinalAttitudeDeg.X);
					Exec->SetNumberField(TEXT("final_roll"), R.FinalAttitudeDeg.Y);
					Exec->SetNumberField(TEXT("min_margin"), R.MinMarginCm);
					Exec->SetBoolField(TEXT("faulted"), R.bFaulted);
					Exec->SetStringField(TEXT("fault"), R.FaultWhy);
					auto Strings = [](const TArray<FString>& In) {
						TArray<TSharedPtr<FJsonValue>> Out;
						for (const FString& L : In)
						{
							Out.Add(MakeShared<FJsonValueString>(L));
						}
						return Out;
					};
					Exec->SetArrayField(TEXT("notes"), Strings(R.Notes));
					Exec->SetArrayField(TEXT("log"), Strings(R.Log));
					Exec->SetArrayField(TEXT("trace"), Strings(R.Trace));
					Row->SetObjectField(TEXT("exec"), Exec);
				}
				Rows.Add(MakeShared<FJsonValueObject>(Row));
				Ar.Logf(TEXT("ramms.Curb.PlanReplay: %s: %s (%.1f ms)"), *FPaths::GetCleanFilename(File), *S.Summary, S.PlanMs);
			}
			FString					  Text;
			TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Text);
			FJsonSerializer::Serialize(Rows, Writer);
			if (!FFileHelper::SaveStringToFile(Text, *Args[1]))
			{
				Ar.Logf(TEXT("ramms.Curb.PlanReplay: cannot write %s"), *Args[1]);
				return;
			}
			Ar.Logf(TEXT("ramms.Curb.PlanReplay: %d plans to %s"), Rows.Num(), *Args[1]);
		}

		FAutoConsoleCommandWithArgsAndOutputDevice CmdCurbPlanReplay(TEXT("ramms.Curb.PlanReplay"),
			TEXT("Development: ramms.Curb.PlanReplay <planner input file, or a folder of *.planin> <out.json> -- plans each input "
				 "(Ramms.Curb.PlanDumpPath writes them) and writes the plans."),
			FConsoleCommandWithArgsAndOutputDeviceDelegate::CreateStatic(&RunPlanReplay));
	} // namespace

	void DumpPlannerInputIfAsked(const FPlannerInput& In, FString& OutNote)
	{
		const FString Path = CVarCurbPlanDumpPath.GetValueOnGameThread();
		if (Path.IsEmpty())
		{
			return;
		}
		FString Why;
		OutNote = SavePlannerInput(In, Path, Why) ? FString::Printf(TEXT("planner input written to %s"), *Path) : Why;
		CVarCurbPlanDumpPath->Set(TEXT(""), ECVF_SetByConsole);
	}
#else
	void DumpPlannerInputIfAsked(const FPlannerInput& In, FString& OutNote)
	{
	}
#endif
} // namespace RammsCurb
