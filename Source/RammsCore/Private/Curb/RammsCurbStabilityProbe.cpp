// Copyright Epic Games, Inc. All Rights Reserved.

// A development console command that evaluates a curb controller's robot model
// (Private/Curb/RammsCurbRobotModel) at poses read from a file, so the model a
// robot is set up with -- its legs' kinematics, the centre of mass and moving
// masses its tuning gives it -- can be checked against an offline reference at
// poses no crossing reaches. Not in shipping builds.

#include "CoreMinimal.h"

#if !UE_BUILD_SHIPPING

	#include "Curb/RammsCurbRobotModel.h"
	#include "Dom/JsonObject.h"
	#include "Engine/World.h"
	#include "EngineUtils.h"
	#include "GameFramework/Pawn.h"
	#include "HAL/IConsoleManager.h"
	#include "Misc/FileHelper.h"
	#include "Ramms5BarLinkageController.h"
	#include "RammsCurbNegotiationController.h"
	#include "Serialization/JsonReader.h"
	#include "Serialization/JsonSerializer.h"
	#include "Serialization/JsonWriter.h"

namespace
{
	TArray<TSharedPtr<FJsonValue>> Numbers(std::initializer_list<double> Values)
	{
		TArray<TSharedPtr<FJsonValue>> Out;
		for (const double V : Values)
		{
			Out.Add(MakeShared<FJsonValueNumber>(V));
		}
		return Out;
	}

	const TCHAR* KindName(ERammsCurbLegKind Kind)
	{
		return Kind == ERammsCurbLegKind::FiveBar ? TEXT("5-bar") : TEXT("4-bar");
	}

	/**
	 * ramms.Curb.StabilityProbe <poses.json> <out.json>: the first curb
	 * controller in the world, its robot model (each leg's radius, contact
	 * width and roll direction, each 5-bar's reach gridded from its
	 * controller's SolveTarget), and each pose's centre of mass and stability
	 * margins (RammsCurb::PoseMargin, chassis level).
	 *
	 * In: { "accel": cm/s^2, "decel": cm/s^2 (both optional: the controller's
	 * tuning), "poses": [ { "id": any, "legs": { "<leg name>": { "z": cm,
	 * "x": cm (a 5-bar's; a 4-bar's comes from its table at z), "support":
	 * bool, "gradient": [dz/dx, dz/dy] (chassis frame, optional) }, ... } } ] }.
	 * A leg not named stays at its zero pose, off the ground.
	 */
	void RunStabilityProbe(const TArray<FString>& Args, UWorld* World, FOutputDevice& Ar)
	{
		if (Args.Num() < 2)
		{
			Ar.Log(TEXT("usage: ramms.Curb.StabilityProbe <poses.json> <out.json>"));
			return;
		}
		URammsCurbNegotiationController* Curb = nullptr;
		if (World)
		{
			for (TActorIterator<APawn> It(World); It && !Curb; ++It)
			{
				Curb = It->FindComponentByClass<URammsCurbNegotiationController>();
			}
		}
		if (!Curb)
		{
			Ar.Log(TEXT("ramms.Curb.StabilityProbe: no curb controller in this world"));
			return;
		}

		FString Text;
		if (!FFileHelper::LoadFileToString(Text, *Args[0]))
		{
			Ar.Logf(TEXT("ramms.Curb.StabilityProbe: cannot read %s"), *Args[0]);
			return;
		}
		TSharedPtr<FJsonObject> In;
		if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), In) || !In.IsValid())
		{
			Ar.Logf(TEXT("ramms.Curb.StabilityProbe: %s is not JSON"), *Args[0]);
			return;
		}

		// The model, as the controller builds it from its legs and tuning.
		const TArray<FRammsCurbLeg>&			   Legs = Curb->Legs;
		TArray<FVector2D>						   ZeroPoses;
		TArray<const URamms5BarLinkageController*> FiveBars;
		ZeroPoses.Init(FVector2D::ZeroVector, Legs.Num());
		FiveBars.Init(nullptr, Legs.Num());
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			if (Legs[i].Kind != ERammsCurbLegKind::FiveBar || !Curb->GetOwner())
			{
				continue;
			}
			for (UActorComponent* Component : Curb->GetOwner()->GetComponents())
			{
				const URamms5BarLinkageController* FiveBar = Cast<URamms5BarLinkageController>(Component);
				if (FiveBar && FiveBar->GetFName() == Legs[i].FiveBarComponent)
				{
					RammsCurb::FiveBarZeroPose(*FiveBar, ZeroPoses[i]);
					FiveBars[i] = FiveBar;
				}
			}
		}
		RammsCurb::FRobotModel Model;
		FString				   Why;
		if (!RammsCurb::BuildRobotModel(Legs, Curb->Tuning, ZeroPoses, Model, Why))
		{
			Ar.Logf(TEXT("ramms.Curb.StabilityProbe: %s"), *Why);
			return;
		}
		double Accel = Curb->Tuning.Acceleration;
		double Decel = Curb->Tuning.Deceleration;
		In->TryGetNumberField(TEXT("accel"), Accel);
		In->TryGetNumberField(TEXT("decel"), Decel);

		TSharedRef<FJsonObject>		   Out = MakeShared<FJsonObject>();
		TSharedRef<FJsonObject>		   M = MakeShared<FJsonObject>();
		TArray<TSharedPtr<FJsonValue>> ModelLegs;
		for (int32 i = 0; i < Legs.Num(); ++i)
		{
			const RammsCurb::FModelLeg& L = Model.Legs[i];
			TSharedRef<FJsonObject>		J = MakeShared<FJsonObject>();
			J->SetStringField(TEXT("name"), Legs[i].Name.ToString());
			J->SetStringField(TEXT("kind"), KindName(L.Kind));
			J->SetNumberField(TEXT("y"), L.Y);
			J->SetNumberField(TEXT("radius"), L.RadiusCm);
			J->SetNumberField(TEXT("contact_width"), L.ContactWidthCm);
			J->SetNumberField(TEXT("roll_direction_deg"), FMath::RadiansToDegrees(FMath::Atan2(L.RollDirection.Y, L.RollDirection.X)));
			J->SetNumberField(TEXT("moving_mass_kg"), L.MovingMassKg);
			J->SetArrayField(TEXT("zero_pose"), Numbers({ L.ZeroPose.X, L.ZeroPose.Y }));
			if (L.FourBar.IsValid())
			{
				const FVector2D Range = L.FourBar.HeightRange();
				J->SetArrayField(TEXT("z_range"), Numbers({ Range.X, Range.Y }));
			}
			if (const URamms5BarLinkageController* FiveBar = FiveBars[i])
			{
				// The leg's reach as the model grids it, from its controller's
				// own test (SolveTarget: the knees' and the hips' limits), over
				// the extent the controller advertises and a centimetre round.
				bool			bValidX = false, bValidZ = false;
				const FVector2D XExtent = FiveBar->GetReachableExtent(false, bValidX);
				const FVector2D ZExtent = FiveBar->GetReachableExtent(true, bValidZ);
				if (bValidX && bValidZ)
				{
					RammsCurb::FFiveBarGrid Grid;
					Grid.Build(
						[FiveBar](double X, double Z) {
							bool bReachable = false;
							FiveBar->SolveTarget(FVector2D(X, Z), bReachable);
							return bReachable;
						},
						FVector2D(FMath::FloorToDouble(XExtent.X) - 1.0, FMath::CeilToDouble(XExtent.Y) + 1.0),
						FVector2D(FMath::FloorToDouble(ZExtent.X) - 1.0, FMath::CeilToDouble(ZExtent.Y) + 1.0));
					const FVector2D			AtZero = Grid.HeightRangeAt(0.0);
					TSharedRef<FJsonObject> G = MakeShared<FJsonObject>();
					G->SetArrayField(TEXT("x_extent"), Numbers({ XExtent.X, XExtent.Y }));
					G->SetArrayField(TEXT("z_extent"), Numbers({ ZExtent.X, ZExtent.Y }));
					G->SetNumberField(TEXT("cells"), Grid.CountReachable());
					G->SetArrayField(TEXT("z_at_x0"), Numbers({ AtZero.X, AtZero.Y }));
					J->SetObjectField(TEXT("grid"), G);
				}
			}
			ModelLegs.Add(MakeShared<FJsonValueObject>(J));
		}
		M->SetArrayField(TEXT("legs"), ModelLegs);
		M->SetNumberField(TEXT("mass_kg"), Model.MassKg);
		M->SetArrayField(TEXT("center_of_mass"), Numbers({ Model.CenterOfMass.X, Model.CenterOfMass.Y, Model.CenterOfMass.Z }));
		M->SetArrayField(TEXT("belly"), Numbers({ Model.BellyHalfExtent.X, Model.BellyHalfExtent.Y, Model.BellyUndersideCm }));
		M->SetNumberField(TEXT("accel"), Accel);
		M->SetNumberField(TEXT("decel"), Decel);
		Out->SetObjectField(TEXT("model"), M);
		Out->SetStringField(TEXT("actor"), GetNameSafe(Curb->GetOwner()));

		TArray<TSharedPtr<FJsonValue>>		  Results;
		const TArray<TSharedPtr<FJsonValue>>* Poses = nullptr;
		if (In->TryGetArrayField(TEXT("poses"), Poses))
		{
			for (const TSharedPtr<FJsonValue>& Value : *Poses)
			{
				const TSharedPtr<FJsonObject> Pose = Value->AsObject();
				if (!Pose.IsValid())
				{
					continue;
				}
				const TSharedPtr<FJsonObject>* LegsIn = nullptr;
				Pose->TryGetObjectField(TEXT("legs"), LegsIn);
				TArray<RammsCurb::FPoseLeg> Legged;
				for (int32 i = 0; i < Legs.Num(); ++i)
				{
					RammsCurb::FPoseLeg&		   P = Legged.AddDefaulted_GetRef();
					const RammsCurb::FModelLeg&	   L = Model.Legs[i];
					const TSharedPtr<FJsonObject>* LegIn = nullptr;
					P.XZ = L.ZeroPose;
					if (!LegsIn || !(*LegsIn)->TryGetObjectField(Legs[i].Name.ToString(), LegIn))
					{
						continue;
					}
					double Z = L.ZeroPose.Y, X = L.ZeroPose.X;
					(*LegIn)->TryGetNumberField(TEXT("z"), Z);
					(*LegIn)->TryGetNumberField(TEXT("x"), X);
					P.XZ = FVector2D(L.FourBar.IsValid() ? L.FourBar.XAt(Z) : X, Z);
					(*LegIn)->TryGetBoolField(TEXT("support"), P.bSupport);
					const TArray<TSharedPtr<FJsonValue>>* Gradient = nullptr;
					if ((*LegIn)->TryGetArrayField(TEXT("gradient"), Gradient) && Gradient->Num() == 2)
					{
						P.Gradient = FVector2D((*Gradient)[0]->AsNumber(), (*Gradient)[1]->AsNumber());
					}
				}
				const RammsCurb::FPoseMargin R = RammsCurb::PoseMargin(Model, Legged, Accel, Decel);
				TSharedRef<FJsonObject>		 J = MakeShared<FJsonObject>();
				if (const TSharedPtr<FJsonValue> Id = Pose->TryGetField(TEXT("id")))
				{
					J->SetField(TEXT("id"), Id);
				}
				J->SetArrayField(TEXT("center_of_mass"), Numbers({ R.CenterOfMass.X, R.CenterOfMass.Y, R.CenterOfMass.Z }));
				J->SetNumberField(TEXT("height"), R.HeightCm);
				J->SetNumberField(TEXT("static"), R.Support ? R.StaticCm : -1e9);
				J->SetNumberField(TEXT("dynamic"), R.Support ? R.DynamicCm : -1e9);
				J->SetNumberField(TEXT("support"), R.Support);
				TArray<TSharedPtr<FJsonValue>> Xs;
				for (const RammsCurb::FPoseLeg& P : Legged)
				{
					Xs.Add(MakeShared<FJsonValueArray>(Numbers({ P.XZ.X, P.XZ.Y })));
				}
				J->SetArrayField(TEXT("legs_xz"), Xs);
				Results.Add(MakeShared<FJsonValueObject>(J));
			}
		}
		Out->SetArrayField(TEXT("poses"), Results);

		FString					  Written;
		TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Written);
		FJsonSerializer::Serialize(Out, Writer);
		if (!FFileHelper::SaveStringToFile(Written, *Args[1]))
		{
			Ar.Logf(TEXT("ramms.Curb.StabilityProbe: cannot write %s"), *Args[1]);
			return;
		}
		Ar.Logf(TEXT("ramms.Curb.StabilityProbe: %d poses of '%s' to %s"), Results.Num(), *GetNameSafe(Curb->GetOwner()), *Args[1]);
	}

	FAutoConsoleCommandWithWorldArgsAndOutputDevice StabilityProbeCommand(TEXT("ramms.Curb.StabilityProbe"),
		TEXT("Development: evaluate the first curb controller's robot model at the poses in a JSON file and write each pose's\n")
			TEXT("centre of mass and stability margins to another (ramms.Curb.StabilityProbe <poses.json> <out.json>)."),
		FConsoleCommandWithWorldArgsAndOutputDeviceDelegate::CreateStatic(&RunStabilityProbe));
} // namespace

#endif // !UE_BUILD_SHIPPING
