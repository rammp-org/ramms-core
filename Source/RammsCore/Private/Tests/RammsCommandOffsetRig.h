// The rig the command offset tests stand a robot base on, shared with the level assist's tests: a bare game world, an
// actor with a robot base over a made-up registry, and an actuation backend with no physics that records every call it
// is given, in order. The tick is called by hand. See RammsCommandOffsetTests.cpp for what is checked on it.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataTable.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "UObject/Package.h"

#include "Ramms5BarKinematics.h"
#include "Ramms5BarLinkageController.h"
#include "RammsActuationBackend.h"
#include "RammsActuationBackendRegistry.h"
#include "RammsControlIds.h"
#include "RammsMotorSpec.h"
#include "RammsRobotBaseComponent.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace RammsCommandOffsetTests
{
	/** One call the backend was given. */
	struct FPush
	{
		enum class EKind : uint8
		{
			Command,
			Profile,
			ClearProfile,
			Release,
		};
		EKind Kind = EKind::Command;
		FName Motor;
		/** The command or the profile's target (the joint's sense). */
		float Value = 0.0f;
		/** A profile's StartFrom, where it has one (it is NaN where the profile is to carry on). */
		bool  bStarts = false;
		float StartFrom = 0.0f;

		FString ToString() const
		{
			static const TCHAR* Kinds[] = { TEXT("command"), TEXT("profile"), TEXT("clear-profile"), TEXT("release") };
			return FString::Printf(TEXT("%s %s %g%s"), Kinds[static_cast<int32>(Kind)], *Motor.ToString(), Value,
				bStarts ? *FString::Printf(TEXT(" from %g"), StartFrom) : TEXT(""));
		}
	};

	inline bool Matches(const FPush& A, const FPush& B)
	{
		return A.Kind == B.Kind && A.Motor == B.Motor && FMath::IsNearlyEqual(A.Value, B.Value, 1e-5f) && A.bStarts == B.bStarts
			&& (!A.bStarts || FMath::IsNearlyEqual(A.StartFrom, B.StartFrom, 1e-5f));
	}

	inline FPush Command(const TCHAR* Motor, float Value)
	{
		FPush P;
		P.Motor = Motor;
		P.Value = Value;
		return P;
	}

	inline FPush Profile(const TCHAR* Motor, float Target)
	{
		FPush P;
		P.Kind = FPush::EKind::Profile;
		P.Motor = Motor;
		P.Value = Target;
		return P;
	}

	inline FPush ProfileFrom(const TCHAR* Motor, float Target, float StartFrom)
	{
		FPush P = Profile(Motor, Target);
		P.bStarts = true;
		P.StartFrom = StartFrom;
		return P;
	}

	inline FPush Released(const TCHAR* Motor)
	{
		FPush P;
		P.Kind = FPush::EKind::Release;
		P.Motor = Motor;
		return P;
	}

	inline FPush ProfileCleared(const TCHAR* Motor)
	{
		FPush P;
		P.Kind = FPush::EKind::ClearProfile;
		P.Motor = Motor;
		return P;
	}

	/** What the backend was given, and what its joints read. */
	struct FBackendLog
	{
		TArray<FPush> Pushes;
		/** A joint's reading (its own sense) where a test sets one; else it reads what it was last told, as a servo
		 *  that has arrived. */
		TMap<FName, float> Joints;
		TMap<FName, float> Told;
		/** Whether the backend eases profiles itself (as the MuJoCo one) or leaves that to the base (as Chaos). */
		bool bProfiles = true;
		/** Where a motor's joint is in the world, where a test puts one; else at the world's origin. */
		TMap<FName, FTransform> Transforms;
		/** The backend's simulation clock (s), where a test keeps one; under 0, as a backend that has none (everything
		 *  then goes by the game world's clock, which on this rig stands at zero). */
		double SimTime = -1.0;
		/** When the poses in Transforms were taken, on that clock, where a test says (a simulator stepping on its own
		 *  thread: the poses a tick reads are older than the clock it reads). Unset: as the clock, now. */
		TOptional<double> TransformTime;

		/** How many commands or profile targets Motor was given. */
		int32 CountFor(FName Motor) const
		{
			int32 N = 0;
			for (const FPush& P : Pushes)
			{
				N += (P.Motor == Motor && (P.Kind == FPush::EKind::Command || P.Kind == FPush::EKind::Profile)) ? 1 : 0;
			}
			return N;
		}

		/** The last value Motor was told (NaN: never). */
		float LastFor(FName Motor) const
		{
			const float* Value = Told.Find(Motor);
			return Value ? *Value : NAN;
		}
	};

	class FCountingBackend final : public IRammsActuationBackend
	{
	public:
		explicit FCountingBackend(const TSharedRef<FBackendLog>& InLog)
			: Log(InLog)
		{
		}

		virtual bool Initialize(URammsRobotBaseComponent&) override { return true; }
		virtual void SetCommand(FName MotorId, float Value) override
		{
			FPush P;
			P.Motor = MotorId;
			P.Value = Value;
			Log->Pushes.Add(P);
			Log->Told.Add(MotorId, Value);
		}
		virtual float GetValue(FName MotorId) const override
		{
			if (const float* Joint = Log->Joints.Find(MotorId))
			{
				return *Joint;
			}
			const float* Value = Log->Told.Find(MotorId);
			return Value ? *Value : 0.0f;
		}
		virtual float  GetVelocity(FName) const override { return 0.0f; }
		virtual double GetSimulationTime() const override { return Log->SimTime; }
		virtual double GetTransformTime() const override { return Log->TransformTime.Get(Log->SimTime); }
		virtual bool  ReleaseMotor(FName MotorId) override
		{
			FPush P;
			P.Kind = FPush::EKind::Release;
			P.Motor = MotorId;
			Log->Pushes.Add(P);
			return true;
		}
		virtual bool GetMotorTransform(FName MotorId, FTransform& OutWorld) const override
		{
			const FTransform* Placed = Log->Transforms.Find(MotorId);
			OutWorld = Placed ? *Placed : FTransform::Identity;
			return true;
		}
		virtual bool SetPositionProfile(FName MotorId, float Target, float, float, float StartFrom) override
		{
			if (!Log->bProfiles)
			{
				return false;
			}
			FPush P;
			P.Kind = FPush::EKind::Profile;
			P.Motor = MotorId;
			P.Value = Target;
			P.bStarts = FMath::IsFinite(StartFrom);
			P.StartFrom = P.bStarts ? StartFrom : 0.0f;
			Log->Pushes.Add(P);
			Log->Told.Add(MotorId, Target);
			return true;
		}
		virtual void ClearPositionProfile(FName MotorId) override
		{
			FPush P;
			P.Kind = FPush::EKind::ClearProfile;
			P.Motor = MotorId;
			Log->Pushes.Add(P);
		}

	private:
		TSharedRef<FBackendLog> Log;
	};

	inline FRammsMotorSpec MotorRow(const TCHAR* Id, ERammsActuatorType Type, FVector2D Range = FVector2D::ZeroVector, const TCHAR* ChaosName = nullptr)
	{
		FRammsMotorSpec Spec;
		Spec.Id = Id;
		Spec.Type = Type;
		Spec.ControlRange = Range;
		if (ChaosName)
		{
			Spec.ChaosName = ChaosName;
		}
		return Spec;
	}

	const FName Crank(TEXT("crank"));
	const FName Lift(TEXT("lift"));
	const FName HipA(TEXT("hip_a"));
	const FName HipB(TEXT("hip_b"));
	const FName Wheel(TEXT("wheel"));
	const FName Spin(TEXT("spin"));

	/** The made-up registry: a crank bounded at +-1 rad; a lift on a reversed joint, bounded at +-2 and eased at
	 *  1.5 units/s; two unbounded hips; a torque wheel and a speed motor. */
	inline TArray<FRammsMotorSpec> ToyMotors()
	{
		TArray<FRammsMotorSpec> Specs;
		Specs.Add(MotorRow(TEXT("crank"), ERammsActuatorType::Position, FVector2D(-1.0, 1.0)));
		FRammsMotorSpec LiftSpec = MotorRow(TEXT("lift"), ERammsActuatorType::Position, FVector2D(-2.0, 2.0));
		LiftSpec.Direction = -1.0f;
		LiftSpec.MaxVelocity = 1.5f;
		Specs.Add(LiftSpec);
		Specs.Add(MotorRow(TEXT("hip_a"), ERammsActuatorType::Position));
		Specs.Add(MotorRow(TEXT("hip_b"), ERammsActuatorType::Position));
		Specs.Add(MotorRow(TEXT("wheel"), ERammsActuatorType::Torque));
		Specs.Add(MotorRow(TEXT("spin"), ERammsActuatorType::Velocity));
		return Specs;
	}

	/**
	 * A bare game world holding one actor with a robot base over Specs, begun play on the recording backend (put in
	 * the MuJoCo slot for as long as the rig lives, whatever was registered there back after).
	 */
	struct FRig
	{
		UWorld*						  World = nullptr;
		AActor*						  Actor = nullptr;
		URammsRobotBaseComponent*	  Base = nullptr;
		TSharedRef<FBackendLog>		  Log = MakeShared<FBackendLog>();
		FRammsActuationBackendFactory Previous;
		/** Something to hold the layer that is not the layer's holder in a test: any object will do. */
		UDataTable* Table = nullptr;

		explicit FRig(const TArray<FRammsMotorSpec>& Specs = ToyMotors())
		{
			Previous = RammsActuationBackends::GetMujocoFactory();
			TSharedRef<FBackendLog> SharedLog = Log;
			RammsActuationBackends::RegisterMujocoFactory(FRammsActuationBackendFactory::CreateLambda(
				[SharedLog](URammsRobotBaseComponent&) -> IRammsActuationBackend* { return new FCountingBackend(SharedLog); }));
			World = UWorld::CreateWorld(EWorldType::Game, /*bInformEngineOfWorld=*/false);
			if (World == nullptr)
			{
				return;
			}
			GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
			Actor = World->SpawnActor<AActor>();
			if (Actor == nullptr)
			{
				return;
			}
			Table = NewObject<UDataTable>(GetTransientPackage());
			Table->RowStruct = FRammsMotorSpec::StaticStruct();
			for (const FRammsMotorSpec& Spec : Specs)
			{
				Table->AddRow(Spec.Id, Spec);
			}
			Base = NewObject<URammsRobotBaseComponent>(Actor, TEXT("RobotBase"));
			Base->MotorTable = Table;
			BeginPlayOf(*Base);
		}

		~FRig()
		{
			if (World != nullptr)
			{
				if (Actor != nullptr)
				{
					// Not begun play as an actor, so its components are ended here, as they were begun.
					for (UActorComponent* Component : Actor->GetComponents())
					{
						if (Component && Component->HasBegunPlay())
						{
							Component->EndPlay(EEndPlayReason::Destroyed);
						}
					}
					Actor->Destroy();
				}
				GEngine->DestroyWorldContext(World);
				World->DestroyWorld(false);
			}
			if (Previous.IsBound())
			{
				RammsActuationBackends::RegisterMujocoFactory(Previous);
			}
			else
			{
				RammsActuationBackends::UnregisterMujocoFactory();
			}
		}

		/** Register Component and begin its play, as its actor's BeginPlay would. */
		static bool BeginPlayOf(UActorComponent& Component)
		{
			Component.RegisterComponent();
			if (!Component.IsRegistered())
			{
				return false;
			}
			Component.BeginPlay();
			return Component.HasBegunPlay();
		}

		bool IsReady() const { return Base != nullptr && Base->HasBegunPlay() && Base->HasBackend(); }

		/** A frame's end for the base: its tick. */
		void Tick(float Dt = 0.02f) const { Base->TickComponent(Dt, LEVELTICK_All, nullptr); }

		const UObject* Layer() const { return Actor; }
		const UObject* Other() const { return Table; }

		float Baseline(FName Id) const
		{
			float Value = NAN;
			return Base->GetMotorCommand(Id, Value) ? Value : NAN;
		}
		float Effective(FName Id) const
		{
			float Value = NAN;
			return Base->GetMotorEffectiveCommand(Id, Value) ? Value : NAN;
		}
	};

	inline bool SameBits(float A, float B)
	{
		return FMemory::Memcmp(&A, &B, sizeof(float)) == 0;
	}

	/** A made-up 5-bar: 12 cm proximal links on pivots 10 cm apart at z = 20, 18 cm distal links, the endpoint
	 *  below them. Its hips are the registry's two. */
	inline FRamms5BarLinkageSpec ToyFiveBar()
	{
		FRamms5BarLinkageSpec Spec;
		Spec.ProximalMotorA = HipA;
		Spec.ProximalMotorB = HipB;
		Spec.PivotA = FVector2D(5.0, 20.0);
		Spec.PivotB = FVector2D(-5.0, 20.0);
		Spec.ProximalLengthA = Spec.ProximalLengthB = 12.0f;
		Spec.DistalLengthA = Spec.DistalLengthB = 18.0f;
		Spec.ZeroDirA = 0.0f;
		Spec.AngleSignA = -1.0f;
		Spec.ZeroDirB = UE_PI;
		Spec.AngleSignB = 1.0f;
		Spec.bElbowUpA = true;
		Spec.bElbowUpB = false;
		Spec.bFlipEndpointSide = false;
		return Spec;
	}

	/** The rig with a 5-bar linkage controller on the two hips, holding no target. */
	struct FLinkageRig : FRig
	{
		URamms5BarLinkageController* Linkage = nullptr;

		FLinkageRig()
		{
			if (Actor != nullptr && Base != nullptr)
			{
				Linkage = NewObject<URamms5BarLinkageController>(Actor, TEXT("ToyLinkage"));
				Linkage->Linkage = ToyFiveBar();
				Linkage->HeightScanLimits = FVector2D(-12.0, 12.0);
				Linkage->TranslationScanLimits = FVector2D(-12.0, 12.0);
				BeginPlayOf(*Linkage);
			}
		}

		bool IsReady() const { return FRig::IsReady() && Linkage != nullptr && Linkage->HasBegunPlay(); }

		/** The hips' angles for an endpoint (robot frame; the linkage is not mirrored). */
		FVector2D HipsFor(FVector2D Endpoint, bool& bReachable) const { return Linkage->SolveTarget(Endpoint, bReachable); }

		/** Whether the controller holds a target, by what it tells the control surface. */
		bool HoldsTarget() const
		{
			FVector2D Target;
			return ReadsTarget(Target);
		}

		/** The target it tells the control surface, fore/aft and height: what a stance ramp and a held key start
		 *  from. False where it tells of none on either axis. */
		bool ReadsTarget(FVector2D& OutTarget) const
		{
			float							X = 0.0f, Z = 0.0f;
			const IRammsControlContributor* Contributor = Linkage;
			const bool						bX = Contributor->ReadTarget(RammsControlIds::Linkage::Translation(Linkage->GetName()), X);
			const bool						bZ = Contributor->ReadTarget(RammsControlIds::Linkage::Height(Linkage->GetName()), Z);
			OutTarget = FVector2D(X, Z);
			return bX && bZ;
		}
	};
} // namespace RammsCommandOffsetTests

#endif // WITH_DEV_AUTOMATION_TESTS
