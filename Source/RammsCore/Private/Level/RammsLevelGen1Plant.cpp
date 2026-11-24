// Copyright Epic Games, Inc. All Rights Reserved.

#include "Level/RammsLevelPlant.h"

#include "Components/SceneComponent.h"
#include "Curb/RammsRammpGen1Curb.h"
#include "GameFramework/Actor.h"
#include "Level/RammsLevelAllocate.h"
#include "Level/RammsLevelAttitude.h"
#include "RammpGen1ControllerComponent.h"
#include "RammsDriveModeSelector.h"
#include "RammsDriveStance.h"
#include "RammsMotorSpec.h"
#include "RammsRammpGen1CurbController.h"
#include "RammsRobotBaseComponent.h"
#include "UObject/WeakObjectPtr.h"

/**
 * The plant of the Gen1 kind of robot (RammsLevelPlant.h): two drive wheels,
 * each on its own elevator, and two pairs of small wheels, each pair on one
 * arm. Every length, table and limit is read from the sibling curb
 * controller's Config and from the motor registry; there is no number of any
 * robot in this file.
 *
 * FOUR LEGS, each one lift motor: drive left, drive right, the rear pair, the
 * front pair. A leg's height is its wheels' bottom height by its support's
 * table (how far the bottom rises, chassis level, from where the robot's
 * reference pose has it on the ground, as the lift's joint turns) plus the
 * wheel's radius: every leg's height is then from the same ground, which is
 * the law's only use for it. "Baseline" heights come from the motors'
 * baseline commands (URammsRobotBaseComponent::GetMotorCommand), "measured"
 * ones from the joints. An offset of dz on a leg becomes, on its motor, the
 * difference between the angles the table gives for (baseline height + dz)
 * and for the baseline height, both looked up the same way, so that dz = 0 is
 * exactly no offset.
 *
 * WHERE THE WHEELS ARE on the chassis is read from the bodies each tick: a
 * drive wheel is levelled where its translator has it. A pair is one leg at
 * the middle of its two wheels, which roll does not reach: its arm is rigid
 * across the robot, so a roll correction goes to the two drive legs alone and
 * leaves the robot on three points (both drive wheels and one wheel of a
 * pair).
 *
 * WHO STANDS. This robot's stances do not stand level in its own frame (it
 * rides a degree and more nose up with its front pair carried, nose down with
 * every lift at zero), so the lowest wheel in the chassis' frame is not the
 * ground. Each tick the flat ground its baselines would rest on is worked out
 * from its centre of mass (the config's mass model, the drive assemblies
 * where their translators have them) and given with the legs
 * (FLegReach::StanceGroundZ, RammsLevel::StanceGround).
 *
 * No seat: the rider's seat is fixed to the chassis, so the legs level the
 * rider, and every seat question is answered "none".
 */
namespace RammsLevel
{
	namespace
	{
		/** How far either side of the baseline a lift's rate is measured over (rad). */
		constexpr float LiftProbeRad = 0.01f;
		/** A lift that turns is never taken to move its wheels slower than
		 *  this (cm/s): at the very end of a table the height all but stops
		 *  changing with the angle, and a rate of nothing would read as "not
		 *  limited". */
		constexpr double LiftRateFloorCmS = 0.1;

		bool SameLiftBits(float A, float B)
		{
			return FMemory::Memcmp(&A, &B, sizeof(float)) == 0;
		}

		/** One leg: one lift motor, and what is remembered of its baseline. */
		struct FLiftLeg
		{
			// ---- found once ----
			FName Motor;
			/** Its motor's ControlRange where its registry row sets one (X < Y),
			 *  and how fast its row lets it turn (rad/s; 0: no limit). */
			FVector2D Range = FVector2D::ZeroVector;
			double	  MaxVelocity = 0.0;
			/** How fast the robot turns it (rad/s): the config's speed. */
			double SpeedRadS = 0.0;
			/** Its wheels' bottom height against its joint's angle. */
			RammsRammpGen1Curb::FSupportTable Table;
			double							  RadiusCm = 0.0;
			bool							  bRoll = false;
			/** Its wheel's body, or its two wheels'. */
			FName							Bodies[2];
			TWeakObjectPtr<USceneComponent> Found[2];
			int32							NumBodies = 0;
			/** The angles it works between (the table's, within the motor's
			 *  range) and the heights at their ends (cm, the table's). */
			float  LoAngle = 0.0f;
			float  HiAngle = 0.0f;
			double MinHeight = 0.0;
			double MaxHeight = 0.0;

			// ---- its baseline, remembered ----
			/** At the last MeasureLegs and at the one before, and whether it had one. */
			float NowBaseline = 0.0f;
			float PrevBaseline = 0.0f;
			bool  bNow = false;
			bool  bPrev = false;
			/** A fold: the baseline before it and the one it left. */
			float PreFoldBaseline = 0.0f;
			float FoldedBaseline = 0.0f;
			bool  bFolded = false;
			/** The offset this plant last left on it, as the base read it back. */
			float LeftOffset = 0.0f;

			// ---- as the last MeasureLegs found it ----
			bool bHasBaseline = false;
			/** ApplyOffsets has put the law's offset on it since it last had no baseline. */
			bool bOffsetOn = false;
			/** Its wheels' bottom height at the baseline (cm, the table's). */
			double BaseHeight = 0.0;
			/** Where its wheel is on the chassis (a pair's middle), and half
			 *  the distance between a pair's two wheels (0 for one wheel). */
			FVector2D XY = FVector2D::ZeroVector;
			double	  HalfTrackCm = 0.0;

			float ClampAngle(float Angle) const { return FMath::Clamp(Angle, LoAngle, HiAngle); }
			/** The bottom's height (cm, the table's) at Angle, within its working angles. */
			double HeightAt(float Angle) const { return Table.HeightAt(ClampAngle(Angle)); }
			/** The angle for a height (cm, the table's), within its working heights. */
			bool AngleFor(double HeightCm, float& OutAngle) const
			{
				if (!Table.AngleFor(static_cast<float>(FMath::Clamp(HeightCm, MinHeight, MaxHeight)), OutAngle))
				{
					return false;
				}
				OutAngle = ClampAngle(OutAngle);
				return true;
			}
		};

		class FGen1Plant final : public IPlant
		{
		public:
			FGen1Plant(URammsRobotBaseComponent& InBase, const URammsRammpGen1CurbController& InCurb, const UObject* InLayer)
				: Base(&InBase)
				, Curb(&InCurb)
				, Layer(InLayer)
			{
			}

			virtual bool   Resolve(FString& OutWhy) override;
			virtual void   LevelledMotorIds(TArray<FName>& OutIds) const override;
			virtual bool   HasSeat() const override { return false; }
			virtual FName  SeatMotorId() const override { return FName(); }
			virtual bool   SeatTrimRange(double& OutOwnDeg, double& OutLowDeg, double& OutHighDeg) override;
			virtual void   ApplySeatTrim(double) override {}
			virtual double FoldedSeatTrimDeg() const override { return 0.0; }
			virtual double SeatTrimOnDeg() const override { return 0.0; }
			virtual bool   UnfoldSeatTrim() override { return false; }
			virtual bool   MeasureLegs(FPlantLegs& OutLegs) override;
			virtual void   RestLegsAt(TConstArrayView<double> Z, TArrayView<FLegReach> InOutLegs) const override;
			virtual void   ClearancePoints(const FTuning& Tuning, FPlantPoints& OutPoints) const override;
			virtual void   ApplyOffsets(TConstArrayView<double> Dz) override;
			virtual bool   OffsetsLost() override;
			virtual void   ZeroOffsets() override;
			virtual void   ZeroLegOffsets() override { ZeroOffsets(); }
			virtual void   Fold() override;
			virtual void   FoldSeatTrim() override {}
			virtual bool   Unfold() override;
			virtual void   ForgetFold() override;
			virtual bool   StanceHeights(const FRammsDriveStance& Stance, FLegValues& OutZ) const override;
			virtual bool   FoldedHeights(FLegValues& OutZ) const override;
			virtual bool   RebaseLegs(TConstArrayView<double> BaseZ, FLegValues& OutDz) override;
			virtual void   FoldOrZero(int32& OutFolded, int32& OutZeroed) override;
			virtual void   SyncBaselineHolders() override;

		private:
			bool AddLeg(FName Motor, const FRammsRammpGen1SupportTable& Table, float RadiusCm, float SpeedDegS, bool bRoll, FName BodyA, FName BodyB, FString& OutWhy);
			void RememberOffsets();

			TWeakObjectPtr<URammsRobotBaseComponent>			Base;
			TWeakObjectPtr<const URammsRammpGen1CurbController> Curb;
			TWeakObjectPtr<const UObject>						Layer;
			TArray<FLiftLeg, TInlineAllocator<4>>				Legs;
			TWeakObjectPtr<USceneComponent>						Chassis;
			bool												bFoldRemembered = false;
			/** The centre of mass on the chassis as the last MeasureLegs found
			 *  it (cm), and whether it could be worked out. */
			FVector2D CentreOfMassXY = FVector2D::ZeroVector;
			bool	  bCentreOfMass = false;
		};

		bool FGen1Plant::AddLeg(FName Motor, const FRammsRammpGen1SupportTable& Table, float RadiusCm, float SpeedDegS, bool bRoll, FName BodyA, FName BodyB, FString& OutWhy)
		{
			FRammsMotorSpec Spec;
			if (Motor.IsNone() || !Base->GetMotorSpec(Motor, Spec))
			{
				OutWhy = FString::Printf(TEXT("the lift motor '%s' is not in the robot's motor table"), *Motor.ToString());
				return false;
			}
			if (Spec.Type != ERammsActuatorType::Position)
			{
				OutWhy = FString::Printf(TEXT("the lift motor '%s' is not a position motor"), *Motor.ToString());
				return false;
			}
			FLiftLeg Leg;
			Leg.Motor = Motor;
			Leg.Range = Spec.ControlRange;
			Leg.MaxVelocity = FMath::Max(0.0f, Spec.MaxVelocity);
			Leg.SpeedRadS = FMath::DegreesToRadians(FMath::Max(0.0, static_cast<double>(SpeedDegS)));
			Leg.Table.AngleRad = Table.AngleRad;
			Leg.Table.HeightCm = Table.HeightCm;
			Leg.RadiusCm = RadiusCm;
			Leg.bRoll = bRoll;
			if (!Leg.Table.IsValid() || !(RadiusCm > 0.0f))
			{
				OutWhy = FString::Printf(TEXT("the lift '%s' has no height table, or its wheel no radius"), *Motor.ToString());
				return false;
			}
			// Between the table's ends, and within the motor's own range where
			// its row sets one.
			Leg.LoAngle = FMath::Min(Leg.Table.AngleRad[0], Leg.Table.AngleRad.Last());
			Leg.HiAngle = FMath::Max(Leg.Table.AngleRad[0], Leg.Table.AngleRad.Last());
			if (Leg.Range.X < Leg.Range.Y)
			{
				Leg.LoAngle = FMath::Max(Leg.LoAngle, static_cast<float>(Leg.Range.X));
				Leg.HiAngle = FMath::Min(Leg.HiAngle, static_cast<float>(Leg.Range.Y));
			}
			const double AtLo = Leg.Table.HeightAt(Leg.LoAngle);
			const double AtHi = Leg.Table.HeightAt(Leg.HiAngle);
			Leg.MinHeight = FMath::Min(AtLo, AtHi);
			Leg.MaxHeight = FMath::Max(AtLo, AtHi);
			if (!(Leg.LoAngle < Leg.HiAngle) || !(Leg.MinHeight < Leg.MaxHeight))
			{
				OutWhy = FString::Printf(TEXT("the lift '%s' has no travel"), *Motor.ToString());
				return false;
			}
			const AActor* Owner = Curb->GetOwner();
			Leg.Bodies[0] = BodyA;
			Leg.Bodies[1] = BodyB;
			Leg.NumBodies = BodyB.IsNone() ? 1 : 2;
			for (int32 Index = 0; Index < Leg.NumBodies; ++Index)
			{
				Leg.Found[Index] = FindBody(Owner, Leg.Bodies[Index]);
				if (!Leg.Found[Index].IsValid())
				{
					OutWhy = FString::Printf(TEXT("the wheel body '%s' is not on the robot"), *Leg.Bodies[Index].ToString());
					return false;
				}
			}
			Legs.Add(Leg);
			return true;
		}

		bool FGen1Plant::Resolve(FString& OutWhy)
		{
			Legs.Reset();
			bFoldRemembered = false;
			bCentreOfMass = false;
			const URammsRammpGen1CurbController* CurbPtr = Curb.Get();
			if (!Base.IsValid() || !CurbPtr)
			{
				OutWhy = TEXT("no robot description");
				return false;
			}
			const FRammsRammpGen1CurbConfig& Config = CurbPtr->Config;
			Chassis = FindBody(CurbPtr->GetOwner(), Config.ChassisBody);
			if (!Chassis.IsValid())
			{
				OutWhy = FString::Printf(TEXT("the chassis body '%s' is not on the robot"), *Config.ChassisBody.ToString());
				return false;
			}
			// In this order, which is the legs' everywhere: drive left, drive
			// right, the rear pair, the front pair.
			return AddLeg(Config.ElevatorLeft, Config.DriveTable, Config.DriveWheelRadiusCm, Config.ElevatorSpeedDegS, /*bRoll=*/true, Config.DriveWheelLeftBody, FName(), OutWhy)
				&& AddLeg(Config.ElevatorRight, Config.DriveTable, Config.DriveWheelRadiusCm, Config.ElevatorSpeedDegS, /*bRoll=*/true, Config.DriveWheelRightBody, FName(), OutWhy)
				&& AddLeg(Config.RearLift, Config.RearTable, Config.RearOmniRadiusCm, Config.RearLiftSpeedDegS, /*bRoll=*/false, Config.RearOmniLeftBody, Config.RearOmniRightBody, OutWhy)
				&& AddLeg(Config.FrontArm, Config.FrontTable, Config.FrontOmniRadiusCm, Config.FrontArmSpeedDegS, /*bRoll=*/false, Config.FrontOmniLeftBody, Config.FrontOmniRightBody, OutWhy);
		}

		void FGen1Plant::LevelledMotorIds(TArray<FName>& OutIds) const
		{
			for (const FLiftLeg& Leg : Legs)
			{
				OutIds.AddUnique(Leg.Motor);
			}
		}

		bool FGen1Plant::SeatTrimRange(double& OutOwnDeg, double& OutLowDeg, double& OutHighDeg)
		{
			OutOwnDeg = OutLowDeg = OutHighDeg = 0.0;
			return false;
		}

		bool FGen1Plant::MeasureLegs(FPlantLegs& OutLegs)
		{
			OutLegs.Reset();
			URammsRobotBaseComponent*			 BasePtr = Base.Get();
			const URammsRammpGen1CurbController* CurbPtr = Curb.Get();
			const USceneComponent*				 ChassisPtr = Chassis.Get();
			if (!BasePtr || !CurbPtr || !ChassisPtr)
			{
				return false;
			}
			// The baselines as they stand, and as they stood a tick ago: what
			// FoldOrZero tells a rewritten motor from an untouched one by.
			for (FLiftLeg& Leg : Legs)
			{
				Leg.PrevBaseline = Leg.NowBaseline;
				Leg.bPrev = Leg.bNow;
				Leg.bNow = BasePtr->GetMotorCommand(Leg.Motor, Leg.NowBaseline);
			}
			FBodyFrame Frame;
			if (!ReadBodyFrame(*ChassisPtr, Frame))
			{
				return false;
			}
			FLegValues HalfTracks;
			for (FLiftLeg& Leg : Legs)
			{
				// Where its wheel is on the chassis now (a pair: the middle of
				// its two, and how far apart they are).
				FVector Place[2] = { FVector::ZeroVector, FVector::ZeroVector };
				for (int32 Index = 0; Index < Leg.NumBodies; ++Index)
				{
					const USceneComponent* Body = Leg.Found[Index].Get();
					if (!Body)
					{
						OutLegs.Reset();
						return false;
					}
					Place[Index] = Frame.ToLocal(Body->GetComponentLocation());
				}
				const FVector Middle = Leg.NumBodies == 2 ? 0.5 * (Place[0] + Place[1]) : Place[0];
				Leg.XY = FVector2D(Middle.X, Middle.Y);
				Leg.HalfTrackCm = Leg.NumBodies == 2 ? 0.5 * FMath::Abs(Place[1].Y - Place[0].Y) : 0.0;
				if (Middle.ContainsNaN())
				{
					OutLegs.Reset();
					return false;
				}

				const float Joint = Leg.ClampAngle(BasePtr->GetMotorValue(Leg.Motor));
				Leg.bHasBaseline = Leg.bNow;
				// A lift nothing has commanded stands where its joint is.
				const float BaseAngle = Leg.bNow ? Leg.ClampAngle(Leg.NowBaseline) : Joint;
				Leg.BaseHeight = Leg.HeightAt(BaseAngle);

				FLegReach Reach;
				Reach.XY = Leg.XY;
				Reach.RadiusCm = Leg.RadiusCm;
				Reach.bRoll = Leg.bRoll;
				Reach.bHasBaseline = Leg.bHasBaseline;
				Reach.BaseZ = Leg.BaseHeight + Leg.RadiusCm;
				Reach.MeasuredZ = Leg.HeightAt(Joint) + Leg.RadiusCm;
				Reach.MinZ = FMath::Min(Leg.MinHeight, Leg.BaseHeight) + Leg.RadiusCm;
				Reach.MaxZ = FMath::Max(Leg.MaxHeight, Leg.BaseHeight) + Leg.RadiusCm;
				// How fast its wheels' height can be asked to move: the speed the
				// robot turns the lift at (and what the motor's row allows, if
				// less), through what the table gives per radian here. Offsets do
				// not pass through the lift controller's own ramp: this is the
				// only limit on them.
				double TurnRadS = Leg.SpeedRadS;
				if (Leg.MaxVelocity > 0.0)
				{
					TurnRadS = TurnRadS > 0.0 ? FMath::Min(TurnRadS, Leg.MaxVelocity) : Leg.MaxVelocity;
				}
				if (TurnRadS > 0.0)
				{
					const float	 A = Leg.ClampAngle(BaseAngle - LiftProbeRad);
					const float	 B = Leg.ClampAngle(BaseAngle + LiftProbeRad);
					const double PerRad = B > A ? FMath::Abs(Leg.HeightAt(B) - Leg.HeightAt(A)) / static_cast<double>(B - A) : 0.0;
					Reach.MaxRateCmS = FMath::Max(LiftRateFloorCmS, TurnRadS * PerRad);
				}
				OutLegs.Add(Reach);
				HalfTracks.Add(Leg.HalfTrackCm);
			}
			// Where the robot's weight is: the chassis' own, and the two drive
			// assemblies where their translators have them.
			const FRammsRammpGen1CurbConfig& Config = CurbPtr->Config;
			const double					 Fixed = FMath::Max(0.0, static_cast<double>(Config.FixedMassKg));
			const double					 Drive = FMath::Max(0.0, static_cast<double>(Config.DriveAssemblyMassKg));
			bCentreOfMass = Fixed + 2.0 * Drive > 0.0 && Legs.Num() >= 2;
			if (bCentreOfMass)
			{
				const FVector2D FromHub(Config.DriveAssemblyComFromHubCm, 0.0);
				CentreOfMassXY = (Fixed * FVector2D(Config.FixedComAheadCm, 0.0) + Drive * (Legs[0].XY + FromHub) + Drive * (Legs[1].XY + FromHub)) / (Fixed + 2.0 * Drive);
				// The flat ground the baselines rest on: who stands is judged
				// from it (its stances do not stand level in the chassis' frame).
				StanceGround(OutLegs, HalfTracks, CentreOfMassXY);
			}
			return true;
		}

		void FGen1Plant::RestLegsAt(TConstArrayView<double> Z, TArrayView<FLegReach> InOutLegs) const
		{
			if (!bCentreOfMass || Z.Num() != Legs.Num() || InOutLegs.Num() != Legs.Num())
			{
				return;
			}
			FPlantLegs At;
			At.Append(InOutLegs.GetData(), InOutLegs.Num());
			FLegValues HalfTracks;
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				At[Index].BaseZ = Z[Index];
				HalfTracks.Add(Legs[Index].HalfTrackCm);
			}
			StanceGround(At, HalfTracks, CentreOfMassXY);
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				InOutLegs[Index].StanceGroundZ = At[Index].StanceGroundZ;
			}
		}

		void FGen1Plant::ClearancePoints(const FTuning& Tuning, FPlantPoints& OutPoints) const
		{
			OutPoints.Reset();
			const URammsRammpGen1CurbController* CurbPtr = Curb.Get();
			if (!CurbPtr)
			{
				return;
			}
			// The frame's underside, a flat box under the chassis: its four
			// corners, at the height the reference pose has them over the
			// ground (the legs' heights are from that same ground).
			const FRammsRammpGen1CurbConfig& Config = CurbPtr->Config;
			if (!(Config.FrameFrontCm > Config.FrameBackCm) || !(Config.FrameHalfWidthCm > 0.0f))
			{
				return;
			}
			for (const double X : { static_cast<double>(Config.FrameBackCm), static_cast<double>(Config.FrameFrontCm) })
			{
				for (const double SignY : { -1.0, 1.0 })
				{
					FClearancePoint& Point = OutPoints.AddDefaulted_GetRef();
					Point.XY = FVector2D(X, SignY * Config.FrameHalfWidthCm);
					Point.UndersideZ = Config.BellyClearanceCm;
					Point.MinCm = Tuning.MinBellyClearanceCm;
				}
			}
		}

		void FGen1Plant::ApplyOffsets(TConstArrayView<double> Dz)
		{
			URammsRobotBaseComponent* BasePtr = Base.Get();
			const UObject*			  LayerPtr = Layer.Get();
			if (!BasePtr || !LayerPtr || Dz.Num() != Legs.Num())
			{
				return;
			}
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				FLiftLeg& Leg = Legs[Index];
				// (A leg that keeps the offset it had, below, counts as carrying
				// the law's: asking again each tick would change nothing.)
				Leg.bOffsetOn = Leg.bHasBaseline;
				if (!Leg.bHasBaseline)
				{
					// Nothing commands this lift: an offset left waiting would
					// be added to whatever next commands it.
					if (BasePtr->GetMotorCommandOffset(Leg.Motor) != 0.0f)
					{
						BasePtr->SetMotorCommandOffset(Leg.Motor, 0.0f, LayerPtr);
					}
					continue;
				}
				if (!FMath::IsFinite(Dz[Index]))
				{
					continue;
				}
				// Both looked up the same way, so that the table's own steps
				// cancel and no offset asked is exactly no offset given.
				float At = 0.0f, To = 0.0f;
				if (Leg.AngleFor(Leg.BaseHeight, At) && Leg.AngleFor(Leg.BaseHeight + Dz[Index], To))
				{
					BasePtr->SetMotorCommandOffset(Leg.Motor, To - At, LayerPtr);
				}
			}
			RememberOffsets();
		}

		void FGen1Plant::RememberOffsets()
		{
			if (const URammsRobotBaseComponent* BasePtr = Base.Get())
			{
				for (FLiftLeg& Leg : Legs)
				{
					Leg.LeftOffset = BasePtr->GetMotorCommandOffset(Leg.Motor);
				}
			}
		}

		bool FGen1Plant::OffsetsLost()
		{
			const URammsRobotBaseComponent* BasePtr = Base.Get();
			if (!BasePtr)
			{
				return false;
			}
			bool bLost = false;
			for (FLiftLeg& Leg : Legs)
			{
				if (!Leg.bHasBaseline)
				{
					// Let go: it carries nothing now, and is owed its offset
					// when it is next commanded.
					Leg.bOffsetOn = false;
					continue;
				}
				bLost |= !Leg.bOffsetOn || !SameLiftBits(BasePtr->GetMotorCommandOffset(Leg.Motor), Leg.LeftOffset);
			}
			return bLost;
		}

		void FGen1Plant::ZeroOffsets()
		{
			URammsRobotBaseComponent* BasePtr = Base.Get();
			const UObject*			  LayerPtr = Layer.Get();
			if (!BasePtr || !LayerPtr)
			{
				return;
			}
			for (const FLiftLeg& Leg : Legs)
			{
				// Only where one is on: a write is a push at the base's next
				// tick, and a motor already on its baseline needs none.
				if (BasePtr->GetMotorCommandOffset(Leg.Motor) != 0.0f)
				{
					BasePtr->SetMotorCommandOffset(Leg.Motor, 0.0f, LayerPtr);
				}
			}
			RememberOffsets();
		}

		void FGen1Plant::Fold()
		{
			URammsRobotBaseComponent* BasePtr = Base.Get();
			const UObject*			  LayerPtr = Layer.Get();
			if (!BasePtr || !LayerPtr)
			{
				return;
			}
			for (FLiftLeg& Leg : Legs)
			{
				Leg.bFolded = false;
				float Baseline = 0.0f, Effective = 0.0f;
				// A motor nothing commands has no baseline to fold into (the
				// base refuses the rebase): its offset is dropped instead.
				if (BasePtr->GetMotorCommand(Leg.Motor, Baseline) && BasePtr->GetMotorEffectiveCommand(Leg.Motor, Effective) && BasePtr->RebaseMotorCommand(Leg.Motor, Effective, LayerPtr))
				{
					Leg.PreFoldBaseline = Baseline;
					Leg.bFolded = BasePtr->GetMotorCommand(Leg.Motor, Leg.FoldedBaseline);
				}
				else if (BasePtr->GetMotorCommandOffset(Leg.Motor) != 0.0f)
				{
					BasePtr->SetMotorCommandOffset(Leg.Motor, 0.0f, LayerPtr);
				}
			}
			bFoldRemembered = true;
			RememberOffsets();
			SyncBaselineHolders();
		}

		bool FGen1Plant::Unfold()
		{
			URammsRobotBaseComponent* BasePtr = Base.Get();
			const UObject*			  LayerPtr = Layer.Get();
			if (!bFoldRemembered || !BasePtr || !LayerPtr)
			{
				return false;
			}
			// Only when every lift is exactly as the fold left it: a taker
			// that has written even one has started, and what it wrote stands.
			for (const FLiftLeg& Leg : Legs)
			{
				float	   Baseline = 0.0f;
				const bool bHas = BasePtr->GetMotorCommand(Leg.Motor, Baseline);
				if (bHas != Leg.bFolded || (bHas && !SameLiftBits(Baseline, Leg.FoldedBaseline)) || BasePtr->GetMotorCommandOffset(Leg.Motor) != 0.0f)
				{
					return false;
				}
			}
			// A baseline is put back only by whoever holds the offset layer:
			// with the layer given up since the fold the base refuses, and
			// then nothing may be reported undone. All or nothing: a refusal
			// part way takes back the ones already done.
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				const FLiftLeg& Leg = Legs[Index];
				if (Leg.bFolded && !BasePtr->RebaseMotorCommand(Leg.Motor, Leg.PreFoldBaseline, LayerPtr))
				{
					for (int32 Done = 0; Done < Index; ++Done)
					{
						if (Legs[Done].bFolded)
						{
							BasePtr->RebaseMotorCommand(Legs[Done].Motor, Legs[Done].FoldedBaseline, LayerPtr);
						}
					}
					return false;
				}
			}
			ForgetFold();
			RememberOffsets();
			SyncBaselineHolders();
			return true;
		}

		void FGen1Plant::ForgetFold()
		{
			bFoldRemembered = false;
			for (FLiftLeg& Leg : Legs)
			{
				Leg.bFolded = false;
			}
		}

		bool FGen1Plant::StanceHeights(const FRammsDriveStance& Stance, FLegValues& OutZ) const
		{
			OutZ.Reset();
			const URammsRammpGen1CurbController* CurbPtr = Curb.Get();
			if (!CurbPtr)
			{
				return false;
			}
			const AActor*				   Owner = CurbPtr->GetOwner();
			const URammsDriveModeSelector* Selector = Owner ? Owner->FindComponentByClass<URammsDriveModeSelector>() : nullptr;
			if (Selector && !Selector->bApplyLinkageStance)
			{
				// No stance is applied at all: there is nothing the lifts are
				// about to be taken to.
				return false;
			}
			for (const FLiftLeg& Leg : Legs)
			{
				// A lift the stance does not name stays where its baseline is.
				const FRammsStanceMotorTarget* Target = Stance.MotorTargets.FindByPredicate([&Leg](const FRammsStanceMotorTarget& Of) { return Of.MotorId == Leg.Motor; });
				OutZ.Add((Target ? Leg.HeightAt(Target->Target) : Leg.BaseHeight) + Leg.RadiusCm);
			}
			return true;
		}

		bool FGen1Plant::FoldedHeights(FLegValues& OutZ) const
		{
			OutZ.Reset();
			if (!bFoldRemembered)
			{
				return false;
			}
			for (const FLiftLeg& Leg : Legs)
			{
				if (!Leg.bFolded)
				{
					OutZ.Reset();
					return false;
				}
				OutZ.Add(Leg.HeightAt(Leg.PreFoldBaseline) + Leg.RadiusCm);
			}
			return true;
		}

		bool FGen1Plant::RebaseLegs(TConstArrayView<double> BaseZ, FLegValues& OutDz)
		{
			OutDz.Reset();
			URammsRobotBaseComponent* BasePtr = Base.Get();
			const UObject*			  LayerPtr = Layer.Get();
			if (!BasePtr || !LayerPtr || BaseZ.Num() != Legs.Num())
			{
				return false;
			}
			// What every lift is told now, and whether a baseline of it can be
			// moved at all (all of them or none: a plane out of some legs only
			// is no plane).
			TArray<float, TInlineAllocator<4>> Told;
			for (const FLiftLeg& Leg : Legs)
			{
				float Baseline = 0.0f, Effective = 0.0f;
				if (!BasePtr->GetMotorCommand(Leg.Motor, Baseline) || !BasePtr->GetMotorEffectiveCommand(Leg.Motor, Effective) || !BasePtr->RebaseMotorCommand(Leg.Motor, Baseline, LayerPtr))
				{
					return false;
				}
				Told.Add(Effective);
			}
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				const FLiftLeg& Leg = Legs[Index];
				float			Angle = 0.0f;
				// (A height past the lift's travel is taken to that end.)
				if (FMath::IsFinite(BaseZ[Index]) && Leg.AngleFor(BaseZ[Index] - Leg.RadiusCm, Angle))
				{
					BasePtr->RebaseMotorCommand(Leg.Motor, Angle, LayerPtr);
				}
			}
			// What that left on each leg, from the baselines as the base now
			// has them (it keeps a baseline within its motor's range).
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				FLiftLeg& Leg = Legs[Index];
				float	  Baseline = 0.0f;
				BasePtr->GetMotorCommand(Leg.Motor, Baseline);
				OutDz.Add(Leg.HeightAt(Told[Index]) - Leg.HeightAt(Baseline));
				// The offsets it now has are the ones it is meant to carry.
				Leg.bHasBaseline = true;
				Leg.bOffsetOn = true;
			}
			ForgetFold();
			RememberOffsets();
			SyncBaselineHolders();
			return true;
		}

		void FGen1Plant::FoldOrZero(int32& OutFolded, int32& OutZeroed)
		{
			OutFolded = OutZeroed = 0;
			URammsRobotBaseComponent* BasePtr = Base.Get();
			const UObject*			  LayerPtr = Layer.Get();
			if (!BasePtr || !LayerPtr)
			{
				return;
			}
			for (FLiftLeg& Leg : Legs)
			{
				if (BasePtr->GetMotorCommandOffset(Leg.Motor) == 0.0f)
				{
					continue;
				}
				// Left alone since the tick before: whoever took the lifts will
				// read this command as where the motor is, so the offset goes
				// into it. Rewritten: the new command is an absolute one, and
				// the offset on top of it would be an error.
				const bool bUntouched = Leg.bNow && Leg.bPrev && SameLiftBits(Leg.NowBaseline, Leg.PrevBaseline);
				float	   Effective = 0.0f;
				if (bUntouched && BasePtr->GetMotorEffectiveCommand(Leg.Motor, Effective) && BasePtr->RebaseMotorCommand(Leg.Motor, Effective, LayerPtr))
				{
					++OutFolded;
				}
				else
				{
					BasePtr->SetMotorCommandOffset(Leg.Motor, 0.0f, LayerPtr);
					++OutZeroed;
				}
			}
			// The baselines the next tick compares with are these.
			for (FLiftLeg& Leg : Legs)
			{
				Leg.bNow = BasePtr->GetMotorCommand(Leg.Motor, Leg.NowBaseline);
			}
			ForgetFold();
			RememberOffsets();
			SyncBaselineHolders();
		}

		void FGen1Plant::SyncBaselineHolders()
		{
			// The lift controller ramps each lift from where it last had it:
			// told the baselines moved under it, its next move starts from
			// where the motor is commanded now.
			const URammsRammpGen1CurbController* CurbPtr = Curb.Get();
			const AActor*						 Owner = CurbPtr ? CurbPtr->GetOwner() : nullptr;
			if (URammpGen1ControllerComponent* Lifts = Owner ? Owner->FindComponentByClass<URammpGen1ControllerComponent>() : nullptr)
			{
				Lifts->SyncTargetsToCommands();
			}
		}
	} // namespace

	TUniquePtr<IPlant> MakeGen1Plant(URammsRobotBaseComponent& Base, const URammsRammpGen1CurbController& Curb, const UObject* Layer)
	{
		return MakeUnique<FGen1Plant>(Base, Curb, Layer);
	}
} // namespace RammsLevel
