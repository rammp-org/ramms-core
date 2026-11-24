// Copyright Epic Games, Inc. All Rights Reserved.

#include "Level/RammsLevelPlant.h"

#include "GameFramework/Actor.h"
#include "Ramms4BarKinematics.h"
#include "Ramms5BarKinematics.h"
#include "Ramms5BarLinkageController.h"
#include "RammsCurbNegotiationController.h"
#include "RammsMotorSpec.h"
#include "RammsRobotBaseComponent.h"
#include "UObject/WeakObjectPtr.h"

/**
 * The plant of the Gen2 kind of robot (RammsLevelPlant.h): corner wheels on
 * crank-driven 4-bars, centre wheels on 5-bars. Every length and limit is read
 * from the sibling curb controller's legs and from the motor registry; there
 * is no number of any robot in this file.
 *
 * A leg's height is its wheel centre's z in the chassis frame, by the leg's
 * own kinematics: a 4-bar's from its crank angle, a 5-bar's from its two hip
 * angles. "Baseline" heights come from the motors' baseline commands
 * (URammsRobotBaseComponent::GetMotorCommand), "measured" ones from the joints.
 * An offset of dz on a leg becomes, on its motors, the difference between the
 * angles that put the wheel at (baseline height + dz) and at the baseline
 * height, both solved the same way, so that dz = 0 is exactly no offset.
 *
 * SEAMS for later: the seat's tilt motor is not among the levelled motors yet
 * (the law works out a seat trim; nothing here applies it), and Adopt against
 * a stance is not built (a hand-back starts again cold).
 */
namespace RammsLevel
{
	namespace
	{
		/** A 5-bar's reach is found by walking its endpoint up and down in
		 *  steps of this (cm), this far at most. */
		constexpr double FiveBarScanStepCm = 0.25;
		constexpr double FiveBarScanMaxCm = 60.0;
		/** The reach is found again once the baseline has moved this far (cm)
		 *  fore or aft from where it was last found. */
		constexpr double FiveBarRescanCm = 0.05;
		/** How far either side of the baseline a leg's rate is measured over
		 *  (rad for a crank, cm for a 5-bar). */
		constexpr float	 CrankProbeRad = 0.01f;
		constexpr double FiveBarProbeCm = 0.5;

		bool SameBits(float A, float B)
		{
			return FMemory::Memcmp(&A, &B, sizeof(float)) == 0;
		}

		/** One levelled motor, and what is remembered of its baseline. */
		struct FPlantMotor
		{
			FName Id;
			/** Its ControlRange where its registry row sets one (X < Y). */
			FVector2D Range = FVector2D::ZeroVector;
			/** How fast its row lets it turn (rad/s); 0: no limit. */
			double MaxVelocity = 0.0;
			/** Its baseline at the last MeasureLegs and at the one before, and
			 *  whether it had one. */
			float NowBaseline = 0.0f;
			float PrevBaseline = 0.0f;
			bool  bNow = false;
			bool  bPrev = false;
			/** A fold: the baseline before it and the one it left. */
			float PreFoldBaseline = 0.0f;
			float FoldedBaseline = 0.0f;
			bool  bFolded = false;
			/** The offset this plant last left on it, as the base read it
			 *  back: what OffsetsLost compares with. */
			float LeftOffset = 0.0f;
		};

		struct FPlantLeg
		{
			/** Which of the curb controller's legs it is. */
			int32 CurbIndex = INDEX_NONE;
			bool  bFourBar = true;
			/** Its motors (indices into the plant's motors): a crank, or hips
			 *  A and B. */
			int32 Motors[2] = { INDEX_NONE, INDEX_NONE };
			int32 NumMotors = 0;
			/** A 5-bar leg's controller. */
			TWeakObjectPtr<URamms5BarLinkageController> FiveBar;
			/** A 4-bar's travel (cm), found once. */
			FVector2D HeightRange = FVector2D::ZeroVector;

			// ---- as the last MeasureLegs found it ----
			/** Every motor of it has a baseline: an offset can be worked out. */
			bool bHasBaseline = false;
			/** ApplyOffsets has put the law's offset on it since it last had
			 *  no baseline. */
			bool bOffsetOn = false;
			/** The baseline height, and for a 5-bar the baseline endpoint in
			 *  the linkage's own frame. */
			double	  BaseZ = 0.0;
			FVector2D BaseEndpoint = FVector2D::ZeroVector;
			double	  MinZ = 0.0;
			double	  MaxZ = 0.0;
			/** A 5-bar's reach as last scanned, and where (its own frame's x). */
			bool	  bScanned = false;
			double	  ScannedAtX = 0.0;
			FVector2D ScannedRange = FVector2D::ZeroVector;
		};

		class FLegPlant final : public IPlant
		{
		public:
			FLegPlant(URammsRobotBaseComponent& InBase, const URammsCurbNegotiationController& InCurb, const UObject* InLayer)
				: Base(&InBase)
				, Curb(&InCurb)
				, Layer(InLayer)
			{
			}

			virtual bool Resolve(FString& OutWhy) override;
			virtual void LevelledMotorIds(TArray<FName>& OutIds) const override;
			virtual bool HasSeat() const override;
			virtual bool MeasureLegs(FPlantLegs& OutLegs) override;
			virtual void ClearancePoints(const FTuning& Tuning, FPlantPoints& OutPoints) const override;
			virtual void ApplyOffsets(TConstArrayView<double> Dz) override;
			virtual bool OffsetsLost() override;
			virtual void ZeroOffsets() override;
			virtual void Fold() override;
			virtual bool Unfold() override;
			virtual void ForgetFold() override;
			virtual bool Adopt(const FRammsDriveStance* NextStance, FVector2D& OutPlaneDeg) override;
			virtual void FoldOrZero(int32& OutFolded, int32& OutZeroed) override;
			virtual void SyncBaselineHolders() override;

		private:
			int32 AddMotor(FName Id, FString& OutWhy);
			bool  WithinRange(const FPlantMotor& Motor, double Angle) const;
			/** A 5-bar's hips for an endpoint (its own frame), within their
			 *  motors' ranges. */
			bool	  SolveHips(const FPlantLeg& Leg, const FRamms5BarLinkageSpec& Spec, const FVector2D& Endpoint, FVector2D& OutHips) const;
			FVector2D ScanFiveBar(const FPlantLeg& Leg, const FRamms5BarLinkageSpec& Spec, const FVector2D& Endpoint) const;
			/** Note every motor's offset as it now stands (after this plant
			 *  changed them), for OffsetsLost. */
			void RememberOffsets();

			TWeakObjectPtr<URammsRobotBaseComponent>			  Base;
			TWeakObjectPtr<const URammsCurbNegotiationController> Curb;
			TWeakObjectPtr<const UObject>						  Layer;
			TArray<FPlantMotor>									  Motors;
			TArray<FPlantLeg>									  Legs;
			bool												  bFoldRemembered = false;
		};

		int32 FLegPlant::AddMotor(FName Id, FString& OutWhy)
		{
			FRammsMotorSpec Spec;
			if (Id.IsNone() || !Base->GetMotorSpec(Id, Spec))
			{
				OutWhy = FString::Printf(TEXT("the leg motor '%s' is not in the robot's motor table"), *Id.ToString());
				return INDEX_NONE;
			}
			if (Spec.Type != ERammsActuatorType::Position)
			{
				OutWhy = FString::Printf(TEXT("the leg motor '%s' is not a position motor"), *Id.ToString());
				return INDEX_NONE;
			}
			FPlantMotor& Motor = Motors.AddDefaulted_GetRef();
			Motor.Id = Id;
			Motor.Range = Spec.ControlRange;
			Motor.MaxVelocity = FMath::Max(0.0f, Spec.MaxVelocity);
			return Motors.Num() - 1;
		}

		bool FLegPlant::Resolve(FString& OutWhy)
		{
			Motors.Reset();
			Legs.Reset();
			bFoldRemembered = false;
			const URammsCurbNegotiationController* CurbPtr = Curb.Get();
			if (!Base.IsValid() || !CurbPtr)
			{
				OutWhy = TEXT("no robot description");
				return false;
			}
			if (CurbPtr->Legs.IsEmpty())
			{
				OutWhy = TEXT("the robot's description names no legs");
				return false;
			}
			const AActor* Owner = CurbPtr->GetOwner();
			for (int32 Index = 0; Index < CurbPtr->Legs.Num(); ++Index)
			{
				const FRammsCurbLeg& CurbLeg = CurbPtr->Legs[Index];
				FPlantLeg			 Leg;
				Leg.CurbIndex = Index;
				Leg.bFourBar = CurbLeg.Kind == ERammsCurbLegKind::FourBar;
				if (Leg.bFourBar)
				{
					Leg.Motors[0] = AddMotor(CurbLeg.FourBar.CrankMotor, OutWhy);
					Leg.NumMotors = 1;
					if (Leg.Motors[0] == INDEX_NONE)
					{
						return false;
					}
					Leg.HeightRange = URamms4BarKinematics::GetHeightRange(CurbLeg.FourBar);
					if (!(Leg.HeightRange.X < Leg.HeightRange.Y))
					{
						OutWhy = FString::Printf(TEXT("the leg '%s' has no travel"), *CurbLeg.Name.ToString());
						return false;
					}
				}
				else
				{
					URamms5BarLinkageController* Found = nullptr;
					if (Owner)
					{
						for (UActorComponent* Component : Owner->GetComponents())
						{
							URamms5BarLinkageController* FiveBar = Cast<URamms5BarLinkageController>(Component);
							if (FiveBar && FiveBar->GetFName() == CurbLeg.FiveBarComponent)
							{
								Found = FiveBar;
								break;
							}
						}
					}
					if (!Found)
					{
						OutWhy = FString::Printf(TEXT("the leg '%s' names a linkage, '%s', that is not on the robot"), *CurbLeg.Name.ToString(),
							*CurbLeg.FiveBarComponent.ToString());
						return false;
					}
					Leg.FiveBar = Found;
					const FRamms5BarLinkageSpec& Spec = Found->GetResolvedLinkage();
					Leg.Motors[0] = AddMotor(Spec.ProximalMotorA, OutWhy);
					Leg.Motors[1] = Leg.Motors[0] != INDEX_NONE ? AddMotor(Spec.ProximalMotorB, OutWhy) : INDEX_NONE;
					Leg.NumMotors = 2;
					if (Leg.Motors[0] == INDEX_NONE || Leg.Motors[1] == INDEX_NONE)
					{
						return false;
					}
				}
				Legs.Add(Leg);
			}
			return true;
		}

		void FLegPlant::LevelledMotorIds(TArray<FName>& OutIds) const
		{
			for (const FPlantMotor& Motor : Motors)
			{
				OutIds.AddUnique(Motor.Id);
			}
		}

		bool FLegPlant::HasSeat() const
		{
			const URammsCurbNegotiationController* CurbPtr = Curb.Get();
			return CurbPtr && CurbPtr->Seat.IsSet() && !CurbPtr->Seat.TiltMotor.IsNone();
		}

		bool FLegPlant::WithinRange(const FPlantMotor& Motor, double Angle) const
		{
			// A row with no range (zero width) leaves the limit to the backend.
			return !(Motor.Range.X < Motor.Range.Y) || (Angle >= Motor.Range.X && Angle <= Motor.Range.Y);
		}

		bool FLegPlant::SolveHips(const FPlantLeg& Leg, const FRamms5BarLinkageSpec& Spec, const FVector2D& Endpoint, FVector2D& OutHips) const
		{
			bool			bReachable = false;
			const FVector2D Hips = URamms5BarKinematics::SolveIK(Spec, Endpoint, bReachable);
			if (!bReachable || !WithinRange(Motors[Leg.Motors[0]], Hips.X) || !WithinRange(Motors[Leg.Motors[1]], Hips.Y))
			{
				return false;
			}
			OutHips = Hips;
			return true;
		}

		FVector2D FLegPlant::ScanFiveBar(const FPlantLeg& Leg, const FRamms5BarLinkageSpec& Spec, const FVector2D& Endpoint) const
		{
			// Walk out from where the endpoint is until the linkage, or a hip's
			// range, stops it: the reach at this fore/aft position. An endpoint
			// that is itself out of reach has none.
			FVector2D Range(Endpoint.Y, Endpoint.Y);
			FVector2D Hips;
			if (!SolveHips(Leg, Spec, Endpoint, Hips))
			{
				return Range;
			}
			const int32 MaxSteps = FMath::CeilToInt32(FiveBarScanMaxCm / FiveBarScanStepCm);
			for (int32 Step = 1; Step <= MaxSteps; ++Step)
			{
				const double Z = Endpoint.Y - Step * FiveBarScanStepCm;
				if (!SolveHips(Leg, Spec, FVector2D(Endpoint.X, Z), Hips))
				{
					break;
				}
				Range.X = Z;
			}
			for (int32 Step = 1; Step <= MaxSteps; ++Step)
			{
				const double Z = Endpoint.Y + Step * FiveBarScanStepCm;
				if (!SolveHips(Leg, Spec, FVector2D(Endpoint.X, Z), Hips))
				{
					break;
				}
				Range.Y = Z;
			}
			return Range;
		}

		bool FLegPlant::MeasureLegs(FPlantLegs& OutLegs)
		{
			OutLegs.Reset();
			URammsRobotBaseComponent*			   BasePtr = Base.Get();
			const URammsCurbNegotiationController* CurbPtr = Curb.Get();
			if (!BasePtr || !CurbPtr)
			{
				return false;
			}
			// The baselines as they stand, and as they stood a tick ago: what
			// FoldOrZero tells a rewritten motor from an untouched one by.
			for (FPlantMotor& Motor : Motors)
			{
				Motor.PrevBaseline = Motor.NowBaseline;
				Motor.bPrev = Motor.bNow;
				Motor.bNow = BasePtr->GetMotorCommand(Motor.Id, Motor.NowBaseline);
			}
			const double LegSpeed = FMath::Max(0.0f, CurbPtr->Tuning.LegSpeed);
			bool		 bAll = true;
			for (FPlantLeg& Leg : Legs)
			{
				if (!CurbPtr->Legs.IsValidIndex(Leg.CurbIndex))
				{
					return false;
				}
				const FRammsCurbLeg& CurbLeg = CurbPtr->Legs[Leg.CurbIndex];
				FLegReach			 Reach;
				Reach.RadiusCm = CurbLeg.WheelRadiusCm;
				// The fastest the leg's height may be asked to move: what the
				// curb's own leg moves are made at, and what the motors' rows
				// let their joints turn at, whichever is less.
				double RateCmS = LegSpeed;
				auto   LimitRate = [&RateCmS](double Limit) {
					  if (Limit > 0.0)
					  {
						  RateCmS = RateCmS > 0.0 ? FMath::Min(RateCmS, Limit) : Limit;
					  }
				};
				if (Leg.bFourBar)
				{
					const FRamms4BarLegSpec& Spec = CurbLeg.FourBar;
					const FPlantMotor&		 Crank = Motors[Leg.Motors[0]];
					const float				 Lo = FMath::Min(Spec.CrankRange.X, Spec.CrankRange.Y);
					const float				 Hi = FMath::Max(Spec.CrankRange.X, Spec.CrankRange.Y);
					const float				 Joint = FMath::Clamp(BasePtr->GetMotorValue(Crank.Id), Lo, Hi);
					Leg.bHasBaseline = Crank.bNow;
					// A crank nothing has commanded stands where its joint is.
					const float		BaseCrank = Crank.bNow ? FMath::Clamp(Crank.NowBaseline, Lo, Hi) : Joint;
					bool			bBase = false, bJoint = false;
					const FVector2D BaseCentre = URamms4BarKinematics::ComputeWheelCenter(Spec, BaseCrank, bBase);
					const FVector2D JointCentre = URamms4BarKinematics::ComputeWheelCenter(Spec, Joint, bJoint);
					if (!bBase || !bJoint)
					{
						bAll = false;
						continue;
					}
					Leg.BaseZ = BaseCentre.Y;
					Leg.MinZ = FMath::Min(Leg.HeightRange.X, Leg.BaseZ);
					Leg.MaxZ = FMath::Max(Leg.HeightRange.Y, Leg.BaseZ);
					Reach.XY = FVector2D(BaseCentre.X, CurbLeg.Wheel.Position.Y);
					Reach.BaseZ = Leg.BaseZ;
					Reach.MeasuredZ = JointCentre.Y;
					const double CrankRate = Crank.MaxVelocity > 0.0 ? Crank.MaxVelocity : static_cast<double>(CurbLeg.MaxCrankRateRadS);
					if (CrankRate > 0.0)
					{
						// How far the wheel rises per radian of crank, here.
						const float		A = FMath::Clamp(BaseCrank - CrankProbeRad, Lo, Hi);
						const float		B = FMath::Clamp(BaseCrank + CrankProbeRad, Lo, Hi);
						bool			bA = false, bB = false;
						const FVector2D CentreA = URamms4BarKinematics::ComputeWheelCenter(Spec, A, bA);
						const FVector2D CentreB = URamms4BarKinematics::ComputeWheelCenter(Spec, B, bB);
						if (bA && bB && B > A)
						{
							LimitRate(CrankRate * FMath::Abs(CentreB.Y - CentreA.Y) / static_cast<double>(B - A));
						}
					}
				}
				else
				{
					URamms5BarLinkageController* FiveBar = Leg.FiveBar.Get();
					if (!FiveBar)
					{
						return false;
					}
					const FRamms5BarLinkageSpec& Spec = FiveBar->GetResolvedLinkage();
					const double				 Hand = Spec.TranslationSign < 0.0f ? -1.0 : 1.0;
					const FPlantMotor&			 HipA = Motors[Leg.Motors[0]];
					const FPlantMotor&			 HipB = Motors[Leg.Motors[1]];
					bool						 bJoint = false;
					// The controller's endpoints are in the robot's frame.
					const FVector2D JointRobot = FiveBar->GetCurrentEndpointChecked(bJoint);
					const FVector2D JointEndpoint(JointRobot.X * Hand, JointRobot.Y);
					Leg.bHasBaseline = HipA.bNow && HipB.bNow;
					bool	  bBase = bJoint;
					FVector2D BaseEndpoint = JointEndpoint;
					if (Leg.bHasBaseline)
					{
						BaseEndpoint = URamms5BarKinematics::ComputeEndpoint(Spec, FVector2D(HipA.NowBaseline, HipB.NowBaseline), bBase);
					}
					if (!bBase || !bJoint)
					{
						bAll = false;
						continue;
					}
					Leg.BaseEndpoint = BaseEndpoint;
					Leg.BaseZ = BaseEndpoint.Y;
					if (!Leg.bScanned || FMath::Abs(BaseEndpoint.X - Leg.ScannedAtX) > FiveBarRescanCm)
					{
						Leg.ScannedRange = ScanFiveBar(Leg, Spec, BaseEndpoint);
						Leg.ScannedAtX = BaseEndpoint.X;
						Leg.bScanned = true;
					}
					Leg.MinZ = FMath::Min(Leg.ScannedRange.X, Leg.BaseZ);
					Leg.MaxZ = FMath::Max(Leg.ScannedRange.Y, Leg.BaseZ);
					Reach.XY = FVector2D(BaseEndpoint.X * Hand, CurbLeg.Wheel.Position.Y);
					Reach.BaseZ = Leg.BaseZ;
					Reach.MeasuredZ = JointEndpoint.Y;
					if (HipA.MaxVelocity > 0.0 || HipB.MaxVelocity > 0.0)
					{
						// How far each hip turns per cm of height, here.
						FVector2D Below, Above;
						if (SolveHips(Leg, Spec, BaseEndpoint - FVector2D(0.0, FiveBarProbeCm), Below)
							&& SolveHips(Leg, Spec, BaseEndpoint + FVector2D(0.0, FiveBarProbeCm), Above))
						{
							const FVector2D PerCm = (Above - Below) / (2.0 * FiveBarProbeCm);
							if (HipA.MaxVelocity > 0.0 && FMath::Abs(PerCm.X) > UE_DOUBLE_SMALL_NUMBER)
							{
								LimitRate(HipA.MaxVelocity / FMath::Abs(PerCm.X));
							}
							if (HipB.MaxVelocity > 0.0 && FMath::Abs(PerCm.Y) > UE_DOUBLE_SMALL_NUMBER)
							{
								LimitRate(HipB.MaxVelocity / FMath::Abs(PerCm.Y));
							}
						}
					}
				}
				Reach.MinZ = Leg.MinZ;
				Reach.MaxZ = Leg.MaxZ;
				Reach.MaxRateCmS = RateCmS;
				Reach.bHasBaseline = Leg.bHasBaseline;
				OutLegs.Add(Reach);
			}
			if (!bAll)
			{
				OutLegs.Reset();
			}
			return bAll;
		}

		void FLegPlant::ClearancePoints(const FTuning& Tuning, FPlantPoints& OutPoints) const
		{
			OutPoints.Reset();
			const URammsCurbNegotiationController* CurbPtr = Curb.Get();
			if (!CurbPtr)
			{
				return;
			}
			// The belly plate, as the curb controller takes it: the authored
			// half extent, or, for an axis left 0, what the corner crank pivots
			// (fore and aft) and the corner wheels (across) span.
			FVector2D Half = CurbPtr->Tuning.ChassisHalfExtent;
			FVector2D Spanned = FVector2D::ZeroVector;
			for (const FRammsCurbLeg& CurbLeg : CurbPtr->Legs)
			{
				if (CurbLeg.Kind == ERammsCurbLegKind::FourBar && CurbLeg.Axle != ERammsCurbAxle::Center)
				{
					Spanned.X = FMath::Max(Spanned.X, FMath::Abs(CurbLeg.FourBar.CrankPivot.X));
					Spanned.Y = FMath::Max(Spanned.Y, FMath::Abs(CurbLeg.Wheel.Position.Y));
				}
			}
			Half.X = Half.X > 0.0 ? Half.X : Spanned.X;
			Half.Y = Half.Y > 0.0 ? Half.Y : Spanned.Y;
			if (!(Half.X > 0.0) || !(Half.Y > 0.0))
			{
				return;
			}
			for (const double SignX : { -1.0, 1.0 })
			{
				for (const double SignY : { -1.0, 1.0 })
				{
					FClearancePoint& Point = OutPoints.AddDefaulted_GetRef();
					Point.XY = FVector2D(SignX * Half.X, SignY * Half.Y);
					Point.UndersideZ = CurbPtr->Tuning.ChassisUnderside;
					Point.MinCm = Tuning.MinBellyClearanceCm;
				}
			}
		}

		void FLegPlant::ApplyOffsets(TConstArrayView<double> Dz)
		{
			URammsRobotBaseComponent*			   BasePtr = Base.Get();
			const URammsCurbNegotiationController* CurbPtr = Curb.Get();
			const UObject*						   LayerPtr = Layer.Get();
			if (!BasePtr || !CurbPtr || !LayerPtr || Dz.Num() != Legs.Num())
			{
				return;
			}
			for (int32 Index = 0; Index < Legs.Num(); ++Index)
			{
				FPlantLeg& Leg = Legs[Index];
				// (A leg that keeps the offset it had, below, counts as carrying
				// the law's: asking again each tick would change nothing.)
				Leg.bOffsetOn = Leg.bHasBaseline;
				if (!Leg.bHasBaseline)
				{
					// Nothing commands this leg (let go for driving by hand):
					// an offset left waiting would be added to whatever next
					// commands it.
					for (int32 Motor = 0; Motor < Leg.NumMotors; ++Motor)
					{
						if (BasePtr->GetMotorCommandOffset(Motors[Leg.Motors[Motor]].Id) != 0.0f)
						{
							BasePtr->SetMotorCommandOffset(Motors[Leg.Motors[Motor]].Id, 0.0f, LayerPtr);
						}
					}
					continue;
				}
				if (!FMath::IsFinite(Dz[Index]) || !CurbPtr->Legs.IsValidIndex(Leg.CurbIndex))
				{
					continue;
				}
				const double Z = FMath::Clamp(Leg.BaseZ + Dz[Index], Leg.MinZ, Leg.MaxZ);
				if (Leg.bFourBar)
				{
					const FRamms4BarLegSpec& Spec = CurbPtr->Legs[Leg.CurbIndex].FourBar;
					// Both solved the same way, so that the solver's own error
					// cancels and no offset asked is exactly no offset given. (A
					// height at the very end of the travel solves to that end.)
					bool		bReachable = false;
					float		WheelX = 0.0f;
					const float At = URamms4BarKinematics::SolveCrankForHeight(Spec, static_cast<float>(Leg.BaseZ), bReachable, WheelX);
					const float To = URamms4BarKinematics::SolveCrankForHeight(Spec, static_cast<float>(Z), bReachable, WheelX);
					BasePtr->SetMotorCommandOffset(Motors[Leg.Motors[0]].Id, To - At, LayerPtr);
				}
				else if (const URamms5BarLinkageController* FiveBar = Leg.FiveBar.Get())
				{
					const FRamms5BarLinkageSpec& Spec = FiveBar->GetResolvedLinkage();
					FVector2D					 At, To;
					// A height the linkage cannot make counts as the leg at its
					// limit: it keeps the offset it had.
					if (SolveHips(Leg, Spec, Leg.BaseEndpoint, At) && SolveHips(Leg, Spec, FVector2D(Leg.BaseEndpoint.X, Z), To))
					{
						BasePtr->SetMotorCommandOffset(Motors[Leg.Motors[0]].Id, static_cast<float>(To.X - At.X), LayerPtr);
						BasePtr->SetMotorCommandOffset(Motors[Leg.Motors[1]].Id, static_cast<float>(To.Y - At.Y), LayerPtr);
					}
				}
			}
			RememberOffsets();
		}

		void FLegPlant::RememberOffsets()
		{
			if (const URammsRobotBaseComponent* BasePtr = Base.Get())
			{
				for (FPlantMotor& Motor : Motors)
				{
					Motor.LeftOffset = BasePtr->GetMotorCommandOffset(Motor.Id);
				}
			}
		}

		bool FLegPlant::OffsetsLost()
		{
			const URammsRobotBaseComponent* BasePtr = Base.Get();
			if (!BasePtr)
			{
				return false;
			}
			bool bLost = false;
			for (FPlantLeg& Leg : Legs)
			{
				if (!Leg.bHasBaseline)
				{
					// Let go: it carries nothing now, and is owed its offset
					// when it is next commanded.
					Leg.bOffsetOn = false;
					continue;
				}
				bLost |= !Leg.bOffsetOn;
				for (int32 Motor = 0; Motor < Leg.NumMotors; ++Motor)
				{
					const FPlantMotor& Of = Motors[Leg.Motors[Motor]];
					bLost |= !SameBits(BasePtr->GetMotorCommandOffset(Of.Id), Of.LeftOffset);
				}
			}
			return bLost;
		}

		void FLegPlant::ZeroOffsets()
		{
			URammsRobotBaseComponent* BasePtr = Base.Get();
			const UObject*			  LayerPtr = Layer.Get();
			if (!BasePtr || !LayerPtr)
			{
				return;
			}
			for (const FPlantMotor& Motor : Motors)
			{
				// Only where one is on: a write is a push at the base's next
				// tick, and a motor already on its baseline needs none.
				if (BasePtr->GetMotorCommandOffset(Motor.Id) != 0.0f)
				{
					BasePtr->SetMotorCommandOffset(Motor.Id, 0.0f, LayerPtr);
				}
			}
			RememberOffsets();
		}

		void FLegPlant::Fold()
		{
			URammsRobotBaseComponent* BasePtr = Base.Get();
			const UObject*			  LayerPtr = Layer.Get();
			if (!BasePtr || !LayerPtr)
			{
				return;
			}
			for (FPlantMotor& Motor : Motors)
			{
				Motor.bFolded = false;
				float Baseline = 0.0f, Effective = 0.0f;
				// A motor nothing commands has no baseline to fold into (the
				// base refuses the rebase): its offset is dropped instead.
				if (BasePtr->GetMotorCommand(Motor.Id, Baseline) && BasePtr->GetMotorEffectiveCommand(Motor.Id, Effective)
					&& BasePtr->RebaseMotorCommand(Motor.Id, Effective, LayerPtr))
				{
					Motor.PreFoldBaseline = Baseline;
					Motor.bFolded = BasePtr->GetMotorCommand(Motor.Id, Motor.FoldedBaseline);
				}
				else if (BasePtr->GetMotorCommandOffset(Motor.Id) != 0.0f)
				{
					BasePtr->SetMotorCommandOffset(Motor.Id, 0.0f, LayerPtr);
				}
			}
			bFoldRemembered = true;
			RememberOffsets();
			SyncBaselineHolders();
		}

		bool FLegPlant::Unfold()
		{
			URammsRobotBaseComponent* BasePtr = Base.Get();
			const UObject*			  LayerPtr = Layer.Get();
			if (!bFoldRemembered || !BasePtr || !LayerPtr)
			{
				return false;
			}
			// Only when every motor is exactly as the fold left it: a taker
			// that has written even one has started, and what it wrote stands.
			for (const FPlantMotor& Motor : Motors)
			{
				float	   Baseline = 0.0f;
				const bool bHas = BasePtr->GetMotorCommand(Motor.Id, Baseline);
				if (bHas != Motor.bFolded || (bHas && !SameBits(Baseline, Motor.FoldedBaseline)) || BasePtr->GetMotorCommandOffset(Motor.Id) != 0.0f)
				{
					return false;
				}
			}
			// A baseline is put back only by whoever holds the offset layer:
			// with the layer given up since the fold the base refuses, and
			// then nothing may be reported undone. All or nothing: a refusal
			// part way takes back the ones already done.
			for (int32 Index = 0; Index < Motors.Num(); ++Index)
			{
				const FPlantMotor& Motor = Motors[Index];
				if (Motor.bFolded && !BasePtr->RebaseMotorCommand(Motor.Id, Motor.PreFoldBaseline, LayerPtr))
				{
					for (int32 Done = 0; Done < Index; ++Done)
					{
						if (Motors[Done].bFolded)
						{
							BasePtr->RebaseMotorCommand(Motors[Done].Id, Motors[Done].FoldedBaseline, LayerPtr);
						}
					}
					return false;
				}
			}
			for (FPlantMotor& Motor : Motors)
			{
				Motor.bFolded = false;
			}
			bFoldRemembered = false;
			RememberOffsets();
			SyncBaselineHolders();
			return true;
		}

		void FLegPlant::ForgetFold()
		{
			bFoldRemembered = false;
			for (FPlantMotor& Motor : Motors)
			{
				Motor.bFolded = false;
			}
		}

		bool FLegPlant::Adopt(const FRammsDriveStance* NextStance, FVector2D& OutPlaneDeg)
		{
			// Not built yet: a hand-back starts again cold, the drive mode's
			// stance ramp un-levels the base and the law levels it again.
			return false;
		}

		void FLegPlant::FoldOrZero(int32& OutFolded, int32& OutZeroed)
		{
			OutFolded = OutZeroed = 0;
			URammsRobotBaseComponent* BasePtr = Base.Get();
			const UObject*			  LayerPtr = Layer.Get();
			if (!BasePtr || !LayerPtr)
			{
				return;
			}
			for (FPlantMotor& Motor : Motors)
			{
				if (BasePtr->GetMotorCommandOffset(Motor.Id) == 0.0f)
				{
					continue;
				}
				// Left alone since the tick before: whoever took the legs will
				// read this command as where the motor is, so the offset goes
				// into it. Rewritten: the new command is an absolute one, and
				// the offset on top of it would be an error.
				const bool bUntouched = Motor.bNow && Motor.bPrev && SameBits(Motor.NowBaseline, Motor.PrevBaseline);
				float	   Effective = 0.0f;
				if (bUntouched && BasePtr->GetMotorEffectiveCommand(Motor.Id, Effective) && BasePtr->RebaseMotorCommand(Motor.Id, Effective, LayerPtr))
				{
					++OutFolded;
				}
				else
				{
					BasePtr->SetMotorCommandOffset(Motor.Id, 0.0f, LayerPtr);
					++OutZeroed;
				}
			}
			// The baselines the next tick compares with are these.
			for (FPlantMotor& Motor : Motors)
			{
				Motor.bNow = BasePtr->GetMotorCommand(Motor.Id, Motor.NowBaseline);
			}
			bFoldRemembered = false;
			RememberOffsets();
			SyncBaselineHolders();
		}

		void FLegPlant::SyncBaselineHolders()
		{
			for (const FPlantLeg& Leg : Legs)
			{
				if (URamms5BarLinkageController* FiveBar = Leg.FiveBar.Get())
				{
					FiveBar->SyncTargetToCommands();
				}
			}
		}
	} // namespace

	TUniquePtr<IPlant> MakeLegPlant(URammsRobotBaseComponent& Base, const URammsCurbNegotiationController& Curb, const UObject* Layer)
	{
		return MakeUnique<FLegPlant>(Base, Curb, Layer);
	}
} // namespace RammsLevel
