// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Level/RammsLevelTypes.h"
#include "Templates/UniquePtr.h"

class URammsCurbNegotiationController;
class URammsRobotBaseComponent;
struct FRammsDriveStance;

/**
 * The robot as the level assist's law sees it: a set of legs, each with a
 * height, a travel and a rate, and the motors those heights are made with.
 * The law (FLaw::Step) works in leg heights only; a plant turns the robot's
 * joints into those heights and the law's height offsets back into motor
 * command offsets (URammsRobotBaseComponent::SetMotorCommandOffset). One kind
 * of plant per kind of robot; the level assist's component picks the kind by
 * which sibling describes the robot. Plain C++, not reflected, not exported.
 *
 * A plant never writes a motor's own command (its baseline): only offsets,
 * and the fold and adopt that move a value between the two without the motor
 * being told anything.
 */
namespace RammsLevel
{
	using FPlantLegs = TArray<FLegReach, TInlineAllocator<8>>;
	using FPlantPoints = TArray<FClearancePoint, TInlineAllocator<8>>;

	class IPlant
	{
	public:
		virtual ~IPlant() = default;

		/** Find the robot's legs and motors. False, with why in plain words,
		 *  when the robot cannot be levelled (a motor missing or not a position
		 *  motor, a leg's linkage not found). Call once every component has
		 *  begun play. */
		virtual bool Resolve(FString& OutWhy) = 0;

		/** Every motor the plant puts an offset on: what the level assist
		 *  claims on the base. */
		virtual void LevelledMotorIds(TArray<FName>& OutIds) const = 0;

		/** The robot has a seat tilt the law's seat trim could move. */
		virtual bool HasSeat() const = 0;

		/**
		 * The legs now, in a fixed order: each one's place, its height at its
		 * baseline command and as its joints measure it, its travel and its
		 * rate (bSupport and BaselineClearanceCm are the law's to fill). Also
		 * notes each motor's baseline, for FoldOrZero. False when a leg's
		 * joints cannot be turned into a height (its linkage does not close):
		 * the law then holds.
		 */
		virtual bool MeasureLegs(FPlantLegs& OutLegs) = 0;

		/** The underside points the law keeps clear of the ground, each asking
		 *  Tuning.MinBellyClearanceCm. */
		virtual void ClearancePoints(const FTuning& Tuning, FPlantPoints& OutPoints) const = 0;

		/** Put the law's height offsets (cm, positive retracts; the legs of the
		 *  last MeasureLegs, in its order) on the motors. A leg that cannot
		 *  reach its height keeps the offset it had. */
		virtual void ApplyOffsets(TConstArrayView<double> Dz) = 0;

		/**
		 * A leg that has a baseline does not carry the offset ApplyOffsets last
		 * put on it: its motors were let go (which clears a motor's offset) and
		 * have been commanded again since, or something else cleared it. As of
		 * the last MeasureLegs. The offsets must then be applied again before
		 * the base next tells the motors, or that leg is told its command bare.
		 */
		virtual bool OffsetsLost() = 0;

		/** Every offset exactly zero. */
		virtual void ZeroOffsets() = 0;

		/** Fold: every motor's offset goes into its baseline (the motor told
		 *  nothing), for a controller about to take the legs over, and the
		 *  baselines from before are remembered. */
		virtual void Fold() = 0;

		/** Undo the last Fold exactly, when nothing has written a levelled
		 *  motor since: the remembered baselines back, the offsets as they
		 *  were. False, nothing changed, when something has, or when the
		 *  baselines cannot be put back (the offset layer is no longer held). */
		virtual bool Unfold() = 0;

		/** The remembered fold is no longer of use (the taker moved on). */
		virtual void ForgetFold() = 0;

		/**
		 * Adopt: take a plane back out of the baselines after a take-over, so
		 * that levelling goes on from the pose the taker left with no motion
		 * (against NextStance, the stance the legs are about to be ramped to;
		 * null: against the baselines remembered at the fold). False when the
		 * plant cannot: the level assist then starts again cold.
		 */
		virtual bool Adopt(const FRammsDriveStance* NextStance, FVector2D& OutPlaneDeg) = 0;

		/**
		 * The backstop for a take-over nobody announced: each motor whose
		 * baseline is as the last MeasureLegs but one found it has its offset
		 * folded into it; each whose baseline was rewritten since has its
		 * offset set to zero (its new command is then what it is told).
		 */
		virtual void FoldOrZero(int32& OutFolded, int32& OutZeroed) = 0;

		/** Tell whoever holds a target of its own for a levelled motor (a 5-bar
		 *  controller) that the baselines moved under it. */
		virtual void SyncBaselineHolders() = 0;
	};

	/**
	 * The plant of a robot whose wheels hang on legs that lift them (the Gen2
	 * kind): crank-driven 4-bars and 5-bars. Its geometry is not copied: the
	 * legs, their linkages and the belly plate are read from Curb, the sibling
	 * curb controller, which the robot's own asset script authors. Layer is
	 * who holds the base's command offset layer (the level assist).
	 */
	TUniquePtr<IPlant> MakeLegPlant(URammsRobotBaseComponent& Base, const URammsCurbNegotiationController& Curb, const UObject* Layer);
} // namespace RammsLevel
