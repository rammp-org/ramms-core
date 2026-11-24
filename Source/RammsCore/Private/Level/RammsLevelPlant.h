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

		/** The robot has a seat tilt the law's seat trim can move: its motor
		 *  is among the levelled ones. */
		virtual bool HasSeat() const = 0;

		/** The seat's tilt motor, where HasSeat; none otherwise. (It is
		 *  levelled, but it is no leg: a drive mode that drives it has not
		 *  taken the legs.) */
		virtual FName SeatMotorId() const = 0;

		/**
		 * Where the seat stands, as of the last MeasureLegs: the pitch its own
		 * commands hold the platform at against the chassis (deg, nose up:
		 * the rider's own setting, with no trim), and how far the trim may
		 * take it from there, down to OutLowDeg (0 or less) and up to
		 * OutHighDeg (0 or more), by the platform's own range and its motor's.
		 * False when there is no seat, or it cannot take a trim now (nothing
		 * commands its tilt yet; its linkage does not close): the law is then
		 * told there is none, and any trim on it is taken off.
		 */
		virtual bool SeatTrimRange(double& OutOwnDeg, double& OutLowDeg, double& OutHighDeg) = 0;

		/** Put the law's seat trim (deg, nose up against the chassis) on the
		 *  seat's tilt motor, as an offset over the seat's own command. */
		virtual void ApplySeatTrim(double TrimDeg) = 0;

		/** The seat trim (deg) that FoldSeatTrim has put into the seat tilt's
		 *  own command and that is in it still: 0 once the tilt's command is no
		 *  longer the one the fold left, or it was taken back out (UnfoldSeatTrim). What a
		 *  taker must take off the pitch it reads back to have the rider's own
		 *  setting. */
		virtual double FoldedSeatTrimDeg() const = 0;

		/** The trim (deg, nose up) the seat's tilt motor carries now as an
		 *  offset over its own command, as the base has it: what ApplySeatTrim
		 *  last put there, or what a hand-back took back out of the command
		 *  (UnfoldSeatTrim). 0 with no seat. */
		virtual double SeatTrimOnDeg() const = 0;

		/**
		 * A taker that had the seat's trim folded for it (FoldSeatTrim) hands
		 * back without having commanded the seat's tilt after all: the trim is
		 * in the tilt's own command still (FoldedSeatTrimDeg), where nothing
		 * will take it out again -- the rider's own setting would read that
		 * much off, and levelling would add the trim a second time. Taken back out here: the
		 * tilt's command is the rider's own again and the trim is its offset,
		 * the motor told nothing. False, nothing changed, when no trim is in
		 * the command any more (something has commanded the tilt since), there
		 * is no seat, or the offset layer is not held.
		 */
		virtual bool UnfoldSeatTrim() = 0;

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
		 * A leg that has a baseline (or the seat) does not carry the offset
		 * ApplyOffsets (ApplySeatTrim) last put on it: its motors were let go (which clears a motor's offset) and
		 * have been commanded again since, or something else cleared it. As of
		 * the last MeasureLegs. The offsets must then be applied again before
		 * the base next tells the motors, or that leg is told its command bare.
		 */
		virtual bool OffsetsLost() = 0;

		/** Every offset exactly zero. */
		virtual void ZeroOffsets() = 0;

		/** Every LEG motor's offset exactly zero; a trim on the seat stays
		 *  (the rider's attitude is not dropped in one write where the legs'
		 *  correction is given up: the level assist walks it off). */
		virtual void ZeroLegOffsets() = 0;

		/** Fold: every LEG motor's offset goes into its baseline (the motor
		 *  told nothing), for a controller about to take the legs over, and
		 *  the baselines from before are remembered. The seat's trim stays on
		 *  the seat: taking the legs is not taking the seat (FoldSeatTrim). */
		virtual void Fold() = 0;

		/** The seat's trim goes into the seat tilt's own command, the motor
		 *  told nothing, for a controller about to command the seat's tilt
		 *  with an absolute command of its own; FoldedSeatTrimDeg then says
		 *  how much is in the command. Nothing with no trim on, or no seat. */
		virtual void FoldSeatTrim() = 0;

		/** Undo the last Fold exactly, when nothing has written a levelled
		 *  motor since: the remembered baselines back, the offsets as they
		 *  were. False, nothing changed, when something has, or when the
		 *  baselines cannot be put back (the offset layer is no longer held). */
		virtual bool Unfold() = 0;

		/** The remembered fold is no longer of use (the taker moved on). */
		virtual void ForgetFold() = 0;

		// ---- adopting a pose after a take-over ----
		// A taker hands the legs back in a pose of its own. The part of that
		// pose that is levelling (PlanAdoption: the plane by which it differs
		// from the stance the legs go to next) is taken back out of the
		// baselines and becomes the offsets, the motors told nothing; the three
		// below are the plant's part of that. When any of them cannot, the
		// level assist starts again from nothing on the pose as it stands.

		/**
		 * Each leg's height (cm; the legs of the last MeasureLegs, in its
		 * order) in Stance, the stance a drive mode's ramp is about to take
		 * the legs to. A leg the stance does not name is where its baseline
		 * has it: the ramp will not move it. False when the stance cannot be
		 * turned into heights, or will not be applied at all.
		 */
		virtual bool StanceHeights(const FRammsDriveStance& Stance, FLegValues& OutZ) const = 0;

		/** Each leg's height (cm, as StanceHeights) at the baselines the last
		 *  Fold remembered: where the legs were commanded before the taker had
		 *  them. False when no fold is remembered, or a motor had no baseline. */
		virtual bool FoldedHeights(FLegValues& OutZ) const = 0;

		/**
		 * Each leg's baseline moved to the height BaseZ[i] (cm; the legs of
		 * the last MeasureLegs, in its order), its motors told nothing: what
		 * they are told stays, as the new baseline plus an offset
		 * (URammsRobotBaseComponent::RebaseMotorCommand). OutDz is the height
		 * offset that leaves on each leg (cm, positive retracts): where it is
		 * told to be less where its baseline now has it. A leg whose joints
		 * cannot make its height keeps the baseline it had. Whoever holds a
		 * target of its own is told (SyncBaselineHolders). False, with
		 * nothing changed, when no baseline can be moved (the offset layer is
		 * not held; a leg nothing commands).
		 */
		virtual bool RebaseLegs(TConstArrayView<double> BaseZ, FLegValues& OutDz) = 0;

		/**
		 * The backstop for a take-over nobody announced: each LEG motor whose
		 * baseline is as the last MeasureLegs but one found it has its offset
		 * folded into it; each whose baseline was rewritten since has its
		 * offset set to zero (its new command is then what it is told).
		 * The seat's tilt is neither: its trim stays on it as an offset. The
		 * mode that took the legs has not thereby taken the seat, and whoever
		 * goes on writing the seat's own command (the seat's controller
		 * rewrites it every tick while it holds the seat) would either read as
		 * a taker, the trim dropped in one write, or write straight over a
		 * trim folded into its command. The level assist leaves it on the seat
		 * while it stands by and goes on from it at the hand-back.
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
