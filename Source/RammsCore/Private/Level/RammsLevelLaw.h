// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Level/RammsLevelAllocate.h"
#include "Level/RammsLevelArbiter.h"
#include "Level/RammsLevelLoop.h"
#include "Level/RammsLevelTypes.h"

/**
 * The level assist's whole per-tick law, engine-free: one function,
 * FLaw::Step, that the component, the tests and the dry run all call. The
 * shell around it only measures (the attitude, each leg's baseline and
 * measured height and reach), passes on what happened (the switch, the mode,
 * a yield, a resume) and carries out what comes back (the legs' offsets, the
 * seat trim, the speed scale, the one-shot action). Units and signs as
 * RammsLevelTypes.h; the pieces are RammsLevelLoop.h's and
 * RammsLevelAllocate.h's, the states RammsLevelArbiter.h's.
 *
 * WHAT ONE STEP DOES, in this order. State is FState; "A", "G", "W", "C",
 * "e" as RammsLevelLoop.h; Ref is Tuning.LevelReferenceDeg.
 *
 * THE RIDER AND THE BASE. On a robot whose seat tilts (In.bSeat, with
 * Tuning.bSeatCoversTransients: "the seat runs") the rider's pitch is the
 * base's plus the seat's, and the two are wanted separately: "R" is the
 * RIDER's wanted attitude (State.Tilt), "D" the part of its pitch that the
 * seat holds (State.SeatShare: the pitch the rider prefers, and what a slope
 * past the legs' reach leaves the base tilted by, taken back out), and the
 * base's is W = R - (D, 0). R is what the tilt policy is about: it, and D,
 * move within its rate and acceleration; the base's W is the one less the
 * other, and may turn faster than either while the seat turns the other way.
 * With no seat that runs, D is 0 and W is R: one wanted attitude, as before.
 *
 * 0. HOUSEKEEPING. Dt is clamped to [0, MaxStepSeconds]. It is the time
 *    between this step's READINGS of the chassis and the last step's (the
 *    attitude, and the speed and turning rate made from where it was): every
 *    filter, gate and rate below is counted on it. Where the readings are
 *    taken at the step that is the time between two steps; where they are
 *    older than the step by a different while each time (a simulator
 *    stepping on its own thread) it is not, and the shell gives the
 *    readings' own (FMotionSense). A leg count that
 *    differs from the last step's (the first step included) counts as
 *    bColdReset. The attitude is VALID when In.bAttitudeValid and
 *    In.AttitudeDeg, every number of every leg (its XY, heights, travel,
 *    rate, radius, clearance, carry share) and every number of every clearance point are
 *    finite: one that is not would reach the terrain estimate through the
 *    plane fit and stay in it. (With a leg that is not a number nothing is
 *    allocated: an open-loop ramp holds its offsets, an adopt keeps none,
 *    until the legs read again.)
 *
 * 1. SENSING, when valid: A = Attitude.Tick(In.AttitudeDeg, Dt,
 *    AttitudeFilterSeconds); c = MeasuredCorrection(Legs);
 *    S = Ground.Tick(A + c, In.SpeedCmS, Ground.Tuning.BreakDeg /
 *    StandingSpanCm(Legs), BaselineTurnDeg(Legs, the baselines of the last
 *    step that took time), Dt, In.YawRateDegPerSec): the ground as far as it can have turned
 *    (FGroundGate); G = Terrain.Tick(S - c, c, Dt, TerrainFilterSeconds),
 *    the low-pass of S; LossSeconds = 0. When not: all hold and
 *    LossSeconds += Dt. When valid the seat hears the same reading through
 *    its own low-pass: SeatGround.Tick(S, Dt, SeatFilterSeconds). The LOOP's
 *    attitude, "A'" below, is S - c: the
 *    attitude less what only the body can have done to it (a knock, braking,
 *    a rock onto a leg set down). It is A whenever the gate lets the reading
 *    through, which is always but in those. The rest at standstill, the seat
 *    trim, the divergence watch, the speed cap's error and everything
 *    reported go on reading A itself. Legs here are In.Legs through SupportMask, going on
 *    from the support set kept in the state (which this does not move on),
 *    and DiscountLateLegs with the offsets last given (LastDz, as the last
 *    step left it; zero while there are none);
 *    or, with bSupportGiven, as given with CarryShare 1 for a leg that
 *    stands and 0 for one that does not: MeasuredCorrection is the plane of
 *    the legs that carry the robot.
 *
 * 2. THE ARBITER. FArbiter::Tick with In's bEnable, Mode, bYield, bResume,
 *    bResumeHasPlane, bStanceChanging, bColdReset (or step 0's), the validity above,
 *    bCorrectionZero = |C| < ReleaseDoneDeg and |Ride| < ReleaseDoneCm and
 *    |seat trim| < ReleaseDoneDeg and the offsets not held back behind the
 *    allocation (g2 below) (all as the last step left them), and
 *    bDiverged = Divergence.HasDiverged(). Its action is applied at once:
 *
 *    StartFromAttitude  C = 0, Loop.Reset, Ride = 0, LastDz = 0; the seat's
 *                       trim := In.SeatTrimOnDeg (what the seat carries as an
 *                       offset now: 0 from Off; a trim taken back out of the
 *                       seat's command at a hand-back, or left on it by a
 *                       take-over nobody announced, otherwise); then as
 *                       ColdResume.
 *    ColdResume         when valid, the filters are seeded from this step's
 *                       readings (A := In.AttitudeDeg, S := G := SeatGround :=
 *                       A + MeasuredCorrection(Legs)); W := A at rest, by
 *                       D := the seat's trim as it stands and R := A + (D, 0),
 *                       both at rest: the rider is wanted where they are, and
 *                       what the seat carries is walked from there;
 *                       Loop.StartFrom(C, W, G); Quiet and
 *                       Divergence reset; the support memory cleared;
 *                       bSaturated, bRateLimited, bReleaseOpen false. C, Ride
 *                       and the seat trim stay as they are, so nothing steps.
 *                       With no attitude ever read (a plane adopted blind) W
 *                       is the reference and the loop is reset; the resume
 *                       that follows the first reading sets them.
 *    Adopt              C := In.ResumePlaneDeg, Ride = 0, the seat's trim :=
 *                       In.SeatTrimOnDeg (a fold came before, on the same step
 *                       even: 0 where its trim stayed in the seat's own command
 *                       for a taker that has commanded the seat since; the trim
 *                       itself where the shell took it back out of a command
 *                       nobody has written) and D with it; then as
 *                       ColdResume;
 *                       LastDz := Allocate(Legs, Points, C, Tuning, 0, 0).Dz.
 *                       In.Legs must be the legs AFTER the shell's rebase, so
 *                       that these are the offsets already on the motors --
 *                       or, where the shell could not leave exactly those on
 *                       every leg, LastDz := In.ResumeDz, the ones it did
 *                       leave: the next steps then bring them to the
 *                       allocation's within the legs' rate (g2), and nothing
 *                       steps. (This step's sensing takes In.ResumeDz for the
 *                       offsets last given, and no support memory.)
 *                       (The plan words this "C and I are set to the plane";
 *                       the integral here is the plane less the feed-forward,
 *                       which is what makes the next C_raw the plane.)
 *    Fold               C = 0, Loop.Reset, Ride = 0, LastDz = 0: the shell
 *                       has put (or now puts) the legs' offsets into the
 *                       baselines, and nothing moves. The seat's trim and D
 *                       are zero here too, though the shell leaves the trim
 *                       ON the seat (whoever takes the legs has not taken the
 *                       seat): the law holds none while it stands by, and
 *                       goes on from what the seat still carries when it
 *                       starts again (In.SeatTrimOnDeg). The speed scale
 *                       is 1 again at once: the drive is let go the moment
 *                       something takes the legs, and what the law shows and
 *                       next asks goes on from what the drive has.
 *    Clear              the same zeroing; this step's outputs are exact zeros
 *                       and bWrite is true.
 *    BeginRelease       ReleaseSeconds = 0, bReleaseOpen = false, Quiet reset.
 *    Freeze, None       nothing.
 *
 * 3. THE STATE'S WORK.
 *
 *    ACTIVE, and RELEASING while valid and not bReleaseOpen (closed loop):
 *    0. On the first such step after an open-loop one (a release taken back
 *       part way through its ramp; a release that lost its attitude and has
 *       it again): W := A' at rest (D and R as a resume sets them) and
 *       Loop.StartFrom(C, W, G), as a resume,
 *       and W starts moving on the next step. The loop was not consulted
 *       during the ramp, and its integral from before would ask again for
 *       what was ramped out. Active also clears bReleaseOpen.
 *    a. Support: a copy of the legs goes through SupportMask with the
 *       remembered mask (kept for the next step) -- unless In.bSupportGiven,
 *       when the legs' own bSupport and BaselineClearanceCm are used as given.
 *    b. Goal: Active, WantedTiltGoalWithLimit(G, Legs, Points, Tuning), which
 *       also says how much of the correction that would hold the reference
 *       the legs must leave, and why: bShort is set once that is over
 *       AtReachOnDeg and until it is back under AtReachOffDeg. Releasing, G
 *       itself: the attitude of the baseline stance on this ground. That is
 *       the BASE's goal. The seat's share has one too: Active with a seat
 *       that runs, DGoal = clamp(In.SeatPreferenceDeg - (Goal.X - Ref.X),
 *       the seat's room) -- the pitch the rider prefers, less what the legs
 *       must leave the base tilted by -- and 0 otherwise (a release gives the
 *       seat back with the base). The seat's room is In.SeatTrimLowDeg ..
 *       In.SeatTrimHighDeg within +-SeatTrimMaxDeg. The RIDER's goal is
 *       Goal + (DGoal, 0): on the reference with their preference, but for
 *       what neither the legs nor the seat can take out.
 *    c. Active only: Quiet.Tick(In.SpeedCmS, |A - W|, In.bStick,
 *       R and D near their goals, the seat's trim not held back by its rate,
 *       and the baselines standing still, Dt). While
 *       quiet, d to h are skipped: W, the integral, C and Ride hold, Dz =
 *       LastDz and bWrite is false. The same on a step with Dt 0.
 *       Nor is there a rest while an offset is still on its way to the
 *       allocation's (g2: a lifted leg being let down at its own pace would
 *       be left where the rest found it).
 *       The baselines stand still unless In.bStanceChanging, or a leg's
 *       BaseZ is more than BaselineStillCm from where it was last found
 *       still (it is then taken as found, for every leg that has a
 *       baseline; one that has none, bHasBaseline false, is not looked at
 *       and keeps its place): a stance being
 *       ramped under a resting base would carry it off level with nothing
 *       written, so there is no rest while one moves, and one that moves
 *       ends a rest.
 *    d. R = Tilt.Tick(the rider's goal, Dt, TiltRateDegPerSec,
 *       TiltAccelDegPerSec2) and D = SeatShare.Tick(DGoal, the same), and
 *       W = R - (D, 0) --
 *       but on a step whose action set W (StartFromAttitude, ColdResume,
 *       Adopt), when W stays where it was put and starts moving on the next
 *       step: this step's C_raw is then exactly the correction already
 *       applied, and the offsets do not move at all. And but while
 *       In.bStanceChanging, when both are braked (Tilt.Brake,
 *       SeatShare.Brake, TiltAccelDegPerSec2): W
 *       comes to rest where it can and waits. A drive mode's stance change
 *       rocks the base onto other wheels by itself; walked on top of that,
 *       the rider is turned by both at once, and the base stalls against the
 *       rock and then catches up faster than the tilt policy allows. The
 *       loop goes on holding the base at W meanwhile (levelling, or a release
 *       not yet begun to walk), and the walk goes on once the stance has
 *       arrived. (A START waits for the stance too: the arbiter's rule.)
 *    e. C_raw = Loop.Tick(A', W, G, Dt, bIntegrate, bSaturated, Tuning, Trim), with
 *       bIntegrate = Active (never in Releasing) and at least
 *       MinSupportForIntegral support legs and not bRateLimited; bSaturated
 *       and bRateLimited as the last step left them. Trim, the integral's own
 *       deadband, is RestTrimDeadbandDeg while Active with the chassis under
 *       QuietSpeedCmS, the stick still, Tilt.IsNear(Goal) and the baselines
 *       standing still (all that is left before a rest is the settle time),
 *       and the loop's own otherwise: what a walk or a knock wound into the
 *       integral is trimmed out before the rest is declared, where it would
 *       otherwise leave the base resting on the deadband's edge. Then the per-axis
 *       limit: Asked = ClampAxes(C_raw, MaxCorrectionDeg), and
 *       bAxisLimited = an axis of C_raw is more than AxisLimitSlackDeg past
 *       its limit.
 *    f. Slewed = SlewCorrection(Legs, C, Asked, Dt,
 *       MaxCorrectionRateDegPerSec, Tuning, RateUse, bRateLimited).
 *    g. Alloc = Allocate(Legs, Points, Slewed, Tuning, Ride,
 *       RideHeightRateCmPerSec Dt); Ride = Alloc.HeightCm; C := Slewed;
 *       bSaturated = Alloc.Fraction < 1, and then C := Alloc.AppliedDeg and
 *       Loop.BackCalculate(Slewed, Alloc.AppliedDeg): the base takes the
 *       rest, and W's goal has already been easing W towards that tilt.
 *    g2. The last word on how fast the offsets move: they go from LastDz
 *       towards Alloc.Dz as a whole. The slew cannot see the height term
 *       (the support legs' mean, which jumps when the support set changes)
 *       nor a fall of the allocation's share from one step to the next; this
 *       does, in two parts. First the step's COMMON HEIGHT, the mean of
 *       Alloc.Dz - LastDz over the support legs (over every leg when none
 *       stands): zero while the same legs stand, and otherwise a change of
 *       ride height, so it is given to every leg alike at no more than
 *       RideHeightRateCmPerSec, a speed (HeightRateCmS) that is picked up
 *       and lost over RideHeightRampSeconds and never more than can still
 *       stop on what is left (FTiltTarget's rule, in one dimension); all of
 *       it at once for a rate of 0 or less. Legs quick enough to make the
 *       whole of it in one frame would all be told it at once, and legs
 *       that do not answer alike pitch the base with it. Then what is left
 *       goes by the largest share (1 at most) that
 *       keeps every leg that COUNTS within LegRateFraction of its MaxRateCmS
 *       over Dt (legs with no rate do not count; no limit for
 *       LegRateFraction <= 0). The legs that count are the support legs and
 *       a lifted leg being brought up; a lifted leg being let DOWN does not
 *       (SlewCorrection's rule, and its reason: such a leg must not hold the
 *       legs that carry the robot to its pace) and is instead given at most
 *       LegRateFraction of its own rate, on its own, catching the plane up
 *       over the next steps.
 *       When the common height or the share holds the offsets back,
 *       bRateLimited is set, RateUse takes the leg-rate use the whole step
 *       asked of the legs that count, and C and Ride stay the allocation's:
 *       the offsets catch them up over the next steps. A lifted leg on its
 *       own pace does not set bRateLimited (the legs that level the base are
 *       not outrun, and the integral goes on trimming), but it does count as
 *       an offset not yet arrived: a release is not done, and no rest is
 *       declared, until it is.
 *       Dz = LastDz = the result; bWrite is true.
 *    h. Active: Divergence.Tick(|A - W|, A, MeasuredCorrection(Legs),
 *       Alloc.Fraction >= 1, Dt) (not while quiet). Releasing: ReleaseSeconds += Dt
 *       (not while In.bStanceChanging keeps the walk waiting), and bReleaseOpen becomes
 *       true once R is near G and D near 0, or ReleaseSeconds > ReleaseMaxSeconds:
 *       what is left of C is then the integral's trim, which no ground
 *       explains, and it is ramped out open loop.
 *
 *    RELEASING otherwise (open loop), and FAULTED not yet latched:
 *       Support as a. On the first such step Ramp (an FTiltTarget) is reset
 *       to C, at rest; then Path = Ramp.Tick(zero, Dt, TiltRateDegPerSec,
 *       TiltAccelDegPerSec2) and C = SlewCorrection(Legs, C, Path, Dt, no
 *       plane rate, Tuning, ...): the correction goes out within the tilt
 *       policy's rate and acceleration, starting and stopping gently, the
 *       loop not consulted. Allocate as g (no back-calculation, and C is not
 *       cut to the share that fits), then g2; bWrite true. D goes on to 0 on
 *       the tilt policy (SeatShare.Tick), and W := A at rest when valid
 *       (R := A + (D, 0)), so the error reads zero. With Dt 0, or legs that are not
 *       numbers, the offsets hold. FOutputs::bOpenLoop says a step was one
 *       of these.
 *
 *    HOLDING: Dz = LastDz, bWrite false; nothing else moves.
 *
 *    OFF, YIELDED, SUSPENDED, FAULTED latched: Dz all zero; bWrite false but
 *    on a step whose action was Clear.
 *
 * 4. SEAT TRIM (FSeatTrim): the share D, plus a COVER of what the base's pitch
 *    is off its wanted by while the legs catch the ground up. The cover runs
 *    with a seat that runs, a valid attitude and the loop closed (Active, and
 *    a release still being walked), and answers
 *
 *      Answer = SeatGround.X - c.X - W.X  -  (C.X - C.X before this step)
 *               + GroundAhead (SeatGround.X - SeatGround.X before this step)
 *
 *    where c is MeasuredCorrection (step 1) and the two steps are left out
 *    on a step with Dt 0 or whose action set W. In words: where the base's
 *    pitch will be, against where it is wanted, when the seat's command lands
 *    a command interval from now. The first three terms are the error as the
 *    loop steers by it (A' - W: the ground as the gate lets it be read, so a
 *    knock or a braking pitch is not answered), heard through the seat's own
 *    low-pass. The next is what the legs were told to add to their plane this
 *    step, which they will have done by then: without it the seat answers
 *    again what the legs have just been sent after, and a knock on a robot on
 *    the move (which passes the gate as ground) is answered twice over. (While W is
 *    walked the legs' step is the walk's, so the seat stands still through a
 *    walk and the rider follows their wanted attitude a command interval
 *    late, as the base follows its own.) The last,
 *    off unless FSeatTrim::Cover.GroundAhead says, takes the ground to go on
 *    turning as it did this step (FSeatCoverTuning says what that buys and
 *    costs). The trim is then FSeatTrim's: D less Deadband(Answer), within the
 *    seat's room, at no more than SeatTrimRateDegPerSec.
 *    With no cover (open loop, no attitude, a seat that does not run) the trim
 *    is D alone, which is on its way to 0. In Holding it holds, and it holds
 *    while quiet, when nothing is written; with no offsets the law's is 0
 *    (what the seat may still carry meanwhile is the shell's to keep, and is
 *    given back as In.SeatTrimOnDeg). (A
 *    release therefore takes the trim out too, and waits for it.)
 *
 * 5. SPEED SCALE. In Active, Releasing and Faulted not yet latched:
 *    SpeedScale.Tick (FSpeedScale) with the error |A - W|, bPastReach as the
 *    status below, In.SpeedCmS, In.AskedSpeedCmS (what the stick asks of the
 *    drive), In.YawRateDegPerSec, and, once the terrain estimate is seeded and
 *    the legs are numbers: the slope Inclination(G); the standing legs' span
 *    StandingSpanCm(Legs); the ground's turning rate (G less the G the last
 *    step left, over Dt), known only on a step that takes time with a valid
 *    attitude, whose action did not seed the estimate afresh, and on which no
 *    baseline is moving (a stance being changed turns G with the ground
 *    standing still); and what the legs can turn the correction at, the
 *    lesser of PlaneRateDegPerSec along G and along that turning rate (of
 *    those that are not 0). Nothing in it is how much reach is left: a robot
 *    standing, or driving steadily, at a limit is not slowed for it. In
 *    Holding it holds. Elsewhere SpeedScale.Relax.
 *
 * 6. OUTPUTS and the status, the first that applies:
 *    Faulted -> Diverged; Holding -> AttitudeLost once LossSeconds >=
 *    MeasurementLossSeconds, else Holding; Off -> Off; Yielded -> Yielded;
 *    Suspended -> Waiting; Releasing -> Releasing; then, Active:
 *    Inclination(A - Ref) > FaultTiltDeg -> TiltFault; the legs stopped
 *    (Fraction < 1 or bAxisLimited) and Inclination(A - Ref) > MaxTiltDeg ->
 *    PastReach; at reach -> AtReach; quiet -> Settled; else Levelling. "At
 *    reach" is bShort, Fraction < 1 or bAxisLimited: the correction that
 *    would hold the reference cannot all be given. It is said for as long as that is so -- a correction that
 *    RESTS on a limit, the wanted tilt having taken the excess, is at reach
 *    still: the base is held off the reference, and the rider is owed why.
 *    ReachLimit names what stops it: the allocation's own limit where its
 *    share is under 1, else the goal's (step b), else the per-axis limit,
 *    else whatever the allocation found nearest. ReachLeft (level.reach) and
 *    MinMarginCm are of the nearest limit of any kind (FAllocation).
 *
 * The rest follows from the pieces:
 *  - ENABLE is bumpless: on the first Active step W is the attitude as
 *    found and C_raw = Kff c_meas = 0, so the first offsets are exactly zero;
 *    from the next step W walks to level on the tilt policy.
 *  - DISABLE ends exactly on the baseline: the release takes C under the
 *    ReleaseDone thresholds, then Clear writes exact zeros and the state is
 *    Off (bClaim false: the shell lets the motors go).
 *  - FOLD THEN ADOPT is motionless: neither changes what the motors are
 *    told, only how it is split between baseline and offset.
 *  - A step with Dt 0 is allowed at any time (a yield or a resume carried out
 *    in the middle of a frame): the arbiter and its action run, nothing else
 *    advances.
 */
namespace RammsLevel
{
	/** What one step is told. Nothing here is kept between steps. */
	struct FInputs
	{
		/** The levelled legs, the same ones in the same order every step.
		 *  bSupport and BaselineClearanceCm are ignored unless bSupportGiven. */
		TConstArrayView<FLegReach> Legs;
		/** The underside points to keep clear; may be empty. */
		TConstArrayView<FClearancePoint> Points;
		/** The chassis' attitude as read (deg; X pitch nose up, Y roll +Y up),
		 *  unfiltered, and whether it was read at all. */
		FVector2D AttitudeDeg = FVector2D::ZeroVector;
		bool	  bAttitudeValid = true;
		/** The chassis' speed over the ground as measured (cm/s, 0 or more):
		 *  for the rest at standstill, for how far the ground can have
		 *  turned (the distance travelled, and how hard the base is being
		 *  braked or driven off: FGroundGate), and for the speed cap (how
		 *  far it is from what the stick asks, AskedSpeedCmS). Made from the
		 *  readings of where the chassis was, on the time between them
		 *  (FMotionSense), not the time between two steps. Whether the
		 *  rider's stick is off centre: for the rest. */
		double SpeedCmS = 0.0;
		bool   bStick = false;
		/** The speed the rider's stick asks of whoever drives, before any
		 *  request to go slower (cm/s, 0 or more); negative where the drive
		 *  cannot say. For the speed cap (FSpeedCapInputs::AskedSpeedCmS). */
		double AskedSpeedCmS = -1.0;
		/** How fast the chassis is turning about the vertical as measured
		 *  (deg/s; the sign does not matter). A robot turning on a slope sees
		 *  the ground turn in its own frame, pitch into roll, by the slope
		 *  times this, with no distance travelled: the ground gate allows the
		 *  reading that much more (FGroundGate). 0 where it is not measured. */
		double YawRateDegPerSec = 0.0;
		/** The stance is being changed under the legs (a drive mode switch in
		 *  progress): no rest meanwhile, whether or not a baseline has moved
		 *  yet; no start from nothing; and the wanted attitude waits where it
		 *  is. */
		bool bStanceChanging = false;
		/** The robot has a seat whose tilt can take a trim now. */
		bool bSeat = false;
		/** With bSeat (and Tuning.bSeatCoversTransients): the pitch the rider
		 *  prefers over the reference (deg, nose up), to be held in the seat
		 *  over a base that stays on the reference. On a robot whose seat
		 *  cannot hold it, leave 0 and put the preference in
		 *  Tuning.LevelReferenceDeg (SplitPreference does both). */
		double SeatPreferenceDeg = 0.0;
		/** With bSeat: how far the seat can be trimmed from where the rider
		 *  has it (deg): down to Low (0 or less), up to High (0 or more). The
		 *  trim is kept within these and within Tuning.SeatTrimMaxDeg. */
		double SeatTrimLowDeg = -1.0e6;
		double SeatTrimHighDeg = 1.0e6;
		/** The trim the seat carries now as an offset over its own command
		 *  (deg, nose up), as the plant reads it back. Read only on a step that
		 *  starts levelling from nothing or adopts a hand-back's plane
		 *  (StartFromAttitude, Adopt): the law's own trim is then set to it, so
		 *  that a trim left on the seat by whatever came before (a fold taken
		 *  back out of the seat's command at a hand-back, a take-over nobody
		 *  announced) is gone on from and not stepped off or added to. 0 where
		 *  the seat carries none. */
		double SeatTrimOnDeg = 0.0;
		/** Take the legs' bSupport and BaselineClearanceCm as given instead of
		 *  working them out (a plant that knows better; a test's wrong set). */
		bool bSupportGiven = false;

		/** The switch, as a level: true while level.assist is On. */
		bool bEnable = false;
		/** The active drive mode's kind. Taken for as long as a taker that
		 *  was yielded to holds the legs. */
		EMode Mode = EMode::Allowed;
		/** Once: a taker has called YieldTo. The legs' offsets (the last
		 *  outputs' Dz) are, or are now to be, folded into the baselines; the
		 *  seat's trim stays on the seat (SeatTrimOnDeg). */
		bool bYield = false;
		/** Once: the taker has called ResumeFrom. With bResumeHasPlane the
		 *  plant has taken ResumePlaneDeg (deg, a correction plane) back out
		 *  of the baselines, which Legs already show. */
		bool	  bResume = false;
		bool	  bResumeHasPlane = false;
		FVector2D ResumePlaneDeg = FVector2D::ZeroVector;
		/** With bResumeHasPlane: the offset the plant left on each leg (cm,
		 *  positive retracts; the legs' order), where it could not leave
		 *  exactly the allocation's for ResumePlaneDeg (a leg at the end of
		 *  its travel). Empty, or not the legs' length: the allocation's. */
		TConstArrayView<double> ResumeDz;
		/** Once: time ran backwards or the robot was moved by hand. */
		bool bColdReset = false;
	};

	/** What one step gives back. */
	struct FOutputs
	{
		/** Each leg's height offset (cm, positive retracts), the legs' order
		 *  and length. Always what the motors should now carry on top of their
		 *  baselines, written or not. */
		FLegValues Dz;
		/** The shell should write Dz and the seat trim this step. False while
		 *  nothing changes (at rest, Holding, no offsets): nothing is then
		 *  pushed and the servos hold. */
		bool bWrite = false;
		/** The motors should be claimed (FArbiter::WantsClaim). */
		bool bClaim = false;

		EState	State = EState::Off;
		EStatus Status = EStatus::Off;
		/** What the shell must do this step, and whether a Fold is for a
		 *  take-over nobody announced (fold or zero, motor by motor). */
		EAction Action = EAction::None;
		bool	bUnexpected = false;

		/** The filtered attitude A, the terrain estimate G, the wanted
		 *  attitude W (all deg). */
		FVector2D AttitudeDeg = FVector2D::ZeroVector;
		FVector2D GroundDeg = FVector2D::ZeroVector;
		FVector2D WantedDeg = FVector2D::ZeroVector;
		/** The correction plane C the legs carry (deg), after the slew and
		 *  any scaling back at reach. (While bRateLimited the offsets may be
		 *  a step or two behind it: step g2.) */
		FVector2D AppliedDeg = FVector2D::ZeroVector;
		/** The ride-height shift (cm, positive raises the chassis). */
		double RideCm = 0.0;
		/** The seat trim (deg, positive nose up against the chassis), and the
		 *  part of it that is the rider's wanted pitch held in the seat (the
		 *  share: the preference, and a slope past the legs' reach taken
		 *  back out); the rest is the cover of what the base is behind by. */
		double SeatTrimDeg = 0.0;
		double SeatShareDeg = 0.0;
		/** The attitude the RIDER is wanted at (deg): W with the seat's share
		 *  added to its pitch. W itself on a robot with no seat. This is what
		 *  moves within the tilt policy. */
		FVector2D RiderWantedDeg = FVector2D::ZeroVector;
		/** The drive-speed scale (MinSpeedScale..1): the share of the speed
		 *  the rider asks that the drive should give (FSpeedScale). */
		double SpeedScale = 1.0;

		/** Inclination(A) (deg): the base's total tilt; level.tilt. */
		double TiltDeg = 0.0;
		/** |A - W| (deg, Euclidean). */
		double ErrorDeg = 0.0;
		/** The last allocation's share, and its reach left and room: of
		 *  whatever is nearest stopping the correction, a leg's travel or not
		 *  (FAllocation); level.reach is ReachLeft. 1, 1 and NoTravelMarginCm
		 *  with no offsets. */
		double Fraction = 1.0;
		double ReachLeft = 1.0;
		double MinMarginCm = NoTravelMarginCm;
		int32  LimitingLeg = INDEX_NONE;
		int32  LimitingPoint = INDEX_NONE;
		/** While at reach (the status AtReach or PastReach): what stops the
		 *  correction. None otherwise. */
		EReachLimit ReachLimit = EReachLimit::None;
		/** The attitude the loop steered by this step (deg): A less what only
		 *  the body can have done (FGroundGate); A itself outside a knock. */
		FVector2D LoopAttitudeDeg = FVector2D::ZeroVector;
		/** The legs' rate use this step (the slew's, or the offsets' own if
		 *  larger; before any cut, so it can pass 1) and whether a rate cut
		 *  the step. */
		double RateUse = 0.0;
		bool   bRateLimited = false;
		/** At rest. */
		bool bQuiet = false;
		/** The correction is being ramped out open loop (a release's last
		 *  part, a release with no attitude, a fault): W is then the measured
		 *  attitude, not a target moved on the tilt policy. */
		bool bOpenLoop = false;
		/** The most correction per axis the robot may carry at the speed it
		 *  has (deg; FCarryLimit, within the tuning's own limits as the law
		 *  counts them), and whether that is less than those limits on either
		 *  axis: the correction is then held back for the speed, which is not
		 *  being at reach. As the last Active step left them. */
		FVector2D CarryDeg = FVector2D::ZeroVector;
		bool	  bEased = false;
		/** The plane the legs' joints make of their baselines by what they
		 *  stand off their commands by under the load (deg; step 1). */
		FVector2D SagDeg = FVector2D::ZeroVector;
	};

	/** Everything the law keeps between steps. Default-constructed is Off. */
	struct FState
	{
		FArbiter		 Arbiter;
		FAttitudeFilter	 Attitude;
		FGroundGate		 Ground;
		FTerrainEstimate Terrain;
		/** The ground as the gate lets it be read, low-passed for the seat
		 *  (Tuning.SeatFilterSeconds): what its cover goes by. */
		FAttitudeFilter SeatGround;
		/** The RIDER's wanted attitude: what moves within the tilt policy.
		 *  The base's, W, is this less the seat's share of its pitch. */
		FTiltTarget Tilt;
		/** The part of the rider's wanted pitch the seat holds (deg, in X; Y
		 *  is not used), moved within the tilt policy too. */
		FTiltTarget SeatShare;
		FLevelLoop	Loop;
		FSeatTrim	SeatTrim;
		FSpeedScale SpeedScale;
		FQuiet		Quiet;
		FDivergence Divergence;
		/** How much correction the robot may carry at the speed it has. */
		FCarryLimit Carry;
		/** The limits it left, and whether they are under the tuning's; and
		 *  the tilt that leaves in the base on top of what the legs' own reach
		 *  does (deg: where the base is wanted, less where it would be with
		 *  only the legs' limits). */
		FVector2D CarryDeg = FVector2D::ZeroVector;
		bool	  bEased = false;
		FVector2D EasedDeg = FVector2D::ZeroVector;
		/** What each leg's joint stands off its command by (cm, positive: the
		 *  wheel higher than told), low-passed (SagFilterSeconds). Empty: none
		 *  read yet. And the plane a release took those for when it began
		 *  (deg), once it has. */
		FLegValues SagCm;
		FVector2D  ReleaseSagDeg = FVector2D::ZeroVector;
		bool	   bReleaseSag = false;

		/** C: the slewed correction plane (deg). */
		FVector2D CorrectionDeg = FVector2D::ZeroVector;
		/** The ride-height shift (cm, positive raises the chassis). */
		double RideCm = 0.0;
		/** How fast the offsets' common height is catching the allocation's
		 *  up (cm/s; step g2), 0 when it has. */
		double HeightRateCmS = 0.0;
		/** The last offsets given, for the steps that hold them. */
		FLegValues LastDz;
		/** Who stood last step, for SupportMask's hysteresis; empty: no memory. */
		TArray<bool, TInlineAllocator<8>> Support;
		/** The last allocation, for the steps that reuse its figures. */
		double		Fraction = 1.0;
		double		ReachLeft = 1.0;
		double		MinMarginCm = NoTravelMarginCm;
		int32		LimitingLeg = INDEX_NONE;
		int32		LimitingPoint = INDEX_NONE;
		EReachLimit Limit = EReachLimit::None;
		EReachLimit NearestLimit = EReachLimit::None;
		/** The correction that would hold the reference cannot all be given
		 *  (with its hysteresis), and what the goal found stops it. */
		bool		bShort = false;
		EReachLimit GoalLimit = EReachLimit::None;
		/** Each leg's baseline height at the last step that took time (for
		 *  how far the stance itself turned since), and where each was last
		 *  found standing still (for "a baseline is moving"). Empty: none yet. */
		FLegValues PrevBaseZ;
		FLegValues StillBaseZ;
		/** The last step's allocation was short of the plane; its loop asked
		 *  past a per-axis limit; a rate cut its slew or its offsets; its
		 *  offsets were held back behind the allocation's by the legs' rate. */
		bool bSaturated = false;
		bool bAxisLimited = false;
		bool bRateLimited = false;
		bool bLegsBehind = false;
		/** The last step that moved the correction ramped it out open loop,
		 *  along this path (the correction's way to zero on the tilt policy). */
		bool		bLoopOpen = false;
		FTiltTarget Ramp;
		/** A release: how long it has run closed loop (s), and that it has
		 *  gone over to the open-loop ramp. */
		double ReleaseSeconds = 0.0;
		bool   bReleaseOpen = false;
		/** How long the attitude has been missing (s). */
		double LossSeconds = 0.0;
		/** G as the last step left it, for how fast the ground is turning
		 *  (the speed cap's). */
		FVector2D PrevGroundDeg = FVector2D::ZeroVector;
		/** The legs' count at the last step; INDEX_NONE before the first. */
		int32 LegCount = INDEX_NONE;

		void Reset() { *this = FState(); }
	};

	/** Where the rider's preferred tilt is held. */
	struct FPreferenceSplit
	{
		/** The part that goes into what "level" means for the BASE (deg;
		 *  pitch, roll): added to the tuning's LevelReferenceDeg. */
		FVector2D BaseDeg = FVector2D::ZeroVector;
		/** The pitch that is held in the seat (deg): FInputs::SeatPreferenceDeg. */
		double SeatDeg = 0.0;
	};

	/**
	 * The rider's preferred tilt (deg; pitch, roll), split between the base
	 * and the seat. With a seat that can take a trim (bSeat) and the trim
	 * switched on, the pitch is the seat's as far as the seat has room for
	 * it: the base stays on its reference and keeps its legs' reach for the
	 * ground. The seat's room is what the law gives its trim (FInputs::
	 * SeatTrimLowDeg .. SeatTrimHighDeg, about where the rider has the seat,
	 * within +-SeatTrimMaxDeg); what of the pitch does not fit there is the
	 * base's again, so that a preference the seat cannot hold whole (a trim
	 * limit under the preference's, a seat the rider has tilted near its
	 * stop) is still held, by the legs, and not dropped. The roll is always
	 * the base's (a seat has no roll axis), and so is the pitch on a robot
	 * whose seat cannot hold it. The level assist's component and the dry
	 * run both split with this, so that they cannot differ.
	 */
	inline FPreferenceSplit SplitPreference(const FVector2D& PreferenceDeg, bool bSeat, bool bSeatCoversTransients, double SeatLowDeg, double SeatHighDeg,
		double SeatTrimMaxDeg)
	{
		FPreferenceSplit Out;
		Out.BaseDeg = PreferenceDeg;
		// (A room that is not a number is no seat: FLaw::Step's own rule.)
		if (bSeat && bSeatCoversTransients && FMath::IsFinite(PreferenceDeg.X) && FMath::IsFinite(SeatLowDeg) && FMath::IsFinite(SeatHighDeg))
		{
			const double MaxDeg = FMath::Max(0.0, SeatTrimMaxDeg);
			Out.SeatDeg = FMath::Clamp(PreferenceDeg.X, FMath::Max(-MaxDeg, FMath::Min(0.0, SeatLowDeg)), FMath::Min(MaxDeg, FMath::Max(0.0, SeatHighDeg)));
			Out.BaseDeg.X = PreferenceDeg.X - Out.SeatDeg;
		}
		return Out;
	}

	struct FLaw
	{
		/** One step of the law, as the head of this file sets out. */
		static FOutputs Step(const FInputs& In, double Dt, const FTuning& Tuning, FState& State);
	};
} // namespace RammsLevel
