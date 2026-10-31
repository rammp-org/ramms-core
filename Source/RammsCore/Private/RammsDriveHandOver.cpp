// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsDriveHandOver.h"

#include "HAL/IConsoleManager.h"
#include "RammsRobotBaseComponent.h"

static TAutoConsoleVariable<int32> CVarRammsDriveHoldWhenIdle(
	TEXT("ramms.Drive.HoldWhenIdle"),
	1,
	TEXT("Drive modes: hold the robot still with no input where their bHoldWhenIdle is on (1, the default), or never (0:\n")
		TEXT("development A/B runs only; the modes then leave their wheels to the control mode as before the hold)."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarRammsDriveHoldBrakes(
	TEXT("ramms.Drive.HoldBrakes"),
	1,
	TEXT("Drive modes holding the robot still (the differential and holonomic drives, the curb crossing's holds): close\n")
		TEXT("the wheels' brakes once the hold has it at rest, where the backend has them (1, the default), or hold with\n")
			TEXT("torque alone (0: development A/B runs only)."),
	ECVF_Default);

namespace RammsDrive
{
	bool HoldWhenIdleEnabled()
	{
		return CVarRammsDriveHoldWhenIdle.GetValueOnGameThread() != 0;
	}

	bool HoldBrakesEnabled()
	{
		return CVarRammsDriveHoldBrakes.GetValueOnGameThread() != 0;
	}

	bool EngageBrakes(URammsRobotBaseComponent& Base, TConstArrayView<FName> Motors, TArray<FName>& InOutBraked)
	{
		if (!HoldBrakesEnabled())
		{
			return InOutBraked.Num() > 0;
		}
		for (const FName Motor : Motors)
		{
			if (!Motor.IsNone() && !InOutBraked.Contains(Motor) && Base.HasMotorBrake(Motor) && Base.SetMotorBrake(Motor, true))
			{
				// SetMotorBrake dropped its speed loop; what the loop last applied
				// would otherwise go on beneath the brake.
				Base.SetMotorCommand(Motor, 0.0f);
				InOutBraked.Add(Motor);
			}
		}
		return InOutBraked.Num() > 0;
	}

	void ReleaseBrakes(URammsRobotBaseComponent& Base, TArray<FName>& InOutBraked)
	{
		for (const FName Motor : InOutBraked)
		{
			Base.SetMotorBrake(Motor, false);
		}
		InOutBraked.Reset();
	}

	double SmoothStep(double X)
	{
		const double T = FMath::Clamp(X, 0.0, 1.0);
		return T * T * (3.0 - 2.0 * T);
	}

	bool IsAtRest(const FRammsDriveMotion& Motion)
	{
		return Motion.IsNearlyZero(static_cast<float>(AtRestSpeedCmS), static_cast<float>(AtRestYawDegS));
	}

	// --- FChassisMotionWindow ----------------------------------------------------

	FChassisMotionWindow::FChassisMotionWindow(double InWindowSeconds, double InMinSpanSeconds)
		: WindowSeconds(FMath::Max(InWindowSeconds, 1e-3))
		, MinSpanSeconds(FMath::Clamp(InMinSpanSeconds, 1e-4, FMath::Max(InWindowSeconds, 1e-3)))
	{
	}

	void FChassisMotionWindow::Add(double Time, const FVector2D& Centre, const FVector2D& Forward)
	{
		if (Forward.IsNearlyZero() || !FMath::IsFinite(Time) || !FMath::IsFinite(Centre.X) || !FMath::IsFinite(Centre.Y))
		{
			return;
		}
		if (Poses.Num() > 0)
		{
			const double Last = Poses.Last().Time;
			if (Time < Last)
			{
				// Time went back: a reset. What was measured before it says
				// nothing about the robot now.
				Poses.Reset();
			}
			else if (Time == Last)
			{
				// The same physics step seen again (two game ticks per step).
				return;
			}
		}
		FPose Pose;
		Pose.Time = Time;
		Pose.Centre = Centre;
		Pose.Yaw = FMath::Atan2(Forward.Y, Forward.X);
		if (Poses.Num() > 0)
		{
			// Unwrapped, so a turn through +-180 deg reads as the turn it is.
			const double Previous = Poses.Last().Yaw;
			Pose.Yaw = Previous + FMath::FindDeltaAngleRadians(Previous, Pose.Yaw);
		}
		Poses.Add(Pose);
		int32 Drop = 0;
		while (Drop < Poses.Num() - 2 && Time - Poses[Drop + 1].Time >= WindowSeconds)
		{
			++Drop;
		}
		if (Drop > 0)
		{
			Poses.RemoveAt(0, Drop, EAllowShrinking::No);
		}
	}

	void FChassisMotionWindow::Reset()
	{
		Poses.Reset();
	}

	bool FChassisMotionWindow::IsValid() const
	{
		return Poses.Num() >= 2 && Poses.Last().Time - Poses[0].Time >= MinSpanSeconds;
	}

	double FChassisMotionWindow::GetLatestTime() const
	{
		return Poses.Num() > 0 ? Poses.Last().Time : -1.0;
	}

	FRammsDriveMotion FChassisMotionWindow::GetMotion() const
	{
		if (!IsValid())
		{
			return FRammsDriveMotion();
		}
		// The least-squares line through every pose in the window, not the
		// difference of its two ends: a pose read a little off its time stamp
		// (the physics clock and the joints are not always read at the same
		// instant) then moves the reading by a share of its error, not all of
		// it -- at the window's end, all of it was 20% of a 30 cm/s speed.
		double	  MeanT = 0.0, MeanYaw = 0.0;
		FVector2D MeanC = FVector2D::ZeroVector;
		for (const FPose& P : Poses)
		{
			MeanT += P.Time;
			MeanC += P.Centre;
			MeanYaw += P.Yaw;
		}
		const double N = static_cast<double>(Poses.Num());
		MeanT /= N;
		MeanC /= N;
		MeanYaw /= N;
		double	  Stt = 0.0, StYaw = 0.0;
		FVector2D StC = FVector2D::ZeroVector;
		for (const FPose& P : Poses)
		{
			const double D = P.Time - MeanT;
			Stt += D * D;
			StC += (P.Centre - MeanC) * D;
			StYaw += (P.Yaw - MeanYaw) * D;
		}
		if (Stt <= 0.0)
		{
			return FRammsDriveMotion();
		}
		const FVector2D V = StC / Stt;
		// The heading at the window's mean time: the velocity of a chassis
		// turning steadily lies along it.
		const FVector2D Forward(FMath::Cos(MeanYaw), FMath::Sin(MeanYaw));
		// Unreal's sense: +Y is to the right of +X.
		const FVector2D Right(-Forward.Y, Forward.X);
		return FRammsDriveMotion(static_cast<float>(FVector2D::DotProduct(V, Forward)), static_cast<float>(FVector2D::DotProduct(V, Right)),
			static_cast<float>(FMath::RadiansToDegrees(StYaw / Stt)));
	}

	// --- the hand-over gate ------------------------------------------------------

	FVector2D MotionExcess(const FRammsDriveMotion& Measured, const FRammsDriveMotion& Carriable)
	{
		return FVector2D(FMath::Abs(Measured.RightCmPerSecond - Carriable.RightCmPerSecond),
			FMath::Abs(Measured.YawRateDegPerSecond - Carriable.YawRateDegPerSecond));
	}

	bool NeedsHandOver(const FRammsDriveMotion& Measured, const FRammsDriveMotion& Carriable, const FHandOverLimits& Limits)
	{
		const FVector2D Excess = MotionExcess(Measured, Carriable);
		return Excess.X > Limits.MaxSidewaysCmS || Excess.Y > Limits.MaxYawExcessDegS;
	}

	bool NeedsHandOverBetween(const FRammsDriveMotion& Measured, const FRammsDriveMotion& OutgoingCarriable,
		const FRammsDriveMotion& IncomingCarriable, const FHandOverLimits& Limits)
	{
		return NeedsHandOver(Measured, IncomingCarriable, Limits) && !NeedsHandOver(Measured, OutgoingCarriable, Limits);
	}

	void FHandOverGate::Reset()
	{
		Elapsed = 0.0;
		Within = 0.0;
	}

	EHandOverGate FHandOverGate::Step(const FRammsDriveMotion& Measured, const FRammsDriveMotion& Carriable, const FHandOverLimits& Limits,
		double DeltaSeconds, bool bMeasured)
	{
		const double Dt = FMath::Max(DeltaSeconds, 0.0);
		Elapsed += Dt;
		if (!bMeasured || NeedsHandOver(Measured, Carriable, Limits))
		{
			Within = 0.0;
		}
		else
		{
			Within += Dt;
			// (A hair's allowance, so that ten 0.01 s ticks make 0.1 s.)
			if (Within >= Limits.SustainSeconds - 1e-9)
			{
				return EHandOverGate::Ready;
			}
		}
		return Elapsed >= Limits.TimeoutSeconds - 1e-9 ? EHandOverGate::TimedOut : EHandOverGate::Waiting;
	}

	// --- the stance ramp ---------------------------------------------------------

	double StanceRampSeconds(const TArray<FStanceChannel>& Channels, double MinSeconds, double MaxLinkageSpeed, double MaxMotorRate)
	{
		// The S-curve's steepest slope is 1.5 times its mean.
		constexpr double PeakOverMean = 1.5;
		double			 Seconds = FMath::Max(MinSeconds, 0.0);
		bool			 bMoves = false;
		for (const FStanceChannel& C : Channels)
		{
			const double Delta = FMath::Abs(C.To - C.From);
			if (Delta <= UE_KINDA_SMALL_NUMBER)
			{
				continue;
			}
			bMoves = true;
			const double Limit = C.bMotor ? MaxMotorRate : MaxLinkageSpeed;
			if (Limit > 0.0)
			{
				Seconds = FMath::Max(Seconds, PeakOverMean * Delta / Limit);
			}
		}
		return bMoves ? Seconds : 0.0;
	}

	void SplitStanceStages(const TArray<FStanceChannel>& Channels, TArray<FStanceChannel>& OutFirst, TArray<FStanceChannel>& OutSecond)
	{
		OutFirst.Reset();
		OutSecond.Reset();
		double Fall = 0.0;
		bool   bHeights = false;
		for (const FStanceChannel& C : Channels)
		{
			if (!C.bMotor && C.bHeight && FMath::Abs(C.To - C.From) > UE_KINDA_SMALL_NUMBER)
			{
				Fall += C.From - C.To;
				bHeights = true;
			}
		}
		if (!bHeights)
		{
			OutFirst = Channels;
			return;
		}
		// Down: the main wheels reach the ground before the corner legs let go
		// of it. Up: the corner legs take the ground before the main wheels
		// leave it.
		const bool bLinkageFirst = Fall > 0.0;
		for (const FStanceChannel& C : Channels)
		{
			(C.bMotor != bLinkageFirst ? OutFirst : OutSecond).Add(C);
		}
	}

	double StanceRampValue(const FStanceChannel& Channel, double Elapsed, double Duration)
	{
		if (Duration <= 0.0)
		{
			return Channel.To;
		}
		return FMath::Lerp(Channel.From, Channel.To, SmoothStep(Elapsed / Duration));
	}

	// --- carrying a motion -------------------------------------------------------

	FRammsDriveMotion RampMotion(const FRammsDriveMotion& From, const FRammsDriveMotion& To, double LinearAccel, double YawAccel,
		double DeltaSeconds)
	{
		const double	  Dt = FMath::Max(DeltaSeconds, 0.0);
		FRammsDriveMotion Out = To;
		const FVector2D	  Step(To.ForwardCmPerSecond - From.ForwardCmPerSecond, To.RightCmPerSecond - From.RightCmPerSecond);
		if (LinearAccel > 0.0)
		{
			const double Most = LinearAccel * Dt;
			const double Size = Step.Size();
			if (Size > Most)
			{
				const FVector2D Taken = Step * (Most / Size);
				Out.ForwardCmPerSecond = static_cast<float>(From.ForwardCmPerSecond + Taken.X);
				Out.RightCmPerSecond = static_cast<float>(From.RightCmPerSecond + Taken.Y);
			}
		}
		if (YawAccel > 0.0)
		{
			const double Most = YawAccel * Dt;
			const double Turn = To.YawRateDegPerSecond - From.YawRateDegPerSecond;
			if (FMath::Abs(Turn) > Most)
			{
				Out.YawRateDegPerSecond = static_cast<float>(From.YawRateDegPerSecond + FMath::Sign(Turn) * Most);
			}
		}
		return Out;
	}

	bool MotionsEqual(const FRammsDriveMotion& A, const FRammsDriveMotion& B, double Tolerance)
	{
		return FMath::Abs(A.ForwardCmPerSecond - B.ForwardCmPerSecond) <= Tolerance && FMath::Abs(A.RightCmPerSecond - B.RightCmPerSecond) <= Tolerance
			&& FMath::Abs(A.YawRateDegPerSecond - B.YawRateDegPerSecond) <= Tolerance;
	}

	FRammsDriveMotion DifferentialCarriable(const FRammsDriveMotion& Motion, double MaxYawDegS)
	{
		const float Yaw = MaxYawDegS > 0.0 ? FMath::Clamp(Motion.YawRateDegPerSecond, static_cast<float>(-MaxYawDegS), static_cast<float>(MaxYawDegS))
										   : Motion.YawRateDegPerSecond;
		return FRammsDriveMotion(Motion.ForwardCmPerSecond, 0.0f, Yaw);
	}

	FVector2D DifferentialWheelSpeeds(const FRammsDriveMotion& Motion, double TrackWidthCm)
	{
		// A right turn (positive yaw) carries the left wheel round the outside.
		const double Half = 0.5 * FMath::Max(TrackWidthCm, 0.0) * FMath::DegreesToRadians(static_cast<double>(Motion.YawRateDegPerSecond));
		return FVector2D(Motion.ForwardCmPerSecond + Half, Motion.ForwardCmPerSecond - Half);
	}

	double CarryBlend(double SinceHoldEnded, double BlendSeconds)
	{
		return BlendSeconds > 0.0 ? SmoothStep(SinceHoldEnded / BlendSeconds) : 1.0;
	}

	FVector HolonomicCommand(const FRammsDriveMotion& Motion, double MaxLinearCmS, double MaxYawDegS)
	{
		const auto Scale = [](double Value, double Full) { return Full > 0.0 ? FMath::Clamp(Value / Full, -1.0, 1.0) : 0.0; };
		return FVector(Scale(Motion.ForwardCmPerSecond, MaxLinearCmS), Scale(Motion.RightCmPerSecond, MaxLinearCmS),
			Scale(Motion.YawRateDegPerSecond, MaxYawDegS));
	}

	FRammsDriveMotion HolonomicMotion(const FVector& Command, double MaxLinearCmS, double MaxYawDegS)
	{
		return FRammsDriveMotion(static_cast<float>(Command.X * MaxLinearCmS), static_cast<float>(Command.Y * MaxLinearCmS),
			static_cast<float>(Command.Z * MaxYawDegS));
	}

	void FCarry::Begin(const FRammsDriveMotion& InMotion, double HoldSeconds, bool bOperatorCommanding)
	{
		bActive = true;
		Motion = InMotion;
		HoldLeft = FMath::Max(HoldSeconds, 0.0);
		bHolding = bOperatorCommanding && HoldLeft > 0.0;
		SinceHold = 0.0;
	}

	void FCarry::Step(double DeltaSeconds, bool bOperatorCommanding)
	{
		if (!bActive)
		{
			return;
		}
		const double Dt = FMath::Max(DeltaSeconds, 0.0);
		if (bHolding)
		{
			HoldLeft -= Dt;
			if (HoldLeft > 0.0 && bOperatorCommanding)
			{
				return;
			}
			// The hold is over: its own time ran out, or nobody is driving.
			bHolding = false;
			SinceHold = 0.0;
			return;
		}
		SinceHold += Dt;
	}

	void FCarry::End()
	{
		bActive = false;
		bHolding = false;
		HoldLeft = 0.0;
		SinceHold = 0.0;
	}

	// --- holding still ------------------------------------------------------------

	double FWheelHold::Rate(double Angle) const
	{
		return bHeld ? FMath::Clamp(-Gain * (Angle - Anchor), -MaxRate, MaxRate) : 0.0;
	}

	FTorqueHoldGains TorqueHoldGains(double InertiaKgM2, double MaxTorqueNm, double RadiusCm, double DecelCmS2)
	{
		// A second-order loop on the wheel's angle: about 6 rad/s, damped 0.8, its
		// integral (what holds a slope's pull at no error) over half a second.
		// Slow enough for a loop closed once a game tick (a 62 ms tick is about a
		// third of a radian of it), stiff enough that a 6 deg slope's pull sags
		// the robot a centimetre or two before the integral takes it.
		constexpr double Omega = 6.0;
		constexpr double Zeta = 0.8;
		constexpr double IntegralSeconds = 0.5;
		const double	 J = FMath::Max(InertiaKgM2, 1e-3);
		FTorqueHoldGains G;
		G.Kp = J * Omega * Omega;
		G.Kd = 2.0 * Zeta * Omega * J;
		G.Ki = G.Kp / IntegralSeconds;
		G.MaxTorque = FMath::Max(MaxTorqueNm, 0.0);
		G.Decel = RadiusCm > 0.0 ? FMath::Max(DecelCmS2, 0.0) / RadiusCm : 0.0;
		G.Inertia = J;
		return G;
	}

	bool AdvanceHoldClock(double T, double& InOutFirst, double& InOutLast, double& OutDt)
	{
		if (InOutLast >= 0.0 && T < InOutLast)
		{
			InOutFirst = T;
			InOutLast = T;
			OutDt = 0.0;
			return true;
		}
		OutDt = InOutLast >= 0.0 ? T - InOutLast : 0.0;
		if (InOutFirst < 0.0)
		{
			InOutFirst = T;
		}
		InOutLast = T;
		return false;
	}

	double FWheelAngle::Update(double Reading, double Rate, double DeltaSeconds)
	{
		const double Dt = FMath::Max(DeltaSeconds, 0.0);
		if (!bStarted)
		{
			bStarted = true;
			Integrated = 0.0;
			ReadingAtStart = Reading;
			Value = Reading;
			return Value;
		}
		Integrated += Rate * Dt;
		if (!bDecided && FMath::Abs(Integrated) >= CheckRad)
		{
			bDecided = true;
			bReadingFollows = FMath::Abs(Reading - ReadingAtStart) >= FollowShare * FMath::Abs(Integrated);
			// From here the rate's travel from where the reading began: the
			// reading stood still meanwhile, the wheel did not.
			Offset = ReadingAtStart;
		}
		Value = bReadingFollows ? Reading : Integrated + Offset;
		return Value;
	}

	double GroundHoldTorque(double MassKg, double GradeDeg, double RadiusCm, int32 Wheels)
	{
		if (MassKg <= 0.0 || RadiusCm <= 0.0 || Wheels <= 0)
		{
			return 0.0;
		}
		return MassKg * 9.81 * FMath::Sin(FMath::DegreesToRadians(GradeDeg)) * (RadiusCm / 100.0) / Wheels;
	}

	void FTorqueHold::Begin(double Angle, double Rate, double InitialIntegral, bool bEase)
	{
		bActive = true;
		bFree = false;
		OffTicks = 0;
		Target = Angle;
		TargetRate = Rate;
		Integral = InitialIntegral;
		LastTorque = 0.0;
		bEaseToRest = bEase;
		bPulledThisBraking = false;
	}

	double FTorqueHold::Step(double Angle, double Rate, double DeltaSeconds, const FTorqueHoldGains& Gains)
	{
		if (!bActive)
		{
			LastTorque = 0.0;
			return 0.0;
		}
		const double Dt = FMath::Max(DeltaSeconds, 0.0);
		// Braking never asks a wheel for more than it has: one slowing faster
		// than its target (a slope's pull uphill, its own drag) takes the target
		// with it, so the hold neither drives it on nor, finding it far off a
		// target that ran ahead, lets go of it as if it were in the air.
		const bool	 bDecided = bPairDecided;
		const double Scale = bDecided ? DecelScale : 1.0;
		bPairDecided = false;
		DecelScale = 1.0;
		if (TargetRate != 0.0 && !bFree && !bDecided)
		{
			const double Along = FMath::Sign(TargetRate);
			if (Along * (TargetRate - Rate) > CatchUpRadS)
			{
				const double Speed = Along * Rate;
				TargetRate = Speed > 0.0 ? Along * (Speed + CatchUpRadS) : 0.0;
				Target = Angle;
			}
		}
		const double RateWas = TargetRate;
		// The target brakes from where the wheel was turning to rest, then stays;
		// twice as hard while the wheel runs on ahead of it (a slope pulling it
		// on), so that downhill the loop brakes it sooner and harder; begun
		// eased, eased to rest at the end unless a slope has pulled it on.
		if (TargetRate != 0.0)
		{
			const bool bPulledOn = !bDecided && !bFree && FMath::Sign(TargetRate) * (Rate - TargetRate) > CatchUpRadS;
			bPulledThisBraking = bPulledThisBraking || (bDecided ? Scale > 1.0 : bPulledOn);
			const double Full = (bDecided ? Scale : (bPulledOn ? PulledOnDecelScale : 1.0)) * Gains.Decel;
			const double Decel = (bEaseToRest && !bPulledThisBraking && Full > 0.0)
				? Full * FMath::Clamp(FMath::Abs(TargetRate) / (Full * BrakeTaperSeconds), BrakeTaperFloor, 1.0)
				: Full;
			const double Slow = Decel * Dt;
			if (Decel <= 0.0 || FMath::Abs(TargetRate) <= Slow)
			{
				const double ToRest = Decel > 0.0 ? FMath::Abs(TargetRate) / Decel : 0.0;
				Target += 0.5 * TargetRate * ToRest;
				TargetRate = 0.0;
			}
			else
			{
				const double Next = TargetRate - FMath::Sign(TargetRate) * Slow;
				Target += 0.5 * (TargetRate + Next) * Dt;
				TargetRate = Next;
			}
		}
		// A wheel that is not carrying (in the air) is let go of rather than
		// spun up by gains set for one that is; held again once it is slow.
		if (bFree)
		{
			if (FMath::Abs(Rate) >= RearmRadS)
			{
				LastTorque = 0.0;
				return 0.0;
			}
			bFree = false;
			Target = Angle;
			TargetRate = 0.0;
			Integral = 0.0;
		}
		// Off its target by FreeSpinRadS the way the loop's own last torque
		// drove it, FreeSpinTicks ticks running: a wheel carrying the robot that
		// runs off (a slope's pull the torque is fighting), or one jolted for a
		// tick, stays held.
		OffTicks = (FMath::Abs(Rate - TargetRate) > FreeSpinRadS && (Rate - TargetRate) * LastTorque > 0.0) ? OffTicks + 1 : 0;
		if (OffTicks >= FreeSpinTicks)
		{
			OffTicks = 0;
			bFree = true;
			Integral = 0.0;
			LastTorque = 0.0;
			return 0.0;
		}
		const double Error = Target - Angle;
		const double Trial = Integral + Gains.Ki * Error * Dt;
		// The braking fed forward: what the wheel's share of the robot needs to
		// slow as the target does (a slope's pull is the integral's to find).
		const double Braking = Dt > 0.0 ? Gains.Inertia * (TargetRate - RateWas) / Dt : 0.0;
		double		 Torque = Gains.Kp * Error + Gains.Kd * (TargetRate - Rate) + Trial + Braking;
		if (Gains.MaxTorque > 0.0 && FMath::Abs(Torque) > Gains.MaxTorque)
		{
			// At the limit: the integral held where it was, so it does not wind up.
			Torque = FMath::Clamp(Torque, -Gains.MaxTorque, Gains.MaxTorque);
		}
		else
		{
			Integral = Gains.MaxTorque > 0.0 ? FMath::Clamp(Trial, -Gains.MaxTorque, Gains.MaxTorque) : Trial;
		}
		LastTorque = Torque;
		return Torque;
	}

	void BoundTurnError(FTorqueHold& Left, double LeftAngle, FTorqueHold& Right, double RightAngle, double MaxDifferenceRad)
	{
		if (!Left.bActive || !Right.bActive || Left.bFree || Right.bFree || MaxDifferenceRad <= 0.0)
		{
			return;
		}
		const double Difference = (Left.Target - LeftAngle) - (Right.Target - RightAngle);
		const double Excess = Difference - FMath::Clamp(Difference, -MaxDifferenceRad, MaxDifferenceRad);
		Left.Target -= 0.5 * Excess;
		Right.Target += 0.5 * Excess;
	}

	void StepTorqueHoldPair(FTorqueHold& Left, double LeftAngle, double LeftRate, FTorqueHold& Right, double RightAngle, double RightRate,
		double DeltaSeconds, const FTorqueHoldGains& LeftGains, const FTorqueHoldGains& RightGains, double MaxTurnDifferenceRad,
		double& OutLeftTorque, double& OutRightTorque)
	{
		const bool bBoth = Left.bActive && Right.bActive && !Left.bFree && !Right.bFree;
		if (bBoth)
		{
			const double MeanTarget = 0.5 * (Left.TargetRate + Right.TargetRate);
			const double MeanRate = 0.5 * (LeftRate + RightRate);
			double		 Scale = 1.0;
			if (MeanTarget != 0.0)
			{
				const double Along = FMath::Sign(MeanTarget);
				if (Along * (MeanTarget - MeanRate) > FTorqueHold::CatchUpRadS)
				{
					// The robot slows faster than its targets: both follow it.
					const double Speed = Along * MeanRate;
					const double Rate = Speed > 0.0 ? Along * (Speed + FTorqueHold::CatchUpRadS) : 0.0;
					Left.TargetRate = Right.TargetRate = Rate;
					Left.Target = LeftAngle;
					Right.Target = RightAngle;
				}
				else if (Along * (MeanRate - MeanTarget) > FTorqueHold::CatchUpRadS)
				{
					Scale = FTorqueHold::PulledOnDecelScale;
				}
			}
			Left.bPairDecided = Right.bPairDecided = true;
			Left.DecelScale = Right.DecelScale = Scale;
		}
		BoundTurnError(Left, LeftAngle, Right, RightAngle, MaxTurnDifferenceRad);
		OutLeftTorque = Left.Step(LeftAngle, LeftRate, DeltaSeconds, LeftGains);
		OutRightTorque = Right.Step(RightAngle, RightRate, DeltaSeconds, RightGains);
	}

	double StanceTurnSeconds(double TurnDeg, double RateDegS, double AccelDegS2)
	{
		const double Turn = FMath::Abs(TurnDeg);
		if (Turn <= 0.0)
		{
			return 0.0;
		}
		// SmoothStep: 1.5 x the mean rate at its steepest, and 6 / T^2 of the turn
		// at its ends.
		const double ByRate = RateDegS > 0.0 ? 1.5 * Turn / RateDegS : 0.0;
		const double ByAccel = AccelDegS2 > 0.0 ? FMath::Sqrt(6.0 * Turn / AccelDegS2) : 0.0;
		return FMath::Max(ByRate, ByAccel);
	}

	// --- the switch itself -------------------------------------------------------

	namespace
	{
		FString MotionText(const FRammsDriveMotion& M)
		{
			return FString::Printf(TEXT("forward %+.1f cm/s, sideways %+.1f cm/s, yaw %+.1f deg/s"), M.ForwardCmPerSecond, M.RightCmPerSecond,
				M.YawRateDegPerSecond);
		}
	} // namespace

	void FModeSwitcher::Reset(int32 ActiveMode)
	{
		Phase = ESwitchPhase::Idle;
		Active = ActiveMode;
		Pending = INDEX_NONE;
		StanceMode = INDEX_NONE;
		bActiveHeld = false;
		bAwaiting = false;
		HeldOutgoing = INDEX_NONE;
		Gate.Reset();
		Paused = 0.0;
		SinceAsked = 0.0;
		ResetStages();
	}

	void FModeSwitcher::ResetStages()
	{
		Stages[0].Reset();
		Stages[1].Reset();
		StageSeconds[0] = StageSeconds[1] = 0.0;
		Stage = 2;
		Elapsed = 0.0;
	}

	bool FModeSwitcher::Request(ISwitchHost& Host, int32 Wanted, const FMotionSample& Measured, const FSwitchConfig& Config)
	{
		if (Wanted == INDEX_NONE || !Host.IsValidMode(Wanted))
		{
			return false;
		}
		if (Phase == ESwitchPhase::Constraining || Phase == ESwitchPhase::OutgoingStage)
		{
			if (Wanted == Pending)
			{
				return true;
			}
			Cancel(Host, TEXT("cancelled: another mode was asked for"), /*bRestore=*/true, Config);
		}
		SinceAsked = 0.0;
		const int32 Out = Active;
		const bool	bOutLive = Out != INDEX_NONE && Host.IsModeActive(Out);

		if (Wanted == Out && bOutLive)
		{
			// Already the one: its stance again (re-selecting is how a stance
			// knocked out by hand is put back), in one stage from where the
			// actuators are. A hold a stance change still running put on it
			// stays until this one is done.
			Status = FString::Printf(TEXT("%s again"), *Host.ModeName(Wanted));
			Host.ActivateMode(Wanted, nullptr, 0.0);
			if (PlanStance(Host, Wanted, /*bTwoStages=*/false, Config))
			{
				Phase = ESwitchPhase::IncomingStage;
			}
			else
			{
				Finish(Host);
			}
			return true;
		}

		// From one drive mode's stance to another's, wheels down before wheels
		// up, in two stages; from a pose no drive mode set (where a curb
		// crossing left the legs, the low-level mode) in one, every actuator
		// together: such poses stand on all their wheels.
		const bool bFromStance = bOutLive && Host.HasStance(Out);
		bool	   bStance = PlanStance(Host, Wanted, bFromStance, Config);
		const bool bMoving = Measured.IsMoving();
		const bool bConstrain = bMoving && bOutLive
			&& NeedsHandOverBetween(Measured.Motion, Host.GetCarriableMotion(Out, Measured.Motion), Host.GetCarriableMotion(Wanted, Measured.Motion),
				Config.Limits);
		const bool bFirstStage = bStance && Stage == 0;
		Status = FString::Printf(TEXT("%s -> %s"), *Host.ModeName(Out), *Host.ModeName(Wanted));

		// On the move, the outgoing mode drives on -- taking off what the
		// incoming one cannot carry -- until that is gone and the first stage
		// has put the incoming mode's wheels down.
		if (bMoving && bOutLive && (bConstrain || bFirstStage))
		{
			ReleaseActive(Host);
			if (Host.BeginHandOver(Out, Wanted))
			{
				bActiveHeld = true;
				Pending = Wanted;
				Phase = bConstrain ? ESwitchPhase::Constraining : ESwitchPhase::OutgoingStage;
				Gate.Reset();
				Paused = 0.0;
				Status += FString::Printf(TEXT(": %s, %s drives on%s"), *MotionText(Measured.Motion), *Host.ModeName(Out),
					bConstrain ? TEXT(" taking off what the other cannot carry") : TEXT(" while the other's wheels go down"));
				Host.Report(Status, false);
				return true;
			}
		}
		if (bConstrain)
		{
			Host.Report(FString::Printf(TEXT("%s cannot carry the robot's motion (%s) and %s cannot take it off first; handing over at once"),
							*Host.ModeName(Wanted), *MotionText(Measured.Motion), *Host.ModeName(Out)),
				true);
		}

		// At once: at rest (or not measurable), or an outgoing mode that cannot
		// keep driving. Carrying the motion the robot has, if it is moving.
		ReleaseActive(Host);
		const double  Hold = bStance ? StageSeconds[0] + StageSeconds[1] : 0.0;
		const FString Carried = Host.ActivateMode(Wanted, bMoving ? &Measured.Motion : nullptr, Hold);
		Active = Wanted;
		Status += FString::Printf(TEXT(" at once, %s"), *Carried);
		// Planned again now: activating the mode may have made actuators
		// available that were not (a low-level mode that withdrew the linkages
		// hands them back only as it stands down), and a stance planned
		// without them would jump them at the end.
		bStance = PlanStance(Host, Wanted, bFromStance, Config);
		if (!bStance)
		{
			Finish(Host);
			Host.Report(Status, false);
			return true;
		}
		Phase = ESwitchPhase::IncomingStage;
		// Held to what the outgoing mode carries until its wheels are up, and,
		// while the first stage of a change between two stances puts this
		// mode's own wheels down, held still.
		if (Out != INDEX_NONE && Out != Wanted && Host.IsValidMode(Out) && Host.BeginHandOver(Wanted, Out))
		{
			bActiveHeld = true;
		}
		if (bFromStance && Stage == 0)
		{
			bAwaiting = true;
			Host.HoldForStance(Wanted, true);
			// The outgoing mode's wheels still carry the robot until this mode's
			// are down: held where they are meanwhile (on a slope it would roll).
			if (Out != INDEX_NONE && Out != Wanted && Host.IsValidMode(Out))
			{
				HeldOutgoing = Out;
				Host.HoldOutgoingWheels(Out, true);
			}
		}
		Host.Report(Status, false);
		return true;
	}

	void FModeSwitcher::Tick(ISwitchHost& Host, double DeltaSeconds, const FMotionSample& Measured, const FSwitchConfig& Config)
	{
		if (Phase == ESwitchPhase::Idle)
		{
			return;
		}
		const double Dt = FMath::Max(DeltaSeconds, 0.0);
		SinceAsked += Dt;
		if (Phase == ESwitchPhase::Constraining || Phase == ESwitchPhase::OutgoingStage)
		{
			if (!Host.IsValidMode(Pending))
			{
				Cancel(Host, TEXT("cancelled: the mode asked for is gone"), /*bRestore=*/true, Config);
				return;
			}
			if (!Host.IsModeActive(Active))
			{
				// Something else stood the outgoing mode down: nothing is
				// driving to hand over from, and the legs are another's.
				Cancel(Host, TEXT("cancelled: the outgoing mode was stood down"), /*bRestore=*/false, Config);
				return;
			}
		}

		if (Phase == ESwitchPhase::Constraining)
		{
			const FRammsDriveMotion Carriable = Host.GetCarriableMotion(Pending, Measured.Motion);
			const EHandOverGate		G = Gate.Step(Measured.Motion, Carriable, Config.Limits, Dt, Measured.bValid);
			const FVector2D			Excess = MotionExcess(Measured.Motion, Carriable);
			if (G == EHandOverGate::TimedOut)
			{
				Cancel(Host,
					(Measured.bValid ? FString::Printf(TEXT("refused: still moving sideways %.1f cm/s (yaw %.1f deg/s past what it turns) after %.1f s"),
										   Excess.X, Excess.Y, Gate.Elapsed)
									 : FString::Printf(TEXT("refused: the motion could not be measured for %.1f s"), Gate.Elapsed))
						+ FString::Printf(TEXT("; %s stays"), *Host.ModeName(Active)),
					/*bRestore=*/true, Config);
				return;
			}
			if (G != EHandOverGate::Ready)
			{
				Status = FString::Printf(TEXT("%s -> %s: waiting for sideways %.1f cm/s < %.1f (yaw %.1f past), %.2f s of %.1f"), *Host.ModeName(Active),
					*Host.ModeName(Pending), Excess.X, Config.Limits.MaxSidewaysCmS, Excess.Y, Gate.Elapsed, Config.Limits.TimeoutSeconds);
				return;
			}
			Status = FString::Printf(TEXT("%s -> %s: sideways %.2f cm/s after %.2f s"), *Host.ModeName(Active), *Host.ModeName(Pending), Excess.X,
				Gate.Elapsed);
			Host.Report(Status, false);
			Phase = ESwitchPhase::OutgoingStage;
			// On into the first stage this same tick.
		}

		if (Phase == ESwitchPhase::OutgoingStage)
		{
			if (Stage == 0)
			{
				// The incoming mode's wheels are going down: the motion they
				// cannot carry stays off until they are down, or they wait.
				const FRammsDriveMotion Carriable = Host.GetCarriableMotion(Pending, Measured.Motion);
				if (!Measured.bValid || NeedsHandOver(Measured.Motion, Carriable, Config.Limits))
				{
					Paused += Dt;
					const FVector2D Excess = MotionExcess(Measured.Motion, Carriable);
					if (Paused >= Config.Limits.TimeoutSeconds - 1e-9)
					{
						Cancel(Host,
							FString::Printf(TEXT("refused: moving sideways %.1f cm/s (yaw %.1f deg/s past what it turns) while its wheels were going down, "
												 "for %.1f s; %s stays"),
								Excess.X, Excess.Y, Paused, *Host.ModeName(Active)),
							/*bRestore=*/true, Config);
						return;
					}
					Status = FString::Printf(TEXT("%s -> %s: wheels paused at %.0f%% of the way down, sideways %.1f cm/s (yaw %.1f past), %.2f s of %.1f"),
						*Host.ModeName(Active), *Host.ModeName(Pending), 100.0 * SmoothStep(Elapsed / FMath::Max(StageSeconds[0], 1e-6)), Excess.X,
						Excess.Y, Paused, Config.Limits.TimeoutSeconds);
					return;
				}
				StepStance(Host, Dt);
				if (Stage == 0)
				{
					return;
				}
			}
			HandOver(Host, Measured);
			return;
		}

		if (Phase == ESwitchPhase::IncomingStage)
		{
			if (StanceMode != Active || !Host.IsModeActive(Active))
			{
				// Another mode took over without asking here: the legs are its
				// now, and the mode that was held has been stood down.
				Phase = ESwitchPhase::Idle;
				StanceMode = INDEX_NONE;
				bActiveHeld = false;
				bAwaiting = false;
				ResetStages();
				return;
			}
			const bool bDone = StepStance(Host, Dt);
			if (bAwaiting && Stage > 0)
			{
				bAwaiting = false;
				Host.HoldForStance(Active, false);
				// Its wheels are down and hold the robot now: the outgoing mode's
				// go up in the second stage.
				ReleaseOutgoing(Host);
			}
			if (bDone)
			{
				Finish(Host);
			}
		}
	}

	void FModeSwitcher::HandOver(ISwitchHost& Host, const FMotionSample& Measured)
	{
		const int32 Out = Active;
		if (bActiveHeld)
		{
			Host.EndHandOver(Out);
			bActiveHeld = false;
		}
		const bool	  bMore = Stage < 2;
		const FString Carried = Host.ActivateMode(Pending, Measured.IsMoving() ? &Measured.Motion : nullptr, bMore ? StageSeconds[1] : 0.0);
		Active = Pending;
		Pending = INDEX_NONE;
		const FString Line = FString::Printf(TEXT("%s -> %s after %.2f s, %s"), *Host.ModeName(Out), *Host.ModeName(Active), SinceAsked, *Carried);
		Status += TEXT("; ") + Line;
		Host.Report(Line, false);
		if (!bMore)
		{
			Finish(Host);
			return;
		}
		// The outgoing mode's wheels are still coming up: held to what they
		// carry until they are.
		Phase = ESwitchPhase::IncomingStage;
		bActiveHeld = Host.BeginHandOver(Active, Out);
	}

	void FModeSwitcher::Finish(ISwitchHost& Host)
	{
		if (StanceMode != INDEX_NONE && StanceMode == Active)
		{
			// The end through the same command a stance taken at once uses.
			Host.ApplyStance(Active);
		}
		ReleaseActive(Host);
		if (StageSeconds[0] > 0.0 || StageSeconds[1] > 0.0)
		{
			Status += FString::Printf(TEXT("; stance in %.2f + %.2f s"), StageSeconds[0], StageSeconds[1]);
		}
		Phase = ESwitchPhase::Idle;
		StanceMode = INDEX_NONE;
		Pending = INDEX_NONE;
		ResetStages();
	}

	void FModeSwitcher::Cancel(ISwitchHost& Host, const FString& Why, bool bRestore, const FSwitchConfig& Config)
	{
		const int32 Was = Pending;
		Pending = INDEX_NONE;
		Status = FString::Printf(TEXT("%s -> %s %s"), *Host.ModeName(Active), *Host.ModeName(Was), *Why);
		Host.Report(Status, true);
		// Back to the active mode's stance from wherever the change had the
		// legs -- its first stage may have been part way, or a change before
		// this one still running -- the mode still held to what the abandoned
		// one carries until it is: those wheels may have gone part way down.
		if (bRestore && Active != INDEX_NONE && Host.IsModeActive(Active) && PlanStance(Host, Active, /*bTwoStages=*/true, Config))
		{
			Phase = ESwitchPhase::IncomingStage;
			Status += FString::Printf(TEXT("; back to its stance in %.2f + %.2f s"), StageSeconds[0], StageSeconds[1]);
			return;
		}
		ReleaseActive(Host);
		Phase = ESwitchPhase::Idle;
		StanceMode = INDEX_NONE;
		ResetStages();
	}

	void FModeSwitcher::ReleaseActive(ISwitchHost& Host)
	{
		const bool bLive = Active != INDEX_NONE && Host.IsValidMode(Active);
		if (bActiveHeld && bLive)
		{
			Host.EndHandOver(Active);
		}
		if (bAwaiting && bLive)
		{
			Host.HoldForStance(Active, false);
		}
		bActiveHeld = false;
		bAwaiting = false;
		ReleaseOutgoing(Host);
	}

	void FModeSwitcher::ReleaseOutgoing(ISwitchHost& Host)
	{
		if (HeldOutgoing != INDEX_NONE && Host.IsValidMode(HeldOutgoing))
		{
			Host.HoldOutgoingWheels(HeldOutgoing, false);
		}
		HeldOutgoing = INDEX_NONE;
	}

	bool FModeSwitcher::PlanStance(ISwitchHost& Host, int32 Mode, bool bTwoStages, const FSwitchConfig& Config)
	{
		StanceMode = Mode;
		ResetStages();
		TArray<FStanceChannel> Channels;
		if (Mode == INDEX_NONE || !Host.GetStanceChannels(Mode, Channels))
		{
			return false;
		}
		if (bTwoStages)
		{
			SplitStanceStages(Channels, Stages[0], Stages[1]);
		}
		else
		{
			Stages[0] = MoveTemp(Channels);
		}
		for (int32 S = 0; S < 2; ++S)
		{
			StageSeconds[S] = StanceRampSeconds(Stages[S], Config.MinStageSeconds, Config.MaxLinkageSpeed, Config.MaxMotorRate);
		}
		if (StageSeconds[0] <= 0.0 && StageSeconds[1] <= 0.0)
		{
			// A jump, as configured, or nothing to move.
			ResetStages();
			return false;
		}
		Stage = StageSeconds[0] > 0.0 ? 0 : 1;
		return true;
	}

	bool FModeSwitcher::StepStance(ISwitchHost& Host, double DeltaSeconds)
	{
		if (Stage >= 2)
		{
			return true;
		}
		const int32	 S = Stage;
		const double Duration = StageSeconds[S];
		Elapsed += FMath::Max(DeltaSeconds, 0.0);
		const bool bEnd = Elapsed >= Duration;
		Host.CommandStance(Stages[S], bEnd ? Duration : Elapsed, Duration);
		if (bEnd)
		{
			Stage = S + 1;
			Elapsed = 0.0;
			if (Stage == 1 && StageSeconds[1] <= 0.0)
			{
				Stage = 2;
			}
		}
		return Stage >= 2;
	}
} // namespace RammsDrive
