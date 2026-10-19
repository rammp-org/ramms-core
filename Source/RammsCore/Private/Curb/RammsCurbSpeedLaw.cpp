// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsCurbSpeedLaw.h"

namespace RammsCurb
{
	float BrakingSpeed(float DistanceCm, float Deceleration)
	{
		return DistanceCm <= 0.0f ? 0.0f : FMath::Sqrt(2.0f * FMath::Max(Deceleration, 0.0f) * DistanceCm);
	}

	float LagBrakingDistance(float DistanceCm, float SpeedCmS, float LagSeconds)
	{
		// A first-order follower v' = (c - v) / tau runs, from now to rest,
		// tau * v(now) farther than its command c does: the integral of c - v
		// is tau * (v(rest) - v(now)).
		return DistanceCm - FMath::Max(0.0f, SpeedCmS) * FMath::Max(0.0f, LagSeconds);
	}

	float AwaitingPlanCap(float ToWindowCm, float LevelInCm, float SpeedCmS, float Deceleration)
	{
		const float Window = BrakingSpeed(LagBrakingDistance(ToWindowCm, SpeedCmS, DriveLagSeconds), Deceleration);
		const float Line = BrakingSpeed(LagBrakingDistance(ToWindowCm - FMath::Max(0.0f, LevelInCm), SpeedCmS, DriveLagSeconds), Deceleration);
		return FMath::Min(Window, Line);
	}

	float AwaitingLevelInStep(float Current, float RoomCm, float LevelInSeconds, float Deceleration, float Dt)
	{
		const float Want = FMath::Max(0.0f, RoomCm) / FMath::Max(LevelInSeconds, 0.1f);
		if (Current <= Want)
		{
			return FMath::Max(0.0f, Current);
		}
		return FMath::Max(Want, Current - FMath::Max(0.0f, Deceleration) * FMath::Max(0.0f, Dt));
	}

	float AwaitingLevelInSeconds(float TurnDeg, float RateDegS, float AccelDegS2, float LagSeconds, float CornersDownSeconds)
	{
		// The S-curve's time (as the planner's SCurveSeconds): a trapezoid in the
		// turn rate, or a triangle when the turn is short.
		const double D = FMath::Abs(static_cast<double>(TurnDeg));
		const double R = FMath::Max(0.1, static_cast<double>(RateDegS));
		const double A = FMath::Max(0.1, static_cast<double>(AccelDegS2));
		const double Turn = D <= R * R / A ? 2.0 * FMath::Sqrt(D / A) : D / R + R / A;
		return static_cast<float>(Turn) + FMath::Max(0.0f, LagSeconds) + 0.3f + FMath::Max(0.0f, CornersDownSeconds);
	}

	double RestHoldSpeed(double OffCm)
	{
		return FMath::Clamp(-RestHoldGainPerSecond * OffCm, -RestHoldMaxCmS, RestHoldMaxCmS);
	}

	float DeadlineSpeed(float RoomCm, float Seconds, float Deceleration)
	{
		const float Room = FMath::Max(0.0f, RoomCm);
		const float T = FMath::Max(Seconds, MinDeadlineSeconds);
		if (Deceleration <= 0.0f)
		{
			return Room / T;
		}
		// The root of v^2 / (2 a) + v T - Room = 0, in the form that keeps its
		// precision when the braking distance is a small part of the room.
		return 2.0f * Room / (T + FMath::Sqrt(T * T + 2.0f * Room / Deceleration));
	}

	float RampSpeed(float Current, float Target, float Acceleration, float Deceleration, float Dt)
	{
		const float Next = Target > Current ? FMath::Min(Current + Acceleration * Dt, Target)
											: FMath::Max(Current - Deceleration * Dt, Target);
		return FMath::Max(0.0f, Next);
	}

	float NextDriveSpeed(const FRammsCurbTuning& Tuning, float Current, float StepFraction, float DeadlineCmS,
		TConstArrayView<float> BrakingDistancesCm, float Dt)
	{
		const float Target = FMath::Min(Tuning.DriveSpeed * StepFraction, DeadlineCmS);
		const float Speed = RampSpeed(Current, Target, Tuning.Acceleration, Tuning.Deceleration, Dt);
		// The braking caps are already braking curves, so they need no ramp of
		// their own.
		float Cap = TNumericLimits<float>::Max();
		for (const float Distance : BrakingDistancesCm)
		{
			Cap = FMath::Min(Cap, BrakingSpeed(Distance, Tuning.Deceleration));
		}
		return FMath::Max(0.0f, FMath::Min(Speed, Cap));
	}

	float EntrySpeed(TConstArrayView<FEntryWheel> Wheels, bool bChassisSpeedValid, float ChassisSpeedCmS)
	{
		float Sum = 0.0f;
		int32 Count = 0;
		for (const FEntryWheel& Wheel : Wheels)
		{
			if (!Wheel.bMain || !Wheel.bOnGround || FMath::Abs(Wheel.RollForward) < 0.5f)
			{
				continue;
			}
			Sum += Wheel.JointRate * Wheel.RadiusCm / Wheel.RollForward;
			++Count;
		}
		if (Count > 0)
		{
			return FMath::Max(0.0f, Sum / Count);
		}
		return bChassisSpeedValid ? FMath::Max(0.0f, ChassisSpeedCmS) : 0.0f;
	}

	float CarrierRate(double WorldTurn, float LastAngle, float Angle, float LastRate, float Rate, float Dt)
	{
		if (Dt <= 0.0f)
		{
			return 0.0f;
		}
		const bool	 bAngleReported = Angle != 0.0f || LastAngle != 0.0f;
		const double Joint = bAngleReported ? FMath::UnwindRadians(static_cast<double>(Angle) - LastAngle)
											: 0.5 * (static_cast<double>(LastRate) + Rate) * Dt;
		return static_cast<float>((WorldTurn - Joint) / Dt);
	}

	float LegMoveSeconds(float RemainingCm, float RateCmS, float ErrorCm, float LagSeconds, float ToleranceCm)
	{
		RemainingCm = FMath::Abs(RemainingCm);
		ErrorCm = FMath::Abs(ErrorCm);
		const float Ramp = RateCmS > KINDA_SMALL_NUMBER ? RemainingCm / RateCmS : 0.0f;
		// The error left when the command arrives: a first-order leg chasing a
		// ramp trails it by the rate times its lag, a leg already lagging more
		// than that by what it lags.
		const float Trail = RemainingCm > KINDA_SMALL_NUMBER ? FMath::Max(RateCmS * LagSeconds, ErrorCm) : ErrorCm;
		const float Tail = (LagSeconds > 0.0f && Trail > ToleranceCm && ToleranceCm > 0.0f)
			? LagSeconds * FMath::Loge(Trail / ToleranceCm)
			: 0.0f;
		return Ramp + Tail;
	}

	float LegReachSeconds(float MeasuredCm, float CommandCm, float RateCmS, float TargetCm, float LagSeconds)
	{
		if (MeasuredCm >= TargetCm)
		{
			return 0.0f;
		}
		if (CommandCm < TargetCm)
		{
			// The command gets there first; on a ramp the leg follows it a lag
			// behind.
			return (RateCmS > KINDA_SMALL_NUMBER ? (TargetCm - CommandCm) / RateCmS : 0.0f) + LagSeconds;
		}
		// The command is past it: the leg closes on it exponentially.
		if (LagSeconds <= 0.0f || CommandCm <= MeasuredCm)
		{
			return 0.0f;
		}
		return LagSeconds * FMath::Loge((CommandCm - MeasuredCm) / FMath::Max(CommandCm - TargetCm, KINDA_SMALL_NUMBER));
	}

	void FDeadlineChain::AddTime(float Seconds)
	{
		Seconds = FMath::Max(0.0f, Seconds);
		Total += Seconds;
		for (FGate& Gate : Gates)
		{
			Gate.SecondsAfter += Seconds;
		}
	}

	void FDeadlineChain::AddGate(float DistanceCm)
	{
		Gates.Add({ FMath::Max(0.0f, DistanceCm), 0.0f });
	}

	float FDeadlineChain::Speed(float RoomCm, float Deceleration) const
	{
		const float Room = RoomCm - DeadlineMarginCm;
		// Held from now on, the chassis must not use up the room before the
		// time-gated work ends: no gate can make that end any sooner.
		float Speed = DeadlineSpeed(Room, Total + DeadlineMarginSeconds, Deceleration);
		for (const FGate& Gate : Gates)
		{
			// The work after a gate cannot start before the chassis reaches it,
			// so behind it only the room beyond the gate is left for that work;
			// the speed it allows there can be braked down to from now.
			const float After = DeadlineSpeed(Room - Gate.DistanceCm, Gate.SecondsAfter + DeadlineMarginSeconds, Deceleration);
			Speed = FMath::Min(Speed, FMath::Sqrt(FMath::Square(After) + 2.0f * FMath::Max(Deceleration, 0.0f) * Gate.DistanceCm));
		}
		return Speed;
	}

	bool FStallWatch::Tick(float CommandedCmS, float MeasuredCmS, bool bMeasuredValid, float Dt, float LimitSeconds)
	{
		if (!bMeasuredValid)
		{
			return false;
		}
		if (CommandedCmS > StallCommandCmS && MeasuredCmS < StallMeasuredCmS)
		{
			Seconds += FMath::Max(0.0f, Dt);
		}
		else
		{
			Seconds = 0.0f;
		}
		return LimitSeconds > 0.0f && Seconds >= LimitSeconds - 1e-6f;
	}

	ERest FRestWatch::Tick(bool bHeld, float MeasuredCmS, bool bMeasuredValid, float Dt, float StoppedCmS, float LimitSeconds)
	{
		if (!bHeld)
		{
			Seconds = 0.0f;
			return ERest::Moving;
		}
		Seconds += FMath::Max(0.0f, Dt);
		if (bMeasuredValid && FMath::Abs(MeasuredCmS) < StoppedCmS)
		{
			return ERest::Rested;
		}
		return LimitSeconds > 0.0f && Seconds >= LimitSeconds - 1e-6f ? ERest::TimedOut : ERest::Moving;
	}

	void FCarrierFit::Add(float XRate, float ZRate, float CarrierRate, float Dt)
	{
		if (Dt <= 0.0f)
		{
			return;
		}
		// Exponential forgetting on simulated time, each tick weighted by its
		// length, so the fit means the same at any frame rate.
		const double Keep = FMath::Exp(-static_cast<double>(Dt) / CarrierFitMemorySeconds);
		const double X = XRate, Z = ZRate, C = CarrierRate, W = Dt;
		Sxx = Keep * Sxx + W * X * X;
		Sxz = Keep * Sxz + W * X * Z;
		Szz = Keep * Szz + W * Z * Z;
		Sxc = Keep * Sxc + W * X * C;
		Szc = Keep * Szc + W * Z * C;
	}

	FVector2D FCarrierFit::GetGains() const
	{
		// Ridge regression: a direction the leg has hardly moved in keeps a gain
		// near 0 rather than whatever the noise says.
		const double A = Sxx + CarrierFitRidge, B = Sxz, D = Szz + CarrierFitRidge;
		const double Det = A * D - B * B;
		if (Det <= UE_SMALL_NUMBER)
		{
			return FVector2D::ZeroVector;
		}
		return FVector2D((D * Sxc - B * Szc) / Det, (A * Szc - B * Sxc) / Det);
	}

	float FCarrierFit::Predict(float XRate, float ZRate) const
	{
		const FVector2D Gains = GetGains();
		return static_cast<float>(Gains.X * XRate + Gains.Y * ZRate);
	}

	void FWheelFeed::Measure(float Dt, const FVector2D& LegXZ, float CarrierRate, const FVector2D& ModelRate)
	{
		if (bHasLast && Dt > 0.0f)
		{
			MeasuredRate = (LegXZ - LastXZ) / Dt;
			MeasuredCarrier = CarrierRate;
			ModelRateLast = ModelRate;
			Fit.Add(MeasuredRate.X, MeasuredRate.Y, MeasuredCarrier, Dt);
		}
		LastXZ = LegXZ;
		bHasLast = true;
		Change = FVector2D::ZeroVector;
	}

	void FWheelFeed::Predict(const FVector2D& ModelRateNext)
	{
		// Only the change: what was measured already carries whatever the
		// model gets wrong about the leg's pace.
		Change = bHasLast ? ModelRateNext - ModelRateLast : FVector2D::ZeroVector;
	}

	float LagStep(float Value, float Command, float Dt, float Lag)
	{
		if (Lag <= 0.0f)
		{
			return Command;
		}
		return Value + (Command - Value) * (1.0f - FMath::Exp(-FMath::Max(Dt, 0.0f) / Lag));
	}

	double SpeedTrimStep(double Trim, double CommandCmS, double CommandFollowedCmS, double MeasuredCmS, bool bMeasuredValid, bool bOnGrade,
		double Dt)
	{
		if (Dt <= 0.0)
		{
			return Trim;
		}
		if (!bOnGrade)
		{
			return Trim * FMath::Exp(-Dt / SpeedTrimFadeSeconds);
		}
		if (!bMeasuredValid || FMath::Abs(CommandCmS - CommandFollowedCmS) > SpeedTrimSteadyCmS)
		{
			return Trim;
		}
		const double Error = FMath::Clamp(CommandFollowedCmS - MeasuredCmS, -SpeedTrimMaxErrorCmS, SpeedTrimMaxErrorCmS);
		return FMath::Clamp(Trim + SpeedTrimGain * Error * Dt, -SpeedTrimMaxCmS, SpeedTrimMaxCmS);
	}

	double SpeedTrimUnderCap(double Trim, double MeasuredCmS, double CommandCmS, double CapCmS)
	{
		if (Trim <= 0.0 || MeasuredCmS <= CapCmS)
		{
			return Trim;
		}
		return FMath::Min(Trim, FMath::Max(0.0, CapCmS - CommandCmS));
	}

	double SpeedFollowStep(double& A, double& B, double CommandCmS, double Dt)
	{
		const double K = 1.0 - FMath::Exp(-FMath::Max(Dt, 0.0) / static_cast<double>(DriveLagSeconds));
		A += (CommandCmS - A) * K;
		B += (A - B) * K;
		return B;
	}

	double CrossTrackSteer(double OffsetCm, double GainDegPerCm, double MaxDeg)
	{
		const double Most = FMath::Max(0.0, MaxDeg);
		return FMath::DegreesToRadians(FMath::Clamp(-GainDegPerCm * OffsetCm, -Most, Most));
	}

	double LateralSpeed(double& Trim, double OffsetCm, bool bAuthority, double Dt)
	{
		if (bAuthority && Dt > 0.0)
		{
			Trim = FMath::Clamp(Trim - LateralTrimGain * OffsetCm * Dt, -LateralMaxCmS, LateralMaxCmS);
		}
		return FMath::Clamp(Trim - LateralGain * OffsetCm, -LateralMaxCmS, LateralMaxCmS);
	}

	double TurnAbout(const FQuat& From, const FQuat& To, const FVector& Axis)
	{
		// The world-frame turn from one to the other, the short way round.
		FQuat Delta = To * From.Inverse();
		Delta.Normalize();
		if (Delta.W < 0.0)
		{
			Delta = FQuat(-Delta.X, -Delta.Y, -Delta.Z, -Delta.W);
		}
		const FVector V(Delta.X, Delta.Y, Delta.Z);
		const double  S = V.Size();
		if (S < UE_SMALL_NUMBER)
		{
			return 0.0;
		}
		const double Angle = 2.0 * FMath::Atan2(S, static_cast<double>(Delta.W));
		return FVector::DotProduct(V / S, Axis) * Angle;
	}
} // namespace RammsCurb
