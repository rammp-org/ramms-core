// Copyright Epic Games, Inc. All Rights Reserved.

#include "Curb/RammsRammpGen1CurbMeter.h"

#include "Curb/RammsCurbRunStats.h"
#include "Curb/RammsRammpGen1Curb.h"

namespace RammsRammpGen1Curb
{
	void FCrossingMeter::Begin(double TimeS)
	{
		Stats = FRammsRammpGen1CurbStats();
		bOn = true;
		T0 = TimeS;
		FrontAt = RearAt = -1.0;
		bHasPrev = bMarginSet = bMoved = false;
		RiderVel = RiderAcc = ChassisVel = FVector::ZeroVector;
		PitchRate = 0.0;
		StillSeconds = SumA2 = 0.0;
		NumA = 0;
	}

	void FCrossingMeter::Add(const FMeterSample& S)
	{
		if (!bOn)
		{
			return;
		}
		Stats.MaxAbsPitchDeg = FMath::Max(Stats.MaxAbsPitchDeg, FMath::Abs(S.PitchDeg));
		Stats.MaxAbsRollDeg = FMath::Max(Stats.MaxAbsRollDeg, FMath::Abs(S.RollDeg));
		Stats.MaxInclinationDeg = FMath::Max(Stats.MaxInclinationDeg, static_cast<float>(RammsCurb::InclinationDeg(S.PitchDeg, S.RollDeg)));

		// The stability margin, against the wheels on the ground (none: the robot is off it, the lowest margin).
		const float Margin = SupportMarginCm(S.Contacts, S.ComXY);
		if (!bMarginSet || Margin < Stats.MinStabilityMarginCm)
		{
			bMarginSet = true;
			Stats.MinStabilityMarginCm = Margin;
			Stats.MinMarginPhase = S.Phase;
		}
		if (S.bFrameOverEdge && (!Stats.bFrameMeasured || S.FrameClearanceCm < Stats.MinFrameClearanceCm))
		{
			Stats.bFrameMeasured = true;
			Stats.MinFrameClearanceCm = S.FrameClearanceCm;
			Stats.MinClearancePhase = S.Phase;
		}

		if (!bHasPrev || S.TimeS <= PrevT)
		{
			// The first sample, or the clock went back (a reset): the filters start again from here.
			bHasPrev = true;
			PrevT = S.TimeS;
			PrevRider = S.RiderPos;
			PrevChassis = S.ChassisPos;
			PrevPitch = S.PitchDeg;
			RiderVel = RiderAcc = ChassisVel = FVector::ZeroVector;
			PitchRate = 0.0;
			return;
		}
		const double  Dt = S.TimeS - PrevT;
		const double  K = Dt / (FilterSeconds + Dt);
		const FVector Vel = RiderVel + K * ((S.RiderPos - PrevRider) / Dt - RiderVel);
		RiderAcc += K * ((Vel - RiderVel) / Dt - RiderAcc);
		RiderVel = Vel;
		ChassisVel += K * ((S.ChassisPos - PrevChassis) / Dt - ChassisVel);
		PitchRate += K * ((S.PitchDeg - PrevPitch) / Dt - PitchRate);
		PrevT = S.TimeS;
		PrevRider = S.RiderPos;
		PrevChassis = S.ChassisPos;
		PrevPitch = S.PitchDeg;

		if (S.TimeS - T0 >= SettleSeconds)
		{
			const FVector H3(S.Heading.X, S.Heading.Y, 0.0);
			const double  Fore = (RiderAcc | H3) / 100.0, Vert = RiderAcc.Z / 100.0; // m/s^2
			const double  Mag = FMath::Sqrt(Fore * Fore + Vert * Vert);
			Stats.PeakRiderForeAftAccelMS2 = FMath::Max(Stats.PeakRiderForeAftAccelMS2, static_cast<float>(FMath::Abs(Fore)));
			Stats.PeakRiderVerticalAccelMS2 = FMath::Max(Stats.PeakRiderVerticalAccelMS2, static_cast<float>(FMath::Abs(Vert)));
			Stats.PeakRiderAccelMS2 = FMath::Max(Stats.PeakRiderAccelMS2, static_cast<float>(Mag));
			SumA2 += Mag * Mag * Dt;
			NumA += 1;
			Stats.RmsRiderAccelMS2 = static_cast<float>(FMath::Sqrt(SumA2 / FMath::Max(S.TimeS - T0 - SettleSeconds, 1.0e-6)));
			Stats.MaxPitchRateDegS = FMath::Max(Stats.MaxPitchRateDegS, static_cast<float>(FMath::Abs(PitchRate)));
		}
		// Stops, once it has moved.
		const double Speed = FMath::Abs(ChassisVel | FVector(S.Heading.X, S.Heading.Y, 0.0));
		bMoved |= Speed >= MovedSpeedCmS;
		if (bMoved)
		{
			if (Speed < StopSpeedCmS)
			{
				const double Before = StillSeconds;
				StillSeconds += Dt;
				Stats.SecondsStopped += static_cast<float>(Dt);
				if (Before < StopSeconds && StillSeconds >= StopSeconds)
				{
					++Stats.Stops;
				}
			}
			else
			{
				StillSeconds = 0.0;
			}
		}
		Stats.CrossingSeconds = static_cast<float>(S.TimeS - T0);
		if (FrontAt >= 0.0)
		{
			Stats.EdgeToEndSeconds = static_cast<float>(S.TimeS - FrontAt);
		}
	}

	void FCrossingMeter::MarkFrontAtEdge(double TimeS)
	{
		if (bOn && FrontAt < 0.0)
		{
			FrontAt = TimeS;
		}
	}

	void FCrossingMeter::MarkRearDown(double TimeS)
	{
		if (bOn && RearAt < 0.0)
		{
			RearAt = TimeS;
			if (FrontAt >= 0.0)
			{
				Stats.EdgeToRearDownSeconds = static_cast<float>(TimeS - FrontAt);
			}
		}
	}

	void FCrossingMeter::End(double TimeS)
	{
		if (!bOn)
		{
			return;
		}
		bOn = false;
		Stats.CrossingSeconds = static_cast<float>(TimeS - T0);
		if (FrontAt >= 0.0)
		{
			Stats.EdgeToEndSeconds = static_cast<float>(TimeS - FrontAt);
		}
	}

	FString FCrossingMeter::Summary() const
	{
		const UEnum* Phases = StaticEnum<ERammsRammpGen1CurbPhase>();
		const auto	 PhaseName = [Phases](ERammsRammpGen1CurbPhase P) { return Phases ? Phases->GetNameStringByValue(static_cast<int64>(P)) : FString(); };
		FString		 Frame = Stats.bFrameMeasured ? FString::Printf(TEXT("frame %.1f cm over the edge (%s)"), Stats.MinFrameClearanceCm, *PhaseName(Stats.MinClearancePhase))
												  : FString(TEXT("frame never over the edge"));
		return FString::Printf(TEXT("measured: %.1f s (front at the edge -> rear down %.1f s, -> end %.1f s); %d stops, %.1f s stopped; pitch %.1f deg, roll %.1f, rate %.1f deg/s; rider peak %.2f m/s^2 (fore/aft %.2f, vertical %.2f), RMS %.2f; stability margin %.1f cm (%s); %s"),
			Stats.CrossingSeconds, Stats.EdgeToRearDownSeconds, Stats.EdgeToEndSeconds, Stats.Stops, Stats.SecondsStopped, Stats.MaxAbsPitchDeg,
			Stats.MaxAbsRollDeg, Stats.MaxPitchRateDegS, Stats.PeakRiderAccelMS2, Stats.PeakRiderForeAftAccelMS2, Stats.PeakRiderVerticalAccelMS2,
			Stats.RmsRiderAccelMS2, Stats.MinStabilityMarginCm, *PhaseName(Stats.MinMarginPhase), *Frame);
	}
} // namespace RammsRammpGen1Curb
