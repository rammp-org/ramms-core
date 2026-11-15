// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsImuTypes.h"

void FRammsImuSampleRing::SetCapacity(int32 Capacity)
{
	Buffer.Reset();
	Buffer.SetNum(FMath::Max(Capacity, 0));
	Count = 0;
}

void FRammsImuSampleRing::Push(const FRammsImuSample& Sample)
{
	const int32 Capacity = Buffer.Num();
	if (Capacity == 0)
	{
		return;
	}
	if (Count > 0)
	{
		FRammsImuSample& Newest = Buffer[static_cast<int32>((Next - 1) % Capacity)];
		if (Newest.SimTimeSeconds == Sample.SimTimeSeconds)
		{
			Newest = Sample; // the same step again (a sub-step, a re-read state): its final state wins
			return;
		}
	}
	Buffer[static_cast<int32>(Next % Capacity)] = Sample;
	++Next;
	Count = FMath::Min(Count + 1, Capacity);
}

int64 FRammsImuSampleRing::Read(int64 Since, int32 Max, TArray<FRammsImuSample>& Out) const
{
	const int64 Oldest = GetOldestSeq();
	const int64 Start = FMath::Clamp(Since, Oldest, Next);
	const int64 Available = Next - Start;
	const int32 Num = static_cast<int32>(FMath::Min<int64>(Available, FMath::Max(Max, 0)));
	const int32 Capacity = Buffer.Num();
	Out.Reserve(Out.Num() + Num);
	for (int32 i = 0; i < Num; ++i)
	{
		Out.Add(Buffer[static_cast<int32>((Start + i) % Capacity)]);
	}
	return Start;
}
