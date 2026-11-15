// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsCameraStreamSession.h"

#include "Engine/World.h"

URammsCameraStreamSession* URammsCameraStreamSession::Get(const UWorld* World)
{
	return World ? World->GetSubsystem<URammsCameraStreamSession>() : nullptr;
}

void URammsCameraStreamSession::Reset()
{
	bActive = false;
	bStartedCapture = false;
	bTurnedOnColor = false;
	bTurnedOnDepth = false;
	bTurnedOnMotion = false;
	bTurnedOnSegmentation = false;
	PriorRate.Reset();
	PriorSerialization.Reset();
}

bool URammsCameraStreamSession::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}
