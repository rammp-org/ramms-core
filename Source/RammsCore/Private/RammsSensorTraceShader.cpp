// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsSensorTraceShader.h"

IMPLEMENT_GLOBAL_SHADER(
	FRammsSensorTraceCS,
	"/RammsCore/Private/RammsSensorTrace.usf",
	"SensorTraceCS",
	SF_Compute);
