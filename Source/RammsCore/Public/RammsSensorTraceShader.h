// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RHIGPUReadback.h"

/**
 * Input data for a single sensor ray (matches HLSL FSensorRayInput).
 * Must be kept in sync with RammsSensorTrace.usf.
 */
struct FSensorRayInput
{
	FVector3f Origin;
	float	  MaxDistance;
	FVector3f Direction;
	float	  MinDistance;
};

/**
 * Output data for a single sensor ray (matches HLSL FSensorRayOutput).
 * Must be kept in sync with RammsSensorTrace.usf.
 */
struct FSensorRayOutput
{
	float	  HitDistance; // -1 if no hit
	FVector3f HitNormal;
	uint32	  bHit; // 1 if hit, 0 if miss
	FVector3f _Padding;
};

/**
 * Compute shader for GPU-accelerated sensor ray tracing via TraceRayInline.
 * Uses RDG parameter types (SHADER_PARAMETER_RDG_BUFFER_SRV/UAV) so that
 * FComputeShaderUtils::AddPass handles all binding and resource transitions,
 * matching the Lumen/MegaLights inline RT dispatch pattern.
 */
class FRammsSensorTraceCS : public FGlobalShader
{
	DECLARE_GLOBAL_SHADER(FRammsSensorTraceCS);
	SHADER_USE_PARAMETER_STRUCT(FRammsSensorTraceCS, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
	SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FSensorRayInput>, InputRays)
	SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FSensorRayOutput>, OutputRays)
	SHADER_PARAMETER(uint32, NumRays)
	SHADER_PARAMETER(uint32, DebugMode)
	SHADER_PARAMETER_RDG_BUFFER_SRV(RaytracingAccelerationStructure, TLAS)
	END_SHADER_PARAMETER_STRUCT()

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return IsRayTracingEnabledForProject(Parameters.Platform)
			&& RHISupportsRayTracing(Parameters.Platform)
			&& RHISupportsInlineRayTracing(Parameters.Platform);
	}

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
	{
		FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
		OutEnvironment.CompilerFlags.Add(CFLAG_InlineRayTracing);
		OutEnvironment.SetDefine(TEXT("ENABLE_INLINE_RAY_TRACING"), 1);
	}
};
