// Copyright Epic Games, Inc. All Rights Reserved.
//
// Private header — not part of the public API.
// Consumers should include RammsSensorTypes.h for the POD ray structs,
// or RammsSensorRayTracer.h for the full GPU trace API.

#pragma once

#include "RammsSensorTypes.h"
#include "GlobalShader.h"
#include "ShaderCompilerCore.h"
#include "ShaderParameterStruct.h"

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
