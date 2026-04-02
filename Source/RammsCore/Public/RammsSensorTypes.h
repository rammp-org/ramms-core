// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * Input data for a single sensor ray (matches HLSL FSensorRayInput).
 * Must be kept bit-identical with the struct in RammsSensorTrace.usf.
 *
 * Layout (32 bytes, float-aligned):
 *   [0..11]  float3 Origin      — Ray origin in world space
 *   [12..15] float  MaxDistance  — Maximum trace distance (TMax)
 *   [16..27] float3 Direction   — Normalized ray direction
 *   [28..31] float  MinDistance  — Minimum trace distance (TMin)
 */
struct FSensorRayInput
{
	FVector3f Origin;	   // offset 0
	float	  MaxDistance; // offset 12
	FVector3f Direction;   // offset 16
	float	  MinDistance; // offset 28
};

/**
 * Output data for a single sensor ray (matches HLSL FSensorRayOutput).
 * Must be kept bit-identical with the struct in RammsSensorTrace.usf.
 *
 * Layout (32 bytes, float-aligned):
 *   [0..3]   float  HitDistance — Distance to hit point (-1 if no hit)
 *   [4..15]  float3 HitNormal  — Approximate surface normal at hit point
 *   [16..19] uint32 bHit       — 1 if hit, 0 if miss
 *   [20..31] float3 _Padding   — Reserved for alignment
 *
 * Note: HitNormal is an approximation derived from the committed triangle's
 * front-face direction in the GPU inline ray trace. It is NOT the true
 * geometric or interpolated mesh normal. Consumers should not treat it as
 * an accurate surface normal for physics or rendering calculations.
 */
struct FSensorRayOutput
{
	float	  HitDistance; // offset 0, -1 if no hit
	FVector3f HitNormal;   // offset 4
	uint32	  bHit;		   // offset 16, 1 if hit, 0 if miss
	FVector3f _Padding;	   // offset 20
};

// ============================================================================
// Compile-time validation: struct layout must match HLSL counterparts exactly.
// Any change to struct members requires a corresponding update in
// Shaders/Private/RammsSensorTrace.usf.
// ============================================================================

static_assert(sizeof(FSensorRayInput) == 32,
	"FSensorRayInput must be 32 bytes to match HLSL struct layout");
static_assert(sizeof(FSensorRayOutput) == 32,
	"FSensorRayOutput must be 32 bytes to match HLSL struct layout");

static_assert(alignof(FSensorRayInput) <= 4,
	"FSensorRayInput alignment must not exceed float alignment (4 bytes)");
static_assert(alignof(FSensorRayOutput) <= 4,
	"FSensorRayOutput alignment must not exceed float alignment (4 bytes)");

static_assert(offsetof(FSensorRayInput, Origin) == 0,
	"FSensorRayInput::Origin offset mismatch with HLSL");
static_assert(offsetof(FSensorRayInput, MaxDistance) == 12,
	"FSensorRayInput::MaxDistance offset mismatch with HLSL");
static_assert(offsetof(FSensorRayInput, Direction) == 16,
	"FSensorRayInput::Direction offset mismatch with HLSL");
static_assert(offsetof(FSensorRayInput, MinDistance) == 28,
	"FSensorRayInput::MinDistance offset mismatch with HLSL");

static_assert(offsetof(FSensorRayOutput, HitDistance) == 0,
	"FSensorRayOutput::HitDistance offset mismatch with HLSL");
static_assert(offsetof(FSensorRayOutput, HitNormal) == 4,
	"FSensorRayOutput::HitNormal offset mismatch with HLSL");
static_assert(offsetof(FSensorRayOutput, bHit) == 16,
	"FSensorRayOutput::bHit offset mismatch with HLSL");
static_assert(offsetof(FSensorRayOutput, _Padding) == 20,
	"FSensorRayOutput::_Padding offset mismatch with HLSL");
