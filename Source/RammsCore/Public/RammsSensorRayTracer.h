// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsSensorTraceShader.h"

class FRammsSensorViewExtension;
class FSceneInterface;

/**
 * Pending ray trace submission — transferred from game thread to render thread.
 */
struct FPendingRayTraceSubmission
{
	TArray<FSensorRayInput>			  Rays;
	TSharedPtr<FRHIGPUBufferReadback> Readback;
	uint64							  SubmitFrame = 0;
	/** Scene this submission targets — used to match against the correct view's TLAS */
	FSceneInterface* TargetScene = nullptr;
};

/**
 * Handle to a pending GPU sensor trace request.
 * Returned by FRammsSensorRayTracer::SubmitTraces() and consumed by
 * IsRequestReady() / HarvestResults().
 */
struct RAMMSCORE_API FRammsSensorTraceRequest
{
	/** Number of rays in this request */
	int32 NumRays = 0;

	/** Readback object for GPU → CPU data transfer (shared with render thread) */
	TSharedPtr<FRHIGPUBufferReadback> Readback;

	/** Frame counter when the request was submitted (for timeout detection) */
	uint64 SubmitFrame = 0;

	/** Whether this request has been submitted and is pending */
	bool bPending = false;
};

/**
 * Shared GPU ray trace manager for sensor components.
 * Dispatches compute shader passes using inline ray tracing (TraceRayInline)
 * and handles async GPU → CPU readback of results.
 *
 * Internally uses a Scene View Extension to dispatch rays in PostTLASBuild,
 * after the TLAS has been built and is available for inline tracing.
 * The TLAS is obtained via the public UE::FXRenderingUtils::RayTracing API.
 *
 * Usage:
 *   1. Check IsAvailable() once at startup
 *   2. Call SubmitTraces() with ray data — returns a request handle
 *   3. Poll IsRequestReady() each tick
 *   4. Call HarvestResults() when ready
 *
 * Thread safety: All public methods must be called from the game thread.
 */
class RAMMSCORE_API FRammsSensorRayTracer
{
public:
	/**
	 * Check whether GPU ray tracing is available on this system.
	 * Tests r.RayTracing CVar, RHI capabilities, and inline RT support.
	 */
	static bool IsAvailable();

	/**
	 * Submit an array of rays for GPU tracing.
	 * Rays are queued and dispatched during the next frame's rendering
	 * (in PostTLASBuild). Results arrive 1–2 frames later.
	 *
	 * @param Rays       Array of ray origins, directions, and distance bounds.
	 * @param Scene      Scene interface for the world these rays belong to (used for TLAS matching).
	 * @return           A request handle for polling and harvesting.
	 */
	static FRammsSensorTraceRequest SubmitTraces(const TArray<FSensorRayInput>& Rays, FSceneInterface* Scene = nullptr);

	/**
	 * Non-blocking check for whether a pending request has completed.
	 */
	static bool IsRequestReady(const FRammsSensorTraceRequest& Request);

	/**
	 * Harvest the results of a completed request.
	 * Must only be called after IsRequestReady() returns true.
	 * Invalidates the request handle.
	 *
	 * @param Request    The completed request (will be reset after harvest).
	 * @return           Array of per-ray trace results.
	 */
	static TArray<FSensorRayOutput> HarvestResults(FRammsSensorTraceRequest& Request);

	/** Maximum frames to wait for a readback before discarding */
	static constexpr int32 MaxReadbackWaitFrames = 10;

private:
	static void EnsureViewExtension();
};
