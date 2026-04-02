// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RammsSensorTypes.h"
#include <atomic>

class FRHIGPUBufferReadback;
class FRammsSensorViewExtension;
class FSceneInterface;

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

	// ---- Async harvest state (internal) ----

	/** Buffer populated by the render thread with harvested ray results */
	TSharedPtr<TArray<FSensorRayOutput>> HarvestedData;

	/** Atomic flag set by the render thread when harvest copy is complete.
	 *  Shared pointer because std::atomic is non-copyable. */
	TSharedPtr<std::atomic<bool>> HarvestReady;

	/** Whether the async harvest render command has been enqueued */
	bool bHarvestEnqueued = false;

	/** Reset all state (call on timeout or after harvest).
	 *  Defined out-of-line in RammsSensorRayTracer.cpp. */
	void Reset();
};

/**
 * Shared GPU ray trace manager for sensor components.
 * Dispatches compute shader passes using inline ray tracing (TraceRayInline)
 * and handles async GPU → CPU readback of results.
 *
 * Internally uses a Scene View Extension to dispatch rays in PostTLASBuild,
 * after the TLAS has been built and is available for inline tracing.
 * Submissions are filtered by scene to ensure rays execute against the
 * correct TLAS when multiple worlds or viewports are active (e.g. PIE + editor).
 *
 * Usage:
 *   1. Check IsAvailable() once at startup
 *   2. Call SubmitTraces() with ray data — returns a request handle
 *   3. Poll IsRequestReady() each tick (triggers async harvest when GPU is done)
 *   4. Call HarvestResults() when ready — returns pre-copied results (non-blocking)
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
	 * Poll whether a pending request is ready for harvest.
	 * When the GPU readback completes, this automatically enqueues an
	 * async render command to copy results to CPU memory without blocking.
	 * Returns true once the async copy is complete (typically the frame
	 * after readback).
	 *
	 * Non-const: internally manages the async harvest lifecycle.
	 */
	static bool IsRequestReady(FRammsSensorTraceRequest& Request);

	/**
	 * Harvest the results of a completed request.
	 * Must only be called after IsRequestReady() returns true.
	 * Non-blocking — results were pre-copied during the harvest phase.
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
