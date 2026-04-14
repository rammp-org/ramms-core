// Copyright Epic Games, Inc. All Rights Reserved.

#include "RammsSensorRayTracer.h"
#include "RammsSensorTraceShader.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RHIGPUReadback.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderingThread.h"
#include "SceneView.h"
#include "SceneInterface.h"
#include "SceneViewExtension.h"
#include "DataDrivenShaderPlatformInfo.h"

// GPU sensor ray tracing requires the Renderer's TLAS access APIs
// (FRayTracingScene::GetLayerView, UE::FXRenderingUtils::RayTracing::HasRayTracingScene,
// ScenePrivate.h). These are only exported on desktop platforms (DX12/Vulkan).
// Mobile platforms (Android/iOS) may define RHI_RAYTRACING=1 for Vulkan RT
// extensions but lack the linker symbols, so we exclude them explicitly.
#if RHI_RAYTRACING && !PLATFORM_ANDROID && !PLATFORM_IOS
	#define RAMMS_GPU_SENSOR_RAYTRACING 1
#else
	#define RAMMS_GPU_SENSOR_RAYTRACING 0
#endif

#if RAMMS_GPU_SENSOR_RAYTRACING
	#include "FXRenderingUtils.h"
	// Private Renderer headers needed for FScene::RayTracingScene and FViewInfo.
	// These types are not exposed through public UE APIs as of 5.7. If a future
	// engine version provides TLAS access via UE::FXRenderingUtils::RayTracing
	// or FSceneViewExtensionBase, prefer that over private includes.
	#if ENGINE_MAJOR_VERSION > 5 || (ENGINE_MAJOR_VERSION == 5 && ENGINE_MINOR_VERSION >= 7)
		#include "ScenePrivate.h"
	#else
		#error "GPU sensor ray tracing requires UE 5.7+ for TLAS access via PostTLASBuild."
	#endif
#endif

// Console variable for easy toggling
static TAutoConsoleVariable<int32> CVarRammsSensorGPUTrace(
	TEXT("r.Ramms.SensorGPUTrace"),
	1,
	TEXT("Enable GPU ray tracing for Ramms sensor components (0 = off, 1 = on).\n")
		TEXT("Requires r.RayTracing=1 and a DX12-capable GPU with inline ray tracing."),
	ECVF_Default);

// Shader diagnostic mode: when enabled, rays 0 and 1 are overridden with TLAS
// probes and input-echo data for debugging struct alignment and TLAS population.
// WARNING: This corrupts results for the first 2 rays — only enable for debugging.
static TAutoConsoleVariable<int32> CVarRammsSensorDebugMode(
	TEXT("r.Ramms.SensorDebugMode"),
	0,
	TEXT("Enable shader diagnostic mode for sensor ray traces (default=0).\n")
		TEXT("When enabled, rays 0-1 are replaced with TLAS probes and input echoes.\n")
			TEXT("Only use for debugging — corrupts the first 2 ray results."),
	ECVF_Default);

// ============================================================================
// Internal submission type (transferred from game thread to render thread)
// ============================================================================

struct FPendingRayTraceSubmission
{
	TArray<FSensorRayInput>			  Rays;
	TSharedPtr<FRHIGPUBufferReadback> Readback;
	uint64							  SubmitFrame = 0;
	/** Scene this submission targets — used to match against the correct view's TLAS.
	 *  nullptr means "dispatch against any scene with a populated TLAS". */
	FSceneInterface* TargetScene = nullptr;
};

// ============================================================================
// RDG pass parameter structs (for dependency tracking only)
// ============================================================================

// Declares RDG dependencies for the readback copy pass
BEGIN_SHADER_PARAMETER_STRUCT(FSensorReadbackDeps, )
RDG_BUFFER_ACCESS(Source, ERHIAccess::CopySrc)
END_SHADER_PARAMETER_STRUCT()

// ============================================================================
// Scene View Extension — dispatches sensor traces after TLAS build
// ============================================================================

class FRammsSensorViewExtension final : public FSceneViewExtensionBase
{
public:
	FRammsSensorViewExtension(const FAutoRegister& AutoRegister)
		: FSceneViewExtensionBase(AutoRegister)
	{
	}

	// ---- FSceneViewExtensionBase interface ----

	virtual void SetupViewFamily(FSceneViewFamily& InViewFamily) override {}
	virtual void SetupView(FSceneViewFamily& InViewFamily, FSceneView& InView) override {}
	virtual void BeginRenderViewFamily(FSceneViewFamily& InViewFamily) override {}

	// Required: opt into PostTLASBuild_RenderThread callbacks
	virtual ESceneViewExtensionFlags GetFlags() const override
	{
		return ESceneViewExtensionFlags::SubscribesToPostTLASBuild;
	}

	virtual bool IsActiveThisFrame_Internal(const FSceneViewExtensionContext& Context) const override
	{
		FScopeLock Lock(&SubmissionLock);
		return PendingSubmissions.Num() > 0;
	}

	virtual void PostTLASBuild_RenderThread(FRDGBuilder& GraphBuilder, FSceneView& InView) override
	{
#if RAMMS_GPU_SENSOR_RAYTRACING
		FSceneInterface* ViewScene = InView.Family ? InView.Family->Scene : nullptr;
		bool			 bHasScene = ViewScene && UE::FXRenderingUtils::RayTracing::HasRayTracingScene(*ViewScene);

		// Skip views without a populated TLAS
		if (!bHasScene)
		{
			return;
		}

		// Take only submissions targeting this view's scene (or null = any scene).
		// Unmatched submissions remain queued for the correct scene's PostTLASBuild callback.
		TArray<FPendingRayTraceSubmission> Matched;
		{
			FScopeLock Lock(&SubmissionLock);
			for (int32 i = PendingSubmissions.Num() - 1; i >= 0; --i)
			{
				if (PendingSubmissions[i].TargetScene == nullptr || PendingSubmissions[i].TargetScene == ViewScene)
				{
					Matched.Add(MoveTemp(PendingSubmissions[i]));
					PendingSubmissions.RemoveAtSwap(i);
				}
			}
		}

		if (Matched.Num() == 0)
		{
			return;
		}

		// Dispatch each matched batch of sensor rays against this view's TLAS
		for (FPendingRayTraceSubmission& Submission : Matched)
		{
			DispatchSensorTrace(GraphBuilder, InView, *ViewScene, Submission);
		}
#endif // RAMMS_GPU_SENSOR_RAYTRACING
	}

	// ---- Public interface (called from game thread) ----

	void AddSubmission(FPendingRayTraceSubmission&& Submission)
	{
		FScopeLock Lock(&SubmissionLock);
		PendingSubmissions.Add(MoveTemp(Submission));
	}

private:
	mutable FCriticalSection		   SubmissionLock;
	TArray<FPendingRayTraceSubmission> PendingSubmissions;

#if RAMMS_GPU_SENSOR_RAYTRACING
	void DispatchSensorTrace(
		FRDGBuilder&				GraphBuilder,
		const FSceneView&			InView,
		FSceneInterface&			SceneInterface,
		FPendingRayTraceSubmission& Submission)
	{
		const int32 NumRays = Submission.Rays.Num();
		if (NumRays == 0)
		{
			return;
		}

		// Cast to FViewInfo to access RDG-tracked TLAS buffer.
		// PostTLASBuild always receives FViewInfo objects (bIsViewInfo==true).
		check(InView.bIsViewInfo);
		const FViewInfo& ViewInfo = static_cast<const FViewInfo&>(InView);

		// Access the render scene's RayTracingScene to get the RDG-tracked TLAS SRV.
		// GetLayerView is RENDERER_API (exported), unlike GetRayTracingSceneLayerViewChecked.
		FScene* RenderScene = InView.Family->Scene->GetRenderScene();
		check(RenderScene);
		FRDGBufferSRVRef TLASSRVRef = RenderScene->RayTracingScene.GetLayerView(
			ERayTracingSceneLayer::Base,
			ViewInfo.GetRayTracingSceneViewHandle());
		check(TLASSRVRef);

		// ---- Transform ray origins from World Space → TLAS World Space ----
		// UE 5.7 builds the TLAS in camera-relative coordinates. All instance
		// transforms are offset by PreViewTranslation (≈ -CameraPosition).
		// Ray origins from GetComponentLocation() are in absolute world space,
		// so they must be offset to match the TLAS coordinate system.
		// TLAS World = World + PreViewTranslation  (for base layer)
		// Directions are unaffected (translation doesn't change direction vectors).
		const FVector	PreViewTranslation = InView.ViewMatrices.GetPreViewTranslation();
		const FVector3f PVTFloat(PreViewTranslation);

		for (FSensorRayInput& Ray : Submission.Rays)
		{
			Ray.Origin += PVTFloat;
		}

		// Log first ray for diagnostics (only periodically, Verbose level)
		static uint32 DiagCounter = 0;
		if ((DiagCounter++ % 300) == 0 && NumRays > 0)
		{
			const FSensorRayInput& R0 = Submission.Rays[0];
			UE_LOG(LogTemp, Verbose, TEXT("RammsSensorRayTracer: Dispatching %d rays (TLAS space), Ray[0]: Origin=(%.1f,%.1f,%.1f) Dir=(%.3f,%.3f,%.3f) TMin=%.1f TMax=%.1f"),
				NumRays,
				R0.Origin.X, R0.Origin.Y, R0.Origin.Z,
				R0.Direction.X, R0.Direction.Y, R0.Direction.Z,
				R0.MinDistance, R0.MaxDistance);
		}

		// Upload ray data to a temporary RDG structured buffer
		FRDGBufferRef InputBuffer = CreateStructuredBuffer(
			GraphBuilder,
			TEXT("SensorRayInput"),
			sizeof(FSensorRayInput),
			NumRays,
			Submission.Rays.GetData(),
			NumRays * sizeof(FSensorRayInput));

		// Create output buffer (SourceCopy for readback)
		FRDGBufferDesc OutputDesc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FSensorRayOutput), NumRays);
		OutputDesc.Usage = EBufferUsageFlags(OutputDesc.Usage | BUF_SourceCopy);
		FRDGBufferRef OutputBuffer = GraphBuilder.CreateBuffer(OutputDesc, TEXT("SensorRayOutput"));

		// ---- Compute dispatch pass (Lumen/MegaLights pattern) ----
		// The shader's FParameters uses SHADER_PARAMETER_RDG_BUFFER_SRV/UAV,
		// so FComputeShaderUtils::AddPass handles all binding and resource transitions.
		auto							   ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
		TShaderMapRef<FRammsSensorTraceCS> CS(ShaderMap);

		FRammsSensorTraceCS::FParameters* PassParameters = GraphBuilder.AllocParameters<FRammsSensorTraceCS::FParameters>();
		PassParameters->InputRays = GraphBuilder.CreateSRV(InputBuffer);
		PassParameters->OutputRays = GraphBuilder.CreateUAV(OutputBuffer);
		PassParameters->NumRays = NumRays;
		PassParameters->DebugMode = CVarRammsSensorDebugMode.GetValueOnRenderThread();
		PassParameters->TLAS = TLASSRVRef;

		const FIntVector GroupCount(FMath::DivideAndRoundUp(NumRays, 64), 1, 1);

		FComputeShaderUtils::AddPass(
			GraphBuilder,
			RDG_EVENT_NAME("RammsSensorTraceDispatch"),
			ERDGPassFlags::Compute | ERDGPassFlags::NeverCull,
			CS,
			PassParameters,
			GroupCount);

		// ---- Readback copy pass ----
		FSensorReadbackDeps* ReadbackParams = GraphBuilder.AllocParameters<FSensorReadbackDeps>();
		ReadbackParams->Source = OutputBuffer;

		TSharedPtr<FRHIGPUBufferReadback> ReadbackShared = Submission.Readback;
		const uint32					  NumBytes = NumRays * sizeof(FSensorRayOutput);

		GraphBuilder.AddPass(
			RDG_EVENT_NAME("RammsSensorReadback"),
			ReadbackParams,
			ERDGPassFlags::Copy | ERDGPassFlags::NeverCull,
			[ReadbackShared, OutputBuffer, NumBytes](FRHICommandList& RHICmdList) {
				ReadbackShared->EnqueueCopy(RHICmdList, OutputBuffer->GetRHI(), NumBytes);
			});
	}
#endif // RAMMS_GPU_SENSOR_RAYTRACING
};

// ============================================================================
// Static singleton for the view extension
// ============================================================================

static TSharedPtr<FRammsSensorViewExtension, ESPMode::ThreadSafe> GRammsSensorViewExtension;

void FRammsSensorRayTracer::EnsureViewExtension()
{
	if (!GRammsSensorViewExtension.IsValid())
	{
		GRammsSensorViewExtension = FSceneViewExtensions::NewExtension<FRammsSensorViewExtension>();
	}

	// The renderer only populates the TLAS when at least one RT rendering effect
	// (Lumen HW RT, RT shadows, reflections, etc.) is active. Ensure
	// r.Lumen.HardwareRayTracing=True in project settings.
}

// ============================================================================
// Public API (game thread)
// ============================================================================

bool FRammsSensorRayTracer::IsAvailable()
{
#if !RAMMS_GPU_SENSOR_RAYTRACING
	return false;
#else
	if (CVarRammsSensorGPUTrace.GetValueOnGameThread() <= 0)
	{
		return false;
	}

	if (!IsRayTracingEnabled())
	{
		return false;
	}

	if (!GRHISupportsRayTracing || !GRHISupportsInlineRayTracing)
	{
		return false;
	}

	return true;
#endif
}

FRammsSensorTraceRequest FRammsSensorRayTracer::SubmitTraces(const TArray<FSensorRayInput>& Rays, FSceneInterface* Scene)
{
	FRammsSensorTraceRequest Request;
	Request.NumRays = Rays.Num();
	Request.SubmitFrame = GFrameCounter;
	Request.bPending = false;

	if (Rays.Num() == 0)
	{
		return Request;
	}

	EnsureViewExtension();

	// Create readback shared between game and render threads
	Request.Readback = MakeShared<FRHIGPUBufferReadback>(TEXT("RammsSensorTraceReadback"));
	Request.bPending = true;

	// Queue the submission for the view extension to dispatch during rendering
	FPendingRayTraceSubmission Submission;
	Submission.Rays = Rays;
	Submission.Readback = Request.Readback;
	Submission.SubmitFrame = GFrameCounter;
	Submission.TargetScene = Scene;

	static uint64 SubmitDiagFrame = 0;
	if (GFrameCounter - SubmitDiagFrame > 600)
	{
		SubmitDiagFrame = GFrameCounter;
		UE_LOG(LogTemp, Verbose, TEXT("RammsSensorRayTracer::SubmitTraces: TargetScene=%p, NumRays=%d"), Scene, Rays.Num());
	}

	GRammsSensorViewExtension->AddSubmission(MoveTemp(Submission));

	return Request;
}

bool FRammsSensorRayTracer::IsRequestReady(FRammsSensorTraceRequest& Request)
{
	if (!Request.bPending)
	{
		return false;
	}

	// Phase 2: harvest render command has been enqueued — check the atomic flag
	if (Request.bHarvestEnqueued)
	{
		return Request.HarvestReady && Request.HarvestReady->load(std::memory_order_acquire);
	}

	// Phase 1: check if the GPU readback is complete
	if (!Request.Readback.IsValid() || !Request.Readback->IsReady())
	{
		return false;
	}

	// GPU readback is done — enqueue an async render command to copy results
	// to CPU memory. This avoids FlushRenderingCommands() and its full
	// pipeline stall, which scales poorly with multiple sensors per frame.
	const int32	 NumRays = Request.NumRays;
	const uint32 CopySize = NumRays * sizeof(FSensorRayOutput);

	Request.HarvestedData = MakeShared<TArray<FSensorRayOutput>>();
	Request.HarvestedData->SetNumUninitialized(NumRays);
	Request.HarvestReady = MakeShared<std::atomic<bool>>(false);

	TSharedPtr<FRHIGPUBufferReadback>	 ReadbackPtr = Request.Readback;
	TSharedPtr<TArray<FSensorRayOutput>> ResultsPtr = Request.HarvestedData;
	TSharedPtr<std::atomic<bool>>		 ReadyFlag = Request.HarvestReady;

	ENQUEUE_RENDER_COMMAND(HarvestSensorReadback)
	(
		[ReadbackPtr, ResultsPtr, CopySize, ReadyFlag](FRHICommandListImmediate& RHICmdList) {
			const void* SrcData = ReadbackPtr->Lock(CopySize);
			if (SrcData)
			{
				FMemory::Memcpy(ResultsPtr->GetData(), SrcData, CopySize);
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("RammsSensorRayTracer: Failed to lock readback buffer"));
				ResultsPtr->Reset();
			}
			ReadbackPtr->Unlock();

			// Signal completion — the acquire in IsRequestReady ensures the
			// game thread sees all writes to ResultsPtr before this flag.
			ReadyFlag->store(true, std::memory_order_release);
		});

	Request.bHarvestEnqueued = true;
	return false; // will be ready on the next poll (typically next frame)
}

TArray<FSensorRayOutput> FRammsSensorRayTracer::HarvestResults(FRammsSensorTraceRequest& Request)
{
	TArray<FSensorRayOutput> Results;

	if (!Request.bPending || !Request.HarvestedData.IsValid())
	{
		Request.Reset();
		return Results;
	}

	Results = MoveTemp(*Request.HarvestedData);

	// Diagnostic: log raw first 2 elements (only when shader debug mode is active)
	if (CVarRammsSensorDebugMode.GetValueOnGameThread() > 0 && Results.Num() >= 2)
	{
		const FSensorRayOutput& R0 = Results[0];
		const FSensorRayOutput& R1 = Results[1];
		static uint64			DiagFrame = 0;
		if (GFrameCounter - DiagFrame > 60)
		{
			DiagFrame = GFrameCounter;
			UE_LOG(LogTemp, Log, TEXT("[RayTrace Diag] Ray[0]: bHit=%u HitDist=%.1f Normal=(%.1f,%.1f,%.1f)  |  "
									  "Ray[1]: bHit=%u(0x%08X) HitDist=%.1f Normal=(%.3f,%.3f,%.3f) Pad=(%.1f,%.1f,%.1f)"),
				R0.bHit, R0.HitDistance, R0.HitNormal.X, R0.HitNormal.Y, R0.HitNormal.Z,
				R1.bHit, R1.bHit, R1.HitDistance, R1.HitNormal.X, R1.HitNormal.Y, R1.HitNormal.Z,
				R1._Padding.X, R1._Padding.Y, R1._Padding.Z);
		}
	}

	Request.Reset();
	return Results;
}

void FRammsSensorTraceRequest::Reset()
{
	NumRays = 0;
	SubmitFrame = 0;
	bPending = false;
	bHarvestEnqueued = false;
	Readback.Reset();
	HarvestedData.Reset();
	HarvestReady.Reset();
}
