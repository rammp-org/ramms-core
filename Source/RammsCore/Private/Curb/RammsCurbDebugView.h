// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Curb/RammsCurbScope.h"
#include "Curb/RammsCurbTerrainFit.h"
#include "RammsCurbTerrain.h"

/**
 * The curb controller's debug view (URammsCurbNegotiationController::DebugView,
 * drawn in RammsCurbNegotiationDebug.cpp): what it keeps between frames, and
 * the few sums it draws from, kept here so the tests can reach them. What it
 * shares with RAMMP Gen1's view -- the graphs, the colour key, the labels and
 * the panel they are drawn in -- is in RammsCurbScope.h. Nothing here feeds
 * back into the controller.
 */
namespace RammsCurb
{
	/** One trace of a detection scan: straight down at XY from TopZ to
	 *  BottomZ, hitting at Z. */
	struct FDebugScanSample
	{
		FVector2D XY = FVector2D::ZeroVector;
		double	  TopZ = 0.0;
		double	  BottomZ = 0.0;
		double	  Z = 0.0;
		bool	  bHit = false;
	};

	/** How a scan sample sits against the planes its scan found. */
	enum class EScanSampleKind : uint8
	{
		/** On the near plane, short of the edge. */
		Near,
		/** On the far plane, past it. */
		Far,
		/** On neither: the curb's face, a second edge, an outlier the fit left
		 *  out -- or any sample of a scan that found no curb. */
		Off,
		/** The trace hit nothing. */
		Miss,
	};

	/** A sample within ResidualCm of the plane on its side of the edge is on
	 *  that plane; one on the near plane just past the edge (the edge is a fit
	 *  through the lanes' edges) is Near. */
	EScanSampleKind ClassifyScanSample(const FDebugScanSample& Sample, const FRammsCurbTerrain& Terrain, float ResidualCm);

	/** The last detection scan, as the view draws it. */
	struct FDebugScan
	{
		bool bValid = false;
		/** Game time it was made (s). */
		double Time = 0.0;
		/** Made by the view while no crossing ran (a preview of what "Cross curb
		 *  ahead" would find), rather than by the controller. */
		bool bPreview = false;
		/** It found a curb (Terrain); else Why says why not. */
		bool			  bFound = false;
		FString			  Why;
		FRammsCurbTerrain Terrain;
		/** The scan's frame: the chassis centre it looked from, its heading. */
		FVector2D				 Origin = FVector2D::ZeroVector;
		FVector2D				 Heading = FVector2D(1.0, 0.0);
		TArray<FDebugScanSample> Samples;
		TArray<FTerrainLane>	 Lanes;
		float					 TrackHalfWidthCm = 0.0f;
		/** How long it took (ms, game thread). */
		double Ms = 0.0;
		/** The tuning's MaxPlaneResidualCm, to classify the samples by. */
		float ResidualCm = 1.5f;
	};

	/** One terrain-monitor probe (the executor's runtime check of the ground
	 *  ahead of and under each wheel). */
	struct FDebugProbe
	{
		int32	  Leg = INDEX_NONE;
		FVector2D XY = FVector2D::ZeroVector;
		double	  TopZ = 0.0;
		double	  BottomZ = 0.0;
		double	  Z = 0.0;
		bool	  bHit = false;
		/** Where it hit against the surface planned on there -- the plane on its
		 *  side of the edge plus the executor's bias for it (cm, + above it).
		 *  A miss counts its depth to BottomZ, as the monitor does. */
		double ErrorCm = 0.0;
		/** Game time it was made (s). */
		double Time = 0.0;
	};

	/** The probe's error (FDebugProbe::ErrorCm) on Terrain, BiasCm the
	 *  executor's (X near plane, Y far). */
	double ProbeErrorCm(const FRammsCurbTerrain& Terrain, const FVector2D& BiasCm, const FVector2D& XY, bool bHit, double Z,
		double BottomZ);

	/** What the view keeps between frames: the graphs, the labels and the
	 *  panel and overlay they are drawn in (FDebugScope), and the sensing
	 *  layer's scan and probes. */
	struct FDebugViewState : FDebugScope
	{
		FDebugScan Scan;
		/** A detection is recording its traces into Capture. */
		bool					 bCapturing = false;
		TArray<FDebugScanSample> Capture;
		/** The next scan is the view's own preview. */
		bool bPreviewScan = false;
		/** Game time of the last preview (s), -1 before any. */
		double LastPreviewTime = -1.0;

		/** This tick's probes, and the trail of older hits (game time). */
		TArray<FDebugProbe> Probes;
		TArray<FDebugProbe> Trail;
	};
} // namespace RammsCurb
