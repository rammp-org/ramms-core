// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Curb/RammsCurbTerrainFit.h"
#include "RammsCurbTerrain.h"

/**
 * The curb controller's debug view (URammsCurbNegotiationController::DebugView,
 * drawn in RammsCurbNegotiationDebug.cpp): what it keeps between frames, and
 * the few sums it draws from, kept here so the tests can reach them. Nothing
 * here feeds back into the controller.
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

	/** The series the graphs plot. */
	enum class EGraphSeries : uint8
	{
		/** The chassis speed measured, commanded, and the floor (cm/s). */
		Speed,
		SpeedCommand,
		SpeedFloor,
		/** The rider's ground speed, at the seat platform's pivot (cm/s): the
		 *  chassis' plus the seat's own shift along it. */
		RiderSpeed,
		/** The base's inclination measured and planned (deg). */
		Tilt,
		TiltPlanned,
		/** The stability margin measured and planned (cm). */
		Margin,
		MarginPlanned,
		Num,
	};

	/** One sample of every series, each valid or not (Valid, a bit per
	 *  EGraphSeries). */
	struct FDebugGraphSample
	{
		/** Simulated time (s). */
		double T = 0.0;
		float  Values[static_cast<int32>(EGraphSeries::Num)] = {};
		uint32 Valid = 0;

		void Set(EGraphSeries Series, float Value)
		{
			Values[static_cast<int32>(Series)] = Value;
			Valid |= 1u << static_cast<uint32>(Series);
		}
		bool  Has(EGraphSeries Series) const { return (Valid & (1u << static_cast<uint32>(Series))) != 0; }
		float Get(EGraphSeries Series) const { return Values[static_cast<int32>(Series)]; }
	};

	/** A graph's vertical range: the valid samples of Series in Samples, and
	 *  Always (a limit or a zero line drawn with them), padded; at least
	 *  MinSpan tall. */
	FVector2D GraphRange(TConstArrayView<FDebugGraphSample> Samples, TConstArrayView<EGraphSeries> Series,
		TConstArrayView<float> Always, float MinSpan);

	/** The convex hull of Points, counter-clockwise, as indices into them
	 *  (Andrew's monotone chain; collinear points left out). Fewer than three
	 *  distinct points: those there are. */
	TArray<int32> ConvexHull2D(TConstArrayView<FVector2D> Points);

	/** Text the world layers put beside what they draw, drawn over the
	 *  viewport at At's place on screen (a map need not have a HUD, which
	 *  DrawDebugString needs). */
	struct FDebugLabel
	{
		FVector At = FVector::ZeroVector;
		FString Text;
		FColor	Colour = FColor::White;
	};

	/** Boxes, lines and text at points in its own frame (Slate units, y down):
	 *  what the colour key and the graphs are drawn with, over the viewport's
	 *  canvas or into the HUD panel's Slate widget. */
	struct FDebugPainter
	{
		float LineHeight = 15.0f;

		virtual ~FDebugPainter() = default;
		virtual void  Box(float X, float Y, float W, float H, const FLinearColor& Colour) const = 0;
		virtual void  Line(float X0, float Y0, float X1, float Y1, const FLinearColor& Colour, float Thickness = 1.0f) const = 0;
		virtual float Text(float X, float Y, const FString& S, const FLinearColor& Colour) const = 0;
		virtual float Width(const FString& S) const = 0;

		void Dashed(float X0, float X1, float Y, const FLinearColor& Colour) const
		{
			for (float X = X0; X < X1; X += 8.0f)
			{
				Line(X, Y, FMath::Min(X + 4.0f, X1), Y, Colour);
			}
		}
	};

	/** What the view keeps between frames. */
	struct FDebugViewState
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

		/** The graphs' samples, oldest first. */
		TArray<FDebugGraphSample> Graph;

		/** This tick's labels. */
		TArray<FDebugLabel> Labels;
	};
} // namespace RammsCurb
