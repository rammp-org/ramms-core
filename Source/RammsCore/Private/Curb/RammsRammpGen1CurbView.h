// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Curb/RammsCurbRunStats.h"
#include "Curb/RammsCurbScope.h"

/**
 * What the RAMMP Gen1 curb controller's debug view (URammsRammpGen1CurbController::DebugView, drawn in
 * RammsRammpGen1CurbDebug.cpp) keeps between frames, beyond the graphs, the labels and the panel every curb controller's
 * view keeps (RammsCurb::FDebugScope). Nothing here feeds back into the controller.
 */
namespace RammsRammpGen1Curb
{
	/** The layers the view has, and offers switches for ("Curb view"): Gen1 finds a curb (its edge and the two levels) and
	 *  has a robot to show, but no scan of planes to show and no plan made ahead of the crossing. */
	inline constexpr uint8 ViewLayers = RammsCurb::ViewLayer::Terrain | RammsCurb::ViewLayer::Robot | RammsCurb::ViewLayer::Graphs
		| RammsCurb::ViewLayer::Labels;

	/** A curb as the terrain layer draws it: the one a crossing runs on, or, while none runs, the one a crossing asked for
	 *  now would cross (or why there is none). */
	struct FViewCurb
	{
		/** A look has been made. */
		bool bValid = false;
		/** It found a curb a crossing would take; else Why says why not. */
		bool	bFound = false;
		FString Why;
		/** Game time of the look (s). */
		double Time = -1.0;
		/** A point on the edge, and square to it in the ground plane, onto the far level. */
		FVector EdgePoint = FVector::ZeroVector;
		FVector EdgeNormal = FVector::ForwardVector;
		/** The step (cm, positive up), the near level's height, and the crossing height chosen for it. */
		float StepCm = 0.0f;
		float NearZ = 0.0f;
		float HeightCm = 0.0f;
	};

	/** What the view keeps between frames. */
	struct FViewState : RammsCurb::FDebugScope
	{
		/** The chassis' and the rider's ground speed, along the chassis' heading, sampled while the view is on (the rider
		 *  at the config's RiderOffsetCm on its SeatBody, as the measurements take it). */
		RammsCurb::FMotionWindow ChassisMotion;
		RammsCurb::FMotionWindow RiderMotion;

		/** The terrain layer's last look ahead, while no crossing runs. */
		FViewCurb Preview;
	};
} // namespace RammsRammpGen1Curb
