// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Templates/Function.h"
#include "UObject/WeakObjectPtrTemplates.h"

class APlayerController;
class SWidget;
class UCanvas;
class UGameViewportClient;
class ULocalPlayer;
struct FRammsControlSurface;
struct FRammsCurbDebugView;
struct FRammsHUDPanelDesc;

/**
 * What the curb controllers' debug views share -- RAMMP Gen2's (URammsCurbNegotiationController::DebugView, drawn in
 * RammsCurbNegotiationDebug.cpp) and RAMMP Gen1's (URammsRammpGen1CurbController::DebugView, RammsRammpGen1CurbDebug.cpp):
 * the graphs' samples and their strip charts, the colour key, the labels beside what the world layers draw, the "Curb
 * view" HUD panel that hosts the key and the graphs (IRammsHUDPanelProvider), their fallback over the viewport when no
 * HUD hosts it, and the "Curb view" switches on the control surface. Each controller decides what it samples, draws and
 * keys; this draws it the same way for both. Nothing here feeds back into a controller.
 */
namespace RammsCurb
{
	/** The colours both views draw with, so that a colour means the same on either robot. */
	namespace ViewColours
	{
		// The ground: the near level, the far one, the edge between them.
		inline constexpr FColor Near(70, 210, 110);
		inline constexpr FColor Far(70, 150, 255);
		inline constexpr FColor NearGrid(40, 140, 70);
		inline constexpr FColor FarGrid(40, 95, 175);
		inline constexpr FColor Edge(255, 225, 60);
		// Text, and how good a number is.
		inline constexpr FColor Text(235, 238, 242);
		inline constexpr FColor Good(90, 210, 120);
		inline constexpr FColor Warn(255, 160, 40);
		inline constexpr FColor Bad(235, 70, 60);
		// The graphs' lines and the attitude arrows: what was measured, what was planned, the rider.
		inline constexpr FColor Measured(0, 220, 255);
		inline constexpr FColor Planned(230, 90, 255);
		inline constexpr FColor Rider(150, 240, 110);
		// A wheel's state in a crossing: on the ground, going up, clear of it, coming down, pressed down to take load,
		// held or idle.
		inline constexpr FColor Planted(90, 210, 120);
		inline constexpr FColor Lifting(255, 225, 60);
		inline constexpr FColor Clear(0, 220, 255);
		inline constexpr FColor Planting(255, 160, 40);
		inline constexpr FColor Loading(220, 90, 255);
		inline constexpr FColor Held(160, 160, 160);
	} // namespace ViewColours

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

	/** The series the graphs plot. A controller sets those it has on each
	 *  sample; a graph draws those it names. */
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

		// RAMMP Gen1's own (URammsRammpGen1CurbController).
		/** What its speed law asks for, before its acceleration limit ramps the
		 *  command to it (cm/s): the approach or crawl speed, or less, held
		 *  short of the edge for a pair not up yet. */
		SpeedTarget,
		/** The chassis' pitch (nose up) and roll (deg), signed. */
		Pitch,
		Roll,
		/** The rear-lift rule's centre of mass ahead of the drive axle (cm). */
		ComAheadOfAxle,
		/** The frame's underside over the curb's edge, above the upper level
		 *  (cm), while it is over it. */
		FrameClearance,
		/** The drive wheels' and the powered front omni wheels' speed at the
		 *  tread (cm/s), measured and commanded. */
		DriveWheelSpeed,
		DriveWheelCommand,
		FrontHubSpeed,
		FrontHubCommand,
		/** Each support's wheels' height (cm, from the reference pose, + up:
		 *  its height table at its joint's angle), measured and commanded. */
		FrontLift,
		FrontLiftCommand,
		DriveLift,
		DriveLiftCommand,
		RearLift,
		RearLiftCommand,
		/** The translators' travel (cm, + forward), measured and commanded. */
		Translator,
		TranslatorCommand,
		Num,
	};
	static_assert(static_cast<int32>(EGraphSeries::Num) <= 32, "FDebugGraphSample::Valid has a bit per series");

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

	/** Sample added to the graphs' samples (oldest first), keeping those of
	 *  its last SpanSeconds; one from before the last starts them again (the
	 *  simulation was reset). */
	void AddGraphSample(TArray<FDebugGraphSample>& Graph, const FDebugGraphSample& Sample, double SpanSeconds);

	/** A graph's vertical range: the valid samples of Series in Samples, and
	 *  Always (a limit or a zero line drawn with them), padded; at least
	 *  MinSpan tall. */
	FVector2D GraphRange(TConstArrayView<FDebugGraphSample> Samples, TConstArrayView<EGraphSeries> Series,
		TConstArrayView<float> Always, float MinSpan);

	/** The convex hull of Points, counter-clockwise, as indices into them
	 *  (Andrew's monotone chain; collinear points left out). Fewer than three
	 *  distinct points: those there are. */
	TArray<int32> ConvexHull2D(TConstArrayView<FVector2D> Points);

	/** A colour key's chip: a square and its words. */
	struct FKeyChip
	{
		FColor	Colour;
		FString Words;
	};

	struct FKeyRow
	{
		FString			 Title;
		TArray<FKeyChip> Chips;
	};

	/** A colour key -- chips in rows Width wide, wrapping -- from Top down;
	 *  its height. Drawn unless bDraw is false. */
	float DrawKey(const FDebugPainter& P, float X, float Top, float Width, TConstArrayView<FKeyRow> Rows, bool bDraw);

	/** A level line across a graph: a limit, a floor, zero. */
	struct FGraphMark
	{
		float		 Value;
		FLinearColor Colour;
	};

	/** A series a graph draws. Its latest value is written after the graph's
	 *  title under its Name; a line with no Name is drawn without (one a named
	 *  line's words already cover, as a command beside what it commands). */
	struct FGraphLine
	{
		EGraphSeries Series;
		FLinearColor Colour;
		const TCHAR* Name;
		bool		 bDashed = false;
	};

	/** One strip chart: its title and latest values on a line at Y, the chart
	 *  Height tall under it, the samples from TFrom over Span seconds across
	 *  Width; its bottom. */
	float DrawGraph(const FDebugPainter& P, float X, float Y, float Width, float Height, const TCHAR* Title,
		TConstArrayView<FDebugGraphSample> Samples, double TFrom, double Span, TConstArrayView<FGraphLine> Lines,
		TConstArrayView<FGraphMark> Marks, float MinSpan, const TCHAR* Units);

	/** Each label's lines centred over its point on screen, on a dark band
	 *  (Project: world to the painter's frame; false when off screen). */
	void PaintLabels(const FDebugPainter& P, TConstArrayView<FDebugLabel> Labels, TFunctionRef<bool(const FVector&, FVector2D&)> Project);

	/** The colour key and the graphs a controller paints, Width wide (Height
	 *  tall, or as tall as they need when 0), from the painter's origin down;
	 *  their height. Drawn unless bDraw is false. */
	using FPaintScope = TFunction<float(const FDebugPainter& Painter, float Width, float Height, bool bDraw)>;

	/** What a controller's view keeps between frames for its graphs, its
	 *  labels and the panel and overlay they are drawn in. Lives as long as the
	 *  controller (Owner below): the widgets reach it only while it does. */
	struct FDebugScope
	{
		/** The graphs' samples, oldest first. */
		TArray<FDebugGraphSample> Graph;

		/** This tick's labels. */
		TArray<FDebugLabel> Labels;

		/** The "Curb view" panel's widget, while a HUD holds it. */
		TWeakPtr<SWidget> PanelWidget;

		/** The labels, on the player's screen under the HUD's panels (the debug
		 *  canvas would draw them over the panels): a Slate overlay, there while
		 *  labels are (UpdateLabelOverlay). */
		TSharedPtr<SWidget>					LabelOverlay;
		TWeakObjectPtr<UGameViewportClient> LabelViewport;
		TWeakObjectPtr<ULocalPlayer>		LabelPlayer;

		/** A HUD hosts the key and graphs panel (its widget is alive). */
		bool IsPanelHosted() const { return PanelWidget.IsValid(); }
	};

	/** Scope's label overlay put on Owner's world's game viewport while
	 *  bWanted, and taken off when not. */
	void UpdateLabelOverlay(FDebugScope& Scope, const UObject& Owner, bool bWanted);

	/** The "Curb view" panel (IRammsHUDPanelProvider::GetHUDPanels): Paint's
	 *  key and graphs, shown while IsWanted. Its widget holds Owner weakly,
	 *  and is Scope's PanelWidget while a HUD holds it. */
	void AddScopePanel(TArray<FRammsHUDPanelDesc>& OutPanels, const UObject& Owner, FDebugScope& Scope, FPaintScope Paint,
		TFunction<bool()> IsWanted);

	/**
	 * The view over the game viewport (UDebugDrawService's "Game" draw), in
	 * Owner's world only: the labels when bLabels and no overlay has them, and
	 * Paint's key and graphs when bPanel and no HUD hosts them, their
	 * bottom-left corner at Anchor (a fraction of the viewport, 0,0 top left).
	 */
	void DrawScopeOverlay(const FDebugScope& Scope, const UObject& Owner, UCanvas* Canvas, APlayerController* Player, bool bLabels,
		bool bPanel, const FVector2D& Anchor, TFunctionRef<float(const FDebugPainter&, float, float, bool)> Paint);

	/** The debug view's layers, as bits: which "Curb view" switches a
	 *  controller offers. */
	namespace ViewLayer
	{
		inline constexpr uint8 Sensing = 1 << 0;
		inline constexpr uint8 Terrain = 1 << 1;
		inline constexpr uint8 Plan = 1 << 2;
		inline constexpr uint8 Robot = 1 << 3;
		inline constexpr uint8 Graphs = 1 << 4;
		inline constexpr uint8 Labels = 1 << 5;
	} // namespace ViewLayer

	/** A layer's switch (one ViewLayer bit): curb.view.sensing (0 Off, 1
	 *  Live, 2 Live + trail), .terrain, .plan, .robot, .graphs, .labels (0
	 *  off, 1 on) -- the Ids URammsCurbNegotiationController::View*ControlId
	 *  name. None for anything else. */
	FName ViewControlId(uint8 Layer);

	/** The switches of Layers, in the "Curb view" group, each showing how
	 *  View has it; none when View.bExposeControls is off. */
	void DescribeViewControls(FRammsControlSurface& OutSurface, const FRammsCurbDebugView& View, uint8 Layers);

	/** A switch of Layers set (an index past the end is the last); false for
	 *  any other Id. */
	bool ApplyViewControl(FRammsCurbDebugView& View, uint8 Layers, FName Id, float Value);

	/** How a switch of Layers is set; false for any other Id. */
	bool ReadViewControl(const FRammsCurbDebugView& View, uint8 Layers, FName Id, float& OutValue);
} // namespace RammsCurb
