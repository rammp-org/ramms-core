// Copyright Epic Games, Inc. All Rights Reserved.

// What the curb controllers' debug views share (RammsCurbScope.h): the graphs'
// samples and strip charts, the colour key, the labels, the "Curb view" HUD
// panel and its fallback over the viewport, and the view's switches. One
// painter draws the key and the graphs both ways: into the panel the HUD hosts
// (IRammsHUDPanelProvider: SRammsCurbScope, Slate), or, with no HUD to host it,
// straight over the game viewport (UDebugDrawService's canvas).

#include "Curb/RammsCurbScope.h"

#include "CanvasItem.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/LocalPlayer.h"
#include "Engine/World.h"
#include "Fonts/FontMeasure.h"
#include "Framework/Application/SlateApplication.h"
#include "GameFramework/PlayerController.h"
#include "GlobalRenderResources.h"
#include "RammsControlTypes.h"
#include "RammsCurbNegotiationTypes.h"
#include "RammsHUDPanelProvider.h"
#include "Rendering/DrawElements.h"
#include "Rendering/SlateRenderer.h"
#include "RenderUtils.h"
#include "SceneInterface.h"
#include "SceneView.h"
#include "Styling/CoreStyle.h"
#include "Widgets/SLeafWidget.h"

// --- the sums the graphs and the support polygon are drawn from (tested in RammsCurbDebugViewTests) ---

namespace RammsCurb
{
	void AddGraphSample(TArray<FDebugGraphSample>& Graph, const FDebugGraphSample& Sample, double SpanSeconds)
	{
		if (Graph.Num() > 0 && Sample.T < Graph.Last().T)
		{
			Graph.Reset(); // the simulation was reset
		}
		Graph.Add(Sample);
		const double From = Sample.T - SpanSeconds;
		int32		 Old = 0;
		while (Old < Graph.Num() && Graph[Old].T < From)
		{
			++Old;
		}
		Graph.RemoveAt(0, Old, EAllowShrinking::No);
	}

	FVector2D GraphRange(TConstArrayView<FDebugGraphSample> Samples, TConstArrayView<EGraphSeries> Series,
		TConstArrayView<float> Always, float MinSpan)
	{
		float Lo = TNumericLimits<float>::Max(), Hi = -TNumericLimits<float>::Max();
		for (const FDebugGraphSample& S : Samples)
		{
			for (const EGraphSeries Which : Series)
			{
				if (S.Has(Which))
				{
					Lo = FMath::Min(Lo, S.Get(Which));
					Hi = FMath::Max(Hi, S.Get(Which));
				}
			}
		}
		for (const float A : Always)
		{
			Lo = FMath::Min(Lo, A);
			Hi = FMath::Max(Hi, A);
		}
		if (Lo > Hi)
		{
			return FVector2D(0.0, MinSpan);
		}
		const float Pad = 0.08f * (Hi - Lo);
		Lo -= Pad;
		Hi += Pad;
		if (Hi - Lo < MinSpan)
		{
			const float Mid = 0.5f * (Lo + Hi);
			Lo = Mid - 0.5f * MinSpan;
			Hi = Mid + 0.5f * MinSpan;
		}
		return FVector2D(Lo, Hi);
	}

	TArray<int32> ConvexHull2D(TConstArrayView<FVector2D> Points)
	{
		TArray<int32> Order;
		for (int32 i = 0; i < Points.Num(); ++i)
		{
			Order.Add(i);
		}
		Order.Sort([&Points](int32 A, int32 B) {
			return Points[A].X < Points[B].X || (Points[A].X == Points[B].X && Points[A].Y < Points[B].Y);
		});
		// Distinct points only.
		TArray<int32> P;
		for (const int32 i : Order)
		{
			if (P.Num() == 0 || !Points[P.Last()].Equals(Points[i], 1e-6))
			{
				P.Add(i);
			}
		}
		if (P.Num() < 3)
		{
			return P;
		}
		auto Cross = [&Points](int32 O, int32 A, int32 B) {
			return FVector2D::CrossProduct(Points[A] - Points[O], Points[B] - Points[O]);
		};
		TArray<int32> Hull;
		Hull.SetNum(2 * P.Num());
		int32 K = 0;
		for (int32 i = 0; i < P.Num(); ++i)
		{
			while (K >= 2 && Cross(Hull[K - 2], Hull[K - 1], P[i]) <= 0.0)
			{
				--K;
			}
			Hull[K++] = P[i];
		}
		for (int32 i = P.Num() - 2, Lower = K + 1; i >= 0; --i)
		{
			while (K >= Lower && Cross(Hull[K - 2], Hull[K - 1], P[i]) <= 0.0)
			{
				--K;
			}
			Hull[K++] = P[i];
		}
		Hull.SetNum(K - 1);
		return Hull;
	}

	// --- the colour key, the graphs, the labels -------------------------------------------------------

	float DrawKey(const FDebugPainter& P, float X, float Top, float Width, TConstArrayView<FKeyRow> Rows, bool bDraw)
	{
		float Y = Top;
		for (const FKeyRow& Row : Rows)
		{
			if (bDraw)
			{
				P.Text(X, Y, Row.Title, FLinearColor(0.6f, 0.65f, 0.7f));
			}
			float Cx = P.Width(Row.Title) + 10.0f;
			for (const FKeyChip& Chip : Row.Chips)
			{
				const float W = 14.0f + P.Width(Chip.Words) + 12.0f;
				if (Cx + W > Width && Cx > 18.0f)
				{
					Y += P.LineHeight + 2.0f;
					Cx = 18.0f;
				}
				if (bDraw)
				{
					P.Box(X + Cx, Y + 3.0f, 10.0f, 10.0f, FLinearColor(Chip.Colour));
					P.Text(X + Cx + 14.0f, Y, Chip.Words, FLinearColor::White);
				}
				Cx += W;
			}
			Y += P.LineHeight + 4.0f;
		}
		return Y - Top;
	}

	float DrawGraph(const FDebugPainter& P, float X, float Y, float Width, float Height, const TCHAR* Title,
		TConstArrayView<FDebugGraphSample> Samples, double TFrom, double Span, TConstArrayView<FGraphLine> Lines,
		TConstArrayView<FGraphMark> Marks, float MinSpan, const TCHAR* Units)
	{
		TArray<EGraphSeries> Series;
		for (const FGraphLine& L : Lines)
		{
			Series.Add(L.Series);
		}
		TArray<float> Always;
		for (const FGraphMark& M : Marks)
		{
			Always.Add(M.Value);
		}
		const FVector2D Range = GraphRange(Samples, Series, Always, MinSpan);

		float Tx = X + P.Text(X, Y, Title, FLinearColor(0.75f, 0.8f, 0.85f)) + 16.0f;
		for (const FGraphLine& L : Lines)
		{
			if (!L.Name)
			{
				continue;
			}
			for (int32 k = Samples.Num() - 1; k >= 0; --k)
			{
				if (Samples[k].Has(L.Series))
				{
					Tx += P.Text(Tx, Y, FString::Printf(TEXT("%s %.1f"), L.Name, Samples[k].Get(L.Series)), L.Colour) + 12.0f;
					break;
				}
			}
		}
		const float Top = Y + P.LineHeight + 2.0f;
		auto		ToX = [&](double T) { return X + static_cast<float>((T - TFrom) / Span) * Width; };
		auto		ToY = [&](float V) { return Top + Height - static_cast<float>((V - Range.X) / (Range.Y - Range.X)) * Height; };
		P.Box(X, Top, Width, Height, FLinearColor(0.02f, 0.03f, 0.04f, 0.72f));
		P.Text(X + 3.0f, Top + 1.0f, FString::Printf(TEXT("%.0f %s"), Range.Y, Units), FLinearColor(0.5f, 0.55f, 0.6f));
		P.Text(X + 3.0f, Top + Height - P.LineHeight - 1.0f, FString::Printf(TEXT("%.0f"), Range.X), FLinearColor(0.5f, 0.55f, 0.6f));
		for (const FGraphMark& M : Marks)
		{
			P.Dashed(X, X + Width, ToY(M.Value), M.Colour);
		}
		// At most a sample a pixel.
		const int32 Stride = FMath::Max(1, Samples.Num() / FMath::Max(1, FMath::RoundToInt(Width)));
		for (const FGraphLine& L : Lines)
		{
			bool  bHave = false;
			float Px = 0.0f, Py = 0.0f;
			for (int32 k = 0; k < Samples.Num(); k += Stride)
			{
				const FDebugGraphSample& S = Samples[k];
				if (!S.Has(L.Series))
				{
					bHave = false;
					continue;
				}
				const float Nx = ToX(S.T), Ny = ToY(FMath::Clamp(S.Get(L.Series), static_cast<float>(Range.X), static_cast<float>(Range.Y)));
				if (bHave)
				{
					P.Line(Px, Py, Nx, Ny, L.Colour, L.bDashed ? 1.0f : 2.0f);
				}
				Px = Nx;
				Py = Ny;
				bHave = true;
			}
		}
		return Top + Height;
	}

	void PaintLabels(const FDebugPainter& P, TConstArrayView<FDebugLabel> Labels, TFunctionRef<bool(const FVector&, FVector2D&)> Project)
	{
		// Each label's lines centred over its point, on a dark band.
		for (const FDebugLabel& L : Labels)
		{
			FVector2D Screen;
			if (!Project(L.At, Screen))
			{
				continue;
			}
			TArray<FString> Lines;
			L.Text.ParseIntoArrayLines(Lines, /*bCullEmpty=*/false);
			float Widest = 0.0f;
			for (const FString& Line : Lines)
			{
				Widest = FMath::Max(Widest, P.Width(Line));
			}
			const float Height = Lines.Num() * P.LineHeight;
			const float Left = static_cast<float>(Screen.X) - 0.5f * Widest;
			const float Top = static_cast<float>(Screen.Y) - Height;
			P.Box(Left - 3.0f, Top - 1.0f, Widest + 6.0f, Height + 2.0f, FLinearColor(0.0f, 0.0f, 0.0f, 0.45f));
			for (int32 k = 0; k < Lines.Num(); ++k)
			{
				P.Text(Left, Top + k * P.LineHeight, Lines[k], FLinearColor(L.Colour));
			}
		}
	}
} // namespace RammsCurb

// --- the painters ---------------------------------------------------------------------------------

namespace
{
	/** Over the game viewport. */
	struct FCurbScopeCanvasPainter : RammsCurb::FDebugPainter
	{
		UCanvas*	 Canvas = nullptr;
		const UFont* Font = nullptr;
		FVector2D	 Origin = FVector2D::ZeroVector;

		virtual void Box(float X, float Y, float W, float H, const FLinearColor& Colour) const override
		{
			FCanvasTileItem Tile(Origin + FVector2D(X, Y), GWhiteTexture, FVector2D(W, H), Colour);
			Tile.BlendMode = SE_BLEND_Translucent;
			Canvas->DrawItem(Tile);
		}
		virtual void Line(float X0, float Y0, float X1, float Y1, const FLinearColor& Colour, float Thickness) const override
		{
			FCanvasLineItem L(Origin + FVector2D(X0, Y0), Origin + FVector2D(X1, Y1));
			L.SetColor(Colour);
			L.LineThickness = Thickness;
			Canvas->DrawItem(L);
		}
		virtual float Text(float X, float Y, const FString& S, const FLinearColor& Colour) const override
		{
			FCanvasTextItem Item(Origin + FVector2D(X, Y), FText::FromString(S), Font, Colour);
			Item.EnableShadow(FLinearColor::Black);
			Canvas->DrawItem(Item);
			return Width(S);
		}
		virtual float Width(const FString& S) const override
		{
			float W = 0.0f, H = 0.0f;
			Canvas->TextSize(Font, S, W, H);
			return W;
		}
	};

	/** Into a Slate widget's paint. Each primitive goes on a layer of its own,
	 *  so they stack in the order drawn. */
	struct FCurbScopeSlatePainter : RammsCurb::FDebugPainter
	{
		const FGeometry*		 Geometry = nullptr;
		FSlateWindowElementList* Elements = nullptr;
		FSlateFontInfo			 Font;
		mutable int32			 Layer = 0;

		virtual void Box(float X, float Y, float W, float H, const FLinearColor& Colour) const override
		{
			FSlateDrawElement::MakeBox(*Elements, ++Layer,
				Geometry->ToPaintGeometry(FVector2f(W, H), FSlateLayoutTransform(FVector2f(X, Y))),
				FCoreStyle::Get().GetBrush(TEXT("WhiteBrush")), ESlateDrawEffect::None, Colour);
		}
		virtual void Line(float X0, float Y0, float X1, float Y1, const FLinearColor& Colour, float Thickness) const override
		{
			TArray<FVector2f> Points{ FVector2f(X0, Y0), FVector2f(X1, Y1) };
			FSlateDrawElement::MakeLines(*Elements, ++Layer, Geometry->ToPaintGeometry(), MoveTemp(Points), ESlateDrawEffect::None, Colour,
				/*bAntialias=*/true, Thickness);
		}
		virtual float Text(float X, float Y, const FString& S, const FLinearColor& Colour) const override
		{
			const float W = Width(S);
			FSlateDrawElement::MakeText(*Elements, ++Layer,
				Geometry->ToPaintGeometry(FVector2f(W + 2.0f, LineHeight), FSlateLayoutTransform(FVector2f(X, Y))), S, Font,
				ESlateDrawEffect::None, Colour);
			return W;
		}
		virtual float Width(const FString& S) const override
		{
			return static_cast<float>(FSlateApplication::Get().GetRenderer()->GetFontMeasureService()->Measure(S, Font).X);
		}
	};
} // namespace

// --- the HUD panel and the labels' overlay --------------------------------------------------------

/** A curb controller's colour key and graphs, as a HUD panel. */
class SRammsCurbScope : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SRammsCurbScope) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, TWeakObjectPtr<const UObject> InOwner, RammsCurb::FPaintScope InPaint)
	{
		OwnerObject = InOwner;
		PaintFn = MoveTemp(InPaint);
		Font = FCoreStyle::GetDefaultFontStyle("Regular", 10);
		SetCanTick(false);
	}

	virtual FVector2D ComputeDesiredSize(float) const override
	{
		if (!OwnerObject.IsValid() || !PaintFn || !FSlateApplication::IsInitialized())
		{
			return FVector2D(DefaultWidth, 40.0);
		}
		FCurbScopeSlatePainter P;
		P.Font = Font;
		P.LineHeight = LineHeight;
		// As tall as it needs at the width it was last given (the key wraps).
		const float Width = LastWidth > 1.0f ? LastWidth : DefaultWidth;
		return FVector2D(DefaultWidth, PaintFn(P, Width, /*Height=*/0.0f, /*bDraw=*/false));
	}

	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override
	{
		FCurbScopeSlatePainter P;
		P.Geometry = &AllottedGeometry;
		P.Elements = &OutDrawElements;
		P.Font = Font;
		P.LineHeight = LineHeight;
		P.Layer = LayerId;
		const FVector2f Size = AllottedGeometry.GetLocalSize();
		LastWidth = Size.X;
		if (OwnerObject.IsValid() && PaintFn)
		{
			PaintFn(P, Size.X, Size.Y, /*bDraw=*/true);
		}
		else
		{
			P.Text(4.0f, 4.0f, TEXT("no curb controller"), FLinearColor(0.6f, 0.6f, 0.6f));
		}
		return P.Layer;
	}

private:
	static constexpr float DefaultWidth = 480.0f;
	static constexpr float LineHeight = 17.0f;

	TWeakObjectPtr<const UObject> OwnerObject;
	RammsCurb::FPaintScope		  PaintFn;
	FSlateFontInfo				  Font;
	mutable float				  LastWidth = 0.0f;

	// Repainted every frame: the graphs move with the simulation.
	virtual bool ComputeVolatility() const override { return true; }
};

/** A curb controller's world-layer labels, on the player's screen under the HUD. */
class SRammsCurbLabels : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SRammsCurbLabels) {}
	SLATE_END_ARGS()

	/** InScope lives as long as InOwner does: it is read only while InOwner is. */
	void Construct(const FArguments& InArgs, TWeakObjectPtr<const UObject> InOwner, const RammsCurb::FDebugScope* InScope)
	{
		OwnerObject = InOwner;
		Scope = InScope;
		Font = FCoreStyle::GetDefaultFontStyle("Regular", 9);
		SetCanTick(false);
		SetVisibility(EVisibility::HitTestInvisible);
	}

	virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D::ZeroVector; }

	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override
	{
		const UObject*	   O = OwnerObject.Get();
		const UWorld*	   World = O ? O->GetWorld() : nullptr;
		APlayerController* Player = World ? World->GetFirstPlayerController() : nullptr;
		if (!Player || !Scope)
		{
			return LayerId;
		}
		int32 PixelsX = 0, PixelsY = 0;
		Player->GetViewportSize(PixelsX, PixelsY);
		const FVector2f Local = AllottedGeometry.GetLocalSize();
		if (PixelsX <= 0 || PixelsY <= 0)
		{
			return LayerId;
		}
		const FVector2D		   ToLocal(Local.X / PixelsX, Local.Y / PixelsY);
		FCurbScopeSlatePainter P;
		P.Geometry = &AllottedGeometry;
		P.Elements = &OutDrawElements;
		P.Font = Font;
		P.Layer = LayerId;
		RammsCurb::PaintLabels(P, Scope->Labels, [Player, ToLocal, Local](const FVector& At, FVector2D& Out) {
			FVector2D Screen;
			if (!Player->ProjectWorldLocationToScreen(At, Screen, /*bPlayerViewportRelative=*/true))
			{
				return false;
			}
			Out = Screen * ToLocal;
			return Out.X >= 0.0 && Out.Y >= 0.0 && Out.X <= Local.X && Out.Y <= Local.Y;
		});
		return P.Layer;
	}

private:
	TWeakObjectPtr<const UObject> OwnerObject;
	const RammsCurb::FDebugScope* Scope = nullptr;
	FSlateFontInfo				  Font;

	virtual bool ComputeVolatility() const override { return true; }
};

namespace RammsCurb
{
	void UpdateLabelOverlay(FDebugScope& Scope, const UObject& Owner, bool bWanted)
	{
		if (bWanted && !Scope.LabelOverlay.IsValid())
		{
			UWorld*				 World = Owner.GetWorld();
			UGameViewportClient* Viewport = World ? World->GetGameViewport() : nullptr;
			ULocalPlayer*		 Player = World ? World->GetFirstLocalPlayerFromController() : nullptr;
			if (Viewport && Player)
			{
				// Under the HUD, which takes the player's screen at 10.
				Scope.LabelOverlay = SNew(SRammsCurbLabels, TWeakObjectPtr<const UObject>(&Owner), &Scope);
				Viewport->AddViewportWidgetForPlayer(Player, Scope.LabelOverlay.ToSharedRef(), 5);
				Scope.LabelViewport = Viewport;
				Scope.LabelPlayer = Player;
			}
		}
		else if (!bWanted && Scope.LabelOverlay.IsValid())
		{
			UGameViewportClient* Viewport = Scope.LabelViewport.Get();
			ULocalPlayer*		 Player = Scope.LabelPlayer.Get();
			if (Viewport && Player)
			{
				Viewport->RemoveViewportWidgetForPlayer(Player, Scope.LabelOverlay.ToSharedRef());
			}
			Scope.LabelOverlay.Reset();
			Scope.LabelViewport.Reset();
			Scope.LabelPlayer.Reset();
		}
	}

	void AddScopePanel(TArray<FRammsHUDPanelDesc>& OutPanels, const UObject& Owner, FDebugScope& Scope, FPaintScope Paint,
		TFunction<bool()> IsWanted)
	{
		FRammsHUDPanelDesc& Panel = OutPanels.AddDefaulted_GetRef();
		Panel.Id = FName(TEXT("CurbView"));
		Panel.Title = NSLOCTEXT("Ramms", "CurbViewPanel", "Curb view");
		const TWeakObjectPtr<const UObject> Self(&Owner);
		FDebugScope*						ScopePtr = &Scope;
		Panel.MakeContent = [Self, ScopePtr, Paint = MoveTemp(Paint)]() -> TSharedRef<SWidget> {
			TSharedRef<SRammsCurbScope> Widget = SNew(SRammsCurbScope, Self, Paint);
			if (Self.IsValid())
			{
				ScopePtr->PanelWidget = Widget;
			}
			return Widget;
		};
		// Shown while it has something to show: the graphs, or the colour key of
		// a layer drawn in the world.
		Panel.IsWanted = MoveTemp(IsWanted);
		Panel.DefaultPlacement = FVector2D(0.28, 1.0);
		Panel.bResizable = true;
	}

	void DrawScopeOverlay(const FDebugScope& Scope, const UObject& Owner, UCanvas* Canvas, APlayerController* Player, bool bLabels,
		bool bPanel, const FVector2D& Anchor, TFunctionRef<float(const FDebugPainter&, float, float, bool)> Paint)
	{
		if (!Canvas || !GEngine)
		{
			return;
		}
		// Only the viewport of the controller's world.
		const UWorld* World = Owner.GetWorld();
		if (Player ? Player->GetWorld() != World
				   : !(Canvas->SceneView && Canvas->SceneView->Family && Canvas->SceneView->Family->Scene
						 && Canvas->SceneView->Family->Scene->GetWorld() == World))
		{
			return;
		}
		FCurbScopeCanvasPainter P;
		P.Canvas = Canvas;
		P.Font = GEngine->GetSmallFont();
		P.LineHeight = 15.0f;

		// The world layers' labels, here only without the overlay under the HUD.
		if (bLabels && !Scope.LabelOverlay.IsValid())
		{
			PaintLabels(P, Scope.Labels, [Canvas](const FVector& At, FVector2D& Out) {
				const FVector Screen = Canvas->Project(At);
				Out = FVector2D(Screen.X, Screen.Y);
				return Screen.Z > 0.0 && Screen.X >= 0.0 && Screen.Y >= 0.0 && Screen.X <= Canvas->ClipX && Screen.Y <= Canvas->ClipY;
			});
		}

		// The key and the graphs, here only while no HUD hosts them as a panel.
		if (Scope.IsPanelHosted() || !bPanel)
		{
			return;
		}
		const float Width = FMath::Clamp(Canvas->ClipX * 0.36f, 300.0f, 560.0f);
		const float Height = Paint(P, Width, 0.0f, /*bDraw=*/false);
		P.Origin = FVector2D(FMath::Clamp(static_cast<float>(Anchor.X), 0.0f, 0.9f) * Canvas->ClipX,
			FMath::Clamp(static_cast<float>(Anchor.Y), 0.1f, 1.0f) * Canvas->ClipY - 6.0f - Height);
		P.Box(-6.0f, -6.0f, Width + 12.0f, Height + 10.0f, FLinearColor(0.02f, 0.03f, 0.04f, 0.6f));
		Paint(P, Width, 0.0f, /*bDraw=*/true);
	}

	// --- the "Curb view" switches -----------------------------------------------------------------------

	FName ViewControlId(uint8 Layer)
	{
		switch (Layer)
		{
			case ViewLayer::Sensing:
				return FName(TEXT("curb.view.sensing"));
			case ViewLayer::Terrain:
				return FName(TEXT("curb.view.terrain"));
			case ViewLayer::Plan:
				return FName(TEXT("curb.view.plan"));
			case ViewLayer::Robot:
				return FName(TEXT("curb.view.robot"));
			case ViewLayer::Graphs:
				return FName(TEXT("curb.view.graphs"));
			case ViewLayer::Labels:
				return FName(TEXT("curb.view.labels"));
			default:
				return NAME_None;
		}
	}

	namespace
	{
		/** The on/off switch Id names among Layers, else null (Sensing is not one: it has three settings). */
		template <typename TView>
		auto CurbViewFlag(TView& View, uint8 Layers, FName Id) -> decltype(&View.bTerrain)
		{
			const auto Is = [Layers, Id](uint8 Layer) { return (Layers & Layer) != 0 && Id == ViewControlId(Layer); };
			return Is(ViewLayer::Terrain) ? &View.bTerrain
				: Is(ViewLayer::Plan)	  ? &View.bPlan
				: Is(ViewLayer::Robot)	  ? &View.bRobot
				: Is(ViewLayer::Graphs)	  ? &View.bGraphs
				: Is(ViewLayer::Labels)	  ? &View.bLabels
										  : nullptr;
		}
	} // namespace

	void DescribeViewControls(FRammsControlSurface& OutSurface, const FRammsCurbDebugView& View, uint8 Layers)
	{
		if (!View.bExposeControls)
		{
			return;
		}
		const FName Group(TEXT("Curb view"));
		auto		AddSwitch = [&](uint8 Layer, const FText& Name, int32 Order, TArray<FText> Labels) {
			   if ((Layers & Layer) == 0)
			   {
				   return;
			   }
			   FRammsControlAxis Switch;
			   Switch.Id = ViewControlId(Layer);
			   Switch.Group = Group;
			   Switch.DisplayName = Name;
			   Switch.Kind = ERammsControlKind::Enum;
			   Switch.Units = ERammsControlUnits::None;
			   Switch.Order = Order;
			   Switch.Range = FVector2D(0.0, FMath::Max(0, Labels.Num() - 1));
			   float Now = 0.0f;
			   ReadViewControl(View, Layers, Switch.Id, Now);
			   Switch.DefaultValue = Now;
			   Switch.EnumLabels = MoveTemp(Labels);
			   OutSurface.Add(Switch);
		};
		const FText Off = NSLOCTEXT("Ramms", "CurbViewOff", "Off");
		const FText On = NSLOCTEXT("Ramms", "CurbViewOn", "On");
		AddSwitch(ViewLayer::Sensing, NSLOCTEXT("Ramms", "CurbViewSensing", "Sensing"), 0,
			{ Off, NSLOCTEXT("Ramms", "CurbViewLive", "Live"), NSLOCTEXT("Ramms", "CurbViewTrail", "Live + trail") });
		AddSwitch(ViewLayer::Terrain, NSLOCTEXT("Ramms", "CurbViewTerrain", "Terrain model"), 1, { Off, On });
		AddSwitch(ViewLayer::Plan, NSLOCTEXT("Ramms", "CurbViewPlan", "Plan"), 2, { Off, On });
		AddSwitch(ViewLayer::Robot, NSLOCTEXT("Ramms", "CurbViewRobot", "Robot state"), 3, { Off, On });
		AddSwitch(ViewLayer::Graphs, NSLOCTEXT("Ramms", "CurbViewGraphs", "Graphs"), 4, { Off, On });
		AddSwitch(ViewLayer::Labels, NSLOCTEXT("Ramms", "CurbViewLabels", "Labels"), 5, { Off, On });
	}

	bool ApplyViewControl(FRammsCurbDebugView& View, uint8 Layers, FName Id, float Value)
	{
		const int32 Index = FMath::RoundToInt(Value);
		if ((Layers & ViewLayer::Sensing) != 0 && Id == ViewControlId(ViewLayer::Sensing))
		{
			View.Sensing = static_cast<ERammsCurbSensingView>(FMath::Clamp(Index, 0, 2));
			return true;
		}
		if (bool* Flag = CurbViewFlag(View, Layers, Id))
		{
			*Flag = Index != 0;
			return true;
		}
		return false;
	}

	bool ReadViewControl(const FRammsCurbDebugView& View, uint8 Layers, FName Id, float& OutValue)
	{
		if ((Layers & ViewLayer::Sensing) != 0 && Id == ViewControlId(ViewLayer::Sensing))
		{
			OutValue = static_cast<float>(View.Sensing);
			return true;
		}
		if (const bool* Flag = CurbViewFlag(View, Layers, Id))
		{
			OutValue = *Flag ? 1.0f : 0.0f;
			return true;
		}
		return false;
	}
} // namespace RammsCurb
