#ifndef LEGEND_DISPLAY_H
#define LEGEND_DISPLAY_H

#ifndef Q_MOC_RUN
#include <memory>

#include <QPointer>
#include <QPointF>

#include <rviz_common/display.hpp>
#include <rviz_rendering/render_window.hpp>

#include <avt_341_rviz_plugins/primitives/legend_renderer.h>
#include <avt_341_rviz_plugins/primitives/screen_overlay.h>
#endif

namespace rviz_common {
namespace properties {
class BoolProperty;
class ColorProperty;
class EditableEnumProperty;
class EnumProperty;
class FloatProperty;
class IntProperty;
class StringProperty;
} // namespace properties
} // namespace rviz_common

namespace avt_341 {
namespace rviz_plugins {

class LegendItemListProperty;

/// RViz display that draws a configurable legend over the 3D view - no topic, it
/// is driven entirely by its properties. Entries are Lines (slim bars, e.g.
/// paths), Gradients (vertical color ramps spanning several rows) and Cells
/// (filled squares), laid out in rows that wrap into further columns past Max
/// Rows (see RenderLegend). Background, border, font, marker size and spacing are
/// all configurable.
///
/// The legend is a screen-space overlay (see ScreenOverlay): it sits at a fixed
/// pixel offset (X, Y) inward from the chosen Anchor corner of the main render
/// window and is unaffected by the camera. Resizing the window keeps it at that
/// corner. It can be dragged with the left mouse button; a drag rewrites X and Y
/// (still relative to the same corner). While the legend is draggable, left
/// clicks and mouse moves over it (and everything during a drag) are consumed, so
/// the camera and the active tool don't also react to them; all other input,
/// including the wheel, passes through.
///
/// The image is re-rendered at most once per frame, in update(), after any
/// property change (loading a config fires dozens) or when the window moves to a
/// screen with a different pixel ratio. Moving the legend never re-renders it.
class LegendDisplay : public rviz_common::Display
{
    Q_OBJECT

public:
    LegendDisplay();
    ~LegendDisplay() override;

    void onInitialize() override;
    void update( float wall_dt, float ros_dt ) override;
    void reset() override;

    /// Drags the legend; installed on the main render window.
    bool eventFilter( QObject* watched, QEvent* event ) override;

protected:
    void onEnable() override;
    void onDisable() override;

private Q_SLOTS:
    /// Content or style changed: re-render on the next update().
    void queueRedraw();
    /// Anchor / X / Y changed: move the legend (no re-render).
    void updatePlacement();
    /// Fill the font dropdown with the installed font families.
    void fillFontOptions( rviz_common::properties::EditableEnumProperty* property );

private:
    LegendStyle toStyle() const;
    ScreenCorner anchor() const;
    bool canDrag() const;
    /// Move the legend so the grabbed point follows the mouse, keeping it on
    /// screen, and store the result in X / Y relative to the anchor corner.
    void dragTo( const QPointF& mouse );
    void setHover( bool hover );
    void endDrag();
    /// Hand cursor while hovering (open) or dragging (closed); otherwise leave
    /// the cursor to the active tool.
    void applyCursor();

    std::unique_ptr<ScreenOverlay> overlay_;
    QPointer<rviz_rendering::RenderWindow> render_window_;

    bool dirty_ = true;
    double rendered_ratio_ = 0.0; ///< Pixel ratio of the current image.

    bool hover_ = false;
    bool dragging_ = false;
    bool cursor_overridden_ = false;
    QPointF drag_grab_; ///< Grabbed point, relative to the legend's top-left.

    rviz_common::properties::StringProperty* title_ = nullptr;

    // Position
    rviz_common::properties::EnumProperty* anchor_ = nullptr;
    rviz_common::properties::IntProperty*  x_ = nullptr;
    rviz_common::properties::IntProperty*  y_ = nullptr;
    rviz_common::properties::BoolProperty* draggable_ = nullptr;

    // Background
    rviz_common::properties::ColorProperty* background_color_ = nullptr;
    rviz_common::properties::FloatProperty* background_alpha_ = nullptr;
    rviz_common::properties::ColorProperty* border_color_ = nullptr;
    rviz_common::properties::IntProperty*   border_width_ = nullptr;
    rviz_common::properties::IntProperty*   corner_radius_ = nullptr;

    // Layout
    rviz_common::properties::IntProperty* max_rows_ = nullptr;
    rviz_common::properties::IntProperty* padding_ = nullptr;
    rviz_common::properties::IntProperty* row_spacing_ = nullptr;
    rviz_common::properties::IntProperty* column_spacing_ = nullptr;

    // Text
    rviz_common::properties::EditableEnumProperty* font_family_ = nullptr;
    rviz_common::properties::IntProperty*          font_size_ = nullptr;
    rviz_common::properties::BoolProperty*         bold_ = nullptr;
    rviz_common::properties::ColorProperty*        text_color_ = nullptr;

    // Markers
    rviz_common::properties::IntProperty*   marker_size_ = nullptr;
    rviz_common::properties::IntProperty*   label_gap_ = nullptr;
    rviz_common::properties::ColorProperty* outline_color_ = nullptr;
    rviz_common::properties::IntProperty*   outline_width_ = nullptr;

    LegendItemListProperty* items_ = nullptr;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // LEGEND_DISPLAY_H
