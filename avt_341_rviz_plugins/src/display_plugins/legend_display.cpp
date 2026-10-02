#include <avt_341_rviz_plugins/display_plugins/legend_display.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include <QColor>
#include <QEvent>
#include <QFontDatabase>
#include <QGuiApplication>
#include <QMouseEvent>
#include <QVariant>

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>

#include <avt_341_rviz_plugins/display_plugins/legend_properties.h>

namespace avt_341::rviz_plugins
{

using rviz_common::properties::BoolProperty;
using rviz_common::properties::ColorProperty;
using rviz_common::properties::EditableEnumProperty;
using rviz_common::properties::EnumProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::IntProperty;
using rviz_common::properties::Property;
using rviz_common::properties::StatusProperty;
using rviz_common::properties::StringProperty;

namespace
{
/// The entries a newly added Legend starts with: the stack's path / map legend.
std::vector<LegendItem> defaultItems()
{
    LegendItem global_path;
    global_path.type = LegendItemType::Line;
    global_path.label = "Global Path";
    global_path.color = QColor( 46, 49, 146 );

    LegendItem local_path;
    local_path.type = LegendItemType::Line;
    local_path.label = "Local Path";
    local_path.color = QColor( 237, 28, 36 );

    LegendItem traversability;
    traversability.type = LegendItemType::Gradient;
    traversability.label = "Traversability";
    traversability.color = QColor( 255, 204, 0 );
    traversability.top_label = "High traversability";
    traversability.bottom_label = "Low traversability";
    traversability.rows = 4;
    traversability.stops = { QColor( 0, 166, 81 ), QColor( 255, 204, 0 ),
                             QColor( 247, 148, 29 ), QColor( 239, 65, 54 ) };

    LegendItem obstacles;
    obstacles.type = LegendItemType::Cell;
    obstacles.label = "Obstacles";
    obstacles.color = QColor( 0, 0, 0 );

    return { global_path, local_path, traversability, obstacles };
}
} // namespace

LegendDisplay::LegendDisplay()
{
    title_ = new StringProperty(
        "Title", "", "Optional bold heading above the entries.",
        this, SLOT( queueRedraw() ), this );

    // --- Position ------------------------------------------------------------
    Property* position = new Property(
        "Position", QVariant(),
        "Where the legend sits in the 3D view. It is anchored to a corner of the view, "
        "not to a place in the world, so the camera never moves or scales it.",
        this );
    anchor_ = new EnumProperty(
        "Anchor", "Top Left",
        "Corner of the 3D view the legend is anchored to. X and Y are measured inward "
        "from this corner, and the legend stays at it when the window is resized.",
        position, SLOT( updatePlacement() ), this );
    anchor_->addOption( "Top Left", static_cast<int>( ScreenCorner::TopLeft ) );
    anchor_->addOption( "Top Right", static_cast<int>( ScreenCorner::TopRight ) );
    anchor_->addOption( "Bottom Left", static_cast<int>( ScreenCorner::BottomLeft ) );
    anchor_->addOption( "Bottom Right", static_cast<int>( ScreenCorner::BottomRight ) );
    x_ = new IntProperty(
        "X", 20,
        "Horizontal distance (px) from the anchor corner's side of the view to the "
        "legend's nearest edge. Updated when the legend is dragged.",
        position, SLOT( updatePlacement() ), this );
    x_->setMin( 0 );
    y_ = new IntProperty(
        "Y", 20,
        "Vertical distance (px) from the anchor corner's top or bottom edge of the view "
        "to the legend's nearest edge. Updated when the legend is dragged.",
        position, SLOT( updatePlacement() ), this );
    y_->setMin( 0 );
    draggable_ = new BoolProperty(
        "Draggable", true,
        "Move the legend by dragging it with the left mouse button. Left clicks on the "
        "legend then don't reach the camera or the active tool.",
        position );

    // --- Background ----------------------------------------------------------
    Property* background = new Property(
        "Background", QVariant(), "The legend box.", this );
    background_color_ = new ColorProperty(
        "Color", QColor( 255, 255, 255 ), "Fill color of the legend box.",
        background, SLOT( queueRedraw() ), this );
    background_alpha_ = new FloatProperty(
        "Alpha", 0.85f, "Opacity of the box fill (0 = transparent).",
        background, SLOT( queueRedraw() ), this );
    background_alpha_->setMin( 0.0f );
    background_alpha_->setMax( 1.0f );
    border_color_ = new ColorProperty(
        "Border Color", QColor( 64, 64, 64 ), "Color of the box border.",
        background, SLOT( queueRedraw() ), this );
    border_width_ = new IntProperty(
        "Border Width", 1, "Thickness of the box border (px); 0 for none.",
        background, SLOT( queueRedraw() ), this );
    border_width_->setMin( 0 );
    corner_radius_ = new IntProperty(
        "Corner Radius", 4, "Rounding of the box corners (px); 0 for square corners.",
        background, SLOT( queueRedraw() ), this );
    corner_radius_->setMin( 0 );

    // --- Layout --------------------------------------------------------------
    Property* layout = new Property(
        "Layout", QVariant(), "Arrangement of the entries.", this );
    max_rows_ = new IntProperty(
        "Max Rows", 10,
        "Rows per column. Every entry takes one row except a gradient, which takes its "
        "Rows; an entry that doesn't fit in the current column starts a new column to "
        "the right.",
        layout, SLOT( queueRedraw() ), this );
    max_rows_->setMin( 1 );
    padding_ = new IntProperty(
        "Padding", 10, "Space between the border and the entries (px).",
        layout, SLOT( queueRedraw() ), this );
    padding_->setMin( 0 );
    row_spacing_ = new IntProperty(
        "Row Spacing", 6, "Vertical space between rows (px).",
        layout, SLOT( queueRedraw() ), this );
    row_spacing_->setMin( 0 );
    column_spacing_ = new IntProperty(
        "Column Spacing", 20, "Horizontal space between columns (px).",
        layout, SLOT( queueRedraw() ), this );
    column_spacing_->setMin( 0 );

    // --- Text ----------------------------------------------------------------
    Property* text = new Property(
        "Text", QVariant(), "Labels and title.", this );
    font_family_ = new EditableEnumProperty(
        "Font", QGuiApplication::font().family(), // RViz's own UI font
        "Font family. Pick an installed font or type a name; an unknown name falls back "
        "to a similar font.",
        text, SLOT( queueRedraw() ), this );
    connect( font_family_, &EditableEnumProperty::requestOptions,
             this, &LegendDisplay::fillFontOptions );
    font_size_ = new IntProperty(
        "Size", 14, "Text height (px).",
        text, SLOT( queueRedraw() ), this );
    font_size_->setMin( 1 );
    bold_ = new BoolProperty(
        "Bold", false, "Bold labels (the title is always bold).",
        text, SLOT( queueRedraw() ), this );
    text_color_ = new ColorProperty(
        "Color", QColor( 0, 0, 0 ), "Color of the labels and title.",
        text, SLOT( queueRedraw() ), this );

    // --- Markers -------------------------------------------------------------
    Property* markers = new Property(
        "Markers", QVariant(), "The swatches left of the labels.", this );
    marker_size_ = new IntProperty(
        "Size", 24,
        "Width of every marker and side of the cell squares (px). Rows are at least "
        "this tall.",
        markers, SLOT( queueRedraw() ), this );
    marker_size_->setMin( 1 );
    label_gap_ = new IntProperty(
        "Label Gap", 10, "Space between a marker and its label (px).",
        markers, SLOT( queueRedraw() ), this );
    label_gap_->setMin( 0 );
    outline_color_ = new ColorProperty(
        "Outline Color", QColor( 0, 0, 0 ), "Color of the marker outlines.",
        markers, SLOT( queueRedraw() ), this );
    outline_width_ = new IntProperty(
        "Outline Width", 1,
        "Thickness of the marker outlines, drawn inside the marker (px); 0 for none.",
        markers, SLOT( queueRedraw() ), this );
    outline_width_->setMin( 0 );

    // --- Entries -------------------------------------------------------------
    items_ = new LegendItemListProperty(
        "Items", defaultItems(), this, this, SLOT( queueRedraw() ) );
}

LegendDisplay::~LegendDisplay()
{
    if ( render_window_ )
    {
        render_window_->removeEventFilter( this );
        if ( cursor_overridden_ )
        {
            render_window_->unsetCursor();
        }
    }
}

void LegendDisplay::onInitialize()
{
    Display::onInitialize();

    rviz_common::RenderPanel* panel = context_->getViewManager()->getRenderPanel();
    render_window_ = panel != nullptr ? panel->getRenderWindow() : nullptr;
    if ( !render_window_ )
    {
        setStatus( StatusProperty::Error, "Viewport", "No main render window to draw the legend on." );
        return;
    }

    overlay_ = std::make_unique<ScreenOverlay>( scene_manager_, render_window_ );
    overlay_->setVisible( isEnabled() );
    updatePlacement();
    render_window_->installEventFilter( this );
}

void LegendDisplay::onEnable()
{
    if ( !overlay_ )
    {
        return;
    }
    overlay_->setVisible( true );
    dirty_ = true;
    context_->queueRender();
}

void LegendDisplay::onDisable()
{
    dragging_ = false;
    hover_ = false;
    applyCursor();
    if ( !overlay_ )
    {
        return;
    }
    overlay_->setVisible( false );
    context_->queueRender();
}

void LegendDisplay::reset()
{
    Display::reset();
    dirty_ = true;
}

void LegendDisplay::update( float /*wall_dt*/, float /*ros_dt*/ )
{
    if ( !overlay_ )
    {
        return;
    }

    // Re-render after edits, or when the window moved to a screen with another
    // pixel ratio (the image is rendered at the screen's resolution).
    const double ratio = overlay_->pixelRatio();
    if ( !dirty_ && ratio == rendered_ratio_ )
    {
        return;
    }
    dirty_ = false;
    rendered_ratio_ = ratio;

    const std::vector<LegendItem> items = items_->visibleItems();
    const QImage image = RenderLegend( toStyle(), items, ratio );
    if ( !overlay_->setImage( image ) )
    {
        setStatus( StatusProperty::Error, "Legend",
                   QString( "Could not create a %1 x %2 px texture for the legend." )
                       .arg( image.width() ).arg( image.height() ) );
    }
    else if ( image.isNull() )
    {
        setStatus( StatusProperty::Warn, "Legend",
                   "Nothing to draw: add or check an entry under Items, or set a Title." );
    }
    else
    {
        setStatus( StatusProperty::Ok, "Legend",
                   QString( "%1 %2 shown" ).arg( items.size() )
                       .arg( items.size() == 1 ? "entry" : "entries" ) );
    }
    context_->queueRender();
}

void LegendDisplay::queueRedraw()
{
    dirty_ = true;
}

void LegendDisplay::updatePlacement()
{
    if ( !overlay_ )
    {
        return;
    }
    overlay_->setPlacement( anchor(), QPointF( x_->getInt(), y_->getInt() ) );
    context_->queueRender();
}

void LegendDisplay::fillFontOptions( EditableEnumProperty* property )
{
    property->clearOptions();
    for ( const QString& family : QFontDatabase().families() )
    {
        property->addOption( family );
    }
}

LegendStyle LegendDisplay::toStyle() const
{
    LegendStyle style;
    style.title = title_->getString();

    style.background_color = background_color_->getColor();
    style.background_alpha = background_alpha_->getFloat();
    style.border_color = border_color_->getColor();
    style.border_width = border_width_->getInt();
    style.corner_radius = corner_radius_->getInt();

    style.max_rows = max_rows_->getInt();
    style.padding = padding_->getInt();
    style.row_spacing = row_spacing_->getInt();
    style.column_spacing = column_spacing_->getInt();

    style.font_family = font_family_->getString();
    style.font_size = font_size_->getInt();
    style.bold = bold_->getBool();
    style.text_color = text_color_->getColor();

    style.marker_size = marker_size_->getInt();
    style.label_gap = label_gap_->getInt();
    style.outline_color = outline_color_->getColor();
    style.outline_width = outline_width_->getInt();
    return style;
}

ScreenCorner LegendDisplay::anchor() const
{
    return static_cast<ScreenCorner>( anchor_->getOptionInt() );
}

bool LegendDisplay::canDrag() const
{
    return overlay_ && isEnabled() && draggable_->getBool() && !overlay_->size().isEmpty();
}

bool LegendDisplay::eventFilter( QObject* watched, QEvent* event )
{
    if ( watched != render_window_ || !overlay_ )
    {
        return Display::eventFilter( watched, event );
    }

    // Returning true consumes the event before the render panel forwards it to
    // the camera / active tool.
    switch ( event->type() )
    {
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonDblClick:
    {
        if ( dragging_ )
        {
            return true; // other buttons pressed mid-drag
        }
        const auto* mouse = static_cast<QMouseEvent*>( event );
        if ( mouse->button() != Qt::LeftButton || !canDrag() ||
             !overlay_->rect().contains( mouse->localPos() ) )
        {
            return false;
        }
        dragging_ = true;
        drag_grab_ = mouse->localPos() - overlay_->rect().topLeft();
        applyCursor();
        return true;
    }
    case QEvent::MouseMove:
    {
        const auto* mouse = static_cast<QMouseEvent*>( event );
        if ( dragging_ )
        {
            if ( mouse->buttons() & Qt::LeftButton )
            {
                dragTo( mouse->localPos() );
                return true;
            }
            endDrag(); // the release was missed (e.g. focus moved away mid-drag)
        }
        // RViz re-applies the active tool's cursor on every mouse event it
        // handles, so the hand cursor only sticks while moves over the legend
        // are kept from it.
        const bool over = canDrag() && overlay_->rect().contains( mouse->localPos() );
        setHover( over );
        return over;
    }
    case QEvent::MouseButtonRelease:
    {
        if ( !dragging_ )
        {
            return false;
        }
        const auto* mouse = static_cast<QMouseEvent*>( event );
        if ( mouse->button() == Qt::LeftButton )
        {
            endDrag();
            setHover( canDrag() && overlay_->rect().contains( mouse->localPos() ) );
        }
        return true;
    }
    case QEvent::Leave:
        setHover( false );
        return false;
    default:
        return false;
    }
}

void LegendDisplay::dragTo( const QPointF& mouse )
{
    const QSizeF viewport = overlay_->viewportSize();
    const QSizeF size = overlay_->size();
    const QPointF wanted = mouse - drag_grab_;
    const QPointF top_left(
        std::clamp( wanted.x(), 0.0, std::max( 0.0, viewport.width() - size.width() ) ),
        std::clamp( wanted.y(), 0.0, std::max( 0.0, viewport.height() - size.height() ) ) );

    // Each set fires updatePlacement(), which moves the overlay.
    const QPointF offset = AnchorOffset( anchor(), top_left, size, viewport );
    x_->setInt( static_cast<int>( std::lround( offset.x() ) ) );
    y_->setInt( static_cast<int>( std::lround( offset.y() ) ) );
}

void LegendDisplay::setHover( bool hover )
{
    if ( hover == hover_ )
    {
        return;
    }
    hover_ = hover;
    applyCursor();
}

void LegendDisplay::endDrag()
{
    dragging_ = false;
    applyCursor();
}

void LegendDisplay::applyCursor()
{
    if ( !render_window_ )
    {
        return;
    }
    if ( dragging_ || hover_ )
    {
        render_window_->setCursor( dragging_ ? Qt::ClosedHandCursor : Qt::OpenHandCursor );
        cursor_overridden_ = true;
    }
    else if ( cursor_overridden_ )
    {
        // Hand the cursor back to the render panel (i.e. the active tool).
        render_window_->unsetCursor();
        cursor_overridden_ = false;
    }
}

} // namespace avt_341::rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::LegendDisplay, rviz_common::Display )
