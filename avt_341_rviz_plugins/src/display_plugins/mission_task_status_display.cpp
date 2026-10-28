#include <avt_341_rviz_plugins/display_plugins/mission_task_status_display.h>

#include <string>
#include <utility>

#include <QColor>
#include <QSizeF>
#include <QString>
#include <QStringList>

#include <OgreCamera.h>
#include <OgreMatrix4.h>
#include <OgreQuaternion.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>

namespace avt_341::rviz_plugins
{

using rviz_common::properties::BoolProperty;
using rviz_common::properties::ColorProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::IntProperty;
using rviz_common::properties::StatusProperty;
using rviz_common::properties::VectorProperty;

namespace
{
/// A -1 task id with an empty description is the mission manager's "no task
/// running" message; a -1 id with a description is one of its internal tasks.
bool hasTask( const avt_341_msgs::msg::MissionTaskStatus& msg )
{
    return msg.task_id >= 0 || !msg.task_description.empty();
}

QString speedText( double task_speed )
{
    return task_speed < 0.0 ? QString( "Speed: -" )
                            : QString( "Speed: %1 m/s" ).arg( task_speed, 0, 'f', 2 );
}

/// The short task label, or the full description when asked for (and as a
/// fallback for publishers that predate the label field).
const std::string& taskText( const avt_341_msgs::msg::MissionTaskStatus& msg, bool verbose )
{
    return verbose || msg.task_label.empty() ? msg.task_description : msg.task_label;
}
} // namespace

MissionTaskStatusDisplay::MissionTaskStatusDisplay()
{
    verbose_description_ = new BoolProperty(
        "Verbose description", false,
        "Show the task's full description instead of its short label.",
        this, SLOT( queueRedraw() ), this );

    offset_ = new VectorProperty(
        "Offset", Ogre::Vector3( 0.0f, 0.0f, 2.0f ),
        "Offset [m] from the origin of the vehicle frame (ego_frame_id), along that "
        "frame's axes, of the point the panel is anchored to. The panel's bottom centre "
        "sits on that point in the view.",
        this );

    background_color_ = new ColorProperty(
        "Background Color", QColor( 30, 30, 30 ), "Fill color of the panel.",
        this, SLOT( queueRedraw() ), this );
    background_alpha_ = new FloatProperty(
        "Background Alpha", 0.8f, "Opacity of the panel fill (0 transparent, 1 opaque).",
        this, SLOT( queueRedraw() ), this );
    background_alpha_->setMin( 0.0f );
    background_alpha_->setMax( 1.0f );

    text_color_ = new ColorProperty(
        "Text Color", QColor( 255, 255, 255 ),
        "Color of the task text and speed text.",
        this, SLOT( queueRedraw() ), this );
    font_size_ = new IntProperty(
        "Font Size", 14, "Text size in pixels.", this, SLOT( queueRedraw() ), this );
    font_size_->setMin( 6 );
    max_width_ = new IntProperty(
        "Max Width", 320, "Widest the panel may get, in pixels; longer text wraps.",
        this, SLOT( queueRedraw() ), this );
    max_width_->setMin( 60 );

    timeout_ = new FloatProperty(
        "Timeout", 2.0f,
        "Hide the panel once no message has arrived for this many seconds (0 never hides it).",
        this );
    timeout_->setMin( 0.0f );
}

MissionTaskStatusDisplay::~MissionTaskStatusDisplay() = default;

void MissionTaskStatusDisplay::onInitialize()
{
    RosTopicDisplay::onInitialize();

    rviz_common::RenderPanel* panel = context_->getViewManager()->getRenderPanel();
    render_window_ = panel != nullptr ? panel->getRenderWindow() : nullptr;
    if ( !render_window_ )
    {
        setStatus( StatusProperty::Error, "Viewport",
                   "No main render window to draw the panel on." );
        return;
    }

    overlay_ = std::make_unique<ScreenOverlay>( scene_manager_, render_window_ );
    overlay_->setVisible( false );
}

void MissionTaskStatusDisplay::onEnable()
{
    RosTopicDisplay::onEnable();
    dirty_ = true;
}

void MissionTaskStatusDisplay::onDisable()
{
    RosTopicDisplay::onDisable();
    setPanelVisible( false );
}

void MissionTaskStatusDisplay::reset()
{
    RosTopicDisplay::reset();
    last_msg_.reset();
    dirty_ = true;
    setPanelVisible( false );
}

void MissionTaskStatusDisplay::processMessage(
    avt_341_msgs::msg::MissionTaskStatus::ConstSharedPtr msg )
{
    last_msg_ = std::move( msg );
    last_msg_time_ = std::chrono::steady_clock::now();
    dirty_ = true;
}

void MissionTaskStatusDisplay::queueRedraw()
{
    dirty_ = true;
}

void MissionTaskStatusDisplay::update( float /*wall_dt*/, float /*ros_dt*/ )
{
    if ( !overlay_ || !last_msg_ )
    {
        return;
    }

    const avt_341_msgs::msg::MissionTaskStatus& msg = *last_msg_;
    if ( !hasTask( msg ) )
    {
        setStatus( StatusProperty::Ok, "Task", "No task running." );
        setPanelVisible( false );
        return;
    }

    const float timeout = timeout_->getFloat();
    if ( timeout > 0.0f )
    {
        const double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_msg_time_ ).count();
        if ( age > timeout )
        {
            setStatus( StatusProperty::Warn, "Task",
                       QString( "No message for %1 s; panel hidden." ).arg( age, 0, 'f', 1 ) );
            setPanelVisible( false );
            return;
        }
    }

    renderPanel();
    const QSizeF size = overlay_->size();
    if ( size.isEmpty() )
    {
        setPanelVisible( false );
        return;
    }

    if ( msg.ego_frame_id.empty() )
    {
        setStatus( StatusProperty::Error, "Transform",
                   "Message has no ego_frame_id to anchor the panel to." );
        setPanelVisible( false );
        return;
    }
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if ( !context_->getFrameManager()->getTransform( msg.ego_frame_id, position, orientation ) )
    {
        setMissingTransformToFixedFrame( msg.ego_frame_id );
        setPanelVisible( false );
        return;
    }
    setTransformOk();

    QPointF anchor;
    if ( !projectToScreen( position + orientation * offset_->getVector(), anchor ) )
    {
        setPanelVisible( false );
        return;
    }

    // Bottom centre of the panel on the anchor point.
    const QPointF placement( anchor.x() - 0.5 * size.width(), anchor.y() - size.height() );
    if ( placement != placement_ )
    {
        placement_ = placement;
        overlay_->setPlacement( ScreenCorner::TopLeft, placement );
        context_->queueRender();
    }
    setPanelVisible( true );
}

void MissionTaskStatusDisplay::renderPanel()
{
    // Re-render after a new message or edit, or when the window moved to a
    // screen with another pixel ratio (the image is rendered at the screen's
    // resolution).
    const double ratio = overlay_->pixelRatio();
    if ( !dirty_ && ratio == rendered_ratio_ )
    {
        return;
    }
    dirty_ = false;
    rendered_ratio_ = ratio;

    const avt_341_msgs::msg::MissionTaskStatus& msg = *last_msg_;
    const QImage image = RenderTextPanel(
        toStyle(),
        { QString::fromStdString( taskText( msg, verbose_description_->getBool() ) ),
          speedText( msg.task_speed ) },
        ratio );
    if ( !overlay_->setImage( image ) )
    {
        setStatus( StatusProperty::Error, "Task",
                   QString( "Could not create a %1 x %2 px texture for the panel." )
                       .arg( image.width() ).arg( image.height() ) );
        return;
    }
    setStatus( StatusProperty::Ok, "Task", QString( "Task %1 shown." ).arg( msg.task_id ) );
    context_->queueRender();
}

TextPanelStyle MissionTaskStatusDisplay::toStyle() const
{
    TextPanelStyle style;
    style.background_color = background_color_->getColor();
    style.background_alpha = background_alpha_->getFloat();
    style.text_color = text_color_->getColor();
    style.font_size = font_size_->getInt();
    style.max_width = max_width_->getInt();
    return style;
}

bool MissionTaskStatusDisplay::projectToScreen( const Ogre::Vector3& point,
                                                QPointF& screen ) const
{
    Ogre::Camera* camera = render_window_
        ? rviz_rendering::RenderWindowOgreAdapter::getOgreCamera( render_window_ )
        : nullptr;
    const QSizeF viewport = overlay_->viewportSize();
    if ( camera == nullptr || viewport.isEmpty() )
    {
        return false;
    }

    // Ogre cameras look down -Z in eye space: a point at or past the eye plane
    // is behind the camera and has no meaningful projection.
    const Ogre::Vector3 eye = camera->getViewMatrix() * point;
    if ( eye.z >= 0.0f )
    {
        return false;
    }
    // Matrix4 * Vector3 performs the perspective divide: normalized device
    // coordinates, with the view spanning [-1, 1] on both axes (y up).
    const Ogre::Vector3 ndc = camera->getProjectionMatrix() * eye;
    if ( ndc.x < -1.0f || ndc.x > 1.0f || ndc.y < -1.0f || ndc.y > 1.0f )
    {
        return false;
    }

    screen = QPointF( ( ndc.x + 1.0 ) * 0.5 * viewport.width(),
                      ( 1.0 - ndc.y ) * 0.5 * viewport.height() );
    return true;
}

void MissionTaskStatusDisplay::setPanelVisible( bool visible )
{
    if ( !overlay_ || panel_visible_ == visible )
    {
        return;
    }
    panel_visible_ = visible;
    overlay_->setVisible( visible );
    context_->queueRender();
}

} // namespace avt_341::rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::MissionTaskStatusDisplay, rviz_common::Display )
