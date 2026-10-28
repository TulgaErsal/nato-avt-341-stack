#ifndef MISSION_TASK_STATUS_DISPLAY_H
#define MISSION_TASK_STATUS_DISPLAY_H

#ifndef Q_MOC_RUN
#include <chrono>
#include <memory>

#include <QPointer>
#include <QPointF>

#include <OgreVector3.h>

#include <avt_341_msgs/msg/mission_task_status.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_rendering/render_window.hpp>

#include <avt_341_rviz_plugins/primitives/screen_overlay.h>
#include <avt_341_rviz_plugins/primitives/text_panel_renderer.h>
#endif

namespace rviz_common {
namespace properties {
class BoolProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class VectorProperty;
} // namespace properties
} // namespace rviz_common

namespace avt_341 {
namespace rviz_plugins {

/// RViz display for avt_341_msgs/MissionTaskStatus: the task label (or, with
/// "Verbose description" on, its full description) and speed setpoint on a
/// colored panel that follows the vehicle frame the message names in
/// ego_frame_id (its base_link). The panel is a 2D screen-space
/// overlay (see ScreenOverlay) whose bottom centre sits on the projected position
/// of a configurable Offset from that frame; it keeps its pixel size whatever the
/// camera does. Background and text colors are configurable.
///
/// The panel is hidden while the anchor point is behind the camera or outside
/// the view, while the ego frame is missing or has no transform to the fixed
/// frame, when the
/// message reports no running task, and (with a non-zero Timeout) once no
/// message has arrived for Timeout seconds.
///
/// The image is re-rendered at most once per frame, in update(), after a new
/// message or style change, or when the window moves to a screen with a
/// different pixel ratio. The position is refreshed every frame from tf.
class MissionTaskStatusDisplay
    : public rviz_common::RosTopicDisplay<avt_341_msgs::msg::MissionTaskStatus>
{
    Q_OBJECT

public:
    MissionTaskStatusDisplay();
    ~MissionTaskStatusDisplay() override;

    void onInitialize() override;
    void update( float wall_dt, float ros_dt ) override;
    void reset() override;

protected:
    void onEnable() override;
    void onDisable() override;
    void processMessage( avt_341_msgs::msg::MissionTaskStatus::ConstSharedPtr msg ) override;

private Q_SLOTS:
    /// Style changed: re-render the panel on the next update().
    void queueRedraw();

private:
    TextPanelStyle toStyle() const;
    /// Re-render the panel image from the last message when it is out of date.
    void renderPanel();
    /// Screen position (render-window logical px) of a point given in the fixed
    /// frame; false when it is behind the camera or outside the view.
    bool projectToScreen( const Ogre::Vector3& point, QPointF& screen ) const;
    void setPanelVisible( bool visible );

    std::unique_ptr<ScreenOverlay> overlay_;
    QPointer<rviz_rendering::RenderWindow> render_window_;

    avt_341_msgs::msg::MissionTaskStatus::ConstSharedPtr last_msg_;
    std::chrono::steady_clock::time_point last_msg_time_;
    bool dirty_ = true;
    double rendered_ratio_ = 0.0; ///< Pixel ratio of the current image.
    bool panel_visible_ = false;
    QPointF placement_;           ///< Top-left of the panel at the last update.

    rviz_common::properties::BoolProperty*  verbose_description_ = nullptr;
    rviz_common::properties::VectorProperty* offset_ = nullptr;
    rviz_common::properties::ColorProperty* background_color_ = nullptr;
    rviz_common::properties::FloatProperty* background_alpha_ = nullptr;
    rviz_common::properties::ColorProperty* text_color_ = nullptr;
    rviz_common::properties::IntProperty*   font_size_ = nullptr;
    rviz_common::properties::IntProperty*   max_width_ = nullptr;
    rviz_common::properties::FloatProperty* timeout_ = nullptr;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // MISSION_TASK_STATUS_DISPLAY_H
