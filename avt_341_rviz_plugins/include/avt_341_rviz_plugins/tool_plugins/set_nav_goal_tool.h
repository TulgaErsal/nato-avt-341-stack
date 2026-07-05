#ifndef SET_NAV_GOAL_TOOL_H
#define SET_NAV_GOAL_TOOL_H

#ifndef Q_MOC_RUN
#include <QObject>

#include <avt_341_msgs/msg/nav_goal_sequence.hpp>

#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>

#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#endif

namespace rviz_common {
namespace properties {
class StringProperty;
class FloatProperty;
class QosProfileProperty;
}
}

namespace avt_341 {
namespace rviz_plugins {

/// RViz tool for setting a navigation goal by clicking to place the goal and
/// dragging to orient it -- exactly like the built-in "2D Goal Pose" tool.
///
/// The placed pose is published as an avt_341_msgs/NavGoalSequence carrying a
/// single NavGoal, on the selected vehicle's "/<vehicle_id>/avt_341/new_waypoints"
/// endpoint (the Vehicle ID and Topic suffix are editable properties).
///
/// Both this tool and the stock goal tool derive from PoseTool, so it inherits
/// the same click-to-place / drag-to-orient interaction and the same Topic and
/// QoS properties. On top of those it adds editable distance- and yaw-threshold
/// properties that populate the corresponding NavGoal fields (negative values
/// request the stack default, matching NavGoal.msg).
class SetNavGoalTool : public rviz_default_plugins::tools::PoseTool
{
    Q_OBJECT

public:
    SetNavGoalTool();
    ~SetNavGoalTool() override;

    void onInitialize() override;

protected:
    /// Build and publish a single-goal NavGoalSequence from the placed pose plus
    /// the threshold properties. \p x, \p y are in the fixed frame and \p theta
    /// is the yaw.
    void onPoseSet( double x, double y, double theta ) override;

private Q_SLOTS:
    /// (Re)create the publisher when the vehicle id, topic name or QoS settings
    /// change.
    void updateTopic();

private:
    rclcpp::Publisher<avt_341_msgs::msg::NavGoalSequence>::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr clock_;

    rviz_common::properties::StringProperty* vehicle_id_property_;
    rviz_common::properties::StringProperty* topic_property_;
    rviz_common::properties::QosProfileProperty* qos_profile_property_;
    rviz_common::properties::FloatProperty* dist_threshold_property_;
    rviz_common::properties::FloatProperty* yaw_threshold_property_;

    rclcpp::QoS qos_profile_;
};

} // end namespace rviz_plugins
} // end namespace avt_341

#endif // SET_NAV_GOAL_TOOL_H
