#include <avt_341_rviz_plugins/tool_plugins/set_nav_goal_tool.h>

#include <string>

#include <avt_341_msgs/msg/nav_goal.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <avt_341_rviz_plugins/components/topic_config.h>

namespace avt_341::rviz_plugins
{

using rviz_common::properties::FloatProperty;
using rviz_common::properties::QosProfileProperty;
using rviz_common::properties::StringProperty;

SetNavGoalTool::SetNavGoalTool()
    : rviz_default_plugins::tools::PoseTool(), qos_profile_( 5 )
{
    // Activation shortcut ('g' is already claimed by the 2D Goal Pose tool).
    shortcut_key_ = 'n';

    // The goal is published on "/<Vehicle ID>/<Topic>"; both parts are editable
    // so a single tool can target any vehicle's waypoint endpoint.
    vehicle_id_property_ = new StringProperty(
        "Vehicle ID", "mrzr2",
        "Vehicle namespace the goal is published under (the leading segment of "
        "the topic path).",
        getPropertyContainer(), SLOT( updateTopic() ), this );

    topic_property_ = new StringProperty(
        "Topic", "avt_341/new_waypoints",
        "Topic suffix, under the vehicle namespace, on which to publish the "
        "NavGoalSequence.",
        getPropertyContainer(), SLOT( updateTopic() ), this );

    qos_profile_property_ = new QosProfileProperty( topic_property_, qos_profile_ );

    // Negative thresholds tell the stack to fall back to its own defaults; see
    // the field documentation in avt_341_msgs/msg/NavGoal.msg.
    dist_threshold_property_ = new FloatProperty(
        "Distance Threshold", -1.0f,
        "Distance (m) within which the goal counts as reached. "
        "A negative value requests the stack default.",
        getPropertyContainer() );

    yaw_threshold_property_ = new FloatProperty(
        "Yaw Threshold", -1.0f,
        "Yaw difference (rad) within which the goal counts as reached. "
        "A negative value requests the stack default.",
        getPropertyContainer() );
}

SetNavGoalTool::~SetNavGoalTool() = default;

void SetNavGoalTool::onInitialize()
{
    PoseTool::onInitialize();
    qos_profile_property_->initialize(
        [this]( rclcpp::QoS profile ) { qos_profile_ = profile; } );
    setName( "Set Nav Goal" );
    updateTopic();
}

void SetNavGoalTool::updateTopic()
{
    rclcpp::Node::SharedPtr raw_node =
        context_->getRosNodeAbstraction().lock()->get_raw_node();
    const std::string topic = makeTopicPath( vehicle_id_property_->getString(),
                                             topic_property_->getString() );
    publisher_ = raw_node->create_publisher<avt_341_msgs::msg::NavGoalSequence>(
        topic, qos_profile_ );
    clock_ = raw_node->get_clock();
}

void SetNavGoalTool::onPoseSet( double x, double y, double theta )
{
    std::string fixed_frame = context_->getFixedFrame().toStdString();

    avt_341_msgs::msg::NavGoal goal;
    goal.header.stamp = clock_->now();
    goal.header.frame_id = fixed_frame;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation = orientationAroundZAxis( theta );
    goal.dist_threshold = dist_threshold_property_->getFloat();
    goal.yaw_threshold = yaw_threshold_property_->getFloat();

    logPose( "nav goal", goal.pose.position, goal.pose.orientation, theta,
             fixed_frame );

    // Send the single placed goal as a one-element NavGoalSequence, which is what
    // the "new_waypoints" endpoint expects.
    avt_341_msgs::msg::NavGoalSequence sequence;
    sequence.header = goal.header;
    sequence.goals.push_back( goal );

    publisher_->publish( sequence );
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::SetNavGoalTool, rviz_common::Tool )
