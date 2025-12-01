#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "avt_341/mission/goal_filtering/obs_avoid_goal_filter.hpp"

#define TOPIC_LEADER_POSE_IN            "avt_341/odometry"
#define TOPIC_FOLLOWER_POSE_IN          "avt_341/candidate_follower_pose"
#define TOPIC_FOLLOWER_POSE_OUT         "avt_341/filtered_follower_pose"

class ObsAvoidGoalFilterNode : public rclcpp::Node {

public:
    ObsAvoidGoalFilterNode()
    : Node("obs_avoid_goal_filter_node") {

        declare_parameter("leader_vehicle_id", "fed");
        std::string leader_vehicle_id = get_parameter("leader_vehicle_id").as_string();

        declare_parameter("vehicle_id", "mrzr2");
        std::string vehicle_id = get_parameter("vehicle_id").as_string();

        leader_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/" + leader_vehicle_id + "/" + TOPIC_LEADER_POSE_IN,
        10,
        std::bind(&ObsAvoidGoalFilterNode::LeaderOdomCallback, this, std::placeholders::_1));

        candidate_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        TOPIC_FOLLOWER_POSE_IN,
        10,
        std::bind(&ObsAvoidGoalFilterNode::CandidateFollowerPoseCallback, this, std::placeholders::_1));

        filtered_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        TOPIC_FOLLOWER_POSE_OUT,
        10);

        auto node_proxy = std::make_shared<avt_341::node::NodeProxy>(this);
        goal_filter_ = std::make_shared<avt_341::mission::ObsAvoidGoalFilter>(node_proxy, vehicle_id);  // streaming, no stored path
    }

private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            leader_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    candidate_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       filtered_pose_pub_;

    std::shared_ptr<avt_341::mission::ObsAvoidGoalFilter> goal_filter_;
    nav_msgs::msg::Odometry::SharedPtr last_leader_odom_;

    void LeaderOdomCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
        last_leader_odom_ = msg;
    }

    void CandidateFollowerPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr candidate_pose_msg) {

        if (last_leader_odom_ == nullptr) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, "No leader odometry received yet.");
            return;
        }

        geometry_msgs::msg::Pose filtered_goal = goal_filter_->Filter(candidate_pose_msg->pose, last_leader_odom_->pose.pose);

        geometry_msgs::msg::PoseStamped filtered_pose_msg;
        filtered_pose_msg.pose = filtered_goal;
        filtered_pose_msg.header = candidate_pose_msg->header;
        filtered_pose_msg.header.stamp = this->now();
        filtered_pose_pub_->publish(filtered_pose_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Note: Put --ros-args -r __ns:=/<vehicle_id> in programs args when debugging from IDE to work with rest of stack

    auto node = std::make_shared<ObsAvoidGoalFilterNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
