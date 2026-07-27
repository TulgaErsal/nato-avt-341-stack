#include <avt_341_nav/node/tf_interface.h>

#ifdef GTE_ROS_HUMBLE
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#else
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#endif

namespace avt_341_nav {
namespace node {

TfInterface::TfInterface(const rclcpp::Node::SharedPtr &node) : node_(node) {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

void TfInterface::publish_tf(const std::string &parent_frame, const std::string &child_frame, const geometry_msgs::msg::PoseStamped &target_pose) {
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.frame_id = parent_frame;
  tf_msg.child_frame_id = child_frame;

  tf_msg.transform.translation.x = target_pose.pose.position.x;
  tf_msg.transform.translation.y = target_pose.pose.position.y;
  tf_msg.transform.translation.z = target_pose.pose.position.z;
  tf_msg.transform.rotation.x = target_pose.pose.orientation.x;
  tf_msg.transform.rotation.y = target_pose.pose.orientation.y;
  tf_msg.transform.rotation.z = target_pose.pose.orientation.z;
  tf_msg.transform.rotation.w = target_pose.pose.orientation.w;

  tf_broadcaster_->sendTransform(tf_msg);
}

void TfInterface::publish_static_tf(const std::string &parent_frame, const std::string &child_frame, const geometry_msgs::msg::PoseStamped &target_pose) {
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.frame_id = parent_frame;
  tf_msg.child_frame_id = child_frame;

  tf_msg.transform.translation.x = target_pose.pose.position.x;
  tf_msg.transform.translation.y = target_pose.pose.position.y;
  tf_msg.transform.translation.z = target_pose.pose.position.z;
  tf_msg.transform.rotation.x = target_pose.pose.orientation.x;
  tf_msg.transform.rotation.y = target_pose.pose.orientation.y;
  tf_msg.transform.rotation.z = target_pose.pose.orientation.z;
  tf_msg.transform.rotation.w = target_pose.pose.orientation.w;

  tf_static_broadcaster_->sendTransform(tf_msg);
}

geometry_msgs::msg::TransformStamped TfInterface::lookup_transform(const std::string &target_frame, const std::string &source_frame) {
  try {
    return tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
    return geometry_msgs::msg::TransformStamped();
  }
}

geometry_msgs::msg::TransformStamped TfInterface::lookup_transform(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &time) {
  try {
    return tf_buffer_->lookupTransform(target_frame, source_frame, time, tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
    return geometry_msgs::msg::TransformStamped();
  }
}

geometry_msgs::msg::TransformStamped TfInterface::lookup_transform(const std::string &target_frame, const rclcpp::Time &target_time,
                                                                   const std::string &source_frame, const rclcpp::Time &source_time,
                                                                   const std::string &fixed_frame) {
  try {
    return tf_buffer_->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
    return geometry_msgs::msg::TransformStamped();
  }
}

geometry_msgs::msg::PoseStamped TfInterface::lookup_pose(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &stamp) {
  auto tx = lookup_transform(target_frame, source_frame, stamp);
  geometry_msgs::msg::PoseStamped pose_msg;

  pose_msg.pose.position.x = tx.transform.translation.x;
  pose_msg.pose.position.y = tx.transform.translation.y;
  pose_msg.pose.position.z = tx.transform.translation.z;

  pose_msg.pose.orientation = tx.transform.rotation;
  pose_msg.header = tx.header;

  return pose_msg;
}

bool TfInterface::transform_cloud(const sensor_msgs::msg::PointCloud2 &in_cloud, sensor_msgs::msg::PointCloud2 &out_cloud, const std::string &target_frame) {
  try {
    out_cloud = tf_buffer_->transform(in_cloud, target_frame, tf2::durationFromSec(0.2));
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform cloud %s to %s: %s", in_cloud.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    out_cloud = in_cloud;
    return false;
  }
}

bool TfInterface::transform_pose(const geometry_msgs::msg::PoseStamped &in_pose, geometry_msgs::msg::PoseStamped &out_pose, const std::string &target_frame, float duration) {
  try {
    out_pose = tf_buffer_->transform(in_pose, target_frame, tf2::durationFromSec(duration));
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform pose %s to %s: %s", in_pose.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    out_pose = in_pose;
    return false;
  }
}

} // namespace node
} // namespace avt_341_nav
