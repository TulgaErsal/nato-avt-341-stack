#ifndef AVT_341_TF_INTERFACE_H
#define AVT_341_TF_INTERFACE_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#ifdef GTE_ROS_HUMBLE
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace avt_341 {
namespace node {

class TfInterface {
public:
  explicit TfInterface(const rclcpp::Node::SharedPtr &node);

  geometry_msgs::msg::TransformStamped lookup_transform(const std::string &target_frame, const std::string &source_frame);
  geometry_msgs::msg::TransformStamped lookup_transform(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &time);
  geometry_msgs::msg::TransformStamped lookup_transform(const std::string &target_frame, const rclcpp::Time &target_time,
                                                        const std::string &source_frame, const rclcpp::Time &source_time,
                                                        const std::string &fixed_frame);

  geometry_msgs::msg::PoseStamped lookup_pose(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &stamp);

  void publish_tf(const std::string &parent_frame, const std::string &child_frame, const geometry_msgs::msg::PoseStamped &target_pose);
  void publish_static_tf(const std::string &parent_frame, const std::string &child_frame, const geometry_msgs::msg::PoseStamped &target_pose);

  bool transform_cloud(const sensor_msgs::msg::PointCloud2 &in_cloud, sensor_msgs::msg::PointCloud2 &out_cloud, const std::string &target_frame);

  bool transform_pose(const geometry_msgs::msg::PoseStamped &in_pose, geometry_msgs::msg::PoseStamped &out_pose, const std::string &target_frame, float duration = 0.2);

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

} // namespace node
} // namespace avt_341

#endif // AVT_341_TF_INTERFACE_H
