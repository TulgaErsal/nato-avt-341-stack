#include "avt_341/node/node_proxy.h"

namespace avt_341 {
namespace node {

#ifdef ROS_1

Rate::Rate(double hz) : rate_(hz) {
}

void Rate::sleep() {
    rate_.sleep();
}

NodeProxy::NodeProxy(const std::string &node_name) {
}

void NodeProxy::initialize_tf_listener() {
  if(tf_buffer_ != nullptr)
    return;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

geometry_msgs::TransformStamped NodeProxy::lookup_transform(const std::string &target_frame, const std::string &source_frame){
  return tf_buffer_->lookupTransform(target_frame, source_frame, ros::Time(0));
}

double NodeProxy::get_now_seconds() const {
    return get_stamp().toSec();
}

ros::Time NodeProxy::get_stamp() const {
    return ros::Time::now();
}

void NodeProxy::spin_some() {
    ros::spinOnce();
}

#else

    Rate::Rate(double hz) : rate_(hz) {
    }

    void Rate::sleep() {
      rate_.sleep();
    }

    NodeProxy::NodeProxy(const std::string &node_name) {
      node_ = rclcpp::Node::make_shared("avt_341_control_node");
      this->get_parameter("/is_empty_waypoints", is_empty_waypoints_, false);
    }

    void NodeProxy::initialize_tf_listener() {
      if(tf_buffer_ != nullptr)
        return;

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    geometry_msgs::msg::TransformStamped NodeProxy::lookup_transform(const std::string &target_frame, const std::string &source_frame){
      try {
        return tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
        return geometry_msgs::msg::TransformStamped();
      }
    }

    rclcpp::Time NodeProxy::get_stamp() const {
      return node_->get_clock()->now();
    }

    double NodeProxy::get_now_seconds() const {
      return get_stamp().seconds();
    }

    void NodeProxy::spin_some() {
      rclcpp::spin_some(node_);
    }

#endif


} // namespace node
} // namespace avt_341
