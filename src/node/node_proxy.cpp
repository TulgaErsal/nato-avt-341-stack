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

double NodeProxy::get_now_seconds() const {
    return get_stamp().toSec();
}

ros::Time NodeProxy::get_stamp() const {
    return ros::Time::now();
}

ros::Duration get_duration(double seconds) const{
    return ros::Duration(seconds);
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

    rclcpp::Logger NodeProxy::get_logger() const {
      return node_->get_logger();
    }

    rclcpp::Time NodeProxy::get_stamp() const {
      return node_->get_clock()->now();
    }

    rclcpp::Duration NodeProxy::get_duration(double seconds) const {
      return rclcpp::Duration::from_seconds(seconds);
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
