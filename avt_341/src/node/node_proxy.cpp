#include "avt_341/node/node_proxy.h"

#ifdef ROS_1

#else
  #ifdef ROS_HUMBLE
  #include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
  #else
  #include "tf2_sensor_msgs/tf2_sensor_msgs.h"
  #endif
#endif

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
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

geometry_msgs::TransformStamped NodeProxy::lookup_transform(const std::string &target_frame, const std::string &source_frame){
  try {
    return tf_buffer_->lookupTransform(target_frame, source_frame, ros::Time(0));
  } catch (const tf2::TransformException & ex) {
    //n->log_warning("Could not transform %s to %s: %s", frame_world.c_str(), frame_cg.c_str(), ex.what());
    return geometry_msgs::TransformStamped();
  }
  
}

geometry_msgs::TransformStamped NodeProxy::lookup_transform(const std::string &target_frame, const std::string &source_frame, const ros::Time &stamp){
  try {
    return tf_buffer_->lookupTransform(target_frame, source_frame, stamp, ros::Duration(0.2));
  } catch (const tf2::TransformException & ex) {
    //n->log_warning("Could not transform %s to %s: %s", frame_world.c_str(), frame_cg.c_str(), ex.what());
    return geometry_msgs::TransformStamped();
  }
}

geometry_msgs::TransformStamped NodeProxy::lookup_transform(const std::string& target_frame, const ros::Time& target_time,
                                                            const std::string& source_frame, const ros::Time& source_time,
                                                            const std::string& fixed_frame){
  try {
    return tf_buffer_->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, ros::Duration(0.2));
  } catch (const tf2::TransformException & ex) {
    //n->log_warning("Could not transform %s to %s: %s", frame_world.c_str(), frame_cg.c_str(), ex.what());
    return geometry_msgs::TransformStamped();
  }
}

bool NodeProxy::transform_cloud(const sensor_msgs::PointCloud2 & in_cloud, sensor_msgs::PointCloud2 & out_cloud, const std::string &target_frame){
	try {
		tf_buffer_->transform(in_cloud, out_cloud, target_frame, ros::Duration(0.2));
	} catch (const tf2::TransformException & ex) {
		log_warning("Could not transform cloud %s to %s: %s", in_cloud.header.frame_id.c_str(), target_frame.c_str(), ex.what());
		return false;
	}
   return true;
}

void NodeProxy::publish_tf(const std::string &parent_frame, const std::string &child_frame, const geometry_msgs::PoseStamped &target_pose) {
      if(tf_buffer_ == nullptr) {
        initialize_tf_listener();
      }

      geometry_msgs::TransformStamped tf_msg;
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

void NodeProxy::publish_static_tf(const std::string &parent_frame, const std::string &child_frame, const geometry_msgs::PoseStamped &target_pose) {
  if(tf_buffer_ == nullptr) {
    initialize_tf_listener();
  }

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

double NodeProxy::get_now_seconds() const {
    return get_stamp().toSec();
}

ros::Time NodeProxy::get_stamp() const {
    return ros::Time::now();
}

void NodeProxy::spin_some() {
    ros::spinOnce();
}

void NodeProxy::spin() {
    ros::spin();
}

#else

    Rate::Rate(double hz) : rate_(hz) {
    }

    void Rate::sleep() {
      rate_.sleep();
    }

    NodeProxy::NodeProxy(const std::string &node_name) {
      node_ = rclcpp::Node::make_shared(node_name);
      this->get_parameter("/is_empty_waypoints", is_empty_waypoints_, false);
    }

    void NodeProxy::initialize_tf_listener() {
      if(tf_buffer_ != nullptr)
        return;

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
      tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    }

    void NodeProxy::publish_tf(const std::string &parent_frame, const std::string &child_frame, const geometry_msgs::msg::PoseStamped &target_pose) {
      if(tf_buffer_ == nullptr) {
        initialize_tf_listener();
      }

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

    void NodeProxy::publish_static_tf(const std::string &parent_frame, const std::string &child_frame, const geometry_msgs::msg::PoseStamped &target_pose) {
      if(tf_buffer_ == nullptr) {
        initialize_tf_listener();
      }

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


    geometry_msgs::msg::TransformStamped NodeProxy::lookup_transform(const std::string &target_frame, const std::string &source_frame){
      try {
        return tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
        return geometry_msgs::msg::TransformStamped();
      }
    }

    geometry_msgs::msg::TransformStamped NodeProxy::lookup_transform(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time & stamp){
      try {
        return tf_buffer_->lookupTransform(target_frame, source_frame, stamp, tf2::durationFromSec(0.2));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
        return geometry_msgs::msg::TransformStamped();
      }
    }

    geometry_msgs::msg::TransformStamped NodeProxy::lookup_transform(const std::string &target_frame, const rclcpp::Time &target_time,
                                                                     const std::string &source_frame, const rclcpp::Time &source_time,
                                                                     const std::string &fixed_frame){
      try {
        return tf_buffer_->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, tf2::durationFromSec(0.2));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
        return geometry_msgs::msg::TransformStamped();
      }
    }

    bool NodeProxy::transform_cloud(const sensor_msgs::msg::PointCloud2 & in_cloud, sensor_msgs::msg::PointCloud2 & out_cloud, const std::string &target_frame){
      try {
        out_cloud = tf_buffer_->transform(in_cloud, target_frame, tf2::durationFromSec(0.2));
//        tf2::doTransform(in_cloud, out_cloud, lookup_transform(target_frame, in_cloud.header.frame_id));
        return true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform cloud %s to %s: %s", in_cloud.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        out_cloud = in_cloud;
        return false;
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

    void NodeProxy::spin() {
      rclcpp::spin(node_);
    }

#endif

const std::string NodeType::LocalPlanner = "local_planner";
const std::string NodeType::GlobalPlanner = "global_planner";
const std::string NodeType::Control = "control";
const std::string NodeType::Perception = "perception";
const std::string NodeType::Mission = "mission";

} // namespace node
} // namespace avt_341
