#include "avt_341/node/node_proxy.h"
#include "avt_341/node/ros_types.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

std::shared_ptr<avt_341::node::NodeProxy> n = nullptr;

avt_341::msg::TransformStamped parseCalibrationFile(const std::string& filename) {
  YAML::Node config = YAML::LoadFile(filename);

  // Extract transformation matrix
  auto T = config["cam0"]["T_cam_lidar"];

  avt_341::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = n->get_stamp();
  transform_stamped.header.frame_id = "os_lidar"; // Change to the correct frame
  transform_stamped.child_frame_id = "flir_camera"; // Change to the correct frame

  // Set translation
  transform_stamped.transform.translation.x = T[0][3].as<double>();
  transform_stamped.transform.translation.y = T[1][3].as<double>();
  transform_stamped.transform.translation.z = T[2][3].as<double>();

  // Extract rotation matrix and convert to quaternion
  const avt_341::msg_tf::Matrix3x3 rotation_matrix(
    T[0][0].as<double>(), T[0][1].as<double>(), T[0][2].as<double>(),
    T[1][0].as<double>(), T[1][1].as<double>(), T[1][2].as<double>(),
    T[2][0].as<double>(), T[2][1].as<double>(), T[2][2].as<double>()
  );

  avt_341::msg_tf::Quaternion q;
  rotation_matrix.getRotation(q);
  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();

  return transform_stamped;
}

int main(int argc, char** argv) {
  n = avt_341::node::init_node(argc, argv, "calibration_tf_publisher_node");

  std::string filename;
  if (!n->get_parameter("calibration_file", filename, std::string{"nofile"})) {
    n->log_error("Failed to get parameter 'calibration_file'");
    return 1;
  }

  const avt_341::msg::TransformStamped transform_stamped = parseCalibrationFile(filename);

  // tf2_ros::StaticTransformBroadcaster static_broadcaster;
  auto static_broadcaster = n->create_static_transform_broadcaster();
  static_broadcaster->sendTransform(transform_stamped);

  n->log_info("Published static transform from %s to %s",
    transform_stamped.header.frame_id.c_str(),
    transform_stamped.child_frame_id.c_str());

  n->spin();
  return 0;
}
