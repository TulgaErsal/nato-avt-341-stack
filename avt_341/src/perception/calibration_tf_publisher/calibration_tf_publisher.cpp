#include "avt_341/node/node_proxy.h"
#include "avt_341/node/ros_types.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

std::shared_ptr<avt_341::node::NodeProxy> n = nullptr;

avt_341::msg::TransformStamped parseCalibrationFile(const std::string& filename, const std::string& lidar_frame, const std::string& camera_frame) {
  YAML::Node config = YAML::LoadFile(filename);
  if (!config) {
    std::cerr << "Failed to load YAML file: " << filename << std::endl;
    throw std::runtime_error("Failed to load YAML file");
  }

  auto T_data = config["transformation_lidar_cam_matrix"]["data"];

  avt_341::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = n->get_stamp();
  transform_stamped.header.frame_id = lidar_frame;
  transform_stamped.child_frame_id = camera_frame;

  // Set translation
  auto transform_data = T_data.as<std::vector<float>>();

  transform_stamped.transform.translation.x = transform_data[3];
  transform_stamped.transform.translation.y = transform_data[7];
  transform_stamped.transform.translation.z = transform_data[11];

  // Extract rotation matrix and convert to quaternion
  const tf2::Matrix3x3 rotation_matrix(
      transform_data[0], transform_data[1], transform_data[2],
      transform_data[4], transform_data[5], transform_data[6],
      transform_data[8], transform_data[9], transform_data[10]
  );

  tf2::Quaternion q;
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

  std::string lidar_frame = n->get_parameter("lidar_frame", std::string{"os_lidar"});
  std::string camera_frame = n->get_parameter("camera_frame", std::string{"flir_optical"});

  const avt_341::msg::TransformStamped transform_stamped = parseCalibrationFile(filename, lidar_frame, camera_frame);

  // tf2_ros::StaticTransformBroadcaster static_broadcaster;
  auto static_broadcaster = n->create_static_transform_broadcaster();
  static_broadcaster->sendTransform(transform_stamped);

  n->log_info("Published static transform from %s to %s",
    transform_stamped.header.frame_id.c_str(),
    transform_stamped.child_frame_id.c_str());

  n->spin();
  return 0;
}
