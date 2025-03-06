#include <ros/ros.h>
// #include <ros/package.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <yaml-cpp/yaml.h>

geometry_msgs::TransformStamped parseCalibrationFile(const std::string& filename) {
  YAML::Node config = YAML::LoadFile(filename);

  // Extract transformation matrix
  auto T = config["cam0"]["T_cam_lidar"];

  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = "lidar_frame"; // Change to the correct frame
  transform_stamped.child_frame_id = "camera_frame"; // Change to the correct frame

  // Set translation
  transform_stamped.transform.translation.x = T[0][3].as<double>();
  transform_stamped.transform.translation.y = T[1][3].as<double>();
  transform_stamped.transform.translation.z = T[2][3].as<double>();

  // Extract rotation matrix and convert to quaternion
  const tf2::Matrix3x3 rotation_matrix(
    T[0][0].as<double>(), T[0][1].as<double>(), T[0][2].as<double>(),
    T[1][0].as<double>(), T[1][1].as<double>(), T[1][2].as<double>(),
    T[2][0].as<double>(), T[2][1].as<double>(), T[2][2].as<double>()
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
  ros::init(argc, argv, "calibration_tf_publisher_node");
  ros::NodeHandle nh;

  std::string filename;
  if (!nh.getParam("calibration_file", filename)) {
    ROS_ERROR("Failed to get param 'calibration_file'");
    return 1;
  }

  geometry_msgs::TransformStamped transform_stamped = parseCalibrationFile(filename);

  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  static_broadcaster.sendTransform(transform_stamped);

  ROS_INFO("Published static transform from %s to %s",
           transform_stamped.header.frame_id.c_str(),
           transform_stamped.child_frame_id.c_str());

  ros::spin();
  return 0;
}
