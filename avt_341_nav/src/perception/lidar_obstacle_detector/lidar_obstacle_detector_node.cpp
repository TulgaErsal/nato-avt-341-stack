/* obstacle_detector_node.cpp

 * Copyright (C) 2021 SS47816

 * ROS Node for 3D LiDAR Obstacle Detection & Tracking Algorithms

**/
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/duration.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "avt_341_nav/perception/lidar_obstacle_detector/lidar_obstacle_detector.hpp"
#include <avt_341_nav/obstacle_detector_params_service.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "avt_341_nav/node/tf_interface.h"


size_t obstacle_id_;
std::string bbox_source_frame_;
std::vector<Box> prev_boxes_, curr_boxes_;
std::shared_ptr<avt_341_nav::perception::LidarObstacleDetector<pcl::PointXYZ>> obstacle_detector;

avt_341_nav::params::obstacle_detector::Params node_params;

// Eigen adapters for fixed-size pointcloud filtering parameters.
Eigen::Vector4f roi_max_point, roi_min_point, body_max_point, body_min_point;
Eigen::Vector3f ground_normal;

// Node handle
rclcpp::Node::SharedPtr nh = nullptr;
std::shared_ptr<avt_341_nav::node::TfInterface> tf;

// Publishers
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_cloud_ground;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_cloud_clusters;
std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> pub_bboxes;

void publishJointCloud(
    const std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> &segmented_clouds,
    const std_msgs::msg::Header& header
    ) {

  pcl::PointCloud<pcl::PointXYZI> joint_cloud;
  joint_cloud.resize(segmented_clouds.first->points.size() + segmented_clouds.second->points.size());

  int c = 0;

  // ground cloud
  for (const auto & pt : segmented_clouds.second->points) {
    pcl::PointXYZI pt_i;
    pt_i.x = pt.x;
    pt_i.y = pt.y;
    pt_i.z = pt.z;
    pt_i.intensity = 0.0;
    joint_cloud.points[c++] = pt_i;
  }

  // non-ground cloud
  for (const auto & pt : segmented_clouds.first->points) {
    pcl::PointXYZI pt_i;
    pt_i.x = pt.x;
    pt_i.y = pt.y;
    pt_i.z = pt.z;
    pt_i.intensity = 1.0;
    joint_cloud.points[c++] = pt_i;
  }

  sensor_msgs::msg::PointCloud2 joint_cloud_msg;
  pcl::toROSMsg(joint_cloud, joint_cloud_msg);
  joint_cloud_msg.fields[3].name = node_params.publish_seg_as_one_field;
  joint_cloud_msg.header = header;
  pub_cloud_clusters->publish(joint_cloud_msg);

}

void publishClouds(std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented_clouds, std_msgs::msg::Header header)
{
	if(segmented_clouds.second->size() > 0){
		sensor_msgs::msg::PointCloud2 ground_cloud;
		pcl::toROSMsg(*(segmented_clouds.second), ground_cloud);
		ground_cloud.header = header;
		pub_cloud_ground->publish(ground_cloud);
	}

	if(segmented_clouds.first->size() > 0) {
		sensor_msgs::msg::PointCloud2 obstacle_cloud;
		pcl::toROSMsg(*(segmented_clouds.first), obstacle_cloud);
		obstacle_cloud.header = header;
		pub_cloud_clusters->publish(obstacle_cloud);
	}
}

void publishDetectedObjects(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, std_msgs::msg::Header header)
{
  for (auto& cluster : cloud_clusters)
  {
    // Create Bounding Boxes
    Box box = node_params.use_pca_box?
      obstacle_detector->pcaBoundingBox(cluster, obstacle_id_) :
      obstacle_detector->axisAlignedBoundingBox(cluster, obstacle_id_);

    obstacle_id_ = (obstacle_id_ < SIZE_MAX)? ++obstacle_id_ : 0;
    curr_boxes_.emplace_back(box);
  }

  // Re-assign Box ids based on tracking result
  if (node_params.use_tracking)
    obstacle_detector->obstacleTracking(
        prev_boxes_, curr_boxes_,
        static_cast<float>(node_params.displacement_threshold),
        static_cast<float>(node_params.iou_threshold));

  // Publish bounding boxes as a MarkerArray (CUBE_LIST wireframes)
  visualization_msgs::msg::MarkerArray marker_array;
  // Delete all previous markers first to avoid stale boxes persisting
  // when the cluster count decreases between frames.
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_marker.header.stamp = header.stamp;
  delete_marker.header.frame_id = bbox_source_frame_;
  marker_array.markers.push_back(delete_marker);
  for (const auto& box : curr_boxes_)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = header.stamp;
    marker.header.frame_id = bbox_source_frame_;
    marker.ns = "lidar_bboxes";
    marker.id = box.id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = box.position.x();
    marker.pose.position.y = box.position.y();
    marker.pose.position.z = box.position.z();
    marker.pose.orientation.w = box.quaternion.w();
    marker.pose.orientation.x = box.quaternion.x();
    marker.pose.orientation.y = box.quaternion.y();
    marker.pose.orientation.z = box.quaternion.z();
    marker.scale.x = box.dimension.x();
    marker.scale.y = box.dimension.y();
    marker.scale.z = box.dimension.z();
    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.0f;
    marker.color.a = 0.3f;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_array.markers.push_back(marker);
  }
  pub_bboxes->publish(marker_array);

  // Update previous bounding boxes
  prev_boxes_.swap(curr_boxes_);
  curr_boxes_.clear();
}

void publishDeleteAll(const std_msgs::msg::Header& header)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_marker.header.stamp = header.stamp;
  delete_marker.header.frame_id = bbox_source_frame_;
  marker_array.markers.push_back(delete_marker);
  pub_bboxes->publish(marker_array);
}

void lidarPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_points)
{
  // Transform point cloud
	sensor_msgs::msg::PointCloud2 lidar_points_transformed;
  if(lidar_points->header.frame_id != node_params.robot_base_link)
  {
    if (!tf->transform_cloud(
            *lidar_points, lidar_points_transformed,
            node_params.robot_base_link)) {
      RCLCPP_WARN(nh->get_logger(), "Unable to transform pointcloud from %s -> %s", lidar_points->header.frame_id.c_str(), node_params.robot_base_link.c_str());
      publishDeleteAll(lidar_points->header);
      return;
    }
  }
  else
  {
    lidar_points_transformed = *lidar_points;
  }

  const auto pointcloud_header = lidar_points_transformed.header;
  bbox_source_frame_ = lidar_points_transformed.header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(lidar_points_transformed, *raw_cloud);

  // Downsampling, ROI, and removing the car roof
  auto filtered_cloud = obstacle_detector->filterCloud(
      raw_cloud, static_cast<float>(node_params.voxel_grid_size),
      roi_min_point, roi_max_point, body_min_point, body_max_point);

  if(filtered_cloud->size() < 10) {
    publishDeleteAll(pointcloud_header);
    return;
  }

  // Transform pointcloud to fixed frame (rotation only)
  pcl::PointCloud<pcl::PointXYZ>::Ptr fixed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  geometry_msgs::msg::TransformStamped fixed_tf = tf->lookup_transform(
      node_params.fixed_frame, node_params.robot_base_link);
  Eigen::Quaternionf q(fixed_tf.transform.rotation.w, fixed_tf.transform.rotation.x, fixed_tf.transform.rotation.y, fixed_tf.transform.rotation.z);
  // Guard against a degenerate quaternion (all zeros) returned when TF is
  // not yet available. A zero quaternion produces a NaN rotation matrix and
  // an empty fixed_cloud after the transform, which crashes the KdTree
  // inside pclFilterNorms.
  if (q.norm() < 1e-6f) {
    RCLCPP_WARN(nh->get_logger(), "Fixed-frame TF not yet available, skipping this scan.");
    publishDeleteAll(pointcloud_header);
    return;
  }
  Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity();
  mat4.block<3,3>(0,0) = q.normalized().toRotationMatrix();
  Eigen::Affine3f transform_fixed;
  transform_fixed.matrix() = mat4;
  pcl::transformPointCloud(*filtered_cloud, *fixed_cloud, transform_fixed);

  if(fixed_cloud->size() < 10) {
    publishDeleteAll(pointcloud_header);
    return;
  }

  // Segment ground and obstacle points using normal filtering
  pcl::PointCloud<pcl::PointXYZ>::Ptr norm_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  obstacle_detector->pclFilterNorms(
      filtered_cloud, fixed_cloud, norm_filtered, ground_filtered,
      ground_normal,
      static_cast<float>(node_params.ground_normal_threshold),
      static_cast<float>(node_params.obstacle_scale),
      static_cast<int>(node_params.obstacle_min_neighbors));

  // Segment the groud plane and obstacles
  //auto segmented_clouds = obstacle_detector->segmentPlane(filtered_cloud, 30, ground_threshold);
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented_clouds;
  segmented_clouds.first = norm_filtered;
  segmented_clouds.second = ground_filtered;

  // Publish ground cloud and obstacle cloud
  if (node_params.publish_seg_as_one) {
    publishJointCloud(segmented_clouds, pointcloud_header);
  }else {
    publishClouds(segmented_clouds, pointcloud_header);
  }


  if(segmented_clouds.first->size() <= 0) {
    publishDeleteAll(pointcloud_header);
    return;
  }

  // Cluster objects
  auto cloud_clusters = obstacle_detector->clustering(
      segmented_clouds.first,
      static_cast<float>(node_params.cluster_threshold),
      static_cast<int>(node_params.cluster_min_size),
      static_cast<int>(node_params.cluster_max_size));

  // Publish Obstacles
  publishDetectedObjects(std::move(cloud_clusters), pointcloud_header);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("obstacle_detector_node");
  tf = std::make_shared<avt_341_nav::node::TfInterface>(nh);

  avt_341_nav::params::obstacle_detector::ParamsListener param_listener(nh);
  node_params = param_listener.get_params();

  roi_max_point = Eigen::Vector4f(
      node_params.roi_max_x, node_params.roi_max_y,
      node_params.roi_max_z, 1.0f);
  roi_min_point = Eigen::Vector4f(
      node_params.roi_min_x, node_params.roi_min_y,
      node_params.roi_min_z, 1.0f);
  body_max_point = Eigen::Vector4f(
      node_params.body_max_x, node_params.body_max_y,
      node_params.body_max_z, 1.0f);
  body_min_point = Eigen::Vector4f(
      node_params.body_min_x, node_params.body_min_y,
      node_params.body_min_z, 1.0f);
  ground_normal = Eigen::Vector3f(
      node_params.ground_normal_x, node_params.ground_normal_y,
      node_params.ground_normal_z);

  auto sub_lidar_points =
      nh->create_subscription<sensor_msgs::msg::PointCloud2>(
          node_params.lidar_points_topic, 1, lidarPointsCallback);
  pub_cloud_ground = nh->create_publisher<sensor_msgs::msg::PointCloud2>(
      node_params.cloud_ground_topic, 1);
  pub_cloud_clusters = nh->create_publisher<sensor_msgs::msg::PointCloud2>(
      node_params.cloud_clusters_topic, 1);
  pub_bboxes =
      nh->create_publisher<visualization_msgs::msg::MarkerArray>(
          node_params.bboxes_topic, 1);

  // Create point processor
  obstacle_detector = std::make_shared<avt_341_nav::perception::LidarObstacleDetector<pcl::PointXYZ>>();
  obstacle_id_ = 0;

  rclcpp::spin(nh);
  return 0;
}
