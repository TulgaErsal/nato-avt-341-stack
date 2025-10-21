/* obstacle_detector_node.cpp

 * Copyright (C) 2021 SS47816

 * ROS Node for 3D LiDAR Obstacle Detection & Tracking Algorithms

**/
#include <string>
#include <memory>

#include "avt_341/node/node_proxy.h"
#include "avt_341/node/ros_types.h"
#include "avt_341/perception/lidar_obstacle_detector/ros2/lidar_obstacle_detector.hpp"

#include <pcl_conversions/pcl_conversions.h>


size_t obstacle_id_;
std::string bbox_target_frame_;
std::string bbox_source_frame_;
std::string robot_base_link_;
std::string fixed_frame_;
std::vector<Box> prev_boxes_, curr_boxes_;
std::shared_ptr<avt_341::perception::LidarObstacleDetector<pcl::PointXYZ>> obstacle_detector;

// Pointcloud Filtering Parameters
bool use_pca_box;
bool use_tracking;
float voxel_grid_size;
Eigen::Vector4f roi_max_point, roi_min_point, body_max_point, body_min_point;
float ground_threshold;
float cluster_threshold;
int cluster_max_size, cluster_min_size;
float displacement_threshold, iou_threshold;
Eigen::Vector3f ground_normal;
float ground_normal_threshold;
float obstacle_scale;
int obstacle_min_neighbors;
bool publish_seg_as_one;
std::string publish_seg_as_one_field;

// Node handle
std::shared_ptr<avt_341::node::NodeProxy> nh = nullptr;

// Publishers
std::shared_ptr<avt_341::node::Publisher<avt_341::msg::PointCloud2>> pub_cloud_ground;
std::shared_ptr<avt_341::node::Publisher<avt_341::msg::PointCloud2>> pub_cloud_clusters;

void publishJointCloud(
    const std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> &segmented_clouds,
    const avt_341::msg::Header& header
    ) {

  pcl::PointCloud<pcl::PointXYZI> joint_cloud;
  joint_cloud.resize(segmented_clouds.first->points.size() + segmented_clouds.second->points.size());

  int c = 0;

  // ground cloud
  for (const auto & pt : segmented_clouds.second->points) {
    joint_cloud.points[c++] = pcl::PointXYZI(pt.x, pt.y, pt.z, 0.0);
  }

  // non-ground cloud
  for (const auto & pt : segmented_clouds.first->points) {
    joint_cloud.points[c++] = pcl::PointXYZI(pt.x, pt.y, pt.z, 1.0);
  }

  avt_341::msg::PointCloud2 joint_cloud_msg;
  pcl::toROSMsg(joint_cloud, joint_cloud_msg);
  joint_cloud_msg.fields[3].name = publish_seg_as_one_field;
  joint_cloud_msg.header = header;
  pub_cloud_clusters->publish(joint_cloud_msg);

}

void publishClouds(std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented_clouds, avt_341::msg::Header header)
{
	if(segmented_clouds.second->size() > 0){
		avt_341::msg::PointCloud2 ground_cloud;
		pcl::toROSMsg(*(segmented_clouds.second), ground_cloud);
		ground_cloud.header = header;
		pub_cloud_ground->publish(ground_cloud);
	}

	if(segmented_clouds.first->size() > 0) {
		avt_341::msg::PointCloud2 obstacle_cloud;
		pcl::toROSMsg(*(segmented_clouds.first), obstacle_cloud);
		obstacle_cloud.header = header;
		pub_cloud_clusters->publish(obstacle_cloud);
	}
}

void publishDetectedObjects(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, avt_341::msg::Header header)
{
  for (auto& cluster : cloud_clusters)
  {
    // Create Bounding Boxes
    Box box = use_pca_box? 
      obstacle_detector->pcaBoundingBox(cluster, obstacle_id_) : 
      obstacle_detector->axisAlignedBoundingBox(cluster, obstacle_id_);
    
    obstacle_id_ = (obstacle_id_ < SIZE_MAX)? ++obstacle_id_ : 0;
    curr_boxes_.emplace_back(box);
  }

  // Re-assign Box ids based on tracking result
  if (use_tracking)
    obstacle_detector->obstacleTracking(prev_boxes_, curr_boxes_, displacement_threshold, iou_threshold);

  // Update previous bounding boxes
  prev_boxes_.swap(curr_boxes_);
  curr_boxes_.clear();
}

void lidarPointsCallback(avt_341::msg::PointCloud2Ptr lidar_points)
{
  // Transform point cloud
	avt_341::msg::PointCloud2 lidar_points_transformed;
  if(lidar_points->header.frame_id != robot_base_link_)
  {
    if (!nh->transform_cloud(*lidar_points, lidar_points_transformed, robot_base_link_)) {
      nh->log_warning("Unable to transform pointcloud from %s -> %s",lidar_points->header.frame_id.c_str(),robot_base_link_.c_str());
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
  auto filtered_cloud = obstacle_detector->filterCloud(raw_cloud, voxel_grid_size, roi_min_point, roi_max_point, body_min_point, body_max_point);

  if(filtered_cloud->size() < 10) return;

  // Transform pointcloud to fixed frame (rotation only)
  pcl::PointCloud<pcl::PointXYZ>::Ptr fixed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  avt_341::msg::TransformStamped fixed_tf = nh->lookup_transform(fixed_frame_, robot_base_link_);
  Eigen::Quaternionf q(fixed_tf.transform.rotation.w, fixed_tf.transform.rotation.x, fixed_tf.transform.rotation.y, fixed_tf.transform.rotation.z);
  Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity();
  mat4.block<3,3>(0,0) = q.normalized().toRotationMatrix();
  Eigen::Affine3f transform_fixed;
  transform_fixed.matrix() = mat4;
  pcl::transformPointCloud(*filtered_cloud, *fixed_cloud, transform_fixed);

  // Segment ground and obstacle points using normal filtering
  pcl::PointCloud<pcl::PointXYZ>::Ptr norm_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  obstacle_detector->pclFilterNorms(filtered_cloud, fixed_cloud, norm_filtered, ground_filtered, ground_normal, ground_normal_threshold, obstacle_scale, obstacle_min_neighbors);

  // Segment the groud plane and obstacles
  //auto segmented_clouds = obstacle_detector->segmentPlane(filtered_cloud, 30, ground_threshold);
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented_clouds;
  segmented_clouds.first = norm_filtered;
  segmented_clouds.second = ground_filtered;

  // Publish ground cloud and obstacle cloud
  if (publish_seg_as_one) {
    publishJointCloud(segmented_clouds, pointcloud_header);
  }else {
    publishClouds(segmented_clouds, pointcloud_header);
  }


  if(segmented_clouds.first->size()<= 0) return;

  // Cluster objects
  auto cloud_clusters = obstacle_detector->clustering(segmented_clouds.first, cluster_threshold, cluster_min_size, cluster_max_size);

  // Publish Obstacles
  publishDetectedObjects(std::move(cloud_clusters), pointcloud_header);
}


int main(int argc, char** argv)
{
  nh = avt_341::node::init_node(argc, argv, "obstacle_detector_node");
  nh->initialize_tf_listener();

  std::string lidar_points_topic;
  std::string cloud_ground_topic;
  std::string cloud_clusters_topic;

  float roi_max_x, roi_max_y, roi_max_z, roi_min_x, roi_min_y, roi_min_z;
  float body_max_x, body_max_y, body_max_z, body_min_x, body_min_y, body_min_z;
  float ground_normal_x, ground_normal_y, ground_normal_z;
  nh->get_parameter("~lidar_points_topic", lidar_points_topic, std::string("/ouster/points"));
  nh->get_parameter("~cloud_ground_topic", cloud_ground_topic, std::string("/avt_341/lidar_detector/cloud_ground"));
  nh->get_parameter("~cloud_clusters_topic", cloud_clusters_topic, std::string("/avt_341/lidar_detector/cloud_clusters"));
  nh->get_parameter("~bbox_target_frame", bbox_target_frame_, std::string("base_link"));
  nh->get_parameter("~robot_base_link", robot_base_link_, std::string("mrzr/base_link"));
  nh->get_parameter("~fixed_frame", fixed_frame_, std::string("map"));
  nh->get_parameter("~use_pca_box", use_pca_box,  false);
  nh->get_parameter("~use_tracking", use_tracking,  true);
  nh->get_parameter("~voxel_grid_size", voxel_grid_size,  0.2f);
  nh->get_parameter("~roi_max_x", roi_max_x,  70.0f);
  nh->get_parameter("~roi_max_y", roi_max_y,  30.0f);
  nh->get_parameter("~roi_max_z", roi_max_z,  3.0f);
  nh->get_parameter("~roi_min_x", roi_min_x,  -5.0f);
  nh->get_parameter("~roi_min_y", roi_min_y,  -30.0f);
  nh->get_parameter("~roi_min_z", roi_min_z,  -2.5f);
  nh->get_parameter("~body_max_x", body_max_x,  0.3f);
  nh->get_parameter("~body_max_y", body_max_y,  0.8f);
  nh->get_parameter("~body_max_z", body_max_z,  2.0f);
  nh->get_parameter("~body_min_x", body_min_x,  -2.2f);
  nh->get_parameter("~body_min_y", body_min_y,  -0.8f);
  nh->get_parameter("~body_min_z", body_min_z,  -0.3f);
  nh->get_parameter("~ground_threshold", ground_threshold,  0.3f);
  nh->get_parameter("~cluster_threshold", cluster_threshold,  0.6f);
  nh->get_parameter("~cluster_max_size", cluster_max_size,  5000);
  nh->get_parameter("~cluster_min_size", cluster_min_size,  10);
  nh->get_parameter("~displacement_threshold", displacement_threshold,  1.0f);
  nh->get_parameter("~iou_threshold", iou_threshold,  1.0f);
  nh->get_parameter("~ground_normal_x", ground_normal_x,  0.0f);
  nh->get_parameter("~ground_normal_y", ground_normal_y,  0.0f);
  nh->get_parameter("~ground_normal_z", ground_normal_z,  1.0f);
  nh->get_parameter("~ground_normal_threshold", ground_normal_threshold,  0.4f);
  nh->get_parameter("~obstacle_scale", obstacle_scale,  1.0f);
  nh->get_parameter("~obstacle_min_neighbors", obstacle_min_neighbors,  10);
  nh->get_parameter("~publish_seg_as_one", publish_seg_as_one,  false);
  nh->get_parameter("~publish_seg_as_one_field", publish_seg_as_one_field,  std::string("gnd_seg"));

  roi_max_point = Eigen::Vector4f(roi_max_x, roi_max_y, roi_max_z, 1);
  roi_min_point = Eigen::Vector4f(roi_min_x, roi_min_y, roi_min_z, 1);
  body_max_point = Eigen::Vector4f(body_max_x, body_max_y, body_max_z, 1);
  body_min_point = Eigen::Vector4f(body_min_x, body_min_y, body_min_z, 1);
  ground_normal = Eigen::Vector3f(ground_normal_x, ground_normal_y, ground_normal_z);

  auto sub_lidar_points = nh->create_subscription<avt_341::msg::PointCloud2>(lidar_points_topic, 1, lidarPointsCallback);
  pub_cloud_ground = nh->create_publisher<avt_341::msg::PointCloud2>(cloud_ground_topic, 1);
  pub_cloud_clusters = nh->create_publisher<avt_341::msg::PointCloud2>(cloud_clusters_topic, 1);

  // Create point processor
  obstacle_detector = std::make_shared<avt_341::perception::LidarObstacleDetector<pcl::PointXYZ>>();
  obstacle_id_ = 0;

  nh->spin();
  return 0;
}
