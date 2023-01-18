/* obstacle_detector_node.cpp

 * Copyright (C) 2021 SS47816

 * ROS Node for 3D LiDAR Obstacle Detection & Tracking Algorithms

**/
#include <chrono>

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <dynamic_reconfigure/server.h>
#include <avt_341/object_fusion_Config.h>

#include "avt_341/perception/box.hpp"

namespace avt_341 {
namespace perception {

// Pointcloud Filtering Parameters
bool USE_PCA_BOX;
bool USE_TRACKING;
float VOXEL_GRID_SIZE;
Eigen::Vector4f ROI_MAX_POINT, ROI_MIN_POINT;
float GROUND_THRESH;
float CLUSTER_THRESH;
int CLUSTER_MAX_SIZE, CLUSTER_MIN_SIZE;
float DISPLACEMENT_THRESH, IOU_THRESH;

/************************************
 * Tracking/Fusion                  *
 ************************************/
void obstacleTracking(const std::vector<Box>& prev_boxes, std::vector<Box>& curr_boxes, const float displacement_thresh, const float iou_thresh);
bool compareBoxes(const Box& a, const Box& b, const float displacement_thresh, const float iou_thresh);
std::vector<std::vector<int>> associateBoxes(const std::vector<Box>& prev_boxes, const std::vector<Box>& curr_boxes, const float displacement_thresh, const float iou_thresh);
std::vector<std::vector<int>> connectionMatrix(const std::vector<std::vector<int>>& connection_pairs, std::vector<int>& left, std::vector<int>& right);
bool hungarianFind(const int i, const std::vector<std::vector<int>>& connection_matrix, std::vector<bool>& right_connected, std::vector<int>& right_pair);
std::vector<int> hungarian(const std::vector<std::vector<int>>& connection_matrix); 
int searchBoxIndex(const std::vector<Box>& Boxes, const int id);

void obstacleTracking(const std::vector<Box>& prev_boxes, std::vector<Box>& curr_boxes, const float displacement_thresh, const float iou_thresh)
{
  // Tracking (based on the change in size and displacement between frames)
  
  if (curr_boxes.empty() || prev_boxes.empty())
  {
    return;
  }
  else
  {
    // vectors containing the id of boxes in left and right sets
    std::vector<int> pre_ids;
    std::vector<int> cur_ids;
    std::vector<int> matches;

    // Associate Boxes that are similar in two frames
    auto connection_pairs = associateBoxes(prev_boxes, curr_boxes, displacement_thresh, iou_thresh);

    if (connection_pairs.empty()) return;

    // Construct the connection matrix for Hungarian Algorithm's use
    auto connection_matrix = connectionMatrix(connection_pairs, pre_ids, cur_ids);

    // Use Hungarian Algorithm to solve for max-matching
    matches = hungarian(connection_matrix);

    for (int j = 0; j < matches.size(); ++j)
    {
      // find the index of the previous box that the current box corresponds to
      const auto pre_id = pre_ids[matches[j]];
      const auto pre_index = searchBoxIndex(prev_boxes, pre_id);
      
      // find the index of the current box that needs to be changed
      const auto cur_id = cur_ids[j]; // right and matches has the same size
      const auto cur_index = searchBoxIndex(curr_boxes, cur_id);
      
      if (pre_index > -1 && cur_index > -1)
      {
        // change the id of the current box to the same as the previous box
        curr_boxes[cur_index].id = prev_boxes[pre_index].id;
      }
    }
  }
}

bool compareBoxes(const Box& a, const Box& b, const float displacement_thresh, const float iou_thresh)
{
  // Percetage Displacements ranging between [0.0, +oo]
  const float dis = sqrt((a.position[0] - b.position[0]) * (a.position[0] - b.position[0]) + (a.position[1] - b.position[1]) * (a.position[1] - b.position[1]) + (a.position[2] - b.position[2]) * (a.position[2] - b.position[2]));

  const float a_max_dim = std::max(a.dimension[0], std::max(a.dimension[1], a.dimension[2]));
  const float b_max_dim = std::max(b.dimension[0], std::max(b.dimension[1], b.dimension[2]));
  const float ctr_dis = dis / std::min(a_max_dim, b_max_dim);

  // Dimension similiarity values between [0.0, 1.0]
  const float x_dim = 2 * (a.dimension[0] - b.dimension[0]) / (a.dimension[0] + b.dimension[0]);
  const float y_dim = 2 * (a.dimension[1] - b.dimension[1]) / (a.dimension[1] + b.dimension[1]);
  const float z_dim = 2 * (a.dimension[2] - b.dimension[2]) / (a.dimension[2] + b.dimension[2]);

  if (ctr_dis <= displacement_thresh && x_dim <= iou_thresh && y_dim <= iou_thresh && z_dim <= iou_thresh)
  {
    return true;
  }
  else
  {
    return false;
  }
}

std::vector<std::vector<int>> associateBoxes(const std::vector<Box>& prev_boxes, const std::vector<Box>& curr_boxes, const float displacement_thresh, const float iou_thresh)
{
  std::vector<std::vector<int>> connection_pairs;

  for (auto& prev_box : prev_boxes)
  {
    for (auto& curBox : curr_boxes)
    {
      // Add the indecies of a pair of similiar boxes to the matrix
      if (compareBoxes(curBox, prev_box, displacement_thresh, iou_thresh))
      {
        connection_pairs.push_back({prev_box.id, curBox.id});
      }
    }
  }

  return connection_pairs;
}

std::vector<std::vector<int>> connectionMatrix(const std::vector<std::vector<int>>& connection_pairs, std::vector<int>& left, std::vector<int>& right)
{
  // Hash the box ids in the connection_pairs to two vectors(sets), left and right
  for (auto& pair : connection_pairs)
  {
    bool left_found = false;
    for (auto i : left)
    {
      if (i == pair[0])
        left_found = true;
    }
    if (!left_found)
      left.push_back(pair[0]);

    bool right_found = false;
    for (auto j : right)
    {
      if (j == pair[1])
        right_found = true;
    }
    if (!right_found)
      right.push_back(pair[1]);
  }

  std::vector<std::vector<int>> connection_matrix(left.size(), std::vector<int>(right.size(), 0));

  for (auto& pair : connection_pairs)
  {
    int left_index = -1;
    for (int i = 0; i < left.size(); ++i)
    {
      if (pair[0] == left[i])
        left_index = i;
    }

    int right_index = -1;
    for (int i = 0; i < right.size(); ++i)
    {
      if (pair[1] == right[i])
        right_index = i;
    }

    if (left_index != -1 && right_index != -1)
      connection_matrix[left_index][right_index] = 1;
  }

  return connection_matrix;
}

bool hungarianFind(const int i, const std::vector<std::vector<int>>& connection_matrix, std::vector<bool>& right_connected, std::vector<int>& right_pair)
{
  for (int j = 0; j < connection_matrix[0].size(); ++j)
  {
    if (connection_matrix[i][j] == 1 && right_connected[j] == false)
    {
      right_connected[j] = true;

      if (right_pair[j] == -1 || hungarianFind(right_pair[j], connection_matrix, right_connected, right_pair))
      {
        right_pair[j] = i;
        return true;
      }
    }
  }
}

std::vector<int> hungarian(const std::vector<std::vector<int>>& connection_matrix)
{
  std::vector<bool> right_connected(connection_matrix[0].size(), false);
  std::vector<int> right_pair(connection_matrix[0].size(), -1);

  int count = 0;
  for (int i = 0; i < connection_matrix.size(); ++i)
  {
    if (hungarianFind(i, connection_matrix, right_connected, right_pair))
      count++;
  }

  std::cout << "For: " << right_pair.size() << " current frame bounding boxes, found: " << count << " matches in previous frame! " << std::endl;

  return right_pair;
}

int searchBoxIndex(const std::vector<Box>& boxes, const int id)
{
  for (int i = 0; i < boxes.size(); i++)
  {
    if (boxes[i].id == id)
    return i;
  }

  return -1;
}

/************************************
 * Fusion Node                      *
 ************************************/
class ObjectFusionNode
{
 public:
  ObjectFusionNode();
  virtual ~ObjectFusionNode() {};

 private:
  size_t obstacle_id_;
  std::string bbox_target_frame_;
  std::vector<Box> prev_boxes_;
  std::vector<Box> curr_boxes_; 

  ros::NodeHandle nh;
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;
  dynamic_reconfigure::Server<object_fusion::object_fusion_Config> server;
  dynamic_reconfigure::Server<object_fusion::object_fusion_Config>::CallbackType f;

  ros::Subscriber sub_jsk_lidar_raw_bboxes;
  ros::Publisher pub_jsk_bboxes;

  void lidarBBoxCallback(const jsk_recognition_msgs::BoundingBoxArray &curr_boxes_);
  jsk_recognition_msgs::BoundingBox transformJskBbox(const Box& box, const std_msgs::Header& header, const geometry_msgs::Pose& pose_transformed);
  void publishDetectedObjects(const std_msgs::Header& header);
};

// Dynamic parameter server callback function
void dynamicParamCallback(object_fusion::object_fusion_Config& config, uint32_t level)
{
  // Pointcloud Filtering Parameters
  USE_PCA_BOX = config.use_pca_box;
  USE_TRACKING = config.use_tracking;
  VOXEL_GRID_SIZE = config.voxel_grid_size;
  ROI_MAX_POINT = Eigen::Vector4f(config.roi_max_x, config.roi_max_y, config.roi_max_z, 1);
  ROI_MIN_POINT = Eigen::Vector4f(config.roi_min_x, config.roi_min_y, config.roi_min_z, 1);
  GROUND_THRESH = config.ground_threshold;
  CLUSTER_THRESH = config.cluster_threshold;
  CLUSTER_MAX_SIZE = config.cluster_max_size;
  CLUSTER_MIN_SIZE = config.cluster_min_size;
  DISPLACEMENT_THRESH = config.displacement_threshold;
  IOU_THRESH = config.iou_threshold;
}

ObjectFusionNode::ObjectFusionNode() : tf2_listener(tf2_buffer)
{
  ros::NodeHandle private_nh("~");
  
  std::string jsk_raw_lidar_bboxes_topic;
  std::string jsk_bboxes_topic;
  
  ROS_ASSERT(private_nh.getParam("jsk_raw_lidar_bboxes_topic", jsk_raw_lidar_bboxes_topic));
  ROS_ASSERT(private_nh.getParam("jsk_bboxes_topic", jsk_bboxes_topic));
  ROS_ASSERT(private_nh.getParam("bbox_target_frame", bbox_target_frame_));

  sub_jsk_lidar_raw_bboxes = nh.subscribe(jsk_raw_lidar_bboxes_topic, 1, &ObjectFusionNode::lidarBBoxCallback, this);
  pub_jsk_bboxes = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(jsk_bboxes_topic, 1);

  // Dynamic Parameter Server & Function
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  // Create point processor
  obstacle_id_ = 0;
}

void ObjectFusionNode::lidarBBoxCallback(const jsk_recognition_msgs::BoundingBoxArray &jsk_boxes)
{
  ROS_DEBUG("Lidar boxes recieved");
  // Time the whole process
  const auto start_time = std::chrono::steady_clock::now();

  //TODO: Convert jsk_boxes to std::vector<Box>
  curr_boxes_.clear();
  curr_boxes_.reserve(jsk_boxes.boxes.size());
  for(auto jsk_box:jsk_boxes.boxes) {
    Box box;
    auto &pose = jsk_box.pose;
    box.position(0) = pose.position.x;
    box.position(1) = pose.position.y;
    box.position(2) = pose.position.z;
    box.quaternion.w() = pose.orientation.w;
    box.quaternion.x() = pose.orientation.x;
    box.quaternion.y() = pose.orientation.y;
    box.quaternion.z() = pose.orientation.z;
    box.dimension(0) = jsk_box.dimensions.x;
    box.dimension(1) = jsk_box.dimensions.y;
    box.dimension(2) = jsk_box.dimensions.z;
    box.id = jsk_box.label;
    curr_boxes_.push_back(box);
  }

  // Publish Obstacles
  publishDetectedObjects(jsk_boxes.header);

  // Time the whole process
  const auto end_time = std::chrono::steady_clock::now();
  const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  ROS_INFO("The object_fusion_node reported %d obstacles in %.3f second", int(prev_boxes_.size()), float(elapsed_time.count()/1000.0));
}

void ObjectFusionNode::publishDetectedObjects(const std_msgs::Header& header)
{
  const std::string &bbox_source_frame_ = header.frame_id;
  // Re-assign Box ids based on tracking result
  if (USE_TRACKING)
    obstacleTracking(prev_boxes_, curr_boxes_, DISPLACEMENT_THRESH, IOU_THRESH);
  
  // Lookup for frame transform between the lidar frame and the target frame
  auto bbox_header = header;
  bbox_header.frame_id = bbox_target_frame_;
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf2_buffer.lookupTransform(bbox_target_frame_, bbox_source_frame_, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_WARN("Frame Transform Given Up! Outputing obstacles in the original LiDAR frame %s instead...", bbox_source_frame_.c_str());
    bbox_header.frame_id = bbox_source_frame_;
    try
    {
      transform_stamped = tf2_buffer.lookupTransform(bbox_source_frame_, bbox_source_frame_, ros::Time(0));
    }
    catch (tf2::TransformException& ex2)
    {
      ROS_ERROR("%s", ex2.what());
      return;
    }
  }

  // Construct Bounding Boxes from the clusters
  jsk_recognition_msgs::BoundingBoxArray jsk_bboxes;
  jsk_bboxes.header = bbox_header;

  // Transform boxes from lidar frame to base_link frame, and convert to jsk and autoware msg formats
  for (auto& box : curr_boxes_)
  {
    geometry_msgs::Pose pose, pose_transformed;
    pose.position.x = box.position(0);
    pose.position.y = box.position(1);
    pose.position.z = box.position(2);
    pose.orientation.w = box.quaternion.w();
    pose.orientation.x = box.quaternion.x();
    pose.orientation.y = box.quaternion.y();
    pose.orientation.z = box.quaternion.z();
    tf2::doTransform(pose, pose_transformed, transform_stamped);

    jsk_bboxes.boxes.emplace_back(transformJskBbox(box, bbox_header, pose_transformed));
  }
  pub_jsk_bboxes.publish(std::move(jsk_bboxes));

  // Update previous bounding boxes
  prev_boxes_.swap(curr_boxes_);
  curr_boxes_.clear();
}

jsk_recognition_msgs::BoundingBox ObjectFusionNode::transformJskBbox(const Box& box, const std_msgs::Header& header, const geometry_msgs::Pose& pose_transformed)
{
  jsk_recognition_msgs::BoundingBox jsk_bbox;
  jsk_bbox.header = header;
  jsk_bbox.pose = pose_transformed;
  jsk_bbox.dimensions.x = box.dimension(0);
  jsk_bbox.dimensions.y = box.dimension(1);
  jsk_bbox.dimensions.z = box.dimension(2);
  jsk_bbox.value = 1.0f;
  jsk_bbox.label = box.id;

  return std::move(jsk_bbox);
}

} // namespace perception
} // namespace avt_341


int main(int argc, char** argv){
  ros::init(argc, argv, "obstacle_fusion_node");
  avt_341::perception::ObjectFusionNode obstacle_fusion_node;
  ros::spin();
  return 0;
}