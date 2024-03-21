// c++ includes
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

#ifdef ROS_1
#include "sensor_msgs/point_cloud_conversion.h"
#else
#define SENSOR_MSGS_SKIP_WARNING
#include "sensor_msgs/point_cloud_conversion.hpp"
#endif

// avt_341 includes
#include "avt_341/perception/elevation_grid.h"

avt_341::perception::ElevationGrid grid;
avt_341::msg::Odometry current_pose;
bool grid_created = false;
double start_time = 0.0;
bool odom_rcvd = false;
std::vector<avt_341::msg::Odometry> current_pose_list;
float overhead_clearance = 100.0f;
double time_register_window = 0.02;
bool cull_lidar_points = false;
float cull_lidar_points_dist_sqr = 10000.0f;
float cull_lidar_points_dist_min_sqr = 0.0f;
std::shared_ptr<avt_341::node::NodeProxy> n = nullptr;

double CalcLidarPointToRobotDistanceSquared(const avt_341::msg::Point& odom_pose, const avt_341::msg::Point32& point)
{
	double dx = odom_pose.x - point.x;
	double dy = odom_pose.y - point.y;
	double dz = odom_pose.z - point.z;
	return dx*dx + dy*dy + dz*dz;
}

double GetPoseToUse(avt_341::msg::Odometry & pose_to_use, avt_341::msg::PointCloud2Ptr rcv_cloud){
  double dt = 1.0;
  for (int i=0;i<current_pose_list.size();i++){
    double dt_this = fabs(avt_341::node::seconds_from_header(current_pose_list[i].header) - avt_341::node::seconds_from_header(rcv_cloud->header));
    if (dt_this<dt){
      pose_to_use = current_pose_list[i];
      dt = dt_this;
    }
  }
	return dt;
}

void PointCloudCallback(avt_341::msg::PointCloud2Ptr rcv_cloud){
	// assumes point cloud is already registered to odom frame
	avt_341::msg::PointCloud point_cloud;

  bool converted = false;
  if(rcv_cloud->header.frame_id != "odom" && rcv_cloud->header.frame_id != "map"){
    avt_341::msg::PointCloud2 out_cloud;
    if(!n->transform_cloud(*rcv_cloud, out_cloud, "map")){
      return;
    }
    converted = sensor_msgs::convertPointCloud2ToPointCloud(out_cloud, point_cloud);
  }else{
    converted = sensor_msgs::convertPointCloud2ToPointCloud(*rcv_cloud, point_cloud);
  }

	if (converted && odom_rcvd){
		std::vector<avt_341::msg::Point32> points;
		std::vector<std::vector<float>> channel_values;
		for(int c = 0; c < point_cloud.channels.size(); c++){
			channel_values.push_back(std::vector<float>());
		}
		for (int p=0;p<point_cloud.points.size();p++){
			avt_341::msg::Point32 tp;
			tp.x = point_cloud.points[p].x;
			tp.y = point_cloud.points[p].y;
			tp.z = point_cloud.points[p].z;
			if ( !(tp.x==0.0 && tp.y==0.0) && !std::isnan(tp.x) && (tp.z-current_pose.pose.pose.position.z)<overhead_clearance ){

				bool add_point = true;
				if (cull_lidar_points)
				{
					avt_341::msg::Odometry pose_to_use;
					GetPoseToUse(pose_to_use, rcv_cloud);
          const double point_dist = CalcLidarPointToRobotDistanceSquared(pose_to_use.pose.pose.position, tp);
          add_point = cull_lidar_points_dist_min_sqr < point_dist && point_dist < cull_lidar_points_dist_sqr;
				}
				if(add_point){
					points.push_back(tp);
					for(int c = 0; c < point_cloud.channels.size(); c++){
						channel_values[c].push_back(point_cloud.channels[c].values[p]);
					}
				}

			}
		}
		point_cloud.points.clear();
		point_cloud.points = points;
		for(int c = 0; c < point_cloud.channels.size(); c++){
			point_cloud.channels[c].values = channel_values[c];
		}
    point_cloud.header.frame_id = rcv_cloud->header.frame_id;
    point_cloud.header.stamp = rcv_cloud->header.stamp;
		grid.AddPoints(point_cloud);
		grid_created = true;
	}
}

void ResetNode(){
  if(n == nullptr){
    return;
  }
  n->log_info("Resetting node");
  grid.Reset();
  grid_created = false;
}

bool reset_called = false;
void ResetCallback(avt_341::msg::StringPtr msg){
  if(msg->data.find(avt_341::node::NodeType::Perception) != std::string::npos){
    reset_called = true;
  }
}

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom){
	current_pose = *rcv_odom;
	odom_rcvd = true;
	current_pose_list.push_back(current_pose);
	if (current_pose_list.size()>50) current_pose_list.erase(current_pose_list.begin());
}

int main(int argc, char *argv[]) {

  grid_created = false;

  n = avt_341::node::init_node(argc, argv, "avt_341_perception_node");
  n->initialize_tf_listener();
  auto pc_sub = n->create_subscription<avt_341::msg::PointCloud2>("avt_341/points",10,PointCloudCallback);
  auto odom_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry",10, OdometryCallback);
  auto reset_sub = n->create_subscription<avt_341::msg::String>("avt_341/reset", 10, ResetCallback);
  auto grid_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/occupancy_grid", 1);
  auto reset_ack_pub = n->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);
  auto grid_segmentation_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/segmentation_grid", 1);

  float grid_width, grid_height, visualization_range;
  n->get_parameter("~grid_width", grid_width, 200.0f);
  n->get_parameter("~grid_height", grid_height, 200.0f);
  grid.SetSize(grid_width,grid_height);

  float grid_res, grid_llx, grid_lly, warmup_time, thresh, grid_dilate_x, grid_dilate_y, grid_dilate_proportion, voxel_height_min, voxel_height_res, clear_method_raytrace_range, clear_method_obj_range_filter;
  bool use_elevation, grid_dilate, clear_method_visualize, clear_method_use_voxels, clear_method_clear_dilation;
  int sampled_threshold;
  std::string clear_method;
  double perception_rate;

  n->get_parameter("~grid_res", grid_res, 1.0f);
  n->get_parameter("~grid_llx", grid_llx, -100.0f);
  n->get_parameter("~grid_lly", grid_lly, -100.0f);
  n->get_parameter("~time_register_window", time_register_window, 0.02);
  n->get_parameter("~warmup_time", warmup_time, 1.0f);
  n->get_parameter("~slope_threshold", thresh, 1.0f);
  n->get_parameter("~use_elevation", use_elevation, false);
  n->get_parameter("~grid_dilate", grid_dilate, true);
  n->get_parameter("~grid_dilate_x", grid_dilate_x, 1.0f);
  n->get_parameter("~grid_dilate_y", grid_dilate_y, 1.0f);
  n->get_parameter("~grid_dilate_proportion", grid_dilate_proportion, 0.8f);
  n->get_parameter("~overhead_clearance", overhead_clearance, 100.0f);
  n->get_parameter("~perception_rate", perception_rate, 100.0);

  n->get_parameter("~clear_method_type", clear_method, std::string("none"));
  n->get_parameter("~clear_method_visualize", clear_method_visualize, false);
  n->get_parameter("~clear_method_visualize_range", visualization_range, 40.0f);
  n->get_parameter("~clear_method_raytrace_range", clear_method_raytrace_range, 50.0f);
  n->get_parameter("~clear_method_use_voxels", clear_method_use_voxels, true);
  n->get_parameter("~clear_method_voxel_height_min", voxel_height_min, 0.0f);
  n->get_parameter("~clear_method_voxel_height_res", voxel_height_res, 0.5f);
  n->get_parameter("~clear_method_immediate_clear_dilation", clear_method_clear_dilation, true);
  n->get_parameter("~clear_method_obs_filter_range", clear_method_obj_range_filter, 1.0f);
  n->get_parameter("~clear_method_sampled_threshold", sampled_threshold, 5);

	bool stitch_points;
	n->get_parameter("~stitch_lidar_points", stitch_points, true);
	float max_point_age;
	n->get_parameter("~clear_method_max_point_age",max_point_age,5.0f);
	bool filter_highest_lidar;
	n->get_parameter("~filter_highest_lidar", filter_highest_lidar, false);
  float cull_lidar_points_dist, cull_lidar_points_dist_min;
  n->get_parameter("~cull_lidar", cull_lidar_points, false);
  n->get_parameter("~cull_lidar_dist", cull_lidar_points_dist, 100.0f);
  n->get_parameter("~cull_lidar_dist_min", cull_lidar_points_dist_min, 0.0f);
  cull_lidar_points_dist_sqr = cull_lidar_points_dist * cull_lidar_points_dist;
  cull_lidar_points_dist_min_sqr = cull_lidar_points_dist_min * cull_lidar_points_dist_min;

	grid.SetSlopeThreshold(thresh);
	grid.SetRes(grid_res);
	grid.SetCorner(grid_llx,grid_lly);
	grid.SetUseElevation(use_elevation);
	grid.SetDilation(grid_dilate, grid_dilate_x, grid_dilate_y, grid_dilate_proportion);
	grid.SetStitchPoints(stitch_points);
	grid.SetFilterHighest(filter_highest_lidar);
	grid.SetMaxPointAge(max_point_age);
	grid.SetCostmapClearingMethod(n, clear_method, visualization_range, clear_method_visualize,
                                clear_method_raytrace_range, clear_method_clear_dilation, clear_method_use_voxels,
                                voxel_height_min, voxel_height_res, clear_method_obj_range_filter, sampled_threshold);

  ResetNode();
  start_time = n->get_now_seconds();
  avt_341::node::Rate rate(perception_rate);
  int nloops = 0;
	while (avt_341::node::ok()){
		double elapsed_time = (n->get_now_seconds()-start_time);
		if (grid_created && elapsed_time > warmup_time) {
			avt_341::msg::OccupancyGrid grd;
      		grd = grid.GetGrid();
			grd.header.stamp = n->get_stamp();
			grid_pub->publish(grd);

			if(grid.has_segmentation()){
				grd = grid.GetGrid(true);
				grd.header.stamp = n->get_stamp();
				grid_segmentation_pub->publish(grd);
			}

      if(clear_method_visualize && nloops % 20 == 0){
        grid.Visualize();
      }

			nloops++;

		}
    
    if(reset_called){
      ResetNode();
      avt_341::msg::String reset_ack_msg;
      reset_ack_msg.data = avt_341::node::NodeType::Perception;
      reset_ack_pub->publish(reset_ack_msg);
      reset_called = false;
    }

		n->spin_some();
		rate.sleep();
	}

	return 0;
}
