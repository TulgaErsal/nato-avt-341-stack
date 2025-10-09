// c++ includes
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <deque>
#include <algorithm>
// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

#ifdef ROS_1
#include "sensor_msgs/point_cloud_conversion.h"
#else
#include "sensor_msgs/point_cloud_conversion.hpp"
#endif

// avt_341 includes
#include "avt_341/perception/elevation_grid.h"

avt_341::perception::ElevationGrid grid;
avt_341::msg::Odometry current_pose;
avt_341::msg::PointCloud ground_points;
bool ground_points_rcvd = false;
bool clear_ground_points;
bool grid_created = false;
double start_time = 0.0;
bool odom_rcvd = false;
std::vector<avt_341::msg::Odometry> current_pose_list;
float overhead_clearance = 100.0f;
double time_register_window = 0.02;
bool cull_lidar_points = false;
float cull_lidar_points_dist_sqr = 10000.0f;
float cull_lidar_points_dist_min_sqr = 0.0f;

float max_grid_width = 0.0f;
float max_grid_height = 0.0f;
double grid_pub_force_full_every_x_sec = 0.0;
double last_full_grid_update = 0.0;

std::shared_ptr<avt_341::node::NodeProxy> n = nullptr;

double CalcLidarPointToRobotDistanceSquared(const avt_341::msg::Point& odom_pose, const avt_341::msg::Point32& point)
{
	double dx = odom_pose.x - point.x;
	double dy = odom_pose.y - point.y;
	double dz = odom_pose.z - point.z;
	return dx * dx + dy * dy + dz * dz;
}

double GetPoseToUse(avt_341::msg::Odometry& pose_to_use, avt_341::msg::PointCloud2Ptr rcv_cloud) {
	double dt = 1.0;
	for (int i = 0; i < current_pose_list.size(); i++) {
		double dt_this = fabs(avt_341::node::seconds_from_header(current_pose_list[i].header) - avt_341::node::seconds_from_header(rcv_cloud->header));
		if (dt_this < dt) {
			pose_to_use = current_pose_list[i];
			dt = dt_this;
		}
	}
	return dt;
}

void PointCloudCallback(avt_341::msg::PointCloud2Ptr rcv_cloud) {
	// assumes point cloud is already registered to odom frame
	avt_341::msg::PointCloud point_cloud;

	bool converted = false;
	if (rcv_cloud->header.frame_id != "odom" && rcv_cloud->header.frame_id != "map") {
		avt_341::msg::PointCloud2 out_cloud;
		if (!n->transform_cloud(*rcv_cloud, out_cloud, "map")) {
			return;
		}
		converted = sensor_msgs::convertPointCloud2ToPointCloud(out_cloud, point_cloud);
	}
	else {
		converted = sensor_msgs::convertPointCloud2ToPointCloud(*rcv_cloud, point_cloud);
	}

	if (clear_ground_points && ground_points_rcvd) {
		grid.ClearPoints(ground_points);
		ground_points_rcvd = false;
	}

	if (converted && odom_rcvd) {
		std::vector<avt_341::msg::Point32> points;
		std::vector<std::vector<float>> channel_values;
		for (int c = 0; c < point_cloud.channels.size(); c++) {
			channel_values.push_back(std::vector<float>());
		}
		for (int p = 0; p < point_cloud.points.size(); p++) {
			avt_341::msg::Point32 tp;
			tp.x = point_cloud.points[p].x;
			tp.y = point_cloud.points[p].y;
			tp.z = point_cloud.points[p].z;
			if (!(tp.x == 0.0 && tp.y == 0.0) && !std::isnan(tp.x) && (tp.z - current_pose.pose.pose.position.z) < overhead_clearance) {

				bool add_point = true;
				if (cull_lidar_points)
				{
					avt_341::msg::Odometry pose_to_use;
					GetPoseToUse(pose_to_use, rcv_cloud);
					const double point_dist = CalcLidarPointToRobotDistanceSquared(pose_to_use.pose.pose.position, tp);
					add_point = cull_lidar_points_dist_min_sqr < point_dist && point_dist < cull_lidar_points_dist_sqr;
				}
				if (add_point) {
					points.push_back(tp);
					for (int c = 0; c < point_cloud.channels.size(); c++) {
						channel_values[c].push_back(point_cloud.channels[c].values[p]);
					}
				}

			}
		}
		point_cloud.points.clear();
		point_cloud.points = points;
		for (int c = 0; c < point_cloud.channels.size(); c++) {
			point_cloud.channels[c].values = channel_values[c];
		}
		point_cloud.header.frame_id = rcv_cloud->header.frame_id;
		point_cloud.header.stamp = rcv_cloud->header.stamp;
		grid.AddPoints(point_cloud);
		grid_created = true;
	}
}

void GroundCallback(avt_341::msg::PointCloud2Ptr rcv_cloud) {
	// assumes point cloud is already registered to odom frame
	avt_341::msg::PointCloud point_cloud;

	bool converted = false;
	if (rcv_cloud->header.frame_id != "odom" && rcv_cloud->header.frame_id != "map") {
		avt_341::msg::PointCloud2 out_cloud;
		if (!n->transform_cloud(*rcv_cloud, out_cloud, "map")) {
			return;
		}
		converted = sensor_msgs::convertPointCloud2ToPointCloud(out_cloud, point_cloud);
	}
	else {
		converted = sensor_msgs::convertPointCloud2ToPointCloud(*rcv_cloud, point_cloud);
	}

	if (converted && odom_rcvd) {
		ground_points = point_cloud;
		ground_points_rcvd = true;
	}
}

void ResetNode() {
	if (n == nullptr) {
		return;
	}
	n->log_info("Resetting node");
	grid.Reset();
	grid_created = false;
}

bool reset_called = false;
void ResetCallback(avt_341::msg::StringPtr msg) {
	if (msg->data.find(avt_341::node::NodeType::Perception) != std::string::npos) {
		reset_called = true;
	}
}

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom) {
	current_pose = *rcv_odom;
	odom_rcvd = true;
	current_pose_list.push_back(current_pose);
	if (current_pose_list.size() > 50) current_pose_list.erase(current_pose_list.begin());
}

bool PublishGrid(bool is_segmentation, bool limit_grid_size, bool is_full_grid_pub, double now_seconds,
	const avt_341::node::Publisher<avt_341::msg::OccupancyGrid>::SharedPtr& grid_pub,
	const avt_341::node::Publisher<avt_341::msg::OccupancyGridUpdate>::SharedPtr& grid_pub_updates) {

	avt_341::msg::OccupancyGrid grid_msg;
	bool is_full_update = false;
	if (limit_grid_size) {
		grid_msg = grid.GetGrid(current_pose.pose.pose.position.x,current_pose.pose.pose.position.y,max_grid_width,max_grid_height, is_segmentation);
	}else {
		is_full_update = is_full_grid_pub || (now_seconds - last_full_grid_update > grid_pub_force_full_every_x_sec);
		if (is_full_update) {
			last_full_grid_update = now_seconds;
			grid_msg = grid.GetGrid(is_segmentation);
		}else {
			avt_341::msg::OccupancyGridUpdate grid_update_msg;
			grid_update_msg = grid.GetGridUpdate(is_segmentation);
			if (grid_update_msg.height > 0 && grid_update_msg.width > 0) {
				grid_update_msg.header.stamp = n->get_stamp();
				grid_pub_updates->publish(grid_update_msg);
			}
			return false;
		}
	}

	grid_msg.header.stamp = n->get_stamp();
	grid_pub->publish(grid_msg);
	return is_full_update;
}

int main(int argc, char* argv[]) {

	grid_created = false;

	n = avt_341::node::init_node(argc, argv, "avt_341_perception_node");
	n->initialize_tf_listener();

	float grid_width, grid_height, visualization_range;
	n->get_parameter("/grid_width", grid_width, 200.0f);
	n->get_parameter("/grid_height", grid_height, 200.0f);
	grid.SetSize(grid_width, grid_height);

	float grid_res, grid_llx, grid_lly, warmup_time, thresh, thresh_max, grid_dilate_x, grid_dilate_y, grid_dilate_proportion, voxel_height_min, voxel_height_res, clear_method_raytrace_range, clear_method_obj_range_filter;
	bool use_elevation, grid_dilate, clear_method_visualize, clear_method_use_voxels, clear_method_clear_dilation, limit_grid_size;
	int sampled_threshold;
	std::string clear_method, grid_pub_method;
	double perception_rate;
	std::string perception_points_topic, ground_points_topic;
	float rms_horizontal_fov_radians, rms_range_meters, rms_time_average_window;

	n->get_parameter("~rms_calc_horizontal_fov_radians", rms_horizontal_fov_radians, 0.7854f); // about 45 degrees
	n->get_parameter("~rms_calc_range_meters", rms_range_meters, 15.0f);
	n->get_parameter("~rms_calc_time_average_window", rms_time_average_window, 1.0f);
	n->get_parameter("~grid_res", grid_res, 1.0f);
	n->get_parameter("~grid_llx", grid_llx, -100.0f);
	n->get_parameter("~grid_lly", grid_lly, -100.0f);
	n->get_parameter("~time_register_window", time_register_window, 0.02);
	n->get_parameter("~warmup_time", warmup_time, 1.0f);
	n->get_parameter("~slope_threshold", thresh, 0.5f);
	n->get_parameter("~slope_threshold_max", thresh_max, 2.5f);
	n->get_parameter("~use_elevation", use_elevation, false);
	n->get_parameter("~grid_dilate", grid_dilate, true);
	n->get_parameter("~grid_dilate_x", grid_dilate_x, 1.0f);
	n->get_parameter("~grid_dilate_y", grid_dilate_y, 1.0f);
	n->get_parameter("~grid_dilate_proportion", grid_dilate_proportion, 0.8f);
	n->get_parameter("~overhead_clearance", overhead_clearance, 100.0f);
	n->get_parameter("~perception_rate", perception_rate, 100.0);
	n->get_parameter("~max_grid_width", max_grid_width, 800.0f);
	n->get_parameter("~max_grid_height", max_grid_height, 800.0f);
	n->get_parameter("~clear_ground_points", clear_ground_points, false);

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

	n->get_parameter("~grid_pub_method", grid_pub_method, avt_341::perception::GridPubMethod::Full);
	n->get_parameter("~grid_pub_force_full_every", grid_pub_force_full_every_x_sec, 10.0);

	if (!avt_341::perception::GridPubMethod::IsGridPubMethodValid(grid_pub_method)){
		n->log_error("Invalid grid_pub_method: %hs", grid_pub_method.c_str());
		return -1;
	}

	limit_grid_size = grid_pub_method == avt_341::perception::GridPubMethod::Window;
	bool is_full_grid_pub = grid_pub_method == avt_341::perception::GridPubMethod::Full;
	bool is_updates_grid_pub = grid_pub_method == avt_341::perception::GridPubMethod::Updates;

	bool stitch_points;
	n->get_parameter("~stitch_lidar_points", stitch_points, true);
	float max_point_age;
	n->get_parameter("~clear_method_max_point_age", max_point_age, 5.0f);
	bool filter_highest_lidar;
	n->get_parameter("~filter_highest_lidar", filter_highest_lidar, false);
	float cull_lidar_points_dist, cull_lidar_points_dist_min;
	n->get_parameter("~cull_lidar", cull_lidar_points, false);
	n->get_parameter("~cull_lidar_dist", cull_lidar_points_dist, 100.0f);
	n->get_parameter("~cull_lidar_dist_min", cull_lidar_points_dist_min, 0.0f);
	cull_lidar_points_dist_sqr = cull_lidar_points_dist * cull_lidar_points_dist;
	cull_lidar_points_dist_min_sqr = cull_lidar_points_dist_min * cull_lidar_points_dist_min;

	n->get_parameter("~perception_points_topic", perception_points_topic, std::string("avt_341/points"));
	n->get_parameter("~ground_points_topic", ground_points_topic, std::string("avt_341/ground_points"));

	auto pc_sub = n->create_subscription<avt_341::msg::PointCloud2>(perception_points_topic, 10, PointCloudCallback);
	auto pc_ground_sub = n->create_subscription<avt_341::msg::PointCloud2>(ground_points_topic, 10, GroundCallback);
	auto odom_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
	auto reset_sub = n->create_subscription<avt_341::msg::String>("avt_341/reset", 10, ResetCallback);
	auto grid_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/occupancy_grid", 1);
	auto reset_ack_pub = n->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);
	auto grid_segmentation_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/segmentation_grid", 1);

	auto rms_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/terrain_rms", 1);
	auto terrain_slope_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/terrain_slope", 1);

	auto grid_pub_updates = is_updates_grid_pub ? n->create_publisher<avt_341::msg::OccupancyGridUpdate>("avt_341/occupancy_grid_updates", 1) : nullptr;
	auto grid_segmentation_pub_updates = is_updates_grid_pub ? n->create_publisher<avt_341::msg::OccupancyGridUpdate>("avt_341/segmentation_grid_updates", 1) :  nullptr;

	n->log_info("Perception node settings:\n"
				"	grid_res: %.2f\n"
				"	slope_threshold: %.2f\n"
				"	slope_threshold_max: %.2f\n "
				"	grid_dilate: %d\n"
				"	grid_pub_method: %hs\n"
				"	grid_pub_force_full_every: %.2f",
				grid_res,
				thresh,
				thresh_max,
				grid_dilate,
				grid_pub_method.c_str(),
				grid_pub_force_full_every_x_sec
				);

	grid.SetSlopeThreshold(thresh, thresh_max);
	grid.SetRes(grid_res);
	grid.SetCorner(grid_llx, grid_lly);
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
	int n_rms_avg = static_cast<int>(rms_time_average_window * perception_rate);
	std::deque<double> rms_buffer;
	std::deque<double> slope_buffer;
	int nloops = 0;
	while (avt_341::node::ok()) {

		const double now_seconds = n->get_now_seconds();

		if (odom_rcvd && (now_seconds - start_time) > warmup_time) {

			bool is_full_update = PublishGrid(false, limit_grid_size, is_full_grid_pub, now_seconds, grid_pub, grid_pub_updates);
			if (grid.has_segmentation()) {
				PublishGrid(true, limit_grid_size, is_full_grid_pub, now_seconds, grid_segmentation_pub, grid_segmentation_pub_updates);
			}
			if (is_full_update){
				last_full_grid_update = now_seconds;
			}

			// get the slope and RMS
			avt_341::msg::Float64 rms_msg, slope_msg;
			float heading = avt_341::utils::GetHeadingFromOrientation(current_pose.pose.pose.orientation);
			float current_rms, current_slope;
			grid.GetSlopeRmsInFov(current_slope, current_rms, current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, heading, rms_horizontal_fov_radians, rms_range_meters);
			slope_buffer.push_back((double)current_slope);
			rms_buffer.push_back((double)current_rms);
			if (slope_buffer.size() > n_rms_avg) {
				slope_buffer.pop_front();
				rms_buffer.pop_front();
			}
			rms_msg.data = std::accumulate(rms_buffer.begin(), rms_buffer.end(), 0.0)/rms_buffer.size();
			slope_msg.data = std::accumulate(slope_buffer.begin(), slope_buffer.end(), 0.0) / slope_buffer.size();

			rms_pub->publish(rms_msg);
			terrain_slope_pub->publish(slope_msg);

			if (clear_method_visualize && nloops % 20 == 0) {
				grid.Visualize();
			}

			nloops++;

		}

		if (reset_called) {
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
