// c++ includes
#include <deque>
// ros includes
#include <avt_341/avt_341_utils.h>
#include <avt_341/core/monitoring.hpp>

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
double start_time = 0.0;
double pc_callback_runtime_threshold = 0.0; // in seconds

float max_grid_width = 0.0f;
float max_grid_height = 0.0f;
double grid_pub_force_full_every_x_sec = 0.0;
double last_full_grid_update = 0.0;

avt_341::core::WindowedMean pc_callback_time(40);
std::shared_ptr<avt_341::node::NodeProxy> n = nullptr;


void PointCloudCallback(avt_341::msg::PointCloud2Ptr rcv_cloud) {

	const double callback_start_time = n->get_now_seconds();

	std::shared_ptr<avt_341::msg::PointCloud2> pc2_ptr = rcv_cloud;
	std::shared_ptr<avt_341::msg::PointCloud> pc_ptr = std::make_shared<avt_341::msg::PointCloud>();

	if (rcv_cloud->header.frame_id != "odom" && rcv_cloud->header.frame_id != "map") {
		pc2_ptr = std::make_shared<avt_341::msg::PointCloud2>();
		if (!n->transform_cloud(*rcv_cloud, *pc2_ptr, "map")) {
			return;
		}
	}

	if (!sensor_msgs::convertPointCloud2ToPointCloud(*pc2_ptr, *pc_ptr)) {
		return;
	}

	avt_341::msg::TransformStamped origin_tx = n->lookup_transform("map", current_pose.child_frame_id);
	avt_341::msg::Pose origin_pose = avt_341::utils::TransformToPose(origin_tx.transform);

	grid.AddPoints(pc_ptr, origin_pose);

	pc_callback_time.AddSample(n->get_now_seconds() - callback_start_time);
	if (pc_callback_time.GetMean() > pc_callback_runtime_threshold) {
		n->log_warning_throttle(1.0, "PointCloudCallback took %.2f ms (> %.2f ms warning threshold).",
			pc_callback_time.GetMean()*1e3,
			pc_callback_runtime_threshold*1e3
			);
	}
}

avt_341::perception::ClearMethodRosParameters ParseClearMethodsConfig() {

	avt_341::perception::ClearMethodRosParameters params;

	// General settings
	n->get_parameter("~clear_method_type", params.clear_methods_str, std::string("none"));
	n->get_parameter("~clear_method_visualize", params.visualize, false);
	n->get_parameter("~clear_method_visualize_range", params.visualization_range, 40.0f);

	// Raytrace clearing
	n->get_parameter("~clear_method_raytrace_range", params.raytrace_range, 50.0f);
	n->get_parameter("~clear_method_use_voxels", params.use_voxels, true);
	n->get_parameter("~clear_method_voxel_height_min", params.voxel_height_min, 0.0f);
	n->get_parameter("~clear_method_voxel_height_res", params.voxel_height_res, 0.5f);
	n->get_parameter("~clear_method_immediate_clear_dilation", params.immediate_clr_dilation, true);
	n->get_parameter("~clear_method_clr_on_scan_below_only", params.clr_on_scan_below_only, false);
	n->get_parameter("~clear_method_lidar_frame", params.lidar_frame, std::string("lidar"));

	// Raytrace clearing + object filter
	n->get_parameter("~clear_method_obs_filter_range", params.obj_range_filter, 1.0f);

	// Time and timed no-obs clearing
	n->get_parameter("~clear_method_sampled_threshold", params.sampled_threshold, 5);
	n->get_parameter("~clear_method_max_point_age", params.max_point_age, 5.0f);
	n->get_parameter("~clear_method_no_obs_dist_threshold", params.no_obs_dist_threshold, 0.25f);

	// Channel-based clearing
	n->get_parameter("~clear_method_channel_to_clear", params.channel_to_clear, std::string("gnd_seg"));
	n->get_parameter("~clear_method_channel_threshold", params.channel_threshold, 0.5f);

	return params;
}

avt_341::perception::PointCloudFilterConfig ParseFilterConfig(const std::string &param_prefix = "") {

	avt_341::perception::PointCloudFilterConfig config;
	n->get_parameter("~" + param_prefix + "cull_lidar", config.enable_dist_filter, false);
	n->get_parameter("~" + param_prefix + "cull_lidar_dist", config.max_dist, -1.0);
	n->get_parameter("~" + param_prefix + "cull_lidar_dist_min", config.min_dist, -1.0);
	n->get_parameter("~" + param_prefix + "cull_lidar_hfov_min", config.min_hfov, -180.0);
	n->get_parameter("~" + param_prefix + "cull_lidar_hfov_max", config.max_hfov, 180.0);
	n->get_parameter("~" + param_prefix + "overhead_clearance", config.max_height_clearance, -1.0);
	return config;
}

void ResetNode() {
	if (n == nullptr) {
		return;
	}
	n->log_info("Resetting node");
	grid.Reset();
}

bool reset_called = false;
void ResetCallback(avt_341::msg::StringPtr msg) {
	if (msg->data.find(avt_341::node::NodeType::Perception) != std::string::npos) {
		reset_called = true;
	}
}

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom) {
	current_pose = *rcv_odom;
}

bool PublishGrid(bool is_segmentation, const std::string & grid_pub_method, double now_seconds,
	const avt_341::node::Publisher<avt_341::msg::OccupancyGrid>::SharedPtr& grid_pub,
	const avt_341::node::Publisher<avt_341::msg::OccupancyGridUpdate>::SharedPtr& grid_pub_updates) {

	avt_341::msg::OccupancyGrid grid_msg;
	bool is_full_update = false;
	if (grid_pub_method == avt_341::perception::GridPubMethod::Window) {
		grid_msg = grid.GetGrid(
			current_pose.pose.pose.position.x,
			current_pose.pose.pose.position.y,
			max_grid_width,
			max_grid_height,
			is_segmentation
			);
	}else {
		const bool is_full_grid_pub = grid_pub_method == avt_341::perception::GridPubMethod::Full;
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

	n = avt_341::node::init_node(argc, argv, "avt_341_perception_node");
	n->initialize_tf_listener();

	// Read parameters
	// --------------------------------------------------------------------------------------------------------------

	float grid_width, grid_height;
	float grid_res, grid_llx, grid_lly, warmup_time, thresh, thresh_max, grid_dilate_x, grid_dilate_y, grid_dilate_proportion;
	bool use_elevation, grid_dilate;
	std::string clear_method, grid_pub_method;
	double perception_rate;
	std::string perception_points_topic;
	float rms_horizontal_fov_radians, rms_range_meters, rms_time_average_window;

	n->get_parameter("/grid_width", grid_width, 200.0f);
	n->get_parameter("/grid_height", grid_height, 200.0f);
	n->get_parameter("~rms_calc_horizontal_fov_radians", rms_horizontal_fov_radians, 0.7854f); // about 45 degrees
	n->get_parameter("~rms_calc_range_meters", rms_range_meters, 15.0f);
	n->get_parameter("~rms_calc_time_average_window", rms_time_average_window, 1.0f);
	n->get_parameter("~grid_res", grid_res, 1.0f);
	n->get_parameter("~grid_llx", grid_llx, -100.0f);
	n->get_parameter("~grid_lly", grid_lly, -100.0f);
	n->get_parameter("~warmup_time", warmup_time, 1.0f);
	n->get_parameter("~slope_threshold", thresh, 0.5f);
	n->get_parameter("~slope_threshold_max", thresh_max, 2.5f);
	n->get_parameter("~use_elevation", use_elevation, false);
	n->get_parameter("~grid_dilate", grid_dilate, true);
	n->get_parameter("~grid_dilate_x", grid_dilate_x, 1.0f);
	n->get_parameter("~grid_dilate_y", grid_dilate_y, 1.0f);
	n->get_parameter("~grid_dilate_proportion", grid_dilate_proportion, 0.8f);
	n->get_parameter("~perception_rate", perception_rate, 100.0);
	n->get_parameter("~max_grid_width", max_grid_width, 800.0f);
	n->get_parameter("~max_grid_height", max_grid_height, 800.0f);

	n->get_parameter("~grid_pub_method", grid_pub_method, avt_341::perception::GridPubMethod::Full);
	n->get_parameter("~grid_pub_force_full_every", grid_pub_force_full_every_x_sec, 10.0);
	n->get_parameter("~pc_callback_warn_time", pc_callback_runtime_threshold, 0.1);

	if (!avt_341::perception::GridPubMethod::IsGridPubMethodValid(grid_pub_method)){
		n->log_error("Invalid grid_pub_method: %hs", grid_pub_method.c_str());
		return -1;
	}

	n->get_parameter("~perception_points_topic", perception_points_topic, std::string("avt_341/points"));

	avt_341::perception::PointCloudFilterConfig pc_filter_config = ParseFilterConfig();
	avt_341::perception::PointCloudFilterConfig pc_cm_filter_config = ParseFilterConfig("clear_method_");
	avt_341::perception::ClearMethodRosParameters clear_methods_config = ParseClearMethodsConfig();

	// Configure grid
	// --------------------------------------------------------------------------------------------------------------

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

	grid.SetNode(n);
	grid.SetSize(grid_width, grid_height);
	grid.SetSlopeThreshold(thresh, thresh_max);
	grid.SetRes(grid_res);
	grid.SetCorner(grid_llx, grid_lly);
	grid.SetUseElevation(use_elevation);
	grid.SetDilation(grid_dilate, grid_dilate_x, grid_dilate_y, grid_dilate_proportion);
	grid.SetGridClearingMethod(clear_methods_config);
	grid.SetPointCloudFilterConfig(pc_filter_config, pc_cm_filter_config);

	// Create publishers + subscribers
	// --------------------------------------------------------------------------------------------------------------
	auto pc_sub = n->create_subscription<avt_341::msg::PointCloud2>(perception_points_topic, 10, PointCloudCallback);
	auto odom_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
	auto reset_sub = n->create_subscription<avt_341::msg::String>("avt_341/reset", 10, ResetCallback);
	auto grid_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/occupancy_grid", 1);
	auto reset_ack_pub = n->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);
	auto grid_segmentation_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/segmentation_grid", 1);

	auto rms_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/terrain_rms", 1);
	auto terrain_slope_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/terrain_slope", 1);

	const bool is_updates_grid_pub = grid_pub_method == avt_341::perception::GridPubMethod::Updates;
	auto grid_pub_updates = is_updates_grid_pub ? n->create_publisher<avt_341::msg::OccupancyGridUpdate>("avt_341/occupancy_grid_updates", 1) : nullptr;
	auto grid_segmentation_pub_updates = is_updates_grid_pub ? n->create_publisher<avt_341::msg::OccupancyGridUpdate>("avt_341/segmentation_grid_updates", 1) :  nullptr;

	// Main loop
	// --------------------------------------------------------------------------------------------------------------

	ResetNode();
	start_time = n->get_now_seconds();
	avt_341::node::Rate rate(perception_rate);
	int n_rms_avg = static_cast<int>(rms_time_average_window * perception_rate);
	std::deque<double> rms_buffer;
	std::deque<double> slope_buffer;
	int nloops = 0;
	while (avt_341::node::ok()) {

		const double now_seconds = n->get_now_seconds();
		const bool odom_rcvd = current_pose.header.stamp.sec > 0;

		if (odom_rcvd && (now_seconds - start_time) > warmup_time) {

			bool is_full_update = PublishGrid(false, grid_pub_method, now_seconds, grid_pub, grid_pub_updates);
			if (grid.HasSegmentation()) {
				PublishGrid(true, grid_pub_method, now_seconds, grid_segmentation_pub, grid_segmentation_pub_updates);
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

			if (clear_methods_config.visualize && nloops % 20 == 0) {
				grid.VisualizeClearMethods();
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
