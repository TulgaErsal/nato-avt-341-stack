// c++ includes
// ros includes
#include <avt_341/avt_341_utils.h>
#include <avt_341/core/monitoring.hpp>

#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

// avt_341 includes
#include "avt_341/perception/costmap.h"

// avt_341::msg::Odometry current_pose;
double start_time = 0.0;
// double pc_callback_runtime_threshold = 0.0; // in seconds

float max_grid_width = 0.0f;
float max_grid_height = 0.0f;
double grid_pub_force_full_every_x_sec = 0.0;
double last_full_grid_update = 0.0;

std::shared_ptr<avt_341::node::NodeProxy> n = nullptr;

bool reset_called = false;
void ResetCallback(avt_341::msg::StringPtr msg) {
	if (msg->data.find(avt_341::node::NodeType::Perception) != std::string::npos) {
		reset_called = true;
	}
}

bool PublishGrid(bool is_segmentation, const std::string & grid_pub_method, double now_seconds,
	const avt_341::node::Publisher<avt_341::msg::OccupancyGrid>::SharedPtr& grid_pub,
	const avt_341::node::Publisher<avt_341::msg::OccupancyGridUpdate>::SharedPtr& grid_pub_updates,
	avt_341::perception::Costmap& grid) {

	avt_341::msg::OccupancyGrid grid_msg;
	bool is_full_update = false;
	if (grid_pub_method == avt_341::perception::GridPubMethod::Window) {
		grid_msg = grid.GetGrid(
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

avt_341::perception::CostmapSizeInfo ParseSizeInfo()
{
	avt_341::perception::CostmapSizeInfo size_info;
	n->get_parameter("/grid_width", size_info.width, 200.0f);
	n->get_parameter("/grid_height", size_info.height, 200.0f);
	n->get_parameter("~grid_res", size_info.res, 1.0f);
	n->get_parameter("~grid_llx", size_info.llx, -100.0f);
	n->get_parameter("~grid_lly", size_info.lly, -100.0f);
	return size_info;
}

avt_341::perception::DilationSettings ParseDilationSettings()
{
	avt_341::perception::DilationSettings settings;
	n->get_parameter("~grid_dilate", settings.enabled, true);
	n->get_parameter("~grid_dilate_x", settings.x, 1.0f);
	n->get_parameter("~grid_dilate_y", settings.y, 1.0f);
	n->get_parameter("~grid_dilate_proportion", settings.proportion, 0.8f);
	return settings;
}

avt_341::perception::TerrainRmsSettings ParseTerrainRmsSettings(float node_rate)
{
	avt_341::perception::TerrainRmsSettings settings;
	n->get_parameter("~rms_calc_horizontal_fov_radians", settings.hfov, 0.7854f); // about 45 degrees
	n->get_parameter("~rms_calc_range_meters", settings.range, 15.0f);
	n->get_parameter("~rms_calc_time_average_window", settings.time_window, 1.0f);
	settings.SetDiscreteRmsWindow(node_rate);
	return settings;
}

avt_341::perception::ThresholdSettings ParseThresholdSettings()
{
	avt_341::perception::ThresholdSettings settings;
	n->get_parameter("~use_elevation", settings.use_elevation, false);
	n->get_parameter("~slope_threshold", settings.thresh, 0.5f);
	n->get_parameter("~slope_threshold_max", settings.thresh_max, 2.5f);
	return settings;
}

int main(int argc, char* argv[]) {

	n = avt_341::node::init_node(argc, argv, "avt_341_perception_node");
	n->initialize_tf_listener();

	// Read parameters
	// --------------------------------------------------------------------------------------------------------------

	// float grid_width, grid_height;
	// float grid_res, grid_llx, grid_lly, warmup_time, thresh, thresh_max, grid_dilate_x, grid_dilate_y, grid_dilate_proportion;
	// bool use_elevation, grid_dilate;
	float warmup_time, perception_rate;
	std::string clear_method, grid_pub_method, layer_combination_method;

	// float rms_horizontal_fov_radians, rms_range_meters, rms_time_average_window;

	// n->get_parameter("~rms_calc_horizontal_fov_radians", rms_horizontal_fov_radians, 0.7854f); // about 45 degrees
	// n->get_parameter("~rms_calc_range_meters", rms_range_meters, 15.0f);
	// n->get_parameter("~rms_calc_time_average_window", rms_time_average_window, 1.0f);
	n->get_parameter("~warmup_time", warmup_time, 1.0f);
	// n->get_parameter("~slope_threshold", thresh, 0.5f);
	// n->get_parameter("~slope_threshold_max", thresh_max, 2.5f);
	// n->get_parameter("~use_elevation", use_elevation, false);

	n->get_parameter("~perception_rate", perception_rate, 100.0f);
	n->get_parameter("~max_grid_width", max_grid_width, 800.0f);
	n->get_parameter("~max_grid_height", max_grid_height, 800.0f);

	n->get_parameter("~grid_pub_method", grid_pub_method, std::string(avt_341::perception::GridPubMethod::Full));
	n->get_parameter("~grid_pub_force_full_every", grid_pub_force_full_every_x_sec, 10.0);
	n->get_parameter("~layer_combination_method", layer_combination_method, std::string("last"));

	if (!avt_341::perception::GridPubMethod::IsValid(grid_pub_method)){
		n->log_error("Invalid grid_pub_method: %hs", grid_pub_method.c_str());
		return -1;
	}

	const avt_341::perception::CostmapSizeInfo size_info = ParseSizeInfo();
	const avt_341::perception::DilationSettings dilation = ParseDilationSettings();
	const avt_341::perception::ThresholdSettings thresholds = ParseThresholdSettings();
	avt_341::perception::TerrainRmsSettings rms_settings = ParseTerrainRmsSettings(perception_rate);
	const avt_341::perception::CostmapSettings settings(size_info, thresholds, dilation, rms_settings);
	avt_341::perception::Costmap grid(n, settings, layer_combination_method);

	// Configure grid
	// --------------------------------------------------------------------------------------------------------------

	n->log_info("Perception node settings:\n"
					"	size_info: %hs\n"
					"	thresholds: %hs\n"
					"	dilation: %hs\n"
					"	grid_pub_method: %hs\n"
					"	n_layers: %d\n"
					"	layer_combine_method: %hs",
					size_info.ToString().c_str(),
					dilation.ToString().c_str(),
					thresholds.ToString().c_str(),
					grid_pub_method.c_str(),
					grid.GetLayerCount(),
					layer_combination_method.c_str()
					);


	// grid.SetGridClearingMethod(clear_methods_config);
	// grid.SetPointCloudFilterConfig(pc_filter_config, pc_cm_filter_config);

	// Create publishers + subscribers
	// --------------------------------------------------------------------------------------------------------------
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

	grid.Reset();
	start_time = n->get_now_seconds();
	avt_341::node::Rate rate(perception_rate);
	// int n_rms_avg = static_cast<int>(rms_time_average_window * perception_rate);
	// std::deque<double> rms_buffer;
	// std::deque<double> slope_buffer;
	int nloops = 0;
	while (avt_341::node::ok()) {

		const double now_seconds = n->get_now_seconds();

		if (grid.HasOdomData() && (now_seconds - start_time) > warmup_time) {

			bool is_full_update = PublishGrid(false, grid_pub_method, now_seconds, grid_pub, grid_pub_updates, grid);
			if (grid.HasSegmentation()) {
				PublishGrid(true, grid_pub_method, now_seconds, grid_segmentation_pub, grid_segmentation_pub_updates, grid);
			}
			if (is_full_update){
				last_full_grid_update = now_seconds;
			}

			// get the slope and RMS
			avt_341::msg::Float64 rms_msg, slope_msg;
			grid.UpdateRmsAndSlope();
			rms_msg.data = grid.GetCurrentRms();
			slope_msg.data = grid.GetCurrentSlope();
			rms_pub->publish(rms_msg);
			terrain_slope_pub->publish(slope_msg);

			if (nloops % 20 == 0) {
				grid.DebugVisualize(); // debug visualization
			}

			nloops++;

		}

		if (reset_called) {
			n->log_info("Resetting node");
			grid.Reset();
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
