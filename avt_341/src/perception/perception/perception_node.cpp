#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/avt_341_utils.h"
#include "avt_341/perception/costmap.h"

double start_time = 0.0;

float max_grid_width = 0.0f;
float max_grid_height = 0.0f;
double grid_pub_force_full_every_x_sec = 0.0;
std::map<std::string, double> last_full_grid_update;

std::map<std::string, avt_341::node::Publisher<avt_341::msg::OccupancyGrid>::SharedPtr> occ_publisher;
std::map<std::string, avt_341::node::Publisher<avt_341::msg::OccupancyGrid>::SharedPtr> seg_publisher;
std::map<std::string, avt_341::node::Publisher<avt_341::msg::OccupancyGridUpdate>::SharedPtr> occ_updates_publisher;
std::map<std::string, avt_341::node::Publisher<avt_341::msg::OccupancyGridUpdate>::SharedPtr> seg_updates_publisher;

std::shared_ptr<avt_341::node::NodeProxy> n = nullptr;

bool reset_called = false;
void ResetCallback(avt_341::msg::StringPtr msg) {
	if (msg->data.find(avt_341::node::NodeType::Perception) != std::string::npos) {
		reset_called = true;
	}
}

void PublishGrid(
	bool is_segmentation,
	const std::string & grid_pub_method,
	double now_seconds,
	const std::string & target_layer,
	avt_341::perception::Costmap& grid
	) {

	avt_341::msg::OccupancyGrid grid_msg;

	if (grid_pub_method == avt_341::perception::GridPubMethod::Window) {
		grid_msg = grid.GetGrid(
			max_grid_width,
			max_grid_height,
			is_segmentation
			);
	}else {
		bool is_full_update = (grid_pub_method == avt_341::perception::GridPubMethod::Full)
			|| (now_seconds - last_full_grid_update[target_layer] > grid_pub_force_full_every_x_sec);
		if (is_full_update) {
			last_full_grid_update[target_layer] = now_seconds;
			grid_msg = grid.GetGrid(is_segmentation, target_layer);
		}else {
			avt_341::msg::OccupancyGridUpdate grid_update_msg;
			grid_update_msg = grid.GetGridUpdate(is_segmentation, target_layer);
			if (grid_update_msg.height > 0 && grid_update_msg.width > 0) {
				grid_update_msg.header.stamp = n->get_stamp();
				auto grid_pub_updates = is_segmentation ? seg_updates_publisher[target_layer] : occ_updates_publisher[target_layer];
				grid_pub_updates->publish(grid_update_msg);
			}
			return;
		}
	}

	auto grid_pub = is_segmentation ? seg_publisher[target_layer] : occ_publisher[target_layer];
	grid_msg.header.stamp = n->get_stamp();
	grid_pub->publish(grid_msg);
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
	float warmup_time, perception_rate;
	std::string clear_method, grid_pub_method, layer_combination_method;
	std::string publish_layers_param;

	n->get_parameter("~warmup_time", warmup_time, 1.0f);
	n->get_parameter("~perception_rate", perception_rate, 100.0f);
	n->get_parameter("~max_grid_width", max_grid_width, 800.0f);
	n->get_parameter("~max_grid_height", max_grid_height, 800.0f);

	n->get_parameter("~grid_pub_method", grid_pub_method, std::string(avt_341::perception::GridPubMethod::Full));
	n->get_parameter("~grid_pub_force_full_every", grid_pub_force_full_every_x_sec, 10.0);
	n->get_parameter("~layer_combination_method", layer_combination_method, std::string("last"));

	// Layers to publish individually in addition to combined costmap layers. Assumed to be comma list in single string
	n->get_parameter("~publish_layers", publish_layers_param, std::string());
	std::vector<std::string> publish_layers = avt_341::utils::SplitByDelimiter(publish_layers_param, ',');

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
					"	layers: %hs",
					size_info.ToString().c_str(),
					thresholds.ToString().c_str(),
					dilation.ToString().c_str(),
					grid_pub_method.c_str(),
					grid.ToLayerInfoString().c_str()
					);

	// Create publishers + subscribers
	// --------------------------------------------------------------------------------------------------------------
	auto reset_sub = n->create_subscription<avt_341::msg::String>("avt_341/reset", 10, ResetCallback);
	auto reset_ack_pub = n->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);
	auto rms_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/terrain_rms", 1);
	auto terrain_slope_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/terrain_slope", 1);

	const bool use_inc_updates = grid_pub_method == avt_341::perception::GridPubMethod::Updates;

	publish_layers.push_back(""); // add empty string to represent combined grid layer for publishing

	for (const auto& layer_id : publish_layers){
		const std::string occ_pub_name = layer_id.empty() ? "avt_341/occupancy_grid" : "avt_341/occ_" + layer_id;
		const std::string seg_pub_name = layer_id.empty() ? "avt_341/segmentation_grid" : "avt_341/seg_" + layer_id;
		occ_publisher[layer_id] = n->create_publisher<avt_341::msg::OccupancyGrid>(occ_pub_name, 1);
		seg_publisher[layer_id] = n->create_publisher<avt_341::msg::OccupancyGrid>(seg_pub_name, 1);
		if (use_inc_updates)
		{
			occ_updates_publisher[layer_id] = n->create_publisher<avt_341::msg::OccupancyGridUpdate>(occ_pub_name + "_updates", 1);
			seg_updates_publisher[layer_id] = n->create_publisher<avt_341::msg::OccupancyGridUpdate>(seg_pub_name + "_updates", 1);
		}
	}

	// Main loop
	// --------------------------------------------------------------------------------------------------------------

	grid.Reset();
	start_time = n->get_now_seconds();
	avt_341::node::Rate rate(perception_rate);
	int nloops = 0;

	while (avt_341::node::ok()) {

		const double now_seconds = n->get_now_seconds();

		if (grid.HasOdomData() && (now_seconds - start_time) > warmup_time) {

			for (const auto& pub_layer: publish_layers){
				PublishGrid(false, grid_pub_method, now_seconds, pub_layer, grid);
				if (grid.HasSegmentation()) {
					PublishGrid(true, grid_pub_method, now_seconds, pub_layer, grid);
				}
			}

			// get the slope and RMS
			avt_341::msg::Float64 rms_msg, slope_msg;
			grid.UpdateRmsAndSlope();
			rms_msg.data = grid.GetCurrentRms();
			slope_msg.data = grid.GetCurrentSlope();
			rms_pub->publish(rms_msg);
			terrain_slope_pub->publish(slope_msg);

			if (nloops % 20 == 0) {
				grid.Visualize(); // debug visualization
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
