#include <map>
#include <string>
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/avt_341_utils.h"
#include "avt_341/perception/costmap.h"
#include "avt_341/perception/perception_settings.hpp"
#include <avt_341/perception_params_service.hpp>

double start_time = 0.0;

std::map<std::string, double> occ_last_full_grid_update;
std::map<std::string, double> seg_last_full_grid_update;

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
	const avt_341::perception::PerceptionSettings& settings,
	double now_seconds,
	const std::string & target_layer,
	avt_341::perception::Costmap& grid
	) {

	avt_341::msg::OccupancyGrid grid_msg;

	if (settings.costmap.publish.method ==
		avt_341::perception::GridPubMethod::Window) {
		grid_msg = grid.GetGrid(
			settings.costmap.publish.max_grid_width,
			settings.costmap.publish.max_grid_height,
			is_segmentation
			);
	}else {

	    std::map<std::string, double> & last_full_grid_update = is_segmentation ? seg_last_full_grid_update : occ_last_full_grid_update;
	    bool is_full_update;

	    // In incremental update mode, grid_pub_force_full_every_x_sec <= 0.0 disables
	    // periodic full grid updates, except for initial publication
	    if (last_full_grid_update.find(target_layer) == last_full_grid_update.end()) {
	        last_full_grid_update[target_layer] = 0.0;
	        is_full_update = true;
	    }
	    else
	    {
	        is_full_update =
				(settings.costmap.publish.method ==
					avt_341::perception::GridPubMethod::Full)
                        || (settings.costmap.publish.force_full_every > 0.0 &&
                            (now_seconds - last_full_grid_update[target_layer] >
								settings.costmap.publish.force_full_every)
                            );
	    }

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

int main(int argc, char* argv[]) {

	n = avt_341::node::init_node(argc, argv, "avt_341_perception_node");
	n->initialize_tf_listener();
	avt_341::params::perception::ParamsListener param_listener(
		n->get_raw_node());
	avt_341::perception::PerceptionSettings settings(
		param_listener.get_params());

	// Layers to publish individually in addition to combined costmap layers. Assumed to be comma list in single string
	std::vector<std::string> publish_layers =
		avt_341::utils::SplitByDelimiter(settings.costmap.publish.layers, ',');

	if (!avt_341::perception::GridPubMethod::IsValid(
			settings.costmap.publish.method)){
		n->log_error("Invalid costmap.publish.method: %hs",
			settings.costmap.publish.method.c_str());
		return -1;
	}

	avt_341::perception::Costmap grid(n, settings);
	param_listener.setUserCallback(
		[&grid](const avt_341::params::perception::Params& updated_params) {
			grid.UpdateThresholds(
				updated_params.costmap.thresholds.thresh,
				updated_params.costmap.thresholds.thresh_max);
		});

	// Configure grid
	// --------------------------------------------------------------------------------------------------------------

	n->log_info("Perception node settings:\n"
					"	size_info: %hs\n"
					"	thresholds: %hs\n"
					"	dilation: %hs\n"
					"	publish method: %hs\n"
					"	layers: %hs",
					settings.size_info_string().c_str(),
					settings.thresholds_string().c_str(),
					settings.dilation_string().c_str(),
					settings.costmap.publish.method.c_str(),
					grid.ToLayerInfoString().c_str()
					);

	// Create publishers + subscribers
	// --------------------------------------------------------------------------------------------------------------
	auto reset_sub = n->create_subscription<avt_341::msg::String>("avt_341/reset", 10, ResetCallback);
	auto reset_ack_pub = n->create_publisher<avt_341::msg::String>("avt_341/reset_ack", 1);
	auto rms_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/terrain_rms", 1);
	auto terrain_slope_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/terrain_slope", 1);

	const bool use_inc_updates =
		settings.costmap.publish.method ==
		avt_341::perception::GridPubMethod::Updates;

	publish_layers.push_back(""); // add empty string to represent combined grid layer for publishing

	for (const auto& layer_id : publish_layers){
		const std::string occ_pub_name = layer_id.empty() ? "avt_341/occupancy_grid" : "avt_341/occ_" + layer_id;
		const std::string seg_pub_name = layer_id.empty() ? "avt_341/segmentation_grid" : "avt_341/seg_" + layer_id;

		if (use_inc_updates)
		{
		    occ_publisher[layer_id] = n->create_latching_publisher<avt_341::msg::OccupancyGrid>(occ_pub_name);
		    seg_publisher[layer_id] = n->create_latching_publisher<avt_341::msg::OccupancyGrid>(seg_pub_name);
			occ_updates_publisher[layer_id] = n->create_publisher<avt_341::msg::OccupancyGridUpdate>(occ_pub_name + "_updates", 1);
			seg_updates_publisher[layer_id] = n->create_publisher<avt_341::msg::OccupancyGridUpdate>(seg_pub_name + "_updates", 1);
		}
	    else
		{
		    occ_publisher[layer_id] = n->create_publisher<avt_341::msg::OccupancyGrid>(occ_pub_name, 1);
		    seg_publisher[layer_id] = n->create_publisher<avt_341::msg::OccupancyGrid>(seg_pub_name, 1);
		}
	}

	// Main loop
	// --------------------------------------------------------------------------------------------------------------

	grid.Reset();
	start_time = n->get_now_seconds();
	avt_341::node::Rate rate(settings.runtime.rate);
	int nloops = 0;
	double last_compute_time_pub = 0.0;

	while (avt_341::node::ok()) {

		const double now_seconds = n->get_now_seconds();

		if (settings.runtime.compute_time_publish_period > 0.0 &&
			now_seconds - last_compute_time_pub >=
				settings.runtime.compute_time_publish_period) {
			last_compute_time_pub = now_seconds;
			grid.PublishComputeTimes();
		}

		if (grid.HasOdomData() &&
			(now_seconds - start_time) > settings.runtime.warmup_time) {

			for (const auto& pub_layer: publish_layers){
				PublishGrid(false, settings, now_seconds, pub_layer, grid);
				if (grid.HasSegmentation()) {
					PublishGrid(true, settings, now_seconds, pub_layer, grid);
				}
			}
			grid.ResetUpdateRegion();

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
			occ_last_full_grid_update.clear();
			seg_last_full_grid_update.clear();
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
