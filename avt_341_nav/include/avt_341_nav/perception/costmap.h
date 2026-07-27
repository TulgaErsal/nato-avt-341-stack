/**
 * Discretized grid costmap used for path planning optimization later in autonomy pipeline.
 * Aggregates costmap layers.
 */

#pragma once

#include <vector>
#include <string>

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "avt_341_nav/avt_341_utils.h"
#include "avt_341_nav/node/tf_interface.h"
#include "avt_341_nav/perception/costmap_dtos.h"
#include "avt_341_nav/perception/perception_settings.hpp"
#include "avt_341_nav/core/compute_time_recorder.hpp"
#include "avt_341_nav/core/grid_components.h"
#include "layers/costmap_layer.h"
#include <deque>
#include <rclcpp/rclcpp.hpp>

namespace avt_341_nav::perception
{

class Costmap {
public:
	Costmap(
		const rclcpp::Node::SharedPtr& node_ref,
		const std::shared_ptr<node::TfInterface>& tf,
		const PerceptionSettings& settings
	);

	bool HasSegmentation() const;

	void Clear() const;
	void ResetUpdateRegion();

	nav_msgs::msg::OccupancyGrid GetGrid(bool is_segmentation = false, const std::string& target_layer = "") const;
	nav_msgs::msg::OccupancyGrid GetGrid(double width, double height, bool is_segmentation = false) const;
	map_msgs::msg::OccupancyGridUpdate GetGridUpdate(bool is_segmentation, const std::string& target_layer = "") const;

	void FillGridMsgCells(std::vector<int8_t> & data, core::GridRegion region, bool is_segmentation, std::string target_layer = "") const;
	void Reset() const;
	void Visualize() const;
	void PublishComputeTimes() const;
	void UpdateThresholds(float slope_threshold, float slope_threshold_max);

	static bool IsPointInCone(const utils::vec2& test_point, const utils::vec2& p, const utils::vec2& v, float r, float angle);
	void UpdateRmsAndSlope();
	std::vector<utils::ivec2> GetCellsInFov() const;
	bool HasOdomData() const { return current_odom_.header.stamp.sec > 0; }
	double GetCurrentRms() const {
		if (rms_buffer_.empty()){
			return 0.0;
		}
		return std::accumulate(rms_buffer_.begin(), rms_buffer_.end(), 0.0)/static_cast<double>(rms_buffer_.size());
	}
	double GetCurrentSlope() const {
		if (slope_buffer_.empty()){
			return 0.0;
		}
		return std::accumulate(slope_buffer_.begin(), slope_buffer_.end(), 0.0) / static_cast<double>(slope_buffer_.size());
	}

	std::string ToLayerInfoString() const;

private:

    std::vector<std::shared_ptr<CostmapLayer>> GetTargetLayers(const std::string& target_layer, bool is_segmentation) const;

	void OdometryCallback(nav_msgs::msg::Odometry::SharedPtr rcv_odom);

	nav_msgs::msg::Odometry current_odom_;

	rclcpp::Node::SharedPtr node_ref_;

	std::shared_ptr<core::ComputeTimeRecorder> compute_time_recorder_;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

	PerceptionSettings settings_;

	std::vector<std::shared_ptr<CostmapLayer>> layers_;

	// Layer combination method, precached as booleans instead of string parameter inputs for efficiency
	std::string layer_cmb_method_;
	bool layer_cmb_last_ = false;
	bool layer_cmd_mn_ = false;


	template <typename T>
	inline void CollectLayerValues(
		const std::vector<std::shared_ptr<CostmapLayer>> & layers,
		std::vector<T>& layer_values,
		std::function<T(const std::shared_ptr<CostmapLayer>&)> value_getter
		) const
	{
		for (const auto& layer : layers) {
			if (const T val = value_getter(layer); val >= static_cast<T>(0)) {
				layer_values.emplace_back(val);
			}
		}
	}

	template<typename T>
	inline T CombineLayerValues(std::vector<T> layer_values, T unknown_value = T{0}) const
	{
		if (layer_values.empty()){
			return unknown_value;
		}

		return layer_cmb_last_ ? layer_values.back()
			       : (layer_cmd_mn_ ? std::accumulate(layer_values.begin(), layer_values.end(), T{0}) / static_cast<int>(layer_values.size())
				          : *std::max_element(layer_values.begin(), layer_values.end()));
	}

	template<typename T>
	inline T GetCombinedLayerValue(
		const std::vector<std::shared_ptr<CostmapLayer>> & layers,
		std::function<T(const std::shared_ptr<CostmapLayer>&)> value_getter,
		T unknown_value = T{0}
		) const
	{
		std::vector<T> layer_values;
		layer_values.reserve(layers.size());
		CollectLayerValues(layers, layer_values, value_getter);
		return CombineLayerValues(layer_values, unknown_value);
	}

	std::deque<double> rms_buffer_;
	std::deque<double> slope_buffer_;
};

}
