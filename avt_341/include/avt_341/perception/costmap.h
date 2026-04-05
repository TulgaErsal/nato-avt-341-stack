/**
 * A slope-based obstacle detection algorithm.
 * The world is divided into 2D cells.
 * The highest and lowest point are used to calculate the slope in each cell.
 * Cells that exceed a slope threshold are flagged as obstcles.
 *
 * \author Chris Goodin
 *
 * \date 9/3/2020
 */

#pragma once

#include <vector>
#include <limits>
#include <string>

#include "point_cloud_filter.hpp"
#include "avt_341/node/ros_types.h"
#include "avt_341/avt_341_utils.h"
#include "avt_341/perception/costmap_dtos.h"
#include "avt_341/core/grid_components.h"
#include "avt_341/perception/clearing_methods/costmap_clearing_method.h"
#include "layers/costmap_layer.h"

namespace avt_341 {
namespace perception {

class Costmap {
public:
	Costmap(
		const std::shared_ptr<node::NodeProxy>& node_ref,
		const CostmapSizeInfo& size_info,
		const ThresholdSettings& thresholds,
		const DilationSettings& dilation,
		const TerrainRmsSettings& terrain_rms_config
		);

	bool HasSegmentation() const;

	// bool PastSlopeThreshold(const Cell& cell) const override;
	// float Slope(const Cell& cell) const override;
	// void AddOccupancy(const avt_341::msg::PointCloud& point_cloud, std::vector< std::vector<Cell> >& cells, bool dilate) override;

	void Clear() const;

	msg::OccupancyGrid GetGrid(bool is_segmentation = false);
	msg::OccupancyGrid GetGrid(double width, double height, bool is_segmentation = false) const;
	msg::OccupancyGridUpdate GetGridUpdate(bool is_segmentation);

	void FillGridMsgCells(std::vector<int8_t> & data, core::GridRegion region, bool is_segmentation) const;
	void Reset() const;
	void DebugVisualize() const;

	// /// x and y in local ENU meters
	// float GetRmsAtCoordinate(float x, float y);
	// /// xi and yi as grid cell indices
	// float GetRmsAtCell(int xi, int yi);
	//
	// /// x and y in local ENU meters
	// float GetTerrainSlopeAtCoordinate(float x, float y);
	// /// xi and yi as grid cell indices
	// float GetTerrainSlopeAtCell(int xi, int yi);
	//
	// void GetSlopeRmsInFov(float& slope, float& rms, float x, float y, float heading, float hfov, float range);

	static bool IsPointInCone(const utils::vec2& test_point, const utils::vec2& p, const utils::vec2& v, float r, float angle);
	void UpdateRmsAndSlope();
	std::vector<utils::ivec2> GetCellsInFov() const;
	bool HasOdomData() const { return current_odom_.header.stamp.sec > 0; }
	double GetCurrentRms() const {
		return std::accumulate(rms_buffer_.begin(), rms_buffer_.end(), 0.0)/static_cast<double>(rms_buffer_.size());
	}
	double GetCurrentSlope() const {
		return std::accumulate(slope_buffer_.begin(), slope_buffer_.end(), 0.0) / static_cast<double>(slope_buffer_.size());
	}

private:

	void OdometryCallback(msg::OdometryPtr rcv_odom);

	// void DilateCell(
	// 	std::vector<std::vector<Cell>>& cells,
	// 	int xi,
	// 	int yi,
	// 	int dsize_x,
	// 	int dsize_y,
	// 	float original_slope = 0.0f);
	msg::Odometry current_odom_;

	std::shared_ptr<node::NodeProxy> node_ref_;
	// PointCloudFilter pc_filter;						// Filter for input point clouds
	// PointCloudFilter pc_cm_filter;					// Additional filter for clearing methods applied after regular filter

	// std::vector<utils::ivec2> GetCellsInFov(float x, float y, float heading, float hfov, float range);
	// uint8_t GetGridCellValue(const Cell& cell) const;
	// void ResizeGrid();
	// std::vector< std::vector<Cell> > cells_;

	// bool has_segmentation_ = false;
	// bool is_resetting_ = false;

    node::Subscriber<msg::Odometry>::SharedPtr odom_sub_;

	CostmapSizeInfo size_info_;
	ThresholdSettings thresholds_;
	DilationSettings dilation_;
	TerrainRmsSettings terrain_rms_config_;

	std::vector<std::shared_ptr<CostmapLayer>> layers_;

	std::deque<double> rms_buffer_;
	std::deque<double> slope_buffer_;
	// std::vector<std::shared_ptr<OccupancyClearingMethod>> clear_methods_;
	// core::GridRegion grid_update_region_;
};

} // namespace perception
} // namespace avt_341