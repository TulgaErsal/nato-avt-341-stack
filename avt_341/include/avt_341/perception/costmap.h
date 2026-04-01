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

class Costmap : public CellObstacleCalculator {
public:
	Costmap(
		const std::shared_ptr<node::NodeProxy>& node_ref,
		const CostmapSizeInfo& size_info,
		const ThresholdSettings& thresholds,
		const DilationSettings& dilation
		);

	bool HasSegmentation() const;

	// bool PastSlopeThreshold(const Cell& cell) const override;
	// float Slope(const Cell& cell) const override;
	// void AddOccupancy(const avt_341::msg::PointCloud& point_cloud, std::vector< std::vector<Cell> >& cells, bool dilate) override;

	void UpdateSlopeParameters(optional<float> tr, optional<float> tr_max);

	void Clear() const;

	avt_341::msg::OccupancyGrid GetGrid(bool is_segmentation = false);

	avt_341::msg::OccupancyGridUpdate GetGridUpdate(bool is_segmentation);

	avt_341::msg::OccupancyGrid GetGrid(double x, double y, double width, double height, bool is_segmentation = false);

	void FillGridMsgCells(std::vector<int8_t> & data, core::GridRegion region, bool is_segmentation) const;
	void Reset() const;
	bool HasData() const;

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


private:

	// void DilateCell(
	// 	std::vector<std::vector<Cell>>& cells,
	// 	int xi,
	// 	int yi,
	// 	int dsize_x,
	// 	int dsize_y,
	// 	float original_slope = 0.0f);

	std::shared_ptr<node::NodeProxy> node_ref_;
	// PointCloudFilter pc_filter;						// Filter for input point clouds
	// PointCloudFilter pc_cm_filter;					// Additional filter for clearing methods applied after regular filter

	// std::vector<utils::ivec2> GetCellsInFov(float x, float y, float heading, float hfov, float range);
	// uint8_t GetGridCellValue(const Cell& cell) const;
	// void ResizeGrid();
	// std::vector< std::vector<Cell> > cells_;

	// bool has_segmentation_ = false;
	// bool is_resetting_ = false;

	CostmapSizeInfo size_info_;
	ThresholdSettings thresholds_;
	DilationSettings dilation_;
	std::vector<std::shared_ptr<CostmapLayer>> layers_;
	// std::vector<std::shared_ptr<OccupancyClearingMethod>> clear_methods_;
	// core::GridRegion grid_update_region_;
};

} // namespace perception
} // namespace avt_341