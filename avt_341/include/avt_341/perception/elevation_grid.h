/**
 * \class ElevationGrid
 *
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
#include "avt_341/perception/elevation_grid_cell.h"
#include "avt_341/core/grid_components.h"
#include "avt_341/perception/clearing_methods/costmap_clearing_method.h"

namespace avt_341 {
namespace perception {

struct GridPubMethod {
	static const std::string Full;
	static const std::string Window;
	static const std::string Updates;
	static bool IsGridPubMethodValid(const std::string & selected_method);
};

class ElevationGrid : public CellObstacleCalculator {
public:
	ElevationGrid();

	~ElevationGrid() override;

	/**
	* Set point cloud filtering configuration
	* \param filter_pc_config Configuration for normal occupancy addition point cloud.
	* \param filter_pc_cm_config Configuration for additional point cloud filtering of costmap clearing methods.
	*/
	void SetPointCloudFilterConfig(
		const PointCloudFilterConfig& filter_pc_config,
		const PointCloudFilterConfig& filter_pc_cm_config);

	/**
		* Add points to be processed
		* \param point_cloud PointCloud message
		*/
	void ProcessPoints(const std::shared_ptr<msg::PointCloud>& pc_ptr, const msg::Pose& vehicle_pose, bool clear_only = false);

	/**
		* Clear points in point cloud
		* \param point_cloud PointCloud message
		*/
	void ClearPoints(const std::shared_ptr<msg::PointCloud>& pc_ptr, const msg::Pose& vehicle_pose);

	bool HasSegmentation() const { return has_segmentation_; }

	void SetNode(const std::shared_ptr<node::NodeProxy>& node_ref);

	void SetSize(float s) {
		width_ = s;
		height_ = s;
		ResizeGrid();
	}

	void SetSize(float width, float height) {
		width_ = width;
		height_ = height;
		ResizeGrid();
	}

	void SetRes(float r) {
		res_ = r;
		ResizeGrid();
	}

	void SetGridClearingMethod(const ClearMethodRosParameters & params);

	void VisualizeClearMethods() const {
		for (auto& cm : clear_methods_) {
			cm->Visualize();
		}
	}

	bool PastSlopeThreshold(const Cell& cell) const override;
	float Slope(const Cell& cell) const override;
	void AddOccupancy(const avt_341::msg::PointCloud& point_cloud, std::vector< std::vector<Cell> >& cells, bool dilate) override;

	void SetSlopeParameters(optional<float> tr, optional<float> tr_max, bool recompute_grid = false);

	void RecomputeGridDilation();

	void SetUseElevation(bool use_elevation) {
		use_elevation_ = use_elevation;
	}

	void ClearGrid();

	void UseDilation(bool use_dil) {
		dilate_ = use_dil;
	}

	avt_341::msg::OccupancyGrid GetGrid(bool is_segmentation = false);

	avt_341::msg::OccupancyGridUpdate GetGridUpdate(bool is_segmentation);

	avt_341::msg::OccupancyGrid GetGrid(double x, double y, double width, double height, bool is_segmentation = false);

	void SetCorner(float llx, float lly) {
		llx_ = llx;
		lly_ = lly;
	}

	inline int GetDilateXSize() const { return dilate_ ? lround(grid_dilate_x_/res_) : 0;}
	inline int GetDilateYSize() const { return dilate_ ? lround(grid_dilate_y_/res_) : 0;}

	void SetDilation(bool grid_dilate, float grid_dilate_x, float grid_dilate_y, float grid_dilate_proportion) {
		dilate_ = grid_dilate;
		grid_dilate_x_ = grid_dilate_x;
		grid_dilate_y_ = grid_dilate_y;
		grid_dilate_proportion_ = grid_dilate_proportion;
	}

	void FillGridMsgCells(std::vector<int8_t> & data, core::GridRegion region, bool is_segmentation) const;
	void ResetUpdateRegion(){ grid_update_region_.Reset();}
	void Reset();
	bool HasData() const;

	/// x and y in local ENU meters
	float GetRmsAtCoordinate(float x, float y);
	/// xi and yi as grid cell indices
	float GetRmsAtCell(int xi, int yi);

	/// x and y in local ENU meters
	float GetTerrainSlopeAtCoordinate(float x, float y);
	/// xi and yi as grid cell indices
	float GetTerrainSlopeAtCell(int xi, int yi);

	void GetSlopeRmsInFov(float& slope, float& rms, float x, float y, float heading, float hfov, float range);

	utils::vec2 CellToPoint(int i, int j) {
		float x = (i + 0.5f) * res_ + llx_;
		float y = (j + 0.5f) * res_ + lly_;
		utils::vec2 p(x, y);
		return p;
	}

	utils::ivec2 PointToCell(float x, float y) {
		int xi = (int)floor((x - llx_) / res_);
		int yi = (int)floor((y - lly_) / res_);
		utils::ivec2 c(xi, yi);
		return c;
	}


private:

	void DilateCell(
		std::vector<std::vector<Cell>>& cells,
		int xi,
		int yi,
		int dsize_x,
		int dsize_y,
		float original_slope = 0.0f);

	std::shared_ptr<node::NodeProxy> node_ref_;
	PointCloudFilter pc_filter;						// Filter for input point clouds
	PointCloudFilter pc_cm_filter;					// Additional filter for clearing methods applied after regular filter

	std::vector<utils::ivec2> GetCellsInFov(float x, float y, float heading, float hfov, float range);
	uint8_t GetGridCellValue(const Cell& cell) const;
	void ResizeGrid();
	std::vector< std::vector<Cell> > cells_;
	float width_;
	float height_;
	float res_;
	float thresh_;
	float thresh_max_;
	int nx_, ny_;
	bool first_display_;
	bool dilate_;
	float llx_;
	float lly_;
	float grid_dilate_x_;
	float grid_dilate_y_;
	float grid_dilate_proportion_;
	bool use_elevation_;
	const uint8_t GRID_MAX_VALUE = 100;
	float grid_slope_mult_ = 50.0f;
	bool has_segmentation_ = false;
	bool is_resetting_ = false;
	std::vector<std::shared_ptr<OccupancyClearingMethod>> clear_methods_;
	core::GridRegion grid_update_region_;
};

} // namespace perception
} // namespace avt_341