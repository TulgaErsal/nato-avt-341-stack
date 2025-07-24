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
#include "avt_341/node/ros_types.h"
#include "avt_341/perception/elevation_grid_cell.h"
#include "avt_341/perception/costmap_clearing_method.h"

namespace avt_341 {
namespace perception {

class ElevationGrid : public CellObstacleCalculator {
public:
	ElevationGrid();

	~ElevationGrid() override;

	/**
		* Add points to be processed
		* \param point_cloud PointCloud message
		*/
	void AddPoints(avt_341::msg::PointCloud& point_cloud);

	/**
		* Clear points in point cloud
		* \param point_cloud PointCloud message
		*/
	void ClearPoints(avt_341::msg::PointCloud& point_cloud);

	bool has_segmentation() const { return has_segmentation_; }

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

	std::shared_ptr<OccupancyClearingMethod> CreateClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref,
		std::string clear_method_type,
		const RaytraceSettings& raytrace_settings,
		const TimedNoObsClearingSettings& timed_clear_settings,
		float visualization_range, bool visualize);

	void SetCostmapClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::string clear_methods_str,
		float visualization_range, bool visualize, float clear_method_raytrace_range, bool clear_method_clear_dilation,
		bool use_voxels, float voxel_height_min, float voxel_height_res, float obj_range_filter, int sampled_threshold);

	void Visualize() const {
		for (auto& cm : clear_methods_) {
			cm->Visualize();
		}
	}

	bool PastSlopeThreshold(const Cell& cell) const override;
	float Slope(const Cell& cell) const override;
	void AddOccupancy(const avt_341::msg::PointCloud& point_cloud, std::vector< std::vector<Cell> >& cells, bool dilate) override;

	void SetMaxPointAge(float mpa) {
		max_point_age_ = mpa;
	}

	void SetSlopeThreshold(float tr, float tr_max) {
		thresh_ = std::max(0.0f, tr);
		thresh_max_ = std::max(tr_max, tr);
		grid_slope_mult_ = static_cast<float>(GRID_MAX_VALUE) / (thresh_max_ - thresh_);
	}

	void SetStitchPoints(bool stitch_points) { stitch_points_ = stitch_points; }

	void SetFilterHighest(bool filter_high) { filter_highest_ = filter_high; }

	void SetUseElevation(bool use_elevation) {
		use_elevation_ = use_elevation;
	}

	void ClearGrid();

	void UseDilation(bool use_dil) {
		dilate_ = use_dil;
	}

	avt_341::msg::OccupancyGrid GetGrid(bool is_segmentation = false);

	avt_341::msg::OccupancyGrid GetGrid(double x, double y, double width, double height, bool is_segmentation = false);

	void SetCorner(float llx, float lly) {
		llx_ = llx;
		lly_ = lly;
	}

	void SetDilation(bool grid_dilate, float grid_dilate_x, float grid_dilate_y, float grid_dilate_proportion) {
		dilate_ = grid_dilate;
		grid_dilate_x_ = grid_dilate_x;
		grid_dilate_y_ = grid_dilate_y;
		grid_dilate_proportion_ = grid_dilate_proportion;
	}

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
	bool stitch_points_;
	bool filter_highest_;
	const uint8_t GRID_MAX_VALUE = 100;
	float grid_slope_mult_ = 50.0f;
	bool has_segmentation_ = false;
	float max_point_age_;
	bool is_resetting_ = false;
	std::vector<std::shared_ptr<OccupancyClearingMethod>> clear_methods_;

};

} // namespace perception
} // namespace avt_341