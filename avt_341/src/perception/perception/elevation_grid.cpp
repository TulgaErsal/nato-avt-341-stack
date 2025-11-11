#include "avt_341/perception/elevation_grid.h"
#include <iostream>
#include <thread>
#include <math.h>
#include <avt_341/perception/clearing_methods/clearing_methods_factory.h>
#include <opencv2/imgproc.hpp>

namespace avt_341 {
namespace perception {

ElevationGrid::ElevationGrid() {
	width_ = 200.0f;
	height_ = 200.0f;
	llx_ = -100.0f;
	lly_ = -100.0f;
	res_ = 0.5f;
	ResizeGrid();
	thresh_ = 0.5f;
	thresh_max_ = 2.5f;
	dilate_ = false;
	grid_dilate_x_ = 2.0f;
	grid_dilate_y_ = 2.0f;
	grid_dilate_proportion_ = 0.8f;
	use_elevation_ = false;
	grid_update_region_.Reset();
}

ElevationGrid::~ElevationGrid() {

}

void ElevationGrid::ResizeGrid() {
	nx_ = (int)ceil(width_ / res_);
	ny_ = (int)ceil(height_ / res_);
	//if (n_%2!=0) n_ = n_+1;
	Cell cell;
	cells_.clear();
	std::vector<Cell> row;
	row.resize(nx_, cell);
	cells_.resize(ny_, row);
}

void ElevationGrid::ClearGrid() {
	Cell empty_cell;
	for (int i = 0; i < (ny_); i++) {
		for (int j = 0; j < (nx_); j++) {
			cells_[i][j] = empty_cell;
		}
	}
}

static bool IsPointInCone(const utils::vec2& test_point, const utils::vec2& p, const utils::vec2& v, float r, float angle) {
	utils::vec2 dir_to_point = test_point - p;
	float dist = utils::length(dir_to_point);

	if (dist > r) return false;

	dir_to_point.normalize();
	float dot_product = utils::dot(v, dir_to_point);

	float cos_angle = cosf(angle);
	return dot_product >= cos_angle;
}

// CTG, 7/23/25
std::vector<utils::ivec2> ElevationGrid::GetCellsInFov(float x, float y, float heading, float hfov, float range) {
	utils::vec2 p(x, y);
	float angle = 0.5f * hfov;
	utils::vec2 v(cosf(heading), sinf(heading));
	std::vector<utils::ivec2> cells_in_fov;
	int xi0 = std::max(0,(int)floor((x - range) / res_));
	int xi1 = std::min(nx_,(int)floor((x + range) / res_));
	int yi0 = std::max(0,(int)floor((y - range) / res_));
	int yi1 = std::min(ny_,(int)floor((y + range) / res_));
	for (int j = yi0; j < yi1; j++) {
		for (int i = xi0; i < xi1; i++) {
			utils::vec2 point = CellToPoint(i, j);
			bool in_view = IsPointInCone(point, p, v, range, angle);
			if (in_view) {
				utils::ivec2 c(i, j);
				cells_in_fov.push_back(c);
			}
		}
	}
	return cells_in_fov;
}

// CTG, 7/23/25
void ElevationGrid::GetSlopeRmsInFov(float& slope, float& rms, float x, float y, float heading, float hfov, float range) {

	std::vector<utils::ivec2> cells = GetCellsInFov(x, y, heading, hfov, range);
	slope = 0.0f;
	rms = 0.0f;

	if (cells.size() < 0)return;

	float navg = (float)cells.size();
	for (int i = 0; i < (int)cells.size(); i++) {
		rms += GetRmsAtCell(cells[i].x, cells[i].y);
		slope += GetTerrainSlopeAtCell(cells[i].x, cells[i].y);

	}
	rms /= navg;
	slope /= navg;
	return;
}

// CTG, 5/8/25
float ElevationGrid::GetRmsAtCoordinate(float x, float y) {
	int xi = (int)floor((x - llx_) / res_);
	int yi = (int)floor((y - lly_) / res_);
	float rms = GetRmsAtCell(xi, yi);
	return rms;
}

// CTG, 5/8/25
float ElevationGrid::GetRmsAtCell(int xi, int yi) {
	float rms = 0.0f;
	if (xi >= 0 && xi < nx_ && yi >= 0 && yi < ny_) {
		rms = cells_[yi][xi].rms;
	}
	return rms;
}

// CTG, 5/8/25
float ElevationGrid::GetTerrainSlopeAtCoordinate(float x, float y) {
	int xi = (int)floor((x - llx_) / res_);
	int yi = (int)floor((y - lly_) / res_);
	float slope = GetTerrainSlopeAtCell(xi, yi);
	return slope;
}


// CTG, 5/8/25
float ElevationGrid::GetTerrainSlopeAtCell(int xi, int yi) {
	float slope = 0.0f;
	if (xi >= 0 && xi < nx_ && yi >= 0 && yi < ny_) {
		int xl = std::max(xi - 1, 0);
		int xh = std::min(xi + 1, nx_ - 1);
		int yl = std::max(yi - 1, 0);
		int yh = std::min(yi + 1, ny_ - 1);
		float dx = (xh - xl) * res_;
		float dy = (yh - yl) * res_;
		float dz_dx = 0.0f;
		if (dx != 0.0f && cells_[yi][xh].num_points > 0 && cells_[yi][xl].num_points > 0) dz_dx = (cells_[yi][xh].low.val - cells_[yi][xl].low.val) / dx;
		float dz_dy = 0.0f;
		if (dy != 0.0f && cells_[yh][xi].num_points > 0 && cells_[yl][xi].num_points > 0) dz_dy = (cells_[yh][xi].low.val - cells_[yl][xi].low.val) / dy;
		slope = sqrtf(dz_dx * dz_dx + dz_dy * dz_dy);
	}
	return slope;
}

void ElevationGrid::AddOccupancy(const avt_341::msg::PointCloud& point_cloud, std::vector< std::vector<Cell> >& cells, bool dilate) {

	bool has_segmentation_local = !point_cloud.channels.empty() && point_cloud.channels[0].name == "segmentation";
	has_segmentation_ = has_segmentation_local || has_segmentation_;

	const int dsize_x = GetDilateXSize();
	const int dsize_y = GetDilateYSize();

	// fill the cells with highest and lowest points
	for (int i = 0; i < point_cloud.points.size(); i++) {
		if (!(point_cloud.points[i].x == 0.0 && point_cloud.points[i].y == 0.0)) {
			int xi = (int)floor((point_cloud.points[i].x - llx_) / res_);
			int yi = (int)floor((point_cloud.points[i].y - lly_) / res_);
			if (xi >= 0 && xi < nx_ && yi >= 0 && yi < ny_) {
				grid_update_region_.UpdateBounds(xi,yi);
				const float original_slope = Slope(cells[yi][xi]);
				float h = point_cloud.points[i].z;

				if (h > cells[yi][xi].high.val) {
					cells[yi][xi].high.val = h;
					cells[yi][xi].high.age = 0.0f;
				}
				if (h < cells[yi][xi].low.val) {
					cells[yi][xi].low.val = h;
					cells[yi][xi].low.age = 0.0f;
				}
				if (has_segmentation_local) {
					float terr_val = point_cloud.channels[0].values[i];
					cells[yi][xi].terrain = fmax(cells[yi][xi].terrain, terr_val);
				}

				// CTG 5/8/25, add calculations necessary for tracking RMS
				cells_[yi][xi].summed_elev += h;
				cells_[yi][xi].num_points += 1;
				if (cells_[yi][xi].num_points > 0) {
					cells_[yi][xi].avg_elev = cells_[yi][xi].summed_elev / cells_[yi][xi].num_points;
					float dh = h - cells_[yi][xi].avg_elev;
					cells_[yi][xi].sum_of_squares += dh * dh;
					cells_[yi][xi].rms = sqrtf(cells_[yi][xi].sum_of_squares / cells_[yi][xi].num_points);
				}
				else {
					cells_[yi][xi].avg_elev = 0.0f;
					cells_[yi][xi].rms = 0.0f;
				}

				// Optional dilation
				if (dilate) {
					if ((!cells[yi][xi].has_dilated || Slope(cells[yi][xi]) > original_slope) && PastSlopeThreshold(cells[yi][xi])) {
						cells[yi][xi].has_dilated = true;
						uint8_t grid_val = static_cast<uint8_t>(grid_dilate_proportion_ * static_cast<float>(GetGridCellValue(cells[yi][xi])));
						for (int xii = std::max(0, xi - dsize_x); xii <= std::min(xi + dsize_x, nx_ - 1); xii++) {
							for (int yii = std::max(0, yi - dsize_y); yii <= std::min(yi + dsize_y, ny_ - 1); yii++) {
								cells[yii][xii].dilated_val = std::max(grid_val, cells[yii][xii].dilated_val);
							}
						}
					}
				}
			}
		}
	}
}

void ElevationGrid::SetPointCloudFilterConfig(
		const PointCloudFilterConfig& filter_pc_config,
		const PointCloudFilterConfig& filter_pc_cm_config) {

	pc_filter.SetConfig(filter_pc_config);
	pc_cm_filter.SetConfig(filter_pc_cm_config);

	node_ref_->log_info(
		"Point cloud culling: %s",
		pc_filter.GetDescription().c_str()
		);

	node_ref_->log_info(
		"Point cloud culling (extra for grid clearing): %s",
		pc_cm_filter.GetDescription().c_str()
		);
}

void ElevationGrid::AddPoints(const std::shared_ptr<msg::PointCloud>& pc_ptr, const msg::Pose& vehicle_pose) {

	if (is_resetting_) {
		return;
	}

	// Filtered point cloud for normal occupancy addition
	auto filtered_pc = pc_filter.Filter(pc_ptr, vehicle_pose);

	// Additional filtering for clearing methods if desired
	auto filtered_cms_pc = clear_methods_.empty() ? filtered_pc : pc_cm_filter.Filter(filtered_pc, vehicle_pose);

	for (auto& cm : clear_methods_) {
		cm->ClearOccupancy(*filtered_cms_pc);
	}
	AddOccupancy(*filtered_pc, cells_, dilate_);
	for (auto& cm : clear_methods_) {
		cm->OnOccupancyAdded(*filtered_cms_pc, vehicle_pose.position);
	}

}

void ElevationGrid::ClearPoints(avt_341::msg::PointCloud& point_cloud) {
	for (auto& cm : clear_methods_) {
		cm->ClearOccupancy(point_cloud);
	}
}

uint8_t ElevationGrid::GetGridCellValue(const Cell& cell) const {
	if (!cell.filled())
		return 0;

	if (use_elevation_) {
		return cell.high.val > thresh_ ? GRID_MAX_VALUE : 0;
	}
	else {
		const auto slope = cell.height() / res_;
		return slope > thresh_ ? static_cast<uint8_t>(std::min(std::max(0.0f, grid_slope_mult_ * slope), static_cast<float>(GRID_MAX_VALUE))) : 0;
	}

}

void ElevationGrid::FillGridMsgCells(std::vector<int8_t> & data, const GridRegion region, bool is_segmentation) const {
	data.resize(region.Width()*region.Height());
	int c = 0;
	for (int j = region.y_min; j < region.y_max; j++) {
		for (int i = region.x_min; i < region.x_max; i++) {
			data[c++] = is_segmentation ? (uint8_t)(cells_[j][i].terrain) : std::max(GetGridCellValue(cells_[j][i]), cells_[j][i].dilated_val);
		}
	}
}

avt_341::msg::OccupancyGridUpdate ElevationGrid::GetGridUpdate(bool is_segmentation) {
	avt_341::msg::OccupancyGridUpdate grid_update_msg;
	grid_update_msg.header.frame_id = "map";
	if (!grid_update_region_.HasData()) {
		return grid_update_msg;
	}
	int dilate_x = GetDilateXSize();
	int dilate_y = GetDilateYSize();
	GridRegion dilated_region = grid_update_region_.Dilate(dilate_x, dilate_y, nx_, ny_);
	grid_update_msg.x = dilated_region.x_min;
	grid_update_msg.y = dilated_region.y_min;
	grid_update_msg.width = dilated_region.Width();
	grid_update_msg.height = dilated_region.Height();

	FillGridMsgCells(grid_update_msg.data, dilated_region, is_segmentation);
	grid_update_region_.Reset();
	return grid_update_msg;
}

avt_341::msg::OccupancyGrid ElevationGrid::GetGrid(bool is_segmentation) {
	avt_341::msg::OccupancyGrid grid;
	grid.header.frame_id = "map";
	grid.info.resolution = res_;
	grid.info.width = nx_;
	grid.info.height = ny_;
	grid.info.origin.position.x = llx_;
	grid.info.origin.position.y = lly_;
	grid.info.origin.orientation.w = 1.0;
	grid.info.origin.orientation.x = 0.0;
	grid.info.origin.orientation.y = 0.0;
	grid.info.origin.orientation.z = 0.0;

	FillGridMsgCells(grid.data, GridRegion(0, nx_, 0, ny_), is_segmentation);
	return grid;
}

avt_341::msg::OccupancyGrid ElevationGrid::GetGrid(double x, double y, double width, double height, bool is_segmentation) {
	double local_x_origin = x - width / 2.0;
	double local_y_origin = y - height / 2.0;
	int local_nx = (int)(width / res_);
	int local_ny = (int)(height / res_);
	int xi_min = std::max(0, (int)((local_x_origin - llx_) / res_));
	int yi_min = std::max(0, (int)((local_y_origin - lly_) / res_));
	int xi_max = std::min(nx_, xi_min + local_nx);
	int yi_max = std::min(ny_, yi_min + local_ny);

	avt_341::msg::OccupancyGrid grid;
	grid.header.frame_id = "map";
	grid.info.resolution = res_;
	grid.info.width = xi_max-xi_min;
	grid.info.height = yi_max-yi_min;
	grid.info.origin.position.x = xi_min * res_ + llx_;
	grid.info.origin.position.y = yi_min * res_ + lly_;
	grid.info.origin.orientation.w = 1.0;
	grid.info.origin.orientation.x = 0.0;
	grid.info.origin.orientation.y = 0.0;
	grid.info.origin.orientation.z = 0.0;

	FillGridMsgCells(grid.data, GridRegion(xi_min, xi_max, yi_min, yi_max), is_segmentation);
	return grid;
}

bool ElevationGrid::PastSlopeThreshold(const Cell& cell) const {
	return cell.height() / res_ > thresh_;
}

float ElevationGrid::Slope(const Cell& cell) const {
	return cell.height() / res_;
}

bool ElevationGrid::HasData() const {
	return std::any_of(cells_.begin(), cells_.end(), [](const std::vector<Cell>& row) {
		return std::any_of(row.begin(), row.end(), [](const Cell& cell) {
			return cell.filled();
		});
	});
}

void ElevationGrid::Reset() {
	is_resetting_ = true;

	while (HasData()) {
		ClearGrid();
		for (auto& cm : clear_methods_) {
			cm->Reset();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	is_resetting_ = false;
}

void ElevationGrid::SetNode(const std::shared_ptr<node::NodeProxy>& node_ref) {
	node_ref_ = node_ref;
}


void ElevationGrid::SetGridClearingMethod(const ClearMethodRosParameters & params) {

	BaseClearingSettings base_config;
	base_config.llx = llx_;
	base_config.lly = lly_;
	base_config.res = res_;
	base_config.grid_dilate_x = dilate_ ? lround(grid_dilate_x_ / res_) : 0;
	base_config.grid_dilate_y = dilate_ ? lround(grid_dilate_y_ / res_) : 0;
	base_config.thresh = thresh_;
	base_config.immediate_clear_dilation = params.immediate_clr_dilation;
	base_config.visualization_range = params.visualization_range;
	base_config.visualize = params.visualize;

	clear_methods_ = ClearingMethodFactory::CreateClearingMethods(node_ref_, cells_, params, base_config, this);
}


} // namespace perception
} //namespace avt_341