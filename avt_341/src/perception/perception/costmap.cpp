#include "avt_341/perception/costmap.h"
#include <math.h>
#include <avt_341/perception/clearing_methods/clearing_methods_factory.h>

#include "avt_341/perception/layers/point_cloud_layer.h"
#include "avt_341/perception/layers/polygon_layer.h"
#include "avt_341/perception/layers/static_grid_layer.h"

namespace avt_341::perception {

Costmap::Costmap(
	const std::shared_ptr<node::NodeProxy>& node_ref,
	const CostmapSizeInfo& size_info,
	const ThresholdSettings& thresholds,
	const DilationSettings& dilation,
	const TerrainRmsSettings& terrain_rms_config
	)
	: node_ref_(node_ref), size_info_(size_info), thresholds_(thresholds), dilation_(dilation), terrain_rms_config_(terrain_rms_config)
{
	// TODO: Should only create those which exist in configuration file, needs parameter refactoring
	std::vector<std::shared_ptr<CostmapLayer>> candidate_layers = {
		std::make_shared<StaticGridLayer>(node_ref, size_info, thresholds, dilation),
		std::make_shared<PolygonLayer>(node_ref, size_info, thresholds, dilation),
		std::make_shared<PointCloudLayer>(node_ref, size_info, thresholds, dilation),
	};

	layers_.clear();
	std::copy_if(candidate_layers.begin(), candidate_layers.end(), std::back_inserter(layers_),
		[](const std::shared_ptr<CostmapLayer>& layer) {
		return layer->IsValid();
	});

	odom_sub_ = node_ref_->create_subscription<msg::Odometry>(
		"avt_341/odometry",
		10,
		std::bind(&Costmap::OdometryCallback, this, std::placeholders::_1));

}

void Costmap::OdometryCallback(msg::OdometryPtr rcv_odom) {
	current_odom_ = *rcv_odom;
	for (const auto & layer : layers_){
		layer->UpdateOdometry(current_odom_);
	}
}

bool Costmap::HasSegmentation() const
{
	return std::all_of(layers_.begin(), layers_.end(),
		[](const std::shared_ptr<CostmapLayer>& layer) { return layer->HasSegmentation(); }
	);
}

void Costmap::Clear() const
{
	for (const auto & layer : layers_) {
		layer->Clear();
	}
}

void Costmap::Reset() const
{
	for (const auto & layer : layers_) {
		layer->Reset();
	}
}

void Costmap::DebugVisualize() const
{
	for (const auto & layer : layers_) {
		layer->Visualize();
	}
}

// void Costmap::ResizeGrid() {
// 	cells_.clear();
// 	std::vector<Cell> row;
// 	row.resize(size_info_.nx());
// 	cells_.resize(size_info_.ny(), row);
// }
//
// void Costmap::ClearGrid() {
// 	const auto ny = size_info_.ny();
// 	const auto nx = size_info_.nx();
// 	for (int i = 0; i < (ny); i++) {
// 		for (int j = 0; j < (nx); j++) {
// 			cells_[i][j] = Cell::Empty();
// 		}
// 	}
// }

// static bool IsPointInCone(const utils::vec2& test_point, const utils::vec2& p, const utils::vec2& v, float r, float angle) {
// 	utils::vec2 dir_to_point = test_point - p;
// 	float dist = utils::length(dir_to_point);
//
// 	if (dist > r) return false;
//
// 	dir_to_point.normalize();
// 	float dot_product = utils::dot(v, dir_to_point);
//
// 	float cos_angle = cosf(angle);
// 	return dot_product >= cos_angle;
// }
//
// // CTG, 7/23/25
// std::vector<utils::ivec2> Costmap::GetCellsInFov(float x, float y, float heading, float hfov, float range) {
// 	utils::vec2 p(x, y);
// 	float angle = 0.5f * hfov;
// 	utils::vec2 v(cosf(heading), sinf(heading));
// 	std::vector<utils::ivec2> cells_in_fov;
// 	int xi0 = std::max(0,(int)floor((x - range) / res_));
// 	int xi1 = std::min(nx_,(int)floor((x + range) / res_));
// 	int yi0 = std::max(0,(int)floor((y - range) / res_));
// 	int yi1 = std::min(ny_,(int)floor((y + range) / res_));
// 	for (int j = yi0; j < yi1; j++) {
// 		for (int i = xi0; i < xi1; i++) {
// 			utils::vec2 point = CellToPoint(i, j);
// 			bool in_view = IsPointInCone(point, p, v, range, angle);
// 			if (in_view) {
// 				utils::ivec2 c(i, j);
// 				cells_in_fov.push_back(c);
// 			}
// 		}
// 	}
// 	return cells_in_fov;
// }
//
// // CTG, 7/23/25
// void Costmap::GetSlopeRmsInFov(float& slope, float& rms, float x, float y, float heading, float hfov, float range) {
//
// 	std::vector<utils::ivec2> cells = GetCellsInFov(x, y, heading, hfov, range);
// 	slope = 0.0f;
// 	rms = 0.0f;
//
// 	if (cells.size() < 0)return;
//
// 	float navg = (float)cells.size();
// 	for (int i = 0; i < (int)cells.size(); i++) {
// 		rms += GetRmsAtCell(cells[i].x, cells[i].y);
// 		slope += GetTerrainSlopeAtCell(cells[i].x, cells[i].y);
//
// 	}
// 	rms /= navg;
// 	slope /= navg;
// 	return;
// }
//
// // CTG, 5/8/25
// float Costmap::GetRmsAtCoordinate(float x, float y) {
// 	int xi = (int)floor((x - llx_) / res_);
// 	int yi = (int)floor((y - lly_) / res_);
// 	float rms = GetRmsAtCell(xi, yi);
// 	return rms;
// }
//
// // CTG, 5/8/25
// float Costmap::GetRmsAtCell(int xi, int yi) {
// 	float rms = 0.0f;
// 	if (xi >= 0 && xi < nx_ && yi >= 0 && yi < ny_) {
// 		rms = cells_[yi][xi].rms;
// 	}
// 	return rms;
// }
//
// // CTG, 5/8/25
// float Costmap::GetTerrainSlopeAtCoordinate(float x, float y) {
// 	int xi = (int)floor((x - llx_) / res_);
// 	int yi = (int)floor((y - lly_) / res_);
// 	float slope = GetTerrainSlopeAtCell(xi, yi);
// 	return slope;
// }
//
//
// // CTG, 5/8/25
// float Costmap::GetTerrainSlopeAtCell(int xi, int yi) {
// 	float slope = 0.0f;
// 	if (xi >= 0 && xi < nx_ && yi >= 0 && yi < ny_) {
// 		int xl = std::max(xi - 1, 0);
// 		int xh = std::min(xi + 1, nx_ - 1);
// 		int yl = std::max(yi - 1, 0);
// 		int yh = std::min(yi + 1, ny_ - 1);
// 		float dx = (xh - xl) * res_;
// 		float dy = (yh - yl) * res_;
// 		float dz_dx = 0.0f;
// 		if (dx != 0.0f && cells_[yi][xh].num_points > 0 && cells_[yi][xl].num_points > 0) dz_dx = (cells_[yi][xh].low.val - cells_[yi][xl].low.val) / dx;
// 		float dz_dy = 0.0f;
// 		if (dy != 0.0f && cells_[yh][xi].num_points > 0 && cells_[yl][xi].num_points > 0) dz_dy = (cells_[yh][xi].low.val - cells_[yl][xi].low.val) / dy;
// 		slope = sqrtf(dz_dx * dz_dx + dz_dy * dz_dy);
// 	}
// 	return slope;
// }

// void Costmap::AddOccupancy(const avt_341::msg::PointCloud& point_cloud, std::vector< std::vector<Cell> >& cells, bool dilate) {
//
// 	bool has_segmentation_local = !point_cloud.channels.empty() && point_cloud.channels[0].name == "segmentation";
// 	has_segmentation_ = has_segmentation_local || has_segmentation_;
//
// 	const int dsize_x = GetDilateXSize();
// 	const int dsize_y = GetDilateYSize();
//
// 	// fill the cells with highest and lowest points
// 	for (int i = 0; i < point_cloud.points.size(); i++) {
// 		if (!(point_cloud.points[i].x == 0.0 && point_cloud.points[i].y == 0.0)) {
// 			int xi = (int)floor((point_cloud.points[i].x - llx_) / res_);
// 			int yi = (int)floor((point_cloud.points[i].y - lly_) / res_);
// 			if (xi >= 0 && xi < nx_ && yi >= 0 && yi < ny_) {
// 				Cell& cell = cells[yi][xi];
// 				grid_update_region_.UpdateBounds(xi,yi);
// 				const float original_slope = Slope(cell);
// 				float h = point_cloud.points[i].z;
//
// 				if (h > cell.high.val) {
// 					cell.high.val = h;
// 					cell.high.age = 0.0f;
// 				}
// 				if (h < cell.low.val) {
// 					cell.low.val = h;
// 					cell.low.age = 0.0f;
// 				}
// 				if (has_segmentation_local) {
// 					float terr_val = point_cloud.channels[0].values[i];
// 					cell.terrain = fmax(cell.terrain, terr_val);
// 				}
//
// 				// CTG 5/8/25, add calculations necessary for tracking RMS
// 				cell.summed_elev += h;
// 				cell.num_points += 1;
// 				if (cell.num_points > 0) {
// 					cell.avg_elev = cell.summed_elev / cell.num_points;
// 					float dh = h - cell.avg_elev;
// 					cell.sum_of_squares += dh * dh;
// 					cell.rms = sqrtf(cell.sum_of_squares / cell.num_points);
// 				}
// 				else {
// 					cell.avg_elev = 0.0f;
// 					cell.rms = 0.0f;
// 				}
//
// 				if (dilate) {
// 					DilateCell(cells, xi, yi, dsize_x, dsize_y, original_slope);
// 				}
// 			}
// 		}
// 	}
// }

//
// void Costmap::RecomputeGridDilation() {
//
// 	for (auto & row : cells_) {
// 		for (auto & cell : row) {
// 			cell.has_dilated = false;
// 			cell.dilated_val = 0;
// 		}
// 	}
//
// 	if (!dilate_) {
// 		return;
// 	}
//
// 	const int dsize_x = GetDilateXSize();
// 	const int dsize_y = GetDilateYSize();
//
// 	for (int xi = 0; xi < nx_; xi++) {
// 		for (int yi = 0; yi < ny_; yi++) {
// 			DilateCell(cells_, xi, yi, dsize_x, dsize_y);
// 		}
// 	}
// }
//
// void Costmap::DilateCell(
// 	std::vector<std::vector<Cell>>& cells,
// 	const int xi,
// 	const int yi,
// 	const int dsize_x,
// 	const int dsize_y,
// 	const float original_slope) {
//
// 	Cell & cell = cells[yi][xi];
//
// 	if (PastSlopeThreshold(cell) && (!cell.has_dilated || Slope(cell) > original_slope)) {
//
// 		cell.has_dilated = true;
// 		auto dilated_val = static_cast<uint8_t>(grid_dilate_proportion_ * static_cast<float>(GetGridCellValue(cell)));
//
// 		for (int xii = std::max(0, xi - dsize_x); xii <= std::min(xi + dsize_x, nx_ - 1); xii++) {
// 			for (int yii = std::max(0, yi - dsize_y); yii <= std::min(yi + dsize_y, ny_ - 1); yii++) {
// 				cells[yii][xii].dilated_val = std::max(dilated_val, cells[yii][xii].dilated_val);
// 			}
// 		}
//
// 	}
// }
//
// uint8_t Costmap::GetGridCellValue(const Cell& cell) const {
// 	if (!cell.filled())
// 		return 0;
//
// 	if (use_elevation_) {
// 		return cell.high.val > thresh_ ? GRID_MAX_VALUE : 0;
// 	}
// 	else {
// 		const auto slope = cell.height() / res_;
// 		return slope > thresh_ ? static_cast<uint8_t>(std::min(std::max(0.0f, grid_slope_mult_ * slope), static_cast<float>(GRID_MAX_VALUE))) : 0;
// 	}
//
// }

void Costmap::FillGridMsgCells(std::vector<int8_t> & data, const core::GridRegion region, bool is_segmentation) const {

	// TODO: Temporary change related to https://github.com/TulgaErsal/nato-avt-341-stack/issues/246
	//data.resize(region.Width()*region.Height());

	bool is_last = false;
	bool is_mean = false;

	int c = 0;
	std::vector<int> layer_values;
	layer_values.reserve(layers_.size());

	for (int i = region.y_min; i < region.y_max; i++) {
		for (int j = region.x_min; j < region.x_max; j++) {

			for (int k = 0; k < layers_.size(); k++){
				const auto & layer = layers_[k];
				const auto val = is_segmentation ? layer.GetSegValue(i, j) : layer.GetOccValue(i, j);
				if (val >= 0){
					layer_values.emplace_back(val);
				}
			}

			if (layer_values.empty()){
				data[c++] = -1;
			}
			else{
				data[c++] = is_last ? layer_values.back()
				: (is_mean ? std::accumulate(layer_values.begin(), layer_values.end(), 0) / layer_values.size()
					: *std::max_element(layer_values.begin(), layer_values.end()));
			}
		}
	}
}

msg::OccupancyGridUpdate Costmap::GetGridUpdate(bool is_segmentation) {
	msg::OccupancyGridUpdate grid_update_msg;
	grid_update_msg.header.frame_id = "map";

	core::GridRegion update_region;
	for (const auto & layer : layers_) {
		update_region.UpdateBounds(layer->GetUpdateRegion());
	}
	if (!update_region.HasData()) {
		return grid_update_msg;
	}
	int dilate_x = dilation_.GetNx(size_info_.res);
	int dilate_y = dilation_.GetNy(size_info_.res);
	core::GridRegion dilated_region = update_region.Dilate(dilate_x, dilate_y, size_info_.nx(), size_info_.ny());
	grid_update_msg.x = dilated_region.x_min;
	grid_update_msg.y = dilated_region.y_min;
	grid_update_msg.width = dilated_region.Width();
	grid_update_msg.height = dilated_region.Height();
	grid_update_msg.data.resize(dilated_region.Width()*dilated_region.Height());

	FillGridMsgCells(grid_update_msg.data, dilated_region, is_segmentation);
	for (const auto & layer : layers_) {
		layer->ResetUpdateRegion();
	}
	return grid_update_msg;
}

msg::OccupancyGrid Costmap::GetGrid(bool is_segmentation) {
	msg::OccupancyGrid grid;
	grid.header.frame_id = "map";
	grid.info = size_info_.ToRosMetadata();
	grid.data.resize(grid.info.width  * grid.info.height);

	FillGridMsgCells(grid.data, core::GridRegion(0, grid.info.width, 0, grid.info.height), is_segmentation);
	return grid;
}

msg::OccupancyGrid Costmap::GetGrid(double width, double height, bool is_segmentation) const {
	double local_x_origin = current_odom_.pose.pose.position.x - width / 2.0;
	double local_y_origin = current_odom_.pose.pose.position.y - height / 2.0;

	int local_nx = static_cast<int>(width / size_info_.res);
	int local_ny = static_cast<int>(height / size_info_.res);
	int xi_min = std::max(0, size_info_.ToXIdx(local_x_origin));
	int yi_min = std::max(0, size_info_.ToYIdx(local_y_origin));
	int xi_max = std::min(size_info_.nx(), xi_min + local_nx);
	int yi_max = std::min(size_info_.ny(), yi_min + local_ny);

	msg::OccupancyGrid grid;
	grid.header.frame_id = "map";
	grid.info = size_info_.ToRosMetadata();
	grid.info.width = local_nx; //xi_max-xi_min;
	grid.info.height =  local_ny; //yi_max-yi_min;
	grid.info.origin.position.x =  size_info_.ToXWorld(xi_min);
	grid.info.origin.position.y = size_info_.ToYWorld(yi_min);
	grid.data.resize(local_nx * local_ny);

	FillGridMsgCells(grid.data, core::GridRegion(xi_min, xi_max, yi_min, yi_max), is_segmentation);
	return grid;
}

bool Costmap::IsPointInCone(const utils::vec2& test_point, const utils::vec2& p, const utils::vec2& v, float r, float angle) {
	utils::vec2 dir_to_point = test_point - p;
	float dist = utils::length(dir_to_point);

	if (dist > r) return false;

	dir_to_point.normalize();
	float dot_product = utils::dot(v, dir_to_point);

	float cos_angle = cosf(angle);
	return dot_product >= cos_angle;
}

// CTG, 7/23/25
std::vector<utils::ivec2> Costmap::GetCellsInFov() const{

	float heading = utils::GetHeadingFromOrientation(current_odom_.pose.pose.orientation);
	const float x = current_odom_.pose.pose.position.x;
	const float y = current_odom_.pose.pose.position.y;

	utils::vec2 p(x, y);
	float angle = 0.5f * terrain_rms_config_.hfov;
	utils::vec2 v(cosf(heading), sinf(heading));
	std::vector<utils::ivec2> cells_in_fov;
	const auto range = terrain_rms_config_.range;

	int xi0 = std::max(0,static_cast<int>((x - range) / size_info_.res));
	int xi1 = std::min(size_info_.nx(),static_cast<int>((x + range) / size_info_.res));
	int yi0 = std::max(0,static_cast<int>((y - range) / size_info_.res));
	int yi1 = std::min(size_info_.ny(),static_cast<int>((y + range) / size_info_.res));
	for (int j = yi0; j < yi1; j++) {
		for (int i = xi0; i < xi1; i++) {
			utils::vec2 point = size_info_.ToPosWorld(i, j);
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
void Costmap::UpdateRmsAndSlope() {

	std::vector<utils::ivec2> idxs = GetCellsInFov();
	float slope = 0.0f;
	float rms = 0.0f;

	if (idxs.empty()){
		return;
	}

	float navg = static_cast<float>(idxs.size());
	for (const auto & idx : idxs) {
		rms += GetRmsAtCell(idx.x, idx.y);
		slope += GetTerrainSlopeAtCell(idx.x, idx.y);
	}
	rms /= navg;
	slope /= navg;

	slope_buffer_.push_back(static_cast<double>(slope));
	rms_buffer_.push_back(static_cast<double>(rms));
	if (slope_buffer_.size() > terrain_rms_config_.n_window) {
		slope_buffer_.pop_front();
		rms_buffer_.pop_front();
	}
}

// bool Costmap::PastSlopeThreshold(const Cell& cell) const {
// 	return cell.height() / res_ > thresh_;
// }
//
// float Costmap::Slope(const Cell& cell) const {
// 	return cell.height() / res_;
// }
//
// bool Costmap::HasData() const {
// 	return std::any_of(cells_.begin(), cells_.end(), [](const std::vector<Cell>& row) {
// 		return std::any_of(row.begin(), row.end(), [](const Cell& cell) {
// 			return cell.filled();
// 		});
// 	});
// }
//
// void Costmap::Reset() {
// 	is_resetting_ = true;
//
// 	while (HasData()) {
// 		ClearGrid();
// 		for (auto& cm : clear_methods_) {
// 			cm->Reset();
// 		}
// 		std::this_thread::sleep_for(std::chrono::milliseconds(200));
// 	}
//
// 	is_resetting_ = false;
// }

//
// void Costmap::SetGridClearingMethod(const ClearMethodRosParameters & params) {
//
// 	BaseClearingSettings base_config;
// 	base_config.size_info = &size_info_;
// 	base_config.thresholds = &thresholds_;
// 	base_config.dilation = &dilation_;
// 	base_config.immediate_clear_dilation = params.immediate_clr_dilation;
// 	base_config.visualization_range = params.visualization_range;
// 	base_config.visualize = params.visualize;
//
// 	clear_methods_ = ClearingMethodFactory::CreateClearingMethods(node_ref_, cells_, params, base_config, this);
// }


} //namespace avt_341::perception