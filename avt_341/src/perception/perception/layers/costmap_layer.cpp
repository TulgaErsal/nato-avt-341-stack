#include "avt_341/perception/layers/costmap_layer.h"
#include <algorithm>

namespace avt_341::perception
{

CostmapLayer::CostmapLayer(
    const std::shared_ptr<node::NodeProxy>& node_ref,
    const PerceptionSettings& settings,
    const std::string& label,
    const std::shared_ptr<core::ComputeTimeRecorder>& compute_time_recorder,
    bool contribute_occupancy,
    bool contribute_segmentation
    )
    : contribute_occupancy_(contribute_occupancy),
    contribute_segmentation_(contribute_segmentation),
    node_ref_(node_ref), compute_time_recorder_(compute_time_recorder),
    settings_(settings), label_(label)
{
    Resize();
}

void CostmapLayer::UpdateThresholds(
    const float slope_threshold, const float slope_threshold_max)
{
    settings_.update_thresholds(slope_threshold, slope_threshold_max);
    RecomputeGridDilation();
}

void CostmapLayer::UpdateOdometry(const msg::Odometry& odom_msg)
{
	current_odom_ = odom_msg;
}

float CostmapLayer::GetRmsAtCoordinate(float x, float y) const
{
	const utils::ivec2 idx = settings_.to_index(x, y);
	return GetRmsAtCell(idx.x, idx.y);
}

float CostmapLayer::GetTerrainSlopeAtCoordinate(float x, float y) {
	const utils::ivec2 idx = settings_.to_index(x, y);
	return GetTerrainSlopeAtCell(idx.x, idx.y);
}


// CTG, 5/8/25
float CostmapLayer::GetTerrainSlopeAtCell(int xi, int yi) {
	float slope = -1.0f;
	const auto& res = settings_.size_info().res;
	const auto nx = settings_.nx();
	const auto ny = settings_.ny();

	if (xi >= 0 && xi < nx && yi >= 0 && yi < ny) {
		int xl = std::max(xi - 1, 0);
		int xh = std::min(xi + 1, nx - 1);
		int yl = std::max(yi - 1, 0);
		int yh = std::min(yi + 1, ny - 1);
		float dx = (xh - xl) * res;
		float dy = (yh - yl) * res;
		float dz_dx = 0.0f;
		if (dx != 0.0f && cells_[yi][xh].num_points > 0 && cells_[yi][xl].num_points > 0) dz_dx = (cells_[yi][xh].low.val - cells_[yi][xl].low.val) / dx;
		float dz_dy = 0.0f;
		if (dy != 0.0f && cells_[yh][xi].num_points > 0 && cells_[yl][xi].num_points > 0) dz_dy = (cells_[yh][xi].low.val - cells_[yl][xi].low.val) / dy;
		slope = sqrtf(dz_dx * dz_dx + dz_dy * dz_dy);
	}
	return slope;
}

void CostmapLayer::Resize() {
    cells_.clear();
    std::vector<Cell> row;
    row.resize(settings_.nx());
    cells_.resize(settings_.ny(), row);
}

void CostmapLayer::Clear() {
    const auto ny = settings_.ny();
    const auto nx = settings_.nx();
    for (int i = 0; i < (ny); i++) {
        for (int j = 0; j < (nx); j++) {
            cells_[i][j] = Cell::Empty();
        }
    }
}

void CostmapLayer::RecomputeGridDilation() {

	if (!settings_.costmap.dilation.enabled) {
		return;
	}

	for (auto & row : cells_) {
		for (auto & cell : row) {
			cell.has_dilated = false;
			cell.dilated_val = -1;
		}
	}

	for (int xi = 0; xi < settings_.nx(); xi++) {
		for (int yi = 0; yi < settings_.ny(); yi++) {
			DilateCell(cells_, xi, yi);
		}
	}
}

void CostmapLayer::DilateCell(
		std::vector<std::vector<Cell>>& cells,
		const int xi,
		const int yi,
		const float original_slope
	) {

	Cell & cell = cells[yi][xi];

	if (PastSlopeThreshold(cell) && (!cell.has_dilated || Slope(cell) > original_slope)) {

		cell.has_dilated = true;
		auto dilated_val = static_cast<int>(
			settings_.costmap.dilation.proportion *
			static_cast<float>(GetGridCellValue(cell)));

		const auto nx = settings_.nx();
		const auto ny = settings_.ny();
		const int dsize_x = settings_.dilation_x_cells();
		const int dsize_y = settings_.dilation_y_cells();
		for (int xii = std::max(0, xi - dsize_x); xii <= std::min(xi + dsize_x, nx - 1); xii++) {
			for (int yii = std::max(0, yi - dsize_y); yii <= std::min(yi + dsize_y, ny - 1); yii++) {
				cells[yii][xii].dilated_val = std::max(dilated_val, cells[yii][xii].dilated_val);
			}
		}

	}
}

int CostmapLayer::GetGridCellValue(const Cell& cell) const {
	if (!cell.filled() || !contribute_occupancy_){
		return -1;
	}

	const auto& thresholds = settings_.costmap.thresholds;
	if (thresholds.use_elevation) {
		return cell.high.val > thresholds.thresh ? GRID_MAX_VALUE : 0;
	}
	const auto slope = cell.height() / settings_.size_info().res;
	return slope > thresholds.thresh
		       ? static_cast<int>(
		             std::min(
		                 std::max(
		                     0.0F,
		                     settings_.grid_slope_multiplier() * slope),
		                 static_cast<float>(GRID_MAX_VALUE)))
		       : 0;

}

bool CostmapLayer::HasData() const {
	return std::any_of(cells_.begin(), cells_.end(), [](const std::vector<Cell>& row) {
		return std::any_of(row.begin(), row.end(), [](const Cell& cell) {
			return cell.filled();
		});
	});
}

void CostmapLayer::Reset() {
	// Nothing to do in base class
}

void CostmapLayer::Visualize() {
	// Nothing to do in base class
}

}
