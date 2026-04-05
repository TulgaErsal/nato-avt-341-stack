#include "avt_341/perception/layers/costmap_layer.h"

namespace avt_341::perception
{

CostmapLayer::CostmapLayer(
    const std::shared_ptr<node::NodeProxy>& node_ref,
    const CostmapSizeInfo& size_info,
    const ThresholdSettings& thresholds,
    const DilationSettings& dilation
    )
    : node_ref_(node_ref), size_info_(size_info), thresholds_(thresholds), dilation_(dilation)
{
    Resize();
}

void CostmapLayer::UpdateOdometry(const msg::Odometry& odom_msg)
{
	current_odom_ = odom_msg;
}

// CTG, 5/8/25
float CostmapLayer::GetRmsAtCoordinate(float x, float y) {
	const utils::ivec2 idx = size_info_.ToIdx(x, y);
	float rms = GetRmsAtCell(idx.x, idx.y);
	return rms;
}

// CTG, 5/8/25
float CostmapLayer::GetRmsAtCell(int xi, int yi) {
	float rms = 0.0f;
	if (xi >= 0 && xi < size_info_.nx() && yi >= 0 && yi < size_info_.ny()) {
		rms = cells_[yi][xi].rms;
	}
	return rms;
}

// CTG, 5/8/25
float CostmapLayer::GetTerrainSlopeAtCoordinate(float x, float y) {
	const utils::ivec2 idx = size_info_.ToIdx(x, y);
	return GetTerrainSlopeAtCell(idx.x, idx.y);
}


// CTG, 5/8/25
float CostmapLayer::GetTerrainSlopeAtCell(int xi, int yi) {
	float slope = 0.0f;
	const auto& res = size_info_.res;
	const auto& nx = size_info_.nx();
	const auto& ny = size_info_.ny();

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
    row.resize(size_info_.nx());
    cells_.resize(size_info_.ny(), row);
}

void CostmapLayer::Clear() {
    const auto ny = size_info_.ny();
    const auto nx = size_info_.nx();
    for (int i = 0; i < (ny); i++) {
        for (int j = 0; j < (nx); j++) {
            cells_[i][j] = Cell::Empty();
        }
    }
}

bool CostmapLayer::PastSlopeThreshold(const Cell& cell) const {
	return cell.height() / size_info_.res > thresholds_.thresh;
}

float CostmapLayer::Slope(const Cell& cell) const {
	return cell.height() / size_info_.res ;
}

void CostmapLayer::RecomputeGridDilation() {

	if (!dilation_.enabled) {
		return;
	}

	for (auto & row : cells_) {
		for (auto & cell : row) {
			cell.has_dilated = false;
			cell.dilated_val = 0;
		}
	}

	for (int xi = 0; xi < size_info_.nx(); xi++) {
		for (int yi = 0; yi < size_info_.ny(); yi++) {
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
		auto dilated_val = static_cast<uint8_t>(dilation_.proportion * static_cast<float>(GetGridCellValue(cell)));

		const auto nx = size_info_.nx();
		const auto ny = size_info_.ny();
		const int dsize_x = dilation_.GetNx(size_info_.res);
		const int dsize_y = dilation_.GetNy(size_info_.res);
		for (int xii = std::max(0, xi - dsize_x); xii <= std::min(xi + dsize_x, nx - 1); xii++) {
			for (int yii = std::max(0, yi - dsize_y); yii <= std::min(yi + dsize_y, ny - 1); yii++) {
				cells[yii][xii].dilated_val = std::max(dilated_val, cells[yii][xii].dilated_val);
			}
		}

	}
}

int CostmapLayer::GetGridCellValue(const Cell& cell) const {
	if (!cell.filled()){
		return -1;
	}

	if (thresholds_.use_elevation) {
		return cell.high.val > thresholds_.thresh ? GRID_MAX_VALUE : 0;
	}
	const auto slope = cell.height() / size_info_.res;
	return slope > thresholds_.thresh ? std::min(std::max(0.0f, thresholds_.grid_slope_mult() * slope), static_cast<float>(GRID_MAX_VALUE)) : 0;

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
