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
std::vector<utils::ivec2> CostmapLayer::GetCellsInFov(float x, float y, float heading, float hfov, float range) {
	utils::vec2 p(x, y);
	float angle = 0.5f * hfov;
	utils::vec2 v(cosf(heading), sinf(heading));
	std::vector<utils::ivec2> cells_in_fov;
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
void CostmapLayer::GetSlopeRmsInFov(float& slope, float& rms, float x, float y, float heading, float hfov, float range) {

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

	if (ignore_dilation_)
	{
		return;
	}

	for (auto & row : cells_) {
		for (auto & cell : row) {
			cell.has_dilated = false;
			cell.dilated_val = 0;
		}
	}

	if (!dilation_.enabled) {
		return;
	}

	const int dsize_x = GetDilationNx();
	const int dsize_y = GetDilationNy();

	for (int xi = 0; xi < size_info_.nx(); xi++) {
		for (int yi = 0; yi < size_info_.ny(); yi++) {
			DilateCell(cells_, xi, yi, dsize_x, dsize_y);
		}
	}
}

void CostmapLayer::DilateCell(
		std::vector<std::vector<Cell>>& cells,
		const int xi,
		const int yi,
		const int dsize_x,
		const int dsize_y,
		const float original_slope
	) {

	Cell & cell = cells[yi][xi];

	if (PastSlopeThreshold(cell) && (!cell.has_dilated || Slope(cell) > original_slope)) {

		cell.has_dilated = true;
		auto dilated_val = static_cast<uint8_t>(dilation_.proportion * static_cast<float>(GetGridCellValue(cell)));

		const auto nx = size_info_.nx();
		const auto ny = size_info_.ny();
		for (int xii = std::max(0, xi - dsize_x); xii <= std::min(xi + dsize_x, nx - 1); xii++) {
			for (int yii = std::max(0, yi - dsize_y); yii <= std::min(yi + dsize_y, ny - 1); yii++) {
				cells[yii][xii].dilated_val = std::max(dilated_val, cells[yii][xii].dilated_val);
			}
		}

	}
}

uint8_t CostmapLayer::GetGridCellValue(const Cell& cell) const {
	if (!cell.filled())
		return 0;

	if (thresholds_.use_elevation) {
		return cell.high.val > thresholds_.thresh ? GRID_MAX_VALUE : 0;
	}
	const auto slope = cell.height() / size_info_.res;
	return slope > thresholds_.thresh ? static_cast<uint8_t>(std::min(std::max(0.0f, thresholds_.grid_slope_mult() * slope), static_cast<float>(GRID_MAX_VALUE))) : 0;

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

}