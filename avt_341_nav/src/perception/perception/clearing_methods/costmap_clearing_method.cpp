#include "avt_341_nav/perception/clearing_methods/costmap_clearing_method.h"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

namespace avt_341_nav::perception {

// BASE CLASS
// ==================================================================================================================
// ==================================================================================================================


OccupancyClearingMethod::OccupancyClearingMethod(std::vector<std::vector<Cell>> &cells,
                                                 const PerceptionSettings &settings,
                                                 CellObstacleCalculator *cell_obstacle_calculator)
    : OccupancyClearingMethod(
        cells,
        static_cast<int>(cells.size()),
        static_cast<int>(cells[0].size()),
        settings,
        cell_obstacle_calculator
    )
{
}

OccupancyClearingMethod::OccupancyClearingMethod(std::vector<std::vector<Cell>> &cells,
                                                    int Ny,
                                                    int Nx,
                                                    const PerceptionSettings &settings,
                                                    CellObstacleCalculator *cell_obstacle_calculator)
    :
    cells_(cells),
    Ny_(Ny),
    Nx_(Nx),
    settings_(settings),
    cell_obstacle_calculator_(cell_obstacle_calculator)
{
}

void OccupancyClearingMethod::Visualize() const {
    // Default empty implementation
}

void OccupancyClearingMethod::OnOccupancyAdded(const sensor_msgs::msg::PointCloud &point_cloud, const geometry_msgs::msg::Point &veh_pos) {
    // Default empty implementation
}

void OccupancyClearingMethod::Reset() {
    // Default empty implementation
}

void OccupancyClearingMethod::ResetInternalCellState(int x, int y) {
    // Default empty implementation
}

void OccupancyClearingMethod::SetSiblingClearingMethods(
    const std::vector<std::shared_ptr<OccupancyClearingMethod>> &sibling_cms) {
    sibling_cms_ = sibling_cms;
}

void OccupancyClearingMethod::BroadcastClearToSiblings(int x, int y) {

    if (sibling_cms_.empty()) {
        return;
    }

    for (const auto& cm : sibling_cms_) {
        cm->ResetInternalCellState(x, y);
    }
}

// NULL CLEARING
// ==================================================================================================================
// ==================================================================================================================

NullClearingMethod::NullClearingMethod(std::vector<std::vector<Cell> > &cells,
                                       const PerceptionSettings &settings,
                                       CellObstacleCalculator *obs_calculator)
    : OccupancyClearingMethod(
        cells,
        static_cast<int>(cells.size()),
        static_cast<int>(cells[0].size()),
        settings,
        obs_calculator) {
}

void NullClearingMethod::ClearOccupancy(const sensor_msgs::msg::PointCloud &point_cloud) {
    // No implementation
}

void OccupancyClearingMethod::RemoveDilationAtCell(int x, int y) const {
    RemoveDilationAtCell(x, y, cells_);
}

void OccupancyClearingMethod::RemoveDilationAtCell(int x, int y, std::vector<std::vector<Cell>> &cells) const {

    const int dy = settings_.dilation_y_cells();
    const int dx = settings_.dilation_x_cells();
    const bool clr_dil =
        settings_.clear_method.immediate_clear_dilation;
    const float thresh = settings_.costmap.thresholds.thresh;

    cells[y][x].has_dilated = false;

    for (int i = std::max(0, y - dy); i <= std::min(Ny_ - 1, y + dy); i++) {
        for (int j = std::max(0, x - dx); j <= std::min(Nx_ - 1, x + dx); j++) {

            if (cells[i][j].dilated_val > 0 && (clr_dil || (cells[i][j].filled() && abs(cells[y][x].high.val - cells[i][j].high.val) < thresh))) {

                bool found_obs = false;

                // TODO: can remove extra loops with counts or set tracking dilated cells
                for (int ii = std::max(0, i - dy); !found_obs && ii <= std::min(Ny_ - 1, i + dy); ii++) {
                    for (int jj = std::max(0, j - dx); !found_obs && jj <= std::min(Nx_ - 1, j + dx); jj++) {
                        found_obs = cell_obstacle_calculator_->PastSlopeThreshold(cells[ii][jj]);
                    }
                }

                if (!found_obs) {
                    cells[i][j].dilated_val = 0;
                }
            }
        }
    }
}

std::string NullClearingMethod::GetDescription() const {
    return "NullClearingMethod: No clearing done";
}

const std::string CostmapClearMethodType::None = "none";
const std::string CostmapClearMethodType::Time = "time";
const std::string CostmapClearMethodType::NoObsTime = "no_obs_time";
const std::string CostmapClearMethodType::Raytrace = "raytrace";
const std::string CostmapClearMethodType::RaytraceWithFiltering = "raytrace_obs_filter";
const std::string CostmapClearMethodType::ChannelThreshold = "channel_threshold";

}
