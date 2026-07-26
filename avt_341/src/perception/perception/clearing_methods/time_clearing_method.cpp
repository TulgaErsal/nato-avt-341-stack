#include "avt_341/perception/clearing_methods/time_clearing_method.h"

namespace avt_341::perception {

TimedClearingMethod::TimedClearingMethod(
    float max_point_age,
    std::vector<std::vector<Cell>> &cells,
    const PerceptionSettings &settings,
    CellObstacleCalculator *obs_calculator
    )
    : OccupancyClearingMethod(cells, settings, obs_calculator),
      max_point_age_(max_point_age) {
}

void TimedClearingMethod::AgeCells(const float dt) const {
    for (int i = 0; i < Ny_; i++) {
        for (int j = 0; j < Nx_; j++) {
            cells_[i][j].AgeCell(dt);
        }
    }
}

void TimedClearingMethod::ClearOccupancy(const msg::PointCloud &point_cloud) {

    const double now = node::seconds_from_header(point_cloud.header);

    if (last_timestamp_ > 0) {
        AgeCells(now - last_timestamp_);
    }

    last_timestamp_ = now;

    for (int yi = 0; yi < Ny_; yi++) {
        for (int xi = 0; xi < Nx_; xi++) {
            if (cells_[yi][xi].low.age > max_point_age_ || cells_[yi][xi].high.age > max_point_age_) {
                cells_[yi][xi].ResetHeight();
                RemoveDilationAtCell(xi, yi);
                BroadcastClearToSiblings(xi, yi);
            }
        }
    }
}

std::string TimedClearingMethod::GetDescription() const {
    return "TimedClearingMethod: max_point_age: " + std::to_string(max_point_age_) + " s";
}

TimedNoObsClearingMethod::TimedNoObsClearingMethod(
    std::vector<std::vector<Cell>> &cells,
    const PerceptionSettings &settings,
    const TimedNoObsClearingSettings &time_config,
    CellObstacleCalculator *obs_calculator
    )
    : OccupancyClearingMethod(cells, settings, obs_calculator),
      time_config_(time_config) {

    std::vector<Cell> row;
    row.resize(Nx_);
    timed_cells_.resize(Ny_, row);

    std::vector<TimedNoObsData> row_data;
    row_data.resize(Nx_);
    timed_cells_data.resize(Ny_, row_data);
}

void TimedNoObsClearingMethod::ClearOccupancy(const msg::PointCloud &point_cloud) {
    // No implementation
}

void TimedNoObsClearingMethod::OnOccupancyAdded(const msg::PointCloud &point_cloud, const msg::Point &veh_pos) {

    cell_obstacle_calculator_->AddOccupancy(point_cloud, timed_cells_, false);

    // Small subtraction in case user enters 0.0 so that this setting is ignored
    constexpr float EPS = 1e-3;
    const float dst_sqr_thresh =
        time_config_.no_obs_dist_threshold *
            time_config_.no_obs_dist_threshold -
        EPS;

    for (int i = 0; i < point_cloud.points.size(); i++) {

        const int xi = static_cast<int>(
            (point_cloud.points[i].x - settings_.size_info().llx) /
            settings_.size_info().res);
        const int yi = static_cast<int>(
            (point_cloud.points[i].y - settings_.size_info().lly) /
            settings_.size_info().res);

        if (xi >= 0 && xi < Nx_ && yi >= 0 && yi < Ny_) {

            if (cell_obstacle_calculator_->PastSlopeThreshold(timed_cells_[yi][xi])) {

                // Monitored timed cell is an obstacle, reset timed no-obstacle check
                timed_cells_[yi][xi].ResetHeight();
                timed_cells_data[yi][xi].Reset(node::seconds_from_header(point_cloud.header));
                timed_cells_data[yi][xi].last_scan_pos_x = veh_pos.x;
                timed_cells_data[yi][xi].last_scan_pos_y = veh_pos.y;

            } else if (cell_obstacle_calculator_->PastSlopeThreshold(cells_[yi][xi])) {

                // Cell in occupancy grid currently marked as an obstacle but monitored timed_cells is not
                const float dx = veh_pos.x - timed_cells_data[yi][xi].last_scan_pos_x;
                const float dy = veh_pos.y - timed_cells_data[yi][xi].last_scan_pos_y;
                const bool dist_thresh_met = dx*dx + dy*dy > dst_sqr_thresh;

                timed_cells_data[yi][xi].num_samples += (dist_thresh_met ? 1 : 0);

                if (dist_thresh_met) {
                    timed_cells_data[yi][xi].last_scan_pos_x = veh_pos.x;
                    timed_cells_data[yi][xi].last_scan_pos_y = veh_pos.y;
                }

                if (timed_cells_data[yi][xi].num_samples >=
                        time_config_.sampled_threshold
                    && node::seconds_from_header(point_cloud.header) - timed_cells_data[yi][xi].obs_time >
                    time_config_.max_point_age) {

                    // But currently timed tracked cell is not an obstacle. Clear it original.
                    cells_[yi][xi].ResetHeight();
                    RemoveDilationAtCell(xi, yi, cells_);
                    ResetInternalCellState(xi, yi);
                }
            }
        }
    }
}

void TimedNoObsClearingMethod::ResetInternalCellState(int xi, int yi) {
    timed_cells_[yi][xi].ResetHeight();
    timed_cells_data[yi][xi].Reset();
}

void TimedNoObsClearingMethod::Reset() {
    OccupancyClearingMethod::Reset();
    for (int yi = 0; yi < Ny_; yi++) {
        for (int xi = 0; xi < Nx_; xi++) {
            ResetInternalCellState(xi, yi);
        }
    }
}

std::string TimedNoObsClearingMethod::GetDescription() const {
    return "TimedNoObsClearingMethod: "
           "time_threshold=" + std::to_string(time_config_.max_point_age) + "s" +
            ", sample_threshold=" +
            std::to_string(time_config_.sampled_threshold) +
            ", distance_threshold=" +
            std::to_string(time_config_.no_obs_dist_threshold) + "m";
}

}
