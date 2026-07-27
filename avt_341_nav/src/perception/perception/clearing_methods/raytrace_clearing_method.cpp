#include "avt_341_nav/perception/clearing_methods/raytrace_clearing_method.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace avt_341_nav::perception {

constexpr int RaytraceClearingMethod::N_VOXELS_PER_CELL;

RaytraceClearingMethod::RaytraceClearingMethod(const rclcpp::Node::SharedPtr & node_ref,
                                               const std::shared_ptr<node::TfInterface> & tf,
                                               std::vector<std::vector<Cell>> & cells,
                                               const PerceptionSettings & settings,
                                               const RaytraceSettings & rt_config,
                                               CellObstacleCalculator *obs_calculator,
                                               bool handle_dilation)
    : RaytraceClearingMethod(
        node_ref,
        tf,
        cells,
        static_cast<int>(cells.size()),
        static_cast<int>(cells[0].size()),
        settings,
        rt_config,
        obs_calculator,
        handle_dilation
        ) {
}

RaytraceClearingMethod::RaytraceClearingMethod(const rclcpp::Node::SharedPtr & node_ref,
                                               const std::shared_ptr<node::TfInterface> & tf,
                                               std::vector<std::vector<Cell>> &cells,
                                               int Ny,
                                               int Nx,
                                               const PerceptionSettings & settings,
                                               const RaytraceSettings & rt_config,
                                               CellObstacleCalculator *obs_calculator,
                                               bool handle_dilation)
    :
    OccupancyClearingMethod(cells, Ny, Nx, settings, obs_calculator),
    node_(node_ref),
    tf_(tf),
    rt_config_(rt_config),
    handle_dilation_(handle_dilation){

    SetLidarFrame();

    if (settings_.clear_method.visualize) {
        minmax_vis_publisher_ = node_ref->create_publisher<visualization_msgs::msg::MarkerArray>("avt_341/occupancy_grid/voxels", 1);
    }

    if (rt_config_.use_voxels) {
        voxel_grid = new std::bitset<N_VOXELS_PER_CELL>[Nx_ * Ny_];
    }
}

void RaytraceClearingMethod::SetLidarFrame() {

    std::string lidar_frame_in = rt_config_.lidar_frame;
    if (lidar_frame_in.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Raytrace Clearing: Lidar frame not set, defaulting to 'lidar'");
        lidar_frame_in = "lidar";
    }

    // Remove leading slash + add slash at end (/)
    auto node_ns = std::string(node_->get_namespace());
    node_ns = node_ns.empty() || node_ns == "/" ? "" : (node_ns.substr(1, node_ns.size() - 1) + "/");
    lidar_frame_ = node_ns + lidar_frame_in;
}

RaytraceClearingMethod::~RaytraceClearingMethod() {
    delete voxel_grid;
}

void RaytraceClearingMethod::CleanupUnattachedDilation(const geometry_msgs::msg::Point &origin, std::vector<std::vector<Cell> > &cells) {

    const int dy = settings_.dilation_y_cells();
    const int dx = settings_.dilation_x_cells();

    // Clean up unattached dilated cells
    int x_0, y_0, x_N, y_N;
    GetGridBounds(origin, rt_config_.raytrace_range, x_0, y_0, x_N, y_N);

    for (int y = y_0; y < y_N; y++) {
        for (int x = x_0; x < x_N; x++) {

            if (cells[y][x].filled() && cells[y][x].dilated_val > 0) {

                bool found_obs = false;

                for (int i = std::max(0, y - dy); !found_obs && i <= std::min(Ny_ - 1, y + dy); i++) {
                    for (int j = std::max(0, x - dx); !found_obs && j <= std::min(Nx_ - 1, x + dx); j++) {

                        //if(cell_obstacle_calculator_->PastSlopeThreshold(cells[i][j]) || abs(cells[x][y].high.val - cells[i][j].high.val) > thresh_){
                        if (cells[i][j].has_dilated ||
                            (cells[i][j].filled() &&
                             abs(cells[y][x].high.val -
                                 cells[i][j].high.val) >
                                 settings_.costmap.thresholds.thresh)) {
                            found_obs = true;
                        }
                    }
                }

                if (!found_obs) {
                    cells[y][x].dilated_val = 0;
                }
            }

        }
    }
}

geometry_msgs::msg::Point RaytraceClearingMethod::TfTransformToPoint(
    const geometry_msgs::msg::TransformStamped &transform) const {
    geometry_msgs::msg::Point point;
    point.x = transform.transform.translation.x;
    point.y = transform.transform.translation.y;
    point.z = transform.transform.translation.z;
    return point;
}

geometry_msgs::msg::Point RaytraceClearingMethod::GetSensorOrigin() const {
    return TfTransformToPoint(tf_->lookup_transform("map", lidar_frame_));
}

geometry_msgs::msg::Point RaytraceClearingMethod::GetSensorOrigin(const rclcpp::Time &stamp) const {
    return TfTransformToPoint(tf_->lookup_transform("map", lidar_frame_, stamp));
}

void RaytraceClearingMethod::ClearOccupancy(const sensor_msgs::msg::PointCloud &point_cloud) {

    geometry_msgs::msg::Point origin = GetSensorOrigin(point_cloud.header.stamp);
    const float rt_range_sqr = rt_config_.raytrace_range * rt_config_.raytrace_range;

    for (const auto &point: point_cloud.points) {
        const auto dx = point.x - origin.x;
        const auto dy = point.y - origin.y;
        if (dx * dx + dy * dy < rt_range_sqr) {
            RaytraceLine(origin, point);
        }
    }

    if (handle_dilation_ &&
        !settings_.clear_method.immediate_clear_dilation) {
        CleanupUnattachedDilation(origin, cells_);
    }

    for (const auto &point: point_cloud.points) {
        int x = settings_.to_x_index(point.x);
        int y = settings_.to_y_index(point.y);
        int z = static_cast<int>((point.z - rt_config_.voxel_height_min) / rt_config_.voxel_height_res);
        if (x >= 0 && x < Nx_ && y >= 0 && y < Ny_ && z >= 0 && z < N_VOXELS_PER_CELL) {
            voxel_grid[y * Nx_ + x].set(z);
        }
    }
}

void RaytraceClearingMethod::ClearVoxelAt(int x, int y, int z) {

    if (x < 0 || x >= Nx_ || y < 0 || y >= Ny_) {
        return;
    }

    const float& vx_h_res = rt_config_.voxel_height_res;
    const float& vx_h_min = rt_config_.voxel_height_min;

    int z_i = static_cast<int>((cells_[y][x].high.val - vx_h_min) / vx_h_res);

    const int check_offset = rt_config_.clr_on_scan_below_only ? 1 : 0;
    if (!cells_[y][x].filled() || (z_i - check_offset) < z || z < 0 || z >= N_VOXELS_PER_CELL) {
        return;
    }

    int z_min = std::max(0, static_cast<int>((cells_[y][x].low.val - vx_h_min) / vx_h_res));

    if (rt_config_.use_voxels) {
        // Remove voxels from z_i down to z
        while (z_i >= z) {
            voxel_grid[y * Nx_ + x].set(z_i, false);
            z_i--;
        }
        // Find next filled cell below z_i. This will be the new max
        while (z_i >= z_min && !voxel_grid[y * Nx_ + x].test(z_i)) {
            z_i--;
        }
    }

    bool was_obstacle = cell_obstacle_calculator_->PastSlopeThreshold(cells_[y][x]);
    if (z_i < z_min) {
        // Empty cell
        cells_[y][x].ResetHeight();
        BroadcastClearToSiblings(x, y);
    } else {
        // Update cell height
        cells_[y][x].high.val = static_cast<float>(z_i) * vx_h_res + vx_h_min;
    }

    // Remove surrounding dilation if cell no longer obstacle
    if (handle_dilation_ && was_obstacle && !cell_obstacle_calculator_->PastSlopeThreshold(cells_[y][x])) {
        RemoveDilationAtCell(x, y, cells_);
    }
}

void RaytraceClearingMethod::RaytraceLine(const geometry_msgs::msg::Point &start, const geometry_msgs::msg::Point32 &end) {

    int x1 = settings_.to_x_index(start.x);
    int y1 = settings_.to_y_index(start.y);
    int z1 = static_cast<int>((start.z - rt_config_.voxel_height_min) / rt_config_.voxel_height_res);
    int x2 = settings_.to_x_index(end.x);
    int y2 = settings_.to_y_index(end.y);

    if (x2 < 0 || x2 >= Nx_ || y2 < 0 || y2 >= Ny_) {
        return;
    }

    int z2 = static_cast<int>((end.z - rt_config_.voxel_height_min) / rt_config_.voxel_height_res);
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int dz = std::abs(z2 - z1);
    int x_step = x2 > x1 ? 1 : -1;
    int y_step = y2 > y1 ? 1 : -1;
    int z_step = z2 > z1 ? 1 : -1;

    if (dx >= dy && dx >= dz) {
        int p1 = 2 * dy - dx;
        int p2 = 2 * dz - dx;
        while (x1 != x2) {
            x1 += x_step;
            if (p1 >= 0) {
                y1 += y_step;
                p1 -= 2 * dx;
            }
            if (p2 >= 0) {
                z1 += z_step;
                p2 -= 2 * dx;
            }
            p1 += 2 * dy;
            p2 += 2 * dz;
            if (x1 == x2 && y1 == y2) {
                break;
            }
            ClearVoxelAt(x1, y1, z1);
        }
    } else if (dy >= dx && dy >= dz) {
        int p1 = 2 * dx - dy;
        int p2 = 2 * dz - dy;
        while (y1 != y2) {
            y1 += y_step;
            if (p1 >= 0) {
                x1 += x_step;
                p1 -= 2 * dy;
            }
            if (p2 >= 0) {
                z1 += z_step;
                p2 -= 2 * dy;
            }
            p1 += 2 * dx;
            p2 += 2 * dz;
            if (x1 == x2 && y1 == y2) {
                break;
            }
            ClearVoxelAt(x1, y1, z1);
        }
    } else {
        int p1 = 2 * dy - dz;
        int p2 = 2 * dx - dz;
        while (z1 != z2) {
            z1 += z_step;
            if (p1 >= 0) {
                y1 += y_step;
                p1 -= 2 * dz;
            }
            if (p2 >= 0) {
                x1 += x_step;
                p2 -= 2 * dz;
            }
            p1 += 2 * dy;
            p2 += 2 * dx;
            if (x1 == x2 && y1 == y2) {
                break;
            }
            ClearVoxelAt(x1, y1, z1);
        }
    }
}

visualization_msgs::msg::Marker RaytraceClearingMethod::GetMarkerMsg(int type, int id, utils::vec3 color, float alpha,
                                                          double z_scale) const {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->now();
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.color.a = alpha;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = z_scale;
    marker.color.r = color.x;
    marker.color.g = color.y;
    marker.color.b = color.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    return marker;
}

void RaytraceClearingMethod::GetGridBounds(
    const geometry_msgs::msg::Point &origin,
    float range,
    int &x_0,
    int &y_0,
    int &x_N,
    int &y_N) const {

    if (range > 0.0) {
        x_0 = static_cast<int>(
            (origin.x - settings_.size_info().llx - range) /
            settings_.size_info().res);
        y_0 = static_cast<int>(
            (origin.y - settings_.size_info().lly - range) /
            settings_.size_info().res);
        x_N =
            x_0 + 2 * static_cast<int>(range / settings_.size_info().res);
        y_N =
            y_0 + 2 * static_cast<int>(range / settings_.size_info().res);
        x_0 = std::min(std::max(0, x_0), Nx_);
        y_0 = std::min(std::max(0, y_0), Ny_);
        x_N = std::min(std::max(0, x_N), Nx_);
        y_N = std::min(std::max(0, y_N), Ny_);
    } else {
        x_0 = 0;
        y_0 = 0;
        x_N = Nx_;
        y_N = Ny_;
    }
}

void RaytraceClearingMethod::Visualize() const {

    if (!settings_.clear_method.visualize){
        return;
    }

    OccupancyClearingMethod::Visualize();

    const float& vx_h_res = rt_config_.voxel_height_res;
    const float& vx_h_min = rt_config_.voxel_height_min;

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker mins_marker = GetMarkerMsg(visualization_msgs::msg::Marker::CUBE_LIST, 0, utils::vec3(0.0, 0.0, 1.0), 1.0f, 0.2);
    visualization_msgs::msg::Marker maxes_marker = GetMarkerMsg(visualization_msgs::msg::Marker::CUBE_LIST, 1, utils::vec3(1.0, 0.0, 0.0), 1.0f, 0.2);
    visualization_msgs::msg::Marker voxel_marker = GetMarkerMsg(visualization_msgs::msg::Marker::CUBE_LIST, 2, utils::vec3(0.8, 0.8, 0.8), 0.4f, vx_h_res);

    const float max_value = std::numeric_limits<float>::max() - 1e-5f;
    geometry_msgs::msg::Point origin = GetSensorOrigin();
    int x_0, y_0, x_N, y_N;
    GetGridBounds(
        origin, settings_.clear_method.visualization_range,
        x_0, y_0, x_N, y_N);

    for (int x = x_0; x < x_N; x++) {
        for (int y = y_0; y < y_N; y++) {

            bool has_value = cells_[y][x].low.val < max_value;
            if (has_value) {
                float x_i = settings_.to_x_world(x);
                float y_i = settings_.to_y_world(y);

                geometry_msgs::msg::Point p1;
                p1.x = x_i;
                p1.y = y_i;
                p1.z = cells_[y][x].low.val;
                mins_marker.points.push_back(p1);

                if (std::abs(cells_[y][x].high.val - cells_[y][x].low.val) > 1e-1) {
                    geometry_msgs::msg::Point p0;
                    p0.x = x_i;
                    p0.y = y_i;
                    p0.z = cells_[y][x].high.val;
                    maxes_marker.points.push_back(p0);
                }

                if (rt_config_.use_voxels) {

                    int z_pos = std::max(0, static_cast<int>((cells_[y][x].low.val - vx_h_min) / vx_h_res));
                    int max_z_pos = std::min(N_VOXELS_PER_CELL, static_cast<int>((cells_[y][x].high.val - vx_h_min) / vx_h_res));

                    while (z_pos <= max_z_pos) {
                        if (voxel_grid[y * Nx_ + x].test(z_pos)) {
                            geometry_msgs::msg::Point p2;
                            p2.x = x_i;
                            p2.y = y_i;
                            p2.z = vx_h_min + (static_cast<float>(z_pos) + 0.5) * vx_h_res;
                            voxel_marker.points.push_back(p2);
                        }
                        z_pos += 1;
                    }
                }
            }
        }
    }
    marker_array.markers.push_back(maxes_marker);
    marker_array.markers.push_back(mins_marker);
    marker_array.markers.push_back(voxel_marker);
    minmax_vis_publisher_->publish(marker_array);
}



void RaytraceClearingMethod::Reset() {

    OccupancyClearingMethod::Reset();

    for (int x = 0; x < Nx_; x++) {
        for (int y = 0; y < Ny_; y++) {
            if (rt_config_.use_voxels) {
                voxel_grid[y * Nx_ + x].reset();
            }
        }
    }

}

void RaytraceClearingMethod::ResetInternalCellState(int x, int y) {
    voxel_grid[y * Nx_ + x].reset();
}


std::string RaytraceClearingMethod::GetDescription() const {
    return "RaytraceClearingMethod: "
           "raytrace_range: " + std::to_string(rt_config_.raytrace_range) + "m" +
           ", voxel_height_res=" +  std::to_string(rt_config_.voxel_height_res) + "m" +
           ", voxel_height_min=" +  std::to_string(rt_config_.voxel_height_min) + "m" +
           ", use_voxels=" + (rt_config_.use_voxels ? "true" : "false") +
           ", clr_on_scan_below_only=" + (rt_config_.clr_on_scan_below_only ? "true" : "false");
}



// RAYTRACE CLEARING WITH OBSTACLE DISTANCE FILTERING
// ==================================================================================================================
// ==================================================================================================================

RaytraceWithFilteringClearingMethod::RaytraceWithFilteringClearingMethod(
    const rclcpp::Node::SharedPtr & node_ref,
    const std::shared_ptr<node::TfInterface> & tf,
    std::vector<std::vector<Cell>> & cells,
    const PerceptionSettings & settings,
    const RaytraceSettings & rt_config,
    CellObstacleCalculator *obs_calculator
    )
    : RaytraceClearingMethod(node_ref,
        tf,
        cells_with_clearing_,
        cells.size(),
        cells[0].size(),
        settings,
        rt_config,
        obs_calculator,
        false
        ),
    cells_without_clearing_(cells) {

    std::vector<Cell> row;
    row.resize(Nx_);
    cells_with_clearing_.resize(Ny_, row);

    // occupancy_delta just used for visualization
    const int N = 2 * static_cast<int>(
        rt_config.raytrace_range / settings_.size_info().res);
    occupancy_delta_ = std::vector<std::vector<bool> >(N, std::vector<bool>(N, false));

    if (settings_.clear_method.visualize) {
        occupancy_delta_publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("avt_341/occupancy_grid/clear_delta", 1);
    }
}

void RaytraceWithFilteringClearingMethod::ClearOccupancy(const sensor_msgs::msg::PointCloud &point_cloud) {
    RaytraceClearingMethod::ClearOccupancy(point_cloud);
    cell_obstacle_calculator_->AddOccupancy(point_cloud, cells_with_clearing_, false);
}

void RaytraceWithFilteringClearingMethod::OnOccupancyAdded(const sensor_msgs::msg::PointCloud &point_cloud,
                                                           const geometry_msgs::msg::Point &veh_pos) {
    geometry_msgs::msg::Point origin = GetSensorOrigin();
    int x_0, y_0, x_N, y_N;
    GetGridBounds(origin, rt_config_.raytrace_range, x_0, y_0, x_N, y_N);
    int search_range = static_cast<int>(
        rt_config_.obj_range_filter / settings_.size_info().res);
    last_position_.x = static_cast<float>(x_0);
    last_position_.y = static_cast<float>(y_0);

    for (int x = x_0; x < x_N; x++) {
        for (int y = y_0; y < y_N; y++) {

            bool candidate_clear = cell_obstacle_calculator_->PastSlopeThreshold(cells_without_clearing_[y][x]) &&
                                   !cell_obstacle_calculator_->PastSlopeThreshold(cells_with_clearing_[y][x]);

            occupancy_delta_[y - y_0][x - x_0] = candidate_clear;

            if (candidate_clear) {
                bool found_obs = false;
                const int i_0 = std::max(0, x - search_range);
                const int i_N = std::min(Nx_ - 1, x + search_range);
                const int j_0 = std::max(0, y - search_range);
                const int j_N = std::min(Ny_ - 1, y + search_range);
                for (int i = i_0; i <= i_N && !found_obs; i++) {
                    for (int j = j_0; j <= j_N && !found_obs; j++) {
                        found_obs |= cell_obstacle_calculator_->PastSlopeThreshold(cells_with_clearing_[j][i]);
                    }
                }
                if (!found_obs) {
                    // Apply cleared cell
                    cells_without_clearing_[y][x].high = cells_with_clearing_[y][x].high;
                    cells_without_clearing_[y][x].low = cells_with_clearing_[y][x].low;
                    RemoveDilationAtCell(x, y, cells_without_clearing_);
                    BroadcastClearToSiblings(x, y);
                }
            }
        }
    }

    CleanupUnattachedDilation(origin, cells_without_clearing_);
}

void RaytraceWithFilteringClearingMethod::Visualize() const {

    if (!settings_.clear_method.visualize){
        return;
    }

    RaytraceClearingMethod::Visualize();

    const int N_size = 2 * static_cast<int>(
        rt_config_.raytrace_range / settings_.size_info().res);

    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header.stamp = node_->now();
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.resolution = settings_.size_info().res;
    occupancy_grid.info.width = N_size;
    occupancy_grid.info.height = N_size;
    occupancy_grid.info.origin.position.x = last_position_.x - 2 * rt_config_.raytrace_range;
    occupancy_grid.info.origin.position.y = last_position_.y - 2 * rt_config_.raytrace_range;
    occupancy_grid.info.origin.position.z = 1.0;
    occupancy_grid.info.origin.orientation.w = 1.0;
    occupancy_grid.info.origin.orientation.x = 0.0;
    occupancy_grid.info.origin.orientation.y = 0.0;
    occupancy_grid.info.origin.orientation.z = 0.0;
    occupancy_grid.data.resize(N_size * N_size);

    for (int i = 0; i < N_size; i++) {
        for (int j = 0; j < N_size; j++) {
            occupancy_grid.data[i * N_size + j] = (uint8_t) (occupancy_delta_[i][j] ? 100 : 0);
        }
    }

    occupancy_delta_publisher_->publish(occupancy_grid);
}

void RaytraceWithFilteringClearingMethod::Reset() {
    RaytraceClearingMethod::Reset();

    Cell empty_cell;
    for (int i = 0; i < Ny_; i++) {
        for (int j = 0; j < Nx_; j++) {
            cells_with_clearing_[i][j] = empty_cell;
        }
    }

    for (auto &row: occupancy_delta_) {
        for (auto &&elem: row) {
            elem = false;
        }
    }

    last_position_ = utils::vec2(0.0, 0.0);
}

void RaytraceWithFilteringClearingMethod::ResetInternalCellState(int x, int y) {
    cells_with_clearing_[y][x].ResetHeight();
    occupancy_delta_[y][x] = false;
}

std::string RaytraceWithFilteringClearingMethod::GetDescription() const {
    return RaytraceClearingMethod::GetDescription();
}

}
