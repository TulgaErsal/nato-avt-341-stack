#include <avt_341/perception/occupancy_grid_parser/occupancy_grid.hpp>

namespace avt_341 {
namespace perception {
namespace occupancy {

OccupancyGrid::OccupancyGrid() {}

OccupancyGrid::OccupancyGrid(
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_message) {
    frame_id_ = occupancy_grid_message->header.frame_id;
    data_ = Eigen::Map<OccupancyGridMatrix>(occupancy_grid_message->data.data(),
                                            occupancy_grid_message->info.height,
                                            occupancy_grid_message->info.width);
    position_ = Eigen::Vector2d(occupancy_grid_message->info.origin.position.x,
                                occupancy_grid_message->info.origin.position.y);
    resolution_ = occupancy_grid_message->info.resolution;
}

int8_t OccupancyGrid::operator()(int row, int column) {
    return data_(row, column);
}

const double& OccupancyGrid::GetResolution() const { return resolution_; }

const Eigen::Vector2d& OccupancyGrid::GetPosition() const { return position_; }

int OccupancyGrid::GetWidth() const { return data_.cols(); }

int OccupancyGrid::GetHeight() const { return data_.rows(); }

std::string OccupancyGrid::GetFrameId() const { return frame_id_; }

} // namespace occupancy
} // namespace perception
} // namespace avt_341