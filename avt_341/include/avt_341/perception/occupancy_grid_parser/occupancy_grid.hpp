#pragma once

#include <Eigen/Geometry>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace avt_341 {
namespace perception {
namespace occupancy {

class OccupancyGrid {
    typedef Eigen::
        Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
            OccupancyGridMatrix;

  public:
    OccupancyGrid();
    OccupancyGrid(
        nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_message);
    int8_t operator()(int row, int column);
    const double& GetResolution() const;
    const Eigen::Vector2d& GetPosition() const;
    int GetWidth() const;
    int GetHeight() const;
    std::string GetFrameId() const;

  private:
    std::string frame_id_;
    OccupancyGridMatrix data_;
    Eigen::Vector2d position_;
    double resolution_;
};

} // namespace occupancy
} // namespace perception
} // namespace avt_341