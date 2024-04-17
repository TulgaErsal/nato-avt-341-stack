#include <avt_341/perception/occupancy_grid_parser/obstacle.hpp>

namespace avt_341 {
namespace perception {
namespace occupancy {

Obstacle::Obstacle()
    : position_(Eigen::Vector3d::Zero()),
      extent_(Eigen::Vector3d::Ones()),
      type_(ObstacleType::BOX) {}

Obstacle::Obstacle(const double& x,
                   const double& y,
                   const double& size,
                   const ObstacleType& obstacle_type)
    : position_(Eigen::Vector3d(x, y, 0.0)),
      extent_(size * Eigen::Vector3d::Ones()),
      type_(obstacle_type) {}

const Eigen::Vector3d& Obstacle::GetPosition() const { return position_; }

const Eigen::Quaterniond& Obstacle::GetOrientation() const {
    return orientation_;
}

const Eigen::Vector3d& Obstacle::GetExtent() const { return extent_; }

const ObstacleType Obstacle::GetType() const { return type_; }

const int32_t Obstacle::GetMarkerType() const {
    if(type_ == ObstacleType::BOX) {
        return 1;
    } else if(type_ == ObstacleType::CYLINDER) {
        return 3;
    } else
        throw std::invalid_argument("Invalid marker type");
}

const double& Obstacle::GetInflationFactor() const { return inflation_factor_; }

} // namespace occupancy
} // namespace perception
} // namespace avt_341