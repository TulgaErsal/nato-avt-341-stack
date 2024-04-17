#pragma once

#include <Eigen/Geometry>

enum ObstacleType { BOX = 0, CYLINDER = 1 };

namespace avt_341 {
namespace perception {
namespace occupancy {

class Obstacle {
  public:
    Obstacle();
    Obstacle(const double& x,
             const double& y,
             const double& radius,
             const ObstacleType& obstacle_type = ObstacleType::BOX);
    const Eigen::Vector3d& GetPosition() const;
    const Eigen::Quaterniond& GetOrientation() const;
    const Eigen::Vector3d& GetExtent() const;
    const ObstacleType GetType() const;
    const int32_t GetMarkerType() const;
    const double& GetInflationFactor() const;

  private:
    Eigen::Vector3d position_;
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d extent_;
    ObstacleType type_;
    double inflation_factor_ = 1.0;
};

} // namespace occupancy
} // namespace perception
} // namespace avt_341