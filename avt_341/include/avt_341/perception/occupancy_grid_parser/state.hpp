#pragma once

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>

namespace avt_341 {
namespace perception {
namespace occupancy {

class State {
  public:
    State();

    State(nav_msgs::msg::Odometry::SharedPtr odometry_message);

    State(const Eigen::Vector3d& position,
          const Eigen::Quaterniond& orientation,
          const double& speed);

    Eigen::Vector3d& GetPosition();

    const Eigen::Vector3d& GetPosition() const;

    const double& GetSpeed() const;

    Eigen::Transform<double, 2, Eigen::Affine> GetTransform();

  private:
    Eigen::Vector3d position_;
    Eigen::Quaterniond orientation_;
    double speed_;
};

} // namespace occupancy
} // namespace perception
} // namespace avt_341