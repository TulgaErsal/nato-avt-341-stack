#include <avt_341/perception/occupancy_grid_parser/state.hpp>

namespace avt_341 {
namespace perception {
namespace occupancy {

State::State() {}

State::State(nav_msgs::msg::Odometry::SharedPtr odometry_message)
    : position_(Eigen::Vector3d(odometry_message->pose.pose.position.x,
                                odometry_message->pose.pose.position.y,
                                odometry_message->pose.pose.position.z)),
      orientation_(
          Eigen::Quaterniond(odometry_message->pose.pose.orientation.x,
                             odometry_message->pose.pose.orientation.y,
                             odometry_message->pose.pose.orientation.z,
                             odometry_message->pose.pose.orientation.w)),
      speed_(odometry_message->twist.twist.linear.x) {}

State::State(const Eigen::Vector3d& position,
             const Eigen::Quaterniond& orientation,
             const double& speed)
    : position_(position),
      orientation_(orientation),
      speed_(speed) {}

Eigen::Vector3d& State::GetPosition() { return position_; }

const Eigen::Vector3d& State::GetPosition() const { return position_; }

const double& State::GetSpeed() const { return speed_; }

Eigen::Transform<double, 2, Eigen::Affine> State::GetTransform() {
    return Eigen::Transform<double, 2, Eigen::Affine>(
        Eigen::Translation2d(position_.block<2, 1>(0, 0)) *
        Eigen::Rotation2Dd(
            orientation_.toRotationMatrix().eulerAngles(0, 1, 2).z()));
}

} // namespace occupancy
} // namespace perception
} // namespace avt_341