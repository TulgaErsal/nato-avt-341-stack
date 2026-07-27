#include <avt_341_nav/planning/local/dwa/dynamic_window.hpp>

namespace avt_341_nav {
namespace planning {
namespace dwa {

DynamicWindow::DynamicWindow()
    : DynamicWindow(std::numeric_limits<double>::min(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::min(),
                    std::numeric_limits<double>::max()) {}

DynamicWindow::DynamicWindow(double minimum_speed,
                             double maximum_speed,
                             double minimum_steering_rate,
                             double maximum_steering_rate)
    : minimum_speed_(minimum_speed),
      maximum_speed_(maximum_speed),
      minimum_steering_rate_(minimum_steering_rate),
      maximum_steering_rate_(maximum_steering_rate) {
    assert(minimum_speed < maximum_speed);
    assert(minimum_steering_rate < maximum_steering_rate);
}

void DynamicWindow::Update(double minimum_speed,
                           double maximum_speed,
                           double minimum_steering_rate,
                           double maximum_steering_rate) {
    minimum_speed_ = minimum_speed;
    maximum_speed_ = maximum_speed;
    minimum_steering_rate_ = minimum_steering_rate;
    maximum_steering_rate_ = maximum_steering_rate;
}

const double& DynamicWindow::GetMinimumSpeed() { return minimum_speed_; }

const double& DynamicWindow::GetMaximumSpeed() { return maximum_speed_; }

const double& DynamicWindow::GetMinimumSteeringRate() {
    return minimum_steering_rate_;
}

const double& DynamicWindow::GetMaximumSteeringRate() {
    return maximum_steering_rate_;
}

} // namespace dwa
} // namespace planning
} // namespace avt_341_nav