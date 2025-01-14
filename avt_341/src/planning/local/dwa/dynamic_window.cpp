#include <avt_341/planning/local/dwa/dynamic_window.hpp>

namespace avt_341 {
namespace planning {
namespace dwa {

DynamicWindow::DynamicWindow() {}

DynamicWindow::DynamicWindow(double speed_min,
                             double speed_max,
                             double speed_ang_min,
                             double speed_ang_max)
    : speed_min_(speed_min),
      speed_max_(speed_max),
      speed_ang_min_(speed_ang_min),
      speed_ang_max_(speed_ang_max) {}

} // namespace dwa
} // namespace planning
} // namespace avt_341