#include <avt_341_nav/planning/local/dwa/cells.hpp>

namespace avt_341_nav {
namespace planning {
namespace dwa {

Cells::Cells() {}

void Cells::Add(double x, double y, double cost) {
    x_.push_back(x);
    y_.push_back(y);
    costs_.push_back(cost);
}

int Cells::GetNumberOfObstacles() { return (int)x_.size(); }

void Cells::Clear() {
    x_.clear();
    y_.clear();
}

} // namespace dwa
} // namespace planning
} // namespace avt_341_nav