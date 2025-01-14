#include <avt_341/planning/local/dwa/obstacles.hpp>

namespace avt_341 {
namespace planning {
namespace dwa {

Obstacles::Obstacles() {}

void Obstacles::Add(double x, double y) {
    x_.push_back(x);
    y_.push_back(y);
}

int Obstacles::GetNumberOfObstacles() { return (int)x_.size(); }

double Obstacles::GetDistance(int i, double x, double y) {
    return std::hypot(x - x_[i], y - y_[i]);
}

void Obstacles::Clear() {
    x_.clear();
    y_.clear();
}

} // namespace dwa
} // namespace planning
} // namespace avt_341