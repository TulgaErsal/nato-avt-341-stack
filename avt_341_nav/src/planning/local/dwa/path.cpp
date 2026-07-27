#include <avt_341_nav/planning/local/dwa/path.hpp>

namespace avt_341_nav {
namespace planning {
namespace dwa {

Path::Path() {}

void Path::Add(double x, double y) {
    x_.push_back(x);
    y_.push_back(y);
}

int Path::FindClosestDistance(double x, double y) const {
    double d_min = std::numeric_limits<double>::infinity();

    for(int i = 0; i < (int)x_.size(); ++i) {
        double d_curr = std::hypot(x - x_[i], y - y_[i]);

        if(d_curr < d_min) { d_min = d_curr; }
    }

    return d_min;
}

} // namespace dwa
} // namespace planning
} // namespace avt_341_nav