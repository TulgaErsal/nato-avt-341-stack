#include "avt_341_nav/core/grid_components.h"
#include <algorithm>
#include <limits>
#include <vector>

namespace avt_341_nav::core{

GridRegion::GridRegion(int x_min, int x_max, int y_min, int y_max)
    : x_min(x_min), x_max(x_max), y_min(y_min), y_max(y_max) {
}

GridRegion::GridRegion() {
    Reset();
}

GridRegion GridRegion::Dilate(int dilate_x, int dilate_y, int nx, int ny) const {
    return {
        std::max(0, x_min - dilate_x),
        std::min(nx, x_max + dilate_x),
        std::max(0, y_min - dilate_y),
        std::min(ny, y_max + dilate_y)
        };
}

void GridRegion::UpdateBounds(const GridRegion & other) {
    if (!other.HasData()){
        return;
    }
    UpdateBounds(other.x_min, other.y_min);
    UpdateBounds(other.x_max-1, other.y_max-1);
}

void GridRegion::UpdateBounds(const int x, const int y, const int width, const int height) {
    UpdateBounds(x, y);
    UpdateBounds(x+width-1, y+height-1);
}

void GridRegion::UpdateBounds(const int x, const int y) {
    // Exclusive range for max index values
    x_min = std::min(x_min, x);
    x_max = std::max(x_max, x+1);
    y_min = std::min(y_min, y);
    y_max = std::max(y_max, y+1);
}

void GridRegion::Reset() {
    x_min = std::numeric_limits<int>::max();
    x_max = std::numeric_limits<int>::lowest();
    y_min = std::numeric_limits<int>::max();
    y_max = std::numeric_limits<int>::lowest();
}

}