#include "avt_341/perception/elevation_grid_components.h"

namespace avt_341{
    namespace perception{

    GridUpdateRegion::GridUpdateRegion(int x_min, int x_max, int y_min, int y_max)
        : x_min(x_min), x_max(x_max), y_min(y_min), y_max(y_max) {
    }

    GridUpdateRegion::GridUpdateRegion() {
        Reset();
    }

    GridUpdateRegion GridUpdateRegion::Dilate(int dilate_x, int dilate_y, int nx, int ny) const {
        return {
            std::max(0, x_min - dilate_x),
            std::min(nx, x_max + dilate_x),
            std::max(0, y_min - dilate_y),
            std::min(ny, y_max + dilate_y)
            };
    }

    void GridUpdateRegion::Update(const int x, const int y) {
        x_min = std::min(x_min, x);
        x_max = std::max(x_max, x+1);
        y_min = std::min(y_min, y);
        y_max = std::max(y_max, y+1);
    }

    void GridUpdateRegion::Reset() {
        x_min = std::numeric_limits<int>::max();
        x_max = std::numeric_limits<int>::lowest();
        y_min = std::numeric_limits<int>::max();
        y_max = std::numeric_limits<int>::lowest();
    }

    const std::string GridPubMethod::Full = "full";
    const std::string GridPubMethod::Window = "window";
    const std::string GridPubMethod::Updates = "updates";

    bool GridPubMethod::IsGridPubMethodValid(const std::string & selected_method){
        std::vector<std::string> valid_methods = {Full, Window, Updates};
        return std::find(valid_methods.begin(), valid_methods.end(), selected_method) != valid_methods.end();
    }

    }
}