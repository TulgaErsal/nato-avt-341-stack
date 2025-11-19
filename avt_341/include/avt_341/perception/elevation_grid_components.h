#ifndef ELEVATION_GRID_COMPONENTS_H
#define ELEVATION_GRID_COMPONENTS_H

#include <vector>
#include <limits>
#include <string>

namespace avt_341{
    namespace perception{

        struct GridPubMethod {
            static const std::string Full;
            static const std::string Window;
            static const std::string Updates;
            static bool IsGridPubMethodValid(const std::string & selected_method);
        };

        struct GridRegion {

            GridRegion(int x_min, int x_max, int y_min, int y_max);
            GridRegion();

            GridRegion Dilate(int dilate_x, int dilate_y, int nx, int ny) const;
            void UpdateBounds(int x, int y);
            void Reset();

            inline bool HasData() const {return x_min < x_max; }
            inline int Width() const {return HasData() ? x_max - x_min : 0; }
            inline int Height() const {return HasData() ? y_max - y_min : 0; }
            int x_min;
            int x_max;
            int y_min;
            int y_max;
        };


    }
}

#endif //ELEVATION_GRID_COMPONENTS_H
