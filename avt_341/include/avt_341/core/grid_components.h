#ifndef GRID_COMPONENTS_H
#define GRID_COMPONENTS_H

namespace avt_341::core{

struct GridRegion {

    GridRegion(int x_min, int x_max, int y_min, int y_max);
    GridRegion();

    GridRegion Dilate(int dilate_x, int dilate_y, int nx, int ny) const;
    void UpdateBounds(int x, int y);
    void UpdateBounds(const int x, const int y, const int width, const int height);

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

#endif //GRID_COMPONENTS_H
