/**
* @file      occupancy_grid_utils.hpp
* @brief     Helpers operating on nav_msgs OccupancyGrid messages.
*/

#ifndef AVT_341_CORE_OCCUPANCY_GRID_UTILS_H
#define AVT_341_CORE_OCCUPANCY_GRID_UTILS_H

#include <algorithm>
#include <cmath>
#include <optional>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace avt_341_nav::core
{

/// Axis-aligned world-space bounding box.
struct WorldAabb {
    double x_min;
    double y_min;
    double x_max;
    double y_max;
};

/// Bounding box over two points, expanded by padding meters on all sides.
inline WorldAabb MakePaddedAabb(
    const double x0, const double y0, const double x1, const double y1, const double padding)
{
    return WorldAabb{
        std::min(x0, x1) - padding,
        std::min(y0, y1) - padding,
        std::max(x0, x1) + padding,
        std::max(y0, y1) + padding};
}

/// Crops src to the intersection of box with the extent of src. The crop origin is snapped to
/// the src cell boundaries so retained cells keep their exact world coordinates. Returns
/// std::nullopt when src is empty or the intersection is degenerate.
inline std::optional<nav_msgs::msg::OccupancyGrid> CropGridToWorldAabb(
    const nav_msgs::msg::OccupancyGrid& src, const WorldAabb& box)
{
    const double res = src.info.resolution;
    const int src_width = static_cast<int>(src.info.width);
    const int src_height = static_cast<int>(src.info.height);
    if (src_width <= 0 || src_height <= 0 || res <= 0.0) {
        return std::nullopt;
    }

    const double src_llx = src.info.origin.position.x;
    const double src_lly = src.info.origin.position.y;

    const int start_col = std::max(0, static_cast<int>(std::floor((box.x_min - src_llx) / res)));
    const int start_row = std::max(0, static_cast<int>(std::floor((box.y_min - src_lly) / res)));
    const int end_col = std::min(src_width, static_cast<int>(std::ceil((box.x_max - src_llx) / res)));
    const int end_row = std::min(src_height, static_cast<int>(std::ceil((box.y_max - src_lly) / res)));

    const int crop_width = end_col - start_col;
    const int crop_height = end_row - start_row;
    if (crop_width <= 0 || crop_height <= 0) {
        return std::nullopt;
    }

    nav_msgs::msg::OccupancyGrid crop;
    crop.header = src.header;
    crop.info.map_load_time = src.info.map_load_time;
    crop.info.resolution = res;
    crop.info.width = crop_width;
    crop.info.height = crop_height;
    crop.info.origin.position.x = src_llx + start_col * res;
    crop.info.origin.position.y = src_lly + start_row * res;
    crop.info.origin.position.z = src.info.origin.position.z;
    crop.info.origin.orientation.w = 1.0;

    crop.data.resize(static_cast<std::size_t>(crop_width) * crop_height);
    for (int r = 0; r < crop_height; r++) {
        const auto src_row_begin =
            src.data.begin() + static_cast<std::size_t>(start_row + r) * src_width + start_col;
        std::copy(src_row_begin, src_row_begin + crop_width,
                  crop.data.begin() + static_cast<std::size_t>(r) * crop_width);
    }

    return crop;
}

}

#endif
