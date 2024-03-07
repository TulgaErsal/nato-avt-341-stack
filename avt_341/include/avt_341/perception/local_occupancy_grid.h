/**
 * Simple local occupancy grid implementation
 * 
 * Evan Vandermate - evanderm@mtu.edu
*/
#ifndef LOCALOCCUPANCYGRID_H
#define LOCALOCCUPANCYGRID_H
#include <string>
#include <math.h>
#include <vector>
#include <iostream>
#include "avt_341/node/ros_types.h"

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

namespace avt_341 {
namespace perception{

class LocalOccupancyGrid {
public:
    LocalOccupancyGrid(std::string frame, float width, float height, float resolution, float dilate_x, float dilate_y);
    
    /**
     * Adds pre-processed (only obstacle points present) point cloud occupancy to the grid
    */
    void AddPoints(avt_341::msg::PointCloud2 &point_cloud, bool dilate);

    /**
     * Converts stored map to OccupancyGrid message
    */
    avt_341::msg::OccupancyGrid GetGrid();

    /** 
     * Update map origin
    */
    void UpdateOrigin(int x, int y);

private:
    void ClearGrid();
    void MapToGrid(float map_x, float map_y, int &grid_x, int &grid_y);

    uint8_t MAX_OCCUPANCY = 100;
    std::string frame_;
    float width_, height_, resolution_, dilate_x_, dilate_y_;
    int nx_, ny_;
    int origin_x_ = 0;
    int origin_y_ = 0;
    std::vector<std::vector<uint8_t>> cells_;

};

} // namespace perception
} // namespace avt_341


#endif