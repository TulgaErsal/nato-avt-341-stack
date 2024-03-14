/**
 * Simple local occupancy grid implementation
 * 
 * Evan Vandermate - evanderm@mtu.edu
*/
#include "avt_341/perception/local_occupancy_grid.h"

namespace avt_341 {
namespace perception{

LocalOccupancyGrid::LocalOccupancyGrid(std::string frame, float width, float height, float resolution, float dilate_x, float dilate_y) {
    frame_ = frame;
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    dilate_x_ = dilate_x;
    dilate_y_ = dilate_y;
    ClearGrid();
}

void LocalOccupancyGrid::ClearGrid() {
    nx_ = (int)ceil(width_/resolution_);
    ny_ = (int)ceil(height_/resolution_);
    cells_.clear();
    std::vector<uint8_t> row;
    row.resize(ny_);
    cells_.resize(nx_,row);
}

avt_341::msg::OccupancyGrid LocalOccupancyGrid::GetGrid() {
    avt_341::msg::OccupancyGrid grid;
    grid.header.frame_id = frame_;
    grid.info.resolution = resolution_;
    grid.info.width = nx_;
    grid.info.height = ny_;
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.orientation.w = 1.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    //grid.data.resize(nx_*ny_);

    for (auto& row : cells_) {
        grid.data.insert(std::end(grid.data), std::begin(row), std::end(row));
    }
    return grid;
}

uint8_t LocalOccupancyGrid::GetOccupancy(float map_x, float map_y) {
    int grid_x, grid_y;
    MapToGrid(map_x, map_y, grid_x, grid_y);
    if (grid_x < 0 || grid_x > nx_ || grid_y < 0 || grid_y > ny_) {
        return 0;
    }
    return cells_[grid_x][grid_y];
}

void LocalOccupancyGrid::UpdateOrigin(int x, int y) {
    origin_x_ = x;
    origin_y_ = y;
}

void LocalOccupancyGrid::AddPoints(avt_341::msg::PointCloud2 &point_cloud, bool dilate) {
    // Check for matching frame
    if (point_cloud.header.frame_id != frame_) {
        std::cout << "LocalOccupancyGrid: pointcloud frame [" << point_cloud.header.frame_id.c_str() 
                  << "] does not match local grid frame [" << frame_.c_str() << "], no occupancy added.\n";
        return;
    }

    // Clear old occupancy
    ClearGrid();

    // Convert pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(point_cloud, *cloud);

    // Process points
    for (auto point : cloud->points){
        // Calculate grid coordinates
        int grid_x, grid_y;
        MapToGrid(point.x, point.y, grid_x, grid_y);

        // Add occupancy at point location
        if (grid_x >= 0 && grid_x < nx_ && grid_y >= 0 && grid_y < ny_) {
            cells_[grid_x][grid_y] = MAX_OCCUPANCY;
        }

        // Apply dilation
        if (dilate) {
            int kernel_size_x = (int)(dilate_x_/resolution_);
            int kernel_size_y = (int)(dilate_y_/resolution_);
            int dilation_min_x = std::max(0, grid_x-kernel_size_x/2);
            int dilation_max_x = std::min(nx_, grid_x+kernel_size_x/2);
            int dilation_min_y = std::max(0, grid_y-kernel_size_y/2);
            int dilation_max_y = std::min(ny_, grid_y+kernel_size_y/2);
            for (int dx = dilation_min_x; dx < dilation_max_x; dx++) {
                for (int dy = dilation_min_y; dy < dilation_max_y; dy++) {
                    cells_[dx][dy] = MAX_OCCUPANCY;
                }
            }
        }
    }
}

void LocalOccupancyGrid::MapToGrid(float map_x, float map_y, int &grid_x, int &grid_y) {
    grid_x = (int)((map_y-origin_y_)/resolution_);
    grid_y = (int)((map_x-origin_x_)/resolution_);
}

} // namespace perception
} // namespace avt_341