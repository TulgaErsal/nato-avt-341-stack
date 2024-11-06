#pragma once

#include <vector>
#include <limits>
#include <string>
#include <iostream>
#include <thread>
#include <math.h>

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include "avt_341/node/ros_types.h"
#include "avt_341/avt_341_utils.h"


namespace avt_341{
namespace perception{

class NormalCell{
public:
    NormalCell(){
        size_ = 1.0f;
        normal.x = 0.0f; 
        normal.y = 0.0f;
        normal.z = 0.0f;
        point_count = 0;
        threshold = 0.5f;
        default_val = 0;
    }
    NormalCell(float size, avt_341::utils::vec3 norm, int count, float thresh, uint8_t default_value){
        size_ = size;
        normal = norm;
        point_count = count;
        threshold = thresh;
        default_val = default_value;
    }

    void ResetNorm(){
        normal.x = 0.0f; 
        normal.y = 0.0f;
        normal.z = 0.0f;
        point_count = 0;
    }

    void AddNormal(float nx, float ny, float nz) {
        normal.x = ((normal.x*point_count)+nx) / (point_count+1);
        normal.y = ((normal.y*point_count)+ny) / (point_count+1);
        normal.z = ((normal.z*point_count)+nz) / (point_count+1);
        point_count++;
    }

    // [0-100], 0 -> high confidence of low-traversability, 100 -> high confidence of high-traversibility
    uint8_t Value() const {
        if (!this->filled()) {
            return default_val;
        }
        uint8_t cell_val = std::sqrt(normal.x*normal.x + normal.y*normal.y + (normal.z-1.0f)*(normal.z-1.0f)) * 50.0f;
        cell_val = 100 - std::min(std::max((uint8_t)(cell_val/threshold), (uint8_t)0), (uint8_t)100);
        return cell_val;
    }

    operator uint8_t() const {
        return this->Value();
    }

    inline avt_341::utils::vec3 Normal() const { return normal; }
    inline bool filled() const { return point_count > 0; }

    float size_;
    avt_341::utils::vec3 normal;
    int point_count;
    float threshold;
    uint8_t default_val;
};

class NormalGrid {
  public:
    NormalGrid();

    ~NormalGrid();

    /**
     * Add points to be processed
     * \param point_cloud PointCloud message
     */
    void AddPoints(pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud);

    void SetSize(float s){
        width_ = s;
        height_ = s;
        ResizeGrid();
    }

    void SetSize(float width,float height){
        width_ = width;
        height_ = height;
        ResizeGrid();
    }

    void SetRes(float r){
        res_ = r;
        ResizeGrid();
    }

    void SetThreshold(float thresh) {
        thresh_ = thresh;
    }

    void AddOccupancy(pcl::PointCloud<pcl::PointNormal>::Ptr pc_normals, std::vector< std::vector<NormalCell> > & cells);

    void ClearGrid();

    avt_341::msg::OccupancyGrid GetGrid(bool is_segmentation=false);

    avt_341::msg::OccupancyGrid GetGrid(double x, double y, double width, double height);

    void SetCorner(float llx, float lly){
        llx_ = llx;
        lly_ = lly;
    }

    void Reset();
    bool HasData() const;

  private:
    uint8_t GetGridCellValue(const NormalCell & cell) const;
    void ResizeGrid();
    std::vector< std::vector<NormalCell> > cells_;
    float width_;
    float height_;
    float res_;
    float thresh_;
    int nx_,ny_;
    float llx_;
    float lly_;
    const uint8_t GRID_MAX_VALUE = 100;
    const float GRID_SLOPE_MULT = 50.0f;

};

} // namespace perception
} // namespace avt_341