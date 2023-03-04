/**
 * \class ElevationGrid
 *
 * A slope-based obstacle detection algorithm. 
 * The world is divided into 2D cells. 
 * The highest and lowest point are used to calculate the slope in each cell.
 * Cells that exceed a slope threshold are flagged as obstcles.
 *
 * \author Chris Goodin
 *
 * \date 9/3/2020
 */
#include <vector>
#include <limits>
#include <string>
#include "avt_341/node/ros_types.h"
#include "avt_341/perception/elevation_grid_cell.h"
#include "avt_341/perception/costmap_clearing_method.h"

namespace avt_341{
namespace perception{

class ElevationGrid : public CellObstacleCalculator{
  public:
    ElevationGrid();

    ~ElevationGrid() override;

    /**
     * Add points to be processed 
     * Modifies the input to be only obstacle points
     * Returns surface points
     * \param point_cloud PointCloud message
     */
    std::vector<avt_341::msg::Point32> AddPoints(avt_341::msg::PointCloud &point_cloud, const avt_341::msg::Odometry & current_pose);

    bool has_segmentation() const { return has_segmentation_; }

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

    void SetCostmapClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, const std::string & clearing_method_type,
                                  float visualization_range, bool visualize, float clear_method_raytrace_range, bool clear_method_clear_dilation,
                                  bool use_voxels, float voxel_height_min, float voxel_height_res, float obj_range_filter){
      auto clear_type = OccupancyClearingMethod::string_to_clear_type(clearing_method_type);
      int dsize_x = lround(grid_dilate_x_/res_);
      int dsize_y = lround(grid_dilate_y_/res_);
      RaytraceSettings raytrace_settings(llx_, lly_, res_, dsize_x, dsize_y, thresh_, clear_method_raytrace_range,
                                         clear_method_clear_dilation, use_voxels, voxel_height_min, voxel_height_res);

      switch(clear_type){
        case CostmapClearMethodType::Time:
          clearing_method_ = std::make_shared<TimedClearingMethod>(max_point_age_, cells_, visualization_range, visualize);
          break;
        case CostmapClearMethodType::Raytrace:
          if(!dilate_ || grid_dilate_x_ <= 0 || grid_dilate_y_ <= 0){
            node_ref->log_warning("Raytrace Clearing: Dilation should be enabled with dilation size > 0 to reduce intermittent obstacle.");
          }
          clearing_method_ = std::make_shared<RaytraceClearingMethod>(node_ref, cells_, visualization_range, visualize, raytrace_settings, this);
          break;
        case CostmapClearMethodType::RaytraceWithFiltering:
          if(!dilate_ || grid_dilate_x_ <= 0 || grid_dilate_y_ <= 0){
            node_ref->log_warning("Raytrace Clearing: Dilation should be enabled with dilation size > 0 to reduce intermittent obstacle.");
          }
          clearing_method_ = std::make_shared<RaytraceWithFilteringClearingMethod>(node_ref, cells_, visualization_range, visualize, raytrace_settings, obj_range_filter, this);
        default:
          clearing_method_ = std::make_shared<NullClearingMethod>(cells_, visualization_range, visualize);
      }
    }

    void Visualize(const avt_341::msg::Odometry & odom) const{
      clearing_method_->Visualize(odom);
    }

  bool PastSlopeThreshold(const Cell &cell) const override;
  float Slope(const Cell &cell) const override;
  void AddOccupancy(const avt_341::msg::PointCloud &point_cloud, std::vector< std::vector<Cell> > & cells, bool dilate) override;

  void SetMaxPointAge(float mpa){
        max_point_age_ = mpa;
    }

    void SetSlopeThreshold(float tr){
        thresh_ = tr;
    }

    void SetStitchPoints(bool stitch_points){ stitch_points_ = stitch_points; }

    void SetFilterHighest(bool filter_high){ filter_highest_ = filter_high; }

    void SetUseElevation(bool use_elevation){
        use_elevation_ = use_elevation;
    }

    void ClearGrid();

    void UseDilation(bool use_dil){
        dilate_ = use_dil;
    }

    avt_341::msg::OccupancyGrid GetGrid(bool is_segmentation=false);

    void SetCorner(float llx, float lly){
        llx_ = llx;
        lly_ = lly;
    }

    void SetDilation(bool grid_dilate, float grid_dilate_x, float grid_dilate_y, float grid_dilate_proportion){
        dilate_ = grid_dilate;
        grid_dilate_x_ = grid_dilate_x;
        grid_dilate_y_ = grid_dilate_y;
        grid_dilate_proportion_ = grid_dilate_proportion;
    }


  private:
    uint8_t GetGridCellValue(const Cell & cell) const;
    void ResizeGrid();
    std::vector< std::vector<Cell> > cells_;
    float width_;
    float height_;
    float res_;
    float thresh_;
    int nx_,ny_;
    bool first_display_;
    bool dilate_;
    float llx_;
    float lly_;
    float grid_dilate_x_;
    float grid_dilate_y_;
    float grid_dilate_proportion_;
    bool use_elevation_;
    bool stitch_points_;
    bool filter_highest_;
    const uint8_t GRID_MAX_VALUE = 100;
    const float GRID_SLOPE_MULT = 50.0f;
    bool has_segmentation_ = false;
    float max_point_age_;
    std::shared_ptr<OccupancyClearingMethod> clearing_method_ = nullptr;

};

} // namespace perception
} // namespace avt_341