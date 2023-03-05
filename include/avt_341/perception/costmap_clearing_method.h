#ifndef AVT_341_COSTMAP_CLEARING_METHOD_H
#define AVT_341_COSTMAP_CLEARING_METHOD_H

#include <bitset>
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/elevation_grid_cell.h"
#include "avt_341/avt_341_utils.h"


namespace avt_341 {
namespace perception {

enum CostmapClearMethodType
{
  None,
  Time,
  Raytrace,
  RaytraceWithFiltering
};

struct RaytraceSettings{
  float llx;
  float lly;
  float res;
  int grid_dilate_x;
  int grid_dilate_y;
  float thresh;
  float raytrace_range;
  bool clear_dilation;
  bool use_voxels;
  float voxel_height_min;
  float voxel_height_res;

  RaytraceSettings(float llx, float lly, float res, int gridDilateX, int gridDilateY, float thresh, float raytraceRange,
                   bool clearDilation, bool useVoxels, float voxelHeightMin, float voxelHeightRes)
                   : llx(llx), lly(lly), res(res), grid_dilate_x(gridDilateX), grid_dilate_y(gridDilateY), thresh(thresh),
                   raytrace_range(raytraceRange), clear_dilation(clearDilation), use_voxels(useVoxels),
                   voxel_height_min(voxelHeightMin), voxel_height_res(voxelHeightRes) {}
};

class OccupancyClearingMethod{

public:
  OccupancyClearingMethod(std::vector< std::vector<Cell>> & costmap_cells, int Nx, int Ny, float visualization_range, bool visualize);
  virtual ~OccupancyClearingMethod() = default;

  virtual void ClearOccupancy(const avt_341::msg::PointCloud &point_cloud) = 0;
  virtual void Visualize() const {};
  virtual void OnOccupancyAdded() {};

  static CostmapClearMethodType string_to_clear_type(const std::string & val) {
    if(val == "none"){ return CostmapClearMethodType::None; }
    if(val == "time"){ return CostmapClearMethodType::Time; }
    if(val == "raytrace"){ return CostmapClearMethodType::Raytrace; }
    if(val == "raytrace_with_filter"){ return CostmapClearMethodType::RaytraceWithFiltering; }
    throw std::runtime_error("Unknown costmap clearing type " + val);
  }

protected:
  bool visualize_;
  float visualization_range_;
  int Nx_;
  int Ny_;
  std::vector< std::vector<Cell>> & cells_;

};

class NullClearingMethod : public OccupancyClearingMethod{
public:
  NullClearingMethod(std::vector< std::vector<Cell>> & cells, float visualization_range, bool visualize);
  void ClearOccupancy(const avt_341::msg::PointCloud &point_cloud) override;
};

class TimedClearingMethod: public OccupancyClearingMethod {

public:
  TimedClearingMethod(float max_point_age, std::vector< std::vector<Cell>> & cells, float visualization_range, bool visualize);
  void ClearOccupancy(const avt_341::msg::PointCloud &point_cloud) override;
  void AgeCells();
private:
  float max_point_age_;
};

class RaytraceClearingMethod: public OccupancyClearingMethod{

public:
  RaytraceClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & cells,
                         float visualization_range, bool visualize, RaytraceSettings settings, CellObstacleCalculator* cell_obstacle_calculator, bool handle_dilation=true);
  RaytraceClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & cells, int Nx, int Ny,
                         float visualization_range, bool visualize, RaytraceSettings settings, CellObstacleCalculator* cell_obstacle_calculator, bool handle_dilation=true);
  virtual ~RaytraceClearingMethod() override;
  void ClearOccupancy(const avt_341::msg::PointCloud &point_cloud) override;
  void Visualize() const override;

protected:
  avt_341::msg::Point GetSensorOrigin() const;
  void GetGridBounds(const avt_341::msg::Point & origin, float range, int & x_0, int & y_0, int & x_N, int & y_N) const;
  avt_341::msg::Marker GetMarkerMsg(int type, int id, utils::vec3 color, float alpha=1.0, double z_scale=1.0) const;
  virtual void RaytraceLine(const avt_341::msg::Point & start, const avt_341::msg::Point32 & end);
  void RemoveDilationAtCell(int x, int y, std::vector< std::vector<Cell>> & cells);
  void CleanupUnattachedDilation(const avt_341::msg::Point & origin, std::vector< std::vector<Cell>> & cells);
  void ClearVoxelAt(int x, int y, int z);

  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::MarkerArray>> minmax_vis_publisher_;
  std::shared_ptr<avt_341::node::NodeProxy> node_;
  CellObstacleCalculator* cell_obstacle_calculator_;
  RaytraceSettings config_;
  const static int N_VOXELS_PER_CELL = 1024;
  std::bitset<N_VOXELS_PER_CELL>* voxel_grid;
  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::MarkerArray>> voxel_vis_publisher_;
  bool handle_dilation_;
};

class RaytraceWithFilteringClearingMethod: public RaytraceClearingMethod{

public:
  RaytraceWithFilteringClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & cells,
  float visualization_range, bool visualize, RaytraceSettings settings, float obj_range_filter, CellObstacleCalculator* cell_obstacle_calculator);

  void ClearOccupancy(const avt_341::msg::PointCloud &point_cloud) override;
  void OnOccupancyAdded() override;
  void Visualize() const override;

protected:
  float obj_filter_range_;
  std::vector< std::vector<Cell>> & cells_without_clearing_;
  std::vector< std::vector<Cell>> cells_with_clearing_;
  std::vector< std::vector<bool>> occupancy_delta_;
  utils::vec2 last_position_;
  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::OccupancyGrid>> occupancy_delta_publisher_;

};

}
}
#endif //AVT_341_COSTMAP_CLEARING_METHOD_H
