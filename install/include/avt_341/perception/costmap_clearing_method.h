#ifndef AVT_341_COSTMAP_CLEARING_METHOD_H
#define AVT_341_COSTMAP_CLEARING_METHOD_H

#include <bitset>
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/elevation_grid_cell.h"
#include "avt_341/avt_341_utils.h"


namespace avt_341 {
namespace perception {

struct CostmapClearMethodType
{
public:
  const static std::string None;
  const static std::string Time;
  const static std::string Raytrace;
  const static std::string RaytraceWithFiltering;
  const static std::string NoObsTime;
};

struct TimedNoObsClearingSettings {
  double time_threshold;
  int sample_threshold;

  TimedNoObsClearingSettings(double time_threshold, int sample_threshold)
    : time_threshold(time_threshold), sample_threshold(sample_threshold) {}
};

struct RaytraceSettings{
  float llx;
  float lly;
  float res;
  int grid_dilate_x;
  int grid_dilate_y;
  float thresh;
  float raytrace_range;
  bool immediate_clear_dilation;
  bool use_voxels;
  float voxel_height_min;
  float voxel_height_res;
  float obj_range_filter;

  RaytraceSettings(float llx, float lly, float res, int gridDilateX, int gridDilateY, float thresh, float raytraceRange,
                   bool immediateClearDilation, bool useVoxels, float voxelHeightMin, float voxelHeightRes, float obj_range_filter)
                   : llx(llx), lly(lly), res(res), grid_dilate_x(gridDilateX), grid_dilate_y(gridDilateY), thresh(thresh),
                     raytrace_range(raytraceRange), immediate_clear_dilation(immediateClearDilation), use_voxels(useVoxels),
                     voxel_height_min(voxelHeightMin), voxel_height_res(voxelHeightRes), obj_range_filter(obj_range_filter) {}
};

class OccupancyClearingMethod{

public:
  OccupancyClearingMethod(std::vector< std::vector<Cell>> & costmap_cells, const RaytraceSettings & config, CellObstacleCalculator* obs_calculator);
  OccupancyClearingMethod(std::vector< std::vector<Cell>> & costmap_cells, float visualization_range, bool visualize, const RaytraceSettings & config, CellObstacleCalculator* obs_calculator);
  OccupancyClearingMethod(std::vector< std::vector<Cell>> & costmap_cells, int Ny, int Nx, float visualization_range, bool visualize, const RaytraceSettings & config, CellObstacleCalculator* obs_calculator);
  virtual ~OccupancyClearingMethod() = default;

  /**
  * Clear occupancy for the given point cloud.
  * @param point_cloud Point cloud being processed.
  */
  virtual void ClearOccupancy(const avt_341::msg::PointCloud &point_cloud) = 0;

  /**
  * Invokes any visualization that clearing method may have.
  */
  virtual void Visualize() const {};

  /**
   * Called after occupancy has beed added to the costmap for the input point cloud.
   * @param point_cloud Point cloud that has been processed.
   */
  virtual void OnOccupancyAdded(const avt_341::msg::PointCloud &point_cloud) {};

  /**
   * Resets the clearing method's local state.
   */
  virtual void Reset() {};

protected:
  void RemoveDilationAtCell(int x, int y, std::vector< std::vector<Cell>> & cells);

  bool visualize_;
  float visualization_range_;
  int Nx_;
  int Ny_;
  std::vector< std::vector<Cell>> & cells_;
  CellObstacleCalculator* cell_obstacle_calculator_;
  RaytraceSettings config_;
};

class NullClearingMethod : public OccupancyClearingMethod{
public:
  NullClearingMethod(std::vector< std::vector<Cell>> & cells, float visualization_range, bool visualize, const RaytraceSettings & config, CellObstacleCalculator* obs_calculator);
  void ClearOccupancy(const avt_341::msg::PointCloud &point_cloud) override;
};

class TimedClearingMethod: public OccupancyClearingMethod {

public:
  TimedClearingMethod(float max_point_age, std::vector< std::vector<Cell>> & cells, float visualization_range, bool visualize, const RaytraceSettings & config, CellObstacleCalculator* obs_calculator);
  void ClearOccupancy(const avt_341::msg::PointCloud &point_cloud) override;
  void AgeCells(const float dt);
private:
  float max_point_age_;
  double last_timestamp_ = -1.0;
};

class RaytraceClearingMethod: public OccupancyClearingMethod{

public:
  const static int N_VOXELS_PER_CELL = 1024;

  RaytraceClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & cells,
                         float visualization_range, bool visualize, const RaytraceSettings & config, CellObstacleCalculator* obs_calculator, bool handle_dilation=true);
  RaytraceClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & cells, int Nx, int Ny,
                         float visualization_range, bool visualize, const RaytraceSettings & config, CellObstacleCalculator* obs_calculatorr, bool handle_dilation=true);
  virtual ~RaytraceClearingMethod() override;
  void ClearOccupancy(const avt_341::msg::PointCloud &point_cloud) override;
  void Visualize() const override;

protected:
  avt_341::msg::Point TfTransformToPoint(const avt_341::msg::TransformStamped & transform) const;
  avt_341::msg::Point GetSensorOrigin(const avt_341::msg::Time & stamp) const;
  avt_341::msg::Point GetSensorOrigin() const;
  void GetGridBounds(const avt_341::msg::Point & origin, float range, int & x_0, int & y_0, int & x_N, int & y_N) const;
  avt_341::msg::Marker GetMarkerMsg(int type, int id, utils::vec3 color, float alpha=1.0, double z_scale=1.0) const;
  virtual void RaytraceLine(const avt_341::msg::Point & start, const avt_341::msg::Point32 & end);
  void CleanupUnattachedDilation(const avt_341::msg::Point & origin, std::vector< std::vector<Cell>> & cells);
  void ClearVoxelAt(int x, int y, int z);
  void Reset() override;

  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::MarkerArray>> minmax_vis_publisher_;
  std::shared_ptr<avt_341::node::NodeProxy> node_;
  std::string lidar_frame_;
  std::bitset<N_VOXELS_PER_CELL>* voxel_grid;
  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::MarkerArray>> voxel_vis_publisher_;
  bool handle_dilation_;
};

class RaytraceWithFilteringClearingMethod: public RaytraceClearingMethod{

public:
  RaytraceWithFilteringClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & cells,
  float visualization_range, bool visualize, const RaytraceSettings & config, CellObstacleCalculator* obs_calculator);

  void ClearOccupancy(const avt_341::msg::PointCloud &point_cloud) override;
  void OnOccupancyAdded(const avt_341::msg::PointCloud &point_cloud) override;
  void Visualize() const override;
  void Reset() override;

protected:
  std::vector< std::vector<Cell>> & cells_without_clearing_;
  std::vector< std::vector<Cell>> cells_with_clearing_;
  std::vector< std::vector<bool>> occupancy_delta_;
  utils::vec2 last_position_;
  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::OccupancyGrid>> occupancy_delta_publisher_;

};

struct TimedNoObsData {
  double obs_time;
  int num_samples;
};

class TimedNoObsClearingMethod: public OccupancyClearingMethod {
public:
  TimedNoObsClearingMethod(std::vector< std::vector<Cell>> & cells, float visualization_range,
                           bool visualize, const TimedNoObsClearingSettings & time_config, const RaytraceSettings & config, CellObstacleCalculator* obs_calculator);
  void ClearOccupancy(const avt_341::msg::PointCloud &point_cloud) override;
  void OnOccupancyAdded(const avt_341::msg::PointCloud &point_cloud) override;
  void Reset() override;
private:
  std::vector< std::vector<Cell>> timed_cells_;
  std::vector< std::vector<TimedNoObsData>> timed_cells_data;
  TimedNoObsClearingSettings time_config_;
};

}
}
#endif //AVT_341_COSTMAP_CLEARING_METHOD_H
