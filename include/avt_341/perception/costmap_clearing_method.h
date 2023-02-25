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
  VoxelRaytrace
};

class CostmapClearingMethod{

public:
  CostmapClearingMethod(std::vector< std::vector<Cell>> & costmap_cells, float visualization_range, bool visualize);
  virtual ~CostmapClearingMethod() = default;

  virtual void Apply(const avt_341::msg::PointCloud &point_cloud, const avt_341::msg::Odometry & current_pose) = 0;
  virtual void Visualize(const avt_341::msg::Odometry & odom) const {};

  static CostmapClearMethodType string_to_clear_type(const std::string & val) {
    if(val == "none"){ return CostmapClearMethodType::None; }
    if(val == "time"){ return CostmapClearMethodType::Time; }
    if(val == "raytrace"){ return CostmapClearMethodType::Raytrace; }
    if(val == "raytrace_voxel"){ return CostmapClearMethodType::VoxelRaytrace; }
    throw std::runtime_error("Unknown costmap clearing type " + val);
  }

protected:
  bool visualize_;
  float visualization_range_;
  int Nx_;
  int Ny_;
  std::vector< std::vector<Cell>> & costmap_cells_;

};

class NullClearingMethod : public CostmapClearingMethod{
public:
  NullClearingMethod(std::vector< std::vector<Cell>> & costmap_cells, float visualization_range, bool visualize);
  void Apply(const avt_341::msg::PointCloud &point_cloud, const avt_341::msg::Odometry & current_pose) override;
};

class TimedClearingMethod: public CostmapClearingMethod {

public:
  TimedClearingMethod(float max_point_age, std::vector< std::vector<Cell>> & costmap_cells, float visualization_range, bool visualize);
  void Apply(const avt_341::msg::PointCloud &point_cloud, const avt_341::msg::Odometry & current_pose) override;
private:
  float max_point_age_;
};

class RaytraceClearingMethod: public CostmapClearingMethod{

public:
  RaytraceClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & costmap_cells, float visualization_range, bool visualize, float llx, float lly, float res);
  virtual ~RaytraceClearingMethod() override = default;
  void Apply(const avt_341::msg::PointCloud &point_cloud, const avt_341::msg::Odometry & current_pose) override;
  void Visualize(const avt_341::msg::Odometry & odom) const override;

protected:
  void GetVoxelBounds(const avt_341::msg::Odometry & odom, int & x_0, int & y_0, int & x_N, int & y_N) const;
  avt_341::msg::Marker GetMarkerMsg(int type, int id, utils::vec3 color, float alpha=1.0, double z_scale=1.0) const;
  virtual void RaytraceLine(const avt_341::msg::Point & start, const avt_341::msg::Point32 & end);
  void Bresenham3D(int off_a, int off_b, int off_c,
                   unsigned int abs_da, unsigned int abs_db, unsigned int abs_dc,
                   int error_b, int error_c, int offset_a, int offset_b, int offset_c, unsigned int &offset,
                   unsigned int &z_mask, unsigned int max_length = UINT_MAX);

  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::MarkerArray>> minmax_vis_publisher_;
  std::shared_ptr<avt_341::node::NodeProxy> node_;
  float llx_;
  float lly_;
  float res_;
};

class VoxelRaytraceClearingMethod: public RaytraceClearingMethod{

public:
  VoxelRaytraceClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & costmap_cells,
                              float visualization_range, bool visualize, float llx, float lly, float res, float voxel_height_min, float voxel_height_res);
  virtual ~VoxelRaytraceClearingMethod() override;
  void Apply(const avt_341::msg::PointCloud &point_cloud, const avt_341::msg::Odometry & current_pose) override;
  void Visualize(const avt_341::msg::Odometry & odom) const override;

protected:
  void RaytraceLine(const avt_341::msg::Point & start, const avt_341::msg::Point32 & end) override;
  void ClearVoxelAt(int x, int y, int z);
  const static int N_VOXELS_PER_CELL = 1024;
  std::bitset<N_VOXELS_PER_CELL>* voxel_grid;
  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::MarkerArray>> voxel_vis_publisher_;
  float voxel_height_min_;
  float voxel_height_res_;
};

}
}
#endif //AVT_341_COSTMAP_CLEARING_METHOD_H
