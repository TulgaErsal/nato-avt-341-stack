#ifndef AVT_341_COSTMAP_CLEARING_METHOD_H
#define AVT_341_COSTMAP_CLEARING_METHOD_H

#include <bitset>
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "elevation_grid.h"

namespace avt_341 {
namespace perception {

class CostmapClearingMethod{

public:
  CostmapClearingMethod(std::vector< std::vector<Cell>> & costmap_cells, bool visualize);

  virtual void Apply(const avt_341::msg::PointCloud &point_cloud){}
  virtual void Visualize() const{}

protected:
  bool visualize_;
  unsigned int Nx_;
  unsigned int Ny_;
  std::vector< std::vector<Cell>> & costmap_cells_;

};

class TimedClearingMethod: public CostmapClearingMethod {

public:
  TimedClearingMethod(float max_point_age, std::vector< std::vector<Cell>> & costmap_cells, bool visualize);

  void Apply(const avt_341::msg::PointCloud &point_cloud) override;

private:
  float max_point_age_;

}

class RaytraceClearingMethod: public CostmapClearingMethod{

public:
  RaytraceClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & costmap_cells,  bool visualize);

  void Bresenham3D(int off_a, int off_b, int off_c,
                                           unsigned int abs_da, unsigned int abs_db, unsigned int abs_dc,
                                           int error_b, int error_c, int offset_a, int offset_b, int offset_c, unsigned int &offset,
                                           unsigned int &z_mask, unsigned int max_length = UINT_MAX);
  void Bresenham2D();
  void Apply(const avt_341::msg::PointCloud &point_cloud) override;
  void Visualize() const override;
protected:

  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::MarkerArray>> minmax_vis_publisher_;

};

class VoxelRaytraceClearingMethod: public RaytraceClearingMethod{

public:
  VoxelRaytraceClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & costmap_cells, bool visualize);

  void Apply(const avt_341::msg::PointCloud &point_cloud) override;
  void Visualize() const override;

protected:
  std::bitset<1024>* voxel_grid;
  std::shared_ptr<avt_341::node::Publisher<avt_341::msg::MarkerArray>> voxel_vis_publisher_;

};






}
}
#endif //AVT_341_COSTMAP_CLEARING_METHOD_H
