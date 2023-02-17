#include "avt_341/perception/costmap_clearing_method.h"

namespace avt_341{
namespace perception{

  CostmapClearingMethod::CostmapClearingMethod(std::vector<std::vector<Cell>> &costmap_cells, bool visualize)
                                              : costmap_cells_(costmap_cells), visualize_(visualize)
  {
    Nx_ = costmap_cells_.size();
    Ny_ = costmap_cells_[0].size();
  }

  TimedClearingMethod::TimedClearingMethod(float max_point_age, std::vector< std::vector<Cell>> & costmap_cells, bool visualize)
      : CostmapClearingMethod(costmap_cells, visualize), max_point_age_(max_point_age){
  }

  NullClearingMethod::NullClearingMethod(std::vector<std::vector<Cell>> &costmap_cells, bool visualize)
      : CostmapClearingMethod(costmap_cells, visualize) {
  }

  void NullClearingMethod::Apply(const msg::PointCloud &point_cloud) {
  }


  void TimedClearingMethod::Apply(const avt_341::msg::PointCloud &point_cloud){
    Cell empty_cell;
    for (int i=0; i<Nx_;i++){
      for (int j=0; j<Ny_; j++){
        if (costmap_cells_[i][j].low.age>max_point_age_ ||
            costmap_cells_[i][j].highest.age>max_point_age_ ||
            costmap_cells_[i][j].second_highest.age>max_point_age_ ||
            costmap_cells_[i][j].high.age>max_point_age_){
          costmap_cells_[i][j]=empty_cell;
      }
      if (costmap_cells_[i][j].dilated_val>0 && costmap_cells_[i][j].dilated_age>max_point_age_){
        costmap_cells_[i][j]=empty_cell;
      }
    }
  }
  }


  RaytraceClearingMethod::RaytraceClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & costmap_cells,  bool visualize)
    : CostmapClearingMethod(costmap_cells, visualize){

    if(visualize_){
      minmax_vis_publisher_ = node_ref->create_publisher<avt_341::msg::MarkerArray>("avt_341/costmap/minmax", 1);
    }
  }

  void RaytraceClearingMethod::Apply(const avt_341::msg::PointCloud &point_cloud) {

  }

  void RaytraceClearingMethod::Bresenham3D(int off_a, int off_b, int off_c,
      unsigned int abs_da, unsigned int abs_db, unsigned int abs_dc,
      int error_b, int error_c, int offset_a, int offset_b, int offset_c, unsigned int &offset,
      unsigned int &z_mask, unsigned int max_length)
  {
    unsigned int end = std::min(max_length, abs_da);
    for (unsigned int i = 0; i < end; ++i)
    {
//      at(offset, z_mask);
      offset_a += off_a;
      error_b += abs_db;
      error_c += abs_dc;
      if ((unsigned int)error_b >= abs_da)
      {
        offset_b += off_b;
        error_b -= abs_da;
      }
      if ((unsigned int)error_c >= abs_da)
      {
        offset_c += off_c;
        error_c -= abs_da;
      }
    }
//    at(offset, z_mask);
  }

  void RaytraceClearingMethod::Bresenham2D()
  {
  }

  void RaytraceClearingMethod::Visualize() const {
    CostmapClearingMethod::Visualize();

    // TODO: Filter based on proximity to robot
    avt_341::msg::MarkerArray marker_array;
    for (int i = 0; i < Nx_; i++) {
      for (int j = 0; j < Ny_; j++) {
        for (int k = 0; k < 2; k++) {
          avt_341::msg::Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = rclcpp::Clock().now();
          marker.id = i * Ny_ + j + (k == 0 ? 0 : Nx_ * Ny_);
          marker.type = avt_341::msg::Marker::CUBE;
          marker.action = avt_341::msg::Marker::ADD;
          marker.pose.position.z = k == 0 ? costmap_cells_[i][j].low.val : costmap_cells_[i][j].high.val;
          marker.pose.position.x = i * 0.1;
          marker.pose.position.y = j * 0.1;
          marker.scale.x = 1.0;
          marker.scale.x = 1.0;
          marker.scale.z = 0.15;
          if(k == 0){
            marker.color.r = 1.0;
          }else{
            marker.color.b = 1.0;
          }
          marker_array.markers.push_back(marker);
        }
      }
    }
    minmax_vis_publisher_->publish(marker_array);
  }

  VoxelRaytraceClearingMethod::VoxelRaytraceClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & costmap_cells, bool visualize)
    : RaytraceClearingMethod(node_ref, costmap_cells, visualize){
    voxel_grid = new std::bitset<1024>[Nx_ * Ny_];
    if(visualize_){
      voxel_vis_publisher_ = node_ref->create_publisher<avt_341::msg::MarkerArray>("avt_341/costmap/voxels", 1);
    }

  }

  void VoxelRaytraceClearingMethod::Apply(const avt_341::msg::PointCloud &point_cloud) {
    RaytraceClearingMethod::Apply(point_cloud);

  }

  void VoxelRaytraceClearingMethod::Visualize() const {
    RaytraceClearingMethod::Visualize();
  }

}
}