#include "avt_341/perception/costmap_clearing_method.h"
#include "std_msgs/msg/color_rgba.hpp"

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
    : node_(node_ref), CostmapClearingMethod(costmap_cells, visualize){

    if(visualize_){
      minmax_vis_publisher_ = node_ref->create_publisher<avt_341::msg::MarkerArray>("avt_341/costmap/minmax", 1);
    }
  }

  void RaytraceClearingMethod::Apply(const avt_341::msg::PointCloud &point_cloud) {
    if(visualize_){
      Visualize();
    }
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

  avt_341::msg::Marker RaytraceClearingMethod::get_marker_msg(int type, int id, bool is_blocked) const{
    avt_341::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->get_stamp();
    marker.id = id;
    marker.type = type;
    marker.action = avt_341::msg::Marker::MODIFY;
    marker.color.a = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    if(is_blocked){
      marker.color.r = 1.0;
    }else{
      marker.color.b = 1.0;
    }
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  void RaytraceClearingMethod::Visualize() const {
    CostmapClearingMethod::Visualize();

//    avt_341::msg::MarkerArray marker_array;
//    avt_341::msg::Marker candidate_paths_marker = get_marker_msg(avt_341::msg::Marker::LINE_LIST, 0, false);
//    candidate_paths_marker.action = avt_341::msg::Marker::MODIFY;
//    for(int i = 0; i < 20; i++){
//      avt_341::msg::Point p0;
//      avt_341::msg::Point p1;
//      p0.x = static_cast<float>(i);
//      p0.y = static_cast<float>(i);
//      p0.z = 0.0;
//      p1.x = static_cast<float>(i+1);
//      p1.y = static_cast<float>(i+1);
//      p1.z = 0.0;
//      candidate_paths_marker.points.push_back(p0);
//      candidate_paths_marker.points.push_back(p1);
//    }
//    marker_array.markers.push_back(candidate_paths_marker);

    avt_341::msg::MarkerArray marker_array;
    avt_341::msg::Marker candidate_paths_marker = get_marker_msg(avt_341::msg::Marker::CUBE_LIST, 0, false);
    candidate_paths_marker.action = avt_341::msg::Marker::MODIFY;
    for(int i = 0; i < 40; i++){
      avt_341::msg::Point p0;
      p0.x = static_cast<float>(i);
      p0.y = static_cast<float>(i);
      p0.z = 0.0;
//      auto color = std_msgs::msg::ColorRGBA();
//      color.a = 1.0;
//      color.g = 1.0;
//      candidate_paths_marker.colors.push_back(color);
      candidate_paths_marker.points.push_back(p0);
    }
    marker_array.markers.push_back(candidate_paths_marker);

//    for (int i = 0; i < 2; i++) {
//      for (int j = 0; j < 2; j++) {
//        for (int k = 0; k < 2; k++) {
//          avt_341::msg::Marker marker;
//          marker.header.frame_id = "map";
//          marker.header.stamp = rclcpp::Clock().now();
//          marker.id = i * Ny_ + j + (k == 0 ? 0 : Nx_ * Ny_);
//          marker.type = avt_341::msg::Marker::POINTS;
//          marker.action = avt_341::msg::Marker::MODIFY;
////          marker.pose.position.z = k == 0 ? costmap_cells_[i][j].low.val : costmap_cells_[i][j].high.val;
//          marker.pose.position.z = 10.0f;
//          marker.pose.position.x = i * 0.1;
//          marker.pose.position.y = j * 0.1;
//          marker.scale.x = 3.0;
//          marker.scale.x = 3.0;
//          marker.scale.z = 0.15;
//          if(k == 0){
//            marker.color.r = 1.0;
//          }else{
//            marker.color.b = 1.0;
//          }
//          marker_array.markers.push_back(marker);
//        }
//      }
//    }
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