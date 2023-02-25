#include "avt_341/perception/costmap_clearing_method.h"
#include "std_msgs/msg/color_rgba.hpp"

namespace avt_341{
namespace perception{

  CostmapClearingMethod::CostmapClearingMethod(std::vector<std::vector<Cell>> &costmap_cells, float visualization_range, bool visualize)
                                              : costmap_cells_(costmap_cells), visualization_range_(visualization_range), visualize_(visualize)
  {
    Nx_ = costmap_cells_.size();
    Ny_ = costmap_cells_[0].size();
  }

  TimedClearingMethod::TimedClearingMethod(float max_point_age, std::vector< std::vector<Cell>> & costmap_cells, float visualization_range, bool visualize)
      : CostmapClearingMethod(costmap_cells, visualization_range, visualize), max_point_age_(max_point_age){
  }

  NullClearingMethod::NullClearingMethod(std::vector<std::vector<Cell>> &costmap_cells, float visualization_range, bool visualize)
      : CostmapClearingMethod(costmap_cells, visualization_range, visualize) {
  }

  void NullClearingMethod::Apply(const msg::PointCloud &point_cloud, const avt_341::msg::Odometry & current_pose) {
  }


  void TimedClearingMethod::Apply(const avt_341::msg::PointCloud &point_cloud, const avt_341::msg::Odometry & current_pose){
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


  RaytraceClearingMethod::RaytraceClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & costmap_cells,
                                                 float visualization_range, bool visualize, float llx, float lly, float res)
    : node_(node_ref), llx_(llx), lly_(lly), res_(res), CostmapClearingMethod(costmap_cells, visualization_range, visualize){

    if(visualize_){
      minmax_vis_publisher_ = node_ref->create_publisher<avt_341::msg::MarkerArray>("avt_341/costmap/voxels",1);
    }
  }

  void RaytraceClearingMethod::Apply(const avt_341::msg::PointCloud &point_cloud, const avt_341::msg::Odometry & current_pose) {
    const auto pos = current_pose.pose.pose.position;
    for(const auto & point : point_cloud.points){
      RaytraceLine(pos, point);
    }
  }
  void RaytraceClearingMethod::RaytraceLine(const avt_341::msg::Point & start, const avt_341::msg::Point32 & end) {

  }

  void VoxelRaytraceClearingMethod::ClearVoxelAt(int x, int y, int z){
    float z_val = static_cast<float>(z) * voxel_height_res_ + voxel_height_min_;
    if(costmap_cells_[x][y].high.val < z_val){
      return;
    }

    int z_i = static_cast<int>((costmap_cells_[x][y].high.val - voxel_height_min_)/voxel_height_res_);
    while(z_i >= z){
      voxel_grid[x*Ny_ + y].set(z_i, false);
      z_i--;
    }

    // Find next filled cell
    int z_min = static_cast<int>((costmap_cells_[x][y].low.val - voxel_height_min_)/voxel_height_res_);
    while(z_i >= z_min && !voxel_grid[x*Ny_ + y].test(z_i)){
      z_i --;
    }

    if(z_i >= z_min){
      // Update cell height
      costmap_cells_[x][y].high.val = static_cast<float>(z_i) * voxel_height_res_ + voxel_height_min_;
    }else{
      // Cell empty
      costmap_cells_[x][y].filled = false;
      costmap_cells_[x][y].obstacle = false;
      costmap_cells_[x][y].high.val = std::numeric_limits<float>::lowest();
      costmap_cells_[x][y].low.val = std::numeric_limits<float>::max();
    }

  }

  void VoxelRaytraceClearingMethod::RaytraceLine(const avt_341::msg::Point & start, const avt_341::msg::Point32 & end) {
    int x1 = static_cast<int>((start.x - llx_) / res_);
    int y1 = static_cast<int>((start.y - lly_) / res_);
    int z1 = static_cast<int>((start.z - voxel_height_min_) / voxel_height_res_);
    int x2 = static_cast<int>((end.x - llx_) / res_);
    int y2 = static_cast<int>((end.y - lly_) / res_);
    int z2 = static_cast<int>((end.z - voxel_height_min_) / voxel_height_res_);
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int dz = std::abs(z2 - z1);
    int x_step = x2 > x1 ? 1 : -1;
    int y_step = y2 > y1 ? 1 : -1;
    int z_step = z2 > z1 ? 1 : -1;

    if(dx >= dy && dx >= dz){
      int p1 = 2 * dy - dx;
      int p2 = 2 * dz - dx;
      while (x2 != x1){
        x1 += x_step;
        if (p1 >= 0){
          y1 += y_step;
          p1 -= 2 * dx;
        }
        p1 += 2 * dy;
        p2 += 2 * dz;
        ClearVoxelAt(x1, y1, z1);
      }
    } else if(dy >= dx && dy >= dz){
      int p1 = 2 * dx - dy;
      int p2 = 2 * dz - dy;
      while(y1 != y2){
        y1 += y_step;
        if (p1 >= 0){
          x1 += x_step;
          p1 -= 2 * dy;
        }
        if (p2 >= 0){
          z1 += z_step;
          p2 -= 2 * dy;
        }
        p1 += 2 * dx;
        p2 += 2 * dz;
        ClearVoxelAt(x1, y1, z1);
      }
    }
    else{
      int p1 = 2 * dy - dz;
      int p2 = 2 * dx - dz;
      while(z1 != z2){
        z1 += z_step;
        if (p1 >= 0){
          y1 += y_step;
          p1 -= 2 * dz;
        }
        if (p2 >= 0){
          x1 += x_step;
          p2 -= 2 * dz;
        }
        p1 += 2 * dy;
        p2 += 2 * dx;
        ClearVoxelAt(x1, y1, z1);
      }
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


  avt_341::msg::Marker RaytraceClearingMethod::GetMarkerMsg(int type, int id, utils::vec3 color, float alpha, double z_scale) const{
    avt_341::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->get_stamp();
    marker.id = id;
    marker.type = type;
    marker.action = avt_341::msg::Marker::MODIFY;
    marker.color.a = alpha;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = z_scale;
    marker.color.r = color.x;
    marker.color.g = color.y;
    marker.color.b = color.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = avt_341::node::make_duration(1.0);
    return marker;
  }

  void RaytraceClearingMethod::GetVoxelBounds(const avt_341::msg::Odometry & odom, int & x_0, int & y_0, int & x_N, int & y_N) const{
    if(visualization_range_ > 0.0){
      x_0 = static_cast<int>((odom.pose.pose.position.x - llx_ - visualization_range_)/res_);
      y_0 = static_cast<int>((odom.pose.pose.position.y - lly_ - visualization_range_)/res_);
      x_N = x_0 + 2*static_cast<int>(visualization_range_/res_);
      y_N = y_0 + 2*static_cast<int>(visualization_range_/res_);
      x_0 = std::min(std::max(0, x_0), Nx_);
      y_0 = std::min(std::max(0, y_0), Ny_);
      x_N = std::min(std::max(0, x_N), Nx_);
      y_N = std::min(std::max(0, y_N), Ny_);
    }else{
      x_0 = 0;
      y_0 = 0;
      x_N = Nx_;
      y_N = Ny_;
    }
  }

  void RaytraceClearingMethod::Visualize(const avt_341::msg::Odometry & odom) const {
    CostmapClearingMethod::Visualize(odom);

    avt_341::msg::MarkerArray marker_array;
    avt_341::msg::Marker mins_marker = GetMarkerMsg(avt_341::msg::Marker::CUBE_LIST, 0, utils::vec3(0.0, 0.0, 1.0), 1.0f, 0.2);
    avt_341::msg::Marker maxes_marker = GetMarkerMsg(avt_341::msg::Marker::CUBE_LIST, 1, utils::vec3(1.0, 0.0, 0.0), 1.0f, 0.2);
    const float max_value = std::numeric_limits<float>::max() - 1e-5;
    const float min_value = std::numeric_limits<float>::lowest() + 1e-5;

    int x_0, y_0, x_N, y_N;
    GetVoxelBounds(odom, x_0, y_0, x_N, y_N);

    for(int i = x_0; i < x_N; i++){
      for(int j = y_0; j < y_N; j++){
        float x_i = llx_ + (static_cast<float>(i) + 0.5)*res_;
        float y_i = lly_ + (static_cast<float>(j) + 0.5)*res_;
        if(costmap_cells_[i][j].low.val < max_value){
          avt_341::msg::Point p1;
          p1.x = x_i;
          p1.y = y_i;
          p1.z = costmap_cells_[i][j].low.val;
          mins_marker.points.push_back(p1);
        }
        if(costmap_cells_[i][j].high.val > min_value && std::abs(costmap_cells_[i][j].high.val - costmap_cells_[i][j].low.val) > 1e-1){
          avt_341::msg::Point p0;
          p0.x = x_i;
          p0.y = y_i;
          p0.z = costmap_cells_[i][j].high.val;
          maxes_marker.points.push_back(p0);
        }
      }
    }
    marker_array.markers.push_back(maxes_marker);
    marker_array.markers.push_back(mins_marker);
    minmax_vis_publisher_->publish(marker_array);
  }

  VoxelRaytraceClearingMethod::VoxelRaytraceClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & costmap_cells,
                                                           float visualization_range, bool visualize, float llx, float lly, float res, float voxel_height_min, float voxel_height_res)
    : voxel_height_min_(voxel_height_min), voxel_height_res_(voxel_height_res), RaytraceClearingMethod(node_ref, costmap_cells, visualization_range, visualize, llx, lly, res){
    voxel_grid = new std::bitset<N_VOXELS_PER_CELL>[Nx_ * Ny_];

  }

  VoxelRaytraceClearingMethod::~VoxelRaytraceClearingMethod(){
    delete voxel_grid;
  }

  void VoxelRaytraceClearingMethod::Apply(const avt_341::msg::PointCloud &point_cloud, const avt_341::msg::Odometry & current_pose) {
    RaytraceClearingMethod::Apply(point_cloud, current_pose);
    for(const auto & point : point_cloud.points){
      int x = static_cast<int>((point.x - llx_)/res_);
      int y = static_cast<int>((point.y - lly_)/res_);
      int z = static_cast<int>((point.z - voxel_height_min_)/voxel_height_res_);
      if(x >= 0 && x < Nx_ && y >= 0 && y < Ny_ && z >= 0 && z < N_VOXELS_PER_CELL){
        voxel_grid[x*Ny_ + y].set(z);
      }
    }
  }

  void VoxelRaytraceClearingMethod::Visualize(const avt_341::msg::Odometry & odom) const {
    avt_341::msg::MarkerArray marker_array;
    avt_341::msg::Marker mins_marker = GetMarkerMsg(avt_341::msg::Marker::CUBE_LIST, 0, utils::vec3(0.0, 0.0, 1.0), 1.0f, 0.2);
    avt_341::msg::Marker maxes_marker = GetMarkerMsg(avt_341::msg::Marker::CUBE_LIST, 1, utils::vec3(1.0, 0.0, 0.0), 1.0f, 0.2);
    avt_341::msg::Marker voxel_marker = GetMarkerMsg(avt_341::msg::Marker::CUBE_LIST, 2, utils::vec3(0.8, 0.8, 0.8), 0.4f, 1.0);
    const float max_value = std::numeric_limits<float>::max() - 1e-5;

    int x_0, y_0, x_N, y_N;
    GetVoxelBounds(odom, x_0, y_0, x_N, y_N);

    for(int i = x_0; i < x_N; i++){
      for(int j = y_0; j < y_N; j++){
        bool has_value = costmap_cells_[i][j].low.val < max_value;
        if(has_value){
          float x_i = llx_ + (static_cast<float>(i) + 0.5)*res_;
          float y_i = lly_ + (static_cast<float>(j) + 0.5)*res_;

          avt_341::msg::Point p1;
          p1.x = x_i;
          p1.y = y_i;
          p1.z = costmap_cells_[i][j].low.val;
          mins_marker.points.push_back(p1);

          if(std::abs(costmap_cells_[i][j].high.val - costmap_cells_[i][j].low.val) > 1e-1){
            avt_341::msg::Point p0;
            p0.x = x_i;
            p0.y = y_i;
            p0.z = costmap_cells_[i][j].high.val;
            maxes_marker.points.push_back(p0);
          }

          auto z_pos = std::max(0, static_cast<int>((costmap_cells_[i][j].low.val - voxel_height_min_)/voxel_height_res_));
          auto max_z_pos = std::min(N_VOXELS_PER_CELL, static_cast<int>((costmap_cells_[i][j].high.val - voxel_height_min_)/voxel_height_res_));
          while(z_pos <= max_z_pos){
            if(voxel_grid[i*Ny_ + j].test(z_pos)){
              avt_341::msg::Point p2;
              p2.x = x_i;
              p2.y = y_i;
              p2.z = voxel_height_min_ + (static_cast<float>(z_pos) + 1.0)*voxel_height_res_;
              voxel_marker.points.push_back(p2);
            }
            z_pos += 1;
          }
        }

      }
    }
    marker_array.markers.push_back(maxes_marker);
    marker_array.markers.push_back(mins_marker);
    marker_array.markers.push_back(voxel_marker);
    minmax_vis_publisher_->publish(marker_array);
  }

}
}