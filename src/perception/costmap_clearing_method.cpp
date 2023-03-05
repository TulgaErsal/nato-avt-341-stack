#include "avt_341/perception/costmap_clearing_method.h"

namespace avt_341{
namespace perception{

  // BASE CLASS
  // ==================================================================================================================
  // ==================================================================================================================

  OccupancyClearingMethod::OccupancyClearingMethod(std::vector<std::vector<Cell>> &cells, int Nx, int Ny, float visualization_range, bool visualize)
                                              : cells_(cells), Nx_(Nx), Ny_(Ny), visualization_range_(visualization_range), visualize_(visualize)
  {
  }

  // NULL CLEARING
  // ==================================================================================================================
  // ==================================================================================================================

  NullClearingMethod::NullClearingMethod(std::vector<std::vector<Cell>> &cells, float visualization_range, bool visualize)
      : OccupancyClearingMethod(cells, cells.size(), cells[0].size(), visualization_range, visualize) {
  }

  void NullClearingMethod::ClearOccupancy(const msg::PointCloud &point_cloud) {
  }

  // TIME BASED CLEARING
  // ==================================================================================================================
  // ==================================================================================================================

  TimedClearingMethod::TimedClearingMethod(float max_point_age, std::vector< std::vector<Cell>> & cells, float visualization_range, bool visualize)
      : max_point_age_(max_point_age), OccupancyClearingMethod(cells, cells.size(), cells[0].size(), visualization_range, visualize) {
  }

  void TimedClearingMethod::AgeCells(const float dt){
    for (int i=0; i<Nx_;i++){
      for (int j=0; j<Ny_; j++){
        cells_[i][j].AgeCell(dt);
      }
    }
  }

  void TimedClearingMethod::ClearOccupancy(const avt_341::msg::PointCloud &point_cloud){
    auto now = node::seconds_from_header(point_cloud.header);
    if(last_timestamp_ > 0){
      AgeCells(now - last_timestamp_);
    }
    last_timestamp_ = now;
    Cell empty_cell;
    for (int i=0; i<Nx_;i++){
      for (int j=0; j<Ny_; j++){
        if (cells_[i][j].low.age > max_point_age_ ||
            cells_[i][j].highest.age > max_point_age_ ||
            cells_[i][j].second_highest.age > max_point_age_ ||
            cells_[i][j].high.age > max_point_age_){
            cells_[i][j]=empty_cell;
        }
        if (cells_[i][j].dilated_val > 0 && cells_[i][j].dilated_age > max_point_age_){
          cells_[i][j]=empty_cell;
        }
    }
  }
  }

  // RAYTRACE CLEARING
  // ==================================================================================================================
  // ==================================================================================================================

  const int RaytraceClearingMethod::N_VOXELS_PER_CELL;

  RaytraceClearingMethod::RaytraceClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & cells,
                                                 float visualization_range, bool visualize, RaytraceSettings settings, CellObstacleCalculator* cell_obstacle_calculator, bool handle_dilation)
                                                 : RaytraceClearingMethod(node_ref, cells, cells.size(), cells[0].size(), visualization_range, visualize, settings, cell_obstacle_calculator, handle_dilation){
  }

  RaytraceClearingMethod::RaytraceClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & cells, int Nx, int Ny,
                                                 float visualization_range, bool visualize, RaytraceSettings settings, CellObstacleCalculator* cell_obstacle_calculator, bool handle_dilation)
      : node_(node_ref), config_(settings), cell_obstacle_calculator_(cell_obstacle_calculator), handle_dilation_(handle_dilation), OccupancyClearingMethod(cells, Nx, Ny, visualization_range, visualize){

    node_->initialize_tf_listener();
    if(visualize_){
      minmax_vis_publisher_ = node_ref->create_publisher<avt_341::msg::MarkerArray>("avt_341/occupancy_grid/voxels",1);
    }
    if(config_.use_voxels){
      voxel_grid = new std::bitset<N_VOXELS_PER_CELL>[Nx_ * Ny_];
    }
  }

  RaytraceClearingMethod::~RaytraceClearingMethod(){
    delete voxel_grid;
  }

  void RaytraceClearingMethod::CleanupUnattachedDilation(const avt_341::msg::Point & origin, std::vector< std::vector<Cell>> & cells){
    // Clean up unattached dilated cells
    int x_0, y_0, x_N, y_N;
    GetGridBounds(origin, config_.raytrace_range, x_0, y_0, x_N, y_N);
    for(int x = x_0; x < x_N; x++){
      for(int y = y_0; y < y_N; y++){
        if(cells[x][y].filled() && cells[x][y].dilated_val > 0){
          bool found_obs = false;
          for(int i = std::max(0, x - config_.grid_dilate_x); !found_obs && i <= std::min(Nx_ - 1, x + config_.grid_dilate_x); i++){
            for(int j = std::max(0, y - config_.grid_dilate_y); !found_obs && j <= std::min(Ny_ - 1, y + config_.grid_dilate_y); j++){
//              if(cell_obstacle_calculator_->PastSlopeThreshold(cells[i][j]) || abs(cells[x][y].high.val - cells[i][j].high.val) > thresh_){
              if(cells[i][j].has_dilated || (cells[i][j].filled() && abs(cells[x][y].high.val - cells[i][j].high.val) > config_.thresh)){
                found_obs = true;
              }
            }
          }
          if(!found_obs){
            cells[x][y].dilated_val = 0;
          }
        } // if dilated_val > 0
      }
    }
  }

  avt_341::msg::Point RaytraceClearingMethod::GetSensorOrigin() const{
    avt_341::msg::Point origin;
    auto lidar_transform = node_->lookup_transform("map", "lidar");
    origin.x = lidar_transform.transform.translation.x;
    origin.y = lidar_transform.transform.translation.y;
    origin.z = lidar_transform.transform.translation.z;
    return origin;
  }

  void RaytraceClearingMethod::ClearOccupancy(const avt_341::msg::PointCloud &point_cloud) {
    avt_341::msg::Point origin = GetSensorOrigin();
    for(const auto & point : point_cloud.points){
      const float dx = point.x - origin.x;
      const float dy = point.y - origin.y;
      if(dx*dx + dy*dy < config_.raytrace_range * config_.raytrace_range){
        RaytraceLine(origin, point);
      }
    }

    if(handle_dilation_ && !config_.clear_dilation){
      CleanupUnattachedDilation(origin, cells_);
    }

    for(const auto & point : point_cloud.points){
      int x = static_cast<int>((point.x - config_.llx) / config_.res);
      int y = static_cast<int>((point.y - config_.lly) / config_.res);
      int z = static_cast<int>((point.z - config_.voxel_height_min) / config_.voxel_height_res);
      if(x >= 0 && x < Nx_ && y >= 0 && y < Ny_ && z >= 0 && z < N_VOXELS_PER_CELL){
        voxel_grid[x*Ny_ + y].set(z);
      }
    }

  }

  void RaytraceClearingMethod::RemoveDilationAtCell(int x, int y, std::vector< std::vector<Cell>> & cells){
    cells[x][y].has_dilated = false;
    for(int i = std::max(0, x - config_.grid_dilate_x); i <= std::min(Nx_ - 1, x + config_.grid_dilate_x); i++){
      for(int j = std::max(0, y - config_.grid_dilate_y); j <= std::min(Ny_ - 1, y + config_.grid_dilate_y); j++){
        if(config_.clear_dilation || (cells[i][j].filled() && abs(cells[x][y].high.val - cells[i][j].high.val) < config_.thresh)){
          bool found_obs = false;
          for(int ii = std::max(0, i - config_.grid_dilate_x); !found_obs && ii <= std::min(Nx_ - 1, i + config_.grid_dilate_x); ii++){
            for(int jj = std::max(0, j - config_.grid_dilate_y); !found_obs && jj <= std::min(Ny_ - 1, j + config_.grid_dilate_y); jj++){
              found_obs = cell_obstacle_calculator_->PastSlopeThreshold(cells[ii][jj]);
            }
          }
          if(!found_obs){
            cells[i][j].dilated_val = 0;
          }
        }
      }
    }

  }

  void RaytraceClearingMethod::ClearVoxelAt(int x, int y, int z){
    int z_i = static_cast<int>((cells_[x][y].high.val - config_.voxel_height_min) / config_.voxel_height_res);
    if(!cells_[x][y].filled() || (z_i <= z || z < 0 || z >= N_VOXELS_PER_CELL)){
//    if((z_i <= z || z < 0 || z >= N_VOXELS_PER_CELL)){
      return;
    }
    int z_min = std::max(0, static_cast<int>((cells_[x][y].low.val - config_.voxel_height_min) / config_.voxel_height_res));

    if(config_.use_voxels){
      while(z_i >= z){
        voxel_grid[x*Ny_ + y].set(z_i, false);
        z_i--;
      }
      // Find next filled cell
      while(z_i >= z_min && !voxel_grid[x*Ny_ + y].test(z_i)){
        z_i --;
      }
    }

    bool was_obstacle = cell_obstacle_calculator_->PastSlopeThreshold(cells_[x][y]);
    if(z_i < z_min){
      // Cell empty
      cells_[x][y].high.val = cells_[x][y].low.val;
    }else{
      // Update cell height
      cells_[x][y].high.val = static_cast<float>(z_i) * config_.voxel_height_res + config_.voxel_height_min;
    }

    // Remove surrounding dilation if cell no longer obstacle
    if(handle_dilation_ && was_obstacle && !cell_obstacle_calculator_->PastSlopeThreshold(cells_[x][y])){
      RemoveDilationAtCell(x, y, cells_);
    }

  }

  void RaytraceClearingMethod::RaytraceLine(const avt_341::msg::Point & start, const avt_341::msg::Point32 & end) {
    int x1 = static_cast<int>((start.x - config_.llx) / config_.res);
    int y1 = static_cast<int>((start.y - config_.lly) / config_.res);
    int z1 = static_cast<int>((start.z - config_.voxel_height_min) / config_.voxel_height_res);
    int x2 = static_cast<int>((end.x - config_.llx) / config_.res);
    int y2 = static_cast<int>((end.y - config_.lly) / config_.res);
    if(x2 < 0 || x2 >= Nx_ || y2 < 0 || y2 >= Ny_){
      return;
    }
    int z2 = static_cast<int>((end.z - config_.voxel_height_min) / config_.voxel_height_res);
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int dz = std::abs(z2 - z1);
    int x_step = x2 > x1 ? 1 : -1;
    int y_step = y2 > y1 ? 1 : -1;
    int z_step = z2 > z1 ? 1 : -1;

    if(dx >= dy && dx >= dz){
      int p1 = 2 * dy - dx;
      int p2 = 2 * dz - dx;
      while (x1 != x2){
        x1 += x_step;
        if (p1 >= 0){
          y1 += y_step;
          p1 -= 2 * dx;
        }
        if (p2 >= 0){
          z1 += z_step;
          p2 -= 2 * dx;
        }
        p1 += 2 * dy;
        p2 += 2 * dz;
        if(x1 == x2 && y1 == y2){
          break;
        }
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
        if(x1 == x2 && y1 == y2){
          break;
        }
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
        if(x1 == x2 && y1 == y2){
          break;
        }
        ClearVoxelAt(x1, y1, z1);
      }
    }

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

  void RaytraceClearingMethod::GetGridBounds(const avt_341::msg::Point & origin, float range, int & x_0, int & y_0, int & x_N, int & y_N) const{
    if(range > 0.0){
      x_0 = static_cast<int>((origin.x - config_.llx - range) / config_.res);
      y_0 = static_cast<int>((origin.y - config_.lly - range) / config_.res);
      x_N = x_0 + 2*static_cast<int>(range / config_.res);
      y_N = y_0 + 2*static_cast<int>(range / config_.res);
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

  void RaytraceClearingMethod::Visualize() const {
    OccupancyClearingMethod::Visualize();

    avt_341::msg::MarkerArray marker_array;
    avt_341::msg::Marker mins_marker = GetMarkerMsg(avt_341::msg::Marker::CUBE_LIST, 0, utils::vec3(0.0, 0.0, 1.0), 1.0f, 0.2);
    avt_341::msg::Marker maxes_marker = GetMarkerMsg(avt_341::msg::Marker::CUBE_LIST, 1, utils::vec3(1.0, 0.0, 0.0), 1.0f, 0.2);
    avt_341::msg::Marker voxel_marker = GetMarkerMsg(avt_341::msg::Marker::CUBE_LIST, 2, utils::vec3(0.8, 0.8, 0.8), 0.4f, config_.voxel_height_res);
    const float max_value = std::numeric_limits<float>::max() - 1e-5;
    avt_341::msg::Point origin = GetSensorOrigin();
    int x_0, y_0, x_N, y_N;
    GetGridBounds(origin, visualization_range_, x_0, y_0, x_N, y_N);

    for(int i = x_0; i < x_N; i++){
      for(int j = y_0; j < y_N; j++){
        bool has_value = cells_[i][j].low.val < max_value;
        if(has_value){
          float x_i = config_.llx + (static_cast<float>(i) + 0.5) * config_.res;
          float y_i = config_.lly + (static_cast<float>(j) + 0.5) * config_.res;

          avt_341::msg::Point p1;
          p1.x = x_i;
          p1.y = y_i;
          p1.z = cells_[i][j].low.val;
          mins_marker.points.push_back(p1);

          if(std::abs(cells_[i][j].high.val - cells_[i][j].low.val) > 1e-1){
            avt_341::msg::Point p0;
            p0.x = x_i;
            p0.y = y_i;
            p0.z = cells_[i][j].high.val;
            maxes_marker.points.push_back(p0);
          }

          if(config_.use_voxels){
            auto z_pos = std::max(0, static_cast<int>((cells_[i][j].low.val - config_.voxel_height_min) / config_.voxel_height_res));
            auto max_z_pos = std::min(N_VOXELS_PER_CELL, static_cast<int>((cells_[i][j].high.val - config_.voxel_height_min) / config_.voxel_height_res));
            while(z_pos <= max_z_pos){
              if(voxel_grid[i*Ny_ + j].test(z_pos)){
                avt_341::msg::Point p2;
                p2.x = x_i;
                p2.y = y_i;
                p2.z = config_.voxel_height_min + (static_cast<float>(z_pos) + 0.5) * config_.voxel_height_res;
                voxel_marker.points.push_back(p2);
              }
              z_pos += 1;
            }
          }
        }

      }
    }
    marker_array.markers.push_back(maxes_marker);
    marker_array.markers.push_back(mins_marker);
    marker_array.markers.push_back(voxel_marker);
    minmax_vis_publisher_->publish(marker_array);
  }

  // RAYTRACE CLEARING WITH OBSTACLE DISTANCE FILTERING
  // ==================================================================================================================
  // ==================================================================================================================

  RaytraceWithFilteringClearingMethod::RaytraceWithFilteringClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::vector< std::vector<Cell>> & cells,
                                                 float visualization_range, bool visualize, RaytraceSettings settings, float obj_range_filter, CellObstacleCalculator* cell_obstacle_calculator)
      : cells_without_clearing_(cells), obj_filter_range_(obj_range_filter), RaytraceClearingMethod(node_ref, cells_with_clearing_, cells.size(),
        cells[0].size(), visualization_range, visualize, settings, cell_obstacle_calculator, false){

    std::vector<Cell> row;
    row.resize(Ny_);
    cells_with_clearing_.resize(Nx_,row);

    const int N = 2*static_cast<int>(config_.raytrace_range/config_.res);
    occupancy_delta_ = std::vector<std::vector<bool>>(N, std::vector<bool>(N, false));

    if(visualize_){
      occupancy_delta_publisher_ = node_->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/occupancy_grid/clear_delta", 1);
    }
  }

  void RaytraceWithFilteringClearingMethod::ClearOccupancy(const avt_341::msg::PointCloud &point_cloud){
    RaytraceClearingMethod::ClearOccupancy(point_cloud);
    cell_obstacle_calculator_->AddOccupancy(point_cloud, cells_with_clearing_, false);
  }

  void RaytraceWithFilteringClearingMethod::OnOccupancyAdded(){
    avt_341::msg::Point origin = GetSensorOrigin();
    int x_0, y_0, x_N, y_N;
    GetGridBounds(origin, config_.raytrace_range, x_0, y_0, x_N, y_N);
    int search_range = static_cast<int>(obj_filter_range_ / config_.res);
    last_position_.x = static_cast<float>(x_0);
    last_position_.y = static_cast<float>(y_0);

    for(int x = x_0; x < x_N; x++){
      for(int y = y_0; y < y_N; y++){
        bool candidate_clear = cell_obstacle_calculator_->PastSlopeThreshold(cells_without_clearing_[x][y]) && !cell_obstacle_calculator_->PastSlopeThreshold(cells_with_clearing_[x][y]);
        occupancy_delta_[x-x_0][y-y_0] = candidate_clear;
        if(candidate_clear){
          bool found_obs = false;
          const int i_0 = std::max(0, x - search_range);
          const int i_N = std::min(Nx_ - 1, x + search_range);
          const int j_0 = std::max(0, y - search_range);
          const int j_N = std::min(Ny_ - 1, y + search_range);
          for(int i = i_0; i <= i_N && !found_obs; i++){
            for(int j = j_0; j <= j_N && !found_obs; j++){
              found_obs |= cell_obstacle_calculator_->PastSlopeThreshold(cells_with_clearing_[i][j]);
            }
          }
          if(!found_obs){
            // Apply cleared cell
            cells_without_clearing_[x][y].high = cells_with_clearing_[x][y].high;
            cells_without_clearing_[x][y].low = cells_with_clearing_[x][y].low;
            RemoveDilationAtCell(x, y, cells_without_clearing_);
          }
        }
      }
    }

    CleanupUnattachedDilation(origin, cells_without_clearing_);
  }

  void RaytraceWithFilteringClearingMethod::Visualize() const {
    RaytraceClearingMethod::Visualize();

    const int N_size = 2*static_cast<int>(config_.raytrace_range / config_.res);
    avt_341::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header.stamp = node_->get_stamp();
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.resolution = config_.res;
    occupancy_grid.info.width = N_size;
    occupancy_grid.info.height = N_size;
    occupancy_grid.info.origin.position.x = last_position_.x - 2*config_.raytrace_range;
    occupancy_grid.info.origin.position.y = last_position_.y - 2*config_.raytrace_range;
    occupancy_grid.info.origin.position.z = 1.0;
    occupancy_grid.info.origin.orientation.w = 1.0;
    occupancy_grid.info.origin.orientation.x = 0.0;
    occupancy_grid.info.origin.orientation.y = 0.0;
    occupancy_grid.info.origin.orientation.z = 0.0;
    occupancy_grid.data.resize(N_size*N_size);
    for(int i = 0; i < N_size; i++){
      for(int j = 0; j < N_size; j++){
        occupancy_grid.data[j*N_size + i] = (uint8_t)(occupancy_delta_[i][j] ? 100 : 0);
      }
    }
    occupancy_delta_publisher_->publish(occupancy_grid);
  }

  }
}