#include "avt_341/perception/elevation_grid.h"
#include <iostream>
#include <thread>
#include <math.h>

namespace avt_341{
namespace perception{

ElevationGrid::ElevationGrid(){
  width_ = 200.0f;
  height_ = 200.0f;
  llx_ = -100.0f;
  lly_ = -100.0f;
  res_ = 0.5f;
  ResizeGrid();
  thresh_ = 0.5f;
  thresh_max_ = 2.5f;
  dilate_ = false;
  grid_dilate_x_ = 2.0f;
  grid_dilate_y_ = 2.0f;
  grid_dilate_proportion_ = 0.8f;
  use_elevation_ = false;
  stitch_points_ = true;
  filter_highest_ = false;
  max_point_age_ = 5.0f;
}
    
ElevationGrid::~ElevationGrid(){

}

void ElevationGrid::ResizeGrid(){
  nx_ = (int)ceil(width_/res_);
  ny_ = (int)ceil(height_/res_);
  //if (n_%2!=0) n_ = n_+1;
  Cell cell;
  cells_.clear();
  std::vector<Cell> row;
  row.resize(nx_,cell);
  cells_.resize(ny_,row);
}

void ElevationGrid::ClearGrid(){
Cell empty_cell;
 for (int i=0;i<(ny_);i++){
    for (int j=0;j<(nx_);j++){ 
      cells_[i][j] = empty_cell;
    }
 }
}

// CTG, 5/8/25
float ElevationGrid::GetRmsAtCoordinate(float x, float y) {
    int xi = (int)floor((x - llx_) / res_);
    int yi = (int)floor((y - lly_) / res_);
    float rms = GetRmsAtCell(xi, yi);
    return rms;
}

// CTG, 5/8/25
float ElevationGrid::GetRmsAtCell(int xi, int yi) {
    float rms = 0.0f;
    if (xi >= 0 && xi < nx_ && yi >= 0 && yi < ny_) {
        rms = cells_[yi][xi].rms;
    }
    return rms;
}

// CTG, 5/8/25
float ElevationGrid::GetTerrainSlopeAtCoordinate(float x, float y) {
    int xi = (int)floor((x - llx_) / res_);
    int yi = (int)floor((y - lly_) / res_);
    float slope = GetTerrainSlopeAtCell(xi, yi);
    return slope;
}

// CTG, 5/8/25
float ElevationGrid::GetTerrainSlopeAtCell(int xi, int yi) {
    float slope = 0.0f;
    if (xi >= 0 && xi < nx_ && yi >= 0 && yi < ny_) {
        int xl = std::max(xi - 1, 0);
        int xh = std::min(xi + 1, nx_ - 1);
        int yl = std::max(yi - 1, 0);
        int yh = std::min(yi + 1, ny_ - 1);
        float dx = (xh - xl) * res_;
        float dy = (yh - yl) * res_;
        float dz_dx = 0.0f;
        if (dx != 0.0f && cells_[yi][xh].num_points > 0 && cells_[yi][xl].num_points > 0) dz_dx = (cells_[yi][xh].low.val - cells_[yi][xl].low.val) / dx;
        float dz_dy = 0.0f;
        if (dy != 0.0f && cells_[yh][xi].num_points > 0 && cells_[yl][xi].num_points > 0) dz_dy = (cells_[yh][xi].low.val - cells_[yl][xi].low.val) / dy;
        slope = sqrtf(dz_dx * dz_dx + dz_dy * dz_dy);
    }
    return slope;
}

void ElevationGrid::AddOccupancy(const avt_341::msg::PointCloud &point_cloud, std::vector< std::vector<Cell> > & cells, bool dilate) {

  bool has_segmentation_local = !point_cloud.channels.empty() && point_cloud.channels[0].name == "segmentation";
  has_segmentation_ = has_segmentation_local || has_segmentation_;

  int dsize_x = lround(grid_dilate_x_/res_);
  int dsize_y = lround(grid_dilate_y_/res_);

  // fill the cells with highest and lowest points
  for (int i=0;i<point_cloud.points.size();i++){
    if (!(point_cloud.points[i].x==0.0 && point_cloud.points[i].y==0.0)){
      int xi = (int)floor((point_cloud.points[i].x - llx_)/res_);
      int yi = (int)floor((point_cloud.points[i].y - lly_)/res_);
      if (xi>=0 && xi<nx_ && yi>=0 &&yi<ny_){
        const float original_slope = Slope(cells[yi][xi]);
        float h = point_cloud.points[i].z;
        if (filter_highest_){
          if (h > cells[yi][xi].highest.val ){
            cells[yi][xi].second_highest = cells[yi][xi].highest;
            cells[yi][xi].highest.val = h;
            cells[yi][xi].highest.age = 0.0f;
            cells[yi][xi].high = cells[yi][xi].second_highest;
          }
          else if (h  > cells[yi][xi].second_highest.val){
            cells[yi][xi].second_highest.val = h;
            cells[yi][xi].second_highest.age = 0.0f;
            cells[yi][xi].high = cells[yi][xi].second_highest;
          }
        }
        else{
          if (h > cells[yi][xi].high.val ) {
            cells[yi][xi].high.val = h;
            cells[yi][xi].high.age = 0.0f;
          }
        }
        if (h < cells[yi][xi].low.val ) {
          cells[yi][xi].low.val = h;
          cells[yi][xi].low.age = 0.0f;
        }
        if (has_segmentation_local){
          float terr_val = point_cloud.channels[0].values[i];
          cells[yi][xi].terrain = fmax(cells[yi][xi].terrain, terr_val);
        }

        // CTG 5/8/25, add calculations necessary for tracking RMS
        cells_[yi][xi].summed_elev += h;
        cells_[yi][xi].num_points += 1;
        if (cells_[yi][xi].num_points > 0) {
            cells_[yi][xi].avg_elev = cells_[yi][xi].summed_elev / cells_[yi][xi].num_points;
            float dh = h - cells_[yi][xi].avg_elev;
            cells_[yi][xi].sum_of_squares += dh * dh;
            cells_[yi][xi].rms = sqrtf(cells_[yi][xi].sum_of_squares / cells_[yi][xi].num_points);
        }
        else {
            cells_[yi][xi].avg_elev = 0.0f;
            cells_[yi][xi].rms = 0.0f;
        }

        // Optional dilation
        if(dilate){
          if( (!cells[yi][xi].has_dilated || Slope(cells[yi][xi]) > original_slope) && PastSlopeThreshold(cells[yi][xi])){
            cells[yi][xi].has_dilated = true;
            uint8_t grid_val = static_cast<uint8_t>(grid_dilate_proportion_ * static_cast<float>(GetGridCellValue(cells[yi][xi])));
            for (int xii=std::max(0, xi-dsize_x); xii <= std::min(xi+dsize_x, nx_-1); xii++){
              for (int yii=std::max(0, yi-dsize_y); yii <= std::min(yi+dsize_y, ny_-1); yii++){
                cells[yii][xii].dilated_val = std::max(grid_val, cells[yii][xii].dilated_val);
              }
            }
          }
        }
      }
    }
  }
}

void ElevationGrid::AddPoints(avt_341::msg::PointCloud &point_cloud){

  if(is_resetting_){
    return;
  }

  if (!stitch_points_){
    ClearGrid();
  }
  for(auto & cm: clear_methods_){
    cm->ClearOccupancy(point_cloud);
  }
  AddOccupancy(point_cloud, cells_, dilate_);
  for(auto & cm: clear_methods_){
    cm->OnOccupancyAdded(point_cloud);
  }

}

void ElevationGrid::ClearPoints(avt_341::msg::PointCloud &point_cloud){
  for(auto & cm: clear_methods_){
    cm->ClearOccupancy(point_cloud);
  }
}

uint8_t ElevationGrid::GetGridCellValue(const Cell & cell) const{
  if(!cell.filled())
    return 0;

  if(use_elevation_){
    return cell.high.val > thresh_ ? GRID_MAX_VALUE : 0;
  }else{
    const auto slope = cell.height()/res_;
    return slope > thresh_ ? static_cast<uint8_t>(std::min(std::max(0.0f, grid_slope_mult_*slope), static_cast<float>(GRID_MAX_VALUE))) : 0;
  }

}

avt_341::msg::OccupancyGrid ElevationGrid::GetGrid(bool is_segmentation){
  avt_341::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = res_;
  grid.info.width = nx_;
  grid.info.height = ny_;
  grid.info.origin.position.x = llx_;
  grid.info.origin.position.y = lly_;
  grid.info.origin.orientation.w = 1.0;
  grid.info.origin.orientation.x = 0.0;
  grid.info.origin.orientation.y = 0.0;
  grid.info.origin.orientation.z = 0.0;

  grid.data.resize(nx_*ny_);
  int c = 0;

  for (int j = 0; j < ny_; j++) {
    for (int i = 0; i < nx_; i++) {
      grid.data[c++] = is_segmentation ? (uint8_t)(cells_[j][i].terrain) : std::max(GetGridCellValue(cells_[j][i]), cells_[j][i].dilated_val);
    }
  }
  
  return grid;
}

avt_341::msg::OccupancyGrid ElevationGrid::GetGrid(double x, double y, double width, double height, bool is_segmentation) {
    double local_x_origin = x - width/2.0;
    double local_y_origin = y - height/2.0;
    int local_nx = (int)(width/res_);
    int local_ny = (int)(height/res_);
    int xi_min = std::max(0,(int)((local_x_origin-llx_)/res_));
    int yi_min = std::max(0,(int)((local_y_origin-lly_)/res_));
    int xi_max = std::min(nx_,xi_min+local_nx);
    int yi_max = std::min(ny_,yi_min+local_ny);

    avt_341::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = res_;
    grid.info.width = local_nx;
    grid.info.height = local_ny;
    grid.info.origin.position.x = xi_min*res_+llx_;
    grid.info.origin.position.y = yi_min*res_+lly_;
    grid.info.origin.orientation.w = 1.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;

    grid.data.resize(local_nx*local_ny);

    int c = 0;
    for (int j = yi_min; j < yi_max; j++) {
        for (int i = xi_min; i < xi_max; i++) {
            //grid.data[nx_*j+i] = is_segmentation ? (uint8_t)(cells_[j][i].terrain) : std::max(GetGridCellValue(cells_[j][i]), cells_[j][i].dilated_val);
            grid.data[c++] = is_segmentation ? (uint8_t)(cells_[j][i].terrain) : std::max(GetGridCellValue(cells_[j][i]), cells_[j][i].dilated_val);
        }
    }

    return grid;
}

bool ElevationGrid::PastSlopeThreshold(const Cell &cell) const {
  return cell.height()/res_ > thresh_;
}

float ElevationGrid::Slope(const Cell &cell) const {
  return cell.height()/res_;
}

bool ElevationGrid::HasData() const{
  return std::any_of(cells_.begin(), cells_.end(), [](const std::vector<Cell> &row){
    return std::any_of(row.begin(), row.end(), [](const Cell &cell){
      return cell.filled();
    });
  });
}

void ElevationGrid::Reset(){
  is_resetting_ = true;

  while(HasData()){
    ClearGrid();
    for(auto & cm: clear_methods_){
      cm->Reset();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  is_resetting_ = false;
}

std::shared_ptr<OccupancyClearingMethod> ElevationGrid::CreateClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref,
                                                                             std::string clear_method_type,
                                                                             const RaytraceSettings & raytrace_settings,
                                                                             const TimedNoObsClearingSettings & timed_clear_settings,
                                                                             float visualization_range, bool visualize){
  clear_method_type.erase(std::remove_if(clear_method_type.begin(), clear_method_type.end(), ::isspace), clear_method_type.end());
  if(clear_method_type == CostmapClearMethodType::Time) {
    return std::make_shared<TimedClearingMethod>(max_point_age_, cells_, visualization_range, visualize, raytrace_settings, this);
  }
  if(clear_method_type == CostmapClearMethodType::Raytrace) {
    if(!dilate_ || grid_dilate_x_ <= 0 || grid_dilate_y_ <= 0){
      node_ref->log_warning("Raytrace Clearing: Dilation should be enabled with dilation size > 0 to reduce intermittent obstacle.");
    }
    return std::make_shared<RaytraceClearingMethod>(node_ref, cells_, visualization_range, visualize, raytrace_settings, this);
  }
  if(clear_method_type == CostmapClearMethodType::RaytraceWithFiltering) {
    if(!dilate_ || grid_dilate_x_ <= 0 || grid_dilate_y_ <= 0){
      node_ref->log_warning("Raytrace Clearing: Dilation should be enabled with dilation size > 0 to reduce intermittent obstacle.");
    }
    return std::make_shared<RaytraceWithFilteringClearingMethod>(node_ref, cells_, visualization_range, visualize, raytrace_settings, this);
  }
  if(clear_method_type == CostmapClearMethodType::NoObsTime) {
    return std::make_shared<TimedNoObsClearingMethod>(cells_, visualization_range, visualize, timed_clear_settings, raytrace_settings, this);
  }
  if(clear_method_type == CostmapClearMethodType::None){
    return std::make_shared<NullClearingMethod>(cells_, visualization_range, visualize, raytrace_settings, this);
  }
  node_ref->log_error("Unknown costmap clearing method: %s", clear_method_type.c_str());
  return nullptr;
}

void ElevationGrid::SetCostmapClearingMethod(std::shared_ptr<avt_341::node::NodeProxy> node_ref, std::string clear_methods_str,
                                             float visualization_range, bool visualize, float clear_method_raytrace_range, bool clear_method_clear_dilation,
                                             bool use_voxels, float voxel_height_min, float voxel_height_res, float obj_range_filter, int sampled_threshold){
  int dsize_x = lround(grid_dilate_x_/res_);
  int dsize_y = lround(grid_dilate_y_/res_);
  RaytraceSettings raytrace_settings(llx_, lly_, res_, dsize_x, dsize_y, thresh_, clear_method_raytrace_range,
                                     clear_method_clear_dilation, use_voxels, voxel_height_min, voxel_height_res, obj_range_filter);
  TimedNoObsClearingSettings timed_clearing_settings(max_point_age_, sampled_threshold);
  clear_methods_.clear();
  const std::string clear_methods_orig = clear_methods_str;
  size_t pos = 0;
  while ((pos = clear_methods_str.find(",")) != std::string::npos) {
    auto clear_method = CreateClearingMethod(node_ref, clear_methods_str.substr(0, pos), raytrace_settings, timed_clearing_settings, visualization_range, visualize);
    clear_methods_.push_back(clear_method);
    clear_methods_str.erase(0, pos + 1);
  }
  clear_methods_.push_back(CreateClearingMethod(node_ref, clear_methods_str.substr(0, pos), raytrace_settings, timed_clearing_settings, visualization_range, visualize));
  node_ref->log_info("Costmap clearing methods: %s (%d)", clear_methods_orig.c_str(), clear_methods_.size());
}


} // namespace perception
} //namespace avt_341