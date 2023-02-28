#include "avt_341/perception/elevation_grid.h"
#include <iostream>
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
  thresh_ = 1.0f;
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
  cells_.clear();
  std::vector<Cell> row;
  row.resize(ny_);
  cells_.resize(nx_,row);
}

void ElevationGrid::ClearGrid(){
Cell empty_cell;
 for (int i=0;i<(nx_);i++){
    for (int j=0;j<(ny_);j++){ 
      cells_[i][j] = empty_cell;
    }
 }
}

void ElevationGrid::AgeCells(){
  float dt = 0.1f; // typical for lidar
  for (int i=0; i<nx_;i++){
    for (int j=0; j<ny_; j++){
      cells_[i][j].AgeCell(dt);
    }
  }

}

std::vector<avt_341::msg::Point32> ElevationGrid::AddPoints(avt_341::msg::PointCloud &point_cloud, const avt_341::msg::Odometry & current_pose){

  bool has_segmentation_local = !point_cloud.channels.empty() && point_cloud.channels[0].name == "segmentation";
  has_segmentation_ = has_segmentation_local || has_segmentation_;

  if (!stitch_points_){
    ClearGrid();
  }
  AgeCells();
  clearing_method_->Apply(point_cloud, current_pose);

  // fill the cells with highest and lowest points
  for (int i=0;i<point_cloud.points.size();i++){
    if (!(point_cloud.points[i].x==0.0 && point_cloud.points[i].y==0.0)){
      int xi = (int)floor((point_cloud.points[i].x - llx_)/res_);
      int yi = (int)floor((point_cloud.points[i].y - lly_)/res_);
      if (xi>=0 && xi<nx_ && yi>=0 &&yi<ny_){
        float h = point_cloud.points[i].z;
//        cells_[xi][yi].filled = true;
        if (filter_highest_){
          if (h > cells_[xi][yi].highest.val ){
            cells_[xi][yi].second_highest = cells_[xi][yi].highest;
            cells_[xi][yi].highest.val = h;
            cells_[xi][yi].highest.age = 0.0f;
            cells_[xi][yi].high = cells_[xi][yi].second_highest;
          }
          else if (h  > cells_[xi][yi].second_highest.val){
            cells_[xi][yi].second_highest.val = h;
            cells_[xi][yi].second_highest.age = 0.0f;
            cells_[xi][yi].high = cells_[xi][yi].second_highest;
          }
        }
        else{
          if (h > cells_[xi][yi].high.val ) {
            cells_[xi][yi].high.val = h;
            cells_[xi][yi].high.age = 0.0f;
          }
        }
        if (h < cells_[xi][yi].low.val ) {
           cells_[xi][yi].low.val = h;
           cells_[xi][yi].low.age = 0.0f;
        }
        if (has_segmentation_local){
          float terr_val = point_cloud.channels[0].values[i];
          cells_[xi][yi].terrain = fmax(cells_[xi][yi].terrain, terr_val);
         }
      }
    }
  }

  std::vector<int> cells_to_dilate_x {};
  std::vector<int> cells_to_dilate_y {};

  //find the slopes
  for (int i=0; i<nx_;i++){
    for (int j=0; j<ny_; j++){
      if (cells_[i][j].filled()){
//        cells_[i][j].height = cells_[i][j].high.val - cells_[i][j].low.val;
        //if (cells_[i][j].height/res_ > thresh_) cells_[i][j].obstacle = true;
//        cells_[i][j].slope = cells_[i][j].height/res_;
        if(!cells_[i][j].has_dilated && PastSlopeThreshold(cells_[i][j])){
          cells_[i][j].has_dilated = true;
          cells_to_dilate_x.push_back(i);
          cells_to_dilate_y.push_back(j);
        }
      } // if cell filled
    } //over j
  } //over i

  //dilate the grid
  if(dilate_){
    int dsize_x = lround(grid_dilate_x_/res_);
    int dsize_y = lround(grid_dilate_y_/res_);

    for(const int & i : cells_to_dilate_x){
      if(i < dsize_x || i >= nx_-dsize_x){
        continue;
      }

      for(const int & j : cells_to_dilate_y){
        if(j < dsize_y || j >= ny_-dsize_y){
          continue;
        }

        if(!cells_[i][j].has_dilated){
          continue;
        }
        uint8_t grid_val = (uint8_t) (grid_dilate_proportion_ * GetGridCellValue( cells_[i][j]));
        for (int id=-dsize_x; id<=dsize_x; id++){
          for (int jd=-dsize_y; jd<=dsize_y; jd++){
            Cell & cell = cells_[i + id][j + jd];
            cell.dilated_val = std::max(grid_val, cell.dilated_val);
          }
        }
      }
    }
  }


  //loop back through the points and remove ground points
  std::vector<avt_341::msg::Point32> points;
  std::vector<avt_341::msg::Point32> surface_points;
  float hscale = 0.2f;
  for (int i=0;i<point_cloud.points.size();i++){
    if (!(point_cloud.points[i].x==0.0 && point_cloud.points[i].y==0.0)){
      int xi = (int)floor((point_cloud.points[i].x - llx_)/res_);
      int yi = (int)floor((point_cloud.points[i].y - lly_)/res_); 
      if (xi>=0 && xi<nx_ && yi>=0 &&yi<ny_){

        if (cells_[xi][yi].obstacle){
          if (point_cloud.points[i].z>(cells_[xi][yi].low.val + hscale*cells_[xi][yi].height())){
            points.push_back(point_cloud.points[i]);
          }
          else{
            surface_points.push_back(point_cloud.points[i]);
          }
        }
        else{
          surface_points.push_back(point_cloud.points[i]);
        }
      }
    }
  }
  point_cloud.points = points;
  return surface_points;
} // method AddPoints

uint8_t ElevationGrid::GetGridCellValue(const Cell & cell) const{
  if(!cell.filled())
    return 0;

  if(use_elevation_){
    return cell.high.val > thresh_ ? GRID_MAX_VALUE : 0;
  }else{
    return PastSlopeThreshold(cell) ? static_cast<uint8_t>(std::min(std::max(0.0f, GRID_SLOPE_MULT*Slope(cell)), static_cast<float>(GRID_MAX_VALUE))) : 0;
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
          grid.data[c++] = is_segmentation ? (uint8_t)(cells_[i][j].terrain) : std::max(GetGridCellValue(cells_[i][j]), cells_[i][j].dilated_val);
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

} // namespace perception
} //namespace avt_341