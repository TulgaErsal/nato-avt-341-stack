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
        const float original_slope = Slope(cells[xi][yi]);
        float h = point_cloud.points[i].z;
        if (filter_highest_){
          if (h > cells[xi][yi].highest.val ){
            cells[xi][yi].second_highest = cells[xi][yi].highest;
            cells[xi][yi].highest.val = h;
            cells[xi][yi].highest.age = 0.0f;
            cells[xi][yi].high = cells[xi][yi].second_highest;
          }
          else if (h  > cells[xi][yi].second_highest.val){
            cells[xi][yi].second_highest.val = h;
            cells[xi][yi].second_highest.age = 0.0f;
            cells[xi][yi].high = cells[xi][yi].second_highest;
          }
        }
        else{
          if (h > cells[xi][yi].high.val ) {
            cells[xi][yi].high.val = h;
            cells[xi][yi].high.age = 0.0f;
          }
        }
        if (h < cells[xi][yi].low.val ) {
          cells[xi][yi].low.val = h;
          cells[xi][yi].low.age = 0.0f;
        }
        if (has_segmentation_local){
          float terr_val = point_cloud.channels[0].values[i];
          cells[xi][yi].terrain = fmax(cells[xi][yi].terrain, terr_val);
        }

        // Optional dilation
        if(dilate){
          if( (!cells[xi][yi].has_dilated || Slope(cells[xi][yi]) > original_slope) && PastSlopeThreshold(cells[xi][yi])){
            cells[xi][yi].has_dilated = true;
            uint8_t grid_val = (uint8_t) (grid_dilate_proportion_ * GetGridCellValue( cells[xi][yi]));
            for (int xii=std::max(0, xi-dsize_x); xii <= std::min(xi+dsize_x, nx_-1); xii++){
              for (int yii=std::max(0, yi-dsize_y); yii <= std::min(yi+dsize_y, ny_-1); yii++){
                cells[xii][yii].dilated_val = std::max(grid_val, cells[xii][yii].dilated_val);
              }
            }
          }
        }
      }
    }
  }

}

std::vector<avt_341::msg::Point32> ElevationGrid::AddPoints(avt_341::msg::PointCloud &point_cloud){

  if(is_resetting_){
    return std::vector<avt_341::msg::Point32>();
  }

  if (!stitch_points_){
    ClearGrid();
  }
  clearing_method_->ClearOccupancy(point_cloud);
  AddOccupancy(point_cloud, cells_, dilate_);
  clearing_method_->OnOccupancyAdded();

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
    clearing_method_->Reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  is_resetting_ = false;
}

} // namespace perception
} //namespace avt_341