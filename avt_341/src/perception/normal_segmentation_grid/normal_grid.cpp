#include "avt_341/perception/normal_grid.h"


namespace avt_341{
namespace perception{

NormalGrid::NormalGrid(){
  width_ = 200.0f;
  height_ = 200.0f;
  llx_ = -100.0f;
  lly_ = -100.0f;
  res_ = 0.5f;
  thresh_ = 0.5f;
  ResizeGrid();
}
    
NormalGrid::~NormalGrid(){

}

void NormalGrid::ResizeGrid(){
  nx_ = (int)ceil(width_/res_);
  ny_ = (int)ceil(height_/res_);
  NormalCell cell(res_, avt_341::utils::vec3(0.0, 0.0, 0.0), 0, thresh_, 50);
  cells_.clear();
  std::vector<NormalCell> row;
  row.resize(nx_,cell);
  cells_.resize(ny_,row);
}

void NormalGrid::ClearGrid(){
NormalCell empty_cell;
 for (int i=0;i<(ny_);i++){
    for (int j=0;j<(nx_);j++){ 
      cells_[i][j] = empty_cell;
    }
 }
}

void NormalGrid::AddOccupancy(pcl::PointCloud<pcl::PointNormal>::Ptr pc_normals, std::vector< std::vector<NormalCell> > & cells) {
  for (const auto& point: *pc_normals) {
    int xi = (int)floor((point.x - llx_)/res_);
    int yi = (int)floor((point.y - lly_)/res_);
    if (xi>=0 && xi<nx_ && yi>=0 && yi<ny_){
      cells[yi][xi].AddNormal(point.normal_x, point.normal_y, point.normal_z);
    }
  }
}

void NormalGrid::AddPoints(pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud){
  AddOccupancy(point_cloud, cells_);
}

uint8_t NormalGrid::GetGridCellValue(const NormalCell & cell) const{
  if(!cell.filled())
    return 0;
  return cell.Value();
}

avt_341::msg::OccupancyGrid NormalGrid::GetGrid(bool is_segmentation){
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

  for (auto& row : cells_) {
    grid.data.insert(std::end(grid.data), std::begin(row), std::end(row));
  }
  
  return grid;
}

avt_341::msg::OccupancyGrid NormalGrid::GetGrid(double x, double y, double width, double height) {
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
            grid.data[c++] = cells_[j][i].Value();
        }
    }

    return grid;
}

bool NormalGrid::HasData() const{
  return std::any_of(cells_.begin(), cells_.end(), [](const std::vector<NormalCell> &row){
    return std::any_of(row.begin(), row.end(), [](const NormalCell &cell){
      return cell.filled();
    });
  });
}

void NormalGrid::Reset(){
  ClearGrid();
}

} // namespace perception
} //namespace avt_341