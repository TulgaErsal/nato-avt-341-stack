#include "avt_341/perception/voxel_grid.hpp"


void VoxelGrid::setVoxel(int l, int w, int h, int value)
{
  if(value < 0) {
    std::cout << "VoxelGrid::setVoxel - illegal value (" << value << "). Setting to 0." << std::endl;
    value = 0; 
  }
  if(l >=0 && l < length && w >=0 && w < width && h >= 0 && h < height) {
    dirtyGrid[l][w][h] = value;
  } else {
    std::cout << "VoxelGrid::setVoxel - dimensions out of bounds (" << l << "," << w << "," << h << ") (" << length << "," << width << "," << height << ")" << std::endl;
  }
}

void VoxelGrid::incrementVoxel(int l, int w, int h, bool clean=false) 
{
  if(l >=0 && l < length && w >=0 && w < width && h >= 0 && h < height) {
    if(clean == true) {
      cleanGrid[l][w][h]++;
      cleanPlane[l][w]++;
    } else {
      dirtyGrid[l][w][h]++;
      dirtyPlane[l][w]++;
    }
    // calculate current difference
      differencePlane[l][w] = static_cast<float>(dirtyPlane[l][w]) / cleanPlane[l][w];
    if(clean==false) {
      std::cout << differencePlane[l][w] << " = " << dirtyPlane[l][w] << " / " << cleanPlane[l][w] << std::endl;
    }
  } else {
    //std::cout << "VoxelGrid::incrementVoxel - dimensions out of bounds (" << l << "," << w << "," << h << ") (" << length << "," << width << "," << height << ")" << std::endl;
  }
}

void VoxelGrid::decrementVoxel(int l, int w, int h)
{
  if(l >=0 && l < length && w >=0 && w < width && h >= 0 && h < height) {
    dirtyGrid[l][w][h]--;
    if(dirtyGrid[l][w][h] < 0) {
      std::cout << "VoxelGrid::decrementVoxel - error decrementing below 0. Resetting to 0." << std::endl;
      dirtyGrid[l][w][h] = 0;
    }
    // update the l x w plane too
    dirtyPlane[l][w]--;
    if(dirtyPlane[l][w] < 0) {
      std::cout << "Why is this happening?" << std::endl;
      dirtyPlane[l][w] = 0; 
    }
    // calculate current difference
    differencePlane[l][w] = static_cast<float>(dirtyPlane[l][w]) / cleanPlane[l][w];
    //std::cout << differencePlane[l][w] << " = " << dirtyPlane[l][w] << " / " << cleanPlane[l][w] << std::endl;
    
  } else {
    //std::cout << "VoxelGrid::decrementVoxel - dimensions out of bounds (" << l << "," << w << "," << h << ") (" << length << "," << width << "," << height << ")" << std::endl;
  }
}

int VoxelGrid::getVoxel(int l, int w, int h, bool clean=false) const
{
  if(l >=0 && l < length && w >=0 && w < width && h >= 0 && h < height) {
    if(clean == true) {
      return cleanGrid[l][w][h];
    } else {
      return dirtyGrid[l][w][h];
    }
  } else {
    std::cout << "VoxelGrid::getVoxel - dimensions out of bounds (" << l << "," << w << "," << h << ") (" << length << "," << width << "," << height << ")" << std::endl;
    return -1;
  }
}

int VoxelGrid::toVoxelCoord(double coordinate) {
  return static_cast<int>(std::floor(coordinate / grid_resolution));
}

std::vector<Voxel> VoxelGrid::drawLine(int x0, int y0, int z0, int x1, int y1, int z1) {
  int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int dz = std::abs(z1 - z0), sz = z0 < z1 ? 1 : -1;
  int dm = std::max(dx, std::max(dy, dz)), i = dm;  // max difference
  x1 = y1 = z1 = dm / 2; // error offset
  std::vector<Voxel> voxels;
  for (; ;) {
    voxels.emplace_back(x0, y0, z0);
    if(i-- == 0) break; // stop
    x1 -= dx; if(x1 < 0) {x1 += dm; x0 += sx; }
    y1 -= dy; if(y1 < 0) {y1 += dm; y0 += sy; }
    z1 -= dz; if(z1 < 0) {z1 += dm; z0 += sz; }
  }
  return voxels;
}

std::vector<Voxel> VoxelGrid::drawLineFromSpherical(int x0, int y0, int z0, double pitch, double azimuth, double range)
{
  range = range/grid_resolution;
  float radPitch = pitch * M_PI / 180.0;
  float radAzimuth = azimuth * M_PI / 180.0;
  int x1 = x0 + round(range * cos(radPitch) * cos(radAzimuth));
  int y1 = y0 + round(range * cos(radPitch) * sin(radAzimuth));
  int z1 = z0 + round(range * sin(radPitch));
  return drawLine(x0, y0, z0, x1, y1, z1); 
}

void VoxelGrid::reset(bool clean=false) {
  if(clean == true) {
    for(auto &plane : cleanGrid) {
      for(auto &row : plane) {
        std::fill(row.begin(), row.end(), 0);
      }
    }
  } else {
    for(auto &plane : dirtyGrid) {
      for(auto &row : plane) {
        std::fill(row.begin(), row.end(), 0);
      }
    }
  }
}

void VoxelGrid::copyCleanToDirty() {
  dirtyGrid = cleanGrid;
  dirtyPlane = cleanPlane;
  std::vector<std::vector<float>> differencePlane(length, std::vector<float>(width, 1.0));
}