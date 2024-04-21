#include "avt_341/perception/voxel_grid.hpp"


void VoxelGrid::setVoxel(int l, int w, int h, int value)
{
  if(value < 0) {
    std::cout << "VoxelGrid::setVoxel - illegal value (" << value << "). Setting to 0." << std::endl;
    value = 0; 
  }
  if(l >=0 && l < length && w >=0 && w < width && h >= 0 && h < height) {
    grid[l][w][h] = value;
  } else {
    std::cout << "VoxelGrid::setVoxel - dimensions out of bounds (" << l << "," << w << "," << h << ") (" << length << "," << width << "," << height << ")" << std::endl;
  }
}

void VoxelGrid::incrementVoxel(int l, int w, int h) 
{
  if(l >=0 && l < length && w >=0 && w < width && h >= 0 && h < height) {
    grid[l][w][h]++;
  } else {
    std::cout << "VoxelGrid::setVoxel - dimensions out of bounds (" << l << "," << w << "," << h << ") (" << length << "," << width << "," << height << ")" << std::endl;
  }
}

void VoxelGrid::decrementVoxel(int l, int w, int h)
{
  if(l >=0 && l < length && w >=0 && w < width && h >= 0 && h < height) {
    grid[l][w][h]--;
    if(grid[l][w][h] < 0) {
      std::cout << "VoxelGrid::decrementVoxel - error decrementing below 0. Resetting to 0." << std::endl;
      grid[l][w][h] = 0;
    }
  } else {
    std::cout << "VoxelGrid::setVoxel - dimensions out of bounds (" << l << "," << w << "," << h << ") (" << length << "," << width << "," << height << ")" << std::endl;
  }
}

int VoxelGrid::getVoxel(int l, int w, int h) const
{
  if(l >=0 && l < length && w >=0 && w < width && h >= 0 && h < height) {
    return grid[l][w][h];
  } else {
    std::cout << "VoxelGrid::setVoxel - dimensions out of bounds (" << l << "," << w << "," << h << ") (" << length << "," << width << "," << height << ")" << std::endl;
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
  float radPitch = pitch * M_PI / 180.0;
  float radAzimuth = azimuth * M_PI / 180.0;
  int x1 = x0 + round(range * cos(radPitch) * cos(radAzimuth));
  int y1 = y0 + round(range * cos(radPitch) * sin(radAzimuth));
  int z1 = z0 + round(range * sin(radPitch));
  return drawLine(x0, y0, z0, x1, y1, z1); 
}