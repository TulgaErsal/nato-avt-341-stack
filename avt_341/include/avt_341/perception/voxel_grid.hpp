#ifndef VOXELGRID_H
#define VOXELGRID_H

// c++ includes
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <tuple>
#include <vector>


using Voxel = std::tuple<int, int, int>;

class VoxelGrid {
public:
  VoxelGrid(int length, int width, int height, double resolution)
        : length(length), width(width), height(height), grid_resolution(resolution),
          cleanGrid(length, std::vector<std::vector<int>>(width, std::vector<int>(height, 0))),
          dirtyGrid(length, std::vector<std::vector<int>>(width, std::vector<int>(height, 0))),
          cleanPlane(length, std::vector<int>(width, 0)),
          dirtyPlane(length, std::vector<int>(width, 0)),
          differencePlane(length, std::vector<float>(width, 1.0)) {}
  void setVoxel(int l, int w, int h, int value);
  void incrementVoxel(int l, int w, int h, bool clean);
  void decrementVoxel(int l, int w, int h);
  int getVoxel(int l, int w, int h, bool clean) const;
  int toVoxelCoord(double coordinate);
  std::vector<Voxel> drawLine(int x0, int y0, int z0, int x1, int y1, int z1);
  std::vector<Voxel> drawLineFromSpherical(int x0, int y0, int z0, double pitch, double azimuth, double range);
  void reset(bool clean);
  void copyCleanToDirty();

  std::vector<std::vector<int>> cleanPlane;
  std::vector<std::vector<int>> dirtyPlane;
  std::vector<std::vector<float>> differencePlane;

private:
  int length, width, height;  // dimensions in # of voxels
  double grid_resolution;      // meters per voxel
  std::vector<std::vector<std::vector<int>>> cleanGrid;
  std::vector<std::vector<std::vector<int>>> dirtyGrid;
  
};
#endif