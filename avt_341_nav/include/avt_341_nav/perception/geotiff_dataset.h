/**
 * Loads a geotiff dataset using gdal libraries
 * 
 * Evan Vandermate - evanderm@mtu.edu
*/
#ifndef GEOTIFFDATASET_H
#define GEOTIFFDATASET_H

#include <fstream>
#include <gdal_priv.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>


namespace avt_341_nav {
namespace planning{

/**
 * Geotiff dataset
*/
class Geotiff {
public:
    Geotiff(std::string tiff_path);
    std::vector<double> GetRasterBand(int band);
    std::string GetProjection();
    void PrintInfo();

    int rows, cols;
    double transform[6];
    float resolution;

private:
    template<typename T>
    std::vector<double> ReadData(int band);

    GDALDataset* dataset;

};

} // namespace planning
} // namespace avt_341_nav

#endif
