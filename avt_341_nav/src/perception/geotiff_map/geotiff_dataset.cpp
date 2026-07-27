/**
 * Loads a geotiff dataset using gdal libraries
 * 
 * Evan Vandermate - evanderm@mtu.edu
*/

// project includes
#include "avt_341_nav/perception/geotiff_dataset.h"
#include <iostream>

namespace avt_341_nav {
namespace planning{

Geotiff::Geotiff(std::string tiff_path) {
    // Register gdal and quiet warnings
    GDALAllRegister();
    CPLPushErrorHandler(CPLQuietErrorHandler);

    // Read geotiff dataset
    dataset = (GDALDataset*) GDALOpen(tiff_path.c_str(), GA_ReadOnly);
    rows   = GDALGetRasterYSize( dataset );
    cols   = GDALGetRasterXSize( dataset );
    dataset->GetGeoTransform(transform);
    resolution = abs(round(transform[1]*1000.0)/1000.0);
}

/**
 * Reads geotiff band given the band type
*/
template<typename T>
std::vector<double> Geotiff::ReadData(int band) {
    // Get tiff band data type
    GDALDataType bandType = GDALGetRasterDataType(dataset->GetRasterBand(band));

    // Get size of band data
    int nbytes = GDALGetDataTypeSizeBytes(bandType);

    // Create buffer for row scan
    T *rowBuff = (T*) CPLMalloc(nbytes*cols);

    // Read data
    std::vector<std::vector<double>> data_map(rows, std::vector<double>(cols, 0.0));
    for (int i=0; i<rows; i++) {
        CPLErr e = dataset->GetRasterBand(band)->RasterIO(GF_Read,0,i,cols,1,rowBuff,cols,1,bandType,0,0);
        if (e != 0) { 
            return {}; 
        }
        for (int j=0; j<cols; j++) {
            data_map[rows-i-1][j] = (double)rowBuff[j]; // Flip rows to convert from left-handed image coordinates
        }
    }

    // Flatten data
    std::vector<double> data;
    for(const auto &row: data_map) {
        data.insert(data.end(), row.begin(), row.end());
    }

    // Cleanup
    CPLFree( rowBuff );

    return data;
}

/**
 * Reads geotiff band into a double vector
*/
std::vector<double> Geotiff::GetRasterBand(int band) {
    // Validate band
    int bands = GDALGetRasterCount( dataset );
    if (band <= 0 || band > bands) { 
        return {}; 
    }

    // Read band based on type
    switch( GDALGetRasterDataType(dataset->GetRasterBand(band)) ) {
    case 0:
        return {}; // GDT_Unknown, or unknown data type.
    case 1:
        // GDAL GDT_Byte (-128 to 127) - unsigned  char
        return ReadData<unsigned char>(band); 
    case 2:
        // GDAL GDT_UInt16 - short
        return ReadData<unsigned short>(band);
    case 3:
        // GDT_Int16
        return ReadData<short>(band);
    case 4:
        // GDT_UInt32
        return ReadData<unsigned int>(band);
    case 5:
        // GDT_Int32
        return ReadData<int>(band);
    case 6:
        // GDT_Float32
        return ReadData<float>(band);
    case 7:
        // GDT_Float64
        return ReadData<double>(band);
    default:     
        break;  
    }
    return {};  
}

std::string Geotiff::GetProjection() {
    OGRSpatialReference* ogr = (OGRSpatialReference*)dataset->GetSpatialRef();
    if (ogr) {
        return ogr->GetAuthorityCode("PROJCS");
    }
    return "";
}

void Geotiff::PrintInfo() {
    std::cout << "origin = \t[ " << transform[0] << ", " << transform[3] << " ]\n"
              << "transform = \t[ " << transform[1] << ", " << transform[2] << ", " << transform[0] << "\n\t\t  "
                                    << transform[4] << ", " << transform[5] << ", " << transform[3] << " ]\n"
              << "resolution =\t" << resolution << "\n"
              << "projection = \tEPSG:" << GetProjection() << std::endl;
}

} // namespace planning
} // namespace avt_341_nav
