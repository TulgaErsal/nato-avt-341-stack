#include "avt_341/perception/point_cloud_filter.hpp"

namespace avt_341::perception {

void PointCloudFilter::Filter(
    const std::vector<msg::Point32> &points_in,
    std::vector<msg::Point32> &points_out){

    points_out.clear();
    for (const auto & p : points_in) {
        if (IsValid(p)) {
            points_out.push_back(p);
        }
    }

}

bool PointCloudFilter::IsValid(const msg::Point32 &point) const {

}

};