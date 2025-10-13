#ifndef POINT_CLOUD_FILTER_H
#define POINT_CLOUD_FILTER_H

#include "avt_341/node/ros_types.h"

namespace avt_341::perception {

struct PointCloudFilterConfig {
    double min_dist = -1.0;
    double max_dist = -1.0;
    double min_hfov = -180.0;
    double max_hfov = 180.0;
    double max_height_clearance = -1.0;
};

class PointCloudFilter {

public:

    PointCloudFilter();
    PointCloudFilter(const PointCloudFilterConfig & config);

    void SetConfig(const PointCloudFilterConfig & config);

    std::shared_ptr<msg::PointCloud> Filter(const std::shared_ptr<msg::PointCloud> &pc, const msg::Pose& origin) const;
    void Filter(const msg::PointCloud &pc, const msg::Pose& origin, msg::PointCloud &pc_out) const;

    inline bool IsValid(const msg::Point32 & point, const msg::Pose& origin, const double& origin_heading) const;

    bool IsEnabled() const;

private:

    void CacheFilterInfo();

    PointCloudFilterConfig config_;

    double min_dist_sqr;
    double max_dist_sqr;

    bool filter_min_dist_;
    bool filter_max_dist_;
    bool filter_hfov_;
    bool filter_height_clearance_;
};


}


#endif //POINT_CLOUD_FILTER_H
