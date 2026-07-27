#ifndef POINT_CLOUD_FILTER_H
#define POINT_CLOUD_FILTER_H

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

namespace avt_341::perception {

struct PointCloudFilterConfig {
    bool enable_dist_filter = false;
    double min_dist = -1.0;
    double max_dist = -1.0;
    double min_hfov = -180.0;
    double max_hfov = 180.0;
    double max_height_clearance = -1.0;
};

class PointCloudFilter {

public:

    PointCloudFilter();
    explicit PointCloudFilter(const PointCloudFilterConfig & config);

    void SetConfig(const PointCloudFilterConfig & config);

    std::shared_ptr<sensor_msgs::msg::PointCloud> Filter(const std::shared_ptr<sensor_msgs::msg::PointCloud> &pc, const geometry_msgs::msg::Pose& origin) const;
    void Filter(const sensor_msgs::msg::PointCloud &pc, const geometry_msgs::msg::Pose& origin, sensor_msgs::msg::PointCloud &pc_out) const;

    inline bool IsValid(const geometry_msgs::msg::Point32 & point, const geometry_msgs::msg::Pose& origin, const double& origin_heading) const;

    bool IsEnabled() const;

    std::string GetDescription() const;

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
