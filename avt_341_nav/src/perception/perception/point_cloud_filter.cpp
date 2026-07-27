#include "avt_341_nav/perception/point_cloud_filter.hpp"

#include <avt_341_nav/avt_341_utils.h>
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

namespace avt_341_nav::perception {

PointCloudFilter::PointCloudFilter()
    : PointCloudFilter(PointCloudFilterConfig()){
}

PointCloudFilter::PointCloudFilter(const PointCloudFilterConfig &config){
    SetConfig(config);
}

void PointCloudFilter::SetConfig(const PointCloudFilterConfig &config) {
    config_ = config;
    CacheFilterInfo();
}

void PointCloudFilter::CacheFilterInfo() {

    const double EPS = 1e-3;

    filter_max_dist_ = config_.enable_dist_filter && config_.max_dist > EPS;
    filter_min_dist_ = config_.enable_dist_filter && config_.min_dist > EPS;
    filter_hfov_ = config_.min_hfov > -180.0 + EPS && config_.max_hfov < 180.0 - EPS && config_.max_hfov > config_.min_hfov;
    filter_height_clearance_ = config_.max_height_clearance > EPS;

    min_dist_sqr = config_.min_dist * config_.min_dist;
    max_dist_sqr = config_.max_dist * config_.max_dist;
}

bool PointCloudFilter::IsEnabled() const {
    return filter_max_dist_ || filter_min_dist_ || filter_hfov_ || filter_height_clearance_;
}

std::shared_ptr<sensor_msgs::msg::PointCloud> PointCloudFilter::Filter(const std::shared_ptr<sensor_msgs::msg::PointCloud> &pc, const geometry_msgs::msg::Pose& origin) const {

    if (!IsEnabled()) {
        return pc;
    }

    std::shared_ptr<sensor_msgs::msg::PointCloud> pc_out = std::make_shared<sensor_msgs::msg::PointCloud>();
    Filter(*pc, origin, *pc_out);
    return pc_out;
}


void PointCloudFilter::Filter(const sensor_msgs::msg::PointCloud &pc, const geometry_msgs::msg::Pose& origin, sensor_msgs::msg::PointCloud &pc_out) const {

    pc_out.header = pc.header;
    pc_out.channels.resize(pc.channels.size());
    std::vector<std::vector<float>> channel_values(pc.channels.size());
    const double origin_heading = utils::GetHeadingFromOrientation(origin.orientation) * 180.0 / M_PI;

    for (int i = 0; i < pc.points.size(); i++) {
        const geometry_msgs::msg::Point32& p = pc.points[i];
        if (IsValid(p, origin, origin_heading)) {
            pc_out.points.push_back(p);
            for (int c = 0; c < pc.channels.size(); c++) {
                channel_values[c].push_back(pc.channels[c].values[i]);
            }
        }
    }

    for (int c = 0; c < pc.channels.size(); c++) {
        pc_out.channels[c].name = pc.channels[c].name;
        pc_out.channels[c].values = channel_values[c];
    }
}

bool PointCloudFilter::IsValid(const geometry_msgs::msg::Point32 &point, const geometry_msgs::msg::Pose& origin, const double& origin_heading) const {

    if (filter_min_dist_ || filter_max_dist_) {
        const double dx = point.x - origin.position.x;
        const double dy = point.y - origin.position.y;
        const double dz = point.z - origin.position.z;
        const double dist_sqr = dx * dx + dy * dy + dz * dz;
        if (filter_min_dist_ && dist_sqr < min_dist_sqr) {
            return false;
        }
        if (filter_max_dist_ && dist_sqr > max_dist_sqr) {
            return false;
        }
    }

    if (filter_height_clearance_ && point.z - origin.position.z > config_.max_height_clearance) {
        return false;
    }

    if (filter_hfov_) {
        const auto point_angle = std::atan2(point.y - origin.position.y, point.x - origin.position.x) * 180.0 / M_PI;
        double angle_diff = point_angle - origin_heading;
        angle_diff -= 360.0f * floorf((static_cast<float>(angle_diff) + 180.0f) * (1.0f / 360.0f));
        if (angle_diff < config_.min_hfov || angle_diff > config_.max_hfov) {
            return false;
        }
    }

    return true;
}

std::string PointCloudFilter::GetDescription() const {

    if (!IsEnabled()) {
        return "Disabled";
    }

    std::string description;

    if (filter_min_dist_) {
        description += "MinDist: " + std::to_string(config_.min_dist) + "m ";
    }

    if (filter_max_dist_) {
        description += "MaxDist: " + std::to_string(config_.max_dist) + "m ";
    }

    if (filter_height_clearance_) {
        description += "Clearance: " + std::to_string(config_.max_height_clearance) + "m ";
    }

    if (filter_hfov_) {
        description += "HFov: [" + std::to_string(config_.min_hfov) + ", " + std::to_string(config_.max_hfov) + "] ";
    }

    return description;
}


};