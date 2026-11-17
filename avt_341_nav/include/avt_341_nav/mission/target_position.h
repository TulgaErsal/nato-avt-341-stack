#ifndef AVT_341_NAV_MISSION_TARGET_POSITION_H
#define AVT_341_NAV_MISSION_TARGET_POSITION_H

#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "avt_341_msgs/msg/target_report.hpp"
#include "avt_341_nav/core/target_extent.hpp"

namespace avt_341_nav {
namespace mission {

/// Where a target is: a LiDAR point, or a camera estimate with the camera position it was seen from. A camera
/// position verified by LiDAR keeps its stretch and also has the LiDAR point.
struct TargetPosition {
    Eigen::Vector2d estimate = Eigen::Vector2d::Zero();
    bool lidar = false;
    Eigen::Vector2d observer = Eigen::Vector2d::Zero();
    std::optional<Eigen::Vector2d> point;

    bool hasLidar() const { return lidar || point.has_value(); }
    Eigen::Vector2d best() const { return point ? *point : estimate; }
};

inline TargetPosition ToTargetPosition(const avt_341_msgs::msg::TargetReport & report) {
    TargetPosition position;
    position.estimate = Eigen::Vector2d(report.position.x, report.position.y);
    position.lidar = report.ranging == avt_341_msgs::msg::TargetReport::RANGING_LIDAR;
    position.observer = Eigen::Vector2d(report.observer.x, report.observer.y);
    return position;
}

struct SameTargetSettings {
    core::CameraExtentSettings extent;
    double distance = 5.0;        // whenever a camera range is involved
    double lidar_distance = 7.0;  // between two LiDAR points
};

namespace detail {

struct Part {
    core::Segment2d segment;
    bool lidar;
};

inline std::vector<Part> Parts(const TargetPosition & position, const core::CameraExtentSettings & extent) {
    if (position.lidar) {
        return {{core::PointSegment(position.estimate), true}};
    }
    std::vector<Part> parts{{core::CameraExtent(position.observer, position.estimate, extent), false}};
    if (position.point) {
        parts.push_back({core::PointSegment(*position.point), true});
    }
    return parts;
}

inline std::vector<Part> Points(const TargetPosition & position) {
    std::vector<Part> points{{core::PointSegment(position.estimate), position.lidar}};
    if (position.point) {
        points.push_back({core::PointSegment(*position.point), true});
    }
    return points;
}

inline double Margin(const std::vector<Part> & a, const std::vector<Part> & b, const SameTargetSettings & settings) {
    double margin = std::numeric_limits<double>::infinity();
    for (const auto & pa : a) {
        for (const auto & pb : b) {
            const double limit = pa.lidar && pb.lidar ? settings.lidar_distance : settings.distance;
            margin = std::min(margin, core::SegmentDistance(pa.segment, pb.segment) - limit);
        }
    }
    return margin;
}

}  // namespace detail

/// How far apart two positions are beyond the distance within which they can be the same target; <= 0 means they
/// fit. A camera position counts as its stretch of the line of sight.
inline double FitMargin(const TargetPosition & a, const TargetPosition & b, const SameTargetSettings & settings) {
    return detail::Margin(detail::Parts(a, settings.extent), detail::Parts(b, settings.extent), settings);
}

inline bool Fits(const TargetPosition & a, const TargetPosition & b, const SameTargetSettings & settings) {
    return FitMargin(a, b, settings) <= 0.0;
}

/// The same test on the positions themselves, without the camera stretch.
inline bool AgreesAsPoints(const TargetPosition & a, const TargetPosition & b, const SameTargetSettings & settings) {
    return detail::Margin(detail::Points(a), detail::Points(b), settings) <= 0.0;
}

/// Distance from @p point to the LiDAR point of @p position, if it has one.
inline std::optional<double> LidarPointDistance(const Eigen::Vector2d & point, const TargetPosition & position) {
    if (position.lidar) {
        return (position.estimate - point).norm();
    }
    if (position.point) {
        return (*position.point - point).norm();
    }
    return std::nullopt;
}

}  // namespace mission
}  // namespace avt_341_nav

#endif  // AVT_341_NAV_MISSION_TARGET_POSITION_H
