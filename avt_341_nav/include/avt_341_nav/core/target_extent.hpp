#ifndef AVT_341_NAV_CORE_TARGET_EXTENT_HPP
#define AVT_341_NAV_CORE_TARGET_EXTENT_HPP

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace avt_341_nav {
namespace core {

/**
 * @brief Where a target can be along the camera's line of sight to it. The camera range estimate reads long more
 *        than short, so the stretch reaches farther toward the camera than away from it.
 */
struct CameraExtentSettings {
    double near_fraction = 0.4;
    double near_max = 15.0;
    double far_fraction = 0.1;
    double far_max = 5.0;
};

struct Segment2d {
    Eigen::Vector2d start = Eigen::Vector2d::Zero();
    Eigen::Vector2d end = Eigen::Vector2d::Zero();
};

inline Segment2d PointSegment(const Eigen::Vector2d& point) {
    return {point, point};
}

/** @brief The stretch of the line of sight from @p observer through the camera estimate @p estimate. */
inline Segment2d CameraExtent(const Eigen::Vector2d& observer, const Eigen::Vector2d& estimate,
                              const CameraExtentSettings& settings) {
    const Eigen::Vector2d ray = estimate - observer;
    const double range = ray.norm();
    if (range <= 0.0) {
        return PointSegment(estimate);
    }
    const Eigen::Vector2d unit = ray / range;
    const double near = std::min(settings.near_fraction * range, settings.near_max);
    const double far = std::min(settings.far_fraction * range, settings.far_max);
    return {estimate - near * unit, estimate + far * unit};
}

inline double PointSegmentDistance(const Eigen::Vector2d& point, const Segment2d& segment) {
    const Eigen::Vector2d direction = segment.end - segment.start;
    const double length_sq = direction.squaredNorm();
    const double t = length_sq > 0.0
                         ? std::clamp((point - segment.start).dot(direction) / length_sq, 0.0, 1.0)
                         : 0.0;
    return (point - (segment.start + t * direction)).norm();
}

inline double SegmentDistance(const Segment2d& a, const Segment2d& b) {
    const auto cross = [](const Eigen::Vector2d& o, const Eigen::Vector2d& p, const Eigen::Vector2d& q) {
        return (p.x() - o.x()) * (q.y() - o.y()) - (p.y() - o.y()) * (q.x() - o.x());
    };
    const double d1 = cross(b.start, b.end, a.start);
    const double d2 = cross(b.start, b.end, a.end);
    const double d3 = cross(a.start, a.end, b.start);
    const double d4 = cross(a.start, a.end, b.end);
    if (d1 * d2 < 0.0 && d3 * d4 < 0.0) {
        return 0.0;
    }
    return std::min({PointSegmentDistance(a.start, b), PointSegmentDistance(a.end, b),
                     PointSegmentDistance(b.start, a), PointSegmentDistance(b.end, a)});
}

}  // namespace core
}  // namespace avt_341_nav

#endif  // AVT_341_NAV_CORE_TARGET_EXTENT_HPP
