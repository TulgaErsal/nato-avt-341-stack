#ifndef AVT_341_CAMERA_PROJECTION_UTILS_HPP
#define AVT_341_CAMERA_PROJECTION_UTILS_HPP

#include <cmath>
#include <cstdint>
#include <optional>

#ifdef GTE_ROS_JAZZY
#include <image_geometry/pinhole_camera_model.hpp>
#else
#include <image_geometry/pinhole_camera_model.h>
#endif

#include <opencv2/core/types.hpp>

namespace avt_341_nav::perception::camera_projection
{

/// Return whether planar displacement meets the scan-retention threshold.
inline bool MeetsPlanarScanDistance(
    const double current_x,
    const double current_y,
    const double reference_x,
    const double reference_y,
    const double distance_threshold)
{
    if (distance_threshold <= 0.0)
    {
        return true;
    }
    const double dx = current_x - reference_x;
    const double dy = current_y - reference_y;
    return dx * dx + dy * dy
        >= distance_threshold * distance_threshold;
}

/// Project a camera-frame point to its nearest in-bounds raw image pixel.
inline std::optional<cv::Point2i> ProjectToRawImage(
    const image_geometry::PinholeCameraModel& camera_model,
    const cv::Point3d& camera_point,
    const uint32_t image_width,
    const uint32_t image_height)
{
    if (image_width == 0 || image_height == 0
        || !std::isfinite(camera_point.x)
        || !std::isfinite(camera_point.y)
        || !std::isfinite(camera_point.z)
        || camera_point.z <= 0.0)
    {
        return std::nullopt;
    }

    const cv::Point2d rectified_pixel =
        camera_model.project3dToPixel(camera_point);
    const cv::Point2d raw_pixel = camera_model.unrectifyPoint(rectified_pixel);
    if (!std::isfinite(raw_pixel.x) || !std::isfinite(raw_pixel.y)
        || raw_pixel.x < -0.5
        || raw_pixel.y < -0.5
        || raw_pixel.x >= static_cast<double>(image_width) - 0.5
        || raw_pixel.y >= static_cast<double>(image_height) - 0.5)
    {
        return std::nullopt;
    }

    return cv::Point2i(
        static_cast<int>(std::lround(raw_pixel.x)),
        static_cast<int>(std::lround(raw_pixel.y)));
}

}

#endif // AVT_341_CAMERA_PROJECTION_UTILS_HPP
