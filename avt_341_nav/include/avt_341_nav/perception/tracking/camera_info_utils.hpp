/**
* @file      camera_info_utils.hpp
* @brief     Camera intrinsics of the image a CameraInfo describes, with the
             binning and ROI of the ROS CameraInfo convention applied.
*/

#ifndef AVT_341_CAMERA_INFO_UTILS_H
#define AVT_341_CAMERA_INFO_UTILS_H

#include <sensor_msgs/msg/camera_info.hpp>

namespace avt_341_nav {
namespace perception {

/**
 * @brief Return a copy of @p info whose K, P, width and height describe the
 * published image itself, with binning and ROI folded in (as
 * image_geometry::PinholeCameraModel does) and both reset.
 *
 * The camera matrix is taken from P (the rectified projection) when it is
 * set, otherwise from K, because the detections are made on rectified images.
 * With P the image has no distortion left, so D is zeroed and R reset to
 * identity; with K the raw distortion D still applies to the scaled K.
 * A downsized image (e.g. image_proc CropDecimate) keeps the full-resolution
 * K/P in its CameraInfo and only reports binning_x/y and roi.
 */
inline sensor_msgs::msg::CameraInfo ToImageCameraInfo(
    const sensor_msgs::msg::CameraInfo& info) {
    const bool has_p = info.p[0] > 0.0 && info.p[5] > 0.0;
    const double fx = has_p ? info.p[0] : info.k[0];
    const double fy = has_p ? info.p[5] : info.k[4];
    const double cx = has_p ? info.p[2] : info.k[2];
    const double cy = has_p ? info.p[6] : info.k[5];

    const double bx = info.binning_x > 1 ? info.binning_x : 1;
    const double by = info.binning_y > 1 ? info.binning_y : 1;
    const bool has_roi = info.roi.width > 0 && info.roi.height > 0;
    const double ox = has_roi ? info.roi.x_offset : 0.0;
    const double oy = has_roi ? info.roi.y_offset : 0.0;

    sensor_msgs::msg::CameraInfo out = info;
    out.k = {fx / bx, 0.0, (cx - ox) / bx,
             0.0, fy / by, (cy - oy) / by,
             0.0, 0.0, 1.0};
    out.p = {fx / bx, 0.0, (cx - ox) / bx, info.p[3] / bx,
             0.0, fy / by, (cy - oy) / by, info.p[7] / by,
             0.0, 0.0, 1.0, 0.0};
    out.width = static_cast<uint32_t>((has_roi ? info.roi.width : info.width) / bx);
    out.height = static_cast<uint32_t>((has_roi ? info.roi.height : info.height) / by);
    if (has_p) {
        out.d.assign(info.d.size(), 0.0);
        out.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    }
    out.binning_x = 0;
    out.binning_y = 0;
    out.roi = sensor_msgs::msg::RegionOfInterest();
    return out;
}

}  // namespace perception
}  // namespace avt_341_nav

#endif  // AVT_341_CAMERA_INFO_UTILS_H
