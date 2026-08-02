/**
* @brief     Validation helpers for ROS message pairs. Kept free of any
*            node/logging dependencies; failures are reported by throwing.
*/

#ifndef AVT_341_ROS_MSG_VALIDATION_HPP
#define AVT_341_ROS_MSG_VALIDATION_HPP

#include <stdexcept>
#include <string>
#include <string_view>

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace avt_341_nav::core
{

/// Throws std::runtime_error when the image is missing or not in the expected encoding.
inline void ValidateImageEncoding(
    const sensor_msgs::msg::Image::ConstSharedPtr& image,
    const std::string_view expected_encoding)
{
    if (image == nullptr || image->encoding != expected_encoding)
    {
        const std::string encoding = image == nullptr ? "none" : image->encoding;
        throw std::runtime_error(
            "unsupported image encoding '" + encoding + "', expected "
            + std::string(expected_encoding) + ".");
    }
}

/// Throws std::runtime_error when the image dimensions are empty or differ
/// from the camera info dimensions.
inline void ValidateImageSizeMatchesCameraInfo(
    const sensor_msgs::msg::Image& image,
    const sensor_msgs::msg::CameraInfo& camera_info)
{
    if (image.width == 0 || image.height == 0
        || image.width != camera_info.width
        || image.height != camera_info.height)
    {
        throw std::runtime_error(
            "image size (" + std::to_string(image.width) + "x"
            + std::to_string(image.height) + ") does not match CameraInfo ("
            + std::to_string(camera_info.width) + "x"
            + std::to_string(camera_info.height) + ").");
    }
}

/// Throws std::runtime_error when the image declares a frame different from
/// the camera info frame.
inline void ValidateImageFrameMatchesCameraInfo(
    const sensor_msgs::msg::Image& image,
    const sensor_msgs::msg::CameraInfo& camera_info)
{
    if (!image.header.frame_id.empty()
        && image.header.frame_id != camera_info.header.frame_id)
    {
        throw std::runtime_error(
            "image frame '" + image.header.frame_id
            + "' does not match CameraInfo frame '"
            + camera_info.header.frame_id + "'.");
    }
}

/// Validates the image encoding and its size and frame against the camera
/// info; throws std::runtime_error on the first failed check.
inline void ValidateImageWithCameraInfo(
    const sensor_msgs::msg::Image::ConstSharedPtr& image,
    const sensor_msgs::msg::CameraInfo& camera_info,
    const std::string_view expected_encoding)
{
    ValidateImageEncoding(image, expected_encoding);
    ValidateImageSizeMatchesCameraInfo(*image, camera_info);
    ValidateImageFrameMatchesCameraInfo(*image, camera_info);
}

}

#endif // AVT_341_ROS_MSG_VALIDATION_HPP
