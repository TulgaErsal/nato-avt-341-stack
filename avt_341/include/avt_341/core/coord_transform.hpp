#ifndef AVT_341_COORD_TRANSFORM_H
#define AVT_341_COORD_TRANSFORM_H

#include <string>

#include <Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <tf2_ros/buffer.h>

namespace avt_341::core
{

/**
 * @brief Transform a three-dimensional point between TF frames using the
 * latest available transform (tf2::TimePointZero).
 *
 * On a TF lookup failure the error is logged on @p logger and the input
 * point is returned unchanged.
 *
 * @param buffer Transform buffer to look the transform up in.
 * @param source_frame Frame ID the point is currently expressed in.
 * @param target_frame Frame ID to transform the point into.
 * @param point Point to be transformed.
 * @param logger Logger used to report TF lookup failures.
 * @return Eigen::Vector3d Transformed point.
 */
Eigen::Vector3d TransformToCoordinates(const tf2_ros::Buffer& buffer,
                                       const std::string& source_frame,
                                       const std::string& target_frame,
                                       const Eigen::Vector3d& point,
                                       const rclcpp::Logger& logger);

}

#endif //AVT_341_COORD_TRANSFORM_H
