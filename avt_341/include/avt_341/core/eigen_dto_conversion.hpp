/**
* @brief     Conversion utilities between Eigen types and the ROS geometry
             message data transfer objects.
*/

#ifndef AVT_341_EIGEN_DTO_CONVERSION_H
#define AVT_341_EIGEN_DTO_CONVERSION_H

#include <array>
#include <cmath>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace avt_341::core
{

/** @brief Convert an Eigen vector to a geometry_msgs Point message. */
inline geometry_msgs::msg::Point ToPointMsg(const Eigen::Vector3d& vector) {
    geometry_msgs::msg::Point message;
    message.x = vector.x();
    message.y = vector.y();
    message.z = vector.z();
    return message;
}

/** @brief Convert an Eigen vector to a geometry_msgs Vector3 message. */
inline geometry_msgs::msg::Vector3 ToVector3Msg(const Eigen::Vector3d& vector) {
    geometry_msgs::msg::Vector3 message;
    message.x = vector.x();
    message.y = vector.y();
    message.z = vector.z();
    return message;
}

/** @brief Convert an Eigen quaternion to a geometry_msgs Quaternion
 *         message. */
inline geometry_msgs::msg::Quaternion ToQuaternionMsg(
    const Eigen::Quaterniond& quaternion) {
    geometry_msgs::msg::Quaternion message;
    message.w = quaternion.w();
    message.x = quaternion.x();
    message.y = quaternion.y();
    message.z = quaternion.z();
    return message;
}

/** @brief Convert a yaw angle (rotation about +z, in radians) to a
 *         geometry_msgs Quaternion message. */
inline geometry_msgs::msg::Quaternion YawToQuaternionMsg(const double yaw) {
    geometry_msgs::msg::Quaternion message;
    message.w = std::cos(yaw / 2.0);
    message.x = 0.0;
    message.y = 0.0;
    message.z = std::sin(yaw / 2.0);
    return message;
}

/** @brief Convert an Eigen 6x6 covariance matrix to the row-major ROS
 *         covariance array used by PoseWithCovariance and
 *         TwistWithCovariance messages. */
inline std::array<double, 36> ToCovarianceMsg(
    const Eigen::Matrix<double, 6, 6>& covariance) {
    std::array<double, 36> message;
    for (size_t i = 0; i < 6; i++) {
        for (size_t j = 0; j < 6; j++) {
            message[i * 6 + j] = covariance(i, j);
        }
    }
    return message;
}

/** @brief Convert a geometry_msgs Point message to an Eigen vector. */
inline Eigen::Vector3d ToEigen(const geometry_msgs::msg::Point& message) {
    return {message.x, message.y, message.z};
}

/** @brief Convert a geometry_msgs Vector3 message to an Eigen vector. */
inline Eigen::Vector3d ToEigen(const geometry_msgs::msg::Vector3& message) {
    return {message.x, message.y, message.z};
}

/** @brief Convert a geometry_msgs Quaternion message to an Eigen
 *         quaternion. */
inline Eigen::Quaterniond ToEigen(
    const geometry_msgs::msg::Quaternion& message) {
    return {message.w, message.x, message.y, message.z};
}

}

#endif  // AVT_341_EIGEN_DTO_CONVERSION_H
