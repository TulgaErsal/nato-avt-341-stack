#ifndef TF_UTILS_H
#define TF_UTILS_H

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace avt_341 {
namespace rviz_plugins {

/// Yaw (rotation about +Z), in radians, extracted from a quaternion message.
/// Delegates to tf2::getYaw, which converts the message to a tf2::Quaternion and
/// reads the yaw off its rotation matrix.
inline double yawOf( const geometry_msgs::msg::Quaternion& q )
{
    return tf2::getYaw( q );
}

} // end namespace rviz_plugins
} // end namespace avt_341

#endif //TF_UTILS_H
