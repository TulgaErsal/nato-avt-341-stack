#include <avt_341/core/coord_transform.hpp>

#ifdef GTE_ROS_HUMBLE
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>

namespace avt_341::core
{

Eigen::Vector3d TransformToCoordinates(const tf2_ros::Buffer& buffer,
                                       const std::string& source_frame,
                                       const std::string& target_frame,
                                       const Eigen::Vector3d& point,
                                       const rclcpp::Logger& logger) {
    geometry_msgs::msg::TransformStamped transform_message;
    try {
        transform_message = buffer.lookupTransform(
            target_frame.c_str(), source_frame.c_str(), tf2::TimePointZero);
    } catch (tf2::TransformException& exception) {
        RCLCPP_ERROR(logger, "Transform lookup %s -> %s failed: %s",
                     source_frame.c_str(), target_frame.c_str(),
                     exception.what());
        return point;
    }

    Eigen::Vector3d transformed_point;
    tf2::doTransform(point, transformed_point, transform_message);
    return transformed_point;
}

}
