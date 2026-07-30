#include <avt_341_nav/core/coord_transform.hpp>

#ifdef GTE_ROS_HUMBLE
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>

#include <avt_341_nav/core/eigen_dto_conversion.hpp>

namespace avt_341_nav::core
{

CoordTransformer::CoordTransformer(const tf2_ros::Buffer& buffer,
                                   const rclcpp::Logger& logger)
    : buffer_(buffer), logger_(logger) {}

Eigen::Vector3d CoordTransformer::Transform(const std::string& source_frame,
                                            const std::string& target_frame,
                                            const Eigen::Vector3d& point) const {
    geometry_msgs::msg::TransformStamped transform_message;
    try {
        transform_message = buffer_.lookupTransform(
            target_frame.c_str(), source_frame.c_str(), tf2::TimePointZero);
    } catch (tf2::TransformException& exception) {
        RCLCPP_ERROR(logger_, "Transform lookup %s -> %s failed: %s",
                     source_frame.c_str(), target_frame.c_str(),
                     exception.what());
        return point;
    }

    Eigen::Vector3d transformed_point;
    tf2::doTransform(point, transformed_point, transform_message);
    return transformed_point;
}

std::optional<Eigen::Quaterniond> CoordTransformer::LookupRotation(
    const std::string& source_frame, const std::string& target_frame) const {
    geometry_msgs::msg::TransformStamped transform_message;
    try {
        transform_message = buffer_.lookupTransform(
            target_frame.c_str(), source_frame.c_str(), tf2::TimePointZero);
    } catch (tf2::TransformException& exception) {
        RCLCPP_ERROR(logger_, "Transform lookup %s -> %s failed: %s",
                     source_frame.c_str(), target_frame.c_str(),
                     exception.what());
        return std::nullopt;
    }

    return ToEigen(transform_message.transform.rotation);
}

geometry_msgs::msg::TransformStamped CoordTransformer::LookupTransform(
    const std::string& source_frame, const std::string& target_frame,
    const tf2::Duration timeout) const {
    try {
        return (timeout > tf2::Duration::zero())
            ? buffer_.lookupTransform(
                target_frame.c_str(), source_frame.c_str(), tf2::TimePointZero, timeout)
            : buffer_.lookupTransform(
                target_frame.c_str(), source_frame.c_str(), tf2::TimePointZero);
    } catch (tf2::TransformException& exception) {
        throw std::runtime_error("Transform lookup " + source_frame + " -> " +
                                 target_frame + " failed: " + exception.what());
    }
}

void CoordTransformer::TransformZones(PolygonZoneCollection& zone_collection,
                                      const std::string& target_frame,
                                      const tf2::Duration timeout) const {
    if (zone_collection.frame.empty() || zone_collection.frame == target_frame) {
        return;
    }

    for (PolygonZone& zone : zone_collection.zones) {
        TransformPoints(zone.vertices, zone_collection.frame, target_frame, timeout);
    }
    zone_collection.frame = target_frame;
}

void CoordTransformer::TransformPoints(std::vector<Eigen::Vector2d>& points,
                                       const std::string& source_frame,
                                       const std::string& target_frame,
                                       const tf2::Duration timeout) const {
    if (source_frame.empty() || source_frame == target_frame) {
        return;
    }

    const geometry_msgs::msg::TransformStamped transform_message =
        LookupTransform(source_frame, target_frame, timeout);
    for (Eigen::Vector2d& point : points) {
        Eigen::Vector3d transformed_point;
        tf2::doTransform(Eigen::Vector3d(point.x(), point.y(), 0.0),
                         transformed_point, transform_message);
        point = transformed_point.head<2>();
    }
}

void CoordTransformer::TransformPath(nav_msgs::msg::Path& path,
                                     const std::string& target_frame,
                                     const tf2::Duration timeout) const {
    if (path.header.frame_id.empty() || path.header.frame_id == target_frame) {
        return;
    }

    if (!path.poses.empty())
    {
        const geometry_msgs::msg::TransformStamped transform_message =
            LookupTransform(path.header.frame_id, target_frame, timeout);
        for (geometry_msgs::msg::PoseStamped& pose : path.poses) {
            tf2::doTransform(pose.pose, pose.pose, transform_message);
            pose.header.frame_id = target_frame;
        }
    }

    path.header.frame_id = target_frame;
}

}
