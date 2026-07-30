#ifndef AVT_341_COORD_TRANSFORM_H
#define AVT_341_COORD_TRANSFORM_H

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/logger.hpp>
#include <tf2_ros/buffer.h>

#include "avt_341_nav/core/geometry/geometry_dto.hpp"

namespace avt_341_nav::core
{

/**
 * @brief Coordinate transformer bound to a TF buffer and a logger. Holds a
 * non-owning reference to the buffer, which must outlive this object.
 */
class CoordTransformer {
   public:
    CoordTransformer(const tf2_ros::Buffer& buffer,
                     const rclcpp::Logger& logger);

    /**
     * @brief Transforms a point between TF frames. On TF lookup failure the
     * error is logged and the input point is returned unchanged.
     */
    Eigen::Vector3d Transform(const std::string& source_frame,
                              const std::string& target_frame,
                              const Eigen::Vector3d& point) const;

    /**
     * @brief Looks up the rotation from the source to the target frame. On TF
     * lookup failure the error is logged and std::nullopt is returned.
     */
    std::optional<Eigen::Quaterniond> LookupRotation(
        const std::string& source_frame,
        const std::string& target_frame) const;

    /**
     * @brief Transforms the polygon zones to the target frame, waiting up to
     * @p timeout for the transform to become available. Exception will be
     * thrown if TF lookup fails.
     */
    void TransformZones(PolygonZoneCollection& zone_collection,
                        const std::string& target_frame,
                        tf2::Duration timeout = tf2::Duration::zero()) const;

    /**
     * @brief Transforms the points from the source to the target frame in
     * place, waiting up to @p timeout for the transform to become available.
     * Exception will be thrown if TF lookup fails.
     */
    void TransformPoints(std::vector<Eigen::Vector2d>& points,
                         const std::string& source_frame,
                         const std::string& target_frame,
                         tf2::Duration timeout = tf2::Duration::zero()) const;

    /**
     * @brief Transforms the path poses from the path's frame to the target
     * frame in place, waiting up to @p timeout for the transform to become
     * available. Exception will be thrown if TF lookup fails.
     */
    void TransformPath(nav_msgs::msg::Path& path,
                       const std::string& target_frame,
                       tf2::Duration timeout = tf2::Duration::zero()) const;

   private:
    geometry_msgs::msg::TransformStamped LookupTransform(
        const std::string& source_frame, const std::string& target_frame,
        tf2::Duration timeout) const;

    const tf2_ros::Buffer& buffer_;
    rclcpp::Logger logger_;
};

}

#endif //AVT_341_COORD_TRANSFORM_H
