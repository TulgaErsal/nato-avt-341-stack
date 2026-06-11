#ifndef AVT_341_COORD_TRANSFORM_H
#define AVT_341_COORD_TRANSFORM_H

#include <optional>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <tf2_ros/buffer.h>

namespace avt_341::core
{

/**
 * @brief Coordinate transformer bound to a TF buffer and a logger, so that
 * callers do not need to thread both through every transform call.
 *
 * Holds a non-owning reference to the TF buffer: the buffer must outlive
 * this object.
 */
class CoordTransformer {
   public:
    /**
     * @param buffer Transform buffer to look transforms up in (non-owning).
     * @param logger Logger used to report TF lookup failures.
     */
    CoordTransformer(const tf2_ros::Buffer& buffer,
                     const rclcpp::Logger& logger);

    /**
     * @brief Transform a three-dimensional point between TF frames using the
     * latest available transform (tf2::TimePointZero).
     *
     * On a TF lookup failure the error is logged and the input point is
     * returned unchanged.
     *
     * @param source_frame Frame ID the point is currently expressed in.
     * @param target_frame Frame ID to transform the point into.
     * @param point Point to be transformed.
     * @return Eigen::Vector3d Transformed point.
     */
    Eigen::Vector3d Transform(const std::string& source_frame,
                              const std::string& target_frame,
                              const Eigen::Vector3d& point) const;

    /**
     * @brief Look up the rotation from @p source_frame to @p target_frame
     * using the latest available transform (tf2::TimePointZero).
     *
     * On a TF lookup failure the error is logged and std::nullopt is
     * returned.
     *
     * @param source_frame Frame ID the rotation is from.
     * @param target_frame Frame ID the rotation is into.
     * @return The rotation quaternion, or std::nullopt on lookup failure.
     */
    std::optional<Eigen::Quaterniond> LookupRotation(
        const std::string& source_frame,
        const std::string& target_frame) const;

   private:
    const tf2_ros::Buffer& buffer_;
    rclcpp::Logger logger_;
};

}

#endif //AVT_341_COORD_TRANSFORM_H
