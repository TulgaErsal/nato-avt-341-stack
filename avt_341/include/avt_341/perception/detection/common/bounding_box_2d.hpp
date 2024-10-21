#pragma once

#include <cmath>

#include <sensor_msgs/msg/region_of_interest.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>

namespace avt_341 {
namespace perception {

/**
 * @brief A two-dimensional bounding box.
 * @details Bounding boxes can be degenerate, e.g. have one or more sides with
 * null length. Be aware that the default constructor of this class provides a
 * degenerate bounding box with null width and height.
 */
class BoundingBox2D {
  public:
    /**
     * @brief Create a new degenerate bounding box.
     */
    BoundingBox2D();

    /**
     * @brief Create a new two-dimensional bounding box.
     *
     * @param x_min Bounding box minimum coordinate in the horizontal direction.
     * @param x_max Bounding box maximum coordinate in the horizontal direction.
     * @param y_min Bounding box minimum coordinate in the vertical direction.
     * @param y_max Bounding box maximum coordinate in the vertical direction.
     */
    BoundingBox2D(const int x_min,
                  const int x_max,
                  const int y_min,
                  const int y_max);

    /**
     * @brief Get the x coordinate of the bounding box center point, in pixel
     * coordinates and floored to the nearest integer.
     *
     * @return int X coordinate of the bounding box center point.
     */
    int GetCenterX();

    /**
     * @brief Get the y coordinate of the bounding box center point, in pixel
     * coordinates and floored to the nearest integer.
     *
     * @return int Y coordinate of the bounding box center point.
     */
    int GetCenterY();

    /**
     * @brief Get bounding box width.
     *
     * @return int Bounding box height in pixels.
     */
    int GetHeight();

    /**
     * @brief Get the bounding box width.
     *
     * @return int Bounding box width in pixels.
     */
    int GetWidth();

    /**
     * @brief Get the bounding box maximum X coordinate.
     *
     * @return int Bounding box maximum X coordinate in pixels.
     */
    int GetXMax();

    /**
     * @brief Get the bounding box minimum X coordinate.
     *
     * @return int Bounding box minimum X coordinate in pixels.
     */
    int GetXMin();

    /**
     * @brief Get the bounding box maximum Y coordinate.
     *
     * @return int Bounding box maximum Y coordinate in pixels.
     */
    int GetYMax();

    /**
     * @brief Get the bounding box minimum Y coordinate.
     *
     * @return int Bounding box minimum Y coordinate in pixels.
     */
    int GetYMin();

    /**
     * @brief Serialize the two-dimensional bounding box to a ROS
     * sensor_msgs/RegionOfInterest message.
     *
     * @return sensor_msgs::msg::RegionOfInterest ROS
     * sensor_msgs/RegionOfInterest message matching the bounding box.
     */
    sensor_msgs::msg::RegionOfInterest ToROSRegionOfInterestMessage();

    /**
     * @brief Serialize the two-dimensional bounding box to a ROS
     * vision_msgs/BoundingBox2D message.
     *
     * @return vision_msgs::msg::BoundingBox2D ROS vision_msgs/BoundingBox2D
     * message matching the bounding box.
     */
    vision_msgs::msg::BoundingBox2D ToROSVisionMessage();

  private:
    /* @brief Bounding box minimum coordinate in the horizontal direction. */
    int x_min_ = 0.0;

    /* @brief Bounding box maximum coordinate in the horizontal direction. */
    int x_max_ = 0.0;

    /* @brief Bounding box minimum coordinate in the vertical direction. */
    int y_min_ = 0.0;

    /* @brief Bounding box maximum coordinate in the vertical direction. */
    int y_max_ = 0.0;
};

} // namespace perception
} // namespace avt_341