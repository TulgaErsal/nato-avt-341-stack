#pragma once

#include <vision_msgs/msg/detection2_d.hpp>

#include <avt_341/perception/detection/common/bounding_box_2d.hpp>
#include <avt_341/perception/detection/common/hypothesis.hpp>

namespace avt_341 {
namespace perception {

/**
 * @brief A detection on a two-dimensional image.
 */
class Detection2D {
  public:
    /**
     * @brief Construct a new two-dimensional detection.
     *
     * @param bounding_box Two-dimensional detection bounding box.
     * @param hypothesis Detection hypothesis.
     */
    Detection2D(BoundingBox2D bounding_box, Hypothesis hypothesis);

    /**
     * @brief Get the detection bounding box.
     *
     * @return BoundingBox2D Detection two-dimensional bounding box.
     */
    BoundingBox2D GetBoundingBox();

    /**
     * @brief Get the detection hypothesis.
     *
     * @return Hypothesis Detection hypothesis.
     */
    Hypothesis GetHypothesis();

    /**
     * @brief Serialize the two-dimensional detection to a ROS
     * vision_msgs/Detection2D message.
     *
     * @return vision_msgs::msg::Detection2D ROS vision_msgs/Detection2D message
     * matching the detection.
     */
    vision_msgs::msg::Detection2D ToROSVisionMessage();

  private:
    /** @brief Detection bounding box. */
    BoundingBox2D bounding_box_;

    /** @brief Detection hypothesis. */
    Hypothesis hypothesis_;
};

} // namespace perception
} // namespace avt_341