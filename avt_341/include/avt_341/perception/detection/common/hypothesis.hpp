#pragma once

#include <stdexcept>

#include <vision_msgs/msg/object_hypothesis.hpp>

namespace avt_341 {
namespace perception {

/**
 * @brief Hypothesis for a detected object.
 */
class Hypothesis {
  public:
    /**
     * @brief Construct a degenerate (invalid ID and null score) hypothesis.
     */
    Hypothesis();

    /**
     * @brief Construct a new hypothesis with a given class ID and confidence
     * score.
     *
     * @param id Hypothesis class ID.
     * @param score Hypothesis confidence score.
     */
    Hypothesis(int id, double score);

    /**
     * @brief Get the hypothesis class ID.
     *
     * @return int Hypothesis class ID.
     */
    int GetID();

    /**
     * @brief Get the hypothesis confidence score.
     *
     * @return double Hypothesis confidence score.
     */
    double GetScore();

    /**
     * @brief Serialize the hypothesis to a ROS vision_msgs/Hypothesis message.
     *
     * @return vision_msgs::msg::ObjectHypothesis ROS vision_msgs/Hypothesis
     * message matching the hypothesis.
     */
    vision_msgs::msg::ObjectHypothesis ToROSVisionHypothesisMessage();

  private:
    /** @brief Hypothesis class ID. */
    int id_ = -1;

    /** @brief Hypothesis confidence score. */
    double score_ = 0.0;
};

} // namespace perception
} // namespace avt_341