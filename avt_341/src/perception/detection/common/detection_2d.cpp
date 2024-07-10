#include <avt_341/perception/detection/common/detection_2d.hpp>

namespace avt_341 {
namespace perception {

Detection2D::Detection2D(BoundingBox2D bounding_box, Hypothesis hypothesis)
    : bounding_box_(bounding_box),
      hypothesis_(hypothesis) {}

BoundingBox2D Detection2D::GetBoundingBox() { return bounding_box_; }

Hypothesis Detection2D::GetHypothesis() { return hypothesis_; }

vision_msgs::msg::Detection2D Detection2D::ToROSVisionMessage() {
    vision_msgs::msg::Detection2D detection_2d_message;
    detection_2d_message.bbox = bounding_box_.ToROSVisionMessage();
    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis = hypothesis_.ToROSVisionHypothesisMessage();
    detection_2d_message.results.push_back(hypothesis);

    // TODO: Consider using the ID field in this message for tracking purposes.
    detection_2d_message.id = std::to_string(-1);

    return detection_2d_message;
}

} // namespace perception
} // namespace avt_341