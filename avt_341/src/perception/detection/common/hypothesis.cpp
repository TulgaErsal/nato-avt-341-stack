#include <avt_341/perception/detection/common/hypothesis.hpp>

namespace avt_341 {
namespace perception {

Hypothesis::Hypothesis() {}

Hypothesis::Hypothesis(int id, double score) : id_(id), score_(score) {
    if(id < 0) {
        throw std::invalid_argument(
            "Hypothesis ID must be equal to or larger than zero.");
    }

    if(score < 0 || score > 1.0) {
        throw std::invalid_argument(
            "Hypothesis score must be a double precision floating point number "
            "between 0.0 and 1.0.");
    }
}

int Hypothesis::GetID() { return id_; }

double Hypothesis::GetScore() { return score_; }

vision_msgs::msg::ObjectHypothesis Hypothesis::ToROSVisionHypothesisMessage() {
    vision_msgs::msg::ObjectHypothesis message;
    message.class_id = std::to_string(id_);
    message.score = score_;
    return message;
}

} // namespace perception
} // namespace avt_341