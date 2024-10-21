#include <avt_341/perception/detection/common/object_visualizer.hpp>

namespace avt_341 {
namespace perception {

ObjectVisualizer::ObjectVisualizer() {}

void ObjectVisualizer::SetClasses(std::vector<std::string>& classes) {
    classes_ = classes;
}

void ObjectVisualizer::Overlay(cv::Mat& image,
                               std::vector<Detection2D> detections) {
    for(auto& detection : detections) {
        auto bounding_box = detection.GetBoundingBox();
        auto hypothesis = detection.GetHypothesis();
        DrawHypothesis(image, bounding_box, hypothesis);
    }
}

void ObjectVisualizer::DrawHypothesis(cv::Mat& image,
                                      BoundingBox2D& bounding_box,
                                      Hypothesis& hypothesis) {
    // Define the bounding box rectangle
    cv::Rect overlay_box(bounding_box.GetXMin(),
                         bounding_box.GetYMin(),
                         bounding_box.GetWidth(),
                         bounding_box.GetHeight());

    std::stringstream stream;
    stream << classes_[hypothesis.GetID()] << " ";
    stream << std::fixed << std::setprecision(2) << hypothesis.GetScore();

    // Get text box size
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(stream.str(),
                                         cv::FONT_HERSHEY_DUPLEX,
                                         font_scale_,
                                         1,
                                         &baseline);

    // Define anchor point for text
    cv::Point anchor_text(bounding_box.GetXMin(),
                          bounding_box.GetYMin() - border_size_);

    // Draw the bounding box
    cv::rectangle(image,
                  overlay_box,
                  colors_[hypothesis.GetID()],
                  border_size_,
                  cv::LINE_8);

    // Define the bounding box rectangle
    cv::Rect overlay_textbox(bounding_box.GetXMin() - border_size_,
                             bounding_box.GetYMin() - text_size.height -
                                 2 * border_size_,
                             text_size.width + 2 * border_size_,
                             text_size.height + 2 * border_size_);

    cv::Scalar text_color;
    if(use_textbox_) {
        text_color = cv::Scalar(255, 255, 255);
        // Draw the bounding box
        cv::rectangle(image,
                      overlay_textbox,
                      colors_[hypothesis.GetID()],
                      cv::FILLED);
    } else {
        text_color = cv::Scalar(colors_[hypothesis.GetID()]);
    }

    cv::putText(image,
                stream.str(),
                anchor_text,
                cv::FONT_HERSHEY_DUPLEX,
                font_scale_,
                text_color,
                1,
                cv::LINE_8,
                false);
}

void ObjectVisualizer::ShuffleColors(int seed) {
    auto random_number_generator = std::default_random_engine(seed);
    std::shuffle(std::begin(colors_),
                 std::end(colors_),
                 random_number_generator);
}

void ObjectVisualizer::UseTextbox(bool use_textbox) {
    use_textbox_ = use_textbox;
}

void ObjectVisualizer::SetFontScale(double font_scale) {
    font_scale_ = font_scale;
}

void ObjectVisualizer::SetBorderSize(int border_size) {
    border_size_ = border_size;
}

} // namespace perception
} // namespace avt_341