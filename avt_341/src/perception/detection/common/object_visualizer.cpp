/**
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +                      _    _    _    _    _    _    _                      +
 +                     / \  / \  / \  / \  / \  / \  / \                     +
 +                    ( A )( V )( T )( - )( 3 )( 4 )( 1 )                    +
 +                     \_/  \_/  \_/  \_/  \_/  \_/  \_/                     +
 +       _    _    _    _    _    _    _    _     _    _    _    _    _      +
 +      / \  / \  / \  / \  / \  / \  / \  / \   / \  / \  / \  / \  / \     +
 +     ( A )( U )( T )( O )( N )( O )( M )( Y ) ( S )( T )( A )( C )( K )    +
 +      \_/  \_/  \_/  \_/  \_/  \_/  \_/  \_/   \_/  \_/  \_/  \_/  \_/     +
 +                                                                           +
 +  AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles  +
 +                                                                           +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

* @file      object_visualizer.cpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Source file for an object detection overlay visualizer.
* @copyright MIT License

             NATO AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles
             Copyright (c) 2024 Dario Sirangelo (dsi@aarhusrobotics.com).

             NOTE: The above copyright only applies to the contents of this file. The source code contained in this file
             is a direct port from the GitHub repository aarhus-robotics/navi, released by the copyright holder under
             the MIT license.

             Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
             associated documentation files (the "Software"), to deal in the Software without restriction, including
             without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
             copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
             following conditions:

             The above copyright notice and this permission notice shall be included in all copies or substantial
             portions of the Software.

             THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
             LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
             EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
             IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
             THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <avt_341/perception/detection/common/object_visualizer.hpp>

namespace avt_341 {
namespace perception {

ObjectVisualizer::ObjectVisualizer() {}

void ObjectVisualizer::SetClasses(std::vector<std::string>& classes) { classes_ = classes; }

void ObjectVisualizer::Overlay(cv::Mat& image, std::vector<Detection2D> detections) {
    for(auto& detection : detections) {
        auto bounding_box = detection.GetBoundingBox();
        auto hypothesis = detection.GetHypothesis();
        DrawHypothesis(image, bounding_box, hypothesis);
    }
}

void ObjectVisualizer::DrawHypothesis(cv::Mat& image, BoundingBox2D& bounding_box, Hypothesis& hypothesis) {
    // Define the bounding box rectangle
    cv::Rect overlay_box(bounding_box.GetXMin(),
                         bounding_box.GetYMin(),
                         bounding_box.GetWidth(),
                         bounding_box.GetHeight());

    std::stringstream stream;
    stream << classes_[GetIndex(hypothesis.GetID())] << " ";
    stream << std::fixed << std::setprecision(2) << hypothesis.GetScore();

    // Get text box size
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(stream.str(), cv::FONT_HERSHEY_DUPLEX, font_scale_, 1, &baseline);

    // Define anchor point for text
    cv::Point anchor_text(bounding_box.GetXMin(), bounding_box.GetYMin() - border_size_);

    // Draw the bounding box
    cv::rectangle(image, overlay_box, colors_[GetIndex(hypothesis.GetID())], border_size_, cv::LINE_8);

    // Define the bounding box rectangle
    cv::Rect overlay_textbox(bounding_box.GetXMin() - border_size_,
                             bounding_box.GetYMin() - text_size.height - 2 * border_size_,
                             text_size.width + 2 * border_size_,
                             text_size.height + 2 * border_size_);

    cv::Scalar text_color;
    if(use_textbox_) {
        text_color = cv::Scalar(255, 255, 255);
        // Draw the bounding box
        cv::rectangle(image, overlay_textbox, colors_[GetIndex(hypothesis.GetID())], cv::FILLED);
    } else {
        text_color = cv::Scalar(colors_[GetIndex(hypothesis.GetID())]);
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
    std::shuffle(std::begin(colors_), std::end(colors_), random_number_generator);
}

void ObjectVisualizer::UseTextbox(bool use_textbox) { use_textbox_ = use_textbox; }

void ObjectVisualizer::SetFontScale(double font_scale) { font_scale_ = font_scale; }

void ObjectVisualizer::SetBorderSize(int border_size) { border_size_ = border_size; }

unsigned int ObjectVisualizer::GetIndex(const std::string& label) {
    auto iterator = std::find(classes_.begin(), classes_.end(), label);
    return iterator - classes_.begin();
}

} // namespace perception
} // namespace avt_341