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

* @file      detection_2d.cpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Source file for a two-dimensional object detection.
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

#include <avt_341_nav/perception/detection/common/detection_2d.hpp>

namespace avt_341_nav {
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
    detection_2d_message.id = std::to_string(-1);

    return detection_2d_message;
}

} // namespace perception
} // namespace avt_341_nav