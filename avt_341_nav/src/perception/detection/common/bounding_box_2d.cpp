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

* @file      bounding_box_2d.cpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Source file for a two-dimensional bounding box class.
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

#include <avt_341/perception/detection/common/bounding_box_2d.hpp>

namespace avt_341 {
namespace perception {

BoundingBox2D::BoundingBox2D() {}

BoundingBox2D::BoundingBox2D(const int x_min, const int x_max, const int y_min, const int y_max)
    : x_min_(x_min),
      x_max_(x_max),
      y_min_(y_min),
      y_max_(y_max) {}

sensor_msgs::msg::RegionOfInterest BoundingBox2D::ToROSRegionOfInterestMessage() {
    sensor_msgs::msg::RegionOfInterest message;
    message.x_offset = x_min_;
    message.y_offset = y_min_;
    message.height = GetHeight();
    message.width = GetWidth();
    message.do_rectify = false;
    return message;
}

vision_msgs::msg::BoundingBox2D BoundingBox2D::ToROSVisionMessage() {
    vision_msgs::msg::BoundingBox2D message;
    message.center.position.x = GetCenterX();
    message.center.position.y = GetCenterY();
    // Bounding boxes from YOLO object detectors do not allow for rotation, hence the bounding box angle is hardcoded to
    // null.
    message.center.theta = 0.0;
    message.size_x = GetWidth();
    message.size_y = GetHeight();
    return message;
}

int BoundingBox2D::GetCenterX() { return std::floor((x_min_ + x_max_) / 2.0); }

int BoundingBox2D::GetCenterY() { return std::floor((y_min_ + y_max_) / 2.0); }

int BoundingBox2D::GetHeight() { return y_max_ - y_min_; }

int BoundingBox2D::GetWidth() { return x_max_ - x_min_; }

int BoundingBox2D::GetXMax() { return x_max_; }

int BoundingBox2D::GetXMin() { return x_min_; }

int BoundingBox2D::GetYMax() { return y_max_; }

int BoundingBox2D::GetYMin() { return y_min_; }

} // namespace perception
} // namespace avt_341