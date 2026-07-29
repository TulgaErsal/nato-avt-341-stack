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

* @file      detection_2d.hpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Header file for a two-dimensional object detection.
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

#pragma once

#include <vision_msgs/msg/detection2_d.hpp>

#include <avt_341_nav/perception/detection/common/bounding_box_2d.hpp>
#include <avt_341_nav/perception/detection/common/hypothesis.hpp>

namespace avt_341_nav {
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
     * @brief Serialize the two-dimensional detection to a ROS vision_msgs/Detection2D message.
     *
     * @return vision_msgs::msg::Detection2D ROS vision_msgs/Detection2D message matching the detection.
     */
    vision_msgs::msg::Detection2D ToROSVisionMessage();

  private:
    /** @brief Detection bounding box. */
    BoundingBox2D bounding_box_;

    /** @brief Detection hypothesis. */
    Hypothesis hypothesis_;
};

} // namespace perception
} // namespace avt_341_nav