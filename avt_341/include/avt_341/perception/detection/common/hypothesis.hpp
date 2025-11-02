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

* @file      hypothesis.hpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Header file for an object hypothesis.
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
     * @brief Construct a new hypothesis with a given class ID and confidence score.
     *
     * @param id Hypothesis class ID.
     * @param score Hypothesis confidence score.
     */
    Hypothesis(const std::string& id, double score);

    /**
     * @brief Get the hypothesis class ID.
     *
     * @return int Hypothesis class ID.
     */
    const std::string& GetID();

    /**
     * @brief Get the hypothesis confidence score.
     *
     * @return double Hypothesis confidence score.
     */
    double GetScore();

    /**
     * @brief Serialize the hypothesis to a ROS vision_msgs/Hypothesis message.
     *
     * @return vision_msgs::msg::ObjectHypothesis ROS vision_msgs/Hypothesis message matching the hypothesis.
     */
    vision_msgs::msg::ObjectHypothesis ToROSVisionHypothesisMessage();

  private:
    /** @brief Hypothesis class ID. */
    std::string id_ = "";

    /** @brief Hypothesis confidence score. */
    double score_ = 0.0;
};

} // namespace perception
} // namespace avt_341