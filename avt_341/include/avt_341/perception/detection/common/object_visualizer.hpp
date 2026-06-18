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

* @file      object_visualizer.hpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Header file for an object detection overlay visualizer.
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

#include <algorithm>
#include <random>

#include <opencv2/opencv.hpp>

#include <avt_341/perception/detection/common/detection_2d.hpp>

namespace avt_341 {
namespace perception {

class ObjectVisualizer {
  public:
    ObjectVisualizer();

    void SetClasses(std::vector<std::string>& classes);

    void Overlay(cv::Mat& image, std::vector<Detection2D> detections);

    void DrawHypothesis(cv::Mat& image, BoundingBox2D& bounding_box, Hypothesis& hypothesis);

    void ShuffleColors(int seed);

    void UseTextbox(bool use_textbox);

    void SetFontScale(double font_scale);

    void SetBorderSize(int border_size);

    unsigned int GetIndex(const std::string& label);

  private:
    std::vector<std::string> classes_;

    std::vector<cv::Scalar> colors_ = std::vector<cv::Scalar>{
        cv::Scalar(105, 105, 105), // Dim gray (#696969)
        cv::Scalar(169, 169, 169), // Dark gray (#a9a9a9)
        cv::Scalar(220, 220, 220), // Gainsboro (#dcdcdc)
        cv::Scalar(47, 79, 79), // Dark slate gray (#2f4f4f)
        cv::Scalar(85, 107, 47), // Dark olive green (#556b2f)
        cv::Scalar(139, 69, 19), // Saddle brown (#8b4513)
        cv::Scalar(46, 139, 87), // Sea green (#2e8b57)
        cv::Scalar(34, 139, 34), // Forest green (#228b22)
        cv::Scalar(25, 25, 112), // Midnight blue (#191970)
        cv::Scalar(139, 0, 0), // Dark red (#8b0000)
        cv::Scalar(128, 128, 0), // Olive (#808000)
        cv::Scalar(72, 61, 139), // Dark slate blue (#483d8b)
        cv::Scalar(188, 143, 143), // Rosy brown (#bc8f8f)
        cv::Scalar(102, 51, 153), // Rebecca purple (#663399)
        cv::Scalar(189, 183, 107), // Dark khaki (#bdb76b)
        cv::Scalar(0, 139, 139), // Dark cyan (#008b8b)
        cv::Scalar(205, 133, 63), // Peru (#cd853f)
        cv::Scalar(70, 130, 180), // Steel blue (#4682b4)
        cv::Scalar(210, 105, 30), // Chocolate (#d2691e)
        cv::Scalar(154, 205, 50), // Yellow green (#9acd32)
        cv::Scalar(0, 0, 139), // Dark blue (#00008b)
        cv::Scalar(50, 205, 50), // Lime green (#32cd32)
        cv::Scalar(218, 165, 32), // Golden rod (#daa520)
        cv::Scalar(143, 188, 143), // Dark sea green (#8fbc8f)
        cv::Scalar(128, 0, 128), // Purple (#800080)
        cv::Scalar(176, 48, 96), // Maroon 3 (#b03060)
        cv::Scalar(210, 180, 140), // Tan (#d2b48c)
        cv::Scalar(102, 205, 170), // Medium aquamarine (#66cdaa)
        cv::Scalar(255, 69, 0), // Orange red (#ff4500)
        cv::Scalar(0, 206, 209), // Dark turquoise (#00ced1)
        cv::Scalar(255, 165, 0), // Orange (#ffa500)
        cv::Scalar(255, 215, 0), // Gold (#ffd700)
        cv::Scalar(199, 21, 133), // Medium violet red (#c71585)
        cv::Scalar(0, 0, 205), // Medium blue (#0000cd)
        cv::Scalar(127, 255, 0), // Chart reuse (#7fff00)
        cv::Scalar(0, 255, 0), // Lime (#00ff00)
        cv::Scalar(186, 85, 211), // Medium orchid (#ba55d3)
        cv::Scalar(0, 250, 154), // Medium spring green (#00fa9a)
        cv::Scalar(138, 43, 226), // Blue violet (#8a2be2)
        cv::Scalar(65, 105, 225), // Royal blue (#4169e1)
        cv::Scalar(220, 20, 60), // Crimson (#dc143c)
        cv::Scalar(0, 255, 255), // Aqua (#00ffff)
        cv::Scalar(0, 191, 255), // Deep sky blue (#00bfff)
        cv::Scalar(147, 112, 219), // Medium purple (#9370db)
        cv::Scalar(0, 0, 255), // Blue (#0000ff)
        cv::Scalar(240, 128, 128), // Light coral (#f08080)
        cv::Scalar(255, 99, 71), // Tomato (#ff6347)
        cv::Scalar(216, 191, 216), // Thistle (#d8bfd8)
        cv::Scalar(255, 0, 255), // Fuchsia (#ff00ff)
        cv::Scalar(30, 144, 255), // Dodger blue (#1e90ff)
        cv::Scalar(219, 112, 147), // Pale violet red (#db7093)
        cv::Scalar(240, 230, 140), // Khaki (#f0e68c)
        cv::Scalar(255, 255, 84), // Laser lemon (#ffff54)
        cv::Scalar(221, 160, 221), // Plum (#dda0dd)
        cv::Scalar(135, 206, 235), // Sky blue (#87ceeb)
        cv::Scalar(255, 20, 147), // Deep pink (#ff1493)
        cv::Scalar(255, 160, 122), // Light salmon (#ffa07a)
        cv::Scalar(175, 238, 238), // Pale turquoise (#afeeee)
        cv::Scalar(238, 130, 238), // Violet (#ee82ee)
        cv::Scalar(152, 251, 152), // Pale green (#98fb98)
        cv::Scalar(127, 255, 212), // Aquamarine (#7fffd4)
        cv::Scalar(255, 105, 180), // Hot pink (#ff69b4)
        cv::Scalar(255, 228, 196), // Bisque (#ffe4c4)
        cv::Scalar(255, 182, 193) // Light pink (#ffb6c1)
    };

    bool use_textbox_ = false;

    double font_scale_ = 1.0;

    int border_size_ = 2;
};

} // namespace perception
} // namespace avt_341