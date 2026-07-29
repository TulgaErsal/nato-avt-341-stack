/**
* @file      math_utils.hpp
* @brief     Scalar math helpers.
*/

#ifndef AVT_341_CORE_MATH_UTILS_H
#define AVT_341_CORE_MATH_UTILS_H

#include <cmath>

namespace avt_341_nav::core
{

/** @brief Signed difference between two angles in radians, wrapped to (-pi, pi]. */
inline double DiffAngle(const double a, const double b) {
    constexpr double two_pi = 2.0 * M_PI;
    double diff = a - b;
    diff -= two_pi * floor((diff + M_PI) / two_pi);
    return diff;
}

/** @brief Signed difference between two angles in degrees, wrapped to (-180, 180]. */
inline double DiffDeg(const double a, const double b) {
    constexpr double s = M_PI / 180.0;
    return DiffAngle(a * s, b * s) / s;
}

}

#endif  // AVT_341_CORE_MATH_UTILS_H
