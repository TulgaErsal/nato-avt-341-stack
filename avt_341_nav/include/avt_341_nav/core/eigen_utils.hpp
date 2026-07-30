/**
* @file      eigen_utils.hpp
* @brief     Small shared math helpers.
*/

#ifndef AVT_341_CORE_EIGEN_UTILS_H
#define AVT_341_CORE_EIGEN_UTILS_H

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace avt_341_nav::core
{

/** @brief Ray-casting point-in-polygon test (works for non-convex polygons).
 *         https://en.wikipedia.org/wiki/Point_in_polygon */
inline bool IsInsidePolygon(const std::vector<Eigen::Vector2d>& poly, double px, double py)
{
    bool inside = false;
    const int n = static_cast<int>(poly.size());
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const double xi = poly[i].x(), yi = poly[i].y();
        const double xj = poly[j].x(), yj = poly[j].y();
        if (((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
            inside = !inside;
            }
    }
    return inside;
}

/** @brief Standard deviation along the major axis of the uncertainty
 *         ellipse of a symmetric 2x2 covariance: sqrt of the largest
 *         eigenvalue (closed form). */
inline double MajorAxisStdDev2x2(const Eigen::Matrix2d& p) {
    const double a = p(0, 0);
    const double c = p(1, 1);
    const double b = 0.5 * (p(0, 1) + p(1, 0));
    const double largest_eigenvalue =
        0.5 * (a + c) + std::sqrt(0.25 * (a - c) * (a - c) + b * b);
    return std::sqrt(std::max(0.0, largest_eigenvalue));
}

}

#endif  // AVT_341_CORE_EIGEN_UTILS_H
