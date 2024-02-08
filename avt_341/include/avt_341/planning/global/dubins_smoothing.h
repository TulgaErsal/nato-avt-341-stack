/**
 * Path smoothing given a minimum radius of curvature using Dubins paths.
 * Used for filtering sharp turns from global plans created by the A* planner.
 * 
 * Evan Vandermate - evanderm@mtu.edu
*/
#ifndef DUBINSSMOOTHING_H
#define DUBINSSMOOTHING_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "avt_341/node/ros_types.h"
#include "avt_341/avt_341_utils.h"


namespace avt_341 {
namespace planning{

/** Simple circle definition */
struct Circle {
    Circle() {}

    Circle(utils::vec2 center, float radius) :
        center_(center), radius_(radius) {}

    std::vector<utils::vec2> intersects(Circle c) {
        utils::vec2 c0 = center_;
        float r0 = radius_;
        utils::vec2 c1 = c.center_;
        float r1 = c.radius_;

        float d = utils::length(c1-c0);

        if (d > r0+r1 || d < abs(r0-r1) || (d == 0 && r0 == r1))
            return {};    // No solutions
        
        float a = (r0*r0 - r1*r1 + d*d) / 2.0 / d;
        float h = sqrt(r0*r0 - a*a);
        float x2 = c0.x + a*(c1.x-c0.x)/d;
        float y2 = c0.y + a*(c1.y-c0.y)/d;

        return { utils::vec2(x2 + h*(c1.y-c0.y)/d, y2 - h*(c1.x-c0.x)/d),
                 utils::vec2(x2 - h*(c1.y-c0.y)/d, y2 + h*(c1.x-c0.x)/d) };
    }

    utils::vec2 center_;
    float radius_;
};

/** Simple vector pose definition */
struct PathPose {
    PathPose() {}

    PathPose(utils::vec2 origin, utils::vec2 direction) :
        origin_(origin), direction_(direction) {}

    /** Calculate both circles tangent to a pose */
    std::vector<Circle> CalculateTangentCircles(float radius) {
        std::vector<Circle> circles;

        // Calculate 1st circle
        utils::vec2 norm1 = { -direction_.y, direction_.x };
        utils::vec2 center1 = { origin_.x + norm1.x*radius,
                                       origin_.y + norm1.y*radius };
        Circle circle1(center1, radius);
        circles.push_back(circle1);

        // Calculate 2nd circle
        utils::vec2 norm2 = { direction_.y, -direction_.x };
        utils::vec2 center2 = { origin_.x + norm2.x*radius,
                                       origin_.y + norm2.y*radius };
        Circle circle2(center2, radius);
        circles.push_back(circle2);

        return circles;
    }

    utils::vec2 origin_;
    utils::vec2 direction_;
};

/**
 * Path smoothing using Dubins paths.
 */
class DubinsSmoothing {
public:
    DubinsSmoothing(std::vector<std::vector<float>> path) :
        path_original(path) {}

    /** Smooth the input path */
    void SmoothPath(float min_radius, float point_separation);

    /** Retrieves smoothed path if smoothing has been performed, otherwise the original path */
    std::vector<std::vector<float>> GetPath() {
        return IsSmoothed ? path_smoothed : path_original;
    }

private:
    /** Retrieve pose at index with direction estimate from neighboring points */
    PathPose PoseAt(std::vector<std::vector<float>> path, int i);

    /** Finds all points in path ahead that are contained in a circle */
    std::vector<int> PointsAheadInCircle(std::vector<std::vector<float>> path, int i, Circle circle);

    std::vector<std::vector<float>> path_original;
    std::vector<std::vector<float>> path_smoothed;
    bool IsSmoothed = false;
};

/**
 * Dubins path generator
 * Reference: https://gieseanw.wordpress.com/2012/10/21/a-comprehensive-step-by-step-tutorial-to-computing-dubins-paths/
*/
class DubinsPath {
public:
    DubinsPath(PathPose start, PathPose goal, float radius) :
        start_(start), goal_(goal), radius_(radius) {}
    
    /** Create Dubins path from start to goal */
    std::vector<std::vector<float>> CreatePath(float pt_sep);

    /** 
     * Create Dubins path given start/goal poses and a radius 
     * direction < 0    -> RSR
     * direction > 0    -> LSL
     * pt_sep <= 0      -> No points generated
    */
    static void DubinsXSX(std::vector<std::vector<float>>& path_out, float& len_out, 
                            PathPose start, PathPose goal, float radius, float pt_sep, float direction);

    /** 
     * Create Dubins path given start/goal poses and a radius 
     * direction < 0 -> RSL
     * direction > 0 -> LSR
     * pt_sep <= 0      -> No points generated
    */
    static void DubinsXSY(std::vector<std::vector<float>>& path_out, float& len_out, 
                            PathPose start, PathPose goal, float radius, float pt_sep, float direction);

private:
    /** Gets path of points along arc between two vectors */
    static std::vector<std::vector<float>> ArcPoints(utils::vec2 v1, utils::vec2 v2, Circle circle, float direction, float pt_sep);

    /** Gets path of points along line between two points */
    static std::vector<std::vector<float>> LinePoints(utils::vec2 p1, utils::vec2 p2, float pt_sep);

    /** Calculates length of an arc */
    static float ArcLength(utils::vec2 v1, utils::vec2 v2, float radius, float direction) {
        float theta = atan2(v2.y,v2.x) - atan2(v1.y,v1.x);
        if (theta < 0 && direction > 0)
            theta += 2.0*M_PI;
        else if (theta > 0 && direction < 0)
            theta -= 2.0*M_PI;
        return abs(theta*radius);
    }
    
    PathPose start_;
    PathPose goal_;
    float radius_;
};

} // namespace planning
} // namespace avt_341

#endif
