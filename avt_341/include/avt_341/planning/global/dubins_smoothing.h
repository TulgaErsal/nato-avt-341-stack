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

namespace avt_341 {
namespace planning{

/** Simple circle definition */
struct Circle {
    Circle() {}

    Circle(std::vector<float> center, float radius) :
        center_(center), radius_(radius) {}

    float ArcLength(std::vector<float> v0, std::vector<float> v1, float direction) {
        float theta = std::atan2(v1[1], v1[0]) - std::atan2(v0[1], v0[0]);
        if (theta < 0 && direction > 0)
            theta += 2.0*M_PI;
        else if (theta > 0 && direction < 0)
            theta += 2.0*M_PI;
        return theta*radius_;
    }

    std::vector<float> center_;
    float radius_;
};

/** Simple vector pose definition */
struct PathPose {
    PathPose() {}

    PathPose(std::vector<float> origin, std::vector<float> direction) :
        origin_(origin), direction_(direction) {}

    /** Calculate both circles tangent to a pose */
    std::vector<Circle> CalculateTangentCircles(float radius) {
        std::vector<Circle> circles;

        // Calculate 1st circle
        std::vector<float> norm1 = { -direction_[1], direction_[0] };
        std::vector<float> center1 = { origin_[0] + norm1[0]*radius,
                                       origin_[1] + norm1[1]*radius };
        Circle circle1(center1, radius);
        circles.push_back(circle1);

        // Calculate 2nd circle
        std::vector<float> norm2 = { direction_[1], -direction_[0] };
        std::vector<float> center2 = { origin_[0] + norm2[0]*radius,
                                       origin_[1] + norm2[1]*radius };
        Circle circle2(center2, radius);
        circles.push_back(circle2);

        return circles;
    }

    std::vector<float> origin_;
    std::vector<float> direction_;
};

class VectorMath {
public:
    /** 2D distance between points */
    static float PointDistance(std::vector<float> p1, std::vector<float> p2) {
        return std::sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]));
    }

    /** Calculate magnitude of vector */
    static float Norm(std::vector<float> v) {
        float norm = 0;
        for (float i : v)
            norm += i*i;
        return std::sqrt(norm);
    }

    /** Compute dot product of 2D vectors */
    static float Dot(std::vector<float> v1, std::vector<float> v2) {
        return v1[0]*v2[0] + v1[1]*v2[1];
    }

    /** Compute cross product of 2D vectors on z-plane and get z component */
    static float CrossOrientation(std::vector<float> v1, std::vector<float> v2) {
        return v1[0]*v2[1] - v1[1]*v2[0];
    }

    /** Subtract 2D vectors elements (v1-v2) */
    static std::vector<float> Subtract(std::vector<float> v1, std::vector<float> v2) {
        std::vector<float> result = { v1[0]-v2[0],
                                    v1[1]-v2[1] };
        return result;
    }

    /** Add 2D vectors elements (v1+v2) */
    static std::vector<float> Add(std::vector<float> v1, std::vector<float> v2) {
        std::vector<float> result = { v1[0]+v2[0],
                                    v1[1]+v2[1] };
        return result;
    }

    /** Multiply all elements in 2D vector by constant */
    static std::vector<float> Multiply(float c, std::vector<float> v) {
        std::vector<float> result = { c*v[0],
                                    c*v[1] };
        return result;
    }

    /** Compute angle between vectors */
    static float VectorAngle(std::vector<float> v1, std::vector<float> v2) {
        return std::acos(VectorMath::Dot(v1,v2)/VectorMath::Norm(v1)/VectorMath::Norm(v2));
    }

    /** Normalize vector */
    static void Normalize(std::vector<float>& v) {
        float norm_v = VectorMath::Norm(v);
        for (float i = 0; i < v.size(); i++)
            v[i] = v[i]/norm_v;
    }
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
    std::vector<std::vector<float>> CreatePath(float point_separation);

private:
    /** Gets path of points along arc between two vectors */
    std::vector<std::vector<float>> ArcPoints(std::vector<float> v1, std::vector<float> v2, Circle circle, float direction, float point_separation);

    /** Gets path of points along line between two points */
    std::vector<std::vector<float>> LinePoints(std::vector<float> p1, std::vector<float> p2, float point_separation);
    
    PathPose start_;
    PathPose goal_;
    float radius_;
};

} // namespace planning
} // namespace avt_341

#endif
