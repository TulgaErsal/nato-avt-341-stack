/**
 * Path smoothing given a minimum radius of curvature using Dubins paths.
 * Used for filtering sharp turns from global plans created by the A* planner.
 * 
 * Evan Vandermate - evanderm@mtu.edu
*/

// project includes
#include "avt_341/planning/global/dubins_smoothing.h"

namespace avt_341 {
namespace planning{

/** Smooth the input path */
void DubinsSmoothing::SmoothPath(float min_radius, float point_separation) {
    if (path_original.empty()) {
        IsSmoothed = false;
        return;
    }

    // Initialize with first point
    int i = 0;
    while (i < path_original.size()) {
        // Check for points contained within tangent circles
        std::vector<Circle> point_circles = PoseAt(path_original,i).CalculateTangentCircles(min_radius);
        std::vector<int> i0 = PointsAheadInCircle(path_original, i, point_circles[0]);
        std::vector<int> i1 = PointsAheadInCircle(path_original, i, point_circles[1]);

        if (i == 0) {
            path_smoothed.push_back(path_original[0]);
            // Check for max radius turn at start of path
            if (!i0.empty()) {
                i = std::min((int)i0.back(), (int)path_original.size()-1);
                path_smoothed.push_back(path_original[i]);
            }
            else if (!i1.empty()) {
                i = std::min((int)i1.back(), (int)path_original.size()-1);
                path_smoothed.push_back(path_original[i]);
            }
            else
                i++;
        }
        else {
            // Determine Dubins start and end poses
            bool intersection_found = false;
            int i_start, i_goal;
            PathPose start, goal;
            if (!i0.empty()) {
                i_start = std::max(i-1, 0);
                i_goal = std::min((int)i0.back()+1, (int)path_original.size()-1);
                start = PoseAt(path_original, i_start);
                goal = PoseAt(path_original, i_goal);
                i = i_goal;
                intersection_found = true;
            }
            else if (!i1.empty()) {
                i_start = std::max(i-1, 0);
                i_goal = std::min((int)i1.back()+1, (int)path_original.size()-1);
                start = PoseAt(path_original, i_start);
                goal = PoseAt(path_original, i_goal);
                i = i_goal;
                intersection_found = true;
            }

            // Generate dubins path for smoothing intersection
            if (intersection_found) {
                DubinsPath dubins(start, goal, min_radius);
                std::vector<std::vector<float>> path_dubins = dubins.CreatePath(point_separation);
                path_smoothed.insert(path_smoothed.end(), path_dubins.begin(), path_dubins.end());
            }
            else {
                path_smoothed.push_back(path_original[i]);
                i++;
            }
        }
    }
    
    IsSmoothed = true;
}

PathPose DubinsSmoothing::PoseAt(std::vector<std::vector<float>> path, int i) {
    
    // Retrieve point origin
    std::vector<float> origin = { path[i][0], path[i][1] };

    // Calculate average slope at path[i]
    std::vector<float> v1;
    std::vector<float> v2;
    if (i == 0) {
        v1 = VectorMath::Subtract(path[i+1],path[i]);
        VectorMath::Normalize(v1);
        v2 = v1;
    }
    else if (i == path.size()-1) {
        v1 = VectorMath::Subtract(path[i],path[i-1]);
        VectorMath::Normalize(v1);
        v2 = v1;
    }
    else {
        v1 = VectorMath::Subtract(path[i+1],path[i]);
        VectorMath::Normalize(v1);
        v2 = VectorMath::Subtract(path[i],path[i-1]);
        VectorMath::Normalize(v2);
    }
    std::vector<float> direction = { (v1[0]+v2[0])/2.0f, (v1[1]+v2[1])/2.0f };
    VectorMath::Normalize(direction);

    // Create pose
    PathPose pose(origin, direction);

    return pose;
}

/** Finds all points in path ahead that are contained in a circle */
std::vector<int> DubinsSmoothing::PointsAheadInCircle(std::vector<std::vector<float>> path, int i, Circle circle) {
    std::vector<int> indices_inside;
    for (int ii = i; ii < path.size(); ii++) {
        if (VectorMath::PointDistance(circle.center_, path[ii]) < circle.radius_-0.001) {
            indices_inside.push_back(ii);
        }
    }
    return indices_inside;
}

/** Create Dubins path from start to goal */
std::vector<std::vector<float>> DubinsPath::CreatePath(float point_separation) {
    // Parse start and goal poses
    std::vector<float> p0 = start_.origin_;
    std::vector<float> v0 = start_.direction_;
    float theta0 = std::atan2(v0[1], v0[0]);
    std::vector<float> pf = goal_.origin_;
    std::vector<float> vf = goal_.direction_;
    float thetaf = std::atan2(vf[1], vf[0]);

    // Determine best circle paths for each point
    float handedness = VectorMath::CrossOrientation(v0,vf);
    handedness /= std::abs(handedness);
    std::vector<float> c1_center = { p0[0] + radius_*std::cos(theta0+handedness*M_PI_2),
                                     p0[1] + radius_*std::sin(theta0+handedness*M_PI_2) };
    std::vector<float> c2_center = { pf[0] + radius_*std::cos(thetaf+handedness*M_PI_2),
                                     pf[1] + radius_*std::sin(thetaf+handedness*M_PI_2) };
    float c1_radius = radius_;
    float c2_radius = radius_;
    Circle c1(c1_center, c1_radius);
    Circle c2(c2_center, c2_radius);
    
    // Calculate outer tangent path
    std::vector<float> v1 = VectorMath::Subtract(c2.center_,c1.center_);
    std::vector<float> n1a = { -v1[1], v1[0] };
    std::vector<float> n1b = { v1[1], -v1[0] };
    std::vector<float> n1 = n1a;
    if (VectorMath::VectorAngle(v0,n1b) < VectorMath::VectorAngle(v0,n1a))
        n1 = n1b;
    VectorMath::Normalize(n1);
    std::vector<float> p1n = VectorMath::Add(c1.center_,VectorMath::Multiply(radius_,n1));
    std::vector<float> p2n = VectorMath::Add(p1n,v1);
    std::vector<float> n2 = VectorMath::Subtract(p2n,c2.center_);
    VectorMath::Normalize(n2);

    // Calculate circular path direction
    float rot_dir = VectorMath::CrossOrientation(VectorMath::Subtract(p0,c1.center_),v0);

    // Generate path
    std::vector<std::vector<float>> path;
    // Arc section #1
    if (VectorMath::PointDistance(p0,p1n) > point_separation) {
        std::vector<std::vector<float>> arc1 = ArcPoints(VectorMath::Subtract(p0,c1.center_),VectorMath::Subtract(p1n,c1.center_),c1,rot_dir,point_separation);
        if (arc1.size() > 1)
            path.insert(path.end(), arc1.begin()+1, arc1.end()-1);
    }
    // Straight section
    std::vector<std::vector<float>> straight = LinePoints(p1n,p2n,point_separation);
    if (straight.size() > 1)
        path.insert(path.end(), straight.begin()+1, straight.end()-1);
    // Arc section #1
    if (VectorMath::PointDistance(pf,p2n) > point_separation) {
        std::vector<std::vector<float>> arc2 = ArcPoints(VectorMath::Subtract(p2n,c2.center_),VectorMath::Subtract(pf,c2.center_),c2,rot_dir,point_separation);
        if (arc2.size() > 1)
            path.insert(path.end(), arc2.begin()+1, arc2.end()-1);
    }

    if (c1.ArcLength(VectorMath::Subtract(p0,c1.center_),VectorMath::Subtract(p1n,c1.center_),rot_dir) > M_PI*radius_
        || c2.ArcLength(VectorMath::Subtract(p2n,c2.center_),VectorMath::Subtract(pf,c2.center_),rot_dir) > M_PI*radius_) {
        std::cout << "\n\tp0: [ " << p0[0] << ", " << p0[1] << " ]"
                  << "\n\tv0: [ " << v0[0] << ", " << v0[1] << " ]"
                  << "\n\tpf: [ " << pf[0] << ", " << pf[1] << " ]"
                  << "\n\tvf: [ " << vf[0] << ", " << vf[1] << " ]" << std::endl;
    }

    return path;
}

/** Gets path of points along arc between two vectors */
std::vector<std::vector<float>> DubinsPath::ArcPoints(std::vector<float> v1, std::vector<float> v2, Circle circle, float direction, float point_separation) {
    std::vector<std::vector<float>> points;
    
    // Normalize vectors
    std::vector<float> v1_norm = v1;
    std::vector<float> v2_norm = v2;
    VectorMath::Normalize(v1);
    VectorMath::Normalize(v2);

    // Calculate vector angles
    float theta1 = std::atan2(v1_norm[1], v1_norm[0]);
    float theta2 = std::atan2(v2_norm[1], v2_norm[0]);

    // Calculate theta increment from linear point distance
    float dtheta = point_separation/circle.radius_;

    // Calculate theta range
    if (direction > 0 && theta1 > theta2)
        theta1 -= 2.0*M_PI;
    else if (direction < 0 && theta1 < theta2)
        theta1 += 2.0*M_PI;

    // Generate arc points
    if (direction > 0) {
        for (float theta = theta1; theta <= theta2; theta += dtheta) {
            std::vector<float> point = { circle.radius_ * std::cos(theta) + circle.center_[0],
                                         circle.radius_ * std::sin(theta) + circle.center_[1] };
            points.push_back(point);
        }
    }
    else {
        for (float theta = theta1; theta >= theta2; theta -= dtheta) {
            std::vector<float> point = { circle.radius_ * std::cos(theta) + circle.center_[0],
                                         circle.radius_ * std::sin(theta) + circle.center_[1] };
            points.push_back(point);
        }
    }
    return points;
}

/** Gets path of points along line between two points */
std::vector<std::vector<float>> DubinsPath::LinePoints(std::vector<float> p1, std::vector<float> p2, float point_separation) {
    // Calulate line vector
    std::vector<float> v12 = VectorMath::Subtract(p2,p1);
    VectorMath::Normalize(v12);

    // Generate line points
    std::vector<std::vector<float>> points;
    for (float dist = 0; dist <= VectorMath::PointDistance(p1,p2); dist += point_separation) {
        std::vector<float> point = { p1[0] + v12[0]*dist,
                                     p1[1] + v12[1]*dist };
        points.push_back(point);
    }
    return points;
}

} // namespace planning
} // namespace avt_341