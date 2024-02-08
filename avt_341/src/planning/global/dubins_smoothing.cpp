/**
 * Path smoothing given a minimum radius of curvature using Dubins paths.
 * Used for filtering sharp turns from global plans created by the A* planner.
 * 
 * Evan Vandermate - evanderm@mtu.edu
*/

// project includes
#include "avt_341/planning/global/dubins_smoothing.h"


using namespace avt_341::utils;

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
    vec2 p1(path[i][0], path[i][1]);
    vec2 p0 = p1;
    vec2 p2 = p1;
    if (i != 0)
        p0 = vec2(path[i-1][0], path[i-1][1]);
    if (i < path.size()-1)
        p2 = vec2(path[i+1][0], path[i+1][1]);

    // Calculate average slope at path[i]
    vec2 v1;
    vec2 v2;
    if (i == 0) {
        v1 = normalize(p2-p1);
        v2 = v1;
    }
    else if (i == path.size()-1) {
        v1 = normalize(p1-p0);
        v2 = v1;
    }
    else {
        v1 = normalize(p2-p1);
        v2 = normalize(p1-p0);
    }
    vec2 direction = normalize(vec2((v1.x+v2.x)/2.0f, (v1.y+v2.y)/2.0f));

    // Create pose
    PathPose pose(p1, direction);

    return pose;
}

/** Finds all points in path ahead that are contained in a circle */
std::vector<int> DubinsSmoothing::PointsAheadInCircle(std::vector<std::vector<float>> path, int i, Circle circle) {
    std::vector<int> indices_inside;
    for (int ii = i; ii < path.size(); ii++) {
        vec2 pt(path[ii][0],path[ii][1]);
        float d = length(circle.center_-pt);
        if (d < circle.radius_-0.001) {
            indices_inside.push_back(ii);
        }
    }
    return indices_inside;
}

/** Create Dubins path from start to goal */
std::vector<std::vector<float>> DubinsPath::CreatePath(float pt_sep) {
    std::vector<float> dubins_lengths(4,0.0);
    std::vector<std::vector<float>> path;

    // Calculate all possible dubins paths
    DubinsXSX(path, dubins_lengths[0], start_, goal_, radius_, -1, -1);
    DubinsXSY(path, dubins_lengths[1], start_, goal_, radius_, -1, -1);
    DubinsXSX(path, dubins_lengths[2], start_, goal_, radius_, -1, 1);
    DubinsXSY(path, dubins_lengths[3], start_, goal_, radius_, -1, 1);

    // Generate shortest dubins path
    int i_min = (int)std::distance(std::begin(dubins_lengths), std::min_element(std::begin(dubins_lengths), std::end(dubins_lengths)));
    path.clear();
    if (i_min == 0)
        DubinsXSX(path, dubins_lengths[0], start_, goal_, radius_, pt_sep, -1);
    else if (i_min == 1)
        DubinsXSY(path, dubins_lengths[1], start_, goal_, radius_, pt_sep, -1);
    else if (i_min == 2)
        DubinsXSX(path, dubins_lengths[2], start_, goal_, radius_, pt_sep, 1);
    else
        DubinsXSY(path, dubins_lengths[3], start_, goal_, radius_, pt_sep, 1);

    return path;
}

/** 
 * Create Dubins path given start/goal poses and a radius 
 * direction < 0    -> RSR
 * direction > 0    -> LSL
 * pt_sep <= 0      -> No points generated
*/
void DubinsPath::DubinsXSX(std::vector<std::vector<float>>& path_out, float& len_out, 
                            PathPose start, PathPose goal, float radius, float pt_sep, float direction) {
    // Parse poses
    vec2 p0 = start.origin_;
    vec2 v0 = normalize(start.direction_);
    vec2 pf = goal.origin_;
    vec2 vf = normalize(goal.direction_);
    float dir  = direction / abs(direction);

    // Calculate tangent circles
    Circle c1({ p0.x + radius*cos(atan2(v0.y,v0.x)+dir*M_PI/2.0),
                p0.y + radius*sin(atan2(v0.y,v0.x)+dir*M_PI/2.0) }, radius);
    Circle c2({ pf.x + radius*cos(atan2(vf.y,vf.x)+dir*M_PI/2.0),
                pf.y + radius*sin(atan2(vf.y,vf.x)+dir*M_PI/2.0) }, radius);
    
    // Calculate vectors
    vec2 v1 = c2.center_-c1.center_;
    vec2 n1 = cross(normalize(v1),vec3(0,0,dir)).xy() * radius;  // Rotate v1 90deg CCW
    vec2 p1n = c1.center_ + n1;
    vec2 p2n = p1n + v1;
    vec2 n2 = p2n - c2.center_;

    // Generate path
    if (pt_sep > 0) {
        std::vector<std::vector<float>> arc1 = ArcPoints(p0-c1.center_,n1,c1,dir,pt_sep);   // R/L
        std::vector<std::vector<float>> straight = LinePoints(p1n,p2n,pt_sep);              // S
        std::vector<std::vector<float>> arc2 = ArcPoints(n2,pf-c2.center_,c2,dir,pt_sep);   // R/L
        path_out.insert(path_out.end(), arc1.begin()+1, arc1.end()-1);
        path_out.insert(path_out.end(), straight.begin()+1, straight.end()-1);
        path_out.insert(path_out.end(), arc2.begin()+1, arc2.end()-1);
    }

    // Calculate path segment lengths
    len_out = 0.0;
    len_out += ArcLength(p0-c1.center_,n1,c1.radius_,dir);
    len_out += length(p2n-p1n);
    len_out += ArcLength(n2,pf-c2.center_,c2.radius_,dir);
}

/** 
 * Create Dubins path given start/goal poses and a radius 
 * direction < 0            -> RSL
 * direction > 0            -> LSR
 * pt_sep <= 0              -> No points generated
 * len_out == "float limit" -> No Solution
*/
void DubinsPath::DubinsXSY(std::vector<std::vector<float>>& path_out, float& len_out,
                            PathPose start, PathPose goal, float radius, float pt_sep, float direction) {
    // Parse poses
    vec2 p0 = start.origin_;
    vec2 v0 = normalize(start.direction_);
    vec2 pf = goal.origin_;
    vec2 vf = normalize(goal.direction_);
    float dir  = direction / abs(direction);

    // Calculate tangent circles
    Circle c1({ p0.x + radius*cos(atan2(v0.y,v0.x)+dir*M_PI/2.0),
                p0.y + radius*sin(atan2(v0.y,v0.x)+dir*M_PI/2.0) }, radius);
    Circle c2({ pf.x + radius*cos(atan2(vf.y,vf.x)-dir*M_PI/2.0),
                pf.y + radius*sin(atan2(vf.y,vf.x)-dir*M_PI/2.0) }, radius);
    
    // Calculate intermediate circles
    vec2 v1 = c2.center_ - c1.center_;
    Circle c3(c1.center_+v1/2.0,length(v1)/2.0);
    Circle c4(c1.center_,2.0*c1.radius_);

    // Calculate outer tangents
    std::vector<vec2> p1_ot_all = c3.intersects(c4);
    if (p1_ot_all.empty()) {
        len_out = std::numeric_limits<float>::max();
        return; // No Solution
    }
    vec2 n3a = p1_ot_all[0]-c1.center_;
    vec2 v3a = c2.center_-p1_ot_all[0];
    vec2 n3b = p1_ot_all[1]-c1.center_;
    vec2 v3b = c2.center_-p1_ot_all[1];

    // Find correct outer tangent
    vec2 n3 = n3a;
    vec2 v3 = v3a;
    vec2 p1_ot = p1_ot_all[0];
    if (-dir*cross(n3b,v3b) < 0) {
        n3 = n3b;
        v3 = v3b;
        p1_ot = p1_ot_all[1];
    }

    // Calculate vectors
    vec2 n1 = normalize(n3)*c1.radius_;
    vec2 p1n = c1.center_+n1;
    vec2 p2n = p1n+v3;
    vec2 n2 = p2n-c2.center_;

    // Generate path
    if (pt_sep > 0) {
        std::vector<std::vector<float>> arc1 = ArcPoints(p0-c1.center_,n1,c1,dir,pt_sep);   // R/L
        std::vector<std::vector<float>> straight = LinePoints(p1n,p2n,pt_sep);              // S
        std::vector<std::vector<float>> arc2 = ArcPoints(n2,pf-c2.center_,c2,-dir,pt_sep);  // L/R
        path_out.insert(path_out.end(), arc1.begin()+1, arc1.end()-1);
        path_out.insert(path_out.end(), straight.begin()+1, straight.end()-1);
        path_out.insert(path_out.end(), arc2.begin()+1, arc2.end()-1);
    }

    // Calculate path segment lengths
    len_out = 0.0;
    len_out += ArcLength(p0-c1.center_,n1,c1.radius_,dir);
    len_out += length(p2n-p1n);
    len_out += ArcLength(n2,pf-c2.center_,c2.radius_,-dir);
}

/** Gets path of points along arc between two vectors */
std::vector<std::vector<float>> DubinsPath::ArcPoints(vec2 v1, vec2 v2, Circle circle, float direction, float pt_sep) {
    std::vector<std::vector<float>> points;

    // Calculate vector angles
    float theta1 = atan2(v1.y, v1.x);
    float theta2 = atan2(v2.y, v2.x);

    // Calculate theta range
    if (direction > 0 && theta1 > theta2)
        theta1 -= 2.0*M_PI;
    else if (direction < 0 && theta1 < theta2)
        theta1 += 2.0*M_PI;

    // Generate arc points
    float dtheta = (direction/abs(direction)) * pt_sep * circle.radius_;
    for (float theta = theta1; (direction > 0 && theta <= theta2) || (direction < 0 && theta >= theta2); theta += dtheta) {
        std::vector<float> point = { circle.radius_ * cos(theta) + circle.center_.x,
                                     circle.radius_ * sin(theta) + circle.center_.y };
        points.push_back(point);
    }

    return points;
}

/** Gets path of points along line between two points */
std::vector<std::vector<float>> DubinsPath::LinePoints(vec2 p1, vec2 p2, float pt_sep) {
    // Calulate line vector
    vec2 v12 = normalize(p2-p1);

    // Generate line points
    std::vector<std::vector<float>> points;
    for (float dist = 0; dist <= length(p1-p2); dist += pt_sep) {
        std::vector<float> point = { p1.x + v12.x*dist,
                                     p1.y + v12.y*dist };
        points.push_back(point);
    }
    return points;
}

} // namespace planning
} // namespace avt_341