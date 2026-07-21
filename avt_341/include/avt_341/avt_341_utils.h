/**
 * \file avt_341_utils.h
 *
 * Structs and inline functions used by all the algorithms.
 *
 * \date 9/3/2020
 */
#ifndef AVT_341_UTILS_H
#define AVT_341_UTILS_H

#include "avt_341/node/ros_types.h"
#include <iomanip>
#include <sstream>
#include <array>
#include <iostream>
#include <numeric> // for std::accumulate

// TODO: Refactor to dto.h and utils.h under core

namespace avt_341 {
namespace utils {

enum NavStackState : int {
  NotInit = -1,
  Active = 0,
  InactiveCoast = 1,
  InactiveGradualStop = 2,
  InactiveHardStop = 3
};

enum NavStateCmd : int {
  GoInactive = 0,
  GoActive = 1
};

inline bool IsValidShutdownBehavior(const int shutdown_behavior)
{
	return shutdown_behavior >= static_cast<int>(InactiveCoast) && shutdown_behavior <= static_cast<int>(InactiveHardStop);
}

// TODO: Just use Eigen? If not, rename in future, should also be pascal case.
struct vec2{
	vec2(){
		x = 0.0f;
		y = 0.0f;
	}
	vec2(float x_, float y_){
		x = x_; 
		y = y_;
	}

	vec2 operator+(const vec2& b) const { return vec2(this->x + b.x, this->y + b.y); }
	vec2 operator-(const vec2& b) const { return vec2(this->x - b.x, this->y - b.y); }
	vec2 operator*(const float s) const { return vec2(s*this->x, s*this->y); }
	vec2 operator/(const float s) const { return vec2(this->x/s, this->y/s); }

	void normalize() {
		float mag = sqrt(x*x + y*y);
		x /= mag;
		y /= mag;
	}

	float mag() {
		return sqrt(x*x + y*y);
	}

	float x;
	float y;
};

struct vec3{
	vec3(){
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}
	vec3(float x_, float y_, float z_){
		x = x_; 
		y = y_;
		z = z_;
	}

	vec3 operator+(const vec3& b) const { return vec3(this->x + b.x, this->y + b.y, this->z + b.z); }
	vec3 operator-(const vec3& b) const { return vec3(this->x - b.x, this->y - b.y, this->z - b.z); }
	vec3 operator*(const float s) const { return vec3(s*this->x, s*this->y, s*this->z); }
	vec3 operator/(const float s) const { return vec3(this->x/s, this->y/s, this->z/s); }
	float operator[](const int idx) const {
		return idx == 0 ? x : (idx == 1 ? y : z);
	};

	vec2 xy() {
		return vec2(x,y);
	}

	float x;
	float y;
	float z;
};

struct ivec2{
	ivec2(){
		x = 0;
		y = 0;
	}
	ivec2(int x_, int y_){
		x = x_; 
		y = y_;
	}
	ivec2 operator+(const ivec2& b) const { return ivec2(this->x + b.x, this->y + b.y); }
	ivec2 operator-(const ivec2& b) const { return ivec2(this->x - b.x, this->y - b.y); }
	ivec2 operator*(const int s) const { return ivec2(s*this->x, s*this->y); }
	ivec2 operator/(const int s) const { return ivec2(this->x/s, this->y/s); }
	int x;
	int y;
};

inline float length(vec2 p){
	float r = (float)sqrt(p.x*p.x + p.y*p.y);
	return r;
}

inline float dot(vec2 p, vec2 q){
	return (p.x*q.x+p.y*q.y);
}

inline vec2 normalize(vec2 v) {
	float mag = sqrt(v.x*v.x + v.y*v.y);
	return vec2(v.x/mag, v.y/mag);
}

inline vec3 cross(vec3 v1, vec3 v2) {
	return vec3( v1.y*v2.z - v1.z*v2.y,
				 v1.z*v2.x - v1.x*v2.z,
				 v1.x*v2.y - v1.y*v2.x );
}

inline vec3 cross(vec2 v1, vec3 v2) {
	return vec3( v1.y*v2.z,
				 -1.0*v1.x*v2.z,
				 v1.x*v2.y - v1.y*v2.x );
}

inline float cross(vec2 v1, vec2 v2) {
	return v1.x*v2.y - v1.y*v2.x;
}

inline double GetDistance(msg::Point p1, msg::Point p2)
{
	const double dx = p1.x - p2.x;
	const double dy = p1.y - p2.y;
	return sqrt(dx*dx + dy*dy);
}

inline float PointLineDistance(vec2 x1, vec2 x2, vec2 x0) {
	float sx1 = x0.x - x1.x;
	float sy1 = x0.y - x1.y;
	float sx2 = x0.x - x2.x;
	float sy2 = x0.y - x2.y;
	float z = sx1*sy2 - sx2*sy1;
	vec2 x21 = x2 - x1;
	float  d = z / length(x21);
	return d;
}

/**
 * Return distance from a point to a segment
 * \param ep1 First endpoint of the segment
 * \param ep2 Second endpoint of the segment
 * \param p The test point 
 */
inline float PointToSegmentDistance(vec2 ep1, vec2 ep2, vec2 p) {
	vec2 v21 = ep2 - ep1;
	vec2 pv1 = p - ep1;
	if (dot(v21, pv1) <= 0.0) {
		float d = length(pv1);
		return d;
	}
	vec2 v12 = ep1 - ep2;
	vec2 pv2 = p - ep2;
	if (dot(v12, pv2) <= 0.0) {
		float d = length(pv2);
		return d;
	}
	float d0 = PointLineDistance(ep1, ep2, p);
	return d0;
}

inline float GetHeadingFromOrientation(const avt_341::msg::Quaternion& orientation){
    avt_341::msg_tf::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w);
    const avt_341::msg_tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
    return static_cast<float>(yaw);
}

/// Convert any type to a string with zero padding
inline std::string ToString(int x, int zero_padding){
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(zero_padding) << x;
  std::string str = ss.str();
  return str;
};

/// Ray-casting point-in-polygon test (works for non-convex polygons).
/// https://en.wikipedia.org/wiki/Point_in_polygon
inline bool IsInsidePolygon(const std::vector<vec2>& poly, double px, double py)
{
	bool inside = false;
	const int n = static_cast<int>(poly.size());
	for (int i = 0, j = n - 1; i < n; j = i++) {
		const double xi = poly[i].x, yi = poly[i].y;
		const double xj = poly[j].x, yj = poly[j].y;
		if (((yi > py) != (yj > py)) &&
			(px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
			inside = !inside;
			}
	}
	return inside;
}

/**
 * @brief Trims the input string, removing all leading and trailing characters that match
 * the specified character (default is space).
 *
 * @param str String to be trimmed.
 * @param char_to_remove Character to remove.
 * @return Trimmed string.
 */
inline std::string Trim(const std::string& str, const char char_to_remove = ' ')
{
    const auto start = str.find_first_not_of(char_to_remove);
    if (start == std::string::npos) return "";
    const auto end = str.find_last_not_of(char_to_remove);
    return str.substr(start, end - start + 1);
}

/**
 * @brief Split a string with a specified delimiter.
 *
 * @param str String to be split.
 * @param delimiter Delimiter character used to split the string.
 * @param trim_whitespace If set, trims whitespaces from split substrings.
 * @return std::vector<std::string> Vector of split substrings (excluding the delimiter).
 */
inline std::vector<std::string> SplitByDelimiter(
	const std::string& str,
	const char delimiter = '-',
	const bool trim_whitespace = true
	){
	std::stringstream stream(str);
	std::vector<std::string> tokens;
	std::string token;
	while(std::getline(stream, token, delimiter)) { tokens.push_back(trim_whitespace ? Trim(token) : token); }
	return tokens;
}

inline double DiffAngle(const double a, const double b) {
	constexpr double two_pi = 2.0 * M_PI;
	double diff = a - b;
	diff -= two_pi * floor((diff + M_PI) / two_pi);
	return diff;
}

inline double DiffDeg(const double a, const double b) {
	constexpr double s = M_PI / 180.0;
	return DiffAngle(a * s, b * s) / s;
}

inline bool UseGoalOrientation(const msg::NavGoal& msg)
{
	return msg.yaw_threshold < M_PI;
}

inline void GetGoalError(const msg::Pose& pose, const msg::NavGoal& goal, double& dist_error, double& yaw_error)
{
	if (UseGoalOrientation(goal))
	{
		const double pose_yaw = GetHeadingFromOrientation(pose.orientation);
		const double goal_yaw = GetHeadingFromOrientation(goal.pose.orientation);
		yaw_error = std::abs(DiffAngle(pose_yaw, goal_yaw));
	}
	else
	{
		yaw_error = 0.0;
	}

	dist_error = GetDistance(pose.position, goal.pose.position);
}

inline bool IsGoalReached(const msg::Pose& pose, const msg::NavGoal& goal)
{
	double dist_diff, yaw_diff;
	GetGoalError(pose, goal, dist_diff, yaw_diff);
	return yaw_diff < goal.yaw_threshold && dist_diff < goal.dist_threshold;
}

inline bool IsGoalReached(const msg::NavState& state, const msg::NavGoal& goal)
{
	return state.goal_distance < goal.dist_threshold && state.goal_yaw_difference < goal.yaw_threshold;
}

} //namespace utils
} //namespace avt_341

#endif
