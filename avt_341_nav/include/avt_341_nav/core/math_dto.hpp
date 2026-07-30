/**
* @file      math_dto.hpp
* @brief     Small vector value types and the geometry helpers operating on them.
*/

#ifndef AVT_341_CORE_MATH_DTO_H
#define AVT_341_CORE_MATH_DTO_H

#include <cmath>

namespace avt_341_nav::core
{

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

}

#endif  // AVT_341_CORE_MATH_DTO_H
