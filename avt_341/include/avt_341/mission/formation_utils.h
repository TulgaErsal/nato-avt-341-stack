#ifndef AVT_341_FORMATION_UTILS_H
#define AVT_341_FORMATION_UTILS_H

#include "avt_341/node/ros_types.h"

// convenient shorthands for adapting TW's code
typedef float Matrix3x3[3][3];
typedef float Vec2d[2];
typedef float TQuat[4];

void ConvertQuaternionToRotMat(TQuat q, Matrix3x3 &R);
void NormalizeVec2D(Vec2d &v);
void PoseToForwardRightVectors(const avt_341::msg::Pose & pose, Vec2d &vx, Vec2d &vy);
bool IsClose(const avt_341::msg::Pose & p1, const avt_341::msg::Pose & p2, double threshold);

inline double PosePlanarDistanceSq(const avt_341::msg::Point & p1, const avt_341::msg::Point & p2){
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return dx*dx + dy*dy;
}

inline double PosePlanarDistance(const avt_341::msg::Point & p1, const avt_341::msg::Point & p2) {
  return sqrt(PosePlanarDistanceSq(p1, p2));
}

class FormationSpeedControlType {
public:
  const static std::string SLOW_DOWN_LEADER;
  const static std::string SPEED_UP_FOLLOWER;
  const static std::string SPEED_UP_FOLLOWER_SIMPLE;
  const static std::string NONE;
};

#endif //AVT_341_FORMATION_UTILS_H
