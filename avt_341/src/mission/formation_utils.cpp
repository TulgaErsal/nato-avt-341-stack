#include "avt_341/mission/formation_utils.h"

void ConvertQuaternionToRotMat(TQuat q, Matrix3x3 &R) {
  R[0][0] = 1.f + 2.f * (-q[1] * q[1] - q[2] * q[2]);
  R[0][1] = 2.f * (q[0] * q[1] - q[2] * q[3]);
  R[0][2] = 2.f * (q[0] * q[2] + q[1] * q[3]);
  R[1][0] = 2.f * (q[0] * q[1] + q[2] * q[3]);
  R[1][1] = 1.f + 2.f * (-q[0] * q[0] - q[2] * q[2]);
  R[1][2] = 2.f * (q[1] * q[2] - q[0] * q[3]);
  R[2][0] = 2.f * (q[0] * q[2] - q[1] * q[3]);
  R[2][1] = 2.f * (q[1] * q[2] + q[0] * q[3]);
  R[2][2] = 1.f + 2.f * (-q[0] * q[0] - q[1] * q[1]);
}

void NormalizeVec2D(Vec2d &v) {
  float l = sqrtf(v[0] * v[0] + v[1] * v[1]);
  if (l == 0.0f)l = 0.0f; else l = 1.0f / l;
  v[0] *= l;
  v[1] *= l;
}

void PoseToForwardRightVectors(const avt_341::msg::Pose & pose, Vec2d &vx, Vec2d &vy) {
  Matrix3x3 leaderRotMatrix;
  TQuat q;
  q[0] = pose.orientation.x;
  q[1] = pose.orientation.y;
  q[2] = pose.orientation.z;
  q[3] = pose.orientation.w;
  ConvertQuaternionToRotMat(q, leaderRotMatrix);
  vx[0] = 0.5 * (leaderRotMatrix[0][0] + leaderRotMatrix[1][1]); //average cos
  vx[1] = 0.5 * (-leaderRotMatrix[0][1] + leaderRotMatrix[1][0]); //average sin
  NormalizeVec2D(vx);
  vy[0] = vx[1];
  vy[1] = -vx[0];
}

bool IsClose(const avt_341::msg::Pose & p1, const avt_341::msg::Pose & p2, double threshold){
  return PosePlanarDistanceSq(p1.position, p2.position) < threshold * threshold;
}

const std::string FormationSpeedControlType::SLOW_DOWN_LEADER = "slow_down_leader";
const std::string FormationSpeedControlType::SPEED_UP_FOLLOWER = "speed_up_follower";
const std::string FormationSpeedControlType::SPEED_UP_FOLLOWER_SIMPLE = "speed_up_follower_simple";
const std::string FormationSpeedControlType::NONE = "none";
