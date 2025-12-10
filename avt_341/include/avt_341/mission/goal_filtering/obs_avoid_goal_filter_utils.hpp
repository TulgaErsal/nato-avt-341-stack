#ifndef OBS_AVOIDANCE_GOAL_FILTER_UTILS_HPP
#define OBS_AVOIDANCE_GOAL_FILTER_UTILS_HPP

#include <Eigen/Dense>
#include <tuple>
#include <string>

namespace avt_341::mission {

// —————————————————————————————————————————————————————————————————————
// Occupancy‐grid cell values
// —————————————————————————————————————————————————————————————————————
#define OCC_FREE     0   // free space
#define OCC_PADDING  1   // safety padding zone
#define OCC_OBSTACLE 2   // obstacle cell

// Collision & intersection
bool isInCollision(const Eigen::MatrixXi& grid, const Eigen::Vector2d& pt);
bool pointsIntersect(const Eigen::MatrixXi& grid,
                     const Eigen::Vector2d& p1,
                     const Eigen::Vector2d& p2);

// Patch extraction
std::tuple<Eigen::MatrixXi, Eigen::Vector2i, Eigen::Vector2i>
extractPatch(const Eigen::MatrixXi& grid,
             const Eigen::Vector2d& contact,
             int patch_pad_width);

// Row‐based avoidance
std::tuple<Eigen::VectorXi, Eigen::MatrixXi, Eigen::Vector2d>
getRowFromPatch(const Eigen::MatrixXi& patch,
                Eigen::Vector2d& patch_point,
                const Eigen::Vector2d& patch_center,
                double angle);

std::tuple<std::string, int, bool>
getDistance(const Eigen::VectorXi& row, int idx, const std::string& dir);

int getFirstFeasibleIndex(const Eigen::VectorXi& row,
                          int prev_idx,
                          int cur_idx,
                          const std::string& dir);

bool isInDeadlock(const Eigen::VectorXi& row,
                  int prev_idx,
                  int cur_idx,
                  int min_obstacle_width);

// Recovery
std::tuple<Eigen::Vector2d, std::string, int, bool>
avoidCollision(const std::tuple<Eigen::MatrixXi, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2i, Eigen::Vector2i>& patch_data,
               double angle,
               const Eigen::VectorXi& row,
               const std::string& dir);

std::tuple<Eigen::Vector2d, std::string, int, bool>
avoidIntersection(const std::tuple<Eigen::MatrixXi, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2i, Eigen::Vector2i>& patch_data,
                  double angle,
                  const Eigen::VectorXi& row,
                  int prev_idx,
                  const std::string& dir,
                  int min_obstacle_width);

}

#endif  // OBS_AVOIDANCE_GOAL_FILTER_UTILS_HPP
