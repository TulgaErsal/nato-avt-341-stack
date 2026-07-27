#ifndef D_STAR_LITE_H
#define D_STAR_LITE_H

#include "avt_341/planning/global/astar.h"
#include <set>
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace avt_341 {
namespace planning {

struct DStarLiteKey {
  float k1;
  float k2;

  bool operator<(const DStarLiteKey& other) const {
    if (k1 < other.k1) return true;
    if (k1 > other.k1) return false;
    return k2 < other.k2;
  }
  bool operator!=(const DStarLiteKey& other) const {
    return k1 != other.k1 || k2 != other.k2;
  }
};

struct DStarNode {
  int index;
  DStarLiteKey key;
  bool operator<(const DStarNode& other) const {
    if (key < other.key) return true;
    if (other.key < key) return false;
    return index < other.index;
  }
};

class DStarLite : public Astar {
public:
  DStarLite(float w_distance,
            float w_occupancy,
            float w_segmentation,
            bool search_diagonals,
            int los_max_iterations,
            bool los_break_on_first);

  virtual ~DStarLite();

  std::vector<Point> PlanPath(nav_msgs::msg::OccupancyGrid* grid,
                              nav_msgs::msg::OccupancyGrid* segmentation_grid,
                              Point goal,
                              Point position) override;

  bool Solve() override;

protected:
  void Initialize();
  DStarLiteKey CalculateKey(int s);
  void UpdateVertex(int u);
  void ComputeShortestPath();
  float Cost(int u, int v);
  float HeuristicDStar(int u, int v);
  std::vector<int> GetNeighbors(int u);

  std::vector<float> g_;
  std::vector<float> rhs_;
  std::vector<DStarLiteKey> open_keys_;
  std::vector<bool> in_open_;
  float km_;
  int s_start_;
  int s_goal_;
  int s_last_;

  std::set<DStarNode> open_list_;

  bool ExtractPath() override;
};

} // namespace planning
} // namespace avt_341

#endif // D_STAR_LITE_H
