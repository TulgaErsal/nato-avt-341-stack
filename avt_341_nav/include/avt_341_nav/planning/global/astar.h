#ifndef ASTAR_H
#define ASTAR_H

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "avt_341_nav/core/compute_time_recorder.hpp"
#include "avt_341_nav/core/math_dto.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace avt_341_nav {
namespace planning {

/// Compute time section ids recorded by the planners. PlanPath is recorded by the node that owns the
/// planner; the rest are recorded here, so the difference between the two is the unmeasured
/// remainder of a plan.
namespace planner_sections {
inline const std::string PLAN_PATH = "plan_path";
inline const std::string GRID_INGEST = PLAN_PATH + "/grid_ingest";
inline const std::string DILATION = PLAN_PATH + "/dilation";
inline const std::string EDT = PLAN_PATH + "/edt";
inline const std::string CLEARANCE_SHIFTS = PLAN_PATH + "/clearance_shifts";
inline const std::string SOLVE = PLAN_PATH + "/solve";
}

typedef core::vec2 Point;
typedef core::vec2 Vec2;

class Index {
public:
  int ix;
  int iy;

  bool operator==(const Index& i) {
    return this->ix == i.ix && this->iy == i.iy;
  }
};

/**
 * Astar map class with solve functions and map accessor methods.
 * The map holds obstacle values from 0 to 100. 0=no obstacle, 100=impassable.
 */
class Astar {
public:

  static const int EdgeDistanceCost = 1;

  /// Constructor
  Astar(float w_distance,
        float w_occupancy,
        float w_segmentation,
        bool search_diagonals,
        int los_max_iterations,
        bool los_break_on_first,
        float no_segmentation_data_cost,
        float no_occupancy_data_cost);

  /// Destructor
  virtual ~Astar();

  /// Inherited from base class, return path in world coordinates
//  std::vector<std::vector<float> >* GetCurrentPath() { return &path_world_; }
  std::vector<Point>& GetCurrentPath() { return path_world_; }

  /// Inherited from base class, return the current map
  std::vector<std::vector<float> >* GetCurrentMap() { return &map_; }

  /// Inherited from base class, return goal in world coordinates
  Point GetCurrentGoal() {
    Index goal_index = FoldIndex(goal_);
    Point goal_world = IndexToPoint(goal_index);
    return goal_world;
  }

  /// Inherited from planner base class.
  float GetXMin() { return llx_; }

  /// Inherited from planner base class.
  float GetYMin() { return lly_; }

  /// Inherited from planner base class.
  float GetGridResolution() { return map_res_; }

  /// Inherited from planner base class. Returns the number of horizontal cells.
  int GetGridWidth() { return width_; }

  /// Inherited from planner base class. Returns the number of vertical cells.
  int GetGridHeight() { return height_; }

  /// Get grid value at coordinates. Returns no_segmentation_data_cost_ if the
  /// coordinate falls outside segmentation_grid's published extent, or if the
  /// cell there is published as -1 (unknown).
  int GetGridValue(nav_msgs::msg::OccupancyGrid* segmentation_grid, double x, double y) const;

  /// Get occupancy value at a flattened grid index. Returns no_occupancy_data_cost_
  /// if the cell there is published as -1 (unknown), instead of passing the raw
  /// negative value through as a cost.
  int GetOccupancyValue(nav_msgs::msg::OccupancyGrid* grid, int index) const;

  /// Inherited from planner base class.
  virtual std::vector<Point> PlanPath(nav_msgs::msg::OccupancyGrid* grid,
                              nav_msgs::msg::OccupancyGrid* segmentation_grid,
                              Point goal,
                              Point position);

  /**
   * Allocate memory for the map and initialize
   * \param height Height of the map, in cells
   * \param width Width of the map, in cells
   * \param init_val Initial value for all cells, from 0 to 100
   */
  void AllocateMap(int height, int width, int init_val);

  /**
   * Set the map value of cell Index(i,j) to val height and set weight using weight parameters.
   * \param index index of the cell to set
   * \param val_height Value to set, [0,100]
   * \param val_seg Segmentation value for weights [0,100]
   */
  void SetMapValue(const Index& index, int val_height, int val_seg);

  /**
   * Returns the map value of cell Index(i,j).
   * \param index index of cell to get
   */
  int GetMapValue(const Index& index) { return weights_[FlattenIndex(index)]; }

  /**
   * Sets cell Index(i,j) as the goal point.
   * \param index index of goal cell
   */
  void SetGoal(const Index& index) { goal_ = FlattenIndex(index); }

  /**
   * Sets cell Index(i,j) as the current location of the vehicle
   * \param index index of goal cell
   */
  void SetStart(const Index& index) { start_ = FlattenIndex(index); }

  /**
   * Solve the A* map. Returns true if a path was found.
   */
  virtual bool Solve();


  virtual float* ExtractCosts() {
    return nullptr;
  }

  /**
   * Get A* path solution before smoothing is applied.
   */
  std::vector<Point> GetPathWorldPreSmoothing() {
    std::vector<Point> path_world_pre_smoothing;
    std::transform(path_.begin(),
                   path_.end(),
                   std::back_inserter(path_world_pre_smoothing),
                   [this](const Index& i) { return IndexToPoint(i); });
    return path_world_pre_smoothing;
  }

  /**
   * Get A* path solution after smoothing but before path is interpolated.
   */
  std::vector<Point>* GetPathWorldPreFill() { return &path_world_pre_fill_; }

  /// Return a list of indices specifying the current path.
  std::vector<Index> GetPath() { return path_; }

  /**
   * Set the ENU coordinates of the bottom left corner
   * \param x The East ENU coordinate
   * \param y The North ENU coordinate
   */
  void SetCornerCoords(float x, float y) {
    llx_ = x;
    lly_ = y;
  }

  /**
   * Set the resolution of the map cells in meters
   * \param res The resolution in meters 
   */
  void SetMapRes(float res) {
    map_res_ = res;
  }

  /**
   * Get the resolution of the map cells in meters
   */
  float GetRes() { return map_res_; }

  /**
   * Convert a point in global ENU to point map index.
   * \param point The ENU coordiante
   */
  Index PointToIndex(const Point& point) const {
    int ix = (int) ((point.x - llx_) / map_res_);
    int iy = (int) ((point.y - lly_) / map_res_);
    return {ix, iy};
  }

  /**
   * Convert a point map index to global ENU point
   * \param index The index of the map cell
   */
  Point IndexToPoint(const Index& index) const {
    float px = (index.ix + 0.5f) * map_res_ + llx_;
    float py = (index.iy + 0.5f) * map_res_ + lly_;
    return {px, py};
  }

  /**
 * Determine if a point map index is in the map.
 * Return false in the point is not in the map
 * \param index The index of the map cell
 */
  bool IsInMap(Index index) const {
    bool isin = false;
    if (index.ix >= 0 && index.ix < width_ && index.iy >= 0 && index.iy < height_) {
      isin = true;
    }
    return isin;
  }

  /**
   * Set the factor by which to dilate the map
   * \param dfac The dilation factor
   */
  void SetDilationFactor(int dfac) { dfac_ = dfac; }

  void RequestCancel() { cancel_.store(true, std::memory_order_relaxed); }
  void ClearCancel() { cancel_.store(false, std::memory_order_relaxed); }

  /**
   * Attach a recorder so that the phases of PlanPath are profiled. Optional: planners are also
   * constructed without a ROS node, in which case no compute times are recorded.
   */
  void SetComputeTimeRecorder(const std::shared_ptr<core::ComputeTimeRecorder>& recorder) {
    compute_time_recorder_ = recorder;
  }

protected:
  static constexpr float INF = std::numeric_limits<float>::infinity();

  /**
   * Record a section for the duration of the returned scope, or nothing when no recorder is
   * attached. Hold the result in its own block per phase: ScopedRecording is not move-assignable,
   * so the result cannot be assigned over an existing recording to start the next phase.
   */
  std::optional<core::ScopedRecording> RecordSection(const std::string& section_id) const {
    if (compute_time_recorder_ == nullptr) {
      return std::nullopt;
    }
    return compute_time_recorder_->RecordScope(section_id);
  }

  std::shared_ptr<core::ComputeTimeRecorder> compute_time_recorder_ = nullptr;

  Index FoldIndex(int n) const;

  int FlattenIndex(int i, int j) const { return j * width_ + i; }

  int FlattenIndex(const Index& i) const { return i.iy * width_ + i.ix; }

  /// Heuristic
  float Heuristic(const Index& i0, const Index& i1) const;

  /// Flattened occupancy grid
  std::vector<float> weights_;

  /// unflattened occupancy grid
  std::vector<std::vector<float> > map_;

  ///height of the grid
  int height_;

  ///width of the grid
  int width_;

  ///flattened index of the goal point
  int goal_;

  ///flattened index of the start point
  int start_;

  /// map dilation factor
  int dfac_;

  /// calculated path
  std::vector<int> paths_;

  //std::vector<MapIndex> path_;
  std::vector<Index> path_;                   // raw path before smoothing by line of sight processing
  std::vector<Point> path_world_pre_fill_;  // path world (smoothed) before filled in
  std::vector<Point> path_world_;           // final output path in world coordinates

  virtual bool ExtractPath();
  void PostSmoothing(const std::vector<Index>& in_path, std::vector<Index>& out_path);
  virtual bool LineOfSight(const Index& i0, const Index& i1);

  std::atomic<bool> cancel_{false};

  float llx_, lly_;
  float map_res_;
  float w_distance_;
  float w_occupancy_;
  float w_segmentation_;
  float no_segmentation_data_cost_;
  float no_occupancy_data_cost_;
  bool search_diagonals_;
  int los_max_iterations_;
  bool los_break_on_first_;
  //bool dubins_smoothing_;
  //float dubins_radius_;

  bool HasUp(int index) const { return index / width_ + 1 < height_; }

  bool HasDown(int index) const { return index / width_ > 0; }

  bool HasLeft(int index) const { return index % width_ > 0; }

  bool HasRight(int index) const { return index % width_ + 1 < width_; }

  int Up(int index) const { return index + width_; }

  int Down(int index) const { return index - width_; }

  static int Left(int index) { return index - 1; }

  static int Right(int index) { return index + 1; }

  int UpLeft(int index) const { return index + width_ - 1; }

  int UpRight(int index) const { return index + width_ + 1; }

  int DownLeft(int index) const { return index - width_ - 1; }

  int DownRight(int index) const { return index - width_ + 1; }
};

} // namespace planning
} // namespace avt_341_nav

#endif
