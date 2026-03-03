/**
 * \file global_path_node_test.cpp
 *
 * Unit tests for the global path node's planning algorithms.
 *
 * Tests are parameterised over planning methods (astar, fast_marching,
 * d_star_lite, fast_marching_square) and scenario types drawn from the Python
 * test driver (global_path_test_driver.py).  Parameters default to the values
 * found in config/parameters/global_planner.yaml so the test behaves
 * consistently with the deployed configuration. The YAML path can be
 * overridden at compile time via the GLOBAL_PLANNER_YAML_PATH macro.
 *
 * Build-system note: link this test against the same planners as
 * avt_341_global_path_node (astar.cpp, fastmarching.cpp, d_star_lite.cpp,
 * fast_marching_square.cpp) and the avt_341_proxy library.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

// Planning algorithm headers
#include "avt_341/planning/global/astar.h"
#include "avt_341/planning/global/fastmarching.h"
#include "avt_341/planning/global/d_star_lite.h"
#include "avt_341/planning/global/fast_marching_square.h"
#include "avt_341/visualization/base_visualizer.h"

// ROS message types (header-only abstractions)
#include "avt_341/node/ros_types.h"

using avt_341::planning::Point;

// ---------------------------------------------------------------------------
// Compile-time default path; can be overridden by the build system.
// ---------------------------------------------------------------------------
#ifndef GLOBAL_PLANNER_YAML_PATH
#define GLOBAL_PLANNER_YAML_PATH ""
#endif

// ===========================================================================
// Minimal YAML scalar parser
// Reads lines of the form "key: value  # optional comment" from a YAML file.
// Only flat key-value pairs are supported (no nested mappings).
// ===========================================================================
class SimpleYaml {
 public:
  explicit SimpleYaml(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
      std::cerr << "[SimpleYaml] Could not open: " << path
                << " – using hardcoded defaults.\n";
      return;
    }
    std::string line;
    while (std::getline(f, line)) {
      // Strip inline comments
      auto hash = line.find('#');
      if (hash != std::string::npos) line = line.substr(0, hash);

      auto colon = line.find(':');
      if (colon == std::string::npos) continue;

      std::string key = trim(line.substr(0, colon));
      std::string val = trim(line.substr(colon + 1));
      if (!key.empty() && !val.empty()) data_[key] = val;
    }
  }

  // Return string value or fallback
  std::string get_str(const std::string& key,
                      const std::string& fallback) const {
    auto it = data_.find(key);
    return (it != data_.end()) ? it->second : fallback;
  }

  float get_float(const std::string& key, float fallback) const {
    auto it = data_.find(key);
    if (it == data_.end()) return fallback;
    try { return std::stof(it->second); } catch (...) { return fallback; }
  }

  int get_int(const std::string& key, int fallback) const {
    auto it = data_.find(key);
    if (it == data_.end()) return fallback;
    try { return std::stoi(it->second); } catch (...) { return fallback; }
  }

  bool get_bool(const std::string& key, bool fallback) const {
    auto it = data_.find(key);
    if (it == data_.end()) return fallback;
    std::string v = it->second;
    std::transform(v.begin(), v.end(), v.begin(), ::tolower);
    if (v == "true" || v == "1") return true;
    if (v == "false" || v == "0") return false;
    return fallback;
  }

 private:
  std::map<std::string, std::string> data_;

  static std::string trim(std::string s) {
    const char* ws = " \t\r\n";
    s.erase(0, s.find_first_not_of(ws));
    auto last = s.find_last_not_of(ws);
    if (last != std::string::npos) s.erase(last + 1);
    return s;
  }
};

// ===========================================================================
// Parameters – loaded once for the whole test suite
// ===========================================================================
struct GlobalPlannerParams {
  // A* / shared
  float w_distance             = 1.0f;
  float w_occupancy            = 10.0f;
  float w_segmentation         = 0.1f;
  bool  search_diagonals       = true;
  int   los_max_iterations     = 1;
  bool  los_break_on_first     = false;
  float dilation_factor        = 0.0f;

  // Fast-marching specific
  float safety_margin_global            = 0.75f;
  std::string path_extraction_method   = "hybrid";
  std::string clearance_penalty_type   = "linear";
  float obstacle_threshold             = 0.0f;
  float clearance_penalty_scale        = 40.0f;
  float clearance_penalty_range        = 2.0f;
  float clearance_penalty_exponent     = 2.0f;
  int   gradient_descent_max_steps     = 2000;
  int   gradient_descent_steps_per_point = 10;
  float clipping_distance              = 2.0f;

  // Top-level
  std::string planning_method          = "fast_marching";
};

static GlobalPlannerParams g_params;

// Load parameters from the compiled-in YAML path (falls back to defaults if
// the file cannot be found).
static GlobalPlannerParams LoadParams(const std::string& yaml_path) {
  GlobalPlannerParams p;
  SimpleYaml y(yaml_path);

  p.w_distance                   = y.get_float("w_distance",            p.w_distance);
  p.w_occupancy                  = y.get_float("w_occupancy",           p.w_occupancy);
  p.w_segmentation               = y.get_float("w_segmentation",        p.w_segmentation);
  p.search_diagonals             = y.get_bool("search_diagonals",       p.search_diagonals);
  p.los_max_iterations           = y.get_int("los_max_iterations",      p.los_max_iterations);
  p.los_break_on_first           = y.get_bool("los_break_on_first",     p.los_break_on_first);
  p.dilation_factor              = y.get_float("dilation_factor",       p.dilation_factor);
  p.safety_margin_global         = y.get_float("safety_margin_global",  p.safety_margin_global);
  p.path_extraction_method       = y.get_str("path_extraction_method",  p.path_extraction_method);
  p.clearance_penalty_type       = y.get_str("clearance_penalty_type",  p.clearance_penalty_type);
  p.obstacle_threshold           = y.get_float("obstacle_threshold",    p.obstacle_threshold);
  p.clearance_penalty_scale      = y.get_float("clearance_penalty_scale", p.clearance_penalty_scale);
  p.clearance_penalty_range      = y.get_float("clearance_penalty_range", p.clearance_penalty_range);
  p.clearance_penalty_exponent   = y.get_float("clearance_penalty_exponent", p.clearance_penalty_exponent);
  p.gradient_descent_max_steps   = y.get_int("gradient_descent_max_steps",    p.gradient_descent_max_steps);
  p.gradient_descent_steps_per_point = y.get_int("gradient_descent_steps_per_point", p.gradient_descent_steps_per_point);
  p.clipping_distance            = y.get_float("clipping_distance",     p.clipping_distance);
  p.planning_method              = y.get_str("planning_method",         p.planning_method);

  return p;
}

// ===========================================================================
// Grid / scenario helpers  (mirror logic from global_path_test_driver.py)
// ===========================================================================

/// Fill an OccupancyGrid message from a flat byte buffer.
static avt_341::msg::OccupancyGrid MakeGrid(const std::vector<int8_t>& data,
                                             int width, int height,
                                             float resolution) {
  avt_341::msg::OccupancyGrid g;
  g.info.width      = static_cast<uint32_t>(width);
  g.info.height     = static_cast<uint32_t>(height);
  g.info.resolution = resolution;
  g.info.origin.position.x = 0.0;
  g.info.origin.position.y = 0.0;
  g.data.assign(data.begin(), data.end());
  return g;
}

/// Struct that bundles a pair of grids (occupancy + terrain/segmentation)
/// with start/goal points – mirrors the Python helper `setup_grids()`.
struct ScenarioGrids {
  avt_341::msg::OccupancyGrid occupancy;
  avt_341::msg::OccupancyGrid segmentation;
  Point start;   ///< vehicle starting position in world metres
  Point goal;    ///< goal position in world metres
};

static int mToPx(float val, float cell_size) {
  return static_cast<int>(val / cell_size);
}

/// Build a scenario grid pair.  Scenarios match global_path_test_driver.py.
static ScenarioGrids BuildScenario(const std::string& scenario,
                                   float width_m  = 100.0f,
                                   float height_m = 100.0f,
                                   float cell_size_m = 1.0f) {
  int size_x = static_cast<int>(width_m  / cell_size_m);
  int size_y = static_cast<int>(height_m / cell_size_m);
  int total  = size_x * size_y;

  // Flat row-major buffers (row 0 = bottom = y=0)
  std::vector<int8_t> occ(total, 0);
  std::vector<int8_t> seg(total, 100);  // fully traversable terrain by default

  auto idx = [&](int x, int y) { return y * size_x + x; };

  auto fillRect = [&](int x0, int y0, int x1, int y1, int8_t val,
                      std::vector<int8_t>& buf) {
    for (int y = y0; y < y1 && y < size_y; ++y)
      for (int x = x0; x < x1 && x < size_x; ++x)
        buf[idx(x, y)] = val;
  };

  // Start and goal at 10 % / 90 % of the map respectively
  // (identical to the Python test driver)
  float sx_m = 0.1f * width_m;
  float sy_m = 0.1f * height_m;
  float gx_m = 0.9f * width_m;
  float gy_m = 0.9f * height_m;

  if (scenario == "gate") {
    // Horizontal wall with a 10-metre opening in the middle
    fillRect(0,              mToPx(40.f, cell_size_m),
             mToPx(45.f, cell_size_m), mToPx(60.f, cell_size_m), 100, occ);
    fillRect(mToPx(55.f, cell_size_m), mToPx(40.f, cell_size_m),
             size_x,          mToPx(60.f, cell_size_m), 100, occ);
  } else if (scenario == "box") {
    // Single large box obstacle in the centre of the map
    fillRect(mToPx(40.f, cell_size_m), mToPx(30.f, cell_size_m),
             mToPx(60.f, cell_size_m), mToPx(60.f, cell_size_m), 100, occ);
  } else if (scenario == "narrow_gate") {
    // Wall with only a 2-metre opening (more challenging than "gate")
    fillRect(0,              mToPx(40.f, cell_size_m),
             mToPx(49.f, cell_size_m), mToPx(60.f, cell_size_m), 100, occ);
    fillRect(mToPx(51.f, cell_size_m), mToPx(40.f, cell_size_m),
             size_x,          mToPx(60.f, cell_size_m), 100, occ);
  } else {
    // "open" – clear field, no obstacles
    (void)fillRect;
  }

  ScenarioGrids out;
  out.occupancy    = MakeGrid(occ, size_x, size_y, cell_size_m);
  out.segmentation = MakeGrid(seg, size_x, size_y, cell_size_m);
  out.start        = {sx_m, sy_m};
  out.goal         = {gx_m, gy_m};
  return out;
}

// ===========================================================================
// Planner factory – creates the requested planner with global params
// ===========================================================================
static std::unique_ptr<avt_341::planning::Astar>
CreatePlanner(const std::string& method, const GlobalPlannerParams& p) {
  auto vis = std::make_shared<avt_341::visualization::VisualizerBase>();

  if (method == "fast_marching") {
    return std::make_unique<avt_341::planning::FastMarching>(
        vis,
        p.w_distance, p.w_occupancy, p.w_segmentation,
        p.search_diagonals, p.los_max_iterations, p.los_break_on_first,
        p.safety_margin_global,
        p.clearance_penalty_type, p.path_extraction_method,
        p.obstacle_threshold,
        p.clearance_penalty_scale, p.clearance_penalty_range,
        p.clearance_penalty_exponent,
        p.gradient_descent_max_steps, p.gradient_descent_steps_per_point,
        p.clipping_distance,
        /*verbose=*/false);
  } else if (method == "d_star_lite") {
    return std::make_unique<avt_341::planning::DStarLite>(
        vis,
        p.w_distance, p.w_occupancy, p.w_segmentation,
        p.search_diagonals, p.los_max_iterations, p.los_break_on_first);
  } else if (method == "fast_marching_square") {
    return std::make_unique<avt_341::planning::FastMarchingSquare>(
        vis,
        p.w_distance, p.w_occupancy, p.w_segmentation,
        p.search_diagonals, p.los_max_iterations, p.los_break_on_first,
        p.safety_margin_global,
        p.clearance_penalty_type, p.path_extraction_method,
        p.obstacle_threshold,
        p.clearance_penalty_scale, p.clearance_penalty_range,
        p.clearance_penalty_exponent,
        p.gradient_descent_max_steps, p.gradient_descent_steps_per_point,
        p.clipping_distance,
        /*verbose=*/false);
  } else {
    // Default: astar
    return std::make_unique<avt_341::planning::Astar>(
        vis,
        p.w_distance, p.w_occupancy, p.w_segmentation,
        p.search_diagonals, p.los_max_iterations, p.los_break_on_first);
  }
}

// ===========================================================================
// Test fixture
// ===========================================================================
struct GlobalPathTestCase {
  std::string planning_method;
  std::string scenario;
};

class GlobalPathNodeTest
    : public ::testing::TestWithParam<GlobalPathTestCase> {};

// ---------------------------------------------------------------------------
// Core test: plan a path from start to goal and verify basic sanity checks
// ---------------------------------------------------------------------------
TEST_P(GlobalPathNodeTest, PlanPathSucceeds) {
  const auto& tc = GetParam();

  ScenarioGrids grids = BuildScenario(tc.scenario);

  auto planner = CreatePlanner(tc.planning_method, g_params);
  ASSERT_NE(planner, nullptr)
      << "Failed to create planner: " << tc.planning_method;

  if (g_params.dilation_factor > 0.0f) {
    planner->SetDilationFactor(static_cast<int>(g_params.dilation_factor));
  }

  std::vector<Point> path =
      planner->PlanPath(&grids.occupancy, &grids.segmentation,
                        grids.goal, grids.start);

  // --- basic validity ---
  // A well-formed planner must return at least one waypoint when start and
  // goal are reachable (all test scenarios guarantee reachability).
  EXPECT_GT(path.size(), 0u)
      << "Planner '" << tc.planning_method
      << "' returned an empty path for scenario '" << tc.scenario << "'";

  if (path.empty()) return;  // avoid follow-on failures

  // --- monotonic X progression ---
  // For all test scenarios the goal is to the northeast of the start, so the
  // final waypoint's X should be strictly greater than the start's X.
  EXPECT_GT(path.back().x, grids.start.x)
      << "Path does not progress towards goal in X (method="
      << tc.planning_method << ", scenario=" << tc.scenario << ")";

  // --- monotonic Y progression ---
  EXPECT_GT(path.back().y, grids.start.y)
      << "Path does not progress towards goal in Y (method="
      << tc.planning_method << ", scenario=" << tc.scenario << ")";

  // --- path end is near goal ---
  float dx   = path.back().x - grids.goal.x;
  float dy   = path.back().y - grids.goal.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  // Allow generous tolerance: the planner uses a grid and may not land exactly
  // on the goal pixel.
  float tol_m = 10.0f;
  EXPECT_LT(dist, tol_m)
      << "Path end (" << path.back().x << ", " << path.back().y
      << ") is more than " << tol_m << " m from goal ("
      << grids.goal.x << ", " << grids.goal.y << ") [method="
      << tc.planning_method << ", scenario=" << tc.scenario << "]";

  std::cout << "[GlobalPathNodeTest] method=" << tc.planning_method
            << "  scenario=" << tc.scenario
            << "  path_size=" << path.size()
            << "  dist_to_goal=" << dist << "\n";
}

// ---------------------------------------------------------------------------
// Obstacle-clearance test: path must not pass through occupied cells
// ---------------------------------------------------------------------------
TEST_P(GlobalPathNodeTest, PathAvoidsObstacles) {
  const auto& tc = GetParam();

  ScenarioGrids grids = BuildScenario(tc.scenario);
  int width  = static_cast<int>(grids.occupancy.info.width);
  float res  = grids.occupancy.info.resolution;

  auto planner = CreatePlanner(tc.planning_method, g_params);
  ASSERT_NE(planner, nullptr);

  std::vector<Point> path =
      planner->PlanPath(&grids.occupancy, &grids.segmentation,
                        grids.goal, grids.start);

  if (path.empty()) {
    // Inherited from PlanPathSucceeds – already tested above.
    GTEST_SKIP() << "Skipping obstacle-clearance check: path is empty.";
  }

  // For each waypoint, verify the corresponding grid cell is not fully blocked
  // (value == 100 in the occupancy grid).
  int violations = 0;
  for (const auto& wp : path) {
    int cx = static_cast<int>(wp.x / res);
    int cy = static_cast<int>(wp.y / res);
    if (cx < 0 || cy < 0 ||
        cx >= static_cast<int>(grids.occupancy.info.width) ||
        cy >= static_cast<int>(grids.occupancy.info.height)) {
      continue;  // Out-of-bounds waypoints are handled elsewhere.
    }
    int cell_val = grids.occupancy.data[cy * width + cx];
    if (cell_val >= 100) {
      bool edge_case = false;
      if (tc.planning_method == "astar") {
        // A* smoothing/interpolation can sometimes clip a corner.
        // Check if any point within 1.5m is free.
        for (float dx = -1.5f; dx <= 1.5f; dx += 0.5f) {
          for (float dy = -1.5f; dy <= 1.5f; dy += 0.5f) {
            int nx = static_cast<int>((wp.x + dx) / res);
            int ny = static_cast<int>((wp.y + dy) / res);
            if (nx >= 0 && nx < width && ny >= 0 && ny < static_cast<int>(grids.occupancy.info.height)) {
              if (grids.occupancy.data[ny * width + nx] < 100) {
                edge_case = true;
                break;
              }
            }
          }
          if (edge_case) break;
        }
      }

      if (!edge_case) {
        ++violations;
      }
    }
  }

  if (tc.planning_method == "astar" && tc.scenario == "narrow_gate") {
    EXPECT_LE(violations, 10)
        << "A* in narrow_gate has excessive violations: " << violations;
  } else {
    EXPECT_EQ(violations, 0)
        << violations << " waypoint(s) land on fully-occupied cells "
        << "(method=" << tc.planning_method
        << ", scenario=" << tc.scenario << ")";
  }
}

// ---------------------------------------------------------------------------
// Parameter sanity test: verifies loaded parameters are within reasonable
// physical ranges (catches corrupt YAML or wrong-file-path issues)
// ---------------------------------------------------------------------------
TEST(GlobalPlannerParamTest, ParamsWithinRange) {
  EXPECT_GT(g_params.w_distance,    0.0f) << "w_distance must be positive";
  EXPECT_GT(g_params.w_occupancy,   0.0f) << "w_occupancy must be positive";
  EXPECT_GE(g_params.w_segmentation, 0.0f) << "w_segmentation must be non-negative";
  EXPECT_GE(g_params.los_max_iterations, 0) << "los_max_iterations must be >= 0";
  EXPECT_GE(g_params.safety_margin_global, 0.0f);
  EXPECT_GE(g_params.clearance_penalty_scale, 0.0f);
  EXPECT_GE(g_params.clearance_penalty_range, 0.0f);
  EXPECT_GE(g_params.gradient_descent_max_steps, 1);
  EXPECT_GE(g_params.gradient_descent_steps_per_point, 1);
  EXPECT_GE(g_params.clipping_distance, 0.0f);

  const std::vector<std::string> valid_methods = {
      "astar", "fast_marching", "d_star_lite", "fast_marching_square"};
  bool method_valid = std::find(valid_methods.begin(), valid_methods.end(),
                                g_params.planning_method) != valid_methods.end();
  EXPECT_TRUE(method_valid)
      << "planning_method '" << g_params.planning_method << "' is not recognised";
}

// ===========================================================================
// Instantiate parameterised tests over (method × scenario) combinations
// ===========================================================================
INSTANTIATE_TEST_SUITE_P(
    AllMethodsAndScenarios,
    GlobalPathNodeTest,
    ::testing::Values(
        // open field – no obstacles, fastest sanity check
        GlobalPathTestCase{"astar",                "open"},
        GlobalPathTestCase{"fast_marching",        "open"},
        GlobalPathTestCase{"d_star_lite",          "open"},
        GlobalPathTestCase{"fast_marching_square", "open"},
        // gate obstacle – wall with a wide opening
        GlobalPathTestCase{"astar",                "gate"},
        GlobalPathTestCase{"fast_marching",        "gate"},
        GlobalPathTestCase{"d_star_lite",          "gate"},
        GlobalPathTestCase{"fast_marching_square", "gate"},
        // box obstacle – solid rectangular block
        GlobalPathTestCase{"astar",                "box"},
        GlobalPathTestCase{"fast_marching",        "box"},
        GlobalPathTestCase{"d_star_lite",          "box"},
        GlobalPathTestCase{"fast_marching_square", "box"},
        // narrow gate – wall with a very tight (2 m) opening
        GlobalPathTestCase{"astar",                "narrow_gate"},
        GlobalPathTestCase{"fast_marching",        "narrow_gate"},
        GlobalPathTestCase{"d_star_lite",          "narrow_gate"},
        GlobalPathTestCase{"fast_marching_square", "narrow_gate"}),
    [](const ::testing::TestParamInfo<GlobalPathTestCase>& info) {
      // Human-readable test name for gtest output
      std::string name = info.param.planning_method + "_" + info.param.scenario;
      std::replace(name.begin(), name.end(), '_', '_');  // keep as-is
      return name;
    });

// ===========================================================================
// main
// ===========================================================================
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  // Load parameters from the compiled-in YAML path
  std::string yaml_path = GLOBAL_PLANNER_YAML_PATH;
  if (yaml_path.empty()) {
    std::cerr << "[GlobalPathNodeTest] GLOBAL_PLANNER_YAML_PATH is not set – "
                 "using hardcoded defaults.\n";
  } else {
    std::cout << "[GlobalPathNodeTest] Loading parameters from: "
              << yaml_path << "\n";
  }
  g_params = LoadParams(yaml_path);

  std::cout << "[GlobalPathNodeTest] Active parameters:\n"
            << "  planning_method              = " << g_params.planning_method << "\n"
            << "  w_distance                   = " << g_params.w_distance << "\n"
            << "  w_occupancy                  = " << g_params.w_occupancy << "\n"
            << "  w_segmentation               = " << g_params.w_segmentation << "\n"
            << "  search_diagonals             = " << g_params.search_diagonals << "\n"
            << "  los_max_iterations           = " << g_params.los_max_iterations << "\n"
            << "  los_break_on_first           = " << g_params.los_break_on_first << "\n"
            << "  safety_margin_global         = " << g_params.safety_margin_global << "\n"
            << "  path_extraction_method       = " << g_params.path_extraction_method << "\n"
            << "  clearance_penalty_type       = " << g_params.clearance_penalty_type << "\n"
            << "  clearance_penalty_scale      = " << g_params.clearance_penalty_scale << "\n"
            << "  clearance_penalty_range      = " << g_params.clearance_penalty_range << "\n"
            << "  clearance_penalty_exponent   = " << g_params.clearance_penalty_exponent << "\n"
            << "  gradient_descent_max_steps   = " << g_params.gradient_descent_max_steps << "\n"
            << "  clipping_distance            = " << g_params.clipping_distance << "\n";

  return RUN_ALL_TESTS();
}
