# Obstacle Avoidance for Vehicle Formations (ROS 2 C++)

This repository contains a **C++ ROS 2 implementation** of an obstacle-avoidance algorithm for **one follower agent** in a vehicle formation. The node runs **online**: each incoming leader point is offset to a follower target, checked against the occupancy grid, corrected if needed, and immediately published (no path arrays are stored).

The algorithm is part of the **avt_341** package and was developed by **Šimon Lehký** and **doc. Ing. Tomáš Haniš, Ph.D.**, at the Department of Control Engineering, FEE, CTU in Prague.

---

## ✅ TODO

- End-to-end testing with actual map source and topics.

---

## 🚀 Overview

- The node subscribes to a leader position as `geometry_msgs/Point`.
- A **macro-defined offset** is applied to get the follower target.
- Using a **macro-mapped** `nav_msgs/OccupancyGrid`, the node:
  1) dilates obstacles by one cell (safety padding),
  2) detects **collisions** (follower point inside obstacle/padding),
  3) detects **segment intersections** (prev→current line-of-sight vs obstacle),
  4) locally corrects the follower target using a **window-based patch** and a **row scan** with rotation.

Processing is **streaming**: the node only keeps the **last published point** (for intersection checks) and a small cache (patch/direction/row index/deadlock) to maintain continuity—no path arrays or step counters.

---

## 🧠 Algorithm Highlights

- **Occupancy grid values (internal):**
  `OCC_FREE = 0`, `OCC_PADDING = 1`, `OCC_OBSTACLE = 2`.

- **Window-based patch extraction (fast):**
  Around the contact point, a fixed window of radius `PATCH_PAD_WIDTH` is cut (`origin = window start`, `padding_offset = (0,0)`). The patch is **rotated** by the motion angle (leader→follower), and a **row** aligned with that direction is extracted.

- **Row scan & decisions:**
  - Direction choice (left/right) uses the **sum of row values** along each side (matches Python).
  - First feasible index requires a **free slice** between the previous and new column indices (again, matches Python).
  - **Deadlock** is declared if the number of `OCC_OBSTACLE` cells between indices ≥ `MIN_OBSTACLE_WIDTH`.

- **Divergence warning:**
  If the follower deviates from the leader by more than `FOLLOWER_DIVERGENCE_THRESHOLD`, a warning is printed (non-blocking).

---

## ⚙️ Configuration (Macros)

You can customize topics, incoming map cell meanings, offset, and logic thresholds via macros:

### Topics & leader→follower offset
Defined in `src/formation_obstacle_avoidance_follower_node.cpp`:
```cpp
// Map value conventions from your map source (change if needed)
#define ROS_OCC_FREE       0
#define ROS_OCC_OCCUPIED   100
#define ROS_OCC_UNKNOWN    -1

// Topic names (retarget as needed)
#define TOPIC_MAP           "map"
#define TOPIC_FOLLOWER_IN   "follower_point"
#define TOPIC_FOLLOWER_OUT  "new_follower_point"

// Follower offset relative to the leader
#define FOLLOWER_OFFSET_X   1.0
#define FOLLOWER_OFFSET_Y   0.0
```
> **Note:** Adjust `ROS_OCC_*` to match your map publisher. Unknown cells are treated as obstacles by default.

### Grid semantics & thresholds
Defined in your obstacle utils header (e.g., `include/formation_obstacle_avoidance_utils.hpp`):
```cpp
// Internal grid values
#define OCC_FREE                 0
#define OCC_PADDING              1
#define OCC_OBSTACLE             2

// Local patch/window half-width around contact (in grid cells)
#define PATCH_PAD_WIDTH          5

// Minimum obstacle width (in cells) along the row to declare deadlock
#define MIN_OBSTACLE_WIDTH       5

// Divergence threshold (in grid units/pixels)
#define FOLLOWER_DIVERGENCE_THRESHOLD 30
```

---

## 📁 Code Structure

- `src/formation_obstacle_avoidance_follower_node.cpp` — ROS 2 node (subscriptions, grid remap, online processing).
- `src/formation_obstacle_avoidance_follower_agent.cpp` / `include/formation_obstacle_avoidance_follower_agent.hpp` — **streaming** agent (`processSample`), keeps only minimal state.
- `src/formation_obstacle_avoidance_utils.cpp` / `include/formation_obstacle_avoidance_utils.hpp` — Collision/intersection checks, **window-based** patch extraction, rotation & row logic, deadlock/divergence helpers.
- `launch/formation_obstacle_avoidance_follower_launch.py` — Launch file.
- `CMakeLists.txt` — Build configuration.

---

## 🧪 Dependencies

- ROS 2 (e.g., Humble/Foxy)
- Eigen3

Install ROS 2 packages:
```bash
sudo apt install ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-nav-msgs
```

---

## 🛠️ Building

This node is built with the rest of the avt_341 package stack. Refer to the stack documentation for more details.

__On Linux:__
```bash 
colcon build --packages-select avt_341 avt_341_msgs
```

__On Windows:__
```bash 
colcon build --merge-install --packages-select avt_341 avt_341_msgs
```

---

## ▶️ Running

```bash
ros2 run avt_341 avt_341_obs_avoidance_goal_filter_node --ros-args -r __ns:=/<vehicle_name>

# Example
ros2 run avt_341 avt_341_obs_avoidance_goal_filter_node --ros-args -r __ns:=/mrzr2
```

Required topics:
- `<leader_vehicle>/avt_341/odometry` (`nav_msgs/msg/Odometry`)  ← Estimated leader pose
- `<follower_vhicle>/avt_341/occupancy_grid` (`nav_msgs/msg/OccupancyGrid`) ← ← for map frame & resolution
- `<follower_vehicle>/avt_341/candiate_follower_pose` (`geometry_msgs/msg/PoseStamped`)  ← Candidate follower target

Output:
- `<follower_vehicle>//avt_341/filtered_follower_pose` (`geometry_msgs/msg/PoseStamped`)  ← Corrected follower target

---

## 🔎 Notes & Tips

- **Map remap:** Unknown cells (`ROS_OCC_UNKNOWN`) are treated as obstacles by default. If your source uses different numeric conventions, **update the `ROS_OCC_*` macros**.
- **Units:** The divergence threshold uses the same units as your grid indices (often pixels/cells).
- **Window vs. component:** This implementation uses a **window** (not connected-component bbox). It’s fast and matches the final Python logic you provided.
- **State reset:** If you relocalize or need to clear history, add a small `reset()` to the agent to clear the last point and caches.

---

## 📦 Future Extensions

- Real-time replanning on divergence
- Multiple followers (instantiate more agents)
- RViz markers for diagnostics
- Parameterizing macros via ROS params or `-D` compile definitions

---

## 📝 License
