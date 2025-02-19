
# This message contains planning step recap information for the Dynamic Window
# Approach (DWA) planner.

# Stamped header for the planner completion time.
std_msgs/Header header

# The optimal trajectory for this planning step.
DwaTrajectory optimal_trajectory

# Array of planned trajectories. This field is optional and may not be populated
# based on planner settings. When provided, this field contains all planned
# trajectories, including the optimal one.
DwaTrajectory[] planned_trajectories
