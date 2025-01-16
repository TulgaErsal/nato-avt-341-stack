# This message contains the individual objective function cost terms values for
# a trajectory planned by the Dynamic Window Approach (DWA) planner. All cost
# terms in this message are already scaled by the user-defined weight factors.

# Goal progress cost term.
float64 goal_cost

# Obstacle avoidance cost term.
float64 obstacle_cost

# Segmentation grid cost term.
float64 segmentation_cost

# Heading deviation cost term.
float64 heading_cost

# Target speed deviation cost term.
float64 speed_cost

# Current trajectory deviation cost term.
float64 deviation_cost