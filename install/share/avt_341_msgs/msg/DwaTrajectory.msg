# This message contains information on a simulated trajectory generated and
# evaluated by the Dynamic Window Approach (DWA) planner.

# Predicted path for the simulated trajectory. 
nav_msgs/Path path

# The individual cost term values for the objective function across the entire
# trajectory. The values contained in this field are already scaled by the planner
# weights.
DwaObjective objective

# Cumulative cost for all objective function cost terms after scaling. This
# field is equivalent to the sum of all the individual terms contained in the
# objective function message. 
float64 cost
