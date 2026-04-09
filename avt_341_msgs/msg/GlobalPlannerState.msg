# State of global planner module.

# Stamped header for the message.
std_msgs/Header header

# Current navigation goal.
avt_341_msgs/NavGoal goal

# Running state of global planner.
# -1 - startup (stopped and not shut down)
# 0 - active
# 1 - bring to a smooth stop but do not shut down
# 2 - bring to a smooth stop and shut down
# 3 - bring to an immediate stop (hard braking) and shut down
int32 run_state

# How long (in seconds) navigation to the current goal has been running.
float64 navigation_duration

# Current distance to the goal.
float64 goal_distance