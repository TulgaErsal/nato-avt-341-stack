# State of navigation

# Stamped header for the message.
std_msgs/Header header

# Current navigation goal.
avt_341_msgs/NavGoal goal

# Running state of global planner. Constants mirror the NavStackState enum
# -1 - uninitialized
# 0 - active
# 1 - no active goal - ctrl applies coasting cmd (vel_x/thr~0, str=0, brk=0)
# 2 - no active goal - ctrl applies ramping brake cmd (vel_x/thr=0, str=0, brk=0-1)
# 3 - no active goal - ctrl applies immediate max brake cmd (vel_x/thr=0, str=0, brk=1)
int32 NOT_INIT = -1
int32 ACTIVE = 0
int32 INACTIVE_COAST = 1
int32 INACTIVE_GRADUAL_STOP = 2
int32 INACTIVE_HARD_STOP = 3

int32 run_state

# How long (in seconds) navigation to the current goal has been running.
float64 goal_duration

# Current distance to the goal.
float64 goal_distance

# Current yaw difference (radians) to the goal
float64 goal_yaw_difference