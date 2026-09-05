# Status information for currently running mission task

# Stamped header for the message.
std_msgs/Header header

# Mission manager task ID being executing associated with the current formation. -1 if no task is running.
int32 task_id -1

# Human-readable description of the current task being executed.
string task_description

# Vehicle currently being followed in formation by ego-vehicle. Empty if ego-vehicle is leader / not following another vehicle.
# is_leader in formation: if !formation_type.empty() && tracked_vehicle.empty()
string tracked_vehicle

# Type of formation being executed. Empty if no formation.
string formation_type

# Target pose associated with task (if any). For example, formation target or move to goal waypoint.
geometry_msgs/Pose target_pose

# List of vehicles involved in the formation. Empty if no formation.
string[] formation_vehicles

# Formation offset of the ego-vehicle from the tracked vehicle, in the tracked vehicle's frame [m].
# Only meaningful while tracked_vehicle is non-empty.
float64 formation_x_offset
float64 formation_y_offset

# Speed setpoint [m/s] the mission manager commands while this task runs.
# Priority: the task's own speed override, else the SET_SPEED value, else the default max speed.
# -1 if no task is running.
float64 task_speed -1.0
