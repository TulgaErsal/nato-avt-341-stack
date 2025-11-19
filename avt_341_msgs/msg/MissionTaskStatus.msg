# Stamped header for the message.
std_msgs/Header header

# Mission manager task ID being executing associated with the current formation.
int32 task_id

# Human-readable description of the current task being executed.
string task_description

# Vehicle currently being followed in formation by ego-vehicle. Empty if ego-vehicle is leader / not following another vehicle.
string tracked_vehicle

# Type of formation being executed. Empty if no formation.
string formation_type

# List of vehicles involved in the formation. Empty if no formation.
string[] formation_vehicles
