# Navigation goal to reach.

# Stamped header for the message.
std_msgs/Header header

# Pose to navigate to.
geometry_msgs/Pose pose

# Distance threshold at which to consider goal reached.
# Put value < 0 to use stack default.
float64 dist_threshold -1

# Yaw threshold (in radians) at which to consider goal reached.
# Put value > pi to disable, value < 0 to use stack default.
float64 yaw_threshold -1
