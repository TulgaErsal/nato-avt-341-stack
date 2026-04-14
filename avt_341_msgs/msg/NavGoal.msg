# Navigation goal to reach.

# Stamped header for the message.
std_msgs/Header header

# Pose to navigate to.
geometry_msgs/Pose pose

# Distance threshold at which to consider goal reached.
float64 threshold

# If provided orientation in goal pose should be matched.
bool use_orientation
