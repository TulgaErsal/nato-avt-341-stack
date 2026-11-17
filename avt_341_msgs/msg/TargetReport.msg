# One report of a target of interest by the object tracker (avt_341/target_reports, avt_341/target_released).

uint8 RANGING_CAMERA=0
uint8 RANGING_LIDAR=1

# Class of the target (the tracker's target class name).
string target_class

# Start of the track that made the report: its identity. Zero for a sighting (a target the tracker saw while
# following another one).
builtin_interfaces/Time track_start

# Target position in the map frame, and its heading [rad].
geometry_msgs/Point position
float64 yaw

# RANGING_LIDAR: the position is a LiDAR box of the target. RANGING_CAMERA: its range is the camera estimate, whose
# bearing is good and whose range is not.
uint8 ranging

# Camera position in the map frame when the target was detected (the origin of its line of sight).
geometry_msgs/Point observer
