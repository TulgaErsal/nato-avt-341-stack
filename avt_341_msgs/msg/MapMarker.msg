# A single map marker (a mission point or target of interest).

# Stamped header for the message.
std_msgs/Header header

# Unique identifier for this marker.
string marker_id

# Human-readable label for this marker.
string label

# Marker type enumeration. 0 = MissionPoint, 1 = TargetOfInterest.
uint8 MISSION_POINT=0
uint8 TARGET_OF_INTEREST=1
uint8 type

# Pose (position and orientation) of the marker.
geometry_msgs/Pose pose
