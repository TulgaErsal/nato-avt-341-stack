# A message defining a static obstacle in three-dimensional space.

# Header defining the obstacle detection time and reference frame of the
# obstacle pose.
std_msgs/Header header

# Obstacle types.
# 0 - TYPE_BOX: Box obstacle. Suitable for accurate representation of occupancy
#               grid cells.
# 1 - TYPE_CYLINDER: Cylindrical obstacle. Suitable for simplified
#                    representation of inflated occupancy grid cells.
uint8 TYPE_BOX=0
uint8 TYPE_CYLINDER=1

# The type of obstacle represented.
uint8 type

# The pose of the obstacle in three-dimensional space.
geometry_msgs/Pose pose

# The estimated size of the obstacle, measured in meters, relative to a
# reference point for a given shape.
#
# - For TYPE_BOX, the reference point is the centroid of the box the components
#   are the box extents in the X, Y and Z (if known, -1.0 otherwise) directions
#   respectively [x, y, z], measured in the box frame of reference.
# - For TYPE_CYLINDER obstacles, the components are the circle radius, the
#   height of the obstacles (if known, -1.0 otherwise), and -1.0.
geometry_msgs/Vector3 extents
