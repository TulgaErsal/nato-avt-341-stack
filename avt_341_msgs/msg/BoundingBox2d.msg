# This message represents a two-dimensional bounding box enclosing a region of
# the image frame where an object detector has formulated a hypothesis. The
# bounding box is aligned to image frame and may not be rotated, hence it is
# uniquely defined by its four corners.
#
# This message is not stamped, as it is meant to be part of a message including
# all detections provided by an object detector for a given image.

# Minimum x coordinate of the bounding box in pixels.
int32 x_min

# Maximum x coordinate of the bounding box in pixels.
int32 x_max

# Minimum y coordinate of the bounding box in pixels.
int32 y_min

# Maximum y coordinate of the bounding box in pixels.
int32 y_max