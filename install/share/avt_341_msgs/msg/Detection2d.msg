# This message represents a single detection associated with an image based on
# the best scoring hypothesis after intersection-over-union (IoU) filtering.
#
# This message is not stamped, as it is meant to be part of a message including
# all detections provided by an object detector for a given image.

# Message header.
std_msgs/Header header

# Detection bounding box.
BoundingBox2d bounding_box

# Detection hypothesis.
Hypothesis hypothesis