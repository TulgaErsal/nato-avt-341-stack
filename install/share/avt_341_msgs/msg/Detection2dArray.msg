# This message represents the results of an object detector for a given image
# frame.

# Header stamped at the completion time of the detection.
std_msgs/Header header

# Collection of detections for the given image frame, in the order specified by
# the image detector.
Detection2d[] detections