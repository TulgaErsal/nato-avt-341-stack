# This message represents an object hypothesis formulated by an object detector
# for a given region of the image.

# The unique identifier of the object class as specified by the object detector.
int16 id

# The human-readable label of the object class as specified by the object
# detector.
string label

# The score associated with the hypothesis, as a floating point number between
# 0.0 and 1.0, including the limits.
float64 score