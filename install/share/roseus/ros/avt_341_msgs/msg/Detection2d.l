;; Auto-generated. Do not edit!


(when (boundp 'avt_341_msgs::Detection2d)
  (if (not (find-package "AVT_341_MSGS"))
    (make-package "AVT_341_MSGS"))
  (shadow 'Detection2d (find-package "AVT_341_MSGS")))
(unless (find-package "AVT_341_MSGS::DETECTION2D")
  (make-package "AVT_341_MSGS::DETECTION2D"))

(in-package "ROS")
;;//! \htmlinclude Detection2d.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass avt_341_msgs::Detection2d
  :super ros::object
  :slots (_header _bounding_box _hypothesis ))

(defmethod avt_341_msgs::Detection2d
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:bounding_box __bounding_box) (instance avt_341_msgs::BoundingBox2d :init))
    ((:hypothesis __hypothesis) (instance avt_341_msgs::Hypothesis :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _bounding_box __bounding_box)
   (setq _hypothesis __hypothesis)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:bounding_box
   (&rest __bounding_box)
   (if (keywordp (car __bounding_box))
       (send* _bounding_box __bounding_box)
     (progn
       (if __bounding_box (setq _bounding_box (car __bounding_box)))
       _bounding_box)))
  (:hypothesis
   (&rest __hypothesis)
   (if (keywordp (car __hypothesis))
       (send* _hypothesis __hypothesis)
     (progn
       (if __hypothesis (setq _hypothesis (car __hypothesis)))
       _hypothesis)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; avt_341_msgs/BoundingBox2d _bounding_box
    (send _bounding_box :serialization-length)
    ;; avt_341_msgs/Hypothesis _hypothesis
    (send _hypothesis :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; avt_341_msgs/BoundingBox2d _bounding_box
       (send _bounding_box :serialize s)
     ;; avt_341_msgs/Hypothesis _hypothesis
       (send _hypothesis :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; avt_341_msgs/BoundingBox2d _bounding_box
     (send _bounding_box :deserialize buf ptr-) (incf ptr- (send _bounding_box :serialization-length))
   ;; avt_341_msgs/Hypothesis _hypothesis
     (send _hypothesis :deserialize buf ptr-) (incf ptr- (send _hypothesis :serialization-length))
   ;;
   self)
  )

(setf (get avt_341_msgs::Detection2d :md5sum-) "aa9743fbf372d547819db142e9d8db87")
(setf (get avt_341_msgs::Detection2d :datatype-) "avt_341_msgs/Detection2d")
(setf (get avt_341_msgs::Detection2d :definition-)
      "# This message represents a single detection associated with an image based on
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
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: avt_341_msgs/BoundingBox2d
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
================================================================================
MSG: avt_341_msgs/Hypothesis
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
")



(provide :avt_341_msgs/Detection2d "aa9743fbf372d547819db142e9d8db87")


