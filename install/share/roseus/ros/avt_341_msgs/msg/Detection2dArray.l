;; Auto-generated. Do not edit!


(when (boundp 'avt_341_msgs::Detection2dArray)
  (if (not (find-package "AVT_341_MSGS"))
    (make-package "AVT_341_MSGS"))
  (shadow 'Detection2dArray (find-package "AVT_341_MSGS")))
(unless (find-package "AVT_341_MSGS::DETECTION2DARRAY")
  (make-package "AVT_341_MSGS::DETECTION2DARRAY"))

(in-package "ROS")
;;//! \htmlinclude Detection2dArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass avt_341_msgs::Detection2dArray
  :super ros::object
  :slots (_header _detections ))

(defmethod avt_341_msgs::Detection2dArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detections __detections) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _detections __detections)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; avt_341_msgs/Detection2d[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; avt_341_msgs/Detection2d[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; avt_341_msgs/Detection2d[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance avt_341_msgs::Detection2d :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get avt_341_msgs::Detection2dArray :md5sum-) "4bd5549dd2967ed5228b993c3abd5a0f")
(setf (get avt_341_msgs::Detection2dArray :datatype-) "avt_341_msgs/Detection2dArray")
(setf (get avt_341_msgs::Detection2dArray :definition-)
      "# This message represents the results of an object detector for a given image
# frame.

# Header stamped at the completion time of the detection.
std_msgs/Header header

# Collection of detections for the given image frame, in the order specified by
# the image detector.
Detection2d[] detections
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
MSG: avt_341_msgs/Detection2d
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



(provide :avt_341_msgs/Detection2dArray "4bd5549dd2967ed5228b993c3abd5a0f")


