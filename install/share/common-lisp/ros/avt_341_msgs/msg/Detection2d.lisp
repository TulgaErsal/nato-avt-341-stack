; Auto-generated. Do not edit!


(cl:in-package avt_341_msgs-msg)


;//! \htmlinclude Detection2d.msg.html

(cl:defclass <Detection2d> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (bounding_box
    :reader bounding_box
    :initarg :bounding_box
    :type avt_341_msgs-msg:BoundingBox2d
    :initform (cl:make-instance 'avt_341_msgs-msg:BoundingBox2d))
   (hypothesis
    :reader hypothesis
    :initarg :hypothesis
    :type avt_341_msgs-msg:Hypothesis
    :initform (cl:make-instance 'avt_341_msgs-msg:Hypothesis)))
)

(cl:defclass Detection2d (<Detection2d>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Detection2d>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Detection2d)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avt_341_msgs-msg:<Detection2d> is deprecated: use avt_341_msgs-msg:Detection2d instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Detection2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:header-val is deprecated.  Use avt_341_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'bounding_box-val :lambda-list '(m))
(cl:defmethod bounding_box-val ((m <Detection2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:bounding_box-val is deprecated.  Use avt_341_msgs-msg:bounding_box instead.")
  (bounding_box m))

(cl:ensure-generic-function 'hypothesis-val :lambda-list '(m))
(cl:defmethod hypothesis-val ((m <Detection2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:hypothesis-val is deprecated.  Use avt_341_msgs-msg:hypothesis instead.")
  (hypothesis m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Detection2d>) ostream)
  "Serializes a message object of type '<Detection2d>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bounding_box) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'hypothesis) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Detection2d>) istream)
  "Deserializes a message object of type '<Detection2d>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bounding_box) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'hypothesis) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Detection2d>)))
  "Returns string type for a message object of type '<Detection2d>"
  "avt_341_msgs/Detection2d")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Detection2d)))
  "Returns string type for a message object of type 'Detection2d"
  "avt_341_msgs/Detection2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Detection2d>)))
  "Returns md5sum for a message object of type '<Detection2d>"
  "aa9743fbf372d547819db142e9d8db87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Detection2d)))
  "Returns md5sum for a message object of type 'Detection2d"
  "aa9743fbf372d547819db142e9d8db87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Detection2d>)))
  "Returns full string definition for message of type '<Detection2d>"
  (cl:format cl:nil "# This message represents a single detection associated with an image based on~%# the best scoring hypothesis after intersection-over-union (IoU) filtering.~%#~%# This message is not stamped, as it is meant to be part of a message including~%# all detections provided by an object detector for a given image.~%~%# Message header.~%std_msgs/Header header~%~%# Detection bounding box.~%BoundingBox2d bounding_box~%~%# Detection hypothesis.~%Hypothesis hypothesis~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: avt_341_msgs/BoundingBox2d~%# This message represents a two-dimensional bounding box enclosing a region of~%# the image frame where an object detector has formulated a hypothesis. The~%# bounding box is aligned to image frame and may not be rotated, hence it is~%# uniquely defined by its four corners.~%#~%# This message is not stamped, as it is meant to be part of a message including~%# all detections provided by an object detector for a given image.~%~%# Minimum x coordinate of the bounding box in pixels.~%int32 x_min~%~%# Maximum x coordinate of the bounding box in pixels.~%int32 x_max~%~%# Minimum y coordinate of the bounding box in pixels.~%int32 y_min~%~%# Maximum y coordinate of the bounding box in pixels.~%int32 y_max~%================================================================================~%MSG: avt_341_msgs/Hypothesis~%# This message represents an object hypothesis formulated by an object detector~%# for a given region of the image.~%~%# The unique identifier of the object class as specified by the object detector.~%int16 id~%~%# The human-readable label of the object class as specified by the object~%# detector.~%string label~%~%# The score associated with the hypothesis, as a floating point number between~%# 0.0 and 1.0, including the limits.~%float64 score~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Detection2d)))
  "Returns full string definition for message of type 'Detection2d"
  (cl:format cl:nil "# This message represents a single detection associated with an image based on~%# the best scoring hypothesis after intersection-over-union (IoU) filtering.~%#~%# This message is not stamped, as it is meant to be part of a message including~%# all detections provided by an object detector for a given image.~%~%# Message header.~%std_msgs/Header header~%~%# Detection bounding box.~%BoundingBox2d bounding_box~%~%# Detection hypothesis.~%Hypothesis hypothesis~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: avt_341_msgs/BoundingBox2d~%# This message represents a two-dimensional bounding box enclosing a region of~%# the image frame where an object detector has formulated a hypothesis. The~%# bounding box is aligned to image frame and may not be rotated, hence it is~%# uniquely defined by its four corners.~%#~%# This message is not stamped, as it is meant to be part of a message including~%# all detections provided by an object detector for a given image.~%~%# Minimum x coordinate of the bounding box in pixels.~%int32 x_min~%~%# Maximum x coordinate of the bounding box in pixels.~%int32 x_max~%~%# Minimum y coordinate of the bounding box in pixels.~%int32 y_min~%~%# Maximum y coordinate of the bounding box in pixels.~%int32 y_max~%================================================================================~%MSG: avt_341_msgs/Hypothesis~%# This message represents an object hypothesis formulated by an object detector~%# for a given region of the image.~%~%# The unique identifier of the object class as specified by the object detector.~%int16 id~%~%# The human-readable label of the object class as specified by the object~%# detector.~%string label~%~%# The score associated with the hypothesis, as a floating point number between~%# 0.0 and 1.0, including the limits.~%float64 score~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Detection2d>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bounding_box))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'hypothesis))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Detection2d>))
  "Converts a ROS message object to a list"
  (cl:list 'Detection2d
    (cl:cons ':header (header msg))
    (cl:cons ':bounding_box (bounding_box msg))
    (cl:cons ':hypothesis (hypothesis msg))
))
