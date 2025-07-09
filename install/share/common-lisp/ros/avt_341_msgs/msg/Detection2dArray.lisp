; Auto-generated. Do not edit!


(cl:in-package avt_341_msgs-msg)


;//! \htmlinclude Detection2dArray.msg.html

(cl:defclass <Detection2dArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detections
    :reader detections
    :initarg :detections
    :type (cl:vector avt_341_msgs-msg:Detection2d)
   :initform (cl:make-array 0 :element-type 'avt_341_msgs-msg:Detection2d :initial-element (cl:make-instance 'avt_341_msgs-msg:Detection2d))))
)

(cl:defclass Detection2dArray (<Detection2dArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Detection2dArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Detection2dArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avt_341_msgs-msg:<Detection2dArray> is deprecated: use avt_341_msgs-msg:Detection2dArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Detection2dArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:header-val is deprecated.  Use avt_341_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <Detection2dArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:detections-val is deprecated.  Use avt_341_msgs-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Detection2dArray>) ostream)
  "Serializes a message object of type '<Detection2dArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Detection2dArray>) istream)
  "Deserializes a message object of type '<Detection2dArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'avt_341_msgs-msg:Detection2d))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Detection2dArray>)))
  "Returns string type for a message object of type '<Detection2dArray>"
  "avt_341_msgs/Detection2dArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Detection2dArray)))
  "Returns string type for a message object of type 'Detection2dArray"
  "avt_341_msgs/Detection2dArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Detection2dArray>)))
  "Returns md5sum for a message object of type '<Detection2dArray>"
  "4bd5549dd2967ed5228b993c3abd5a0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Detection2dArray)))
  "Returns md5sum for a message object of type 'Detection2dArray"
  "4bd5549dd2967ed5228b993c3abd5a0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Detection2dArray>)))
  "Returns full string definition for message of type '<Detection2dArray>"
  (cl:format cl:nil "# This message represents the results of an object detector for a given image~%# frame.~%~%# Header stamped at the completion time of the detection.~%std_msgs/Header header~%~%# Collection of detections for the given image frame, in the order specified by~%# the image detector.~%Detection2d[] detections~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: avt_341_msgs/Detection2d~%# This message represents a single detection associated with an image based on~%# the best scoring hypothesis after intersection-over-union (IoU) filtering.~%#~%# This message is not stamped, as it is meant to be part of a message including~%# all detections provided by an object detector for a given image.~%~%# Message header.~%std_msgs/Header header~%~%# Detection bounding box.~%BoundingBox2d bounding_box~%~%# Detection hypothesis.~%Hypothesis hypothesis~%================================================================================~%MSG: avt_341_msgs/BoundingBox2d~%# This message represents a two-dimensional bounding box enclosing a region of~%# the image frame where an object detector has formulated a hypothesis. The~%# bounding box is aligned to image frame and may not be rotated, hence it is~%# uniquely defined by its four corners.~%#~%# This message is not stamped, as it is meant to be part of a message including~%# all detections provided by an object detector for a given image.~%~%# Minimum x coordinate of the bounding box in pixels.~%int32 x_min~%~%# Maximum x coordinate of the bounding box in pixels.~%int32 x_max~%~%# Minimum y coordinate of the bounding box in pixels.~%int32 y_min~%~%# Maximum y coordinate of the bounding box in pixels.~%int32 y_max~%================================================================================~%MSG: avt_341_msgs/Hypothesis~%# This message represents an object hypothesis formulated by an object detector~%# for a given region of the image.~%~%# The unique identifier of the object class as specified by the object detector.~%int16 id~%~%# The human-readable label of the object class as specified by the object~%# detector.~%string label~%~%# The score associated with the hypothesis, as a floating point number between~%# 0.0 and 1.0, including the limits.~%float64 score~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Detection2dArray)))
  "Returns full string definition for message of type 'Detection2dArray"
  (cl:format cl:nil "# This message represents the results of an object detector for a given image~%# frame.~%~%# Header stamped at the completion time of the detection.~%std_msgs/Header header~%~%# Collection of detections for the given image frame, in the order specified by~%# the image detector.~%Detection2d[] detections~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: avt_341_msgs/Detection2d~%# This message represents a single detection associated with an image based on~%# the best scoring hypothesis after intersection-over-union (IoU) filtering.~%#~%# This message is not stamped, as it is meant to be part of a message including~%# all detections provided by an object detector for a given image.~%~%# Message header.~%std_msgs/Header header~%~%# Detection bounding box.~%BoundingBox2d bounding_box~%~%# Detection hypothesis.~%Hypothesis hypothesis~%================================================================================~%MSG: avt_341_msgs/BoundingBox2d~%# This message represents a two-dimensional bounding box enclosing a region of~%# the image frame where an object detector has formulated a hypothesis. The~%# bounding box is aligned to image frame and may not be rotated, hence it is~%# uniquely defined by its four corners.~%#~%# This message is not stamped, as it is meant to be part of a message including~%# all detections provided by an object detector for a given image.~%~%# Minimum x coordinate of the bounding box in pixels.~%int32 x_min~%~%# Maximum x coordinate of the bounding box in pixels.~%int32 x_max~%~%# Minimum y coordinate of the bounding box in pixels.~%int32 y_min~%~%# Maximum y coordinate of the bounding box in pixels.~%int32 y_max~%================================================================================~%MSG: avt_341_msgs/Hypothesis~%# This message represents an object hypothesis formulated by an object detector~%# for a given region of the image.~%~%# The unique identifier of the object class as specified by the object detector.~%int16 id~%~%# The human-readable label of the object class as specified by the object~%# detector.~%string label~%~%# The score associated with the hypothesis, as a floating point number between~%# 0.0 and 1.0, including the limits.~%float64 score~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Detection2dArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Detection2dArray>))
  "Converts a ROS message object to a list"
  (cl:list 'Detection2dArray
    (cl:cons ':header (header msg))
    (cl:cons ':detections (detections msg))
))
