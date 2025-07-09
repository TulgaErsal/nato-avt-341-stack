; Auto-generated. Do not edit!


(cl:in-package avt_341_msgs-msg)


;//! \htmlinclude BoundingBox2d.msg.html

(cl:defclass <BoundingBox2d> (roslisp-msg-protocol:ros-message)
  ((x_min
    :reader x_min
    :initarg :x_min
    :type cl:integer
    :initform 0)
   (x_max
    :reader x_max
    :initarg :x_max
    :type cl:integer
    :initform 0)
   (y_min
    :reader y_min
    :initarg :y_min
    :type cl:integer
    :initform 0)
   (y_max
    :reader y_max
    :initarg :y_max
    :type cl:integer
    :initform 0))
)

(cl:defclass BoundingBox2d (<BoundingBox2d>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BoundingBox2d>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BoundingBox2d)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avt_341_msgs-msg:<BoundingBox2d> is deprecated: use avt_341_msgs-msg:BoundingBox2d instead.")))

(cl:ensure-generic-function 'x_min-val :lambda-list '(m))
(cl:defmethod x_min-val ((m <BoundingBox2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:x_min-val is deprecated.  Use avt_341_msgs-msg:x_min instead.")
  (x_min m))

(cl:ensure-generic-function 'x_max-val :lambda-list '(m))
(cl:defmethod x_max-val ((m <BoundingBox2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:x_max-val is deprecated.  Use avt_341_msgs-msg:x_max instead.")
  (x_max m))

(cl:ensure-generic-function 'y_min-val :lambda-list '(m))
(cl:defmethod y_min-val ((m <BoundingBox2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:y_min-val is deprecated.  Use avt_341_msgs-msg:y_min instead.")
  (y_min m))

(cl:ensure-generic-function 'y_max-val :lambda-list '(m))
(cl:defmethod y_max-val ((m <BoundingBox2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:y_max-val is deprecated.  Use avt_341_msgs-msg:y_max instead.")
  (y_max m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BoundingBox2d>) ostream)
  "Serializes a message object of type '<BoundingBox2d>"
  (cl:let* ((signed (cl:slot-value msg 'x_min)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'x_max)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y_min)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y_max)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BoundingBox2d>) istream)
  "Deserializes a message object of type '<BoundingBox2d>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x_min) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x_max) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y_min) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y_max) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BoundingBox2d>)))
  "Returns string type for a message object of type '<BoundingBox2d>"
  "avt_341_msgs/BoundingBox2d")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BoundingBox2d)))
  "Returns string type for a message object of type 'BoundingBox2d"
  "avt_341_msgs/BoundingBox2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BoundingBox2d>)))
  "Returns md5sum for a message object of type '<BoundingBox2d>"
  "da93b905cb7f2e0b04662214969ec2ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BoundingBox2d)))
  "Returns md5sum for a message object of type 'BoundingBox2d"
  "da93b905cb7f2e0b04662214969ec2ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BoundingBox2d>)))
  "Returns full string definition for message of type '<BoundingBox2d>"
  (cl:format cl:nil "# This message represents a two-dimensional bounding box enclosing a region of~%# the image frame where an object detector has formulated a hypothesis. The~%# bounding box is aligned to image frame and may not be rotated, hence it is~%# uniquely defined by its four corners.~%#~%# This message is not stamped, as it is meant to be part of a message including~%# all detections provided by an object detector for a given image.~%~%# Minimum x coordinate of the bounding box in pixels.~%int32 x_min~%~%# Maximum x coordinate of the bounding box in pixels.~%int32 x_max~%~%# Minimum y coordinate of the bounding box in pixels.~%int32 y_min~%~%# Maximum y coordinate of the bounding box in pixels.~%int32 y_max~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BoundingBox2d)))
  "Returns full string definition for message of type 'BoundingBox2d"
  (cl:format cl:nil "# This message represents a two-dimensional bounding box enclosing a region of~%# the image frame where an object detector has formulated a hypothesis. The~%# bounding box is aligned to image frame and may not be rotated, hence it is~%# uniquely defined by its four corners.~%#~%# This message is not stamped, as it is meant to be part of a message including~%# all detections provided by an object detector for a given image.~%~%# Minimum x coordinate of the bounding box in pixels.~%int32 x_min~%~%# Maximum x coordinate of the bounding box in pixels.~%int32 x_max~%~%# Minimum y coordinate of the bounding box in pixels.~%int32 y_min~%~%# Maximum y coordinate of the bounding box in pixels.~%int32 y_max~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BoundingBox2d>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BoundingBox2d>))
  "Converts a ROS message object to a list"
  (cl:list 'BoundingBox2d
    (cl:cons ':x_min (x_min msg))
    (cl:cons ':x_max (x_max msg))
    (cl:cons ':y_min (y_min msg))
    (cl:cons ':y_max (y_max msg))
))
