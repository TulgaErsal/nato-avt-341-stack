; Auto-generated. Do not edit!


(cl:in-package avt_341_msgs-msg)


;//! \htmlinclude OccupiedCell.msg.html

(cl:defclass <OccupiedCell> (roslisp-msg-protocol:ros-message)
  ((x_index
    :reader x_index
    :initarg :x_index
    :type cl:integer
    :initform 0)
   (y_index
    :reader y_index
    :initarg :y_index
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0))
)

(cl:defclass OccupiedCell (<OccupiedCell>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OccupiedCell>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OccupiedCell)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avt_341_msgs-msg:<OccupiedCell> is deprecated: use avt_341_msgs-msg:OccupiedCell instead.")))

(cl:ensure-generic-function 'x_index-val :lambda-list '(m))
(cl:defmethod x_index-val ((m <OccupiedCell>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:x_index-val is deprecated.  Use avt_341_msgs-msg:x_index instead.")
  (x_index m))

(cl:ensure-generic-function 'y_index-val :lambda-list '(m))
(cl:defmethod y_index-val ((m <OccupiedCell>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:y_index-val is deprecated.  Use avt_341_msgs-msg:y_index instead.")
  (y_index m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <OccupiedCell>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:data-val is deprecated.  Use avt_341_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OccupiedCell>) ostream)
  "Serializes a message object of type '<OccupiedCell>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'x_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'x_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'y_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'y_index)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OccupiedCell>) istream)
  "Deserializes a message object of type '<OccupiedCell>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'x_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'x_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'y_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'y_index)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OccupiedCell>)))
  "Returns string type for a message object of type '<OccupiedCell>"
  "avt_341_msgs/OccupiedCell")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OccupiedCell)))
  "Returns string type for a message object of type 'OccupiedCell"
  "avt_341_msgs/OccupiedCell")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OccupiedCell>)))
  "Returns md5sum for a message object of type '<OccupiedCell>"
  "3364ced6c6173e4d6f7f283e1479eee9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OccupiedCell)))
  "Returns md5sum for a message object of type 'OccupiedCell"
  "3364ced6c6173e4d6f7f283e1479eee9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OccupiedCell>)))
  "Returns full string definition for message of type '<OccupiedCell>"
  (cl:format cl:nil "uint32 x_index~%uint32 y_index~%int8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OccupiedCell)))
  "Returns full string definition for message of type 'OccupiedCell"
  (cl:format cl:nil "uint32 x_index~%uint32 y_index~%int8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OccupiedCell>))
  (cl:+ 0
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OccupiedCell>))
  "Converts a ROS message object to a list"
  (cl:list 'OccupiedCell
    (cl:cons ':x_index (x_index msg))
    (cl:cons ':y_index (y_index msg))
    (cl:cons ':data (data msg))
))
