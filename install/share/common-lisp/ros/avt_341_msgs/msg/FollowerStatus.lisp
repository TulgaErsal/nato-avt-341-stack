; Auto-generated. Do not edit!


(cl:in-package avt_341_msgs-msg)


;//! \htmlinclude FollowerStatus.msg.html

(cl:defclass <FollowerStatus> (roslisp-msg-protocol:ros-message)
  ((leader_name
    :reader leader_name
    :initarg :leader_name
    :type cl:string
    :initform "")
   (x_offset
    :reader x_offset
    :initarg :x_offset
    :type cl:float
    :initform 0.0)
   (y_offset
    :reader y_offset
    :initarg :y_offset
    :type cl:float
    :initform 0.0)
   (use_leader
    :reader use_leader
    :initarg :use_leader
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FollowerStatus (<FollowerStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FollowerStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FollowerStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avt_341_msgs-msg:<FollowerStatus> is deprecated: use avt_341_msgs-msg:FollowerStatus instead.")))

(cl:ensure-generic-function 'leader_name-val :lambda-list '(m))
(cl:defmethod leader_name-val ((m <FollowerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:leader_name-val is deprecated.  Use avt_341_msgs-msg:leader_name instead.")
  (leader_name m))

(cl:ensure-generic-function 'x_offset-val :lambda-list '(m))
(cl:defmethod x_offset-val ((m <FollowerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:x_offset-val is deprecated.  Use avt_341_msgs-msg:x_offset instead.")
  (x_offset m))

(cl:ensure-generic-function 'y_offset-val :lambda-list '(m))
(cl:defmethod y_offset-val ((m <FollowerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:y_offset-val is deprecated.  Use avt_341_msgs-msg:y_offset instead.")
  (y_offset m))

(cl:ensure-generic-function 'use_leader-val :lambda-list '(m))
(cl:defmethod use_leader-val ((m <FollowerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:use_leader-val is deprecated.  Use avt_341_msgs-msg:use_leader instead.")
  (use_leader m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FollowerStatus>) ostream)
  "Serializes a message object of type '<FollowerStatus>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'leader_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'leader_name))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_leader) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FollowerStatus>) istream)
  "Deserializes a message object of type '<FollowerStatus>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'leader_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'leader_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_offset) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_offset) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'use_leader) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FollowerStatus>)))
  "Returns string type for a message object of type '<FollowerStatus>"
  "avt_341_msgs/FollowerStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FollowerStatus)))
  "Returns string type for a message object of type 'FollowerStatus"
  "avt_341_msgs/FollowerStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FollowerStatus>)))
  "Returns md5sum for a message object of type '<FollowerStatus>"
  "61986abde49829e549c712689cef646e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FollowerStatus)))
  "Returns md5sum for a message object of type 'FollowerStatus"
  "61986abde49829e549c712689cef646e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FollowerStatus>)))
  "Returns full string definition for message of type '<FollowerStatus>"
  (cl:format cl:nil "string leader_name~%float64 x_offset~%float64 y_offset~%bool use_leader~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FollowerStatus)))
  "Returns full string definition for message of type 'FollowerStatus"
  (cl:format cl:nil "string leader_name~%float64 x_offset~%float64 y_offset~%bool use_leader~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FollowerStatus>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'leader_name))
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FollowerStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'FollowerStatus
    (cl:cons ':leader_name (leader_name msg))
    (cl:cons ':x_offset (x_offset msg))
    (cl:cons ':y_offset (y_offset msg))
    (cl:cons ':use_leader (use_leader msg))
))
