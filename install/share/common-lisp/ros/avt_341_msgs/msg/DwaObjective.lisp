; Auto-generated. Do not edit!


(cl:in-package avt_341_msgs-msg)


;//! \htmlinclude DwaObjective.msg.html

(cl:defclass <DwaObjective> (roslisp-msg-protocol:ros-message)
  ((goal_cost
    :reader goal_cost
    :initarg :goal_cost
    :type cl:float
    :initform 0.0)
   (obstacle_cost
    :reader obstacle_cost
    :initarg :obstacle_cost
    :type cl:float
    :initform 0.0)
   (segmentation_cost
    :reader segmentation_cost
    :initarg :segmentation_cost
    :type cl:float
    :initform 0.0)
   (heading_cost
    :reader heading_cost
    :initarg :heading_cost
    :type cl:float
    :initform 0.0)
   (speed_cost
    :reader speed_cost
    :initarg :speed_cost
    :type cl:float
    :initform 0.0)
   (deviation_cost
    :reader deviation_cost
    :initarg :deviation_cost
    :type cl:float
    :initform 0.0))
)

(cl:defclass DwaObjective (<DwaObjective>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DwaObjective>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DwaObjective)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avt_341_msgs-msg:<DwaObjective> is deprecated: use avt_341_msgs-msg:DwaObjective instead.")))

(cl:ensure-generic-function 'goal_cost-val :lambda-list '(m))
(cl:defmethod goal_cost-val ((m <DwaObjective>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:goal_cost-val is deprecated.  Use avt_341_msgs-msg:goal_cost instead.")
  (goal_cost m))

(cl:ensure-generic-function 'obstacle_cost-val :lambda-list '(m))
(cl:defmethod obstacle_cost-val ((m <DwaObjective>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:obstacle_cost-val is deprecated.  Use avt_341_msgs-msg:obstacle_cost instead.")
  (obstacle_cost m))

(cl:ensure-generic-function 'segmentation_cost-val :lambda-list '(m))
(cl:defmethod segmentation_cost-val ((m <DwaObjective>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:segmentation_cost-val is deprecated.  Use avt_341_msgs-msg:segmentation_cost instead.")
  (segmentation_cost m))

(cl:ensure-generic-function 'heading_cost-val :lambda-list '(m))
(cl:defmethod heading_cost-val ((m <DwaObjective>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:heading_cost-val is deprecated.  Use avt_341_msgs-msg:heading_cost instead.")
  (heading_cost m))

(cl:ensure-generic-function 'speed_cost-val :lambda-list '(m))
(cl:defmethod speed_cost-val ((m <DwaObjective>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:speed_cost-val is deprecated.  Use avt_341_msgs-msg:speed_cost instead.")
  (speed_cost m))

(cl:ensure-generic-function 'deviation_cost-val :lambda-list '(m))
(cl:defmethod deviation_cost-val ((m <DwaObjective>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:deviation_cost-val is deprecated.  Use avt_341_msgs-msg:deviation_cost instead.")
  (deviation_cost m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DwaObjective>) ostream)
  "Serializes a message object of type '<DwaObjective>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goal_cost))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'obstacle_cost))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'segmentation_cost))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heading_cost))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'speed_cost))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'deviation_cost))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DwaObjective>) istream)
  "Deserializes a message object of type '<DwaObjective>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal_cost) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obstacle_cost) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'segmentation_cost) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_cost) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_cost) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'deviation_cost) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DwaObjective>)))
  "Returns string type for a message object of type '<DwaObjective>"
  "avt_341_msgs/DwaObjective")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DwaObjective)))
  "Returns string type for a message object of type 'DwaObjective"
  "avt_341_msgs/DwaObjective")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DwaObjective>)))
  "Returns md5sum for a message object of type '<DwaObjective>"
  "0c23918921b89b7797bde9cc90d54688")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DwaObjective)))
  "Returns md5sum for a message object of type 'DwaObjective"
  "0c23918921b89b7797bde9cc90d54688")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DwaObjective>)))
  "Returns full string definition for message of type '<DwaObjective>"
  (cl:format cl:nil "# This message contains the individual objective function cost terms values for~%# a trajectory planned by the Dynamic Window Approach (DWA) planner. All cost~%# terms in this message are already scaled by the user-defined weight factors.~%~%# Goal progress cost term.~%float64 goal_cost~%~%# Obstacle avoidance cost term.~%float64 obstacle_cost~%~%# Segmentation grid cost term.~%float64 segmentation_cost~%~%# Heading deviation cost term.~%float64 heading_cost~%~%# Target speed deviation cost term.~%float64 speed_cost~%~%# Current trajectory deviation cost term.~%float64 deviation_cost~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DwaObjective)))
  "Returns full string definition for message of type 'DwaObjective"
  (cl:format cl:nil "# This message contains the individual objective function cost terms values for~%# a trajectory planned by the Dynamic Window Approach (DWA) planner. All cost~%# terms in this message are already scaled by the user-defined weight factors.~%~%# Goal progress cost term.~%float64 goal_cost~%~%# Obstacle avoidance cost term.~%float64 obstacle_cost~%~%# Segmentation grid cost term.~%float64 segmentation_cost~%~%# Heading deviation cost term.~%float64 heading_cost~%~%# Target speed deviation cost term.~%float64 speed_cost~%~%# Current trajectory deviation cost term.~%float64 deviation_cost~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DwaObjective>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DwaObjective>))
  "Converts a ROS message object to a list"
  (cl:list 'DwaObjective
    (cl:cons ':goal_cost (goal_cost msg))
    (cl:cons ':obstacle_cost (obstacle_cost msg))
    (cl:cons ':segmentation_cost (segmentation_cost msg))
    (cl:cons ':heading_cost (heading_cost msg))
    (cl:cons ':speed_cost (speed_cost msg))
    (cl:cons ':deviation_cost (deviation_cost msg))
))
