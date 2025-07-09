; Auto-generated. Do not edit!


(cl:in-package avt_341_msgs-msg)


;//! \htmlinclude DwaTrajectory.msg.html

(cl:defclass <DwaTrajectory> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path))
   (objective
    :reader objective
    :initarg :objective
    :type avt_341_msgs-msg:DwaObjective
    :initform (cl:make-instance 'avt_341_msgs-msg:DwaObjective))
   (cost
    :reader cost
    :initarg :cost
    :type cl:float
    :initform 0.0))
)

(cl:defclass DwaTrajectory (<DwaTrajectory>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DwaTrajectory>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DwaTrajectory)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avt_341_msgs-msg:<DwaTrajectory> is deprecated: use avt_341_msgs-msg:DwaTrajectory instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <DwaTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:path-val is deprecated.  Use avt_341_msgs-msg:path instead.")
  (path m))

(cl:ensure-generic-function 'objective-val :lambda-list '(m))
(cl:defmethod objective-val ((m <DwaTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:objective-val is deprecated.  Use avt_341_msgs-msg:objective instead.")
  (objective m))

(cl:ensure-generic-function 'cost-val :lambda-list '(m))
(cl:defmethod cost-val ((m <DwaTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:cost-val is deprecated.  Use avt_341_msgs-msg:cost instead.")
  (cost m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DwaTrajectory>) ostream)
  "Serializes a message object of type '<DwaTrajectory>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'objective) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'cost))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DwaTrajectory>) istream)
  "Deserializes a message object of type '<DwaTrajectory>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'objective) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cost) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DwaTrajectory>)))
  "Returns string type for a message object of type '<DwaTrajectory>"
  "avt_341_msgs/DwaTrajectory")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DwaTrajectory)))
  "Returns string type for a message object of type 'DwaTrajectory"
  "avt_341_msgs/DwaTrajectory")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DwaTrajectory>)))
  "Returns md5sum for a message object of type '<DwaTrajectory>"
  "02949f28799d0ba3776b1635badb3cca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DwaTrajectory)))
  "Returns md5sum for a message object of type 'DwaTrajectory"
  "02949f28799d0ba3776b1635badb3cca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DwaTrajectory>)))
  "Returns full string definition for message of type '<DwaTrajectory>"
  (cl:format cl:nil "# This message contains information on a simulated trajectory generated and~%# evaluated by the Dynamic Window Approach (DWA) planner.~%~%# Predicted path for the simulated trajectory. ~%nav_msgs/Path path~%~%# The individual cost term values for the objective function across the entire~%# trajectory. The values contained in this field are already scaled by the planner~%# weights.~%DwaObjective objective~%~%# Cumulative cost for all objective function cost terms after scaling. This~%# field is equivalent to the sum of all the individual terms contained in the~%# objective function message. ~%float64 cost~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: avt_341_msgs/DwaObjective~%# This message contains the individual objective function cost terms values for~%# a trajectory planned by the Dynamic Window Approach (DWA) planner. All cost~%# terms in this message are already scaled by the user-defined weight factors.~%~%# Goal progress cost term.~%float64 goal_cost~%~%# Obstacle avoidance cost term.~%float64 obstacle_cost~%~%# Segmentation grid cost term.~%float64 segmentation_cost~%~%# Heading deviation cost term.~%float64 heading_cost~%~%# Target speed deviation cost term.~%float64 speed_cost~%~%# Current trajectory deviation cost term.~%float64 deviation_cost~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DwaTrajectory)))
  "Returns full string definition for message of type 'DwaTrajectory"
  (cl:format cl:nil "# This message contains information on a simulated trajectory generated and~%# evaluated by the Dynamic Window Approach (DWA) planner.~%~%# Predicted path for the simulated trajectory. ~%nav_msgs/Path path~%~%# The individual cost term values for the objective function across the entire~%# trajectory. The values contained in this field are already scaled by the planner~%# weights.~%DwaObjective objective~%~%# Cumulative cost for all objective function cost terms after scaling. This~%# field is equivalent to the sum of all the individual terms contained in the~%# objective function message. ~%float64 cost~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: avt_341_msgs/DwaObjective~%# This message contains the individual objective function cost terms values for~%# a trajectory planned by the Dynamic Window Approach (DWA) planner. All cost~%# terms in this message are already scaled by the user-defined weight factors.~%~%# Goal progress cost term.~%float64 goal_cost~%~%# Obstacle avoidance cost term.~%float64 obstacle_cost~%~%# Segmentation grid cost term.~%float64 segmentation_cost~%~%# Heading deviation cost term.~%float64 heading_cost~%~%# Target speed deviation cost term.~%float64 speed_cost~%~%# Current trajectory deviation cost term.~%float64 deviation_cost~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DwaTrajectory>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'objective))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DwaTrajectory>))
  "Converts a ROS message object to a list"
  (cl:list 'DwaTrajectory
    (cl:cons ':path (path msg))
    (cl:cons ':objective (objective msg))
    (cl:cons ':cost (cost msg))
))
