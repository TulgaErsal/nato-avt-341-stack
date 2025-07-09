; Auto-generated. Do not edit!


(cl:in-package avt_341_msgs-msg)


;//! \htmlinclude DwaInfo.msg.html

(cl:defclass <DwaInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (optimal_trajectory
    :reader optimal_trajectory
    :initarg :optimal_trajectory
    :type avt_341_msgs-msg:DwaTrajectory
    :initform (cl:make-instance 'avt_341_msgs-msg:DwaTrajectory))
   (planned_trajectories
    :reader planned_trajectories
    :initarg :planned_trajectories
    :type (cl:vector avt_341_msgs-msg:DwaTrajectory)
   :initform (cl:make-array 0 :element-type 'avt_341_msgs-msg:DwaTrajectory :initial-element (cl:make-instance 'avt_341_msgs-msg:DwaTrajectory))))
)

(cl:defclass DwaInfo (<DwaInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DwaInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DwaInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avt_341_msgs-msg:<DwaInfo> is deprecated: use avt_341_msgs-msg:DwaInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DwaInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:header-val is deprecated.  Use avt_341_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'optimal_trajectory-val :lambda-list '(m))
(cl:defmethod optimal_trajectory-val ((m <DwaInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:optimal_trajectory-val is deprecated.  Use avt_341_msgs-msg:optimal_trajectory instead.")
  (optimal_trajectory m))

(cl:ensure-generic-function 'planned_trajectories-val :lambda-list '(m))
(cl:defmethod planned_trajectories-val ((m <DwaInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:planned_trajectories-val is deprecated.  Use avt_341_msgs-msg:planned_trajectories instead.")
  (planned_trajectories m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DwaInfo>) ostream)
  "Serializes a message object of type '<DwaInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'optimal_trajectory) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'planned_trajectories))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'planned_trajectories))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DwaInfo>) istream)
  "Deserializes a message object of type '<DwaInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'optimal_trajectory) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'planned_trajectories) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'planned_trajectories)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'avt_341_msgs-msg:DwaTrajectory))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DwaInfo>)))
  "Returns string type for a message object of type '<DwaInfo>"
  "avt_341_msgs/DwaInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DwaInfo)))
  "Returns string type for a message object of type 'DwaInfo"
  "avt_341_msgs/DwaInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DwaInfo>)))
  "Returns md5sum for a message object of type '<DwaInfo>"
  "bda53ade1812b34f2bb2ce8584d13baa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DwaInfo)))
  "Returns md5sum for a message object of type 'DwaInfo"
  "bda53ade1812b34f2bb2ce8584d13baa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DwaInfo>)))
  "Returns full string definition for message of type '<DwaInfo>"
  (cl:format cl:nil "~%# This message contains planning step recap information for the Dynamic Window~%# Approach (DWA) planner.~%~%# Stamped header for the planner completion time.~%std_msgs/Header header~%~%# The optimal trajectory for this planning step.~%DwaTrajectory optimal_trajectory~%~%# Array of planned trajectories. This field is optional and may not be populated~%# based on planner settings. When provided, this field contains all planned~%# trajectories, including the optimal one.~%DwaTrajectory[] planned_trajectories~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: avt_341_msgs/DwaTrajectory~%# This message contains information on a simulated trajectory generated and~%# evaluated by the Dynamic Window Approach (DWA) planner.~%~%# Predicted path for the simulated trajectory. ~%nav_msgs/Path path~%~%# The individual cost term values for the objective function across the entire~%# trajectory. The values contained in this field are already scaled by the planner~%# weights.~%DwaObjective objective~%~%# Cumulative cost for all objective function cost terms after scaling. This~%# field is equivalent to the sum of all the individual terms contained in the~%# objective function message. ~%float64 cost~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: avt_341_msgs/DwaObjective~%# This message contains the individual objective function cost terms values for~%# a trajectory planned by the Dynamic Window Approach (DWA) planner. All cost~%# terms in this message are already scaled by the user-defined weight factors.~%~%# Goal progress cost term.~%float64 goal_cost~%~%# Obstacle avoidance cost term.~%float64 obstacle_cost~%~%# Segmentation grid cost term.~%float64 segmentation_cost~%~%# Heading deviation cost term.~%float64 heading_cost~%~%# Target speed deviation cost term.~%float64 speed_cost~%~%# Current trajectory deviation cost term.~%float64 deviation_cost~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DwaInfo)))
  "Returns full string definition for message of type 'DwaInfo"
  (cl:format cl:nil "~%# This message contains planning step recap information for the Dynamic Window~%# Approach (DWA) planner.~%~%# Stamped header for the planner completion time.~%std_msgs/Header header~%~%# The optimal trajectory for this planning step.~%DwaTrajectory optimal_trajectory~%~%# Array of planned trajectories. This field is optional and may not be populated~%# based on planner settings. When provided, this field contains all planned~%# trajectories, including the optimal one.~%DwaTrajectory[] planned_trajectories~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: avt_341_msgs/DwaTrajectory~%# This message contains information on a simulated trajectory generated and~%# evaluated by the Dynamic Window Approach (DWA) planner.~%~%# Predicted path for the simulated trajectory. ~%nav_msgs/Path path~%~%# The individual cost term values for the objective function across the entire~%# trajectory. The values contained in this field are already scaled by the planner~%# weights.~%DwaObjective objective~%~%# Cumulative cost for all objective function cost terms after scaling. This~%# field is equivalent to the sum of all the individual terms contained in the~%# objective function message. ~%float64 cost~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: avt_341_msgs/DwaObjective~%# This message contains the individual objective function cost terms values for~%# a trajectory planned by the Dynamic Window Approach (DWA) planner. All cost~%# terms in this message are already scaled by the user-defined weight factors.~%~%# Goal progress cost term.~%float64 goal_cost~%~%# Obstacle avoidance cost term.~%float64 obstacle_cost~%~%# Segmentation grid cost term.~%float64 segmentation_cost~%~%# Heading deviation cost term.~%float64 heading_cost~%~%# Target speed deviation cost term.~%float64 speed_cost~%~%# Current trajectory deviation cost term.~%float64 deviation_cost~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DwaInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'optimal_trajectory))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'planned_trajectories) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DwaInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'DwaInfo
    (cl:cons ':header (header msg))
    (cl:cons ':optimal_trajectory (optimal_trajectory msg))
    (cl:cons ':planned_trajectories (planned_trajectories msg))
))
