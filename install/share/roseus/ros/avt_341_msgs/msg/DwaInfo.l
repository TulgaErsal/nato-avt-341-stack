;; Auto-generated. Do not edit!


(when (boundp 'avt_341_msgs::DwaInfo)
  (if (not (find-package "AVT_341_MSGS"))
    (make-package "AVT_341_MSGS"))
  (shadow 'DwaInfo (find-package "AVT_341_MSGS")))
(unless (find-package "AVT_341_MSGS::DWAINFO")
  (make-package "AVT_341_MSGS::DWAINFO"))

(in-package "ROS")
;;//! \htmlinclude DwaInfo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass avt_341_msgs::DwaInfo
  :super ros::object
  :slots (_header _optimal_trajectory _planned_trajectories ))

(defmethod avt_341_msgs::DwaInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:optimal_trajectory __optimal_trajectory) (instance avt_341_msgs::DwaTrajectory :init))
    ((:planned_trajectories __planned_trajectories) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _optimal_trajectory __optimal_trajectory)
   (setq _planned_trajectories __planned_trajectories)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:optimal_trajectory
   (&rest __optimal_trajectory)
   (if (keywordp (car __optimal_trajectory))
       (send* _optimal_trajectory __optimal_trajectory)
     (progn
       (if __optimal_trajectory (setq _optimal_trajectory (car __optimal_trajectory)))
       _optimal_trajectory)))
  (:planned_trajectories
   (&rest __planned_trajectories)
   (if (keywordp (car __planned_trajectories))
       (send* _planned_trajectories __planned_trajectories)
     (progn
       (if __planned_trajectories (setq _planned_trajectories (car __planned_trajectories)))
       _planned_trajectories)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; avt_341_msgs/DwaTrajectory _optimal_trajectory
    (send _optimal_trajectory :serialization-length)
    ;; avt_341_msgs/DwaTrajectory[] _planned_trajectories
    (apply #'+ (send-all _planned_trajectories :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; avt_341_msgs/DwaTrajectory _optimal_trajectory
       (send _optimal_trajectory :serialize s)
     ;; avt_341_msgs/DwaTrajectory[] _planned_trajectories
     (write-long (length _planned_trajectories) s)
     (dolist (elem _planned_trajectories)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; avt_341_msgs/DwaTrajectory _optimal_trajectory
     (send _optimal_trajectory :deserialize buf ptr-) (incf ptr- (send _optimal_trajectory :serialization-length))
   ;; avt_341_msgs/DwaTrajectory[] _planned_trajectories
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _planned_trajectories (let (r) (dotimes (i n) (push (instance avt_341_msgs::DwaTrajectory :init) r)) r))
     (dolist (elem- _planned_trajectories)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get avt_341_msgs::DwaInfo :md5sum-) "bda53ade1812b34f2bb2ce8584d13baa")
(setf (get avt_341_msgs::DwaInfo :datatype-) "avt_341_msgs/DwaInfo")
(setf (get avt_341_msgs::DwaInfo :definition-)
      "
# This message contains planning step recap information for the Dynamic Window
# Approach (DWA) planner.

# Stamped header for the planner completion time.
std_msgs/Header header

# The optimal trajectory for this planning step.
DwaTrajectory optimal_trajectory

# Array of planned trajectories. This field is optional and may not be populated
# based on planner settings. When provided, this field contains all planned
# trajectories, including the optimal one.
DwaTrajectory[] planned_trajectories

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
MSG: avt_341_msgs/DwaTrajectory
# This message contains information on a simulated trajectory generated and
# evaluated by the Dynamic Window Approach (DWA) planner.

# Predicted path for the simulated trajectory. 
nav_msgs/Path path

# The individual cost term values for the objective function across the entire
# trajectory. The values contained in this field are already scaled by the planner
# weights.
DwaObjective objective

# Cumulative cost for all objective function cost terms after scaling. This
# field is equivalent to the sum of all the individual terms contained in the
# objective function message. 
float64 cost

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: avt_341_msgs/DwaObjective
# This message contains the individual objective function cost terms values for
# a trajectory planned by the Dynamic Window Approach (DWA) planner. All cost
# terms in this message are already scaled by the user-defined weight factors.

# Goal progress cost term.
float64 goal_cost

# Obstacle avoidance cost term.
float64 obstacle_cost

# Segmentation grid cost term.
float64 segmentation_cost

# Heading deviation cost term.
float64 heading_cost

# Target speed deviation cost term.
float64 speed_cost

# Current trajectory deviation cost term.
float64 deviation_cost
")



(provide :avt_341_msgs/DwaInfo "bda53ade1812b34f2bb2ce8584d13baa")


