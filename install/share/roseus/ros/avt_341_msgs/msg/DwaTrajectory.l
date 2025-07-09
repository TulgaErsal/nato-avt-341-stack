;; Auto-generated. Do not edit!


(when (boundp 'avt_341_msgs::DwaTrajectory)
  (if (not (find-package "AVT_341_MSGS"))
    (make-package "AVT_341_MSGS"))
  (shadow 'DwaTrajectory (find-package "AVT_341_MSGS")))
(unless (find-package "AVT_341_MSGS::DWATRAJECTORY")
  (make-package "AVT_341_MSGS::DWATRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude DwaTrajectory.msg.html
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass avt_341_msgs::DwaTrajectory
  :super ros::object
  :slots (_path _objective _cost ))

(defmethod avt_341_msgs::DwaTrajectory
  (:init
   (&key
    ((:path __path) (instance nav_msgs::Path :init))
    ((:objective __objective) (instance avt_341_msgs::DwaObjective :init))
    ((:cost __cost) 0.0)
    )
   (send-super :init)
   (setq _path __path)
   (setq _objective __objective)
   (setq _cost (float __cost))
   self)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:objective
   (&rest __objective)
   (if (keywordp (car __objective))
       (send* _objective __objective)
     (progn
       (if __objective (setq _objective (car __objective)))
       _objective)))
  (:cost
   (&optional __cost)
   (if __cost (setq _cost __cost)) _cost)
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Path _path
    (send _path :serialization-length)
    ;; avt_341_msgs/DwaObjective _objective
    (send _objective :serialization-length)
    ;; float64 _cost
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Path _path
       (send _path :serialize s)
     ;; avt_341_msgs/DwaObjective _objective
       (send _objective :serialize s)
     ;; float64 _cost
       (sys::poke _cost (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Path _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;; avt_341_msgs/DwaObjective _objective
     (send _objective :deserialize buf ptr-) (incf ptr- (send _objective :serialization-length))
   ;; float64 _cost
     (setq _cost (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get avt_341_msgs::DwaTrajectory :md5sum-) "02949f28799d0ba3776b1635badb3cca")
(setf (get avt_341_msgs::DwaTrajectory :datatype-) "avt_341_msgs/DwaTrajectory")
(setf (get avt_341_msgs::DwaTrajectory :definition-)
      "# This message contains information on a simulated trajectory generated and
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



(provide :avt_341_msgs/DwaTrajectory "02949f28799d0ba3776b1635badb3cca")


