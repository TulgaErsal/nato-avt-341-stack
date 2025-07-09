;; Auto-generated. Do not edit!


(when (boundp 'avt_341_msgs::DwaObjective)
  (if (not (find-package "AVT_341_MSGS"))
    (make-package "AVT_341_MSGS"))
  (shadow 'DwaObjective (find-package "AVT_341_MSGS")))
(unless (find-package "AVT_341_MSGS::DWAOBJECTIVE")
  (make-package "AVT_341_MSGS::DWAOBJECTIVE"))

(in-package "ROS")
;;//! \htmlinclude DwaObjective.msg.html


(defclass avt_341_msgs::DwaObjective
  :super ros::object
  :slots (_goal_cost _obstacle_cost _segmentation_cost _heading_cost _speed_cost _deviation_cost ))

(defmethod avt_341_msgs::DwaObjective
  (:init
   (&key
    ((:goal_cost __goal_cost) 0.0)
    ((:obstacle_cost __obstacle_cost) 0.0)
    ((:segmentation_cost __segmentation_cost) 0.0)
    ((:heading_cost __heading_cost) 0.0)
    ((:speed_cost __speed_cost) 0.0)
    ((:deviation_cost __deviation_cost) 0.0)
    )
   (send-super :init)
   (setq _goal_cost (float __goal_cost))
   (setq _obstacle_cost (float __obstacle_cost))
   (setq _segmentation_cost (float __segmentation_cost))
   (setq _heading_cost (float __heading_cost))
   (setq _speed_cost (float __speed_cost))
   (setq _deviation_cost (float __deviation_cost))
   self)
  (:goal_cost
   (&optional __goal_cost)
   (if __goal_cost (setq _goal_cost __goal_cost)) _goal_cost)
  (:obstacle_cost
   (&optional __obstacle_cost)
   (if __obstacle_cost (setq _obstacle_cost __obstacle_cost)) _obstacle_cost)
  (:segmentation_cost
   (&optional __segmentation_cost)
   (if __segmentation_cost (setq _segmentation_cost __segmentation_cost)) _segmentation_cost)
  (:heading_cost
   (&optional __heading_cost)
   (if __heading_cost (setq _heading_cost __heading_cost)) _heading_cost)
  (:speed_cost
   (&optional __speed_cost)
   (if __speed_cost (setq _speed_cost __speed_cost)) _speed_cost)
  (:deviation_cost
   (&optional __deviation_cost)
   (if __deviation_cost (setq _deviation_cost __deviation_cost)) _deviation_cost)
  (:serialization-length
   ()
   (+
    ;; float64 _goal_cost
    8
    ;; float64 _obstacle_cost
    8
    ;; float64 _segmentation_cost
    8
    ;; float64 _heading_cost
    8
    ;; float64 _speed_cost
    8
    ;; float64 _deviation_cost
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _goal_cost
       (sys::poke _goal_cost (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _obstacle_cost
       (sys::poke _obstacle_cost (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _segmentation_cost
       (sys::poke _segmentation_cost (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading_cost
       (sys::poke _heading_cost (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _speed_cost
       (sys::poke _speed_cost (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _deviation_cost
       (sys::poke _deviation_cost (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _goal_cost
     (setq _goal_cost (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _obstacle_cost
     (setq _obstacle_cost (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _segmentation_cost
     (setq _segmentation_cost (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading_cost
     (setq _heading_cost (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _speed_cost
     (setq _speed_cost (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _deviation_cost
     (setq _deviation_cost (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get avt_341_msgs::DwaObjective :md5sum-) "0c23918921b89b7797bde9cc90d54688")
(setf (get avt_341_msgs::DwaObjective :datatype-) "avt_341_msgs/DwaObjective")
(setf (get avt_341_msgs::DwaObjective :definition-)
      "# This message contains the individual objective function cost terms values for
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



(provide :avt_341_msgs/DwaObjective "0c23918921b89b7797bde9cc90d54688")


