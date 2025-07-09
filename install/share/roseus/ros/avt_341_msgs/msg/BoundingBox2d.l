;; Auto-generated. Do not edit!


(when (boundp 'avt_341_msgs::BoundingBox2d)
  (if (not (find-package "AVT_341_MSGS"))
    (make-package "AVT_341_MSGS"))
  (shadow 'BoundingBox2d (find-package "AVT_341_MSGS")))
(unless (find-package "AVT_341_MSGS::BOUNDINGBOX2D")
  (make-package "AVT_341_MSGS::BOUNDINGBOX2D"))

(in-package "ROS")
;;//! \htmlinclude BoundingBox2d.msg.html


(defclass avt_341_msgs::BoundingBox2d
  :super ros::object
  :slots (_x_min _x_max _y_min _y_max ))

(defmethod avt_341_msgs::BoundingBox2d
  (:init
   (&key
    ((:x_min __x_min) 0)
    ((:x_max __x_max) 0)
    ((:y_min __y_min) 0)
    ((:y_max __y_max) 0)
    )
   (send-super :init)
   (setq _x_min (round __x_min))
   (setq _x_max (round __x_max))
   (setq _y_min (round __y_min))
   (setq _y_max (round __y_max))
   self)
  (:x_min
   (&optional __x_min)
   (if __x_min (setq _x_min __x_min)) _x_min)
  (:x_max
   (&optional __x_max)
   (if __x_max (setq _x_max __x_max)) _x_max)
  (:y_min
   (&optional __y_min)
   (if __y_min (setq _y_min __y_min)) _y_min)
  (:y_max
   (&optional __y_max)
   (if __y_max (setq _y_max __y_max)) _y_max)
  (:serialization-length
   ()
   (+
    ;; int32 _x_min
    4
    ;; int32 _x_max
    4
    ;; int32 _y_min
    4
    ;; int32 _y_max
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _x_min
       (write-long _x_min s)
     ;; int32 _x_max
       (write-long _x_max s)
     ;; int32 _y_min
       (write-long _y_min s)
     ;; int32 _y_max
       (write-long _y_max s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _x_min
     (setq _x_min (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _x_max
     (setq _x_max (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y_min
     (setq _y_min (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y_max
     (setq _y_max (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get avt_341_msgs::BoundingBox2d :md5sum-) "da93b905cb7f2e0b04662214969ec2ff")
(setf (get avt_341_msgs::BoundingBox2d :datatype-) "avt_341_msgs/BoundingBox2d")
(setf (get avt_341_msgs::BoundingBox2d :definition-)
      "# This message represents a two-dimensional bounding box enclosing a region of
# the image frame where an object detector has formulated a hypothesis. The
# bounding box is aligned to image frame and may not be rotated, hence it is
# uniquely defined by its four corners.
#
# This message is not stamped, as it is meant to be part of a message including
# all detections provided by an object detector for a given image.

# Minimum x coordinate of the bounding box in pixels.
int32 x_min

# Maximum x coordinate of the bounding box in pixels.
int32 x_max

# Minimum y coordinate of the bounding box in pixels.
int32 y_min

# Maximum y coordinate of the bounding box in pixels.
int32 y_max
")



(provide :avt_341_msgs/BoundingBox2d "da93b905cb7f2e0b04662214969ec2ff")


