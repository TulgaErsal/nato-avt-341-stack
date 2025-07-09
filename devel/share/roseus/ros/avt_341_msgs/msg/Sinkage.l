;; Auto-generated. Do not edit!


(when (boundp 'avt_341_msgs::Sinkage)
  (if (not (find-package "AVT_341_MSGS"))
    (make-package "AVT_341_MSGS"))
  (shadow 'Sinkage (find-package "AVT_341_MSGS")))
(unless (find-package "AVT_341_MSGS::SINKAGE")
  (make-package "AVT_341_MSGS::SINKAGE"))

(in-package "ROS")
;;//! \htmlinclude Sinkage.msg.html


(defclass avt_341_msgs::Sinkage
  :super ros::object
  :slots (_n ))

(defmethod avt_341_msgs::Sinkage
  (:init
   (&key
    ((:n __n) 0.0)
    )
   (send-super :init)
   (setq _n (float __n))
   self)
  (:n
   (&optional __n)
   (if __n (setq _n __n)) _n)
  (:serialization-length
   ()
   (+
    ;; float64 _n
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _n
       (sys::poke _n (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _n
     (setq _n (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get avt_341_msgs::Sinkage :md5sum-) "65f0673a55739dec984a47bd3a9c8f0d")
(setf (get avt_341_msgs::Sinkage :datatype-) "avt_341_msgs/Sinkage")
(setf (get avt_341_msgs::Sinkage :definition-)
      "float64 n

")



(provide :avt_341_msgs/Sinkage "65f0673a55739dec984a47bd3a9c8f0d")


