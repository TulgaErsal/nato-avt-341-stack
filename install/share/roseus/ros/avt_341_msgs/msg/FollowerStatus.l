;; Auto-generated. Do not edit!


(when (boundp 'avt_341_msgs::FollowerStatus)
  (if (not (find-package "AVT_341_MSGS"))
    (make-package "AVT_341_MSGS"))
  (shadow 'FollowerStatus (find-package "AVT_341_MSGS")))
(unless (find-package "AVT_341_MSGS::FOLLOWERSTATUS")
  (make-package "AVT_341_MSGS::FOLLOWERSTATUS"))

(in-package "ROS")
;;//! \htmlinclude FollowerStatus.msg.html


(defclass avt_341_msgs::FollowerStatus
  :super ros::object
  :slots (_leader_name _x_offset _y_offset _use_leader ))

(defmethod avt_341_msgs::FollowerStatus
  (:init
   (&key
    ((:leader_name __leader_name) "")
    ((:x_offset __x_offset) 0.0)
    ((:y_offset __y_offset) 0.0)
    ((:use_leader __use_leader) nil)
    )
   (send-super :init)
   (setq _leader_name (string __leader_name))
   (setq _x_offset (float __x_offset))
   (setq _y_offset (float __y_offset))
   (setq _use_leader __use_leader)
   self)
  (:leader_name
   (&optional __leader_name)
   (if __leader_name (setq _leader_name __leader_name)) _leader_name)
  (:x_offset
   (&optional __x_offset)
   (if __x_offset (setq _x_offset __x_offset)) _x_offset)
  (:y_offset
   (&optional __y_offset)
   (if __y_offset (setq _y_offset __y_offset)) _y_offset)
  (:use_leader
   (&optional (__use_leader :null))
   (if (not (eq __use_leader :null)) (setq _use_leader __use_leader)) _use_leader)
  (:serialization-length
   ()
   (+
    ;; string _leader_name
    4 (length _leader_name)
    ;; float64 _x_offset
    8
    ;; float64 _y_offset
    8
    ;; bool _use_leader
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _leader_name
       (write-long (length _leader_name) s) (princ _leader_name s)
     ;; float64 _x_offset
       (sys::poke _x_offset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_offset
       (sys::poke _y_offset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _use_leader
       (if _use_leader (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _leader_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _leader_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _x_offset
     (setq _x_offset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_offset
     (setq _y_offset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _use_leader
     (setq _use_leader (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get avt_341_msgs::FollowerStatus :md5sum-) "61986abde49829e549c712689cef646e")
(setf (get avt_341_msgs::FollowerStatus :datatype-) "avt_341_msgs/FollowerStatus")
(setf (get avt_341_msgs::FollowerStatus :definition-)
      "string leader_name
float64 x_offset
float64 y_offset
bool use_leader

")



(provide :avt_341_msgs/FollowerStatus "61986abde49829e549c712689cef646e")


