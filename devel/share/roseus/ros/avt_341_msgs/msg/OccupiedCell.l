;; Auto-generated. Do not edit!


(when (boundp 'avt_341_msgs::OccupiedCell)
  (if (not (find-package "AVT_341_MSGS"))
    (make-package "AVT_341_MSGS"))
  (shadow 'OccupiedCell (find-package "AVT_341_MSGS")))
(unless (find-package "AVT_341_MSGS::OCCUPIEDCELL")
  (make-package "AVT_341_MSGS::OCCUPIEDCELL"))

(in-package "ROS")
;;//! \htmlinclude OccupiedCell.msg.html


(defclass avt_341_msgs::OccupiedCell
  :super ros::object
  :slots (_x_index _y_index _data ))

(defmethod avt_341_msgs::OccupiedCell
  (:init
   (&key
    ((:x_index __x_index) 0)
    ((:y_index __y_index) 0)
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _x_index (round __x_index))
   (setq _y_index (round __y_index))
   (setq _data (round __data))
   self)
  (:x_index
   (&optional __x_index)
   (if __x_index (setq _x_index __x_index)) _x_index)
  (:y_index
   (&optional __y_index)
   (if __y_index (setq _y_index __y_index)) _y_index)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; uint32 _x_index
    4
    ;; uint32 _y_index
    4
    ;; int8 _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _x_index
       (write-long _x_index s)
     ;; uint32 _y_index
       (write-long _y_index s)
     ;; int8 _data
       (write-byte _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _x_index
     (setq _x_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _y_index
     (setq _y_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int8 _data
     (setq _data (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _data 127) (setq _data (- _data 256)))
   ;;
   self)
  )

(setf (get avt_341_msgs::OccupiedCell :md5sum-) "3364ced6c6173e4d6f7f283e1479eee9")
(setf (get avt_341_msgs::OccupiedCell :datatype-) "avt_341_msgs/OccupiedCell")
(setf (get avt_341_msgs::OccupiedCell :definition-)
      "uint32 x_index
uint32 y_index
int8 data

")



(provide :avt_341_msgs/OccupiedCell "3364ced6c6173e4d6f7f283e1479eee9")


