;; Auto-generated. Do not edit!


(when (boundp 'avt_341_msgs::LiorfSaveMap)
  (if (not (find-package "AVT_341_MSGS"))
    (make-package "AVT_341_MSGS"))
  (shadow 'LiorfSaveMap (find-package "AVT_341_MSGS")))
(unless (find-package "AVT_341_MSGS::LIORFSAVEMAP")
  (make-package "AVT_341_MSGS::LIORFSAVEMAP"))
(unless (find-package "AVT_341_MSGS::LIORFSAVEMAPREQUEST")
  (make-package "AVT_341_MSGS::LIORFSAVEMAPREQUEST"))
(unless (find-package "AVT_341_MSGS::LIORFSAVEMAPRESPONSE")
  (make-package "AVT_341_MSGS::LIORFSAVEMAPRESPONSE"))

(in-package "ROS")





(defclass avt_341_msgs::LiorfSaveMapRequest
  :super ros::object
  :slots (_resolution _destination ))

(defmethod avt_341_msgs::LiorfSaveMapRequest
  (:init
   (&key
    ((:resolution __resolution) 0.0)
    ((:destination __destination) "")
    )
   (send-super :init)
   (setq _resolution (float __resolution))
   (setq _destination (string __destination))
   self)
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:destination
   (&optional __destination)
   (if __destination (setq _destination __destination)) _destination)
  (:serialization-length
   ()
   (+
    ;; float32 _resolution
    4
    ;; string _destination
    4 (length _destination)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _resolution
       (sys::poke _resolution (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _destination
       (write-long (length _destination) s) (princ _destination s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _resolution
     (setq _resolution (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _destination
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _destination (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass avt_341_msgs::LiorfSaveMapResponse
  :super ros::object
  :slots (_success ))

(defmethod avt_341_msgs::LiorfSaveMapResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass avt_341_msgs::LiorfSaveMap
  :super ros::object
  :slots ())

(setf (get avt_341_msgs::LiorfSaveMap :md5sum-) "9b82c64d089149d300598523af304f22")
(setf (get avt_341_msgs::LiorfSaveMap :datatype-) "avt_341_msgs/LiorfSaveMap")
(setf (get avt_341_msgs::LiorfSaveMap :request) avt_341_msgs::LiorfSaveMapRequest)
(setf (get avt_341_msgs::LiorfSaveMap :response) avt_341_msgs::LiorfSaveMapResponse)

(defmethod avt_341_msgs::LiorfSaveMapRequest
  (:response () (instance avt_341_msgs::LiorfSaveMapResponse :init)))

(setf (get avt_341_msgs::LiorfSaveMapRequest :md5sum-) "9b82c64d089149d300598523af304f22")
(setf (get avt_341_msgs::LiorfSaveMapRequest :datatype-) "avt_341_msgs/LiorfSaveMapRequest")
(setf (get avt_341_msgs::LiorfSaveMapRequest :definition-)
      "float32 resolution
string destination
---
bool success

")

(setf (get avt_341_msgs::LiorfSaveMapResponse :md5sum-) "9b82c64d089149d300598523af304f22")
(setf (get avt_341_msgs::LiorfSaveMapResponse :datatype-) "avt_341_msgs/LiorfSaveMapResponse")
(setf (get avt_341_msgs::LiorfSaveMapResponse :definition-)
      "float32 resolution
string destination
---
bool success

")



(provide :avt_341_msgs/LiorfSaveMap "9b82c64d089149d300598523af304f22")


