;; Auto-generated. Do not edit!


(when (boundp 'avt_341_msgs::Communication)
  (if (not (find-package "AVT_341_MSGS"))
    (make-package "AVT_341_MSGS"))
  (shadow 'Communication (find-package "AVT_341_MSGS")))
(unless (find-package "AVT_341_MSGS::COMMUNICATION")
  (make-package "AVT_341_MSGS::COMMUNICATION"))

(in-package "ROS")
;;//! \htmlinclude Communication.msg.html


(defclass avt_341_msgs::Communication
  :super ros::object
  :slots (_sender_name _msg_id _type _formation _receiver_name _leader_name _follower1_name _follower2_name _follower3_name _objective_name _desired_speed _priority_type _termination_method _x_scale _y_scale _x_offset _y_offset _distance _target_msg_id ))

(defmethod avt_341_msgs::Communication
  (:init
   (&key
    ((:sender_name __sender_name) "")
    ((:msg_id __msg_id) 0)
    ((:type __type) "")
    ((:formation __formation) "")
    ((:receiver_name __receiver_name) "")
    ((:leader_name __leader_name) "")
    ((:follower1_name __follower1_name) "")
    ((:follower2_name __follower2_name) "")
    ((:follower3_name __follower3_name) "")
    ((:objective_name __objective_name) "")
    ((:desired_speed __desired_speed) 0.0)
    ((:priority_type __priority_type) "")
    ((:termination_method __termination_method) "")
    ((:x_scale __x_scale) 0.0)
    ((:y_scale __y_scale) 0.0)
    ((:x_offset __x_offset) 0.0)
    ((:y_offset __y_offset) 0.0)
    ((:distance __distance) 0.0)
    ((:target_msg_id __target_msg_id) 0)
    )
   (send-super :init)
   (setq _sender_name (string __sender_name))
   (setq _msg_id (round __msg_id))
   (setq _type (string __type))
   (setq _formation (string __formation))
   (setq _receiver_name (string __receiver_name))
   (setq _leader_name (string __leader_name))
   (setq _follower1_name (string __follower1_name))
   (setq _follower2_name (string __follower2_name))
   (setq _follower3_name (string __follower3_name))
   (setq _objective_name (string __objective_name))
   (setq _desired_speed (float __desired_speed))
   (setq _priority_type (string __priority_type))
   (setq _termination_method (string __termination_method))
   (setq _x_scale (float __x_scale))
   (setq _y_scale (float __y_scale))
   (setq _x_offset (float __x_offset))
   (setq _y_offset (float __y_offset))
   (setq _distance (float __distance))
   (setq _target_msg_id (round __target_msg_id))
   self)
  (:sender_name
   (&optional __sender_name)
   (if __sender_name (setq _sender_name __sender_name)) _sender_name)
  (:msg_id
   (&optional __msg_id)
   (if __msg_id (setq _msg_id __msg_id)) _msg_id)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:formation
   (&optional __formation)
   (if __formation (setq _formation __formation)) _formation)
  (:receiver_name
   (&optional __receiver_name)
   (if __receiver_name (setq _receiver_name __receiver_name)) _receiver_name)
  (:leader_name
   (&optional __leader_name)
   (if __leader_name (setq _leader_name __leader_name)) _leader_name)
  (:follower1_name
   (&optional __follower1_name)
   (if __follower1_name (setq _follower1_name __follower1_name)) _follower1_name)
  (:follower2_name
   (&optional __follower2_name)
   (if __follower2_name (setq _follower2_name __follower2_name)) _follower2_name)
  (:follower3_name
   (&optional __follower3_name)
   (if __follower3_name (setq _follower3_name __follower3_name)) _follower3_name)
  (:objective_name
   (&optional __objective_name)
   (if __objective_name (setq _objective_name __objective_name)) _objective_name)
  (:desired_speed
   (&optional __desired_speed)
   (if __desired_speed (setq _desired_speed __desired_speed)) _desired_speed)
  (:priority_type
   (&optional __priority_type)
   (if __priority_type (setq _priority_type __priority_type)) _priority_type)
  (:termination_method
   (&optional __termination_method)
   (if __termination_method (setq _termination_method __termination_method)) _termination_method)
  (:x_scale
   (&optional __x_scale)
   (if __x_scale (setq _x_scale __x_scale)) _x_scale)
  (:y_scale
   (&optional __y_scale)
   (if __y_scale (setq _y_scale __y_scale)) _y_scale)
  (:x_offset
   (&optional __x_offset)
   (if __x_offset (setq _x_offset __x_offset)) _x_offset)
  (:y_offset
   (&optional __y_offset)
   (if __y_offset (setq _y_offset __y_offset)) _y_offset)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:target_msg_id
   (&optional __target_msg_id)
   (if __target_msg_id (setq _target_msg_id __target_msg_id)) _target_msg_id)
  (:serialization-length
   ()
   (+
    ;; string _sender_name
    4 (length _sender_name)
    ;; int32 _msg_id
    4
    ;; string _type
    4 (length _type)
    ;; string _formation
    4 (length _formation)
    ;; string _receiver_name
    4 (length _receiver_name)
    ;; string _leader_name
    4 (length _leader_name)
    ;; string _follower1_name
    4 (length _follower1_name)
    ;; string _follower2_name
    4 (length _follower2_name)
    ;; string _follower3_name
    4 (length _follower3_name)
    ;; string _objective_name
    4 (length _objective_name)
    ;; float64 _desired_speed
    8
    ;; string _priority_type
    4 (length _priority_type)
    ;; string _termination_method
    4 (length _termination_method)
    ;; float64 _x_scale
    8
    ;; float64 _y_scale
    8
    ;; float64 _x_offset
    8
    ;; float64 _y_offset
    8
    ;; float64 _distance
    8
    ;; int32 _target_msg_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _sender_name
       (write-long (length _sender_name) s) (princ _sender_name s)
     ;; int32 _msg_id
       (write-long _msg_id s)
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; string _formation
       (write-long (length _formation) s) (princ _formation s)
     ;; string _receiver_name
       (write-long (length _receiver_name) s) (princ _receiver_name s)
     ;; string _leader_name
       (write-long (length _leader_name) s) (princ _leader_name s)
     ;; string _follower1_name
       (write-long (length _follower1_name) s) (princ _follower1_name s)
     ;; string _follower2_name
       (write-long (length _follower2_name) s) (princ _follower2_name s)
     ;; string _follower3_name
       (write-long (length _follower3_name) s) (princ _follower3_name s)
     ;; string _objective_name
       (write-long (length _objective_name) s) (princ _objective_name s)
     ;; float64 _desired_speed
       (sys::poke _desired_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _priority_type
       (write-long (length _priority_type) s) (princ _priority_type s)
     ;; string _termination_method
       (write-long (length _termination_method) s) (princ _termination_method s)
     ;; float64 _x_scale
       (sys::poke _x_scale (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_scale
       (sys::poke _y_scale (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x_offset
       (sys::poke _x_offset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_offset
       (sys::poke _y_offset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _target_msg_id
       (write-long _target_msg_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _sender_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _sender_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _msg_id
     (setq _msg_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _formation
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _formation (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _receiver_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _receiver_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _leader_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _leader_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _follower1_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _follower1_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _follower2_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _follower2_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _follower3_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _follower3_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _objective_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _objective_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _desired_speed
     (setq _desired_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _priority_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _priority_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _termination_method
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _termination_method (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _x_scale
     (setq _x_scale (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_scale
     (setq _y_scale (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x_offset
     (setq _x_offset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_offset
     (setq _y_offset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _distance
     (setq _distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _target_msg_id
     (setq _target_msg_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get avt_341_msgs::Communication :md5sum-) "b7d0b7fede5233ad75ab393a95ddc673")
(setf (get avt_341_msgs::Communication :datatype-) "avt_341_msgs/Communication")
(setf (get avt_341_msgs::Communication :definition-)
      "string sender_name
int32 msg_id
string type
string formation
string receiver_name
string leader_name
string follower1_name
string follower2_name
string follower3_name
string objective_name
float64 desired_speed
string priority_type
string termination_method
float64 x_scale
float64 y_scale
float64 x_offset
float64 y_offset
float64 distance
int32 target_msg_id
")



(provide :avt_341_msgs/Communication "b7d0b7fede5233ad75ab393a95ddc673")


