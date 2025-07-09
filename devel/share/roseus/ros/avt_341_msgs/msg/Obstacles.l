;; Auto-generated. Do not edit!


(when (boundp 'avt_341_msgs::Obstacles)
  (if (not (find-package "AVT_341_MSGS"))
    (make-package "AVT_341_MSGS"))
  (shadow 'Obstacles (find-package "AVT_341_MSGS")))
(unless (find-package "AVT_341_MSGS::OBSTACLES")
  (make-package "AVT_341_MSGS::OBSTACLES"))

(in-package "ROS")
;;//! \htmlinclude Obstacles.msg.html


(defclass avt_341_msgs::Obstacles
  :super ros::object
  :slots (_id _obstacle_size_meters _data ))

(defmethod avt_341_msgs::Obstacles
  (:init
   (&key
    ((:id __id) 0)
    ((:obstacle_size_meters __obstacle_size_meters) 0.0)
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _obstacle_size_meters (float __obstacle_size_meters))
   (setq _data __data)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:obstacle_size_meters
   (&optional __obstacle_size_meters)
   (if __obstacle_size_meters (setq _obstacle_size_meters __obstacle_size_meters)) _obstacle_size_meters)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int64 _id
    8
    ;; float64 _obstacle_size_meters
    8
    ;; float64[] _data
    (* 8    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _id) (= (length (_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_id . bv) 0) 0) s)
              (write-long (ash (elt (_id . bv) 1) -1) s))
             ((and (class _id) (= (length (_id . bv)) 1)) ;; big1
              (write-long (elt (_id . bv) 0) s)
              (write-long (if (>= _id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _id s)(write-long (if (>= _id 0) 0 #xffffffff) s)))
     ;; float64 _obstacle_size_meters
       (sys::poke _obstacle_size_meters (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _id
#+(or :alpha :irix6 :x86_64)
      (setf _id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _obstacle_size_meters
     (setq _obstacle_size_meters (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get avt_341_msgs::Obstacles :md5sum-) "4a55c04528cfeec7dafb93064c59d2dd")
(setf (get avt_341_msgs::Obstacles :datatype-) "avt_341_msgs/Obstacles")
(setf (get avt_341_msgs::Obstacles :definition-)
      "int64 id
float64 obstacle_size_meters
float64[] data

")



(provide :avt_341_msgs/Obstacles "4a55c04528cfeec7dafb93064c59d2dd")


