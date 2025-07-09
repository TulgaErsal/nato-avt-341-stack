;; Auto-generated. Do not edit!


(when (boundp 'avt_341_msgs::Hypothesis)
  (if (not (find-package "AVT_341_MSGS"))
    (make-package "AVT_341_MSGS"))
  (shadow 'Hypothesis (find-package "AVT_341_MSGS")))
(unless (find-package "AVT_341_MSGS::HYPOTHESIS")
  (make-package "AVT_341_MSGS::HYPOTHESIS"))

(in-package "ROS")
;;//! \htmlinclude Hypothesis.msg.html


(defclass avt_341_msgs::Hypothesis
  :super ros::object
  :slots (_id _label _score ))

(defmethod avt_341_msgs::Hypothesis
  (:init
   (&key
    ((:id __id) 0)
    ((:label __label) "")
    ((:score __score) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _label (string __label))
   (setq _score (float __score))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:serialization-length
   ()
   (+
    ;; int16 _id
    2
    ;; string _label
    4 (length _label)
    ;; float64 _score
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _id
       (write-word _id s)
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; float64 _score
       (sys::poke _score (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _id
     (setq _id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _score
     (setq _score (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get avt_341_msgs::Hypothesis :md5sum-) "881cf7be7c3682970b3b9514be4103fc")
(setf (get avt_341_msgs::Hypothesis :datatype-) "avt_341_msgs/Hypothesis")
(setf (get avt_341_msgs::Hypothesis :definition-)
      "# This message represents an object hypothesis formulated by an object detector
# for a given region of the image.

# The unique identifier of the object class as specified by the object detector.
int16 id

# The human-readable label of the object class as specified by the object
# detector.
string label

# The score associated with the hypothesis, as a floating point number between
# 0.0 and 1.0, including the limits.
float64 score
")



(provide :avt_341_msgs/Hypothesis "881cf7be7c3682970b3b9514be4103fc")


