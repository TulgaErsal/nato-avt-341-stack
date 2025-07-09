; Auto-generated. Do not edit!


(cl:in-package avt_341_msgs-msg)


;//! \htmlinclude Hypothesis.msg.html

(cl:defclass <Hypothesis> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (label
    :reader label
    :initarg :label
    :type cl:string
    :initform "")
   (score
    :reader score
    :initarg :score
    :type cl:float
    :initform 0.0))
)

(cl:defclass Hypothesis (<Hypothesis>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hypothesis>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hypothesis)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avt_341_msgs-msg:<Hypothesis> is deprecated: use avt_341_msgs-msg:Hypothesis instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Hypothesis>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:id-val is deprecated.  Use avt_341_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'label-val :lambda-list '(m))
(cl:defmethod label-val ((m <Hypothesis>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:label-val is deprecated.  Use avt_341_msgs-msg:label instead.")
  (label m))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <Hypothesis>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:score-val is deprecated.  Use avt_341_msgs-msg:score instead.")
  (score m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hypothesis>) ostream)
  "Serializes a message object of type '<Hypothesis>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'label))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hypothesis>) istream)
  "Deserializes a message object of type '<Hypothesis>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'score) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hypothesis>)))
  "Returns string type for a message object of type '<Hypothesis>"
  "avt_341_msgs/Hypothesis")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hypothesis)))
  "Returns string type for a message object of type 'Hypothesis"
  "avt_341_msgs/Hypothesis")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hypothesis>)))
  "Returns md5sum for a message object of type '<Hypothesis>"
  "881cf7be7c3682970b3b9514be4103fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hypothesis)))
  "Returns md5sum for a message object of type 'Hypothesis"
  "881cf7be7c3682970b3b9514be4103fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hypothesis>)))
  "Returns full string definition for message of type '<Hypothesis>"
  (cl:format cl:nil "# This message represents an object hypothesis formulated by an object detector~%# for a given region of the image.~%~%# The unique identifier of the object class as specified by the object detector.~%int16 id~%~%# The human-readable label of the object class as specified by the object~%# detector.~%string label~%~%# The score associated with the hypothesis, as a floating point number between~%# 0.0 and 1.0, including the limits.~%float64 score~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hypothesis)))
  "Returns full string definition for message of type 'Hypothesis"
  (cl:format cl:nil "# This message represents an object hypothesis formulated by an object detector~%# for a given region of the image.~%~%# The unique identifier of the object class as specified by the object detector.~%int16 id~%~%# The human-readable label of the object class as specified by the object~%# detector.~%string label~%~%# The score associated with the hypothesis, as a floating point number between~%# 0.0 and 1.0, including the limits.~%float64 score~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hypothesis>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'label))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hypothesis>))
  "Converts a ROS message object to a list"
  (cl:list 'Hypothesis
    (cl:cons ':id (id msg))
    (cl:cons ':label (label msg))
    (cl:cons ':score (score msg))
))
