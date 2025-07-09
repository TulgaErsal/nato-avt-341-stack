; Auto-generated. Do not edit!


(cl:in-package avt_341_msgs-msg)


;//! \htmlinclude Sinkage.msg.html

(cl:defclass <Sinkage> (roslisp-msg-protocol:ros-message)
  ((n
    :reader n
    :initarg :n
    :type cl:float
    :initform 0.0))
)

(cl:defclass Sinkage (<Sinkage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sinkage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sinkage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avt_341_msgs-msg:<Sinkage> is deprecated: use avt_341_msgs-msg:Sinkage instead.")))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <Sinkage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:n-val is deprecated.  Use avt_341_msgs-msg:n instead.")
  (n m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sinkage>) ostream)
  "Serializes a message object of type '<Sinkage>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'n))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sinkage>) istream)
  "Deserializes a message object of type '<Sinkage>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'n) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sinkage>)))
  "Returns string type for a message object of type '<Sinkage>"
  "avt_341_msgs/Sinkage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sinkage)))
  "Returns string type for a message object of type 'Sinkage"
  "avt_341_msgs/Sinkage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sinkage>)))
  "Returns md5sum for a message object of type '<Sinkage>"
  "65f0673a55739dec984a47bd3a9c8f0d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sinkage)))
  "Returns md5sum for a message object of type 'Sinkage"
  "65f0673a55739dec984a47bd3a9c8f0d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sinkage>)))
  "Returns full string definition for message of type '<Sinkage>"
  (cl:format cl:nil "float64 n~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sinkage)))
  "Returns full string definition for message of type 'Sinkage"
  (cl:format cl:nil "float64 n~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sinkage>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sinkage>))
  "Converts a ROS message object to a list"
  (cl:list 'Sinkage
    (cl:cons ':n (n msg))
))
