; Auto-generated. Do not edit!


(cl:in-package avt_341_msgs-srv)


;//! \htmlinclude LiorfSaveMap-request.msg.html

(cl:defclass <LiorfSaveMap-request> (roslisp-msg-protocol:ros-message)
  ((resolution
    :reader resolution
    :initarg :resolution
    :type cl:float
    :initform 0.0)
   (destination
    :reader destination
    :initarg :destination
    :type cl:string
    :initform ""))
)

(cl:defclass LiorfSaveMap-request (<LiorfSaveMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LiorfSaveMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LiorfSaveMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avt_341_msgs-srv:<LiorfSaveMap-request> is deprecated: use avt_341_msgs-srv:LiorfSaveMap-request instead.")))

(cl:ensure-generic-function 'resolution-val :lambda-list '(m))
(cl:defmethod resolution-val ((m <LiorfSaveMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-srv:resolution-val is deprecated.  Use avt_341_msgs-srv:resolution instead.")
  (resolution m))

(cl:ensure-generic-function 'destination-val :lambda-list '(m))
(cl:defmethod destination-val ((m <LiorfSaveMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-srv:destination-val is deprecated.  Use avt_341_msgs-srv:destination instead.")
  (destination m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LiorfSaveMap-request>) ostream)
  "Serializes a message object of type '<LiorfSaveMap-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'resolution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'destination))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'destination))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LiorfSaveMap-request>) istream)
  "Deserializes a message object of type '<LiorfSaveMap-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'resolution) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'destination) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'destination) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LiorfSaveMap-request>)))
  "Returns string type for a service object of type '<LiorfSaveMap-request>"
  "avt_341_msgs/LiorfSaveMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LiorfSaveMap-request)))
  "Returns string type for a service object of type 'LiorfSaveMap-request"
  "avt_341_msgs/LiorfSaveMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LiorfSaveMap-request>)))
  "Returns md5sum for a message object of type '<LiorfSaveMap-request>"
  "9b82c64d089149d300598523af304f22")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LiorfSaveMap-request)))
  "Returns md5sum for a message object of type 'LiorfSaveMap-request"
  "9b82c64d089149d300598523af304f22")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LiorfSaveMap-request>)))
  "Returns full string definition for message of type '<LiorfSaveMap-request>"
  (cl:format cl:nil "float32 resolution~%string destination~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LiorfSaveMap-request)))
  "Returns full string definition for message of type 'LiorfSaveMap-request"
  (cl:format cl:nil "float32 resolution~%string destination~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LiorfSaveMap-request>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'destination))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LiorfSaveMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LiorfSaveMap-request
    (cl:cons ':resolution (resolution msg))
    (cl:cons ':destination (destination msg))
))
;//! \htmlinclude LiorfSaveMap-response.msg.html

(cl:defclass <LiorfSaveMap-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LiorfSaveMap-response (<LiorfSaveMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LiorfSaveMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LiorfSaveMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avt_341_msgs-srv:<LiorfSaveMap-response> is deprecated: use avt_341_msgs-srv:LiorfSaveMap-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <LiorfSaveMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-srv:success-val is deprecated.  Use avt_341_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LiorfSaveMap-response>) ostream)
  "Serializes a message object of type '<LiorfSaveMap-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LiorfSaveMap-response>) istream)
  "Deserializes a message object of type '<LiorfSaveMap-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LiorfSaveMap-response>)))
  "Returns string type for a service object of type '<LiorfSaveMap-response>"
  "avt_341_msgs/LiorfSaveMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LiorfSaveMap-response)))
  "Returns string type for a service object of type 'LiorfSaveMap-response"
  "avt_341_msgs/LiorfSaveMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LiorfSaveMap-response>)))
  "Returns md5sum for a message object of type '<LiorfSaveMap-response>"
  "9b82c64d089149d300598523af304f22")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LiorfSaveMap-response)))
  "Returns md5sum for a message object of type 'LiorfSaveMap-response"
  "9b82c64d089149d300598523af304f22")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LiorfSaveMap-response>)))
  "Returns full string definition for message of type '<LiorfSaveMap-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LiorfSaveMap-response)))
  "Returns full string definition for message of type 'LiorfSaveMap-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LiorfSaveMap-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LiorfSaveMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LiorfSaveMap-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LiorfSaveMap)))
  'LiorfSaveMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LiorfSaveMap)))
  'LiorfSaveMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LiorfSaveMap)))
  "Returns string type for a service object of type '<LiorfSaveMap>"
  "avt_341_msgs/LiorfSaveMap")