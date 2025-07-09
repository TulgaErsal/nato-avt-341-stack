; Auto-generated. Do not edit!


(cl:in-package avt_341_msgs-msg)


;//! \htmlinclude LiorfCloudInfo.msg.html

(cl:defclass <LiorfCloudInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (start_ring_index
    :reader start_ring_index
    :initarg :start_ring_index
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (end_ring_index
    :reader end_ring_index
    :initarg :end_ring_index
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (point_col_ind
    :reader point_col_ind
    :initarg :point_col_ind
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (point_range
    :reader point_range
    :initarg :point_range
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (imu_available
    :reader imu_available
    :initarg :imu_available
    :type cl:integer
    :initform 0)
   (odom_available
    :reader odom_available
    :initarg :odom_available
    :type cl:integer
    :initform 0)
   (imu_roll_init
    :reader imu_roll_init
    :initarg :imu_roll_init
    :type cl:float
    :initform 0.0)
   (imu_pitch_init
    :reader imu_pitch_init
    :initarg :imu_pitch_init
    :type cl:float
    :initform 0.0)
   (imu_yaw_init
    :reader imu_yaw_init
    :initarg :imu_yaw_init
    :type cl:float
    :initform 0.0)
   (initial_guess_x
    :reader initial_guess_x
    :initarg :initial_guess_x
    :type cl:float
    :initform 0.0)
   (initial_guess_y
    :reader initial_guess_y
    :initarg :initial_guess_y
    :type cl:float
    :initform 0.0)
   (initial_guess_z
    :reader initial_guess_z
    :initarg :initial_guess_z
    :type cl:float
    :initform 0.0)
   (initial_guess_roll
    :reader initial_guess_roll
    :initarg :initial_guess_roll
    :type cl:float
    :initform 0.0)
   (initial_guess_pitch
    :reader initial_guess_pitch
    :initarg :initial_guess_pitch
    :type cl:float
    :initform 0.0)
   (initial_guess_yaw
    :reader initial_guess_yaw
    :initarg :initial_guess_yaw
    :type cl:float
    :initform 0.0)
   (cloud_deskewed
    :reader cloud_deskewed
    :initarg :cloud_deskewed
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (cloud_corner
    :reader cloud_corner
    :initarg :cloud_corner
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (cloud_surface
    :reader cloud_surface
    :initarg :cloud_surface
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (key_frame_cloud
    :reader key_frame_cloud
    :initarg :key_frame_cloud
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (key_frame_color
    :reader key_frame_color
    :initarg :key_frame_color
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (key_frame_poses
    :reader key_frame_poses
    :initarg :key_frame_poses
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (key_frame_map
    :reader key_frame_map
    :initarg :key_frame_map
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass LiorfCloudInfo (<LiorfCloudInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LiorfCloudInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LiorfCloudInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name avt_341_msgs-msg:<LiorfCloudInfo> is deprecated: use avt_341_msgs-msg:LiorfCloudInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:header-val is deprecated.  Use avt_341_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'start_ring_index-val :lambda-list '(m))
(cl:defmethod start_ring_index-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:start_ring_index-val is deprecated.  Use avt_341_msgs-msg:start_ring_index instead.")
  (start_ring_index m))

(cl:ensure-generic-function 'end_ring_index-val :lambda-list '(m))
(cl:defmethod end_ring_index-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:end_ring_index-val is deprecated.  Use avt_341_msgs-msg:end_ring_index instead.")
  (end_ring_index m))

(cl:ensure-generic-function 'point_col_ind-val :lambda-list '(m))
(cl:defmethod point_col_ind-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:point_col_ind-val is deprecated.  Use avt_341_msgs-msg:point_col_ind instead.")
  (point_col_ind m))

(cl:ensure-generic-function 'point_range-val :lambda-list '(m))
(cl:defmethod point_range-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:point_range-val is deprecated.  Use avt_341_msgs-msg:point_range instead.")
  (point_range m))

(cl:ensure-generic-function 'imu_available-val :lambda-list '(m))
(cl:defmethod imu_available-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:imu_available-val is deprecated.  Use avt_341_msgs-msg:imu_available instead.")
  (imu_available m))

(cl:ensure-generic-function 'odom_available-val :lambda-list '(m))
(cl:defmethod odom_available-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:odom_available-val is deprecated.  Use avt_341_msgs-msg:odom_available instead.")
  (odom_available m))

(cl:ensure-generic-function 'imu_roll_init-val :lambda-list '(m))
(cl:defmethod imu_roll_init-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:imu_roll_init-val is deprecated.  Use avt_341_msgs-msg:imu_roll_init instead.")
  (imu_roll_init m))

(cl:ensure-generic-function 'imu_pitch_init-val :lambda-list '(m))
(cl:defmethod imu_pitch_init-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:imu_pitch_init-val is deprecated.  Use avt_341_msgs-msg:imu_pitch_init instead.")
  (imu_pitch_init m))

(cl:ensure-generic-function 'imu_yaw_init-val :lambda-list '(m))
(cl:defmethod imu_yaw_init-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:imu_yaw_init-val is deprecated.  Use avt_341_msgs-msg:imu_yaw_init instead.")
  (imu_yaw_init m))

(cl:ensure-generic-function 'initial_guess_x-val :lambda-list '(m))
(cl:defmethod initial_guess_x-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:initial_guess_x-val is deprecated.  Use avt_341_msgs-msg:initial_guess_x instead.")
  (initial_guess_x m))

(cl:ensure-generic-function 'initial_guess_y-val :lambda-list '(m))
(cl:defmethod initial_guess_y-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:initial_guess_y-val is deprecated.  Use avt_341_msgs-msg:initial_guess_y instead.")
  (initial_guess_y m))

(cl:ensure-generic-function 'initial_guess_z-val :lambda-list '(m))
(cl:defmethod initial_guess_z-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:initial_guess_z-val is deprecated.  Use avt_341_msgs-msg:initial_guess_z instead.")
  (initial_guess_z m))

(cl:ensure-generic-function 'initial_guess_roll-val :lambda-list '(m))
(cl:defmethod initial_guess_roll-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:initial_guess_roll-val is deprecated.  Use avt_341_msgs-msg:initial_guess_roll instead.")
  (initial_guess_roll m))

(cl:ensure-generic-function 'initial_guess_pitch-val :lambda-list '(m))
(cl:defmethod initial_guess_pitch-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:initial_guess_pitch-val is deprecated.  Use avt_341_msgs-msg:initial_guess_pitch instead.")
  (initial_guess_pitch m))

(cl:ensure-generic-function 'initial_guess_yaw-val :lambda-list '(m))
(cl:defmethod initial_guess_yaw-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:initial_guess_yaw-val is deprecated.  Use avt_341_msgs-msg:initial_guess_yaw instead.")
  (initial_guess_yaw m))

(cl:ensure-generic-function 'cloud_deskewed-val :lambda-list '(m))
(cl:defmethod cloud_deskewed-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:cloud_deskewed-val is deprecated.  Use avt_341_msgs-msg:cloud_deskewed instead.")
  (cloud_deskewed m))

(cl:ensure-generic-function 'cloud_corner-val :lambda-list '(m))
(cl:defmethod cloud_corner-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:cloud_corner-val is deprecated.  Use avt_341_msgs-msg:cloud_corner instead.")
  (cloud_corner m))

(cl:ensure-generic-function 'cloud_surface-val :lambda-list '(m))
(cl:defmethod cloud_surface-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:cloud_surface-val is deprecated.  Use avt_341_msgs-msg:cloud_surface instead.")
  (cloud_surface m))

(cl:ensure-generic-function 'key_frame_cloud-val :lambda-list '(m))
(cl:defmethod key_frame_cloud-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:key_frame_cloud-val is deprecated.  Use avt_341_msgs-msg:key_frame_cloud instead.")
  (key_frame_cloud m))

(cl:ensure-generic-function 'key_frame_color-val :lambda-list '(m))
(cl:defmethod key_frame_color-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:key_frame_color-val is deprecated.  Use avt_341_msgs-msg:key_frame_color instead.")
  (key_frame_color m))

(cl:ensure-generic-function 'key_frame_poses-val :lambda-list '(m))
(cl:defmethod key_frame_poses-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:key_frame_poses-val is deprecated.  Use avt_341_msgs-msg:key_frame_poses instead.")
  (key_frame_poses m))

(cl:ensure-generic-function 'key_frame_map-val :lambda-list '(m))
(cl:defmethod key_frame_map-val ((m <LiorfCloudInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader avt_341_msgs-msg:key_frame_map-val is deprecated.  Use avt_341_msgs-msg:key_frame_map instead.")
  (key_frame_map m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LiorfCloudInfo>) ostream)
  "Serializes a message object of type '<LiorfCloudInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'start_ring_index))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'start_ring_index))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'end_ring_index))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'end_ring_index))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'point_col_ind))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'point_col_ind))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'point_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'point_range))
  (cl:let* ((signed (cl:slot-value msg 'imu_available)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'odom_available)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_roll_init))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_pitch_init))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imu_yaw_init))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initial_guess_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initial_guess_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initial_guess_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initial_guess_roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initial_guess_pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initial_guess_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_deskewed) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_corner) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_surface) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'key_frame_cloud) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'key_frame_color) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'key_frame_poses) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'key_frame_map) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LiorfCloudInfo>) istream)
  "Deserializes a message object of type '<LiorfCloudInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'start_ring_index) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'start_ring_index)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'end_ring_index) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'end_ring_index)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'point_col_ind) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'point_col_ind)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'point_range) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'point_range)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'imu_available) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'odom_available) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_roll_init) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_pitch_init) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imu_yaw_init) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initial_guess_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initial_guess_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initial_guess_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initial_guess_roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initial_guess_pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initial_guess_yaw) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_deskewed) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_corner) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_surface) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'key_frame_cloud) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'key_frame_color) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'key_frame_poses) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'key_frame_map) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LiorfCloudInfo>)))
  "Returns string type for a message object of type '<LiorfCloudInfo>"
  "avt_341_msgs/LiorfCloudInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LiorfCloudInfo)))
  "Returns string type for a message object of type 'LiorfCloudInfo"
  "avt_341_msgs/LiorfCloudInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LiorfCloudInfo>)))
  "Returns md5sum for a message object of type '<LiorfCloudInfo>"
  "77d1090deb18ae73b305d35de4ac9b40")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LiorfCloudInfo)))
  "Returns md5sum for a message object of type 'LiorfCloudInfo"
  "77d1090deb18ae73b305d35de4ac9b40")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LiorfCloudInfo>)))
  "Returns full string definition for message of type '<LiorfCloudInfo>"
  (cl:format cl:nil "# Cloud Info~%std_msgs/Header header ~%~%int32[] start_ring_index~%int32[] end_ring_index~%~%int32[]  point_col_ind # point column index in range image~%float32[] point_range # point range ~%~%int64 imu_available~%int64 odom_available~%~%# Attitude for LOAM initialization~%float32 imu_roll_init~%float32 imu_pitch_init~%float32 imu_yaw_init~%~%# Initial guess from imu pre-integration~%float32 initial_guess_x~%float32 initial_guess_y~%float32 initial_guess_z~%float32 initial_guess_roll~%float32 initial_guess_pitch~%float32 initial_guess_yaw~%~%# Point cloud messages~%sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed~%sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature~%sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature~%~%# 3rd party messages~%sensor_msgs/PointCloud2 key_frame_cloud~%sensor_msgs/PointCloud2 key_frame_color~%sensor_msgs/PointCloud2 key_frame_poses~%sensor_msgs/PointCloud2 key_frame_map~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LiorfCloudInfo)))
  "Returns full string definition for message of type 'LiorfCloudInfo"
  (cl:format cl:nil "# Cloud Info~%std_msgs/Header header ~%~%int32[] start_ring_index~%int32[] end_ring_index~%~%int32[]  point_col_ind # point column index in range image~%float32[] point_range # point range ~%~%int64 imu_available~%int64 odom_available~%~%# Attitude for LOAM initialization~%float32 imu_roll_init~%float32 imu_pitch_init~%float32 imu_yaw_init~%~%# Initial guess from imu pre-integration~%float32 initial_guess_x~%float32 initial_guess_y~%float32 initial_guess_z~%float32 initial_guess_roll~%float32 initial_guess_pitch~%float32 initial_guess_yaw~%~%# Point cloud messages~%sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed~%sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature~%sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature~%~%# 3rd party messages~%sensor_msgs/PointCloud2 key_frame_cloud~%sensor_msgs/PointCloud2 key_frame_color~%sensor_msgs/PointCloud2 key_frame_poses~%sensor_msgs/PointCloud2 key_frame_map~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LiorfCloudInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'start_ring_index) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'end_ring_index) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'point_col_ind) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'point_range) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     8
     8
     4
     4
     4
     4
     4
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_deskewed))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_corner))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_surface))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'key_frame_cloud))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'key_frame_color))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'key_frame_poses))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'key_frame_map))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LiorfCloudInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'LiorfCloudInfo
    (cl:cons ':header (header msg))
    (cl:cons ':start_ring_index (start_ring_index msg))
    (cl:cons ':end_ring_index (end_ring_index msg))
    (cl:cons ':point_col_ind (point_col_ind msg))
    (cl:cons ':point_range (point_range msg))
    (cl:cons ':imu_available (imu_available msg))
    (cl:cons ':odom_available (odom_available msg))
    (cl:cons ':imu_roll_init (imu_roll_init msg))
    (cl:cons ':imu_pitch_init (imu_pitch_init msg))
    (cl:cons ':imu_yaw_init (imu_yaw_init msg))
    (cl:cons ':initial_guess_x (initial_guess_x msg))
    (cl:cons ':initial_guess_y (initial_guess_y msg))
    (cl:cons ':initial_guess_z (initial_guess_z msg))
    (cl:cons ':initial_guess_roll (initial_guess_roll msg))
    (cl:cons ':initial_guess_pitch (initial_guess_pitch msg))
    (cl:cons ':initial_guess_yaw (initial_guess_yaw msg))
    (cl:cons ':cloud_deskewed (cloud_deskewed msg))
    (cl:cons ':cloud_corner (cloud_corner msg))
    (cl:cons ':cloud_surface (cloud_surface msg))
    (cl:cons ':key_frame_cloud (key_frame_cloud msg))
    (cl:cons ':key_frame_color (key_frame_color msg))
    (cl:cons ':key_frame_poses (key_frame_poses msg))
    (cl:cons ':key_frame_map (key_frame_map msg))
))
