// Auto-generated. Do not edit!

// (in-package avt_341_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class LiorfCloudInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.start_ring_index = null;
      this.end_ring_index = null;
      this.point_col_ind = null;
      this.point_range = null;
      this.imu_available = null;
      this.odom_available = null;
      this.imu_roll_init = null;
      this.imu_pitch_init = null;
      this.imu_yaw_init = null;
      this.initial_guess_x = null;
      this.initial_guess_y = null;
      this.initial_guess_z = null;
      this.initial_guess_roll = null;
      this.initial_guess_pitch = null;
      this.initial_guess_yaw = null;
      this.cloud_deskewed = null;
      this.cloud_corner = null;
      this.cloud_surface = null;
      this.key_frame_cloud = null;
      this.key_frame_color = null;
      this.key_frame_poses = null;
      this.key_frame_map = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('start_ring_index')) {
        this.start_ring_index = initObj.start_ring_index
      }
      else {
        this.start_ring_index = [];
      }
      if (initObj.hasOwnProperty('end_ring_index')) {
        this.end_ring_index = initObj.end_ring_index
      }
      else {
        this.end_ring_index = [];
      }
      if (initObj.hasOwnProperty('point_col_ind')) {
        this.point_col_ind = initObj.point_col_ind
      }
      else {
        this.point_col_ind = [];
      }
      if (initObj.hasOwnProperty('point_range')) {
        this.point_range = initObj.point_range
      }
      else {
        this.point_range = [];
      }
      if (initObj.hasOwnProperty('imu_available')) {
        this.imu_available = initObj.imu_available
      }
      else {
        this.imu_available = 0;
      }
      if (initObj.hasOwnProperty('odom_available')) {
        this.odom_available = initObj.odom_available
      }
      else {
        this.odom_available = 0;
      }
      if (initObj.hasOwnProperty('imu_roll_init')) {
        this.imu_roll_init = initObj.imu_roll_init
      }
      else {
        this.imu_roll_init = 0.0;
      }
      if (initObj.hasOwnProperty('imu_pitch_init')) {
        this.imu_pitch_init = initObj.imu_pitch_init
      }
      else {
        this.imu_pitch_init = 0.0;
      }
      if (initObj.hasOwnProperty('imu_yaw_init')) {
        this.imu_yaw_init = initObj.imu_yaw_init
      }
      else {
        this.imu_yaw_init = 0.0;
      }
      if (initObj.hasOwnProperty('initial_guess_x')) {
        this.initial_guess_x = initObj.initial_guess_x
      }
      else {
        this.initial_guess_x = 0.0;
      }
      if (initObj.hasOwnProperty('initial_guess_y')) {
        this.initial_guess_y = initObj.initial_guess_y
      }
      else {
        this.initial_guess_y = 0.0;
      }
      if (initObj.hasOwnProperty('initial_guess_z')) {
        this.initial_guess_z = initObj.initial_guess_z
      }
      else {
        this.initial_guess_z = 0.0;
      }
      if (initObj.hasOwnProperty('initial_guess_roll')) {
        this.initial_guess_roll = initObj.initial_guess_roll
      }
      else {
        this.initial_guess_roll = 0.0;
      }
      if (initObj.hasOwnProperty('initial_guess_pitch')) {
        this.initial_guess_pitch = initObj.initial_guess_pitch
      }
      else {
        this.initial_guess_pitch = 0.0;
      }
      if (initObj.hasOwnProperty('initial_guess_yaw')) {
        this.initial_guess_yaw = initObj.initial_guess_yaw
      }
      else {
        this.initial_guess_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('cloud_deskewed')) {
        this.cloud_deskewed = initObj.cloud_deskewed
      }
      else {
        this.cloud_deskewed = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('cloud_corner')) {
        this.cloud_corner = initObj.cloud_corner
      }
      else {
        this.cloud_corner = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('cloud_surface')) {
        this.cloud_surface = initObj.cloud_surface
      }
      else {
        this.cloud_surface = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('key_frame_cloud')) {
        this.key_frame_cloud = initObj.key_frame_cloud
      }
      else {
        this.key_frame_cloud = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('key_frame_color')) {
        this.key_frame_color = initObj.key_frame_color
      }
      else {
        this.key_frame_color = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('key_frame_poses')) {
        this.key_frame_poses = initObj.key_frame_poses
      }
      else {
        this.key_frame_poses = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('key_frame_map')) {
        this.key_frame_map = initObj.key_frame_map
      }
      else {
        this.key_frame_map = new sensor_msgs.msg.PointCloud2();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LiorfCloudInfo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [start_ring_index]
    bufferOffset = _arraySerializer.int32(obj.start_ring_index, buffer, bufferOffset, null);
    // Serialize message field [end_ring_index]
    bufferOffset = _arraySerializer.int32(obj.end_ring_index, buffer, bufferOffset, null);
    // Serialize message field [point_col_ind]
    bufferOffset = _arraySerializer.int32(obj.point_col_ind, buffer, bufferOffset, null);
    // Serialize message field [point_range]
    bufferOffset = _arraySerializer.float32(obj.point_range, buffer, bufferOffset, null);
    // Serialize message field [imu_available]
    bufferOffset = _serializer.int64(obj.imu_available, buffer, bufferOffset);
    // Serialize message field [odom_available]
    bufferOffset = _serializer.int64(obj.odom_available, buffer, bufferOffset);
    // Serialize message field [imu_roll_init]
    bufferOffset = _serializer.float32(obj.imu_roll_init, buffer, bufferOffset);
    // Serialize message field [imu_pitch_init]
    bufferOffset = _serializer.float32(obj.imu_pitch_init, buffer, bufferOffset);
    // Serialize message field [imu_yaw_init]
    bufferOffset = _serializer.float32(obj.imu_yaw_init, buffer, bufferOffset);
    // Serialize message field [initial_guess_x]
    bufferOffset = _serializer.float32(obj.initial_guess_x, buffer, bufferOffset);
    // Serialize message field [initial_guess_y]
    bufferOffset = _serializer.float32(obj.initial_guess_y, buffer, bufferOffset);
    // Serialize message field [initial_guess_z]
    bufferOffset = _serializer.float32(obj.initial_guess_z, buffer, bufferOffset);
    // Serialize message field [initial_guess_roll]
    bufferOffset = _serializer.float32(obj.initial_guess_roll, buffer, bufferOffset);
    // Serialize message field [initial_guess_pitch]
    bufferOffset = _serializer.float32(obj.initial_guess_pitch, buffer, bufferOffset);
    // Serialize message field [initial_guess_yaw]
    bufferOffset = _serializer.float32(obj.initial_guess_yaw, buffer, bufferOffset);
    // Serialize message field [cloud_deskewed]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.cloud_deskewed, buffer, bufferOffset);
    // Serialize message field [cloud_corner]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.cloud_corner, buffer, bufferOffset);
    // Serialize message field [cloud_surface]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.cloud_surface, buffer, bufferOffset);
    // Serialize message field [key_frame_cloud]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.key_frame_cloud, buffer, bufferOffset);
    // Serialize message field [key_frame_color]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.key_frame_color, buffer, bufferOffset);
    // Serialize message field [key_frame_poses]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.key_frame_poses, buffer, bufferOffset);
    // Serialize message field [key_frame_map]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.key_frame_map, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LiorfCloudInfo
    let len;
    let data = new LiorfCloudInfo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [start_ring_index]
    data.start_ring_index = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [end_ring_index]
    data.end_ring_index = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [point_col_ind]
    data.point_col_ind = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [point_range]
    data.point_range = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [imu_available]
    data.imu_available = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [odom_available]
    data.odom_available = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [imu_roll_init]
    data.imu_roll_init = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [imu_pitch_init]
    data.imu_pitch_init = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [imu_yaw_init]
    data.imu_yaw_init = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initial_guess_x]
    data.initial_guess_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initial_guess_y]
    data.initial_guess_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initial_guess_z]
    data.initial_guess_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initial_guess_roll]
    data.initial_guess_roll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initial_guess_pitch]
    data.initial_guess_pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initial_guess_yaw]
    data.initial_guess_yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cloud_deskewed]
    data.cloud_deskewed = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [cloud_corner]
    data.cloud_corner = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [cloud_surface]
    data.cloud_surface = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [key_frame_cloud]
    data.key_frame_cloud = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [key_frame_color]
    data.key_frame_color = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [key_frame_poses]
    data.key_frame_poses = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [key_frame_map]
    data.key_frame_map = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.start_ring_index.length;
    length += 4 * object.end_ring_index.length;
    length += 4 * object.point_col_ind.length;
    length += 4 * object.point_range.length;
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.cloud_deskewed);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.cloud_corner);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.cloud_surface);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.key_frame_cloud);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.key_frame_color);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.key_frame_poses);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.key_frame_map);
    return length + 68;
  }

  static datatype() {
    // Returns string type for a message object
    return 'avt_341_msgs/LiorfCloudInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '77d1090deb18ae73b305d35de4ac9b40';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Cloud Info
    std_msgs/Header header 
    
    int32[] start_ring_index
    int32[] end_ring_index
    
    int32[]  point_col_ind # point column index in range image
    float32[] point_range # point range 
    
    int64 imu_available
    int64 odom_available
    
    # Attitude for LOAM initialization
    float32 imu_roll_init
    float32 imu_pitch_init
    float32 imu_yaw_init
    
    # Initial guess from imu pre-integration
    float32 initial_guess_x
    float32 initial_guess_y
    float32 initial_guess_z
    float32 initial_guess_roll
    float32 initial_guess_pitch
    float32 initial_guess_yaw
    
    # Point cloud messages
    sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed
    sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature
    sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature
    
    # 3rd party messages
    sensor_msgs/PointCloud2 key_frame_cloud
    sensor_msgs/PointCloud2 key_frame_color
    sensor_msgs/PointCloud2 key_frame_poses
    sensor_msgs/PointCloud2 key_frame_map
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: sensor_msgs/PointCloud2
    # This message holds a collection of N-dimensional points, which may
    # contain additional information such as normals, intensity, etc. The
    # point data is stored as a binary blob, its layout described by the
    # contents of the "fields" array.
    
    # The point cloud data may be organized 2d (image-like) or 1d
    # (unordered). Point clouds organized as 2d images may be produced by
    # camera depth sensors such as stereo or time-of-flight.
    
    # Time of sensor data acquisition, and the coordinate frame ID (for 3d
    # points).
    Header header
    
    # 2D structure of the point cloud. If the cloud is unordered, height is
    # 1 and width is the length of the point cloud.
    uint32 height
    uint32 width
    
    # Describes the channels and their layout in the binary data blob.
    PointField[] fields
    
    bool    is_bigendian # Is this data bigendian?
    uint32  point_step   # Length of a point in bytes
    uint32  row_step     # Length of a row in bytes
    uint8[] data         # Actual point data, size is (row_step*height)
    
    bool is_dense        # True if there are no invalid points
    
    ================================================================================
    MSG: sensor_msgs/PointField
    # This message holds the description of one point entry in the
    # PointCloud2 message format.
    uint8 INT8    = 1
    uint8 UINT8   = 2
    uint8 INT16   = 3
    uint8 UINT16  = 4
    uint8 INT32   = 5
    uint8 UINT32  = 6
    uint8 FLOAT32 = 7
    uint8 FLOAT64 = 8
    
    string name      # Name of field
    uint32 offset    # Offset from start of point struct
    uint8  datatype  # Datatype enumeration, see above
    uint32 count     # How many elements in the field
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LiorfCloudInfo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.start_ring_index !== undefined) {
      resolved.start_ring_index = msg.start_ring_index;
    }
    else {
      resolved.start_ring_index = []
    }

    if (msg.end_ring_index !== undefined) {
      resolved.end_ring_index = msg.end_ring_index;
    }
    else {
      resolved.end_ring_index = []
    }

    if (msg.point_col_ind !== undefined) {
      resolved.point_col_ind = msg.point_col_ind;
    }
    else {
      resolved.point_col_ind = []
    }

    if (msg.point_range !== undefined) {
      resolved.point_range = msg.point_range;
    }
    else {
      resolved.point_range = []
    }

    if (msg.imu_available !== undefined) {
      resolved.imu_available = msg.imu_available;
    }
    else {
      resolved.imu_available = 0
    }

    if (msg.odom_available !== undefined) {
      resolved.odom_available = msg.odom_available;
    }
    else {
      resolved.odom_available = 0
    }

    if (msg.imu_roll_init !== undefined) {
      resolved.imu_roll_init = msg.imu_roll_init;
    }
    else {
      resolved.imu_roll_init = 0.0
    }

    if (msg.imu_pitch_init !== undefined) {
      resolved.imu_pitch_init = msg.imu_pitch_init;
    }
    else {
      resolved.imu_pitch_init = 0.0
    }

    if (msg.imu_yaw_init !== undefined) {
      resolved.imu_yaw_init = msg.imu_yaw_init;
    }
    else {
      resolved.imu_yaw_init = 0.0
    }

    if (msg.initial_guess_x !== undefined) {
      resolved.initial_guess_x = msg.initial_guess_x;
    }
    else {
      resolved.initial_guess_x = 0.0
    }

    if (msg.initial_guess_y !== undefined) {
      resolved.initial_guess_y = msg.initial_guess_y;
    }
    else {
      resolved.initial_guess_y = 0.0
    }

    if (msg.initial_guess_z !== undefined) {
      resolved.initial_guess_z = msg.initial_guess_z;
    }
    else {
      resolved.initial_guess_z = 0.0
    }

    if (msg.initial_guess_roll !== undefined) {
      resolved.initial_guess_roll = msg.initial_guess_roll;
    }
    else {
      resolved.initial_guess_roll = 0.0
    }

    if (msg.initial_guess_pitch !== undefined) {
      resolved.initial_guess_pitch = msg.initial_guess_pitch;
    }
    else {
      resolved.initial_guess_pitch = 0.0
    }

    if (msg.initial_guess_yaw !== undefined) {
      resolved.initial_guess_yaw = msg.initial_guess_yaw;
    }
    else {
      resolved.initial_guess_yaw = 0.0
    }

    if (msg.cloud_deskewed !== undefined) {
      resolved.cloud_deskewed = sensor_msgs.msg.PointCloud2.Resolve(msg.cloud_deskewed)
    }
    else {
      resolved.cloud_deskewed = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.cloud_corner !== undefined) {
      resolved.cloud_corner = sensor_msgs.msg.PointCloud2.Resolve(msg.cloud_corner)
    }
    else {
      resolved.cloud_corner = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.cloud_surface !== undefined) {
      resolved.cloud_surface = sensor_msgs.msg.PointCloud2.Resolve(msg.cloud_surface)
    }
    else {
      resolved.cloud_surface = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.key_frame_cloud !== undefined) {
      resolved.key_frame_cloud = sensor_msgs.msg.PointCloud2.Resolve(msg.key_frame_cloud)
    }
    else {
      resolved.key_frame_cloud = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.key_frame_color !== undefined) {
      resolved.key_frame_color = sensor_msgs.msg.PointCloud2.Resolve(msg.key_frame_color)
    }
    else {
      resolved.key_frame_color = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.key_frame_poses !== undefined) {
      resolved.key_frame_poses = sensor_msgs.msg.PointCloud2.Resolve(msg.key_frame_poses)
    }
    else {
      resolved.key_frame_poses = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.key_frame_map !== undefined) {
      resolved.key_frame_map = sensor_msgs.msg.PointCloud2.Resolve(msg.key_frame_map)
    }
    else {
      resolved.key_frame_map = new sensor_msgs.msg.PointCloud2()
    }

    return resolved;
    }
};

module.exports = LiorfCloudInfo;
