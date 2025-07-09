// Auto-generated. Do not edit!

// (in-package avt_341_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DwaObjective = require('./DwaObjective.js');
let nav_msgs = _finder('nav_msgs');

//-----------------------------------------------------------

class DwaTrajectory {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.path = null;
      this.objective = null;
      this.cost = null;
    }
    else {
      if (initObj.hasOwnProperty('path')) {
        this.path = initObj.path
      }
      else {
        this.path = new nav_msgs.msg.Path();
      }
      if (initObj.hasOwnProperty('objective')) {
        this.objective = initObj.objective
      }
      else {
        this.objective = new DwaObjective();
      }
      if (initObj.hasOwnProperty('cost')) {
        this.cost = initObj.cost
      }
      else {
        this.cost = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DwaTrajectory
    // Serialize message field [path]
    bufferOffset = nav_msgs.msg.Path.serialize(obj.path, buffer, bufferOffset);
    // Serialize message field [objective]
    bufferOffset = DwaObjective.serialize(obj.objective, buffer, bufferOffset);
    // Serialize message field [cost]
    bufferOffset = _serializer.float64(obj.cost, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DwaTrajectory
    let len;
    let data = new DwaTrajectory(null);
    // Deserialize message field [path]
    data.path = nav_msgs.msg.Path.deserialize(buffer, bufferOffset);
    // Deserialize message field [objective]
    data.objective = DwaObjective.deserialize(buffer, bufferOffset);
    // Deserialize message field [cost]
    data.cost = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += nav_msgs.msg.Path.getMessageSize(object.path);
    return length + 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'avt_341_msgs/DwaTrajectory';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '02949f28799d0ba3776b1635badb3cca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message contains information on a simulated trajectory generated and
    # evaluated by the Dynamic Window Approach (DWA) planner.
    
    # Predicted path for the simulated trajectory. 
    nav_msgs/Path path
    
    # The individual cost term values for the objective function across the entire
    # trajectory. The values contained in this field are already scaled by the planner
    # weights.
    DwaObjective objective
    
    # Cumulative cost for all objective function cost terms after scaling. This
    # field is equivalent to the sum of all the individual terms contained in the
    # objective function message. 
    float64 cost
    
    ================================================================================
    MSG: nav_msgs/Path
    #An array of poses that represents a Path for a robot to follow
    Header header
    geometry_msgs/PoseStamped[] poses
    
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
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: avt_341_msgs/DwaObjective
    # This message contains the individual objective function cost terms values for
    # a trajectory planned by the Dynamic Window Approach (DWA) planner. All cost
    # terms in this message are already scaled by the user-defined weight factors.
    
    # Goal progress cost term.
    float64 goal_cost
    
    # Obstacle avoidance cost term.
    float64 obstacle_cost
    
    # Segmentation grid cost term.
    float64 segmentation_cost
    
    # Heading deviation cost term.
    float64 heading_cost
    
    # Target speed deviation cost term.
    float64 speed_cost
    
    # Current trajectory deviation cost term.
    float64 deviation_cost
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DwaTrajectory(null);
    if (msg.path !== undefined) {
      resolved.path = nav_msgs.msg.Path.Resolve(msg.path)
    }
    else {
      resolved.path = new nav_msgs.msg.Path()
    }

    if (msg.objective !== undefined) {
      resolved.objective = DwaObjective.Resolve(msg.objective)
    }
    else {
      resolved.objective = new DwaObjective()
    }

    if (msg.cost !== undefined) {
      resolved.cost = msg.cost;
    }
    else {
      resolved.cost = 0.0
    }

    return resolved;
    }
};

module.exports = DwaTrajectory;
