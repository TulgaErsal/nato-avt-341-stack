// Auto-generated. Do not edit!

// (in-package avt_341_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DwaTrajectory = require('./DwaTrajectory.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DwaInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.optimal_trajectory = null;
      this.planned_trajectories = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('optimal_trajectory')) {
        this.optimal_trajectory = initObj.optimal_trajectory
      }
      else {
        this.optimal_trajectory = new DwaTrajectory();
      }
      if (initObj.hasOwnProperty('planned_trajectories')) {
        this.planned_trajectories = initObj.planned_trajectories
      }
      else {
        this.planned_trajectories = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DwaInfo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [optimal_trajectory]
    bufferOffset = DwaTrajectory.serialize(obj.optimal_trajectory, buffer, bufferOffset);
    // Serialize message field [planned_trajectories]
    // Serialize the length for message field [planned_trajectories]
    bufferOffset = _serializer.uint32(obj.planned_trajectories.length, buffer, bufferOffset);
    obj.planned_trajectories.forEach((val) => {
      bufferOffset = DwaTrajectory.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DwaInfo
    let len;
    let data = new DwaInfo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [optimal_trajectory]
    data.optimal_trajectory = DwaTrajectory.deserialize(buffer, bufferOffset);
    // Deserialize message field [planned_trajectories]
    // Deserialize array length for message field [planned_trajectories]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.planned_trajectories = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.planned_trajectories[i] = DwaTrajectory.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += DwaTrajectory.getMessageSize(object.optimal_trajectory);
    object.planned_trajectories.forEach((val) => {
      length += DwaTrajectory.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'avt_341_msgs/DwaInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bda53ade1812b34f2bb2ce8584d13baa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    # This message contains planning step recap information for the Dynamic Window
    # Approach (DWA) planner.
    
    # Stamped header for the planner completion time.
    std_msgs/Header header
    
    # The optimal trajectory for this planning step.
    DwaTrajectory optimal_trajectory
    
    # Array of planned trajectories. This field is optional and may not be populated
    # based on planner settings. When provided, this field contains all planned
    # trajectories, including the optimal one.
    DwaTrajectory[] planned_trajectories
    
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
    MSG: avt_341_msgs/DwaTrajectory
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
    const resolved = new DwaInfo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.optimal_trajectory !== undefined) {
      resolved.optimal_trajectory = DwaTrajectory.Resolve(msg.optimal_trajectory)
    }
    else {
      resolved.optimal_trajectory = new DwaTrajectory()
    }

    if (msg.planned_trajectories !== undefined) {
      resolved.planned_trajectories = new Array(msg.planned_trajectories.length);
      for (let i = 0; i < resolved.planned_trajectories.length; ++i) {
        resolved.planned_trajectories[i] = DwaTrajectory.Resolve(msg.planned_trajectories[i]);
      }
    }
    else {
      resolved.planned_trajectories = []
    }

    return resolved;
    }
};

module.exports = DwaInfo;
