// Auto-generated. Do not edit!

// (in-package avt_341_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DwaObjective {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.goal_cost = null;
      this.obstacle_cost = null;
      this.segmentation_cost = null;
      this.heading_cost = null;
      this.speed_cost = null;
      this.deviation_cost = null;
    }
    else {
      if (initObj.hasOwnProperty('goal_cost')) {
        this.goal_cost = initObj.goal_cost
      }
      else {
        this.goal_cost = 0.0;
      }
      if (initObj.hasOwnProperty('obstacle_cost')) {
        this.obstacle_cost = initObj.obstacle_cost
      }
      else {
        this.obstacle_cost = 0.0;
      }
      if (initObj.hasOwnProperty('segmentation_cost')) {
        this.segmentation_cost = initObj.segmentation_cost
      }
      else {
        this.segmentation_cost = 0.0;
      }
      if (initObj.hasOwnProperty('heading_cost')) {
        this.heading_cost = initObj.heading_cost
      }
      else {
        this.heading_cost = 0.0;
      }
      if (initObj.hasOwnProperty('speed_cost')) {
        this.speed_cost = initObj.speed_cost
      }
      else {
        this.speed_cost = 0.0;
      }
      if (initObj.hasOwnProperty('deviation_cost')) {
        this.deviation_cost = initObj.deviation_cost
      }
      else {
        this.deviation_cost = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DwaObjective
    // Serialize message field [goal_cost]
    bufferOffset = _serializer.float64(obj.goal_cost, buffer, bufferOffset);
    // Serialize message field [obstacle_cost]
    bufferOffset = _serializer.float64(obj.obstacle_cost, buffer, bufferOffset);
    // Serialize message field [segmentation_cost]
    bufferOffset = _serializer.float64(obj.segmentation_cost, buffer, bufferOffset);
    // Serialize message field [heading_cost]
    bufferOffset = _serializer.float64(obj.heading_cost, buffer, bufferOffset);
    // Serialize message field [speed_cost]
    bufferOffset = _serializer.float64(obj.speed_cost, buffer, bufferOffset);
    // Serialize message field [deviation_cost]
    bufferOffset = _serializer.float64(obj.deviation_cost, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DwaObjective
    let len;
    let data = new DwaObjective(null);
    // Deserialize message field [goal_cost]
    data.goal_cost = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [obstacle_cost]
    data.obstacle_cost = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [segmentation_cost]
    data.segmentation_cost = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading_cost]
    data.heading_cost = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [speed_cost]
    data.speed_cost = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [deviation_cost]
    data.deviation_cost = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'avt_341_msgs/DwaObjective';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0c23918921b89b7797bde9cc90d54688';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new DwaObjective(null);
    if (msg.goal_cost !== undefined) {
      resolved.goal_cost = msg.goal_cost;
    }
    else {
      resolved.goal_cost = 0.0
    }

    if (msg.obstacle_cost !== undefined) {
      resolved.obstacle_cost = msg.obstacle_cost;
    }
    else {
      resolved.obstacle_cost = 0.0
    }

    if (msg.segmentation_cost !== undefined) {
      resolved.segmentation_cost = msg.segmentation_cost;
    }
    else {
      resolved.segmentation_cost = 0.0
    }

    if (msg.heading_cost !== undefined) {
      resolved.heading_cost = msg.heading_cost;
    }
    else {
      resolved.heading_cost = 0.0
    }

    if (msg.speed_cost !== undefined) {
      resolved.speed_cost = msg.speed_cost;
    }
    else {
      resolved.speed_cost = 0.0
    }

    if (msg.deviation_cost !== undefined) {
      resolved.deviation_cost = msg.deviation_cost;
    }
    else {
      resolved.deviation_cost = 0.0
    }

    return resolved;
    }
};

module.exports = DwaObjective;
