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

class FollowerStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.leader_name = null;
      this.x_offset = null;
      this.y_offset = null;
      this.use_leader = null;
    }
    else {
      if (initObj.hasOwnProperty('leader_name')) {
        this.leader_name = initObj.leader_name
      }
      else {
        this.leader_name = '';
      }
      if (initObj.hasOwnProperty('x_offset')) {
        this.x_offset = initObj.x_offset
      }
      else {
        this.x_offset = 0.0;
      }
      if (initObj.hasOwnProperty('y_offset')) {
        this.y_offset = initObj.y_offset
      }
      else {
        this.y_offset = 0.0;
      }
      if (initObj.hasOwnProperty('use_leader')) {
        this.use_leader = initObj.use_leader
      }
      else {
        this.use_leader = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FollowerStatus
    // Serialize message field [leader_name]
    bufferOffset = _serializer.string(obj.leader_name, buffer, bufferOffset);
    // Serialize message field [x_offset]
    bufferOffset = _serializer.float64(obj.x_offset, buffer, bufferOffset);
    // Serialize message field [y_offset]
    bufferOffset = _serializer.float64(obj.y_offset, buffer, bufferOffset);
    // Serialize message field [use_leader]
    bufferOffset = _serializer.bool(obj.use_leader, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FollowerStatus
    let len;
    let data = new FollowerStatus(null);
    // Deserialize message field [leader_name]
    data.leader_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [x_offset]
    data.x_offset = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_offset]
    data.y_offset = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [use_leader]
    data.use_leader = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.leader_name);
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'avt_341_msgs/FollowerStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '61986abde49829e549c712689cef646e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string leader_name
    float64 x_offset
    float64 y_offset
    bool use_leader
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FollowerStatus(null);
    if (msg.leader_name !== undefined) {
      resolved.leader_name = msg.leader_name;
    }
    else {
      resolved.leader_name = ''
    }

    if (msg.x_offset !== undefined) {
      resolved.x_offset = msg.x_offset;
    }
    else {
      resolved.x_offset = 0.0
    }

    if (msg.y_offset !== undefined) {
      resolved.y_offset = msg.y_offset;
    }
    else {
      resolved.y_offset = 0.0
    }

    if (msg.use_leader !== undefined) {
      resolved.use_leader = msg.use_leader;
    }
    else {
      resolved.use_leader = false
    }

    return resolved;
    }
};

module.exports = FollowerStatus;
