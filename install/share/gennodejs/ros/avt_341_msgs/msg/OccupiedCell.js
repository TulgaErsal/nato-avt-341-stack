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

class OccupiedCell {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x_index = null;
      this.y_index = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('x_index')) {
        this.x_index = initObj.x_index
      }
      else {
        this.x_index = 0;
      }
      if (initObj.hasOwnProperty('y_index')) {
        this.y_index = initObj.y_index
      }
      else {
        this.y_index = 0;
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OccupiedCell
    // Serialize message field [x_index]
    bufferOffset = _serializer.uint32(obj.x_index, buffer, bufferOffset);
    // Serialize message field [y_index]
    bufferOffset = _serializer.uint32(obj.y_index, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _serializer.int8(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OccupiedCell
    let len;
    let data = new OccupiedCell(null);
    // Deserialize message field [x_index]
    data.x_index = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [y_index]
    data.y_index = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'avt_341_msgs/OccupiedCell';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3364ced6c6173e4d6f7f283e1479eee9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 x_index
    uint32 y_index
    int8 data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OccupiedCell(null);
    if (msg.x_index !== undefined) {
      resolved.x_index = msg.x_index;
    }
    else {
      resolved.x_index = 0
    }

    if (msg.y_index !== undefined) {
      resolved.y_index = msg.y_index;
    }
    else {
      resolved.y_index = 0
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = 0
    }

    return resolved;
    }
};

module.exports = OccupiedCell;
