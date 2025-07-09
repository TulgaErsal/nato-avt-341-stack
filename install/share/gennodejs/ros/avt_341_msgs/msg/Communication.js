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

class Communication {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sender_name = null;
      this.msg_id = null;
      this.type = null;
      this.formation = null;
      this.receiver_name = null;
      this.leader_name = null;
      this.follower1_name = null;
      this.follower2_name = null;
      this.follower3_name = null;
      this.objective_name = null;
      this.desired_speed = null;
      this.priority_type = null;
      this.termination_method = null;
      this.x_scale = null;
      this.y_scale = null;
      this.x_offset = null;
      this.y_offset = null;
      this.distance = null;
      this.target_msg_id = null;
    }
    else {
      if (initObj.hasOwnProperty('sender_name')) {
        this.sender_name = initObj.sender_name
      }
      else {
        this.sender_name = '';
      }
      if (initObj.hasOwnProperty('msg_id')) {
        this.msg_id = initObj.msg_id
      }
      else {
        this.msg_id = 0;
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = '';
      }
      if (initObj.hasOwnProperty('formation')) {
        this.formation = initObj.formation
      }
      else {
        this.formation = '';
      }
      if (initObj.hasOwnProperty('receiver_name')) {
        this.receiver_name = initObj.receiver_name
      }
      else {
        this.receiver_name = '';
      }
      if (initObj.hasOwnProperty('leader_name')) {
        this.leader_name = initObj.leader_name
      }
      else {
        this.leader_name = '';
      }
      if (initObj.hasOwnProperty('follower1_name')) {
        this.follower1_name = initObj.follower1_name
      }
      else {
        this.follower1_name = '';
      }
      if (initObj.hasOwnProperty('follower2_name')) {
        this.follower2_name = initObj.follower2_name
      }
      else {
        this.follower2_name = '';
      }
      if (initObj.hasOwnProperty('follower3_name')) {
        this.follower3_name = initObj.follower3_name
      }
      else {
        this.follower3_name = '';
      }
      if (initObj.hasOwnProperty('objective_name')) {
        this.objective_name = initObj.objective_name
      }
      else {
        this.objective_name = '';
      }
      if (initObj.hasOwnProperty('desired_speed')) {
        this.desired_speed = initObj.desired_speed
      }
      else {
        this.desired_speed = 0.0;
      }
      if (initObj.hasOwnProperty('priority_type')) {
        this.priority_type = initObj.priority_type
      }
      else {
        this.priority_type = '';
      }
      if (initObj.hasOwnProperty('termination_method')) {
        this.termination_method = initObj.termination_method
      }
      else {
        this.termination_method = '';
      }
      if (initObj.hasOwnProperty('x_scale')) {
        this.x_scale = initObj.x_scale
      }
      else {
        this.x_scale = 0.0;
      }
      if (initObj.hasOwnProperty('y_scale')) {
        this.y_scale = initObj.y_scale
      }
      else {
        this.y_scale = 0.0;
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
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
      }
      if (initObj.hasOwnProperty('target_msg_id')) {
        this.target_msg_id = initObj.target_msg_id
      }
      else {
        this.target_msg_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Communication
    // Serialize message field [sender_name]
    bufferOffset = _serializer.string(obj.sender_name, buffer, bufferOffset);
    // Serialize message field [msg_id]
    bufferOffset = _serializer.int32(obj.msg_id, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.string(obj.type, buffer, bufferOffset);
    // Serialize message field [formation]
    bufferOffset = _serializer.string(obj.formation, buffer, bufferOffset);
    // Serialize message field [receiver_name]
    bufferOffset = _serializer.string(obj.receiver_name, buffer, bufferOffset);
    // Serialize message field [leader_name]
    bufferOffset = _serializer.string(obj.leader_name, buffer, bufferOffset);
    // Serialize message field [follower1_name]
    bufferOffset = _serializer.string(obj.follower1_name, buffer, bufferOffset);
    // Serialize message field [follower2_name]
    bufferOffset = _serializer.string(obj.follower2_name, buffer, bufferOffset);
    // Serialize message field [follower3_name]
    bufferOffset = _serializer.string(obj.follower3_name, buffer, bufferOffset);
    // Serialize message field [objective_name]
    bufferOffset = _serializer.string(obj.objective_name, buffer, bufferOffset);
    // Serialize message field [desired_speed]
    bufferOffset = _serializer.float64(obj.desired_speed, buffer, bufferOffset);
    // Serialize message field [priority_type]
    bufferOffset = _serializer.string(obj.priority_type, buffer, bufferOffset);
    // Serialize message field [termination_method]
    bufferOffset = _serializer.string(obj.termination_method, buffer, bufferOffset);
    // Serialize message field [x_scale]
    bufferOffset = _serializer.float64(obj.x_scale, buffer, bufferOffset);
    // Serialize message field [y_scale]
    bufferOffset = _serializer.float64(obj.y_scale, buffer, bufferOffset);
    // Serialize message field [x_offset]
    bufferOffset = _serializer.float64(obj.x_offset, buffer, bufferOffset);
    // Serialize message field [y_offset]
    bufferOffset = _serializer.float64(obj.y_offset, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float64(obj.distance, buffer, bufferOffset);
    // Serialize message field [target_msg_id]
    bufferOffset = _serializer.int32(obj.target_msg_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Communication
    let len;
    let data = new Communication(null);
    // Deserialize message field [sender_name]
    data.sender_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [msg_id]
    data.msg_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [formation]
    data.formation = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [receiver_name]
    data.receiver_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [leader_name]
    data.leader_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [follower1_name]
    data.follower1_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [follower2_name]
    data.follower2_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [follower3_name]
    data.follower3_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [objective_name]
    data.objective_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [desired_speed]
    data.desired_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [priority_type]
    data.priority_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [termination_method]
    data.termination_method = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [x_scale]
    data.x_scale = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_scale]
    data.y_scale = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x_offset]
    data.x_offset = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_offset]
    data.y_offset = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [target_msg_id]
    data.target_msg_id = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.sender_name);
    length += _getByteLength(object.type);
    length += _getByteLength(object.formation);
    length += _getByteLength(object.receiver_name);
    length += _getByteLength(object.leader_name);
    length += _getByteLength(object.follower1_name);
    length += _getByteLength(object.follower2_name);
    length += _getByteLength(object.follower3_name);
    length += _getByteLength(object.objective_name);
    length += _getByteLength(object.priority_type);
    length += _getByteLength(object.termination_method);
    return length + 100;
  }

  static datatype() {
    // Returns string type for a message object
    return 'avt_341_msgs/Communication';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b7d0b7fede5233ad75ab393a95ddc673';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string sender_name
    int32 msg_id
    string type
    string formation
    string receiver_name
    string leader_name
    string follower1_name
    string follower2_name
    string follower3_name
    string objective_name
    float64 desired_speed
    string priority_type
    string termination_method
    float64 x_scale
    float64 y_scale
    float64 x_offset
    float64 y_offset
    float64 distance
    int32 target_msg_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Communication(null);
    if (msg.sender_name !== undefined) {
      resolved.sender_name = msg.sender_name;
    }
    else {
      resolved.sender_name = ''
    }

    if (msg.msg_id !== undefined) {
      resolved.msg_id = msg.msg_id;
    }
    else {
      resolved.msg_id = 0
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = ''
    }

    if (msg.formation !== undefined) {
      resolved.formation = msg.formation;
    }
    else {
      resolved.formation = ''
    }

    if (msg.receiver_name !== undefined) {
      resolved.receiver_name = msg.receiver_name;
    }
    else {
      resolved.receiver_name = ''
    }

    if (msg.leader_name !== undefined) {
      resolved.leader_name = msg.leader_name;
    }
    else {
      resolved.leader_name = ''
    }

    if (msg.follower1_name !== undefined) {
      resolved.follower1_name = msg.follower1_name;
    }
    else {
      resolved.follower1_name = ''
    }

    if (msg.follower2_name !== undefined) {
      resolved.follower2_name = msg.follower2_name;
    }
    else {
      resolved.follower2_name = ''
    }

    if (msg.follower3_name !== undefined) {
      resolved.follower3_name = msg.follower3_name;
    }
    else {
      resolved.follower3_name = ''
    }

    if (msg.objective_name !== undefined) {
      resolved.objective_name = msg.objective_name;
    }
    else {
      resolved.objective_name = ''
    }

    if (msg.desired_speed !== undefined) {
      resolved.desired_speed = msg.desired_speed;
    }
    else {
      resolved.desired_speed = 0.0
    }

    if (msg.priority_type !== undefined) {
      resolved.priority_type = msg.priority_type;
    }
    else {
      resolved.priority_type = ''
    }

    if (msg.termination_method !== undefined) {
      resolved.termination_method = msg.termination_method;
    }
    else {
      resolved.termination_method = ''
    }

    if (msg.x_scale !== undefined) {
      resolved.x_scale = msg.x_scale;
    }
    else {
      resolved.x_scale = 0.0
    }

    if (msg.y_scale !== undefined) {
      resolved.y_scale = msg.y_scale;
    }
    else {
      resolved.y_scale = 0.0
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

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
    }

    if (msg.target_msg_id !== undefined) {
      resolved.target_msg_id = msg.target_msg_id;
    }
    else {
      resolved.target_msg_id = 0
    }

    return resolved;
    }
};

module.exports = Communication;
