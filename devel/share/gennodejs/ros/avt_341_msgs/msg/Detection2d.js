// Auto-generated. Do not edit!

// (in-package avt_341_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BoundingBox2d = require('./BoundingBox2d.js');
let Hypothesis = require('./Hypothesis.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Detection2d {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.bounding_box = null;
      this.hypothesis = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('bounding_box')) {
        this.bounding_box = initObj.bounding_box
      }
      else {
        this.bounding_box = new BoundingBox2d();
      }
      if (initObj.hasOwnProperty('hypothesis')) {
        this.hypothesis = initObj.hypothesis
      }
      else {
        this.hypothesis = new Hypothesis();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Detection2d
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [bounding_box]
    bufferOffset = BoundingBox2d.serialize(obj.bounding_box, buffer, bufferOffset);
    // Serialize message field [hypothesis]
    bufferOffset = Hypothesis.serialize(obj.hypothesis, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Detection2d
    let len;
    let data = new Detection2d(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [bounding_box]
    data.bounding_box = BoundingBox2d.deserialize(buffer, bufferOffset);
    // Deserialize message field [hypothesis]
    data.hypothesis = Hypothesis.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += Hypothesis.getMessageSize(object.hypothesis);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'avt_341_msgs/Detection2d';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aa9743fbf372d547819db142e9d8db87';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message represents a single detection associated with an image based on
    # the best scoring hypothesis after intersection-over-union (IoU) filtering.
    #
    # This message is not stamped, as it is meant to be part of a message including
    # all detections provided by an object detector for a given image.
    
    # Message header.
    std_msgs/Header header
    
    # Detection bounding box.
    BoundingBox2d bounding_box
    
    # Detection hypothesis.
    Hypothesis hypothesis
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
    MSG: avt_341_msgs/BoundingBox2d
    # This message represents a two-dimensional bounding box enclosing a region of
    # the image frame where an object detector has formulated a hypothesis. The
    # bounding box is aligned to image frame and may not be rotated, hence it is
    # uniquely defined by its four corners.
    #
    # This message is not stamped, as it is meant to be part of a message including
    # all detections provided by an object detector for a given image.
    
    # Minimum x coordinate of the bounding box in pixels.
    int32 x_min
    
    # Maximum x coordinate of the bounding box in pixels.
    int32 x_max
    
    # Minimum y coordinate of the bounding box in pixels.
    int32 y_min
    
    # Maximum y coordinate of the bounding box in pixels.
    int32 y_max
    ================================================================================
    MSG: avt_341_msgs/Hypothesis
    # This message represents an object hypothesis formulated by an object detector
    # for a given region of the image.
    
    # The unique identifier of the object class as specified by the object detector.
    int16 id
    
    # The human-readable label of the object class as specified by the object
    # detector.
    string label
    
    # The score associated with the hypothesis, as a floating point number between
    # 0.0 and 1.0, including the limits.
    float64 score
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Detection2d(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.bounding_box !== undefined) {
      resolved.bounding_box = BoundingBox2d.Resolve(msg.bounding_box)
    }
    else {
      resolved.bounding_box = new BoundingBox2d()
    }

    if (msg.hypothesis !== undefined) {
      resolved.hypothesis = Hypothesis.Resolve(msg.hypothesis)
    }
    else {
      resolved.hypothesis = new Hypothesis()
    }

    return resolved;
    }
};

module.exports = Detection2d;
