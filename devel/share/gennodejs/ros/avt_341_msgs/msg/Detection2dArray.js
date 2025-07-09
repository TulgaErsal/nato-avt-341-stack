// Auto-generated. Do not edit!

// (in-package avt_341_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Detection2d = require('./Detection2d.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Detection2dArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.detections = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('detections')) {
        this.detections = initObj.detections
      }
      else {
        this.detections = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Detection2dArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [detections]
    // Serialize the length for message field [detections]
    bufferOffset = _serializer.uint32(obj.detections.length, buffer, bufferOffset);
    obj.detections.forEach((val) => {
      bufferOffset = Detection2d.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Detection2dArray
    let len;
    let data = new Detection2dArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [detections]
    // Deserialize array length for message field [detections]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.detections = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.detections[i] = Detection2d.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.detections.forEach((val) => {
      length += Detection2d.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'avt_341_msgs/Detection2dArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4bd5549dd2967ed5228b993c3abd5a0f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message represents the results of an object detector for a given image
    # frame.
    
    # Header stamped at the completion time of the detection.
    std_msgs/Header header
    
    # Collection of detections for the given image frame, in the order specified by
    # the image detector.
    Detection2d[] detections
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
    MSG: avt_341_msgs/Detection2d
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
    const resolved = new Detection2dArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.detections !== undefined) {
      resolved.detections = new Array(msg.detections.length);
      for (let i = 0; i < resolved.detections.length; ++i) {
        resolved.detections[i] = Detection2d.Resolve(msg.detections[i]);
      }
    }
    else {
      resolved.detections = []
    }

    return resolved;
    }
};

module.exports = Detection2dArray;
