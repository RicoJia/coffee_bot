// Auto-generated. Do not edit!

// (in-package nuturtlebot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SensorData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.left_encoder = null;
      this.right_encoder = null;
    }
    else {
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('left_encoder')) {
        this.left_encoder = initObj.left_encoder
      }
      else {
        this.left_encoder = 0.0;
      }
      if (initObj.hasOwnProperty('right_encoder')) {
        this.right_encoder = initObj.right_encoder
      }
      else {
        this.right_encoder = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SensorData
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [left_encoder]
    bufferOffset = _serializer.float32(obj.left_encoder, buffer, bufferOffset);
    // Serialize message field [right_encoder]
    bufferOffset = _serializer.float32(obj.right_encoder, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SensorData
    let len;
    let data = new SensorData(null);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [left_encoder]
    data.left_encoder = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_encoder]
    data.right_encoder = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'nuturtlebot/SensorData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5e11846fa8bcf550efab217e45ea9e19';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time stamp
    float32 left_encoder
    float32 right_encoder
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SensorData(null);
    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    if (msg.left_encoder !== undefined) {
      resolved.left_encoder = msg.left_encoder;
    }
    else {
      resolved.left_encoder = 0.0
    }

    if (msg.right_encoder !== undefined) {
      resolved.right_encoder = msg.right_encoder;
    }
    else {
      resolved.right_encoder = 0.0
    }

    return resolved;
    }
};

module.exports = SensorData;
