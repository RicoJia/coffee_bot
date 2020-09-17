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

class WheelCommands {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left_velocity = null;
      this.right_velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('left_velocity')) {
        this.left_velocity = initObj.left_velocity
      }
      else {
        this.left_velocity = 0;
      }
      if (initObj.hasOwnProperty('right_velocity')) {
        this.right_velocity = initObj.right_velocity
      }
      else {
        this.right_velocity = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WheelCommands
    // Serialize message field [left_velocity]
    bufferOffset = _serializer.int32(obj.left_velocity, buffer, bufferOffset);
    // Serialize message field [right_velocity]
    bufferOffset = _serializer.int32(obj.right_velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WheelCommands
    let len;
    let data = new WheelCommands(null);
    // Deserialize message field [left_velocity]
    data.left_velocity = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [right_velocity]
    data.right_velocity = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'nuturtlebot/WheelCommands';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '55f26e03298a44797b568254274af487';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 left_velocity
    int32 right_velocity
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WheelCommands(null);
    if (msg.left_velocity !== undefined) {
      resolved.left_velocity = msg.left_velocity;
    }
    else {
      resolved.left_velocity = 0
    }

    if (msg.right_velocity !== undefined) {
      resolved.right_velocity = msg.right_velocity;
    }
    else {
      resolved.right_velocity = 0
    }

    return resolved;
    }
};

module.exports = WheelCommands;
