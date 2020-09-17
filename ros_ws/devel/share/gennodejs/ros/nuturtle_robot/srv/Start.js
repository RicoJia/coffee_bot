// Auto-generated. Do not edit!

// (in-package nuturtle_robot.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class StartRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ccw_or_forward = null;
    }
    else {
      if (initObj.hasOwnProperty('ccw_or_forward')) {
        this.ccw_or_forward = initObj.ccw_or_forward
      }
      else {
        this.ccw_or_forward = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartRequest
    // Serialize message field [ccw_or_forward]
    bufferOffset = _serializer.bool(obj.ccw_or_forward, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartRequest
    let len;
    let data = new StartRequest(null);
    // Deserialize message field [ccw_or_forward]
    data.ccw_or_forward = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'nuturtle_robot/StartRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e3cef58e896b7b1fe8be0821b7a77707';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ccw_or_forward
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartRequest(null);
    if (msg.ccw_or_forward !== undefined) {
      resolved.ccw_or_forward = msg.ccw_or_forward;
    }
    else {
      resolved.ccw_or_forward = false
    }

    return resolved;
    }
};

class StartResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartResponse
    let len;
    let data = new StartResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'nuturtle_robot/StartResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: StartRequest,
  Response: StartResponse,
  md5sum() { return 'e3cef58e896b7b1fe8be0821b7a77707'; },
  datatype() { return 'nuturtle_robot/Start'; }
};
