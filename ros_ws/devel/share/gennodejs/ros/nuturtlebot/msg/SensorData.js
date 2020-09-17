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
      this.accelX = null;
      this.accelY = null;
      this.accelZ = null;
      this.gyroX = null;
      this.gyroY = null;
      this.gyroZ = null;
      this.magX = null;
      this.magY = null;
      this.magZ = null;
      this.battery_voltage = null;
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
        this.left_encoder = 0;
      }
      if (initObj.hasOwnProperty('right_encoder')) {
        this.right_encoder = initObj.right_encoder
      }
      else {
        this.right_encoder = 0;
      }
      if (initObj.hasOwnProperty('accelX')) {
        this.accelX = initObj.accelX
      }
      else {
        this.accelX = 0;
      }
      if (initObj.hasOwnProperty('accelY')) {
        this.accelY = initObj.accelY
      }
      else {
        this.accelY = 0;
      }
      if (initObj.hasOwnProperty('accelZ')) {
        this.accelZ = initObj.accelZ
      }
      else {
        this.accelZ = 0;
      }
      if (initObj.hasOwnProperty('gyroX')) {
        this.gyroX = initObj.gyroX
      }
      else {
        this.gyroX = 0;
      }
      if (initObj.hasOwnProperty('gyroY')) {
        this.gyroY = initObj.gyroY
      }
      else {
        this.gyroY = 0;
      }
      if (initObj.hasOwnProperty('gyroZ')) {
        this.gyroZ = initObj.gyroZ
      }
      else {
        this.gyroZ = 0;
      }
      if (initObj.hasOwnProperty('magX')) {
        this.magX = initObj.magX
      }
      else {
        this.magX = 0;
      }
      if (initObj.hasOwnProperty('magY')) {
        this.magY = initObj.magY
      }
      else {
        this.magY = 0;
      }
      if (initObj.hasOwnProperty('magZ')) {
        this.magZ = initObj.magZ
      }
      else {
        this.magZ = 0;
      }
      if (initObj.hasOwnProperty('battery_voltage')) {
        this.battery_voltage = initObj.battery_voltage
      }
      else {
        this.battery_voltage = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SensorData
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [left_encoder]
    bufferOffset = _serializer.int32(obj.left_encoder, buffer, bufferOffset);
    // Serialize message field [right_encoder]
    bufferOffset = _serializer.int32(obj.right_encoder, buffer, bufferOffset);
    // Serialize message field [accelX]
    bufferOffset = _serializer.int16(obj.accelX, buffer, bufferOffset);
    // Serialize message field [accelY]
    bufferOffset = _serializer.int16(obj.accelY, buffer, bufferOffset);
    // Serialize message field [accelZ]
    bufferOffset = _serializer.int16(obj.accelZ, buffer, bufferOffset);
    // Serialize message field [gyroX]
    bufferOffset = _serializer.int16(obj.gyroX, buffer, bufferOffset);
    // Serialize message field [gyroY]
    bufferOffset = _serializer.int16(obj.gyroY, buffer, bufferOffset);
    // Serialize message field [gyroZ]
    bufferOffset = _serializer.int16(obj.gyroZ, buffer, bufferOffset);
    // Serialize message field [magX]
    bufferOffset = _serializer.int16(obj.magX, buffer, bufferOffset);
    // Serialize message field [magY]
    bufferOffset = _serializer.int16(obj.magY, buffer, bufferOffset);
    // Serialize message field [magZ]
    bufferOffset = _serializer.int16(obj.magZ, buffer, bufferOffset);
    // Serialize message field [battery_voltage]
    bufferOffset = _serializer.float32(obj.battery_voltage, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SensorData
    let len;
    let data = new SensorData(null);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [left_encoder]
    data.left_encoder = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [right_encoder]
    data.right_encoder = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [accelX]
    data.accelX = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [accelY]
    data.accelY = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [accelZ]
    data.accelZ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [gyroX]
    data.gyroX = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [gyroY]
    data.gyroY = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [gyroZ]
    data.gyroZ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [magX]
    data.magX = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [magY]
    data.magY = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [magZ]
    data.magZ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [battery_voltage]
    data.battery_voltage = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 38;
  }

  static datatype() {
    // Returns string type for a message object
    return 'nuturtlebot/SensorData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e5bb303db7aaeaf900294f1b85ecc818';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time stamp
    int32 left_encoder
    int32 right_encoder
    int16 accelX
    int16 accelY
    int16 accelZ
    int16 gyroX
    int16 gyroY
    int16 gyroZ
    int16 magX
    int16 magY
    int16 magZ
    float32 battery_voltage
    
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
      resolved.left_encoder = 0
    }

    if (msg.right_encoder !== undefined) {
      resolved.right_encoder = msg.right_encoder;
    }
    else {
      resolved.right_encoder = 0
    }

    if (msg.accelX !== undefined) {
      resolved.accelX = msg.accelX;
    }
    else {
      resolved.accelX = 0
    }

    if (msg.accelY !== undefined) {
      resolved.accelY = msg.accelY;
    }
    else {
      resolved.accelY = 0
    }

    if (msg.accelZ !== undefined) {
      resolved.accelZ = msg.accelZ;
    }
    else {
      resolved.accelZ = 0
    }

    if (msg.gyroX !== undefined) {
      resolved.gyroX = msg.gyroX;
    }
    else {
      resolved.gyroX = 0
    }

    if (msg.gyroY !== undefined) {
      resolved.gyroY = msg.gyroY;
    }
    else {
      resolved.gyroY = 0
    }

    if (msg.gyroZ !== undefined) {
      resolved.gyroZ = msg.gyroZ;
    }
    else {
      resolved.gyroZ = 0
    }

    if (msg.magX !== undefined) {
      resolved.magX = msg.magX;
    }
    else {
      resolved.magX = 0
    }

    if (msg.magY !== undefined) {
      resolved.magY = msg.magY;
    }
    else {
      resolved.magY = 0
    }

    if (msg.magZ !== undefined) {
      resolved.magZ = msg.magZ;
    }
    else {
      resolved.magZ = 0
    }

    if (msg.battery_voltage !== undefined) {
      resolved.battery_voltage = msg.battery_voltage;
    }
    else {
      resolved.battery_voltage = 0.0
    }

    return resolved;
    }
};

module.exports = SensorData;
