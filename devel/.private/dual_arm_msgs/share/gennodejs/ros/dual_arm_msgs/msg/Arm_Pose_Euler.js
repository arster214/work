// Auto-generated. Do not edit!

// (in-package dual_arm_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Arm_Pose_Euler {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Px = null;
      this.Py = null;
      this.Pz = null;
      this.Rx = null;
      this.Ry = null;
      this.Rz = null;
    }
    else {
      if (initObj.hasOwnProperty('Px')) {
        this.Px = initObj.Px
      }
      else {
        this.Px = 0.0;
      }
      if (initObj.hasOwnProperty('Py')) {
        this.Py = initObj.Py
      }
      else {
        this.Py = 0.0;
      }
      if (initObj.hasOwnProperty('Pz')) {
        this.Pz = initObj.Pz
      }
      else {
        this.Pz = 0.0;
      }
      if (initObj.hasOwnProperty('Rx')) {
        this.Rx = initObj.Rx
      }
      else {
        this.Rx = 0.0;
      }
      if (initObj.hasOwnProperty('Ry')) {
        this.Ry = initObj.Ry
      }
      else {
        this.Ry = 0.0;
      }
      if (initObj.hasOwnProperty('Rz')) {
        this.Rz = initObj.Rz
      }
      else {
        this.Rz = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Arm_Pose_Euler
    // Serialize message field [Px]
    bufferOffset = _serializer.float32(obj.Px, buffer, bufferOffset);
    // Serialize message field [Py]
    bufferOffset = _serializer.float32(obj.Py, buffer, bufferOffset);
    // Serialize message field [Pz]
    bufferOffset = _serializer.float32(obj.Pz, buffer, bufferOffset);
    // Serialize message field [Rx]
    bufferOffset = _serializer.float32(obj.Rx, buffer, bufferOffset);
    // Serialize message field [Ry]
    bufferOffset = _serializer.float32(obj.Ry, buffer, bufferOffset);
    // Serialize message field [Rz]
    bufferOffset = _serializer.float32(obj.Rz, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Arm_Pose_Euler
    let len;
    let data = new Arm_Pose_Euler(null);
    // Deserialize message field [Px]
    data.Px = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Py]
    data.Py = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Pz]
    data.Pz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Rx]
    data.Rx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Ry]
    data.Ry = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Rz]
    data.Rz = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dual_arm_msgs/Arm_Pose_Euler';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '14cf69e90ba61a0c7ae2933996244f58';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 Px
    float32 Py
    float32 Pz
    float32 Rx
    float32 Ry
    float32 Rz
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Arm_Pose_Euler(null);
    if (msg.Px !== undefined) {
      resolved.Px = msg.Px;
    }
    else {
      resolved.Px = 0.0
    }

    if (msg.Py !== undefined) {
      resolved.Py = msg.Py;
    }
    else {
      resolved.Py = 0.0
    }

    if (msg.Pz !== undefined) {
      resolved.Pz = msg.Pz;
    }
    else {
      resolved.Pz = 0.0
    }

    if (msg.Rx !== undefined) {
      resolved.Rx = msg.Rx;
    }
    else {
      resolved.Rx = 0.0
    }

    if (msg.Ry !== undefined) {
      resolved.Ry = msg.Ry;
    }
    else {
      resolved.Ry = 0.0
    }

    if (msg.Rz !== undefined) {
      resolved.Rz = msg.Rz;
    }
    else {
      resolved.Rz = 0.0
    }

    return resolved;
    }
};

module.exports = Arm_Pose_Euler;
